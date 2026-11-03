// robot-console: realtime web console for the person-follow robot.
//
//	browser ── HTTP :8080 ──► robot-console (this binary, Go stdlib only)
//	                            ├─ /proc system + process monitor, log file tailer
//	                            ├─ whitelisted stack scripts (start_all.sh …)
//	                            └─ stdio ──► robot_console_gateway.py (rclpy)
//
// Server→browser data uses Server-Sent Events (one stream per tab, channels
// chosen with /api/subscribe), camera views use MJPEG, commands are POSTs.
package main

import (
	"bytes"
	"context"
	"embed"
	"encoding/json"
	"flag"
	"fmt"
	"io"
	"io/fs"
	"log"
	"net"
	"net/http"
	"os"
	"os/signal"
	"path/filepath"
	"strconv"
	"strings"
	"syscall"
	"time"
)

//go:embed web
var webFS embed.FS

type Server struct {
	hub     *Hub
	gw      *Gateway
	logs    *LogStore
	cams    *Cams
	scripts *ScriptRunner
}

func main() {
	exe, _ := os.Executable()
	base := filepath.Dir(exe)
	addr := flag.String("addr", ":8080", "listen address")
	gwScript := flag.String("gateway", filepath.Join(base, "..", "gateway", "run_gateway.sh"), "ROS gateway launcher")
	stackDir := flag.String("stack-dir", "/nvt_ros2_example/slam_toolbox_rf2o", "directory with start_all.sh / stop_all.sh / start_ai.sh / stop_ai.sh")
	webDir := flag.String("web", "", "serve the UI from this directory instead of the embedded copy (UI development)")
	flag.Parse()

	ctx, cancel := signal.NotifyContext(context.Background(), syscall.SIGINT, syscall.SIGTERM)
	defer cancel()

	hub := NewHub()
	logs := NewLogStore(hub)
	cams := NewCams(hub)
	gw := NewGateway(*gwScript, hub, logs, cams)
	scripts := NewScriptRunner(*stackDir, hub)
	sys := NewSysMon(hub, gw, scripts)
	s := &Server{hub: hub, gw: gw, logs: logs, cams: cams, scripts: scripts}

	go gw.Run(ctx)
	go logs.TailFiles(ctx.Done())
	go sys.Run(ctx.Done())
	emitEvent(hub, "CONSOLE", "INFO", "robot-console started")

	mux := http.NewServeMux()
	mux.HandleFunc("GET /api/events", s.handleEvents)
	mux.HandleFunc("POST /api/subscribe", s.handleSubscribe)
	mux.HandleFunc("GET /api/camera", s.handleCamera)
	mux.HandleFunc("POST /api/cmd", s.handleCmd)
	mux.HandleFunc("POST /api/teleop", s.handleTeleop)
	mux.HandleFunc("POST /api/stack/{name}", s.handleStack)
	mux.HandleFunc("GET /api/logs.txt", s.handleLogDump)
	mux.HandleFunc("GET /api/ping", func(w http.ResponseWriter, r *http.Request) {
		writeJSON(w, 200, map[string]any{"t": nowSec()})
	})
	var static http.Handler
	if *webDir != "" {
		static = http.FileServer(http.Dir(*webDir))
	} else {
		sub, _ := fs.Sub(webFS, "web")
		static = http.FileServer(http.FS(sub))
	}
	mux.Handle("GET /", http.HandlerFunc(func(w http.ResponseWriter, r *http.Request) {
		w.Header().Set("Cache-Control", "no-cache")
		static.ServeHTTP(w, r)
	}))

	srv := &http.Server{Addr: *addr, Handler: mux, ReadHeaderTimeout: 10 * time.Second}
	go func() {
		<-ctx.Done()
		sctx, c := context.WithTimeout(context.Background(), 3*time.Second)
		defer c()
		_ = srv.Shutdown(sctx)
	}()
	log.Printf("robot-console listening on %s (gateway %s, stack %s)", *addr, *gwScript, *stackDir)
	if err := srv.ListenAndServe(); err != nil && err != http.ErrServerClosed {
		log.Fatal(err)
	}
	time.Sleep(500 * time.Millisecond) // let the gateway get its SIGTERM
}

func clientIP(r *http.Request) string {
	host, _, err := net.SplitHostPort(r.RemoteAddr)
	if err != nil {
		return r.RemoteAddr
	}
	return host
}

func writeJSON(w http.ResponseWriter, code int, v any) {
	w.Header().Set("Content-Type", "application/json")
	w.WriteHeader(code)
	_ = json.NewEncoder(w).Encode(v)
}

func splitList(s string) []string {
	var out []string
	for _, x := range strings.Split(s, ",") {
		if x = strings.TrimSpace(x); x != "" {
			out = append(out, x)
		}
	}
	return out
}

// GET /api/events?ch=a,b — SSE stream; the first message carries the session
// id used by /api/subscribe to change channels without reconnecting.
func (s *Server) handleEvents(w http.ResponseWriter, r *http.Request) {
	fl, ok := w.(http.Flusher)
	if !ok {
		http.Error(w, "streaming unsupported", 500)
		return
	}
	c := s.hub.AddClient(clientIP(r), splitList(r.URL.Query().Get("ch")))
	defer s.hub.RemoveClient(c)
	h := w.Header()
	h.Set("Content-Type", "text/event-stream")
	h.Set("Cache-Control", "no-cache")
	h.Set("X-Accel-Buffering", "no")
	fmt.Fprintf(w, "retry: 2000\ndata: {\"ch\":\"hello\",\"d\":{\"sid\":%q,\"ip\":%q}}\n\n", c.id, c.addr)
	fl.Flush()
	ping := time.NewTicker(15 * time.Second)
	defer ping.Stop()
	var buf bytes.Buffer
	for {
		select {
		case <-r.Context().Done():
			return
		case <-ping.C:
			if _, err := io.WriteString(w, ": ping\n\n"); err != nil {
				return
			}
			fl.Flush()
		case <-c.notify:
			pending, queued := s.hub.Drain(c)
			buf.Reset()
			for ch, d := range pending {
				writeSSE(&buf, ch, d)
			}
			for ch, items := range queued {
				var b bytes.Buffer
				b.WriteString(`{"items":[`)
				for i, it := range items {
					if i > 0 {
						b.WriteByte(',')
					}
					b.Write(it)
				}
				b.WriteString(`]}`)
				writeSSE(&buf, ch, b.Bytes())
			}
			if buf.Len() == 0 {
				continue
			}
			if _, err := w.Write(buf.Bytes()); err != nil {
				return
			}
			fl.Flush()
		}
	}
}

func writeSSE(buf *bytes.Buffer, ch string, d []byte) {
	chj, _ := json.Marshal(ch)
	buf.WriteString(`data: {"ch":`)
	buf.Write(chj)
	buf.WriteString(`,"d":`)
	if len(d) == 0 {
		buf.WriteString("null")
	} else {
		buf.Write(d)
	}
	buf.WriteString("}\n\n")
}

func (s *Server) handleSubscribe(w http.ResponseWriter, r *http.Request) {
	var body struct {
		SID string   `json:"sid"`
		Ch  []string `json:"ch"`
	}
	if err := json.NewDecoder(io.LimitReader(r.Body, 64*1024)).Decode(&body); err != nil {
		writeJSON(w, 400, map[string]any{"ok": false, "msg": err.Error()})
		return
	}
	if !s.hub.Subscribe(body.SID, body.Ch) {
		writeJSON(w, 404, map[string]any{"ok": false, "msg": "unknown session"})
		return
	}
	writeJSON(w, 200, map[string]any{"ok": true})
}

// GET /api/camera?src=color|debug&fps=5 — MJPEG.
func (s *Server) handleCamera(w http.ResponseWriter, r *http.Request) {
	src := r.URL.Query().Get("src")
	if _, ok := map[string]bool{"color": true, "debug": true}[src]; !ok {
		http.Error(w, "src must be color or debug", 400)
		return
	}
	fps, _ := strconv.ParseFloat(r.URL.Query().Get("fps"), 64)
	if fps <= 0 || fps > 15 {
		fps = 5
	}
	fl, _ := w.(http.Flusher)
	sub := s.cams.Subscribe(src, fps)
	defer s.cams.Unsubscribe(src, sub)
	w.Header().Set("Content-Type", "multipart/x-mixed-replace; boundary=frame")
	w.Header().Set("Cache-Control", "no-cache")
	minGap := time.Duration(float64(time.Second) / fps * 0.9)
	var last time.Time
	for {
		select {
		case <-r.Context().Done():
			return
		case f := <-sub.ch:
			if time.Since(last) < minGap {
				continue
			}
			last = time.Now()
			if _, err := fmt.Fprintf(w, "--frame\r\nContent-Type: image/jpeg\r\nContent-Length: %d\r\n\r\n", len(f)); err != nil {
				return
			}
			if _, err := w.Write(f); err != nil {
				return
			}
			if _, err := io.WriteString(w, "\r\n"); err != nil {
				return
			}
			if fl != nil {
				fl.Flush()
			}
		}
	}
}

var cmdTimeouts = map[string]time.Duration{
	// goal/waypoints: the Python gateway creates its action client on first use
	"goal": 6 * time.Second, "waypoints": 6 * time.Second, "cancel": 3 * time.Second,
	"estop": 3 * time.Second, "initialpose": 3 * time.Second, "clear_costmaps": 3 * time.Second,
	"node_info": 5 * time.Second,
}

func (s *Server) handleCmd(w http.ResponseWriter, r *http.Request) {
	var body map[string]any
	if err := json.NewDecoder(io.LimitReader(r.Body, 1<<20)).Decode(&body); err != nil {
		writeJSON(w, 400, map[string]any{"ok": false, "msg": err.Error()})
		return
	}
	name, _ := body["cmd"].(string)
	timeout, ok := cmdTimeouts[name]
	if !ok {
		writeJSON(w, 400, map[string]any{"ok": false, "msg": "unknown command"})
		return
	}
	body["src"] = clientIP(r)
	rep, err := s.gw.Call(body, timeout)
	if err != nil {
		writeJSON(w, 503, map[string]any{"ok": false, "msg": err.Error()})
		return
	}
	writeJSON(w, 200, rep)
}

// POST /api/teleop {vx,vy,wz} — keepalive while a drive button is held.
func (s *Server) handleTeleop(w http.ResponseWriter, r *http.Request) {
	var v struct{ Vx, Vy, Wz float64 }
	if err := json.NewDecoder(io.LimitReader(r.Body, 4096)).Decode(&v); err != nil {
		writeJSON(w, 400, map[string]any{"ok": false, "msg": err.Error()})
		return
	}
	err := s.gw.Send(map[string]any{"cmd": "teleop", "vx": v.Vx, "vy": v.Vy, "wz": v.Wz, "src": clientIP(r)})
	if err != nil {
		writeJSON(w, 503, map[string]any{"ok": false, "msg": err.Error()})
		return
	}
	writeJSON(w, 200, map[string]any{"ok": true})
}

func (s *Server) handleStack(w http.ResponseWriter, r *http.Request) {
	if err := s.scripts.Start(r.PathValue("name"), clientIP(r)); err != nil {
		writeJSON(w, 409, map[string]any{"ok": false, "msg": err.Error()})
		return
	}
	writeJSON(w, 200, map[string]any{"ok": true})
}

func (s *Server) handleLogDump(w http.ResponseWriter, r *http.Request) {
	w.Header().Set("Content-Type", "text/plain; charset=utf-8")
	w.Header().Set("Content-Disposition",
		fmt.Sprintf("attachment; filename=robot-console-%s.log", time.Now().Format("20060102-150405")))
	s.logs.Dump(w)
}
