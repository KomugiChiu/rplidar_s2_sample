package main

import (
	"bufio"
	"context"
	"encoding/binary"
	"encoding/json"
	"errors"
	"fmt"
	"io"
	"log"
	"os/exec"
	"runtime"
	"strings"
	"sync"
	"sync/atomic"
	"syscall"
	"time"
)

var errGatewayDown = errors.New("ROS gateway is not running")

type Reply struct {
	ID   int64           `json:"id"`
	OK   bool            `json:"ok"`
	Msg  string          `json:"msg"`
	Data json.RawMessage `json:"data,omitempty"`
}

type GatewayStatus struct {
	Up       bool     `json:"up"`
	Since    float64  `json:"since"`
	Restarts int      `json:"restarts"`
	LastErr  string   `json:"last_err,omitempty"`
	Stderr   []string `json:"stderr"`
}

// Gateway supervises robot_console_gateway.py and speaks its stdio protocol.
type Gateway struct {
	script string
	hub    *Hub
	logs   *LogStore
	cams   *Cams

	mu       sync.Mutex
	stdin    io.WriteCloser
	up       bool
	since    time.Time
	restarts int
	lastErr  string
	stderr   []string
	waiters  map[int64]chan Reply
	nextID   atomic.Int64
}

func NewGateway(script string, hub *Hub, logs *LogStore, cams *Cams) *Gateway {
	g := &Gateway{script: script, hub: hub, logs: logs, cams: cams, waiters: map[int64]chan Reply{}}
	hub.OnDemand = func() { go g.sendDemand() }
	return g
}

func (g *Gateway) Status() GatewayStatus {
	g.mu.Lock()
	defer g.mu.Unlock()
	s := GatewayStatus{Up: g.up, Restarts: g.restarts, LastErr: g.lastErr,
		Stderr: append([]string(nil), g.stderr...)}
	if !g.since.IsZero() {
		s.Since = float64(g.since.UnixMilli()) / 1000
	}
	return s
}

// Run keeps the gateway alive until ctx is cancelled.
func (g *Gateway) Run(ctx context.Context) {
	// Pdeathsig is tied to the OS thread that started the child.
	runtime.LockOSThread()
	backoff := 2 * time.Second
	for ctx.Err() == nil {
		start := time.Now()
		err := g.runOnce(ctx)
		if ctx.Err() != nil {
			return
		}
		msg := "exited"
		if err != nil && !errors.Is(err, io.EOF) {
			msg = err.Error()
		}
		g.mu.Lock()
		g.restarts++
		g.lastErr = msg
		g.mu.Unlock()
		log.Printf("gateway %s; restarting in %s", msg, backoff)
		emitEvent(g.hub, "CONSOLE", "ERROR", "ROS gateway "+msg+"; restarting")
		if time.Since(start) > 30*time.Second {
			backoff = 2 * time.Second
		} else if backoff < 16*time.Second {
			backoff *= 2
		}
		select {
		case <-ctx.Done():
			return
		case <-time.After(backoff):
		}
	}
}

func (g *Gateway) runOnce(ctx context.Context) error {
	cmd := exec.Command("bash", g.script)
	cmd.SysProcAttr = &syscall.SysProcAttr{Setpgid: true, Pdeathsig: syscall.SIGTERM}
	stdin, err := cmd.StdinPipe()
	if err != nil {
		return err
	}
	stdout, err := cmd.StdoutPipe()
	if err != nil {
		return err
	}
	stderr, err := cmd.StderrPipe()
	if err != nil {
		return err
	}
	if err := cmd.Start(); err != nil {
		return err
	}
	pid := cmd.Process.Pid
	g.mu.Lock()
	g.stdin, g.up, g.since = stdin, true, time.Now()
	g.mu.Unlock()
	log.Printf("gateway started (pid %d)", pid)

	go g.readStderr(stderr)
	done := make(chan struct{})
	go func() {
		select {
		case <-ctx.Done():
			_ = syscall.Kill(-pid, syscall.SIGTERM)
		case <-done:
		}
	}()
	g.sendDemand()
	err = g.readFrames(stdout)
	close(done)
	_ = syscall.Kill(-pid, syscall.SIGTERM)
	waitErr := make(chan error, 1)
	go func() { waitErr <- cmd.Wait() }()
	select {
	case <-waitErr:
	case <-time.After(3 * time.Second):
		_ = syscall.Kill(-pid, syscall.SIGKILL)
		<-waitErr
	}
	g.mu.Lock()
	g.up, g.stdin = false, nil
	for id, ch := range g.waiters {
		ch <- Reply{ID: id, OK: false, Msg: errGatewayDown.Error()}
		delete(g.waiters, id)
	}
	g.mu.Unlock()
	return err
}

func (g *Gateway) readStderr(r io.Reader) {
	sc := bufio.NewScanner(r)
	sc.Buffer(make([]byte, 64*1024), 1024*1024)
	for sc.Scan() {
		line := sc.Text()
		log.Printf("gateway: %s", line)
		g.mu.Lock()
		g.stderr = append(g.stderr, line)
		if len(g.stderr) > 40 {
			g.stderr = g.stderr[len(g.stderr)-40:]
		}
		g.mu.Unlock()
		if strings.Contains(line, "Traceback") || strings.Contains(line, "Error") {
			g.logs.Add([]LogEntry{{T: nowSec(), Lvl: "E", Node: "robot_console", Msg: line, Src: "console"}})
		}
	}
}

type frameHdr struct {
	Ch string          `json:"ch"`
	D  json.RawMessage `json:"d"`
}

func (g *Gateway) readFrames(r io.Reader) error {
	br := bufio.NewReaderSize(r, 1<<20)
	var hdr [8]byte
	for {
		if _, err := io.ReadFull(br, hdr[:]); err != nil {
			return err
		}
		hl := binary.BigEndian.Uint32(hdr[:4])
		pl := binary.BigEndian.Uint32(hdr[4:])
		if hl > 64<<20 || pl > 64<<20 {
			return fmt.Errorf("protocol error (frame %d/%d bytes)", hl, pl)
		}
		hb := make([]byte, hl)
		if _, err := io.ReadFull(br, hb); err != nil {
			return err
		}
		var payload []byte
		if pl > 0 {
			payload = make([]byte, pl)
			if _, err := io.ReadFull(br, payload); err != nil {
				return err
			}
		}
		var f frameHdr
		if err := json.Unmarshal(hb, &f); err != nil {
			log.Printf("gateway: bad frame header: %v", err)
			continue
		}
		g.dispatch(f, payload)
	}
}

func (g *Gateway) dispatch(f frameHdr, payload []byte) {
	switch {
	case f.Ch == "reply":
		var r Reply
		if json.Unmarshal(f.D, &r) == nil {
			g.mu.Lock()
			ch := g.waiters[r.ID]
			delete(g.waiters, r.ID)
			g.mu.Unlock()
			if ch != nil {
				ch <- r
			}
		}
	case f.Ch == "log":
		var x struct {
			Items []LogEntry `json:"items"`
		}
		if json.Unmarshal(f.D, &x) == nil {
			g.logs.IngestROS(x.Items)
		}
	case f.Ch == "events":
		var x struct {
			Items []json.RawMessage `json:"items"`
		}
		if json.Unmarshal(f.D, &x) == nil {
			g.hub.Append("events", x.Items)
		}
	case strings.HasPrefix(f.Ch, "cam:"):
		g.cams.Publish(strings.TrimPrefix(f.Ch, "cam:"), payload)
	default:
		g.hub.Set(f.Ch, f.D)
	}
}

// Send writes a command without waiting for a reply.
func (g *Gateway) Send(cmd map[string]any) error {
	b, err := json.Marshal(cmd)
	if err != nil {
		return err
	}
	b = append(b, '\n')
	g.mu.Lock()
	defer g.mu.Unlock()
	if !g.up || g.stdin == nil {
		return errGatewayDown
	}
	_, err = g.stdin.Write(b)
	return err
}

// Call sends a command and waits for its reply.
func (g *Gateway) Call(cmd map[string]any, timeout time.Duration) (Reply, error) {
	id := g.nextID.Add(1)
	ch := make(chan Reply, 1)
	g.mu.Lock()
	g.waiters[id] = ch
	g.mu.Unlock()
	cmd["id"] = id
	if err := g.Send(cmd); err != nil {
		g.mu.Lock()
		delete(g.waiters, id)
		g.mu.Unlock()
		return Reply{}, err
	}
	select {
	case r := <-ch:
		return r, nil
	case <-time.After(timeout):
		g.mu.Lock()
		delete(g.waiters, id)
		g.mu.Unlock()
		return Reply{}, errors.New("gateway did not answer in time")
	}
}

func (g *Gateway) sendDemand() {
	_ = g.Send(map[string]any{"cmd": "demand", "ch": g.hub.Demand()})
}

// ---------------------------------------------------------------- cameras

type camSub struct {
	ch chan []byte
}

// Cams fans JPEG frames out to MJPEG viewers and tells the gateway which
// camera streams are wanted (and at what rate).
type Cams struct {
	hub  *Hub
	mu   sync.Mutex
	subs map[string]map[*camSub]float64
}

func NewCams(hub *Hub) *Cams {
	return &Cams{hub: hub, subs: map[string]map[*camSub]float64{}}
}

func (c *Cams) Subscribe(src string, fps float64) *camSub {
	s := &camSub{ch: make(chan []byte, 1)}
	c.mu.Lock()
	if c.subs[src] == nil {
		c.subs[src] = map[*camSub]float64{}
	}
	c.subs[src][s] = fps
	c.mu.Unlock()
	c.update(src)
	return s
}

func (c *Cams) Unsubscribe(src string, s *camSub) {
	c.mu.Lock()
	delete(c.subs[src], s)
	c.mu.Unlock()
	c.update(src)
}

func (c *Cams) update(src string) {
	c.mu.Lock()
	max := 0.0
	for _, f := range c.subs[src] {
		if f > max {
			max = f
		}
	}
	c.mu.Unlock()
	c.hub.SetExtraDemand("cam:"+src, max)
}

func (c *Cams) Publish(src string, frame []byte) {
	c.mu.Lock()
	defer c.mu.Unlock()
	for s := range c.subs[src] {
		select {
		case s.ch <- frame:
		default: // viewer is slow: replace the stale frame
			select {
			case <-s.ch:
			default:
			}
			select {
			case s.ch <- frame:
			default:
			}
		}
	}
}

func nowSec() float64 { return float64(time.Now().UnixMicro()) / 1e6 }

func emitEvent(hub *Hub, owner, level, text string) {
	b, _ := json.Marshal(map[string]any{"t": nowSec(), "owner": owner, "level": level, "text": text})
	hub.Append("events", []json.RawMessage{b})
}
