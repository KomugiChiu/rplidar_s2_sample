package main

import (
	"bufio"
	"encoding/json"
	"errors"
	"fmt"
	"io"
	"os"
	"os/exec"
	"path/filepath"
	"sync"
	"syscall"
	"time"
)

// Whitelisted stack scripts (relative to -stack-dir).  Nothing else can be run.
var stackScripts = map[string]string{
	"start_all": "start_all.sh",
	"start_all_amcl": "start_all_amcl.sh",
	"stop_all":  "stop_all.sh",
	"stop_all_amcl":  "stop_all_amcl.sh",
	"start_ai":  "start_ai.sh",
	"stop_ai":   "stop_ai.sh",
}

const scriptTimeout = 4 * time.Minute

type ScriptState struct {
	Name    string   `json:"name"`
	Running bool     `json:"running"`
	By      string   `json:"by"`
	Started float64  `json:"started"`
	Ended   float64  `json:"ended,omitempty"`
	Exit    *int     `json:"exit,omitempty"`
	Lines   []string `json:"lines"`
}

type ScriptRunner struct {
	dir string
	hub *Hub
	mu  sync.Mutex
	st  ScriptState
}

func NewScriptRunner(dir string, hub *Hub) *ScriptRunner {
	r := &ScriptRunner{dir: dir, hub: hub}
	r.publish()
	return r
}

func (r *ScriptRunner) Available() map[string]bool {
	out := map[string]bool{}
	for k, f := range stackScripts {
		_, err := os.Stat(filepath.Join(r.dir, f))
		out[k] = err == nil
	}
	return out
}

func (r *ScriptRunner) publish() {
	r.mu.Lock()
	st := r.st
	st.Lines = append([]string(nil), r.st.Lines...)
	r.mu.Unlock()
	b, _ := json.Marshal(map[string]any{"state": st, "available": r.Available(), "dir": r.dir})
	r.hub.Set("script", b)
}

func (r *ScriptRunner) Start(name, by string) error {
	file, ok := stackScripts[name]
	if !ok {
		return errors.New("unknown script")
	}
	path := filepath.Join(r.dir, file)
	if _, err := os.Stat(path); err != nil {
		return fmt.Errorf("%s not found", path)
	}
	r.mu.Lock()
	if r.st.Running {
		r.mu.Unlock()
		return fmt.Errorf("%s is still running", r.st.Name)
	}
	r.st = ScriptState{Name: name, Running: true, By: by, Started: nowSec(),
		Lines: []string{"$ " + path}}
	r.mu.Unlock()

	cmd := exec.Command("bash", path)
	cmd.Dir = r.dir
	cmd.Env = append(os.Environ(), "ROS_DOMAIN_ID="+envOr("ROS_DOMAIN_ID", "99"))
	cmd.SysProcAttr = &syscall.SysProcAttr{Setsid: true}
	// the scripts background their nodes with setsid + log redirection; do
	// not wait forever if one of them still holds the pipe
	cmd.WaitDelay = 3 * time.Second
	pr, pw := io.Pipe()
	cmd.Stdout, cmd.Stderr = pw, pw
	if err := cmd.Start(); err != nil {
		r.finish(-1, err.Error())
		return err
	}
	emitEvent(r.hub, "CONSOLE", "WARN", fmt.Sprintf("%s started from %s", file, by))
	r.publish()

	go func() {
		sc := bufio.NewScanner(pr)
		sc.Buffer(make([]byte, 64*1024), 1024*1024)
		last := time.Now()
		for sc.Scan() {
			r.mu.Lock()
			r.st.Lines = append(r.st.Lines, ansiRe.ReplaceAllString(sc.Text(), ""))
			if len(r.st.Lines) > 400 {
				r.st.Lines = r.st.Lines[len(r.st.Lines)-400:]
			}
			r.mu.Unlock()
			if time.Since(last) > 250*time.Millisecond {
				last = time.Now()
				r.publish()
			}
		}
	}()
	go func() {
		timer := time.AfterFunc(scriptTimeout, func() {
			_ = syscall.Kill(-cmd.Process.Pid, syscall.SIGTERM)
		})
		err := cmd.Wait()
		timer.Stop()
		pw.Close()
		code := 0
		msg := ""
		if err != nil {
			code = -1
			var ee *exec.ExitError
			if errors.As(err, &ee) {
				code = ee.ExitCode()
			}
			msg = err.Error()
		}
		r.finish(code, msg)
	}()
	return nil
}

func (r *ScriptRunner) finish(code int, msg string) {
	r.mu.Lock()
	r.st.Running = false
	r.st.Ended = nowSec()
	r.st.Exit = &code
	if msg != "" {
		r.st.Lines = append(r.st.Lines, "! "+msg)
	}
	name := r.st.Name
	r.mu.Unlock()
	lvl := "INFO"
	if code != 0 {
		lvl = "ERROR"
	}
	emitEvent(r.hub, "CONSOLE", lvl, fmt.Sprintf("%s finished (exit %d)", stackScripts[name], code))
	r.publish()
}

func envOr(k, def string) string {
	if v := os.Getenv(k); v != "" {
		return v
	}
	return def
}
