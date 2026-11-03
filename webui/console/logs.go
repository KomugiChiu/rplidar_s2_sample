package main

import (
	"encoding/json"
	"fmt"
	"io"
	"os"
	"path/filepath"
	"regexp"
	"sort"
	"strconv"
	"strings"
	"sync"
	"syscall"
	"time"
)

type LogEntry struct {
	T     float64 `json:"t"`
	Lvl   string  `json:"lvl"`
	Node  string  `json:"node"`
	Msg   string  `json:"msg"`
	Src   string  `json:"src"`
	Owner string  `json:"owner,omitempty"`
	Hint  string  `json:"hint,omitempty"`
}

// Log files written by the stack scripts; first matching glob sets the owner.
var logGlobs = []struct{ glob, owner string }{
	{"/tmp/lidar_rplidar.log", "LIDAR"},
	{"/tmp/lidar_rf2o.log", "ODOM"},
	{"/tmp/lidar_slam_toolbox.log", "SLAM"},
	{"/tmp/slam_toolbox.log", "SLAM"},
	{"/tmp/lidar_static_*.log", "TF"},
	{"/tmp/slam_toolbox_rf2o_ekf/logs/ekf.log", "EKF"},
	{"/tmp/slam_toolbox_rf2o_ekf/logs/realsense.log", "CAMERA"},
	{"/tmp/slam_toolbox_rf2o_ekf/logs/*.log", "TF"},
	{"/tmp/raspybot_nav2/logs/yahboomcar.log", "CHASSIS"},
	{"/tmp/raspybot_nav2/logs/*.log", "NAV2"},
	{"/tmp/slam_toolbox_rf2o_ai/demo_manager.log", "DEMO"},
	{"/tmp/slam_toolbox_rf2o_ai/realsense.log", "CAMERA"},
	{"/tmp/slam_toolbox_rf2o_ai/*.log", "AI"},
	{"/tmp/nav2_logs/logger.out", "TOOLS"},
}

// ROS logger name -> owner (exact name first, then prefix).
var nodeOwnerExact = map[string]string{"scan": "NAV2", "getTransform": "NAV2", "launch": ""}
var nodeOwnerPrefix = []struct{ prefix, owner string }{
	{"rplidar", "LIDAR"}, {"CLaserOdometry2D", "ODOM"}, {"rf2o", "ODOM"}, {"ekf", "EKF"},
	{"slam_toolbox", "SLAM"}, {"map_server", "SLAM"}, {"amcl", "SLAM"},
	{"static_transform_publisher", "TF"}, {"transform_listener", "TF"},
	{"camera", "CAMERA"}, {"realsense", "CAMERA"},
	{"ai_human_detection", "AI"}, {"base_to_ai_human_detection_tf", "AI"}, {"demo_manager", "DEMO"},
	{"driver_node", "CHASSIS"}, {"robot_console", "CONSOLE"},
	{"controller_server", "NAV2"}, {"planner_server", "NAV2"}, {"bt_navigator", "NAV2"},
	{"behavior_server", "NAV2"}, {"velocity_smoother", "NAV2"}, {"collision_monitor", "NAV2"},
	{"smoother_server", "NAV2"}, {"waypoint_follower", "NAV2"}, {"lifecycle_manager", "NAV2"},
	{"global_costmap", "NAV2"}, {"local_costmap", "NAV2"}, {"docking_server", "NAV2"},
	{"route_server", "NAV2"}, {"nav2", "NAV2"},
}

// Known failure strings -> owner + one-line hint (from debug_slam_lidar_ekf spec).
var hintRules = []struct {
	re    *regexp.Regexp
	owner string
	hint  string
}{
	{regexp.MustCompile(`operation time out|Failed to connect|checksum|Error, cannot bind`), "LIDAR",
		"rplidar serial problem (USB re-enumeration / power)"},
	{regexp.MustCompile(`Waiting for laser_scans`), "LIDAR", "rf2o gets no /scan (lidar stopped publishing)"},
	{regexp.MustCompile(`passed to lookupTransform argument target_frame does not exist`), "ODOM",
		"rf2o started before static TF: odom rotated 180°, restart the stack"},
	{regexp.MustCompile(`Failed to compute odom pose|Dropping scan`), "SLAM", "slam_toolbox cannot keep up (TF / CPU)"},
	{regexp.MustCompile(`Message Filter dropping`), "TF", "consumer waiting for TF (map→odom / odom→base late) or CPU"},
	{regexp.MustCompile(`Failed to meet update rate`), "EKF", "EKF overloaded (CPU) — odom→base_footprint may lag"},
	{regexp.MustCompile(`Transform data too old|xtrapolation`), "TF", "TF timing: a transform arrived late"},
	{regexp.MustCompile(`Robot to stop due to invalid source`), "NAV2", "collision_monitor stopped the robot (scan/TF timing)"},
	{regexp.MustCompile(`timestamps differ on`), "NAV2", "collision_monitor ignoring stale /scan (>1 s old)"},
	{regexp.MustCompile(`Failed to make progress`), "NAV2", "progress checker: robot did not move 0.10 m in 15 s"},
	{regexp.MustCompile(`costmap stop`), "NAV2", "controller: obstacle in the stop box"},
	{regexp.MustCompile(`ultrasonic stop`), "NAV2", "controller: sonar below stop distance"},
	{regexp.MustCompile(`Collision Ahead`), "NAV2", "recovery refused: collision ahead"},
	{regexp.MustCompile(`process has died|exit code -?[1-9]`), "", "process crashed"},
	{regexp.MustCompile(`NavigateToPose did not succeed|TARGET_LOST|NAVIGATION_FAILED`), "DEMO",
		"demo_manager stopped following (AI team)"},
	{regexp.MustCompile(`Using latest TF because source timestamp`), "DEMO",
		"AI timestamps older than the TF buffer (AI latency / clock)"},
}

// Lines dropped at ingestion (counted, shown in the UI as "hidden").
var noiseRules = []struct {
	label string
	node  *regexp.Regexp
	msg   *regexp.Regexp
}{
	{"rf2o per-scan INFO", regexp.MustCompile(`^CLaserOdometry2D`),
		regexp.MustCompile(`^(execution time|Laser odom|Robot-base odom)`)},
	{"AI per-person position INFO", regexp.MustCompile(`^ai_human_detection$`), regexp.MustCompile(`^Person -?\d+:`)},
	{"AI 'Couldn't find any detections'", regexp.MustCompile(`.*`), regexp.MustCompile(`^Couldn't find any detections`)},
	{"AI per-frame 'MJPG frame'", regexp.MustCompile(`^ai_human_detection$`), regexp.MustCompile(`MJPG frame: \d+ x \d+$`)},
	{"controller 'path downsampled'", regexp.MustCompile(`^controller_server$`), regexp.MustCompile(`^path downsampled`)},
	{"controller 'Passing new path'", regexp.MustCompile(`^controller_server$`), regexp.MustCompile(`^Passing new path`)},
}

var (
	ansiRe     = regexp.MustCompile(`\x1b\[[0-9;]*m`)
	launchRe   = regexp.MustCompile(`^\[([A-Za-z0-9_.\-]+)-\d+\] `)
	rosLineRe  = regexp.MustCompile(`^\[(DEBUG|INFO|WARN|WARNING|ERROR|FATAL)\] \[(\d+\.\d+)\] \[([^\]]+)\]: (.*)$`)
	launchLvRe = regexp.MustCompile(`^\[(DEBUG|INFO|WARN|WARNING|ERROR|FATAL)\] \[([^\]]+)\]: (.*)$`)
	errWordRe  = regexp.MustCompile(`(?i)\b(error|failed|fatal|exception|traceback)\b`)
	warnWordRe = regexp.MustCompile(`(?i)\bwarn(ing)?\b`)
)

type tailFile struct {
	path    string
	owner   string
	ino     uint64
	off     int64
	partial string
}

type LogStore struct {
	hub    *Hub
	mu     sync.Mutex
	ring   []LogEntry
	seen   map[string]float64 // rosout node|msg -> wall time, to drop file duplicates
	hidden map[string]int
	files  map[string]*tailFile
}

func NewLogStore(hub *Hub) *LogStore {
	return &LogStore{hub: hub, seen: map[string]float64{}, hidden: map[string]int{}, files: map[string]*tailFile{}}
}

func ownerOfNode(node string) string {
	if o, ok := nodeOwnerExact[node]; ok {
		return o
	}
	n := strings.TrimPrefix(node, "/")
	for _, p := range nodeOwnerPrefix {
		if strings.HasPrefix(n, p.prefix) || strings.Contains(n, "."+p.prefix) {
			return p.owner
		}
	}
	return ""
}

func levelLetter(s string) string {
	switch s {
	case "DEBUG":
		return "D"
	case "WARN", "WARNING":
		return "W"
	case "ERROR":
		return "E"
	case "FATAL":
		return "F"
	}
	return "I"
}

// IngestROS takes /rosout and /debug_log entries from the gateway.
func (l *LogStore) IngestROS(items []LogEntry) {
	now := nowSec()
	l.mu.Lock()
	for _, e := range items {
		l.seen[e.Node+"|"+e.Msg] = now
	}
	l.mu.Unlock()
	l.Add(items)
}

// Add classifies, filters noise and publishes entries.
func (l *LogStore) Add(items []LogEntry) {
	out := make([]json.RawMessage, 0, len(items))
	l.mu.Lock()
	for _, e := range items {
		noisy := false
		for _, n := range noiseRules {
			if n.node.MatchString(e.Node) && n.msg.MatchString(e.Msg) {
				l.hidden[n.label]++
				noisy = true
				break
			}
		}
		if noisy {
			continue
		}
		if len(e.Msg) > 2000 {
			e.Msg = e.Msg[:2000] + "…"
		}
		if e.Owner == "" {
			e.Owner = ownerOfNode(e.Node)
		}
		for _, h := range hintRules {
			if h.re.MatchString(e.Msg) {
				e.Hint = h.hint
				if h.owner != "" {
					e.Owner = h.owner
				}
				break
			}
		}
		l.ring = append(l.ring, e)
		b, _ := json.Marshal(e)
		out = append(out, b)
	}
	if len(l.ring) > 8000 {
		l.ring = append([]LogEntry(nil), l.ring[len(l.ring)-5000:]...)
	}
	l.mu.Unlock()
	l.hub.Append("log", out)
}

func (l *LogStore) Dump(w io.Writer) {
	l.mu.Lock()
	ring := append([]LogEntry(nil), l.ring...)
	l.mu.Unlock()
	for _, e := range ring {
		ts := time.UnixMicro(int64(e.T * 1e6)).Format("2006-01-02 15:04:05.000")
		hint := ""
		if e.Hint != "" {
			hint = "  ← " + e.Hint
		}
		fmt.Fprintf(w, "%s %s %-8s %-9s [%s] %s%s\n", ts, e.Lvl, e.Owner, e.Src, e.Node, e.Msg, hint)
	}
}

// Stats is published on the "logstats" channel.
func (l *LogStore) Stats() map[string]any {
	l.mu.Lock()
	defer l.mu.Unlock()
	files := []map[string]any{}
	for _, f := range l.files {
		files = append(files, map[string]any{"path": f.path, "owner": f.owner, "size": f.off})
	}
	sort.Slice(files, func(i, j int) bool { return files[i]["path"].(string) < files[j]["path"].(string) })
	hidden := map[string]int{}
	for k, v := range l.hidden {
		hidden[k] = v
	}
	return map[string]any{"files": files, "hidden": hidden, "stored": len(l.ring)}
}

// TailFiles follows every stack log file (they appear when the stack starts).
func (l *LogStore) TailFiles(stop <-chan struct{}) {
	first := true
	scan := time.NewTicker(3 * time.Second)
	read := time.NewTicker(700 * time.Millisecond)
	defer scan.Stop()
	defer read.Stop()
	l.discover(first)
	for {
		select {
		case <-stop:
			return
		case <-scan.C:
			l.discover(false)
			l.pruneSeen()
			b, _ := json.Marshal(l.Stats())
			l.hub.Set("logstats", b)
		case <-read.C:
			l.readAll()
		}
	}
}

func (l *LogStore) discover(startup bool) {
	for _, g := range logGlobs {
		matches, _ := filepath.Glob(g.glob)
		for _, p := range matches {
			l.mu.Lock()
			_, known := l.files[p]
			l.mu.Unlock()
			if known {
				continue
			}
			st, err := os.Stat(p)
			if err != nil || st.IsDir() {
				continue
			}
			f := &tailFile{path: p, owner: g.owner, ino: inode(st)}
			if startup {
				// existing file: only the recent context
				f.off = st.Size() - 32*1024
				if f.off < 0 {
					f.off = 0
				}
			}
			l.mu.Lock()
			l.files[p] = f
			l.mu.Unlock()
		}
	}
}

func inode(st os.FileInfo) uint64 {
	if s, ok := st.Sys().(*syscall.Stat_t); ok {
		return s.Ino
	}
	return 0
}

func (l *LogStore) readAll() {
	l.mu.Lock()
	files := make([]*tailFile, 0, len(l.files))
	for _, f := range l.files {
		files = append(files, f)
	}
	l.mu.Unlock()
	for _, f := range files {
		st, err := os.Stat(f.path)
		if err != nil {
			continue
		}
		if ino := inode(st); ino != f.ino || st.Size() < f.off {
			// recreated by a restart of the stack
			f.ino, f.off, f.partial = ino, 0, ""
		}
		if st.Size() == f.off {
			continue
		}
		fh, err := os.Open(f.path)
		if err != nil {
			continue
		}
		n := st.Size() - f.off
		if n > 512*1024 { // burst: skip ahead, keep the tail
			f.off = st.Size() - 512*1024
			f.partial = ""
			n = 512 * 1024
		}
		buf := make([]byte, n)
		k, _ := fh.ReadAt(buf, f.off)
		fh.Close()
		f.off += int64(k)
		text := f.partial + string(buf[:k])
		lines := strings.Split(text, "\n")
		f.partial = lines[len(lines)-1]
		l.ingestFile(f, lines[:len(lines)-1])
	}
}

func (l *LogStore) ingestFile(f *tailFile, lines []string) {
	now := nowSec()
	base := strings.TrimSuffix(filepath.Base(f.path), filepath.Ext(f.path))
	out := make([]LogEntry, 0, len(lines))
	l.mu.Lock()
	for _, raw := range lines {
		line := strings.TrimRight(ansiRe.ReplaceAllString(raw, ""), "\r ")
		if line == "" {
			continue
		}
		e := LogEntry{T: now, Lvl: "I", Node: base, Msg: line, Src: "file:" + filepath.Base(f.path), Owner: f.owner}
		rest := line
		if m := launchRe.FindStringSubmatch(rest); m != nil {
			e.Node = m[1]
			rest = rest[len(m[0]):]
		}
		if m := rosLineRe.FindStringSubmatch(rest); m != nil {
			e.Lvl = levelLetter(m[1])
			if t, err := strconv.ParseFloat(m[2], 64); err == nil {
				e.T = t
			}
			e.Node, e.Msg = m[3], m[4]
			if o := ownerOfNode(e.Node); o != "" {
				e.Owner = o
			}
			// same message already came in through /rosout
			if ts, ok := l.seen[e.Node+"|"+e.Msg]; ok && now-ts < 15 {
				continue
			}
		} else if m := launchLvRe.FindStringSubmatch(rest); m != nil {
			e.Lvl = levelLetter(m[1])
			e.Node, e.Msg = m[2], m[3]
		} else {
			e.Msg = rest
			if errWordRe.MatchString(rest) {
				e.Lvl = "E"
			} else if warnWordRe.MatchString(rest) {
				e.Lvl = "W"
			}
		}
		out = append(out, e)
	}
	l.mu.Unlock()
	if len(out) > 0 {
		l.Add(out)
	}
}

func (l *LogStore) pruneSeen() {
	now := nowSec()
	l.mu.Lock()
	for k, t := range l.seen {
		if now-t > 20 {
			delete(l.seen, k)
		}
	}
	l.mu.Unlock()
}
