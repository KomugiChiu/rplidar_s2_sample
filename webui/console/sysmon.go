package main

import (
	"bufio"
	"encoding/json"
	"fmt"
	"os"
	"os/exec"
	"path/filepath"
	"regexp"
	"sort"
	"strconv"
	"strings"
	"syscall"
	"time"
)

// Stack processes.  Match is a substring of the command line; the
// "ros2 run/launch" python wrappers are skipped so only the node counts.
var procDefs = []struct {
	Name, Owner, Match string
	Core               bool // required for navigation
}{
	{"rplidar_node", "LIDAR", "rplidar_node", true},
	{"rf2o_laser_odometry", "ODOM", "rf2o_laser_odometry_node", true},
	{"slam_toolbox", "SLAM", "slam_toolbox_node", true},
	{"ekf_node", "EKF", "robot_localization/ekf_node", true},
	{"realsense2_camera", "CAMERA", "realsense2_camera_node", true},
	{"static_transform_publisher", "TF", "tf2_ros/static_transform_publisher", true},
	{"controller_server", "NAV2", "nav2_controller/controller_server", true},
	{"planner_server", "NAV2", "nav2_planner/planner_server", true},
	{"bt_navigator", "NAV2", "nav2_bt_navigator/bt_navigator", true},
	{"behavior_server", "NAV2", "nav2_behaviors/behavior_server", true},
	{"velocity_smoother", "NAV2", "nav2_velocity_smoother/velocity_smoother", true},
	{"collision_monitor", "NAV2", "nav2_collision_monitor/collision_monitor", true},
	{"smoother_server", "NAV2", "nav2_smoother/smoother_server", true},
	{"waypoint_follower", "NAV2", "nav2_waypoint_follower/waypoint_follower", true},
	{"lifecycle_manager", "NAV2", "nav2_lifecycle_manager/lifecycle_manager", true},
	{"route_server", "NAV2", "nav2_route/route_server", false},
	{"keepout_container", "NAV2", "__node:=keepout_container", false},
	{"docking_server", "NAV2", "opennav_docking/opennav_docking", false},
	{"Mcnamu_driver", "CHASSIS", "yahboomcar_bringup/Mcnamu_driver", true},
	{"record_cmd_vel", "TOOLS", "record_cmd_vel.py", false},
	{"ai_human_detection", "AI", "ai_human_detection/ai_human_detection_node", false},
	{"demo_manager", "DEMO", "demo_manager/demo_manager_node", false},
	{"console gateway", "CONSOLE", "robot_console_gateway.py", false},
}

var pidFiles = []struct{ path, owner string }{
	{"/tmp/slam_toolbox_rf2o/pids", "LIDAR"},
	{"/tmp/slam_toolbox_rf2o_ekf/pids", "EKF"},
	{"/tmp/raspybot_nav2/pids", "NAV2"},
	{"/tmp/slam_toolbox_rf2o_ai/pids", "AI"},
}

const lidarSerial = "/dev/ttyUSB0"

var dmesgRe = regexp.MustCompile(`ttyUSB|cp210x|usb .*disconnect|over-current|USB disconnect`)

type Check struct {
	ID     string `json:"id"`
	Owner  string `json:"owner"`
	Name   string `json:"name"`
	Level  string `json:"level"`
	Value  string `json:"value"`
	Detail string `json:"detail"`
}

type procSample struct {
	ticks uint64
	at    time.Time
}

type ProcInfo struct {
	Name    string  `json:"name"`
	Owner   string  `json:"owner"`
	PID     int     `json:"pid"`
	CPU     float64 `json:"cpu"`
	RSS     float64 `json:"rss_mb"`
	Threads int     `json:"threads"`
	Up      float64 `json:"up_s"`
	Cmd     string  `json:"cmd"`
	Missing bool    `json:"missing,omitempty"`
	Core    bool    `json:"core,omitempty"`
}

type SysMon struct {
	hub      *Hub
	gw       *Gateway
	scripts  *ScriptRunner
	prevCPU  [][2]uint64
	procPrev map[int]procSample
	alive    map[string]bool
	bad      map[string]int
	dmesg    []string
	dmesgAt  time.Time
	bootTime float64
	clkTck   float64
	pageMB   float64

	lastProcs    []ProcInfo
	lastPidFiles []map[string]any
	lastStackUp  bool
	procChecks   []Check
}

func NewSysMon(hub *Hub, gw *Gateway, scripts *ScriptRunner) *SysMon {
	s := &SysMon{hub: hub, gw: gw, scripts: scripts, procPrev: map[int]procSample{},
		alive: map[string]bool{}, bad: map[string]int{}, clkTck: 100, pageMB: float64(os.Getpagesize()) / 1048576}
	if b, err := os.ReadFile("/proc/stat"); err == nil {
		for _, line := range strings.Split(string(b), "\n") {
			if strings.HasPrefix(line, "btime ") {
				s.bootTime, _ = strconv.ParseFloat(strings.TrimSpace(line[6:]), 64)
			}
		}
	}
	return s
}

func (s *SysMon) Run(stop <-chan struct{}) {
	t := time.NewTicker(time.Second)
	defer t.Stop()
	n := 0
	for {
		select {
		case <-stop:
			return
		case <-t.C:
			n++
			s.sample(n)
		}
	}
}

// escalate: ERROR only after `need` bad samples in a row (spec: avoid one-off spikes).
func (s *SysMon) escalate(id, level string, need int) string {
	if level == "ERROR" || level == "WARN" {
		s.bad[id]++
		if level == "ERROR" && s.bad[id] < need {
			return "WARN"
		}
		if level == "WARN" && s.bad[id] < need/2 {
			return "OK"
		}
		return level
	}
	s.bad[id] = 0
	return level
}

func (s *SysMon) sample(n int) {
	out := map[string]any{"t": nowSec()}
	checks := []Check{}
	add := func(c Check) { checks = append(checks, c) }

	// CPU
	cores, total := s.cpu()
	out["cpu"] = map[string]any{"total": total, "cores": cores}
	lvl := "OK"
	if total > 85 {
		lvl = "ERROR"
	} else if total > 75 {
		lvl = "WARN"
	}
	add(Check{"sys:cpu", "SYSTEM", "CPU total (4 cores)", s.escalate("cpu", lvl, 10), fmt.Sprintf("%.0f %%", total),
		"baseline 46–53 % with the full stack"})
	for i, c := range cores {
		l := "OK"
		if c > 95 {
			l = "ERROR"
		} else if c > 90 {
			l = "WARN"
		}
		l = s.escalate(fmt.Sprintf("core%d", i), l, 10)
		if l != "OK" {
			add(Check{fmt.Sprintf("sys:core%d", i), "SYSTEM", fmt.Sprintf("CPU core %d", i), l,
				fmt.Sprintf("%.0f %%", c), "saturated core: control loops will miss deadlines"})
		}
	}
	// load average (shown, not alarmed: D-state kernel threads make it meaningless)
	if b, err := os.ReadFile("/proc/loadavg"); err == nil {
		out["loadavg"] = strings.Join(strings.Fields(string(b))[:3], " ")
	}

	// memory
	mem := meminfo()
	out["mem"] = mem
	avail := mem["MemAvailable"] / 1024
	lvl = "OK"
	if avail < 300 {
		lvl = "ERROR"
	} else if avail < 500 {
		lvl = "WARN"
	}
	add(Check{"sys:mem", "SYSTEM", "Memory available", lvl, fmt.Sprintf("%.0f MB", avail),
		fmt.Sprintf("of %.0f MB", mem["MemTotal"]/1024)})

	// temperature
	temps := []map[string]any{}
	maxT := 0.0
	zones, _ := filepath.Glob("/sys/class/thermal/thermal_zone*")
	for _, z := range zones {
		b, err := os.ReadFile(z + "/temp")
		if err != nil {
			continue
		}
		v, err := strconv.ParseFloat(strings.TrimSpace(string(b)), 64)
		if err != nil {
			continue
		}
		v /= 1000
		typ, _ := os.ReadFile(z + "/type")
		temps = append(temps, map[string]any{"zone": filepath.Base(z), "type": strings.TrimSpace(string(typ)), "c": v})
		if v > maxT {
			maxT = v
		}
	}
	out["temps"] = temps
	if len(temps) > 0 {
		lvl = "OK"
		if maxT > 90 {
			lvl = "ERROR"
		} else if maxT > 85 {
			lvl = "WARN"
		}
		add(Check{"sys:temp", "SYSTEM", "SoC temperature", s.escalate("temp", lvl, 3), fmt.Sprintf("%.0f °C", maxT), ""})
	}

	// disks
	disks := []map[string]any{}
	for _, p := range []string{"/", "/tmp"} {
		var st syscall.Statfs_t
		if syscall.Statfs(p, &st) != nil {
			continue
		}
		free := float64(st.Bavail) * float64(st.Bsize) / 1048576
		tot := float64(st.Blocks) * float64(st.Bsize) / 1048576
		disks = append(disks, map[string]any{"path": p, "free_mb": free, "total_mb": tot})
		lvl = "OK"
		if free < 500 {
			lvl = "ERROR"
		} else if free < 1024 {
			lvl = "WARN"
		}
		add(Check{"sys:disk" + p, "SYSTEM", "Free space " + p, lvl, fmt.Sprintf("%.1f GB", free/1024), "logs and CSVs grow in /tmp"})
	}
	out["disks"] = disks

	// Wi-Fi
	if w := wifi(); w != nil {
		out["wifi"] = w
		if lv, ok := w["level"].(float64); ok {
			lvl = "OK"
			if lv < -80 {
				lvl = "ERROR"
			} else if lv < -72 {
				lvl = "WARN"
			}
			add(Check{"sys:wifi", "SYSTEM", "Wi-Fi signal (" + w["iface"].(string) + ")", s.escalate("wifi", lvl, 3),
				fmt.Sprintf("%.0f dBm", lv), "the console and RViz need a stable link"})
		}
	}
	if b, err := os.ReadFile("/proc/uptime"); err == nil {
		out["uptime"], _ = strconv.ParseFloat(strings.Fields(string(b))[0], 64)
	}
	_, err := os.Stat("/run/systemd/timesync/synchronized")
	out["clock_synced"] = err == nil

	// processes + PID files (every 2 s, cached in between)
	if n%2 == 1 {
		procChecks := []Check{}
		procs, running := s.processes()
		stackUp := false
		for _, p := range procs {
			if p.Core && !p.Missing && p.Owner != "CONSOLE" {
				stackUp = true
			}
		}
		if !stackUp {
			procChecks = append(procChecks, Check{"proc:stack", "SYSTEM", "ROS stack", "ERROR", "not running",
				"start it with start_all.sh (System tab)"})
		}
		for _, d := range procDefs {
			was, is := s.alive[d.Name], running[d.Name]
			if was && !is {
				emitEvent(s.hub, d.Owner, map[bool]string{true: "ERROR", false: "WARN"}[d.Core], d.Name+" process exited")
			} else if !was && is && n > 2 {
				emitEvent(s.hub, d.Owner, "INFO", d.Name+" process started")
			}
			s.alive[d.Name] = is
			if d.Core && stackUp && !is {
				procChecks = append(procChecks, Check{"proc:" + d.Name, d.Owner, "Process " + d.Name, "ERROR", "not running", ""})
			}
		}
		// PID files written by the start scripts
		pf := []map[string]any{}
		for _, f := range pidFiles {
			b, err := os.ReadFile(f.path)
			if err != nil {
				continue
			}
			for _, line := range strings.Split(strings.TrimSpace(string(b)), "\n") {
				fs := strings.Fields(line)
				if len(fs) < 2 {
					continue
				}
				pid, _ := strconv.Atoi(fs[0])
				_, err := os.Stat(fmt.Sprintf("/proc/%d", pid))
				alive := err == nil
				pf = append(pf, map[string]any{"file": f.path, "name": fs[1], "pid": pid, "alive": alive})
				if !alive {
					procChecks = append(procChecks, Check{"pid:" + f.path + fs[1], f.owner, "PID file entry " + fs[1],
						"WARN", "dead", fmt.Sprintf("pid %d in %s is gone (crashed or stopped by hand)", pid, f.path)})
				}
			}
		}
		s.lastProcs, s.lastPidFiles, s.lastStackUp, s.procChecks = procs, pf, stackUp, procChecks
	}
	out["procs"] = s.lastProcs
	out["pidfiles"] = s.lastPidFiles
	out["stack_up"] = s.lastStackUp
	checks = append(checks, s.procChecks...)

	// lidar serial port + USB events
	_, err = os.Stat(lidarSerial)
	out["serial"] = map[string]any{"path": lidarSerial, "present": err == nil}
	if err != nil {
		add(Check{"lidar:serial", "LIDAR", "Lidar serial " + lidarSerial, "ERROR", "missing",
			"USB unplugged or re-enumerated (ttyUSB1?)"})
	}
	if time.Since(s.dmesgAt) > 15*time.Second {
		s.dmesgAt = time.Now()
		s.dmesg = usbEvents()
	}
	out["dmesg"] = s.dmesg

	out["gateway"] = s.gw.Status()
	if gs := s.gw.Status(); !gs.Up {
		add(Check{"console:gateway", "CONSOLE", "Console ROS gateway", "ERROR", "down",
			"the web console cannot see ROS; see stderr in the System tab"})
	} else {
		add(Check{"console:gateway", "CONSOLE", "Console ROS gateway", "OK", "up",
			fmt.Sprintf("restarts %d", gs.Restarts)})
	}
	out["clients"] = s.hub.ClientCount()
	out["checks"] = checks
	b, _ := json.Marshal(out)
	s.hub.Set("sys", b)
}

func (s *SysMon) cpu() ([]float64, float64) {
	f, err := os.Open("/proc/stat")
	if err != nil {
		return nil, 0
	}
	defer f.Close()
	var cur [][2]uint64 // busy, total ; index 0 = all
	sc := bufio.NewScanner(f)
	for sc.Scan() {
		line := sc.Text()
		if !strings.HasPrefix(line, "cpu") {
			break
		}
		fs := strings.Fields(line)
		var tot, idle uint64
		for i, v := range fs[1:] {
			x, _ := strconv.ParseUint(v, 10, 64)
			if i < 8 {
				tot += x
			}
			if i == 3 || i == 4 {
				idle += x
			}
		}
		cur = append(cur, [2]uint64{tot - idle, tot})
	}
	prev := s.prevCPU
	s.prevCPU = cur
	if len(prev) != len(cur) || len(cur) == 0 {
		return nil, 0
	}
	pct := func(i int) float64 {
		db := float64(cur[i][0] - prev[i][0])
		dt := float64(cur[i][1] - prev[i][1])
		if dt <= 0 {
			return 0
		}
		return 100 * db / dt
	}
	cores := make([]float64, len(cur)-1)
	for i := range cores {
		cores[i] = pct(i + 1)
	}
	return cores, pct(0)
}

func meminfo() map[string]float64 {
	out := map[string]float64{}
	b, err := os.ReadFile("/proc/meminfo")
	if err != nil {
		return out
	}
	for _, line := range strings.Split(string(b), "\n") {
		fs := strings.Fields(line)
		if len(fs) >= 2 {
			v, _ := strconv.ParseFloat(fs[1], 64)
			out[strings.TrimSuffix(fs[0], ":")] = v
		}
	}
	return map[string]float64{"MemTotal": out["MemTotal"], "MemAvailable": out["MemAvailable"]}
}

func wifi() map[string]any {
	b, err := os.ReadFile("/proc/net/wireless")
	if err != nil {
		return nil
	}
	lines := strings.Split(strings.TrimSpace(string(b)), "\n")
	if len(lines) < 3 {
		return nil
	}
	fs := strings.Fields(lines[2])
	if len(fs) < 4 {
		return nil
	}
	q, _ := strconv.ParseFloat(strings.TrimSuffix(fs[2], "."), 64)
	lv, _ := strconv.ParseFloat(strings.TrimSuffix(fs[3], "."), 64)
	return map[string]any{"iface": strings.TrimSuffix(fs[0], ":"), "quality": q, "level": lv}
}

func usbEvents() []string {
	out, err := exec.Command("dmesg", "--time-format", "iso").Output()
	if err != nil {
		return nil
	}
	var ev []string
	for _, line := range strings.Split(string(out), "\n") {
		if dmesgRe.MatchString(line) {
			ev = append(ev, line)
		}
	}
	if len(ev) > 8 {
		ev = ev[len(ev)-8:]
	}
	return ev
}

func (s *SysMon) processes() ([]ProcInfo, map[string]bool) {
	now := time.Now()
	entries, _ := os.ReadDir("/proc")
	found := []ProcInfo{}
	running := map[string]bool{}
	seen := map[int]bool{}
	for _, e := range entries {
		pid, err := strconv.Atoi(e.Name())
		if err != nil {
			continue
		}
		cl, err := os.ReadFile(fmt.Sprintf("/proc/%d/cmdline", pid))
		if err != nil || len(cl) == 0 {
			continue
		}
		args := strings.Split(strings.TrimRight(string(cl), "\x00"), "\x00")
		wrapper := false
		for _, a := range args {
			if strings.HasSuffix(a, "/bin/ros2") || a == "ros2" {
				wrapper = true
				break
			}
		}
		if wrapper || args[0] == "bash" || strings.HasSuffix(args[0], "/bash") {
			continue
		}
		cmd := strings.Join(args, " ")
		for _, d := range procDefs {
			if !strings.Contains(cmd, d.Match) {
				continue
			}
			p := ProcInfo{Name: d.Name, Owner: d.Owner, PID: pid, Core: d.Core}
			if len(cmd) > 160 {
				cmd = cmd[:160] + "…"
			}
			p.Cmd = cmd
			if st, err := os.ReadFile(fmt.Sprintf("/proc/%d/stat", pid)); err == nil {
				// fields after the ")" of comm
				rest := string(st)
				if i := strings.LastIndexByte(rest, ')'); i >= 0 {
					fs := strings.Fields(rest[i+1:])
					if len(fs) > 22 {
						ut, _ := strconv.ParseUint(fs[11], 10, 64)
						stt, _ := strconv.ParseUint(fs[12], 10, 64)
						p.Threads, _ = strconv.Atoi(fs[17])
						start, _ := strconv.ParseFloat(fs[19], 64)
						rss, _ := strconv.ParseFloat(fs[21], 64)
						p.RSS = rss * s.pageMB
						if s.bootTime > 0 {
							p.Up = float64(now.Unix()) - (s.bootTime + start/s.clkTck)
						}
						ticks := ut + stt
						if prev, ok := s.procPrev[pid]; ok {
							dt := now.Sub(prev.at).Seconds()
							if dt > 0 && ticks >= prev.ticks {
								p.CPU = 100 * float64(ticks-prev.ticks) / s.clkTck / dt
							}
						}
						s.procPrev[pid] = procSample{ticks, now}
					}
				}
			}
			seen[pid] = true
			running[d.Name] = true
			found = append(found, p)
			break
		}
	}
	for pid := range s.procPrev {
		if !seen[pid] {
			delete(s.procPrev, pid)
		}
	}
	for _, d := range procDefs {
		if !running[d.Name] {
			found = append(found, ProcInfo{Name: d.Name, Owner: d.Owner, Missing: true, Core: d.Core})
		}
	}
	sort.SliceStable(found, func(i, j int) bool { return found[i].CPU > found[j].CPU })
	return found, running
}
