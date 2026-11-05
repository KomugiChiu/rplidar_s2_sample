package main

import (
	"context"
	"net/http"
	"os"
	"os/exec"
	"path/filepath"
	"sync"
	"time"
)

// Fixed map name used by the Map tab "Save map" button and by
// start_all_amcl.sh (maps/slam_map_latest.yaml).
const savedMapName = "slam_map_latest"

var mapSaveMu sync.Mutex

func (s *Server) handleSaveMap(w http.ResponseWriter, r *http.Request) {
	mapSaveMu.Lock()
	defer mapSaveMu.Unlock()

	dir := filepath.Join(s.scripts.dir, "maps")
	if err := os.MkdirAll(dir, 0o755); err != nil {
		writeJSON(w, 500, map[string]any{"ok": false, "msg": "create maps dir: " + err.Error()})
		return
	}
	prefix := filepath.Join(dir, savedMapName)

	ctx, cancel := context.WithTimeout(r.Context(), 30*time.Second)
	defer cancel()

	cmd := exec.CommandContext(ctx, "bash", "-lc",
		`source /opt/ros/jazzy/setup.bash && exec ros2 run nav2_map_server map_saver_cli -f "$1"`,
		"bash", prefix)
	cmd.Env = append(os.Environ(), "ROS_DOMAIN_ID="+envOr("ROS_DOMAIN_ID", "99"))

	out, err := cmd.CombinedOutput()
	if err != nil {
		msg := err.Error()
		if ctx.Err() == context.DeadlineExceeded {
			msg = "map save timed out"
		}
		writeJSON(w, 500, map[string]any{"ok": false, "msg": msg, "output": string(out)})
		return
	}

	writeJSON(w, 200, map[string]any{
		"ok":     true,
		"msg":    "saved",
		"path":   prefix + ".yaml",
		"output": string(out),
	})
}
