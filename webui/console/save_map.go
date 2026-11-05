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

// Fixed map name used by the Map tab "Save map" button, start_all_amcl.sh
// and slam_toolbox_localization.yaml (maps/slam_map_latest.yaml / .posegraph).
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

	// Also serialize the posegraph (needed by slam_toolbox localization mode).
	// Skipped automatically when slam_toolbox is not running (AMCL mode).
	pgOK, pgOut := savePosegraph(prefix)

	msg := "saved (map only)"
	if pgOK {
		msg = "saved (map + posegraph)"
	}
	writeJSON(w, 200, map[string]any{
		"ok":               true,
		"msg":              msg,
		"path":             prefix + ".yaml",
		"posegraph":        pgOK,
		"posegraph_path":   prefix + ".posegraph",
		"output":           string(out),
		"posegraph_output": pgOut,
	})
}

// savePosegraph calls /slam_toolbox/serialize_map and verifies that both
// <prefix>.posegraph and <prefix>.data were (re)written.
func savePosegraph(prefix string) (bool, string) {
	ctx, cancel := context.WithTimeout(context.Background(), 120*time.Second)
	defer cancel()

	start := time.Now()
	cmd := exec.CommandContext(ctx, "bash", "-lc",
		`source /opt/ros/jazzy/setup.bash && exec ros2 service call `+
			`/slam_toolbox/serialize_map slam_toolbox/srv/SerializePoseGraph `+
			`"{filename: '$1'}"`,
		"bash", prefix)
	cmd.Env = append(os.Environ(), "ROS_DOMAIN_ID="+envOr("ROS_DOMAIN_ID", "99"))

	out, err := cmd.CombinedOutput()
	if err != nil {
		return false, string(out) + "\n" + err.Error()
	}

	files := []string{prefix + ".posegraph", prefix + ".data"}
	for _, f := range files {
		st, statErr := os.Stat(f)
		if statErr != nil {
			return false, string(out) + "\nmissing: " + f
		}
		// allow 2 s of filesystem timestamp slack
		if st.ModTime().Before(start.Add(-2 * time.Second)) {
			return false, string(out) + "\nstale: " + f
		}
	}
	return true, string(out)
}
