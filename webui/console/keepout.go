package main

import (
	"context"
	"encoding/json"
	"fmt"
	"io"
	"math"
	"net/http"
	"os"
	"os/exec"
	"path/filepath"
	"strings"
	"sync"
	"time"
)

// Keep-out zones (Map tab "Keep-out" tool).
//
// Zones are polygons in the map frame.  Saving writes
//
//	maps/keepout_zones.json          the polygons (what the UI edits)
//	maps/keepout_mask.{pgm,yaml}     the same zones rasterised for nav2_map_server
//
// and reloads the mask live through /keepout_filter_mask_server/load_map.
// The mask server + costmap_filter_info_server are started by start_nav2.sh
// (nav2_1028/keepout_filter_launch.py); both costmaps run a KeepoutFilter.
const (
	keepoutZonesFile = "keepout_zones.json"
	keepoutMaskName  = "keepout_mask"
	keepoutRes       = 0.05 // m/cell, same as the costmaps
	keepoutMaxZones  = 64
	keepoutMaxPts    = 256
	keepoutMaxCells  = 4000 * 4000
	keepoutLoadSrv   = "/keepout_filter_mask_server/load_map"
	keepoutProcPat   = "__node:=keepout_container"
)

type KeepoutZone struct {
	Name string       `json:"name"`
	Pts  [][2]float64 `json:"pts"`
}

type KeepoutDoc struct {
	Zones   []KeepoutZone `json:"zones"`
	Updated float64       `json:"updated,omitempty"`
	By      string        `json:"by,omitempty"`
}

var keepoutMu sync.Mutex

func (s *Server) keepoutDir() string { return filepath.Join(s.scripts.dir, "maps") }

func keepoutServerUp() bool {
	return exec.Command("pgrep", "-f", keepoutProcPat).Run() == nil
}

// GET /api/keepout
func (s *Server) handleKeepoutGet(w http.ResponseWriter, r *http.Request) {
	keepoutMu.Lock()
	defer keepoutMu.Unlock()
	doc := KeepoutDoc{Zones: []KeepoutZone{}}
	if b, err := os.ReadFile(filepath.Join(s.keepoutDir(), keepoutZonesFile)); err == nil {
		if err := json.Unmarshal(b, &doc); err != nil {
			writeJSON(w, 500, map[string]any{"ok": false, "msg": "bad " + keepoutZonesFile + ": " + err.Error()})
			return
		}
	}
	writeJSON(w, 200, map[string]any{"ok": true, "zones": doc.Zones, "updated": doc.Updated, "by": doc.By,
		"server": keepoutServerUp(), "mask": filepath.Join(s.keepoutDir(), keepoutMaskName+".yaml")})
}

// POST /api/keepout {zones:[{name, pts:[[x,y],…]}]}
func (s *Server) handleKeepoutSave(w http.ResponseWriter, r *http.Request) {
	var doc KeepoutDoc
	if err := json.NewDecoder(io.LimitReader(r.Body, 1<<20)).Decode(&doc); err != nil {
		writeJSON(w, 400, map[string]any{"ok": false, "msg": err.Error()})
		return
	}
	if err := validateZones(doc.Zones); err != nil {
		writeJSON(w, 400, map[string]any{"ok": false, "msg": err.Error()})
		return
	}
	if doc.Zones == nil {
		doc.Zones = []KeepoutZone{}
	}
	doc.Updated, doc.By = nowSec(), clientIP(r)

	keepoutMu.Lock()
	defer keepoutMu.Unlock()
	dir := s.keepoutDir()
	if err := os.MkdirAll(dir, 0o755); err != nil {
		writeJSON(w, 500, map[string]any{"ok": false, "msg": err.Error()})
		return
	}
	yamlPath, cells, err := writeKeepoutMask(dir, doc.Zones)
	if err != nil {
		writeJSON(w, 500, map[string]any{"ok": false, "msg": "write mask: " + err.Error()})
		return
	}
	b, _ := json.MarshalIndent(doc, "", "  ")
	if err := writeFileAtomic(filepath.Join(dir, keepoutZonesFile), b); err != nil {
		writeJSON(w, 500, map[string]any{"ok": false, "msg": err.Error()})
		return
	}

	rep := map[string]any{"ok": true, "saved": true, "zones": len(doc.Zones), "cells": cells, "mask": yamlPath}
	if !keepoutServerUp() {
		rep["applied"] = false
		rep["msg"] = "saved; keep-out server not running — it loads this mask when Nav2 starts"
	} else if out, err := keepoutReload(r.Context(), yamlPath); err != nil {
		rep["applied"] = false
		rep["msg"] = "saved, but live reload failed: " + err.Error()
		rep["output"] = out
	} else {
		rep["applied"] = true
		rep["msg"] = fmt.Sprintf("%d keep-out zone(s) applied to Nav2", len(doc.Zones))
	}
	lvl := "INFO"
	if rep["applied"] != true {
		lvl = "WARN"
	}
	emitEvent(s.hub, "CONSOLE", lvl, fmt.Sprintf("keep-out zones saved from %s: %v", doc.By, rep["msg"]))
	writeJSON(w, 200, rep)
}

func validateZones(zs []KeepoutZone) error {
	if len(zs) > keepoutMaxZones {
		return fmt.Errorf("at most %d zones", keepoutMaxZones)
	}
	for i, z := range zs {
		if len(z.Pts) < 3 || len(z.Pts) > keepoutMaxPts {
			return fmt.Errorf("zone %d: needs 3..%d points", i+1, keepoutMaxPts)
		}
		if len(z.Name) > 64 {
			return fmt.Errorf("zone %d: name too long", i+1)
		}
		for _, p := range z.Pts {
			for _, v := range p {
				if math.IsNaN(v) || math.IsInf(v, 0) || math.Abs(v) > 1000 {
					return fmt.Errorf("zone %d: coordinate out of range", i+1)
				}
			}
		}
	}
	return nil
}

// writeKeepoutMask rasterises the zones into a trinary map_server image:
// inside a zone = occupied (black, -> 100 -> lethal), elsewhere free.
// The mask only covers the zones' bounding box; KeepoutFilter ignores
// costmap cells outside the mask.
func writeKeepoutMask(dir string, zs []KeepoutZone) (string, int, error) {
	x0, y0, w, h := 0.0, 0.0, 1, 1
	if len(zs) > 0 {
		minX, minY, maxX, maxY := math.Inf(1), math.Inf(1), math.Inf(-1), math.Inf(-1)
		for _, z := range zs {
			for _, p := range z.Pts {
				minX, maxX = math.Min(minX, p[0]), math.Max(maxX, p[0])
				minY, maxY = math.Min(minY, p[1]), math.Max(maxY, p[1])
			}
		}
		// one free cell of margin around everything
		x0 = math.Floor(minX/keepoutRes)*keepoutRes - keepoutRes
		y0 = math.Floor(minY/keepoutRes)*keepoutRes - keepoutRes
		w = int(math.Ceil((maxX-x0)/keepoutRes)) + 2
		h = int(math.Ceil((maxY-y0)/keepoutRes)) + 2
		if w*h > keepoutMaxCells {
			return "", 0, fmt.Errorf("zones span %dx%d cells, too large", w, h)
		}
	}
	img := make([]byte, w*h)
	for i := range img {
		img[i] = 254
	}
	cells := 0
	for _, z := range zs {
		for row := 0; row < h; row++ {
			// image row 0 is the top (largest y); sample cell centres
			y := y0 + (float64(h-1-row)+0.5)*keepoutRes
			for col := 0; col < w; col++ {
				x := x0 + (float64(col)+0.5)*keepoutRes
				if img[row*w+col] != 0 && pointInPolygon(x, y, z.Pts) {
					img[row*w+col] = 0
					cells++
				}
			}
		}
	}

	pgm := filepath.Join(dir, keepoutMaskName+".pgm")
	var buf strings.Builder
	fmt.Fprintf(&buf, "P5\n# keep-out mask written by robot-console\n%d %d\n255\n", w, h)
	if err := writeFileAtomic(pgm, append([]byte(buf.String()), img...)); err != nil {
		return "", 0, err
	}
	yamlPath := filepath.Join(dir, keepoutMaskName+".yaml")
	y := fmt.Sprintf("image: %s.pgm\nmode: trinary\nresolution: %g\norigin: [%.4f, %.4f, 0.0]\nnegate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.25\n",
		keepoutMaskName, keepoutRes, x0, y0)
	return yamlPath, cells, writeFileAtomic(yamlPath, []byte(y))
}

// even-odd rule
func pointInPolygon(x, y float64, pts [][2]float64) bool {
	in := false
	for i, j := 0, len(pts)-1; i < len(pts); j, i = i, i+1 {
		xi, yi, xj, yj := pts[i][0], pts[i][1], pts[j][0], pts[j][1]
		if (yi > y) != (yj > y) && x < (xj-xi)*(y-yi)/(yj-yi)+xi {
			in = !in
		}
	}
	return in
}

func writeFileAtomic(path string, b []byte) error {
	tmp := path + ".tmp"
	if err := os.WriteFile(tmp, b, 0o644); err != nil {
		return err
	}
	return os.Rename(tmp, path)
}

// keepoutReload asks the mask server to reload the yaml (it republishes
// /keepout_filter_mask, which both KeepoutFilters pick up).
func keepoutReload(parent context.Context, yamlPath string) (string, error) {
	ctx, cancel := context.WithTimeout(parent, 25*time.Second)
	defer cancel()
	req := fmt.Sprintf("{map_url: '%s'}", yamlPath)
	cmd := exec.CommandContext(ctx, "bash", "-c",
		`source /opt/ros/jazzy/setup.bash && exec ros2 service call "$1" nav2_msgs/srv/LoadMap "$2"`,
		"bash", keepoutLoadSrv, req)
	cmd.Env = append(os.Environ(), "ROS_DOMAIN_ID="+envOr("ROS_DOMAIN_ID", "99"))
	b, err := cmd.CombinedOutput()
	out := string(b)
	if ctx.Err() == context.DeadlineExceeded {
		return out, fmt.Errorf("%s timed out", keepoutLoadSrv)
	}
	if err != nil {
		return out, err
	}
	// response: nav2_msgs.srv.LoadMap_Response(map=..., result=0)
	if strings.Contains(out, "result=0)") {
		return out, nil
	}
	if len(out) > 2000 {
		out = out[len(out)-2000:]
	}
	return out, fmt.Errorf("map_server rejected the mask")
}
