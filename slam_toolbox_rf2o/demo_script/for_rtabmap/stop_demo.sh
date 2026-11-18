#!/usr/bin/env bash
# Stop the RTAB-Map demo started by start_demo.sh.
#
# Order:
#   1. AI processes recorded in the demo PID file
#   2. RTAB-Map + lidar/TF + icp_odometry
#   3. rf2o            (no-op when ODOM_SOURCE=icp)
#   4. Nav2 + driver
#   5. EKF + RealSense
#
# Usage:
#   ./stop_demo.sh
#   DRY_RUN=1 ./stop_demo.sh
set -eo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]:-$0}")" && pwd)"
RTABMAP_DIR="${RTABMAP_DIR:-/nvt_ros2_example/rtabmap}"
RF2O_DIR="${RF2O_DIR:-/nvt_ros2_example/rf2o}"
NAV2_DIR="${NAV2_DIR:-/nvt_ros2_example/nav2}"
EKF_DIR="${EKF_DIR:-/nvt_ros2_example/ekf}"
DEMO_INSTALL="${DEMO_INSTALL:-/nvt_ros2_example/demo_manager/install}"
RUN_DIR="${RUN_DIR:-/tmp/demo_script}"
AI_PID_FILE="${AI_PID_FILE:-$RUN_DIR/ai_pids}"
DRY_RUN="${DRY_RUN:-0}"

log() { echo "[stop_rtabmap_demo] $*"; }

log "stopping RTAB-Map demo (DRY_RUN=$DRY_RUN)"

# --- AI ---
if [ -s "$AI_PID_FILE" ]; then
  while read -r pid name; do
    [ -n "${pid:-}" ] || continue
    kill -0 "$pid" 2>/dev/null || continue
    log "stopping $name (pid $pid)"
    [ "$DRY_RUN" = "1" ] && continue
    kill -TERM -- "-$pid" 2>/dev/null || kill -TERM "$pid" 2>/dev/null || true
  done < "$AI_PID_FILE"
  if [ "$DRY_RUN" != "1" ]; then
    sleep 2
    while read -r pid name; do
      [ -n "${pid:-}" ] || continue
      if kill -0 "$pid" 2>/dev/null; then
        log "killing $name (pid $pid)"
        kill -KILL -- "-$pid" 2>/dev/null || kill -KILL "$pid" 2>/dev/null || true
      fi
    done < "$AI_PID_FILE"
    rm -f "$AI_PID_FILE"
  fi
else
  log "no AI PID file: $AI_PID_FILE"
fi

# --- RTAB-Map / rf2o / Nav2 / EKF ---
for spec in \
  "demo_manager:$DEMO_INSTALL/stop_demo_only.sh" \
  "rtabmap:$RTABMAP_DIR/stop_rtabmap.sh" \
  "rf2o:$RF2O_DIR/stop.sh" \
  "nav2:$NAV2_DIR/stop_nav2.sh" \
  "ekf:$EKF_DIR/stop_ekf.sh"; do
  name="${spec%%:*}"
  script="${spec#*:}"
  if [ -x "$script" ]; then
    if [ "$DRY_RUN" = "1" ]; then
      log "would run $script"
    else
      log "running $script"
      "$script" || true
    fi
  else
    log "WARN: $name stop script not found: $script"
  fi
done

log "done"
