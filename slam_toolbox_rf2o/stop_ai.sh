#!/usr/bin/env bash
# Stop RealSense + AI detection + AI TF + DemoManager started by start_ai.sh.
#
# Does NOT stop SLAM / Nav2 / rf2o / lidar.
#
# Usage:
#   ./stop_ai.sh
#   DRY_RUN=1 ./stop_ai.sh
#
# The follow service is stopped first (if available), then all AI-related
# processes started by start_ai.sh are terminated.

set -eo pipefail

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-99}"

KIT_DIR="${KIT_DIR:-/nvt_ros2_example/demo_manager/fixed_build}"
LOG_DIR="${LOG_DIR:-/tmp/slam_toolbox_rf2o_ai}"
PID_FILE="${PID_FILE:-$LOG_DIR/pids}"
DRY_RUN="${DRY_RUN:-0}"

SCRIPT_PATH="$(readlink -f -- "${BASH_SOURCE[0]:-$0}" 2>/dev/null || echo "${BASH_SOURCE[0]:-$0}")"
SCRIPT_DIR="$(cd -- "$(dirname -- "$SCRIPT_PATH")" && pwd)"

log() { echo "[stop_ai] $*"; }

# --- stop follow service / zero cmd_vel before killing demo manager ---------
if [ "$DRY_RUN" != "1" ] && [ -x "$KIT_DIR/tools/stop_follow.sh" ]; then
  log "stopping follow service (if running)"
  timeout 8 "$KIT_DIR/tools/stop_follow.sh" || true
fi

# --- 1. kill recorded process groups ---------------------------------------
if [ -s "$PID_FILE" ]; then
  while read -r pid name; do
    [ -n "${pid:-}" ] || continue
    if kill -0 "$pid" 2>/dev/null; then
      log "stopping $name (pid $pid)"
      if [ "$DRY_RUN" = "1" ]; then
        continue
      fi
      kill -TERM -- "-$pid" 2>/dev/null || kill -TERM "$pid" 2>/dev/null || true
    fi
  done < "$PID_FILE"

  if [ "$DRY_RUN" != "1" ]; then
    sleep 2
    while read -r pid name; do
      [ -n "${pid:-}" ] || continue
      if kill -0 "$pid" 2>/dev/null; then
        log "killing $name (pid $pid)"
        kill -KILL -- "-$pid" 2>/dev/null || kill -KILL "$pid" 2>/dev/null || true
      fi
    done < "$PID_FILE"
    rm -f "$PID_FILE"
  fi
else
  log "no PID file: $PID_FILE"
fi

# --- 2. fallback: kill known AI process names ------------------------------
# Do NOT pkill realsense2_camera globally: the EKF stack may own the IMU
# camera.  The AI RGBD launch is stopped via the PID file above.
PATTERNS=(
  'ai_human_detection_node'
  'base_to_ai_human_detection_tf'
  'ai_tf_only.launch.py'
  'demo_manager_only.launch.py'
  'demo_manager_node'
  'person_target_recovery_node'
  'ai_sensor_integration.launch.py'
)

if [ "$DRY_RUN" = "1" ]; then
  log "DRY_RUN: would pkill patterns: ${PATTERNS[*]}"
  exit 0
fi

for pat in "${PATTERNS[@]}"; do
  pkill -TERM -f "$pat" 2>/dev/null || true
done

sleep 2

for pat in "${PATTERNS[@]}"; do
  pkill -KILL -f "$pat" 2>/dev/null || true
done

log "AI stack stopped"
