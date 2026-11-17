#!/usr/bin/env bash
# Stop the stack started by start_demo.sh, start_demo_static.sh, or
# start_mapping.sh.
#
# Order:
#   1. AI processes recorded in the demo PID file (if any)
#   2. standalone rf2o + slam_toolbox stop scripts
#   3. Nav2 / chassis driver and EKF / RealSense
#
# Usage:
#   ./stop_demo.sh
#   DRY_RUN=1 ./stop_demo.sh     # only list what would be stopped
#
# If AI was started outside these scripts, use ../stop_ai.sh separately.
set -eo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]:-$0}")" && pwd)"
ROOT_DIR="$(cd -- "$SCRIPT_DIR/.." && pwd)"

# Same defaults as demo_common.sh.
RUN_DIR="${RUN_DIR:-/tmp/demo_script}"
AI_PID_FILE="${AI_PID_FILE:-$RUN_DIR/ai_pids}"
SLAM_DIR="${SLAM_DIR:-/nvt_ros2_example/slam_toolbox}"
RF2O_DIR="${RF2O_DIR:-/nvt_ros2_example/rf2o}"
NAV2_DIR="${NAV2_DIR:-/nvt_ros2_example/nav2}"
EKF_DIR="${EKF_DIR:-/nvt_ros2_example/ekf}"
DRY_RUN="${DRY_RUN:-0}"

log() { echo "[stop_demo] $*"; }

log "stopping demo stack (DRY_RUN=$DRY_RUN)"

# --- 1. AI processes recorded by start_demo* --------------------------------
if [ -s "$AI_PID_FILE" ]; then
  while read -r pid name; do
    [ -n "${pid:-}" ] || continue
    if ! kill -0 "$pid" 2>/dev/null; then
      continue
    fi
    log "stopping $name (pid $pid)"
    if [ "$DRY_RUN" = "1" ]; then
      continue
    fi
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

# --- 2. standalone slam_toolbox + rf2o --------------------------------------
for d in "$RF2O_DIR" "$SLAM_DIR"; do
  if [ -x "$d/stop.sh" ]; then
    if [ "$DRY_RUN" = "1" ]; then
      log "would run $d/stop.sh"
    else
      log "running $d/stop.sh"
      "$d/stop.sh" || true
    fi
  else
    log "WARN: $d/stop.sh not found"
  fi
done

# --- 3. Nav2 / EKF -----------------------------------------------------------
for d in "$NAV2_DIR" "$EKF_DIR"; do
  if [ -x "$d/stop.sh" ]; then
    if [ "$DRY_RUN" = "1" ]; then
      log "would run $d/stop.sh"
    else
      log "running $d/stop.sh"
      "$d/stop.sh" || true
    fi
  elif [ -x "$d/stop_nav2.sh" ]; then
    if [ "$DRY_RUN" = "1" ]; then
      log "would run $d/stop_nav2.sh"
    else
      log "running $d/stop_nav2.sh"
      "$d/stop_nav2.sh" || true
    fi
  elif [ -x "$d/stop_ekf.sh" ]; then
    if [ "$DRY_RUN" = "1" ]; then
      log "would run $d/stop_ekf.sh"
    else
      log "running $d/stop_ekf.sh"
      "$d/stop_ekf.sh" || true
    fi
  else
    log "WARN: no stop script in $d"
  fi
done

log "done"
