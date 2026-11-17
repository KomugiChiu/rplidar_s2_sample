#!/usr/bin/env bash
# Stop the stack started by start_demo.sh, start_demo_static.sh, or
# start_mapping.sh.
#
# Order:
#   1. AI processes recorded in the demo PID file (if any)
#   2. standalone slam_toolbox + rf2o stop scripts
#   3. Nav2 / EKF / RealSense / chassis driver via ../stop_all.sh
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
for d in "$SLAM_DIR" "$RF2O_DIR"; do
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

# --- 3. Nav2 / EKF / driver --------------------------------------------------
if [ -x "$ROOT_DIR/stop_all.sh" ]; then
  if [ "$DRY_RUN" = "1" ]; then
    log "would run $ROOT_DIR/stop_all.sh"
  else
    log "running stop_all.sh"
    "$ROOT_DIR/stop_all.sh"
  fi
else
  log "WARN: $ROOT_DIR/stop_all.sh not found"
fi

log "done"
