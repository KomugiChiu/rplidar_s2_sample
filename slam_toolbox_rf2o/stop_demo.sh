#!/usr/bin/env bash
# stop_demo.sh — stop the person-follow demo stack on the board.
#
# Order:
#   1. stop_ai.sh   -> AI node, AI TF, person_target_recovery, demo_manager
#   2. stop_all.sh  -> SLAM, Nav2, EKF, RealSense, rplidar, chassis driver
#
# Usage:
#   ./stop_demo.sh
#
# Log:
#   /tmp/stop_demo.log  (override with LOG=/path)

set -u -o pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]:-$0}")" && pwd)"
LOG="${LOG:-/tmp/stop_demo.log}"

{
  echo "=============================================="
  echo " stop_demo: $(date '+%Y-%m-%d %H:%M:%S')"
  echo "=============================================="

  echo
  echo "== [1/2] stop_ai.sh =="
  if ! "$SCRIPT_DIR/stop_ai.sh"; then
    echo "[WARN] stop_ai.sh failed; continuing with stop_all.sh" >&2
  fi

  echo
  echo "== [2/2] stop_all.sh =="
  if ! "$SCRIPT_DIR/stop_all.sh"; then
    echo "[ERROR] stop_all.sh failed" >&2
    exit 1
  fi

  echo
  echo "== remaining follow/stack processes =="
  pgrep -af '[a]i_human_detection_node|[p]erson_target_recovery_node|[d]emo_manager_node|[s]lam_toolbox|[b]t_navigator|[c]ontroller_server|[r]ealsense2_camera_node|[r]plidar_node' \
    | cut -c1-120 || echo "none"

  echo
  echo "stop_demo done: $(date '+%Y-%m-%d %H:%M:%S')"
} 2>&1 | tee -a "$LOG"
