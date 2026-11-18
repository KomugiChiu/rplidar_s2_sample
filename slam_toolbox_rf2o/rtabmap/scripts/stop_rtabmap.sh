#!/usr/bin/env bash
# Stop RPLIDAR S2 + static laser TFs + RTAB-Map started by start_rtabmap.sh.
# rf2o / EKF are stopped separately.
#
# Usage:
#   ./stop_rtabmap.sh
#   DRY_RUN=1 ./stop_rtabmap.sh
#   KEEP_LIDAR=1 ./stop_rtabmap.sh
set -eo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]:-$0}")" && pwd)"
RUN_DIR="${RUN_DIR:-/tmp/rtabmap_only}"
PID_FILE="${PID_FILE:-$RUN_DIR/pids}"
DRY_RUN="${DRY_RUN:-0}"
KEEP_LIDAR="${KEEP_LIDAR:-0}"

log() { echo "[rtabmap] $*"; }
kill_pid() {
  local sig="$1" pid="$2"
  kill "-$sig" -- "-$pid" 2>/dev/null || kill "-$sig" "$pid" 2>/dev/null || true
}

if [ -s "$PID_FILE" ]; then
  while read -r pid name; do
    [ -n "${pid:-}" ] || continue
    kill -0 "$pid" 2>/dev/null || continue
    log "stopping $name (pid $pid)"
    [ "$DRY_RUN" = "1" ] && continue
    kill_pid TERM "$pid"
  done < "$PID_FILE"

  if [ "$DRY_RUN" != "1" ]; then
    sleep 2
    while read -r pid name; do
      [ -n "${pid:-}" ] || continue
      if kill -0 "$pid" 2>/dev/null; then
        log "killing $name (pid $pid)"
        kill_pid KILL "$pid"
      fi
    done < "$PID_FILE"
    rm -f "$PID_FILE"
  fi
else
  log "no PID file: $PID_FILE"
fi

# --- sweep ---
# ros2 launch children can survive the process-group TERM (they get
# reparented to init), and a leftover rplidar_node keeps /dev/ttyUSB0 busy so
# the next start fails with "process died: rplidar".  Always sweep by name.
if [ "$DRY_RUN" != "1" ]; then
  sweep() {
    pkill -TERM -f 'rtabmap_slam/rtabmap' 2>/dev/null || true
    pkill -TERM -f 'rtabmap_odom/icp_odometry' 2>/dev/null || true
    pkill -TERM -f 'ros2 launch rtabmap_launch' 2>/dev/null || true
    pkill -TERM -f 'static_transform_publisher.*base_footprint.*base_link' 2>/dev/null || true
    pkill -TERM -f 'static_transform_publisher.*base_link.*laser' 2>/dev/null || true
    if [ "$KEEP_LIDAR" != "1" ]; then
      pkill -TERM -f 'ros2 launch rplidar_ros rplidar_s2_launch.py' 2>/dev/null || true
      pkill -TERM -f 'rplidar_ros/rplidar_node' 2>/dev/null || true
    fi
  }
  sweep
  sleep 2
  pkill -KILL -f 'rtabmap_slam/rtabmap' 2>/dev/null || true
  pkill -KILL -f 'rtabmap_odom/icp_odometry' 2>/dev/null || true
  pkill -KILL -f 'ros2 launch rtabmap_launch' 2>/dev/null || true
  pkill -KILL -f 'static_transform_publisher.*base_footprint.*base_link' 2>/dev/null || true
  pkill -KILL -f 'static_transform_publisher.*base_link.*laser' 2>/dev/null || true
  if [ "$KEEP_LIDAR" != "1" ]; then
    pkill -KILL -f 'ros2 launch rplidar_ros rplidar_s2_launch.py' 2>/dev/null || true
    pkill -KILL -f 'rplidar_ros/rplidar_node' 2>/dev/null || true
  fi
fi

log "done"
