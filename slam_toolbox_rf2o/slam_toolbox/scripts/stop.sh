#!/usr/bin/env bash
# Stop RPLIDAR S2 + static laser TFs + slam_toolbox started by this folder.
# rf2o is stopped separately with ../rf2o/stop.sh.
#
# Usage:
#   ./stop.sh
#   DRY_RUN=1 ./stop.sh
#   KEEP_LIDAR=1 ./stop.sh    # keep rplidar_node if owned by someone else
set -eo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]:-$0}")" && pwd)"
RUN_DIR="${RUN_DIR:-/tmp/slam_toolbox_only}"
PID_FILE="${PID_FILE:-$RUN_DIR/pids}"
DRY_RUN="${DRY_RUN:-0}"
KEEP_LIDAR="${KEEP_LIDAR:-0}"

log() { echo "[slam_toolbox] $*"; }

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
  if [ "$DRY_RUN" != "1" ]; then
    pkill -TERM -f '[a]sync_slam_toolbox_node' 2>/dev/null || true
    pkill -TERM -f '[l]ocalization_slam_toolbox_node' 2>/dev/null || true
    pkill -TERM -f 'ros2 launch slam_toolbox' 2>/dev/null || true
    pkill -TERM -f 'static_transform_publisher.*base_footprint.*base_link' 2>/dev/null || true
    pkill -TERM -f 'static_transform_publisher.*base_link.*laser' 2>/dev/null || true
    if [ "$KEEP_LIDAR" != "1" ]; then
      pkill -TERM -f 'ros2 launch rplidar_ros rplidar_s2_launch.py' 2>/dev/null || true
      pkill -TERM -f 'rplidar_ros/rplidar_node' 2>/dev/null || true
    fi
  fi
fi

log "done"
