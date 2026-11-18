#!/usr/bin/env bash
# Unified slam_toolbox start script.
#
# Modes:
#   mapping       build a new map (online_async_launch.py)
#   localization  load an existing posegraph (localization_launch.py)
#
# Usage:
#   ./start.sh mapping
#   ./start.sh localization
#   ./start.sh                 # default: mapping
#
# Environment:
#   SLAM_PARAMS=/path/to/params.yaml
#   SKIP_LIDAR=1               # reuse an existing rplidar_node
#
# Stop with: ./stop.sh
set -eo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]:-$0}")" && pwd)"

usage() {
  sed -n '2,18p' "$0" | sed 's/^# \{0,1\}//'
}

MODE="${1:-${SLAM_MODE:-mapping}}"
case "$MODE" in
  mapping|map) MODE="mapping" ;;
  localization|localization-only|static|static-map|static_map) MODE="localization" ;;
  --help|-h) usage; exit 0 ;;
  *)
    echo "ERROR: unknown mode '$MODE' (use mapping or localization)" >&2
    usage >&2
    exit 2
    ;;
esac

export SLAM_MODE="$MODE"

if [ "$MODE" = "localization" ]; then
  SLAM_PARAMS="${SLAM_PARAMS:-$SCRIPT_DIR/slam_toolbox_localization.yaml}"
  export SLAM_PARAMS

  [ -f "$SLAM_PARAMS" ] || {
    echo "ERROR: localization params not found: $SLAM_PARAMS" >&2
    exit 1
  }

  map_file="$(sed -n 's/^[[:space:]]*map_file_name:[[:space:]]*"\{0,1\}\([^"]*\)"\{0,1\}[[:space:]]*$/\1/p' \
    "$SLAM_PARAMS" | head -n1)"
  [ -n "$map_file" ] || {
    echo "ERROR: no map_file_name in $SLAM_PARAMS" >&2
    exit 1
  }
  if [ ! -f "${map_file}.posegraph" ] || [ ! -f "${map_file}.data" ]; then
    echo "ERROR: posegraph not found: ${map_file}.posegraph / .data" >&2
    exit 1
  fi
  if ! grep -qE '^[[:space:]]*(map_start_pose|map_start_at_dock):' "$SLAM_PARAMS"; then
    echo "WARN: neither map_start_pose nor map_start_at_dock is set in $SLAM_PARAMS"
  fi
else
  SLAM_PARAMS="${SLAM_PARAMS:-$SCRIPT_DIR/slam_toolbox_rf2o.yaml}"
  export SLAM_PARAMS
  [ -f "$SLAM_PARAMS" ] || {
    echo "ERROR: mapping params not found: $SLAM_PARAMS" >&2
    exit 1
  }
fi

echo "== slam_toolbox =="
echo "   mode   : $MODE"
echo "   params : $SLAM_PARAMS"
echo "   domain : ${ROS_DOMAIN_ID:-99}"

exec "$SCRIPT_DIR/_start_slam.sh"
