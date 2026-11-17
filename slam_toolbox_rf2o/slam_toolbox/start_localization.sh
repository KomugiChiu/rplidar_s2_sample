#!/usr/bin/env bash
# Start RPLIDAR S2 + rf2o + slam_toolbox in LOCALIZATION mode.
#
# This loads a previously saved posegraph. It does not build a new map.
#
# Usage:
#   ./start_localization.sh
#   SLAM_PARAMS=/path/other_localization.yaml ./start_localization.sh
#
# The posegraph path (without extension) is the map_file_name inside the yaml.
# A valid posegraph consists of:
#   <map_file_name>.posegraph
#   <map_file_name>.data
#
# Save one while running mapping mode:
#   ./save_map.sh <name>
#
# Stop with: ./stop.sh
set -eo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]:-$0}")" && pwd)"
ROOT_DIR="$(cd -- "$SCRIPT_DIR/.." && pwd)"

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-99}"
export SLAM_MODE="localization"
SLAM_PARAMS="${SLAM_PARAMS:-$ROOT_DIR/slam_toolbox_localization.yaml}"
export SLAM_PARAMS

[ -f "$SLAM_PARAMS" ] || {
  echo "ERROR: localization params not found: $SLAM_PARAMS" >&2
  exit 1
}

map_file="$(sed -n 's/^[[:space:]]*map_file_name:[[:space:]]*"\{0,1\}\([^"]*\)"\{0,1\}[[:space:]]*$/\1/p' \
  "$SLAM_PARAMS" | head -n1)"

if [ -z "$map_file" ]; then
  echo "ERROR: no map_file_name in $SLAM_PARAMS" >&2
  echo "       add: map_file_name: \"$ROOT_DIR/maps/<name>\"" >&2
  exit 1
fi
if [ ! -f "${map_file}.posegraph" ] || [ ! -f "${map_file}.data" ]; then
  echo "ERROR: posegraph not found for map_file_name: $map_file" >&2
  echo "       expected: ${map_file}.posegraph and ${map_file}.data" >&2
  echo "       save one while mapping with: ./save_map.sh <name>" >&2
  exit 1
fi

if ! grep -qE '^[[:space:]]*(map_start_pose|map_start_at_dock):' "$SLAM_PARAMS"; then
  echo "WARN: neither map_start_pose nor map_start_at_dock is set in $SLAM_PARAMS"
fi

echo "== slam_toolbox localization =="
echo "   params   : $SLAM_PARAMS"
echo "   posegraph: ${map_file}.posegraph"
echo "   domain   : $ROS_DOMAIN_ID"

exec "$ROOT_DIR/start_slam_toolbox_rf2o.sh"
