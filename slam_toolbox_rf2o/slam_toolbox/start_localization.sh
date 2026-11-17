#!/usr/bin/env bash
# Start RPLIDAR S2 + static laser TFs + slam_toolbox in LOCALIZATION mode.
#
# rf2o is NOT started here. Start it with ../rf2o/start.sh.
#
# Usage:
#   ./start_localization.sh
#   SLAM_PARAMS=/path/other_localization.yaml ./start_localization.sh
#
# The posegraph path (without extension) is map_file_name in the yaml:
#   <map_file_name>.posegraph
#   <map_file_name>.data
#
# Stop with: ./stop.sh
set -eo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]:-$0}")" && pwd)"

export SLAM_MODE="localization"
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

exec "$SCRIPT_DIR/_start_slam.sh"
