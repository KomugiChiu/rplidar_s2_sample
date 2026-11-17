#!/usr/bin/env bash
# Start RPLIDAR S2 + static laser TFs + slam_toolbox in MAPPING mode.
#
# rf2o is NOT started here. Start it with ../rf2o/start.sh.
#
# Usage:
#   ./start_mapping.sh
#   SLAM_PARAMS=/path/other_mapping.yaml ./start_mapping.sh
#   SKIP_LIDAR=1 ./start_mapping.sh    # reuse an existing rplidar_node
#
# Stop with: ./stop.sh
set -eo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]:-$0}")" && pwd)"

export SLAM_MODE="mapping"
SLAM_PARAMS="${SLAM_PARAMS:-$SCRIPT_DIR/slam_toolbox_rf2o.yaml}"
export SLAM_PARAMS

exec "$SCRIPT_DIR/_start_slam.sh"
