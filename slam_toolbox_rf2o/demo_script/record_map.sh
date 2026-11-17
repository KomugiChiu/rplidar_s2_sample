#!/usr/bin/env bash
# Save the current slam_toolbox map + posegraph.
#
# This is a shortcut for $SLAM_DIR/save_map.sh.
# It must be used while start_mapping.sh is running (mapping mode).
#
# Usage:
#   ./record_map.sh                 # maps/slam_map_<timestamp>
#   ./record_map.sh demo_map        # maps/demo_map
#   MAP_DIR=/path ./record_map.sh x
set -eo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]:-$0}")" && pwd)"
SLAM_DIR="${SLAM_DIR:-/nvt_ros2_example/slam_toolbox}"

exec "$SLAM_DIR/save_map.sh" "$@"
