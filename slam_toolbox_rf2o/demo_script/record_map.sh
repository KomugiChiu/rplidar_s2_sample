#!/usr/bin/env bash
# Save the current slam_toolbox map + posegraph.
#
# This is a shortcut for ../slam_toolbox/save_map.sh.
# It must be used while start_mapping.sh is running (mapping mode).
#
# Usage:
#   ./record_map.sh                 # maps/slam_map_<timestamp>
#   ./record_map.sh demo_map        # maps/demo_map
#   MAP_DIR=/path ./record_map.sh x
set -eo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]:-$0}")" && pwd)"
ROOT_DIR="$(cd -- "$SCRIPT_DIR/.." && pwd)"

exec "$ROOT_DIR/slam_toolbox/save_map.sh" "$@"
