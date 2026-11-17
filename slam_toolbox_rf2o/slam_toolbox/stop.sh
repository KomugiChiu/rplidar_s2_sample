#!/usr/bin/env bash
# Stop the RPLIDAR S2 + rf2o + slam_toolbox processes started by this folder.
#
# Usage:
#   ./stop.sh
#   DRY_RUN=1 ./stop.sh
#   KEEP_DRIVER=1 ./stop.sh     # keep rplidar_node if owned by someone else
set -eo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]:-$0}")" && pwd)"
ROOT_DIR="$(cd -- "$SCRIPT_DIR/.." && pwd)"

exec "$ROOT_DIR/stop_slam_toolbox_rf2o.sh" "$@"
