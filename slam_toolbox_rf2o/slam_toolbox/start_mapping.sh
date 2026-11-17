#!/usr/bin/env bash
# Start RPLIDAR S2 + rf2o + slam_toolbox in MAPPING mode.
#
# This is a thin wrapper around ../start_slam_toolbox_rf2o.sh:
#   SLAM_MODE=mapping
#   SLAM_PARAMS=slam_toolbox_rf2o.yaml
#
# Usage:
#   ./start_mapping.sh
#   SLAM_PARAMS=/path/other_mapping.yaml ./start_mapping.sh
#   QUIET=0 ./start_mapping.sh
#
# Stop with: ./stop.sh
set -eo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]:-$0}")" && pwd)"
ROOT_DIR="$(cd -- "$SCRIPT_DIR/.." && pwd)"

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-99}"
export SLAM_MODE="mapping"
SLAM_PARAMS="${SLAM_PARAMS:-$ROOT_DIR/slam_toolbox_rf2o.yaml}"
export SLAM_PARAMS

[ -f "$SLAM_PARAMS" ] || {
  echo "ERROR: mapping params not found: $SLAM_PARAMS" >&2
  exit 1
}

echo "== slam_toolbox mapping =="
echo "   params : $SLAM_PARAMS"
echo "   domain : $ROS_DOMAIN_ID"

exec "$ROOT_DIR/start_slam_toolbox_rf2o.sh"
