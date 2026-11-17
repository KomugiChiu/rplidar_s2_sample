#!/usr/bin/env bash
# Mapping-only entry point: RPLIDAR S2 + rf2o + slam_toolbox.
#
# This is a shortcut for ../slam_toolbox/start_mapping.sh.
# It does NOT start EKF, Nav2, AI, or the chassis driver.
#
# For the full mapping demo (SLAM + EKF + Nav2 + AI), use:
#   ./start_demo.sh
#
# Usage:
#   ./start_mapping.sh
#   SLAM_PARAMS=/path/other_mapping.yaml ./start_mapping.sh
#
# Stop the SLAM stack with:
#   ../slam_toolbox/stop.sh
set -eo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]:-$0}")" && pwd)"
ROOT_DIR="$(cd -- "$SCRIPT_DIR/.." && pwd)"

exec "$ROOT_DIR/slam_toolbox/start_mapping.sh" "$@"
