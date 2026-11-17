#!/usr/bin/env bash
# Start the full person-follow demo in MAPPING mode.
#
# Starts, in order:
#   rplidar S2 + rf2o + slam_toolbox (mapping)
#   RealSense + EKF
#   Nav2 + chassis driver
#   AI detection + person_target_recovery + demo_manager
#
# This script intentionally does NOT call ../start_all.sh or ../start_ai.sh.
# It orchestrates the lower-level start scripts and the vendor helpers directly.
#
# Usage:
#   ./start_demo.sh
#   START_AI=0 ./start_demo.sh
#   RECORD=1 ./start_demo.sh
#   NAV2_PARAMS_FILE=$NAV2_DIR/nav2_params_plugin.yaml ./start_demo.sh
#
# Stop with: ../stop_demo.sh
set -eo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]:-$0}")" && pwd)"
# shellcheck source=demo_common.sh
source "$SCRIPT_DIR/demo_common.sh"

export SLAM_MODE="mapping"
SLAM_PARAMS="${SLAM_PARAMS:-$SLAM_DIR/slam_toolbox_rf2o.yaml}"
export SLAM_PARAMS

[ -f "$SLAM_PARAMS" ] || demo_die "mapping params not found: $SLAM_PARAMS"

demo_preflight
demo_check_not_running

echo "== start_demo (mapping) =="
echo "   domain : $ROS_DOMAIN_ID"
echo "   slam   : $SLAM_PARAMS"
echo "   nav2   : $NAV2_PARAMS_FILE"

demo_start_slam
demo_start_ekf
demo_wait_for_slam
demo_start_nav2
demo_start_ai
demo_maybe_record
demo_print_done
