#!/usr/bin/env bash
# Start the full person-follow demo with slam_toolbox LOCALIZATION mode.
#
# Starts, in order:
#   rplidar S2 + rf2o + slam_toolbox (localization, loads an existing posegraph)
#   RealSense + EKF
#   Nav2 + chassis driver
#   AI detection + person_target_recovery + demo_manager
#
# This script intentionally does NOT call ../start_all_localization.sh,
# ../start_all.sh or ../start_ai.sh. It orchestrates the lower-level start
# scripts and the vendor helpers directly.
#
# Requirement:
#   <map_file_name>.posegraph and <map_file_name>.data must exist.
#   <map_file_name> is read from slam_toolbox_localization.yaml.
#
# Usage:
#   ./start_demo_static.sh
#   SLAM_PARAMS=../slam_toolbox_localization.yaml ./start_demo_static.sh
#   START_AI=0 ./start_demo_static.sh
#
# Stop with: ../stop_demo.sh
set -eo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]:-$0}")" && pwd)"
# shellcheck source=demo_common.sh
source "$SCRIPT_DIR/demo_common.sh"

export SLAM_MODE="localization"
SLAM_PARAMS="${SLAM_PARAMS:-$ROOT_DIR/slam_toolbox_localization.yaml}"
export SLAM_PARAMS

[ -f "$SLAM_PARAMS" ] || demo_die "localization params not found: $SLAM_PARAMS"
demo_validate_posegraph

demo_preflight
demo_check_not_running

echo "== start_demo_static (localization) =="
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
