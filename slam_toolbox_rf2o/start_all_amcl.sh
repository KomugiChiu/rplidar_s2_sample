#!/usr/bin/env bash
# Start the whole stack in STATIC MAP + AMCL mode (no slam_toolbox).
#
#   ./start_all_amcl.sh
#   MAP_FILE=/path/other_map.yaml ./start_all_amcl.sh
#   NAV2_PARAMS_FILE=.../nav2_params_amcl.yaml ./start_all_amcl.sh
#   RECORD=0 ./start_all_amcl.sh          # do not start the cmd_vel logger
#
# What runs:
#   rplidar S2     -> /scan
#   rf2o           -> /odom_rf2o (unused by Nav2) + static TF chain
#   RealSense+EKF  -> /odometry/filtered + TF odom->base_footprint
#   map_server     -> /map          (static map, transient local)
#   AMCL           -> TF map->odom  (initial pose 0,0,0 = mapping start pose)
#   Nav2 + driver  -> nav2_params_amcl.yaml (DeadbandController, same as plugin flow)
#
# The map must have been saved from a slam_toolbox session; see nav2_1028.md §14.
# IMPORTANT: place the robot at the same spot/heading it had when that map was
# started, otherwise re-localize from RViz with "2D Pose Estimate"
# (or publish geometry_msgs/PoseWithCovarianceStamped on /initialpose).
set -eo pipefail

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-99}"

SCRIPT_PATH="$(readlink -f -- "${BASH_SOURCE[0]:-$0}" 2>/dev/null || echo "${BASH_SOURCE[0]:-$0}")"
SCRIPT_DIR="$(cd -- "$(dirname -- "$SCRIPT_PATH")" && pwd)"

MAP_FILE="${MAP_FILE:-$SCRIPT_DIR/maps/slam_map_20261102.yaml}"
NAV2_PARAMS_FILE="${NAV2_PARAMS_FILE:-$SCRIPT_DIR/nav2_1028/nav2_params_amcl.yaml}"

[ -f "$MAP_FILE" ] || {
    echo "ERROR: map yaml not found: $MAP_FILE"
    echo "       save one with: ros2 run nav2_map_server map_saver_cli -f $SCRIPT_DIR/maps/<name>"
    exit 1
}
[ -f "${MAP_FILE%.yaml}.pgm" ] || [ -f "${MAP_FILE%.yml}.pgm" ] || {
    echo "ERROR: map image (.pgm) next to $MAP_FILE not found"
    exit 1
}
[ -f "$NAV2_PARAMS_FILE" ] || {
    echo "ERROR: params file not found: $NAV2_PARAMS_FILE"
    exit 1
}

# slam_toolbox must NOT run: AMCL publishes map->odom in this mode.
export SLAM_MODE=none
export USE_LOCALIZATION=1
export MAP_FILE
export NAV2_PARAMS_FILE

echo "== static map + AMCL =="
echo "   map   : $MAP_FILE"
echo "   params: $NAV2_PARAMS_FILE"

exec "$SCRIPT_DIR/start_all.sh"
