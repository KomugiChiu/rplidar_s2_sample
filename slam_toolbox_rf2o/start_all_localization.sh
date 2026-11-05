#!/usr/bin/env bash
# Start the whole stack with slam_toolbox in LOCALIZATION mode
# (loads a previously saved posegraph; no mapping, no loop closure).
#
#   ./start_all_localization.sh
#   SLAM_PARAMS=/path/other_localization.yaml ./start_all_localization.sh
#   RECORD=1 ./start_all_localization.sh          # start the cmd_vel logger
#
# What runs:
#   rplidar S2     -> /scan
#   rf2o           -> /odom_rf2o + static TF chain
#   RealSense+EKF  -> /odometry/filtered + TF odom->base_footprint
#   slam_toolbox   -> /map + TF map->odom (localization_launch.py)
#   Nav2 + driver  -> nav2_params_plugin.yaml (DeadbandController)
#
# Requires a posegraph saved earlier with /slam_toolbox/serialize_map:
#   maps/<name>.posegraph + maps/<name>.data
# The posegraph path (without extension) is the `map_file_name` inside the yaml.
#
# IMPORTANT: the robot must be placed at `map_start_pose` (or the dock, if
# map_start_at_dock: true), otherwise localization starts from a wrong guess.
set -eo pipefail

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-99}"

SCRIPT_PATH="$(readlink -f -- "${BASH_SOURCE[0]:-$0}" 2>/dev/null || echo "${BASH_SOURCE[0]:-$0}")"
SCRIPT_DIR="$(cd -- "$(dirname -- "$SCRIPT_PATH")" && pwd)"

SLAM_PARAMS="${SLAM_PARAMS:-$SCRIPT_DIR/slam_toolbox_localization.yaml}"
NAV2_PARAMS_FILE="${NAV2_PARAMS_FILE:-${PARAMS_FILE:-$SCRIPT_DIR/nav2_1028/nav2_params_plugin.yaml}}"
unset PARAMS_FILE

[ -f "$SLAM_PARAMS" ] || {
    echo "ERROR: localization params not found: $SLAM_PARAMS"
    exit 1
}
[ -f "$NAV2_PARAMS_FILE" ] || {
    echo "ERROR: Nav2 params not found: $NAV2_PARAMS_FILE"
    exit 1
}

# --- validate the posegraph referenced by the yaml --------------------------
map_file="$(sed -n 's/^[[:space:]]*map_file_name:[[:space:]]*"\{0,1\}\([^"]*\)"\{0,1\}[[:space:]]*$/\1/p' \
    "$SLAM_PARAMS" | head -n1)"

if [ -z "$map_file" ]; then
    echo "ERROR: no map_file_name in $SLAM_PARAMS"
    echo "       add: map_file_name: \"$SCRIPT_DIR/maps/<name>\""
    exit 1
fi
if [ ! -f "${map_file}.posegraph" ] || [ ! -f "${map_file}.data" ]; then
    echo "ERROR: posegraph not found for map_file_name: $map_file"
    echo "       expected: ${map_file}.posegraph and ${map_file}.data"
    echo "       save it with:"
    echo "         ros2 service call /slam_toolbox/serialize_map \\"
    echo "           slam_toolbox/srv/SerializePoseGraph \"{filename: '$map_file'}\""
    exit 1
fi

if ! grep -qE '^[[:space:]]*(map_start_pose|map_start_at_dock):' "$SLAM_PARAMS"; then
    echo "WARN: neither map_start_pose nor map_start_at_dock is set in $SLAM_PARAMS"
fi

echo "== slam_toolbox localization =="
echo "   params  : $SLAM_PARAMS"
echo "   posegraph: ${map_file}.posegraph"
echo "   nav2    : $NAV2_PARAMS_FILE"

export SLAM_MODE=localization
export SLAM_PARAMS
export NAV2_PARAMS_FILE

exec "$SCRIPT_DIR/start_all.sh"
