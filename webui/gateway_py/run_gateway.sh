#!/usr/bin/env bash
# Environment for the Python gateway; started (and restarted) by robot-console
# when launched with ../start_console_py.sh.
# stdout carries binary frames to Go - nothing else may print to it.
set -eo pipefail   # NOTE: no -u (ROS setup files need unset vars)

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-99}"
export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET
unset ROS_LOCALHOST_ONLY

DEMO_WS="${DEMO_WS:-/root/ros2_ws}"
AI_WS="${AI_WS:-/nvt_ros2_example/ai_human_detection}"
SCRIPT_DIR="$(cd -- "$(dirname -- "$(readlink -f -- "${BASH_SOURCE[0]}")")" && pwd)"

source /opt/ros/jazzy/setup.bash >&2
# ai_human_detection/PersonInfoArray python messages; local_setup does not run
# the apt block of the vendor setup.bash
if [ -f "$DEMO_WS/install/local_setup.bash" ]; then source "$DEMO_WS/install/local_setup.bash" >&2; fi
if [ -f "$AI_WS/local_setup.bash" ]; then source "$AI_WS/local_setup.bash" >&2; fi

# lower priority than the navigation stack
exec nice -n 10 python3 "$SCRIPT_DIR/robot_console_gateway.py"
