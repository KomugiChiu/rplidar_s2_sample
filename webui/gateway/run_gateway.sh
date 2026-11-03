#!/usr/bin/env bash
# Environment for robot_console_gateway; started (and restarted) by robot-console.
# stdout carries binary frames to Go - nothing else may print to it.
set -eo pipefail   # NOTE: no -u (ROS setup files need unset vars)

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-99}"
export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET
unset ROS_LOCALHOST_ONLY

DEMO_WS="${DEMO_WS:-/root/ros2_ws}"
AI_WS="${AI_WS:-/nvt_ros2_example/ai_human_detection}"
SCRIPT_DIR="$(cd -- "$(dirname -- "$(readlink -f -- "${BASH_SOURCE[0]}")")" && pwd)"

source /opt/ros/jazzy/setup.bash >&2
# message type support only (ai_human_detection/PersonInfoArray is decoded at
# runtime); local_setup does not run the apt block of the vendor setup.bash
if [ -f "$DEMO_WS/install/local_setup.bash" ]; then source "$DEMO_WS/install/local_setup.bash" >&2; fi
if [ -f "$AI_WS/local_setup.bash" ]; then source "$AI_WS/local_setup.bash" >&2; fi

BIN="$SCRIPT_DIR/build/robot_console_gateway"
if [ ! -x "$BIN" ]; then
  echo "gateway binary missing: run $SCRIPT_DIR/build.sh" >&2
  exit 1
fi
exec "$BIN"
