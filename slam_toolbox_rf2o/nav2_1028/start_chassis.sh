#!/usr/bin/env bash
# Start ONLY the Yahboomcar chassis driver (Mcnamu_driver), without Nav2.
#
#   /cmd_vel -> Mcnamu_driver (no gate, no remap)
#
# Usage (board):
#   ./start_chassis.sh
#   START_DRIVER=... (not used here)
# Stop with: ./stop_chassis.sh
set -eo pipefail

# Keep the whole robot stack on the same DDS domain.
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-99}"

ROS_DISTRO="${ROS_DISTRO:-jazzy}"
ROBOT_WS="${ROBOT_WS:-/root/raspybot/yahboomcar_ws}"
LIBRARY_WS="${LIBRARY_WS:-/root/raspybot/library_ws/py_install}"

SCRIPT_PATH="$(readlink -f -- "${BASH_SOURCE[0]:-$0}" 2>/dev/null || echo "${BASH_SOURCE[0]:-$0}")"
SCRIPT_DIR="$(cd -- "$(dirname -- "$SCRIPT_PATH")" && pwd)"
STOP_SCRIPT="$SCRIPT_DIR/stop_chassis.sh"

RUN_DIR="${RUN_DIR:-/tmp/nav2_chassis}"
LOG_DIR="${LOG_DIR:-$RUN_DIR/logs}"
PID_FILE="${PID_FILE:-$RUN_DIR/pids}"
LOG_FILE="$LOG_DIR/chassis.log"

mkdir -p "$LOG_DIR"

if [ ! -f "$ROBOT_WS/install/local_setup.bash" ]; then
    echo "ERROR: yahboomcar workspace not found: $ROBOT_WS/install/local_setup.bash"
    exit 1
fi

source "/opt/ros/$ROS_DISTRO/setup.bash"
source "$ROBOT_WS/install/local_setup.bash"
export PYTHONPATH="$LIBRARY_WS:${PYTHONPATH:-}"
export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET
unset ROS_LOCALHOST_ONLY

# Refuse a second instance.
if pgrep -f "[M]cnamu_driver" >/dev/null 2>&1; then
    echo "chassis driver already running - leaving it alone"
    pgrep -af "[M]cnamu_driver" | head -n 3
    exit 0
fi

setsid ros2 launch yahboomcar_bringup bringup.launch.py \
    </dev/null >"$LOG_FILE" 2>&1 &
PID=$!
echo "$PID chassis group" > "$PID_FILE"

sleep 3
if ! kill -0 "$PID" 2>/dev/null; then
    echo "ERROR: chassis driver failed to start"
    tail -n 40 "$LOG_FILE" || true
    rm -f "$PID_FILE"
    exit 1
fi

echo "chassis driver started"
echo "  pid     : $PID"
echo "  cmd_vel : /cmd_vel (direct, no remap)"
echo "  log     : $LOG_FILE"
echo "stop with: $STOP_SCRIPT"
