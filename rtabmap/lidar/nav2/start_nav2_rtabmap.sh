#!/usr/bin/env bash
# Nav2 that consumes rtabmap's map.
#
#   rtabmap (start_rtabmap_laser.sh) provides:
#       /rtabmap/map          OccupancyGrid (transient local)
#       map -> odom           TF (rtabmap SLAM node)
#       odom -> base_footprint  TF (icp_odometry)
#
#   nav2 (this script) is launched with
#       slam:=False use_localization:=False
#   so NEITHER slam_toolbox NOR amcl NOR map_server is started: the global
#   costmap's static layer takes the map directly from /rtabmap/map
#   (see nav2_params_rtabmap.yaml).
#
# The stack is started in the background: this script returns immediately and
# does not occupy the console.  Stop it with stop_nav2.sh.
#
# Usage (on board):
#   ./start_nav2_rtabmap.sh
#   START_DRIVER=0 ./start_nav2_rtabmap.sh      # do not start the chassis driver
#   ./stop_nav2.sh                              # stop everything
set -eo pipefail

ROS_DISTRO="jazzy"
ROBOT_WS="/root/raspybot/yahboomcar_ws"
LIBRARY_WS="/root/raspybot/library_ws/py_install"
RUN_DIR="/tmp/raspybot_nav2"          # shared with stop_nav2.sh
LOG_DIR="$RUN_DIR/logs"
PID_FILE="$RUN_DIR/pids"

# Override these when starting, if needed.
PARAMS_FILE="${PARAMS_FILE:-/nvt_ros2_example/nav2/nav2_params_rtabmap.yaml}"
USE_SIM_TIME="${USE_SIM_TIME:-false}"
START_DRIVER="${START_DRIVER:-true}"

mkdir -p "$LOG_DIR"

source "/opt/ros/$ROS_DISTRO/setup.bash"
source "$ROBOT_WS/install/local_setup.bash"
export PYTHONPATH="$LIBRARY_WS:${PYTHONPATH:-}"

# Also export for Rviz
export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET
unset ROS_LOCALHOST_ONLY

[ -f "$PARAMS_FILE" ] || { echo "ERROR: params file not found: $PARAMS_FILE"; exit 1; }

if [ -s "$PID_FILE" ]; then
    while read -r pid name; do
        if kill -0 "$pid" 2>/dev/null; then
            echo "ERROR: $name is already running with PID $pid"
            echo "       run stop_nav2.sh first"
            exit 1
        fi
    done < "$PID_FILE"
fi
: > "$PID_FILE"

# Start a process in its own session (survives this script, keeps the console
# free) and remember it, so stop_nav2.sh can kill the whole group.
start_process() {
    local name="$1"
    shift
    echo "Starting $name..."
    setsid bash -c '
        source "/opt/ros/jazzy/setup.bash"
        source "/root/raspybot/yahboomcar_ws/install/local_setup.bash"
        export PYTHONPATH="/root/raspybot/library_ws/py_install:${PYTHONPATH:-}"
        exec "$@"
    ' bash "$@" >"$LOG_DIR/$name.log" 2>&1 &
    local pid=$!
    echo "$pid $name" >> "$PID_FILE"
    sleep 3
    if ! kill -0 "$pid" 2>/dev/null; then
        echo "ERROR: $name failed to start"
        tail -n 80 "$LOG_DIR/$name.log"
        /nvt_ros2_example/nav2/stop_nav2.sh >/dev/null 2>&1 || true
        exit 1
    fi
    echo "$name started with PID $pid"
}

if [ "$START_DRIVER" = "true" ]; then
    start_process yahboomcar \
        ros2 launch yahboomcar_bringup bringup.launch.py
fi

# slam:=False use_localization:=False -> no slam_toolbox, no amcl, no
# map_server.  The map comes from rtabmap (/rtabmap/map).
start_process nav2 \
    ros2 launch nav2_bringup bringup_launch.py \
        slam:=False \
        use_localization:=False \
        params_file:="$PARAMS_FILE" \
        use_sim_time:="$USE_SIM_TIME" \
        autostart:=True \
        use_composition:=False

echo
echo "Nav2 (rtabmap map) started in the background."
echo "  params : $PARAMS_FILE"
echo "  logs   : $LOG_DIR"
echo "  map    : /rtabmap/map  (from rtabmap)"
echo "Stop it with: /nvt_ros2_example/nav2/stop_nav2.sh"
