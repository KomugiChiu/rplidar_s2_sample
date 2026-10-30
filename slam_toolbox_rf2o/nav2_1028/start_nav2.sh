#!/usr/bin/env bash
# Clean Nav2 integration for slam_toolbox_rf2o + ekf:
#   1. /nvt_ros2_example/slam_toolbox_rf2o/start_slam_toolbox_rf2o.sh
#        -> /scan, /odom_rf2o, /map, TF map->odom, base_footprint->base_link->laser
#   2. /nvt_ros2_example/slam_toolbox_rf2o/ekf/start_ekf.sh
#        -> D455 IMU, TF base_footprint->camera_link, /odometry/filtered, TF odom->base_footprint
#   3. This script (start_nav2.sh)
#        -> yahboomcar Mcnamu_driver (directly on /cmd_vel, no gate)
#        -> nav2_bringup (slam:=False use_localization:=False)
set -eo pipefail

# Every node of this stack must run in the same DDS domain.
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-99}"

ROS_DISTRO="${ROS_DISTRO:-jazzy}"
ROBOT_WS="${ROBOT_WS:-/root/raspybot/yahboomcar_ws}"
LIBRARY_WS="${LIBRARY_WS:-/root/raspybot/library_ws/py_install}"

SCRIPT_PATH="$(readlink -f -- "${BASH_SOURCE[0]:-$0}" 2>/dev/null || echo "${BASH_SOURCE[0]:-$0}")"
SCRIPT_DIR="$(cd -- "$(dirname -- "$SCRIPT_PATH")" && pwd)"

RUN_DIR="${RUN_DIR:-/tmp/raspybot_nav2}"
LOG_DIR="${LOG_DIR:-$RUN_DIR/logs}"
PID_FILE="${PID_FILE:-$RUN_DIR/pids}"

PARAMS_FILE="${PARAMS_FILE:-$SCRIPT_DIR/nav2_params.yaml}"
STOP_SCRIPT="${STOP_SCRIPT:-$SCRIPT_DIR/stop_nav2.sh}"
USE_SIM_TIME="${USE_SIM_TIME:-false}"
START_DRIVER="${START_DRIVER:-true}"
SKIP_CHECK="${SKIP_CHECK:-0}"
# minimum-command floor between collision_monitor and the chassis driver
FLOOR_SCRIPT="${FLOOR_SCRIPT:-$SCRIPT_DIR/tools/speed_floor.py}"
VX_FLOOR="${VX_FLOOR:-0.16}"
WZ_FLOOR="${WZ_FLOOR:-0.52}"
# floor sits BEFORE collision_monitor so the safety layer keeps the final say
FLOOR_INPUT="${FLOOR_INPUT:-/cmd_vel_smoothed}"
FLOOR_OUTPUT="${FLOOR_OUTPUT:-/cmd_vel_floor}"
# directional stop sits AFTER collision_monitor: only forward is blocked
DIR_STOP_SCRIPT="${DIR_STOP_SCRIPT:-$SCRIPT_DIR/tools/directional_stop.py}"
SAFE_TOPIC="${SAFE_TOPIC:-/cmd_vel_safe}"
STOP_DISTANCE="${STOP_DISTANCE:-0.30}"
# 1 = external speed_floor + directional_stop; 0 = controller plugin does it
USE_EXTERNAL_SAFETY="${USE_EXTERNAL_SAFETY:-1}"
# workspace containing nav2_deadband_controller (plugin mode)
PLUGIN_WS="${PLUGIN_WS:-/root/nav2_deadband_ws}"

mkdir -p "$LOG_DIR"

if [ ! -f "$PARAMS_FILE" ]; then
    echo "ERROR: Required file not found: $PARAMS_FILE"
    exit 1
fi

source "/opt/ros/$ROS_DISTRO/setup.bash"
if [ -f "$PLUGIN_WS/install/local_setup.bash" ]; then
    source "$PLUGIN_WS/install/local_setup.bash"
fi
if [ -f "$ROBOT_WS/install/local_setup.bash" ]; then
    source "$ROBOT_WS/install/local_setup.bash"
fi
export PYTHONPATH="$LIBRARY_WS:${PYTHONPATH:-}"
export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET
unset ROS_LOCALHOST_ONLY

# Preflight: ensure slam_toolbox_rf2o (/map) and ekf (/odometry/filtered + TF) are up
check_prerequisites() {
    local map_info odom_info tf_out
    map_info="$(timeout 10 ros2 topic info /map 2>/dev/null || true)"
    [[ "$map_info" =~ Publisher\ count:\ [1-9] ]] || return 1
    odom_info="$(timeout 10 ros2 topic info /odometry/filtered 2>/dev/null || true)"
    [[ "$odom_info" =~ Publisher\ count:\ [1-9] ]] || return 1
    tf_out="$(timeout 6 ros2 run tf2_ros tf2_echo map base_link 2>&1 || true)"
    [[ "$tf_out" == *Translation* ]] || return 1
    return 0
}

if [ "$SKIP_CHECK" != "1" ]; then
    CHECK_OK=0
    for attempt in 1 2 3; do
        if check_prerequisites; then
            CHECK_OK=1
            break
        fi
        echo "preflight: waiting for /map, /odometry/filtered, and TF map -> base_link ($attempt/3)..."
        sleep 3
    done
    if [ "$CHECK_OK" != "1" ]; then
        echo "ERROR: prerequisites not ready (/map, /odometry/filtered, or TF map -> base_link missing)."
        echo "       Please start:"
        echo "         1) /nvt_ros2_example/slam_toolbox_rf2o/start_slam_toolbox_rf2o.sh"
        echo "         2) /nvt_ros2_example/slam_toolbox_rf2o/ekf/start_ekf.sh"
        echo "       (or re-run with SKIP_CHECK=1)"
        exit 1
    fi
fi

if [ -s "$PID_FILE" ]; then
    while read -r pid name; do
        [ -n "${pid:-}" ] || continue
        if kill -0 "$pid" 2>/dev/null; then
            echo "ERROR: $name is already running with PID $pid"
            echo "       run $STOP_SCRIPT first"
            exit 1
        fi
    done < "$PID_FILE"
fi
: > "$PID_FILE"

start_process() {
    local name="$1"
    shift
    echo "Starting $name..."
    setsid bash -c '
        source "/opt/ros/'"$ROS_DISTRO"'/setup.bash"
        if [ -f "'"$PLUGIN_WS"'/install/local_setup.bash" ]; then
            source "'"$PLUGIN_WS"'/install/local_setup.bash"
        fi
        if [ -f "'"$ROBOT_WS"'/install/local_setup.bash" ]; then
            source "'"$ROBOT_WS"'/install/local_setup.bash"
        fi
        export PYTHONPATH="'"$LIBRARY_WS"':${PYTHONPATH:-}"
        export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET
        unset ROS_LOCALHOST_ONLY
        exec "$@"
    ' bash "$@" </dev/null >"$LOG_DIR/$name.log" 2>&1 &
    local pid=$!
    echo "$pid $name" >> "$PID_FILE"
    sleep 3
    if ! kill -0 "$pid" 2>/dev/null; then
        echo "ERROR: $name failed to start"
        tail -n 80 "$LOG_DIR/$name.log"
        "$STOP_SCRIPT" >/dev/null 2>&1 || true
        exit 1
    fi
    echo "$name started with PID $pid"
}

if [ "$START_DRIVER" = "true" ]; then
    # collision_monitor -> directional_stop -> driver
    DRIVER_TOPIC="/cmd_vel"
    if [ "$USE_EXTERNAL_SAFETY" != "1" ]; then
        echo "external speed_floor/directional_stop disabled (controller plugin mode)"
    elif [ -f "$FLOOR_SCRIPT" ]; then
        if pgrep -f "[s]peed_floor.py" >/dev/null 2>&1; then
            echo "speed_floor already running - skipping"
        else
            start_process speed_floor python3 "$FLOOR_SCRIPT" \
                --ros-args -p input_topic:="$FLOOR_INPUT" \
                           -p output_topic:="$FLOOR_OUTPUT" \
                           -p vx_floor:="$VX_FLOOR" \
                           -p wz_floor:="$WZ_FLOOR"
        fi
    else
        echo "WARN: $FLOOR_SCRIPT not found - no deadband compensation"
    fi
    if [ "$USE_EXTERNAL_SAFETY" = "1" ] && [ -f "$DIR_STOP_SCRIPT" ]; then
        DRIVER_TOPIC="$SAFE_TOPIC"
        if pgrep -f "[d]irectional_stop.py" >/dev/null 2>&1; then
            echo "directional_stop already running - skipping"
        else
            start_process directional_stop python3 "$DIR_STOP_SCRIPT" \
                --ros-args -p input_topic:=/cmd_vel \
                           -p output_topic:="$SAFE_TOPIC" \
                           -p stop_distance:="$STOP_DISTANCE"
        fi
    elif [ "$USE_EXTERNAL_SAFETY" = "1" ]; then
        echo "WARN: $DIR_STOP_SCRIPT not found - driver uses /cmd_vel"
    fi
    if pgrep -f "[M]cnamu_driver" >/dev/null 2>&1; then
        echo "yahboomcar driver already running - skipping"
    else
        start_process yahboomcar \
            ros2 run yahboomcar_bringup Mcnamu_driver \
                --ros-args -r cmd_vel:="$DRIVER_TOPIC"
    fi
fi

start_process nav2 \
    ros2 launch nav2_bringup bringup_launch.py \
        slam:=False \
        use_localization:=False \
        params_file:="$PARAMS_FILE" \
        use_sim_time:="$USE_SIM_TIME" \
        autostart:=True \
        use_composition:=False

echo
echo "Nav2 stack started."
echo "  params : $PARAMS_FILE"
echo "  odom   : /odometry/filtered (ekf)"
echo "  map    : /map (slam_toolbox)"
if [ "$USE_EXTERNAL_SAFETY" = "1" ]; then
    echo "  cmd_vel: $FLOOR_INPUT -> speed_floor (vx>=$VX_FLOOR, wz>=$WZ_FLOOR) -> $FLOOR_OUTPUT -> collision_monitor -> /cmd_vel -> directional_stop -> $SAFE_TOPIC -> driver"
else
    echo "  cmd_vel: controller plugin (DeadbandController + MPPI) -> velocity_smoother -> collision_monitor -> /cmd_vel -> driver"
fi
echo "  logs   : $LOG_DIR"
echo "Stop with: $STOP_SCRIPT"
