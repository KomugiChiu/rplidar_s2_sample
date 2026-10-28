#!/usr/bin/env bash
# Nav2 for the slam_toolbox + rf2o flow.
#
#   start_slam_toolbox_rf2o.sh provides:
#       /scan                       (rplidar S2)
#       /odom_rf2o                  (rf2o laser odometry)
#       /map                        (slam_toolbox, transient local)
#       map -> odom -> base_footprint -> base_link -> laser   (TF)
#
#   nav2 (this script) is launched with
#       slam:=False use_localization:=False
#   so NEITHER slam_toolbox NOR amcl NOR map_server is started: the map and
#   map->odom come from the running slam_toolbox, the odometry from rf2o
#   (nav2_params_rf2o.yaml: odom_topic=/odom_rf2o, static layer reads /map).
#
# The stack is started in the background: this script returns immediately and
# does not occupy the console.  Stop it with stop_nav2_rf2o.sh.
#
# Usage (on board):
#   ./start_nav2_rf2o.sh                       # normal start (nav2 may rotate)
#   START_DRIVER=0 ./start_nav2_rf2o.sh        # do not start the chassis driver
#   TRANSLATION_ONLY=1 ./start_nav2_rf2o.sh    # strip angular.z again (no rotation)
#   SKIP_CHECK=1 ./start_nav2_rf2o.sh          # skip the /map + TF preflight
#   ./stop_nav2_rf2o.sh                        # stop everything
set -eo pipefail   # NOTE: no -u (ROS setup files need unset vars)

ROS_DISTRO="${ROS_DISTRO:-jazzy}"
ROBOT_WS="${ROBOT_WS:-/root/raspybot/yahboomcar_ws}"
LIBRARY_WS="${LIBRARY_WS:-/root/raspybot/library_ws/py_install}"

SCRIPT_PATH="$(readlink -f -- "${BASH_SOURCE[0]:-$0}" 2>/dev/null || echo "${BASH_SOURCE[0]:-$0}")"
SCRIPT_DIR="$(cd -- "$(dirname -- "$SCRIPT_PATH")" && pwd)"

RUN_DIR="${RUN_DIR:-/tmp/slam_toolbox_rf2o_nav2}"
LOG_DIR="$RUN_DIR/logs"
PID_FILE="${PID_FILE:-$RUN_DIR/pids}"
PARAMS_FILE="${PARAMS_FILE:-$SCRIPT_DIR/nav2_params_rf2o.yaml}"
STOP_SCRIPT="${STOP_SCRIPT:-$SCRIPT_DIR/stop_nav2_rf2o.sh}"
GATE_SCRIPT="${GATE_SCRIPT:-$SCRIPT_DIR/translation_only.py}"

USE_SIM_TIME="${USE_SIM_TIME:-false}"
START_DRIVER="${START_DRIVER:-true}"
# TRANSLATION_ONLY=true -> the chassis may only translate: forward/backward
# and sideways (mecanum), never a rotation.  A small node sits between the
# collision monitor and the driver and forces the angular velocity to zero.
# This is the default because the chassis MCU can only rotate in >=0.4 s
# bursts (11-15 deg per burst), so closed-loop yaw control oscillates.
# TRANSLATION_ONLY=false -> the same node passes angular.z through (capped at
# WZ_MAX_OUT) so nav2 may turn; it still acts as the cmd_vel watchdog.
TRANSLATION_ONLY="${TRANSLATION_ONLY:-${STRAIGHT_ONLY:-true}}"
# measured chassis gain compensation (wheels ~1.2x faster than commanded)
VX_SCALE="${VX_SCALE:-0.85}"
# rotation command cap when TRANSLATION_ONLY=false (MCU soft-start limit)
WZ_MAX_OUT="${WZ_MAX_OUT:-0.20}"
# strafing is off by default: the single forward ultrasonic is blind to the sides
ALLOW_STRAFE="${ALLOW_STRAFE:-false}"
# safety stops in metres (gate defaults are the same)
SONAR_STOP="${SONAR_STOP:-0.65}"
SCAN_STOP="${SCAN_STOP:-0.45}"
# enable the laser forward-sector safety stop (tall obstacles).  Set
# SCAN_STOP_ENABLED=false to exclude it (testing only).
SCAN_STOP_ENABLED="${SCAN_STOP_ENABLED:-true}"
# enable the forward ultrasonic safety stop (low obstacles invisible to the 2D
# lidar).  Set USE_ULTRASONIC=false to exclude it (testing only).
USE_ULTRASONIC="${USE_ULTRASONIC:-true}"
# zero the chassis command after this long without nav2 input (s).  The
# smoother's velocity_timeout already stops 0.1 s after the upstream stops;
# 0.2 s here is ~4 control periods and adds only ~11 deg at 1 rad/s.
WATCHDOG_TIMEOUT="${WATCHDOG_TIMEOUT:-0.1}"
GATE_TOPIC="${GATE_TOPIC:-/cmd_vel_straight}"
# gate input topic (default /cmd_vel; use /cmd_vel_smoothed to bypass the
# collision_monitor, which publishes at only ~5 Hz on this board)
GATE_INPUT="${GATE_INPUT:-/cmd_vel}"
SKIP_CHECK="${SKIP_CHECK:-0}"

mkdir -p "$LOG_DIR"
[ -f "$PARAMS_FILE" ] || { echo "ERROR: params file not found: $PARAMS_FILE"; exit 1; }
[ -f "$GATE_SCRIPT" ] || { echo "WARN: gate script not found: $GATE_SCRIPT (TRANSLATION_ONLY ignored)"; TRANSLATION_ONLY=false; }

source "/opt/ros/$ROS_DISTRO/setup.bash"
if [ -f "$ROBOT_WS/install/local_setup.bash" ]; then
  source "$ROBOT_WS/install/local_setup.bash"
fi
export PYTHONPATH="$LIBRARY_WS:${PYTHONPATH:-}"
export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET
unset ROS_LOCALHOST_ONLY

# --- preflight: the SLAM flow has to be up ------------------------------------
# The first ros2 CLI call on this board has to start the discovery daemon and
# a fresh tf2_echo needs a moment to fill its buffer, so every attempt is
# retried a few times before giving up.
check_slam_once() {
  local info tf_out
  # NOTE: no `| grep -q` here: grep -q exits after the first match, the writer
  # gets SIGPIPE and `set -o pipefail` would report the check as failed.
  info="$(timeout 15 ros2 topic info /map 2>/dev/null || true)"
  [[ "$info" =~ Publisher\ count:\ [1-9] ]] || return 1
  tf_out="$(timeout 6 ros2 run tf2_ros tf2_echo map base_link 2>&1 || true)"
  [[ "$tf_out" == *Translation* ]] || return 1
  return 0
}
if [ "$SKIP_CHECK" != "1" ]; then
  CHECK_OK=0
  for attempt in 1 2 3; do
    if check_slam_once; then CHECK_OK=1; break; fi
    echo "preflight: waiting for /map and TF map -> base_link ($attempt/3)"
    sleep 3
  done
  if [ "$CHECK_OK" != "1" ]; then
    echo "ERROR: /map has no publisher and/or TF map -> base_link is not available"
    echo "       start /nvt_ros2_example/slam_toolbox_rf2o/start_slam_toolbox_rf2o.sh first"
    echo "       (or re-run with SKIP_CHECK=1)"
    exit 1
  fi
fi

# --- refuse a second instance -------------------------------------------------
if [ -s "$PID_FILE" ]; then
  while read -r pid name mode; do
    [ -n "${pid:-}" ] || continue
    if kill -0 "$pid" 2>/dev/null; then
      echo "ERROR: $name is already running with PID $pid"
      echo "       run $STOP_SCRIPT first"
      exit 1
    fi
  done < "$PID_FILE"
fi
: > "$PID_FILE"

# Start a process in its own session (survives this script, keeps the console
# free) and remember it as "pid name group" for stop_nav2_rf2o.sh.
start_process() {
  local name="$1"
  shift
  echo "Starting $name..."
  setsid bash -c '
      source "/opt/ros/'"$ROS_DISTRO"'/setup.bash"
      source "'"$ROBOT_WS"'/install/local_setup.bash"
      export PYTHONPATH="'"$LIBRARY_WS"':${PYTHONPATH:-}"
      exec "$@"
    ' bash "$@" </dev/null >"$LOG_DIR/$name.log" 2>&1 &
  local pid=$!
  echo "$pid $name group" >> "$PID_FILE"
  sleep 3
  if ! kill -0 "$pid" 2>/dev/null; then
    echo "ERROR: $name failed to start"
    tail -n 40 "$LOG_DIR/$name.log"
    "$STOP_SCRIPT" >/dev/null 2>&1 || true
    exit 1
  fi
  echo "  $name started with PID $pid"
}

# --- chassis driver -----------------------------------------------------------
if [ "$START_DRIVER" = "true" ]; then
  if pgrep -f "[M]cnamu_driver" >/dev/null 2>&1; then
    echo "chassis driver already running - leaving it alone"
  elif [ -f "$GATE_SCRIPT" ]; then
    # The gate always sits between nav2 and the driver: besides stripping or
    # passing the angular velocity it is the cmd_vel watchdog (the driver has
    # no timeout of its own).  (ros2 run instead of ros2 launch: "ros2 launch"
    # in jazzy does not accept --ros-args; the yahboomcar launch sets no
    # parameters.)
    if [ "$TRANSLATION_ONLY" = "true" ]; then
      start_process translation_gate python3 "$GATE_SCRIPT" \
        --ros-args -p input_topic:="$GATE_INPUT" \
                   -p vx_scale:="$VX_SCALE" \
                   -p allow_strafe:="$ALLOW_STRAFE" \
                   -p ultrasonic_stop_distance:="$SONAR_STOP" \
                   -p use_ultrasonic:="$USE_ULTRASONIC" \
                   -p scan_stop_distance:="$SCAN_STOP" \
                   -p use_scan_stop:="$SCAN_STOP_ENABLED" \
                   -p watchdog_timeout:="$WATCHDOG_TIMEOUT"
    else
      start_process translation_gate python3 "$GATE_SCRIPT" \
        --ros-args -p input_topic:="$GATE_INPUT" \
                   -p allow_rotation:=true -p vx_scale:="$VX_SCALE" \
                   -p wz_max_out:="$WZ_MAX_OUT" \
                   -p allow_strafe:="$ALLOW_STRAFE" \
                   -p ultrasonic_stop_distance:="$SONAR_STOP" \
                   -p use_ultrasonic:="$USE_ULTRASONIC" \
                   -p scan_stop_distance:="$SCAN_STOP" \
                   -p use_scan_stop:="$SCAN_STOP_ENABLED" \
                   -p watchdog_timeout:="$WATCHDOG_TIMEOUT"
    fi
    start_process yahboomcar \
      ros2 run yahboomcar_bringup Mcnamu_driver \
        --ros-args -r "cmd_vel:=$GATE_TOPIC"
  else
    start_process yahboomcar \
      ros2 run yahboomcar_bringup Mcnamu_driver
  fi
fi

# --- nav2 (no slam_toolbox / amcl / map_server) -------------------------------
start_process nav2 \
  ros2 launch nav2_bringup bringup_launch.py \
    slam:=False \
    use_localization:=False \
    params_file:="$PARAMS_FILE" \
    use_sim_time:="$USE_SIM_TIME" \
    autostart:=True \
    use_composition:=False

echo
echo "Nav2 (slam_toolbox + rf2o) started in the background:"
echo "  params : $PARAMS_FILE"
echo "  odom   : /odom_rf2o (rf2o)"
echo "  map    : /map (slam_toolbox)"
echo "  logs   : $LOG_DIR"
echo "stop it with: $STOP_SCRIPT"
