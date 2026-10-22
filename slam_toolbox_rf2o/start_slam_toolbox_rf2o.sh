#!/usr/bin/env bash
# Laser 2D mapping on board: RPLIDAR S2 (/scan) + rf2o laser odometry + slam_toolbox.
# Same skeleton as start_rtabmap_laser.sh:
#   - every started process is recorded in $PID_FILE  -> stop_slam_toolbox_rf2o.sh
#   - QUIET / NO_LOG / LOG_LEVEL / LOG_DIR control the console output
#   - the script may be started from any working directory
#
# Division of labour:
#   rplidar_ros   -> /scan
#   rf2o          -> /odom_rf2o  +  odom -> base_footprint   (dynamic)
#   slam_toolbox  -> /map         +  map   -> odom            (dynamic)
#   static tf     -> base_footprint -> base_link -> laser
#
# Usage (on board):
#   ./start_slam_toolbox_rf2o.sh                       # map only (runs in background)
#   QUIET=0 ./start_slam_toolbox_rf2o.sh               # show the node output in the console
#   SLAM_MODE=localization ./start_slam_toolbox_rf2o.sh
#   ./stop_slam_toolbox_rf2o.sh                        # stop everything
# Defaults: LASER_YAW_DEG=180, QUIET=1.  The stack is started in the
# background: this script returns immediately and the console stays free.
set -eo pipefail   # NOTE: no -u (ROS setup files need unset vars)

SCAN_TOPIC="${SCAN_TOPIC:-/scan}"
LASER_FRAME="${LASER_FRAME:-laser}"
BASE_FRAME="${BASE_FRAME:-base_footprint}"
BASE_LINK_FRAME="${BASE_LINK_FRAME:-base_link}"
BASE_LINK_Z="${BASE_LINK_Z:-0.10}"
LASER_Z="${LASER_Z:-0.20}"
SERIAL_PORT="${SERIAL_PORT:-/dev/ttyUSB0}"
ANGLE_COMPENSATE="${ANGLE_COMPENSATE:-true}"
SCAN_MODE="${SCAN_MODE:-DenseBoost}"
RPLIDAR_WS="${RPLIDAR_WS:-/nvt_ros2_example/rplidar_ros/}"
RF2O_WS="${RF2O_WS:-/root/rf2o_ws}"
SKIP_LIDAR="${SKIP_LIDAR:-0}"
SLAM_MODE="${SLAM_MODE:-mapping}"          # mapping | localization
ODOM_TOPIC="${ODOM_TOPIC:-/odom_rf2o}"
VO_FRAME_ID="${VO_FRAME_ID:-odom}"
# --- logging ---
QUIET="${QUIET:-1}"
NO_LOG="${NO_LOG:-0}"
LOG_LEVEL="${LOG_LEVEL:-}"
LOG_DIR="${LOG_DIR:-/tmp}"
RUN_DIR="${RUN_DIR:-/tmp/slam_toolbox_rf2o}"
PID_FILE="${PID_FILE:-$RUN_DIR/pids}"
if [ "$QUIET" = "1" ] && [ -z "$LOG_LEVEL" ]; then LOG_LEVEL=warn; fi
if [ "$NO_LOG" = "1" ]; then
  QUIET=1
  LOG_FILE=/dev/null
  LIDAR_LOG=/dev/null
  TF_BASE_LOG=/dev/null
  TF_LASER_LOG=/dev/null
  RF2O_LOG=/dev/null
  SLAM_LOG=/dev/null
else
  mkdir -p "$LOG_DIR"
  LOG_FILE="${LOG_FILE:-$LOG_DIR/slam_toolbox.log}"
  LIDAR_LOG="$LOG_DIR/lidar_rplidar.log"
  TF_BASE_LOG="$LOG_DIR/lidar_static_base_link.log"
  TF_LASER_LOG="$LOG_DIR/lidar_static_tf.log"
  RF2O_LOG="$LOG_DIR/lidar_rf2o.log"
  SLAM_LOG="$LOG_DIR/lidar_slam_toolbox.log"
fi
mkdir -p "$RUN_DIR"
# Absolute path of this script (symlinks resolved) and of its directory.
SCRIPT_PATH="$(readlink -f -- "${BASH_SOURCE[0]:-$0}" 2>/dev/null || echo "${BASH_SOURCE[0]:-$0}")"
SCRIPT_DIR="$(cd -- "$(dirname -- "$SCRIPT_PATH")" && pwd)"
STOP_SCRIPT="$SCRIPT_DIR/stop_slam_toolbox_rf2o.sh"
SLAM_PARAMS="${SLAM_PARAMS:-$SCRIPT_DIR/slam_toolbox_rf2o.yaml}"
echo "slam params: $SLAM_PARAMS"
[ -f "$SLAM_PARAMS" ] || { echo "slam params not found: $SLAM_PARAMS"; exit 1; }

source /opt/ros/jazzy/setup.bash
# Prefer local_setup.bash: a workspace setup.bash may run apt on every source.
if [ -f "$RPLIDAR_WS/local_setup.bash" ]; then
  # shellcheck disable=SC1090
  source "$RPLIDAR_WS/local_setup.bash"
elif [ -f "$RPLIDAR_WS/setup.bash" ]; then
  # shellcheck disable=SC1090
  source "$RPLIDAR_WS/setup.bash"
fi
if [ -f "$RF2O_WS/install/local_setup.bash" ]; then
  # shellcheck disable=SC1090
  source "$RF2O_WS/install/local_setup.bash"
fi

# --- preflight ---
command -v ros2 >/dev/null || { echo "no ROS2 env"; exit 1; }
ros2 pkg prefix rf2o_laser_odometry >/dev/null 2>&1 \
  || { echo "rf2o_laser_odometry not installed (build $RF2O_WS)"; exit 1; }
ros2 pkg prefix slam_toolbox >/dev/null 2>&1 \
  || { echo "slam_toolbox not installed"; exit 1; }

# Refuse to start a second instance while a previous one is still running.
if [ -s "$PID_FILE" ]; then
  while read -r pid name mode; do
    [ -n "${pid:-}" ] || continue
    if kill -0 "$pid" 2>/dev/null; then
      echo "ERROR: $name is already running with PID $pid"
      echo "       run stop_slam_toolbox_rf2o.sh first"
      exit 1
    fi
  done < "$PID_FILE"
fi
: > "$PID_FILE"

# Remember a started process for stop_slam_toolbox_rf2o.sh.
#   mode "group" -> setsid'ed session leader, the whole group gets killed
#   mode "proc"  -> single process (it stops its children itself)
remember_pid() { echo "$1 $2 $3" >> "$PID_FILE"; }

cleanup() {
  # The stack is meant to keep running after this script exits -> don't stop
  # anything then (KEEP_RUNNING is set at the very end).
  [ "${KEEP_RUNNING:-0}" = "1" ] && return 0
  echo "--- stopping ---"
  if [ -x "$STOP_SCRIPT" ]; then
    if [ "$SKIP_LIDAR" = "1" ]; then
      KEEP_DRIVER=1 "$STOP_SCRIPT" || true
    else
      "$STOP_SCRIPT" || true
    fi
  else
    pkill -f "[s]tatic_transform_publisher" 2>/dev/null || true
    pkill -f "[r]f2o_laser_odometry_node" 2>/dev/null || true
    pkill -f "[s]lam_toolbox" 2>/dev/null || true
    if [ "$SKIP_LIDAR" = "0" ]; then
      pkill -f "[r]plidar_node" 2>/dev/null || true
    fi
  fi
}
# NOTE: no `exec` on the final launch: `exec` would replace the shell and this
# trap (and every other cleanup) would be lost.
trap cleanup EXIT INT TERM

# --- lidar driver (S2) ---
if [ "$SKIP_LIDAR" = "0" ]; then
  [ -e "$SERIAL_PORT" ] || { echo "no $SERIAL_PORT"; exit 1; }
  [ -r "$SERIAL_PORT" ] && [ -w "$SERIAL_PORT" ] \
    || echo "WARN: $SERIAL_PORT not rw (need dialout group or sudo chmod 666)"
  setsid ros2 launch rplidar_ros rplidar_s2_launch.py \
    serial_port:="$SERIAL_PORT" serial_baudrate:=1000000 \
    frame_id:="$LASER_FRAME" angle_compensate:="$ANGLE_COMPENSATE" \
    scan_mode:="$SCAN_MODE" \
    </dev/null >"$LIDAR_LOG" 2>&1 &
  remember_pid "$!" rplidar_driver group
  echo "rplidar S2 starting on $SERIAL_PORT ..."
  sleep 6
fi
if ! timeout "${SCAN_CHECK_TIMEOUT:-5}" ros2 topic hz "$SCAN_TOPIC" 2>/dev/null | grep -q "average rate"; then
  echo "WARN: no data on $SCAN_TOPIC yet (may be discovery lag), continuing anyway"
fi

# --- static TF chain: base_footprint -> base_link -> laser ---
# odom->base_footprint is published by rf2o, map->odom by slam_toolbox.
# LASER_YAW_DEG: 0 or 180.  The TF must describe the real mounting of the S2,
# otherwise the odometry is mirrored (driving forward would decrease the odom x
# and the map would be mirrored).
# Default 180 - VERIFIED on this robot (same value as start_rtabmap_laser.sh):
# pushing the robot 20 cm towards its front increased /odom's x from -0.150 to
# +0.386, so the odom's +X is the driving direction with yaw=180.
# To re-check: push the robot forward and watch /odom_rf2o's x (bigger = correct).
LASER_YAW_DEG="${LASER_YAW_DEG:-180}"
if [ "$LASER_YAW_DEG" = "180" ]; then LQZ=1; LQW=0; else LQZ=0; LQW=1; fi
LASER_REL_Z="$(awk -v a="$LASER_Z" -v b="$BASE_LINK_Z" 'BEGIN{printf "%.3f", a-b}')"
setsid ros2 run tf2_ros static_transform_publisher \
  --x 0 --y 0 --z "$BASE_LINK_Z" --qx 0 --qy 0 --qz 0 --qw 1 \
  --frame-id "$BASE_FRAME" --child-frame-id "$BASE_LINK_FRAME" \
  </dev/null >"$TF_BASE_LOG" 2>&1 &
remember_pid "$!" tf_base_link group
sleep 1
setsid ros2 run tf2_ros static_transform_publisher \
  --x 0 --y 0 --z "$LASER_REL_Z" --qx 0 --qy 0 --qz "$LQZ" --qw "$LQW" \
  --frame-id "$BASE_LINK_FRAME" --child-frame-id "$LASER_FRAME" \
  </dev/null >"$TF_LASER_LOG" 2>&1 &
remember_pid "$!" tf_laser group
sleep 1

# --- rf2o: laser odometry -> /odom_rf2o + odom->base_footprint ---
# NOTE: the parameters come from a file; an empty value on the command line
# (e.g. -p init_pose_from_topic:=) is rejected by the rcl argument parser.
RF2O_PARAMS="${RF2O_PARAMS:-$SCRIPT_DIR/rf2o_params.yaml}"
setsid ros2 run rf2o_laser_odometry rf2o_laser_odometry_node \
  --ros-args --params-file "$RF2O_PARAMS" \
  -p laser_scan_topic:="$SCAN_TOPIC" \
  -p odom_topic:="$ODOM_TOPIC" \
  -p base_frame_id:="$BASE_FRAME" \
  -p odom_frame_id:="$VO_FRAME_ID" \
  </dev/null >"$RF2O_LOG" 2>&1 &
remember_pid "$!" rf2o_odometry group
echo "rf2o odometry starting (${ODOM_TOPIC}) ..."

# --- slam_toolbox: /map + map->odom ---
# async (mapping) / localization: the SLAM node publishes map->odom itself,
# the odom->base_footprint part comes from rf2o.
if [ "$SLAM_MODE" = "localization" ]; then
  setsid ros2 launch slam_toolbox localization_launch.py \
    slam_params_file:="$SLAM_PARAMS" use_sim_time:=false \
    </dev/null >"$SLAM_LOG" 2>&1 &
else
  # async: scan processing runs in its own thread instead of the message
  # callback - on this 4-core ARM board sync mode pegs the CPU at ~220%.
  setsid ros2 launch slam_toolbox online_async_launch.py \
    slam_params_file:="$SLAM_PARAMS" use_sim_time:=false \
    </dev/null >"$SLAM_LOG" 2>&1 &
fi
remember_pid "$!" slam_toolbox group

sleep 4
if [ "$QUIET" = "1" ]; then
  if [ "$NO_LOG" = "1" ]; then
    echo "logs: disabled (NO_LOG=1)"
  else
    echo "logs:"
    echo "  slam_toolbox  : $SLAM_LOG"
    echo "  rf2o odometry : $RF2O_LOG"
    echo "  rplidar driver: $LIDAR_LOG"
    echo "  static TF     : $TF_BASE_LOG , $TF_LASER_LOG"
  fi
fi

# --- leave the stack running in the background (the console stays free) ---
# Same pattern as start_rtabmap_laser.sh: every process was started with
# setsid, so this script can simply exit.  No waiting for topics.
DEAD=""
if [ -f "$PID_FILE" ]; then
  while read -r pid name mode; do
    [ -n "${pid:-}" ] || continue
    kill -0 "$pid" 2>/dev/null || DEAD="$DEAD $name($pid)"
  done < "$PID_FILE"
fi
if [ -n "$DEAD" ]; then
  echo "ERROR: process(es) died:$DEAD"
  tail -n 20 "$SLAM_LOG" 2>/dev/null || true
  exit 1          # the EXIT trap cleans the driver / static TF up
fi

# The nodes keep running: do not let the EXIT trap stop them.
KEEP_RUNNING=1
echo
echo "slam_toolbox_rf2o started in the background:"
echo "  scan topic : $SCAN_TOPIC"
echo "  odom topic : $ODOM_TOPIC"
echo "  tf         : map->$VO_FRAME_ID ($SLAM_MODE) / $VO_FRAME_ID->$BASE_FRAME->$BASE_LINK_FRAME->$LASER_FRAME (yaw ${LASER_YAW_DEG} deg)"
echo "  pids       : $PID_FILE"
echo "stop it with: $STOP_SCRIPT"
