#!/usr/bin/env bash
# ICP odometry only (NO rtabmap SLAM node) + the static TF chain nav2 needs.
#
# Why a separate script:
#   When nav2 does the mapping (nav2_bringup with slam:=True -> slam_toolbox),
#   nav2 only needs the *odometry*.  The rtabmap SLAM node next to slam_toolbox
#   only burns CPU (on this 4 core board the load goes to 40-50, the planner
#   drops to ~1 Hz) and makes the map->odom authority ambiguous.
#
# What this starts:
#   /rplidar_node              S2 driver  -> /scan (frame "laser")
#   /rtabmap/icp_odometry      -> /odom + the TF odom->base_footprint
#   static_transform x2       base_footprint->base_link->laser
# The map frame and map->odom are published by nav2's slam_toolbox.
#
# Usage (on board):
#   ./start_icp_odom.sh
#   SERIAL_PORT=/dev/ttyUSB1 ./start_icp_odom.sh
#   QUIET=1 ./start_icp_odom.sh
# Stop it with ./stop_rtabmap_laser.sh (it knows this flow's PID file).
set -eo pipefail   # NOTE: no -u (ROS setup.bash needs unset vars)

ROS_DISTRO="${ROS_DISTRO:-jazzy}"
SCAN_TOPIC="${SCAN_TOPIC:-/scan}"
LASER_FRAME="${LASER_FRAME:-laser}"
BASE_FRAME="${BASE_FRAME:-base_footprint}"
BASE_LINK_FRAME="${BASE_LINK_FRAME:-base_link}"
BASE_LINK_Z="${BASE_LINK_Z:-0.10}"     # base_footprint -> base_link height
LASER_Z="${LASER_Z:-0.20}"              # lidar height above the floor
# LASER_YAW_DEG: 0 or 180.  The S2's scan angle 0 points along the arrow
# printed on the device.  If that arrow points backwards relative to the
# robot's driving direction, the pose in RViz ends up opposite to the S2
# arrow -> use 180 so the laser's X axis (and with it the odometry and the
# map) is aligned with the direction the robot actually drives.
LASER_YAW_DEG="${LASER_YAW_DEG:-0}"
SERIAL_PORT="${SERIAL_PORT:-/dev/ttyUSB0}"
ANGLE_COMPENSATE="${ANGLE_COMPENSATE:-true}"
SCAN_MODE="${SCAN_MODE:-DenseBoost}"
ODOM_TOPIC="${ODOM_TOPIC:-/odom}"       # absolute: nav2 subscribes to /odom
VO_FRAME_ID="${VO_FRAME_ID:-odom}"      # TF frame id (no leading slash!)
SCAN_CLOUD_UNUSED="${SCAN_CLOUD_UNUSED:-/scan_cloud_unused}"
RPLIDAR_WS="${RPLIDAR_WS:-/nvt_ros2_example/rplidar_ros/}"
QUIET="${QUIET:-0}"
NO_LOG="${NO_LOG:-0}"
LOG_LEVEL="${LOG_LEVEL:-}"
LOG_DIR="${LOG_DIR:-/tmp}"
RUN_DIR="${RUN_DIR:-/tmp/icp_odom}"
PID_FILE="${PID_FILE:-$RUN_DIR/pids}"

# Absolute path of this script (symlinks resolved) and of its directory.
SCRIPT_PATH="$(readlink -f -- "${BASH_SOURCE[0]:-$0}" 2>/dev/null || echo "${BASH_SOURCE[0]:-$0}")"
SCRIPT_DIR="$(cd -- "$(dirname -- "$SCRIPT_PATH")" && pwd)"
STOP_SCRIPT="$SCRIPT_DIR/stop_rtabmap_laser.sh"
# rtabmap parameters (ICP tolerances) - the .ini next to this script
CFG="${CFG:-$SCRIPT_DIR/rtabmap_laser_mapping.ini}"

if [ "$QUIET" = "1" ] && [ -z "$LOG_LEVEL" ]; then LOG_LEVEL=warn; fi
if [ "$NO_LOG" = "1" ]; then
  QUIET=1
  LOG_FILE=/dev/null; LIDAR_LOG=/dev/null; TF_BASE_LOG=/dev/null
  TF_LASER_LOG=/dev/null; ODOM_LOG=/dev/null
else
  mkdir -p "$LOG_DIR"
  LOG_FILE="$LOG_DIR/icp_odometry.log"
  LIDAR_LOG="$LOG_DIR/lidar_rplidar.log"
  TF_BASE_LOG="$LOG_DIR/lidar_static_base_link.log"
  TF_LASER_LOG="$LOG_DIR/lidar_static_tf.log"
  ODOM_LOG="$LOG_DIR/icp_odometry.log"
fi
mkdir -p "$RUN_DIR"

# --- environment ---
source "/opt/ros/$ROS_DISTRO/setup.bash"
# Prefer local_setup.bash: the workspace's setup.bash ends with an
# auto-generated `sudo apt-get install ...` that would run on every start.
if [ -f "$RPLIDAR_WS/local_setup.bash" ]; then
  # shellcheck disable=SC1090
  source "$RPLIDAR_WS/local_setup.bash"
elif [ -f "$RPLIDAR_WS/setup.bash" ]; then
  # shellcheck disable=SC1090
  source "$RPLIDAR_WS/setup.bash"
fi

# --- preflight ---
command -v ros2 >/dev/null || { echo "no ROS2 env"; exit 1; }
ODOM_BIN="$(ros2 pkg prefix rtabmap_odom 2>/dev/null)/lib/rtabmap_odom/icp_odometry"
[ -x "$ODOM_BIN" ] || { echo "icp_odometry not installed ($ODOM_BIN)"; exit 1; }
TF_BIN="$(ros2 pkg prefix tf2_ros 2>/dev/null)/lib/tf2_ros/static_transform_publisher"
[ -x "$TF_BIN" ] || { echo "static_transform_publisher not installed"; exit 1; }
[ -f "$CFG" ] || { echo "cfg not found: $CFG"; exit 1; }
[ -e "$SERIAL_PORT" ] || { echo "no $SERIAL_PORT"; exit 1; }
[ -r "$SERIAL_PORT" ] && [ -w "$SERIAL_PORT" ] \
  || echo "WARN: $SERIAL_PORT not rw (need dialout group or sudo chmod 666)"

# --- helpers ---
# Remember a started process for stop_rtabmap_laser.sh.
#   mode "group" -> setsid'ed session leader: the whole group gets killed
#   mode "proc"  -> single process
remember_pid() { echo "$1 $2 $3" >> "$PID_FILE"; }

cleanup() {
  echo "--- stopping ---"
  if [ -x "$STOP_SCRIPT" ]; then
    PID_FILE="$PID_FILE" EXTRA_PID_FILES="/tmp/rtabmap_laser/pids" "$STOP_SCRIPT" || true
  else
    [ -n "${DRIVER_PID:-}" ] && kill "$DRIVER_PID" 2>/dev/null || true
    [ -n "${TF_BASE_PID:-}" ] && kill "$TF_BASE_PID" 2>/dev/null || true
    [ -n "${TF_LASER_PID:-}" ] && kill "$TF_LASER_PID" 2>/dev/null || true
  fi
}
trap cleanup EXIT INT TERM

: > "$PID_FILE"

echo "cfg: $CFG"
echo "odom: $ODOM_TOPIC   tf: odom->$BASE_FRAME->$BASE_LINK_FRAME->$LASER_FRAME"

# --- lidar driver (S2) ---
setsid ros2 launch rplidar_ros rplidar_s2_launch.py \
  serial_port:="$SERIAL_PORT" angle_compensate:="$ANGLE_COMPENSATE" \
  scan_mode:="$SCAN_MODE" frame_id:="$LASER_FRAME" \
  </dev/null >"$LIDAR_LOG" 2>&1 &
DRIVER_PID=$!
remember_pid "$DRIVER_PID" rplidar_driver group
echo "rplidar S2 starting on $SERIAL_PORT ..."
sleep 6

# --- static TF: base_footprint -> base_link -> laser ---
# LASER_Z is the lidar height above the floor (footprint z=0); the laser link
# hangs under base_link with the relative offset so the total stays LASER_Z.
LASER_REL_Z="$(awk -v a="$LASER_Z" -v b="$BASE_LINK_Z" 'BEGIN{printf "%.3f", a-b}')"
if [ "$LASER_YAW_DEG" = "180" ]; then LQZ=1; LQW=0; else LQZ=0; LQW=1; fi
setsid "$TF_BIN" \
  --x 0 --y 0 --z "$BASE_LINK_Z" --qx 0 --qy 0 --qz 0 --qw 1 \
  --frame-id "$BASE_FRAME" --child-frame-id "$BASE_LINK_FRAME" \
  </dev/null >"$TF_BASE_LOG" 2>&1 &
TF_BASE_PID=$!
remember_pid "$TF_BASE_PID" tf_base_link group
sleep 1
setsid "$TF_BIN" \
  --x 0 --y 0 --z "$LASER_REL_Z" --qx 0 --qy 0 --qz "$LQZ" --qw "$LQW" \
  --frame-id "$BASE_LINK_FRAME" --child-frame-id "$LASER_FRAME" \
  </dev/null >"$TF_LASER_LOG" 2>&1 &
TF_LASER_PID=$!
remember_pid "$TF_LASER_PID" tf_laser group
sleep 1

# --- ICP odometry ---
# Run the executable directly (not `ros2 run`): `ros2 run` forks a wrapper
# process, and killing the wrapper leaves the node running.
#   -r scan_cloud:=<dummy> : icp_odometry subscribes to /scan_cloud by default
#     and nothing publishes it (only the rtabmap SLAM node would, and it waits
#     for the odometry) -> the odometry would never start.
#   -p guess_frame_id:=<base frame> : the odometry needs the laser->odom
#     transform to process the first scan, but the odom frame only exists
#     after the first odometry message -> bootstrap deadlock.  The guess frame
#     (statically connected to the laser) breaks the deadlock.
LOG_ARG=()
if [ -n "$LOG_LEVEL" ]; then LOG_ARG=(--log-level "icp_odometry:=$LOG_LEVEL"); fi

if [ "$QUIET" = "1" ]; then
  if [ "$NO_LOG" = "1" ]; then
    echo "logs: disabled (NO_LOG=1)"
  else
    echo "logs:"
    echo "  icp_odometry : $ODOM_LOG"
    echo "  rplidar      : $LIDAR_LOG"
    echo "  static TF    : $TF_BASE_LOG , $TF_LASER_LOG"
  fi
  echo "press Ctrl+C to stop"
fi

# Foreground subshell: the odometry keeps the terminal's SIGINT (so it shuts
# down gracefully) and its PID is recorded for the stop script.
(
  echo "$BASHPID icp_odometry proc" >> "$PID_FILE"
  exec "$ODOM_BIN" --ros-args \
    -r __node:=icp_odometry -r __ns:=/rtabmap \
    -r scan:="$SCAN_TOPIC" \
    -r odom:="$ODOM_TOPIC" \
    -r scan_cloud:="$SCAN_CLOUD_UNUSED" \
    -r imu:=/imu/data \
    -p frame_id:="$BASE_FRAME" \
    -p odom_frame_id:="$VO_FRAME_ID" \
    -p publish_tf:=true \
    -p subscribe_scan:=true \
    -p subscribe_scan_cloud:=false \
    -p guess_frame_id:="$BASE_FRAME" \
    -p wait_for_transform:=0.2 \
    -p wait_imu_to_init:=false \
    -p approx_sync:=true \
    -p topic_queue_size:=10 \
    -p sync_queue_size:=10 \
    -p always_process_most_recent_frame:=true \
    -p config_path:="$CFG" \
    "${LOG_ARG[@]}"
) </dev/null >"$ODOM_LOG" 2>&1
