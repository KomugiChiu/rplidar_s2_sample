#!/usr/bin/env bash
# Laser 2D mapping on board: RPLidar S2 (/scan) + rtabmap icp_odometry + rtabmap.
# Run ON BOARD (komugi@board). No camera needed.
# Follows env convention (slam_toolbox_rf2o): frame `laser`, base `base_footprint`,
# S2 baud 1000000, DenseBoost; rplidar_ros from ~/test/rplidar_ros workspace.
#
# Usage (on board):
#   ./start_rtabmap_laser.sh                   # map only
#   BAG=~/bags/lidar_$(date +%Y%m%d) ./start_rtabmap_laser.sh   # map + record
#   SKIP_LIDAR=1 ./start_rtabmap_laser.sh      # /scan already up, skip driver
#   QUIET=1 ./start_rtabmap_laser.sh           # quiet console, node output -> log files
#   ./stop_rtabmap_laser.sh                    # stop everything this script started
# Env overrides:
#   SCAN_TOPIC (default /scan)  LASER_FRAME (default laser)
#   BASE_FRAME (default base_footprint)  BASE_LINK_FRAME (default base_link)
#   BASE_LINK_Z (default 0.10)  LASER_Z (default 0.20)
#   SERIAL_PORT (default /dev/ttyUSB0)  ANGLE_COMPENSATE (default false: weak board)
#   SCAN_MODE (default DenseBoost)  RPLIDAR_WS (default ~/test/rplidar_ros)
#   CFG (default: <this script's dir>/rtabmap_laser_mapping.ini)
#   QUIET (default 0: 1 = keep the console clean, all node output -> log files)
#   NO_LOG (default 0: 1 = no log files at all, everything -> /dev/null)
#   LOG_LEVEL (default: info; forwarded to the launch as log_level:=<level>)
#   LOG_DIR (default /tmp: where the log files are written)
# The script may be started from any working directory: the .ini and the
# stop script are always looked up next to this file.
set -eo pipefail   # NOTE: no -u (ROS setup.bash needs unset vars)

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
SKIP_LIDAR="${SKIP_LIDAR:-0}"
BAG="${BAG:-}"
# --- logging ---
# QUIET=1 keeps the console clean: the rtabmap launch output is written to a
# file instead of the terminal.  LOG_LEVEL lowers the verbosity of the
# rtabmap/icp_odometry nodes (info|warn|error|debug).
QUIET="${QUIET:-0}"
NO_LOG="${NO_LOG:-0}"
LOG_LEVEL="${LOG_LEVEL:-}"
LOG_DIR="${LOG_DIR:-/tmp}"
if [ "$QUIET" = "1" ] && [ -z "$LOG_LEVEL" ]; then LOG_LEVEL=warn; fi
if [ "$NO_LOG" = "1" ]; then
  # no log files at all: everything is discarded
  QUIET=1
  LOG_FILE=/dev/null
  LIDAR_LOG=/dev/null
  TF_BASE_LOG=/dev/null
  TF_LASER_LOG=/dev/null
  BAG_LOG=/dev/null
else
  mkdir -p "$LOG_DIR"
  LOG_FILE="${LOG_FILE:-$LOG_DIR/lidar_rtabmap.log}"
  LIDAR_LOG="$LOG_DIR/lidar_rplidar.log"
  TF_BASE_LOG="$LOG_DIR/lidar_static_base_link.log"
  TF_LASER_LOG="$LOG_DIR/lidar_static_tf.log"
  BAG_LOG="$LOG_DIR/lidar_bag.log"
fi
# Absolute path of this script (symlinks resolved) and of its directory.
SCRIPT_PATH="$(readlink -f -- "${BASH_SOURCE[0]:-$0}" 2>/dev/null || echo "${BASH_SOURCE[0]:-$0}")"
SCRIPT_DIR="$(cd -- "$(dirname -- "$SCRIPT_PATH")" && pwd)"
STOP_SCRIPT="$SCRIPT_DIR/stop_rtabmap_laser.sh"
# rtabmap parameters: the .ini next to this script (absolute path), so the
# script can be started from any working directory.
# Override with CFG=/path/to/other.ini
CFG="${CFG:-$SCRIPT_DIR/rtabmap_laser_mapping.ini}"
echo "cfg: $CFG"
[ -f "$CFG" ] || { echo "cfg not found: $CFG"; exit 1; }

source /opt/ros/jazzy/setup.bash
# Prefer local_setup.bash: the workspace's setup.bash ends with an
# auto-generated `sudo apt-get install ...` (added by build_and_package.sh)
# which would run on every start and spam the console.
if [ -f "$RPLIDAR_WS/local_setup.bash" ]; then
  # shellcheck disable=SC1090
  source "$RPLIDAR_WS/local_setup.bash"
elif [ -f "$RPLIDAR_WS/setup.bash" ]; then
  # shellcheck disable=SC1090
  source "$RPLIDAR_WS/setup.bash"
fi

# --- preflight ---
command -v ros2 >/dev/null || { echo "no ROS2 env"; exit 1; }
[ -x /opt/ros/jazzy/lib/rtabmap_odom/icp_odometry ] \
  || { echo "icp_odometry not installed"; exit 1; }

cleanup() {
  echo "--- stopping ---"
  # The driver / static TF / bag recorder are started with `setsid`, so they
  # do NOT get the terminal's Ctrl+C.  stop_rtabmap_laser.sh owns that kill
  # list and also rescues leftovers from an earlier run that was killed.
  if [ -x "$STOP_SCRIPT" ]; then
    if [ "$SKIP_LIDAR" = "1" ]; then
      KEEP_DRIVER=1 "$STOP_SCRIPT" || true
    else
      "$STOP_SCRIPT" || true
    fi
  else
    pkill -f "[s]tatic_transform_publisher" 2>/dev/null || true
    # only kill the driver WE started (SKIP_LIDAR=1 means it belongs to someone else)
    if [ "$SKIP_LIDAR" = "0" ]; then
      pkill -f "[r]plidar_node" 2>/dev/null || true
    fi
  fi
  [ -n "${BAGPID:-}" ] && kill "$BAGPID" 2>/dev/null || true
}
# NOTE: no `exec` on the final `ros2 launch` below.  With `exec` the shell is
# replaced and this trap (plus every other cleanup) is lost, so the setsid
# children would survive Ctrl+C as orphans.  Without `exec` the trap runs on
# Ctrl+C / SIGTERM / normal exit.
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
  echo "rplidar S2 starting on $SERIAL_PORT ..."
  sleep 6
fi
# NOTE: warn-only on purpose. `ros2 topic hz/info` discovery is flaky on this
# board (false negatives while data actually flows); rtabmap/icp wait for
# data gracefully on their own, so a hard abort here causes more harm.
if ! timeout 10 ros2 topic hz "$SCAN_TOPIC" 2>/dev/null | grep -q "average rate"; then
  echo "WARN: no data on $SCAN_TOPIC yet (may be discovery lag), continuing anyway"
fi

# static TF chain: base_footprint -> base_link -> laser
# (rtabmap needs TF scan->frame_id; nav2 consumes base_link).
# LASER_YAW_DEG: 0 or 180. Use 180 if the odom arrow in rviz points opposite
# to the S2 printed arrow (base X then aligns with laser angle-0 flipped).
LASER_YAW_DEG="${LASER_YAW_DEG:-0}"
if [ "$LASER_YAW_DEG" = "180" ]; then LQZ=1; LQW=0; else LQZ=0; LQW=1; fi
# Static TF chain: base_footprint -> base_link -> laser.
# LASER_Z is the lidar height above ground (footprint z=0); the laser
# link hangs under base_link with the relative offset so the total is
# unchanged (BASE_LINK_Z + relative = LASER_Z).
LASER_REL_Z="$(awk -v a="$LASER_Z" -v b="$BASE_LINK_Z" 'BEGIN{printf "%.3f", a-b}')"
setsid ros2 run tf2_ros static_transform_publisher \
  --x 0 --y 0 --z "$BASE_LINK_Z" --qx 0 --qy 0 --qz 0 --qw 1 \
  --frame-id "$BASE_FRAME" --child-frame-id "$BASE_LINK_FRAME" \
  </dev/null >"$TF_BASE_LOG" 2>&1 &
sleep 1
setsid ros2 run tf2_ros static_transform_publisher \
  --x 0 --y 0 --z "$LASER_REL_Z" --qx 0 --qy 0 --qz "$LQZ" --qw "$LQW" \
  --frame-id "$BASE_LINK_FRAME" --child-frame-id "$LASER_FRAME" \
  </dev/null >"$TF_LASER_LOG" 2>&1 &
sleep 1

# optional bag (sensor data only; map artifacts saved separately)
if [ -n "$BAG" ]; then
  setsid ros2 bag record -o "$BAG" \
    "$SCAN_TOPIC" /tf /tf_static /rtabmap/odom \
    </dev/null >"$BAG_LOG" 2>&1 &
  BAGPID=$!
  echo "recording -> $BAG (pid $BAGPID)"
fi

# ICP pairing tolerances for handheld walking (defaults 0.2/0.1 reject normal
# steps as "out of bounds" -> tracking gaps -> fragmented map).
# Set via rtabmap_laser_mapping.ini + cfg:= below (ini values are strings, no
# type issues). Do NOT pass odom_args:=... here: empty value aborts the whole launch
# ("malformed launch argument"), and multi-pair strings arrive as one argv
# token which rtabmap's parser ignores. set_str_param.py remains for
# live-tuning a RUNNING node.  (CFG is resolved at the top of this script.)

# rtabmap: icp odometry ON (=odom source), ALL visual subscriptions OFF
# (subscribe_rgbd:=false alone is NOT enough: subscribe_depth defaults true
#  and also pulls rgb+depth+camera_info; visual_odometry:=false only stops
#  the odometry node). With camera topics in the sync set, rtabmap waits
#  forever ("Did not receive data") -> no map.
LAUNCH_ARGS=(
  rtabmap_launch rtabmap.launch.py
  visual_odometry:=false
  subscribe_rgbd:=false
  depth:=false
  subscribe_rgb:=false
  stereo:=false
  icp_odometry:=true
  subscribe_scan:=true
  scan_topic:="$SCAN_TOPIC"
  frame_id:="$BASE_FRAME"
  cfg:="$CFG"
  rtabmap_viz:=false
  rviz:=false
  approx_sync:=true
  rtabmap_args:='--delete_db_on_start'
)
# LOG_LEVEL (info|warn|error|debug) -> node verbosity; empty = launch default
if [ -n "$LOG_LEVEL" ]; then LAUNCH_ARGS+=(log_level:="$LOG_LEVEL"); fi

if [ "$QUIET" = "1" ]; then
  # keep the console clean: everything the nodes print goes to the log files
  if [ "$NO_LOG" = "1" ]; then
    echo "logs: disabled (NO_LOG=1)"
  else
    echo "logs:"
    echo "  rtabmap/icp   : $LOG_FILE   (tail -f $LOG_FILE)"
    echo "  rplidar driver: $LIDAR_LOG"
    echo "  static TF     : $TF_BASE_LOG , $TF_LASER_LOG"
    if [ -n "$BAG" ]; then echo "  bag recorder  : $BAG_LOG"; fi
  fi
  echo "press Ctrl+C to stop"
  ros2 launch "${LAUNCH_ARGS[@]}" >"$LOG_FILE" 2>&1
else
  ros2 launch "${LAUNCH_ARGS[@]}"
fi
