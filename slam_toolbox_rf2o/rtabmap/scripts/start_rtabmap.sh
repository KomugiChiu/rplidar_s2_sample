#!/usr/bin/env bash
# Start RPLIDAR S2 + static laser TFs + RTAB-Map (2D lidar mapping).
#
# Odometry comes from the rtabmap package itself:
#   rtabmap_odom/icp_odometry  (scan matching on /scan)
#     -> /odom_icp (topic)  +  TF odom -> base_footprint
#   rtabmap_slam/rtabmap       (subscribe_scan, odom_topic=/odom_icp)
#     -> /rtabmap/map       +  TF map -> odom
#
# So this script is self-contained for mapping: no rf2o, no EKF, no camera.
# (ODOM_SOURCE=external switches back to the rf2o + EKF stack, which publishes
# /odometry/filtered and owns odom -> base_footprint.  Never run both: two
# nodes publishing odom -> base_footprint makes the TF tree flip-flop.)
#
# Usage:
#   ./start_rtabmap.sh
#   SKIP_LIDAR=1 ./start_rtabmap.sh      # reuse an existing rplidar_node
#   DELETE_DB_ON_START=1 ./start_rtabmap.sh
#   ODOM_SOURCE=external ./start_rtabmap.sh   # rf2o + EKF instead of ICP
#
# Stop with: ./stop_rtabmap.sh
set -eo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]:-$0}")" && pwd)"

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-99}"
RPLIDAR_WS="${RPLIDAR_WS:-$SCRIPT_DIR/../rplidar_ros}"
SCAN_TOPIC="${SCAN_TOPIC:-/scan}"
SERIAL_PORT="${SERIAL_PORT:-/dev/ttyUSB0}"
SCAN_MODE="${SCAN_MODE:-DenseBoost}"
ANGLE_COMPENSATE="${ANGLE_COMPENSATE:-true}"
LASER_FRAME="${LASER_FRAME:-laser}"
BASE_FRAME="${BASE_FRAME:-base_footprint}"
BASE_LINK_FRAME="${BASE_LINK_FRAME:-base_link}"
BASE_LINK_Z="${BASE_LINK_Z:-0.10}"
LASER_Z="${LASER_Z:-0.20}"
LASER_YAW_DEG="${LASER_YAW_DEG:-180}"
SKIP_LIDAR="${SKIP_LIDAR:-0}"
TF_WAIT="${TF_WAIT:-30}"

ODOM_SOURCE="${ODOM_SOURCE:-icp}"          # icp | external
if [ "$ODOM_SOURCE" = "icp" ]; then
  ODOM_TOPIC="${ODOM_TOPIC:-/odom_icp}"
else
  ODOM_TOPIC="${ODOM_TOPIC:-/odometry/filtered}"
fi
VO_FRAME_ID="${VO_FRAME_ID:-odom}"
MAP_TOPIC="${MAP_TOPIC:-/rtabmap/map}"
RTABMAP_CFG="${RTABMAP_CFG:-$SCRIPT_DIR/rtabmap_laser_mapping.ini}"
DATABASE_PATH="${DATABASE_PATH:-$SCRIPT_DIR/rtabmap.db}"
DELETE_DB_ON_START="${DELETE_DB_ON_START:-0}"
LOG_LEVEL="${LOG_LEVEL:-warn}"

RUN_DIR="${RUN_DIR:-/tmp/rtabmap_only}"
PID_FILE="${PID_FILE:-$RUN_DIR/pids}"
LOG_DIR="${LOG_DIR:-$RUN_DIR/logs}"

[ -f "$RTABMAP_CFG" ] || { echo "ERROR: RTAB-Map cfg not found: $RTABMAP_CFG" >&2; exit 1; }

# shellcheck disable=SC1090
source /opt/ros/jazzy/setup.bash
if [ -f "$RPLIDAR_WS/local_setup.bash" ]; then
  # shellcheck disable=SC1090
  source "$RPLIDAR_WS/local_setup.bash"
elif [ -f "$RPLIDAR_WS/setup.bash" ]; then
  # shellcheck disable=SC1090
  source "$RPLIDAR_WS/setup.bash"
else
  echo "ERROR: rplidar workspace not found: $RPLIDAR_WS" >&2
  exit 1
fi
ros2 pkg prefix rtabmap_slam >/dev/null 2>&1 || {
  echo "ERROR: rtabmap_slam not installed" >&2
  exit 1
}

mkdir -p "$RUN_DIR" "$LOG_DIR"

if pgrep -f '[r]tabmap_slam/rtabmap|ros2 launch rtabmap_launch' >/dev/null 2>&1; then
  echo "ERROR: RTAB-Map is already running; stop it first." >&2
  exit 1
fi
if [ "$SKIP_LIDAR" = "0" ] && pgrep -f 'rplidar_ros/rplidar_node' >/dev/null 2>&1; then
  echo "ERROR: rplidar_node is already running; set SKIP_LIDAR=1 to reuse it." >&2
  exit 1
fi

if [ -s "$PID_FILE" ]; then
  while read -r pid name; do
    [ -n "${pid:-}" ] || continue
    if kill -0 "$pid" 2>/dev/null; then
      echo "ERROR: $name is already running (pid $pid); run ./stop_rtabmap.sh first" >&2
      exit 1
    fi
  done < "$PID_FILE"
fi
: > "$PID_FILE"

KEEP_RUNNING=0
start_bg() {
  local name="$1"; shift
  setsid "$@" </dev/null >"$LOG_DIR/$name.log" 2>&1 &
  local pid=$!
  echo "$pid $name" >> "$PID_FILE"
  echo "  $name pid=$pid"
}

cleanup() {
  [ "$KEEP_RUNNING" = "1" ] && return 0
  echo "startup failed; stopping partial stack" >&2
  "$SCRIPT_DIR/stop_rtabmap.sh" >/dev/null 2>&1 || true
}
trap cleanup EXIT INT TERM

echo "== RTAB-Map lidar mapping =="
echo "   scan   : $SCAN_TOPIC"
echo "   odom   : $ODOM_TOPIC ($ODOM_SOURCE)"
echo "   map    : $MAP_TOPIC"
echo "   db     : $DATABASE_PATH"
echo "   domain : $ROS_DOMAIN_ID"

if [ "$ODOM_SOURCE" = "icp" ]; then
  if pgrep -f '[r]f2o_laser_odometry_node|[e]kf_node' >/dev/null 2>&1; then
    echo "WARN: rf2o/EKF is running; ICP odometry and EKF would both publish" >&2
    echo "      TF odom -> $BASE_FRAME.  Stop them first: ../rf2o/stop.sh ../ekf/stop_ekf.sh" >&2
  fi
fi

# --- lidar ---
if [ "$SKIP_LIDAR" = "0" ]; then
  [ -e "$SERIAL_PORT" ] || { echo "ERROR: no $SERIAL_PORT" >&2; exit 1; }
  start_bg rplidar ros2 launch rplidar_ros rplidar_s2_launch.py \
    serial_port:="$SERIAL_PORT" serial_baudrate:=1000000 \
    frame_id:="$LASER_FRAME" angle_compensate:="$ANGLE_COMPENSATE" \
    scan_mode:="$SCAN_MODE"
  sleep 6
fi

# --- static TF chain: base_footprint -> base_link -> laser ---
if [ "$LASER_YAW_DEG" = "180" ]; then LQZ=1; LQW=0; else LQZ=0; LQW=1; fi
LASER_REL_Z="$(awk -v a="$LASER_Z" -v b="$BASE_LINK_Z" 'BEGIN{printf "%.3f", a-b}')"
start_bg tf_base ros2 run tf2_ros static_transform_publisher \
  --x 0 --y 0 --z "$BASE_LINK_Z" --qx 0 --qy 0 --qz 0 --qw 1 \
  --frame-id "$BASE_FRAME" --child-frame-id "$BASE_LINK_FRAME"
sleep 1
start_bg tf_laser ros2 run tf2_ros static_transform_publisher \
  --x 0 --y 0 --z "$LASER_REL_Z" --qx 0 --qy 0 --qz "$LQZ" --qw "$LQW" \
  --frame-id "$BASE_LINK_FRAME" --child-frame-id "$LASER_FRAME"
sleep 1

waited=0
while [ "$waited" -lt "$TF_WAIT" ]; do
  if ( timeout 3 ros2 run tf2_ros tf2_echo "$BASE_FRAME" "$LASER_FRAME" 2>&1 || true ) |
       grep -q "Translation"; then
    break
  fi
  sleep 2
  waited=$((waited + 2))
done
if [ "$waited" -ge "$TF_WAIT" ]; then
  echo "WARN: TF $BASE_FRAME -> $LASER_FRAME not ready after ${TF_WAIT}s" >&2
fi

# --- RTAB-Map (+ its own ICP odometry) ---
if [ "$ODOM_SOURCE" = "icp" ]; then
  ICP_ODOMETRY=true
  PUBLISH_TF_ODOM=true
else
  ICP_ODOMETRY=false
  PUBLISH_TF_ODOM=false
fi

RTABMAP_ARGS=(
  rtabmap_launch rtabmap.launch.py
  visual_odometry:=false
  icp_odometry:="$ICP_ODOMETRY"
  publish_tf_odom:="$PUBLISH_TF_ODOM"
  subscribe_rgbd:=false
  depth:=false
  subscribe_rgb:=false
  stereo:=false
  subscribe_scan:=true
  scan_topic:="$SCAN_TOPIC"
  frame_id:="$BASE_FRAME"
  odom_topic:="$ODOM_TOPIC"
  vo_frame_id:="$VO_FRAME_ID"
  map_frame_id:=map
  map_topic:="$MAP_TOPIC"
  publish_tf_map:=true
  cfg:="$RTABMAP_CFG"
  database_path:="$DATABASE_PATH"
  rtabmap_viz:=false
  rviz:=false
  approx_sync:=true
  wait_for_transform:=0.2
  log_level:="$LOG_LEVEL"
)
if [ "$DELETE_DB_ON_START" = "1" ]; then
  RTABMAP_ARGS+=(args:=--delete_db_on_start)
fi

start_bg rtabmap ros2 launch "${RTABMAP_ARGS[@]}"
sleep 5

DEAD=""
while read -r pid name; do
  kill -0 "$pid" 2>/dev/null || DEAD="$DEAD $name($pid)"
done < "$PID_FILE"
if [ -n "$DEAD" ]; then
  echo "ERROR: process(es) died:$DEAD" >&2
  tail -n 40 "$LOG_DIR/rtabmap.log" >&2 || true
  exit 1
fi

KEEP_RUNNING=1
echo
echo "RTAB-Map started in the background"
echo "  pids : $PID_FILE"
echo "  logs : $LOG_DIR"
echo "  stop : $SCRIPT_DIR/stop_rtabmap.sh"
if [ "$ODOM_SOURCE" = "icp" ]; then
  echo "  odom : icp_odometry -> $ODOM_TOPIC + TF odom -> $BASE_FRAME (self-contained)"
else
  echo "  odom : start separately with ../rf2o/start.sh + ../ekf/start_ekf.sh"
fi
