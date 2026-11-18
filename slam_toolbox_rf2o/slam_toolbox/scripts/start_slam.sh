#!/usr/bin/env bash
# Unified slam_toolbox start script.
#
# Modes:
#   mapping       build a new map (online_async_launch.py)
#   localization  load an existing posegraph (localization_launch.py)
#
# Usage:
#   ./start_slam.sh mapping
#   ./start_slam.sh localization
#   ./start_slam.sh                 # default: mapping
#
# Environment:
#   SLAM_PARAMS=/path/to/params.yaml
#   SKIP_LIDAR=1               # reuse an existing rplidar_node
#
# Stop with: ./stop_slam.sh
set -eo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]:-$0}")" && pwd)"

usage() {
  sed -n '2,18p' "$0" | sed 's/^# \{0,1\}//'
}

MODE="${1:-${SLAM_MODE:-mapping}}"
case "$MODE" in
  mapping|map) MODE="mapping" ;;
  localization|localization-only|static|static-map|static_map) MODE="localization" ;;
  --help|-h) usage; exit 0 ;;
  *)
    echo "ERROR: unknown mode '$MODE' (use mapping or localization)" >&2
    usage >&2
    exit 2
    ;;
esac

export SLAM_MODE="$MODE"

if [ "$MODE" = "localization" ]; then
  SLAM_PARAMS="${SLAM_PARAMS:-$SCRIPT_DIR/slam_toolbox_localization.yaml}"
  export SLAM_PARAMS

  [ -f "$SLAM_PARAMS" ] || {
    echo "ERROR: localization params not found: $SLAM_PARAMS" >&2
    exit 1
  }

  map_file="$(sed -n 's/^[[:space:]]*map_file_name:[[:space:]]*"\{0,1\}\([^"]*\)"\{0,1\}[[:space:]]*$/\1/p' \
    "$SLAM_PARAMS" | head -n1)"
  [ -n "$map_file" ] || {
    echo "ERROR: no map_file_name in $SLAM_PARAMS" >&2
    exit 1
  }
  if [ ! -f "${map_file}.posegraph" ] || [ ! -f "${map_file}.data" ]; then
    echo "ERROR: posegraph not found: ${map_file}.posegraph / .data" >&2
    exit 1
  fi
  if ! grep -qE '^[[:space:]]*(map_start_pose|map_start_at_dock):' "$SLAM_PARAMS"; then
    echo "WARN: neither map_start_pose nor map_start_at_dock is set in $SLAM_PARAMS"
  fi
else
  SLAM_PARAMS="${SLAM_PARAMS:-$SCRIPT_DIR/slam_toolbox_rf2o.yaml}"
  export SLAM_PARAMS
  [ -f "$SLAM_PARAMS" ] || {
    echo "ERROR: mapping params not found: $SLAM_PARAMS" >&2
    exit 1
  }
fi

echo "== slam_toolbox =="
echo "   mode   : $MODE"
echo "   params : $SLAM_PARAMS"
echo "   domain : ${ROS_DOMAIN_ID:-99}"

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

RUN_DIR="${RUN_DIR:-/tmp/slam_toolbox_only}"
PID_FILE="${PID_FILE:-$RUN_DIR/pids}"
LOG_DIR="${LOG_DIR:-$RUN_DIR/logs}"


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
ros2 pkg prefix slam_toolbox >/dev/null 2>&1 || {
  echo "ERROR: slam_toolbox not installed" >&2
  exit 1
}

mkdir -p "$RUN_DIR" "$LOG_DIR"

if pgrep -f '[a]sync_slam_toolbox_node|[l]ocalization_slam_toolbox_node' >/dev/null 2>&1; then
  echo "ERROR: a slam_toolbox node is already running; stop it first." >&2
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
      echo "ERROR: $name is already running (pid $pid); run ./stop_slam.sh first" >&2
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
  "$SCRIPT_DIR/stop_slam.sh" >/dev/null 2>&1 || true
}
trap cleanup EXIT INT TERM

echo "== slam_toolbox $SLAM_MODE =="
echo "   params : $SLAM_PARAMS"
echo "   scan   : $SCAN_TOPIC"
echo "   domain : $ROS_DOMAIN_ID"

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

# rf2o reads this TF once on the first scan; wait for it here as well.
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

# --- slam_toolbox ---
if [ "$SLAM_MODE" = "localization" ]; then
  start_bg slam ros2 launch slam_toolbox localization_launch.py \
    slam_params_file:="$SLAM_PARAMS" use_sim_time:=false
else
  start_bg slam ros2 launch slam_toolbox online_async_launch.py \
    slam_params_file:="$SLAM_PARAMS" use_sim_time:=false
fi
sleep 4

DEAD=""
while read -r pid name; do
  kill -0 "$pid" 2>/dev/null || DEAD="$DEAD $name($pid)"
done < "$PID_FILE"
if [ -n "$DEAD" ]; then
  echo "ERROR: process(es) died:$DEAD" >&2
  tail -n 30 "$LOG_DIR/slam.log" >&2 || true
  exit 1
fi

KEEP_RUNNING=1
echo
echo "slam_toolbox $SLAM_MODE started in the background"
echo "  pids : $PID_FILE"
echo "  logs : $LOG_DIR"
echo "  stop : $SCRIPT_DIR/stop_slam.sh"
echo "  rf2o : start separately with ../rf2o/start.sh"
