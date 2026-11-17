#!/usr/bin/env bash
# Shared implementation for start_mapping.sh / start_localization.sh.
# Starts RPLIDAR S2, the static laser TF chain, and slam_toolbox.
# rf2o is started separately by ../rf2o/start.sh.
set -eo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]:-$0}")" && pwd)"

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-99}"
SLAM_MODE="${SLAM_MODE:-mapping}"
SLAM_PARAMS="${SLAM_PARAMS:-$SCRIPT_DIR/slam_toolbox_rf2o.yaml}"
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

[ -f "$SLAM_PARAMS" ] || { echo "ERROR: SLAM params not found: $SLAM_PARAMS" >&2; exit 1; }

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
      echo "ERROR: $name is already running (pid $pid); run ./stop.sh first" >&2
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
  "$SCRIPT_DIR/stop.sh" >/dev/null 2>&1 || true
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
echo "  stop : $SCRIPT_DIR/stop.sh"
echo "  rf2o : start separately with ../rf2o/start.sh"
