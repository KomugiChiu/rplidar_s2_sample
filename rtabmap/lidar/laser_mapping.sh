#!/usr/bin/env bash
# Laser 2D mapping on board: RPLidar S2 (/scan) + rtabmap icp_odometry + rtabmap.
# Run ON BOARD (komugi@board). No camera needed.
# Follows env convention (slam_toolbox_rf2o): frame `laser`, base `base_footprint`,
# S2 baud 1000000, DenseBoost; rplidar_ros from ~/test/rplidar_ros workspace.
#
# Usage (on board):
#   ./laser_mapping.sh                         # map only
#   BAG=~/bags/lidar_$(date +%Y%m%d) ./laser_mapping.sh   # map + record
#   SKIP_LIDAR=1 ./laser_mapping.sh            # /scan already up, skip driver
# Env overrides:
#   SCAN_TOPIC (default /scan)  LASER_FRAME (default laser)
#   BASE_FRAME (default base_footprint)  LASER_Z (default 0.20)
#   SERIAL_PORT (default /dev/ttyUSB0)  ANGLE_COMPENSATE (default false: weak board)
#   SCAN_MODE (default DenseBoost)  RPLIDAR_WS (default ~/test/rplidar_ros)
set -eo pipefail   # NOTE: no -u (ROS setup.bash needs unset vars)

SCAN_TOPIC="${SCAN_TOPIC:-/scan}"
LASER_FRAME="${LASER_FRAME:-laser}"
BASE_FRAME="${BASE_FRAME:-base_footprint}"
LASER_Z="${LASER_Z:-0.20}"
SERIAL_PORT="${SERIAL_PORT:-/dev/ttyUSB0}"
ANGLE_COMPENSATE="${ANGLE_COMPENSATE:-true}"
SCAN_MODE="${SCAN_MODE:-DenseBoost}"
RPLIDAR_WS="${RPLIDAR_WS:-$HOME/test/rplidar_ros}"
SKIP_LIDAR="${SKIP_LIDAR:-0}"
BAG="${BAG:-}"

source /opt/ros/jazzy/setup.bash
if [ -f "$RPLIDAR_WS/setup.bash" ]; then
  # shellcheck disable=SC1090
  source "$RPLIDAR_WS/setup.bash"
fi

# --- preflight ---
command -v ros2 >/dev/null || { echo "no ROS2 env"; exit 1; }
[ -x /opt/ros/jazzy/lib/rtabmap_odom/icp_odometry ] \
  || { echo "icp_odometry not installed"; exit 1; }

cleanup() {
  echo "--- stopping ---"
  pkill -f "[s]tatic_transform_publisher" 2>/dev/null || true
  # only kill the driver WE started (SKIP_LIDAR=1 means it belongs to someone else)
  if [ "$SKIP_LIDAR" = "0" ]; then
    pkill -f "[r]plidar_node" 2>/dev/null || true
  fi
  [ -n "${BAGPID:-}" ] && kill "$BAGPID" 2>/dev/null || true
}
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
    </dev/null >/tmp/lidar_rplidar.log 2>&1 &
  echo "rplidar S2 starting on $SERIAL_PORT ..."
  sleep 6
fi
# NOTE: warn-only on purpose. `ros2 topic hz/info` discovery is flaky on this
# board (false negatives while data actually flows); rtabmap/icp wait for
# data gracefully on their own, so a hard abort here causes more harm.
if ! timeout 10 ros2 topic hz "$SCAN_TOPIC" 2>/dev/null | grep -q "average rate"; then
  echo "WARN: no data on $SCAN_TOPIC yet (may be discovery lag), continuing anyway"
fi

# static TF: base -> laser (rtabmap needs TF scan->frame_id).
# LASER_YAW_DEG: 0 or 180. Use 180 if the odom arrow in rviz points opposite
# to the S2 printed arrow (base X then aligns with laser angle-0 flipped).
LASER_YAW_DEG="${LASER_YAW_DEG:-0}"
if [ "$LASER_YAW_DEG" = "180" ]; then LQZ=1; LQW=0; else LQZ=0; LQW=1; fi
setsid ros2 run tf2_ros static_transform_publisher \
  --x 0 --y 0 --z "$LASER_Z" --qx 0 --qy 0 --qz "$LQZ" --qw "$LQW" \
  --frame-id "$BASE_FRAME" --child-frame-id "$LASER_FRAME" \
  </dev/null >/tmp/lidar_static_tf.log 2>&1 &
sleep 2

# optional bag (sensor data only; map artifacts saved separately)
if [ -n "$BAG" ]; then
  setsid ros2 bag record -o "$BAG" \
    "$SCAN_TOPIC" /tf /tf_static /rtabmap/odom \
    </dev/null >/tmp/lidar_bag.log 2>&1 &
  BAGPID=$!
  echo "recording -> $BAG (pid $BAGPID)"
fi

# ICP pairing tolerances for handheld walking (defaults 0.2/0.1 reject normal
# steps as "out of bounds" -> tracking gaps -> fragmented map).
# Set via laser_mapping.ini + cfg:= below (ini values are strings, no type
# issues). Do NOT pass odom_args:=... here: empty value aborts the whole launch
# ("malformed launch argument"), and multi-pair strings arrive as one argv
# token which rtabmap's parser ignores. set_str_param.py remains for
# live-tuning a RUNNING node.
CFG="${CFG:-$HOME/rtabmap_lidar/laser_mapping.ini}"

# rtabmap: icp odometry ON (=odom source), ALL visual subscriptions OFF
# (subscribe_rgbd:=false alone is NOT enough: subscribe_depth defaults true
#  and also pulls rgb+depth+camera_info; visual_odometry:=false only stops
#  the odometry node). With camera topics in the sync set, rtabmap waits
#  forever ("Did not receive data") -> no map.
exec ros2 launch rtabmap_launch rtabmap.launch.py \
  visual_odometry:=false \
  subscribe_rgbd:=false \
  depth:=false \
  subscribe_rgb:=false \
  stereo:=false \
  icp_odometry:=true \
  subscribe_scan:=true \
  scan_topic:="$SCAN_TOPIC" \
  frame_id:="$BASE_FRAME" \
  cfg:="$CFG" \
  rtabmap_viz:=false \
  rviz:=false \
  approx_sync:=true \
  rtabmap_args:='--delete_db_on_start'
