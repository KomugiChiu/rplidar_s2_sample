#!/bin/bash
# 啟動 D455 IMU + base->camera 靜態 TF + ekf（跟隨 start_*_rf2o.sh 風格）。
# 前提：SLAM（rf2o publish_tf=false）已啟動。
#
# WITH_EKF=0 -> 只起 RealSense 與 base_footprint->camera_link 靜態 TF，不起
# ekf_node。給「odom 由別的節點負責」的流程用（例如 RTAB-Map 內建
# icp_odometry），因為兩個節點同時發佈 odom -> base_footprint 會互搶 TF。
set -u
# Every node of this stack must run in the same DDS domain.
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-99}"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
RUN_DIR="${RUN_DIR:-/tmp/slam_toolbox_rf2o_ekf}"
PID_FILE="${PID_FILE:-$RUN_DIR/pids}"
LOG_DIR="${LOG_DIR:-$RUN_DIR/logs}"
PARAMS_FILE="${EKF_PARAMS_FILE:-$SCRIPT_DIR/ekf_params.yaml}"
CAM_X="${CAM_X:-0.08}"
CAM_Y="${CAM_Y:-0.0}"
CAM_Z="${CAM_Z:-0.10}"
ROS_DISTRO="${ROS_DISTRO:-jazzy}"
WITH_EKF="${WITH_EKF:-1}"

mkdir -p "$RUN_DIR" "$LOG_DIR"
[ -f "$PARAMS_FILE" ] || { echo "ERROR: params file not found: $PARAMS_FILE"; exit 1; }

start_process() {
  local name="$1"
  shift
  echo "Starting $name..."
  setsid bash -c '
      source "/opt/ros/'"$ROS_DISTRO"'/setup.bash"
      export ROS_AUTODISCOVERY_RANGE=SUBNET
      unset ROS_LOCALHOST_ONLY
      exec "$@"
    ' bash "$@" </dev/null >"$LOG_DIR/$name.log" 2>&1 &
  local pid=$!
  echo "$pid $name" >> "$PID_FILE"
  sleep 3
  if ! kill -0 "$pid" 2>/dev/null; then
    echo "ERROR: $name failed to start"
    tail -n 40 "$LOG_DIR/$name.log"
    exit 1
  fi
  echo "  $name started with PID $pid"
}

# D455：gyro+accel + RGBD 在同一個 node（EKF 與 AI 共用，避免 IMU-only 衝突）
start_process realsense \
  ros2 launch realsense2_camera rs_launch.py \
    enable_gyro:=true enable_accel:=true \
    enable_rgbd:=true enable_sync:=true align_depth.enable:=true \
    enable_color:=true enable_depth:=true \
    enable_infra1:=false enable_infra2:=false \
    rgb_camera.color_profile:=640x480x15 \
    depth_module.depth_profile:=640x480x15

# base_footprint -> camera_link（D455 朝前水平置中：高 0.10、前 0.08）
start_process static_tf \
  ros2 run tf2_ros static_transform_publisher \
    --x "$CAM_X" --y "$CAM_Y" --z "$CAM_Z" \
    --qx 0 --qy 0 --qz 0 --qw 1 \
    --frame-id base_footprint --child-frame-id camera_link

if [ "$WITH_EKF" = "1" ]; then
  start_process ekf \
    ros2 run robot_localization ekf_node \
      --ros-args --params-file "$PARAMS_FILE"
fi

echo
if [ "$WITH_EKF" = "1" ]; then
  echo "ekf fusion started (logs: $LOG_DIR)"
else
  echo "RealSense + camera TF started without ekf (WITH_EKF=0, logs: $LOG_DIR)"
fi
echo "stop it with: $SCRIPT_DIR/stop_ekf.sh"
