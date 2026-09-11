#!/bin/bash
# x86 ROS2 S2 驗證：起 headless node 取 /scan (不開 rviz，SSH 也能跑)。
# 用法: ./s2_ros_check.sh [/dev/ttyUSB0] [秒數]
# 需求: /opt/ros/jazzy + ~/ros2_ws (跑過 install_driver.sh)；需接實機。
set -u
PORT="${1:-/dev/ttyUSB0}"
SEC="${2:-12}"
source /opt/ros/jazzy/setup.bash
[ -f ~/ros2_ws/install/setup.bash ] && source ~/ros2_ws/install/setup.bash
# 驗證用 workspace (本次實測路徑)，有才加
[ -f /tmp/opencode/lidar_ws/install/setup.bash ] && source /tmp/opencode/lidar_ws/install/setup.bash

echo "port=${PORT} sec=${SEC}"
ls -l "${PORT}"
ros2 pkg prefix rplidar_ros
ros2 launch rplidar_ros rplidar_s2_launch.py &
LAUNCH_PID=$!
trap "kill ${LAUNCH_PID} 2>/dev/null || true" EXIT
sleep 6
echo "--- /scan once ---"
timeout 10 ros2 topic echo /scan --once | head -n 20
echo "--- /scan hz ---"
timeout 10 ros2 topic hz /scan
echo "CHECK DONE (Ctrl-C 或等待 exit 自動殺 node)"
kill ${LAUNCH_PID} 2>/dev/null || true
