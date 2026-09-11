#!/bin/bash
# x86 (Ubuntu 24.04 + Jazzy) ROS2 driver 安裝
# 結論(2026-09-10實測): apt 的 ros-jazzy-rplidar-ros=2.1.0 沒有 S2 launch，
# S2 一律從 source 編 (ros2 branch)。需 sudo 的只有 apt/rosdep 步驟。
set -e
source /opt/ros/jazzy/setup.bash

echo "=== 1. 基礎工具 (需 sudo) ==="
sudo apt update
sudo apt install -y python3-colcon-common-extensions git

echo "=== 2. source 編 rplidar_ros (含 S2) ==="
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
if [ ! -d rplidar_ros ]; then
  git clone -b ros2 https://github.com/Slamtec/rplidar_ros.git
fi
cd ~/ros2_ws
rosdep update || true
# rosdep 需 sudo；失敗可跳過(本包依賴只有 rclcpp/sensor_msgs，多半已齊)
sudo rosdep install --from-paths src --ignore-src -r -y || \
  rosdep install --from-paths src --ignore-src -r -y --simulate || true
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source ~/ros2_ws/install/setup.bash
ls ~/ros2_ws/install/rplidar_ros/share/rplidar_ros/launch/ | grep s2

echo "=== 3. 串口權限 ==="
if [ -f ~/ros2_ws/src/rplidar_ros/scripts/create_udev_rules.sh ]; then
  (cd ~/ros2_ws/src/rplidar_ros && sudo bash scripts/create_udev_rules.sh) || true
fi
id | grep -q dialout && echo "已在 dialout (免重登)" || \
  echo "請執行: sudo usermod -aG dialout \$USER 後重登；臨時可用 sudo chmod 666 /dev/ttyUSB0"
echo "INSTALL DONE"
