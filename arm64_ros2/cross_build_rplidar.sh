#!/bin/bash
# rplidar_ros cross compile 到 aarch64 (x86_64 Ubuntu 24.04 host)。
# 前提 (假設已就緒): qemu-user-static、g++-aarch64-linux-gnu、
#   apt 已見 :arm64 (ubuntu.sources + ros2.list 含 Architectures: amd64 arm64 且 apt update 過)、
#   host 已裝 ros-jazzy + colcon (source /opt/ros/jazzy/setup.bash)。
# 產出: ~/lidar_ws_arm64/install/ (整包 scp 到板子即可)。
set -e
source /opt/ros/jazzy/setup.bash

# ---- 1. 準備 sysroot 目錄 ----
mkdir -p ~/ros2_sysroot/{debs,sysroot}
cd ~/ros2_sysroot

# ---- 2. 下載 rplidar_ros 建置所需的 arm64 套件 (含傳遞相依) ----
# rplidar_ros 直接相依: rclcpp / sensor_msgs / std_srvs / rclcpp_components
sudo apt-get install --print-uris --no-install-recommends -y \
  ros-jazzy-rclcpp:arm64 \
  ros-jazzy-sensor-msgs:arm64 \
  ros-jazzy-std-srvs:arm64 \
  ros-jazzy-rclcpp-components:arm64 \
  | grep -oE "'https?://[^']+'" | tr -d "'" > debs.list
wc -l debs.list
wget -i debs.list -P debs/

# ---- 3. 解壓成 sysroot ----
for f in debs/*.deb; do dpkg-deb -x "$f" sysroot; done

# 確認是 arm64：
file sysroot/opt/ros/jazzy/lib/librclcpp.so
# 期待: ELF 64-bit LSB shared object, ARM aarch64 ...

# ---- 4. 取 rplidar_ros 原始碼 (ros2 branch 才有 S2) ----
mkdir -p ~/lidar_ws_arm64/src
cd ~/lidar_ws_arm64/src
[ -d rplidar_ros ] || git clone -b ros2 https://github.com/Slamtec/rplidar_ros.git
cd ~/lidar_ws_arm64

# ---- 5. cross 編譯 ----
export SYSROOT=~/ros2_sysroot/sysroot
export ROS2_PREFIX=$SYSROOT/opt/ros/jazzy
colcon build \
  --merge-install \
  --cmake-args \
    -DCMAKE_TOOLCHAIN_FILE=~/Downloads/komugi/lidar/arm64_ros2/toolchain-aarch64-ros2.cmake \
    -DCMAKE_PREFIX_PATH=$ROS2_PREFIX \
    -DAMENT_PREFIX_PATH=$ROS2_PREFIX \
    -DCMAKE_BUILD_TYPE=Release

# ---- 6. 驗證是 aarch64 ----
file install/rplidar_ros/lib/rplidar_ros/rplidar_node
# 期待: ELF 64-bit LSB executable, ARM aarch64 ...
aarch64-linux-gnu-readelf -h install/rplidar_ros/lib/rplidar_ros/rplidar_node | grep -E "Class|Machine"
aarch64-linux-gnu-objdump -p install/rplidar_ros/lib/rplidar_ros/rplidar_node | grep NEEDED | head

echo "CROSS BUILD DONE: ~/lidar_ws_arm64/install/"
echo "部署: scp -r ~/lidar_ws_arm64/install <board>:~/ ; 板上需先 apt 裝同名 ros-jazzy-* (arm64 runtime)，再 source install/setup.bash"
