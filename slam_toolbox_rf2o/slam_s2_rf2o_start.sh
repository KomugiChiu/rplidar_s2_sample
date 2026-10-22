#!/bin/bash
# slam_toolbox + rf2o + S2 一鍵啟動：source 環境 + 起 slam_s2_rf2o_headless.launch.py
# 用法：
#   ~/slam_s2_rf2o_start.sh
#   ~/slam_s2_rf2o_start.sh laser_height:=0.15
#   ~/slam_s2_rf2o_start.sh serial_port:=/dev/ttyUSB1 angle_compensate:=false
set -e
source /opt/ros/jazzy/setup.bash
source ~/test/rplidar_ros/setup.bash
source ~/rf2o_ws/install/setup.bash
exec ros2 launch ~/slam_s2_rf2o_headless.launch.py "$@"
