#!/bin/bash
# 板上 static 版重播前置：static odom->base + toolbox(async, sim time)，全部丟背景
source /opt/ros/jazzy/setup.bash
cd ~
pkill -f "static_transform_publisher.*odom base_footprint" 2>/dev/null
sleep 1
nohup ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 odom base_footprint --ros-args -p use_sim_time:=true > /tmp/static_replay.log 2>&1 < /dev/null &
echo $! > /tmp/static_replay.pid
nohup ros2 launch slam_toolbox online_async_launch.py slam_params_file:=$HOME/slam_toolbox.yaml use_sim_time:=true > /tmp/toolbox_replay.log 2>&1 < /dev/null &
echo $! > /tmp/toolbox_replay.pid
disown -a
echo "static_pid=$(cat /tmp/static_replay.pid) toolbox_pid=$(cat /tmp/toolbox_replay.pid)"
