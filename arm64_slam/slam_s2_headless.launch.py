#!/usr/bin/env python3
# 一鍵 headless 建圖：rplidar S2 + 2x static tf + slam_toolbox async
# 對應 README §1 的 4 個終端，合併為 1 個 launch。
# 板上用法 (先 scp 本檔上板)：
#   ros2 launch ~/slam_s2_headless.launch.py
#   ros2 launch ~/slam_s2_headless.launch.py serial_port:=/dev/ttyUSB1 angle_compensate:=false
#   ros2 launch ~/slam_s2_headless.launch.py slam_params_file:=~/my.yaml laser_height:=0.15
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    serial_port = LaunchConfiguration('serial_port', default='/dev/ttyUSB0')
    angle_compensate = LaunchConfiguration('angle_compensate', default='true')
    scan_mode = LaunchConfiguration('scan_mode', default='DenseBoost')
    laser_height = LaunchConfiguration('laser_height', default='0.2')
    slam_params_file = LaunchConfiguration(
        'slam_params_file',
        default=os.path.expanduser('~/slam_toolbox.yaml'))

    rplidar = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('rplidar_ros'),
                'launch', 'rplidar_s2_launch.py',
            ]),
        ]),
        launch_arguments={
            'serial_port': serial_port,
            'serial_baudrate': '1000000',
            'frame_id': 'laser',
            'angle_compensate': angle_compensate,
            'scan_mode': scan_mode,
        }.items(),
    )

    odom_to_base = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='odom_to_base_footprint',
        arguments=['0', '0', '0', '0', '0', '0', 'odom', 'base_footprint'],
        output='screen',
    )

    # 高度用 launch arg 拼 (x y z qx qy qz qw frame child)，z 取 laser_height
    base_to_laser = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_laser',
        arguments=['0', '0', laser_height, '0', '0', '0', 'base_footprint', 'laser'],
        output='screen',
    )

    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('slam_toolbox'),
                'launch', 'online_async_launch.py',
            ]),
        ]),
        launch_arguments={
            'slam_params_file': slam_params_file,
            'use_sim_time': 'false',
        }.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument('serial_port', default_value=serial_port,
                              description='rplidar serial port'),
        DeclareLaunchArgument('angle_compensate', default_value=angle_compensate,
                              description='rplidar angle_compensate (小板吃力就 false)'),
        DeclareLaunchArgument('scan_mode', default_value=scan_mode,
                              description='rplidar scan mode'),
        DeclareLaunchArgument('laser_height', default_value=laser_height,
                              description='雷達手持高度 (m)'),
        DeclareLaunchArgument('slam_params_file', default_value=slam_params_file,
                              description='slam_toolbox yaml 全路徑'),
        rplidar,
        odom_to_base,
        base_to_laser,
        slam,
    ])
