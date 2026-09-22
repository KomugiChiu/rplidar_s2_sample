#!/usr/bin/env python3
# 一鍵 headless 建圖：rplidar S2 + static tf + rf2o laser odom + slam_toolbox async
# 注意：odom→base_footprint 由 rf2o 動態發布，不可再加 static 版（TF 衝突）。
# 板上用法 (先傳上板，見 README)：
#   ros2 launch ~/slam_s2_rf2o_headless.launch.py
#   ros2 launch ~/slam_s2_rf2o_headless.launch.py laser_height:=0.15
#   ros2 launch ~/slam_s2_rf2o_headless.launch.py serial_port:=/dev/ttyUSB1 angle_compensate:=false
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
    rf2o_freq = LaunchConfiguration('rf2o_freq', default='10.0')
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

    # 高度用 launch arg 拼 (x y z qx qy qz qw frame child)，z 取 laser_height
    # 只有這一條 static tf；odom→base_footprint 由 rf2o 發動態，不可再補 static
    base_to_laser = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_laser',
        arguments=['0', '0', laser_height, '0', '0', '0', 'base_footprint', 'laser'],
        output='screen',
    )

    rf2o = Node(
        package='rf2o_laser_odometry',
        executable='rf2o_laser_odometry_node',
        name='rf2o_laser_odometry',
        output='screen',
        parameters=[{
            'laser_scan_topic': '/scan',
            'odom_topic': '/odom_rf2o',
            'publish_tf': True,
            'base_frame_id': 'base_footprint',
            'odom_frame_id': 'odom',
            'init_pose_from_topic': '',
            'freq': rf2o_freq,
        }],
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
        DeclareLaunchArgument('rf2o_freq', default_value=rf2o_freq,
                              description='rf2o odom 發布頻率 (Hz，對齊雷達 10Hz)'),
        DeclareLaunchArgument('slam_params_file', default_value=slam_params_file,
                              description='slam_toolbox yaml 全路徑'),
        rplidar,
        base_to_laser,
        rf2o,
        slam,
    ])
