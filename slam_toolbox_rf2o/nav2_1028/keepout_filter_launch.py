# Keep-out zones for Nav2 (costmap KeepoutFilter).
#
# One component container with:
#   keepout_filter_mask_server  (nav2_map_server)  -> /keepout_filter_mask  (map frame)
#   costmap_filter_info_server                     -> /costmap_filter_info  (type 0 = keepout)
#   lifecycle_manager_keepout                      autostarts both
#
# Started by start_nav2.sh (KEEPOUT=1, the default).  The mask is drawn in the
# web console (Map tab -> Keep-out) which rewrites maps/keepout_mask.{pgm,yaml}
# and reloads it live through /keepout_filter_mask_server/load_map.
#
#   ros2 launch keepout_filter_launch.py mask:=/path/keepout_mask.yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    mask = LaunchConfiguration('mask')
    use_sim_time = LaunchConfiguration('use_sim_time')

    container = ComposableNodeContainer(
        name='keepout_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        output='screen',
        composable_node_descriptions=[
            ComposableNode(
                package='nav2_map_server',
                plugin='nav2_map_server::MapServer',
                name='keepout_filter_mask_server',
                parameters=[{
                    'use_sim_time': use_sim_time,
                    'yaml_filename': mask,
                    'topic_name': '/keepout_filter_mask',
                    'frame_id': 'map',
                }],
            ),
            ComposableNode(
                package='nav2_map_server',
                plugin='nav2_map_server::CostmapFilterInfoServer',
                name='costmap_filter_info_server',
                parameters=[{
                    'use_sim_time': use_sim_time,
                    'type': 0,  # keepout / preferred lanes
                    'filter_info_topic': '/costmap_filter_info',
                    'mask_topic': '/keepout_filter_mask',
                    'base': 0.0,
                    'multiplier': 1.0,
                }],
            ),
            ComposableNode(
                package='nav2_lifecycle_manager',
                plugin='nav2_lifecycle_manager::LifecycleManager',
                name='lifecycle_manager_keepout',
                parameters=[{
                    'use_sim_time': use_sim_time,
                    'autostart': True,
                    'node_names': ['keepout_filter_mask_server', 'costmap_filter_info_server'],
                }],
            ),
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument('mask', description='keep-out mask yaml (map_server format)'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        container,
    ])
