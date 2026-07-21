import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_share = get_package_share_directory('forklift_task_manager')
    stations_file = LaunchConfiguration('stations_file')
    routes_file = LaunchConfiguration('routes_file')
    pallet_slots_file = LaunchConfiguration('pallet_slots_file')
    max_retries = LaunchConfiguration('max_retries')
    navigate_to_pose_action = LaunchConfiguration('navigate_to_pose_action')
    fork_move_to_action = LaunchConfiguration('fork_move_to_action')
    detect_pallet_offset_action = LaunchConfiguration('detect_pallet_offset_action')
    move_relative_action = LaunchConfiguration('move_relative_action')
    enforce_pallet_approach_station = LaunchConfiguration(
        'enforce_pallet_approach_station'
    )
    use_dispatch_reporter = LaunchConfiguration('use_dispatch_reporter')
    robot_id = LaunchConfiguration('robot_id')
    dispatch_report_topic = LaunchConfiguration('dispatch_report_topic')
    dispatch_http_url = LaunchConfiguration('dispatch_http_url')
    battery_low_threshold = LaunchConfiguration('battery_low_threshold')
    use_sim_time = LaunchConfiguration('use_sim_time')

    return LaunchDescription([
        DeclareLaunchArgument(
            'stations_file',
            default_value=os.path.join(package_share, 'config', 'stations.yaml'),
        ),
        DeclareLaunchArgument(
            'routes_file',
            default_value=os.path.join(package_share, 'config', 'routes.yaml'),
        ),
        DeclareLaunchArgument(
            'pallet_slots_file',
            default_value=os.path.join(package_share, 'config', 'pallet_slots.yaml'),
        ),
        DeclareLaunchArgument('max_retries', default_value='1'),
        DeclareLaunchArgument(
            'navigate_to_pose_action', default_value='navigate_to_pose'
        ),
        DeclareLaunchArgument(
            'fork_move_to_action', default_value='/forklift/fork/move_to'
        ),
        DeclareLaunchArgument(
            'detect_pallet_offset_action',
            default_value='/forklift/perception/detect_pallet_offset',
        ),
        DeclareLaunchArgument(
            'move_relative_action', default_value='/forklift/fine_motion/move_relative'
        ),
        DeclareLaunchArgument('enforce_pallet_approach_station', default_value='true'),
        DeclareLaunchArgument('use_dispatch_reporter', default_value='true'),
        DeclareLaunchArgument('robot_id', default_value='forklift_001'),
        DeclareLaunchArgument(
            'dispatch_report_topic', default_value='/forklift/dispatch_report'
        ),
        DeclareLaunchArgument('dispatch_http_url', default_value=''),
        DeclareLaunchArgument('battery_low_threshold', default_value='20.0'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        Node(
            package='forklift_task_manager',
            executable='task_manager_node',
            name='forklift_task_manager',
            output='screen',
            parameters=[{
                'stations_file': stations_file,
                'routes_file': routes_file,
                'pallet_slots_file': pallet_slots_file,
                'max_retries': max_retries,
                'navigate_to_pose_action': navigate_to_pose_action,
                'fork_move_to_action': fork_move_to_action,
                'detect_pallet_offset_action': detect_pallet_offset_action,
                'move_relative_action': move_relative_action,
                'enforce_pallet_approach_station': enforce_pallet_approach_station,
                'use_sim_time': use_sim_time,
            }],
        ),
        Node(
            package='forklift_task_manager',
            executable='dispatch_reporter_node',
            name='forklift_dispatch_reporter',
            output='screen',
            condition=IfCondition(use_dispatch_reporter),
            parameters=[{
                'robot_id': robot_id,
                'report_topic': dispatch_report_topic,
                'dispatch_http_url': dispatch_http_url,
                'battery_low_threshold': battery_low_threshold,
                'use_sim_time': use_sim_time,
            }],
        ),
    ])
