import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_share = get_package_share_directory('forklift_task_manager')
    stations_file = LaunchConfiguration('stations_file')
    routes_file = LaunchConfiguration('routes_file')
    max_retries = LaunchConfiguration('max_retries')
    navigate_to_pose_action = LaunchConfiguration('navigate_to_pose_action')
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
        DeclareLaunchArgument('max_retries', default_value='1'),
        DeclareLaunchArgument(
            'navigate_to_pose_action', default_value='navigate_to_pose'
        ),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        Node(
            package='forklift_task_manager',
            executable='task_manager_node',
            name='forklift_task_manager',
            output='screen',
            parameters=[{
                'stations_file': stations_file,
                'routes_file': routes_file,
                'max_retries': max_retries,
                'navigate_to_pose_action': navigate_to_pose_action,
                'use_sim_time': use_sim_time,
            }],
        ),
    ])
