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
    pallet_slots_file = LaunchConfiguration('pallet_slots_file')
    approach_params_file = LaunchConfiguration('approach_params_file')
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
            default_value=os.path.join(
                package_share, 'config', 'pallet_slots.yaml'
            ),
        ),
        DeclareLaunchArgument(
            'approach_params_file',
            default_value=os.path.join(
                package_share, 'config', 'xfl201_pallet_approach.yaml'
            ),
        ),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        Node(
            package='forklift_task_manager',
            executable='task_manager_node',
            name='forklift_task_manager',
            output='screen',
            parameters=[
                approach_params_file,
                {
                    'stations_file': stations_file,
                    'routes_file': routes_file,
                    'pallet_slots_file': pallet_slots_file,
                    'use_sim_time': use_sim_time,
                },
            ],
        ),
    ])
