import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_share = get_package_share_directory('forklift_nav2_demo')

    use_sim_time = LaunchConfiguration('use_sim_time')
    map_path = LaunchConfiguration('map_path')

    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(package_share, 'launch', 'forklift_slam.launch.py')),
        launch_arguments={
            'use_sim_time': use_sim_time,
        }.items(),
    )

    auto_mapper = Node(
        package='forklift_nav2_demo',
        executable='forklift_auto_slam_mapper',
        name='forklift_auto_slam_mapper',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'map_path': map_path,
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument(
            'map_path',
            default_value='/home/pl/robotest/forklift_factory_auto_map',
            description='Output map path without .yaml/.pgm extension.'),
        slam_launch,
        auto_mapper,
    ])
