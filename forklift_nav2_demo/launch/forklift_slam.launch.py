import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    package_share = get_package_share_directory('forklift_nav2_demo')
    turtlebot3_cartographer_share = get_package_share_directory('turtlebot3_cartographer')

    use_sim_time = LaunchConfiguration('use_sim_time')
    cartographer_config_dir = LaunchConfiguration('cartographer_config_dir')
    configuration_basename = LaunchConfiguration('configuration_basename')

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(package_share, 'launch', 'forklift_gazebo.launch.py')),
    )

    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(turtlebot3_cartographer_share, 'launch', 'cartographer.launch.py')),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'cartographer_config_dir': cartographer_config_dir,
            'configuration_basename': configuration_basename,
        }.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument(
            'cartographer_config_dir',
            default_value=os.path.join(package_share, 'config'),
            description='Cartographer config directory.'),
        DeclareLaunchArgument(
            'configuration_basename',
            default_value='forklift_2d.lua',
            description='Cartographer lua config file.'),
        gazebo_launch,
        slam_launch,
    ])
