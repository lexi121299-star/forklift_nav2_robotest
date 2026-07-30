import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_share = get_package_share_directory('forklift_vehicle_interface')

    params_file = LaunchConfiguration('params_file')
    dry_run = LaunchConfiguration('dry_run')
    can_interface = LaunchConfiguration('can_interface')
    publish_tf = LaunchConfiguration('publish_tf')
    odom_frame_id = LaunchConfiguration('odom_frame_id')
    base_frame_id = LaunchConfiguration('base_frame_id')

    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(package_share, 'config', 'xfl201_vehicle_interface.yaml')),
        DeclareLaunchArgument('dry_run', default_value='true'),
        DeclareLaunchArgument('can_interface', default_value='can0'),
        DeclareLaunchArgument('publish_tf', default_value='true'),
        DeclareLaunchArgument('odom_frame_id', default_value='odom'),
        DeclareLaunchArgument('base_frame_id', default_value='base_link'),
        Node(
            package='forklift_vehicle_interface',
            executable='xfl201_vehicle_interface',
            name='xfl201_vehicle_interface',
            output='screen',
            parameters=[
                params_file,
                {
                    'dry_run': dry_run,
                    'can_interface': can_interface,
                    'publish_tf': publish_tf,
                    'odom_frame_id': odom_frame_id,
                    'base_frame_id': base_frame_id,
                },
            ],
        ),
    ])
