from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    wheel_base = LaunchConfiguration('wheel_base')
    max_velocity_mps = LaunchConfiguration('max_velocity_mps')
    max_steering_angle_rad = LaunchConfiguration('max_steering_angle_rad')
    command_timeout_sec = LaunchConfiguration('command_timeout_sec')
    control_rate_hz = LaunchConfiguration('control_rate_hz')
    cmd_vel_topic = LaunchConfiguration('cmd_vel_topic')

    bridge = Node(
        package='forklift_vehicle_interface',
        executable='sim_command_bridge',
        name='sim_command_bridge',
        output='screen',
        parameters=[{
            'wheel_base': wheel_base,
            'max_velocity_mps': max_velocity_mps,
            'max_steering_angle_rad': max_steering_angle_rad,
            'command_timeout_sec': command_timeout_sec,
            'control_rate_hz': control_rate_hz,
            'cmd_vel_topic': cmd_vel_topic,
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument('wheel_base', default_value='1.2'),
        DeclareLaunchArgument('max_velocity_mps', default_value='0.45'),
        DeclareLaunchArgument('max_steering_angle_rad', default_value='0.55'),
        DeclareLaunchArgument('command_timeout_sec', default_value='0.5'),
        DeclareLaunchArgument('control_rate_hz', default_value='20.0'),
        DeclareLaunchArgument('cmd_vel_topic', default_value='/cmd_vel'),
        bridge,
    ])
