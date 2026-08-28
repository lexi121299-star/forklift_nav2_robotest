from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    wheel_base = LaunchConfiguration('wheel_base')
    max_velocity_mps = LaunchConfiguration('max_velocity_mps')
    max_steering_angle_rad = LaunchConfiguration('max_steering_angle_rad')
    max_angular_velocity_radps = LaunchConfiguration('max_angular_velocity_radps')
    allow_pivot_turn = LaunchConfiguration('allow_pivot_turn')
    pivot_steering_angle_rad = LaunchConfiguration('pivot_steering_angle_rad')
    pivot_steering_tolerance_rad = LaunchConfiguration('pivot_steering_tolerance_rad')
    pivot_turn_radius = LaunchConfiguration('pivot_turn_radius')
    pivot_angular_velocity_radps = LaunchConfiguration('pivot_angular_velocity_radps')
    command_timeout_sec = LaunchConfiguration('command_timeout_sec')
    control_rate_hz = LaunchConfiguration('control_rate_hz')
    cmd_vel_topic = LaunchConfiguration('cmd_vel_topic')
    twist_fallback_topic = LaunchConfiguration('twist_fallback_topic')
    twist_fallback_timeout_sec = LaunchConfiguration('twist_fallback_timeout_sec')

    bridge = Node(
        package='forklift_vehicle_interface',
        executable='sim_command_bridge',
        name='sim_command_bridge',
        output='screen',
        parameters=[{
            'wheel_base': wheel_base,
            'max_velocity_mps': max_velocity_mps,
            'max_steering_angle_rad': max_steering_angle_rad,
            'max_angular_velocity_radps': max_angular_velocity_radps,
            'allow_pivot_turn': allow_pivot_turn,
            'pivot_steering_angle_rad': pivot_steering_angle_rad,
            'pivot_steering_tolerance_rad': pivot_steering_tolerance_rad,
            'pivot_turn_radius': pivot_turn_radius,
            'pivot_angular_velocity_radps': pivot_angular_velocity_radps,
            'command_timeout_sec': command_timeout_sec,
            'control_rate_hz': control_rate_hz,
            'cmd_vel_topic': cmd_vel_topic,
            'twist_fallback_topic': twist_fallback_topic,
            'twist_fallback_timeout_sec': twist_fallback_timeout_sec,
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument('wheel_base', default_value='1.4'),
        DeclareLaunchArgument('max_velocity_mps', default_value='0.45'),
        DeclareLaunchArgument('max_steering_angle_rad', default_value='1.5707963267948966'),
        DeclareLaunchArgument('max_angular_velocity_radps', default_value='0.8'),
        DeclareLaunchArgument('allow_pivot_turn', default_value='true'),
        DeclareLaunchArgument('pivot_steering_angle_rad', default_value='1.5707963267948966'),
        DeclareLaunchArgument('pivot_steering_tolerance_rad', default_value='0.03'),
        DeclareLaunchArgument('pivot_turn_radius', default_value='0.6'),
        DeclareLaunchArgument('pivot_angular_velocity_radps', default_value='0.5'),
        DeclareLaunchArgument('command_timeout_sec', default_value='0.5'),
        DeclareLaunchArgument('control_rate_hz', default_value='20.0'),
        DeclareLaunchArgument('cmd_vel_topic', default_value='/cmd_vel'),
        DeclareLaunchArgument('twist_fallback_topic', default_value=''),
        DeclareLaunchArgument('twist_fallback_timeout_sec', default_value='0.5'),
        bridge,
    ])
