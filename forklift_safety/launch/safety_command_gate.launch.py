from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    enabled = LaunchConfiguration('enabled')
    raw_command_topic = LaunchConfiguration('raw_command_topic')
    gated_command_topic = LaunchConfiguration('gated_command_topic')
    recovery_twist_topic = LaunchConfiguration('recovery_twist_topic')
    command_timeout_sec = LaunchConfiguration('command_timeout_sec')
    recovery_timeout_sec = LaunchConfiguration('recovery_timeout_sec')
    max_forward_velocity_mps = LaunchConfiguration('max_forward_velocity_mps')
    max_reverse_velocity_mps = LaunchConfiguration('max_reverse_velocity_mps')
    max_recovery_velocity_mps = LaunchConfiguration('max_recovery_velocity_mps')
    max_recovery_angular_velocity_radps = LaunchConfiguration(
        'max_recovery_angular_velocity_radps')
    max_steering_angle_rad = LaunchConfiguration('max_steering_angle_rad')
    wheel_base = LaunchConfiguration('wheel_base')
    pivot_turn_radius = LaunchConfiguration('pivot_turn_radius')
    pivot_steering_angle_rad = LaunchConfiguration('pivot_steering_angle_rad')
    control_rate_hz = LaunchConfiguration('control_rate_hz')
    use_sim_time = LaunchConfiguration('use_sim_time')

    return LaunchDescription([
        DeclareLaunchArgument('enabled', default_value='true'),
        DeclareLaunchArgument('raw_command_topic', default_value='/forklift/control_cmd_raw'),
        DeclareLaunchArgument('gated_command_topic', default_value='/forklift/control_cmd'),
        DeclareLaunchArgument('recovery_twist_topic', default_value='/cmd_vel'),
        DeclareLaunchArgument('command_timeout_sec', default_value='0.5'),
        DeclareLaunchArgument('recovery_timeout_sec', default_value='0.5'),
        DeclareLaunchArgument('max_forward_velocity_mps', default_value='0.45'),
        DeclareLaunchArgument('max_reverse_velocity_mps', default_value='0.15'),
        DeclareLaunchArgument('max_recovery_velocity_mps', default_value='0.10'),
        DeclareLaunchArgument('max_recovery_angular_velocity_radps', default_value='0.30'),
        DeclareLaunchArgument(
            'max_steering_angle_rad',
            default_value='1.5707963267948966'),
        DeclareLaunchArgument('wheel_base', default_value='1.2'),
        DeclareLaunchArgument('pivot_turn_radius', default_value='0.6'),
        DeclareLaunchArgument(
            'pivot_steering_angle_rad',
            default_value='1.5707963267948966'),
        DeclareLaunchArgument('control_rate_hz', default_value='20.0'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        Node(
            package='forklift_safety',
            executable='safety_command_gate',
            name='safety_command_gate',
            output='screen',
            parameters=[{
                'enabled': enabled,
                'raw_command_topic': raw_command_topic,
                'gated_command_topic': gated_command_topic,
                'recovery_twist_topic': recovery_twist_topic,
                'command_timeout_sec': command_timeout_sec,
                'recovery_timeout_sec': recovery_timeout_sec,
                'max_forward_velocity_mps': max_forward_velocity_mps,
                'max_reverse_velocity_mps': max_reverse_velocity_mps,
                'max_recovery_velocity_mps': max_recovery_velocity_mps,
                'max_recovery_angular_velocity_radps': max_recovery_angular_velocity_radps,
                'max_steering_angle_rad': max_steering_angle_rad,
                'wheel_base': wheel_base,
                'pivot_turn_radius': pivot_turn_radius,
                'pivot_steering_angle_rad': pivot_steering_angle_rad,
                'control_rate_hz': control_rate_hz,
                'use_sim_time': use_sim_time,
            }],
        ),
    ])
