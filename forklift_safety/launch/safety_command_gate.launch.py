from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    enabled = LaunchConfiguration('enabled')
    raw_command_topic = LaunchConfiguration('raw_command_topic')
    gated_command_topic = LaunchConfiguration('gated_command_topic')
    recovery_twist_topic = LaunchConfiguration('recovery_twist_topic')
    localization_topic = LaunchConfiguration('localization_topic')
    localization_message_type = LaunchConfiguration('localization_message_type')
    costmap_topic = LaunchConfiguration('costmap_topic')
    costmap_message_type = LaunchConfiguration('costmap_message_type')
    command_timeout_sec = LaunchConfiguration('command_timeout_sec')
    recovery_timeout_sec = LaunchConfiguration('recovery_timeout_sec')
    costmap_timeout_sec = LaunchConfiguration('costmap_timeout_sec')
    costmap_monitor_enabled = LaunchConfiguration('costmap_monitor_enabled')
    collision_check_enabled = LaunchConfiguration('collision_check_enabled')
    footprint = LaunchConfiguration('footprint')
    footprint_sample_spacing = LaunchConfiguration('footprint_sample_spacing')
    footprint_collision_cost_threshold = LaunchConfiguration(
        'footprint_collision_cost_threshold')
    unknown_is_collision = LaunchConfiguration('unknown_is_collision')
    collision_check_horizon_sec = LaunchConfiguration('collision_check_horizon_sec')
    collision_check_time_step_sec = LaunchConfiguration('collision_check_time_step_sec')
    max_forward_velocity_mps = LaunchConfiguration('max_forward_velocity_mps')
    max_reverse_velocity_mps = LaunchConfiguration('max_reverse_velocity_mps')
    max_recovery_velocity_mps = LaunchConfiguration('max_recovery_velocity_mps')
    max_recovery_angular_velocity_radps = LaunchConfiguration(
        'max_recovery_angular_velocity_radps')
    max_steering_angle_rad = LaunchConfiguration('max_steering_angle_rad')
    max_drive_rpm = LaunchConfiguration('max_drive_rpm')
    drive_accel_time_sec = LaunchConfiguration('drive_accel_time_sec')
    drive_decel_time_sec = LaunchConfiguration('drive_decel_time_sec')
    wheel_base = LaunchConfiguration('wheel_base')
    pivot_turn_radius = LaunchConfiguration('pivot_turn_radius')
    rear_axle_x_offset = LaunchConfiguration('rear_axle_x_offset')
    pivot_steering_angle_rad = LaunchConfiguration('pivot_steering_angle_rad')
    control_rate_hz = LaunchConfiguration('control_rate_hz')
    use_sim_time = LaunchConfiguration('use_sim_time')

    return LaunchDescription([
        DeclareLaunchArgument('enabled', default_value='true'),
        DeclareLaunchArgument('raw_command_topic', default_value='/forklift/control_cmd_raw'),
        DeclareLaunchArgument('gated_command_topic', default_value='/forklift/control_cmd'),
        DeclareLaunchArgument('recovery_twist_topic', default_value='/cmd_vel'),
        DeclareLaunchArgument('localization_topic', default_value='/odom'),
        DeclareLaunchArgument('localization_message_type', default_value='odometry'),
        DeclareLaunchArgument('costmap_topic', default_value='/local_costmap/costmap_raw'),
        DeclareLaunchArgument('costmap_message_type', default_value='costmap_raw'),
        DeclareLaunchArgument('command_timeout_sec', default_value='0.5'),
        DeclareLaunchArgument('recovery_timeout_sec', default_value='0.5'),
        # local_costmap publishes at 2 Hz (0.5 s nominal period).  The watchdog
        # needs scheduling margin; matching the nominal period causes false
        # timeouts on every slightly late update.
        DeclareLaunchArgument('costmap_timeout_sec', default_value='1.5'),
        DeclareLaunchArgument('costmap_monitor_enabled', default_value='true'),
        DeclareLaunchArgument('collision_check_enabled', default_value='true'),
        DeclareLaunchArgument(
            'footprint',
            default_value='[[0.843, 0.58], [0.843, -0.58], [-2.043, -0.58], [-2.043, 0.58]]'),
        DeclareLaunchArgument('footprint_sample_spacing', default_value='0.05'),
        DeclareLaunchArgument('footprint_collision_cost_threshold', default_value='253'),
        DeclareLaunchArgument('unknown_is_collision', default_value='true'),
        DeclareLaunchArgument('collision_check_horizon_sec', default_value='1.0'),
        DeclareLaunchArgument('collision_check_time_step_sec', default_value='0.1'),
        DeclareLaunchArgument('max_forward_velocity_mps', default_value='0.45'),
        DeclareLaunchArgument('max_reverse_velocity_mps', default_value='0.15'),
        DeclareLaunchArgument('max_recovery_velocity_mps', default_value='0.10'),
        DeclareLaunchArgument('max_recovery_angular_velocity_radps', default_value='0.30'),
        DeclareLaunchArgument(
            'max_steering_angle_rad',
            default_value='1.5707963267948966'),
        DeclareLaunchArgument('max_drive_rpm', default_value='2485.0'),
        DeclareLaunchArgument('drive_accel_time_sec', default_value='5.0'),
        DeclareLaunchArgument('drive_decel_time_sec', default_value='3.0'),
        DeclareLaunchArgument('wheel_base', default_value='1.4'),
        DeclareLaunchArgument('pivot_turn_radius', default_value='0.6'),
        DeclareLaunchArgument('rear_axle_x_offset', default_value='-0.34'),
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
                'localization_topic': localization_topic,
                'localization_message_type': localization_message_type,
                'costmap_topic': costmap_topic,
                'costmap_message_type': costmap_message_type,
                'command_timeout_sec': command_timeout_sec,
                'recovery_timeout_sec': recovery_timeout_sec,
                'costmap_timeout_sec': costmap_timeout_sec,
                'costmap_monitor_enabled': costmap_monitor_enabled,
                'collision_check_enabled': collision_check_enabled,
                # Foxy otherwise YAML-parses the string into a nested sequence,
                # which launch_ros rejects as a non-uniform parameter array.
                'footprint': ParameterValue(footprint, value_type=str),
                'footprint_sample_spacing': footprint_sample_spacing,
                'footprint_collision_cost_threshold': footprint_collision_cost_threshold,
                'unknown_is_collision': unknown_is_collision,
                'collision_check_horizon_sec': collision_check_horizon_sec,
                'collision_check_time_step_sec': collision_check_time_step_sec,
                'max_forward_velocity_mps': max_forward_velocity_mps,
                'max_reverse_velocity_mps': max_reverse_velocity_mps,
                'max_recovery_velocity_mps': max_recovery_velocity_mps,
                'max_recovery_angular_velocity_radps': max_recovery_angular_velocity_radps,
                'max_steering_angle_rad': max_steering_angle_rad,
                'max_drive_rpm': max_drive_rpm,
                'drive_accel_time_sec': drive_accel_time_sec,
                'drive_decel_time_sec': drive_decel_time_sec,
                'wheel_base': wheel_base,
                'pivot_turn_radius': pivot_turn_radius,
                'rear_axle_x_offset': rear_axle_x_offset,
                'pivot_steering_angle_rad': pivot_steering_angle_rad,
                'control_rate_hz': control_rate_hz,
                'use_sim_time': use_sim_time,
            }],
        ),
    ])
