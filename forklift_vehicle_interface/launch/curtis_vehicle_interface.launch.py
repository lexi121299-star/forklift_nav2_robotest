from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    dry_run = LaunchConfiguration('dry_run')
    can_interface = LaunchConfiguration('can_interface')
    command_timeout_sec = LaunchConfiguration('command_timeout_sec')
    control_rate_hz = LaunchConfiguration('control_rate_hz')
    feedback_poll_rate_hz = LaunchConfiguration('feedback_poll_rate_hz')
    state_publish_rate_hz = LaunchConfiguration('state_publish_rate_hz')
    feedback_timeout_sec = LaunchConfiguration('feedback_timeout_sec')
    publish_tf = LaunchConfiguration('publish_tf')
    odom_frame_id = LaunchConfiguration('odom_frame_id')
    base_frame_id = LaunchConfiguration('base_frame_id')
    drive_wheel_radius_m = LaunchConfiguration('drive_wheel_radius_m')
    drive_gear_ratio = LaunchConfiguration('drive_gear_ratio')
    drive_track_width_m = LaunchConfiguration('drive_track_width_m')
    drive_wheel_base_m = LaunchConfiguration('drive_wheel_base_m')
    pivot_steering_angle_rad = LaunchConfiguration('pivot_steering_angle_rad')
    pivot_turn_radius_m = LaunchConfiguration('pivot_turn_radius_m')
    max_drive_rpm = LaunchConfiguration('max_drive_rpm')
    min_drive_rpm = LaunchConfiguration('min_drive_rpm')
    invert_drive_direction = LaunchConfiguration('invert_drive_direction')
    invert_feedback_drive_direction = LaunchConfiguration(
        'invert_feedback_drive_direction')
    odom_angular_scale = LaunchConfiguration('odom_angular_scale')
    invert_steering_angle = LaunchConfiguration('invert_steering_angle')

    interface = Node(
        package='forklift_vehicle_interface',
        executable='curtis_vehicle_interface',
        name='curtis_vehicle_interface',
        output='screen',
        parameters=[{
            'dry_run': dry_run,
            'can_interface': can_interface,
            'command_timeout_sec': command_timeout_sec,
            'control_rate_hz': control_rate_hz,
            'feedback_poll_rate_hz': feedback_poll_rate_hz,
            'state_publish_rate_hz': state_publish_rate_hz,
            'feedback_timeout_sec': feedback_timeout_sec,
            'publish_tf': publish_tf,
            'odom_frame_id': odom_frame_id,
            'base_frame_id': base_frame_id,
            'drive_wheel_radius_m': drive_wheel_radius_m,
            'drive_gear_ratio': drive_gear_ratio,
            'drive_track_width_m': drive_track_width_m,
            'drive_wheel_base_m': drive_wheel_base_m,
            'pivot_steering_angle_rad': pivot_steering_angle_rad,
            'pivot_turn_radius_m': pivot_turn_radius_m,
            'max_drive_rpm': max_drive_rpm,
            'min_drive_rpm': min_drive_rpm,
            'invert_drive_direction': invert_drive_direction,
            'invert_feedback_drive_direction': invert_feedback_drive_direction,
            'odom_angular_scale': odom_angular_scale,
            'invert_steering_angle': invert_steering_angle,
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument('dry_run', default_value='true'),
        DeclareLaunchArgument('can_interface', default_value='can0'),
        DeclareLaunchArgument('command_timeout_sec', default_value='0.5'),
        DeclareLaunchArgument('control_rate_hz', default_value='20.0'),
        DeclareLaunchArgument('feedback_poll_rate_hz', default_value='50.0'),
        DeclareLaunchArgument('state_publish_rate_hz', default_value='20.0'),
        DeclareLaunchArgument('feedback_timeout_sec', default_value='0.5'),
        DeclareLaunchArgument('publish_tf', default_value='true'),
        DeclareLaunchArgument('odom_frame_id', default_value='odom'),
        DeclareLaunchArgument('base_frame_id', default_value='base_link'),
        DeclareLaunchArgument('drive_wheel_radius_m', default_value='0.2285'),
        DeclareLaunchArgument('drive_gear_ratio', default_value='26.75'),
        DeclareLaunchArgument('drive_track_width_m', default_value='0.937'),
        DeclareLaunchArgument('drive_wheel_base_m', default_value='1.4'),
        DeclareLaunchArgument('pivot_steering_angle_rad', default_value='1.5707963267948966'),
        DeclareLaunchArgument('pivot_turn_radius_m', default_value='0.6'),
        DeclareLaunchArgument('max_drive_rpm', default_value='2485.0'),
        DeclareLaunchArgument('min_drive_rpm', default_value='100.0'),
        DeclareLaunchArgument('invert_drive_direction', default_value='false'),
        DeclareLaunchArgument(
            'invert_feedback_drive_direction',
            default_value='false',
            description='Invert decoded drive RPM before odometry integration.'),
        DeclareLaunchArgument(
            'odom_angular_scale',
            default_value='1.0',
            description=(
                'Scale the odometry yaw rate independently from longitudinal '
                'drive feedback; use -1.0 to invert only yaw.')),
        DeclareLaunchArgument('invert_steering_angle', default_value='false'),
        interface,
    ])
