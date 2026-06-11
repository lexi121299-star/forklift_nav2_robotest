import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    RegisterEventHandler,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_share = get_package_share_directory('forklift_nav2_demo')
    nav2_bringup_share = get_package_share_directory('nav2_bringup')
    turtlebot3_navigation_share = get_package_share_directory('turtlebot3_navigation2')

    map_file = LaunchConfiguration('map')
    nav2_params_file = LaunchConfiguration('nav2_params_file')
    use_rviz = LaunchConfiguration('use_rviz')
    use_sim_time = LaunchConfiguration('use_sim_time')
    nav2_start_delay = LaunchConfiguration('nav2_start_delay')
    gazebo_gui = LaunchConfiguration('gazebo_gui')
    use_sim_command_bridge = LaunchConfiguration('use_sim_command_bridge')
    bridge_wheel_base = LaunchConfiguration('bridge_wheel_base')
    bridge_max_velocity_mps = LaunchConfiguration('bridge_max_velocity_mps')
    bridge_max_steering_angle_rad = LaunchConfiguration('bridge_max_steering_angle_rad')
    bridge_max_angular_velocity_radps = LaunchConfiguration('bridge_max_angular_velocity_radps')
    bridge_allow_pivot_turn = LaunchConfiguration('bridge_allow_pivot_turn')
    bridge_pivot_steering_angle_rad = LaunchConfiguration('bridge_pivot_steering_angle_rad')
    bridge_pivot_steering_tolerance_rad = LaunchConfiguration(
        'bridge_pivot_steering_tolerance_rad')
    bridge_pivot_turn_radius = LaunchConfiguration('bridge_pivot_turn_radius')
    bridge_command_timeout_sec = LaunchConfiguration('bridge_command_timeout_sec')
    bridge_control_rate_hz = LaunchConfiguration('bridge_control_rate_hz')
    bridge_cmd_vel_topic = LaunchConfiguration('bridge_cmd_vel_topic')
    sim_ready_timeout = LaunchConfiguration('sim_ready_timeout')
    rmw_implementation = LaunchConfiguration('rmw_implementation')
    use_composition = LaunchConfiguration('use_composition')

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(package_share, 'launch', 'forklift_gazebo.launch.py')),
        launch_arguments={
            'gui': gazebo_gui,
            'use_sim_command_bridge': use_sim_command_bridge,
            'bridge_wheel_base': bridge_wheel_base,
            'bridge_max_velocity_mps': bridge_max_velocity_mps,
            'bridge_max_steering_angle_rad': bridge_max_steering_angle_rad,
            'bridge_max_angular_velocity_radps': bridge_max_angular_velocity_radps,
            'bridge_allow_pivot_turn': bridge_allow_pivot_turn,
            'bridge_pivot_steering_angle_rad': bridge_pivot_steering_angle_rad,
            'bridge_pivot_steering_tolerance_rad': bridge_pivot_steering_tolerance_rad,
            'bridge_pivot_turn_radius': bridge_pivot_turn_radius,
            'bridge_command_timeout_sec': bridge_command_timeout_sec,
            'bridge_control_rate_hz': bridge_control_rate_hz,
            'bridge_cmd_vel_topic': bridge_cmd_vel_topic,
        }.items(),
    )

    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_share, 'launch', 'bringup_launch.py')),
        launch_arguments={
            'map': map_file,
            'params_file': nav2_params_file,
            'use_sim_time': use_sim_time,
            'use_composition': use_composition,
        }.items(),
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', os.path.join(nav2_bringup_share, 'rviz', 'nav2_default_view.rviz')],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(use_rviz),
        output='screen',
    )

    wait_for_sim_ready = Node(
        package='forklift_nav2_demo',
        executable='wait_for_sim_ready',
        name='wait_for_sim_ready',
        output='screen',
        arguments=[
            '--odom-topic', '/odom',
            '--odom-frame', 'odom',
            '--base-frame', 'base_link',
            '--timeout', sim_ready_timeout,
        ],
        parameters=[{'use_sim_time': use_sim_time}],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'map',
            default_value=os.path.join(
                turtlebot3_navigation_share, 'map', 'map.yaml'),
            description='Map file for Nav2 localization.'),
        DeclareLaunchArgument(
            'nav2_params_file',
            default_value=os.path.join(
                package_share, 'config', 'forklift_nav2.yaml'),
            description='Nav2 parameter file.'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument(
            'nav2_start_delay',
            default_value='5.0',
            description='Seconds to wait before checking Gazebo clock, odom, and TF readiness.'),
        DeclareLaunchArgument(
            'gazebo_gui',
            default_value='true',
            description='Whether to start the Gazebo client GUI.'),
        DeclareLaunchArgument(
            'use_sim_command_bridge',
            default_value='false',
            description='Start the /forklift/control_cmd to /cmd_vel bridge with Gazebo.'),
        DeclareLaunchArgument('bridge_wheel_base', default_value='1.2'),
        DeclareLaunchArgument('bridge_max_velocity_mps', default_value='0.45'),
        DeclareLaunchArgument(
            'bridge_max_steering_angle_rad',
            default_value='1.5707963267948966'),
        DeclareLaunchArgument('bridge_max_angular_velocity_radps', default_value='0.8'),
        DeclareLaunchArgument('bridge_allow_pivot_turn', default_value='true'),
        DeclareLaunchArgument(
            'bridge_pivot_steering_angle_rad',
            default_value='1.5707963267948966'),
        DeclareLaunchArgument('bridge_pivot_steering_tolerance_rad', default_value='0.03'),
        DeclareLaunchArgument('bridge_pivot_turn_radius', default_value='0.6'),
        DeclareLaunchArgument('bridge_command_timeout_sec', default_value='0.5'),
        DeclareLaunchArgument('bridge_control_rate_hz', default_value='20.0'),
        DeclareLaunchArgument(
            'bridge_cmd_vel_topic',
            default_value='/forklift/sim_cmd_vel',
            description='Gazebo command topic used by sim_command_bridge mode.'),
        DeclareLaunchArgument(
            'sim_ready_timeout',
            default_value='0.0',
            description='Seconds before warning while waiting for readiness. 0 waits silently.'),
        DeclareLaunchArgument(
            'rmw_implementation',
            default_value='rmw_fastrtps_cpp',
            description='RMW implementation inherited by Gazebo, Nav2, RViz, and helper nodes.'),
        DeclareLaunchArgument(
            'use_composition',
            default_value='False',
            description='Whether to use Nav2 composed bringup. False is easier to debug in P0.'),
        SetEnvironmentVariable('RMW_IMPLEMENTATION', rmw_implementation),
        gazebo_launch,
        TimerAction(
            period=nav2_start_delay,
            actions=[wait_for_sim_ready],
        ),
        RegisterEventHandler(
            OnProcessExit(
                target_action=wait_for_sim_ready,
                on_exit=[nav2_launch, rviz],
            ),
        ),
    ])
