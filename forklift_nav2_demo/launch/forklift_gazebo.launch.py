import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    package_share = get_package_share_directory('forklift_nav2_demo')
    gazebo_ros_share = get_package_share_directory('gazebo_ros')

    world = LaunchConfiguration('world')
    x_pose = LaunchConfiguration('x_pose')
    y_pose = LaunchConfiguration('y_pose')
    z_pose = LaunchConfiguration('z_pose')
    yaw = LaunchConfiguration('yaw')
    gui = LaunchConfiguration('gui')
    use_sim_command_bridge = LaunchConfiguration('use_sim_command_bridge')
    bridge_wheel_base = LaunchConfiguration('bridge_wheel_base')
    bridge_max_velocity_mps = LaunchConfiguration('bridge_max_velocity_mps')
    bridge_max_steering_angle_rad = LaunchConfiguration('bridge_max_steering_angle_rad')
    bridge_command_timeout_sec = LaunchConfiguration('bridge_command_timeout_sec')
    bridge_control_rate_hz = LaunchConfiguration('bridge_control_rate_hz')
    bridge_cmd_vel_topic = LaunchConfiguration('bridge_cmd_vel_topic')

    urdf_file = os.path.join(package_share, 'urdf', 'forklift_diff_drive.urdf.xacro')
    robot_description = ParameterValue(
        Command([
            'xacro ',
            urdf_file,
        ]),
        value_type=str,
    )
    bridge_robot_description = ParameterValue(
        Command([
            'xacro ',
            urdf_file,
            ' gazebo_cmd_vel_topic:=',
            bridge_cmd_vel_topic,
        ]),
        value_type=str,
    )

    gzserver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_share, 'launch', 'gzserver.launch.py')),
        launch_arguments={
            'world': world,
            'init': 'true',
            'factory': 'true',
            'force_system': 'true',
        }.items(),
    )

    gzclient = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_share, 'launch', 'gzclient.launch.py')),
        condition=IfCondition(gui),
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        condition=UnlessCondition(use_sim_command_bridge),
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': True,
        }],
    )

    bridge_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        condition=IfCondition(use_sim_command_bridge),
        parameters=[{
            'robot_description': bridge_robot_description,
            'use_sim_time': True,
        }],
    )

    spawn_robot = Node(
        package='forklift_nav2_demo',
        executable='spawn_forklift_entity',
        arguments=[
            '--entity', 'forklift',
            '--x', x_pose,
            '--y', y_pose,
            '--z', z_pose,
            '--yaw', yaw,
        ],
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': True,
        }],
        condition=UnlessCondition(use_sim_command_bridge),
        output='screen',
    )

    bridge_spawn_robot = Node(
        package='forklift_nav2_demo',
        executable='spawn_forklift_entity',
        arguments=[
            '--entity', 'forklift',
            '--x', x_pose,
            '--y', y_pose,
            '--z', z_pose,
            '--yaw', yaw,
        ],
        parameters=[{
            'robot_description': bridge_robot_description,
            'use_sim_time': True,
        }],
        condition=IfCondition(use_sim_command_bridge),
        output='screen',
    )

    sim_command_bridge = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('forklift_vehicle_interface'),
                'launch',
                'sim_command_bridge.launch.py')),
        condition=IfCondition(use_sim_command_bridge),
        launch_arguments={
            'wheel_base': bridge_wheel_base,
            'max_velocity_mps': bridge_max_velocity_mps,
            'max_steering_angle_rad': bridge_max_steering_angle_rad,
            'command_timeout_sec': bridge_command_timeout_sec,
            'control_rate_hz': bridge_control_rate_hz,
            'cmd_vel_topic': bridge_cmd_vel_topic,
        }.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'world',
            default_value=os.path.join(
                package_share, 'worlds', 'forklift_warehouse.world'),
            description='Gazebo world file.'),
        DeclareLaunchArgument('x_pose', default_value='-2.0'),
        DeclareLaunchArgument('y_pose', default_value='-0.5'),
        DeclareLaunchArgument('z_pose', default_value='0.05'),
        DeclareLaunchArgument('yaw', default_value='0.0'),
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument(
            'use_sim_command_bridge',
            default_value='false',
            description='Start the simulation bridge from /forklift/control_cmd to /cmd_vel.'),
        DeclareLaunchArgument('bridge_wheel_base', default_value='1.2'),
        DeclareLaunchArgument('bridge_max_velocity_mps', default_value='0.45'),
        DeclareLaunchArgument('bridge_max_steering_angle_rad', default_value='0.55'),
        DeclareLaunchArgument('bridge_command_timeout_sec', default_value='0.5'),
        DeclareLaunchArgument('bridge_control_rate_hz', default_value='20.0'),
        DeclareLaunchArgument(
            'bridge_cmd_vel_topic',
            default_value='/forklift/sim_cmd_vel',
            description='Gazebo command topic used by sim_command_bridge mode.'),
        gzserver,
        gzclient,
        robot_state_publisher,
        bridge_robot_state_publisher,
        spawn_robot,
        bridge_spawn_robot,
        sim_command_bridge,
    ])
