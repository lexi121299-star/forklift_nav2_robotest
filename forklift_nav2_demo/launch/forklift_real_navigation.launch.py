import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
import yaml


def footprint_from_params(params_file):
    try:
        with open(params_file, 'r') as stream:
            params = yaml.safe_load(stream) or {}
    except (OSError, yaml.YAMLError):
        return ''

    footprint = (
        params.get('local_costmap', {})
        .get('local_costmap', {})
        .get('ros__parameters', {})
        .get('footprint')
    )
    return footprint if isinstance(footprint, str) and footprint.strip() else ''


def generate_launch_description():
    demo_share = get_package_share_directory('forklift_nav2_demo')
    nav2_share = get_package_share_directory('nav2_bringup')
    vehicle_share = get_package_share_directory('forklift_vehicle_interface')
    safety_share = get_package_share_directory('forklift_safety')

    map_file = LaunchConfiguration('map')
    params_file = LaunchConfiguration('nav2_params_file')
    default_bt_xml_filename = LaunchConfiguration('default_bt_xml_filename')
    use_rviz = LaunchConfiguration('use_rviz')
    rviz_config = LaunchConfiguration('rviz_config')
    vehicle_dry_run = LaunchConfiguration('vehicle_dry_run')
    can_interface = LaunchConfiguration('can_interface')
    invert_drive_direction = LaunchConfiguration('invert_drive_direction')
    invert_feedback_drive_direction = LaunchConfiguration(
        'invert_feedback_drive_direction')
    odom_angular_scale = LaunchConfiguration('odom_angular_scale')
    invert_steering_angle = LaunchConfiguration('invert_steering_angle')
    use_fine_motion_adapter = LaunchConfiguration('use_fine_motion_adapter')
    nav2_start_delay = LaunchConfiguration('nav2_start_delay')
    use_localization_adapter = LaunchConfiguration('use_localization_adapter')
    localization_topic = LaunchConfiguration('localization_topic')
    odom_topic = LaunchConfiguration('odom_topic')
    map_frame_id = LaunchConfiguration('map_frame_id')
    odom_frame_id = LaunchConfiguration('odom_frame_id')
    localization_base_frame_id = LaunchConfiguration('localization_base_frame_id')
    odom_base_frame_id = LaunchConfiguration('odom_base_frame_id')
    base_frame_id = LaunchConfiguration('base_frame_id')
    localization_adapter_stamp_with_current_time = LaunchConfiguration(
        'localization_adapter_stamp_with_current_time')
    localization_adapter_publish_rate_hz = LaunchConfiguration(
        'localization_adapter_publish_rate_hz')
    localization_offset_x_m = LaunchConfiguration('localization_offset_x_m')
    localization_offset_y_m = LaunchConfiguration('localization_offset_y_m')
    localization_offset_yaw_rad = LaunchConfiguration('localization_offset_yaw_rad')

    robot_description = ParameterValue(
        Command([
            'xacro ',
            os.path.join(demo_share, 'urdf', 'forklift_diff_drive.urdf.xacro'),
        ]),
        value_type=str,
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description, 'use_sim_time': False}],
    )

    vehicle_interface = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(vehicle_share, 'launch', 'curtis_vehicle_interface.launch.py')),
        launch_arguments={
            'dry_run': vehicle_dry_run,
            'can_interface': can_interface,
            'publish_tf': 'true',
            'odom_frame_id': odom_frame_id,
            'base_frame_id': odom_base_frame_id,
            'invert_drive_direction': invert_drive_direction,
            'invert_feedback_drive_direction': invert_feedback_drive_direction,
            'odom_angular_scale': odom_angular_scale,
            'invert_steering_angle': invert_steering_angle,
        }.items(),
    )

    fine_motion_adapter = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(vehicle_share, 'launch', 'fine_motion_adapter.launch.py')),
        condition=IfCondition(use_fine_motion_adapter),
        launch_arguments={'use_sim_time': 'false'}.items(),
    )

    localization_adapter = Node(
        package='forklift_nav2_demo',
        executable='map_odom_localization_adapter',
        name='map_odom_localization_adapter',
        output='screen',
        condition=IfCondition(use_localization_adapter),
        parameters=[{
            'use_sim_time': False,
            'localization_topic': localization_topic,
            'odom_topic': odom_topic,
            'map_frame_id': map_frame_id,
            'odom_frame_id': odom_frame_id,
            'localization_base_frame_id': localization_base_frame_id,
            'odom_base_frame_id': odom_base_frame_id,
            'base_frame_id': base_frame_id,
            'stamp_with_current_time': localization_adapter_stamp_with_current_time,
            'publish_rate_hz': localization_adapter_publish_rate_hz,
            'localization_offset_x_m': localization_offset_x_m,
            'localization_offset_y_m': localization_offset_y_m,
            'localization_offset_yaw_rad': localization_offset_yaw_rad,
        }],
    )

    def launch_safety(context, *args, **kwargs):
        footprint = footprint_from_params(params_file.perform(context))
        arguments = {
            'use_sim_time': 'false',
            'enabled': 'true',
            'collision_check_enabled': 'true',
            'costmap_monitor_enabled': 'true',
            'raw_command_topic': '/forklift/control_cmd_raw',
            'gated_command_topic': '/forklift/control_cmd',
            'recovery_twist_topic': '/cmd_vel',
            'localization_topic': '/odom',
            'localization_message_type': 'odometry',
            'costmap_timeout_sec': '1.5',
            'collision_check_horizon_sec': '0.6',
            'collision_check_time_step_sec': '0.1',
            'pallet_exemption_reverse_only': 'false',
            'wheel_base': '1.2',
            'rear_axle_x_offset': '0.0',
        }
        if footprint:
            arguments['footprint'] = footprint
        return [IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(safety_share, 'launch', 'safety_command_gate.launch.py')),
            launch_arguments=arguments.items(),
        )]

    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[params_file, {
            'use_sim_time': False,
            'yaml_filename': map_file,
        }],
        remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')],
    )

    lifecycle_manager_map = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_map',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'autostart': True,
            'node_names': ['map_server'],
        }],
    )

    nav2_navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_share, 'launch', 'navigation_launch.py')),
        launch_arguments={
            'params_file': params_file,
            'use_sim_time': 'false',
            'autostart': 'true',
            'map_subscribe_transient_local': 'true',
            'default_bt_xml_filename': default_bt_xml_filename,
        }.items(),
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': False}],
        condition=IfCondition(use_rviz),
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'map',
            default_value='/workspace/forklift_factory_big_map_clean.yaml'),
        DeclareLaunchArgument(
            'nav2_params_file',
            default_value=os.path.join(
                demo_share, 'config', 'forklift_nav2_real_external_localization_foxy.yaml')),
        DeclareLaunchArgument(
            'default_bt_xml_filename',
            default_value=os.path.join(
                demo_share, 'behavior_trees', 'forklift_oru_plan_once_with_recovery.xml'),
            description='Plan-once navigation tree used by the real forklift.'),
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument(
            'rviz_config',
            default_value=os.path.join(
                demo_share, 'rviz', 'forklift_real_light.rviz'),
            description='RViz config file. The default is a lightweight real-vehicle view.'),
        DeclareLaunchArgument(
            'vehicle_dry_run',
            default_value='true',
            description='Safety default. Set false only when CAN hardware is ready.'),
        DeclareLaunchArgument('can_interface', default_value='can0'),
        DeclareLaunchArgument(
            'invert_drive_direction',
            default_value='false',
            description='Swap forward/reverse bits sent to the Curtis controller.'),
        DeclareLaunchArgument(
            'invert_feedback_drive_direction',
            default_value='true',
            description=(
                'Invert decoded drive RPM for odometry without changing the '
                'command sent to the Curtis controller.')),
        DeclareLaunchArgument(
            'odom_angular_scale',
            default_value='-1.0',
            description=(
                'Correct the real vehicle odometry yaw direction independently '
                'from its longitudinal feedback direction.')),
        DeclareLaunchArgument(
            'invert_steering_angle',
            default_value='false',
            description='Invert steering angle sent to the Curtis controller.'),
        DeclareLaunchArgument(
            'use_fine_motion_adapter',
            default_value='false',
            description=(
                'Start the low-speed MoveRelative action server. Enable it for '
                'Task Manager pallet approach tests.')),
        DeclareLaunchArgument('nav2_start_delay', default_value='3.0'),
        DeclareLaunchArgument(
            'use_localization_adapter',
            default_value='false',
            description=(
                'Start adapter for external map->base_link Odometry on '
                'localization_topic and publish map->odom TF. Keep false if '
                'the localization system already publishes map->odom.')),
        DeclareLaunchArgument(
            'localization_topic',
            default_value='/fusion/localization',
            description=(
                'External localization Odometry topic. Used by the adapter when '
                'use_localization_adapter=true.')),
        DeclareLaunchArgument(
            'odom_topic',
            default_value='/odom',
            description='Vehicle odometry topic used by the localization adapter.'),
        DeclareLaunchArgument('map_frame_id', default_value='map'),
        DeclareLaunchArgument('odom_frame_id', default_value='odom'),
        DeclareLaunchArgument(
            'localization_base_frame_id',
            default_value='base_link',
            description='Base frame used by external localization messages.'),
        DeclareLaunchArgument(
            'odom_base_frame_id',
            default_value='base_footprint',
            description='Base frame used by vehicle odometry and odom TF.'),
        DeclareLaunchArgument(
            'base_frame_id',
            default_value='base_link',
            description='Navigation robot base frame used by Nav2.'),
        DeclareLaunchArgument(
            'localization_adapter_stamp_with_current_time',
            default_value='true',
            description='Stamp adapter-published map->odom TF with current ROS time.'),
        DeclareLaunchArgument(
            'localization_adapter_publish_rate_hz',
            default_value='20.0',
            description='Periodic map->odom TF publish rate for the localization adapter.'),
        DeclareLaunchArgument(
            'localization_offset_x_m',
            default_value='0.0',
            description='Temporary map-frame x offset applied to external localization.'),
        DeclareLaunchArgument(
            'localization_offset_y_m',
            default_value='0.0',
            description='Temporary map-frame y offset applied to external localization.'),
        DeclareLaunchArgument(
            'localization_offset_yaw_rad',
            default_value='0.0',
            description='Temporary yaw offset applied to external localization.'),
        robot_state_publisher,
        vehicle_interface,
        fine_motion_adapter,
        localization_adapter,
        OpaqueFunction(function=launch_safety),
        map_server,
        lifecycle_manager_map,
        TimerAction(period=nav2_start_delay, actions=[nav2_navigation, rviz]),
    ])
