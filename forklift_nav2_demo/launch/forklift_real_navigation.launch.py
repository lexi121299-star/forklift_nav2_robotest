import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from nav2_common.launch import RewrittenYaml
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
    use_rviz = LaunchConfiguration('use_rviz')
    vehicle_dry_run = LaunchConfiguration('vehicle_dry_run')
    vehicle_model = LaunchConfiguration('vehicle_model')
    can_interface = LaunchConfiguration('can_interface')
    runtime_velocity_limit_mps = LaunchConfiguration('runtime_velocity_limit_mps')
    use_fine_motion_adapter = LaunchConfiguration('use_fine_motion_adapter')
    nav2_start_delay = LaunchConfiguration('nav2_start_delay')

    def selected_vehicle_model(context):
        selected = vehicle_model.perform(context).strip().lower()
        if selected not in {'curtis', 'xfl201'}:
            raise RuntimeError('vehicle_model must be one of: curtis, xfl201')
        return selected

    def default_nav2_params_file(selected):
        if selected == 'xfl201':
            return os.path.join(demo_share, 'config', 'xfl201_nav2_foxy.yaml')
        return os.path.join(demo_share, 'config', 'forklift_nav2_oru_test_foxy.yaml')

    def resolve_nav2_params_file(context):
        configured = params_file.perform(context).strip()
        if configured:
            return configured
        return default_nav2_params_file(selected_vehicle_model(context))

    def robot_xacro_file(selected):
        if selected == 'xfl201':
            return os.path.join(demo_share, 'urdf', 'xfl201_steered.urdf.xacro')
        return os.path.join(demo_share, 'urdf', 'forklift_diff_drive.urdf.xacro')

    def launch_robot_state_publisher(context, *args, **kwargs):
        robot_description = ParameterValue(
            Command(['xacro ', robot_xacro_file(selected_vehicle_model(context))]),
            value_type=str,
        )
        return [Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description, 'use_sim_time': False}],
        )]

    def launch_vehicle_interface(context, *args, **kwargs):
        selected = selected_vehicle_model(context)
        common_arguments = {
            'dry_run': vehicle_dry_run,
            'can_interface': can_interface,
            'publish_tf': 'true',
            'odom_frame_id': 'odom',
            'base_frame_id': 'base_link',
        }
        if selected == 'curtis':
            launch_file = 'curtis_vehicle_interface.launch.py'
        else:
            launch_file = 'xfl201_vehicle_interface.launch.py'
        return [IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(vehicle_share, 'launch', launch_file)),
            launch_arguments=common_arguments.items(),
        )]

    def launch_safety(context, *args, **kwargs):
        selected = selected_vehicle_model(context)
        footprint = footprint_from_params(resolve_nav2_params_file(context))
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
            'base_frame_id': 'base_link',
            'costmap_timeout_sec': '1.5',
            'footprint_collision_cost_threshold': '254',
            'collision_check_horizon_sec': '0.1',
            'collision_check_time_step_sec': '0.05',
            'dynamic_stop_reaction_time_sec': '0.9',
            'dynamic_stop_brake_deceleration_mps2': '1.5',
            'dynamic_stop_clearance_m': '0.5',
            'scan_protection_enabled': 'true',
            'scan_topic': '/scan',
            'scan_timeout_sec': '0.4',
            'scan_required_range_m': '8.0',
            'scan_collision_sample_spacing_m': '0.05',
            'scan_collision_padding_m': '0.05',
            'scan_require_motion_fov_coverage': 'true',
            'max_forward_velocity_mps': runtime_velocity_limit_mps.perform(context),
            'pallet_exemption_reverse_only': 'true',
            'pallet_exemption_length_m': '1.40',
            'pallet_exemption_width_m': '1.30',
        }
        if selected == 'xfl201':
            arguments.update({
                'wheel_base': '1.47',
                'pivot_turn_radius': '1.743',
                'rear_axle_x_offset': '0.0',
                'max_drive_rpm': '3000.0',
                'drive_accel_time_sec': '1.0',
                'drive_decel_time_sec': '1.0',
            })
        else:
            arguments.update({
                'wheel_base': '1.2',
                'pivot_turn_radius': '0.6',
                'rear_axle_x_offset': '-0.34',
            })
        if footprint:
            arguments['footprint'] = footprint
        return [IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(safety_share, 'launch', 'safety_command_gate.launch.py')),
            launch_arguments=arguments.items(),
        )]

    def launch_nav2(context, *args, **kwargs):
        nav2_params = RewrittenYaml(
            source_file=resolve_nav2_params_file(context),
            param_rewrites={
                'max_velocity': runtime_velocity_limit_mps.perform(context),
            },
            convert_types=True,
        )
        return [IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav2_share, 'launch', 'bringup_launch.py')),
            launch_arguments={
                'map': map_file,
                'params_file': nav2_params,
                'use_sim_time': 'false',
                'autostart': 'true',
                'use_composition': 'False',
            }.items(),
        )]

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', os.path.join(nav2_share, 'rviz', 'nav2_default_view.rviz')],
        parameters=[{'use_sim_time': False}],
        condition=IfCondition(use_rviz),
        output='screen',
    )

    fine_motion_adapter = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(vehicle_share, 'launch', 'fine_motion_adapter.launch.py')
        ),
        condition=IfCondition(use_fine_motion_adapter),
        launch_arguments={'use_sim_time': 'false'}.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'map',
            default_value='/workspace/forklift_factory_big_map_clean.yaml'),
        DeclareLaunchArgument(
            'nav2_params_file',
            default_value='',
            description=(
                'Optional Nav2 params file. Empty selects the default for '
                'vehicle_model.'
            )),
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument(
            'vehicle_dry_run',
            default_value='true',
            description='Safety default. Set false only when CAN hardware is ready.'),
        DeclareLaunchArgument(
            'vehicle_model',
            default_value='curtis',
            description='Vehicle interface model: curtis or xfl201.'),
        DeclareLaunchArgument('can_interface', default_value='can0'),
        DeclareLaunchArgument(
            'runtime_velocity_limit_mps',
            default_value='0.45',
            description=(
                'Shared forward speed ceiling for FollowPath and Safety Gate. '
                'Raise only after measured braking tests.'
            ),
        ),
        DeclareLaunchArgument(
            'use_fine_motion_adapter',
            default_value='false',
            description='Start MoveRelative and PivotRelative action servers.',
        ),
        DeclareLaunchArgument('nav2_start_delay', default_value='3.0'),
        OpaqueFunction(function=launch_robot_state_publisher),
        OpaqueFunction(function=launch_vehicle_interface),
        fine_motion_adapter,
        OpaqueFunction(function=launch_safety),
        TimerAction(period=nav2_start_delay, actions=[OpaqueFunction(function=launch_nav2), rviz]),
    ])
