import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_share = get_package_share_directory('forklift_task_manager')
    stations_file = LaunchConfiguration('stations_file')
    routes_file = LaunchConfiguration('routes_file')
    pallet_slots_file = LaunchConfiguration('pallet_slots_file')
    approach_params_file = LaunchConfiguration('approach_params_file')
    legacy_goal_pose_enabled = LaunchConfiguration('legacy_goal_pose_enabled')
    navigation_goal_topic = LaunchConfiguration('navigation_goal_topic')
    pallet_goal_topic = LaunchConfiguration('pallet_goal_topic')
    compute_path_timeout_sec = LaunchConfiguration('compute_path_timeout_sec')
    pallet_selection_total_timeout_sec = LaunchConfiguration(
        'pallet_selection_total_timeout_sec'
    )
    hierarchical_navigation_enabled = LaunchConfiguration(
        'hierarchical_navigation_enabled'
    )
    topology_planner_id = LaunchConfiguration('topology_planner_id')
    use_sim_time = LaunchConfiguration('use_sim_time')

    return LaunchDescription([
        DeclareLaunchArgument(
            'stations_file',
            default_value=os.path.join(package_share, 'config', 'stations.yaml'),
        ),
        DeclareLaunchArgument(
            'routes_file',
            default_value=os.path.join(package_share, 'config', 'routes.yaml'),
        ),
        DeclareLaunchArgument(
            'pallet_slots_file',
            default_value=os.path.join(
                package_share, 'config', 'pallet_slots.yaml'
            ),
        ),
        DeclareLaunchArgument(
            'approach_params_file',
            default_value=os.path.join(
                package_share, 'config', 'pallet_approach.yaml'
            ),
        ),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('legacy_goal_pose_enabled', default_value='false'),
        DeclareLaunchArgument(
            'navigation_goal_topic', default_value='/forklift/navigation_goal'
        ),
        DeclareLaunchArgument(
            'pallet_goal_topic', default_value='/forklift/pallet_goal'
        ),
        DeclareLaunchArgument('compute_path_timeout_sec', default_value='12.0'),
        DeclareLaunchArgument(
            'pallet_selection_total_timeout_sec', default_value='20.0'
        ),
        DeclareLaunchArgument(
            'hierarchical_navigation_enabled', default_value='true'
        ),
        DeclareLaunchArgument('topology_planner_id', default_value='TopologyOnly'),
        Node(
            package='forklift_task_manager',
            executable='task_manager_node',
            name='forklift_task_manager',
            output='screen',
            parameters=[
                approach_params_file,
                {
                    'stations_file': stations_file,
                    'routes_file': routes_file,
                    'pallet_slots_file': pallet_slots_file,
                    'legacy_goal_pose_enabled': legacy_goal_pose_enabled,
                    'navigation_goal_topic': navigation_goal_topic,
                    'pallet_goal_topic': pallet_goal_topic,
                    'compute_path_timeout_sec': compute_path_timeout_sec,
                    'pallet_selection_total_timeout_sec': (
                        pallet_selection_total_timeout_sec
                    ),
                    'hierarchical_navigation_enabled': (
                        hierarchical_navigation_enabled
                    ),
                    'topology_planner_id': topology_planner_id,
                    'use_sim_time': use_sim_time,
                },
            ],
        ),
    ])
