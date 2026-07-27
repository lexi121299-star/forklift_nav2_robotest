from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    action_name = LaunchConfiguration('action_name')
    command_topic = LaunchConfiguration('command_topic')
    fork_state_topic = LaunchConfiguration('fork_state_topic')

    return LaunchDescription([
        DeclareLaunchArgument('action_name', default_value='/forklift/fork/move_to'),
        DeclareLaunchArgument('command_topic', default_value='/forklift/control_cmd_raw'),
        DeclareLaunchArgument(
            'fork_state_topic',
            default_value='/forklift/fork/joint_state',
        ),
        Node(
            package='forklift_vehicle_interface',
            executable='fork_control_adapter',
            name='fork_control_adapter',
            output='screen',
            parameters=[{
                'action_name': action_name,
                'command_topic': command_topic,
                'fork_state_topic': fork_state_topic,
            }],
        ),
    ])
