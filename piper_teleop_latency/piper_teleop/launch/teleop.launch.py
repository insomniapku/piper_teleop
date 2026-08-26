from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
import os


def generate_launch_description():
    """Launch piper teleoperation node"""

    # Declare arguments
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('piper_teleop'),
            'config',
            'teleop_config.yaml'
        ]),
        description='Path to teleoperation config file'
    )

    # Teleoperation node
    teleop_node = Node(
        package='piper_teleop',
        executable='piper_teleop_node',
        name='piper_teleop_node',
        output='screen',
        parameters=[LaunchConfiguration('config_file')],
        emulate_tty=True
    )

    return LaunchDescription([
        config_file_arg,
        teleop_node
    ])
