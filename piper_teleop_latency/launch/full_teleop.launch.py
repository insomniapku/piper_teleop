from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
import os


def generate_launch_description():
    """Launch full teleoperation system with XR interface and control"""

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

    xr_mode_arg = DeclareLaunchArgument(
        'xr_mode',
        default_value='sdk',
        description='XR mode: sdk or mock'
    )

    urdf_path_arg = DeclareLaunchArgument(
        'urdf_path',
        default_value=EnvironmentVariable('PIPER_URDF_PATH', default_value=''),
        description='Path to Piper URDF file (or set PIPER_URDF_PATH)'
    )

    # XR Interface node (publishes controller data)
    xr_interface_node = Node(
        package='piper_teleop',
        executable='xr_interface_node',
        name='xr_interface',
        output='screen',
        parameters=[{
            'mode': LaunchConfiguration('xr_mode'),
            'publish_rate': 100,
            'tracking_timeout_ms': 1000,
        }],
        emulate_tty=True
    )

    # Full teleoperation node (subscribes to controller data and controls robot)
    full_teleop_node = Node(
        package='piper_teleop',
        executable='full_teleop_node',
        name='full_teleop_node',
        output='screen',
        parameters=[
            LaunchConfiguration('config_file'),
            {'urdf_path': LaunchConfiguration('urdf_path')},
        ],
        emulate_tty=True
    )

    return LaunchDescription([
        config_file_arg,
        xr_mode_arg,
        urdf_path_arg,
        xr_interface_node,
        full_teleop_node
    ])
