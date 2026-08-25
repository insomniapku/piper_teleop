#!/usr/bin/env python3
"""
Launch file for Pico Teleop Data Collection
启动完整的数据采集系统
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution
import os


def generate_launch_description():
    """生成launch描述"""

    # 参数
    output_dir_arg = DeclareLaunchArgument(
        'output_dir',
        default_value=PathJoinSubstitution([
            EnvironmentVariable('HOME'),
            'data',
            'piper_demos',
        ]),
        description='Output directory for collected data'
    )

    urdf_path_arg = DeclareLaunchArgument(
        'urdf_path',
        default_value=EnvironmentVariable('PIPER_URDF_PATH', default_value=''),
        description='Path to Piper URDF file (or set PIPER_URDF_PATH)'
    )

    control_rate_arg = DeclareLaunchArgument(
        'control_rate',
        default_value='50',
        description='Control loop rate in Hz'
    )

    record_camera_arg = DeclareLaunchArgument(
        'record_camera',
        default_value='true',
        description='Enable camera recording'
    )

    xr_mode_arg = DeclareLaunchArgument(
        'xr_mode',
        default_value='auto',
        description='XR mode: auto, sdk, or mock'
    )

    # 获取参数值
    output_dir = LaunchConfiguration('output_dir')
    urdf_path = LaunchConfiguration('urdf_path')
    control_rate = LaunchConfiguration('control_rate')
    record_camera = LaunchConfiguration('record_camera')
    xr_mode = LaunchConfiguration('xr_mode')

    # 节点

    # 1. XR Interface Node
    xr_interface_node = Node(
        package='piper_teleop',
        executable='xr_interface_node',
        name='xr_interface',
        parameters=[{
            'mode': xr_mode,
            'publish_rate': 100,
            'tracking_timeout_ms': 1000,
        }],
        output='screen'
    )

    # 2. Data Collection Node
    data_collection_node = Node(
        package='piper_teleop',
        executable='data_collection_node',
        name='data_collection',
        parameters=[{
            'output_dir': output_dir,
            'urdf_path': urdf_path,
            'control_rate': control_rate,
            'record_camera': record_camera,
            'trigger_button': 'B',
        }],
        output='screen'
    )

    # 3. XR Monitor (可选，用于可视化)
    xr_monitor_node = Node(
        package='piper_teleop',
        executable='xr_monitor',
        name='xr_monitor',
        output='screen'
    )

    return LaunchDescription([
        # 参数声明
        output_dir_arg,
        urdf_path_arg,
        control_rate_arg,
        record_camera_arg,
        xr_mode_arg,

        # 节点
        xr_interface_node,
        data_collection_node,
        xr_monitor_node,
    ])
