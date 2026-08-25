#!/usr/bin/env python3
"""
Data Collection Launch Node
启动完整的Pico遥操作数据采集系统
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState, Image, CameraInfo
from geometry_msgs.msg import Pose, PoseStamped
from std_msgs.msg import Bool, Float32
import numpy as np
import h5py
import os
import time
from datetime import datetime
from typing import Optional, Dict, List
import cv2
from cv_bridge import CvBridge

from piper_teleop.xr_interface import XRInterface
from piper_teleop.teleop_mapping import TeleopMapper
from piper_teleop.ik_solver import PiperIKSolver
from piper_teleop.teleop_controller import PiperTeleopController


class DataCollectionNode(Node):
    """
    数据采集节点

    功能:
    1. 订阅XR输入
    2. 计算IK并发送控制命令
    3. 记录所有传感器数据
    4. 保存为HDF5格式
    """

    def __init__(self):
        super().__init__('data_collection_node')

        # 参数
        self.declare_parameter(
            'output_dir',
            os.path.join(os.path.expanduser('~'), 'data', 'piper_demos')
        )
        self.declare_parameter(
            'urdf_path',
            os.environ.get('PIPER_URDF_PATH', '')
        )
        self.declare_parameter('control_rate', 50)
        self.declare_parameter('record_camera', True)
        self.declare_parameter('trigger_button', 'B')

        self.output_dir = self.get_parameter('output_dir').value
        self.urdf_path = self.get_parameter('urdf_path').value
        self.control_rate = self.get_parameter('control_rate').value
        self.record_camera = self.get_parameter('record_camera').value
        self.trigger_button = self.get_parameter('trigger_button').value

        if not os.path.isfile(self.urdf_path):
            raise FileNotFoundError(
                'Set the urdf_path ROS parameter or PIPER_URDF_PATH to a Piper URDF file.'
            )

        # 创建输出目录
        os.makedirs(self.output_dir, exist_ok=True)

        # 初始化遥操作控制器
        self.controller = PiperTeleopController(
            self.urdf_path,
            control_frequency=self.control_rate
        )

        # 数据缓冲区
        self.is_recording = False
        self.episode_number = self._get_next_episode_number()
        self.data_buffer = {
            'timestamps': [],
            'joint_positions': [],
            'joint_velocities': [],
            'joint_torques': [],
            'joint_commands': [],
            'ee_position': [],
            'ee_orientation': [],
            'gripper_position': [],
            'vr_right_position': [],
            'vr_right_orientation': [],
            'vr_left_position': [],
            'vr_left_orientation': [],
            'camera_color': [],
            'camera_depth': [],
            'camera_timestamps': [],
        }

        # ROS订阅
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states_feedback',
            self._joint_state_callback,
            10
        )

        self.end_pose_sub = self.create_subscription(
            PoseStamped,
            '/end_pose_stamped',
            self._end_pose_callback,
            10
        )

        if self.record_camera:
            self.bridge = CvBridge()
            self.camera_color_sub = self.create_subscription(
                Image,
                '/camera/color/image_raw',
                self._camera_color_callback,
                10
            )
            self.camera_depth_sub = self.create_subscription(
                Image,
                '/camera/depth/image_rect_raw',
                self._camera_depth_callback,
                10
            )

        # ROS发布
        self.joint_cmd_pub = self.create_publisher(
            JointState,
            '/joint_ctrl_single',
            10
        )

        self.recording_status_pub = self.create_publisher(
            Bool,
            '/data_collection/recording_status',
            10
        )

        # 状态变量
        self.last_joint_state = None
        self.last_end_pose = None
        self.last_camera_color = None
        self.last_camera_depth = None
        self.last_button_state = False
        self.start_time = 0

        # 主控制循环
        self.control_timer = self.create_timer(
            1.0 / self.control_rate,
            self._control_loop
        )

        self.get_logger().info('='*60)
        self.get_logger().info('Data Collection Node Started')
        self.get_logger().info('='*60)
        self.get_logger().info(f'Output directory: {self.output_dir}')
        self.get_logger().info(f'Next episode: {self.episode_number:03d}')
        self.get_logger().info(f'Control rate: {self.control_rate} Hz')
        self.get_logger().info(f'Camera recording: {self.record_camera}')
        self.get_logger().info(f'Trigger button: {self.trigger_button}')
        self.get_logger().info('='*60)
        self.get_logger().info(f'Press {self.trigger_button} button to start/stop recording')
        self.get_logger().info('='*60)

    def _get_next_episode_number(self) -> int:
        """获取下一个episode编号"""
        if not os.path.exists(self.output_dir):
            return 1

        existing = [f for f in os.listdir(self.output_dir) if f.startswith('episode_') and f.endswith('.h5')]
        if not existing:
            return 1

        numbers = [int(f.split('_')[1].split('.')[0]) for f in existing]
        return max(numbers) + 1

    def _joint_state_callback(self, msg: JointState):
        """接收关节状态"""
        self.last_joint_state = msg

        if self.is_recording:
            # 记录数据
            self.data_buffer['timestamps'].append(time.time() - self.start_time)
            self.data_buffer['joint_positions'].append(list(msg.position))
            self.data_buffer['joint_velocities'].append(list(msg.velocity) if msg.velocity else [0]*6)
            self.data_buffer['joint_torques'].append(list(msg.effort) if msg.effort else [0]*6)

    def _end_pose_callback(self, msg: PoseStamped):
        """接收末端位姿"""
        self.last_end_pose = msg

        if self.is_recording:
            pos = msg.pose.position
            ori = msg.pose.orientation
            self.data_buffer['ee_position'].append([pos.x, pos.y, pos.z])
            self.data_buffer['ee_orientation'].append([ori.x, ori.y, ori.z, ori.w])

    def _camera_color_callback(self, msg: Image):
        """接收彩色图像"""
        if self.is_recording:
            try:
                cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
                self.data_buffer['camera_color'].append(cv_image)
                self.data_buffer['camera_timestamps'].append(time.time() - self.start_time)
            except Exception as e:
                self.get_logger().warn(f'Failed to convert color image: {e}')

    def _camera_depth_callback(self, msg: Image):
        """接收深度图像"""
        if self.is_recording:
            try:
                cv_image = self.bridge.imgmsg_to_cv2(msg, 'passthrough')
                self.data_buffer['camera_depth'].append(cv_image)
            except Exception as e:
                self.get_logger().warn(f'Failed to convert depth image: {e}')

    def _control_loop(self):
        """主控制循环"""
        # TODO: 从XR interface获取VR数据
        # 这里使用mock数据作为示例
        vr_pos = np.array([0.0, 0.0, -0.3])
        vr_quat = np.array([0.0, 0.0, 0.0, 1.0])

        # 检查录制按钮
        button_pressed = False  # TODO: 从XR interface获取按钮状态

        if button_pressed and not self.last_button_state:
            # 按钮按下
            if not self.is_recording:
                self._start_recording()
            else:
                self._stop_recording()

        self.last_button_state = button_pressed

        # 如果正在录制，记录VR数据
        if self.is_recording:
            self.data_buffer['vr_right_position'].append(list(vr_pos))
            self.data_buffer['vr_right_orientation'].append(list(vr_quat))

        # 发布录制状态
        status_msg = Bool()
        status_msg.data = self.is_recording
        self.recording_status_pub.publish(status_msg)

    def _start_recording(self):
        """开始录制"""
        self.is_recording = True
        self.start_time = time.time()

        # 清空缓冲区
        for key in self.data_buffer:
            self.data_buffer[key] = []

        self.get_logger().info('='*60)
        self.get_logger().info(f'🔴 RECORDING STARTED - Episode {self.episode_number:03d}')
        self.get_logger().info('='*60)

    def _stop_recording(self):
        """停止录制并保存数据"""
        self.is_recording = False
        duration = time.time() - self.start_time

        self.get_logger().info('='*60)
        self.get_logger().info(f'⏹️  RECORDING STOPPED - Duration: {duration:.2f}s')
        self.get_logger().info('='*60)

        # 保存数据
        filename = os.path.join(self.output_dir, f'episode_{self.episode_number:03d}.h5')
        self._save_episode(filename)

        self.get_logger().info(f'✅ Data saved to: {filename}')
        self.get_logger().info(f'   Samples: {len(self.data_buffer["timestamps"])}')
        self.get_logger().info(f'   Duration: {duration:.2f}s')
        self.get_logger().info(f'   Rate: {len(self.data_buffer["timestamps"])/duration:.1f} Hz')
        self.get_logger().info('='*60)

        # 准备下一个episode
        self.episode_number += 1

    def _save_episode(self, filename: str):
        """保存episode到HDF5文件"""
        with h5py.File(filename, 'w') as f:
            # 保存时间序列数据
            for key, data in self.data_buffer.items():
                if len(data) > 0:
                    if key == 'camera_color' or key == 'camera_depth':
                        # 图像数据需要堆叠
                        f.create_dataset(key, data=np.array(data), compression='gzip')
                    else:
                        f.create_dataset(key, data=np.array(data))

            # 保存元数据
            f.attrs['episode_number'] = self.episode_number
            f.attrs['date'] = datetime.now().isoformat()
            f.attrs['control_rate'] = self.control_rate
            f.attrs['duration'] = time.time() - self.start_time
            f.attrs['robot'] = 'Piper'


def main(args=None):
    rclpy.init(args=args)
    node = DataCollectionNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down data collection node')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
