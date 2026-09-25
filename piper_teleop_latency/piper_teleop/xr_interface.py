#!/usr/bin/env python3
"""
XR Input Interface for Pico Controller (M2)
Provides XR controller data via:
1. xrobotoolkit_sdk (if available) - real Pico XR device
2. Mock data - for testing without XR hardware
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import numpy as np
import time
from typing import Optional

from piper_teleop.xr_data_types import XRControllerState, XRHeadsetState, XRData

# Try to import xrobotoolkit SDK
try:
    import xrobotoolkit_sdk as xrt
    XRT_AVAILABLE = True
except ImportError:
    XRT_AVAILABLE = False


class XRInterface(Node):
    """
    ROS2 Node for XR input interface

    Modes:
    - 'sdk': Use xrobotoolkit_sdk (requires real Pico XR)
    - 'mock': Use mock data for testing
    """

    def __init__(self, mode: str = 'auto'):
        super().__init__('xr_interface')

        # Parameters
        self.declare_parameter('mode', mode)
        self.declare_parameter('tracking_timeout_ms', 1000)  # 1 second
        self.declare_parameter('publish_rate', 100)  # Hz

        mode = self.get_parameter('mode').value
        self.tracking_timeout_ms = self.get_parameter('tracking_timeout_ms').value
        self.publish_rate = self.get_parameter('publish_rate').value

        # Auto-detect mode
        if mode == 'auto':
            if XRT_AVAILABLE:
                mode = 'sdk'
                self.get_logger().info('xrobotoolkit_sdk detected, using SDK mode')
            else:
                mode = 'mock'
                self.get_logger().warn('xrobotoolkit_sdk not available, using MOCK mode')

        self.mode = mode

        # XR data state
        self.xr_data = XRData()
        self.last_update_time = 0

        # Publishers for debug/visualization
        self.right_controller_pose_pub = self.create_publisher(
            PoseStamped,
            '/xr/right_controller/pose',
            10
        )
        self.left_controller_pose_pub = self.create_publisher(
            PoseStamped,
            '/xr/left_controller/pose',
            10
        )
        self.headset_pose_pub = self.create_publisher(
            PoseStamped,
            '/xr/headset/pose',
            10
        )

        # Initialize based on mode
        if self.mode == 'sdk':
            self._init_sdk_mode()
        elif self.mode == 'mock':
            self._init_mock_mode()
        else:
            raise ValueError(f"Invalid mode: {mode}. Must be 'sdk', 'mock', or 'auto'")

        # Timer for updating XR data
        self.update_timer = self.create_timer(
            1.0 / self.publish_rate,
            self._update_callback
        )

        self.get_logger().info(f'XRInterface initialized in {self.mode.upper()} mode')
        self.get_logger().info(f'Tracking timeout: {self.tracking_timeout_ms} ms')
        self.get_logger().info(f'Update rate: {self.publish_rate} Hz')

    def _init_sdk_mode(self):
        """Initialize SDK mode with real XR device"""
        try:
            xrt.init()
            self.get_logger().info('xrobotoolkit_sdk initialized successfully')
        except Exception as e:
            self.get_logger().error(f'Failed to initialize xrobotoolkit_sdk: {e}')
            self.get_logger().warn('Falling back to MOCK mode')
            self.mode = 'mock'
            self._init_mock_mode()

    def _init_mock_mode(self):
        """Initialize mock mode for testing"""
        self.get_logger().warn('=' * 60)
        self.get_logger().warn('MOCK MODE: Using simulated XR data')
        self.get_logger().warn('For real XR control, install xrobotoolkit_sdk')
        self.get_logger().warn('=' * 60)

        # Mock data starts at a reasonable pose
        self.mock_time = 0.0
        self.mock_grip_active = False
        self.mock_trigger_value = 0.0

    def _update_callback(self):
        """Main update callback - reads XR data and publishes"""
        if self.mode == 'sdk':
            self._update_from_sdk()
        elif self.mode == 'mock':
            self._update_from_mock()

        # Publish poses for visualization
        self._publish_poses()

        # Check for tracking timeout
        self._check_tracking_timeout()

    def _update_from_sdk(self):
        """Update XR data from SDK"""
        try:
            # Get timestamp
            timestamp_ns = xrt.get_time_stamp_ns()
            self.xr_data.timestamp_ns = timestamp_ns
            self.last_update_time = time.time_ns() // 1000000  # Convert to ms

            # Right controller
            right_pose = xrt.get_right_controller_pose()  # [x, y, z, qx, qy, qz, qw]
            self.xr_data.right_controller = XRControllerState(
                position=np.array(right_pose[:3]),
                quaternion=np.array([right_pose[3], right_pose[4], right_pose[5], right_pose[6]]),
                grip_value=xrt.get_right_grip(),
                trigger_value=xrt.get_right_trigger(),
                button_a=xrt.get_A_button(),
                button_b=xrt.get_B_button(),
                menu_button=xrt.get_right_menu_button(),
                axis_click=xrt.get_right_axis_click(),
                joystick_x=xrt.get_right_axis()[0],
                joystick_y=xrt.get_right_axis()[1],
                tracking_valid=True,
                timestamp_ns=timestamp_ns
            )

            # Left controller
            left_pose = xrt.get_left_controller_pose()
            self.xr_data.left_controller = XRControllerState(
                position=np.array(left_pose[:3]),
                quaternion=np.array([left_pose[3], left_pose[4], left_pose[5], left_pose[6]]),
                grip_value=xrt.get_left_grip(),
                trigger_value=xrt.get_left_trigger(),
                button_x=xrt.get_X_button(),
                button_y=xrt.get_Y_button(),
                menu_button=xrt.get_left_menu_button(),
                axis_click=xrt.get_left_axis_click(),
                joystick_x=xrt.get_left_axis()[0],
                joystick_y=xrt.get_left_axis()[1],
                tracking_valid=True,
                timestamp_ns=timestamp_ns
            )

            # Headset
            headset_pose = xrt.get_headset_pose()
            self.xr_data.headset = XRHeadsetState(
                position=np.array(headset_pose[:3]),
                quaternion=np.array([headset_pose[3], headset_pose[4], headset_pose[5], headset_pose[6]]),
                tracking_valid=True,
                timestamp_ns=timestamp_ns
            )

        except Exception as e:
            self.get_logger().error(f'Error reading XR SDK data: {e}')
            self._mark_tracking_invalid()

    def _update_from_mock(self):
        """Update XR data from mock source"""
        # Generate mock data for testing
        self.mock_time += 1.0 / self.publish_rate
        timestamp_ns = int(self.mock_time * 1e9)
        self.last_update_time = time.time_ns() // 1000000

        # Mock right controller - simple circular motion for testing
        t = self.mock_time
        base_pos = np.array([0.3, 0.0, 0.4])
        offset = np.array([0.05 * np.sin(t), 0.05 * np.cos(t), 0.02 * np.sin(2*t)])

        # Mock grip: toggle every 5 seconds
        self.mock_grip_active = (int(t) % 10) < 5
        grip_value = 1.0 if self.mock_grip_active else 0.0

        # Mock trigger: sine wave
        self.mock_trigger_value = 0.5 + 0.5 * np.sin(t * 0.5)

        self.xr_data.right_controller = XRControllerState(
            position=base_pos + offset,
            quaternion=np.array([0.0, 0.0, 0.0, 1.0]),  # Identity quaternion
            grip_value=grip_value,
            trigger_value=self.mock_trigger_value,
            button_a=False,
            button_b=False,
            menu_button=False,
            axis_click=False,
            joystick_x=0.0,
            joystick_y=0.0,
            tracking_valid=True,
            timestamp_ns=timestamp_ns
        )

        # Mock left controller (static for now)
        self.xr_data.left_controller = XRControllerState(
            position=np.array([0.3, 0.3, 0.4]),
            quaternion=np.array([0.0, 0.0, 0.0, 1.0]),
            grip_value=0.0,
            trigger_value=0.0,
            tracking_valid=True,
            timestamp_ns=timestamp_ns
        )

        # Mock headset
        self.xr_data.headset = XRHeadsetState(
            position=np.array([0.0, 0.0, 1.6]),
            quaternion=np.array([0.0, 0.0, 0.0, 1.0]),
            tracking_valid=True,
            timestamp_ns=timestamp_ns
        )

        self.xr_data.timestamp_ns = timestamp_ns

    def _publish_poses(self):
        """Publish poses for visualization"""
        current_time = self.get_clock().now()

        # Right controller
        if self.xr_data.right_controller is not None:
            msg = PoseStamped()
            msg.header.stamp = current_time.to_msg()
            msg.header.frame_id = 'xr_world'
            msg.pose.position.x = self.xr_data.right_controller.position[0]
            msg.pose.position.y = self.xr_data.right_controller.position[1]
            msg.pose.position.z = self.xr_data.right_controller.position[2]
            msg.pose.orientation.x = self.xr_data.right_controller.quaternion[0]
            msg.pose.orientation.y = self.xr_data.right_controller.quaternion[1]
            msg.pose.orientation.z = self.xr_data.right_controller.quaternion[2]
            msg.pose.orientation.w = self.xr_data.right_controller.quaternion[3]
            self.right_controller_pose_pub.publish(msg)

        # Left controller
        if self.xr_data.left_controller is not None:
            msg = PoseStamped()
            msg.header.stamp = current_time.to_msg()
            msg.header.frame_id = 'xr_world'
            msg.pose.position.x = self.xr_data.left_controller.position[0]
            msg.pose.position.y = self.xr_data.left_controller.position[1]
            msg.pose.position.z = self.xr_data.left_controller.position[2]
            msg.pose.orientation.x = self.xr_data.left_controller.quaternion[0]
            msg.pose.orientation.y = self.xr_data.left_controller.quaternion[1]
            msg.pose.orientation.z = self.xr_data.left_controller.quaternion[2]
            msg.pose.orientation.w = self.xr_data.left_controller.quaternion[3]
            self.left_controller_pose_pub.publish(msg)

        # Headset
        if self.xr_data.headset is not None:
            msg = PoseStamped()
            msg.header.stamp = current_time.to_msg()
            msg.header.frame_id = 'xr_world'
            msg.pose.position.x = self.xr_data.headset.position[0]
            msg.pose.position.y = self.xr_data.headset.position[1]
            msg.pose.position.z = self.xr_data.headset.position[2]
            msg.pose.orientation.x = self.xr_data.headset.quaternion[0]
            msg.pose.orientation.y = self.xr_data.headset.quaternion[1]
            msg.pose.orientation.z = self.xr_data.headset.quaternion[2]
            msg.pose.orientation.w = self.xr_data.headset.quaternion[3]
            self.headset_pose_pub.publish(msg)

    def _check_tracking_timeout(self):
        """Check if tracking has timed out"""
        current_time_ms = time.time_ns() // 1000000
        elapsed = current_time_ms - self.last_update_time

        if elapsed > self.tracking_timeout_ms:
            if self.xr_data.right_controller and self.xr_data.right_controller.tracking_valid:
                self.get_logger().warn(f'Tracking timeout: {elapsed} ms')
            self._mark_tracking_invalid()

    def _mark_tracking_invalid(self):
        """Mark all tracking as invalid"""
        if self.xr_data.right_controller:
            self.xr_data.right_controller.tracking_valid = False
        if self.xr_data.left_controller:
            self.xr_data.left_controller.tracking_valid = False
        if self.xr_data.headset:
            self.xr_data.headset.tracking_valid = False

    def get_right_controller(self) -> Optional[XRControllerState]:
        """Get right controller state"""
        return self.xr_data.right_controller

    def get_left_controller(self) -> Optional[XRControllerState]:
        """Get left controller state"""
        return self.xr_data.left_controller

    def get_headset(self) -> Optional[XRHeadsetState]:
        """Get headset state"""
        return self.xr_data.headset

    def is_tracking_valid(self, controller: str = 'right') -> bool:
        """Check if controller tracking is valid"""
        if controller == 'right':
            return (self.xr_data.right_controller is not None and
                    self.xr_data.right_controller.tracking_valid)
        elif controller == 'left':
            return (self.xr_data.left_controller is not None and
                    self.xr_data.left_controller.tracking_valid)
        return False

    def destroy_node(self):
        """Cleanup when node is destroyed"""
        if self.mode == 'sdk' and XRT_AVAILABLE:
            try:
                xrt.close()
                self.get_logger().info('xrobotoolkit_sdk closed')
            except:
                pass
        super().destroy_node()


def main(args=None):
    """Main entry point for xr_interface_node"""
    rclpy.init(args=args)

    try:
        node = XRInterface(mode='auto')
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'Error in xr_interface_node: {e}')
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
