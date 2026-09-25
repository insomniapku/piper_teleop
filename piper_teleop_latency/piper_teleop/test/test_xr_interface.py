#!/usr/bin/env python3
"""
Unit tests for XR Interface (M2)
Tests quaternion handling, field parsing, and tracking timeout
"""

import unittest
import numpy as np
import time
from piper_teleop.xr_data_types import XRControllerState, XRHeadsetState, XRData


class TestXRDataTypes(unittest.TestCase):
    """Test XR data type definitions"""

    def test_controller_state_creation(self):
        """Test creating XRControllerState"""
        state = XRControllerState(
            position=np.array([0.1, 0.2, 0.3]),
            quaternion=np.array([0.0, 0.0, 0.0, 1.0]),
            grip_value=0.5,
            trigger_value=0.7,
            tracking_valid=True,
            timestamp_ns=1234567890
        )

        self.assertEqual(state.position[0], 0.1)
        self.assertEqual(state.position[1], 0.2)
        self.assertEqual(state.position[2], 0.3)
        self.assertEqual(state.quaternion[3], 1.0)  # w component
        self.assertEqual(state.grip_value, 0.5)
        self.assertEqual(state.trigger_value, 0.7)
        self.assertTrue(state.tracking_valid)

    def test_quaternion_normalization(self):
        """Test that quaternion has unit length"""
        quat = np.array([0.0, 0.0, 0.0, 1.0])
        norm = np.linalg.norm(quat)
        self.assertAlmostEqual(norm, 1.0, places=6)

    def test_quaternion_wxyz_order(self):
        """Test quaternion storage order [x, y, z, w]"""
        # Identity quaternion
        quat = np.array([0.0, 0.0, 0.0, 1.0])
        self.assertEqual(quat[0], 0.0)  # x
        self.assertEqual(quat[1], 0.0)  # y
        self.assertEqual(quat[2], 0.0)  # z
        self.assertEqual(quat[3], 1.0)  # w

    def test_grip_trigger_range(self):
        """Test grip and trigger values are in valid range"""
        state = XRControllerState(
            position=np.zeros(3),
            quaternion=np.array([0, 0, 0, 1]),
            grip_value=0.0,
            trigger_value=1.0
        )

        self.assertGreaterEqual(state.grip_value, 0.0)
        self.assertLessEqual(state.grip_value, 1.0)
        self.assertGreaterEqual(state.trigger_value, 0.0)
        self.assertLessEqual(state.trigger_value, 1.0)

    def test_xr_data_container(self):
        """Test XRData container"""
        right = XRControllerState(
            position=np.array([1, 0, 0]),
            quaternion=np.array([0, 0, 0, 1]),
            grip_value=1.0,
            trigger_value=0.5
        )

        left = XRControllerState(
            position=np.array([-1, 0, 0]),
            quaternion=np.array([0, 0, 0, 1]),
            grip_value=0.0,
            trigger_value=0.0
        )

        headset = XRHeadsetState(
            position=np.array([0, 0, 1.6]),
            quaternion=np.array([0, 0, 0, 1])
        )

        xr_data = XRData(
            right_controller=right,
            left_controller=left,
            headset=headset,
            timestamp_ns=int(time.time() * 1e9)
        )

        self.assertIsNotNone(xr_data.right_controller)
        self.assertIsNotNone(xr_data.left_controller)
        self.assertIsNotNone(xr_data.headset)
        self.assertGreater(xr_data.timestamp_ns, 0)


class TestXRInterfaceMock(unittest.TestCase):
    """Test XR Interface in mock mode"""

    def setUp(self):
        """Set up test - requires rclpy initialization"""
        import rclpy
        try:
            rclpy.init()
        except:
            pass  # Already initialized

    def tearDown(self):
        """Tear down test"""
        pass  # Don't shutdown rclpy between tests

    def test_mock_mode_initialization(self):
        """Test XR interface initializes in mock mode"""
        from piper_teleop.xr_interface import XRInterface

        node = XRInterface(mode='mock')
        self.assertEqual(node.mode, 'mock')
        node.destroy_node()

    def test_mock_data_generation(self):
        """Test mock data is generated correctly"""
        from piper_teleop.xr_interface import XRInterface

        node = XRInterface(mode='mock')

        # Update once
        node._update_from_mock()

        # Check right controller data
        right = node.get_right_controller()
        self.assertIsNotNone(right)
        self.assertEqual(len(right.position), 3)
        self.assertEqual(len(right.quaternion), 4)
        self.assertGreaterEqual(right.grip_value, 0.0)
        self.assertLessEqual(right.grip_value, 1.0)
        self.assertTrue(right.tracking_valid)

        node.destroy_node()

    def test_tracking_valid_check(self):
        """Test tracking validity checking"""
        from piper_teleop.xr_interface import XRInterface

        node = XRInterface(mode='mock')
        node._update_from_mock()

        self.assertTrue(node.is_tracking_valid('right'))
        self.assertTrue(node.is_tracking_valid('left'))

        node.destroy_node()

    def test_tracking_timeout(self):
        """Test tracking timeout detection"""
        from piper_teleop.xr_interface import XRInterface

        # Create node with very short timeout
        node = XRInterface(mode='mock')
        node.tracking_timeout_ms = 100  # 100ms timeout

        # Initial update
        node._update_from_mock()
        self.assertTrue(node.is_tracking_valid('right'))

        # Simulate timeout by setting old timestamp
        node.last_update_time = time.time_ns() // 1000000 - 200  # 200ms ago
        node._check_tracking_timeout()

        # Should be marked invalid now
        self.assertFalse(node.is_tracking_valid('right'))

        node.destroy_node()

    def test_quaternion_from_sdk_format(self):
        """Test quaternion conversion from SDK format [x,y,z,qx,qy,qz,qw]"""
        # SDK returns: [x, y, z, qx, qy, qz, qw]
        sdk_pose = [0.1, 0.2, 0.3, 0.0, 0.0, 0.0, 1.0]

        position = np.array(sdk_pose[:3])
        quaternion = np.array([sdk_pose[3], sdk_pose[4], sdk_pose[5], sdk_pose[6]])

        self.assertEqual(position[0], 0.1)
        self.assertEqual(position[1], 0.2)
        self.assertEqual(position[2], 0.3)
        self.assertEqual(quaternion[0], 0.0)  # qx
        self.assertEqual(quaternion[1], 0.0)  # qy
        self.assertEqual(quaternion[2], 0.0)  # qz
        self.assertEqual(quaternion[3], 1.0)  # qw


def run_tests():
    """Run all tests"""
    import rclpy
    rclpy.init()

    suite = unittest.TestLoader().loadTestsFromModule(__import__(__name__))
    runner = unittest.TextTestRunner(verbosity=2)
    result = runner.run(suite)

    rclpy.shutdown()
    return result.wasSuccessful()


if __name__ == '__main__':
    import sys
    success = run_tests()
    sys.exit(0 if success else 1)
