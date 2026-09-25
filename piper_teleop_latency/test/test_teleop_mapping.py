#!/usr/bin/env python3
"""
Unit tests for Teleoperation Mapping (M3)
Tests delta mapping, coordinate transforms, workspace limits, and tracking loss
"""

import unittest
import numpy as np
from scipy.spatial.transform import Rotation as R

from piper_teleop.teleop_mapping import (
    TeleopMapper,
    normalize_quaternion,
    quat_diff_as_rotvec,
    apply_workspace_limit
)


class TestQuaternionUtils(unittest.TestCase):
    """Test quaternion utility functions"""

    def test_normalize_quaternion(self):
        """Test quaternion normalization"""
        # Non-normalized quaternion
        quat = np.array([0.5, 0.5, 0.5, 0.5])
        normalized = normalize_quaternion(quat)

        # Should be unit length
        self.assertAlmostEqual(np.linalg.norm(normalized), 1.0, places=6)

    def test_normalize_zero_quaternion(self):
        """Test normalization of near-zero quaternion"""
        quat = np.array([0.0, 0.0, 0.0, 0.0])
        normalized = normalize_quaternion(quat)

        # Should return identity quaternion
        np.testing.assert_array_almost_equal(normalized, [0, 0, 0, 1])

    def test_quat_diff_identity(self):
        """Test quaternion difference for identical quaternions"""
        q1 = np.array([0.0, 0.0, 0.0, 1.0])
        q2 = np.array([0.0, 0.0, 0.0, 1.0])

        rotvec = quat_diff_as_rotvec(q1, q2)

        # Should be zero rotation
        self.assertAlmostEqual(np.linalg.norm(rotvec), 0.0, places=6)

    def test_quat_diff_90deg_z(self):
        """Test quaternion difference for 90° rotation around Z"""
        q1 = np.array([0.0, 0.0, 0.0, 1.0])  # Identity
        q2 = R.from_euler('z', np.pi/2).as_quat()  # 90° around Z

        rotvec = quat_diff_as_rotvec(q1, q2)

        # Should be ~90° (pi/2 radians) around Z axis
        self.assertAlmostEqual(np.linalg.norm(rotvec), np.pi/2, places=5)
        # Z component should dominate
        self.assertGreater(abs(rotvec[2]), abs(rotvec[0]))
        self.assertGreater(abs(rotvec[2]), abs(rotvec[1]))


class TestWorkspaceLimit(unittest.TestCase):
    """Test workspace limiting"""

    def test_no_clamp_inside_workspace(self):
        """Test position inside workspace is not clamped"""
        position = np.array([0.3, 0.0, 0.4])
        workspace = {
            'x_min': 0.1, 'x_max': 0.6,
            'y_min': -0.4, 'y_max': 0.4,
            'z_min': 0.0, 'z_max': 0.8
        }

        clamped, was_clamped = apply_workspace_limit(position, workspace)

        self.assertFalse(was_clamped)
        np.testing.assert_array_equal(clamped, position)

    def test_clamp_x_min(self):
        """Test clamping at X minimum"""
        position = np.array([0.05, 0.0, 0.4])
        workspace = {
            'x_min': 0.1, 'x_max': 0.6,
            'y_min': -0.4, 'y_max': 0.4,
            'z_min': 0.0, 'z_max': 0.8
        }

        clamped, was_clamped = apply_workspace_limit(position, workspace)

        self.assertTrue(was_clamped)
        self.assertEqual(clamped[0], 0.1)

    def test_clamp_multiple_axes(self):
        """Test clamping on multiple axes"""
        position = np.array([0.7, -0.5, -0.1])
        workspace = {
            'x_min': 0.1, 'x_max': 0.6,
            'y_min': -0.4, 'y_max': 0.4,
            'z_min': 0.0, 'z_max': 0.8
        }

        clamped, was_clamped = apply_workspace_limit(position, workspace)

        self.assertTrue(was_clamped)
        self.assertEqual(clamped[0], 0.6)  # X clamped to max
        self.assertEqual(clamped[1], -0.4)  # Y clamped to min
        self.assertEqual(clamped[2], 0.0)   # Z clamped to min


class TestTeleopMapper(unittest.TestCase):
    """Test TeleopMapper class"""

    def setUp(self):
        """Set up test configuration"""
        self.config = {
            'position_mapping': {
                'scale': 1.0,
                'max_delta': 0.1
            },
            'rotation_mapping': {
                'scale': 1.0,
                'max_delta': 0.2,
                'max_angle': np.pi
            },
            'xr_to_piper_transform': {
                'translation': [0.0, 0.0, 0.0],
                'rotation': [0.0, 0.0, 0.0]  # Identity transform for testing
            },
            'safety': {
                'workspace_limit': {
                    'x_min': 0.1, 'x_max': 0.6,
                    'y_min': -0.4, 'y_max': 0.4,
                    'z_min': 0.0, 'z_max': 0.8
                },
                'enable_workspace_limit': True
            }
        }

    def test_initialization(self):
        """Test mapper initialization"""
        mapper = TeleopMapper(self.config)

        self.assertFalse(mapper.is_active())
        self.assertEqual(mapper.position_scale, 1.0)
        self.assertEqual(mapper.rotation_scale, 1.0)

    def test_set_reference(self):
        """Test setting reference poses"""
        mapper = TeleopMapper(self.config)

        xr_pos = np.array([0.5, 0.0, 0.5])
        xr_quat = np.array([0.0, 0.0, 0.0, 1.0])
        ee_pos = np.array([0.3, 0.0, 0.4])
        ee_quat = np.array([0.0, 0.0, 0.0, 1.0])

        mapper.set_reference(xr_pos, xr_quat, ee_pos, ee_quat)

        self.assertTrue(mapper.is_active())
        np.testing.assert_array_equal(mapper.xr_reference_pose['position'], xr_pos)
        np.testing.assert_array_equal(mapper.ee_reference_pose['position'], ee_pos)

    def test_reset_reference(self):
        """Test resetting reference poses"""
        mapper = TeleopMapper(self.config)

        xr_pos = np.array([0.5, 0.0, 0.5])
        xr_quat = np.array([0.0, 0.0, 0.0, 1.0])
        ee_pos = np.array([0.3, 0.0, 0.4])
        ee_quat = np.array([0.0, 0.0, 0.0, 1.0])

        mapper.set_reference(xr_pos, xr_quat, ee_pos, ee_quat)
        self.assertTrue(mapper.is_active())

        mapper.reset_reference()
        self.assertFalse(mapper.is_active())

    def test_controller_not_moving_target_not_moving(self):
        """Test: XR controller not moving → target EE not moving"""
        mapper = TeleopMapper(self.config)

        # Set reference
        xr_pos = np.array([0.5, 0.0, 0.5])
        xr_quat = np.array([0.0, 0.0, 0.0, 1.0])
        ee_pos = np.array([0.3, 0.0, 0.4])
        ee_quat = np.array([0.0, 0.0, 0.0, 1.0])

        mapper.set_reference(xr_pos, xr_quat, ee_pos, ee_quat)

        # Compute target with same pose (no movement)
        target_pose = mapper.compute_target_pose(xr_pos, xr_quat)

        # Target should match reference
        self.assertAlmostEqual(target_pose.position.x, ee_pos[0], places=6)
        self.assertAlmostEqual(target_pose.position.y, ee_pos[1], places=6)
        self.assertAlmostEqual(target_pose.position.z, ee_pos[2], places=6)

        # Orientation should also match
        self.assertAlmostEqual(target_pose.orientation.w, 1.0, places=6)

    def test_move_positive_x(self):
        """Test: Move +X in XR → Move +X in robot (identity transform)"""
        mapper = TeleopMapper(self.config)

        # Set reference
        xr_pos_ref = np.array([0.5, 0.0, 0.5])
        xr_quat = np.array([0.0, 0.0, 0.0, 1.0])
        ee_pos_ref = np.array([0.3, 0.0, 0.4])
        ee_quat = np.array([0.0, 0.0, 0.0, 1.0])

        mapper.set_reference(xr_pos_ref, xr_quat, ee_pos_ref, ee_quat)

        # Move +0.05m in X
        xr_pos_new = xr_pos_ref + np.array([0.05, 0.0, 0.0])
        target_pose = mapper.compute_target_pose(xr_pos_new, xr_quat)

        # Target should move +0.05m in X (with scale=1.0)
        self.assertAlmostEqual(target_pose.position.x, ee_pos_ref[0] + 0.05, places=6)
        self.assertAlmostEqual(target_pose.position.y, ee_pos_ref[1], places=6)
        self.assertAlmostEqual(target_pose.position.z, ee_pos_ref[2], places=6)

    def test_move_negative_x(self):
        """Test: Move -X in XR → Move -X in robot"""
        mapper = TeleopMapper(self.config)

        xr_pos_ref = np.array([0.5, 0.0, 0.5])
        xr_quat = np.array([0.0, 0.0, 0.0, 1.0])
        ee_pos_ref = np.array([0.3, 0.0, 0.4])
        ee_quat = np.array([0.0, 0.0, 0.0, 1.0])

        mapper.set_reference(xr_pos_ref, xr_quat, ee_pos_ref, ee_quat)

        # Move -0.03m in X
        xr_pos_new = xr_pos_ref + np.array([-0.03, 0.0, 0.0])
        target_pose = mapper.compute_target_pose(xr_pos_new, xr_quat)

        self.assertAlmostEqual(target_pose.position.x, ee_pos_ref[0] - 0.03, places=6)

    def test_move_positive_y(self):
        """Test: Move +Y in XR → Move +Y in robot"""
        mapper = TeleopMapper(self.config)

        xr_pos_ref = np.array([0.5, 0.0, 0.5])
        xr_quat = np.array([0.0, 0.0, 0.0, 1.0])
        ee_pos_ref = np.array([0.3, 0.0, 0.4])
        ee_quat = np.array([0.0, 0.0, 0.0, 1.0])

        mapper.set_reference(xr_pos_ref, xr_quat, ee_pos_ref, ee_quat)

        xr_pos_new = xr_pos_ref + np.array([0.0, 0.04, 0.0])
        target_pose = mapper.compute_target_pose(xr_pos_new, xr_quat)

        self.assertAlmostEqual(target_pose.position.y, ee_pos_ref[1] + 0.04, places=6)

    def test_move_negative_y(self):
        """Test: Move -Y in XR → Move -Y in robot"""
        mapper = TeleopMapper(self.config)

        xr_pos_ref = np.array([0.5, 0.0, 0.5])
        xr_quat = np.array([0.0, 0.0, 0.0, 1.0])
        ee_pos_ref = np.array([0.3, 0.0, 0.4])
        ee_quat = np.array([0.0, 0.0, 0.0, 1.0])

        mapper.set_reference(xr_pos_ref, xr_quat, ee_pos_ref, ee_quat)

        xr_pos_new = xr_pos_ref + np.array([0.0, -0.02, 0.0])
        target_pose = mapper.compute_target_pose(xr_pos_new, xr_quat)

        self.assertAlmostEqual(target_pose.position.y, ee_pos_ref[1] - 0.02, places=6)

    def test_move_positive_z(self):
        """Test: Move +Z in XR → Move +Z in robot"""
        mapper = TeleopMapper(self.config)

        xr_pos_ref = np.array([0.5, 0.0, 0.5])
        xr_quat = np.array([0.0, 0.0, 0.0, 1.0])
        ee_pos_ref = np.array([0.3, 0.0, 0.4])
        ee_quat = np.array([0.0, 0.0, 0.0, 1.0])

        mapper.set_reference(xr_pos_ref, xr_quat, ee_pos_ref, ee_quat)

        xr_pos_new = xr_pos_ref + np.array([0.0, 0.0, 0.06])
        target_pose = mapper.compute_target_pose(xr_pos_new, xr_quat)

        self.assertAlmostEqual(target_pose.position.z, ee_pos_ref[2] + 0.06, places=6)

    def test_move_negative_z(self):
        """Test: Move -Z in XR → Move -Z in robot"""
        mapper = TeleopMapper(self.config)

        xr_pos_ref = np.array([0.5, 0.0, 0.5])
        xr_quat = np.array([0.0, 0.0, 0.0, 1.0])
        ee_pos_ref = np.array([0.3, 0.0, 0.4])
        ee_quat = np.array([0.0, 0.0, 0.0, 1.0])

        mapper.set_reference(xr_pos_ref, xr_quat, ee_pos_ref, ee_quat)

        xr_pos_new = xr_pos_ref + np.array([0.0, 0.0, -0.03])
        target_pose = mapper.compute_target_pose(xr_pos_new, xr_quat)

        self.assertAlmostEqual(target_pose.position.z, ee_pos_ref[2] - 0.03, places=6)

    def test_rotation_z_axis(self):
        """Test: Rotation around Z axis"""
        mapper = TeleopMapper(self.config)

        xr_pos = np.array([0.5, 0.0, 0.5])
        xr_quat_ref = np.array([0.0, 0.0, 0.0, 1.0])
        ee_pos = np.array([0.3, 0.0, 0.4])
        ee_quat_ref = np.array([0.0, 0.0, 0.0, 1.0])

        mapper.set_reference(xr_pos, xr_quat_ref, ee_pos, ee_quat_ref)

        # Rotate 45° around Z
        xr_quat_new = R.from_euler('z', np.pi/4).as_quat()
        target_pose = mapper.compute_target_pose(xr_pos, xr_quat_new)

        # Verify quaternion is normalized
        quat = np.array([
            target_pose.orientation.x,
            target_pose.orientation.y,
            target_pose.orientation.z,
            target_pose.orientation.w
        ])
        self.assertAlmostEqual(np.linalg.norm(quat), 1.0, places=6)

        # Position should not change
        self.assertAlmostEqual(target_pose.position.x, ee_pos[0], places=6)

    def test_position_scale(self):
        """Test position scaling"""
        config = self.config.copy()
        config['position_mapping']['scale'] = 0.5  # Half scale

        mapper = TeleopMapper(config)

        xr_pos_ref = np.array([0.5, 0.0, 0.5])
        xr_quat = np.array([0.0, 0.0, 0.0, 1.0])
        ee_pos_ref = np.array([0.3, 0.0, 0.4])
        ee_quat = np.array([0.0, 0.0, 0.0, 1.0])

        mapper.set_reference(xr_pos_ref, xr_quat, ee_pos_ref, ee_quat)

        # Move 0.1m in XR
        xr_pos_new = xr_pos_ref + np.array([0.1, 0.0, 0.0])
        target_pose = mapper.compute_target_pose(xr_pos_new, xr_quat)

        # Should move 0.05m in robot (half scale)
        self.assertAlmostEqual(target_pose.position.x, ee_pos_ref[0] + 0.05, places=6)

    def test_workspace_clamp(self):
        """Test workspace clamping"""
        mapper = TeleopMapper(self.config)

        xr_pos_ref = np.array([0.5, 0.0, 0.5])
        xr_quat = np.array([0.0, 0.0, 0.0, 1.0])
        ee_pos_ref = np.array([0.3, 0.0, 0.4])
        ee_quat = np.array([0.0, 0.0, 0.0, 1.0])

        mapper.set_reference(xr_pos_ref, xr_quat, ee_pos_ref, ee_quat)

        # Try to move outside workspace (move +1m in X)
        xr_pos_new = xr_pos_ref + np.array([1.0, 0.0, 0.0])
        target_pose = mapper.compute_target_pose(xr_pos_new, xr_quat)

        # Should be clamped to x_max = 0.6
        self.assertLessEqual(target_pose.position.x, 0.6)

    def test_max_delta_clamp(self):
        """Test per-step delta clamping"""
        mapper = TeleopMapper(self.config)

        xr_pos_ref = np.array([0.5, 0.0, 0.5])
        xr_quat = np.array([0.0, 0.0, 0.0, 1.0])
        ee_pos_ref = np.array([0.3, 0.0, 0.4])
        ee_quat = np.array([0.0, 0.0, 0.0, 1.0])

        mapper.set_reference(xr_pos_ref, xr_quat, ee_pos_ref, ee_quat)

        # Try to move 0.5m (exceeds max_delta=0.1)
        xr_pos_new = xr_pos_ref + np.array([0.5, 0.0, 0.0])
        target_pose = mapper.compute_target_pose(xr_pos_new, xr_quat)

        # Delta should be clamped to max_delta
        delta = target_pose.position.x - ee_pos_ref[0]
        self.assertLessEqual(abs(delta), 0.1 + 1e-6)

    def test_reference_reset_no_jump(self):
        """Test: Re-gripping doesn't cause jump"""
        mapper = TeleopMapper(self.config)

        # Initial grip at position A
        xr_pos_1 = np.array([0.5, 0.0, 0.5])
        xr_quat = np.array([0.0, 0.0, 0.0, 1.0])
        ee_pos_1 = np.array([0.3, 0.0, 0.4])
        ee_quat = np.array([0.0, 0.0, 0.0, 1.0])

        mapper.set_reference(xr_pos_1, xr_quat, ee_pos_1, ee_quat)

        # Move to position B
        xr_pos_2 = np.array([0.55, 0.0, 0.5])
        target_1 = mapper.compute_target_pose(xr_pos_2, xr_quat)

        # Release and re-grip at position B
        mapper.reset_reference()
        ee_pos_2 = np.array([target_1.position.x, target_1.position.y, target_1.position.z])
        mapper.set_reference(xr_pos_2, xr_quat, ee_pos_2, ee_quat)

        # Immediate target should match reference (no jump)
        target_2 = mapper.compute_target_pose(xr_pos_2, xr_quat)

        self.assertAlmostEqual(target_2.position.x, ee_pos_2[0], places=6)
        self.assertAlmostEqual(target_2.position.y, ee_pos_2[1], places=6)
        self.assertAlmostEqual(target_2.position.z, ee_pos_2[2], places=6)

    def test_tracking_loss_invalidates_mapping(self):
        """Test: Tracking loss invalidates mapping"""
        mapper = TeleopMapper(self.config)

        xr_pos = np.array([0.5, 0.0, 0.5])
        xr_quat = np.array([0.0, 0.0, 0.0, 1.0])
        ee_pos = np.array([0.3, 0.0, 0.4])
        ee_quat = np.array([0.0, 0.0, 0.0, 1.0])

        mapper.set_reference(xr_pos, xr_quat, ee_pos, ee_quat)
        self.assertTrue(mapper.is_active())

        # Simulate tracking loss
        mapper.set_tracking_valid(False)

        # Mapping should be deactivated
        self.assertFalse(mapper.is_active())

        # compute_target_pose should return None
        target = mapper.compute_target_pose(xr_pos, xr_quat)
        self.assertIsNone(target)

    def test_quaternion_normalization(self):
        """Test quaternion normalization in output"""
        mapper = TeleopMapper(self.config)

        xr_pos = np.array([0.5, 0.0, 0.5])
        xr_quat = np.array([0.1, 0.1, 0.1, 0.9])  # Not normalized
        ee_pos = np.array([0.3, 0.0, 0.4])
        ee_quat = np.array([0.0, 0.0, 0.0, 1.0])

        mapper.set_reference(xr_pos, xr_quat, ee_pos, ee_quat)

        target = mapper.compute_target_pose(xr_pos, xr_quat)

        # Output quaternion should be normalized
        quat = np.array([
            target.orientation.x,
            target.orientation.y,
            target.orientation.z,
            target.orientation.w
        ])
        self.assertAlmostEqual(np.linalg.norm(quat), 1.0, places=6)


def run_tests():
    """Run all tests"""
    suite = unittest.TestLoader().loadTestsFromModule(__import__(__name__))
    runner = unittest.TextTestRunner(verbosity=2)
    result = runner.run(suite)
    return result.wasSuccessful()


if __name__ == '__main__':
    import sys
    success = run_tests()
    sys.exit(0 if success else 1)
