#!/usr/bin/env python3

import unittest
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation

from piper_ik_v2 import (
    PIPER_JOINT_NAMES,
    PiperPinocchioIK,
    radians_to_sdk_mdeg,
    sdk_mdeg_to_radians,
)


URDF_PATH = Path(__file__).resolve().parent / "assets" / "piper_description.urdf"


class TestPiperPinocchioIK(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.solver = PiperPinocchioIK(URDF_PATH)
        cls.seed = np.array([0.0, 1.5, -1.5, 0.3, 0.2, 0.1])

    def target_from_delta(self, position_delta, rotation_vector):
        transform = self.solver.forward_transform(self.seed)
        target_position = transform.translation + np.asarray(position_delta)
        target_rotation = (
            Rotation.from_rotvec(rotation_vector).as_matrix() @ transform.rotation
        )
        target_quaternion = Rotation.from_matrix(target_rotation).as_quat()
        return target_position, target_quaternion

    def test_model_is_reduced_to_six_arm_joints(self):
        self.assertEqual(self.solver.model.nq, 6)
        self.assertEqual(self.solver.model.nv, 6)
        self.assertEqual(
            tuple(str(name) for name in self.solver.model.names[1:]),
            PIPER_JOINT_NAMES,
        )
        np.testing.assert_allclose(
            self.solver.lower_limits,
            np.array([-2.618, 0.0, -2.967, -1.745, -1.22, -2.0944]),
            atol=1.0e-9,
        )
        np.testing.assert_allclose(
            self.solver.upper_limits,
            np.array([2.618, 3.14, 0.0, 1.745, 1.22, 2.0944]),
            atol=1.0e-9,
        )

    def test_fk_pose_is_an_exact_ik_target(self):
        position, quaternion = self.solver.forward_pose(self.seed)
        result = self.solver.solve(position, quaternion, self.seed)
        self.assertTrue(result.success, result)
        self.assertLess(result.position_error_m, 1.0e-9)
        self.assertLess(result.orientation_error_rad, 1.0e-9)
        np.testing.assert_allclose(result.joint_angles_rad, self.seed, atol=1.0e-9)

    def test_position_priority_avoids_wrist_flip(self):
        position, quaternion = self.target_from_delta(
            [0.0, 0.01, 0.0], [0.0, 0.0, 0.0]
        )
        result = self.solver.solve(position, quaternion, self.seed)
        self.assertTrue(result.success, result)
        self.assertLess(result.position_error_m, 5.0e-4)
        self.assertLess(np.max(np.abs(result.joint_angles_rad - self.seed)), 0.15)

    def test_mixed_pose_tracks_rotation_softly(self):
        rotation_vector = np.deg2rad([3.0, -4.0, 5.0])
        position, quaternion = self.target_from_delta(
            [0.01, 0.01, 0.01], rotation_vector
        )
        initial_orientation_error = np.linalg.norm(rotation_vector)
        result = self.solver.solve(position, quaternion, self.seed)
        self.assertTrue(result.success, result)
        self.assertLess(result.position_error_m, 5.0e-4)
        self.assertLess(result.orientation_error_rad, initial_orientation_error)
        self.assertLess(np.max(np.abs(result.joint_angles_rad - self.seed)), 0.25)

    def test_position_only_mode(self):
        solver = PiperPinocchioIK(URDF_PATH, orientation_weight=0.0)
        position, quaternion = self.target_from_delta(
            [0.015, -0.01, 0.005], [0.2, -0.1, 0.3]
        )
        result = solver.solve(position, quaternion, self.seed)
        self.assertTrue(result.success, result)
        self.assertLess(result.position_error_m, 5.0e-4)

    def test_unreachable_target_is_rejected(self):
        _, quaternion = self.solver.forward_pose(self.seed)
        result = self.solver.solve([2.0, 2.0, 2.0], quaternion, self.seed)
        self.assertFalse(result.success)
        self.assertGreater(result.position_error_m, 0.1)

    def test_quaternion_sign_is_equivalent(self):
        position, quaternion = self.solver.forward_pose(self.seed)
        positive = self.solver.solve(position, quaternion, self.seed)
        negative = self.solver.solve(position, -quaternion, self.seed)
        self.assertTrue(positive.success)
        self.assertTrue(negative.success)
        np.testing.assert_allclose(
            positive.joint_angles_rad,
            negative.joint_angles_rad,
            atol=1.0e-8,
        )

    def test_sdk_unit_round_trip(self):
        joints = np.array([0.1, 1.2, -1.4, 0.2, -0.3, 0.4])
        encoded = radians_to_sdk_mdeg(joints)
        decoded = sdk_mdeg_to_radians(encoded)
        np.testing.assert_allclose(decoded, joints, atol=np.deg2rad(0.00051))

    def test_invalid_quaternion_is_rejected(self):
        position, _ = self.solver.forward_pose(self.seed)
        with self.assertRaises(ValueError):
            self.solver.solve(position, [0.0, 0.0, 0.0, 0.0], self.seed)


if __name__ == "__main__":
    unittest.main(verbosity=2)
