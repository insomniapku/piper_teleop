#!/usr/bin/env python3
"""
Integration tests for M5: Complete Pico → Mapping → IK pipeline
Tests the full teleoperation chain with safety features
"""

import unittest
import numpy as np
import os

from pathlib import Path
from piper_teleop.teleop_controller import PiperTeleopController, ControllerState


class TestPiperTeleopController(unittest.TestCase):
    """Test complete teleoperation controller"""

    @classmethod
    def setUpClass(cls):
        configured_path = os.environ.get("PIPER_URDF_PATH")
        candidates = [
            Path(configured_path) if configured_path else None,
            Path(__file__).resolve().parents[3] / "piper_description.urdf",
            Path(__file__).resolve().parents[2] / "piper_description.urdf",
        ]
        urdf_path = next((str(path) for path in candidates if path and path.is_file()), None)

        if urdf_path is None:
            raise unittest.SkipTest("Set PIPER_URDF_PATH to run tests that require the Piper URDF")

        cls.controller = PiperTeleopController(
        cls.urdf_path = urdf_path
            urdf_path,
            control_frequency=100.0,
            enable_velocity_limiting=True,
            enable_fallback_on_ik_failure=True
        )

    def setUp(self):
        """Reset controller before each test"""
        self.controller.reset()

    def test_initialization(self):
        """Test controller initialization"""
        self.assertEqual(self.controller.state, ControllerState.IDLE)
        self.assertIsNotNone(self.controller.current_joint_angles)
        self.assertEqual(len(self.controller.current_joint_angles), 6)

    def test_single_update(self):
        """Test single control update"""
        # VR input (in front of user)
        vr_pos = np.array([0.0, 0.0, -0.3])
        vr_quat = np.array([0.0, 0.0, 0.0, 1.0])

        result = self.controller.update(vr_pos, vr_quat, dt=0.01)

        self.assertTrue(result['success'])
        self.assertIn('joint_angles', result)
        self.assertEqual(len(result['joint_angles']), 6)
        self.assertIn('target_pose', result)
        self.assertIn('stats', result)

        print(f"\nSingle update test:")
        print(f"  VR position: {vr_pos}")
        print(f"  Target robot position: {result['target_pose']['position']}")
        print(f"  Joint solution: {result['joint_angles']}")
        print(f"  State: {result['state']}")

    def test_continuous_updates(self):
        """Test continuous control updates (simulating real-time loop)"""
        # Start position
        vr_pos = np.array([0.0, 0.0, -0.3])
        vr_quat = np.array([0.0, 0.0, 0.0, 1.0])

        n_steps = 10
        results = []

        for i in range(n_steps):
            # Move VR controller forward slowly
            vr_pos_current = vr_pos + np.array([0.01 * i, 0.0, 0.0])

            result = self.controller.update(vr_pos_current, vr_quat, dt=0.01)
            results.append(result)

        # All should succeed
        successes = sum(1 for r in results if r['success'])
        self.assertEqual(successes, n_steps)

        print(f"\nContinuous update test ({n_steps} steps):")
        print(f"  Successful: {successes}/{n_steps}")
        print(f"  Final state: {results[-1]['state']}")

    def test_ik_uses_previous_as_seed(self):
        """Test that IK uses previous joint angles as seed for continuity"""
        vr_pos = np.array([0.0, 0.0, -0.3])
        vr_quat = np.array([0.0, 0.0, 0.0, 1.0])

        # First update
        result1 = self.controller.update(vr_pos, vr_quat, dt=0.01)
        joints1 = result1['joint_angles']

        # Small movement
        vr_pos2 = vr_pos + np.array([0.01, 0.0, 0.0])
        result2 = self.controller.update(vr_pos2, vr_quat, dt=0.01)
        joints2 = result2['joint_angles']

        # Joints should be similar (continuous)
        joint_diff = np.abs(joints2 - joints1)
        max_diff = np.max(joint_diff)

        self.assertLess(max_diff, 0.5, "Joint changes too large for small VR movement")

        print(f"\nSeed continuity test:")
        print(f"  VR movement: 0.01 m in X")
        print(f"  Max joint change: {max_diff:.4f} rad ({np.rad2deg(max_diff):.2f}°)")
        print(f"  ✓ Continuous motion")

    def test_joint_limits_enforcement(self):
        """Test that joint limits are enforced"""
        # Set to configuration near limit
        near_limit = np.array([2.5, 1.5, -0.1, 0.0, 0.0, 0.0])  # joint1 near +2.618 limit
        self.controller.set_joint_state(near_limit)

        # Try to move further (should trigger limit or IK failure)
        vr_pos = np.array([0.0, 0.5, -0.3])  # Far to the side
        vr_quat = np.array([0.0, 0.0, 0.0, 1.0])

        result = self.controller.update(vr_pos, vr_quat, dt=0.01)

        # Should either fail or use fallback
        if not result['success']:
            self.assertIn('limit', result.get('error_message', '').lower())
        else:
            # Fallback used
            self.assertTrue(result.get('is_fallback', False))

        print(f"\nJoint limit test:")
        print(f"  State: {result['state']}")
        print(f"  Fallback used: {result.get('is_fallback', False)}")

    def test_velocity_limits_enforcement(self):
        """Test that velocity limits are enforced"""
        vr_pos1 = np.array([0.0, 0.0, -0.3])
        vr_quat = np.array([0.0, 0.0, 0.0, 1.0])

        # First position
        result1 = self.controller.update(vr_pos1, vr_quat, dt=0.01)
        joints1 = result1['joint_angles']

        # Large sudden movement (should trigger velocity limiting)
        vr_pos2 = np.array([0.2, 0.0, -0.3])  # 20cm sudden jump
        result2 = self.controller.update(vr_pos2, vr_quat, dt=0.01)

        if result2['success']:
            joints2 = result2['joint_angles']

            # Compute actual velocities
            velocities = (joints2 - joints1) / 0.01
            max_velocity = np.max(np.abs(velocities))

            # Should be at or below limit (5.0 rad/s)
            self.assertLessEqual(max_velocity, 5.1, "Velocity limit not enforced")

            print(f"\nVelocity limit test:")
            print(f"  VR jump: 0.2 m")
            print(f"  Max joint velocity: {max_velocity:.2f} rad/s")
            print(f"  Velocity limit: 5.0 rad/s")
            if result2['state'] == ControllerState.VELOCITY_LIMITED:
                print(f"  ✓ Velocity limiting applied")
        else:
            print(f"\nVelocity limit test: IK failed for large movement")

    def test_ik_failure_fallback(self):
        """Test IK failure fallback strategy"""
        # First, establish valid position
        vr_pos1 = np.array([0.0, 0.0, -0.3])
        vr_quat = np.array([0.0, 0.0, 0.0, 1.0])
        result1 = self.controller.update(vr_pos1, vr_quat, dt=0.01)
        self.assertTrue(result1['success'])
        last_valid = result1['joint_angles'].copy()

        # Try unreachable position
        vr_pos2 = np.array([2.0, 0.0, -0.3])  # Very far
        result2 = self.controller.update(vr_pos2, vr_quat, dt=0.01)

        # Should succeed with fallback
        self.assertTrue(result2['success'])
        self.assertTrue(result2.get('is_fallback', False))

        # Should return last valid position
        np.testing.assert_array_almost_equal(
            result2['joint_angles'],
            last_valid,
            decimal=6
        )

        print(f"\nIK failure fallback test:")
        print(f"  Unreachable position: {vr_pos2}")
        print(f"  State: {result2['state']}")
        print(f"  Fallback activated: {result2.get('is_fallback', False)}")
        print(f"  ✓ Holding last valid position")

    def test_statistics_tracking(self):
        """Test that statistics are tracked correctly"""
        vr_pos = np.array([0.0, 0.0, -0.3])
        vr_quat = np.array([0.0, 0.0, 0.0, 1.0])

        # Multiple updates
        for i in range(5):
            self.controller.update(vr_pos, vr_quat, dt=0.01)
            vr_pos[0] += 0.01

        stats = self.controller.get_stats()

        self.assertEqual(stats['total_updates'], 5)
        self.assertGreaterEqual(stats['successful_ik'], 0)
        self.assertIn('ik_success_rate', stats)

        print(f"\nStatistics tracking:")
        print(f"  Total updates: {stats['total_updates']}")
        print(f"  Successful IK: {stats['successful_ik']}")
        print(f"  Failed IK: {stats['failed_ik']}")
        print(f"  IK success rate: {stats['ik_success_rate']:.2%}")

    def test_get_current_ee_pose(self):
        """Test getting current EE pose"""
        vr_pos = np.array([0.0, 0.0, -0.3])
        vr_quat = np.array([0.0, 0.0, 0.0, 1.0])

        result = self.controller.update(vr_pos, vr_quat, dt=0.01)
        self.assertTrue(result['success'])

        # Get current EE pose
        ee_pose = self.controller.get_current_ee_pose()

        self.assertIsNotNone(ee_pose)
        self.assertIn('position', ee_pose)
        self.assertIn('orientation', ee_pose)

        print(f"\nCurrent EE pose:")
        print(f"  Position: ({ee_pose['position']['x']:.4f}, "
              f"{ee_pose['position']['y']:.4f}, {ee_pose['position']['z']:.4f})")

    def test_realistic_teleoperation_sequence(self):
        """Test realistic teleoperation sequence"""
        print(f"\n{'='*70}")
        print("REALISTIC TELEOPERATION SEQUENCE TEST")
        print('='*70)

        # Simulate 1 second of teleoperation at 100 Hz
        dt = 0.01  # 10ms
        n_steps = 100

        # Start in front
        vr_pos_start = np.array([0.0, 0.0, -0.3])
        vr_quat = np.array([0.0, 0.0, 0.0, 1.0])

        # Move in a small arc
        successful_updates = 0
        ik_failures = 0
        velocity_limits = 0

        for i in range(n_steps):
            # Circular motion
            t = i / n_steps
            angle = t * np.pi / 4  # 45° arc

            vr_pos = vr_pos_start + np.array([
                0.05 * np.sin(angle),  # Side-to-side
                0.02 * (1 - np.cos(angle)),  # Up
                0.0
            ])

            result = self.controller.update(vr_pos, vr_quat, dt=dt)

            if result['success']:
                successful_updates += 1
            if result['state'] == ControllerState.IK_FAILED:
                ik_failures += 1
            if result['state'] == ControllerState.VELOCITY_LIMITED:
                velocity_limits += 1

        stats = self.controller.get_stats()

        print(f"\nSimulation: {n_steps} steps at {1/dt:.0f} Hz")
        print(f"  Duration: {n_steps * dt:.1f} seconds")
        print(f"  Successful updates: {successful_updates}/{n_steps} "
              f"({successful_updates/n_steps*100:.1f}%)")
        print(f"  IK failures: {ik_failures}")
        print(f"  Velocity limits hit: {velocity_limits}")
        print(f"  Fallback activations: {stats['fallback_activations']}")
        print(f"\nFinal statistics:")
        print(f"  Total IK attempts: {stats['total_updates']}")
        print(f"  IK success rate: {stats['ik_success_rate']:.2%}")
        print(f"  Velocity violations: {stats['velocity_violations']}")

        # Should have high success rate
        self.assertGreater(successful_updates / n_steps, 0.8,
                          "Success rate should be > 80%")

    def test_set_joint_state(self):
        """Test setting joint state (for robot feedback integration)"""
        custom_joints = np.array([0.1, 1.0, -1.0, 0.2, 0.1, 0.0])

        self.controller.set_joint_state(custom_joints)

        np.testing.assert_array_almost_equal(
            self.controller.current_joint_angles,
            custom_joints,
            decimal=6
        )

        print(f"\nSet joint state test:")
        print(f"  Custom joints: {custom_joints}")
        print(f"  Current joints: {self.controller.current_joint_angles}")
        print(f"  ✓ State updated")

    def test_different_control_frequencies(self):
        """Test different control frequencies"""
        frequencies = [50, 100, 200]  # Hz

        for freq in frequencies:
            controller = PiperTeleopController(
                self.urdf_path,
                control_frequency=freq
            )

            self.assertEqual(controller.control_frequency, freq)
            self.assertAlmostEqual(controller.control_dt, 1.0/freq)

        print(f"\nControl frequency test:")
        for freq in frequencies:
            print(f"  {freq} Hz → dt = {1.0/freq*1000:.2f} ms ✓")


def run_tests():
    """Run all tests"""
    suite = unittest.TestLoader().loadTestsFromTestCase(TestPiperTeleopController)
    runner = unittest.TextTestRunner(verbosity=2)
    result = runner.run(suite)
    return result.wasSuccessful()


if __name__ == '__main__':
    import sys
    success = run_tests()
    sys.exit(0 if success else 1)
