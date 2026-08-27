#!/usr/bin/env python3
"""Offline trajectory test for the Piper V2 IK backend."""

from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation

from piper_ik_v2 import PiperPinocchioIK


def main() -> int:
    repo = Path(__file__).resolve().parent
    solver = PiperPinocchioIK(repo / "assets" / "piper_description.urdf")
    initial_joints = np.array([0.0, 1.5, -1.5, 0.3, 0.2, 0.1])
    reference = solver.forward_transform(initial_joints)
    previous = initial_joints.copy()
    failures = 0
    max_position_error = 0.0
    max_orientation_error = 0.0
    max_joint_step = 0.0
    min_singular_value = float("inf")

    for frame in range(121):
        phase = 2.0 * np.pi * frame / 120.0
        position_delta = np.array(
            [
                0.025 * np.sin(phase),
                0.015 * (1.0 - np.cos(phase)),
                0.015 * np.sin(2.0 * phase),
            ]
        )
        rotation_delta = Rotation.from_euler(
            "xyz",
            [
                np.deg2rad(8.0) * np.sin(phase),
                np.deg2rad(6.0) * np.sin(2.0 * phase),
                np.deg2rad(12.0) * np.sin(phase),
            ],
        )
        target_position = reference.translation + position_delta
        target_rotation = rotation_delta.as_matrix() @ reference.rotation
        target_quaternion = Rotation.from_matrix(target_rotation).as_quat()
        result = solver.solve(
            target_position,
            target_quaternion,
            previous,
            posture_reference_joint_angles_rad=initial_joints,
        )
        if not result.success:
            failures += 1
            continue
        joint_step = float(np.max(np.abs(result.joint_angles_rad - previous)))
        max_joint_step = max(max_joint_step, joint_step)
        max_position_error = max(max_position_error, result.position_error_m)
        max_orientation_error = max(max_orientation_error, result.orientation_error_rad)
        min_singular_value = min(min_singular_value, result.min_singular_value)
        previous = result.joint_angles_rad

    returned_home = float(np.max(np.abs(previous - initial_joints)))
    passed = (
        failures == 0
        and max_position_error <= solver.position_tolerance_m
        and max_joint_step < 0.12
        and returned_home < 0.03
        and np.all(np.isfinite(previous))
    )
    print(
        "V2 IK DRY-RUN SUMMARY: "
        f"frames=121, failures={failures}, "
        f"max_position_error_mm={max_position_error * 1000.0:.3f}, "
        f"max_orientation_error_deg={np.rad2deg(max_orientation_error):.3f}, "
        f"max_joint_step_deg={np.rad2deg(max_joint_step):.3f}, "
        f"returned_home_max_deg={np.rad2deg(returned_home):.3f}, "
        f"min_singular_value={min_singular_value:.6f}, "
        f"result={'PASS' if passed else 'FAIL'}"
    )
    return 0 if passed else 1


if __name__ == "__main__":
    raise SystemExit(main())
