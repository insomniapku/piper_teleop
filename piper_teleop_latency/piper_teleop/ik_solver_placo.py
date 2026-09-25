#!/usr/bin/env python3
"""
Piper IK Solver using Placo (M4 improved)
Replaces PyKDL with Placo for better convergence

Based on Piper URDF with 6-DoF arm (joint1~joint6)
EE frame: link6
"""

import numpy as np
import placo
from typing import Optional, Tuple


class PiperIKSolverPlaco:
    """
    IK solver for Piper 6-DoF arm using Placo

    URDF Structure:
    - base_link
    - joint1~joint6 (revolute, Z axis)
    - link6 (EE frame)

    Units: meters, radians
    """

    def __init__(self, urdf_path: str):
        """
        Initialize IK solver from URDF

        Args:
            urdf_path: Path to Piper URDF file
        """
        self.urdf_path = urdf_path

        # Load robot with Placo
        self.robot = placo.RobotWrapper(urdf_path, placo.Flags.ignore_collisions)

        # Joint names
        self.joint_names = [f"joint{i}" for i in range(1, 7)]

        # Joint limits (from URDF)
        self.joint_limits_lower = np.array([
            -2.6179938,  # joint1
            0.0,         # joint2
            -2.9670597,  # joint3
            -1.7453292,  # joint4
            -1.7453292,  # joint5
            -3.1415926,  # joint6
        ])

        self.joint_limits_upper = np.array([
            2.6179938,   # joint1
            3.1415926,   # joint2
            0.3490658,   # joint3
            1.7453292,   # joint4
            1.7453292,   # joint5
            3.1415926,   # joint6
        ])

        # EE frame name
        self.ee_frame = "link6"

        # Default seed (neutral pose from M3 config)
        self.default_seed = np.array([0.0, 0.8, -0.8, 0.0, 0.0, 0.0])

    def solve_ik(
        self,
        target_position: np.ndarray,
        target_orientation: np.ndarray,
        seed: Optional[np.ndarray] = None,
        retry_with_random_seeds: bool = True
    ) -> Optional[np.ndarray]:
        """
        Solve IK for target pose

        Args:
            target_position: Target position [x, y, z] in meters
            target_orientation: Target orientation as rotation matrix (3x3)
            seed: Initial joint angles (optional)
            retry_with_random_seeds: Try random seeds if initial solve fails

        Returns:
            Joint angles [j1, j2, j3, j4, j5, j6] or None if failed
        """
        # Use default seed if not provided
        if seed is None:
            seed = self.default_seed.copy()

        # Try with initial seed
        result = self._solve_ik_with_seed(target_position, target_orientation, seed)
        if result is not None:
            return result

        # Try with random seeds if enabled
        if retry_with_random_seeds:
            for _ in range(10):
                random_seed = np.random.uniform(
                    self.joint_limits_lower,
                    self.joint_limits_upper
                )
                result = self._solve_ik_with_seed(target_position, target_orientation, random_seed)
                if result is not None:
                    return result

        return None

    def _solve_ik_with_seed(
        self,
        target_position: np.ndarray,
        target_orientation: np.ndarray,
        seed_angles: np.ndarray,
        max_iterations: int = 100,
        position_tolerance: float = 1e-3,
        orientation_tolerance: float = 1e-3
    ) -> Optional[np.ndarray]:
        """
        Attempt IK solve with a specific seed using Placo

        Args:
            target_position: Target position [x, y, z] in meters
            target_orientation: Target orientation as rotation matrix (3x3)
            seed_angles: Seed joint angles
            max_iterations: Maximum solver iterations
            position_tolerance: Position convergence threshold (meters)
            orientation_tolerance: Orientation convergence threshold (radians)

        Returns:
            Joint angles or None if failed
        """
        try:
            # Get joint indices in state vector
            joint_indices = [self.robot.get_joint_offset(name) for name in self.joint_names]

            # Set robot state to seed using state vector
            for idx, angle in zip(joint_indices, seed_angles):
                self.robot.state.q[idx] = float(angle)

            # Set floating base to identity (fixed at origin)
            self.robot.state.q[0:3] = [0, 0, 0]  # position
            self.robot.state.q[3:7] = [0, 0, 0, 1]  # quaternion

            self.robot.update_kinematics()

            # Create Placo solver
            solver = placo.KinematicsSolver(self.robot)
            solver.mask_fbase(True)  # Mask floating base
            solver.dt = 0.01

            # Add frame task for EE
            T_target = np.eye(4)
            T_target[:3, :3] = target_orientation
            T_target[:3, 3] = target_position

            task = solver.add_frame_task(self.ee_frame, T_target)
            task.configure("ee_task", "soft", 1.0)

            # Iterative solving with convergence check
            for iteration in range(max_iterations):
                # Solve one step
                solver.solve(True)

                # Check convergence based on actual error
                T_current = self.robot.get_T_world_frame(self.ee_frame)

                # Position error
                pos_error = np.linalg.norm(T_current[:3, 3] - target_position)

                # Orientation error (Frobenius norm of rotation difference)
                R_diff = target_orientation.T @ T_current[:3, :3]
                angle_error = np.arccos(np.clip((np.trace(R_diff) - 1) / 2, -1, 1))

                # Check if converged
                if pos_error < position_tolerance and angle_error < orientation_tolerance:
                    # Extract solution from state vector
                    joint_angles = np.array([self.robot.state.q[idx] for idx in joint_indices])

                    # Verify joint limits
                    if self.check_joint_limits(joint_angles):
                        return joint_angles
                    else:
                        return None

            # Max iterations reached, check if close enough
            T_current = self.robot.get_T_world_frame(self.ee_frame)
            pos_error = np.linalg.norm(T_current[:3, 3] - target_position)
            R_diff = target_orientation.T @ T_current[:3, :3]
            angle_error = np.arccos(np.clip((np.trace(R_diff) - 1) / 2, -1, 1))

            # Accept if within 10x tolerance
            if pos_error < position_tolerance * 10 and angle_error < orientation_tolerance * 10:
                joint_angles = np.array([self.robot.state.q[idx] for idx in joint_indices])
                if self.check_joint_limits(joint_angles):
                    return joint_angles

            return None

        except Exception as e:
            return None

    def check_joint_limits(self, joint_angles: np.ndarray) -> bool:
        """
        Check if joint angles are within limits

        Args:
            joint_angles: Joint angles [j1, j2, j3, j4, j5, j6]

        Returns:
            True if within limits
        """
        if len(joint_angles) != 6:
            return False

        for i in range(6):
            if joint_angles[i] < self.joint_limits_lower[i] - 1e-6:
                return False
            if joint_angles[i] > self.joint_limits_upper[i] + 1e-6:
                return False

        return True

    def compute_fk(self, joint_angles: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        """
        Compute forward kinematics

        Args:
            joint_angles: Joint angles [j1, j2, j3, j4, j5, j6]

        Returns:
            (position, orientation) where orientation is 3x3 rotation matrix
        """
        # Get joint indices in state vector
        joint_indices = [self.robot.get_joint_offset(name) for name in self.joint_names]

        # Set robot state using state vector
        for idx, angle in zip(joint_indices, joint_angles):
            self.robot.state.q[idx] = float(angle)

        # Set floating base to identity
        self.robot.state.q[0:3] = [0, 0, 0]
        self.robot.state.q[3:7] = [0, 0, 0, 1]

        self.robot.update_kinematics()

        # Get EE frame transform
        T = self.robot.get_T_world_frame(self.ee_frame)

        position = T[:3, 3].copy()
        orientation = T[:3, :3].copy()

        return position, orientation
