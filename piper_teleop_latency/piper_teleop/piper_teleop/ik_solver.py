#!/usr/bin/env python3
"""
Piper IK Solver (M4)
Uses PyKDL for inverse kinematics with Newton-Raphson method

Based on Piper URDF with 6-DoF arm (joint1~joint6)
EE frame: link6
"""

import numpy as np
from PyKDL import *
from typing import Optional, Tuple
import xml.etree.ElementTree as ET


class PiperIKSolver:
    """
    IK solver for Piper 6-DoF arm using PyKDL

    URDF Structure:
    - base_link
    - joint1 (revolute, Z axis)
    - joint2 (revolute, Z axis)
    - joint3 (revolute, Z axis)
    - joint4 (revolute, Z axis)
    - joint5 (revolute, Z axis)
    - joint6 (revolute, Z axis)
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

        # Parse URDF and build KDL chain
        self.chain = self._build_kdl_chain_from_urdf()

        # Joint limits (from URDF)
        self.joint_limits_lower = np.array([
            -2.6179938,  # joint1
            0.0,         # joint2
            -2.9670597,  # joint3
            -1.7453292,  # joint4
            -1.2217304,  # joint5
            -2.0943951   # joint6
        ])

        self.joint_limits_upper = np.array([
            2.6179938,   # joint1
            3.1415926,   # joint2
            0.0,         # joint3
            1.7453292,   # joint4
            1.2217304,   # joint5
            2.0943951    # joint6
        ])

        # Joint velocity limits (from URDF)
        self.joint_velocity_limits = np.array([5.0] * 6)

        # Create FK and IK solvers
        self.fk_solver = ChainFkSolverPos_recursive(self.chain)
        self.ik_vel_solver = ChainIkSolverVel_pinv(self.chain)

        # Increase max iterations and relax epsilon for better convergence
        self.ik_pos_solver = ChainIkSolverPos_NR(
            self.chain,
            self.fk_solver,
            self.ik_vel_solver,
            maxiter=500,    # Increased from 100
            eps=1e-4        # Relaxed from 1e-6
        )

        self.n_joints = 6

    def _build_kdl_chain_from_urdf(self) -> Chain:
        """
        Build KDL chain from URDF file

        Returns:
            KDL Chain from base_link to link6
        """
        tree = ET.parse(self.urdf_path)
        root = tree.getroot()

        chain = Chain()

        # Joint definitions from URDF
        joint_data = [
            # (parent_link, child_link, xyz, rpy, axis, lower, upper)
            ('base_link', 'link1', [0, 0, 0.123], [0, 0, 0], [0, 0, 1], -2.6179938, 2.6179938),
            ('link1', 'link2', [0, 0, 0], [1.5707963, -0.1357866, -3.1415926], [0, 0, 1], 0, 3.1415926),
            ('link2', 'link3', [0.28503, 0, 0], [0, 0, -1.7938494], [0, 0, 1], -2.9670597, 0),
            ('link3', 'link4', [-0.02198, -0.25075, 0], [1.5707963, 0, 0], [0, 0, 1], -1.7453292, 1.7453292),
            ('link4', 'link5', [0, 0, 0], [-1.5707963, 0, 0], [0, 0, 1], -1.2217304, 1.2217304),
            ('link5', 'link6', [8.8259e-05, -0.091, 0], [1.5707963, 0, 0], [0, 0, 1], -2.0943951, 2.0943951),
        ]

        for parent, child, xyz, rpy, axis, lower, upper in joint_data:
            # Create frame from origin
            frame = Frame()

            # Set translation
            frame.p = Vector(xyz[0], xyz[1], xyz[2])

            # Set rotation from RPY
            frame.M = Rotation.RPY(rpy[0], rpy[1], rpy[2])

            # Create joint
            joint_axis = Vector(axis[0], axis[1], axis[2])
            joint = Joint(Joint.RotZ)  # All joints rotate around Z in their local frame

            # Create segment
            segment = Segment(child, joint, frame)
            chain.addSegment(segment)

        return chain

    def compute_fk(self, joint_angles: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        """
        Compute forward kinematics

        Args:
            joint_angles: Joint angles [j1, j2, j3, j4, j5, j6] in radians

        Returns:
            (position, quaternion) where:
            - position: [x, y, z] in meters
            - quaternion: [x, y, z, w]
        """
        if len(joint_angles) != 6:
            raise ValueError(f"Expected 6 joint angles, got {len(joint_angles)}")

        # Create KDL joint array
        q = JntArray(6)
        for i in range(6):
            q[i] = joint_angles[i]

        # Compute FK
        ee_frame = Frame()
        self.fk_solver.JntToCart(q, ee_frame)

        # Extract position
        position = np.array([ee_frame.p.x(), ee_frame.p.y(), ee_frame.p.z()])

        # Extract rotation as quaternion
        quat = ee_frame.M.GetQuaternion()
        quaternion = np.array([quat[0], quat[1], quat[2], quat[3]])  # [x, y, z, w]

        return position, quaternion

    def compute_ik(
        self,
        target_pos: np.ndarray,
        target_quat: np.ndarray,
        seed_angles: Optional[np.ndarray] = None,
        max_iterations: int = 100,
        tolerance: float = 1e-6,
        retry_with_random_seeds: bool = True
    ) -> Optional[np.ndarray]:
        """
        Compute inverse kinematics

        Args:
            target_pos: Target position [x, y, z] in meters
            target_quat: Target quaternion [x, y, z, w]
            seed_angles: Seed joint angles [j1, j2, j3, j4, j5, j6] in radians
            max_iterations: Maximum IK iterations
            tolerance: Position/orientation tolerance
            retry_with_random_seeds: If True, retry with random seeds on failure

        Returns:
            Joint angles [j1, j2, j3, j4, j5, j6] in radians, or None if IK fails
        """
        # Validate inputs
        if len(target_pos) != 3:
            raise ValueError(f"Expected 3D position, got {len(target_pos)}")
        if len(target_quat) != 4:
            raise ValueError(f"Expected quaternion [x,y,z,w], got {len(target_quat)}")

        # Normalize quaternion
        quat_norm = np.linalg.norm(target_quat)
        if quat_norm < 1e-8:
            target_quat = np.array([0, 0, 0, 1])
        else:
            target_quat = target_quat / quat_norm

        # Create target frame
        target_frame = Frame()
        target_frame.p = Vector(target_pos[0], target_pos[1], target_pos[2])
        target_frame.M = Rotation.Quaternion(
            target_quat[0], target_quat[1], target_quat[2], target_quat[3]
        )

        # Try with provided seed first
        if seed_angles is not None:
            if len(seed_angles) != 6:
                raise ValueError(f"Expected 6 seed angles, got {len(seed_angles)}")

            result = self._solve_ik_with_seed(target_frame, seed_angles)
            if result is not None:
                return result

        # Try with mid-range seed
        mid_range = np.array([(self.joint_limits_lower[i] + self.joint_limits_upper[i]) / 2.0
                              for i in range(6)])
        result = self._solve_ik_with_seed(target_frame, mid_range)
        if result is not None:
            return result

        # Try with random seeds if enabled
        if retry_with_random_seeds:
            # Increase number of random seeds from 5 to 20
            for _ in range(20):
                random_seed = np.random.uniform(
                    self.joint_limits_lower,
                    self.joint_limits_upper
                )
                result = self._solve_ik_with_seed(target_frame, random_seed)
                if result is not None:
                    return result

        # All attempts failed
        return None

    def _solve_ik_with_seed(self, target_frame: Frame, seed_angles: np.ndarray) -> Optional[np.ndarray]:
        """
        Attempt IK solve with a specific seed

        Args:
            target_frame: Target frame
            seed_angles: Seed joint angles

        Returns:
            Joint angles or None if failed
        """
        # Create seed
        q_init = JntArray(6)
        for i in range(6):
            q_init[i] = seed_angles[i]

        # Solve IK
        q_out = JntArray(6)
        result = self.ik_pos_solver.CartToJnt(q_init, target_frame, q_out)

        if result < 0:
            # IK failed
            return None

        # Extract solution
        joint_angles = np.array([q_out[i] for i in range(6)])

        # Check joint limits
        if not self.check_joint_limits(joint_angles):
            return None

        return joint_angles

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

    def check_velocity_limits(
        self,
        current_angles: np.ndarray,
        target_angles: np.ndarray,
        dt: float
    ) -> bool:
        """
        Check if velocity between current and target is within limits

        Args:
            current_angles: Current joint angles
            target_angles: Target joint angles
            dt: Time step (seconds)

        Returns:
            True if velocity is within limits
        """
        if dt <= 0:
            return False

        velocities = (target_angles - current_angles) / dt

        for i in range(6):
            if abs(velocities[i]) > self.joint_velocity_limits[i]:
                return False

        return True

    def compute_ik_with_regularization(
        self,
        target_pos: np.ndarray,
        target_quat: np.ndarray,
        seed_angles: np.ndarray,
        regularization_weight: float = 0.01
    ) -> Optional[np.ndarray]:
        """
        Compute IK with joint regularization (prefer staying close to seed)

        Args:
            target_pos: Target position
            target_quat: Target quaternion
            seed_angles: Seed/reference joint angles
            regularization_weight: Weight for staying close to seed

        Returns:
            Joint angles or None
        """
        # First try standard IK
        solution = self.compute_ik(target_pos, target_quat, seed_angles)

        if solution is None:
            return None

        # Check if solution is close to seed (regularization preference)
        # If solution deviates significantly, could try optimization here
        # For now, just return the solution

        return solution

    def get_joint_info(self) -> dict:
        """Get joint limit and velocity information"""
        return {
            'n_joints': self.n_joints,
            'joint_names': ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6'],
            'position_limits_lower': self.joint_limits_lower.tolist(),
            'position_limits_upper': self.joint_limits_upper.tolist(),
            'velocity_limits': self.joint_velocity_limits.tolist(),
        }
