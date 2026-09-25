#!/usr/bin/env python3
"""
Piper IK Solver using Placo (Improved Version)
Based on teleop_a7lite architecture for better convergence

Key improvements over PyKDL version:
- Uses Placo's constraint-based QP solver
- Maintains robot state for incremental solving
- Automatic joint limit handling
- Built-in regularization towards neutral pose
- Better convergence in singular configurations
"""

import numpy as np
import placo
from typing import Optional, Tuple
from scipy.spatial.transform import Rotation as R


class PiperIKSolverPlacoImproved:
    """
    IK solver for Piper 6-DoF arm using Placo

    Architecture matches teleop_a7lite for consistency and reliability.

    URDF Structure:
    - base_link
    - joint1~joint6 (revolute, Z axis)
    - link6 (EE frame)

    Units: meters, radians
    """

    def __init__(self, urdf_path: str, dt: float = 0.01):
        """
        Initialize Placo-based IK solver

        Args:
            urdf_path: Path to Piper URDF file
            dt: Time step for solver (default 0.01s = 100Hz)
        """
        self.urdf_path = urdf_path
        self.dt = dt

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

        # EE frame name
        self.ee_frame = "link6"

        # Default neutral pose (避免奇异配置)
        self.neutral_pose = np.array([0.0, 0.8, -0.8, 0.0, 0.0, 0.0])

        # Create solver first
        self._setup_solver()

        # Initialize robot state
        self.set_joint_state(self.neutral_pose)

        print(f"[PiperIKSolverPlaco] Initialized with dt={dt}s")
        print(f"  Joint names: {self.joint_names}")
        print(f"  EE frame: {self.ee_frame}")

    def _setup_solver(self):
        """Setup Placo kinematics solver with tasks"""
        self.solver = placo.KinematicsSolver(self.robot)
        self.solver.dt = self.dt

        # Don't mask floating base since Piper has fixed base
        # Placo will automatically handle this

        # Create frame task for end-effector
        # Start with current EE pose
        T_current = self.robot.get_T_world_frame(self.ee_frame)
        self.ee_task = self.solver.add_frame_task(self.ee_frame, T_current)
        self.ee_task.configure("end_effector", "soft", 1.0)

        # Add joint regularization towards neutral pose
        # This helps avoid singular configurations
        joints_task = self.solver.add_joints_task()
        joint_targets = {name: pos for name, pos in zip(self.joint_names, self.neutral_pose)}
        joints_task.set_joints(joint_targets)
        joints_task.configure("joints_regularization", "soft", 1e-4)

        # Add manipulability task to avoid singularities
        manipulability_task = self.solver.add_manipulability_task(self.ee_frame, "both", 1.0)
        manipulability_task.configure("manipulability", "soft", 1e-2)

        print(f"[PiperIKSolverPlaco] Solver setup complete")
        print(f"  - Frame task: {self.ee_frame}")
        print(f"  - Joint regularization: enabled")
        print(f"  - Manipulability task: enabled")

    def set_joint_state(self, joint_angles: np.ndarray):
        """
        Set current joint state

        Args:
            joint_angles: Joint angles [j1, j2, j3, j4, j5, j6] in radians
        """
        if len(joint_angles) != 6:
            raise ValueError(f"Expected 6 joint angles, got {len(joint_angles)}")

        # IMPORTANT: Placo maintains state in robot.state.q, not via set_joint
        # We need to update the state vector directly

        # Find joint indices in the state vector
        joint_indices = [self.robot.get_joint_offset(name) for name in self.joint_names]

        # Set joint values in state vector
        for idx, angle in zip(joint_indices, joint_angles):
            self.robot.state.q[idx] = float(angle)

        # Update kinematics
        self.robot.update_kinematics()

        # Also need to update the solver's internal state if it exists
        # This ensures the EE task is synced with current robot pose
        if hasattr(self, 'ee_task'):
            T_current = self.robot.get_T_world_frame(self.ee_frame)
            self.ee_task.T_world_frame = T_current

    def get_joint_state(self) -> np.ndarray:
        """
        Get current joint state

        Returns:
            Joint angles [j1, j2, j3, j4, j5, j6] in radians
        """
        # Get joint values from state vector
        joint_indices = [self.robot.get_joint_offset(name) for name in self.joint_names]
        return np.array([self.robot.state.q[idx] for idx in joint_indices])

    def compute_fk(self, joint_angles: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        """
        Compute forward kinematics

        Args:
            joint_angles: Joint angles [j1, j2, j3, j4, j5, j6]

        Returns:
            (position, quaternion) where:
            - position: [x, y, z] in meters
            - quaternion: [x, y, z, w]
        """
        # Save current state
        current_state = self.get_joint_state().copy()

        # Set to query state
        self.set_joint_state(joint_angles)

        # Get EE transform
        T = self.robot.get_T_world_frame(self.ee_frame)
        position = T[:3, 3].copy()

        # Extract quaternion from rotation matrix
        rot = R.from_matrix(T[:3, :3])
        quaternion = rot.as_quat()  # [x, y, z, w]

        # Restore original state
        self.set_joint_state(current_state)

        return position, quaternion

    def solve_ik(
        self,
        target_position: np.ndarray,
        target_orientation: np.ndarray,
        seed: Optional[np.ndarray] = None,
        max_iterations: int = 200,
        position_tolerance: float = 1e-4,
        orientation_tolerance: float = 1e-4
    ) -> Optional[np.ndarray]:
        """
        Solve IK for target pose using Placo

        Args:
            target_position: Target position [x, y, z] in meters
            target_orientation: Target orientation as quaternion [x, y, z, w]
            seed: Initial joint angles (optional, uses current state if None)
            max_iterations: Maximum solver iterations
            position_tolerance: Position convergence threshold (meters)
            orientation_tolerance: Orientation convergence threshold (radians)

        Returns:
            Joint angles [j1, j2, j3, j4, j5, j6] or None if failed
        """
        # Save current state to restore if IK fails
        original_state = self.get_joint_state().copy()

        # Set seed if provided
        if seed is not None:
            if len(seed) != 6:
                raise ValueError(f"Expected 6 seed angles, got {len(seed)}")
            self.set_joint_state(seed)
        else:
            # Use current state as seed
            pass

        # Build target transform
        T_target = np.eye(4)
        rot = R.from_quat(target_orientation)
        T_target[:3, :3] = rot.as_matrix()
        T_target[:3, 3] = target_position

        # Update task target
        self.ee_task.T_world_frame = T_target

        # Solve iteratively
        converged = False
        for i in range(max_iterations):
            try:
                # Solve one step
                self.solver.solve(True)

                # Check convergence
                T_current = self.robot.get_T_world_frame(self.ee_frame)

                # Position error
                pos_error = np.linalg.norm(T_current[:3, 3] - target_position)

                # Orientation error (using rotation matrix difference)
                R_error = T_target[:3, :3].T @ T_current[:3, :3]
                angle_error = np.arccos(np.clip((np.trace(R_error) - 1) / 2, -1, 1))

                # Check convergence
                if pos_error < position_tolerance and angle_error < orientation_tolerance:
                    # Success!
                    solution = self.get_joint_state().copy()

                    # Verify joint limits
                    if self.check_joint_limits(solution):
                        converged = True
                        break
                    else:
                        # Joint limits violated, restore and fail
                        self.set_joint_state(original_state)
                        return None

            except Exception as e:
                print(f"[PiperIKSolverPlaco] Solver exception: {e}")
                self.set_joint_state(original_state)
                return None

        if not converged:
            # Max iterations reached
            # Check if we're close enough (relaxed tolerance)
            T_current = self.robot.get_T_world_frame(self.ee_frame)
            pos_error = np.linalg.norm(T_current[:3, 3] - target_position)
            R_error = T_target[:3, :3].T @ T_current[:3, :3]
            angle_error = np.arccos(np.clip((np.trace(R_error) - 1) / 2, -1, 1))

            # Accept solution if reasonably close (10mm position, 5 degree orientation)
            if pos_error < 0.01 and angle_error < np.deg2rad(5):
                solution = self.get_joint_state().copy()
                if self.check_joint_limits(solution):
                    return solution

            # Not close enough, restore original state and fail
            self.set_joint_state(original_state)
            return None

        return solution

    def solve_ik_incremental(
        self,
        target_position: np.ndarray,
        target_orientation: np.ndarray,
        num_steps: int = 1
    ) -> Optional[np.ndarray]:
        """
        Solve IK incrementally (better for continuous control)

        This method updates the robot state incrementally, making it suitable
        for real-time teleoperation where targets change smoothly.

        Args:
            target_position: Target position [x, y, z] in meters
            target_orientation: Target orientation as quaternion [x, y, z, w]
            num_steps: Number of solver steps (default 1 for real-time)

        Returns:
            Joint angles [j1, j2, j3, j4, j5, j6] or None if failed
        """
        # Build target transform
        T_target = np.eye(4)
        rot = R.from_quat(target_orientation)
        T_target[:3, :3] = rot.as_matrix()
        T_target[:3, 3] = target_position

        # Update task target
        self.ee_task.T_world_frame = T_target

        # Solve for specified number of steps
        for _ in range(num_steps):
            try:
                self.solver.solve(True)
            except Exception as e:
                print(f"[PiperIKSolverPlaco] Solver exception: {e}")
                return None

        # Get solution
        solution = self.get_joint_state()

        # Verify joint limits
        if self.check_joint_limits(solution):
            return solution
        else:
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

    def get_joint_info(self) -> dict:
        """Get joint limit information"""
        return {
            'n_joints': 6,
            'joint_names': self.joint_names,
            'position_limits_lower': self.joint_limits_lower.tolist(),
            'position_limits_upper': self.joint_limits_upper.tolist(),
        }

    def reset_to_neutral(self):
        """Reset robot to neutral pose"""
        self.set_joint_state(self.neutral_pose)
