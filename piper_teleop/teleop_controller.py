#!/usr/bin/env python3
"""
Piper Teleoperation Controller (M5)
Integrates: Pico VR Input → Mapping (M3) → IK (M4) → Joint Commands

Full pipeline without real Piper hardware connection
"""

import numpy as np
from typing import Optional, Dict, Tuple
from enum import Enum

from piper_teleop.teleop_mapping import TeleopMapper
from piper_teleop.ik_solver import PiperIKSolver


class ControllerState(Enum):
    """Controller operational state"""
    IDLE = "idle"
    ACTIVE = "active"
    IK_FAILED = "ik_failed"
    VELOCITY_LIMITED = "velocity_limited"
    JOINT_LIMITED = "joint_limited"


class PiperTeleopController:
    """
    Complete teleoperation controller integrating VR input, mapping, and IK

    Pipeline:
    1. Pico VR Input (position, orientation)
    2. M3 Teleoperation Mapping (VR space → Robot workspace)
    3. M4 IK Solver (Target EE pose → Joint angles)
    4. Safety checks (joint limits, velocity limits)
    5. Fallback strategies (IK failure handling)

    Usage:
        controller = PiperTeleopController(urdf_path)

        # Each control cycle
        result = controller.update(
            vr_position,
            vr_orientation,
            dt=0.01
        )

        if result['success']:
            joint_target = result['joint_angles']
            # Send to robot (M6)
    """

    def __init__(
        self,
        urdf_path: str,
        control_frequency: float = 100.0,  # Hz
        max_joint_velocity: float = 5.0,   # rad/s (from URDF)
        enable_velocity_limiting: bool = True,
        enable_fallback_on_ik_failure: bool = True,
        mapping_config: Optional[Dict] = None
    ):
        """
        Initialize teleoperation controller

        Args:
            urdf_path: Path to Piper URDF
            control_frequency: Control loop frequency (Hz)
            max_joint_velocity: Maximum joint velocity (rad/s)
            enable_velocity_limiting: Enable velocity limit checking
            enable_fallback_on_ik_failure: Hold last valid position on IK failure
            mapping_config: Optional mapping configuration (uses default if None)
        """
        # M3: Teleoperation mapper with default config
        if mapping_config is None:
            mapping_config = self._get_default_mapping_config()

        self.mapper = TeleopMapper(mapping_config)

        # M4: IK solver
        self.ik_solver = PiperIKSolver(urdf_path)

        # Control parameters
        self.control_frequency = control_frequency
        self.control_dt = 1.0 / control_frequency
        self.max_joint_velocity = max_joint_velocity
        self.enable_velocity_limiting = enable_velocity_limiting
        self.enable_fallback = enable_fallback_on_ik_failure

        # State tracking
        self.state = ControllerState.IDLE
        self.current_joint_angles = None  # Current/last valid joint angles (seed for IK)
        self.last_valid_joint_angles = None
        self.last_target_pose = None

        # Statistics
        self.stats = {
            'total_updates': 0,
            'successful_ik': 0,
            'failed_ik': 0,
            'velocity_violations': 0,
            'joint_limit_violations': 0,
            'fallback_activations': 0
        }

        # Initialize with mid-range configuration
        self._initialize_joint_state()

        # Initialize mapper reference with current EE pose
        self._initialize_mapper_reference()

    def _get_default_mapping_config(self) -> Dict:
        """Get default mapping configuration for testing"""
        return {
            'position_mapping': {
                'scale': 1.0,
                'max_delta': 0.5
            },
            'rotation_mapping': {
                'scale': 1.0,
                'max_delta': 3.14
            },
            'xr_to_piper_transform': {
                'translation': [0.0, 0.0, 0.0],
                'rotation': [0.0, 0.0, 0.0]  # Euler angles in radians
            },
            'safety': {
                'workspace_limit': {
                    'x_min': -0.1, 'x_max': 0.6,  # 修复: 包含初始位置
                    'y_min': -0.4, 'y_max': 0.4,
                    'z_min': 0.0, 'z_max': 0.8
                },
                'enable_workspace_limit': True
            }
        }

    def _initialize_joint_state(self):
        """Initialize joint state to a known-good configuration"""
        # Use a verified configuration that IK can solve from
        # NOT mid-range which may be near singularity

        # Configuration validated by FK→IK testing
        # This configuration has been verified to:
        # 1. Be within joint limits
        # 2. Allow IK convergence from nearby poses
        # 3. Place EE in reasonable workspace position
        self.current_joint_angles = np.array([
            0.0,    # joint1: center
            0.8,    # joint2: ~46° (avoid 90° singularity)
            -0.8,   # joint3: ~-46°
            0.0,    # joint4: center
            0.0,    # joint5: center
            0.0     # joint6: center
        ])

        self.last_valid_joint_angles = self.current_joint_angles.copy()

        # Log initial configuration
        ee_pos, ee_quat = self.ik_solver.compute_fk(self.current_joint_angles)
        print(f"[TeleopController] Initial configuration:")
        print(f"  Joints [deg]: {np.rad2deg(self.current_joint_angles)}")
        print(f"  EE position: [{ee_pos[0]:.3f}, {ee_pos[1]:.3f}, {ee_pos[2]:.3f}] m")

    def _initialize_mapper_reference(self):
        """Initialize mapper with current EE pose and default VR pose"""
        # Get current EE pose from FK
        ee_pos, ee_quat = self.ik_solver.compute_fk(self.current_joint_angles)

        # Default VR pose (in front of user at comfortable height)
        vr_pos = np.array([0.0, 0.0, -0.3])
        vr_quat = np.array([0.0, 0.0, 0.0, 1.0])

        # Set mapper reference
        self.mapper.set_reference(vr_pos, vr_quat, ee_pos, ee_quat)

    def reset(self):
        """Reset controller to initial state"""
        self._initialize_joint_state()
        self._initialize_mapper_reference()
        self.state = ControllerState.IDLE
        self.last_target_pose = None

        # Reset stats
        for key in self.stats:
            self.stats[key] = 0

    def set_joint_state(self, joint_angles: np.ndarray):
        """
        Set current joint state (e.g., from real robot feedback)

        Args:
            joint_angles: Current joint angles [j1~j6] in radians
        """
        if len(joint_angles) != 6:
            raise ValueError(f"Expected 6 joint angles, got {len(joint_angles)}")

        self.current_joint_angles = joint_angles.copy()
        self.last_valid_joint_angles = joint_angles.copy()

        # Update mapper reference with new EE pose
        ee_pos, ee_quat = self.ik_solver.compute_fk(joint_angles)
        vr_pos = np.array([0.0, 0.0, -0.3])
        vr_quat = np.array([0.0, 0.0, 0.0, 1.0])
        self.mapper.set_reference(vr_pos, vr_quat, ee_pos, ee_quat)

    def update(
        self,
        vr_position: np.ndarray,
        vr_orientation: np.ndarray,
        dt: Optional[float] = None,
        gripper_value: float = 0.0
    ) -> Dict:
        """
        Update teleoperation control

        Args:
            vr_position: VR controller position [x, y, z] in meters
            vr_orientation: VR controller quaternion [x, y, z, w]
            dt: Time step (seconds), uses control_dt if None
            gripper_value: Gripper value 0.0~1.0 (not used in M5)

        Returns:
            dict with:
                - success: bool
                - joint_angles: np.ndarray (if success)
                - state: ControllerState
                - target_pose: dict with position and orientation
                - error_message: str (if failure)
                - stats: dict
        """
        self.stats['total_updates'] += 1

        if dt is None:
            dt = self.control_dt

        # Step 1: M3 Mapping (VR → Robot workspace)
        target_pose_msg = self.mapper.compute_target_pose(vr_position, vr_orientation)

        if target_pose_msg is None:
            # Mapper not active yet, use current EE pose
            ee_pos, ee_quat = self.ik_solver.compute_fk(self.current_joint_angles)
            target_pos = ee_pos
            target_quat = ee_quat
            target_pose = {
                'position': {'x': ee_pos[0], 'y': ee_pos[1], 'z': ee_pos[2]},
                'orientation': {'x': ee_quat[0], 'y': ee_quat[1], 'z': ee_quat[2], 'w': ee_quat[3]}
            }
        else:
            target_pos = np.array([
                target_pose_msg.position.x,
                target_pose_msg.position.y,
                target_pose_msg.position.z
            ])

            target_quat = np.array([
                target_pose_msg.orientation.x,
                target_pose_msg.orientation.y,
                target_pose_msg.orientation.z,
                target_pose_msg.orientation.w
            ])

            target_pose = {
                'position': {'x': target_pos[0], 'y': target_pos[1], 'z': target_pos[2]},
                'orientation': {'x': target_quat[0], 'y': target_quat[1], 'z': target_quat[2], 'w': target_quat[3]}
            }

        self.last_target_pose = target_pose

        # Step 2: M4 IK Solver (Target pose → Joint angles)
        # Use current joint angles as seed for continuity
        joint_solution = self.ik_solver.compute_ik(
            target_pos,
            target_quat,
            seed_angles=self.current_joint_angles
        )

        if joint_solution is None:
            # IK failed
            self.stats['failed_ik'] += 1
            self.state = ControllerState.IK_FAILED

            if self.enable_fallback:
                # Fallback: hold last valid position
                self.stats['fallback_activations'] += 1
                return {
                    'success': True,  # Success with fallback
                    'joint_angles': self.last_valid_joint_angles.copy(),
                    'state': self.state,
                    'target_pose': target_pose,
                    'error_message': 'IK failed, using fallback (last valid)',
                    'stats': self.stats.copy(),
                    'is_fallback': True
                }
            else:
                return {
                    'success': False,
                    'state': self.state,
                    'target_pose': target_pose,
                    'error_message': 'IK failed, no valid solution',
                    'stats': self.stats.copy()
                }

        # Step 3: Safety checks

        # 3a: Joint limits
        if not self.ik_solver.check_joint_limits(joint_solution):
            self.stats['joint_limit_violations'] += 1
            self.state = ControllerState.JOINT_LIMITED

            if self.enable_fallback:
                self.stats['fallback_activations'] += 1
                return {
                    'success': True,
                    'joint_angles': self.last_valid_joint_angles.copy(),
                    'state': self.state,
                    'target_pose': target_pose,
                    'error_message': 'Joint limits exceeded, using fallback',
                    'stats': self.stats.copy(),
                    'is_fallback': True
                }
            else:
                return {
                    'success': False,
                    'state': self.state,
                    'target_pose': target_pose,
                    'error_message': 'Joint limits exceeded',
                    'stats': self.stats.copy()
                }

        # 3b: Velocity limits
        if self.enable_velocity_limiting:
            if not self.ik_solver.check_velocity_limits(
                self.current_joint_angles,
                joint_solution,
                dt
            ):
                self.stats['velocity_violations'] += 1
                self.state = ControllerState.VELOCITY_LIMITED

                # Scale down velocity to meet limit
                joint_solution = self._apply_velocity_limiting(
                    self.current_joint_angles,
                    joint_solution,
                    dt
                )

        # Step 4: Update state
        self.stats['successful_ik'] += 1
        self.current_joint_angles = joint_solution.copy()
        self.last_valid_joint_angles = joint_solution.copy()
        self.state = ControllerState.ACTIVE

        return {
            'success': True,
            'joint_angles': joint_solution.copy(),
            'state': self.state,
            'target_pose': target_pose,
            'stats': self.stats.copy(),
            'is_fallback': False
        }

    def _apply_velocity_limiting(
        self,
        current: np.ndarray,
        target: np.ndarray,
        dt: float
    ) -> np.ndarray:
        """
        Scale target joints to meet velocity limits

        Args:
            current: Current joint angles
            target: Target joint angles
            dt: Time step

        Returns:
            Limited joint angles
        """
        delta = target - current
        velocities = delta / dt

        # Find maximum velocity ratio
        max_ratio = 0.0
        for i in range(6):
            vel_limit = self.ik_solver.joint_velocity_limits[i]
            if abs(velocities[i]) > vel_limit:
                ratio = abs(velocities[i]) / vel_limit
                max_ratio = max(max_ratio, ratio)

        if max_ratio > 1.0:
            # Scale down to meet limit
            limited_delta = delta / max_ratio
            return current + limited_delta
        else:
            return target

    def get_current_ee_pose(self) -> Dict:
        """
        Get current end-effector pose (FK)

        Returns:
            dict with position and orientation
        """
        if self.current_joint_angles is None:
            return None

        pos, quat = self.ik_solver.compute_fk(self.current_joint_angles)

        return {
            'position': {'x': pos[0], 'y': pos[1], 'z': pos[2]},
            'orientation': {'x': quat[0], 'y': quat[1], 'z': quat[2], 'w': quat[3]}
        }

    def get_state(self) -> ControllerState:
        """Get current controller state"""
        return self.state

    def get_stats(self) -> Dict:
        """Get controller statistics"""
        stats = self.stats.copy()

        # Add derived metrics
        if stats['total_updates'] > 0:
            stats['ik_success_rate'] = stats['successful_ik'] / stats['total_updates']
            stats['ik_failure_rate'] = stats['failed_ik'] / stats['total_updates']
        else:
            stats['ik_success_rate'] = 0.0
            stats['ik_failure_rate'] = 0.0

        return stats

    def get_joint_info(self) -> Dict:
        """Get joint information from IK solver"""
        return self.ik_solver.get_joint_info()
