#!/usr/bin/env python3
"""
Teleoperation Mapping Module (M3)
Handles delta pose mapping from XR controller to Piper EE target

Reference-based delta mapping to avoid jumps when re-gripping:
  p_target = p_ee_ref + scale * R_map * (p_xr - p_xr_ref)
  R_target = delta_R_piper * R_ee_ref

Coordinate transform R_map is configurable and requires calibration.
"""

import numpy as np
from scipy.spatial.transform import Rotation as R
from geometry_msgs.msg import Pose
from typing import Optional, Tuple


def normalize_quaternion(quat: np.ndarray) -> np.ndarray:
    """
    Normalize quaternion to unit length

    Args:
        quat: Quaternion [x, y, z, w]

    Returns:
        Normalized quaternion
    """
    norm = np.linalg.norm(quat)
    if norm < 1e-8:
        return np.array([0.0, 0.0, 0.0, 1.0])
    return quat / norm


def quat_diff_as_rotvec(q_ref: np.ndarray, q_current: np.ndarray) -> np.ndarray:
    """
    Compute rotation difference from q_ref to q_current as rotation vector

    delta_R = q_current * q_ref^-1

    Args:
        q_ref: Reference quaternion [x, y, z, w]
        q_current: Current quaternion [x, y, z, w]

    Returns:
        Rotation vector [rx, ry, rz] (angle-axis)
    """
    # Normalize inputs
    q_ref = normalize_quaternion(q_ref)
    q_current = normalize_quaternion(q_current)

    # Compute delta rotation
    R_ref = R.from_quat(q_ref)
    R_current = R.from_quat(q_current)
    delta_R = R_current * R_ref.inv()

    return delta_R.as_rotvec()


def apply_workspace_limit(position: np.ndarray, workspace: dict) -> Tuple[np.ndarray, bool]:
    """
    Apply workspace limits and clamp position

    Args:
        position: Position [x, y, z]
        workspace: Dict with x_min, x_max, y_min, y_max, z_min, z_max

    Returns:
        (clamped_position, was_clamped)
    """
    clamped = position.copy()
    was_clamped = False

    if clamped[0] < workspace['x_min']:
        clamped[0] = workspace['x_min']
        was_clamped = True
    elif clamped[0] > workspace['x_max']:
        clamped[0] = workspace['x_max']
        was_clamped = True

    if clamped[1] < workspace['y_min']:
        clamped[1] = workspace['y_min']
        was_clamped = True
    elif clamped[1] > workspace['y_max']:
        clamped[1] = workspace['y_max']
        was_clamped = True

    if clamped[2] < workspace['z_min']:
        clamped[2] = workspace['z_min']
        was_clamped = True
    elif clamped[2] > workspace['z_max']:
        clamped[2] = workspace['z_max']
        was_clamped = True

    return clamped, was_clamped


class TeleopMapper:
    """
    Maps XR controller motion to robot end-effector motion using delta/reference mapping

    COORDINATE SYSTEM ASSUMPTIONS:
    - XR frame: Defined by XRoboToolkit SDK
    - Piper base frame: Robot base_link
    - Transform R_map requires real-world calibration

    IMPORTANT: R_map is a configurable parameter that must be calibrated
    with the actual robot setup. Do NOT assume the default is correct.
    """

    def __init__(self, config: dict):
        """
        Initialize teleoperation mapper

        Args:
            config: Configuration dictionary with mapping parameters
        """
        self.config = config

        # Reference poses (set when grip is activated)
        self.xr_reference_pose = None  # XR controller reference [pos, quat]
        self.ee_reference_pose = None  # EE reference [pos, quat]

        # Active flag
        self.active = False

        # Tracking state
        self.tracking_valid = True

        # Get parameters from config
        self.position_scale = config['position_mapping']['scale']
        self.rotation_scale = config['rotation_mapping']['scale']
        self.max_position_delta = config['position_mapping']['max_delta']
        self.max_rotation_delta = config['rotation_mapping']['max_delta']

        # Coordinate transform parameters (REQUIRES CALIBRATION)
        # R_map: Transform from XR frame deltas to Piper base frame deltas
        self.xr_to_piper_translation = np.array(
            config['xr_to_piper_transform']['translation']
        )

        # Build rotation matrix from config
        euler_angles = config['xr_to_piper_transform']['rotation']
        self.R_map = R.from_euler('xyz', euler_angles).as_matrix()

        # Workspace limits
        self.workspace_limit = config['safety']['workspace_limit']
        self.enable_workspace_limit = config['safety']['enable_workspace_limit']

        # Rotation limit
        self.max_rotation_angle = config['rotation_mapping'].get('max_angle', np.pi)

    def set_reference(self, xr_pos: np.ndarray, xr_quat: np.ndarray,
                     ee_pos: np.ndarray, ee_quat: np.ndarray):
        """
        Set reference poses for delta mapping (called when grip is activated)

        This establishes the correspondence between XR controller pose and
        robot EE pose, allowing subsequent movements to be relative.

        Args:
            xr_pos: XR controller position [x, y, z]
            xr_quat: XR controller quaternion [x, y, z, w]
            ee_pos: End-effector position [x, y, z]
            ee_quat: End-effector quaternion [x, y, z, w]
        """
        # Normalize quaternions
        xr_quat = normalize_quaternion(xr_quat)
        ee_quat = normalize_quaternion(ee_quat)

        self.xr_reference_pose = {
            'position': xr_pos.copy(),
            'quaternion': xr_quat
        }
        self.ee_reference_pose = {
            'position': ee_pos.copy(),
            'quaternion': ee_quat
        }
        self.active = True
        self.tracking_valid = True

    def reset_reference(self):
        """Reset reference poses (when grip is released)"""
        self.xr_reference_pose = None
        self.ee_reference_pose = None
        self.active = False

    def is_active(self) -> bool:
        """Check if mapping is active"""
        return self.active

    def set_tracking_valid(self, valid: bool):
        """Set tracking validity status"""
        self.tracking_valid = valid
        if not valid:
            # Invalidate mapping when tracking is lost
            self.reset_reference()

    def compute_target_pose(self, xr_pos: np.ndarray, xr_quat: np.ndarray) -> Optional[Pose]:
        """
        Compute target EE pose from current XR controller pose

        Algorithm:
        1. Compute delta in XR frame: delta_xr = xr_current - xr_ref
        2. Transform to Piper frame: delta_piper = R_map * delta_xr
        3. Apply scale and limits
        4. Compute target: target = ee_ref + scale * delta_piper
        5. Same for rotation using quaternions

        Args:
            xr_pos: Current XR controller position [x, y, z]
            xr_quat: Current XR controller quaternion [x, y, z, w]

        Returns:
            Target EE pose as geometry_msgs/Pose, or None if mapping inactive
        """
        if not self.active or self.xr_reference_pose is None:
            return None

        if not self.tracking_valid:
            return None

        # Normalize input quaternion
        xr_quat = normalize_quaternion(xr_quat)

        # === POSITION MAPPING ===
        # Compute position delta in XR frame
        delta_pos_xr = xr_pos - self.xr_reference_pose['position']

        # Apply coordinate transform: delta_piper = R_map * delta_xr
        delta_pos_piper = self.R_map @ delta_pos_xr

        # Apply position scale
        delta_pos_piper *= self.position_scale

        # Apply position delta limit (per-step safety)
        delta_norm = np.linalg.norm(delta_pos_piper)
        if delta_norm > self.max_position_delta:
            delta_pos_piper = delta_pos_piper / delta_norm * self.max_position_delta

        # Compute target position
        target_pos = self.ee_reference_pose['position'] + delta_pos_piper

        # Apply workspace limits
        if self.enable_workspace_limit:
            target_pos, was_clamped = apply_workspace_limit(target_pos, self.workspace_limit)

        # === ROTATION MAPPING ===
        # Compute rotation delta in XR frame
        delta_rotvec_xr = quat_diff_as_rotvec(
            self.xr_reference_pose['quaternion'],
            xr_quat
        )

        # Apply coordinate transform to rotation delta
        # Transform rotation vector: delta_piper = R_map * delta_xr
        delta_rotvec_piper = self.R_map @ delta_rotvec_xr

        # Apply rotation scale
        delta_rotvec_piper *= self.rotation_scale

        # Apply rotation delta limit (per-step safety)
        delta_angle = np.linalg.norm(delta_rotvec_piper)
        if delta_angle > self.max_rotation_delta:
            delta_rotvec_piper = delta_rotvec_piper / delta_angle * self.max_rotation_delta

        # Apply absolute rotation limit
        if delta_angle > self.max_rotation_angle:
            delta_rotvec_piper = delta_rotvec_piper / delta_angle * self.max_rotation_angle

        # Convert back to rotation and apply to reference
        delta_R_piper = R.from_rotvec(delta_rotvec_piper)
        ee_ref_rot = R.from_quat(self.ee_reference_pose['quaternion'])
        target_rot = delta_R_piper * ee_ref_rot
        target_quat = normalize_quaternion(target_rot.as_quat())

        # Create Pose message
        target_pose = Pose()
        target_pose.position.x = float(target_pos[0])
        target_pose.position.y = float(target_pos[1])
        target_pose.position.z = float(target_pos[2])
        target_pose.orientation.x = float(target_quat[0])
        target_pose.orientation.y = float(target_quat[1])
        target_pose.orientation.z = float(target_quat[2])
        target_pose.orientation.w = float(target_quat[3])

        return target_pose

    def get_delta_info(self, xr_pos: np.ndarray, xr_quat: np.ndarray) -> Optional[dict]:
        """
        Get debug info about current delta mapping

        Returns:
            Dict with delta_pos_xr, delta_pos_piper, delta_angle_xr, delta_angle_piper
        """
        if not self.active or self.xr_reference_pose is None:
            return None

        xr_quat = normalize_quaternion(xr_quat)

        delta_pos_xr = xr_pos - self.xr_reference_pose['position']
        delta_pos_piper = self.R_map @ delta_pos_xr * self.position_scale

        delta_rotvec_xr = quat_diff_as_rotvec(
            self.xr_reference_pose['quaternion'],
            xr_quat
        )
        delta_rotvec_piper = self.R_map @ delta_rotvec_xr * self.rotation_scale

        return {
            'delta_pos_xr': delta_pos_xr,
            'delta_pos_piper': delta_pos_piper,
            'delta_angle_xr': np.linalg.norm(delta_rotvec_xr),
            'delta_angle_piper': np.linalg.norm(delta_rotvec_piper),
        }
