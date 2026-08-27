#!/usr/bin/env python3
"""Position-priority, soft-orientation IK for Piper XR teleoperation.

All orientation math stays in SO(3). The module never converts an end-effector
target to Piper RX/RY/RZ Euler angles and contains no CAN or hardware I/O.
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Sequence

import numpy as np
import pinocchio as pin
from scipy.spatial.transform import Rotation


PIPER_JOINT_NAMES = tuple(f"joint{i}" for i in range(1, 7))
SDK_UNITS_PER_DEGREE = 1000.0


@dataclass(frozen=True)
class IKResult:
    """One incremental IK result."""

    success: bool
    joint_angles_rad: np.ndarray
    position_error_m: float
    orientation_error_rad: float
    iterations: int
    min_singular_value: float
    clipped_to_joint_limits: bool
    reason: str


def radians_to_sdk_mdeg(joint_angles_rad: Sequence[float]) -> np.ndarray:
    """Convert six radians to Piper SDK 0.001-degree units."""

    values = np.asarray(joint_angles_rad, dtype=np.float64)
    if values.shape != (6,) or not np.all(np.isfinite(values)):
        raise ValueError("joint_angles_rad must contain six finite values")
    return np.rint(np.rad2deg(values) * SDK_UNITS_PER_DEGREE).astype(np.int64)


def sdk_mdeg_to_radians(joint_angles_mdeg: Sequence[float]) -> np.ndarray:
    """Convert Piper SDK 0.001-degree feedback to radians."""

    values = np.asarray(joint_angles_mdeg, dtype=np.float64)
    if values.shape != (6,) or not np.all(np.isfinite(values)):
        raise ValueError("joint_angles_mdeg must contain six finite values")
    return np.deg2rad(values / SDK_UNITS_PER_DEGREE)


class PiperPinocchioIK:
    """Incremental Piper IK with previous-solution regularization.

    The official Piper URDF contains two additional gripper joints. They are
    locked while building the reduced model so only joint1 through joint6 are
    commanded. The seed must be the measured joints on clutch engagement or
    the previous accepted IK solution on subsequent frames.
    """

    def __init__(
        self,
        urdf_path: str | Path,
        *,
        end_effector_frame: str = "link6",
        position_weight: float = 20.0,
        orientation_weight: float = 0.2,
        posture_weight: float = 1.0e-2,
        damping: float = 1.0e-6,
        max_internal_joint_step_rad: float = 0.08,
        position_tolerance_m: float = 5.0e-4,
        orientation_tolerance_rad: float = np.deg2rad(0.5),
        max_iterations: int = 80,
    ) -> None:
        self.urdf_path = Path(urdf_path).expanduser().resolve()
        if not self.urdf_path.is_file():
            raise FileNotFoundError(self.urdf_path)
        if position_weight <= 0.0 or orientation_weight < 0.0:
            raise ValueError("position weight must be positive and orientation weight non-negative")
        if posture_weight < 0.0 or damping <= 0.0:
            raise ValueError("posture weight must be non-negative and damping positive")
        if max_internal_joint_step_rad <= 0.0 or max_iterations <= 0:
            raise ValueError("step and iteration limits must be positive")

        full_model = pin.buildModelFromUrdf(str(self.urdf_path))
        available = {str(name) for name in full_model.names}
        missing = set(PIPER_JOINT_NAMES).difference(available)
        if missing:
            raise ValueError(f"URDF is missing Piper joints: {sorted(missing)}")
        locked_joint_ids = [
            joint_id
            for joint_id, name in enumerate(full_model.names)
            if joint_id > 0 and str(name) not in PIPER_JOINT_NAMES
        ]
        self.model = (
            pin.buildReducedModel(full_model, locked_joint_ids, pin.neutral(full_model))
            if locked_joint_ids
            else full_model
        )
        if self.model.nq != 6 or self.model.nv != 6:
            raise ValueError(
                f"expected a six-DoF model, got nq={self.model.nq}, nv={self.model.nv}"
            )
        if not self.model.existFrame(end_effector_frame):
            raise ValueError(f"URDF has no frame named {end_effector_frame!r}")

        self.data = self.model.createData()
        self.end_effector_frame = end_effector_frame
        self.frame_id = self.model.getFrameId(end_effector_frame)
        self.position_weight = float(position_weight)
        self.orientation_weight = float(orientation_weight)
        self.posture_weight = float(posture_weight)
        self.damping = float(damping)
        self.max_internal_joint_step_rad = float(max_internal_joint_step_rad)
        self.position_tolerance_m = float(position_tolerance_m)
        self.orientation_tolerance_rad = float(orientation_tolerance_rad)
        self.max_iterations = int(max_iterations)
        self.lower_limits = np.asarray(self.model.lowerPositionLimit, dtype=np.float64)
        self.upper_limits = np.asarray(self.model.upperPositionLimit, dtype=np.float64)

    @staticmethod
    def _validate_vector(values: Sequence[float], size: int, label: str) -> np.ndarray:
        array = np.asarray(values, dtype=np.float64)
        if array.shape != (size,) or not np.all(np.isfinite(array)):
            raise ValueError(f"{label} must contain {size} finite values")
        return array.copy()

    @classmethod
    def _normalize_quaternion_xyzw(cls, quaternion_xyzw: Sequence[float]) -> np.ndarray:
        quaternion = cls._validate_vector(quaternion_xyzw, 4, "quaternion")
        norm = float(np.linalg.norm(quaternion))
        if norm < 1.0e-9:
            raise ValueError("quaternion norm is zero")
        return quaternion / norm

    def forward_transform(self, joint_angles_rad: Sequence[float]) -> pin.SE3:
        """Return base-to-link6 SE(3) for six joint angles."""

        q = self._validate_vector(joint_angles_rad, 6, "joint angles")
        pin.forwardKinematics(self.model, self.data, q)
        pin.updateFramePlacements(self.model, self.data)
        return self.data.oMf[self.frame_id].copy()

    def forward_pose(self, joint_angles_rad: Sequence[float]) -> tuple[np.ndarray, np.ndarray]:
        """Return position in metres and quaternion in xyzw order."""

        transform = self.forward_transform(joint_angles_rad)
        quaternion = Rotation.from_matrix(transform.rotation).as_quat()
        return transform.translation.copy(), quaternion

    @staticmethod
    def _pose_errors(current: pin.SE3, target: pin.SE3) -> tuple[float, float]:
        position_error = float(np.linalg.norm(current.translation - target.translation))
        rotation_delta = current.rotation.T @ target.rotation
        orientation_error = float(
            np.linalg.norm(Rotation.from_matrix(rotation_delta).as_rotvec())
        )
        return position_error, orientation_error

    def solve(
        self,
        target_position_m: Sequence[float],
        target_quaternion_xyzw: Sequence[float],
        seed_joint_angles_rad: Sequence[float],
        posture_reference_joint_angles_rad: Sequence[float] | None = None,
    ) -> IKResult:
        """Solve one pose without random restarts.

        Position is the acceptance criterion. Orientation is a soft objective,
        and its residual is reported. On failure the caller must hold the
        previous accepted joints rather than send this result to hardware.
        """

        target_position = self._validate_vector(target_position_m, 3, "target position")
        target_quaternion = self._normalize_quaternion_xyzw(target_quaternion_xyzw)
        seed = self._validate_vector(seed_joint_angles_rad, 6, "seed joints")
        posture_reference = (
            seed
            if posture_reference_joint_angles_rad is None
            else self._validate_vector(
                posture_reference_joint_angles_rad,
                6,
                "posture reference joints",
            )
        )
        q = np.clip(seed, self.lower_limits, self.upper_limits)
        clipped = not np.array_equal(q, seed)
        target = pin.SE3(Rotation.from_quat(target_quaternion).as_matrix(), target_position)
        task_weights = np.array(
            [
                self.position_weight,
                self.position_weight,
                self.position_weight,
                self.orientation_weight,
                self.orientation_weight,
                self.orientation_weight,
            ],
            dtype=np.float64,
        )
        min_singular_value = 0.0
        reason = "iteration limit"

        for iteration in range(1, self.max_iterations + 1):
            current = self.forward_transform(q)
            position_error, orientation_error = self._pose_errors(current, target)
            orientation_done = (
                self.orientation_weight == 0.0
                or orientation_error <= self.orientation_tolerance_rad
            )
            if position_error <= self.position_tolerance_m and orientation_done:
                reason = "pose tolerance reached"
                break

            current_to_target = current.actInv(target)
            error = pin.log6(current_to_target).vector
            jacobian = pin.computeFrameJacobian(
                self.model,
                self.data,
                q,
                self.frame_id,
                pin.ReferenceFrame.LOCAL,
            )
            jacobian = -pin.Jlog6(current_to_target.inverse()) @ jacobian
            singular_values = np.linalg.svd(jacobian, compute_uv=False)
            min_singular_value = float(singular_values[-1])

            rows = [task_weights[:, None] * jacobian]
            residuals = [task_weights * error]
            if self.posture_weight > 0.0:
                posture_scale = np.sqrt(self.posture_weight)
                rows.append(posture_scale * np.eye(6))
                residuals.append(posture_scale * (q - posture_reference))
            damping_scale = np.sqrt(self.damping)
            rows.append(damping_scale * np.eye(6))
            residuals.append(np.zeros(6, dtype=np.float64))
            augmented_jacobian = np.vstack(rows)
            augmented_error = np.concatenate(residuals)
            delta_q = np.linalg.lstsq(
                augmented_jacobian, -augmented_error, rcond=None
            )[0]
            max_component = float(np.max(np.abs(delta_q)))
            if max_component > self.max_internal_joint_step_rad:
                delta_q *= self.max_internal_joint_step_rad / max_component
            if float(np.linalg.norm(delta_q)) < 1.0e-9:
                reason = "solver stalled"
                break
            candidate = pin.integrate(self.model, q, delta_q)
            limited = np.clip(candidate, self.lower_limits, self.upper_limits)
            clipped = clipped or not np.array_equal(candidate, limited)
            q = limited
        else:
            iteration = self.max_iterations

        current = self.forward_transform(q)
        position_error, orientation_error = self._pose_errors(current, target)
        success = bool(
            np.all(np.isfinite(q)) and position_error <= self.position_tolerance_m
        )
        if success and reason != "pose tolerance reached":
            reason = "position tolerance reached; orientation kept soft"
        elif not success and clipped:
            reason = "target not reached within joint limits"
        return IKResult(
            success=success,
            joint_angles_rad=q.copy(),
            position_error_m=position_error,
            orientation_error_rad=orientation_error,
            iterations=iteration,
            min_singular_value=min_singular_value,
            clipped_to_joint_limits=clipped,
            reason=reason,
        )
