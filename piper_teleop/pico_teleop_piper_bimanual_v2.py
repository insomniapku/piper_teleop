#!/usr/bin/env python3
"""Single-process bimanual Pico teleoperation for two Piper arms.

One XR SDK client reads both controllers.  Each hand owns an independent
clutch, reference pose, IK solver, joint limiter, CAN interface, and gripper.
Dry-run is the default and never imports the XR or Piper SDKs.
"""

from __future__ import annotations

import argparse
import math
import sys
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Callable, Optional

import numpy as np
from scipy.spatial.transform import Rotation

from pico_teleop_piper_ik_v2 import (
    GRIPPER_OPEN_UNITS,
    R_XR_TO_PIPER,
    SAFE_TEST_RADIUS_M,
    XR_STALE_SECONDS,
    XR_VALID_FRAMES_REQUIRED,
    PiperJointHardware,
    binary_gripper_trigger,
    clamp_norm,
    installation_rotation,
    limit_joint_step,
    limit_target_speed,
    normalize_quaternion,
    relative_target_rotation,
    run_dry_run,
    update_grip_state,
    valid_xr_position,
    workspace_bounds_for_reference,
)
from piper_ik_v2 import PiperPinocchioIK
from trajectory_recorder import TrajectoryRecorder


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Pico-to-dual-Piper bimanual quaternion IK (dry-run by default)."
    )
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument("--dry-run", action="store_true")
    mode.add_argument("--hardware", action="store_true")
    parser.add_argument("--urdf", default="assets/piper_description.urdf")
    parser.add_argument("--left-can-name", default="can0")
    parser.add_argument("--right-can-name", default="can1")
    parser.add_argument("--control-rate", type=float, default=50.0)
    parser.add_argument("--position-scale", type=float, default=0.8)
    parser.add_argument("--rotation-scale", type=float, default=1.0)
    parser.add_argument("--max-orientation-delta-deg", type=float, default=180.0)
    parser.add_argument("--max-speed", type=float, default=0.08)
    parser.add_argument("--max-joint-step-deg", type=float, default=5.0)
    parser.add_argument("--ik-position-tolerance-mm", type=float, default=2.0)
    parser.add_argument("--no-joint-step-limit", action="store_true")
    parser.add_argument("--speed-percent", type=int, default=100)
    parser.add_argument("--left-yaw-deg", type=int, choices=(0, 90, -90, 180), default=0)
    parser.add_argument("--right-yaw-deg", type=int, choices=(0, 90, -90, 180), default=0)
    parser.add_argument("--no-speed-limit", action="store_true")
    parser.add_argument("--no-workspace-limit", action="store_true")
    parser.add_argument("--position-only", action="store_true")
    parser.add_argument("--safe-test", action="store_true")
    parser.add_argument("--no-gripper", action="store_true")
    parser.add_argument("--binary-gripper", action="store_true")
    parser.add_argument("--enable-recording", action="store_true", help="Enable trajectory recording with B button")
    parser.add_argument("--recording-dir", default="recordings", help="Directory for recordings")
    parser.add_argument("--camera-fps", type=int, default=30, help="Camera recording FPS")
    args = parser.parse_args()

    if args.left_can_name == args.right_can_name:
        parser.error("left and right CAN interfaces must be different")
    if args.control_rate <= 0.0 or args.position_scale <= 0.0:
        parser.error("control rate and position scale must be positive")
    if args.rotation_scale <= 0.0 or not 0.0 < args.max_orientation_delta_deg <= 180.0:
        parser.error("invalid orientation scale/range")
    if (
        args.max_speed <= 0.0
        or args.max_joint_step_deg <= 0.0
        or args.ik_position_tolerance_mm <= 0.0
    ):
        parser.error("speed, joint-step, and IK tolerance must be positive")
    if not 1 <= args.speed_percent <= 100:
        parser.error("--speed-percent must be in [1, 100]")
    if args.safe_test and (args.no_speed_limit or args.no_joint_step_limit):
        parser.error("--safe-test requires speed and joint-step limits")
    if args.safe_test:
        args.position_scale = 0.25
        args.rotation_scale = min(args.rotation_scale, 0.5)
        args.max_orientation_delta_deg = min(args.max_orientation_delta_deg, 10.0)
        args.max_speed = 0.02
        args.max_joint_step_deg = min(args.max_joint_step_deg, 0.25)
        args.speed_percent = min(args.speed_percent, 10)
        args.no_gripper = True

    args.dry_run = not args.hardware
    urdf_path = Path(args.urdf).expanduser()
    if not urdf_path.is_absolute():
        urdf_path = Path(__file__).resolve().parent / urdf_path
    args.urdf = urdf_path.resolve()
    return args


@dataclass
class ArmChannel:
    label: str
    hardware: PiperJointHardware
    solver: PiperPinocchioIK
    r_final: np.ndarray
    get_pose: Callable[[], object]
    get_grip: Callable[[], float]
    get_trigger: Callable[[], float]
    args: argparse.Namespace

    grip_active: bool = False
    activation_pending: bool = False
    require_regrip: bool = True
    valid_frames: int = 0
    invalid_announced: bool = False
    xr_reference_robot: Optional[np.ndarray] = None
    xr_reference_quaternion: Optional[np.ndarray] = None
    ee_reference_position: Optional[np.ndarray] = None
    ee_reference_rotation: Optional[np.ndarray] = None
    posture_reference: Optional[np.ndarray] = None
    previous_joints: Optional[np.ndarray] = None
    previous_position: Optional[np.ndarray] = None
    previous_trigger: Optional[float] = None
    last_gripper_send: float = 0.0
    failure_count: int = 0
    joint_limit_announced: bool = False

    def log(self, message: str) -> None:
        print(f"[{self.label}] {message}")

    def clear_clutch(self) -> None:
        self.xr_reference_robot = None
        self.xr_reference_quaternion = None
        self.ee_reference_position = None
        self.ee_reference_rotation = None
        self.posture_reference = None
        self.previous_joints = None
        self.previous_position = None
        self.joint_limit_announced = False

    def update_buttons(self, now: float) -> None:
        grip = float(self.get_grip())
        trigger = float(self.get_trigger())
        self.grip_active, rising, falling = update_grip_state(self.grip_active, grip)
        if rising and not self.require_regrip:
            self.activation_pending = True
        if falling:
            self.activation_pending = False
            self.clear_clutch()
            self.log("[clutch] released; joint target updates stopped")

        if self.grip_active and not self.args.no_gripper:
            trigger = float(np.clip(trigger, 0.0, 1.0))
            if self.args.binary_gripper:
                trigger = binary_gripper_trigger(self.previous_trigger, trigger)
            changed = (
                self.previous_trigger is None
                or abs(trigger - self.previous_trigger) > 0.01
            )
            if changed or now - self.last_gripper_send >= 0.20:
                self.hardware.send_gripper(trigger)
                self.previous_trigger = trigger
                self.last_gripper_send = now

    def block_for_stale(self) -> None:
        self.require_regrip = True
        self.activation_pending = False
        self.valid_frames = 0
        self.clear_clutch()

    def process_pose(self, dt: float) -> None:
        pose = self.get_pose()
        xr_position = np.asarray(pose[:3], dtype=np.float64)
        xr_quaternion = normalize_quaternion(pose[3:7])
        if not valid_xr_position(xr_position) or xr_quaternion is None:
            if not self.invalid_announced:
                self.log("[XR ERROR] invalid pose; blocked until 10 valid frames")
            self.invalid_announced = True
            self.valid_frames = 0
            self.require_regrip = True
            self.activation_pending = False
            self.clear_clutch()
            return

        self.valid_frames = min(self.valid_frames + 1, XR_VALID_FRAMES_REQUIRED)
        xr_robot = self.r_final @ xr_position
        if self.valid_frames >= XR_VALID_FRAMES_REQUIRED:
            if self.require_regrip or self.invalid_announced:
                self.log("[XR READY] 10 valid frames; Grip may activate")
            self.require_regrip = False
            self.invalid_announced = False
            if self.grip_active and self.xr_reference_robot is None:
                self.activation_pending = True

        if self.activation_pending and not self.require_regrip:
            self.activation_pending = False
            measured = self.hardware.read_joints()
            if measured is None:
                self.log("[PIPER ERROR] no complete feedback; release and re-clutch")
                self.require_regrip = True
                self.clear_clutch()
                return
            transform = self.solver.forward_transform(measured)
            self.xr_reference_robot = xr_robot.copy()
            self.xr_reference_quaternion = xr_quaternion.copy()
            self.ee_reference_position = transform.translation.copy()
            self.ee_reference_rotation = transform.rotation.copy()
            self.posture_reference = measured.copy()
            self.previous_joints = measured.copy()
            self.previous_position = self.ee_reference_position.copy()
            self.failure_count = 0
            self.joint_limit_announced = False
            self.hardware.confirm_joint_mode()
            self.hardware.send_joints(measured)
            self.log(
                "[clutch] activated; first target equals measured joints "
                f"{np.array2string(np.rad2deg(measured), precision=2)}deg"
            )
            return

        ready = all(
            value is not None
            for value in (
                self.xr_reference_robot,
                self.xr_reference_quaternion,
                self.ee_reference_position,
                self.ee_reference_rotation,
                self.posture_reference,
                self.previous_joints,
                self.previous_position,
            )
        )
        if not (self.grip_active and not self.require_regrip and ready):
            return

        relative = self.args.position_scale * (xr_robot - self.xr_reference_robot)
        if self.args.safe_test:
            relative = clamp_norm(relative, SAFE_TEST_RADIUS_M)
        raw_position = self.ee_reference_position + relative
        if not self.args.no_workspace_limit:
            lower, upper = workspace_bounds_for_reference(self.ee_reference_position)
            raw_position = np.clip(raw_position, lower, upper)
        target_position = (
            raw_position
            if self.args.no_speed_limit
            else limit_target_speed(
                self.previous_position, raw_position, self.args.max_speed, dt
            )
        )
        target_rotation = relative_target_rotation(
            self.xr_reference_quaternion,
            xr_quaternion,
            self.ee_reference_rotation,
            self.r_final,
            self.args.rotation_scale,
            self.args.max_orientation_delta_deg,
        )
        result = self.solver.solve(
            target_position,
            Rotation.from_matrix(target_rotation).as_quat(),
            self.previous_joints,
            posture_reference_joint_angles_rad=self.posture_reference,
        )
        if not result.success:
            self.failure_count += 1
            if self.failure_count == 1 or self.failure_count % 25 == 0:
                self.log(
                    f"[IK HOLD] {result.reason}; position error="
                    f"{result.position_error_m * 1000.0:.2f}mm"
                )
            return

        if self.args.no_joint_step_limit:
            command_joints = result.joint_angles_rad
            was_limited = False
        else:
            command_joints, was_limited = limit_joint_step(
                self.previous_joints,
                result.joint_angles_rad,
                math.radians(self.args.max_joint_step_deg),
            )
        if was_limited and not self.joint_limit_announced:
            requested_step = float(
                np.max(np.abs(result.joint_angles_rad - self.previous_joints))
            )
            self.log(
                f"[JOINT LIMIT] requested {math.degrees(requested_step):.3f}deg; "
                f"limited to {self.args.max_joint_step_deg:g}deg/frame"
            )
            self.joint_limit_announced = True
        elif not was_limited:
            self.joint_limit_announced = False
        self.hardware.send_joints(command_joints)
        self.previous_joints = command_joints
        self.previous_position = self.solver.forward_transform(
            command_joints
        ).translation.copy()
        self.failure_count = 0


def run_bimanual_dry_run(args: argparse.Namespace) -> int:
    print("BIMANUAL V2 DRY-RUN: validating left mapping")
    left = run_dry_run(
        args, installation_rotation(args.left_yaw_deg) @ R_XR_TO_PIPER
    )
    print("BIMANUAL V2 DRY-RUN: validating right mapping")
    right = run_dry_run(
        args, installation_rotation(args.right_yaw_deg) @ R_XR_TO_PIPER
    )
    return 0 if left == 0 and right == 0 else 1


def run_hardware(args: argparse.Namespace) -> int:
    print("!" * 76)
    print(
        "BIMANUAL V2 HARDWARE MODE: enables "
        f"{args.left_can_name}/{args.right_can_name} and sends live targets."
    )
    print(
        f"Left Grip controls {args.left_can_name}; right Grip controls "
        f"{args.right_can_name} independently."
    )
    print("No inter-arm collision avoidance is implemented.")
    print("It will NOT home, reset, or disable either arm on exit.")
    if args.enable_recording:
        print("Recording enabled: Press B button to start/stop trajectory recording.")
    print("!" * 76)
    try:
        confirmation = input("Type BIMANUAL exactly to continue: ").strip()
    except EOFError:
        confirmation = ""
    if confirmation != "BIMANUAL":
        print("Cancelled; no XR/Piper SDK imported and no CAN interface opened.")
        return 2

    try:
        import xrobotoolkit_sdk as xrt
    except ImportError as exc:
        raise RuntimeError("XRoboToolkit SDK is unavailable") from exc

    left_solver = PiperPinocchioIK(
        args.urdf,
        orientation_weight=0.0 if args.position_only else 0.2,
        position_tolerance_m=args.ik_position_tolerance_mm / 1000.0,
    )
    right_solver = PiperPinocchioIK(
        args.urdf,
        orientation_weight=0.0 if args.position_only else 0.2,
        position_tolerance_m=args.ik_position_tolerance_mm / 1000.0,
    )

    # Initialize recorder if enabled
    recorder = None
    if args.enable_recording:
        recorder = TrajectoryRecorder(
            camera_indices=(0, 2, 8),  # Three cameras: video0, video2, video8
            fps=args.camera_fps,
            recording_dir=args.recording_dir,
        )

    xr_initialized = False
    previous_b_button = False

    try:
        left_hardware = PiperJointHardware(args.left_can_name, args.speed_percent)
        right_hardware = PiperJointHardware(args.right_can_name, args.speed_percent)
        if not args.no_gripper:
            left_hardware.send_gripper(0.0)
            right_hardware.send_gripper(0.0)
            print(f"[both] startup full-open gripper command sent ({GRIPPER_OPEN_UNITS} units)")

        xrt.init()
        xr_initialized = True
        left = ArmChannel(
            f"LEFT/{args.left_can_name}",
            left_hardware,
            left_solver,
            installation_rotation(args.left_yaw_deg) @ R_XR_TO_PIPER,
            xrt.get_left_controller_pose,
            xrt.get_left_grip,
            xrt.get_left_trigger,
            args,
        )
        right = ArmChannel(
            f"RIGHT/{args.right_can_name}",
            right_hardware,
            right_solver,
            installation_rotation(args.right_yaw_deg) @ R_XR_TO_PIPER,
            xrt.get_right_controller_pose,
            xrt.get_right_grip,
            xrt.get_right_trigger,
            args,
        )
        arms = (left, right)
        print(
            f"Bimanual V2 ready: {args.control_rate:g}Hz, scale={args.position_scale:g}, "
            f"rotation_scale={args.rotation_scale:g}, max_joint_step="
            f"{'unlimited' if args.no_joint_step_limit else f'{args.max_joint_step_deg:g}deg/frame'}, "
            f"IK tolerance={args.ik_position_tolerance_mm:g}mm, speed={args.speed_percent}%, "
            f"left={args.left_can_name}/yaw{args.left_yaw_deg}, "
            f"right={args.right_can_name}/yaw{args.right_yaw_deg}"
        )

        period = 1.0 / args.control_rate
        next_tick = last_cycle = last_new_xr_time = time.monotonic()
        last_timestamp = None
        stale_announced = False
        while True:
            cycle_start = time.monotonic()
            next_tick += period
            dt = float(np.clip(cycle_start - last_cycle, 0.005, 0.05))
            last_cycle = cycle_start

            for arm in arms:
                arm.update_buttons(cycle_start)

            # Handle B button for recording
            if recorder:
                current_b_button = xrt.get_B_button()
                if current_b_button and not previous_b_button:
                    # Button pressed (rising edge)
                    if recorder.is_recording:
                        recorder.stop_recording()
                        print("[RECORDER] Recording stopped (B button pressed)")
                    else:
                        if recorder.start_recording():
                            print("[RECORDER] Recording started (B button pressed)")
                previous_b_button = current_b_button

            timestamp = xrt.get_time_stamp_ns()
            if timestamp != last_timestamp:
                last_timestamp = timestamp
                last_new_xr_time = cycle_start
                stale_announced = False
                for arm in arms:
                    arm.process_pose(dt)

            # Record joint angles if recording is active
            if recorder and recorder.is_recording:
                left_joints = left.previous_joints
                right_joints = right.previous_joints
                if left_joints is not None and right_joints is not None:
                    recorder.record_frame(left_joints, right_joints)

            if cycle_start - last_new_xr_time > XR_STALE_SECONDS:
                if not stale_announced:
                    print("[BOTH] [XR STALE] >0.2s; both arms stopped")
                    stale_announced = True
                for arm in arms:
                    arm.block_for_stale()

            remaining = next_tick - time.monotonic()
            if remaining > 0.0:
                time.sleep(remaining)
            else:
                next_tick = time.monotonic()
    except KeyboardInterrupt:
        print("\nStopped both target streams. Arms remain enabled; no reset/disable sent.")
        return 0
    finally:
        if recorder:
            recorder.cleanup()
        if xr_initialized:
            xrt.close()


def main() -> int:
    args = parse_args()
    return run_hardware(args) if args.hardware else run_bimanual_dry_run(args)


if __name__ == "__main__":
    sys.exit(main())
