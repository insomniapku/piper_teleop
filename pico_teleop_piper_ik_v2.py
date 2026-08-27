#!/usr/bin/env python3
"""Pico pose teleoperation for Piper using continuous six-joint IK.

The default mode is an offline dry-run. XR and CAN SDKs are imported only
after ``--hardware`` and an interactive ``ARM`` confirmation.
"""

from __future__ import annotations

import argparse
import math
import sys
import time
from pathlib import Path
from typing import Optional, Tuple

import numpy as np
from scipy.spatial.transform import Rotation

from piper_ik_v2 import PiperPinocchioIK, radians_to_sdk_mdeg, sdk_mdeg_to_radians


R_XR_TO_PIPER = np.array(
    [[0.0, 0.0, -1.0], [-1.0, 0.0, 0.0], [0.0, 1.0, 0.0]], dtype=np.float64
)
WORKSPACE_MIN_M = np.array([0.10, -0.20, 0.10], dtype=np.float64)
WORKSPACE_MAX_M = np.array([0.35, 0.20, 0.35], dtype=np.float64)
GRIP_ON_THRESHOLD = 0.80
GRIP_OFF_THRESHOLD = 0.60
XR_STALE_SECONDS = 0.20
XR_VALID_FRAMES_REQUIRED = 10
XR_POSITION_EPS_M = 1.0e-6
XR_QUATERNION_EPS = 1.0e-8
SAFE_TEST_RADIUS_M = 0.02
GRIPPER_OPEN_UNITS = 100_000
GRIPPER_BINARY_CLOSE_THRESHOLD = 0.60
GRIPPER_BINARY_OPEN_THRESHOLD = 0.40


def installation_rotation(yaw_deg: int) -> np.ndarray:
    yaw = math.radians(yaw_deg)
    return np.array(
        [[math.cos(yaw), -math.sin(yaw), 0.0],
         [math.sin(yaw), math.cos(yaw), 0.0],
         [0.0, 0.0, 1.0]], dtype=np.float64
    )


def normalize_quaternion(value: np.ndarray) -> Optional[np.ndarray]:
    value = np.asarray(value, dtype=np.float64)
    if value.shape != (4,) or not np.all(np.isfinite(value)):
        return None
    norm = float(np.linalg.norm(value))
    return None if norm <= XR_QUATERNION_EPS else value / norm


def valid_xr_position(value: np.ndarray) -> bool:
    value = np.asarray(value, dtype=np.float64)
    return bool(
        value.shape == (3,) and np.all(np.isfinite(value))
        and np.linalg.norm(value) > XR_POSITION_EPS_M
    )


def update_grip_state(was_active: bool, grip: float) -> Tuple[bool, bool, bool]:
    active = was_active
    if not active and grip >= GRIP_ON_THRESHOLD:
        active = True
    elif active and grip <= GRIP_OFF_THRESHOLD:
        active = False
    return active, active and not was_active, was_active and not active


def binary_gripper_trigger(previous: Optional[float], raw: float) -> float:
    raw = float(np.clip(raw, 0.0, 1.0))
    if raw >= GRIPPER_BINARY_CLOSE_THRESHOLD:
        return 1.0
    if raw <= GRIPPER_BINARY_OPEN_THRESHOLD:
        return 0.0
    return 0.0 if previous is None else float(previous >= 0.5)


def clamp_norm(value: np.ndarray, maximum: float) -> np.ndarray:
    norm = float(np.linalg.norm(value))
    return value if norm <= maximum or norm == 0.0 else value * (maximum / norm)


def limit_target_speed(
    previous: np.ndarray, raw_target: np.ndarray, maximum_speed_mps: float, dt: float
) -> np.ndarray:
    return previous + clamp_norm(raw_target - previous, maximum_speed_mps * dt)


def limit_joint_step(
    previous: np.ndarray, candidate: np.ndarray, maximum_step_rad: float
) -> Tuple[np.ndarray, bool]:
    """Limit every joint component while preserving a continuous command."""
    delta = candidate - previous
    limited_delta = np.clip(delta, -maximum_step_rad, maximum_step_rad)
    limited = not np.array_equal(delta, limited_delta)
    return previous + limited_delta, limited


def workspace_bounds_for_reference(reference: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
    return np.minimum(WORKSPACE_MIN_M, reference), np.maximum(WORKSPACE_MAX_M, reference)


def relative_target_rotation(
    xr_reference_quaternion: np.ndarray,
    xr_current_quaternion: np.ndarray,
    ee_reference_rotation: np.ndarray,
    r_final: np.ndarray,
    rotation_scale: float,
    maximum_delta_deg: float,
) -> np.ndarray:
    """Map relative controller SO(3) motion into a Piper target rotation."""
    reference = normalize_quaternion(xr_reference_quaternion)
    current = normalize_quaternion(xr_current_quaternion)
    if reference is None or current is None:
        raise ValueError("invalid XR quaternion")
    delta_xr = Rotation.from_quat(current) * Rotation.from_quat(reference).inv()
    delta_rotvec = r_final @ delta_xr.as_rotvec() * rotation_scale
    delta_rotvec = clamp_norm(delta_rotvec, math.radians(maximum_delta_deg))
    return Rotation.from_rotvec(delta_rotvec).as_matrix() @ ee_reference_rotation


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Pico-to-Piper position-priority quaternion IK (dry-run by default)."
    )
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument("--dry-run", action="store_true")
    mode.add_argument("--hardware", action="store_true")
    parser.add_argument("--urdf", default="assets/piper_description.urdf")
    parser.add_argument("--can-name", default="can0")
    parser.add_argument("--controller-hand", choices=("left", "right"), default="left")
    parser.add_argument("--control-rate", type=float, default=50.0)
    parser.add_argument("--position-scale", type=float, default=0.8)
    parser.add_argument("--rotation-scale", type=float, default=1.0)
    parser.add_argument("--max-orientation-delta-deg", type=float, default=45.0)
    parser.add_argument("--max-speed", type=float, default=0.08)
    parser.add_argument("--max-joint-step-deg", type=float, default=2.0)
    parser.add_argument(
        "--no-joint-step-limit",
        action="store_true",
        help="send each accepted IK solution without the extra per-frame joint limiter",
    )
    parser.add_argument("--speed-percent", type=int, default=40)
    parser.add_argument("--yaw-deg", type=int, choices=(0, 90, -90, 180), default=0)
    parser.add_argument("--no-speed-limit", action="store_true")
    parser.add_argument("--no-workspace-limit", action="store_true")
    parser.add_argument("--position-only", action="store_true")
    parser.add_argument("--safe-test", action="store_true")
    parser.add_argument("--no-gripper", action="store_true")
    parser.add_argument("--binary-gripper", action="store_true")
    args = parser.parse_args()

    if args.control_rate <= 0.0 or args.position_scale <= 0.0:
        parser.error("control rate and position scale must be positive")
    if args.rotation_scale <= 0.0 or not 0.0 < args.max_orientation_delta_deg <= 180.0:
        parser.error("invalid orientation scale/range")
    if args.max_speed <= 0.0 or args.max_joint_step_deg <= 0.0:
        parser.error("speed and joint-step limits must be positive")
    if not 1 <= args.speed_percent <= 100:
        parser.error("--speed-percent must be in [1, 100]")
    if args.safe_test and args.no_speed_limit:
        parser.error("--safe-test cannot be combined with --no-speed-limit")
    if args.safe_test and args.no_joint_step_limit:
        parser.error("--safe-test cannot be combined with --no-joint-step-limit")
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


class PiperJointHardware:
    """Narrow SDK wrapper, constructed only after interactive confirmation."""

    def __init__(self, can_name: str, speed_percent: int):
        try:
            from piper_sdk import C_PiperInterface_V2
        except ImportError as exc:
            raise RuntimeError("Piper SDK is unavailable") from exc
        self.speed_percent = speed_percent
        self.piper = C_PiperInterface_V2(can_name=can_name, judge_flag=False)
        self.piper.SetSDKGripperRangeParam(0.0, GRIPPER_OPEN_UNITS / 1_000_000.0)
        self.piper.ConnectPort()
        time.sleep(0.2)
        for _ in range(100):
            if self.piper.EnablePiper():
                break
            time.sleep(0.01)
        else:
            raise RuntimeError("Piper enable failed; no joint target was sent")

    def confirm_joint_mode(self) -> None:
        self.piper.MotionCtrl_2(0x01, 0x01, self.speed_percent, 0x00)

    def read_joints(self) -> Optional[np.ndarray]:
        try:
            message = self.piper.GetArmJointMsgs()
            if message is None or message.time_stamp <= 0 or message.Hz <= 0:
                return None
            state = message.joint_state
            raw = np.array(
                [state.joint_1, state.joint_2, state.joint_3,
                 state.joint_4, state.joint_5, state.joint_6], dtype=np.float64
            )
        except (AttributeError, TypeError, ValueError):
            return None
        joints = sdk_mdeg_to_radians(raw)
        return joints if np.all(np.isfinite(joints)) else None

    def send_joints(self, joints_rad: np.ndarray) -> None:
        target = radians_to_sdk_mdeg(joints_rad)
        self.piper.JointCtrl(*(int(value) for value in target))

    def send_gripper(self, trigger: float) -> None:
        trigger = float(np.clip(trigger, 0.0, 1.0))
        position = int(round((1.0 - trigger) * GRIPPER_OPEN_UNITS))
        self.piper.GripperCtrl(position, 1000, 0x01, 0)


def run_dry_run(args: argparse.Namespace, r_final: np.ndarray) -> int:
    """Exercise the complete mapping -> IK -> joint safety gate without CAN."""
    print("V2 CONTROL DRY-RUN: no XR import, no Piper SDK import, no CAN access")
    solver = PiperPinocchioIK(
        args.urdf, orientation_weight=0.0 if args.position_only else 0.2
    )
    reference_joints = np.array([0.0, 1.5, -1.5, 0.3, 0.2, 0.1])
    reference = solver.forward_transform(reference_joints)
    xr_reference_position = np.array([0.25, -0.15, 1.10])
    xr_reference_quaternion = Rotation.identity().as_quat()
    previous_position = reference.translation.copy()
    previous_joints = reference_joints.copy()
    first_target_no_jump = False
    failures = limited_steps = accepted = 0
    max_joint_step = max_position_error = max_orientation_error = 0.0
    period = 1.0 / args.control_rate

    for frame in range(121):
        phase = 2.0 * np.pi * frame / 120.0
        xr_position = xr_reference_position + np.array(
            [0.025 * np.sin(phase), 0.015 * (1.0 - np.cos(phase)),
             0.012 * np.sin(2.0 * phase)]
        )
        xr_rotation = Rotation.from_euler(
            "xyz", [8.0 * np.sin(phase), 6.0 * np.sin(2.0 * phase),
                     12.0 * np.sin(phase)], degrees=True
        )
        relative = args.position_scale * (r_final @ (xr_position - xr_reference_position))
        if args.safe_test:
            relative = clamp_norm(relative, SAFE_TEST_RADIUS_M)
        raw_position = reference.translation + relative
        if not args.no_workspace_limit:
            lower, upper = workspace_bounds_for_reference(reference.translation)
            raw_position = np.clip(raw_position, lower, upper)
        target_position = (
            raw_position if args.no_speed_limit
            else limit_target_speed(previous_position, raw_position, args.max_speed, period)
        )
        target_rotation = relative_target_rotation(
            xr_reference_quaternion, xr_rotation.as_quat(), reference.rotation,
            r_final, args.rotation_scale, args.max_orientation_delta_deg
        )
        result = solver.solve(
            target_position, Rotation.from_matrix(target_rotation).as_quat(),
            previous_joints, posture_reference_joint_angles_rad=reference_joints
        )
        if not result.success:
            failures += 1
            continue
        if args.no_joint_step_limit:
            command_joints = result.joint_angles_rad
            was_limited = False
        else:
            command_joints, was_limited = limit_joint_step(
                previous_joints,
                result.joint_angles_rad,
                math.radians(args.max_joint_step_deg),
            )
        step = float(np.max(np.abs(command_joints - previous_joints)))
        if frame == 0:
            first_target_no_jump = step < 1.0e-7
        limited_steps += int(was_limited)
        previous_joints = command_joints
        previous_position = solver.forward_transform(command_joints).translation.copy()
        accepted += 1
        max_joint_step = max(max_joint_step, step)
        max_position_error = max(max_position_error, result.position_error_m)
        max_orientation_error = max(max_orientation_error, result.orientation_error_rad)

    returned_home = float(np.max(np.abs(previous_joints - reference_joints)))
    returned_transform = solver.forward_transform(previous_joints)
    returned_position_error = float(
        np.linalg.norm(returned_transform.translation - reference.translation)
    )
    returned_orientation_error = float(
        np.linalg.norm(
            Rotation.from_matrix(
                returned_transform.rotation.T @ reference.rotation
            ).as_rotvec()
        )
    )
    passed = bool(
        first_target_no_jump and failures == 0
        and accepted == 121
        and returned_position_error <= solver.position_tolerance_m
        and (args.position_only or returned_orientation_error <= math.radians(1.0))
        and max_position_error <= solver.position_tolerance_m
    )
    print(
        "V2 CONTROL DRY-RUN SUMMARY: "
        f"frames=121, accepted={accepted}, ik_failures={failures}, "
        f"joint_step_limit={'disabled' if args.no_joint_step_limit else f'{args.max_joint_step_deg:g}deg/frame'}, "
        f"joint_step_limited_frames={limited_steps}, first_target_no_jump={first_target_no_jump}, "
        f"max_position_error_mm={max_position_error * 1000.0:.3f}, "
        f"max_orientation_error_deg={math.degrees(max_orientation_error):.3f}, "
        f"max_joint_step_deg={math.degrees(max_joint_step):.3f}, "
        f"returned_home_max_deg={math.degrees(returned_home):.3f}, "
        f"returned_position_error_mm={returned_position_error * 1000.0:.3f}, "
        f"returned_orientation_error_deg={math.degrees(returned_orientation_error):.3f}, "
        f"result={'PASS' if passed else 'FAIL'}"
    )
    return 0 if passed else 1


def run_hardware(args: argparse.Namespace, r_final: np.ndarray) -> int:
    print("!" * 76)
    print("V2 HARDWARE MODE: enables Piper and sends live JOINT targets on CAN.")
    print("It will NOT home, reset, or disable the arm on exit.")
    if args.safe_test:
        print("SAFE TEST: 10% robot speed, 0.25 scale, 2 cm radius, gripper disabled.")
    print("!" * 76)
    try:
        confirmation = input("Type ARM exactly to continue: ").strip()
    except EOFError:
        confirmation = ""
    if confirmation != "ARM":
        print("Cancelled; no XR/Piper SDK imported and no CAN interface opened.")
        return 2

    try:
        import xrobotoolkit_sdk as xrt
    except ImportError as exc:
        raise RuntimeError("XRoboToolkit SDK is unavailable") from exc

    solver = PiperPinocchioIK(
        args.urdf, orientation_weight=0.0 if args.position_only else 0.2
    )
    hardware: Optional[PiperJointHardware] = None
    xr_initialized = False
    try:
        hardware = PiperJointHardware(args.can_name, args.speed_percent)
        if not args.no_gripper:
            hardware.send_gripper(0.0)
            print(f"[gripper] startup full-open command sent ({GRIPPER_OPEN_UNITS} units)")
        xrt.init()
        xr_initialized = True
        print(
            f"V2 ready: {args.control_rate:g}Hz, hand={args.controller_hand}, "
            f"position_scale={args.position_scale:g}, rotation_scale={args.rotation_scale:g}, "
            f"orientation={'off' if args.position_only else 'soft quaternion IK'}, "
            f"orientation_range=+/-{args.max_orientation_delta_deg:g}deg, "
            f"max_joint_step={'unlimited' if args.no_joint_step_limit else f'{args.max_joint_step_deg:g}deg/frame'}, "
            f"robot_speed={args.speed_percent}%, yaw={args.yaw_deg}deg"
        )
        if args.controller_hand == "left":
            get_grip, get_trigger = xrt.get_left_grip, xrt.get_left_trigger
            get_pose = xrt.get_left_controller_pose
        else:
            get_grip, get_trigger = xrt.get_right_grip, xrt.get_right_trigger
            get_pose = xrt.get_right_controller_pose

        period = 1.0 / args.control_rate
        next_tick = last_cycle = last_new_xr_time = time.monotonic()
        last_timestamp = None
        grip_active = activation_pending = False
        require_regrip = True
        valid_frames = 0
        stale_announced = invalid_announced = False
        xr_reference_robot = xr_reference_quaternion = None
        ee_reference_position = ee_reference_rotation = None
        posture_reference = previous_joints = previous_position = None
        previous_trigger = None
        last_gripper_send = 0.0
        failure_count = 0
        joint_limit_announced = False

        def clear_clutch() -> None:
            nonlocal xr_reference_robot, xr_reference_quaternion
            nonlocal ee_reference_position, ee_reference_rotation
            nonlocal posture_reference, previous_joints, previous_position
            nonlocal joint_limit_announced
            xr_reference_robot = xr_reference_quaternion = None
            ee_reference_position = ee_reference_rotation = None
            posture_reference = previous_joints = previous_position = None
            joint_limit_announced = False

        while True:
            cycle_start = time.monotonic()
            next_tick += period
            dt = float(np.clip(cycle_start - last_cycle, 0.005, 0.05))
            last_cycle = cycle_start
            timestamp = xrt.get_time_stamp_ns()
            grip = float(get_grip())
            trigger = float(get_trigger())
            grip_active, rising, falling = update_grip_state(grip_active, grip)
            if rising and not require_regrip:
                activation_pending = True
            if falling:
                activation_pending = False
                clear_clutch()
                print("[clutch] released; joint target updates stopped")

            new_pose = timestamp != last_timestamp
            if new_pose:
                last_timestamp = timestamp
                last_new_xr_time = cycle_start
                stale_announced = False
                pose = get_pose()
                xr_position = np.asarray(pose[:3], dtype=np.float64)
                xr_quaternion = normalize_quaternion(pose[3:7])
                if not valid_xr_position(xr_position) or xr_quaternion is None:
                    if not invalid_announced:
                        print("[XR ERROR] invalid pose; blocked until 10 valid frames and re-clutch")
                    invalid_announced = True
                    valid_frames = 0
                    require_regrip = True
                    activation_pending = False
                    clear_clutch()
                    new_pose = False
                else:
                    valid_frames = min(valid_frames + 1, XR_VALID_FRAMES_REQUIRED)
                    xr_robot = r_final @ xr_position
                    if valid_frames >= XR_VALID_FRAMES_REQUIRED:
                        if require_regrip or invalid_announced:
                            print("[XR READY] 10 valid frames; grip may activate")
                        require_regrip = False
                        invalid_announced = False
                        if grip_active and xr_reference_robot is None:
                            activation_pending = True

                if activation_pending and not require_regrip and new_pose:
                    activation_pending = False
                    measured = hardware.read_joints()
                    if measured is None:
                        print("[PIPER ERROR] no complete joint feedback; release and re-clutch")
                        require_regrip = True
                        clear_clutch()
                    else:
                        transform = solver.forward_transform(measured)
                        xr_reference_robot = xr_robot.copy()
                        xr_reference_quaternion = xr_quaternion.copy()
                        ee_reference_position = transform.translation.copy()
                        ee_reference_rotation = transform.rotation.copy()
                        posture_reference = previous_joints = measured.copy()
                        previous_position = ee_reference_position.copy()
                        failure_count = 0
                        joint_limit_announced = False
                        hardware.confirm_joint_mode()
                        hardware.send_joints(measured)
                        print(
                            "[clutch] activated; first target equals measured joints "
                            f"{np.array2string(np.rad2deg(measured), precision=2)}deg"
                        )
                elif (
                    new_pose and grip_active and not require_regrip
                    and xr_reference_robot is not None and xr_reference_quaternion is not None
                    and ee_reference_position is not None and ee_reference_rotation is not None
                    and posture_reference is not None and previous_joints is not None
                    and previous_position is not None
                ):
                    relative = args.position_scale * (xr_robot - xr_reference_robot)
                    if args.safe_test:
                        relative = clamp_norm(relative, SAFE_TEST_RADIUS_M)
                    raw_position = ee_reference_position + relative
                    if not args.no_workspace_limit:
                        lower, upper = workspace_bounds_for_reference(ee_reference_position)
                        raw_position = np.clip(raw_position, lower, upper)
                    target_position = (
                        raw_position if args.no_speed_limit
                        else limit_target_speed(previous_position, raw_position, args.max_speed, dt)
                    )
                    target_rotation = relative_target_rotation(
                        xr_reference_quaternion, xr_quaternion, ee_reference_rotation,
                        r_final, args.rotation_scale, args.max_orientation_delta_deg
                    )
                    result = solver.solve(
                        target_position, Rotation.from_matrix(target_rotation).as_quat(),
                        previous_joints, posture_reference_joint_angles_rad=posture_reference
                    )
                    if not result.success:
                        failure_count += 1
                        if failure_count == 1 or failure_count % 25 == 0:
                            print(
                                f"[IK HOLD] {result.reason}; position error="
                                f"{result.position_error_m * 1000.0:.2f}mm"
                            )
                    else:
                        if args.no_joint_step_limit:
                            command_joints = result.joint_angles_rad
                            was_limited = False
                        else:
                            command_joints, was_limited = limit_joint_step(
                                previous_joints,
                                result.joint_angles_rad,
                                math.radians(args.max_joint_step_deg),
                            )
                        if was_limited and not joint_limit_announced:
                            requested_step = float(
                                np.max(np.abs(result.joint_angles_rad - previous_joints))
                            )
                            print(
                                f"[JOINT LIMIT] requested {math.degrees(requested_step):.3f}deg; "
                                f"limited to {args.max_joint_step_deg:g}deg/frame"
                            )
                            joint_limit_announced = True
                        elif not was_limited:
                            joint_limit_announced = False
                        hardware.send_joints(command_joints)
                        previous_joints = command_joints
                        previous_position = solver.forward_transform(
                            command_joints
                        ).translation.copy()
                        failure_count = 0

            if cycle_start - last_new_xr_time > XR_STALE_SECONDS:
                if not stale_announced:
                    print("[XR STALE] >0.2s; updates stopped; 10 valid frames required")
                    stale_announced = True
                require_regrip = True
                activation_pending = False
                valid_frames = 0
                clear_clutch()

            if grip_active and not args.no_gripper:
                trigger = float(np.clip(trigger, 0.0, 1.0))
                if args.binary_gripper:
                    trigger = binary_gripper_trigger(previous_trigger, trigger)
                changed = previous_trigger is None or abs(trigger - previous_trigger) > 0.01
                if changed or cycle_start - last_gripper_send >= 0.20:
                    hardware.send_gripper(trigger)
                    previous_trigger = trigger
                    last_gripper_send = cycle_start

            remaining = next_tick - time.monotonic()
            if remaining > 0.0:
                time.sleep(remaining)
            else:
                next_tick = time.monotonic()
    except KeyboardInterrupt:
        print("\nStopped updates. Arm remains enabled; no home/reset/disable sent.")
        return 0
    finally:
        if xr_initialized:
            xrt.close()


def main() -> int:
    args = parse_args()
    r_final = installation_rotation(args.yaw_deg) @ R_XR_TO_PIPER
    return run_hardware(args, r_final) if args.hardware else run_dry_run(args, r_final)


if __name__ == "__main__":
    sys.exit(main())
