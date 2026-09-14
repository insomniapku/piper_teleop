#!/usr/bin/env python3
"""Single-process bimanual Pico teleoperation for two Piper arms.

One XR SDK client reads both controllers.  Each hand owns an independent
clutch, reference pose, IK solver, joint limiter, CAN interface, and gripper.
Dry-run is the default and never imports the XR or Piper SDKs.
"""

from __future__ import annotations

import argparse
import math
import os
import select
import sys
import time
from dataclasses import dataclass, field
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

# Captured from live can0/can1 feedback on 2026-09-14.
DEFAULT_HOME_JOINTS_DEG = "0.693,-0.227,-9.648,2.311,19.343,0.000"
DEFAULT_RIGHT_HOME_JOINTS_DEG = "-1.513,-0.349,-1.076,1.592,17.133,-0.399"


def parse_joints_deg(text: str) -> np.ndarray:
    parts = [item.strip() for item in str(text).split(",")]
    if len(parts) != 6:
        raise argparse.ArgumentTypeError(
            "home joints must be 6 comma-separated degree values"
        )
    try:
        values = np.array([float(item) for item in parts], dtype=np.float64)
    except ValueError as exc:
        raise argparse.ArgumentTypeError(
            "home joints must be numeric degree values"
        ) from exc
    if not np.all(np.isfinite(values)):
        raise argparse.ArgumentTypeError("home joints must be finite")
    return np.deg2rad(values)


def attach_negative_option_values(argv: list[str], option_names: tuple[str, ...]) -> list[str]:
    """Keep values that start with '-' attached to their option names.

    argparse otherwise treats ``--right-home-joints-deg -1.5,...`` as a missing
    argument followed by an unknown flag.
    """
    names = set(option_names)
    rewritten: list[str] = []
    index = 0
    while index < len(argv):
        argument = argv[index]
        if (
            argument in names
            and index + 1 < len(argv)
            and argv[index + 1].startswith("-")
            and not argv[index + 1].startswith("--")
        ):
            rewritten.append(f"{argument}={argv[index + 1]}")
            index += 2
            continue
        rewritten.append(argument)
        index += 1
    return rewritten


class KeyboardPoller:
    """Non-blocking single-character stdin reader for the control loop."""

    def __init__(self) -> None:
        self._fd: Optional[int] = None
        self._old_term = None
        self._old_flags: Optional[int] = None
        if not sys.stdin.isatty():
            return
        try:
            import termios
            import tty
        except ImportError:
            return
        fd = sys.stdin.fileno()
        self._old_term = termios.tcgetattr(fd)
        self._old_flags = fcntl_get_flags(fd)
        tty.setcbreak(fd)
        fcntl_set_nonblocking(fd)
        self._fd = fd

    def pressed(self, keys: str = "aA") -> bool:
        if self._fd is None:
            return False
        ready, _, _ = select.select([sys.stdin], [], [], 0)
        if not ready:
            return False
        try:
            chunk = os.read(self._fd, 64).decode("utf-8", errors="ignore")
        except OSError:
            return False
        return any(character in keys for character in chunk)

    def close(self) -> None:
        if self._fd is None:
            return
        try:
            import termios

            if self._old_term is not None:
                termios.tcsetattr(self._fd, termios.TCSADRAIN, self._old_term)
            if self._old_flags is not None:
                fcntl_set_flags(self._fd, self._old_flags)
        except Exception:
            pass
        self._fd = None


def fcntl_get_flags(fd: int) -> int:
    import fcntl

    return fcntl.fcntl(fd, fcntl.F_GETFL)


def fcntl_set_flags(fd: int, flags: int) -> None:
    import fcntl

    fcntl.fcntl(fd, fcntl.F_SETFL, flags)


def fcntl_set_nonblocking(fd: int) -> None:
    import fcntl

    fcntl_set_flags(fd, fcntl_get_flags(fd) | os.O_NONBLOCK)


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
    parser.add_argument(
        "--home-joints-deg",
        default=DEFAULT_HOME_JOINTS_DEG,
        help="Shared 6-joint home pose in degrees, comma-separated (j1..j6)",
    )
    parser.add_argument(
        "--left-home-joints-deg",
        default=None,
        help="Optional left-arm home pose in degrees; defaults to --home-joints-deg",
    )
    parser.add_argument(
        "--right-home-joints-deg",
        default=DEFAULT_RIGHT_HOME_JOINTS_DEG,
        help="Optional right-arm home pose in degrees; defaults to the captured right-arm pose",
    )
    parser.add_argument(
        "--home-duration",
        type=float,
        default=3.0,
        help="Seconds to interpolate both arms to the fixed home pose after A terminates teleop",
    )
    args = parser.parse_args(
        attach_negative_option_values(
            sys.argv[1:],
            (
                "--home-joints-deg",
                "--left-home-joints-deg",
                "--right-home-joints-deg",
            ),
        )
    )

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
    if args.home_duration <= 0.0:
        parser.error("--home-duration must be positive")
    try:
        args.left_home_joints_rad = parse_joints_deg(
            args.left_home_joints_deg or args.home_joints_deg
        )
        args.right_home_joints_rad = parse_joints_deg(
            args.right_home_joints_deg or args.home_joints_deg
        )
    except argparse.ArgumentTypeError as exc:
        parser.error(str(exc))
    if args.safe_test:
        args.position_scale = 0.25
        args.rotation_scale = min(args.rotation_scale, 0.5)
        args.max_orientation_delta_deg = min(args.max_orientation_delta_deg, 10.0)
        args.max_speed = 0.02
        args.max_joint_step_deg = min(args.max_joint_step_deg, 0.25)
        args.speed_percent = min(args.speed_percent, 10)
        args.no_gripper = True

    args.dry_run = not args.hardware
    if args.enable_recording:
        if args.camera_fps <= 0.0:
            parser.error("--camera-fps must be positive")
        recording_ratio = args.control_rate / args.camera_fps
        if recording_ratio < 1.0 or abs(recording_ratio - round(recording_ratio)) > 1.0e-9:
            parser.error(
                "--control-rate must be >= and an integer multiple of --camera-fps "
                "when recording without interpolation"
            )
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
    home_joints: np.ndarray = field(default_factory=lambda: np.zeros(6, dtype=np.float64))

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
    homing_active: bool = False
    homing_start_joints: Optional[np.ndarray] = None
    homing_elapsed: float = 0.0

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

    def cancel_homing(self) -> None:
        self.homing_active = False
        self.homing_start_joints = None
        self.homing_elapsed = 0.0

    def start_homing(self) -> bool:
        measured = self.hardware.read_joints()
        if measured is None and self.previous_joints is not None:
            measured = self.previous_joints.copy()
        if measured is None:
            self.log("[HOME] no joint feedback; cannot start homing")
            return False
        self.grip_active = False
        self.require_regrip = True
        self.activation_pending = False
        self.valid_frames = 0
        self.clear_clutch()
        self.hardware.confirm_joint_mode()
        if not self.args.no_gripper:
            self.hardware.send_gripper(0.0)
            self.previous_trigger = 0.0
            self.last_gripper_send = time.monotonic()
        self.homing_start_joints = measured.copy()
        self.previous_joints = measured.copy()
        self.homing_elapsed = 0.0
        self.homing_active = True
        self.log(
            "[HOME] teleop terminated; moving to "
            f"{np.array2string(np.rad2deg(self.home_joints), precision=1)}deg "
            f"over {self.args.home_duration:g}s"
        )
        return True

    def process_homing(self, dt: float) -> None:
        if not self.homing_active or self.homing_start_joints is None:
            return
        duration = max(float(self.args.home_duration), dt)
        self.homing_elapsed = min(self.homing_elapsed + dt, duration)
        alpha = self.homing_elapsed / duration
        target = (1.0 - alpha) * self.homing_start_joints + alpha * self.home_joints
        if not self.args.no_joint_step_limit:
            target, _ = limit_joint_step(
                self.previous_joints
                if self.previous_joints is not None
                else self.homing_start_joints,
                target,
                math.radians(self.args.max_joint_step_deg),
            )
        self.hardware.send_joints(target)
        self.previous_joints = target
        remaining = float(np.max(np.abs(target - self.home_joints)))
        if remaining < math.radians(0.5):
            self.hardware.send_joints(self.home_joints)
            self.previous_joints = self.home_joints.copy()
            self.homing_active = False
            self.homing_start_joints = None
            self.require_regrip = True
            self.log("[HOME] reached fixed pose; re-grip before teleop")

    def update_buttons(self, now: float) -> None:
        if self.homing_active:
            return
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

    def state_for_recording(
        self,
    ) -> Optional[tuple[np.ndarray, int, float | None, int, float]]:
        """Return arm feedback, gripper command, and mapped timestamps."""
        measured = self.hardware.read_joints_with_timestamp()
        gripper_trigger = 0.0 if self.previous_trigger is None else self.previous_trigger
        if measured is not None:
            joints, feedback_timestamp_ns, device_timestamp, host_timestamp_ns = measured
            return (
                joints.copy(),
                feedback_timestamp_ns,
                device_timestamp,
                host_timestamp_ns,
                float(gripper_trigger),
            )
        if self.previous_joints is not None:
            timestamp_ns = time.monotonic_ns()
            return (
                self.previous_joints.copy(),
                timestamp_ns,
                None,
                timestamp_ns,
                float(gripper_trigger),
            )
        return None

    def process_pose(self, dt: float) -> None:
        if self.homing_active:
            return
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
    print("Ctrl+C leaves both arms enabled at the last commanded pose.")
    print(
        "Press A (Pico) or keyboard a to terminate teleop, stop recording, "
        "and interpolate both arms to the fixed home pose."
    )
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
            camera_indices=(0, 2, 8),  # Logical slots: two Dabai streams and RealSense
            fps=args.camera_fps,
            recording_dir=args.recording_dir,
        )

    xr_initialized = False
    previous_b_button = False
    previous_a_button = False
    keyboard = KeyboardPoller()

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
            home_joints=args.left_home_joints_rad.copy(),
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
            home_joints=args.right_home_joints_rad.copy(),
        )
        arms = (left, right)
        print(
            f"Bimanual V2 ready: {args.control_rate:g}Hz, scale={args.position_scale:g}, "
            f"rotation_scale={args.rotation_scale:g}, max_joint_step="
            f"{'unlimited' if args.no_joint_step_limit else f'{args.max_joint_step_deg:g}deg/frame'}, "
            f"IK tolerance={args.ik_position_tolerance_mm:g}mm, speed={args.speed_percent}%, "
            f"left={args.left_can_name}/yaw{args.left_yaw_deg}, "
            f"right={args.right_can_name}/yaw{args.right_yaw_deg}, "
            f"home={np.array2string(np.rad2deg(args.left_home_joints_rad), precision=1)}deg/"
            f"{np.array2string(np.rad2deg(args.right_home_joints_rad), precision=1)}deg "
            f"in {args.home_duration:g}s"
        )

        period = 1.0 / args.control_rate
        recording_cycle_interval = int(round(args.control_rate / args.camera_fps))
        next_tick = last_cycle = last_new_xr_time = time.monotonic()
        last_timestamp = None
        stale_announced = False
        control_cycle_index = 0
        recording_start_cycle = 0
        while True:
            cycle_start_ns = time.monotonic_ns()
            cycle_start = cycle_start_ns / 1.0e9
            control_cycle_index += 1
            next_tick += period
            dt = float(np.clip(cycle_start - last_cycle, 0.005, 0.05))
            last_cycle = cycle_start

            for arm in arms:
                arm.update_buttons(cycle_start)

            current_a_button = bool(xrt.get_A_button())
            home_requested = (current_a_button and not previous_a_button) or keyboard.pressed()
            previous_a_button = current_a_button
            if home_requested:
                already_homing = any(arm.homing_active for arm in arms)
                if already_homing:
                    print("[HOME] ignored; already returning to the fixed pose")
                else:
                    if recorder and recorder.is_recording:
                        recorder.stop_recording()
                        print("[HOME] recording stopped; returning to the fixed pose")
                    started = [arm.start_homing() for arm in arms]
                    if not all(started):
                        for arm, ok in zip(arms, started):
                            if ok:
                                arm.cancel_homing()
                        print("[HOME] aborted; missing joint feedback on at least one arm")

            # Handle B button for recording
            if recorder:
                current_b_button = xrt.get_B_button()
                if current_b_button and not previous_b_button:
                    # Button pressed (rising edge)
                    if recorder.is_recording:
                        recorder.stop_recording()
                        print("[RECORDER] Recording stopped (B button pressed)")
                    else:
                        if any(arm.homing_active for arm in arms):
                            print("[RECORDER] ignored; wait for homing to finish")
                        elif recorder.start_recording():
                            print("[RECORDER] Recording started (B button pressed)")
                            # Start on the next tick so the first timestamp is not before
                            # the recorder's session start time. At 60/30 Hz this is every 2 cycles.
                            recording_start_cycle = control_cycle_index + 1
                previous_b_button = current_b_button

            for arm in arms:
                if arm.homing_active:
                    arm.process_homing(dt)

            timestamp = xrt.get_time_stamp_ns()
            if timestamp != last_timestamp:
                last_timestamp = timestamp
                last_new_xr_time = cycle_start
                stale_announced = False
                for arm in arms:
                    arm.process_pose(dt)

            # Sample actual arm feedback at every 60 Hz control tick. Recording
            # requests are matched to these samples by the asynchronous writer.
            if recorder and recorder.is_recording:
                left_state = left.state_for_recording()
                right_state = right.state_for_recording()
                if left_state is not None and right_state is not None:
                    (
                        left_joints,
                        left_timestamp_ns,
                        left_device_timestamp,
                        left_host_timestamp_ns,
                        left_gripper_trigger,
                    ) = left_state
                    (
                        right_joints,
                        right_timestamp_ns,
                        right_device_timestamp,
                        right_host_timestamp_ns,
                        right_gripper_trigger,
                    ) = right_state
                    recorder.record_state(
                        left_joints,
                        right_joints,
                        left_timestamp_ns,
                        right_timestamp_ns,
                        left_device_timestamp,
                        right_device_timestamp,
                        left_host_timestamp_ns,
                        right_host_timestamp_ns,
                        left_gripper_trigger,
                        right_gripper_trigger,
                    )
                    if (
                        (control_cycle_index - recording_start_cycle)
                        % recording_cycle_interval
                        == 0
                    ):
                        recorder.record_frame(
                            timestamp_ns=max(left_timestamp_ns, right_timestamp_ns)
                        )

            if cycle_start - last_new_xr_time > XR_STALE_SECONDS:
                if not stale_announced:
                    print("[BOTH] [XR STALE] >0.2s; both arms stopped")
                    stale_announced = True
                for arm in arms:
                    if not arm.homing_active:
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
        keyboard.close()
        if recorder:
            recorder.cleanup()
        if xr_initialized:
            xrt.close()


def main() -> int:
    args = parse_args()
    return run_hardware(args) if args.hardware else run_bimanual_dry_run(args)


if __name__ == "__main__":
    sys.exit(main())
