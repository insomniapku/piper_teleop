#!/usr/bin/env python3
"""Low-latency Pico XR pose teleoperation for a Piper arm.

The default execution path is a short, hardware-free dry-run. Real XR/CAN
interfaces are imported and opened only with ``--hardware`` and an explicit
``ARM`` confirmation.
"""

import argparse
import math
import sys
import time
import warnings
from typing import Optional, Tuple

import numpy as np
from scipy.spatial.transform import Rotation


R_XR_TO_PIPER = np.array(
    [
        [0.0, 0.0, -1.0],
        [-1.0, 0.0, 0.0],
        [0.0, 1.0, 0.0],
    ],
    dtype=np.float64,
)

WORKSPACE_MIN_M = np.array([0.10, -0.20, 0.10], dtype=np.float64)
WORKSPACE_MAX_M = np.array([0.35, 0.20, 0.35], dtype=np.float64)
GRIP_ON_THRESHOLD = 0.80
GRIP_OFF_THRESHOLD = 0.60
XR_STALE_SECONDS = 0.20
XR_ZERO_POSITION_EPS_M = 1e-6
XR_QUATERNION_EPS = 1e-8
XR_VALID_FRAMES_REQUIRED = 10
SAFE_TEST_RADIUS_M = 0.03
PIPER_POSITION_UNITS_PER_METER = 1_000_000
GRIPPER_OPEN_UNITS = 100_000
GRIPPER_BINARY_CLOSE_THRESHOLD = 0.60
GRIPPER_BINARY_OPEN_THRESHOLD = 0.40


def installation_rotation(yaw_deg: int) -> np.ndarray:
    """Return the permitted extra installation yaw rotation."""
    yaw_rad = math.radians(yaw_deg)
    cos_yaw = math.cos(yaw_rad)
    sin_yaw = math.sin(yaw_rad)
    return np.array(
        [
            [cos_yaw, -sin_yaw, 0.0],
            [sin_yaw, cos_yaw, 0.0],
            [0.0, 0.0, 1.0],
        ],
        dtype=np.float64,
    )


def limit_target_speed(
    previous_cmd: np.ndarray,
    raw_target: np.ndarray,
    max_speed_mps: float,
    dt: float,
) -> np.ndarray:
    """Limit one target update using the measured monotonic loop period."""
    step = raw_target - previous_cmd
    max_step = max_speed_mps * dt
    step_norm = float(np.linalg.norm(step))
    if step_norm > max_step and step_norm > 0.0:
        step = step / step_norm * max_step
    return previous_cmd + step


def clamp_relative_radius(delta: np.ndarray, radius_m: float) -> np.ndarray:
    """Limit relative travel for commissioning mode only."""
    delta_norm = float(np.linalg.norm(delta))
    if delta_norm > radius_m and delta_norm > 0.0:
        return delta / delta_norm * radius_m
    return delta


def workspace_bounds_for_reference(reference_m: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
    """Include the clutch pose without allowing motion farther outside limits.

    A Piper can legitimately be enabled at a folded pose outside the configured
    teleoperation box.  Clipping the first zero-delta target directly to the box
    would create motion even when the XR controller is stationary.  Expanding
    each bound only as far as the clutch pose preserves the no-jump invariant
    while still preventing motion farther away from the configured workspace.
    """
    reference_m = np.asarray(reference_m, dtype=np.float64)
    return (
        np.minimum(WORKSPACE_MIN_M, reference_m),
        np.maximum(WORKSPACE_MAX_M, reference_m),
    )


def is_valid_xr_position(position_m: np.ndarray) -> bool:
    """Reject non-finite and exact-zero XR poses used for tracking loss."""
    position_m = np.asarray(position_m, dtype=np.float64)
    return bool(
        position_m.shape == (3,)
        and np.all(np.isfinite(position_m))
        and np.linalg.norm(position_m) > XR_ZERO_POSITION_EPS_M
    )


def normalize_xr_quaternion(quaternion: np.ndarray) -> Optional[np.ndarray]:
    """Return a normalized [x, y, z, w] quaternion, or None if invalid."""
    quaternion = np.asarray(quaternion, dtype=np.float64)
    if quaternion.shape != (4,) or not np.all(np.isfinite(quaternion)):
        return None
    norm = float(np.linalg.norm(quaternion))
    if norm <= XR_QUATERNION_EPS:
        return None
    return quaternion / norm


def map_relative_orientation(
    xr_reference_quat: np.ndarray,
    xr_current_quat: np.ndarray,
    ee_reference_rotation: Rotation,
    ee_reference_euler_deg: np.ndarray,
    r_final: np.ndarray,
    rotation_scale: float,
    max_delta_deg: float,
    orientation_mode: str,
    wrist_roll_sign: float,
    wrist_pitch_scale: float,
    wrist_pitch_sign: float,
) -> Tuple[int, int, int]:
    """Map relative XR rotation to Piper extrinsic XYZ Euler millidegrees."""
    reference = normalize_xr_quaternion(xr_reference_quat)
    current = normalize_xr_quaternion(xr_current_quat)
    if reference is None or current is None:
        raise ValueError("invalid XR controller quaternion")

    reference_rotation = Rotation.from_quat(reference)
    current_rotation = Rotation.from_quat(current)
    if orientation_mode in ("tool-roll", "tool-wrist"):
        # OpenXR controller grip pose uses local -Z as its pointing direction.
        # Local Z roll maps to Piper tool-local Z (primarily wrist joint 6).
        # In tool-wrist mode, controller local X pitch maps to Piper tool-local
        # Y (primarily wrist joint 5), letting the gripper tilt up/down.
        delta_local = reference_rotation.inv() * current_rotation
        delta_local_rotvec = delta_local.as_rotvec()
        tool_roll = (
            float(delta_local_rotvec[2]) * rotation_scale * wrist_roll_sign
        )
        tool_pitch = 0.0
        if orientation_mode == "tool-wrist":
            tool_pitch = (
                float(delta_local_rotvec[0])
                * wrist_pitch_scale
                * wrist_pitch_sign
            )
        delta_rotvec_piper = np.array(
            [0.0, tool_pitch, tool_roll], dtype=np.float64
        )
    elif orientation_mode == "full":
        delta_xr = current_rotation * reference_rotation.inv()
        delta_rotvec_piper = r_final @ delta_xr.as_rotvec() * rotation_scale
    else:
        raise ValueError(f"unsupported orientation mode: {orientation_mode}")
    delta_angle = float(np.linalg.norm(delta_rotvec_piper))
    max_delta_rad = math.radians(max_delta_deg)
    if delta_angle > max_delta_rad and delta_angle > 0.0:
        delta_rotvec_piper *= max_delta_rad / delta_angle

    if orientation_mode in ("tool-roll", "tool-wrist"):
        target_rotation = ee_reference_rotation * Rotation.from_rotvec(
            delta_rotvec_piper
        )
    else:
        target_rotation = Rotation.from_rotvec(delta_rotvec_piper) * ee_reference_rotation
    # Piper uses static/extrinsic XYZ Euler angles. Near pitch +/-90 degrees
    # SciPy warns about the equivalent Euler representation, but still returns
    # a valid physical rotation; suppress the per-frame warning flood.
    with warnings.catch_warnings():
        warnings.simplefilter("ignore", UserWarning)
        canonical_euler_deg = target_rotation.as_euler("xyz", degrees=True)
    reference_euler_deg = np.asarray(ee_reference_euler_deg, dtype=np.float64)
    if reference_euler_deg.shape != (3,):
        raise ValueError("invalid Piper Euler reference")

    # Static XYZ Euler has two equivalent branches. Close to pitch +/-90 deg,
    # SciPy's canonical branch can jump RX and RZ by ~180 deg even for a small
    # physical rotation. Generate both branches, unwrap each near the clutch
    # reference, and send the numerically closest equivalent representation.
    alternate_euler_deg = np.array(
        [
            canonical_euler_deg[0] + 180.0,
            180.0 - canonical_euler_deg[1],
            canonical_euler_deg[2] + 180.0,
        ],
        dtype=np.float64,
    )

    def unwrap_near(candidate: np.ndarray) -> np.ndarray:
        return candidate + 360.0 * np.round(
            (reference_euler_deg - candidate) / 360.0
        )

    canonical_euler_deg = unwrap_near(canonical_euler_deg)
    alternate_euler_deg = unwrap_near(alternate_euler_deg)
    if np.linalg.norm(alternate_euler_deg - reference_euler_deg) < np.linalg.norm(
        canonical_euler_deg - reference_euler_deg
    ):
        euler_deg = alternate_euler_deg
    else:
        euler_deg = canonical_euler_deg
    if not np.all(np.isfinite(euler_deg)):
        raise ValueError("non-finite Piper Euler target")
    euler_mdeg = np.rint(euler_deg * 1000.0).astype(int)
    return tuple(int(value) for value in euler_mdeg)


def update_grip_state(was_active: bool, grip_value: float) -> Tuple[bool, bool, bool]:
    """Apply grip hysteresis and return active, rising, and falling states."""
    active = was_active
    if not active and grip_value >= GRIP_ON_THRESHOLD:
        active = True
    elif active and grip_value <= GRIP_OFF_THRESHOLD:
        active = False
    return active, active and not was_active, was_active and not active


def binary_gripper_trigger(
    previous_state: Optional[float], raw_trigger: float
) -> float:
    """Return only 0.0 (fully open) or 1.0 (fully closed), with hysteresis."""
    raw_trigger = float(np.clip(raw_trigger, 0.0, 1.0))
    if raw_trigger >= GRIPPER_BINARY_CLOSE_THRESHOLD:
        return 1.0
    if raw_trigger <= GRIPPER_BINARY_OPEN_THRESHOLD:
        return 0.0
    if previous_state is None:
        return 0.0
    return 1.0 if previous_state >= 0.5 else 0.0


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Pico position clutch control for Piper (dry-run by default)."
    )
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument(
        "--dry-run",
        action="store_true",
        help="run the built-in XR trajectory without importing hardware SDKs (default)",
    )
    mode.add_argument(
        "--hardware",
        action="store_true",
        help="allow real XR/CAN setup after an interactive ARM confirmation",
    )
    parser.add_argument("--can-name", default="can0")
    parser.add_argument("--control-rate", type=float, default=50.0)
    parser.add_argument("--position-scale", type=float, default=1.0)
    parser.add_argument("--max-speed", type=float, default=0.08)
    parser.add_argument(
        "--no-speed-limit",
        action="store_true",
        help="send the mapped target without Python step limiting",
    )
    parser.add_argument(
        "--no-workspace-limit",
        action="store_true",
        help="do not clamp Cartesian X/Y/Z targets in software",
    )
    parser.add_argument("--speed-percent", type=int, default=40)
    parser.add_argument("--yaw-deg", type=int, choices=(0, 90, -90, 180), default=0)
    parser.add_argument(
        "--controller-hand",
        choices=("left", "right"),
        default="right",
        help="XR controller used for pose, grip clutch, and trigger",
    )
    parser.add_argument(
        "--follow-orientation",
        action="store_true",
        help="map relative controller quaternion changes to Piper RX/RY/RZ",
    )
    parser.add_argument("--rotation-scale", type=float, default=1.0)
    parser.add_argument("--max-orientation-delta-deg", type=float, default=60.0)
    parser.add_argument(
        "--orientation-mode",
        choices=("full", "tool-roll", "tool-wrist"),
        default="full",
    )
    parser.add_argument(
        "--wrist-roll-sign",
        type=float,
        choices=(-1.0, 1.0),
        default=1.0,
    )
    parser.add_argument("--wrist-pitch-scale", type=float, default=1.0)
    parser.add_argument(
        "--wrist-pitch-sign",
        type=float,
        choices=(-1.0, 1.0),
        default=1.0,
    )
    parser.add_argument(
        "--safe-test",
        action="store_true",
        help="force scale=0.5, speed=0.03 m/s, and a 3 cm clutch radius",
    )
    parser.add_argument("--no-gripper", action="store_true")
    parser.add_argument(
        "--binary-gripper",
        action="store_true",
        help="command only fully open or fully closed gripper positions",
    )
    args = parser.parse_args()

    if args.control_rate <= 0.0:
        parser.error("--control-rate must be positive")
    if args.position_scale <= 0.0:
        parser.error("--position-scale must be positive")
    if args.rotation_scale <= 0.0:
        parser.error("--rotation-scale must be positive")
    if args.wrist_pitch_scale <= 0.0:
        parser.error("--wrist-pitch-scale must be positive")
    if not 0.0 < args.max_orientation_delta_deg <= 180.0:
        parser.error("--max-orientation-delta-deg must be in (0, 180]")
    if args.max_speed <= 0.0:
        parser.error("--max-speed must be positive")
    if not 1 <= args.speed_percent <= 100:
        parser.error("--speed-percent must be in [1, 100]")
    if args.safe_test and args.no_speed_limit:
        parser.error("--safe-test and --no-speed-limit cannot be used together")

    if args.safe_test:
        args.position_scale = 0.5
        args.max_speed = 0.03

    # An omitted mode is deliberately safe: it means dry-run, never hardware.
    args.dry_run = not args.hardware
    return args


class PiperHardware:
    """Thin wrapper around SDK calls; constructed only after ARM confirmation."""

    def __init__(self, can_name: str, speed_percent: int):
        try:
            from piper_sdk import C_PiperInterface_V2
        except ImportError as exc:
            raise RuntimeError(
                "Piper SDK is unavailable; install the repository piper_sdk package first"
            ) from exc

        self.speed_percent = speed_percent
        self.piper = C_PiperInterface_V2(can_name=can_name, judge_flag=False)
        # This workstation's Piper firmware reports max_range_config=100 mm
        # (large gripper). Match the SDK feedback filter to the hardware range.
        self.piper.SetSDKGripperRangeParam(0.0, GRIPPER_OPEN_UNITS / 1_000_000.0)
        self.piper.ConnectPort()
        time.sleep(0.2)

        for _ in range(100):
            if self.piper.EnablePiper():
                break
            time.sleep(0.01)
        else:
            raise RuntimeError("Piper enable failed; no motion command was sent")

        # MOVEP is an online point-target mode. Set it once here, then confirm it
        # only on each clutch rising edge; the 50 Hz loop sends EndPoseCtrl only.
        self.confirm_movep()

    def confirm_movep(self) -> None:
        self.piper.MotionCtrl_2(0x01, 0x00, self.speed_percent, 0x00)

    def read_end_pose(self) -> Optional[Tuple[np.ndarray, Tuple[int, int, int]]]:
        """Read actual xyz in meters and Euler feedback in SDK integer units."""
        try:
            message = self.piper.GetArmEndPoseMsgs()
            if message is None or message.time_stamp <= 0 or message.Hz <= 0:
                return None
            pose = message.end_pose
            raw = np.array(
                [pose.X_axis, pose.Y_axis, pose.Z_axis], dtype=np.float64
            )
            orientation = (int(pose.RX_axis), int(pose.RY_axis), int(pose.RZ_axis))
        except (AttributeError, TypeError, ValueError):
            return None

        position_m = raw / PIPER_POSITION_UNITS_PER_METER
        if not np.all(np.isfinite(position_m)):
            return None
        return position_m, orientation

    def send_pose(self, position_m: np.ndarray, orientation: Tuple[int, int, int]) -> None:
        xyz = np.rint(position_m * PIPER_POSITION_UNITS_PER_METER).astype(int)
        self.piper.EndPoseCtrl(
            int(xyz[0]), int(xyz[1]), int(xyz[2]), *orientation
        )

    def send_gripper(self, trigger_value: float) -> None:
        # Preserve the working legacy direction: trigger 0=open, trigger 1=closed.
        trigger_value = float(np.clip(trigger_value, 0.0, 1.0))
        gripper_position = int(round((1.0 - trigger_value) * GRIPPER_OPEN_UNITS))
        self.piper.GripperCtrl(gripper_position, 1000, 0x01, 0)


def run_dry_run(args: argparse.Namespace, r_final: np.ndarray) -> int:
    """Exercise mapping, clutch reference, speed limiting, and deadline timing."""
    print("DRY-RUN: no XR SDK import, no Piper SDK import, no CAN access")
    print(f"R_FINAL=\n{r_final}")

    period = 1.0 / args.control_rate
    duration_s = 2.4
    start = time.monotonic()
    next_tick = start
    last_cycle = start
    ee_reference = np.array([0.20, 0.0, 0.20], dtype=np.float64)
    base_xr = np.array([0.20, -0.10, 1.00], dtype=np.float64)
    xr_reference_robot: Optional[np.ndarray] = None
    previous_command: Optional[np.ndarray] = None
    grip_active = False
    first_target_ok = False
    speed_limit_observed = False
    finite_targets = True
    target_count = 0
    overrun_count = 0
    last_print = -1.0

    while True:
        cycle_start = time.monotonic()
        elapsed = cycle_start - start
        if elapsed >= duration_s:
            break
        next_tick += period
        dt = float(np.clip(cycle_start - last_cycle, 0.005, 0.05))
        last_cycle = cycle_start

        grip = 0.9 if 0.20 <= elapsed < 2.20 else 0.0
        xr_position = base_xr.copy()
        if 0.20 <= elapsed < 0.80:
            xr_position[0] += 0.12 * (elapsed - 0.20)
        elif elapsed >= 0.80:
            xr_position[0] += 0.072
        if 0.80 <= elapsed < 1.40:
            xr_position[2] -= 0.10 * (elapsed - 0.80)
        elif elapsed >= 1.40:
            xr_position[2] -= 0.060

        grip_active, rising, falling = update_grip_state(grip_active, grip)
        xr_robot = r_final @ xr_position

        if rising:
            xr_reference_robot = xr_robot.copy()
            previous_command = ee_reference.copy()
            target = previous_command.copy()
            first_target_ok = bool(np.array_equal(target, ee_reference))
            target_count += 1
            print(
                "first target="
                f"{np.array2string(target, precision=4)} (actual EE; no jump)"
            )
        elif grip_active and xr_reference_robot is not None and previous_command is not None:
            relative = args.position_scale * (xr_robot - xr_reference_robot)
            if args.safe_test:
                relative = clamp_relative_radius(relative, SAFE_TEST_RADIUS_M)
            raw_target = ee_reference + relative
            if not args.no_workspace_limit:
                workspace_min, workspace_max = workspace_bounds_for_reference(
                    ee_reference
                )
                raw_target = np.clip(raw_target, workspace_min, workspace_max)
            if args.no_speed_limit:
                target = raw_target
            else:
                target = limit_target_speed(
                    previous_command, raw_target, args.max_speed, dt
                )
                if not np.allclose(target, raw_target, rtol=0.0, atol=1e-12):
                    speed_limit_observed = True
            previous_command = target
            finite_targets = finite_targets and bool(np.all(np.isfinite(target)))
            target_count += 1
            if elapsed - last_print >= 0.20:
                print(
                    f"t={elapsed:4.2f}s xr={np.array2string(xr_position, precision=3)} "
                    f"target={np.array2string(target, precision=4)}"
                )
                last_print = elapsed
        elif falling:
            xr_reference_robot = None
            previous_command = None

        remaining = next_tick - time.monotonic()
        if remaining > 0.0:
            time.sleep(remaining)
        else:
            overrun_count += 1
            next_tick = time.monotonic()

    wall_time = time.monotonic() - start
    measured_rate = target_count / max(wall_time - 0.40, period)
    mapping_non_identity = not np.allclose(r_final, np.eye(3))
    rate_ok = abs(measured_rate - args.control_rate) <= max(3.0, args.control_rate * 0.10)
    outside_reference = np.array([0.0561, 0.0, 0.2133], dtype=np.float64)
    outside_min, outside_max = workspace_bounds_for_reference(outside_reference)
    outside_start_no_drift = bool(
        np.array_equal(
            np.clip(outside_reference, outside_min, outside_max), outside_reference
        )
    )
    zero_pose_rejected = not is_valid_xr_position(np.zeros(3, dtype=np.float64))
    nominal_pose_accepted = is_valid_xr_position(
        np.array([0.43, -0.32, 0.47], dtype=np.float64)
    )
    passed = all(
        (
            mapping_non_identity,
            first_target_ok,
            args.no_speed_limit or speed_limit_observed,
            finite_targets,
            rate_ok,
            outside_start_no_drift,
            zero_pose_rejected,
            nominal_pose_accepted,
        )
    )
    print(
        "DRY-RUN SUMMARY: "
        f"mapping_non_identity={mapping_non_identity}, "
        f"first_target_no_jump={first_target_ok}, "
        f"speed_limit={'disabled' if args.no_speed_limit else 'enabled'}, "
        f"workspace_limit={'disabled' if args.no_workspace_limit else 'enabled'}, "
        f"speed_limit_observed={speed_limit_observed}, finite={finite_targets}, "
        f"target_rate={measured_rate:.1f}Hz, overruns={overrun_count}, "
        f"outside_start_no_drift={outside_start_no_drift}, "
        f"zero_pose_rejected={zero_pose_rejected}, "
        f"nominal_pose_accepted={nominal_pose_accepted}, "
        f"result={'PASS' if passed else 'FAIL'}"
    )
    return 0 if passed else 1


def run_hardware(args: argparse.Namespace, r_final: np.ndarray) -> int:
    print("!" * 72)
    print("HARDWARE MODE: this will enable Piper and send live MOVEP targets on CAN.")
    print("The program will NOT home, reset, or disable the arm on exit.")
    print("!" * 72)
    try:
        confirmation = input("Type ARM exactly to continue: ").strip()
    except EOFError:
        confirmation = ""
    if confirmation != "ARM":
        print("Hardware startup cancelled; no SDK was imported and no interface was opened.")
        return 2

    try:
        import xrobotoolkit_sdk as xrt
    except ImportError as exc:
        raise RuntimeError("XRoboToolkit SDK is unavailable") from exc

    hardware: Optional[PiperHardware] = None
    xr_initialized = False
    try:
        hardware = PiperHardware(args.can_name, args.speed_percent)
        if not args.no_gripper:
            # Start every gripper-enabled teleoperation session from a known,
            # fully open state instead of inheriting the previous hardware pose.
            hardware.send_gripper(0.0)
            print(
                f"[gripper] startup full-open command sent "
                f"({GRIPPER_OPEN_UNITS} units)"
            )
        xrt.init()
        xr_initialized = True
        print(
            f"Live position control ready: {args.control_rate:g} Hz, "
            f"scale={args.position_scale:g}, "
            f"max_speed={'unlimited' if args.no_speed_limit else f'{args.max_speed:g} m/s'}, "
            f"speed_limit={'disabled' if args.no_speed_limit else 'enabled'}, "
            f"workspace_limit={'disabled' if args.no_workspace_limit else 'enabled'}, "
            f"speed_percent={args.speed_percent}, yaw={args.yaw_deg} deg, "
            f"controller={args.controller_hand}, "
            f"orientation={args.orientation_mode if args.follow_orientation else 'hold'}, "
            f"roll_scale={args.rotation_scale:g}, "
            f"roll_sign={args.wrist_roll_sign:+g}, "
            f"pitch_scale={args.wrist_pitch_scale:g}, "
            f"pitch_sign={args.wrist_pitch_sign:+g}, "
            f"orientation_range=+/-{args.max_orientation_delta_deg:g} deg, "
            f"gripper={'disabled' if args.no_gripper else ('binary' if args.binary_gripper else 'proportional')}"
        )

        if args.controller_hand == "left":
            get_grip = xrt.get_left_grip
            get_trigger = xrt.get_left_trigger
            get_controller_pose = xrt.get_left_controller_pose
        else:
            get_grip = xrt.get_right_grip
            get_trigger = xrt.get_right_trigger
            get_controller_pose = xrt.get_right_controller_pose

        period = 1.0 / args.control_rate
        next_tick = time.monotonic()
        last_cycle = next_tick
        last_xr_timestamp = None
        last_new_xr_time = next_tick
        grip_active = False
        activation_pending = False
        require_regrip = True
        stale_announced = False
        invalid_pose_announced = False
        valid_xr_frames = 0
        xr_reference_robot: Optional[np.ndarray] = None
        xr_reference_quat: Optional[np.ndarray] = None
        ee_reference: Optional[np.ndarray] = None
        held_orientation: Optional[Tuple[int, int, int]] = None
        ee_reference_rotation: Optional[Rotation] = None
        previous_command: Optional[np.ndarray] = None
        previous_trigger: Optional[float] = None
        last_gripper_send = 0.0
        overrun_count = 0

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
                # A clutch edge may arrive between XR pose frames. Consume it on
                # the next new timestamp instead of calibrating from repeated data.
                activation_pending = True

            if falling:
                activation_pending = False
                xr_reference_robot = None
                xr_reference_quat = None
                ee_reference = None
                held_orientation = None
                ee_reference_rotation = None
                previous_command = None
                print("[clutch] released; motion targets stopped")

            new_xr_pose = timestamp != last_xr_timestamp
            if new_xr_pose:
                last_xr_timestamp = timestamp
                last_new_xr_time = cycle_start
                stale_announced = False
                controller_pose = get_controller_pose()
                xr_position = np.asarray(controller_pose[:3], dtype=np.float64)
                xr_quaternion = normalize_xr_quaternion(controller_pose[3:7])
                orientation_invalid = args.follow_orientation and xr_quaternion is None
                if not is_valid_xr_position(xr_position) or orientation_invalid:
                    if not invalid_pose_announced:
                        print(
                            f"[XR ERROR] invalid/zero {args.controller_hand}-controller "
                            "pose/quaternion; "
                            "motion blocked until 10 consecutive valid frames"
                        )
                    invalid_pose_announced = True
                    valid_xr_frames = 0
                    require_regrip = True
                    activation_pending = False
                    xr_reference_robot = None
                    xr_reference_quat = None
                    ee_reference = None
                    held_orientation = None
                    ee_reference_rotation = None
                    previous_command = None
                    new_xr_pose = False
                else:
                    valid_xr_frames = min(
                        valid_xr_frames + 1, XR_VALID_FRAMES_REQUIRED
                    )
                    if valid_xr_frames >= XR_VALID_FRAMES_REQUIRED:
                        if require_regrip or invalid_pose_announced:
                            print(
                                "[XR READY] 10 consecutive valid frames; "
                                "held grip may activate immediately"
                            )
                        require_regrip = False
                        invalid_pose_announced = False
                        if grip_active and xr_reference_robot is None:
                            activation_pending = True
                    xr_robot = r_final @ xr_position

                if activation_pending and not require_regrip and new_xr_pose:
                    activation_pending = False
                    feedback = hardware.read_end_pose()
                    if feedback is None:
                        print(
                            "[PIPER ERROR] no complete end-pose feedback; "
                            "motion blocked until grip is released and pressed again"
                        )
                        require_regrip = True
                    else:
                        ee_reference, held_orientation = feedback
                        xr_reference_robot = xr_robot.copy()
                        if args.follow_orientation:
                            xr_reference_quat = xr_quaternion.copy()
                            ee_reference_rotation = Rotation.from_euler(
                                "xyz",
                                np.asarray(held_orientation, dtype=np.float64) / 1000.0,
                                degrees=True,
                            )
                        previous_command = ee_reference.copy()
                        hardware.confirm_movep()
                        # The rising-edge target is exactly actual feedback: no jump.
                        hardware.send_pose(previous_command, held_orientation)
                        print(
                            "[clutch] activated at actual EE "
                            f"{np.array2string(ee_reference, precision=4)}"
                        )
                elif (
                    new_xr_pose
                    and grip_active
                    and not require_regrip
                    and xr_reference_robot is not None
                    and ee_reference is not None
                    and held_orientation is not None
                    and previous_command is not None
                ):
                    relative = args.position_scale * (xr_robot - xr_reference_robot)
                    if args.safe_test:
                        relative = clamp_relative_radius(relative, SAFE_TEST_RADIUS_M)
                    raw_target = ee_reference + relative
                    if not args.no_workspace_limit:
                        workspace_min, workspace_max = workspace_bounds_for_reference(
                            ee_reference
                        )
                        raw_target = np.clip(
                            raw_target, workspace_min, workspace_max
                        )
                    if args.no_speed_limit:
                        target = raw_target
                    else:
                        target = limit_target_speed(
                            previous_command, raw_target, args.max_speed, dt
                        )
                    command_orientation = held_orientation
                    if args.follow_orientation:
                        if xr_reference_quat is None or ee_reference_rotation is None:
                            raise RuntimeError("orientation reference is unavailable")
                        try:
                            command_orientation = map_relative_orientation(
                                xr_reference_quat,
                            xr_quaternion,
                            ee_reference_rotation,
                            np.asarray(held_orientation, dtype=np.float64) / 1000.0,
                            r_final,
                            args.rotation_scale,
                            args.max_orientation_delta_deg,
                            args.orientation_mode,
                            args.wrist_roll_sign,
                            args.wrist_pitch_scale,
                            args.wrist_pitch_sign,
                        )
                        except ValueError as exc:
                            print(f"[ORIENTATION ERROR] {exc}; motion blocked")
                            require_regrip = True
                            xr_reference_robot = None
                            xr_reference_quat = None
                            ee_reference = None
                            held_orientation = None
                            ee_reference_rotation = None
                            previous_command = None
                            continue
                    if np.all(np.isfinite(target)):
                        hardware.send_pose(target, command_orientation)
                        previous_command = target
                    else:
                        print("[TARGET ERROR] non-finite target; motion blocked")
                        require_regrip = True
                        xr_reference_robot = None
                        xr_reference_quat = None
                        ee_reference = None
                        held_orientation = None
                        ee_reference_rotation = None
                        previous_command = None

            if cycle_start - last_new_xr_time > XR_STALE_SECONDS:
                if not stale_announced:
                    print(
                        "[XR STALE] timestamp unchanged for >0.2 s; "
                        "motion stopped; resumes after 10 valid frames"
                    )
                    stale_announced = True
                require_regrip = True
                activation_pending = False
                valid_xr_frames = 0
                xr_reference_robot = None
                xr_reference_quat = None
                ee_reference = None
                held_orientation = None
                ee_reference_rotation = None
                previous_command = None

            # Gripper may still follow a changed trigger when the XR pose timestamp
            # repeats. Refresh static values at most once per 0.2 s.
            if grip_active and not args.no_gripper:
                trigger = float(np.clip(trigger, 0.0, 1.0))
                if args.binary_gripper:
                    trigger = binary_gripper_trigger(previous_trigger, trigger)
                trigger_changed = (
                    previous_trigger is None or abs(trigger - previous_trigger) > 0.01
                )
                refresh_due = cycle_start - last_gripper_send >= 0.20
                if trigger_changed or refresh_due:
                    hardware.send_gripper(trigger)
                    previous_trigger = trigger
                    last_gripper_send = cycle_start

            remaining = next_tick - time.monotonic()
            if remaining > 0.0:
                time.sleep(remaining)
            else:
                overrun_count += 1
                next_tick = time.monotonic()
                if overrun_count % 100 == 1:
                    print(f"[timing] deadline overruns: {overrun_count}")
    except KeyboardInterrupt:
        print("\nStopping command updates. Arm remains enabled; no home/reset/disable sent.")
        return 0
    finally:
        if xr_initialized:
            xrt.close()


def main() -> int:
    args = parse_args()
    r_final = installation_rotation(args.yaw_deg) @ R_XR_TO_PIPER
    if args.hardware:
        return run_hardware(args, r_final)
    return run_dry_run(args, r_final)


if __name__ == "__main__":
    sys.exit(main())
