#!/usr/bin/env python3
"""Low-latency, position-only Pico XR teleoperation for a Piper arm.

The default execution path is a short, hardware-free dry-run. Real XR/CAN
interfaces are imported and opened only with ``--hardware`` and an explicit
``ARM`` confirmation.
"""

import argparse
import math
import sys
import time
from typing import Optional, Tuple

import numpy as np


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
SAFE_TEST_RADIUS_M = 0.03
PIPER_POSITION_UNITS_PER_METER = 1_000_000
GRIPPER_OPEN_UNITS = 70_000


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


def update_grip_state(was_active: bool, grip_value: float) -> Tuple[bool, bool, bool]:
    """Apply grip hysteresis and return active, rising, and falling states."""
    active = was_active
    if not active and grip_value >= GRIP_ON_THRESHOLD:
        active = True
    elif active and grip_value <= GRIP_OFF_THRESHOLD:
        active = False
    return active, active and not was_active, was_active and not active


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
    parser.add_argument("--speed-percent", type=int, default=40)
    parser.add_argument("--yaw-deg", type=int, choices=(0, 90, -90, 180), default=0)
    parser.add_argument(
        "--safe-test",
        action="store_true",
        help="force scale=0.5, speed=0.03 m/s, and a 3 cm clutch radius",
    )
    parser.add_argument("--no-gripper", action="store_true")
    args = parser.parse_args()

    if args.control_rate <= 0.0:
        parser.error("--control-rate must be positive")
    if args.position_scale <= 0.0:
        parser.error("--position-scale must be positive")
    if args.max_speed <= 0.0:
        parser.error("--max-speed must be positive")
    if not 1 <= args.speed_percent <= 100:
        parser.error("--speed-percent must be in [1, 100]")

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
            raw_target = np.clip(
                ee_reference + relative, WORKSPACE_MIN_M, WORKSPACE_MAX_M
            )
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
    passed = all(
        (mapping_non_identity, first_target_ok, speed_limit_observed, finite_targets, rate_ok)
    )
    print(
        "DRY-RUN SUMMARY: "
        f"mapping_non_identity={mapping_non_identity}, "
        f"first_target_no_jump={first_target_ok}, "
        f"speed_limit_observed={speed_limit_observed}, finite={finite_targets}, "
        f"target_rate={measured_rate:.1f}Hz, overruns={overrun_count}, "
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
        xrt.init()
        xr_initialized = True
        print(
            f"Live position control ready: {args.control_rate:g} Hz, "
            f"scale={args.position_scale:g}, max_speed={args.max_speed:g} m/s, "
            f"yaw={args.yaw_deg} deg"
        )

        period = 1.0 / args.control_rate
        next_tick = time.monotonic()
        last_cycle = next_tick
        last_xr_timestamp = None
        last_new_xr_time = next_tick
        grip_active = False
        activation_pending = False
        require_regrip = False
        stale_announced = False
        xr_reference_robot: Optional[np.ndarray] = None
        ee_reference: Optional[np.ndarray] = None
        held_orientation: Optional[Tuple[int, int, int]] = None
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
            grip = float(xrt.get_right_grip())
            trigger = float(xrt.get_right_trigger())
            grip_active, rising, falling = update_grip_state(grip_active, grip)

            if rising and not require_regrip:
                # A clutch edge may arrive between XR pose frames. Consume it on
                # the next new timestamp instead of calibrating from repeated data.
                activation_pending = True

            if falling:
                activation_pending = False
                xr_reference_robot = None
                ee_reference = None
                held_orientation = None
                previous_command = None
                require_regrip = False
                print("[clutch] released; motion targets stopped")

            new_xr_pose = timestamp != last_xr_timestamp
            if new_xr_pose:
                last_xr_timestamp = timestamp
                last_new_xr_time = cycle_start
                stale_announced = False
                if not grip_active:
                    # Fresh data observed while released: the next rising edge may
                    # recover from a prior stale-frame latch.
                    require_regrip = False
                controller_pose = xrt.get_right_controller_pose()
                xr_position = np.asarray(controller_pose[:3], dtype=np.float64)
                if xr_position.shape != (3,) or not np.all(np.isfinite(xr_position)):
                    print("[XR ERROR] invalid right-controller position; target not sent")
                    require_regrip = True
                    activation_pending = False
                    xr_reference_robot = None
                    new_xr_pose = False
                else:
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
                        previous_command = ee_reference.copy()
                        hardware.confirm_movep()
                        # The rising-edge target is exactly actual feedback: no jump.
                        hardware.send_pose(previous_command, held_orientation)
                        print(
                            "[clutch] activated at actual EE "
                            f"{np.array2string(ee_reference, precision=4)}"
                        )
                elif (
                    grip_active
                    and not require_regrip
                    and xr_reference_robot is not None
                    and ee_reference is not None
                    and held_orientation is not None
                    and previous_command is not None
                ):
                    relative = args.position_scale * (xr_robot - xr_reference_robot)
                    if args.safe_test:
                        relative = clamp_relative_radius(relative, SAFE_TEST_RADIUS_M)
                    raw_target = np.clip(
                        ee_reference + relative, WORKSPACE_MIN_M, WORKSPACE_MAX_M
                    )
                    target = limit_target_speed(
                        previous_command, raw_target, args.max_speed, dt
                    )
                    if np.all(np.isfinite(target)):
                        hardware.send_pose(target, held_orientation)
                        previous_command = target
                    else:
                        print("[TARGET ERROR] non-finite target; motion blocked")
                        require_regrip = True
                        xr_reference_robot = None

            if cycle_start - last_new_xr_time > XR_STALE_SECONDS:
                if not stale_announced:
                    print(
                        "[XR STALE] timestamp unchanged for >0.2 s; "
                        "motion stopped, release and press grip to recover"
                    )
                    stale_announced = True
                require_regrip = True
                xr_reference_robot = None
                ee_reference = None
                held_orientation = None
                previous_command = None

            # Gripper may still follow a changed trigger when the XR pose timestamp
            # repeats. Refresh static values at most once per 0.2 s.
            if grip_active and not args.no_gripper:
                trigger = float(np.clip(trigger, 0.0, 1.0))
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
