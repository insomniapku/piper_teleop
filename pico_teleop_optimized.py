#!/usr/bin/env python3
"""
Piper机械臂Pico遥操作 - 低延迟优化版
基于 piper_teleop_latency 的优化技术

主要改进:
1. 速度限制 - 防止突然运动，减少追赶延迟
2. XR时间戳验证 - 检测数据停滞
3. 精确定时控制 - 稳定的50Hz控制频率
4. Clutch机制 - 激活时从实际位置开始，无跳变
5. 机械臂反馈读取 - 零延迟启动
6. 触发器防抖 - 减少总线负载
"""

import argparse
import sys
import time
from typing import Optional, Tuple

import numpy as np


# ========== 配置常量 ==========
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
PIPER_POSITION_UNITS_PER_METER = 1_000_000
GRIPPER_OPEN_UNITS = 70_000


# ========== 核心算法函数 ==========

def limit_target_speed(
    previous_cmd: np.ndarray,
    raw_target: np.ndarray,
    max_speed_mps: float,
    dt: float,
) -> np.ndarray:
    """
    限制目标速度，防止突然运动

    这是减少延迟的关键：限制单次移动距离，让机械臂能跟上
    """
    step = raw_target - previous_cmd
    max_step = max_speed_mps * dt
    step_norm = float(np.linalg.norm(step))
    if step_norm > max_step and step_norm > 0.0:
        step = step / step_norm * max_step
    return previous_cmd + step


def workspace_bounds_for_reference(reference_m: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
    """
    自适应工作空间边界

    如果机械臂在工作区外启动，允许在当前位置开始，避免初始跳变
    """
    reference_m = np.asarray(reference_m, dtype=np.float64)
    return (
        np.minimum(WORKSPACE_MIN_M, reference_m),
        np.maximum(WORKSPACE_MAX_M, reference_m),
    )


def update_grip_state(was_active: bool, grip_value: float) -> Tuple[bool, bool, bool]:
    """
    应用握把滞后，返回激活状态、上升沿、下降沿

    滞后防止在阈值附近抖动
    """
    active = was_active
    if not active and grip_value >= GRIP_ON_THRESHOLD:
        active = True
    elif active and grip_value <= GRIP_OFF_THRESHOLD:
        active = False
    return active, active and not was_active, was_active and not active


# ========== Piper硬件接口 ==========

class PiperHardware:
    """Piper机械臂硬件接口封装"""

    def __init__(self, can_name: str, speed_percent: int):
        try:
            from piper_sdk import C_PiperInterface_V2
        except ImportError as exc:
            raise RuntimeError(
                "Piper SDK未安装，请先安装 piper_sdk"
            ) from exc

        print(f"正在连接Piper机械臂 (CAN: {can_name})...")
        self.speed_percent = speed_percent
        self.piper = C_PiperInterface_V2(can_name=can_name, judge_flag=False)
        self.piper.ConnectPort()
        time.sleep(0.2)

        print("正在使能机械臂...")
        for _ in range(100):
            if self.piper.EnablePiper():
                break
            time.sleep(0.01)
        else:
            raise RuntimeError("机械臂使能失败，请检查连接")

        print("✓ 机械臂已使能")

        # 读取固件版本
        try:
            firmware = self.piper.GetPiperFirmwareVersion()
            print(f"✓ 固件版本: {firmware}")
        except:
            pass

        # MOVEP模式：在线点目标模式
        self.confirm_movep()

    def confirm_movep(self) -> None:
        """确认MOVEP运动模式"""
        self.piper.MotionCtrl_2(0x01, 0x00, self.speed_percent, 0x00)

    def read_end_pose(self) -> Optional[Tuple[np.ndarray, Tuple[int, int, int]]]:
        """
        读取实际末端位姿

        返回: (位置_米, 欧拉角_整数单位) 或 None
        """
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
        """发送位姿命令"""
        xyz = np.rint(position_m * PIPER_POSITION_UNITS_PER_METER).astype(int)
        self.piper.EndPoseCtrl(
            int(xyz[0]), int(xyz[1]), int(xyz[2]), *orientation
        )

    def send_gripper(self, trigger_value: float) -> None:
        """发送夹爪命令 (0=开, 1=闭)"""
        trigger_value = float(np.clip(trigger_value, 0.0, 1.0))
        gripper_position = int(round((1.0 - trigger_value) * GRIPPER_OPEN_UNITS))
        self.piper.GripperCtrl(gripper_position, 1000, 0x01, 0)


# ========== 优化的遥操作控制器 ==========

class OptimizedPiperTeleopController:
    """低延迟Piper遥操作控制器"""

    def __init__(
        self,
        can_name: str = "can0",
        control_rate: float = 50.0,
        position_scale: float = 1.0,
        max_speed: float = 0.08,
        speed_percent: int = 40,
        no_gripper: bool = False,
    ):
        self.control_rate = control_rate
        self.position_scale = position_scale
        self.max_speed = max_speed
        self.no_gripper = no_gripper

        # 初始化硬件
        self.hardware = PiperHardware(can_name, speed_percent)

        # VR空间映射参数
        self.vr_center = np.array([0.0, 0.0, 0.0])
        self.vr_range = np.array([0.4, 0.4, 0.4])  # ±40cm
        self.robot_center = np.array([0.225, 0.0, 0.225])
        self.robot_range = np.array([0.125, 0.20, 0.125])

        # 坐标转换矩阵
        self.r_final = R_XR_TO_PIPER

        # 控制状态
        self.grip_active = False
        self.activation_pending = False
        self.require_regrip = False
        self.xr_reference_robot: Optional[np.ndarray] = None
        self.ee_reference: Optional[np.ndarray] = None
        self.held_orientation: Optional[Tuple[int, int, int]] = None
        self.previous_command: Optional[np.ndarray] = None
        self.previous_trigger: Optional[float] = None

        # 时间戳管理
        self.last_xr_timestamp = None
        self.last_new_xr_time = 0.0
        self.stale_announced = False

        # 夹爪控制
        self.last_gripper_send = 0.0

        # 统计
        self.overrun_count = 0
        self.command_count = 0

        print("\n" + "="*70)
        print("  Piper低延迟遥操作控制器已就绪")
        print("="*70)
        print(f"控制频率: {control_rate} Hz")
        print(f"位置缩放: {position_scale}")
        print(f"最大速度: {max_speed} m/s")
        print(f"速度百分比: {speed_percent}%")
        print("="*70 + "\n")

    def map_vr_to_robot(self, vr_pos: np.ndarray) -> np.ndarray:
        """将VR控制器位置映射到机械臂工作空间"""
        # 标准化VR位置 (-1 to 1)
        vr_normalized = (vr_pos - self.vr_center) / self.vr_range
        vr_normalized = np.clip(vr_normalized, -1, 1)

        # 映射到机械臂空间
        robot_pos = self.robot_center + vr_normalized * self.robot_range

        return robot_pos

    def calibrate_vr_center(self, vr_pos: np.ndarray) -> None:
        """校准VR空间中心"""
        self.vr_center = np.array(vr_pos)
        print(f"\n[校准] VR中心已设置")
        print(f"       位置: [{self.vr_center[0]:.3f}, {self.vr_center[1]:.3f}, {self.vr_center[2]:.3f}]")
        print(f"       请在此位置周围 ±40cm 范围内移动控制器\n")

    def run(self):
        """主控制循环"""
        print("\n操作说明:")
        print("  - 握住右手控制器握把 (>80%): 激活控制")
        print("  - 首次握住会校准VR空间中心")
        print("  - 移动控制器: 机械臂跟随（有速度限制，更平滑）")
        print("  - 按右扳机: 控制夹爪")
        print("  - 松开握把: 停止控制")
        print("  - Ctrl+C: 退出程序")
        print("="*70 + "\n")

        # 初始化XR SDK
        try:
            import xrobotoolkit_sdk as xrt
        except ImportError as exc:
            raise RuntimeError("XRoboToolkit SDK未安装") from exc

        xrt.init()
        print("✓ XRoboToolkit SDK已初始化\n")

        period = 1.0 / self.control_rate
        next_tick = time.monotonic()
        last_cycle = next_tick
        calibrated = False

        try:
            while True:
                cycle_start = time.monotonic()
                next_tick += period
                dt = float(np.clip(cycle_start - last_cycle, 0.005, 0.05))
                last_cycle = cycle_start

                # 读取XR数据
                timestamp = xrt.get_time_stamp_ns()
                grip = float(xrt.get_right_grip())
                trigger = float(xrt.get_right_trigger())

                # 更新握把状态
                grip_active_prev = self.grip_active
                self.grip_active, rising, falling = update_grip_state(self.grip_active, grip)

                # 处理握把上升沿（刚刚握住）
                if rising and not self.require_regrip:
                    self.activation_pending = True
                    print("[握把] 检测到握住，等待稳定数据...")

                # 处理握把下降沿（松开）
                if falling:
                    self.activation_pending = False
                    self.xr_reference_robot = None
                    self.ee_reference = None
                    self.held_orientation = None
                    self.previous_command = None
                    self.require_regrip = False
                    calibrated = False
                    print("[握把] 松开，运动停止\n")

                # 检查XR数据新鲜度
                new_xr_pose = timestamp != self.last_xr_timestamp
                if new_xr_pose:
                    self.last_xr_timestamp = timestamp
                    self.last_new_xr_time = cycle_start
                    self.stale_announced = False
                    if not self.grip_active:
                        # 松开时观察到新数据，可以恢复
                        self.require_regrip = False

                    # 读取控制器位姿
                    controller_pose = xrt.get_right_controller_pose()
                    xr_position = np.asarray(controller_pose[:3], dtype=np.float64)

                    if xr_position.shape != (3,) or not np.all(np.isfinite(xr_position)):
                        print("[XR错误] 无效的控制器位置数据")
                        self.require_regrip = True
                        self.activation_pending = False
                        self.xr_reference_robot = None
                        new_xr_pose = False
                    else:
                        xr_robot = self.r_final @ xr_position

                    # 处理激活（首次握住后的新数据）
                    if self.activation_pending and not self.require_regrip and new_xr_pose:
                        self.activation_pending = False

                        # 首次校准
                        if not calibrated:
                            self.calibrate_vr_center(xr_position)
                            calibrated = True

                        # 读取机械臂实际位置
                        feedback = self.hardware.read_end_pose()
                        if feedback is None:
                            print("[Piper错误] 无法读取末端位姿，需要重新握住")
                            self.require_regrip = True
                        else:
                            self.ee_reference, self.held_orientation = feedback
                            self.xr_reference_robot = xr_robot.copy()
                            self.previous_command = self.ee_reference.copy()
                            self.hardware.confirm_movep()

                            # 发送首个目标 = 当前实际位置（无跳变）
                            self.hardware.send_pose(self.previous_command, self.held_orientation)
                            self.command_count = 0
                            print(f"[激活] 起始位置: [{self.ee_reference[0]:.3f}, "
                                  f"{self.ee_reference[1]:.3f}, {self.ee_reference[2]:.3f}] (无跳变)")

                    # 正常控制（握住且已激活）
                    elif (
                        self.grip_active
                        and not self.require_regrip
                        and self.xr_reference_robot is not None
                        and self.ee_reference is not None
                        and self.held_orientation is not None
                        and self.previous_command is not None
                    ):
                        # 计算相对运动
                        relative = self.position_scale * (xr_robot - self.xr_reference_robot)

                        # 映射到机械臂工作空间
                        workspace_min, workspace_max = workspace_bounds_for_reference(
                            self.ee_reference
                        )
                        raw_target = np.clip(
                            self.ee_reference + relative, workspace_min, workspace_max
                        )

                        # 应用速度限制（关键优化）
                        target = limit_target_speed(
                            self.previous_command, raw_target, self.max_speed, dt
                        )

                        if np.all(np.isfinite(target)):
                            self.hardware.send_pose(target, self.held_orientation)
                            self.previous_command = target
                            self.command_count += 1

                            # 定期打印状态
                            if self.command_count % 50 == 0:
                                print(f"[运行] 命令: {self.command_count:4d} | "
                                      f"位置: [{target[0]:.3f}, {target[1]:.3f}, {target[2]:.3f}]")
                        else:
                            print("[目标错误] 非有限值，停止运动")
                            self.require_regrip = True
                            self.xr_reference_robot = None

                # 检查XR数据停滞
                if cycle_start - self.last_new_xr_time > XR_STALE_SECONDS:
                    if not self.stale_announced:
                        print("\n[XR停滞] 数据超过0.2s未更新，停止运动，松开并重新握住恢复\n")
                        self.stale_announced = True
                    self.require_regrip = True
                    self.xr_reference_robot = None
                    self.ee_reference = None
                    self.held_orientation = None
                    self.previous_command = None

                # 夹爪控制（防抖）
                if self.grip_active and not self.no_gripper:
                    trigger = float(np.clip(trigger, 0.0, 1.0))
                    trigger_changed = (
                        self.previous_trigger is None
                        or abs(trigger - self.previous_trigger) > 0.01
                    )
                    refresh_due = cycle_start - self.last_gripper_send >= 0.20

                    if trigger_changed or refresh_due:
                        self.hardware.send_gripper(trigger)
                        self.previous_trigger = trigger
                        self.last_gripper_send = cycle_start

                # 精确定时
                remaining = next_tick - time.monotonic()
                if remaining > 0.0:
                    time.sleep(remaining)
                else:
                    self.overrun_count += 1
                    next_tick = time.monotonic()
                    if self.overrun_count % 100 == 1:
                        print(f"[定时] 延迟超限: {self.overrun_count}次")

        except KeyboardInterrupt:
            print("\n\n检测到Ctrl+C，正在退出...")
            print("机械臂保持使能状态，未执行归位/复位/失能")

        finally:
            xrt.close()
            print("\n程序已退出")


# ========== 主函数 ==========

def parse_args():
    parser = argparse.ArgumentParser(
        description="Piper低延迟Pico遥操作控制"
    )
    parser.add_argument("--can-name", default="can0", help="CAN接口名称")
    parser.add_argument("--control-rate", type=float, default=50.0, help="控制频率(Hz)")
    parser.add_argument("--position-scale", type=float, default=1.0, help="位置缩放因子")
    parser.add_argument("--max-speed", type=float, default=0.08, help="最大速度(m/s)")
    parser.add_argument("--speed-percent", type=int, default=40, help="机械臂速度百分比(1-100)")
    parser.add_argument("--no-gripper", action="store_true", help="禁用夹爪控制")
    parser.add_argument(
        "--safe-test",
        action="store_true",
        help="安全测试模式: scale=0.5, speed=0.03 m/s",
    )

    args = parser.parse_args()

    if args.control_rate <= 0.0:
        parser.error("--control-rate必须为正数")
    if args.position_scale <= 0.0:
        parser.error("--position-scale必须为正数")
    if args.max_speed <= 0.0:
        parser.error("--max-speed必须为正数")
    if not 1 <= args.speed_percent <= 100:
        parser.error("--speed-percent必须在[1, 100]范围内")

    if args.safe_test:
        args.position_scale = 0.5
        args.max_speed = 0.03
        print("\n⚠️  安全测试模式已启用 ⚠️\n")

    return args


def main():
    args = parse_args()

    print("="*70)
    print("  Piper机械臂 Pico遥操作系统 - 低延迟优化版")
    print("="*70)
    print("\n关键优化:")
    print("  ✓ 速度限制 - 平滑运动，减少追赶延迟")
    print("  ✓ XR时间戳验证 - 检测数据停滞")
    print("  ✓ 精确定时控制 - 稳定控制频率")
    print("  ✓ Clutch机制 - 无跳变激活")
    print("  ✓ 机械臂反馈 - 零延迟启动")
    print("  ✓ 触发器防抖 - 减少总线负载")
    print("="*70 + "\n")

    try:
        controller = OptimizedPiperTeleopController(
            can_name=args.can_name,
            control_rate=args.control_rate,
            position_scale=args.position_scale,
            max_speed=args.max_speed,
            speed_percent=args.speed_percent,
            no_gripper=args.no_gripper,
        )
        controller.run()
    except Exception as e:
        print(f"\n错误: {e}")
        import traceback
        traceback.print_exc()
        return 1

    return 0


if __name__ == "__main__":
    sys.exit(main())
