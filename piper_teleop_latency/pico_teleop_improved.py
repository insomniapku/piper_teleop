#!/usr/bin/env python3
"""
Piper机械臂Pico遥操作 - 改进版（绝对位置映射 + 双手控制）

改进点:
1. 绝对位置映射 - 控制器位置直接对应机械臂位置
2. 双手控制 - 左手和右手都可以控制
3. 可视化映射区域 - 打印当前映射关系
"""

import time
import numpy as np
from scipy.spatial.transform import Rotation as R
import sys

print("="*70)
print("  Piper机械臂 Pico遥操作系统 - 改进版")
print("="*70)

# 检查依赖
try:
    import xrobotoolkit_sdk as xrt
    print("✓ XRoboToolkit SDK已加载")
except ImportError:
    print("✗ 错误: XRoboToolkit SDK未安装")
    sys.exit(1)

try:
    from piper_sdk import C_PiperInterface_V2
    print("✓ Piper SDK已加载")
except ImportError:
    print("✗ 错误: Piper SDK未安装")
    sys.exit(1)

print("="*70 + "\n")


class ImprovedPiperTeleopController:
    """改进的Piper遥操作控制器 - 绝对位置映射"""

    def __init__(self, can_name="can0"):
        print("初始化Piper机械臂连接...")

        # 初始化Piper SDK
        self.piper = C_PiperInterface_V2(can_name=can_name, judge_flag=False)
        self.piper.ConnectPort()
        time.sleep(0.2)

        # 使能机械臂
        print("使能机械臂...")
        retry = 0
        while not self.piper.EnablePiper() and retry < 100:
            time.sleep(0.01)
            retry += 1

        if retry >= 100:
            raise RuntimeError("无法使能机械臂，请检查连接")

        print("✓ 机械臂已使能")

        # 读取固件版本
        firmware = self.piper.GetPiperFirmwareVersion()
        print(f"✓ 固件版本: {firmware}")

        # 配置夹爪
        print("配置夹爪...")
        self.piper.GripperCtrl(0, 1000, 0x02, 0)
        time.sleep(0.5)
        self.piper.GripperCtrl(0, 1000, 0x01, 0)
        time.sleep(0.5)
        print("✓ 夹爪已配置")

        # 移动到初始位置
        print("移动到初始位置...")
        self.move_to_home()
        print("✓ 准备就绪")

        # 控制模式：'right' 或 'left'
        self.active_hand = None

        # 映射参数 - VR空间到机械臂工作空间
        # VR坐标范围 (根据实际测量调整)
        self.vr_center = np.array([0.0, 0.0, 0.0])  # VR空间中心
        self.vr_range = np.array([0.4, 0.4, 0.4])   # VR工作范围 (±40cm)

        # 机械臂工作空间
        self.robot_center = np.array([0.225, 0.0, 0.225])  # 机械臂工作中心
        self.robot_range = np.array([0.125, 0.20, 0.125])  # 机械臂工作范围

        # 工作空间限制
        self.x_limit = (0.10, 0.35)
        self.y_limit = (-0.20, 0.20)
        self.z_limit = (0.10, 0.35)

        # 统计
        self.control_count = 0
        self.last_print_time = time.time()

    def move_to_home(self):
        """移动到初始位置"""
        self.piper.MotionCtrl_2(0x01, 0x00, 30, 0x00)
        self.piper.EndPoseCtrl(150000, 0, 200000, -179900, 0, -179900)
        time.sleep(3)

    def map_vr_to_robot(self, vr_pos):
        """
        将VR控制器位置映射到机械臂工作空间
        使用线性映射: VR空间 → 机械臂空间
        """
        # 标准化VR位置 (-1 to 1)
        vr_normalized = (vr_pos - self.vr_center) / self.vr_range

        # 裁剪到 [-1, 1]
        vr_normalized = np.clip(vr_normalized, -1, 1)

        # 映射到机械臂空间
        robot_pos = self.robot_center + vr_normalized * self.robot_range

        # 应用工作空间限制
        robot_pos[0] = np.clip(robot_pos[0], self.x_limit[0], self.x_limit[1])
        robot_pos[1] = np.clip(robot_pos[1], self.y_limit[0], self.y_limit[1])
        robot_pos[2] = np.clip(robot_pos[2], self.z_limit[0], self.z_limit[1])

        return robot_pos

    def calibrate_vr_center(self, vr_pos):
        """校准VR空间中心"""
        self.vr_center = np.array(vr_pos)
        print(f"\n[校准] VR中心已设置: {self.vr_center}")
        print(f"       请在此位置周围 ±40cm 范围内移动控制器")

    def move_to_position(self, target_pos):
        """移动到目标位置"""
        x = int(target_pos[0] * 1000000)
        y = int(target_pos[1] * 1000000)
        z = int(target_pos[2] * 1000000)

        self.piper.MotionCtrl_2(0x01, 0x02, 50, 0x00)
        self.piper.EndPoseCtrl(x, y, z, -179900, 0, -179900)

    def control_gripper(self, trigger_value):
        """控制夹爪"""
        gripper_pos = int((1.0 - trigger_value) * 70000)
        self.piper.GripperCtrl(gripper_pos, 1000, 0x01, 0)

    def run(self):
        """主控制循环"""
        print("\n" + "="*70)
        print("  开始遥操作 - 改进模式")
        print("="*70)
        print("操作说明:")
        print("  - 握住左手或右手控制器握把: 激活控制")
        print("  - 首次握住会设置VR空间中心（校准）")
        print("  - 移动控制器: 机械臂直接跟随到对应位置")
        print("  - 按对应扳机: 控制夹爪")
        print("  - B按钮: 紧急停止")
        print("  - Ctrl+C: 退出程序")
        print("="*70 + "\n")

        # 初始化XR SDK
        xrt.init()
        print("XRoboToolkit SDK已初始化\n")

        calibrated = False
        emergency_stop = False

        try:
            while not emergency_stop:
                # 读取双手数据
                left_grip = xrt.get_left_grip()
                right_grip = xrt.get_right_grip()
                left_trigger = xrt.get_left_trigger()
                right_trigger = xrt.get_right_trigger()
                b_button = xrt.get_B_button()

                # 检查紧急停止
                if b_button:
                    print("\n[紧急停止] B按钮被按下")
                    emergency_stop = True
                    break

                # 获取控制器位姿
                left_pose = xrt.get_left_controller_pose()
                right_pose = xrt.get_right_controller_pose()

                left_pos = np.array(left_pose[:3])
                right_pos = np.array(right_pose[:3])

                # 确定激活的手
                active = None
                active_pos = None
                active_trigger = 0.0

                if left_grip > 0.5:
                    active = 'left'
                    active_pos = left_pos
                    active_trigger = left_trigger
                elif right_grip > 0.5:
                    active = 'right'
                    active_pos = right_pos
                    active_trigger = right_trigger

                # 处理激活状态
                if active:
                    # 首次激活：校准VR中心
                    if not calibrated:
                        self.calibrate_vr_center(active_pos)
                        calibrated = True
                        time.sleep(0.5)
                        continue

                    # 切换手
                    if self.active_hand != active:
                        print(f"\n[切换] 使用{active}手控制")
                        self.active_hand = active

                    self.control_count += 1

                    # 映射VR位置到机械臂位置
                    target_pos = self.map_vr_to_robot(active_pos)

                    # 移动机械臂
                    self.move_to_position(target_pos)

                    # 控制夹爪
                    self.control_gripper(active_trigger)

                    # 定期打印状态
                    if time.time() - self.last_print_time > 1.0:
                        vr_offset = active_pos - self.vr_center
                        print(f"[{active}手] 控制次数: {self.control_count:4d} | "
                              f"VR偏移: [{vr_offset[0]:+.3f}, {vr_offset[1]:+.3f}, {vr_offset[2]:+.3f}] | "
                              f"机械臂: [{target_pos[0]:.3f}, {target_pos[1]:.3f}, {target_pos[2]:.3f}] | "
                              f"扳机: {active_trigger:.2f}")
                        self.last_print_time = time.time()
                else:
                    # 松开握把
                    if self.active_hand is not None:
                        print(f"\n[停止] {self.active_hand}手松开，等待重新激活...\n")
                        self.active_hand = None
                        self.control_count = 0

                # 控制频率
                time.sleep(0.01)  # 100Hz

        except KeyboardInterrupt:
            print("\n\n检测到Ctrl+C，正在退出...")

        # 清理
        xrt.close()
        print("\n程序已退出")


def main():
    """主函数"""
    try:
        controller = ImprovedPiperTeleopController(can_name="can0")
        controller.run()
    except Exception as e:
        print(f"\n错误: {e}")
        import traceback
        traceback.print_exc()
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
