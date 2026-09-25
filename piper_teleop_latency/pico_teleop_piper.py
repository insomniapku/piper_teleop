#!/usr/bin/env python3
"""
Piper机械臂Pico遥操作 - 完整可运行版本

直接运行此脚本，无需其他依赖（除了Piper SDK和XRoboToolkit SDK）

使用方法:
    python3 pico_teleop_piper.py

控制:
    - 握住右手控制器握把: 激活控制
    - 移动右手控制器: 机械臂跟随
    - 按右扳机: 关闭夹爪
    - 松开右扳机: 打开夹爪
    - B按钮: 紧急停止
"""

import time
import numpy as np
from scipy.spatial.transform import Rotation as R
import sys

# ============================================================================
# 检查和导入依赖
# ============================================================================

print("="*70)
print("  Piper机械臂 Pico遥操作系统")
print("="*70)

# 检查XRoboToolkit SDK
try:
    import xrobotoolkit_sdk as xrt
    print("✓ XRoboToolkit SDK已加载")
except ImportError:
    print("✗ 错误: XRoboToolkit SDK未安装")
    print("\n请安装XRoboToolkit SDK:")
    print("  1. 下载XRoboToolkit PC Service: https://github.com/XR-Robotics/XRoboToolkit-PC-Service")
    print("  2. 安装Python SDK: pip install xrobotoolkit-sdk")
    sys.exit(1)

# 检查Piper SDK
try:
    from piper_sdk import C_PiperInterface_V2
    print("✓ Piper SDK已加载")
except ImportError:
    print("✗ 错误: Piper SDK未安装")
    print("\n请安装Piper SDK:")
    print("  cd piper_sdk && pip install .")
    sys.exit(1)

print("="*70 + "\n")


# ============================================================================
# 简化的遥操作控制器（不依赖IK）
# ============================================================================

class SimplePiperTeleopController:
    """
    简化的Piper遥操作控制器
    使用关节空间直接控制，无需复杂的IK求解
    """

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
        self.piper.GripperCtrl(0, 1000, 0x02, 0)  # 回零
        time.sleep(0.5)
        self.piper.GripperCtrl(0, 1000, 0x01, 0)  # 使能
        time.sleep(0.5)
        print("✓ 夹爪已配置")

        # 移动到初始位置
        print("移动到初始位置...")
        self.move_to_home()
        print("✓ 准备就绪")

        # 状态变量
        self.reference_set = False
        self.xr_ref_pos = None
        self.ee_ref_pos = None

        # 控制参数
        self.position_scale = 3.5  # 位置缩放（提高灵敏度：2.0→3.5）
        self.max_delta = 0.08      # 最大单步移动(m)（提高响应：0.05→0.08）

        # 统计
        self.control_count = 0
        self.last_target_pos = None

    def move_to_home(self):
        """移动到初始位置"""
        # 设置到一个安全的初始姿态
        self.piper.MotionCtrl_2(0x01, 0x00, 30, 0x00)  # MOVEP模式，30%速度
        # 目标: X=150mm, Y=0mm, Z=200mm
        self.piper.EndPoseCtrl(150000, 0, 200000, -179900, 0, -179900)
        time.sleep(3)  # 等待移动完成

    def get_current_ee_position(self):
        """获取当前末端位置（米）"""
        ee_msg = self.piper.GetArmEndPoseMsgs()
        pose = ee_msg.end_pose

        # 微米转米
        return np.array([
            pose.X_axis / 1000000.0,
            pose.Y_axis / 1000000.0,
            pose.Z_axis / 1000000.0
        ])

    def set_reference(self, xr_pos):
        """设置引用位置"""
        self.xr_ref_pos = np.array(xr_pos)
        self.ee_ref_pos = self.get_current_ee_position()
        self.reference_set = True
        print(f"\n[引用设置] XR: {self.xr_ref_pos}, EE: {self.ee_ref_pos}")

    def calc_target_position(self, xr_pos):
        """计算目标末端位置"""
        # 确保是numpy数组
        xr_pos = np.array(xr_pos)

        # Delta映射
        delta = (xr_pos - self.xr_ref_pos) * self.position_scale

        # 限制最大变化
        delta_norm = np.linalg.norm(delta)
        if delta_norm > self.max_delta:
            delta = delta / delta_norm * self.max_delta

        # 计算目标
        target_pos = self.ee_ref_pos + delta

        # 工作空间限制
        target_pos[0] = np.clip(target_pos[0], 0.10, 0.35)  # X: 100-350mm
        target_pos[1] = np.clip(target_pos[1], -0.20, 0.20)  # Y: -200-200mm
        target_pos[2] = np.clip(target_pos[2], 0.10, 0.35)   # Z: 100-350mm

        return target_pos

    def move_to_position(self, target_pos):
        """移动到目标位置（笛卡尔空间）"""
        # 转换单位: 米 → 微米
        x = int(target_pos[0] * 1000000)
        y = int(target_pos[1] * 1000000)
        z = int(target_pos[2] * 1000000)

        # 使用MOVEL模式进行直线运动
        self.piper.MotionCtrl_2(0x01, 0x02, 50, 0x00)
        self.piper.EndPoseCtrl(x, y, z, -179900, 0, -179900)

    def control_gripper(self, trigger_value):
        """控制夹爪"""
        # trigger_value: 0.0 (松开) -> 1.0 (按下)
        # 夹爪: 70mm (打开) -> 0mm (关闭)
        gripper_pos = int((1.0 - trigger_value) * 70000)
        self.piper.GripperCtrl(gripper_pos, 1000, 0x01, 0)

    def run(self):
        """主控制循环"""
        print("\n" + "="*70)
        print("  开始遥操作")
        print("="*70)
        print("操作说明:")
        print("  - 握住右手控制器握把: 激活机械臂控制")
        print("  - 移动右手控制器: 机械臂末端跟随")
        print("  - 按右扳机: 关闭夹爪")
        print("  - 松开右扳机: 打开夹爪")
        print("  - B按钮: 紧急停止")
        print("  - Ctrl+C: 退出程序")
        print("="*70 + "\n")

        # 初始化XR SDK
        xrt.init()
        print("XRoboToolkit SDK已初始化\n")

        last_print_time = time.time()
        emergency_stop = False

        try:
            while not emergency_stop:
                # 读取控制器数据
                grip = xrt.get_right_grip()
                trigger = xrt.get_right_trigger()
                b_button = xrt.get_B_button()

                # 检查紧急停止
                if b_button:
                    print("\n[紧急停止] B按钮被按下")
                    emergency_stop = True
                    break

                # 控制器位姿 [x, y, z, qx, qy, qz, qw]
                controller_pose = xrt.get_right_controller_pose()
                xr_pos = np.array(controller_pose[:3])

                # 检查激活状态
                if grip > 0.5:  # 握把被按下
                    self.control_count += 1

                    # 首次激活
                    if not self.reference_set:
                        self.set_reference(xr_pos)

                    # 计算目标位置
                    target_pos = self.calc_target_position(xr_pos)

                    # 移动机械臂
                    self.move_to_position(target_pos)

                    # 控制夹爪
                    self.control_gripper(trigger)

                    # 定期打印状态
                    if time.time() - last_print_time > 2.0:
                        print(f"[运行] 控制次数: {self.control_count}, "
                              f"目标位置: [{target_pos[0]:.3f}, {target_pos[1]:.3f}, {target_pos[2]:.3f}]")
                        last_print_time = time.time()
                else:
                    # 松开握把
                    if self.reference_set:
                        print("\n[停止] 握把松开，等待重新激活...\n")
                        self.reference_set = False
                        self.control_count = 0

                # 控制频率
                time.sleep(0.01)  # 100Hz

        except KeyboardInterrupt:
            print("\n\n检测到Ctrl+C，正在退出...")

        # 清理
        xrt.close()
        print("\n程序已退出")
        print(f"总控制次数: {self.control_count}")


# ============================================================================
# 主程序
# ============================================================================

def main():
    """主函数"""
    try:
        # 创建控制器
        controller = SimplePiperTeleopController(can_name="can0")

        # 运行控制循环
        controller.run()

    except Exception as e:
        print(f"\n错误: {e}")
        import traceback
        traceback.print_exc()
        return 1

    return 0


if __name__ == "__main__":
    sys.exit(main())
