#!/usr/bin/env python3
"""
XRoboToolkit连接诊断工具
帮助排查Pico控制器数据为0的问题
"""

import time
import xrobotoolkit_sdk as xrt

print("="*70)
print("  XRoboToolkit 连接诊断")
print("="*70)

# 初始化
print("\n[步骤 1] 初始化SDK...")
xrt.init()
print("✓ SDK初始化成功")
time.sleep(2)  # 等待连接稳定

# 检查各种数据源
print("\n[步骤 2] 检查数据源...")

print("\n--- 控制器数据 ---")
for i in range(5):
    left_pose = xrt.get_left_controller_pose()
    right_pose = xrt.get_right_controller_pose()
    headset_pose = xrt.get_headset_pose()

    left_grip = xrt.get_left_grip()
    right_grip = xrt.get_right_grip()
    left_trigger = xrt.get_left_trigger()
    right_trigger = xrt.get_right_trigger()

    print(f"\n[{i+1}s]")
    print(f"  左手控制器位置: [{left_pose[0]:.3f}, {left_pose[1]:.3f}, {left_pose[2]:.3f}]")
    print(f"  右手控制器位置: [{right_pose[0]:.3f}, {right_pose[1]:.3f}, {right_pose[2]:.3f}]")
    print(f"  头显位置: [{headset_pose[0]:.3f}, {headset_pose[1]:.3f}, {headset_pose[2]:.3f}]")
    print(f"  左握把: {left_grip:.2f}, 右握把: {right_grip:.2f}")
    print(f"  左扳机: {left_trigger:.2f}, 右扳机: {right_trigger:.2f}")

    time.sleep(1)

# 检查按钮
print("\n--- 按钮状态 ---")
print("请按下控制器上的按钮进行测试...")
for i in range(5):
    a = xrt.get_A_button()
    b = xrt.get_B_button()
    x = xrt.get_X_button()
    y = xrt.get_Y_button()

    print(f"[{i+1}s] A:{a} B:{b} X:{x} Y:{y}")
    time.sleep(1)

# 检查手部追踪
print("\n--- 手部追踪 ---")
left_active = xrt.get_left_hand_is_active()
right_active = xrt.get_right_hand_is_active()
print(f"左手追踪: {'✓ 激活' if left_active else '✗ 未激活'}")
print(f"右手追踪: {'✓ 激活' if right_active else '✗ 未激活'}")

# 检查时间戳
print("\n--- 时间戳 ---")
for i in range(3):
    ts = xrt.get_time_stamp_ns()
    print(f"[{i+1}] 时间戳: {ts} ns")
    time.sleep(1)

# 诊断结果
print("\n" + "="*70)
print("  诊断结果")
print("="*70)

final_right_pose = xrt.get_right_controller_pose()
final_headset_pose = xrt.get_headset_pose()

if all(x == 0 for x in final_right_pose[:3]):
    print("\n✗ 问题确认: 控制器数据全为0")
    print("\n可能的原因和解决方案:")
    print("\n1. XRoboToolkit PC Service配置问题")
    print("   - 检查PC Service程序中是否显示设备已连接")
    print("   - 查看PC Service日志中是否有错误信息")
    print("   - 尝试在PC Service中重新扫描设备")

    print("\n2. Pico头显连接模式问题")
    print("   - 确认使用的是正确的连接模式（USB/WiFi）")
    print("   - 头显IP: 192.168.111.85")
    print("   - 尝试在PC Service设置中手动添加此IP")

    print("\n3. Pico头显设置问题")
    print("   - 打开Pico头显设置")
    print("   - 检查'开发者选项'是否已启用")
    print("   - 检查'USB调试'是否已启用")
    print("   - 重启头显后重试")

    print("\n4. PC Service版本问题")
    print("   - 确认PC Service版本与SDK版本兼容")
    print("   - 尝试更新到最新版本")

    print("\n5. 测试设备问题")
    print("   - 输出显示'TestDevice'可能是测试设备")
    print("   - 需要确保连接的是真实的Pico设备")
    print("   - 检查PC Service中的设备列表")

elif all(abs(x) < 0.001 for x in final_right_pose[:3]) and all(abs(x) < 0.001 for x in final_headset_pose[:3]):
    print("\n⚠ 数据接近0但不完全为0")
    print("   - 可能控制器和头显都放在原点附近")
    print("   - 尝试拿起控制器并移动")
    print("   - 戴上头显并四处看看")

else:
    print("\n✓ 控制器数据正常!")
    print(f"   右手控制器: {final_right_pose[:3]}")
    print(f"   头显: {final_headset_pose[:3]}")
    print("\n可以正常运行遥操作程序:")
    print("   python3 pico_teleop_piper.py")

# 清理
xrt.close()
print("\n诊断完成")
