#!/usr/bin/env python3
"""
控制器数据实时监控工具
用于诊断哪些输入有数据
"""

import xrobotoolkit_sdk as xrt
import time

print("="*70)
print("  控制器数据实时监控")
print("="*70)
print("\n初始化SDK...")
xrt.init()
time.sleep(2)

print("\n开始监控，请操作控制器...\n")
print("按Ctrl+C停止\n")

try:
    for i in range(60):  # 监控60秒
        # 左手数据
        left_pose = xrt.get_left_controller_pose()
        left_grip = xrt.get_left_grip()
        left_trigger = xrt.get_left_trigger()

        # 右手数据
        right_pose = xrt.get_right_controller_pose()
        right_grip = xrt.get_right_grip()
        right_trigger = xrt.get_right_trigger()

        # 按钮
        a_btn = xrt.get_A_button()
        b_btn = xrt.get_B_button()
        x_btn = xrt.get_X_button()
        y_btn = xrt.get_Y_button()

        print(f"[{i+1}s]")
        print(f"  左手: 位置={left_pose[:3]}, 握把={left_grip:.2f}, 扳机={left_trigger:.2f}")
        print(f"  右手: 位置={right_pose[:3]}, 握把={right_grip:.2f}, 扳机={right_trigger:.2f}")
        print(f"  按钮: A={a_btn}, B={b_btn}, X={x_btn}, Y={y_btn}")

        # 标记有数据的输入
        active_inputs = []
        if left_grip > 0.01:
            active_inputs.append("左手握把")
        if right_grip > 0.01:
            active_inputs.append("右手握把")
        if left_trigger > 0.01:
            active_inputs.append("左扳机")
        if right_trigger > 0.01:
            active_inputs.append("右扳机")
        if any([a_btn, b_btn, x_btn, y_btn]):
            active_inputs.append("按钮")
        if abs(left_pose[0]) > 0.01 or abs(left_pose[1]) > 0.01 or abs(left_pose[2]) > 0.01:
            active_inputs.append("左手位置")
        if abs(right_pose[0]) > 0.01 or abs(right_pose[1]) > 0.01 or abs(right_pose[2]) > 0.01:
            active_inputs.append("右手位置")

        if active_inputs:
            print(f"  ✓ 活跃输入: {', '.join(active_inputs)}")
        else:
            print(f"  ⚠ 无活跃输入")

        print()
        time.sleep(1)

except KeyboardInterrupt:
    print("\n停止监控")

xrt.close()
print("\n完成")
