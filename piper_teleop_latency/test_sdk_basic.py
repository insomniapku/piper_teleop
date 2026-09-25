#!/usr/bin/env python3
"""
SDK基础功能测试
测试Piper SDK和XRoboToolkit SDK是否正常工作
"""

import sys
import time

print("="*70)
print("  SDK基础功能测试")
print("="*70)

# ============================================================================
# 1. 测试Piper SDK
# ============================================================================
print("\n[测试 1/2] Piper SDK")
print("-"*70)

try:
    from piper_sdk import C_PiperInterface_V2
    print("✓ Piper SDK导入成功")

    # 尝试连接
    print("尝试连接机械臂...")
    piper = C_PiperInterface_V2(can_name="can0", judge_flag=False)
    piper.ConnectPort()
    time.sleep(0.2)
    print("✓ 机械臂连接成功")

    # 读取固件版本
    try:
        firmware = piper.GetPiperFirmwareVersion()
        print(f"✓ 固件版本: {firmware}")
    except Exception as e:
        print(f"⚠ 无法读取固件版本: {e}")

    # 尝试使能
    print("尝试使能机械臂...")
    retry = 0
    enabled = False
    while not enabled and retry < 50:
        enabled = piper.EnablePiper()
        if not enabled:
            time.sleep(0.01)
            retry += 1

    if enabled:
        print("✓ 机械臂使能成功")

        # 读取状态
        print("\n当前机械臂状态:")
        ee_msg = piper.GetArmEndPoseMsgs()
        pose = ee_msg.end_pose
        print(f"  末端位置: X={pose.X_axis/1000:.1f}mm, Y={pose.Y_axis/1000:.1f}mm, Z={pose.Z_axis/1000:.1f}mm")

        try:
            joint_msg = piper.GetArmJointMsgs()
            # 不同SDK版本可能有不同的数据结构
            print(f"  关节状态: 已读取")
        except Exception as e:
            print(f"  关节状态: 无法读取详细信息")

    else:
        print("✗ 机械臂使能失败")
        print("  可能原因:")
        print("    - CAN连接问题")
        print("    - 机械臂未上电")
        print("    - 急停按钮被按下")

    piper_ok = enabled

except ImportError:
    print("✗ Piper SDK未安装")
    print("  安装方法: cd piper_sdk && pip install .")
    piper_ok = False
except Exception as e:
    print(f"✗ Piper SDK测试失败: {e}")
    import traceback
    traceback.print_exc()
    piper_ok = False

# ============================================================================
# 2. 测试XRoboToolkit SDK
# ============================================================================
print("\n[测试 2/2] XRoboToolkit SDK")
print("-"*70)

try:
    import xrobotoolkit_sdk as xrt
    print("✓ XRoboToolkit SDK导入成功")

    # 初始化
    print("初始化XR SDK...")
    xrt.init()
    print("✓ XR SDK初始化成功")

    # 读取控制器数据
    print("\n请操作Pico右手控制器...")
    print("  - 握住握把")
    print("  - 按扳机")
    print("  - 移动控制器")
    print("\n读取数据中 (5秒)...\n")

    for i in range(5):
        grip = xrt.get_right_grip()
        trigger = xrt.get_right_trigger()
        pose = xrt.get_right_controller_pose()

        print(f"  [{i+1}s] 握把={grip:.2f}, 扳机={trigger:.2f}, "
              f"位置=[{pose[0]:.3f}, {pose[1]:.3f}, {pose[2]:.3f}]")
        time.sleep(1)

    # 检查数据有效性
    final_pose = xrt.get_right_controller_pose()
    if all(x == 0 for x in final_pose[:3]):
        print("\n⚠ 警告: 控制器位置全为0")
        print("  可能原因:")
        print("    - Pico头显未连接")
        print("    - XRoboToolkit PC Service未运行")
        print("    - 控制器未开机或未配对")
        xr_ok = False
    else:
        print("\n✓ 控制器数据正常")
        xr_ok = True

    xrt.close()

except ImportError:
    print("✗ XRoboToolkit SDK未安装")
    print("  安装方法:")
    print("    1. 下载并运行 XRoboToolkit PC Service")
    print("    2. pip install xrobotoolkit-sdk")
    xr_ok = False
except Exception as e:
    print(f"✗ XRoboToolkit SDK测试失败: {e}")
    import traceback
    traceback.print_exc()
    xr_ok = False

# ============================================================================
# 总结
# ============================================================================
print("\n" + "="*70)
print("  测试总结")
print("="*70)

print(f"\nPiper SDK:         {'✓ 正常' if piper_ok else '✗ 异常'}")
print(f"XRoboToolkit SDK:  {'✓ 正常' if xr_ok else '✗ 异常'}")

if piper_ok and xr_ok:
    print("\n✓ 所有SDK测试通过，可以运行遥操作程序:")
    print("  python3 pico_teleop_piper.py")
    sys.exit(0)
else:
    print("\n✗ SDK测试未全部通过，请根据上述提示修复问题")
    sys.exit(1)
