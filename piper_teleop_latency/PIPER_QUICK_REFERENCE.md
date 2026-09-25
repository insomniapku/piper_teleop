# Piper机械臂SDK快速参考

## 测试结果总结

✅ **已验证功能**：
- 固件版本读取：S-V1.8-9
- CAN通信：200Hz，双CAN模块
- 夹爪控制：开/关/半开功能正常
- 关节运动：6轴关节控制正常
- 笛卡尔运动：直线运动(MOVEL)和点到点(MOVEP)正常

## 快速开始

```bash
# 1. 激活虚拟环境
source piper_venv/bin/activate

# 2. 激活CAN模块（如果有多个CAN模块，需要指定USB端口）
cd piper_sdk/piper_sdk
bash can_activate.sh can0 1000000 "1-11.2:1.0"

# 3. 运行测试脚本
cd ~/lcz0820
python test_piper_complete_demo.py
```

## 核心API参考

### 1. 初始化和连接

```python
from piper_sdk import *

# 创建接口实例
piper = C_PiperInterface_V2(can_name="can0", judge_flag=False)
piper.ConnectPort()

# 使能机械臂
while not piper.EnablePiper():
    time.sleep(0.01)
```

### 2. 运动控制模式

```python
# MotionCtrl_2(enable, mode, speed, coord_mode)
# enable: 0x01 使能
# mode: 
#   0x00 = MOVEP (点到点运动，关节插值)
#   0x01 = 位置控制模式
#   0x02 = MOVEL (直线运动，笛卡尔插值)
# speed: 0-100 (速度百分比)
# coord_mode: 0x00 关节空间, 0x01 笛卡尔空间

piper.MotionCtrl_2(0x01, 0x01, 50, 0x00)
```

### 3. 关节控制

```python
# 单位：毫度 (1000*degree)
# 弧度转毫度：factor = 57295.7795 (1000*180/π)

factor = 57295.7795
position_rad = [0.2, 0.2, -0.2, 0.3, -0.2, 0.5]  # 弧度
joints = [round(p * factor) for p in position_rad]

piper.MotionCtrl_2(0x01, 0x01, 50, 0x00)
piper.JointCtrl(joints[0], joints[1], joints[2], joints[3], joints[4], joints[5])
```

### 4. 笛卡尔空间控制

```python
# 单位：微米 (0.001mm)
# X, Y, Z: 位置 (微米)
# RX, RY, RZ: 姿态 (毫度)

# MOVEP模式 - 点到点
piper.MotionCtrl_2(0x01, 0x00, 50, 0x00)
piper.EndPoseCtrl(150000, -50000, 150000, -179900, 0, -179900)

# MOVEL模式 - 直线运动
piper.MotionCtrl_2(0x01, 0x02, 50, 0x00)
piper.EndPoseCtrl(250000, 50000, 150000, -179900, 0, -179900)
```

### 5. 夹爪控制

```python
# GripperCtrl(position, speed, mode, reserved)
# position: 位置 (微米, 0-70000)
# speed: 速度 (通常1000)
# mode: 
#   0x01 = 位置控制
#   0x02 = 回零
# reserved: 保留参数 (通常0)

# 回零并使能
piper.GripperCtrl(0, 1000, 0x02, 0)  # 回零
time.sleep(1)
piper.GripperCtrl(0, 1000, 0x01, 0)  # 使能
time.sleep(1)

# 控制夹爪位置
piper.GripperCtrl(50000, 1000, 0x01, 0)  # 打开到50mm
piper.GripperCtrl(0, 1000, 0x01, 0)      # 关闭
piper.GripperCtrl(25000, 1000, 0x01, 0)  # 半开到25mm
```

### 6. 状态读取

```python
# 读取关节角度
joint_msg = piper.GetArmJointMsgs()
print(joint_msg.joint_state)  # [j1, j2, j3, j4, j5, j6]

# 读取末端位姿
end_pose_msg = piper.GetArmEndPoseMsgs()
pose = end_pose_msg.end_pose
print(f"X: {pose.X_axis/1000} mm")  # 微米转毫米
print(f"Y: {pose.Y_axis/1000} mm")
print(f"Z: {pose.Z_axis/1000} mm")
print(f"RX: {pose.RX_axis/1000} 度")  # 毫度转度
print(f"RY: {pose.RY_axis/1000} 度")
print(f"RZ: {pose.RZ_axis/1000} 度")

# 读取夹爪状态
gripper_msg = piper.GetArmGripperMsgs()
print(f"位置: {gripper_msg.grippers_angle/1000} mm")
print(f"力量: {gripper_msg.grippers_effort/1000}")

# 读取机械臂状态
status = piper.GetArmStatus()
print(status)

# 读取固件版本
firmware = piper.GetPiperFirmwareVersion()
print(firmware)
```

## 单位转换参考

| 参数 | SDK单位 | 实际单位 | 转换因子 |
|------|---------|----------|----------|
| 位置 (X,Y,Z) | 微米 (μm) | 毫米 (mm) | ÷1000 |
| 角度 (关节、姿态) | 毫度 | 度 (°) | ÷1000 |
| 弧度 → 毫度 | - | - | ×57295.7795 |
| 夹爪位置 | 微米 (μm) | 毫米 (mm) | ÷1000 |

## 常用坐标范围

- **X轴**: 约 -300mm ~ 300mm
- **Y轴**: 约 -300mm ~ 300mm
- **Z轴**: 约 100mm ~ 400mm
- **夹爪**: 0mm ~ 70mm

## 可用测试脚本

| 脚本 | 功能 |
|------|------|
| `test_piper_basic.py` | 基础测试：读取版本和关节角度 |
| `test_piper_switch_slave.py` | 切换到从臂模式 |
| `test_piper_tcp_pose.py` | 实时显示末端位姿 |
| `test_piper_gripper.py` | 夹爪控制演示 |
| `test_piper_joint_control.py` | 关节运动控制演示 |
| `test_piper_cartesian.py` | 笛卡尔空间运动演示 |
| `test_piper_complete_demo.py` | 完整功能演示 |

## 注意事项

1. **安全第一**：运行脚本前确保机械臂周围没有障碍物
2. **速度控制**：初次测试建议使用较低速度（30-50%）
3. **运动范围**：确保目标位置在机械臂工作空间内
4. **CAN模块**：多个CAN模块时必须指定USB端口硬件地址
5. **使能要求**：所有运动控制前必须先使能机械臂

## 错误处理

### CAN模块未激活
```bash
# 检查CAN接口
ifconfig

# 重新激活
cd piper_sdk/piper_sdk
bash can_activate.sh can0 1000000 "1-11.2:1.0"
```

### 机械臂无法使能
- 检查从臂模式是否设置
- 确认机械臂已上电
- 检查CAN连接

### 运动指令无响应
- 确认已调用 `MotionCtrl_2` 设置运动模式
- 检查目标位置是否在工作空间内
- 查看机械臂状态 `GetArmStatus()`

## 更多资源

- SDK源码: `piper_sdk/piper_sdk/`
- 官方Demo: `piper_sdk/piper_sdk/demo/V2/`
- 接口文档: `piper_sdk/asserts/V2/INTERFACE_V2.MD`
- 中文文档: `piper_sdk/README(ZH).MD`
- GitHub: https://github.com/agilexrobotics/piper_sdk
