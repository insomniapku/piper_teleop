# Piper机械臂Pico遥操作集成指南

## 概述

基于XRoboToolkit SDK，我们可以实现用Pico头显控制Piper机械臂的完整遥操作系统。

## 架构设计

```
Pico头显 → XRoboToolkit PC Service → Python SDK → Placo IK → Piper SDK → 机械臂
   ↓              ↓                      ↓           ↓          ↓
控制器位姿      USB连接              遥操作映射    关节角度   CAN通信
```

## 核心组件

### 1. XRoboToolkit PC Service（必需）
- **下载**: https://github.com/XR-Robotics/XRoboToolkit-PC-Service
- **功能**: 连接Pico头显，通过USB传输数据
- **安装**: Windows或Linux PC上运行

### 2. XrClient - Pico数据接口
```python
from xrobotoolkit_teleop.common.xr_client import XrClient

# 初始化客户端
xr_client = XrClient()

# 获取控制器位姿 [x, y, z, qx, qy, qz, qw]
right_pose = xr_client.get_pose_by_name("right_controller")
left_pose = xr_client.get_pose_by_name("left_controller")

# 获取握把/扳机值 (0.0-1.0)
right_grip = xr_client.get_key_value_by_name("right_grip")
right_trigger = xr_client.get_key_value_by_name("right_trigger")

# 获取按钮状态 (True/False)
b_button = xr_client.get_button_state_by_name("B")
```

### 3. 控制映射

**按钮功能**:
- **右握把 (Right Grip)**: 激活机械臂控制
- **右扳机 (Right Trigger)**: 控制夹爪开合（0.0=打开，1.0=关闭）
- **B按钮**: 开始/停止数据记录
- **右摇杆点击**: 紧急停止

**位姿映射**:
- 控制器位置 → 机械臂末端位置（通过delta映射）
- 控制器姿态 → 机械臂末端姿态

## 实现步骤

### 步骤1: 安装XRoboToolkit SDK

```bash
# 下载并安装XRoboToolkit PC Service
# https://github.com/XR-Robotics/XRoboToolkit-PC-Service

# 克隆示例代码
cd ~/lcz0820
git clone https://github.com/XR-Robotics/XRoboToolkit-Teleop-Sample-Python.git

# 安装Python SDK（使用conda环境）
cd XRoboToolkit-Teleop-Sample-Python
bash setup_conda.sh --conda piper_teleop_env
conda activate piper_teleop_env
bash setup_conda.sh --install
```

### 步骤2: 创建Piper遥操作控制器

需要创建类似于`DualArmURController`的`PiperTeleopController`，包含：

1. **初始化**
   - 连接Piper SDK (CAN)
   - 初始化Placo IK求解器
   - 配置遥操作参数

2. **控制循环**
   - 读取控制器位姿
   - 检查握把激活状态
   - 计算目标末端位姿（delta映射）
   - IK求解得到关节角度
   - 发送关节指令到Piper

3. **夹爪控制**
   - 读取扳机值
   - 映射到夹爪位置（0-70mm）

### 步骤3: Piper机械臂配置

```python
PIPER_CONFIG = {
    "right_arm": {
        "link_name": "link6",              # 末端执行器链接名
        "pose_source": "right_controller",  # 使用右手控制器
        "control_trigger": "right_grip",    # 握把激活控制
        "control_mode": "pose",             # 6DOF位姿控制
        "gripper_config": {
            "type": "parallel",
            "gripper_trigger": "right_trigger",
            "joint_names": ["gripper"],
            "open_pos": [70.0],             # 70mm打开
            "close_pos": [0.0],             # 0mm关闭
        },
    }
}
```

## 代码结构

```
piper_teleop/
├── piper_teleop_controller.py      # 主控制器（新建）
│   ├── PiperTeleopController
│   │   ├── __init__()              # 初始化Piper SDK + Placo
│   │   ├── run_controller_thread() # 控制循环
│   │   ├── calc_target_pose()      # Delta映射
│   │   ├── solve_ik()              # IK求解
│   │   └── send_joint_command()    # 发送CAN指令
│
├── scripts/
│   └── teleop_piper_hardware.py    # 主运行脚本（新建）
│
└── config/
    └── piper_config.yaml           # Piper参数配置
```

## 关键代码片段

### PiperTeleopController核心逻辑

```python
from xrobotoolkit_sdk import XrClient
from piper_sdk import C_PiperInterface_V2
from piper_teleop.ik_solver_placo import PiperIKSolverPlaco
import numpy as np

class PiperTeleopController:
    def __init__(self, xr_client: XrClient):
        self.xr_client = xr_client
        
        # 初始化Piper SDK
        self.piper = C_PiperInterface_V2(can_name="can0", judge_flag=False)
        self.piper.ConnectPort()
        
        # 使能机械臂
        while not self.piper.EnablePiper():
            time.sleep(0.01)
        
        # 初始化IK求解器
        self.ik_solver = PiperIKSolverPlaco(urdf_path="piper_description.urdf")
        
        # 状态变量
        self.reference_set = False
        self.xr_ref_pose = None
        self.ee_ref_pose = None
        
    def run_controller_thread(self, stop_signal):
        """主控制循环"""
        while not stop_signal.is_set():
            # 1. 读取控制器数据
            grip = self.xr_client.get_key_value_by_name("right_grip")
            trigger = self.xr_client.get_key_value_by_name("right_trigger")
            controller_pose = self.xr_client.get_pose_by_name("right_controller")
            
            # 2. 检查激活状态
            if grip > 0.5:  # 握把按下
                if not self.reference_set:
                    # 首次激活，设置引用位姿
                    ee_pose = self.get_current_ee_pose()
                    self.set_reference(controller_pose, ee_pose)
                
                # 3. 计算目标位姿（delta映射）
                target_pose = self.calc_target_pose(controller_pose)
                
                # 4. IK求解
                joint_angles = self.ik_solver.solve_ik(
                    target_position=target_pose[:3],
                    target_orientation=self.quat_to_matrix(target_pose[3:])
                )
                
                # 5. 发送关节指令
                if joint_angles is not None:
                    self.send_joint_command(joint_angles)
                
                # 6. 控制夹爪
                gripper_pos = int(trigger * 70000)  # 0-70mm转微米
                self.piper.GripperCtrl(gripper_pos, 1000, 0x01, 0)
            else:
                # 松开握把，重置引用
                self.reference_set = False
            
            time.sleep(0.01)  # 100Hz
```

## 完整实现计划

### 已完成 ✓
- [x] Piper SDK集成和测试
- [x] 遥操作映射算法（TeleopMapper）
- [x] Placo IK求解器集成
- [x] 基础架构设计

### 待实现
- [ ] 安装XRoboToolkit PC Service
- [ ] 安装xrobotoolkit_sdk Python包
- [ ] 创建PiperTeleopController
- [ ] 实现delta映射和IK求解循环
- [ ] 集成Piper SDK控制
- [ ] 测试和调试

## 使用流程

### 1. 硬件准备
- Pico头显连接到PC（USB）
- Piper机械臂连接到PC（CAN）
- 启动XRoboToolkit PC Service

### 2. 运行遥操作
```bash
# 激活conda环境
conda activate piper_teleop_env

# 运行Piper遥操作
python scripts/teleop_piper_hardware.py
```

### 3. 操作流程
1. 戴上Pico头显
2. 握住右手控制器的握把激活控制
3. 移动控制器 → 机械臂跟随
4. 按扳机 → 夹爪闭合
5. 松开扳机 → 夹爪打开
6. 松开握把 → 停止控制

## 调试建议

### 检查Pico连接
```python
from xrobotoolkit_teleop.common.xr_client import XrClient
xr = XrClient()
print(xr.get_pose_by_name("right_controller"))  # 应该输出位姿
```

### 测试IK求解器
```bash
python test_ik_simple.py
```

### 验证CAN通信
```bash
source piper_venv/bin/activate
python test_piper_basic.py
```

## 参考资源

- **XRoboToolkit示例**: `XRoboToolkit-Teleop-Sample-Python/`
- **UR5遥操作示例**: `scripts/hardware/teleop_dual_ur5e_hardware.py`
- **Piper SDK文档**: `piper_sdk/README(ZH).MD`
- **我们的实现**: `piper_teleop/piper_teleop/full_teleop_node.py`

## 下一步行动

1. **安装XRoboToolkit PC Service** （最优先）
   - 下载并运行PC Service程序
   - 连接Pico头显测试

2. **创建PiperTeleopController**
   - 参考UR5示例
   - 集成Piper SDK
   
3. **测试和调试**
   - 先测试Pico数据读取
   - 再测试IK和机械臂控制
   - 最后整合完整流程

需要我帮你创建完整的PiperTeleopController代码吗？
