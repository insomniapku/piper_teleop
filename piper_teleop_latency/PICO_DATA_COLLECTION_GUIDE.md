# Pico VR 数据采集详细实施指南 - 完全可执行版本

## 目标

使用Pico VR头显和手柄，通过piper_teleop系统采集机器人遥操作数据，用于机器人学习和演示。

## ⚡ 快速开始

**如果你只想快速测试系统**：

```bash
cd /home/zktitan/lcz0820/piper_teleop
python3 scripts/start_data_collection.py
```

该脚本会自动检查所有前置条件并启动系统。

---

## 系统架构

```
┌─────────────────┐
│   Pico VR       │
│   头显 + 手柄    │
└────────┬────────┘
         │ WiFi/USB
         ▼
┌─────────────────────────────────────┐
│   XR Interface (XRoboToolkit)       │
│   - 读取手柄位置/姿态                 │
│   - 读取按钮状态                     │
│   - 发布ROS话题                      │
└────────┬────────────────────────────┘
         │ ROS Topics
         ▼
┌─────────────────────────────────────┐
│   Piper Teleop Controller           │
│   - M3: Mapping (VR → Robot)        │
│   - M4: IK Solver                   │
│   - M5: Safety & Control            │
└────────┬────────────────────────────┘
         │ Joint Commands
         ▼
┌─────────────────────────────────────┐
│   Piper Robot                       │
│   - 执行关节命令                     │
│   - 反馈关节状态                     │
└─────────────────────────────────────┘
         │
         ▼
┌─────────────────────────────────────┐
│   Data Logger                       │
│   - 关节位置/速度/力矩                │
│   - 末端位姿                         │
│   - 相机图像                         │
│   - 时间戳对齐                       │
└─────────────────────────────────────┘
```

---

## 前置准备

### 硬件要求

1. **Pico VR头显**
   - Pico 4 或 Pico Neo 3
   - 手柄 x2（左右手）
   - 充满电

2. **Piper机器人**
   - 已连接并通电
   - CAN总线或串口连接正常

3. **工作站/电脑**
   - Ubuntu 22.04
   - ROS 2 Jazzy
   - WiFi连接能力

4. **相机（可选）**
   - RealSense D435i 或类似
   - USB 3.0连接

### 软件依赖

```bash
# ROS 2环境
source /opt/ros/jazzy/setup.bash

# Python依赖
pip install numpy scipy pykdl

# XRoboToolkit (Pico通信)
# 需要安装xrobotoolkit相关包
```

---

## 步骤1: 配置Pico VR连接

### 1.1 检查XR Interface状态

piper_teleop已经集成了XR接口，支持三种模式：

- **mock**: 模拟数据（用于测试，无需真实Pico）
- **sdk**: 使用XRoboToolkit SDK连接真实Pico
- **auto**: 自动检测（有SDK则用SDK，否则用mock）

**测试XR接口**：

```bash
cd /home/zktitan/lcz0820/piper_teleop
source /opt/ros/jazzy/setup.bash

# 使用mock模式测试（无需Pico硬件）
ros2 run piper_teleop xr_test_node --ros-args -p mode:=mock
```

**预期输出**：
```
[XR Interface] Using MOCK mode
[XR Interface] Right controller: pos=[0.200, -0.300, -0.500], quat=[0.000, 0.000, 0.000, 1.000]
[XR Interface] Left controller: pos=[-0.200, -0.300, -0.500], quat=[0.000, 0.000, 0.000, 1.000]
[XR Interface] Tracking valid: True
```

### 1.2 配置真实Pico连接（可选）

**仅在有真实Pico VR硬件时执行此步骤**。

#### 选项A: 安装XRoboToolkit SDK

```bash
# 检查是否已安装
python3 -c "import xrobotoolkit_sdk; print('SDK已安装')" 2>/dev/null || echo "SDK未安装"

# 如果未安装，从源码安装
cd /home/zktitan
git clone https://github.com/rdk-x/XRoboToolkit.git
cd XRoboToolkit
pip3 install -e .
```

#### 选项B: 使用现有teleop_a7lite中的SDK

```bash
# 添加到Python路径
export PYTHONPATH=/home/zktitan/lcz0820/Teleop-a7lite:$PYTHONPATH

# 添加到 ~/.bashrc 以永久生效
echo 'export PYTHONPATH=/home/zktitan/lcz0820/Teleop-a7lite:$PYTHONPATH' >> ~/.bashrc
```

### 1.3 Pico硬件连接

**方式A: WiFi连接（推荐）**

1. Pico头显连接到与工作站相同的WiFi网络
2. 在Pico上启动XR Stream应用
3. 记录Pico的IP地址

```bash
# 测试连接
ping <PICO_IP>
```

4. 配置XR客户端：

```python
# 创建配置文件: ~/.xr_config.yaml
xr_device:
  type: "pico"
  ip: "192.168.1.100"  # 替换为你的Pico IP
  port: 8080
  streaming:
    pose_rate: 90  # Hz
    button_rate: 60  # Hz
```

**方式B: USB连接**

```bash
# 通过USB连接Pico
adb devices  # 确认连接

# 端口转发
adb forward tcp:8080 tcp:8080
```

### 1.3 测试XR输入

```bash
# 运行XR测试节点
cd /home/zktitan/lcz0820/piper_teleop
source install/setup.bash

ros2 run piper_teleop xr_test_node
```

**预期输出**:
```
[XR Interface] Connected to Pico
[XR Interface] Right controller: pos=[0.2, -0.3, -0.5], quat=[0, 0, 0, 1]
[XR Interface] Left controller: pos=[-0.2, -0.3, -0.5], quat=[0, 0, 0, 1]
[XR Interface] Right grip: 0.00, Left grip: 0.00
```

---

## 步骤2: 校准坐标系

### 2.1 确定坐标系对应关系

**Pico VR坐标系**:
```
原点: 头显位置
X轴: 右
Y轴: 上
Z轴: 后（用户面向的反方向）
单位: 米
```

**Piper机器人坐标系**:
```
原点: base_link
X轴: 前
Y轴: 左
Z轴: 上
单位: 米
```

### 2.2 执行校准流程

**方法1: 手动校准（推荐）**

1. 将机器人移动到参考位置（如home位置）
2. 在Piper工作空间中标记4个参考点
3. 使用手柄依次触碰这4个点
4. 运行校准程序计算变换矩阵

```bash
ros2 run piper_teleop calibrate_coordinates

# 按照提示操作:
# 1. 触碰点1（机器人前方中心，桌面高度）
# 2. 触碰点2（机器人右侧30cm）
# 3. 触碰点3（机器人上方20cm）
# 4. 触碰点4（机器人左侧30cm）
```

**输出示例**:
```yaml
xr_to_piper_transform:
  translation: [0.5, 0.0, 0.8]  # 米
  rotation: [0.0, 0.0, 1.5708]  # 弧度 (RPY)
  scale: 1.0
```

2. 更新配置文件：

```bash
# 编辑配置文件
nano /home/zktitan/lcz0820/piper_teleop/config/teleop_config.yaml

# 更新 xr_to_piper_transform 部分
xr_to_piper_transform:
  translation: [0.5, 0.0, 0.8]  # 从校准获得
  rotation: [0.0, 0.0, 1.5708]  # 从校准获得
```

**方法2: 视觉校准（精确）**

如果有相机，可以使用ArUco标记进行自动校准：

```bash
# 在机器人末端贴ArUco标记
ros2 run piper_teleop visual_calibration

# 移动手柄到机器人末端位置
# 程序自动计算变换
```

### 2.3 验证校准

```bash
ros2 run piper_teleop test_calibration

# 测试步骤:
# 1. 移动手柄到已知位置
# 2. 查看映射后的机器人空间坐标
# 3. 确认坐标合理
```

---

## 步骤3: 配置数据采集

### 3.1 配置采集参数

编辑配置文件：

```bash
nano /home/zktitan/lcz0820/piper_teleop/config/data_collection.yaml
```

```yaml
data_collection:
  # 采集频率
  control_rate: 50  # Hz，机器人控制频率
  log_rate: 50      # Hz，数据记录频率
  
  # 采集内容
  record_joint_states: true
  record_end_effector_pose: true
  record_joint_torques: true
  record_gripper_state: true
  record_camera: true
  
  # 相机配置
  camera:
    color_width: 640
    color_height: 480
    depth_width: 640
    depth_height: 480
    fps: 30
    enable_depth: true
    
  # 存储配置
  output_dir: "/home/zktitan/data/piper_demos"
  file_format: "hdf5"  # hdf5 or rosbag
  compression: true
  
  # 触发配置
  trigger:
    button: "B"  # Pico B键开始/停止录制
    min_duration: 1.0  # 最小录制时长（秒）
    max_duration: 300.0  # 最大录制时长（秒）
  
  # 安全限制
  workspace_limit:
    x_min: -0.1
    x_max: 0.6
    y_min: -0.4
    y_max: 0.4
    z_min: 0.0
    z_max: 0.8
```

### 3.2 创建数据存储目录

```bash
mkdir -p /home/zktitan/data/piper_demos
mkdir -p /home/zktitan/data/piper_demos/raw
mkdir -p /home/zktitan/data/piper_demos/processed
```

---

## 步骤4: 运行数据采集

### 4.1 方法A: 使用快速启动脚本（推荐）

```bash
cd /home/zktitan/lcz0820/piper_teleop

# 运行快速启动脚本
python3 scripts/start_data_collection.py
```

该脚本会：
1. 自动检查所有前置条件
2. 创建必要的目录
3. 启动数据采集系统
4. 显示使用说明

### 4.2 方法B: 手动启动各组件

**终端1: 启动Piper机器人接口（如果有真实机器人）**

```bash
cd /home/zktitan/lcz0820
source install/setup.bash

# 启动Piper硬件接口
ros2 run piper_driver piper_interface_node
```

**终端2: 启动相机（可选）**

```bash
# 启动RealSense相机
ros2 launch realsense2_camera rs_launch.py \
  enable_color:=true \
  enable_depth:=false \
  color_width:=640 \
  color_height:=480 \
  fps:=30
```

**终端3: 启动数据采集系统**

```bash
cd /home/zktitan/lcz0820/piper_teleop
source /opt/ros/jazzy/setup.bash

# Mock模式（用于测试，无需真实硬件）
ros2 launch piper_teleop data_collection.launch.py \
  xr_mode:=mock \
  record_camera:=false

# 真实Pico模式（需要XRoboToolkit SDK）
ros2 launch piper_teleop data_collection.launch.py \
  xr_mode:=sdk \
  record_camera:=true \
  output_dir:=/home/zktitan/data/piper_demos
```

**预期输出**:
```
[xr_interface] XRInterface initialized in MOCK mode
[data_collection] Data Collection Node Started
================================================================
Output directory: /home/zktitan/data/piper_demos
Next episode: 001
Control rate: 50 Hz
Camera recording: False
Trigger button: B
================================================================
Press B button to start/stop recording
================================================================
```

### 4.2 执行数据采集

1. **戴上Pico头显**
   - 确认画面清晰
   - 确认手柄追踪正常

2. **测试控制**
   - 握住右手柄的握把键（Grip）
   - 移动手柄，观察机器人跟随
   - 松开握把键，机器人停止跟随

3. **开始录制**
   - 按下B键开始录制
   - 看到提示: `[Data Logger] Recording started - Episode 001`

4. **执行演示任务**
   - 握住握把键控制机器人
   - 执行目标任务（如抓取、放置）
   - 动作平滑、准确

5. **停止录制**
   - 再次按下B键停止录制
   - 看到提示: `[Data Logger] Recording stopped - Saved to /home/zktitan/data/piper_demos/episode_001.h5`

### 4.3 采集多个演示

重复步骤4.2，采集多个episode：

```
episode_001.h5  # 第一次演示
episode_002.h5  # 第二次演示
episode_003.h5  # ...
```

**推荐采集数量**:
- 训练用: 50-200个演示
- 测试用: 10-20个演示
- 验证用: 5-10个演示

---

## 步骤5: 验证数据质量

### 5.1 检查数据文件

```bash
cd /home/zktitan/data/piper_demos

# 列出所有episode
ls -lh episode_*.h5

# 预期输出:
# -rw-r--r-- 1 user user  45M Jan 15 10:23 episode_001.h5
# -rw-r--r-- 1 user user  38M Jan 15 10:28 episode_002.h5
```

### 5.2 查看数据内容

```bash
# 安装h5py（如果还没有）
pip install h5py

# 查看数据结构
python3 << EOF
import h5py
import numpy as np

with h5py.File('episode_001.h5', 'r') as f:
    print("数据集:")
    for key in f.keys():
        print(f"  {key}: shape={f[key].shape}, dtype={f[key].dtype}")
    
    # 查看时间信息
    timestamps = f['timestamps'][:]
    duration = timestamps[-1] - timestamps[0]
    rate = len(timestamps) / duration
    print(f"\n时长: {duration:.2f}秒")
    print(f"采样率: {rate:.2f} Hz")
EOF
```

**预期输出**:
```
数据集:
  timestamps: shape=(2500,), dtype=float64
  joint_positions: shape=(2500, 6), dtype=float32
  joint_velocities: shape=(2500, 6), dtype=float32
  joint_torques: shape=(2500, 6), dtype=float32
  ee_position: shape=(2500, 3), dtype=float32
  ee_orientation: shape=(2500, 4), dtype=float32
  gripper_position: shape=(2500,), dtype=float32
  camera_color: shape=(1500, 480, 640, 3), dtype=uint8
  camera_depth: shape=(1500, 480, 640), dtype=uint16
  camera_timestamps: shape=(1500,), dtype=float64

时长: 50.00秒
采样率: 50.00 Hz
```

### 5.3 可视化轨迹

**使用提供的可视化工具**：

```bash
cd /home/zktitan/data/piper_demos

# 可视化单个episode
python3 /home/zktitan/lcz0820/piper_teleop/scripts/visualize_data.py episode_001.h5
```

**输出**：
- 控制台：详细的数据质量报告
- 文件：`episode_001_visualization.png`（轨迹图）

**可视化包含**：
- 关节位置随时间变化
- 末端执行器位置轨迹
- 关节速度曲线
- 数据质量检查结果

### 5.4 数据质量检查清单

✅ **检查项**:
- [ ] 数据文件大小合理（20-100 MB）
- [ ] 采样率一致（50 Hz）
- [ ] 关节位置在限位内
- [ ] 关节速度合理（< 5 rad/s）
- [ ] 末端轨迹平滑
- [ ] 相机图像清晰
- [ ] 时间戳单调递增
- [ ] 无数据缺失（NaN）

---

## 步骤6: 数据后处理

### 6.1 数据清洗

**单个episode清洗**：

```bash
cd /home/zktitan/data/piper_demos

# 清洗单个episode
python3 /home/zktitan/lcz0820/piper_teleop/scripts/clean_data.py episode_001.h5

# 输出: episode_001_clean.h5
```

**批量清洗所有episode**：

```bash
cd /home/zktitan/data/piper_demos

# 批量清洗
python3 /home/zktitan/lcz0820/piper_teleop/scripts/batch_process.py . clean
```

**清洗内容**：
1. ✅ 移除NaN值
2. ✅ 移除超出关节限位的样本
3. ✅ 移除速度异常点（>5 rad/s）
4. ✅ Savitzky-Golay滤波器平滑轨迹

### 6.2 数据增强（可选）

```python
# 创建脚本: augment_data.py
def augment_episode(input_file, output_prefix, num_augmentations=5):
    """
    数据增强:
    1. 添加小幅噪声
    2. 时间缩放
    3. 空间平移
    """
    with h5py.File(input_file, 'r') as f:
        for i in range(num_augmentations):
            # 添加噪声（关节位置 ±0.01 rad）
            joint_pos_aug = f['joint_positions'][:] + \
                           np.random.normal(0, 0.01, f['joint_positions'].shape)
            
            output_file = f"{output_prefix}_aug_{i:03d}.h5"
            with h5py.File(output_file, 'w') as f_out:
                # 保存增强后的数据
                f_out.create_dataset('joint_positions', data=joint_pos_aug)
                # 复制其他数据
                for key in f.keys():
                    if key != 'joint_positions':
                        f_out.create_dataset(key, data=f[key][:])
```

### 6.3 生成训练/测试集划分

```bash
cd /home/zktitan/data/piper_demos

# 划分数据集（默认80%训练，20%测试）
python3 /home/zktitan/lcz0820/piper_teleop/scripts/batch_process.py . split
```

**输出结构**：
```
/home/zktitan/data/piper_demos/
├── train/
│   ├── episode_001_clean.h5
│   ├── episode_002_clean.h5
│   └── ...
├── test/
│   ├── episode_010_clean.h5
│   └── ...
└── processed/
    ├── episode_001_clean.h5
    └── ...
```

---

## 故障排除

### 问题1: Pico连接失败

**症状**: `[XR Interface] Failed to connect to Pico`

**解决方案**:
1. 检查WiFi连接: `ping <PICO_IP>`
2. 确认Pico上XR Stream应用正在运行
3. 检查防火墙: `sudo ufw allow 8080`
4. 重启Pico头显

### 问题2: 机器人不跟随手柄

**症状**: 手柄移动，机器人不动

**解决方案**:
1. 确认握住握把键（Grip）
2. 检查坐标系校准: `ros2 run piper_teleop test_calibration`
3. 查看IK求解成功率: 检查终端输出
4. 确认机器人在工作空间内

### 问题3: IK求解失败率高

**症状**: `[Piper Teleop] IK failed, using fallback`

**解决方案**:
1. 检查workspace limit配置
2. 减小移动速度
3. 避免奇异位形
4. 使用更好的种子配置

### 问题4: 数据采集卡顿

**症状**: 采样率不稳定

**解决方案**:
1. 降低相机分辨率: 640x480 → 320x240
2. 降低相机帧率: 30 → 15 Hz
3. 禁用深度相机（如不需要）
4. 使用SSD存储而非HDD

### 问题5: 相机图像模糊

**症状**: 保存的图像质量差

**解决方案**:
1. 调整相机曝光
2. 增加环境光照
3. 清洁相机镜头
4. 提高图像分辨率

---

## 最佳实践

### 数据采集建议

1. **环境准备**
   - 充足的光照
   - 清晰的工作空间
   - 移除障碍物

2. **演示质量**
   - 动作平滑、自然
   - 速度适中（不要太快）
   - 避免急停或急转
   - 每个episode完成完整任务

3. **多样性**
   - 不同的起始位置
   - 不同的物体摆放
   - 不同的执行速度
   - 包含成功和失败案例（用于学习鲁棒性）

4. **数据标注**
   - 记录任务描述
   - 标注关键帧
   - 记录特殊事件（如碰撞、掉落）

### 数据组织

```
/home/zktitan/data/piper_demos/
├── task1_pick_cube/
│   ├── train/
│   │   ├── episode_001_clean.h5
│   │   ├── episode_002_clean.h5
│   │   └── ...
│   ├── test/
│   │   ├── episode_050_clean.h5
│   │   └── ...
│   └── metadata.json
├── task2_stack_blocks/
│   └── ...
└── README.md
```

### 元数据记录

在每个任务目录创建`metadata.json`:

```json
{
  "task_name": "pick_cube",
  "description": "Pick up a red cube from the table and place it in a box",
  "robot": "Piper",
  "environment": "lab_table_1",
  "date_collected": "2024-01-15",
  "num_episodes": 50,
  "success_rate": 0.92,
  "collector": "operator_1",
  "objects": ["red_cube_5cm", "cardboard_box_15cm"],
  "notes": "Good lighting conditions, stable tracking"
}
```

---

## 下一步

数据采集完成后，可以用于：

1. **机器人学习**
   - 行为克隆 (Behavior Cloning)
   - 强化学习 (Reinforcement Learning)
   - 模仿学习 (Imitation Learning)

2. **系统改进**
   - 分析失败案例
   - 优化IK求解器
   - 改进控制策略

3. **数据分享**
   - 上传到数据集仓库
   - 发布论文
   - 开源贡献

---

## 参考资源

- **piper_teleop代码**: `/home/zktitan/lcz0820/piper_teleop`
- **配置文件**: `config/teleop_config.yaml`
- **测试脚本**: `piper_teleop/test/`
- **问题诊断**: `PIPER_TELEOP_ISSUES.md`
- **Placo优化**: `PLACO_OPTIMIZATION_SUMMARY.md`

## 支持

遇到问题请参考：
1. 故障排除章节
2. 已创建的诊断文档
3. ROS 2日志: `ros2 topic echo /rosout`
4. Piper日志: `/var/log/piper/`

---

**祝数据采集顺利！** 🎉
