# Pico遥操作Piper机械臂 - 快速开始

## 一键运行

```bash
python3 pico_teleop_piper.py
```

## 前提条件

### 1. 安装XRoboToolkit PC Service

**下载链接**: https://github.com/XR-Robotics/XRoboToolkit-PC-Service

- Windows: 下载并安装PC Service程序
- Linux: 参考README编译安装

**启动服务**:
- 运行XRoboToolkit PC Service程序
- 保持程序运行状态

### 2. 安装Python依赖

```bash
# 如果还没有安装Piper SDK
cd piper_sdk
pip install .

# 安装XRoboToolkit Python SDK
pip install xrobotoolkit-sdk

# 或从源码安装
cd XRoboToolkit-Teleop-Sample-Python
bash setup.sh
```

### 3. 硬件连接

1. **Pico头显** → PC (USB连接)
2. **Piper机械臂** → PC (CAN连接)

```bash
# 激活CAN接口
cd piper_sdk/piper_sdk
bash can_activate.sh can0 1000000 "1-11.2:1.0"
```

## 使用方法

### 启动程序

```bash
# 确保虚拟环境已激活（如果使用）
source piper_venv/bin/activate

# 运行遥操作程序
python3 pico_teleop_piper.py
```

### 控制说明

| 操作 | 功能 |
|------|------|
| **握住右手握把** | 激活机械臂控制 |
| **移动右手控制器** | 机械臂末端跟随移动 |
| **按右扳机** | 关闭夹爪 |
| **松开右扳机** | 打开夹爪 |
| **B按钮** | 紧急停止 |
| **Ctrl+C** | 退出程序 |

### 操作流程

1. 戴上Pico头显
2. 运行程序，等待初始化完成
3. 握住右手控制器的握把
4. 第一次握住会设置引用位置
5. 移动控制器，机械臂跟随
6. 按扳机控制夹爪
7. 松开握把停止控制
8. 可以重新握住继续控制

## 特性

✅ **无需IK求解** - 直接使用笛卡尔空间控制
✅ **Delta映射** - 相对运动，避免跳变
✅ **工作空间保护** - 自动限制在安全范围内
✅ **实时控制** - 100Hz控制频率
✅ **夹爪同步** - 扳机实时控制夹爪开合

## 参数调整

可以在代码中修改这些参数:

```python
# 位置映射比例（值越大，控制器小动作对应机械臂大动作）
self.position_scale = 2.0

# 最大单步移动距离（米）
self.max_delta = 0.05

# 工作空间限制（米）
X: 0.10 ~ 0.35
Y: -0.20 ~ 0.20
Z: 0.10 ~ 0.35
```

## 故障排除

### XRoboToolkit SDK未找到

```bash
# 检查PC Service是否运行
ps aux | grep XRobo

# 检查Pico连接
# 应该能在PC Service界面看到设备连接状态
```

### Piper SDK未找到

```bash
# 重新安装
cd piper_sdk
pip install . --force-reinstall
```

### 机械臂无法使能

```bash
# 检查CAN连接
ifconfig | grep can0

# 重新激活CAN
cd piper_sdk/piper_sdk
bash can_activate.sh can0 1000000 "1-11.2:1.0"

# 检查机械臂是否上电
```

### 控制不响应

1. 确保握住握把
2. 检查控制器电量
3. 尝试重启程序
4. 检查PC Service连接状态

## 安全注意事项

⚠️ **重要**:
- 首次使用时使用较小的`position_scale`（如1.0）
- 确保机械臂周围没有障碍物
- 随时准备按B按钮紧急停止
- 不要突然大幅移动控制器

## 技术细节

### 控制流程

```
1. 读取Pico控制器位姿
2. 检查握把是否按下
3. 首次激活: 记录当前XR和机械臂位置作为引用
4. 计算Delta: (当前XR位置 - 引用XR位置) × 比例
5. 应用Delta: 引用机械臂位置 + Delta
6. 发送笛卡尔坐标指令到Piper
7. 同时根据扳机控制夹爪
```

### 为什么不用IK？

这个版本直接使用Piper SDK的笛卡尔空间控制功能:
- `EndPoseCtrl()` - 直接指定末端位置
- SDK内部处理IK求解
- 更简单，更可靠

## 下一步

如果需要更高级的功能:
- 姿态控制（旋转）
- 多机械臂协调
- 数据记录
- 自定义按钮映射

可以参考 `teleop_piper_pico.py` 中的完整版本。

## 文档

- XRoboToolkit: https://github.com/XR-Robotics/XRoboToolkit-Teleop-Sample-Python
- Piper SDK: `piper_sdk/README(ZH).MD`
- 完整集成指南: `PICO_INTEGRATION_GUIDE.md`
