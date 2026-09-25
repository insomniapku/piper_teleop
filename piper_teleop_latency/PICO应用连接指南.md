# Pico头显连接问题 - 最终解决方案

## 🔍 问题确认

经过测试发现：
1. ✅ Pico头显网络可达（192.168.111.85）
2. ✅ PC和Pico在同一网段（192.168.111.123/24）
3. ✅ RoboticsServiceProcess正在运行
4. ✅ 配置文件已更新为Pico IP
5. ❌ **仍然显示TestDevice，数据全为0**

## 💡 根本原因

**TestDevice是RoboticsServiceProcess的内置模拟设备**。要连接真实Pico头显，需要：

### Pico头显端运行配套应用

RoboticsServiceProcess需要Pico头显上运行**XRoboToolkit客户端应用**才能建立连接。

---

## 🎯 解决方案

### 方案1: 在Pico头显上安装XRoboToolkit应用 ⭐ 必需

#### 步骤1: 查找Pico应用
在Pico头显中查找以下应用：
- **XRoboToolkit**
- **RoboticsService Client**
- **XR Robot**
- 或类似名称的应用

#### 步骤2: 安装应用（如果没有）

**选项A: 通过SideQuest安装**
```bash
# 1. 在PC上安装SideQuest
# 下载: https://sidequestvr.com/

# 2. 启用Pico开发者模式
# Pico头显: 设置 → 关于 → 多次点击版本号 → 开启开发者模式

# 3. USB连接Pico到PC
# 确认授权弹窗

# 4. 通过SideQuest安装XRoboToolkit APK
```

**选项B: 通过ADB安装**
```bash
# 1. 启用USB调试（同上）

# 2. 连接Pico
adb devices

# 3. 安装APK（需要从XRoboToolkit获取APK文件）
adb install path/to/XRoboToolkit.apk

# 4. 查看已安装应用
adb shell pm list packages | grep -i xrobo
```

#### 步骤3: 在Pico中配置连接

启动XRoboToolkit应用后：
1. 进入设置/Settings
2. 输入PC IP地址: **192.168.111.123**
3. 端口: **60061** (默认)
4. 点击连接/Connect

---

### 方案2: 使用Unity示例应用

README提到有Unity示例程序，可能包含Pico端应用：

```bash
# 在PC上查找Unity应用
find /home/zktitan/20260803/XRoboToolkit-PC-Service -name "*.apk" -o -name "Unity*"
```

如果找到APK文件，通过ADB安装到Pico：
```bash
adb install path/to/UnityApp.apk
```

---

### 方案3: 检查现有Pico应用

你的Pico可能已经安装了应用，但没有配置：

#### 在Pico头显中：
1. 打开应用库
2. 查找"未知来源"或"侧载应用"
3. 查找XRoboToolkit相关应用
4. 启动应用
5. 配置PC IP: 192.168.111.123

---

## 📋 完整操作流程

### 阶段1: 准备Pico头显

```bash
# 1. 启用开发者模式（在Pico头显中）
设置 → 系统 → 关于 → 多次点击"版本号" → 开发者模式

# 2. 启用USB调试
设置 → 系统 → 开发者选项 → USB调试

# 3. 连接USB到PC（如果需要安装应用）
USB-C连接Pico到PC

# 4. 确认连接
adb devices
```

### 阶段2: 查找或安装XRoboToolkit应用

**检查是否已安装：**
```bash
adb shell pm list packages | grep -i "xrobo\|robot\|toolkit"
```

**如果未安装，需要获取APK：**
1. 查看XRoboToolkit GitHub Releases
2. 或联系XRoboToolkit支持获取APK
3. 或检查项目文件中是否有APK

### 阶段3: 在Pico中运行应用并连接

1. 戴上Pico头显
2. 启动XRoboToolkit应用
3. 输入PC IP: **192.168.111.123**
4. 点击连接

### 阶段4: 验证连接

```bash
# 在PC上运行测试
cd /home/zktitan/lcz0820
python3 test_sdk_basic.py
```

**期望输出**：
```
device found: Pico 4  # 不再是TestDevice！
右手控制器位置: [0.123, -0.045, 0.678]  # 非零值
```

---

## 🔧 替代方案：使用ROS直接控制

如果Pico连接一直有问题，可以暂时跳过VR，直接用其他方式控制：

### 选项1: 键盘控制
```python
# 修改pico_teleop_piper.py
# 使用键盘输入代替VR控制器
import keyboard

# WASD控制位置
# 空格控制夹爪
```

### 选项2: ROS joystick
```bash
# 使用游戏手柄通过ROS控制
ros2 run joy joy_node
```

### 选项3: 手动示教
```python
# 使用Piper的示教模式
# 手动拖动机械臂到目标位置
# 记录轨迹
```

---

## 📞 需要的信息

为了进一步帮助你，请提供：

1. **Pico型号**：
   - Pico 4?
   - Pico Neo 3?
   - 其他？

2. **已安装的应用**：
   ```bash
   adb devices
   adb shell pm list packages
   ```

3. **XRoboToolkit来源**：
   - 从哪里获取的这套系统？
   - 是否有完整的安装包或文档？

4. **项目背景**：
   - 这是课题项目吗？
   - 是否有指导老师或技术支持？

---

## 🎯 关键点总结

**TestDevice的真相**：
- TestDevice是内置的模拟设备
- 用于在没有真实VR设备时进行开发测试
- 要连接真实Pico，需要：
  1. Pico上运行XRoboToolkit客户端应用
  2. 应用中配置PC的IP地址
  3. 建立连接后，TestDevice会被真实设备替代

**当前状态**：
- ✅ PC端完全正常（Piper SDK, XRoboToolkit SDK, RoboticsServiceProcess）
- ❌ 缺少Pico端的客户端应用或未配置连接

**下一步行动**：
1. 在Pico头显中查找XRoboToolkit应用
2. 如果没有，需要获取并安装APK
3. 配置应用连接到PC IP
4. 重新测试

---

## 📚 相关文档

- XRoboToolkit GitHub: https://github.com/XR-Robotics/XRoboToolkit-PC-Service
- Pico开发者文档: https://developer-global.pico-interactive.com/
- SideQuest: https://sidequestvr.com/

---

## 💬 建议

由于这个问题涉及到Pico端应用的安装和配置，建议：

1. **联系原项目提供者**
   - 询问Pico端APK文件
   - 获取完整的部署文档

2. **查看XRoboToolkit官方资源**
   - GitHub Issues
   - 用户论坛
   - 技术支持

3. **考虑替代方案**
   - 如果VR部分不是核心需求，可以先用其他输入方式（键盘、手柄）
   - 机械臂SDK部分已完全正常，可以先进行机械臂控制的开发和测试

---

**文档创建**: 2026-08-26  
**核心问题**: 缺少Pico端客户端应用  
**状态**: 等待Pico应用配置
