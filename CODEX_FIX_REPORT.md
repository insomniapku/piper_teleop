# Pico XR 遥操作 AgileX Piper 延迟与映射问题修复报告

## 1. 摘要

本次快速修复针对三类现场问题：

1. 手柄与机械臂运动方向或比例不一致；
2. 跟随延迟明显；
3. 手柄停止后机械臂继续执行积累目标，出现“追赶”。

修复采用独立新增入口 `pico_teleop_piper_fixed.py`，没有重写 ROS2、Placo 或 PyKDL，没有删除或覆盖旧脚本。控制链路调整为：

```text
最新 Pico position
→ 官方 XR 坐标转换
→ grip 相对位移
→ 工作空间裁剪
→ 基于真实 dt 的目标限速
→ Piper MOVEP + 最新 EndPoseCtrl
```

本次仅执行静态检查和内置无硬件 dry-run。没有初始化 XR SDK、连接 Pico、连接 `can0`，也没有发送任何真实机器人或夹爪命令。

## 2. 仓库与版本

| 项目 | 内容 |
| --- | --- |
| 实际修改仓库 | `https://github.com/gxccc123/piper_teleop.git` |
| 克隆结果 | PRIMARY HTTPS 成功，未使用 SSH 或备用仓库 |
| 修改前 commit | `69ee69349a4962652d1711769f1a0f8f4a6c813a` |
| 修复分支 | `fix/pico-teleop-latency` |
| 控制代码 commit | `043b32740fb70110f9367a722cae9596494dcaba` |
| XR 官方仓库 | `https://github.com/XR-Robotics/XRoboToolkit-Teleop-Sample-Python.git` |
| XR 官方参考 commit | `79e5cb8a56e3455515ce1b476e993c764ec58739` |

PRIMARY 仓库可正常访问，因此未使用 `insomniapku/piper_teleop`。未执行 `git push`。XR 官方仓库只读且工作树干净。

## 3. 修改范围

只修改目标仓库中的：

- `pico_teleop_piper_fixed.py`：新增低延迟、安全门控直连入口；
- `CODEX_FIX_REPORT.md`：本报告。

以下内容均未修改或删除：

- `pico_teleop_piper.py`；
- `pico_teleop_improved.py`；
- ROS2 节点、launch 和配置；
- Placo、PyKDL、IK 实现；
- Piper SDK 及其示例；
- XR 官方仓库。

没有安装或升级依赖，没有增加 pytest/CI、GUI、可视化、多线程队列或新框架。

## 4. 实际入口定位

### 4.1 结论

当前直接读取 Pico 并调用 Piper SDK 的实际入口是根目录 `pico_teleop_piper.py`。依据：

- 脚本自身声明 `python3 pico_teleop_piper.py` 为直接运行方式；
- 直接调用 `get_right_controller_pose`、`get_right_grip`、`get_right_trigger`；
- 直接构造 `C_PiperInterface_V2`；
- 直接调用 `MotionCtrl_2`、`EndPoseCtrl`、`GripperCtrl`。

`pico_teleop_improved.py` 是另一套双手、绝对位置实验入口。根 README 主要指向 ROS launch，包内 README 指向 ROS2 launch，但包内 README 仍将 Pico 映射和 Piper 硬件接口标为待实现，因此本次没有改 ROS2 路径。

### 4.2 重复文件

仓库内存在内容相同的重复 package 文件，例如：

```text
piper_teleop/xr_interface.py
piper_teleop/piper_teleop/xr_interface.py

piper_teleop/full_teleop_node.py
piper_teleop/piper_teleop/full_teleop_node.py
```

这是既有仓库结构问题。本次为最小修复，没有清理目录或迁移入口。

## 5. 官方与 SDK 对照

### 5.1 XR 官方实现

官方 `xrobotoolkit_teleop/utils/geometry.py` 定义：

```python
R_HEADSET_TO_WORLD = np.array([
    [ 0,  0, -1],
    [-1,  0,  0],
    [ 0,  1,  0],
])
```

官方 `BaseTeleopController` 先转换控制器位置，再保存 grip 激活时的控制器和末端参考，以相对位移生成目标。新脚本沿用这两个关键语义。

### 5.2 Piper SDK

仓库内 SDK 示例和接口定义确认：

- `MotionCtrl_2(0x01, 0x00, speed, 0x00)`：MOVEP；
- `MotionCtrl_2(0x01, 0x02, speed, 0x00)`：MOVEL；
- `EndPoseCtrl` 位置单位：`0.001 mm`；
- 1 米对应整数 `1,000,000`；
- 末端反馈位置也是 `0.001 mm`；
- RX/RY/RZ 反馈单位为 `0.001°`。

新脚本继续使用这些单位，未引入新的转换层。

## 6. 根因与修改

### 6.1 坐标系未转换

旧代码直接计算：

```python
delta = (xr_pos - xr_ref_pos) * position_scale
```

这错误假设 XR 与 Piper 的三轴同向。官方默认关系实际为：

```text
Piper X = -XR Z
Piper Y = -XR X
Piper Z =  XR Y
```

新脚本集中定义：

```python
R_XR_TO_PIPER = np.array([
    [ 0.0,  0.0, -1.0],
    [-1.0,  0.0,  0.0],
    [ 0.0,  1.0,  0.0],
], dtype=np.float64)
```

并计算：

```python
R_FINAL = R_INSTALLATION @ R_XR_TO_PIPER
xr_pos_robot = R_FINAL @ xr_pos
delta_robot = xr_pos_robot - xr_ref_pos_robot
target = ee_ref_pos + position_scale * delta_robot
```

`--yaw-deg` 只允许 0、90、-90、180，解决常见安装朝向差异，不做复杂自动标定。

### 6.2 比例过大

旧直连入口默认 `position_scale=3.5`，会将尚未正确转换的输入进一步放大。

新脚本：

- 正常默认：`1.0`；
- `--safe-test`：强制 `0.5`。

### 6.3 `max_delta` 语义错误

旧实现把从 grip 参考点开始的累计位移截到 0.08 m。这不是单周期限制，而是把机械臂永久限制在参考点附近固定半径。

新实现正常模式取消累计位移截断，改为：

```python
step = raw_target - previous_command
max_step = max_speed_mps * dt
if norm(step) > max_step:
    step = normalize(step) * max_step
target = previous_command + step
```

`dt` 来自 `time.monotonic()` 的实际周期，并裁剪到 0.005–0.05 s。默认 `max_speed=0.08 m/s`。

只有 `--safe-test` 额外限制 grip 相对活动范围为 3 cm，这一范围不影响正常模式。

### 6.4 MOVEL 造成路径积累

旧代码在每帧执行：

```python
MotionCtrl_2(0x01, 0x02, 50, 0x00)
EndPoseCtrl(...)
```

MOVEL 用于直线路径，高频提交目标可能让固件继续执行已有路径段，造成延迟和停止后的追赶。

新脚本默认 MOVEP：

- Piper 启动完成后设置一次 MOVEP；
- grip rising edge 再确认一次；
- 50 Hz 循环主要只发最新 `EndPoseCtrl`；
- 不在每帧重建对象或重新初始化接口。

### 6.5 循环频率漂移

旧代码使用 `time.sleep(0.01)`，实际周期包含读取、计算、CAN 发送和额外 10 ms，并非稳定 100 Hz。

新代码采用 50 Hz deadline loop：

```python
next_tick += period
remaining = next_tick - time.monotonic()
```

超期时记录 overrun，并从当前 monotonic 时间重建 deadline，避免累计漂移。

### 6.6 未处理重复和过期 XR 数据

旧代码不读取 `get_time_stamp_ns()`，会重复处理相同 XR 帧，也无法识别断流。

新逻辑：

- timestamp 变化：使用最新 pose 计算目标；
- timestamp 相同：不产生重复末端目标；
- trigger 有变化时仍可按节流规则更新夹爪；
- timestamp 超过 0.2 s 不变：标记 stale、清参考、停止发送新末端目标；
- XR 恢复后不能在 grip 仍握住时自动恢复；
- 必须松开，观察到新帧后重新握住。

### 6.7 固定姿态

旧代码始终发送：

```python
RX = -179900
RY = 0
RZ = -179900
```

新脚本在 grip rising edge 调用 `GetArmEndPoseMsgs()`，保存实际 X/Y/Z 和 RX/RY/RZ。该次 grip 持续期间只更新 position，始终保持激活时的实际姿态。

若反馈为空、时间戳无效、反馈 Hz 无效或 position 非有限值：

- 不发送末端运动目标；
- 打印明确错误；
- 要求松开并重新 grip。

### 6.8 Grip clutch

旧代码只有 `grip > 0.5` 单阈值。新代码加入滞回：

| 状态 | 阈值 |
| --- | ---: |
| 激活 | ≥ 0.80 |
| 释放 | ≤ 0.60 |

0.60–0.80 区间保持原状态，避免阈值附近抖动。

状态行为：

| 事件 | 行为 |
| --- | --- |
| Rising | 等待最新 XR 帧，读取实际 Piper 位姿，保存参考，确认 MOVEP |
| 第一目标 | 严格等于当前实际末端位置，避免握住瞬间跳变 |
| Held + 新帧 | 映射、工作空间裁剪、限速、发送最新位置 |
| Held + 重复帧 | 不发送重复末端目标 |
| Falling | 清参考并停止新末端目标；不回 home、不失能 |
| Stale/反馈失败 | 锁止运动并要求重新 grip |

### 6.9 工作空间

沿用旧直连脚本范围：

| 轴 | 最小 | 最大 |
| --- | ---: | ---: |
| X | 0.10 m | 0.35 m |
| Y | -0.20 m | 0.20 m |
| Z | 0.10 m | 0.35 m |

处理顺序为先裁剪 `raw_target`，再按上一次发送目标做速度限制。

### 6.10 夹爪

保留旧方向：

```text
trigger=0.0 → 打开至 70000
trigger=1.0 → 关闭至 0
```

发送条件：

- trigger 相对上次发送值变化超过 0.01；或
- 距上次发送达到 0.2 s，进行静态刷新。

`--no-gripper` 可彻底禁止夹爪命令。没有自动做夹爪回零。

## 7. 安全模式与参数

### 7.1 `--safe-test`

| 参数 | 安全值 |
| --- | ---: |
| `position_scale` | 0.5 |
| `max_speed` | 0.03 m/s |
| grip 相对活动半径 | 0.03 m |

### 7.2 命令行参数

| 参数 | 默认值 | 说明 |
| --- | ---: | --- |
| `--can-name` | `can0` | CAN 接口 |
| `--control-rate` | `50` | 控制频率 Hz |
| `--position-scale` | `1.0` | 位置比例 |
| `--max-speed` | `0.08` | 目标限速 m/s |
| `--speed-percent` | `40` | MOVEP 速度百分比 |
| `--yaw-deg` | `0` | 安装 yaw |
| `--safe-test` | 关闭 | 首次实机限制 |
| `--no-gripper` | 关闭 | 禁止夹爪命令 |
| `--dry-run` | 默认 | 内置无硬件模拟 |
| `--hardware` | 关闭 | 显式进入硬件确认 |

## 8. 硬件安全门

未指定 `--hardware` 时，程序强制 dry-run，并且不会：

- import `xrobotoolkit_sdk`；
- import/实例化 `C_PiperInterface_V2`；
- 调用 `xrt.init()`；
- 连接 CAN；
- 调用 Piper 或夹爪命令。

指定 `--hardware` 后先打印醒目警告，并要求输入：

```text
ARM
```

输入不完全匹配就退出，此时仍未导入硬件 SDK、未打开接口。

确认后才允许连接、使能、设置 MOVEP 和按 clutch 发送目标。启动不自动回 home、不发送固定初始位置、不关节回零。退出不发送 home、`DisableArm` 或 `ResetPiper`；只停止更新并关闭 XR 客户端，机械臂保持已使能状态，由现场既有安全流程处理。

## 9. 无硬件验证结果

### 9.1 语法

```bash
python -m py_compile pico_teleop_piper_fixed.py
```

结果：通过。

### 9.2 空白与差异

```bash
git diff --check
git diff --cached --check
```

结果：通过。

### 9.3 Dry-run

```bash
python pico_teleop_piper_fixed.py --dry-run
```

验证内容：

- 程序自动启动和退出；
- 映射矩阵不是 identity；
- 第一帧 target 等于模拟实际 EE；
- 速度限制实际触发；
- target 全部 finite；
- deadline 频率接近 50 Hz。

实际摘要：

```text
mapping_non_identity=True
first_target_no_jump=True
speed_limit_observed=True
finite=True
target_rate=50.0Hz
overruns=0
result=PASS
```

## 10. 第一次真实测试

本次未执行以下命令。现场执行前必须确认工作空间无人无障碍、急停可用、CAN/Pico 正常，并由人员准备立即停止系统。

### 10.1 首次 commissioning

```bash
cd /root/data/piper_teleop
python pico_teleop_piper_fixed.py \
    --hardware \
    --safe-test \
    --can-name can0 \
    --yaw-deg 0
```

确认现场安全后才输入 `ARM`。初次建议加 `--no-gripper`，先单独验证机械臂方向。

### 10.2 Yaw 排查

保持 `--safe-test`，每次只做小幅单轴动作。默认应满足：

- XR X → Piper -Y；
- XR Y → Piper +Z；
- XR Z → Piper -X。

若安装方向不同，停止程序后分别尝试：

```text
--yaw-deg 90
--yaw-deg -90
--yaw-deg 180
```

每次修改 yaw 后都重新启动和建立 grip 参考，不在 grip 持续期间切换。

### 10.3 正常模式

只有在方向、停止行为、无跳变和工作空间全部确认后使用：

```bash
cd /root/data/piper_teleop
python pico_teleop_piper_fixed.py \
    --hardware \
    --can-name can0 \
    --position-scale 1.0 \
    --max-speed 0.08 \
    --speed-percent 40 \
    --yaw-deg 0
```

## 11. 硬件验收清单

必须重点观察：

1. 三个方向是否正确，有无串轴或比例异常；
2. 手柄停下后机械臂是否仍长时间继续追赶；
3. 松开再握住 grip 时是否发生位置或姿态跳变。

建议同时记录：

- 第一条目标是否保持实际位置；
- 快速移动后最大追赶距离和停止时间；
- XR 断流 0.2 s 后是否停止新目标；
- XR 恢复但 grip 未松开时是否继续锁止；
- 重新 grip 后是否从新的实际 EE 建立参考；
- Piper 末端反馈 Hz 是否稳定；
- 工作空间边界是否出现突兀贴边运动；
- trigger 方向是否仍为 0=开、1=闭。

## 12. 回退

新脚本为独立文件，停止 `pico_teleop_piper_fixed.py` 即可停止使用新路径。旧 `pico_teleop_piper.py` 和 `pico_teleop_improved.py` 完全保留。

注意：旧 `pico_teleop_piper.py` 启动会自动移动到固定 home，并保留 MOVEL、固定姿态和无 timestamp 等旧行为。因此旧文件可用于代码回溯，但不能在未知现场直接当作安全回退命令运行。

控制实现 commit 为 `043b32740fb70110f9367a722cae9596494dcaba`。如需撤销，应由维护人员在确认依赖后使用常规 `git revert`。本次未执行任何 destructive Git 操作。

## 13. 剩余风险

以下只能通过真实硬件确认：

- Pico 实际安装 yaw；
- Piper 固件是否在 MOVEP 下稳定覆盖旧目标；
- 0.08 m/s 和 40% 是否适合当前负载；
- 三组末端反馈 CAN 帧与反馈 Hz 是否稳定；
- 既有工作空间是否适合当前安装；
- 0.2 s stale 阈值是否适合现场 XR 更新率；
- 速度限制引入的短距离正常滞后是否可接受；
- 夹爪实际行程是否仍为 70 mm。

这些参数应基于 `--safe-test` 现场记录做小幅调整，不建议在本轮基础上立即扩展为 ROS2、Placo 或复杂预测控制重构。

## 14. 结论

本次修复直接处理了坐标轴错误、比例过激、`max_delta` 语义错误、MOVEL 路径积累、控制周期漂移、重复/过期 XR 帧、固定末端姿态和 grip 抖动。

软件侧最小修复与无硬件检查已经完成。最终现场效果应以三项核心验收现象为准：三轴方向、停止后追赶、重新 grip 无跳变。
