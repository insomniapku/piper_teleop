# Piper XR 遥操作仓库版本与操作手册

> 本文档不包含密码、令牌或其他凭据。

## 1. 版本结论

仓库内代码按用途分成三类。操作者不要根据文件名猜测，按下表选择。

| 类别 | 状态 | 入口 | 用途 |
|---|---|---|---|
| V1 Cartesian Stable | 当前正式版 | `run_piper_normal_teleop.sh` | 日常真机遥操作 |
| V1 Safe Test | 当前安全测试版 | `run_piper_safe_teleop.sh` | 首次连机、CAN 或方向验证 |
| V2 Joint IK | 已真机测试，仍在现场调参 | `pico_teleop_piper_ik_v2.py` | 四元数末端旋转、Pinocchio IK、关节连续控制 |
| Legacy / Experiments | 仅供参考 | 旧 Python/ROS/IK 文件 | 不作为正式启动入口 |

### V1 已冻结基线

- 分支：`snapshot/piper-teleop-v1-working-20260826`
- 标签：`piper-teleop-v1-working-20260826`
- 提交：`6b2b18b08d394118fcfdc473df25d957c9fe27b5`
- 工作站目录：`/home/zktitan/piper_teleop_latency`
- 核心程序：`pico_teleop_piper_fixed.py`

V1 当前行为：

- 左手手柄控制左机械臂；
- 位置比例 `0.8`；
- Piper 硬件速度百分比 `100`；
- 不启用 Python 软件速度限制；
- 不启用 Python XYZ 工作空间裁剪；
- 夹爪为二值控制，只允许完整张开或完整闭合；
- 末端姿态在按下 Grip 时锁定，暂不跟随手柄旋转；
- 通过 Piper `EndPoseCtrl` 发送笛卡尔目标。

### V2 开发基线

- 分支：`feature/piper-ik-orientation-v2`
- 工作站目录：`/home/zktitan/piper_teleop_ik_v2`
- 起点：V1 冻结提交 `6b2b18b`
- 当前状态：已完成离线门槛和操作者在场真机测试；仍保留独立入口，V1 作为回退。

V2 目标：保持 XR 姿态为四元数/旋转矩阵，使用 Piper URDF 与 Pinocchio
求解 J1-J6，并使用上一帧关节解保证连续。位置为主要任务，姿态为软任务，
不再把手柄旋转直接转换成 `RX/RY/RZ` 欧拉角。

### Legacy 文件

下列文件可能包含历史实验或旧架构，保留用于追溯，但不应直接启动真机：

- `pico_teleop_piper.py`
- `pico_teleop_improved.py`
- `piper_teleop/ik_solver.py`
- `piper_teleop/ik_solver_placo.py`
- `piper_teleop/ik_solver_placo_improved.py`
- `piper_teleop/` 下旧 ROS 节点和重复副本

旧 PyKDL 求解器含硬编码运动链；旧 Placo 实验曾记录 Piper URDF 上的收敛问题。
除非正在做回归研究，不要将它们连接 CAN 真机。

## 2. 操作前安全检查

1. 机械臂周围清空人员、线缆和易碎物。
2. 操作者能够立即触及急停。
3. 夹爪中没有未固定物体。
4. 同一时间只能运行一个遥操作/SDK 控制进程。
5. 不要在正式遥操作运行时并行执行 `test_piper_*.py` 或 SDK MoveJ/JointCtrl 示例。
6. 首次恢复连接或改过代码后，先运行 Safe Test，不要直接运行正式版。

## 3. 从 Mac 登录工作站

```bash
ping 192.168.111.123
ssh zktitan@192.168.111.123
```

如果 Ping 和 SSH 都超时：

- 确认 Linux 工作站已开机且未休眠；
- 确认 Mac 与工作站处于可互通的局域网/VPN；
- 在 Mac 上检查 `route -n get 192.168.111.123`；
- 不要在网络不可达时重复启动新的遥操作进程。

## 4. Pico 与 PC Service

1. 在 Linux 上启动 XRoboToolkit PC Service：

   ```bash
   /opt/apps/roboticsservice/runService.sh
   ```

   如果已通过桌面图标运行，则不要重复启动第二个实例。

2. 在 Pico 中打开 XRoboToolkit 应用。
3. 输入 PC Service 当前显示或日志给出的端口。

端口不要写死。历史调试中出现过 `13579` 和 `63901`，但实际操作必须以当前
PC Service 显示的端口为准。

4. 保持 Pico 应用在前台，确认 Linux 日志持续收到新的手柄姿态，而不是只显示
   “connected” 后数据变旧。

## 5. 检查 CAN

```bash
ip -details link show can0
```

应看到 `can0` 存在且为 `UP`。如果 CAN 尚未配置，使用仓库/SDK 已验证的 Piper
CAN 激活脚本；不要在不了解适配器名称和波特率时自行猜测配置。

## 6. 首次或恢复后的 Safe Test

```bash
cd /home/zktitan/piper_teleop_latency
./run_piper_safe_teleop.sh
```

Safe Test 特性：

- 硬件速度百分比 20%；
- 小范围位移；
- 不控制夹爪；
- 用于确认 Pico 数据、方向、CAN 和真机响应。

程序要求确认时，观察机械臂环境后再按提示输入确认词。测试时先做小幅单轴动作，
依次确认前后、左右、上下。

## 7. 启动当前正式版 V1

```bash
cd /home/zktitan/piper_teleop_latency
./run_piper_normal_teleop.sh
```

等价的完整命令为：

```bash
.venv/bin/python -u pico_teleop_piper_fixed.py \
  --hardware \
  --controller-hand left \
  --position-scale 0.8 \
  --no-speed-limit \
  --no-workspace-limit \
  --binary-gripper \
  --can-name can0 \
  --speed-percent 100 \
  --yaw-deg 0
```

正常情况下优先使用启动脚本，避免手工漏掉参数。

## 8. 手柄操作

- 左手 Grip：按住时启用机械臂跟随；松开时停止更新目标并重新允许手柄归位。
- 每次 Grip 从松开到按下，程序以真机当前末端位置作为新参考，首帧不应跳动。
- 左手 Trigger：二值夹爪控制；松开为完整张开，按下为完整闭合。
- 当前 V1 不跟随手柄旋转；夹爪姿态保持 Grip 激活时的末端姿态。

建议动作顺序：

1. 松开 Grip，把手柄移到舒服的位置；
2. 按住 Grip；
3. 缓慢移动手柄，确认机械臂方向正确；
4. 到达手柄活动边缘时松开 Grip，手柄归位后重新按住；
5. 使用 Trigger 开闭夹爪。

## 9. 停止程序

1. 先松开 Grip；
2. 确认机械臂静止；
3. 在 SSH 终端按 `Ctrl-C`；
4. 检查没有残留控制进程：

   ```bash
   pgrep -af pico_teleop_piper
   ```

不要使用宽泛的 `pkill python`。如果确有残留，只处理输出中明确对应本仓库的 PID。

### V2 取消额外逐帧关节限速

现场确认 1°/帧输出限幅会在快速操作时频繁介入。需要关闭这层额外限幅时，V2
启动命令加入：

```text
--no-joint-step-limit
```

它不能与 `--safe-test` 同时使用。关闭后仍然存在：J1-J6 的 URDF/SDK/机械限位、
Piper 固件速度和加速度保护、IK 不可达目标保持、XR 超过 0.2 秒不更新即停止，
以及 Grip clutch。关闭逐帧限幅并不等于取消机械臂硬限制。

## 10. 常见问题

### Pico 显示连接，但机械臂不动

- 检查 Pico 应用是否在前台；
- 检查输入的是否为 PC Service 当前端口；
- 检查日志中的姿态时间戳是否持续更新；
- 确认使用左手手柄并且 Grip 超过激活阈值；
- 检查遥操作进程是否仍在运行。

### SDK 测试能动，但遥操作不动

这通常说明 CAN/机械臂通信正常，问题位于 Pico 数据、Grip 激活、坐标映射或
遥操作进程。不要继续重复发送 SDK 大动作。

### 左右或前后反向

正式 V1 已验证参数是 `--controller-hand left --yaw-deg 0`。先检查是否启动了错误
脚本或手工传了不同参数，不要立刻修改多处符号。

### 夹爪只张开一半

确认正式命令包含 `--binary-gripper`。V1 的完整张开目标是 Piper 当前配置允许的
最大开度，不应使用旧的连续半开映射。

### 末端无法旋转

这是 V1 当前的设计，不是连接故障。末端四元数旋转将在 V2 经 IK 验证后提供。
不要临时给 V1 加 `--follow-orientation` 做正式操作，因为该路径仍会转换成欧拉角。

## 11. V2 开发和发布门槛

V2 必须依次通过：

1. URDF 与 J1-J6 限位验证；
2. FK/IK 一致性测试；
3. 小位移与混合位姿离线测试；
4. 四元数符号等价与单位转换测试；
5. Pico 录制轨迹 dry-run，确认关节目标连续；
6. 无 CAN 的长时间运行测试；
7. 操作者在场的低速、小范围真机测试；
8. 验证通过后才能新增 `run_piper_v2_teleop.sh` 正式入口。

完成上述门槛后可标注 `EXPERIMENTAL / LIVE TESTED`；在现场调参稳定并形成固定启动
脚本之前，不应替换 V1 的 `SUPPORTED` 回退入口。

## 12. 建议的仓库整理方式

为避免破坏现有导入路径，第一阶段不要大规模移动 Python 文件。先增加：

```text
README.md                    # 只保留项目入口和正式启动方式
VERSIONS.md                  # 各版本状态、分支、提交和适用范围
docs/OPERATIONS_ZH.md        # 本操作手册
docs/ARCHITECTURE_V2.md      # V2 四元数/IK 架构
run_piper_normal_teleop.sh   # 唯一正式 V1 入口
run_piper_safe_teleop.sh     # 唯一 V1 安全测试入口
```

根 README 应醒目标记：

- `SUPPORTED`：两个 V1 启动脚本；
- `EXPERIMENTAL`：V2 dry-run；
- `LEGACY`：其余历史入口，不保证真机安全。

第二阶段在测试覆盖后，才将旧文件移动到 `legacy/`，避免一次整理同时改变运行行为。
