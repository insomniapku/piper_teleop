# Teleoperation Versions

## V1 Cartesian Stable

这是当前唯一允许日常连接真机的正式版本。

- 发布分支：`release/piper-teleop-v1`
- 原样快照分支：`snapshot/piper-teleop-v1-working-20260826`
- 原样标签：`piper-teleop-v1-working-20260826`
- 验证提交：`6b2b18b08d394118fcfdc473df25d957c9fe27b5`
- 核心程序：`pico_teleop_piper_fixed.py`
- 正式入口：`run_piper_normal_teleop.sh`
- 安全入口：`run_piper_safe_teleop.sh`

真机验证配置：

```text
left controller -> left Piper arm
position scale  = 0.8
hardware speed  = 100%
software speed limit    = off
software XYZ workspace  = off
gripper          = binary full-open/full-close
orientation      = held at clutch activation
```

V1 使用 Piper `EndPoseCtrl`。代码中存在可选的实验性姿态映射参数，但正式启动
脚本不启用它们。不要为正式操作临时添加 `--follow-orientation`。

## V1 Safe Test

Safe Test 与 V1 使用相同核心控制器，但采用低风险参数：

- Piper 硬件速度 20%；
- 3 cm clutch 半径与较低映射速度；
- 不控制夹爪。

它是恢复网络、CAN、Pico 或代码修改后的第一启动入口，不是单独的产品版本。

## V2 Joint IK

V2 是新的末端旋转/关节 IK 开发线。

- 分支：`feature/piper-ik-orientation-v2`
- 状态：`EXPERIMENTAL / LIVE TESTED`，仍在现场调参
- 起点：V1 验证提交 `6b2b18b`

目标架构：

```text
Pico relative position + quaternion
                 -> base-frame SE(3) target
                 -> Pinocchio position-priority / soft-orientation IK
                 -> continuous J1-J6 targets
                 -> Piper JointCtrl
```

已完成 9 项 IK 单元测试、121 帧连续轨迹 dry-run、完整控制链 dry-run，以及
操作者在场的低速/正式参数真机测试。V2 仍保留为独立实验入口，不替换 V1 回退版。

现场调参将正式逐帧关节输出上限从 1°提高为 2°，减少快速手柄运动时的滞后。
`--no-joint-step-limit` 仅保留用于离线诊断，不应连接真机；完全取消该保护曾导致
raw IK 大跳步和机械臂接近硬限位。真实关节角限制、Piper 固件保护、IK 不可达保持、
XR 0.2 秒超时和 Grip clutch 均继续生效。

## Legacy / Experiments

以下代码保留用于历史追溯，不作为正式入口：

- `pico_teleop_piper.py`
- `pico_teleop_improved.py`
- `piper_teleop/ik_solver.py`
- `piper_teleop/ik_solver_placo.py`
- `piper_teleop/ik_solver_placo_improved.py`
- 旧 ROS 节点和重复包目录

## 发布规则

1. V1 标签保持不可变。
2. 真机可用入口必须位于根目录启动脚本，并在 README 标为 `SUPPORTED`。
3. 实验代码必须标为 `EXPERIMENTAL / NO LIVE HARDWARE`。
4. 不提交 `.venv/`、日志、录制数据、密码、令牌或主机专用凭据。
5. 同一 CAN 总线上不得同时运行多个控制程序。
6. 完整操作方法见 [`docs/OPERATIONS_ZH.md`](docs/OPERATIONS_ZH.md)。
