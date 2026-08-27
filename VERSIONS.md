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
- 状态：`EXPERIMENTAL / NO LIVE HARDWARE`
- 起点：V1 验证提交 `6b2b18b`

目标架构：

```text
Pico relative position + quaternion
                 -> base-frame SE(3) target
                 -> Pinocchio position-priority / soft-orientation IK
                 -> continuous J1-J6 targets
                 -> Piper JointCtrl
```

在 V2 完成离线轨迹、关节限位、不可达目标、单位转换和操作者在场的低速验证前，
仓库不得提供 V2 正式真机启动脚本。

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
