# Piper Teleop 问题诊断报告

## 发现的问题

### 🔴 问题1: Workspace Limit配置不合理
**现象**: IK成功率0%，所有求解都失败

**根本原因**:
- 初始EE位置: `x=-0.018m, y=-0.018m, z=0.226m` (在基座附近)
- 工作空间限制: `x_min=0.1m`
- **初始位置在工作空间外！**

**影响**:
```python
# mapper.set_reference() 时
target_pos = [-0.018, -0.018, 0.226]  # 初始EE位置
# apply_workspace_limit() 强制限制到
target_pos_clamped = [0.1, -0.018, 0.226]  # x被截断到0.1

# IK求解一个距离当前位置0.118m远的突变目标 → 失败
```

**解决方案**: 
1. 调整工作空间限制以包含初始位置
2. 或者在设置参考时不应用工作空间限制

---

### 🔴 问题2: 使用PyKDL而不是Placo

**teleop_a7lite的做法** (正确):
```python
# 使用Placo进行IK
self.placo_robot = placo.RobotWrapper(urdf_path)
self.solver = placo.KinematicsSolver(self.placo_robot)
task = self.solver.add_frame_task(link_name, T_target)
self.solver.solve(True)  # 迭代求解，自动处理约束
```

**piper_teleop的做法** (问题):
```python
# 使用PyKDL
self.ik_pos_solver = ChainIkSolverPos_NR(...)  # Newton-Raphson
result = self.ik_pos_solver.CartToJnt(q_init, target_frame, q_out)
```

**对比**:
| 特性 | Placo | PyKDL |
|------|-------|-------|
| 算法 | 约束优化QP求解器 | Newton-Raphson |
| 收敛性 | 优秀 | 一般，依赖seed |
| 关节限制 | 自动软约束 | 需手动后验证 |
| 正则化 | 内置关节正则化 | 无 |
| 奇异点处理 | 自动处理 | 容易卡住 |

---

### 🔴 问题3: 架构不一致

**teleop_a7lite架构**:
```
XRClient → BaseTeleopController (Placo) → Hardware
                ↓
         _update_ik() 使用Placo在线求解
         持续更新 placo_robot.state.q
```

**piper_teleop架构**:
```
VR Input → TeleopMapper → IK Solver (PyKDL) → Joint Commands
                            ↓
                    每次调用独立求解
                    无状态持续性
```

**差异**:
- A7Lite: Placo维护机器人状态，增量式求解
- Piper: 每次IK都是独立的求解问题

---

## 解决方案

### 方案A: 切换到Placo (推荐)

**优点**:
- ✅ 与teleop_a7lite架构一致
- ✅ 更好的IK收敛性
- ✅ 自动处理约束和正则化
- ✅ 增量式求解更稳定

**实施**:
1. 创建 `PiperPlacoIKSolver` 替代 `PiperIKSolver`
2. 使用 `placo.RobotWrapper` 和 `placo.KinematicsSolver`
3. 保持状态连续性

### 方案B: 修复现有PyKDL实现

**优点**:
- ✅ 保持现有代码结构
- ✅ 无需额外依赖

**实施**:
1. 修复工作空间限制配置
2. 改进seed策略
3. 添加更多重试机制

---

## 立即可修复的问题

### 修复1: 工作空间限制
```yaml
# config/teleop_config.yaml
safety:
  workspace_limit:
    x_min: -0.1   # 改为-0.1 (包含初始位置)
    x_max: 0.6
    y_min: -0.4
    y_max: 0.4
    z_min: 0.0    # 改为0.0 (包含z=0.226)
    z_max: 0.8
```

### 修复2: 不在set_reference时应用workspace limit
```python
# teleop_mapping.py 的 set_reference() 方法
# 移除这行:
# if self.enable_workspace_limit:
#     ee_pos, _ = apply_workspace_limit(ee_pos, self.workspace_limit)
```

---

## 推荐实施路径

### 阶段1: 快速修复 (立即)
1. ✅ 修复workspace limit配置
2. ✅ 在set_reference时不应用workspace limit
3. ✅ 测试IK成功率改善

### 阶段2: 切换到Placo (完整解决)
1. ✅ 实现 PiperPlacoIKSolver
2. ✅ 集成到 TeleopController
3. ✅ 对比测试性能

### 阶段3: 重构为统一架构
1. ✅ 使用 BaseTeleopController 基类
2. ✅ 实现 PiperHardwareTeleopController
3. ✅ 与 A7LiteTeleopController 保持一致

---

## 测试验证

### 验证IK修复:
```python
# 测试1: FK→IK一致性
seed = [0.0, 0.8, -0.8, 0.0, 0.0, 0.0]
pos, quat = solver.compute_fk(seed)
result = solver.compute_ik(pos, quat, seed)
assert result is not None  # 应该成功

# 测试2: 初始位置在工作空间内
initial_ee_pos = [-0.018, -0.018, 0.226]
assert workspace['x_min'] <= initial_ee_pos[0] <= workspace['x_max']

# 测试3: 完整链路
controller = PiperTeleopController(urdf_path)
result = controller.update(vr_pos, vr_quat)
assert result['success'] == True
assert result.get('is_fallback', False) == False  # 不应该使用fallback
```
