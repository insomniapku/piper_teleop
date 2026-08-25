# Piper Teleoperation

ROS2 package for teleoperating Piper robot using Pico XR controller.

## Development Status

### M1: Package Structure ✓ (Current)
- [x] Created independent ROS2 package
- [x] Configuration system (YAML-based)
- [x] Basic module structure
- [x] Launch files

### M2-M9: To Be Implemented
- [ ] M2: Pico /xr_data interface
- [ ] M3: Teleoperation mapping (delta pose)
- [ ] M4: IK solver (Placo or alternative)
- [ ] M5: Dry-run simulation mode
- [ ] M6: Piper hardware interface
- [ ] M7: Safety mechanisms
- [ ] M8: Real hardware testing
- [ ] M9: Demonstration data recorder

## Package Structure

```
piper_teleop/
├── config/
│   └── teleop_config.yaml          # Main configuration file
├── launch/
│   └── teleop.launch.py            # Launch file
├── piper_teleop/
│   ├── __init__.py
│   ├── piper_teleop_node.py        # Main node
│   ├── xr_interface.py             # XR input (M2)
│   ├── teleop_mapping.py           # Delta mapping (M3)
│   ├── ik_solver.py                # IK solver (M4 - TBD)
│   ├── piper_interface.py          # Hardware interface (M6 - TBD)
│   └── data_recorder.py            # Data recorder (M9 - TBD)
├── package.xml
├── setup.py
└── README.md
```

## Configuration

All parameters are controlled via `config/teleop_config.yaml`:

- **XR Input**: Topic names, controller selection
- **Position Mapping**: Scale, max delta, coordinate transforms
- **Rotation Mapping**: Scale, max delta
- **IK Settings**: Solver parameters, joint limits
- **Safety**: Workspace limits, velocity limits, timeouts
- **Gripper**: Trigger thresholds, positions
- **Debug**: Visualization, logging

## Building

```bash
cd ~/lcz0820
colcon build --packages-select piper_teleop
source install/setup.bash
```

## Running (M1 - Basic Test)

```bash
ros2 launch piper_teleop teleop.launch.py
```

## Dependencies

- ROS2 Humble
- Python 3.10+
- numpy
- scipy
- pyyaml

## Design Principles

1. **No modifications to piper_ros**: Independent package
2. **Delta/reference mapping**: No direct pose mapping
3. **YAML configuration**: All parameters configurable
4. **Safety first**: Multiple safety layers before hardware
5. **Staged development**: M1 → M9 sequential implementation

## Topics

### Subscribed (Planned)
- `/xr_data`: XR controller data (M2)
- `/joint_states_feedback`: Piper joint feedback (M6)
- `/end_pose_stamped`: Piper EE pose (M6)

### Published (Planned)
- `/joint_ctrl_single`: Piper joint commands (M6)
- `/piper_teleop/target_pose`: Target EE pose (debug)
- `/piper_teleop/joint_target`: Target joint angles (debug)

## Safety Features (M7)

- Workspace limits
- Joint limits
- Velocity limits
- Command timeout/watchdog
- Tracking loss protection
- IK failure protection
- Emergency stop

## Author

Created for Pico XR → Piper teleoperation system
