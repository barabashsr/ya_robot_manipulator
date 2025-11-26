# Joy Control Package

Configurable joystick control package for robotic manipulators with trajectory controller support.

## Architecture (Updated for Trajectory Controllers)

The joy_control package uses a hybrid controller architecture:

```
┌─────────────────────────────────────────────────────────────────────────┐
│                    SMOOTH JOYSTICK CONTROL ARCHITECTURE                  │
├─────────────────────────────────────────────────────────────────────────┤
│                                                                          │
│  Joy Callback (20Hz)                   Trajectory Timer (10Hz)          │
│        │                                       │                         │
│        ▼                                       ▼                         │
│  Read joystick axes                   Send accumulated targets          │
│        │                              as trajectory goals               │
│        ▼                                       │                         │
│  Calculate velocity                            ▼                         │
│  (axis × velocity_scale)              Trajectory controller             │
│        │                              interpolates smoothly             │
│        ▼                              (spline interpolation)            │
│  Accumulate target position                    │                         │
│  (current + velocity × dt)                     ▼                         │
│        │                              Robot moves smoothly              │
│        ▼                              (no jerky steps)                  │
│  Clamp to limits                                                        │
│  (from manipulator_params.yaml)                                         │
│                                                                          │
└─────────────────────────────────────────────────────────────────────────┘
```

### Controller Types

| Joint Type | Controller | Interface | Behavior |
|------------|------------|-----------|----------|
| 7 motion joints | JointTrajectoryController | Action | Smooth interpolated motion |
| 2 container jaws | ForwardCommandController | Topic | Instant response |

### Motion Joints (Trajectory - Smooth)

These joints use `FollowJointTrajectory` action for smooth, interpolated motion:

- `base_main_frame_joint` - Base railway X-axis
- `main_frame_selector_frame_joint` - Selector vertical Z-axis
- `selector_frame_gripper_joint` - Gripper Y-axis
- `selector_frame_picker_frame_joint` - Picker vertical Z-axis
- `picker_frame_picker_rail_joint` - Picker Y-axis
- `picker_rail_picker_base_joint` - Picker X-axis slider
- `picker_base_picker_jaw_joint` - Picker jaw

### Container Jaws (ForwardCommand - Instant)

These joints use ForwardCommandController for instant response:

- `selector_left_container_jaw_joint`
- `selector_right_container_jaw_joint`

## Joint Limits (Single Source of Truth)

Joint limits are loaded at runtime from `manipulator_params.yaml`.
**DO NOT** add limits to `manipulator_joy_config.yaml`.

```yaml
# WRONG - Do not duplicate limits
axis_mappings.base_main_frame.limits: [0.0, 4.0]  # REMOVE THIS

# CORRECT - limits loaded automatically from manipulator_params.yaml
axis_mappings.base_main_frame.joint_name: "base_main_frame_joint"
```

### Soft Limits (from manipulator_params.yaml)

| Joint | Min | Max | Velocity |
|-------|-----|-----|----------|
| base_main_frame_joint | 0.1 | 3.9 | 2.0 |
| main_frame_selector_frame_joint | 0.05 | 1.45 | 2.0 |
| selector_frame_gripper_joint | -0.39 | 0.39 | 2.0 |
| selector_frame_picker_frame_joint | 0.005 | 0.29 | 2.0 |
| picker_frame_picker_rail_joint | -0.29 | 0.29 | 2.0 |
| picker_rail_picker_base_joint | 0.005 | 0.24 | 2.0 |
| picker_base_picker_jaw_joint | 0.005 | 0.19 | 2.0 |
| selector_left_container_jaw_joint | -0.19 | 0.19 | 2.0 |
| selector_right_container_jaw_joint | -0.19 | 0.19 | 2.0 |

## Features

- **Trajectory Controller Support** - 7 motion joints use smooth trajectory interpolation
- **Dual-Mode Architecture** - Container jaws use instant ForwardCommand
- **Single Source of Truth** - Limits loaded from manipulator_params.yaml
- **YAML-based configuration** - All mappings defined in YAML files
- **Axis mappings** - Map joystick axes to joint controllers
- **Button actions** - Picker jaw and container jaw control
- **Safety features** - Enable button (L1), turbo mode (R1)
- **Configurable deadzones** - Per-axis or global
- **Velocity scaling** - Individual speed limits per joint
- **Goal Preemption** - Previous trajectory cancelled for responsive control

## Quick Start

```bash
# Build the package
colcon build --packages-select joy_control

# Source workspace
source install/setup.bash

# Launch with simulation (joy enabled)
ros2 launch manipulator_control manipulator_simulation.launch.py enable_joy:=true
```

## Configuration

Edit `config/manipulator_joy_config.yaml` to customize:

### Global Settings

```yaml
joy_controller_node:
  ros__parameters:
    update_rate: 20.0                # Joy callback rate (Hz)
    trajectory_update_rate: 10.0     # Trajectory goal send rate (Hz)
    scale_linear: 0.5                # Max speed scale
    deadzone: 0.1                    # Global deadzone
    enable_button: 4                 # L1 - hold to enable
    enable_turbo_button: 5           # R1 - hold for 2x speed
```

### New Parameters (Trajectory Control)

| Parameter | Default | Description |
|-----------|---------|-------------|
| trajectory_update_rate | 10.0 | Trajectory goal send rate (Hz) |
| trajectory_duration_min | 0.05 | Minimum trajectory duration (s) |
| trajectory_duration_max | 2.0 | Maximum trajectory duration (s) |

### Axis Mappings

```yaml
axis_mappings:
  base_main_frame:
    joint_name: "base_main_frame_joint"  # Required - full joint name
    axis_index: 1                         # Which joystick axis
    axis_scale: -1.0                      # Invert if needed
    velocity_scale: 0.5                   # Individual speed limit
    # NOTE: limits field REMOVED - loaded from manipulator_params.yaml
```

### Button Actions

```yaml
button_actions:
  picker_jaw_extend:
    button_index: 2          # Triangle
    action_type: "hold"      # Hold button to move
    joint: "picker_base_picker_jaw"
    target_position: 0.19    # Using soft limit
```

## Controller Layout (PlayStation)

### Axes (Movement)

| Control | Joint | Controller Type |
|---------|-------|-----------------|
| Left Stick Y | base_main_frame_joint | Trajectory |
| Left Stick X | picker_rail_picker_base_joint | Trajectory |
| Right Stick Y | main_frame_selector_frame_joint | Trajectory |
| Right Stick X | selector_frame_gripper_joint | Trajectory |
| D-Pad Y | selector_frame_picker_frame_joint | Trajectory |
| D-Pad X | picker_frame_picker_rail_joint | Trajectory |

### Buttons (Actions)

| Button | Action | Joint | Controller Type |
|--------|--------|-------|-----------------|
| L1 | Enable control (hold) | - | - |
| R1 | Turbo mode (hold) | - | - |
| Triangle | Picker jaw extend | picker_base_picker_jaw_joint | Trajectory |
| Cross | Picker jaw retract | picker_base_picker_jaw_joint | Trajectory |
| L2 | Container jaws open | Both container jaws | ForwardCommand |
| R2 | Container jaws close | Both container jaws | ForwardCommand |

## Usage Examples

### Default Configuration with Joy

```bash
ros2 launch manipulator_control manipulator_simulation.launch.py enable_joy:=true
```

### Standalone Joy Control

```bash
ros2 launch joy_control joy_control.launch.py
```

### Custom Joystick Device

```bash
ros2 launch joy_control joy_control.launch.py joy_dev:=/dev/input/js1
```

## Testing

### Verify Joystick Connection

```bash
ls /dev/input/js*
ros2 run joy joy_node
ros2 topic echo /joy
```

### Verify Action Clients (Trajectory Joints)

```bash
# Should show 7 action clients for trajectory joints
ros2 node info /joy_controller_node | grep "Action Clients"
```

### Verify Publishers (Forward Command Joints)

```bash
# Container jaw topics
ros2 topic echo /selector_left_container_jaw_joint_controller/commands
ros2 topic echo /selector_right_container_jaw_joint_controller/commands
```

### Monitor Trajectory Actions

```bash
# Watch trajectory actions being sent
ros2 action list | grep follow_joint_trajectory
```

## Troubleshooting

### Motion is still jerky

- Verify trajectory controllers are active: `ros2 control list_controllers`
- Check `trajectory_update_rate` is not too high (10Hz recommended)
- Ensure simulation is running with trajectory controllers

### Joint doesn't respond

- Check action server is available: `ros2 action list | grep follow_joint_trajectory`
- Verify `joint_name` in config matches actual joint name
- Ensure joint limits were loaded: check node logs for "Loaded limits for X joints"

### Limits seem wrong

- Limits are loaded from `manipulator_params.yaml`, not config file
- Check `ros2 topic echo /joint_states` for actual positions
- Verify `manipulator_description` package is built and installed

### Container jaws don't respond instantly

- Container jaws should use ForwardCommand (not trajectory)
- Check they're correctly routed to `/commands` topic, not action
- Verify button indices 6 (L2) and 7 (R2) are correct

### Controller not detected

```bash
ls -l /dev/input/js0
sudo usermod -a -G input $USER
# Log out and back in
```

### Control not working

- Make sure to **hold L1** button to enable
- Check that controllers are active: `ros2 control list_controllers`
- Verify joint states are being received: check logs for "Initial joint positions received"

## License

MIT

## Author

Generated for ya_robot_manipulator project
