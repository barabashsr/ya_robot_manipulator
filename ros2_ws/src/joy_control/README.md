# Joy Control Package

Configurable joystick control package for robotic manipulators.

## Features

- ✅ **YAML-based configuration** - All mappings defined in YAML files
- ✅ **Axis mappings** - Map joystick axes to joint controllers
- ✅ **Button actions** - Reserve buttons for future actions/services
- ✅ **Safety features** - Enable button, turbo mode
- ✅ **Configurable deadzones** - Per-axis or global
- ✅ **Velocity scaling** - Individual speed limits per joint
- ✅ **Limit enforcement** - Automatic clamping to joint limits

## Quick Start

```bash
# Build the package
colcon build --packages-select joy_control

# Source workspace
source install/setup.bash

# Launch with default config
ros2 launch joy_control joy_control.launch.py
```

## Configuration

Edit `config/manipulator_joy_config.yaml` to customize:

### Global Settings

```yaml
joy_control:
  ros__parameters:
    update_rate: 20.0        # Hz
    scale_linear: 0.5        # Max speed scale
    scale_angular: 0.3
    deadzone: 0.1            # Global deadzone
    enable_button: 4         # L1 - hold to enable
    enable_turbo_button: 5   # R1 - hold for 2x speed
```

### Axis Mappings

```yaml
axis_mappings:
  base_main_frame:
    controller_topic: "/base_main_frame_joint_controller/commands"
    axis_index: 1            # Which joystick axis
    axis_scale: -1.0         # Invert if needed
    velocity_scale: 0.5      # Individual speed limit
    limits: [0.0, 4.0]       # Joint limits
```

### Button Actions

```yaml
button_actions:
  picker_jaw_extend:
    button_index: 2          # Triangle
    action_type: "hold"      # "press", "hold", "toggle"
    joint: "picker_base_picker_jaw"
    target_position: 0.2
```

## Controller Layout (PlayStation)

### Axes (Movement)
- **Left Stick ↕**: Base railway
- **Left Stick ↔**: Picker X-slider
- **Right Stick ↕**: Selector vertical
- **Right Stick ↔**: Gripper horizontal
- **L2/R2 Triggers**: Container jaws
- **D-Pad ↕**: Picker vertical
- **D-Pad ↔**: Picker horizontal

### Buttons (Actions - Reserved)
- **L1**: Enable control (hold)
- **R1**: Turbo mode (hold)
- **Triangle**: Picker jaw extend
- **Cross**: Picker jaw retract
- **Square**: Home all (future)
- **Circle**: Emergency stop (future)
- **Options**: Store position (future)

## Usage Examples

### Default Configuration

```bash
ros2 launch joy_control joy_control.launch.py
```

### Custom Config File

```bash
ros2 launch joy_control joy_control.launch.py \
  config_file:=/path/to/my_config.yaml
```

### Custom Joystick Device

```bash
ros2 launch joy_control joy_control.launch.py \
  joy_dev:=/dev/input/js1
```

## Testing

### Verify Joystick Connection

```bash
ls /dev/input/js*
ros2 run joy joy_node
ros2 topic echo /joy
```

### Monitor Commands

```bash
# Watch all controller commands
ros2 topic list | grep commands

# Echo specific controller
ros2 topic echo /base_main_frame_joint_controller/commands
```

## Creating Custom Configurations

1. Copy the default config:
```bash
cp config/manipulator_joy_config.yaml config/my_robot_config.yaml
```

2. Edit axis indices to match your controller
3. Adjust velocity scales and limits
4. Map buttons to your desired actions

5. Launch with custom config:
```bash
ros2 launch joy_control joy_control.launch.py \
  config_file:=$(ros2 pkg prefix joy_control)/share/joy_control/config/my_robot_config.yaml
```

## Adding New Button Actions

Future button actions can be added by:

1. Adding configuration in YAML:
```yaml
button_actions:
  my_custom_action:
    button_index: 8
    action_type: "press"
    service_name: "/my/service"
    enabled: true
```

2. Implementing the action in `scripts/joy_controller_node.py`

## Troubleshooting

**Controller not detected:**
```bash
ls -l /dev/input/js0
sudo usermod -a -G input $USER
# Log out and back in
```

**Axes/buttons don't match:**
- Run `ros2 topic echo /joy` and check indices
- Update YAML configuration accordingly

**Control not working:**
- Make sure to **hold L1** button to enable
- Check that controllers are active: `ros2 control list_controllers`

## Architecture

```
┌─────────────┐
│  Joystick   │
│  Hardware   │
└──────┬──────┘
       │
       ▼
┌─────────────┐
│  joy_node   │  (reads hardware)
└──────┬──────┘
       │ /joy topic
       ▼
┌─────────────────────┐
│ joy_controller_node │  (YAML-configured mapping)
└──────┬──────────────┘
       │
       ├─► /controller1/commands
       ├─► /controller2/commands
       └─► /controllerN/commands
```

## License

MIT

## Author

Generated for ya_robot_manipulator project
