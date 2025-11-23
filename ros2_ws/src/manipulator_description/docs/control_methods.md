# Manipulator Control Methods

This document describes different methods to control the manipulator.

## Prerequisites

Make sure the manipulator is running:
```bash
source install/setup.bash
ros2 launch manipulator_description manipulator_control.launch.py
```

---

## Method 1: Command Line (Manual Testing)

Send individual joint commands via command line:

```bash
# Move base railway to 2.0m
ros2 topic pub /base_main_frame_joint_controller/commands \
  std_msgs/msg/Float64MultiArray "data: [2.0]" --once

# Raise selector to 0.5m
ros2 topic pub /main_frame_selector_frame_joint_controller/commands \
  std_msgs/msg/Float64MultiArray "data: [0.5]" --once

# Open container jaws
ros2 topic pub /selector_left_container_jaw_joint_controller/commands \
  std_msgs/msg/Float64MultiArray "data: [0.15]" --once

# Move gripper forward
ros2 topic pub /selector_frame_gripper_joint_controller/commands \
  std_msgs/msg/Float64MultiArray "data: [0.2]" --once
```

### Available Controllers

1. `base_main_frame_joint_controller` - Railway X-axis (0.0 to 4.0 m)
2. `main_frame_selector_frame_joint_controller` - Selector Z-axis (0.0 to 1.5 m)
3. `selector_left_container_jaw_joint_controller` - Container jaws (-0.2 to 0.2 m)
4. `selector_frame_gripper_joint_controller` - Gripper Y-axis (-0.4 to 0.4 m)
5. `selector_frame_picker_frame_joint_controller` - Picker Z-axis (0.0 to 0.3 m)
6. `picker_frame_picker_rail_joint_controller` - Picker Y-axis (-0.3 to 0.3 m)
7. `picker_rail_picker_base_joint_controller` - Picker X-axis (0.0 to 0.12 m)
8. `picker_base_picker_jaw_joint_controller` - Picker jaw (0.0 to 0.2 m)

---

## Method 2: GUI Control (RQT Publisher)

Launch the GUI control interface:

```bash
source install/setup.bash
ros2 launch manipulator_description manipulator_gui_control.launch.py
```

This launches:
- **RQT Publisher**: For publishing commands to controllers
- **RQT Reconfigure**: For dynamic parameter adjustment

### Using RQT Publisher:

1. In the RQT Publisher window, click the **dropdown** at the top
2. Select a topic: `/base_main_frame_joint_controller/commands`
3. Set the message type: `std_msgs/msg/Float64MultiArray`
4. Click the **+** button to add the topic
5. Expand the topic and enter values in the `data[0]` field
6. Check the checkbox to enable publishing
7. Click **Publish** or enable auto-publish

**Tip**: You can add multiple topics and control them simultaneously.

### RQT Reconfigure:

Use this to dynamically adjust controller parameters without restarting.

---

## Method 3: PlayStation Controller (Joystick)

Control the manipulator with a PlayStation controller (PS3/PS4/PS5).

### Setup

1. **Connect your controller**:
   - USB: Plug in the controller
   - Bluetooth: Pair via system settings

2. **Verify joystick is detected**:
```bash
ls /dev/input/js*
# Should show: /dev/input/js0 (or js1, js2, etc.)
```

3. **Test joystick input**:
```bash
ros2 run joy joy_node
ros2 topic echo /joy
# Move joystick and press buttons - you should see output
```

### Launch Joystick Control

```bash
source install/setup.bash
ros2 launch manipulator_description manipulator_joy_control.launch.py
```

**With custom device**:
```bash
ros2 launch manipulator_description manipulator_joy_control.launch.py joy_dev:=/dev/input/js1
```

**Adjust speed**:
```bash
ros2 launch manipulator_description manipulator_joy_control.launch.py \
  scale_linear:=0.8 scale_angular:=0.5
```

### Controller Mapping (PlayStation)

| Input | Control | Range |
|-------|---------|-------|
| **Left Stick ↕** | Base Railway (X-axis) | 0.0 to 4.0 m |
| **Right Stick ↕** | Selector Vertical (Z-axis) | 0.0 to 1.5 m |
| **Right Stick ↔** | Gripper Y-axis | -0.4 to 0.4 m |
| **L2 / R2 Triggers** | Container Jaws (close/open) | -0.2 to 0.2 m |
| **D-Pad ↑ / ↓** | Picker Vertical (Z-axis) | 0.0 to 0.3 m |
| **D-Pad ← / →** | Picker Y-axis | -0.3 to 0.3 m |
| **L1 / R1** | Picker X-axis Slider | 0.0 to 0.12 m |
| **△ / ✕ (Triangle/Cross)** | Picker Jaw Extension | 0.0 to 0.2 m |

### Control Tips:

- **Velocity Control**: The joystick controls velocity, not position. The joints will move continuously while you hold the stick/button.
- **Deadzone**: Small movements near the center are ignored (10% deadzone)
- **Safety**: Release all inputs to stop movement
- **Speed Adjustment**: Use launch parameters to adjust speed scales

---

## Method 4: Python Script (Programmatic Control)

Create custom control sequences with Python:

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import time

class ManipulatorController(Node):
    def __init__(self):
        super().__init__('manipulator_controller')

        # Create publishers for each controller
        self.base_pub = self.create_publisher(
            Float64MultiArray,
            '/base_main_frame_joint_controller/commands',
            10
        )
        self.selector_pub = self.create_publisher(
            Float64MultiArray,
            '/main_frame_selector_frame_joint_controller/commands',
            10
        )

    def move_to_position(self, base_pos, selector_pos):
        """Move base and selector to specified positions"""
        base_msg = Float64MultiArray()
        base_msg.data = [base_pos]

        selector_msg = Float64MultiArray()
        selector_msg.data = [selector_pos]

        self.base_pub.publish(base_msg)
        self.selector_pub.publish(selector_msg)

        self.get_logger().info(f'Moving to: base={base_pos}, selector={selector_pos}')

def main():
    rclpy.init()
    controller = Node('manipulator_controller')

    # Example: Move to different positions
    controller.move_to_position(1.0, 0.5)  # Move to position 1
    time.sleep(2.0)

    controller.move_to_position(2.0, 1.0)  # Move to position 2
    time.sleep(2.0)

    controller.move_to_position(0.0, 0.0)  # Return to home

    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

Save as `my_control.py` and run:
```bash
ros2 run your_package my_control.py
```

---

## Method 5: MoveIt Integration (Advanced)

For complex motion planning and trajectory execution, integrate with MoveIt2:

```bash
# Install MoveIt2 (if not already installed)
sudo apt install ros-jazzy-moveit

# Create MoveIt config package (future work)
ros2 launch moveit_setup_assistant setup_assistant.launch.py
```

**Note**: MoveIt integration requires additional configuration and is recommended for complex pick-and-place operations.

---

## Monitoring Joint States

Monitor current joint positions:

```bash
# All joint states
ros2 topic echo /joint_states

# Specific joint
ros2 topic echo /joint_states --field name --field position

# Plot in real-time
ros2 run rqt_plot rqt_plot /joint_states/position[0]
```

## Controller Status

Check controller status:

```bash
# List all controllers
ros2 control list_controllers

# Show hardware components
ros2 control list_hardware_components

# View controller info
ros2 control list_controller_types
```

---

## Troubleshooting

### Joystick not detected

```bash
# Check device permissions
ls -l /dev/input/js0

# Add user to input group (if needed)
sudo usermod -a -G input $USER
# Log out and log back in
```

### Controllers not responding

```bash
# Check if controllers are active
ros2 control list_controllers

# Restart a controller
ros2 control switch_controllers \
  --deactivate base_main_frame_joint_controller
ros2 control switch_controllers \
  --activate base_main_frame_joint_controller
```

### Joystick button mapping different

Edit `scripts/joy_control.py` and adjust button/axis indices based on your controller. Test with:
```bash
ros2 topic echo /joy
```

---

## Safety Notes

⚠️ **Important**:
- Always have a way to **emergency stop** the robot
- Start with **low speed scales** (0.3-0.5) when testing
- Be aware of **joint limits** - controllers will clamp to safe ranges
- The **right container jaw** is a mimic joint and follows the left jaw
- In Gazebo, the mimic constraint doesn't work (dartsim limitation)

---

## Next Steps

- Create custom control sequences for pick-and-place operations
- Integrate vision systems for autonomous control
- Add force/torque sensing for contact detection
- Implement MoveIt2 for advanced motion planning
