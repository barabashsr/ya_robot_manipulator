# Quick Start Guide

## 1. Launch the Manipulator

```bash
cd ~/robo/ya_robot_manipulator/ros2_ws
source install/setup.bash
ros2 launch manipulator_description manipulator_control.launch.py
```

This launches:
- ✅ Gazebo simulation
- ✅ RViz visualization
- ✅ ROS2 Control with 8 active controllers
- ✅ Joint state broadcaster

---

## 2. Control Options

### Option A: GUI Control (Easiest)

**In a new terminal:**
```bash
source install/setup.bash
ros2 launch manipulator_description manipulator_gui_control.launch.py
```

Use RQT Publisher to send commands to individual controllers.

### Option B: PlayStation Controller (Most Fun)

**In a new terminal:**
```bash
source install/setup.bash
ros2 launch manipulator_description manipulator_joy_control.launch.py
```

**Controls:**
- Left Stick: Move base railway
- Right Stick: Move selector up/down and gripper left/right
- D-Pad: Control picker
- L1/R1: Picker slider
- Triangle/Cross: Picker jaw

### Option C: Command Line (Quick Test)

```bash
# Move railway to 2 meters
ros2 topic pub /base_main_frame_joint_controller/commands \
  std_msgs/msg/Float64MultiArray "data: [2.0]" --once

# Raise selector to 0.5 meters
ros2 topic pub /main_frame_selector_frame_joint_controller/commands \
  std_msgs/msg/Float64MultiArray "data: [0.5]" --once
```

---

## 3. Monitor Status

```bash
# View all joint positions
ros2 topic echo /joint_states

# Check controller status
ros2 control list_controllers

# Expected output:
# joint_state_broadcaster[joint_state_broadcaster/JointStateBroadcaster] active
# base_main_frame_joint_controller[forward_command_controller/ForwardCommandController] active
# ... (8 controllers total)
```

---

## 4. Troubleshooting

**Controllers not active:**
```bash
ros2 control list_controllers
```

**Joystick not working:**
```bash
ls /dev/input/js*  # Should show js0 or js1
ros2 topic echo /joy  # Test joystick input
```

**Meshes not visible:**
- Check RViz - meshes should be visible there
- Gazebo mesh loading may show warnings (non-critical)

---

## Full Documentation

See `docs/control_methods.md` for detailed control methods and `docs/manipulator_description.md` for complete package documentation.
