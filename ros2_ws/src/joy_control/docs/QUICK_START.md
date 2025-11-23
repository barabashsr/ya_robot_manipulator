# Joy Control - Quick Start

## 1. Launch the Manipulator

**Terminal 1:**
```bash
cd ~/robo/ya_robot_manipulator/ros2_ws
source install/setup.bash
ros2 launch manipulator_description manipulator_control.launch.py
```

## 2. Launch Joystick Control

**Terminal 2:**
```bash
source install/setup.bash
ros2 launch joy_control joy_control.launch.py
```

## 3. Control the Robot

1. **Hold L1 button** to enable control (safety feature)
2. **Move sticks** to control joints:
   - Left Stick: Railway + Picker slider
   - Right Stick: Selector + Gripper
   - D-Pad: Picker vertical/horizontal
   - L2/R2: Container jaws
3. **Hold R1** for turbo mode (2x speed)
4. **Press Triangle/Cross** to extend/retract picker jaw

## Configuration Location

Edit: `src/joy_control/config/manipulator_joy_config.yaml`

After editing, rebuild:
```bash
colcon build --packages-select joy_control
source install/setup.bash
```

## Customization

See `README.md` for full configuration options and axis/button mapping details.
