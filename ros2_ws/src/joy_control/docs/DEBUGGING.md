# Joy Control Debugging Guide

## Quick Start

After building the package, source the workspace:
```bash
source install/setup.bash
```

## Test 1: Check Raw Joystick Input

Run the axis tester to verify your controller is connected and which axes/buttons map to which controls:

```bash
ros2 run joy_control test_joy_axes.py
```

Move all sticks, press all buttons, and verify the indices match the configuration.

## Test 2: Monitor Published Commands

Run the debug monitor to see what commands are being published to the controllers:

```bash
ros2 run joy_control debug_joy_control.py
```

This will show:
- When you press L1 (enable button)
- When you move D-Pad (picker controls)
- When you move Right Stick Y (selector vertical)
- When you press Triangle/Cross (picker jaw buttons)
- All commands being published to joint controllers

**Important**: Hold L1 button while testing - control is only enabled when L1 is held!

## Test 3: Run the Full System

Launch the main controller:

```bash
ros2 launch joy_control joy_control.launch.py
```

Watch for log messages showing:
- "Loaded X axis mappings"
- "Loaded X button actions"
- Status updates every 2 seconds

## Current Known Issues

### Issue 1: Only One D-Pad Direction Works
**Status**: Fixed in latest build
**Fix**: Updated `apply_deadzone()` to handle discrete D-Pad values and pass `axis_index` parameter
**Test**: Move D-Pad up/down and left/right while holding L1

### Issue 2: Selector Vertical (Right Stick Y) Not Working
**Mapping**: Axis 4 → `/main_frame_selector_frame_joint_controller/commands`
**Debug Steps**:
1. Run debug monitor
2. Move Right Stick Y while holding L1
3. Check if "Right Stick Y (axis 4)" appears in debug output
4. Check if "selector_vertical" commands are published

### Issue 3: Picker Vertical (D-Pad Y) Not Working
**Mapping**: Axis 7 → `/selector_frame_picker_frame_joint_controller/commands`
**Debug Steps**:
1. Run debug monitor
2. Press D-Pad up/down while holding L1
3. Check if "D-Pad: Y=" appears in debug output
4. Check if "picker_vertical" commands are published

### Issue 4: Picker Jaw Buttons Not Working
**Mapping**:
- Button 2 (Triangle) → Extend to 0.2m (hold action)
- Button 0 (Cross) → Retract to 0.0m (hold action)
**Debug Steps**:
1. Run debug monitor
2. Press Triangle/Cross while holding L1
3. Check if button press messages appear
4. Check if "picker_jaw" commands are published

### Issue 5: Container Jaws Behave Differently in RViz vs Gazebo
**Expected Behavior**:
- Gazebo (dartsim): Only left jaw moves (mimic joints not supported)
- RViz: Both jaws should move and maintain position

**Observed**: Both jaws move in RViz but return to 0 when button released

**Possible Causes**:
1. L2 trigger axis might return to neutral (need to investigate axis behavior)
2. Position tracking might not be working correctly for trigger axes

## Debugging Tips

### Check Controller Manager
```bash
ros2 control list_controllers
```

Should show all 8 joint controllers as "active" and "claimed".

### Check Joint States
```bash
ros2 topic echo /joint_states
```

Move controls and verify joint positions update.

### Check Specific Controller Commands
```bash
ros2 topic echo /main_frame_selector_frame_joint_controller/commands
```

Replace with any controller topic. Should show [position] array when you move the mapped control while holding L1.

### Verify joy Topic
```bash
ros2 topic echo /joy
```

Verify raw joystick data is being published.

## Configuration File

All mappings are in:
```
src/joy_control/config/manipulator_joy_config.yaml
```

Key parameters:
- `enable_button: 4` - L1 must be held for control
- `enable_turbo_button: 5` - R1 for 2x speed
- `deadzone: 0.1` - Applies to analog sticks only
- `update_rate: 20.0` - Hz for publishing commands

## PlayStation Controller Reference

**Axes**:
- 0 = Left Stick X
- 1 = Left Stick Y
- 2 = L2 Trigger
- 3 = Right Stick X
- 4 = Right Stick Y
- 5 = R2 Trigger
- 6 = D-Pad X (discrete: -1, 0, 1)
- 7 = D-Pad Y (discrete: -1, 0, 1)

**Buttons**:
- 0 = Cross (X)
- 1 = Circle (O)
- 2 = Triangle
- 3 = Square
- 4 = L1 (ENABLE)
- 5 = R1 (TURBO)
- 8 = Share
- 9 = Options

## Next Steps

If an axis/button doesn't work:

1. **Verify Hardware**: Run `test_joy_axes.py` and confirm the input is detected
2. **Verify Mapping**: Check YAML config has correct axis_index and controller_topic
3. **Verify Enable**: Make sure L1 is held down during testing
4. **Verify Publishing**: Run `debug_joy_control.py` to see if commands are published
5. **Verify Reception**: Echo controller topic to see if commands reach ros2_control
6. **Check Limits**: Position might be hitting joint limits (check YAML limits array)
