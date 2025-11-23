# Joystick Control Fixes - Summary

## Issues Fixed

### 1. ✅ D-Pad Not Working Properly
**Problem**: D-Pad discrete values (1.0, 0, -1.0) were being scaled like analog sticks
**Fix**: Updated `apply_deadzone()` to detect D-Pad axes (6, 7) and skip scaling
**File**: `src/joy_control/scripts/joy_controller_node.py:177-191, 225`

### 2. ✅ Container Jaw Mimic Not Working in Gazebo
**Problem**: dartsim doesn't support mimic constraints
**Solution**: Created separate controller for right jaw, both controlled by same axis with opposite scale
**Files Changed**:
- `src/manipulator_description/config/manipulator_controllers.yaml` - Added right jaw controller
- `src/manipulator_description/launch/manipulator_control.launch.py` - Added right jaw to spawn list
- `src/joy_control/config/manipulator_joy_config.yaml` - Added right jaw mapping with negative scale
- `src/joy_control/scripts/joy_controller_node.py` - Added to known mappings
- `src/joy_control/scripts/debug_joy_control.py` - Monitor both jaws

### 3. ✅ Debug Tools Enhanced
**Added**:
- Right Stick Y monitoring for selector vertical debugging
- Triangle/Cross button detection for picker jaw debugging
- Separate monitoring for left/right container jaws

## Testing Instructions

### Step 1: Source Workspace
```bash
cd ~/robo/ya_robot_manipulator/ros2_ws
source install/setup.bash
```

### Step 2: Launch Main System
```bash
ros2 launch manipulator_description manipulator_control.launch.py
```

Wait for Gazebo and all controllers to load.

### Step 3: Verify Controllers
In a new terminal:
```bash
source install/setup.bash
ros2 control list_controllers
```

You should now see **9 controllers** (including the new right jaw):
- base_main_frame_joint_controller
- main_frame_selector_frame_joint_controller
- selector_left_container_jaw_joint_controller
- **selector_right_container_jaw_joint_controller** ← NEW
- selector_frame_gripper_joint_controller
- selector_frame_picker_frame_joint_controller
- picker_frame_picker_rail_joint_controller
- picker_rail_picker_base_joint_controller
- picker_base_picker_jaw_joint_controller

### Step 4: Test Raw Joystick
```bash
ros2 run joy_control test_joy_axes.py
```

Verify your controller buttons/axes are detected.

### Step 5: Run Debug Monitor
```bash
ros2 run joy_control debug_joy_control.py
```

### Step 6: Launch Joy Control
In another terminal:
```bash
source install/setup.bash
ros2 launch joy_control joy_control.launch.py
```

Should show:
```
Loaded 9 axis mappings  ← Was 8, now 9
Hold button 4 (L1) to enable control
```

### Step 7: Test Each Control (HOLD L1!)

**IMPORTANT**: You must hold L1 (button 4) for any control to work!

Test each control and check debug monitor output:

1. **Base Railway** (Left Stick Y):
   - Move left stick up/down while holding L1
   - Should see `base_main_frame -> X.XXX` in debug output

2. **Selector Vertical** (Right Stick Y):
   - Move right stick up/down while holding L1
   - Should see `selector_vertical -> X.XXX` in debug output

3. **Gripper Y-axis** (Right Stick X):
   - Move right stick left/right while holding L1
   - Should see `gripper -> X.XXX` in debug output

4. **Container Jaws** (L2 Trigger):
   - Press L2 trigger while holding L1
   - Should see BOTH:
     - `container_jaw_L -> X.XXX`
     - `container_jaw_R -> X.XXX` (with opposite sign)
   - Both jaws should move in Gazebo now!

5. **Picker Vertical** (D-Pad Y):
   - Press D-Pad up/down while holding L1
   - Should see `picker_vertical -> X.XXX` in debug output
   - Both directions should work now!

6. **Picker Horizontal** (D-Pad X):
   - Press D-Pad left/right while holding L1
   - Should see `picker_horizontal -> X.XXX` in debug output
   - Both directions should work now!

7. **Picker Slider** (Left Stick X):
   - Move left stick left/right while holding L1
   - Should see `picker_slider -> X.XXX` in debug output

8. **Picker Jaw Buttons** (Triangle/Cross):
   - Hold Triangle while holding L1
   - Should see `picker_jaw -> X.XXX` increasing toward 0.2
   - Hold Cross while holding L1
   - Should see `picker_jaw -> X.XXX` decreasing toward 0.0

## Remaining Issues to Check

### Issue A: Selector Vertical Still Not Working?
If Right Stick Y still doesn't move the selector:

1. Check debug output - is "Right Stick Y (axis 4)" detected?
2. Check if commands are published - is "selector_vertical -> X.XXX" shown?
3. Check joint limits - might be at limit already
4. Try moving in both directions

### Issue B: Picker Jaw Buttons Still Not Working?
If Triangle/Cross don't move picker jaw:

1. Check debug output - are button presses detected?
2. Check if commands are published - is "picker_jaw -> X.XXX" shown?
3. Verify you're holding L1 while pressing Triangle/Cross
4. Check that axis_index is -1 in YAML (buttons use action system, not axis)

### Issue C: Container Jaws Return to Zero
If container jaws close when L2 released:

This is expected behavior! The L2 trigger is an axis that returns to neutral when released.
The velocity-based control means when trigger is released (value = 0), velocity = 0, but position is maintained.

If position is NOT maintained, there might be an issue with position tracking for trigger axes.

## Configuration Reference

All joystick mappings in: `src/joy_control/config/manipulator_joy_config.yaml`

### Current Axis Assignments:
- Axis 0 (Left Stick X) → Picker slider
- Axis 1 (Left Stick Y) → Base railway
- Axis 2 (L2 Trigger) → Container jaws (both)
- Axis 3 (Right Stick X) → Gripper Y-axis
- Axis 4 (Right Stick Y) → Selector vertical
- Axis 6 (D-Pad X) → Picker horizontal
- Axis 7 (D-Pad Y) → Picker vertical

### Button Assignments:
- Button 0 (Cross) → Picker jaw retract (hold)
- Button 2 (Triangle) → Picker jaw extend (hold)
- Button 4 (L1) → **ENABLE CONTROL**
- Button 5 (R1) → Turbo mode (2x speed)

## Files Modified

1. `src/manipulator_description/config/manipulator_controllers.yaml`
   - Added selector_right_container_jaw_joint_controller

2. `src/manipulator_description/launch/manipulator_control.launch.py`
   - Added right jaw controller to spawn list

3. `src/joy_control/config/manipulator_joy_config.yaml`
   - Added right jaw axis mapping (same axis, opposite scale)

4. `src/joy_control/scripts/joy_controller_node.py`
   - Fixed apply_deadzone() to handle D-Pad discrete values
   - Added axis_index parameter to apply_deadzone()
   - Updated joy_callback to pass axis_index
   - Added selector_right_container_jaw to known mappings

5. `src/joy_control/scripts/debug_joy_control.py`
   - Added Right Stick Y monitoring
   - Added button press detection
   - Added separate monitoring for both container jaws

## Next Steps

After testing, if issues persist:

1. **Check joint limits**: Joints might be at their limits
   ```bash
   ros2 topic echo /joint_states
   ```

2. **Check controller commands**: See if commands are reaching controllers
   ```bash
   ros2 topic echo /main_frame_selector_frame_joint_controller/commands
   ```

3. **Increase velocity scales**: If movement is too slow, increase velocity_scale in YAML

4. **Adjust axis scales**: If direction is wrong, change axis_scale sign in YAML
