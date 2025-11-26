# Testing with RQt and CLI - Manipulator Control

This guide explains how to use RQt tools and command-line tools for manual testing and debugging of the manipulator control system.

## Prerequisites

- Simulation running: `ros2 launch manipulator_control manipulator_simulation.launch.py`
- RQt installed: `sudo apt install ros-jazzy-rqt ros-jazzy-rqt-topic ros-jazzy-rqt-console`

## Quick Start

```bash
# Terminal 1: Launch simulation
cd ~/robo/ya_robot_manipulator/ros2_ws
source install/setup.bash
ros2 launch manipulator_control manipulator_simulation.launch.py

# Terminal 2: Launch RQt for monitoring
source /opt/ros/jazzy/setup.bash
source ~/robo/ya_robot_manipulator/ros2_ws/install/setup.bash
rqt
```

## Sending MoveJoint Goals

**Note:** RQt's `rqt_action` plugin is only a message type browser - it cannot send goals. Use the command line instead.

### Command Line (Recommended)

```bash
# Move base rail to position 1.5m
ros2 action send_goal /move_joint manipulator_control/action/MoveJoint \
  "{joint_name: 'base_main_frame_joint', target_position: 1.5, max_velocity: 0.5}" --feedback

# Move Z-axis to 0.8m
ros2 action send_goal /move_joint manipulator_control/action/MoveJoint \
  "{joint_name: 'main_frame_selector_frame_joint', target_position: 0.8, max_velocity: 0.5}" --feedback

# Move picker jaw
ros2 action send_goal /move_joint manipulator_control/action/MoveJoint \
  "{joint_name: 'picker_base_picker_jaw_joint', target_position: 0.15, max_velocity: 0.3}" --feedback
```

Available joints and their safe ranges:

| Joint Name | Range (soft limits) | Description |
|------------|---------------------|-------------|
| base_main_frame_joint | 0.1 - 3.9 m | X-axis railway |
| main_frame_selector_frame_joint | 0.05 - 1.45 m | Z-axis vertical |
| selector_frame_gripper_joint | -0.39 - 0.39 m | Y-axis gripper |
| selector_frame_picker_frame_joint | 0.005 - 0.29 m | Z-axis picker |
| picker_frame_picker_rail_joint | -0.29 - 0.29 m | Y-axis picker rail |
| picker_rail_picker_base_joint | 0.005 - 0.24 m | X-axis picker extension |
| picker_base_picker_jaw_joint | 0.005 - 0.19 m | Picker jaw open/close |
| selector_left_container_jaw_joint | -0.19 - 0.19 m | Left container jaw |
| selector_right_container_jaw_joint | -0.19 - 0.19 m | Right container jaw |

### Understanding Output

The `--feedback` flag shows real-time progress:
- `current_position`: Real-time joint position
- `progress_percent`: Completion percentage (0-100)

Result shows:
- `success`: True if target reached within 0.01m tolerance
- `final_position`: Actual position achieved
- `execution_time`: Time taken in seconds
- `message`: Human-readable status

## Sending MoveJointGroup Goals

For coordinated multi-joint movements, use the command line.

### Navigation Group (2 joints)

```bash
ros2 action send_goal /move_joint_group manipulator_control/action/MoveJointGroup \
  "{joint_names: ['base_main_frame_joint', 'main_frame_selector_frame_joint'], target_positions: [2.0, 0.8], max_velocity: 0.5}" --feedback
```

### Picker Group (4 joints)

```bash
ros2 action send_goal /move_joint_group manipulator_control/action/MoveJointGroup \
  "{joint_names: ['selector_frame_picker_frame_joint', 'picker_frame_picker_rail_joint', 'picker_rail_picker_base_joint', 'picker_base_picker_jaw_joint'], target_positions: [0.15, 0.0, 0.12, 0.1], max_velocity: 0.3}" --feedback
```

### Container Mimic Mode

The container group uses mimic mode for symmetric jaw motion:
- Input: Single `opening` value (e.g., 0.2)
- Left jaw moves to: `-opening/2` (e.g., -0.1)
- Right jaw moves to: `+opening/2` (e.g., +0.1)

```bash
ros2 action send_goal /move_joint_group manipulator_control/action/MoveJointGroup \
  "{joint_names: ['container'], target_positions: [0.2], max_velocity: 0.2}" --feedback
```

## Monitoring Limit Switches

View real-time limit switch states.

### Setup

1. Menu: **Plugins -> Topics -> Topic Monitor**
2. Click the refresh button to see available topics
3. Expand: `/manipulator/end_switches/`
4. Select switches to monitor

### All 18 Limit Switch Topics

```
/manipulator/end_switches/base_main_frame_min
/manipulator/end_switches/base_main_frame_max
/manipulator/end_switches/selector_frame_min
/manipulator/end_switches/selector_frame_max
/manipulator/end_switches/gripper_left
/manipulator/end_switches/gripper_right
/manipulator/end_switches/picker_frame_min
/manipulator/end_switches/picker_frame_max
/manipulator/end_switches/picker_rail_min
/manipulator/end_switches/picker_rail_max
/manipulator/end_switches/picker_retracted
/manipulator/end_switches/picker_extended
/manipulator/end_switches/picker_jaw_opened
/manipulator/end_switches/picker_jaw_closed
/manipulator/end_switches/container_left_min
/manipulator/end_switches/container_left_max
/manipulator/end_switches/container_right_min
/manipulator/end_switches/container_right_max
```

### Testing Switch Triggers

1. Monitor a switch (e.g., `picker_jaw_closed`)
2. Use Action Client to move the joint to trigger position
3. Watch the Bool value change from `False` to `True`

Example trigger positions:
- `base_main_frame_min`: position 0.0 (tolerance 0.01)
- `picker_jaw_closed`: position 0.01 (tolerance 0.005)
- `picker_jaw_opened`: position 0.19 (tolerance 0.005)

## Viewing State Markers

State markers visualize electromagnet and target states.

### Using RViz (Recommended for Markers)

```bash
# Terminal 3: Launch RViz
rviz2
```

1. Click **Add** button
2. Select **By topic** -> `/visualization_marker_array` -> **MarkerArray**
3. Set Fixed Frame to: `base_link`

### Triggering Markers

Use RQt Topic Publisher or command line:

```bash
# Engage electromagnet (shows magnet marker)
ros2 topic pub --once /manipulator/electromagnet/engaged std_msgs/msg/Bool "data: true"

# Set target address (shows green cube)
ros2 topic pub --once /manipulator/target_address std_msgs/msg/String "data: 'A,1,1'"

# Clear target (removes marker)
ros2 topic pub --once /manipulator/target_address std_msgs/msg/String "data: ''"
```

## Monitoring Logs

View node logs and warnings.

### Setup

1. Menu: **Plugins -> Logging -> Console**
2. Logs from all nodes appear here
3. Use filters to focus on specific severity levels

### Useful Log Sources

- `move_joint_server`: Goal acceptance, progress, timeouts
- `move_joint_group_server`: Group coordination, mimic calculations
- `virtual_limit_switch_node`: Switch state changes
- `state_marker_publisher`: Marker updates

## Saving and Loading Perspectives

Save your RQt layout for future sessions.

### Saving

1. Arrange plugins in your preferred layout
2. Menu: **File -> Save Perspective As...**
3. Enter name: `manipulator_dev`
4. Save location: `~/.config/ros.org/rqt_gui/` (default)

### Loading

1. Menu: **File -> Open Perspective**
2. Select `manipulator_dev`

### Installing Project Perspective

The project includes a pre-configured perspective:

```bash
# Copy to config directory
mkdir -p ~/.config/ros.org/rqt_gui/
cp ~/robo/ya_robot_manipulator/ros2_ws/src/manipulator_control/config/manipulator_dev.perspective \
   ~/.config/ros.org/rqt_gui/

# Or load directly
rqt --perspective-file ~/robo/ya_robot_manipulator/ros2_ws/install/manipulator_control/share/manipulator_control/config/manipulator_dev.perspective
```

## Recommended RQt Layout

```
+---------------------------+---------------------------+
|     Action Client         |    Topic Monitor          |
|   (MoveJoint)             |  /manipulator/end_switches|
|                           |                           |
+---------------------------+---------------------------+
|     Action Client         |    Console                |
|   (MoveJointGroup)        |  (All logs)               |
|                           |                           |
+---------------------------+---------------------------+
```

## Troubleshooting

### Action Server Not Available

```bash
# Check if servers are running
ros2 action list | grep move
# Expected:
# /move_joint
# /move_joint_group
```

### No Switch Topics

```bash
# Check limit switch node
ros2 node list | grep limit
# Expected: /virtual_limit_switch_node

# Verify topics
ros2 topic list | grep end_switches | head -5
```

### Goal Rejected

Check the RQt Console for rejection reasons:
- "unknown joint" - Check joint name spelling
- "position outside limits" - Use soft limit ranges from table above

### Timeout Errors

- Default timeout is 30 seconds
- Use lower `max_velocity` for more time
- Check if Gazebo physics is running (not paused)

## Running Automated Tests

After manual verification, run the automated test suite:

```bash
cd ~/robo/ya_robot_manipulator/ros2_ws
source install/setup.bash

# Ensure simulation is running in another terminal

# Run all Epic 2 tests
pytest src/manipulator_control/test/test_epic2_joint_control.py -v

# Run specific test class
pytest src/manipulator_control/test/test_epic2_joint_control.py::TestMoveJoint -v

# Run with timeout protection
pytest src/manipulator_control/test/test_epic2_joint_control.py -v --timeout=300
```
