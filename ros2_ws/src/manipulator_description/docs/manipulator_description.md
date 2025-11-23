# Manipulator Description Package

ROS2 package containing URDF/xacro robot description and control configuration for the manipulator system.

## Package Structure

```
manipulator_description/
├── config/
│   ├── manipulator_params.yaml      # All robot parameters (masses, inertias, limits, etc.)
│   └── manipulator_controllers.yaml # ROS2 control configuration
├── launch/
│   ├── display.launch.py            # RViz visualization only
│   └── manipulator_control.launch.py # Full simulation with Gazebo + RViz
├── meshes/
│   └── manipulator/                 # STL mesh files (11 files)
├── rviz/
│   └── manipulator_control.rviz     # RViz configuration
└── urdf/
    ├── manipulator/
    │   ├── manipulator_base.urdf.xacro      # Base assembly (railway system)
    │   ├── manipulator_selector.urdf.xacro  # Selector assembly (gripper + jaws)
    │   ├── manipulator_picker.urdf.xacro    # Picker assembly
    │   ├── manipulator.urdf.xacro           # Main manipulator integration
    │   └── ros2_control.xacro               # Hardware interface definitions
    ├── robot.urdf.xacro             # Top-level robot (manipulator + storage)
    ├── storage_system.urdf.xacro    # Storage cabinets and frames
    └── materials.xacro              # Material definitions
```

## Robot Structure

### Assembly Breakdown

**Base Assembly** (railway system):
- `base_link` - Railway base (world-fixed)
- `main_frame` - Railway carriage (moves along X-axis: 0-4m)
- Joint: `base_main_frame_joint` (prismatic, X-axis)

**Selector Assembly** (container gripper):
- `selector_frame` - Vertical frame (Z-axis: 0-1.5m)
- `left_container_jaw` / `right_container_jaw` - Mirrored jaws (Y-axis: ±0.2m)
- `gripper` - Magnet holder (Y-axis: ±0.4m)
- `left_gripper_magnet` / `right_gripper_magnet` - Reference frames only
- Joints: 3 controlled + 1 mimic

**Picker Assembly** (item picker):
- `picker_frame` - Vertical mount (Z-axis: 0-0.3m)
- `picker_rail` - Y-axis rail (±0.3m)
- `picker_base` - X-axis slider (0-0.12m)
- `picker_jaw` - Extension jaw (0-0.2m)
- Joints: 4 controlled

**Total**: 8 controlled joints + 1 mimic joint

## ROS2 Control Configuration

### Controllers

All joints use `forward_command_controller/ForwardCommandController` with position interface:

1. `base_main_frame_joint_controller` - X-axis railway (0-4m)
2. `main_frame_selector_frame_joint_controller` - Z-axis vertical (0-1.5m)
3. `selector_left_container_jaw_joint_controller` - Left jaw (±0.2m)
4. `selector_frame_gripper_joint_controller` - Gripper Y-axis (±0.4m)
5. `selector_frame_picker_frame_joint_controller` - Picker Z-axis (0-0.3m)
6. `picker_frame_picker_rail_joint_controller` - Picker Y-axis (±0.3m)
7. `picker_rail_picker_base_joint_controller` - Picker X-axis (0-0.12m)
8. `picker_base_picker_jaw_joint_controller` - Picker extension (0-0.2m)

**Note**: `selector_right_container_jaw_joint` mimics the left jaw (multiplier=1.0, offset=0.0)

### Hardware Interfaces

- **Simulation**: `gz_ros2_control/GazeboSimSystem` (Gazebo plugin)
- **Mock**: `mock_components/GenericSystem` (for testing without Gazebo)

Each joint exposes:
- **Command interfaces**: position, velocity
- **State interfaces**: position, velocity

## Usage

### Launch Gazebo Simulation

```bash
# Source workspace
source install/setup.bash

# Launch with Gazebo + RViz + controllers
ros2 launch manipulator_description manipulator_control.launch.py

# Launch with mock hardware (no Gazebo)
ros2 launch manipulator_description manipulator_control.launch.py sim:=false
```

### Control Joints

Send position commands (in meters or radians):

```bash
# Move railway to 2.0m along X-axis
ros2 topic pub /base_main_frame_joint_controller/commands \
  std_msgs/msg/Float64MultiArray "data: [2.0]" --once

# Raise selector to 0.5m
ros2 topic pub /main_frame_selector_frame_joint_controller/commands \
  std_msgs/msg/Float64MultiArray "data: [0.5]" --once

# Open container jaws to 0.15m
ros2 topic pub /selector_left_container_jaw_joint_controller/commands \
  std_msgs/msg/Float64MultiArray "data: [0.15]" --once
```

### Monitor Joint States

```bash
# All joint states
ros2 topic echo /joint_states

# List available topics
ros2 topic list | grep -E "(commands|states)"

# Check controller status
ros2 control list_controllers
```

## Technical Details

### Clock Synchronization

The launch file includes a clock bridge (`ros_gz_bridge`) to synchronize Gazebo simulation time with ROS2:
- All nodes use `use_sim_time: true` when `sim:=true`
- Clock published on `/clock` topic from Gazebo

### Mesh Visualization

- **Mesh Configuration**: All mesh filenames are configurable via `config/manipulator_params.yaml`
- **Path Construction**: Xacro files construct the full path as `file://$(find manipulator_description)/meshes/manipulator/${mesh_filename}`
- **YAML Format**: Store only the filename (e.g., `mesh: "base_link.STL"`)
- **Full Path**: Automatically resolved to `file://$(find manipulator_description)/meshes/manipulator/base_link.STL`
- Meshes located in: `meshes/manipulator/*.STL`

**Example YAML configuration**:
```yaml
base_assembly:
  base_link:
    mesh: "base_link.STL"  # Just the filename
    color: [0.79216, 0.81961, 0.93333, 1.0]
    # ... other parameters
```

**How it works**:
- The xacro macro reads the mesh filename from YAML
- Constructs the full URI: `file://$(find manipulator_description)/meshes/manipulator/${mesh_filename}`
- This format works for both RViz and Gazebo

### Mimic Joint Limitation

The right container jaw uses a mimic constraint in URDF:
```xml
<mimic joint="selector_left_container_jaw_joint" multiplier="1.0" offset="0.0"/>
```

**Known issue**: Gazebo's dartsim physics engine doesn't support mimic constraints. The right jaw won't follow the left jaw in simulation but will work correctly in real hardware (if hardware supports it).

**Workaround**: For Gazebo simulation, you can:
1. Accept that only the left jaw moves
2. Create a separate controller for the right jaw (removes mimic)
3. Use a different physics engine (if available)

### Collision Meshes

**Note**: Dartsim doesn't support mesh collisions from SDF. The collision geometries are currently ignored. For physics interactions, consider:
1. Adding primitive collision shapes (boxes, cylinders)
2. Converting STL meshes to simpler collision approximations
3. Using the meshes only for visualization

## Controller Manager

The controller manager runs at 100 Hz (configurable in `manipulator_controllers.yaml`).

### List Active Controllers

```bash
ros2 control list_controllers
```

Expected output:
```
joint_state_broadcaster[joint_state_broadcaster/JointStateBroadcaster] active
base_main_frame_joint_controller[forward_command_controller/ForwardCommandController] active
main_frame_selector_frame_joint_controller[forward_command_controller/ForwardCommandController] active
selector_left_container_jaw_joint_controller[forward_command_controller/ForwardCommandController] active
selector_frame_gripper_joint_controller[forward_command_controller/ForwardCommandController] active
selector_frame_picker_frame_joint_controller[forward_command_controller/ForwardCommandController] active
picker_frame_picker_rail_joint_controller[forward_command_controller/ForwardCommandController] active
picker_rail_picker_base_joint_controller[forward_command_controller/ForwardCommandController] active
picker_base_picker_jaw_joint_controller[forward_command_controller/ForwardCommandController] active
```

## Troubleshooting

### Controllers fail to load

Check controller configuration:
```bash
# View loaded controllers
ros2 control list_controllers

# Check controller manager logs
ros2 topic echo /rosout | grep controller_manager
```

### Meshes not visible in Gazebo

- Verify meshes exist: `ls src/manipulator_description/meshes/manipulator/`
- Check `GZ_SIM_RESOURCE_PATH` environment variable
- View Gazebo logs for mesh loading errors

### use_sim_time warnings

If you see warnings about `use_sim_time`, ensure:
1. Clock bridge is running: `ros2 topic list | grep clock`
2. All nodes have `use_sim_time: true` parameter
3. Gazebo is publishing clock: `ros2 topic echo /clock`

## References

- [ROS2 Control Documentation](https://control.ros.org/)
- [Gazebo Garden Documentation](https://gazebosim.org/)
- [gz_ros2_control](https://github.com/ros-controls/gz_ros2_control)
