# manipulator_control

Control package for the rail-mounted robotic manipulator with autonomous warehouse operations.

## Overview

This package provides action servers, services, and custom interfaces for controlling a 9-DOF rail-mounted manipulator in a Level 3 autonomous warehouse environment. The manipulator performs box extraction, item picking, and placement operations using ROS2 control framework and Gazebo simulation.

## Package Structure

```
manipulator_control/
├── action/          # Action interface definitions (.action files)
├── srv/             # Service interface definitions (.srv files)
├── msg/             # Custom message definitions (.msg files)
├── src/             # Python node implementations
├── launch/          # Launch files for nodes and simulation
├── config/          # YAML configuration files
├── test/            # Unit and integration tests
├── CMakeLists.txt   # Build configuration
├── package.xml      # Package metadata and dependencies
└── README.md        # This file
```

## Dependencies

**ROS2 Packages:**
- rclcpp, rclpy (C++/Python ROS2 client libraries)
- std_msgs, geometry_msgs, sensor_msgs (Standard ROS2 messages)
- action_msgs, std_srvs (Action and service support)
- tf2, tf2_ros, tf2_geometry_msgs (Transform library)
- controller_manager, controller_interface, ros2_control, ros2_controllers (Control framework)
- ros_gz_bridge, ros_gz_interfaces, ros_gz_sim (Gazebo integration)

**System Requirements:**
- ROS2 Jazzy
- Gazebo Harmonic
- Ubuntu 24.04 (recommended)

## Build Instructions

1. **Source ROS2 environment:**
   ```bash
   source /opt/ros/jazzy/setup.bash
   ```

2. **Navigate to workspace:**
   ```bash
   cd ros2_ws
   ```

3. **Build the package:**
   ```bash
   colcon build --packages-select manipulator_control
   ```

4. **Source the workspace:**
   ```bash
   source install/setup.bash
   ```

## Interface Definitions

### Actions (Defined in Story 1.2)

**Low-Level Actions:**
- `MoveJoint.action` - Move a single joint to target position
- `MoveJointGroup.action` - Move multiple joints simultaneously

**Mid-Level Actions:**
- `NavigateToAddress.action` - Navigate to warehouse storage address
- `ExtractBox.action` - Extract box from storage cabinet
- `ReturnBox.action` - Return box to original address
- `PutBox.action` - Place box at specified address
- `MoveBoxToLoad.action` - Move box to loading position
- `ManipulateContainer.action` - Open/close jaw for containers
- `GetContainer.action` - Retrieve container from storage
- `PlaceContainer.action` - Place container back to storage
- `PickItem.action` - Pick item from department within box

**High-Level Actions:**
- `PickItemFromStorage.action` - Complete workflow to pick item from storage

### Services (Defined in Story 1.3)

- `GetAddressCoordinates.srv` - Resolve warehouse address to 3D coordinates
- `ToggleElectromagnet.srv` - Control electromagnet for box attachment
- `SpawnBox.srv` - Dynamically spawn box in simulation
- `DespawnBox.srv` - Remove box from simulation
- `ValidateAddress.srv` - Validate warehouse address and constraints

### Messages (Defined in Story 1.3)

- `Address.msg` - Warehouse address (side, cabinet, row, column)
- `JointCommand.msg` - Joint position command with velocity limit
- `LimitSwitchState.msg` - Virtual limit switch status

## Launch Files

### Unified Simulation Launch (Recommended)

Launch the complete simulation environment with a single command:

```bash
source install/setup.bash
ros2 launch manipulator_control manipulator_simulation.launch.py
```

This unified launch file starts:
- Gazebo simulation with robot model
- All joint controllers (ros2_control)
- RViz visualization
- Virtual limit switches node (18 switches, publishing at 10 Hz)

**With joystick control enabled:**
```bash
ros2 launch manipulator_control manipulator_simulation.launch.py enable_joy:=true
```

### Individual Launch Files

**Virtual limit switches only** (requires Gazebo + controllers running):
```bash
ros2 launch manipulator_control virtual_limit_switches.launch.py
```

**Gazebo + controllers** (from manipulator_description package):
```bash
ros2 launch manipulator_description manipulator_control.launch.py
```

## Virtual Limit Switches

The package includes a virtual limit switch simulation system with 18 limit switches (2 per joint) monitoring joint positions.

**Switch topics** (std_msgs/Bool at 10 Hz):
```
/manipulator/end_switches/base_main_frame_min
/manipulator/end_switches/base_main_frame_max
/manipulator/end_switches/selector_frame_min
/manipulator/end_switches/selector_frame_max
/manipulator/end_switches/gripper_extended
/manipulator/end_switches/gripper_retracted
/manipulator/end_switches/picker_frame_min
/manipulator/end_switches/picker_frame_max
/manipulator/end_switches/picker_rail_min
/manipulator/end_switches/picker_rail_max
/manipulator/end_switches/picker_base_min
/manipulator/end_switches/picker_base_max
/manipulator/end_switches/picker_jaw_opened
/manipulator/end_switches/picker_jaw_closed
/manipulator/end_switches/container_left_min
/manipulator/end_switches/container_left_max
/manipulator/end_switches/container_right_min
/manipulator/end_switches/container_right_max
```

**Verify switch operation:**
```bash
# Check all 18 topics exist
ros2 topic list | grep end_switches

# Monitor a specific switch
ros2 topic echo /manipulator/end_switches/picker_jaw_closed

# Check publish rate
ros2 topic hz /manipulator/end_switches/picker_jaw_closed
```

**Configuration:**
Switch trigger positions and tolerances are defined in `config/limit_switches.yaml`

## Electromagnet Simulator (Story 4A.2)

The package includes a simulated electromagnet for box attachment/detachment using Gazebo's DetachableJoint plugin.

**Service:**
```
/manipulator/electromagnet/toggle (manipulator_control/srv/ToggleElectromagnet)
```

**State Topic:**
```
/manipulator/electromagnet/engaged (std_msgs/Bool) @ 10 Hz
```

**Usage:**
```bash
# Activate electromagnet (attaches to nearby box if within 5cm)
ros2 service call /manipulator/electromagnet/toggle manipulator_control/srv/ToggleElectromagnet "{activate: true}"

# Deactivate electromagnet (releases attached box)
ros2 service call /manipulator/electromagnet/toggle manipulator_control/srv/ToggleElectromagnet "{activate: false}"

# Monitor magnet state
ros2 topic echo /manipulator/electromagnet/engaged
```

**Behavior:**
- Activation requires a box within `proximity_distance` (default 5cm) of gripper magnet
- Publishes to DetachableJoint attach/detach topics for Gazebo physics
- Returns success=false if no box in proximity on activation

**Configuration:**
Electromagnet parameters are defined in `config/electromagnet.yaml`

## Box Spawn Manager (Story 4A.3)

The package provides dynamic box spawning with department child links in the TF tree for box extraction operations.

**Services:**
```
/manipulator/box_spawn/spawn (manipulator_control/srv/SpawnBox)
/manipulator/box_spawn/despawn (manipulator_control/srv/DespawnBox)
```

**SpawnBox Request:**
- `box_id` (string): Box identifier, format: `box_{side}_{cabinet}_{row}_{col}` (e.g., `box_l_1_2_3`)
- `side` (string): Cabinet side - `left` or `right`
- `cabinet_num` (uint8): Cabinet number (1-4)
- `row` (uint8): Row within cabinet (1-N)
- `column` (uint8): Column within cabinet (1-N)

**SpawnBox Response:**
- `success` (bool): True if box spawned successfully
- `box_id` (string): Confirmed box identifier
- `department_count` (uint8): Number of departments in spawned box
- `message` (string): Status message

**DespawnBox Request/Response:**
- Request: `box_id` (string)
- Response: `success` (bool), `message` (string)

**Usage:**
```bash
# Spawn a box at left cabinet 1, row 2, column 3
ros2 service call /manipulator/box_spawn/spawn manipulator_control/srv/SpawnBox \
  "{box_id: 'box_l_1_2_3', side: 'left', cabinet_num: 1, row: 2, column: 3}"

# Verify TF frames exist
ros2 run tf2_ros tf2_echo world box_l_1_2_3_base_link
ros2 run tf2_ros tf2_echo world box_l_1_2_3_dept_1_link

# Despawn the box
ros2 service call /manipulator/box_spawn/despawn manipulator_control/srv/DespawnBox \
  "{box_id: 'box_l_1_2_3'}"

# Verify no orphan processes
ps aux | grep robot_state_publisher
```

**Features:**
- Generates URDF with base_link and N department child links
- Launches robot_state_publisher subprocess for TF broadcasting
- Attaches box to gripper_magnet frame via static transform
- (Simulation) Spawns visual model in Gazebo with DetachableJoint plugin
- Department Y positions follow formula: `y = offset_y + (dept_num - 1) * step_y`

**TF Frame Structure:**
```
world
  └── left_gripper_magnet
        └── box_l_1_2_3_base_link (static transform)
              ├── box_l_1_2_3_dept_1_link
              ├── box_l_1_2_3_dept_2_link
              └── ... (N departments based on cabinet config)
```

**Configuration:**
Box spawner parameters are defined in `config/box_spawner.yaml`

## Usage Examples

**Using action interfaces (Python):**
```python
from manipulator_control.action import MoveJoint, NavigateToAddress
# Action client implementation in Epic 2+
```

**Using service interfaces (Python):**
```python
from manipulator_control.srv import GetAddressCoordinates
# Service client implementation in Epic 3+
```

**Using custom messages (Python):**
```python
from manipulator_control.msg import Address, JointCommand, LimitSwitchState
# Message usage in Epic 2+
```

## Testing

Run package tests:
```bash
colcon test --packages-select manipulator_control
```

View test results:
```bash
colcon test-result --verbose
```

## Related Packages

- `manipulator_description` - URDF/Xacro robot description and ros2_control configuration

## Development Status

- **Epic 1** (Current): Package structure and interface definitions
- **Epic 2+**: Action server implementations and control logic

## License

Apache-2.0

## Maintainer

robo (barabashsr@gmail.com)
