# ROS2 Control Architecture
## ya_robot_manipulator Level 3 Control System

**Document Version:** 2.0
**Date:** 2025-11-24
**Project:** ya_robot_manipulator
**Scope:** Level 3 - ROS2 Action-Based Control System

**Revision History:**
- v1.0: Initial draft
- v2.0: Corrected warehouse addressing, controller architecture, picker state machine, simulation requirements

---

## Executive Summary

This architecture defines the Level 3 ROS2 control system for the ya_robot_manipulator warehouse automation platform. The system implements an action-based control hierarchy that bridges high-level task commands (Level 2 - Storage Optimization) with low-level hardware control (Level 4 - Hardware Interface). The architecture prioritizes:

1. **Action Composition** - Building complex warehouse operations from primitive motion actions
2. **Kinematic Chain Separation** - Treating 4 distinct mechanical subsystems independently
3. **Simulation/Hardware Separation** - Core logic remains hardware-agnostic
4. **Extensibility** - Easy addition of new actions and behaviors

The system follows proven ROS2 patterns: ros2_control for hardware abstraction, ROS2 Actions for long-running tasks with feedback, and composable nodes for efficient execution.

---

## System Architecture Overview

### Four-Level System Hierarchy

```
┌─────────────────────────────────────────────────────────┐
│ Level 1: Accounting System (External - Not in scope)   │
└─────────────────────────────────────────────────────────┘
                          │
┌─────────────────────────────────────────────────────────┐
│ Level 2: Storage Optimization (External - Future)      │
│  - Issues high-level tasks via REST/MQTT               │
│  - "Pick item X from box Y at address Z"               │
└─────────────────────────────────────────────────────────┘
                          │
                    [Bridge Node]
                          │
┌─────────────────────────────────────────────────────────┐
│ Level 3: ROS2 Control System (THIS ARCHITECTURE)       │
│                                                         │
│  ┌─────────────────────────────────────────────┐      │
│  │  GUI Command Node (Testing/Simulation)      │      │
│  │  - Replaces Level 2 during development      │      │
│  │  - Issues high-level action goals           │      │
│  └─────────────────────────────────────────────┘      │
│                     │                                   │
│  ┌─────────────────────────────────────────────┐      │
│  │         Action Server Hierarchy              │      │
│  │                                               │      │
│  │  High-Level Actions (Warehouse Operations)   │      │
│  │  - PickItemFromStorage                       │      │
│  │  - StoreItemInContainer                      │      │
│  │  - ReturnBox                                 │      │
│  │                                               │      │
│  │  Mid-Level Actions (Kinematic Operations)    │      │
│  │  - NavigateToAddress (railway+selector)      │      │
│  │  - ExtractBox (gripper+trajectory)           │      │
│  │  - PickItem (picker mechanism)               │      │
│  │  - ManipulateContainer (container jaws)      │      │
│  │                                               │      │
│  │  Low-Level Actions (Joint Control)           │      │
│  │  - MoveJointGroup                            │      │
│  │  - ExecuteTrajectory                         │      │
│  └─────────────────────────────────────────────┘      │
│                     │                                   │
│  ┌─────────────────────────────────────────────┐      │
│  │      State Management & Coordination         │      │
│  │  - Manipulator state tracker                │      │
│  │  - Safety monitors                           │      │
│  │  - Error recovery                            │      │
│  └─────────────────────────────────────────────┘      │
└─────────────────────────────────────────────────────────┘
                          │
┌─────────────────────────────────────────────────────────┐
│ Level 4: Hardware Interface (Existing)                 │
│  - ros2_control hardware interface                     │
│  - JointTrajectoryController                           │
│  - Gazebo simulation OR real hardware                  │
└─────────────────────────────────────────────────────────┘
```

---

## Package Structure

The system is split into **two ROS2 packages** with clear separation of concerns:

### Package 1: `manipulator_control` (NEW - Focus of this architecture)

**Purpose:** Level 3 control logic - actions, coordination, and Level 2 bridge

**Location:** `/ros2_ws/src/manipulator_control/`

```
manipulator_control/
├── package.xml
├── setup.py
├── CMakeLists.txt (if C++ nodes needed)
├── resource/
│   └── manipulator_control
├── config/
│   ├── action_servers.yaml          # Action server configurations
│   ├── kinematic_chains.yaml        # Joint groups for each subsystem
│   ├── warehouse_addresses.yaml     # Cabinet layout and coordinates
│   └── gui_defaults.yaml            # GUI default values
├── launch/
│   ├── control_system.launch.py     # Main launch: all action servers
│   ├── gui_commander.launch.py      # GUI node for testing
│   └── bridge_node.launch.py        # Future: Level 2 bridge
├── manipulator_control/
│   ├── __init__.py
│   │
│   ├── actions/                     # Action server implementations
│   │   ├── __init__.py
│   │   │
│   │   ├── high_level/              # Warehouse operation actions
│   │   │   ├── __init__.py
│   │   │   ├── pick_item_from_storage.py
│   │   │   ├── store_item_in_container.py
│   │   │   └── return_box.py
│   │   │
│   │   ├── mid_level/               # Kinematic operation actions
│   │   │   ├── __init__.py
│   │   │   ├── navigate_to_address.py
│   │   │   ├── extract_box.py
│   │   │   ├── pick_item.py
│   │   │   └── manipulate_container.py
│   │   │
│   │   └── low_level/               # Joint control actions
│   │       ├── __init__.py
│   │       ├── move_joint_group.py
│   │       └── execute_trajectory.py
│   │
│   ├── state/                       # State management
│   │   ├── __init__.py
│   │   ├── manipulator_state.py     # Current pose, joint states
│   │   ├── warehouse_state.py       # Box locations, container status
│   │   └── safety_monitor.py        # Collision avoidance, limits
│   │
│   ├── bridge/                      # Level 2 communication (future)
│   │   ├── __init__.py
│   │   ├── rest_bridge.py           # REST API server
│   │   ├── mqtt_bridge.py           # MQTT subscriber
│   │   └── message_translator.py    # Level 2 ↔ ROS2 Action conversion
│   │
│   ├── gui/                         # Testing GUI (replaces bridge)
│   │   ├── __init__.py
│   │   ├── command_panel.py         # Main GUI node
│   │   ├── widgets/
│   │   │   ├── __init__.py
│   │   │   ├── action_selector.py   # Dropdown for action types
│   │   │   ├── parameter_input.py   # Input fields for action params
│   │   │   └── status_display.py    # Feedback and result display
│   │   └── utils/
│   │       ├── __init__.py
│   │       └── action_client_manager.py
│   │
│   ├── planning/                    # Motion planning utilities
│   │   ├── __init__.py
│   │   ├── trajectory_generator.py  # Simple trajectory planning
│   │   ├── kinematics.py            # Forward/inverse kinematics helpers
│   │   └── collision_checker.py     # Basic collision detection
│   │
│   └── utils/                       # Common utilities
│       ├── __init__.py
│       ├── joint_groups.py          # Joint group definitions
│       ├── coordinate_transforms.py # TF utilities
│       └── error_handlers.py        # Error recovery patterns
│
├── msg/                             # Custom message definitions
│   ├── ManipulatorState.msg
│   ├── WarehouseAddress.msg
│   └── ItemDescriptor.msg
│
├── srv/                             # Service definitions
│   ├── GetManipulatorState.srv
│   └── ValidateAddress.srv
│
├── action/                          # Action definitions
│   ├── PickItemFromStorage.action
│   ├── NavigateToAddress.action
│   ├── ExtractBox.action
│   ├── PickItem.action
│   ├── ManipulateContainer.action
│   ├── MoveJointGroup.action
│   └── ExecuteTrajectory.action
│
└── test/                            # Unit and integration tests
    ├── test_action_servers.py
    ├── test_state_management.py
    └── test_trajectory_generation.py
```

### Package 2: `manipulator_description` (EXISTING - Hardware layer)

**Purpose:** URDF, ros2_control interfaces, controller configurations

**Location:** `/ros2_ws/src/manipulator_description/`

**Key Files:**
- `urdf/manipulator/ros2_control.xacro` - Hardware interface definition
- `config/manipulator_controllers.yaml` - Controller parameters
- Launch files for Gazebo simulation

**Separation Rationale:**
- Hardware interface changes rarely
- Control logic evolves frequently
- Clean boundary between simulation and control algorithms
- Easier to swap hardware implementations

---

## Kinematic Chain Architecture

The manipulator has **9 degrees of freedom** organized into **4 functional kinematic chains**:

### 1. Railway + Selector Chain (Navigation)

**Purpose:** Position manipulator at target cabinet address

**Joints:**
- `base_main_frame_joint` (X-axis railway: 0.0 → 4.0 m)
- `main_frame_selector_frame_joint` (Z-axis vertical: -0.01 → 1.5 m)

**Actions Using This Chain:**
- `NavigateToAddress` - Move to cabinet row/column coordinates
- High-level actions use this for positioning before box extraction

**Control Mode:** Position control with velocity limits

### 2. Gripper + Selector Chain (Box Extraction)

**Purpose:** Extract boxes from cabinet shelves using magnetic gripper

**Joints:**
- `main_frame_selector_frame_joint` (Z-axis - shared with navigation)
- `selector_frame_gripper_joint` (Y-axis magnet holder: -0.4 → 0.4 m)

**Actions Using This Chain:**
- `ExtractBox` - Coordinated motion to reach into cabinet and pull box
- Trajectory planning required for collision avoidance with cabinet frames

**Control Mode:** Coordinated trajectory execution

**Notes:**
- Selector Z-axis joint is shared between navigation and extraction
- Requires careful state management to avoid conflicts

### 3. Picker Chain (Item Manipulation)

**Purpose:** Pick individual items from extracted boxes

**Joints:**
- `selector_frame_picker_frame_joint` (Z-axis picker vertical: -0.01 → 0.3 m)
- `picker_frame_picker_rail_joint` (Y-axis picker rail: -0.3 → 0.3 m)
- `picker_rail_picker_base_joint` (X-axis picker slider: 0.0 → 0.12 m)
- `picker_base_picker_jaw_joint` (X-axis picker extension: 0.0 → 0.2 m)

**Actions Using This Chain:**
- `PickItem` - Fine manipulation to grasp items from boxes

**Control Mode:** Precision position control

### 4. Container Jaw Chain (Container Management)

**Purpose:** Grip and manipulate storage container attached to manipulator

**Joints:**
- `selector_left_container_jaw_joint` (-0.2 → 0.2 m)
- `selector_right_container_jaw_joint` (-0.2 → 0.2 m)

**Actions Using This Chain:**
- `ManipulateContainer` - Open/close jaws to grip container
- Used during high-level storage operations

**Control Mode:** Synchronized position control (mimic behavior in software)

**Notes:**
- Right jaw was originally mimic joint, but dartsim doesn't support mimic
- Both jaws controlled independently, software ensures synchronization

---

## Action Hierarchy Design

### Action Design Philosophy

1. **Three-Level Hierarchy:**
   - **High-Level:** Complete warehouse operations (business logic)
   - **Mid-Level:** Kinematic operations (motion primitives)
   - **Low-Level:** Joint control (hardware abstraction)

2. **Composability:**
   - High-level actions call mid-level actions as clients
   - Mid-level actions call low-level actions
   - Each level provides feedback to its caller

3. **Idempotency:**
   - Actions can be safely retried
   - State checks before execution
   - Graceful handling of partial completion

4. **Feedback Design:**
   - Progress percentage
   - Current operation description
   - Estimated time remaining (if applicable)

### High-Level Actions (Warehouse Operations)

#### `PickItemFromStorage`

**Purpose:** Complete operation to retrieve an item from warehouse storage

**Goal:**
```yaml
warehouse_address:
  row: int          # Cabinet row (0-N)
  column: int       # Cabinet column (0-M)
  shelf: int        # Shelf height (0-K)
item_id: string     # Unique item identifier
target_container: string  # Container to place item in
```

**Feedback:**
```yaml
current_operation: string  # "navigating", "extracting_box", "picking_item", "returning_box"
progress_percent: int      # 0-100
estimated_time_remaining: duration
```

**Result:**
```yaml
success: bool
item_picked: bool
item_stored_in_container: bool
box_returned: bool
final_message: string
```

**Implementation Pattern:**
1. Call `NavigateToAddress(address)`
2. Call `ExtractBox(address)`
3. Call `PickItem(item_id, item_position)`
4. Call `ManipulateContainer(PLACE_ITEM)`
5. Call `ReturnBox(address)`

**Error Recovery:**
- Failed navigation → Retry with safety check
- Failed extraction → Check box presence, retry
- Failed pick → Mark item unreachable, log for Level 2
- Any failure → Return box before aborting

#### `StoreItemInContainer`

**Purpose:** Place an item (already in gripper) into storage container

**Goal:**
```yaml
item_id: string
target_container_slot: int
```

**Implementation:** Simplified operation, mainly container jaw control

#### `ReturnBox`

**Purpose:** Return extracted box to its cabinet position

**Goal:**
```yaml
warehouse_address: WarehouseAddress
```

**Implementation:** Reverse of `ExtractBox` operation

---

### Mid-Level Actions (Kinematic Operations)

#### `NavigateToAddress`

**Purpose:** Move manipulator to position aligned with target cabinet address

**Goal:**
```yaml
warehouse_address:
  row: int
  column: int
  shelf: int
approach_distance: float  # Distance from cabinet face (default: 0.1m)
```

**Feedback:**
```yaml
current_joint_positions: float[]
distance_to_target: float
progress_percent: int
```

**Result:**
```yaml
success: bool
final_position: geometry_msgs/Pose
positioning_error: float  # meters
```

**Implementation:**
1. Load warehouse layout from `warehouse_addresses.yaml`
2. Calculate target (x, z) coordinates for address
3. Generate trajectory for railway + selector joints
4. Execute via `ExecuteTrajectory` action
5. Verify final position within tolerance

**Joint Groups Used:**
- Railway + Selector (2 DOF)

**Coordinate Mapping:**
```python
# Example calculation
x_position = row * row_spacing + row_offset
z_position = shelf * shelf_spacing + base_height
```

#### `ExtractBox`

**Purpose:** Reach into cabinet and extract box using magnetic gripper

**Goal:**
```yaml
warehouse_address: WarehouseAddress
box_depth: float  # How far to reach into cabinet
extraction_speed: float  # Slow for safety
```

**Feedback:**
```yaml
current_phase: string  # "approaching", "engaging_magnet", "extracting", "clearing"
gripper_depth: float
magnet_contact_detected: bool
progress_percent: int
```

**Result:**
```yaml
success: bool
box_extracted: bool
magnet_engaged: bool
```

**Implementation:**
1. Verify manipulator at correct address (`NavigateToAddress` complete)
2. **Phase 1: Approach**
   - Move gripper Y-axis toward cabinet (negative direction)
   - Slow approach until depth reached
3. **Phase 2: Engage Magnet**
   - Activate electromagnet (via GPIO or service call)
   - Wait for contact confirmation
4. **Phase 3: Extract**
   - Slowly retract gripper Y-axis (positive direction)
   - Monitor for resistance (stuck box detection)
5. **Phase 4: Clear**
   - Move to safe position away from cabinet

**Joint Groups Used:**
- Gripper Y-axis
- (Optional) Selector Z-axis for trajectory optimization

**Safety Considerations:**
- Collision detection with cabinet frame
- Force limits on gripper extension
- Magnet power monitoring

#### `PickItem`

**Purpose:** Use picker mechanism to grasp item from extracted box

**Goal:**
```yaml
item_position:
  x: float  # Position in box coordinate frame
  y: float
  z: float
grasp_type: string  # "top_grasp", "side_grasp"
```

**Feedback:**
```yaml
current_phase: string  # "positioning", "approaching", "grasping", "retracting"
picker_position: geometry_msgs/Point
progress_percent: int
```

**Result:**
```yaml
success: bool
item_grasped: bool
grasp_quality: float  # 0.0-1.0 confidence
```

**Implementation:**
1. Calculate picker joint positions for target in box frame
2. Move picker to position above item
3. Descend picker Z-axis
4. Extend picker jaw/gripper
5. Close gripper (vacuum, mechanical jaw, or magnet)
6. Retract upward
7. Verify grasp success (sensor feedback)

**Joint Groups Used:**
- Picker chain (4 DOF)

#### `ManipulateContainer`

**Purpose:** Control container jaws for item placement

**Goal:**
```yaml
operation: string  # "OPEN", "CLOSE", "RELEASE_ITEM"
jaw_opening: float  # Target opening width (meters)
```

**Feedback:**
```yaml
current_jaw_positions: float[2]  # [left, right]
opening_width: float
progress_percent: int
```

**Result:**
```yaml
success: bool
final_opening: float
```

**Implementation:**
1. Calculate target positions for left/right jaws
2. Ensure synchronized motion (software mimic)
3. Execute coordinated movement
4. Verify final positions match (within tolerance)

**Joint Groups Used:**
- Container jaws (2 DOF synchronized)

**Synchronization Logic:**
```python
# Mimic behavior in software
left_target = -jaw_opening / 2.0
right_target = jaw_opening / 2.0
# Send both commands simultaneously
```

---

### Low-Level Actions (Joint Control)

#### `MoveJointGroup`

**Purpose:** Move a named group of joints to target positions

**Goal:**
```yaml
joint_group: string  # "navigation", "picker", "gripper", "container"
target_positions: float[]
max_velocity: float
motion_profile: string  # "trapezoidal", "s_curve"
```

**Feedback:**
```yaml
current_positions: float[]
velocity: float[]
progress_percent: int
```

**Result:**
```yaml
success: bool
final_positions: float[]
position_error: float[]
execution_time: duration
```

**Implementation:**
1. Load joint group from `kinematic_chains.yaml`
2. Validate target positions within joint limits
3. Generate simple point-to-point trajectory
4. Send to appropriate JointTrajectoryController
5. Monitor execution, provide feedback

**Joint Groups Defined in Config:**
```yaml
# kinematic_chains.yaml
joint_groups:
  navigation:
    joints: [base_main_frame_joint, main_frame_selector_frame_joint]
  gripper:
    joints: [selector_frame_gripper_joint]
  picker:
    joints: [selector_frame_picker_frame_joint, picker_frame_picker_rail_joint,
             picker_rail_picker_base_joint, picker_base_picker_jaw_joint]
  container:
    joints: [selector_left_container_jaw_joint, selector_right_container_jaw_joint]
```

#### `ExecuteTrajectory`

**Purpose:** Execute a pre-computed joint trajectory with precise timing

**Goal:**
```yaml
joint_names: string[]
trajectory_points:
  - positions: float[]
    velocities: float[]
    accelerations: float[]
    time_from_start: duration
```

**Feedback:**
```yaml
current_point_index: int
time_elapsed: duration
progress_percent: int
```

**Result:**
```yaml
success: bool
trajectory_error: float
final_positions: float[]
```

**Implementation:**
1. Validate trajectory (continuity, limits, timing)
2. Convert to `trajectory_msgs/JointTrajectory`
3. Send to ros2_control JointTrajectoryController
4. Monitor execution via `/joint_states`
5. Detect deviations and trigger error recovery if needed

**Used By:**
- All mid-level actions that need coordinated motion
- Motion planning outputs

---

## State Management Architecture

### Manipulator State Tracker

**Purpose:** Centralized state management for manipulator system

**Published Topics:**
- `/manipulator/state` (Custom `ManipulatorState` msg) @ 10 Hz

**State Information:**
```yaml
# ManipulatorState.msg
header: std_msgs/Header
current_action: string
action_status: string  # "idle", "executing", "paused", "error"
joint_positions: float[]
joint_velocities: float[]
current_address: WarehouseAddress
box_extracted: bool
box_id: string
item_in_gripper: bool
magnet_engaged: bool
errors: string[]
```

**Services Provided:**
- `/manipulator/get_state` - Query current state
- `/manipulator/reset_state` - Emergency reset

**Implementation:**
- Subscribes to `/joint_states`
- Monitors all action server statuses
- Aggregates sensor data (magnet status, gripper sensors)
- Publishes unified state

### Warehouse State Manager

**Purpose:** Track warehouse layout and box/item locations

**Services:**
- `/warehouse/validate_address` - Check if address is valid
- `/warehouse/get_box_at_address` - Query box information
- `/warehouse/update_box_location` - Update after manipulation

**State Storage:**
- In-memory map of addresses → box IDs
- Could be backed by database or ROS parameter server
- For now: Simple YAML config file

**Configuration:**
```yaml
# warehouse_addresses.yaml
layout:
  rows: 10
  columns: 8
  shelves_per_column: 5
  row_spacing: 0.4  # meters
  shelf_spacing: 0.3  # meters
  base_height: 0.1  # meters from ground

addresses:
  - row: 0
    column: 2
    shelf: 3
    box_id: "BOX_12345"
    items: ["ITEM_001", "ITEM_002"]
  # ... more addresses
```

### Safety Monitor

**Purpose:** Real-time safety checks and emergency stop

**Monitored Conditions:**
- Joint position limits (soft limits before hardware limits)
- Velocity limits exceeded
- Collision detection (simple bounding box checks)
- Communication timeouts
- Emergency stop button status

**Behaviors:**
- Publish warnings on `/manipulator/safety_warnings`
- Trigger action preemption if critical violation
- Log all safety events

**Implementation:**
```python
class SafetyMonitor(Node):
    def __init__(self):
        # Subscribe to joint_states, check limits
        # Subscribe to collision sensor topics
        # Monitor action server heartbeats
        pass

    def check_joint_limits(self, joint_states):
        # Compare to soft limits
        # Issue warnings if approaching limits
        pass

    def emergency_stop(self):
        # Preempt all active actions
        # Send zero velocity commands
        # Publish emergency state
        pass
```

---

## GUI Command Node Architecture

**Purpose:** Testing and simulation interface to replace Level 2 bridge during development

**Technology:** Python + PyQt5 (or RQt plugin)

**Rationale:**
- PyQt5 has good ROS2 integration patterns
- RQt plugins integrate with ROS2 tooling
- Easier development than web-based GUI for testing
- Can run alongside RViz for visualization

### GUI Features

#### Main Window Layout

```
┌────────────────────────────────────────────────────────┐
│  Manipulator Control GUI                        [X]    │
├────────────────────────────────────────────────────────┤
│                                                        │
│  ┌─── Action Selection ──────────────────────────┐   │
│  │                                                │   │
│  │  Action Type: [PickItemFromStorage     ▼]     │   │
│  │                                                │   │
│  └────────────────────────────────────────────────┘   │
│                                                        │
│  ┌─── Parameters ────────────────────────────────┐   │
│  │                                                │   │
│  │  Row:     [3    ]   Column: [5    ]           │   │
│  │  Shelf:   [2    ]                             │   │
│  │  Item ID: [ITEM_12345]                        │   │
│  │  Container: [CONT_A]                          │   │
│  │                                                │   │
│  │         [Load Presets ▼]  [Save Preset]       │   │
│  │                                                │   │
│  └────────────────────────────────────────────────┘   │
│                                                        │
│  ┌─── Action Control ────────────────────────────┐   │
│  │                                                │   │
│  │     [Send Goal]  [Cancel]  [Emergency Stop]   │   │
│  │                                                │   │
│  └────────────────────────────────────────────────┘   │
│                                                        │
│  ┌─── Status & Feedback ────────────────────────┐    │
│  │                                                │   │
│  │  Status: Executing                            │   │
│  │  Operation: Extracting box from cabinet       │   │
│  │                                                │   │
│  │  Progress: ████████████░░░░░░░░  60%          │   │
│  │                                                │   │
│  │  [Clear] ──────────────────────────────────── │   │
│  │  [12:34:56] Goal accepted: PickItemFromStorage│   │
│  │  [12:34:57] Navigating to address (3, 5, 2)   │   │
│  │  [12:35:02] Navigation complete               │   │
│  │  [12:35:03] Extracting box...                 │   │
│  │                                                │   │
│  └────────────────────────────────────────────────┘   │
│                                                        │
│  ┌─── System State ──────────────────────────────┐   │
│  │                                                │   │
│  │  Current Address: (3, 5, 2)                   │   │
│  │  Box Extracted: Yes (BOX_789)                 │   │
│  │  Magnet Status: Engaged                       │   │
│  │                                                │   │
│  └────────────────────────────────────────────────┘   │
│                                                        │
└────────────────────────────────────────────────────────┘
```

#### Supported Actions in GUI

**High-Level:**
1. Pick Item From Storage
2. Store Item In Container
3. Return Box

**Mid-Level (for testing):**
4. Navigate To Address
5. Extract Box
6. Pick Item
7. Manipulate Container

**Low-Level (for debugging):**
8. Move Joint Group
9. Execute Trajectory (load from file)

#### Parameter Input Forms

Dynamic form generation based on action type:

```python
action_params = {
    'PickItemFromStorage': [
        ('row', 'int', 0, 10),
        ('column', 'int', 0, 8),
        ('shelf', 'int', 0, 5),
        ('item_id', 'string'),
        ('target_container', 'string')
    ],
    'NavigateToAddress': [
        ('row', 'int', 0, 10),
        ('column', 'int', 0, 8),
        ('shelf', 'int', 0, 5),
        ('approach_distance', 'float', 0.0, 0.5)
    ],
    # ... more actions
}
```

### GUI Implementation Structure

```python
# manipulator_control/gui/command_panel.py

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from PyQt5.QtWidgets import (QMainWindow, QWidget, QVBoxLayout,
                              QComboBox, QPushButton, QTextEdit, QProgressBar)
from PyQt5.QtCore import QTimer, pyqtSignal

class ManipulatorCommandPanel(Node):
    """Main GUI node for issuing manipulator commands"""

    # Qt signals for thread-safe GUI updates
    feedback_signal = pyqtSignal(str)
    progress_signal = pyqtSignal(int)
    result_signal = pyqtSignal(str, bool)

    def __init__(self):
        super().__init__('manipulator_command_panel')

        # Action clients for each action type
        self.action_clients = {
            'PickItemFromStorage': ActionClient(
                self, PickItemFromStorage, 'pick_item_from_storage'
            ),
            # ... more clients
        }

        # GUI setup
        self.init_ui()

        # Qt timer for ROS2 spinning
        self.ros_timer = QTimer()
        self.ros_timer.timeout.connect(self.spin_ros)
        self.ros_timer.start(100)  # 10 Hz

    def init_ui(self):
        """Initialize PyQt5 GUI"""
        # Build widget tree as shown in layout above
        pass

    def spin_ros(self):
        """Spin ROS2 executor in Qt event loop"""
        rclpy.spin_once(self, timeout_sec=0)

    def send_goal_clicked(self):
        """Handle Send Goal button click"""
        action_type = self.action_selector.currentText()
        params = self.get_current_parameters()

        goal = self.build_goal(action_type, params)
        client = self.action_clients[action_type]

        future = client.send_goal_async(
            goal,
            feedback_callback=self.feedback_callback
        )
        future.add_done_callback(self.goal_response_callback)

    def feedback_callback(self, feedback_msg):
        """Handle action feedback (runs in ROS thread)"""
        feedback = feedback_msg.feedback
        # Emit Qt signal for thread-safe GUI update
        self.feedback_signal.emit(feedback.current_operation)
        self.progress_signal.emit(feedback.progress_percent)

    def goal_response_callback(self, future):
        """Handle goal acceptance/rejection"""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.result_signal.emit("Goal rejected", False)
            return

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.result_callback)

    def result_callback(self, future):
        """Handle action result"""
        result = future.result().result
        success = result.success
        message = result.final_message
        self.result_signal.emit(message, success)
```

### GUI Launch Configuration

```python
# launch/gui_commander.launch.py

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='manipulator_control',
            executable='command_panel_node',
            name='command_panel',
            output='screen',
            parameters=[
                {'gui_defaults_file': 'config/gui_defaults.yaml'}
            ]
        ),
    ])
```

### Alternative: RQt Plugin

If using RQt plugin approach:

```python
# Inherit from rqt_gui_py.plugin.Plugin
# Register plugin in plugin.xml
# Load via rqt framework

class ManipulatorControlPlugin(Plugin):
    def __init__(self, context):
        super().__init__(context)
        self._widget = QWidget()
        # Same GUI code as above
        context.add_widget(self._widget)
```

**Advantages of RQt plugin:**
- Integrates with RQt perspective saving
- Can be combined with other RQt plugins
- Standard ROS2 plugin discovery

**Advantages of standalone PyQt5:**
- Simpler deployment
- More flexibility in window management
- Easier debugging

**Recommendation:** Start with standalone PyQt5, consider RQt plugin later if needed

---

## Level 2 Bridge Architecture (Future Implementation)

**Purpose:** Interface between Storage Optimization System (Level 2) and ROS2 actions

**Note:** This section defines the contract for future implementation. GUI node currently replaces this.

### Communication Options

#### Option 1: REST API Bridge

**Advantages:**
- Language agnostic
- Easy to test with curl/Postman
- Standard HTTP status codes
- Can use existing web frameworks

**Technology Stack:**
- Python Flask or FastAPI
- Runs as ROS2 node
- Converts REST → ROS2 Actions

**Endpoints:**

```
POST /api/v1/tasks/pick_item
  Body:
    {
      "task_id": "TASK_12345",
      "warehouse_address": {
        "row": 3,
        "column": 5,
        "shelf": 2
      },
      "item_id": "ITEM_789",
      "target_container": "CONT_A",
      "priority": "normal"
    }

  Response:
    {
      "status": "accepted",
      "action_id": "uuid",
      "estimated_completion": "2025-11-24T13:45:00Z"
    }

GET /api/v1/tasks/{action_id}/status
  Response:
    {
      "action_id": "uuid",
      "status": "executing",
      "progress_percent": 60,
      "current_operation": "extracting_box",
      "estimated_time_remaining": "PT45S"
    }

POST /api/v1/tasks/{action_id}/cancel
  Response:
    {
      "status": "cancelling"
    }
```

**Implementation Node:**

```python
# bridge/rest_bridge.py

from flask import Flask, request, jsonify
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
import threading

class RestBridgeNode(Node):
    def __init__(self):
        super().__init__('rest_bridge')

        # Action clients
        self.pick_client = ActionClient(
            self, PickItemFromStorage, 'pick_item_from_storage'
        )

        # Flask app
        self.app = Flask(__name__)
        self.setup_routes()

        # Track active actions
        self.active_actions = {}

    def setup_routes(self):
        @self.app.route('/api/v1/tasks/pick_item', methods=['POST'])
        def pick_item():
            data = request.json
            # Convert REST → ROS2 Action goal
            goal = self.build_pick_goal(data)

            # Send goal
            future = self.pick_client.send_goal_async(goal)
            action_id = str(uuid.uuid4())

            # Store action handle
            self.active_actions[action_id] = future

            return jsonify({
                'status': 'accepted',
                'action_id': action_id
            })

        # ... more routes

    def run_flask(self):
        self.app.run(host='0.0.0.0', port=5000)

    def run(self):
        # Run Flask in separate thread
        flask_thread = threading.Thread(target=self.run_flask)
        flask_thread.start()

        # Spin ROS2 node
        rclpy.spin(self)
```

#### Option 2: MQTT Bridge

**Advantages:**
- Publish/subscribe pattern
- Better for event-driven systems
- Lower latency
- Good for distributed systems

**Technology Stack:**
- Python paho-mqtt
- MQTT broker (Mosquitto)
- ROS2 node as MQTT client

**Topics:**

```
# Level 2 publishes commands
warehouse/commands/pick_item
  Payload:
    {
      "task_id": "TASK_12345",
      "warehouse_address": {...},
      "item_id": "ITEM_789",
      "target_container": "CONT_A"
    }

# ROS2 publishes status updates
warehouse/status/TASK_12345
  Payload:
    {
      "task_id": "TASK_12345",
      "status": "executing",
      "progress_percent": 60,
      "current_operation": "extracting_box"
    }

# ROS2 publishes results
warehouse/results/TASK_12345
  Payload:
    {
      "task_id": "TASK_12345",
      "success": true,
      "completion_time": "2025-11-24T13:45:00Z"
    }
```

**Implementation Node:**

```python
# bridge/mqtt_bridge.py

import paho.mqtt.client as mqtt
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
import json

class MqttBridgeNode(Node):
    def __init__(self):
        super().__init__('mqtt_bridge')

        # MQTT client
        self.mqtt_client = mqtt.Client()
        self.mqtt_client.on_connect = self.on_mqtt_connect
        self.mqtt_client.on_message = self.on_mqtt_message
        self.mqtt_client.connect('localhost', 1883, 60)

        # ROS2 action clients
        self.pick_client = ActionClient(
            self, PickItemFromStorage, 'pick_item_from_storage'
        )

    def on_mqtt_connect(self, client, userdata, flags, rc):
        # Subscribe to command topics
        client.subscribe('warehouse/commands/#')

    def on_mqtt_message(self, client, userdata, msg):
        topic = msg.topic
        payload = json.loads(msg.payload)

        if 'pick_item' in topic:
            self.handle_pick_command(payload)
        # ... more handlers

    def handle_pick_command(self, payload):
        task_id = payload['task_id']
        goal = self.build_pick_goal(payload)

        future = self.pick_client.send_goal_async(
            goal,
            feedback_callback=lambda fb: self.publish_status(task_id, fb)
        )
        future.add_done_callback(
            lambda f: self.publish_result(task_id, f)
        )

    def publish_status(self, task_id, feedback_msg):
        status = {
            'task_id': task_id,
            'status': 'executing',
            'progress_percent': feedback_msg.feedback.progress_percent,
            'current_operation': feedback_msg.feedback.current_operation
        }
        self.mqtt_client.publish(
            f'warehouse/status/{task_id}',
            json.dumps(status)
        )
```

### Message Translation Layer

**Purpose:** Convert between Level 2 message format and ROS2 Action goals

```python
# bridge/message_translator.py

class MessageTranslator:
    """Bidirectional translation between Level 2 and ROS2 formats"""

    @staticmethod
    def level2_to_ros2_pick_goal(level2_msg):
        """Convert Level 2 pick command to ROS2 PickItemFromStorage goal"""
        goal = PickItemFromStorage.Goal()
        goal.warehouse_address.row = level2_msg['warehouse_address']['row']
        goal.warehouse_address.column = level2_msg['warehouse_address']['column']
        goal.warehouse_address.shelf = level2_msg['warehouse_address']['shelf']
        goal.item_id = level2_msg['item_id']
        goal.target_container = level2_msg['target_container']
        return goal

    @staticmethod
    def ros2_feedback_to_level2_status(ros2_feedback, task_id):
        """Convert ROS2 action feedback to Level 2 status message"""
        return {
            'task_id': task_id,
            'status': 'executing',
            'progress_percent': ros2_feedback.progress_percent,
            'current_operation': ros2_feedback.current_operation,
            'estimated_time_remaining': ros2_feedback.estimated_time_remaining.to_sec()
        }

    @staticmethod
    def ros2_result_to_level2_result(ros2_result, task_id):
        """Convert ROS2 action result to Level 2 result message"""
        return {
            'task_id': task_id,
            'success': ros2_result.success,
            'item_picked': ros2_result.item_picked,
            'item_stored_in_container': ros2_result.item_stored_in_container,
            'box_returned': ros2_result.box_returned,
            'message': ros2_result.final_message
        }
```

### API Contract Definition

**Level 2 → Level 3 Requirements:**

1. **Task Identification:** Each command must have unique `task_id`
2. **Warehouse Address Format:** Consistent coordinate system (row, column, shelf)
3. **Item Identification:** Unique `item_id` for tracking
4. **Priority Levels:** (optional) "low", "normal", "high", "urgent"
5. **Timeout Handling:** Level 2 must handle action timeouts
6. **Cancellation Support:** Level 2 can cancel in-progress tasks

**Level 3 → Level 2 Guarantees:**

1. **Status Updates:** Periodic feedback at minimum 1 Hz
2. **Progress Reporting:** 0-100% completion estimate
3. **Error Reporting:** Structured error codes and messages
4. **Result Confirmation:** Clear success/failure indication
5. **State Persistence:** System state survives node restarts

**Error Codes:**

```python
ERROR_CODES = {
    1000: "INVALID_ADDRESS",
    1001: "BOX_NOT_FOUND",
    1002: "ITEM_NOT_REACHABLE",
    1003: "GRIPPER_FAILURE",
    1004: "NAVIGATION_FAILED",
    1005: "COLLISION_DETECTED",
    1006: "TIMEOUT",
    1007: "EMERGENCY_STOP",
    1008: "SYSTEM_FAULT"
}
```

---

## Technology Stack Decisions

### Core ROS2 Dependencies

| Component | Technology | Version | Rationale |
|-----------|-----------|---------|-----------|
| ROS2 Distribution | **Humble Hawksbill** | 22.04 LTS | Long-term support, stable, widely adopted |
| Hardware Abstraction | **ros2_control** | Latest (Humble) | Industry standard for hardware interfaces |
| Simulation | **Gazebo Sim (Ignition)** | Fortress/Garden | Modern physics, better sensor simulation |
| Action Interface | **rclpy.action** | Built-in | Python action servers/clients |
| TF Management | **tf2_ros** | Built-in | Standard transform handling |

### GUI Technology

| Component | Technology | Rationale |
|-----------|-----------|-----------|
| GUI Framework | **PyQt5** | Good ROS2 integration, mature, rich widgets |
| Alternative | **RQt Plugin** | Integrates with ROS2 tooling (consider for v2) |

### Bridge Technology (Future)

| Option | Technology | Use Case |
|--------|-----------|----------|
| REST API | **FastAPI** | Synchronous request/response, easy testing |
| MQTT | **paho-mqtt + Mosquitto** | Event-driven, low latency, pub/sub |
| **Recommendation** | **REST for initial implementation** | Simpler development, easier debugging |

### Development Tools

| Purpose | Tool |
|---------|------|
| Build System | colcon |
| Linting | ruff (Python), ament_lint (ROS2) |
| Testing | pytest + pytest-ros |
| Documentation | Sphinx + rosdoc2 |
| Version Control | Git |

---

## Implementation Patterns & Conventions

### Naming Conventions

**ROS2 Topics:**
- Format: `/<namespace>/<category>/<specific>`
- Example: `/manipulator/state/current_pose`
- Use lowercase with underscores

**ROS2 Actions:**
- Format: `<action_name>` (snake_case)
- Example: `pick_item_from_storage`
- Action definition files: PascalCase (e.g., `PickItemFromStorage.action`)

**ROS2 Services:**
- Format: `<service_name>` (snake_case)
- Example: `get_manipulator_state`

**Python Modules:**
- snake_case for files and functions
- PascalCase for classes
- UPPER_CASE for constants

**Configuration Files:**
- YAML format for all config files
- snake_case for keys
- Clear hierarchical structure

### Error Handling Strategy

**Action Server Error Handling:**

```python
class ActionServerErrorHandler:
    """Consistent error handling across all action servers"""

    @staticmethod
    def handle_error(exception, goal_handle, logger):
        """Standardized error handling"""
        error_type = type(exception).__name__
        error_msg = str(exception)

        logger.error(f"Action failed: {error_type} - {error_msg}")

        # Abort action with structured result
        result = goal_handle.action_type.Result()
        result.success = False
        result.error_code = ERROR_CODES.get(error_type, 9999)
        result.error_message = error_msg

        goal_handle.abort(result)
        return result
```

**Error Recovery Patterns:**

1. **Transient Failures:** Retry with exponential backoff
   - Network timeouts
   - Sensor glitches
   - Motor controller resets

2. **Configuration Errors:** Fail fast with clear message
   - Invalid parameters
   - Missing config files
   - Undefined joint groups

3. **Safety Violations:** Emergency stop, require manual reset
   - Joint limit exceeded
   - Collision detected
   - E-stop triggered

4. **Partial Completion:** Return to safe state, report progress
   - Action cancelled mid-execution
   - Timeout during operation

### Logging Standards

**Log Levels:**

```python
# DEBUG: Detailed debugging information
self.get_logger().debug(f"Joint positions: {joint_positions}")

# INFO: Normal operation milestones
self.get_logger().info("Navigation to address (3,5,2) complete")

# WARN: Unexpected but handled conditions
self.get_logger().warn("Approaching joint limit, slowing down")

# ERROR: Operation failed but system stable
self.get_logger().error("Failed to extract box: magnet not engaged")

# FATAL: Critical system failure
self.get_logger().fatal("Emergency stop triggered, all operations halted")
```

**Logging Format:**

```python
# Include context in log messages
self.get_logger().info(
    f"[{action_name}] {operation_phase}: {status_message}",
    throttle_duration_sec=1.0  # Avoid log spam
)

# Example output:
# [PickItemFromStorage] Extracting box: Gripper at depth 0.25m
```

### Testing Strategy

**Unit Tests:**
- Test individual action servers in isolation
- Mock ROS2 interfaces (topics, actions, services)
- Use pytest fixtures for common setups

```python
# test/test_action_servers.py

import pytest
from manipulator_control.actions.mid_level.navigate_to_address import NavigateToAddress

def test_address_calculation():
    """Test warehouse address to joint position calculation"""
    action_server = NavigateToAddress()

    address = {'row': 3, 'column': 5, 'shelf': 2}
    x, z = action_server.calculate_position(address)

    assert x == pytest.approx(1.2, abs=0.01)
    assert z == pytest.approx(0.7, abs=0.01)
```

**Integration Tests:**
- Launch full system in Gazebo
- Send action goals via test clients
- Verify expected robot behavior
- Check state updates

```python
# test/test_integration.py

import launch_testing
import rclpy
from rclpy.action import ActionClient

class TestNavigationIntegration(unittest.TestCase):
    def test_navigate_to_address(self):
        """Integration test: Navigate action in Gazebo"""
        rclpy.init()
        node = rclpy.create_node('test_client')

        client = ActionClient(node, NavigateToAddress, 'navigate_to_address')
        client.wait_for_server(timeout_sec=5.0)

        goal = NavigateToAddress.Goal()
        goal.warehouse_address.row = 3
        goal.warehouse_address.column = 5
        goal.warehouse_address.shelf = 2

        future = client.send_goal_async(goal)
        rclpy.spin_until_future_complete(node, future)

        goal_handle = future.result()
        assert goal_handle.accepted

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(node, result_future)

        result = result_future.result().result
        assert result.success
        assert result.positioning_error < 0.01
```

**Simulation Tests:**
- All action servers must pass in Gazebo before hardware
- Record bag files of successful operations for regression testing
- Visualize in RViz during development

### Configuration Management

**Hierarchical Config Files:**

```yaml
# config/action_servers.yaml
action_servers:
  navigate_to_address:
    action_type: "manipulator_control_msgs/action/NavigateToAddress"
    max_concurrent_goals: 1
    timeout_sec: 30.0

  extract_box:
    action_type: "manipulator_control_msgs/action/ExtractBox"
    max_concurrent_goals: 1
    timeout_sec: 45.0
    gripper_speed_slow: 0.05  # m/s
    gripper_speed_fast: 0.2   # m/s
    magnet_engage_delay: 1.0  # seconds
```

**Loading Pattern:**

```python
class ActionServerBase(Node):
    def __init__(self, config_key):
        super().__init__(config_key + '_server')

        # Declare and get parameters
        self.declare_parameter('config_file', 'config/action_servers.yaml')
        config_file = self.get_parameter('config_file').value

        # Load YAML
        with open(config_file, 'r') as f:
            config = yaml.safe_load(f)

        self.config = config['action_servers'][config_key]

        # Apply configuration
        self.timeout = self.config['timeout_sec']
```

---

## Data Architecture

### Custom Message Definitions

#### `WarehouseAddress.msg`

```
# Identifies a specific location in the warehouse
uint32 row      # Cabinet row (0-N)
uint32 column   # Cabinet column (0-M)
uint32 shelf    # Shelf height (0-K)
```

#### `ItemDescriptor.msg`

```
# Describes an item to be picked
string item_id                    # Unique identifier
geometry_msgs/Point position      # Position in box frame
string grasp_type                 # "top", "side", "pinch"
float32 estimated_weight          # kg (optional)
```

#### `ManipulatorState.msg`

```
std_msgs/Header header

# Current action
string current_action             # Empty if idle
string action_status              # "idle", "executing", "paused", "error"

# Joint state
float64[] joint_positions
float64[] joint_velocities
string[] joint_names

# Location context
WarehouseAddress current_address
bool at_valid_address

# Gripper/Picker state
bool box_extracted
string box_id                     # Empty if no box
bool magnet_engaged
bool item_in_picker

# Container state
float64 container_jaw_opening     # meters

# Safety
string[] active_warnings
string[] errors
```

### Database Schema (Optional Future Enhancement)

If persistent storage needed for warehouse state:

**Tables:**

```sql
-- Warehouse layout
CREATE TABLE warehouse_addresses (
    id SERIAL PRIMARY KEY,
    row INTEGER NOT NULL,
    column INTEGER NOT NULL,
    shelf INTEGER NOT NULL,
    UNIQUE(row, column, shelf)
);

-- Boxes
CREATE TABLE boxes (
    id SERIAL PRIMARY KEY,
    box_id VARCHAR(50) UNIQUE NOT NULL,
    current_address_id INTEGER REFERENCES warehouse_addresses(id),
    is_extracted BOOLEAN DEFAULT FALSE,
    last_accessed TIMESTAMP
);

-- Items
CREATE TABLE items (
    id SERIAL PRIMARY KEY,
    item_id VARCHAR(50) UNIQUE NOT NULL,
    current_box_id INTEGER REFERENCES boxes(id),
    position_x FLOAT,
    position_y FLOAT,
    position_z FLOAT,
    grasp_type VARCHAR(20),
    weight_kg FLOAT
);

-- Action history
CREATE TABLE action_history (
    id SERIAL PRIMARY KEY,
    action_type VARCHAR(100) NOT NULL,
    goal_data JSONB,
    result_data JSONB,
    success BOOLEAN,
    started_at TIMESTAMP,
    completed_at TIMESTAMP,
    duration_sec FLOAT
);
```

**For initial implementation:** Use YAML files for warehouse layout, expand to DB if needed

---

## API Contracts

### Action Definitions

All action definition files located in `manipulator_control/action/`

#### High-Level Actions

**PickItemFromStorage.action**

```
# Goal
WarehouseAddress warehouse_address
string item_id
string target_container
---
# Result
bool success
bool item_picked
bool item_stored_in_container
bool box_returned
string final_message
uint32 error_code
---
# Feedback
string current_operation
uint8 progress_percent
duration estimated_time_remaining
geometry_msgs/Pose current_pose
```

**StoreItemInContainer.action**

```
# Goal
string item_id
uint32 target_container_slot
---
# Result
bool success
string final_message
---
# Feedback
string current_operation
uint8 progress_percent
```

**ReturnBox.action**

```
# Goal
WarehouseAddress warehouse_address
string box_id
---
# Result
bool success
bool box_returned_successfully
string final_message
---
# Feedback
string current_operation
uint8 progress_percent
```

#### Mid-Level Actions

**NavigateToAddress.action**

```
# Goal
WarehouseAddress warehouse_address
float32 approach_distance  # Default: 0.1m from cabinet face
---
# Result
bool success
geometry_msgs/Pose final_position
float32 positioning_error  # meters
string message
---
# Feedback
float64[] current_joint_positions
float32 distance_to_target
uint8 progress_percent
```

**ExtractBox.action**

```
# Goal
WarehouseAddress warehouse_address
float32 box_depth       # How far to reach (default: 0.3m)
float32 extraction_speed  # m/s (default: 0.05)
---
# Result
bool success
bool box_extracted
bool magnet_engaged
string box_id
string message
---
# Feedback
string current_phase  # "approaching", "engaging_magnet", "extracting", "clearing"
float32 gripper_depth
bool magnet_contact_detected
uint8 progress_percent
```

**PickItem.action**

```
# Goal
ItemDescriptor item
---
# Result
bool success
bool item_grasped
float32 grasp_quality  # 0.0-1.0
string message
---
# Feedback
string current_phase  # "positioning", "approaching", "grasping", "retracting"
geometry_msgs/Point picker_position
uint8 progress_percent
```

**ManipulateContainer.action**

```
# Goal
string operation  # "OPEN", "CLOSE", "RELEASE_ITEM"
float32 jaw_opening  # Target width in meters
---
# Result
bool success
float32 final_opening
string message
---
# Feedback
float64[2] current_jaw_positions  # [left, right]
float32 opening_width
uint8 progress_percent
```

#### Low-Level Actions

**MoveJointGroup.action**

```
# Goal
string joint_group  # "navigation", "picker", "gripper", "container"
float64[] target_positions
float32 max_velocity
string motion_profile  # "trapezoidal", "s_curve"
---
# Result
bool success
float64[] final_positions
float64[] position_error
duration execution_time
string message
---
# Feedback
float64[] current_positions
float64[] velocity
uint8 progress_percent
```

**ExecuteTrajectory.action**

```
# Goal
trajectory_msgs/JointTrajectory trajectory
---
# Result
bool success
float32 trajectory_error
float64[] final_positions
string message
---
# Feedback
uint32 current_point_index
duration time_elapsed
uint8 progress_percent
```

### Service Definitions

**GetManipulatorState.srv**

```
# Request
# (empty)
---
# Response
ManipulatorState state
```

**ValidateAddress.srv**

```
# Request
WarehouseAddress address
---
# Response
bool is_valid
string message
```

---

## Deployment Architecture

### Launch File Organization

#### Main System Launch

```python
# launch/control_system.launch.py

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    return LaunchDescription([

        # State management
        Node(
            package='manipulator_control',
            executable='manipulator_state_tracker',
            name='state_tracker',
            output='screen'
        ),

        Node(
            package='manipulator_control',
            executable='warehouse_state_manager',
            name='warehouse_state',
            output='screen',
            parameters=[{'warehouse_config': 'config/warehouse_addresses.yaml'}]
        ),

        Node(
            package='manipulator_control',
            executable='safety_monitor',
            name='safety_monitor',
            output='screen'
        ),

        # Low-level action servers
        Node(
            package='manipulator_control',
            executable='move_joint_group_server',
            name='move_joint_group_server',
            output='screen',
            parameters=[{'config_file': 'config/action_servers.yaml'}]
        ),

        Node(
            package='manipulator_control',
            executable='execute_trajectory_server',
            name='execute_trajectory_server',
            output='screen'
        ),

        # Mid-level action servers
        Node(
            package='manipulator_control',
            executable='navigate_to_address_server',
            name='navigate_to_address_server',
            output='screen'
        ),

        Node(
            package='manipulator_control',
            executable='extract_box_server',
            name='extract_box_server',
            output='screen'
        ),

        Node(
            package='manipulator_control',
            executable='pick_item_server',
            name='pick_item_server',
            output='screen'
        ),

        Node(
            package='manipulator_control',
            executable='manipulate_container_server',
            name='manipulate_container_server',
            output='screen'
        ),

        # High-level action servers
        Node(
            package='manipulator_control',
            executable='pick_item_from_storage_server',
            name='pick_item_from_storage_server',
            output='screen'
        ),

        Node(
            package='manipulator_control',
            executable='store_item_in_container_server',
            name='store_item_in_container_server',
            output='screen'
        ),

        Node(
            package='manipulator_control',
            executable='return_box_server',
            name='return_box_server',
            output='screen'
        ),

    ])
```

#### Complete Simulation Launch

```python
# launch/simulation.launch.py

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    return LaunchDescription([

        # Launch Gazebo with warehouse world
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                FindPackageShare('manipulator_description'),
                '/launch/gazebo.launch.py'
            ])
        ),

        # Launch controllers
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                FindPackageShare('manipulator_description'),
                '/launch/controllers.launch.py'
            ])
        ),

        # Launch control system
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                FindPackageShare('manipulator_control'),
                '/launch/control_system.launch.py'
            ])
        ),

        # Launch GUI
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                FindPackageShare('manipulator_control'),
                '/launch/gui_commander.launch.py'
            ])
        ),

        # Launch RViz
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', FindPackageShare('manipulator_control') + '/rviz/simulation.rviz']
        ),

    ])
```

### Development vs Production Modes

**Development (Simulation):**
- Run `simulation.launch.py`
- Includes GUI, RViz, Gazebo
- Verbose logging (DEBUG level)
- All safety checks enabled but non-critical

**Production (Real Hardware):**
- Run `production.launch.py`
- No GUI (use Level 2 bridge instead)
- INFO level logging
- Strict safety monitoring
- E-stop integration
- Hardware-specific parameters

```python
# launch/production.launch.py (example difference)

Node(
    package='manipulator_control',
    executable='safety_monitor',
    name='safety_monitor',
    output='screen',
    parameters=[{
        'strict_mode': True,
        'emergency_stop_gpio_pin': 17,
        'enable_collision_detection': True
    }]
),
```

---

## Security Considerations

### Access Control

**Level 2 Bridge Authentication:**
- API key authentication for REST bridge
- TLS encryption for MQTT bridge
- IP whitelisting for trusted clients

**ROS2 Security (Optional Future):**
- ROS2 DDS security plugins (SROS2)
- Certificate-based authentication
- Topic/action access control lists

### Safety Systems

**Emergency Stop Integration:**
- Hardware E-stop button → GPIO → Safety Monitor
- Software E-stop command via ROS2 service
- All actions preempted immediately on E-stop
- Manual reset required to resume

**Collision Avoidance:**
- Simple bounding box checks in planning utilities
- Joint limit monitoring (soft limits)
- Velocity limiting near obstacles

**Fail-Safe Behaviors:**
- On node crash: Controllers hold position
- On communication loss: Timeout → safe state
- On invalid command: Reject with error, don't execute

---

## Performance Considerations

### Real-Time Requirements

**Control Loop Frequencies:**
- Joint state publishing: 50-100 Hz (from ros2_control)
- State tracker updates: 10 Hz
- Action feedback: 1-10 Hz (depends on action)
- GUI updates: 10 Hz

**Latency Targets:**
- Action goal acceptance: < 100 ms
- Emergency stop response: < 50 ms
- State query response: < 10 ms

### Resource Usage

**Expected Resource Consumption:**
- CPU: ~30-40% of single core (simulation)
- Memory: ~500 MB (all nodes combined)
- Network: < 10 Mbps (ROS2 DDS traffic)

**Optimization Opportunities:**
- Use composable nodes to reduce overhead
- Batch state updates where appropriate
- Tune DDS QoS for reliability vs. latency tradeoff

### Scalability

**Current Architecture Supports:**
- Single manipulator
- 10-20 concurrent action requests (queued)
- 100s of warehouse addresses

**Future Scalability:**
- Multiple manipulators: Namespace separation
- Distributed control: ROS2 multi-machine setup
- Load balancing: Multiple action servers per type

---

## Development Roadmap

### Phase 1: Foundation (Current Focus)

**Deliverables:**
1. ✅ Package structure created (`manipulator_control`)
2. ✅ Action definitions written and compiled
3. ✅ Low-level action servers implemented
   - `MoveJointGroup`
   - `ExecuteTrajectory`
4. State management nodes implemented
5. Basic testing in Gazebo

**Success Criteria:**
- Can send joint group commands via actions
- State tracker publishes current manipulator state
- All nodes launch without errors

### Phase 2: Mid-Level Actions

**Deliverables:**
1. `NavigateToAddress` server implemented
2. Warehouse address mapping working
3. `ExtractBox` server with trajectory planning
4. `PickItem` server implementation
5. `ManipulateContainer` server
6. Integration tests in Gazebo

**Success Criteria:**
- Can navigate to any valid warehouse address
- Can extract box from cabinet (simulated magnet)
- Can pick item from box (simulated gripper)
- Container jaws operate synchronously

### Phase 3: High-Level Actions & GUI

**Deliverables:**
1. `PickItemFromStorage` server (composition)
2. GUI command panel implemented
3. Full end-to-end testing
4. Error recovery tested
5. Documentation complete

**Success Criteria:**
- Can execute complete pick operation from GUI
- Errors handled gracefully with recovery
- All action feedback displays in GUI
- System runs reliably for 10+ consecutive operations

### Phase 4: Level 2 Bridge

**Deliverables:**
1. REST API bridge implemented
2. API contract validated with Level 2 team
3. MQTT bridge (optional alternative)
4. Integration testing with real Level 2 system

**Success Criteria:**
- Level 2 can issue commands via REST API
- Status updates reach Level 2 in real-time
- System handles command queue from Level 2

### Phase 5: Hardware Integration (Future)

**Deliverables:**
1. Real hardware interface plugin
2. Sensor integration (encoders, limit switches)
3. Magnet control via GPIO
4. Gripper control integration
5. Safety system integration (E-stop, sensors)

**Success Criteria:**
- Same action servers work on real hardware
- No code changes in `manipulator_control` package
- Hardware parameters in config files only

---

## Validation & Testing

### Validation Checklist

**Architecture Completeness:**
- [x] All 9 DOF joints addressed
- [x] Four kinematic chains defined
- [x] Action hierarchy clear (high/mid/low levels)
- [x] State management architecture complete
- [x] GUI design specified
- [x] Level 2 bridge contract defined
- [x] Package structure defined
- [x] Sim/Real separation maintained

**ROS2 Best Practices:**
- [x] Uses ros2_control for hardware abstraction
- [x] Actions for long-running tasks
- [x] Services for queries
- [x] Topics for state publishing
- [x] Launch files for deployment
- [x] Config files for parameters
- [x] Composable node-ready architecture

**Operational Requirements:**
- [x] Can navigate to any cabinet address
- [x] Can extract boxes from cabinets
- [x] Can pick items from boxes
- [x] Can manipulate container jaws
- [x] Complete workflow testable in simulation
- [x] GUI provides testing interface
- [x] Error handling specified
- [x] Safety monitoring included

### Testing Strategy Summary

**Unit Tests:**
- Action server logic (goal processing, feedback generation)
- Trajectory generation functions
- Coordinate transformation utilities
- State management updates

**Integration Tests:**
- Launch full system in Gazebo
- Execute actions via test clients
- Verify robot motion and state updates
- Test error conditions and recovery

**System Tests:**
- End-to-end warehouse operations
- Complete pick-and-place cycles
- Multi-action sequences
- Stress testing (repeated operations)

**Acceptance Criteria:**
- 90%+ action success rate in simulation
- < 1cm positioning error for navigation
- Successful box extraction 95%+ of attempts
- All actions provide meaningful feedback
- GUI responsive and error-free

---

## References & Resources

### ROS2 Control Documentation
- [ros2_control hardware interface types](https://control.ros.org/rolling/doc/ros2_control/hardware_interface/doc/hardware_interface_types_userdoc.html)
- [Hardware Components documentation](https://control.ros.org/rolling/doc/ros2_control/hardware_interface/doc/hardware_components_userdoc.html)
- [ros2_control demos GitHub](https://github.com/ros-controls/ros2_control_demos)
- [ThinkRobotics ROS2 Control Guide](https://thinkrobotics.com/blogs/learn/ros-2-control-for-custom-robots-mastering-real-time-robot-control)

### ROS2 Actions
- [Creating ROS 2 Actions - Foxglove](https://foxglove.dev/blog/creating-ros2-actions)
- [ROS2 Action Server/Client Tutorials](https://docs.ros.org/en/humble/Tutorials/Intermediate/Writing-an-Action-Server-Client.html)

### MoveIt2 Integration (Optional Future)
- [MoveIt Low Level Controllers](https://moveit.picknik.ai/humble/doc/examples/controller_configuration/controller_configuration_tutorial.html)
- [Tutorial on mobile manipulation using ros2_control and MoveIt 2](https://discourse.openrobotics.org/t/tutorial-on-mobile-manipulation-using-ros2-control-and-moveit-2/32716)
- [MoveIt Servo for realtime control](https://moveit.ai/moveit/ros2/servo/jog/2020/09/09/moveit2-servo.html)

### GUI Development
- [Creating RQT Plugins in ROS2 - Stack Overflow](https://stackoverflow.com/questions/77934193/how-can-i-create-a-simple-gui-using-an-rqt-plugin)
- [RQt Documentation - ROS2 Humble](https://docs.ros.org/en/humble/Concepts/Intermediate/About-RQt.html)
- [RQt Custom Plugin Tutorial - GitHub](https://github.com/ChoKasem/rqt_tut)
- [Building ROS2 RQt Plugin Guide](https://github.com/flobotics/custom_ros2_documentation/blob/main/Building-ROS2-Rqt-Plugin.md)

### Warehouse Robotics Patterns
- [Amazon Warehouse Robotics - Robotics Academy](https://jderobot.github.io/RoboticsAcademy/exercises/MobileRobots/single_robot_amazon_warehouse/)
- [CompROS: Composable ROS2 Architecture](https://www.researchgate.net/publication/354956131_CompROS_A_composable_ROS2_based_architecture_for_real-time_embedded_robotic_development)

### Project Context
- `README.md` - System overview and level definitions
- `ros2_ws/src/manipulator_description/urdf/manipulator/ros2_control.xacro` - Existing hardware interface
- `ros2_ws/src/joy_control/` - Joystick control reference implementation

---

## Glossary

**Level 1:** Accounting System (external, not in scope)
**Level 2:** Storage Optimization System (issues high-level tasks)
**Level 3:** ROS2 Control System (this architecture - action-based control)
**Level 4:** Hardware Interface (ros2_control, motors, sensors)

**Box:** Storage unit in cabinet shelves containing items
**Container:** Large vessel hanging on container jaws, receives picked items
**Item:** Individual object being manipulated (picked from box, placed in container)

**DOF:** Degrees of Freedom (9 for this manipulator)
**Kinematic Chain:** Group of joints that move together for a specific function

**Action:** ROS2 long-running task with goal, feedback, and result
**Action Server:** Node that executes actions
**Action Client:** Node that sends goals to action servers

**ros2_control:** ROS2 framework for hardware abstraction
**JointTrajectoryController:** Standard controller for joint motion
**Gazebo/Ignition:** Physics simulation environment

**GUI Command Node:** Testing interface replacing Level 2 bridge
**Bridge Node:** Future interface between Level 2 and ROS2 actions

---

## Appendices

### Appendix A: Joint Group Configuration

```yaml
# config/kinematic_chains.yaml

joint_groups:

  navigation:
    description: "Railway + Selector for positioning at cabinet addresses"
    joints:
      - base_main_frame_joint
      - main_frame_selector_frame_joint
    default_velocity: 0.5  # m/s
    default_acceleration: 0.2  # m/s²

  gripper:
    description: "Magnetic gripper for box extraction"
    joints:
      - selector_frame_gripper_joint
    default_velocity: 0.1  # m/s (slow for safety)
    default_acceleration: 0.05  # m/s²

  picker:
    description: "Picker mechanism for item manipulation"
    joints:
      - selector_frame_picker_frame_joint
      - picker_frame_picker_rail_joint
      - picker_rail_picker_base_joint
      - picker_base_picker_jaw_joint
    default_velocity: 0.08  # m/s (precision motion)
    default_acceleration: 0.04  # m/s²

  container:
    description: "Container jaws (synchronized)"
    joints:
      - selector_left_container_jaw_joint
      - selector_right_container_jaw_joint
    mimic_behavior: true  # Software synchronization
    default_velocity: 0.05  # m/s
    default_acceleration: 0.02  # m/s²
```

### Appendix B: Example Warehouse Configuration

```yaml
# config/warehouse_addresses.yaml

warehouse_layout:
  description: "Test warehouse for development"
  rows: 5  # 0-4
  columns: 6  # 0-5
  shelves_per_column: 4  # 0-3

  # Physical dimensions
  row_spacing: 0.8  # meters between row centers
  column_spacing: 0.4  # meters between column centers (unused for single-axis)
  shelf_spacing: 0.35  # meters between shelf centers
  base_height: 0.15  # meters from ground to first shelf

  # Safety margins
  cabinet_face_offset: 0.1  # meters from cabinet face when navigating

# Sample box locations (for testing)
boxes:
  - box_id: "BOX_001"
    address: {row: 0, column: 2, shelf: 1}
    items:
      - item_id: "ITEM_1001"
        position: {x: 0.05, y: 0.03, z: 0.02}
        grasp_type: "top"
      - item_id: "ITEM_1002"
        position: {x: 0.08, y: 0.06, z: 0.02}
        grasp_type: "top"

  - box_id: "BOX_002"
    address: {row: 1, column: 3, shelf: 2}
    items:
      - item_id: "ITEM_2001"
        position: {x: 0.04, y: 0.04, z: 0.03}
        grasp_type: "top"
```

### Appendix C: Action Server Base Class Template

```python
# manipulator_control/utils/action_server_base.py

from rclpy.node import Node
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
import yaml

class ActionServerBase(Node):
    """Base class for all manipulator action servers"""

    def __init__(self, node_name, action_type, action_name, execute_callback):
        super().__init__(node_name)

        # Load configuration
        self.declare_parameter('config_file', 'config/action_servers.yaml')
        config_file = self.get_parameter('config_file').value
        self.load_config(config_file, action_name)

        # Create action server
        self._action_server = ActionServer(
            self,
            action_type,
            action_name,
            execute_callback=execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=ReentrantCallbackGroup()
        )

        self.get_logger().info(f'{node_name} initialized')

    def load_config(self, config_file, action_name):
        """Load configuration from YAML"""
        try:
            with open(config_file, 'r') as f:
                config = yaml.safe_load(f)
                self.config = config['action_servers'].get(action_name, {})
        except Exception as e:
            self.get_logger().warn(f'Failed to load config: {e}')
            self.config = {}

    def goal_callback(self, goal_request):
        """Handle goal request"""
        self.get_logger().info('Received goal request')
        # Can add validation here
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        """Handle cancellation request"""
        self.get_logger().info('Received cancel request')
        return CancelResponse.ACCEPT

    def publish_feedback(self, goal_handle, feedback_msg):
        """Helper to publish feedback"""
        goal_handle.publish_feedback(feedback_msg)
```

---

**End of Architecture Document**

This architecture document provides a complete blueprint for implementing the Level 3 ROS2 control system for the ya_robot_manipulator. All sections are ready for implementation, with clear separation between simulation and future hardware integration.
