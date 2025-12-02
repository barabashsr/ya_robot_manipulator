# ROS2 Control Architecture v2.0 - CORRECTED
## ya_robot_manipulator Level 3 Control System

**Document Version:** 2.3
**Date:** 2025-11-27
**Corrections Applied:** Based on URDF analysis, detailed user feedback, unified limits architecture, JointTrajectoryController migration, and Epic 7 Hardware Interface architecture

---

## CRITICAL CORRECTIONS FROM V1.0

### 1. ✅ Controller Architecture (Hybrid: Trajectory + Forward Controllers)

**UPDATED 2025-11-26:** The system uses a **hybrid controller architecture**:
- **7 JointTrajectoryControllers** for motion joints (smooth trajectory interpolation, action-based)
- **2 ForwardCommandControllers** for container jaw joints (simple position commands, topic-based)

**From:** `/ros2_ws/src/manipulator_description/config/manipulator_controllers.yaml`

**Trajectory Controllers (7 motion joints):**
| Controller | Joint | Interface | Benefits |
|------------|-------|-----------|----------|
| `base_main_frame_joint_controller` | base_main_frame_joint | Action: `follow_joint_trajectory` | Smooth interpolated X-axis motion |
| `main_frame_selector_frame_joint_controller` | main_frame_selector_frame_joint | Action: `follow_joint_trajectory` | Smooth interpolated Z-axis motion |
| `selector_frame_gripper_joint_controller` | selector_frame_gripper_joint | Action: `follow_joint_trajectory` | Smooth interpolated Y-axis motion |
| `selector_frame_picker_frame_joint_controller` | selector_frame_picker_frame_joint | Action: `follow_joint_trajectory` | Smooth picker Z-axis motion |
| `picker_frame_picker_rail_joint_controller` | picker_frame_picker_rail_joint | Action: `follow_joint_trajectory` | Smooth picker Y-axis motion |
| `picker_rail_picker_base_joint_controller` | picker_rail_picker_base_joint | Action: `follow_joint_trajectory` | Smooth picker extension |
| `picker_base_picker_jaw_joint_controller` | picker_base_picker_jaw_joint | Action: `follow_joint_trajectory` | Smooth jaw motion |

**Forward Controllers (2 container jaws - simple open/close):**
| Controller | Joint | Interface | Reason |
|------------|-------|-----------|--------|
| `selector_left_container_jaw_joint_controller` | selector_left_container_jaw_joint | Topic: `/commands` | Simple gripper, no trajectory needed |
| `selector_right_container_jaw_joint_controller` | selector_right_container_jaw_joint | Topic: `/commands` | Simple gripper, mimic joint |

**Implementation Strategy:**
- **Trajectory joints:** Use `FollowJointTrajectory` action for smooth interpolated motion with feedback
- **Container jaws:** Continue using topic-based position commands (Float64MultiArray)
- **ControllerInterface:** Unified API abstracts controller type - callers use same `command_joint()` method
- **MoveIt2 ready:** Trajectory interface is MoveIt's native control method for future integration

**Benefits of Trajectory Controllers:**
| Benefit | Description |
|---------|-------------|
| Coordinated multi-joint motion | Action interface enables synchronized trajectories |
| Smooth interpolated motion | Spline interpolation eliminates jerky position jumps |
| MoveIt2 integration | `FollowJointTrajectory` is MoveIt's native interface |
| Velocity/acceleration limits | Controller-level enforcement of motion constraints |
| Goal monitoring | Built-in goal tolerance and timeout handling |
| Feedback during motion | Progress updates via action feedback |

**Interface Comparison:**
| Aspect | ForwardCommand (jaws) | Trajectory (motion joints) |
|--------|----------------------|---------------------------|
| Interface Type | Topic | Action |
| Topic/Action Name | `/{joint}_controller/commands` | `/{joint}_controller/follow_joint_trajectory` |
| Message Type | `std_msgs/Float64MultiArray` | `control_msgs/action/FollowJointTrajectory` |
| Motion Profile | Instant jump | Interpolated trajectory |
| Feedback | None | Progress feedback during execution |
| Result | None | Success/failure with error codes |

### 2. ✅ Warehouse Addressing System (CORRECTED)

**INCORRECT V1.0 Understanding:** Thought addressing was (row, column, shelf)

**CORRECT Addressing from URDF:**

```
Complete Item Address Format: (side, cabinet_num, row, column, department)

Where:
- side: "left" or "right" (two cabinet rows)
- cabinet_num: Cabinet number in that row (1, 2, 3, 4...)
- row: Vertical position in cabinet (1=top, increasing downward)
- column: Horizontal position (1=leftmost from manipulator view)
- department: Compartment in extracted box (1 to N departments)

Box Address = (side, cabinet_num, row, column)
Item Address = Box Address + department

Example Addresses:
- Box Address: addr_l_1_2_3 = Left row, Cabinet 1, Row 2, Column 3
- Item Address: addr_l_1_2_3_dept_5 = Same box, Department 5
- Box Address: addr_r_4_1_5 = Right row, Cabinet 4, Row 1, Column 5
- Item Address: addr_r_4_1_5_dept_2 = Same box, Department 2

IMPORTANT: Departments only exist after box is extracted and spawned!
```

**URDF Link Names for Addresses:**
```
Link naming pattern: addr_{side_abbrev}_{cabinet}_{row}_{col}

Examples from URDF:
- addr_l_1_1_1  → Left, Cabinet 1, Row 1, Column 1
- addr_l_1_1_2  → Left, Cabinet 1, Row 1, Column 2
- addr_r_2_3_4  → Right, Cabinet 2, Row 3, Column 4
```

**Cabinet Configurations from `storage_params.yaml`:**

**Left Row:**
1. Cabinet 1: 4x10x10 (4 columns, 10 rows, 10 departments)
2. Cabinet 2: 4x10x10
3. Cabinet 3: 4x6x10
4. Cabinet 4: 5x12x14

**Right Row:**
1. Cabinet 1: 5x12x14
2. Cabinet 2: 5x8x14
3. Cabinet 3: 6x14x16
4. Cabinet 4: 4x10x10

**Address Resolution Method:**
```python
# NO need to store addresses in memory!
# They already exist as TF frames in URDF

def get_address_coordinates(side, cabinet, row, column):
    """
    Get address coordinates by looking up TF frame

    Args:
        side: "left" or "right"
        cabinet: int (1-4)
        row: int (1 to max_rows for cabinet)
        column: int (1 to max_columns for cabinet)

    Returns:
        (x, y, z) coordinates in world frame
    """
    side_abbrev = 'l' if side == 'left' else 'r'
    frame_name = f"addr_{side_abbrev}_{cabinet}_{row}_{column}"

    # Use TF2 to lookup transform
    transform = tf_buffer.lookup_transform(
        'world',  # or 'base_link'
        frame_name,
        rclpy.time.Time()
    )

    return (
        transform.transform.translation.x,
        transform.transform.translation.y,
        transform.transform.translation.z
    )
```

**Services for Address Operations:**

```python
# Service: GetAddressCoordinates.srv
# Request:
string side          # "left" or "right"
uint8 cabinet_num
uint8 row
uint8 column
---
# Response:
bool success
geometry_msgs/Pose pose
string error_message
```

### 3. ✅ Picker State Machine (Limit Switch Based)

**CORRECTED:** Picker uses **limit switches (end stops)** for position detection, NOT precision position control.

**Hardware Reality:**
- Steppers with **NO encoders**
- Position is approximated
- **ALL axes have TWO limit switches** (min and max position)
- End switches detect: jaw closed, jaw opened, picker extended, picker retracted

**Picker Operation as State Machine (CORRECTED):**

The picker does NOT extend into the box - it LOWERS into it! The extension mechanism is used for positioning the item above the container for release.

```
States for PickItem Action (CORRECTED):

1. POSITION_Y
   - Move picker rail to target department position (Y-axis)
   - Joint: picker_frame_picker_rail_joint
   - Monitor: picker_rail_min/max for safety
   - Transition: When Y position reached

2. OPEN_JAW (if not already open)
   - Command picker jaw to open
   - Joint: picker_base_picker_jaw_joint
   - Monitor end switch: picker_jaw_opened
   - Transition: When picker_jaw_opened == TRUE

3. LOWER_PICKER
   - Lower picker into box (Z-axis negative)
   - Joint: selector_frame_picker_frame_joint
   - Monitor end switch: picker_frame_min
   - Transition: When picker_frame_min == TRUE or position reached

4. CLOSE_JAW
   - Command picker jaw to close on item
   - Joint: picker_base_picker_jaw_joint
   - Monitor end switch: picker_jaw_closed
   - Transition: When picker_jaw_closed == TRUE (item grasped)

5. LIFT_PICKER
   - Lift picker out of box (Z-axis positive)
   - Joint: selector_frame_picker_frame_joint
   - Monitor end switch: picker_frame_max or position
   - Transition: When clear of box

6. EXTEND_PICKER
   - Extend picker over container (X-axis positive)
   - Joint: picker_rail_picker_base_joint
   - Monitor end switch: picker_extended
   - Transition: When picker_extended == TRUE

7. RELEASE_ITEM
   - Open jaw to drop item into container
   - Joint: picker_base_picker_jaw_joint
   - Monitor end switch: picker_jaw_opened
   - Transition: When picker_jaw_opened == TRUE

8. RETRACT_PICKER
   - Retract picker to home position (X-axis negative)
   - Joint: picker_rail_picker_base_joint
   - Monitor end switch: picker_retracted
   - Transition: When picker_retracted == TRUE

States: IDLE → POSITION_Y → OPEN_JAW → LOWER_PICKER → CLOSE_JAW → LIFT_PICKER → EXTEND_PICKER → RELEASE_ITEM → RETRACT_PICKER → SUCCESS
```

**End Switch Topics (CORRECTED with real switch names from limit_switches.yaml):**

```
# Picker jaw switches (grasp control)
/manipulator/end_switches/picker_jaw_opened          (bool) - X=0.19
/manipulator/end_switches/picker_jaw_closed          (bool) - X=0.01

# Picker extension switches (X-axis: picker_rail_picker_base_joint)
/manipulator/end_switches/picker_retracted           (bool) - X=0.01
/manipulator/end_switches/picker_extended            (bool) - X=0.24

# Picker frame switches (Z-axis: selector_frame_picker_frame_joint)
/manipulator/end_switches/picker_frame_min           (bool) - Z=0.005
/manipulator/end_switches/picker_frame_max           (bool) - Z=0.295

# Picker rail switches (Y-axis: picker_frame_picker_rail_joint)
/manipulator/end_switches/picker_rail_min            (bool) - Y=-0.29
/manipulator/end_switches/picker_rail_max            (bool) - Y=+0.29

# Gripper switches (Y-axis: selector_frame_gripper_joint) - LEFT/RIGHT not extend/retract!
/manipulator/end_switches/gripper_left               (bool) - Y=+0.39 (toward left cabinets)
/manipulator/end_switches/gripper_right              (bool) - Y=-0.39 (toward right cabinets)

# Selector frame switches (Z-axis: main_frame_selector_frame_joint)
/manipulator/end_switches/selector_frame_min         (bool) - Z=0.005
/manipulator/end_switches/selector_frame_max         (bool) - Z=1.90

# Base main frame switches (X-axis: base_main_frame_joint)
/manipulator/end_switches/base_main_frame_min        (bool) - X=0.01
/manipulator/end_switches/base_main_frame_max        (bool) - X=3.9

# Container jaw switches (Y-axis)
/manipulator/end_switches/container_left_min         (bool)
/manipulator/end_switches/container_left_max         (bool)
/manipulator/end_switches/container_right_min        (bool)
/manipulator/end_switches/container_right_max        (bool)
```

**Implementation Note:** State transitions driven by switch events, not position thresholds

---

## Core Physical Workflows Reference

This section provides the definitive reference for all manipulator workflows, describing the exact joints and axes involved in each operation. All action implementations MUST follow these workflows.

**Controller Interface Note (Updated 2025-11-26):**

All workflows now use `JointTrajectoryController` for motion joints, enabling:
- Smooth trajectory interpolation between positions
- Action-based interface with feedback and result
- Coordinated multi-joint motion via trajectory composition

Container jaw operations continue to use `ForwardCommandController` (topic-based).

**Example: Commanding Joints via ControllerInterface**

```python
# ControllerInterface abstracts controller type - same API for all joints
controller_interface = ControllerInterface(node)

# Wait for trajectory action servers at startup
controller_interface.wait_for_action_servers(timeout_sec=30.0)

# Command trajectory joint (auto-calculates duration based on velocity limits)
controller_interface.command_joint('base_main_frame_joint', 2.0)

# Command trajectory joint with explicit duration
controller_interface.command_joint('base_main_frame_joint', 2.0, duration_sec=5.0)

# Command forward joint (container jaw) - same API, routed internally
controller_interface.command_joint('selector_left_container_jaw_joint', 0.1)

# Async trajectory with callbacks
def on_result(success, error_msg):
    if success:
        print("Goal reached!")
    else:
        print(f"Failed: {error_msg}")

controller_interface.command_trajectory_with_callback(
    'base_main_frame_joint', 2.0, 3.0,
    result_callback=on_result
)
```

### Physical Layout Summary

**Coordinate System (World Frame):**
- **X-axis:** Along warehouse rail (base_main_frame_joint) - cabinet columns
- **Y-axis:** Into/out of cabinets (gripper, picker rail) - cabinet depth
- **Z-axis:** Vertical (selector_frame, picker_frame) - cabinet rows

**Joint-to-Axis Mapping:**

| Joint | Axis | Range | Purpose |
|-------|------|-------|---------|
| `base_main_frame_joint` | **X** | 0.0→4.0m | Railway carriage along warehouse |
| `main_frame_selector_frame_joint` | **Z** | -0.01→1.5m | Selector vertical lift |
| `selector_frame_gripper_joint` | **Y** | -0.4→0.4m | Gripper into LEFT(+Y) or RIGHT(-Y) cabinets |
| `selector_left_container_jaw_joint` | **-Y** | -0.2→0.2m | Left container jaw |
| `selector_right_container_jaw_joint` | **+Y** | -0.2→0.2m | Right container jaw (mirrored) |
| `selector_frame_picker_frame_joint` | **Z** | -0.01→0.3m | Picker vertical position |
| `picker_frame_picker_rail_joint` | **Y** | -0.3→0.3m | Picker Y-rail (along box departments) |
| `picker_rail_picker_base_joint` | **X** | 0.0→0.25m | Picker X-extension (over container) |
| `picker_base_picker_jaw_joint` | **X** | 0.0→0.2m | Picker jaw grasp |

---

### Workflow 1: Navigation to Address (NavigateToAddress)

**Purpose:** Position the manipulator in front of a cabinet address.

**Joints Used:**
- `base_main_frame_joint` (X-axis) - Move along rail to cabinet position
- `main_frame_selector_frame_joint` (Z-axis) - Move vertically to row height

**Sequence:**

| Step | Joint | Axis | Movement | Switch Feedback |
|------|-------|------|----------|-----------------|
| 1. Move to cabinet X position | `base_main_frame_joint` | X | Target X from TF frame | `base_main_frame_min/max` (safety) |
| 2. Move to row Z height | `main_frame_selector_frame_joint` | Z | Target Z from TF frame | `selector_frame_min/max` (safety) |

**Notes:**
- X and Z can move simultaneously for efficiency
- No Y movement during navigation - gripper stays centered
- Position accuracy: ±0.02m

---

### Workflow 2: Box Extraction from RIGHT Cabinets (ExtractBox - Right Side)

**Purpose:** Extract a box from the RIGHT cabinet row using the electromagnet.

**Joints Used:**
- `selector_frame_gripper_joint` (Y-axis) - Gripper moves RIGHT then LEFT
- `main_frame_selector_frame_joint` (Z-axis) - Optional: trajectory sync for clearance

**Gripper has TWO magnets:** Left magnet for left cabinets, Right magnet for right cabinets.

**Sequence:**

| Step | Joint(s) | Axis | Movement | Switch Feedback |
|------|----------|------|----------|-----------------|
| 1. Move gripper RIGHT toward cabinet | `selector_frame_gripper_joint` | Y- | Negative Y toward right cabinet | `gripper_right` triggers at Y=-0.39 |
| 2. Engage RIGHT electromagnet | - | - | Activate right magnet | - |
| 3. Wait for magnetic attachment | - | - | ~0.5s delay | - |
| 4. Extract LEFT (pull box out) | `selector_frame_gripper_joint` | Y+ | Positive Y, pull box toward center | Crosses center, approaches `gripper_left` |
| (Optional: Z trajectory sync) | `+ main_frame_selector_frame_joint` | Z | Small Z adjustment to clear cabinet frame | - |
| 5. Stop at extraction position | `selector_frame_gripper_joint` | Y+ | Stop near center or toward left | - |

**Final State:** Box attached to gripper, positioned near center (Y≈0) or toward left side.

---

### Workflow 3: Box Extraction from LEFT Cabinets (ExtractBox - Left Side)

**Purpose:** Extract a box from the LEFT cabinet row.

**Sequence (Mirror of Right Side):**

| Step | Joint(s) | Axis | Movement | Switch Feedback |
|------|----------|------|----------|-----------------|
| 1. Move gripper LEFT toward cabinet | `selector_frame_gripper_joint` | Y+ | Positive Y toward left cabinet | `gripper_left` triggers at Y=+0.39 |
| 2. Engage LEFT electromagnet | - | - | Activate left magnet | - |
| 3. Wait for magnetic attachment | - | - | ~0.5s delay | - |
| 4. Extract RIGHT (pull box out) | `selector_frame_gripper_joint` | Y- | Negative Y, pull box toward center | Crosses center, approaches `gripper_right` |
| 5. Stop at extraction position | `selector_frame_gripper_joint` | Y- | Stop near center or toward right | - |

---

### Workflow 4: Box Return (ReturnBox)

**Purpose:** Return an extracted box to its original cabinet address.

**Sequence (for RIGHT cabinet - mirror for LEFT):**

| Step | Joint(s) | Axis | Movement | Switch Feedback |
|------|----------|------|----------|-----------------|
| 1. Navigate to original address | `base_main_frame_joint`, `main_frame_selector_frame_joint` | X, Z | Position in front of cabinet | - |
| 2. Move gripper toward cabinet | `selector_frame_gripper_joint` | Y- | Insert box back into right cabinet | `gripper_right` approaches |
| (Optional: Z trajectory sync) | `+ main_frame_selector_frame_joint` | Z | Adjust for cabinet frame clearance | - |
| 3. Deactivate electromagnet | - | - | Release box | - |
| 4. Retract gripper | `selector_frame_gripper_joint` | Y+ | Pull gripper back to center | - |

---

### Workflow 5: Item Picking (PickItem) - CORRECTED

**Purpose:** Pick an item from a specific department within an extracted box.

**Physical Understanding:**
- Box is extracted and held by gripper
- Departments are aligned along the Y-axis (perpendicular to XZ plane)
- Picker LOWERS into the box (Z-axis), does NOT extend into it
- Picker extension (X-axis) is used to position item OVER the container for release

**Joints Used:**
- `picker_frame_picker_rail_joint` (Y-axis) - Position picker over target department
- `picker_base_picker_jaw_joint` (X-axis) - Open/close jaw
- `selector_frame_picker_frame_joint` (Z-axis) - Lower/lift picker
- `picker_rail_picker_base_joint` (X-axis) - Extend picker over container

**Sequence:**

| Step | State | Joint | Axis | Movement | Switch Feedback |
|------|-------|-------|------|----------|-----------------|
| 1 | POSITION_Y | `picker_frame_picker_rail_joint` | Y | Move picker rail to department Y position | `picker_rail_min/max` (safety) |
| 2 | OPEN_JAW | `picker_base_picker_jaw_joint` | X | Open jaw fully | `picker_jaw_opened` |
| 3 | LOWER_PICKER | `selector_frame_picker_frame_joint` | Z- | Lower picker into box | `picker_frame_min` |
| 4 | CLOSE_JAW | `picker_base_picker_jaw_joint` | X | Close jaw on item | `picker_jaw_closed` |
| 5 | LIFT_PICKER | `selector_frame_picker_frame_joint` | Z+ | Lift item out of box | `picker_frame_max` or position |
| 6 | EXTEND_PICKER | `picker_rail_picker_base_joint` | X+ | Extend picker over container | `picker_extended` |
| 7 | RELEASE_ITEM | `picker_base_picker_jaw_joint` | X | Open jaw to drop item | `picker_jaw_opened` |
| 8 | RETRACT_PICKER | `picker_rail_picker_base_joint` | X- | Retract to home position | `picker_retracted` |

**Key Insight:** The picker starts in a RETRACTED state (X≈0) where its X position already aligns with items in the box. The extension is only used AFTER lifting to position over the container.

---

### Workflow 6: Container Jaw Operation (ManipulateContainer)

**Purpose:** Open/close container jaws to grip or release a collection container.

**Joints Used (synchronized - software mimic in simulation):**
- `selector_left_container_jaw_joint` (-Y axis)
- `selector_right_container_jaw_joint` (+Y axis, mirrored)

**Sequence:**

| Action | Left Jaw Movement | Right Jaw Movement | Result |
|--------|-------------------|--------------------| -------|
| OPEN | Move Y- (away from center) | Move Y+ (away from center) | Jaws spread apart |
| CLOSE | Move Y+ (toward center) | Move Y- (toward center) | Jaws grip container |

**Switch Feedback:** `container_left_min/max`, `container_right_min/max`

---

### Workflow 7: Container Retrieval (GetContainer)

**Purpose:** Retrieve a collection container from a storage location.

**Sequence:**

| Step | Action | Joint(s) |
|------|--------|----------|
| 1 | Open jaws wider than container | Container jaw joints |
| 2 | Navigate to container position | X, Z joints |
| 3 | Lower selector to container height | Z joint |
| 4 | Close jaws to grip container | Container jaw joints |
| 5 | Lift selector to raise container | Z joint |

---

### Switch Name Reference Table

| Real Switch Name | Joint | Position | Semantic Meaning |
|------------------|-------|----------|------------------|
| `base_main_frame_min` | base_main_frame_joint | X=0.01 | Start of rail |
| `base_main_frame_max` | base_main_frame_joint | X=3.9 | End of rail |
| `selector_frame_min` | main_frame_selector_frame_joint | Z=0.005 | Selector at bottom |
| `selector_frame_max` | main_frame_selector_frame_joint | Z=1.90 | Selector at top |
| `gripper_left` | selector_frame_gripper_joint | Y=+0.39 | Gripper extended toward LEFT cabinets |
| `gripper_right` | selector_frame_gripper_joint | Y=-0.39 | Gripper extended toward RIGHT cabinets |
| `picker_frame_min` | selector_frame_picker_frame_joint | Z=0.005 | Picker lowered (in box) |
| `picker_frame_max` | selector_frame_picker_frame_joint | Z=0.295 | Picker raised (out of box) |
| `picker_rail_min` | picker_frame_picker_rail_joint | Y=-0.29 | Picker rail at one end |
| `picker_rail_max` | picker_frame_picker_rail_joint | Y=+0.29 | Picker rail at other end |
| `picker_retracted` | picker_rail_picker_base_joint | X=0.01 | Picker retracted (home) |
| `picker_extended` | picker_rail_picker_base_joint | X=0.24 | Picker extended (over container) |
| `picker_jaw_opened` | picker_base_picker_jaw_joint | X=0.19 | Jaw open |
| `picker_jaw_closed` | picker_base_picker_jaw_joint | X=0.01 | Jaw closed (gripping) |
| `container_left_min` | selector_left_container_jaw_joint | Y=-0.095 | Left jaw at min |
| `container_left_max` | selector_left_container_jaw_joint | Y=+0.095 | Left jaw at max |
| `container_right_min` | selector_right_container_jaw_joint | Y=-0.095 | Right jaw at min |
| `container_right_max` | selector_right_container_jaw_joint | Y=+0.095 | Right jaw at max |

---

### 4. ✅ Simulation Requirements (Limit Switches & Electromagnets)

**NEW REQUIREMENT:** Simulate limit switches and electromagnets in Gazebo

#### A. Limit Switch Simulation

**Approach:** Use Gazebo contact sensors or joint limit detection

**Implementation Options:**

**Option 1: Contact Sensors (Recommended)**

Add contact sensor plugins to picker jaw links:

```xml
<!-- In picker URDF/xacro -->
<gazebo reference="picker_jaw_link">
  <sensor name="jaw_closed_sensor" type="contact">
    <contact>
      <collision>picker_jaw_collision</collision>
    </contact>
    <plugin name="gazebo_ros_bumper_controller" filename="libgazebo_ros_bumper.so">
      <ros>
        <namespace>/manipulator</namespace>
        <remapping>bumper_states:=end_switches/picker_jaw_closed_raw</remapping>
      </ros>
      <frame_name>picker_jaw_link</frame_name>
    </plugin>
    <always_on>true</always_on>
    <update_rate>50</update_rate>
  </sensor>
</gazebo>
```

Then create a node that converts contact sensor output to boolean switch topics.

**Option 2: Virtual Limit Switches (Simpler for start)**

Create a simulation helper node that monitors joint positions and publishes virtual switch states:

```python
class VirtualLimitSwitchNode(Node):
    """Simulates limit switches based on joint positions"""

    def __init__(self):
        super().__init__('virtual_limit_switches')

        # Subscribe to joint states
        self.create_subscription(
            JointState, '/joint_states',
            self.joint_state_callback, 10
        )

        # Publishers for switch states
        self.jaw_closed_pub = self.create_publisher(
            Bool, '/manipulator/end_switches/picker_jaw_closed', 10
        )
        # ... more switches

        # Load switch trigger positions from YAML
        self.load_switch_config()

    def joint_state_callback(self, msg):
        # Find picker jaw joint position
        jaw_idx = msg.name.index('picker_base_picker_jaw_joint')
        jaw_pos = msg.position[jaw_idx]

        # Check if at closed position (max limit)
        if jaw_pos >= self.jaw_closed_threshold:
            self.jaw_closed_pub.publish(Bool(data=True))
        else:
            self.jaw_closed_pub.publish(Bool(data=False))

        # ... similar for other switches
```

**Configuration (CORRECTED switch names - matches limit_switches.yaml):**

```yaml
# config/limit_switches.yaml
limit_switches:
  # Picker jaw (grasp)
  picker_jaw_closed:
    joint: "picker_base_picker_jaw_joint"
    trigger_position: 0.01  # Closed position
    trigger_tolerance: 0.01

  picker_jaw_opened:
    joint: "picker_base_picker_jaw_joint"
    trigger_position: 0.19  # Open position
    trigger_tolerance: 0.01

  # Picker extension (X-axis) - NOT "jaw_extended/retracted"
  picker_extended:
    joint: "picker_rail_picker_base_joint"
    trigger_position: 0.24  # Extended over container
    trigger_tolerance: 0.01

  picker_retracted:
    joint: "picker_rail_picker_base_joint"
    trigger_position: 0.01  # Home position
    trigger_tolerance: 0.01

  # Gripper (Y-axis) - LEFT/RIGHT semantics, NOT extend/retract
  gripper_left:
    joint: "selector_frame_gripper_joint"
    trigger_position: 0.39  # Toward left cabinets (+Y)
    trigger_tolerance: 0.01

  gripper_right:
    joint: "selector_frame_gripper_joint"
    trigger_position: -0.39  # Toward right cabinets (-Y)
    trigger_tolerance: 0.01
```

**References for ROS2 Jazzy & Gazebo Harmonic:**
- [gz_ros2_control Jazzy Documentation](https://control.ros.org/jazzy/doc/gz_ros2_control/doc/index.html) - Official ros2_control integration for Gazebo Harmonic
- [Gazebo Contact Sensors (Classic)](https://classic.gazebosim.org/tutorials?tut=ros_gzplugins) - Legacy reference
- [Gazebo Sim Plugins and Sensors for ROS2 - Medium Tutorial](https://medium.com/@alitekes1/gazebo-sim-plugin-and-sensors-for-acquire-data-from-simulation-environment-681d8e2ad853) - Contact sensor examples for Gazebo Harmonic
- [MOGI-ROS Gazebo Basics](https://github.com/MOGI-ROS/Week-3-4-Gazebo-basics) - Introduction to URDF and Gazebo Harmonic with ROS2 Jazzy

#### B. Electromagnet Simulation

**Requirement:** Activate/deactivate magnet, attach/detach boxes controlled from ROS2

**Approach:** Manual attachment toggle via ROS2 service (simulates turning magnet on/off)

**CORRECTED Implementation Strategy:**

Instead of automatic attachment on contact, we want **explicit control from ROS2**:
1. Action server calls `/manipulator/electromagnet/toggle` service with `activate=true`
2. Simulator checks if box is near gripper
3. If yes and magnet activated → attach box to gripper in Gazebo
4. When `activate=false` → detach box

**Method 1: Gazebo Attach/Detach Services (RECOMMENDED)**

Use Gazebo's built-in attach/detach services, triggered by our ROS2 service:

```xml
<gazebo>
  <plugin name="gazebo_grasp_fix" filename="libgazebo_grasp_fix.so">
    <arm>
      <arm_name>gripper_magnet</arm_name>
      <palm_link>gripper_magnet_link</palm_link>
      <gripper_link>gripper_magnet_link</gripper_link>
    </arm>
    <forces_angle_tolerance>100</forces_angle_tolerance>
    <update_rate>10</update_rate>
    <grip_count_threshold>2</grip_count_threshold>
    <max_grip_count>5</max_grip_count>
    <release_tolerance>0.005</release_tolerance>
    <disable_collisions_on_attach>false</disable_collisions_on_attach>
    <contact_topic>__default_topic__</contact_topic>
  </plugin>
</gazebo>
```

**Method 2: Custom Gazebo Service (More Control)**

Create a ROS2 service to toggle attachment:

```python
# Service definition: ToggleElectromagnet.srv
bool activate  # True = engage, False = release
---
bool success
string message
```

**Service Implementation:**

```python
class ElectromagnetSimulatorNode(Node):
    """Simulates electromagnet attach/detach in Gazebo"""

    def __init__(self):
        super().__init__('electromagnet_simulator')

        # Service for magnet control
        self.create_service(
            ToggleElectromagnet,
            '/manipulator/electromagnet/toggle',
            self.toggle_magnet_callback
        )

        # Gazebo service clients for attaching/detaching
        self.attach_client = self.create_client(
            Attach, '/gazebo/attach'
        )
        self.detach_client = self.create_client(
            Detach, '/gazebo/detach'
        )

        self.magnet_engaged = False

    def toggle_magnet_callback(self, request, response):
        if request.activate:
            # Attach box to gripper
            result = self.attach_box_to_gripper()
            self.magnet_engaged = result
        else:
            # Detach box
            result = self.detach_box()
            self.magnet_engaged = False

        response.success = result
        return response

    def attach_box_to_gripper(self):
        # Find nearest box using contact sensor or proximity
        box_name = self.find_nearest_box()

        # Call Gazebo attach service
        req = Attach.Request()
        req.model_name_1 = 'manipulator'
        req.link_name_1 = 'gripper_magnet_link'
        req.model_name_2 = box_name
        req.link_name_2 = 'box_link'

        future = self.attach_client.call_async(req)
        # ... handle future
        return True
```

**Hardware Control Interface (for real electromagnet):**

When transitioning to hardware, implement hardware interface:

```python
# For real hardware with GPIO control
class ElectromagnetHardwareInterface:
    def __init__(self, gpio_pin):
        import RPi.GPIO as GPIO  # or other GPIO library
        self.pin = gpio_pin
        GPIO.setup(self.pin, GPIO.OUT)

    def engage(self):
        GPIO.output(self.pin, GPIO.HIGH)

    def release(self):
        GPIO.output(self.pin, GPIO.LOW)
```

**References for ROS2 Jazzy & Gazebo Harmonic:**
- [Gazebo Detachable Joint System Documentation](https://gazebosim.org/docs/harmonic/ros2_integration/) - Modern approach for attach/detach in Gazebo Harmonic
- [Clearpath Manipulation in Gazebo Harmonic](https://docs.clearpathrobotics.com/docs/ros/tutorials/manipulation/gazebo/) - Practical manipulation examples
- [Gazebo magnet simulation discussion (legacy)](https://answers.ros.org/question/11606/gazebo-magnet-simulation/)
- [manipulation_worlds grasp_hack plugin (legacy)](https://github.com/topics/gripper)

### 5. ✅ Container Jaw Mimic (Software Mimic for Simulation Only)

**CLARIFICATION:** Container jaws are **mechanically mimicked** in hardware. Software mimic is ONLY for Gazebo simulation (because dartsim doesn't support URDF mimic joints).

**Implementation:**

```python
def control_container_jaws(target_opening_width):
    """
    Control both jaws synchronously

    Args:
        target_opening_width: Total opening in meters (jaw to jaw)
    """
    # Calculate individual jaw positions (symmetric)
    left_target = -target_opening_width / 2.0
    right_target = target_opening_width / 2.0

    # Send commands simultaneously
    left_controller.publish(Float64(data=left_target))
    right_controller.publish(Float64(data=right_target))
```

**For Hardware:** No synchronization code needed - mechanical mimic handles it!

### 6. ✅ Container Retrieval Action

**NEW ACTION:** Get container from storage location

**GetContainer Action Definition:**

```
# GetContainer.action

# Goal
string container_id           # Container identifier
string container_location     # "storage_left_1", "storage_right_2", etc.
float32 container_width      # Width to set jaw opening
---
# Result
bool success
bool container_retrieved
bool jaws_gripping
string message
---
# Feedback
string current_phase  # "opening_jaws", "navigating", "positioning", "closing_jaws", "lifting"
uint8 progress_percent
```

**CORRECTED Implementation Steps:**

1. **Open Jaws:** Set jaw opening wider than container width
2. **Navigate:** Move rails + selector to container position (using address system or predefined positions)
3. **Position Selector:** Lower selector to container height
4. **Close Jaws:** Grip container
5. **Lift Selector:** Raise selector to lift container (container now hangs on jaws)
6. **Verify:** Check jaw positions indicate container is gripped and lifted

**PlaceContainer Action (opposite operation):**

```
# PlaceContainer.action

# Goal
string container_location     # Where to place
---
# Result
bool success
bool container_placed
---
# Feedback
string current_phase  # "navigating", "lowering", "opening_jaws"
uint8 progress_percent
```

**Implementation Steps:**

1. **Navigate:** Move to container placement position
2. **Lower Selector:** Lower container until it rests on support
3. **Open Jaws:** Release container
4. **Raise Selector:** Lift selector away from container

**Predefined Container Positions:**

```yaml
# config/container_storage.yaml
container_storage_locations:
  storage_left_1:
    coordinates:
      x: -0.5
      y: 0.3
      z: 0.8
    max_container_width: 0.35

  storage_right_1:
    coordinates:
      x: 4.5
      y: -0.3
      z: 0.8
    max_container_width: 0.35
```

### 7. ✅ Dynamic Box Spawning & Department Frames

**REQUIREMENT:** Boxes are NOT in URDF. They are spawned dynamically when extracted, despawned when returned.

**Architecture Component: Box Spawner Node**

**Node: `box_spawn_manager_node`**

**Responsibilities:**
1. Spawn box model in Gazebo when ExtractBox action succeeds
2. Generate department frames as TF broadcasts
3. Add visual markers for departments
4. Despawn box when ReturnBox action completes

**Implementation:**

```python
class BoxSpawnManagerNode(Node):
    """Manages dynamic box spawning and department frame generation"""

    def __init__(self):
        super().__init__('box_spawn_manager')

        # Gazebo model spawning service client
        self.spawn_model_client = self.create_client(
            SpawnEntity, '/gazebo/spawn_entity'
        )
        self.delete_model_client = self.create_client(
            DeleteEntity, '/gazebo/delete_entity'
        )

        # TF broadcaster for department frames
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # Marker publisher for department visualization
        self.marker_pub = self.create_publisher(
            MarkerArray, '/visualization_marker_array', 10
        )

        # Services
        self.create_service(
            SpawnBox, '/manipulator/spawn_box',
            self.spawn_box_callback
        )
        self.create_service(
            DespawnBox, '/manipulator/despawn_box',
            self.despawn_box_callback
        )

        # Load box configurations from storage_params.yaml
        self.load_box_configurations()

        self.active_boxes = {}  # Track spawned boxes

    def spawn_box_callback(self, request, response):
        """
        Spawn box at address with departments

        Request:
            string box_id
            string side
            uint8 cabinet_num
            uint8 row
            uint8 column
            uint8 num_departments
        """
        box_id = request.box_id
        address = (request.side, request.cabinet_num,
                   request.row, request.column)

        # Get box parameters from storage_params.yaml
        box_params = self.get_box_parameters(
            request.cabinet_num, request.num_departments
        )

        # Get spawn position from address TF frame
        spawn_pose = self.get_address_pose(address)

        # Generate box SDF/URDF
        box_model = self.generate_box_model(
            box_id, box_params, request.num_departments
        )

        # Spawn in Gazebo
        spawn_req = SpawnEntity.Request()
        spawn_req.name = box_id
        spawn_req.xml = box_model
        spawn_req.initial_pose = spawn_pose

        future = self.spawn_model_client.call_async(spawn_req)
        # ... wait for result

        # Start broadcasting department TF frames
        self.start_department_broadcasts(
            box_id, box_params, request.num_departments
        )

        # Publish department markers
        self.publish_department_markers(
            box_id, box_params, request.num_departments
        )

        # Track active box
        self.active_boxes[box_id] = {
            'address': address,
            'num_departments': request.num_departments
        }

        response.success = True
        return response

    def generate_box_model(self, box_id, params, num_departments):
        """Generate SDF/URDF model for box with departments"""
        # Use storage_params.yaml dimensions
        width = params['width']
        height = params['height']
        dept_depth = params['department_depth']
        total_depth = dept_depth * num_departments

        # Generate simple box model (can be template file)
        model = f"""
        <?xml version="1.0"?>
        <sdf version="1.6">
          <model name="{box_id}">
            <link name="box_link">
              <visual name="visual">
                <geometry>
                  <box>
                    <size>{width} {total_depth} {height}</size>
                  </box>
                </geometry>
                <material>
                  <ambient>0.3 0.5 0.8 1.0</ambient>
                  <diffuse>0.3 0.5 0.8 1.0</diffuse>
                </material>
              </visual>
              <collision name="collision">
                <geometry>
                  <box>
                    <size>{width} {total_depth} {height}</size>
                  </box>
                </geometry>
              </collision>
              <inertial>
                <mass>0.5</mass>
                <inertia>
                  <ixx>0.01</ixx>
                  <iyy>0.01</iyy>
                  <izz>0.01</izz>
                </inertia>
              </inertial>
            </link>
          </model>
        </sdf>
        """
        return model

    def start_department_broadcasts(self, box_id, params, num_depts):
        """Broadcast TF frames for each department in box"""
        timer = self.create_timer(
            0.1,  # 10 Hz
            lambda: self.publish_department_tfs(box_id, params, num_depts)
        )
        self.active_boxes[box_id]['tf_timer'] = timer

    def publish_department_tfs(self, box_id, params, num_depts):
        """Publish department frame transforms"""
        dept_depth = params['department_depth']
        dept_offset = params['department_offset_y']

        transforms = []
        for dept_num in range(1, num_depts + 1):
            t = TransformStamped()
            t.header.stamp = self.get_clock().now().to_msg()
            t.header.frame_id = f"{box_id}_link"
            t.child_frame_id = f"{box_id}_dept_{dept_num}"

            # Calculate department position in box frame
            t.transform.translation.x = 0.0
            t.transform.translation.y = dept_offset + (dept_num - 1) * dept_depth
            t.transform.translation.z = 0.0
            t.transform.rotation.w = 1.0

            transforms.append(t)

        for t in transforms:
            self.tf_broadcaster.sendTransform(t)

    def publish_department_markers(self, box_id, params, num_depts):
        """Publish visual markers for department centers (pick locations)"""
        marker_array = MarkerArray()

        dept_depth = params['department_depth']
        dept_offset = params['department_offset_y']
        width = params['width']
        height = params['height']

        for dept_num in range(1, num_depts + 1):
            # Calculate department center position
            # This matches the TF frame position (dept center, not boundary)
            dept_center_y = dept_offset + (dept_num - 1) * dept_depth + dept_depth / 2.0

            marker = Marker()
            marker.header.frame_id = f"{box_id}_link"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = f"{box_id}_departments"
            marker.id = dept_num
            marker.type = Marker.SPHERE  # Sphere at center
            marker.action = Marker.ADD

            # Position at department center (where picker should go)
            marker.pose.position.x = 0.0
            marker.pose.position.y = dept_center_y
            marker.pose.position.z = 0.0
            marker.pose.orientation.w = 1.0

            # Size (small sphere to mark pick location)
            marker.scale.x = 0.015
            marker.scale.y = 0.015
            marker.scale.z = 0.015

            # Color (red semi-transparent sphere)
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
            marker.color.a = 0.6

            marker_array.markers.append(marker)

            # Optional: Add text label for department number
            text_marker = Marker()
            text_marker.header = marker.header
            text_marker.ns = f"{box_id}_dept_labels"
            text_marker.id = dept_num
            text_marker.type = Marker.TEXT_VIEW_FACING
            text_marker.action = Marker.ADD
            text_marker.pose = marker.pose
            text_marker.pose.position.z = height / 2.0 + 0.02  # Above box
            text_marker.scale.z = 0.02  # Text height
            text_marker.color.r = 1.0
            text_marker.color.g = 1.0
            text_marker.color.b = 1.0
            text_marker.color.a = 1.0
            text_marker.text = f"D{dept_num}"

            marker_array.markers.append(text_marker)

        self.marker_pub.publish(marker_array)

    def despawn_box_callback(self, request, response):
        """Despawn box when returned to cabinet"""
        box_id = request.box_id

        if box_id not in self.active_boxes:
            response.success = False
            response.message = f"Box {box_id} not active"
            return response

        # Stop TF broadcasts
        timer = self.active_boxes[box_id].get('tf_timer')
        if timer:
            timer.cancel()

        # Delete markers
        self.delete_department_markers(box_id)

        # Delete from Gazebo
        delete_req = DeleteEntity.Request()
        delete_req.name = box_id
        future = self.delete_model_client.call_async(delete_req)
        # ... wait for result

        # Remove from tracking
        del self.active_boxes[box_id]

        response.success = True
        return response

    def get_box_parameters(self, cabinet_num, num_depts):
        """Load box parameters from storage_params.yaml"""
        # Parse storage_params.yaml
        # Return dimensions for the box type at this cabinet
        # Example:
        return {
            'width': 0.06,
            'height': 0.09,
            'department_depth': 0.02,
            'department_offset_y': 0.005
        }
```

**Service Definitions:**

```
# SpawnBox.srv
string box_id
string side
uint8 cabinet_num
uint8 row
uint8 column
uint8 num_departments
---
bool success
string message


# DespawnBox.srv
string box_id
---
bool success
string message
```

**Integration with ExtractBox Action:**

```python
# In ExtractBox action server
def execute_callback(self, goal_handle):
    # ... extraction motion ...

    # On successful extraction, spawn box
    spawn_req = SpawnBox.Request()
    spawn_req.box_id = f"box_{goal.cabinet}_{goal.row}_{goal.col}"
    spawn_req.side = goal.side
    spawn_req.cabinet_num = goal.cabinet
    spawn_req.row = goal.row
    spawn_req.column = goal.column
    spawn_req.num_departments = self.get_num_depts(goal.cabinet)

    future = self.spawn_box_client.call_async(spawn_req)
    # ... wait and check

    result.box_extracted = True
    result.box_id = spawn_req.box_id
    return result
```

### 8. ✅ Visual Markers for State Indication

**REQUIREMENT:** Use colored markers to indicate system state

**Markers to Implement:**

1. **Magnet Engaged:** Red sphere on gripper magnet link
2. **Target Address:** Green box at target address frame
3. **Empty Address:** Red box at address (when box extracted)
4. **Department Markers:** Red transparent dividers (handled by box spawner)

**Implementation: Marker Publisher Node**

```python
class StateMarkerPublisherNode(Node):
    """Publishes visual markers for system state indication"""

    def __init__(self):
        super().__init__('state_marker_publisher')

        self.marker_pub = self.create_publisher(
            MarkerArray, '/visualization_marker_array', 10
        )

        # Subscribe to manipulator state
        self.create_subscription(
            ManipulatorState, '/manipulator/state',
            self.state_callback, 10
        )

        # Subscribe to electromagnet state
        self.create_subscription(
            Bool, '/manipulator/electromagnet/engaged',
            self.magnet_callback, 10
        )

        self.current_state = None
        self.magnet_engaged = False
        self.target_address = None
        self.extracted_addresses = set()

        # Publish markers at 10 Hz
        self.create_timer(0.1, self.publish_markers)

    def publish_markers(self):
        markers = MarkerArray()

        # 1. Magnet engaged marker
        if self.magnet_engaged:
            magnet_marker = self.create_magnet_marker()
            markers.markers.append(magnet_marker)

        # 2. Target address marker (green)
        if self.target_address:
            target_marker = self.create_address_marker(
                self.target_address,
                color=(0.0, 1.0, 0.0, 0.5),  # Green
                marker_id=1000
            )
            markers.markers.append(target_marker)

        # 3. Extracted address markers (red)
        for idx, address in enumerate(self.extracted_addresses):
            empty_marker = self.create_address_marker(
                address,
                color=(1.0, 0.0, 0.0, 0.5),  # Red
                marker_id=2000 + idx
            )
            markers.markers.append(empty_marker)

        self.marker_pub.publish(markers)

    def create_magnet_marker(self):
        """Create red sphere marker on gripper magnet"""
        marker = Marker()
        marker.header.frame_id = "gripper_magnet_link"  # Or appropriate link name
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "magnet_status"
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD

        marker.pose.position.x = 0.0
        marker.pose.position.y = 0.0
        marker.pose.position.z = 0.0
        marker.pose.orientation.w = 1.0

        marker.scale.x = 0.05
        marker.scale.y = 0.05
        marker.scale.z = 0.05

        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 0.8

        return marker

    def create_address_marker(self, address, color, marker_id):
        """Create colored box marker at address frame"""
        side, cabinet, row, col = address
        side_abbrev = 'l' if side == 'left' else 'r'
        frame_name = f"addr_{side_abbrev}_{cabinet}_{row}_{col}"

        marker = Marker()
        marker.header.frame_id = frame_name
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "address_status"
        marker.id = marker_id
        marker.type = Marker.CUBE
        marker.action = Marker.ADD

        marker.pose.position.x = 0.0
        marker.pose.position.y = 0.1  # Offset into cabinet
        marker.pose.position.z = 0.0
        marker.pose.orientation.w = 1.0

        # Size from box dimensions
        marker.scale.x = 0.06
        marker.scale.y = 0.2
        marker.scale.z = 0.09

        marker.color.r = color[0]
        marker.color.g = color[1]
        marker.color.b = color[2]
        marker.color.a = color[3]

        return marker
```

### 9. ✅ Configuration Management (Single Source of Truth)

**REQUIREMENT:** Use existing YAML files, no duplication

**CRITICAL: Unified Limits Architecture (Updated 2025-11-25)**

All joint limits follow a single source of truth architecture:

```
manipulator_params.yaml (PRIMARY SOURCE)
        │
        ├──► URDF <limit> tags (hard limits)
        ├──► URDF <safety_controller> tags (soft limits)
        ├──► ros2_control.xacro <command_interface> limits (= soft limits)
        └──► ControllerInterface validation (= soft limits)
```

**Three Types of Limits:**
| Type | Source | Purpose |
|------|--------|---------|
| Hard Limits | `manipulator_params.yaml` → URDF `<limit>` | Physical joint range |
| Soft Limits | `manipulator_params.yaml` → URDF `<safety_controller>` | Operational safety margin |
| Controller Limits | `manipulator_params.yaml` → ros2_control `<command_interface>` | **= Soft Limits** |

**Configuration Hierarchy:**

```
Primary Config Files (manipulator_description package):
1. manipulator_params.yaml         → Joint limits (hard + soft), physical params (SINGLE SOURCE OF TRUTH)
2. manipulator_controllers.yaml    → Controller names, types, joint mappings
3. storage_params.yaml             → Storage system dimensions, cabinet layouts, departments

Secondary Config Files (manipulator_control package):
4. limit_switches.yaml             → 18 switch trigger positions (references joints from params)
5. action_servers.yaml             → Action server timeouts, speeds, tolerances
6. kinematic_chains.yaml           → Joint group definitions for coordinated motion
7. visualization.yaml              → All visualization markers
8. trajectory.yaml                 → YZ trajectory speeds, safety margins
9. electromagnet.yaml              → Magnet simulation parameters
10. box_spawner.yaml               → Box spawning, TF broadcast rates
11. load_positions.yaml            → External load station coordinates
12. container_storage.yaml         → Container storage locations and slots
13. pick_item_states.yaml          → Picker state machine configuration
14. error_handling.yaml            → Error codes and recovery strategies
15. testing.yaml                   → Test suite and profiling configuration
```

**Load Order:**

```python
class ActionServerBase(Node):
    def __init__(self, node_name):
        super().__init__(node_name)

        # 1. Load joint limits from manipulator_params.yaml (SINGLE SOURCE)
        # Soft limits are used for validation (same as controller command_interface limits)
        self.joint_limits = self.load_joint_limits_from_params()

        # 2. Load storage system config
        self.storage_config = self.load_yaml(
            'manipulator_description',
            'config/storage_params.yaml'
        )

        # 3. Load action-specific config
        self.action_config = self.load_yaml(
            'manipulator_control',
            'config/action_servers.yaml'
        )

    def load_joint_limits_from_params(self):
        """Load soft limits from manipulator_params.yaml (single source of truth)"""
        params = self.load_yaml('manipulator_description', 'config/manipulator_params.yaml')
        limits = {}
        for assembly_name, assembly in params.items():
            if not isinstance(assembly, dict):
                continue
            for key, value in assembly.items():
                if isinstance(value, dict) and 'safety_controller' in value:
                    sc = value['safety_controller']
                    limits[key] = {
                        'min': sc['soft_lower'],
                        'max': sc['soft_upper'],
                        'velocity': value['limits'].get('velocity', 1.0),
                    }
        return limits

    def load_yaml(self, package_name, relative_path):
        """Load YAML from package"""
        from ament_index_python.packages import get_package_share_directory
        import yaml

        pkg_path = get_package_share_directory(package_name)
        full_path = os.path.join(pkg_path, relative_path)

        with open(full_path, 'r') as f:
            return yaml.safe_load(f)
```

### 9.1 ✅ Joystick Control Package (joy_control)

**UPDATED 2025-11-26:** The `joy_control` package provides manual joystick control for testing and development with **smooth motion** via streaming trajectory goals.

**Smooth Motion Architecture (Option C - Streaming Trajectory Goals):**

The joystick controller uses a dual-timer architecture to provide smooth motion:

```
┌─────────────────────────────────────────────────────────────────────┐
│                    SMOOTH JOYSTICK CONTROL                          │
├─────────────────────────────────────────────────────────────────────┤
│                                                                      │
│  Joy Callback (20Hz)              Trajectory Timer (10Hz)           │
│        │                                  │                         │
│        ▼                                  ▼                         │
│  Read joystick axes              Send accumulated targets           │
│        │                         as trajectory goals                │
│        ▼                                  │                         │
│  Calculate velocity                       ▼                         │
│  (axis × velocity_scale)         Trajectory controller              │
│        │                         interpolates smoothly              │
│        ▼                         (spline interpolation)             │
│  Accumulate target position              │                         │
│  (current + velocity × dt)               ▼                         │
│        │                         Robot moves smoothly               │
│        ▼                         (no jerky steps)                   │
│  Clamp to limits                                                    │
│  (from manipulator_params.yaml)                                     │
│                                                                      │
└─────────────────────────────────────────────────────────────────────┘
```

**Why This Approach:**

| Issue with ForwardCommand | Solution with Streaming Trajectories |
|--------------------------|-------------------------------------|
| Instant position jumps (jerky) | Trajectory controller interpolates smoothly |
| 20Hz position updates create steps | 10Hz trajectory goals with spline interpolation |
| No velocity profile | Duration calculated from joint max velocity |
| Abrupt stops when releasing | Trajectory completes smoothly to last target |

**Controller Interface Compatibility:**

The joy_control package uses the hybrid controller architecture:
- **7 motion joints:** `JointTrajectoryController` (action-based `FollowJointTrajectory`) - smooth motion
- **2 container jaws:** `ForwardCommandController` (topic-based `Float64MultiArray`) - instant response

**Joint Classification:**

```python
# Motion joints use trajectory controller (action clients) - SMOOTH
TRAJECTORY_JOINTS = frozenset([
    'base_main_frame_joint',
    'main_frame_selector_frame_joint',
    'selector_frame_gripper_joint',
    'selector_frame_picker_frame_joint',
    'picker_frame_picker_rail_joint',
    'picker_rail_picker_base_joint',
    'picker_base_picker_jaw_joint'
])

# Container jaws use forward command controller (topic publishers) - INSTANT
FORWARD_COMMAND_JOINTS = frozenset([
    'selector_left_container_jaw_joint',
    'selector_right_container_jaw_joint'
])
```

**Configuration (Single Source of Truth):**

Joint limits AND velocities are loaded from `manipulator_params.yaml` at runtime - **NOT duplicated** in `manipulator_joy_config.yaml`:

```yaml
# manipulator_joy_config.yaml - CORRECT (no limits field)
joy_controller_node:
  ros__parameters:
    # Timing parameters
    update_rate: 20.0              # Joy callback rate (Hz)
    trajectory_update_rate: 10.0   # Trajectory goal send rate (Hz)

    # Trajectory duration bounds
    trajectory_duration_min: 0.05  # Minimum duration (s)
    trajectory_duration_max: 2.0   # Maximum duration (s)

    # Axis mappings - NO LIMITS (loaded from manipulator_params.yaml)
    axis_mappings.base_main_frame.joint_name: "base_main_frame_joint"
    axis_mappings.base_main_frame.axis_index: 1
    axis_mappings.base_main_frame.axis_scale: -1.0
    axis_mappings.base_main_frame.velocity_scale: 0.5
    # limits: LOADED FROM manipulator_params.yaml (safety_controller)
    # velocity: LOADED FROM manipulator_params.yaml (limits.velocity)

# WRONG - DO NOT duplicate limits like this:
# axis_mappings.base_main_frame.limits: [0.0, 4.0]  # DUPLICATED!
```

**Joystick Mapping (PlayStation Controller):**

| Control | Axis/Button | Joint | Range (from params) |
|---------|-------------|-------|---------------------|
| Left Stick Y | Axis 1 | base_main_frame_joint | 0.1-3.9m |
| Right Stick Y | Axis 4 | main_frame_selector_frame_joint | 0.05-1.45m |
| Right Stick X | Axis 3 | selector_frame_gripper_joint | -0.39-0.39m |
| D-Pad Y | Axis 7 | selector_frame_picker_frame_joint | 0.005-0.29m |
| D-Pad X | Axis 6 | picker_frame_picker_rail_joint | -0.29-0.29m |
| Left Stick X | Axis 0 | picker_rail_picker_base_joint | 0.005-0.24m |
| Triangle/Cross | Button 2/0 | picker_base_picker_jaw_joint | 0.005-0.19m |
| L2/R2 | Button 6/7 | container jaws | -0.19-0.19m |
| L1 | Button 4 | Enable control (hold) | - |
| R1 | Button 5 | Turbo mode (hold) | - |

**Streaming Trajectory Implementation:**

```python
def joy_callback(self, msg: Joy):
    """Process joystick input - accumulate target positions (20Hz)"""
    if not self.enabled:
        return

    dt = 1.0 / self.joy_update_rate

    for joint_name, mapping in self.axis_mappings.items():
        axis_value = self.get_axis_value(msg, mapping)
        if abs(axis_value) < 0.01:
            continue  # Deadzone

        # Velocity mode: calculate target from current + velocity × dt
        velocity = axis_value * mapping['velocity_scale'] * self.scale_linear
        if self.turbo_enabled:
            velocity *= 2.0

        current = self.joint_positions.get(joint_name, 0.0)
        target = current + velocity * dt

        # Clamp to limits from manipulator_params.yaml
        limits = self.joint_limits[joint_name]
        target = max(limits['min'], min(limits['max'], target))

        # Accumulate target (will be sent by trajectory timer)
        self.pending_targets[joint_name] = target

def send_trajectory_goals(self):
    """Send accumulated targets as trajectory goals (10Hz)"""
    for joint_name, target in self.pending_targets.items():
        # Skip if target hasn't changed significantly
        if abs(target - self.last_sent_targets.get(joint_name, 0)) < 0.001:
            continue

        if joint_name in self.TRAJECTORY_JOINTS:
            # Cancel previous goal (preemption for responsiveness)
            self._cancel_previous_goal(joint_name)

            # Calculate duration from distance and max velocity
            current = self.joint_positions.get(joint_name, target)
            distance = abs(target - current)
            max_velocity = self.joint_limits[joint_name]['velocity']
            duration = max(0.05, min(2.0, distance / max_velocity))

            # Send trajectory goal
            self._send_trajectory_goal(joint_name, target, duration)
        else:
            # Container jaws: instant forward command
            self._send_forward_command(joint_name, target)

        self.last_sent_targets[joint_name] = target
```

**Launch Configuration:**

```bash
# Enable joystick control with simulation
ros2 launch manipulator_control manipulator_simulation.launch.py enable_joy:=true

# Joystick control requires holding L1 button to enable motion
# Hold R1 for turbo mode (2× velocity)
```

### 10. ✅ GUI Approach (Standard RQt Tools First)

**REVISED DECISION:** Use **standard RQt tools** for initial development, create custom plugin only if needed

**Standard RQt Tools Available:**

1. **rqt_action** - Send action goals, monitor feedback/results
2. **rqt_publisher** - Publish to topics (joint commands, electromagnet toggle)
3. **rqt_service_caller** - Call services (spawn box, get address)
4. **rqt_topic** - Monitor topic data (joint states, end switches)
5. **rqt_tf_tree** - Visualize TF frames (verify address frames, departments)
6. **rqt_graph** - Visualize node/topic graph
7. **rqt_console** - View log messages
8. **rqt_reconfigure** - Dynamic parameter tuning

**Development Approach:**

**Phase 1-3:** Use standard RQt tools
- Test actions with `rqt_action`
- Monitor system with `rqt_topic` and `rqt_console`
- Verify TF with `rqt_tf_tree`

**Phase 4+:** Evaluate if custom plugin needed
- If standard tools sufficient → stick with them
- If workflow cumbersome → develop custom panel

**Example RQt Session:**

```bash
# Launch full system
ros2 launch manipulator_control simulation.launch.py

# In separate terminals, launch RQt tools:
rqt  # Main RQt window

# Load plugins:
# - Plugins → Actions → Action Type Browser (send action goals)
# - Plugins → Topics → Topic Monitor (view end switches)
# - Plugins → Visualization → TF Tree (verify frames)
# - Plugins → Logging → Console (debug messages)

# Save perspective: Perspectives → Create Perspective → "Manipulator Development"
```

**If Custom Plugin Needed Later:**

**Implementation:**

```python
# manipulator_control/gui/rqt_command_panel.py

from rqt_gui_py.plugin import Plugin
from python_qt_binding.QtWidgets import QWidget, QVBoxLayout, QPushButton
from rclpy.action import ActionClient

class ManipulatorCommandPanel(Plugin):
    """RQt plugin for manipulator control"""

    def __init__(self, context):
        super().__init__(context)
        self.setObjectName('ManipulatorCommandPanel')

        # Create widget
        self._widget = QWidget()
        self._widget.setWindowTitle('Manipulator Control')

        # Build UI
        self.init_ui()

        # Add to context
        if context.serial_number() > 1:
            self._widget.setWindowTitle(
                self._widget.windowTitle() +
                f' ({context.serial_number()})'
            )
        context.add_widget(self._widget)

        # ROS2 node (provided by rqt)
        self._node = context.node

        # Action clients
        self.init_action_clients()

    def init_ui(self):
        layout = QVBoxLayout()
        # ... build UI similar to standalone version
        self._widget.setLayout(layout)

    def init_action_clients(self):
        self.action_clients = {
            'PickItemFromStorage': ActionClient(
                self._node,
                PickItemFromStorage,
                'pick_item_from_storage'
            ),
            # ... more clients
        }

    def shutdown_plugin(self):
        # Cleanup
        pass
```

**Plugin XML Registration:**

```xml
<!-- manipulator_control/plugin.xml -->
<library path="manipulator_control.gui">
  <class name="Manipulator Command Panel"
         type="manipulator_control.gui.rqt_command_panel.ManipulatorCommandPanel"
         base_class_type="rqt_gui_py::Plugin">
    <description>
      GUI for issuing high-level commands to the manipulator
    </description>
    <qtgui>
      <label>Manipulator Control</label>
      <icon type="theme">input-gaming</icon>
      <statustip>Control warehouse manipulator actions</statustip>
    </qtgui>
  </class>
</library>
```

**Package.xml Entry:**

```xml
<export>
  <rqt_gui plugin="${prefix}/plugin.xml"/>
</export>
```

**Launch:**

```bash
ros2 run rqt_gui rqt_gui --standalone manipulator_control.gui.rqt_command_panel.ManipulatorCommandPanel
# Or within rqt
rqt
# Then: Plugins → Manipulator → Manipulator Control
```

### 11. ✅ Additional Box Manipulation Actions

**NEW REQUIREMENTS:** Box relocation and loading operations

#### A. PutBox Action - Place Box in Different Address

**Purpose:** Put extracted box into any empty address (enables box reorganization)

**Constraints:**
- Target address must be empty
- Box width must match cabinet column configuration
- Box can be placed in different cabinet/row/column than original

**PutBox.action Definition:**

```
# PutBox.action

# Goal
string box_id                 # Currently held box
string target_side            # "left" or "right"
uint8 target_cabinet_num
uint8 target_row
uint8 target_column
---
# Result
bool success
bool box_placed
bool address_verified_empty
string message
---
# Feedback
string current_phase  # "verifying_address", "navigating", "extending_gripper", "releasing", "retracting"
uint8 progress_percent
geometry_msgs/Pose current_pose
```

**Implementation Logic:**

```python
def execute_put_box(goal):
    # 1. Verify target address is empty
    target_address = (goal.target_side, goal.target_cabinet_num,
                     goal.target_row, goal.target_column)

    if not is_address_empty(target_address):
        return Result(success=False, message="Target address occupied")

    # 2. Get box parameters from box_id
    box_params = get_box_parameters(goal.box_id)

    # 3. Verify box width matches target cabinet columns
    target_cabinet_config = get_cabinet_config(goal.target_side,
                                               goal.target_cabinet_num)
    if box_params['columns'] != target_cabinet_config['columns']:
        return Result(success=False,
                     message=f"Box width mismatch: {box_params['columns']} cols "
                            f"vs cabinet {target_cabinet_config['columns']} cols")

    # 4. Navigate to target address (X, Z positioning)
    navigate_result = navigate_to_address_client.send_goal(target_address)
    if not navigate_result.success:
        return Result(success=False, message="Navigation failed")

    # 5. Execute YZ trajectory to insert box into cabinet
    # Start: gripper at front of cabinet, box outside
    # End: gripper extended into cabinet, box at address
    yz_trajectory = generate_insertion_trajectory(
        current_y=gripper_current_y,
        target_y=address_y_depth,
        current_z=gripper_current_z,
        target_z=address_z_height,
        approach_speed=0.05  # Slow for safety
    )

    execute_yz_trajectory(yz_trajectory)

    # 6. Release electromagnet
    electromagnet_toggle(activate=False)

    # 7. Retract gripper (reverse YZ trajectory)
    retract_trajectory = generate_retraction_trajectory(
        start_y=address_y_depth,
        end_y=gripper_safe_y,
        speed=0.08
    )

    execute_yz_trajectory(retract_trajectory)

    # 8. Despawn box (visual representation)
    despawn_box(goal.box_id)

    # 9. Mark address as occupied
    mark_address_occupied(target_address, goal.box_id)

    return Result(success=True, box_placed=True)
```

#### B. MoveBoxToLoad Action - Box Loading Operation

**Purpose:** Complete workflow to move box to loading position and optionally return to storage

**Use Case:**
- Extract box from storage
- Move to predefined load position (for external access/inspection)
- Optionally move back to storage (same or different address)

**MoveBoxToLoad.action Definition:**

```
# MoveBoxToLoad.action

# Goal
# Source address
string source_side
uint8 source_cabinet_num
uint8 source_row
uint8 source_column

# Load position (predefined or custom)
string load_position_name     # "load_station_left", "load_station_right", "inspection_area"

# Return behavior
bool return_to_storage        # If true, return box after loading
string return_side            # Optional: different address for return
uint8 return_cabinet_num      # If 0, use source address
uint8 return_row
uint8 return_column
---
# Result
bool success
bool box_extracted
bool box_at_load_position
bool box_returned              # Only if return_to_storage=true
string box_id
string message
---
# Feedback
string current_operation  # "extracting", "moving_to_load", "at_load_position", "returning", "completed"
uint8 progress_percent
duration elapsed_time
```

**Implementation Workflow:**

```python
def execute_move_box_to_load(goal):
    feedback = Feedback()

    # Phase 1: Extract box from source address
    feedback.current_operation = "extracting"
    feedback.progress_percent = 10
    publish_feedback(feedback)

    extract_goal = ExtractBox.Goal()
    extract_goal.side = goal.source_side
    extract_goal.cabinet_num = goal.source_cabinet_num
    extract_goal.row = goal.source_row
    extract_goal.column = goal.source_column

    extract_result = extract_box_client.send_goal_and_wait(extract_goal)

    if not extract_result.success:
        return Result(success=False, message="Extraction failed")

    box_id = extract_result.box_id

    # Phase 2: Move to load position
    feedback.current_operation = "moving_to_load"
    feedback.progress_percent = 40
    publish_feedback(feedback)

    load_position = get_load_position_coordinates(goal.load_position_name)

    # Navigate to load position (X, Z)
    navigate_to_coordinates(load_position['x'], load_position['z'])

    # Lower selector to load height if needed
    move_joint('main_frame_selector_frame_joint', load_position['z'])

    # Phase 3: At load position (wait/external interaction)
    feedback.current_operation = "at_load_position"
    feedback.progress_percent = 60
    publish_feedback(feedback)

    # Box is now accessible for external system (Level 2, operator, etc.)
    # Could wait for external signal, timeout, or immediate proceed

    # Phase 4: Return to storage (if requested)
    if goal.return_to_storage:
        feedback.current_operation = "returning"
        feedback.progress_percent = 75
        publish_feedback(feedback)

        # Determine return address
        if goal.return_cabinet_num == 0:
            # Return to original address
            return_address = (goal.source_side, goal.source_cabinet_num,
                            goal.source_row, goal.source_column)
        else:
            # Return to specified different address
            return_address = (goal.return_side, goal.return_cabinet_num,
                            goal.return_row, goal.return_column)

        # Put box at return address
        put_goal = PutBox.Goal()
        put_goal.box_id = box_id
        put_goal.target_side = return_address[0]
        put_goal.target_cabinet_num = return_address[1]
        put_goal.target_row = return_address[2]
        put_goal.target_column = return_address[3]

        put_result = put_box_client.send_goal_and_wait(put_goal)

        if not put_result.success:
            return Result(success=False, box_at_load_position=True,
                         box_returned=False, box_id=box_id,
                         message="Return to storage failed")

    # Phase 5: Complete
    feedback.current_operation = "completed"
    feedback.progress_percent = 100
    publish_feedback(feedback)

    return Result(
        success=True,
        box_extracted=True,
        box_at_load_position=True,
        box_returned=goal.return_to_storage,
        box_id=box_id
    )
```

**Load Position Configuration:**

```yaml
# config/load_positions.yaml
load_positions:
  load_station_left:
    description: "Left side loading station"
    coordinates:
      x: 0.5      # Middle of rail
      y: 0.0      # Centered
      z: 1.0      # Comfortable access height

  load_station_right:
    description: "Right side loading station"
    coordinates:
      x: 3.5
      y: 0.0
      z: 1.0

  inspection_area:
    description: "Visual inspection position"
    coordinates:
      x: 2.0      # Center of workspace
      y: 0.0
      z: 1.2      # Higher for camera view
```

#### C. YZ Trajectory Generation for Box Insertion/Extraction (Parametric Curves)

**Purpose:** Execute smooth YZ trajectories for box insertion/extraction using parametric curves

**Approach:** Predefined Bezier curves designed in SVG, converted to YAML waypoints, executed via JointTrajectoryController

**Why Parametric Curves (not collision checking):**
- Predictable, repeatable motion profiles
- Easy to edit visually in Inkscape/SVG editor
- No runtime computation overhead
- Smooth spline interpolation via JointTrajectoryController

**Trajectory Types:**

1. **Insertion Trajectory (PutBox, ReturnBox)**
   - Start: Gripper at cabinet front (Y=0)
   - Path: Bezier curve moving Y+ into cabinet with subtle Z adjustments
   - End: Box at address depth (Y≈0.4m)

2. **Extraction Trajectory (ExtractBox)**
   - Start: Gripper extended into cabinet (Y≈0.4m)
   - Path: Bezier curve moving Y- out with slight Z lift for clearance
   - End: Box fully extracted (Y=0)

**Trajectory Config (external scaling - SVG is unitless):**

```yaml
# config/trajectory_config.yaml
trajectories:
  extract_left:
    svg_file: trajectories/extract_left.svg
    mapping:
      x_range: [0, 100]        # SVG X coordinate range
      y_output: [0.0, 0.4]     # Joint Y output range (meters)
      y_center: 50             # SVG Y value that maps to Z=0
      z_scale: 0.001           # Meters per SVG unit (1 unit = 1mm)
    sampling:
      num_points: 20           # Waypoints to generate
      waypoint_duration: 0.5   # Seconds between waypoints
```

**SVG Source Files (unitless curves):**

```svg
<!-- config/trajectories/extract_left.svg -->
<svg viewBox="0 0 100 100">
  <!-- Unitless Bezier curves - scaling defined in trajectory_config.yaml -->
  <path id="insertion" d="M 0,50 C 30,50 70,48 100,50"/>
  <path id="extraction" d="M 100,50 C 70,52 30,52 0,50"/>
</svg>
```

**Converter Tool (Development Only):**

```bash
# Install converter dependency (dev only)
pip install svgpathtools

# Convert SVG to YAML using config (run once, commit result)
python3 scripts/svg_to_trajectory.py \
    --config config/trajectory_config.yaml \
    --trajectory extract_left \
    -o config/extraction_trajectories.yaml
```

**Generated YAML Format:**

```yaml
# config/extraction_trajectories.yaml (generated, committed)
source_svg: trajectories/extract_left.svg
config_used: trajectory_config.yaml
trajectories:
  insertion:
    - {y: 0.0, z: 0.0}
    - {y: 0.021, z: 0.0}
    - {y: 0.084, z: 0.001}
    # ... sampled from Bezier curve
  extraction:
    - {y: 0.4, z: 0.0}
    - {y: 0.379, z: -0.002}
    # ...
```

**Runtime Trajectory Generator:**

```python
class YZTrajectoryGenerator:
    """Load and execute parametric curve trajectories for box operations"""

    def __init__(self, config_path: str = 'config/extraction_trajectories.yaml'):
        with open(config_path) as f:
            self.config = yaml.safe_load(f)
        self.trajectories = self.config['trajectories']

    def load_trajectory(self, name: str, side: str, base_y: float, base_z: float,
                       waypoint_duration: float = 0.5) -> list[dict]:
        """
        Load trajectory and transform to world coordinates.

        Args:
            name: Trajectory name ('insertion' or 'extraction')
            side: Cabinet side ('left' or 'right') - right side flips Y sign
            base_y: Starting Y position (gripper center)
            base_z: Starting Z position (selector height)
            waypoint_duration: Time between waypoints (seconds)

        Returns:
            List of waypoints with world coordinates and timing
        """
        waypoints = self.trajectories[name]
        sign = 1.0 if side == 'left' else -1.0

        return [
            {
                'y': base_y + sign * wp['y'],
                'z': base_z + wp['z'],
                'time_from_start': i * waypoint_duration
            }
            for i, wp in enumerate(waypoints)
        ]

    def execute_trajectory(self, waypoints: list[dict],
                          trajectory_client) -> bool:
        """
        Execute trajectory via JointTrajectoryController.

        Args:
            waypoints: Transformed waypoints from load_trajectory()
            trajectory_client: ActionClient for FollowJointTrajectory

        Returns:
            True if trajectory completed successfully
        """
        from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
        from builtin_interfaces.msg import Duration

        traj = JointTrajectory()
        traj.joint_names = [
            'selector_frame_gripper_joint',      # Y-axis
            'main_frame_selector_frame_joint'    # Z-axis
        ]

        for wp in waypoints:
            point = JointTrajectoryPoint()
            point.positions = [wp['y'], wp['z']]
            point.velocities = [0.0, 0.0]  # Let controller interpolate
            secs = int(wp['time_from_start'])
            nsecs = int((wp['time_from_start'] - secs) * 1e9)
            point.time_from_start = Duration(sec=secs, nanosec=nsecs)
            traj.points.append(point)

        goal = FollowJointTrajectory.Goal()
        goal.trajectory = traj

        result = trajectory_client.send_goal_and_wait(goal)
        return result.error_code == FollowJointTrajectory.Result.SUCCESSFUL
```

**Integration in ExtractBox:**

```python
def execute_extract_box(goal):
    # ... navigate to address ...

    # Generate extraction trajectory
    traj_gen = YZTrajectoryGenerator(storage_config)

    current_y = get_joint_position('selector_frame_gripper_joint')
    current_z = get_joint_position('main_frame_selector_frame_joint')

    # Target: extend into cabinet to box depth
    target_y = address_y_position + box_depth / 2.0
    target_z = current_z  # Maintain Z

    # Insertion trajectory (to reach box)
    insertion_waypoints = traj_gen.generate_insertion_trajectory(
        current_y, current_z,
        target_y, target_z,
        approach_speed=0.05
    )

    traj_gen.execute_yz_trajectory(insertion_waypoints,
                                   ['main_frame_selector_frame_joint',
                                    'selector_frame_gripper_joint'])

    # Engage electromagnet
    toggle_electromagnet(True)
    time.sleep(0.5)  # Wait for magnetic attachment

    # Extraction trajectory (pull box out)
    safe_y = 0.0  # Gripper retracted position
    extraction_waypoints = traj_gen.generate_extraction_trajectory(
        target_y, target_z,
        safe_y,
        retract_speed=0.08
    )

    traj_gen.execute_yz_trajectory(extraction_waypoints,
                                   ['main_frame_selector_frame_joint',
                                    'selector_frame_gripper_joint'])

    # ... spawn box, mark extracted ...
```

### 12. ✅ No Item Location Storage

**CORRECTED:** Level 3 does NOT store item locations. That's Level 2's responsibility.

**What Level 3 Needs:**
- Level 2 provides item position in PickItem goal
- Level 3 executes pick at that position
- No database, no tracking

**PickItem Goal (Corrected):**

```
# PickItem.action

# Goal
geometry_msgs/Point item_position_in_box  # Level 2 provides this!
string grasp_type                         # "top", "side"
---
# Result
bool success
bool item_grasped
---
# Feedback
string current_phase
uint8 progress_percent
```

---

## Updated Package Structure

```
manipulator_control/
├── package.xml
├── setup.py
├── plugin.xml  ← NEW: RQt plugin registration
├── resource/
├── config/
│   ├── action_servers.yaml
│   ├── limit_switches.yaml       ← NEW: Switch trigger positions
│   └── container_storage.yaml
├── launch/
│   ├── control_system.launch.py
│   ├── gui_rqt.launch.py         ← NEW: Launch RQt plugin
│   ├── simulation_helpers.launch.py  ← NEW: Spawner, markers, switches
│   └── bridge_node.launch.py
├── manipulator_control/
│   ├── __init__.py
│   │
│   ├── actions/
│   │   ├── high_level/
│   │   │   ├── pick_item_from_storage.py
│   │   │   ├── get_container.py      ← NEW
│   │   │   └── return_box.py
│   │   ├── mid_level/
│   │   │   ├── navigate_to_address.py
│   │   │   ├── extract_box.py
│   │   │   ├── pick_item.py          ← Uses state machine
│   │   │   └── manipulate_container.py
│   │   └── low_level/
│   │       ├── move_joint.py         ← Single joint command
│   │       └── move_joint_group.py   ← Coordinated motion
│   │
│   ├── state/
│   │   ├── manipulator_state.py
│   │   └── safety_monitor.py
│   │
│   ├── simulation/                    ← NEW: Simulation helpers
│   │   ├── __init__.py
│   │   ├── box_spawn_manager.py      ← Dynamic box spawning
│   │   ├── virtual_limit_switches.py ← Simulate end stops
│   │   ├── electromagnet_simulator.py ← Magnet attach/detach
│   │   └── state_marker_publisher.py ← Visual markers
│   │
│   ├── gui/
│   │   ├── __init__.py
│   │   └── rqt_command_panel.py      ← RQt plugin
│   │
│   ├── utils/
│   │   ├── __init__.py
│   │   ├── address_resolver.py       ← TF-based address lookup
│   │   ├── config_loader.py          ← YAML loading utilities
│   │   ├── controller_interface.py   ← Send commands to individual controllers
│   │   ├── yz_trajectory_generator.py ← YZ plane trajectories for box insertion/extraction
│   │   └── address_validator.py      ← Check address empty, box width match
│   │
│   └── bridge/                        ← Future Level 2 interface
│       ├── rest_bridge.py
│       └── mqtt_bridge.py
│
├── action/
│   ├── PickItemFromStorage.action
│   ├── GetContainer.action            ← NEW
│   ├── PlaceContainer.action          ← NEW
│   ├── NavigateToAddress.action
│   ├── ExtractBox.action
│   ├── ReturnBox.action
│   ├── PutBox.action                  ← NEW: Box relocation
│   ├── MoveBoxToLoad.action           ← NEW: Box loading operation
│   ├── PickItem.action               ← Updated (no item DB lookup)
│   ├── ManipulateContainer.action
│   ├── MoveJoint.action              ← NEW: Single joint
│   └── MoveJointGroup.action         ← Coordinated joints
│
├── srv/
│   ├── GetAddressCoordinates.srv     ← NEW
│   ├── SpawnBox.srv                  ← NEW
│   ├── DespawnBox.srv                ← NEW
│   ├── ToggleElectromagnet.srv       ← NEW
│   └── GetManipulatorState.srv
│
└── msg/
    ├── ManipulatorState.msg
    └── WarehouseAddress.msg           ← Updated structure
```

---

## Development Roadmap - Phased Implementation

**Philosophy:** Each phase should be independently testable in Gazebo simulation using standard RQt tools.

---

### Phase 0: Prerequisites & Environment Setup

**Objective:** Ensure clean development environment

**Tasks:**
1. ✅ Verify Gazebo + ros2_control working (ALREADY DONE - joystick control works)
2. ✅ Verify all 9 joints controllable individually
3. ✅ Verify TF tree includes all address frames
4. Create manipulator_control package structure
5. Define all action/service/message interfaces

**Deliverables:**
- Empty `manipulator_control` package with correct structure
- All `.action`, `.srv`, `.msg` files defined
- Package compiles successfully

**Testing:**
```bash
cd ros2_ws
colcon build --packages-select manipulator_control
source install/setup.bash
ros2 interface list | grep manipulator_control  # Should show all interfaces
```

**Estimated Effort:** 0.5-1 day

**Blockers:** None

---

### Phase 1: Simulation Foundation & Joint Control

**Objective:** Build simulation support infrastructure and basic joint control

**Tasks:**

**1.1 Virtual Limit Switch Node**
- Implement `virtual_limit_switches_node.py`
- Load switch config from `limit_switches.yaml`
- Publish switch states based on joint positions
- Test with manual joint commands

**1.2 Controller Interface Utility**
- Implement `controller_interface.py`
- Helper to send position commands to individual joint controllers
- Load controller names from `manipulator_controllers.yaml`

**1.3 MoveJoint Action Server**
- Single joint position control
- No limit switch integration yet (just position commands)
- Test with `rqt_action`

**1.4 State Marker Publisher Node**
- Implement basic markers (magnet indicator only for now)
- Subscribe to simple magnet state topic

**Deliverables:**
- `virtual_limit_switches_node` running, publishing all switch states
- `move_joint_action_server` can move individual joints
- Can monitor switches in `rqt_topic`
- Can send joint commands via `rqt_action`

**Testing:**
```bash
# Terminal 1: Launch Gazebo + controllers (existing)
ros2 launch manipulator_description gazebo.launch.py

# Terminal 2: Launch simulation helpers
ros2 run manipulator_control virtual_limit_switches_node

# Terminal 3: Launch action server
ros2 run manipulator_control move_joint_action_server

# Terminal 4: Monitor switches
ros2 topic echo /manipulator/end_switches/picker_jaw_opened

# Terminal 5: Send action goal
rqt  # Use Action Type Browser to send MoveJoint goals

# Verify: Joint moves, switches change state when limits reached
```

**Success Criteria:**
- All 18 limit switches (9 joints × 2) publish correct states
- Can move any joint via MoveJoint action
- Switches trigger at correct positions (within 0.01m tolerance)

**Estimated Effort:** 2-3 days

**Blockers:** None (all prerequisites from Phase 0)

---

### Phase 2: Address Resolution & Navigation

**Objective:** Enable navigation to warehouse addresses

**Tasks:**

**2.1 Address Resolver Utility**
- Implement `address_resolver.py`
- TF2 lookup for address frame coordinates
- Service: `GetAddressCoordinates`
- Validate against storage_params.yaml

**2.2 MoveJointGroup Action Server**
- Coordinate multiple joints (e.g., rail + selector)
- Use joint group definitions from config
- Simple sequential motion (no complex trajectories yet)

**2.3 NavigateToAddress Action Server**
- Use address resolver to get target coordinates
- Convert coordinates to joint positions (inverse kinematics - simple for XZ motion)
- Command rail + selector joints
- Verify position reached

**2.4 Address Marker Visualization**
- Extend state marker publisher
- Green marker on target address
- Test with manual address goals

**Deliverables:**
- Can query any address coordinates via service
- `NavigateToAddress` action moves manipulator to any cabinet address
- Green marker shows target address in RViz

**Testing:**
```bash
# Launch system
ros2 launch manipulator_description gazebo.launch.py
ros2 run manipulator_control virtual_limit_switches_node
ros2 run manipulator_control navigate_to_address_action_server

# Test address resolution
ros2 service call /manipulator/get_address_coordinates \
  manipulator_control/srv/GetAddressCoordinates \
  "{side: 'left', cabinet_num: 1, row: 2, column: 3}"

# Test navigation
rqt  # Use Action Browser to send NavigateToAddress goal
# Target: left, cabinet 1, row 2, column 3

# Verify in RViz:
# - Manipulator moves to correct X/Z position
# - Green marker appears at target address
# - Position error < 0.02m
```

**Success Criteria:**
- Address resolution returns correct (x, y, z) for all addresses
- Navigation reaches target within 0.02m error
- Repeatable (3/3 successful navigations to same address)

**Estimated Effort:** 2-3 days

**Blockers:** Phase 1 complete

---

### Phase 3: Box Handling - Spawn & Electromagnet

**Objective:** Dynamic box spawning, magnetic attachment, and parametric curve-based YZ trajectory execution

**Tasks:**

**3.1 YZ Trajectory Generator Utility (Parametric Curves)**
- Create SVG source files for insertion/extraction curves (`config/trajectories/*.svg`)
- Implement `scripts/svg_to_trajectory.py` converter (dev tool, requires svgpathtools)
- Generate `config/extraction_trajectories.yaml` from SVG
- Implement `yz_trajectory_generator.py` runtime loader
- Load waypoints, transform for side/position, execute via JointTrajectoryController
- Test smooth motion with multiple addresses

**3.2 Address Validator Utility**
- Implement `address_validator.py`
- Check if address is empty/occupied
- Verify box width matches cabinet column count
- Load cabinet configurations from storage_params.yaml

**3.3 Electromagnet Simulator Node**
- Implement `electromagnet_simulator_node.py`
- Service: `ToggleElectromagnet`
- Use Gazebo attach/detach services
- Proximity check before attaching
- Publish magnet state

**3.4 Box Spawn Manager Node (Basic)**
- Implement `box_spawn_manager_node.py`
- Service: `SpawnBox`, `DespawnBox`
- Generate SDF model from storage_params.yaml
- Spawn at address TF frame position
- NO department frames yet (Phase 4)

**3.5 ExtractBox Action Server**
- Navigate to address (call NavigateToAddress)
- Load and execute "insertion" trajectory from YAML (to reach box)
- Toggle electromagnet ON
- Load and execute "extraction" trajectory from YAML (pull box out)
- Call SpawnBox service
- Mark address as "extracted" (red marker)

**3.6 ReturnBox Action Server**
- Navigate to original address
- Load and execute "insertion" trajectory from YAML
- Toggle electromagnet OFF
- Load and execute "extraction" trajectory from YAML
- Call DespawnBox service
- Remove red marker

**3.7 PutBox Action Server**
- Validate target address (empty + width match)
- Navigate to target address
- Generate and execute YZ insertion trajectory
- Toggle electromagnet OFF (release box)
- Generate and execute YZ extraction trajectory
- Call DespawnBox service
- Mark address as occupied

**3.8 MoveBoxToLoad Action Server**
- Composite action: ExtractBox → Navigate to load position → Optional PutBox
- Load position configuration from YAML
- Test with predefined load stations

**Deliverables:**
- Can spawn/despawn boxes via service calls
- Boxes appear at correct addresses in Gazebo
- ExtractBox action extracts box, spawns it, attaches to gripper
- ReturnBox returns box to cabinet

**Testing:**
```bash
# Launch full system
ros2 launch manipulator_control simulation_phase3.launch.py

# Test 1: YZ Trajectory Generator (manual)
ros2 run manipulator_control test_yz_trajectory.py
# Should move gripper in/out of cabinet smoothly without collision

# Test 2: Box spawning
ros2 service call /manipulator/spawn_box \
  manipulator_control/srv/SpawnBox \
  "{box_id: 'test_box', side: 'left', cabinet_num: 1, row: 2, column: 3, num_departments: 10}"
# Verify box appears in Gazebo at correct position

# Test 3: ExtractBox with YZ trajectory
rqt  # Send ExtractBox action goal
# side: left, cabinet: 1, row: 2, col: 3

# Verify:
# 1. Manipulator navigates to address (X, Z)
# 2. Gripper extends using YZ trajectory (smooth, no collision)
# 3. Magnet engages (red sphere marker appears)
# 4. Box attaches to gripper
# 5. Gripper retracts using YZ trajectory
# 6. Red marker appears at address (indicating empty)

# Test 4: PutBox (box relocation)
rqt  # Send PutBox goal
# box_id: test_box
# target: left, cabinet: 2, row: 3, col: 2

# Verify:
# 1. Address validation passes (empty, width match)
# 2. Navigate to target address
# 3. YZ insertion trajectory executes
# 4. Magnet releases
# 5. YZ extraction trajectory executes
# 6. Box despawns, new address marked occupied

# Test 5: MoveBoxToLoad
rqt  # Send MoveBoxToLoad goal
# source: left, cab 1, row 2, col 3
# load_position: load_station_left
# return_to_storage: true
# return: left, cab 1, row 5, col 1 (different address)

# Verify:
# 1. Extracts box from source
# 2. Moves to load position
# 3. Returns to different address (tests PutBox integration)

# Test 6: ReturnBox
rqt  # Send ReturnBox goal (to original address)
# Verify: Box returned, marker removed
```

**Success Criteria:**
- YZ trajectories execute smoothly without cabinet collisions
- ExtractBox successfully extracts 3/3 times with YZ trajectories
- PutBox validates addresses and relocates boxes correctly
- MoveBoxToLoad complete workflow succeeds
- ReturnBox places boxes back correctly
- Address validation prevents invalid placements (wrong width, occupied address)

**Estimated Effort:** 4-5 days (increased for YZ trajectory + new actions)

**Blockers:** Phase 2 complete

---

### Phase 4: Item Picking - State Machine & Departments

**Objective:** Implement picker state machine and department frame generation

**Tasks:**

**4.1 Update Box Spawn Manager - Department Frames**
- Generate department TF frames during box spawn
- Broadcast department transforms at 10 Hz
- Publish department markers (red spheres at centers)

**4.2 ManipulateContainer Action Server**
- Open/close container jaws
- Synchronized motion (software mimic)
- Lift/lower selector for container pickup

**4.3 GetContainer / PlaceContainer Action Servers**
- Complete container retrieval workflow
- Test with predefined container positions

**4.4 PickItem Action Server (State Machine)**
- Implement full state machine with limit switch monitoring
- States: APPROACH → OPEN_JAW → EXTEND → CLOSE → RETRACT → LIFT
- Each state waits for appropriate limit switch
- Navigate to department frame (not box center)

**Deliverables:**
- Boxes spawn with department frames visible in TF tree
- Department markers visible in RViz
- PickItem action executes full state machine
- Can pick from specific departments

**Testing:**
```bash
# Launch system
ros2 launch manipulator_control simulation_phase4.launch.py

# 1. Extract a box
rqt  # ExtractBox: left, cab 1, row 2, col 3

# 2. Verify department frames
ros2 run tf2_ros tf2_echo box_l_1_2_3_link box_l_1_2_3_dept_5
# Should show transform to department 5

# 3. View department markers in RViz
# Should see red spheres and "D1", "D2", etc. labels

# 4. Test container retrieval
rqt  # GetContainer: storage_left_1, width: 0.3

# 5. Test picking
rqt  # PickItem goal:
# - department_num: 5
# - grasp_type: "top"

# Monitor state transitions:
ros2 topic echo /pick_item/_action/feedback

# Monitor limit switches:
rqt_topic  # Watch picker switches change

# Verify:
# - Picker moves to department 5 center
# - Jaw opens (switch triggered)
# - Jaw extends (switch triggered)
# - Jaw closes (switch triggered)
# - Jaw retracts (switch triggered)
# - Picker lifts
```

**Success Criteria:**
- Department frames correctly positioned (verified with tf2_echo)
- Department markers visible and labeled correctly
- PickItem state machine completes without errors
- All state transitions wait for correct switches
- Can pick from different departments (test 3 different dept nums)

**Estimated Effort:** 4-5 days

**Blockers:** Phase 3 complete

---

### Phase 5: High-Level Workflow & Integration

**Objective:** Complete end-to-end pick operation

**Tasks:**

**5.1 PickItemFromStorage Action Server**
- Compose mid-level actions:
  1. GetContainer (if needed)
  2. NavigateToAddress (box address)
  3. ExtractBox
  4. NavigateToAddress (picking position - adjust for picker reach)
  5. PickItem (department)
  6. Place item in container (ManipulateContainer)
  7. ReturnBox

**5.2 Error Handling & Recovery**
- Timeout handling for each sub-action
- Retry logic for transient failures
- Safe abort (return box if extraction succeeded)

**5.3 Complete State Marker Visualization**
- All markers integrated:
  - Magnet engaged (red sphere)
  - Target box address (green box)
  - Extracted addresses (red box)
  - Department markers (red spheres)
  - Container gripped indicator

**5.4 Integration Testing & Documentation**
- Test complete workflow 10 times
- Document failure cases
- Create troubleshooting guide

**Deliverables:**
- PickItemFromStorage action works end-to-end
- Error handling prevents stuck states
- Full visualization suite working
- System runs reliably

**Testing:**
```bash
# Full system launch
ros2 launch manipulator_control simulation.launch.py

# Load RQt perspective with all monitoring tools
rqt --perspective-file config/manipulator_dev.perspective

# Execute complete pick operation
rqt  # PickItemFromStorage action:
# - side: left
# - cabinet: 1
# - row: 2
# - column: 3
# - department: 5
# - container_id: "CONT_A"

# Monitor throughout:
# - Action feedback in rqt_action
# - TF tree in rqt_tf_tree
# - Log messages in rqt_console
# - Markers in RViz

# Verify complete workflow:
# 1. Container retrieved
# 2. Navigate to box address
# 3. Box extracted and spawned
# 4. Navigate to picking position
# 5. Item picked from department 5
# 6. Item placed in container
# 7. Box returned to cabinet
# 8. Success result received
```

**Success Criteria:**
- Complete pick operation succeeds 8/10 times
- Failures gracefully recover (box returned)
- All visualizations correct throughout workflow
- System can be restarted and repeat operation

**Estimated Effort:** 5-6 days

**Blockers:** Phase 4 complete

---

### Phase 6: Level 2 Bridge (Future Work)

**Objective:** Interface with Level 2 storage optimization system

**Tasks:**
1. Define REST API contract with Level 2 team
2. Implement REST bridge node
3. Message translation layer
4. Integration testing with Level 2 system

**Estimated Effort:** 3-5 days (after Level 2 system available)

**Blockers:** Level 2 system must be defined and accessible

---

## Phase Dependencies & Blockers Summary

```
Phase 0 (Prerequisites)
   ↓
Phase 1 (Sim Foundation)
   ↓
Phase 2 (Navigation)
   ↓
Phase 3 (Box Handling)
   ↓
Phase 4 (Item Picking)
   ↓
Phase 5 (Integration)
   ↓
Phase 6 (Level 2 Bridge - Future)
```

**Critical Path:** All phases are sequential dependencies

**Parallel Work Opportunities:**
- Documentation can be written during any phase
- Config file tuning can happen alongside testing
- RViz visualization setup can happen in Phase 1

---

## Testing Strategy Per Phase

### Phase 1-2: Unit Testing
- Individual nodes in isolation
- Mock dependencies
- pytest for Python code

### Phase 3-4: Integration Testing
- Multiple nodes together
- Gazebo simulation required
- Manual testing with RQt tools

### Phase 5: System Testing
- End-to-end workflows
- Reliability testing (10+ runs)
- Performance testing (timing, success rate)

### Phase 6: Acceptance Testing
- Level 2 integration
- Real-world scenarios
- Long-duration testing

---

## Risk Mitigation

**Risk:** Gazebo attachment unreliable
**Mitigation:** Test electromagnet simulator early (Phase 3), have fallback to static joint approach

**Risk:** Limit switch simulation inaccurate
**Mitigation:** Tune switch trigger positions in yaml, test Phase 1 thoroughly before proceeding

**Risk:** TF lookups fail for department frames
**Mitigation:** Validate TF tree structure in Phase 4 before implementing PickItem

**Risk:** Action server timeouts too aggressive
**Mitigation:** Add configurable timeouts in yaml, tune during Phase 5 testing

---

## Development Environment Recommendations

**IDE:** VS Code with Python + ROS extensions
**Debugging:** rqt_console + Python logging, ros2 launch with --log-level debug
**Version Control:** Git branches for each phase
**Testing:** Create bash scripts for common test sequences per phase

---

## ROS2 Jazzy & Gazebo Harmonic Specific Implementation Guide

**Target Versions:**
- **ROS2 Distribution:** Jazzy Jalisco (May 2025)
- **Gazebo Version:** Harmonic (Official pairing for Jazzy)
- **Verification Date:** November 2025

### Key Architecture Components with Jazzy/Harmonic Specifics

#### 1. ros2_control Integration with gz_ros2_control

**Official Documentation:** [gz_ros2_control for Jazzy](https://control.ros.org/jazzy/doc/gz_ros2_control/doc/index.html)

**URDF/Xacro Plugin Setup:**

```xml
<gazebo>
  <plugin filename="gz_ros2_control-system" name="gz_ros2_control::GazeboSimROS2ControlPlugin">
    <parameters>$(find manipulator_description)/config/manipulator_controllers.yaml</parameters>
  </plugin>
</gazebo>
```

**Controller Configuration (Hybrid Architecture):**

```yaml
controller_manager:
  ros__parameters:
    update_rate: 100  # Hz

    # JointTrajectoryController for 7 motion joints (smooth interpolation)
    base_main_frame_joint_controller:
      type: joint_trajectory_controller/JointTrajectoryController
    main_frame_selector_frame_joint_controller:
      type: joint_trajectory_controller/JointTrajectoryController
    # ... repeat for 5 more motion joints

    # ForwardCommandController for 2 container jaws (instant response)
    selector_left_container_jaw_joint_controller:
      type: forward_command_controller/ForwardCommandController
    selector_right_container_jaw_joint_controller:
      type: forward_command_controller/ForwardCommandController

# Trajectory controller parameters
base_main_frame_joint_controller:
  ros__parameters:
    joints: [base_main_frame_joint]
    command_interfaces: [position]
    state_interfaces: [position, velocity]
    interpolation_method: splines
```

**Reference:** [JointTrajectoryController Documentation - Jazzy](https://control.ros.org/jazzy/doc/ros2_controllers/joint_trajectory_controller/doc/userdoc.html)

**Dual-Mode Controller Interface (Python - ROS2 Jazzy):**

```python
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from std_msgs.msg import Float64MultiArray
import yaml
from ament_index_python.packages import get_package_share_directory
import os

class ControllerInterface:
    """
    Utility for commanding both JointTrajectoryController and ForwardCommandController.
    NOT a ROS2 node - requires parent node reference.

    - 7 motion joints: FollowJointTrajectory action (smooth interpolation)
    - 2 container jaws: Float64MultiArray topic (instant response)

    Loads soft limits from manipulator_params.yaml (single source of truth)
    and validates commands before sending.
    """

    TRAJECTORY_JOINTS = frozenset([
        'base_main_frame_joint', 'main_frame_selector_frame_joint',
        'selector_frame_gripper_joint', 'selector_frame_picker_frame_joint',
        'picker_frame_picker_rail_joint', 'picker_rail_picker_base_joint',
        'picker_base_picker_jaw_joint'
    ])

    FORWARD_COMMAND_JOINTS = frozenset([
        'selector_left_container_jaw_joint', 'selector_right_container_jaw_joint'
    ])

    def __init__(self, node):
        self.node = node
        self.logger = node.get_logger()

        # Load soft limits from manipulator_params.yaml (single source of truth)
        self.joint_limits = self._load_joint_limits_from_params()

        # Create action clients for trajectory joints
        self.trajectory_clients = {}
        for joint_name in self.TRAJECTORY_JOINTS:
            action_name = f'/{joint_name}_controller/follow_joint_trajectory'
            self.trajectory_clients[joint_name] = ActionClient(
                node, FollowJointTrajectory, action_name
            )

        # Create publishers for forward command joints (container jaws)
        self.forward_publishers = {}
        for joint_name in self.FORWARD_COMMAND_JOINTS:
            topic = f'/{joint_name}_controller/commands'
            self.forward_publishers[joint_name] = node.create_publisher(
                Float64MultiArray, topic, 10
            )

    def _load_joint_limits_from_params(self):
        """Load soft limits from manipulator_params.yaml"""
        pkg_path = get_package_share_directory('manipulator_description')
        params_file = os.path.join(pkg_path, 'config', 'manipulator_params.yaml')

        with open(params_file, 'r') as f:
            params = yaml.safe_load(f)

        limits = {}
        for assembly_name, assembly in params.items():
            if not isinstance(assembly, dict):
                continue
            for key, value in assembly.items():
                if isinstance(value, dict) and 'safety_controller' in value:
                    sc = value['safety_controller']
                    limits[key] = {'min': sc['soft_lower'], 'max': sc['soft_upper']}
        return limits

    def command_joint(self, joint_name: str, position: float) -> bool:
        """Command single joint to position. Returns False if validation fails."""
        if joint_name not in self.joint_limits:
            self.logger.warning(f"Unknown joint: {joint_name}")
            return False

        limits = self.joint_limits[joint_name]
        if not (limits['min'] <= position <= limits['max']):
            self.logger.warning(f"Position {position} outside limits [{limits['min']}, {limits['max']}]")
            return False

        msg = Float64()
        msg.data = position
        self.publishers[joint_name].publish(msg)
        return True
```

**Reference:** [Example 1: RRBot - Jazzy](https://control.ros.org/jazzy/doc/ros2_control_demos/example_1/doc/userdoc.html)

---

#### 2. TF2 Frame Lookups (ROS2 Jazzy)

**Official Tutorial:** [Writing a TF2 Listener (Python) - Jazzy](https://docs.ros.org/en/jazzy/Tutorials/Intermediate/Tf2/Writing-A-Tf2-Listener-Py.html)

**Address Resolution via TF2 in ROS2 Jazzy:**

```python
import rclpy
from rclpy.node import Node
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
import rclpy.time

class AddressResolver(Node):
    def __init__(self):
        super().__init__('address_resolver')

        # TF2 buffer and listener (Jazzy style)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

    def get_address_coordinates(self, side, cabinet, row, column):
        """
        Resolve address coordinates via TF2 lookup

        Returns: (x, y, z) tuple or None if lookup fails
        """
        side_abbrev = 'l' if side == 'left' else 'r'
        frame_name = f"addr_{side_abbrev}_{cabinet}_{row}_{column}"

        try:
            # Lookup transform with latest available time
            transform = self.tf_buffer.lookup_transform(
                'world',  # Target frame
                frame_name,  # Source frame
                rclpy.time.Time(),  # Latest available
                timeout=rclpy.duration.Duration(seconds=1.0)
            )

            return (
                transform.transform.translation.x,
                transform.transform.translation.y,
                transform.transform.translation.z
            )
        except TransformException as ex:
            self.get_logger().error(f'Could not transform world to {frame_name}: {ex}')
            return None
```

**Additional References:**
- [TF2 Introduction - Jazzy](https://docs.ros.org/en/jazzy/Tutorials/Intermediate/Tf2/Introduction-To-Tf2.html)
- [TF2 Main Index - Jazzy](https://docs.ros.org/en/jazzy/Tutorials/Intermediate/Tf2/Tf2-Main.html)

---

#### 3. Action Servers (ROS2 Jazzy - rclpy)

**Official Tutorial:** [Writing Action Server and Client (Python) - Jazzy](https://docs.ros.org/en/jazzy/Tutorials/Intermediate/Writing-an-Action-Server-Client/Py.html)

**Example Action Server Implementation:**

```python
import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from manipulator_control.action import ExtractBox

class ExtractBoxActionServer(Node):
    def __init__(self):
        super().__init__('extract_box_action_server')

        self._action_server = ActionServer(
            self,
            ExtractBox,
            'extract_box',
            self.execute_callback
        )

    def execute_callback(self, goal_handle):
        """Execute extraction with feedback publishing"""
        self.get_logger().info('Executing extract box...')

        # Publish feedback
        feedback_msg = ExtractBox.Feedback()
        feedback_msg.current_phase = 'navigating'
        feedback_msg.progress_percent = 25
        goal_handle.publish_feedback(feedback_msg)

        # ... perform extraction logic ...

        # Return result
        goal_handle.succeed()

        result = ExtractBox.Result()
        result.success = True
        result.box_extracted = True
        result.box_id = f"box_{goal.side}_{goal.cabinet_num}_{goal.row}_{goal.column}"

        return result

def main(args=None):
    rclpy.init(args=args)
    node = ExtractBoxActionServer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
```

**Reference:** [action_tutorials_py demos - Jazzy](https://github.com/ros2/demos/tree/jazzy/action_tutorials)

---

#### 4. Dynamic Model Spawning (Gazebo Harmonic with ROS2 Jazzy)

**Official Documentation:** [Spawn Gazebo Model from ROS2 - Harmonic](https://gazebosim.org/docs/harmonic/ros2_spawn_model/)

**Using ros_gz_sim create Node:**

```bash
# Spawn model using create node (replaces old spawn_entity.py)
ros2 run ros_gz_sim create \
  -name box_l_1_2_3 \
  -file /path/to/box_model.sdf \
  -x 1.5 -y 2.0 -z 0.8
```

**Programmatic Spawning via Service Call:**

```python
from ros_gz_interfaces.srv import SpawnEntity
import rclpy
from rclpy.node import Node

class BoxSpawnManager(Node):
    def __init__(self):
        super().__init__('box_spawn_manager')

        # Service client for Gazebo entity spawning
        self.spawn_client = self.create_client(
            SpawnEntity,
            '/world/warehouse/create'
        )

    def spawn_box(self, box_id, x, y, z, sdf_content):
        """Spawn box model in Gazebo Harmonic"""
        req = SpawnEntity.Request()
        req.name = box_id
        req.xml = sdf_content  # SDF or URDF string
        req.initial_pose.position.x = x
        req.initial_pose.position.y = y
        req.initial_pose.position.z = z

        future = self.spawn_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)

        return future.result().success
```

**Alternative: Using Launch File:**

```python
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node

def generate_launch_description():
    spawn_box = ExecuteProcess(
        cmd=['ros2', 'run', 'ros_gz_sim', 'create',
             '-name', 'test_box',
             '-file', 'path/to/box.sdf',
             '-x', '1.0', '-y', '2.0', '-z', '0.5'],
        output='screen'
    )

    return LaunchDescription([spawn_box])
```

**Key Changes from Gazebo Classic:**
- `spawn_entity.py` → `ros_gz_sim create` node
- `-entity` argument → `-name` argument
- Service name: `/world/{world_name}/create` instead of `/spawn_entity`

**References:**
- [ROS2 Spawn Model Tutorial - Harmonic](https://gazebosim.org/docs/harmonic/ros2_spawn_model/)
- [Spawn URDF - Harmonic](https://gazebosim.org/docs/harmonic/spawn_urdf/)
- [Migrating from Gazebo Classic - Harmonic](https://gazebosim.org/docs/harmonic/migrating_gazebo_classic_ros2_packages/)

---

#### 5. Contact Sensors for Limit Switches (Gazebo Harmonic)

**Modern Contact Sensor Configuration (SDF format):**

```xml
<sensor name="picker_jaw_contact" type="contact">
  <contact>
    <collision>picker_jaw_collision</collision>
    <topic>/manipulator/contacts/picker_jaw</topic>
  </contact>
  <always_on>true</always_on>
  <update_rate>100</update_rate>
  <visualize>false</visualize>
</sensor>
```

**ROS2 Bridge for Contact Sensor (Jazzy):**

```python
from gz.msgs import Contacts
from ros_gz_bridge import create_bridge
from std_msgs.msg import Bool

class ContactToSwitchConverter(Node):
    """Convert Gazebo contact sensor to ROS2 bool topic"""

    def __init__(self):
        super().__init__('contact_to_switch')

        # Subscribe to Gazebo contact topic (via ros_gz_bridge)
        self.create_subscription(
            Contacts,
            '/manipulator/contacts/picker_jaw',
            self.contact_callback,
            10
        )

        # Publish ROS2 boolean switch state
        self.switch_pub = self.create_publisher(
            Bool,
            '/manipulator/end_switches/picker_jaw_closed',
            10
        )

    def contact_callback(self, msg):
        """Convert contact to boolean switch state"""
        has_contact = len(msg.contact) > 0

        switch_msg = Bool()
        switch_msg.data = has_contact
        self.switch_pub.publish(switch_msg)
```

**Reference:** [Gazebo Sim Plugins and Sensors - Medium Tutorial](https://medium.com/@alitekes1/gazebo-sim-plugin-and-sensors-for-acquire-data-from-simulation-environment-681d8e2ad853)

---

#### 6. Electromagnet Simulation with Detachable Joints (Gazebo Harmonic)

**Detachable Joint System Plugin (Gazebo Harmonic Approach):**

The modern approach uses the detachable joint system, which allows attach/detach operations via Gazebo Transport topics.

**SDF Configuration:**

```xml
<gazebo>
  <plugin filename="gz-sim-detachable-joint-system" name="gz::sim::systems::DetachableJoint">
    <parent_link>gripper_magnet_link</parent_link>
    <child_model>box_l_1_2_3</child_model>
    <child_link>box_link</child_link>
    <topic>/manipulator/electromagnet/attach_detach</topic>
  </plugin>
</gazebo>
```

**ROS2 Service for Electromagnet Control:**

```python
from std_srvs.srv import SetBool
import rclpy

class ElectromagnetSimulator(Node):
    def __init__(self):
        super().__init__('electromagnet_simulator')

        # Service for ROS2 control
        self.create_service(
            SetBool,
            '/manipulator/electromagnet/toggle',
            self.toggle_callback
        )

        # Publisher to Gazebo detachable joint topic
        self.attach_pub = self.create_publisher(
            # Gazebo Transport message type
            gz_msgs.StringMsg,
            '/manipulator/electromagnet/attach_detach',
            10
        )

    def toggle_callback(self, request, response):
        """Toggle electromagnet (attach/detach)"""
        msg = gz_msgs.StringMsg()
        msg.data = 'attach' if request.data else 'detach'

        self.attach_pub.publish(msg)

        response.success = True
        response.message = f"Electromagnet {'engaged' if request.data else 'released'}"
        return response
```

**Alternative: Using Gazebo Services Directly:**

```python
from ros_gz_interfaces.srv import SetEntityPose

# Use Gazebo's built-in attach/detach via fixed joint creation
# This is more reliable than physics-based friction
```

**References:**
- [Gazebo Detachable Joint System](https://gazebosim.org/docs/harmonic/ros2_integration/)
- [Clearpath Manipulation Tutorial](https://docs.clearpathrobotics.com/docs/ros/tutorials/manipulation/gazebo/)

---

#### 7. ros_gz_bridge for ROS2-Gazebo Communication (Jazzy/Harmonic)

**Official Integration:** [ROS2 Gazebo Integration](https://gazebosim.org/docs/latest/ros2_integration/)

**Installing ros_gz for Jazzy:**

```bash
sudo apt install ros-jazzy-ros-gz
```

**Bridge Configuration:**

```yaml
# config/ros_gz_bridge.yaml
- topic_name: "/joint_states"
  ros_type: "sensor_msgs/msg/JointState"
  gz_type: "gz.msgs.Model"
  direction: GZ_TO_ROS

- topic_name: "/clock"
  ros_type: "rosgraph_msgs/msg/Clock"
  gz_type: "gz.msgs.Clock"
  direction: GZ_TO_ROS
```

**Launching Bridge:**

```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{
            'config_file': 'path/to/ros_gz_bridge.yaml'
        }],
        output='screen'
    )

    return LaunchDescription([bridge])
```

---

#### 8. Complete Launch File Example (ROS2 Jazzy + Gazebo Harmonic)

```python
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_description = get_package_share_directory('manipulator_description')
    pkg_control = get_package_share_directory('manipulator_control')

    # Launch Gazebo Harmonic
    gazebo = ExecuteProcess(
        cmd=['gz', 'sim', '-r', 'warehouse.sdf'],
        output='screen'
    )

    # Spawn robot with ros_gz_sim
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'manipulator',
            '-file', os.path.join(pkg_description, 'urdf', 'manipulator.urdf'),
            '-x', '2.0',
            '-y', '0.0',
            '-z', '0.0'
        ],
        output='screen'
    )

    # Load ros2_control controllers
    load_controllers = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'base_main_frame_joint_controller',
            'main_frame_selector_frame_joint_controller',
            # ... all 9 controllers
        ],
        output='screen'
    )

    # Launch ros_gz_bridge
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{
            'config_file': os.path.join(pkg_control, 'config', 'ros_gz_bridge.yaml')
        }],
        output='screen'
    )

    # Action servers
    extract_box_server = Node(
        package='manipulator_control',
        executable='extract_box_action_server',
        output='screen'
    )

    return LaunchDescription([
        gazebo,
        spawn_robot,
        load_controllers,
        bridge,
        extract_box_server
    ])
```

---

### Version Verification and Compatibility

**ROS2 Jazzy + Gazebo Harmonic Compatibility Matrix:**

| Component | Version | Verification Date | Status |
|-----------|---------|-------------------|--------|
| ROS2 Distribution | Jazzy Jalisco | Nov 2025 | ✅ LTS |
| Gazebo | Harmonic | Nov 2025 | ✅ Official pairing |
| gz_ros2_control | 2.x (Jazzy) | Nov 2025 | ✅ Stable |
| ros_gz | Jazzy branch | Nov 2025 | ✅ Active |
| ros2_control | 4.x (Jazzy) | Nov 2025 | ✅ Stable |

**Installation Commands (Ubuntu 24.04):**

```bash
# Install ROS2 Jazzy
sudo apt install ros-jazzy-desktop

# Install Gazebo Harmonic
sudo apt install gz-harmonic

# Install ros_gz for Jazzy/Harmonic integration
sudo apt install ros-jazzy-ros-gz

# Install ros2_control packages
sudo apt install ros-jazzy-ros2-control ros-jazzy-ros2-controllers

# Install gz_ros2_control
sudo apt install ros-jazzy-gz-ros2-control
```

**References:**
- [ROS2 Jazzy Release Notes](https://docs.ros.org/en/rolling/Releases/Release-Jazzy-Jalisco.html)
- [Installing Gazebo with ROS](https://gazebosim.org/docs/latest/ros_installation/)
- [gz_ros2_control Installation](https://control.ros.org/jazzy/doc/gz_ros2_control/doc/index.html)

---

### Epic 7: Hardware Interface for Real Robot Control

**Full Documentation:** `docs/architecture-epic7-hardware-interface.md`

**Overview:** Epic 7 implements a ros2_control hardware interface plugin that enables real hardware control via Modbus RTU, allowing hot-swappable transition between simulation and real hardware.

**Key Architectural Decisions:**

| Decision | Choice | Rationale |
|----------|--------|-----------|
| Implementation Language | **Python** (proof-of-concept) | Matches existing `examples/modbus_driver/`, faster iteration |
| Hardware Interface Type | `SystemInterface` | Controls 7 joints as coordinated system |
| Communication Protocol | Modbus RTU | Existing working driver, standard industrial protocol |
| Container Jaws | Mock interface | No Modbus; future discrete I/O epic |

**Hardware Plugin Architecture:**

```
┌─────────────────────────────────────┐
│  controller_manager (100 Hz)        │
│  - 7 JointTrajectoryControllers     │
│  - 2 ForwardCommandControllers      │
└──────────────┬──────────────────────┘
               │
               ▼
┌─────────────────────────────────────┐
│  ModbusHardwareInterface (Python)   │
│  - export_state_interfaces()        │
│  - export_command_interfaces()      │
│  - read() / write() at 100 Hz       │
└──────────────┬──────────────────────┘
               │
               ▼
┌─────────────────────────────────────┐
│  ModbusDriver (minimalmodbus)       │
│  - FC4: Read Input Registers        │
│  - FC6: Write Holding Registers     │
└──────────────┬──────────────────────┘
               │
               ▼
        /dev/ttyACM0 (Serial RS-485)
               │
    ┌──────────┼──────────┐
    ▼          ▼          ▼
 Slave 1    Slave 2    Slave 3
 (X, Z)     (Y, B)     (A, C, D)
```

**URDF Hardware Switching:**

```xml
<!-- In ros2_control.xacro -->
<xacro:if value="${sim}">
  <plugin>gz_ros2_control/GazeboSimSystem</plugin>
</xacro:if>
<xacro:unless value="${sim}">
  <plugin>manipulator_hardware/ModbusHardwareInterface</plugin>
  <param name="config_file">$(find manipulator_hardware)/config/hardware_config.yaml</param>
</xacro:unless>
```

**Launch Usage:**

```bash
# Simulation (default)
ros2 launch manipulator_control manipulator_simulation.launch.py

# Real hardware
ros2 launch manipulator_control manipulator_simulation.launch.py use_sim_time:=false
```

**Joint-to-Hardware Mapping:**

| Joint | Slave | Axis | Position Reg | Command Reg |
|-------|-------|------|--------------|-------------|
| base_main_frame_joint | 1 | X | 1003 | 2999 |
| main_frame_selector_frame_joint | 1 | Z | 1010 | 3008 |
| selector_frame_gripper_joint | 2 | Y | 1003 | 2999 |
| selector_frame_picker_frame_joint | 3 | A | 1003 | 2999 |
| picker_frame_picker_rail_joint | 2 | B | 1010 | 3008 |
| picker_rail_picker_base_joint | 3 | C | 1010 | 3008 |
| picker_base_picker_jaw_joint | 3 | D | 1017 | 3017 |

**Reference Implementation:** `examples/modbus_driver/` (working Python Modbus driver)

**Key ROS2 Control Documentation:**
- [Writing a Hardware Component (Jazzy)](https://control.ros.org/jazzy/doc/ros2_control/hardware_interface/doc/writing_new_hardware_component.html)
- [Hardware Interface Types](https://control.ros.org/jazzy/doc/ros2_control/hardware_interface/doc/hardware_interface_types_userdoc.html)

---

### Summary of Jazzy/Harmonic Specific Changes

**Key Updates from Older Versions:**

1. **Gazebo Command:** `gazebo` → `gz sim`
2. **Spawning:** `spawn_entity.py` → `ros_gz_sim create`
3. **Plugin Names:** `libgazebo_ros_*` → `gz-sim-*`
4. **Bridge Package:** `ros_ign_bridge` → `ros_gz_bridge`
5. **World Service:** `/spawn_entity` → `/world/{world_name}/create`
6. **Controller Spawner:** Uses `controller_manager/spawner` (same)
7. **TF2 API:** Consistent with Jazzy (no major changes from Humble/Iron)
8. **Action API:** rclpy.action module (consistent across ROS2)

**Breaking Changes to Watch:**
- Gazebo Classic (gazebo11) is EOL - must use Gazebo Sim (gz-sim)
- Contact sensor plugin names changed
- SDF format preferred over URDF for Gazebo-specific features
- ros_gz_bridge has limited message type support (check compatibility)

---

## References
- `/ros2_ws/src/manipulator_description/urdf/storage_system.urdf.xacro` - Cabinet rows
- `/ros2_ws/src/manipulator_description/urdf/cabinet_row.urdf.xacro` - Cabinet array
- `/ros2_ws/src/manipulator_description/urdf/cabinet.urdf.xacro` - Single cabinet
- `/ros2_ws/src/manipulator_description/urdf/box_placement_frames.urdf.xacro` - Address frames
- `/ros2_ws/src/manipulator_description/config/storage_params.yaml` - Dimensions and config

### Controllers and Joint Limits
- `/ros2_ws/src/manipulator_description/config/manipulator_params.yaml` - **SINGLE SOURCE** for joint limits (hard + soft)
- `/ros2_ws/src/manipulator_description/config/manipulator_controllers.yaml` - Hybrid controllers (7 trajectory + 2 forward)
- `/ros2_ws/src/manipulator_description/urdf/manipulator/ros2_control.xacro` - Hardware interface (loads limits from manipulator_params.yaml)

### Gazebo Simulation
- [Gazebo Classic End of Life (Jan 2025)](https://classic.gazebosim.org/tutorials?tut=ros_gzplugins)
- [gz_ros2_control Documentation](https://control.ros.org/humble/doc/gz_ros2_control/doc/index.html)
- [Gazebo Contact Sensors](https://answers.ros.org/question/11606/gazebo-magnet-simulation/)
- [Gazebo Magnet Simulation Approaches](https://github.com/topics/gripper)

---

## Summary of Architecture Features

### Core Corrections from v1.0

✅ **Controllers:** Hybrid architecture - 7 JointTrajectoryControllers (motion joints) + 2 ForwardCommandControllers (container jaws)
✅ **Addressing:** (side, cabinet, row, column, department) - resolved via TF frames
✅ **No Address Storage:** Use TF lookups to existing URDF frames
✅ **Picker:** State machine with limit switches (all axes have 2 switches = 18 total)
✅ **Simulation:** Virtual limit switches + electromagnet simulator nodes needed
✅ **Box Spawning:** Dynamic Gazebo spawning with department frame generation
✅ **Container Jaws:** Software mimic only for simulation (hardware is mechanically mimicked)
✅ **Container Retrieval:** GetContainer + PlaceContainer actions with lift/lower
✅ **Markers:** Visual state indication (magnet, target address, extracted address, departments)
✅ **Config:** Use existing YAML files, no duplication
✅ **GUI:** Standard RQt tools first, custom plugin only if needed
✅ **No Item DB:** Level 2 provides item positions, Level 3 just executes

### Additional Features Added

✅ **Box Relocation (PutBox):** Place extracted box in any empty address with matching width
✅ **Box Loading (MoveBoxToLoad):** Complete workflow for external access with optional return to different address
✅ **YZ Trajectory Generation:** Safe cabinet insertion/extraction trajectories avoiding frame collisions
✅ **Address Validation:** Automatic checks for empty addresses and box width compatibility
✅ **Load Positions:** Configurable external access points for box loading/inspection

### Complete Action Set

**High-Level Warehouse Operations:**
- PickItemFromStorage - Complete item retrieval workflow
- MoveBoxToLoad - Box loading with optional relocation

**Mid-Level Box Operations:**
- ExtractBox - Remove box from cabinet (with YZ trajectory)
- ReturnBox - Return box to original address (with YZ trajectory)
- PutBox - Place box in different address (with validation + YZ trajectory)

**Mid-Level Container Operations:**
- GetContainer - Retrieve container with lift
- PlaceContainer - Place container with lower
- ManipulateContainer - Open/close jaws

**Mid-Level Navigation:**
- NavigateToAddress - Move to cabinet address (X, Z positioning)

**Mid-Level Picking:**
- PickItem - State machine with limit switches

**Low-Level Control:**
- MoveJoint - Single joint position command
- MoveJointGroup - Coordinated multi-joint motion

### Key Utilities

- **YZTrajectoryGenerator** - Safe insertion/extraction path planning
- **AddressResolver** - TF frame lookup for coordinates
- **AddressValidator** - Empty check + width compatibility
- **ControllerInterface** - Dual-mode joint command (trajectory actions + forward topics)
- **VirtualLimitSwitches** - Simulation of 18 end switches

### Configuration Files

```
manipulator_description/config/ (PRIMARY - DO NOT DUPLICATE):
  - manipulator_params.yaml    → SINGLE SOURCE for joint limits (hard + soft), physical params
  - manipulator_controllers.yaml → Controller names, types, joint mappings
  - storage_params.yaml        → Cabinet dimensions, box configs, department layouts

manipulator_control/config/ (SECONDARY - grouped by function):
  - limit_switches.yaml        → 18 switch trigger positions (Story 2.1)
  - action_servers.yaml        → Action timeouts, tolerances, speeds (Stories 2.3+)
  - kinematic_chains.yaml      → Joint group definitions (Story 2.5)
  - visualization.yaml         → All visualization markers (Story 2.4+)
  - trajectory.yaml            → YZ trajectory speeds, margins (Story 4A.1)
  - electromagnet.yaml         → Magnet simulation parameters (Story 4A.2)
  - box_spawner.yaml           → Box spawning, TF rates (Story 4A.3)
  - load_positions.yaml        → External load stations (Story 4B.4)
  - container_storage.yaml     → Container locations/slots (Story 5.4)
  - pick_item_states.yaml      → Picker state machine (Story 5.5)
  - error_handling.yaml        → Error codes, recovery (Story 6.2)
  - testing.yaml               → Test suite, profiling (Story 6.4+)
```

**Configuration Reuse Policy:**
- Load existing configs, create ONLY node-specific parameters
- Reference other configs by key names, not duplicate values
- Stories ADD sections to existing files, not new files where possible

### Development Phases

**Phase 0:** Package setup (0.5-1 day)
**Phase 1:** Simulation foundation + joint control (2-3 days)
**Phase 2:** Navigation + addressing (2-3 days)
**Phase 3:** Box handling + YZ trajectories (4-5 days) ← EXPANDED
**Phase 4:** Item picking + departments (4-5 days)
**Phase 5:** Integration + testing (5-6 days)
**Phase 6:** Level 2 bridge (future)
**Phase 7:** Hardware Interface (INDEPENDENT - can develop in parallel with Phases 1-6)

**Total: ~18-23 days (Phases 0-6) + Phase 7 (parallel)**

### Epic 7: Hardware Interface (Independent Development Track)

✅ **Hardware Interface:** Python ros2_control SystemInterface with Modbus RTU
✅ **Hot-Swappable:** URDF `sim` parameter switches between simulation and hardware
✅ **Configuration-Driven:** YAML-based joint-to-register mapping
✅ **Reference Implementation:** `examples/modbus_driver/` working Python driver
✅ **Full Documentation:** `docs/architecture-epic7-hardware-interface.md`

---

**This v2.2 document reflects the complete system architecture with all box manipulation capabilities, YZ trajectory planning, hardware interface architecture, and phased development roadmap ready for implementation.**
