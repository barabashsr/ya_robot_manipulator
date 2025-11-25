# Epic Technical Specification: Simulation Foundation & Joint Control

Date: 2025-11-25
Author: SM Agent (Bob)
Epic ID: epic-2
Status: Draft

---

## Overview

Epic 2 establishes the simulation infrastructure for limit switches, electromagnets, and basic joint control actions. This epic enables testing of all subsequent functionality in Gazebo without hardware dependencies.

The epic delivers a complete simulation environment with observable joint control, state visualization, and foundational action servers ready for mid-level action composition in Epic 3+.

## Objectives and Scope

**In Scope:**
- Virtual limit switch simulation for all 9 joints (18 switches total)
- Controller interface utility for commanding individual ForwardCommandControllers
- MoveJoint action server for single joint control
- MoveJointGroup action server for coordinated multi-joint motion
- State marker publisher for RViz visualization
- Test scripts and RQt documentation

**Out of Scope:**
- Electromagnet simulation (deferred - uses ToggleElectromagnet service from Epic 1)
- Address navigation (Epic 3)
- Box extraction operations (Epic 4A)
- Hardware interface layer (Growth phase)

## System Architecture Alignment

**Architecture References:**
- Lines 13-28: Individual ForwardCommandControllers per joint (NOT JointTrajectoryController)
- Lines 247-306: Virtual limit switch implementation pattern
- Lines 314-441: Electromagnet simulation approach
- Lines 443-466: Container jaw software mimic

**Key Constraints:**
- Individual ForwardCommandControllers - each joint has `/[joint_name]_controller/commands` topic (plural)
- Position commands via Float64MultiArray messages (single element array)
- Joint limits from ros2_control.xacro
- ROS2 Jazzy + Gazebo Harmonic compatibility

**Package Dependencies:**
- manipulator_control (created in Epic 1)
- manipulator_description (URDF, ros2_control config)
- rclpy, std_msgs, sensor_msgs, geometry_msgs
- visualization_msgs (for markers)

## Unified Launch File Pattern

**CRITICAL:** All Epic 2 stories that create nodes MUST update `manipulator_simulation.launch.py`.

### Launch File Location
`ros2_ws/src/manipulator_control/launch/manipulator_simulation.launch.py`

### Simulation vs Hardware Separation

Use `use_sim_time` argument to control node launching:

| Category | Nodes | Launch Condition |
|----------|-------|------------------|
| **Simulation-Only** | virtual_limit_switches | `IfCondition(use_sim_time)` |
| **Hardware-Only** | (future: GPIO interfaces) | `UnlessCondition(use_sim_time)` |
| **Common** | MoveJoint server, MoveJointGroup server, state_marker_publisher | Always (no condition) |

### Story Launch File Updates

| Story | Node to Add | Category | Delay |
|-------|-------------|----------|-------|
| 2.1 | virtual_limit_switches | Simulation-Only | 3s |
| 2.2 | (utility, no node) | N/A | N/A |
| 2.3 | move_joint_server | Common | 3s |
| 2.4 | state_marker_publisher | Common | 3s |
| 2.5 | move_joint_group_server | Common | 3s |
| 2.6 | (test/docs, no node) | N/A | N/A |

### Launch File Pattern

```python
from launch.conditions import IfCondition, UnlessCondition

# Declare use_sim_time argument
use_sim_time_arg = DeclareLaunchArgument(
    'use_sim_time',
    default_value='true',
    description='Use simulation time (Gazebo) or real hardware'
)
use_sim_time = LaunchConfiguration('use_sim_time')

# Simulation-only node
virtual_limit_switches_node = TimerAction(
    period=3.0,
    actions=[
        Node(
            package='manipulator_control',
            executable='virtual_limit_switches.py',
            name='virtual_limit_switches',
            parameters=[{'use_sim_time': use_sim_time}],
            condition=IfCondition(use_sim_time)  # Only in simulation
        )
    ]
)

# Common node (no condition)
move_joint_server_node = TimerAction(
    period=3.0,
    actions=[
        Node(
            package='manipulator_control',
            executable='move_joint_server.py',
            name='move_joint_server',
            parameters=[{'use_sim_time': use_sim_time}]
            # No condition - runs in both sim and hardware
        )
    ]
)
```

## Detailed Design

### Services and Modules

**Package:** `manipulator_control`

| Module | File | Responsibility |
|--------|------|----------------|
| VirtualLimitSwitchNode | `src/virtual_limit_switches.py` | Simulate 18 limit switches based on joint positions |
| ControllerInterface | `src/controller_interface.py` | Utility for commanding ForwardCommandControllers |
| MoveJointActionServer | `src/move_joint_server.py` | Action server for single joint control |
| MoveJointGroupActionServer | `src/move_joint_group_server.py` | Action server for coordinated joint groups |
| StateMarkerPublisher | `src/state_marker_publisher.py` | RViz marker visualization |

### Configuration Files

### CRITICAL: Configuration Reuse Policy

**DO NOT duplicate hardware configuration.** All Epic 2 nodes MUST load existing configurations:

| Configuration | Source File (Single Source of Truth) | Used By |
|--------------|--------------------------------------|---------|
| Joint limits | `manipulator_description/urdf/manipulator/ros2_control.xacro` | ControllerInterface validation |
| Controller names | `manipulator_description/config/manipulator_controllers.yaml` | ControllerInterface topic discovery |
| Switch positions | `manipulator_control/config/limit_switches.yaml` | VirtualLimitSwitchNode |
| Joint groups | `manipulator_control/config/kinematic_chains.yaml` (NEW) | MoveJointGroup action server |
| Action timeouts | `manipulator_control/config/action_servers.yaml` (NEW) | MoveJoint, MoveJointGroup |

**Node-specific config only:** Each new node creates config ONLY for parameters unique to that node (e.g., publish rates, timeouts), NOT for hardware definitions.

---

**config/limit_switches.yaml (VALIDATED - actual implementation):**

**CRITICAL:** Switch names use semantic naming that reflects physical function:
- Gripper: `gripper_left` / `gripper_right` (NOT extended/retracted - reflects cabinet direction)
- Picker extension: `picker_extended` / `picker_retracted` (NOT picker_base_min/max)

```yaml
# Virtual limit switch configuration
# Source: ros2_ws/src/manipulator_control/config/limit_switches.yaml
switches:
  # Base assembly (X-axis rail)
  base_main_frame_min:
    joint: "base_main_frame_joint"
    trigger_position: 0.0  # Near joint min (0.0)
    trigger_tolerance: 0.01
  base_main_frame_max:
    joint: "base_main_frame_joint"
    trigger_position: 3.9  # Near joint max (4.0 from ros2_control.xacro)
    trigger_tolerance: 0.01

  # Selector frame (Z-axis vertical: -0.01 to 1.5m per ros2_control.xacro)
  selector_frame_min:
    joint: "main_frame_selector_frame_joint"
    trigger_position: 0.005
    trigger_tolerance: 0.01
  selector_frame_max:
    joint: "main_frame_selector_frame_joint"
    trigger_position: 1.45  # Joint max is 1.5, switch at 1.45 within range
    trigger_tolerance: 0.01

  # Gripper Y-axis - LEFT/RIGHT semantics (NOT extend/retract!)
  gripper_left:
    joint: "selector_frame_gripper_joint"
    trigger_position: 0.39  # +Y toward LEFT cabinets
    trigger_tolerance: 0.01
  gripper_right:
    joint: "selector_frame_gripper_joint"
    trigger_position: -0.39  # -Y toward RIGHT cabinets
    trigger_tolerance: 0.01

  # Picker frame (Z-axis vertical)
  picker_frame_min:
    joint: "selector_frame_picker_frame_joint"
    trigger_position: 0.005
    trigger_tolerance: 0.01
  picker_frame_max:
    joint: "selector_frame_picker_frame_joint"
    trigger_position: 0.295
    trigger_tolerance: 0.01

  # Picker rail (Y-axis along departments)
  picker_rail_min:
    joint: "picker_frame_picker_rail_joint"
    trigger_position: -0.29
    trigger_tolerance: 0.01
  picker_rail_max:
    joint: "picker_frame_picker_rail_joint"
    trigger_position: 0.29
    trigger_tolerance: 0.01

  # Picker extension (X-axis) - retracted/extended semantics
  picker_retracted:
    joint: "picker_rail_picker_base_joint"
    trigger_position: 0.01  # Home position
    trigger_tolerance: 0.01
  picker_extended:
    joint: "picker_rail_picker_base_joint"
    trigger_position: 0.24  # Extended over container
    trigger_tolerance: 0.01

  # Picker jaw (X-axis grasp)
  picker_jaw_closed:
    joint: "picker_base_picker_jaw_joint"
    trigger_position: 0.01  # Closed/gripping
    trigger_tolerance: 0.005
  picker_jaw_opened:
    joint: "picker_base_picker_jaw_joint"
    trigger_position: 0.19  # Open
    trigger_tolerance: 0.005

  # Container jaws (left)
  container_left_min:
    joint: "selector_left_container_jaw_joint"
    trigger_position: -0.095
    trigger_tolerance: 0.01
  container_left_max:
    joint: "selector_left_container_jaw_joint"
    trigger_position: 0.095
    trigger_tolerance: 0.01

  # Container jaws (right)
  container_right_min:
    joint: "selector_right_container_jaw_joint"
    trigger_position: -0.095
    trigger_tolerance: 0.01
  container_right_max:
    joint: "selector_right_container_jaw_joint"
    trigger_position: 0.095
    trigger_tolerance: 0.01

publish_rate: 10.0  # Hz
```

**config/kinematic_chains.yaml:**
```yaml
# Joint group definitions for MoveJointGroup action
joint_groups:
  navigation:
    joints:
      - base_main_frame_joint
      - main_frame_selector_frame_joint
    description: "X-Z positioning for address navigation"
    default_velocity: 0.5

  gripper:
    joints:
      - selector_frame_gripper_joint
      - main_frame_selector_frame_joint
    description: "Gripper Y-axis with Z adjustment"
    default_velocity: 0.3

  picker:
    joints:
      - selector_frame_picker_frame_joint
      - picker_frame_picker_rail_joint
      - picker_rail_picker_base_joint
      - picker_base_picker_jaw_joint
    description: "All picker joints for item picking"
    default_velocity: 0.2

  container:
    joints:
      - selector_left_container_jaw_joint
      - selector_right_container_jaw_joint
    description: "Container jaws (synchronized mimic)"
    default_velocity: 0.1
    mimic_mode: true  # left = -width/2, right = +width/2
```

**config/action_servers.yaml:**
```yaml
# Action server configuration
move_joint:
  timeout_sec: 30.0
  position_tolerance: 0.01
  feedback_rate: 10.0  # Hz

move_joint_group:
  timeout_sec: 30.0
  position_tolerance: 0.01
  feedback_rate: 10.0
  coordination_timeout: 1.0  # All joints must arrive within this window
```

### APIs and Interfaces

**Controller Interface Utility:**
```python
class ControllerInterface:
    """Utility for commanding individual ForwardCommandControllers"""

    def __init__(self, node: Node):
        """Initialize with reference to ROS2 node"""

    def command_joint(self, joint_name: str, position: float) -> bool:
        """
        Command a single joint to target position.
        Returns False if joint unknown or position out of limits.
        Publishes Float64 to /[joint_name]_controller/command
        """

    def command_joint_group(self, joint_names: List[str], positions: List[float]) -> bool:
        """
        Command multiple joints simultaneously.
        Returns False if any joint fails validation.
        """

    def get_joint_position(self, joint_name: str) -> Optional[float]:
        """
        Get current joint position from /joint_states.
        Returns None if joint not found.
        """

    def get_joint_limits(self, joint_name: str) -> Tuple[float, float]:
        """Return (min_limit, max_limit) for joint"""
```

**Controller Topic Mapping (CORRECTED - /commands plural, Float64MultiArray):**
| Joint Name | Controller Topic |
|------------|------------------|
| base_main_frame_joint | /base_main_frame_joint_controller/commands |
| main_frame_selector_frame_joint | /main_frame_selector_frame_joint_controller/commands |
| selector_left_container_jaw_joint | /selector_left_container_jaw_joint_controller/commands |
| selector_right_container_jaw_joint | /selector_right_container_jaw_joint_controller/commands |
| selector_frame_gripper_joint | /selector_frame_gripper_joint_controller/commands |
| selector_frame_picker_frame_joint | /selector_frame_picker_frame_joint_controller/commands |
| picker_frame_picker_rail_joint | /picker_frame_picker_rail_joint_controller/commands |
| picker_rail_picker_base_joint | /picker_rail_picker_base_joint_controller/commands |
| picker_base_picker_jaw_joint | /picker_base_picker_jaw_joint_controller/commands |

**Limit Switch Topics (CORRECTED):**

**CRITICAL:** Use semantic switch names that reflect physical function:

```
/manipulator/end_switches/base_main_frame_min        (std_msgs/Bool) X=0.01
/manipulator/end_switches/base_main_frame_max        (std_msgs/Bool) X=3.9
/manipulator/end_switches/selector_frame_min         (std_msgs/Bool) Z=0.005
/manipulator/end_switches/selector_frame_max         (std_msgs/Bool) Z=1.90
/manipulator/end_switches/gripper_left               (std_msgs/Bool) Y=+0.39 toward LEFT cabinets
/manipulator/end_switches/gripper_right              (std_msgs/Bool) Y=-0.39 toward RIGHT cabinets
/manipulator/end_switches/picker_frame_min           (std_msgs/Bool) Z=0.005
/manipulator/end_switches/picker_frame_max           (std_msgs/Bool) Z=0.295
/manipulator/end_switches/picker_rail_min            (std_msgs/Bool) Y=-0.29
/manipulator/end_switches/picker_rail_max            (std_msgs/Bool) Y=+0.29
/manipulator/end_switches/picker_retracted           (std_msgs/Bool) X=0.01 home position
/manipulator/end_switches/picker_extended            (std_msgs/Bool) X=0.24 over container
/manipulator/end_switches/picker_jaw_opened          (std_msgs/Bool) X=0.19
/manipulator/end_switches/picker_jaw_closed          (std_msgs/Bool) X=0.01
/manipulator/end_switches/container_left_min         (std_msgs/Bool)
/manipulator/end_switches/container_left_max         (std_msgs/Bool)
/manipulator/end_switches/container_right_min        (std_msgs/Bool)
/manipulator/end_switches/container_right_max        (std_msgs/Bool)
```

### Workflows and Sequencing

**Epic 2 Implementation Sequence:**

```
Story 2.1: Virtual Limit Switch Simulation
├─ Create config/limit_switches.yaml
├─ Implement VirtualLimitSwitchNode
├─ Subscribe to /joint_states
├─ Publish 18 switch states at 10 Hz
└─ Verify: rostopic echo shows state changes

Story 2.2: Controller Interface Utility
├─ Create src/controller_interface.py
├─ Implement command_joint(), command_joint_group()
├─ Implement get_joint_position()
├─ Load joint limits from ros2_control.xacro
└─ Verify: Unit test joint commands

Story 2.3: MoveJoint Action Server
├─ Create src/move_joint_server.py
├─ Implement action server using ControllerInterface
├─ Publish feedback at 10 Hz
├─ Handle preemption and timeout
└─ Verify: rqt_action sends goal, joint moves

Story 2.4: State Marker Publisher
├─ Create src/state_marker_publisher.py
├─ Implement magnet engaged marker (red sphere)
├─ Implement target address marker (green cube)
├─ Implement extracted address markers (red cubes)
└─ Verify: Markers visible in RViz

Story 2.5: MoveJointGroup Action Server
├─ Create config/kinematic_chains.yaml
├─ Create src/move_joint_group_server.py
├─ Implement group motion with coordination
├─ Implement container jaw mimic mode
└─ Verify: Navigation group moves both joints

Story 2.6: Test Script and Documentation
├─ Create test/test_epic2_joint_control.py
├─ Create docs/TESTING_WITH_RQT.md
├─ Run all automated tests
└─ Verify: 90%+ tests pass
```

## Non-Functional Requirements

### Performance

- Limit switch publish rate: 10 Hz minimum
- Action feedback rate: 10 Hz minimum
- Joint position tolerance: ±0.01m (NFR-002)
- Action timeout: 30 seconds default
- Marker update rate: 10 Hz

### Reliability

- MoveJoint action supports preemption (cancellation)
- Invalid commands return failure without crashing
- Joint limits validated before commanding
- Coordinated motion timeout for MoveJointGroup

### Observability

- All limit switch states visible via rostopic
- State markers visible in RViz
- Action feedback provides progress percentage
- Console logging for debugging

## Dependencies and Integrations

**ROS2 Dependencies:**
```python
# Python imports
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from std_msgs.msg import Bool, Float64
from sensor_msgs.msg import JointState
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from manipulator_control.action import MoveJoint, MoveJointGroup
```

**Package Dependencies:**
- manipulator_control (Epic 1 interfaces)
- manipulator_description (URDF, config files)

**Integration Points:**
- `/joint_states` topic (from joint_state_broadcaster)
- Controller command topics (from ForwardCommandControllers)
- TF frames (from robot_state_publisher)

## Acceptance Criteria (Authoritative)

**AC-2.1: Virtual Limit Switches**
- [ ] 18 limit switch topics published at 10 Hz
- [ ] Switch states transition when joint reaches trigger threshold
- [ ] Configuration loaded from limit_switches.yaml

**AC-2.2: Controller Interface**
- [ ] command_joint() publishes to correct controller topic
- [ ] command_joint_group() commands multiple joints simultaneously
- [ ] get_joint_position() returns current position from /joint_states
- [ ] Invalid positions rejected with logged warning

**AC-2.3: MoveJoint Action Server**
- [ ] Action accepts goal, moves joint, returns result
- [ ] Feedback published at 10 Hz with progress
- [ ] Preemption supported (cancel during motion)
- [ ] Timeout after 30 seconds

**AC-2.4: State Marker Publisher**
- [ ] Magnet marker visible when engaged
- [ ] Target address marker visible during navigation
- [ ] Markers visible in RViz MarkerArray display

**AC-2.5: MoveJointGroup Action Server**
- [ ] Navigation group moves both joints
- [ ] Container group synchronizes jaw positions
- [ ] Aggregate progress reported in feedback

**AC-2.6: Test Suite**
- [ ] Test script runs all Epic 2 tests
- [ ] 90%+ tests pass
- [ ] RQt documentation complete

## Test Strategy Summary

**Unit Tests:**
- ControllerInterface: validate joint limits, command formatting
- Limit switch thresholds: verify trigger logic

**Integration Tests:**
- MoveJoint: send goal via action client, verify joint moves
- MoveJointGroup: send group goal, verify coordinated motion
- Limit switches: move joint to limit, verify switch triggers

**Manual Tests:**
- RViz: verify markers appear correctly
- rqt_action: send goals via GUI

**Test Commands:**
```bash
# Launch Gazebo simulation
ros2 launch manipulator_description gazebo.launch.py

# Run Epic 2 tests
cd /home/robo/robo/ya_robot_manipulator/ros2_ws
source install/setup.bash
python3 -m pytest src/manipulator_control/test/test_epic2_joint_control.py -v

# Manual verification
ros2 topic list | grep end_switches  # Should show 18 topics
ros2 topic echo /manipulator/end_switches/picker_jaw_closed  # Should show bool
ros2 action send_goal /move_joint manipulator_control/action/MoveJoint "{joint_name: 'base_main_frame_joint', target_position: 1.0, max_velocity: 0.5}"
```

## Authoritative Joint Limits Reference

**Source of Truth:** `manipulator_description/urdf/manipulator/ros2_control.xacro`

| Joint | Min | Max | Unit | Notes |
|-------|-----|-----|------|-------|
| base_main_frame_joint | 0.0 | 4.0 | m | X-axis rail |
| main_frame_selector_frame_joint | -0.01 | 1.5 | m | Z-axis vertical |
| selector_frame_gripper_joint | -0.4 | 0.4 | m | Y-axis gripper |
| selector_frame_picker_frame_joint | -0.01 | 0.3 | m | Z-axis picker vertical |
| picker_frame_picker_rail_joint | -0.3 | 0.3 | m | Y-axis picker rail |
| picker_rail_picker_base_joint | 0.0 | 0.25 | m | X-axis picker extension |
| picker_base_picker_jaw_joint | 0.0 | 0.2 | m | X-axis picker jaw |
| selector_left_container_jaw_joint | -0.2 | 0.2 | m | Y-axis left jaw |
| selector_right_container_jaw_joint | -0.2 | 0.2 | m | Y-axis right jaw |

**IMPORTANT:** ControllerInterface MUST load these limits from ros2_control.xacro, NOT hardcode them.

---

## Risks, Assumptions, Open Questions

**Risks:**

**R1: Joint limit accuracy**
- **Likelihood:** Medium
- **Impact:** Low
- **Mitigation:** Load limits from ros2_control.xacro at runtime - DO NOT hardcode

**R2: Gazebo timing issues**
- **Likelihood:** Medium
- **Impact:** Medium
- **Mitigation:** Use simulation time, add timeout handling

**R3: selector_frame_max switch unreachable** ✅ RESOLVED
- **Issue:** limit_switches.yaml had selector_frame_max at 1.90, but ros2_control.xacro joint max is 1.5
- **Resolution:** Updated limit_switches.yaml to set selector_frame_max trigger_position to 1.45 (within joint range)

**Assumptions:**

**A1:** Gazebo + ros2_control is running with all controllers spawned
**A2:** Joint state broadcaster publishes /joint_states at 100 Hz
**A3:** ForwardCommandControllers accept Float64 position commands

**Open Questions:**

**Q1:** Should limit switches use contact sensors or position-based simulation?
- **Answer:** Position-based (Option 2 from architecture) - simpler implementation

**Q2:** How to handle container jaw mimic in MoveJointGroup?
- **Answer:** Software mimic: left = -width/2, right = +width/2
