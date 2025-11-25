# ya_robot_manipulator - Epic Breakdown

**Author:** BMad
**Date:** 2025-11-25
**Project Level:** Level 3 (High Complexity - Robotics Control System)
**Target Scale:** Medical Supply Warehouse Automation

---

## Overview

This document provides the complete epic and story breakdown for ya_robot_manipulator, decomposing the requirements from the [PRD](./prd.md) into implementable stories.

**Living Document Notice:** This is the initial version incorporating both PRD requirements and Architecture technical decisions. Ready for Phase 4 implementation.

---

## Configuration Reuse Policy

**CRITICAL REQUIREMENT:** DO NOT duplicate hardware configuration in new nodes. All nodes MUST load existing configurations:

| Configuration | Source File (Single Source of Truth) | Notes |
|--------------|--------------------------------------|-------|
| Joint limits | `manipulator_description/urdf/manipulator/ros2_control.xacro` | Load at runtime, DO NOT hardcode |
| Controller names | `manipulator_description/config/manipulator_controllers.yaml` | Discover topics dynamically |
| Switch positions | `manipulator_control/config/limit_switches.yaml` | Already created, use as-is |
| Storage params | `manipulator_description/config/storage_params.yaml` | Box dimensions, cabinet config |
| Manipulator params | `manipulator_description/config/manipulator_params.yaml` | Joint params reference |

**Node-specific config:** Create NEW config files ONLY for parameters unique to that node (publish rates, timeouts, action-specific settings). Never duplicate joint limits, switch positions, or controller names.

---

## Unified Launch File Pattern

**CRITICAL REQUIREMENT:** All Epic 2+ stories that add nodes, action servers, or services MUST update the unified launch file.

### Launch File Location
`ros2_ws/src/manipulator_control/launch/manipulator_simulation.launch.py`

### Simulation vs Hardware Separation

The launch file uses `use_sim_time` argument to control which nodes are launched:

```python
# Launch argument
use_sim_time_arg = DeclareLaunchArgument(
    'use_sim_time',
    default_value='true',
    description='Use simulation time (true) or real hardware (false)'
)
use_sim_time = LaunchConfiguration('use_sim_time')

# Simulation-only nodes (only launch when use_sim_time=true)
# - virtual_limit_switches (simulates hardware limit switches)
# - electromagnet_simulator (simulates hardware electromagnet)
# - box_spawner (Gazebo box spawning)

# Hardware-only nodes (only launch when use_sim_time=false)
# - hardware_limit_switches (reads real GPIO)
# - hardware_electromagnet (controls real GPIO)

# Common nodes (launch regardless of use_sim_time)
# - All action servers (MoveJoint, MoveJointGroup, etc.)
# - State marker publisher
# - Controller interface
```

### Node Categories

| Category | Nodes | Condition |
|----------|-------|-----------|
| **Simulation-Only** | virtual_limit_switches, electromagnet_simulator, box_spawner | `use_sim_time=true` |
| **Hardware-Only** | (future: hardware interfaces) | `use_sim_time=false` |
| **Common** | Action servers, state_marker_publisher, utilities | Always |

### Story Requirements

Each story that creates a new node MUST:
1. Add the node to `manipulator_simulation.launch.py`
2. Set appropriate `condition` (simulation-only, hardware-only, or common)
3. Use `use_sim_time` parameter for time-sensitive nodes
4. Add delayed start (`TimerAction`) if node depends on Gazebo/controllers

### Usage

```bash
# Simulation mode (default)
ros2 launch manipulator_control manipulator_simulation.launch.py

# Hardware mode
ros2 launch manipulator_control manipulator_simulation.launch.py use_sim_time:=false

# With joystick (simulation)
ros2 launch manipulator_control manipulator_simulation.launch.py enable_joy:=true
```

### Simulation Restart Policy

**CRITICAL REQUIREMENT:** Before relaunching simulation or tests, agents/developers MUST terminate existing processes:

```bash
# Kill Gazebo and RViz before relaunch
pkill -9 gazebo || true
pkill -9 gzserver || true
pkill -9 gzclient || true
pkill -9 rviz2 || true

# Alternative: kill all ROS2 nodes
pkill -9 -f ros2 || true
```

**Why:** Gazebo and RViz do not cleanly restart when launched while previous instances are running. This causes:
- Port conflicts
- Zombie processes
- Failed controller initialization
- TF tree corruption

**Rule:** Any story test script or manual testing that launches simulation MUST include cleanup commands before launch.

---

## Mandatory Developer Validation Requirements

**CRITICAL REQUIREMENT:** Every story MUST include developer self-validation before marking as complete.

### Testing Requirements

All stories that implement code MUST include:

1. **Build Verification** - `colcon build` succeeds with exit code 0
2. **Unit Tests** (if applicable) - pytest or equivalent passes
3. **Integration Tests** - Manual or scripted tests in Gazebo environment
4. **CLI Verification** - Verify nodes, topics, services, actions are available

**Rule:** Developer MUST execute ALL tests in the story's "Test Requirements" section and document results in "Dev Agent Record" section before marking story as `done`.

**Failure Handling:** If any test fails:
- Story status remains `in_progress`
- Failure details documented in Dev Agent Record
- Fix implemented and ALL tests re-run
- Only mark `done` when ALL tests pass

### Documentation Update Requirements

**CRITICAL REQUIREMENT:** After all tests pass, developer MUST update package documentation.

**Required Updates:**

1. **Package README.md** (`ros2_ws/src/manipulator_control/README.md`)
   - Add new nodes/executables to "Nodes" section
   - Add new topics/services/actions to "Interfaces" section
   - Update "Usage" examples if applicable
   - Add any new configuration files to "Configuration" section

2. **Config File Comments** - All new YAML files must have header comments explaining purpose and parameters

3. **Code Docstrings** - All new Python modules/classes/functions must have docstrings

**Rule:** Story is NOT complete until documentation reflects all implemented functionality. Reviewers should be able to understand what was added by reading the README.

### Story Completion Checklist

Before marking any story as `done`, verify:

- [ ] All acceptance criteria met
- [ ] Build succeeds (exit code 0)
- [ ] All unit tests pass
- [ ] All integration/manual tests pass and documented
- [ ] Package README.md updated with new functionality
- [ ] Code has appropriate docstrings
- [ ] Config files have header comments
- [ ] Dev Agent Record section filled with test results

---

## Functional Requirements Inventory

**FR-001:** Warehouse Address Navigation - Navigate manipulator to any warehouse address (side, cabinet, row, column)
**FR-002:** Address Coordinate Resolution - Resolve addresses via TF frames (no hardcoded coordinates)
**FR-003:** Box Extraction from Storage - Extract boxes using YZ motion and electromagnet
**FR-004:** Box Return to Original Address - Return extracted boxes to original storage addresses
**FR-005:** Item Picking with State Machine - Pick items from departments using limit switch-driven state machine
**FR-006:** Department Frame Generation - Generate and broadcast TF frames for department visualization (RViz)
**FR-007:** Limit Switch Simulation - Simulate 18 limit switches for all joints (Gazebo only)
**FR-008:** Electromagnet Simulation - Simulate electromagnet attachment/detachment (Gazebo only)
**FR-009:** Dynamic Box Spawning - Spawn/despawn boxes in Gazebo for physics simulation
**FR-010:** YZ Trajectory Generation - Generate collision-free trajectories for cabinet insertion/extraction
**FR-011:** Low-Level Joint Control - Provide MoveJoint and MoveJointGroup action interfaces
**FR-012:** Visual State Markers - Publish RViz markers for system state visualization
**FR-013:** Configuration from YAML - Load all configuration from YAML files (no hardcoded values)
**FR-014:** Box Relocation (PutBox) - Place extracted box in different empty address with validation
**FR-015:** Box Loading Station Operations - Move boxes to loading stations with optional relocation
**FR-016:** Container Jaw Manipulation - Synchronous control of container jaw joints
**FR-017:** Container Retrieval and Placement - Retrieve and place containers from predefined locations
**FR-018:** Address Validation Utilities - Validate addresses (empty check, width compatibility)
**FR-019:** Complete Pick-from-Storage Workflow - Execute complete autonomous pick operation
**FR-020:** RQt Tool Integration - Full observability/control via standard RQt tools

---

## FR Coverage Map

**Epic 1 (Package Setup & Interface Definitions):** Foundation for all FRs - establishes project structure and interfaces
**Epic 2 (Simulation Foundation & Joint Control):** FR-007, FR-011 (partial), FR-012 (partial), FR-013 (partial)
**Epic 3 (Address Navigation System):** FR-001, FR-002, FR-011 (complete), FR-012 (enhanced), FR-013 (partial)
**Epic 4A (Box Extraction Core):** FR-003, FR-004, FR-008, FR-009, FR-010, FR-013 (partial)
**Epic 4B (Advanced Box Operations):** FR-014, FR-015, FR-018
**Epic 5 (Item Picking & Department Frames):** FR-005, FR-006, FR-012 (complete), FR-013 (partial), FR-016, FR-017
**Epic 6 (High-Level Workflows & System Integration):** FR-019, FR-020, FR-013 (complete), all NFRs

---

## Epics Summary

**Epic 1:** Package Setup & Interface Definitions (3 stories) - Foundation
**Epic 2:** Simulation Foundation & Joint Control (6 stories) - Basic control + observability
**Epic 3:** Address Navigation System (6 stories) - Navigate to any warehouse address
**Epic 4A:** Box Extraction Core (5 stories) - Extract and return boxes from storage
**Epic 4B:** Advanced Box Operations (4 stories) - Box relocation and loading stations
**Epic 5:** Item Picking & Department Frames (8 stories) - Pick items from departments within boxes
**Epic 6:** High-Level Workflows & System Integration (8 stories) - Complete autonomous operation + validation

**Total: 7 epics, 40 stories**

---

## Epic 1: Package Setup & Interface Definitions

**Goal:** Establish the foundational ROS2 package structure and define all action/service/message interfaces required for the manipulator control system. This epic creates the scaffolding that enables all subsequent development.

**Value Delivered:** Development environment ready with validated, testable interfaces - developers can begin implementation immediately after Epic 1.

### Story 1.1: Create ROS2 Package Structure

As a developer,
I want to create the manipulator_control ROS2 package with proper structure,
So that I have an organized foundation for implementing control system components.

**Acceptance Criteria:**

**Given** a ROS2 workspace exists at `ros2_ws/src`
**When** I create the manipulator_control package
**Then** the package structure includes:
- Standard ROS2 package layout (src/, include/, launch/, config/, msg/, srv/, action/)
- CMakeLists.txt with dependencies: rclcpp, rclpy, std_msgs, geometry_msgs, sensor_msgs, tf2_ros, ros2_control
- package.xml with correct dependencies and maintainer information
- README.md with package description and build instructions

**And** `colcon build --packages-select manipulator_control` succeeds without errors

**Prerequisites:** None (first story)

**Technical Notes:**
- Verify manipulator_description package exists (contains URDF, config files)
- Add dependency on custom message packages if creating new message types
- Include ros2_control dependencies: controller_manager, controller_interface

---

### Story 1.2: Define All Action Interfaces

As a developer,
I want to define all action interfaces (.action files) for manipulator control,
So that action servers have standardized goal/feedback/result structures.

**Acceptance Criteria:**

**Given** the manipulator_control package structure exists
**When** I create action definitions in `action/` directory
**Then** the following action files are defined with complete specifications:

**Low-Level Actions:**
- `MoveJoint.action` - Goal: joint_name (string), target_position (float64), max_velocity (float64, optional). Feedback: current_position, progress_percent. Result: success (bool), final_position
- `MoveJointGroup.action` - Goal: joint_names (string[]), target_positions (float64[]), max_velocity (float64, optional). Feedback: current_positions, progress_percent. Result: success, final_positions

**Mid-Level Actions:**
- `NavigateToAddress.action`, `ExtractBox.action`, `ReturnBox.action`, `PutBox.action`, `MoveBoxToLoad.action`
- `ManipulateContainer.action`, `GetContainer.action`, `PlaceContainer.action`, `PickItem.action`

**High-Level Actions:**
- `PickItemFromStorage.action` - Goal: address, department_num, container_id. Feedback: phase, operation_step, elapsed_time. Result: success, total_time

**And** all actions compile successfully with `colcon build`
**And** action interface documentation is auto-generated from comments

**Prerequisites:** Story 1.1

**Technical Notes:**
- Use consistent naming: PascalCase for action names, snake_case for field names
- Include detailed comments in .action files (will be used for documentation)
- Follow architecture action hierarchy (lines 2391-2414)

---

### Story 1.3: Define Service and Message Interfaces

As a developer,
I want to define all service and custom message interfaces,
So that utilities and nodes have standardized communication contracts.

**Acceptance Criteria:**

**Given** the package structure exists
**When** I create service and message definitions
**Then** the following interfaces are defined:

**Services:**
- `GetAddressCoordinates.srv`, `ToggleElectromagnet.srv`, `SpawnBox.srv`, `DespawnBox.srv`, `ValidateAddress.srv`

**Custom Messages:**
- `Address.msg` - side (string), cabinet_num (int), row (int), column (int)
- `JointCommand.msg`, `LimitSwitchState.msg`

**And** all interfaces compile successfully
**And** stub Python/C++ code can import these interfaces

**Prerequisites:** Story 1.1

**Technical Notes:**
- Keep messages simple - avoid deep nesting
- Services should always return success bool + error_message for diagnostics

---

## Epic 2: Simulation Foundation & Joint Control

**Goal:** Establish simulation infrastructure for limit switches, electromagnets, and basic joint control actions. This epic enables testing of all subsequent functionality in Gazebo without hardware dependencies.

**Value Delivered:** Complete simulation environment with observable joint control, state visualization, and foundational action servers ready for mid-level action composition.

### Story 2.1: Implement Virtual Limit Switch Simulation

As a developer,
I want to simulate all 18 limit switches (2 per joint) in Gazebo,
So that picker state machine and safety monitoring can operate without physical hardware.

**Acceptance Criteria:**

**Given** the manipulator has 9 joints with position limits defined in ros2_control.xacro
**When** I launch the virtual_limit_switches_node
**Then** the node publishes boolean states for all 18 switches at 10 Hz:
- `/manipulator/end_switches/base_main_frame_min` and `_max` (X-axis rail)
- `/manipulator/end_switches/selector_frame_min` and `_max` (Z-axis selector)
- `/manipulator/end_switches/gripper_left` and `gripper_right` (Y-axis gripper - LEFT/RIGHT not extend/retract!)
- `/manipulator/end_switches/picker_frame_min` and `_max` (Z-axis picker vertical)
- `/manipulator/end_switches/picker_rail_min` and `_max` (Y-axis picker rail)
- `/manipulator/end_switches/picker_retracted` and `picker_extended` (X-axis picker extension)
- `/manipulator/end_switches/picker_jaw_opened` and `picker_jaw_closed` (X-axis picker jaw)
- `/manipulator/end_switches/container_left_min`, `container_left_max` (Y-axis left jaw)
- `/manipulator/end_switches/container_right_min`, `container_right_max` (Y-axis right jaw)

**And** switch states transition to TRUE when joint position reaches trigger threshold (within 0.01m tolerance)
**And** switch trigger positions are loaded from `config/limit_switches.yaml`
**And** the node subscribes to `/joint_states` to monitor all joint positions

**Prerequisites:** Story 1.1 (package structure exists)

**Technical Notes:**
- Reference architecture "Core Physical Workflows Reference" section and "Switch Name Reference Table"
- Configuration file: `manipulator_control/config/limit_switches.yaml` with trigger_position and trigger_tolerance for each switch
- **CRITICAL:** Use correct switch names from `ros2_ws/src/manipulator_control/config/limit_switches.yaml`:
  - `gripper_left` (Y=+0.39) / `gripper_right` (Y=-0.39) - NOT gripper_extended/retracted
  - `picker_extended` (X=0.24) / `picker_retracted` (X=0.01) - NOT picker_base_min/max
  - `picker_jaw_opened` (X=0.19) / `picker_jaw_closed` (X=0.01) - grasp control
- Publish rate: 10 Hz minimum for picker state machine responsiveness
- Example: picker_jaw_closed triggers when picker_base_picker_jaw_joint <= 0.01

**Implementation Notes (Config Reuse):**
- **USE:** `manipulator_control/config/limit_switches.yaml` - already exists with all switch definitions
- **DO NOT CREATE:** New config file for switch positions - load from existing file
- ✅ `selector_frame_max` corrected to 1.45 (was 1.90, exceeding joint max of 1.5)

---

### Story 2.2: Create Controller Interface Utility

As a developer,
I want a utility to send position commands to individual ForwardCommandController instances,
So that action servers can control joints without hardcoding controller topic names.

**Acceptance Criteria:**

**Given** the manipulator uses individual ForwardCommandControllers defined in manipulator_controllers.yaml
**When** I use the ControllerInterface utility to command a joint
**Then** the utility publishes Float64 position commands to the correct controller topic:
- Format: `/[controller_name]/commands` (e.g., `/base_main_frame_joint_controller/commands`) - NOTE: plural "commands", uses Float64MultiArray

**And** the utility loads controller names from `/ros2_ws/src/manipulator_description/config/manipulator_controllers.yaml`
**And** the utility provides methods:
- `command_joint(joint_name: str, position: float) -> bool`
- `command_joint_group(joint_names: List[str], positions: List[float]) -> bool`
- `get_joint_position(joint_name: str) -> float` (from /joint_states)

**And** the utility validates position commands against joint limits before publishing
**And** invalid commands return False with logged warning

**Prerequisites:** Story 1.1

**Technical Notes:**
- Reference architecture lines 13-28 for controller architecture (individual ForwardCommandControllers, NOT JointTrajectoryController)
- Controller topics: each joint has `/[joint_name]_controller/commands` (Float64MultiArray) - NOTE: plural "commands"
- Load joint limits from ros2_control.xacro or query controller_manager parameters
- This utility will be used by MoveJoint and MoveJointGroup action servers
- Cache joint limits on initialization for validation performance
- **LAUNCH FILE UPDATE:** Update `manipulator_simulation.launch.py` to add `use_sim_time` argument and apply `IfCondition(use_sim_time)` to virtual_limit_switches node (simulation-only). See "Unified Launch File Pattern" section.

**Implementation Notes (Config Reuse):**
- **USE:** `manipulator_description/config/manipulator_controllers.yaml` - controller names and joint mappings
- **USE:** `manipulator_description/urdf/manipulator/ros2_control.xacro` - joint limits (parse XACRO or use ROS2 param server)
- **DO NOT CREATE:** New config for joint limits or controller topics - load from existing files
- **CREATE ONLY:** Utility-specific parameters if needed (e.g., retry count, logging level)

---

### Story 2.3: Implement MoveJoint Action Server

As a robotics operator,
I want to command individual joints to target positions,
So that I can test joint control and build higher-level coordinated motions.

**Acceptance Criteria:**

**Given** the MoveJoint action interface is defined with goal (joint_name, target_position, max_velocity), feedback (current_position, progress_percent), and result (success, final_position, execution_time)
**When** I send a MoveJoint goal via action client
**Then** the action server:
1. Validates joint_name exists and target_position is within limits
2. Uses ControllerInterface to command the joint
3. Publishes feedback at 10 Hz with current position and progress (0-100%)
4. Monitors /joint_states to detect when target reached (within 0.01m tolerance)
5. Returns success result when position reached or timeout (30 seconds default)

**And** action supports preemption (cancellation during motion)
**And** action server loads timeout from `config/action_servers.yaml`
**And** action can be tested using `rqt_action` tool

**Prerequisites:** Story 2.2 (ControllerInterface exists)

**Technical Notes:**
- Reference architecture lines 2183-2197 for low-level action implementation patterns
- Action definition: `manipulator_control/action/MoveJoint.action`
- Configuration: `config/action_servers.yaml` with timeout_sec and position_tolerance
- Use ControllerInterface.command_joint() for position commands
- Subscribe to /joint_states to monitor progress
- Simple point-to-point motion (no trajectory interpolation at this level)
- NFR-002: Position accuracy ±0.01m for storage operations
- **LAUNCH FILE UPDATE:** Add move_joint_server node to `manipulator_simulation.launch.py` as Common node (no condition - runs in both sim and hardware). Use 3s delayed start.

**Implementation Notes (Config Reuse):**
- **USE:** ControllerInterface (Story 2.2) - already loads joint limits and controller topics
- **CREATE:** `config/action_servers.yaml` - NEW file for action-specific parameters only:
  - `timeout_sec`, `position_tolerance`, `feedback_rate` - these are action server parameters, NOT hardware config
- **DO NOT DUPLICATE:** Joint limits in action_servers.yaml - get from ControllerInterface

---

### Story 2.4: Implement State Marker Publisher for Visualization

As a developer,
I want visual markers in RViz showing system state (magnet engaged, target address, extracted addresses),
So that I can observe system behavior during testing and debugging.

**Acceptance Criteria:**

**Given** the manipulator control system is running
**When** I launch the state_marker_publisher_node
**Then** the node publishes MarkerArray messages to `/visualization_marker_array` at 10 Hz with:

**Magnet Engaged Marker:**
- Red sphere (0.05m diameter, 80% opacity) attached to gripper_magnet_link frame
- Visible only when `/manipulator/electromagnet/engaged` is TRUE

**Target Address Marker:**
- Green semi-transparent cube (50% opacity) at target address TF frame
- Size matches box dimensions from storage_params.yaml
- Visible when NavigateToAddress or ExtractBox action is active

**Extracted Address Markers:**
- Red semi-transparent cubes (50% opacity) at addresses with extracted boxes
- Persist until ReturnBox or PutBox action completes for that address

**And** marker namespace is "manipulator_state" for filtering in RViz
**And** markers are visible in RViz Displays → MarkerArray
**And** node subscribes to `/manipulator/state` topic for system state updates

**Prerequisites:** Story 1.1

**Technical Notes:**
- Reference architecture lines 880-1007 for marker implementation details
- Marker types: Marker.SPHERE for magnet, Marker.CUBE for addresses
- Frame references: gripper_magnet_link (or selector_frame_gripper_link), addr_l_1_2_3 format for addresses
- Use storage_params.yaml box dimensions for marker sizes (width, height, depth)
- Color scheme: Red (1.0, 0.0, 0.0), Green (0.0, 1.0, 0.0) with alpha for transparency
- Department markers will be added in Epic 5 (Item Picking)

**Implementation Notes (Config Reuse):**
- **USE:** `manipulator_description/config/storage_params.yaml` - box dimensions for marker sizes
- **USE:** TF frames from URDF (addr_l_1_2_3 format) - address positions
- **CREATE:** `config/state_markers.yaml` - NEW file for marker-specific parameters only:
  - `marker_namespace`, `update_rate`, `colors`, `sizes` - visualization parameters
- **DO NOT DUPLICATE:** Box dimensions or address positions - load from existing files

---

### Story 2.5: Implement MoveJointGroup Action Server

As a developer,
I want to command groups of joints simultaneously (navigation, gripper, picker, container),
So that I can execute coordinated multi-joint motions efficiently.

**Acceptance Criteria:**

**Given** the MoveJointGroup action interface is defined with goal (joint_group, target_positions, max_velocity), feedback (current_positions, progress_percent), and result (success, final_positions, position_error, execution_time)
**When** I send a MoveJointGroup goal with joint_group="navigation" and 2 target positions
**Then** the action server:
1. Loads joint group definition from `config/kinematic_chains.yaml`
2. Validates target_positions array length matches joint count in group
3. Commands all joints simultaneously using ControllerInterface
4. Monitors all joints and computes aggregate progress (average of individual joint progress)
5. Returns success when ALL joints reach targets (within 0.01m tolerance) or timeout

**And** supported joint groups are:
- "navigation": [base_main_frame_joint, main_frame_selector_frame_joint]
- "gripper": [selector_frame_gripper_joint, main_frame_selector_frame_joint]
- "picker": [4 picker joints]
- "container": [selector_left_container_jaw_joint, selector_right_container_jaw_joint]

**And** container group implements software mimic (synchronized jaw motion)
**And** action provides feedback at 10 Hz with current_positions array and progress_percent

**Prerequisites:** Story 2.3 (MoveJoint server exists), Story 2.2 (ControllerInterface)

**Technical Notes:**
- Reference architecture lines 2386-2423 for joint group definitions
- Configuration: `config/kinematic_chains.yaml` with joint lists, default velocities
- For container jaws: left_target = -opening/2, right_target = +opening/2 for synchronized motion
- Use ControllerInterface.command_joint_group() for simultaneous commands
- Aggregate progress: sum(individual_progress) / joint_count
- All joints must reach targets within 1 second of each other (coordinate arrival)

**Implementation Notes (Config Reuse):**
- **USE:** ControllerInterface (Story 2.2) - already loads joint limits and controller topics
- **USE:** `config/action_servers.yaml` (Story 2.3) - shared timeout/tolerance settings
- **CREATE:** `config/kinematic_chains.yaml` - NEW file for joint group definitions:
  - `joint_groups` with joint lists, default velocities, descriptions
  - `mimic_mode` for container jaws - this is group coordination logic, not hardware config
- **DO NOT DUPLICATE:** Joint names or limits - reference from existing config

---

### Story 2.6: Create Test Script and RQt Documentation

As a developer,
I want test scripts and RQt tool usage documentation,
So that I can validate Epic 2 functionality and train other developers.

**Acceptance Criteria:**

**Given** Epic 2 stories 2.1-2.5 are implemented
**When** I run the test script `test/test_epic2_joint_control.py`
**Then** the script executes automated tests:
1. Verify all 18 limit switches publish and change state correctly
2. Send MoveJoint commands to each of the 9 joints and verify position reached
3. Send MoveJointGroup command to navigation group and verify coordinated motion
4. Verify container jaw synchronization (both jaws mirror positions)
5. Verify state markers appear/disappear correctly in RViz

**And** test results are logged with pass/fail status and execution time
**And** at least 90% of tests pass (success criteria for Epic 2 completion)

**And** RQt documentation exists at `docs/TESTING_WITH_RQT.md` explaining:
- How to launch rqt with standard tools (rqt_action, rqt_topic, rqt_console)
- How to send MoveJoint goals using rqt_action GUI
- How to monitor limit switches using rqt_topic
- How to save/load RQt perspectives for development workflow
- Screenshots of typical RQt layout for manipulator testing

**Prerequisites:** Stories 2.1-2.5

**Technical Notes:**
- Test script uses pytest and ros2 launch_testing framework
- Launch manipulator_description gazebo.launch.py before running tests
- Test timeout: 5 minutes total (allows Gazebo startup + motion tests)
- RQt perspective file: `config/manipulator_dev.perspective` with pre-configured plugins
- Reference architecture lines 1069-1110 for RQt tool approach
- NFR-008: 95% success rate for basic operations (validate with test runs)

---

## Epic 3: Address Navigation System

**Goal:** Implement TF-based warehouse address resolution and navigation, enabling the manipulator to move to any cabinet location (side, cabinet, row, column) with precision positioning.

**Value Delivered:** Complete navigation capability to 100+ warehouse addresses using existing TF frames, with address validation and coordinate resolution services.

### Story 3.1: Implement Address Resolver Utility

As a developer,
I want a utility to resolve warehouse addresses to (x, y, z) coordinates using TF frames,
So that action servers can navigate without hardcoded address tables.

**Acceptance Criteria:**

**Given** warehouse addresses exist as TF frames in URDF with format `addr_{side}_{cabinet}_{row}_{column}`
**When** I call the address resolver with (side="left", cabinet=1, row=2, column=3)
**Then** the utility:
1. Constructs frame name "addr_l_1_2_3"
2. Uses TF2 lookup to get transform from "world" to address frame
3. Returns (x, y, z) coordinates in world frame
4. Returns error if frame doesn't exist or TF lookup fails (timeout 1.0 second)

**And** the utility validates address against storage_params.yaml cabinet configurations:
- Left row: Cabinets 1-4 with specified column/row counts
- Right row: Cabinets 1-4 with specified column/row counts
- Returns error for invalid cabinet numbers or out-of-range rows/columns

**And** utility provides methods:
- `get_address_coordinates(side, cabinet, row, column) -> (x, y, z, success, error_msg)`
- `validate_address(side, cabinet, row, column) -> (valid, error_msg)`
- `get_cabinet_config(side, cabinet) -> dict` (returns columns, rows, departments)

**Prerequisites:** Story 1.1 (package structure)

**Technical Notes:**
- Reference architecture lines 86-119 for address resolution method using TF2
- Address format: side_abbrev = 'l' for left, 'r' for right
- Example frames: addr_l_1_1_1, addr_r_4_3_5
- Use tf2_ros.Buffer and tf2_ros.TransformListener for lookups
- Load cabinet configurations from storage_params.yaml (cabinet_rows section)
- NFR-003: Address validation response time < 100ms

**Implementation Notes (Config Reuse):**
- **USE:** `manipulator_description/config/storage_params.yaml` - cabinet configurations, row/column counts
- **USE:** TF frames from URDF (addr_{side}_{cabinet}_{row}_{column} format)
- **DO NOT CREATE:** New config for address lookups - all coordinates resolved via TF

---

### Story 3.2: Implement GetAddressCoordinates Service

As a developer,
I want a ROS2 service to query address coordinates,
So that any node can resolve addresses without duplicating TF lookup logic.

**Acceptance Criteria:**

**Given** the GetAddressCoordinates service is defined with request (side, cabinet_num, row, column) and response (success, pose, error_message)
**When** I call the service with a valid address
**Then** the service:
1. Uses AddressResolver utility to get coordinates
2. Returns geometry_msgs/Pose with position (x, y, z) and orientation (identity quaternion)
3. Returns success=true

**When** I call with an invalid address (non-existent cabinet or out-of-range row/column)
**Then** the service returns success=false with descriptive error_message (e.g., "Cabinet 5 does not exist on left side" or "Row 15 exceeds cabinet 1 max of 10 rows")

**And** service response time is < 100ms for valid addresses (NFR-003)
**And** service can be tested using `ros2 service call` command

**Prerequisites:** Story 3.1 (AddressResolver exists)

**Technical Notes:**
- Service definition: `srv/GetAddressCoordinates.srv`
- Reference architecture lines 121-135 for service structure
- Service name: `/manipulator/get_address_coordinates`
- Use AddressResolver.get_address_coordinates() for implementation
- Cache cabinet configurations on node startup for performance
- Log all service calls at DEBUG level for troubleshooting

**Implementation Notes (Config Reuse):**
- **USE:** AddressResolver (Story 3.1) - already loads storage_params.yaml
- **DO NOT CREATE:** New config - this is a service wrapper around AddressResolver

---

### Story 3.3: Implement MoveJointGroup with Joint Groups Configuration

As a developer,
I want to extend MoveJointGroup to support predefined joint groups from configuration,
So that navigation and other coordinated motions use consistent groupings.

**Acceptance Criteria:**

**Given** `config/kinematic_chains.yaml` defines joint groups with joints, default_velocity, default_acceleration
**When** MoveJointGroup action server starts
**Then** it loads all joint group definitions from config file

**And** when I send goal with joint_group="navigation", the server:
1. Looks up [base_main_frame_joint, main_frame_selector_frame_joint]
2. Applies default_velocity=0.5 m/s if max_velocity not specified in goal
3. Commands both joints simultaneously

**And** for joint_group="container", the server implements software mimic:
- Calculates symmetric positions: left = -opening/2, right = +opening/2
- Commands both jaws to create desired opening width
- Verifies synchronized arrival (both within 0.01m of target within 1 second)

**And** invalid joint_group names are rejected with error message listing valid groups

**Prerequisites:** Story 2.5 (MoveJointGroup basic implementation)

**Technical Notes:**
- Reference architecture lines 2386-2423 for joint group configuration structure
- Configuration file: `config/kinematic_chains.yaml`
- Joint groups: navigation (2 DOF), gripper (1 DOF), picker (4 DOF), container (2 DOF)
- Container mimic behavior: Ensures jaws open symmetrically around center line
- Use yaml.safe_load() to parse configuration on startup

**Implementation Notes (Config Reuse):**
- **USE:** `manipulator_control/config/kinematic_chains.yaml` - joint group definitions (created in Story 2.5)
- **USE:** ControllerInterface (Story 2.2) - already loads joint limits and controller topics
- **DO NOT DUPLICATE:** Joint names - reference from kinematic_chains.yaml

---

### Story 3.4: Implement NavigateToAddress Action Server

As a robotics operator,
I want to navigate the manipulator to any warehouse address,
So that I can position for box extraction or item picking operations.

**Acceptance Criteria:**

**Given** the NavigateToAddress action is defined with goal (side, cabinet_num, row, column, approach_distance) and result (success, final_position, positioning_error, message)
**When** I send a NavigateToAddress goal for address (left, 1, 2, 3) with approach_distance=0.1m
**Then** the action server:
1. Validates address using AddressResolver
2. Gets target coordinates (x, y, z) via GetAddressCoordinates service
3. Calculates joint positions: x_joint = target_x, z_joint = target_z
4. Sends MoveJointGroup goal with joint_group="navigation" and [x_joint, z_joint]
5. Monitors progress and publishes feedback (current_joint_positions, distance_to_target, progress_percent) at 5 Hz
6. Returns success when positioning_error < 0.02m (NFR-002)

**And** approach_distance parameter offsets Y-axis position (default 0.1m from cabinet face) to avoid collision
**And** action timeout is 30 seconds (configurable in action_servers.yaml)
**And** final_position in result contains actual (x, y, z) reached
**And** action can be tested with rqt_action GUI

**Prerequisites:** Story 3.3 (MoveJointGroup with groups), Story 3.2 (GetAddressCoordinates service), Story 3.1 (AddressResolver)

**Technical Notes:**
- Reference architecture lines 387-430 for NavigateToAddress implementation
- Action definition: `action/NavigateToAddress.action`
- Simple inverse kinematics: For XZ plane, joint positions = target coordinates directly (prismatic joints)
- Positioning error = sqrt((x_target - x_actual)^2 + (z_target - z_actual)^2)
- NFR-002: Position accuracy ±0.02m for cabinet operations
- Update state marker publisher to show green marker at target address during navigation

**Implementation Notes (Config Reuse):**
- **USE:** `manipulator_control/config/action_servers.yaml` - timeout_sec, position_tolerance (add navigate_to_address section)
- **USE:** GetAddressCoordinates service (Story 3.2), MoveJointGroup (Story 2.5)
- **DO NOT CREATE:** Separate config for navigation - add section to action_servers.yaml

---

### Story 3.5: Add Address State Markers to Visualization

As a developer,
I want visual markers in RViz showing target and extracted addresses,
So that I can observe navigation targets and track which addresses have been accessed.

**Acceptance Criteria:**

**Given** the state_marker_publisher_node is running
**When** NavigateToAddress action becomes active with target address
**Then** a GREEN semi-transparent cube marker appears at the target address TF frame:
- Frame: addr_l_1_2_3 (or corresponding address)
- Size: matches box dimensions from storage_params.yaml (width, height, depth)
- Color: (0.0, 1.0, 0.0) with alpha=0.5
- Namespace: "target_address"

**When** ExtractBox action succeeds for an address
**Then** a RED semi-transparent cube marker appears at that address:
- Indicates "box extracted, address empty"
- Persists until ReturnBox or PutBox completes for that address
- Namespace: "extracted_addresses"

**And** markers update at 10 Hz with current system state
**And** all address markers are visible in RViz when MarkerArray display is enabled
**And** marker IDs are unique and stable (derived from address coordinates)

**Prerequisites:** Story 2.4 (basic state marker publisher), Story 3.4 (NavigateToAddress action)

**Technical Notes:**
- Reference architecture lines 976-1007 for address marker creation
- Load box dimensions from storage_params.yaml box_configurations section
- Marker pose.position.y offset: 0.1m into cabinet (visual hint of location)
- Track extracted addresses in state_marker_publisher node state
- Markers persist across action completions until explicitly cleared
- Use marker action=DELETE to remove markers when boxes returned

**Implementation Notes (Config Reuse):**
- **USE:** `manipulator_control/config/visualization.yaml` - marker colors, sizes (add target_address, extracted_addresses sections)
- **USE:** `manipulator_description/config/storage_params.yaml` - box dimensions for marker sizes
- **DO NOT CREATE:** Separate marker config - extend visualization.yaml from Story 2.4

---

### Story 3.6: Create Navigation Test Suite

As a developer,
I want automated tests validating navigation to multiple addresses,
So that I can ensure positioning accuracy meets requirements across all cabinet configurations.

**Acceptance Criteria:**

**Given** Epic 3 stories 3.1-3.5 are implemented
**When** I run `test/test_epic3_navigation.py`
**Then** the test suite executes:

**Test 1: Address Resolution**
- Query coordinates for 10 sample addresses (mix of left/right, different cabinets/rows/columns)
- Verify all return valid (x, y, z) coordinates
- Verify invalid addresses return error messages

**Test 2: Navigation Accuracy**
- Navigate to 5 different addresses in sequence
- Verify positioning error < 0.02m for each (NFR-002)
- Verify execution time < 30 seconds per navigation

**Test 3: Marker Visualization**
- Navigate to address, verify green marker appears at target
- Verify marker has correct size (from storage_params)
- Verify marker removed when action completes

**Test 4: Cabinet Coverage**
- Navigate to at least one address in each of 8 cabinets (4 left, 4 right)
- Verify no collisions or limit violations
- Verify repeatable positioning (same address twice yields same position)

**And** test results show pass/fail for each test with execution time
**And** at least 95% of navigation attempts succeed (NFR-008)
**And** test documentation explains how to run tests and interpret results

**Prerequisites:** Stories 3.1-3.5

**Technical Notes:**
- Use pytest with launch_testing for Gazebo integration
- Test addresses: Sample from all cabinet types (4x10, 5x12, 6x14, etc.)
- Positioning error measurement: Compare /joint_states actual vs. expected from TF
- Test timeout: 10 minutes (includes Gazebo startup + multiple navigation sequences)
- NFR-002: ±0.02m position accuracy required for box extraction
- NFR-008: 95%+ success rate for navigation operations

**Implementation Notes (Config Reuse):**
- **USE:** Existing configurations from Stories 3.1-3.5
- **DO NOT CREATE:** New config - tests use existing configs

---

## Epic 4A: Box Extraction Core

**Goal:** Implement YZ trajectory generation, electromagnet simulation, dynamic box spawning, and core box extraction/return actions with collision-free motion.

**Value Delivered:** Complete box handling capability with 100+ boxes extractable/returnable, enabling all downstream item picking and box relocation workflows.

### Story 4A.1: Implement YZ Trajectory Generator Utility

As a developer,
I want a utility to generate safe YZ plane trajectories for box insertion/extraction,
So that gripper motion avoids cabinet frame collisions during box operations.

**Acceptance Criteria:**

**Given** cabinet dimensions and wall thickness are defined in storage_params.yaml
**When** I call generate_insertion_trajectory(current_y, current_z, target_y, target_z, approach_speed)
**Then** the utility returns a list of waypoints with:
- Each waypoint: {y: float, z: float, speed: float, description: string}
- Waypoints ensure gripper clears cabinet top frame (safety_margin=0.02m)
- Path adjusts Z height if current_z risks top frame collision
- Smooth progression from current to target position

**When** I call generate_extraction_trajectory(current_y, current_z, safe_y, retract_speed)
**Then** the utility returns waypoints for safe retraction:
- Checks if current_z near top frame (cabinet exterior.z - wall_thickness)
- Lowers Z if needed before retracting Y
- Smooth Y- motion to safe position outside cabinet

**And** utility provides execute_yz_trajectory(waypoints, joint_names) method:
- Commands joints using MoveJointGroup action client
- Waits for each waypoint completion before proceeding to next
- Returns success/failure for overall trajectory execution

**And** trajectories are tested in Gazebo without collisions for 10 different addresses

**Prerequisites:** Story 3.3 (MoveJointGroup with joint groups)

**Technical Notes:**
- Reference architecture lines 1468-1657 for YZ trajectory generation algorithms
- Utility class: YZTrajectoryGenerator in utils/yz_trajectory_generator.py
- Cabinet dimensions from storage_params.yaml: exterior.z=1.4m, wall_thickness=0.02m
- Frame clearance check: current_z > (exterior.z - wall_thickness - safety_margin)
- Joint names for YZ motion: [main_frame_selector_frame_joint, selector_frame_gripper_joint]
- Approach speeds: 0.05 m/s (slow/safe), 0.08 m/s (normal), 0.1 m/s (fast)
- Waypoint descriptions aid debugging (e.g., "Align Z height", "Insert to target depth")

**Implementation Notes (Config Reuse):**
- **USE:** `manipulator_description/config/storage_params.yaml` - cabinet exterior dimensions, wall_thickness
- **CREATE:** `manipulator_control/config/trajectory.yaml` - NEW file for trajectory-specific parameters:
  - `safety_margin`, `clearance_height`, `speeds` (approach, normal, fast)
- **DO NOT DUPLICATE:** Cabinet dimensions - load from storage_params.yaml

---

### Story 4A.2: Implement Electromagnet Simulator and Service

As a developer,
I want to simulate electromagnet attachment/detachment in Gazebo,
So that box extraction actions can control magnet state via ROS2 service.

**Acceptance Criteria:**

**Given** the ToggleElectromagnet service is defined with request (activate: bool) and response (success: bool, message: string)
**When** I call the service with activate=true
**Then** the electromagnet_simulator_node:
1. Checks if a box is within proximity (< 0.05m from gripper_magnet_link)
2. If yes, publishes Empty message to `/model/{box_id}/detachable_joint/attach` topic
   - Uses DetachableJoint plugin to create fixed joint between gripper and box
   - Box model must have DetachableJoint plugin configured in its URDF/SDF
3. Publishes magnet state to `/manipulator/electromagnet/engaged` (Bool, true)
4. Returns success=true

**When** I call with activate=false
**Then** the node:
1. Publishes Empty message to `/model/{box_id}/detachable_joint/detach` topic
2. Box is released from gripper in Gazebo physics simulation
3. Publishes magnet state engaged=false
4. Returns success=true

**And** magnet state topic publishes at 10 Hz for visualization
**And** red sphere marker appears on gripper when magnet engaged (via state_marker_publisher)
**And** service can be called via `ros2 service call` for testing

**Prerequisites:** Story 1.1 (package structure)

**Technical Notes:**
- Reference architecture lines 312-414 for electromagnet simulation approaches
- Service definition: `srv/ToggleElectromagnet.srv`
- **Gazebo Harmonic Implementation:** Use **DetachableJoint** plugin for attach/detach
  - Attach topic: `/model/{box_id}/detachable_joint/attach` (publish Empty message)
  - Detach topic: `/model/{box_id}/detachable_joint/detach` (publish Empty message)
  - Plugin creates fixed joint between `left_gripper_magnet` link and `{box_id}_base_link`
  - Reference: [DetachableJoint Plugin Documentation](https://gazebosim.org/api/sim/9/detachablejoints.html)
- Proximity check: Use TF to calculate distance between gripper_magnet_link and box_link
- Alternative approach: Use `gazebo_ros_link_attacher` for older compatibility
- For hardware: Replace with GPIO control (e.g., RPi.GPIO for electromagnet relay)
- Service name: `/manipulator/electromagnet/toggle`

**Implementation Notes (Config Reuse):**
- **CREATE:** `manipulator_control/config/electromagnet.yaml` - NEW file for electromagnet parameters:
  - `proximity_distance`, `engagement_wait_sec`, `disengagement_wait_sec`, `state_topic`
- **DO NOT DUPLICATE:** Link names - reference from URDF

---

### Story 4A.3: Implement Dynamic Box Spawner with Department Frame Generation

As a developer,
I want to dynamically add box URDF with department child links to the ROS2 TF tree,
So that department positions are available for real item detection and picking in both hardware and simulation modes.

**CRITICAL UNDERSTANDING:**
- **PRIMARY GOAL:** Add box URDF to ROS2 TF tree → Department frames available for item positioning
- **SECONDARY GOAL:** Spawn visual representation in Gazebo (simulation only, optional)
- **Why this matters:** On real hardware, there's no Gazebo, but we NEED department TF frames to know where items are located within the box. The TF tree is the source of truth for item positions.

**Implementation Order:**
1. **FIRST:** Generate URDF + launch robot_state_publisher → TF frames exist
2. **SECOND (optional):** If in simulation, also spawn visual model in Gazebo
3. **Result:** Department frames work identically for hardware and simulation

**Acceptance Criteria:**

**Given** the SpawnBox service is defined with request (box_id, side, cabinet_num, row, column, num_departments) and response (success, message)
**When** I call SpawnBox for address (left, 1, 2, 3) with 10 departments
**Then** the box_spawn_manager_node:

**Phase 1: Generate URDF String (progress 0-20%)**
1. Loads box parameters from storage_params.yaml (width, height, depth based on cabinet columns/rows)
2. Gets spawn position from address TF frame (addr_l_1_2_3) via TF lookup
3. Generates URDF XML string with:
   - `<robot name="{box_id}">` root element
   - Base link: `{box_id}_base_link` with visual/collision/inertial (box geometry with dimensions from storage_params)
   - Department child links: `{box_id}_dept_1_link` through `{box_id}_dept_{num_departments}_link`
   - Fixed joints connecting base_link to each department link with origins:
     - x=0, y=offset_y + (dept_num - 1) × dept_depth, z=0 (equally spaced along Y-axis)
     - dept_depth and offset_y from storage_params.yaml department_configurations
   - Each department link has small visual marker (sphere radius 0.01m) for RViz visualization
   - Inertial properties: mass=0.5kg total distributed across links

**Phase 2: Add Box to ROS2 TF Tree (progress 20-70%) - PRIMARY GOAL**
4. Spawns a robot_state_publisher node for this box with parameters:
   - robot_description = URDF string
   - frame_prefix = "" (no prefix, use box_id in URDF)
   - use_sim_time = True (for simulation) or False (for hardware)
   - **This publishes TF transforms for base_link and all department child links to /tf_static**
   - **Department frames are NOW available in TF tree for item picking operations**
5. Publishes static transform from **gripper magnet link** to `{box_id}_base_link`
   - Uses tf2_ros.StaticTransformBroadcaster
   - Transform: `left_gripper_magnet` → `{box_id}_base_link` at (0, 0, 0) with appropriate rotation
   - **CRITICAL:** Box is attached to the gripper/electromagnet, not to world frame
   - This makes the box follow the gripper as it moves during extraction
6. Tracks robot_state_publisher process for cleanup on despawn
7. **At this point, department TF frames are fully available: `{box_id}_dept_1_link`, etc.**
8. **Department frames move with the gripper** since box is attached to gripper magnet link

**Phase 3: Spawn in Gazebo (progress 70-90%) - SECONDARY (simulation only)**
9. **IF in simulation mode** (check if Gazebo services are available):
   - **Step 1:** Add DetachableJoint plugin to URDF for Gazebo attachment:
     ```xml
     <gazebo>
       <plugin filename="gz-sim-detachable-joint-system" name="gz::sim::systems::DetachableJoint">
         <parent_link>left_gripper_magnet</parent_link>
         <child_model>{box_id}</child_model>
         <child_link>{box_id}_base_link</child_link>
         <topic>/model/{box_id}/detachable_joint</topic>
       </plugin>
     </gazebo>
     ```
   - **Step 2:** Creates SpawnEntity request (from `ros_gz_interfaces.srv.SpawnEntity`):
     - entity_factory.name = box_id (e.g., "box_l_1_2_3")
     - entity_factory.sdf = URDF string with DetachableJoint plugin (Gazebo Harmonic accepts URDF in SDF field)
     - entity_factory.allow_renaming = False
     - entity_factory.pose = get current pose of left_gripper_magnet via TF lookup
   - **Step 3:** Calls `/world/{world_name}/create` service (default: `/world/default/create`)
   - **Step 4:** Waits for Gazebo to confirm entity spawned
   - **Step 5:** Publishes Empty message to `/model/{box_id}/detachable_joint/attach` to engage electromagnet
     - DetachableJoint plugin creates fixed joint between `left_gripper_magnet` and `{box_id}_base_link`
     - This creates physics constraint so box moves with gripper in simulation
10. **ELSE (hardware mode)**: Skip Gazebo spawning - TF frames already available from Phase 2

**Phase 4: Visualization Markers (progress 90-100%)**
11. Publishes department marker array (red spheres + text labels D1, D2, ...) to /visualization_marker_array
12. Returns success=true with box_id

**When** I call DespawnBox with box_id
**Then** the node:
1. **IF in simulation mode**: Detach box from gripper first:
   - Publishes Empty message to `/model/{box_id}/detachable_joint/detach`
   - Wait 0.2 seconds for Gazebo to process detachment
2. Kills the robot_state_publisher node for this box (removes TF frames from TF tree)
3. Stops publishing static transform gripper_magnet → {box_id}_base_link
4. Deletes department markers from /visualization_marker_array
5. **IF in simulation mode**: Calls DeleteEntity service to remove box from Gazebo:
   ```python
   from ros_gz_interfaces.srv import DeleteEntity
   req = DeleteEntity.Request()
   req.entity.name = box_id
   req.entity.type = 2  # MODEL type (gz.msgs.Entity.Type.MODEL)
   # Call service: /world/default/remove
   ```
6. Removes box from internal tracking
7. Returns success=true

**And** department TF frames are available in TF tree: `{box_id}_dept_1_link`, `{box_id}_dept_2_link`, etc.
**And** TF frames persist at 10Hz via robot_state_publisher (no manual broadcasting needed)
**And** department markers include text labels (D1, D2, ...) above box for visualization
**And** active boxes are tracked internally with their robot_state_publisher PIDs for cleanup
**And** the approach works for both simulation (Gazebo) and hardware (TF frames exist without Gazebo)

**Prerequisites:** Story 1.1 (package structure)

**Technical Notes - ROS2 Jazzy + Gazebo Harmonic Specific:**

**Required Packages:**
- `ros_gz_interfaces` - Provides SpawnEntity and DeleteEntity services for Gazebo Harmonic
- `ros_gz_sim` - Gazebo Harmonic ROS2 bridge (provides `/world/{world_name}/create` and `/world/{world_name}/remove` services)
- `robot_state_publisher` - Standard ROS2 package for TF broadcasting from URDF

**Service Endpoints (Gazebo Harmonic):**
- Spawn: `/world/default/create` (type: `ros_gz_interfaces/srv/SpawnEntity`)
- Delete: `/world/default/remove` (type: `ros_gz_interfaces/srv/DeleteEntity`)
- World name may vary - check with `ros2 service list | grep create`

**Python Implementation Example:**
```python
from ros_gz_interfaces.srv import SpawnEntity, DeleteEntity
from geometry_msgs.msg import Pose

# Create service client
spawn_client = self.create_client(SpawnEntity, '/world/default/create')

# Prepare request
req = SpawnEntity.Request()
req.entity_factory.name = "box_l_1_2_3"
req.entity_factory.sdf = urdf_string  # URDF works in sdf field
req.entity_factory.allow_renaming = False
req.entity_factory.pose.position.x = 1.0
req.entity_factory.pose.position.y = 2.0
req.entity_factory.pose.position.z = 0.5

# Call service
future = spawn_client.call_async(req)
```

**References:**
- **Gazebo Harmonic Spawn:** [Spawn a Gazebo model from ROS 2](https://gazebosim.org/docs/harmonic/ros2_spawn_model/)
- **ros_gz_interfaces Package:** [ROS Package: ros_gz_interfaces](https://index.ros.org/p/ros_gz_interfaces/)
- **Service Examples:** [Object spawner for Gazebo Harmonic](https://robotics.stackexchange.com/questions/113266/object-spawner-and-despawner-service-for-conveyor-belt-in-gazebo-harmonic-gz-si)
- **URDF with robot_state_publisher:** [Using URDF with robot_state_publisher](https://docs.ros.org/en/humble/Tutorials/Intermediate/URDF/Using-URDF-with-Robot-State-Publisher.html)
- **ROS2 Jazzy + Harmonic Integration:** [Issue with ros2 jazzy and gazebo harmonic](https://community.gazebosim.org/t/issue-with-ros2-jazzy-and-gazebo-harmonic-ros-integration/2954)

**URDF Structure Example:**
```xml
<robot name="box_l_1_2_3">
  <link name="box_l_1_2_3_base_link">
    <visual><geometry><box size="0.4 0.6 0.3"/></geometry></visual>
    <collision><geometry><box size="0.4 0.6 0.3"/></geometry></collision>
    <inertial><mass value="0.5"/><inertia ixx="0.01" iyy="0.01" izz="0.01" ixy="0" ixz="0" iyz="0"/></inertial>
  </link>

  <link name="box_l_1_2_3_dept_1_link">
    <visual><geometry><sphere radius="0.01"/></geometry></visual>
  </link>

  <joint name="box_l_1_2_3_dept_1_joint" type="fixed">
    <parent link="box_l_1_2_3_base_link"/>
    <child link="box_l_1_2_3_dept_1_link"/>
    <origin xyz="0 0.05 0" rpy="0 0 0"/>  <!-- offset_y + 0 * dept_depth -->
  </joint>

  <!-- Repeat for dept_2 through dept_N with increasing Y offsets -->
</robot>
```

**Implementation Details:**

**Box Configuration (from storage_params.yaml):**
- Box dimensions: box_configurations (columns_4, columns_5, columns_6) × (rows_6, rows_8, rows_10, rows_12, rows_14)
- Department calculations: origin.y = offset_y + (dept_num - 1) × dept_depth from department_configurations

**Service Definitions:**
- Custom services: `srv/SpawnBox.srv`, `srv/DespawnBox.srv` (wrappers for user-friendly interface)
- Internal usage: `ros_gz_interfaces.srv.SpawnEntity` and `ros_gz_interfaces.srv.DeleteEntity`

**robot_state_publisher Launch:**
```python
import subprocess
cmd = [
    'ros2', 'run', 'robot_state_publisher', 'robot_state_publisher',
    '--ros-args',
    '-p', f'robot_description:={urdf_string}',
    '-p', 'use_sim_time:=true',
    '-p', f'frame_prefix:='  # No prefix, box_id already in link names
]
process = subprocess.Popen(cmd)
```

**Process Tracking:**
```python
self.active_boxes[box_id] = {
    "rsp_process": process,
    "urdf": urdf_string,
    "num_departments": num_departments
}
```

**Visualization:**
- Marker types: `Marker.SPHERE` (dept center, red, 0.02m radius) + `Marker.TEXT_VIEW_FACING` (label "D1", "D2", etc.)
- Update rate: 10 Hz (republish to maintain markers)

**Priority and Hardware/Simulation Mode:**

**PRIMARY (always executed):**
1. Generate URDF with department child links
2. Launch robot_state_publisher → **Department TF frames in TF tree**
3. Publish static transform world → box_base_link
4. **Result:** Department positions available for item picking via TF lookups

**SECONDARY (simulation only):**
5. Spawn visual box model in Gazebo (optional, for visualization)

**Critical Understanding:**
- **Hardware mode:** Steps 1-4 only → Department TF frames fully functional for real item detection and picking
- **Simulation mode:** Steps 1-5 → TF frames + visual representation in Gazebo
- **Department frames are the GOAL** - they provide precise 3D positions for items within the box
- **Gazebo spawning is OPTIONAL** - only for visual feedback during simulation testing

**Implementation Notes (Config Reuse):**
- **USE:** `manipulator_description/config/storage_params.yaml` - box dimensions, department_configurations
- **CREATE:** `manipulator_control/config/box_spawner.yaml` - NEW file for spawner-specific parameters:
  - `tf_broadcast_rate_hz`, `department_frame_prefix`, `gazebo.world_name`, `gazebo.allow_renaming`
- **DO NOT DUPLICATE:** Box dimensions or department calculations - load from storage_params.yaml

---

### Story 4A.4: Implement ExtractBox Action Server

As a robotics operator,
I want to extract boxes from cabinet addresses using coordinated YZ motion and electromagnet,
So that I can access items stored in boxes for picking operations.

**Acceptance Criteria:**

**Given** the ExtractBox action is defined with goal (side, cabinet_num, row, column, box_depth, extraction_speed) and result (success, box_extracted, magnet_engaged, box_id, message)
**When** I send ExtractBox goal for address (left, 1, 2, 3)
**Then** the action server executes the following phases:

**Phase 1: Navigate (progress 0-20%)**
- Call NavigateToAddress action client for target address
- Wait for navigation completion
- Feedback: current_phase="navigating"

**Phase 2: Approach (progress 20-40%)**
- Get current gripper Y/Z positions from /joint_states
- Generate YZ insertion trajectory to reach box (target_y = address_y + box_depth/2)
- Execute trajectory using YZTrajectoryGenerator
- Feedback: current_phase="approaching", gripper_depth=current_y

**Phase 3: Engage Magnet (progress 40-50%)**
- Call ToggleElectromagnet service with activate=true
- Wait 0.5 seconds for magnetic attachment
- Feedback: current_phase="engaging_magnet", magnet_contact_detected=true

**Phase 4: Spawn Box URDF (progress 50-60%) - CRITICAL TIMING**
- **AFTER electromagnet engages**, call SpawnBox service with address and num_departments (from cabinet config)
- Generate unique box_id: "box_{side}_{cabinet}_{row}_{col}"
- **SpawnBox creates:**
  - URDF with department child links
  - robot_state_publisher broadcasting TF transforms
  - Static transform: `left_gripper_magnet` → `{box_id}_base_link`
  - **Result:** Box is now attached to gripper magnet in TF tree
  - **Department frames are NOW available:** `{box_id}_dept_1_link`, `{box_id}_dept_2_link`, etc.
  - **These frames will move with the gripper** during extraction
- If simulation mode: Also spawns visual box model in Gazebo attached to gripper
- Mark address as extracted (red marker via state update)
- Feedback: current_phase="spawning_box", box_id="{box_id}"

**Phase 5: Extract (progress 60-90%)**
- Generate YZ extraction trajectory to safe position (safe_y=0.0)
- Execute trajectory at extraction_speed (default 0.08 m/s)
- **Box TF frames follow gripper** throughout extraction motion
- **Department frames remain accessible** for subsequent PickItem actions
- Feedback: current_phase="extracting"

**Phase 6: Complete (progress 90-100%)**
- Verify final position reached
- **Box is now extracted with all department frames available in TF tree**
- Feedback: current_phase="complete"

**And** result returns box_id, success=true, box_extracted=true, magnet_engaged=true
**And** action timeout: 45 seconds (configurable)
**And** action handles failures gracefully (e.g., magnet engagement failure aborts cleanly)

**Prerequisites:** Story 4A.1 (YZ trajectory generator), Story 4A.2 (electromagnet), Story 4A.3 (box spawner), Story 3.4 (NavigateToAddress)

**Technical Notes:**
- Reference architecture lines 431-479, 1613-1657 for ExtractBox logic
- Action definition: `action/ExtractBox.action`
- Box depth parameter: Distance to extend into cabinet (default 0.3m, from box_configurations)
- Get num_departments from storage_params cabinet configuration (e.g., 4x10x10 → 10 departments)
- NFR-004: 90% success rate for box extraction operations
- NFR-005: Complete extraction cycle < 60 seconds

**Implementation Notes (Config Reuse):**
- **USE:** `manipulator_control/config/action_servers.yaml` - add extract_box section (timeout_sec, approach_speed, extraction_speed)
- **USE:** `manipulator_control/config/trajectory.yaml` - trajectory generation parameters
- **USE:** `manipulator_control/config/electromagnet.yaml` - magnet engagement parameters
- **DO NOT CREATE:** Separate extract_box config - add section to action_servers.yaml

---

### Story 4A.5: Implement ReturnBox Action Server

As a robotics operator,
I want to return extracted boxes to their original cabinet addresses,
So that I can restore warehouse organization after item picking operations.

**Acceptance Criteria:**

**Given** the ReturnBox action is defined with goal (side, cabinet_num, row, column, box_id) and result (success, box_returned_successfully, message)
**When** I send ReturnBox goal for box previously extracted from address (left, 1, 2, 3)
**Then** the action server executes reverse extraction phases:

**Phase 1: Navigate (progress 0-25%)**
- Call NavigateToAddress for target address
- Feedback: current_operation="navigating"

**Phase 2: Insert (progress 25-60%)**
- Get current gripper Y/Z positions
- Get target address Y/Z from TF lookup
- Generate YZ insertion trajectory to place box at address depth
- Execute trajectory at slow speed (0.05 m/s for safety)
- Feedback: current_operation="inserting_box"

**Phase 3: Release (progress 60-70%)**
- Call ToggleElectromagnet with activate=false
- Wait 0.3 seconds for detachment
- Feedback: current_operation="releasing_magnet"

**Phase 4: Retract (progress 70-95%)**
- Generate YZ extraction trajectory to safe position
- Execute trajectory at normal speed (0.08 m/s)
- Feedback: current_operation="retracting"

**Phase 5: Cleanup (progress 95-100%)**
- Call DespawnBox service with box_id
- Remove red extracted marker for address
- Feedback: current_operation="completing"

**And** result returns success=true, box_returned_successfully=true
**And** action validates box_id matches expected format before proceeding
**And** action timeout: 45 seconds

**Prerequisites:** Story 4A.4 (ExtractBox for testing), Story 4A.1 (YZ trajectory), Story 4A.2 (electromagnet), Story 4A.3 (box spawner)

**Technical Notes:**
- Reference architecture section on ReturnBox (opposite of ExtractBox)
- Action definition: `action/ReturnBox.action`
- YZ insertion trajectory: Same as ExtractBox but executed in forward direction (Y+)
- Validate box exists before returning (check against active_boxes tracking)
- NFR-004: 90% success rate for return operations
- ReturnBox is prerequisite for Epic 6 complete workflow testing

**Implementation Notes (Config Reuse):**
- **USE:** `manipulator_control/config/action_servers.yaml` - add return_box section (timeout_sec, insertion_speed, retraction_speed)
- **USE:** `manipulator_control/config/trajectory.yaml` - trajectory generation parameters
- **USE:** `manipulator_control/config/electromagnet.yaml` - magnet release parameters
- **DO NOT CREATE:** Separate return_box config - add section to action_servers.yaml

---

## Epic 4B: Advanced Box Operations

**Goal:** Implement box relocation (PutBox) and loading station operations (MoveBoxToLoad) with address validation, enabling flexible box handling beyond simple extract/return cycles.

**Value Delivered:** Complete box management capability allowing box reorganization and external access at loading stations, supporting warehouse optimization workflows.

### Story 4B.1: Implement Address Validator Utility

As a developer,
I want a utility to validate addresses for box placement operations,
So that PutBox and MoveBoxToLoad actions can ensure safe and valid box relocations.

**Acceptance Criteria:**

**Given** storage_params.yaml defines cabinet configurations with column/row/department counts
**When** I call validate_address_empty(side, cabinet, row, column)
**Then** the utility:
1. Checks internal address occupancy tracker (maintained by spawn manager)
2. Returns (is_empty: bool, error_message: string)
3. Returns is_empty=false if address currently has box (spawned or not extracted)

**When** I call validate_box_width_compatibility(box_id, target_cabinet)
**Then** the utility:
1. Gets box column count from box_id or active_boxes tracking
2. Gets target cabinet column count from storage_params
3. Returns (compatible: bool, error_message: string)
4. Returns compatible=false if box columns ≠ cabinet columns (e.g., 4-column box in 5-column cabinet)

**And** utility provides get_address_occupancy_state() → dict of all address occupancy
**And** utility validates cabinet exists and row/column within bounds
**And** validation response time < 50ms

**Prerequisites:** Story 4A.3 (box spawner with tracking), Story 3.1 (address resolver)

**Technical Notes:**
- Reference architecture lines 1212-1303 for PutBox validation logic
- Utility class: AddressValidator in utils/address_validator.py
- Occupancy tracking: Maintain dict {(side, cabinet, row, col): box_id or None}
- Update occupancy on SpawnBox (mark occupied) and DespawnBox (mark empty)
- Box width compatibility: Critical for physical fit (4-col box = 0.06m width, 5-col = 0.055m width)
- Error messages: "Address (left, 1, 2, 3) is occupied by box_l_1_2_3", "Box width 4-col incompatible with cabinet 5-col"

**Implementation Notes (Config Reuse):**
- **USE:** `manipulator_description/config/storage_params.yaml` - cabinet configurations, box dimensions
- **DO NOT CREATE:** New config - validator uses storage_params.yaml

---

### Story 4B.2: Implement Occupancy Tracker in Box Spawn Manager

As a developer,
I want the box spawn manager to track address occupancy state,
So that address validation can prevent invalid box placement operations.

**Acceptance Criteria:**

**Given** the box_spawn_manager_node is running
**When** SpawnBox service is called successfully
**Then** the node updates occupancy tracker:
- Marks address (side, cabinet, row, column) as occupied by box_id
- Publishes occupancy state to `/manipulator/address_occupancy` (custom AddressOccupancy.msg) at 1 Hz

**When** DespawnBox service is called successfully
**Then** the node marks address as empty (None) in occupancy tracker

**And** node provides service `/manipulator/get_occupancy_state` returning all address states
**And** occupancy state persists in node memory (does not survive node restart)
**And** occupancy tracker initialized empty on startup (assumes all addresses empty initially)

**Prerequisites:** Story 4A.3 (box spawn manager)

**Technical Notes:**
- Occupancy data structure: Dict[(str, int, int, int), Optional[str]] = {(side, cab, row, col): box_id}
- Custom message: AddressOccupancy.msg with arrays of addresses and box_ids
- Service definition: `srv/GetOccupancyState.srv`
- Future enhancement: Persist occupancy to database for system restart recovery
- Integration point for Epic 6 complete workflow tracking

**Implementation Notes (Config Reuse):**
- **USE:** Box spawn manager internal state (from Story 4A.3)
- **DO NOT CREATE:** New config - occupancy is runtime state, not configuration

---

### Story 4B.3: Implement PutBox Action Server

As a robotics operator,
I want to place an extracted box into a different empty address,
So that I can reorganize warehouse storage or correct box placements.

**Acceptance Criteria:**

**Given** the PutBox action is defined with goal (box_id, target_side, target_cabinet_num, target_row, target_column) and result (success, box_placed, address_verified_empty, message)
**When** I send PutBox goal to move box_l_1_2_3 to address (left, 2, 3, 4)
**Then** the action server executes:

**Phase 1: Validation (progress 0-15%)**
- Call AddressValidator.validate_address_empty() for target address
- If occupied, abort with error "Target address occupied by {existing_box_id}"
- Call AddressValidator.validate_box_width_compatibility()
- If incompatible, abort with error "Box width mismatch"
- Feedback: current_phase="verifying_address"

**Phase 2: Navigate (progress 15-35%)**
- Call NavigateToAddress for target address
- Feedback: current_phase="navigating"

**Phase 3: Insert (progress 35-70%)**
- Generate YZ insertion trajectory to place box at target address depth
- Execute trajectory at approach_speed=0.05 m/s (slow for safety)
- Feedback: current_phase="extending_gripper", current_pose updates

**Phase 4: Release (progress 70-80%)**
- Call ToggleElectromagnet with activate=false
- Wait 0.3 seconds for detachment
- Feedback: current_phase="releasing"

**Phase 5: Retract and Cleanup (progress 80-100%)**
- Generate YZ extraction trajectory to safe position
- Execute trajectory
- Call DespawnBox to remove visual box from old position
- Update occupancy tracker: target address occupied, source address empty
- Feedback: current_phase="retracting"

**And** result returns success=true, box_placed=true, address_verified_empty=true
**And** action timeout: 60 seconds
**And** failed validation aborts immediately without motion

**Prerequisites:** Story 4B.1 (address validator), Story 4B.2 (occupancy tracker), Story 4A.4 (ExtractBox for obtaining box), Story 4A.1 (YZ trajectory)

**Technical Notes:**
- Reference architecture lines 1212-1303 for PutBox implementation
- Action definition: `action/PutBox.action`
- Validation must happen BEFORE any motion (fail fast principle)
- Cabinet column compatibility: 4x10 box only fits in 4x10, 4x6 cabinets (same column count)
- Example cabinet configs: Left row has 4x10x10, 4x10x10, 4x6x10, 5x12x14
- Use YZTrajectoryGenerator.generate_insertion_trajectory() for placement
- NFR-006: Box relocation operations complete within 90 seconds

**Implementation Notes (Config Reuse):**
- **USE:** `manipulator_control/config/action_servers.yaml` - add put_box section (timeout_sec)
- **USE:** `manipulator_control/config/trajectory.yaml` - trajectory generation parameters
- **USE:** `manipulator_control/config/electromagnet.yaml` - magnet release parameters
- **DO NOT CREATE:** Separate put_box config - add section to action_servers.yaml

---

### Story 4B.4: Implement MoveBoxToLoad Action Server

As a robotics operator,
I want to move boxes to loading stations and optionally return them to storage,
So that I can enable external access for inspection, packing, or inventory operations.

**Acceptance Criteria:**

**Given** the MoveBoxToLoad action is defined with goal (source_side, source_cabinet_num, source_row, source_column, load_position_name, return_to_storage, return_side, return_cabinet_num, return_row, return_column) and result (success, box_extracted, box_at_load_position, box_returned, box_id, message)
**When** I send MoveBoxToLoad goal with source=(left, 1, 2, 3), load_position="load_station_left", return_to_storage=true, return=(left, 1, 5, 1)
**Then** the action server executes composite workflow:

**Phase 1: Extract from Source (progress 0-35%)**
- Call ExtractBox action client for source address
- Wait for completion
- Store box_id from result
- Feedback: current_operation="extracting", progress updates

**Phase 2: Move to Load Position (progress 35-55%)**
- Load coordinates from config/load_positions.yaml for load_position_name
- Call MoveJointGroup for navigation group with (x, z) coordinates
- Feedback: current_operation="moving_to_load", elapsed_time updates

**Phase 3: At Load Position (progress 55-60%)**
- Pause at load position (configurable wait time or external signal)
- Feedback: current_operation="at_load_position"
- Box accessible for external operations

**Phase 4: Return to Storage (progress 60-100%, optional)**
- If return_to_storage=true and return_cabinet_num>0:
  - Call PutBox action with return address and box_id
  - Feedback: current_operation="returning"
- Else if return_to_storage=true and return_cabinet_num=0:
  - Call ReturnBox action to original source address
  - Feedback: current_operation="returning"
- Else:
  - Skip return phase
  - Feedback: current_operation="completed"

**And** result returns all phase completion statuses and final box_id
**And** action timeout: 120 seconds (extended for complete workflow)
**And** if ExtractBox fails, action aborts immediately
**And** if return fails, action returns box_at_load_position=true, box_returned=false with descriptive error

**Prerequisites:** Story 4B.3 (PutBox), Story 4A.4 (ExtractBox), Story 4A.5 (ReturnBox)

**Technical Notes:**
- Reference architecture lines 1306-1466 for MoveBoxToLoad workflow
- Action definition: `action/MoveBoxToLoad.action`
- Configuration: `config/load_positions.yaml` with predefined load station coordinates
- Example load positions: load_station_left (x=0.5, z=1.0), load_station_right (x=3.5, z=1.0), inspection_area (x=2.0, z=1.2)
- Return logic: return_cabinet_num=0 means return to source, >0 means relocate to new address
- NFR-006: Complete move-to-load-and-return cycle < 150 seconds
- Use case: External system inspection, packing operations, inventory verification

**Implementation Notes (Config Reuse):**
- **USE:** `manipulator_control/config/action_servers.yaml` - add move_box_to_load section (timeout_sec)
- **CREATE:** `manipulator_control/config/load_positions.yaml` - NEW file for load station coordinates:
  - `positions` (load_station_left, load_station_right, inspection_area)
- **DO NOT DUPLICATE:** Source address handling - uses existing ExtractBox, ReturnBox, PutBox actions

---

## Epic 5: Item Picking & Department Frames

**Goal:** Implement department frame generation with TF broadcasting, limit switch-based picker state machine, container jaw manipulation, and complete container retrieval workflow.

**Value Delivered:** Ability to pick items from specific departments within extracted boxes using state-machine-driven picker control, with containers retrieved and placed for item storage.

### Story 5.1: Enhance Box Spawner with Department Frame Broadcasting

As a developer,
I want extracted boxes to generate TF frames for each department at 10 Hz,
So that PickItem action can navigate picker to precise department locations.

**Acceptance Criteria:**

**Given** a box is spawned with num_departments=10 via SpawnBox service
**When** the box_spawn_manager_node is running
**Then** the node broadcasts TF frames at 10 Hz for each department:
- Frame names: `{box_id}_dept_1` through `{box_id}_dept_10`
- Parent frame: `{box_id}_link`
- Positions: Calculated from storage_params department_configurations
  - Y-axis offset: offset_y + (dept_num - 1) * step_y
  - X-axis: 0.0 (centered in box width)
  - Z-axis: 0.0 (centered in box height)

**And** department frames are verifiable using `ros2 run tf2_ros tf2_echo {box_id}_link {box_id}_dept_5`
**And** TF tree shows all department frames as children of box_link
**And** frames stop broadcasting when DespawnBox is called
**And** broadcasting timer runs at 10 Hz (0.1 second period)

**Prerequisites:** Story 4A.3 (basic box spawner)

**Technical Notes:**
- Reference architecture lines 699-729 for department TF broadcasting
- Use tf2_ros.TransformBroadcaster.sendTransform() with TransformStamped array
- Department parameters from storage_params.yaml department_configurations (departments_10, departments_14, departments_16)
- Example: departments_10 has depth=0.02m, offset_y=0.005m, step_y=0.024m
- Store timer handle in active_boxes[box_id]['tf_timer'] for cleanup
- Frame timestamps: self.get_clock().now().to_msg() for each broadcast

**Implementation Notes (Config Reuse):**
- **USE:** `manipulator_description/config/storage_params.yaml` - department_configurations (offset_y, step_y, depth)
- **USE:** `manipulator_control/config/box_spawner.yaml` - tf_broadcast_rate_hz (from Story 4A.3)
- **DO NOT DUPLICATE:** Department calculations - load from storage_params.yaml

---

### Story 5.2: Add Department Markers to Visualization

As a developer,
I want visual markers in RViz showing department centers and labels,
So that I can verify department frame positioning and observe picker targets.

**Acceptance Criteria:**

**Given** a box is spawned with departments
**When** the box_spawn_manager_node publishes markers
**Then** MarkerArray includes for each department:

**Department Center Marker:**
- Type: Marker.SPHERE
- Position: Department center (frame {box_id}_dept_{num} position)
- Size: 0.015m diameter
- Color: Red (1.0, 0.0, 0.0) with alpha=0.6 (semi-transparent)
- Namespace: "{box_id}_departments"

**Department Label Marker:**
- Type: Marker.TEXT_VIEW_FACING
- Position: Above box (z = box_height/2 + 0.02m)
- Text: "D{dept_num}" (e.g., "D1", "D5", "D10")
- Size: 0.02m text height
- Color: White (1.0, 1.0, 1.0) with alpha=1.0
- Namespace: "{box_id}_dept_labels"

**And** markers are published at 10 Hz to `/visualization_marker_array`
**And** markers are visible in RViz when MarkerArray display enabled
**And** markers deleted (action=DELETE) when DespawnBox called
**And** marker IDs are unique per department (derived from dept_num)

**Prerequisites:** Story 5.1 (department TF frames), Story 2.4 (marker publisher foundation)

**Technical Notes:**
- Reference architecture lines 730-789 for department marker implementation
- Marker header.frame_id = "{box_id}_link" for all department markers
- Department center calculation: Uses same offset_y + step_y logic as TF frames
- Text marker positioned above box to avoid occlusion by box geometry
- Color scheme: Red semi-transparent spheres match "target location" visual convention
- Marker lifetimes: 0 (persistent) since managed by explicit DELETE on despawn

**Implementation Notes (Config Reuse):**
- **USE:** `manipulator_control/config/visualization.yaml` - add department_centers, dept_labels sections (marker colors, sizes, label_height)
- **USE:** `manipulator_description/config/storage_params.yaml` - box dimensions for label positioning
- **DO NOT CREATE:** Separate marker config - extend visualization.yaml from Story 2.4

---

### Story 5.3: Implement ManipulateContainer Action for Jaw Control

As a robotics operator,
I want to control container jaw opening and closing synchronously,
So that I can grip, hold, and release containers for item storage operations.

**Acceptance Criteria:**

**Given** the ManipulateContainer action is defined with goal (operation, jaw_opening) and result (success, final_opening, message)
**When** I send goal with operation="OPEN", jaw_opening=0.3
**Then** the action server:
1. Calculates symmetric jaw positions: left_target = -0.15, right_target = +0.15
2. Commands both jaws simultaneously using ControllerInterface
3. Monitors jaw positions from /joint_states
4. Publishes feedback (current_jaw_positions, opening_width, progress_percent) at 10 Hz
5. Returns success when both jaws within 0.01m of target

**When** I send operation="CLOSE", jaw_opening=0.1
**Then** jaws move to left=-0.05, right=+0.05 (synchronized closure)

**When** I send operation="RELEASE_ITEM"
**Then** jaws open slightly from current position (increment by 0.05m) to drop item

**And** software mimic ensures synchronized motion (both jaws reach target within 1 second of each other)
**And** action timeout: 15 seconds
**And** jaw opening range validated: 0.0m (closed) to 0.4m (max open per joint limits)

**Prerequisites:** Story 2.2 (ControllerInterface)

**Technical Notes:**
- Reference architecture lines 519-558 for ManipulateContainer implementation
- Action definition: `action/ManipulateContainer.action`
- Joint names: selector_left_container_jaw_joint, selector_right_container_jaw_joint
- Software mimic formula: left = -opening/2, right = +opening/2 (symmetric around center)
- Synchronization check: abs(left_position + opening/2) < 0.01 AND abs(right_position - opening/2) < 0.01
- Joint limits: Both jaws -0.2 to +0.2 (±0.2m range), so max opening = 0.4m total
- Hardware note: Real hardware has mechanical mimic, software mimic only for Gazebo simulation

**Implementation Notes (Config Reuse):**
- **USE:** `manipulator_control/config/action_servers.yaml` - add manipulate_container section (timeout_sec, sync_tolerance, sync_timeout_sec)
- **USE:** `manipulator_control/config/kinematic_chains.yaml` - container joint group definition (from Story 2.5)
- **USE:** ControllerInterface (Story 2.2) - for joint commands
- **DO NOT DUPLICATE:** Joint names or limits - use kinematic_chains.yaml

---

### Story 5.4: Implement GetContainer and PlaceContainer Actions

As a robotics operator,
I want to retrieve containers from storage locations and place them back,
So that I can use containers to collect picked items during warehouse operations.

**Acceptance Criteria:**

**Given** the GetContainer action is defined with goal (container_id, container_location, container_width) and result (success, container_retrieved, jaws_gripping, message)
**When** I send GetContainer goal with container_location="storage_left_1", container_width=0.3
**Then** the action server executes:

**Phase 1: Open Jaws (progress 0-20%)**
- Call ManipulateContainer with operation="OPEN", jaw_opening=container_width+0.05 (margin)
- Feedback: current_phase="opening_jaws"

**Phase 2: Navigate (progress 20-50%)**
- Load coordinates from config/container_storage.yaml for location
- Call MoveJointGroup for navigation group to position at container
- Feedback: current_phase="navigating"

**Phase 3: Position Selector (progress 50-70%)**
- Lower selector to container height (from config)
- Align jaws around container
- Feedback: current_phase="positioning"

**Phase 4: Close Jaws (progress 70-90%)**
- Call ManipulateContainer with operation="CLOSE", jaw_opening=container_width
- Verify grip using jaw position feedback
- Feedback: current_phase="closing_jaws"

**Phase 5: Lift (progress 90-100%)**
- Raise selector by 0.2m to lift container off support
- Verify container lifted (Z position increased)
- Feedback: current_phase="lifting"

**And** result returns container_retrieved=true, jaws_gripping=true

**Given** the PlaceContainer action is defined with goal (container_location) and result (success, container_placed, message)
**When** I send PlaceContainer goal
**Then** action executes reverse workflow: Navigate → Lower → Open Jaws → Raise selector

**Prerequisites:** Story 5.3 (ManipulateContainer), Story 3.4 (navigation)

**Technical Notes:**
- Reference architecture lines 465-541 for GetContainer/PlaceContainer workflows
- Action definitions: `action/GetContainer.action`, `action/PlaceContainer.action`
- Configuration: `config/container_storage.yaml` with predefined storage locations
- Example locations: storage_left_1 (x=-0.5, y=0.3, z=0.8, max_width=0.35), storage_right_1 (x=4.5, y=-0.3, z=0.8)
- Container detection: Verify jaw positions indicate object gripped (resistance/position feedback)
- Lift height: 0.2m above storage position to clear support and verify grip
- Action timeouts: GetContainer 30s, PlaceContainer 25s

**Implementation Notes (Config Reuse):**
- **USE:** `manipulator_control/config/action_servers.yaml` - add get_container, place_container sections (timeout_sec, jaw_margin, lift_height)
- **CREATE:** `manipulator_control/config/container_storage.yaml` - NEW file for container locations:
  - `storage_locations` (position, max_width for each), `slot_config` (default_max_slots, slot_margin, insertion_depth)
- **USE:** ManipulateContainer action (Story 5.3) for jaw control

---

### Story 5.5: Implement PickItem Action with Limit Switch State Machine (CORRECTED)

As a robotics operator,
I want to pick items from specific departments in extracted boxes using limit switch feedback,
So that I can retrieve items reliably without precise position encoders.

**CRITICAL WORKFLOW CORRECTION:**
The picker does NOT extend into the box - it LOWERS into it (Z-axis). The extension mechanism (X-axis) is used AFTER lifting to position the item OVER the container for release.

**Acceptance Criteria:**

**Given** the PickItem action is defined with goal (box_id, department_num, grasp_type) and result (success, item_grasped, message)
**When** I send PickItem goal with box_id="box_l_1_2_3", department_num=5
**Then** the action server executes state machine:

**State 1: POSITION_Y (progress 0-15%)**
- Get department frame transform: {box_id}_dept_5
- Calculate picker Y position to align with department center
- Move `picker_frame_picker_rail_joint` (Y-axis) to department position
- Transition: When Y position reached (within 0.02m)
- Monitor: `picker_rail_min/max` for safety
- Feedback: current_phase="positioning_y"

**State 2: OPEN_JAW (progress 15-25%)**
- Command `picker_base_picker_jaw_joint` to open (position toward 0.19, max limit)
- Monitor `/manipulator/end_switches/picker_jaw_opened` (trigger at X=0.19)
- Transition: When picker_jaw_opened=TRUE
- Feedback: current_phase="opening_jaw"

**State 3: LOWER_PICKER (progress 25-40%)**
- Command `selector_frame_picker_frame_joint` to lower (Z-axis negative, toward min)
- Monitor `/manipulator/end_switches/picker_frame_min` (trigger at Z=0.005)
- Transition: When picker_frame_min=TRUE or target Z reached
- Feedback: current_phase="lowering_picker"

**State 4: CLOSE_JAW (progress 40-55%)**
- Command `picker_base_picker_jaw_joint` to close (position toward 0.01, min limit)
- Monitor `/manipulator/end_switches/picker_jaw_closed` (trigger at X=0.01)
- Transition: When picker_jaw_closed=TRUE (item grasped)
- Feedback: current_phase="grasping"

**State 5: LIFT_PICKER (progress 55-70%)**
- Command `selector_frame_picker_frame_joint` to raise (Z-axis positive, toward max)
- Monitor `/manipulator/end_switches/picker_frame_max` (trigger at Z=0.295) or position
- Transition: When clear of box (picker_frame_max=TRUE or target Z reached)
- Feedback: current_phase="lifting_picker"

**State 6: EXTEND_PICKER (progress 70-80%)**
- Command `picker_rail_picker_base_joint` to extend (X-axis positive, toward max)
- Monitor `/manipulator/end_switches/picker_extended` (trigger at X=0.24)
- Transition: When picker_extended=TRUE (positioned over container)
- Feedback: current_phase="extending_picker"

**State 7: RELEASE_ITEM (progress 80-90%)**
- Command `picker_base_picker_jaw_joint` to open (position toward 0.19)
- Monitor `/manipulator/end_switches/picker_jaw_opened` (trigger at X=0.19)
- Transition: When picker_jaw_opened=TRUE (item released into container)
- Feedback: current_phase="releasing_item"

**State 8: RETRACT_PICKER (progress 90-100%)**
- Command `picker_rail_picker_base_joint` to retract (X-axis negative, toward min)
- Monitor `/manipulator/end_switches/picker_retracted` (trigger at X=0.01)
- Transition: When picker_retracted=TRUE (home position)
- Feedback: current_phase="retracting_picker"

**And** each state has timeout (5 seconds per state, 45 seconds total action timeout)
**And** state transitions only on limit switch events (not position thresholds)
**And** if any state times out, action aborts with descriptive error
**And** result returns success=true, item_grasped=true if state machine completes

**Prerequisites:** Story 2.1 (virtual limit switches), Story 5.1 (department frames), Story 2.2 (ControllerInterface)

**Technical Notes:**
- Reference architecture "Core Physical Workflows Reference" section, Workflow 5
- Action definition: `action/PickItem.action`
- State machine implementation: Use rclpy state machine or simple enum-based states
- **Correct Limit Switch Names:**
  - `picker_jaw_opened` (X=0.19) / `picker_jaw_closed` (X=0.01) - jaw grasp
  - `picker_extended` (X=0.24) / `picker_retracted` (X=0.01) - picker X-axis extension
  - `picker_frame_min` (Z=0.005) / `picker_frame_max` (Z=0.295) - picker Z-axis vertical
  - `picker_rail_min` (Y=-0.29) / `picker_rail_max` (Y=+0.29) - picker Y-axis rail
- **Joint to Movement Mapping:**
  - `selector_frame_picker_frame_joint` (Z) - Lower INTO box / Lift OUT of box
  - `picker_frame_picker_rail_joint` (Y) - Position along department axis
  - `picker_rail_picker_base_joint` (X) - Extend OVER container after lift
  - `picker_base_picker_jaw_joint` (X) - Open/close jaw for grasp
- Department center coordinates: From TF lookup of {box_id}_dept_{department_num}
- NFR-007: Item picking operations 85%+ success rate (validated in Epic 6 testing)

**Implementation Notes (Config Reuse):**
- **USE:** `manipulator_control/config/limit_switches.yaml` - switch trigger positions and joint mappings
- **USE:** `manipulator_control/config/kinematic_chains.yaml` - picker joint group definition (from Story 2.5)
- **USE:** `manipulator_control/config/action_servers.yaml` - add pick_item section (timeout_sec, state_timeout_sec)
- **CREATE:** `manipulator_control/config/pick_item_states.yaml` - NEW file for state machine configuration:
  - `states` (each state: joint, switch, target), references limit_switches.yaml switch names
- **DO NOT DUPLICATE:** Switch positions or joint limits - reference existing configs

---

### Story 5.6: Validate Department Frame Positioning and Picker Reach

As a developer,
I want to validate that department frames are correctly positioned and reachable by picker,
So that PickItem action can reliably target all departments in all box configurations.

**Acceptance Criteria:**

**Given** boxes with different department counts are spawned (10-dept, 14-dept, 16-dept configurations)
**When** I query department frame positions using TF
**Then** for each box type, department frames are:
- Equally spaced along Y-axis per step_y from storage_params
- Within picker reachable workspace (picker_frame_picker_rail_joint range -0.3 to +0.3m)
- Centered in X-axis within box width
- Verifiable with `ros2 run tf2_ros tf2_echo` showing correct transforms

**And** when I send PickItem goals targeting departments 1, middle, and last for each box type
**Then** picker successfully reaches all departments without joint limit violations

**And** test script `test/test_epic5_department_frames.py` validates:
1. Department frame positions match calculated values from storage_params
2. All departments reachable (TF transform lookup succeeds)
3. Picker motion to each department completes without errors
4. Frame spacing consistent (dept_2 - dept_1 = step_y)

**And** test results show 100% frame position accuracy and 90%+ picker reach success

**Prerequisites:** Story 5.1 (department frames), Story 5.5 (PickItem action)

**Technical Notes:**
- Test box configurations: 4x10x10 (10 dept), 5x12x14 (14 dept), 6x14x16 (16 dept)
- Department spacing validation: Measure transform differences between consecutive departments
- Picker workspace: Y-axis (-0.3 to +0.3m from selector frame), X-axis (0 to 0.12m extension), Z-axis (-0.01 to 0.3m vertical)
- Edge case testing: First department (dept_1) and last department (dept_10/14/16)
- Storage params reference: department_configurations section with offset_y and step_y values
- Use pytest with launch_testing for Gazebo simulation integration

**Implementation Notes (Config Reuse):**
- **USE:** `manipulator_description/config/storage_params.yaml` - department_configurations, box dimensions
- **USE:** `manipulator_description/config/manipulator_params.yaml` - picker joint limits
- **DO NOT CREATE:** New config - tests use existing configs

---

### Story 5.7: Create PlaceItemInContainer Action

As a robotics operator,
I want to place picked items into retrieved containers,
So that I can collect multiple items during warehouse picking operations.

**Acceptance Criteria:**

**Given** the PlaceItemInContainer action is defined with goal (container_id, slot_number) and result (success, item_placed, message)
**When** I send goal after PickItem succeeds (item in picker jaw)
**Then** the action server:

**Phase 1: Position Above Container (progress 0-40%)**
- Calculate container slot position based on slot_number and container geometry
- Move picker to position above container slot
- Feedback: current_phase="positioning_above_container"

**Phase 2: Lower Picker (progress 40-60%)**
- Lower picker Z-axis to insert item into container
- Stop at calculated depth (container opening - margin)
- Feedback: current_phase="lowering_item"

**Phase 3: Open Jaw (progress 60-80%)**
- Command picker jaw to open (release item)
- Monitor picker_jaw_opened switch
- Feedback: current_phase="releasing_item"

**Phase 4: Retract Picker (progress 80-100%)**
- Raise picker Z-axis to clear container
- Return picker to safe position
- Feedback: current_phase="retracting"

**And** result returns success=true, item_placed=true
**And** action validates container is retrieved (jaws gripping) before attempting placement
**And** action timeout: 20 seconds

**Prerequisites:** Story 5.5 (PickItem), Story 5.4 (GetContainer for container retrieval)

**Technical Notes:**
- Action definition: `action/PlaceItemInContainer.action`
- Container slot calculation: Divide container width by max_slots (e.g., 0.3m / 6 slots = 0.05m per slot)
- Container position: From selector jaw positions and container_width parameter
- Picker jaw opening: Command to opened position (0.01) to release item
- Integration with PickItem: Item must be in picker jaw (picker_jaw_closed=true) at start
- Future enhancement: Add item tracking in container slots (Epic 6)

**Implementation Notes (Config Reuse):**
- **USE:** `manipulator_control/config/action_servers.yaml` - add place_item_in_container section (timeout_sec)
- **USE:** `manipulator_control/config/container_storage.yaml` - slot_config (slot_margin, default_max_slots)
- **USE:** `manipulator_control/config/limit_switches.yaml` - picker_jaw_opened switch for release confirmation
- **DO NOT DUPLICATE:** Container dimensions - use jaw positions to calculate slot positions

---

### Story 5.8: Create Epic 5 Integration Test Suite

As a developer,
I want comprehensive tests validating item picking workflow with departments,
So that I can ensure picker state machine and department frame system work reliably.

**Acceptance Criteria:**

**Given** Epic 5 stories 5.1-5.7 are implemented
**When** I run `test/test_epic5_item_picking.py`
**Then** the test suite executes:

**Test 1: Department Frame Validation**
- Spawn box with 10 departments
- Verify all 10 department TF frames exist and positioned correctly
- Verify department markers visible in RViz

**Test 2: Picker State Machine**
- Send PickItem goal for department 5
- Monitor state transitions via feedback
- Verify all 6 states execute in order
- Verify limit switches trigger at correct states

**Test 3: Container Operations**
- GetContainer from storage_left_1
- Verify jaws grip container
- Verify container lifted
- PlaceContainer back to storage
- Verify container released

**Test 4: Complete Pick-and-Place**
- GetContainer
- ExtractBox from address
- PickItem from department 5
- PlaceItemInContainer
- Verify complete workflow succeeds

**Test 5: Multi-Department Picking**
- Pick from departments 1, 5, 10 sequentially
- Verify picker reaches all departments
- Verify no joint limit violations

**And** test results show:
- 100% department frame positioning accuracy
- 85%+ picker state machine success rate (NFR-007)
- 90%+ complete workflow success rate
- Execution time < 90 seconds per complete pick operation

**And** test documentation includes failure analysis and troubleshooting guide

**Prerequisites:** Stories 5.1-5.7

**Technical Notes:**
- Use pytest with launch_testing, requires Gazebo simulation running
- Test timeout: 15 minutes (multiple workflows with Gazebo)
- Monitor limit switch topics to validate state transitions
- Record bag files of successful runs for regression testing
- NFR-007: Item picking accuracy 85%+ for successful grasps
- Integration with Epic 6: This validates building blocks for high-level PickItemFromStorage workflow

**Implementation Notes (Config Reuse):**
- **USE:** Existing configurations from Stories 5.1-5.7
- **DO NOT CREATE:** New config - tests use existing configs

---

## Epic 6: High-Level Workflows & System Integration

**Goal:** Implement complete autonomous pick-from-storage workflow, comprehensive error handling, system-wide reliability testing, performance profiling, and complete documentation for Level 2 integration readiness.

**Value Delivered:** Production-ready Level 3 control system with validated 85%+ end-to-end success rate, complete observability via RQt tools, and ready for Level 2 bridge integration.

### Story 6.1: Implement PickItemFromStorage High-Level Action

As a robotics operator,
I want to execute a complete autonomous pick operation from a warehouse address,
So that I can retrieve items with a single high-level command integrating all subsystems.

**Acceptance Criteria:**

**Given** the PickItemFromStorage action is defined with goal (side, cabinet_num, row, column, department_num, container_id) and result (success, item_picked, item_stored_in_container, box_returned, total_time, message)
**When** I send goal for (left, 1, 2, 3, dept 5, container "CONT_A")
**Then** the action server orchestrates complete workflow:

**Phase 1: Get Container (progress 0-15%)**
- Call GetContainer with container_id
- If no container specified, skip to Phase 2
- Verify container retrieved
- Feedback: current_operation="retrieving_container", elapsed_time updates

**Phase 2: Navigate to Box Address (progress 15-25%)**
- Call NavigateToAddress for (side, cabinet, row, column)
- Feedback: current_operation="navigating_to_address"

**Phase 3: Extract Box (progress 25-45%)**
- Call ExtractBox for address
- Verify box extracted and spawned with departments
- Store box_id from result
- Feedback: current_operation="extracting_box"

**Phase 4: Position for Picking (progress 45-55%)**
- Adjust manipulator position for picker workspace reach
- May require NavigateToAddress with offset or MoveJointGroup adjustment
- Feedback: current_operation="positioning_for_picking"

**Phase 5: Pick Item (progress 55-75%)**
- Call PickItem with box_id, department_num
- Verify item grasped (picker_jaw_closed=true)
- Feedback: current_operation="picking_item"

**Phase 6: Place in Container (progress 75-85%)**
- If container retrieved in Phase 1:
  - Call PlaceItemInContainer with container_id
  - Verify item placed
- Feedback: current_operation="placing_item_in_container"

**Phase 7: Return Box (progress 85-100%)**
- Call ReturnBox with box_id to original address
- Verify box returned and despawned
- Feedback: current_operation="returning_box"

**And** result returns:
- success=true if all phases complete
- item_picked=true if Phase 5 succeeds
- item_stored_in_container=true if Phase 6 succeeds (or N/A if no container)
- box_returned=true if Phase 7 succeeds
- total_time=duration of complete operation

**And** action timeout: 180 seconds (3 minutes for complete workflow)
**And** if any phase fails, execute recovery logic (see Story 6.2)
**And** action publishes detailed feedback at 2 Hz with phase, operation description, progress, and elapsed time

**Prerequisites:** All Epic 5 actions (GetContainer, ExtractBox, PickItem, PlaceItemInContainer, ReturnBox)

**Technical Notes:**
- Reference architecture lines 317-359 for PickItemFromStorage high-level workflow
- Action definition: `action/PickItemFromStorage.action`
- Phase progress allocation: Weighted by expected duration (extraction and picking longest phases)
- Positioning for picking: Picker Y-axis reach is -0.3 to +0.3m from selector frame, may need rail adjustment to center box in workspace
- Error handling: Each sub-action failure triggers specific recovery (Story 6.2)
- NFR-001: Complete pick cycle < 120 seconds target, 180 seconds max timeout
- NFR-008: 85%+ success rate for complete autonomous pick operations

**Implementation Notes (Config Reuse):**
- **USE:** `manipulator_control/config/action_servers.yaml` - add pick_item_from_storage section (timeout_sec)
- **USE:** All existing action server configs for sub-action calls
- **DO NOT CREATE:** Separate high-level config - reuses existing action configurations

---

### Story 6.2: Implement Comprehensive Error Handling and Recovery

As a developer,
I want robust error handling for all action servers with automatic recovery strategies,
So that transient failures don't require manual intervention and system state remains safe.

**Acceptance Criteria:**

**Given** PickItemFromStorage or any mid-level action encounters a failure
**When** the failure is transient (timeout, sensor glitch, communication loss)
**Then** the action server implements retry logic:
- Retry up to 2 times with exponential backoff (1s, 2s delays)
- Log retry attempts at WARN level
- If all retries fail, proceed to recovery strategy

**When** NavigateToAddress fails (positioning error > 0.02m)
**Then** retry navigation once with reduced velocity (50% of default)

**When** ExtractBox fails at magnet engagement phase
**Then** retract gripper, re-attempt approach trajectory, retry magnet engagement once

**When** PickItem state machine times out in any state
**Then** reset picker to safe position (all joints to mid-range), log state timeout, abort action

**When** any failure occurs after box extraction (Phases 4-7 in PickItemFromStorage)
**Then** mandatory recovery: Call ReturnBox to restore warehouse state before aborting

**And** all error messages include:
- Error code (from error code enumeration)
- Descriptive message
- Phase/state where error occurred
- Suggested recovery action for operator

**And** error recovery success rate: 60%+ of transient failures recovered automatically
**And** no action failure leaves manipulator in unsafe state (joints at limits, box extracted without return)

**Prerequisites:** Story 6.1 (PickItemFromStorage for testing recovery)

**Technical Notes:**
- Reference architecture lines 1393-1433 for error handling patterns
- Error codes enumeration: 1000-1099 (navigation), 1100-1199 (extraction), 1200-1299 (picking), 1300-1399 (container), 1400-1499 (magnet/sensor)
- Retry logic: Use rclpy.Rate for exponential backoff delays
- Safe state definition: All joints mid-range, magnet OFF, no extracted boxes (or returned)
- Log all errors at ERROR level with full context (action name, goal parameters, error details)
- Recovery testing: Inject failures in test suite to validate recovery paths

**Implementation Notes (Config Reuse):**
- **CREATE:** `manipulator_control/config/error_handling.yaml` - NEW file for error handling configuration:
  - `retry_policy` (max_retries, backoff_delays_sec), `error_codes`, `recovery_strategies`
- **USE:** All existing action server configs for retry parameters
- **DO NOT DUPLICATE:** Action timeouts - reference from action_servers.yaml

---

### Story 6.3: Add Complete State Markers and Observability

As a developer,
I want comprehensive visual markers and status topics for full system observability,
So that developers and operators can monitor all aspects of manipulator state in RViz and RQt.

**Acceptance Criteria:**

**Given** the manipulator control system is running
**When** I view RViz with MarkerArray display enabled
**Then** all markers are visible and updating:

**Existing Markers (from previous epics):**
- Magnet engaged: Red sphere on gripper (Epic 2)
- Target address: Green cube (Epic 3)
- Extracted addresses: Red cubes (Epic 3)
- Department centers: Red spheres with labels (Epic 5)

**New Markers for Epic 6:**
- **Container Gripped:** Green ring around container jaws when jaws gripping (diameter matches jaw opening)
- **Picker Active:** Yellow sphere at picker jaw location when PickItem action active
- **Current Action:** Text marker above manipulator showing action name and phase (e.g., "PickItemFromStorage: extracting_box 35%")
- **Error State:** Large red "X" marker above manipulator when action fails (persists 5 seconds)

**And** system publishes ManipulatorState message at 10 Hz to `/manipulator/state` including:
- current_action: string (action name or "idle")
- action_status: string ("idle", "executing", "paused", "error", "completed")
- joint_positions: float64[] (all 9 joints)
- joint_velocities: float64[] (all 9 joints)
- current_address: (side, cabinet, row, column) or null
- box_extracted: bool
- box_id: string or empty
- item_in_picker: bool (picker_jaw_closed switch state)
- magnet_engaged: bool
- container_gripped: bool (jaw positions indicate grip)
- active_warnings: string[] (e.g., "Approaching joint limit", "Low positioning accuracy")
- errors: string[] (current error messages)

**And** all markers and state updates are observable via `ros2 topic echo` and RViz
**And** marker update rate: 10 Hz for real-time responsiveness

**Prerequisites:** Story 2.4 (marker publisher), Story 5.2 (department markers), Story 6.1 (high-level action)

**Technical Notes:**
- Reference architecture lines 652-686 for ManipulatorState message structure
- Custom message: `msg/ManipulatorState.msg` with comprehensive state fields
- Container gripped detection: Both jaw positions within 0.02m of expected symmetric positions
- Current action marker: TEXT_VIEW_FACING type, positioned at (rail_center_x, 0, selector_z + 0.5) for visibility
- Error marker: Marker.MESH_RESOURCE with red X or CUBE with ARROW pointing down
- Marker namespaces: Use consistent naming ("magnet_state", "picker_state", "container_state", "action_status", "error_indicator")

**Implementation Notes (Config Reuse):**
- **USE:** `manipulator_control/config/visualization.yaml` - add container_gripped, picker_active, current_action_text, error_state marker sections
- **USE:** All existing marker configurations from Stories 2.4, 3.5, 5.2
- **DO NOT CREATE:** Separate marker config - extend visualization.yaml

---

### Story 6.4: Create Comprehensive Integration Test Harness

As a developer,
I want automated integration tests covering all epics and end-to-end workflows,
So that I can validate system reliability and identify regressions quickly.

**Acceptance Criteria:**

**Given** all epics 1-5 are implemented
**When** I run `test/test_integration_full_system.py`
**Then** the test harness executes comprehensive test suite:

**Test Suite 1: Basic Operations (20 tests)**
- Joint control: MoveJoint for all 9 joints (9 tests)
- Joint groups: Navigation, picker, gripper, container (4 tests)
- Limit switches: All 18 switches trigger correctly (18 tests validation)
- Address resolution: 10 sample addresses (10 tests)
- Total: 20 operation tests

**Test Suite 2: Mid-Level Actions (25 tests)**
- NavigateToAddress: 8 cabinets × 2 addresses each = 16 tests
- ExtractBox: 3 different cabinet types (4-col, 5-col, 6-col) = 3 tests
- PickItem: 3 department positions (first, middle, last) = 3 tests
- Container operations: GetContainer + PlaceContainer = 2 tests
- ReturnBox: 1 test

**Test Suite 3: Advanced Box Operations (5 tests)**
- PutBox: Relocate box to different address = 1 test
- PutBox validation: Verify empty address check = 1 test
- PutBox validation: Verify width compatibility check = 1 test
- MoveBoxToLoad: Load + return = 1 test
- MoveBoxToLoad: Load + relocate = 1 test

**Test Suite 4: End-to-End Workflows (10 tests)**
- PickItemFromStorage: Complete workflow 5 different addresses = 5 tests
- PickItemFromStorage: With container retrieval = 2 tests
- PickItemFromStorage: Different department numbers (1, 5, 10) = 3 tests

**Test Suite 5: Error Conditions (10 tests)**
- Navigation to invalid address = 1 test (expect abort)
- ExtractBox with magnet failure = 1 test (expect retry + abort)
- PickItem with state timeout = 1 test (expect abort)
- PutBox to occupied address = 1 test (expect abort before motion)
- PickItemFromStorage with recovery = 3 tests (inject failures at different phases)
- Cancelled actions: Preemption during motion = 3 tests

**And** test results report:
- Total tests: 70
- Pass rate: ≥ 90% (63+ tests pass)
- Execution time: < 60 minutes
- Detailed log of failures with error messages and timestamps

**And** test harness includes:
- Automated Gazebo launch and teardown
- Result summary in JUnit XML format for CI integration
- Screenshots of RViz during failures (if display available)
- Bag file recording of failed test runs for debugging

**Prerequisites:** Story 6.1 (PickItemFromStorage), Story 6.2 (error handling), all Epic 1-5 stories

**Technical Notes:**
- Use pytest with launch_testing, parametrized tests for multiple addresses/departments
- Test execution: Run in headless Gazebo for CI environments (LIBGL_ALWAYS_SOFTWARE=1)
- Timeout per test: 120 seconds for complete workflow tests, 30 seconds for mid-level actions
- NFR-008: 95%+ success rate for navigation/box operations, 85%+ for complete pick operations
- Test addresses selection: Cover all 8 cabinet types, edge rows (top, bottom), edge columns (left, right)
- Error injection: Use service calls or topic publications to simulate failures (e.g., force magnet service to return failure)

**Implementation Notes (Config Reuse):**
- **USE:** `manipulator_control/config/testing.yaml` - NEW file for test suite configuration:
  - `integration_tests.success_rate_minimum`
- **USE:** All existing action server configs for timeouts
- **DO NOT CREATE:** Separate test config per test type - group in testing.yaml

---

### Story 6.5: Implement Reliability and Stress Testing

As a developer,
I want long-duration and stress tests validating system reliability,
So that I can ensure the system meets NFR success rate requirements in production-like scenarios.

**Acceptance Criteria:**

**Given** the integration test suite passes at 90%+
**When** I run `test/test_reliability_stress.py`
**Then** the stress test executes:

**Reliability Test (100 iterations, ~3 hours):**
- Execute PickItemFromStorage 100 times with randomized addresses and departments
- Addresses: Random selection from all 8 cabinets, varied rows/columns
- Departments: Random dept_num from 1 to max for cabinet
- No manual intervention between iterations
- Results:
  - Success rate: ≥ 85% (≥85 successful picks) - NFR-008
  - Mean execution time: < 120 seconds per pick - NFR-001
  - Max execution time: < 180 seconds (timeout threshold)
  - Zero unsafe states (extracted boxes always returned on failure)

**Stress Test (Rapid Operations, 30 minutes):**
- Execute rapid action sequences with minimal delays:
  - 50 NavigateToAddress operations (10 seconds between each)
  - 25 ExtractBox + ReturnBox cycles (30 seconds between each)
  - 20 PickItem operations from same box (15 seconds between each)
- Monitor system metrics:
  - Joint controller health (no crashes)
  - TF tree integrity (no frame drops)
  - Memory usage (< 1GB total)
  - CPU usage (< 80% sustained)
- Results:
  - No system crashes or node failures
  - Success rate: ≥ 90% for all operations
  - Performance stable (no degradation over time)

**Endurance Test (12 hour overnight, optional):**
- Continuous operation with 5-minute idle periods between operations
- Monitor for memory leaks, TF frame accumulation, marker bloat
- Results:
  - System operational after 12 hours
  - Memory usage increase < 20% over duration
  - Success rate maintained ≥ 85%

**And** all test results logged with:
- Per-iteration success/failure status
- Execution time statistics (mean, std dev, min, max)
- Error frequency by type (navigation, extraction, picking, etc.)
- System resource usage graphs (if plotted)

**Prerequisites:** Story 6.4 (integration test harness passes), Story 6.2 (error handling for recovery)

**Technical Notes:**
- Randomized address selection: Use random.choice() with seed for reproducibility
- Test execution: Run on dedicated machine with consistent Gazebo performance
- NFR validation: NFR-001 (120s pick cycle), NFR-008 (85% success rate), NFR-004/005 (box operation success rates)
- Failure analysis: Log all failures with full action goal parameters for reproducibility
- Resource monitoring: Use `psutil` Python library for memory/CPU tracking
- Overnight test: Optional for final validation before production deployment
- Success criteria: 85%+ success rate is minimum for Level 2 integration readiness

**Implementation Notes (Config Reuse):**
- **USE:** `manipulator_control/config/testing.yaml` - add reliability_test, stress_test, endurance_test sections:
  - `reliability_test.iterations`, `stress_test.duration_minutes`, `success_threshold`
- **USE:** All existing action server configs for timeouts
- **DO NOT CREATE:** Separate stress test config - extend testing.yaml

---

### Story 6.6: Create Test Harness and Performance Profiling Tools

As a developer,
I want tools to profile action execution times and identify performance bottlenecks,
So that I can optimize slow operations and meet timing requirements consistently.

**Acceptance Criteria:**

**Given** the system is operational
**When** I run `tools/profile_actions.py --action NavigateToAddress --iterations 20`
**Then** the profiling tool:
1. Executes action 20 times with varied parameters
2. Measures and logs for each iteration:
   - Total execution time
   - Time per phase (if action has phases)
   - Feedback update rate
   - Result return time
3. Outputs performance report:
   - Mean execution time: X.XX seconds
   - Standard deviation: X.XX seconds
   - Min/Max times: X.XX / X.XX seconds
   - 95th percentile: X.XX seconds (target for timeout tuning)
   - Histogram of execution times (ASCII art or PNG)

**When** I run `tools/profile_actions.py --action PickItemFromStorage --iterations 10`
**Then** the tool profiles complete workflow with phase breakdown:
- Phase 1 (Get Container): X.X seconds
- Phase 2 (Navigate): X.X seconds
- Phase 3 (Extract Box): X.X seconds
- Phase 4 (Position): X.X seconds
- Phase 5 (Pick Item): X.X seconds
- Phase 6 (Place in Container): X.X seconds
- Phase 7 (Return Box): X.X seconds
- Total: X.X seconds (compare to NFR-001 target of 120s)

**And** tool identifies slow phases:
- Phases exceeding 20% of total time flagged as "slow"
- Suggestions: "Phase 3 (Extract Box) takes 35% of total time - consider optimizing YZ trajectory speeds"

**And** tool supports profiling options:
- `--output-csv` exports results to CSV for analysis
- `--plot` generates execution time histogram PNG
- `--compare baseline.json current.json` shows performance changes

**Prerequisites:** Story 6.1 (PickItemFromStorage for complete workflow profiling)

**Technical Notes:**
- Profiling tool: Python script using rclpy action clients to call actions
- Timing measurement: Use time.perf_counter() for sub-millisecond accuracy
- Phase timing: Extract from action feedback timestamps
- Statistical analysis: Use numpy for mean, std dev, percentiles
- Histogram: Use matplotlib for PNG plots or ASCII art for terminal output
- Baseline comparison: Store JSON with execution time statistics, compare current run to detect regressions
- Use cases: Identify timeout values (95th percentile + margin), validate NFR-001 timing requirements, detect performance regressions after code changes

**Implementation Notes (Config Reuse):**
- **USE:** `manipulator_control/config/testing.yaml` - add profiling section (iterations_per_action)
- **USE:** All existing action server configs for timeout baselines
- **DO NOT CREATE:** Separate profiling config - extend testing.yaml

---

### Story 6.7: Performance Profiling and Optimization

As a developer,
I want to profile and optimize system performance to meet NFR timing requirements,
So that operations complete within specified time limits consistently.

**Acceptance Criteria:**

**Given** the profiling tool from Story 6.6 is available
**When** I profile all mid-level and high-level actions
**Then** performance meets NFR requirements:

**NFR-001: PickItemFromStorage Complete Cycle**
- Target: < 120 seconds mean execution time
- Measured: [Actual value from profiling]
- Status: PASS if < 120s, FAIL if ≥ 120s

**NFR-005: ExtractBox Cycle Time**
- Target: < 60 seconds per extraction
- Measured: [Actual value]
- Status: PASS/FAIL

**NFR-006: Box Relocation (PutBox)**
- Target: < 90 seconds
- Measured: [Actual value]
- Status: PASS/FAIL

**When** any action exceeds target by > 10%
**Then** optimization applied:
- YZ trajectory speeds increased (if collision-safe)
- Joint group default velocities tuned in config
- Unnecessary wait delays removed (e.g., magnet engagement wait reduced from 0.5s to 0.3s if reliable)
- Action feedback publish rates reduced if causing overhead (e.g., 10 Hz → 5 Hz)

**And** after optimization:
- Re-run profiling to validate improvements
- Update `config/action_servers.yaml` with optimized parameters
- Document performance tuning in `docs/PERFORMANCE_TUNING.md`

**And** all optimizations validated for safety:
- No increase in collision risk
- No reduction in success rates
- Joint velocities within limits (0.5 m/s max for navigation, 0.1 m/s for precision operations)

**Prerequisites:** Story 6.6 (profiling tools)

**Technical Notes:**
- Reference NFRs: NFR-001 (120s pick cycle), NFR-004 (90% extraction success), NFR-005 (60s extraction), NFR-006 (90s relocation)
- Optimization parameters in action_servers.yaml: default_velocity, approach_speed, retract_speed, wait_delays
- Typical slow phases: ExtractBox YZ trajectory (slow approach_speed 0.05 m/s), PickItem state machine (multiple state transitions)
- Safety validation: Test optimized speeds 10 times, verify no collisions or errors
- Performance documentation: List all tuned parameters with before/after execution times
- Trade-offs: Speed vs. accuracy (faster motion may reduce positioning accuracy), speed vs. reliability (faster may increase failure rates)

**Implementation Notes (Config Reuse):**
- **USE:** `manipulator_control/config/testing.yaml` - add nfr_targets section (pick_cycle_sec, extract_cycle_sec, relocation_sec)
- **USE:** All existing action server configs - optimization updates go to existing files
- **DO NOT CREATE:** Separate NFR config - add targets to testing.yaml

---

### Story 6.8: Complete System Documentation and RQt Usage Guide

As a developer or operator,
I want comprehensive documentation covering all system functionality, testing, and operation,
So that I can use, maintain, and extend the system effectively.

**Acceptance Criteria:**

**Given** all Epic 1-6 stories are implemented
**When** I access project documentation
**Then** the following documentation exists and is complete:

**1. System Architecture Document (Already exists)**
- Path: `docs/architecture-ros2-control.md`
- Content: Complete system design, action hierarchy, package structure, development phases
- Status: Complete from planning phase

**2. API Reference Documentation**
- Path: `docs/API_REFERENCE.md`
- Content:
  - All action interfaces (goal/result/feedback fields) with descriptions
  - All service interfaces (request/response fields)
  - All custom messages
  - Example usage code snippets for each action
  - Return value descriptions and error codes

**3. Configuration Guide**
- Path: `docs/CONFIGURATION_GUIDE.md`
- Content:
  - All YAML configuration files explained (storage_params, action_servers, limit_switches, etc.)
  - Parameter descriptions and valid ranges
  - How to tune performance parameters (velocities, timeouts)
  - How to add new addresses, cabinets, load positions

**4. Testing Guide**
- Path: `docs/TESTING_GUIDE.md`
- Content:
  - How to run unit tests, integration tests, reliability tests
  - Test suite descriptions and expected pass rates
  - How to interpret test results and debug failures
  - How to use profiling tools

**5. RQt Usage Guide (Epic 2 deliverable, enhanced)**
- Path: `docs/TESTING_WITH_RQT.md`
- Content:
  - How to launch RQt with all relevant plugins
  - How to send action goals using rqt_action
  - How to monitor topics (joint_states, limit_switches, markers) using rqt_topic
  - How to view logs using rqt_console
  - How to visualize TF tree using rqt_tf_tree
  - Saved RQt perspectives for common tasks (development, testing, debugging)
  - Screenshots of typical RQt layouts

**6. Troubleshooting Guide**
- Path: `docs/TROUBLESHOOTING.md`
- Content:
  - Common failure modes and solutions (magnet won't engage, picker state timeout, navigation positioning error)
  - How to use markers and state topics for debugging
  - How to manually reset system state
  - Emergency stop procedures
  - Gazebo simulation quirks and workarounds

**7. Performance Tuning Guide**
- Path: `docs/PERFORMANCE_TUNING.md`
- Content: (Created in Story 6.7)
  - Profiling tool usage
  - Optimization strategies
  - Tuned parameter values with rationale
  - Trade-off analysis (speed vs. safety vs. reliability)

**8. README.md (Project root)**
- Path: `README.md`
- Content:
  - System overview and capabilities
  - Quick start guide (how to build and launch simulation)
  - Link to all documentation
  - System requirements (ROS2 Humble, Gazebo, dependencies)
  - Known limitations and future work

**And** all code has inline documentation:
- Action server classes: Docstrings for __init__, execute_callback, and key methods
- Utility classes: Docstrings for all public methods
- Configuration YAML files: Comments explaining parameters

**And** documentation includes:
- Table of contents for each document
- Code examples formatted correctly (syntax highlighting)
- Clear section headings and consistent formatting
- Version information (match to architecture document v2.0)

**Prerequisites:** All Epic 1-6 stories (documentation describes implemented system)

**Technical Notes:**
- Use Markdown format for all documentation (consistent with existing docs)
- Code examples: Use Python syntax highlighting (```python)
- Screenshots: Use RViz and RQt with clear labels/annotations
- RQt perspective files: Store in `config/` directory (.perspective extension)
- API documentation: Can be auto-generated from action/service definitions using rosdoc2
- Update existing README.md to link to new documentation
- Documentation review: Have another developer validate clarity and completeness

**Implementation Notes (Config Reuse):**
- **DOCUMENT:** All 11 new config files from this plan + 4 existing config files
- **DOCUMENT:** Configuration Reuse Policy and single source of truth principles
- **DO NOT CREATE:** New config - this story produces documentation only

---


---

## FR Coverage Matrix

This matrix shows complete traceability from Functional Requirements to Epic Stories:

| FR | Description | Epic | Stories |
|----|-------------|------|---------|
| FR-001 | Warehouse Address Navigation | Epic 3 | 3.1, 3.2, 3.4 |
| FR-002 | Address Coordinate Resolution | Epic 3 | 3.1, 3.2 |
| FR-003 | Box Extraction from Storage | Epic 4A | 4A.1, 4A.2, 4A.3, 4A.4 |
| FR-004 | Box Return to Original Address | Epic 4A | 4A.1, 4A.5 |
| FR-005 | Item Picking with State Machine | Epic 5 | 5.5 |
| FR-006 | Department Frame Generation | Epic 5 | 5.1, 5.2 |
| FR-007 | Limit Switch Simulation | Epic 2 | 2.1 |
| FR-008 | Electromagnet Simulation | Epic 4A | 4A.2 |
| FR-009 | Dynamic Box Spawning | Epic 4A | 4A.3 |
| FR-010 | YZ Trajectory Generation | Epic 4A | 4A.1 |
| FR-011 | Low-Level Joint Control | Epic 2, 3 | 2.2, 2.3, 3.3 |
| FR-012 | Visual State Markers | Epic 2, 3, 5, 6 | 2.4, 3.5, 5.2, 6.3 |
| FR-013 | Configuration from YAML | All Epics | All configuration-loading stories |
| FR-014 | Box Relocation (PutBox) | Epic 4B | 4B.1, 4B.2, 4B.3 |
| FR-015 | Box Loading Station Operations | Epic 4B | 4B.4 |
| FR-016 | Container Jaw Manipulation | Epic 5 | 5.3 |
| FR-017 | Container Retrieval and Placement | Epic 5 | 5.4 |
| FR-018 | Address Validation Utilities | Epic 4B | 4B.1 |
| FR-019 | Complete Pick-from-Storage Workflow | Epic 6 | 6.1 |
| FR-020 | RQt Tool Integration | Epic 2, 6 | 2.6, 6.8 |

**Coverage Validation:** ✅ All 20 functional requirements mapped to implementation stories

---

## Summary

### Epic Breakdown Summary

**Total Implementation Scope:**
- **7 Epics** organized by functional capability and natural implementation sequence
- **40 Stories** sized for single developer session completion
- **Complete FR Coverage** - All 20 functional requirements traced to stories
- **NFR Integration** - All 13 non-functional requirements addressed

### Epic Sequencing Rationale

1. **Epic 1 (Foundation):** Establishes package structure and interfaces - enables all subsequent work
2. **Epic 2 (Basic Control):** Implements low-level joint control and simulation infrastructure
3. **Epic 3 (Navigation):** Builds address-based navigation on top of joint control
4. **Epic 4A (Box Core):** Implements fundamental box extraction and return operations
5. **Epic 4B (Box Advanced):** Adds box relocation and loading station capabilities
6. **Epic 5 (Item Picking):** Implements department-level picking within extracted boxes
7. **Epic 6 (Integration):** Delivers complete autonomous workflows with validation

### Success Criteria Mapping

**Technical Success Metrics (from PRD):**
- ✅ **Reliability:** Stories 6.5, 6.4 validate ≥95% simulation success rate
- ✅ **Performance:** Stories 6.6, 6.7 validate ≤120s pick operations (NFR-001)
- ✅ **Accuracy:** Stories 3.6, 5.6 validate ±5mm navigation, ±2mm picking (NFR-002)
- ✅ **Safety:** Story 6.2 implements comprehensive error handling and recovery

**Operational Success Metrics:**
- ✅ **System Availability:** Story 6.8 documents startup procedures (≤10s to ready)
- ✅ **Integration Readiness:** Story 6.1 provides Level 2 action interface
- ✅ **Configuration Flexibility:** All epics use YAML configuration (FR-013)

**Proof-of-Concept Goals:**
- ✅ **Algorithm Validation:** Epic 6 includes 100-iteration reliability testing
- ✅ **Address Space Coverage:** Story 3.6 validates all 8 cabinets accessible
- ✅ **Hardware Readiness:** Clear Gazebo vs hardware separation throughout

### Development Workflow

**For Implementation (Phase 4):**
1. Use `/bmad:bmm:workflows:dev-story` to implement stories sequentially
2. Each story includes detailed acceptance criteria for validation
3. Test stories (2.5, 3.6, 4A.X, 5.6, 6.4, 6.5) provide continuous validation
4. Epic 6 concludes with complete system validation before hardware deployment

**Context Available:**
- ✅ PRD with all functional and non-functional requirements
- ✅ Architecture with implementation details and technical decisions  
- ✅ This Epic Breakdown with 40 actionable stories

### Next Steps

**Immediate:** Begin Phase 4 Implementation with Epic 1, Story 1.1
**After MVP:** Proceed to Growth Phase - Hardware Deployment (Epics 7-9 from PRD)
**Vision:** Level 2 integration, multi-manipulator coordination, medical domain enhancements

---

_This epic breakdown created through Product Manager workflow with Advanced Elicitation (Journey Mapping) applied._
_All stories sized for autonomous dev agent implementation with detailed acceptance criteria._
_Ready for Phase 4: Sprint Planning and Implementation._

