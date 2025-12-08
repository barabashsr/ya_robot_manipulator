# Epic Technical Specification: Box Extraction Core

Date: 2025-11-27
Author: BMad
Epic ID: 4A
Status: In Progress

---

## Implementation Status

**Last Updated:** 2025-12-05

| Story | Status | Implementation |
|-------|--------|----------------|
| 4a-1 | DONE | `src/utils/yz_trajectory_generator.py` - Trajectory generation utility |
| 4a-1a | DONE | `scripts/test_trajectory_with_markers.py` - Visualization test script |
| 4a-2 | Backlog | Electromagnet simulator (planned) |
| 4a-3 | Backlog | Box spawner with department frames (planned) |
| 4a-4 | Backlog | ExtractBox action server (planned) |
| 4a-5 | Backlog | ReturnBox action server (planned) |

### Implemented Files (Stories 4a-1, 4a-1a)

**Utilities:**
- `ros2_ws/src/manipulator_control/src/utils/yz_trajectory_generator.py`
  - Loads waypoints from YAML, transforms for side/position
  - Builds JointTrajectory messages for YZ coordinated motion
  - Methods: `load_trajectory()`, `transform_waypoints()`, `build_joint_trajectory()`

**Scripts:**
- `ros2_ws/src/manipulator_control/scripts/test_trajectory_with_markers.py`
  - Publishes `/trajectory_markers` (MarkerArray) for RViz visualization
  - Supports `--side`, `--trajectory`, `--address`, `--no-execute` args

**Config:**
- `ros2_ws/src/manipulator_control/config/extraction_trajectories.yaml` - Waypoint data
- `ros2_ws/src/manipulator_control/config/trajectory_config.yaml` - Timing parameters

---

## Overview

Epic 4A implements the core box handling capabilities for the ya_robot_manipulator warehouse automation system. This epic delivers the foundational infrastructure for extracting boxes from cabinet storage addresses and returning them after item picking operations. The implementation covers **parametric curve-based YZ trajectory generation** (SVG source → YAML waypoints) for smooth cabinet insertion/extraction, electromagnet simulation for magnetic box attachment in Gazebo, dynamic box spawning with department TF frame generation, and the ExtractBox/ReturnBox action servers.

This epic bridges the navigation system (Epic 3) with item picking capabilities (Epic 5) by enabling physical box manipulation. Without Epic 4A, boxes remain inaccessible in cabinets - with it, the system can autonomously retrieve any box from the 8-cabinet warehouse array, providing access to 100+ storage addresses.

## Objectives and Scope

**In Scope:**
- YZ trajectory generator utility using parametric curves (SVG source files converted to YAML waypoints) for smooth cabinet insertion/extraction
- Electromagnet simulator node using Gazebo DetachableJoint plugin for magnetic attachment/detachment
- Dynamic box spawner with URDF generation, robot_state_publisher integration, and department TF frame broadcasting
- ExtractBox action server implementing full extraction workflow (navigate → approach → engage magnet → spawn box → extract)
- ReturnBox action server implementing reverse extraction workflow (navigate → insert → release magnet → despawn → retract)
- Configuration files for trajectory parameters, electromagnet settings, and box spawner options
- Integration with Epic 3 NavigateToAddress and MoveJointGroup actions

**Out of Scope:**
- Box relocation to different addresses (PutBox - Epic 4B)
- Loading station operations (MoveBoxToLoad - Epic 4B)
- Address validation for box placement (Epic 4B)
- Item picking from departments within boxes (Epic 5)
- Department visual markers with labels (partial in 4A.3, completed in Epic 5)
- Container jaw manipulation for item containers (Epic 5)

## System Architecture Alignment

Epic 4A aligns with the **Phase 3: Box Handling** architecture layer, building on:

**Dependencies from Earlier Phases:**
- **Phase 1 (Epic 2):** ControllerInterface utility, limit switch simulation, MoveJoint action server
- **Phase 2 (Epic 3):** NavigateToAddress action, MoveJointGroup with joint groups, GetAddressCoordinates service, AddressResolver utility

**Architecture Components:**
- Uses `selector_frame_gripper_joint` (Y-axis) for cabinet insertion/extraction
- Uses `main_frame_selector_frame_joint` (Z-axis) for clearance trajectory coordination
- Electromagnet links: `left_gripper_magnet` and `right_gripper_magnet` on gripper frame
- Storage parameters from `manipulator_description/config/storage_params.yaml` (cabinet dimensions, wall thickness, box configurations)

**Coordinate System (from Architecture):**
- **Y-axis:** Into/out of cabinets (gripper motion) - positive Y = left cabinets, negative Y = right cabinets
- **Z-axis:** Vertical selector motion for row positioning and frame clearance
- Box extraction uses predefined parametric curves (designed in SVG, converted to YAML waypoints) for smooth coordinated YZ motion

## Detailed Design

### Services and Modules

| Module | Responsibility | Inputs | Outputs |
|--------|---------------|--------|---------|
| `yz_trajectory_generator.py` | Load and transform parametric curve waypoints | Trajectory name, side, base position | List of waypoints with timing |
| `scripts/svg_to_trajectory.py` | Convert SVG paths to YAML waypoints (dev tool) | SVG file path | YAML waypoint file |
| `electromagnet_simulator_node.py` | Control Gazebo magnetic attachment | Service request (activate bool) | Engagement state, attachment result |
| `box_spawn_manager_node.py` | Spawn/despawn boxes with department TF | SpawnBox/DespawnBox service requests | Box URDF, TF frames, Gazebo model |
| `extract_box_server.py` | Execute box extraction workflow | ExtractBox action goal | Extracted box with department frames |
| `return_box_server.py` | Execute box return workflow | ReturnBox action goal | Box returned, address restored |

### Data Models and Contracts

**YZ Waypoint Structure:**
```python
@dataclass
class YZWaypoint:
    y: float              # Target Y position (meters)
    z: float              # Target Z offset (meters)
    time_from_start: float  # Seconds from trajectory start
```

**Trajectory Config (external scaling):**
```yaml
# config/trajectory_config.yaml
trajectories:
  extract_left:
    svg_file: trajectories/extract_left.svg
    mapping:
      x_range: [0, 100]        # SVG X coordinate range
      y_output: [0.0, 0.4]     # Joint Y output range (meters)
      y_center: 50             # SVG Y value that maps to Z=0
      z_scale: 0.001           # Meters per SVG unit (1mm)
    sampling:
      num_points: 20
      waypoint_duration: 0.5
```

**SVG Source Format (unitless):**
```svg
<!-- config/trajectories/extract_left.svg -->
<svg viewBox="0 0 100 100">
  <!-- Unitless curves - see trajectory_config.yaml for scaling -->
  <path id="insertion" d="M 0,50 C 30,50 70,48 100,50"/>
  <path id="extraction" d="M 100,50 C 70,52 30,52 0,50"/>
</svg>
```

**Generated Waypoints YAML:**
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

**Box State Tracking:**
```python
@dataclass
class ActiveBox:
    box_id: str                    # e.g., "box_l_1_2_3"
    rsp_process: subprocess.Popen  # robot_state_publisher process
    urdf: str                      # Generated URDF string
    num_departments: int           # Department count
    source_address: dict           # {side, cabinet, row, column}
```

**Cabinet Configuration (from storage_params.yaml):**
```yaml
cabinet_dimensions:
  exterior:
    x: 0.7   # Width
    y: 0.66  # Depth
    z: 1.4   # Height
  wall_thickness: 0.02
```

### APIs and Interfaces

**ToggleElectromagnet Service:**
```
srv/ToggleElectromagnet.srv
---
bool activate        # True to engage, False to release
---
bool success
string message
```

**SpawnBox Service:**
```
srv/SpawnBox.srv
---
string box_id
string side               # "left" or "right"
int32 cabinet_num
int32 row
int32 column
int32 num_departments
---
bool success
string message
string spawned_box_id
```

**DespawnBox Service:**
```
srv/DespawnBox.srv
---
string box_id
---
bool success
string message
```

**ExtractBox Action:**
```
action/ExtractBox.action
---
# Goal
string side
int32 cabinet_num
int32 row
int32 column
float32 box_depth          # Default 0.3m
float32 extraction_speed   # Default 0.08 m/s
---
# Result
bool success
bool box_extracted
bool magnet_engaged
string box_id
string message
---
# Feedback
string current_phase       # "navigating", "approaching", "engaging_magnet", "spawning_box", "extracting", "complete"
float32 progress_percent
float32 gripper_depth
bool magnet_contact_detected
```

**ReturnBox Action:**
```
action/ReturnBox.action
---
# Goal
string side
int32 cabinet_num
int32 row
int32 column
string box_id
---
# Result
bool success
bool box_returned_successfully
string message
---
# Feedback
string current_operation   # "navigating", "inserting_box", "releasing_magnet", "retracting", "completing"
float32 progress_percent
```

### Workflows and Sequencing

**ExtractBox Workflow:**
```
Phase 1: Navigate (0-20%)
├── Call NavigateToAddress(side, cabinet, row, column)
└── Wait for completion

Phase 2: Approach (20-40%)
├── Get current gripper Y/Z from /joint_states
├── Load "insertion" trajectory from extraction_trajectories.yaml
├── Transform waypoints: apply side sign (+/-), add base Y/Z positions
├── Execute via JointTrajectoryController
└── Monitor gripper_depth in feedback

Phase 3: Engage Magnet (40-50%)
├── Call ToggleElectromagnet(activate=true)
├── Verify proximity check passed
└── Wait 0.5s for magnetic attachment

Phase 4: Spawn Box (50-60%)
├── Generate box_id: "box_{side}_{cabinet}_{row}_{col}"
├── Call SpawnBox service
│   ├── Generate URDF with department child links
│   ├── Launch robot_state_publisher
│   ├── Broadcast static transform gripper_magnet → box_base_link
│   └── (Simulation) Spawn in Gazebo with DetachableJoint
├── Update state markers (red extracted marker)
└── Department TF frames now available

Phase 5: Extract (60-90%)
├── Load "extraction" trajectory from extraction_trajectories.yaml
├── Transform waypoints: apply side sign, add base positions
├── Execute via JointTrajectoryController (spline interpolation)
└── Box TF frames follow gripper throughout motion

Phase 6: Complete (90-100%)
├── Verify final position
└── Return success with box_id
```

**ReturnBox Workflow:**
```
Phase 1: Navigate (0-25%)
└── NavigateToAddress to original address

Phase 2: Insert (25-60%)
├── Load "insertion" trajectory, transform for target address
└── Execute via JointTrajectoryController

Phase 3: Release (60-70%)
├── Call ToggleElectromagnet(activate=false)
└── Wait 0.3s for detachment

Phase 4: Retract (70-95%)
├── Load "extraction" trajectory, transform for current position
└── Execute via JointTrajectoryController

Phase 5: Cleanup (95-100%)
├── Call DespawnBox(box_id)
├── Remove extracted marker for address
└── Return success
```

## Non-Functional Requirements

### Performance

| Metric | Target | Source |
|--------|--------|--------|
| ExtractBox operation time | ≤60 seconds | NFR-005 |
| ReturnBox operation time | ≤60 seconds | NFR-005 |
| YZ trajectory execution accuracy | ±2cm | NFR-002 |
| Magnet engagement response | <0.5 seconds | Story 4A.2 |
| TF frame broadcast rate | 10 Hz | FR-006 |
| Department frame availability | <2 seconds after spawn | Story 4A.3 |

### Security

- No network interfaces exposed (internal ROS2 only)
- Service access through standard ROS2 middleware
- No credential handling in this epic

### Reliability/Availability

| Requirement | Target | Source |
|-------------|--------|--------|
| ExtractBox success rate | ≥90% | NFR-004 |
| ReturnBox success rate | ≥90% | NFR-004 |
| Zero unrecoverable states | 100% | NFR-005 |
| Timeout protection | All actions | NFR-005 |
| Safe abort capability | All phases | NFR-005 |

**Failure Recovery:**
- Magnet engagement failure → Abort cleanly, retract gripper
- Box spawn failure → Disengage magnet, abort extraction
- Trajectory failure → Stop motion, report error with current position

### Observability

- Action feedback published at 5 Hz during operations
- Magnet state published to `/manipulator/electromagnet/engaged` at 10 Hz
- State markers updated via `/visualization_marker_array` for:
  - Red extracted address markers
  - Magnet engagement indicator on gripper
- ROS2 logging at DEBUG level for trajectory waypoints, INFO for phase transitions

## Dependencies and Integrations

**ROS2 Package Dependencies:**
```xml
<depend>rclpy</depend>
<depend>geometry_msgs</depend>
<depend>std_msgs</depend>
<depend>std_srvs</depend>
<depend>tf2_ros</depend>
<depend>visualization_msgs</depend>
<depend>control_msgs</depend>
<depend>ros_gz_interfaces</depend>       <!-- Gazebo Harmonic spawn/delete -->
<depend>robot_state_publisher</depend>   <!-- TF broadcasting for boxes -->
```

**Internal Dependencies:**
- `manipulator_interfaces` - Action/service definitions (Epic 1)
- `manipulator_description` - URDF, storage_params.yaml
- `manipulator_control` (Epic 2-3) - ControllerInterface, MoveJointGroup, NavigateToAddress

**External Integrations:**
- **Gazebo Harmonic:** `/world/default/create` and `/world/default/remove` services via `ros_gz_interfaces`
- **DetachableJoint Plugin:** For magnetic attachment simulation in Gazebo

**Configuration Files Created:**
| File | Purpose |
|------|---------|
| `config/trajectory_config.yaml` | Scaling config: maps SVG units to real-world meters |
| `config/trajectories/extract_left.svg` | SVG source for extraction curves (unitless, editable in Inkscape) |
| `config/extraction_trajectories.yaml` | Generated YAML waypoints (from SVG + config) |
| `config/electromagnet.yaml` | Magnet engagement parameters (proximity_distance, wait times) |
| `config/box_spawner.yaml` | Box spawn parameters (TF rate, Gazebo world name) |
| `config/action_servers.yaml` | (Extended) Extract/return box timeouts |

**Development Tools:**
| Tool | Purpose |
|------|---------|
| `scripts/svg_to_trajectory.py` | Convert SVG paths to YAML waypoints using config scaling (requires `svgpathtools`) |

## Acceptance Criteria (Authoritative)

**AC-4A.1: YZ Trajectory Generation (Parametric Curves)**
1. `trajectory_config.yaml` exists with mapping parameters (x_range, y_output, y_center, z_scale) and sampling parameters for each trajectory
2. Given an SVG file and config entry, when svg_to_trajectory.py is run, then YAML waypoints file is generated using scaling from config (not hardcoded)
3. Given trajectory name and side, when load_trajectory() is called, then waypoints are loaded from YAML and transformed (sign flip for right side, base position offset)
4. Given transformed waypoints, when execute_trajectory() is called, then JointTrajectoryController executes smooth motion and method returns success/failure
5. SVG source files exist for extraction with insertion and extraction paths (unitless curves, scaling in config)

**AC-4A.1a: End Effector Trajectory Visualization (Hotfix)**
1. Given side='left', when publish_trajectory_markers() is called, then markers are published in `left_gripper_magnet` frame
2. Given side='right', when publish_trajectory_markers() is called, then markers are published in `right_gripper_magnet` frame
3. Waypoint positions are relative to end effector frame origin (Y = forward into cabinet, Z = vertical clearance)
4. Test script works without requiring `--address` parameter (uses end effector frame by default)

**AC-4A.2: Electromagnet Simulation**
1. When ToggleElectromagnet(activate=true) is called with box within 5cm, then DetachableJoint attach message is published and engaged state is true
2. When ToggleElectromagnet(activate=false) is called with box attached, then DetachableJoint detach message is published and engaged state is false
3. Magnet state publishes to `/manipulator/electromagnet/engaged` at 10 Hz

**AC-4A.3: Box Spawner**
1. When SpawnBox is called, then URDF is generated with base_link and N department child links
2. When SpawnBox completes, then robot_state_publisher is running and department TF frames are available
3. When SpawnBox is called in simulation mode, then box model is spawned in Gazebo attached to gripper
4. When DespawnBox is called, then robot_state_publisher is terminated and TF frames are removed
5. Department frame positions follow formula: y = offset_y + (dept_num - 1) × dept_depth

**AC-4A.4: ExtractBox Action**
1. When ExtractBox goal is received, then action navigates to address, approaches, engages magnet, spawns box, and extracts
2. When extraction completes successfully, then box_id is returned and department TF frames are available
3. When magnet engagement fails, then action aborts cleanly and gripper retracts
4. Action provides feedback with current_phase and progress_percent at 5 Hz

**AC-4A.5: ReturnBox Action**
1. When ReturnBox goal is received, then action navigates to address, inserts box, releases magnet, and retracts
2. When return completes successfully, then box is despawned and extracted marker is removed
3. Action validates box_id exists before proceeding

## Traceability Mapping

| AC | Spec Section | Component | Test Approach |
|----|--------------|-----------|---------------|
| AC-4A.1.1 | YZ Trajectory | trajectory_config.yaml | Config file existence and schema validation |
| AC-4A.1.2 | YZ Trajectory | svg_to_trajectory.py | Unit test config-based scaling (not hardcoded) |
| AC-4A.1.3 | YZ Trajectory | yz_trajectory_generator.py | Unit test waypoint loading and transformation |
| AC-4A.1.4 | YZ Trajectory | yz_trajectory_generator.py | Integration test with JointTrajectoryController |
| AC-4A.1.5 | YZ Trajectory | config/trajectories/*.svg | File existence and path validity check |
| AC-4A.1a.1 | YZ Trajectory | yz_trajectory_generator.py | Unit test marker frame selection (left side) |
| AC-4A.1a.2 | YZ Trajectory | yz_trajectory_generator.py | Unit test marker frame selection (right side) |
| AC-4A.1a.3 | YZ Trajectory | yz_trajectory_generator.py | Unit test relative coordinate calculation |
| AC-4A.1a.4 | YZ Trajectory | test_trajectory_with_markers.py | Integration test without --address |
| AC-4A.2.1 | Electromagnet | electromagnet_simulator_node.py | Service call test with proximity |
| AC-4A.2.2 | Electromagnet | electromagnet_simulator_node.py | Service call test for detachment |
| AC-4A.2.3 | Electromagnet | electromagnet_simulator_node.py | Topic monitoring test |
| AC-4A.3.1 | Box Spawner | box_spawn_manager_node.py | URDF generation unit test |
| AC-4A.3.2 | Box Spawner | box_spawn_manager_node.py | TF frame availability test |
| AC-4A.3.3 | Box Spawner | box_spawn_manager_node.py | Gazebo integration test |
| AC-4A.3.4 | Box Spawner | box_spawn_manager_node.py | Despawn cleanup test |
| AC-4A.3.5 | Box Spawner | box_spawn_manager_node.py | Department position calculation test |
| AC-4A.4.1 | ExtractBox | extract_box_server.py | Full workflow integration test |
| AC-4A.4.2 | ExtractBox | extract_box_server.py | TF frame verification post-extract |
| AC-4A.4.3 | ExtractBox | extract_box_server.py | Magnet failure abort test |
| AC-4A.4.4 | ExtractBox | extract_box_server.py | Feedback monitoring test |
| AC-4A.5.1 | ReturnBox | return_box_server.py | Full workflow integration test |
| AC-4A.5.2 | ReturnBox | return_box_server.py | Despawn verification test |
| AC-4A.5.3 | ReturnBox | return_box_server.py | Invalid box_id rejection test |

## Risks, Assumptions, Open Questions

**Risks:**
- **R1:** Gazebo DetachableJoint plugin may have timing issues with rapid attach/detach cycles. *Mitigation:* Add configurable wait times after attach/detach operations.
- **R2:** robot_state_publisher subprocess management may have cleanup issues on crashes. *Mitigation:* Implement process tracking with cleanup on node shutdown.
- **R3:** SVG curve design may produce suboptimal motion profiles. *Mitigation:* Test curves visually in simulation, iterate on SVG design, regenerate YAML.

**Assumptions:**
- **A1:** Gazebo Harmonic ros_gz_interfaces services are available at `/world/default/create` and `/world/default/remove`
- **A2:** DetachableJoint plugin is configured in manipulator URDF for gripper magnet links
- **A3:** Epic 3 NavigateToAddress and MoveJointGroup actions are fully operational
- **A4:** storage_params.yaml contains complete cabinet and department configurations

**Open Questions:**
- **Q1:** Should box mass (0.5kg) be configurable per cabinet type or fixed? *Current assumption:* Fixed at 0.5kg for all boxes.
- **Q2:** What happens if extraction is cancelled mid-trajectory? *Current plan:* Stop motion, keep box attached if magnet engaged, return failure with partial state.
- **Q3 (RESOLVED):** How should trajectories be defined? *Resolution:* Use SVG Bezier curves as source (editable in Inkscape), convert to YAML waypoints with `svg_to_trajectory.py`, execute via JointTrajectoryController with spline interpolation.

## Test Strategy Summary

**Test Levels:**

1. **Unit Tests:**
   - SVG to YAML conversion (svg_to_trajectory.py)
   - Waypoint loading and transformation (yz_trajectory_generator.py)
   - URDF generation with department child links
   - Department position calculations

2. **Integration Tests:**
   - Electromagnet service with Gazebo DetachableJoint
   - Box spawner with robot_state_publisher and TF verification
   - ExtractBox full workflow in Gazebo
   - ReturnBox full workflow in Gazebo

3. **System Tests:**
   - Extract and return boxes from all 8 cabinets
   - Verify 90%+ success rate over 20+ operations
   - Stress test with rapid extract/return cycles

**Test Frameworks:**
- pytest for unit tests
- launch_testing for ROS2 integration tests
- Gazebo simulation environment for system tests

**Coverage of ACs:**
- All 18 acceptance criteria have corresponding test cases
- Critical paths (AC-4A.4.1, AC-4A.5.1) have multiple test scenarios

**Edge Cases:**
- Box extraction at top row (Z clearance critical)
- Box extraction at boundary columns
- Electromagnet timeout during engagement
- Gazebo spawn service unavailable (hardware mode fallback)
