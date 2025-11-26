# Epic Technical Specification: Address Navigation System

Date: 2025-11-27
Author: BMad
Epic ID: 3
Status: Draft

---

## Overview

Epic 3 implements the Address Navigation System for the ya_robot_manipulator warehouse automation system. This epic enables the manipulator to resolve warehouse addresses (side, cabinet, row, column) to world-frame coordinates using TF frames and navigate accurately to any of the 100+ storage locations across 8 cabinets.

The navigation system forms the foundation for all subsequent box handling operations (Epic 4) by providing precise positioning capability within ±2cm accuracy requirement (NFR-002). It leverages the existing TF frame tree from the URDF rather than maintaining hardcoded coordinate tables, ensuring configuration changes require only YAML updates.

## Objectives and Scope

**In Scope:**
- TF-based address resolution utility (no hardcoded coordinates)
- GetAddressCoordinates ROS2 service for address lookups
- Extension of MoveJointGroup to support predefined joint groups
- NavigateToAddress action server for cabinet positioning
- Visual markers for target and extracted addresses in RViz
- Comprehensive navigation test suite validating accuracy

**Out of Scope:**
- Box extraction/return operations (Epic 4A)
- YZ trajectory generation for cabinet insertion (Epic 4A)
- Electromagnet control (Epic 4A)
- Item picking operations (Epic 5)
- High-level workflow composition (Epic 6)

## System Architecture Alignment

**Architecture Reference:** `docs/architecture-ros2-control-v2-CORRECTIONS.md`

**Key Architectural Decisions:**
1. **TF Frame Resolution (Section 2):** All addresses exist as TF frames in URDF with format `addr_{side}_{cabinet}_{row}_{column}`. Resolution via TF2 lookup eliminates coordinate duplication.
2. **Hybrid Controller Architecture (Section 1):** Uses JointTrajectoryController for 7 motion joints (smooth interpolated motion) and ForwardCommandController for 2 container jaws.
3. **Joint Groups (Section 9):** Navigation uses 2 DOF group [base_main_frame_joint, main_frame_selector_frame_joint] for XZ plane positioning.

**Components Referenced:**
- ControllerInterface utility (Epic 2, Story 2.2)
- MoveJointGroup action server (Epic 2, Story 2.5)
- State marker publisher (Epic 2, Story 2.4)
- storage_params.yaml (cabinet configurations)

**Constraints:**
- Position accuracy: ±2cm (NFR-002)
- Navigation timeout: 30 seconds max
- TF lookup timeout: 1.0 second
- Service response time: <100ms (NFR-003)

## Detailed Design

### Services and Modules

| Module | Responsibility | Inputs | Outputs | Owner |
|--------|---------------|--------|---------|-------|
| AddressResolver | TF-based coordinate resolution | (side, cabinet, row, col) | (x, y, z, success, error) | Story 3.1 |
| GetAddressCoordinates Service | ROS2 service wrapper | GetAddressCoordinates.srv | geometry_msgs/Pose | Story 3.2 |
| MoveJointGroup (extended) | Joint group coordination | joint_group name, targets | motion execution | Story 3.3 |
| NavigateToAddress Action | High-level navigation | address + approach_distance | positioning result | Story 3.4 |
| State Marker Publisher (extended) | Address visualization | system state | MarkerArray | Story 3.5 |

### Data Models and Contracts

**Address Frame Naming Convention:**
```
Pattern: addr_{side_abbrev}_{cabinet}_{row}_{col}
- side_abbrev: 'l' for left, 'r' for right
- cabinet: 1-4
- row: 1 to max_rows (cabinet-dependent)
- col: 1 to max_cols (cabinet-dependent)

Examples:
- addr_l_1_2_3 → Left side, Cabinet 1, Row 2, Column 3
- addr_r_4_1_5 → Right side, Cabinet 4, Row 1, Column 5
```

**Cabinet Configurations (from storage_params.yaml):**
```yaml
# Left Row
Cabinet 1: 4 columns × 10 rows × 10 departments
Cabinet 2: 4 columns × 10 rows × 10 departments
Cabinet 3: 4 columns × 6 rows × 10 departments
Cabinet 4: 5 columns × 12 rows × 14 departments

# Right Row
Cabinet 1: 5 columns × 12 rows × 14 departments
Cabinet 2: 5 columns × 8 rows × 14 departments
Cabinet 3: 6 columns × 14 rows × 16 departments
Cabinet 4: 4 columns × 10 rows × 10 departments
```

**Joint Group Definitions (kinematic_chains.yaml):**
```yaml
joint_groups:
  navigation:
    joints: [base_main_frame_joint, main_frame_selector_frame_joint]
    default_velocity: 0.5  # m/s
    description: "XZ plane positioning for cabinet access"

  gripper:
    joints: [selector_frame_gripper_joint]
    default_velocity: 0.3
    description: "Y-axis gripper extension"

  picker:
    joints:
      - selector_frame_picker_frame_joint
      - picker_frame_picker_rail_joint
      - picker_rail_picker_base_joint
      - picker_base_picker_jaw_joint
    default_velocity: 0.2
    description: "Picker mechanism control"

  container:
    joints: [selector_left_container_jaw_joint, selector_right_container_jaw_joint]
    default_velocity: 0.1
    mimic_mode: symmetric  # Software mimic for simulation
    description: "Container jaw control"
```

### APIs and Interfaces

**GetAddressCoordinates Service:**
```
# srv/GetAddressCoordinates.srv
# Request
string side          # "left" or "right"
uint8 cabinet_num    # 1-4
uint8 row            # 1 to max_rows
uint8 column         # 1 to max_cols
---
# Response
bool success
geometry_msgs/Pose pose  # Position in world frame
string error_message     # Descriptive error if success=false
```

**NavigateToAddress Action:**
```
# action/NavigateToAddress.action
# Goal
string side              # "left" or "right"
uint8 cabinet_num        # 1-4
uint8 row                # 1 to max_rows
uint8 column             # 1 to max_cols
float64 approach_distance  # Offset from cabinet face (default 0.1m)
---
# Result
bool success
geometry_msgs/Point final_position  # Actual (x, y, z) reached
float64 positioning_error           # Distance from target
string message
---
# Feedback
float64[] current_joint_positions  # [x, z]
float64 distance_to_target
uint8 progress_percent
```

**MoveJointGroup Action (extended):**
```
# action/MoveJointGroup.action
# Goal
string joint_group           # Group name from kinematic_chains.yaml
float64[] target_positions   # Target for each joint in group
float64 max_velocity         # Optional, uses default if not set
---
# Result
bool success
float64[] final_positions
float64[] position_errors
float64 execution_time
---
# Feedback
float64[] current_positions
uint8 progress_percent
```

**Error Codes:**
| Code | Description |
|------|-------------|
| INVALID_SIDE | Side must be "left" or "right" |
| INVALID_CABINET | Cabinet number out of range (1-4) |
| INVALID_ROW | Row exceeds cabinet max rows |
| INVALID_COLUMN | Column exceeds cabinet max columns |
| TF_LOOKUP_FAILED | TF frame not found or timeout |
| MOTION_TIMEOUT | Joint motion exceeded timeout |
| POSITION_ERROR | Final position exceeds tolerance |

### Workflows and Sequencing

**Navigation Sequence (NavigateToAddress):**
```
1. Validate address parameters
   └── AddressResolver.validate_address(side, cabinet, row, column)
   └── Return error if invalid

2. Resolve coordinates
   └── Call GetAddressCoordinates service
   └── Receive (x, y, z) in world frame

3. Calculate joint targets
   └── x_joint = target_x (prismatic, direct mapping)
   └── z_joint = target_z (prismatic, direct mapping)

4. Execute motion
   └── Send MoveJointGroup goal (joint_group="navigation")
   └── Targets: [x_joint, z_joint]
   └── Monitor via feedback at 5 Hz

5. Verify positioning
   └── Calculate error: sqrt((x_target - x_actual)² + (z_target - z_actual)²)
   └── Success if error < 0.02m

6. Update visualization
   └── Publish target address marker (green cube)
   └── Remove marker on completion
```

**Address Resolution Flow:**
```
AddressResolver.get_address_coordinates(side, cabinet, row, column)
    │
    ├─► Validate parameters against storage_params.yaml
    │   └── Check cabinet exists (1-4)
    │   └── Check row in range (1 to max_rows)
    │   └── Check column in range (1 to max_cols)
    │
    ├─► Construct frame name
    │   └── side_abbrev = 'l' if side == 'left' else 'r'
    │   └── frame_name = f"addr_{side_abbrev}_{cabinet}_{row}_{column}"
    │
    ├─► TF2 lookup
    │   └── tf_buffer.lookup_transform('world', frame_name, time)
    │   └── Timeout: 1.0 second
    │
    └─► Return coordinates
        └── (x, y, z) from transform.translation
```

## Non-Functional Requirements

### Performance

| Metric | Target | Verification |
|--------|--------|--------------|
| Navigation time per address | ≤30 seconds | Test suite timing |
| GetAddressCoordinates response | <100ms | Service call measurement |
| TF lookup timeout | 1.0 second | Configuration |
| Feedback publish rate | 5 Hz | Action server implementation |
| Marker update rate | 10 Hz | Marker publisher |

### Security

- No external network access required
- All coordinates resolved locally via TF
- Service calls authenticated by ROS2 DDS security (if enabled)
- No sensitive data in address parameters

### Reliability/Availability

| Requirement | Implementation |
|-------------|----------------|
| 95%+ navigation success rate | Validation in test suite |
| Zero stuck states | Timeout protection on all operations |
| Graceful degradation | Return error with message on failures |
| Repeatable positioning | Same address yields same position ±0.01m |

### Observability

**Logging:**
- DEBUG: All service calls with parameters
- INFO: Navigation start/complete with addresses
- WARN: Position error approaching tolerance
- ERROR: Validation failures, TF lookup failures

**Metrics (via /joint_states):**
- Current joint positions
- Position errors vs. targets

**Visualization:**
- Target address marker (green cube)
- Extracted address markers (red cubes)
- All markers in namespace "manipulator_state"

## Dependencies and Integrations

**Internal Dependencies (Epic 2):**
| Component | Version | Purpose |
|-----------|---------|---------|
| ControllerInterface | Story 2.2 | Joint command abstraction |
| MoveJointGroup action | Story 2.5 | Multi-joint coordination |
| State marker publisher | Story 2.4 | Visualization base |

**External Dependencies:**
| Package | Version | Purpose |
|---------|---------|---------|
| tf2_ros | ROS2 Humble | TF lookups and transforms |
| geometry_msgs | ROS2 Humble | Pose, Point messages |
| visualization_msgs | ROS2 Humble | MarkerArray |
| rclpy | ROS2 Humble | Python ROS2 client |

**Configuration Files:**
| File | Source | Purpose |
|------|--------|---------|
| storage_params.yaml | manipulator_description | Cabinet configurations |
| kinematic_chains.yaml | manipulator_control | Joint group definitions |
| action_servers.yaml | manipulator_control | Timeouts, tolerances |

## Acceptance Criteria (Authoritative)

1. **AC-3.1:** AddressResolver resolves valid addresses to (x, y, z) coordinates via TF lookup
2. **AC-3.2:** AddressResolver validates addresses against storage_params.yaml cabinet configurations
3. **AC-3.3:** GetAddressCoordinates service responds in <100ms for valid addresses
4. **AC-3.4:** GetAddressCoordinates returns descriptive error for invalid addresses
5. **AC-3.5:** MoveJointGroup loads joint group definitions from kinematic_chains.yaml
6. **AC-3.6:** MoveJointGroup "container" group implements software mimic (symmetric jaw motion)
7. **AC-3.7:** NavigateToAddress positions within ±2cm of target (NFR-002)
8. **AC-3.8:** NavigateToAddress completes within 30 seconds timeout
9. **AC-3.9:** Green cube marker appears at target address during navigation
10. **AC-3.10:** Navigation test suite achieves ≥95% success rate across all 8 cabinets

## Traceability Mapping

| AC | Spec Section | Component | Test Approach |
|----|--------------|-----------|---------------|
| AC-3.1 | Data Models / Address Frame | AddressResolver | Unit test with mock TF |
| AC-3.2 | Data Models / Cabinet Configs | AddressResolver | Unit test boundary conditions |
| AC-3.3 | Performance NFRs | GetAddressCoordinates | Service call timing |
| AC-3.4 | APIs / Error Codes | GetAddressCoordinates | Invalid input tests |
| AC-3.5 | Data Models / Joint Groups | MoveJointGroup | Config loading test |
| AC-3.6 | Data Models / Joint Groups | MoveJointGroup | Container jaw test |
| AC-3.7 | Performance NFRs | NavigateToAddress | Position measurement |
| AC-3.8 | Performance NFRs | NavigateToAddress | Timeout test |
| AC-3.9 | Observability | State marker publisher | RViz visual verification |
| AC-3.10 | Reliability NFRs | Navigation test suite | Automated test execution |

## Risks, Assumptions, Open Questions

**Risks:**
| Risk | Impact | Mitigation |
|------|--------|------------|
| TF frame tree incomplete | High - navigation fails | Validate URDF has all address frames before Epic 3 |
| Joint limit violations during navigation | Medium | Validate targets against joint limits in ControllerInterface |
| Gazebo physics drift affects accuracy | Low | Use joint_states for verification, not TF |

**Assumptions:**
- URDF contains complete TF frame tree for all addresses (addr_l_1_1_1 through addr_r_4_max_max)
- storage_params.yaml cabinet configurations match URDF frame definitions
- JointTrajectoryController migration (Story 2.3.1) completed successfully
- ControllerInterface handles both trajectory and forward command joints transparently

**Open Questions:**
- None - all design decisions resolved in architecture document

## Test Strategy Summary

**Test Levels:**

1. **Unit Tests:**
   - AddressResolver validation logic
   - Frame name construction
   - Joint group configuration parsing

2. **Integration Tests:**
   - GetAddressCoordinates service end-to-end
   - MoveJointGroup with kinematic_chains.yaml
   - NavigateToAddress with Gazebo simulation

3. **System Tests (test_epic3_navigation.py):**
   - Address resolution for 10 sample addresses
   - Navigation to 5 addresses with accuracy measurement
   - Cabinet coverage (all 8 cabinets)
   - Marker visualization verification

**Coverage:**
- 100% of acceptance criteria
- All cabinet types (4x10, 5x12, 6x14)
- Both left and right cabinet rows
- Edge cases (first row, last row, first column, last column)

**Test Frameworks:**
- pytest for unit tests
- launch_testing for Gazebo integration
- ros2 service call for service verification
- rqt_action for manual action testing
