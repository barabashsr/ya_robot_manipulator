# Epic Technical Specification: Package Setup & Interface Definitions

Date: 2025-11-25
Author: BMad
Epic ID: epic-1
Status: Draft

---

## Overview

Epic 1 establishes the foundational ROS2 package structure for the ya_robot_manipulator control system and defines all action/service/message interfaces required for autonomous warehouse manipulation. This epic creates the scaffolding that enables all subsequent development by providing a complete, compilable ROS2 package with standardized interface contracts.

The manipulator_control package will house all control logic, action servers, utilities, and simulation helpers for the Level 3 autonomous warehouse system. By defining all interfaces upfront (40+ action/service/message definitions), we ensure consistent communication contracts across the 40-story implementation and enable parallel development of Epic 2+ stories.

## Objectives and Scope

**In Scope:**
- Create manipulator_control ROS2 package with standard directory structure
- Define all action interfaces (Low/Mid/High level - 10+ actions)
- Define all service interfaces (5+ services for utilities and spawning)
- Define custom message types (Address, JointCommand, LimitSwitchState)
- Configure CMakeLists.txt and package.xml with complete dependencies
- Validate package compilation and interface generation
- Document package structure and interface usage

**Out of Scope:**
- Implementation of action servers (Epic 2+)
- Implementation of services and utilities (Epic 2+)
- Launch files and runtime configuration (Epic 2+)
- URDF and controller configurations (already in manipulator_description package)
- Hardware interface layer (Growth phase)

## System Architecture Alignment

**Architecture References:**
- Lines 1689-1773: Complete package structure defined
- Lines 1749-1772: Action/Service/Message interface definitions
- Lines 2391-2414: Action hierarchy (Low/Mid/High level organization)

**Key Constraints:**
- Individual ForwardCommandControllers per joint (Architecture Line 13-28)
- No database at Level 3 - stateless operation (Architecture Line 1659-1684)
- Configuration-driven design - YAML for all parameters (Architecture Line 1013-1066)
- ROS2 Jazzy + Gazebo Harmonic compatibility (Architecture Line 2349-2352)

**Package Dependencies:**
- manipulator_description (URDF, ros2_control config, storage params)
- ros2_control framework (controller_manager, controller_interface)
- Standard ROS2 packages (rclcpp, rclpy, std_msgs, geometry_msgs, sensor_msgs, tf2_ros)
- Gazebo integration (ros_gz_bridge, ros_gz_interfaces)

## Detailed Design

### Services and Modules

**Package:** `manipulator_control`

| Directory | Responsibility | Owner/Role |
|-----------|----------------|------------|
| `action/` | Action interface definitions (.action files) | Dev (Story 1.2) |
| `srv/` | Service interface definitions (.srv files) | Dev (Story 1.3) |
| `msg/` | Custom message definitions (.msg files) | Dev (Story 1.3) |
| `src/` | Python node implementations (future epics) | Dev (Epic 2+) |
| `include/manipulator_control/` | C++ headers (if needed) | Dev (Epic 2+) |
| `launch/` | Launch files for nodes and simulation (future) | Dev (Epic 2+) |
| `config/` | YAML configuration files (future) | Dev (Epic 2+) |
| `test/` | Unit and integration tests (future) | Dev (Epic 6) |
| `CMakeLists.txt` | Build configuration | Dev (Story 1.1) |
| `package.xml` | Package metadata and dependencies | Dev (Story 1.1) |
| `README.md` | Package documentation | Dev (Story 1.1) |

**Build System:**
- CMake-based ROS2 package using ament_cmake
- Generates action/service/message interfaces for Python and C++
- Installs launch files, config files, and Python scripts

### Data Models and Contracts

**Custom Message Types:**

**1. Address.msg** - Warehouse address specification
```
string side          # "left" or "right"
uint8 cabinet_num    # 1-4
uint8 row            # 1-N (per cabinet config)
uint8 column         # 1-N (per cabinet config)
```

**2. JointCommand.msg** - Joint position command
```
string joint_name
float64 target_position
float64 max_velocity  # Optional, 0.0 = use default
```

**3. LimitSwitchState.msg** - Limit switch status
```
string switch_name
bool triggered
float64 trigger_position
float64 current_position
```

**Action Hierarchy:**

**Low-Level Actions (Direct Joint Control):**
- MoveJoint.action
- MoveJointGroup.action

**Mid-Level Actions (Box & Navigation Operations):**
- NavigateToAddress.action
- ExtractBox.action
- ReturnBox.action
- PutBox.action
- MoveBoxToLoad.action
- ManipulateContainer.action
- GetContainer.action
- PlaceContainer.action
- PickItem.action

**High-Level Actions (Composite Workflows):**
- PickItemFromStorage.action

### APIs and Interfaces

**Action Definitions (10+ interfaces):**

**1. MoveJoint.action**
```
# Goal
string joint_name
float64 target_position
float64 max_velocity  # 0.0 = use default
---
# Result
bool success
float64 final_position
float64 execution_time
string message
---
# Feedback
float64 current_position
float64 progress_percent
```

**2. MoveJointGroup.action**
```
# Goal
string[] joint_names
float64[] target_positions
float64 max_velocity  # 0.0 = use default
---
# Result
bool success
float64[] final_positions
float64 position_error
float64 execution_time
string message
---
# Feedback
float64[] current_positions
float64 progress_percent
```

**3. NavigateToAddress.action**
```
# Goal
string side          # "left" or "right"
uint8 cabinet_num
uint8 row
uint8 column
---
# Result
bool success
geometry_msgs/Pose final_pose
float64 execution_time
string message
---
# Feedback
string current_phase      # "navigating_x", "navigating_z", "positioning"
float64 progress_percent
```

**4. ExtractBox.action**
```
# Goal
string side
uint8 cabinet_num
uint8 row
uint8 column
---
# Result
bool success
bool box_extracted
string box_id
float64 execution_time
string message
---
# Feedback
string current_operation  # "navigating", "generating_trajectory", "extending", "engaging_magnet", "retracting"
float64 progress_percent
```

**5. ReturnBox.action**
```
# Goal
# No parameters - returns currently held box to original address
---
# Result
bool success
bool box_returned
string box_id
string returned_to_address
float64 execution_time
string message
---
# Feedback
string current_operation  # "navigating", "extending", "releasing", "retracting"
float64 progress_percent
```

**6. PutBox.action**
```
# Goal
string target_side
uint8 target_cabinet_num
uint8 target_row
uint8 target_column
---
# Result
bool success
bool box_placed
string box_id
string placed_at_address
float64 execution_time
string message
---
# Feedback
string current_operation
float64 progress_percent
```

**7. MoveBoxToLoad.action**
```
# Goal
string source_side
uint8 source_cabinet_num
uint8 source_row
uint8 source_column
string load_position_name  # From load_positions.yaml
bool return_to_source      # If false, optionally specify new address
string new_side            # Optional, for relocation
uint8 new_cabinet_num      # Optional
uint8 new_row              # Optional
uint8 new_column           # Optional
---
# Result
bool success
string box_id
bool box_at_load
bool box_relocated
string message
---
# Feedback
string current_phase       # "extracting", "moving_to_load", "at_load_position", "returning", "relocating", "completed"
string current_operation
float64 progress_percent
float64 elapsed_time
```

**8. ManipulateContainer.action**
```
# Goal
string command             # "open" or "close"
float64 target_width       # Jaw opening width (m), 0.0 for "close"
---
# Result
bool success
float64 final_left_position
float64 final_right_position
float64 actual_width
float64 execution_time
string message
---
# Feedback
float64 left_jaw_position
float64 right_jaw_position
float64 current_width
float64 progress_percent
```

**9. GetContainer.action / PlaceContainer.action**
```
# Goal
string container_id        # From container_storage.yaml
---
# Result
bool success
string container_id
float64 execution_time
string message
---
# Feedback
string current_operation   # "opening_jaws", "navigating", "lowering", "gripping", "lifting"
float64 progress_percent
```

**10. PickItem.action**
```
# Goal
uint8 department_num       # 1-N, department within extracted box
---
# Result
bool success
bool item_picked
uint8 department_num
float64 execution_time
string message
---
# Feedback
string current_state       # State machine: "IDLE", "APPROACH", "OPEN_JAW", "EXTEND", "CLOSE", "RETRACT", "LIFT", "SUCCESS"
bool picker_jaw_opened
bool picker_jaw_extended
bool picker_jaw_closed
bool picker_jaw_retracted
float64 elapsed_time
```

**11. PickItemFromStorage.action**
```
# Goal
string side
uint8 cabinet_num
uint8 row
uint8 column
uint8 department_num
string container_id        # Optional, if container needed
---
# Result
bool success
bool item_retrieved
string box_id
uint8 department_num
float64 total_time
string message
---
# Feedback
string current_phase       # "get_container", "navigate", "extract", "pick", "return"
string operation_step
float64 progress_percent
float64 elapsed_time
```

**Service Definitions (5+ interfaces):**

**1. GetAddressCoordinates.srv**
```
# Request
string side
uint8 cabinet_num
uint8 row
uint8 column
---
# Response
bool success
geometry_msgs/Pose pose
string error_message
```

**2. ToggleElectromagnet.srv**
```
# Request
bool activate             # true = engage, false = release
---
# Response
bool success
bool magnet_engaged
string message
```

**3. SpawnBox.srv**
```
# Request
string box_id             # Format: box_{side}_{cabinet}_{row}_{col}
string side
uint8 cabinet_num
uint8 row
uint8 column
---
# Response
bool success
string box_id
uint8 department_count
string message
```

**4. DespawnBox.srv**
```
# Request
string box_id
---
# Response
bool success
string message
```

**5. ValidateAddress.srv**
```
# Request
string side
uint8 cabinet_num
uint8 row
uint8 column
bool check_empty          # Check if address is currently unoccupied
bool check_width_match    # Check if box width matches cabinet column count
uint8 box_width           # Required if check_width_match = true
---
# Response
bool valid
bool is_empty
bool width_compatible
string error_message
```

### Workflows and Sequencing

**Epic 1 Implementation Sequence:**

```
Story 1.1: Package Structure
├─ Create directory structure
├─ Write CMakeLists.txt
├─ Write package.xml
├─ Write README.md
└─ Verify: colcon build succeeds

Story 1.2: Action Definitions
├─ Create action/ directory
├─ Write MoveJoint.action
├─ Write MoveJointGroup.action
├─ Write NavigateToAddress.action
├─ Write ExtractBox.action
├─ Write ReturnBox.action
├─ Write PutBox.action
├─ Write MoveBoxToLoad.action
├─ Write ManipulateContainer.action
├─ Write GetContainer.action
├─ Write PlaceContainer.action
├─ Write PickItem.action
├─ Write PickItemFromStorage.action
└─ Verify: colcon build generates interfaces

Story 1.3: Service and Message Definitions
├─ Create srv/ directory
├─ Write GetAddressCoordinates.srv
├─ Write ToggleElectromagnet.srv
├─ Write SpawnBox.srv
├─ Write DespawnBox.srv
├─ Write ValidateAddress.srv
├─ Create msg/ directory
├─ Write Address.msg
├─ Write JointCommand.msg
├─ Write LimitSwitchState.msg
└─ Verify: colcon build generates interfaces
```

**Validation Flow:**
1. Create package structure → Verify build succeeds (no interfaces yet)
2. Add action definitions → Verify action interfaces generated
3. Add service/message definitions → Verify all interfaces generated
4. Test interface imports in Python: `from manipulator_control.action import MoveJoint`
5. Test interface imports in C++: `#include <manipulator_control/action/move_joint.hpp>`

## Non-Functional Requirements

### Performance

**Build Performance:**
- Package compilation time: < 30 seconds on standard development machine
- Interface generation time: < 10 seconds for all 15+ interfaces
- No compilation warnings or errors

**Interface Generation:**
- All action/service/message interfaces must generate Python and C++ bindings
- Generated files must be importable immediately after build

### Security

**N/A for Epic 1** - No runtime security concerns for interface definitions.

**Build Security:**
- Use official ROS2 packages from apt repositories
- Pin ROS2 Jazzy version in package.xml
- No third-party or unverified dependencies

### Reliability/Availability

**Build Reliability:**
- Package must build cleanly in CI/CD environment
- colcon build must succeed with zero errors, zero warnings
- Interface generation must be deterministic (same output for same input)

**Dependency Management:**
- All dependencies must be available in ROS2 Jazzy apt repositories
- No dependency on deprecated or EOL packages

### Observability

**Build Observability:**
- CMake output must clearly indicate interface generation progress
- Build errors must provide actionable error messages
- Generated interface files must be inspectable in install/manipulator_control/

**Documentation:**
- README.md documents package purpose and build instructions
- Action/service/message files include inline comments explaining fields
- CMakeLists.txt includes comments for each major section

## Dependencies and Integrations

**ROS2 Core Dependencies:**
```xml
<!-- package.xml -->
<depend>rclcpp</depend>
<depend>rclpy</depend>
<depend>std_msgs</depend>
<depend>geometry_msgs</depend>
<depend>sensor_msgs</depend>
<depend>std_srvs</depend>
<depend>action_msgs</depend>
<depend>rosidl_default_generators</depend>
<exec_depend>rosidl_default_runtime</exec_depend>
<member_of_group>rosidl_interface_packages</member_of_group>
```

**TF2 Dependencies:**
```xml
<depend>tf2</depend>
<depend>tf2_ros</depend>
<depend>tf2_geometry_msgs</depend>
```

**ros2_control Dependencies:**
```xml
<depend>controller_manager</depend>
<depend>controller_interface</depend>
<depend>ros2_control</depend>
<depend>ros2_controllers</depend>
```

**Gazebo Integration Dependencies:**
```xml
<depend>ros_gz_bridge</depend>
<depend>ros_gz_interfaces</depend>
<depend>ros_gz_sim</depend>
```

**Build Dependencies:**
```xml
<buildtool_depend>ament_cmake</buildtool_depend>
<buildtool_depend>ament_cmake_python</buildtool_depend>
```

**Test Dependencies (Future):**
```xml
<test_depend>ament_lint_auto</test_depend>
<test_depend>ament_lint_common</test_depend>
<test_depend>launch_testing_ament_cmake</test_depend>
```

**Package Dependencies:**
- manipulator_description (sibling package with URDF and config)

**Integration Points:**
- manipulator_description provides URDF, ros2_control config, storage_params.yaml
- manipulator_control consumes these at runtime (Epic 2+)
- No runtime integration in Epic 1 (interfaces only)

## Acceptance Criteria (Authoritative)

**AC-1: Package Structure Created**
- [ ] manipulator_control package exists at ros2_ws/src/manipulator_control/
- [ ] Directory structure includes: action/, srv/, msg/, src/, launch/, config/, test/
- [ ] CMakeLists.txt exists with ament_cmake configuration
- [ ] package.xml exists with all dependencies listed
- [ ] README.md exists with package description

**AC-2: Package Compiles Successfully**
- [ ] `colcon build --packages-select manipulator_control` exits with code 0
- [ ] No compilation errors
- [ ] No compilation warnings
- [ ] Build completes in < 30 seconds

**AC-3: All Action Interfaces Defined**
- [ ] 12 action files created in action/ directory
- [ ] Each action has Goal, Result, Feedback sections
- [ ] Generated Python interfaces importable: `from manipulator_control.action import *`
- [ ] Generated C++ headers exist in install/manipulator_control/include/

**AC-4: All Service Interfaces Defined**
- [ ] 5 service files created in srv/ directory
- [ ] Each service has Request and Response sections
- [ ] Generated Python interfaces importable: `from manipulator_control.srv import *`

**AC-5: All Message Interfaces Defined**
- [ ] 3 message files created in msg/ directory
- [ ] Address.msg, JointCommand.msg, LimitSwitchState.msg exist
- [ ] Generated Python interfaces importable: `from manipulator_control.msg import *`

**AC-6: Interface Documentation**
- [ ] Each .action file includes comments explaining goal/result/feedback
- [ ] Each .srv file includes comments explaining request/response
- [ ] Each .msg file includes comments explaining fields
- [ ] README.md documents how to use interfaces

**AC-7: Build System Configuration**
- [ ] CMakeLists.txt configures rosidl_generate_interfaces for all interfaces
- [ ] CMakeLists.txt includes ament_export_dependencies
- [ ] package.xml includes rosidl_default_generators and rosidl_default_runtime
- [ ] package.xml member_of_group rosidl_interface_packages

## Traceability Mapping

| AC | PRD Requirement | Architecture Reference | Components/Files | Test Validation |
|----|-----------------|------------------------|------------------|-----------------|
| AC-1 | FR-013 (YAML Config) | Lines 1691-1773 | CMakeLists.txt, package.xml, README.md | Story 1.1 verification |
| AC-2 | NFR-011 (Config Mgmt) | Lines 1013-1066 | Build system | `colcon build` exit code 0 |
| AC-3 | FR-011 (Joint Control) | Lines 1749-1772 | action/*.action files | Import test in Story 1.2 |
| AC-4 | FR-002 (Address Resolution) | Lines 121-135 | srv/*.srv files | Import test in Story 1.3 |
| AC-5 | FR-001 (Navigation) | Lines 67-70 | msg/Address.msg | Import test in Story 1.3 |
| AC-6 | NFR-012 (Logging/Debug) | N/A | Interface comments | Manual review |
| AC-7 | NFR-010 (ROS2 Standards) | Lines 2349-2352 | CMakeLists.txt, package.xml | Build system validation |

## Risks, Assumptions, Open Questions

**Risks:**

**R1: Interface Changes During Implementation**
- **Likelihood:** Medium
- **Impact:** Low (interfaces can be versioned)
- **Mitigation:** Define comprehensive interfaces upfront based on architecture; allow additions but avoid breaking changes
- **Next Step:** Review interfaces in Story 1.2 before proceeding to Epic 2

**R2: Missing Dependencies on Target Platform**
- **Likelihood:** Low
- **Impact:** High (blocks all development)
- **Mitigation:** Validate all apt packages available in ROS2 Jazzy repository; test on Ubuntu 24.04
- **Next Step:** Verify ros-jazzy-* packages installed in Story 1.1

**Assumptions:**

**A1:** manipulator_description package already exists with URDF and ros2_control config (verified: examples/ar4_ros_driver exists in repo)

**A2:** ROS2 Jazzy and Gazebo Harmonic are installed on development machine

**A3:** colcon workspace is initialized at ros2_ws/

**A4:** Interface definitions can be finalized without runtime testing (will be validated in Epic 2+)

**Open Questions:**

**Q1:** Should we create separate packages for actions, services, messages (like nav2_msgs)?
- **Answer:** No, single manipulator_control package is sufficient for this project size

**Q2:** Do we need C++ implementations, or is Python sufficient?
- **Answer:** Python sufficient for MVP (simulation); C++ may be needed for hardware deployment (Growth phase)

**Q3:** Should custom messages use standard ROS2 types where possible (e.g., geometry_msgs/Pose for address)?
- **Answer:** Use custom Address.msg for semantic clarity; use standard types (Pose, Twist, etc.) where appropriate

## Test Strategy Summary

**Epic 1 Test Strategy:**

**Build Verification Tests:**
1. **Package Creation Test** (Story 1.1)
   - Test: Run `colcon build --packages-select manipulator_control`
   - Pass Criteria: Exit code 0, no errors/warnings
   - Frequency: After Story 1.1 completion

2. **Interface Generation Test** (Story 1.2, 1.3)
   - Test: Verify generated files exist in install/manipulator_control/
   - Pass Criteria: All .action, .srv, .msg files generated
   - Frequency: After each interface definition story

3. **Python Import Test** (Story 1.2, 1.3)
   - Test: `python3 -c "from manipulator_control.action import MoveJoint"`
   - Pass Criteria: Import succeeds with no errors
   - Frequency: After interface generation

4. **C++ Include Test** (Story 1.2, 1.3)
   - Test: Create minimal C++ node that includes generated headers
   - Pass Criteria: Compilation succeeds
   - Frequency: After interface generation (optional for MVP)

**Integration Tests (Future Epics):**
- Action server functionality tests (Epic 2+)
- Service call/response tests (Epic 2+)
- Message publishing/subscription tests (Epic 2+)

**Test Automation:**
- Use colcon test for automated build verification
- Create simple test scripts in test/ directory (Epic 6)
- CI/CD pipeline runs colcon build + colcon test on every commit (Future)

**Success Criteria:**
- All build verification tests pass
- All interfaces importable in Python
- No blocking issues for Epic 2 development
