# Story 1.3: Define Service and Message Interfaces

Status: review

## Story

As a **developer**,
I want **all service (.srv) and message (.msg) interface definitions created in the manipulator_control package**,
so that **I can implement service servers and use custom message types in subsequent epics**.

## Acceptance Criteria

1. **AC-1:** 5 service files exist in `ros2_ws/src/manipulator_control/srv/` directory
2. **AC-2:** 3 message files exist in `ros2_ws/src/manipulator_control/msg/` directory
3. **AC-3:** Each service file has properly defined Request and Response sections
4. **AC-4:** CMakeLists.txt updated with rosidl_generate_interfaces for all srv and msg files
5. **AC-5:** `colcon build --packages-select manipulator_control` exits with code 0, no errors, no warnings
6. **AC-6:** Python interfaces importable: `from manipulator_control.srv import GetAddressCoordinates, ...`
7. **AC-7:** Python interfaces importable: `from manipulator_control.msg import Address, JointCommand, LimitSwitchState`
8. **AC-8:** Each interface file includes comments explaining fields

## Tasks / Subtasks

- [x] Task 1: Create Service Definitions (AC: 1, 3, 8)
  - [x] 1.1 Create `GetAddressCoordinates.srv` - resolve warehouse address to pose
  - [x] 1.2 Create `ToggleElectromagnet.srv` - activate/deactivate magnet
  - [x] 1.3 Create `SpawnBox.srv` - spawn box in Gazebo at address
  - [x] 1.4 Create `DespawnBox.srv` - remove box from Gazebo
  - [x] 1.5 Create `ValidateAddress.srv` - validate address and check occupancy

- [x] Task 2: Create Message Definitions (AC: 2, 8)
  - [x] 2.1 Create `Address.msg` - warehouse address (side, cabinet, row, column)
  - [x] 2.2 Create `JointCommand.msg` - joint position command
  - [x] 2.3 Create `LimitSwitchState.msg` - limit switch status

- [x] Task 3: Update CMakeLists.txt for Interface Generation (AC: 4)
  - [x] 3.1 Add all 5 srv files to rosidl_generate_interfaces
  - [x] 3.2 Add all 3 msg files to rosidl_generate_interfaces
  - [x] 3.3 Ensure DEPENDENCIES includes geometry_msgs (used by GetAddressCoordinates.srv)

- [x] Task 4: Validate Build and Interface Generation (AC: 5, 6, 7)
  - [x] 4.1 Run `colcon build --packages-select manipulator_control`
  - [x] 4.2 Verify exit code 0, no errors, no warnings
  - [x] 4.3 Test Python srv imports
  - [x] 4.4 Test Python msg imports
  - [x] 4.5 Verify C++ headers generated

## Dev Notes

### Learnings from Previous Stories

**From Story 1-1-create-ros2-package-structure (Status: review)**
- Package structure created at `ros2_ws/src/manipulator_control/`
- `ros2_control` package not available - use `controller_manager`, `controller_interface`, `ros2_controllers` instead
- Build verified: exit code 0

**From Story 1-2-define-all-action-interfaces (Status: review)**
- 12 action files created and verified
- CMakeLists.txt rosidl_generate_interfaces pattern established
- Build time: ~21.6s
- Python imports: all 12 actions verified
- C++ headers: 84 generated

[Source: docs/sprint-artifacts/1-1-create-ros2-package-structure.md#Dev-Agent-Record]
[Source: docs/sprint-artifacts/1-2-define-all-action-interfaces.md#Dev-Agent-Record]

### Architecture References

- [Source: docs/sprint-artifacts/tech-spec-epic-1.md#Data-Models] Custom message definitions (Lines 80-103)
- [Source: docs/sprint-artifacts/tech-spec-epic-1.md#Service-Definitions] Service interface definitions (Lines 355-424)

### Service Definitions (from tech-spec)

**1. GetAddressCoordinates.srv** - Uses geometry_msgs/Pose
```
# Request
string side, uint8 cabinet_num, uint8 row, uint8 column
# Response
bool success, geometry_msgs/Pose pose, string error_message
```

**2. ToggleElectromagnet.srv**
```
# Request
bool activate
# Response
bool success, bool magnet_engaged, string message
```

**3. SpawnBox.srv**
```
# Request
string box_id, string side, uint8 cabinet_num, uint8 row, uint8 column
# Response
bool success, string box_id, uint8 department_count, string message
```

**4. DespawnBox.srv**
```
# Request
string box_id
# Response
bool success, string message
```

**5. ValidateAddress.srv**
```
# Request
string side, uint8 cabinet_num, uint8 row, uint8 column, bool check_empty, bool check_width_match, uint8 box_width
# Response
bool valid, bool is_empty, bool width_compatible, string error_message
```

### Message Definitions (from tech-spec)

**1. Address.msg** - Warehouse address
```
string side          # "left" or "right"
uint8 cabinet_num    # 1-4
uint8 row            # 1-N
uint8 column         # 1-N
```

**2. JointCommand.msg** - Joint position command
```
string joint_name
float64 target_position
float64 max_velocity  # 0.0 = use default
```

**3. LimitSwitchState.msg** - Limit switch status
```
string switch_name
bool triggered
float64 trigger_position
float64 current_position
```

### Technical Constraints

- ROS2 Jazzy + Gazebo Harmonic compatibility
- GetAddressCoordinates.srv uses geometry_msgs/Pose - DEPENDENCIES already includes geometry_msgs
- Follow exact interface definitions from tech-spec-epic-1.md
- Include field comments for each interface

### CMakeLists.txt Update Pattern

Add to existing rosidl_generate_interfaces block:
```cmake
rosidl_generate_interfaces(${PROJECT_NAME}
  # Actions (from Story 1.2)
  "action/MoveJoint.action"
  ... (12 actions)
  # Services (Story 1.3)
  "srv/GetAddressCoordinates.srv"
  "srv/ToggleElectromagnet.srv"
  "srv/SpawnBox.srv"
  "srv/DespawnBox.srv"
  "srv/ValidateAddress.srv"
  # Messages (Story 1.3)
  "msg/Address.msg"
  "msg/JointCommand.msg"
  "msg/LimitSwitchState.msg"
  DEPENDENCIES geometry_msgs
)
```

### Test Requirements (MANDATORY after implementation)

**Build Verification:**
```bash
# Must pass with exit code 0, no errors, no warnings
cd /home/robo/robo/ya_robot_manipulator/ros2_ws
colcon build --packages-select manipulator_control
echo "Exit code: $?"
```

**Python Service Import Tests:**
```bash
# Source the workspace first
source /home/robo/robo/ya_robot_manipulator/ros2_ws/install/setup.bash

# Test all service imports (must all succeed)
python3 -c "from manipulator_control.srv import GetAddressCoordinates; print('GetAddressCoordinates OK')"
python3 -c "from manipulator_control.srv import ToggleElectromagnet; print('ToggleElectromagnet OK')"
python3 -c "from manipulator_control.srv import SpawnBox; print('SpawnBox OK')"
python3 -c "from manipulator_control.srv import DespawnBox; print('DespawnBox OK')"
python3 -c "from manipulator_control.srv import ValidateAddress; print('ValidateAddress OK')"
```

**Python Message Import Tests:**
```bash
# Test all message imports (must all succeed)
python3 -c "from manipulator_control.msg import Address; print('Address OK')"
python3 -c "from manipulator_control.msg import JointCommand; print('JointCommand OK')"
python3 -c "from manipulator_control.msg import LimitSwitchState; print('LimitSwitchState OK')"
```

**C++ Header Generation Verification:**
```bash
# Verify srv headers exist
ls /home/robo/robo/ya_robot_manipulator/ros2_ws/install/manipulator_control/include/manipulator_control/manipulator_control/srv/*.hpp 2>/dev/null | wc -l
# Expected: 15+ headers (Request, Response, etc. for each service)

# Verify msg headers exist
ls /home/robo/robo/ya_robot_manipulator/ros2_ws/install/manipulator_control/include/manipulator_control/manipulator_control/msg/*.hpp 2>/dev/null | wc -l
# Expected: 9+ headers (3 messages × 3 types each)
```

**File Count Verification:**
```bash
# Verify service files
ls /home/robo/robo/ya_robot_manipulator/ros2_ws/src/manipulator_control/srv/*.srv | wc -l
# Expected: 5

# Verify message files
ls /home/robo/robo/ya_robot_manipulator/ros2_ws/src/manipulator_control/msg/*.msg | wc -l
# Expected: 3
```

**Complete Interface Test (all actions + services + messages):**
```bash
source /home/robo/robo/ya_robot_manipulator/ros2_ws/install/setup.bash
python3 << 'EOF'
# Test all interfaces
try:
    # Actions (12)
    from manipulator_control.action import MoveJoint, MoveJointGroup, NavigateToAddress
    from manipulator_control.action import ExtractBox, ReturnBox, PutBox, MoveBoxToLoad
    from manipulator_control.action import ManipulateContainer, GetContainer, PlaceContainer
    from manipulator_control.action import PickItem, PickItemFromStorage
    print("✓ All 12 actions imported")

    # Services (5)
    from manipulator_control.srv import GetAddressCoordinates, ToggleElectromagnet
    from manipulator_control.srv import SpawnBox, DespawnBox, ValidateAddress
    print("✓ All 5 services imported")

    # Messages (3)
    from manipulator_control.msg import Address, JointCommand, LimitSwitchState
    print("✓ All 3 messages imported")

    print("\n✅ ALL 20 INTERFACES VERIFIED!")
except ImportError as e:
    print(f"❌ Import failed: {e}")
EOF
```

### References

- [Source: docs/sprint-artifacts/tech-spec-epic-1.md#Acceptance-Criteria] AC-4, AC-5 service/message criteria
- [Source: docs/sprint-artifacts/tech-spec-epic-1.md#Test-Strategy] Interface generation and import tests

## Dev Agent Record

### Context Reference

- `docs/sprint-artifacts/1-3-define-service-and-message-interfaces.context.xml`

### Agent Model Used

Claude Sonnet 4.5 (claude-sonnet-4-5-20250929)

### Debug Log References

**Implementation Sequence:**
1. Created 5 service files following exact specifications from tech-spec-epic-1.md:355-424
   - GetAddressCoordinates.srv (uses geometry_msgs/Pose)
   - ToggleElectromagnet.srv
   - SpawnBox.srv
   - DespawnBox.srv
   - ValidateAddress.srv
2. Created 3 message files following exact specifications from tech-spec-epic-1.md:80-103
   - Address.msg (warehouse address)
   - JointCommand.msg (joint position command)
   - LimitSwitchState.msg (limit switch status)
3. Updated CMakeLists.txt: added 5 srv + 3 msg to existing rosidl_generate_interfaces block
4. Build: 24.9s, exit code 0, no errors/warnings
5. Python imports: All 5 services + 3 messages imported successfully
6. Comprehensive test: All 20 interfaces verified (12 actions + 5 services + 3 messages)
7. C++ headers: 35 service headers + 22 message headers generated

**Validation Results:**
- Service file count: 5 ✓
- Message file count: 3 ✓
- Build exit code: 0 ✓
- Build warnings: 0 ✓
- Python srv imports: 5/5 ✓
- Python msg imports: 3/3 ✓
- Service C++ headers: 35 ✓
- Message C++ headers: 22 ✓
- Total interfaces: 20/20 ✓

### Completion Notes List

**Story 1.3 Complete:**
- All 4 tasks completed with all subtasks
- 5 service files created in ros2_ws/src/manipulator_control/srv/
- 3 message files created in ros2_ws/src/manipulator_control/msg/
- Each service has Request/Response sections with field comments
- Each message has field definitions with comments
- CMakeLists.txt updated with all srv and msg files
- Build successful: 24.9s, exit code 0
- All Python srv/msg imports verified
- C++ headers generated (35 srv + 22 msg = 57 total)
- **Epic 1 Complete**: All 20 interfaces defined and verified (12 actions + 5 services + 3 messages)
- All ACs satisfied (AC-1 through AC-8)

### File List

**Created:**
- ros2_ws/src/manipulator_control/srv/GetAddressCoordinates.srv
- ros2_ws/src/manipulator_control/srv/ToggleElectromagnet.srv
- ros2_ws/src/manipulator_control/srv/SpawnBox.srv
- ros2_ws/src/manipulator_control/srv/DespawnBox.srv
- ros2_ws/src/manipulator_control/srv/ValidateAddress.srv
- ros2_ws/src/manipulator_control/msg/Address.msg
- ros2_ws/src/manipulator_control/msg/JointCommand.msg
- ros2_ws/src/manipulator_control/msg/LimitSwitchState.msg

**Modified:**
- ros2_ws/src/manipulator_control/CMakeLists.txt (added 5 srv + 3 msg to rosidl_generate_interfaces)

## Change Log

| Date | Change | Author |
|------|--------|--------|
| 2025-11-25 | Story drafted from tech-spec-epic-1.md with learnings from Stories 1.1 and 1.2 | SM Agent (Bob) |
| 2025-11-25 | 5 service + 3 message interfaces created, CMakeLists.txt updated, build verified (24.9s, exit 0), all 20 interfaces validated | Dev Agent (Amelia) |
