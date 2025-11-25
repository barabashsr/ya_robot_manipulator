# Story 1.2: Define All Action Interfaces

Status: review

## Story

As a **developer**,
I want **all action interface definitions (.action files) created in the manipulator_control package**,
so that **I can implement action servers in subsequent epics with consistent, well-defined contracts**.

## Acceptance Criteria

1. **AC-1:** 12 action files exist in `ros2_ws/src/manipulator_control/action/` directory
2. **AC-2:** Each action file has properly defined Goal, Result, and Feedback sections
3. **AC-3:** CMakeLists.txt updated with rosidl_generate_interfaces for all action files
4. **AC-4:** `colcon build --packages-select manipulator_control` exits with code 0, no errors, no warnings
5. **AC-5:** Python interfaces importable: `from manipulator_control.action import MoveJoint, MoveJointGroup, ...`
6. **AC-6:** C++ headers generated in `install/manipulator_control/include/`
7. **AC-7:** Each action file includes comments explaining goal/result/feedback fields

## Tasks / Subtasks

- [x] Task 1: Create Low-Level Action Definitions (AC: 1, 2, 7)
  - [x] 1.1 Create `MoveJoint.action` - single joint position control
  - [x] 1.2 Create `MoveJointGroup.action` - multiple joints synchronized

- [x] Task 2: Create Mid-Level Navigation Actions (AC: 1, 2, 7)
  - [x] 2.1 Create `NavigateToAddress.action` - move to warehouse address
  - [x] 2.2 Create `ExtractBox.action` - extract box from storage
  - [x] 2.3 Create `ReturnBox.action` - return held box to original address
  - [x] 2.4 Create `PutBox.action` - place box at target address
  - [x] 2.5 Create `MoveBoxToLoad.action` - extract, move to load, optionally relocate

- [x] Task 3: Create Container/Picker Actions (AC: 1, 2, 7)
  - [x] 3.1 Create `ManipulateContainer.action` - open/close container jaws
  - [x] 3.2 Create `GetContainer.action` - retrieve container from storage
  - [x] 3.3 Create `PlaceContainer.action` - place container back
  - [x] 3.4 Create `PickItem.action` - pick item from department using limit switches

- [x] Task 4: Create High-Level Composite Action (AC: 1, 2, 7)
  - [x] 4.1 Create `PickItemFromStorage.action` - complete workflow: get container, navigate, extract, pick, return

- [x] Task 5: Update CMakeLists.txt for Interface Generation (AC: 3)
  - [x] 5.1 Uncomment/add rosidl_generate_interfaces block
  - [x] 5.2 List all 12 action files
  - [x] 5.3 Add DEPENDENCIES for geometry_msgs (used by NavigateToAddress)

- [x] Task 6: Validate Build and Interface Generation (AC: 4, 5, 6)
  - [x] 6.1 Run `colcon build --packages-select manipulator_control`
  - [x] 6.2 Verify exit code 0, no errors, no warnings
  - [x] 6.3 Test Python import: `python3 -c "from manipulator_control.action import MoveJoint"`
  - [x] 6.4 Verify C++ headers exist in install/manipulator_control/include/

## Dev Notes

### Learnings from Previous Story

**From Story 1-1-create-ros2-package-structure (Status: review)**

- **Package Structure Created**: `ros2_ws/src/manipulator_control/` with all subdirectories
- **Build Configuration Ready**: CMakeLists.txt and package.xml configured for interface generation
- **Dependency Correction**: `ros2_control` package not available - use `controller_manager`, `controller_interface`, `ros2_controllers` instead
- **Build Verified**: colcon build exits with code 0, no errors, no warnings

[Source: docs/sprint-artifacts/1-1-create-ros2-package-structure.md#Dev-Agent-Record]

### Architecture References

- [Source: docs/sprint-artifacts/tech-spec-epic-1.md#APIs-and-Interfaces] All 12 action definitions (Lines 127-353)
- [Source: docs/architecture-ros2-control-v2-CORRECTIONS.md] Action hierarchy: Low/Mid/High level organization

### Action Hierarchy

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

### Technical Constraints

- ROS2 Jazzy + Gazebo Harmonic compatibility
- Actions requiring geometry_msgs/Pose must declare DEPENDENCIES in rosidl_generate_interfaces
- Follow exact interface definitions from tech-spec-epic-1.md
- Include field comments for each action

### CMakeLists.txt Update Pattern

```cmake
rosidl_generate_interfaces(${PROJECT_NAME}
  "action/MoveJoint.action"
  "action/MoveJointGroup.action"
  "action/NavigateToAddress.action"
  "action/ExtractBox.action"
  "action/ReturnBox.action"
  "action/PutBox.action"
  "action/MoveBoxToLoad.action"
  "action/ManipulateContainer.action"
  "action/GetContainer.action"
  "action/PlaceContainer.action"
  "action/PickItem.action"
  "action/PickItemFromStorage.action"
  DEPENDENCIES geometry_msgs
)
```

### Test Requirements (MANDATORY after implementation)

**Build Verification:**
```bash
# Must pass with exit code 0, no errors, no warnings
colcon build --packages-select manipulator_control
```

**Python Interface Import Tests:**
```bash
# Source the workspace first
source install/setup.bash

# Test all action imports (must all succeed)
python3 -c "from manipulator_control.action import MoveJoint; print('MoveJoint OK')"
python3 -c "from manipulator_control.action import MoveJointGroup; print('MoveJointGroup OK')"
python3 -c "from manipulator_control.action import NavigateToAddress; print('NavigateToAddress OK')"
python3 -c "from manipulator_control.action import ExtractBox; print('ExtractBox OK')"
python3 -c "from manipulator_control.action import ReturnBox; print('ReturnBox OK')"
python3 -c "from manipulator_control.action import PutBox; print('PutBox OK')"
python3 -c "from manipulator_control.action import MoveBoxToLoad; print('MoveBoxToLoad OK')"
python3 -c "from manipulator_control.action import ManipulateContainer; print('ManipulateContainer OK')"
python3 -c "from manipulator_control.action import GetContainer; print('GetContainer OK')"
python3 -c "from manipulator_control.action import PlaceContainer; print('PlaceContainer OK')"
python3 -c "from manipulator_control.action import PickItem; print('PickItem OK')"
python3 -c "from manipulator_control.action import PickItemFromStorage; print('PickItemFromStorage OK')"
```

**C++ Header Generation Verification:**
```bash
# Verify headers exist
ls install/manipulator_control/include/manipulator_control/action/*.hpp | wc -l
# Expected: 12 or more header files (Goal, Result, Feedback for each action)
```

**File Count Verification:**
```bash
# Verify all 12 action files created
ls ros2_ws/src/manipulator_control/action/*.action | wc -l
# Expected: 12
```

### References

- [Source: docs/sprint-artifacts/tech-spec-epic-1.md#Acceptance-Criteria] AC-3 action interface criteria
- [Source: docs/sprint-artifacts/tech-spec-epic-1.md#Test-Strategy] Interface generation and import tests

## Dev Agent Record

### Context Reference

- `docs/sprint-artifacts/1-2-define-all-action-interfaces.context.xml`

### Agent Model Used

Claude Sonnet 4.5 (claude-sonnet-4-5-20250929)

### Debug Log References

**Implementation Sequence:**
1. Created 12 action files following exact specifications from tech-spec-epic-1.md:127-353
2. Low-level: MoveJoint, MoveJointGroup (direct joint control)
3. Mid-level: NavigateToAddress, ExtractBox, ReturnBox, PutBox, MoveBoxToLoad (box/navigation ops)
4. Container/Picker: ManipulateContainer, GetContainer, PlaceContainer, PickItem
5. High-level: PickItemFromStorage (composite workflow)
6. Updated CMakeLists.txt: uncommented rosidl_generate_interfaces, added all 12 actions with geometry_msgs dependency
7. Build: 21.6s, exit code 0, no errors/warnings
8. Python imports: All 12 actions imported successfully
9. C++ headers: 84 headers generated (12 actions × 7 types)

**Validation Results:**
- Action file count: 12 ✓
- Build exit code: 0 ✓
- Build warnings: 0 ✓
- Python imports: 12/12 ✓
- C++ headers: 84 generated ✓

### Completion Notes List

**Story 1.2 Complete:**
- All 6 tasks completed with all subtasks
- 12 action files created in ros2_ws/src/manipulator_control/action/
- Each action has Goal, Result, Feedback sections with field comments
- CMakeLists.txt updated with rosidl_generate_interfaces
- Build successful: 21.6s, exit code 0
- All Python imports verified
- C++ headers generated at install/manipulator_control/include/manipulator_control/manipulator_control/action/
- All ACs satisfied (AC-1 through AC-7)

### File List

**Created:**
- ros2_ws/src/manipulator_control/action/MoveJoint.action
- ros2_ws/src/manipulator_control/action/MoveJointGroup.action
- ros2_ws/src/manipulator_control/action/NavigateToAddress.action
- ros2_ws/src/manipulator_control/action/ExtractBox.action
- ros2_ws/src/manipulator_control/action/ReturnBox.action
- ros2_ws/src/manipulator_control/action/PutBox.action
- ros2_ws/src/manipulator_control/action/MoveBoxToLoad.action
- ros2_ws/src/manipulator_control/action/ManipulateContainer.action
- ros2_ws/src/manipulator_control/action/GetContainer.action
- ros2_ws/src/manipulator_control/action/PlaceContainer.action
- ros2_ws/src/manipulator_control/action/PickItem.action
- ros2_ws/src/manipulator_control/action/PickItemFromStorage.action

**Modified:**
- ros2_ws/src/manipulator_control/CMakeLists.txt (uncommented rosidl_generate_interfaces, added 12 actions)

## Change Log

| Date | Change | Author |
|------|--------|--------|
| 2025-11-25 | Story drafted from tech-spec-epic-1.md with learnings from Story 1.1 | SM Agent (Bob) |
| 2025-11-25 | 12 action interfaces created, CMakeLists.txt updated, build verified (21.6s, exit 0), Python imports and C++ headers validated | Dev Agent (Amelia) |
