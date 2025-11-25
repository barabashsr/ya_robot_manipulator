# Story 1.2: Define All Action Interfaces

Status: drafted

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

- [ ] Task 1: Create Low-Level Action Definitions (AC: 1, 2, 7)
  - [ ] 1.1 Create `MoveJoint.action` - single joint position control
  - [ ] 1.2 Create `MoveJointGroup.action` - multiple joints synchronized

- [ ] Task 2: Create Mid-Level Navigation Actions (AC: 1, 2, 7)
  - [ ] 2.1 Create `NavigateToAddress.action` - move to warehouse address
  - [ ] 2.2 Create `ExtractBox.action` - extract box from storage
  - [ ] 2.3 Create `ReturnBox.action` - return held box to original address
  - [ ] 2.4 Create `PutBox.action` - place box at target address
  - [ ] 2.5 Create `MoveBoxToLoad.action` - extract, move to load, optionally relocate

- [ ] Task 3: Create Container/Picker Actions (AC: 1, 2, 7)
  - [ ] 3.1 Create `ManipulateContainer.action` - open/close container jaws
  - [ ] 3.2 Create `GetContainer.action` - retrieve container from storage
  - [ ] 3.3 Create `PlaceContainer.action` - place container back
  - [ ] 3.4 Create `PickItem.action` - pick item from department using limit switches

- [ ] Task 4: Create High-Level Composite Action (AC: 1, 2, 7)
  - [ ] 4.1 Create `PickItemFromStorage.action` - complete workflow: get container, navigate, extract, pick, return

- [ ] Task 5: Update CMakeLists.txt for Interface Generation (AC: 3)
  - [ ] 5.1 Uncomment/add rosidl_generate_interfaces block
  - [ ] 5.2 List all 12 action files
  - [ ] 5.3 Add DEPENDENCIES for geometry_msgs (used by NavigateToAddress)

- [ ] Task 6: Validate Build and Interface Generation (AC: 4, 5, 6)
  - [ ] 6.1 Run `colcon build --packages-select manipulator_control`
  - [ ] 6.2 Verify exit code 0, no errors, no warnings
  - [ ] 6.3 Test Python import: `python3 -c "from manipulator_control.action import MoveJoint"`
  - [ ] 6.4 Verify C++ headers exist in install/manipulator_control/include/

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

### References

- [Source: docs/sprint-artifacts/tech-spec-epic-1.md#Acceptance-Criteria] AC-3 action interface criteria
- [Source: docs/sprint-artifacts/tech-spec-epic-1.md#Test-Strategy] Interface generation and import tests

## Dev Agent Record

### Context Reference

<!-- Path(s) to story context XML will be added here by context workflow -->

### Agent Model Used

### Debug Log References

### Completion Notes List

### File List

## Change Log

| Date | Change | Author |
|------|--------|--------|
| 2025-11-25 | Story drafted from tech-spec-epic-1.md with learnings from Story 1.1 | SM Agent (Bob) |
