# Story 4A-1a: End Effector Trajectory Visualization (Hotfix)

Status: **DONE**

## Story

As a developer,
I want trajectory markers to be visualized in the address frame with correct coordinate transformations,
So that I can see the planned path where the end effector will travel relative to the cabinet slot during box extraction/insertion.

## Core Concepts

### What This Story Fixes

**Story 4A-1a is a hotfix to Story 4A-1** that addresses multiple trajectory visualization and execution issues:

1. **Marker Coordinate System** - Fixed joint-to-address-frame coordinate conversion for proper marker placement
2. **Insertion Trajectory Shape** - Fixed Z-axis inversion so insertion curves UP like extraction (same physical path)
3. **Full Box Operation Sequence** - Created comprehensive test script with all 5 phases (NAVIGATE, APPROACH, EXTRACT, INSERT, RETRACT)
4. **Configuration-Driven Values** - Magnet offset and cabinet depth loaded from YAML configs, not hardcoded

### Frame and Coordinate System

The address frame has identity rotation (aligned with world axes):
- **Origin**: At the box location (inside cabinet, at cabinet_depth from robot center)
- **Y axis**: Same as world Y (positive toward left cabinet)
- **Z axis**: Vertical (same as world Z)

Coordinate transformations for markers in address frame:
```
For LEFT cabinet:
- Robot center is at world Y=0
- Box/address origin is at world Y=cabinet_depth (0.4m)
- Gripper extends in +Y direction to reach box

In address frame local coordinates:
- Y=0 means at the address origin (at the box)
- Y=-cabinet_depth means at robot center (outside cabinet)
- Extraction: starts at Y=0 (at box), ends at Y=-0.4 (outside)
- Insertion: starts at Y=-0.4 (outside), ends at Y=0 (at box)
```

### Full Box Operation Sequence (5 Phases)

The complete workflow for box extraction/insertion:

```
┌─────────────────────────────────────────────────────────────┐
│  PHASE 1: NAVIGATE                                          │
│  - Position robot at X-Z coordinates for target address     │
│  - Uses NavigateToAddress action                            │
│  - Gripper starts at Y=0 (retracted center position)        │
└────────────────────────────┬────────────────────────────────┘
                             ▼
┌─────────────────────────────────────────────────────────────┐
│  PHASE 2: APPROACH                                          │
│  - Straight line Y movement INTO cabinet                    │
│  - Moves gripper joint to Y=cabinet_depth - magnet_offset   │
│  - For left: 0.0 → 0.36m (magnet reaches 0.4m at address)   │
│  - Aligns magnet origin with address origin                 │
└────────────────────────────┬────────────────────────────────┘
                             ▼
┌─────────────────────────────────────────────────────────────┐
│  PHASE 3: EXTRACT                                           │
│  - Curved trajectory to pull box OUT of cabinet             │
│  - Joint Y: 0.36 → -0.04 (20 waypoints)                     │
│  - Curves UP during extraction for clearance                │
│  - End: magnet at Y=0 (outside cabinet)                     │
└────────────────────────────┬────────────────────────────────┘
                             ▼
┌─────────────────────────────────────────────────────────────┐
│  PHASE 4: INSERT                                            │
│  - Curved trajectory to push box back INTO cabinet          │
│  - Joint Y: -0.04 → 0.36 (20 waypoints)                     │
│  - Curves UP (same path shape as extraction, reversed)      │
│  - End: magnet at Y=0.4 (at address origin)                 │
└────────────────────────────┬────────────────────────────────┘
                             ▼
┌─────────────────────────────────────────────────────────────┐
│  PHASE 5: RETRACT                                           │
│  - Straight line Y movement back to center                  │
│  - Moves selector_frame_gripper_joint to Y=0                │
│  - Gripper returns to safe navigation position              │
└─────────────────────────────────────────────────────────────┘
```

### Key Concept: End Effector vs Joint Position

The magnet (end effector) is **0.040m ahead** of the gripper joint in Y direction:

```
Magnet Offset Geometry:

    ┌─────────────────┐
    │  Gripper Joint  │ ← selector_frame_gripper_joint (Y position)
    │   (Y = 0.36)    │
    └────────┬────────┘
             │
             │ 0.040m (magnet_offset)
             │
             ▼
    ┌─────────────────┐
    │   Magnet Tip    │ ← left_gripper_magnet frame (Y = 0.40)
    │ (End Effector)  │
    └─────────────────┘

Conversion formulas:
- end_effector_Y = joint_Y + magnet_offset
- joint_Y = end_effector_Y - magnet_offset

Example for LEFT cabinet:
- To get magnet at Y=0.4 (at address): joint_Y = 0.4 - 0.04 = 0.36
- To get magnet at Y=0 (outside): joint_Y = 0 - 0.04 = -0.04
```

### Kinematic Chain Context

```
world
└── odom
    └── base (robot rail base)
        └── main_frame
            └── selector_frame
                └── gripper (selector_frame_gripper_joint - Y axis)
                    ├── left_gripper_magnet  ← End effector for LEFT cabinets
                    └── right_gripper_magnet ← End effector for RIGHT cabinets
```

The trajectory controls:
- `selector_frame_gripper_joint` (Y-axis into cabinet)
- `main_frame_selector_frame_joint` (Z-axis for clearance curve)

## Acceptance Criteria

1. **AC1 - Address Frame Markers:** Given an address frame (e.g., `addr_l_1_5_2`), when `publish_trajectory_markers()` is called, then markers are published in that address frame, showing the stationary target path where the end effector will travel. Falls back to `odom` frame if no address specified.

2. **AC2 - Trajectory Path Visualization:** Markers show the planned path the end effector will follow, with:
   - Green sphere at start position
   - Cyan spheres for intermediate waypoints
   - Red sphere at end position
   - Line strip connecting all waypoints
   - Arrow showing direction (start to end)

3. **AC3 - Joint-to-Address Coordinate Conversion:** Waypoint joint positions are converted to address frame coordinates using: `address_Y = (joint_Y * sign + magnet_offset) - cabinet_depth`. This shows where the magnet tip will be relative to the box location.

4. **AC4 - Test Script Update:** `test_box_operation_sequence.py` provides full 5-phase workflow with marker visualization in the target address frame.

5. **AC5 - Gazebo Validation:** Markers visible in RViz as stationary path at the target address location, clearly showing where the end effector will travel during box operations.

## Tasks / Subtasks

- [x] Task 1: Update `publish_trajectory_markers()` coordinate conversion (AC1, AC3)
  - [x] 1.1 Use `address_frame` parameter for `frame_id` (falls back to `odom` if not provided)
  - [x] 1.2 Implement `joint_to_address_y()` conversion: `address_Y = (joint_Y * sign + magnet_offset) - cabinet_depth`
  - [x] 1.3 Load `cabinet_depth` from trajectory_config.yaml and `magnet_offset` from kinematic_chains.yaml

- [x] Task 2: Create comprehensive test script (AC4)
  - [x] 2.1 Create `test_box_operation_sequence.py` with full 5-phase workflow
  - [x] 2.2 Implement phases: NAVIGATE, APPROACH, EXTRACT, INSERT, RETRACT
  - [x] 2.3 Pass address frame to marker publishing for stationary path visualization

- [x] Task 3: Validate in simulation (AC2, AC5)
  - [x] 3.1 Launch simulation and run test script with `--side left --cabinet 1 --row 5 --col 2`
  - [x] 3.2 Verify markers appear at address location (stationary in world space)
  - [x] 3.3 Confirm extraction markers: Y=0 (at box) to Y=-0.4 (outside cabinet)
  - [x] 3.4 Confirm insertion markers: Y=-0.4 (outside) to Y=0 (at box)

- [x] Task 4: Update documentation
  - [x] 4.1 Update docstrings in `yz_trajectory_generator.py` with address frame conversion math
  - [x] 4.2 Update test script docstring with coordinate system explanation

## Dev Notes

### Configuration Files

**kinematic_chains.yaml** - Added `magnet_offset` for end effector position calculation:
```yaml
gripper:
  joints:
    - selector_frame_gripper_joint
    - main_frame_selector_frame_joint
  end_effector_frames:
    left: "left_gripper_magnet"
    right: "right_gripper_magnet"
  magnet_offset: 0.040  # meters (40mm) - magnet tip ahead of joint
  coordinate_mapping:
    selector_frame_gripper_joint:
      axis: "y"       # Into/out of cabinet
    main_frame_selector_frame_joint:
      axis: "z"       # Vertical position
```

**trajectory_config.yaml** - Provides `cabinet_depth` via `y_output` range:
```yaml
trajectories:
  extract_left:
    mapping:
      y_output: [0.0, 0.4]  # cabinet_depth = max(y_output) = 0.4m
```

### Marker Coordinate Transformation

The `publish_trajectory_markers()` function converts joint positions to address frame coordinates:

```python
def joint_to_address_y(joint_y: float) -> float:
    """Convert joint Y position to address frame Y position."""
    # End effector position in world Y
    ee_world_y = sign * joint_y + magnet_offset
    # Address frame Y = ee_world_Y - cabinet_depth
    return ee_world_y - cabinet_depth
```

**Example for extraction (left side):**
- Joint goes: 0.36 → -0.04
- First waypoint: ee_Y = 0.36 + 0.04 = 0.4, addr_Y = 0.4 - 0.4 = 0 (at box)
- Last waypoint: ee_Y = -0.04 + 0.04 = 0, addr_Y = 0 - 0.4 = -0.4 (outside)
- Markers: Y=0 (green start at box) → Y=-0.4 (red end outside)

**Example for insertion (left side):**
- Joint goes: -0.04 → 0.36
- First waypoint: ee_Y = -0.04 + 0.04 = 0, addr_Y = 0 - 0.4 = -0.4 (outside)
- Last waypoint: ee_Y = 0.36 + 0.04 = 0.4, addr_Y = 0.4 - 0.4 = 0 (at box)
- Markers: Y=-0.4 (green start outside) → Y=0 (red end at box)

### Insertion Z-Axis Fix

The raw YAML has insertion Z values inverted (negative) compared to extraction (positive):
```yaml
extraction: z goes 0 → +0.0248 → 0  (curves UP)
insertion:  z goes 0 → -0.0149 → 0  (curves DOWN - WRONG!)
```

Fixed in `load_trajectory()` by negating Z for insertion:
```python
z_sign = -1.0 if name == 'insertion' else 1.0
z=base_z + z_sign * wp['z']
```

Now both trajectories follow the same physical path shape (curve UP).

### Test Script Usage

```bash
# Full sequence with execution
python3 scripts/test_box_operation_sequence.py --side left --cabinet 1 --row 5 --col 2

# Visualization only (no movement)
python3 scripts/test_box_operation_sequence.py --side left --cabinet 1 --row 5 --col 2 --no-execute

# Step through phases manually
python3 scripts/test_box_operation_sequence.py --side left --cabinet 1 --row 5 --col 2 --pause
```

### Dependencies

- Story 4A-1 (completed) - Provides base YZTrajectoryGenerator implementation
- NavigateToAddress action server (Story 3.4)
- TF tree with `left_gripper_magnet` and `right_gripper_magnet` frames
- FollowJointTrajectory controllers for Y and Z joints

### References

- [Source: docs/sprint-artifacts/4a-1-implement-yz-trajectory-generator-utility.md]
- [Source: ros2_ws/src/manipulator_control/config/kinematic_chains.yaml]
- [Source: ros2_ws/src/manipulator_control/config/trajectory_config.yaml]
- [Source: ros2_ws/src/manipulator_control/src/utils/yz_trajectory_generator.py]

## Change Log

| Date | Change | Author |
|------|--------|--------|
| 2025-12-04 | Initial draft | SM Agent (Bob) |
| 2025-12-04 | Implementation complete - all fixes applied | Dev Agent (Claude) |
| 2025-12-05 | Updated AC1, AC3-5 and Tasks to match actual implementation (address frame markers, not end effector frame) | SM Agent (Bob) |

## Dev Agent Record

### Context Reference

- Parent story: `docs/sprint-artifacts/4a-1-implement-yz-trajectory-generator-utility.md`
- Story context: `docs/sprint-artifacts/4a-1a-end-effector-trajectory-visualization.context.xml`

### Agent Model Used

- Claude Opus 4.5 (claude-opus-4-5-20250114)

### Completion Notes List

1. **Fixed marker coordinate transformation** - Implemented proper joint-to-address-frame conversion in `publish_trajectory_markers()`. Markers now correctly show trajectory path in address frame with Y=0 at box location and Y=-0.4 outside cabinet.

2. **Fixed insertion trajectory Z-axis** - Added `z_sign` multiplier in `load_trajectory()` to negate Z values for insertion. Both extraction and insertion now follow the same curved path shape (UP), just in opposite Y directions.

3. **Created full box operation test script** - New `test_box_operation_sequence.py` implements all 5 phases:
   - NAVIGATE: Position robot at address X-Z
   - APPROACH: Straight line into cabinet (aligns magnet with address origin)
   - EXTRACT: Curved trajectory pulling box out
   - INSERT: Curved trajectory pushing box back
   - RETRACT: Return gripper joint to Y=0

4. **Configuration-driven parameters** - Script loads `magnet_offset` from `kinematic_chains.yaml` and `cabinet_depth` from `trajectory_config.yaml` instead of hardcoded values.

5. **Proper RETRACT implementation** - Moves `selector_frame_gripper_joint` directly to Y=0, not the end effector to joint origin (no overshoot).

### Test Results Summary

**Test Command:**
```bash
python3 scripts/test_box_operation_sequence.py --side left --cabinet 1 --row 5 --col 2
```

**Results:**
```
PHASE 1: NAVIGATE - Position at address X-Z
  Navigation complete in 0.03s, error=0.0000m

PHASE 2: APPROACH - Straight line into cabinet
  Magnet target: Y=0.400, Joint target: Y=0.360
  Moving gripper joint Y: 0.000 -> 0.360

PHASE 3: EXTRACT - Pull box out of cabinet
  Loaded 20 extraction waypoints
  Joint Y: 0.360 -> -0.040
  Extraction trajectory completed!

PHASE 4: INSERT - Push box back into cabinet
  Loaded 20 insertion waypoints
  Joint Y: -0.040 -> 0.360
  Insertion trajectory completed!

PHASE 5: RETRACT - Return gripper to center
  Moving gripper joint Y: 0.359 -> 0.000

BOX OPERATION SEQUENCE COMPLETE
```

**Marker Visualization:**
- Extraction markers: Start at address origin (Y=0), end outside (Y=-0.4), curve UP
- Insertion markers: Start outside (Y=-0.4), end at address origin (Y=0), curve UP
- Both trajectories display correctly in RViz with address frame

### File List

| File | Purpose |
|------|---------|
| `src/utils/yz_trajectory_generator.py` | Fixed `publish_trajectory_markers()` coordinate conversion; Added `z_sign` for insertion Z-axis fix |
| `scripts/test_box_operation_sequence.py` | NEW: Full 5-phase box operation test script with visualization |
| `config/kinematic_chains.yaml` | Added `magnet_offset: 0.040` configuration |
