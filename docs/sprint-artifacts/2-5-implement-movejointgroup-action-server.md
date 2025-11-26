# Story 2.5: Implement MoveJointGroup Action Server

Status: Done

## Story

As a developer,
I want to command groups of joints simultaneously (navigation, gripper, picker, container),
So that I can execute coordinated multi-joint motions efficiently.

## Acceptance Criteria

1. **Action Server (AC1):** MoveJointGroup action server running at `/move_joint_group` accepting goals with `joint_names[]`, `target_positions[]`, `max_velocity`
2. **Joint Group Config (AC2):** Loads predefined joint group definitions from `config/kinematic_chains.yaml`
3. **Validation (AC3):** Validates `target_positions` array length matches joint count and all positions within soft limits
4. **Simultaneous Command (AC4):** Commands all joints simultaneously using ControllerInterface
5. **Aggregate Progress (AC5):** Monitors all joints and computes aggregate progress (average of individual joint progress)
6. **Success Condition (AC6):** Returns success when ALL joints reach targets within 0.01m tolerance or timeout (30s default)
7. **Feedback (AC7):** Publishes feedback at 10 Hz with `current_positions[]` array and `progress_percent`
8. **Predefined Groups (AC8):** Supports named groups: "navigation", "gripper", "picker", "container"
9. **Container Mimic (AC9):** Container group implements software mimic (left = -opening/2, right = +opening/2)
10. **Coordinated Arrival (AC10):** All joints must reach targets within 1 second of each other

## Tasks / Subtasks

- [x] Task 1: Create kinematic_chains.yaml configuration file (AC: 2, 8)
  - [x] 1.1 Create `manipulator_control/config/kinematic_chains.yaml`
  - [x] 1.2 Define "navigation" group: [base_main_frame_joint, main_frame_selector_frame_joint]
  - [x] 1.3 Define "gripper" group: [selector_frame_gripper_joint, main_frame_selector_frame_joint]
  - [x] 1.4 Define "picker" group: [selector_frame_picker_frame_joint, picker_frame_picker_rail_joint, picker_rail_picker_base_joint, picker_base_picker_jaw_joint]
  - [x] 1.5 Define "container" group with mimic_mode: true: [selector_left_container_jaw_joint, selector_right_container_jaw_joint]
  - [x] 1.6 Add default_velocity and description for each group

- [x] Task 2: Implement MoveJointGroupServer node (AC: 1, 2, 3, 4)
  - [x] 2.1 Create `manipulator_control/src/move_joint_group_server.py`
  - [x] 2.2 Initialize with ControllerInterface (reuse from Story 2.2)
  - [x] 2.3 Load joint group definitions from kinematic_chains.yaml
  - [x] 2.4 Create ActionServer for `/move_joint_group` action
  - [x] 2.5 Implement goal_callback with validation (joint count, limits)
  - [x] 2.6 Implement cancel_callback for preemption support

- [x] Task 3: Implement execute_callback with simultaneous motion (AC: 4, 5, 6, 7)
  - [x] 3.1 Command all joints simultaneously using ControllerInterface.command_joint_group()
  - [x] 3.2 Calculate individual joint progress: (traveled / total_distance) * 100
  - [x] 3.3 Calculate aggregate progress: average of all individual progress values
  - [x] 3.4 Publish feedback at 10 Hz with current_positions[] and progress_percent
  - [x] 3.5 Check success condition: ALL joints within 0.01m tolerance
  - [x] 3.6 Handle timeout (30s default from action_servers.yaml)

- [x] Task 4: Implement container jaw mimic mode (AC: 9)
  - [x] 4.1 Detect when joint_group is "container" or contains container jaws
  - [x] 4.2 Accept single "opening" value in target_positions (not two separate values)
  - [x] 4.3 Calculate: left_target = -opening/2, right_target = +opening/2
  - [x] 4.4 Command both jaws with calculated symmetric positions

- [x] Task 5: Implement coordinated arrival check (AC: 10)
  - [x] 5.1 Track completion time for each joint (when it first reaches tolerance)
  - [x] 5.2 Success requires all joints complete within 1 second window
  - [x] 5.3 If coordination fails, log warning but still succeed if all positions reached

- [x] Task 6: Update launch file and package (AC: 1)
  - [x] 6.1 Add move_joint_group_server node to `manipulator_simulation.launch.py`
  - [x] 6.2 Configure as Common node (no condition - runs in both sim and hardware)
  - [x] 6.3 Use 3s delayed start with TimerAction
  - [x] 6.4 Add install rule for kinematic_chains.yaml in CMakeLists.txt
  - [x] 6.5 Build and verify node launches without errors

- [x] Task 7: Manual testing and verification (AC: 1-10)
  - [x] 7.1 Test navigation group: move base_main_frame and selector_frame simultaneously
  - [x] 7.2 Test picker group: move all 4 picker joints together
  - [x] 7.3 Test container group: verify symmetric jaw opening
  - [x] 7.4 Test validation: send wrong array length, verify rejection
  - [x] 7.5 Test validation: send out-of-limits position, verify rejection
  - [x] 7.6 Verify feedback publishes at ~10 Hz during motion
  - [x] 7.7 **MANDATORY: Developer must personally verify coordinated motion in Gazebo/RViz**

## Dev Notes

### Core Implementation Concept

**MoveJointGroup vs MoveJoint:**
- `MoveJoint` = single joint control (Story 2.3)
- `MoveJointGroup` = coordinated multi-joint control with named groups

**Two Usage Modes:**

1. **Named Group Mode:**
   ```
   goal.joint_group = "navigation"  # Uses predefined group from kinematic_chains.yaml
   goal.target_positions = [1.5, 0.8]  # Must match joint count in group
   ```

2. **Explicit Joint Mode:**
   ```
   goal.joint_names = ["base_main_frame_joint", "selector_frame_gripper_joint"]
   goal.target_positions = [1.5, 0.2]
   ```

### Container Jaw Mimic Mode

**Problem:** Container jaws must move symmetrically (left = -x, right = +x) for proper gripping.

**Solution:** When `mimic_mode: true` in kinematic_chains.yaml:
- Accept single "opening width" value
- Calculate: `left = -opening/2`, `right = +opening/2`
- Command both jaws with calculated values

**Example:**
```yaml
container:
  joints:
    - selector_left_container_jaw_joint
    - selector_right_container_jaw_joint
  mimic_mode: true  # Special handling
  default_velocity: 0.1
```
```python
# Goal: open jaws to 0.15m total width
goal.joint_group = "container"
goal.target_positions = [0.15]  # Single value = opening width

# Server calculates:
left_target = -0.075   # -opening/2
right_target = +0.075  # +opening/2
```

### Aggregate Progress Calculation

```python
def calc_aggregate_progress(self, start_positions, current_positions, target_positions):
    """
    Average progress across all joints.

    Individual progress = (traveled / total_distance) * 100
    Aggregate progress = sum(individual_progress) / joint_count
    """
    total_progress = 0.0
    for start, current, target in zip(start_positions, current_positions, target_positions):
        total_distance = abs(target - start)
        if total_distance < 0.001:
            individual_progress = 100.0  # Already at target
        else:
            traveled = abs(current - start)
            individual_progress = min(100.0, (traveled / total_distance) * 100.0)
        total_progress += individual_progress

    return total_progress / len(start_positions)
```

### Coordinated Arrival Window

**Requirement:** All joints must reach targets within 1 second of each other.

**Implementation:**
```python
completion_times = {}  # joint_name -> timestamp when first reached tolerance

for joint_name in joint_names:
    if abs(current - target) <= 0.01:  # Within tolerance
        if joint_name not in completion_times:
            completion_times[joint_name] = time.time()

# Check coordination when all complete
if len(completion_times) == len(joint_names):
    earliest = min(completion_times.values())
    latest = max(completion_times.values())
    coordination_ok = (latest - earliest) <= 1.0
```

### Configuration Files

**kinematic_chains.yaml:**
```yaml
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

### Project Structure Notes

- Follow MoveJointServer pattern from `src/move_joint_server.py`
- Reuse ControllerInterface from Story 2.2
- Reuse action_servers.yaml for shared timeout/tolerance settings

### References

- [Source: docs/epics.md#Story 2.5]
- [Source: docs/sprint-artifacts/tech-spec-epic-2.md#MoveJointGroup Action Server]
- [Source: ros2_ws/src/manipulator_control/src/move_joint_server.py - pattern reference]
- [Source: ros2_ws/src/manipulator_control/src/controller_interface.py - utility to use]
- [Source: ros2_ws/src/manipulator_control/action/MoveJointGroup.action - interface definition]

## Dev Agent Record

### Context Reference

- docs/sprint-artifacts/2-5-implement-movejointgroup-action-server.context.xml

### Agent Model Used

claude-opus-4-5-20251101

### Debug Log References

### Completion Notes List

- AC-1: `/move_joint_group` action server running and accepting goals
- AC-2: Loads 4 joint groups from `kinematic_chains.yaml` on startup
- AC-3: Rejects goals with wrong array length or out-of-limits positions
- AC-4: Commands all joints simultaneously via ControllerInterface.command_joint_group()
- AC-5: Aggregate progress = average of individual joint progress values
- AC-6: Success when all joints within 0.01m tolerance, timeout at 30s
- AC-7: Feedback published at 10Hz with current_positions[] and progress_percent
- AC-8: Supports named groups: navigation, gripper, picker, container
- AC-9: Container mimic mode: opening=0.15 -> left=-0.075, right=+0.075
- AC-10: Coordinated arrival tracking with 1-second window (logs warning if exceeded)

### File List

- `ros2_ws/src/manipulator_control/config/kinematic_chains.yaml` (new)
- `ros2_ws/src/manipulator_control/src/move_joint_group_server.py` (new)
- `ros2_ws/src/manipulator_control/launch/manipulator_simulation.launch.py` (modified)
- `ros2_ws/src/manipulator_control/CMakeLists.txt` (modified)

