# Story 2.3.1: Migrate Motion Joints to JointTrajectoryController

Status: ready-for-dev

## Story

As a **developer**,
I want **motion joints to use JointTrajectoryController with action-based interface**,
So that **we have smooth trajectory interpolation, feedback during motion, and MoveIt2 compatibility**.

## Acceptance Criteria

1. **AC-1:** 7 motion joints use `JointTrajectoryController` (action-based)
2. **AC-2:** 2 container jaw joints remain `ForwardCommandController` (topic-based)
3. **AC-3:** `manipulator_controllers.yaml` updated with new controller types and parameters
4. **AC-4:** ControllerInterface supports dual-mode operation (action + topic routing)
5. **AC-5:** Same `command_joint()` API works for all joints (abstraction preserved)
6. **AC-6:** New `wait_for_action_servers()` method for startup synchronization
7. **AC-7:** New `cancel_trajectory()` method for preemption support
8. **AC-8:** `ros2 control list_controllers` shows correct controller types
9. **AC-9:** All 7 trajectory actions available via `ros2 action list`
10. **AC-10:** MoveJoint action server continues to work unchanged (regression test)
11. **AC-11:** All existing Story 2.3 tests pass with new controller architecture

## Key Concepts & Core Logic

### Why Migrate to JointTrajectoryController?

| Issue with ForwardCommand | Solution with Trajectory |
|--------------------------|--------------------------|
| Instant position jumps (jerky) | Smooth spline interpolation |
| No feedback during motion | Action feedback with progress |
| No velocity profile | Duration-based trajectory execution |
| Abrupt stops | Smooth deceleration to target |
| Not MoveIt2 compatible | Native MoveIt2 interface |

### Hybrid Controller Architecture

After migration, the system uses TWO controller types:

```
┌─────────────────────────────────────────────────────────────────────────┐
│                    HYBRID CONTROLLER ARCHITECTURE                        │
├─────────────────────────────────────────────────────────────────────────┤
│                                                                          │
│  JointTrajectoryController (7 motion joints)                            │
│  ─────────────────────────────────────────────                          │
│  • base_main_frame_joint_controller                                     │
│  • main_frame_selector_frame_joint_controller                           │
│  • selector_frame_gripper_joint_controller                              │
│  • selector_frame_picker_frame_joint_controller                         │
│  • picker_frame_picker_rail_joint_controller                            │
│  • picker_rail_picker_base_joint_controller                             │
│  • picker_base_picker_jaw_joint_controller                              │
│                                                                          │
│  Interface: /{controller}/follow_joint_trajectory (Action)              │
│  Message: control_msgs/action/FollowJointTrajectory                     │
│  Features: Smooth interpolation, feedback, preemption                   │
│                                                                          │
├─────────────────────────────────────────────────────────────────────────┤
│                                                                          │
│  ForwardCommandController (2 container jaws)                            │
│  ─────────────────────────────────────────────                          │
│  • selector_left_container_jaw_joint_controller                         │
│  • selector_right_container_jaw_joint_controller                        │
│                                                                          │
│  Interface: /{controller}/commands (Topic)                              │
│  Message: std_msgs/Float64MultiArray                                    │
│  Features: Instant response, simple open/close                          │
│                                                                          │
└─────────────────────────────────────────────────────────────────────────┘
```

### Joint Classification

```python
# Motion joints use trajectory controller (action clients)
TRAJECTORY_JOINTS = frozenset([
    'base_main_frame_joint',
    'main_frame_selector_frame_joint',
    'selector_frame_gripper_joint',
    'selector_frame_picker_frame_joint',
    'picker_frame_picker_rail_joint',
    'picker_rail_picker_base_joint',
    'picker_base_picker_jaw_joint'
])

# Container jaws use forward command controller (topic publishers)
FORWARD_COMMAND_JOINTS = frozenset([
    'selector_left_container_jaw_joint',
    'selector_right_container_jaw_joint'
])
```

### Controller YAML Configuration Changes

**BEFORE (ForwardCommandController):**
```yaml
base_main_frame_joint_controller:
  ros__parameters:
    joints:
      - base_main_frame_joint
    interface_name: position
```

**AFTER (JointTrajectoryController):**
```yaml
base_main_frame_joint_controller:
  ros__parameters:
    joints:
      - base_main_frame_joint
    command_interfaces:
      - position
    state_interfaces:
      - position
      - velocity
    action_monitor_rate: 20.0
    allow_partial_joints_goal: false
    allow_nonzero_velocity_at_trajectory_end: false
    interpolation_method: splines
    constraints:
      stopped_velocity_tolerance: 0.01
      goal_time: 0.0
      base_main_frame_joint:
        trajectory: 0.1
        goal: 0.01
```

### ControllerInterface Dual-Mode Architecture

```
┌─────────────────────────────────────────────────────────────────────────┐
│                      ControllerInterface (Updated)                       │
├─────────────────────────────────────────────────────────────────────────┤
│                                                                          │
│  command_joint(joint_name, position, duration_sec=None)                 │
│      │                                                                   │
│      ├─── if joint in TRAJECTORY_JOINTS ───► ActionClient               │
│      │         └─► send_goal_async(FollowJointTrajectory)               │
│      │                                                                   │
│      └─── if joint in FORWARD_COMMAND_JOINTS ───► Publisher             │
│                └─► publish(Float64MultiArray)                            │
│                                                                          │
│  NEW Methods:                                                            │
│  • wait_for_action_servers(timeout_sec=30.0) - Block until all ready    │
│  • cancel_trajectory(joint_name) - Cancel active goal                   │
│  • command_trajectory_with_callback(...) - Async with callbacks         │
│                                                                          │
└─────────────────────────────────────────────────────────────────────────┘
```

### Duration Calculation

For trajectory joints, duration is calculated from distance and max velocity:

```python
def _calculate_duration(self, joint_name: str, current: float, target: float) -> float:
    """Calculate trajectory duration based on distance and max velocity."""
    distance = abs(target - current)
    max_velocity = self.joint_limits[joint_name]['velocity']  # From manipulator_params.yaml

    duration = distance / max_velocity if max_velocity > 0 else 1.0
    duration = max(0.1, min(10.0, duration))  # Clamp to [0.1, 10.0] seconds

    return duration
```

### Trajectory Message Construction

```python
def _build_trajectory_goal(self, joint_name: str, target: float, duration: float):
    """Build FollowJointTrajectory goal message."""
    from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
    from control_msgs.action import FollowJointTrajectory
    from builtin_interfaces.msg import Duration

    trajectory = JointTrajectory()
    trajectory.joint_names = [joint_name]

    point = JointTrajectoryPoint()
    point.positions = [target]
    point.velocities = [0.0]  # Stop at target
    point.time_from_start = Duration(
        sec=int(duration),
        nanosec=int((duration % 1) * 1e9)
    )
    trajectory.points = [point]

    goal = FollowJointTrajectory.Goal()
    goal.trajectory = trajectory

    return goal
```

## Tasks / Subtasks

- [ ] Task 1: Update manipulator_controllers.yaml (AC: 1, 2, 3)
  - [ ] 1.1 Change 7 motion joint controllers to `joint_trajectory_controller/JointTrajectoryController`
  - [ ] 1.2 Add trajectory controller parameters (interpolation, constraints, tolerances)
  - [ ] 1.3 Keep 2 container jaw controllers as `forward_command_controller/ForwardCommandController`
  - [ ] 1.4 Verify YAML syntax with `ros2 param dump`

- [ ] Task 2: Update ControllerInterface for Dual-Mode (AC: 4, 5, 6, 7)
  - [ ] 2.1 Add `TRAJECTORY_JOINTS` and `FORWARD_COMMAND_JOINTS` constants
  - [ ] 2.2 Create ActionClients for 7 trajectory joints in `__init__()`
  - [ ] 2.3 Keep Publishers for 2 forward command joints
  - [ ] 2.4 Update `command_joint()` to route based on joint type
  - [ ] 2.5 Add `wait_for_action_servers()` method
  - [ ] 2.6 Add `cancel_trajectory()` method
  - [ ] 2.7 Add `command_trajectory_with_callback()` method
  - [ ] 2.8 Add `_calculate_duration()` helper
  - [ ] 2.9 Add `_build_trajectory_goal()` helper

- [ ] Task 3: Update Package Dependencies (AC: 4)
  - [ ] 3.1 Add `control_msgs` to package.xml
  - [ ] 3.2 Verify CMakeLists.txt dependencies

- [ ] Task 4: Developer Self-Validation - MANDATORY (AC: 8, 9, 10, 11)
  - [ ] 4.1 Execute Phase 1: Build Verification
  - [ ] 4.2 Execute Phase 2: Controller Type Verification
  - [ ] 4.3 Execute Phase 3: Action Interface Verification
  - [ ] 4.4 Execute Phase 4: Topic Verification
  - [ ] 4.5 Execute Phase 5: Trajectory Joint Motion Tests (all 7 joints)
  - [ ] 4.6 Execute Phase 6: Forward Command Joint Tests (both jaws)
  - [ ] 4.7 Execute Phase 7: MoveJoint Action Server Regression
  - [ ] 4.8 Execute Phase 8: Visual Verification in Gazebo
  - [ ] 4.9 Document ALL test results in Dev Agent Record

## Dev Notes

### Learnings from Previous Stories

**From Story 2.2 - ControllerInterface (Status: done):**
- ControllerInterface is NOT a node - requires parent node reference
- Joint limits loaded from `manipulator_params.yaml` including `velocity`
- Uses `/{joint}_controller/commands` pattern for ForwardCommand

**From Story 2.3 - MoveJoint Action Server (Status: done):**
- MoveJoint uses ControllerInterface.command_joint() internally
- Position tolerance: 0.01m
- Timeout: 30s default
- PREEMPT policy for new goals

### Configuration Reuse Policy

**MODIFY (existing files):**
- `manipulator_description/config/manipulator_controllers.yaml` - controller types
- `manipulator_control/src/controller_interface.py` - dual-mode support

**USE (no changes):**
- `manipulator_params.yaml` - joint limits and velocities (single source of truth)

**ADD (to existing file):**
- `manipulator_control/package.xml` - add `control_msgs` dependency

### Project Structure

```
ros2_ws/src/
├── manipulator_description/
│   └── config/
│       └── manipulator_controllers.yaml    # MODIFY - controller types
│
└── manipulator_control/
    ├── src/
    │   └── controller_interface.py         # MODIFY - dual-mode support
    ├── test/
    │   └── test_story_2_3_1_trajectory_controllers.py  # NEW - comprehensive test
    └── package.xml                         # MODIFY - add control_msgs
```

### References

- [Source: docs/migration-forward-to-trajectory-controllers.md] Complete migration plan
- [Source: docs/architecture-ros2-control-v2-CORRECTIONS.md#Section-1] Hybrid controller architecture
- [Source: docs/epics.md#Story-2.3.1] Acceptance criteria
- [Source: docs/sprint-artifacts/2-2-create-controller-interface-utility.md] ControllerInterface API
- [Source: docs/sprint-artifacts/2-3-implement-movejoint-action-server.md] MoveJoint implementation

## Test Requirements (MANDATORY)

**CRITICAL:** Developer MUST execute ALL tests below and document results before marking story complete. Tests are organized by joint groups for parallel execution where applicable.

### Joint Groups for Parallel Testing

| Group | Joints | Can Test In Parallel |
|-------|--------|---------------------|
| Navigation | base_main_frame_joint, main_frame_selector_frame_joint | Yes |
| Gripper | selector_frame_gripper_joint | Independent |
| Picker | selector_frame_picker_frame_joint, picker_frame_picker_rail_joint, picker_rail_picker_base_joint, picker_base_picker_jaw_joint | Yes |
| Container | selector_left_container_jaw_joint, selector_right_container_jaw_joint | Yes |

---

### Phase 1: Build Verification

```bash
# Kill any existing simulation processes first
pkill -9 gazebo || true
pkill -9 gzserver || true
pkill -9 gzclient || true
pkill -9 rviz2 || true

cd /home/robo/robo/ya_robot_manipulator/ros2_ws
colcon build --packages-select manipulator_description manipulator_control
echo "Build exit code: $?"
# Expected: 0
```

**Record Result:**
- [ ] Build successful (exit code 0)
- [ ] No errors or warnings

---

### Phase 2: Controller Type Verification

```bash
source /home/robo/robo/ya_robot_manipulator/ros2_ws/install/setup.bash
ros2 launch manipulator_description manipulator_control.launch.py &

# Wait for controllers to load
sleep 15

# Verify controller types
ros2 control list_controllers
```

**Expected Output (verify EACH line):**

| Controller | Expected Type | Status |
|------------|---------------|--------|
| joint_state_broadcaster | joint_state_broadcaster/JointStateBroadcaster | active |
| base_main_frame_joint_controller | **joint_trajectory_controller/JointTrajectoryController** | active |
| main_frame_selector_frame_joint_controller | **joint_trajectory_controller/JointTrajectoryController** | active |
| selector_frame_gripper_joint_controller | **joint_trajectory_controller/JointTrajectoryController** | active |
| selector_frame_picker_frame_joint_controller | **joint_trajectory_controller/JointTrajectoryController** | active |
| picker_frame_picker_rail_joint_controller | **joint_trajectory_controller/JointTrajectoryController** | active |
| picker_rail_picker_base_joint_controller | **joint_trajectory_controller/JointTrajectoryController** | active |
| picker_base_picker_jaw_joint_controller | **joint_trajectory_controller/JointTrajectoryController** | active |
| selector_left_container_jaw_joint_controller | **forward_command_controller/ForwardCommandController** | active |
| selector_right_container_jaw_joint_controller | **forward_command_controller/ForwardCommandController** | active |

**Record Result:**
- [ ] 7 trajectory controllers active
- [ ] 2 forward command controllers active
- [ ] All controller names correct

---

### Phase 3: Action Interface Verification

```bash
ros2 action list | grep follow_joint_trajectory
```

**Expected Output (all 7 actions):**
```
/base_main_frame_joint_controller/follow_joint_trajectory
/main_frame_selector_frame_joint_controller/follow_joint_trajectory
/selector_frame_gripper_joint_controller/follow_joint_trajectory
/selector_frame_picker_frame_joint_controller/follow_joint_trajectory
/picker_frame_picker_rail_joint_controller/follow_joint_trajectory
/picker_rail_picker_base_joint_controller/follow_joint_trajectory
/picker_base_picker_jaw_joint_controller/follow_joint_trajectory
```

**Record Result:**
- [ ] All 7 trajectory actions available

---

### Phase 4: Topic Verification (Container Jaws Only)

```bash
ros2 topic list | grep "_controller/commands"
```

**Expected Output (only 2 topics):**
```
/selector_left_container_jaw_joint_controller/commands
/selector_right_container_jaw_joint_controller/commands
```

**Verify NO trajectory joint command topics exist:**
```bash
ros2 topic list | grep "base_main_frame.*commands"
# Expected: NO OUTPUT (topic removed)
```

**Record Result:**
- [ ] Only 2 command topics (container jaws)
- [ ] Motion joint command topics removed

---

### Phase 5: Trajectory Joint Motion Tests (All 7 Joints)

**Test each trajectory joint with action goal. Run navigation and picker groups in parallel.**

#### 5.1 Navigation Group (Parallel)

**Terminal A - Base Joint:**
```bash
ros2 action send_goal /base_main_frame_joint_controller/follow_joint_trajectory \
  control_msgs/action/FollowJointTrajectory \
  "{trajectory: {joint_names: ['base_main_frame_joint'], points: [{positions: [2.0], velocities: [0.0], time_from_start: {sec: 3, nanosec: 0}}]}}" \
  --feedback
```

**Terminal B - Selector Frame Joint (same time):**
```bash
ros2 action send_goal /main_frame_selector_frame_joint_controller/follow_joint_trajectory \
  control_msgs/action/FollowJointTrajectory \
  "{trajectory: {joint_names: ['main_frame_selector_frame_joint'], points: [{positions: [0.8], velocities: [0.0], time_from_start: {sec: 2, nanosec: 0}}]}}" \
  --feedback
```

**Expected for BOTH:**
- Feedback messages during motion
- Result: `error_code: 0` (SUCCESSFUL)

**Record Result:**
- [ ] base_main_frame_joint: reached 2.0m, smooth motion
- [ ] main_frame_selector_frame_joint: reached 0.8m, smooth motion

#### 5.2 Gripper Joint

```bash
ros2 action send_goal /selector_frame_gripper_joint_controller/follow_joint_trajectory \
  control_msgs/action/FollowJointTrajectory \
  "{trajectory: {joint_names: ['selector_frame_gripper_joint'], points: [{positions: [0.2], velocities: [0.0], time_from_start: {sec: 1, nanosec: 0}}]}}" \
  --feedback
```

**Record Result:**
- [ ] selector_frame_gripper_joint: reached 0.2m, smooth motion

#### 5.3 Picker Group (Parallel - 4 joints)

**Terminal A:**
```bash
ros2 action send_goal /selector_frame_picker_frame_joint_controller/follow_joint_trajectory \
  control_msgs/action/FollowJointTrajectory \
  "{trajectory: {joint_names: ['selector_frame_picker_frame_joint'], points: [{positions: [0.15], velocities: [0.0], time_from_start: {sec: 1, nanosec: 0}}]}}" \
  --feedback
```

**Terminal B:**
```bash
ros2 action send_goal /picker_frame_picker_rail_joint_controller/follow_joint_trajectory \
  control_msgs/action/FollowJointTrajectory \
  "{trajectory: {joint_names: ['picker_frame_picker_rail_joint'], points: [{positions: [0.15], velocities: [0.0], time_from_start: {sec: 1, nanosec: 0}}]}}" \
  --feedback
```

**Terminal C:**
```bash
ros2 action send_goal /picker_rail_picker_base_joint_controller/follow_joint_trajectory \
  control_msgs/action/FollowJointTrajectory \
  "{trajectory: {joint_names: ['picker_rail_picker_base_joint'], points: [{positions: [0.12], velocities: [0.0], time_from_start: {sec: 1, nanosec: 0}}]}}" \
  --feedback
```

**Terminal D:**
```bash
ros2 action send_goal /picker_base_picker_jaw_joint_controller/follow_joint_trajectory \
  control_msgs/action/FollowJointTrajectory \
  "{trajectory: {joint_names: ['picker_base_picker_jaw_joint'], points: [{positions: [0.1], velocities: [0.0], time_from_start: {sec: 1, nanosec: 0}}]}}" \
  --feedback
```

**Record Result:**
- [ ] selector_frame_picker_frame_joint: reached 0.15m, smooth motion
- [ ] picker_frame_picker_rail_joint: reached 0.15m, smooth motion
- [ ] picker_rail_picker_base_joint: reached 0.12m, smooth motion
- [ ] picker_base_picker_jaw_joint: reached 0.1m, smooth motion

---

### Phase 6: Forward Command Joint Tests (Both Container Jaws)

**Test both jaws in parallel:**

**Terminal A - Left Jaw:**
```bash
ros2 topic pub --once /selector_left_container_jaw_joint_controller/commands \
  std_msgs/msg/Float64MultiArray "{data: [0.1]}"
```

**Terminal B - Right Jaw:**
```bash
ros2 topic pub --once /selector_right_container_jaw_joint_controller/commands \
  std_msgs/msg/Float64MultiArray "{data: [0.1]}"
```

**Verify positions:**
```bash
ros2 topic echo /joint_states --once | grep -A 11 "name:"
```

**Record Result:**
- [ ] selector_left_container_jaw_joint: moved to ~0.1m
- [ ] selector_right_container_jaw_joint: moved to ~0.1m
- [ ] Instant response (no interpolation delay)

---

### Phase 7: MoveJoint Action Server Regression (All 9 Joints)

**This is the critical regression test - MoveJoint must work unchanged.**

**Launch full simulation with MoveJoint server:**
```bash
pkill -9 gazebo || true
sleep 2
source /home/robo/robo/ya_robot_manipulator/ros2_ws/install/setup.bash
ros2 launch manipulator_control manipulator_simulation.launch.py &
sleep 15
```

**Test all 9 joints via MoveJoint action:**

```bash
#!/bin/bash
# Save as /tmp/test_movejoint_all_joints.sh

source /home/robo/robo/ya_robot_manipulator/ros2_ws/install/setup.bash

echo "=== MoveJoint Regression Test (All 9 Joints) ==="

# Test positions (within soft limits)
declare -A TEST_POSITIONS=(
    ["base_main_frame_joint"]="2.0"
    ["main_frame_selector_frame_joint"]="0.7"
    ["selector_frame_gripper_joint"]="0.2"
    ["selector_frame_picker_frame_joint"]="0.15"
    ["picker_frame_picker_rail_joint"]="0.1"
    ["picker_rail_picker_base_joint"]="0.12"
    ["picker_base_picker_jaw_joint"]="0.09"
    ["selector_left_container_jaw_joint"]="0.1"
    ["selector_right_container_jaw_joint"]="0.1"
)

PASSED=0
FAILED=0

for joint in "${!TEST_POSITIONS[@]}"; do
    pos="${TEST_POSITIONS[$joint]}"
    echo ""
    echo "Testing: $joint -> $pos"

    result=$(ros2 action send_goal /move_joint manipulator_control/action/MoveJoint \
        "{joint_name: '$joint', target_position: $pos, max_velocity: 0.0}" 2>&1)

    if echo "$result" | grep -q "success: true"; then
        echo "  ✓ PASS"
        ((PASSED++))
    else
        echo "  ✗ FAIL"
        echo "$result"
        ((FAILED++))
    fi
done

echo ""
echo "=== Results: $PASSED passed, $FAILED failed ==="

if [ $FAILED -eq 0 ]; then
    echo "All MoveJoint regression tests PASSED"
    exit 0
else
    echo "Some MoveJoint regression tests FAILED"
    exit 1
fi
```

**Run the test:**
```bash
chmod +x /tmp/test_movejoint_all_joints.sh
/tmp/test_movejoint_all_joints.sh
```

**Record Result:**
- [ ] base_main_frame_joint: PASS
- [ ] main_frame_selector_frame_joint: PASS
- [ ] selector_frame_gripper_joint: PASS
- [ ] selector_frame_picker_frame_joint: PASS
- [ ] picker_frame_picker_rail_joint: PASS
- [ ] picker_rail_picker_base_joint: PASS
- [ ] picker_base_picker_jaw_joint: PASS
- [ ] selector_left_container_jaw_joint: PASS
- [ ] selector_right_container_jaw_joint: PASS

---

### Phase 8: Visual Verification in Gazebo

Open Gazebo and observe motion during tests:

**Checklist:**
- [ ] Motion is smooth (no jerky steps)
- [ ] Velocity profile is visible (accelerate → cruise → decelerate)
- [ ] Robot stops smoothly at target (no overshoot)
- [ ] Container jaws respond instantly (different from trajectory motion)

---

### Phase 9: Comprehensive Test Script

Create and run the comprehensive test script:

```bash
#!/bin/bash
# Save as /tmp/test_story_2_3_1_complete.sh

source /home/robo/robo/ya_robot_manipulator/ros2_ws/install/setup.bash

echo "=============================================="
echo "  Story 2.3.1 Complete Validation Suite"
echo "=============================================="
echo ""

TOTAL_TESTS=0
PASSED_TESTS=0

# Function to record test result
record_test() {
    local test_name="$1"
    local result="$2"  # 0=pass, 1=fail
    ((TOTAL_TESTS++))
    if [ $result -eq 0 ]; then
        echo "  ✓ $test_name"
        ((PASSED_TESTS++))
    else
        echo "  ✗ $test_name"
    fi
}

echo "=== Phase 2: Controller Type Verification ==="
# Check trajectory controllers (7)
for joint in base_main_frame main_frame_selector_frame selector_frame_gripper \
             selector_frame_picker_frame picker_frame_picker_rail \
             picker_rail_picker_base picker_base_picker_jaw; do
    if ros2 control list_controllers 2>/dev/null | grep "${joint}_joint_controller" | grep -q "joint_trajectory_controller"; then
        record_test "${joint}_joint_controller is JointTrajectoryController" 0
    else
        record_test "${joint}_joint_controller is JointTrajectoryController" 1
    fi
done

# Check forward command controllers (2)
for joint in selector_left_container_jaw selector_right_container_jaw; do
    if ros2 control list_controllers 2>/dev/null | grep "${joint}_joint_controller" | grep -q "forward_command_controller"; then
        record_test "${joint}_joint_controller is ForwardCommandController" 0
    else
        record_test "${joint}_joint_controller is ForwardCommandController" 1
    fi
done

echo ""
echo "=== Phase 3: Action Interface Verification ==="
# Check trajectory actions (7)
for joint in base_main_frame main_frame_selector_frame selector_frame_gripper \
             selector_frame_picker_frame picker_frame_picker_rail \
             picker_rail_picker_base picker_base_picker_jaw; do
    if ros2 action list 2>/dev/null | grep -q "/${joint}_joint_controller/follow_joint_trajectory"; then
        record_test "/${joint}_joint_controller/follow_joint_trajectory available" 0
    else
        record_test "/${joint}_joint_controller/follow_joint_trajectory available" 1
    fi
done

echo ""
echo "=== Phase 4: Topic Verification ==="
# Check command topics (should be only 2)
CMD_TOPICS=$(ros2 topic list 2>/dev/null | grep "_controller/commands" | wc -l)
if [ "$CMD_TOPICS" -eq 2 ]; then
    record_test "Only 2 command topics exist (container jaws)" 0
else
    record_test "Only 2 command topics exist (container jaws) - found $CMD_TOPICS" 1
fi

echo ""
echo "=== Phase 7: MoveJoint Action Server ==="
if ros2 action list 2>/dev/null | grep -q "/move_joint"; then
    record_test "/move_joint action available" 0
else
    record_test "/move_joint action available" 1
fi

echo ""
echo "=============================================="
echo "  SUMMARY: $PASSED_TESTS / $TOTAL_TESTS tests passed"
echo "=============================================="

if [ $PASSED_TESTS -eq $TOTAL_TESTS ]; then
    echo "  ALL TESTS PASSED"
    exit 0
else
    echo "  SOME TESTS FAILED"
    exit 1
fi
```

**Run:**
```bash
chmod +x /tmp/test_story_2_3_1_complete.sh
/tmp/test_story_2_3_1_complete.sh
```

**Record Result:**
- [ ] All verification tests passed
- [ ] Script exit code 0

---

## Dev Agent Record

### Context Reference

- Story Context XML: `docs/sprint-artifacts/2-3-1-migrate-motion-joints-to-trajectory-controller.context.xml`

### Agent Model Used

(To be filled by dev agent)

### Debug Log References

(To be filled by dev agent)

### Completion Notes List

- **AC-1**: [ ] 7 motion joints use JointTrajectoryController
- **AC-2**: [ ] 2 container jaws remain ForwardCommandController
- **AC-3**: [ ] manipulator_controllers.yaml updated
- **AC-4**: [ ] ControllerInterface dual-mode implemented
- **AC-5**: [ ] command_joint() API preserved
- **AC-6**: [ ] wait_for_action_servers() added
- **AC-7**: [ ] cancel_trajectory() added
- **AC-8**: [ ] ros2 control list_controllers shows correct types
- **AC-9**: [ ] All 7 trajectory actions available
- **AC-10**: [ ] MoveJoint regression tests pass
- **AC-11**: [ ] All Story 2.3 tests pass

### Test Results

| Phase | Test | Result | Notes |
|-------|------|--------|-------|
| 1 | Build | | |
| 2 | Controller types (7 trajectory) | | |
| 2 | Controller types (2 forward) | | |
| 3 | Action interfaces (7) | | |
| 4 | Topic verification (2 only) | | |
| 5.1 | base_main_frame_joint | | |
| 5.1 | main_frame_selector_frame_joint | | |
| 5.2 | selector_frame_gripper_joint | | |
| 5.3 | selector_frame_picker_frame_joint | | |
| 5.3 | picker_frame_picker_rail_joint | | |
| 5.3 | picker_rail_picker_base_joint | | |
| 5.3 | picker_base_picker_jaw_joint | | |
| 6 | selector_left_container_jaw_joint | | |
| 6 | selector_right_container_jaw_joint | | |
| 7 | MoveJoint regression (9 joints) | | |
| 8 | Visual verification | | |
| 9 | Comprehensive script | | |

### File List

- `ros2_ws/src/manipulator_description/config/manipulator_controllers.yaml` (MODIFIED)
- `ros2_ws/src/manipulator_control/src/controller_interface.py` (MODIFIED)
- `ros2_ws/src/manipulator_control/package.xml` (MODIFIED)
- `ros2_ws/src/manipulator_control/test/test_story_2_3_1_trajectory_controllers.py` (NEW - optional)

## Change Log

| Date | Change | Author |
|------|--------|--------|
| 2025-11-26 | Story drafted with comprehensive test requirements covering all 9 joints | SM Agent (Bob) |
