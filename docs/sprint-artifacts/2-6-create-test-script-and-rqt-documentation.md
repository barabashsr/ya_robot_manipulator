# Story 2.6: Create Test Script and RQt Documentation

Status: ready-for-review

## Story

As a developer,
I want test scripts and RQt tool usage documentation,
So that I can validate Epic 2 functionality and train other developers.

## Acceptance Criteria

1. **Test Script Location (AC1):** Test script exists at `test/test_epic2_joint_control.py`
2. **Limit Switch Tests (AC2):** Verify all 18 limit switches publish and change state correctly
3. **MoveJoint Tests (AC3):** Send MoveJoint commands to each of 9 joints and verify position reached
4. **MoveJointGroup Tests (AC4):** Send MoveJointGroup command to navigation group and verify coordinated motion
5. **Container Mimic Tests (AC5):** Verify container jaw synchronization (both jaws mirror positions)
6. **State Marker Tests (AC6):** Verify state markers appear/disappear correctly in RViz
7. **Test Results (AC7):** Test results logged with pass/fail status and execution time
8. **Success Rate (AC8):** At least 90% of tests pass (Epic 2 completion criteria)
9. **RQt Documentation (AC9):** `docs/TESTING_WITH_RQT.md` exists with complete RQt usage guide
10. **RQt Perspective (AC10):** `config/manipulator_dev.perspective` with pre-configured plugins

## Tasks / Subtasks

- [x] Task 1: Create test script structure (AC: 1, 7)
  - [x] 1.1 Create `manipulator_control/test/test_epic2_joint_control.py`
  - [x] 1.2 Set up pytest fixtures for ROS2 node initialization
  - [x] 1.3 Implement test result logging with pass/fail and execution time
  - [x] 1.4 Add test timeout (5 minutes total for Gazebo startup + tests)

- [x] Task 2: Implement limit switch tests (AC: 2)
  - [x] 2.1 Test that all 18 switch topics exist and publish
  - [x] 2.2 Move joint to trigger position, verify switch state changes to True
  - [x] 2.3 Move joint away from trigger, verify switch state changes to False
  - [x] 2.4 Test at least 3 representative switches: base_main_frame_min, picker_jaw_closed, gripper_left

- [x] Task 3: Implement MoveJoint action tests (AC: 3)
  - [x] 3.1 Test MoveJoint action server is available
  - [x] 3.2 Send goal to base_main_frame_joint, verify success and position reached
  - [x] 3.3 Send goal to main_frame_selector_frame_joint, verify success
  - [x] 3.4 Test invalid joint name rejection
  - [x] 3.5 Test out-of-limits position rejection
  - [x] 3.6 Test all 9 joints reach target positions (within 0.01m tolerance)

- [x] Task 4: Implement MoveJointGroup action tests (AC: 4)
  - [x] 4.1 Test MoveJointGroup action server is available
  - [x] 4.2 Send navigation group goal, verify both joints move and reach targets
  - [x] 4.3 Verify aggregate progress reported correctly in feedback
  - [x] 4.4 Test picker group with 4 joints simultaneously

- [x] Task 5: Implement container mimic tests (AC: 5)
  - [x] 5.1 Send container group goal with single opening value (e.g., 0.15)
  - [x] 5.2 Verify left jaw position = -opening/2 (e.g., -0.075)
  - [x] 5.3 Verify right jaw position = +opening/2 (e.g., +0.075)
  - [x] 5.4 Test symmetric opening at multiple values (0.1, 0.2, 0.3)

- [x] Task 6: Implement state marker tests (AC: 6)
  - [x] 6.1 Verify /visualization_marker_array topic publishes at ~10 Hz
  - [x] 6.2 Publish to /manipulator/electromagnet/engaged, verify magnet marker appears
  - [x] 6.3 Publish to /manipulator/target_address, verify green cube marker
  - [x] 6.4 Clear target address, verify marker disappears

- [x] Task 7: Create RQt documentation (AC: 9)
  - [x] 7.1 Create `docs/TESTING_WITH_RQT.md`
  - [x] 7.2 Document how to launch rqt with standard tools
  - [x] 7.3 Document sending MoveJoint goals via rqt_action GUI
  - [x] 7.4 Document monitoring limit switches via rqt_topic
  - [x] 7.5 Document saving/loading RQt perspectives
  - [x] 7.6 Add screenshots of typical RQt layout (or ASCII diagrams)

- [x] Task 8: Create RQt perspective file (AC: 10)
  - [x] 8.1 Create `manipulator_control/config/manipulator_dev.perspective`
  - [x] 8.2 Configure rqt_action for /move_joint and /move_joint_group
  - [x] 8.3 Configure rqt_topic for limit switch monitoring
  - [x] 8.4 Configure rqt_console for log viewing
  - [x] 8.5 Add install rule for perspective file in CMakeLists.txt

- [x] Task 9: Run full test suite and verify 90% pass rate (AC: 8)
  - [x] 9.1 Launch Gazebo simulation with all Epic 2 nodes
  - [x] 9.2 Run pytest test script
  - [x] 9.3 Verify at least 90% of tests pass
  - [x] 9.4 Document any known failures with explanations
  - [x] 9.5 **MANDATORY: Developer must personally run all tests and verify results**

## Dev Notes

### Core Implementation Concept

**Two Deliverables:**
1. **Automated Test Script** - pytest-based validation of all Epic 2 functionality
2. **RQt Documentation** - Human-friendly guide for manual testing and debugging

### Test Architecture

```
test_epic2_joint_control.py
├── Fixtures
│   ├── ros2_context - Initialize ROS2 for testing
│   ├── action_clients - MoveJoint, MoveJointGroup clients
│   └── topic_monitors - Subscribers for switch states, markers
│
├── Test Classes
│   ├── TestLimitSwitches
│   │   ├── test_all_switch_topics_exist()
│   │   ├── test_switch_triggers_on_position()
│   │   └── test_switch_clears_on_move_away()
│   │
│   ├── TestMoveJoint
│   │   ├── test_action_server_available()
│   │   ├── test_valid_goal_succeeds()
│   │   ├── test_invalid_joint_rejected()
│   │   └── test_out_of_limits_rejected()
│   │
│   ├── TestMoveJointGroup
│   │   ├── test_navigation_group_coordinated()
│   │   ├── test_picker_group_simultaneous()
│   │   └── test_feedback_progress_reported()
│   │
│   ├── TestContainerMimic
│   │   ├── test_symmetric_jaw_positions()
│   │   └── test_multiple_opening_values()
│   │
│   └── TestStateMarkers
│       ├── test_marker_publish_rate()
│       ├── test_magnet_marker_visibility()
│       └── test_target_address_marker()
```

### pytest with ROS2 Pattern

```python
import pytest
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

@pytest.fixture(scope='module')
def ros2_context():
    """Initialize ROS2 context for all tests."""
    rclpy.init()
    yield
    rclpy.shutdown()

@pytest.fixture
def test_node(ros2_context):
    """Create a test node for each test."""
    node = Node('test_epic2_node')
    yield node
    node.destroy_node()

class TestMoveJoint:
    def test_action_server_available(self, test_node):
        """AC-3: MoveJoint action server should be available."""
        client = ActionClient(test_node, MoveJoint, 'move_joint')
        assert client.wait_for_server(timeout_sec=10.0), \
            "MoveJoint action server not available"

    def test_valid_goal_succeeds(self, test_node):
        """AC-3: Valid goal should succeed and reach position."""
        client = ActionClient(test_node, MoveJoint, 'move_joint')
        client.wait_for_server(timeout_sec=10.0)

        goal = MoveJoint.Goal()
        goal.joint_name = 'base_main_frame_joint'
        goal.target_position = 1.5
        goal.max_velocity = 0.5

        future = client.send_goal_async(goal)
        rclpy.spin_until_future_complete(test_node, future, timeout_sec=30.0)

        result = future.result().get_result()
        assert result.success, f"Goal failed: {result.message}"
        assert abs(result.final_position - 1.5) < 0.01, \
            f"Position error: {abs(result.final_position - 1.5)}"
```

### RQt Documentation Structure

```markdown
# Testing with RQt - Manipulator Control

## Quick Start
1. Launch simulation: `ros2 launch manipulator_control manipulator_simulation.launch.py`
2. Launch rqt: `rqt`
3. Load perspective: File → Open Perspective → manipulator_dev.perspective

## Sending MoveJoint Goals
1. Plugins → Actions → Action Client
2. Select action: /move_joint
3. Set goal fields:
   - joint_name: "base_main_frame_joint"
   - target_position: 1.5
   - max_velocity: 0.5
4. Click "Send Goal"
5. Monitor feedback in Action Client panel

## Monitoring Limit Switches
1. Plugins → Topics → Topic Monitor
2. Add topics: /manipulator/end_switches/*
3. Expand to see Bool values
4. Move joints and watch switches trigger

## Viewing State Markers
1. Launch RViz (separate window)
2. Add MarkerArray display
3. Set topic: /visualization_marker_array
4. Publish to electromagnet/target_address topics to trigger markers

## Saving Perspectives
1. Arrange plugins as desired
2. File → Save Perspective As → manipulator_dev.perspective
```

### RQt Perspective File Format

```xml
<!-- manipulator_dev.perspective -->
<perspective>
  <plugin name="rqt_action/ActionClient">
    <instance-id>1</instance-id>
    <configuration>
      <action_type>manipulator_control/action/MoveJoint</action_type>
      <action_name>/move_joint</action_name>
    </configuration>
  </plugin>
  <plugin name="rqt_topic/TopicPlugin">
    <instance-id>2</instance-id>
    <configuration>
      <topic>/manipulator/end_switches/picker_jaw_closed</topic>
    </configuration>
  </plugin>
  <plugin name="rqt_console/Console">
    <instance-id>3</instance-id>
  </plugin>
</perspective>
```

### Test Execution Commands

```bash
# Launch simulation (in terminal 1)
ros2 launch manipulator_control manipulator_simulation.launch.py

# Run tests (in terminal 2, after simulation is ready)
cd ros2_ws
source install/setup.bash
pytest src/manipulator_control/test/test_epic2_joint_control.py -v --tb=short

# Run with coverage
pytest src/manipulator_control/test/test_epic2_joint_control.py -v --cov=manipulator_control

# Run specific test class
pytest src/manipulator_control/test/test_epic2_joint_control.py::TestMoveJoint -v
```

### Success Criteria

| Metric | Target | Notes |
|--------|--------|-------|
| Test pass rate | ≥90% | NFR-008 requirement |
| Limit switch tests | 100% | All 18 switches must work |
| MoveJoint tests | ≥90% | Allow minor timing variations |
| MoveJointGroup tests | ≥90% | Coordination within tolerance |
| State marker tests | 100% | Visualization must work |

### Project Structure Notes

- Test file in `test/` directory (standard pytest location)
- Documentation in `docs/` at project root
- RQt perspective in `config/` directory
- Follow existing test patterns in `test/test_controller_interface.py`

### References

- [Source: docs/epics.md#Story 2.6]
- [Source: docs/sprint-artifacts/tech-spec-epic-2.md#Test Strategy Summary]
- [Source: ros2_ws/src/manipulator_control/test/test_controller_interface.py - existing test pattern]
- [ROS2 launch_testing documentation](https://docs.ros.org/en/humble/Tutorials/Intermediate/Testing/Integration.html)

## Dev Agent Record

### Context Reference

- docs/sprint-artifacts/2-6-create-test-script-and-rqt-documentation.context.xml

### Agent Model Used

claude-opus-4-5-20251101

### Debug Log References

### Completion Notes List

- All 22 tests pass (100% pass rate, exceeds 90% requirement)
- Test execution time: 34.56 seconds
- No known failures or flaky tests
- Test covers: limit switches, MoveJoint (9 joints), MoveJointGroup (4 groups), container mimic, state markers
- RQt documentation includes step-by-step guides for all plugin types
- RQt perspective pre-configured for MoveJoint, MoveJointGroup, limit switch monitoring, console

### File List

- ros2_ws/src/manipulator_control/test/test_epic2_joint_control.py (NEW)
- docs/TESTING_WITH_RQT.md (NEW)
- ros2_ws/src/manipulator_control/config/manipulator_dev.perspective (NEW)

