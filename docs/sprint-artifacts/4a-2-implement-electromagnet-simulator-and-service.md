# Story 4A.2: Implement Electromagnet Simulator and Service

Status: drafted

## Story

As a developer,
I want to simulate electromagnet attachment/detachment in Gazebo,
so that box extraction actions can control magnet state via ROS2 service.

## Core Concepts

This story implements the **virtual electromagnet simulation** that enables box manipulation in the Gazebo simulation environment. The core concepts are:

1. **DetachableJoint Plugin Integration**: Gazebo Harmonic's DetachableJoint plugin creates/removes a fixed joint between the gripper magnet link and box model, simulating magnetic attachment without actual physics.

2. **Proximity-Gated Engagement**: The electromagnet only engages if a box is within 5cm of the gripper magnet link. TF2 lookups calculate this distance in real-time.

3. **State Publication Pattern**: Continuous 10Hz state publication on `/manipulator/electromagnet/engaged` enables visualization (red sphere marker on gripper) and upstream action server coordination.

4. **Service-Based Control**: `ToggleElectromagnet` service provides the command interface. Actions call this service during box extraction/return workflows.

5. **Simulation-Only Implementation**: This is a Gazebo-specific node. Hardware implementation (GPIO relay control) will be a separate story.

## Acceptance Criteria

1. **AC1 - Service Definition Exists**: `srv/ToggleElectromagnet.srv` is defined with `bool activate` request and `bool success, string message` response
2. **AC2 - Activation with Proximity Check**: When `ToggleElectromagnet(activate=true)` is called with box within 5cm, DetachableJoint attach message is published and engaged state becomes true
3. **AC3 - Activation without Proximity Fails**: When `ToggleElectromagnet(activate=true)` is called with NO box in proximity, service returns success=false with appropriate message, state remains false
4. **AC4 - Deactivation Releases Box**: When `ToggleElectromagnet(activate=false)` is called with box attached, DetachableJoint detach message is published and engaged state becomes false
5. **AC5 - State Topic Publication**: Magnet state publishes to `/manipulator/electromagnet/engaged` (std_msgs/Bool) at 10 Hz
6. **AC6 - Configuration from YAML**: Proximity distance, wait times, and topic names load from `config/electromagnet.yaml`
7. **AC7 - CLI Testable**: Service can be called via `ros2 service call /manipulator/electromagnet/toggle manipulator_interfaces/srv/ToggleElectromagnet "{activate: true}"`

## Tasks / Subtasks

- [ ] **Task 1: Define Service Interface** (AC: #1)
  - [ ] Create `manipulator_interfaces/srv/ToggleElectromagnet.srv` with documented fields
  - [ ] Build `manipulator_interfaces` package and verify service shows in `ros2 interface list`

- [ ] **Task 2: Create Configuration File** (AC: #6)
  - [ ] Create `manipulator_control/config/electromagnet.yaml` with parameters:
    - `proximity_distance: 0.05` (meters)
    - `engagement_wait_sec: 0.5`
    - `disengagement_wait_sec: 0.3`
    - `state_topic: "/manipulator/electromagnet/engaged"`
    - `state_publish_rate: 10.0` (Hz)
    - `left_gripper_frame: "left_gripper_magnet"`
    - `right_gripper_frame: "right_gripper_magnet"`

- [ ] **Task 3: Implement Electromagnet Simulator Node** (AC: #2, #3, #4, #5)
  - [ ] Create `src/electromagnet_simulator_node.py` with:
    - ROS2 node class `ElectromagnetSimulatorNode`
    - Service server for `/manipulator/electromagnet/toggle`
    - TF2 buffer and listener for proximity checks
    - Timer for state publication at configured rate
    - Publishers for DetachableJoint attach/detach topics
  - [ ] Implement `_check_box_proximity()` method:
    - Query TF for all frames matching pattern `box_*_base_link`
    - Calculate distance from gripper_magnet to each box
    - Return closest box_id if within proximity_distance, else None
  - [ ] Implement `_toggle_callback()` service handler:
    - If activate=true: check proximity, publish to attach topic if box found
    - If activate=false: publish to detach topic if currently engaged
    - Track engaged state and attached_box_id
  - [ ] Implement state timer callback publishing Bool to state_topic

- [ ] **Task 4: Add Node to Launch Configuration** (AC: #7)
  - [ ] Add `electromagnet_simulator_node` to `manipulator_simulation.launch.py`
  - [ ] Pass config file path as parameter

- [ ] **Task 5: Developer Self-Testing** (MANDATORY)
  - [ ] **Unit Test**: Verify service interface builds correctly
  - [ ] **Integration Test 1**: Launch simulation, call service with no box nearby, verify `success=false`
  - [ ] **Integration Test 2**: Spawn test box near gripper, call `activate=true`, verify attach topic receives message
  - [ ] **Integration Test 3**: With box attached, call `activate=false`, verify detach topic receives message
  - [ ] **Integration Test 4**: Echo state topic, verify 10Hz publication rate
  - [ ] **Test Script**: Create `scripts/test_electromagnet.py` for manual verification:
    ```bash
    # Commands to run:
    ros2 launch manipulator_control manipulator_simulation.launch.py
    ros2 topic hz /manipulator/electromagnet/engaged  # Should show ~10Hz
    ros2 service call /manipulator/electromagnet/toggle manipulator_interfaces/srv/ToggleElectromagnet "{activate: true}"
    ```

- [ ] **Documentation Sync** (MANDATORY)
  - [ ] Update `ros2_ws/src/manipulator_control/README.md` with electromagnet node documentation
  - [ ] Add docstrings to `electromagnet_simulator_node.py` module/class/methods
  - [ ] Update `manipulator_interfaces/README.md` with ToggleElectromagnet service

## Dev Notes

### Architecture Alignment

- **Phase 3: Box Handling** - This story is a prerequisite for ExtractBox/ReturnBox actions
- **DetachableJoint Plugin**: Gazebo Harmonic system plugin - no custom plugin needed
- **Gripper Frames**: `left_gripper_magnet` and `right_gripper_magnet` are defined in URDF
- **Topic Pattern**: DetachableJoint topics follow `/model/{model_name}/detachable_joint/attach|detach`

### Implementation Pattern

```python
class ElectromagnetSimulatorNode(Node):
    def __init__(self):
        super().__init__('electromagnet_simulator_node')

        # Load config
        self.declare_parameter('config_file', '')
        config = self._load_config()

        # State
        self.engaged = False
        self.attached_box_id = None

        # TF2 for proximity checks
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Service
        self.srv = self.create_service(
            ToggleElectromagnet,
            '/manipulator/electromagnet/toggle',
            self._toggle_callback
        )

        # State publisher
        self.state_pub = self.create_publisher(Bool, config['state_topic'], 10)
        self.state_timer = self.create_timer(1.0/config['state_publish_rate'], self._publish_state)
```

### DetachableJoint Topic Format

```python
# To attach box "box_l_1_2_3" to gripper:
attach_pub = self.create_publisher(Empty, '/model/box_l_1_2_3/detachable_joint/attach', 10)
attach_pub.publish(Empty())

# To detach:
detach_pub = self.create_publisher(Empty, '/model/box_l_1_2_3/detachable_joint/detach', 10)
detach_pub.publish(Empty())
```

### Proximity Check Logic

```python
def _check_box_proximity(self, gripper_frame: str) -> Optional[str]:
    """Find closest box within proximity distance.

    Returns box_id if found, None otherwise.
    """
    # List all TF frames, filter for box_*_base_link pattern
    # For each, lookup transform gripper_frame -> box_frame
    # Calculate distance, return closest if < proximity_distance
```

### Project Structure Notes

- **Service File**: `ros2_ws/src/manipulator_interfaces/srv/ToggleElectromagnet.srv`
- **Node File**: `ros2_ws/src/manipulator_control/src/electromagnet_simulator_node.py`
- **Config File**: `ros2_ws/src/manipulator_control/config/electromagnet.yaml`
- **Test Script**: `ros2_ws/src/manipulator_control/scripts/test_electromagnet.py`

### Testing Standards

**MANDATORY Developer Self-Testing Before Story Completion:**

1. Service must respond correctly to all test cases (proximity/no-proximity, engage/disengage)
2. State topic must publish at specified rate (verify with `ros2 topic hz`)
3. All code paths must be exercised manually before marking complete
4. Test script must run without errors in clean simulation environment

### References

- [Source: docs/sprint-artifacts/tech-spec-epic-4a.md#AC-4A.2] - Acceptance criteria definitions
- [Source: docs/epics.md#Story-4A.2] - Original story specification with DetachableJoint details
- [Source: tech-spec-epic-4a.md#Electromagnet Simulation] - Service interface and data models

## Dev Agent Record

### Context Reference

<!-- Path(s) to story context XML will be added here by context workflow -->

### Agent Model Used

<!-- Will be filled by dev agent -->

### Debug Log References

### Completion Notes List

### File List
