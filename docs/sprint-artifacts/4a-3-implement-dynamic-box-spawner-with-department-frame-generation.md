# Story 4A.3: Implement Dynamic Box Spawner with Department Frame Generation

Status: drafted

## Story

As a developer,
I want to dynamically add box URDF with department child links to the ROS2 TF tree,
so that department positions are available for item detection and picking in both hardware and simulation modes.

## Core Concepts

This story implements the **dynamic box spawner** - a critical component enabling box manipulation and item picking operations. The core concepts are:

1. **TF Tree as Source of Truth**: The primary goal is adding box/department frames to the ROS2 TF tree. On real hardware, there's no Gazebo, but department TF frames are essential for knowing where items are located within extracted boxes. The TF tree is the authoritative source for item positions.

2. **URDF Generation with Department Child Links**: Each spawned box is represented as a dynamically-generated URDF robot with:
   - A base link (`{box_id}_base_link`) with visual/collision/inertial properties
   - N department child links (`{box_id}_dept_1_link` through `{box_id}_dept_N_link`) at calculated Y-offsets
   - This URDF is fed to `robot_state_publisher` which broadcasts all transforms to `/tf_static`

3. **Gripper Attachment Pattern**: Boxes are attached to the gripper magnet frame (not world frame). A static transform `left_gripper_magnet` -> `{box_id}_base_link` makes the box follow the gripper during extraction/insertion motions. This is crucial for both TF tree consistency and Gazebo physics.

4. **Dual-Mode Operation**: Works identically for simulation and hardware:
   - **Phase 1-2 (ALWAYS)**: Generate URDF + launch robot_state_publisher -> TF frames exist
   - **Phase 3 (SIMULATION ONLY)**: Spawn visual model in Gazebo with DetachableJoint plugin

5. **DetachableJoint Plugin Integration**: In simulation, Gazebo's DetachableJoint plugin creates a physics constraint between gripper and box. The electromagnet service controls attach/detach, enabling realistic box physics during extraction.

6. **Process Lifecycle Management**: Each spawned box runs its own `robot_state_publisher` subprocess. The spawner tracks PIDs for clean shutdown on despawn to prevent orphan processes and stale TF frames.

## Acceptance Criteria

1. **AC1 - Service Interfaces**: `SpawnBox.srv` and `DespawnBox.srv` are defined in `manipulator_control/srv/`
2. **AC2 - URDF Generation**: When SpawnBox is called, URDF is generated with base_link and N department child links based on `storage_params.yaml` dimensions
3. **AC3 - TF Frame Availability**: When SpawnBox completes, robot_state_publisher is running and department TF frames are queryable via `ros2 run tf2_ros tf2_echo`
4. **AC4 - Gripper Attachment**: Box base_link is attached to gripper_magnet frame via static transform (box follows gripper motion)
5. **AC5 - Gazebo Spawn (Simulation)**: In simulation mode, box model is spawned in Gazebo at gripper position using `ros_gz_interfaces/srv/SpawnEntity`
6. **AC6 - DetachableJoint Integration**: In simulation, box spawns with DetachableJoint plugin configured and initially attached to gripper
7. **AC7 - Department Position Formula**: Department frame Y positions follow: `y = offset_y + (dept_num - 1) * dept_depth` from storage_params.yaml
8. **AC8 - Despawn Cleanup**: When DespawnBox is called, robot_state_publisher is terminated, static transform stops, and (simulation) Gazebo model is deleted
9. **AC9 - Configuration from YAML**: All box dimensions, department configurations, and spawner parameters load from YAML files
10. **AC10 - CLI Testable**: Services callable via `ros2 service call /manipulator/box_spawn/spawn ...`

## Tasks / Subtasks

- [ ] **Task 1: Define Service Interfaces** (AC: #1)
  - [ ] Create `manipulator_control/srv/SpawnBox.srv` with fields: box_id, side, cabinet_num, row, column, num_departments
  - [ ] Create `manipulator_control/srv/DespawnBox.srv` with field: box_id
  - [ ] Update CMakeLists.txt to generate service interfaces
  - [ ] Build and verify services appear in `ros2 interface list`

- [ ] **Task 2: Create Box Spawner Configuration** (AC: #9)
  - [ ] Create `manipulator_control/config/box_spawner.yaml` with parameters:
    - `tf_publish_rate: 10.0` (Hz)
    - `gazebo_world_name: "default"`
    - `spawn_service: "/world/{world_name}/create"`
    - `delete_service: "/world/{world_name}/remove"`
    - `box_mass: 0.5` (kg)
    - `department_marker_radius: 0.01` (m)

- [ ] **Task 3: Implement URDF Generator Utility** (AC: #2, #7)
  - [ ] Create `src/utils/box_urdf_generator.py` with:
    - Load box dimensions from storage_params.yaml based on cabinet configuration
    - Generate URDF XML string with base_link (box geometry, mass, inertia)
    - Generate N department child links with small sphere visuals
    - Generate fixed joints with Y-offsets per department configuration
    - (Simulation) Include DetachableJoint Gazebo plugin in URDF
  - [ ] Unit test: verify URDF is valid XML with expected link/joint count

- [ ] **Task 4: Implement Box Spawn Manager Node** (AC: #3, #4, #5, #6, #8)
  - [ ] Create `src/box_spawn_manager_node.py` with:
    - Service servers for `/manipulator/box_spawn/spawn` and `/manipulator/box_spawn/despawn`
    - TF2 buffer and StaticTransformBroadcaster for gripper->box transform
    - Internal tracking dict: `active_boxes: Dict[str, ActiveBox]` with RSP process handles
    - Detection of simulation vs hardware mode via Gazebo service availability
  - [ ] Implement `_spawn_box()` handler:
    - Phase 1: Call box_urdf_generator to create URDF string
    - Phase 2: Launch robot_state_publisher subprocess with URDF
    - Phase 2: Publish static transform gripper_magnet -> box_base_link
    - Phase 3 (sim): Call Gazebo SpawnEntity service
    - Phase 3 (sim): Publish to attach topic for DetachableJoint
  - [ ] Implement `_despawn_box()` handler:
    - (sim) Publish to detach topic, wait 0.2s
    - Kill robot_state_publisher subprocess
    - Stop static transform broadcasting
    - (sim) Call Gazebo DeleteEntity service
    - Remove from active_boxes tracking

- [ ] **Task 5: Add Node to Launch Configuration** (AC: #10)
  - [ ] Add `box_spawn_manager_node` to `manipulator_simulation.launch.py`
  - [ ] Use simulation-only condition (only launch when use_sim_time=true)
  - [ ] Pass config file path as parameter

- [ ] **Task 6: Developer Self-Testing** (MANDATORY)
  - [ ] **Unit Test 1**: Verify URDF generation produces valid XML with correct link count
  - [ ] **Unit Test 2**: Verify department Y-offset calculation matches formula
  - [ ] **Integration Test 1**: Launch simulation, call SpawnBox, verify TF frames exist via `ros2 run tf2_ros tf2_echo world box_l_1_2_3_dept_1_link`
  - [ ] **Integration Test 2**: With box spawned, verify box moves with gripper (navigate to different address)
  - [ ] **Integration Test 3**: Call DespawnBox, verify TF frames are removed and no orphan RSP processes
  - [ ] **Integration Test 4**: Verify Gazebo shows box model attached to gripper (visual inspection)
  - [ ] **CLI Test**: Document commands in README for manual testing

- [ ] **Documentation Sync** (MANDATORY)
  - [ ] Update `ros2_ws/src/manipulator_control/README.md` with box spawner documentation
  - [ ] Add docstrings to `box_urdf_generator.py` and `box_spawn_manager_node.py`
  - [ ] Document SpawnBox/DespawnBox service interfaces

## Dev Notes

### Architecture Alignment

- **Phase 3: Box Handling** - This story provides the box representation layer for ExtractBox/ReturnBox actions
- **Gripper Frames**: `left_gripper_magnet` and `right_gripper_magnet` are defined in URDF
- **DetachableJoint Plugin**: Gazebo Harmonic system plugin - configured per-box in generated URDF
- **robot_state_publisher**: Standard ROS2 package for TF broadcasting from URDF

### Implementation Pattern

```python
@dataclass
class ActiveBox:
    box_id: str                    # e.g., "box_l_1_2_3"
    rsp_process: subprocess.Popen  # robot_state_publisher process
    urdf: str                      # Generated URDF string
    num_departments: int           # Department count
    source_address: dict           # {side, cabinet, row, column}

class BoxSpawnManagerNode(Node):
    def __init__(self):
        super().__init__('box_spawn_manager_node')

        # Load config
        self.declare_parameter('config_file', '')
        self.config = self._load_config()

        # State tracking
        self.active_boxes: Dict[str, ActiveBox] = {}

        # TF2 for position lookups and static broadcasting
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.static_broadcaster = StaticTransformBroadcaster(self)

        # Detect simulation mode
        self.simulation_mode = self._check_gazebo_available()

        # Services
        self.spawn_srv = self.create_service(SpawnBox, '/manipulator/box_spawn/spawn', self._spawn_callback)
        self.despawn_srv = self.create_service(DespawnBox, '/manipulator/box_spawn/despawn', self._despawn_callback)

        # Gazebo clients (simulation only)
        if self.simulation_mode:
            self.spawn_client = self.create_client(SpawnEntity, f'/world/{self.config["gazebo_world_name"]}/create')
            self.delete_client = self.create_client(DeleteEntity, f'/world/{self.config["gazebo_world_name"]}/remove')
```

### URDF Structure

```xml
<robot name="box_l_1_2_3">
  <link name="box_l_1_2_3_base_link">
    <visual>
      <geometry><box size="0.4 0.6 0.3"/></geometry>
      <material name="box_material"><color rgba="0.6 0.4 0.2 1.0"/></material>
    </visual>
    <collision><geometry><box size="0.4 0.6 0.3"/></geometry></collision>
    <inertial>
      <mass value="0.5"/>
      <inertia ixx="0.01" iyy="0.01" izz="0.01" ixy="0" ixz="0" iyz="0"/>
    </inertial>
  </link>

  <link name="box_l_1_2_3_dept_1_link">
    <visual><geometry><sphere radius="0.01"/></geometry></visual>
  </link>

  <joint name="box_l_1_2_3_dept_1_joint" type="fixed">
    <parent link="box_l_1_2_3_base_link"/>
    <child link="box_l_1_2_3_dept_1_link"/>
    <origin xyz="0 0.05 0" rpy="0 0 0"/>  <!-- offset_y + 0 * dept_depth -->
  </joint>

  <!-- Repeat for dept_2 through dept_N with increasing Y offsets -->

  <!-- Gazebo DetachableJoint plugin (simulation only) -->
  <gazebo>
    <plugin filename="gz-sim-detachable-joint-system" name="gz::sim::systems::DetachableJoint">
      <parent_link>left_gripper_magnet</parent_link>
      <child_model>box_l_1_2_3</child_model>
      <child_link>box_l_1_2_3_base_link</child_link>
      <topic>/model/box_l_1_2_3/detachable_joint</topic>
    </plugin>
  </gazebo>
</robot>
```

### robot_state_publisher Launch

```python
import subprocess

def _launch_rsp(self, box_id: str, urdf: str) -> subprocess.Popen:
    """Launch robot_state_publisher for a box."""
    cmd = [
        'ros2', 'run', 'robot_state_publisher', 'robot_state_publisher',
        '--ros-args',
        '-p', f'robot_description:={urdf}',
        '-p', f'use_sim_time:={str(self.use_sim_time).lower()}',
        '-r', f'__node:=rsp_{box_id}'  # Unique node name
    ]
    return subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
```

### Gazebo Spawn/Delete Pattern

```python
from ros_gz_interfaces.srv import SpawnEntity, DeleteEntity
from std_msgs.msg import Empty

# Spawn in Gazebo
req = SpawnEntity.Request()
req.entity_factory.name = box_id
req.entity_factory.sdf = urdf_with_gazebo_plugin  # URDF works in sdf field
req.entity_factory.allow_renaming = False
# Get spawn pose from gripper TF
gripper_tf = self.tf_buffer.lookup_transform('world', 'left_gripper_magnet', ...)
req.entity_factory.pose = self._tf_to_pose(gripper_tf)
future = self.spawn_client.call_async(req)

# Attach via DetachableJoint
attach_pub = self.create_publisher(Empty, f'/model/{box_id}/detachable_joint/attach', 10)
attach_pub.publish(Empty())

# Delete from Gazebo
del_req = DeleteEntity.Request()
del_req.entity.name = box_id
del_req.entity.type = 2  # MODEL type
future = self.delete_client.call_async(del_req)
```

### Project Structure Notes

- **Service Files**: `ros2_ws/src/manipulator_control/srv/SpawnBox.srv`, `DespawnBox.srv`
- **Utility**: `ros2_ws/src/manipulator_control/src/utils/box_urdf_generator.py`
- **Node File**: `ros2_ws/src/manipulator_control/src/box_spawn_manager_node.py`
- **Config File**: `ros2_ws/src/manipulator_control/config/box_spawner.yaml`

### Testing Standards

**MANDATORY Developer Self-Testing Before Story Completion:**

1. TF frames must appear within 2 seconds of SpawnBox completion
2. Department frame count must match requested num_departments
3. Box must visually follow gripper in Gazebo when gripper moves
4. DespawnBox must leave no orphan processes (verify with `ps aux | grep robot_state_publisher`)
5. All service calls must complete within 5 seconds

### Learnings from Previous Story

**From Story 4a-2 (Status: ready-for-dev)**

- **Electromagnet Service Available**: `ToggleElectromagnet` service is implemented at `/manipulator/electromagnet/toggle`
- **Node Pattern**: Follow same config loading pattern via `ament_index_python`
- **TF2 Pattern**: Use TF2 buffer/listener for frame lookups (see `electromagnet_simulator_node.py`)
- **Launch Integration**: Add to `manipulator_simulation.launch.py` with simulation-only condition

[Source: docs/sprint-artifacts/4a-2-implement-electromagnet-simulator-and-service.md#Dev-Agent-Record]

### Dependencies

**Required Packages:**
- `ros_gz_interfaces` - SpawnEntity and DeleteEntity services for Gazebo Harmonic
- `robot_state_publisher` - TF broadcasting from URDF
- `tf2_ros` - Transform lookups and static broadcasting

**Internal Dependencies:**
- `storage_params.yaml` - Box dimensions and department configurations
- Electromagnet service from Story 4a-2 (for attach/detach coordination)

### References

- [Source: docs/sprint-artifacts/tech-spec-epic-4a.md#AC-4A.3] - Acceptance criteria definitions
- [Source: docs/epics.md#Story-4A.3] - Original story specification with implementation details
- [Source: docs/sprint-artifacts/tech-spec-epic-4a.md#Box Spawner] - Service interface definitions
- [Gazebo Harmonic Spawn](https://gazebosim.org/docs/harmonic/ros2_spawn_model/) - External reference
- [robot_state_publisher URDF](https://docs.ros.org/en/humble/Tutorials/Intermediate/URDF/Using-URDF-with-Robot-State-Publisher.html) - External reference

## Dev Agent Record

### Context Reference

<!-- Path(s) to story context XML will be added here by context workflow -->

### Agent Model Used

{{agent_model_name_version}}

### Debug Log References

### Completion Notes List

### File List
