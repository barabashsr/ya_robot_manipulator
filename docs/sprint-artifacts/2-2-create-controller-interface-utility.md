# Story 2.2: Create Controller Interface Utility

Status: ready-for-dev

## Story

As a **developer**,
I want **a utility to send position commands to individual ForwardCommandController instances**,
So that **action servers can control joints without hardcoding controller topic names**.

## Acceptance Criteria

1. **AC-1:** ControllerInterface class exists at `ros2_ws/src/manipulator_control/src/controller_interface.py`
2. **AC-2:** Utility provides `command_joint(joint_name: str, position: float) -> bool` method that publishes Float64 to `/[joint_name]_controller/command`
3. **AC-3:** Utility provides `command_joint_group(joint_names: List[str], positions: List[float]) -> bool` method for simultaneous multi-joint commands
4. **AC-4:** Utility provides `get_joint_position(joint_name: str) -> Optional[float]` method that reads from `/joint_states`
5. **AC-5:** Utility provides `get_joint_limits(joint_name: str) -> Tuple[float, float]` method returning (min, max) limits
6. **AC-6:** Position commands are validated against joint limits before publishing; invalid commands return False with logged warning
7. **AC-7:** Controller names and joint limits are loaded from `manipulator_controllers.yaml` and `ros2_control.xacro` (or hardcoded based on these files)
8. **AC-8:** Update `manipulator_simulation.launch.py` to add `use_sim_time` argument with default `true` and apply `IfCondition(use_sim_time)` to virtual_limit_switches node

## Tasks / Subtasks

- [ ] Task 1: Create ControllerInterface Class Structure (AC: 1, 7)
  - [ ] 1.1 Create `src/controller_interface.py` with class skeleton
  - [ ] 1.2 Define JOINT_CONTROLLERS dict mapping joint names to controller topics
  - [ ] 1.3 Define JOINT_LIMITS dict mapping joint names to (min, max) tuples
  - [ ] 1.4 Initialize node reference and joint_states storage in `__init__`

- [ ] Task 2: Implement Joint State Subscription (AC: 4)
  - [ ] 2.1 Subscribe to `/joint_states` topic (sensor_msgs/JointState)
  - [ ] 2.2 Store latest joint positions in dict keyed by joint name
  - [ ] 2.3 Implement `get_joint_position(joint_name)` returning Optional[float]

- [ ] Task 3: Implement Command Publishers (AC: 2, 3)
  - [ ] 3.1 Create publishers dict for all 9 controller topics on initialization
  - [ ] 3.2 Implement `command_joint(joint_name, position)` with limit validation
  - [ ] 3.3 Implement `command_joint_group(joint_names, positions)` for multi-joint commands
  - [ ] 3.4 Return False and log warning for unknown joints or out-of-limit positions

- [ ] Task 4: Implement Joint Limits Methods (AC: 5, 6)
  - [ ] 4.1 Implement `get_joint_limits(joint_name)` returning Tuple[float, float]
  - [ ] 4.2 Implement `validate_position(joint_name, position)` helper method
  - [ ] 4.3 Log warning messages for invalid positions with joint name and limits

- [ ] Task 5: Update Launch File (AC: 8)
  - [ ] 5.1 Add `use_sim_time` DeclareLaunchArgument with default 'true'
  - [ ] 5.2 Add `IfCondition(use_sim_time)` to virtual_limit_switches node
  - [ ] 5.3 Pass `use_sim_time` parameter to all nodes

- [ ] Task 6: Update Package and Build (AC: 1)
  - [ ] 6.1 Ensure CMakeLists.txt installs controller_interface.py
  - [ ] 6.2 Rebuild package
  - [ ] 6.3 Verify installation

- [ ] Task 7: Validate Implementation (AC: 2, 3, 4, 5, 6)
  - [ ] 7.1 Run unit tests for ControllerInterface
  - [ ] 7.2 Test command_joint with valid and invalid positions
  - [ ] 7.3 Test command_joint_group with multiple joints
  - [ ] 7.4 Test get_joint_position with running simulation
  - [ ] 7.5 Verify launch file with use_sim_time argument

## Dev Notes

### Learnings from Story 2.1

**From Story 2-1-implement-virtual-limit-switch-simulation (Status: done)**
- Virtual limit switches working at 10 Hz, all 18 topics verified
- Config adjusted for Gazebo simulation limits (5mm inset from joint boundaries)
- Unified launch file `manipulator_simulation.launch.py` created with delayed node starts
- Build time ~2.5s for manipulator_control package

[Source: docs/sprint-artifacts/2-1-implement-virtual-limit-switch-simulation.md#Dev-Agent-Record]

### Architecture References

- [Source: docs/architecture-ros2-control-v2-CORRECTIONS.md] Lines 13-28: Individual ForwardCommandControllers per joint
- [Source: docs/sprint-artifacts/tech-spec-epic-2.md] Controller Interface Utility specification

### Joint Controllers Mapping

| Joint Name | Controller Topic |
|------------|------------------|
| base_main_frame_joint | /base_main_frame_joint_controller/command |
| main_frame_selector_frame_joint | /main_frame_selector_frame_joint_controller/command |
| selector_left_container_jaw_joint | /selector_left_container_jaw_joint_controller/command |
| selector_right_container_jaw_joint | /selector_right_container_jaw_joint_controller/command |
| selector_frame_gripper_joint | /selector_frame_gripper_joint_controller/command |
| selector_frame_picker_frame_joint | /selector_frame_picker_frame_joint_controller/command |
| picker_frame_picker_rail_joint | /picker_frame_picker_rail_joint_controller/command |
| picker_rail_picker_base_joint | /picker_rail_picker_base_joint_controller/command |
| picker_base_picker_jaw_joint | /picker_base_picker_jaw_joint_controller/command |

### Joint Limits Reference (from ros2_control.xacro)

| Joint Name | Min | Max |
|------------|-----|-----|
| base_main_frame_joint | 0.0 | 4.0 |
| main_frame_selector_frame_joint | -0.01 | 1.5 |
| selector_left_container_jaw_joint | -0.2 | 0.2 |
| selector_right_container_jaw_joint | -0.2 | 0.2 |
| selector_frame_gripper_joint | -0.4 | 0.4 |
| selector_frame_picker_frame_joint | -0.01 | 0.3 |
| picker_frame_picker_rail_joint | -0.3 | 0.3 |
| picker_rail_picker_base_joint | 0.0 | 0.12 |
| picker_base_picker_jaw_joint | 0.0 | 0.2 |

### Implementation Pattern (from tech-spec)

```python
from typing import List, Optional, Tuple
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState


class ControllerInterface:
    """Utility for commanding individual ForwardCommandControllers"""

    # Controller topic mapping
    JOINT_CONTROLLERS = {
        'base_main_frame_joint': '/base_main_frame_joint_controller/command',
        'main_frame_selector_frame_joint': '/main_frame_selector_frame_joint_controller/command',
        'selector_left_container_jaw_joint': '/selector_left_container_jaw_joint_controller/command',
        'selector_right_container_jaw_joint': '/selector_right_container_jaw_joint_controller/command',
        'selector_frame_gripper_joint': '/selector_frame_gripper_joint_controller/command',
        'selector_frame_picker_frame_joint': '/selector_frame_picker_frame_joint_controller/command',
        'picker_frame_picker_rail_joint': '/picker_frame_picker_rail_joint_controller/command',
        'picker_rail_picker_base_joint': '/picker_rail_picker_base_joint_controller/command',
        'picker_base_picker_jaw_joint': '/picker_base_picker_jaw_joint_controller/command',
    }

    # Joint limits (min, max) from ros2_control.xacro
    JOINT_LIMITS = {
        'base_main_frame_joint': (0.0, 4.0),
        'main_frame_selector_frame_joint': (-0.01, 1.5),
        'selector_left_container_jaw_joint': (-0.2, 0.2),
        'selector_right_container_jaw_joint': (-0.2, 0.2),
        'selector_frame_gripper_joint': (-0.4, 0.4),
        'selector_frame_picker_frame_joint': (-0.01, 0.3),
        'picker_frame_picker_rail_joint': (-0.3, 0.3),
        'picker_rail_picker_base_joint': (0.0, 0.12),
        'picker_base_picker_jaw_joint': (0.0, 0.2),
    }

    def __init__(self, node: Node):
        """Initialize with reference to ROS2 node"""
        self._node = node
        self._joint_positions: dict = {}

        # Create publishers for each controller
        self._publishers = {}
        for joint_name, topic in self.JOINT_CONTROLLERS.items():
            self._publishers[joint_name] = self._node.create_publisher(
                Float64, topic, 10
            )

        # Subscribe to joint states
        self._node.create_subscription(
            JointState, '/joint_states', self._joint_state_callback, 10
        )

    def _joint_state_callback(self, msg: JointState):
        """Update current joint positions from /joint_states"""
        for i, name in enumerate(msg.name):
            self._joint_positions[name] = msg.position[i]

    def command_joint(self, joint_name: str, position: float) -> bool:
        """
        Command a single joint to target position.
        Returns False if joint unknown or position out of limits.
        """
        if joint_name not in self.JOINT_CONTROLLERS:
            self._node.get_logger().warning(f'Unknown joint: {joint_name}')
            return False

        min_limit, max_limit = self.JOINT_LIMITS[joint_name]
        if position < min_limit or position > max_limit:
            self._node.get_logger().warning(
                f'Position {position} out of limits [{min_limit}, {max_limit}] for {joint_name}'
            )
            return False

        msg = Float64()
        msg.data = position
        self._publishers[joint_name].publish(msg)
        return True

    def command_joint_group(self, joint_names: List[str], positions: List[float]) -> bool:
        """
        Command multiple joints simultaneously.
        Returns False if any joint fails validation.
        """
        if len(joint_names) != len(positions):
            self._node.get_logger().warning('joint_names and positions must have same length')
            return False

        # Validate all first
        for joint_name, position in zip(joint_names, positions):
            if joint_name not in self.JOINT_CONTROLLERS:
                self._node.get_logger().warning(f'Unknown joint: {joint_name}')
                return False
            min_limit, max_limit = self.JOINT_LIMITS[joint_name]
            if position < min_limit or position > max_limit:
                self._node.get_logger().warning(
                    f'Position {position} out of limits [{min_limit}, {max_limit}] for {joint_name}'
                )
                return False

        # Command all joints
        for joint_name, position in zip(joint_names, positions):
            msg = Float64()
            msg.data = position
            self._publishers[joint_name].publish(msg)

        return True

    def get_joint_position(self, joint_name: str) -> Optional[float]:
        """
        Get current joint position from /joint_states.
        Returns None if joint not found.
        """
        return self._joint_positions.get(joint_name)

    def get_joint_limits(self, joint_name: str) -> Tuple[float, float]:
        """Return (min_limit, max_limit) for joint"""
        return self.JOINT_LIMITS.get(joint_name, (0.0, 0.0))
```

### Launch File Update Pattern

```python
from launch.conditions import IfCondition

# Add to generate_launch_description():

# Declare use_sim_time argument
use_sim_time_arg = DeclareLaunchArgument(
    'use_sim_time',
    default_value='true',
    description='Use simulation time (true for Gazebo, false for hardware)'
)
use_sim_time = LaunchConfiguration('use_sim_time')

# Update virtual_limit_switches_node with condition
virtual_limit_switches_node = TimerAction(
    period=3.0,
    actions=[
        Node(
            package='manipulator_control',
            executable='virtual_limit_switches.py',
            name='virtual_limit_switches',
            output='screen',
            parameters=[
                {'config_file': limit_switches_config},
                {'use_sim_time': use_sim_time}
            ],
            condition=IfCondition(use_sim_time)  # Only launch in simulation
        )
    ]
)
```

### Technical Constraints

- ROS2 Jazzy + Gazebo Harmonic compatibility
- Must work with existing ForwardCommandControllers
- Publishers created once on initialization (not per-call)
- Joint state subscription uses QoS depth 10
- All methods non-blocking (publishers are async)

### Test Requirements (MANDATORY after implementation)

**1. Build Verification:**
```bash
cd /home/robo/robo/ya_robot_manipulator/ros2_ws
colcon build --packages-select manipulator_control
echo "Exit code: $?"
```

**2. Launch Simulation (Terminal 1):**
```bash
source /home/robo/robo/ya_robot_manipulator/ros2_ws/install/setup.bash
ros2 launch manipulator_control manipulator_simulation.launch.py
```

**3. Test Launch File use_sim_time Argument:**
```bash
# Default (use_sim_time=true) - virtual limit switches should launch
ros2 launch manipulator_control manipulator_simulation.launch.py
# Check node is running:
ros2 node list | grep virtual_limit_switches
# Expected: /virtual_limit_switches

# With use_sim_time=false - virtual limit switches should NOT launch
ros2 launch manipulator_control manipulator_simulation.launch.py use_sim_time:=false
# Check node is NOT running:
ros2 node list | grep virtual_limit_switches
# Expected: (empty - no output)
```

**4. Test ControllerInterface - Interactive Python Test:**
```bash
source /home/robo/robo/ya_robot_manipulator/ros2_ws/install/setup.bash
python3 << 'EOF'
import rclpy
from rclpy.node import Node
import sys
sys.path.insert(0, '/home/robo/robo/ya_robot_manipulator/ros2_ws/src/manipulator_control/src')
from controller_interface import ControllerInterface
import time

rclpy.init()
node = Node('test_controller_interface')
ci = ControllerInterface(node)

# Spin briefly to receive joint states
executor = rclpy.get_global_executor()
executor.add_node(node)
for _ in range(20):
    executor.spin_once(timeout_sec=0.1)

print("=== ControllerInterface Test ===\n")

# Test 1: Get joint position
print("1. Testing get_joint_position():")
pos = ci.get_joint_position('base_main_frame_joint')
print(f"   base_main_frame_joint position: {pos}")
if pos is not None:
    print("   PASS: Joint position retrieved")
else:
    print("   FAIL: Joint position is None (is simulation running?)")

# Test 2: Get joint limits
print("\n2. Testing get_joint_limits():")
limits = ci.get_joint_limits('base_main_frame_joint')
print(f"   base_main_frame_joint limits: {limits}")
if limits == (0.0, 4.0):
    print("   PASS: Correct limits returned")
else:
    print("   FAIL: Unexpected limits")

# Test 3: Command joint (valid)
print("\n3. Testing command_joint() with valid position:")
result = ci.command_joint('base_main_frame_joint', 1.0)
print(f"   command_joint('base_main_frame_joint', 1.0) = {result}")
if result:
    print("   PASS: Valid command accepted")
else:
    print("   FAIL: Valid command rejected")

# Test 4: Command joint (invalid - out of limits)
print("\n4. Testing command_joint() with invalid position (out of limits):")
result = ci.command_joint('base_main_frame_joint', 10.0)
print(f"   command_joint('base_main_frame_joint', 10.0) = {result}")
if not result:
    print("   PASS: Invalid command rejected")
else:
    print("   FAIL: Invalid command accepted")

# Test 5: Command joint (invalid - unknown joint)
print("\n5. Testing command_joint() with unknown joint:")
result = ci.command_joint('nonexistent_joint', 1.0)
print(f"   command_joint('nonexistent_joint', 1.0) = {result}")
if not result:
    print("   PASS: Unknown joint rejected")
else:
    print("   FAIL: Unknown joint accepted")

# Test 6: Command joint group (valid)
print("\n6. Testing command_joint_group() with valid positions:")
result = ci.command_joint_group(
    ['base_main_frame_joint', 'main_frame_selector_frame_joint'],
    [0.5, 0.3]
)
print(f"   command_joint_group(['base_main_frame_joint', 'main_frame_selector_frame_joint'], [0.5, 0.3]) = {result}")
if result:
    print("   PASS: Valid group command accepted")
else:
    print("   FAIL: Valid group command rejected")

# Test 7: Command joint group (invalid - one out of limits)
print("\n7. Testing command_joint_group() with one invalid position:")
result = ci.command_joint_group(
    ['base_main_frame_joint', 'main_frame_selector_frame_joint'],
    [0.5, 10.0]  # second position out of limits
)
print(f"   command_joint_group(['base_main_frame_joint', 'main_frame_selector_frame_joint'], [0.5, 10.0]) = {result}")
if not result:
    print("   PASS: Invalid group command rejected")
else:
    print("   FAIL: Invalid group command accepted")

# Test 8: Verify joint actually moved
print("\n8. Verifying joint movement (wait 2s for Gazebo):")
time.sleep(2)
for _ in range(10):
    executor.spin_once(timeout_sec=0.1)
new_pos = ci.get_joint_position('base_main_frame_joint')
print(f"   base_main_frame_joint new position: {new_pos}")
if new_pos is not None and abs(new_pos - 0.5) < 0.1:
    print("   PASS: Joint moved to target position")
else:
    print("   WARN: Joint position not at target (may need more time)")

node.destroy_node()
rclpy.shutdown()

print("\n=== Test Complete ===")
EOF
```

**5. Test All 9 Joints Command:**
```bash
source /home/robo/robo/ya_robot_manipulator/ros2_ws/install/setup.bash
python3 << 'EOF'
import rclpy
from rclpy.node import Node
import sys
sys.path.insert(0, '/home/robo/robo/ya_robot_manipulator/ros2_ws/src/manipulator_control/src')
from controller_interface import ControllerInterface
import time

rclpy.init()
node = Node('test_all_joints')
ci = ControllerInterface(node)

# Spin to initialize
executor = rclpy.get_global_executor()
executor.add_node(node)
for _ in range(20):
    executor.spin_once(timeout_sec=0.1)

print("=== Testing All 9 Joints ===\n")

# Test positions (within limits for each joint)
test_positions = {
    'base_main_frame_joint': 1.0,
    'main_frame_selector_frame_joint': 0.5,
    'selector_left_container_jaw_joint': 0.05,
    'selector_right_container_jaw_joint': -0.05,
    'selector_frame_gripper_joint': 0.1,
    'selector_frame_picker_frame_joint': 0.1,
    'picker_frame_picker_rail_joint': 0.1,
    'picker_rail_picker_base_joint': 0.05,
    'picker_base_picker_jaw_joint': 0.1,
}

all_passed = True
for joint_name, position in test_positions.items():
    result = ci.command_joint(joint_name, position)
    status = "PASS" if result else "FAIL"
    if not result:
        all_passed = False
    print(f"  {status}: {joint_name} -> {position}")

print(f"\nOverall: {'ALL PASSED' if all_passed else 'SOME FAILED'}")

# Wait and verify positions
print("\nWaiting 3s for joints to move...")
time.sleep(3)
for _ in range(30):
    executor.spin_once(timeout_sec=0.1)

print("\nCurrent joint positions:")
for joint_name in test_positions.keys():
    pos = ci.get_joint_position(joint_name)
    print(f"  {joint_name}: {pos}")

node.destroy_node()
rclpy.shutdown()
EOF
```

**6. Verify Limit Validation:**
```bash
source /home/robo/robo/ya_robot_manipulator/ros2_ws/install/setup.bash
python3 << 'EOF'
import rclpy
from rclpy.node import Node
import sys
sys.path.insert(0, '/home/robo/robo/ya_robot_manipulator/ros2_ws/src/manipulator_control/src')
from controller_interface import ControllerInterface

rclpy.init()
node = Node('test_limits')
ci = ControllerInterface(node)

print("=== Testing Joint Limit Validation ===\n")

# Test each joint with out-of-range values
test_cases = [
    ('base_main_frame_joint', -1.0, 'below min'),
    ('base_main_frame_joint', 5.0, 'above max'),
    ('picker_rail_picker_base_joint', -0.1, 'below min'),
    ('picker_rail_picker_base_joint', 0.2, 'above max'),
    ('selector_frame_gripper_joint', -0.5, 'below min'),
    ('selector_frame_gripper_joint', 0.5, 'above max'),
]

all_rejected = True
for joint_name, position, description in test_cases:
    result = ci.command_joint(joint_name, position)
    if result:
        all_rejected = False
        print(f"  FAIL: {joint_name} {description} ({position}) was accepted!")
    else:
        print(f"  PASS: {joint_name} {description} ({position}) was rejected")

print(f"\nOverall: {'ALL LIMITS ENFORCED' if all_rejected else 'LIMIT VALIDATION FAILED'}")

node.destroy_node()
rclpy.shutdown()
EOF
```

### References

- [Source: docs/sprint-artifacts/tech-spec-epic-2.md#APIs-and-Interfaces] ControllerInterface specification
- [Source: ros2_ws/src/manipulator_description/config/manipulator_controllers.yaml] Controller definitions
- [Source: ros2_ws/src/manipulator_description/urdf/manipulator/ros2_control.xacro] Joint limits

## Dev Agent Record

### Context Reference

- `docs/sprint-artifacts/2-2-create-controller-interface-utility.context.xml`

### Agent Model Used

(To be filled by dev agent)

### Debug Log References

(To be filled by dev agent)

### Completion Notes List

(To be filled by dev agent)

### File List

**To Create:**
- ros2_ws/src/manipulator_control/src/controller_interface.py

**To Modify:**
- ros2_ws/src/manipulator_control/launch/manipulator_simulation.launch.py (add use_sim_time)
- ros2_ws/src/manipulator_control/CMakeLists.txt (if needed for installation)

## Change Log

| Date | Change | Author |
|------|--------|--------|
| 2025-11-25 | Story drafted from tech-spec-epic-2.md and epics.md | SM Agent (Bob) |
