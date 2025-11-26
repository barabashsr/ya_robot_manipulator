# Story 2.3.2: Update joy_control Package for Trajectory Controllers

Status: ready-for-dev

## Story

As a **developer**,
I want **the joy_control package to use JointTrajectoryController for motion joints and load limits from manipulator_params.yaml**,
So that **joystick control provides smooth motion compatible with the new architecture and follows the single-source-of-truth principle**.

## Acceptance Criteria

1. **AC-1:** 7 motion joints use `FollowJointTrajectory` action clients (smooth interpolation)
2. **AC-2:** 2 container jaw joints continue using `/commands` topic publishers (instant response)
3. **AC-3:** Joint limits loaded from `manipulator_params.yaml` at runtime (no duplication)
4. **AC-4:** Joint velocities loaded from `manipulator_params.yaml` for duration calculation
5. **AC-5:** Dual-timer architecture: Joy callback (20Hz) + Trajectory timer (10Hz)
6. **AC-6:** Streaming trajectory goals provide smooth motion (no jerky steps)
7. **AC-7:** Previous trajectory goal preempted when new goal sent (responsive control)
8. **AC-8:** All 9 joints remain controllable via joystick
9. **AC-9:** Container jaws respond instantly (ForwardCommand behavior unchanged)
10. **AC-10:** Config file updated: limits removed, trajectory params added
11. **AC-11:** README.md updated with new architecture documentation
12. **AC-12:** All tests pass for all 9 joints

## Key Concepts & Core Logic

### Why This Migration?

| Issue with Current Implementation | Solution |
|----------------------------------|----------|
| Uses ForwardCommandController topics | Use JointTrajectoryController actions for 7 motion joints |
| Instant position jumps (jerky) | Trajectory controller interpolates smoothly |
| Hardcoded limits in config | Load from `manipulator_params.yaml` (single source of truth) |
| Duplicated limit values | Remove `limits` field from config |
| No velocity profile | Calculate duration from `limits.velocity` in params |

### Smooth Motion Architecture (Streaming Trajectory Goals)

```
┌─────────────────────────────────────────────────────────────────────────┐
│                    SMOOTH JOYSTICK CONTROL ARCHITECTURE                  │
├─────────────────────────────────────────────────────────────────────────┤
│                                                                          │
│  Joy Callback (20Hz)                   Trajectory Timer (10Hz)          │
│        │                                       │                         │
│        ▼                                       ▼                         │
│  Read joystick axes                   Send accumulated targets          │
│        │                              as trajectory goals               │
│        ▼                                       │                         │
│  Calculate velocity                            ▼                         │
│  (axis × velocity_scale)              Trajectory controller             │
│        │                              interpolates smoothly             │
│        ▼                              (spline interpolation)            │
│  Accumulate target position                    │                         │
│  (current + velocity × dt)                     ▼                         │
│        │                              Robot moves smoothly              │
│        ▼                              (no jerky steps)                  │
│  Clamp to limits                                                        │
│  (from manipulator_params.yaml)                                         │
│                                                                          │
└─────────────────────────────────────────────────────────────────────────┘
```

### Joint Classification

```python
# Motion joints use trajectory controller (action clients) - SMOOTH
TRAJECTORY_JOINTS = frozenset([
    'base_main_frame_joint',
    'main_frame_selector_frame_joint',
    'selector_frame_gripper_joint',
    'selector_frame_picker_frame_joint',
    'picker_frame_picker_rail_joint',
    'picker_rail_picker_base_joint',
    'picker_base_picker_jaw_joint'
])

# Container jaws use forward command controller (topic publishers) - INSTANT
FORWARD_COMMAND_JOINTS = frozenset([
    'selector_left_container_jaw_joint',
    'selector_right_container_jaw_joint'
])
```

### Loading Limits from manipulator_params.yaml

**Current (WRONG - duplicated limits):**
```yaml
# manipulator_joy_config.yaml - DUPLICATED!
axis_mappings.base_main_frame.limits: [0.0, 4.0]  # Different from soft limits!
axis_mappings.main_frame_selector_frame.limits: [0.0, 1.5]  # Wrong!
```

**After (CORRECT - load at runtime):**
```python
def _load_joint_limits_from_params(self) -> dict:
    """Load limits and velocities from manipulator_params.yaml"""
    pkg_path = get_package_share_directory('manipulator_description')
    params_file = os.path.join(pkg_path, 'config', 'manipulator_params.yaml')

    with open(params_file, 'r') as f:
        params = yaml.safe_load(f)

    limits = {}
    for assembly_name, assembly in params.items():
        if not isinstance(assembly, dict):
            continue
        for key, value in assembly.items():
            if isinstance(value, dict) and 'safety_controller' in value:
                sc = value['safety_controller']
                limits[key] = {
                    'min': sc['soft_lower'],
                    'max': sc['soft_upper'],
                    'velocity': value.get('limits', {}).get('velocity', 1.0),
                }
    return limits
```

**Correct Soft Limits (from manipulator_params.yaml):**

| Joint | Current Config (WRONG) | Soft Limits (CORRECT) |
|-------|------------------------|----------------------|
| base_main_frame_joint | [0.0, 4.0] | [0.1, 3.9] |
| main_frame_selector_frame_joint | [0.0, 1.5] | [0.05, 1.45] |
| selector_frame_gripper_joint | [-0.4, 0.4] | [-0.39, 0.39] |
| selector_frame_picker_frame_joint | [0.0, 0.3] | [0.005, 0.29] |
| picker_frame_picker_rail_joint | [-0.3, 0.3] | [-0.29, 0.29] |
| picker_rail_picker_base_joint | [0.0, 0.25] | [0.005, 0.24] |
| picker_base_picker_jaw_joint | [0.0, 0.2] | [0.005, 0.19] |
| selector_left_container_jaw_joint | [-0.2, 0.2] | [-0.19, 0.19] |
| selector_right_container_jaw_joint | [-0.2, 0.2] | [-0.19, 0.19] |

### Duration Calculation

Trajectory duration is calculated from distance and max velocity:

```python
def _calculate_duration(self, joint_name: str, current: float, target: float) -> float:
    """Calculate trajectory duration based on distance and max velocity."""
    distance = abs(target - current)
    max_velocity = self.joint_limits[joint_name]['velocity']  # From manipulator_params.yaml

    duration = distance / max_velocity if max_velocity > 0 else 0.5
    duration = max(self.trajectory_duration_min, min(self.trajectory_duration_max, duration))

    return duration
```

### Trajectory Goal Preemption

For responsive joystick control, previous goals are cancelled when new goals arrive:

```python
def _send_trajectory_goal(self, joint_name: str, target: float):
    """Send trajectory goal with preemption of previous goal."""
    client = self.trajectory_clients.get(joint_name)
    if not client or not client.server_is_ready():
        return

    # Cancel previous goal for this joint (preemption)
    if joint_name in self.current_goal_handles:
        old_handle = self.current_goal_handles[joint_name]
        if old_handle is not None:
            old_handle.cancel_goal_async()

    # Calculate duration and send new goal
    current = self.joint_positions.get(joint_name, target)
    duration = self._calculate_duration(joint_name, current, target)

    goal = self._build_trajectory_goal(joint_name, target, duration)
    future = client.send_goal_async(goal)
    future.add_done_callback(lambda f, jn=joint_name: self._on_goal_response(f, jn))
```

## Implementation Details

### Core Class Structure

```python
#!/usr/bin/env python3
"""
Configurable Joystick Controller Node with Trajectory Controller Support

Supports hybrid controller architecture:
- 7 motion joints: JointTrajectoryController (smooth interpolation)
- 2 container jaws: ForwardCommandController (instant response)

Joint limits loaded from manipulator_params.yaml (single source of truth).
"""

import os
import yaml
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from sensor_msgs.msg import Joy, JointState
from std_msgs.msg import Float64MultiArray
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from ament_index_python.packages import get_package_share_directory


class ConfigurableJoyController(Node):
    """Maps joystick inputs to robot joint controllers with smooth trajectory control"""

    # Joint classification
    TRAJECTORY_JOINTS = frozenset([
        'base_main_frame_joint',
        'main_frame_selector_frame_joint',
        'selector_frame_gripper_joint',
        'selector_frame_picker_frame_joint',
        'picker_frame_picker_rail_joint',
        'picker_rail_picker_base_joint',
        'picker_base_picker_jaw_joint'
    ])

    FORWARD_COMMAND_JOINTS = frozenset([
        'selector_left_container_jaw_joint',
        'selector_right_container_jaw_joint'
    ])

    def __init__(self):
        super().__init__('joy_controller_node')

        # Declare parameters
        self.declare_parameter('update_rate', 20.0)
        self.declare_parameter('trajectory_update_rate', 10.0)
        self.declare_parameter('trajectory_duration_min', 0.05)
        self.declare_parameter('trajectory_duration_max', 2.0)
        self.declare_parameter('scale_linear', 0.5)
        self.declare_parameter('deadzone', 0.1)
        self.declare_parameter('enable_button', 4)
        self.declare_parameter('enable_turbo_button', 5)

        # Get parameters
        self.update_rate = self.get_parameter('update_rate').value
        self.trajectory_update_rate = self.get_parameter('trajectory_update_rate').value
        self.trajectory_duration_min = self.get_parameter('trajectory_duration_min').value
        self.trajectory_duration_max = self.get_parameter('trajectory_duration_max').value
        self.scale_linear = self.get_parameter('scale_linear').value
        self.deadzone = self.get_parameter('deadzone').value
        self.enable_button = self.get_parameter('enable_button').value
        self.enable_turbo_button = self.get_parameter('enable_turbo_button').value

        # Load joint limits from manipulator_params.yaml (SINGLE SOURCE OF TRUTH)
        self.joint_limits = self._load_joint_limits_from_params()
        self.get_logger().info(f'Loaded limits for {len(self.joint_limits)} joints from manipulator_params.yaml')

        # Control state
        self.enabled = False
        self.turbo_enabled = False
        self.ever_enabled = False
        self.joint_states_received = False

        # Joint positions
        self.joint_positions = {}
        self.pending_targets = {}
        self.last_sent_targets = {}
        self.current_goal_handles = {}

        # Load axis/button mappings from parameters
        self.axis_mappings = {}
        self.button_actions = {}
        self._load_axis_mappings()
        self._load_button_actions()

        # Create action clients for trajectory joints
        self.trajectory_clients = {}
        for joint in self.TRAJECTORY_JOINTS:
            if joint in self.joint_limits:
                action_name = f'/{joint}_controller/follow_joint_trajectory'
                self.trajectory_clients[joint] = ActionClient(
                    self, FollowJointTrajectory, action_name
                )
                self.get_logger().info(f'Created trajectory action client: {action_name}')

        # Create publishers for forward command joints (container jaws)
        self.forward_publishers = {}
        for joint in self.FORWARD_COMMAND_JOINTS:
            if joint in self.joint_limits:
                topic = f'/{joint}_controller/commands'
                self.forward_publishers[joint] = self.create_publisher(
                    Float64MultiArray, topic, 10
                )
                self.get_logger().info(f'Created forward command publisher: {topic}')

        # Subscribe to joint states
        self.joint_state_sub = self.create_subscription(
            JointState, '/joint_states', self._joint_state_cb, 10
        )

        # Subscribe to joystick
        self.joy_sub = self.create_subscription(
            Joy, 'joy', self._joy_callback, 10
        )

        # Dual timer architecture
        # Joy processing happens in callback (triggered by /joy messages ~20Hz)
        # Trajectory goals sent at controlled rate
        self.trajectory_timer = self.create_timer(
            1.0 / self.trajectory_update_rate,
            self._send_trajectory_goals
        )

        self.get_logger().info('Joy Controller started with trajectory controller support')
        self.get_logger().info(f'  Trajectory joints: {len(self.trajectory_clients)}')
        self.get_logger().info(f'  Forward command joints: {len(self.forward_publishers)}')
        self.get_logger().info(f'  Hold button {self.enable_button} (L1) to enable control')

    def _load_joint_limits_from_params(self) -> dict:
        """Load limits and velocities from manipulator_params.yaml (SINGLE SOURCE OF TRUTH)"""
        try:
            pkg_path = get_package_share_directory('manipulator_description')
            params_file = os.path.join(pkg_path, 'config', 'manipulator_params.yaml')

            with open(params_file, 'r') as f:
                params = yaml.safe_load(f)

            limits = {}
            for assembly_name, assembly in params.items():
                if not isinstance(assembly, dict):
                    continue
                for key, value in assembly.items():
                    if isinstance(value, dict) and 'safety_controller' in value:
                        sc = value['safety_controller']
                        limits[key] = {
                            'min': sc['soft_lower'],
                            'max': sc['soft_upper'],
                            'velocity': value.get('limits', {}).get('velocity', 1.0),
                        }

            self.get_logger().info(f'Loaded joint limits from {params_file}')
            return limits

        except Exception as e:
            self.get_logger().error(f'Error loading joint limits: {e}')
            return {}

    def _joint_state_cb(self, msg: JointState):
        """Update joint positions from /joint_states"""
        for i, name in enumerate(msg.name):
            if i < len(msg.position):
                self.joint_positions[name] = msg.position[i]

        if not self.joint_states_received:
            self.joint_states_received = True
            self.get_logger().info('Initial joint positions received')

    def _joy_callback(self, msg: Joy):
        """Process joystick input - accumulate target positions"""
        # Check enable/turbo buttons
        if len(msg.buttons) > self.enable_button:
            self.enabled = msg.buttons[self.enable_button] == 1
            if self.enabled and not self.ever_enabled:
                self.ever_enabled = True
                self.get_logger().info('Control enabled')

        if len(msg.buttons) > self.enable_turbo_button:
            self.turbo_enabled = msg.buttons[self.enable_turbo_button] == 1

        if not self.enabled:
            return

        dt = 1.0 / self.update_rate
        turbo_mult = 2.0 if self.turbo_enabled else 1.0

        # Process axis mappings
        for mapping_name, mapping in self.axis_mappings.items():
            axis_idx = mapping.get('axis_index', -1)
            if axis_idx < 0 or axis_idx >= len(msg.axes):
                continue

            joint_name = mapping.get('joint_name')
            if not joint_name or joint_name not in self.joint_limits:
                continue

            # Read and filter axis value
            raw_value = msg.axes[axis_idx]
            axis_value = self._apply_deadzone(raw_value, axis_idx)

            if abs(axis_value) < 0.01:
                continue

            # Calculate velocity and target
            axis_value *= mapping.get('axis_scale', 1.0)
            velocity = axis_value * mapping.get('velocity_scale', 0.5) * self.scale_linear * turbo_mult

            current = self.joint_positions.get(joint_name, 0.0)
            target = current + velocity * dt

            # Clamp to limits from manipulator_params.yaml
            limits = self.joint_limits[joint_name]
            target = max(limits['min'], min(limits['max'], target))

            # Accumulate target
            self.pending_targets[joint_name] = target

        # Process button actions (for container jaws and picker jaw)
        self._process_button_actions(msg)

    def _send_trajectory_goals(self):
        """Send accumulated targets as trajectory goals (10Hz)"""
        if not self.enabled or not self.ever_enabled or not self.joint_states_received:
            return

        for joint_name, target in list(self.pending_targets.items()):
            # Skip if target hasn't changed significantly
            last_sent = self.last_sent_targets.get(joint_name)
            if last_sent is not None and abs(target - last_sent) < 0.001:
                continue

            if joint_name in self.TRAJECTORY_JOINTS:
                self._send_trajectory_goal(joint_name, target)
            elif joint_name in self.FORWARD_COMMAND_JOINTS:
                self._send_forward_command(joint_name, target)

            self.last_sent_targets[joint_name] = target

    def _send_trajectory_goal(self, joint_name: str, target: float):
        """Send trajectory goal with calculated duration"""
        client = self.trajectory_clients.get(joint_name)
        if not client or not client.server_is_ready():
            return

        # Cancel previous goal (preemption for responsiveness)
        if joint_name in self.current_goal_handles:
            old_handle = self.current_goal_handles[joint_name]
            if old_handle is not None:
                old_handle.cancel_goal_async()

        # Calculate duration from distance and max velocity
        current = self.joint_positions.get(joint_name, target)
        distance = abs(target - current)
        max_velocity = self.joint_limits[joint_name]['velocity']

        duration = distance / max_velocity if max_velocity > 0 else 0.5
        duration = max(self.trajectory_duration_min, min(self.trajectory_duration_max, duration))

        # Build and send trajectory goal
        goal = self._build_trajectory_goal(joint_name, target, duration)
        future = client.send_goal_async(goal)
        future.add_done_callback(lambda f, jn=joint_name: self._on_goal_response(f, jn))

    def _build_trajectory_goal(self, joint_name: str, target: float, duration: float):
        """Build FollowJointTrajectory goal message"""
        trajectory = JointTrajectory()
        trajectory.joint_names = [joint_name]

        point = JointTrajectoryPoint()
        point.positions = [target]
        point.velocities = [0.0]
        point.time_from_start = Duration(
            sec=int(duration),
            nanosec=int((duration % 1) * 1e9)
        )
        trajectory.points = [point]

        goal = FollowJointTrajectory.Goal()
        goal.trajectory = trajectory
        return goal

    def _on_goal_response(self, future, joint_name: str):
        """Store goal handle for potential preemption"""
        try:
            goal_handle = future.result()
            if goal_handle.accepted:
                self.current_goal_handles[joint_name] = goal_handle
        except Exception as e:
            self.get_logger().debug(f'Goal response error for {joint_name}: {e}')

    def _send_forward_command(self, joint_name: str, target: float):
        """Send forward command for container jaws (instant response)"""
        publisher = self.forward_publishers.get(joint_name)
        if publisher:
            msg = Float64MultiArray()
            msg.data = [target]
            publisher.publish(msg)

    def _apply_deadzone(self, value: float, axis_index: int) -> float:
        """Apply deadzone to axis value"""
        # D-Pad axes (6, 7) are discrete
        if axis_index in [6, 7]:
            return value if abs(value) >= 0.5 else 0.0

        if abs(value) < self.deadzone:
            return 0.0
        sign = 1.0 if value > 0 else -1.0
        return sign * (abs(value) - self.deadzone) / (1.0 - self.deadzone)

    def _process_button_actions(self, msg: Joy):
        """Process button actions for container jaws and picker jaw"""
        # Implementation similar to current, but using pending_targets
        # for trajectory joints and direct publish for forward command joints
        pass  # Simplified for story - full implementation in actual code
```

### Updated Config File Structure

```yaml
# manipulator_joy_config.yaml - UPDATED FOR TRAJECTORY CONTROLLERS
# ==================================================================

joy_controller_node:
  ros__parameters:
    # ============================================
    # Global Settings
    # ============================================
    update_rate: 20.0                # Joy callback rate (Hz)
    trajectory_update_rate: 10.0     # Trajectory goal send rate (Hz)
    scale_linear: 0.5
    deadzone: 0.1
    enable_button: 4                 # L1 - hold to enable
    enable_turbo_button: 5           # R1 - hold for 2x speed

    # Trajectory duration bounds (seconds)
    trajectory_duration_min: 0.05    # Prevents micro-movements
    trajectory_duration_max: 2.0     # Caps long motions

    # ============================================
    # Axis Mappings
    # ============================================
    # NOTE: limits field REMOVED - loaded from manipulator_params.yaml
    # NOTE: controller_topic field REMOVED - derived from joint_name

    # Base Railway - Left Stick Y
    axis_mappings.base_main_frame.joint_name: "base_main_frame_joint"
    axis_mappings.base_main_frame.axis_index: 1
    axis_mappings.base_main_frame.axis_scale: -1.0
    axis_mappings.base_main_frame.velocity_scale: 0.5

    # Selector Vertical - Right Stick Y
    axis_mappings.main_frame_selector_frame.joint_name: "main_frame_selector_frame_joint"
    axis_mappings.main_frame_selector_frame.axis_index: 4
    axis_mappings.main_frame_selector_frame.axis_scale: -1.0
    axis_mappings.main_frame_selector_frame.velocity_scale: 0.3

    # Gripper Y-axis - Right Stick X
    axis_mappings.selector_frame_gripper.joint_name: "selector_frame_gripper_joint"
    axis_mappings.selector_frame_gripper.axis_index: 3
    axis_mappings.selector_frame_gripper.axis_scale: -1.0
    axis_mappings.selector_frame_gripper.velocity_scale: 0.3

    # Picker Vertical - D-Pad Y
    axis_mappings.selector_frame_picker_frame.joint_name: "selector_frame_picker_frame_joint"
    axis_mappings.selector_frame_picker_frame.axis_index: 7
    axis_mappings.selector_frame_picker_frame.axis_scale: 1.0
    axis_mappings.selector_frame_picker_frame.velocity_scale: 0.2

    # Picker Y-axis - D-Pad X
    axis_mappings.picker_frame_picker_rail.joint_name: "picker_frame_picker_rail_joint"
    axis_mappings.picker_frame_picker_rail.axis_index: 6
    axis_mappings.picker_frame_picker_rail.axis_scale: -1.0
    axis_mappings.picker_frame_picker_rail.velocity_scale: 0.25

    # Picker X-axis slider - Left Stick X
    axis_mappings.picker_rail_picker_base.joint_name: "picker_rail_picker_base_joint"
    axis_mappings.picker_rail_picker_base.axis_index: 0
    axis_mappings.picker_rail_picker_base.axis_scale: -1.0
    axis_mappings.picker_rail_picker_base.velocity_scale: 0.25

    # Picker Jaw - Button controlled (axis_index: -1)
    axis_mappings.picker_base_picker_jaw.joint_name: "picker_base_picker_jaw_joint"
    axis_mappings.picker_base_picker_jaw.axis_index: -1
    axis_mappings.picker_base_picker_jaw.axis_scale: 1.0
    axis_mappings.picker_base_picker_jaw.velocity_scale: 0.2

    # Container Jaws - Button controlled, ForwardCommand
    axis_mappings.selector_left_container_jaw.joint_name: "selector_left_container_jaw_joint"
    axis_mappings.selector_left_container_jaw.axis_index: -1
    axis_mappings.selector_left_container_jaw.velocity_scale: 0.3

    axis_mappings.selector_right_container_jaw.joint_name: "selector_right_container_jaw_joint"
    axis_mappings.selector_right_container_jaw.axis_index: -1
    axis_mappings.selector_right_container_jaw.velocity_scale: 0.3

    # ============================================
    # Button Actions (unchanged from current)
    # ============================================
    # ... (same as current config)
```

## Tasks / Subtasks

- [ ] Task 1: Update joy_controller_node.py for Dual-Mode Control (AC: 1-9)
  - [ ] 1.1 Add `TRAJECTORY_JOINTS` and `FORWARD_COMMAND_JOINTS` constants
  - [ ] 1.2 Add `_load_joint_limits_from_params()` method
  - [ ] 1.3 Create ActionClients for 7 trajectory joints in `__init__()`
  - [ ] 1.4 Create Publishers only for 2 forward command joints
  - [ ] 1.5 Add trajectory timer (10Hz) for streaming goals
  - [ ] 1.6 Implement `_send_trajectory_goal()` with duration calculation
  - [ ] 1.7 Implement `_build_trajectory_goal()` helper
  - [ ] 1.8 Implement `_on_goal_response()` for goal handle storage
  - [ ] 1.9 Implement goal preemption (cancel previous goal)
  - [ ] 1.10 Update axis processing to use `pending_targets`
  - [ ] 1.11 Route button actions to correct controller type

- [ ] Task 2: Update Configuration File (AC: 10)
  - [ ] 2.1 Remove `limits` field from all axis_mappings
  - [ ] 2.2 Remove `controller_topic` field (derive from joint_name)
  - [ ] 2.3 Add `joint_name` field to each mapping
  - [ ] 2.4 Add `trajectory_update_rate` parameter
  - [ ] 2.5 Add `trajectory_duration_min` and `trajectory_duration_max` parameters

- [ ] Task 3: Update Package Dependencies (AC: 1)
  - [ ] 3.1 Add `control_msgs` to package.xml
  - [ ] 3.2 Verify CMakeLists.txt dependencies

- [ ] Task 4: Update README.md Documentation (AC: 11)
  - [ ] 4.1 Document trajectory controller architecture
  - [ ] 4.2 Document joint classification (trajectory vs forward)
  - [ ] 4.3 Document single-source-of-truth for limits
  - [ ] 4.4 Update architecture diagram
  - [ ] 4.5 Document new parameters
  - [ ] 4.6 Add troubleshooting for trajectory issues

- [ ] Task 5: Developer Self-Validation - MANDATORY (AC: 12)
  - [ ] 5.1 Execute Phase 1: Build Verification
  - [ ] 5.2 Execute Phase 2: Action Client Verification
  - [ ] 5.3 Execute Phase 3: All 7 Trajectory Joints Motion Test
  - [ ] 5.4 Execute Phase 4: Both Container Jaws Test
  - [ ] 5.5 Execute Phase 5: Smooth Motion Visual Verification
  - [ ] 5.6 Execute Phase 6: Limits Verification (match manipulator_params.yaml)
  - [ ] 5.7 Execute Phase 7: Button Actions Test
  - [ ] 5.8 Document ALL test results in Dev Agent Record

## Dev Notes

### Learnings from Previous Stories

**From Story 2.3.1 - Trajectory Controller Migration (Status: done):**
- 7 motion joints now use JointTrajectoryController
- 2 container jaws remain ForwardCommandController
- Action name pattern: `/{joint}_controller/follow_joint_trajectory`
- Topic name pattern: `/{joint}_controller/commands` (forward command only)
- Duration calculation: `distance / max_velocity`

**From Story 2.2 - ControllerInterface (Status: done):**
- Joint limits loaded from `manipulator_params.yaml`
- Parsing pattern for safety_controller soft limits
- Velocity available from `limits.velocity`

### Configuration Reuse Policy

**USE (no changes):**
- `manipulator_params.yaml` - joint limits and velocities (SINGLE SOURCE OF TRUTH)

**MODIFY:**
- `joy_control/scripts/joy_controller_node.py` - major rewrite for trajectory support
- `joy_control/config/manipulator_joy_config.yaml` - remove limits, add trajectory params
- `joy_control/package.xml` - add control_msgs dependency
- `joy_control/README.md` - document new architecture

### Project Structure

```
ros2_ws/src/joy_control/
├── scripts/
│   └── joy_controller_node.py      # MODIFY - trajectory support
├── config/
│   └── manipulator_joy_config.yaml # MODIFY - remove limits
├── launch/
│   └── joy_control.launch.py       # No changes needed
├── package.xml                     # MODIFY - add control_msgs
└── README.md                       # MODIFY - document architecture
```

### References

- [Source: docs/architecture-ros2-control-v2-CORRECTIONS.md#Section-9.1] Joy control architecture
- [Source: docs/epics.md#Story-2.3.2] Acceptance criteria
- [Source: docs/sprint-artifacts/2-3-1-migrate-motion-joints-to-trajectory-controller.md] Trajectory migration

## Test Requirements (MANDATORY)

**CRITICAL:** Developer MUST execute ALL tests below and document results before marking story complete.

### Phase 1: Build Verification

```bash
cd /home/robo/robo/ya_robot_manipulator/ros2_ws
colcon build --packages-select joy_control
echo "Build exit code: $?"
# Expected: 0
```

**Record Result:**
- [ ] Build successful (exit code 0)
- [ ] No errors or warnings

---

### Phase 2: Action Client Verification

```bash
# Launch simulation
source /home/robo/robo/ya_robot_manipulator/ros2_ws/install/setup.bash
ros2 launch manipulator_control manipulator_simulation.launch.py enable_joy:=true &
sleep 15

# Verify joy controller node is running
ros2 node list | grep joy_controller
# Expected: /joy_controller_node

# Check node info for action clients
ros2 node info /joy_controller_node | grep -A 20 "Action Clients"
```

**Expected:** 7 action clients for trajectory joints:
```
Action Clients:
  /base_main_frame_joint_controller/follow_joint_trajectory
  /main_frame_selector_frame_joint_controller/follow_joint_trajectory
  /selector_frame_gripper_joint_controller/follow_joint_trajectory
  /selector_frame_picker_frame_joint_controller/follow_joint_trajectory
  /picker_frame_picker_rail_joint_controller/follow_joint_trajectory
  /picker_rail_picker_base_joint_controller/follow_joint_trajectory
  /picker_base_picker_jaw_joint_controller/follow_joint_trajectory
```

**Record Result:**
- [ ] All 7 trajectory action clients created

---

### Phase 3: All 7 Trajectory Joints Motion Test

**Connect joystick and test each joint for smooth motion:**

| Joint | Control | Test Action | Expected |
|-------|---------|-------------|----------|
| base_main_frame_joint | Left Stick Y | Push up/down | Smooth X-axis motion |
| main_frame_selector_frame_joint | Right Stick Y | Push up/down | Smooth Z-axis motion |
| selector_frame_gripper_joint | Right Stick X | Push left/right | Smooth Y-axis motion |
| selector_frame_picker_frame_joint | D-Pad Y | Press up/down | Smooth picker Z motion |
| picker_frame_picker_rail_joint | D-Pad X | Press left/right | Smooth picker Y motion |
| picker_rail_picker_base_joint | Left Stick X | Push left/right | Smooth picker X motion |
| picker_base_picker_jaw_joint | Triangle/Cross | Hold buttons | Smooth jaw motion |

**Test Script (save as /tmp/test_joy_trajectory.sh):**
```bash
#!/bin/bash
source /home/robo/robo/ya_robot_manipulator/ros2_ws/install/setup.bash

echo "=== Joy Control Trajectory Test ==="
echo "Prerequisites:"
echo "  1. Simulation running with enable_joy:=true"
echo "  2. Joystick connected"
echo ""
echo "Instructions:"
echo "  1. Hold L1 to enable control"
echo "  2. Test each joint for SMOOTH motion (no jerky steps)"
echo "  3. Verify turbo mode with R1"
echo ""

# Monitor trajectory actions being sent
echo "Monitoring trajectory actions (Ctrl+C to stop)..."
echo "You should see action goals being sent when moving joystick"
ros2 topic echo /base_main_frame_joint_controller/follow_joint_trajectory/_action/status --once &
sleep 1

echo ""
echo "Manual verification required - mark each as PASS/FAIL"
```

**Record Result:**
- [ ] base_main_frame_joint: smooth motion
- [ ] main_frame_selector_frame_joint: smooth motion
- [ ] selector_frame_gripper_joint: smooth motion
- [ ] selector_frame_picker_frame_joint: smooth motion
- [ ] picker_frame_picker_rail_joint: smooth motion
- [ ] picker_rail_picker_base_joint: smooth motion
- [ ] picker_base_picker_jaw_joint: smooth motion

---

### Phase 4: Both Container Jaws Test (ForwardCommand)

**Test container jaws for instant response (no interpolation delay):**

| Joint | Control | Test Action | Expected |
|-------|---------|-------------|----------|
| selector_left_container_jaw_joint | L2 button | Hold L2 | Instant open |
| selector_right_container_jaw_joint | L2 button | Hold L2 | Instant open (same time) |
| Both jaws | R2 button | Hold R2 | Instant close |

**Verify topic commands (not actions):**
```bash
# Should see commands published to topics, NOT actions
ros2 topic echo /selector_left_container_jaw_joint_controller/commands --once &
ros2 topic echo /selector_right_container_jaw_joint_controller/commands --once &

# Hold L2 and verify both topics receive messages
```

**Record Result:**
- [ ] selector_left_container_jaw_joint: instant response
- [ ] selector_right_container_jaw_joint: instant response
- [ ] Both jaws respond simultaneously

---

### Phase 5: Smooth Motion Visual Verification

**In Gazebo, visually verify:**

- [ ] Motion is smooth (no jerky steps)
- [ ] Velocity profile visible (accelerate → cruise → decelerate)
- [ ] Robot stops smoothly when releasing joystick
- [ ] Container jaws respond instantly (different from trajectory motion)
- [ ] Turbo mode (R1) increases speed but stays smooth

---

### Phase 6: Limits Verification

**Verify limits match manipulator_params.yaml soft limits:**

```bash
#!/bin/bash
# Test that joints stop at correct soft limits

echo "=== Limits Verification ==="
echo "Push each joint to its limit and verify position"

# Get current positions
ros2 topic echo /joint_states --once | grep -A 20 "position"
```

**Expected limits (from manipulator_params.yaml):**

| Joint | Min | Max | Test |
|-------|-----|-----|------|
| base_main_frame_joint | 0.1 | 3.9 | Push to both limits |
| main_frame_selector_frame_joint | 0.05 | 1.45 | Push to both limits |
| selector_frame_gripper_joint | -0.39 | 0.39 | Push to both limits |
| selector_frame_picker_frame_joint | 0.005 | 0.29 | Push to both limits |
| picker_frame_picker_rail_joint | -0.29 | 0.29 | Push to both limits |
| picker_rail_picker_base_joint | 0.005 | 0.24 | Push to both limits |
| picker_base_picker_jaw_joint | 0.005 | 0.19 | Push to both limits |
| selector_left_container_jaw_joint | -0.19 | 0.19 | Push to both limits |
| selector_right_container_jaw_joint | -0.19 | 0.19 | Push to both limits |

**Record Result:**
- [ ] All joints stop at correct soft limits
- [ ] No hardcoded limits in config being used

---

### Phase 7: Button Actions Test

**Test button-controlled joints:**

| Button | Action | Joint | Expected |
|--------|--------|-------|----------|
| Triangle | Hold | picker_base_picker_jaw_joint | Extend to 0.19 |
| Cross | Hold | picker_base_picker_jaw_joint | Retract to 0.005 |
| L2 | Hold | Both container jaws | Open to 0.19 |
| R2 | Hold | Both container jaws | Close to -0.19 |

**Record Result:**
- [ ] Triangle extends picker jaw smoothly
- [ ] Cross retracts picker jaw smoothly
- [ ] L2 opens both jaws instantly
- [ ] R2 closes both jaws instantly

---

### Phase 8: Comprehensive Test Script

```bash
#!/bin/bash
# Save as /tmp/test_story_2_3_2_complete.sh

source /home/robo/robo/ya_robot_manipulator/ros2_ws/install/setup.bash

echo "=============================================="
echo "  Story 2.3.2 Complete Validation Suite"
echo "=============================================="
echo ""

TOTAL_TESTS=0
PASSED_TESTS=0

record_test() {
    local test_name="$1"
    local result="$2"
    ((TOTAL_TESTS++))
    if [ $result -eq 0 ]; then
        echo "  ✓ $test_name"
        ((PASSED_TESTS++))
    else
        echo "  ✗ $test_name"
    fi
}

echo "=== Phase 1: Build Verification ==="
if colcon build --packages-select joy_control 2>/dev/null; then
    record_test "joy_control builds successfully" 0
else
    record_test "joy_control builds successfully" 1
fi

echo ""
echo "=== Phase 2: Node Verification ==="
if ros2 node list 2>/dev/null | grep -q "joy_controller"; then
    record_test "joy_controller_node running" 0
else
    record_test "joy_controller_node running" 1
fi

echo ""
echo "=== Phase 3: Config File Verification ==="
CONFIG_FILE="/home/robo/robo/ya_robot_manipulator/ros2_ws/src/joy_control/config/manipulator_joy_config.yaml"

# Check limits field is removed
if ! grep -q "\.limits:" "$CONFIG_FILE" 2>/dev/null; then
    record_test "limits field removed from config" 0
else
    record_test "limits field removed from config" 1
fi

# Check joint_name field exists
if grep -q "\.joint_name:" "$CONFIG_FILE" 2>/dev/null; then
    record_test "joint_name field added to config" 0
else
    record_test "joint_name field added to config" 1
fi

# Check trajectory_update_rate parameter
if grep -q "trajectory_update_rate:" "$CONFIG_FILE" 2>/dev/null; then
    record_test "trajectory_update_rate parameter added" 0
else
    record_test "trajectory_update_rate parameter added" 1
fi

echo ""
echo "=== Phase 4: Package.xml Verification ==="
PACKAGE_FILE="/home/robo/robo/ya_robot_manipulator/ros2_ws/src/joy_control/package.xml"
if grep -q "control_msgs" "$PACKAGE_FILE" 2>/dev/null; then
    record_test "control_msgs dependency added" 0
else
    record_test "control_msgs dependency added" 1
fi

echo ""
echo "=== Phase 5: README.md Verification ==="
README_FILE="/home/robo/robo/ya_robot_manipulator/ros2_ws/src/joy_control/README.md"
if grep -q "JointTrajectoryController\|trajectory" "$README_FILE" 2>/dev/null; then
    record_test "README documents trajectory architecture" 0
else
    record_test "README documents trajectory architecture" 1
fi

if grep -q "manipulator_params.yaml" "$README_FILE" 2>/dev/null; then
    record_test "README documents single source of truth" 0
else
    record_test "README documents single source of truth" 1
fi

echo ""
echo "=============================================="
echo "  SUMMARY: $PASSED_TESTS / $TOTAL_TESTS automated tests passed"
echo "=============================================="
echo ""
echo "  MANUAL TESTS REQUIRED:"
echo "  - Phase 3: All 7 trajectory joints smooth motion"
echo "  - Phase 4: Both container jaws instant response"
echo "  - Phase 5: Visual verification in Gazebo"
echo "  - Phase 6: Limits match manipulator_params.yaml"
echo "  - Phase 7: Button actions work correctly"
echo ""

if [ $PASSED_TESTS -eq $TOTAL_TESTS ]; then
    echo "  AUTOMATED TESTS PASSED"
    exit 0
else
    echo "  SOME AUTOMATED TESTS FAILED"
    exit 1
fi
```

---

## README.md Update Requirements (MANDATORY)

The developer MUST update `ros2_ws/src/joy_control/README.md` to include:

### Required Sections

1. **Architecture Overview** - Updated diagram showing trajectory vs forward command
2. **Controller Types** - Document which joints use which controller
3. **Single Source of Truth** - Explain limits loaded from manipulator_params.yaml
4. **New Parameters** - Document trajectory_update_rate, duration_min/max
5. **Troubleshooting** - Add trajectory-specific issues

### Required Content

```markdown
## Architecture (Updated for Trajectory Controllers)

The joy_control package now uses a hybrid controller architecture:

### Controller Types

| Joint Type | Controller | Interface | Behavior |
|------------|------------|-----------|----------|
| 7 motion joints | JointTrajectoryController | Action | Smooth interpolated motion |
| 2 container jaws | ForwardCommandController | Topic | Instant response |

### Motion Joints (Trajectory - Smooth)
- base_main_frame_joint
- main_frame_selector_frame_joint
- selector_frame_gripper_joint
- selector_frame_picker_frame_joint
- picker_frame_picker_rail_joint
- picker_rail_picker_base_joint
- picker_base_picker_jaw_joint

### Container Jaws (ForwardCommand - Instant)
- selector_left_container_jaw_joint
- selector_right_container_jaw_joint

## Joint Limits (Single Source of Truth)

Joint limits are loaded at runtime from `manipulator_params.yaml`.
**DO NOT** add limits to `manipulator_joy_config.yaml`.

```yaml
# WRONG - Do not duplicate limits
axis_mappings.base_main_frame.limits: [0.0, 4.0]  # REMOVE THIS

# CORRECT - limits loaded automatically from manipulator_params.yaml
axis_mappings.base_main_frame.joint_name: "base_main_frame_joint"
```

## New Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| trajectory_update_rate | 10.0 | Trajectory goal send rate (Hz) |
| trajectory_duration_min | 0.05 | Minimum trajectory duration (s) |
| trajectory_duration_max | 2.0 | Maximum trajectory duration (s) |

## Troubleshooting

### Motion is still jerky
- Verify trajectory controllers are active: `ros2 control list_controllers`
- Check trajectory_update_rate is not too high (10Hz recommended)

### Joint doesn't respond
- Check action server is available: `ros2 action list | grep follow_joint_trajectory`
- Verify joint_name in config matches actual joint name

### Limits seem wrong
- Limits are loaded from manipulator_params.yaml, not config file
- Check `ros2 topic echo /joint_states` for actual positions
```

---

## Dev Agent Record

### Context Reference

- Story Context XML: `docs/sprint-artifacts/2-3-2-update-joy-control-for-trajectory-controllers.context.xml`

### Agent Model Used

(To be filled by dev agent)

### Debug Log References

(To be filled by dev agent)

### Completion Notes List

- **AC-1**: [ ] 7 motion joints use FollowJointTrajectory action clients
- **AC-2**: [ ] 2 container jaws use /commands topic publishers
- **AC-3**: [ ] Limits loaded from manipulator_params.yaml
- **AC-4**: [ ] Velocities loaded from manipulator_params.yaml
- **AC-5**: [ ] Dual-timer architecture implemented
- **AC-6**: [ ] Streaming trajectory goals provide smooth motion
- **AC-7**: [ ] Goal preemption implemented
- **AC-8**: [ ] All 9 joints controllable
- **AC-9**: [ ] Container jaws respond instantly
- **AC-10**: [ ] Config file updated
- **AC-11**: [ ] README.md updated
- **AC-12**: [ ] All tests pass

### Test Results

| Phase | Test | Result | Notes |
|-------|------|--------|-------|
| 1 | Build | | |
| 2 | Action clients (7) | | |
| 3 | base_main_frame_joint smooth | | |
| 3 | main_frame_selector_frame_joint smooth | | |
| 3 | selector_frame_gripper_joint smooth | | |
| 3 | selector_frame_picker_frame_joint smooth | | |
| 3 | picker_frame_picker_rail_joint smooth | | |
| 3 | picker_rail_picker_base_joint smooth | | |
| 3 | picker_base_picker_jaw_joint smooth | | |
| 4 | selector_left_container_jaw_joint instant | | |
| 4 | selector_right_container_jaw_joint instant | | |
| 5 | Visual verification | | |
| 6 | Limits match params | | |
| 7 | Button actions | | |
| 8 | Comprehensive script | | |

### File List

- `ros2_ws/src/joy_control/scripts/joy_controller_node.py` (MODIFIED - major)
- `ros2_ws/src/joy_control/config/manipulator_joy_config.yaml` (MODIFIED)
- `ros2_ws/src/joy_control/package.xml` (MODIFIED)
- `ros2_ws/src/joy_control/README.md` (MODIFIED - mandatory)

## Change Log

| Date | Change | Author |
|------|--------|--------|
| 2025-11-26 | Story drafted with comprehensive implementation details, test requirements, and README update mandate | SM Agent (Bob) |
