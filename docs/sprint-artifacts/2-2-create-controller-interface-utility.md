# Story 2.2: Create Controller Interface Utility

Status: done

## Story

As a **developer**,
I want **a utility to send position commands to individual ForwardCommandController instances**,
so that **action servers can control joints without hardcoding controller topic names**.

## Acceptance Criteria

1. **AC-1:** ControllerInterface class exists at `ros2_ws/src/manipulator_control/src/controller_interface.py`
2. **AC-2:** Utility discovers and maps all 9 joint controllers from `manipulator_controllers.yaml`
3. **AC-3:** `command_joint(joint_name, position)` publishes Float64 to correct controller topic
4. **AC-4:** `command_joint_group(joint_names, positions)` commands multiple joints simultaneously
5. **AC-5:** `get_joint_position(joint_name)` returns current position from `/joint_states`
6. **AC-6:** Position commands are validated against joint limits before publishing
7. **AC-7:** Invalid commands return False with logged warning (not exception)
8. **AC-8:** Joint limits loaded from `ros2_control.xacro` or ROS2 parameters at runtime
9. **AC-9:** Unit tests exist at `test/test_controller_interface.py`
10. **AC-10:** Manual test script demonstrates all methods working in Gazebo

## Key Concepts & Core Logic

### What is ControllerInterface?

The ControllerInterface is a **utility class** (not a standalone node) that abstracts joint control. It provides a clean API for action servers to command joints without needing to know:
- Controller topic naming conventions
- Joint limit values
- How to construct Float64 messages

### Why Individual ForwardCommandControllers?

This manipulator uses **9 individual ForwardCommandControllers** (one per joint) instead of JointTrajectoryController. Each controller:
- Listens on: `/[joint_name]_controller/commands` (std_msgs/Float64MultiArray)
- Commands position directly (no trajectory interpolation)
- Simpler but requires external coordination for multi-joint motion

### Controller Topic Pattern (CORRECTED: /commands plural, Float64MultiArray)

```
Joint Name                              Controller Topic
-------------------------------------------------------
base_main_frame_joint                   /base_main_frame_joint_controller/commands
main_frame_selector_frame_joint         /main_frame_selector_frame_joint_controller/commands
selector_frame_gripper_joint            /selector_frame_gripper_joint_controller/commands
selector_frame_picker_frame_joint       /selector_frame_picker_frame_joint_controller/commands
picker_frame_picker_rail_joint          /picker_frame_picker_rail_joint_controller/commands
picker_rail_picker_base_joint           /picker_rail_picker_base_joint_controller/commands
picker_base_picker_jaw_joint            /picker_base_picker_jaw_joint_controller/commands
selector_left_container_jaw_joint       /selector_left_container_jaw_joint_controller/commands
selector_right_container_jaw_joint      /selector_right_container_jaw_joint_controller/commands
```

### Unified Limits Architecture

**Single Source of Truth:** `manipulator_params.yaml` defines ALL limits

| Limit Type | Source | Purpose |
|------------|--------|---------|
| **Hard Limits** | `manipulator_params.yaml` → URDF `<limit>` | Physical joint range |
| **Soft Limits** | `manipulator_params.yaml` → URDF `<safety_controller>` | Operational safety margin |
| **Controller Limits** | `manipulator_params.yaml` → ros2_control `<command_interface>` | **= Soft Limits** |

**CRITICAL:** Controller command_interface limits now use **soft limits** from `safety_controller` in `manipulator_params.yaml`. This ensures:
- Controllers enforce operational safety margins (not just physical limits)
- Single source of truth - change once, applies everywhere
- ControllerInterface validates against the same limits controllers use

### Soft Limits = Controller Limits (from manipulator_params.yaml)

| Joint | Hard Limits | **Soft Limits (= Controller Limits)** |
|-------|-------------|---------------------------------------|
| base_main_frame_joint | 0.0 → 4.0 | **0.1 → 3.9** |
| main_frame_selector_frame_joint | -0.01 → 1.5 | **0.05 → 1.45** |
| selector_frame_gripper_joint | -0.4 → 0.4 | **-0.39 → 0.39** |
| selector_frame_picker_frame_joint | -0.01 → 0.3 | **0.005 → 0.29** |
| picker_frame_picker_rail_joint | -0.3 → 0.3 | **-0.29 → 0.29** |
| picker_rail_picker_base_joint | 0.0 → 0.25 | **0.005 → 0.24** |
| picker_base_picker_jaw_joint | 0.0 → 0.2 | **0.005 → 0.19** |
| selector_left_container_jaw_joint | -0.2 → 0.2 | **-0.19 → 0.19** |
| selector_right_container_jaw_joint | -0.2 → 0.2 | **-0.19 → 0.19** |

**All 9 joints now have soft limits defined in `manipulator_params.yaml`**

### Core Class Design

```python
class ControllerInterface:
    """
    Utility for commanding individual ForwardCommandControllers.

    This class is NOT a ROS2 node - it requires a parent node to be passed
    during initialization. This allows action servers to share this utility
    without creating separate node instances.

    Usage:
        class MyActionServer(Node):
            def __init__(self):
                super().__init__('my_action_server')
                self.controller = ControllerInterface(self)  # Pass node reference

                # Command a joint
                success = self.controller.command_joint('base_main_frame_joint', 1.5)
    """

    def __init__(self, node: Node):
        """
        Initialize ControllerInterface with reference to parent ROS2 node.

        Args:
            node: ROS2 node instance for creating publishers/subscribers
        """
        self.node = node
        self.logger = node.get_logger()

        # Load joint limits (from config or parameters)
        self.joint_limits = self._load_joint_limits()

        # Create publishers for each controller
        self.publishers = {}
        for joint_name in self.joint_limits.keys():
            topic = f'/{joint_name}_controller/commands'
            self.publishers[joint_name] = node.create_publisher(Float64MultiArray, topic, 10)

        # Subscribe to joint states for position feedback
        self.joint_positions = {}
        node.create_subscription(JointState, '/joint_states', self._joint_state_cb, 10)

    def command_joint(self, joint_name: str, position: float) -> bool:
        """
        Command a single joint to target position.

        Returns False if:
        - Joint name not recognized
        - Position outside limits

        Does NOT wait for motion completion - just sends command.
        """
        pass

    def command_joint_group(self, joint_names: List[str], positions: List[float]) -> bool:
        """
        Command multiple joints simultaneously.

        All commands sent in rapid succession (not truly simultaneous,
        but close enough for position control).

        Returns False if ANY joint fails validation.
        """
        pass

    def get_joint_position(self, joint_name: str) -> Optional[float]:
        """
        Get current joint position from /joint_states.

        Returns None if joint not found or no data received yet.
        """
        pass

    def get_joint_limits(self, joint_name: str) -> Optional[Tuple[float, float]]:
        """Return (min_limit, max_limit) for joint, or None if unknown."""
        pass
```

### Data Flow

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                           ControllerInterface                                │
│                                                                              │
│  ┌──────────────────────────────────────────────────────────────────────┐   │
│  │                    Joint Limits (from config)                         │   │
│  │  base_main_frame: {min: 0.0, max: 4.0}                               │   │
│  │  selector_frame: {min: -0.01, max: 1.5}                               │   │
│  │  ...                                                                  │   │
│  └──────────────────────────────────────────────────────────────────────┘   │
│                                                                              │
│  command_joint("base_main_frame_joint", 2.0)                                │
│      │                                                                       │
│      ▼                                                                       │
│  ┌─────────────────────────────────────────┐                                │
│  │ 1. Validate joint exists                 │                                │
│  │ 2. Check: 0.0 <= 2.0 <= 4.0 ✓           │                                │
│  │ 3. Create Float64MultiArray(data=[2.0]) │                                │
│  │ 4. Publish to topic                     │                                │
│  └─────────────────────────────────────────┘                                │
│                    │                                                         │
│                    ▼                                                         │
│         /base_main_frame_joint_controller/commands                          │
│                    │                                                         │
└────────────────────│─────────────────────────────────────────────────────────┘
                     │
                     ▼
         ┌───────────────────────────────────┐
         │  ForwardCommandController         │
         │  (managed by ros2_control)        │
         │                                   │
         │  Receives Float64MultiArray       │
         └───────────────────────────────────┘
                     │
                     ▼
         ┌───────────────────────────────────┐
         │  /joint_states                    │
         │  (from joint_state_broadcaster)   │
         │                                   │
         │  Publishes all joint positions    │
         └───────────────────────────────────┘
                     │
                     ▼
         ┌───────────────────────────────────┐
         │  ControllerInterface subscribes   │
         │  Updates self.joint_positions     │
         └───────────────────────────────────┘
```

### Why Not Hardcode Limits?

Joint limits MUST be loaded from existing config (not hardcoded, not duplicated) because:
1. Single source of truth already exists: `manipulator_params.yaml`
2. URDF xacro files already load from this YAML
3. NO DUPLICATION - ControllerInterface loads from the same file

### Loading Limits Strategy

**DECISION: Load from `manipulator_params.yaml` (existing file - NO new config!)**

The `manipulator_description/config/manipulator_params.yaml` already defines all joint limits. The URDF xacro files load from this YAML. ControllerInterface will load from the SAME file.

**Single Source of Truth:**
```
manipulator_params.yaml
        │
        ├──► URDF xacro (joint limits)
        ├──► ros2_control.xacro (command_interface limits)
        └──► ControllerInterface (validation limits)
```

**Existing Structure in manipulator_params.yaml:**
```yaml
base_assembly:
  base_main_frame_joint:
    limits: {lower: 0.0, upper: 4.0, effort: 2000.0, velocity: 2.0}
    safety_controller: {soft_lower: 0.1, soft_upper: 3.9, k_velocity: 0}

selector_assembly:
  main_frame_selector_frame_joint:
    limits: {lower: -0.01, upper: 1.5, effort: 2000.0, velocity: 2.0}
    safety_controller: {soft_lower: 0.05, soft_upper: 1.45, k_velocity: 0}
  # ... etc
```

**ControllerInterface will parse this structure to extract limits for all 9 joints.**

**Parsing Logic (pseudocode):**
```python
def _load_joint_limits_from_params(self, params_file: str) -> dict:
    """Parse manipulator_params.yaml to extract soft limits (= controller limits)."""
    with open(params_file, 'r') as f:
        params = yaml.safe_load(f)

    limits = {}
    # Search through all assemblies for joint definitions
    for assembly_name, assembly in params.items():
        if not isinstance(assembly, dict):
            continue
        for key, value in assembly.items():
            # Joint entries have 'safety_controller' with soft limits
            if isinstance(value, dict) and 'safety_controller' in value:
                joint_name = key
                sc = value['safety_controller']
                limits[joint_name] = {
                    'min': sc['soft_lower'],  # Controller enforces soft limits
                    'max': sc['soft_upper'],
                    'velocity': value['limits'].get('velocity', 1.0),
                }
    return limits
```

**Note:** ControllerInterface validates against **soft limits** (safety_controller) because these are exactly what the ForwardCommandController command_interface enforces.

## Tasks / Subtasks

- [x] Task 1: Implement ControllerInterface Class (AC: 1, 2, 3, 4, 5, 6, 7, 8)
  - [x] 1.1 Create `src/controller_interface.py`
  - [x] 1.2 Implement `__init__()` - load limits from manipulator_params.yaml, create publishers, subscribe to joint_states
  - [x] 1.3 Implement `_load_joint_limits_from_params()` - parse manipulator_params.yaml structure
  - [x] 1.4 Implement `_joint_state_cb()` - update position cache
  - [x] 1.5 Implement `command_joint()` - validate and publish
  - [x] 1.6 Implement `command_joint_group()` - iterate and command
  - [x] 1.7 Implement `get_joint_position()` - return from cache
  - [x] 1.8 Implement `get_joint_limits()` - return tuple (also soft limits if available)
  - [x] 1.9 Add logging for all validation failures

- [x] Task 2: Create Test Script (AC: 9, 10)
  - [x] 2.1 Create `test/test_controller_interface.py` with pytest unit tests
  - [x] 2.2 Create `scripts/test_controller_interface_manual.py` for Gazebo testing
  - [x] 2.3 Add comprehensive test commands to story documentation

- [x] Task 3: Update Package Configuration (AC: 1)
  - [x] 3.1 Update CMakeLists.txt to install controller_interface.py
  - [x] 3.2 Build and verify installation

- [x] Task 4: Developer Self-Validation (MANDATORY)
  - [x] 4.1 Run unit tests (pytest) - 20/20 passed
  - [x] 4.2 Run manual Gazebo test with documented commands - 9/9 passed
  - [x] 4.3 Verify all joints respond to commands - all 9 joints moved
  - [x] 4.4 Verify limit validation rejects out-of-range commands - confirmed

## Dev Notes

### Learnings from Previous Story

**From Story 2-1-implement-virtual-limit-switch-simulation (Status: done)**

- **Virtual Limit Switches Ready:** 18 topics publishing at `/manipulator/end_switches/*`
- **Unified Launch File:** Use `ros2 launch manipulator_control manipulator_simulation.launch.py`
- **Config Pattern Established:** YAML files in `config/`, loaded via parameter
- **Trigger Position Adjustments:** Real Gazebo joint limits have ~5mm inset from URDF limits (Gazebo boundary bouncing)
- **CMakeLists Pattern:** Install Python to `lib/${PROJECT_NAME}` with RENAME, install config/launch to `share/`
- **Build Time:** ~0.7s for rebuild, ~2.5s clean build

[Source: docs/sprint-artifacts/2-1-implement-virtual-limit-switch-simulation.md#Dev-Agent-Record]

### Configuration Reuse Policy

**USE (Single Source of Truth - NO NEW CONFIG FILES):**
- `manipulator_description/config/manipulator_params.yaml` - **PRIMARY SOURCE** for joint limits (both hard and soft)
- `manipulator_description/config/manipulator_controllers.yaml` - controller names reference

**DO NOT CREATE:**
- NO new YAML config files for limits
- NO duplication of limit values anywhere

**DO NOT DUPLICATE:**
- Controller limits inside Python code
- Controller topic names (derive from joint names using pattern: `/{joint_name}_controller/commands`)

### Project Structure

```
ros2_ws/src/manipulator_control/
├── config/
│   └── limit_switches.yaml           # Story 2.1 (existing)
├── src/
│   ├── virtual_limit_switches.py     # Story 2.1 (existing)
│   └── controller_interface.py       # Story 2.2 (NEW)
├── test/
│   └── test_controller_interface.py  # Story 2.2 (NEW)
├── scripts/
│   └── test_controller_interface_manual.py  # Story 2.2 (NEW)
└── launch/
    ├── virtual_limit_switches.launch.py  # Story 2.1 (existing)
    └── manipulator_simulation.launch.py  # Story 2.1 (existing)

# Limits loaded from (NO NEW FILE):
ros2_ws/src/manipulator_description/config/manipulator_params.yaml
```

### References

- [Source: docs/architecture-ros2-control-v2-CORRECTIONS.md#Lines-13-28] Individual ForwardCommandControllers architecture
- [Source: docs/sprint-artifacts/tech-spec-epic-2.md#Controller-Interface-Utility] API specification
- [Source: docs/epics.md#Story-2.2] Acceptance criteria and technical notes
- [Source: ros2_ws/src/manipulator_description/urdf/manipulator/ros2_control.xacro] Joint limits
- [Source: ros2_ws/src/manipulator_description/config/manipulator_controllers.yaml] Controller definitions

## Test Requirements (MANDATORY)

### 1. Build Verification

```bash
cd /home/robo/robo/ya_robot_manipulator/ros2_ws
colcon build --packages-select manipulator_control
echo "Exit code: $?"
# Expected: 0
```

### 2. Unit Tests (Pytest)

```bash
source /home/robo/robo/ya_robot_manipulator/ros2_ws/install/setup.bash
cd /home/robo/robo/ya_robot_manipulator/ros2_ws/src/manipulator_control

# Run unit tests
python3 -m pytest test/test_controller_interface.py -v

# Expected: All tests pass
```

### 3. Gazebo Integration Test (Manual)

**Terminal 1 - Launch Simulation:**
```bash
source /home/robo/robo/ya_robot_manipulator/ros2_ws/install/setup.bash
ros2 launch manipulator_control manipulator_simulation.launch.py
```

**Terminal 2 - Run Manual Test Script:**
```bash
source /home/robo/robo/ya_robot_manipulator/ros2_ws/install/setup.bash
python3 /home/robo/robo/ya_robot_manipulator/ros2_ws/src/manipulator_control/scripts/test_controller_interface_manual.py
```

### 4. Manual Command-Line Tests

```bash
source /home/robo/robo/ya_robot_manipulator/ros2_ws/install/setup.bash

# Test individual joint command
python3 << 'EOF'
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
import sys
sys.path.insert(0, '/home/robo/robo/ya_robot_manipulator/ros2_ws/src/manipulator_control/src')
from controller_interface import ControllerInterface

rclpy.init()
node = Node('test_controller')
ctrl = ControllerInterface(node)

# Test command_joint
print("\n=== Testing command_joint ===")
result = ctrl.command_joint('base_main_frame_joint', 1.0)
print(f"Command base to 1.0m: {result}")

# Test invalid position (should fail)
result = ctrl.command_joint('base_main_frame_joint', 10.0)  # Outside limits
print(f"Command base to 10.0m (invalid): {result} (expected: False)")

# Test invalid joint name
result = ctrl.command_joint('fake_joint', 1.0)
print(f"Command fake_joint: {result} (expected: False)")

# Test get_joint_position
import time
time.sleep(0.5)  # Wait for joint_states
pos = ctrl.get_joint_position('base_main_frame_joint')
print(f"\nCurrent base position: {pos}")

# Test command_joint_group
print("\n=== Testing command_joint_group ===")
result = ctrl.command_joint_group(
    ['base_main_frame_joint', 'main_frame_selector_frame_joint'],
    [2.0, 0.5]
)
print(f"Command navigation group: {result}")

node.destroy_node()
rclpy.shutdown()
print("\n=== Tests Complete ===")
EOF
```

### 5. Verify Joint Movement in Gazebo

After running commands above, visually verify in Gazebo/RViz:
- Base should move along X-axis
- Selector frame should move vertically

Also verify with:
```bash
# Check current joint positions
ros2 topic echo /joint_states --once | grep -A 20 "position"
```

### 6. Comprehensive Validation Script

```bash
#!/bin/bash
# Save as /tmp/test_controller_interface.sh and run

source /home/robo/robo/ya_robot_manipulator/ros2_ws/install/setup.bash

echo "=== ControllerInterface Validation ==="
echo ""

# 1. Check file exists
echo "1. Checking files installed..."
if [ -f "/home/robo/robo/ya_robot_manipulator/ros2_ws/src/manipulator_control/src/controller_interface.py" ]; then
    echo "   ✓ controller_interface.py exists"
else
    echo "   ✗ controller_interface.py NOT FOUND"
fi

if [ -f "/home/robo/robo/ya_robot_manipulator/ros2_ws/src/manipulator_description/config/manipulator_params.yaml" ]; then
    echo "   ✓ manipulator_params.yaml exists (joint limits source)"
else
    echo "   ✗ manipulator_params.yaml NOT FOUND"
fi

# 2. Check controller topics exist
echo ""
echo "2. Checking controller topics (requires Gazebo running)..."
TOPICS=$(ros2 topic list 2>/dev/null | grep "_controller/commands" | wc -l)
echo "   Found $TOPICS/9 controller command topics"

# 3. List controller topics
echo ""
echo "3. Controller command topics:"
ros2 topic list 2>/dev/null | grep "_controller/commands" | while read topic; do
    echo "   - $topic"
done

echo ""
echo "=== Run pytest and manual tests next ==="
```

## Dev Agent Record

### Context Reference

- [Story Context XML](./2-2-create-controller-interface-utility.context.xml)

### Agent Model Used

Claude Opus 4.5 (claude-opus-4-5-20251101)

### Debug Log References

- Initial implementation used `/command` topic pattern; corrected to `/commands` (ForwardCommandController uses Float64MultiArray)

### Completion Notes List

- Unit tests: 20/20 passed
- Manual Gazebo tests: 9/9 passed
- All 9 joints discovered and commandable
- Soft limits correctly loaded from manipulator_params.yaml
- Invalid positions rejected with warnings logged

### File List

- `ros2_ws/src/manipulator_control/src/controller_interface.py` (NEW)
- `ros2_ws/src/manipulator_control/test/test_controller_interface.py` (NEW)
- `ros2_ws/src/manipulator_control/scripts/test_controller_interface_manual.py` (NEW)
- `ros2_ws/src/manipulator_control/CMakeLists.txt` (MODIFIED)

## Change Log

| Date | Change | Author |
|------|--------|--------|
| 2025-11-25 | Story drafted with comprehensive key concepts, core logic explanation, and testing framework | SM Agent (Bob) |
| 2025-11-26 | Implementation complete - ControllerInterface utility with all tests passing | Dev Agent (Amelia) |
