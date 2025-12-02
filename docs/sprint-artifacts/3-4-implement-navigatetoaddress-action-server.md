# Story 3.4: Implement NavigateToAddress Action Server

Status: ready-for-dev

## Story

As a developer,
I want a high-level action to navigate the manipulator to any warehouse address,
So that higher-level workflows (ExtractBox, PickItemFromStorage) can position the system with a single action call.

## Core Concepts

### What This Story Delivers

**NavigateToAddress is the bridge between address resolution and physical motion. It combines the coordinate lookup capability from Story 3.2 with the joint group motion from Story 3.3 to provide a complete navigation solution:**

1. **Address-to-Motion Orchestration** - Takes a warehouse address (side, cabinet, row, column) and executes the complete navigation sequence:
   - Call GetAddressCoordinates service to resolve address → (x, y, z) world coordinates
   - Use TF to compute required joint positions that place the **end effector frame** at target
   - Send MoveJointGroup goal with "navigation" group
   - Verify final positioning within ±2cm tolerance (NFR-002)

2. **Configurable End Effector Frame** - Each joint group defines which TF frame should align with the target:
   - Configured in `kinematic_chains.yaml` as `end_effector_frame` per group
   - For navigation group: `selector_frame` (the platform carrying the gripper)
   - Optional `end_effector_offset: [x, y, z]` for fine-tuning alignment
   - Position verification uses this frame, not raw joint positions

3. **World-to-Joint Coordinate Mapping** - Critical: joint positions ≠ world coordinates!
   - Joint positions are relative to their parent frame origins
   - Navigation group kinematic chain:
     ```
     world → base_link → base_main_frame_joint [X] → main_frame
                         → main_frame_selector_frame_joint [Z, origin offset: 0.301m] → selector_frame
     ```
   - Mapping requires accounting for joint origin offsets from URDF

4. **Approach Distance Support** - Optional offset from cabinet face for safe approach:
   - Default 0.0m (position at exact address)
   - Used by ExtractBox to maintain clearance before insertion trajectory
   - Applied as Y-axis offset in coordinate calculation

5. **Position Error Verification** - After motion completes:
   - Look up actual `end_effector_frame` position via TF
   - Calculate Euclidean distance from target address frame
   - Reject result if error > 2cm (positioning_error field in result)

6. **Progress Feedback** - Continuous feedback during navigation:
   - Current end effector position [x, y, z] in world frame
   - Distance to target (Euclidean)
   - Progress percent (0-100%)

### Why This Matters for Epic 3

NavigateToAddress is the ACTION INTERFACE that Epic 4 and beyond will use for all positioning:
- ExtractBox calls NavigateToAddress before YZ trajectory insertion
- ReturnBox calls NavigateToAddress to return to original address
- PutBox calls NavigateToAddress for box relocation
- PickItemFromStorage uses it for all box positioning

### Architecture Pattern

```
NavigateToAddress Action
         │
         ├─► GetAddressCoordinates Service (Story 3.2)
         │   └── Returns: (x, y, z) world coordinates + orientation
         │
         ├─► World-to-Joint Coordinate Mapping
         │   └── TF lookup: world → end_effector_frame
         │   └── Compute joint targets that place end_effector at target world position
         │   └── Account for joint origin offsets from kinematic chain
         │
         ├─► MoveJointGroup Action Client (Story 3.3)
         │   └── joint_group="navigation", targets=[x_joint, z_joint]
         │
         └─► Position Verification via TF
             └── Lookup actual end_effector_frame position
             └── Compare to target, verify within tolerance
```

### Key Implementation Details

- **End Effector Frame**: Configurable per joint group in `kinematic_chains.yaml`
- **Coordinate Mapping**: Uses TF to compute joint targets (NOT direct 1:1 mapping)
- **Service Client Timeout**: 5 seconds for GetAddressCoordinates call
- **Action Client Timeout**: 30 seconds for MoveJointGroup execution (configurable)
- **Position Tolerance**: 0.02m (2cm) for success verification
- **Feedback Rate**: 5 Hz during motion

### Kinematic Chain Analysis (Navigation Group)

```
Joint: base_main_frame_joint
  - Type: prismatic, Axis: X
  - Origin: [0, 0, 0] relative to base_link
  - World X = joint_position (direct mapping)

Joint: main_frame_selector_frame_joint
  - Type: prismatic, Axis: Z
  - Origin: [0.538, 0, 0.301] relative to main_frame
  - World Z of selector_frame = 0.301 + joint_position

Therefore:
  x_joint = target_x_world
  z_joint = target_z_world - 0.301  (subtract joint origin Z offset)
```

### Configuration Extension (kinematic_chains.yaml)

```yaml
joint_groups:
  navigation:
    joints:
      - base_main_frame_joint
      - main_frame_selector_frame_joint
    description: "X-Z positioning for address navigation"
    default_velocity: 0.5
    default_acceleration: 0.25
    # NEW: End effector configuration
    end_effector_frame: "selector_frame"  # TF frame to align with target
    end_effector_offset: [0.0, 0.0, 0.0]  # Optional offset from frame origin [x, y, z]
    coordinate_mapping:  # Joint origin offsets from world frame
      base_main_frame_joint: {axis: "x", offset: 0.0}
      main_frame_selector_frame_joint: {axis: "z", offset: 0.301}
```

## Acceptance Criteria

1. **Action Interface (AC1):** NavigateToAddress action defined in `action/NavigateToAddress.action` with goal (side, cabinet_num, row, column, approach_distance), result (success, final_position, positioning_error, message), feedback (current_position, distance_to_target, progress_percent)
2. **Address Resolution (AC2):** Action calls GetAddressCoordinates service to resolve address to world coordinates before motion
3. **Navigation Group Usage (AC3):** Action uses MoveJointGroup with joint_group="navigation" for coordinated XZ motion
4. **End Effector Configuration (AC4):** Navigation group in `kinematic_chains.yaml` defines `end_effector_frame` (TF frame to align with target) and `coordinate_mapping` (joint-to-world axis mapping with offsets)
5. **World-to-Joint Mapping (AC5):** Joint target positions computed using `coordinate_mapping` offsets from config (e.g., z_joint = z_world - 0.301 for selector_frame joint)
6. **Position Verification via TF (AC6):** After motion completes, actual `end_effector_frame` position looked up via TF and compared to target. positioning_error = Euclidean distance. Success requires error < 2cm (NFR-002)
7. **Approach Distance (AC7):** When approach_distance > 0, target Y coordinate is adjusted (Y = address_Y - approach_distance for left side, Y = address_Y + approach_distance for right side)
8. **Timeout Handling (AC8):** Navigation timeout of 30 seconds (configurable in action_servers.yaml). Timeout returns success=false with message "Navigation timeout exceeded"
9. **Invalid Address Rejection (AC9):** Invalid addresses (from GetAddressCoordinates failure) immediately rejected with success=false and error message from service
10. **Feedback During Motion (AC10):** Feedback published at 5 Hz with current end effector position (from TF), distance_to_target, progress_percent
11. **Logging (AC11):** INFO level logging for navigation start/complete with addresses, WARN for positioning errors near tolerance, ERROR for failures
12. **MANDATORY Test Verification (AC12):** All unit tests pass, integration test verifies navigation to 3 addresses from different cabinets with accuracy measurement via TF lookup, results documented in Dev Agent Record

## Tasks / Subtasks

- [ ] Task 1: Extend kinematic_chains.yaml with end effector configuration (AC: 4)
  - [ ] 1.1 Add `end_effector_frame: "selector_frame"` to navigation group
  - [ ] 1.2 Add `end_effector_offset: [0.0, 0.0, 0.0]` for optional fine-tuning
  - [ ] 1.3 Add `coordinate_mapping` section with joint-to-world axis/offset for each joint:
    - `base_main_frame_joint: {axis: "x", offset: 0.0}`
    - `main_frame_selector_frame_joint: {axis: "z", offset: 0.301}`
  - [ ] 1.4 Document the configuration format in YAML comments

- [ ] Task 2: Create NavigateToAddress.action interface (AC: 1)
  - [ ] 2.1 Create `ros2_ws/src/manipulator_interfaces/action/NavigateToAddress.action`
  - [ ] 2.2 Define Goal: string side, uint8 cabinet_num, uint8 row, uint8 column, float64 approach_distance
  - [ ] 2.3 Define Result: bool success, geometry_msgs/Point final_position, float64 positioning_error, string message
  - [ ] 2.4 Define Feedback: geometry_msgs/Point current_position, float64 distance_to_target, uint8 progress_percent
  - [ ] 2.5 Update `manipulator_interfaces/CMakeLists.txt` to generate action
  - [ ] 2.6 Run `colcon build --packages-select manipulator_interfaces` and verify generation

- [ ] Task 3: Create NavigateToAddressServer class structure (AC: 2, 3, 4)
  - [ ] 3.1 Create `ros2_ws/src/manipulator_control/src/navigate_to_address_server.py`
  - [ ] 3.2 Initialize node with name 'navigate_to_address_server'
  - [ ] 3.3 Create action server for '/navigate_to_address' action
  - [ ] 3.4 Create service client for '/manipulator/get_address_coordinates'
  - [ ] 3.5 Create action client for '/move_joint_group'
  - [ ] 3.6 Initialize TF buffer and listener for position verification
  - [ ] 3.7 Load kinematic_chains.yaml and extract navigation group config (end_effector_frame, coordinate_mapping)
  - [ ] 3.8 Load timeout configuration from action_servers.yaml

- [ ] Task 4: Implement world-to-joint coordinate mapping (AC: 5)
  - [ ] 4.1 Create `_world_to_joint_positions(target_x, target_y, target_z)` method
  - [ ] 4.2 For each joint in group, apply coordinate_mapping: `joint_pos = world_coord[axis] - offset`
  - [ ] 4.3 Return list of joint target positions in same order as group's joints list
  - [ ] 4.4 Log the mapping: "Target world (x, y, z) → joint positions [...]"

- [ ] Task 5: Implement goal acceptance callback (AC: 9)
  - [ ] 5.1 In `goal_callback()`, validate address parameters (side, cabinet, row, column ranges)
  - [ ] 5.2 Check if GetAddressCoordinates service is available (reject if not)
  - [ ] 5.3 Check if MoveJointGroup action server is available (reject if not)
  - [ ] 5.4 Return ACCEPT if all checks pass

- [ ] Task 6: Implement execute callback - address resolution (AC: 2, 7, 9)
  - [ ] 6.1 Call GetAddressCoordinates service with address from goal
  - [ ] 6.2 Handle service call timeout (5 seconds)
  - [ ] 6.3 If service returns success=false, abort with error message
  - [ ] 6.4 Extract (x, y, z) from service response pose
  - [ ] 6.5 If approach_distance > 0, adjust Y coordinate based on side

- [ ] Task 7: Implement execute callback - motion execution (AC: 3, 5, 10)
  - [ ] 7.1 Call `_world_to_joint_positions()` to compute joint targets
  - [ ] 7.2 Create MoveJointGroup goal: joint_names=["navigation"], target_positions from mapping
  - [ ] 7.3 Send goal to MoveJointGroup action client
  - [ ] 7.4 Implement feedback callback that looks up end_effector_frame via TF
  - [ ] 7.5 Publish NavigateToAddress feedback at 5 Hz with current position, distance, progress

- [ ] Task 8: Implement execute callback - position verification via TF (AC: 6, 8)
  - [ ] 8.1 Wait for MoveJointGroup result (with timeout handling)
  - [ ] 8.2 Look up actual `end_effector_frame` position via TF: `tf_buffer.lookup_transform('world', end_effector_frame)`
  - [ ] 8.3 Apply `end_effector_offset` if configured (transform offset to world frame)
  - [ ] 8.4 Calculate positioning_error = Euclidean distance from target world position
  - [ ] 8.5 If positioning_error > 0.02m, return success=false with warning message
  - [ ] 8.6 Populate result: final_position (from TF), positioning_error, success, message

- [ ] Task 9: Implement logging (AC: 11)
  - [ ] 9.1 INFO: "Navigating to address {side}-{cabinet}-{row}-{column}"
  - [ ] 9.2 INFO: "World target: ({x:.3f}, {y:.3f}, {z:.3f}) → Joint targets: [...]"
  - [ ] 9.3 INFO: "Navigation complete: end_effector at ({x:.3f}, {y:.3f}, {z:.3f}), error={err:.4f}m"
  - [ ] 9.4 WARN: If positioning_error > 0.015m (approaching tolerance)
  - [ ] 9.5 ERROR: On service call failure, action timeout, position verification failure

- [ ] Task 10: Add launch configuration (AC: 2)
  - [ ] 10.1 Add navigate_to_address_server to manipulator_simulation.launch.py
  - [ ] 10.2 Use TimerAction to delay start until dependencies are ready (4+ seconds)
  - [ ] 10.3 Verify action appears in `ros2 action list`

- [ ] Task 11: Create unit tests (AC: 12)
  - [ ] 11.1 Create `ros2_ws/src/manipulator_control/test/test_navigate_to_address.py`
  - [ ] 11.2 Test: Action interface message generation (imports work)
  - [ ] 11.3 Test: World-to-joint coordinate mapping with offsets
  - [ ] 11.4 Test: Approach distance calculation for left side (Y decreases)
  - [ ] 11.5 Test: Approach distance calculation for right side (Y increases)
  - [ ] 11.6 Test: Position error Euclidean calculation
  - [ ] 11.7 Test: Config loading (end_effector_frame, coordinate_mapping)

- [ ] Task 12: Integration verification via TF (AC: 6, 12)
  - [ ] 12.1 Launch simulation with navigate_to_address_server running
  - [ ] 12.2 Send goal: left-1-1-1, verify end_effector_frame aligns with addr_l_1_1_1
  - [ ] 12.3 Send goal: left-3-5-3, verify navigation to different cabinet
  - [ ] 12.4 Send goal: right-4-1-4, verify right side navigation
  - [ ] 12.5 For each: `ros2 run tf2_ros tf2_echo world selector_frame` vs `ros2 run tf2_ros tf2_echo world addr_X_X_X_X`
  - [ ] 12.6 Measure positioning_error for each (must be < 2cm)
  - [ ] 12.7 Test invalid address rejection: left-5-1-1

- [ ] Task 13: **MANDATORY Developer Validation** (AC: ALL)
  - [ ] 13.1 Run `colcon build --packages-select manipulator_interfaces manipulator_control` - must exit 0
  - [ ] 13.2 Run `pytest test/test_navigate_to_address.py -v` - all tests must pass
  - [ ] 13.3 Verify navigation to 3 different addresses in live simulation
  - [ ] 13.4 Verify positioning_error < 2cm for all successful navigations (measured via TF)
  - [ ] 13.5 Verify approach_distance offsets work correctly
  - [ ] 13.6 Document test results in Dev Agent Record section
  - [ ] 13.7 **Story is NOT complete until all tests pass and results documented**

## Dev Notes

### Action Interface Definition

```
# action/NavigateToAddress.action
# Goal
string side              # "left" or "right"
uint8 cabinet_num        # 1-4
uint8 row                # 1 to max_rows
uint8 column             # 1 to max_cols
float64 approach_distance  # Offset from cabinet face (default 0.0m)
---
# Result
bool success
geometry_msgs/Point final_position  # Actual end effector position (x, y, z) from TF
float64 positioning_error           # Distance from target in meters
string message
---
# Feedback
geometry_msgs/Point current_position  # Current end effector position from TF
float64 distance_to_target
uint8 progress_percent               # 0-100
```

### Extended kinematic_chains.yaml Configuration

```yaml
joint_groups:
  navigation:
    joints:
      - base_main_frame_joint
      - main_frame_selector_frame_joint
    description: "X-Z positioning for address navigation"
    default_velocity: 0.5
    default_acceleration: 0.25

    # End effector configuration for NavigateToAddress
    end_effector_frame: "selector_frame"  # TF frame to align with target
    end_effector_offset: [0.0, 0.0, 0.0]  # Optional [x, y, z] offset from frame origin

    # World-to-joint coordinate mapping
    # For each joint: which world axis it controls and the offset from world origin
    coordinate_mapping:
      base_main_frame_joint:
        axis: "x"       # This joint controls world X position
        offset: 0.0     # Joint origin X offset from world (joint_pos = world_x - offset)
      main_frame_selector_frame_joint:
        axis: "z"       # This joint controls world Z position
        offset: 0.301   # Joint origin Z offset from world (joint_pos = world_z - 0.301)
```

### Core Implementation Pattern

```python
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, ActionClient, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from tf2_ros import Buffer, TransformListener
from manipulator_interfaces.action import NavigateToAddress, MoveJointGroup
from manipulator_interfaces.srv import GetAddressCoordinates
from geometry_msgs.msg import Point
from ament_index_python.packages import get_package_share_directory
import yaml
import math
import os


class NavigateToAddressServer(Node):
    """High-level action server for warehouse address navigation."""

    POSITION_TOLERANCE = 0.02  # 2cm (NFR-002)
    SERVICE_TIMEOUT = 5.0  # seconds
    ACTION_TIMEOUT = 30.0  # seconds

    def __init__(self):
        super().__init__('navigate_to_address_server')

        # Load configuration from kinematic_chains.yaml
        self._load_config()

        # TF buffer for position verification
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        # Callback group for concurrent operations
        self._callback_group = ReentrantCallbackGroup()

        # Action server
        self._action_server = ActionServer(
            self,
            NavigateToAddress,
            '/navigate_to_address',
            execute_callback=self._execute_callback,
            goal_callback=self._goal_callback,
            cancel_callback=self._cancel_callback,
            callback_group=self._callback_group
        )

        # Service client for address resolution
        self._address_client = self.create_client(
            GetAddressCoordinates,
            '/manipulator/get_address_coordinates',
            callback_group=self._callback_group
        )

        # Action client for joint group motion
        self._move_group_client = ActionClient(
            self,
            MoveJointGroup,
            '/move_joint_group',
            callback_group=self._callback_group
        )

        self.get_logger().info(
            f"NavigateToAddress ready. End effector: {self._end_effector_frame}"
        )

    def _load_config(self):
        """Load navigation group config from kinematic_chains.yaml."""
        pkg_path = get_package_share_directory('manipulator_control')
        config_path = os.path.join(pkg_path, 'config', 'kinematic_chains.yaml')

        with open(config_path, 'r') as f:
            config = yaml.safe_load(f)

        nav_group = config['joint_groups']['navigation']
        self._joints = nav_group['joints']
        self._end_effector_frame = nav_group.get('end_effector_frame', 'selector_frame')
        self._end_effector_offset = nav_group.get('end_effector_offset', [0.0, 0.0, 0.0])
        self._coordinate_mapping = nav_group.get('coordinate_mapping', {})

        self.get_logger().info(f"Loaded coordinate_mapping: {self._coordinate_mapping}")

    def _world_to_joint_positions(self, target_x: float, target_y: float, target_z: float) -> list:
        """
        Convert world coordinates to joint positions using coordinate_mapping.

        For each joint, looks up which world axis it controls and subtracts the offset.
        Example: z_joint = z_world - 0.301 for selector_frame joint
        """
        world_coords = {'x': target_x, 'y': target_y, 'z': target_z}
        joint_positions = []

        for joint_name in self._joints:
            mapping = self._coordinate_mapping.get(joint_name, {})
            axis = mapping.get('axis', 'x')  # Default to x if not specified
            offset = mapping.get('offset', 0.0)

            joint_pos = world_coords[axis] - offset
            joint_positions.append(joint_pos)

            self.get_logger().debug(
                f"Joint {joint_name}: world_{axis}={world_coords[axis]:.3f} - "
                f"offset={offset:.3f} = {joint_pos:.3f}"
            )

        return joint_positions

    def _get_end_effector_position(self) -> tuple:
        """
        Look up current end effector position via TF.

        Returns (x, y, z, success, error_msg)
        """
        try:
            transform = self._tf_buffer.lookup_transform(
                'world',
                self._end_effector_frame,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=1.0)
            )

            x = transform.transform.translation.x + self._end_effector_offset[0]
            y = transform.transform.translation.y + self._end_effector_offset[1]
            z = transform.transform.translation.z + self._end_effector_offset[2]

            return (x, y, z, True, '')
        except Exception as e:
            return (0.0, 0.0, 0.0, False, f"TF lookup failed: {e}")

    async def _execute_callback(self, goal_handle):
        """Execute navigation to address."""
        goal = goal_handle.request
        self.get_logger().info(
            f"Navigating to {goal.side}-{goal.cabinet_num}-{goal.row}-{goal.column}"
        )

        # Step 1: Resolve address to world coordinates
        address_result = await self._resolve_address(goal)
        if not address_result['success']:
            goal_handle.abort()
            return self._create_result(False, Point(), 0.0, address_result['message'])

        target_x = address_result['x']
        target_y = address_result['y']
        target_z = address_result['z']

        # Step 2: Convert world coordinates to joint positions
        joint_targets = self._world_to_joint_positions(target_x, target_y, target_z)
        self.get_logger().info(
            f"World target: ({target_x:.3f}, {target_y:.3f}, {target_z:.3f}) → "
            f"Joint targets: {[f'{p:.3f}' for p in joint_targets]}"
        )

        # Step 3: Execute navigation via MoveJointGroup
        motion_result = await self._execute_navigation(goal_handle, joint_targets)
        if not motion_result['success']:
            goal_handle.abort()
            return self._create_result(False, Point(), 0.0, motion_result['message'])

        # Step 4: Verify final position via TF
        actual_x, actual_y, actual_z, tf_success, tf_error = self._get_end_effector_position()
        if not tf_success:
            goal_handle.abort()
            return self._create_result(False, Point(), 0.0, tf_error)

        # Calculate positioning error (XZ plane for navigation, Y handled by approach_distance)
        positioning_error = math.sqrt(
            (target_x - actual_x) ** 2 + (target_z - actual_z) ** 2
        )

        self.get_logger().info(
            f"End effector at ({actual_x:.3f}, {actual_y:.3f}, {actual_z:.3f}), "
            f"error={positioning_error:.4f}m"
        )

        # Check tolerance
        if positioning_error > self.POSITION_TOLERANCE:
            message = f"Position error {positioning_error:.4f}m exceeds {self.POSITION_TOLERANCE}m"
            self.get_logger().error(message)
            goal_handle.abort()
            return self._create_result(
                False, Point(x=actual_x, y=actual_y, z=actual_z),
                positioning_error, message
            )

        # Success
        if positioning_error > 0.015:
            self.get_logger().warning(f"Position error approaching tolerance: {positioning_error:.4f}m")

        goal_handle.succeed()
        return self._create_result(
            True, Point(x=actual_x, y=actual_y, z=actual_z),
            positioning_error, f"Navigation complete: error={positioning_error:.4f}m"
        )

    # ... remaining methods (_resolve_address, _execute_navigation, etc.)
```

### World-to-Joint Coordinate Mapping

**Key insight:** Joint positions are NOT world coordinates. Each joint has an origin offset from its parent frame.

```python
# From kinematic_chains.yaml coordinate_mapping:
#   base_main_frame_joint: {axis: "x", offset: 0.0}
#   main_frame_selector_frame_joint: {axis: "z", offset: 0.301}

# To position end effector at world (1.5, 0.4, 0.8):
x_joint = 1.5 - 0.0 = 1.5      # Direct mapping for X
z_joint = 0.8 - 0.301 = 0.499  # Subtract Z offset for selector joint

# Joint group goal: target_positions=[1.5, 0.499]
```

### Position Verification via TF

```bash
# After navigation, verify end effector aligned with target address:
ros2 run tf2_ros tf2_echo world selector_frame
# Should match (approximately):
ros2 run tf2_ros tf2_echo world addr_l_1_1_1

# Difference should be < 2cm in XZ plane
```

### Approach Distance Calculation

```python
def _apply_approach_distance(self, x, y, z, side, approach_distance):
    """
    Adjust Y coordinate for approach distance.

    For left cabinets: Y offset is positive (approach from -Y side)
    For right cabinets: Y offset is negative (approach from +Y side)
    """
    if approach_distance <= 0:
        return x, y, z

    if side == 'left':
        adjusted_y = y - approach_distance  # Step back from cabinet
    else:  # right
        adjusted_y = y + approach_distance  # Step back from cabinet

    return x, adjusted_y, z
```

### CLI Testing Commands

```bash
# Test navigation to left cabinet 1
ros2 action send_goal /navigate_to_address manipulator_interfaces/action/NavigateToAddress \
  "{side: 'left', cabinet_num: 1, row: 1, column: 1, approach_distance: 0.0}"

# Test navigation to right cabinet
ros2 action send_goal /navigate_to_address manipulator_interfaces/action/NavigateToAddress \
  "{side: 'right', cabinet_num: 4, row: 1, column: 4, approach_distance: 0.0}"

# Test with approach distance
ros2 action send_goal /navigate_to_address manipulator_interfaces/action/NavigateToAddress \
  "{side: 'left', cabinet_num: 2, row: 5, column: 2, approach_distance: 0.1}"

# Monitor feedback
ros2 topic echo /navigate_to_address/_action/feedback
```

### Project Structure Notes

- New interface: `ros2_ws/src/manipulator_interfaces/action/NavigateToAddress.action`
- New server: `ros2_ws/src/manipulator_control/src/navigate_to_address_server.py`
- New test: `ros2_ws/src/manipulator_control/test/test_navigate_to_address.py`
- **Modified:** `ros2_ws/src/manipulator_control/config/kinematic_chains.yaml` (add end_effector_frame, coordinate_mapping)
- Modified: `ros2_ws/src/manipulator_control/CMakeLists.txt` (add node installation)
- Modified: `ros2_ws/src/manipulator_control/launch/manipulator_simulation.launch.py` (add server)
- Depends on: GetAddressCoordinates service (Story 3.2), MoveJointGroup action (Story 3.3)

### Learnings from Previous Stories

**From Story 3-3 (MoveJointGroup with Joint Groups):**
- MoveJointGroup uses "navigation" group name to coordinate base_main_frame_joint and main_frame_selector_frame_joint
- Default velocity 0.5 m/s applied from kinematic_chains.yaml config
- Aggregate progress feedback available during motion
- 30 unit tests established pattern for testing

**From Story 3-2 (GetAddressCoordinates Service):**
- Service at `/manipulator/get_address_coordinates` returns full pose (position + orientation)
- Response time < 100ms (well within timeout)
- Error messages from AddressResolver flow through service

**From Story 3-1 (AddressResolver):**
- Address validation covers all cabinet configurations
- TF lookup with 1.0s timeout and fallback reference frames
- 51 total tests pass across address resolver and service

### Test Strategy

**Unit Tests (test_navigate_to_address.py):**
1. Action interface imports and message structure
2. Coordinate-to-joint mapping (x→x_joint, z→z_joint)
3. Approach distance calculation - left side (Y decreases)
4. Approach distance calculation - right side (Y increases)
5. Position error Euclidean distance calculation
6. Progress percent calculation (distance-based)

**Integration Tests (live simulation):**
1. Navigate to left-1-1-1, verify position error < 2cm
2. Navigate to left-3-5-3, verify different cabinet works
3. Navigate to right-4-1-4, verify right side navigation
4. Invalid address left-5-1-1, verify rejection with error message
5. Approach distance test, verify Y offset applied

### Configuration (action_servers.yaml)

```yaml
navigate_to_address:
  service_timeout: 5.0  # seconds for GetAddressCoordinates
  action_timeout: 30.0  # seconds for MoveJointGroup
  position_tolerance: 0.02  # meters (2cm)
  feedback_rate: 5.0  # Hz
```

### References

- [Source: docs/sprint-artifacts/tech-spec-epic-3.md#APIs and Interfaces - NavigateToAddress Action]
- [Source: docs/sprint-artifacts/tech-spec-epic-3.md#Workflows and Sequencing - Navigation Sequence]
- [Source: docs/prd.md#FR-001: Warehouse Address Navigation]
- [Source: docs/prd.md#NFR-002: Position Accuracy ±2cm]
- [Source: docs/architecture-ros2-control-v2-CORRECTIONS.md#Phase 2 - Address Navigation]
- [Source: docs/sprint-artifacts/3-3-implement-movejointgroup-with-joint-groups-configuration.md]
- [Source: docs/sprint-artifacts/3-2-implement-getaddresscoordinates-service.md]

## Change Log

| Date | Change | Author |
|------|--------|--------|
| 2025-11-27 | Initial draft | SM Agent (Bob) |

## Dev Agent Record

### Context Reference

- docs/sprint-artifacts/3-4-implement-navigatetoaddress-action-server.context.xml

### Agent Model Used

(To be filled by Dev Agent)

### Debug Log References

(To be filled by Dev Agent)

### Completion Notes List

(To be filled by Dev Agent upon completion)

### File List

(To be filled by Dev Agent upon completion)
