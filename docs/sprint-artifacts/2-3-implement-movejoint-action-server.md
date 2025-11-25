# Story 2.3: Implement MoveJoint Action Server

Status: ready-for-dev

## Story

As a **robotics operator**,
I want **to command individual joints to target positions via action interface**,
so that **I can test joint control and build higher-level coordinated motions**.

## Acceptance Criteria

1. **AC-1:** MoveJoint action server node exists at `ros2_ws/src/manipulator_control/src/move_joint_server.py`
2. **AC-2:** Server validates joint_name exists and target_position is within soft limits
3. **AC-3:** Server uses ControllerInterface to command the joint
4. **AC-4:** Feedback published at 10 Hz with current_position and progress_percent (0-100%)
5. **AC-5:** Server monitors `/joint_states` to detect when target reached (within 0.01m tolerance)
6. **AC-6:** Returns success result when position reached or aborts on timeout (30 seconds default)
7. **AC-7:** Action supports preemption (cancellation during motion)
8. **AC-8:** Timeout loaded from `config/action_servers.yaml`
9. **AC-9:** Action can be tested using `ros2 action send_goal` CLI
10. **AC-10:** Action server added to `manipulator_simulation.launch.py` as Common node with 3s delayed start
11. **AC-11:** New goal PREEMPTS (cancels) any currently executing goal - no queueing, no rejection

## Key Concepts & Core Logic

### What is a ROS2 Action Server?

Actions are the standard ROS2 pattern for **long-running tasks with feedback**. Unlike services (request/response), actions provide:
- **Goal:** What to achieve (joint_name, target_position)
- **Feedback:** Periodic updates during execution (current_position, progress_percent)
- **Result:** Final outcome when complete (success, final_position, execution_time)
- **Preemption:** Ability to cancel mid-execution

### MoveJoint Action Definition (from Story 1.2)

```
# Goal: Command a single joint to move to target position
string joint_name           # Name of joint to move
float64 target_position     # Target position in meters or radians
float64 max_velocity        # Maximum velocity (0.0 = use default)

---
# Result: Final state after action completes
bool success                # True if joint reached target
float64 final_position      # Actual final position achieved
float64 execution_time      # Total time taken (seconds)
string message              # Human-readable result message

---
# Feedback: Published periodically during execution
float64 current_position    # Current joint position
float64 progress_percent    # Completion percentage (0.0 to 100.0)
```

### Why Use ControllerInterface?

MoveJoint action server does NOT:
- Directly publish to controller topics
- Hardcode joint limits
- Know controller topic naming patterns

Instead, it uses **ControllerInterface** (Story 2.2) which:
- Already has publishers to all 9 controller topics
- Validates positions against soft limits
- Subscribes to `/joint_states` for position feedback

### Action Server Architecture

```
┌─────────────────────────────────────────────────────────────────────────┐
│                        MoveJointServer Node                              │
│                                                                          │
│  ┌──────────────────────────────────────────────────────────────────┐   │
│  │                    ControllerInterface                            │   │
│  │    (shared utility - handles publishers, limits, positions)      │   │
│  └──────────────────────────────────────────────────────────────────┘   │
│                                                                          │
│  ┌──────────────────────────────────────────────────────────────────┐   │
│  │                   ActionServer (MoveJoint.action)                 │   │
│  │                                                                   │   │
│  │   Goal Received ───► Validate ───► Command Joint ───► Monitor    │   │
│  │         │                              │                 │        │   │
│  │         │                              ▼                 ▼        │   │
│  │         │              ControllerInterface      Check /joint_states│   │
│  │         │              .command_joint()         position          │   │
│  │         │                                        │                │   │
│  │         ▼                                        ▼                │   │
│  │   Publish Feedback ◄─────────── Loop at 10 Hz ────────►         │   │
│  │         │                                                         │   │
│  │         ▼                                                         │   │
│  │   Position Reached? ──────► Return Success Result                │   │
│  │         │                                                         │   │
│  │   Timeout? ──────────────► Return Aborted Result                 │   │
│  │         │                                                         │   │
│  │   Cancel Request? ────────► Return Canceled Result               │   │
│  └──────────────────────────────────────────────────────────────────┘   │
└─────────────────────────────────────────────────────────────────────────┘
```

### Goal Policy: PREEMPT (New Goal Cancels Current)

**CRITICAL DECISION:** When a new goal arrives while executing, the server PREEMPTS (cancels) the current goal and starts the new one.

| Policy | Our Choice | Rationale |
|--------|------------|-----------|
| REJECT | ❌ | Frustrating - operator must wait or manually cancel |
| **PREEMPT** | ✅ | Responsive - latest command wins, safe for joint control |
| QUEUE | ❌ | Dangerous - accumulated goals cause unexpected motion |

**Implementation:**

```python
def goal_callback(self, goal_request):
    """Accept or reject incoming goal - PREEMPT policy."""
    # Validate goal first (joint name, limits)
    if not self._validate_goal(goal_request):
        return GoalResponse.REJECT

    # PREEMPT: Accept new goal - this automatically cancels any executing goal
    # when using handle_accepted_callback with PREEMPT policy
    self.get_logger().info('Accepting new goal (preempting any current goal)')
    return GoalResponse.ACCEPT_AND_EXECUTE
```

**Behavior:**
1. New goal arrives while joint is moving to position A
2. Current goal is canceled (status: CANCELED)
3. New goal starts immediately - joint now moves to position B
4. Only one goal executes at a time

**Safety:** This is safe because:
- ForwardCommandController accepts new position commands at any time
- No trajectory interpolation - just direct position command
- Joint limits still validated before accepting new goal

### Core State Machine

```
                    ┌──────────────┐
                    │   WAITING    │ ◄─── No active goal
                    └──────┬───────┘
                           │ Goal received
                           ▼
                    ┌──────────────┐
                    │  VALIDATING  │ ─── Check joint_name, limits
                    └──────┬───────┘
                           │ Valid
                           ▼
                    ┌──────────────┐
    New Goal ──────►│   EXECUTING  │ ─── Command sent, monitoring
    (preempts)      └──────┬───────┘
        │                  │
        │     ┌────────────┼────────────┐
        │     │            │            │
        ▼     ▼            ▼            ▼
       ┌──────────┐ ┌──────────┐ ┌──────────┐
       │ CANCELED │ │ SUCCEEDED │ │  ABORTED │
       │(preempt) │ │ (reached) │ │ (timeout)│
       └──────────┘ └──────────┘ └──────────┘
```

### Progress Calculation

Progress is calculated as distance traveled toward goal:

```python
def calculate_progress(start_pos: float, current_pos: float, target_pos: float) -> float:
    """
    Calculate progress percentage (0.0 to 100.0).

    Progress = (distance_traveled / total_distance) * 100
    """
    total_distance = abs(target_pos - start_pos)
    if total_distance < 0.001:  # Already at target
        return 100.0

    distance_traveled = abs(current_pos - start_pos)
    progress = (distance_traveled / total_distance) * 100.0
    return min(100.0, max(0.0, progress))  # Clamp to [0, 100]
```

### Position Tolerance

The action succeeds when:
```python
abs(current_position - target_position) <= position_tolerance  # Default: 0.01m
```

This tolerance is:
- Loaded from `config/action_servers.yaml`
- Matches NFR-002 (position accuracy ±0.01m for storage operations)

### Timeout Handling

```python
# From config/action_servers.yaml
timeout_sec: 30.0  # Default timeout for MoveJoint

# In execute_callback
elapsed = (self.get_clock().now() - start_time).nanoseconds / 1e9
if elapsed > self.timeout_sec:
    return self._abort_goal("Timeout: position not reached within {self.timeout_sec}s")
```

### Preemption (Cancel Request)

```python
def execute_callback(self, goal_handle):
    while not position_reached:
        # Check for cancel request
        if goal_handle.is_cancel_requested:
            goal_handle.canceled()
            return MoveJoint.Result(
                success=False,
                final_position=current_position,
                message="Goal canceled by client"
            )

        # Continue monitoring...
```

### Config File: action_servers.yaml (NEW)

```yaml
# config/action_servers.yaml
# Action server parameters - NOT hardware config

move_joint:
  timeout_sec: 30.0           # Max time to reach target
  position_tolerance: 0.01    # Success threshold (meters)
  feedback_rate: 10.0         # Hz

move_joint_group:
  timeout_sec: 30.0
  position_tolerance: 0.01
  feedback_rate: 10.0
```

### Implementation Pattern (Python)

```python
#!/usr/bin/env python3
"""
MoveJoint Action Server

Commands individual joints to target positions with feedback.
Uses ControllerInterface for joint control and limit validation.
"""

import time
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup

from manipulator_control.action import MoveJoint
from controller_interface import ControllerInterface


class MoveJointServer(Node):
    def __init__(self):
        super().__init__('move_joint_server')

        # Load configuration
        self._load_config()

        # Initialize ControllerInterface (shared utility)
        self.controller = ControllerInterface(self)

        # Create action server
        self._action_server = ActionServer(
            self,
            MoveJoint,
            'move_joint',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=ReentrantCallbackGroup()
        )

        self.get_logger().info('MoveJoint action server ready')

    def _load_config(self):
        """Load action server parameters from YAML."""
        # Load from config/action_servers.yaml
        self.timeout_sec = 30.0
        self.position_tolerance = 0.01
        self.feedback_rate = 10.0

    def goal_callback(self, goal_request):
        """Accept or reject incoming goal."""
        joint_name = goal_request.joint_name
        target = goal_request.target_position

        # Validate joint exists
        if joint_name not in self.controller.get_all_joint_names():
            self.get_logger().warning(f'Rejecting goal: unknown joint {joint_name}')
            return GoalResponse.REJECT

        # Validate position within limits
        limits = self.controller.get_joint_limits(joint_name)
        if limits and not (limits[0] <= target <= limits[1]):
            self.get_logger().warning(
                f'Rejecting goal: position {target} outside limits {limits}'
            )
            return GoalResponse.REJECT

        self.get_logger().info(f'Accepting goal: {joint_name} -> {target}')
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        """Accept cancel request."""
        self.get_logger().info('Received cancel request')
        return CancelResponse.ACCEPT

    async def execute_callback(self, goal_handle):
        """Execute the motion goal."""
        joint_name = goal_handle.request.joint_name
        target = goal_handle.request.target_position

        # Get starting position
        start_pos = self.controller.get_joint_position(joint_name)
        if start_pos is None:
            goal_handle.abort()
            return MoveJoint.Result(
                success=False,
                message=f'Cannot read position for {joint_name}'
            )

        # Send command
        self.controller.command_joint(joint_name, target)

        # Monitor until complete
        start_time = self.get_clock().now()
        feedback_msg = MoveJoint.Feedback()
        rate = self.create_rate(self.feedback_rate)

        while True:
            # Check cancel
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                return MoveJoint.Result(
                    success=False,
                    final_position=self.controller.get_joint_position(joint_name),
                    message='Canceled'
                )

            # Get current position
            current_pos = self.controller.get_joint_position(joint_name)
            elapsed = (self.get_clock().now() - start_time).nanoseconds / 1e9

            # Check success
            if abs(current_pos - target) <= self.position_tolerance:
                goal_handle.succeed()
                return MoveJoint.Result(
                    success=True,
                    final_position=current_pos,
                    execution_time=elapsed,
                    message='Target reached'
                )

            # Check timeout
            if elapsed > self.timeout_sec:
                goal_handle.abort()
                return MoveJoint.Result(
                    success=False,
                    final_position=current_pos,
                    execution_time=elapsed,
                    message=f'Timeout after {elapsed:.1f}s'
                )

            # Publish feedback
            feedback_msg.current_position = current_pos
            feedback_msg.progress_percent = self._calc_progress(start_pos, current_pos, target)
            goal_handle.publish_feedback(feedback_msg)

            rate.sleep()

    def _calc_progress(self, start: float, current: float, target: float) -> float:
        """Calculate progress percentage."""
        total = abs(target - start)
        if total < 0.001:
            return 100.0
        traveled = abs(current - start)
        return min(100.0, (traveled / total) * 100.0)


def main(args=None):
    rclpy.init(args=args)
    node = MoveJointServer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### Launch File Update

Add to `manipulator_simulation.launch.py`:

```python
# MoveJoint action server (Common - runs in both sim and hardware)
move_joint_server = TimerAction(
    period=3.0,  # 3 second delay for controllers to be ready
    actions=[
        Node(
            package='manipulator_control',
            executable='move_joint_server',
            name='move_joint_server',
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen'
        )
    ]
)
```

## Tasks / Subtasks

- [ ] Task 1: Create Configuration File (AC: 8)
  - [ ] 1.1 Create `config/action_servers.yaml` with timeout, tolerance, feedback_rate
  - [ ] 1.2 Document config parameters

- [ ] Task 2: Implement MoveJoint Action Server (AC: 1, 2, 3, 4, 5, 6, 7)
  - [ ] 2.1 Create `src/move_joint_server.py`
  - [ ] 2.2 Implement `__init__()` - load config, create ControllerInterface, create ActionServer
  - [ ] 2.3 Implement `_load_config()` - load from action_servers.yaml
  - [ ] 2.4 Implement `goal_callback()` - validate joint_name and position limits
  - [ ] 2.5 Implement `cancel_callback()` - accept all cancel requests
  - [ ] 2.6 Implement `execute_callback()` - main execution loop with feedback
  - [ ] 2.7 Implement `_calc_progress()` - progress percentage calculation
  - [ ] 2.8 Add comprehensive logging

- [ ] Task 3: Update Package Configuration (AC: 1, 10)
  - [ ] 3.1 Update CMakeLists.txt to install move_joint_server.py as executable
  - [ ] 3.2 Update CMakeLists.txt to install config/action_servers.yaml
  - [ ] 3.3 Add node to manipulator_simulation.launch.py with 3s TimerAction delay

- [ ] Task 4: Create Test Scripts (AC: 9)
  - [ ] 4.1 Create `scripts/test_move_joint_action.py` - comprehensive CLI test
  - [ ] 4.2 Document CLI test commands using `ros2 action send_goal`

- [ ] Task 5: Developer Self-Validation (MANDATORY)
  - [ ] 5.1 Build package and verify no errors
  - [ ] 5.2 Launch simulation and verify action server starts
  - [ ] 5.3 Test valid goal - joint moves to target, feedback received, success returned
  - [ ] 5.4 Test invalid joint name - goal rejected
  - [ ] 5.5 Test out-of-range position - goal rejected
  - [ ] 5.6 Test cancel during motion - goal cancels, joint stops
  - [ ] 5.7 Verify position tolerance works (0.01m)
  - [ ] 5.8 Document all test results in Dev Agent Record

## Dev Notes

### Learnings from Previous Stories

**From Story 2.2 - ControllerInterface (Status: done):**
- ControllerInterface is NOT a node - requires parent node reference
- Uses `/commands` topic (Float64MultiArray), not `/command`
- Soft limits loaded from `manipulator_params.yaml`
- Joint positions cached from `/joint_states` subscription
- All 9 joints available: base_main_frame_joint, main_frame_selector_frame_joint, etc.

**From Story 2.1 - Virtual Limit Switches (Status: done):**
- Unified launch file: `ros2 launch manipulator_control manipulator_simulation.launch.py`
- TimerAction for delayed node start (controllers need time)
- Install pattern: executables to `lib/${PROJECT_NAME}`

### Configuration Reuse Policy

**USE (from existing files):**
- Joint limits via `ControllerInterface.get_joint_limits()` (from manipulator_params.yaml)
- Joint names via `ControllerInterface.get_all_joint_names()`
- Position feedback via `ControllerInterface.get_joint_position()`

**CREATE (new file):**
- `config/action_servers.yaml` - action-specific parameters ONLY (timeout, tolerance, rate)

**DO NOT DUPLICATE:**
- Joint limits in action_servers.yaml
- Controller topic names

### Project Structure

```
ros2_ws/src/manipulator_control/
├── action/
│   └── MoveJoint.action              # Story 1.2 (existing)
├── config/
│   ├── limit_switches.yaml           # Story 2.1 (existing)
│   └── action_servers.yaml           # Story 2.3 (NEW)
├── src/
│   ├── virtual_limit_switches.py     # Story 2.1 (existing)
│   ├── controller_interface.py       # Story 2.2 (existing)
│   └── move_joint_server.py          # Story 2.3 (NEW)
├── scripts/
│   └── test_move_joint_action.py     # Story 2.3 (NEW)
└── launch/
    └── manipulator_simulation.launch.py  # UPDATE - add move_joint_server
```

### References

- [Source: docs/epics.md#Story-2.3] Acceptance criteria
- [Source: docs/sprint-artifacts/2-2-create-controller-interface-utility.md] ControllerInterface API
- [Source: ros2_ws/src/manipulator_control/action/MoveJoint.action] Action definition
- [Source: docs/architecture-ros2-control-v2-CORRECTIONS.md#Lines-2183-2197] Action implementation patterns

## Test Requirements (MANDATORY)

**CRITICAL:** Developer MUST execute ALL tests below and document results before marking story complete.

### 1. Build Verification

```bash
# Kill any existing simulation processes first
pkill -9 gazebo || true
pkill -9 gzserver || true
pkill -9 gzclient || true
pkill -9 rviz2 || true

cd /home/robo/robo/ya_robot_manipulator/ros2_ws
colcon build --packages-select manipulator_control
echo "Build exit code: $?"
# Expected: 0
```

### 2. Launch and Verify Action Server

```bash
source /home/robo/robo/ya_robot_manipulator/ros2_ws/install/setup.bash
ros2 launch manipulator_control manipulator_simulation.launch.py &

# Wait for startup (controllers + 3s delay for action server)
sleep 10

# Verify action server is running
ros2 action list | grep move_joint
# Expected: /move_joint

ros2 action info /move_joint
# Expected: Action: manipulator_control/action/MoveJoint
#           Action clients: 0
#           Action servers: 1
```

### 3. Test Valid Goal

```bash
# Send goal: move base joint to 1.5m
ros2 action send_goal /move_joint manipulator_control/action/MoveJoint \
  "{joint_name: 'base_main_frame_joint', target_position: 1.5, max_velocity: 0.0}" \
  --feedback

# Expected output:
# - Feedback messages with current_position and progress_percent
# - Result: success=True, final_position≈1.5, execution_time>0
```

### 4. Test Invalid Joint Name

```bash
ros2 action send_goal /move_joint manipulator_control/action/MoveJoint \
  "{joint_name: 'fake_joint', target_position: 1.0, max_velocity: 0.0}"

# Expected: Goal rejected (no result returned, server logs warning)
```

### 5. Test Out-of-Range Position

```bash
ros2 action send_goal /move_joint manipulator_control/action/MoveJoint \
  "{joint_name: 'base_main_frame_joint', target_position: 99.0, max_velocity: 0.0}"

# Expected: Goal rejected (position outside soft limits 0.1-3.9)
```

### 6. Test Cancel During Motion

```bash
# Terminal 1: Send long-distance goal
ros2 action send_goal /move_joint manipulator_control/action/MoveJoint \
  "{joint_name: 'base_main_frame_joint', target_position: 3.5, max_velocity: 0.0}" \
  --feedback &

# Terminal 2: Cancel after a few seconds
sleep 2
ros2 action cancel /move_joint

# Expected: Goal status changes to CANCELED
```

### 7. Test Preemption (New Goal Cancels Current) - AC-11

```bash
# This test verifies that sending a new goal while one is executing
# causes the current goal to be CANCELED and the new goal to start

# Terminal 1: Send first goal (long distance, will take time)
ros2 action send_goal /move_joint manipulator_control/action/MoveJoint \
  "{joint_name: 'base_main_frame_joint', target_position: 3.5, max_velocity: 0.0}" \
  --feedback &
FIRST_PID=$!

# Wait 2 seconds (first goal is executing, joint moving)
sleep 2

# Terminal 1: Send second goal (different target) - should preempt first
ros2 action send_goal /move_joint manipulator_control/action/MoveJoint \
  "{joint_name: 'base_main_frame_joint', target_position: 0.5, max_velocity: 0.0}" \
  --feedback

# Expected behavior:
# 1. First goal status changes to CANCELED (not SUCCEEDED, not ABORTED)
# 2. Second goal starts immediately
# 3. Joint changes direction, moves toward 0.5 instead of 3.5
# 4. Second goal completes with SUCCEEDED

# Verify first goal was canceled (check output from background process)
wait $FIRST_PID
# Should show: "Goal was canceled"
```

### 8. Verify Position Tolerance

```bash
# Move to position, check final position is within 0.01m of target
ros2 action send_goal /move_joint manipulator_control/action/MoveJoint \
  "{joint_name: 'main_frame_selector_frame_joint', target_position: 0.75, max_velocity: 0.0}"

# Check actual position
ros2 topic echo /joint_states --once | grep -A 20 "name:" | grep -A 1 "main_frame_selector"
# Expected: position value between 0.74 and 0.76
```

### 9. Comprehensive Validation Script

```bash
#!/bin/bash
# Save as /tmp/test_move_joint.sh

source /home/robo/robo/ya_robot_manipulator/ros2_ws/install/setup.bash

echo "=== MoveJoint Action Server Validation ==="
echo ""

# Check action server exists
echo "1. Checking action server..."
if ros2 action list 2>/dev/null | grep -q "/move_joint"; then
    echo "   ✓ /move_joint action available"
else
    echo "   ✗ /move_joint NOT FOUND - is simulation running?"
    exit 1
fi

# Check config file
echo ""
echo "2. Checking configuration..."
if [ -f "/home/robo/robo/ya_robot_manipulator/ros2_ws/src/manipulator_control/config/action_servers.yaml" ]; then
    echo "   ✓ action_servers.yaml exists"
else
    echo "   ✗ action_servers.yaml NOT FOUND"
fi

# Check executable
echo ""
echo "3. Checking executable..."
if [ -f "/home/robo/robo/ya_robot_manipulator/ros2_ws/install/manipulator_control/lib/manipulator_control/move_joint_server" ]; then
    echo "   ✓ move_joint_server executable installed"
else
    echo "   ✗ move_joint_server NOT INSTALLED"
fi

echo ""
echo "=== Manual Tests Required ==="
echo "Run the following tests and verify results:"
echo "1. ros2 action send_goal /move_joint ... (valid goal)"
echo "2. ros2 action send_goal /move_joint ... (invalid joint)"
echo "3. ros2 action send_goal /move_joint ... (out of range)"
echo "4. Cancel test during motion"
```

## Dev Agent Record

### Context Reference

- Story Context XML: (to be generated by SM agent)

### Agent Model Used

(to be filled by dev agent)

### Debug Log References

(to be filled by dev agent)

### Completion Notes List

(to be filled by dev agent)

### File List

- `ros2_ws/src/manipulator_control/config/action_servers.yaml` (NEW)
- `ros2_ws/src/manipulator_control/src/move_joint_server.py` (NEW)
- `ros2_ws/src/manipulator_control/scripts/test_move_joint_action.py` (NEW)
- `ros2_ws/src/manipulator_control/launch/manipulator_simulation.launch.py` (MODIFIED)
- `ros2_ws/src/manipulator_control/CMakeLists.txt` (MODIFIED)

## Change Log

| Date | Change | Author |
|------|--------|--------|
| 2025-11-26 | Story drafted with comprehensive key concepts, core logic explanation, and mandatory testing requirements | SM Agent (Bob) |
