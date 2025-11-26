# Migration Plan: ForwardCommandController to JointTrajectoryController

**Document Version:** 1.0
**Date:** 2025-11-26
**ROS2 Distribution:** Jazzy
**Gazebo Version:** Harmonic
**Status:** Ready for Implementation

---

## Table of Contents

1. [Executive Summary](#1-executive-summary)
2. [Scope and Goals](#2-scope-and-goals)
3. [Current vs Target Architecture](#3-current-vs-target-architecture)
4. [Architecture Document Sections to Update](#4-architecture-document-sections-to-update)
5. [Implementation Plan](#5-implementation-plan)
6. [Configuration Changes](#6-configuration-changes)
7. [Code Changes](#7-code-changes)
8. [Testing Strategy](#8-testing-strategy)
9. [Rollback Plan](#9-rollback-plan)

---

## 1. Executive Summary

This document details the migration of **7 joints** from `forward_command_controller/ForwardCommandController` to individual `joint_trajectory_controller/JointTrajectoryController`s. The **2 container jaw joints** will remain as `ForwardCommandController` due to their simple open/close operation.

### Benefits of Migration

| Benefit | Description |
|---------|-------------|
| **Coordinated multi-joint motion** | Action interface (`FollowJointTrajectory`) enables synchronized trajectories |
| **Smooth interpolated motion** | Spline interpolation eliminates jerky position jumps |
| **MoveIt2 integration** | `FollowJointTrajectory` is MoveIt's native interface |
| **Velocity/acceleration limits** | Controller-level enforcement of motion constraints |
| **Goal monitoring** | Built-in goal tolerance and timeout handling |

---

## 2. Scope and Goals

### 2.1 Joints to Migrate (7 joints)

| Joint | Assembly | Axis | Range | Velocity |
|-------|----------|------|-------|----------|
| `base_main_frame_joint` | Base | X (railway) | 0.1 - 3.9 m | 2.0 m/s |
| `main_frame_selector_frame_joint` | Selector | Z (vertical) | 0.05 - 1.45 m | 2.0 m/s |
| `selector_frame_gripper_joint` | Selector | Y (into cabinets) | -0.39 - 0.39 m | 1.0 m/s |
| `selector_frame_picker_frame_joint` | Picker | Z (vertical) | 0.005 - 0.29 m | 1.0 m/s |
| `picker_frame_picker_rail_joint` | Picker | Y (along box) | -0.29 - 0.29 m | 1.0 m/s |
| `picker_rail_picker_base_joint` | Picker | X (extension) | 0.005 - 0.24 m | 1.0 m/s |
| `picker_base_picker_jaw_joint` | Picker | X (jaw) | 0.005 - 0.19 m | 1.0 m/s |

### 2.2 Joints to Keep as ForwardCommandController (2 joints)

| Joint | Assembly | Reason |
|-------|----------|--------|
| `selector_left_container_jaw_joint` | Selector | Simple gripper, no trajectory needed |
| `selector_right_container_jaw_joint` | Selector | Simple gripper, mimic joint in hardware |

### 2.3 Interface Changes

| Aspect | Old (ForwardCommand) | New (Trajectory) |
|--------|---------------------|------------------|
| **Interface Type** | Topic | Action |
| **Topic/Action Name** | `/{joint}_controller/commands` | `/{joint}_controller/follow_joint_trajectory` |
| **Message Type** | `std_msgs/Float64MultiArray` | `control_msgs/action/FollowJointTrajectory` |
| **Motion Profile** | Instant jump | Interpolated trajectory |
| **Feedback** | None | Progress feedback during execution |
| **Result** | None | Success/failure with error codes |

---

## 3. Current vs Target Architecture

### 3.1 Current Architecture (ForwardCommandController)

```
┌─────────────────────────────────────────────────────────────────────┐
│                        CURRENT ARCHITECTURE                          │
├─────────────────────────────────────────────────────────────────────┤
│                                                                      │
│  ┌──────────────────┐     /{joint}_controller/commands              │
│  │  ControllerInterface │────────────────────────────►  ForwardCmd   │
│  │  (Publisher)         │     Float64MultiArray          Controller  │
│  └──────────────────┘                                    (9 joints)  │
│           │                                                   │      │
│           │                                                   │      │
│           ▼                                                   ▼      │
│  ┌──────────────────┐                              ┌─────────────┐  │
│  │  /joint_states   │◄─────────────────────────────│  Hardware   │  │
│  │  (Subscriber)    │                              │  Interface  │  │
│  └──────────────────┘                              └─────────────┘  │
│                                                                      │
│  Control Flow: Publish position → Controller sets immediately       │
│  No interpolation, no feedback, no goal monitoring                  │
│                                                                      │
└─────────────────────────────────────────────────────────────────────┘
```

### 3.2 Target Architecture (JointTrajectoryController)

```
┌─────────────────────────────────────────────────────────────────────┐
│                        TARGET ARCHITECTURE                           │
├─────────────────────────────────────────────────────────────────────┤
│                                                                      │
│  ┌──────────────────┐   /{joint}_controller/follow_joint_trajectory │
│  │  ControllerInterface │─────────────────────────►  JointTrajectory │
│  │  (ActionClient)      │   FollowJointTrajectory     Controller    │
│  └──────────────────┘           Action                (7 joints)    │
│           │                        │                       │        │
│           │◄───────────────────────┘                       │        │
│           │    feedback, result                            │        │
│           │                                                │        │
│  ┌──────────────────┐   /{jaw}_controller/commands         │        │
│  │  ControllerInterface │────────────────────────► ForwardCmd       │
│  │  (Publisher)         │  Float64MultiArray      Controller        │
│  └──────────────────┘                             (2 jaws)          │
│           │                                            │            │
│           ▼                                            ▼            │
│  ┌──────────────────┐                        ┌─────────────┐       │
│  │  /joint_states   │◄───────────────────────│  Hardware   │       │
│  │  (Subscriber)    │                        │  Interface  │       │
│  └──────────────────┘                        └─────────────┘       │
│                                                                      │
│  Control Flow:                                                       │
│  - Trajectory joints: Send goal → Controller interpolates → Result  │
│  - Container jaws: Publish position → Controller sets immediately   │
│                                                                      │
└─────────────────────────────────────────────────────────────────────┘
```

---

## 4. Architecture Document Sections to Update

**Target Document:** `/home/robo/robo/ya_robot_manipulator/docs/architecture-ros2-control-v2-CORRECTIONS.md`

### 4.1 Section: "1. ✅ Controller Architecture (Individual Position Controllers)"

**Current Content (Lines ~12-33):**
```markdown
**CORRECTED:** The system uses **individual ForwardCommandControllers** for each joint...
Each of the 9 joints has its own position controller:
- `base_main_frame_joint_controller`
...
**Implementation Strategy:**
- **Primary:** Send individual position commands to each joint controller
```

**Required Update:**
```markdown
### 1. ✅ Controller Architecture (Hybrid: Trajectory + Forward Controllers)

**UPDATED 2025-11-26:** The system uses a **hybrid controller architecture**:
- **7 JointTrajectoryControllers** for motion joints (smooth trajectory interpolation)
- **2 ForwardCommandControllers** for container jaw joints (simple position commands)

**Trajectory Controllers (7 joints):**
- `base_main_frame_joint_controller` (JointTrajectoryController)
- `main_frame_selector_frame_joint_controller` (JointTrajectoryController)
- `selector_frame_gripper_joint_controller` (JointTrajectoryController)
- `selector_frame_picker_frame_joint_controller` (JointTrajectoryController)
- `picker_frame_picker_rail_joint_controller` (JointTrajectoryController)
- `picker_rail_picker_base_joint_controller` (JointTrajectoryController)
- `picker_base_picker_jaw_joint_controller` (JointTrajectoryController)

**Forward Controllers (2 joints - unchanged):**
- `selector_left_container_jaw_joint_controller` (ForwardCommandController)
- `selector_right_container_jaw_joint_controller` (ForwardCommandController)

**Implementation Strategy:**
- **Trajectory joints:** Use `FollowJointTrajectory` action for smooth interpolated motion
- **Container jaws:** Continue using topic-based position commands (simple open/close)
- **Coordinated motion:** Higher-level action servers compose multi-joint trajectories
- **MoveIt2 ready:** Trajectory interface is MoveIt's native control method
```

### 4.2 Section: "Configuration Management (Single Source of Truth)" (~Lines 1266-1363)

**Required Addition after existing content:**

```markdown
#### Controller Type Mapping

| Joint | Controller Type | Interface |
|-------|----------------|-----------|
| base_main_frame_joint | `JointTrajectoryController` | Action: `follow_joint_trajectory` |
| main_frame_selector_frame_joint | `JointTrajectoryController` | Action: `follow_joint_trajectory` |
| selector_frame_gripper_joint | `JointTrajectoryController` | Action: `follow_joint_trajectory` |
| selector_frame_picker_frame_joint | `JointTrajectoryController` | Action: `follow_joint_trajectory` |
| picker_frame_picker_rail_joint | `JointTrajectoryController` | Action: `follow_joint_trajectory` |
| picker_rail_picker_base_joint | `JointTrajectoryController` | Action: `follow_joint_trajectory` |
| picker_base_picker_jaw_joint | `JointTrajectoryController` | Action: `follow_joint_trajectory` |
| selector_left_container_jaw_joint | `ForwardCommandController` | Topic: `/commands` |
| selector_right_container_jaw_joint | `ForwardCommandController` | Topic: `/commands` |

#### JointTrajectoryController Parameters

All trajectory controllers share these common parameters:

```yaml
{joint}_controller:
  ros__parameters:
    joints: [{joint_name}]
    command_interfaces: [position]
    state_interfaces: [position, velocity]
    action_monitor_rate: 20.0
    allow_partial_joints_goal: false
    allow_nonzero_velocity_at_trajectory_end: false
    interpolation_method: splines
    constraints:
      stopped_velocity_tolerance: 0.01
      goal_time: 0.0  # No timeout
      {joint_name}:
        trajectory: 0.1  # Path tolerance (m)
        goal: 0.01       # Goal tolerance (m) - matches NFR-002
```
```

### 4.3 Section: "Core Physical Workflows Reference" (~Lines 247-403)

**Required Update to Workflow Notes:**

Add the following note at the beginning of the workflows section:

```markdown
**Controller Interface Note (Updated 2025-11-26):**

All workflows now use `JointTrajectoryController` for motion joints, enabling:
- Smooth trajectory interpolation between positions
- Action-based interface with feedback and result
- Coordinated multi-joint motion via trajectory composition

Container jaw operations continue to use `ForwardCommandController` (topic-based).

**Example: Sending Trajectory Command**

```python
# Using ControllerInterface (abstracts controller type)
controller_interface.command_joint('base_main_frame_joint', target_position=2.0, duration_sec=3.0)

# Direct action call (for trajectory joints)
goal = FollowJointTrajectory.Goal()
goal.trajectory.joint_names = ['base_main_frame_joint']
goal.trajectory.points = [
    JointTrajectoryPoint(
        positions=[2.0],
        velocities=[0.0],
        time_from_start=Duration(sec=3)
    )
]
action_client.send_goal_async(goal)
```
```

### 4.4 Section: "YZ Trajectory Generation" (~Lines 1765-1800)

**Required Update:**

The YZ trajectory generation section should be updated to reflect that trajectories are now executed via `JointTrajectoryController`:

```markdown
#### YZ Trajectory Execution with JointTrajectoryController

**Updated Implementation:**

With `JointTrajectoryController`, YZ trajectories can now be executed with proper interpolation:

```python
def execute_yz_trajectory(self, waypoints: List[Tuple[float, float]], duration_per_segment: float):
    """
    Execute YZ trajectory using JointTrajectoryController

    Args:
        waypoints: List of (y, z) positions for gripper and selector
        duration_per_segment: Time between waypoints
    """
    # Build trajectory for gripper (Y-axis)
    gripper_trajectory = JointTrajectory()
    gripper_trajectory.joint_names = ['selector_frame_gripper_joint']

    # Build trajectory for selector (Z-axis)
    selector_trajectory = JointTrajectory()
    selector_trajectory.joint_names = ['main_frame_selector_frame_joint']

    for i, (y, z) in enumerate(waypoints):
        time_from_start = Duration(sec=int((i + 1) * duration_per_segment))

        gripper_trajectory.points.append(
            JointTrajectoryPoint(positions=[y], velocities=[0.0], time_from_start=time_from_start)
        )
        selector_trajectory.points.append(
            JointTrajectoryPoint(positions=[z], velocities=[0.0], time_from_start=time_from_start)
        )

    # Send both trajectories (execute in parallel)
    gripper_goal = FollowJointTrajectory.Goal(trajectory=gripper_trajectory)
    selector_goal = FollowJointTrajectory.Goal(trajectory=selector_trajectory)

    # Non-blocking sends
    gripper_future = self.gripper_client.send_goal_async(gripper_goal)
    selector_future = self.selector_client.send_goal_async(selector_goal)

    # Wait for both to complete
    rclpy.spin_until_future_complete(self.node, gripper_future)
    rclpy.spin_until_future_complete(self.node, selector_future)
```

**Benefit:** Smooth, coordinated YZ motion with proper velocity profiles instead of step-wise position jumps.
```

### 4.5 NEW Section to Add: "Controller Interface API"

Add after Configuration Management section:

```markdown
### 12. ✅ Controller Interface API (Dual-Mode Support)

**Purpose:** Unified interface for commanding both trajectory and forward controllers.

**File:** `manipulator_control/src/controller_interface.py`

**API Overview:**

```python
class ControllerInterface:
    """
    Unified interface for commanding manipulator joints.

    Supports both:
    - JointTrajectoryController (7 motion joints) - Action-based
    - ForwardCommandController (2 container jaws) - Topic-based
    """

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

    def command_joint(self, joint_name: str, position: float,
                      duration_sec: float = None) -> bool:
        """
        Command a single joint to target position.

        Automatically routes to appropriate controller type.
        For trajectory joints, duration is auto-calculated if not provided.
        """

    def command_trajectory_with_callback(
        self, joint_name: str, position: float, duration_sec: float,
        result_callback: Callable = None,
        feedback_callback: Callable = None
    ) -> bool:
        """
        Send trajectory goal with callbacks for async operation.
        """

    def cancel_trajectory(self, joint_name: str) -> bool:
        """Cancel active trajectory goal for a joint."""

    def wait_for_action_servers(self, timeout_sec: float = 10.0) -> bool:
        """Wait for all trajectory action servers to be available."""
```

**Usage Examples:**

```python
# Initialize with parent ROS2 node
controller = ControllerInterface(node)

# Wait for action servers at startup
controller.wait_for_action_servers(timeout_sec=30.0)

# Command trajectory joint (auto-calculates duration)
controller.command_joint('base_main_frame_joint', 2.0)

# Command trajectory joint with explicit duration
controller.command_joint('base_main_frame_joint', 2.0, duration_sec=5.0)

# Command forward joint (container jaw)
controller.command_joint('selector_left_container_jaw_joint', 0.1)

# Async trajectory with callbacks
def on_result(success, error_msg):
    if success:
        print("Goal reached!")
    else:
        print(f"Failed: {error_msg}")

controller.command_trajectory_with_callback(
    'base_main_frame_joint', 2.0, 3.0,
    result_callback=on_result
)

# Command multiple joints
controller.command_joint_group(
    ['base_main_frame_joint', 'main_frame_selector_frame_joint'],
    [2.0, 1.0],
    duration_sec=3.0
)
```
```

---

## 5. Implementation Plan

### Step 1: Update Controller Configuration YAML

**File:** `ros2_ws/src/manipulator_description/config/manipulator_controllers.yaml`

**Full replacement content:**

```yaml
# ROS2 Control Configuration for Manipulator
# Hybrid Architecture: 7 JointTrajectoryControllers + 2 ForwardCommandControllers
#
# Updated: 2025-11-26
# Migration: ForwardCommandController → JointTrajectoryController for motion joints

controller_manager:
  ros__parameters:
    update_rate: 100  # Hz

    # Joint State Broadcaster (publishes /joint_states)
    joint_state_broadcaster:
      type: joint_state_broadcaster/JointStateBroadcaster

    # ============================================================
    # JOINT TRAJECTORY CONTROLLERS (7 motion joints)
    # ============================================================

    # BASE ASSEMBLY
    base_main_frame_joint_controller:
      type: joint_trajectory_controller/JointTrajectoryController

    # SELECTOR ASSEMBLY (excluding container jaws)
    main_frame_selector_frame_joint_controller:
      type: joint_trajectory_controller/JointTrajectoryController

    selector_frame_gripper_joint_controller:
      type: joint_trajectory_controller/JointTrajectoryController

    # PICKER ASSEMBLY
    selector_frame_picker_frame_joint_controller:
      type: joint_trajectory_controller/JointTrajectoryController

    picker_frame_picker_rail_joint_controller:
      type: joint_trajectory_controller/JointTrajectoryController

    picker_rail_picker_base_joint_controller:
      type: joint_trajectory_controller/JointTrajectoryController

    picker_base_picker_jaw_joint_controller:
      type: joint_trajectory_controller/JointTrajectoryController

    # ============================================================
    # FORWARD COMMAND CONTROLLERS (2 container jaws - UNCHANGED)
    # ============================================================

    selector_left_container_jaw_joint_controller:
      type: forward_command_controller/ForwardCommandController

    selector_right_container_jaw_joint_controller:
      type: forward_command_controller/ForwardCommandController

# ============================================================
# JOINT STATE BROADCASTER CONFIGURATION
# ============================================================

joint_state_broadcaster:
  ros__parameters:
    joints:
      - base_main_frame_joint
      - main_frame_selector_frame_joint
      - selector_left_container_jaw_joint
      - selector_right_container_jaw_joint
      - selector_frame_gripper_joint
      - selector_frame_picker_frame_joint
      - picker_frame_picker_rail_joint
      - picker_rail_picker_base_joint
      - picker_base_picker_jaw_joint

# ============================================================
# JOINT TRAJECTORY CONTROLLER CONFIGURATIONS
# ============================================================

# Base Assembly - X-axis railway (0.1-3.9m, 2.0 m/s max)
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

# Selector Assembly - Z-axis vertical (0.05-1.45m, 2.0 m/s max)
main_frame_selector_frame_joint_controller:
  ros__parameters:
    joints:
      - main_frame_selector_frame_joint
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
      main_frame_selector_frame_joint:
        trajectory: 0.1
        goal: 0.01

# Selector Assembly - Gripper Y-axis (-0.39 to 0.39m, 1.0 m/s max)
selector_frame_gripper_joint_controller:
  ros__parameters:
    joints:
      - selector_frame_gripper_joint
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
      selector_frame_gripper_joint:
        trajectory: 0.05
        goal: 0.01

# Picker Assembly - Z-axis vertical (0.005-0.29m, 1.0 m/s max)
selector_frame_picker_frame_joint_controller:
  ros__parameters:
    joints:
      - selector_frame_picker_frame_joint
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
      selector_frame_picker_frame_joint:
        trajectory: 0.05
        goal: 0.01

# Picker Assembly - Y-axis rail (-0.29 to 0.29m, 1.0 m/s max)
picker_frame_picker_rail_joint_controller:
  ros__parameters:
    joints:
      - picker_frame_picker_rail_joint
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
      picker_frame_picker_rail_joint:
        trajectory: 0.05
        goal: 0.01

# Picker Assembly - X-axis base slider (0.005-0.24m, 1.0 m/s max)
picker_rail_picker_base_joint_controller:
  ros__parameters:
    joints:
      - picker_rail_picker_base_joint
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
      picker_rail_picker_base_joint:
        trajectory: 0.05
        goal: 0.01

# Picker Assembly - Jaw extension (0.005-0.19m, 1.0 m/s max)
picker_base_picker_jaw_joint_controller:
  ros__parameters:
    joints:
      - picker_base_picker_jaw_joint
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
      picker_base_picker_jaw_joint:
        trajectory: 0.05
        goal: 0.01

# ============================================================
# FORWARD COMMAND CONTROLLER CONFIGURATIONS (Container Jaws)
# ============================================================

selector_left_container_jaw_joint_controller:
  ros__parameters:
    joints:
      - selector_left_container_jaw_joint
    interface_name: position

selector_right_container_jaw_joint_controller:
  ros__parameters:
    joints:
      - selector_right_container_jaw_joint
    interface_name: position
```

### Step 2: Verify Launch File (No Changes Expected)

**File:** `ros2_ws/src/manipulator_description/launch/manipulator_control.launch.py`

The launch file spawns controllers by name. Since controller names remain unchanged (only types changed in YAML), no modifications are needed.

**Verification checklist:**
- [ ] `controllers_to_spawn` list contains all 9 controller names
- [ ] Controllers spawned after `joint_state_broadcaster`
- [ ] Event handlers correctly trigger controller spawning

### Step 3: Update ControllerInterface

**File:** `ros2_ws/src/manipulator_control/src/controller_interface.py`

See Section 7 for complete updated code.

### Step 4: Build and Deploy

```bash
cd /home/robo/robo/ya_robot_manipulator/ros2_ws
colcon build --packages-select manipulator_description manipulator_control
source install/setup.bash
```

---

## 6. Configuration Changes Summary

### 6.1 Files Modified

| File | Change Type | Description |
|------|-------------|-------------|
| `config/manipulator_controllers.yaml` | **Major** | Controller types + trajectory params |
| `src/controller_interface.py` | **Major** | Dual-mode control (action + topic) |
| `launch/manipulator_control.launch.py` | **Verify only** | No changes expected |

### 6.2 Files NOT Modified

| File | Reason |
|------|--------|
| `urdf/manipulator/ros2_control.xacro` | Hardware interfaces already compatible |
| `config/manipulator_params.yaml` | Joint limits used as-is |
| `config/limit_switches.yaml` | End switch logic unchanged |

### 6.3 New ROS2 Topics/Actions

**Added Actions (7):**
```
/base_main_frame_joint_controller/follow_joint_trajectory
/main_frame_selector_frame_joint_controller/follow_joint_trajectory
/selector_frame_gripper_joint_controller/follow_joint_trajectory
/selector_frame_picker_frame_joint_controller/follow_joint_trajectory
/picker_frame_picker_rail_joint_controller/follow_joint_trajectory
/picker_rail_picker_base_joint_controller/follow_joint_trajectory
/picker_base_picker_jaw_joint_controller/follow_joint_trajectory
```

**Removed Topics (7):**
```
/base_main_frame_joint_controller/commands
/main_frame_selector_frame_joint_controller/commands
/selector_frame_gripper_joint_controller/commands
/selector_frame_picker_frame_joint_controller/commands
/picker_frame_picker_rail_joint_controller/commands
/picker_rail_picker_base_joint_controller/commands
/picker_base_picker_jaw_joint_controller/commands
```

**Unchanged Topics (2):**
```
/selector_left_container_jaw_joint_controller/commands
/selector_right_container_jaw_joint_controller/commands
```

---

## 7. Code Changes

### 7.1 Updated ControllerInterface

**File:** `ros2_ws/src/manipulator_control/src/controller_interface.py`

```python
#!/usr/bin/env python3
"""
Controller Interface Utility - Updated for JointTrajectoryController

Supports both:
- ForwardCommandController: Topic-based commands (Float64MultiArray)
- JointTrajectoryController: Action-based commands (FollowJointTrajectory)

Single Source of Truth: Joint soft limits loaded from manipulator_params.yaml
Topic Pattern (Forward): /{joint_name}_controller/commands
Action Pattern (Trajectory): /{joint_name}_controller/follow_joint_trajectory
"""

from typing import Dict, List, Optional, Tuple, Callable
import os
import yaml

from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.action import FollowJointTrajectory
from builtin_interfaces.msg import Duration
from ament_index_python.packages import get_package_share_directory


class ControllerInterface:
    """
    Utility for commanding both ForwardCommand and JointTrajectory controllers.

    This class is NOT a ROS2 node - it requires a parent node to be passed
    during initialization.
    """

    # Joints using ForwardCommandController (container jaws only)
    FORWARD_COMMAND_JOINTS = frozenset([
        'selector_left_container_jaw_joint',
        'selector_right_container_jaw_joint'
    ])

    # Joints using JointTrajectoryController
    TRAJECTORY_JOINTS = frozenset([
        'base_main_frame_joint',
        'main_frame_selector_frame_joint',
        'selector_frame_gripper_joint',
        'selector_frame_picker_frame_joint',
        'picker_frame_picker_rail_joint',
        'picker_rail_picker_base_joint',
        'picker_base_picker_jaw_joint'
    ])

    def __init__(self, node: Node):
        """
        Initialize ControllerInterface with reference to parent ROS2 node.

        Args:
            node: ROS2 node instance for creating publishers/action clients
        """
        self.node = node
        self.logger = node.get_logger()

        # Load joint limits from manipulator_params.yaml
        self.joint_limits = self._load_joint_limits_from_params()

        if not self.joint_limits:
            self.logger.error('No joint limits loaded - ControllerInterface not functional')
            return

        self.logger.info(f'Loaded limits for {len(self.joint_limits)} joints')

        # Create publishers for ForwardCommandController joints
        self.forward_publishers: Dict[str, any] = {}
        for joint_name in self.FORWARD_COMMAND_JOINTS:
            if joint_name in self.joint_limits:
                topic = f'/{joint_name}_controller/commands'
                self.forward_publishers[joint_name] = node.create_publisher(
                    Float64MultiArray, topic, 10
                )
                self.logger.debug(f'Created ForwardCommand publisher: {topic}')

        # Create action clients for JointTrajectoryController joints
        self.trajectory_clients: Dict[str, ActionClient] = {}
        self._trajectory_goals: Dict[str, any] = {}

        for joint_name in self.TRAJECTORY_JOINTS:
            if joint_name in self.joint_limits:
                action_name = f'/{joint_name}_controller/follow_joint_trajectory'
                self.trajectory_clients[joint_name] = ActionClient(
                    node,
                    FollowJointTrajectory,
                    action_name,
                    callback_group=ReentrantCallbackGroup()
                )
                self.logger.debug(f'Created Trajectory action client: {action_name}')

        # Subscribe to joint states
        self.joint_positions: Dict[str, float] = {}
        self.joint_velocities: Dict[str, float] = {}
        self._joint_state_sub = node.create_subscription(
            JointState,
            '/joint_states',
            self._joint_state_cb,
            10
        )

    def _load_joint_limits_from_params(self) -> Dict[str, Dict[str, float]]:
        """Load soft limits from manipulator_params.yaml."""
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
                        joint_name = key
                        sc = value['safety_controller']
                        limits[joint_name] = {
                            'min': sc['soft_lower'],
                            'max': sc['soft_upper'],
                            'velocity': value.get('limits', {}).get('velocity', 1.0),
                        }

            self.logger.info(f'Loaded joint limits from {params_file}')
            return limits

        except FileNotFoundError as e:
            self.logger.error(f'manipulator_params.yaml not found: {e}')
            return {}
        except Exception as e:
            self.logger.error(f'Error loading joint limits: {e}')
            return {}

    def _joint_state_cb(self, msg: JointState) -> None:
        """Update position/velocity cache from /joint_states."""
        for i, name in enumerate(msg.name):
            if i < len(msg.position):
                self.joint_positions[name] = msg.position[i]
            if i < len(msg.velocity):
                self.joint_velocities[name] = msg.velocity[i]

    def is_trajectory_joint(self, joint_name: str) -> bool:
        """Check if joint uses JointTrajectoryController."""
        return joint_name in self.TRAJECTORY_JOINTS

    def is_forward_joint(self, joint_name: str) -> bool:
        """Check if joint uses ForwardCommandController."""
        return joint_name in self.FORWARD_COMMAND_JOINTS

    def command_joint(self, joint_name: str, position: float,
                      duration_sec: float = None) -> bool:
        """
        Command a single joint to target position.

        For ForwardCommand joints: sends immediate position command
        For Trajectory joints: sends trajectory with calculated duration

        Args:
            joint_name: Name of joint to command
            position: Target position (validated against soft limits)
            duration_sec: Motion duration (auto-calculated if None)

        Returns:
            True if command sent successfully
        """
        # Validate joint exists
        if joint_name not in self.joint_limits:
            self.logger.warning(f'Unknown joint: {joint_name}')
            return False

        # Validate position against soft limits
        limits = self.joint_limits[joint_name]
        if not (limits['min'] <= position <= limits['max']):
            self.logger.warning(
                f"Position {position} outside limits [{limits['min']}, {limits['max']}] "
                f"for {joint_name}"
            )
            return False

        # Route to appropriate controller type
        if joint_name in self.FORWARD_COMMAND_JOINTS:
            return self._command_forward(joint_name, position)
        else:
            # Calculate duration if not provided
            if duration_sec is None:
                current = self.joint_positions.get(joint_name)
                if current is not None:
                    distance = abs(position - current)
                    velocity = limits['velocity']
                    duration_sec = max(0.5, (distance / velocity) * 1.2)
                else:
                    duration_sec = 2.0
            return self._command_trajectory(joint_name, position, duration_sec)

    def _command_forward(self, joint_name: str, position: float) -> bool:
        """Send command to ForwardCommandController."""
        if joint_name not in self.forward_publishers:
            self.logger.error(f'No publisher for: {joint_name}')
            return False

        msg = Float64MultiArray()
        msg.data = [float(position)]
        self.forward_publishers[joint_name].publish(msg)
        self.logger.debug(f'Forward command: {joint_name} -> {position}')
        return True

    def _command_trajectory(self, joint_name: str, position: float,
                           duration_sec: float) -> bool:
        """Send trajectory goal to JointTrajectoryController."""
        if joint_name not in self.trajectory_clients:
            self.logger.error(f'No action client for: {joint_name}')
            return False

        client = self.trajectory_clients[joint_name]

        if not client.wait_for_server(timeout_sec=1.0):
            self.logger.error(f'Action server not available: {joint_name}')
            return False

        # Build trajectory
        trajectory = JointTrajectory()
        trajectory.joint_names = [joint_name]

        point = JointTrajectoryPoint()
        point.positions = [position]
        point.velocities = [0.0]
        point.accelerations = [0.0]
        point.time_from_start = Duration(
            sec=int(duration_sec),
            nanosec=int((duration_sec % 1) * 1e9)
        )
        trajectory.points = [point]

        goal = FollowJointTrajectory.Goal()
        goal.trajectory = trajectory

        future = client.send_goal_async(goal)
        self._trajectory_goals[joint_name] = future

        self.logger.debug(f'Trajectory goal: {joint_name} -> {position} in {duration_sec}s')
        return True

    def command_trajectory_with_callback(
        self,
        joint_name: str,
        position: float,
        duration_sec: float,
        result_callback: Callable = None,
        feedback_callback: Callable = None
    ) -> bool:
        """
        Send trajectory goal with optional callbacks.

        Args:
            joint_name: Name of joint to command
            position: Target position
            duration_sec: Motion duration
            result_callback: Called when goal completes (success: bool, error_msg: str)
            feedback_callback: Called during execution with progress

        Returns:
            True if goal sent
        """
        if joint_name not in self.trajectory_clients:
            return False

        if joint_name in self.FORWARD_COMMAND_JOINTS:
            self.logger.warning(f'{joint_name} is ForwardCommand, no callbacks')
            return self._command_forward(joint_name, position)

        client = self.trajectory_clients[joint_name]

        if not client.wait_for_server(timeout_sec=1.0):
            return False

        trajectory = JointTrajectory()
        trajectory.joint_names = [joint_name]

        point = JointTrajectoryPoint()
        point.positions = [position]
        point.velocities = [0.0]
        point.time_from_start = Duration(
            sec=int(duration_sec),
            nanosec=int((duration_sec % 1) * 1e9)
        )
        trajectory.points = [point]

        goal = FollowJointTrajectory.Goal()
        goal.trajectory = trajectory

        future = client.send_goal_async(goal, feedback_callback=feedback_callback)

        if result_callback:
            future.add_done_callback(
                lambda f: self._handle_goal_response(f, result_callback)
            )

        return True

    def _handle_goal_response(self, future, result_callback):
        """Handle goal acceptance and get result."""
        goal_handle = future.result()
        if not goal_handle.accepted:
            result_callback(False, "Goal rejected")
            return

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(
            lambda f: result_callback(
                f.result().result.error_code == 0,
                f.result().result.error_string
            )
        )

    def cancel_trajectory(self, joint_name: str) -> bool:
        """Cancel active trajectory goal."""
        if joint_name not in self.trajectory_clients:
            return False

        if joint_name in self._trajectory_goals:
            future = self._trajectory_goals[joint_name]
            if future.done():
                goal_handle = future.result()
                if goal_handle and goal_handle.accepted:
                    goal_handle.cancel_goal_async()
                    return True
        return False

    def command_joint_group(self, joint_names: List[str], positions: List[float],
                           duration_sec: float = None) -> bool:
        """
        Command multiple joints simultaneously.

        Args:
            joint_names: List of joint names
            positions: List of target positions
            duration_sec: Motion duration for trajectory joints

        Returns:
            True if all commands sent
        """
        if len(joint_names) != len(positions):
            self.logger.warning('Mismatched joint_names and positions lengths')
            return False

        # Validate all joints first
        for joint_name, position in zip(joint_names, positions):
            if joint_name not in self.joint_limits:
                self.logger.warning(f'Unknown joint: {joint_name}')
                return False

            limits = self.joint_limits[joint_name]
            if not (limits['min'] <= position <= limits['max']):
                self.logger.warning(
                    f"Position {position} outside limits for {joint_name}"
                )
                return False

        # Send all commands
        for joint_name, position in zip(joint_names, positions):
            self.command_joint(joint_name, position, duration_sec)

        return True

    def get_joint_position(self, joint_name: str) -> Optional[float]:
        """Get current joint position."""
        return self.joint_positions.get(joint_name)

    def get_joint_velocity(self, joint_name: str) -> Optional[float]:
        """Get current joint velocity."""
        return self.joint_velocities.get(joint_name)

    def get_joint_limits(self, joint_name: str) -> Optional[Tuple[float, float]]:
        """Get soft limits (min, max) for a joint."""
        if joint_name not in self.joint_limits:
            return None
        limits = self.joint_limits[joint_name]
        return (limits['min'], limits['max'])

    def get_trajectory_joint_names(self) -> List[str]:
        """Get list of joints using JointTrajectoryController."""
        return [j for j in self.joint_limits.keys() if j in self.TRAJECTORY_JOINTS]

    def get_forward_joint_names(self) -> List[str]:
        """Get list of joints using ForwardCommandController."""
        return [j for j in self.joint_limits.keys() if j in self.FORWARD_COMMAND_JOINTS]

    def wait_for_action_servers(self, timeout_sec: float = 10.0) -> bool:
        """Wait for all trajectory action servers to be available."""
        for joint_name, client in self.trajectory_clients.items():
            if not client.wait_for_server(timeout_sec=timeout_sec):
                self.logger.error(f'Timeout waiting for {joint_name} action server')
                return False
        self.logger.info('All trajectory action servers available')
        return True
```

---

## 8. Testing Strategy

### 8.1 Build Verification

```bash
cd /home/robo/robo/ya_robot_manipulator/ros2_ws
colcon build --packages-select manipulator_description manipulator_control
source install/setup.bash
```

### 8.2 Controller Spawning Test

```bash
# Launch simulation
ros2 launch manipulator_description manipulator_control.launch.py

# Verify controller types
ros2 control list_controllers
```

**Expected output:**
```
joint_state_broadcaster          [joint_state_broadcaster/JointStateBroadcaster] active
base_main_frame_joint_controller [joint_trajectory_controller/JointTrajectoryController] active
main_frame_selector_frame_joint_controller [joint_trajectory_controller/JointTrajectoryController] active
selector_frame_gripper_joint_controller [joint_trajectory_controller/JointTrajectoryController] active
selector_frame_picker_frame_joint_controller [joint_trajectory_controller/JointTrajectoryController] active
picker_frame_picker_rail_joint_controller [joint_trajectory_controller/JointTrajectoryController] active
picker_rail_picker_base_joint_controller [joint_trajectory_controller/JointTrajectoryController] active
picker_base_picker_jaw_joint_controller [joint_trajectory_controller/JointTrajectoryController] active
selector_left_container_jaw_joint_controller [forward_command_controller/ForwardCommandController] active
selector_right_container_jaw_joint_controller [forward_command_controller/ForwardCommandController] active
```

### 8.3 Action Interface Test

```bash
# List available actions
ros2 action list

# Expected: 7 trajectory actions
# /base_main_frame_joint_controller/follow_joint_trajectory
# /main_frame_selector_frame_joint_controller/follow_joint_trajectory
# ...

# Test trajectory command
ros2 action send_goal /base_main_frame_joint_controller/follow_joint_trajectory \
  control_msgs/action/FollowJointTrajectory \
  "{trajectory: {joint_names: ['base_main_frame_joint'], points: [{positions: [2.0], velocities: [0.0], time_from_start: {sec: 3, nanosec: 0}}]}}"
```

### 8.4 ForwardCommand Test (Container Jaws)

```bash
# Test left container jaw (still topic-based)
ros2 topic pub --once /selector_left_container_jaw_joint_controller/commands \
  std_msgs/Float64MultiArray "{data: [0.1]}"
```

### 8.5 MoveJoint Integration Test

```bash
# Start action servers
ros2 run manipulator_control move_joint_server

# Test trajectory joint via MoveJoint
ros2 action send_goal /move_joint manipulator_control/action/MoveJoint \
  "{joint_name: 'base_main_frame_joint', target_position: 2.0}"

# Test forward joint via MoveJoint
ros2 action send_goal /move_joint manipulator_control/action/MoveJoint \
  "{joint_name: 'selector_left_container_jaw_joint', target_position: 0.1}"
```

---

## 9. Rollback Plan

If issues arise during or after migration:

### 9.1 Quick Rollback (YAML Only)

```bash
# Revert controller configuration
git checkout HEAD -- ros2_ws/src/manipulator_description/config/manipulator_controllers.yaml

# Rebuild
colcon build --packages-select manipulator_description

# Restart
ros2 launch manipulator_description manipulator_control.launch.py
```

### 9.2 Full Rollback (Including Code)

```bash
# Revert all changes
git checkout HEAD -- ros2_ws/src/manipulator_description/
git checkout HEAD -- ros2_ws/src/manipulator_control/

# Rebuild
colcon build

# Restart
ros2 launch manipulator_description manipulator_control.launch.py
```

---

## References

- [JointTrajectoryController Documentation (Jazzy)](https://control.ros.org/jazzy/doc/ros2_controllers/joint_trajectory_controller/doc/userdoc.html)
- [gz_ros2_control for Jazzy](https://control.ros.org/jazzy/doc/gz_ros2_control/doc/index.html)
- [ROS2 Control Demos (Jazzy)](https://control.ros.org/jazzy/doc/ros2_control_demos/example_1/doc/userdoc.html)
- [FollowJointTrajectory Action](https://docs.ros2.org/latest/api/control_msgs/action/FollowJointTrajectory.html)
