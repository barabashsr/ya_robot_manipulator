# Epic 7: Hardware Interface Architecture

**Document Version:** 1.0
**Date:** 2025-11-27
**Parent Document:** `docs/architecture-ros2-control-v2-CORRECTIONS.md`
**Tech Spec:** `docs/tech-spec-hardware-interface.md`

---

## Overview

This document provides comprehensive architecture guidance for implementing the ros2_control hardware interface that enables real hardware control via Modbus RTU. The implementation uses **Python** for rapid proof-of-concept development, with a clear path to C++ for production.

**Key Decision:** Python hardware interface using `ros2_control_py` framework for initial implementation, matching the existing working `examples/modbus_driver/` codebase.

---

## Table of Contents

1. [System Architecture](#system-architecture)
2. [ros2_control Hardware Interface Fundamentals](#ros2_control-hardware-interface-fundamentals)
3. [Python Hardware Interface Implementation](#python-hardware-interface-implementation)
4. [Modbus RTU Communication Layer](#modbus-rtu-communication-layer)
5. [URDF ros2_control Integration](#urdf-ros2_control-integration)
6. [Configuration Architecture](#configuration-architecture)
7. [Unit Conversion & Calibration](#unit-conversion--calibration)
8. [Error Handling & Recovery](#error-handling--recovery)
9. [Testing Strategy](#testing-strategy)
10. [Future: C++ Production Implementation](#future-c-production-implementation)
11. [References](#references)

---

## System Architecture

### High-Level Component Diagram

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                           ROS2 Control Framework                             │
├─────────────────────────────────────────────────────────────────────────────┤
│                                                                              │
│  ┌─────────────────────────────────────────────────────────────────────┐    │
│  │               controller_manager (10 Hz hardware / 100 Hz sim)       │    │
│  │  ┌─────────────────────┐  ┌─────────────────────┐                   │    │
│  │  │ JointTrajectory     │  │ ForwardCommand      │                   │    │
│  │  │ Controller (×7)     │  │ Controller (×2)     │                   │    │
│  │  │ - motion joints     │  │ - container jaws    │                   │    │
│  │  └─────────┬───────────┘  └─────────┬───────────┘                   │    │
│  └────────────┼────────────────────────┼───────────────────────────────┘    │
│               │                        │                                     │
│               ▼                        ▼                                     │
│  ┌─────────────────────────────────────────────────────────────────────┐    │
│  │                    Hardware Interface Layer                          │    │
│  │  ┌───────────────────────────────┐  ┌─────────────────────────────┐ │    │
│  │  │  ModbusHardwareInterface      │  │  MockSystemInterface        │ │    │
│  │  │  (Python - SystemInterface)   │  │  (Container jaws - mock)    │ │    │
│  │  │                               │  │                             │ │    │
│  │  │  State Interfaces:            │  │  State: position (echo)    │ │    │
│  │  │  - position (7 joints)        │  │  Command: position         │ │    │
│  │  │  - velocity (7 joints)        │  │                             │ │    │
│  │  │                               │  │  Note: mock for now         │ │    │
│  │  │  Command Interfaces:          │  └─────────────────────────────┘ │    │
│  │  │  - position (7 joints)        │                                   │    │
│  │  └───────────┬───────────────────┘                                   │    │
│  └──────────────┼───────────────────────────────────────────────────────┘    │
│                 │                                                            │
└─────────────────┼────────────────────────────────────────────────────────────┘
                  │
                  ▼
┌─────────────────────────────────────────────────────────────────────────────┐
│                         Modbus RTU Driver Layer                              │
│  ┌─────────────────────────────────────────────────────────────────────┐    │
│  │  ModbusDriver (minimalmodbus wrapper)                                │    │
│  │                                                                      │    │
│  │  - connect(port, baudrate)                                          │    │
│  │  - read_input_register(slave_id, register)   → FC4                  │    │
│  │  - write_holding_register(slave_id, register, value) → FC6          │    │
│  │  - read_device_status(slave_id) → error_code, ready, busy           │    │
│  └───────────┬─────────────────────────────────────────────────────────┘    │
└──────────────┼──────────────────────────────────────────────────────────────┘
               │
               ▼
        /dev/ttyACM0 (Serial RS-485)
               │
    ┌──────────┼──────────┬──────────┐
    ▼          ▼          ▼
┌────────┐ ┌────────┐ ┌────────┐
│Slave 1 │ │Slave 2 │ │Slave 3 │
│engineXZ│ │engineYB│ │engineACD│
│ X, Z   │ │ Y, B   │ │ A,C,D  │
│joints  │ │joints  │ │joints  │
└────────┘ └────────┘ └────────┘
```

### Joint-to-Hardware Mapping

| ROS2 Joint Name | Physical Axis | Slave ID | Device | Ordinate | Position Reg | Command Reg |
|-----------------|---------------|----------|--------|----------|--------------|-------------|
| `base_main_frame_joint` | X | 1 | engineXZ | ord1 | 1003 | 3005 |
| `main_frame_selector_frame_joint` | Z | 1 | engineXZ | ord2 | 1010 | 3015 |
| `selector_frame_gripper_joint` | Y | 2 | engineYB | ord1 | 1003 | 3005 |
| `selector_frame_picker_frame_joint` | A | 3 | engineACD | ord1 | 1003 | 3005 |
| `picker_frame_picker_rail_joint` | B | 2 | engineYB | ord2 | 1010 | 3015 |
| `picker_rail_picker_base_joint` | C | 3 | engineACD | ord2 | 1010 | 3015 |
| `picker_base_picker_jaw_joint` | D | 3 | engineACD | ord3 | 1017 | 3025 |
| `selector_left_container_jaw_joint` | - | - | **Mock** | - | - | - |
| `selector_right_container_jaw_joint` | - | - | **Mock** | - | - | - |

**Note:** Container jaws use mock interface for now (returns commanded position as state).

---

## Critical Hardware Constraints

### Axis Classification by Motion Type

| Axis Type | Joints | Motion Behavior | Position Feedback |
|-----------|--------|-----------------|-------------------|
| **Continuous** | X, Z, Y, A, B | Servo-like, move to exact position | Accurate encoder feedback |
| **Discrete** | C, D | Move until limit switch activates | Inaccurate during motion |

### Discrete Axes (C and D) - Special Handling Required

**Axes affected:**
- `picker_rail_picker_base_joint` (C axis) - Slave 3, ord2
- `picker_base_picker_jaw_joint` (D axis) - Slave 3, ord3

**Firmware Definition (from STM32 driver):**

```c
// examples/driver_stm32/Core/Inc/main.h
#define DISCRETTE 0   // Axis type for discrete mode - ON/OFF to limit
#define COORDINATE 1  // Axis type for continuous mode - servo positioning
```

**Behavior characteristics:**

1. **Binary ON/OFF control:** Discrete axes use binary 0/1 commands, NOT position values:
   - Command `0` → Move to MIN limit switch
   - Command `1` → Move to MAX limit switch
   - Motor runs until physical limit switch activates (not to exact position)

2. **Inaccurate position during motion:** Position register reports values during motion, but these are NOT accurate until motion completes and limit switch confirms position

3. **Busy blocking:** Device reports `module_is_busy=1` while motor is running. Interface MUST:
   - Poll busy status before sending new commands
   - Wait for motion completion (busy=0) before reading final position
   - NOT send new commands while busy

4. **Sequential operation constraint:** On Slave 3 (engineACD), only ONE axis can move at a time:
   - Cannot run A + C simultaneously
   - Cannot run A + D simultaneously
   - Cannot run C + D simultaneously
   - Must complete one motion before starting another

5. **Motion control options (two methods observed in Python driver):**

   **Method A - Via null_ord coil + direction coil:**
   - Set direction coil (e.g., `direction_ord2` = 2010) to 0 or 1
   - Write `true` to `null_ord` coil (e.g., 2009) to trigger motion toward selected limit
   - Used in: `move_ordinate_YB_to_limit_switch()`, `move_ordinate_A_to_limit_switch()`

   **Method B - Via target position register with 0/1 value:**
   - Write `0` or `1` to target position register (e.g., `ord2_given` = 3015)
   - Firmware interprets 0/1 as direction for discrete axes
   - Used in: `box_capture_activation()`, `throw_box_into_container()`

   **Recommended for ROS2:** Method B is simpler - write 0 or 1 to target register.
   The direction coil method may be needed if the simple 0/1 doesn't work reliably.

### Slave-Level Operation Constraints

| Slave | Device | Axes | Constraint |
|-------|--------|------|------------|
| 1 | engineXZ | X, Z | Can run X and Z simultaneously |
| 2 | engineYB | Y, B | Can run Y and B simultaneously |
| 3 | engineACD | A, C, D | **SEQUENTIAL ONLY** - one axis at a time |

### Hardware Interface Handling Strategy

**Design Philosophy:** Treat STM32 firmware as temporary solution. All adaptation happens in hardware interface layer - no firmware or ROS2 controller changes.

#### Critical Firmware Constraint: No Goal Override

**The firmware rejects new position commands if the previous goal hasn't been achieved.** This means we cannot send distant targets - we must use incremental commands only.

#### Incremental Position Commands (All Continuous Axes)

**Problem:** Firmware rejects commands if previous goal not reached.

**Solution:** Only send position deltas that can be achieved within one control cycle.

```python
def write(self, time, period):
    for joint_name, target_pos in self.hw_commands.items():
        joint_cfg = self.joint_configs[joint_name]

        if joint_cfg.get('discrete_axis', False):
            self._handle_discrete_axis_write(joint_name, joint_cfg, target_pos)
            continue

        # CONTINUOUS AXIS: Incremental commands only
        current_pos = self.hw_positions[joint_name]

        # Calculate max movement per cycle based on joint velocity limit
        max_delta = joint_cfg['max_velocity'] * period.nanoseconds / 1e9

        # Clamp command to incremental step
        delta = target_pos - current_pos
        if abs(delta) > max_delta:
            delta = max_delta if delta > 0 else -max_delta

        incremental_target = current_pos + delta

        # Send incremental target (firmware will accept it)
        self._send_position_command(joint_name, incremental_target)
```

**Key:** Never send a target that's further than what can be reached in one cycle. No queuing - commands are immediate or skipped.

#### Discrete Axes (C and D) - Position Readable During Motion

**Clarification:** Position register IS readable during discrete axis motion - it provides actual interpolated position. The axis just can't be controlled mid-motion.

**For discrete axes in `read()`:**

```python
def _handle_discrete_axis_read(self, joint_name, jcfg):
    """Read discrete axis - position IS readable during motion."""
    slave_id = jcfg['slave_id']

    # Read actual position from hardware (works during motion)
    position_pulses = self.modbus_driver.read_position(
        slave_id, jcfg['current_position_register']
    )

    # Convert to meters and return - ROS2 sees smooth feedback
    return position_pulses / jcfg['pulses_per_meter']
```

**For discrete axes in `write()`:**

```python
def _handle_discrete_axis_write(self, joint_name, jcfg, command):
    """Write discrete axis - binary 0/1 control."""
    slave_id = jcfg['slave_id']

    # Check if Slave 3 is busy (sequential constraint)
    if self._is_slave_busy(slave_id):
        return  # Skip this cycle, motion in progress

    # Threshold command to binary
    target_state = 1 if command >= 0.5 else 0

    # Check if already at target (via limit switches)
    at_min = self.modbus_driver.read_input_register(slave_id, jcfg['min_limit_register'])
    at_max = self.modbus_driver.read_input_register(slave_id, jcfg['max_limit_register'])

    current_state = 1 if at_max else 0

    if target_state == current_state:
        return  # Already there

    # Send binary command (0 or 1 to target register)
    self.modbus_driver.write_position(slave_id, jcfg['target_position_register'], target_state)
```

#### Control Flow Summary

```
read() cycle:
├── For each joint:
│   ├── Read position from hardware (all axes, including discrete during motion)
│   └── Update hw_positions[joint] - ROS2 sees smooth feedback
└── Return positions to ROS2

write() cycle:
├── For each joint with changed command:
│   ├── If discrete axis:
│   │   ├── Check slave busy → skip if busy
│   │   ├── Threshold to 0/1
│   │   ├── Check limit switches → skip if already there
│   │   └── Send binary command
│   └── If continuous axis:
│       ├── Calculate incremental delta (max per cycle)
│       ├── Clamp to max_velocity * period
│       └── Send incremental target (never distant goals)
└── Done (no queuing - immediate or skip)
```

### Limit Switch Interface

**Requirement:** Hardware interface must expose limit switch states for:
- Simulation parity (virtual limit switches already implemented)
- Safety monitoring
- Homing procedures
- **Discrete axis position feedback** (C and D axes determine position from limit switches)

**Limit Switch Register Map (from firmware):**

| Slave | Axis | Joint | MIN Register | MAX Register |
|-------|------|-------|--------------|--------------|
| 1 | X (ord1) | base_main_frame_joint | 1006 | 1007 |
| 1 | Z (ord2) | main_frame_selector_frame_joint | 1013 | 1014 |
| 2 | Y (ord1) | selector_frame_gripper_joint | 1006 | 1007 |
| 2 | B (ord2) | picker_frame_picker_rail_joint | 1013 | 1014 |
| 3 | A (ord1) | selector_frame_picker_frame_joint | 1006 | 1007 |
| 3 | C (ord2) | picker_rail_picker_base_joint | 1013 | 1014 |
| 3 | D (ord3) | picker_base_picker_jaw_joint | 1020 | 1021 |

**Register Pattern per Ordinate (Input Registers FC4):**

```yaml
# From examples/modbus_driver/configuration.yml
ordinate_1:
  min_limit_switch: 1006
  max_limit_switch: 1007
ordinate_2:
  min_limit_switch: 1013
  max_limit_switch: 1014
ordinate_3:
  min_limit_switch: 1020
  max_limit_switch: 1021
```

**Limit Switch Values:**
- `0` = Limit switch NOT activated (not at limit)
- `1` = Limit switch activated (at limit position)

**Reading Limit Switches:**

```python
# Read limit switch status using FC4 (read input register)
min_limit = modbus_driver.read_input_register(slave_id, min_limit_register)
max_limit = modbus_driver.read_input_register(slave_id, max_limit_register)

# For discrete axes: determine position from limit state
if min_limit:
    position = min_position  # e.g., 0.0 meters
elif max_limit:
    position = max_position  # e.g., 0.29 meters for C axis
else:
    position = last_known_position  # In transit (shouldn't happen for discrete)
```

**Interface exposure options:**

1. **State interfaces:** Export as `<joint>/limit_switch_min` and `<joint>/limit_switch_max`
2. **ROS2 topic:** Publish on `/limit_switches` as custom message
3. **Service:** Provide `/get_limit_switch_states` service

**Recommended:** State interfaces for consistency with ros2_control pattern.

**Critical for Discrete Axes:** For C and D axes, limit switches are the PRIMARY source of position truth. The position register is unreliable during motion.

### Configuration Update for Discrete Axes

```yaml
joints:
  picker_rail_picker_base_joint:  # C axis - DISCRETE MODE
    slave_id: 3
    ordinate: 2
    current_position_register: 1010  # Input FC4 (unreliable during motion)
    target_position_register: 3015   # Holding FC6 (triggers motion)
    speed_register: 3016
    # Discrete axis configuration
    discrete_axis: true              # Binary 0/1 control mode
    direction_coil: 2010             # Coil FC5 for direction control
    min_limit_register: 1013         # MIN limit switch status
    max_limit_register: 1014         # MAX limit switch status
    min_position: 0.0                # Position value when at MIN limit
    max_position: 0.29               # Position value when at MAX limit (meters)

  picker_base_picker_jaw_joint:   # D axis - DISCRETE MODE
    slave_id: 3
    ordinate: 3
    current_position_register: 1017  # Input FC4 (unreliable during motion)
    target_position_register: 3025   # Holding FC6 (triggers motion)
    speed_register: 3026
    # Discrete axis configuration
    discrete_axis: true              # Binary 0/1 control mode
    direction_coil: 2015             # Coil FC5 for direction control
    min_limit_register: 1020         # MIN limit switch status
    max_limit_register: 1021         # MAX limit switch status
    min_position: 0.0                # Position value when at MIN limit
    max_position: 0.19               # Position value when at MAX limit (meters)
```

### Discrete Axis Command Flow

```
ROS2 Command (0.0 or 1.0)
        │
        ▼
┌───────────────────────────────┐
│  Threshold: cmd >= 0.5 → 1    │
│             cmd <  0.5 → 0    │
└───────────────────────────────┘
        │
        ▼
┌───────────────────────────────┐
│  Check module_is_busy (1002)  │
│  If busy → defer to next cycle│
└───────────────────────────────┘
        │ (not busy)
        ▼
┌───────────────────────────────┐
│  Check current limit switches │
│  If already at target → skip  │
└───────────────────────────────┘
        │ (need to move)
        ▼
┌───────────────────────────────┐
│  Write target register (FC6)  │
│  Write 0 or 1 to trigger      │
│  e.g., ord2_given (3015)      │
└───────────────────────────────┘
        │
        ▼
    Motor runs until limit switch
        │
        ▼
┌───────────────────────────────┐
│  module_is_busy → 0           │
│  Read limit switch for state  │
└───────────────────────────────┘
```

**Note:** The Python driver shows two methods for discrete axis control. Method B (writing 0/1 to target register) is simpler and recommended. If this doesn't work reliably, fall back to Method A (direction coil + null_ord coil).

### Graceful Controller Handling

**Problem:** JointTrajectoryController expects smooth, continuous position feedback. Discrete axes violate this expectation.

**Solutions:**

1. **Position hold during motion:** Report last known good position until motion completes
2. **Velocity zeroing:** Report velocity=0 during discrete motion (not estimable)
3. **Goal tolerance relaxation:** Configure larger goal tolerance for discrete axes
4. **Separate controller:** Use ForwardCommandController for discrete axes instead of JointTrajectoryController

**Recommended:** Position hold + goal tolerance relaxation for MVP.

---

## ros2_control Hardware Interface Fundamentals

### What is a Hardware Interface?

In ros2_control, hardware components are **plugins** dynamically loaded by the `controller_manager`. They abstract the physical hardware behind a standardized API of **state interfaces** (read-only sensor data) and **command interfaces** (writable control outputs).

### Interface Types

| Type | Base Class | Use Case |
|------|------------|----------|
| **SystemInterface** | `hardware_interface.SystemInterface` | Multi-joint systems (robots) |
| **ActuatorInterface** | `hardware_interface.ActuatorInterface` | Single actuator |
| **SensorInterface** | `hardware_interface.SensorInterface` | Read-only sensors |

**Our choice:** `SystemInterface` - controls 7 motion joints as a coordinated system.

### Lifecycle States

The hardware interface follows the ROS2 lifecycle pattern:

```
                    ┌─────────────────┐
                    │   UNCONFIGURED  │ ← on_init()
                    │  (initialized)  │
                    └────────┬────────┘
                             │ on_configure()
                             ▼
                    ┌─────────────────┐
                    │    INACTIVE     │ ← Communication started
                    │ (states readable│   Command interfaces NOT available
                    │  commands NOT)  │
                    └────────┬────────┘
                             │ on_activate()
                             ▼
                    ┌─────────────────┐
                    │     ACTIVE      │ ← Full operation
                    │ (read + write)  │   read() and write() called at 10Hz (hardware)
                    └────────┬────────┘
                             │ on_deactivate()
                             ▼
                    ┌─────────────────┐
                    │    INACTIVE     │
                    └────────┬────────┘
                             │ on_cleanup()
                             ▼
                    ┌─────────────────┐
                    │   UNCONFIGURED  │
                    └─────────────────┘
```

### Control Loop (called by controller_manager)

**Update Rate Decision:** 10 Hz for hardware mode (vs 100 Hz simulation)

| Mode | Update Rate | Rationale |
|------|-------------|-----------|
| Simulation | 100 Hz | No I/O latency, Gazebo physics timing |
| Hardware | 10 Hz | Modbus RTU timing: 7 reads × ~5ms = 35ms fits in 100ms budget |

**Timing Budget Analysis (10 Hz = 100ms/cycle):**
- 7 position reads (FC4): 7 × 5ms = 35ms
- 7 command writes (FC6): 7 × 5ms = 35ms (worst case, all changed)
- Overhead/margin: 30ms
- **Total: 100ms budget satisfied**

**Future optimization path:** Multi-register reads or C++ implementation for higher rates.

```python
# Pseudo-code of controller_manager loop (10 Hz for hardware)
while running:
    # 1. READ: Get current state from hardware
    hardware_interface.read(current_time, period)

    # 2. UPDATE: Controllers compute commands from states
    for controller in active_controllers:
        controller.update(current_time, period)

    # 3. WRITE: Send commands to hardware
    hardware_interface.write(current_time, period)

    sleep_until_next_cycle()
```

---

## Python Hardware Interface Implementation

### Package Structure

```
ros2_ws/src/manipulator_hardware/
├── CMakeLists.txt
├── package.xml
├── setup.py                              # Python package setup
├── manipulator_hardware.xml              # pluginlib export
├── manipulator_hardware/
│   ├── __init__.py
│   ├── modbus_hardware_interface.py      # Main hardware interface
│   └── modbus_driver.py                  # Modbus communication layer
├── config/
│   └── hardware_config.yaml              # Hardware configuration
└── test/
    ├── test_modbus_driver.py
    └── test_unit_conversion.py
```

### Main Hardware Interface Class

```python
# manipulator_hardware/modbus_hardware_interface.py

import rclpy
from rclpy.node import Node
from hardware_interface import SystemInterface
from hardware_interface.types import HardwareInfo, CallbackReturn, return_type
import yaml
from pathlib import Path
from typing import List, Dict, Any, Optional
from .modbus_driver import ModbusDriver


class ModbusHardwareInterface(SystemInterface):
    """
    ros2_control hardware interface for Modbus RTU motor controllers.

    Implements SystemInterface to control 7 motion joints via Modbus RTU
    and 2 container jaw joints via mock (echoed commands).

    Reference: examples/modbus_driver/ for Modbus patterns
    Reference: https://control.ros.org/jazzy/doc/ros2_control/hardware_interface/doc/writing_new_hardware_component.html
    """

    def __init__(self):
        super().__init__()
        self.logger = None
        self.modbus_driver: Optional[ModbusDriver] = None
        self.config: Dict[str, Any] = {}

        # Joint configuration loaded from YAML
        self.joint_configs: Dict[str, Dict[str, Any]] = {}

        # State storage (updated in read(), exposed via state interfaces)
        self.hw_positions: Dict[str, float] = {}      # Current positions [meters]
        self.hw_velocities: Dict[str, float] = {}     # Current velocities [m/s]

        # Command storage (written by controllers, sent in write())
        self.hw_commands: Dict[str, float] = {}       # Commanded positions [meters]
        self.hw_commands_prev: Dict[str, float] = {}  # Previous commands (for change detection)

        # Mock joints (container jaws)
        self.mock_joints = [
            'selector_left_container_jaw_joint',
            'selector_right_container_jaw_joint'
        ]

    # =========================================================================
    # Lifecycle Callbacks
    # =========================================================================

    def on_init(self, hardware_info: HardwareInfo) -> CallbackReturn:
        """
        Initialize hardware interface from URDF ros2_control tags.

        Called once when controller_manager loads the plugin.
        Parse hardware_info, load configuration, allocate memory.

        Args:
            hardware_info: Parsed from URDF <ros2_control> tags

        Returns:
            CallbackReturn.SUCCESS if initialization successful
            CallbackReturn.ERROR if critical failure
        """
        # Call parent to set up base interface descriptions
        if super().on_init(hardware_info) != CallbackReturn.SUCCESS:
            return CallbackReturn.ERROR

        self.logger = rclpy.logging.get_logger('ModbusHardwareInterface')
        self.logger.info('on_init: Initializing Modbus hardware interface')

        # Load configuration file path from URDF parameters
        config_file = None
        for param in hardware_info.hardware_parameters:
            if param.name == 'config_file':
                config_file = param.value
                break

        if not config_file:
            self.logger.error('on_init: config_file parameter not specified in URDF')
            return CallbackReturn.ERROR

        # Load YAML configuration
        try:
            self.config = self._load_config(config_file)
            self.logger.info(f'on_init: Loaded configuration from {config_file}')
        except Exception as e:
            self.logger.error(f'on_init: Failed to load config: {e}')
            return CallbackReturn.ERROR

        # Initialize joint state/command storage
        for joint in hardware_info.joints:
            joint_name = joint.name
            self.hw_positions[joint_name] = 0.0
            self.hw_velocities[joint_name] = 0.0
            self.hw_commands[joint_name] = 0.0
            self.hw_commands_prev[joint_name] = 0.0

            # Load joint-specific config (skip mock joints)
            if joint_name not in self.mock_joints:
                if joint_name in self.config.get('joints', {}):
                    self.joint_configs[joint_name] = self.config['joints'][joint_name]
                else:
                    self.logger.warn(f'on_init: No config for joint {joint_name}')

        self.logger.info(f'on_init: Configured {len(self.joint_configs)} Modbus joints, '
                        f'{len(self.mock_joints)} mock joints')

        return CallbackReturn.SUCCESS

    def on_configure(self, previous_state) -> CallbackReturn:
        """
        Configure hardware communication.

        Called when transitioning UNCONFIGURED → INACTIVE.
        Establish Modbus connection, verify communication with all devices.

        Args:
            previous_state: Previous lifecycle state

        Returns:
            CallbackReturn.SUCCESS if all devices reachable
            CallbackReturn.ERROR if connection fails
        """
        self.logger.info('on_configure: Establishing Modbus connection')

        modbus_config = self.config.get('modbus', {})
        port = modbus_config.get('port', '/dev/ttyACM0')
        baudrate = modbus_config.get('baudrate', 115200)

        # Create Modbus driver
        self.modbus_driver = ModbusDriver()

        try:
            if not self.modbus_driver.connect(port, baudrate):
                self.logger.error(f'on_configure: Failed to connect to {port}')
                return CallbackReturn.ERROR

            self.logger.info(f'on_configure: Connected to {port} at {baudrate} baud')
        except Exception as e:
            self.logger.error(f'on_configure: Connection error: {e}')
            return CallbackReturn.ERROR

        # Verify communication with each slave device
        slave_ids = set()
        for joint_name, jcfg in self.joint_configs.items():
            slave_ids.add(jcfg['slave_id'])

        for slave_id in slave_ids:
            try:
                ready = self.modbus_driver.is_device_ready(slave_id)
                if ready is None:
                    self.logger.warn(f'on_configure: Slave {slave_id} not responding')
                else:
                    self.logger.info(f'on_configure: Slave {slave_id} ready={ready}')
            except Exception as e:
                self.logger.error(f'on_configure: Slave {slave_id} check failed: {e}')
                return CallbackReturn.ERROR

        # Read initial positions
        self._read_all_positions()

        # Initialize commands to current positions (prevents jump on activate)
        for joint_name in self.hw_positions:
            self.hw_commands[joint_name] = self.hw_positions[joint_name]
            self.hw_commands_prev[joint_name] = self.hw_positions[joint_name]

        return CallbackReturn.SUCCESS

    def on_activate(self, previous_state) -> CallbackReturn:
        """
        Activate hardware for control.

        Called when transitioning INACTIVE → ACTIVE.
        Command interfaces become available to controllers.

        Args:
            previous_state: Previous lifecycle state

        Returns:
            CallbackReturn.SUCCESS
        """
        self.logger.info('on_activate: Hardware interface activated')

        # Sync commands to current positions
        for joint_name in self.hw_positions:
            self.hw_commands[joint_name] = self.hw_positions[joint_name]
            self.hw_commands_prev[joint_name] = self.hw_positions[joint_name]

        return CallbackReturn.SUCCESS

    def on_deactivate(self, previous_state) -> CallbackReturn:
        """
        Deactivate hardware control.

        Called when transitioning ACTIVE → INACTIVE.
        Hold current position, command interfaces become unavailable.

        Args:
            previous_state: Previous lifecycle state

        Returns:
            CallbackReturn.SUCCESS
        """
        self.logger.info('on_deactivate: Hardware interface deactivated')
        return CallbackReturn.SUCCESS

    def on_cleanup(self, previous_state) -> CallbackReturn:
        """
        Cleanup hardware resources.

        Called when transitioning INACTIVE → UNCONFIGURED.
        Disconnect from Modbus devices.

        Args:
            previous_state: Previous lifecycle state

        Returns:
            CallbackReturn.SUCCESS
        """
        self.logger.info('on_cleanup: Disconnecting Modbus')

        if self.modbus_driver:
            self.modbus_driver.disconnect()
            self.modbus_driver = None

        return CallbackReturn.SUCCESS

    def on_error(self, previous_state) -> CallbackReturn:
        """
        Handle error state.

        Called when error occurs during operation.
        Attempt graceful shutdown.

        Args:
            previous_state: State when error occurred

        Returns:
            CallbackReturn.SUCCESS if recovery possible
        """
        self.logger.error(f'on_error: Error from state {previous_state}')

        if self.modbus_driver:
            self.modbus_driver.disconnect()

        return CallbackReturn.SUCCESS

    # =========================================================================
    # Interface Export (called by framework after on_init)
    # =========================================================================

    def export_state_interfaces(self) -> List:
        """
        Export state interfaces for all joints.

        State interfaces are read-only and updated in read().
        Controllers and joint_state_broadcaster read from these.

        Returns:
            List of StateInterface objects
        """
        state_interfaces = []

        for joint_name in self.hw_positions.keys():
            # Position interface
            state_interfaces.append(
                self.create_state_interface(
                    joint_name, 'position',
                    lambda jn=joint_name: self.hw_positions[jn]
                )
            )
            # Velocity interface
            state_interfaces.append(
                self.create_state_interface(
                    joint_name, 'velocity',
                    lambda jn=joint_name: self.hw_velocities[jn]
                )
            )

        self.logger.info(f'export_state_interfaces: Exported {len(state_interfaces)} interfaces')
        return state_interfaces

    def export_command_interfaces(self) -> List:
        """
        Export command interfaces for all joints.

        Command interfaces are writable by controllers.
        Values are sent to hardware in write().

        Returns:
            List of CommandInterface objects
        """
        command_interfaces = []

        for joint_name in self.hw_commands.keys():
            command_interfaces.append(
                self.create_command_interface(
                    joint_name, 'position',
                    lambda jn=joint_name: self.hw_commands[jn],
                    lambda val, jn=joint_name: self._set_command(jn, val)
                )
            )

        self.logger.info(f'export_command_interfaces: Exported {len(command_interfaces)} interfaces')
        return command_interfaces

    def _set_command(self, joint_name: str, value: float):
        """Callback for command interface writes."""
        self.hw_commands[joint_name] = value

    # =========================================================================
    # Control Loop Methods (called at 10 Hz for hardware mode)
    # =========================================================================

    def read(self, time, period) -> return_type:
        """
        Read current state from hardware.

        Called every control cycle (10 Hz hardware) BEFORE controllers update.
        Read positions from Modbus registers, update state interfaces.

        Args:
            time: Current ROS time
            period: Time since last call

        Returns:
            return_type.OK on success
            return_type.ERROR on critical failure
        """
        try:
            self._read_all_positions()
            self._calculate_velocities(period)
            return return_type.OK
        except Exception as e:
            self.logger.error(f'read: Error reading positions: {e}')
            return return_type.ERROR

    def write(self, time, period) -> return_type:
        """
        Write commands to hardware.

        Called every control cycle (10 Hz hardware) AFTER controllers update.
        Send commanded positions to Modbus registers.

        Args:
            time: Current ROS time
            period: Time since last call

        Returns:
            return_type.OK on success
            return_type.ERROR on critical failure
        """
        try:
            self._write_changed_commands()
            return return_type.OK
        except Exception as e:
            self.logger.error(f'write: Error writing commands: {e}')
            return return_type.ERROR

    # =========================================================================
    # Internal Methods
    # =========================================================================

    def _load_config(self, config_file: str) -> Dict[str, Any]:
        """Load YAML configuration file."""
        path = Path(config_file)
        if not path.exists():
            raise FileNotFoundError(f'Config file not found: {config_file}')

        with open(path, 'r') as f:
            return yaml.safe_load(f)

    def _read_all_positions(self):
        """Read positions from all Modbus joints."""
        for joint_name, jcfg in self.joint_configs.items():
            pulses = self.modbus_driver.read_input_register(
                jcfg['slave_id'],
                jcfg['position_register']
            )

            if pulses is not None:
                # Convert pulses to meters
                self.hw_positions[joint_name] = self._pulses_to_meters(
                    pulses, jcfg
                )
            else:
                self.logger.warn(f'_read_all_positions: Failed to read {joint_name}')

        # Mock joints: echo commanded position
        for joint_name in self.mock_joints:
            self.hw_positions[joint_name] = self.hw_commands[joint_name]

    def _calculate_velocities(self, period):
        """Calculate velocities from position change."""
        dt = period.nanoseconds / 1e9  # Convert to seconds
        if dt <= 0:
            return

        for joint_name in self.hw_positions:
            # Simple finite difference
            # Note: For better accuracy, use encoder velocity if available
            prev_pos = getattr(self, '_prev_positions', {}).get(joint_name, 0.0)
            self.hw_velocities[joint_name] = (
                self.hw_positions[joint_name] - prev_pos
            ) / dt

        # Store current positions for next cycle
        self._prev_positions = dict(self.hw_positions)

    def _write_changed_commands(self):
        """Write commands only if changed (optimization)."""
        for joint_name, jcfg in self.joint_configs.items():
            cmd = self.hw_commands[joint_name]
            prev_cmd = self.hw_commands_prev[joint_name]

            # Only write if command changed (reduces bus traffic)
            if abs(cmd - prev_cmd) > 1e-6:
                pulses = self._meters_to_pulses(cmd, jcfg)

                success = self.modbus_driver.write_holding_register(
                    jcfg['slave_id'],
                    jcfg['command_register'],
                    pulses
                )

                if success:
                    self.hw_commands_prev[joint_name] = cmd
                else:
                    self.logger.warn(f'_write_changed_commands: Failed to write {joint_name}')

        # Mock joints: nothing to write (state echoes command)

    def _pulses_to_meters(self, pulses: int, jcfg: Dict) -> float:
        """Convert encoder pulses to meters."""
        pulses_per_meter = jcfg.get('pulses_per_meter', 100000)
        direction = jcfg.get('direction', 1)
        return (pulses / pulses_per_meter) * direction

    def _meters_to_pulses(self, meters: float, jcfg: Dict) -> int:
        """Convert meters to encoder pulses."""
        pulses_per_meter = jcfg.get('pulses_per_meter', 100000)
        direction = jcfg.get('direction', 1)
        return int(meters * pulses_per_meter * direction)
```

### Plugin Export for pluginlib

```xml
<!-- manipulator_hardware.xml -->
<library path="manipulator_hardware">
  <class name="manipulator_hardware/ModbusHardwareInterface"
         type="manipulator_hardware.modbus_hardware_interface.ModbusHardwareInterface"
         base_class_type="hardware_interface::SystemInterface">
    <description>
      Modbus RTU hardware interface for ya_robot_manipulator.
      Controls 7 motion joints via Modbus, 2 container jaws via mock.
    </description>
  </class>
</library>
```

### CMakeLists.txt for Python Package

```cmake
cmake_minimum_required(VERSION 3.8)
project(manipulator_hardware)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

# Install config files
install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

# Export plugin
pluginlib_export_plugin_description_file(hardware_interface manipulator_hardware.xml)

ament_package()
```

### package.xml

```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>manipulator_hardware</name>
  <version>0.1.0</version>
  <description>Modbus RTU hardware interface for ya_robot_manipulator</description>
  <maintainer email="bmad@example.com">BMad</maintainer>
  <license>MIT</license>

  <buildtool_depend>ament_cmake</buildtool_depend>
  <buildtool_depend>ament_cmake_python</buildtool_depend>

  <depend>rclpy</depend>
  <depend>hardware_interface</depend>
  <depend>pluginlib</depend>
  <depend>controller_manager</depend>

  <exec_depend>python3-minimalmodbus</exec_depend>
  <exec_depend>python3-yaml</exec_depend>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
```

---

## Modbus RTU Communication Layer

### ModbusDriver Class

Based on working `examples/modbus_driver/modbus_rtu.py`:

```python
# manipulator_hardware/modbus_driver.py

import minimalmodbus
import serial
from typing import Optional
import threading


class ModbusDriver:
    """
    Modbus RTU communication driver.

    Wraps minimalmodbus library for serial communication with
    stepper motor controllers.

    Reference: examples/modbus_driver/modbus_rtu.py

    Modbus Function Codes Used:
    - FC3: Read Holding Registers (read command echo)
    - FC4: Read Input Registers (read position, status)
    - FC6: Write Single Holding Register (write command)
    """

    def __init__(self):
        self._instruments: dict[int, minimalmodbus.Instrument] = {}
        self._port: Optional[str] = None
        self._baudrate: int = 115200
        self._lock = threading.Lock()  # Thread safety for shared serial

    def connect(self, port: str, baudrate: int = 115200) -> bool:
        """
        Establish serial connection.

        Args:
            port: Serial port (e.g., '/dev/ttyACM0')
            baudrate: Baud rate (default 115200)

        Returns:
            True if connection successful
        """
        self._port = port
        self._baudrate = baudrate

        # Test connection by creating instrument for slave 1
        try:
            self._get_instrument(1)
            return True
        except Exception:
            return False

    def disconnect(self):
        """Close serial connection."""
        with self._lock:
            for instrument in self._instruments.values():
                try:
                    instrument.serial.close()
                except Exception:
                    pass
            self._instruments.clear()

    def _get_instrument(self, slave_id: int) -> minimalmodbus.Instrument:
        """
        Get or create instrument for slave device.

        minimalmodbus uses one Instrument per slave address,
        but they share the serial port.
        """
        if slave_id not in self._instruments:
            instrument = minimalmodbus.Instrument(self._port, slave_id)
            instrument.serial.baudrate = self._baudrate
            instrument.serial.bytesize = 8
            instrument.serial.parity = serial.PARITY_NONE
            instrument.serial.stopbits = 1
            instrument.serial.timeout = 0.5  # 500ms timeout
            instrument.mode = minimalmodbus.MODE_RTU
            instrument.clear_buffers_before_each_transaction = True
            self._instruments[slave_id] = instrument

        return self._instruments[slave_id]

    # =========================================================================
    # Register Read Operations
    # =========================================================================

    def read_input_register(self, slave_id: int, register: int) -> Optional[int]:
        """
        Read single input register (FC4).

        Input registers are read-only, typically sensor data.
        Used for: current position, error codes, status.

        Args:
            slave_id: Modbus slave address (1, 2, or 3)
            register: Register address

        Returns:
            Register value (unsigned 16-bit) or None on error
        """
        with self._lock:
            try:
                instrument = self._get_instrument(slave_id)
                return instrument.read_register(
                    register,
                    number_of_decimals=0,
                    functioncode=4,  # FC4: Read Input Registers
                    signed=False
                )
            except Exception as e:
                # Log but don't raise - allows retry next cycle
                return None

    def read_holding_register(self, slave_id: int, register: int) -> Optional[int]:
        """
        Read single holding register (FC3).

        Holding registers are read/write, typically commands.
        Used for: reading back commanded values.

        Args:
            slave_id: Modbus slave address
            register: Register address

        Returns:
            Register value or None on error
        """
        with self._lock:
            try:
                instrument = self._get_instrument(slave_id)
                return instrument.read_register(
                    register,
                    number_of_decimals=0,
                    functioncode=3,  # FC3: Read Holding Registers
                    signed=False
                )
            except Exception:
                return None

    # =========================================================================
    # Register Write Operations
    # =========================================================================

    def write_holding_register(self, slave_id: int, register: int,
                                value: int) -> bool:
        """
        Write single holding register (FC6).

        Args:
            slave_id: Modbus slave address
            register: Register address
            value: Value to write (unsigned 16-bit)

        Returns:
            True if write successful
        """
        with self._lock:
            try:
                instrument = self._get_instrument(slave_id)
                instrument.write_register(
                    register,
                    value,
                    number_of_decimals=0,
                    functioncode=6,  # FC6: Write Single Register
                    signed=False
                )
                return True
            except Exception:
                return False

    # =========================================================================
    # Status Queries
    # =========================================================================

    def read_error_code(self, slave_id: int, register: int = 999) -> Optional[int]:
        """
        Read device error code.

        Args:
            slave_id: Modbus slave address
            register: Error code register (default 999 from config)

        Returns:
            Error code (0 = no error) or None on comm failure
        """
        return self.read_input_register(slave_id, register)

    def is_device_ready(self, slave_id: int, register: int = 1001) -> Optional[bool]:
        """
        Check if device is ready for commands.

        Args:
            slave_id: Modbus slave address
            register: Ready status register (default 1001)

        Returns:
            True if ready, False if not ready, None on comm failure
        """
        value = self.read_input_register(slave_id, register)
        if value is None:
            return None
        return bool(value)  # 1 = ready, 0 = not ready

    def is_device_busy(self, slave_id: int, register: int = 1002) -> Optional[bool]:
        """
        Check if device is currently executing command.

        Args:
            slave_id: Modbus slave address
            register: Busy status register (default 1002)

        Returns:
            True if busy, False if idle, None on comm failure
        """
        value = self.read_input_register(slave_id, register)
        if value is None:
            return None
        return bool(value)  # 1 = busy, 0 = idle
```

### Modbus Register Map Reference (Updated)

**Source:** STM32 firmware and `examples/modbus_driver/configuration.yml`

#### Input Registers (Read-Only) - FC4

| Address | Name | Type | Description |
|---------|------|------|-------------|
| 999 | err_code | int | Error/status code (0 = OK) |
| 1001 | module_ready | bool | Module ready for commands (1 = ready) |
| 1002 | module_is_busy | bool | Module busy executing (1 = busy) |
| **Ordinate 1** ||||
| 1003 | ord1_current | int | Current position (pulses) |
| 1004 | ord1_drive_active | bool | Drive active status |
| 1005 | ord1_brake_active | bool | Brake active status |
| 1006 | min_limit_switch_ord1 | bool | MIN limit switch (1 = at limit) |
| 1007 | max_limit_switch_ord1 | bool | MAX limit switch (1 = at limit) |
| **Ordinate 2** ||||
| 1010 | ord2_current | int | Current position (pulses) |
| 1011 | ord2_drive_active | bool | Drive active status |
| 1012 | ord2_brake_active | bool | Brake active status |
| 1013 | min_limit_switch_ord2 | bool | MIN limit switch (1 = at limit) |
| 1014 | max_limit_switch_ord2 | bool | MAX limit switch (1 = at limit) |
| **Ordinate 3** ||||
| 1017 | ord3_current | int | Current position (pulses) |
| 1018 | ord3_drive_active | bool | Drive active status |
| 1019 | ord3_brake_active | bool | Brake active status |
| 1020 | min_limit_switch_ord3 | bool | MIN limit switch (1 = at limit) |
| 1021 | max_limit_switch_ord3 | bool | MAX limit switch (1 = at limit) |
| **Discrete Inputs** ||||
| 1024 | discrete_input1 | bool | Discrete input 1 |
| 1025 | discrete_input2 | bool | Discrete input 2 |

#### Holding Registers (Read/Write) - FC3/FC6

| Address | Name | Type | Description |
|---------|------|------|-------------|
| 2999 | module_address | int | Module Modbus address |
| **Ordinate 1** ||||
| 3003 | ord1_name | char | Axis name (ASCII) |
| 3004 | ord1_type | int | Axis type (0=DISCRETE, 1=COORDINATE) |
| 3005 | ord1_given | int | Target position (pulses) |
| 3006 | ord1_speed_given | int | Speed (pulses/sec) |
| 3007 | ord1_pulse_turn | int | Pulses per revolution |
| 3008 | ord1_acl_dcl_point | int | Acceleration/deceleration |
| **Ordinate 2** ||||
| 3013 | ord2_name | char | Axis name (ASCII) |
| 3014 | ord2_type | int | Axis type (0=DISCRETE, 1=COORDINATE) |
| 3015 | ord2_given | int | Target position (pulses) |
| 3016 | ord2_speed_given | int | Speed (pulses/sec) |
| 3017 | ord2_pulse_turn | int | Pulses per revolution |
| 3018 | ord2_acl_dcl_point | int | Acceleration/deceleration |
| **Ordinate 3** ||||
| 3023 | ord3_name | char | Axis name (ASCII) |
| 3024 | ord3_type | int | Axis type (0=DISCRETE, 1=COORDINATE) |
| 3025 | ord3_given | int | Target position (pulses) |
| 3026 | ord3_speed_given | int | Speed (pulses/sec) |
| 3027 | ord3_pulse_turn | int | Pulses per revolution |
| 3028 | ord3_acl_dcl_point | int | Acceleration/deceleration |

#### Coil Registers (Discrete Outputs) - FC1/FC5

| Address | Name | Description |
|---------|------|-------------|
| 1999 | soft_reset | Activate SoftReset |
| 2000 | save_params | Save parameters to flash memory |
| **Ordinate 1** |||
| 2004 | null_ord1 | Zero axis 1 coordinate |
| 2005 | direction_ord1 | Axis 1 direction (0=toward MIN, 1=toward MAX) |
| **Ordinate 2** |||
| 2009 | null_ord2 | Zero axis 2 coordinate |
| 2010 | direction_ord2 | Axis 2 direction (0=toward MIN, 1=toward MAX) |
| **Ordinate 3** |||
| 2014 | null_ord3 | Zero axis 3 coordinate |
| 2015 | direction_ord3 | Axis 3 direction (0=toward MIN, 1=toward MAX) |
| **Discrete Outputs** |||
| 2019 | discrete_output1 | Discrete output 1 |
| 2020 | discrete_output2 | Discrete output 2 |
| 2021 | discrete_output3 | Discrete output 3 |

### ModbusDriver Coil Write Method (Optional - Fallback for Discrete Axes)

**Primary approach:** Write 0 or 1 directly to target position register. This is simpler and observed in `box_capture_activation()` and `throw_box_into_container()` in the Python driver.

**Fallback approach:** If writing 0/1 to target register doesn't trigger motion reliably, implement coil-based control:

```python
def write_coil(self, slave_id: int, coil_address: int, value: bool) -> bool:
    """
    Write single coil (FC5). Fallback for direction control on discrete axes.

    Primary method: Write 0/1 to target register (simpler)
    Fallback method: Use direction coil + null_ord coil

    Args:
        slave_id: Modbus slave address (1, 2, or 3)
        coil_address: Coil register address (e.g., 2010 for ord2 direction)
        value: True = ON (1), False = OFF (0)

    Returns:
        True if write successful
    """
    with self._lock:
        try:
            instrument = self._get_instrument(slave_id)
            instrument.write_bit(
                coil_address,
                value,
                functioncode=5  # FC5: Write Single Coil
            )
            return True
        except Exception as e:
            self._log_error(f'Coil write failed: slave={slave_id} addr={coil_address}: {e}')
            return False
```

**Coil addresses for fallback method:**

| Axis | Direction Coil | Null (trigger) Coil |
|------|----------------|---------------------|
| ord1 | 2005 | 2004 |
| ord2 (C axis) | 2010 | 2009 |
| ord3 (D axis) | 2015 | 2014 |

---

## URDF ros2_control Integration

### ros2_control.xacro Modification

Modify `ros2_ws/src/manipulator_description/urdf/manipulator/ros2_control.xacro`:

```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">

  <xacro:macro name="manipulator_ros2_control" params="
    name
    sim:=true
    initial_positions:=${dict(
      base_main_frame_joint=0.1,
      main_frame_selector_frame_joint=0.05,
      selector_frame_gripper_joint=0.0,
      selector_frame_picker_frame_joint=0.005,
      picker_frame_picker_rail_joint=0.0,
      picker_rail_picker_base_joint=0.005,
      picker_base_picker_jaw_joint=0.005,
      selector_left_container_jaw_joint=0.0,
      selector_right_container_jaw_joint=0.0
    )}">

    <ros2_control name="${name}" type="system">

      <!-- Hardware Plugin Selection -->
      <xacro:if value="${sim}">
        <!-- SIMULATION: Gazebo ros2_control plugin -->
        <hardware>
          <plugin>gz_ros2_control/GazeboSimSystem</plugin>
        </hardware>
      </xacro:if>

      <xacro:unless value="${sim}">
        <!-- REAL HARDWARE: Modbus RTU interface -->
        <hardware>
          <plugin>manipulator_hardware/ModbusHardwareInterface</plugin>
          <param name="config_file">$(find manipulator_hardware)/config/hardware_config.yaml</param>
        </hardware>
      </xacro:unless>

      <!-- Joint Interfaces (same for both sim and hardware) -->

      <!-- Motion Joints (7) - controlled via Modbus -->
      <xacro:macro name="motion_joint_interface" params="name initial_position">
        <joint name="${name}">
          <command_interface name="position">
            <param name="min">${xacro.load_yaml('$(find manipulator_description)/config/manipulator_params.yaml')['joint_limits'][name]['min']}</param>
            <param name="max">${xacro.load_yaml('$(find manipulator_description)/config/manipulator_params.yaml')['joint_limits'][name]['max']}</param>
          </command_interface>
          <state_interface name="position">
            <param name="initial_value">${initial_position}</param>
          </state_interface>
          <state_interface name="velocity"/>
        </joint>
      </xacro:macro>

      <xacro:motion_joint_interface name="base_main_frame_joint"
        initial_position="${initial_positions['base_main_frame_joint']}"/>
      <xacro:motion_joint_interface name="main_frame_selector_frame_joint"
        initial_position="${initial_positions['main_frame_selector_frame_joint']}"/>
      <xacro:motion_joint_interface name="selector_frame_gripper_joint"
        initial_position="${initial_positions['selector_frame_gripper_joint']}"/>
      <xacro:motion_joint_interface name="selector_frame_picker_frame_joint"
        initial_position="${initial_positions['selector_frame_picker_frame_joint']}"/>
      <xacro:motion_joint_interface name="picker_frame_picker_rail_joint"
        initial_position="${initial_positions['picker_frame_picker_rail_joint']}"/>
      <xacro:motion_joint_interface name="picker_rail_picker_base_joint"
        initial_position="${initial_positions['picker_rail_picker_base_joint']}"/>
      <xacro:motion_joint_interface name="picker_base_picker_jaw_joint"
        initial_position="${initial_positions['picker_base_picker_jaw_joint']}"/>

      <!-- Container Jaw Joints (2) - mock interface -->
      <joint name="selector_left_container_jaw_joint">
        <command_interface name="position"/>
        <state_interface name="position">
          <param name="initial_value">${initial_positions['selector_left_container_jaw_joint']}</param>
        </state_interface>
        <state_interface name="velocity"/>
      </joint>

      <joint name="selector_right_container_jaw_joint">
        <command_interface name="position"/>
        <state_interface name="position">
          <param name="initial_value">${initial_positions['selector_right_container_jaw_joint']}</param>
        </state_interface>
        <state_interface name="velocity"/>
      </joint>

    </ros2_control>

  </xacro:macro>

</robot>
```

### Launch File Hardware Mode

Modify launch to support hardware mode:

```python
# In manipulator_simulation.launch.py

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Declare use_sim_time argument
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time if true, hardware if false'
    )

    use_sim_time = LaunchConfiguration('use_sim_time')

    # Robot description with sim parameter
    robot_description_content = Command([
        'xacro ',
        PathJoinSubstitution([
            FindPackageShare('manipulator_description'),
            'urdf', 'robot.urdf.xacro'
        ]),
        ' sim:=', use_sim_time  # Pass sim parameter to xacro
    ])

    # ... rest of launch file
```

**Usage:**

```bash
# Simulation mode (default)
ros2 launch manipulator_control manipulator_simulation.launch.py

# Real hardware mode
ros2 launch manipulator_control manipulator_simulation.launch.py use_sim_time:=false
```

---

## Configuration Architecture

### hardware_config.yaml

```yaml
# manipulator_hardware/config/hardware_config.yaml
#
# Hardware configuration for Modbus RTU interface
# Maps ROS2 joints to Modbus slave/register addresses

# Modbus Connection
modbus:
  port: "/dev/ttyACM0"
  baudrate: 115200
  timeout_ms: 500
  retry_count: 3

# Status Registers (same for all slaves)
status_registers:
  error_code: 999
  module_ready: 1001    # 1 = ready for commands
  module_busy: 1002     # 1 = busy executing

# Joint-to-Hardware Mapping
joints:
  # =========================================================================
  # Slave 1: engineXZ (X and Z axes)
  # =========================================================================
  base_main_frame_joint:  # X axis
    slave_id: 1
    ordinate: 1
    position_register: 1003   # inp_reg.ord1_current
    command_register: 3005    # hold_reg.ord1_given
    pulses_per_meter: 100000  # 0.01mm resolution
    direction: 1              # 1 or -1 for axis inversion

  main_frame_selector_frame_joint:  # Z axis
    slave_id: 1
    ordinate: 2
    position_register: 1010   # inp_reg.ord2_current
    command_register: 3015    # hold_reg.ord2_given
    pulses_per_meter: 100000
    direction: 1

  # =========================================================================
  # Slave 2: engineYB (Y and B axes)
  # =========================================================================
  selector_frame_gripper_joint:  # Y axis
    slave_id: 2
    ordinate: 1
    position_register: 1003
    command_register: 3005
    pulses_per_meter: 100000
    direction: 1

  picker_frame_picker_rail_joint:  # B axis
    slave_id: 2
    ordinate: 2
    position_register: 1010
    command_register: 3015
    pulses_per_meter: 100000
    direction: 1

  # =========================================================================
  # Slave 3: engineACD (A, C, D axes)
  # =========================================================================
  selector_frame_picker_frame_joint:  # A axis
    slave_id: 3
    ordinate: 1
    position_register: 1003
    command_register: 3005
    pulses_per_meter: 100000
    direction: 1

  picker_rail_picker_base_joint:  # C axis
    slave_id: 3
    ordinate: 2
    position_register: 1010
    command_register: 3015
    pulses_per_meter: 100000
    direction: 1

  picker_base_picker_jaw_joint:  # D axis
    slave_id: 3
    ordinate: 3
    position_register: 1017
    command_register: 3025
    pulses_per_meter: 100000
    direction: 1

# Mock joints (no hardware mapping)
# Container jaws - will be discrete I/O in future
mock_joints:
  - selector_left_container_jaw_joint
  - selector_right_container_jaw_joint
```

---

## Unit Conversion & Calibration

### Conversion Formulas

```
Position (meters) → Pulses (for write):
  pulses = position_meters × pulses_per_meter × direction

Pulses → Position (meters) (for read):
  position_meters = pulses ÷ pulses_per_meter × direction
```

### Precision Analysis

| Parameter | Value |
|-----------|-------|
| `pulses_per_meter` | 100,000 |
| 1 meter | 100,000 pulses |
| 1 mm | 100 pulses |
| 0.01 mm | 1 pulse (minimum resolution) |
| Max range (16-bit unsigned) | 65,535 pulses = 0.65535 m |

**Note:** For joints with >0.65m range, use 32-bit registers (two consecutive 16-bit registers).

### Direction Calibration

The `direction` parameter handles axis inversion:
- `direction: 1` - Positive ROS direction = positive pulse direction
- `direction: -1` - Positive ROS direction = negative pulse direction

Calibrate by:
1. Command small positive motion in ROS
2. Observe physical direction
3. Set `direction: -1` if motion is reversed

---

## Error Handling & Recovery

### Error Types and Handling

| Error | Detection | Handling |
|-------|-----------|----------|
| Serial port unavailable | `connect()` returns False | `on_configure()` returns ERROR, prevents activation |
| Modbus timeout | `read_*()` returns None | Log warning, return last known position, retry next cycle |
| Device not ready | `is_device_ready()` = False | Log warning, skip write, retry next cycle |
| Device error code | `read_error_code()` > 0 | Log error, optionally trigger `on_error()` |
| Position out of range | Comparison with limits | Clamp (already enforced by controller) |

### Recovery Strategies

```python
# In read() method - graceful degradation
def read(self, time, period) -> return_type:
    errors = 0
    for joint_name, jcfg in self.joint_configs.items():
        pulses = self.modbus_driver.read_input_register(
            jcfg['slave_id'],
            jcfg['position_register']
        )

        if pulses is not None:
            self.hw_positions[joint_name] = self._pulses_to_meters(pulses, jcfg)
        else:
            errors += 1
            # Keep last known position (don't update)
            self.logger.warn(f'read: Timeout on {joint_name}, using last value')

    # Only return ERROR if ALL joints failed
    if errors == len(self.joint_configs):
        return return_type.ERROR

    return return_type.OK
```

### Watchdog Timer (optional enhancement)

```python
# Add to ModbusHardwareInterface for production
def _check_communication_health(self):
    """Check if communication is healthy, trigger error if not."""
    consecutive_failures = getattr(self, '_consecutive_failures', 0)

    if consecutive_failures > 10:  # 10 cycles = 1s at 10Hz
        self.logger.error('Communication failure threshold exceeded')
        # Trigger lifecycle error transition
        return False

    return True
```

---

## Testing Strategy

### Unit Tests

```python
# test/test_modbus_driver.py

import pytest
from unittest.mock import Mock, patch
from manipulator_hardware.modbus_driver import ModbusDriver


class TestModbusDriver:

    @patch('manipulator_hardware.modbus_driver.minimalmodbus.Instrument')
    def test_connect_success(self, mock_instrument):
        """Test successful serial connection."""
        driver = ModbusDriver()
        result = driver.connect('/dev/ttyACM0', 115200)
        assert result is True

    @patch('manipulator_hardware.modbus_driver.minimalmodbus.Instrument')
    def test_read_input_register(self, mock_instrument):
        """Test reading input register."""
        mock_inst = Mock()
        mock_inst.read_register.return_value = 12345
        mock_instrument.return_value = mock_inst

        driver = ModbusDriver()
        driver.connect('/dev/ttyACM0', 115200)

        result = driver.read_input_register(1, 1003)

        assert result == 12345
        mock_inst.read_register.assert_called_once_with(
            1003, number_of_decimals=0, functioncode=4, signed=False
        )

    @patch('manipulator_hardware.modbus_driver.minimalmodbus.Instrument')
    def test_read_timeout_returns_none(self, mock_instrument):
        """Test timeout returns None instead of raising."""
        mock_inst = Mock()
        mock_inst.read_register.side_effect = Exception('Timeout')
        mock_instrument.return_value = mock_inst

        driver = ModbusDriver()
        driver.connect('/dev/ttyACM0', 115200)

        result = driver.read_input_register(1, 1003)

        assert result is None


class TestUnitConversion:
    """Test unit conversion accuracy."""

    def test_pulses_to_meters(self):
        """Test pulse to meter conversion."""
        jcfg = {'pulses_per_meter': 100000, 'direction': 1}

        # 100,000 pulses = 1 meter
        assert abs(_pulses_to_meters(100000, jcfg) - 1.0) < 1e-9

        # 1 pulse = 0.00001 meters (0.01 mm)
        assert abs(_pulses_to_meters(1, jcfg) - 0.00001) < 1e-9

    def test_meters_to_pulses(self):
        """Test meter to pulse conversion."""
        jcfg = {'pulses_per_meter': 100000, 'direction': 1}

        assert _meters_to_pulses(1.0, jcfg) == 100000
        assert _meters_to_pulses(0.00001, jcfg) == 1

    def test_direction_inversion(self):
        """Test direction parameter."""
        jcfg_pos = {'pulses_per_meter': 100000, 'direction': 1}
        jcfg_neg = {'pulses_per_meter': 100000, 'direction': -1}

        assert _pulses_to_meters(1000, jcfg_pos) == 0.01
        assert _pulses_to_meters(1000, jcfg_neg) == -0.01
```

### Integration Tests

```python
# test/test_hardware_interface_integration.py

import pytest
import launch_testing
from launch import LaunchDescription
from launch_ros.actions import Node


@pytest.mark.launch_test
def generate_test_description():
    """Launch controller_manager with mock hardware."""
    return LaunchDescription([
        Node(
            package='controller_manager',
            executable='ros2_control_node',
            parameters=[
                {'robot_description': get_robot_description(sim=False)},
                {'controller_manager.update_rate': 100}
            ],
        ),
        launch_testing.actions.ReadyToTest()
    ])


class TestHardwareInterfaceIntegration:

    def test_plugin_loads(self, proc_output):
        """Verify plugin loads without errors."""
        proc_output.assertWaitFor(
            'ModbusHardwareInterface',
            timeout=10.0
        )

    def test_state_interfaces_available(self, proc_info):
        """Verify state interfaces are exported."""
        # Check /controller_manager/list_hardware_interfaces
        pass

    def test_joint_states_publishing(self, proc_info):
        """Verify /joint_states topic publishes."""
        pass
```

### Hardware Validation Tests

```bash
#!/bin/bash
# test_hardware.sh - Run on real hardware

echo "=== Hardware Interface Validation ==="

# 1. Check serial port
echo "1. Checking serial port..."
ls -la /dev/ttyACM0 || { echo "FAIL: Serial port not found"; exit 1; }

# 2. Launch hardware mode
echo "2. Launching hardware mode..."
ros2 launch manipulator_control manipulator_simulation.launch.py use_sim_time:=false &
LAUNCH_PID=$!
sleep 10

# 3. Check controllers
echo "3. Checking controllers..."
ros2 control list_controllers

# 4. Check joint states
echo "4. Reading joint states..."
ros2 topic echo /joint_states --once

# 5. Test single joint motion
echo "5. Testing single joint..."
ros2 action send_goal /base_main_frame_joint_controller/follow_joint_trajectory \
  control_msgs/action/FollowJointTrajectory \
  "{trajectory: {joint_names: ['base_main_frame_joint'], points: [{positions: [0.5], time_from_start: {sec: 2}}]}}"

# Cleanup
kill $LAUNCH_PID
echo "=== Test Complete ==="
```

---

## Future: C++ Production Implementation

After proof-of-concept validation, port to C++ for:
- **Realtime performance:** Deterministic timing guarantees
- **Lower latency:** No Python GIL, native serial I/O
- **Production stability:** Compile-time type checking

### C++ Implementation Skeleton

```cpp
// include/manipulator_hardware/modbus_hardware_interface.hpp

#ifndef MANIPULATOR_HARDWARE__MODBUS_HARDWARE_INTERFACE_HPP_
#define MANIPULATOR_HARDWARE__MODBUS_HARDWARE_INTERFACE_HPP_

#include <memory>
#include <string>
#include <vector>
#include <unordered_map>

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/state.hpp"

namespace manipulator_hardware
{

class ModbusHardwareInterface : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(ModbusHardwareInterface)

  // Lifecycle callbacks
  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_cleanup(
    const rclcpp_lifecycle::State & previous_state) override;

  // Control loop
  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  // Modbus driver (use libmodbus)
  std::unique_ptr<ModbusDriver> modbus_driver_;

  // Joint configuration
  struct JointConfig {
    uint8_t slave_id;
    uint16_t position_register;
    uint16_t command_register;
    double pulses_per_meter;
    int direction;
  };
  std::unordered_map<std::string, JointConfig> joint_configs_;

  // State storage
  std::vector<double> hw_positions_;
  std::vector<double> hw_velocities_;
  std::vector<double> hw_commands_;
};

}  // namespace manipulator_hardware

#endif  // MANIPULATOR_HARDWARE__MODBUS_HARDWARE_INTERFACE_HPP_
```

### C++ Dependencies

```cmake
# CMakeLists.txt additions for C++
find_package(libmodbus REQUIRED)

add_library(manipulator_hardware SHARED
  src/modbus_hardware_interface.cpp
  src/modbus_driver.cpp
)

target_link_libraries(manipulator_hardware
  modbus  # libmodbus
)

ament_target_dependencies(manipulator_hardware
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)
```

---

## References

### ROS2 Control Documentation

- [Writing a Hardware Component (Jazzy)](https://control.ros.org/jazzy/doc/ros2_control/hardware_interface/doc/writing_new_hardware_component.html)
- [Hardware Interface Types](https://control.ros.org/jazzy/doc/ros2_control/hardware_interface/doc/hardware_interface_types_userdoc.html)
- [Example 7: 6DOF Robot Tutorial](https://control.ros.org/jazzy/doc/ros2_control_demos/example_7/doc/userdoc.html)
- [ros2_control Getting Started](https://control.ros.org/jazzy/doc/getting_started/getting_started.html)

### Project Files

| Reference | Path |
|-----------|------|
| Working Modbus driver | `examples/modbus_driver/modbus_rtu.py` |
| High-level motor control | `examples/modbus_driver/control.py` |
| Register configuration | `examples/modbus_driver/configuration.yml` |
| URDF ros2_control | `ros2_ws/src/manipulator_description/urdf/manipulator/ros2_control.xacro` |
| Controller config | `ros2_ws/src/manipulator_description/config/manipulator_controllers.yaml` |
| Joint limits | `ros2_ws/src/manipulator_description/config/manipulator_params.yaml` |
| Tech spec | `docs/tech-spec-hardware-interface.md` |

### Hardware Documentation

| Component | Slave ID | Axes |
|-----------|----------|------|
| engineXZ | 1 | X (ord1), Z (ord2) |
| engineYB | 2 | Y (ord1), B (ord2) |
| engineACD | 3 | A (ord1), C (ord2), D (ord3) |

### ROS2 Jazzy System Headers

| Header | Location |
|--------|----------|
| SystemInterface | `/opt/ros/jazzy/include/hardware_interface/hardware_interface/system_interface.hpp` |
| HardwareComponentInterface | `/opt/ros/jazzy/include/hardware_interface/hardware_interface/hardware_component_interface.hpp` |

---

## Summary

This architecture provides:

1. **Python-first implementation** matching existing `examples/modbus_driver/` patterns
2. **Clean separation** between hardware interface (lifecycle, state/command) and communication (ModbusDriver)
3. **Hot-swappable** simulation/hardware via URDF `sim` parameter
4. **Configuration-driven** joint mapping (no code changes for calibration)
5. **Graceful error handling** with retry logic and last-known-value fallback
6. **Clear upgrade path** to C++ for production

**Next Steps:**
1. Create `manipulator_hardware` package
2. Implement `ModbusDriver` (adapt from `examples/modbus_driver/modbus_rtu.py`)
3. Implement `ModbusHardwareInterface`
4. Test with mock serial, then real hardware
5. Update URDF and launch files
