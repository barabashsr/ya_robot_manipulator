# Story 7.2: Implement ModbusHardwareInterface Plugin

Status: Done

## Story

As a developer,
I want to implement the ros2_control SystemInterface plugin,
so that JointTrajectoryControllers can command real hardware.

## Core Concepts

### ros2_control Hardware Interface Fundamentals

**What is a Hardware Interface?**

In ros2_control, hardware components are **plugins** dynamically loaded by the `controller_manager`. They abstract the physical hardware behind a standardized API of:
- **State Interfaces:** Read-only sensor data (position, velocity) - updated in `read()`
- **Command Interfaces:** Writable control outputs (position commands) - consumed in `write()`

The same controllers (JointTrajectoryController, ForwardCommandController) work identically whether the hardware is simulated (Gazebo) or real (Modbus).

### SystemInterface Base Class

```python
from hardware_interface import SystemInterface
from hardware_interface.types import HardwareInfo, CallbackReturn, return_type

class ModbusHardwareInterface(SystemInterface):
    # Lifecycle callbacks (called by controller_manager)
    def on_init(self, hardware_info: HardwareInfo) -> CallbackReturn
    def on_configure(self, previous_state) -> CallbackReturn
    def on_activate(self, previous_state) -> CallbackReturn
    def on_deactivate(self, previous_state) -> CallbackReturn
    def on_cleanup(self, previous_state) -> CallbackReturn
    def on_error(self, previous_state) -> CallbackReturn

    # Interface exports (called after on_init)
    def export_state_interfaces(self) -> list
    def export_command_interfaces(self) -> list

    # Control loop (called at update rate - 10 Hz for hardware)
    def read(self, time, period) -> return_type
    def write(self, time, period) -> return_type
```

### Lifecycle State Machine

```
                    ┌─────────────────┐
                    │   UNCONFIGURED  │ ← on_init()
                    │  (initialized)  │   Parse URDF, load config
                    └────────┬────────┘
                             │ on_configure()
                             ▼
                    ┌─────────────────┐
                    │    INACTIVE     │ ← Connect to Modbus
                    │ (states readable│   Read initial positions
                    │  commands NOT)  │   Commands NOT available
                    └────────┬────────┘
                             │ on_activate()
                             ▼
                    ┌─────────────────┐
                    │     ACTIVE      │ ← Full operation
                    │ (read + write)  │   read()/write() called at 10Hz
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

### Unit Conversion: Meters ↔ Pulses

The hardware interface is the **boundary** between ROS2 units (SI) and hardware units (pulses):

```python
# ROS2 → Hardware (in write())
pulses = int(position_meters * pulses_per_meter * direction)

# Hardware → ROS2 (in read())
position_meters = pulses / pulses_per_meter * direction
```

**Precision:**
- 1 meter = 100,000 pulses (configurable per joint)
- 1 mm = 100 pulses
- 0.01 mm = 1 pulse (minimum resolution)

**Direction:**
- `direction = 1` → positive pulse = positive meter movement
- `direction = -1` → positive pulse = negative meter movement (axis inversion)

### Update Rate Decision

| Mode | Update Rate | Rationale |
|------|-------------|-----------|
| Simulation | 100 Hz | No I/O latency, Gazebo physics timing |
| **Hardware** | **10 Hz** | Modbus RTU timing: 7 reads × ~5ms = 35ms fits in 100ms budget |

**Timing Budget (10 Hz = 100ms cycle):**
- 7 position reads (FC4): 7 × 5ms = 35ms
- 7 command writes (FC6): 7 × 5ms = 35ms (worst case)
- Overhead: 30ms
- **Total: 100ms budget satisfied**

### Joint Classification

| Joint Type | Count | Communication | State Interface | Command Interface |
|------------|-------|---------------|-----------------|-------------------|
| **Continuous joints** | 5 | Modbus RTU | Accurate position | Write to hardware |
| **Discrete joints** | 2 | Modbus RTU | Inaccurate during motion | Write to hardware |
| **Mock joints** | 2 | None | Echo command | Accept command |

**Continuous joints** (X, Z, Y, A, B): Servo-like motion to exact position with accurate feedback.

**Discrete joints** (C, D): Move until limit switch activates, inaccurate position during motion.

**Mock joints** (container jaws): Return commanded position as state - placeholder for future discrete I/O.

### The Three Axis Types

This story handles **three distinct axis types** with different control approaches:

#### 1. CONTINUOUS AXES (X, Z, Y, A, B) — 5 joints

| Joint | Slave | Axis |
|-------|-------|------|
| `base_main_frame_joint` | 1 | X |
| `main_frame_selector_frame_joint` | 1 | Z |
| `selector_frame_gripper_joint` | 2 | Y |
| `selector_frame_picker_frame_joint` | 3 | A |
| `picker_frame_picker_rail_joint` | 2 | B |

**Behavior:** Servo-like positioning — motor moves to exact commanded position with accurate encoder feedback.

**CRITICAL - Incremental Commands Only:**
The STM32 firmware **rejects new position commands if the previous goal hasn't been achieved**. We cannot send distant targets — we must use incremental commands only.

```python
# In write() for continuous axes:
max_delta = joint_cfg['max_velocity'] * period_sec
delta = target_pos - current_pos
if abs(delta) > max_delta:
    delta = max_delta if delta > 0 else -max_delta
incremental_target = current_pos + delta
# Send incremental_target (firmware will accept it)
```

#### 2. DISCRETE AXES (C, D) — 2 joints

| Joint | Slave | Axis | MIN Limit Reg | MAX Limit Reg |
|-------|-------|------|---------------|---------------|
| `picker_rail_picker_base_joint` | 3 | C (ord2) | 1013 | 1014 |
| `picker_base_picker_jaw_joint` | 3 | D (ord3) | 1020 | 1021 |

**Behavior:** Binary ON/OFF control — motor runs until limit switch activates (NOT to exact position).

**Command Mapping:**
- `command >= 0.5` → Write `1` → Move toward MAX limit switch
- `command < 0.5` → Write `0` → Move toward MIN limit switch

**Position Feedback:** Position register IS readable during motion (provides interpolated values), but final position is determined by which limit switch activates.

**Constraints:**
- Must check `module_is_busy` (register 1002) before sending new commands
- Slave 3 allows only ONE axis motion at a time (A, C, D are sequential)

#### 3. MOCK JOINTS (container jaws) — 2 joints

| Joint | Hardware |
|-------|----------|
| `selector_left_container_jaw_joint` | None (mock) |
| `selector_right_container_jaw_joint` | None (mock) |

**Behavior:** State = Command (echo). No Modbus communication. Placeholder for future discrete I/O.

### Limit Switch Interface

Hardware interface reads limit switch states via FC4 (input registers):

| Slave | Axis | Joint | MIN Reg | MAX Reg |
|-------|------|-------|---------|---------|
| 1 | X (ord1) | base_main_frame_joint | 1006 | 1007 |
| 1 | Z (ord2) | main_frame_selector_frame_joint | 1013 | 1014 |
| 2 | Y (ord1) | selector_frame_gripper_joint | 1006 | 1007 |
| 2 | B (ord2) | picker_frame_picker_rail_joint | 1013 | 1014 |
| 3 | A (ord1) | selector_frame_picker_frame_joint | 1006 | 1007 |
| 3 | C (ord2) | picker_rail_picker_base_joint | 1013 | 1014 |
| 3 | D (ord3) | picker_base_picker_jaw_joint | 1020 | 1021 |

**Values:** `0` = not at limit, `1` = at limit

**Usage for discrete axes:**
- Before writing command, check if already at target limit (skip redundant write)
- Final position determined by limit switch state:
  - `min_limit=1` → position = `min_position` (e.g., 0.0m)
  - `max_limit=1` → position = `max_position` (e.g., 0.29m for C axis)

### Slave-Level Operation Constraints

| Slave | Device | Axes | Simultaneous Motion |
|-------|--------|------|---------------------|
| 1 | engineXZ | X, Z | **YES** - can run together |
| 2 | engineYB | Y, B | **YES** - can run together |
| 3 | engineACD | A, C, D | **NO** - one at a time only |

### Handling Strategy Summary

| Scenario | Action |
|----------|--------|
| `write()` continuous axis | Send **incremental** command (max_velocity × period) |
| `write()` discrete axis | Threshold to 0/1, check busy, check limit, send |
| `write()` to C/D while slave busy | **Defer** - skip this cycle, retry next |
| `read()` from any axis | Read position register (works during motion) |
| Multiple commands to slave 3 | **Sequential** - process one axis per cycle |

### State Interface: Limit Switches (Optional Enhancement)

Hardware interface should expose limit switch states via state interfaces:
- `<joint>/limit_switch_min` (bool)
- `<joint>/limit_switch_max` (bool)

This enables:
- Parity with simulation (virtual limit switches already implemented)
- Safety monitoring
- Homing procedures

### Hardware Mapping (Updated Registers)

| ROS2 Joint | Slave | Ord | Pos Reg | Cmd Reg | pulses/m | dir |
|------------|-------|-----|---------|---------|----------|-----|
| `base_main_frame_joint` | 1 | 1 | 1003 | 3005 | 100000 | 1 |
| `main_frame_selector_frame_joint` | 1 | 2 | 1010 | 3015 | 100000 | 1 |
| `selector_frame_gripper_joint` | 2 | 1 | 1003 | 3005 | 100000 | 1 |
| `selector_frame_picker_frame_joint` | 3 | 1 | 1003 | 3005 | 100000 | 1 |
| `picker_frame_picker_rail_joint` | 2 | 2 | 1010 | 3015 | 100000 | 1 |
| `picker_rail_picker_base_joint` | 3 | 2 | 1010 | 3015 | 100000 | 1 |
| `picker_base_picker_jaw_joint` | 3 | 3 | 1017 | 3025 | 100000 | 1 |
| `selector_left_container_jaw_joint` | - | - | Mock | Mock | - | - |
| `selector_right_container_jaw_joint` | - | - | Mock | Mock | - | - |

## Acceptance Criteria

1. **Given** ModbusDriver is implemented (Story 7.1)
   **When** I implement ModbusHardwareInterface
   **Then** the plugin implements `hardware_interface.SystemInterface` with all lifecycle callbacks

2. **Given** the plugin is implemented
   **When** `export_state_interfaces()` is called
   **Then** it exports:
   - 9 position state interfaces (7 Modbus + 2 mock)
   - 9 velocity state interfaces (7 Modbus + 2 mock)

3. **Given** the plugin is implemented
   **When** `export_command_interfaces()` is called
   **Then** it exports:
   - 9 position command interfaces (7 Modbus + 2 mock)

4. **Given** the plugin is active
   **When** `read()` is called
   **Then** it:
   - Reads position from all 7 Modbus joints via ModbusDriver
   - Converts pulses → meters using `pulses_per_meter` and `direction`
   - Estimates velocity from position delta
   - Sets mock joint positions to their command values

5. **Given** the plugin is active
   **When** `write()` is called
   **Then** it:
   - Detects changed commands (skip unchanged to reduce bus traffic)
   - Converts meters → pulses using `pulses_per_meter` and `direction`
   - Writes position commands to hardware via ModbusDriver
   - Skips write for mock joints

6. **Given** configuration file `hardware_config.yaml`
   **When** `on_init()` parses configuration
   **Then** it loads:
   - Modbus port, baudrate, timeout
   - Per-joint: slave_id, position_register, command_register, pulses_per_meter, direction

7. **Given** the plugin
   **When** registered via pluginlib
   **Then** `manipulator_hardware.xml` exports:
   ```xml
   <class name="manipulator_hardware/ModbusHardwareInterface"
          type="manipulator_hardware.modbus_hardware_interface.ModbusHardwareInterface"
          base_class_type="hardware_interface::SystemInterface">
   ```

8. **Given** unit tests
   **When** run with mock ModbusDriver
   **Then** tests verify:
   - Lifecycle state transitions
   - Interface export counts
   - Unit conversion accuracy (meters ↔ pulses)
   - Read/write cycle with mocked driver

9. **Given** discrete axis (C or D) command
   **When** Slave 3 is busy (`module_is_busy=1`)
   **Then** `write()` defers the command (skip this cycle, retry next)
   **And** no error is raised to controller

10. **Given** discrete axis (C or D) command
    **When** command value is received from controller
    **Then** `write()` thresholds it to binary:
    - `command >= 0.5` → Write `1` (toward MAX limit)
    - `command < 0.5` → Write `0` (toward MIN limit)
    **And** checks limit switch state to skip if already at target

11. **Given** multiple commands to Slave 3 axes (A, C, D)
    **When** `write()` is called
    **Then** only ONE axis command is sent per cycle
    **And** remaining commands are queued for next cycles

12. **Given** continuous axis (X, Z, Y, A, B) command
    **When** `write()` is called
    **Then** it sends **incremental** position commands only:
    - Calculate `max_delta = max_velocity × period`
    - Clamp `(target - current)` to `±max_delta`
    - Send `current + clamped_delta` to hardware
    **And** firmware accepts the achievable incremental target

13. **MANDATORY: Given** a standalone test script
    **When** developer runs self-test WITHOUT controller_manager
    **Then** developer MUST verify:
    - Plugin instantiation works
    - Configuration loads correctly
    - ModbusDriver connects (with available device)
    - `read()` returns valid positions in meters
    - `write()` sends correct pulses to hardware
    - Unit conversion is accurate (send 0.5m, verify ~50000 pulses)
    - **Discrete axis handling:** Command while busy is deferred gracefully
    - **Incremental commands:** Continuous axis respects max_velocity limit
    - **Document results in completion notes**

## Tasks / Subtasks

- [x] Task 1: Create ModbusHardwareInterface class structure (AC: #1)
  - [x] 1.1 Create `manipulator_hardware/modbus_hardware_interface.py`
  - [x] 1.2 Import SystemInterface from hardware_interface
  - [x] 1.3 Define class with `__init__` initializing state/command storage
  - [x] 1.4 Define mock_joints list for container jaws

- [x] Task 2: Implement lifecycle callbacks (AC: #1, #6)
  - [x] 2.1 Implement `on_init()` - parse HardwareInfo, load YAML config
  - [x] 2.2 Implement `on_configure()` - connect ModbusDriver, verify devices
  - [x] 2.3 Implement `on_activate()` - sync commands to current positions
  - [x] 2.4 Implement `on_deactivate()` - hold position, log
  - [x] 2.5 Implement `on_cleanup()` - disconnect ModbusDriver
  - [x] 2.6 Implement `on_error()` - graceful shutdown

- [x] Task 3: Implement interface exports (AC: #2, #3)
  - [x] 3.1 Implement `export_state_interfaces()` - 9 position + 9 velocity
  - [x] 3.2 Implement `export_command_interfaces()` - 9 position

- [x] Task 4: Implement control loop (AC: #4, #5, #9, #10, #11, #12)
  - [x] 4.1 Implement `read()` - read positions from all axes, convert pulses→meters, estimate velocity
  - [x] 4.2 Implement `write()` for continuous axes - **incremental commands only**:
    - [x] 4.2.1 Calculate `max_delta = max_velocity × period`
    - [x] 4.2.2 Clamp delta to achievable increment
    - [x] 4.2.3 Send `current + clamped_delta` (not distant goals)
  - [x] 4.3 Implement `write()` for discrete axes (C, D):
    - [x] 4.3.1 Threshold command to binary (>=0.5 → 1, else → 0)
    - [x] 4.3.2 Check `module_is_busy` - defer if busy
    - [x] 4.3.3 Check limit switches - skip if already at target
    - [x] 4.3.4 Write 0 or 1 to target register
  - [x] 4.4 Handle mock joints (state = command, no hardware write)
  - [x] 4.5 Implement `_is_slave_busy(slave_id)` helper method
  - [x] 4.6 Implement `_read_limit_switch(slave_id, register)` helper method
  - [x] 4.7 Implement Slave 3 sequential command queue (one axis per cycle)

- [x] Task 5: Implement unit conversion helpers (AC: #4, #5)
  - [x] 5.1 Create `_pulses_to_meters(pulses, joint_config)` method
  - [x] 5.2 Create `_meters_to_pulses(meters, joint_config)` method
  - [x] 5.3 Add unit tests for conversion accuracy

- [x] Task 6: Update hardware_config.yaml (AC: #6)
  - [x] 6.1 Verify joint mapping matches updated tech-spec registers
  - [x] 6.2 Add `pulses_per_meter` and `direction` for each joint
  - [x] 6.3 Add `max_velocity` for continuous axes (incremental clamping)
  - [x] 6.4 Add `min_limit_register`, `max_limit_register` for all axes
  - [x] 6.5 Add `discrete_axis`, `min_position`, `max_position` for C/D axes
  - [x] 6.6 Ensure mock joints are NOT in config (handled by code)

- [x] Task 7: Register plugin via pluginlib (AC: #7)
  - [x] 7.1 Update `manipulator_hardware.xml` with correct Python class path
  - [x] 7.2 Verify package.xml has pluginlib export

- [x] Task 8: Write unit tests (AC: #8, #9, #10, #11, #12)
  - [x] 8.1 Create `test/test_modbus_hardware_interface.py`
  - [x] 8.2 Test lifecycle transitions with mock driver
  - [x] 8.3 Test interface export counts (9+9 state, 9 command)
  - [x] 8.4 Test unit conversion accuracy
  - [x] 8.5 Test read/write cycle with mock driver
  - [x] 8.6 Test discrete axis: binary threshold (0.5 boundary)
  - [x] 8.7 Test discrete axis: write deferred when busy
  - [x] 8.8 Test discrete axis: skip write when already at limit
  - [x] 8.9 Test Slave 3 sequential: only one command per cycle
  - [x] 8.10 Test continuous axis: incremental commands clamped to max_velocity

- [x] Task 9: **MANDATORY** Self-test script (AC: #13)
  - [x] 9.1 Create `scripts/test_hardware_interface.py` standalone script
  - [x] 9.2 Instantiate ModbusHardwareInterface directly
  - [x] 9.3 Call lifecycle methods manually (init, configure, activate)
  - [x] 9.4 Call read() and print positions in meters
  - [x] 9.5 Call write() with known values and verify pulses sent
  - [x] 9.6 Verify unit conversion (0.5m → ~50000 pulses)
  - [x] 9.7 Test incremental command clamping for continuous axis
  - [x] 9.8 Test discrete axis binary threshold and limit switch check
  - [x] 9.9 Document test results in completion notes

- [x] Task 10: Build and test verification
  - [x] 10.1 Run colcon build
  - [x] 10.2 Run colcon test (unit tests)
  - [ ] 10.3 Run self-test script with real hardware (pending hardware availability)

## Dev Notes

### Architecture Patterns

- **Composition:** ModbusHardwareInterface HAS-A ModbusDriver (not inheritance)
- **Dependency Injection:** ModbusDriver created in `on_configure()`, allows mocking
- **Optional returns:** Propagate None from ModbusDriver for error handling
- **Change detection:** Only write commands that changed (reduce bus traffic)

### Package Structure

```
ros2_ws/src/manipulator_hardware/
├── manipulator_hardware/
│   ├── __init__.py
│   ├── modbus_driver.py          # Story 7-1 (DONE)
│   └── modbus_hardware_interface.py  # This story
├── config/
│   └── hardware_config.yaml      # Update with full mapping
├── scripts/
│   └── test_hardware_interface.py    # Mandatory self-test
└── test/
    ├── test_modbus_driver.py     # Story 7-1 (DONE)
    └── test_modbus_hardware_interface.py  # This story
```

### Source Tree Components

| File | Action | Description |
|------|--------|-------------|
| `manipulator_hardware/modbus_hardware_interface.py` | CREATE | Main plugin class |
| `config/hardware_config.yaml` | UPDATE | Add all 7 joints with correct registers |
| `manipulator_hardware.xml` | UPDATE | Plugin registration |
| `scripts/test_hardware_interface.py` | CREATE | Mandatory self-test script |
| `test/test_modbus_hardware_interface.py` | CREATE | Unit tests |

### Testing Standards

- **Unit tests:** pytest with mocked ModbusDriver
- **Integration test:** Self-test script without controller_manager
- **Coverage target:** ≥80% for ModbusHardwareInterface

### Configuration Schema (hardware_config.yaml)

```yaml
modbus:
  port: "/dev/ttyUSB0"  # Verified in 7-1
  baudrate: 115200
  timeout_ms: 500
  retry_count: 3

joints:
  # =========================================================================
  # CONTINUOUS AXES - Servo positioning with incremental commands
  # =========================================================================
  base_main_frame_joint:  # X axis
    slave_id: 1
    ordinate: 1
    position_register: 1003
    command_register: 3005
    pulses_per_meter: 100000
    direction: 1
    max_velocity: 0.5             # m/s - for incremental command clamping
    min_limit_register: 1006
    max_limit_register: 1007

  main_frame_selector_frame_joint:  # Z axis
    slave_id: 1
    ordinate: 2
    position_register: 1010
    command_register: 3015
    pulses_per_meter: 100000
    direction: 1
    max_velocity: 0.5
    min_limit_register: 1013
    max_limit_register: 1014

  selector_frame_gripper_joint:  # Y axis
    slave_id: 2
    ordinate: 1
    position_register: 1003
    command_register: 3005
    pulses_per_meter: 100000
    direction: 1
    max_velocity: 0.5
    min_limit_register: 1006
    max_limit_register: 1007

  selector_frame_picker_frame_joint:  # A axis
    slave_id: 3
    ordinate: 1
    position_register: 1003
    command_register: 3005
    pulses_per_meter: 100000
    direction: 1
    max_velocity: 0.5
    min_limit_register: 1006
    max_limit_register: 1007

  picker_frame_picker_rail_joint:  # B axis
    slave_id: 2
    ordinate: 2
    position_register: 1010
    command_register: 3015
    pulses_per_meter: 100000
    direction: 1
    max_velocity: 0.5
    min_limit_register: 1013
    max_limit_register: 1014

  # =========================================================================
  # DISCRETE AXES - Binary 0/1 control, move to limit switch
  # =========================================================================
  picker_rail_picker_base_joint:  # C axis - DISCRETE
    slave_id: 3
    ordinate: 2
    position_register: 1010
    command_register: 3015
    pulses_per_meter: 100000
    direction: 1
    discrete_axis: true           # Binary 0/1 control mode
    min_limit_register: 1013
    max_limit_register: 1014
    min_position: 0.0             # Position when at MIN limit (meters)
    max_position: 0.29            # Position when at MAX limit (meters)

  picker_base_picker_jaw_joint:   # D axis - DISCRETE
    slave_id: 3
    ordinate: 3
    position_register: 1017
    command_register: 3025
    pulses_per_meter: 100000
    direction: 1
    discrete_axis: true           # Binary 0/1 control mode
    min_limit_register: 1020
    max_limit_register: 1021
    min_position: 0.0             # Position when at MIN limit (meters)
    max_position: 0.19            # Position when at MAX limit (meters)

# Note: mock joints (container jaws) are NOT in config
# They are handled specially in code:
#   - selector_left_container_jaw_joint
#   - selector_right_container_jaw_joint

status_registers:
  error_code: 999
  module_ready: 1001
  module_busy: 1002
```

### Self-Test Script Template

```python
#!/usr/bin/env python3
"""Standalone test for ModbusHardwareInterface without controller_manager."""

from manipulator_hardware.modbus_hardware_interface import ModbusHardwareInterface

def main():
    print("=== ModbusHardwareInterface Self-Test ===")

    # 1. Instantiate
    hw = ModbusHardwareInterface()
    print("✓ Instantiation OK")

    # 2. Load config (simulated HardwareInfo)
    # ... create mock HardwareInfo with config_file parameter

    # 3. Call lifecycle
    # hw.on_init(hardware_info)
    # hw.on_configure(None)
    # hw.on_activate(None)

    # 4. Test read()
    # hw.read(time, period)
    # print positions in meters

    # 5. Test write()
    # Set command to 0.5m
    # hw.write(time, period)
    # Verify ~50000 pulses sent

    # 6. Cleanup
    # hw.on_deactivate(None)
    # hw.on_cleanup(None)

if __name__ == '__main__':
    main()
```

### Detected Conflicts or Variances

- **Register addresses updated:** Tech-spec corrected command registers (3005/3015/3025 instead of 2999/3008/3017)
- **Status registers swapped:** module_ready=1001, module_busy=1002 (per updated tech-spec)

### References

- [Source: docs/architecture-epic7-hardware-interface.md#Python-Hardware-Interface-Implementation]
- [Source: docs/sprint-artifacts/tech-spec-epic-7.md#APIs-and-Interfaces]
- [Source: docs/sprint-artifacts/tech-spec-epic-7.md#Hardware-Mapping-Reference]
- [Source: docs/epics.md#Story-7.2]
- [Source: ros2_ws/src/manipulator_hardware/manipulator_hardware/modbus_driver.py] - ModbusDriver from Story 7-1

## Dev Agent Record

### Context Reference

- docs/sprint-artifacts/7-2-implement-modbushardwareinterface-plugin.context.xml

### Agent Model Used

Claude Opus 4.5

### Debug Log References

None

### Completion Notes List

**2025-11-28: Implementation Complete**

1. **ModbusHardwareInterface Plugin Created** (`modbus_hardware_interface.py`)
   - Full SystemInterface implementation with all lifecycle callbacks
   - 9 joints: 5 continuous, 2 discrete, 2 mock
   - Incremental command clamping for continuous axes (max_velocity * period)
   - Binary threshold (0.5) for discrete axes with busy/limit checks
   - Slave 3 sequential constraint enforced (one axis per cycle)

2. **Unit Tests** (`test/test_modbus_hardware_interface.py`)
   - 33 tests, all passing
   - Coverage: lifecycle, interface exports, unit conversion, discrete/continuous axes

3. **Self-Test Script** (`scripts/test_hardware_interface.py`)
   - Standalone validation without controller_manager
   - 9 tests, all passing with mock driver
   - Verifies: instantiation, config, lifecycle, conversion, read/write, cleanup

4. **Self-Test Results (Mock Mode)**:
   ```
   [PASS] Instantiation - 9 joints configured
   [PASS] Config Loading - 7 Modbus + 2 mock joints
   [PASS] Lifecycle (Mock) - on_activate syncs positions
   [PASS] Unit Conversion - 0.5m ↔ 50000 pulses, roundtrip accurate
   [PASS] Read Cycle - positions updated, mock joints echo command
   [PASS] Write Continuous - incremental clamp: 5000 pulses (0.05m/cycle @ 10Hz)
   [PASS] Write Discrete - threshold 0.49→0, 0.5→1; busy deferred
   [PASS] Interface Exports - 18 state (9 pos + 9 vel), 9 command
   [PASS] Cleanup - driver disconnected
   ```

5. **Pending**: Real hardware test (AC #13.3) - requires physical device connection

### File List

| File | Action |
|------|--------|
| `ros2_ws/src/manipulator_hardware/manipulator_hardware/modbus_hardware_interface.py` | CREATE |
| `ros2_ws/src/manipulator_hardware/config/hardware_config.yaml` | UPDATE |
| `ros2_ws/src/manipulator_hardware/manipulator_hardware.xml` | UPDATE |
| `ros2_ws/src/manipulator_hardware/scripts/test_hardware_interface.py` | CREATE |
| `ros2_ws/src/manipulator_hardware/test/test_modbus_hardware_interface.py` | CREATE |
| `ros2_ws/src/manipulator_hardware/setup.py` | UPDATE |
