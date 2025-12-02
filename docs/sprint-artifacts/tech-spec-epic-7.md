# Epic Technical Specification: Hardware Interface (Modbus RTU)

Date: 2025-11-27
Author: BMad
Epic ID: 7
Status: Draft

---

## Overview

Epic 7 implements a **ros2_control SystemInterface plugin** (`ModbusHardwareInterface`) that enables the ya_robot_manipulator to control real stepper motors via Modbus RTU serial communication. This epic bridges the gap between simulation (using `gz_ros2_control/GazeboSimSystem`) and real hardware deployment, allowing the same JointTrajectoryControllers and action servers developed in Epics 1-6 to command physical motors without code changes.

The hardware interface communicates with 3 Modbus RTU slave devices controlling 7 motion joints, converting between ROS2 position units (meters) and hardware units (pulses). The 2 container jaw joints use a mock interface as a placeholder for future discrete I/O control. This epic is **independent** of Epics 1-6 and can be developed in parallel, requiring only the base ros2_control framework.

## Objectives and Scope

**In Scope:**

- ros2_control `SystemInterface` plugin implementation in C++
- ModbusDriver class for serial communication with Modbus RTU protocol
- YAML configuration for joint-to-Modbus mapping (slave ID, registers, unit conversion)
- Unit conversion: meters ↔ pulses (configurable `pulses_per_meter` per joint)
- 7 motion joints via Modbus RTU (3 slave devices)
- 2 container jaw joints via mock interface (state = command, no hardware)
- URDF xacro integration (replace `mock_components/GenericSystem` when `sim=false`)
- Basic error handling, timeout, and connection recovery
- Plugin export via pluginlib for ros2_control discovery

**Out of Scope:**

- Container jaw discrete I/O implementation (future epic)
- Electromagnet control (separate GPIO, handled elsewhere)
- Limit switch hardware reading (handled by separate node in future)
- Advanced motion profiling (handled by JointTrajectoryController)
- Velocity control mode (position-only for this epic)
- Multi-threaded Modbus communication (single-threaded, sequential access)

## System Architecture Alignment

**Architecture Reference:** `docs/architecture-ros2-control-v2-CORRECTIONS.md`

This epic aligns with the existing ros2_control architecture by implementing a hardware-agnostic SystemInterface:

```
┌─────────────────────────────────────────────────────────────────┐
│                     ros2_control Framework                       │
├─────────────────────────────────────────────────────────────────┤
│  JointTrajectoryController (7) │ ForwardCommandController (2)   │
├────────────────────────────────┴────────────────────────────────┤
│                    Hardware Interface Layer                      │
├─────────────────────────────────────────────────────────────────┤
│  ┌─────────────────────────┐  ┌─────────────────────────────┐   │
│  │ ModbusHardwareInterface │  │ MockSystemInterface         │   │
│  │ (7 motion joints)       │  │ (2 container jaws)          │   │
│  └───────────┬─────────────┘  └─────────────────────────────┘   │
│              │                                                   │
│              ▼                                                   │
│  ┌─────────────────────────┐                                    │
│  │   Modbus RTU Driver     │                                    │
│  │   (libmodbus/custom)    │                                    │
│  └───────────┬─────────────┘                                    │
└──────────────┼──────────────────────────────────────────────────┘
               ▼
        /dev/ttyACM0 (Serial)
               │
    ┌──────────┼──────────┐
    ▼          ▼          ▼
┌────────┐ ┌────────┐ ┌────────┐
│Slave 1 │ │Slave 2 │ │Slave 3 │
│engineXZ│ │engineYB│ │engineACD│
│ X, Z   │ │ Y, B   │ │ A,C,D  │
└────────┘ └────────┘ └────────┘
```

**Key Architecture Constraints:**

- **Hot-swappable:** URDF `sim` parameter switches between Gazebo and hardware plugins
- **Interface compatibility:** Exports same state/command interfaces as Gazebo plugin
- **Update rate:** 100 Hz (matched to controller_manager)
- **Lifecycle compliance:** Full ros2_control lifecycle (init → configure → activate)

## Detailed Design

### Services and Modules

| Module | Responsibility | Location |
|--------|---------------|----------|
| `manipulator_hardware` | ROS2 Python package containing hardware interface plugin | `ros2_ws/src/manipulator_hardware/` |
| `ModbusHardwareInterface` | SystemInterface plugin - exports state/command interfaces, orchestrates read/write | `manipulator_hardware/modbus_hardware_interface.py` |
| `ModbusDriver` | Low-level Modbus RTU communication using minimalmodbus (connect, read, write registers) | `manipulator_hardware/modbus_driver.py` |
| `hardware_config.yaml` | Joint-to-hardware mapping configuration | `config/hardware_config.yaml` |

**Implementation Language:** Python with `ros2_control_py` framework (matching existing `examples/modbus_driver/` codebase)

**Package Structure:**

```
ros2_ws/src/manipulator_hardware/
├── CMakeLists.txt                    # Minimal (for ament_python)
├── package.xml
├── setup.py                          # Python package setup
├── setup.cfg
├── manipulator_hardware.xml          # pluginlib export
├── manipulator_hardware/
│   ├── __init__.py
│   ├── modbus_hardware_interface.py  # Main hardware interface
│   └── modbus_driver.py              # Modbus communication layer
├── config/
│   └── hardware_config.yaml
└── test/
    ├── test_modbus_driver.py
    └── test_unit_conversion.py
```

### Data Models and Contracts

**JointConfig Structure:**

```python
# Joint configuration loaded from YAML
joint_config = {
    'name': str,              # ROS2 joint name
    'slave_id': int,          # Modbus slave address (1-3)
    'ordinate': int,          # Axis ordinate on slave (1-3)
    'position_register': int, # Input register for current position
    'command_register': int,  # Holding register for target position
    'pulses_per_meter': float,# Unit conversion factor
    'direction': int,         # 1 or -1 for axis inversion
}
```

**Hardware State Storage:**

```python
# Per-joint state dictionaries (9 joints: 7 motion + 2 mock)
self.hw_positions: Dict[str, float] = {}    # Current positions [meters]
self.hw_velocities: Dict[str, float] = {}   # Current velocities [m/s, estimated]
self.hw_commands: Dict[str, float] = {}     # Commanded positions [meters]
```

**Configuration Schema (hardware_config.yaml):**

```yaml
modbus:
  port: "/dev/ttyACM0"
  baudrate: 115200
  timeout_ms: 500
  retry_count: 3

joints:
  base_main_frame_joint:
    slave_id: 1
    ordinate: 1
    position_register: 1003
    command_register: 3005
    pulses_per_meter: 100000
    direction: 1
  # ... additional joints

status_registers:
  error_code: 999
  module_ready: 1001    # 1 = ready for commands
  module_busy: 1002     # 1 = busy executing
```

### APIs and Interfaces

**ModbusDriver Class (Python):**

```python
from typing import Optional

class ModbusDriver:
    """Modbus RTU communication driver using minimalmodbus."""

    def __init__(self, port: str = '/dev/ttyACM0',
                 baudrate: int = 115200,
                 timeout_sec: float = 0.5,
                 retry_count: int = 3):
        ...

    def connect(self) -> bool:
        """Establish serial connection. Returns True if successful."""

    def disconnect(self) -> None:
        """Close serial connection."""

    def read_position(self, slave_id: int, register: int) -> Optional[int]:
        """Read position from input register (FC4). Returns pulses or None."""

    def write_position(self, slave_id: int, register: int, value: int) -> bool:
        """Write position to holding register (FC6). Returns True if successful."""

    def read_error_code(self, slave_id: int) -> Optional[int]:
        """Read error code register (999)."""

    def is_device_ready(self, slave_id: int) -> Optional[bool]:
        """Check module_ready register (1001). Returns True if ready."""

    def is_device_busy(self, slave_id: int) -> Optional[bool]:
        """Check module_is_busy register (1002). Returns True if busy."""
```

**ModbusHardwareInterface Plugin (Python):**

```python
from hardware_interface import SystemInterface
from hardware_interface.types import HardwareInfo, CallbackReturn, return_type

class ModbusHardwareInterface(SystemInterface):
    """ros2_control hardware interface for Modbus RTU motor controllers."""

    # Lifecycle callbacks
    def on_init(self, hardware_info: HardwareInfo) -> CallbackReturn: ...
    def on_configure(self, previous_state) -> CallbackReturn: ...
    def on_activate(self, previous_state) -> CallbackReturn: ...
    def on_deactivate(self, previous_state) -> CallbackReturn: ...
    def on_cleanup(self, previous_state) -> CallbackReturn: ...
    def on_error(self, previous_state) -> CallbackReturn: ...

    # Interface exports
    def export_state_interfaces(self) -> list: ...
    def export_command_interfaces(self) -> list: ...

    # Control loop callbacks (called at 10 Hz for hardware)
    def read(self, time, period) -> return_type: ...
    def write(self, time, period) -> return_type: ...
```

**Pluginlib Export (manipulator_hardware.xml):**

```xml
<library path="manipulator_hardware">
  <class name="manipulator_hardware/ModbusHardwareInterface"
         type="manipulator_hardware.modbus_hardware_interface.ModbusHardwareInterface"
         base_class_type="hardware_interface::SystemInterface">
    <description>Modbus RTU hardware interface for ya_robot_manipulator (Python)</description>
  </class>
</library>
```

### Workflows and Sequencing

**Lifecycle Sequence:**

```
on_init() ──────────────────────────────────────────────────────────────►
  │ Parse HardwareInfo from URDF
  │ Load hardware_config.yaml
  │ Initialize JointConfig structures for all 9 joints
  │ Identify which joints are Modbus (7) vs mock (2)
  ▼
on_configure() ─────────────────────────────────────────────────────────►
  │ Create ModbusDriver instance
  │ Connect to serial port
  │ Verify communication with all 3 slaves (read status registers)
  │ Return ERROR if any slave unreachable
  ▼
on_activate() ──────────────────────────────────────────────────────────►
  │ Read initial positions from all Modbus joints
  │ Initialize hw_commands_ = hw_positions_ (no immediate motion)
  │ Enable command interfaces
  ▼
read() / write() loop @ 100 Hz ─────────────────────────────────────────►
  │ read():  For each Modbus joint, read position register
  │          Convert pulses → meters, update hw_positions_
  │          Estimate velocity from position delta
  │          For mock joints, hw_positions_ = hw_commands_
  │ write(): For each Modbus joint, if command changed:
  │          Convert meters → pulses, write command register
  │          For mock joints, no hardware write
  ▼
on_deactivate() ────────────────────────────────────────────────────────►
  │ Stop commanding (hold last position)
  │ Disable command interfaces
  ▼
on_cleanup() ───────────────────────────────────────────────────────────►
  │ Disconnect from serial port
  │ Release resources
```

**Unit Conversion Flow:**

```
Controller Command (meters)
         │
         ▼
┌─────────────────────────────────┐
│ pulses = meters × pulses_per_m  │
│        × direction              │
└─────────────────────────────────┘
         │
         ▼
   Modbus Write (pulses)
         │
         ▼
   Hardware Motor Movement
         │
         ▼
   Modbus Read (pulses)
         │
         ▼
┌─────────────────────────────────┐
│ meters = pulses / pulses_per_m  │
│        × direction              │
└─────────────────────────────────┘
         │
         ▼
  State Interface (meters)
```

## Non-Functional Requirements

### Performance

| Metric | Target | Rationale |
|--------|--------|-----------|
| Update rate | 100 Hz (10ms cycle) | Match controller_manager rate |
| Serial timeout | 500ms (configurable) | Allow for Modbus RTU timing |
| Position precision | 0.01mm (1 pulse) | pulses_per_meter = 100,000 |
| Modbus read latency | < 5ms per joint | RTU frame timing |
| Total read cycle | < 50ms for 7 joints | Leaves headroom in 10ms cycle |
| Write optimization | Only write changed commands | Reduce bus traffic |

**Unit Conversion Precision:**
- 1 meter = 100,000 pulses
- 1 mm = 100 pulses
- 0.01 mm = 1 pulse (minimum resolution)
- Example: Joint at 2.5m → 250,000 pulses

### Security

| Requirement | Implementation |
|-------------|----------------|
| Serial port access | Requires `dialout` group membership |
| No network exposure | Modbus RTU is local serial only |
| Configuration validation | YAML schema validation on load |
| Input sanitization | Register values clamped to valid ranges |

**Note:** Hardware safety relies on motor controller firmware limits. The ROS2 interface does not implement software safety limits beyond joint_limits_interface.

### Reliability/Availability

| Requirement | Target | Handling |
|-------------|--------|----------|
| Serial port availability | Required at configure | Return ERROR from on_configure() |
| Modbus timeout | Recoverable | Log warning, return last known position, retry next cycle |
| Device error code | Monitored | Log error, check error register, optionally transition to error state |
| Position out of range | Clamped | Joint limits enforced by controller (not hardware interface) |
| Connection recovery | Automatic | Re-establish connection on next configure cycle |

**Error Handling Strategy:**

```
Timeout during read() → Log WARN, use last position, retry next cycle
Timeout during write() → Log WARN, retry up to retry_count times
Device error != 0 → Log ERROR, read error register, report to controller_manager
Serial disconnect → Return ERROR from read()/write(), trigger on_error()
```

### Observability

| Signal | Type | Purpose |
|--------|------|---------|
| `/joint_states` | sensor_msgs/JointState | Published by joint_state_broadcaster from state interfaces |
| ROS2 logs | rclcpp logging | INFO: lifecycle transitions, WARN: timeouts, ERROR: failures |
| `ros2 control list_hardware_interfaces` | CLI | Verify state/command interfaces exported |
| `ros2 control list_controllers` | CLI | Verify controllers active |

**Diagnostic Topics (future enhancement):**
- `/manipulator_hardware/modbus_stats` - Communication statistics
- `/manipulator_hardware/device_status` - Per-slave error codes

## Dependencies and Integrations

### Framework Dependencies

| Dependency | Version | Purpose |
|------------|---------|---------|
| `hardware_interface` | ros2_control | Base class for SystemInterface |
| `pluginlib` | ros2 | Plugin export and discovery |
| `rclcpp_lifecycle` | ros2 | Lifecycle node support |
| `rclcpp` | ros2 | ROS2 C++ client library |
| `yaml-cpp` | system | YAML configuration parsing |
| `libmodbus` (or custom) | 3.1.x | Modbus RTU communication |

### Internal Module Dependencies

| Module | Dependency | Purpose |
|--------|------------|---------|
| `manipulator_description` | URDF, controller config | ros2_control.xacro, joint definitions |
| `manipulator_control` | Action servers | Use same interfaces (unchanged) |

### Integration Points

**Upstream (receives commands from):**

| Component | Interface | Description |
|-----------|-----------|-------------|
| `controller_manager` | read()/write() calls | 100 Hz control loop |
| JointTrajectoryController (7) | Command interfaces | Position commands for motion joints |
| ForwardCommandController (2) | Command interfaces | Position commands for container jaws |

**Downstream (sends data to):**

| Component | Interface | Description |
|-----------|-----------|-------------|
| `joint_state_broadcaster` | State interfaces | Reads position/velocity, publishes `/joint_states` |
| Modbus slaves (3) | Serial /dev/ttyACM0 | Position commands and readings |

### Configuration Files

| File | Purpose | Modified By |
|------|---------|-------------|
| `manipulator_hardware/config/hardware_config.yaml` | Joint-to-Modbus mapping | **CREATE** (Story 7.1) |
| `manipulator_description/config/manipulator_controllers.yaml` | Controller definitions | **UNCHANGED** |
| `manipulator_description/config/manipulator_params.yaml` | Joint limits | **UNCHANGED** |
| `manipulator_description/urdf/manipulator/ros2_control.xacro` | Hardware plugin selection | **MODIFY** (Story 7.3) |
| `manipulator_description/package.xml` | Add exec_depend | **MODIFY** (Story 7.3) |

### Hardware Mapping Reference

| ROS2 Joint | Axis | Slave ID | Device | Ordinate | Position Reg | Command Reg |
|------------|------|----------|--------|----------|--------------|-------------|
| `base_main_frame_joint` | X | 1 | engineXZ | ord1 | 1003 | 3005 |
| `main_frame_selector_frame_joint` | Z | 1 | engineXZ | ord2 | 1010 | 3015 |
| `selector_frame_gripper_joint` | Y | 2 | engineYB | ord1 | 1003 | 3005 |
| `selector_frame_picker_frame_joint` | A | 3 | engineACD | ord1 | 1003 | 3005 |
| `picker_frame_picker_rail_joint` | B | 2 | engineYB | ord2 | 1010 | 3015 |
| `picker_rail_picker_base_joint` | C | 3 | engineACD | ord2 | 1010 | 3015 |
| `picker_base_picker_jaw_joint` | D | 3 | engineACD | ord3 | 1017 | 3025 |
| `selector_left_container_jaw_joint` | - | - | **Mock** | - | - | - |
| `selector_right_container_jaw_joint` | - | - | **Mock** | - | - | - |

### Critical Hardware Constraints

#### Axis Classification by Motion Type

| Axis Type | Joints | Motion Behavior | Position Feedback |
|-----------|--------|-----------------|-------------------|
| **Continuous** | X, Z, Y, A, B | Servo-like, move to exact position | Accurate encoder |
| **Discrete** | C, D | Move until limit switch activates | Inaccurate during motion |

#### Discrete Axes (C and D) - Special Handling

**Affected joints:**
- `picker_rail_picker_base_joint` (C) - Slave 3, ord2
- `picker_base_picker_jaw_joint` (D) - Slave 3, ord3

**Constraints:**
1. Accept target but move until limit switch (not exact position)
2. Position inaccurate during motion - hold previous value
3. Device busy while motor runs - defer new commands
4. Slave 3: ONE axis at a time (sequential operation)

#### Slave-Level Constraints

| Slave | Device | Simultaneous Motion |
|-------|--------|---------------------|
| 1 | engineXZ | X + Z allowed |
| 2 | engineYB | Y + B allowed |
| 3 | engineACD | **SEQUENTIAL ONLY** |

#### Limit Switch Interface (Future)

State interfaces for limit switches:
- `<joint>/limit_switch_min`
- `<joint>/limit_switch_max`

## Acceptance Criteria (Authoritative)

### AC-7.1: Hardware Interface Package Creation

1. **Given** a ROS2 workspace exists at `ros2_ws/src`
   **When** I run `colcon build --packages-select manipulator_hardware`
   **Then** the package builds successfully with no errors

2. **Given** ModbusDriver is implemented
   **When** I run unit tests with mock serial port
   **Then** all tests pass (connect, read, write, timeout, error handling)

### AC-7.2: Plugin Registration and Loading

3. **Given** ModbusHardwareInterface plugin is implemented
   **When** I query `ros2 control list_hardware_interfaces`
   **Then** the plugin appears in available hardware interfaces

4. **Given** plugin is loaded via ros2_control.xacro with `sim=false`
   **When** controller_manager starts
   **Then** ModbusHardwareInterface exports:
   - 9 position state interfaces
   - 9 velocity state interfaces
   - 9 position command interfaces

### AC-7.3: Hardware Communication

5. **Given** hardware mode (`use_sim_time:=false`)
   **When** launch file executes
   **Then** ModbusHardwareInterface loads and connects to serial port `/dev/ttyACM0`

6. **Given** connected hardware interface
   **When** JointTrajectoryController sends position goal
   **Then** joint moves to target position with accuracy ±0.01m

7. **Given** connected hardware interface
   **When** `/joint_states` is queried
   **Then** positions reflect actual hardware state (read from Modbus registers)

### AC-7.4: Error Handling

8. **Given** serial port disconnected
   **When** hardware interface attempts communication
   **Then** error is logged, interface enters error state gracefully (no crash)

9. **Given** Modbus timeout occurs
   **When** read() is called
   **Then** last known position is returned, warning logged, retry on next cycle

### AC-7.5: Configuration

10. **Given** `hardware_config.yaml` exists
    **When** `pulses_per_meter` is modified for a joint
    **Then** unit conversion reflects new value without code changes

### AC-7.6: URDF Integration

11. **Given** `sim=true` launch argument
    **When** URDF is processed
    **Then** GazeboSimSystem plugin is used (simulation mode works)

12. **Given** `sim=false` launch argument
    **When** URDF is processed
    **Then** ModbusHardwareInterface plugin is used (hardware mode)

## Traceability Mapping

| AC ID | Spec Section | Component | Story | Test Approach |
|-------|--------------|-----------|-------|---------------|
| AC-7.1 | Services and Modules | manipulator_hardware package | 7-1 | Unit test with mock serial |
| AC-7.2 | APIs and Interfaces | ModbusHardwareInterface | 7-2 | Integration test, plugin discovery |
| AC-7.3 | Workflows and Sequencing | ModbusDriver | 7-1 | Unit test, mock Modbus slave |
| AC-7.4 | Data Models | JointConfig, YAML | 7-1, 7-2 | Configuration loading test |
| AC-7.5 | Data Models | Unit conversion | 7-2 | Unit test, precision verification |
| AC-7.6 | Reliability/Availability | Error handling | 7-2 | Fault injection test |
| AC-7.7 | Integration Points | URDF xacro | 7-3 | Launch test, both modes |
| AC-7.8 | Performance | Timing | 7-2 | Profiling, latency measurement |

### FR-to-Story Mapping

| Functional Requirement | Description | Stories |
|-----------------------|-------------|---------|
| FR-HW-001 | Real Hardware Control | 7-1, 7-2, 7-3 |
| FR-HW-002 | Modbus RTU Communication | 7-1 |
| FR-HW-003 | Unit Conversion (meters ↔ pulses) | 7-2 |
| FR-HW-004 | Hot-swappable Simulation/Hardware | 7-3 |
| FR-HW-005 | Configuration from YAML | 7-1, 7-2 |

## Risks, Assumptions, Open Questions

### Risks

| ID | Risk | Impact | Likelihood | Mitigation |
|----|------|--------|------------|------------|
| R1 | Serial port timing conflicts with 100Hz update rate | High | Medium | Profile early, optimize Modbus batching if needed |
| R2 | Modbus library compatibility issues | Medium | Low | Test libmodbus first, have custom implementation backup |
| R3 | Motor controller firmware differences from examples | Medium | Medium | Verify register addresses against actual hardware before integration |
| R4 | Container jaw discrete I/O blocked on mock | Low | High | Mock interface is acceptable for MVP, discrete I/O is future epic |
| R5 | Position drift without encoder feedback | Medium | Medium | Rely on limit switch homing (future), document limitation |

### Assumptions

| ID | Assumption | Validation |
|----|------------|------------|
| A1 | Serial port `/dev/ttyACM0` is available and accessible | Verify during on_configure(), fail gracefully |
| A2 | Motor controllers respond within 500ms timeout | Configurable timeout, tested with real hardware |
| A3 | Register addresses match `examples/modbus_driver/configuration.yml` | Verified against hardware documentation |
| A4 | Position values fit in 32-bit signed integer (±2,147,483,647 pulses) | At 100k pulses/m, supports ±21km range (more than sufficient) |
| A5 | Controllers are stateless between runs (no persistent state) | Standard stepper behavior, confirmed with hardware team |
| A6 | User has `dialout` group membership for serial port access | Document in setup instructions |

### Open Questions

| ID | Question | Owner | Status | Resolution |
|----|----------|-------|--------|------------|
| Q1 | Should we use libmodbus or custom Modbus RTU implementation? | Dev | Open | Evaluate libmodbus first for stability |
| Q2 | What is the exact CRC polynomial for Modbus RTU? | Dev | Resolved | Standard CRC-16 (polynomial 0xA001) |
| Q3 | How to handle motor controller firmware updates? | Hardware | Open | Document firmware version requirements |
| Q4 | Should velocity be estimated or is there a velocity register? | Hardware | Open | Check with hardware team, estimate from position delta initially |

## Test Strategy Summary

### Unit Tests (Story 7-1, 7-2)

| Test | Description | Framework |
|------|-------------|-----------|
| `test_modbus_driver.py` | ModbusDriver connect/disconnect, read/write registers | pytest + unittest.mock |
| `test_unit_conversion.py` | meters ↔ pulses conversion accuracy | pytest |
| `test_config_loading.py` | YAML configuration parsing | pytest |
| `test_timeout_handling.py` | Timeout and retry behavior | pytest + mock |

### Integration Tests (Story 7-2, 7-3)

| Test | Description | Framework |
|------|-------------|-----------|
| Plugin load test | Verify plugin discovered by controller_manager | launch_testing |
| Interface export test | Verify 9 state + 9 command interfaces | ros2 control CLI |
| Lifecycle test | on_init → on_configure → on_activate sequence | launch_testing |
| Simulation fallback | Verify `sim=true` still uses GazeboSimSystem | launch_testing |

### Hardware Tests (Story 7-3)

| Test | Description | Execution |
|------|-------------|-----------|
| Single joint movement | Command one joint, verify position feedback | Manual with real hardware |
| Multi-joint coordinated | Send trajectory to navigation joints | Manual with real hardware |
| Error recovery | Disconnect serial, verify graceful handling | Manual fault injection |
| Long-running stability | 1-hour continuous operation | Overnight test |

### Test Coverage Targets

| Category | Target | Metric |
|----------|--------|--------|
| Unit test coverage | ≥ 80% | Lines covered in ModbusDriver, unit conversion |
| Integration test pass rate | 100% | All launch tests must pass |
| Hardware test success | ≥ 95% | Successful joint movements |

### Test Execution

```bash
# Unit tests
cd ros2_ws
colcon test --packages-select manipulator_hardware

# Integration tests (simulation mode)
ros2 launch manipulator_control manipulator_simulation.launch.py use_sim_time:=true

# Hardware tests (requires real hardware connected)
ros2 launch manipulator_control manipulator_simulation.launch.py use_sim_time:=false
```
