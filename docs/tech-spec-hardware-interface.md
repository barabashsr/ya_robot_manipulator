# ya_robot_manipulator - Hardware Interface Technical Specification

**Author:** BMad
**Date:** 2025-11-27
**Project Level:** Level 3 (Robotics Control System)
**Change Type:** New Epic - Hardware Interface for Real Robot Control
**Development Context:** Brownfield (integrating into existing ros2_control framework)

---

## Context

### Available Documents

- **PRD:** `docs/prd.md` - Product requirements for warehouse automation
- **Architecture:** `docs/architecture-ros2-control-v2-CORRECTIONS.md` - Complete ROS2 control architecture
- **Epics:** `docs/epics.md` - 6 existing epics (42 stories) for simulation-based development
- **Modbus Driver:** `examples/modbus_driver/` - Working Python script for motor control via Modbus RTU

### Project Stack

| Component | Version/Details |
|-----------|-----------------|
| ROS2 | Humble/Jazzy |
| ros2_control | Standard ros2_control framework |
| Simulation | gz_ros2_control (Gazebo) |
| Controllers | 7 JointTrajectoryController + 2 ForwardCommandController |
| Python | 3.10+ |
| Modbus Library | minimalmodbus |

### Existing Codebase Structure

**ros2_control URDF Integration:**
- `ros2_ws/src/manipulator_description/urdf/manipulator/ros2_control.xacro`
- Hardware plugin selection via `sim` xacro parameter (line 27-41)
- Current hardware-mode placeholder: `mock_components/GenericSystem`

**Controller Configuration:**
- `ros2_ws/src/manipulator_description/config/manipulator_controllers.yaml`
- 9 joints, hybrid controller architecture

**Existing Modbus Driver:**
- `examples/modbus_driver/modbus_rtu.py` - Low-level Modbus communication
- `examples/modbus_driver/control.py` - High-level motor control functions
- `examples/modbus_driver/configuration.yml` - Register mapping and device config

---

## The Change

### Problem Statement

The manipulator control system currently operates only in simulation using `gz_ros2_control/GazeboSimSystem`. To control the real hardware, we need a ROS2 hardware interface plugin that:

1. Communicates with stepper motor controllers via Modbus RTU
2. Converts between ROS2 position units (meters) and hardware units (pulses)
3. Integrates seamlessly with existing JointTrajectoryControllers
4. Is hot-swappable with simulation via URDF xacro parameter

### Proposed Solution

Implement a **ros2_control SystemInterface plugin** (`ModbusHardwareInterface`) that:

1. **Reads** joint positions from Modbus input registers → publishes to `/joint_states`
2. **Writes** commanded positions to Modbus holding registers
3. **Loads** configuration (device mapping, unit conversion, registers) from YAML
4. **Handles** 7 motion joints via Modbus, 2 container jaws via mock (future: discrete I/O)

**Architecture:**

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
│  │   (minimalmodbus)       │                                    │
│  └───────────┬─────────────┘                                    │
│              │                                                   │
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

### Scope

**In Scope:**

- ros2_control `SystemInterface` plugin implementation in C++ (or Python via hardware_interface)
- YAML configuration for joint-to-Modbus mapping
- Unit conversion: meters ↔ pulses (configurable pulses_per_meter)
- 7 motion joints via Modbus RTU
- 2 container jaws via mock interface (placeholder for future discrete I/O)
- URDF xacro integration (replace `mock_components/GenericSystem`)
- Basic error handling and connection recovery

**Out of Scope:**

- Container jaw discrete I/O implementation (future epic)
- Electromagnet control (separate GPIO, handled elsewhere)
- Limit switch hardware reading (handled by separate node in future)
- Advanced motion profiling (handled by JointTrajectoryController)
- Velocity control mode (position-only for now)

---

## Implementation Details

### Source Tree Changes

| Path | Action | Description |
|------|--------|-------------|
| `ros2_ws/src/manipulator_hardware/` | **CREATE** | New package for hardware interface |
| `ros2_ws/src/manipulator_hardware/CMakeLists.txt` | **CREATE** | Build configuration |
| `ros2_ws/src/manipulator_hardware/package.xml` | **CREATE** | Package dependencies |
| `ros2_ws/src/manipulator_hardware/include/manipulator_hardware/modbus_hardware_interface.hpp` | **CREATE** | Hardware interface header |
| `ros2_ws/src/manipulator_hardware/src/modbus_hardware_interface.cpp` | **CREATE** | Hardware interface implementation |
| `ros2_ws/src/manipulator_hardware/src/modbus_driver.cpp` | **CREATE** | Modbus RTU communication layer |
| `ros2_ws/src/manipulator_hardware/config/hardware_config.yaml` | **CREATE** | Hardware configuration |
| `ros2_ws/src/manipulator_description/urdf/manipulator/ros2_control.xacro` | **MODIFY** | Replace mock plugin with modbus plugin |
| `docs/epics.md` | **MODIFY** | Add Epic 7 |

### Technical Approach

**1. Hardware Interface Plugin (C++)**

Implement `hardware_interface::SystemInterface`:

```cpp
class ModbusHardwareInterface : public hardware_interface::SystemInterface
{
public:
  // Lifecycle callbacks
  CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;
  CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

  // Interface exports
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  // Control loop callbacks (called at controller_manager update_rate)
  return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;
  return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  std::unique_ptr<ModbusDriver> modbus_driver_;
  std::vector<JointConfig> joint_configs_;

  // State storage
  std::vector<double> hw_positions_;      // Current positions (meters)
  std::vector<double> hw_velocities_;     // Current velocities (m/s)
  std::vector<double> hw_commands_;       // Commanded positions (meters)
};
```

**2. Modbus Driver Layer**

Wrapper around minimalmodbus (or libmodbus for C++):

```cpp
class ModbusDriver
{
public:
  bool connect(const std::string& port, int baudrate);
  void disconnect();

  // Read current position (returns pulses, converted to meters by caller)
  std::optional<int32_t> read_position(uint8_t slave_id, uint16_t register_addr);

  // Write target position (accepts pulses)
  bool write_position(uint8_t slave_id, uint16_t register_addr, int32_t pulses);

  // Status queries
  std::optional<uint16_t> read_error_code(uint8_t slave_id);
  std::optional<bool> is_device_ready(uint8_t slave_id);
  std::optional<bool> is_device_busy(uint8_t slave_id);
};
```

**3. YAML Configuration Structure**

```yaml
# hardware_config.yaml
modbus:
  port: "/dev/ttyACM0"
  baudrate: 115200
  timeout_ms: 500
  retry_count: 3

# Joint-to-hardware mapping
joints:
  base_main_frame_joint:
    slave_id: 1                    # engineXZ
    ordinate: 1                    # ord1 = X axis
    position_register: 1003        # inp_reg.ord1_current
    command_register: 2999         # hold_reg.ord1_given
    pulses_per_meter: 100000       # 0.01mm precision = 100,000 pulses/meter
    direction: 1                   # 1 or -1 for axis inversion

  main_frame_selector_frame_joint:
    slave_id: 1                    # engineXZ
    ordinate: 2                    # ord2 = Z axis
    position_register: 1010        # inp_reg.ord2_current
    command_register: 3008         # hold_reg.ord2_given
    pulses_per_meter: 100000
    direction: 1

  selector_frame_gripper_joint:
    slave_id: 2                    # engineYB
    ordinate: 1                    # ord1 = Y axis
    position_register: 1003
    command_register: 2999
    pulses_per_meter: 100000
    direction: 1

  selector_frame_picker_frame_joint:
    slave_id: 3                    # engineACD
    ordinate: 1                    # ord1 = A axis
    position_register: 1003
    command_register: 2999
    pulses_per_meter: 100000
    direction: 1

  picker_frame_picker_rail_joint:
    slave_id: 2                    # engineYB
    ordinate: 2                    # ord2 = B axis
    position_register: 1010
    command_register: 3008
    pulses_per_meter: 100000
    direction: 1

  picker_rail_picker_base_joint:
    slave_id: 3                    # engineACD
    ordinate: 2                    # ord2 = C axis
    position_register: 1010
    command_register: 3008
    pulses_per_meter: 100000
    direction: 1

  picker_base_picker_jaw_joint:
    slave_id: 3                    # engineACD
    ordinate: 3                    # ord3 = D axis
    position_register: 1017
    command_register: 3017
    pulses_per_meter: 100000
    direction: 1

# Status registers (per slave)
status_registers:
  error_code: 999                  # inp_reg.err_code
  module_ready: 1002               # inp_reg.module_ready
  module_busy: 1001                # inp_reg.module_is_busy
```

**4. URDF Integration**

Modify `ros2_control.xacro`:

```xml
<xacro:unless value="${sim}">
  <hardware>
    <plugin>manipulator_hardware/ModbusHardwareInterface</plugin>
    <param name="config_file">$(find manipulator_hardware)/config/hardware_config.yaml</param>
  </hardware>
</xacro:unless>
```

### Existing Patterns to Follow

**From existing ros2_control.xacro:**
- Load parameters from external YAML via `xacro.load_yaml()`
- Use `sim` parameter for hardware switching
- Joint interface definitions (position command, position/velocity state)

**From existing modbus driver:**
- ConfigLoader pattern for YAML configuration
- Register addressing scheme (inp_reg, hold_reg, disc_o_reg)
- Error handling with None returns and exception catching
- Device status checking before commands

**From ros2_control conventions:**
- SystemInterface lifecycle (on_init → on_configure → on_activate)
- Export state/command interfaces per joint
- read()/write() called at controller_manager update_rate (100 Hz)

### Integration Points

**Upstream (receives commands from):**
- `controller_manager` calls `read()` and `write()` at 100 Hz
- JointTrajectoryControllers write to command interfaces
- ForwardCommandControllers (for container jaws - mock)

**Downstream (sends data to):**
- `/joint_states` (via joint_state_broadcaster reading state interfaces)
- Modbus devices via serial port

**Configuration Files:**
- `manipulator_hardware/config/hardware_config.yaml` - Hardware mapping
- `manipulator_description/config/manipulator_controllers.yaml` - Controllers (unchanged)
- `manipulator_description/config/manipulator_params.yaml` - Joint limits (unchanged)

---

## Development Context

### Relevant Existing Code

| File | Reference |
|------|-----------|
| `examples/modbus_driver/modbus_rtu.py` | Modbus communication patterns |
| `examples/modbus_driver/control.py` | Register mappings, status checks |
| `examples/modbus_driver/configuration.yml` | Device and register configuration |
| `ros2_ws/src/manipulator_description/urdf/manipulator/ros2_control.xacro:27-41` | Hardware plugin switching |

### Dependencies

**Framework/Libraries:**

| Dependency | Purpose |
|------------|---------|
| `hardware_interface` | ros2_control hardware interface base classes |
| `pluginlib` | Plugin export macros |
| `rclcpp_lifecycle` | Lifecycle node support |
| `libmodbus` (or custom) | Modbus RTU communication |

**Internal Modules:**

- `manipulator_description` - URDF, controller config, joint params
- `manipulator_control` - Action servers (unchanged, use same interfaces)

### Configuration Changes

| File | Change |
|------|--------|
| `manipulator_description/urdf/manipulator/ros2_control.xacro` | Replace mock plugin reference |
| `manipulator_description/package.xml` | Add `manipulator_hardware` exec_depend |

### Test Framework & Standards

**Unit Tests:** pytest (Python) / gtest (C++)
**Integration Tests:** Launch tests with mock serial port
**Hardware Tests:** Manual verification with real hardware

---

## Implementation Stack

| Layer | Technology |
|-------|------------|
| ROS2 Framework | ros2_control, hardware_interface |
| Controllers | JointTrajectoryController, ForwardCommandController |
| Hardware Plugin | C++ SystemInterface |
| Communication | libmodbus or custom Modbus RTU |
| Configuration | YAML |
| Testing | gtest, launch_testing |

---

## Technical Details

### Unit Conversion

**Precision requirement:** 0.01mm = 0.00001m

**Conversion formula:**
```
pulses = position_meters * pulses_per_meter * direction
position_meters = pulses / pulses_per_meter * direction
```

**With pulses_per_meter = 100,000:**
- 1 meter = 100,000 pulses
- 1 mm = 100 pulses
- 0.01 mm = 1 pulse (minimum resolution)

**Example:** Joint at 2.5m → 250,000 pulses

### Modbus Communication Timing

- **Update rate:** 100 Hz (10ms cycle)
- **Serial timeout:** 500ms (configurable)
- **Retry count:** 3 attempts before error

**Per-cycle operations:**
1. Read 7 position registers (7 × ~5ms with RTU timing)
2. Write 7 command registers (if changed)

**Optimization:**
- Only write changed commands
- Use multi-register reads where possible (slaves support different registers)

### Error Handling

| Error | Handling |
|-------|----------|
| Serial port unavailable | Return ERROR from on_configure(), prevent activation |
| Modbus timeout | Log warning, return last known position, retry next cycle |
| Device error code != 0 | Log error, check error register, optionally stop |
| Position out of range | Clamp to joint limits (already enforced by controller) |

### Thread Safety

- `read()` and `write()` called from single controller_manager thread
- No additional threading in hardware interface
- Serial port access is sequential (not concurrent)

---

## Development Setup

```bash
# 1. Create the package
cd ros2_ws/src
ros2 pkg create manipulator_hardware \
  --build-type ament_cmake \
  --dependencies hardware_interface pluginlib rclcpp_lifecycle

# 2. Build
cd ros2_ws
colcon build --packages-select manipulator_hardware

# 3. Test with mock serial (simulation mode still works)
ros2 launch manipulator_control manipulator_simulation.launch.py use_sim_time:=true

# 4. Test with real hardware
ros2 launch manipulator_control manipulator_simulation.launch.py use_sim_time:=false
```

---

## Implementation Guide

### Setup Steps

1. Create `manipulator_hardware` package structure
2. Set up CMakeLists.txt with hardware_interface dependencies
3. Create plugin export XML for pluginlib

### Implementation Steps

**Story 7-1: Create Hardware Interface Package and Modbus Driver**
1. Create package scaffolding
2. Implement ModbusDriver class (C++ wrapper for serial/Modbus)
3. Unit test with mock serial port

**Story 7-2: Implement ModbusHardwareInterface Plugin**
1. Implement SystemInterface lifecycle callbacks
2. Load configuration from YAML
3. Implement read()/write() with unit conversion
4. Export plugin via pluginlib
5. Integration test with mock

**Story 7-3: URDF Integration and Hardware Testing**
1. Modify ros2_control.xacro to use new plugin
2. Test with real hardware
3. Tune timing and error handling
4. Documentation

### Testing Strategy

**Unit Tests:**
- ModbusDriver: Mock serial port, verify register read/write
- Unit conversion: Verify meters ↔ pulses accuracy
- Configuration loading: Verify YAML parsing

**Integration Tests:**
- Launch test: Verify plugin loads correctly
- Controller test: Send trajectory, verify execution
- Error handling: Simulate timeout, verify recovery

**Hardware Tests:**
- Single joint movement
- Multi-joint coordinated motion
- Long-running stability test

### Acceptance Criteria

1. **Given** hardware mode (`use_sim_time:=false`)
   **When** launch file executes
   **Then** ModbusHardwareInterface loads and connects to serial port

2. **Given** connected hardware interface
   **When** JointTrajectoryController sends goal
   **Then** joint moves to target position (±0.01m accuracy)

3. **Given** connected hardware interface
   **When** `/joint_states` is queried
   **Then** positions reflect actual hardware state

4. **Given** serial port disconnected
   **When** hardware interface attempts communication
   **Then** error is logged, interface enters error state gracefully

5. **Given** configuration YAML
   **When** `pulses_per_meter` is modified
   **Then** unit conversion reflects new value without code changes

---

## Developer Resources

### File Paths Reference

```
ros2_ws/src/manipulator_hardware/
├── CMakeLists.txt
├── package.xml
├── manipulator_hardware.xml          # pluginlib export
├── include/
│   └── manipulator_hardware/
│       ├── modbus_hardware_interface.hpp
│       ├── modbus_driver.hpp
│       └── visibility_control.h
├── src/
│   ├── modbus_hardware_interface.cpp
│   └── modbus_driver.cpp
├── config/
│   └── hardware_config.yaml
└── test/
    ├── test_modbus_driver.cpp
    └── test_unit_conversion.cpp
```

### Key Code Locations

| Component | Location |
|-----------|----------|
| Hardware plugin switching | `manipulator_description/urdf/manipulator/ros2_control.xacro:27-41` |
| Controller config | `manipulator_description/config/manipulator_controllers.yaml` |
| Joint limits | `manipulator_description/config/manipulator_params.yaml` |
| Example Modbus code | `examples/modbus_driver/` |

### Documentation to Update

- `docs/epics.md` - Add Epic 7
- `ros2_ws/src/manipulator_hardware/README.md` - Package documentation
- `docs/architecture-ros2-control-v2-CORRECTIONS.md` - Add hardware interface section

---

## UX/UI Considerations

No UI/UX impact - backend/infrastructure change only.

---

## Deployment Strategy

### Deployment Steps

1. Build `manipulator_hardware` package
2. Verify simulation still works (`use_sim_time:=true`)
3. Connect to real hardware, launch with `use_sim_time:=false`
4. Monitor for communication errors
5. Run acceptance tests

### Rollback Plan

1. Revert ros2_control.xacro to use `mock_components/GenericSystem`
2. Rebuild manipulator_description
3. Simulation continues to work

### Monitoring

- Check `/joint_states` for position updates
- Monitor ROS2 logs for Modbus errors
- Verify controller status via `ros2 control list_controllers`

---

## Joint-to-Hardware Mapping Reference

| ROS2 Joint | Axis | Slave ID | Device | Ordinate | Position Reg | Command Reg |
|------------|------|----------|--------|----------|--------------|-------------|
| `base_main_frame_joint` | X | 1 | engineXZ | ord1 | 1003 | 2999 |
| `main_frame_selector_frame_joint` | Z | 1 | engineXZ | ord2 | 1010 | 3008 |
| `selector_frame_gripper_joint` | Y | 2 | engineYB | ord1 | 1003 | 2999 |
| `selector_frame_picker_frame_joint` | A | 3 | engineACD | ord1 | 1003 | 2999 |
| `picker_frame_picker_rail_joint` | B | 2 | engineYB | ord2 | 1010 | 3008 |
| `picker_rail_picker_base_joint` | C | 3 | engineACD | ord2 | 1010 | 3008 |
| `picker_base_picker_jaw_joint` | D | 3 | engineACD | ord3 | 1017 | 3017 |
| `selector_left_container_jaw_joint` | - | - | **Mock** | - | - | - |
| `selector_right_container_jaw_joint` | - | - | **Mock** | - | - | - |

**Note:** Container jaws use mock interface. Future epic will add discrete I/O control.
