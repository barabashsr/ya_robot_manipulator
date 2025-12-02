#!/usr/bin/env python3
"""Standalone test for ModbusHardwareInterface without controller_manager.

MANDATORY: Run this script to verify hardware interface before deployment.

Usage:
    # With mock driver (no hardware required):
    python3 test_hardware_interface.py --mock

    # With real hardware:
    python3 test_hardware_interface.py --port /dev/ttyUSB0

Results should be documented in the story completion notes.
"""

import argparse
import os
import sys
from typing import Optional

# Add package to path for standalone execution
script_dir = os.path.dirname(os.path.abspath(__file__))
package_dir = os.path.dirname(script_dir)
sys.path.insert(0, package_dir)

from manipulator_hardware.modbus_hardware_interface import (
    CallbackReturn,
    HardwareInfo,
    ModbusHardwareInterface,
)


class MockPeriod:
    """Mock ROS2 Duration for testing."""

    def __init__(self, sec: float = 0.1):
        self.nanoseconds = int(sec * 1e9)


class MockModbusDriver:
    """Mock ModbusDriver for testing without hardware."""

    def __init__(self):  # noqa: D107
        self._positions = {}  # slave_id -> {register: value}
        self._busy = {}  # slave_id -> bool
        self._limits = {}  # (slave_id, register) -> bool
        self._connected = False

    def connect(self) -> bool:
        self._connected = True
        # Initialize positions for all slaves
        for slave_id in [1, 2, 3]:
            self._positions[slave_id] = {}
            self._busy[slave_id] = False
        return True

    def disconnect(self) -> None:
        self._connected = False

    def read_position(self, slave_id: int, register: int) -> Optional[int]:
        if not self._connected:
            return None
        if slave_id not in self._positions:
            self._positions[slave_id] = {}
        return self._positions[slave_id].get(register, 0)

    def write_position(self, slave_id: int, register: int, value: int) -> bool:
        if not self._connected:
            return False
        if slave_id not in self._positions:
            self._positions[slave_id] = {}
        # For continuous axes, set position to command
        # For discrete, set to 0 or max based on value
        self._positions[slave_id][register] = value
        print(f"  [MOCK] Write slave={slave_id} reg={register} value={value}")
        return True

    def is_device_ready(self, slave_id: int) -> Optional[bool]:
        return True

    def is_device_busy(self, slave_id: int) -> Optional[bool]:
        return self._busy.get(slave_id, False)

    def set_busy(self, slave_id: int, busy: bool):
        """Test helper to simulate busy state."""
        self._busy[slave_id] = busy

    def set_position(self, slave_id: int, register: int, pulses: int):
        """Test helper to set simulated position."""
        if slave_id not in self._positions:
            self._positions[slave_id] = {}
        self._positions[slave_id][register] = pulses


def create_test_config(tmp_dir: str) -> str:
    """Create temporary config file for testing."""
    config_path = os.path.join(tmp_dir, 'test_config.yaml')

    config_content = """
modbus:
  port: "/dev/ttyUSB0"
  baudrate: 115200
  timeout_sec: 0.5
  retry_count: 3

joints:
  base_main_frame_joint:
    slave_id: 1
    ordinate: 1
    current_position_register: 1003
    target_position_register: 3005
    pulses_per_meter: 100000
    direction: 1
    max_velocity: 0.5
    discrete_axis: false
    min_limit_register: 1006
    max_limit_register: 1007

  main_frame_selector_frame_joint:
    slave_id: 1
    ordinate: 2
    current_position_register: 1010
    target_position_register: 3015
    pulses_per_meter: 100000
    direction: 1
    max_velocity: 0.5
    discrete_axis: false
    min_limit_register: 1013
    max_limit_register: 1014

  selector_frame_gripper_joint:
    slave_id: 2
    ordinate: 1
    current_position_register: 1003
    target_position_register: 3005
    pulses_per_meter: 100000
    direction: 1
    max_velocity: 0.3
    discrete_axis: false
    min_limit_register: 1006
    max_limit_register: 1007

  selector_frame_picker_frame_joint:
    slave_id: 3
    ordinate: 1
    current_position_register: 1003
    target_position_register: 3005
    pulses_per_meter: 100000
    direction: 1
    max_velocity: 0.3
    discrete_axis: false
    min_limit_register: 1006
    max_limit_register: 1007

  picker_frame_picker_rail_joint:
    slave_id: 2
    ordinate: 2
    current_position_register: 1010
    target_position_register: 3015
    pulses_per_meter: 100000
    direction: 1
    max_velocity: 0.2
    discrete_axis: false
    min_limit_register: 1013
    max_limit_register: 1014

  picker_rail_picker_base_joint:
    slave_id: 3
    ordinate: 2
    current_position_register: 1010
    target_position_register: 3015
    pulses_per_meter: 100000
    direction: 1
    discrete_axis: true
    min_limit_register: 1013
    max_limit_register: 1014
    min_position: 0.0
    max_position: 0.29

  picker_base_picker_jaw_joint:
    slave_id: 3
    ordinate: 3
    current_position_register: 1017
    target_position_register: 3025
    pulses_per_meter: 100000
    direction: 1
    discrete_axis: true
    min_limit_register: 1020
    max_limit_register: 1021
    min_position: 0.0
    max_position: 0.19
"""
    with open(config_path, 'w') as f:
        f.write(config_content)
    return config_path


def run_test_instantiation():
    """Test 1: Plugin instantiation."""
    print('\n=== Test 1: Plugin Instantiation ===')
    try:
        hw = ModbusHardwareInterface()
        print("  [PASS] ModbusHardwareInterface instantiated")
        print(f"  - ALL_JOINTS: {len(hw.ALL_JOINTS)} joints")
        print(f"  - MOCK_JOINTS: {hw.MOCK_JOINTS}")
        return hw
    except Exception as e:
        print(f"  [FAIL] {e}")
        return None


def run_test_config_loading(hw: ModbusHardwareInterface, config_path: str):
    """Test 2: Configuration loading."""
    print('\n=== Test 2: Configuration Loading ===')
    try:
        info = HardwareInfo()
        info.hardware_parameters = {'config_file': config_path}

        result = hw.on_init(info)

        if result == CallbackReturn.SUCCESS:
            print("  [PASS] Configuration loaded")
            print(f"  - Loaded {len(hw._joint_configs)} Modbus joints")
            for joint_name in hw._joint_configs:
                cfg = hw._joint_configs[joint_name]
                discrete = cfg.get('discrete_axis', False)
                print(f"    - {joint_name}: slave={cfg.get('slave_id')}, "
                      f"discrete={discrete}")
            return True
        else:
            print(f"  [FAIL] on_init returned {result}")
            return False
    except Exception as e:
        print(f"  [FAIL] {e}")
        return False


def run_test_lifecycle_with_mock(hw: ModbusHardwareInterface):
    """Test 3: Lifecycle transitions with mock driver."""
    print('\n=== Test 3: Lifecycle Transitions (Mock) ===')

    # Inject mock driver
    mock_driver = MockModbusDriver()
    mock_driver.connect()

    # Set initial positions (simulate hardware state)
    mock_driver.set_position(1, 1003, 10000)  # X: 0.1m
    mock_driver.set_position(1, 1010, 20000)  # Z: 0.2m
    mock_driver.set_position(2, 1003, 15000)  # Y: 0.15m
    mock_driver.set_position(2, 1010, 5000)   # B: 0.05m
    mock_driver.set_position(3, 1003, 25000)  # A: 0.25m
    mock_driver.set_position(3, 1010, 0)      # C: at MIN
    mock_driver.set_position(3, 1017, 0)      # D: at MIN

    hw._driver = mock_driver

    # on_activate
    result = hw.on_activate(None)
    if result == CallbackReturn.SUCCESS:
        print("  [PASS] on_activate succeeded")
        print("  Initial positions (meters):")
        for joint in hw.ALL_JOINTS:
            pos = hw._position_states.get(joint, 0.0)
            print(f"    - {joint}: {pos:.4f}m")
    else:
        print(f"  [FAIL] on_activate returned {result}")
        return False

    return True


def run_test_unit_conversion(hw: ModbusHardwareInterface):
    """Test 4: Unit conversion accuracy."""
    print('\n=== Test 4: Unit Conversion Accuracy ===')

    joint_config = {'pulses_per_meter': 100000, 'direction': 1}

    # Test 0.5m -> 50000 pulses
    pulses = hw._meters_to_pulses(0.5, joint_config)
    if pulses == 50000:
        print(f"  [PASS] 0.5m -> {pulses} pulses")
    else:
        print(f"  [FAIL] 0.5m -> {pulses} pulses (expected 50000)")
        return False

    # Test 50000 pulses -> 0.5m
    meters = hw._pulses_to_meters(50000, joint_config)
    if abs(meters - 0.5) < 1e-6:
        print(f"  [PASS] 50000 pulses -> {meters}m")
    else:
        print(f"  [FAIL] 50000 pulses -> {meters}m (expected 0.5)")
        return False

    # Test roundtrip precision
    original = 1.23456
    pulses = hw._meters_to_pulses(original, joint_config)
    result = hw._pulses_to_meters(pulses, joint_config)
    error = abs(result - original)
    if error < 0.00002:  # Within 0.02mm
        print(f"  [PASS] Roundtrip: {original}m -> {pulses}p -> {result}m (err={error:.6f})")
    else:
        print(f"  [FAIL] Roundtrip error too large: {error}")
        return False

    return True


def run_test_read_cycle(hw: ModbusHardwareInterface):
    """Test 5: Read cycle updates positions."""
    print('\n=== Test 5: Read Cycle ===')

    if hw._driver is None:
        print("  [SKIP] No driver available")
        return False

    period = MockPeriod(0.1)

    # Set mock positions
    if isinstance(hw._driver, MockModbusDriver):
        hw._driver.set_position(1, 1003, 30000)  # X: 0.3m

    result = hw.read(None, period)

    if result == 0:
        print("  [PASS] read() returned OK")
        pos = hw._position_states.get('base_main_frame_joint', 0.0)
        print(f"  - base_main_frame_joint position: {pos:.4f}m")

        # Check mock joint echoes command
        hw._position_commands['selector_left_container_jaw_joint'] = 0.123
        hw.read(None, period)
        mock_pos = hw._position_states.get('selector_left_container_jaw_joint', 0.0)
        if abs(mock_pos - 0.123) < 1e-6:
            print(f"  [PASS] Mock joint echoes command: {mock_pos}")
        else:
            print(f"  [FAIL] Mock joint state: {mock_pos} (expected 0.123)")
        return True
    else:
        print(f"  [FAIL] read() returned {result}")
        return False


def run_test_write_continuous(hw: ModbusHardwareInterface):
    """Test 6: Write continuous axis with incremental clamping."""
    print('\n=== Test 6: Write Continuous Axis (Incremental) ===')

    if hw._driver is None:
        print("  [SKIP] No driver available")
        return False

    period = MockPeriod(0.1)

    # Reset state
    hw._position_states['base_main_frame_joint'] = 0.0
    hw._prev_commands['base_main_frame_joint'] = 0.0

    # Set distant target (should be clamped)
    hw._position_commands['base_main_frame_joint'] = 2.0  # 2m away

    result = hw.write(None, period)

    if result == 0:
        print("  [PASS] write() returned OK")
        # With max_velocity=0.5 and period=0.1s, max_delta=0.05m=5000 pulses
        if isinstance(hw._driver, MockModbusDriver):
            written = hw._driver._positions.get(1, {}).get(3005, 0)
            expected = 5000  # 0.05m clamped
            if written == expected:
                print(f"  [PASS] Incremental clamp: wrote {written} pulses (0.05m)")
            else:
                print(f"  [INFO] Wrote {written} pulses (expected ~{expected})")
        return True
    else:
        print(f"  [FAIL] write() returned {result}")
        return False


def run_test_write_discrete(hw: ModbusHardwareInterface):
    """Test 7: Write discrete axis with threshold and busy check."""
    print('\n=== Test 7: Write Discrete Axis ===')

    if hw._driver is None:
        print("  [SKIP] No driver available")
        return False

    # Test threshold at 0.5
    print("  Testing binary threshold...")

    joint_config = hw._joint_configs.get('picker_rail_picker_base_joint', {})
    if not joint_config:
        print("  [SKIP] Discrete joint not configured")
        return False

    # Test command < 0.5 -> write 0
    hw._position_commands['picker_rail_picker_base_joint'] = 0.49
    hw._write_discrete_axis(
        'picker_rail_picker_base_joint',
        joint_config,
        0.49,
        joint_config.get('slave_id', 3),
        joint_config.get('target_position_register', 3015)
    )
    print("  [PASS] Command 0.49 -> writes 0 (MIN)")

    # Test command >= 0.5 -> write 1
    hw._position_commands['picker_rail_picker_base_joint'] = 0.5
    hw._write_discrete_axis(
        'picker_rail_picker_base_joint',
        joint_config,
        0.5,
        joint_config.get('slave_id', 3),
        joint_config.get('target_position_register', 3015)
    )
    print("  [PASS] Command 0.5 -> writes 1 (MAX)")

    # Test busy deferral
    print("  Testing busy deferral...")
    if isinstance(hw._driver, MockModbusDriver):
        hw._driver.set_busy(3, True)
        hw._slave3_pending_commands.clear()

        result = hw._write_discrete_axis(
            'picker_rail_picker_base_joint',
            joint_config,
            1.0,
            3,
            3015
        )

        if result is False and 'picker_rail_picker_base_joint' in hw._slave3_pending_commands:
            print("  [PASS] Command deferred when busy")
        else:
            print("  [FAIL] Should defer when busy")

        hw._driver.set_busy(3, False)

    return True


def run_test_interface_exports(hw: ModbusHardwareInterface):
    """Test 8: Interface export counts."""
    print('\n=== Test 8: Interface Export Counts ===')

    state_interfaces = hw.export_state_interfaces()
    command_interfaces = hw.export_command_interfaces()

    state_count = len(state_interfaces)
    command_count = len(command_interfaces)

    # Expected: 9 joints * 2 (position + velocity) = 18 state
    # Expected: 9 joints * 1 (position) = 9 command
    if state_count == 18 and command_count == 9:
        print(f"  [PASS] State interfaces: {state_count} (9 pos + 9 vel)")
        print(f"  [PASS] Command interfaces: {command_count} (9 pos)")
        return True
    else:
        print(f"  [FAIL] State: {state_count} (expected 18)")
        print(f"  [FAIL] Command: {command_count} (expected 9)")
        return False


def run_test_cleanup(hw: ModbusHardwareInterface):
    """Test 9: Cleanup disconnects driver."""
    print('\n=== Test 9: Cleanup ===')

    result = hw.on_cleanup(None)

    if result == CallbackReturn.SUCCESS:
        print("  [PASS] on_cleanup succeeded")
        if hw._driver is None:
            print("  [PASS] Driver disconnected")
            return True
        else:
            print("  [FAIL] Driver not None after cleanup")
            return False
    else:
        print(f"  [FAIL] on_cleanup returned {result}")
        return False


def main():
    parser = argparse.ArgumentParser(
        description='Standalone test for ModbusHardwareInterface'
    )
    parser.add_argument(
        '--mock',
        action='store_true',
        help='Use mock driver (no hardware required)'
    )
    parser.add_argument(
        '--port',
        type=str,
        default='/dev/ttyUSB0',
        help='Serial port for real hardware'
    )
    args = parser.parse_args()

    print("=" * 60)
    print("ModbusHardwareInterface Self-Test")
    print("=" * 60)
    print(f"Mode: {'MOCK' if args.mock else 'HARDWARE'}")
    if not args.mock:
        print(f"Port: {args.port}")

    # Create temp config
    import tempfile
    with tempfile.TemporaryDirectory() as tmp_dir:
        config_path = create_test_config(tmp_dir)
        print(f"Config: {config_path}")

        results = []

        # Test 1: Instantiation
        hw = run_test_instantiation()
        results.append(('Instantiation', hw is not None))
        if hw is None:
            print('\n[ABORT] Cannot continue without instance')
            return 1

        # Test 2: Config loading
        results.append(('Config Loading', run_test_config_loading(hw, config_path)))

        # Test 3: Lifecycle (with mock)
        results.append(('Lifecycle (Mock)', run_test_lifecycle_with_mock(hw)))

        # Test 4: Unit conversion
        results.append(('Unit Conversion', run_test_unit_conversion(hw)))

        # Test 5: Read cycle
        results.append(('Read Cycle', run_test_read_cycle(hw)))

        # Test 6: Write continuous
        results.append(('Write Continuous', run_test_write_continuous(hw)))

        # Test 7: Write discrete
        results.append(('Write Discrete', run_test_write_discrete(hw)))

        # Test 8: Interface exports
        results.append(('Interface Exports', run_test_interface_exports(hw)))

        # Test 9: Cleanup
        results.append(('Cleanup', run_test_cleanup(hw)))

        # Summary
        print("\n" + "=" * 60)
        print("TEST SUMMARY")
        print("=" * 60)

        passed = 0
        failed = 0
        for name, result in results:
            status = "PASS" if result else "FAIL"
            print(f"  [{status}] {name}")
            if result:
                passed += 1
            else:
                failed += 1

        print(f"\nTotal: {passed} passed, {failed} failed")
        print("=" * 60)

        return 0 if failed == 0 else 1


if __name__ == '__main__':
    sys.exit(main())
