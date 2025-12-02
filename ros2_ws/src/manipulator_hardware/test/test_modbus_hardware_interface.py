"""Unit tests for ModbusHardwareInterface class."""

import os
from unittest.mock import MagicMock, patch

import pytest


class TestModbusHardwareInterfaceImport:
    """Test importing ModbusHardwareInterface."""

    def test_import(self):
        """Test that ModbusHardwareInterface can be imported."""
        from manipulator_hardware.modbus_hardware_interface import ModbusHardwareInterface
        assert ModbusHardwareInterface is not None


class TestModbusHardwareInterfaceInit:
    """Test ModbusHardwareInterface initialization."""

    def test_init_creates_storage(self):
        """Test initialization creates state and command storage."""
        from manipulator_hardware.modbus_hardware_interface import ModbusHardwareInterface

        hw = ModbusHardwareInterface()

        # Check all joints have storage initialized
        assert len(hw._position_states) == 9
        assert len(hw._velocity_states) == 9
        assert len(hw._position_commands) == 9

        # Check mock joints are defined
        assert 'selector_left_container_jaw_joint' in hw.MOCK_JOINTS
        assert 'selector_right_container_jaw_joint' in hw.MOCK_JOINTS

    def test_all_joints_defined(self):
        """Test all 9 joints are defined."""
        from manipulator_hardware.modbus_hardware_interface import ModbusHardwareInterface

        hw = ModbusHardwareInterface()

        assert len(hw.ALL_JOINTS) == 9
        assert 'base_main_frame_joint' in hw.ALL_JOINTS
        assert 'picker_base_picker_jaw_joint' in hw.ALL_JOINTS


class TestUnitConversion:
    """Test unit conversion methods (AC: #4, #5)."""

    def test_pulses_to_meters_default(self):
        """Test pulses to meters with default config."""
        from manipulator_hardware.modbus_hardware_interface import ModbusHardwareInterface

        hw = ModbusHardwareInterface()
        joint_config = {'pulses_per_meter': 100000, 'direction': 1}

        # 50000 pulses = 0.5 meters
        result = hw._pulses_to_meters(50000, joint_config)
        assert abs(result - 0.5) < 1e-6

    def test_pulses_to_meters_negative_direction(self):
        """Test pulses to meters with negative direction."""
        from manipulator_hardware.modbus_hardware_interface import ModbusHardwareInterface

        hw = ModbusHardwareInterface()
        joint_config = {'pulses_per_meter': 100000, 'direction': -1}

        # 50000 pulses = -0.5 meters (inverted)
        result = hw._pulses_to_meters(50000, joint_config)
        assert abs(result - (-0.5)) < 1e-6

    def test_meters_to_pulses_default(self):
        """Test meters to pulses with default config."""
        from manipulator_hardware.modbus_hardware_interface import ModbusHardwareInterface

        hw = ModbusHardwareInterface()
        joint_config = {'pulses_per_meter': 100000, 'direction': 1}

        # 0.5 meters = 50000 pulses
        result = hw._meters_to_pulses(0.5, joint_config)
        assert result == 50000

    def test_meters_to_pulses_negative_direction(self):
        """Test meters to pulses with negative direction."""
        from manipulator_hardware.modbus_hardware_interface import ModbusHardwareInterface

        hw = ModbusHardwareInterface()
        joint_config = {'pulses_per_meter': 100000, 'direction': -1}

        # 0.5 meters = -50000 pulses (inverted)
        result = hw._meters_to_pulses(0.5, joint_config)
        assert result == -50000

    def test_conversion_roundtrip(self):
        """Test meters->pulses->meters roundtrip accuracy."""
        from manipulator_hardware.modbus_hardware_interface import ModbusHardwareInterface

        hw = ModbusHardwareInterface()
        joint_config = {'pulses_per_meter': 100000, 'direction': 1}

        original = 1.23456
        pulses = hw._meters_to_pulses(original, joint_config)
        result = hw._pulses_to_meters(pulses, joint_config)

        # Should be within 0.01mm (1 pulse = 0.00001m)
        assert abs(result - original) < 0.00002


class TestInterfaceExports:
    """Test interface export methods (AC: #2, #3)."""

    def test_export_state_interfaces_count(self):
        """Test export_state_interfaces returns correct count."""
        from manipulator_hardware.modbus_hardware_interface import ModbusHardwareInterface

        hw = ModbusHardwareInterface()
        interfaces = hw.export_state_interfaces()

        # 9 joints × 2 (position + velocity) = 18 interfaces
        assert len(interfaces) == 18

    def test_export_state_interfaces_types(self):
        """Test export_state_interfaces returns position and velocity."""
        from manipulator_hardware.modbus_hardware_interface import ModbusHardwareInterface

        hw = ModbusHardwareInterface()
        interfaces = hw.export_state_interfaces()

        # Count interface types
        position_count = sum(1 for _, t in interfaces if t == 'position')
        velocity_count = sum(1 for _, t in interfaces if t == 'velocity')

        assert position_count == 9
        assert velocity_count == 9

    def test_export_command_interfaces_count(self):
        """Test export_command_interfaces returns correct count."""
        from manipulator_hardware.modbus_hardware_interface import ModbusHardwareInterface

        hw = ModbusHardwareInterface()
        interfaces = hw.export_command_interfaces()

        # 9 joints × 1 (position only) = 9 interfaces
        assert len(interfaces) == 9

    def test_export_command_interfaces_type(self):
        """Test export_command_interfaces returns position only."""
        from manipulator_hardware.modbus_hardware_interface import ModbusHardwareInterface

        hw = ModbusHardwareInterface()
        interfaces = hw.export_command_interfaces()

        # All should be position
        for joint_name, interface_type in interfaces:
            assert interface_type == 'position'


class TestLifecycleTransitions:
    """Test lifecycle state transitions (AC: #1)."""

    def test_on_init_success(self, tmp_path):
        """Test on_init with valid config."""
        from manipulator_hardware.modbus_hardware_interface import (
            ModbusHardwareInterface,
            HardwareInfo,
            CallbackReturn,
        )

        # Create temp config
        config_file = tmp_path / 'hardware_config.yaml'
        config_file.write_text("""
modbus:
  port: "/dev/ttyUSB0"
  baudrate: 115200
joints:
  base_main_frame_joint:
    slave_id: 1
    current_position_register: 1003
    target_position_register: 3005
    pulses_per_meter: 100000
    direction: 1
""")

        hw = ModbusHardwareInterface()
        info = HardwareInfo()
        info.hardware_parameters = {'config_file': str(config_file)}

        result = hw.on_init(info)

        assert result == CallbackReturn.SUCCESS
        assert 'base_main_frame_joint' in hw._joint_configs

    def test_on_init_missing_config(self):
        """Test on_init fails with missing config."""
        from manipulator_hardware.modbus_hardware_interface import (
            ModbusHardwareInterface,
            HardwareInfo,
            CallbackReturn,
        )

        hw = ModbusHardwareInterface()
        info = HardwareInfo()
        info.hardware_parameters = {'config_file': '/nonexistent/path.yaml'}

        result = hw.on_init(info)

        assert result == CallbackReturn.ERROR

    def test_on_configure_connects_driver(self, tmp_path):
        """Test on_configure creates and connects ModbusDriver."""
        from manipulator_hardware.modbus_hardware_interface import (
            ModbusHardwareInterface,
            HardwareInfo,
            CallbackReturn,
        )

        # Create temp config
        config_file = tmp_path / 'hardware_config.yaml'
        config_file.write_text("""
modbus:
  port: "/dev/ttyUSB0"
  baudrate: 115200
joints:
  base_main_frame_joint:
    slave_id: 1
    current_position_register: 1003
    target_position_register: 3005
""")

        hw = ModbusHardwareInterface()
        info = HardwareInfo()
        info.hardware_parameters = {'config_file': str(config_file)}

        hw.on_init(info)

        # Mock the ModbusDriver
        with patch('manipulator_hardware.modbus_hardware_interface.ModbusDriver') as MockDriver:
            mock_driver = MagicMock()
            mock_driver.connect.return_value = True
            mock_driver.is_device_ready.return_value = True
            MockDriver.return_value = mock_driver

            result = hw.on_configure(None)

            assert result == CallbackReturn.SUCCESS
            MockDriver.assert_called_once()
            mock_driver.connect.assert_called_once()

    def test_on_cleanup_disconnects(self, tmp_path):
        """Test on_cleanup disconnects driver."""
        from manipulator_hardware.modbus_hardware_interface import (
            ModbusHardwareInterface,
            HardwareInfo,
            CallbackReturn,
        )

        config_file = tmp_path / 'hardware_config.yaml'
        config_file.write_text("""
modbus:
  port: "/dev/ttyUSB0"
joints:
  test_joint:
    slave_id: 1
    current_position_register: 1003
    target_position_register: 3005
""")

        hw = ModbusHardwareInterface()
        info = HardwareInfo()
        info.hardware_parameters = {'config_file': str(config_file)}
        hw.on_init(info)

        with patch('manipulator_hardware.modbus_hardware_interface.ModbusDriver') as MockDriver:
            mock_driver = MagicMock()
            mock_driver.connect.return_value = True
            mock_driver.is_device_ready.return_value = True
            MockDriver.return_value = mock_driver

            hw.on_configure(None)
            result = hw.on_cleanup(None)

            assert result == CallbackReturn.SUCCESS
            mock_driver.disconnect.assert_called_once()
            assert hw._driver is None


class TestDiscreteAxisThreshold:
    """Test discrete axis binary threshold (AC: #10)."""

    def test_threshold_below_0_5(self):
        """Test command < 0.5 maps to 0."""
        from manipulator_hardware.modbus_hardware_interface import ModbusHardwareInterface

        hw = ModbusHardwareInterface()
        hw._joint_configs = {
            'picker_rail_picker_base_joint': {
                'slave_id': 3,
                'target_position_register': 3015,
                'discrete_axis': True,
            }
        }

        # Mock driver
        mock_driver = MagicMock()
        mock_driver.is_device_busy.return_value = False
        mock_driver.write_position.return_value = True
        hw._driver = mock_driver

        # Test command = 0.49 (should write 0)
        hw._write_discrete_axis(
            'picker_rail_picker_base_joint',
            hw._joint_configs['picker_rail_picker_base_joint'],
            0.49, 3, 3015
        )

        mock_driver.write_position.assert_called_with(3, 3015, 0)

    def test_threshold_at_0_5(self):
        """Test command == 0.5 maps to 1."""
        from manipulator_hardware.modbus_hardware_interface import ModbusHardwareInterface

        hw = ModbusHardwareInterface()
        hw._joint_configs = {
            'picker_rail_picker_base_joint': {
                'slave_id': 3,
                'target_position_register': 3015,
                'discrete_axis': True,
            }
        }

        mock_driver = MagicMock()
        mock_driver.is_device_busy.return_value = False
        mock_driver.write_position.return_value = True
        hw._driver = mock_driver

        # Test command = 0.5 (should write 1)
        hw._write_discrete_axis(
            'picker_rail_picker_base_joint',
            hw._joint_configs['picker_rail_picker_base_joint'],
            0.5, 3, 3015
        )

        mock_driver.write_position.assert_called_with(3, 3015, 1)

    def test_threshold_above_0_5(self):
        """Test command > 0.5 maps to 1."""
        from manipulator_hardware.modbus_hardware_interface import ModbusHardwareInterface

        hw = ModbusHardwareInterface()
        hw._joint_configs = {
            'picker_rail_picker_base_joint': {
                'slave_id': 3,
                'target_position_register': 3015,
                'discrete_axis': True,
            }
        }

        mock_driver = MagicMock()
        mock_driver.is_device_busy.return_value = False
        mock_driver.write_position.return_value = True
        hw._driver = mock_driver

        # Test command = 0.51 (should write 1)
        hw._write_discrete_axis(
            'picker_rail_picker_base_joint',
            hw._joint_configs['picker_rail_picker_base_joint'],
            0.51, 3, 3015
        )

        mock_driver.write_position.assert_called_with(3, 3015, 1)


class TestDiscreteAxisBusyDefer:
    """Test discrete axis defers when busy (AC: #9)."""

    def test_defer_when_busy(self):
        """Test write deferred when slave is busy."""
        from manipulator_hardware.modbus_hardware_interface import ModbusHardwareInterface

        hw = ModbusHardwareInterface()
        hw._joint_configs = {
            'picker_rail_picker_base_joint': {
                'slave_id': 3,
                'target_position_register': 3015,
                'discrete_axis': True,
            }
        }

        mock_driver = MagicMock()
        mock_driver.is_device_busy.return_value = True  # BUSY
        hw._driver = mock_driver

        result = hw._write_discrete_axis(
            'picker_rail_picker_base_joint',
            hw._joint_configs['picker_rail_picker_base_joint'],
            1.0, 3, 3015
        )

        # Should defer (return False) and not write
        assert result is False
        mock_driver.write_position.assert_not_called()

        # Should be queued
        assert 'picker_rail_picker_base_joint' in hw._slave3_pending_commands


class TestDiscreteAxisLimitSkip:
    """Test discrete axis skips write when already at limit (AC: #10)."""

    def test_skip_when_at_min_limit(self):
        """Test skip write when going to MIN and already at MIN."""
        from manipulator_hardware.modbus_hardware_interface import ModbusHardwareInterface

        hw = ModbusHardwareInterface()
        hw._joint_configs = {
            'picker_rail_picker_base_joint': {
                'slave_id': 3,
                'target_position_register': 3015,
                'discrete_axis': True,
                'min_limit_register': 1013,
                'max_limit_register': 1014,
            }
        }

        mock_driver = MagicMock()
        mock_driver.is_device_busy.return_value = False
        # Simulate reading limit switch - at MIN
        mock_driver.read_position.return_value = 1  # At limit
        hw._driver = mock_driver

        result = hw._write_discrete_axis(
            'picker_rail_picker_base_joint',
            hw._joint_configs['picker_rail_picker_base_joint'],
            0.0,  # Going to MIN (command < 0.5)
            3, 3015
        )

        # Should return True (success) but not write
        assert result is True
        mock_driver.write_position.assert_not_called()

    def test_skip_when_at_max_limit(self):
        """Test skip write when going to MAX and already at MAX."""
        from manipulator_hardware.modbus_hardware_interface import ModbusHardwareInterface

        hw = ModbusHardwareInterface()
        hw._joint_configs = {
            'picker_rail_picker_base_joint': {
                'slave_id': 3,
                'target_position_register': 3015,
                'discrete_axis': True,
                'min_limit_register': 1013,
                'max_limit_register': 1014,
            }
        }

        mock_driver = MagicMock()
        mock_driver.is_device_busy.return_value = False

        def read_position_side_effect(slave_id, register):
            if register == 1014:  # MAX limit
                return 1  # At limit
            return 0

        mock_driver.read_position.side_effect = read_position_side_effect
        hw._driver = mock_driver

        result = hw._write_discrete_axis(
            'picker_rail_picker_base_joint',
            hw._joint_configs['picker_rail_picker_base_joint'],
            1.0,  # Going to MAX (command >= 0.5)
            3, 3015
        )

        # Should return True (success) but not write
        assert result is True
        mock_driver.write_position.assert_not_called()


class TestContinuousAxisIncremental:
    """Test continuous axis incremental commands (AC: #12)."""

    def test_incremental_clamp_large_delta(self):
        """Test large delta is clamped to max_velocity * period."""
        from manipulator_hardware.modbus_hardware_interface import ModbusHardwareInterface

        hw = ModbusHardwareInterface()
        hw._joint_configs = {
            'base_main_frame_joint': {
                'slave_id': 1,
                'target_position_register': 3005,
                'pulses_per_meter': 100000,
                'direction': 1,
                'max_velocity': 0.5,  # m/s
                'discrete_axis': False,
            }
        }
        hw._position_states['base_main_frame_joint'] = 0.0  # Current = 0

        mock_driver = MagicMock()
        mock_driver.write_position.return_value = True
        hw._driver = mock_driver

        # Target = 2.0m (large jump), period = 0.1s
        # max_delta = 0.5 * 0.1 = 0.05m
        # incremental_target = 0 + 0.05 = 0.05m = 5000 pulses
        hw._write_continuous_axis(
            'base_main_frame_joint',
            hw._joint_configs['base_main_frame_joint'],
            2.0,  # Target far away
            0.1,  # 100ms period
            1, 3005
        )

        # Should write clamped incremental position
        mock_driver.write_position.assert_called_once()
        args = mock_driver.write_position.call_args[0]
        assert args[0] == 1  # slave_id
        assert args[1] == 3005  # register
        assert args[2] == 5000  # 0.05m in pulses

    def test_incremental_small_delta_not_clamped(self):
        """Test small delta passes through unclamped."""
        from manipulator_hardware.modbus_hardware_interface import ModbusHardwareInterface

        hw = ModbusHardwareInterface()
        hw._joint_configs = {
            'base_main_frame_joint': {
                'slave_id': 1,
                'target_position_register': 3005,
                'pulses_per_meter': 100000,
                'direction': 1,
                'max_velocity': 0.5,
                'discrete_axis': False,
            }
        }
        hw._position_states['base_main_frame_joint'] = 0.0

        mock_driver = MagicMock()
        mock_driver.write_position.return_value = True
        hw._driver = mock_driver

        # Target = 0.01m (small), period = 0.1s
        # max_delta = 0.5 * 0.1 = 0.05m
        # delta = 0.01m < max_delta, so not clamped
        # incremental_target = 0.01m = 1000 pulses
        hw._write_continuous_axis(
            'base_main_frame_joint',
            hw._joint_configs['base_main_frame_joint'],
            0.01,
            0.1,
            1, 3005
        )

        args = mock_driver.write_position.call_args[0]
        assert args[2] == 1000  # 0.01m in pulses

    def test_incremental_negative_direction(self):
        """Test incremental command in negative direction."""
        from manipulator_hardware.modbus_hardware_interface import ModbusHardwareInterface

        hw = ModbusHardwareInterface()
        hw._joint_configs = {
            'base_main_frame_joint': {
                'slave_id': 1,
                'target_position_register': 3005,
                'pulses_per_meter': 100000,
                'direction': 1,
                'max_velocity': 0.5,
                'discrete_axis': False,
            }
        }
        hw._position_states['base_main_frame_joint'] = 1.0  # Current = 1.0m

        mock_driver = MagicMock()
        mock_driver.write_position.return_value = True
        hw._driver = mock_driver

        # Target = 0.0m (negative direction), period = 0.1s
        # delta = 0 - 1 = -1.0m, clamped to -0.05m
        # incremental_target = 1.0 - 0.05 = 0.95m = 95000 pulses
        hw._write_continuous_axis(
            'base_main_frame_joint',
            hw._joint_configs['base_main_frame_joint'],
            0.0,
            0.1,
            1, 3005
        )

        args = mock_driver.write_position.call_args[0]
        assert args[2] == 95000


class TestSlave3Sequential:
    """Test Slave 3 sequential constraint (AC: #11)."""

    def test_only_one_command_per_cycle(self, tmp_path):
        """Test only one Slave 3 axis gets command per cycle."""
        from manipulator_hardware.modbus_hardware_interface import (
            ModbusHardwareInterface,
            HardwareInfo,
        )

        config_file = tmp_path / 'hardware_config.yaml'
        config_file.write_text("""
modbus:
  port: "/dev/ttyUSB0"
joints:
  selector_frame_picker_frame_joint:  # A axis - slave 3
    slave_id: 3
    current_position_register: 1003
    target_position_register: 3005
    pulses_per_meter: 100000
    direction: 1
    max_velocity: 0.3
    discrete_axis: false
  picker_rail_picker_base_joint:  # C axis - slave 3
    slave_id: 3
    current_position_register: 1010
    target_position_register: 3015
    discrete_axis: true
    min_limit_register: 1013
    max_limit_register: 1014
  picker_base_picker_jaw_joint:  # D axis - slave 3
    slave_id: 3
    current_position_register: 1017
    target_position_register: 3025
    discrete_axis: true
    min_limit_register: 1020
    max_limit_register: 1021
""")

        hw = ModbusHardwareInterface()
        info = HardwareInfo()
        info.hardware_parameters = {'config_file': str(config_file)}
        hw.on_init(info)

        # Set up different commands for all three Slave 3 joints
        hw._position_commands['selector_frame_picker_frame_joint'] = 0.5
        hw._position_commands['picker_rail_picker_base_joint'] = 1.0
        hw._position_commands['picker_base_picker_jaw_joint'] = 1.0

        # Initialize states
        hw._position_states['selector_frame_picker_frame_joint'] = 0.0
        hw._position_states['picker_rail_picker_base_joint'] = 0.0
        hw._position_states['picker_base_picker_jaw_joint'] = 0.0

        mock_driver = MagicMock()
        mock_driver.write_position.return_value = True
        mock_driver.is_device_busy.return_value = False
        mock_driver.read_position.return_value = 0  # Not at limit
        hw._driver = mock_driver

        # Create mock period
        class MockPeriod:
            nanoseconds = 100_000_000  # 100ms

        hw.write(None, MockPeriod())

        # Only ONE write should have been made to slave 3
        slave3_writes = [
            call for call in mock_driver.write_position.call_args_list
            if call[0][0] == 3
        ]
        assert len(slave3_writes) == 1


class TestReadCycle:
    """Test read() cycle (AC: #4)."""

    def test_read_updates_positions(self, tmp_path):
        """Test read() updates position states from hardware."""
        from manipulator_hardware.modbus_hardware_interface import (
            ModbusHardwareInterface,
            HardwareInfo,
        )

        config_file = tmp_path / 'hardware_config.yaml'
        config_file.write_text("""
modbus:
  port: "/dev/ttyUSB0"
joints:
  base_main_frame_joint:
    slave_id: 1
    current_position_register: 1003
    target_position_register: 3005
    pulses_per_meter: 100000
    direction: 1
""")

        hw = ModbusHardwareInterface()
        info = HardwareInfo()
        info.hardware_parameters = {'config_file': str(config_file)}
        hw.on_init(info)

        mock_driver = MagicMock()
        mock_driver.read_position.return_value = 50000  # 0.5m
        hw._driver = mock_driver

        class MockPeriod:
            nanoseconds = 100_000_000

        hw.read(None, MockPeriod())

        assert abs(hw._position_states['base_main_frame_joint'] - 0.5) < 1e-6

    def test_read_estimates_velocity(self, tmp_path):
        """Test read() estimates velocity from position delta."""
        from manipulator_hardware.modbus_hardware_interface import (
            ModbusHardwareInterface,
            HardwareInfo,
        )

        config_file = tmp_path / 'hardware_config.yaml'
        config_file.write_text("""
modbus:
  port: "/dev/ttyUSB0"
joints:
  base_main_frame_joint:
    slave_id: 1
    current_position_register: 1003
    target_position_register: 3005
    pulses_per_meter: 100000
    direction: 1
""")

        hw = ModbusHardwareInterface()
        info = HardwareInfo()
        info.hardware_parameters = {'config_file': str(config_file)}
        hw.on_init(info)

        mock_driver = MagicMock()
        hw._driver = mock_driver

        class MockPeriod:
            nanoseconds = 100_000_000  # 0.1s

        # First read: 0.0m
        hw._prev_positions['base_main_frame_joint'] = 0.0
        mock_driver.read_position.return_value = 10000  # 0.1m
        hw.read(None, MockPeriod())

        # velocity = (0.1 - 0.0) / 0.1 = 1.0 m/s
        assert abs(hw._velocity_states['base_main_frame_joint'] - 1.0) < 1e-6

    def test_read_mock_joints_echo_command(self, tmp_path):
        """Test mock joints return command as state."""
        from manipulator_hardware.modbus_hardware_interface import (
            ModbusHardwareInterface,
            HardwareInfo,
        )

        config_file = tmp_path / 'hardware_config.yaml'
        config_file.write_text("""
modbus:
  port: "/dev/ttyUSB0"
joints: {}
""")

        hw = ModbusHardwareInterface()
        info = HardwareInfo()
        info.hardware_parameters = {'config_file': str(config_file)}
        hw.on_init(info)

        mock_driver = MagicMock()
        hw._driver = mock_driver

        # Set command for mock joint
        hw._position_commands['selector_left_container_jaw_joint'] = 0.123

        class MockPeriod:
            nanoseconds = 100_000_000

        hw.read(None, MockPeriod())

        # Mock joint state should equal command
        assert hw._position_states['selector_left_container_jaw_joint'] == 0.123


class TestStateCommandAccess:
    """Test get_state and set_command methods."""

    def test_get_state_position(self):
        """Test get_state returns position."""
        from manipulator_hardware.modbus_hardware_interface import ModbusHardwareInterface

        hw = ModbusHardwareInterface()
        hw._position_states['base_main_frame_joint'] = 1.234

        result = hw.get_state('base_main_frame_joint', 'position')
        assert result == 1.234

    def test_get_state_velocity(self):
        """Test get_state returns velocity."""
        from manipulator_hardware.modbus_hardware_interface import ModbusHardwareInterface

        hw = ModbusHardwareInterface()
        hw._velocity_states['base_main_frame_joint'] = 0.5

        result = hw.get_state('base_main_frame_joint', 'velocity')
        assert result == 0.5

    def test_set_command_valid(self):
        """Test set_command sets position command."""
        from manipulator_hardware.modbus_hardware_interface import ModbusHardwareInterface

        hw = ModbusHardwareInterface()

        result = hw.set_command('base_main_frame_joint', 'position', 2.0)

        assert result is True
        assert hw._position_commands['base_main_frame_joint'] == 2.0

    def test_set_command_invalid_type(self):
        """Test set_command rejects non-position type."""
        from manipulator_hardware.modbus_hardware_interface import ModbusHardwareInterface

        hw = ModbusHardwareInterface()

        result = hw.set_command('base_main_frame_joint', 'velocity', 1.0)

        assert result is False
