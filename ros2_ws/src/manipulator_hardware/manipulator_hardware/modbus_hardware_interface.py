"""ModbusHardwareInterface - ros2_control SystemInterface plugin for Modbus RTU hardware."""

import os
from typing import Optional

import yaml

try:
    import rclpy.logging
    ROS_LOGGING_AVAILABLE = True
except ImportError:
    ROS_LOGGING_AVAILABLE = False

from .modbus_driver import ModbusDriver


class CallbackReturn:
    """Lifecycle callback return values."""

    SUCCESS = 0
    FAILURE = 1
    ERROR = 2


class HardwareInfo:
    """Hardware info parsed from URDF/YAML."""

    def __init__(self):
        self.name: str = ''
        self.hardware_parameters: dict = {}


class ModbusHardwareInterface:
    """ros2_control SystemInterface plugin for Modbus RTU motor controllers.

    Implements lifecycle callbacks and read/write cycle for 9 joints:
    - 5 continuous axes (X, Z, Y, A, B) - incremental servo positioning
    - 2 discrete axes (C, D) - binary limit switch control
    - 2 mock axes (container jaws) - echo command as state
    """

    # Mock joints that don't use hardware
    MOCK_JOINTS = [
        'selector_left_container_jaw_joint',
        'selector_right_container_jaw_joint',
    ]

    # All joints in order (Modbus + mock)
    ALL_JOINTS = [
        'base_main_frame_joint',
        'main_frame_selector_frame_joint',
        'selector_frame_gripper_joint',
        'selector_frame_picker_frame_joint',
        'picker_frame_picker_rail_joint',
        'picker_rail_picker_base_joint',
        'picker_base_picker_jaw_joint',
        'selector_left_container_jaw_joint',
        'selector_right_container_jaw_joint',
    ]

    def __init__(self):
        """Initialize storage for state and command interfaces."""
        # Logger
        if ROS_LOGGING_AVAILABLE:
            self._logger = rclpy.logging.get_logger('modbus_hardware_interface')
        else:
            self._logger = None

        # Hardware driver (created in on_configure)
        self._driver: Optional[ModbusDriver] = None

        # Configuration from YAML
        self._config: dict = {}
        self._joint_configs: dict = {}

        # State storage (position and velocity per joint)
        self._position_states: dict[str, float] = {}
        self._velocity_states: dict[str, float] = {}
        self._prev_positions: dict[str, float] = {}

        # Command storage
        self._position_commands: dict[str, float] = {}
        self._prev_commands: dict[str, float] = {}

        # Slave 3 command queue for sequential processing
        self._slave3_pending_commands: list[str] = []

        # Initialize storage for all joints
        for joint in self.ALL_JOINTS:
            self._position_states[joint] = 0.0
            self._velocity_states[joint] = 0.0
            self._prev_positions[joint] = 0.0
            self._position_commands[joint] = 0.0
            self._prev_commands[joint] = 0.0

    def _log_debug(self, msg: str) -> None:
        """Log debug message."""
        if self._logger:
            self._logger.debug(msg)

    def _log_info(self, msg: str) -> None:
        """Log info message."""
        if self._logger:
            self._logger.info(msg)

    def _log_warn(self, msg: str) -> None:
        """Log warning message."""
        if self._logger:
            self._logger.warning(msg)

    def _log_error(self, msg: str) -> None:
        """Log error message."""
        if self._logger:
            self._logger.error(msg)

    # =========================================================================
    # Unit Conversion Helpers (AC: #4, #5)
    # =========================================================================

    def _pulses_to_meters(self, pulses: int, joint_config: dict) -> float:
        """Convert hardware pulses to ROS2 SI meters.

        Args:
            pulses: Raw pulse count from hardware
            joint_config: Joint configuration with pulses_per_meter and direction

        Returns:
            Position in meters
        """
        pulses_per_meter = joint_config.get('pulses_per_meter', 100000)
        direction = joint_config.get('direction', 1)
        return (pulses / pulses_per_meter) * direction

    def _meters_to_pulses(self, meters: float, joint_config: dict) -> int:
        """Convert ROS2 SI meters to hardware pulses.

        Args:
            meters: Position in meters
            joint_config: Joint configuration with pulses_per_meter and direction

        Returns:
            Pulse count for hardware
        """
        pulses_per_meter = joint_config.get('pulses_per_meter', 100000)
        direction = joint_config.get('direction', 1)
        return int(meters * pulses_per_meter * direction)

    # =========================================================================
    # Helper Methods
    # =========================================================================

    def _is_slave_busy(self, slave_id: int) -> bool:
        """Check if slave device is busy executing a command.

        Args:
            slave_id: Modbus slave ID

        Returns:
            True if busy, False if idle or on error (fail-safe)
        """
        if self._driver is None:
            return True  # Assume busy if no driver

        result = self._driver.is_device_busy(slave_id)
        if result is None:
            self._log_warn(f'Failed to read busy status for slave {slave_id}')
            return True  # Assume busy on error (fail-safe)
        return result

    def _read_limit_switch(self, slave_id: int, register: int) -> Optional[bool]:
        """Read limit switch state from hardware.

        Args:
            slave_id: Modbus slave ID
            register: Limit switch register address

        Returns:
            True if at limit, False if not, None on error
        """
        if self._driver is None:
            return None

        value = self._driver.read_position(slave_id, register)
        if value is None:
            return None
        return bool(value)

    def _is_modbus_joint(self, joint_name: str) -> bool:
        """Check if joint uses Modbus hardware (not mock)."""
        return joint_name not in self.MOCK_JOINTS

    def _is_discrete_axis(self, joint_name: str) -> bool:
        """Check if joint is a discrete axis (C or D)."""
        if joint_name not in self._joint_configs:
            return False
        return self._joint_configs[joint_name].get('discrete_axis', False)

    def _is_slave3_joint(self, joint_name: str) -> bool:
        """Check if joint belongs to Slave 3 (A, C, D axes)."""
        if joint_name not in self._joint_configs:
            return False
        return self._joint_configs[joint_name].get('slave_id') == 3

    # =========================================================================
    # Lifecycle Callbacks (AC: #1, #6)
    # =========================================================================

    def on_init(self, hardware_info: HardwareInfo) -> int:
        """Initialize hardware interface from HardwareInfo.

        Parses URDF parameters and loads YAML configuration.

        Args:
            hardware_info: Hardware configuration from URDF

        Returns:
            CallbackReturn.SUCCESS or CallbackReturn.ERROR
        """
        self._log_info('on_init: Initializing ModbusHardwareInterface')

        # Get config file path from hardware parameters
        config_file = hardware_info.hardware_parameters.get('config_file', '')
        if not config_file:
            # Default path
            config_file = os.path.join(
                os.path.dirname(__file__),
                '..', 'config', 'hardware_config.yaml'
            )

        # Load YAML configuration
        try:
            with open(config_file, 'r') as f:
                self._config = yaml.safe_load(f)
            self._log_info(f'Loaded config from {config_file}')
        except Exception as e:
            self._log_error(f'Failed to load config: {e}')
            return CallbackReturn.ERROR

        # Extract joint configurations
        self._joint_configs = self._config.get('joints', {})
        if not self._joint_configs:
            self._log_error('No joints configured in hardware_config.yaml')
            return CallbackReturn.ERROR

        self._log_info(f'Configured {len(self._joint_configs)} Modbus joints + '
                       f'{len(self.MOCK_JOINTS)} mock joints')

        return CallbackReturn.SUCCESS

    def on_configure(self, previous_state) -> int:
        """Configure hardware - connect to Modbus devices.

        Args:
            previous_state: Previous lifecycle state (unused)

        Returns:
            CallbackReturn.SUCCESS or CallbackReturn.ERROR
        """
        self._log_info('on_configure: Connecting to Modbus devices')

        # Get Modbus configuration
        modbus_config = self._config.get('modbus', {})
        port = modbus_config.get('port', '/dev/ttyUSB0')
        baudrate = modbus_config.get('baudrate', 115200)
        timeout_sec = modbus_config.get('timeout_sec', 0.5)
        retry_count = modbus_config.get('retry_count', 3)

        # Create and connect driver
        self._driver = ModbusDriver(
            port=port,
            baudrate=baudrate,
            timeout_sec=timeout_sec,
            retry_count=retry_count
        )

        if not self._driver.connect():
            self._log_error(f'Failed to connect to {port}')
            return CallbackReturn.ERROR

        self._log_info(f'Connected to {port} at {baudrate} baud')

        # Verify devices are ready
        for slave_id in [1, 2, 3]:
            ready = self._driver.is_device_ready(slave_id)
            if ready is None:
                self._log_warn(f'Could not verify slave {slave_id} ready status')
            elif not ready:
                self._log_warn(f'Slave {slave_id} not ready')
            else:
                self._log_info(f'Slave {slave_id} ready')

        return CallbackReturn.SUCCESS

    def on_activate(self, previous_state) -> int:
        """Activate hardware - sync commands to current positions.

        Args:
            previous_state: Previous lifecycle state (unused)

        Returns:
            CallbackReturn.SUCCESS or CallbackReturn.ERROR
        """
        self._log_info('on_activate: Syncing commands to current positions')

        # Read initial positions for all Modbus joints
        for joint_name, joint_config in self._joint_configs.items():
            slave_id = joint_config.get('slave_id')
            pos_reg = joint_config.get('current_position_register')

            if slave_id is None or pos_reg is None:
                continue

            pulses = self._driver.read_position(slave_id, pos_reg)
            if pulses is not None:
                meters = self._pulses_to_meters(pulses, joint_config)
                self._position_states[joint_name] = meters
                self._prev_positions[joint_name] = meters
                self._position_commands[joint_name] = meters
                self._prev_commands[joint_name] = meters
                self._log_debug(f'{joint_name}: {meters:.4f}m ({pulses} pulses)')
            else:
                self._log_warn(f'Failed to read initial position for {joint_name}')

        # Initialize mock joints to 0
        for joint_name in self.MOCK_JOINTS:
            self._position_states[joint_name] = 0.0
            self._position_commands[joint_name] = 0.0

        self._log_info('on_activate: Complete')
        return CallbackReturn.SUCCESS

    def on_deactivate(self, previous_state) -> int:
        """Deactivate hardware - hold position.

        Args:
            previous_state: Previous lifecycle state (unused)

        Returns:
            CallbackReturn.SUCCESS
        """
        self._log_info('on_deactivate: Holding position')
        # Commands will not be written while deactivated
        return CallbackReturn.SUCCESS

    def on_cleanup(self, previous_state) -> int:
        """Cleanup hardware - disconnect Modbus.

        Args:
            previous_state: Previous lifecycle state (unused)

        Returns:
            CallbackReturn.SUCCESS
        """
        self._log_info('on_cleanup: Disconnecting Modbus')

        if self._driver is not None:
            self._driver.disconnect()
            self._driver = None

        return CallbackReturn.SUCCESS

    def on_error(self, previous_state) -> int:
        """Handle error state - graceful shutdown.

        Args:
            previous_state: Previous lifecycle state (unused)

        Returns:
            CallbackReturn.SUCCESS
        """
        self._log_error('on_error: Graceful shutdown')

        if self._driver is not None:
            self._driver.disconnect()
            self._driver = None

        return CallbackReturn.SUCCESS

    # =========================================================================
    # Interface Exports (AC: #2, #3)
    # =========================================================================

    def export_state_interfaces(self) -> list:
        """Export state interfaces for controller_manager.

        Returns:
            List of (joint_name, interface_type) tuples:
            - 9 position interfaces (7 Modbus + 2 mock)
            - 9 velocity interfaces (7 Modbus + 2 mock)
        """
        interfaces = []

        for joint_name in self.ALL_JOINTS:
            interfaces.append((joint_name, 'position'))
            interfaces.append((joint_name, 'velocity'))

        self._log_info(f'Exported {len(interfaces)} state interfaces')
        return interfaces

    def export_command_interfaces(self) -> list:
        """Export command interfaces for controller_manager.

        Returns:
            List of (joint_name, interface_type) tuples:
            - 9 position command interfaces (7 Modbus + 2 mock)
        """
        interfaces = []

        for joint_name in self.ALL_JOINTS:
            interfaces.append((joint_name, 'position'))

        self._log_info(f'Exported {len(interfaces)} command interfaces')
        return interfaces

    # =========================================================================
    # State/Command Access Methods
    # =========================================================================

    def get_state(self, joint_name: str, interface_type: str) -> Optional[float]:
        """Get state value for a joint interface.

        Args:
            joint_name: Name of the joint
            interface_type: 'position' or 'velocity'

        Returns:
            State value or None if not found
        """
        if interface_type == 'position':
            return self._position_states.get(joint_name)
        elif interface_type == 'velocity':
            return self._velocity_states.get(joint_name)
        return None

    def set_command(self, joint_name: str, interface_type: str, value: float) -> bool:
        """Set command value for a joint interface.

        Args:
            joint_name: Name of the joint
            interface_type: 'position' only
            value: Command value in meters

        Returns:
            True if set, False if invalid
        """
        if interface_type != 'position':
            return False
        if joint_name not in self.ALL_JOINTS:
            return False

        self._position_commands[joint_name] = value
        return True

    # =========================================================================
    # Control Loop (AC: #4, #5, #9, #10, #11, #12)
    # =========================================================================

    def read(self, time, period) -> int:
        """Read state from hardware.

        Called at 10Hz. Reads positions from all Modbus joints,
        converts pulses to meters, estimates velocity.

        Args:
            time: Current ROS time (unused)
            period: Time since last read (Duration with nanoseconds attr)

        Returns:
            0 for OK, non-zero for error
        """
        # Calculate period in seconds
        if hasattr(period, 'nanoseconds'):
            period_sec = period.nanoseconds / 1e9
        else:
            period_sec = 0.1  # Default 10Hz

        # Read Modbus joints
        for joint_name, joint_config in self._joint_configs.items():
            slave_id = joint_config.get('slave_id')
            pos_reg = joint_config.get('current_position_register')

            if slave_id is None or pos_reg is None:
                continue

            pulses = self._driver.read_position(slave_id, pos_reg)
            if pulses is not None:
                meters = self._pulses_to_meters(pulses, joint_config)

                # Estimate velocity from position delta
                prev_pos = self._prev_positions.get(joint_name, meters)
                velocity = (meters - prev_pos) / period_sec if period_sec > 0 else 0.0

                self._position_states[joint_name] = meters
                self._velocity_states[joint_name] = velocity
                self._prev_positions[joint_name] = meters
            else:
                self._log_warn(f'Failed to read position for {joint_name}')

        # Mock joints: state = command
        for joint_name in self.MOCK_JOINTS:
            self._position_states[joint_name] = self._position_commands[joint_name]
            self._velocity_states[joint_name] = 0.0

        return 0

    def write(self, time, period) -> int:
        """Write commands to hardware.

        Called at 10Hz. Handles three joint types:
        - Continuous axes: Incremental commands clamped to max_velocity
        - Discrete axes: Binary 0/1 with busy/limit checks
        - Mock joints: No hardware write

        Args:
            time: Current ROS time (unused)
            period: Time since last write (Duration with nanoseconds attr)

        Returns:
            0 for OK, non-zero for error
        """
        if self._driver is None:
            return 1

        # Calculate period in seconds
        if hasattr(period, 'nanoseconds'):
            period_sec = period.nanoseconds / 1e9
        else:
            period_sec = 0.1  # Default 10Hz

        # Track if we've written to Slave 3 this cycle (sequential constraint)
        slave3_written_this_cycle = False

        for joint_name, joint_config in self._joint_configs.items():
            # Skip if command unchanged
            current_cmd = self._position_commands.get(joint_name, 0.0)
            prev_cmd = self._prev_commands.get(joint_name, 0.0)

            # For continuous axes, also check if we've reached target
            if not self._is_discrete_axis(joint_name):
                current_pos = self._position_states.get(joint_name, 0.0)
                if abs(current_cmd - prev_cmd) < 1e-6 and abs(current_cmd - current_pos) < 1e-4:
                    continue

            slave_id = joint_config.get('slave_id')
            cmd_reg = joint_config.get('target_position_register')

            if slave_id is None or cmd_reg is None:
                continue

            # Slave 3 sequential constraint (AC: #11)
            if slave_id == 3:
                if slave3_written_this_cycle:
                    # Queue for next cycle
                    if joint_name not in self._slave3_pending_commands:
                        self._slave3_pending_commands.append(joint_name)
                    continue

            # Handle discrete axes (AC: #9, #10)
            if self._is_discrete_axis(joint_name):
                success = self._write_discrete_axis(
                    joint_name, joint_config, current_cmd, slave_id, cmd_reg
                )
                if success and slave_id == 3:
                    slave3_written_this_cycle = True
            else:
                # Handle continuous axes (AC: #12)
                success = self._write_continuous_axis(
                    joint_name, joint_config, current_cmd, period_sec, slave_id, cmd_reg
                )
                if success and slave_id == 3:
                    slave3_written_this_cycle = True

            if success:
                self._prev_commands[joint_name] = current_cmd

        # Process one pending Slave 3 command if we haven't written this cycle
        if not slave3_written_this_cycle and self._slave3_pending_commands:
            joint_name = self._slave3_pending_commands.pop(0)
            joint_config = self._joint_configs.get(joint_name)
            if joint_config:
                current_cmd = self._position_commands.get(joint_name, 0.0)
                slave_id = joint_config.get('slave_id')
                cmd_reg = joint_config.get('target_position_register')

                if self._is_discrete_axis(joint_name):
                    self._write_discrete_axis(
                        joint_name, joint_config, current_cmd, slave_id, cmd_reg
                    )
                else:
                    self._write_continuous_axis(
                        joint_name, joint_config, current_cmd, period_sec, slave_id, cmd_reg
                    )

        return 0

    def _write_continuous_axis(
        self,
        joint_name: str,
        joint_config: dict,
        target_pos: float,
        period_sec: float,
        slave_id: int,
        cmd_reg: int
    ) -> bool:
        """Write incremental command to continuous axis.

        Clamps delta to max_velocity * period to satisfy firmware constraint.

        Args:
            joint_name: Joint name
            joint_config: Joint configuration
            target_pos: Target position in meters
            period_sec: Period in seconds
            slave_id: Modbus slave ID
            cmd_reg: Command register address

        Returns:
            True if write successful
        """
        current_pos = self._position_states.get(joint_name, 0.0)
        max_velocity = joint_config.get('max_velocity', 0.5)

        # Calculate max delta for this cycle
        max_delta = max_velocity * period_sec

        # Calculate and clamp delta
        delta = target_pos - current_pos
        if abs(delta) > max_delta:
            delta = max_delta if delta > 0 else -max_delta

        # Calculate incremental target
        incremental_target = current_pos + delta

        # Convert to pulses
        pulses = self._meters_to_pulses(incremental_target, joint_config)

        # Clamp to valid range (32-bit signed, Modbus uses two registers for 32-bit)
        # Note: For positions > 65535 pulses (0.65m), firmware uses 32-bit registers
        pulses = max(0, min(2147483647, pulses))

        # Write to hardware
        success = self._driver.write_position(slave_id, cmd_reg, pulses)

        if success:
            self._log_debug(
                f'{joint_name}: {incremental_target:.4f}m ({pulses} pulses) '
                f'[delta={delta:.4f}, max={max_delta:.4f}]'
            )
        else:
            self._log_warn(f'Failed to write {joint_name}')

        return success

    def _write_discrete_axis(
        self,
        joint_name: str,
        joint_config: dict,
        command: float,
        slave_id: int,
        cmd_reg: int
    ) -> bool:
        """Write binary command to discrete axis.

        Thresholds command to 0/1, checks busy status and limit switches.

        Args:
            joint_name: Joint name
            joint_config: Joint configuration
            command: Command value (thresholded at 0.5)
            slave_id: Modbus slave ID
            cmd_reg: Command register address

        Returns:
            True if write successful or deferred
        """
        # Check if slave is busy (AC: #9)
        if self._is_slave_busy(slave_id):
            self._log_debug(f'{joint_name}: Slave {slave_id} busy, deferring')
            # Queue for next cycle
            if joint_name not in self._slave3_pending_commands:
                self._slave3_pending_commands.append(joint_name)
            return False

        # Threshold to binary (AC: #10)
        binary_cmd = 1 if command >= 0.5 else 0

        # Check limit switches (AC: #10)
        min_limit_reg = joint_config.get('min_limit_register')
        max_limit_reg = joint_config.get('max_limit_register')

        if binary_cmd == 0 and min_limit_reg:
            # Going to MIN - check if already there
            at_min = self._read_limit_switch(slave_id, min_limit_reg)
            if at_min:
                self._log_debug(f'{joint_name}: Already at MIN limit, skipping')
                return True

        if binary_cmd == 1 and max_limit_reg:
            # Going to MAX - check if already there
            at_max = self._read_limit_switch(slave_id, max_limit_reg)
            if at_max:
                self._log_debug(f'{joint_name}: Already at MAX limit, skipping')
                return True

        # Write binary command
        success = self._driver.write_position(slave_id, cmd_reg, binary_cmd)

        if success:
            self._log_debug(f'{joint_name}: {binary_cmd} ({"MAX" if binary_cmd else "MIN"})')
        else:
            self._log_warn(f'Failed to write {joint_name}')

        return success
