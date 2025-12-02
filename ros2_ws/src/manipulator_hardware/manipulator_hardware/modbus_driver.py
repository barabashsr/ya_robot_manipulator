"""Modbus RTU communication driver for motor controllers."""

import time
from typing import Optional

try:
    import minimalmodbus
    import serial
    MODBUS_AVAILABLE = True
except ImportError:
    MODBUS_AVAILABLE = False
    minimalmodbus = None
    serial = None

try:
    import rclpy.logging
    ROS_LOGGING_AVAILABLE = True
except ImportError:
    ROS_LOGGING_AVAILABLE = False


class ModbusDriver:
    """Modbus RTU communication driver for motor controllers."""

    # Status register addresses (Input Registers FC4)
    REG_ERR_CODE = 999
    REG_MODULE_READY = 1001      # 1 = ready for commands
    REG_MODULE_IS_BUSY = 1002    # 1 = busy executing

    def __init__(
        self,
        port: str = '/dev/ttyACM0',
        baudrate: int = 115200,
        timeout_sec: float = 0.5,
        retry_count: int = 3
    ):
        """Initialize with port, baudrate, timeout_sec and retry_count."""
        self._port = port
        self._baudrate = baudrate
        self._timeout_sec = timeout_sec
        self._retry_count = retry_count
        self._connected = False
        self._instruments: dict[int, 'minimalmodbus.Instrument'] = {}

        # Set up logger
        if ROS_LOGGING_AVAILABLE:
            self._logger = rclpy.logging.get_logger('modbus_driver')
        else:
            self._logger = None

    def _log_debug(self, msg: str) -> None:
        """Log debug message."""
        if self._logger:
            self._logger.debug(msg)

    def _log_warn(self, msg: str) -> None:
        """Log warning message."""
        if self._logger:
            self._logger.warning(msg)

    def _log_error(self, msg: str) -> None:
        """Log error message."""
        if self._logger:
            self._logger.error(msg)

    def _get_instrument(self, slave_id: int) -> Optional['minimalmodbus.Instrument']:
        """Get or create minimalmodbus Instrument for slave_id."""
        if not self._connected:
            self._log_warn(f'Cannot get instrument for slave {slave_id}: not connected')
            return None

        if slave_id not in self._instruments:
            try:
                instrument = minimalmodbus.Instrument(self._port, slave_id)
                instrument.serial.baudrate = self._baudrate
                instrument.serial.bytesize = 8
                instrument.serial.parity = serial.PARITY_NONE
                instrument.serial.stopbits = 1
                instrument.serial.timeout = self._timeout_sec
                instrument.mode = minimalmodbus.MODE_RTU
                instrument.clear_buffers_before_each_transaction = True
                self._instruments[slave_id] = instrument
                self._log_debug(f'Created instrument for slave {slave_id}')
            except Exception as e:
                self._log_error(f'Failed to create instrument for slave {slave_id}: {e}')
                return None

        return self._instruments[slave_id]

    def connect(self) -> bool:
        """Establish serial connection. Returns True if successful."""
        if not MODBUS_AVAILABLE:
            self._log_error('minimalmodbus library not available')
            return False

        try:
            # Test connection by creating an instrument
            test_instrument = minimalmodbus.Instrument(self._port, 1)
            test_instrument.serial.baudrate = self._baudrate
            test_instrument.serial.bytesize = 8
            test_instrument.serial.parity = serial.PARITY_NONE
            test_instrument.serial.stopbits = 1
            test_instrument.serial.timeout = self._timeout_sec
            test_instrument.mode = minimalmodbus.MODE_RTU

            # Try to open the port
            if not test_instrument.serial.is_open:
                test_instrument.serial.open()

            self._connected = True
            self._instruments = {1: test_instrument}
            self._log_debug(f'Connected to {self._port} at {self._baudrate} baud')
            return True

        except Exception as e:
            self._log_error(f'Failed to connect to {self._port}: {e}')
            self._connected = False
            return False

    def disconnect(self) -> None:
        """Close serial connection."""
        for slave_id, instrument in self._instruments.items():
            try:
                if instrument.serial.is_open:
                    instrument.serial.close()
                    self._log_debug(f'Closed connection for slave {slave_id}')
            except Exception as e:
                self._log_warn(f'Error closing connection for slave {slave_id}: {e}')

        self._instruments.clear()
        self._connected = False
        self._log_debug('Disconnected')

    def _read_register_with_retry(
        self,
        slave_id: int,
        register: int,
        functioncode: int
    ) -> Optional[int]:
        """Read register with retry logic. Returns value or None on failure."""
        instrument = self._get_instrument(slave_id)
        if instrument is None:
            return None

        for attempt in range(self._retry_count):
            try:
                value = instrument.read_register(
                    register,
                    number_of_decimals=0,
                    functioncode=functioncode,
                    signed=False
                )
                self._log_debug(
                    f'Read slave={slave_id} reg={register} fc={functioncode}: {value}'
                )
                return value

            except Exception as e:
                self._log_warn(
                    f'Read failed (attempt {attempt + 1}/{self._retry_count}): {e}'
                )
                if attempt < self._retry_count - 1:
                    time.sleep(0.1)  # Brief delay before retry

        self._log_error(
            f'Read failed after {self._retry_count} attempts: '
            f'slave={slave_id} reg={register}'
        )
        return None

    def _write_register_with_retry(
        self,
        slave_id: int,
        register: int,
        value: int
    ) -> bool:
        """Write holding register with retry logic. Returns True if successful."""
        instrument = self._get_instrument(slave_id)
        if instrument is None:
            return False

        for attempt in range(self._retry_count):
            try:
                instrument.write_register(
                    register,
                    value,
                    number_of_decimals=0,
                    functioncode=6,
                    signed=False
                )
                self._log_debug(
                    f'Write slave={slave_id} reg={register}: {value}'
                )
                return True

            except Exception as e:
                self._log_warn(
                    f'Write failed (attempt {attempt + 1}/{self._retry_count}): {e}'
                )
                if attempt < self._retry_count - 1:
                    time.sleep(0.1)  # Brief delay before retry

        self._log_error(
            f'Write failed after {self._retry_count} attempts: '
            f'slave={slave_id} reg={register} value={value}'
        )
        return False

    def read_position(self, slave_id: int, register: int) -> Optional[int]:
        """Read position from input register FC4. Returns pulses or None."""
        return self._read_register_with_retry(slave_id, register, functioncode=4)

    def write_position(self, slave_id: int, register: int, value: int) -> bool:
        """Write position to holding register FC6. Returns True if successful."""
        return self._write_register_with_retry(slave_id, register, value)

    def read_error_code(self, slave_id: int) -> Optional[int]:
        """Read error code register 999. Returns 0 if OK, None on failure."""
        return self._read_register_with_retry(
            slave_id, self.REG_ERR_CODE, functioncode=4
        )

    def is_device_ready(self, slave_id: int) -> Optional[bool]:
        """Check module_ready register 1001. Returns True if ready, None on failure."""
        value = self._read_register_with_retry(
            slave_id, self.REG_MODULE_READY, functioncode=4
        )
        if value is None:
            return None
        return bool(value)

    def is_device_busy(self, slave_id: int) -> Optional[bool]:
        """Check module_is_busy register 1002. Returns True if busy, None on failure."""
        value = self._read_register_with_retry(
            slave_id, self.REG_MODULE_IS_BUSY, functioncode=4
        )
        if value is None:
            return None
        return bool(value)

    @property
    def is_connected(self) -> bool:
        """Return connection status."""
        return self._connected

    @property
    def port(self) -> str:
        """Return serial port path."""
        return self._port

    @property
    def baudrate(self) -> int:
        """Return serial baudrate."""
        return self._baudrate
