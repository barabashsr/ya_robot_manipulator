"""Unit tests for ModbusDriver class."""

from unittest.mock import MagicMock


class TestModbusDriverImport:
    """Test importing ModbusDriver with and without dependencies."""

    def test_import_modbus_driver(self):
        """Test that ModbusDriver can be imported."""
        from manipulator_hardware.modbus_driver import ModbusDriver
        assert ModbusDriver is not None


class TestModbusDriverInit:
    """Test ModbusDriver initialization."""

    def test_init_default_values(self):
        """Test initialization with default values."""
        from manipulator_hardware.modbus_driver import ModbusDriver
        driver = ModbusDriver()
        assert driver.port == '/dev/ttyACM0'
        assert driver.baudrate == 115200
        assert driver._timeout_sec == 0.5
        assert driver._retry_count == 3
        assert driver.is_connected is False

    def test_init_custom_values(self):
        """Test initialization with custom values."""
        from manipulator_hardware.modbus_driver import ModbusDriver
        driver = ModbusDriver(
            port='/dev/ttyUSB0',
            baudrate=9600,
            timeout_sec=1.0,
            retry_count=5
        )
        assert driver.port == '/dev/ttyUSB0'
        assert driver.baudrate == 9600
        assert driver._timeout_sec == 1.0
        assert driver._retry_count == 5


class TestModbusDriverConnect:
    """Test connection handling."""

    def test_connect_success(self):
        """Test successful connection."""
        # Import the actual minimalmodbus to create proper mock
        import manipulator_hardware.modbus_driver as driver_module

        # Store original values
        orig_modbus_available = driver_module.MODBUS_AVAILABLE
        orig_minimalmodbus = driver_module.minimalmodbus
        orig_serial = driver_module.serial

        try:
            # Create mock objects
            mock_instrument = MagicMock()
            mock_instrument.serial.is_open = False

            mock_minimalmodbus = MagicMock()
            mock_minimalmodbus.Instrument.return_value = mock_instrument
            mock_minimalmodbus.MODE_RTU = 'rtu'

            mock_serial = MagicMock()
            mock_serial.PARITY_NONE = 'N'

            # Patch module-level variables
            driver_module.MODBUS_AVAILABLE = True
            driver_module.minimalmodbus = mock_minimalmodbus
            driver_module.serial = mock_serial

            driver = driver_module.ModbusDriver()
            result = driver.connect()

            assert result is True
            assert driver.is_connected is True
            mock_instrument.serial.open.assert_called_once()

        finally:
            # Restore original values
            driver_module.MODBUS_AVAILABLE = orig_modbus_available
            driver_module.minimalmodbus = orig_minimalmodbus
            driver_module.serial = orig_serial

    def test_connect_failure(self):
        """Test connection failure when port not available."""
        import manipulator_hardware.modbus_driver as driver_module

        orig_modbus_available = driver_module.MODBUS_AVAILABLE
        orig_minimalmodbus = driver_module.minimalmodbus
        orig_serial = driver_module.serial

        try:
            mock_minimalmodbus = MagicMock()
            mock_minimalmodbus.Instrument.side_effect = Exception('Port not found')

            mock_serial = MagicMock()
            mock_serial.PARITY_NONE = 'N'

            driver_module.MODBUS_AVAILABLE = True
            driver_module.minimalmodbus = mock_minimalmodbus
            driver_module.serial = mock_serial

            driver = driver_module.ModbusDriver()
            result = driver.connect()

            assert result is False
            assert driver.is_connected is False

        finally:
            driver_module.MODBUS_AVAILABLE = orig_modbus_available
            driver_module.minimalmodbus = orig_minimalmodbus
            driver_module.serial = orig_serial

    def test_connect_no_library(self):
        """Test connection fails when minimalmodbus not available."""
        import manipulator_hardware.modbus_driver as driver_module

        orig_modbus_available = driver_module.MODBUS_AVAILABLE

        try:
            driver_module.MODBUS_AVAILABLE = False

            driver = driver_module.ModbusDriver()
            result = driver.connect()

            assert result is False

        finally:
            driver_module.MODBUS_AVAILABLE = orig_modbus_available


class TestModbusDriverDisconnect:
    """Test disconnection handling."""

    def test_disconnect(self):
        """Test disconnection closes serial port."""
        import manipulator_hardware.modbus_driver as driver_module

        orig_modbus_available = driver_module.MODBUS_AVAILABLE
        orig_minimalmodbus = driver_module.minimalmodbus
        orig_serial = driver_module.serial

        try:
            mock_instrument = MagicMock()
            mock_instrument.serial.is_open = True

            mock_minimalmodbus = MagicMock()
            mock_minimalmodbus.Instrument.return_value = mock_instrument
            mock_minimalmodbus.MODE_RTU = 'rtu'

            mock_serial = MagicMock()
            mock_serial.PARITY_NONE = 'N'

            driver_module.MODBUS_AVAILABLE = True
            driver_module.minimalmodbus = mock_minimalmodbus
            driver_module.serial = mock_serial

            driver = driver_module.ModbusDriver()
            driver.connect()
            driver.disconnect()

            assert driver.is_connected is False
            mock_instrument.serial.close.assert_called()

        finally:
            driver_module.MODBUS_AVAILABLE = orig_modbus_available
            driver_module.minimalmodbus = orig_minimalmodbus
            driver_module.serial = orig_serial


class TestModbusDriverReadPosition:
    """Test read_position method."""

    def test_read_position_success(self):
        """Test successful position read using FC4."""
        import manipulator_hardware.modbus_driver as driver_module

        orig_modbus_available = driver_module.MODBUS_AVAILABLE
        orig_minimalmodbus = driver_module.minimalmodbus
        orig_serial = driver_module.serial

        try:
            mock_instrument = MagicMock()
            mock_instrument.serial.is_open = False
            mock_instrument.read_register.return_value = 12345

            mock_minimalmodbus = MagicMock()
            mock_minimalmodbus.Instrument.return_value = mock_instrument
            mock_minimalmodbus.MODE_RTU = 'rtu'

            mock_serial = MagicMock()
            mock_serial.PARITY_NONE = 'N'

            driver_module.MODBUS_AVAILABLE = True
            driver_module.minimalmodbus = mock_minimalmodbus
            driver_module.serial = mock_serial

            driver = driver_module.ModbusDriver()
            driver.connect()
            result = driver.read_position(slave_id=1, register=1003)

            assert result == 12345
            mock_instrument.read_register.assert_called_with(
                1003,
                number_of_decimals=0,
                functioncode=4,
                signed=False
            )

        finally:
            driver_module.MODBUS_AVAILABLE = orig_modbus_available
            driver_module.minimalmodbus = orig_minimalmodbus
            driver_module.serial = orig_serial

    def test_read_position_timeout(self):
        """Test read position returns None on timeout."""
        import manipulator_hardware.modbus_driver as driver_module

        orig_modbus_available = driver_module.MODBUS_AVAILABLE
        orig_minimalmodbus = driver_module.minimalmodbus
        orig_serial = driver_module.serial

        try:
            mock_instrument = MagicMock()
            mock_instrument.serial.is_open = False
            mock_instrument.read_register.side_effect = Exception('Timeout')

            mock_minimalmodbus = MagicMock()
            mock_minimalmodbus.Instrument.return_value = mock_instrument
            mock_minimalmodbus.MODE_RTU = 'rtu'

            mock_serial = MagicMock()
            mock_serial.PARITY_NONE = 'N'

            driver_module.MODBUS_AVAILABLE = True
            driver_module.minimalmodbus = mock_minimalmodbus
            driver_module.serial = mock_serial

            driver = driver_module.ModbusDriver(retry_count=1)
            driver.connect()
            result = driver.read_position(slave_id=1, register=1003)

            assert result is None

        finally:
            driver_module.MODBUS_AVAILABLE = orig_modbus_available
            driver_module.minimalmodbus = orig_minimalmodbus
            driver_module.serial = orig_serial

    def test_read_position_not_connected(self):
        """Test read position returns None when not connected."""
        from manipulator_hardware.modbus_driver import ModbusDriver
        driver = ModbusDriver()
        result = driver.read_position(slave_id=1, register=1003)
        assert result is None


class TestModbusDriverWritePosition:
    """Test write_position method."""

    def test_write_position_success(self):
        """Test successful position write using FC6."""
        import manipulator_hardware.modbus_driver as driver_module

        orig_modbus_available = driver_module.MODBUS_AVAILABLE
        orig_minimalmodbus = driver_module.minimalmodbus
        orig_serial = driver_module.serial

        try:
            mock_instrument = MagicMock()
            mock_instrument.serial.is_open = False

            mock_minimalmodbus = MagicMock()
            mock_minimalmodbus.Instrument.return_value = mock_instrument
            mock_minimalmodbus.MODE_RTU = 'rtu'

            mock_serial = MagicMock()
            mock_serial.PARITY_NONE = 'N'

            driver_module.MODBUS_AVAILABLE = True
            driver_module.minimalmodbus = mock_minimalmodbus
            driver_module.serial = mock_serial

            driver = driver_module.ModbusDriver()
            driver.connect()
            result = driver.write_position(slave_id=1, register=2999, value=5000)

            assert result is True
            mock_instrument.write_register.assert_called_with(
                2999,
                5000,
                number_of_decimals=0,
                functioncode=6,
                signed=False
            )

        finally:
            driver_module.MODBUS_AVAILABLE = orig_modbus_available
            driver_module.minimalmodbus = orig_minimalmodbus
            driver_module.serial = orig_serial

    def test_write_position_failure(self):
        """Test write position returns False on failure."""
        import manipulator_hardware.modbus_driver as driver_module

        orig_modbus_available = driver_module.MODBUS_AVAILABLE
        orig_minimalmodbus = driver_module.minimalmodbus
        orig_serial = driver_module.serial

        try:
            mock_instrument = MagicMock()
            mock_instrument.serial.is_open = False
            mock_instrument.write_register.side_effect = Exception('Write error')

            mock_minimalmodbus = MagicMock()
            mock_minimalmodbus.Instrument.return_value = mock_instrument
            mock_minimalmodbus.MODE_RTU = 'rtu'

            mock_serial = MagicMock()
            mock_serial.PARITY_NONE = 'N'

            driver_module.MODBUS_AVAILABLE = True
            driver_module.minimalmodbus = mock_minimalmodbus
            driver_module.serial = mock_serial

            driver = driver_module.ModbusDriver(retry_count=1)
            driver.connect()
            result = driver.write_position(slave_id=1, register=2999, value=5000)

            assert result is False

        finally:
            driver_module.MODBUS_AVAILABLE = orig_modbus_available
            driver_module.minimalmodbus = orig_minimalmodbus
            driver_module.serial = orig_serial

    def test_write_position_not_connected(self):
        """Test write position returns False when not connected."""
        from manipulator_hardware.modbus_driver import ModbusDriver
        driver = ModbusDriver()
        result = driver.write_position(slave_id=1, register=2999, value=5000)
        assert result is False


class TestModbusDriverStatusMethods:
    """Test status register methods."""

    def test_read_error_code(self):
        """Test read_error_code reads register 999."""
        import manipulator_hardware.modbus_driver as driver_module

        orig_modbus_available = driver_module.MODBUS_AVAILABLE
        orig_minimalmodbus = driver_module.minimalmodbus
        orig_serial = driver_module.serial

        try:
            mock_instrument = MagicMock()
            mock_instrument.serial.is_open = False
            mock_instrument.read_register.return_value = 0

            mock_minimalmodbus = MagicMock()
            mock_minimalmodbus.Instrument.return_value = mock_instrument
            mock_minimalmodbus.MODE_RTU = 'rtu'

            mock_serial = MagicMock()
            mock_serial.PARITY_NONE = 'N'

            driver_module.MODBUS_AVAILABLE = True
            driver_module.minimalmodbus = mock_minimalmodbus
            driver_module.serial = mock_serial

            driver = driver_module.ModbusDriver()
            driver.connect()
            result = driver.read_error_code(slave_id=1)

            assert result == 0
            mock_instrument.read_register.assert_called_with(
                999,
                number_of_decimals=0,
                functioncode=4,
                signed=False
            )

        finally:
            driver_module.MODBUS_AVAILABLE = orig_modbus_available
            driver_module.minimalmodbus = orig_minimalmodbus
            driver_module.serial = orig_serial

    def test_is_device_ready(self):
        """Test is_device_ready reads register 1002 and returns bool."""
        import manipulator_hardware.modbus_driver as driver_module

        orig_modbus_available = driver_module.MODBUS_AVAILABLE
        orig_minimalmodbus = driver_module.minimalmodbus
        orig_serial = driver_module.serial

        try:
            mock_instrument = MagicMock()
            mock_instrument.serial.is_open = False
            mock_instrument.read_register.return_value = 1

            mock_minimalmodbus = MagicMock()
            mock_minimalmodbus.Instrument.return_value = mock_instrument
            mock_minimalmodbus.MODE_RTU = 'rtu'

            mock_serial = MagicMock()
            mock_serial.PARITY_NONE = 'N'

            driver_module.MODBUS_AVAILABLE = True
            driver_module.minimalmodbus = mock_minimalmodbus
            driver_module.serial = mock_serial

            driver = driver_module.ModbusDriver()
            driver.connect()
            result = driver.is_device_ready(slave_id=1)

            assert result is True
            mock_instrument.read_register.assert_called_with(
                1002,
                number_of_decimals=0,
                functioncode=4,
                signed=False
            )

        finally:
            driver_module.MODBUS_AVAILABLE = orig_modbus_available
            driver_module.minimalmodbus = orig_minimalmodbus
            driver_module.serial = orig_serial

    def test_is_device_busy(self):
        """Test is_device_busy reads register 1001 and returns bool."""
        import manipulator_hardware.modbus_driver as driver_module

        orig_modbus_available = driver_module.MODBUS_AVAILABLE
        orig_minimalmodbus = driver_module.minimalmodbus
        orig_serial = driver_module.serial

        try:
            mock_instrument = MagicMock()
            mock_instrument.serial.is_open = False
            mock_instrument.read_register.return_value = 0

            mock_minimalmodbus = MagicMock()
            mock_minimalmodbus.Instrument.return_value = mock_instrument
            mock_minimalmodbus.MODE_RTU = 'rtu'

            mock_serial = MagicMock()
            mock_serial.PARITY_NONE = 'N'

            driver_module.MODBUS_AVAILABLE = True
            driver_module.minimalmodbus = mock_minimalmodbus
            driver_module.serial = mock_serial

            driver = driver_module.ModbusDriver()
            driver.connect()
            result = driver.is_device_busy(slave_id=1)

            assert result is False
            mock_instrument.read_register.assert_called_with(
                1001,
                number_of_decimals=0,
                functioncode=4,
                signed=False
            )

        finally:
            driver_module.MODBUS_AVAILABLE = orig_modbus_available
            driver_module.minimalmodbus = orig_minimalmodbus
            driver_module.serial = orig_serial


class TestModbusDriverRetryLogic:
    """Test retry logic."""

    def test_retry_logic_success_after_failure(self):
        """Test retry succeeds after initial failures."""
        import manipulator_hardware.modbus_driver as driver_module

        orig_modbus_available = driver_module.MODBUS_AVAILABLE
        orig_minimalmodbus = driver_module.minimalmodbus
        orig_serial = driver_module.serial

        try:
            mock_instrument = MagicMock()
            mock_instrument.serial.is_open = False
            # Fail twice, then succeed
            mock_instrument.read_register.side_effect = [
                Exception('Timeout'),
                Exception('Timeout'),
                42
            ]

            mock_minimalmodbus = MagicMock()
            mock_minimalmodbus.Instrument.return_value = mock_instrument
            mock_minimalmodbus.MODE_RTU = 'rtu'

            mock_serial = MagicMock()
            mock_serial.PARITY_NONE = 'N'

            driver_module.MODBUS_AVAILABLE = True
            driver_module.minimalmodbus = mock_minimalmodbus
            driver_module.serial = mock_serial

            driver = driver_module.ModbusDriver(retry_count=3)
            driver.connect()
            result = driver.read_position(slave_id=1, register=1003)

            assert result == 42
            assert mock_instrument.read_register.call_count == 3

        finally:
            driver_module.MODBUS_AVAILABLE = orig_modbus_available
            driver_module.minimalmodbus = orig_minimalmodbus
            driver_module.serial = orig_serial

    def test_retry_logic_all_failures(self):
        """Test retry exhausts all attempts on persistent failure."""
        import manipulator_hardware.modbus_driver as driver_module

        orig_modbus_available = driver_module.MODBUS_AVAILABLE
        orig_minimalmodbus = driver_module.minimalmodbus
        orig_serial = driver_module.serial

        try:
            mock_instrument = MagicMock()
            mock_instrument.serial.is_open = False
            mock_instrument.read_register.side_effect = Exception('Timeout')

            mock_minimalmodbus = MagicMock()
            mock_minimalmodbus.Instrument.return_value = mock_instrument
            mock_minimalmodbus.MODE_RTU = 'rtu'

            mock_serial = MagicMock()
            mock_serial.PARITY_NONE = 'N'

            driver_module.MODBUS_AVAILABLE = True
            driver_module.minimalmodbus = mock_minimalmodbus
            driver_module.serial = mock_serial

            driver = driver_module.ModbusDriver(retry_count=3)
            driver.connect()
            result = driver.read_position(slave_id=1, register=1003)

            assert result is None
            assert mock_instrument.read_register.call_count == 3

        finally:
            driver_module.MODBUS_AVAILABLE = orig_modbus_available
            driver_module.minimalmodbus = orig_minimalmodbus
            driver_module.serial = orig_serial


class TestModbusDriverRegisterConstants:
    """Test register address constants."""

    def test_status_register_constants(self):
        """Test status register constants are defined correctly."""
        from manipulator_hardware.modbus_driver import ModbusDriver

        assert ModbusDriver.REG_ERR_CODE == 999
        assert ModbusDriver.REG_MODULE_IS_BUSY == 1001
        assert ModbusDriver.REG_MODULE_READY == 1002
