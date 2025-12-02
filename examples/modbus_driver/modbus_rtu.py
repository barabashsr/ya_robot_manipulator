import minimalmodbus
import serial
from typing import List, Any, Dict, Optional
from pathlib import Path
import yaml

def _load_config() -> Dict[str, Any]:
    try:
        base_dir = Path(__file__).resolve().parent
        config_path = base_dir / "configuration.yml"
        if not config_path.exists():
            return {}
        with open(config_path, "r", encoding="utf-8") as f:
            data = yaml.safe_load(f)
            return data if isinstance(data, dict) else {}
    except Exception:
        return {}


def _cfg_get(key: str, default: Any = None) -> Any:
    cfg = _load_config()
    try:
        cur: Any = cfg
        for k in key.split("."):
            cur = cur[k]
        return cur
    except Exception:
        return default

def create_instrument(slave_id: int, port: str | None = None) -> minimalmodbus.Instrument:
    if port is None:
        port = _cfg_get("modbus.port", "/tmp/ttyACM0")
        baudrate = _cfg_get("modbus.baudrate", 19200)
    instrument = minimalmodbus.Instrument(port, slave_id)
    instrument.serial.baudrate = baudrate
    instrument.serial.bytesize = 8
    instrument.serial.parity = serial.PARITY_NONE
    instrument.serial.stopbits = 1
    instrument.serial.timeout = 0.5
    instrument.mode = minimalmodbus.MODE_RTU
    instrument.clear_buffers_before_each_transaction = True
    return instrument
 
def read_holding_register(register, slave_id: int) -> int | None:
    instrument = create_instrument(slave_id)
    try:
        return instrument.read_register(register, number_of_decimals=0, functioncode=3, signed=False)
    except Exception as exc: 
        print(f"Read failed: {exc}")
        return None

def read_input_register(register, slave_id: int) -> int | None:
    instrument = create_instrument(slave_id)
    try:
        return instrument.read_register(register, number_of_decimals=0, functioncode=4, signed=False)
    except Exception as exc: 
        print(f"Read failed: {exc}")
        return None

def read_coil_register(register, slave_id: int) -> int | None:
    instrument = create_instrument(slave_id)
    try:
        return instrument.read_bit(register, functioncode=1)
    except Exception as exc: 
        print(f"Read failed: {exc}")
        return None

def read_coil_registers(register, slave_id: int, number_of_bits: int = 3) -> list | None:
    instrument = create_instrument(slave_id)
    try:
        return instrument.read_bits(register, number_of_bits, functioncode=1)
    except Exception as exc: 
        print(f"Read failed: {exc}")
        return None

def write_coil_register(register, slave_id: int, value_bit: int = 1) -> int | None:
    instrument = create_instrument(slave_id)
    try:
        return instrument.write_bit(register, value_bit, functioncode=5)
    except Exception as exc: 
        print(f"Read failed: {exc}")
        return False

def write_holding_register(register, slave_id: int, value: int) -> bool:
    instrument = create_instrument(slave_id)
    try:
        instrument.write_register(register, value, number_of_decimals=0, functioncode=6, signed=False)
        return True
    except Exception as exc:
        print(f"Write failed: {exc}")
        return None

#def write_holding_registers(register, slave_id, )


def write_coil_registers(register: int, slave_id: int, value_bits: List) -> int | None:
    instrument = create_instrument(slave_id)
    try:
        return instrument.write_bits(register, value_bits)
    except Exception as exc: 
        print(f"Write failed: {exc}")
        return False






