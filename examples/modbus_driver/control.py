import yaml
import os
import time
from typing import Dict, Any, Optional
from pathlib import Path
from app.modbus_rtu import (
    read_holding_register,
    write_holding_register,
    read_coil_register,
    write_coil_register,
    read_coil_registers,
    write_coil_registers,
    read_input_register,
)
from app.offsets import offset_ai, offset_ao, offset_do
from app.global_state import current_task_state, telemetry
from app.metrics import (
    set_module_ready_status,
    set_module_error_status,
    set_axis_current_state,
    set_current_operation_status,
)


class ConfigLoader:
    """Загрузчик конфигурации из YAML файла"""
    
    def __init__(self, config_file: str = "configuration.yml"):
        self.config_file = Path(__file__).parent / config_file
        self._config: Optional[Dict[str, Any]] = None
    
    def load_config(self) -> Dict[str, Any]:
        """Загружает конфигурацию из YAML файла"""
        if not self.config_file.exists():
            raise FileNotFoundError(f"Файл конфигурации не найден: {self.config_file}")
        
        try:
            with open(self.config_file, 'r', encoding='utf-8') as file:
                self._config = yaml.safe_load(file)
            return self._config
        except yaml.YAMLError as e:
            raise ValueError(f"Ошибка парсинга YAML: {e}")
        except Exception as e:
            raise RuntimeError(f"Ошибка чтения файла: {e}")
    
    def get(self, key: str, default: Any = None) -> Any:
        """Получает значение по ключу (поддерживает вложенные ключи через точку)"""
        if self._config is None:
            self.load_config()
        
        keys = key.split('.')
        value = self._config
        
        try:
            for k in keys:
                value = value[k]
            return value
        except (KeyError, TypeError):
            return default


# Глобальный экземпляр загрузчика конфигурации
config_loader = ConfigLoader()

def get_config(key: str = None, default: Any = None) -> Any:
    """Удобная функция для получения конфигурации"""
    if key is None:
        return config_loader.load_config()
    return config_loader.get(key, default)


 



cfg = get_config()

def _module_label(slave_id: int) -> str:
    return f"slave_{slave_id}"

""" Считывание статуса устройства"""
def read_error_code_device(slave_id: int, register: int | None = None) -> int | None:
    if register is None:
        register = cfg['inp_reg']['err_code']
    try:
        return read_input_register(register, slave_id)
    except Exception as exc:
        print("Error:", exc)
        return None


""" Проверка статуса готовности устройства к исполнению комманд """
def read_ready_status_device(slave_id: int, register: int | None = None) ->  int | None:
    if register is None:
        register = cfg['inp_reg']['module_ready']
    try:
        return read_input_register(register, slave_id)
    except Exception as exc:
        print("Error:", exc)
        return None

""" Проверка занятости устройства """
def read_command_prep_device(slave_id: int, register: int | None = None) -> int | None:
    if register is None:
        register = cfg['inp_reg']['module_is_busy']
    try:
        return read_input_register(register, slave_id)
    except Exception as exc:
        print("Error:", exc)
        return None   

""" Получение текущего значение оси """
def read_сurrent_ord(slave_id: int, ord: int | None = None, register: int | None = None) -> int | None:
    if register is None:
        if ord is None:
            return None, "Ошибка: Не указан номер ординаты"
        register = cfg['inp_reg'][f"ord{ord}_current"]
    try:
        return read_input_register(register, slave_id)
    except Exception as exc:
        print("Error:", exc)
        return None

""" Получение имени оси ASCII буква """
def read_ord_name(slave_id: int, ord: int, register: int | None = None) -> str | None:
    if register is None:
        register = cfg['inp_reg'][f"ord{ord}_name"]
    try:
        return chr(read_input_register(register, slave_id))
    except Exception as exc:
        print("Error:", exc)
        return None

""" Получение или изменение заданного значения оси """
def ord_value_register(slave_id: int, ord: int | None = None, wr_flag: bool = False, value: int | None = None, register: int | None = None) -> bool | None:
    if register is None:
        if ord is None:
            return None, "Ошибка: Не указан номер ординаты"
        register = cfg['hold_reg'][f"ord{ord}_given"]
    if wr_flag:
        if value is None:
            return None, "Ошибка: Не указано значение для записи в регистр"
        try:
            return write_holding_register(register, slave_id, value)
        except Exception as exc:
            print("Error: ", exc)
            return None
    else:
        try:
            return read_holding_register(register, slave_id)
        except Exception as exc:
            print("Error: ", exc)
            return None

""" Получение и изменение заданной скорости оси """
def ord_speed_register(slave_id: int, ord: int, wr_flag: bool = False, value: int | None = None, register: int | None = None) -> bool | int | None:
    if register is None:
        register = cfg['hold_reg'][f"ord{ord}_speed_given"]
    if wr_flag:
        if value is None:
            return None, "Ошибка: Не указано значение для записи в регистр"
        try:
            return write_holding_register(register, slave_id, value)
        except Exception as exc:
            print("Error: ", exc)
            return None
    else:
        try:
            return read_holding_register(register, slave_id)
        except Exception as exc:
            print("Error: ", exc)
            return None


""" Получение или изменение числа импульсов на оброт двигателя оси """
def ord_pulse_turn_register(slave_id: int, ord: int, wr_flag: bool = False, value: int | None = None, register: int | None = None) -> bool | int | None:
    if register is None:
        register = cfg['hold_reg'][f"ord{ord}_pulse_turn"]
    if wr_flag:
        if value is None:
            return None, "Ошибка: Не указано значение для записи в регистр"
        try:
            return write_holding_register(register, slave_id, value)
        except Exception as exc:
            print("Error: ", exc)
            return None
    else:
        try:
            return read_holding_register(register, slave_id)
        except Exception as exc:
            print("Error: ", exc)
            return None

""" Получение или изменение заданного значения ускорения/торможения (множитель) оси """
def ord_acl_dcl_register(slave_id: int, ord: int, wr_flag: bool = False, value: int | None = None, register: int | None = None) -> bool | int | None:
    if register is None:
        register = cfg['hold_reg'][f"ord{ord}_acl_dcl_point"]
    if wr_flag:
        if value is None:
            return None, "Ошибка: Не указано значение для записи в регистр"
        try:
            return write_holding_register(register, slave_id, value)
        except Exception as exc:
            print("Error: ", exc)
            return None
    else:
        try:
            return read_holding_register(register, slave_id)
        except Exception as exc:
            print("Error: ", exc)
            return None

""" Активация SoftReset """
def soft_reset_coil(slave_id: int, register: int | None = None, value: bool = True, wr_flag: bool = True) -> int | None:
    if register is None:
        register = cfg['disc_o_reg']['soft_reset']
    if wr_flag:
        try:
            return write_coil_register(register, slave_id, value)
        except Exception as exc:
            print("Error: ", exc)
            return None
    else:
        try:
            return read_coil_register(register, slave_id)
        except Exception as exc:
            print("Error: ", exc)
            return None

""" Указание направление движения оси Y """
def direction_Y_coil(slave_id: int, ord: int | None = None, register: int | None = None, value: bool = False, wr_flag: bool = True) -> int | None:
    if register is None:
        if ord is None:
            return None, "Ошибка: Не указан номер ординаты"
        register = cfg['disc_o_reg'][f"direction_ord{ord}"]
    if wr_flag:
        try:
            return write_coil_register(register, slave_id, value)
        except Exception as exc:
            print("Error: ", exc)
            return None
    else:
        try:
            return read_coil_register(register, slave_id)
        except Exception as exc:
            print("Error: ", exc)
            return None

""" Указание направление движения оси B """
def direction_B_coil(slave_id: int, ord: int | None = None, register: int | None = None, value: bool = False, wr_flag: bool = True) -> int | bool | None:
    if register is None:
        if ord is None:
            return None, "Ошибка: Не указан номер ординаты"
        register = cfg['disc_o_reg'][f"direction_ord{ord}"]
    if wr_flag:
        try:
            return write_coil_register(register, slave_id, value)
        except Exception as exc:
            print("Error: ", exc)
            return False
    else:
        try:
            return read_coil_register(register, slave_id)
        except Exception as exc:
            print("Error: ", exc)
            return False



""" Активация концевого выключателя MIN оси """
def min_limit_switch_coil(slave_id: int, ord: int | None = None, register: int | None = None) -> int | None:
    if register is None:
        if ord is None:
            return None, "Ошибка: Не указан номер ординаты"
        register = cfg['inp_reg'][f"min_limit_switch_ord{ord}"]
    try:
        return read_input_register(register, slave_id)
    except Exception as exc:
        print("Error: ", exc)
        return None
   
""" Активация концевого выключателя MAX оси """
def max_limit_switch_coil(slave_id: int, ord: int | None = None, register: int | None = None) -> int | None:
    if register is None:
        if ord is None:
            return None, "Ошибка: Не указан номер ординаты"
        register = cfg['inp_reg'][f"max_limit_switch_ord{ord}"]
    try:
        return read_input_register(register, slave_id)
    except Exception as exc:
        print("Error: ", exc)
        return None

""" Активация привода оси (Возможен просмотр состояния) """
def engine_activ_coil(slave_id: int, ord: int, register: int | None = None, value: bool = True, wr_flag: bool = True) -> int | None:
    if register is None:
        register = cfg['disc_o_reg'][f"engine_activ_ord{ord}"]
    if wr_flag:
        try:
            return write_coil_register(register, slave_id, value)
        except Exception as exc:
            print("Error: ", exc)
            return None
    else:
        try:
            return read_coil_register(register, slave_id)
        except Exception as exc:
            print("Error: ", exc)
            return None

""" Деактивация привода оси """
def engine_deactive_coil(slave_id: int, ord: int, register: int | None = None) -> str | None:
    if register is None:
        register = cfg['disc_o_reg'][f"engine_activ_ord{ord}"]
    try:
        engine_activ_coil(slave_id, ord, register, value=False)
    except Exception as exc:
        print("Error: ", exc)

""" Активация тормоза оси (Возможен просмотр состояния) """
def brake_activ_coil(slave_id: int, ord: int, register: int | None = None, value: bool = True, wr_flag: bool = True) -> int | None:
    if register is None:
        register = cfg['disc_o_reg'][f"brake_activ_ord{ord}"]
    if wr_flag:
        try:
            return write_coil_register(register, slave_id, value)
        except Exception as exc:
            print("Error: ", exc)
            return None
    else:
        try:
            return read_coil_register(register, slave_id)
        except Exception as exc:
            print("Error: ", exc)
            return None

""" Деактивация тормоза оси """
def brake_deactive_coil(slave_id: int, ord: int, register: int | None = None) -> str | None:
    if register is None:
        register = cfg['disc_o_reg'][f"brake_activ_ord{ord}"]
    try:
        engine_activ_coil(slave_id, ord, register, value=False)
    except Exception as exc:
        print("Error: ", exc)

""" Активация/деактивация электромагнита """
def magnet_action(slave_id: int, value: bool, register: int | None = None) -> bool | None:
    if register is None:
        register = cfg['disc_o_reg'][f"magnet"]
    try:
        return write_coil_register(register, slave_id, value)
    except Exception as exc:
        current_task_state.error_code = 255
        print("Ошибка активации/деактивации электромагнита: {exc}")
        raise Exception(f"Ошибка активации/деактивации электромагнита: {exc}")

""" Обнуление координаты Оси """
def null_ord_coil(slave_id: int, ord: int | None = None, register: int | None = None, wr_flag: bool = True, value: bool = True) -> int | bool | None:
    if register is None:
        if ord is None:
            return None, "Ошибка: Не указан номер ординаты"
        register = cfg['disc_o_reg'][f"null_ord{ord}"]
    if wr_flag:
        try:
            return write_coil_register(register, slave_id, value)
        except Exception as exc:
            print("Error: ", exc)
            return False
    else:
        try:
            return read_coil_register(register, slave_id)
        except Exception as exc:
            print("Error: ", exc)
            return None    

def move_ordinate_XZ(slave_id: int, xord: int, zord: int, reg_X: int, reg_Z: int, cur_X_reg: int, cur_Z_reg: int, err_XZ_reg: int, state_XZ_reg: int) -> int | None:
    set_current_operation_status("Движение ординаты X и Z")
    val = ord_value_register(slave_id, wr_flag=True, value=xord, register=reg_X)
    if val is None:
        current_task_state.error_code = 255
        print("Ошибка чтения: коммуникация/исключение")
        raise Exception("Неудалось установить значение оси X")
    val = ord_value_register(slave_id, wr_flag=True, value=zord, register=reg_Z)
    if val is None:
        current_task_state.error_code = 255
        print("Ошибка чтения: коммуникация/исключение")
        raise Exception("Неудалось установить значение оси Z")

    counter = 0
    max_attempts = 15
    while counter < max_attempts:
        current_state_X = read_сurrent_ord(slave_id, register=cur_X_reg)
        set_axis_current_state("engineXZ", "X", current_state_X)
        telemetry.position.x = current_state_X
        current_state_Z = read_сurrent_ord(slave_id, register=cur_Z_reg)
        set_axis_current_state("engineXZ", "Z", current_state_Z)
        telemetry.lift.z = current_state_Z
        error_state = read_error_code_device(slave_id, err_XZ_reg)
        ready_state_XZ = read_ready_status_device(slave_id, state_XZ_reg)
        if current_state_X == xord and current_state_Z == zord:
            if ready_state_XZ == 0:
                set_module_ready_status("engineXZ", 1)
                time.sleep(1)
                return None
        else:
            set_module_ready_status("engineXZ", 0)
        if error_state is not None and error_state > 0:
            current_task_state.error_code = error_state
            print("Ошибка движения оси XZ: ", error_state)
            set_module_error_status("engineXZ", error_state)
            raise Exception(f"Ошибка движения оси XZ: {error_state}")
        print(f"Движение к заданным координатам, {counter} секунд. Текущее положение Z {current_state_Z} ::: Текущее положение X {current_state_X}")
        current_task_state.message = f"Движение к заданным координатам, {counter} секунд. Текущее положение Z {current_state_Z} ::: Текущее положение X {current_state_X}"
        time.sleep(1)
        counter += 1
    current_task_state.error_code = 255
    set_module_error_status("engineXZ", current_task_state.error_code)
    print("Ошибка движения оси XZ: предел времени движения")
    raise Exception(f"Предел времени движения оси XZ")


def move_ordinate_YB_to_limit_switch(slave_id: int, rightunit: bool, null_YB_reg: int, direc_YB_ord: int, limit_switch_reg: int, err_YB_reg: int, state_YB_reg: int, axis_name: str | None = None) -> bool | None:
    if rightunit:
        YB_dir = direction_Y_coil(slave_id, register=direc_YB_ord, value=True)
        if YB_dir is not None:
            current_task_state.error_code = 255
            raise Exception(f"Ошибка установки направления оси {axis_name}")
    else:
        YB_dir = direction_Y_coil(slave_id, register=direc_YB_ord, value=False)
        if YB_dir is not None:
            current_task_state.error_code = 255
            raise Exception(f"Ошибка установки направления оси {axis_name}")
    move_YB = null_ord_coil(slave_id, register=null_YB_reg, value=True)
    if move_YB is not None:
        current_task_state.error_code = 255
        raise Exception(f"Ошибка установки направления оси {axis_name}")
    counter = 0
    max_attempts = 10
    while counter < max_attempts:
        error_state = read_error_code_device(slave_id, err_YB_reg)
        ready_state_YB = read_ready_status_device(slave_id, state_YB_reg)
        if rightunit:
            state_limit_switch = min_limit_switch_coil(slave_id, register=limit_switch_reg)
            if state_limit_switch > 0:
                if ready_state_YB == 0:
                    time.sleep(1)
                    return None
        else:
            state_limit_switch = max_limit_switch_coil(slave_id, register=limit_switch_reg)
            if state_limit_switch > 0:
                if ready_state_YB == 0:
                    return None
        if error_state is not None and error_state > 0:
            current_task_state.error_code = error_state
            print(f"Ошибка движения оси {axis_name}: {error_state}")
            raise Exception(f"Ошибка движения оси {axis_name}: {error_state}")
        print(f"Движение оси {axis_name} до концевика ::: {counter} секунд")
        current_task_state.message = f"Движение оси {axis_name} до концевика ::: {counter} секунд"
        time.sleep(1)
        counter += 1
    current_task_state.error_code = 255
    print(f"Ошибка движения оси {axis_name}: предел времени движения")
    raise Exception(f"Предел времени движения оси {axis_name}")

def move_YB(slave_id: int, ybord: int, reg_YB: int, cur_YB_reg: int, err_YB_reg: int, state_YB_reg: int, axis_name: str | None = None) -> bool | None:
    val = ord_value_register(slave_id, wr_flag=True, value=ybord, register=reg_YB)
    if val is None:
        current_task_state.error_code = 255
        raise Exception(f"Неудалось установить значение оси {axis_name}")
    
    counter = 0
    max_attempts = 10
    while counter < max_attempts:
        current_state_YB = read_сurrent_ord(slave_id, register=cur_YB_reg)
        error_state = read_error_code_device(slave_id, err_YB_reg)
        ready_state_YB = read_ready_status_device(slave_id, state_YB_reg)
        if current_state_YB == ybord:
            if ready_state_YB == 0:
                time.sleep(2)
                return None
        if error_state is not None and error_state > 0:
            current_task_state.error_code = error_state
            raise Exception(f"Ошибка движения оси {axis_name}: {error_state}")
        print(f"Движение оси {axis_name} к заданной координате, {counter} секунд. Текущее положение {axis_name} {current_state_YB}")
        current_task_state.message = f"Движение оси {axis_name} к заданной координате, {counter} секунд. Текущее положение {axis_name} {current_state_YB}"
        time.sleep(1)
        counter += 1
    current_task_state.error_code = 255
    print(f"Ошибка движения оси {axis_name}: предел времени движения")
    raise Exception(f"Предел времени движения оси {axis_name}")

def move_ordinate_A_to_limit_switch(slave_id: int, null_reg: int, limit_switch_reg: int, err_reg: int, state_reg: int, switch_min: bool) -> bool | None:
    move_A = null_ord_coil(slave_id, register=null_reg, value=True)
    if move_A is not None:
        current_task_state.error_code = 255
        raise Exception("Неудалось дать команду для опускание захвата")
    counter = 0
    max_attempts = 10
    while counter < max_attempts:
        error_state = read_error_code_device(slave_id, err_reg)
        ready_state_ACD = read_ready_status_device(slave_id, state_reg)
        if switch_min:
            state_limit_switch = min_limit_switch_coil(slave_id, register=limit_switch_reg)
            if state_limit_switch > 0:
                if ready_state_ACD == 0:
                    return None
        else:
            state_limit_switch = max_limit_switch_coil(slave_id, register=limit_switch_reg)
            if state_limit_switch > 0:
                if ready_state_ACD == 0:
                    return None
        
        if error_state is not None and error_state > 0:
            current_task_state.error_code = error_state
            raise Exception(f"Ошибка движения захвата: {error_state}")
        print(f"Отпускаем захват ::: {counter} секунд")
        current_task_state.message = f"Отпускаем захват ::: {counter} секунд"
        time.sleep(1)
        counter += 1
    current_task_state.error_code = 255
    print("Ошибка движения захвата до коробки: предел времени движения")
    raise Exception("Предел времени движения захвата до коробки")

def box_capture_activation(slave_id: int, active_reg: int, limit_switch_reg, value: bool, err_reg: int, state_reg: int) -> bool | None:
    val = ord_value_register(slave_id, wr_flag=True, value=int(value), register=active_reg)
    if val is None:
        current_task_state.error_code = 255
        print("Неудалось дать команду захвата коробки")
        raise Exception("Неудалось дать команду захвата коробки")
    
    counter = 0
    max_attempts = 6
    while counter < max_attempts:
        error_state = read_error_code_device(slave_id, err_reg)
        ready_state_ACD = read_ready_status_device(slave_id, state_reg)
        state_limit_switch = max_limit_switch_coil(slave_id, register=limit_switch_reg)
        if state_limit_switch > 0:
            if ready_state_ACD == 0:
                return None
        if error_state is not None and error_state > 0:
            current_task_state.error_code = error_state
            print(f"Ошибка {'захвата коробки' if value else 'сброса коробки'}: {error_state}")
            raise Exception(f"Ошибка {'захвата коробки' if value else 'сброса коробки'}: {error_state}")
        print(f"Идет  {'захват коробки' if value else 'сброс коробки в контейнер'}, {counter} секунд.")
        current_task_state.message = f"Идет  {'захват коробки' if value else 'сброс коробки в контейнер'}, {counter} секунд."
        time.sleep(1)
        counter += 1
    current_task_state.error_code = 255
    print(f"Ошибка {'захвата коробки' if value else 'сброса коробки'}: предел времени движения")
    raise Exception("Предел времени ожидания для {'захвата коробки' if value else 'сброса коробки'}")

def box_lifting_up(slave_id: int, aord: int, reg_A: int, cur_A_reg: int, err_reg: int, state_reg: int) -> bool | None:
    val = ord_value_register(slave_id, wr_flag=True, value=aord, register=reg_A)
    if val is None:
        current_task_state.error_code = 255
        raise Exception("Неудалось дать команду поднятия захвата")
    
    counter = 0
    max_attempts = 5
    while counter < max_attempts:
        current_state_A = read_сurrent_ord(slave_id, register=cur_A_reg)
        error_state = read_error_code_device(slave_id, err_reg)
        ready_state_ACD = read_ready_status_device(slave_id, state_reg)
        if current_state_A == aord:
            if ready_state_ACD == 0:
                return None
        if error_state is not None and error_state > 0:
            current_task_state.error_code = error_state
            raise Exception(f"Ошибка поднятия захвата: {error_state}")
        print(f"Процесс поднятия захвата, {counter} секунд. Текущее положение ОСИ 'А' {current_state_A}")
        current_task_state.message = f"Процесс поднятия захвата, {counter} секунд. Текущее положение ОСИ 'А' {current_state_A}"
        time.sleep(1)
        counter += 1
    current_task_state.error_code = 255
    print("Ошибка поднятия захвата: предел времени движения")
    raise Exception("Предел времени для поднятия коробки")

def throw_box_into_container(slave_id: int, active_reg: int, limit_switch_reg, value: bool, err_reg: int, state_reg: int) -> bool | None:
    val = ord_value_register(slave_id, wr_flag=True, value=int(value), register=active_reg)
    if val is None:
        current_task_state.error_code = 255
        print("Неудалось дать команду движения захвата до точки сброса коробки")
        raise Exception("Неудалось дать команду движения захвата до точки сброса коробки")
    
    counter = 0
    max_attempts = 7
    while counter < max_attempts:
        error_state = read_error_code_device(slave_id, err_reg)
        ready_state_ACD = read_ready_status_device(slave_id, state_reg)
        state_limit_switch = max_limit_switch_coil(slave_id, register=limit_switch_reg)
        if state_limit_switch > 0:
            if ready_state_ACD == 0:
                return None
        if error_state is not None and error_state > 0:
            current_task_state.error_code = error_state
            print(f"Ошибка движения захвата { 'до точки сброса коробки' if value else 'от точки сброса коробки' }: {error_state}")
            raise Exception(f"Ошибка движения захвата { 'до точки сброса коробки' if value else 'от точки сброса коробки' }: {error_state}")
        print(f"Движение захвата { 'до точки сброса коробки' if value else 'от точки сброса коробки' }, {counter} секунд.")
        current_task_state.message = f"Движение захвата { 'до точки сброса коробки' if value else 'от точки сброса коробки' }, {counter} секунд."
        time.sleep(1)
        counter += 1
    current_task_state.error_code = 255
    print(f"Ошибка движения захвата { 'до точки сброса коробки' if value else 'от точки сброса коробки' }: предел времени движения")
    raise Exception(f"Предел времени для движения захвата { 'до точки сброса коробки' if value else 'от точки сброса коробки' }")











        
        

                 














if __name__ == "__main__":
    print("module_ready: ",     read_error_code_device(cfg['modbus']['devices']['engine1']))
