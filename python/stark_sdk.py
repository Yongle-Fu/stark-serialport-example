import abc
import os
import platform
import numpy as np
from enum import IntEnum  # Enum declarations
from stark_logger import logwrap, SRLOG

from cffi import FFI

ffi = FFI()

def fatal_error(msg):
    print("FATAL_ERROR:" + msg)

def load_library():
    lib_dir = os.path.join(os.path.abspath(os.path.dirname(__file__)), "..", "dist")
    print('dist_dir:', lib_dir)

    # 1. load header
    with open(os.path.join(lib_dir, "include", "stark_sdk.h"), encoding='utf-8') as sdk_header:
        sdk_header = sdk_header.read() \
            .split("CFFI_DEF_START")[1] \
            .split("CFFI_DEF_END")[0] \
            .replace("SDK_EXTERN ", "") \
            .replace("#if defined(_WIN32) || TARGET_OS_OSX", "") \
            .replace("#endif", "")
        ffi.cdef(sdk_header)

    # 2. find library path and load
    arch = platform.architecture()[0]
    if platform.system() == "Darwin":
        return ffi.dlopen(os.path.join(lib_dir, "shared", "mac", "libstark.dylib"))
    elif platform.system() == "Linux":
        return ffi.dlopen(os.path.join(lib_dir, "shared", "linux", "libstark.so"))
    elif platform.system() == "Windows" and arch == "64bit":
        # add lib_path to environment variable 'PATH' to load the dependent DLLs.
        lib_path = os.path.join(lib_dir, "shared", "win")
        os.environ["PATH"] += os.pathsep + lib_path
        return ffi.dlopen(lib_path.join("stark.dll"))
    else:
        raise Exception("Unsupported platform: " + platform.system() + ", arch: " + arch)
    
# load StarkSDK library
libstark = load_library()    
class StarkSDK:
    @classmethod
    def set_log_level(cls, level):
        return libstark.stark_set_log_level(level)
    
    @staticmethod
    def dispose():
        ffi.dlclose(libstark)
        # os.kill(os.getpid(), signal.SIGKILL)

    @staticmethod
    def get_sdk_version():
        return ffi.string(libstark.stark_get_sdk_version()).decode("utf-8")
    @staticmethod
    def gen_msg_id():
        return libstark.stark_gen_msg_id()     

    @staticmethod
    def set_write_data_callback(cb):   
        StarkSDK.__on_write_data = cb
        libstark.stark_set_write_data_callback(StarkSDK.__on_write_data_internal);  

    @staticmethod
    @ffi.callback("int(char*, uint8_t*, int)")
    def __on_write_data_internal(uuid_ptr, data, length):
        if StarkSDK.__on_write_data is not None:
            # np_data = np.frombuffer(data, dtype=np.uint8)
            # return StarkSDK.__on_write_data(ffi.cast("char(*)", np_data.ctypes.data), len(np_data))
            value = ffi.buffer(data, length)[:]
            return StarkSDK.__on_write_data(value)
        return -1     

class StarkDeviceListener(abc.ABC):
    def on_error(self, error):
        pass  # print("on_error not implemented: %i" % error.code
        # sys.exit(1)
    def on_motorboard_info(self, motorboard_info):
        pass
    def on_hand_type(self, hand_type):
        pass
    def on_serialport_cfg(self, serialport_cfg):
        pass
    def on_voltage(self, voltage):
        pass
    def on_limit_current(self, limit_current):
        pass
    def on_force_level(self, force_level):
        pass
    def on_finger_status(self, finger_status):
        pass
    def on_finger_movement_status(self, finger_movement_status):
        pass
    def on_button_event(self, button_event):
        pass
    
class LogLevel(IntEnum):
    debug = 0
    info = 1
    warning = 2
    error = 3
    none = 4

class Connectivity(IntEnum):
    connecting = 0
    connected = 1
    disconnecting = 2
    disconnected = 3    

class StarkMessageId(IntEnum):
    MainBoard = 1
    MotorBoard = 2
    App = 3

class FingerId(IntEnum):
    thumb = 1
    thumb_aux = 2
    index = 3
    middle = 4
    ring = 5
    pinky = 6

class HandType(IntEnum):
    regular_right = 1
    regular_left = 2
    mini_right = 3
    mini_left = 4

class ForceLevel(IntEnum):
    small = 0
    normal = 1
    full = 2
class FingerMovementState(IntEnum):
    idle = 0
    running = 1
    protected_zone = 2
    stall = 3
class LedMode(IntEnum):
    off = 1
    keep = 2
    blink = 3
    one_shot = 4
    blink0_5hz = 5
    blink2hz = 6
class LedColor(IntEnum):
    unchanged = 0
    r = 1
    g = 2
    rg = 3
    b = 4
    rb = 5
    gb = 6
    rgb = 7    
class ButtonPressState(IntEnum):
    pressing = 1
    not_pressing = 2

class StarkErrorCode(IntEnum):
    none = 0
    unknown = -1
    invalid_params = -2
    invalid_data = -3
    bleDeviceUnreachable = -128
    bleDisabled = -129
    bleUnavailable = -130
    bleDataWriteFailure = -131
    device_not_connected = -160
    device_uuid_unavailable = -196
    # Stark error codes
    stark_emg_init = -1002
    stark_imu_init = -1003
    stark_ppg_init = -1004
    stark_battery_voltage = -1005
    stark_battery_temperature = -1006
    stark_hardware_version = -1007
    stark_flash_init = -1008
    stark_ble_init = -1009

# Wrapper objects
class StarkError:
    code = None
    message = None

    def __init__(self, code):
        self.code = code
        c_msg = libstark.stark_err_code_to_msg(code)
        self.message = ffi.string(c_msg).decode("utf-8")

class SerialPortCfg:  
    serial_device_id = None
    baudrate = None

    def __init__(self, c_cfg):
        self.serial_device_id = c_cfg.serial_device_id
        self.baudrate = c_cfg.baudrate
class MotorboardInfo:
    hand_type = None
    sn = None
    fw_version = None

    def __init__(self, c_info):
        self.hand_type = HandType(c_info.hand_type)
        self.sn = ffi.string(c_info.sn, 40).decode("utf-8")
        self.fw_version = ffi.string(c_info.fw_version, 40).decode("utf-8")

class StarkFingerStatus:
    finger_positions = None
    finger_speeds = None
    finger_currents = None
    finger_pwms = None

    def __init__(self, c_status):
        self.finger_positions = ffi.unpack(c_status.finger_positions, 6)
        self.finger_speeds = ffi.unpack(c_status.finger_speeds, 6)
        self.finger_currents = ffi.unpack(c_status.finger_currents, 6)
        self.finger_pwms = ffi.unpack(c_status.finger_pwms, 6)        

class LedInfo:
    led_color = None
    led_mode = None

    def __init__(self, c_info):
        self.led_color = LedColor(c_info.led_color)
        self.led_mode = LedMode(c_info.led_mode)

class ButtonPressEvent:
    timestamp = None
    button_id = None
    press_status = None

    def __init__(self, c_info):
        self.timestamp = c_info.timestamp
        self.button_id = c_info.button_id
        self.press_status = ButtonPressState(c_info.press_status)     

class StarkDevice(StarkDeviceListener):
    @classmethod
    def create_serial_device(cls, id, name):
        uuid = str(id)
        if uuid in cls._device_map:
            device = cls._device_map.get(uuid)
            return device
        device = StarkDevice(uuid, name, id)
        cls._device_map[uuid] = device
        
        device_ptr = libstark.stark_create_device(uuid.encode('utf-8'))
        if device_ptr is not ffi.NULL:
            StarkDevice._device_pointer_map[uuid] = device_ptr
        return device

    _device_pointer_map = {}
    _device_map = {}
    _config_response_callbacks = {}

    __address = 0
    __uuid = None
    __name = None
    __listener = None

    def __init__(self, uuid, name, address):
        self.__uuid = uuid
        self.__name = name
        self.__address = address

    @property
    def uuid(self):
        return self.__uuid

    @property
    def addr(self):
        return self.__address

    @property
    def name(self):
        return self.__name

    @property
    def firmware_revision(self):
        if self.__uuid in StarkDevice._device_pointer_map:
            firmware_rev = libstark.stark_get_firmware_revision(StarkDevice._device_pointer_map[self.__uuid])
            return ffi.string(firmware_rev, 64).decode("utf-8")
        print("Never connected to the device; firmware revision is not available")
        return None

    def did_receive_data(self, data):  
        if self.__uuid in StarkDevice._device_pointer_map:
            libstark.stark_did_receive_data(StarkDevice._device_pointer_map[self.__uuid], ffi.new("uint8_t[]", data), len(data))        

    def get_hand_type(self, cb=None):
        self.__on_hand_type = cb
        if self.__uuid in StarkDevice._device_pointer_map:
            libstark.stark_get_hand_type(StarkDevice._device_pointer_map[self.__uuid], StarkDevice.__on_hand_type_internal)

    def get_serialport_cfg(self, cb=None):
        self.__on_get_serialport_cfg = cb
        if self.__uuid in StarkDevice._device_pointer_map:
            libstark.stark_get_serialport_cfg(StarkDevice._device_pointer_map[self.__uuid], StarkDevice.__on_serialport_cfg_internal)

    def set_serialport_cfg(self, baudrate, cb=None):
        if self.__uuid in StarkDevice._device_pointer_map:
            libstark.stark_set_serialport_cfg(StarkDevice._device_pointer_map[self.__uuid], baudrate)           

    def set_serial_device_id(self, device_id, cb=None):
        if self.__uuid in StarkDevice._device_pointer_map:
            libstark.stark_set_serial_device_id(StarkDevice._device_pointer_map[self.__uuid], device_id)

    def get_motorboard_info(self, cb=None):
        self.__on_motorboard_info = cb
        if self.__uuid in StarkDevice._device_pointer_map:
            libstark.stark_get_motorboard_info(StarkDevice._device_pointer_map[self.__uuid], StarkDevice.__on_motorboard_info_internal)

    def get_voltage(self, cb=None):
        self.__on_voltage = cb
        if self.__uuid in StarkDevice._device_pointer_map:
            libstark.stark_get_voltage(StarkDevice._device_pointer_map[self.__uuid], StarkDevice.__on_voltage_internal)

    def get_max_current(self, cb=None):
        self.__on_limit_current = cb
        if self.__uuid in StarkDevice._device_pointer_map:
            libstark.stark_get_max_current(StarkDevice._device_pointer_map[self.__uuid], StarkDevice.__on_limit_current_internal)

    def set_max_current(self, max_current):
        if self.__uuid in StarkDevice._device_pointer_map:
            libstark.stark_set_max_current(StarkDevice._device_pointer_map[self.__uuid], max_current)

    def get_force_level(self, cb=None):
        self.__on_force_level = cb
        if self.__uuid in StarkDevice._device_pointer_map:
            libstark.stark_get_force_level(StarkDevice._device_pointer_map[self.__uuid], StarkDevice.__on_force_level_internal)                     

    def set_force_level(self, force_level):
        if self.__uuid in StarkDevice._device_pointer_map:
            libstark.stark_set_force_level(StarkDevice._device_pointer_map[self.__uuid], force_level)
    
    def get_finger_status(self, cb=None):
        self.__on_finger_status = cb
        if self.__uuid in StarkDevice._device_pointer_map:
            libstark.stark_get_finger_status(StarkDevice._device_pointer_map[self.__uuid], StarkDevice.__on_finger_status_internal)

    def get_finger_movement_status(self, cb=None):
        self.__on_finger_movement_status = cb
        if self.__uuid in StarkDevice._device_pointer_map:
            libstark.stark_get_finger_movement_status(StarkDevice._device_pointer_map[self.__uuid], StarkDevice.__on_finger_movement_status_internal)

    def reset_finger_positions(self):
        if self.__uuid in StarkDevice._device_pointer_map:
            libstark.stark_reset_finger_positions(StarkDevice._device_pointer_map[self.__uuid])

    def set_finger_position(self, finger_position):
        if self.__uuid in StarkDevice._device_pointer_map:
            libstark.stark_set_finger_position(StarkDevice._device_pointer_map[self.__uuid], finger_position)

    def set_finger_positions(self, finger_positions):
        if self.__uuid in StarkDevice._device_pointer_map:
            libstark.stark_set_finger_positions(StarkDevice._device_pointer_map[self.__uuid], ffi.new("int[]", finger_positions))

    def set_finger_speed(self, finger_speed):
        if self.__uuid in StarkDevice._device_pointer_map:
            libstark.stark_set_finger_speed(StarkDevice._device_pointer_map[self.__uuid], finger_speed)

    def set_finger_speeds(self, finger_speeds):
        if self.__uuid in StarkDevice._device_pointer_map:
            libstark.stark_set_finger_speeds(StarkDevice._device_pointer_map[self.__uuid], ffi.new("int[]", finger_speeds))

    def set_led_info(self, led_mode, led_color):
        if self.__uuid in StarkDevice._device_pointer_map:
            libstark.stark_set_led_info(StarkDevice._device_pointer_map[self.__uuid], led_mode, led_color)

    def get_button_event(self, cb=None):
        self.__on_button_event = cb
        if self.__uuid in StarkDevice._device_pointer_map:
            libstark.stark_get_button_event(StarkDevice._device_pointer_map[self.__uuid], StarkDevice.__on_button_event_internal)

    @staticmethod
    @ffi.callback("void(char*, MotorboardInfo*)")         
    def __on_motorboard_info_internal(uuid_ptr, c_info):
        uuid = ffi.string(uuid_ptr, 40).decode("utf-8")
        if uuid in StarkDevice._device_map:
            device = StarkDevice._device_map[uuid]
            if device.__on_motorboard_info is not None:
                device.__on_motorboard_info(MotorboardInfo(c_info))
            if device.__listener is not None:
                device.__listener.on_motorboard_info(MotorboardInfo(c_info))
        else:
            fatal_error("__on_motorboard_info_internal:device unavailable for:" + uuid)  

    @staticmethod
    @ffi.callback("void(char*, int)")
    def __on_hand_type_internal(uuid_ptr, hand_type):
        uuid = ffi.string(uuid_ptr, 40).decode("utf-8")
        if uuid in StarkDevice._device_map:
            device = StarkDevice._device_map[uuid]
            if device.__on_hand_type is not None:
                device.__on_hand_type(hand_type)
            if device.__listener is not None:
                device.__listener.on_hand_type(hand_type)
        else:
            fatal_error("__on_hand_type_internal:device unavailable for:" + uuid)

    @staticmethod
    @ffi.callback("void(char*, SerialPortCfg*)")
    def __on_serialport_cfg_internal(uuid_ptr, c_cfg):
        uuid = ffi.string(uuid_ptr, 40).decode("utf-8")
        if uuid in StarkDevice._device_map:
            device = StarkDevice._device_map[uuid]
            if device.__on_serialport_cfg is not None:
                device.__on_serialport_cfg(SerialPortCfg(c_cfg))
            if device.__listener is not None:
                device.__listener.on_serialport_cfg(SerialPortCfg(c_cfg))
        else:
            fatal_error("__on_serialport_cfg_internal:device unavailable for:" + uuid)                

    @staticmethod
    @ffi.callback("void(char*, float)")
    def __on_voltage_internal(uuid_ptr, voltage):
        uuid = ffi.string(uuid_ptr, 40).decode("utf-8")
        if uuid in StarkDevice._device_map:
            device = StarkDevice._device_map[uuid]
            if device.__on_voltage is not None:
                device.__on_voltage(voltage)
            if device.__listener is not None:
                device.__listener.on_voltage(voltage)
        else:
            fatal_error("__on_voltage_internal:device unavailable for:" + uuid)

    @staticmethod
    @ffi.callback("void(char*, int)")
    def __on_limit_current_internal(uuid_ptr, limit_current):
        uuid = ffi.string(uuid_ptr, 40).decode("utf-8")
        if uuid in StarkDevice._device_map:
            device = StarkDevice._device_map[uuid]
            if device.__on_limit_current is not None:
                device.__on_limit_current(limit_current)
            if device.__listener is not None:
                device.__listener.on_limit_current(limit_current)
        else:
            fatal_error("__on_limit_current_internal:device unavailable for:" + uuid)

    @staticmethod
    @ffi.callback("void(char*, int)")
    def __on_force_level_internal(uuid_ptr, force_level):
        uuid = ffi.string(uuid_ptr, 40).decode("utf-8")
        if uuid in StarkDevice._device_map:
            device = StarkDevice._device_map[uuid]
            if device.__on_force_level is not None:
                device.__on_force_level(force_level)
            if device.__listener is not None:
                device.__listener.on_force_level(force_level)
        else:
            fatal_error("__on_force_level_internal:device unavailable for:" + uuid)

    @staticmethod
    @ffi.callback("void(char*, StarkFingerStatus**)")
    def __on_finger_status_internal(uuid_ptr, c_status):
        uuid = ffi.string(uuid_ptr, 40).decode("utf-8")
        if uuid in StarkDevice._device_map:
            device = StarkDevice._device_map[uuid]
            statuses = []
            for i in range(6):
                statuses.append(StarkFingerStatus(c_status[i]))
            if device.__on_finger_status is not None:
                device.__on_finger_status(statuses)        
            if device.__listener is not None:
                device.__listener.on_finger_status(statuses)
        else:
            fatal_error("__on_finger_status_internal:device unavailable for:" + uuid)    

    @staticmethod
    @ffi.callback("void(char*, int*)")
    def __on_finger_movement_status_internal(uuid_ptr, c_status):
        uuid = ffi.string(uuid_ptr, 40).decode("utf-8")
        if uuid in StarkDevice._device_map:
            device = StarkDevice._device_map[uuid]
            statuses = []
            for i in range(6):
                statuses.append(FingerMovementState(c_status[i]))
            if device.__on_finger_movement_status is not None:
                device.__on_finger_movement_status(statuses)        
            if device.__listener is not None:
                device.__listener.on_finger_movement_status(statuses)
        else:
            fatal_error("__on_finger_movement_status_internal:device unavailable for:" + uuid)        

    @staticmethod
    @ffi.callback("void(char*, LedInfo*)")
    def __on_led_info_internal(uuid_ptr, c_info):
        uuid = ffi.string(uuid_ptr, 40).decode("utf-8")
        if uuid in StarkDevice._device_map:
            device = StarkDevice._device_map[uuid]
            if device.__on_led_info is not None:
                device.__on_led_info(LedInfo(c_info))
            if device.__listener is not None:
                device.__listener.on_led_info(LedInfo(c_info))
        else:
            fatal_error("__on_led_info_internal:device unavailable for:" + uuid)

    @staticmethod
    @ffi.callback("void(char*, ButtonPressEvent*)")
    def __on_button_event_internal(uuid_ptr, c_info):
        uuid = ffi.string(uuid_ptr, 40).decode("utf-8")
        if uuid in StarkDevice._device_map:
            device = StarkDevice._device_map[uuid]
            if device.__on_button_event is not None:
                device.__on_button_event(ButtonPressEvent(c_info))
            if device.__listener is not None:
                device.__listener.on_button_event(ButtonPressEvent(c_info))
        else:
            fatal_error("__on_button_event_internal:device unavailable for:" + uuid)          

