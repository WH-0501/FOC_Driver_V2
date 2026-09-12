"""CANFD 从机通讯协议常量定义，与固件 can_mapp.h / motor.h 对齐。"""

from enum import IntEnum


class CMD(IntEnum):
    """功能码定义。"""

    # 主机 -> 从机（广播 0x00 或单播 Node-ID）
    SYNC = 0x00
    TAR_POS = 0x01
    TAR_VEL = 0x02
    TAR_CURR = 0x03
    TAR_POS_VEL = 0x12
    TAR_POS_CURR = 0x13
    TAR_VEL_CURR = 0x23
    TAR_POS_VEL_CURR = 0x30
    FAULT_RESET = 0x39

    ENABLE = 0x40
    CONTROL_MODE = 0x41
    SET_POS_PID = 0x42
    SET_VEL_PID = 0x43
    SET_CUR_PID = 0x44
    SET_STIFF_DAMP = 0x45
    SET_PROTECT = 0x46
    GET_POS_PID = 0x47
    GET_VEL_PID = 0x48
    GET_CUR_PID = 0x49
    GET_STIFF_DAMP = 0x4A
    GET_PROTECT = 0x4B
    SET_NODE_ID = 0x4C
    SAVE_PARAMS = 0x4D
    RESET_TO_FACTORY = 0x4E
    CLEAR_FAULT = 0x4F
    SET_ORIGIN = 0x50
    ALIGN_ELECTRIC_ANGLE = 0xAB

    # IAP（仲裁 ID = 0x7FF 或 Node-ID）
    IAP_ENTER = 0xF8
    IAP_START = 0xF9
    IAP_DATA = 0xFA
    IAP_END = 0xFB
    IAP_STATUS = 0xFC
    IAP_FW_VERSION = 0xFD
    IAP_VERIFY = 0xFE
    IAP_RESET = 0xFF

    # 从机 -> 主机周期上报
    PERIOD_REPORT = 0x00


class ControlMode(IntEnum):
    """与固件 datatypes.h control_mode_t 一致。"""

    IDLE = 0
    TORQUE = 1
    SPEED = 2
    POSITION = 3
    CSP = 4
    CSV = 5
    CST = 6
    MIT = 7
    HOMING = 8
    TORQUE_OPEN_LOOP = 9
    SPEED_OPEN_LOOP = 10
    POSITION_OPEN_LOOP = 11
    VOLTAGE = 12
    RESERVED = 13


MODE_NAMES = {
    ControlMode.IDLE: "空闲 (IDLE)",
    ControlMode.TORQUE: "转矩 (TORQUE)",
    ControlMode.SPEED: "速度环 (SPEED)",
    ControlMode.POSITION: "位置环 (POSITION)",
    ControlMode.CSP: "周期位置 (CSP)",
    ControlMode.CSV: "周期速度 (CSV)",
    ControlMode.CST: "周期转矩 (CST)",
    ControlMode.MIT: "MIT 阻抗",
    ControlMode.HOMING: "回零 (HOMING)",
    ControlMode.TORQUE_OPEN_LOOP: "转矩开环 (TORQUE_OL)",
    ControlMode.SPEED_OPEN_LOOP: "速度开环 (SPEED_OL)",
    ControlMode.POSITION_OPEN_LOOP: "位置开环 (POSITION_OL)",
    ControlMode.VOLTAGE: "电压模式 (VOLTAGE)",
    ControlMode.RESERVED: "保留 (RESERVED)",
}

# 控制面板可选模式（不含 RESERVED）
SELECTABLE_CONTROL_MODES = tuple(m for m in ControlMode if m is not ControlMode.RESERVED)


def mode_needs_position(mode: ControlMode) -> bool:
    return mode in (
        ControlMode.POSITION,
        ControlMode.CSP,
        ControlMode.POSITION_OPEN_LOOP,
        ControlMode.MIT,
    )


def mode_needs_velocity(mode: ControlMode) -> bool:
    return mode in (
        ControlMode.SPEED,
        ControlMode.CSV,
        ControlMode.SPEED_OPEN_LOOP,
        ControlMode.MIT,
    )


def mode_needs_current(mode: ControlMode) -> bool:
    return mode in (
        ControlMode.TORQUE,
        ControlMode.CST,
        ControlMode.TORQUE_OPEN_LOOP,
        ControlMode.MIT,
    )


class MotorError(IntEnum):
    NORMAL = 0
    OTP_ERR = 1
    OCP_ERR = 2
    UVLO_ERR = 3
    OVP_ERR = 4
    NFAULT_ERR = 5
    ENCODER_ERR = 6
    PHASE_LOSS_ERR = 7


ERROR_NAMES = {
    MotorError.NORMAL: "正常",
    MotorError.OTP_ERR: "过温",
    MotorError.OCP_ERR: "过流",
    MotorError.UVLO_ERR: "欠压",
    MotorError.OVP_ERR: "过压",
    MotorError.NFAULT_ERR: "驱动故障",
    MotorError.ENCODER_ERR: "编码器故障",
    MotorError.PHASE_LOSS_ERR: "缺相",
}

# 广播地址与 IAP 地址
CAN_BROADCAST_ID = 0x00
CAN_IAP_ID = 0x7FF

# PID Q12 格式系数（与固件 PID_FORMAT 一致）
PID_FORMAT = 4096.0
PID_RAW_MAX = 65535
PID_VALUE_MAX = PID_RAW_MAX / PID_FORMAT  # 单参数 float 上限，约 15.9998

# 位置 Q7 缩放：V1 为 °×128，V2 为 rad×128，均为 raw/128
POS_SCALE = 128.0

# 协议数据域长度（不含 CNT + CMD）
CMD_DATA_LEN = {
    CMD.SYNC: 0,
    CMD.ENABLE: 1,
    CMD.CONTROL_MODE: 1,
    CMD.SET_POS_PID: 6,
    CMD.SET_VEL_PID: 6,
    CMD.SET_CUR_PID: 6,
    CMD.SET_STIFF_DAMP: 4,
    CMD.SET_PROTECT: 6,
    CMD.GET_POS_PID: 0,
    CMD.GET_VEL_PID: 0,
    CMD.GET_CUR_PID: 0,
    CMD.GET_STIFF_DAMP: 0,
    CMD.GET_PROTECT: 0,
    CMD.SET_NODE_ID: 2,
    CMD.SAVE_PARAMS: 0,
    CMD.RESET_TO_FACTORY: 0,
    CMD.CLEAR_FAULT: 0,
    CMD.FAULT_RESET: 0,
    CMD.SET_ORIGIN: 0,
    CMD.ALIGN_ELECTRIC_ANGLE: 0,
    CMD.IAP_ENTER: 0,
    CMD.IAP_END: 0,
    CMD.IAP_STATUS: 0,
    CMD.IAP_FW_VERSION: 0,
    CMD.IAP_VERIFY: 0,
    CMD.IAP_RESET: 0,
}

# CAN FD 标准 DLC 对应的数据长度
CANFD_DLC_LENGTHS = (0, 1, 2, 3, 4, 5, 6, 7, 8, 12, 16, 20, 24, 32, 48, 64)

# IAP staging 区最大固件体长度（与 iap_upgrade.h 一致）
IAP_MAX_BODY_SIZE = 61 * 1024 - 32  # 62432
