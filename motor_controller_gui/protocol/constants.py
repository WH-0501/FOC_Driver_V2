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
    COMMAND_ACK = 0x7E
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
    GET_MOTOR_PARAM = 0x53
    GET_SPEED_LIMITS = 0xB5
    SET_CURRENT_SOFT_SPEED = 0xB6
    VERIFY_ELECTRICAL = 0xB1
    VERIFY_SAMPLE = 0xB2
    ALIGN_ELECTRIC_ANGLE = 0xAB
    KTH71_CAL_START = 0xAD
    KTH71_CAL_STATUS = 0xAE
    KTH71_CAL_STOP = 0xAF
    KTH71_CAL_TRIGGER = 0xB0

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


class ElecAlignMethod(IntEnum):
    """与固件 foc_calibration.h cal_elec_align_method_t 一致。"""

    UD = 0
    UQ = 1
    DUAL_POINT = 2
    DUAL_POINT_UQ = 3
    HFI = 4


ELEC_ALIGN_METHOD_NAMES = {
    ElecAlignMethod.UD: "Ud 单点 (d轴电压锁定)",
    ElecAlignMethod.UQ: "Uq 单点 (dex-con)",
    ElecAlignMethod.DUAL_POINT: "Ud 双点 (静摩擦对消)",
    ElecAlignMethod.DUAL_POINT_UQ: "Uq 双点",
    ElecAlignMethod.HFI: "HFI 高频注入",
}

ELEC_ALIGN_METHOD_HINTS = {
    ElecAlignMethod.UD: "d 轴锁定：offset = raw − θ_open；可与 Ud 双点对照。",
    ElecAlignMethod.UQ: "q 轴锁定（dex-con）：offset = raw；与 Ud 是两套零位，数值不同是正常的。",
    ElecAlignMethod.DUAL_POINT: "Ud 双点静摩擦对消，offset 公式同 Ud 单点。",
    ElecAlignMethod.DUAL_POINT_UQ: "Uq 双点，offset = raw − (θ_open − π/2)。",
    ElecAlignMethod.HFI: "HFI 扫角后按 Ud 公式写 offset。",
}

# 0xAB 应答：14 B，电角 int32 大端，1 LSB = 1e-6 rad（与固件 CAN_FD_ALIGN_* 一致）
ALIGN_REPLY_LEN = 14
MOTOR_PARAM_REPLY_LEN = 16
MOTOR_PARAM_FLAG_ALIGN_DONE = 1 << 0
ALIGN_ANGLE_SCALE = 1_000_000.0
ALIGN_FLAG_OK = 1 << 0
ALIGN_FLAG_FLIPPED = 1 << 1
ALIGN_FLAG_ENGAGED = 1 << 2
ALIGN_FLAG_DONE = 1 << 3


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
    """V2 周期保护子码（BIT7=0），与 can_fd_codec_fault_to_error / V2_PROTECTION_WIRE_ERROR_NAMES 一致。"""

    NORMAL = 0
    HW_OCP = 0x01
    SW_OCP = 0x02
    HW_OVP = 0x03
    SW_OVP = 0x04
    HW_UVP = 0x05
    SW_UVP = 0x06
    PHASE_LOSS = 0x07
    ENCODER = 0x08
    OVERTEMP = 0x09
    COMM = 0x0A
    POS_CMD = 0x0B
    VEL_CMD = 0x0C
    CURR_CMD = 0x0D
    HOMING_TIMEOUT = 0x0E
    FRAME_LOSS = 0x0F
    SHAFT_OVERSPEED = 0x10
    ALIGN_FAILED = 0x11
    CALIBRATION = 0x12
    CTRL_SATURATION = 0x13
    WATCHDOG = 0x14
    PARAM = 0x15
    SHORT_CIRCUIT = 0x16
    STALL = 0x17
    CURR_IMBALANCE = 0x18
    CURR_SENSOR = 0x19
    STORAGE = 0x1A
    BRAKE = 0x1B
    DESAT = 0x1C
    GATE_NFAULT = 0x1D
    LOGIC_UV = 0x1E
    PHASE_W_OPEN = 0x1F
    USER_CMD = 0x20
    UNCLASSIFIED = 0x7F


class ErrorCode(IntEnum):
    """与固件 error_types.h error_t 一致（API 返回值 / MC 命令应答）。"""

    ERR_NONE = 0
    ERR_NULL = 1
    ERR_PARAM = 2
    ERR_STATE = 3
    ERR_TIMEOUT = 4
    ERR_NOT_IMPL = 5
    ERR_COMM = 6
    ERR_STORAGE = 7
    ERR_SENSOR = 8
    ERR_FAIL = 9

    ERR_ALIGIN_SPREAD_OUT = 10
    ERR_ALIGIN_ROTOR_NOT_ENGAGED = 11
    ERR_ALIGIN_FAILED = 12

    ERR_FAULT_LATCHED = 20
    ERR_OFFSET_CAL = 21
    ERR_CTRL_MODE = 22
    ERR_CALIBRATING = 23
    ERR_CAL_TYPE = 24
    ERR_NO_FAULT = 25


API_ERROR_NAMES = {
    ErrorCode.ERR_NONE: "成功",
    ErrorCode.ERR_NULL: "空指针",
    ErrorCode.ERR_PARAM: "参数非法",
    ErrorCode.ERR_STATE: "状态不允许",
    ErrorCode.ERR_TIMEOUT: "超时",
    ErrorCode.ERR_NOT_IMPL: "未实现",
    ErrorCode.ERR_COMM: "通讯错误",
    ErrorCode.ERR_STORAGE: "存储错误",
    ErrorCode.ERR_SENSOR: "传感器错误",
    ErrorCode.ERR_FAIL: "未分类错误",
    ErrorCode.ERR_ALIGIN_SPREAD_OUT: "对齐采样分散度过大",
    ErrorCode.ERR_ALIGIN_ROTOR_NOT_ENGAGED: "转子未吸合",
    ErrorCode.ERR_ALIGIN_FAILED: "对齐失败",
    ErrorCode.ERR_FAULT_LATCHED: "故障已锁存",
    ErrorCode.ERR_OFFSET_CAL: "电流零偏未校准",
    ErrorCode.ERR_CTRL_MODE: "控制模式不允许",
    ErrorCode.ERR_CALIBRATING: "正在校准",
    ErrorCode.ERR_CAL_TYPE: "校准类型非法",
    ErrorCode.ERR_NO_FAULT: "无锁存故障",
}


def api_error_name(code: int) -> str:
    try:
        return API_ERROR_NAMES[ErrorCode(code)]
    except (ValueError, KeyError):
        return f"未知错误码"


def format_api_error_detail(code: int) -> str:
    """格式化 API 错误，含 error_t 数值便于对照固件 error_types.h。"""
    return f"{api_error_name(code)} (error_t={code})"


# B6 扩展：4 字节 = 输出轴软限速 RPM(u16) + 标志(u16)，bit0 硬超速保护使能
SPEED_LIMIT_FLAG_HARD_OVERSPEED = 0x0001

KTH71_ANLC_STATUS_NAMES = {
    0: "未开始",
    1: "校准中",
    2: "失败",
    3: "完成",
    0xFF: "读取失败",
}

ERROR_NAMES = {
    MotorError.NORMAL: "正常",
    MotorError.HW_OCP: "硬件过流",
    MotorError.SW_OCP: "软件过流",
    MotorError.HW_OVP: "硬件过压",
    MotorError.SW_OVP: "软件过压",
    MotorError.HW_UVP: "硬件欠压",
    MotorError.SW_UVP: "软件欠压",
    MotorError.PHASE_LOSS: "缺相",
    MotorError.ENCODER: "编码器故障",
    MotorError.OVERTEMP: "软件、硬件过温",
    MotorError.COMM: "通信故障",
    MotorError.POS_CMD: "位置命令异常",
    MotorError.VEL_CMD: "速度命令异常",
    MotorError.CURR_CMD: "电流命令异常",
    MotorError.HOMING_TIMEOUT: "回零超时 (10s)",
    MotorError.FRAME_LOSS: "丢帧",
    MotorError.SHAFT_OVERSPEED: "电机轴硬超速",
    MotorError.ALIGN_FAILED: "电角对齐失败",
    MotorError.CALIBRATION: "校准失败",
    MotorError.CTRL_SATURATION: "控制饱和",
    MotorError.WATCHDOG: "控制看门狗",
    MotorError.PARAM: "参数非法",
    MotorError.SHORT_CIRCUIT: "输出短路",
    MotorError.STALL: "堵转",
    MotorError.CURR_IMBALANCE: "三相电流不平衡",
    MotorError.CURR_SENSOR: "电流采样异常",
    MotorError.STORAGE: "存储异常",
    MotorError.BRAKE: "制动异常",
    MotorError.DESAT: "去饱和保护",
    MotorError.GATE_NFAULT: "驱动 nFAULT 有效",
    MotorError.LOGIC_UV: "逻辑电源欠压",
    MotorError.PHASE_W_OPEN: "W 相开路",
    MotorError.USER_CMD: "用户故障指令",
    MotorError.UNCLASSIFIED: "未分类保护故障",
}

# 广播地址与 IAP 地址
CAN_BROADCAST_ID = 0x00
CAN_IAP_ID = 0x7FF

# PID Q12（V1/V2 FOC：K = raw / PID_FORMAT，与固件 CAN_FD_PID_FORMAT 一致）
PID_FORMAT = 4096.0
PID_RAW_MAX = 65535
PID_VALUE_MAX = PID_RAW_MAX / PID_FORMAT  # 单参数 K 上限，约 15.9998
PID_K_MIN_STEP = 1.0 / PID_FORMAT  # 最小非零 K 步进（raw 差 1）

# 电流环 Ki：线值/config 为 Ki_phys / CUR_PID_KI_SCALE（与固件 FOC_PID_CUR_KI_SCALE 一致）
CUR_PID_KI_SCALE = 10000.0
CUR_PID_KI_PHYS_MAX = PID_VALUE_MAX * CUR_PID_KI_SCALE

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
    CMD.GET_MOTOR_PARAM: 0,
    CMD.SET_NODE_ID: 2,
    CMD.SAVE_PARAMS: 0,
    CMD.RESET_TO_FACTORY: 0,
    CMD.CLEAR_FAULT: 0,
    CMD.FAULT_RESET: 0,
    CMD.SET_ORIGIN: 0,
    CMD.ALIGN_ELECTRIC_ANGLE: 1,
    CMD.KTH71_CAL_START: 0,
    CMD.KTH71_CAL_STATUS: 0,
    CMD.KTH71_CAL_STOP: 0,
    CMD.KTH71_CAL_TRIGGER: 0,
    CMD.IAP_ENTER: 0,
    CMD.IAP_END: 0,
    CMD.IAP_STATUS: 0,
    CMD.IAP_FW_VERSION: 0,
    CMD.IAP_VERIFY: 0,
    CMD.IAP_RESET: 0,
}

# CAN FD 标准 DLC 对应的数据长度
CANFD_DLC_LENGTHS = (0, 1, 2, 3, 4, 5, 6, 7, 8, 12, 16, 20, 24, 32, 48, 64)

# IAP staging 区（dex-con-firmware-v5 / iap.c 与 iap_upgrade.h）
IAP_MAX_BODY_SIZE = 61 * 1024 - 32  # 62432，仅 App 体
IAP_MAX_FILE_SIZE = 61 * 1024  # 含 32B ASIX 头的整包上限
IAP_V5_SUCCESS = 0x80  # dex-con IAPError_t：ERROR_IAP_SUCCESS
IAP_V5_MIN_START_PAYLOAD = 32  # IAP_Process START：data[2..] 至少 32B ASIX 头

# IAP V5 帧格式（对齐 USB_CANFD CSV / gen_iap_efw.py）
IAP_V5_CHUNK_SIZE = 62  # START/DATA 每帧固件字节数（64B 帧 = CNT+CMD+62）
IAP_V5_FRAME_SIZE = 64
IAP_V5_PAD_BYTE = 0xFF

# IAP 时序（秒；对齐 dex-con iap.c + CAN-FD通信协议.md）
# START 内同步擦除，完成后 FC(state=2,err=0x80)；注释「应答后才开始下一帧接收」
IAP_START_ERASE_WAIT_S = 120.0  # 等待擦除 FC 上限
IAP_START_MIN_RESPONSE_WAIT_S = 0.5  # 此前忽略擦除过程中的瞬时 FC
IAP_START_ERASE_POLL_S = 2.0  # 保留常量，擦除等待期间不主动发 FC 查询
IAP_START_DATA_DEADLINE_S = 2.5  # 收到 FC 后须尽快发 DATA（3s 超时从 START 起算且含擦除）
IAP_START_FC_POLL_S = 0.02  # 等待擦除 FC 轮询间隔（须快，擦除耗掉大部分 3s 窗口）
IAP_START_DATA_BURST_FRAMES = 32  # 收到 FC 后立即连发的 DATA 帧数
IAP_FIRMWARE_TIMEOUT_MS = 3000  # dex-con iap.c IAP_TIMEOUT
IAP_V5_RECOVERY_WAIT_S = 15.0  # 失败后等待再重试
IAP_V5_IDLE_WAIT_S = 90.0  # 轮询等到 state=0 & error=0x80（0x8F 丢帧不算空闲）
IAP_STATUS_QUERY_TIMEOUT_S = 0.5
IAP_POST_VERIFY_QUERY_S = 5.0
IAP_DATA_FRAME_INTERVAL_S = 0.001  # DATA 帧间隔
IAP_START_MIN_FIRMWARE_CHUNK = 25  # 8 + 25 > 32（协议文档 IAP_START 载荷 >32）
IAP_ASIX_SIGNATURE = b"ASIX"
IAP_ASIX_HEADER_SIZE = 32  # 与 iap.c / Bootloader 一致；START 起整包写入 staging
