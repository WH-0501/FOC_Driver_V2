"""固件 CAN 协议版本：V1.0.0（旧版模式枚举）与 V2.0.0（datatypes.h）。"""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum, IntEnum
from typing import Dict, Tuple

from .constants import (
    ERROR_NAMES,
    ControlMode,
    MODE_NAMES,
    SELECTABLE_CONTROL_MODES,
    mode_needs_current as _v2_needs_current,
    mode_needs_position as _v2_needs_position,
    mode_needs_velocity as _v2_needs_velocity,
)


class ProtocolVersion(str, Enum):
    V1_0_0 = "1.0.0"
    V2_0_0 = "2.0.0"


DEFAULT_PROTOCOL_VERSION = ProtocolVersion.V2_0_0


class ControlModeV1(IntEnum):
    """V1.0.0 固件 control_mode_t。"""

    PP = 0
    CSP = 1
    CST = 2
    HM = 3
    CSV = 4
    MIT = 5


@dataclass(frozen=True)
class ModeOption:
    wire: int
    label: str


V1_MODE_OPTIONS: Tuple[ModeOption, ...] = (
    ModeOption(ControlModeV1.PP, "PP (位置)"),
    ModeOption(ControlModeV1.CSP, "CSP (周期位置)"),
    ModeOption(ControlModeV1.CST, "CST (周期转矩)"),
    ModeOption(ControlModeV1.HM, "HM (回零)"),
    ModeOption(ControlModeV1.CSV, "CSV (周期速度)"),
    ModeOption(ControlModeV1.MIT, "MIT 阻抗"),
)

V1_MODE_NAMES = {opt.wire: opt.label for opt in V1_MODE_OPTIONS}

# V1.0.0 周期上报错误字节：BIT7=0 保护错误，BIT7=1 IAP 错误；BIT6~0 为子码
V1_PROTECTION_ERROR_NAMES: Dict[int, str] = {
    0x00: "正常",
    0x01: "硬件过流",
    0x02: "软件过流",
    0x03: "硬件过压",
    0x04: "软件过压",
    0x05: "硬件欠压",
    0x06: "软件欠压",
    0x07: "缺相",
    0x08: "编码器故障",
    0x09: "软件、硬件过温",
    0x0A: "通信故障",
    0x0B: "位置命令异常",
    0x0C: "速度命令异常",
    0x0D: "电流命令异常",
    0x0E: "回零超时 (10s)",
    0x0F: "丢帧",
}

V1_IAP_ERROR_NAMES: Dict[int, str] = {
    0x00: "正常",
    0x01: "无效命令",
    0x02: "无效参数",
    0x03: "Flash 擦除错误",
    0x04: "Flash 写入错误",
    0x05: "Flash 验证错误",
    0x06: "CRC 校验错误",
    0x07: "超时错误",
    0x08: "设备忙",
    0x09: "不支持操作",
    0x0A: "通信错误",
    0x0B: "响应超时 (3s)",
    0x0C: "拒绝升级",
    0x0D: "内存分配失败",
    0x0E: "Flash 切换错误",
}

V1_IAP_STATUS_NAMES: Dict[int, str] = {
    0: "空闲状态",
    1: "准备升级",
    2: "接收固件",
    3: "验证固件",
    4: "编程中",
    5: "完成",
    6: "错误",
}

# V2 / 当前 FOC 固件 iap_status_code_t
V2_IAP_STATUS_NAMES: Dict[int, str] = {
    0: "空闲",
    1: "就绪",
    2: "已验证",
    3: "错误",
}

V2_IAP_ERROR_NAMES: Dict[int, str] = {
    0: "正常",
    1: "忙",
    2: "大小错误",
    3: "擦除失败",
    4: "写入失败",
    5: "CRC 校验失败",
    6: "状态错误",
    7: "溢出",
    8: "数据不完整",
}

# dex-con-firmware-v5 motor_control/project/IAP/iap.h
V5_IAP_STATUS_NAMES: Dict[int, str] = {
    0: "空闲",
    1: "准备升级",
    2: "接收固件",
    3: "验证固件",
    4: "编程中",
    5: "完成",
    6: "错误",
}

V5_IAP_ERROR_NAMES: Dict[int, str] = {
    0x80: "成功",
    0x81: "无效命令",
    0x82: "无效参数",
    0x83: "Flash 擦除错误",
    0x84: "Flash 写入错误",
    0x85: "Flash 验证错误",
    0x86: "CRC 校验错误",
    0x87: "超时错误",
    0x88: "设备忙",
    0x89: "不支持操作",
    0x8A: "通信错误",
    0x8B: "响应超时 (3s)",
    0x8C: "拒绝升级",
    0x8D: "内存分配失败",
    0x8E: "Flash 切换错误",
    0x8F: "丢帧",
}

# V1 默认模式：CSV（周期速度）
V1_DEFAULT_WIRE = int(ControlModeV1.CSV)
# V2 默认模式：速度开环
V2_DEFAULT_WIRE = int(ControlMode.SPEED_OPEN_LOOP)


def parse_protocol_version(value: str) -> ProtocolVersion:
    for item in ProtocolVersion:
        if item.value == value or item.name == value:
            return item
    raise ValueError(f"未知协议版本: {value}")


def selectable_mode_options(version: ProtocolVersion) -> Tuple[ModeOption, ...]:
    if version == ProtocolVersion.V1_0_0:
        return V1_MODE_OPTIONS
    return tuple(ModeOption(int(mode), MODE_NAMES[mode]) for mode in SELECTABLE_CONTROL_MODES)


def default_mode_wire(version: ProtocolVersion) -> int:
    return V1_DEFAULT_WIRE if version == ProtocolVersion.V1_0_0 else V2_DEFAULT_WIRE


def parse_status_byte(version: ProtocolVersion, status: int) -> Tuple[int, bool, int]:
    """解析周期上报状态字节：高 4 位使能，低 4 位模式。"""
    mode = status & 0x0F
    enable_nibble = (status >> 4) & 0x0F
    if version == ProtocolVersion.V1_0_0:
        enable = enable_nibble == 1
    else:
        enable = enable_nibble != 0
    return mode, enable, enable_nibble


def parse_error_byte(version: ProtocolVersion, error: int) -> Dict[str, object]:
    """按协议版本解析错误字节。"""
    if version == ProtocolVersion.V1_0_0:
        is_iap = bool(error & 0x80)
        subcode = error & 0x7F
        if is_iap:
            name = V1_IAP_ERROR_NAMES.get(subcode, f"IAP 未知 (0x{subcode:02X})")
            error_type = "IAP"
        else:
            name = V1_PROTECTION_ERROR_NAMES.get(subcode, f"未知 (0x{subcode:02X})")
            error_type = "保护"
        if subcode == 0 and not is_iap:
            display = "正常"
        elif subcode == 0 and is_iap:
            display = f"[{error_type}] 正常"
        else:
            display = f"[{error_type}] {name}"
        return {
            "error_code": error,
            "error_subcode": subcode,
            "error_is_iap": is_iap,
            "error_type": error_type,
            "error_name": display,
        }

    name = ERROR_NAMES.get(error, f"未知 ({error})")
    return {
        "error_code": error,
        "error_subcode": error,
        "error_is_iap": False,
        "error_type": "保护",
        "error_name": name,
    }


def iap_status_name(version: ProtocolVersion, status: int) -> str:
    if version == ProtocolVersion.V1_0_0:
        return V1_IAP_STATUS_NAMES.get(status, f"未知 ({status})")
    return V2_IAP_STATUS_NAMES.get(status, f"未知 ({status})")


def iap_error_name(version: ProtocolVersion, error: int) -> str:
    if version == ProtocolVersion.V1_0_0:
        return V1_IAP_ERROR_NAMES.get(error, f"未知 (0x{error:02X})")
    return V2_IAP_ERROR_NAMES.get(error, f"未知 ({error})")


def iap_wire_uses_dex_v5_format(status: int, error: int) -> bool:
    """dex-con IAP：ERROR_IAP_SUCCESS=0x80，错误码 0x81~0x8F。"""
    if error == 0x80:
        return True
    if 0x81 <= error <= 0x8F:
        return True
    return False


def iap_is_dex_v5_success(error: int) -> bool:
    return error == 0x80


def iap_is_error_state(version: ProtocolVersion, status: int, error: int = 0) -> bool:
    if iap_wire_uses_dex_v5_format(status, error):
        return status == 6 or not iap_is_dex_v5_success(error)
    if iap_wire_uses_v1_format(status, error):
        return status >= 6
    if version == ProtocolVersion.V1_0_0:
        return status >= 6
    return status == 3


def iap_is_ready_state(version: ProtocolVersion, status: int) -> bool:
    return status == 1


def iap_is_receiving_state(status: int) -> bool:
    """dex-con START 成功后 state=IAP_STATE_RECEIVING (2)。"""
    return status == 2


def iap_is_verified_state(version: ProtocolVersion, status: int) -> bool:
    if version == ProtocolVersion.V1_0_0:
        return status in (5, 2)
    return status == 2


def iap_is_complete_state(status: int) -> bool:
    """dex-con VERIFY 成功后 state=IAP_STATE_COMPLETE (5)。"""
    return status == 5


def iap_wire_uses_v1_format(status: int, error: int) -> bool:
    """IAP 0xFC 应答与运动控制协议版本无关；V1 设备常用 status>=6 或 error bit7。"""
    if iap_wire_uses_dex_v5_format(status, error):
        return False
    return status >= 6 or bool(error & 0x80) or error > 8


def parse_iap_status_payload(
    version: ProtocolVersion, payload: bytes
) -> Dict[str, object]:
    """解析 IAP 0xFC 应答：[status][error]。"""
    if not payload:
        return {}
    iap_state = payload[0]
    error = payload[1] if len(payload) >= 2 else 0
    if iap_wire_uses_dex_v5_format(iap_state, error):
        err_name = V5_IAP_ERROR_NAMES.get(error, f"未知 (0x{error:02X})")
        state_name = V5_IAP_STATUS_NAMES.get(iap_state, f"未知 ({iap_state})")
        return {
            "iap_state": iap_state,
            "iap_state_name": state_name,
            "error_code": error,
            "error_name": err_name,
            "iap_format": "dex_v5",
        }
    iap_ver = (
        ProtocolVersion.V1_0_0
        if iap_wire_uses_v1_format(iap_state, error)
        else version
    )
    if iap_ver == ProtocolVersion.V1_0_0:
        err_info = parse_error_byte(ProtocolVersion.V1_0_0, error)
        err_name = str(err_info.get("error_name", "正常"))
        err_code = int(err_info.get("error_subcode", error))
    else:
        err_name = iap_error_name(version, error)
        err_code = error
    return {
        "iap_state": iap_state,
        "iap_state_name": iap_status_name(iap_ver, iap_state),
        "error_code": err_code,
        "error_name": err_name if err_code else "正常",
        "iap_format": "v1" if iap_ver == ProtocolVersion.V1_0_0 else "v2",
    }


def mode_display_name(version: ProtocolVersion, wire: int) -> str:
    if version == ProtocolVersion.V1_0_0:
        return V1_MODE_NAMES.get(wire, f"未知({wire})")
    return MODE_NAMES.get(wire, f"未知({wire})")


def mode_needs_position(version: ProtocolVersion, wire: int) -> bool:
    if version == ProtocolVersion.V1_0_0:
        return wire in (ControlModeV1.PP, ControlModeV1.CSP, ControlModeV1.MIT)
    try:
        return _v2_needs_position(ControlMode(wire))
    except ValueError:
        return False


def mode_needs_velocity(version: ProtocolVersion, wire: int) -> bool:
    if version == ProtocolVersion.V1_0_0:
        return wire in (ControlModeV1.CSV, ControlModeV1.CST, ControlModeV1.MIT)
    try:
        return _v2_needs_velocity(ControlMode(wire))
    except ValueError:
        return False


def mode_needs_current(version: ProtocolVersion, wire: int) -> bool:
    if version == ProtocolVersion.V1_0_0:
        return wire in (ControlModeV1.CST, ControlModeV1.MIT)
    try:
        return _v2_needs_current(ControlMode(wire))
    except ValueError:
        return False


def mode_hint(version: ProtocolVersion, wire: int) -> str:
    if version == ProtocolVersion.V1_0_0:
        if wire == ControlModeV1.HM:
            return "HM 回零：由固件回零流程驱动，无需手动下发目标。"
        if wire == ControlModeV1.MIT:
            return "MIT 模式：可同时下发位置/速度/电流；刚度阻尼在参数页配置。"
        parts = []
        if mode_needs_position(version, wire):
            parts.append("位置")
        if mode_needs_velocity(version, wire):
            parts.append("速度")
        if mode_needs_current(version, wire):
            parts.append("电流/转矩")
        return f"当前模式有效给定：{'、'.join(parts) if parts else '无'}"

    try:
        mode = ControlMode(wire)
    except ValueError:
        return f"未知模式 ({wire})"

    if mode is ControlMode.IDLE:
        return "空闲模式：请先设置其它模式并失能后再切换。"
    if mode is ControlMode.HOMING:
        return "回零模式：由固件回零流程驱动，无需手动下发目标。"
    if mode is ControlMode.VOLTAGE:
        return "电压模式：Vd/Vq 给定暂未在此面板提供，请通过其它接口配置。"
    if mode is ControlMode.MIT:
        return "MIT 模式：可同时下发位置/速度/电流；刚度阻尼在参数页配置。"
    parts = []
    if mode_needs_position(version, wire):
        parts.append("位置")
    if mode_needs_velocity(version, wire):
        parts.append("速度")
    if mode_needs_current(version, wire):
        parts.append("电流/转矩")
    return f"当前模式有效给定：{'、'.join(parts) if parts else '无'}"
