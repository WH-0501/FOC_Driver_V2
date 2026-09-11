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
    return f"状态 {status}"


def parse_iap_status_payload(
    version: ProtocolVersion, payload: bytes
) -> Dict[str, object]:
    """解析 IAP 0xFC 应答（V1：状态 + 错误码）。"""
    if not payload:
        return {}
    iap_state = payload[0]
    result: Dict[str, object] = {
        "iap_state": iap_state,
        "iap_state_name": iap_status_name(version, iap_state),
    }
    if len(payload) >= 2:
        result.update(parse_error_byte(version, payload[1]))
    return result


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
        return wire in (ControlModeV1.CSV, ControlModeV1.MIT)
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
