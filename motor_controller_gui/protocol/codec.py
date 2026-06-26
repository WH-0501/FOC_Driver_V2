"""CANFD 协议编解码，帧格式: [CNT, CMD, DATA...]，仲裁段 ID 为 Node-ID。"""

import struct
from typing import Any, Dict, List, Optional, Tuple

from .constants import (
    CANFD_DLC_LENGTHS,
    CMD,
    ERROR_NAMES,
    MODE_NAMES,
    MotorError,
    PID_FORMAT,
    POS_SCALE,
)


class FrameCounter:
    """发送帧计数器 0~255 循环。"""

    def __init__(self) -> None:
        self._cnt = 0

    def next(self) -> int:
        value = self._cnt
        self._cnt = (self._cnt + 1) & 0xFF
        return value


def pad_canfd_payload(data: bytes) -> bytes:
    """按 CAN FD DLC 规则尾部填充 0x00。"""
    length = len(data)
    for dlc_len in CANFD_DLC_LENGTHS:
        if length <= dlc_len:
            if length < dlc_len:
                return data + b"\x00" * (dlc_len - length)
            return data
    return data


def pack_frame(cmd: CMD, payload: bytes = b"", cnt: int = 0) -> bytes:
    """打包一帧: [CNT, CMD, payload...]。"""
    return bytes([cnt & 0xFF, int(cmd)]) + payload


def parse_frame(data: bytes) -> Optional[Tuple[int, int, bytes]]:
    """解析帧，返回 (cnt, cmd, payload)。"""
    if len(data) < 2:
        return None
    return data[0], data[1], data[2:]


def format_bytes_hex(data: bytes) -> str:
    """字节序列 -> 大写十六进制字符串，空格分隔。"""
    return " ".join(f"{b:02X}" for b in data)


def format_bytes_binary(data: bytes) -> str:
    """字节序列 -> 二进制字符串，每字节 8 位、空格分隔。"""
    return " ".join(format(b, "08b") for b in data)


# ---------- 参数打包（大端，与固件一致） ----------

def pack_pid_params(p: float, i: float, d: float) -> bytes:
    p_int = int(round(p * PID_FORMAT))
    i_int = int(round(i * PID_FORMAT))
    d_int = int(round(d * PID_FORMAT))
    return struct.pack(">HHH", p_int, i_int, d_int)


def pack_stiff_damp(stiff: float, damp: float) -> bytes:
    stiff_int = int(round(stiff * PID_FORMAT))
    damp_int = int(round(damp * PID_FORMAT))
    return struct.pack(">HH", stiff_int, damp_int)


def pack_protect(over_temp: int, over_current_ma: int, under_voltage_v: int, over_voltage_v: int) -> bytes:
    """保护参数: OTP(2) + OCP(2) + UVLO(1) + OVP(1)。"""
    return struct.pack(">HHBB", over_temp, over_current_ma, under_voltage_v, over_voltage_v)


def pack_target_entry(motor_id: int, value: int) -> bytes:
    """单电机目标: id(1) + value(2, 大端 int16)。"""
    return struct.pack(">Bh", motor_id, value)


def pack_multi_targets(motor_targets: List[Tuple[int, int]]) -> bytes:
    """多电机目标列表打包。"""
    return b"".join(pack_target_entry(mid, val) for mid, val in motor_targets)


def angle_to_pos_raw(degrees: float) -> int:
    """角度(°) -> Q7 位置 raw 值。"""
    return int(round(degrees * POS_SCALE))


def pos_raw_to_angle(raw: int) -> float:
    return raw / POS_SCALE


# ---------- 应答解析 ----------

# 周期上报 payload 布局（不含 CNT/CMD）:
# 旧版 DLC12 / payload≤10: pos(2) vel(2) iq(2) status(1) error(1) [+ CAN FD 填充 0]
# 新版 DLC20 / payload≥16: pos vel iq id ia ib ic status error
PERIOD_LEGACY_CORE_LEN = 8
PERIOD_EXTENDED_LEN = 16


def _parse_current_ma(payload: bytes, offset: int) -> int:
    if len(payload) < offset + 2:
        return 0
    return struct.unpack(">h", payload[offset : offset + 2])[0]


def _payload_tail_is_zero_padding(payload: bytes, start: int) -> bool:
    return all(b == 0 for b in payload[start:])


def detect_period_report_variant(payload: bytes) -> str:
    """
    识别周期上报格式。
    - legacy: 旧固件 8 字节有效数据，DLC12 时尾部为 0 填充
    - extended: 新固件含 Id/Ia/Ib/Ic，payload≥16
    """
    n = len(payload)
    if n < PERIOD_LEGACY_CORE_LEN:
        return "unknown"
    if n >= PERIOD_EXTENDED_LEN:
        return "extended"
    if n > PERIOD_LEGACY_CORE_LEN and _payload_tail_is_zero_padding(payload, PERIOD_LEGACY_CORE_LEN):
        return "legacy"
    return "legacy"


def parse_period_report(payload: bytes) -> Dict[str, Any]:
    """
    解析周期上报 payload（不含 CNT/CMD），自动兼容旧/新固件。

    旧版（CAN FD DLC=12，有效 payload 8B，尾部 0 填充）:
        pos, vel, iq(mA), status, error
    新版（CAN FD DLC=20，有效 payload 16B，尾部 0 填充）:
        pos, vel, iq, id, ia, ib, ic, status, error
    """
    variant = detect_period_report_variant(payload)
    if variant == "unknown":
        return {}

    pos = struct.unpack(">h", payload[0:2])[0]
    vel = struct.unpack(">h", payload[2:4])[0]
    iq = _parse_current_ma(payload, 4)

    if variant == "extended":
        id_ma = _parse_current_ma(payload, 6)
        ia_ma = _parse_current_ma(payload, 8)
        ib_ma = _parse_current_ma(payload, 10)
        ic_ma = _parse_current_ma(payload, 12)
        status = payload[14]
        error = payload[15]
        has_phase_current = True
    else:
        id_ma = ia_ma = ib_ma = ic_ma = 0
        status = payload[6]
        error = payload[7]
        has_phase_current = False

    mode = status & 0x0F
    enable = (status >> 4) & 0x0F
    return {
        "position_raw": pos,
        "position_deg": pos_raw_to_angle(pos),
        "velocity": vel,
        "iq_ma": iq,
        "id_ma": id_ma,
        "ia_ma": ia_ma,
        "ib_ma": ib_ma,
        "ic_ma": ic_ma,
        "current_ma": iq,
        "status": status,
        "mode": mode,
        "mode_name": MODE_NAMES.get(mode, f"未知({mode})"),
        "enable": bool(enable),
        "error_code": error,
        "error_name": ERROR_NAMES.get(error, f"未知({error})"),
        "report_variant": variant,
        "has_phase_current": has_phase_current,
    }


def parse_pid_response(payload: bytes) -> Dict[str, float]:
    if len(payload) < 6:
        return {}
    p_raw, i_raw, d_raw = struct.unpack(">HHH", payload[0:6])
    return {
        "P": p_raw / PID_FORMAT,
        "I": i_raw / PID_FORMAT,
        "D": d_raw / PID_FORMAT,
    }


def parse_stiff_damp_response(payload: bytes) -> Dict[str, float]:
    if len(payload) < 4:
        return {}
    stiff, damp = struct.unpack(">HH", payload[0:4])
    return {"stiffness": stiff / PID_FORMAT, "damping": damp / PID_FORMAT}


def parse_protect_response(payload: bytes) -> Dict[str, Any]:
    if len(payload) < 6:
        return {}
    otp, ocp = struct.unpack(">HH", payload[0:4])
    uvlo = payload[4]
    ovp = payload[5]
    return {
        "over_temperature": otp,
        "over_current_ma": ocp,
        "over_current_A": ocp / 1000.0,
        "under_voltage_V": uvlo,
        "over_voltage_V": ovp,
    }


def parse_bool_result(payload: bytes) -> bool:
    return len(payload) >= 1 and payload[0] == 1


def parse_node_id_response(payload: bytes) -> Optional[int]:
    if len(payload) < 2:
        return None
    return struct.unpack(">H", payload[0:2])[0]
