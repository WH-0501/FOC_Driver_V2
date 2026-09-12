"""CANFD 协议编解码，帧格式: [CNT, CMD, DATA...]，仲裁段 ID 为 Node-ID。"""

import math
import struct
from typing import Any, Dict, List, Optional, Tuple

from .constants import (
    CANFD_DLC_LENGTHS,
    CMD,
    IAP_MAX_BODY_SIZE,
    MotorError,
    PID_FORMAT,
    PID_RAW_MAX,
    PID_VALUE_MAX,
    POS_SCALE,
)
from .version import (
    DEFAULT_PROTOCOL_VERSION,
    ProtocolVersion,
    mode_display_name,
    parse_error_byte,
    parse_status_byte,
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
    return _pad_canfd_payload(data, 0x00)


def pad_iap_payload(data: bytes) -> bytes:
    """IAP 数据帧按 CAN FD DLC 规则尾部填充 0xFF（协议要求）。"""
    return _pad_canfd_payload(data, 0xFF)


def _pad_canfd_payload(data: bytes, fill: int) -> bytes:
    length = len(data)
    for dlc_len in CANFD_DLC_LENGTHS:
        if length <= dlc_len:
            if length < dlc_len:
                return data + bytes([fill]) * (dlc_len - length)
            return data
    return data


def _iap_payload_capacity(remaining: int, header_bytes: int = 0) -> int:
    """选取不超过 remaining 的最大 IAP 载荷，使 CNT+CMD+载荷 恰好落在标准 DLC 上（避免填充写入 Flash）。"""
    best = 0
    min_total = 2 + header_bytes
    for dlc in CANFD_DLC_LENGTHS:
        if dlc < min_total:
            continue
        payload = dlc - 2 - header_bytes
        if payload <= remaining and payload > best:
            best = payload
    if best > 0:
        return best
    return min(remaining, max(dlc - 2 - header_bytes for dlc in CANFD_DLC_LENGTHS if dlc >= min_total))


def iap_start_first_chunk_size(body_size: int) -> int:
    """IAP_START 首包固件数据字节数（载荷 = 8B 头 + chunk，且 payload_len > 8）。"""
    return _iap_payload_capacity(body_size, header_bytes=8)


def iap_data_chunk_size(remaining: int) -> int:
    """IAP_DATA 单帧固件数据字节数。"""
    return _iap_payload_capacity(remaining, header_bytes=0)


def firmware_checksum32(data: bytes) -> int:
    """与 iap_upgrade.c / Bootloader 一致的 32 位累加校验（小端字）。"""
    body_len = len(data)
    count = body_len >> 2
    cksum = 0
    for i in range(count):
        data32 = struct.unpack_from("<I", data, i * 4)[0]
        cksum = (cksum + data32) & 0xFFFFFFFF
        if cksum < data32:
            cksum = (cksum + 1) & 0xFFFFFFFF
    rem = body_len & 3
    if rem:
        data32 = struct.unpack_from("<I", data, count * 4)[0]
        if rem == 3:
            data32 &= 0x00FFFFFF
        elif rem == 2:
            data32 &= 0x0000FFFF
        else:
            data32 &= 0x000000FF
        cksum = (cksum + data32) & 0xFFFFFFFF
        if cksum < data32:
            cksum = (cksum + 1) & 0xFFFFFFFF
    return cksum


def pack_iap_start_payload(body_size: int, firmware: bytes) -> bytes:
    """打包 IAP_START 载荷：[body_size BE][checksum32 BE][首包固件数据]。"""
    if body_size <= 0 or body_size > IAP_MAX_BODY_SIZE:
        raise ValueError(f"固件大小无效: {body_size}（最大 {IAP_MAX_BODY_SIZE}）")
    computed = firmware_checksum32(firmware)
    checksum32 = (~computed) & 0xFFFFFFFF
    chunk_len = iap_start_first_chunk_size(body_size)
    header = struct.pack(">II", body_size, checksum32)
    return header + firmware[:chunk_len]


def _is_binary_fw_version(payload: bytes) -> bool:
    """
    协议文档 IAP 0xFD 二进制格式:
    V + major(1B) + '.' + minor(1B) + '.' + patch(1B) + '_' + 日期时间(每字节十进制两位)
    例: 56 03 2E 01 2E 05 5F 1A 06 ... -> V3.1.5_260610113015
    """
    if len(payload) < 8:
        return False
    if payload[0] != ord("V"):
        return False
    if payload[2] != ord(".") or payload[4] != ord(".") or payload[6] != ord("_"):
        return False
    # 纯 ASCII 串如 V1.0.0_FOC：版本位为 '0'~'9'
    return not (
        0x30 <= payload[1] <= 0x39
        and 0x30 <= payload[3] <= 0x39
        and 0x30 <= payload[5] <= 0x39
    )


def parse_fw_version_payload(payload: bytes) -> str:
    """解析 IAP 0xFD 固件版本应答（兼容 ASCII 与协议二进制两种格式）。"""
    if not payload:
        return ""
    end = payload.find(b"\x00")
    if end >= 0:
        payload = payload[:end]
    if not payload:
        return ""
    if _is_binary_fw_version(payload):
        major = payload[1]
        minor = payload[3]
        patch = payload[5]
        date_str = "".join(f"{b:02d}" for b in payload[7:])
        return f"V{major}.{minor}.{patch}_{date_str}"
    return payload.decode("ascii", errors="replace")


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

def float_to_q12(value: float) -> int:
    """float -> Q12 uint16，与固件 can_fd_codec_float_to_q12 一致。"""
    scaled = value * PID_FORMAT
    if scaled < 0.0:
        scaled = 0.0
    if scaled > PID_RAW_MAX:
        scaled = float(PID_RAW_MAX)
    return int(round(scaled))


def pid_params_in_range(p: float, i: float, d: float) -> Optional[str]:
    """超出 Q12 可编码范围时返回错误说明，否则返回 None。"""
    for name, val in (("P", p), ("I", i), ("D", d)):
        if val < 0.0:
            return f"{name}={val} 不能为负数"
        if val > PID_VALUE_MAX:
            return f"{name}={val} 超出 Q12 上限 {PID_VALUE_MAX:.4f}（协议 uint16 最大 65535）"
    return None


def pack_pid_params(p: float, i: float, d: float) -> bytes:
    return struct.pack(">HHH", float_to_q12(p), float_to_q12(i), float_to_q12(d))


def pack_stiff_damp(stiff: float, damp: float) -> bytes:
    return struct.pack(">HH", float_to_q12(stiff), float_to_q12(damp))


def pack_protect(over_temp: int, over_current_ma: int, under_voltage_v: int, over_voltage_v: int) -> bytes:
    """保护参数: OTP(2) + OCP(2) + UVLO(1) + OVP(1)。"""
    return struct.pack(">HHBB", over_temp, over_current_ma, under_voltage_v, over_voltage_v)


def pack_target_entry(motor_id: int, value: int) -> bytes:
    """单电机目标: id(1) + value(2, 大端 int16)。"""
    return struct.pack(">Bh", motor_id, value)


def pack_multi_targets(motor_targets: List[Tuple[int, int]]) -> bytes:
    """多电机目标列表打包。"""
    return b"".join(pack_target_entry(mid, val) for mid, val in motor_targets)


def deg_to_pos_raw(deg: float) -> int:
    """角度(°) -> Q7 位置 raw（V1.0.0，1 LSB = 1/128 °）。"""
    return int(round(deg * POS_SCALE))


def pos_raw_to_deg_v1(raw: int) -> float:
    """Q7 位置 raw -> 角度(°)（V1.0.0）。"""
    return raw / POS_SCALE


def rad_to_pos_raw(rad: float) -> int:
    """弧度(rad) -> Q7 位置 raw（V2.0.0，1 LSB = 1/128 rad）。"""
    return int(round(rad * POS_SCALE))


def pos_raw_to_rad(raw: int) -> float:
    """Q7 位置 raw -> 弧度(rad)（V2.0.0）。"""
    return raw / POS_SCALE


def pos_uses_rad(protocol_version: ProtocolVersion) -> bool:
    """V2.0.0 起 CAN 位置单位为 rad，此前为 °。"""
    return protocol_version == ProtocolVersion.V2_0_0


def encode_position(protocol_version: ProtocolVersion, value: float) -> int:
    """按协议版本将 GUI 位置值编码为 Q7 raw。"""
    if pos_uses_rad(protocol_version):
        return rad_to_pos_raw(value)
    return deg_to_pos_raw(value)


def decode_position_from_raw(
    protocol_version: ProtocolVersion, raw: int
) -> Dict[str, float]:
    """
    按协议版本解析 Q7 位置 raw。
    position_plot：波形/控制用（V1=°，V2=rad）；position_deg：状态页显示。
    """
    if pos_uses_rad(protocol_version):
        pos_rad = pos_raw_to_rad(raw)
        pos_deg = math.degrees(pos_rad)
        pos_plot = pos_rad
    else:
        pos_deg = pos_raw_to_deg_v1(raw)
        pos_rad = math.radians(pos_deg)
        pos_plot = pos_deg
    return {
        "position_rad": pos_rad,
        "position_deg": pos_deg,
        "position_plot": pos_plot,
    }


def angle_to_pos_raw(degrees: float) -> int:
    """角度(°) -> Q7 位置 raw（V1.0.0 兼容别名）。"""
    return deg_to_pos_raw(degrees)


def pos_raw_to_angle(raw: int) -> float:
    """Q7 位置 raw -> 角度(°)（V1.0.0 兼容别名）。"""
    return pos_raw_to_deg_v1(raw)


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


def parse_period_report(
    payload: bytes,
    protocol_version: ProtocolVersion = DEFAULT_PROTOCOL_VERSION,
) -> Dict[str, Any]:
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

    mode, enable, enable_nibble = parse_status_byte(protocol_version, status)
    error_info = parse_error_byte(protocol_version, error)
    pos_fields = decode_position_from_raw(protocol_version, pos)
    return {
        "position_raw": pos,
        **pos_fields,
        "velocity": vel,
        "iq_ma": iq,
        "id_ma": id_ma,
        "ia_ma": ia_ma,
        "ib_ma": ib_ma,
        "ic_ma": ic_ma,
        "current_ma": iq,
        "status": status,
        "mode": mode,
        "mode_name": mode_display_name(protocol_version, mode),
        "enable": enable,
        "enable_nibble": enable_nibble,
        "error_code": error_info["error_code"],
        "error_subcode": error_info["error_subcode"],
        "error_is_iap": error_info["error_is_iap"],
        "error_type": error_info["error_type"],
        "error_name": error_info["error_name"],
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
