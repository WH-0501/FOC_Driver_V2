"""CANFD 协议编解码，帧格式: [CNT, CMD, DATA...]，仲裁段 ID 为 Node-ID。"""

import math
import struct
from typing import Any, Dict, List, Optional, Tuple

from .constants import (
    CANFD_DLC_LENGTHS,
    CMD,
    IAP_ASIX_HEADER_SIZE,
    IAP_ASIX_SIGNATURE,
    IAP_MAX_BODY_SIZE,
    IAP_MAX_FILE_SIZE,
    IAP_START_MIN_FIRMWARE_CHUNK,
    IAP_V5_CHUNK_SIZE,
    IAP_V5_FRAME_SIZE,
    IAP_V5_PAD_BYTE,
    IAP_V5_MIN_START_PAYLOAD,
    ErrorCode,
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


def iap_start_first_chunk_size(body_size: int, *, prefer_largest: bool = True) -> int:
    """
    IAP_START 首包固件数据字节数。
    - 协议要求 IAP_START 载荷 >32B（固件头 + 数据；头为 ASIX 格式，内含 FileLen/CRC32）
    - 帧长须落在标准 DLC 上，避免 0 填充被写入 Flash
    - 默认取最大合法首包（64B 帧 = 54B 固件数据），减少总帧数
    """
    if body_size <= 0:
        return 0
    min_chunk = min(IAP_START_MIN_FIRMWARE_CHUNK, body_size)
    candidates: list[int] = []
    for dlc in CANFD_DLC_LENGTHS:
        if dlc < 10:
            continue
        chunk = dlc - 10  # 2(CNT+CMD) + 8(header)
        if chunk >= min_chunk and chunk <= body_size:
            candidates.append(chunk)
    if not candidates:
        return min(body_size, max(2, min_chunk))
    return max(candidates) if prefer_largest else min(candidates)


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


def format_asix_header_log(data: bytes) -> str:
    """格式化 START 首帧中的 32B ASIX 固件头，便于日志核对。"""
    header = data[:IAP_ASIX_HEADER_SIZE]
    info = parse_asix_header(header)
    hex32 = format_bytes_hex(header)
    if info is None:
        return f"IAP START 固件头（无法解析）HEX=[{hex32}]"
    sig = header[:4].decode("ascii", errors="replace")
    return (
        f"IAP START 固件头 32B: Signature={sig!r} ImageType={info['image_type']} "
        f"FileLen={info['file_len']} Body={info['body_size']} B "
        f"Checksum32=0x{info['checksum32']:08X} "
        f"首帧数据 +{max(0, len(data) - IAP_ASIX_HEADER_SIZE)} B "
        f"HEX=[{hex32}]"
    )


def parse_asix_header(data: bytes) -> Optional[Dict[str, int]]:
    """
    解析 32B UPGRADE_RT_HEAD（dex-con iap.c / iap_upgrade.c 布局）。
    FileLen @12 BE；Checksum32 @20 LE。
    """
    if len(data) < IAP_ASIX_HEADER_SIZE or data[:4] != IAP_ASIX_SIGNATURE:
        return None
    file_len = struct.unpack(">I", data[12:16])[0]
    if file_len <= IAP_ASIX_HEADER_SIZE:
        return None
    body_size = file_len - IAP_ASIX_HEADER_SIZE
    checksum32 = struct.unpack("<I", data[20:24])[0]
    image_type = struct.unpack("<H", data[4:6])[0]
    return {
        "file_len": file_len,
        "body_size": body_size,
        "checksum32": checksum32,
        "image_type": image_type,
    }


def build_asix_header(body: bytes) -> bytes:
    """为纯 App .bin 生成 dex-con 兼容的 32B ASIX 头。"""
    file_len = IAP_ASIX_HEADER_SIZE + len(body)
    checksum32 = (~firmware_checksum32(body)) & 0xFFFFFFFF
    header = bytearray(IAP_ASIX_HEADER_SIZE)
    header[0:4] = IAP_ASIX_SIGNATURE
    struct.pack_into("<H", header, 4, 3)  # ImageType
    struct.pack_into("<H", header, 6, 0)  # Compress
    struct.pack_into("<I", header, 8, 0)  # ContentOffset
    struct.pack_into(">I", header, 12, file_len)
    struct.pack_into("<I", header, 20, checksum32)
    return bytes(header)


def prepare_iap_firmware_image(data: bytes) -> tuple[bytes, str]:
    """
    准备 dex-con IAP 整包镜像（含 32B ASIX 头）。
    IAP_Process START 会 memcpy(&data[2], 32) 并整包写入 Flash。
    """
    info = parse_asix_header(data)
    if info is not None:
        file_len = info["file_len"]
        note = f"ASIX 整包 {len(data)} B（FileLen={file_len}，体 {info['body_size']} B）"
        if len(data) != file_len:
            note += f"，与声明长度 {file_len} B 不一致，按文件实际长度传输"
        return data, note

    if len(data) > IAP_MAX_BODY_SIZE:
        raise ValueError(f"固件体 {len(data)} B 超过上限 {IAP_MAX_BODY_SIZE} B")
    image = build_asix_header(data) + data
    return image, f"已补 ASIX 头，整包 {len(image)} B（体 {len(data)} B）"


def prepare_iap_firmware_body(data: bytes) -> tuple[bytes, str, Optional[int]]:
    """V2 iap_upgrade.c 用：剥离 ASIX，仅传 App 体。"""
    info = parse_asix_header(data)
    if info is None:
        return data, "", None
    body = data[IAP_ASIX_HEADER_SIZE:]
    note = (
        f"检测到 ASIX staging 头，已跳过 {IAP_ASIX_HEADER_SIZE}B，"
        f"实际传输体 {len(body)} B"
    )
    return body, note, info["checksum32"]


def iap_v5_start_payload_size(remaining: int, *, prefer_largest: bool = True) -> int:
    """dex-con START 载荷：至少 32B ASIX 头 + 可选后续体，帧长须为标准 DLC。"""
    if remaining <= 0:
        return 0
    min_payload = min(IAP_V5_MIN_START_PAYLOAD, remaining)
    candidates: list[int] = []
    for dlc in CANFD_DLC_LENGTHS:
        payload = dlc - 2
        if payload < min_payload or payload > remaining:
            continue
        candidates.append(payload)
    if not candidates:
        return remaining
    return max(candidates) if prefer_largest else min(candidates)


def split_iap_v5_image_chunks(image: bytes, chunk_size: int = IAP_V5_CHUNK_SIZE) -> list[bytes]:
    """将 ASIX 整包切为 START/DATA 用的固定长度块（末块可不足 chunk_size）。"""
    if not image:
        return []
    chunks: list[bytes] = []
    offset = 0
    while offset < len(image):
        chunks.append(image[offset : offset + chunk_size])
        offset += chunk_size
    return chunks


def build_iap_v5_can_frame(cmd: CMD, payload: bytes, cnt: int) -> bytes:
    """
    构造 dex-con IAP CAN-FD 帧（对齐 CSV 回放）。
    START/DATA：CNT+CMD+payload 不足 64B 时用 0xFF 补齐至 64B。
    其余命令：仅 CNT+CMD（DLC=2）。
    """
    frame = bytes([cnt & 0xFF, int(cmd)]) + payload
    if int(cmd) in (CMD.IAP_START, CMD.IAP_DATA):
        if len(frame) > IAP_V5_FRAME_SIZE:
            raise ValueError(f"IAP 帧过长: {len(frame)} > {IAP_V5_FRAME_SIZE}")
        if len(frame) < IAP_V5_FRAME_SIZE:
            frame += bytes([IAP_V5_PAD_BYTE]) * (IAP_V5_FRAME_SIZE - len(frame))
        return frame
    if len(frame) not in CANFD_DLC_LENGTHS:
        raise ValueError(f"IAP 命令帧 DLC 非法: len={len(frame)}")
    return frame


def pack_iap_v5_start_payload(image: bytes, payload_size: Optional[int] = None) -> bytes:
    """dex-con IAP_START：CNT/CMD 之后为 ASIX 头 + 首段固件（不剥离头）。"""
    if not image:
        raise ValueError("固件镜像为空")
    if len(image) > IAP_MAX_FILE_SIZE:
        raise ValueError(f"固件整包 {len(image)} B 超过上限 {IAP_MAX_FILE_SIZE} B")
    size = payload_size if payload_size is not None else iap_v5_start_payload_size(len(image))
    size = max(min(size, len(image)), min(IAP_V5_MIN_START_PAYLOAD, len(image)))
    return image[:size]


def pack_iap_start_payload(
    body_size: int,
    firmware: bytes,
    *,
    checksum32: Optional[int] = None,
    first_chunk: Optional[int] = None,
) -> bytes:
    """打包 IAP_START 载荷：[body_size BE][checksum32 BE][首包固件数据]。"""
    if body_size <= 0 or body_size > IAP_MAX_BODY_SIZE:
        raise ValueError(f"固件大小无效: {body_size}（最大 {IAP_MAX_BODY_SIZE}）")
    if checksum32 is None:
        computed = firmware_checksum32(firmware)
        checksum32 = (~computed) & 0xFFFFFFFF
    chunk_len = first_chunk if first_chunk is not None else iap_start_first_chunk_size(body_size)
    header = struct.pack(">II", body_size, checksum32 & 0xFFFFFFFF)
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
    if _is_binary_fw_version(payload):
        major = payload[1]
        minor = payload[3]
        patch = payload[5]
        date_bytes = payload[7:]
        while date_bytes and date_bytes[-1] == 0:
            date_bytes = date_bytes[:-1]
        date_str = "".join(f"{b:02d}" for b in date_bytes)
        return f"V{major}.{minor}.{patch}_{date_str}"
    end = payload.find(b"\x00")
    if end >= 0:
        payload = payload[:end]
    if not payload:
        return ""
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
    """解析布尔应答（保存参数等）：1 成功，0 失败。"""
    return len(payload) >= 1 and payload[0] == 1


def parse_api_result(payload: bytes) -> tuple[bool, int]:
    """解析 MC API 应答：payload[0] 为 error_t，0 表示成功。"""
    if not payload:
        return False, int(ErrorCode.ERR_FAIL)
    code = int(payload[0])
    return code == int(ErrorCode.ERR_NONE), code


def parse_kth71_anlc_status(payload: bytes) -> Optional[int]:
    if len(payload) < 1:
        return None
    return int(payload[0])


def parse_node_id_response(payload: bytes) -> Optional[int]:
    if len(payload) < 2:
        return None
    return struct.unpack(">H", payload[0:2])[0]
