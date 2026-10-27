"""V2 PID 线值：K = raw / 4096，与固件 can_fd_codec_pack/unpack_pid 一致。"""
from __future__ import annotations

import struct
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "motor_controller_gui"))

from protocol.codec import (
    pack_cur_pid_params,
    pack_pid_params,
    parse_cur_pid_response,
    parse_pid_response,
    pid_wire_from_float,
)
from protocol.constants import CUR_PID_KI_SCALE, PID_FORMAT, PID_K_MIN_STEP
from protocol.version import ProtocolVersion

FMT = PID_FORMAT


def firmware_unpack(payload: bytes) -> tuple[float, float, float]:
    p_raw, i_raw, d_raw = struct.unpack(">HHH", payload[:6])
    return (p_raw / FMT, i_raw / FMT, d_raw / FMT)


def firmware_pack(kp: float, ki: float, kd: float) -> bytes:
    def wire(k: float) -> int:
        scaled = k * FMT
        if scaled < 0:
            scaled = 0.0
        if scaled > 65535:
            scaled = 65535.0
        return int(round(scaled))

    return struct.pack(">HHH", wire(kp), wire(ki), wire(kd))


def test_doc_example_raw_61():
    payload = bytes([0x00, 0x3D, 0x00, 0x04, 0x00, 0x00])
    kp, ki, kd = firmware_unpack(payload)
    assert abs(kp - 61 / FMT) < 1e-9
    assert abs(ki - 4 / FMT) < 1e-9
    assert kd == 0.0
    parsed = parse_pid_response(payload, ProtocolVersion.V2_0_0)
    assert abs(parsed["P"] - kp) < 1e-9


def test_default_velocity_pid():
    kp_def, ki_def = 0.0008, 0.003
    assert pid_wire_from_float(kp_def, ProtocolVersion.V2_0_0) == 3
    assert pid_wire_from_float(ki_def, ProtocolVersion.V2_0_0) == 12
    payload = firmware_pack(kp_def, ki_def, 0.0)
    assert payload == pack_pid_params(kp_def, ki_def, 0.0, ProtocolVersion.V2_0_0)
    kp, ki, _ = firmware_unpack(payload)
    assert abs(kp - 3 / FMT) < 1e-9
    assert abs(ki - 12 / FMT) < 1e-9


def test_roundtrip_vectors():
    vectors = [
        (0.0008, 0.003, 0.0),
        (2.0, 0.0, 0.0),
        (1.0, 0.2, 0.0),
        (61 / FMT, 4 / FMT, 0.0),
    ]
    for kp, ki, kd in vectors:
        fw = firmware_pack(kp, ki, kd)
        gui = pack_pid_params(kp, ki, kd, ProtocolVersion.V2_0_0)
        assert fw == gui
        parsed = parse_pid_response(fw, ProtocolVersion.V2_0_0)
        p2, i2, d2 = firmware_unpack(fw)
        assert abs(parsed["P"] - p2) < 1e-9
        assert abs(parsed["I"] - i2) < 1e-9
        assert abs(parsed["D"] - d2) < 1e-9


def test_min_step():
    assert abs(PID_K_MIN_STEP - 1.0 / FMT) < 1e-12


def test_cur_pid_ki_scale():
    kp, ki_phys, kd = 4.0, 1507.0, 0.0
    payload = pack_cur_pid_params(kp, ki_phys, kd, ProtocolVersion.V2_0_0)
    kp_w, ki_w, kd_w = firmware_unpack(payload)
    assert abs(kp_w - kp) < 1e-9
    assert abs(ki_w - ki_phys / CUR_PID_KI_SCALE) < 1e-3
    parsed = parse_cur_pid_response(payload, ProtocolVersion.V2_0_0)
    assert abs(parsed["P"] - kp) < 1e-9
    assert abs(parsed["I"] - ki_phys) < 1.0  # Q12 量化
    assert abs(ki_w * CUR_PID_KI_SCALE - ki_phys) < 1.0  # Q12 步进


if __name__ == "__main__":
    test_doc_example_raw_61()
    test_default_velocity_pid()
    test_roundtrip_vectors()
    test_min_step()
    test_cur_pid_ki_scale()
    print("PASS: V2 PID Q12 codec")
