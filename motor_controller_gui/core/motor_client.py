"""高层 CAN 电机通讯客户端。"""

import threading
import time
from collections import deque
from typing import Callable, Deque, Dict, List, Optional, Tuple

from canbus.interface import CanThread
from protocol import (
    CAN_BROADCAST_ID,
    CAN_IAP_ID,
    CANFD_DLC_LENGTHS,
    CMD,
    DEFAULT_PROTOCOL_VERSION,
    FrameCounter,
    ProtocolVersion,
    build_iap_v5_can_frame,
    format_bytes_hex,
    pack_frame,
    pack_multi_targets,
    pack_pid_params,
    pack_protect,
    pack_stiff_damp,
    pad_canfd_payload,
)


class MotorClient:
    """封装协议发送逻辑，供 UI 层调用。"""

    def __init__(self, can_thread: CanThread, log_fn: Optional[Callable[[str], None]] = None):
        self.can = can_thread
        self.counter = FrameCounter()
        self.iap_counter = FrameCounter()
        self._iap_session = False
        self._iap_arb_id = CAN_IAP_ID
        self._log_fn = log_fn
        self.protocol_version: ProtocolVersion = DEFAULT_PROTOCOL_VERSION
        self._iap_lock = threading.Lock()
        self._iap_event = threading.Event()
        self._iap_responses: Deque[tuple] = deque()

    def set_protocol_version(self, version: ProtocolVersion) -> None:
        self.protocol_version = version

    def _log_tx(
        self,
        arb_id: int,
        cmd: CMD,
        frame: bytes,
        data: bytes,
        *,
        log_fn: Optional[Callable[[str], None]] = None,
    ) -> None:
        logger = log_fn if log_fn is not None else self._log_fn
        if logger is None:
            return
        cnt = frame[0] if frame else 0
        cmd_name = cmd.name if isinstance(cmd, CMD) else f"CMD_{int(cmd):02X}"
        logger(
            f"下发 NodeID=0x{arb_id:03X} "
            f"[CNT=0x{cnt:02X} 帧计数] CMD=0x{int(cmd):02X}({cmd_name}) "
            f"LEN={len(data)} "
            f"HEX=[{format_bytes_hex(data)}] "
        )

    def begin_iap_session(self, motor_id: int) -> None:
        """IAP 升级：独立帧计数从 0 起；单电机使用 Node-ID（协议文档）。"""
        self.iap_counter = FrameCounter()
        self._iap_arb_id = motor_id
        self._iap_session = True
        self.clear_iap_wait()

    def end_iap_session(self) -> None:
        self._iap_session = False

    def _send_iap_v5(self, cmd: CMD, payload: bytes = b"", *, log: bool = True) -> bool:
        cnt = self.iap_counter.next()
        try:
            data = build_iap_v5_can_frame(cmd, payload, cnt)
        except ValueError:
            return False
        if log:
            self._log_tx(
                self._iap_arb_id,
                cmd,
                data[: min(len(data), 2 + len(payload))],
                data,
            )
        return self.can.send_message(self._iap_arb_id, data, is_fd=True)

    def iap_enter_start_burst(self, motor_id: int, start_payload: bytes) -> tuple[int, int, float, float]:
        """
        连续发送 ENTER(CNT=0) + START(CNT=1)；须等 FC(擦除完成) 后再发 DATA。
        返回 (enter_cnt, start_cnt, start_sent_at, can_elapsed_s)；失败时 start_sent_at=0。
        """
        if not self._iap_session:
            self.begin_iap_session(motor_id)
        enter_cnt = self.iap_counter.next()
        start_cnt = self.iap_counter.next()
        try:
            enter_frame = build_iap_v5_can_frame(CMD.IAP_ENTER, b"", enter_cnt)
            start_frame = build_iap_v5_can_frame(CMD.IAP_START, start_payload, start_cnt)
        except ValueError:
            return (-1, -1, 0.0, 0.0)
        t_can = time.monotonic()
        ok = self.can.send_messages_burst(
            [(self._iap_arb_id, enter_frame), (self._iap_arb_id, start_frame)]
        )
        can_elapsed = time.monotonic() - t_can
        start_sent_at = time.monotonic()
        if not ok:
            return (-1, -1, 0.0, can_elapsed)
        # 日志异步写入，避免 Qt 阻塞导致错过擦除 FC / 3s DATA 窗口
        if self._log_fn is not None:
            arb_id = self._iap_arb_id
            log_fn = self._log_fn

            def _log_burst() -> None:
                log_fn(
                    f"下发 NodeID=0x{arb_id:03X} "
                    f"[CNT=0x{enter_cnt:02X} 帧计数] CMD=0xF8(IAP_ENTER) LEN=2 "
                    f"HEX=[{format_bytes_hex(enter_frame)}] "
                )
                log_fn(
                    f"下发 NodeID=0x{arb_id:03X} "
                    f"[CNT=0x{start_cnt:02X} 帧计数] CMD=0xF9(IAP_START) LEN={len(start_frame)} "
                    f"HEX=[{format_bytes_hex(start_frame)}] "
                )

            threading.Thread(target=_log_burst, daemon=True).start()
        return (enter_cnt, start_cnt, start_sent_at, can_elapsed)

    def iap_send_data_burst(
        self, chunks: list[bytes], *, log_first: bool = False, log_async: bool = False
    ) -> int:
        """连发多帧 IAP_DATA，返回成功发送的帧数。"""
        if not chunks:
            return 0
        frames: list[tuple[int, bytes]] = []
        built: list[tuple[bytes, bytes]] = []
        for chunk in chunks:
            cnt = self.iap_counter.next()
            try:
                frame = build_iap_v5_can_frame(CMD.IAP_DATA, chunk, cnt)
            except ValueError:
                break
            frames.append((self._iap_arb_id, frame))
            built.append((chunk, frame))
        if not frames:
            return 0
        if not self.can.send_messages_burst(frames):
            return 0
        if log_first and self._log_fn is not None and built:
            chunk, frame = built[0]
            if log_async:
                arb_id = self._iap_arb_id
                log_fn = self._log_fn
                snap = frame[: min(len(frame), 2 + len(chunk))]

                def _log() -> None:
                    log_fn(
                        f"下发 NodeID=0x{arb_id:03X} "
                        f"[CNT=0x{snap[0]:02X} 帧计数] CMD=0xFA(IAP_DATA) "
                        f"LEN={len(frame)} HEX=[{format_bytes_hex(frame)}] "
                    )

                threading.Thread(target=_log, daemon=True).start()
            else:
                self._log_tx(
                    self._iap_arb_id,
                    CMD.IAP_DATA,
                    frame[: min(len(frame), 2 + len(chunk))],
                    frame,
                )
        return len(frames)

    def _send(self, arb_id: int, cmd: CMD, payload: bytes = b"", *, exact_dlc: bool = False) -> bool:
        frame = pack_frame(cmd, payload, self.counter.next())
        if exact_dlc:
            if len(frame) not in CANFD_DLC_LENGTHS:
                return False
            data = frame
        else:
            data = pad_canfd_payload(frame)
        self._log_tx(arb_id, cmd, frame, data)
        return self.can.send_message(arb_id, data, is_fd=True)

    def clear_iap_wait(self) -> None:
        with self._iap_lock:
            self._iap_responses.clear()
            self._iap_event.clear()

    def drain_iap_status(self, motor_id: int) -> int:
        """丢弃队列中该电机的 IAP_STATUS（如 FC 后的滞后 0x87），返回丢弃条数。"""
        drained = 0
        while True:
            if self._take_iap_response(motor_id, CMD.IAP_STATUS) is None:
                break
            drained += 1
        return drained

    def deliver_iap_response(self, motor_id: int, cmd: int, payload: bytes) -> None:
        with self._iap_lock:
            self._iap_responses.append((motor_id, int(cmd), payload))
            self._iap_event.set()

    def _take_iap_response(self, motor_id: int, cmd: int) -> Optional[bytes]:
        target_cmd = int(cmd)
        with self._iap_lock:
            for idx, (mid, rsp_cmd, payload) in enumerate(self._iap_responses):
                if mid == motor_id and rsp_cmd == target_cmd:
                    del self._iap_responses[idx]
                    if not self._iap_responses:
                        self._iap_event.clear()
                    return payload
        return None

    def wait_iap_response(self, motor_id: int, cmd: int, timeout: float) -> Optional[bytes]:
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            payload = self._take_iap_response(motor_id, cmd)
            if payload is not None:
                return payload
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                break
            self._iap_event.wait(timeout=min(0.05, remaining))
        return self._take_iap_response(motor_id, cmd)

    def poll_iap_error(self, motor_id: int, timeout: float = 0.05) -> Optional[bytes]:
        from protocol.version import iap_is_dex_v5_success, iap_wire_uses_dex_v5_format

        payload = self.wait_iap_response(motor_id, CMD.IAP_STATUS, timeout)
        if payload is None or len(payload) < 2:
            return None
        status, err = payload[0], payload[1]
        if iap_wire_uses_dex_v5_format(status, err):
            if status >= 6 or not iap_is_dex_v5_success(err):
                return payload
            return None
        if err != 0 or status >= 3:
            return payload
        return None

    # ---------- 广播命令 ----------

    def broadcast_sync(self) -> bool:
        return self._send(CAN_BROADCAST_ID, CMD.SYNC)

    def broadcast_enable(self, enable: bool) -> bool:
        return self._send(CAN_BROADCAST_ID, CMD.ENABLE, bytes([1 if enable else 0]))

    def broadcast_mode(self, mode: int) -> bool:
        return self._send(CAN_BROADCAST_ID, CMD.CONTROL_MODE, bytes([mode & 0xFF]))

    def broadcast_set_origin(self) -> bool:
        return self._send(CAN_BROADCAST_ID, CMD.SET_ORIGIN)

    def align_electric_angle(self, motor_id: int, method: int = 0) -> bool:
        """单播电角度对齐；method 见 ElecAlignMethod。"""
        return self._send(motor_id, CMD.ALIGN_ELECTRIC_ANGLE, bytes([method & 0xFF]))

    def broadcast_align_angle(self, method: int = 0) -> bool:
        """广播电角度对齐；method 见 ElecAlignMethod。"""
        return self._send(CAN_BROADCAST_ID, CMD.ALIGN_ELECTRIC_ANGLE, bytes([method & 0xFF]))

    def kth71_cal_start(self, motor_id: int) -> bool:
        return self._send(motor_id, CMD.KTH71_CAL_START)

    def kth71_cal_trigger(self, motor_id: int) -> bool:
        return self._send(motor_id, CMD.KTH71_CAL_TRIGGER)

    def kth71_cal_stop(self, motor_id: int) -> bool:
        return self._send(motor_id, CMD.KTH71_CAL_STOP)

    def kth71_cal_status(self, motor_id: int) -> bool:
        return self._send(motor_id, CMD.KTH71_CAL_STATUS)

    def broadcast_set_pos_pid(self, p: float, i: float, d: float) -> bool:
        return self._send(CAN_BROADCAST_ID, CMD.SET_POS_PID, pack_pid_params(p, i, d))

    def broadcast_set_vel_pid(self, p: float, i: float, d: float) -> bool:
        return self._send(CAN_BROADCAST_ID, CMD.SET_VEL_PID, pack_pid_params(p, i, d))

    def broadcast_set_cur_pid(self, p: float, i: float, d: float) -> bool:
        return self._send(CAN_BROADCAST_ID, CMD.SET_CUR_PID, pack_pid_params(p, i, d))

    def broadcast_set_stiff_damp(self, stiff: float, damp: float) -> bool:
        return self._send(CAN_BROADCAST_ID, CMD.SET_STIFF_DAMP, pack_stiff_damp(stiff, damp))

    def broadcast_set_protect(
        self, over_temp: int, over_current_ma: int, under_voltage_v: int, over_voltage_v: int
    ) -> bool:
        return self._send(
            CAN_BROADCAST_ID,
            CMD.SET_PROTECT,
            pack_protect(over_temp, over_current_ma, under_voltage_v, over_voltage_v),
        )

    def broadcast_get_pos_pid(self) -> bool:
        return self._send(CAN_BROADCAST_ID, CMD.GET_POS_PID)

    def broadcast_get_vel_pid(self) -> bool:
        return self._send(CAN_BROADCAST_ID, CMD.GET_VEL_PID)

    def broadcast_get_cur_pid(self) -> bool:
        return self._send(CAN_BROADCAST_ID, CMD.GET_CUR_PID)

    def broadcast_get_stiff_damp(self) -> bool:
        return self._send(CAN_BROADCAST_ID, CMD.GET_STIFF_DAMP)

    def broadcast_get_protect(self) -> bool:
        return self._send(CAN_BROADCAST_ID, CMD.GET_PROTECT)

    def broadcast_save_params(self) -> bool:
        return self._send(CAN_BROADCAST_ID, CMD.SAVE_PARAMS)

    def broadcast_reset_factory(self) -> bool:
        return self._send(CAN_BROADCAST_ID, CMD.RESET_TO_FACTORY)

    def broadcast_clear_fault(self) -> bool:
        return self._send(CAN_BROADCAST_ID, CMD.CLEAR_FAULT)

    def broadcast_fault_reset(self) -> bool:
        ok = self._send(CAN_BROADCAST_ID, CMD.FAULT_RESET)
        self.broadcast_enable(False)
        return ok

    # ---------- 多电机目标（广播） ----------

    def broadcast_target_positions(self, targets: List[Tuple[int, int]]) -> bool:
        """targets: [(motor_id, pos_raw), ...]"""
        if not targets:
            return False
        return self._send(CAN_BROADCAST_ID, CMD.TAR_POS, pack_multi_targets(targets))

    def broadcast_target_velocities(self, targets: List[Tuple[int, int]]) -> bool:
        return self._send(CAN_BROADCAST_ID, CMD.TAR_VEL, pack_multi_targets(targets))

    def broadcast_target_currents(self, targets: List[Tuple[int, int]]) -> bool:
        return self._send(CAN_BROADCAST_ID, CMD.TAR_CURR, pack_multi_targets(targets))

    def broadcast_target_pos_vel(self, targets: List[Tuple[int, int, int]]) -> bool:
        """targets: [(motor_id, pos_raw, vel), ...]"""
        payload = b""
        for mid, pos, vel in targets:
            payload += bytes([mid]) + pos.to_bytes(2, "big", signed=True) + vel.to_bytes(2, "big", signed=True)
        return self._send(CAN_BROADCAST_ID, CMD.TAR_POS_VEL, payload)

    # ---------- 单电机命令（仲裁 ID = motor_id） ----------

    def set_node_id(self, motor_id: int, new_id: int) -> bool:
        payload = new_id.to_bytes(2, "big")
        return self._send(motor_id, CMD.SET_NODE_ID, payload)

    def get_firmware_version(self, motor_id: int) -> bool:
        return self.iap_get_version(motor_id)

    # ---------- IAP（单电机，仲裁 ID = motor_id） ----------

    def iap_enter(self, motor_id: int) -> bool:
        if self._iap_session:
            return self._send_iap_v5(CMD.IAP_ENTER)
        return self._send(motor_id, CMD.IAP_ENTER)

    def iap_start(self, motor_id: int, payload: bytes, *, log: bool = True) -> bool:
        if self._iap_session:
            return self._send_iap_v5(CMD.IAP_START, payload, log=log)
        return self._send(motor_id, CMD.IAP_START, payload, exact_dlc=True)

    def iap_data(self, motor_id: int, chunk: bytes, *, log: bool = False) -> bool:
        if self._iap_session:
            return self._send_iap_v5(CMD.IAP_DATA, chunk, log=log)
        return self._send(motor_id, CMD.IAP_DATA, chunk, exact_dlc=True)

    def iap_end(self, motor_id: int) -> bool:
        if self._iap_session:
            return self._send_iap_v5(CMD.IAP_END)
        return self._send(motor_id, CMD.IAP_END)

    def iap_verify(self, motor_id: int) -> bool:
        if self._iap_session:
            return self._send_iap_v5(CMD.IAP_VERIFY)
        return self._send(motor_id, CMD.IAP_VERIFY)

    def iap_reset(self, motor_id: int) -> bool:
        if self._iap_session:
            return self._send_iap_v5(CMD.IAP_RESET)
        return self._send(motor_id, CMD.IAP_RESET)

    def iap_get_version(self, motor_id: int) -> bool:
        return self._send(motor_id, CMD.IAP_FW_VERSION)

    def iap_query_status(self, motor_id: int) -> bool:
        if self._iap_session:
            return self._send_iap_v5(CMD.IAP_STATUS, log=False)
        return self._send(motor_id, CMD.IAP_STATUS)

    def scan_motor(self, motor_id: int) -> bool:
        """向指定 ID 发送获取 PID 命令用于探测在线。"""
        return self._send(motor_id, CMD.GET_POS_PID)

    # ---------- 单电机参数读写（仲裁 ID = motor_id） ----------

    def get_pos_pid(self, motor_id: int) -> bool:
        return self._send(motor_id, CMD.GET_POS_PID)

    def get_vel_pid(self, motor_id: int) -> bool:
        return self._send(motor_id, CMD.GET_VEL_PID)

    def get_cur_pid(self, motor_id: int) -> bool:
        return self._send(motor_id, CMD.GET_CUR_PID)

    def get_stiff_damp(self, motor_id: int) -> bool:
        return self._send(motor_id, CMD.GET_STIFF_DAMP)

    def get_protect(self, motor_id: int) -> bool:
        return self._send(motor_id, CMD.GET_PROTECT)

    def read_all_params(self, motor_id: int) -> None:
        """读取单电机全部可配置参数（PID / 刚度阻尼 / 保护）。"""
        self.get_pos_pid(motor_id)
        self.get_vel_pid(motor_id)
        self.get_cur_pid(motor_id)
        self.get_stiff_damp(motor_id)
        self.get_protect(motor_id)

    def set_pos_pid(self, motor_id: int, p: float, i: float, d: float) -> bool:
        return self._send(motor_id, CMD.SET_POS_PID, pack_pid_params(p, i, d))

    def set_vel_pid(self, motor_id: int, p: float, i: float, d: float) -> bool:
        return self._send(motor_id, CMD.SET_VEL_PID, pack_pid_params(p, i, d))

    def set_cur_pid(self, motor_id: int, p: float, i: float, d: float) -> bool:
        print(f"set_cur_pid: p={p}, i={i}, d={d}")
        return self._send(motor_id, CMD.SET_CUR_PID, pack_pid_params(p, i, d))

    def set_stiff_damp(self, motor_id: int, stiff: float, damp: float) -> bool:
        return self._send(motor_id, CMD.SET_STIFF_DAMP, pack_stiff_damp(stiff, damp))

    def set_protect(
        self, motor_id: int, over_temp: int, over_current_ma: int, under_voltage_v: int, over_voltage_v: int
    ) -> bool:
        return self._send(
            motor_id,
            CMD.SET_PROTECT,
            pack_protect(over_temp, over_current_ma, under_voltage_v, over_voltage_v),
        )

    def save_params(self, motor_id: int) -> bool:
        return self._send(motor_id, CMD.SAVE_PARAMS)

    def reset_factory(self, motor_id: int) -> bool:
        return self._send(motor_id, CMD.RESET_TO_FACTORY)

    def clear_fault(self, motor_id: int) -> bool:
        return self._send(motor_id, CMD.CLEAR_FAULT)

    def fault_reset(self, motor_id: int) -> bool:
        ok = self._send(motor_id, CMD.FAULT_RESET)
        self._send(motor_id, CMD.ENABLE, bytes([0]))
        return ok
