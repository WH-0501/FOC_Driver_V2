"""高层 CAN 电机通讯客户端。"""

from typing import Callable, Dict, List, Optional, Tuple

from canbus.interface import CanThread
from protocol import (
    CAN_BROADCAST_ID,
    CAN_IAP_ID,
    CMD,
    FrameCounter,
    format_bytes_binary,
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
        self._log_fn = log_fn

    def _log_tx(self, arb_id: int, cmd: CMD, frame: bytes, data: bytes) -> None:
        if self._log_fn is None:
            return
        cnt = frame[0] if frame else 0
        cmd_name = cmd.name if isinstance(cmd, CMD) else f"CMD_{int(cmd):02X}"
        self._log_fn(
            f"下发 NodeID=0x{arb_id:03X} "
            f"[CNT=0x{cnt:02X} 帧计数] CMD=0x{int(cmd):02X}({cmd_name}) "
            f"LEN={len(data)} "
            f"HEX=[{format_bytes_hex(data)}] "
            # f"BIN=[{format_bytes_binary(data)}]"
        )

    def _send(self, arb_id: int, cmd: CMD, payload: bytes = b"") -> bool:
        frame = pack_frame(cmd, payload, self.counter.next())
        data = pad_canfd_payload(frame)
        self._log_tx(arb_id, cmd, frame, data)
        return self.can.send_message(arb_id, data, is_fd=True)

    # ---------- 广播命令 ----------

    def broadcast_sync(self) -> bool:
        return self._send(CAN_BROADCAST_ID, CMD.SYNC)

    def broadcast_enable(self, enable: bool) -> bool:
        return self._send(CAN_BROADCAST_ID, CMD.ENABLE, bytes([1 if enable else 0]))

    def broadcast_mode(self, mode: int) -> bool:
        return self._send(CAN_BROADCAST_ID, CMD.CONTROL_MODE, bytes([mode & 0xFF]))

    def broadcast_set_origin(self) -> bool:
        return self._send(CAN_BROADCAST_ID, CMD.SET_ORIGIN)

    def broadcast_align_angle(self) -> bool:
        return self._send(CAN_BROADCAST_ID, CMD.ALIGN_ELECTRIC_ANGLE)

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
        return self._send(motor_id, CMD.IAP_FW_VERSION)

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
