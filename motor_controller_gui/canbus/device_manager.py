"""CAN 设备连接状态管理。"""

from enum import Enum, auto
from typing import List, Optional, Tuple

import can

from canbus.device_profiles import DEVICE_PROFILES, DeviceProfile
from canbus.zlg_native import ZlgCanSession, find_can_driver_dll


class DeviceState(Enum):
    CLOSED = auto()
    OPENED = auto()
    RUNNING = auto()


class CanDeviceManager:
    def __init__(self) -> None:
        self.profile: Optional[DeviceProfile] = None
        self.device_index: int = 0
        self.bus: Optional[can.BusABC] = None
        self._zlg: Optional[ZlgCanSession] = None
        self.state = DeviceState.CLOSED
        self.channel = 0
        self._serial_port: str = ""
        self._resolved_device_type: Optional[int] = None
        self.last_error: str = ""
        self.last_scan_message: str = ""
        self.last_open_warning: str = ""

    @property
    def is_running(self) -> bool:
        return self.state == DeviceState.RUNNING and self._backend_ready()

    def _backend_ready(self) -> bool:
        if self.profile and self.profile.backend == "zlgcan":
            return self._zlg is not None and self._zlg.channel_handle != 0
        return self.bus is not None

    @property
    def is_device_open(self) -> bool:
        return self.state in (DeviceState.OPENED, DeviceState.RUNNING) and (
            self._zlg is not None or self.bus is not None
        )

    def set_profile(self, profile_name: str) -> None:
        for item in DEVICE_PROFILES:
            if item.name == profile_name:
                self.profile = item
                return
        raise ValueError(f"未知设备型号: {profile_name}")

    def scan_devices(self, profile_name: str, max_index: int = 4) -> Tuple[List[int], str]:
        self.set_profile(profile_name)
        if self.profile.backend != "zlgcan":
            self.last_scan_message = ""
            return [0], ""

        device_type = self.profile.device_type or 42
        found, message, resolved_type = ZlgCanSession.scan(
            device_type,
            max_index,
            expected_channels=self.profile.channel_count,
            driver_dll=self.profile.driver_dll,
        )
        self._resolved_device_type = resolved_type if resolved_type == device_type else device_type
        self.last_scan_message = message
        return found, message

    def open_device(self, profile_name: str, device_index: int = 0, serial_port: str = "") -> None:
        self.close_device()
        self.set_profile(profile_name)
        self.device_index = device_index
        self._serial_port = serial_port
        self.last_error = ""
        self.last_open_warning = ""

        if self.profile.backend == "zlgcan":
            dll_path, err, _backend = find_can_driver_dll(self.profile.driver_dll)
            if dll_path is None:
                raise RuntimeError(err)
            self._zlg = ZlgCanSession()
            self._zlg.open_device(
                self.profile.device_type or 42,
                device_index,
                resolved_type=self._resolved_device_type,
                channel_count=self.profile.channel_count,
                abit_baud=self.profile.bitrate,
                dbit_baud=self.profile.data_bitrate,
                resistance=self.profile.resistance,
                driver_dll=self.profile.driver_dll,
            )
            self.last_open_warning = self._zlg.open_type_mismatch
        elif self.profile.backend == "slcan":
            if not serial_port:
                raise ValueError("请选择串口")
            channel = f"{serial_port}@{self.profile.tty_baudrate}"
            self.bus = can.Bus(
                interface="slcan",
                channel=channel,
                bitrate=self.profile.bitrate,
                fd=self.profile.fd,
                data_bitrate=self.profile.data_bitrate,
            )
        elif self.profile.backend == "pcan":
            self.bus = can.Bus(
                interface="pcan",
                channel=self.profile.default_channel,
                bitrate=self.profile.bitrate,
                fd=self.profile.fd,
                data_bitrate=self.profile.data_bitrate,
            )
        else:
            raise ValueError(f"不支持的后端: {self.profile.backend}")

        self.state = DeviceState.OPENED

    def start_channel(self, channel: int = 0) -> None:
        if self.profile is None:
            raise RuntimeError("未选择设备型号")
        self.channel = channel

        if self.profile.backend == "zlgcan":
            if self._zlg is None:
                raise RuntimeError("请先打开设备")
            self._zlg.start_channel(
                channel,
                abit_baud=self.profile.bitrate,
                dbit_baud=self.profile.data_bitrate,
                resistance=self.profile.resistance,
            )
        elif self.bus is None:
            raise RuntimeError("请先打开设备")

        self.state = DeviceState.RUNNING

    def stop_channel(self) -> None:
        if self._zlg is not None:
            self._zlg.stop_channel()
        self.state = DeviceState.OPENED

    def close_device(self) -> None:
        if self._zlg is not None:
            self._zlg.close_device()
            self._zlg = None
        if self.bus is not None:
            try:
                self.bus.shutdown()
            except Exception:
                pass
            self.bus = None
        self._resolved_device_type = None
        self.state = DeviceState.CLOSED

    def recv(self, timeout: float = 0.1) -> Optional[can.Message]:
        if not self.is_running:
            return None
        try:
            if self._zlg is not None:
                item = self._zlg.recv(timeout)
                if item is None:
                    return None
                arb_id, data, ts = item
                return can.Message(
                    arbitration_id=arb_id,
                    data=data,
                    is_extended_id=False,
                    is_fd=True,
                    timestamp=ts,
                )
            if self.bus is not None:
                return self.bus.recv(timeout=timeout)
        except Exception as exc:
            self.last_error = str(exc)
            return None
        return None

    def recv_batch(self, timeout: float = 0.05, max_messages: int = 64) -> list:
        """连续读取多帧，避免周期上报占满缓冲导致命令应答丢失。"""
        if not self.is_running:
            return []
        try:
            if self._zlg is not None:
                items = self._zlg.recv_batch(max_messages=max_messages, timeout=timeout)
                return [
                    can.Message(
                        arbitration_id=arb_id,
                        data=data,
                        is_extended_id=False,
                        is_fd=True,
                        timestamp=ts,
                    )
                    for arb_id, data, ts in items
                ]
            messages = []
            while len(messages) < max_messages:
                wait = timeout if not messages else 0.0
                msg = self.recv(timeout=wait)
                if msg is None:
                    break
                messages.append(msg)
            return messages
        except Exception as exc:
            self.last_error = str(exc)
            return []

    def send(self, arbitration_id: int, data: bytes, is_fd: bool = True) -> bool:
        if not self.is_running:
            return False
        try:
            if self._zlg is not None:
                return self._zlg.send(arbitration_id, data, is_fd)
            if self.bus is not None:
                self.bus.send(
                    can.Message(
                        arbitration_id=arbitration_id,
                        data=data,
                        is_extended_id=False,
                        is_fd=is_fd,
                    )
                )
                return True
        except Exception as exc:
            self.last_error = str(exc)
            return False
        return False

    def device_label(self) -> str:
        if not self.profile:
            return "未选择设备"
        if self._zlg is not None:
            label = self._zlg.device_label(self.profile.name)
            if self.state == DeviceState.RUNNING:
                ch = self.channel
                return f"{label}  通道{ch + 1}"
            return label
        return f"{self.profile.name}  设备{self.device_index}"

    def device_info_text(self) -> str:
        if self._zlg and self._zlg.device_info:
            info = self._zlg.device_info
            serial = ZlgCanSession.hardware_serial(info)
            hw_type = ZlgCanSession.hardware_name(info)
            dll_name = ""
            dll_dir = ""
            if self._zlg.api is not None:
                dll_name = self._zlg.api.dll_path.name
                dll_dir = str(self._zlg.api._dll_dir)
            lines = [
                f"硬件类型: {hw_type}",
                f"序列号: {serial}",
                f"CAN 通道数: {info.can_Num}",
                f"设备类型: {ZlgCanSession.hardware_type_label(self._zlg.device_type)}",
                f"设备类型号: {self._zlg.device_type}",
            ]
            if dll_name:
                lines.append(f"驱动库: {dll_name} ({dll_dir})")
            if self._zlg.open_type_mismatch:
                lines.extend(["", f"注意: {self._zlg.open_type_mismatch}"])
            if self.profile and self.profile.channel_count != self._zlg.channel_count:
                lines.extend([
                    "",
                    f"注意: 当前 Profile 为 {self.profile.channel_count} 通道，"
                    f"硬件仅 {self._zlg.channel_count} 通道；请使用通道 1 (API CAN0)",
                ])
            if self.state == DeviceState.RUNNING:
                ch = self.channel
                lines.extend([
                    "",
                    f"当前活动通道: 通道{ch + 1} (API CAN{ch})",
                    f"通道句柄: {self._zlg.channel_handle:#x}",
                ])
            elif self.state == DeviceState.OPENED:
                lines.append("")
                lines.append("当前活动通道: 未启动（请在界面选择通道后点「启动通道」）")
            if self.profile:
                lines.append("")
                lines.append(
                    self._zlg.channel_config_summary(
                        self.profile.bitrate,
                        self.profile.data_bitrate,
                        self.profile.resistance,
                    )
                )
            return "\n".join(lines)
        if self.profile:
            return (
                f"型号: {self.profile.name}\n"
                f"CANFD 标准: ISO\n"
                f"仲裁段: {self.profile.bitrate // 1000} kbps\n"
                f"数据段: {self.profile.data_bitrate // 1000} kbps\n"
                f"终端电阻: {'开启' if self.profile.resistance else '关闭'}"
            )
        return "无设备信息"
