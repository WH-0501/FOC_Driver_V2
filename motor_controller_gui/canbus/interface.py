"""CAN/CANFD 通信线程。"""

from typing import Optional

from PyQt5.QtCore import QThread, pyqtSignal

from canbus.device_manager import CanDeviceManager, DeviceState


class CanThread(QThread):
    """后台 CAN 接收线程，配合 CanDeviceManager 使用。"""

    msg_received = pyqtSignal(int, bytes, float)
    connection_status = pyqtSignal(bool, str)

    def __init__(self, parent=None):
        super().__init__(parent)
        self.manager = CanDeviceManager()
        self.running = False

    @property
    def bus(self):
        return self.manager.bus

    def _is_device_open(self) -> bool:
        return self.manager.is_device_open

    def open_device(self, profile_name: str, device_index: int = 0, serial_port: str = "") -> None:
        try:
            self.manager.open_device(profile_name, device_index, serial_port)
            self.connection_status.emit(True, f"设备已打开: {self.manager.device_label()}")
        except Exception as exc:
            self.connection_status.emit(False, f"打开设备失败: {exc}")

    def start_channel(self, channel: int = 0) -> None:
        try:
            if not self._is_device_open():
                raise RuntimeError("请先打开设备")
            self.manager.start_channel(channel)
            if not self.isRunning():
                self.running = True
                self.start()
            self.connection_status.emit(True, f"CAN{channel} 已启动")
        except Exception as exc:
            self.connection_status.emit(False, f"启动通道失败: {exc}")

    def stop_channel(self) -> None:
        self.running = False
        self.wait(2000)
        self.manager.stop_channel()
        self.connection_status.emit(True, "通道已停止")

    def close_device(self) -> None:
        self.running = False
        self.wait(2000)
        self.manager.close_device()
        self.connection_status.emit(False, "设备已关闭")

    def send_message(self, arbitration_id: int, data: bytes, is_fd: bool = True) -> bool:
        return self.manager.send(arbitration_id, data, is_fd)

    def run(self) -> None:
        while self.running and self.manager.is_running:
            try:
                drained = 0
                while drained < 512:
                    wait = 0.05 if drained == 0 else 0.0
                    batch = self.manager.recv_batch(timeout=wait, max_messages=64)
                    if not batch:
                        break
                    for msg in batch:
                        self.msg_received.emit(msg.arbitration_id, bytes(msg.data), msg.timestamp)
                    drained += len(batch)
                if drained == 0:
                    self.msleep(5)
            except Exception as exc:
                if self.running:
                    print(f"接收错误: {exc}")
