"""CAN/CANFD 通信线程。"""

import threading

from PyQt5.QtCore import QThread, pyqtSignal

from canbus.device_manager import CanDeviceManager, DeviceState


class CanThread(QThread):
    """后台 CAN 接收线程，配合 CanDeviceManager 使用。"""

    msg_received = pyqtSignal(int, bytes, float)
    connection_status = pyqtSignal(bool, str)
    scan_finished = pyqtSignal(list, str)
    scan_failed = pyqtSignal(str)

    def __init__(self, parent=None):
        super().__init__(parent)
        self.manager = CanDeviceManager()
        self.running = False
        self._send_lock = threading.Lock()
        self._recv_lock = threading.Lock()
        self._lifecycle_lock = threading.Lock()

    @property
    def bus(self):
        return self.manager.bus

    def _is_device_open(self) -> bool:
        return self.manager.is_device_open

    def open_device(self, profile_name: str, device_index: int = 0, serial_port: str = "") -> None:
        def action() -> None:
            with self._lifecycle_lock:
                self.manager.open_device(profile_name, device_index, serial_port)
            self.connection_status.emit(True, f"设备已打开: {self.manager.device_label()}")

        def task() -> None:
            try:
                action()
            except Exception as exc:
                self.connection_status.emit(False, f"打开设备失败: {exc}")

        threading.Thread(target=task, daemon=True).start()

    def start_channel(self, channel: int = 0) -> None:
        def task() -> None:
            try:
                with self._lifecycle_lock:
                    if not self._is_device_open():
                        raise RuntimeError("请先打开设备")
                    self.manager.start_channel(channel)
                if not self.isRunning():
                    self.running = True
                    self.start()
                zlg = self.manager._zlg
                handle_hint = f", handle={zlg.channel_handle:#x}" if zlg and zlg.channel_handle else ""
                self.connection_status.emit(
                    True,
                    f"通道{channel + 1} (API CAN{channel}) 已启动{handle_hint}",
                )
            except Exception as exc:
                self.connection_status.emit(False, f"启动通道失败: {exc}")

        threading.Thread(target=task, daemon=True).start()

    def stop_channel(self) -> None:
        def task() -> None:
            self.running = False
            self.wait(2000)
            with self._lifecycle_lock:
                self.manager.stop_channel()
            self.connection_status.emit(True, "通道已停止")

        threading.Thread(target=task, daemon=True).start()

    def close_device(self) -> None:
        def task() -> None:
            self.running = False
            self.wait(2000)
            with self._lifecycle_lock:
                self.manager.close_device()
            self.connection_status.emit(False, "设备已关闭")

        threading.Thread(target=task, daemon=True).start()

    def send_message(self, arbitration_id: int, data: bytes, is_fd: bool = True) -> bool:
        with self._send_lock:
            return self.manager.send(arbitration_id, data, is_fd)

    def send_messages_burst(self, frames: list[tuple[int, bytes]], is_fd: bool = True) -> bool:
        """连续发送多帧；发送/接收分锁，避免周期上报阻塞 IAP。"""
        with self._send_lock:
            return self.manager.send_burst(frames, is_fd)

    def run(self) -> None:
        while self.running and self.manager.is_running:
            try:
                drained = 0
                while drained < 512:
                    wait = 0.05 if drained == 0 else 0.0
                    with self._recv_lock:
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
