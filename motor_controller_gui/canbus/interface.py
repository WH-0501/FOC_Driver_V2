"""CAN/CANFD 通信线程。"""

import threading
import time
from collections import deque
from protocol import CMD

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
        self._telemetry_lock = threading.Lock()
        self._traffic_lock = threading.Lock()
        self._telemetry = deque(maxlen=32768)
        self.telemetry_dropped = 0
        self.telemetry_stale = 0
        self.last_send_ms = 0.0
        self.traffic_tx_frames = 0
        self.traffic_tx_bytes = 0
        self.traffic_rx_frames = 0
        self.traffic_rx_bytes = 0

    def reset_traffic_stats(self) -> None:
        with self._traffic_lock:
            self.traffic_tx_frames = 0
            self.traffic_tx_bytes = 0
            self.traffic_rx_frames = 0
            self.traffic_rx_bytes = 0

    def _record_tx(self, nbytes: int) -> None:
        with self._traffic_lock:
            self.traffic_tx_frames += 1
            self.traffic_tx_bytes += max(0, int(nbytes))

    def _record_rx(self, nbytes: int) -> None:
        with self._traffic_lock:
            self.traffic_rx_frames += 1
            self.traffic_rx_bytes += max(0, int(nbytes))

    def snapshot_traffic(self) -> tuple[int, int, int, int]:
        with self._traffic_lock:
            return (
                self.traffic_tx_frames,
                self.traffic_tx_bytes,
                self.traffic_rx_frames,
                self.traffic_rx_bytes,
            )

    def clear_telemetry(self):
        with self._telemetry_lock:
            self._telemetry.clear()
        self.reset_traffic_stats()

    def take_telemetry(self, limit=1024, max_age=None, with_received_time=False):
        with self._telemetry_lock:
            if max_age is not None:
                cutoff = time.monotonic() - max_age
                while self._telemetry and self._telemetry[0][3] < cutoff:
                    self._telemetry.popleft()
                    self.telemetry_stale += 1
                    self.telemetry_dropped += 1
            items = [self._telemetry.popleft() for _ in range(min(limit, len(self._telemetry)))]
            return items if with_received_time else [item[:3] for item in items]

    @property
    def telemetry_age_ms(self):
        with self._telemetry_lock:
            return max(0.0, (time.monotonic()-self._telemetry[0][3])*1000) if self._telemetry else 0.0

    @property
    def telemetry_pending(self):
        with self._telemetry_lock:
            return len(self._telemetry)

    @property
    def bus(self):
        return self.manager.bus

    def _is_device_open(self) -> bool:
        return self.manager.is_device_open

    def open_device(self, profile_name: str, device_index: int = 0, serial_port: str = "") -> None:
        def action() -> None:
            with self._lifecycle_lock:
                self.clear_telemetry()
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
        started = time.perf_counter()
        try:
            with self._send_lock:
                ok = self.manager.send(arbitration_id, data, is_fd)
            if ok:
                self._record_tx(len(data))
            return ok
        finally:
            self.last_send_ms = (time.perf_counter()-started)*1000

    def send_messages_burst(self, frames: list[tuple[int, bytes]], is_fd: bool = True) -> bool:
        """连续发送多帧；发送/接收分锁，避免周期上报阻塞 IAP。"""
        with self._send_lock:
            ok = self.manager.send_burst(frames, is_fd)
        if ok:
            for _arb_id, payload in frames:
                self._record_tx(len(payload))
        return ok

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
                        data = bytes(msg.data)
                        self._record_rx(len(data))
                        if len(data) >= 2 and data[1] == CMD.PERIOD_REPORT:
                            with self._telemetry_lock:
                                if len(self._telemetry) == self._telemetry.maxlen:
                                    self.telemetry_dropped += 1
                                self._telemetry.append((msg.arbitration_id, data, msg.timestamp, time.monotonic()))
                        else:
                            self.msg_received.emit(msg.arbitration_id, data, msg.timestamp)
                    drained += len(batch)
                if drained == 0:
                    self.msleep(5)
            except Exception as exc:
                if self.running:
                    print(f"接收错误: {exc}")
