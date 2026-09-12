"""主窗口。"""

import sys
import threading
import time
from typing import Dict, Optional

from PyQt5.QtCore import QTime, QTimer, pyqtSignal

from PyQt5.QtWidgets import (
    QApplication,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QMainWindow,
    QMessageBox,
    QSpinBox,
    QPushButton,
    QSplitter,
    QTextEdit,
    QVBoxLayout,
    QWidget,
)

from canbus.device_manager import DeviceState
from canbus.interface import CanThread
from core.motor_client import MotorClient
from protocol import (
    CMD,
    format_bytes_binary,
    format_bytes_hex,
    parse_bool_result,
    parse_frame,
    parse_fw_version_payload,
    parse_iap_status_payload,
    parse_node_id_response,
    parse_period_report,
    parse_pid_response,
    parse_protect_response,
    parse_stiff_damp_response,
    parse_protocol_version,
)
from ui.connection_panel import ConnectionPanel
from ui.motor_panel import MotorPanel
from ui.waveform_widget import WaveformWidget

STATUS_UI_INTERVAL = 0.05  # 20 Hz


class MainWindow(QMainWindow):
    _log_signal = pyqtSignal(str)

    def __init__(self):
        super().__init__()
        self._log_signal.connect(self._append_log)
        self.can_thread = CanThread()
        self.client = MotorClient(self.can_thread, log_fn=self.log)
        self.motor_panel: Optional[MotorPanel] = None
        self._motor_id = 1
        self._scan_pending = False
        self._status_latest: Dict[int, dict] = {}
        self._status_last_ui: Dict[int, float] = {}
        self._init_ui()
        self._init_connections()

    def _init_ui(self) -> None:
        self.setWindowTitle(self._window_title())
        self.setMinimumSize(1280, 900)

        central = QWidget()
        root = QVBoxLayout()

        self.connection_panel = ConnectionPanel(
            device_info_fn=lambda: self.can_thread.manager.device_info_text()
        )
        root.addWidget(self.connection_panel)

        mgr = QGroupBox("电机")
        mgr_layout = QHBoxLayout()
        self.motor_id_spin = QSpinBox()
        self.motor_id_spin.setRange(1, 0x7FE)
        self.motor_id_spin.setValue(1)
        self.motor_id_spin.valueChanged.connect(self._on_motor_id_changed)
        self.scan_btn = QPushButton("扫描在线 (1-21)")
        mgr_layout.addWidget(QLabel("Node-ID:"))
        mgr_layout.addWidget(self.motor_id_spin)
        mgr_layout.addWidget(self.scan_btn)
        mgr_layout.addStretch()
        mgr.setLayout(mgr_layout)
        root.addWidget(mgr)

        # 主体：中间波形 + 右侧控制
        body_splitter = QSplitter()
        body_splitter.setChildrenCollapsible(False)

        self.waveform_widget = WaveformWidget()
        body_splitter.addWidget(self.waveform_widget)

        right_panel = QWidget()
        right_layout = QVBoxLayout()
        right_layout.setContentsMargins(0, 0, 0, 0)

        self.motor_panel_host = QWidget()
        self.motor_panel_layout = QVBoxLayout()
        self.motor_panel_layout.setContentsMargins(0, 0, 0, 0)
        self.motor_panel_host.setLayout(self.motor_panel_layout)
        right_layout.addWidget(self.motor_panel_host)
        right_panel.setLayout(right_layout)
        body_splitter.addWidget(right_panel)

        body_splitter.setStretchFactor(0, 3)
        body_splitter.setStretchFactor(1, 2)
        body_splitter.setSizes([780, 420])
        root.addWidget(body_splitter, 1)

        log_group = QGroupBox("通信日志")
        log_layout = QVBoxLayout()
        self.log_text = QTextEdit()
        self.log_text.setReadOnly(True)
        self.log_text.setMaximumHeight(120)
        log_layout.addWidget(self.log_text)
        log_group.setLayout(log_layout)
        root.addWidget(log_group)

        central.setLayout(root)
        self.setCentralWidget(central)

    def _init_connections(self) -> None:
        panel = self.connection_panel
        panel.open_device_requested.connect(self._on_open_device)
        panel.scan_devices_requested.connect(self._on_scan_devices)
        panel.start_channel_requested.connect(self._on_start_channel)
        panel.stop_channel_requested.connect(self._on_stop_channel)
        panel.close_device_requested.connect(self._on_close_device)
        panel.protocol_version_changed.connect(self._on_protocol_version_changed)
        self.scan_btn.clicked.connect(self._scan_motors)
        self.can_thread.connection_status.connect(self._on_connection_status)
        self.can_thread.scan_finished.connect(self._on_scan_finished)
        self.can_thread.scan_failed.connect(self._on_scan_failed)
        self.can_thread.msg_received.connect(self._handle_can_message)

    def _make_target_callback(self, motor_id: int):
        def _cb(pos=None, vel=None, curr=None):
            self.waveform_widget.set_target(motor_id, pos=pos, vel=vel, iq=curr)
        return _cb

    def _window_title(self) -> str:
        ver = self.client.protocol_version.value
        return f"电机调试上位机 (CANFD) — 协议 V{ver}"

    def log(self, msg: str) -> None:
        """线程安全：任意线程可调用，实际写入在主线程执行。"""
        self._log_signal.emit(msg)

    def _append_log(self, msg: str) -> None:
        self.log_text.append(f"[{QTime.currentTime().toString('HH:mm:ss')}] {msg}")

    def _on_protocol_version_changed(self, version_text: str) -> None:
        try:
            version = parse_protocol_version(version_text)
        except ValueError as exc:
            self.log(str(exc))
            return
        self.client.set_protocol_version(version)
        if self.motor_panel:
            self.motor_panel.apply_protocol_version(version)
        self.waveform_widget.apply_protocol_version(version)
        self.setWindowTitle(self._window_title())
        unit = "rad" if version.value == "2.0.0" else "°"
        self.log(f"固件协议已切换为 V{version.value}（位置单位: {unit}）")

    def _log_rx(self, arb_id: int, cnt: int, cmd: int, data: bytes) -> None:
        try:
            cmd_name = CMD(cmd).name
        except ValueError:
            cmd_name = f"CMD_{cmd:02X}"
        self.log(
            f"接收 NodeID=0x{arb_id:03X} [CNT=0x{cnt:02X}] CMD=0x{cmd:02X}({cmd_name}) "
            f"LEN={len(data)} "
            f"HEX=[{format_bytes_hex(data)}] "
            f"BIN=[{format_bytes_binary(data)}]"
        )

    def _on_connection_status(self, ok: bool, msg: str) -> None:
        if "打开设备" in msg or "设备已打开" in msg or (not ok and "打开" in msg):
            self.connection_panel.set_busy_opening(False)
        self.log(msg)
        if ok and "已启动" in msg:
            ch = self.can_thread.manager.channel
            self.connection_panel.set_device_state(
                DeviceState.RUNNING,
                self.can_thread.manager.device_label(),
                f"通道{ch + 1} (CAN{ch}) 运行中",
                active_channel=ch,
            )
            if self.motor_panel is None:
                self._ensure_motor_panel(self._motor_id)
            QTimer.singleShot(100, self._sync_motor_params)
        elif ok and "已打开" in msg:
            mgr = self.can_thread.manager
            warning = mgr.last_open_warning
            profile = mgr.profile
            if mgr._zlg is not None:
                self.connection_panel.apply_hardware_channels(mgr._zlg.channel_count, warning)
            self.connection_panel.set_device_state(
                DeviceState.OPENED,
                mgr.device_label(),
                warning or "设备已打开，请启动通道",
            )
            if profile and profile.auto_start_channel:
                # ZQWL 经 COM 枚举，等设备信息稳定后再 InitCAN
                QTimer.singleShot(200, lambda: self.can_thread.start_channel(0))
            if warning:
                QMessageBox.warning(
                    self,
                    "设备信息提示",
                    f"{warning}\n\n硬件 CAN 通道数: {mgr._zlg.channel_count if mgr._zlg else 1}",
                )
            elif mgr._zlg is not None and mgr._zlg.channel_count >= 2:
                self.log(
                    f"已识别双通道 USBCANFD-200U："
                    f"通道1=API CAN0，通道2=API CAN1，"
                    f"序列号 {mgr._zlg.hardware_serial(mgr._zlg.device_info)}"
                )
        elif not ok and ("失败" in msg or "已关闭" in msg):
            detail = msg.split(":", 1)[-1].strip() if ":" in msg else msg
            self.connection_panel.set_device_state(DeviceState.CLOSED, message=detail)

    def _on_open_device(self, profile_name: str, device_index: int, serial_port: str) -> None:
        self.connection_panel.set_busy_opening(True)
        self.log(f"正在打开设备 ({profile_name})…")
        self.can_thread.open_device(profile_name, device_index, serial_port)

    def _on_scan_devices(self, profile_name: str) -> None:
        self.log(f"正在扫描设备 ({profile_name})…")

        def task() -> None:
            try:
                indices, message = self.can_thread.manager.scan_devices(profile_name)
                self.can_thread.scan_finished.emit(indices, message or "")
            except Exception as exc:
                self.can_thread.scan_failed.emit(str(exc))

        threading.Thread(target=task, daemon=True).start()

    def _on_scan_finished(self, indices: list, message: str) -> None:
        self.connection_panel.set_scan_result(indices, message)
        if indices:
            detail = f"，{message}" if message else ""
            self.log(f"扫描完成，发现设备索引: {indices}{detail}")
        else:
            self.log(f"扫描完成: {message or '未发现设备'}")

    def _on_scan_failed(self, err: str) -> None:
        self.log(f"扫描失败: {err}")

    def _on_start_channel(self, channel: int) -> None:
        self.can_thread.start_channel(channel)

    def _on_stop_channel(self) -> None:
        self.can_thread.stop_channel()
        self.connection_panel.set_device_state(
            DeviceState.OPENED,
            self.can_thread.manager.device_label(),
            "通道已停止",
        )

    def _on_close_device(self) -> None:
        self.can_thread.close_device()
        self.connection_panel.set_device_state(DeviceState.CLOSED, message="未连接")

    def _on_motor_id_changed(self, motor_id: int) -> None:
        self._motor_id = motor_id
        if self.motor_panel is not None:
            self.motor_panel.motor_id = motor_id
            self.motor_panel.control_widget.motor_id = motor_id
            self.motor_panel.param_widget.motor_id = motor_id
            self.motor_panel.iap_widget.set_motor_id(motor_id)
            self.motor_panel.status_widget.set_motor_id(motor_id)
        self.waveform_widget.set_motor(motor_id)
        self.log(f"当前电机 Node-ID: {motor_id}")

    def _sync_motor_params(self) -> None:
        if not self.can_thread.manager.is_running or self.motor_panel is None:
            return
        self.client.read_all_params(self._motor_id)
        self.log(f"已请求同步电机 {self._motor_id} 参数")

    def _ensure_motor_panel(self, motor_id: int) -> None:
        if self.motor_panel is not None:
            return
        panel = MotorPanel(
            motor_id,
            self.client,
            log_fn=self.log,
            target_callback=self._make_target_callback(motor_id),
        )
        self.motor_panel = panel
        self.motor_panel_layout.addWidget(panel)
        self.waveform_widget.set_motor(motor_id)
        self.log(f"已加载电机面板 (Node-ID={motor_id})")

    def _set_scan_motor_id(self, motor_id: int) -> None:
        self.motor_id_spin.blockSignals(True)
        self.motor_id_spin.setValue(motor_id)
        self.motor_id_spin.blockSignals(False)
        self._on_motor_id_changed(motor_id)

    def _scan_motors(self) -> None:
        self._scan_pending = True
        self.log("开始扫描电机 ID 1~21 ...")
        for mid in range(1, 22): 
            self.client.scan_motor(mid)

    def _maybe_update_status(self, motor_id: int, report: dict) -> None:
        if motor_id != self._motor_id or self.motor_panel is None:
            return
        self._status_latest[motor_id] = report
        now = time.perf_counter()
        last = self._status_last_ui.get(motor_id, 0.0)
        if now - last < STATUS_UI_INTERVAL:
            return
        self.motor_panel.update_status(report)
        self._status_last_ui[motor_id] = now

    def _handle_can_message(self, arb_id: int, data: bytes, timestamp: float) -> None:
        parsed = parse_frame(data)
        if parsed is None:
            if data:
                self.log(
                    f"接收 NodeID=0x{arb_id:03X} LEN={len(data)} "
                    f"HEX=[{format_bytes_hex(data)}] BIN=[{format_bytes_binary(data)}] (帧格式无效)"
                )
            return
        cnt, cmd, payload = parsed

        if cmd == CMD.PERIOD_REPORT:
            report = parse_period_report(payload, self.client.protocol_version)
            if report:
                if self._scan_pending:
                    self._set_scan_motor_id(arb_id)
                    self._ensure_motor_panel(arb_id)
                if arb_id == self._motor_id:
                    self._maybe_update_status(arb_id, report)
                self.waveform_widget.append_sample(arb_id, report, timestamp=timestamp)
            return
        # else:
        #     print(f"cmd: {cmd}")
        #     print(f"payload: {payload}")
        #     print(f"data: {data}")
        #     print(f"arb_id: {arb_id}")
        #     print(f"cnt: {cnt}")
        #     print(f"timestamp: {timestamp}")
        #     print(f"parsed: {parsed}")

        self._log_rx(arb_id, cnt, cmd, data)

        if cmd == CMD.IAP_STATUS:
            self.client.deliver_iap_response(arb_id, cmd, payload)
            iap = parse_iap_status_payload(self.client.protocol_version, payload)
            if iap:
                err = iap.get("error_name", "")
                self.log(
                    f"电机 {arb_id} IAP: {iap.get('iap_state_name', '?')}"
                    + (f"，{err}" if err and err != "正常" else "")
                )
            if arb_id == self._motor_id and self.motor_panel:
                self.motor_panel.on_iap_status(payload)
            return

        if cmd == CMD.IAP_FW_VERSION:
            version = parse_fw_version_payload(payload)
            self.client.deliver_iap_response(arb_id, cmd, payload)
            self.log(f"电机 {arb_id} 固件版本: {version or payload.hex()}")
            if self._scan_pending:
                self._set_scan_motor_id(arb_id)
                self._ensure_motor_panel(arb_id)
            if arb_id == self._motor_id and self.motor_panel:
                self.motor_panel.on_firmware_version(version)
            return

        panel = self.motor_panel if arb_id == self._motor_id else None

        if cmd in (CMD.GET_POS_PID, CMD.SET_POS_PID):
            pid = parse_pid_response(payload)
            if panel:
                panel.apply_pid("pos", pid)
            self.log(f"电机 {arb_id} 位置PID: P={pid.get('P', 0):.4f} I={pid.get('I', 0):.4f} D={pid.get('D', 0):.4f}")
            if self._scan_pending:
                self._set_scan_motor_id(arb_id)
                self._ensure_motor_panel(arb_id)
        elif cmd in (CMD.GET_VEL_PID, CMD.SET_VEL_PID):
            pid = parse_pid_response(payload)
            if panel:
                panel.apply_pid("vel", pid)
            self.log(f"电机 {arb_id} 速度PID: P={pid.get('P', 0):.4f} I={pid.get('I', 0):.4f} D={pid.get('D', 0):.4f}")
        elif cmd in (CMD.GET_CUR_PID, CMD.SET_CUR_PID):
            pid = parse_pid_response(payload)
            if panel:
                panel.apply_pid("cur", pid)
            self.log(f"电机 {arb_id} 电流PID: P={pid.get('P', 0):.4f} I={pid.get('I', 0):.4f} D={pid.get('D', 0):.4f}")
        elif cmd in (CMD.GET_STIFF_DAMP, CMD.SET_STIFF_DAMP):
            sd = parse_stiff_damp_response(payload)
            if panel:
                panel.apply_stiff_damp(sd)
            self.log(
                f"电机 {arb_id} 刚度阻尼: stiff={sd.get('stiffness', 0):.4f} damp={sd.get('damping', 0):.4f}"
            )
        elif cmd in (CMD.GET_PROTECT, CMD.SET_PROTECT):
            prot = parse_protect_response(payload)
            if panel:
                panel.apply_protect(prot)
            self.log(
                f"电机 {arb_id} 保护: OTP={prot.get('over_temperature')} "
                f"OCP={prot.get('over_current_ma')}mA UVLO={prot.get('under_voltage_V')}V "
                f"OVP={prot.get('over_voltage_V')}V"
            )
        elif cmd == CMD.SAVE_PARAMS:
            ok = parse_bool_result(payload)
            self.log(f"电机 {arb_id} 保存参数: {'成功' if ok else '失败'}")
        elif cmd == CMD.RESET_TO_FACTORY:
            ok = parse_bool_result(payload)
            self.log(f"电机 {arb_id} 恢复出厂: {'成功' if ok else '失败'}")
        elif cmd == CMD.CLEAR_FAULT:
            ok = parse_bool_result(payload)
            self.log(f"电机 {arb_id} 清除故障: {'成功' if ok else '无锁存故障'}")
        elif cmd == CMD.FAULT_RESET:
            ok = parse_bool_result(payload)
            self.log(f"电机 {arb_id} 异常复位: {'成功' if ok else '失败'}")
        elif cmd == CMD.ALIGN_ELECTRIC_ANGLE:
            ok = parse_bool_result(payload)
            self.log(f"电机 {arb_id} 电角度对齐: {'成功' if ok else '失败'}")
        elif cmd == CMD.SET_NODE_ID:
            new_id = parse_node_id_response(payload)
            if new_id is not None:
                self.log(f"电机 {arb_id} Node-ID 已设为 {new_id}（保存参数后重启生效）")
        else:
            self.log(f"电机 {arb_id} 未处理命令 CMD=0x{cmd:02X} payload={payload.hex()}")


def run() -> None:
    app = QApplication(sys.argv)
    win = MainWindow()
    win.show()
    sys.exit(app.exec_())
