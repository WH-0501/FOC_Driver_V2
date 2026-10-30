"""主窗口。"""

import sys
import threading
import time
from typing import Dict, Optional

from PyQt5.QtCore import Qt, QTime, QTimer, pyqtSignal

from PyQt5.QtWidgets import (
    QApplication,
    QDialog,
    QGroupBox,
    QDockWidget,
    QMenu,
    QScrollArea,
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
    ErrorCode,
    KTH71_ANLC_STATUS_NAMES,
    format_align_result_lines,
    format_api_error_detail,
    format_bytes_binary,
    format_bytes_hex,
    parse_align_result,
    parse_api_result,
    parse_bool_result,
    parse_kth71_anlc_status,
    parse_motor_param_response,
    parse_frame,
    parse_fw_version_payload,
    parse_iap_status_payload,
    parse_node_id_response,
    parse_period_report,
    parse_cur_pid_response,
    parse_pid_response,
    parse_protect_response,
    parse_stiff_damp_response,
    parse_protocol_version,
)
from protocol.version import mode_display_name
from ui.connection_panel import ConnectionPanel
from ui.motor_panel import MotorPanel
from ui.waveform_widget import WaveformWidget

STATUS_UI_INTERVAL = 0.05  # 20 Hz
TRAFFIC_RATE_INTERVAL_S = 0.5


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
        self._fault_log_state = {}
        self._status_received_at = {}
        self._status_latest: Dict[int, dict] = {}
        self._status_last_ui: Dict[int, float] = {}
        self._status_displayed: Dict[int, dict] = {}
        self._status_pending: Dict[int, bool] = {}
        self._traffic_sample: Optional[tuple] = None
        self._traffic_display = (0.0, 0.0, 0.0, 0.0)
        self._init_ui()
        self._init_connections()
        self._ensure_motor_panel(self._motor_id)
        self._on_motor_id_changed(self._motor_id)

    def _init_ui(self) -> None:
        self.setWindowTitle(self._window_title())
        self.setMinimumSize(1100, 700)
        self.resize(1500, 900)

        central = QWidget()
        root = QVBoxLayout()

        self.connection_panel = ConnectionPanel(
            device_info_fn=lambda: self.can_thread.manager.device_info_text()
        )
        self.connection_dialog = QDialog(self)
        self.connection_dialog.setWindowTitle("连接与设备")
        connection_layout = QVBoxLayout(self.connection_dialog)
        connection_layout.addWidget(self.connection_panel)
        self.connection_dialog.resize(850, 260)
        self.connection_menu = self.menuBar().addMenu("连接与设备")
        self.connection_settings_action = self.connection_menu.addAction("连接设置…")
        self.connection_settings_action.triggered.connect(self._show_connection_settings)
        self.connection_menu.addSeparator()
        self._connection_actions = []
        for label, button in (("打开设备", self.connection_panel.open_btn),
                              ("启动通道", self.connection_panel.start_btn),
                              ("停止通道", self.connection_panel.stop_btn),
                              ("关闭设备", self.connection_panel.close_btn),
                              ("设备信息…", self.connection_panel.info_btn)):
            action = self.connection_menu.addAction(label)
            action.triggered.connect(lambda checked=False, b=button: b.click())
            self._connection_actions.append((action, button))
        self.connection_menu.aboutToShow.connect(self._update_connection_menu)
        self.analysis_menu = self.menuBar().addMenu("分析")
        self.step_analysis_action = self.analysis_menu.addAction("阶跃响应分析…")
        self.step_analysis_action.triggered.connect(lambda: self.waveform_widget._show_step_analysis())
        self._view_menu = self.menuBar().addMenu("视图")

        # Inline selector in the menu bar; no dedicated motor-management row.
        motor_corner = QWidget(self)
        mgr_layout = QHBoxLayout(motor_corner)
        mgr_layout.setContentsMargins(4, 0, 4, 0)
        self.motor_id_spin = QSpinBox()
        self.motor_id_spin.setRange(1, 0x7FE)
        self.motor_id_spin.setValue(1)
        self.motor_id_spin.valueChanged.connect(self._on_motor_id_changed)
        self.scan_btn = QPushButton("扫描在线 (1-21)")
        mgr_layout.addWidget(QLabel("Node-ID:"))
        mgr_layout.addWidget(self.motor_id_spin)
        mgr_layout.addWidget(self.scan_btn)
        self.menuBar().setCornerWidget(motor_corner, Qt.TopRightCorner)

        # 主体：中间波形 + 右侧控制
        body_splitter = QSplitter()
        body_splitter.setChildrenCollapsible(False)

        self.waveform_widget = WaveformWidget()
        body_splitter.addWidget(self.waveform_widget)

        right_panel = QWidget()
        right_panel.setMinimumWidth(320)
        right_layout = QVBoxLayout()
        right_layout.setContentsMargins(0, 0, 0, 0)

        self.motor_panel_host = QWidget()
        self.motor_panel_layout = QVBoxLayout()
        self.motor_panel_layout.setContentsMargins(0, 0, 0, 0)
        self.motor_panel_host.setLayout(self.motor_panel_layout)
        right_layout.addWidget(self.motor_panel_host)
        right_panel.setLayout(right_layout)
        control_scroll = QScrollArea()
        control_scroll.setWidgetResizable(True)
        control_scroll.setWidget(right_panel)
        body_splitter.addWidget(control_scroll)

        body_splitter.setStretchFactor(0, 7)
        body_splitter.setStretchFactor(1, 3)
        body_splitter.setSizes([900, 300])
        root.addWidget(body_splitter, 1)

        log_group = QGroupBox("通信日志")
        log_layout = QVBoxLayout()
        self.log_text = QTextEdit()
        self.log_text.setReadOnly(True)
        self.log_text.document().setMaximumBlockCount(2000)
        self.log_text.setContextMenuPolicy(Qt.CustomContextMenu)
        self.log_text.customContextMenuRequested.connect(self._show_log_context_menu)
        log_layout.addWidget(self.log_text)
        log_group.setLayout(log_layout)
        self.log_dock = QDockWidget("通信日志", self)
        self.log_dock.setObjectName("logDock")
        self.log_dock.setWidget(log_group)
        # 可拖动停靠；不用 Closable/Floatable，避免标题栏关闭/浮动按钮
        self.log_dock.setFeatures(QDockWidget.DockWidgetMovable)
        self.addDockWidget(Qt.BottomDockWidgetArea, self.log_dock)
        self.log_dock_view_action = self._view_menu.addAction("通信日志")
        self.log_dock_view_action.setCheckable(True)
        self.log_dock_view_action.setChecked(True)
        self.log_dock_view_action.triggered.connect(self._on_log_dock_view_toggled)
        self.log_dock.visibilityChanged.connect(self._sync_log_dock_view_action)

        central.setLayout(root)
        self.setCentralWidget(central)

    def _show_connection_settings(self):
        self.connection_dialog.show()
        self.connection_dialog.raise_()
        self.connection_dialog.activateWindow()

    def _on_log_dock_view_toggled(self, checked: bool) -> None:
        self.log_dock.setVisible(checked)

    def _sync_log_dock_view_action(self, visible: bool) -> None:
        if self.log_dock_view_action.isChecked() != visible:
            self.log_dock_view_action.blockSignals(True)
            self.log_dock_view_action.setChecked(visible)
            self.log_dock_view_action.blockSignals(False)

    def _update_connection_menu(self):
        for action, button in self._connection_actions:
            action.setEnabled(button.isEnabled())

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
        self._receive_timer = QTimer(self)
        self._receive_timer.timeout.connect(self._drain_telemetry)
        self._receive_timer.start(16)

        self._status_flush_timer = QTimer(self)
        self._status_flush_timer.setInterval(int(STATUS_UI_INTERVAL * 1000))
        self._status_flush_timer.timeout.connect(self._flush_pending_status)
        self._status_flush_timer.start()

    def _drain_telemetry(self):
        self.client.expire_command_acks()
        # Yield regularly to buttons and command responses, even during overload.
        deadline = time.perf_counter() + .004
        while True:
            batch = self.can_thread.take_telemetry(64, max_age=.250, with_received_time=True)
            for arb_id, data, timestamp, received_at in batch:
                self._handle_can_message(arb_id, data, timestamp, received_at)
            if len(batch) < 64 or time.perf_counter() >= deadline:
                break
        self._update_traffic_rates()
        self.statusBar().showMessage(self._status_bar_message())

    def _update_traffic_rates(self) -> None:
        now = time.monotonic()
        tx_f, tx_b, rx_f, rx_b = self.can_thread.snapshot_traffic()
        if self._traffic_sample is None:
            self._traffic_sample = (now, tx_f, tx_b, rx_f, rx_b)
            return
        t0, f0, b0, rf0, rb0 = self._traffic_sample
        dt = now - t0
        if dt < TRAFFIC_RATE_INTERVAL_S:
            return
        if dt > 0.0:
            self._traffic_display = (
                (tx_f - f0) / dt,
                (tx_b - b0) / dt / 1024.0,
                (rx_f - rf0) / dt,
                (rx_b - rb0) / dt / 1024.0,
            )
        self._traffic_sample = (now, tx_f, tx_b, rx_f, rx_b)

    def _traffic_status_text(self) -> str:
        # 以上位机为中心：下行=本机发送(TX)，上行=本机接收(RX)
        if not self.can_thread.manager.is_running:
            return "下行(发) — · 上行(收) —"
        tx_fps, tx_kbs, rx_fps, rx_kbs = self._traffic_display
        return (
            f"下行(发) {tx_fps:.1f} fps · {tx_kbs:.1f} KB/s | "
            f"上行(收) {rx_fps:.1f} fps · {rx_kbs:.1f} KB/s"
        )

    def _status_bar_message(self) -> str:
        ct = self.can_thread
        queue_part = (
            f"接收队列 {ct.telemetry_pending}/32768 | "
            f"队列最旧 {ct.telemetry_age_ms:.1f} ms | "
            f"过载/过期丢弃 {ct.telemetry_dropped} 帧（过期 {ct.telemetry_stale}） | "
            f"最近发送调用 {ct.last_send_ms:.1f} ms"
        )
        return f"{self._traffic_status_text()}  ||  {queue_part}"

    def closeEvent(self, event):
        self.can_thread.running = False
        if not self.can_thread.wait(2000):
            event.ignore()
            self.log("接收线程尚未退出，请稍后关闭")
            return
        self.can_thread.manager.close_device()
        event.accept()

    def _status_force_update(self, motor_id: int, report: dict) -> bool:
        """错误码、使能、模式变化时立即刷新 UI。"""
        prev = self._status_displayed.get(motor_id)
        if prev is None:
            return True
        for key in ("error_code", "enable", "mode"):
            if report.get(key) != prev.get(key):
                return True
        return False

    def _apply_status_ui(self, motor_id: int, report: dict) -> None:
        if self.motor_panel is None or motor_id != self._motor_id:
            return
        self.motor_panel.update_status(report)
        self._status_last_ui[motor_id] = time.perf_counter()
        self._status_displayed[motor_id] = {
            "error_code": report.get("error_code"),
            "enable": report.get("enable"),
            "mode": report.get("mode"),
        }
        self._status_pending[motor_id] = False

    def _flush_pending_status(self) -> None:
        motor_id = self._motor_id
        if self.motor_panel is not None:
            age = time.perf_counter() - self._status_received_at.get(motor_id, float("-inf"))
            self.motor_panel.status_widget.set_feedback_stale(age > 1.0)
        if not self._status_pending.get(motor_id):
            return
        report = self._status_latest.get(motor_id)
        if report is not None:
            self._apply_status_ui(motor_id, report)

    def _make_target_callback(self, motor_id: int):
        def _cb(pos=None, vel=None, curr=None):
            self.waveform_widget.set_target(self._motor_id, pos=pos, vel=vel, iq=curr)
        return _cb

    def _window_title(self) -> str:
        ver = self.client.protocol_version.value
        return f"电机调试工作台 · CAN / CAN-FD — 协议 V{ver}"

    def log(self, msg: str) -> None:
        """线程安全：任意线程可调用，实际写入在主线程执行。"""
        self._log_signal.emit(msg)

    def _log_api_result(
        self,
        arb_id: int,
        action: str,
        payload: bytes,
        *,
        no_fault_label: Optional[str] = None,
    ) -> None:
        ok, code = parse_api_result(payload)
        if ok:
            self.log(f"电机 {arb_id} {action}: 成功")
            return
        if no_fault_label and code == int(ErrorCode.ERR_NO_FAULT):
            self.log(f"电机 {arb_id} {action}: {no_fault_label} (error_t={code})")
            return
        self.log(f"电机 {arb_id} {action}: {format_api_error_detail(code)}")

    def _append_log(self, msg: str) -> None:
        from protocol.time_util import format_log_timestamp

        self.log_text.append(f"[{format_log_timestamp()}] {msg}")

    def _clear_comm_log(self) -> None:
        self.log_text.clear()

    def _show_log_context_menu(self, pos) -> None:
        menu = self.log_text.createStandardContextMenu()
        if menu is None:
            menu = QMenu(self)
        menu.addSeparator()
        clear_action = menu.addAction("清除日志")
        clear_action.triggered.connect(self._clear_comm_log)
        menu.exec_(self.log_text.mapToGlobal(pos))

    def _on_protocol_version_changed(self, version_text: str) -> None:
        try:
            version = parse_protocol_version(version_text)
        except ValueError as exc:
            self.log(str(exc))
            return
        self.can_thread.clear_telemetry()
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
            self._fault_log_state.clear()
            self._status_received_at.clear()
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
        self._status_displayed.pop(motor_id, None)
        self._status_pending.pop(motor_id, None)
        if self.motor_panel is not None:
            self.motor_panel.motor_id = motor_id
            self.motor_panel.control_widget.motor_id = motor_id
            self.motor_panel.param_widget.motor_id = motor_id
            self.motor_panel.control_widget.speed_limits.reset()
            self.motor_panel.iap_widget.set_motor_id(motor_id)
            self.motor_panel.status_widget.set_motor_id(motor_id)
            self.motor_panel.cal_widget.set_motor_id(motor_id)
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
        self._scan_pending = False
        self.motor_id_spin.blockSignals(True)
        self.motor_id_spin.setValue(motor_id)
        self.motor_id_spin.blockSignals(False)
        self._on_motor_id_changed(motor_id)

    def _scan_motors(self) -> None:
        self._scan_pending = True
        self.log("开始扫描电机 ID 1~21 ...")
        for mid in range(1, 22): 
            self.client.scan_motor(mid)

    def _log_fault_change(self, motor_id, report):
        key = report.get("error_code")
        previous = self._fault_log_state.get(motor_id)
        if key == previous:
            return
        self._fault_log_state[motor_id] = key
        if key or (previous is not None and previous):
            self.log(f"电机 {motor_id} 故障状态: {report.get('error_name', '--')}")

    def _maybe_update_status(self, motor_id: int, report: dict) -> None:
        if motor_id != self._motor_id or self.motor_panel is None:
            return
        self._status_received_at[motor_id] = time.perf_counter()
        self._status_latest[motor_id] = report
        force = self._status_force_update(motor_id, report)
        now = time.perf_counter()
        last = self._status_last_ui.get(motor_id, 0.0)
        if not force and (now - last) < STATUS_UI_INTERVAL:
            self._status_pending[motor_id] = True
            return
        self._apply_status_ui(motor_id, report)

    def _handle_can_message(self, arb_id: int, data: bytes, timestamp: float, received_at=None) -> None:
        parsed = parse_frame(data)
        if parsed is None:
            if data:
                self.log(
                    f"接收 NodeID=0x{arb_id:03X} LEN={len(data)} "
                    f"HEX=[{format_bytes_hex(data)}] BIN=[{format_bytes_binary(data)}] (帧格式无效)"
                )
            return
        cnt, cmd, payload = parsed

        if cmd == CMD.COMMAND_ACK:
            self.client.command_ack(arb_id, payload)
            return

        if cmd == CMD.PERIOD_REPORT:
            report = parse_period_report(payload, self.client.protocol_version)
            if report:
                self._log_fault_change(arb_id, report)
                if self._scan_pending:
                    self._set_scan_motor_id(arb_id)
                    self._ensure_motor_panel(arb_id)
                if arb_id == self._motor_id:
                    self._maybe_update_status(arb_id, report)
                self.waveform_widget.append_sample(arb_id, report, timestamp=timestamp, received_at=received_at)
            return
        # else:
        #     print(f"cmd: {cmd}")
        #     print(f"payload: {payload}")
        #     print(f"data: {data}")
        #     print(f"arb_id: {arb_id}")
        #     print(f"cnt: {cnt}")
        #     print(f"timestamp: {timestamp}")
        #     print(f"parsed: {parsed}")

        if cmd not in (CMD.VERIFY_SAMPLE, CMD.GET_SPEED_LIMITS):
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

        if cmd in (CMD.GET_SPEED_LIMITS, CMD.SET_CURRENT_SOFT_SPEED):
            if panel:
                panel.control_widget.speed_limits.receive(payload, cmd == CMD.SET_CURRENT_SOFT_SPEED)
            return
        if cmd == CMD.VERIFY_SAMPLE:
            if panel:
                panel.cal_widget.show_verify_sample(payload)
            return
        if cmd == CMD.VERIFY_ELECTRICAL:
            if panel:
                panel.cal_widget.show_verify_result(payload)
            return

        if cmd in (CMD.GET_POS_PID, CMD.SET_POS_PID):
            pid = parse_pid_response(payload, self.client.protocol_version)
            if panel:
                panel.apply_pid("pos", pid)
            self.log(f"电机 {arb_id} 位置PID: P={pid.get('P', 0):.4f} I={pid.get('I', 0):.4f} D={pid.get('D', 0):.4f}")
            if self._scan_pending:
                self._set_scan_motor_id(arb_id)
                self._ensure_motor_panel(arb_id)
        elif cmd in (CMD.GET_VEL_PID, CMD.SET_VEL_PID):
            pid = parse_pid_response(payload, self.client.protocol_version)
            if panel:
                panel.apply_pid("vel", pid)
            self.log(f"电机 {arb_id} 速度PID: P={pid.get('P', 0):.4f} I={pid.get('I', 0):.4f} D={pid.get('D', 0):.4f}")
        elif cmd in (CMD.GET_CUR_PID, CMD.SET_CUR_PID):
            pid = parse_cur_pid_response(payload, self.client.protocol_version)
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
        elif cmd == CMD.GET_MOTOR_PARAM:
            mp = parse_motor_param_response(payload)
            if mp is not None and panel:
                panel.apply_motor_param(mp)
                self.log(
                    f"电机 {arb_id} 本体参数: pp={mp.get('pole_pairs')} "
                    f"dir={mp.get('direction')} gear={mp.get('gear_ratio'):.3f}"
                )
            elif mp is None:
                self.log(f"电机 {arb_id} 本体参数: 应答无效 (len={len(payload)})")
        elif cmd == CMD.SAVE_PARAMS:
            ok = parse_bool_result(payload)
            if panel:
                panel.apply_save_params_result(ok)
            if ok:
                self.log(f"电机 {arb_id} 保存参数: 成功")
            else:
                raw = payload[0] if payload else None
                self.log(
                    f"电机 {arb_id} 保存参数: 存储失败"
                    + (f" (应答={raw})" if raw is not None else " (无应答)")
                )
        elif cmd == CMD.RESET_TO_FACTORY:
            ok = parse_bool_result(payload)
            if ok:
                self.log(f"电机 {arb_id} 恢复全参数出厂: 成功")
                if arb_id == self._motor_id:
                    self.client.read_all_params(arb_id)
            else:
                raw = payload[0] if payload else None
                self.log(
                    f"电机 {arb_id} 恢复全参数出厂: 失败"
                    + (f" (应答={raw})" if raw is not None else " (无应答)")
                )
        elif cmd == CMD.RESET_PID_TO_FACTORY:
            ok = parse_bool_result(payload)
            if ok:
                self.log(f"电机 {arb_id} 恢复 PID 出厂: 成功")
                if arb_id == self._motor_id:
                    self.client.read_all_params(arb_id)
            else:
                raw = payload[0] if payload else None
                self.log(
                    f"电机 {arb_id} 恢复 PID 出厂: 失败"
                    + (f" (应答={raw})" if raw is not None else " (无应答)")
                )
        elif cmd in (CMD.ENABLE, CMD.CONTROL_MODE):
            self._log_api_result(arb_id, '使能/失能' if cmd == CMD.ENABLE else '设置模式', payload)
        elif cmd == CMD.CLEAR_FAULT:
            self._log_api_result(arb_id, "清除故障", payload, no_fault_label="无锁存故障")
        elif cmd == CMD.FAULT_RESET:
            self._log_api_result(arb_id, "异常复位", payload)
        elif cmd == CMD.ALIGN_ELECTRIC_ANGLE:
            align = parse_align_result(payload)
            if panel:
                panel.apply_align_result(align)
            self.log(f"电机 {arb_id} 电角度对齐: " + "；".join(format_align_result_lines(align)))
            if align.get("ok"):
                self.client.get_motor_param(arb_id)
        elif cmd == CMD.KTH71_CAL_START:
            self._log_api_result(arb_id, "KTH71 开始校准", payload)
        elif cmd == CMD.KTH71_CAL_TRIGGER:
            self._log_api_result(arb_id, "KTH71 触发 ANLC", payload)
        elif cmd == CMD.KTH71_CAL_STOP:
            self._log_api_result(arb_id, "KTH71 停止校准", payload)
        elif cmd == CMD.KTH71_CAL_STATUS:
            status = parse_kth71_anlc_status(payload)
            if status is None:
                self.log(f"电机 {arb_id} KTH71 校准状态: 应答无效 (payload={payload.hex()})")
            else:
                name = KTH71_ANLC_STATUS_NAMES.get(status, f"未知状态")
                self.log(f"电机 {arb_id} KTH71 校准状态: {name} (anlc={status})")
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
