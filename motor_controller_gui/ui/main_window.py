"""主窗口。"""

import sys
import time
from typing import Dict, Optional

from PyQt5.QtCore import QTime, QTimer
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
    QTabWidget,
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
    parse_node_id_response,
    parse_period_report,
    parse_pid_response,
    parse_protect_response,
    parse_stiff_damp_response,
)
from ui.connection_panel import ConnectionPanel
from ui.motor_panel import MotorPanel
from ui.multi_motor_widget import MultiMotorWidget
from ui.waveform_widget import WaveformWidget

STATUS_UI_INTERVAL = 0.05  # 20 Hz


class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.can_thread = CanThread()
        self.client = MotorClient(self.can_thread, log_fn=self.log)
        self.motor_panels: Dict[int, MotorPanel] = {}
        self._scan_pending = False
        self._status_latest: Dict[int, dict] = {}
        self._status_last_ui: Dict[int, float] = {}
        self._init_ui()
        self._init_connections()

    def _init_ui(self) -> None:
        self.setWindowTitle("电机调试上位机 (CANFD)")
        self.setMinimumSize(1280, 900)

        central = QWidget()
        root = QVBoxLayout()

        self.connection_panel = ConnectionPanel(
            device_info_fn=lambda: self.can_thread.manager.device_info_text()
        )
        root.addWidget(self.connection_panel)

        mgr = QGroupBox("电机管理")
        mgr_layout = QHBoxLayout()
        self.motor_id_spin = QSpinBox()
        self.motor_id_spin.setRange(1, 0x7FE)
        self.motor_id_spin.setValue(1)
        self.add_motor_btn = QPushButton("添加电机")
        self.remove_motor_btn = QPushButton("移除当前电机")
        self.scan_btn = QPushButton("扫描在线电机 (1-21)")
        mgr_layout.addWidget(QLabel("电机 ID:"))
        mgr_layout.addWidget(self.motor_id_spin)
        mgr_layout.addWidget(self.add_motor_btn)
        mgr_layout.addWidget(self.remove_motor_btn)
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

        self.main_tabs = QTabWidget()
        self.motor_tab = QTabWidget()
        self.motor_tab.currentChanged.connect(self._on_motor_tab_changed)
        self.main_tabs.addTab(self.motor_tab, "单电机控制")

        self.multi_motor_widget = MultiMotorWidget(
            self.client,
            get_motor_ids=lambda: list(self.motor_panels.keys()),
            log_fn=self.log,
        )
        self.main_tabs.addTab(self.multi_motor_widget, "多电机批量")
        right_layout.addWidget(self.main_tabs)
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
        self.add_motor_btn.clicked.connect(self._add_motor)
        self.remove_motor_btn.clicked.connect(self._remove_motor)
        self.scan_btn.clicked.connect(self._scan_motors)
        self.can_thread.connection_status.connect(self._on_connection_status)
        self.can_thread.msg_received.connect(self._handle_can_message)

    def _make_target_callback(self, motor_id: int):
        def _cb(pos=None, vel=None, curr=None):
            self.waveform_widget.set_target(motor_id, pos=pos, vel=vel, iq=curr)
        return _cb

    def _on_motor_tab_changed(self, index: int) -> None:
        if index < 0:
            self.waveform_widget.set_motor(None)
            return
        text = self.motor_tab.tabText(index)
        try:
            motor_id = int(text.split()[-1])
            self.waveform_widget.set_motor(motor_id)
        except ValueError:
            self.waveform_widget.set_motor(None)

    def log(self, msg: str) -> None:
        self.log_text.append(f"[{QTime.currentTime().toString('HH:mm:ss')}] {msg}")

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
        self.log(msg)
        if ok and "已启动" in msg:
            self.connection_panel.set_device_state(
                DeviceState.RUNNING,
                self.can_thread.manager.device_label(),
                "通道已全部启动",
            )
            if not self.motor_panels:
                self._add_motor_by_id(1)
            QTimer.singleShot(100, self._sync_all_motor_params)
        elif ok and "已打开" in msg:
            self.connection_panel.set_device_state(
                DeviceState.OPENED,
                self.can_thread.manager.device_label(),
                "设备已打开，请启动通道",
            )
        elif not ok and ("失败" in msg or "已关闭" in msg):
            detail = msg.split(":", 1)[-1].strip() if ":" in msg else msg
            self.connection_panel.set_device_state(DeviceState.CLOSED, message=detail)

    def _on_open_device(self, profile_name: str, device_index: int, serial_port: str) -> None:
        self.can_thread.open_device(profile_name, device_index, serial_port)

    def _on_scan_devices(self, profile_name: str) -> None:
        try:
            indices, message = self.can_thread.manager.scan_devices(profile_name)
            self.connection_panel.set_scan_result(indices, message)
            if indices:
                self.log(f"扫描完成，发现设备索引: {indices}")
            else:
                self.log(f"扫描完成: {message or '未发现设备'}")
        except Exception as exc:
            QMessageBox.warning(self, "扫描失败", str(exc))

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

    def _add_motor(self) -> None:
        self._add_motor_by_id(self.motor_id_spin.value())

    def _sync_all_motor_params(self) -> None:
        if not self.can_thread.manager.is_running or not self.motor_panels:
            return
        for motor_id in self.motor_panels:
            self.client.read_all_params(motor_id)
        self.log(f"已请求同步 {len(self.motor_panels)} 台电机参数")

    def _add_motor_by_id(self, motor_id: int) -> None:
        if motor_id in self.motor_panels:
            self.log(f"电机 ID {motor_id} 已存在")
            return
        panel = MotorPanel(
            motor_id,
            self.client,
            log_fn=self.log,
            target_callback=self._make_target_callback(motor_id),
        )
        self.motor_panels[motor_id] = panel
        self.motor_tab.addTab(panel, f"电机 {motor_id}")
        self.motor_tab.setCurrentWidget(panel)
        self.waveform_widget.set_motor(motor_id)
        self.log(f"添加电机 ID {motor_id}")

    def _remove_motor(self) -> None:
        idx = self.motor_tab.currentIndex()
        if idx < 0:
            return
        text = self.motor_tab.tabText(idx)
        try:
            motor_id = int(text.split()[-1])
        except ValueError:
            return
        if motor_id in self.motor_panels:
            del self.motor_panels[motor_id]
        self._status_latest.pop(motor_id, None)
        self._status_last_ui.pop(motor_id, None)
        self.motor_tab.removeTab(idx)
        self.log(f"移除电机 {motor_id}")
        self._on_motor_tab_changed(self.motor_tab.currentIndex())

    def _scan_motors(self) -> None:
        self._scan_pending = True
        self.log("开始扫描电机 ID 1~21 ...")
        for mid in range(1, 22):
            self.client.scan_motor(mid)

    def _maybe_update_status(self, motor_id: int, report: dict) -> None:
        if motor_id not in self.motor_panels:
            return
        self._status_latest[motor_id] = report
        now = time.perf_counter()
        last = self._status_last_ui.get(motor_id, 0.0)
        if now - last < STATUS_UI_INTERVAL:
            return
        self.motor_panels[motor_id].update_status(report)
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
            report = parse_period_report(payload)
            if report:
                if arb_id not in self.motor_panels and self._scan_pending:
                    self._add_motor_by_id(arb_id)
                if arb_id in self.motor_panels:
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

        panel = self.motor_panels.get(arb_id)

        if cmd in (CMD.GET_POS_PID, CMD.SET_POS_PID):
            pid = parse_pid_response(payload)
            if panel:
                panel.apply_pid("pos", pid)
            self.log(f"电机 {arb_id} 位置PID: P={pid.get('P', 0):.4f} I={pid.get('I', 0):.4f} D={pid.get('D', 0):.4f}")
            if self._scan_pending and arb_id not in self.motor_panels:
                self._add_motor_by_id(arb_id)
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
        elif cmd == CMD.ALIGN_ELECTRIC_ANGLE:
            ok = parse_bool_result(payload)
            self.log(f"电机 {arb_id} 电角度对齐: {'成功' if ok else '失败'}")
        elif cmd == CMD.SET_NODE_ID:
            new_id = parse_node_id_response(payload)
            if new_id is not None:
                self.log(f"电机 {arb_id} Node-ID 已设为 {new_id}（保存参数后重启生效）")
        elif cmd == CMD.IAP_FW_VERSION:
            version = payload.decode("ascii", errors="replace") if payload else ""
            self.log(f"电机 {arb_id} 固件版本: {version or payload.hex()}")
            if self._scan_pending and arb_id not in self.motor_panels:
                self._add_motor_by_id(arb_id)
        elif cmd == CMD.IAP_STATUS:
            self.log(f"电机 {arb_id} IAP 状态: {payload.hex()}")
        else:
            self.log(f"电机 {arb_id} 未处理命令 CMD=0x{cmd:02X} payload={payload.hex()}")


def run() -> None:
    app = QApplication(sys.argv)
    win = MainWindow()
    win.show()
    sys.exit(app.exec_())
