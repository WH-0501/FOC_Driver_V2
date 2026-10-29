"""Output-shaft RPM limits. The absolute limit is read-only firmware data."""
import struct
import time
from PyQt5.QtCore import QTimer
from PyQt5.QtWidgets import (
    QCheckBox,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QPushButton,
    QSpinBox,
    QVBoxLayout,
)
from protocol import api_error_name

SPEED_LIMIT_FLAG_HARD_OVERSPEED = 0x0001


def parse_speed_limits(payload):
    if len(payload) < 20 or payload[0] not in (2, 3):
        return None
    version, error, hard, soft, mode, status, rpm, requested, applied = struct.unpack_from(
        ">BBHHBBiii", payload
    )
    if not 0 <= soft <= hard or hard == 0:
        return None
    active = bool(status & 1)
    if version >= 3:
        hard_enabled = bool(status & 2)
    else:
        hard_enabled = True
    return dict(
        error=error,
        hard=hard,
        soft=soft,
        mode=mode,
        active=active,
        hard_overspeed_enabled=hard_enabled,
        rpm=rpm / 1000,
        requested=requested / 1000,
        applied=applied / 1000,
    )


class SpeedLimitWidget(QGroupBox):
    def __init__(self, motor_id_fn, client, parent=None):
        super().__init__("电流 / 扭矩模式最大速度", parent)
        self.motor_id_fn = motor_id_fn
        self.client = client
        self.pending = 0
        self.dirty = False
        self.read_pending = 0
        self.confirmed = False
        layout = QVBoxLayout(self)
        self.hard = QLabel("固件硬限速：待读取（不可修改）")
        layout.addWidget(self.hard)
        self.hard_protect = QCheckBox("使能固件硬超速保护（电机轴 3000 RPM，锁存故障并关断）")
        self.hard_protect.setChecked(True)
        layout.addWidget(self.hard_protect)
        row = QHBoxLayout()
        row.addWidget(QLabel("输出轴最大速度:"))
        self.soft = QSpinBox()
        self.soft.setRange(0, 3000)
        self.soft.setSuffix(" RPM")
        self.soft.setSpecialValueText("关闭软限速")
        self.soft.setValue(54)
        row.addWidget(self.soft)
        layout.addLayout(row)
        row = QHBoxLayout()
        self.read_btn = QPushButton("读取")
        self.apply_btn = QPushButton("发送限速")
        row.addWidget(self.read_btn)
        row.addWidget(self.apply_btn)
        layout.addLayout(row)
        hint = QLabel(
            "达到软限速的 90% 开始削减加速电流，保留制动电流。\n"
            "0 仅关闭软限速。硬超速保护可通过上方勾选关闭（RAM，不保存 Flash）。\n"
            "软/硬配置可运行中调整，重启后恢复默认。"
        )
        hint.setWordWrap(True)
        layout.addWidget(hint)
        self.live = QLabel("输出轴转速 —  用户 Iq —  限制后 Iq —")
        self.live.setWordWrap(True)
        layout.addWidget(self.live)
        self.status = QLabel("可直接编辑并发送；设备当前值尚未确认")
        self.status.setWordWrap(True)
        layout.addWidget(self.status)
        self.read_btn.clicked.connect(self.read)
        self.apply_btn.clicked.connect(self.apply)
        self.soft.valueChanged.connect(self._changed)
        self.hard_protect.toggled.connect(self._changed)
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.poll)
        self.timer.start(500)

    def _changed(self):
        self.dirty = True

    def reset(self):
        self.pending = 0
        self.dirty = False
        self.read_pending = 0
        self.confirmed = False
        self.soft.setEnabled(True)
        self.apply_btn.setEnabled(True)
        self.soft.blockSignals(True)
        self.soft.setValue(54)
        self.soft.blockSignals(False)
        self.hard_protect.blockSignals(True)
        self.hard_protect.setChecked(True)
        self.hard_protect.blockSignals(False)
        self.hard.setText("固件硬限速：待读取（不可修改）")
        self.live.setText("输出轴转速 —  用户 Iq —  限制后 Iq —")
        self.status.setText("已切换电机；当前值尚未确认，可直接发送限速")

    def read(self):
        self.dirty = False
        self._query()

    def _query(self):
        if not self.read_pending:
            self.read_pending = time.monotonic()
        if not self.client.get_speed_limits(self.motor_id_fn()):
            self.read_pending = 0
            self.status.setText("读取发送失败，请检查连接")

    def poll(self):
        if self.read_pending and time.monotonic() - self.read_pending > 3:
            self.read_pending = 0
            self.status.setText("读取超时：请检查连接及固件是否支持限速接口；仍可编辑并发送")
        if self.pending:
            if time.monotonic() - self.pending > 3:
                self.pending = 0
                self.apply_btn.setEnabled(True)
                self.status.setText("应用超时，请读取确认设备实际值")
            return
        if self.isVisible() and self.client.can.manager.is_running:
            self._query()

    def apply(self):
        if self.pending:
            return
        self.read_pending = 0
        self.pending = time.monotonic()
        self.apply_btn.setEnabled(False)
        if self.client.set_current_soft_speed(
            self.motor_id_fn(),
            self.soft.value(),
            hard_overspeed_enabled=self.hard_protect.isChecked(),
        ):
            self.status.setText("已发送，等待设备确认…")
        else:
            self.pending = 0
            self.apply_btn.setEnabled(True)
            self.status.setText("发送失败，未确认应用")

    def receive(self, payload, is_set=False):
        d = parse_speed_limits(payload)
        if d is None:
            self.status.setText("限速应答版本或数据不匹配，请更新固件与 GUI")
            return
        self.read_pending = 0
        if not self.confirmed and not is_set:
            self.status.setText("已读取当前配置（软限速仅保存在 RAM）")
        self.confirmed = True
        if is_set:
            self.pending = 0
            self.dirty = bool(d["error"])
            self.status.setText(
                "限速已应用（RAM）" if not d["error"] else f"设置被拒绝：{api_error_name(d['error'])}"
            )
        hard_state = "已使能" if d["hard_overspeed_enabled"] else "已关闭"
        self.hard.setText(f"固件硬限速：{d['hard']} RPM（输出轴，不可改阈值）· 保护 {hard_state}")
        self.soft.setMaximum(d["hard"])
        self.soft.setEnabled(True)
        self.apply_btn.setEnabled(not bool(self.pending))
        if not self.dirty:
            self.soft.blockSignals(True)
            self.soft.setValue(d["soft"])
            self.soft.blockSignals(False)
            self.hard_protect.blockSignals(True)
            self.hard_protect.setChecked(d["hard_overspeed_enabled"])
            self.hard_protect.blockSignals(False)
        suffix = "软限速介入" if d["active"] else "软限速未介入"
        if d["mode"] == 1:
            self.live.setText(
                f"输出轴 {d['rpm']:.1f} RPM · 用户 Iq {d['requested']:.3f} A · "
                f"限制后 Iq {d['applied']:.3f} A · {suffix}"
            )
        else:
            self.live.setText(
                f"输出轴 {d['rpm']:.1f} RPM · 当前非电流模式（软限速仅作用于电流模式）"
            )
