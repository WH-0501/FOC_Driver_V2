"""Read-only step analysis of one motor's immutable raw snapshot."""
import csv
from dataclasses import asdict
from PyQt5.QtWidgets import (QDialog, QVBoxLayout, QHBoxLayout, QFormLayout,
    QLabel, QDoubleSpinBox, QComboBox, QPushButton, QTableWidget,
    QTableWidgetItem, QFileDialog, QMessageBox, QAbstractItemView)
from core.step_response import StepSettings, analyze_steps, summarize


FIELDS = [('source', '起点来源'), ('start_s', '阶跃时刻(s)'), ('from_ma', '原指令(mA)'), ('to_ma', '目标(mA)'),
          ('baseline_ma', '实际初值(mA)'), ('rise_ms', '上升时间(ms)'),
          ('overshoot_pct', '原始峰值超调(%)'), ('settling_ms', '调节时间(ms)'),
          ('oscillations', '交替波动次数'), ('oscillation_ma', '振荡幅值(mA)'),
          ('oscillation_pct', '振荡幅值(%)'), ('band_ma', '误差带±mA'),
          ('tail_mean', '末段均值(mA)'), ('tail_error', '末段偏差(mA)'), ('tail_pp', '末段峰峰值(mA)'),
          ('sample_ms', '采样间隔(ms)'), ('window_ms', '观察窗(ms)'), ('status', '数据说明')]


class StepAnalysisDialog(QDialog):
    def __init__(self, data, motor_id, parent=None, cursor_start=None, position_unit="rad"):
        super().__init__(parent)
        self.data, self.motor_id, self.rows = data, motor_id, []
        self.position_unit = position_unit
        self.fields = list(FIELDS)
        self.setWindowTitle(f'通用阶跃分析 · 电机 {motor_id}')
        self.resize(1150, 720)
        layout = QVBoxLayout(self)
        note = QLabel('分析原始缓存快照，不发送指令；请选择与记录工作模式匹配的通道。位置直接使用记录值，不自动处理角度回绕。\n'
            '当前协议未提供设备侧指令生效时间/高速采样时间戳；目标曲线为上位机记录，所有结果均为遥测估算，不能直接用于控制环验收。\n'
            '上升时间按实际初值→目标的10%～90%计算；超调按该阶跃幅值归一化。调节时间仅证明当前观察窗内保持。\n'
            '超调取观察窗内原始峰值，可能包含尖峰；交替波动次数含扰动，不能等同于欠阻尼振铃次数。末段统计取窗口最后20%（至少5点）。')
        note.setWordWrap(True)
        layout.addWidget(note)
        controls = QHBoxLayout()
        form = QFormLayout()
        self.loop = QComboBox()
        self.loop.addItems(['电流 Iq', '速度', '位置'])
        form.addRow('分析通道', self.loop)
        self.trigger = QComboBox()
        self.trigger.addItems(['自动：目标指令变化', '手动：响应起点（使能/恒定目标）'])
        form.addRow('起点识别方式', self.trigger)
        self.start_time = QDoubleSpinBox()
        self.end_time = QDoubleSpinBox()
        for spin in (self.start_time, self.end_time):
            spin.setDecimals(6)
            spin.setRange(float(data['times'][0]), float(data['times'][-1]))
            spin.setSuffix(' s')
        self.start_time.setValue(cursor_start if cursor_start is not None else float(data['times'][0]))
        self.end_time.setValue(float(data['times'][-1]))
        form.addRow('响应起点（可从第一游标带入）', self.start_time)
        form.addRow('观察终点', self.end_time)
        self.start_time.setEnabled(False)
        self.end_time.setEnabled(False)
        self.inputs = {}
        for key, label, value, minimum, suffix in (
            ('min_step_ma', '最小阶跃', 10, .001, ' mA'),
            ('band_percent', '调节误差带', 2, 0, ' %'),
            ('absolute_band_ma', '绝对误差带下限', 5, 0, ' mA'),
            ('hold_ms', '最短保持时间', 5, .001, ' ms'),
            ('noise_ma', '波动噪声门限', 5, 0, ' mA'),
            ('window_ms', '每次阶跃观察窗（0=全段）', 100, 0, ' ms'),
            ('crossing_hold_ms', '10%/90%交越保持', 2, 0, ' ms')):
            spin = QDoubleSpinBox()
            spin.setDecimals(6)
            spin.setRange(minimum, 1000000)
            spin.setValue(value)
            spin.setSuffix(suffix)
            self.inputs[key] = spin
            form.addRow(label, spin)
        self.basis = QComboBox()
        self.basis.addItems(['目标绝对值（零目标仅用绝对误差带）', '阶跃幅值'])
        form.addRow('误差带/振荡百分比基准', self.basis)
        controls.addLayout(form)
        limits = QFormLayout()
        self.limits = {}
        for key, label, value in [('rise_ms', '上升时间参考上限(ms)', 1.5),
                                  ('overshoot_pct', '超调参考上限(%)', 5),
                                  ('settling_ms', '调节时间参考上限(ms)', 5),
                                  ('oscillations', '振荡次数参考上限', 3),
                                  ('oscillation_pct', '振荡幅值参考上限(%)', 5)]:
            spin = QDoubleSpinBox()
            spin.setRange(0, 1000000)
            spin.setDecimals(3)
            spin.setValue(value)
            self.limits[key] = spin
            limits.addRow(label, spin)
        hint = QLabel('参考上限0=不检查；速度/位置默认不套用电流限值。观察窗是分析设置，并非验收阈值。\n采样不足可能漏掉峰值和振荡。\n振荡有效门限取设置值与阶跃前峰峰噪声一半的较大者。')
        hint.setWordWrap(True)
        limits.addRow(hint)
        controls.addLayout(limits)
        layout.addLayout(controls)
        buttons = QHBoxLayout()
        calculate = QPushButton('分析 / 重新计算')
        self.export = QPushButton('导出分析 CSV')
        buttons.addWidget(calculate)
        buttons.addWidget(self.export)
        layout.addLayout(buttons)
        self.table = QTableWidget(0, len(FIELDS)+1)
        self.table.setHorizontalHeaderLabels([label for _, label in self.fields]+['超限参考项'])
        self.table.setEditTriggers(QAbstractItemView.NoEditTriggers)
        layout.addWidget(self.table, 1)
        self.summary = QLabel()
        self.summary.setWordWrap(True)
        layout.addWidget(self.summary)
        calculate.clicked.connect(self.calculate)
        self.export.clicked.connect(self.export_csv)
        for spin in list(self.inputs.values()) + list(self.limits.values()):
            spin.valueChanged.connect(self.invalidate)
        self.basis.currentIndexChanged.connect(self.invalidate)
        self.trigger.currentIndexChanged.connect(self.trigger_changed)
        self.start_time.valueChanged.connect(self.invalidate)
        self.end_time.valueChanged.connect(self.invalidate)
        self.loop.currentIndexChanged.connect(self.loop_changed)
        self.loop_changed(0)

    def loop_changed(self, index):
        self.target_key, self.actual_key, self.unit = [
            ('target_iq', 'iq', 'mA'), ('target_vel', 'actual_vel', 'RPM'),
            ('target_pos', 'actual_pos', self.position_unit)][index]
        self.fields = [(k, label.replace('mA', self.unit)) for k, label in FIELDS]
        self.table.setHorizontalHeaderLabels([label for _, label in self.fields]+['超限参考项'])
        # Per-channel starting settings, not motor-specific acceptance criteria.
        presets = [(10, 5, 5, 5, 100), (1, .5, .5, 50, 1000),
                   (.001, .001, .001, 50, 2000) if self.unit == 'rad' else (.1, .05, .05, 50, 2000)]
        for key, value in zip(('min_step_ma', 'absolute_band_ma', 'noise_ma', 'hold_ms', 'window_ms'), presets[index]):
            self.inputs[key].setValue(value)
        for key in ('min_step_ma', 'absolute_band_ma', 'noise_ma'):
            self.inputs[key].setSuffix(' '+self.unit)
        for key, spin in self.limits.items():
            spin.setValue(dict(rise_ms=1.5, overshoot_pct=5, settling_ms=5, oscillations=3, oscillation_pct=5)[key] if index == 0 else 0)
        self.basis.setCurrentIndex(0 if index == 0 else 1)
        self.calculate()

    def trigger_changed(self, index):
        self.start_time.setEnabled(index == 1)
        self.end_time.setEnabled(index == 1)
        self.invalidate()

    def invalidate(self, *_):
        self.export.setEnabled(False)
        self.table.setRowCount(0)
        self.summary.setText('设置已修改，请重新计算。')

    def calculate(self):
        self.settings = StepSettings(**{k: v.value() for k, v in self.inputs.items()},
                                     target_relative=self.basis.currentIndex() == 0)
        self.thresholds = {k: v.value() for k, v in self.limits.items()}
        self.manual_start = self.start_time.value() if self.trigger.currentIndex() == 1 else None
        self.manual_end = min(self.end_time.value(), float(self.data['times'][-1])) if self.trigger.currentIndex() == 1 else None
        try:
            self.rows = analyze_steps(self.data['times'], self.data.get(self.target_key, []), self.data.get(self.actual_key, []), self.settings,
                                      self.manual_start, self.manual_end)
        except ValueError as exc:
            self.invalidate()
            self.summary.setText(str(exc))
            return
        self.table.setRowCount(len(self.rows))
        labels = dict(self.fields)
        for i, row in enumerate(self.rows):
            exceeded = [labels[k] for k, limit in self.thresholds.items()
                        if limit > 0 and row[k] is not None and row[k] > limit]
            row['exceeded'] = '、'.join(exceeded) or '无已观测超限（非合格判定）'
            for col, key in enumerate([k for k, _ in self.fields]+['exceeded']):
                value = row[key]
                text = '—' if value is None else (f'{value:.6g}' if isinstance(value, float) else str(value))
                self.table.setItem(i, col, QTableWidgetItem(text))
        self.table.resizeColumnsToContents()
        stats = summarize(self.rows)
        parts = [f'{self.loop.currentText()}（{self.unit}）共识别 {len(self.rows)} 次阶跃；按当前观察窗统计，包含初始未稳态及噪声影响，缺失值不计入。']
        for key, (count, mean, worst) in stats.items():
            if count:
                parts.append(f'{labels[key]}：n={count}，均值 {mean:.4g}，最大 {worst:.4g}')
        if not self.rows:
            parts.append('未检测到目标指令阶跃。若实际值因使能等原因发生跃迁，请选择“手动：响应起点”，设定跃迁前时刻和观察终点后重新计算。保持真实目标，不用实际平台值替代目标。')
        self.summary.setText('\n'.join(parts))
        self.export.setEnabled(bool(self.rows))

    def export_csv(self):
        path, _ = QFileDialog.getSaveFileName(self, '导出阶跃分析', f'motor_{self.motor_id}_steps.csv', 'CSV (*.csv)')
        if not path:
            return
        try:
            with open(path, 'w', newline='', encoding='utf-8-sig') as stream:
                writer = csv.writer(stream)
                writer.writerow(['motor_id', self.motor_id, '通道', self.loop.currentText(), '单位', self.unit, '遥测估算，非验收结论'])
                writer.writerow(['配置', str(asdict(self.settings)), '参考上限', str(self.thresholds)])
                writer.writerow(['手动起点(s)', self.manual_start, '观察终点(s)', self.manual_end])
                keys = [k for k, _ in self.fields]+['exceeded']
                writer.writerow([label for _, label in self.fields]+['超限参考项'])
                writer.writerows([[row[k] for k in keys] for row in self.rows])
                writer.writerow([])
                writer.writerow(['统计项', '有效数量', '均值', '最大值'])
                writer.writerows([[key, *value] for key, value in summarize(self.rows).items()])
        except OSError as exc:
            QMessageBox.warning(self, '导出失败', str(exc))
