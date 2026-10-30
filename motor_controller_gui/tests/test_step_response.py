import os
os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')
import unittest
import numpy as np
from core.step_response import StepSettings, analyze_steps, summarize


class StepResponseTests(unittest.TestCase):
    def signal(self, initial=0, target=1000, tau=.001):
        t = np.arange(-.005, .030, .00005)
        r = np.where(t < 0, initial, target)
        y = initial+(target-initial)*(1-np.exp(-np.maximum(t, 0)/tau))
        return t, r, y

    def test_first_order_positive_negative_and_nonzero_baseline(self):
        for initial, target in [(0, 1000), (1000, -1000), (-400, -1200)]:
            t, r, y = self.signal(initial, target)
            row = analyze_steps(t, r, y, StepSettings(absolute_band_ma=0, target_relative=False))[0]
            self.assertAlmostEqual(row['rise_ms'], np.log(9), delta=.01)
            self.assertEqual(row['overshoot_pct'], 0)
            self.assertAlmostEqual(row['settling_ms'], -np.log(.02), delta=.06)
            self.assertEqual(row['oscillations'], 0)

    def test_underdamped_analytic_overshoot(self):
        t, r, _ = self.signal()
        zeta, wn = .5, 3000
        u = np.maximum(t, 0)
        wd = wn*np.sqrt(1-zeta*zeta)
        y = 1000*(1-np.exp(-zeta*wn*u)*(np.cos(wd*u)+zeta/np.sqrt(1-zeta*zeta)*np.sin(wd*u)))
        row = analyze_steps(t, r, y)[0]
        expected = np.exp(-np.pi*zeta/np.sqrt(1-zeta*zeta))*100
        self.assertAlmostEqual(row['overshoot_pct'], expected, delta=.1)
        self.assertGreater(row['oscillations'], 0)
        self.assertGreater(row['oscillation_ma'], 0)

    def test_unsettled_and_short_observation(self):
        t, r, y = self.signal()
        y[t >= 0] = 950
        row = analyze_steps(t, r, y)[0]
        self.assertIsNone(row['settling_ms'])
        t, r, y = self.signal()
        mask = t < .004
        self.assertIsNone(analyze_steps(t[mask], r[mask], y[mask])[0]['settling_ms'])

    def test_zero_target_and_missing_samples(self):
        t, r, y = self.signal(1000, 0)
        row = analyze_steps(t, r, y)[0]
        self.assertIsNone(row['oscillation_pct'])
        self.assertEqual(row['band_ma'], 5)
        y[150] = np.nan
        row = analyze_steps(t, r, y)[0]
        self.assertIsNone(row['rise_ms'])
        self.assertIn('无效', row['status'])
        self.assertEqual(summarize([row])['rise_ms'], (0, None, None))

    def test_next_command_truncates_even_below_minimum(self):
        t, r, y = self.signal()
        r[t >= .002] = 1001
        rows = analyze_steps(t, r, y)
        self.assertEqual(len(rows), 1)
        self.assertIsNone(rows[0]['settling_ms'])
        self.assertLess(rows[0]['window_ms'], 2)

    def test_low_resolution_and_time_validation(self):
        t = np.arange(30)*.001
        r = np.r_[np.zeros(5), np.ones(25)*1000]
        row = analyze_steps(t, r, r)[0]
        self.assertIsNone(row['rise_ms'])
        self.assertIn('未完整捕获', row['status'])
        with self.assertRaises(ValueError):
            analyze_steps([0, 0], [0, 1000], [0, 1000])
        self.assertEqual(analyze_steps(t, np.zeros(30), np.zeros(30)), [])

    def test_dialog_statistics_and_settings_invalidation(self):
        from PyQt5.QtWidgets import QApplication
        from ui.step_analysis_dialog import StepAnalysisDialog
        app = QApplication.instance() or QApplication([])
        t, r, y = self.signal()
        dialog = StepAnalysisDialog(dict(times=t, target_iq=r, iq=y), 2)
        try:
            self.assertEqual(dialog.table.rowCount(), 1)
            self.assertTrue(dialog.export.isEnabled())
            self.assertIn('均值', dialog.summary.text())
            dialog.inputs['hold_ms'].setValue(100)
            self.assertFalse(dialog.export.isEnabled())
            dialog.calculate()
            self.assertIsNone(dialog.rows[0]['settling_ms'])
        finally:
            dialog.close()

    def test_constant_command_enable_response(self):
        t, _, y = self.signal(target=200)
        r = np.full(len(t), 250.)
        self.assertEqual(analyze_steps(t, r, y), [])
        row = analyze_steps(t, r, y, manual_start_s=-.0001)[0]
        self.assertEqual(row['to_ma'], 250)
        self.assertEqual(row['from_ma'], 250)
        self.assertIsNone(row['settling_ms'])
        self.assertIsNone(row['rise_ms'])  # 90% of real target never reached
        self.assertIn('手动', row['source'])

    def test_manual_negative_response_and_bounds(self):
        t, _, y = self.signal(0, -1000)
        r = np.full(len(t), -1000.)
        row = analyze_steps(t, r, y, manual_start_s=-.0001)[0]
        self.assertAlmostEqual(row['rise_ms'], np.log(9), delta=.01)
        self.assertIsNotNone(row['settling_ms'])
        with self.assertRaises(ValueError):
            analyze_steps(t, r, y, manual_start_s=t[0]-1)
        with self.assertRaises(ValueError):
            analyze_steps(t, r, y, manual_start_s=0, manual_end_s=-.001)
        r[t >= .002] = -500
        row = analyze_steps(t, r, y, manual_start_s=-.0001)[0]
        self.assertLess(row['window_ms'], 2.2)
        self.assertIsNone(row['settling_ms'])

    def test_manual_dialog_cursor_and_export_metadata(self):
        from PyQt5.QtWidgets import QApplication
        from ui.step_analysis_dialog import StepAnalysisDialog
        app = QApplication.instance() or QApplication([])
        t, _, y = self.signal()
        dialog = StepAnalysisDialog(dict(times=t, target_iq=np.full(len(t),1000.), iq=y),
                                    1, cursor_start=-.0001)
        try:
            self.assertEqual(dialog.table.rowCount(), 0)
            dialog.trigger.setCurrentIndex(1)
            self.assertTrue(dialog.start_time.isEnabled())
            dialog.calculate()
            self.assertEqual(dialog.table.rowCount(), 1)
            self.assertEqual(dialog.manual_start, -.0001)
            self.assertTrue(dialog.export.isEnabled())
        finally:
            dialog.close()

    def test_window_excludes_late_disturbance_without_hiding_raw_peak(self):
        t = np.arange(-.01, .5, .001)
        r = np.where(t < 0, 0., 1000.)
        y = 1000*(1-np.exp(-np.maximum(t, 0)/.005))
        y[np.argmin(abs(t-.3))] = 2000
        full = analyze_steps(t, r, y)[0]
        limited = analyze_steps(t, r, y, StepSettings(window_ms=100))[0]
        self.assertAlmostEqual(full['overshoot_pct'], 100)
        self.assertEqual(limited['overshoot_pct'], 0)
        self.assertLessEqual(limited['window_ms'], 100.001)
        self.assertAlmostEqual(limited['tail_error'], 0, delta=.001)

    def test_isolated_spike_does_not_confirm_ninety_percent(self):
        t = np.arange(-.01, .1, .001)
        r = np.where(t < 0, 0., 250.)
        y = 200*(1-np.exp(-np.maximum(t, 0)/.005))
        y[np.argmin(abs(t-.03))] = 500
        row = analyze_steps(t, r, y, StepSettings(crossing_hold_ms=2))[0]
        self.assertIsNotNone(row['rise_ms'])
        self.assertIsNone(row['qualified_rise_ms'])
        self.assertAlmostEqual(row['rise_ms'], (row['t90_s']-row['t10_s'])*1000)
        self.assertIsNone(row['settling_ms'])
        self.assertGreater(row['overshoot_pct'], 90)  # raw spike stays visible
        self.assertAlmostEqual(row['tail_error'], -50, delta=.01)

    def test_long_capture_seconds_and_held_crossing_are_distinct(self):
        t = 1005.0 + np.arange(-20, 101)*.001
        r = np.where(t < 1005.0, 0., 250.)
        y = np.where(t < 1005.0, 0., 200.)
        y[np.argmin(abs(t-1005.010))] = 230
        y[t >= 1005.050] = 250
        row = analyze_steps(t, r, y, StepSettings(crossing_hold_ms=2))[0]
        self.assertLess(row['rise_ms'], 12)
        self.assertGreater(row['qualified_rise_ms'], 45)
        self.assertAlmostEqual(row['level10'], 25)
        self.assertAlmostEqual(row['level90'], 225)
        self.assertAlmostEqual(row['rise_ms'], (row['t90_s']-row['t10_s'])*1000)

    def test_position_velocity_channel_units_and_independent_limits(self):
        from PyQt5.QtWidgets import QApplication
        from ui.step_analysis_dialog import StepAnalysisDialog
        app = QApplication.instance() or QApplication([])
        t, r, y = self.signal()
        data = dict(times=t, target_iq=r, iq=y, target_vel=r/10, actual_vel=y/10,
                    target_pos=r/1000, actual_pos=y/1000)
        dialog = StepAnalysisDialog(data, 1)
        try:
            for index, unit, target in [(1, 'RPM', 100), (2, 'rad', 1)]:
                dialog.loop.setCurrentIndex(index)
                self.assertEqual(dialog.unit, unit)
                self.assertEqual(dialog.rows[0]['to_ma'], target)
                self.assertAlmostEqual(dialog.rows[0]['rise_ms'], np.log(9), delta=.01)
                self.assertEqual(dialog.limits['rise_ms'].value(), 0)
                self.assertIn(unit, dialog.table.horizontalHeaderItem(3).text())
        finally:
            dialog.close()


if __name__ == '__main__':
    unittest.main()
