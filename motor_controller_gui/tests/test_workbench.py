import os
os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')
import unittest
from unittest.mock import Mock, patch
import numpy as np
import can
from PyQt5.QtWidgets import QApplication
from PyQt5.QtCore import Qt, QPoint, QPointF
from PyQt5.QtGui import QWheelEvent
import pyqtgraph as pg
from core.waveform_buffer import WaveformBuffer
from core.waveform_analysis import peak_envelope, spectrum
from core.motor_client import MotorClient
from canbus.device_manager import CanDeviceManager
from canbus.device_profiles import DEVICE_PROFILES
from canbus.interface import CanThread
from protocol import CMD, pack_frame, pad_canfd_payload
from ui.waveform_widget import WaveformWidget


class BufferTests(unittest.TestCase):
    def test_wrap_window_and_independent_snapshot(self):
        b = WaveformBuffer(max_points=7)
        for i in range(22):
            b.append_sample({'velocity': i}, i * .001)
        snap = b.snapshot()
        np.testing.assert_array_equal(snap['actual_vel'], np.arange(15, 22))
        window = b.snapshot_window(.0175, .0205, ['actual_vel'])
        np.testing.assert_array_equal(window['actual_vel'], [18, 19, 20])
        b.append_sample({'velocity': 99}, .022)
        self.assertEqual(snap['actual_vel'][-1], 21)
        b.clear()
        self.assertEqual(len(b.snapshot()['times']), 0)

    def test_missing_currents_and_timestamp_gaps(self):
        b = WaveformBuffer(10)
        for stamp in (10, 10, 9, 9.5):
            b.append_sample({'has_phase_current': False, 'id_ma': 0}, stamp)
        np.testing.assert_allclose(b.times, [0, .001, .002, .502])
        self.assertTrue(np.isnan(b.snapshot()['id']).all())

    def test_resize_retains_newest_and_continuous_time(self):
        b = WaveformBuffer(7)
        for i in range(20):
            b.append_sample({'velocity': i}, i * .001)
        b.resize(4)
        np.testing.assert_array_equal(b.snapshot()['actual_vel'], [16, 17, 18, 19])
        b.resize(30)
        b.append_sample({'velocity': 20}, .020)
        self.assertAlmostEqual(b.latest_time, .020)
        np.testing.assert_array_equal(b.snapshot()['actual_vel'], [16, 17, 18, 19, 20])

    def test_envelope_preserves_extrema_and_budget(self):
        y = np.zeros(120000)
        y[397] = 100
        y[398] = -80
        x, out = peak_envelope(np.arange(len(y)), y, 1500)
        self.assertLessEqual(len(out), 1500)
        self.assertEqual(max(out), 100)
        self.assertEqual(min(out), -80)
        self.assertTrue(np.all(np.diff(x) > 0))

    def test_fft_amplitude_and_frequency(self):
        t = np.arange(4096) / 1024
        f, a = spectrum(3 * np.sin(2*np.pi*64*t), 4096, 1/1024)
        self.assertEqual(f[a.argmax()], 64)
        self.assertAlmostEqual(a.max(), 3, places=4)
        self.assertEqual(len(spectrum([np.nan]*256, 256)[0]), 0)


class TransportTests(unittest.TestCase):
    def test_fd_wire_unchanged(self):
        transport = Mock()
        transport.manager.profile.fd = True
        client = MotorClient(transport)
        for count, payload in enumerate((b'', b'12', b'123456789', bytes(range(40)))):
            client._send(3, CMD.ENABLE, payload)
            transport.send_message.assert_called_with(
                3, pad_canfd_payload(pack_frame(CMD.ENABLE, payload, count)), is_fd=True)

    def test_classic_short_frames_and_no_truncation(self):
        transport = Mock()
        transport.manager.profile.fd = False
        transport.send_message.return_value = True
        client = MotorClient(transport)
        self.assertTrue(client._send(2, CMD.ENABLE, b'\x01'))
        transport.send_message.assert_called_once_with(2, b'\x00'+bytes([CMD.ENABLE])+b'\x01', is_fd=False)
        self.assertFalse(client._send(2, CMD.ENABLE, bytes(7)))
        self.assertEqual(transport.send_message.call_count, 1)

    def test_virtual_bus_roundtrip(self):
        manager = CanDeviceManager()
        profile = next(p for p in DEVICE_PROFILES if p.backend == 'virtual')
        manager.open_device(profile.name, serial_port='unit-test')
        other = can.Bus(interface='virtual', channel='unit-test')
        try:
            manager.start_channel()
            self.assertTrue(manager.send(2, bytes(range(20))))
            message = other.recv(.1)
            self.assertTrue(message.is_fd)
            self.assertEqual(bytes(message.data), bytes(range(20)))
            other.send(can.Message(arbitration_id=3, data=b'123', is_extended_id=False))
            self.assertEqual(manager.recv_batch(.1)[0].arbitration_id, 3)
        finally:
            manager.close_device()
            other.shutdown()

    def test_receiver_bounds_telemetry_and_prioritizes_response(self):
        thread = CanThread()
        thread.running = True
        thread.manager = Mock()
        thread.manager.is_running = True
        telemetry = can.Message(arbitration_id=1, data=bytes([0, CMD.PERIOD_REPORT])+bytes(18), is_fd=True)
        response = can.Message(arbitration_id=1, data=bytes([0, CMD.IAP_STATUS, 2, 128]), is_fd=True)
        batches = iter([[telemetry]*64 for _ in range(520)] + [[response]])
        def receive(**kwargs):
            batch = next(batches, None)
            if batch is None:
                thread.running = False
                return []
            return batch
        thread.manager.recv_batch.side_effect = receive
        replies = []
        thread.msg_received.connect(lambda *args: replies.append(args))
        thread.run()
        self.assertEqual(thread.telemetry_pending, 32768)
        self.assertEqual(thread.telemetry_dropped, 512)
        self.assertEqual(len(replies), 1)
        self.assertEqual(len(thread.take_telemetry(128)), 128)


class WidgetTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.app = QApplication.instance() or QApplication([])

    def _wheel(self, widget, delta=120):
        plot = widget._plot
        pos = plot.mapFromScene(widget._view_box.sceneBoundingRect().center())
        event = QWheelEvent(QPointF(pos), QPointF(plot.viewport().mapToGlobal(pos)),
                            QPoint(), QPoint(0, delta), Qt.NoButton, Qt.NoModifier,
                            Qt.NoScrollPhase, False)
        QApplication.sendEvent(plot.viewport(), event)

    def test_wheel_zoom_keeps_live_updates_and_zoom_span(self):
        w = WaveformWidget()
        self.addCleanup(w.close)
        w._timer.stop()
        w.resize(1200, 700)
        w.set_motor(1)
        for i in range(7000):
            w.append_sample(1, {'velocity': i}, i*.001)
        w.show()
        w._refresh_plot()
        self.app.processEvents()
        before = w.window.value()
        self._wheel(w)
        self.assertFalse(w._paused)
        self.assertIsNone(w._frozen)
        self.assertLess(w.window.value(), before)
        zoom_span = w.window.value()
        y_range = w._view_box.viewRange()[1][:]
        w.append_sample(1, {'velocity': 8888}, 7.5)
        w._refresh_plot()
        x, y = w._curves['actual_vel'].getData()
        self.assertAlmostEqual(x[-1], 7.5)
        self.assertEqual(y[-1], 8888)
        left, right = w._view_box.viewRange()[0]
        self.assertAlmostEqual(right, 7.5)
        self.assertAlmostEqual(right-left, zoom_span)
        np.testing.assert_allclose(w._view_box.viewRange()[1], y_range)
        self.assertTrue(w.pause_btn.isEnabled())

    def test_drag_freezes_history_and_wheel_does_not_resume_it(self):
        w = WaveformWidget()
        self.addCleanup(w.close)
        w._timer.stop()
        w.resize(1200, 700)
        w.set_motor(1)
        for i in range(1000):
            w.append_sample(1, {'velocity': i}, i*.001)
        w.show()
        w._refresh_plot()
        self.app.processEvents()
        event = Mock()
        event.isStart.return_value = True
        event.button.return_value = Qt.LeftButton
        with patch.object(pg.ViewBox, 'mouseDragEvent'):
            w._view_box.mouseDragEvent(event)
        self.assertTrue(w._paused)
        frozen = w._frozen['times'].copy()
        self._wheel(w)
        w.append_sample(1, {'velocity': 2000}, 2)
        w._refresh_plot()
        self.assertTrue(w._paused)
        np.testing.assert_array_equal(w._frozen['times'], frozen)
        w._on_resume()
        w._refresh_plot()
        self.assertEqual(w._curves['actual_vel'].getData()[1][-1], 2000)

    def test_freeze_and_all_analysis_modes(self):
        w = WaveformWidget()
        w._timer.stop()
        w.set_motor(1)
        for i in range(5000):
            w.append_sample(1, {'velocity': i, 'iq_ma': i % 20}, i*.001)
        w._on_pause()
        frozen = w._frozen['times'].copy()
        w.append_sample(1, {'velocity': 999}, 5)
        np.testing.assert_array_equal(w._frozen['times'], frozen)
        self.assertEqual(len(w._buffers[1]), 5001)
        for mode in range(5):
            w.mode.setCurrentIndex(mode)
            w._refresh_plot()
        w._on_resume()
        w.clear_current()
        w._refresh_plot()
        self.assertEqual(len(w._buffers[1]), 0)
        w.close()


if __name__ == '__main__':
    unittest.main()
