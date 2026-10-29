import os
os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')
import unittest
from unittest.mock import Mock, patch
import numpy as np
from core.waveform_buffer import WaveformBuffer
from core.motor_client import MotorClient
from canbus.interface import CanThread
from protocol import CMD


class CommandLatencyTests(unittest.TestCase):
    def test_queued_feedback_keeps_target_at_receive_time(self):
        b = WaveformBuffer(20)
        b.set_target(iq=250, changed_at=12.)
        # These old frames reach the GUI AFTER the user has sent the target.
        for i, receipt in enumerate([10., 11., 12., 13.]):
            b.append_sample({'iq_ma': 0 if i < 2 else 200}, i*.001, receipt)
        np.testing.assert_array_equal(b.snapshot()['target_iq'], [0, 0, 250, 250])

    def test_multiple_commands_preserve_order_and_clear(self):
        b = WaveformBuffer(20)
        b.set_target(iq=100, changed_at=10.)
        b.set_target(iq=-100, changed_at=11.)
        for i, receipt in enumerate([9., 10., 10.5, 11.]):
            b.append_sample({'iq_ma': 0}, i*.001, receipt)
        np.testing.assert_array_equal(b.snapshot()['target_iq'], [0, 100, 100, -100])
        self.assertFalse(b._target_events)
        b.clear()
        b.append_sample({'iq_ma': 0}, 2., 12.)
        self.assertEqual(b.snapshot()['target_iq'][0], -100)

    def test_stale_telemetry_is_counted_and_recent_frames_retained(self):
        thread = CanThread()
        thread._telemetry.extend([(1, b'old', 99., 8.), (1, b'new', 100., 9.9)])
        with patch('canbus.interface.time.monotonic', return_value=10.):
            self.assertEqual(thread.telemetry_age_ms, 2000)
            result = thread.take_telemetry(max_age=.25, with_received_time=True)
        self.assertEqual(result, [(1, b'new', 100., 9.9)])
        self.assertEqual(thread.telemetry_stale, 1)
        self.assertEqual(thread.telemetry_dropped, 1)

    def test_command_transmits_before_logging_and_reports_failure(self):
        events = []
        transport = Mock()
        transport.manager.profile.fd = True
        transport.send_message.side_effect = lambda *a, **kw: events.append('send') or False
        client = MotorClient(transport, lambda msg: events.append(msg))
        self.assertFalse(client.broadcast_enable(True))
        self.assertEqual(events[0], 'send')
        self.assertIn('失败', events[-1])
        self.assertIn('耗时=', events[-1])

    def test_send_does_not_wait_for_receive_lock(self):
        thread = CanThread()
        thread.manager = Mock()
        thread.manager.send.return_value = True
        with thread._recv_lock:
            self.assertTrue(thread.send_message(1, b'command'))
        self.assertGreaterEqual(thread.last_send_ms, 0)

    def test_gui_drain_yields_after_budget(self):
        from PyQt5.QtWidgets import QApplication
        from ui.main_window import MainWindow
        app = QApplication.instance() or QApplication([])
        window = MainWindow()
        try:
            window.can_thread.take_telemetry = Mock(return_value=[(1, b'', 1., 1.)]*64)
            window._handle_can_message = Mock()
            with patch('ui.main_window.time.perf_counter', side_effect=[0., .005]):
                window._drain_telemetry()
            window.can_thread.take_telemetry.assert_called_once_with(64, max_age=.250, with_received_time=True)
            self.assertEqual(window._handle_can_message.call_count, 64)
        finally:
            window.close()


if __name__ == '__main__':
    unittest.main()
