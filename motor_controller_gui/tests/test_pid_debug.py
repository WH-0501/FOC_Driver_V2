import os
os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')
import struct
import unittest
from unittest.mock import Mock, patch
from PyQt5.QtWidgets import QApplication
from protocol.constants import CMD
from protocol.pid_debug import parse_debug
from ui.pid_debug_dialog import PidDebugDialog
from core.motor_client import MotorClient


def sample(flags=1):
    return struct.pack('>BBI14f', 1, flags, 123456, .25, .2, .2, .01, .00005,
                       .1, .3, 24, 5, 4, .1508, 12, 10, 0)


class DebugTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.app = QApplication.instance() or QApplication([])

    def test_single_frame_and_units(self):
        data = sample()
        self.assertEqual(len(data)+2, 64)
        row = parse_debug(CMD.DEBUG_CURRENT, data)
        self.assertAlmostEqual(row['error'], .05, places=6)
        self.assertAlmostEqual(row['p_term'], .2, places=6)
        self.assertAlmostEqual(row['ki_effective'], 1508, places=3)
        self.assertAlmostEqual(parse_debug(CMD.DEBUG_SPEED, data)['ki_effective'], .1508, places=6)
        self.assertFalse(parse_debug(CMD.DEBUG_POSITION, sample(0))['active'])
        self.assertIsNone(parse_debug(CMD.DEBUG_CURRENT, data[:-1]))
        self.assertIsNone(parse_debug(CMD.DEBUG_CURRENT, bytes([2])+data[1:]))

    def test_mode_confirmation_timeout_and_close(self):
        client = Mock()
        client.configure_pid_debug.return_value = True
        d = PidDebugDialog(client, 7)
        try:
            d.mode.setCurrentIndex(1)
            d.checks[2].setChecked(True)
            d.apply_mode()
            client.configure_pid_debug.assert_called_with(7, 5, 100)
            self.assertEqual(d.confirmed, 0)
            d.receive(CMD.DEBUG_CONFIG, bytes([1, 0, 5, 0, 100]), 0)
            self.assertEqual(d.confirmed, 5)
            d.renew()
            client.configure_pid_debug.assert_called_with(7, 5, 100, log=False)
            d.receive(CMD.DEBUG_CURRENT, sample(), 1)
            d.receive(CMD.DEBUG_POSITION, sample(0), 1.1)
            d.render()
            self.assertEqual(len(d.buffers[0]), 1)
            self.assertIn('未执行', d.loop_infos[2].text())
            d.last_ack -= 4
            d.renew()
            self.assertEqual(d.requested[0], 0)
            self.assertEqual(d.confirmed, 0)
        finally:
            d.close()
        client.configure_pid_debug.assert_called_with(7, 0, 100, log=False)

    def test_overlay_common_clock_and_locked_range(self):
        d = PidDebugDialog(Mock(), 1)
        try:
            d.receive(CMD.DEBUG_CURRENT, sample(), 100)
            d.receive(CMD.DEBUG_SPEED, sample(), 100.125)
            d.curve_checks[1, 'feedback'].setChecked(True)
            d.render()
            self.assertAlmostEqual(d.buffers[1][-1]['receive_s'], .125)
            self.assertEqual(d.curves[1, 'feedback'].getData()[0][0], .125)
            d.lock_view.setChecked(True)
            d.plot.setXRange(.02, .08, padding=0)
            d.plot.setYRange(-.1, .4, padding=0)
            previous = d.plot.viewRange()
            frozen_times = d.curves[0, 'feedback'].getData()[0].copy()
            frozen_value = d.value_labels[0, 'feedback'].text()
            d.receive(CMD.DEBUG_CURRENT, sample(), 120)
            d.render()
            self.assertEqual(previous, d.plot.viewRange())
            self.assertEqual(list(frozen_times), list(d.curves[0, 'feedback'].getData()[0]))
            self.assertEqual(frozen_value, d.value_labels[0, 'feedback'].text())
            self.assertEqual(len(d.buffers[0]), 2)
            d.curve_checks[0, 'iq'].setChecked(True)
            d.render()
            self.assertEqual(list(d.curves[0, 'iq'].getData()[0]), list(frozen_times))
            d.select_curves('none')
            self.assertFalse(any(c.isVisible() for c in d.curves.values()))
            d.reset_view()
            self.assertFalse(d.lock_view.isChecked())
            self.assertIsNone(d._frozen)
            self.assertGreater(d.plot.viewRange()[0][1], 19)
            d.clear()
            self.assertIsNone(d.origin)
        finally:
            d.close()

    def test_select_normal_stops_renewal_and_ignores_old_ack(self):
        client = Mock()
        client.configure_pid_debug.return_value = True
        d = PidDebugDialog(client, 1)
        try:
            d.mode.setCurrentIndex(1)
            d.apply_mode()
            d.receive(CMD.DEBUG_CONFIG, bytes([1, 0, 1, 0, 100]), 0)
            d.mode.setCurrentIndex(0)
            client.configure_pid_debug.assert_called_with(1, 0, 100, log=False)
            d.receive(CMD.DEBUG_CONFIG, bytes([1, 0, 1, 0, 100]), 0)
            self.assertTrue(d._stop_pending)
            client.configure_pid_debug.reset_mock()
            d.renew()
            client.configure_pid_debug.assert_called_once_with(1, 0, 100, log=False)
            d.receive(CMD.DEBUG_CONFIG, bytes([1, 0, 0, 0, 100]), 0)
            self.assertEqual(d.confirmed, 0)
            self.assertFalse(d._stop_pending)
            client.configure_pid_debug.reset_mock()
            d.renew()
            client.configure_pid_debug.assert_not_called()
        finally:
            d.close()

    def test_client_unicast_fd_and_iap_guards(self):
        transport = Mock()
        transport.manager.is_running = True
        transport.manager.profile.fd = True
        transport.send_message.return_value = True
        client = MotorClient(transport)
        self.assertTrue(client.configure_pid_debug(3, 7, 200))
        call = transport.send_message.call_args
        self.assertEqual(call.args[0], 3)
        self.assertEqual(call.args[1][1:6], bytes([0xB8, 1, 7, 0, 200]))
        with self.assertRaises(ValueError):
            client.configure_pid_debug(0, 1)
        client._iap_session = True
        self.assertFalse(client.configure_pid_debug(3, 1))
        client._iap_session = False
        transport.manager.profile.fd = False
        self.assertFalse(client.configure_pid_debug(3, 1))
