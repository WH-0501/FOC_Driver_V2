import os
os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')
import unittest
from unittest.mock import Mock
from PyQt5.QtWidgets import QApplication
from ui.param_widget import MotorParamWidget
from protocol import ProtocolVersion, pack_cur_pid_params, parse_cur_pid_response, PID_VALUE_MAX, cur_pid_params_in_range

class CurrentKiTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.app = QApplication.instance() or QApplication([])

    def test_readback_and_resend_preserve_scaled_ki(self):
        client = Mock()
        client.protocol_version = ProtocolVersion.V2_0_0
        w = MotorParamWidget(1, client)
        try:
            wire = pack_cur_pid_params(4.021, 0.1508, 0)
            decoded = parse_cur_pid_response(wire)
            w.apply_pid('cur', decoded)
            self.assertAlmostEqual(decoded['I'], 618/4096)
            self.assertEqual(wire[2:4], bytes([2, 106]))
            self.assertLess(w.cur_i.maximum(), 16.001)
            self.assertIsNotNone(cur_pid_params_in_range(4.021, 1507.96, 0))
            self.assertAlmostEqual(w.cur_i.value(), decoded['I'], places=4)
            w._set_cur_pid()
            args = client.set_cur_pid.call_args.args
            self.assertEqual(pack_cur_pid_params(*args[1:]), wire)
            self.assertLess(w.pos_i.maximum(), 16.001)
            self.assertLess(w.vel_i.maximum(), 16.001)
        finally:
            w.close()
