import os
os.environ.setdefault('QT_QPA_PLATFORM','offscreen')
import struct,unittest
from unittest.mock import Mock
from PyQt5.QtWidgets import QApplication
from ui.speed_limit_widget import SpeedLimitWidget,parse_speed_limits
from core.motor_client import MotorClient
from protocol import CMD

def packet(soft=54,active=0,error=0):
    return struct.pack('>BBHHBBiii',2,error,60,soft,1,active,-50000,-2000,-1000)

class SpeedLimitsTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):cls.app=QApplication.instance() or QApplication([])
    def test_codec(self):
        d=parse_speed_limits(packet()+bytes(2))
        self.assertEqual(d['rpm'],-50);self.assertEqual(d['hard'],60);self.assertEqual(d['applied'],-1)
        self.assertIsNone(parse_speed_limits(b''));self.assertIsNone(parse_speed_limits(packet(soft=61)))
    def test_gui_edit_readback_and_confirmation(self):
        c=Mock();w=SpeedLimitWidget(lambda:1,c)
        self.assertTrue(w.apply_btn.isEnabled())
        w.receive(packet());self.assertEqual(w.soft.value(),54)
        w.soft.setValue(10);w.receive(packet());self.assertEqual(w.soft.value(),10)
        w.apply();c.set_current_soft_speed.assert_called_once_with(1,10)
        self.assertTrue(w.pending);w.receive(packet(soft=10,active=1),True)
        self.assertFalse(w.pending);self.assertIn('已应用',w.status.text());self.assertIn('软限速介入',w.live.text())
        w.reset();self.assertTrue(w.apply_btn.isEnabled());w.close()
    def test_send_before_read_and_timeout(self):
        c=Mock();w=SpeedLimitWidget(lambda:7,c)
        w.soft.setValue(12);w.apply()
        c.set_current_soft_speed.assert_called_once_with(7,12)
        self.assertNotIn('已应用',w.status.text())
        w.pending-=4;w.poll()
        self.assertIn('超时',w.status.text());self.assertTrue(w.apply_btn.isEnabled())
        w.receive(packet(soft=12),True);self.assertIn('已应用',w.status.text())
        w.reset();self.assertFalse(w.confirmed);self.assertEqual(w.soft.value(),54)
        w.read();w.read_pending-=4;w.poll();self.assertIn('读取超时',w.status.text());w.close()
    def test_reject_old_shaft_units(self):
        old=bytearray(packet());old[0]=1
        self.assertIsNone(parse_speed_limits(old))
        self.assertEqual(int(CMD.GET_SPEED_LIMITS),0xB5)
        self.assertEqual(int(CMD.SET_CURRENT_SOFT_SPEED),0xB6)
    def test_control_page_owns_limit(self):
        from ui.control_widget import MotorControlWidget
        from protocol import ProtocolVersion
        c=Mock();c.protocol_version=list(ProtocolVersion)[0]
        w=MotorControlWidget(1,c)
        self.assertIsInstance(w.speed_limits,SpeedLimitWidget)
        w.motor_id=8;w.speed_limits.soft.setValue(16);w.speed_limits.apply()
        c.set_current_soft_speed.assert_called_once_with(8,16);w.close()
    def test_client_no_hard_write_and_bounds(self):
        c=MotorClient(Mock());c._send=Mock(return_value=True)
        self.assertTrue(c.set_current_soft_speed(1,0));self.assertTrue(c.set_current_soft_speed(1,3000))
        self.assertFalse(c.set_current_soft_speed(1,3001));self.assertFalse(c.set_current_soft_speed(0,100))
        self.assertEqual(c._send.call_args.args[1],CMD.SET_CURRENT_SOFT_SPEED)

if __name__=='__main__':unittest.main()
