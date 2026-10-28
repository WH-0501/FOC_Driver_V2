import os
os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')
import math
import struct
import unittest
from unittest.mock import Mock, patch
from PyQt5.QtWidgets import QApplication
from protocol.alignment_verify import parse_verify_sample, parse_verify_result, VerificationRun
from ui.alignment_verify_dialog import AlignmentVerifyDialog
from ui.cal_widget import MotorCalWidget


def sample(session=7, seq=0, settled=True, phase=1):
    return struct.pack('>BBBBHHIiiiii', 1, phase, int(settled), 0, session, seq,
                       seq*900, 6280000, 1000, 4185, -120000, 130000)


def result(session=7, error=0, count=9):
    return struct.pack('>BBHHHi', 1, error, session, count, 0, 10000)


class VerifyTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.app = QApplication.instance() or QApplication([])

    def test_decode_padded_frames_and_signed_values(self):
        s = parse_verify_sample(sample()+bytes(14))
        self.assertEqual(s['mechanical'], -.12)
        self.assertEqual(s['sequence'], 0)
        self.assertAlmostEqual(s['error'], .004185)
        self.assertEqual(parse_verify_result(result()+bytes(2))['count'], 9)
        for payload in (b'', sample()[:31], b'\x02'+sample()[1:]):
            self.assertIsNone(parse_verify_sample(payload))
        self.assertIsNone(parse_verify_result(result()[:11]))

    def test_integrity_and_old_sessions(self):
        r=VerificationRun(7)
        self.assertFalse(r.accept(parse_verify_sample(sample(session=8))))
        for i in range(9): self.assertTrue(r.accept(parse_verify_sample(sample(seq=i))))
        self.assertFalse(r.accept(parse_verify_sample(sample(seq=8))))
        self.assertFalse(r.finish(parse_verify_result(result(session=8))))
        self.assertTrue(r.finish(parse_verify_result(result())))
        self.assertTrue(r.passed)
        self.assertFalse(r.accept(parse_verify_sample(sample(seq=9))))
        for missing in (0,4,8):
            r=VerificationRun(7)
            for i in range(9):
                if i!=missing:r.accept(parse_verify_sample(sample(seq=i)))
            r.finish(parse_verify_result(result()))
            self.assertFalse(r.passed)

    def test_compressed_time_cannot_pass(self):
        r=VerificationRun(7)
        for i in range(9):
            s=parse_verify_sample(sample(seq=i));s['time_s']=i*.04;r.accept(s)
        r.finish(parse_verify_result(result()))
        self.assertFalse(r.passed)

    def test_sweep_points_cannot_substitute_settled_points(self):
        r=VerificationRun(7)
        for i in range(9):r.accept(parse_verify_sample(sample(seq=i,settled=False)))
        r.finish(parse_verify_result(result()))
        self.assertFalse(r.passed)

    def test_gui_live_result_and_failure(self):
        client=Mock();client.verify_electrical.return_value=True
        d=AlignmentVerifyDialog(1,client)
        with patch('ui.alignment_verify_dialog.secrets.randbelow',return_value=6):d.start()
        self.assertTrue(d.busy)
        d.receive_result(result(session=8));self.assertTrue(d.busy)
        for i in range(9):d.receive_sample(sample(seq=i))
        d.refresh()
        self.assertEqual(len(d.error_curve.getData()[0]),9)
        d.receive_result(result())
        self.assertIn('通过',d.status.text());self.assertFalse(d.busy)
        self.assertTrue(d.export_btn.isEnabled())
        d.start();d.receive_result(result(session=d.run.session,error=20,count=0))
        self.assertIn('未通过',d.status.text());self.assertFalse(d.busy)
        d.close()

    def test_send_failure_timeout_and_modal_close(self):
        client=Mock();client.verify_electrical.return_value=False
        d=AlignmentVerifyDialog(1,client);d.start()
        self.assertIn('发送失败',d.status.text())
        client.verify_electrical.return_value=True;d.start();d.show()
        d.reject();self.assertTrue(d.busy)
        d.started-=26;d.refresh()
        self.assertIn('超时',d.status.text());self.assertFalse(d.busy)
        d.close()

    def test_cal_panel_wires_dialog(self):
        client=Mock()
        w=MotorCalWidget(1,client)
        self.assertEqual(w._current_align_method(), 0)  # UD default
        w._open_verify();self.assertIsNotNone(w.verify_dialog)
        w.set_motor_id(2);self.assertIsNone(w.verify_dialog)
        w.close()


if __name__=='__main__':unittest.main()
