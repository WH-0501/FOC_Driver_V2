import os
os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')
import unittest
from PyQt5.QtWidgets import QApplication
from ui.waveform_widget import WaveformWidget

class CursorViewTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.app = QApplication.instance() or QApplication([])

    def test_enable_zoom_pan_and_keep_manual_pick(self):
        w = WaveformWidget()
        try:
            w._plot.setXRange(100, 106, padding=0)
            w.cursor_cb.setChecked(True)
            self.assertAlmostEqual(w._cursors[0].value(), 102)
            self.assertAlmostEqual(w._cursors[1].value(), 104)
            w._cursors[0].setValue(102.4)
            w._plot.setXRange(102, 103, padding=0)
            self.assertAlmostEqual(w._cursors[0].value(), 102.4)
            self.assertAlmostEqual(w._cursors[1].value(), 104)
            w._plot.setXRange(200, 200.01, padding=0)
            self.assertEqual([c.value() for c in w._cursors], [102.4, 104])
            from unittest.mock import Mock
            from PyQt5.QtCore import Qt, QPointF
            event = Mock()
            event.button.return_value = Qt.LeftButton
            event.isStart.return_value = True
            event.isFinish.return_value = False
            event.buttonDownPos.return_value = QPointF(103, 0)
            event.pos.return_value = QPointF(104, 0)
            w._cursor_region.mouseDragEvent(event)
            self.assertAlmostEqual(w._cursors[0].value(), 103.4)
            self.assertAlmostEqual(w._cursors[1].value(), 105)
            event.accept.assert_called_once()
            w.cursor_cb.setChecked(False)
            previous = [c.value() for c in w._cursors]
            w._plot.setXRange(300, 301, padding=0)
            self.assertEqual(previous, [c.value() for c in w._cursors])
            w.cursor_cb.setChecked(True)
            self.assertTrue(all(300 < c.value() < 301 for c in w._cursors))
        finally:
            w.close()
