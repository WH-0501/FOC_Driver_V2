import os
os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')
import unittest
from unittest.mock import patch
from PyQt5.QtWidgets import QApplication, QDockWidget
from PyQt5.QtCore import Qt
from ui.main_window import MainWindow
from canbus.device_manager import DeviceState


class MenuLayoutTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.app = QApplication.instance() or QApplication([])

    def test_layout_and_analysis_action(self):
        w = MainWindow()
        try:
            self.assertFalse(w.connection_dialog.isVisible())
            self.assertIsNotNone(w.menuBar().cornerWidget(Qt.TopRightCorner))
            self.assertNotIn('connectionDock', [d.objectName() for d in w.findChildren(QDockWidget)])
            self.assertFalse(hasattr(w.waveform_widget, 'step_btn'))
            with patch.object(w.waveform_widget, '_show_step_analysis') as analysis:
                w.step_analysis_action.trigger()
                analysis.assert_called_once()
            w.motor_id_spin.setValue(5)
            self.assertEqual(w._motor_id, 5)
            self.assertEqual(w.motor_panel.control_widget.motor_id, 5)
        finally:
            w.close()

    def test_connection_settings_reuses_panel_and_menu_state(self):
        w = MainWindow()
        try:
            panel = w.connection_panel
            w.connection_settings_action.trigger()
            self.assertTrue(w.connection_dialog.isVisible())
            w.connection_dialog.close()
            self.assertIs(w.connection_panel, panel)
            w.connection_settings_action.trigger()
            self.assertTrue(w.connection_dialog.isVisible())
            w._update_connection_menu()
            for action, button in w._connection_actions:
                self.assertEqual(action.isEnabled(), button.isEnabled())
            with patch.object(panel.open_btn, 'click') as click:
                w._connection_actions[0][0].trigger()
                click.assert_called_once()
        finally:
            w.close()
