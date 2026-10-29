import os
os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')
import unittest
from unittest.mock import Mock, patch
from PyQt5.QtWidgets import QApplication
from core.motor_client import MotorClient
from protocol import CMD
from ui.control_widget import MotorControlWidget


class AckTests(unittest.TestCase):
    def client(self):
        transport = Mock()
        transport.manager.profile.fd = True
        transport.send_message.return_value = True
        logs = []
        return MotorClient(transport, logs.append), transport, logs

    def test_request_identity_and_rtt(self):
        client, transport, logs = self.client()
        with patch('core.motor_client.time.monotonic', return_value=10.):
            client.set_enable(3, True)
        data = transport.send_message.call_args.args[1]
        self.assertEqual(transport.send_message.call_args.args[0], 3)
        client.command_ack(4, bytes([data[0], CMD.ENABLE]))
        self.assertEqual(len(client._command_pending), 1)
        with patch('core.motor_client.time.monotonic', return_value=10.025):
            client.command_ack(3, bytes([data[0], CMD.ENABLE]))
        self.assertFalse(client._command_pending)
        self.assertIn('25.00ms', logs[-1])

    def test_broadcast_failure_and_timeout(self):
        client, transport, logs = self.client()
        client.broadcast_enable(False)
        self.assertFalse(client._command_pending)
        transport.send_message.return_value = False
        client.set_enable(3, True)
        self.assertFalse(client._command_pending)
        transport.send_message.return_value = True
        with patch('core.motor_client.time.monotonic', return_value=10.):
            client.set_mode(3, 1)
        with patch('core.motor_client.time.monotonic', return_value=41.):
            client.expire_command_acks()
        self.assertIn('未收到', logs[-1])
        self.assertEqual(transport.send_message.call_count, 3)

    def test_single_motor_controls_always_unicast(self):
        app = QApplication.instance() or QApplication([])
        client, transport, logs = self.client()
        widget = MotorControlWidget(3, client)
        try:
            widget.enable_btn.click()
            self.assertEqual(transport.send_message.call_args.args[0], 3)
            widget.motor_id = 7
            widget.disable_btn.click()
            self.assertEqual(transport.send_message.call_args.args[0], 7)
            widget.set_mode_btn.click()
            self.assertEqual(transport.send_message.call_args.args[0], 7)
            widget.sync_btn.click()
            self.assertEqual(transport.send_message.call_args.args[0], 7)
        finally:
            widget.close()

    def test_batch_only_addresses_checked_motors(self):
        from ui.multi_motor_widget import MultiMotorWidget
        app = QApplication.instance() or QApplication([])
        client, transport, logs = self.client()
        widget = MultiMotorWidget(client, lambda: [2, 5, 8])
        try:
            widget.refresh_from_motors()
            widget.table.cellWidget(1, 4).setChecked(False)
            for button in (widget.enable_all_btn, widget.disable_all_btn,
                           widget.fault_reset_all_btn, widget.send_pos_btn,
                           widget.send_vel_btn, widget.send_curr_btn):
                transport.send_message.reset_mock()
                button.click()
                expected = [2, 2, 8, 8] if button is widget.fault_reset_all_btn else [2, 8]
                self.assertEqual([c.args[0] for c in transport.send_message.call_args_list], expected)
            for row in (0, 2):
                widget.table.cellWidget(row, 4).setChecked(False)
            transport.send_message.reset_mock()
            widget.enable_all_btn.click()
            transport.send_message.assert_not_called()
        finally:
            widget.close()

    def test_origin_uses_selected_motor(self):
        from ui.cal_widget import MotorCalWidget
        app = QApplication.instance() or QApplication([])
        client, transport, logs = self.client()
        widget = MotorCalWidget(9, client)
        try:
            widget.origin_btn.click()
            self.assertEqual(transport.send_message.call_args.args[0], 9)
            self.assertEqual(transport.send_message.call_args.args[1][1], CMD.SET_ORIGIN)
        finally:
            widget.close()
