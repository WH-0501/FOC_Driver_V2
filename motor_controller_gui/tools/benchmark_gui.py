"""Synthetic 21-motor 1kHz virtual-CAN replay; includes Qt paint/event loop.

Run from source: python tools/benchmark_gui.py. Does not transmit to hardware.
"""
import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import json
import struct
import threading
import time
import numpy as np
import can
from PyQt5.QtCore import QTimer
from PyQt5.QtWidgets import QApplication
from ui.main_window import MainWindow
from protocol import CMD

app = QApplication([])
w = MainWindow()
w.waveform_widget._curve_selector.select_all(True)
profile = 'Virtual CAN-FD (软件测试)'
w.can_thread.manager.open_device(profile, serial_port='benchmark')
w.can_thread.manager.start_channel()
w.can_thread.running = True
w.can_thread.start()
w.show()
gaps = []
last = time.perf_counter()
sent = [0]

def heartbeat():
    global last
    now = time.perf_counter()
    gaps.append((now-last)*1000)
    last = now

def producer():
    peer = can.Bus(interface='virtual', channel='benchmark')
    begin = time.perf_counter()
    try:
        for tick in range(5000):
            deadline = begin + tick*.001
            delay = deadline-time.perf_counter()
            if delay > 0:
                time.sleep(delay)
            value = int(500*np.sin(tick*.02))
            payload = struct.pack('>hhhhhhhBB', value, value, value, 0, value, -value, 0, 0, 0)
            for motor in range(1, 22):
                peer.send(can.Message(arbitration_id=motor, is_extended_id=False, is_fd=True,
                                      data=bytes([tick % 256, CMD.PERIOD_REPORT])+payload+bytes(2)))
                sent[0] += 1
    finally:
        peer.shutdown()

def finish():
    result = dict(sent_frames=sent[0],
        cached_samples=sum(len(b) for b in w.waveform_widget._buffers.values()),
        dropped_frames=w.can_thread.telemetry_dropped,
        pending=w.can_thread.telemetry_pending,
        heartbeat_16ms_p95_ms=round(float(np.percentile(gaps,95)),2),
        heartbeat_max_ms=round(max(gaps),2))
    print(json.dumps(result, indent=2))
    w.close()
    app.quit()

timer = QTimer()
timer.timeout.connect(heartbeat)
timer.start(16)
threading.Thread(target=producer, daemon=True).start()
QTimer.singleShot(6500, finish)
app.exec_()
