"""JSON-lines compatibility process. Never imports the original GUI tree."""
import json
import sys
import threading
import time
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parent))
sys.path.insert(0, str(Path(__file__).resolve().parent / 'site-packages'))
from canbus.device_manager import CanDeviceManager

lock = threading.Lock()
manager = CanDeviceManager()
running = threading.Event()

def emit(obj):
    with lock:
        sys.stdout.write(json.dumps(obj, separators=(',', ':')) + '\n')
        sys.stdout.flush()

def receive():
    try:
        while running.is_set():
            batch = manager.recv_batch(timeout=0.002, max_messages=256)
            if batch:
                emit({'frames': [[m.arbitration_id, bytes(m.data).hex(), m.timestamp] for m in batch]})
            elif manager.last_error:
                raise RuntimeError(manager.last_error)
    except Exception as exc:
        emit({'error': str(exc)})
        running.clear()

worker = None
try:
    for line in sys.stdin:
        try:
            req = json.loads(line)
            if req['op'] == 'open':
                manager.open_device(req['profile'], req.get('index', 0), req.get('channel', ''))
                manager.start_channel(int(req.get('can_channel', 0)))
                running.set()
                worker = threading.Thread(target=receive, daemon=True)
                worker.start()
                emit({'opened': True})
            elif req['op'] == 'send':
                # BRS is required for FD; legacy manager does not set it for python-can.
                import can
                data = bytes.fromhex(req['data'])
                if manager.bus is not None:
                    fd = manager.profile.fd
                    if len(data) > (64 if fd else 8):
                        raise ValueError('经典 CAN 无法发送此帧；请选择 CAN FD 后端')
                    manager.bus.send(can.Message(arbitration_id=req['node'], data=data,
                                                is_extended_id=False, is_fd=fd, bitrate_switch=fd), timeout=.05)
                elif not manager.send(req['node'], data, True):
                    raise RuntimeError(manager.last_error or '发送失败')
            elif req['op'] == 'close':
                break
        except Exception as exc:
            emit({'error': str(exc)})
finally:
    running.clear()
    if worker:
        worker.join(timeout=1)
    manager.close_device()
