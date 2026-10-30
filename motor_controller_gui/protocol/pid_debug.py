"""One 64-byte CAN-FD frame per loop, including CNT and CMD."""
import math
import struct
from protocol.constants import CMD

COMMANDS = (CMD.DEBUG_CURRENT, CMD.DEBUG_SPEED, CMD.DEBUG_POSITION)
FIELDS = ('reference', 'feedback', 'iq', 'id', 'dt', 'integral', 'output',
          'vbus', 'shaft_speed', 'kp', 'ki_config', 'limit', 'i_limit', 'ramp')


def parse_debug(cmd, payload):
    if cmd not in COMMANDS or len(payload) != 62 or payload[0] != 1:
        return None
    version, flags, tick, *values = struct.unpack('>BBI14f', payload)
    if not all(math.isfinite(v) for v in values):
        return None
    result = dict(zip(FIELDS, values))
    result.update(loop=COMMANDS.index(cmd), active=bool(flags & 1), flags=flags,
                  device_tick_us=tick)
    result['error'] = result['reference'] - result['feedback']
    result['p_term'] = result['kp'] * result['error']
    result['ki_effective'] = result['ki_config'] * (10000 if result['loop'] == 0 else 1)
    return result
