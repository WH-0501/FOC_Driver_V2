"""One 64-byte CAN-FD frame per loop, including CNT and CMD."""
import math
import struct
from protocol.constants import CMD

COMMANDS = (CMD.DEBUG_CURRENT, CMD.DEBUG_SPEED, CMD.DEBUG_POSITION)
FIELDS_V1 = ('reference', 'feedback', 'iq', 'id', 'dt', 'integral', 'output',
             'vbus', 'shaft_speed', 'kp', 'ki_config', 'limit', 'i_limit', 'ramp')
FIELDS_V2 = ('reference', 'feedback', 'iq', 'id', 'dt', 'integral', 'output',
             'vbus', 'shaft_speed', 'kp', 'ki_config', 'limit',
             'cur_loop_us', 'vel_loop_us', 'pos_loop_us', 'ctrl_run_us')
FIELDS_V3 = ('reference', 'feedback', 'iq', 'id', 'dt', 'integral', 'output',
             'vbus', 'kp', 'ki_config', 'limit',
             'cur_loop_us', 'vel_loop_us', 'pos_loop_us', 'ctrl_run_us', 'control_loop_us')
FIELDS_V5 = ('reference', 'feedback', 'iq', 'id', 'dt', 'integral', 'output',
             'vbus', 'kp',
             'cur_loop_us', 'vel_loop_us', 'pos_loop_us', 'ctrl_run_us', 'control_loop_us',
             'sense_update_us', 'current_sense_us', 'encoder_update_us')
FIELDS = FIELDS_V5


def _finish_row(cmd, flags, tick, fields, values):
    if not all(math.isfinite(v) for v in values):
        return None
    result = dict(zip(fields, values))
    result.update(loop=COMMANDS.index(cmd), active=bool(flags & 1), flags=flags,
                  device_tick_us=tick)
    result['error'] = result['reference'] - result['feedback']
    result['p_term'] = result['kp'] * result['error']
    result['ki_effective'] = result['ki_config'] * (10000 if result['loop'] == 0 else 1)
    return result


def parse_debug(cmd, payload):
    if cmd not in COMMANDS or len(payload) != 62:
        return None
    ver = payload[0]
    if ver == 1:
        flags, tick = struct.unpack('>BI', payload[1:6])
        values = struct.unpack('>14f', payload[6:])
        return _finish_row(cmd, flags, tick, FIELDS_V1, values)
    if ver == 2:
        flags, tick = struct.unpack('>BI', payload[1:6])
        floats = struct.unpack('>12f', payload[6:54])
        loops = struct.unpack('>4H', payload[54:62])
        if not all(math.isfinite(v) for v in floats):
            return None
        result = dict(zip(FIELDS_V2[:12], floats))
        result.update(zip(FIELDS_V2[12:], loops))
        result.update(loop=COMMANDS.index(cmd), active=bool(flags & 1), flags=flags,
                      device_tick_us=tick)
        result['error'] = result['reference'] - result['feedback']
        result['p_term'] = result['kp'] * result['error']
        result['ki_effective'] = result['ki_config'] * (10000 if result['loop'] == 0 else 1)
        return result
    if ver == 3:
        flags, tick = struct.unpack('>BI', payload[1:6])
        floats = struct.unpack('>11f', payload[6:50])
        loops = struct.unpack('>5H', payload[50:60])
        if not all(math.isfinite(v) for v in floats):
            return None
        result = dict(zip(FIELDS_V3[:11], floats))
        result.update(zip(FIELDS_V3[11:], loops))
        result.update(loop=COMMANDS.index(cmd), active=bool(flags & 1), flags=flags,
                      device_tick_us=tick)
        result['error'] = result['reference'] - result['feedback']
        result['p_term'] = result['kp'] * result['error']
        result['ki_effective'] = result['ki_config'] * (10000 if result['loop'] == 0 else 1)
        return result
    if ver == 4:
        flags, tick = struct.unpack('>BI', payload[1:6])
        floats = struct.unpack('>11f', payload[6:50])
        loops = struct.unpack('>6H', payload[50:62])
        if not all(math.isfinite(v) for v in floats):
            return None
        result = dict(zip(FIELDS_V4[:11], floats))
        result.update(zip(FIELDS_V4[11:], loops))
        result.update(loop=COMMANDS.index(cmd), active=bool(flags & 1), flags=flags,
                      device_tick_us=tick)
        result['error'] = result['reference'] - result['feedback']
        result['p_term'] = result['kp'] * result['error']
        result['ki_effective'] = result['ki_config'] * (10000 if result['loop'] == 0 else 1)
        return result
    if ver == 5:
        flags, tick = struct.unpack('>BI', payload[1:6])
        floats = struct.unpack('>9f', payload[6:42])
        loops = struct.unpack('>8H', payload[42:58])
        if not all(math.isfinite(v) for v in floats):
            return None
        result = dict(zip(FIELDS_V5[:9], floats))
        result.update(zip(FIELDS_V5[9:], loops))
        result.update(loop=COMMANDS.index(cmd), active=bool(flags & 1), flags=flags,
                      device_tick_us=tick, ki_config=float('nan'))
        result['error'] = result['reference'] - result['feedback']
        result['p_term'] = result['kp'] * result['error']
        result['ki_effective'] = float('nan')
        return result
    return None
