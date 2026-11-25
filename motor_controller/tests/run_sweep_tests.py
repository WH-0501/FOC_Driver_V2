"""Native numerical, lifecycle and wire regression; never connects to a motor."""
from pathlib import Path
import argparse
import os
import re
import subprocess

p = argparse.ArgumentParser()
p.add_argument('--build-dir', type=Path, required=True)
p.add_argument('--cc', default='C:/Qt/Tools/mingw1310_64/bin/gcc.exe')
p.add_argument('--msvc', action='store_true', help='Use MSVC when Windows blocks MinGW executables')
a = p.parse_args()
root = Path(__file__).resolve().parents[1]
tests = root / 'tests'
app = root / 'project/APP'
sweep = app / 'FOC/sweep'
work = a.build_dir.resolve()
work.mkdir(parents=True, exist_ok=True)
env = os.environ.copy()
env['PATH'] = str(Path(a.cc).parent) + os.pathsep + env['PATH']

def function(src, name, keep_directives=False):
    if not keep_directives: src = re.sub(r'^#.*\n', '', src, flags=re.M)
    match = re.search(r'^(?:static )?(?:void|float|bool|uint8_t|can_data_length_type|error_status) ' + name + r'\([^;]*?\)\s*\{', src, re.M)
    assert match, name
    start = match.start()
    end = src.index('{', start) + 1
    depth = 1
    while depth:
        depth += (src[end] == '{') - (src[end] == '}')
        end += 1
    return src[start:end] + '\n'

foc = (app / 'FOC/foc.c').read_text(encoding='utf-8')
impl = ''.join(function(foc, name) for name in (
    'foc_update_speed_pid', 'foc_position_speed_ref', 'foc_reject_control_mode',
    'foc_control_run', 'foc_control_loop'))
(work / 'sweep_foc.inc').write_text(impl, encoding='utf-8')
# Verify lifecycle/transport hooks in the real entry points as well as executing
# the actual algorithm, adapter, protocol and FOC dispatch below.
motor = (app / 'motor.c').read_text(encoding='utf-8')
for name in ('motor_disarm', 'motor_shutdown'):
    assert 'foc_sweep_on_disarm(m)' in function(motor, name)
protocol = (app / 'comm/protocol/custom/can_fd_custom.c').read_text(encoding='utf-8')
assert 'foc_sweep_stop(m, SWEEP_COMMAND)' in function(protocol, 'can_fd_custom_apply_curr')
assert 's_request_unicast && sweep_protocol_handle' in protocol
assert 'cmd <= SWEEP_CMD_RESULT && frame->id != s_node_id' in protocol
assert 'foc_sweep_check_voltage' not in foc and 'foc_sweep_inject' not in foc
assert 'if (foc_sweep_is_active(m)) foc_sweep_run_isr' in foc

schedule = function((app/'comm/comm.c').read_text(encoding='utf-8'), 'comm_tx_ready')
schedule += function(protocol, 'can_fd_custom_send_frame')
schedule += function(protocol, 'can_fd_custom_poll', keep_directives=True)
(work/'schedule_protocol.inc').write_text(schedule, encoding='utf-8')

port = (app/'comm/transport/can_port_at32.c').read_text(encoding='utf-8')
transport = ''.join(function(port, name, keep_directives=True) for name in (
    'can_port_dlc_to_bytes', 'can_port_bytes_to_dlc', 'can_port_at32_write', 'can_port_at32_kick_bulk', 'can_port_at32_tx_ready', 'can_port_at32_send'))
(work/'sweep_transport.inc').write_text(transport, encoding='utf-8')
vendor = (root/'libraries/drivers/src/at32m412_416_can.c').read_text(encoding='utf-8')
(work/'sweep_vendor_write.inc').write_text(function(vendor,'can_txbuf_write',keep_directives=True),encoding='utf-8')
assert 'can_port_at32_kick_bulk();' in port[port.index('static unsigned int can_port_at32_poll_rx'):]
includes = [tests/'sweep_support', tests/'c06_support', app, app/'FOC', app/'comm', root/'project/Board', sweep, work]
for name, sources in (
    ('schedule', [tests/'test_sweep_schedule.c']),
    ('transport', [tests/'test_sweep_transport.c', sweep/'sweep_record.c']),
    ('record', [tests/'test_sweep_record.c', sweep/'sweep_record.c']),
    ('wave', [tests/'test_sweep_chirp_wave.c']),
    ('parity', [tests/'test_sweep_parity.c', tests/'sweep_support/reference_sweep_engine.c', sweep/'sweep_engine.c', sweep/'sweep_chirp.c', sweep/'sweep_prepare.c']),
    ('grid', [tests/'test_sweep_grid.c', sweep/'sweep_engine.c', sweep/'sweep_chirp.c']),
    ('engine', [tests/'test_sweep_engine.c', sweep/'sweep_engine.c', sweep/'sweep_chirp.c', sweep/'sweep_prepare.c']),
    ('integration', [tests/'test_sweep_integration.c', sweep/'sweep_engine.c', sweep/'sweep_chirp.c', sweep/'sweep_prepare.c',
                     sweep/'foc_sweep.c', sweep/'sweep_protocol.c', sweep/'sweep_record.c', app/'FOC/foc_fsm.c']),
    ('disabled', [tests/'test_sweep_disabled.c', sweep/'sweep_protocol.c']),
):
    exe = work / (name + '.exe')
    defines = ['FOC_SWEEP_ENABLED=0' if name == 'disabled' else 'FOC_SWEEP_ENABLED=1']
    cmd = [a.cc, *['-D'+d for d in defines], '-std=c11', '-O2', '-Wall', '-Wextra', '-Werror', '-Wno-unused-parameter', '-Wno-unused-function',
           *['-I'+str(i) for i in includes], *[str(s) for s in sources], '-lm', '-o', str(exe)]
    if a.msvc:
        vcvars = Path('C:/Program Files (x86)/Microsoft Visual Studio/2022/BuildTools/VC/Auxiliary/Build/vcvars64.bat')
        cmd = ['cl', '/nologo', '/TC', '/std:c11', '/utf-8', '/O2', '/W3', '/WX',
               *['/D'+d for d in defines], *['/I'+str(i) for i in includes],
               *[str(s) for s in sources], '/Fe:'+str(exe)]
        batch = work / (name+'.cmd')
        batch.write_text('@echo off\ncall "'+str(vcvars)+'" >nul\n'+subprocess.list2cmdline(cmd)+
                         '\nif errorlevel 1 exit /b 1\n"'+str(exe)+'"\nexit /b %errorlevel%\n', encoding='utf-8')
        subprocess.run(['cmd', '/d', '/c', str(batch)], cwd=work, check=True)
    else:
        subprocess.run(cmd, cwd=work, env=env, check=True)
        subprocess.run([str(exe)], cwd=work, env=env, check=True)
