"""Execute extracted production PWM functions using MSVC and timer mocks.
Does not measure MCU timing or simulate the analog power stage.
"""
from pathlib import Path
import subprocess,tempfile
r=Path(__file__).resolve().parents[1]
text=(r/'project/Board/board_at32.c').read_text(encoding='utf-8')
def block(begin,end):return text[text.index(begin):text.index(end,text.index(begin))]
impl=block('static void pwm_stage_compare(', '/* All phase channels')
impl+=block('error_t pwm_hw_stop(', 'void board_fault_emergency_stop_hw(')
impl+=block('static bool pwm_at32_duties_in_range(', 'void board_get_phase_current(')
for channel,value in ((3,'u'),(2,'v'),(1,'w')):
    impl=impl.replace(f'PWM_TIM_HANDLE->c{channel}dt = {value};',f'tmr_channel_value_set(PWM_TIM_HANDLE, {channel-1}u, {value});')
header=(r/'project/Board/board.h').read_text(encoding='utf-8')
begin=header.index('static inline bool pwm_duties_in_range(')
reference=header[begin:header.index('\n}',begin)+2]
vcvars=Path('C:/Program Files (x86)/Microsoft Visual Studio/2022/BuildTools/VC/Auxiliary/Build/vcvars64.bat')
with tempfile.TemporaryDirectory(prefix='pwm_guard_',ignore_cleanup_errors=True) as temp:
    w=Path(temp)
    (w/'pwm_impl.inc').write_text(reference+'\n'+impl,encoding='utf-8')
    cmds=['@echo off',f'call "{vcvars}" >nul','if errorlevel 1 exit /b 1']
    for direct in (0,1):
        exe=w/f'pwm_guard_{direct}.exe'
        cmd=['cl','/nologo','/TC','/std:c11','/utf-8','/O2','/fp:precise','/W3',f'/DAT32_PWM_INLINE_STAGE_CCR={direct}',
             '/I'+str(w),'/I'+str(r/'tests/c06_support'),'/I'+str(r/'project/APP'),'/I'+str(r/'project/APP/FOC'),'/I'+str(r/'project/Board'),
             str(r/'tests/test_pwm_guard.c'),'/Fe:'+str(exe)]
        cmds += [subprocess.list2cmdline(cmd),'if errorlevel 1 exit /b 1',str(exe),'if errorlevel 1 exit /b 1']
    batch=w/'run.cmd';batch.write_text('\n'.join(cmds),encoding='utf-8')
    subprocess.run(['cmd','/d','/c',str(batch)],cwd=w,check=True)
