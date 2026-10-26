"""Run real current_sense code and extracted AT32 polling code with native mocks.
Windows: requires MSVC C tools. Does not model analog settling or real ADC timing.
"""
from pathlib import Path
import argparse, subprocess, tempfile

parser=argparse.ArgumentParser()
parser.add_argument('--root',type=Path,default=Path(__file__).resolve().parents[1])
args=parser.parse_args()
root=args.root
tests=Path(__file__).resolve().parent
vcvars=Path(r'C:\Program Files (x86)\Microsoft Visual Studio\2022\BuildTools\VC\Auxiliary\Build\vcvars64.bat')
with tempfile.TemporaryDirectory(prefix='foc_s03_',ignore_cleanup_errors=True) as temp:
    work=Path(temp)
    board=(root/'project/Board/board_at32.c').read_text(encoding='utf-8')
    impl=board[board.index('error_t current_hw_calibration_begin'):board.index('error_t current_hw_init')]
    (work/'offset_adc_impl.inc').write_text(impl,encoding='utf-8')
    commands=['@echo off',f'call "{vcvars}" >nul','if errorlevel 1 exit /b 1']
    for name,sources in [('test_offset_calibration',[root/'project/APP/FOC/current_sense.c',tests/'test_offset_calibration.c']),
                         ('test_offset_adc',[tests/'test_offset_adc.c'])]:
        cmd=['cl','/nologo','/TC','/std:c11','/utf-8','/O2','/W3',
             '/I'+str(tests/'s03_support'),'/I'+str(root/'project/APP'),'/I'+str(root/'project/APP/FOC'),'/I'+str(work),
             *map(str,sources),'/Fe:'+str(work/(name+'.exe'))]
        commands += [subprocess.list2cmdline(cmd),'if errorlevel 1 exit /b 1',f'"{work/(name+".exe")}"','if errorlevel 1 exit /b 1']
    batch=work/'run.cmd'
    batch.write_text('\n'.join(commands),encoding='utf-8')
    subprocess.run(['cmd','/d','/c',str(batch)],cwd=work,check=True)

foc=(root/'project/APP/FOC/foc.c').read_text(encoding='utf-8')
mc=(root/'project/APP/mc_interface.c').read_text(encoding='utf-8')
assert foc.index('if (!m->current_offset_cal_done)',foc.index('current_sense_init(m);')) < foc.index('cal_elec_align(m);',foc.index('current_sense_init(m);'))
assert mc.index('return ERR_OFFSET_CAL;',mc.index('foc_init(m, &cfg);')) < mc.index('ret = mc_runtime_start();')
print('PASS S03: initialization fails before auto-alignment and current-loop registration')
