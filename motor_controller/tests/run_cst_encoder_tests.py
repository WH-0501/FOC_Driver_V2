"""Host regression for the encoder path. Run with python tests/run_cst_encoder_tests.py."""
from itertools import product
import argparse
import re
from pathlib import Path
import subprocess
import tempfile

parser = argparse.ArgumentParser()
parser.add_argument('--build-dir', type=Path)
args = parser.parse_args()
tests = Path(__file__).resolve().parent
app = tests.parent / 'project' / 'APP'
vcvars = Path(r'C:\Program Files (x86)\Microsoft Visual Studio\2022\BuildTools\VC\Auxiliary\Build\vcvars64.bat')

def check(work):
    work.mkdir(parents=True, exist_ok=True)
    (work/'board.h').write_text('#include "datatypes.h"\nvoid board_get_phase_current(motor_handle_t *m);\nvoid board_get_vbus_and_temp(motor_handle_t *m);\nuint32_t board_time_us(void);\nuint32_t __get_PRIMASK(void);\nvoid __disable_irq(void);\nvoid __set_PRIMASK(uint32_t);\n')
    # Build the dual variant by changing only the private joint binding in a temporary copy.
    production = app/'FOC/encoder.c'
    dual_source, count = re.subn(r'(\[ENCODER_ROLE_JOINT\]\s*=\s*\{\s*\.dev\s*=\s*)ENCODER_DEV_NONE(\s*\})',
                                  r'\g<1>ENCODER_DEV_KTH71, .kth71_cs = KTH71_CS_1\2',
                                  production.read_text(encoding='utf-8'))
    assert count == 1, 'expected exactly one unbound joint entry'
    dual = work/'encoder_dual.c'
    dual.write_text(dual_source, encoding='utf-8')
    boards = {'single': production, 'dual': dual}
    for board, divisor, pll in product(boards, (1, 5), (0, 1)):
        exe = work / f'encoder_{board}_{divisor}_{pll}.exe'
        cmd = ['cl','/nologo','/O2','/TC','/std:c11','/utf-8',f'/DFW_FOC_OUTER_LOOP_DIV={divisor}', f'/DFOC_ENCODER_USE_PLL={pll}',
               '/I'+str(work),'/I'+str(tests/'c06_support'),'/I'+str(app),'/I'+str(app/'FOC'),
               '/I'+str(app/'Driver'),'/I'+str(app/'config'),'/I'+str(app.parent/'Board'),str(tests/'test_cst_encoder.c'),str(boards[board]),str(app/'FOC/foc_filter.c'),str(app/'FOC/foc_sense.c'),'/Fe:'+str(exe)]
        batch = work / 'build.cmd'
        batch.write_text('@echo off\ncall "'+str(vcvars)+'" >nul\nif errorlevel 1 exit /b 1\n'
                         +subprocess.list2cmdline(cmd)+'\nexit /b %errorlevel%\n')
        subprocess.run(['cmd','/d','/c',str(batch)],cwd=work,check=True)
        subprocess.run([str(exe), board],cwd=work,check=True)

if args.build_dir:
    check(args.build_dir.resolve())
else:
    with tempfile.TemporaryDirectory(prefix='foc_encoder_') as temp:
        check(Path(temp))

