from pathlib import Path
import subprocess, tempfile
tests=Path(__file__).resolve().parent;app=tests.parent/'project/APP'
vcvars=Path('C:/Program Files (x86)/Microsoft Visual Studio/2022/BuildTools/VC/Auxiliary/Build/vcvars64.bat')
with tempfile.TemporaryDirectory(prefix='encoder_electrical_',ignore_cleanup_errors=True) as tmp:
    work=Path(tmp)
    (work/'board.h').write_text('#include "datatypes.h"\nuint32_t board_time_us(void);\n')
    cmd=['cl','/nologo','/O2','/TC','/std:c11','/utf-8','/DFOC_ENCODER_USE_PLL=0',
         '/I'+str(work),'/I'+str(tests/'c06_support'),'/I'+str(app),'/I'+str(app/'FOC'),
         '/I'+str(app/'Driver'),'/I'+str(app/'config'),'/I'+str(app.parent/'Board'),
         str(tests/'test_encoder_electrical.c'),str(app/'FOC/encoder.c'),str(app/'FOC/foc_filter.c'),'/Fe:'+str(work/'test.exe')]
    batch=work/'build.cmd'
    batch.write_text('@echo off\ncall "'+str(vcvars)+'" >nul\nif errorlevel 1 exit /b 1\n'+subprocess.list2cmdline(cmd)+'\nexit /b %errorlevel%\n')
    subprocess.run(['cmd','/d','/c',str(batch)],cwd=work,check=True)
    subprocess.run([str(work/'test.exe')],cwd=work,check=True)
