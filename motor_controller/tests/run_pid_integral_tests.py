"""Windows native MSVC tests of actual motor_config.c and foc_pid.c (no MCU)."""
from pathlib import Path
import argparse, subprocess, tempfile
p=argparse.ArgumentParser()
p.add_argument('--root',type=Path,default=Path(__file__).resolve().parents[1])
root=p.parse_args().root
tests=Path(__file__).resolve().parent
vcvars=Path(r'C:\Program Files (x86)\Microsoft Visual Studio\2022\BuildTools\VC\Auxiliary\Build\vcvars64.bat')
with tempfile.TemporaryDirectory(prefix='foc_c03_',ignore_cleanup_errors=True) as temp:
    work=Path(temp)
    cmd=['cl','/nologo','/TC','/std:c11','/utf-8','/O2','/W3',
         '/I'+str(tests/'c03_support'),'/I'+str(root/'project/APP'),
         '/I'+str(root/'project/APP/FOC'),'/I'+str(root/'project/APP/config'),
         str(root/'project/APP/config/motor_config.c'),str(root/'project/APP/FOC/foc_pid.c'),
         str(tests/'test_pid_integral_defaults.c'),'/Fe:'+str(work/'pid_test.exe')]
    batch=work/'run.cmd'
    batch.write_text('\n'.join(['@echo off',f'call "{vcvars}" >nul','if errorlevel 1 exit /b 1',
        subprocess.list2cmdline(cmd),'if errorlevel 1 exit /b 1',f'"{work/"pid_test.exe"}"','exit /b %errorlevel%']),encoding='utf-8')
    subprocess.run(['cmd','/d','/c',str(batch)],cwd=work,check=True)
