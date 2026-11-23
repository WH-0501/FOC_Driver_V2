"""Exercise actual IAP code with Flash mocks; never accesses hardware."""
from pathlib import Path
import argparse, subprocess
p=argparse.ArgumentParser(); p.add_argument('--build-dir',type=Path,required=True); a=p.parse_args()
root=Path(__file__).resolve().parents[1]; out=a.build_dir.resolve();out.mkdir(parents=True,exist_ok=True)
app=root/'project/APP'; src=app/'comm/protocol/custom'
cmd=['cl','/nologo','/TC','/std:c11','/utf-8','/W3','/O2','/wd4312',
     '/I'+str(root/'tests/iap_support'),'/I'+str(src),'/I'+str(app/'config'),
     str(root/'tests/test_iap_boundaries.c'),str(src/'iap_upgrade.c'),'/Fe:'+str(out/'iap.exe')]
vc=Path('C:/Program Files (x86)/Microsoft Visual Studio/2022/BuildTools/VC/Auxiliary/Build/vcvars64.bat')
batch=out/'run.cmd';batch.write_text('@echo off\ncall "'+str(vc)+'" >nul\n'+subprocess.list2cmdline(cmd)+'\nif errorlevel 1 exit /b 1\n"'+str(out/'iap.exe')+'"\nexit /b %errorlevel%\n')
subprocess.run(['cmd','/d','/c',str(batch)],cwd=out,check=True)
