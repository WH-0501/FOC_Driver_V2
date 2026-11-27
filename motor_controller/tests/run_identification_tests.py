"""Compile real identification code; synthetic plant only, never accesses hardware."""
from pathlib import Path
import argparse, os, subprocess
p=argparse.ArgumentParser()
p.add_argument('--build-dir',type=Path,required=True)
p.add_argument('--cc',default='C:/Qt/Tools/mingw1310_64/bin/gcc.exe')
a=p.parse_args()
root=Path(__file__).resolve().parents[1]
app=root/'project/APP'
ident=app/'FOC/identification'
out=a.build_dir.resolve(); out.mkdir(parents=True,exist_ok=True)
env=os.environ.copy(); env['PATH']=str(Path(a.cc).parent)+os.pathsep+env['PATH']
exe=out/'ident_engine_test.exe'
cmd=[a.cc,'-std=c11','-O2','-Wall','-Wextra','-Werror','-I'+str(ident),'-I'+str(app/'FOC'),'-I'+str(app),
     str(root/'tests/test_ident_engine.c'),str(ident/'ident_engine.c'),str(app/'FOC/foc_math.c'),'-lm','-o',str(exe)]
subprocess.run(cmd,cwd=out,env=env,check=True)
subprocess.run([str(exe)],cwd=out,env=env,check=True)
exe=out/'ident_integration_test.exe'
includes=[root/'tests/sweep_support',root/'tests/c06_support',app,app/'config',app/'FOC',root/'project/Board']
cmd=[a.cc,'-std=c11','-O2','-Wall','-Wextra','-Werror','-Wno-unused-function',*['-I'+str(x) for x in includes],
     str(root/'tests/test_ident_integration.c'),str(ident/'ident_engine.c'),str(app/'FOC/foc_math.c'),
     str(ident/'foc_identification.c'),str(ident/'ident_protocol.c'),str(app/'FOC/foc_fsm.c'),
     '-lm','-o',str(exe)]
subprocess.run(cmd,cwd=out,env=env,check=True)
subprocess.run([str(exe)],cwd=out,env=env,check=True)
