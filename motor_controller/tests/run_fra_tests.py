from pathlib import Path
import argparse,subprocess,os
p=argparse.ArgumentParser();p.add_argument('--build-dir',type=Path,required=True);a=p.parse_args();a.build_dir.mkdir(parents=True,exist_ok=True)
r=Path(__file__).resolve().parents[1];exe=(a.build_dir/'fra_test.exe').resolve();env=os.environ.copy();env['PATH']='C:/Qt/Tools/mingw1310_64/bin;'+env['PATH']
subprocess.run(['C:/Qt/Tools/mingw1310_64/bin/gcc.exe','-std=c99','-O2','-Wall','-Wextra','-Werror',str(r/'project/APP/FOC/fra.c'),str(r/'tests/test_fra.c'),'-I'+str(r/'project/APP/FOC'),'-lm','-o',str(exe)],env=env,check=True)
subprocess.run([str(exe)],env=env,check=True)
