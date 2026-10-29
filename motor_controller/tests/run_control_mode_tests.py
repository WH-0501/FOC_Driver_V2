"""Native Windows MSVC regression of actual mode API, FSM and dispatch code.
Hardware/sensing are mocks; exact functions are extracted to avoid unrelated
encoder, storage and calibration dependencies in this focused test.
"""
from pathlib import Path
import argparse,subprocess,tempfile
p=argparse.ArgumentParser();p.add_argument('--root',type=Path,default=Path(__file__).resolve().parents[1])
p.add_argument('--build-dir',type=Path);p.add_argument('--compile-only',action='store_true')
args=p.parse_args();root=args.root;tests=Path(__file__).resolve().parent
vcvars=Path(r'C:\Program Files (x86)\Microsoft Visual Studio\2022\BuildTools\VC\Auxiliary\Build\vcvars64.bat')
def function(source,name):
    import re
    match=re.search(r'^(?:static )?(?:error_t|void|bool|control_mode_t) '+name+r'\([^;]*?\)\s*\{',source,re.M)
    assert match,name
    start=match.start();brace=source.index('{',match.start());level=1;end=brace+1
    while level:
        if source[end]=='{':level+=1
        elif source[end]=='}':level-=1
        end+=1
    return re.sub(r'^#.*\n','',source[start:end],flags=re.M)+'\n'
app=root/'project/APP'
api=(app/'mc_interface.c').read_text(encoding='utf-8')
foc=(app/'FOC/foc.c').read_text(encoding='utf-8')
protocol=(app/'comm/protocol/custom/can_fd_custom.c').read_text(encoding='utf-8')
impl=function((app/'FOC/foc_fault.c').read_text(encoding='utf-8'),'motor_fault_check_shaft_speed')+''.join(function(api,n) for n in ('mc_set_ctrl_mode','mc_enable'))
impl+=''.join(function(foc,n) for n in ('foc_reject_control_mode','foc_control_run','foc_control_loop'))
impl+=function(protocol,'can_fd_custom_wire_mode_to_ctrl')
impl+=function((app/'Driver/motor_driver.c').read_text(encoding='utf-8'),'motor_driver_disarm')
start=protocol.index('    case CAN_FD_CMD_CONTROL_MODE:')
end=protocol.index('    case ',start+10)
impl+='static void test_protocol_mode(const uint8_t *payload,uint8_t payload_len) { error_t result; uint8_t reply_payload[1]; switch(CAN_FD_CMD_CONTROL_MODE) {\n'+protocol[start:end]+'}\n}\n'
from contextlib import nullcontext
if args.build_dir: args.build_dir.mkdir(parents=True,exist_ok=True)
with (nullcontext(str(args.build_dir.resolve())) if args.build_dir else tempfile.TemporaryDirectory(prefix='foc_c06_',ignore_cleanup_errors=True)) as temp:
    work=Path(temp);(work/'c06_impl.inc').write_text(impl,encoding='utf-8')
    cmd=['cl','/nologo','/TC','/std:c11','/utf-8','/O2','/W3',
         '/I'+str(tests/'c06_support'),'/I'+str(app),'/I'+str(app/'FOC'),'/I'+str(work),
         str(app/'FOC/foc_fsm.c'),str(tests/'test_control_mode_safety.c'),'/Fe:'+str(work/'mode_test.exe')]
    batch=work/'run.cmd';batch.write_text('\n'.join(['@echo off',f'call "{vcvars}" >nul','if errorlevel 1 exit /b 1',subprocess.list2cmdline(cmd),'if errorlevel 1 exit /b 1',('rem compile only' if args.compile_only else f'"{work/"mode_test.exe"}"'),'exit /b %errorlevel%']),encoding='utf-8')
    subprocess.run(['cmd','/d','/c',str(batch)],cwd=work,check=True)
