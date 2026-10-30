"""Compile actual protocol reply queue/on_rx functions against a fake CAN port."""
from pathlib import Path
import argparse
import re
import subprocess

p = argparse.ArgumentParser()
p.add_argument('--build-dir', type=Path, required=True)
a = p.parse_args()
root = Path(__file__).resolve().parents[1]
src = (root/'project/APP/comm/protocol/custom/can_fd_custom.c').read_text(encoding='utf-8')

def function(name):
    match = re.search(r'^static (?:void|bool) '+name+r'\([^;]*?\)\s*\{', src, re.M)
    start = match.start()
    end = src.index('{', start)+1
    depth = 1
    while depth:
        depth += (src[end] == '{') - (src[end] == '}')
        end += 1
    return src[start:end]+'\n'

code = '\n#include <stdint.h>\n#include <stdbool.h>\n#include <string.h>\n#include <assert.h>\n#include <stdio.h>\n#define CAN_FD_CMD_DEBUG_CURRENT 0xB9\n#define STATE_RUNNING 1\n#define STATE_FAULT 2\n#define CTRL_MODE_TORQUE 1\n#define CTRL_MODE_SPEED 2\n#define CTRL_MODE_POSITION 3\ntypedef int control_mode_t;\ntypedef struct {float target,feedback,dt,last_integral,last_output;uint32_t timestamp;} pid_state_t;\ntypedef struct {float kp,ki,limit,i_limit,ramp;} pid_param_t;\nstatic struct {\n int ctrl_mode,fsm;\n pid_state_t iq_pid,velocity_pid,position_pid;\n struct {struct {pid_param_t iq,velocity,position;} pid;} config;\n struct {float i_q,i_d,vbus_filtered;struct {float velocity_rad_s;} shaft;} state;\n} motor, *s_motor=&motor;\nstatic uint8_t s_debug_mask, s_debug_next;\nstatic uint16_t s_debug_clock;\nstatic uint32_t s_debug_age, s_debug_elapsed[3], s_debug_period=10000;\nstatic uint32_t ticks;\nstatic unsigned sent;\nstatic bool blocked, iap;\nstatic uint8_t last[62],cmds[16];\nstatic unsigned s_node_id=1;\nstatic uint32_t __get_PRIMASK(void){return 0;}\nstatic void __disable_irq(void){}\nstatic void __set_PRIMASK(uint32_t x){(void)x;}\nstatic uint32_t board_time_us(void){return ticks;}\nstatic bool can_fd_iap_is_active(void){return iap;}\nstatic bool can_fd_custom_send_frame(unsigned id,uint8_t cmd,const uint8_t *p,unsigned n){\n assert(id==1&&n==62);if(blocked)return false;\n cmds[sent++]=cmd;memcpy(last,p,n);return true;\n}\n'
code += function("can_fd_debug_send")+function("can_fd_debug_clock")+function("can_fd_debug_poll")
code += '\nint main(void){\n motor.fsm=STATE_RUNNING;motor.ctrl_mode=CTRL_MODE_TORQUE;\n motor.iq_pid.target=.25f;motor.iq_pid.timestamp=0x12345678;\n motor.config.pid.iq.kp=4;motor.state.vbus_filtered=24;\n can_fd_debug_poll();assert(sent==0);\n s_debug_mask=7;ticks=10000;can_fd_debug_clock();\n blocked=true;can_fd_debug_poll();assert(sent==0&&s_debug_elapsed[0]>=10000);\n blocked=false;can_fd_debug_poll();\n assert(sent==1&&cmds[0]==0xB9&&last[0]==1&&last[1]==1);\n assert(last[2]==0x12&&last[5]==0x78&&last[6]==0x3E&&last[7]==0x80);\n can_fd_debug_poll();assert(sent==2&&cmds[1]==0xBA&&last[1]==0);\n can_fd_debug_poll();assert(sent==3&&cmds[2]==0xBB);\n can_fd_debug_poll();assert(sent==3);\n for(int i=0;i<301;i++){ticks+=10000;can_fd_debug_clock();}\n assert(s_debug_mask==0); /* lease expiry across 16-bit wraps */\n s_debug_mask=1;s_debug_age=0;iap=true;can_fd_debug_clock();assert(s_debug_mask==0);\n puts("PASS debug: one frame/loop, endian, active flag, normal off, busy retry, fairness, rate, lease, IAP");\n return 0;\n}\n'
w = a.build_dir.resolve()
w.mkdir(parents=True, exist_ok=True)
(w/'debug_test.c').write_text(code, encoding='utf-8')
vc = Path(r'C:/Program Files (x86)/Microsoft Visual Studio/2022/BuildTools/VC/Auxiliary/Build/vcvars64.bat')
batch = w/'build.cmd'
batch.write_text(f'@echo off\ncall "{vc}" >nul\ncl /nologo /TC /std:c11 /utf-8 /W3 /WX debug_test.c /Fe:debug_test.exe\nif errorlevel 1 exit /b 1\ndebug_test.exe\nexit /b %errorlevel%', encoding='utf-8')
subprocess.run(['cmd', '/d', '/c', str(batch)], cwd=w, check=True)
