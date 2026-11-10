"""Verify the PID debug payload and scheduler against a fake CAN port."""
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
    match = re.search(r'^static (?:void|bool|uint16_t) '+name+r'\([^;]*?\)\s*\{', src, re.M)
    start = match.start()
    end = src.index('{', start)+1
    depth = 1
    while depth:
        depth += (src[end] == '{') - (src[end] == '}')
        end += 1
    return src[start:end]+'\n'

code = r'''
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <assert.h>
#include <stdio.h>
#include "motor.h"
#include "can_fd_custom.h"
#include "can_fd_custom_codec.h"
static motor_handle_t motor, *s_motor = &motor;
static motor_runtime_t runtime;
motor_runtime_t *motor_runtime(motor_handle_t *m) { return m == s_motor ? &runtime : NULL; }
static uint8_t s_debug_mask, s_debug_next;
static uint16_t s_debug_clock;
static uint32_t s_debug_age, s_debug_elapsed[3], s_debug_period=10000;
static uint32_t ticks;
static unsigned sent;
static bool blocked, iap;
static uint8_t last[62],cmds[16];
static unsigned s_node_id=1;
static uint32_t __get_PRIMASK(void){return 0;}
static void __disable_irq(void){}
static void __set_PRIMASK(uint32_t x){(void)x;}
static uint32_t board_time_us(void){return ticks;}
static bool can_fd_iap_is_active(void){return iap;}
static bool can_fd_custom_send_frame(unsigned id,uint8_t cmd,const uint8_t *p,unsigned n){
 assert(id==1&&n==62);if(blocked)return false;
 cmds[sent++]=cmd;memcpy(last,p,n);return true;
}
'''
code += function("can_fd_debug_us_u16")+function("can_fd_debug_send")+function("can_fd_debug_clock")+function("can_fd_debug_poll")
code += r'''
int main(void){
 runtime.foc_control_loop_us=0x1234;
 motor.fsm=STATE_RUNNING;motor.ctrl_mode=CTRL_MODE_TORQUE;
 motor.ref.iq=.25f;motor.ref.speed_rad_s=2.0f;motor.ref.position_rad=-1.0f;motor.iq_pid.target=.12f;motor.iq_pid.timestamp=0x12345678;
 motor.config.pid.iq.kp=4;motor.state.vbus_filtered=24;
 can_fd_debug_poll();assert(sent==0);
 s_debug_mask=7;ticks=10000;can_fd_debug_clock();
 blocked=true;can_fd_debug_poll();assert(sent==0&&s_debug_elapsed[0]>=10000);
 blocked=false;can_fd_debug_poll();
 assert(sent==1&&cmds[0]==0xB9&&last[0]==CAN_FD_DEBUG_PAYLOAD_VERSION&&last[1]==9);
 assert(last[50]==0x12 && last[51]==0x34);
 assert(last[2]==0x12&&last[5]==0x78&&last[6]==0x3E&&last[7]==0x80);
 can_fd_debug_poll();assert(sent==2&&cmds[1]==0xBA&&last[1]==8&&last[6]==0x40&&last[7]==0);
 can_fd_debug_poll();assert(sent==3&&cmds[2]==0xBB&&last[6]==0xBF&&last[7]==0x80);
 can_fd_debug_poll();assert(sent==3);
 motor.ctrl_mode=CTRL_MODE_CST;can_fd_debug_send(0);assert(sent==4&&last[1]==9&&last[6]==0x3E&&last[7]==0x80);
 can_fd_debug_send(1);assert(sent==5&&last[1]==8);
 for(int i=0;i<301;i++){ticks+=10000;can_fd_debug_clock();}
 assert(s_debug_mask==7); /* persistent across 16-bit wraps, no renewal */
 s_debug_mask=1;s_debug_age=0;iap=true;can_fd_debug_clock();assert(s_debug_mask==0);
 puts("PASS debug: one frame/loop, endian, active flag, normal off, busy retry, fairness, rate, persistent subscription, IAP");
 return 0;
}
'''
w = a.build_dir.resolve()
w.mkdir(parents=True, exist_ok=True)
(w/'debug_test.c').write_text(code, encoding='utf-8')
vc = Path(r'C:/Program Files (x86)/Microsoft Visual Studio/2022/BuildTools/VC/Auxiliary/Build/vcvars64.bat')
app = root/'project/APP'
custom = app/'comm/protocol/custom'
includes = ' '.join(f'/I"{path}"' for path in (root/'tests/c06_support', app, app/'FOC', app/'Driver', app/'comm', custom))
batch = w/'build.cmd'
batch.write_text(f'@echo off\ncall "{vc}" >nul\ncl /nologo /TC /std:c11 /utf-8 /W3 /WX {includes} debug_test.c "{custom}/can_fd_custom_codec.c" /Fe:debug_test.exe\nif errorlevel 1 exit /b 1\ndebug_test.exe\nexit /b %errorlevel%', encoding='utf-8')
subprocess.run(['cmd', '/d', '/c', str(batch)], cwd=w, check=True)
