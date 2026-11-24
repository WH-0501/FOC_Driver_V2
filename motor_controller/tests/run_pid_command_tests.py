"""Exercise actual PID command branches and codec; no hardware or motor enable."""
from pathlib import Path
import argparse, subprocess

p=argparse.ArgumentParser()
p.add_argument('--root',type=Path,default=Path(__file__).resolve().parents[1])
p.add_argument('--build-dir',type=Path,required=True)
a=p.parse_args();root=a.root.resolve();w=a.build_dir.resolve();w.mkdir(parents=True,exist_ok=True)
app=root/'project/APP';custom=app/'comm/protocol/custom'
src=(custom/'can_fd_custom.c').read_text(encoding='utf-8')
branches=''
for begin,end in [('SET_POS_PID','SET_STIFF_DAMP'),('GET_POS_PID','GET_STIFF_DAMP')]:
    branches+=src[src.index('    case CAN_FD_CMD_'+begin+':'):src.index('    case CAN_FD_CMD_'+end+':')]
code=r'''
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <assert.h>
#include <stdio.h>
#include "motor.h"
#include "can_fd_custom.h"
#include "can_fd_custom_codec.h"
static bool disabled;
static unsigned clears,replies;
static uint8_t last_cmd,last[6];
static bool can_fd_custom_require_disabled(void){return disabled;}
static void mc_clear_pid_state(void){++clears;}
static void can_fd_custom_reply_pid(uint8_t cmd,const pid_param_t *pid){
  ++replies;last_cmd=cmd;can_fd_codec_pack_pid(pid->kp,pid->ki,pid->kd,last);
}
static void handle(motor_handle_t *m,uint8_t cmd,const uint8_t *payload,uint8_t payload_len){
switch(cmd){
'''+branches+r'''
default:assert(0);}}
int main(void){
  const uint8_t set[3]={CAN_FD_CMD_SET_POS_PID,CAN_FD_CMD_SET_VEL_PID,CAN_FD_CMD_SET_CUR_PID};
  const uint8_t get[3]={CAN_FD_CMD_GET_POS_PID,CAN_FD_CMD_GET_VEL_PID,CAN_FD_CMD_GET_CUR_PID};
  motor_handle_t m,expected;
  uint32_t random=1234567;
  unsigned n,k,len,d,cases=0;
  for(n=0;n<100;++n) for(k=0;k<3;++k) for(len=0;len<=8;++len) for(d=0;d<2;++d){
    uint8_t data[8],reply[6];pid_param_t *pid;
    memset(&m,0,sizeof m);
    m.config.pid.position.limit=12.25f;m.config.pid.velocity.limit=3.5f;
    m.config.pid.id.limit=6.75f;m.config.pid.iq.limit=7.25f;
    m.config.pid.id.i_limit=2.25f;m.config.pid.iq.i_limit=2.75f;
    m.config.pid.id.ramp=8;m.config.pid.iq.ramp=9;
    for(unsigned j=0;j<sizeof data;++j){random=random*1664525u+1013904223u;data[j]=(uint8_t)(random>>24);}
    expected=m;disabled=d!=0;clears=replies=0;
    pid=k==0?&expected.config.pid.position:k==1?&expected.config.pid.velocity:&expected.config.pid.iq;
    if(d&&len>=6){
      pid->kp=((unsigned)data[0]*256u+data[1])/4096.0f;
      pid->ki=((unsigned)data[2]*256u+data[3])/4096.0f;
      pid->kd=((unsigned)data[4]*256u+data[5])/4096.0f;
      if(k==2){expected.config.pid.id.kp=pid->kp;expected.config.pid.id.ki=pid->ki;expected.config.pid.id.kd=pid->kd;}
    }
    handle(&m,set[k],data,(uint8_t)len);
    assert(memcmp(&m,&expected,sizeof m)==0);
    assert(clears==(unsigned)(d&&len>=6)&&replies==clears);
    if(replies){assert(last_cmd==get[k]);assert(memcmp(last,data,6)==0);}
    clears=replies=0;
    handle(&m,get[k],data,(uint8_t)len);
    assert(clears==0&&replies==1&&last_cmd==get[k]);
    can_fd_codec_pack_pid(pid->kp,pid->ki,pid->kd,reply);
    assert(memcmp(last,reply,6)==0&&memcmp(&m,&expected,sizeof m)==0);++cases;
  }
  printf("PASS PID commands: %u cases; exact wire/gains, d/q limits retained, short packets and enabled writes rejected\n",cases);
  return 0;
}
'''
(w/'pid_commands.c').write_text(code,encoding='utf-8')
vc=Path('C:/Program Files (x86)/Microsoft Visual Studio/2022/BuildTools/VC/Auxiliary/Build/vcvars64.bat')
includes=[root/'tests/c06_support',app,app/'FOC',app/'Driver',app/'comm',custom]
cmd=['cl','/nologo','/TC','/std:c11','/utf-8','/O2','/W3','/WX',*['/I'+str(x) for x in includes],str(w/'pid_commands.c'),str(custom/'can_fd_custom_codec.c'),'/Fe:'+str(w/'pid_commands.exe')]
batch=w/'build.cmd'
batch.write_text('@echo off\ncall "'+str(vc)+'" >nul\n'+subprocess.list2cmdline(cmd)+'\nif errorlevel 1 exit /b 1\n"'+str(w/'pid_commands.exe')+'"\nexit /b %errorlevel%\n',encoding='utf-8')
subprocess.run(['cmd','/d','/c',str(batch)],cwd=w,check=True)
