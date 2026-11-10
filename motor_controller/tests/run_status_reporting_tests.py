from pathlib import Path
import subprocess, os
import argparse
p=argparse.ArgumentParser();p.add_argument('--build-dir',type=Path,required=True);args=p.parse_args()
base=args.build_dir.resolve();base.mkdir(parents=True,exist_ok=True)
root=Path(__file__).resolve().parents[1]
def func(path):
 s=path.read_text(encoding='utf-8');a=s.index('static void can_fd_custom_poll(void)\n{'); b=s.index('\nstatic void can_fd_custom_bind_transport',a);return s[a:b]
prefix='''#include <stdint.h>
#include <stdio.h>
static uint32_t ticks, s_last_report_us, s_report_elapsed, s_report_interval_us=1000;
static uint16_t s_report_clock;
static unsigned s_debug_mask=1, s_reply_count, reports;
static uint32_t board_time_us(void){return ticks & 65535;}
static void can_fd_iap_poll(void){}
static void can_fd_debug_clock(void){}
static void can_fd_custom_flush_reply(void){}
static void can_fd_debug_poll(void){}
static void can_fd_custom_period_report(void){++reports;}
'''
main='''int main(void){
for(ticks=100;ticks<=10000000;ticks+=100) can_fd_custom_poll();
printf("debug 10s reports=%u (expected 50)\\n",reports);
if(reports!=50) return 1;
s_debug_mask=0;s_report_interval_us=1000000;reports=0;
for(;ticks<=20000000;ticks+=100) can_fd_custom_poll();
if(reports!=10) return 2;
s_report_interval_us=1000;reports=0;
for(;ticks<=21000000;ticks+=100) can_fd_custom_poll();
if(reports!=1000) return 3;
s_reply_count=1;
for(;ticks<=23000000;ticks+=100) can_fd_custom_poll();
if(reports!=1000) return 4;
s_reply_count=0;can_fd_custom_poll();
if(reports!=1001) return 5;
puts("PASS: 16-bit wraps, 5Hz debug status, 1Hz/1000Hz normal, pending replies");return 0;}
'''
env=os.environ.copy();env['PATH']=r'C:\Qt\Tools\mingw1310_64\bin;'+env['PATH']
for name in ('can_fd_custom',):
 c=base/(name+'_test.c');exe=base/(name+'_test.exe');c.write_text(prefix+func(root/'project/APP/comm/protocol/custom/can_fd_custom.c')+main,encoding='utf-8');subprocess.run(['C:/Qt/Tools/mingw1310_64/bin/gcc.exe',str(c),'-o',str(exe)],env=env,check=True);r=subprocess.run([str(exe)],env=env);assert (r.returncode!=0 if name=='original' else r.returncode==0)
