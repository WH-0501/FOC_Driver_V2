from pathlib import Path
import subprocess,re,argparse
p=argparse.ArgumentParser();p.add_argument('--root',type=Path,required=True);p.add_argument('--build-dir',type=Path,required=True);a=p.parse_args()
src=(a.root/'project/Board/dwt_profile_delay.c').read_text(encoding='utf-8')
header=(a.root/'project/Board/dwt_profile_delay.h').read_text(encoding='utf-8')
strip=lambda s:re.sub(r'^#include.*$','',s,flags=re.M)
stub='''#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
static struct {uint32_t DEMCR;} debug;
static struct {uint32_t CTRL,CYCCNT;} counter;
#define CoreDebug (&debug)
#define DWT (&counter)
#define CoreDebug_DEMCR_TRCENA_Msk (1u<<24)
#define DWT_CTRL_CYCCNTENA_Msk 1u
static uint32_t SystemCoreClock=180000000u;
static unsigned nop_count;
#define __NOP() do {DWT->CYCCNT+=180u;++nop_count;} while(0)
'''
tests='''
#define CHECK(x) do{if(!(x)){printf("FAIL line %d\\n",__LINE__);exit(1);}}while(0)
int main(void){uint32_t before;
 debug.DEMCR=CoreDebug_DEMCR_TRCENA_Msk;counter.CTRL=0;counter.CYCCNT=1234;
 dwt_init();CHECK(dwt_cycle_tim.dwt_freq_MHz==180u);CHECK(counter.CTRL&1u);CHECK(counter.CYCCNT==1234);
 before=counter.CYCCNT;dwt_blocking_delay_ms(5);CHECK(counter.CYCCNT-before>=900000u);
 counter.CYCCNT=0xFFFFFF00u;before=counter.CYCCNT;dwt_blocking_delay_ms(5);CHECK((uint32_t)(counter.CYCCNT-before)>=900000u);
 before=counter.CYCCNT;dwt_init();CHECK(counter.CYCCNT==before);
 dwt_cycle_tim.dwt_freq_MHz=0;dwt_blocking_delay_ms(700);CHECK(counter.CYCCNT-before>=126000000u);
 debug.DEMCR=0;counter.CTRL=0;dwt_blocking_delay_ms(1);CHECK(debug.DEMCR&&counter.CTRL);
 puts("PASS DWT: debugger-preenabled trace, counter enable, preserved counter, zero-factor recovery, 5/700ms delay, wrap");return 0;}
'''
w=a.build_dir.resolve();w.mkdir(parents=True,exist_ok=True)
(w/'test.c').write_text(stub+strip(header)+strip(src)+tests,encoding='utf-8')
vc=Path(r'C:/Program Files (x86)/Microsoft Visual Studio/2022/BuildTools/VC/Auxiliary/Build/vcvars64.bat')
(w/'build.cmd').write_text(f'@echo off\ncall "{vc}" >nul\ncl /nologo /TC /std:c11 /utf-8 /W3 /WX test.c /Fe:test.exe\nexit /b %errorlevel%')
subprocess.run(['cmd','/d','/c',str(w/'build.cmd')],cwd=w,check=True)
