from pathlib import Path
import subprocess, tempfile, re, argparse
default_root=Path(__file__).resolve().parents[1]
if not (default_root/'project/APP').exists():
 default_root=Path(r'D:/workspaces/code-registories/FOC/FOC_Driver_V2/motor_controller')
p=argparse.ArgumentParser();p.add_argument('--root',type=Path,default=default_root);p.add_argument('--source',type=Path);p.add_argument('--build-dir',type=Path);p.add_argument('--compile-only',action='store_true');a=p.parse_args()
root=a.root
source=(a.source or root/'project/APP/FOC/foc_calibration.c').read_text(encoding='utf-8')
header=(root/'project/APP/FOC/foc_calibration.h').read_text(encoding='utf-8')
common=(root/'project/APP/common.h').read_text(encoding='utf-8')
macros='\n'.join(x for x in common.splitlines() if re.match(r'#define ALIGN_',x))
header=re.sub(r'^#include.*$','',header,flags=re.M)
source=re.sub(r'^#include.*$','',source,flags=re.M)
stub=r'''
#include <math.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#define _PI 3.14159265358979323846f
#define _2PI (2.0f*_PI)
#define _3PI_2 (1.5f*_PI)
#define CURRENT_SENSE_PHASE_POLARITY (-1.0f)
#define DIR_CW 1
#define DIR_CCW -1
#define FAULT_ALIGN_FAILED 1
typedef int error_t;
enum {ERR_NONE,ERR_NULL,ERR_PARAM,ERR_ALIGIN_FAILED,ERR_ALIGIN_SPREAD_OUT};
typedef struct {
 struct {struct {float theta_elec_offset_rad;int direction;unsigned pole_pairs;} param;} config;
 struct {struct {float angle_rad;} shaft;float theta_elec_rad,i_d,i_q;} state;
 struct {unsigned pwm_enable;} out;int id_pid,iq_pid;
} motor_handle_t;
typedef struct {bool align_direction_flipped,align_ok,align_engaged,electrical_align_done;
 float align_raw_elec_rad,align_raw_engage_rad,align_polarity_test_iq,
 align_theta_before_rad,align_theta_after_rad,open_loop_elec_angle_rad;} motor_runtime_t;
static motor_runtime_t runtime;
static float physical,field,zero;static int sensor_sign=1,physical_pp=5,stuck,armed,fault;
static int jitter,reads;
static float angle_normalize(float a){a=fmodf(a,_2PI);return a<0?a+_2PI:a;}
static motor_runtime_t *motor_runtime(motor_handle_t *m){(void)m;return &runtime;}
static void motor_arm(motor_handle_t *m){(void)m;armed=1;}
static void motor_disarm(motor_handle_t *m){(void)m;armed=0;}
static void motor_fault_raise(motor_handle_t *m,int f){(void)m;fault|=f;}
static void delay_ms(uint32_t ms){(void)ms;}
static int foc_pid_clear(int *p){*p=0;return 0;}
static void foc_voltage(motor_handle_t *m,float uq,float ud,float theta){
 (void)m;
 if(!stuck && armed && (ud!=0 || uq!=0)){
  float target=theta+atan2f(uq,ud);
  physical+=(angle_normalize(target-field+_PI)-_PI)/(float)physical_pp;
  field=target;
 }
}
static void foc_sense_update(motor_handle_t *m){
 float noise=jitter?((++reads&1)?0.0001f:-0.0001f):0;
 m->state.shaft.angle_rad=angle_normalize((sensor_sign*physical+zero+noise)*m->config.param.direction);
 m->state.theta_elec_rad=angle_normalize(m->state.shaft.angle_rad*m->config.param.pole_pairs-m->config.param.theta_elec_offset_rad);
 m->state.i_q=-0.388f;
}
'''
tests=r'''
#define CHECK(x) do{if(!(x)){printf("FAIL line %d: %s\n",__LINE__,#x);exit(1);}}while(0)
static void reset(motor_handle_t *m){memset(m,0,sizeof(*m));memset(&runtime,0,sizeof(runtime));m->config.param.direction=1;m->config.param.pole_pairs=5;m->config.param.theta_elec_offset_rad=1.23f;physical=field=0;zero=0.7f;sensor_sign=1;physical_pp=5;stuck=fault=armed=jitter=reads=0;}
int main(void){motor_handle_t m;cal_elec_align_cfg_t cfg;cal_ctx_t ctx;float spread;
 for(int sign=-1;sign<=1;sign+=2)for(int initial=-1;initial<=1;initial+=2)for(int pp=1;pp<=7;pp+=2){
 reset(&m);sensor_sign=sign;m.config.param.direction=initial;physical_pp=pp;m.config.param.pole_pairs=pp;
 CHECK(cal_elec_align(&m)==ERR_NONE);CHECK(m.config.param.direction==sign);CHECK(runtime.align_ok&&runtime.electrical_align_done&&!armed&&!fault);
 CHECK(fabsf(angle_normalize(m.state.theta_elec_rad-ALIGN_ELECTRICAL_ANGLE_RAD+_PI)-_PI)<0.001f);
 CHECK(fabsf(angle_normalize(m.config.param.theta_elec_offset_rad-sign*zero*pp+_PI)-_PI)<0.001f);
 }
 reset(&m);stuck=1;CHECK(cal_elec_align(&m)!=ERR_NONE);CHECK(m.config.param.direction==1&&m.config.param.theta_elec_offset_rad==1.23f&&!armed&&!runtime.align_ok);
 reset(&m);physical_pp=7;CHECK(cal_elec_align(&m)!=ERR_NONE);CHECK(m.config.param.theta_elec_offset_rad==1.23f&&!armed);
 reset(&m);cal_elec_align_cfg_default(&cfg);cfg.sample_interval_ms=0;CHECK(cal_elec_align_ex(&m,CAL_ELEC_ALIGN_UD,&cfg)==ERR_PARAM);CHECK(!armed&&m.config.param.theta_elec_offset_rad==1.23f);
 reset(&m);memset(&ctx,0,sizeof(ctx));ctx.m=&m;ctx.rt=&runtime;cal_elec_align_cfg_default(&ctx.cfg);zero=0;jitter=1;
 CHECK(fabsf(angle_normalize(cal_sample_raw_at_lock(&ctx,0,0,0,&spread)+_PI)-_PI)<0.001f);CHECK(spread>=0&&spread<0.002f);
 CHECK(fabsf(angle_normalize(cal_offset_friction_avg(0.01f,_2PI-0.01f)+_PI)-_PI)<0.001f);
 reset(&m);ctx.m=&m;ctx.align_angle=0;ctx.method=CAL_ELEC_ALIGN_UD;zero=0;m.config.param.theta_elec_offset_rad=0.01f;CHECK(cal_theta_matches_expected(&ctx,0.02f));
 puts("PASS: 16 direction/pole combinations, stalled rotor, wrong pole count, invalid period, wrap averaging, signed angle tolerance and rollback");return 0;}
'''
vcvars=Path(r'C:/Program Files (x86)/Microsoft Visual Studio/2022/BuildTools/VC/Auxiliary/Build/vcvars64.bat')
if True:
 w=(a.build_dir or Path(__file__).resolve().parent/'test_build').resolve();w.mkdir(exist_ok=True);(w/'test.c').write_text(stub+'\n'+macros+'\n'+header+'\n'+source+'\n'+tests,encoding='utf-8')
 cmd=['cl','/nologo','/TC','/std:c11','/utf-8','/W3','/WX','test.c','/Fe:test.exe']
 (w/'run.cmd').write_text('\n'.join(['@echo off',f'call "{vcvars}" >nul','if errorlevel 1 exit /b 1',subprocess.list2cmdline(cmd),'exit /b %errorlevel%']))
 subprocess.run(['cmd','/d','/c',str(w/'run.cmd')],cwd=w,check=True)
 if not a.compile_only:subprocess.run([str(w/'test.exe')],cwd=w,check=True)

