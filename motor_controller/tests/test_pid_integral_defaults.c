#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "motor_config.h"
#include "foc_pid.h"
#include "common.h"

static uint32_t now_us = 1000;
uint32_t dwt_get_ticks_us(void) { return now_us; }
static unsigned char blob[4096];
static uint32_t blob_len;
static bool read_blob(void *ctx,uint8_t *data,uint32_t len) {
  (void)ctx; if(len!=blob_len) return false; memcpy(data,blob,len); return true;
}
static bool write_blob(void *ctx,const uint8_t *data,uint32_t len) {
  (void)ctx; assert(len<=sizeof blob); memcpy(blob,data,len); blob_len=len; return true;
}
int main(void) {
  motor_config_t cfg,loaded;
  pid_state_t state;
  pid_param_t param;
  float out=0,previous=0;
  unsigned i;
  motor_config_storage_ops_t storage={read_blob,write_blob,NULL,NULL};
  motor_config_set_defaults(&cfg);
  assert(cfg.pid.velocity.ki>0 && cfg.pid.velocity.i_limit>0);
  assert(fabsf(cfg.pid.velocity.i_limit-cfg.limits.iq_limit*DEF_I_LIMIT_FACTOR)<1e-6f);
  assert(cfg.pid.position.i_limit>0 && cfg.pid.id.i_limit>0 && cfg.pid.iq.i_limit>0);
  assert(foc_pid_init(&state,1)==ERR_NONE);
  for(i=0;i<100;++i) {
    now_us+=1000;
    assert(foc_pid_calc(&state,&cfg.pid.velocity,1,0,&out)==ERR_NONE);
    assert(state.last_integral>previous);
    previous=state.last_integral;
  }
  assert(fabsf(state.last_integral-cfg.pid.velocity.ki*.1f)<1e-6f);
  assert(out>cfg.pid.velocity.kp);
  /* A zero error retains the learned holding-current contribution. */
  now_us+=1000; assert(foc_pid_calc(&state,&cfg.pid.velocity,1,1,&out)==ERR_NONE);
  assert(fabsf(out-previous)<1e-6f);

  param=cfg.pid.velocity; param.i_limit=0; state.last_integral=1;
  now_us+=1000; assert(foc_pid_calc(&state,&param,1,0,&out)==ERR_NONE);
  assert(state.last_integral==0 && fabsf(out-param.kp)<1e-7f);
  /* Integral budget constrains this sample's output as well as saved state. */
  param.kp=param.kd=param.ramp=0; param.ki=100; param.limit=1; param.i_limit=.05f;
  foc_pid_init(&state,1);
  for(i=0;i<100;++i) {
    now_us+=1000; assert(foc_pid_calc(&state,&param,1,0,&out)==ERR_NONE);
    assert(fabsf(out)<=.050001f && fabsf(state.last_integral)<=.050001f);
  }
  for(i=0;i<100;++i) {
    now_us+=1000; assert(foc_pid_calc(&state,&param,-1,0,&out)==ERR_NONE);
    assert(fabsf(out)<=.050001f && fabsf(state.last_integral)<=.050001f);
  }
  param.i_limit=-1; assert(foc_pid_calc(&state,&param,1,0,&out)==ERR_PARAM);
  param.i_limit=NAN; assert(foc_pid_calc(&state,&param,1,0,&out)==ERR_PARAM);
  param.i_limit=INFINITY; assert(foc_pid_calc(&state,&param,1,0,&out)==ERR_PARAM);
  /* Persisted zero is explicit disable, not silently overwritten on loading. */
  motor_config_clear_storage(); assert(motor_config_register_storage(&storage));
  cfg.pid.velocity.i_limit=0; assert(motor_config_save(&cfg)); assert(motor_config_load(&loaded));
  assert(loaded.pid.velocity.i_limit==0 && loaded.pid.velocity.ki==cfg.pid.velocity.ki);
  assert(motor_config_reset(&loaded) && loaded.pid.velocity.i_limit>0);
  puts("PASS C03: default integral accumulation/holding output, zero-disable, signed bounds, invalid budgets, stored-config preservation/reset");
  return 0;
}
