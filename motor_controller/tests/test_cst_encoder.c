/* Exercise the public encoder API using two separately addressable SPI devices. */
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "encoder.h"
#include "foc_sense.h"
#include "encoder/kth71xx.h"
#include "motor.h"

static motor_handle_t motor;
static motor_runtime_t runtime;
static uint32_t now_us, irq_mask;
static uint16_t counts[2];
static bool ok[2];
static unsigned reads[2], inits[2];
static bool dual;
float s_inv_gear_ratio = 1.0f;

uint32_t board_time_us(void) { return (uint16_t)now_us; }
static uint32_t cycle_counter;
uint32_t dwt_get_ticks(void) { cycle_counter += 37u; return cycle_counter; }
uint32_t dwt_get_ticks_us(void) { return now_us; }
uint32_t dwt_ticks_to_us(uint32_t ticks) { return ticks / 180u; }
uint32_t dwt_elapsed_us(uint32_t start,uint32_t end) { return (end-start)/180u; }
void board_get_phase_current(motor_handle_t *m) { (void)m; }
void board_get_vbus_and_temp(motor_handle_t *m) { (void)m; }
void current_sense_process_sample(motor_handle_t *m) { (void)m; }
uint32_t __get_PRIMASK(void) { return irq_mask; }
void __disable_irq(void) { irq_mask=1u; }
void __set_PRIMASK(uint32_t mask) { irq_mask=mask; }
motor_runtime_t *motor_runtime(motor_handle_t *m) { return m==&motor ? &runtime : NULL; }
void KTH71_InitCS(kth71_cs_t cs) {
  assert(cs==KTH71_CS_0 || cs==KTH71_CS_1);
  ++inits[cs];
}
unsigned char KTH71_ReadAngleCS(kth71_cs_t cs, uint16_t *raw) {
  assert(cs==KTH71_CS_0 || cs==KTH71_CS_1);
  ++reads[cs];
  *raw = (uint16_t)(counts[cs] << 3);
  return ok[cs];
}

static void reset(void) {
  memset(&motor,0,sizeof(motor)); memset(&runtime,0,sizeof(runtime));
  memset(reads,0,sizeof(reads)); ok[0]=ok[1]=true; now_us=65500u;
  motor.ctrl_mode=CTRL_MODE_CST; motor.fsm=STATE_RUNNING;
  motor.config.param.pole_pairs=5u; motor.config.param.direction=1;
  motor.config.param.gear_ratio=2.0f;
  s_inv_gear_ratio=1.0f/motor.config.param.gear_ratio;
  encoder_init();
  assert(inits[KTH71_CS_0]>0u && (inits[KTH71_CS_1]>0u)==dual);
}
/* Exercise the actual sensing path; manual sampling intentionally omits the speed call. */
static void sample(int shaft, int joint, unsigned dt_us, bool outer, bool automatic) {
  unsigned sr=reads[0], jr=reads[1];
  counts[0]=(uint16_t)(shaft & 8191); counts[1]=(uint16_t)(joint & 8191);
  now_us+=dt_us;
  if (automatic) foc_sense_update_timed(&motor,NULL,outer);
  else encoder_update(&motor,outer);
  assert(reads[0]==sr+1u && reads[1]==jr+(dual && outer));
  assert(motor.state.theta_elec_rad ==
         angle_normalize(motor.state.shaft.angle_rad * 5.0f));
}

static void check_schedule(void) {
  reset();
  unsigned n=4u*FW_FOC_OUTER_LOOP_DIV;
  for (unsigned i=0;i<n;++i) {
    bool outer=i%FW_FOC_OUTER_LOOP_DIV==0u;
    encoder_sample_t prior=motor.state.joint;
    float prior_speed=motor.state.shaft.velocity_rad_s;
    sample(100+2*i,500-i,50,outer,true);
    assert(motor.state.shaft.counts==100u+2*i);
    if (!outer) {
      assert(motor.state.shaft.velocity_rad_s==prior_speed);
      assert(motor.state.joint.position_rad==prior.position_rad);
      assert(motor.state.joint.velocity_rad_s==prior.velocity_rad_s);
    } else {
      if (dual) assert(motor.state.joint.counts==500u-i);
      else assert(motor.state.joint.position_rad==motor.state.shaft.position_rad/2.0f);
      assert(!encoder_estimate_velocity(&motor));
    }
  }
  assert(reads[0]==n && reads[1]==(dual ? 4u : 0u));
  float saved=motor.state.shaft.velocity_rad_s;
  foc_sense_update(&motor); /* Standalone calibration samples the shaft only. */
  assert(reads[0]==n+1 && reads[1]==(dual ? 4u : 0u) && motor.state.shaft.velocity_rad_s==saved);
}

#if !FOC_ENCODER_USE_PLL
static void check_window(void) {
  reset(); assert(!encoder_estimate_velocity(&motor));
  sample(8000,8000,50,true,false);
  assert(encoder_estimate_velocity(&motor) && !encoder_estimate_velocity(&motor));
  unsigned n=2000; /* Multi-turn and more than one 16-bit timer period. */
  for (unsigned i=1;i<=n;++i) {
    sample(8000+10*(int)i,8000-5*(int)i,50,i%FW_FOC_OUTER_LOOP_DIV==0u,false);
    assert(motor.state.joint.velocity_rad_s==0 && motor.state.shaft.velocity_rad_s==0);
  }
  unsigned sr=reads[0], jr=reads[1];
  irq_mask=1u; assert(encoder_estimate_velocity(&motor) && irq_mask==1u);
  irq_mask=0u; assert(!encoder_estimate_velocity(&motor) && !irq_mask);
  assert(reads[0]==sr && reads[1]==jr);
  float tau=1e6f/(_2PI*FOC_MEAS_LPF_FC_HZ);
  float speed=10.0f*RAW_ANGLE_U16_TO_RAD/50e-6f*(100000.0f/(100000.0f+tau));
  assert(fabsf(motor.state.shaft.velocity_rad_s-speed)<0.001f);
  assert(fabsf(motor.state.joint.velocity_rad_s-motor.state.shaft.velocity_rad_s*s_inv_gear_ratio)<0.001f);
}

static void check_crc(void) {
  reset(); unsigned selected=dual ? 1u : 0u;
  sample(100,300,50,true,true);
  ok[selected]=false;
  for (unsigned i=1;i<=FW_FOC_OUTER_LOOP_DIV;++i) {
    sample(100+2*i,300-i,50,i==FW_FOC_OUTER_LOOP_DIV,true);
    if (!dual || i!=FW_FOC_OUTER_LOOP_DIV) assert(motor.state.joint.velocity_rad_s==0.0f);
  }
  assert(!encoder_estimate_velocity(&motor));
  if (dual) {
    assert(motor.state.shaft.velocity_rad_s>0.0f);
    assert(fabsf(motor.state.joint.velocity_rad_s-motor.state.shaft.velocity_rad_s*s_inv_gear_ratio)<0.001f);
  } else {
    assert(motor.state.joint.velocity_rad_s==0.0f);
  }
  ok[selected]=true;
  for (unsigned i=1;i<=FW_FOC_OUTER_LOOP_DIV;++i) {
    unsigned k=i+FW_FOC_OUTER_LOOP_DIV;
    sample(100+2*k,300-k,50,i==FW_FOC_OUTER_LOOP_DIV,true);
  }
  float dt=100.0f*FW_FOC_OUTER_LOOP_DIV;
  float expected=RAW_ANGLE_U16_TO_RAD/50e-6f*
                 dt/(dt+1e6f/(_2PI*FOC_MEAS_LPF_FC_HZ));
  if (dual) assert(fabsf(motor.state.joint.velocity_rad_s-motor.state.shaft.velocity_rad_s*s_inv_gear_ratio)<0.001f);
  else assert(fabsf(motor.state.joint.velocity_rad_s-expected)<0.001f);
}

static void check_shaft_protection_sample(void) {
  reset();
  sample(500,400,50,true,true);
  for (unsigned i=1;i<=12u*FW_FOC_OUTER_LOOP_DIV;++i) {
    float last=motor.state.shaft.velocity_rad_s;
    bool outer=i%FW_FOC_OUTER_LOOP_DIV==0u;
    sample(500+100*i,400,50,outer,true);
    if (!outer) assert(motor.state.shaft.velocity_rad_s==last);
    assert(fabsf(motor.state.joint.velocity_rad_s-motor.state.shaft.velocity_rad_s*s_inv_gear_ratio)<0.001f);
  }
  assert(motor.state.shaft.velocity_rad_s>MAX_VELOCITY_RAD_S);
}

static void check_delayed_estimate_and_reset(void) {
  reset();
  sample(200,500,50,true,false);
  sample(202,499,50,true,false);
  assert(encoder_estimate_velocity(&motor) && !encoder_estimate_velocity(&motor));
  assert(motor.state.joint.velocity_rad_s>0);
  encoder_init(); encoder_shaft_debug_t dbg;
  assert(encoder_get_shaft_debug(&motor,&dbg) && dbg.velocity_rad_s_filt==0);
  ok[0]=ok[1]=false; sample(600,650,50,true,true);
  assert(motor.state.shaft.counts==0 && motor.state.joint.velocity_rad_s==0);
  ok[0]=ok[1]=true; sample(600,650,50,true,true);
  assert(motor.state.joint.velocity_rad_s==0 && !encoder_estimate_velocity(&motor));
}
#else
static void check_pll_rates(void) {
  reset();
  for (unsigned i=0;i<=1500;++i) {
    sample(200+2*(int)i,5000-(int)i,50,i%FW_FOC_OUTER_LOOP_DIV==0,true);
  }
  float shaft_speed=2.0f*RAW_ANGLE_U16_TO_RAD/50e-6f;
  assert(fabsf(motor.state.shaft.velocity_rad_s-shaft_speed)<0.08f*shaft_speed);
  float joint_speed=0.5f*shaft_speed;
  assert(fabsf(motor.state.joint.velocity_rad_s-joint_speed)<0.08f*shaft_speed);
  sample(3201,3499,20000,true,true); /* Re-anchor after a long sampling gap. */
  assert(motor.state.shaft.velocity_rad_s==0 && motor.state.joint.velocity_rad_s==0);
}
#endif

int main(int argc, char **argv) {
  assert(argc==2 && (!strcmp(argv[1],"single") || !strcmp(argv[1],"dual")));
  dual=!strcmp(argv[1],"dual");
  if (FOC_PROFILE_CST_OUTER == 2) {
    foc_sense_marks_t marks = {0};
    reset();
    cycle_counter = UINT32_MAX - 120u; /* Single-axis position timing crosses zero. */
    foc_sense_update_timed(&motor, &marks, true);
    assert(marks.joint_position_ticks == (dual ? 0u : 37u));
    assert(marks.velocity_ticks == 37u);
  }
  check_schedule();
#if FOC_ENCODER_USE_PLL
  check_pll_rates();
#else
  check_window(); check_crc(); check_shaft_protection_sample(); check_delayed_estimate_and_reset();
#endif
  printf("PASS %s, outer divisor %u, PLL %u\n",argv[1],(unsigned)FW_FOC_OUTER_LOOP_DIV,(unsigned)FOC_ENCODER_USE_PLL);
}
