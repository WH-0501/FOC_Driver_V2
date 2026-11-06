#ifndef MOTOR_FRA_H
#define MOTOR_FRA_H
#include <stdint.h>

#define FRA_MAX 16

/* 扫频状态。数值与 CAN 状态字节一致。 */
#define FRA_STATE_IDLE       0u /* 空闲 */
#define FRA_STATE_ACQUIRING  1u /* 电流环正在注入并累加 */
#define FRA_STATE_DONE       2u /* 全部频点完成，可以读取结果 */
#define FRA_STATE_ABORTED    3u /* 已中止，结果不可用 */

/* 中止原因，可按位组合。数值与状态帧 flags 一致。 */
#define FRA_FLAG_CANCEL   1u /* 主机取消，或运行条件已消失 */
#define FRA_FLAG_LIMIT    2u /* 电流、电压或斜率限幅，激励已不再线性 */
#define FRA_FLAG_TIMING   4u /* 电流环间隔或执行时间超出窗口 */
#define FRA_FLAG_CONFLICT 8u /* 扫频期间收到竞争的电流目标 */

/* 分段耗时峰值的下标，与状态帧里 stage_us 的顺序一致。 */
#define FRA_STAGE_SENSE        0u /* foc_sense_update */
#define FRA_STAGE_CONTROL      1u /* foc_control_run */
#define FRA_STAGE_INJECT       2u /* fra_step：生成本拍激励并相关 */
#define FRA_STAGE_ENCODER      3u /* encoder_update */
#define FRA_STAGE_CURRENT_PID  4u /* d/q 电流 PI */
#define FRA_STAGE_VOLTAGE      5u /* 逆 Park 与 PWM 调制 */
#define FRA_STAGE_COUNT        6u

/* fra_timing_reason()：只保留第一次失败的原因。 */
#define FRA_TIMING_NONE      0u
#define FRA_TIMING_INTERVAL  1u /* 相邻两次电流环间隔不在 40..60 us */
#define FRA_TIMING_DURATION  2u /* 单次电流环执行超过 45 us */

/* 主机命令里的扫频方法。 */
#define FRA_METHOD_MULTISINE 1u /* 多音同时注入 */
#define FRA_METHOD_SINE      2u /* 逐个正弦扫过 */

/* fra_start 的返回值，同时作为 CAN 应答错误码。 */
#define FRA_START_OK    0
#define FRA_START_BUSY  1 /* 正在采集，拒绝重入 */
#define FRA_START_PARAM 2

/* fra_result 的返回值。 */
#define FRA_RESULT_OK    0
#define FRA_RESULT_STATE 1 /* 尚未完成，或频点下标无效 */
#define FRA_RESULT_WEAK  2 /* 输入或输出能量过低 */

typedef struct {
  float hz;
  float input;      /* 激励有效值 */
  float output;     /* 反馈有效值 */
  float real;       /* H1 实部，输出相对于输入 */
  float imag;       /* H1 虚部 */
  float coherence;  /* 相干系数，1 表示完全相干 */
  uint16_t blocks;  /* 参与平均的整周期块数 */
} fra_result_t;

/* 准备频点并进入采集。low/high 单位 Hz，peak 为电流激励峰值，fs 为电流环采样率。 */
int fra_start(unsigned method, unsigned count, float low, float high,
              float peak, float fs, uint16_t id);
/* 用本拍反馈推进相关，返回应写入 Iq 的激励。非采集状态返回 0。 */
float fra_step(float feedback);
void fra_abort(unsigned flags);

unsigned fra_state(void);
unsigned fra_flags(void);
unsigned fra_count(void);
unsigned fra_progress(void);
unsigned fra_method(void);
uint16_t fra_id(void);
uint32_t fra_samples(void);
float fra_fs(void);

/* owned：扫频仍占用 CST 电流指令，采集结束并卸力后由 fra_release 交还。 */
unsigned fra_owned(void);
void fra_release(void);

/* 记录单拍电流环耗时。超过 45 us 时以 FRA_FLAG_TIMING 中止。 */
void fra_timing(unsigned duration_us);
unsigned fra_max_us(void);
/* 记录相邻两拍的间隔。离开 40..60 us 时中止，并记下第一次失败的样本号。 */
void fra_interval(unsigned interval_us);
unsigned fra_min_interval(void);
unsigned fra_max_interval(void);
unsigned fra_last_interval(void);
unsigned fra_timing_reason(void);
uint32_t fra_failure_sample(void);

/* 记录某个 FRA_STAGE_* 的耗时峰值，单位 us。 */
void fra_stage(unsigned stage, unsigned duration_us);
unsigned fra_stage_us(unsigned stage);
/* 扫频开始前，最近一次正常 CST 周期的耗时和间隔。 */
void fra_baseline(unsigned duration_us, unsigned interval_us);
/* 在 fra_timing 之前调用：本拍耗时创新高时，记下对应的 DWT 周期数。 */
void fra_cycle_measure(uint32_t cycles, unsigned duration_us);
unsigned fra_baseline_duration(void);
unsigned fra_baseline_interval(void);
uint32_t fra_peak_cycles(void);

int fra_result(unsigned index, fra_result_t *out);
#endif
