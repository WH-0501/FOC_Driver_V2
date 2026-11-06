/* 电流环内的频率响应扫频，每 50 us 执行一次。
 * 调试构建也保持优化，避免扫频路径和发布版本的时序不一致。
 * 三角运算只在 fra_start 里做；ISR 中只旋转相位并累加相关。 */
#if defined(__CC_ARM)
#include "at32m412_416.h"
#pragma push
#pragma O2
#define FRA_BARRIER() __schedule_barrier()
#elif defined(__GNUC__)
#define FRA_BARRIER() __asm__ volatile("" ::: "memory")
#else
#define FRA_BARRIER() ((void)0)
#endif
#include "fra.h"
#include <math.h>
#include <string.h>

#define FRA_PI 3.14159265358979323846f
/* 电流环标称 50 us。间隔离开这个窗口，或单拍执行过长，都视为时序失败。 */
#define FRA_INTERVAL_MIN_US 40u
#define FRA_INTERVAL_MAX_US 60u
#define FRA_DURATION_MAX_US 45u
#define FRA_MULTISINE_BLOCKS 3u
#define FRA_SINE_BLOCKS 4u

/* 一个激励音：旋转相量，以及输入/输出的同相、正交相关。 */
typedef struct {
  float hz;
  float sin_phase;     /* 当前相位的 sin / cos */
  float cos_phase;
  float sin_delta;     /* 每拍旋转一步的 sin / cos */
  float cos_delta;
  float sin_origin;    /* 每个相关块开始时要回到的相位 */
  float cos_origin;
  float in_real;       /* 本块激励的同相 / 正交累加 */
  float in_imag;
  float out_real;      /* 本块反馈的同相 / 正交累加 */
  float out_imag;
  float in_power;      /* 多块平均后的激励功率、反馈功率 */
  float out_power;
  float cross_real;    /* 多块平均后的互谱 */
  float cross_imag;
  unsigned blocks;
  unsigned length;     /* 步进正弦：一个相关块的样本数 */
  unsigned settle;     /* 步进正弦：开始相关前丢弃的样本数 */
} fra_tone_t;

/* 电流环 ISR 与前台命令共用。state / flags / owned 跨上下文读写。 */
typedef struct {
  volatile unsigned state;
  volatile unsigned flags;
  volatile unsigned owned;

  unsigned max_us;
  unsigned min_interval;
  unsigned max_interval;
  unsigned last_interval;
  unsigned timing_reason;
  uint32_t failure_sample;
  uint32_t peak_cycles;
  unsigned baseline_duration;
  unsigned baseline_interval;
  unsigned stage_us[FRA_STAGE_COUNT];

  unsigned method;
  unsigned count;
  unsigned tone_index;
  unsigned sample_index;
  unsigned length;
  unsigned settle;
  unsigned blocks;
  unsigned progress;
  uint16_t id;
  uint32_t samples;
  float fs;
  float scale;
  fra_tone_t tone[FRA_MAX];
} fra_context_t;

static fra_context_t s_fra;

static void fra_reset_phase(fra_tone_t *tone)
{
  tone->sin_phase = tone->sin_origin;
  tone->cos_phase = tone->cos_origin;
}

/* (cos + j sin) 乘以每拍旋转量，避免在 ISR 里调用 sin/cos。 */
static void fra_rotate(fra_tone_t *tone)
{
  float sin_phase = tone->sin_phase * tone->cos_delta + tone->cos_phase * tone->sin_delta;

  tone->cos_phase = tone->cos_phase * tone->cos_delta - tone->sin_phase * tone->sin_delta;
  tone->sin_phase = sin_phase;
}

/* 步进正弦进入下一个频点。相位和块长度都在采集前准备好。 */
static void fra_setup_sine(void)
{
  fra_tone_t *tone = &s_fra.tone[s_fra.tone_index];

  fra_reset_phase(tone);
  s_fra.length = tone->length;
  s_fra.settle = tone->settle;
  s_fra.sample_index = 0;
  s_fra.blocks = 0;
}

int fra_start(unsigned method, unsigned count, float low, float high,
              float peak, float fs, uint16_t id)
{
  unsigned i;
  unsigned j;
  float last = 0.0f;
  float maxsum = 0.0f;

  if (s_fra.state == FRA_STATE_ACQUIRING)
  {
    return FRA_START_BUSY;
  }
  if ((method != FRA_METHOD_MULTISINE && method != FRA_METHOD_SINE) || count < 3 || count > FRA_MAX ||
      !isfinite(low) || !isfinite(high) || !isfinite(peak) || !isfinite(fs) ||
      low < 20 || high <= low || high > 2000 || high > fs / 10 ||
      peak < .005f || peak > .5f || fs < 10000 || fs > 40000)
  {
    return FRA_START_PARAM;
  }

  memset(&s_fra, 0, sizeof(s_fra));
  s_fra.method = method;
  s_fra.count = count;
  s_fra.id = id;
  s_fra.fs = fs;
  s_fra.scale = peak;
  /* 多音相关块约 50 ms，丢弃同样长度后再开始累加。 */
  s_fra.length = (unsigned)(fs * .05f + .5f);
  s_fra.settle = s_fra.length;

  for (i = 0; i < count; i++)
  {
    fra_tone_t *tone = &s_fra.tone[i];
    float freq;

    /* 收窄扫频至少要留出 20 Hz、40 Hz 和第一个目标频点。 */
    if (low > 40 && count < 4)
    {
      return FRA_START_PARAM;
    }
    /* 收窄频段时仍固定保留 20 Hz 和 40 Hz，便于和全频段结果对比。 */
    if (low > 40 && i < 2)
    {
      freq = (i == 0) ? 20.0f : 40.0f;
    }
    else if (low > 40)
    {
      freq = low * powf(high / low, (float)(i - 2) / (count - 3));
    }
    else
    {
      freq = low * powf(high / low, (float)i / (count - 1));
    }

    if (method == FRA_METHOD_MULTISINE)
    {
      /* 对齐到相关块的频率格子，保证一个块内相位闭合。 */
      freq = (float)((unsigned)(freq / (fs / s_fra.length) + .5f)) * (fs / s_fra.length);
    }
    else
    {
      freq = fs / ceilf(fs / freq);
    }
    if ((method == FRA_METHOD_MULTISINE) && (freq <= last))
    {
      freq = last + fs / s_fra.length;
    }
    if (freq <= last || freq > high + .01f)
    {
      s_fra.state = FRA_STATE_IDLE;
      return FRA_START_PARAM;
    }

    last = freq;
    tone->hz = freq;
    tone->sin_delta = sinf(2 * FRA_PI * freq / fs);
    tone->cos_delta = cosf(2 * FRA_PI * freq / fs);
    /* Schroeder 相位，压低多音叠加后的峰值。 */
    tone->sin_origin = sinf(-FRA_PI * i * (i - 1.0f) / count);
    tone->cos_origin = cosf(-FRA_PI * i * (i - 1.0f) / count);
    if (method == FRA_METHOD_SINE)
    {
      unsigned period = (unsigned)(fs / freq + .5f);
      unsigned settling = (unsigned)(fs * .02f);

      tone->hz = fs / period;
      tone->sin_delta = sinf(2 * FRA_PI / period);
      tone->cos_delta = cosf(2 * FRA_PI / period);
      tone->sin_origin = 0.0f;
      tone->cos_origin = 1.0f;
      tone->length = period * 4u;
      if (settling < tone->length)
      {
        settling = tone->length;
      }
      /* 稳定时间至少 20 ms，并且取整到周期，避免相关窗切开波形。 */
      tone->settle = ((settling + period - 1) / period) * period;
    }
    fra_reset_phase(tone);
  }

  if (method == FRA_METHOD_MULTISINE)
  {
    /* 先量出整段周期波形的峰值，再把缩放定到请求的电流峰值以内。 */
    for (j = 0; j < s_fra.length; j++)
    {
      float sum = 0.0f;

      for (i = 0; i < count; i++)
      {
        sum += s_fra.tone[i].sin_phase;
        fra_rotate(&s_fra.tone[i]);
      }
      if (fabsf(sum) > maxsum)
      {
        maxsum = fabsf(sum);
      }
    }
    s_fra.scale = peak / (maxsum * 1.001f);
    for (i = 0; i < count; i++)
    {
      fra_reset_phase(&s_fra.tone[i]);
    }
  }
  else
  {
    fra_setup_sine();
  }

  {
#if defined(__CC_ARM)
    uint32_t irq = __get_PRIMASK();
    __disable_irq();
#endif
    /* 先标记占用，再发布采集状态，避免前台看到正在采集但尚未占用。 */
    s_fra.owned = 1u;
    FRA_BARRIER();
    s_fra.state = FRA_STATE_ACQUIRING;
#if defined(__CC_ARM)
    __set_PRIMASK(irq);
#endif
  }
  return FRA_START_OK;
}

float fra_step(float feedback)
{
  unsigned i;
  unsigned begin;
  unsigned end;
  float excitation = 0.0f;

  if (s_fra.state != FRA_STATE_ACQUIRING)
  {
    return 0.0f;
  }
  if (!isfinite(feedback))
  {
    fra_abort(FRA_FLAG_LIMIT);
    return 0.0f;
  }

  /* 多音同时相关；步进正弦只处理当前频点。 */
  begin = (s_fra.method == FRA_METHOD_MULTISINE) ? 0u : s_fra.tone_index;
  end = (s_fra.method == FRA_METHOD_MULTISINE) ? s_fra.count : s_fra.tone_index + 1u;
  for (i = begin; i < end; i++)
  {
    excitation += s_fra.tone[i].sin_phase * s_fra.scale;
  }
  for (i = begin; i < end; i++)
  {
    fra_tone_t *tone = &s_fra.tone[i];

    if (s_fra.sample_index >= s_fra.settle)
    {
      tone->in_real += excitation * tone->cos_phase;
      tone->in_imag -= excitation * tone->sin_phase;
      tone->out_real += feedback * tone->cos_phase;
      tone->out_imag -= feedback * tone->sin_phase;
    }
    fra_rotate(tone);
  }

  s_fra.sample_index++;
  s_fra.samples++;
  if ((s_fra.sample_index >= s_fra.settle) &&
      ((s_fra.sample_index - s_fra.settle) % s_fra.length == 0))
  {
    /* 稳定段结束的那个边界只负责对齐相位，不把它算成一个相关块。 */
    if (s_fra.sample_index > s_fra.settle)
    {
      for (i = begin; i < end; i++)
      {
        fra_tone_t *tone = &s_fra.tone[i];
        float norm = 2.0f / s_fra.length;
        float in_real = tone->in_real * norm;
        float in_imag = tone->in_imag * norm;
        float out_real = tone->out_real * norm;
        float out_imag = tone->out_imag * norm;

        tone->in_power += in_real * in_real + in_imag * in_imag;
        tone->out_power += out_real * out_real + out_imag * out_imag;
        tone->cross_real += out_real * in_real + out_imag * in_imag;
        tone->cross_imag += out_imag * in_real - out_real * in_imag;
        tone->blocks++;
        tone->in_real = 0.0f;
        tone->in_imag = 0.0f;
        tone->out_real = 0.0f;
        tone->out_imag = 0.0f;
      }
      s_fra.blocks++;
    }
    for (i = begin; i < end; i++)
    {
      fra_reset_phase(&s_fra.tone[i]);
    }
    if (s_fra.blocks >= ((s_fra.method == FRA_METHOD_MULTISINE) ? FRA_MULTISINE_BLOCKS : FRA_SINE_BLOCKS))
    {
      if ((s_fra.method == FRA_METHOD_MULTISINE) || (++s_fra.tone_index == s_fra.count))
      {
        s_fra.progress = s_fra.count;
        FRA_BARRIER();
        s_fra.state = FRA_STATE_DONE;
      }
      else
      {
        s_fra.progress = s_fra.tone_index;
        fra_setup_sine();
      }
    }
  }
  return excitation;
}

void fra_abort(unsigned flags)
{
  /* 采集中可以直接中止。完成后纯取消保留结果；限幅或时序失败仍把结果降为中止。 */
  if ((s_fra.state == FRA_STATE_ACQUIRING) ||
      (s_fra.owned && (s_fra.state == FRA_STATE_DONE) && (flags & ~FRA_FLAG_CANCEL)))
  {
    s_fra.flags |= flags;
    s_fra.state = FRA_STATE_ABORTED;
  }
}

unsigned fra_state(void) { return s_fra.state; }
unsigned fra_flags(void) { return s_fra.flags; }
unsigned fra_count(void) { return s_fra.count; }
unsigned fra_progress(void) { return s_fra.progress; }
unsigned fra_method(void) { return s_fra.method; }
uint16_t fra_id(void) { return s_fra.id; }
uint32_t fra_samples(void) { return s_fra.samples; }
float fra_fs(void) { return s_fra.fs; }

int fra_result(unsigned index, fra_result_t *result)
{
  fra_tone_t *tone;

  if ((s_fra.state != FRA_STATE_DONE) || (index >= s_fra.count) || (result == NULL))
  {
    return FRA_RESULT_STATE;
  }

  tone = &s_fra.tone[index];
  memset(result, 0, sizeof(*result));
  result->hz = tone->hz;
  result->blocks = (uint16_t)tone->blocks;
  if ((tone->in_power < 1e-15f) || (tone->out_power < 1e-15f) || !tone->blocks)
  {
    return FRA_RESULT_WEAK;
  }

  result->input = sqrtf(tone->in_power / tone->blocks);
  result->output = sqrtf(tone->out_power / tone->blocks);
  result->real = tone->cross_real / tone->in_power;
  result->imag = tone->cross_imag / tone->in_power;
  result->coherence = (tone->cross_real * tone->cross_real + tone->cross_imag * tone->cross_imag) /
                      (tone->in_power * tone->out_power);
  if (result->coherence > 1.0f)
  {
    result->coherence = 1.0f;
  }
  return FRA_RESULT_OK;
}

unsigned fra_owned(void) { return s_fra.owned; }
void fra_release(void) { s_fra.owned = 0u; }

void fra_baseline(unsigned duration_us, unsigned interval_us)
{
  s_fra.baseline_duration = duration_us;
  s_fra.baseline_interval = interval_us;
}

void fra_cycle_measure(uint32_t cycles, unsigned duration_us)
{
  if (duration_us >= s_fra.max_us)
  {
    s_fra.peak_cycles = cycles;
  }
}

unsigned fra_baseline_duration(void) { return s_fra.baseline_duration; }
unsigned fra_baseline_interval(void) { return s_fra.baseline_interval; }
uint32_t fra_peak_cycles(void) { return s_fra.peak_cycles; }
unsigned fra_max_us(void) { return s_fra.max_us; }

void fra_timing(unsigned duration_us)
{
  if (duration_us > s_fra.max_us)
  {
    s_fra.max_us = duration_us;
  }
  if (duration_us > FRA_DURATION_MAX_US)
  {
    if (!s_fra.timing_reason)
    {
      s_fra.timing_reason = FRA_TIMING_DURATION;
      s_fra.failure_sample = s_fra.samples;
    }
    /* 直接写下超时，已经因其他原因中止时也要带上时序标志。 */
    s_fra.flags |= FRA_FLAG_TIMING;
    s_fra.state = FRA_STATE_ABORTED;
  }
}

void fra_interval(unsigned interval_us)
{
  s_fra.last_interval = interval_us;
  if (!s_fra.min_interval || (interval_us < s_fra.min_interval))
  {
    s_fra.min_interval = interval_us;
  }
  if (interval_us > s_fra.max_interval)
  {
    s_fra.max_interval = interval_us;
  }
  if ((interval_us < FRA_INTERVAL_MIN_US) || (interval_us > FRA_INTERVAL_MAX_US))
  {
    if (!s_fra.timing_reason)
    {
      s_fra.timing_reason = FRA_TIMING_INTERVAL;
      s_fra.failure_sample = s_fra.samples;
    }
    fra_abort(FRA_FLAG_TIMING);
  }
}

unsigned fra_min_interval(void) { return s_fra.min_interval; }
unsigned fra_max_interval(void) { return s_fra.max_interval; }
unsigned fra_last_interval(void) { return s_fra.last_interval; }
unsigned fra_timing_reason(void) { return s_fra.timing_reason; }
uint32_t fra_failure_sample(void) { return s_fra.failure_sample; }

#if defined(__CC_ARM)
#pragma pop
#endif

void fra_stage(unsigned stage, unsigned duration_us)
{
  if ((stage < FRA_STAGE_COUNT) && (duration_us > s_fra.stage_us[stage]))
  {
    s_fra.stage_us[stage] = duration_us;
  }
}

unsigned fra_stage_us(unsigned stage)
{
  return (stage < FRA_STAGE_COUNT) ? s_fra.stage_us[stage] : 0u;
}
