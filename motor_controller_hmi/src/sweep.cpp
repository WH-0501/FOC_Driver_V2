#include "sweep_policy.h"
#include "sweep.h"
#include <algorithm>
#include <array>
#include <cmath>
#include <numbers>
namespace mt {
namespace {
struct Fit {
  double amplitude = 0, phase = 0, quality = 0;
  bool ok = false;
};
Fit fit(const std::vector<SweepSample> &s, double hz, bool output) {
  double a[3][4]{};
  double mean = 0;
  for (auto &p : s) {
    double angle = 2 * std::numbers::pi * hz * (p.time - s.front().time);
    double x[3] = {std::sin(angle), std::cos(angle), 1};
    double y = output ? p.output : p.input;
    mean += y;
    for (int i = 0; i < 3; ++i) {
      for (int j = 0; j < 3; ++j)
        a[i][j] += x[i] * x[j];
      a[i][3] += x[i] * y;
    }
  }
  for (int k = 0; k < 3; ++k) {
    int pivot = k;
    for (int i = k + 1; i < 3; ++i)
      if (std::abs(a[i][k]) > std::abs(a[pivot][k]))
        pivot = i;
    if (std::abs(a[pivot][k]) < 1e-10)
      return {};
    for (int j = 0; j < 4; ++j)
      std::swap(a[k][j], a[pivot][j]);
    double v = a[k][k];
    for (int j = k; j < 4; ++j)
      a[k][j] /= v;
    for (int i = 0; i < 3; ++i)
      if (i != k) {
        double f = a[i][k];
        for (int j = k; j < 4; ++j)
          a[i][j] -= f * a[k][j];
      }
  }
  mean /= s.size();
  double residual = 0, total = 0;
  for (auto &p : s) {
    double angle = 2 * std::numbers::pi * hz * (p.time - s.front().time);
    double y = output ? p.output : p.input;
    residual += std::pow(
        y - (a[0][3] * std::sin(angle) + a[1][3] * std::cos(angle) + a[2][3]),
        2);
    total += std::pow(y - mean, 2);
  }
  return {std::hypot(a[0][3], a[1][3]), std::atan2(a[1][3], a[0][3]),
          total > 1e-18 ? std::max(0., 1 - residual / total) : 0, true};
}
} // namespace
SweepPoint measureSweep(double hz, const std::vector<SweepSample> &s) {
  SweepPoint p;
  p.hz = hz;
  p.count = int(s.size());
  auto reject = [&](QString message) {
    p.status = message;
    return p;
  };
  if (!std::isfinite(hz) || hz <= 0)
    return reject("频率无效");
  if (s.size() < 32)
    return reject("采样不足 32 点");
  double maxGap = 0;
  int distinct = 1;
  for (size_t i = 0; i < s.size(); ++i) {
    if (!std::isfinite(s[i].time) || !std::isfinite(s[i].input) ||
        !std::isfinite(s[i].output))
      return reject("非有限采样");
    if (i) {
      double dt = s[i].time - s[i - 1].time;
      if (dt < 0)
        return reject("时钟回退");
      if (dt > 0)
        ++distinct;
      maxGap = std::max(maxGap, dt);
    }
  }
  double cycles = (s.back().time - s.front().time) * hz;
  if (cycles < 2.8)
    return reject("稳态观察不足 3 周期");
  if (distinct / cycles < 10 || maxGap * hz > .2)
    return reject("采样率不足或时间缺口过大");
  auto x = fit(s, hz, false), y = fit(s, hz, true);
  if (!x.ok || !y.ok || x.amplitude < 1e-6 || y.amplitude < 1e-12)
    return reject("输入/响应幅值不足");
  p.input = x.amplitude;
  p.output = y.amplitude;
  p.inputFit = x.quality;
  p.outputFit = y.quality;
  p.gainDb = 20 * std::log10(y.amplitude / x.amplitude);
  p.phaseDeg =
      std::remainder((y.phase - x.phase) * 180 / std::numbers::pi, 360.);
  if (x.quality < .9)
    return reject("输入给定失真：正弦拟合质量不足");
  // R² includes broadband measurement noise, not just uncertainty of the
  // synchronous fundamental. Validate repeatability over complete periods.
  std::vector<double> gains, phases;
  for (int cycle = 0; cycle < int(std::floor(cycles)); ++cycle) {
    std::vector<SweepSample> block;
    double begin = s.front().time + cycle / hz, end = begin + 1 / hz;
    for (auto &v : s) if (v.time >= begin && v.time < end) block.push_back(v);
    if (block.size() < 10) continue;
    auto bx = fit(block, hz, false), by = fit(block, hz, true);
    if (!bx.ok || !by.ok || bx.amplitude < 1e-6) continue;
    gains.push_back(by.amplitude / bx.amplitude);
    phases.push_back(std::remainder((by.phase-bx.phase)*180/std::numbers::pi-p.phaseDeg,360.));
  }
  if (gains.size() < 3) return reject("至少需要 3 个完整周期验证重复性");
  double variation=0, phaseVariation=0, gain=y.amplitude/x.amplitude;
  for (size_t i=0;i<gains.size();++i) {
    variation += std::pow(gains[i]/gain-1,2);
    phaseVariation += phases[i]*phases[i];
  }
  variation=std::sqrt(variation/gains.size());
  phaseVariation=std::sqrt(phaseVariation/gains.size());
  if (variation>.15 || phaseVariation>10)
    return reject(QString("周期间不稳定：幅值偏差 %1% / 相位偏差 %2°").arg(variation*100,0,'f',1).arg(phaseVariation,0,'f',1));
  // Coherent harmonics must not be mistaken for harmless broadband noise.
  double harmonicPower=0;
  for (int harmonic=2;harmonic<=7;++harmonic) {
    if (distinct/cycles < harmonic*4) break;
    auto h=fit(s,hz*harmonic,true);
    if(h.ok) harmonicPower+=h.amplitude*h.amplitude;
  }
  if (std::sqrt(harmonicPower)/y.amplitude > .25)
    return reject("反馈谐波过大：疑似非线性/限幅，不能按线性带宽解释");
  p.valid = true;
  p.status = y.quality < .8
      ? QString("有效·噪声较大；周期幅值偏差 %1% / 相位 %2°").arg(variation*100,0,'f',1).arg(phaseVariation,0,'f',1)
      : "有效";
  return p;
}
QString sweepAbortMessage(const QJsonObject &o) {
  const int flags=o["flags"].toInt(), reason=o["motion_reason"].toInt();
  QString detail;
  if(flags&0x10) {
    if(reason==18 || reason==19) {
      const double value=o["motion_value"].toDouble(), limit=o["motion_limit"].toDouble();
      if(reason==18)
        detail=QString("电机轴位移越界：相对起点 %1 rad（%2°），绝对值上限 %3 rad（%4°）")
          .arg(value,0,'g',7).arg(value*180/std::numbers::pi,0,'f',2)
          .arg(limit,0,'g',7).arg(limit*180/std::numbers::pi,0,'f',2);
      else
        detail=QString("电机轴速度越界：实际 %1 rad/s（%2 RPM），绝对值上限 %3 rad/s（%4 RPM）")
          .arg(value,0,'g',7).arg(value*60/(2*std::numbers::pi),0,'f',2)
          .arg(limit,0,'g',7).arg(limit*60/(2*std::numbers::pi),0,'f',2);
      if(o["method"].toInt()==2)
        detail+=QString(" · 触发频率 %1 Hz（第 %2/%3 点）").arg(o["motion_hz"].toDouble(),0,'g',7)
          .arg(o["motion_index"].toInt()+1).arg(o["count"].toInt());
      else if(o["method"].toInt()==3) detail+=QString(" · Chirp 瞬时激励频率 %1 Hz").arg(o["motion_hz"].toDouble(),0,'g',7);
      else detail+=" · 多正弦并行激励（无单一触发频率）";
    } else detail=o["motion_diagnostic_invalid"].toBool()
        ? "运动保护：诊断扩展数据无效，无法区分速度或位移越界"
        : "运动保护：速度或位移越界（固件未提供详细诊断）";
  } else if(flags&4) {
    detail=o["timing_reason"].toInt()==1 ? "采样周期间隔异常"
        : o["timing_reason"].toInt()==2 ? "FOC 执行耗时超限" : "采样时序或执行耗时异常";
    if(o.contains("failure_sample"))detail+=QString(" · 时序异常采样计数 %1").arg(o["failure_sample"].toDouble(),0,'f',0);
  } else if(flags&2) detail="电流、电压或温度限制介入";
  else if(flags&8) detail="外部控制命令中止扫频";
  else if(flags&32) detail="反馈出现非有限值";
  else if(flags&64) detail="录波队列丢样，已中止 Chirp 采集；需检查上传与主循环调度";
  else detail="停止或运行条件变化";
  return QString("设备中止：%1 · flags=0x%2 · 中止前已执行 %3 个采样周期")
      .arg(detail).arg(flags,0,16).arg(o["samples"].toDouble(),0,'f',0);
}
SweepBandwidthResult sweepBandwidthResult(const std::vector<SweepPoint> &p,bool localRetest) {
  SweepBandwidthResult result;
  auto reject=[&](QString message){result.message=message;return result;};
  if(localRetest)return reject("局部复测：仅显示原始 H1，未建立本次低频基准，不判定绝对带宽");
  if(p.size()<sweepPolicy::baselineMinPoints+1)return reject(QString("未测得带宽：至少需要 %1 个低频平台点及后续响应点").arg(sweepPolicy::baselineMinPoints));
  for(size_t i=0;i<p.size();++i)
    if(!std::isfinite(p[i].hz)||p[i].hz<=0||(i&&p[i].hz<=p[i-1].hz))
      return reject("未测得带宽：频点必须为有限递增正数");
  // Unit-gain current tracking only. These are baseline qualification limits,
  // not filters or relaxed H1 validity gates; never skip an invalid first point.
  using namespace sweepPolicy;
  auto valid=[](const SweepPoint &v){return v.valid&&std::isfinite(v.gainDb)&&std::isfinite(v.phaseDeg)&&std::isfinite(v.inputFit)&&v.inputFit>=.9;};
  std::vector<double> gains;double lo=0,hi=0;
  for(size_t i=0;i<p.size()&&p[i].hz<=p[0].hz*baselineMaxRatio;++i){
    const auto &v=p[i];
    if(!valid(v)||std::abs(std::remainder(v.phaseDeg,360.))>phaseLimitDeg||
       std::abs(v.gainDb)>unityToleranceDb||(i&&v.hz/p[i-1].hz>maxAdjacentRatio))break;
    double a=i?std::min(lo,v.gainDb):v.gainDb,b=i?std::max(hi,v.gainDb):v.gainDb;
    if(b-a>flatDb)break;
    gains.push_back(v.gainDb);lo=a;hi=b;
  }
  if(gains.size()<baselineMinPoints||p[gains.size()-1].hz/p[0].hz<baselineMinRatio)
    return reject(QString("未测得带宽：低频平台不足（需连续有效 ≥%1 点、跨度 ≥%2 倍、波动 ≤%3 dB、相位 ≤%4°、增益接近 0 dB）；请核对低频覆盖及平台有效性，或选择局部复测")
      .arg(baselineMinPoints).arg(baselineMinRatio).arg(flatDb).arg(phaseLimitDeg));
  result.baselineCount=unsigned(gains.size());result.baselineLowHz=p[0].hz;result.baselineHighHz=p[gains.size()-1].hz;
  std::sort(gains.begin(),gains.end());size_t mid=gains.size()/2;
  result.baselineDb=gains.size()%2?gains[mid]:(gains[mid-1]+gains[mid])/2;
  result.baselineValid=true;double threshold=result.baselineDb-3;
  QString base=QString("低频平台 [%1, %2] Hz / %3 点，中位数 %4 dB")
    .arg(result.baselineLowHz,0,'g',5).arg(result.baselineHighHz,0,'g',5).arg(result.baselineCount).arg(result.baselineDb,0,'f',2);
  for(size_t i=result.baselineCount;i<p.size();++i){
    if(!valid(p[i-1])||!valid(p[i]))return reject("未测得带宽：存在无效频点，需补测后确定首次交点 · "+base);
    if(p[i].hz/p[i-1].hz>maxAdjacentRatio)return reject("未测得带宽：频点间隔过大，需补测，不能跨未测区确认首次交点 · "+base);
    if(p[i-1].gainDb>threshold&&p[i].gainDb<=threshold){
      double alpha=(threshold-p[i-1].gainDb)/(p[i].gainDb-p[i-1].gainDb);
      result.hz=std::exp(std::log(p[i-1].hz)+alpha*std::log(p[i].hz/p[i-1].hz));
      result.found=true;result.lowHz=p[i-1].hz;result.highHz=p[i].hz;
      result.message=QString("−3 dB 带宽 ≈ %1 Hz · 交点区间 [%2, %3] Hz · ").arg(result.hz,0,'g',6).arg(result.lowHz,0,'g',5).arg(result.highHz,0,'g',5)+base;
      return result;
    }
  }
  return reject("未测得带宽：当前频段未出现相对低频平台的 −3 dB 交点 · "+base);
}
QString sweepBandwidth(const std::vector<SweepPoint> &points,bool localRetest) {return sweepBandwidthResult(points,localRetest).message;}
SweepCrossingResult sweepClosedLoopCrossingResult(const std::vector<SweepPoint> &p) {
  SweepCrossingResult result;
  result.message="fc：无有效 ≥100 Hz 闭环 0 dB 向下交点";
  for(size_t i=0;i<p.size();++i)
    if(!std::isfinite(p[i].hz)||p[i].hz<=0||(i&&p[i].hz<=p[i-1].hz)) {
      result.message="fc：频率无效或未递增";return result;
    }
  bool earlierGap=false;
  for(size_t i=1;i<p.size();++i) {
    const auto &a=p[i-1],&b=p[i];
    if(b.hz<100)continue;
    // Preserve the low-frequency-anchor gap guard, without rejecting ordinary
    // brackets that straddle 100 Hz (e.g. 93 -> 126 Hz).
    if((a.hz<=40&&b.hz>100)||!a.valid||!b.valid||
       !std::isfinite(a.gainDb)||!std::isfinite(b.gainDb)) {
      earlierGap=true;continue;
    }
    if(a.gainDb>=0&&b.gainDb<=0&&(a.gainDb>0||b.gainDb<0)) {
      const double f=a.gainDb==0?a.hz:b.gainDb==0?b.hz:
        std::exp(std::log(a.hz)-a.gainDb/(b.gainDb-a.gainDb)*std::log(b.hz/a.hz));
      if(f<100)continue;
      result.found=true;result.firstConfirmed=!earlierGap;
      result.hz=f;result.lowHz=a.hz;result.highHz=b.hz;
      result.message=earlierGap
        ?QString("fc 候选 = %1 Hz（前段缺口，首次未确认）").arg(f,0,'g',6)
        :QString("fc = %1 Hz（闭环 0 dB 向下交点，诊断）").arg(f,0,'g',6);
      return result;
    }
  }
  if(earlierGap)result.message="fc：有效相邻点无交点；存在缺口，首次未确认";
  return result;
}
double sweepClosedLoopCrossing(const std::vector<SweepPoint> &p) {
  const auto result=sweepClosedLoopCrossingResult(p);
  return result.firstConfirmed?result.hz:0;
}

} // namespace mt
