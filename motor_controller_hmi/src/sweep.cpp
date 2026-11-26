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
QString sweepBandwidth(const std::vector<SweepPoint> &p) {
  if (p.size() < 3 || !p[0].valid || !p[1].valid)
    return "未测得带宽：需要至少 3 个频点及有效低频基准";
  if (std::abs(p[0].gainDb - p[1].gainDb) > .5)
    return "未测得带宽：低频基准不平坦，请降低起始频率";
  for (size_t i = 1; i < p.size(); ++i)
    if (!(p[i].hz > p[i - 1].hz))
      return "未测得带宽：频点必须递增";
  double base = (p[0].gainDb + p[1].gainDb) / 2, threshold = base - 3;
  for (size_t i = 2; i < p.size(); ++i) {
    if (!p[i - 1].valid || !p[i].valid)
      return "未测得带宽：存在无效频点，需补测后确定首次交点";
    if (p[i - 1].gainDb > threshold && p[i].gainDb <= threshold) {
      double alpha =
          (threshold - p[i - 1].gainDb) / (p[i].gainDb - p[i - 1].gainDb);
      double bw = std::exp(std::log(p[i - 1].hz) +
                           alpha * std::log(p[i].hz / p[i - 1].hz));
      return QString("−3 dB 带宽 ≈ %1 Hz · 交点区间 [%2, %3] Hz · 低频基准 %4 "
                     "dB（前两点均值）")
          .arg(bw, 0, 'g', 6)
          .arg(p[i - 1].hz, 0, 'g', 5)
          .arg(p[i].hz, 0, 'g', 5)
          .arg(base, 0, 'f', 2);
    }
  }
  return "未测得带宽：当前频段未出现相对低频基准的 −3 dB 交点";
}
} // namespace mt
