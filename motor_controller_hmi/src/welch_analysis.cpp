#include "welch_analysis.h"
#include <algorithm>
#include <cmath>
#include <complex>
#include <limits>
#include <numbers>
#include <QStringList>
namespace mt { namespace {
using Complex=std::complex<double>;
constexpr double pi=std::numbers::pi;
void fft(std::vector<Complex> &a) {
  const size_t n=a.size();
  for(size_t i=1,j=0;i<n;++i){size_t bit=n>>1;for(;j&bit;bit>>=1)j^=bit;j^=bit;if(i<j)std::swap(a[i],a[j]);}
  for(size_t length=2;length<=n;length<<=1){const Complex step=std::polar(1.,-2*pi/length);
    for(size_t base=0;base<n;base+=length){Complex z=1;
      for(size_t j=0;j<length/2;++j){Complex u=a[base+j],v=z*a[base+j+length/2];a[base+j]=u+v;a[base+j+length/2]=u-v;z*=step;}}}
}
WelchMatch match(const SweepPoint &p,const std::vector<SweepPoint> &ref) {
  WelchMatch m;
  if(!p.valid)return m;
  auto good=[](const SweepPoint &v){return v.valid&&v.inputFit>=.9&&std::isfinite(v.gainDb)&&std::isfinite(v.phaseDeg);};
  for(size_t i=0;i<ref.size();++i){const auto &b=ref[i];
    if(good(b)&&std::abs(b.hz-p.hz)<1e-4){m.available=true;m.gainDb=b.gainDb;m.phaseDeg=b.phaseDeg;break;}
    if(!i)continue;
    const auto &a=ref[i-1];
    if(good(a)&&good(b)&&a.hz>0&&a.hz<p.hz&&p.hz<b.hz&&b.hz/a.hz<=1.3){
      double t=std::log(p.hz/a.hz)/std::log(b.hz/a.hz);m.available=m.interpolated=true;
      m.gainDb=a.gainDb+t*(b.gainDb-a.gainDb);
      m.phaseDeg=std::remainder(a.phaseDeg+t*std::remainder(b.phaseDeg-a.phaseDeg,360.),360.);break;}}
  if(m.available){m.gainDelta=p.gainDb-m.gainDb;m.phaseDelta=std::remainder(p.phaseDeg-m.phaseDeg,360.);}return m;
}
} // namespace
WelchAnalysis analyzeWelch(const SweepCapture &c,unsigned length,const std::vector<SweepPoint> &raw,const std::vector<SweepPoint> &stepped){
  WelchAnalysis out;out.localRetest=c.localRetest;out.segmentLength=length;out.sampleHz=c.fs;
  auto fail=[&](QString s){out.error=s;return out;};
  if(c.method!=3||!c.completed||!c.recording.complete(c.expected))return fail("Welch：要求完整、零缺样且零拒收的 Chirp 录波；不插补。");
  if(!c.timing.valid(c.fs)||c.expected!=c.timing.totalSamples(c.fs))return fail("Welch：录波长度与设备时序不一致。");
  if(length<2048||length>16384||(length&(length-1)))return fail("Welch：窗长仅支持 2048、4096、8192、16384 点。");
  if(c.points.size()<3)return fail("Welch：缺少扫频范围。");
  double previous=0;
  for(const auto &p:c.points){if(!std::isfinite(p.hz)||p.hz<20||p.hz<=previous||p.hz>2000||p.hz>c.fs*.1)return fail("Welch：扫频频率范围无效。");previous=p.hz;}
  for(size_t i=0;i<c.recording.samples.size();++i){const auto &v=c.recording.samples[i];
    if(v.index!=i||!std::isfinite(v.input)||!std::isfinite(v.output))return fail("Welch：存在序号缺口或非有限采样，不计算。");}
  const size_t start=c.timing.periodSamples(c.fs)*c.timing.settlePeriods,count=c.expected-start,hop=length/2,bins=length/2+1;
  if(count<length)return fail("Welch：测量段短于窗长。");
  out.segments=unsigned((count-length)/hop+1);
  if(out.segments<3)return fail("Welch：至少需要三个完整分段。");
  out.usedSamples=unsigned(length+(out.segments-1)*hop);out.tailSamples=unsigned(count-out.usedSamples);
  out.binHz=c.fs/length;
  std::vector<double> window(length),xx(bins),yy(bins),xx2(bins);std::vector<Complex> xy(bins),x(length),y(length);
  double sumw=0,sumw2=0;
  for(unsigned i=0;i<length;++i){window[i]=.5-.5*std::cos(2*pi*i/length);sumw+=window[i];sumw2+=window[i]*window[i];}
  out.enbwHz=c.fs*sumw2/(sumw*sumw);
  for(unsigned segment=0;segment<out.segments;++segment){size_t base=start+segment*hop;double mx=0,my=0;
    for(unsigned i=0;i<length;++i){const auto &v=c.recording.samples[base+i];mx+=v.input;my+=v.output;}mx/=length;my/=length;
    for(unsigned i=0;i<length;++i){const auto &v=c.recording.samples[base+i];x[i]=(v.input-mx)*window[i];y[i]=(v.output-my)*window[i];}
    fft(x);fft(y);
    for(size_t k=0;k<bins;++k){double power=std::norm(x[k]);xx[k]+=power;xx2[k]+=power*power;yy[k]+=std::norm(y[k]);xy[k]+=std::conj(x[k])*y[k];}}
  for(size_t k=1;k<bins;++k){double hz=k*out.binHz;if(hz<c.points.front().hz||hz>c.points.back().hz)continue;
    const double density=(k==length/2?1.:2.)/(c.fs*sumw2*out.segments);
    WelchBin b;b.inputPsd=xx[k]*density;b.outputPsd=yy[k]*density;b.crossReal=xy[k].real()*density;b.crossImag=xy[k].imag()*density;
    b.energySegments=xx2[k]>0?xx[k]*xx[k]/xx2[k]:0;
    const Complex h=xx[k]>1e-30?xy[k]/xx[k]:Complex{};
    SweepPoint p;p.hz=hz;p.count=int(out.segments);p.outputFit=out.segments;
    // Per-bin RMS from the one-sided PSD, not the sinusoid amplitude used by raw H1.
    p.input=std::sqrt(b.inputPsd*out.binHz);p.output=std::sqrt(b.outputPsd*out.binHz);
    p.gainDb=std::abs(h)>1e-30?20*std::log10(std::abs(h)):std::numeric_limits<double>::quiet_NaN();p.phaseDeg=std::arg(h)*180/pi;
    p.inputFit=xx[k]*yy[k]>1e-30?std::clamp(std::norm(xy[k])/(xx[k]*yy[k]),0.,1.):0;
    p.valid=p.input>1e-6&&std::abs(h)>1e-6&&p.inputFit>=.9&&std::isfinite(p.gainDb)&&std::isfinite(p.phaseDeg);
    p.status=p.valid?"Welch/CSD · 仅对照":"Welch/CSD · 相干性/激励不足";
    b.raw=match(p,raw);b.stepped=match(p,stepped);out.points.push_back(p);out.bins.push_back(b);
  }
  if(out.points.size()<3)return fail("Welch：当前窗长在扫频范围内不足三个频点。");
  for(const auto &group:{std::pair{QString("原始 H1"),&raw},std::pair{QString("逐频正弦"),&stepped}})
    for(const auto &p:*group.second){auto m=match(p,out.points);m.gainDelta=-m.gainDelta;m.phaseDelta=-m.phaseDelta;out.comparisons.push_back({group.first,p,m});}
  return out;
}
QString welchAnalysisSummary(const WelchAnalysis &a){
  if(!a.ok())return a.error;
  const auto valid=std::count_if(a.points.begin(),a.points.end(),[](const auto &p){return p.valid;});
  return QString("Welch/CSD 对照：未滤波的设备定义测量段；周期 Hann 窗 %1 点（%2 ms），50% 重叠，每段去均值；%3 段算术平均，FFT 栅格 %4 Hz，ENBW %5 Hz。尾部 %6 点不构成完整窗，未参与此对照（非丢样）。\n有效 %7/%8 个原生频点；γ² ≥0.9，无效点保留并断线。重叠段数不等于独立平均次数；能量等效段数也不是自由度。\nWelch 诊断：%9；基准按有效低频平台判断，仅作对照，不替代原始 H1。窗长影响泄漏和分辨率，不保证比周期 H1 更准确。")
    .arg(a.segmentLength).arg(1000*a.segmentLength/a.sampleHz,0,'g',6).arg(a.segments).arg(a.binHz,0,'g',6).arg(a.enbwHz,0,'g',6).arg(a.tailSamples)
    .arg(valid).arg(a.points.size()).arg(sweepBandwidth(a.points,a.localRetest));
}
QByteArray welchAnalysisCsv(const WelchAnalysis &a){
  QByteArray out="frequency_hz,welch_gain_db,welch_phase_deg,coherence,valid,input_psd_A2_per_Hz,output_psd_A2_per_Hz,csd_real_A2_per_Hz,csd_imag_A2_per_Hz,energy_equivalent_segments,segments,nperseg,bin_hz,enbw_hz,raw_gain_db,raw_phase_deg,welch_minus_raw_db,welch_minus_raw_deg,raw_interpolated,stepped_gain_db,stepped_phase_deg,welch_minus_stepped_db,welch_minus_stepped_deg,stepped_interpolated\n";
  auto num=[](double x){return QString::number(x,'g',17);};
  for(size_t i=0;i<a.points.size();++i){const auto &p=a.points[i];const auto &b=a.bins[i];QStringList row{num(p.hz),num(p.gainDb),num(p.phaseDeg),num(p.inputFit),p.valid?"1":"0",num(b.inputPsd),num(b.outputPsd),num(b.crossReal),num(b.crossImag),num(b.energySegments),QString::number(a.segments),QString::number(a.segmentLength),num(a.binHz),num(a.enbwHz)};
    for(const auto &m:{b.raw,b.stepped}){if(m.available)row<<num(m.gainDb)<<num(m.phaseDeg)<<num(m.gainDelta)<<num(m.phaseDelta)<<(m.interpolated?"1":"0");else row<<""<<""<<""<<""<<"";}
    out+=row.join(',').toUtf8()+'\n';}return out;
}
QByteArray welchComparisonCsv(const WelchAnalysis &a){
  QByteArray out="reference,frequency_hz,reference_gain_db,reference_phase_deg,reference_coherence,reference_valid,welch_available,welch_gain_db,welch_phase_deg,welch_minus_reference_db,welch_minus_reference_deg,welch_interpolated\n";
  auto num=[](double x){return QString::number(x,'g',17);};
  for(const auto &c:a.comparisons){const auto &p=c.reference;const auto &m=c.welch;
    QStringList row{c.source,num(p.hz),num(p.gainDb),num(p.phaseDeg),num(p.inputFit),p.valid?"1":"0",m.available?"1":"0"};
    if(m.available)row<<num(m.gainDb)<<num(m.phaseDeg)<<num(m.gainDelta)<<num(m.phaseDelta)<<(m.interpolated?"1":"0");else row<<""<<""<<""<<""<<"";
    out+=row.join(',').toUtf8()+'\n';}return out;
}
} // namespace mt
