#include "sweep_policy.h"
#include "chirp_analysis.h"
#include <algorithm>
#include <cmath>
#include <complex>
#include <numbers>
#include <numeric>
#include <limits>
#include <map>
#include <QStringList>
namespace mt {
namespace {
constexpr double pi=std::numbers::pi;
using Complex=std::complex<double>;
double db(double a){return a>1e-30?20*std::log10(a):-600.;}
double degrees(Complex a){return std::arg(a)*180/pi;}
// 63-tap Blackman windowed sinc, unit DC gain. Two centered circular passes
// give |B(f)|^2 >= 0 with zero phase, including exact periodic boundaries.
// This is NOT reflection-padded filtfilt: each complete device period is periodic.
std::vector<double> kernel(double cutoff,double fs) {
  std::vector<double> h(63);double sum=0;
  for(int i=0;i<63;++i){int m=i-31;double x=2*cutoff/fs;
    h[i]=(m?std::sin(pi*x*m)/(pi*m):x)*(.42-.5*std::cos(2*pi*i/62)+.08*std::cos(4*pi*i/62));sum+=h[i];}
  for(auto &v:h)v/=sum;
  return h;
}
std::vector<double> circularPass(const std::vector<double> &v,const std::vector<double> &h,size_t n) {
  std::vector<double> out(v.size());
  for(size_t base=0;base<v.size();base+=n)for(size_t i=0;i<n;++i){double y=0;
    for(int k=0;k<int(h.size());++k){auto j=(static_cast<long long>(i)+k-31+static_cast<long long>(n))%static_cast<long long>(n);y+=h[k]*v[base+size_t(j)];}out[base+i]=y;}
  return out;
}
double filterGain(const std::vector<double> &h,double f,double fs){double b=0;for(int i=0;i<int(h.size());++i)b+=h[i]*std::cos(2*pi*f/fs*(i-31));return b*b;}
Complex line(const std::vector<double> &v,size_t start,size_t n,double f,double fs){
  Complex sum=0,z=1,rot=std::polar(1.,-2*pi*f/fs);
  for(size_t i=0;i<n;++i){sum+=v[start+i]*z;z*=rot;}return sum*(2./n);
}
SweepPoint estimate(double f,const std::vector<Complex> &x,const std::vector<Complex> &y){
  double xx=0,yy=0,xa=0,ya=0;Complex xy=0;
  const int blocks=int(x.size());
  for(int b=0;b<blocks;++b){xx+=std::norm(x[b]);yy+=std::norm(y[b]);xy+=std::conj(x[b])*y[b];xa+=std::abs(x[b]);ya+=std::abs(y[b]);}
  Complex h=xx>1e-30?xy/xx:Complex{};SweepPoint p;
  p.hz=f;p.input=xa/blocks;p.output=ya/blocks;p.gainDb=db(std::abs(h));p.phaseDeg=degrees(h);
  p.inputFit=xx*yy>1e-30?std::clamp(std::norm(xy)/(xx*yy),0.,1.):0;p.outputFit=p.count=blocks;
  p.valid=p.input>1e-6&&std::abs(h)>1e-6&&p.inputFit>=.9;
  p.status=p.valid?QString("原始 H1 · %1 周期").arg(blocks):"原始 H1 · 相干性/幅值不足";return p;
}
struct SpectralPool {
  double xx=0,yy=0,xa=0,ya=0;Complex xy=0;unsigned count=0;
  void add(Complex x,Complex y){xx+=std::norm(x);yy+=std::norm(y);xy+=std::conj(x)*y;xa+=std::abs(x);ya+=std::abs(y);++count;}
  SweepPoint point(double hz,int blocks)const{
    Complex h=xx>1e-30?xy/xx:Complex{};SweepPoint p;
    p.hz=hz;p.input=count?xa/count:0;p.output=count?ya/count:0;p.gainDb=db(std::abs(h));p.phaseDeg=degrees(h);
    p.inputFit=xx*yy>1e-30?std::clamp(std::norm(xy)/(xx*yy),0.,1.):0;p.outputFit=p.count=blocks;
    p.valid=p.input>1e-6&&std::abs(h)>1e-6&&p.inputFit>=.9;
    p.status=p.valid?"邻频原始谱平均 · 诊断对比":"邻频原始谱平均 · 相干性/幅值不足";return p;
  }
};
void compare(ChirpFrequencyCheck &c,const SweepPoint &p,const std::vector<SweepPoint> &ref){
  // No extrapolation, no crossing invalid points or the large 40->600 Hz gap.
  for(size_t j=0;j<ref.size();++j){const auto &b=ref[j];
    if(b.valid && std::abs(b.hz-p.hz)<1e-4){c.referenceAvailable=true;c.referenceGain=b.gainDb;c.referencePhase=b.phaseDeg;break;}
    if(!j)continue;
    const auto &a=ref[j-1];
    if(a.valid&&b.valid&&a.hz<p.hz&&p.hz<b.hz&&b.hz/a.hz<=1.3){
      double t=std::log(p.hz/a.hz)/std::log(b.hz/a.hz);c.referenceAvailable=c.referenceInterpolated=true;
      c.referenceGain=a.gainDb+t*(b.gainDb-a.gainDb);
      c.referencePhase=std::remainder(a.phaseDeg+t*std::remainder(b.phaseDeg-a.phaseDeg,360.),360.);break;}}
  if(c.referenceAvailable){c.gainDelta=p.gainDb-c.referenceGain;c.phaseDelta=std::remainder(p.phaseDeg-c.referencePhase,360.);}
}
}
std::vector<double> feedbackMean16(const SweepRecording &r) {
  std::vector<double> out(r.samples.size(),std::numeric_limits<double>::quiet_NaN());
  std::array<double,16> ring{};double sum=0;unsigned used=0,pos=0;quint32 next=0;
  for(size_t i=0;i<r.samples.size();++i){const auto &s=r.samples[i];
    if(s.index!=next||!std::isfinite(s.output)){ring.fill(0);sum=0;used=pos=0;}
    next=s.index+1;if(!std::isfinite(s.output))continue;
    sum-=ring[pos];ring[pos]=s.output;sum+=s.output;pos=(pos+1)%16;used=std::min(used+1,16u);
    if(used==16)out[i]=sum/16;
  }return out;
}
// Fourth-order Butterworth, two unity-DC-gain biquads. Odd reflection and
// steady-state initialization suppress end transients; never bridge missing data.
std::vector<double> feedbackLpf(const SweepRecording &r,double fs,double fc) {
  if(!std::isfinite(fs)||fs<=0||!std::isfinite(fc)||fc<20||fc>=fs*.45||r.samples.size()<2)return {};
  const size_t n=r.samples.size(),pad=std::min(n-1,size_t(std::ceil(10*fs/fc)));
  for(size_t i=0;i<n;++i)if(r.samples[i].index!=i||!std::isfinite(r.samples[i].output))return {};
  std::vector<double> v(n+2*pad);
  for(size_t i=0;i<n;++i)v[pad+i]=r.samples[i].output;
  for(size_t i=0;i<pad;++i){v[pad-1-i]=2*v[pad]-r.samples[i+1].output;
    v[pad+n+i]=2*v[pad+n-1]-r.samples[n-2-i].output;}
  const double k=std::tan(pi*fc/fs);
  auto pass=[&]{for(double q:{0.541196100146197,1.306562964876377}){
    double norm=1/(1+k/q+k*k),b0=k*k*norm,b1=2*b0,b2=b0;
    double a1=2*(k*k-1)*norm,a2=(1-k/q+k*k)*norm;
    double z1=(1-b0)*v.front(),z2=(b2-a2)*v.front();
    for(auto &x:v){double y=b0*x+z1;z1=b1*x-a1*y+z2;z2=b2*x-a2*y;x=y;}
  }};
  pass();std::reverse(v.begin(),v.end());pass();std::reverse(v.begin(),v.end());
  return {v.begin()+pad,v.begin()+pad+n};
}
QString feedbackLpfDescription(double fs,double fc) {
  return QString("反馈 LPF 对照：四阶 Butterworth，单程截止 %1 Hz（−3 dB）；正反向零相位，总响应为单程幅值平方（截止处 −6 dB）。完整录波连续处理，奇对称延拓；边界仍可能有偏差。给定不滤波，仅对照，不作为最终带宽，不恢复带内信噪比。采样率 %2 Hz。")
    .arg(fc,0,'g',6).arg(fs,0,'g',6);
}
QString feedbackMean16Description(double fs) {
  if(!std::isfinite(fs)||fs<=0)return "反馈 Iq 16 点因果滑动均值；等待采样率。原始录波保留。";
  return QString("反馈 Iq 16 点因果滑动均值：延迟 %1 μs，首零点 %2 Hz；给定不滤波。结果包含滤波器衰减和相移，不代表电流环原始带宽。尖峰被摊到 16 点，未删除；启动/缺口后前 15 点不显示。")
    .arg(7.5e6/fs,0,'g',6).arg(fs/16,0,'g',6);
}
std::vector<SweepPoint> chirpFrequencyGrid(double low,double high,unsigned count) {
  std::vector<SweepPoint> out;
  if(!std::isfinite(low)||!std::isfinite(high)||low<sweepPolicy::minHz||high>sweepPolicy::maxHz||high<=low||count<sweepPolicy::minPoints||count>sweepPolicy::maxPoints)return out;
  double step=std::pow(high/low,1./(count-1)),next=low,previous=0;
  for(unsigned i=0;i<count;++i){double f=i==count-1?high:next;next*=step;
    f=std::max(std::round(f),previous+1);
    if(f>high)return {};
    SweepPoint p;p.hz=f;p.status="待完整录波后计算";out.push_back(p);previous=f;
  }
  return out;
}
std::vector<SweepPoint> sweepMeasurementGrid(int method,double low,double high,
    unsigned count,double fs,bool localRetest) {
  using namespace sweepPolicy;
  if(method!=2 && method!=3)return {};
  if(count<minPoints || count>maxPoints)return {};
  if(!localRetest)low=baselineStartHz;
  for(unsigned n=count;n<=maxPoints;++n) {
    auto grid=method==2?steppedFrequencyGrid(low,high,n,fs):chirpFrequencyGrid(low,high,n);
    if(localRetest)return grid;
    if(grid.empty())continue;
    size_t baseline=1;bool covered=true;
    for(size_t i=1;i<grid.size();++i) {
      if(grid[i].hz/grid[i-1].hz>maxAdjacentRatio)covered=false;
      if(grid[i].hz<=grid.front().hz*baselineMaxRatio)++baseline;
    }
    if(covered && baseline>=baselineMinPoints && baseline<grid.size() &&
       grid[baseline-1].hz/grid.front().hz>=baselineMinRatio)return grid;
  }
  return {};
}
ChirpAnalysis analyzeChirp(const SweepCapture &capture,const ChirpAnalysisOptions &options,const std::vector<SweepPoint> &reference){
  ChirpAnalysis out;auto fail=[&](QString s){out.error=s;return out;};
  if(capture.method!=3)return fail("请选择设备周期 Chirp 的 .samples.csv 录波");
  if(!capture.completed||!capture.recording.complete(capture.expected))return fail("录波未完成或存在缺样/拒收：不插补，不计算带宽");
  double fs=capture.fs;
  const auto &timing=capture.timing;
  if(!timing.valid(fs)||capture.expected!=timing.totalSamples(fs))
    return fail("录波长度与设备 Chirp 时序不一致");
  const int blocks=int(timing.measurePeriods);
  const size_t n=timing.periodSamples(fs),start=n*timing.settlePeriods;
  out.cycles.resize(blocks);out.cycleStats.resize(blocks);
  out.periodSeconds=timing.periodMs/1000.;out.settleSeconds=start/fs;
  if(capture.points.size()<3)return fail("缺少本次测量的频点信息");
  double previous=0;
  for(const auto &p:capture.points){
    if(!std::isfinite(p.hz)||p.hz<=previous||p.hz>fs*.1||std::abs(p.hz-std::round(p.hz))>1e-5)
      return fail("Chirp 频点必须为递增的整 Hz 谱线且不超过采样率的 1/10");
    previous=p.hz;
  }
  for(size_t i=0;i<capture.recording.samples.size();++i){const auto &v=capture.recording.samples[i];
    if(v.index!=i||!std::isfinite(v.input)||!std::isfinite(v.output))return fail("采样序号不连续或存在非有限值");}
  if(options.filter!=ChirpFilter::Off&&options.filter!=ChirpFilter::FeedbackMean16&&(!std::isfinite(options.cutoffHz)||options.cutoffHz<20||options.cutoffHz>=fs*.45))return fail("滤波截止频率须为 20 Hz 至 0.45 × 采样率之间");
  if(options.neighborHalfWidthHz>40)return fail("邻频平均半宽不得超过 40 Hz");
  std::vector<double> x(blocks*n),y(blocks*n);
  for(size_t i=0;i<blocks*n;++i){x[i]=capture.recording.samples[start+i].input;y[i]=capture.recording.samples[start+i].output;}
  for(size_t i=0;i<n;++i){double xm=0,ym=0;for(int b=0;b<blocks;++b){xm+=x[b*n+i];ym+=y[b*n+i];}xm/=blocks;ym/=blocks;
    for(int b=0;b<blocks;++b){double u=x[b*n+i],v=y[b*n+i];auto &s=out.cycleStats[b];
      out.inputRepeatRms+=(u-xm)*(u-xm);s.residualRms+=(v-ym)*(v-ym);s.mean+=v;s.rms+=v*v;s.peak=std::max(s.peak,std::abs(v));s.over02A+=std::abs(v)>.2;
      for(int c=b+1;c<blocks;++c)out.inputRepeatMax=std::max(out.inputRepeatMax,std::abs(u-x[c*n+i]));}}
  double power=0,residual=0;
  for(auto &s:out.cycleStats){power+=s.rms;residual+=s.residualRms;s.mean/=n;s.rms=std::sqrt(s.rms/n);s.residualRms=std::sqrt(s.residualRms/n);}
  out.inputRepeatRms=std::sqrt(out.inputRepeatRms/(blocks*n));out.outputRms=std::sqrt(power/(blocks*n));out.outputResidualRms=std::sqrt(residual/(blocks*n));
  // Linear decomposition for observation only. No samples are removed from H1.
  std::vector<double> spikeOnly(y.size());double spikePower=0;
  for(int b=0;b<blocks;++b)for(size_t i=0;i<n;){
    if(std::abs(y[b*n+i])<=.2){++i;continue;}
    size_t first=i,peak=i;
    while(i<n&&std::abs(y[b*n+i])>.2){double v=y[b*n+i];spikeOnly[b*n+i]=v;spikePower+=v*v;if(std::abs(v)>std::abs(y[b*n+peak]))peak=i;++i;}
    size_t base=start+b*n,globalPeak=base+peak;
    out.spikes.push_back({unsigned(b+1),unsigned(base+first),unsigned(base+i-1),unsigned(globalPeak),
      capture.recording.samples[globalPeak].input,y[b*n+peak],
      capture.recording.samples[base+first-1].output,
      base+i<capture.recording.samples.size()?double(capture.recording.samples[base+i].output):std::numeric_limits<double>::quiet_NaN()});
  }
  out.spikeEnergyFraction=power>0?spikePower/power:0;
  using Lines=std::pair<std::vector<Complex>,std::vector<Complex>>;
  std::map<int,Lines> cache;
  auto spectra=[&](int hz)->const Lines &{
    auto [it,inserted]=cache.try_emplace(hz);
    if(inserted){it->second.first.resize(blocks);it->second.second.resize(blocks);}
    if(inserted)for(int b=0;b<blocks;++b){it->second.first[b]=line(x,b*n,n,hz,fs);it->second.second[b]=line(y,b*n,n,hz,fs);}
    return it->second;
  };
  std::vector<double> taps;
  if(options.filter==ChirpFilter::FeedbackMean16||options.filter==ChirpFilter::FeedbackLpf){
    out.filteredRecording=options.filter==ChirpFilter::FeedbackMean16?feedbackMean16(capture.recording):feedbackLpf(capture.recording,fs,options.cutoffHz);
    if(out.filteredRecording.size()!=capture.expected)return fail("滤波对照无法处理当前录波");
    out.filteredOutput.assign(out.filteredRecording.begin()+start,out.filteredRecording.end());out.filteredInput=x;
    double sum=0;for(double v:out.filteredOutput)sum+=v*v;out.filteredOutputRms=std::sqrt(sum/y.size());
  } else if(options.filter!=ChirpFilter::Off){taps=kernel(options.cutoffHz,fs);
    out.filteredOutput=circularPass(circularPass(y,taps,n),taps,n);
    out.filteredInput=options.filter==ChirpFilter::Both?circularPass(circularPass(x,taps,n),taps,n):x;
    double sum=0;for(double v:out.filteredOutput)sum+=v*v;out.filteredOutputRms=std::sqrt(sum/y.size());}
  for(const auto &p:capture.points){const auto &[xx,yy]=spectra(int(p.hz));std::vector<Complex> xf(blocks),yf(blocks);ChirpFrequencyCheck check;check.gain.resize(blocks);check.phase.resize(blocks);
    Complex meanY=std::accumulate(yy.begin(),yy.end(),Complex{})/double(blocks);
    for(int b=0;b<blocks;++b){
      check.spikeLineRms+=std::norm(line(spikeOnly,b*n,n,p.hz,fs));
      check.residualLineRms+=std::norm(yy[b]-meanY);
      Complex h=std::norm(xx[b])>1e-30?yy[b]/xx[b]:Complex{};
      check.gain[b]=db(std::abs(h));check.phase[b]=degrees(h);
      SweepPoint point;point.hz=p.hz;point.gainDb=check.gain[b];point.phaseDeg=check.phase[b];
      point.input=std::abs(xx[b]);point.output=std::abs(yy[b]);point.valid=point.input>1e-6&&std::abs(h)>1e-6;
      point.inputFit=std::numeric_limits<double>::quiet_NaN(); // single block has no meaningful coherence
      out.cycles[b].push_back(point);
      if(!out.filteredOutput.empty()){xf[b]=line(out.filteredInput,b*n,n,p.hz,fs);yf[b]=line(out.filteredOutput,b*n,n,p.hz,fs);}}
    for(int a=0;a<blocks;++a)for(int b=a+1;b<blocks;++b){check.gainSpread=std::max(check.gainSpread,std::abs(check.gain[a]-check.gain[b]));check.phaseSpread=std::max(check.phaseSpread,std::abs(std::remainder(check.phase[a]-check.phase[b],360.)));}
    out.raw.push_back(estimate(p.hz,xx,yy));compare(check,out.raw.back(),reference);
    check.spikeLineRms=std::sqrt(check.spikeLineRms/blocks);check.residualLineRms=std::sqrt(check.residualLineRms/blocks);
    if(options.neighborHalfWidthHz){
      // Keep the two baseline lines unchanged. Symmetric bands only, capped at
      // 5% of center frequency and captured bounds; no endpoint extrapolation.
      int radius=out.raw.size()<=2?0:std::min({int(options.neighborHalfWidthHz),int(std::floor(.05*p.hz)),int(p.hz-capture.points.front().hz),int(capture.points.back().hz-p.hz)});
      check.bandLowHz=p.hz-radius;check.bandHighHz=p.hz+radius;check.bandLines=unsigned(2*radius+1);
      SpectralPool pool;for(int hz=int(check.bandLowHz);hz<=int(check.bandHighHz);++hz){const auto &[bx,by]=spectra(hz);for(int b=0;b<blocks;++b)pool.add(bx[b],by[b]);}
      out.neighborhood.push_back(pool.point(p.hz,blocks));
    }
    if(!out.filteredOutput.empty()){
      if(options.filter==ChirpFilter::FeedbackMean16){Complex response=0;for(int k=0;k<16;++k)response+=std::polar(1./16,-2*pi*p.hz*k/fs);check.filterDb=db(std::abs(response));}
      else if(options.filter==ChirpFilter::FeedbackLpf)check.filterDb=db(1/(1+std::pow(std::tan(pi*p.hz/fs)/std::tan(pi*options.cutoffHz/fs),8)));
      else check.filterDb=db(filterGain(taps,p.hz,fs));
      auto f=estimate(p.hz,xf,yf);f.status=options.filter==ChirpFilter::FeedbackMean16?"16 点均值后 H1 · 含滤波器响应":options.filter==ChirpFilter::FeedbackLpf?"LPF 零相位对照 · 非原始带宽":"零相位滤波对比 · 不代替原始带宽";
      out.filtered.push_back(f);}
    out.checks.push_back(check);
  }
  return out;
}
QString chirpAnalysisSummary(const SweepCapture &c,const ChirpAnalysisOptions &o,const ChirpAnalysis &a){
  if(!a.ok())return a.error;
  QString s=QString("run %1 · %2/%3 点 · 丢弃 0–%7 s，分析 %7–%8 s 的 %9 个完整周期（单周期 %10 s）\n原始 H1：%4\n输入周期最大差 %5 A；反馈非周期残差 RMS %6 A（包含噪声与非平稳变化，不等同于纯噪声）")
    .arg(c.recording.run).arg(c.recording.samples.size()).arg(c.expected).arg(sweepBandwidth(a.raw,c.localRetest)).arg(a.inputRepeatMax,0,'g',5).arg(a.outputResidualRms,0,'g',5).arg(a.settleSeconds).arg(c.expected/c.fs).arg(a.cycles.size()).arg(a.periodSeconds);
  if(a.inputRepeatMax>1e-6)s+="\n注意：给定各周期不完全重复，周期一致性需结合输入变化判断。";
  if(o.filter==ChirpFilter::FeedbackMean16)s+="\n"+feedbackMean16Description(c.fs)+"\n16 点均值仅对照；最终带宽仍采用原始 H1。";
  else if(o.filter==ChirpFilter::FeedbackLpf)s+="\n"+feedbackLpfDescription(c.fs,o.cutoffHz)+QString("\n反馈 RMS %1 → %2 A").arg(a.outputRms,0,'g',6).arg(a.filteredOutputRms,0,'g',6);
  else if(o.filter!=ChirpFilter::Off)s+=QString("\n%1 · 63 抽头 Blackman FIR，两次居中循环卷积，过渡中心 %2 Hz（非 −3 dB 点）；反馈 RMS %3 → %4 A。只适用于完整周期。滤波不修复同频带噪声，相干性不应因此改善；仅滤反馈会改变增益。")
    .arg(o.filter==ChirpFilter::Both?"给定/反馈同滤（零相位）":"仅反馈滤波（零相位）").arg(o.cutoffHz).arg(a.outputRms,0,'g',5).arg(a.filteredOutputRms,0,'g',5);
  s+=QString("\n|Iq| >0.2 A：%1 个连续事件，占测量段平方和 %2%；仅观察，不删点。谱线贡献为各测量周期复幅值的 RMS，不是独立噪声功率。")
    .arg(a.spikes.size()).arg(100*a.spikeEnergyFraction,0,'f',2);
  if(o.neighborHalfWidthHz)s+=QString("\n邻频对比：请求半宽 ±%1 Hz，实际不超过中心频率 5%%；首两点及边界保留单谱线。对原始互谱/自谱先求和再求 H1，相干性门槛仍为 0.9。仅诊断，不替代原始带宽；谱线数量不代表独立周期数。")
    .arg(o.neighborHalfWidthHz).replace("5%%","5%");
  s+="\n逐周期曲线为单周期 Y/X，不显示单周期相干性。相位差按 ±180° 环绕处理；不剔除尖峰或低相干周期。";
  return s;
}
QByteArray chirpAnalysisCsv(const ChirpAnalysis &a){
  QByteArray csv="frequency_hz,raw_gain_db,raw_phase_deg,raw_coherence,raw_valid";
  for(const char *suffix:{"gain_db","phase_deg"})for(size_t b=0;b<a.cycles.size();++b)
    csv+=",cycle"+QByteArray::number(b+1)+"_"+suffix;
  csv+=",gain_spread_db,phase_spread_deg,filtered_gain_db,filtered_phase_deg,filtered_coherence,filter_response_db,stepped_gain_db,stepped_phase_deg,chirp_minus_stepped_db,chirp_minus_stepped_deg,stepped_interpolated,spike_line_rms_A,residual_line_rms_A,band_low_hz,band_high_hz,band_lines,band_gain_db,band_phase_deg,band_coherence,band_valid\n";
  auto num=[](double x){return QString::number(x,'g',17);};
  for(size_t i=0;i<a.raw.size();++i){const auto &p=a.raw[i];const auto &c=a.checks[i];QStringList row{num(p.hz),num(p.gainDb),num(p.phaseDeg),num(p.inputFit),p.valid?"1":"0"};
    for(double v:c.gain)row<<num(v);
    for(double v:c.phase)row<<num(v);
    row<<num(c.gainSpread)<<num(c.phaseSpread);
    if(a.filtered.empty())row<<""<<""<<""<<"";else{const auto &f=a.filtered[i];row<<num(f.gainDb)<<num(f.phaseDeg)<<num(f.inputFit)<<num(c.filterDb);}
    if(c.referenceAvailable)row<<num(c.referenceGain)<<num(c.referencePhase)<<num(c.gainDelta)<<num(c.phaseDelta)<<(c.referenceInterpolated?"1":"0");else row<<""<<""<<""<<""<<"";
    row<<num(c.spikeLineRms)<<num(c.residualLineRms);
    if(a.neighborhood.empty())row<<""<<""<<""<<""<<""<<""<<"";
    else{const auto &b=a.neighborhood[i];row<<num(c.bandLowHz)<<num(c.bandHighHz)<<QString::number(c.bandLines)<<num(b.gainDb)<<num(b.phaseDeg)<<num(b.inputFit)<<(b.valid?"1":"0");}
    csv+=row.join(',').toUtf8()+'\n';}
  return csv;
}
QByteArray chirpSpikesCsv(const ChirpAnalysis &a,double fs){
  QByteArray csv="cycle,first_index,last_index,peak_index,peak_time_s,cycle_time_s,duration_us,input_A,peak_Iq_A,before_Iq_A,after_Iq_A\n";
  if(!std::isfinite(fs)||fs<=0)return csv;
  for(const auto &s:a.spikes){QStringList row;for(unsigned v:{s.cycle,s.firstIndex,s.lastIndex,s.peakIndex})row<<QString::number(v);
    for(double v:{s.peakIndex/fs,s.peakIndex/fs-a.settleSeconds-(s.cycle-1)*a.periodSeconds,(s.lastIndex-s.firstIndex+1)*1e6/fs,s.input,s.peak,s.before,s.after})row<<QString::number(v,'g',17);
    csv+=row.join(',').toUtf8()+'\n';}return csv;
}
} // namespace mt
