#include "sweep_recording.h"
#include <QtEndian>
#include <QJsonArray>
#include <QJsonDocument>
#include <algorithm>
#include <cmath>
#include <complex>
#include <cstring>
#include <numbers>
namespace mt {
quint32 ChirpTiming::periodSamples(double fs) const {return valid(fs)?quint32(fs)*periodMs/1000:0;}
quint32 ChirpTiming::totalSamples(double fs) const {return periodSamples(fs)*(settlePeriods+measurePeriods);}
bool ChirpTiming::valid(double fs) const {
  return std::isfinite(fs)&&fs>=10000&&fs<=40000&&fs==std::floor(fs)&&
    periodMs>=1000&&periodMs<=60000&&periodMs%1000==0&&settlePeriods>=1&&settlePeriods<=253&&measurePeriods<=254&&
    measurePeriods>=2&&settlePeriods+measurePeriods<=255&&
    quint64(fs)*(periodMs/1000)*(settlePeriods+measurePeriods)<=1200000;
}
QString ChirpTiming::description() const {
  return QString("%1 s / 周期 · %2 个稳定周期 + %3 个测量周期 · 共 %4 s")
    .arg(periodMs/1000.).arg(settlePeriods).arg(measurePeriods).arg(periodMs/1000.*(settlePeriods+measurePeriods));
}
bool readChirpTiming(const QJsonObject &o,double fs,ChirpTiming &out) {
  ChirpTiming t;
  if(o.contains("chirp_timing_revision")) {
    if(o["chirp_timing_revision"].toInt()!=1&&o["chirp_timing_revision"].toInt()!=2)return false;
    t.periodMs=unsigned(o["chirp_period_ms"].toInt());
    t.settlePeriods=unsigned(o["chirp_settle_periods"].toInt());
    t.measurePeriods=unsigned(o["chirp_measure_periods"].toInt());
  } else if(o["chirp_timing_required"].toBool()||!o["host_h1"].toBool()||o["chirp_periods"].toInt()!=4) return false;
  if(!t.valid(fs))return false;
  out=t;return true;
}

void SweepRecording::reset(quint32 n, quint16 r) {
  node=n; run=r; missing=rejected=0; samples.clear();
}
bool SweepRecording::append(quint32 n, const QByteArray &p) {
  if (n!=node || !run) return false;
  if (p.size()<8) { ++rejected; return false; }
  if (qFromBigEndian<quint16>(p.constData()+2)!=run) return false;
  unsigned count=quint8(p[1]);
  quint32 first=qFromBigEndian<quint32>(p.constData()+4);
  if (p[0]!=1 || count<1 || count>6 || p.size()<8+int(count)*8 ||
      first>1200000u-count || samples.size()+count>1200000u) { ++rejected; return false; }
  quint32 next=samples.empty()?0:samples.back().index+1;
  if (first<next) { ++rejected; return false; }
  SweepRawSample batch[6];
  for(unsigned i=0;i<count;++i) {
    batch[i].index=first+i;
    quint32 x=qFromBigEndian<quint32>(p.constData()+8+8*i);
    quint32 y=qFromBigEndian<quint32>(p.constData()+12+8*i);
    memcpy(&batch[i].input,&x,4);memcpy(&batch[i].output,&y,4);
    if(!std::isfinite(batch[i].input)||!std::isfinite(batch[i].output)){++rejected;return false;}
  }
  missing+=first-next;
  samples.insert(samples.end(),batch,batch+count);
  return true;
}
bool SweepRecording::complete(quint32 expected) const {
  return expected>0 && !missing && !rejected && samples.size()==expected &&
         samples.front().index==0 && samples.back().index==expected-1;
}
QString SweepRecording::summary(quint32 expected, bool supported, bool measurementCompleted) const {
  if(!supported) return "固件未确认录波上传；保留设备频响，需更新配套固件";
  if(!samples.empty()&&samples.back().index+1>expected)
    return QString("同步录波已收 %1 点 · 序号覆盖 %2 点 · 缺样 %3 · 拒收帧 %4 · 设备状态快照 %5 点已落后，未确认完整，不用于带宽估计")
      .arg(samples.size()).arg(samples.back().index+1).arg(missing).arg(rejected).arg(expected);
  quint64 tail=expected && (samples.empty() || samples.back().index+1<expected)
      ? expected-(samples.empty()?0:samples.back().index+1) : 0;
  return QString("同步录波 %1 / %2 点 · 缺样 %3 · 拒收帧 %4 · %5")
      .arg(samples.size()).arg(expected).arg(missing+tail).arg(rejected)
      .arg(complete(expected)?(measurementCompleted?"测量完成且录波齐全，可复算 H1":"已产生数据收齐；测量未完成，不用于带宽估计"):"尚未完整，不用于录波带宽估计");
}
std::vector<SweepPoint> steppedFrequencyGrid(double low,double high,unsigned count,double fs) {
  if(!std::isfinite(low)||!std::isfinite(high)||!std::isfinite(fs)||fs<10000||fs>40000||
     fs!=std::floor(fs)||low<20||high<=low||high>std::min(2000.,fs*.1)||count<3||count>16)return {};
  std::vector<SweepPoint> out;
  auto add=[&](double f){auto p=std::llround(fs/f);double hz=fs/p;
    if(p<10||hz>2000||(!out.empty()&&hz<=out.back().hz))return false;
    SweepPoint point;point.hz=hz;point.status="等待完整逐频录波";out.push_back(point);return true;};
  unsigned tones=count;
  auto first=std::llround(fs/low),last=std::llround(fs/high),previous=first+1;
  if(first-last+1<tones)return {};
  for(unsigned i=0;i<tones;++i) {
    double requested=i==0?low:i+1==tones?high:std::exp(std::log(low)+(std::log(high)-std::log(low))*i/(tones-1));
    auto period=std::clamp(std::llround(fs/requested),last+tones-1-i,previous-1);
    if(!add(fs/period))return {};
    previous=period;
  }
  return out;
}
std::vector<SweepPoint> recordingH1(const SweepRecording &r,int method,double fs,
                                  const std::vector<SweepPoint> &points,quint32 expected,const ChirpTiming &timing) {
  std::vector<SweepPoint> out;
  if(!r.complete(expected)||fs<10000||fs>40000||method<1||method>3||points.empty())return out;
  if(method==3&&(!timing.valid(fs)||expected!=timing.totalSamples(fs)))return out;
  size_t offset=0;
  for(const auto &original:points) {
    if(!std::isfinite(original.hz)||original.hz<=0||original.hz>fs*.1)return {};
    size_t period=size_t(std::llround(fs/original.hz));
    size_t length=method==2?4*period:(method==3?timing.periodSamples(fs):size_t(std::llround(fs*.05)));
    size_t settle=method==2?((std::max((size_t(std::llround(fs))+49)/50,length)+period-1)/period)*period:(method==3?timing.settlePeriods*length:length);
    unsigned blocks=method==2?4:method==3?timing.measurePeriods:3;
    if(!length||offset+settle+blocks*length>r.samples.size())return {};
    double xx=0,yy=0,xa=0,ya=0;std::complex<double> cross=0;
    auto rotate=std::polar(1.,-2*std::numbers::pi*original.hz/fs);
    for(unsigned b=0;b<blocks;++b) {
      std::complex<double> x=0,y=0,z=1;
      for(size_t i=0;i<length;++i){const auto &v=r.samples[offset+settle+b*length+i];x+=double(v.input)*z;y+=double(v.output)*z;z*=rotate;}
      x*=2./length;y*=2./length;
      xx+=std::norm(x);yy+=std::norm(y);cross+=std::conj(x)*y;xa+=std::abs(x);ya+=std::abs(y);
    }
    SweepPoint p;p.hz=original.hz;p.input=xa/blocks;p.output=ya/blocks;p.outputFit=blocks;p.count=blocks;
    auto h=xx>1e-30?cross/xx:std::complex<double>{};
    double gain=std::abs(h);
    p.inputFit=xx*yy>1e-30?std::clamp(std::norm(cross)/(xx*yy),0.,1.):0;
    p.gainDb=gain>0?20*std::log10(gain):0;p.phaseDeg=std::arg(h)*180/std::numbers::pi;
    p.valid=p.input>1e-6&&gain>1e-6&&p.inputFit>=.9;
    p.status=p.valid?"有效·完整录波复算 H1":"录波相干性/幅值不足";
    out.push_back(p);
    if(method==2)offset+=settle+blocks*length;
  }
  if((method==2?offset:(method==3?timing.totalSamples(fs):size_t(std::llround(fs*.2))))!=expected)return {};
  return out;
}
bool writeSweepCapture(QIODevice &file,const SweepCapture &c) {
  QJsonArray points;
  for(const auto &p:c.points)points.append(QJsonObject{{"hz",p.hz},{"input",p.input},{"output",p.output},
    {"gain",p.gainDb},{"phase",p.phaseDeg},{"coherence",p.inputFit},{"blocks",p.count},{"valid",p.valid},{"status",p.status}});
  QJsonObject meta{{"format",3},{"method",c.method},{"sample_hz",c.fs},{"expected",double(c.expected)},
    {"run",int(c.recording.run)},{"node",double(c.recording.node)},{"complete_run",c.completed},
    {"missing",double(c.recording.missing)},{"rejected",double(c.recording.rejected)},{"points",points},{"diagnostic",c.diagnostic}};
  meta["measurement_scope"]=c.localRetest?"local":"full";
  if(c.method==3) {
    if(!c.timing.valid(c.fs))return false;
    meta["chirp_timing_revision"]=1;meta["chirp_period_ms"]=int(c.timing.periodMs);
    meta["chirp_settle_periods"]=int(c.timing.settlePeriods);meta["chirp_measure_periods"]=int(c.timing.measurePeriods);
  }
  auto put=[&](const QByteArray &b){return file.write(b)==b.size();};
  if(!put("# motortune_sweep "+QJsonDocument(meta).toJson(QJsonDocument::Compact)+"\n") ||
     !put("sample_index,time_s,input_A,feedback_A\n")) return false;
  for(const auto &v:c.recording.samples)
    if(!put(QString("%1,%2,%3,%4\n").arg(v.index).arg(v.index/std::max(1.,c.fs),0,'g',17)
            .arg(v.input,0,'g',9).arg(v.output,0,'g',9).toUtf8()))return false;
  return true;
}
bool readSweepCapture(QIODevice &file,const QByteArray &metadata,SweepCapture &out,QString &error) {
  SweepCapture c;
  auto m=QJsonDocument::fromJson(metadata).object();
  c.method=m["method"].toInt();c.fs=m["sample_hz"].toDouble();
  if(m.contains("measurement_scope")&&m["measurement_scope"]!="full"&&m["measurement_scope"]!="local") {error="测量用途元数据无效";return false;}
  if(m["format"].toInt()==3&&!m.contains("measurement_scope")) {error="缺少测量用途元数据";return false;}
  c.localRetest=m["measurement_scope"]=="local";
  double expected=m["expected"].toDouble(-1);
  if((m["format"].toInt()<1||m["format"].toInt()>3) || c.method<1 || c.method>3 || !std::isfinite(c.fs) ||
     c.fs<10000 || c.fs>40000 || expected<0 || expected>1200000 || expected!=std::floor(expected) ||
     file.readLine().trimmed()!="sample_index,time_s,input_A,feedback_A") {
    error="录波元数据/表头无效";return false;
  }
  if(c.method==3&&m["format"].toInt()>=2&&(!m.contains("chirp_timing_revision")||!readChirpTiming(m,c.fs,c.timing))) {
    error="Chirp 时序元数据缺失或无效";return false;
  }
  if(c.method==3&&m["format"].toInt()==1&&m.contains("chirp_timing_revision")) {error="时序扩展要求录波格式 2";return false;}
  if(c.method==3&&m["complete_run"].toBool()&&expected!=c.timing.totalSamples(c.fs)) {error="录波长度与设备时序不匹配";return false;}
  c.expected=quint32(expected);c.completed=m["complete_run"].toBool();c.diagnostic=m["diagnostic"].toString();
  c.recording.reset(quint32(m["node"].toDouble()),quint16(m["run"].toInt()));
  double rejected=m["rejected"].toDouble();
  if(!std::isfinite(rejected)||rejected<0||rejected>1200000) {error="拒收计数无效";return false;}
  c.recording.rejected=quint64(rejected);
  double previous=0;
  for(const auto &v:m["points"].toArray()) {
    auto p=v.toObject();SweepPoint point;
    point.hz=p["hz"].toDouble();point.input=p["input"].toDouble();point.output=p["output"].toDouble();
    point.gainDb=p["gain"].toDouble();point.phaseDeg=p["phase"].toDouble();
    point.inputFit=p["coherence"].toDouble();point.count=p["blocks"].toInt();point.outputFit=point.count;
    point.valid=p["valid"].toBool();point.status=p["status"].toString();
    if(point.hz<=previous||point.hz>2000||point.input<0||point.output<0||point.inputFit<0||point.inputFit>1||
       point.count<0||point.count>int(c.method==3?c.timing.measurePeriods:4)||c.points.size()>=16) {error="录波频点元数据无效";return false;}
    previous=point.hz;c.points.push_back(point);
  }
  quint32 next=0;int line=2;
  while(!file.atEnd()) {
    auto text=file.readLine().trimmed();++line;if(text.isEmpty())continue;
    auto fields=text.split(',');double values[4]={};bool good=fields.size()==4;
    if(good)for(int i=0;i<4;++i){bool ok;values[i]=fields[i].toDouble(&ok);good=good&&ok&&std::isfinite(values[i]);}
    if(!good||values[0]<next||values[0]>=1200000||values[0]!=std::floor(values[0])||
       std::abs(values[1]-values[0]/c.fs)>1e-8||std::abs(values[2])>1e6||std::abs(values[3])>1e6) {
      error=QString("录波第 %1 行无效（序号/采样时间/数值）").arg(line);return false;
    }
    auto index=quint32(values[0]);c.recording.missing+=index-next;next=index+1;
    c.recording.samples.push_back({index,float(values[2]),float(values[3])});
  }
  if(c.completed && (!c.expected || c.points.size()<3)) {error="完成状态缺少样本数或频点";return false;}
  out=std::move(c);return true;
}
} // namespace mt
