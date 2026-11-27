#include "sweep_recording.h"
#include "chirp_analysis.h"
#include "protocol.h"
#include <QtTest>
#include <QtEndian>
#include <QBuffer>
#include <QJsonDocument>
#include <QJsonArray>
#include <cmath>
#include <cstring>
#include <numbers>
using namespace mt;
static QByteArray packet(quint32 first, unsigned n=6,quint16 run=42) {
  QByteArray p(8+8*n,0);p[0]=1;p[1]=char(n);qToBigEndian(run,p.data()+2);qToBigEndian(first,p.data()+4);
  for(unsigned i=0;i<2*n;++i){float f=float(first+i)*.00001f;quint32 bits;memcpy(&bits,&f,4);qToBigEndian(bits,p.data()+8+4*i);}
  return p;
}
class RecordingTests:public QObject {
 Q_OBJECT
private slots:
 void historicalRecordingsPreserveH1() {
  auto dir=qEnvironmentVariable("MOTORTUNE_RANGE_RECORDINGS");if(dir.isEmpty())QSKIP("Optional historical capture directory");
  for(int run=1;run<=3;++run){
    QFile f(dir+QString("/test1126-0.2A-%1.csv.samples.csv").arg(run));QVERIFY(f.open(QIODevice::ReadOnly));
    auto meta=f.readLine().mid(18);SweepCapture c;QString error;QVERIFY2(readSweepCapture(f,meta,c,error),qPrintable(error));
    QVERIFY(c.completed&&c.recording.complete(80000));
    auto a=analyzeChirp(c,{});QVERIFY2(a.ok(),qPrintable(a.error));QCOMPARE(a.raw.size(),c.points.size());
    double delta=0;for(size_t i=0;i<a.raw.size();++i){delta=std::max(delta,std::abs(a.raw[i].gainDb-c.points[i].gainDb));QVERIFY(std::abs(a.raw[i].phaseDeg-c.points[i].phaseDeg)<1e-7);QVERIFY(std::abs(a.raw[i].inputFit-c.points[i].inputFit)<1e-8);QCOMPARE(a.raw[i].valid,c.points[i].valid);}
    QVERIFY(delta<1e-7);auto bw=sweepBandwidthResult(a.raw);QVERIFY(bw.baselineValid&&bw.found);
    QVERIFY(!sweepBandwidthResult(a.raw,true).found);
    qInfo().noquote()<<QString("capture %1: max raw gain delta %2 dB; %3").arg(run).arg(delta,0,'g',6).arg(bw.message);
  }
 }

 void requestedRangeAndScope() {
  auto chirp=chirpFrequencyGrid(600,1500,16);QCOMPARE(chirp.size(),16u);
  QCOMPARE(chirp.front().hz,600.);QCOMPARE(chirp.back().hz,1500.);
  auto stepped=steppedFrequencyGrid(600,1500,16,20000);QCOMPARE(stepped.size(),16u);
  QVERIFY(std::abs(stepped.front().hz-20000./33)<1e-9);
  QVERIFY(std::abs(stepped.back().hz-20000./13)<1e-9);
  for(size_t i=1;i<stepped.size();++i)QVERIFY(stepped[i].hz>stepped[i-1].hz);
  QCOMPARE(chirpFrequencyGrid(600,1500,3).size(),3u);
  for(bool local:{false,true}) {
    SweepCapture c;c.method=3;c.localRetest=local;c.fs=20000;c.recording.reset(1,4);c.points=chirp;
    QByteArray bytes;QBuffer out(&bytes);out.open(QIODevice::WriteOnly);QVERIFY(writeSweepCapture(out,c));out.close();
    QBuffer in(&bytes);in.open(QIODevice::ReadOnly);auto meta=in.readLine().mid(18);auto start=in.pos();SweepCapture restored;QString error;
    QVERIFY2(readSweepCapture(in,meta,restored,error),qPrintable(error));QCOMPARE(restored.localRetest,local);
    auto m=QJsonDocument::fromJson(meta).object();QCOMPARE(m["format"].toInt(),3);
    m["measurement_scope"]="bad";in.seek(start);QVERIFY(!readSweepCapture(in,QJsonDocument(m).toJson(),restored,error));
    m.remove("measurement_scope");in.seek(start);QVERIFY(!readSweepCapture(in,QJsonDocument(m).toJson(),restored,error));
    m["format"]=2;in.seek(start);QVERIFY(readSweepCapture(in,QJsonDocument(m).toJson(),restored,error));QVERIFY(!restored.localRetest);
  }
 }

 void transportIntegrity() {
  SweepRecording r;r.reset(1,42);
  QVERIFY(!r.append(2,packet(0)));QVERIFY(!r.append(1,packet(0,6,41)));
  QCOMPARE(r.rejected,0u);QVERIFY(r.append(1,packet(0)));QVERIFY(r.complete(6));
  QVERIFY(!r.complete(12));QVERIFY(r.append(1,packet(12)));
  QCOMPARE(r.missing,6u);QVERIFY(!r.complete(18));QVERIFY(r.summary(24,true).contains("缺样 12"));
  QVERIFY(!r.append(1,packet(6)));QCOMPARE(r.rejected,1u);
  r.reset(1,42);auto p=packet(0);p[1]=7;QVERIFY(!r.append(1,p));QVERIFY(r.samples.empty());
  p=packet(0);p.chop(1);QVERIFY(!r.append(1,p));
  p=packet(0);qToBigEndian(quint32(0x7fc00000),p.data()+8);QVERIFY(!r.append(1,p));
  p=packet(0);p.append(QByteArray(6,0));QVERIFY(r.append(1,p));QVERIFY(!r.complete(6));
  r.reset(1,43);QVERIFY(!r.append(1,packet(0)));QVERIFY(r.samples.empty());
 }
 void rawH1KnownResponse() {
  const double fs=20000,phase=-.42,gain=.7;
  for(int method:{1,2,3}) {
    SweepRecording r;r.reset(1,42);std::vector<SweepPoint> points;
    for(double f:{20.,40.,1000.}) {SweepPoint p;p.hz=f;points.push_back(p);}
    size_t offset=0;
    auto append=[&](double f,size_t n,bool multiple){
      for(size_t i=0;i<n;++i) {
        double x=0,y=0;
        for(double hz:multiple?std::vector<double>{20.,40.,1000.}:std::vector<double>{f}) {
          double a=2*std::numbers::pi*hz*i/fs;x+=.01*std::sin(a);y+=.01*gain*std::sin(a+phase);
        }
        r.samples.push_back({quint32(offset++),float(x),float(y)});
      }
    };
    if(method!=2)append(0,method==1?4000:80000,true);
    else for(auto p:points){size_t period=size_t(std::llround(fs/p.hz)),length=4*period;size_t settle=((std::max(size_t(400),length)+period-1)/period)*period;append(p.hz,settle+4*length,false);}
    auto out=recordingH1(r,method,fs,points,quint32(r.samples.size()));
    QCOMPARE(out.size(),points.size());
    for(auto p:out){QVERIFY(p.valid);QVERIFY(std::abs(p.gainDb-20*std::log10(gain))<1e-5);QVERIFY(std::abs(p.phaseDeg-phase*180/std::numbers::pi)<1e-5);QVERIFY(p.inputFit>.99999);}
    r.missing=1;QVERIFY(recordingH1(r,method,fs,points,quint32(r.samples.size())).empty());
  }
 }
 void actualFirmwareFrames() {
  const auto dir=qEnvironmentVariable("MOTORTUNE_CAPTURE_FIXTURES");
  if(dir.isEmpty())QSKIP("Set MOTORTUNE_CAPTURE_FIXTURES to native integration output for cross-language regression");
  for(int method:{1,2,3}) {
    QFile file(dir+QString("/capture-%1.frames").arg(method));QVERIFY(file.open(QIODevice::ReadOnly));
    SweepRecording r;r.reset(1,0x1234);std::vector<SweepPoint> device;
    while(!file.atEnd()){
      auto line=file.readLine().trimmed();
      if(line.startsWith("R ")){
        auto fields=line.split(' ');QCOMPARE(fields.size(),7);
        SweepPoint p;p.hz=fields[1].toDouble();p.input=fields[2].toDouble();p.output=fields[3].toDouble();
        double re=fields[4].toDouble(),im=fields[5].toDouble();
        p.gainDb=20*std::log10(std::hypot(re,im));p.phaseDeg=std::atan2(im,re)*180/std::numbers::pi;p.inputFit=fields[6].toDouble();device.push_back(p);
      }else QVERIFY(r.append(1,QByteArray::fromHex(line)));
    }
    auto points=recordingH1(r,method,20000,device,quint32(r.samples.size()));
    QCOMPARE(points.size(),device.size());
    for(size_t i=0;i<points.size();++i) {
      QVERIFY2(std::abs(points[i].gainDb-device[i].gainDb)<.005,qPrintable(QString("mode %1 hz %2 gain %3 / %4").arg(method).arg(points[i].hz).arg(points[i].gainDb).arg(device[i].gainDb)));
      QVERIFY(std::abs(points[i].phaseDeg-device[i].phaseDeg)<.05);
      QVERIFY(std::abs(points[i].inputFit-device[i].inputFit)<.001);
    }
  }
 }
 void offlineCsv() {
  QJsonObject meta{{"format",1},{"method",1},{"sample_hz",20000},{"expected",6},{"run",42},{"node",1},{"complete_run",false}};
  auto metadata=QJsonDocument(meta).toJson(QJsonDocument::Compact);
  QByteArray csv="sample_index,time_s,input_A,feedback_A\n0,0,0.125,-0.25\n1,0.00005,0.15,0.1\n5,0.00025,0,0\n";
  QBuffer f(&csv);f.open(QIODevice::ReadOnly);SweepCapture c;QString error;
  QVERIFY(readSweepCapture(f,metadata,c,error));QCOMPARE(c.recording.samples.size(),3u);
  QCOMPARE(c.recording.missing,3u);QVERIFY(!c.recording.complete(6));
  csv.replace("1,0.00005","1,0.00006");f.seek(0);QVERIFY(!readSweepCapture(f,metadata,c,error));
  csv.replace("1,0.00006","1,0.00005");csv.replace("5,0.00025","0,0");f.seek(0);QVERIFY(!readSweepCapture(f,metadata,c,error));
  QCOMPARE(QByteArray("# motortune_sweep ").size(),18);
  QByteArray encoded;QBuffer output(&encoded);output.open(QIODevice::WriteOnly);
  QVERIFY(writeSweepCapture(output,c));output.close();
  QBuffer input(&encoded);input.open(QIODevice::ReadOnly);SweepCapture restored;
  QVERIFY(readSweepCapture(input,input.readLine().mid(18),restored,error));
  QCOMPARE(restored.recording.samples.size(),c.recording.samples.size());
  QCOMPARE(restored.recording.missing,c.recording.missing);
  for(size_t i=0;i<c.recording.samples.size();++i){
    QCOMPARE(restored.recording.samples[i].input,c.recording.samples[i].input);
    QCOMPARE(restored.recording.samples[i].output,c.recording.samples[i].output);
    QCOMPARE(restored.recording.samples[i].index,c.recording.samples[i].index);
  }
 }
 void negotiation() {
  QByteArray p(46,0);p[0]=1;p[40]=char(0xa9);p[41]=1;p[42]=1;
  QVERIFY(response(0xbc,p,Version::V2)["recording"].toBool());
  QVERIFY(!response(0xbc,p.left(40),Version::V2).contains("recording"));
  QVERIFY(!response(0xbc,p,Version::V2).contains("record_dropped"));
  p.resize(62);p.fill(0);p[0]=1;p[40]=char(0xa9);p[41]=1;p[42]=1;
  QVERIFY(!response(0xbc,p,Version::V2).contains("record_dropped"));
  p[43]=char(0xaa);qToBigEndian<quint32>(17804,p.data()+44);qToBigEndian<quint32>(62196,p.data()+48);
  QCOMPARE(response(0xbd,p,Version::V2)["record_dropped"].toDouble(),17804.);
  QCOMPARE(response(0xbd,p,Version::V2)["record_submitted"].toDouble(),62196.);
  QVERIFY(!response(0xbd,p.left(51),Version::V2).contains("record_dropped"));
  p[41]=2;QVERIFY(!response(0xbc,p,Version::V2).contains("recording"));
 }
};
QTEST_GUILESS_MAIN(RecordingTests)
#include "sweep_recording_tests.moc"
