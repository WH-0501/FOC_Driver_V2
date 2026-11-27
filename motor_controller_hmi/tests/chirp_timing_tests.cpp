#include "sweep_recording.h"
#include "chirp_analysis.h"
#include "welch_analysis.h"
#include "protocol.h"
#include <QtTest>
#include <QtEndian>
#include <QBuffer>
#include <QJsonDocument>
#include <cmath>
#include <numbers>
using namespace mt;
class TimingTests:public QObject {
 Q_OBJECT
private slots:
 void negotiation() {
  QByteArray p(62,0);p[0]=1;p[40]=char(0xa9);p[41]=1;p[52]=char(0xac);p[53]=1;p[54]=7;p[55]=7;
  qToBigEndian<quint32>(20000,p.data()+8);
  p[56]=char(0xae);p[57]=1;qToBigEndian<quint16>(4000,p.data()+58);p[60]=1;p[61]=6;
  auto o=response(0xbd,p,Version::V2);ChirpTiming t;
  QVERIFY(readChirpTiming(o,20000,t));QCOMPARE(t.periodSamples(20000),80000u);QCOMPARE(t.totalSamples(20000),560000u);
  QVERIFY(!readChirpTiming(response(0xbd,p.left(56),Version::V2),20000,t));
  QVERIFY(!o["sweep_exact_range"].toBool());
  p[57]=2;o=response(0xbd,p,Version::V2);QVERIFY(o["sweep_exact_range"].toBool());QVERIFY(readChirpTiming(o,20000,t));
  p[57]=3;QVERIFY(!readChirpTiming(response(0xbd,p,Version::V2),20000,t));p[57]=1;
  p[61]=1;QVERIFY(!readChirpTiming(response(0xbd,p,Version::V2),20000,t));p[61]=6;
  p[60]=0;QVERIFY(!readChirpTiming(response(0xbd,p,Version::V2),20000,t));p[60]=1;
  p[40]=16;p[41]=char(0xa8);p[42]=1; // aborted motion retains timing at a distinct offset
  QVERIFY(readChirpTiming(response(0xbd,p,Version::V2),20000,t));QCOMPARE(t.measurePeriods,6u);
  QJsonObject legacy{{"host_h1",true},{"chirp_periods",4}};
  QVERIFY(readChirpTiming(legacy,20000,t));QCOMPARE(t.totalSamples(20000),80000u);
  legacy["chirp_periods"]=7;QVERIFY(!readChirpTiming(legacy,20000,t));
  o["chirp_settle_periods"]=-1;QVERIFY(!readChirpTiming(o,20000,t));
 }
 void dynamicWindows() {
  SweepCapture c;c.method=3;c.fs=20000;c.timing={4000,1,6};c.expected=c.timing.totalSamples(c.fs);c.completed=true;c.recording.reset(1,12);
  for(double f:{20.,40.,600.,1000.,1500.}){SweepPoint p;p.hz=f;c.points.push_back(p);}
  const double gain=.7,phase=-.32;
  for(quint32 i=0;i<c.expected;++i){double x=0,y=0;
    for(const auto &p:c.points){double a=2*std::numbers::pi*p.hz*i/c.fs;x+=.01*std::sin(a);y+=.01*gain*std::sin(a+phase);}
    // Large, intentionally wrong settling response must never enter an estimate.
    if(i<80000)y=2*x;
    c.recording.samples.push_back({i,float(x),float(y)});
  }
  auto a=analyzeChirp(c,{});QVERIFY2(a.ok(),qPrintable(a.error));QCOMPARE(a.cycles.size(),6u);QCOMPARE(a.cycleStats.size(),6u);
  QCOMPARE(a.settleSeconds,4.);QCOMPARE(a.periodSeconds,4.);
  for(const auto &p:a.raw){QCOMPARE(p.count,6);QVERIFY(p.valid);QVERIFY(std::abs(p.gainDb-20*std::log10(gain))<1e-5);QVERIFY(std::abs(p.phaseDeg-phase*180/std::numbers::pi)<1e-5);}
  auto oldPath=recordingH1(c.recording,3,c.fs,c.points,c.expected,c.timing);
  QCOMPARE(oldPath.size(),a.raw.size());for(size_t i=0;i<a.raw.size();++i)QVERIFY(std::abs(oldPath[i].gainDb-a.raw[i].gainDb)<1e-10);
  auto csv=chirpAnalysisCsv(a);QVERIFY(csv.contains("cycle6_gain_db"));QVERIFY(csv.contains("cycle6_phase_deg"));
  const auto lines=csv.split('\n');QCOMPARE(lines[0].split(',').size(),lines[1].split(',').size());
  auto w=analyzeWelch(c,4096,a.raw);QVERIFY2(w.ok(),qPrintable(w.error));QCOMPARE(w.usedSamples+w.tailSamples,480000u);
  auto filtered=analyzeChirp(c,{ChirpFilter::FeedbackMean16});QVERIFY(filtered.ok());QCOMPARE(filtered.filteredOutput.size(),480000u);QCOMPARE(filtered.filteredRecording.size(),560000u);
  for(size_t i=0;i<a.raw.size();++i)QCOMPARE(filtered.raw[i].gainDb,a.raw[i].gainDb);
  c.points=a.raw;QByteArray data;QBuffer out(&data);out.open(QIODevice::WriteOnly);QVERIFY(writeSweepCapture(out,c));out.close();
  QBuffer in(&data);in.open(QIODevice::ReadOnly);auto meta=in.readLine().mid(18);auto bodyStart=in.pos();SweepCapture restored;QString error;
  QVERIFY2(readSweepCapture(in,meta,restored,error),qPrintable(error));QVERIFY(restored.timing==c.timing);QCOMPARE(restored.expected,560000u);QVERIFY(restored.recording.complete(restored.expected));
  auto m=QJsonDocument::fromJson(meta).object();m.remove("chirp_period_ms");in.seek(bodyStart);
  QVERIFY(!readSweepCapture(in,QJsonDocument(m).toJson(),restored,error));
  c.recording.missing=1;QVERIFY(!analyzeChirp(c,{}).ok());QVERIFY(!analyzeWelch(c,4096,a.raw).ok());
  c.recording.missing=0;c.timing={};QVERIFY(!analyzeChirp(c,{}).ok());
 }
};
QTEST_GUILESS_MAIN(TimingTests)
#include "chirp_timing_tests.moc"
