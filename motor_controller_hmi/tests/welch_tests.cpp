#include "welch_analysis.h"
#include "chirp_analysis.h"
#include "chirp_analysis_dialog.h"
#include <QtTest>
#include <QtWidgets>
#include <complex>
#include <numbers>
using namespace mt;
namespace {
constexpr double pi=std::numbers::pi;
SweepCapture capture(){
  SweepCapture c;c.method=3;c.fs=20000;c.expected=80000;c.completed=true;c.recording.reset(1,42);
  c.points=chirpFrequencyGrid(20,2000,16);
  for(unsigned i=0;i<c.expected;++i){double t=(i%20000)/c.fs;float x=float(.05*std::sin(2*pi*(20*t+990*t*t)));
    float y=i>=2?.7f*c.recording.samples[i-2].input:0;c.recording.samples.push_back({i,x,y});}return c;
}
}
class WelchTests:public QObject {
 Q_OBJECT
private slots:
 void knownGainDelayAndSpectrum(){
  auto c=capture();auto a=analyzeWelch(c,2048);QVERIFY2(a.ok(),qPrintable(a.error));QCOMPARE(a.segments,57u);QCOMPARE(a.usedSamples,59392u);QCOMPARE(a.tailSamples,608u);
  QCOMPARE(a.binHz,20000./2048);QVERIFY(std::abs(a.enbwHz-1.5*a.binHz)<1e-10);
  for(auto p:a.points)if(p.hz>=100&&p.hz<=1800){QVERIFY(p.valid);QVERIFY(std::abs(p.gainDb-20*std::log10(.7))<.08);QVERIFY(std::abs(p.phaseDeg+360*p.hz*2/c.fs)<.3);}
  // Independent direct DFT checks FFT normalization, conjugation and Hann/CSD scaling.
  unsigned length=2048,k=100;double xx=0,yy=0,sw2=0;std::complex<double> xy=0;
  for(unsigned j=0;j<length;++j){double w=.5-.5*std::cos(2*pi*j/length);sw2+=w*w;}
  for(unsigned b=0;b<a.segments;++b){size_t base=20000+b*1024;double mx=0,my=0;
    for(unsigned j=0;j<length;++j){mx+=c.recording.samples[base+j].input;my+=c.recording.samples[base+j].output;}mx/=length;my/=length;
    std::complex<double> x=0,y=0;
    for(unsigned j=0;j<length;++j){double w=.5-.5*std::cos(2*pi*j/length);auto z=std::polar(1.,-2*pi*k*j/length);const auto &v=c.recording.samples[base+j];x+=(v.input-mx)*w*z;y+=(v.output-my)*w*z;}
    xx+=std::norm(x);yy+=std::norm(y);xy+=std::conj(x)*y;}
  double scale=2/(c.fs*sw2*a.segments);size_t i=0;while(a.points[i].hz<k*a.binHz)++i;
  auto b=a.bins[i];QVERIFY(std::abs(b.inputPsd-xx*scale)<1e-16);QVERIFY(std::abs(b.outputPsd-yy*scale)<1e-16);
  QVERIFY(std::abs(b.crossReal-xy.real()*scale)<1e-16);QVERIFY(std::abs(b.crossImag-xy.imag()*scale)<1e-16);
 }
 void rejectIncompleteAndBadSamples(){
  const auto good=capture();
  auto c=good;c.completed=false;QVERIFY(!analyzeWelch(c).ok());c=good;c.recording.missing=1;QVERIFY(!analyzeWelch(c).ok());
  c=good;c.recording.rejected=1;QVERIFY(!analyzeWelch(c).ok());c=good;c.recording.samples[40000].index++;QVERIFY(!analyzeWelch(c).ok());
  c=good;c.recording.samples[40000].output=std::numeric_limits<float>::quiet_NaN();QVERIFY(!analyzeWelch(c).ok());
  c=good;c.method=2;QVERIFY(!analyzeWelch(c).ok());QVERIFY(!analyzeWelch(good,3000).ok());
 }
 void coherenceAndInvalidGap(){
  auto c=capture();quint32 state=123;
  for(auto &v:c.recording.samples){state=1664525u*state+1013904223u;v.output=float((double(state)/4294967296.-.5)*.2);}
  auto a=analyzeWelch(c);QVERIFY(a.ok());unsigned rejected=0;
  for(const auto &p:a.points){if(p.inputFit<.9){QVERIFY(!p.valid);++rejected;}}
  QVERIFY(rejected>a.points.size()*9/10);QVERIFY(!sweepBandwidthResult(a.points).found);
  std::vector<SweepPoint> ref(3);for(int i=0;i<3;++i){ref[i].hz=900+100*i;ref[i].gainDb=-1;ref[i].inputFit=1;ref[i].valid=i!=1;}
  auto b=analyzeWelch(capture(),4096,ref,ref);
  for(size_t i=0;i<b.points.size();++i)if(b.points[i].hz>900&&b.points[i].hz<1100){QVERIFY(!b.bins[i].raw.available);QVERIFY(!b.bins[i].stepped.available);}
  // Bandwidth uses the existing conservative gate on the full, unfiltered native grid.
  auto p=b.points;for(auto &v:p){v.valid=true;v.gainDb=0;}p[2].valid=false;p[4].gainDb=-4;
  QVERIFY(!sweepBandwidthResult(p).found);
 }
 void comparisonAndRawUnchanged(){
  auto c=capture();auto before=analyzeChirp(c,{});QVERIFY(before.ok());auto a=analyzeWelch(c,4096,before.raw,before.raw);QVERIFY(a.ok());
  auto after=analyzeChirp(c,{});QCOMPARE(after.raw.size(),before.raw.size());unsigned matched=0;
  for(size_t i=0;i<before.raw.size();++i){QCOMPARE(before.raw[i].gainDb,after.raw[i].gainDb);QCOMPARE(before.raw[i].inputFit,after.raw[i].inputFit);}
  for(const auto &b:a.comparisons)matched+=b.welch.available;QVERIFY(matched>0);
  auto csv=welchAnalysisCsv(a);QVERIFY(csv.startsWith("frequency_hz,welch_gain_db"));QCOMPARE(csv.count('\n'),qsizetype(a.points.size()+1));
 }
 void controlsKeepRaw(){
  auto c=capture();auto d=createChirpAnalysisDialog(nullptr,c);d->show();QTest::qWait(50);
  auto selector=d->findChild<QComboBox*>("chirpOfflineWelch");QVERIFY(selector);QCOMPARE(selector->currentData().toUInt(),4096u);
  auto raw=d->findChild<QTableWidget*>("chirpOfflinePoints");auto table=d->findChild<QTableWidget*>("chirpWelchPoints");QVERIFY(raw&&table);QVERIFY(table->rowCount()>100);
  QString gain=raw->item(5,1)->text();selector->setCurrentIndex(0);QCOMPARE(table->rowCount(),0);QCOMPARE(raw->item(5,1)->text(),gain);
  selector->setCurrentIndex(3);QVERIFY(table->rowCount()>100);QCOMPARE(raw->item(5,1)->text(),gain);
  QVERIFY(d->findChild<QLabel*>("chirpWelchSummary")->text().contains("0.9"));delete d;
 }
};
QTEST_MAIN(WelchTests)
#include "welch_tests.moc"
