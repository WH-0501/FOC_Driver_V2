#include "sweep.h"
#include "protocol.h"
#include <QtTest>
#include <cmath>
#include <numbers>
#include <random>
#include <QtEndian>
#include <cstring>
#include <limits>
using namespace mt;
class SweepTests : public QObject {
  Q_OBJECT
private slots:
  void qualifiedBaselineAndLocalScope() {
    std::vector<SweepPoint> p;
    for(auto [f,g]:{std::pair{20.,-.1},std::pair{30.,.1},std::pair{40.,0.},std::pair{80.,-1.},std::pair{160.,-2.},std::pair{320.,-4.}}) {
      SweepPoint v;v.hz=f;v.gainDb=g;v.phaseDeg=f<=40?-2:-30;v.inputFit=.99;v.valid=true;p.push_back(v);
    }
    const auto original=p;auto b=sweepBandwidthResult(p);
    QVERIFY(b.found);QCOMPARE(b.baselineCount,3u);QCOMPARE(b.baselineDb,0.);
    QVERIFY(std::abs(b.hz-std::sqrt(160.*320.))<1e-9);
    QVERIFY(!sweepBandwidthResult(p,true).found);QVERIFY(!sweepBandwidthResult(p,true).baselineValid);
    for(size_t i=0;i<p.size();++i)QCOMPARE(p[i].gainDb,original[i].gainDb);
    p[3].valid=false;QVERIFY(!sweepBandwidthResult(p).found);p=original;
    p[3].inputFit=.8999;QVERIFY(!sweepBandwidthResult(p).found);p=original;
    p[0].valid=false;QVERIFY(!sweepBandwidthResult(p).baselineValid);p=original;
    p[2].phaseDeg=-16;QVERIFY(!sweepBandwidthResult(p).baselineValid);p=original;
    p[2].gainDb=.6;QVERIFY(!sweepBandwidthResult(p).baselineValid);p=original;
    for(auto &v:p)v.gainDb-=3;QVERIFY(!sweepBandwidthResult(p).baselineValid);p=original;
    p[3].hz=100;QVERIFY(!sweepBandwidthResult(p).found);p=original;
    // Qualification is based on response, not literal 20/40 Hz anchors.
    for(auto &v:p)v.hz/=20;QVERIFY(sweepBandwidthResult(p).found);
    QCOMPARE(sweepBandwidthResult(p).baselineLowHz,1.);
  }

  void closedLoopCrossingInterpolation() {
    auto make=[](std::initializer_list<std::pair<double,double>> rows){
      std::vector<SweepPoint> p;for(auto [f,g]:rows){SweepPoint v;v.hz=f;v.gainDb=g;v.valid=true;p.push_back(v);}return p;
    };
    // No sample exactly at zero; the search threshold applies to the crossing,
    // not to the left endpoint. Log-frequency interpolation matches the plot.
    auto p=make({{20,-.1},{40,-.1},{93,1},{126,-1},{200,-2}});
    auto r=sweepClosedLoopCrossingResult(p);
    QVERIFY(r.found&&r.firstConfirmed);QVERIFY(std::abs(r.hz-std::sqrt(93.*126.))<1e-10);
    QCOMPARE(r.lowHz,93.);QCOMPARE(r.highHz,126.);
    QCOMPARE(sweepClosedLoopCrossing(p),r.hz);
    p[2].gainDb=.01;p[3].gainDb=-1;QVERIFY(!sweepClosedLoopCrossingResult(p).found); // crossing <100
    p=make({{200,.2613310917},{267,-.1468987801}});
    QVERIFY(std::abs(sweepClosedLoopCrossing(p)-240.63437656)<1e-6); // real run 64122 bracket
    p=make({{93,1},{100,0},{126,-1}});QCOMPARE(sweepClosedLoopCrossing(p),100.);
    p=make({{100,0},{126,-1}});QCOMPARE(sweepClosedLoopCrossing(p),100.);
    p=make({{100,0},{126,0},{200,0}});QVERIFY(!sweepClosedLoopCrossingResult(p).found);
    p=make({{100,-1},{126,1}});QVERIFY(!sweepClosedLoopCrossingResult(p).found); // downward fc only
  }
  void closedLoopCrossingNeverBridgesGaps() {
    std::vector<SweepPoint> p(6);
    const double hz[]={20,40,100,200,300,400},gain[]={0,0,1,-1,1,-1};
    for(size_t i=0;i<p.size();++i){p[i].hz=hz[i];p[i].gainDb=gain[i];p[i].valid=true;}
    p[3].valid=false;
    auto r=sweepClosedLoopCrossingResult(p);
    QVERIFY(r.found);QVERIFY(!r.firstConfirmed);QCOMPARE(r.lowHz,300.);QCOMPARE(r.highHz,400.);
    QVERIFY(std::abs(r.hz-std::sqrt(300.*400.))<1e-10);
    QVERIFY(r.message.contains("候选"));QVERIFY(r.message.contains("首次未确认"));
    QCOMPARE(sweepClosedLoopCrossing(p),0.); // legacy caller cannot mistake candidate for confirmed fc
    p.resize(4);QVERIFY(!sweepClosedLoopCrossingResult(p).found);
    p[3].valid=true;p[2].valid=false;QVERIFY(!sweepClosedLoopCrossingResult(p).found);
    p[2].valid=true;p[2].gainDb=std::numeric_limits<double>::quiet_NaN();QVERIFY(!sweepClosedLoopCrossingResult(p).found);
    p[2].gainDb=1;p[2].hz=600;p[3].hz=800;p[1].gainDb=1;
    r=sweepClosedLoopCrossingResult(p);QVERIFY(r.found&&!r.firstConfirmed);QCOMPARE(r.lowHz,600.);
    p[2].gainDb=-1;QVERIFY(!sweepClosedLoopCrossingResult(p).found); // never use anchor 40 -> 600
    p[2].hz=p[1].hz;QVERIFY(!sweepClosedLoopCrossingResult(p).found);
    p[2].hz=std::numeric_limits<double>::infinity();QVERIFY(!sweepClosedLoopCrossingResult(p).found);
  }
  void fcDoesNotChangeRawH1OrBandwidth() {
    std::vector<SweepPoint> p;
    for(auto [f,g]:{std::pair{20.,0.},std::pair{30.,0.},std::pair{40.,0.},std::pair{60.,.1},std::pair{90.,1.},std::pair{126.,-1.},std::pair{200.,-1.},std::pair{400.,-1.},std::pair{800.,-2.},std::pair{1600.,-4.}}){SweepPoint v;v.hz=f;v.gainDb=g;v.valid=true;v.inputFit=1;p.push_back(v);}
    const auto original=p;const auto before=sweepBandwidthResult(p);
    const auto fc=sweepClosedLoopCrossingResult(p);QVERIFY(fc.found&&fc.firstConfirmed);
    const auto after=sweepBandwidthResult(p);
    QCOMPARE(before.message,after.message);QCOMPARE(before.hz,after.hz);QVERIFY(after.found);
    QVERIFY(std::abs(after.hz-std::sqrt(800.*1600.))<1e-9);
    for(size_t i=0;i<p.size();++i){QCOMPARE(p[i].gainDb,original[i].gainDb);QCOMPARE(p[i].valid,original[i].valid);}
    p[3].valid=false;QVERIFY(!sweepBandwidthResult(p).found); // bandwidth still refuses earlier invalid points
  }
  void motionAbortDiagnostics() {
    QByteArray p(62,0);p[0]=1;p[2]=3;p[3]=2;p[6]=16;p[15]=char(159);p[17]=16;
    p[27]=char(0xa5);p[31]=char(159);p[38]=char(0xa6);p[39]=1;
    p[40]=19;p[41]=char(0xa8);p[42]=1;p[43]=0;
    auto put=[&](int offset,float value){quint32 bits;memcpy(&bits,&value,4);qToBigEndian<quint32>(bits,p.data()+offset);};
    put(44,-5.25f);put(48,5);put(52,20);
    auto o=response(0xbd,p,Version::V2);
    QVERIFY(o["valid"].toBool());QCOMPARE(o["motion_reason"].toInt(),19);
    QCOMPARE(o["motion_value"].toDouble(),-5.25);QCOMPARE(o["motion_limit"].toDouble(),5.0);
    QCOMPARE(o["motion_hz"].toDouble(),20.0);
    auto message=sweepAbortMessage(o);
    QVERIFY(message.contains("速度越界"));QVERIFY(message.contains("-5.25 rad/s"));
    QVERIFY(message.contains("20 Hz"));QVERIFY(message.contains("第 1/16 点"));
    QVERIFY(message.contains("159 个采样周期"));QVERIFY(!message.contains("异常位置"));
    p[40]=18;p[43]=2;put(44,-.5f);put(48,.5f);put(52,588.2353f);
    o=response(0xbd,p,Version::V2);message=sweepAbortMessage(o);
    QVERIFY(message.contains("位移越界"));QVERIFY(message.contains("相对起点 -0.5 rad"));
    QVERIFY(message.contains("第 3/16 点"));QVERIFY(message.contains("588.235"));
    p[3]=1;p[43]=char(255);put(52,0);
    message=sweepAbortMessage(response(0xbd,p,Version::V2));
    QVERIFY(message.contains("多正弦并行"));QVERIFY(!message.contains("触发频率 0"));
    p[3]=3;put(52,333.25f);
    o=response(0xbd,p,Version::V2);
    QVERIFY(o["valid"].toBool());QCOMPARE(o["motion_hz"].toDouble(),333.25);
    message=sweepAbortMessage(o);QVERIFY(message.contains("Chirp 瞬时激励频率 333.25 Hz"));
    p[3]=1;put(52,0);
    QVERIFY(!response(0xbd,p.left(40),Version::V2).contains("motion_reason"));
    message=sweepAbortMessage(response(0xbd,p.left(40),Version::V2));
    QVERIFY(message.contains("固件未提供详细诊断"));QVERIFY(!message.contains("异常位置"));
    p[42]=2;QVERIFY(!response(0xbd,p,Version::V2).contains("motion_reason"));p[42]=1;
    p[39]=5;QVERIFY(!response(0xbd,p,Version::V2).contains("motion_reason"));p[39]=1;
    put(44,std::numeric_limits<float>::quiet_NaN());
    o=response(0xbd,p,Version::V2);QVERIFY(o["valid"].toBool());
    QVERIFY(o["motion_diagnostic_invalid"].toBool());QVERIFY(!o.contains("motion_value"));
    QVERIFY(sweepAbortMessage(o).contains("诊断扩展数据无效"));
    put(44,5.25f);p[2]=1;QVERIFY(!response(0xbd,p,Version::V2).contains("motion_reason"));
  }
  void deviceStartRejectionDetail() {
    // 48 effective bytes plus CAN-FD padding, 75.25 C in big-endian binary32.
    QByteArray p(62,0);p[0]=1;p[1]=1;p[38]=char(0xa6);p[39]=1;
    p[40]=21;p[41]=char(0xa7);p[42]=1;p[44]=0x42;p[45]=char(0x96);p[46]=char(0x80);
    auto o=response(0xbc,p,Version::V2);
    QVERIFY(o["valid"].toBool());QVERIFY(!o["ok"].toBool());
    QCOMPARE(o["reject_reason"].toInt(),21);QCOMPARE(o["reject_value"].toDouble(),75.25);
    QVERIFY(o["message"].toString().contains("电机温度"));
    QVERIFY(o["message"].toString().contains("75.25"));
    QVERIFY(!o.contains("encoder_us"));
    p[40]=char(250);o=response(0xbc,p,Version::V2);
    QVERIFY(o["message"].toString().contains("未知启动拒绝原因"));
    QVERIFY(!response(0xbc,p.left(40),Version::V2).contains("reject_reason"));
    p[39]=5;QVERIFY(!response(0xbc,p,Version::V2).contains("reject_reason"));
    p[39]=1;p[42]=2;QVERIFY(!response(0xbc,p,Version::V2).contains("reject_reason"));
  }
  void deviceFeatureDisabledReply() {
    QByteArray p(22,0);p[0]=1;p[1]=3;p[4]=0x12;p[5]=0x34;
    const auto o=response(0xbc,p,Version::V2);
    QVERIFY(o["valid"].toBool());QVERIFY(!o["ok"].toBool());
    QCOMPARE(o["run"].toInt(),0x1234);
    QVERIFY(o["message"].toString().contains("未编译扫频"));
  }
  void compactTimingRevisionDecode() {
    QByteArray p(46,0);p[0]=1;p[2]=2;p[21]=49;p[23]=51;p[25]=50;p[27]=char(0xa5);
    p[33]=8;p[35]=17;p[37]=4;p[38]=char(0xa6);p[39]=1;
    const auto o=response(0xbd,p,Version::V2);
    QVERIFY(o["valid"].toBool());QCOMPARE(o["interval_min_us"].toInt(),49);
    QCOMPARE(o["interval_max_us"].toInt(),51);QCOMPARE(o["fra_us"].toInt(),4);
    QCOMPARE(o["control_us"].toInt(),17);QCOMPARE(o["timing_revision"].toInt(),1);
    QVERIFY(!o.contains("encoder_us"));
  }
  void timingDiagnosticsDecodeAfterAbort() {
    QByteArray p(46,0);p[0]=1;p[2]=3;p[6]=16;p[15]=1;p[17]=4;p[19]=35;p[21]=75;p[23]=75;p[25]=75;p[26]=1;p[27]=char(0xa5);p[31]=1;
    auto o=response(0xbd,p,Version::V2);QVERIFY(o["valid"].toBool());QCOMPARE(o["max_us"].toInt(),35);QCOMPARE(o["interval_last_us"].toInt(),75);QCOMPARE(o["failure_sample"].toInt(),1);QCOMPARE(o["timing_reason"].toInt(),1);
  }
  void deviceProtocolAcceptsFdPadding() {
    QByteArray status(22,0);status[0]=1;status[2]=2;status[3]=1;status[6]=16;
    auto s=response(0xbc,status,Version::V2);QVERIFY(s["valid"].toBool());QVERIFY(s["ok"].toBool());QCOMPARE(s["count"].toInt(),16);
    QByteArray point(46,0);point[0]=1;point[5]=2;
    QVERIFY(response(0xbe,point,Version::V2)["valid"].toBool());
    QVERIFY(!response(0xbc,status.left(10),Version::V2)["valid"].toBool());
    point[0]=2;QVERIFY(!response(0xbe,point,Version::V2)["valid"].toBool());
  }

  void noisyFundamentalRemainsMeasurable() {
    std::mt19937 rng(12345);
    std::normal_distribution<double> noise(0,.029);
    std::vector<SweepSample> data;
    for(int i=0;i<10000;++i) {
      double t=i*.001,a=2*std::numbers::pi*.5*t;
      data.push_back({t,.05*std::sin(a),.05*std::sin(a-.1)+noise(rng)});
    }
    auto p=measureSweep(.5,data);
    QVERIFY2(p.valid,qPrintable(p.status));
    QVERIFY(p.outputFit<.8);QVERIFY(std::abs(p.gainDb)<.5);
    QVERIFY(std::abs(p.phaseDeg+.1*180/std::numbers::pi)<2);
    for(auto &v:data)if(v.time>5)v.output*=2;
    QVERIFY(!measureSweep(.5,data).valid);
  }

  void knownLowPass() {
    std::vector<SweepPoint> points;
    for (double hz : {.5, .75, 1., 2., 4., 8., 10., 20., 40.}) {
      std::vector<SweepSample> data;
      double gain = 1 / std::sqrt(1 + hz * hz / 100),
             phase = -std::atan(hz / 10);
      for (int i = 0; i < 1000; ++i) {
        double t = i / (hz * 200);
        double a = 2 * std::numbers::pi * hz * t;
        data.push_back(
            {t, .2 * std::sin(a) + .3, .2 * gain * std::sin(a + phase) - .1});
      }
      auto p = measureSweep(hz, data);
      QVERIFY2(p.valid, qPrintable(p.status));
      QVERIFY(std::abs(p.gainDb - 20 * std::log10(gain)) < 1e-8);
      QVERIFY(std::abs(p.phaseDeg - phase * 180 / std::numbers::pi) < 1e-8);
      points.push_back(p);
    }
    QVERIFY(sweepBandwidth(points).contains("交点区间 [10, 20]"));
    points[3].valid = false;
    QVERIFY(sweepBandwidth(points).contains("无效频点"));
  }
  void noCrossingAndNoInput() {
    std::vector<SweepPoint> points;
    for (double hz : {1., 1.5, 2., 4.}) {
      std::vector<SweepSample> data;
      for (int i = 0; i < 500; ++i) {
        double t = i / (hz * 100);
        double y = std::sin(2 * std::numbers::pi * hz * t);
        data.push_back({t, y, y});
      }
      points.push_back(measureSweep(hz, data));
      for (auto &p : data)
        p.input = 0;
      QVERIFY(!measureSweep(hz, data).valid);
      data[100].time = -1;
      QVERIFY(measureSweep(hz, data).status.contains("回退"));
    }
    QVERIFY(sweepBandwidth(points).contains("未出现"));
  }
  void rejectShortOrNoisy() {
    std::vector<SweepSample> data;
    for (int i = 0; i < 500; ++i) {
      double t = i * .01, a = 2 * std::numbers::pi * t;
      data.push_back({t, std::sin(a), std::sin(a) + 2 * std::sin(7 * a)});
    }
    QVERIFY(measureSweep(1, data).status.contains("谐波"));
    data.resize(100);
    QVERIFY(measureSweep(1, data).status.contains("周期"));
  }
};
QTEST_GUILESS_MAIN(SweepTests)
#include "sweep_tests.moc"
