#include "sweep.h"
#include "protocol.h"
#include <QtTest>
#include <cmath>
#include <numbers>
#include <random>
using namespace mt;
class SweepTests : public QObject {
  Q_OBJECT
private slots:
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
    for (double hz : {.5, 1., 5., 10., 20., 40.}) {
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
    for (double hz : {1., 2., 4.}) {
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
