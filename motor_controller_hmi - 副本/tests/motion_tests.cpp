#include "motion_control.h"
#include <QtEndian>
#include <QtTest>
using namespace mt;

class EnabledChanges : public QObject {
  public:
    int count = 0;
    bool eventFilter(QObject *, QEvent *event) override {
        if (event->type() == QEvent::EnabledChange)
            ++count;
        return false;
    }
};

class MotionTests : public QObject {
    Q_OBJECT
    template <class T> static T *child(QObject &o, const char *name) { return o.findChild<T *>(name); }
    static QByteArray limitPacket(int format, int flags) {
        QByteArray p(20, 0);
        p[0] = char(format);
        p[6] = 1;
        p[7] = char(flags);
        qToBigEndian<quint16>(60, p.data() + 2);
        qToBigEndian<quint16>(54, p.data() + 4);
        qToBigEndian<qint32>(-52000, p.data() + 8);
        qToBigEndian<qint32>(-500, p.data() + 12);
        qToBigEndian<qint32>(-185, p.data() + 16);
        return p;
    }
  private slots:
    void modeCompatibility() {
        // Independent compatibility matrix from the legacy V1/V2 control panels.
        const QList<int> v1Masks = {1, 1, 6, 0, 2, 7};
        const QList<int> v2Masks = {0, 4, 2, 1, 1, 2, 4, 7, 0, 4, 2, 1, 0};
        for (Version v : {Version::V1, Version::V2}) {
            auto masks = v == Version::V1 ? v1Masks : v2Masks;
            for (int m = 0; m < masks.size(); ++m)
                for (int c = 0; c < 3; ++c)
                    QCOMPARE(modeAcceptsTarget(v, m, c), bool(masks[m] & (1 << c)));
            QVERIFY(!modeAcceptsTarget(v, 255, 2));
        }
    }
    void speedLimitFormats() {
        auto v3 = response(0xb5, limitPacket(3, 1) + QByteArray(2, 0), Version::V2);
        QVERIFY(v3["valid"].toBool());
        QVERIFY(v3["ok"].toBool());
        QVERIFY(!v3["hard_enabled"].toBool());
        QVERIFY(v3["limiting"].toBool());
        QCOMPARE(v3["speed_rpm"].toDouble(), -52.0);
        QCOMPARE(v3["iq_reference"].toDouble(), -.5);
        QCOMPARE(v3["iq_limited"].toDouble(), -.185);
        auto v2 = response(0xb5, limitPacket(2, 0), Version::V1);
        QVERIFY(v2["hard_enabled"].toBool());
        QVERIFY(!v2["limiting"].toBool());
        for (const auto &p :
             {QByteArray(), limitPacket(3, 0).left(16), limitPacket(1, 2), limitPacket(4, 2)}) {
            auto o = response(0xb6, p, Version::V2);
            QVERIFY(!o["valid"].toBool());
            QVERIFY(!o["ok"].toBool());
        }
        auto p = limitPacket(3, 2);
        qToBigEndian<quint16>(61, p.data() + 4);
        QVERIFY(!response(0xb5, p, Version::V2)["valid"].toBool());
        p = limitPacket(3, 2);
        p[1] = 2;
        QVERIFY(!response(0xb6, p, Version::V2)["ok"].toBool());
        QVERIFY(!requiresDisabled(0xb6));
    }
    void runtimeGates() {
        Session s;
        QSignalSpy failures(&s, &Session::commandFailed), replies(&s, &Session::replied);
        s.open({});
        QTRY_VERIFY(s.telemetryFresh(1));
        s.command(1, 0x41, QByteArray(1, 1));
        QTRY_COMPARE(s.latest(1)->mode, 1);
        s.command(1, 0x40, QByteArray(1, 1));
        QTRY_VERIFY(s.latest(1)->enabled);
        auto wrongMode = s.command(1, 3, target(1, .2, 2, Version::V2), 6);
        QTRY_VERIFY(!failures.empty());
        QCOMPARE(failures.last()[0].toULongLong(), wrongMode);
        auto wrongChannel = s.command(1, 2, target(1, 10, 1, Version::V2));
        QTRY_COMPARE(failures.size(), 2);
        QCOMPARE(failures.last()[0].toULongLong(), wrongChannel);
        auto tune = s.command(1, 0xb6, be16(0) + be16(0));
        auto result = [&]() -> QJsonObject {
            for (auto row : replies)
                if (row[0].toULongLong() == tune)
                    return row[3].toJsonObject();
            return {};
        };
        QTRY_VERIFY(result()["ok"].toBool());
        QCOMPARE(result()["soft_rpm"].toInt(), 0);
        QVERIFY(!result()["hard_enabled"].toBool());
        QVERIFY(s.latest(1)->enabled);
        s.command(1, 0x7f, be16(0));
        QTRY_VERIFY(!s.telemetryFresh(1));
        auto stale = s.command(1, 0xb6, be16(54) + be16(1));
        QTRY_VERIFY(failures.size() >= 3);
        QCOMPARE(failures.last()[0].toULongLong(), stale);
        s.shutdown();
        QVERIFY(s.wait(3000));
    }
    void liveControls() {
        Session s;
        MotionControl w(s);
        w.resize(1050, 250);
        w.show();
        s.open({});
        QTRY_VERIFY(s.telemetryFresh(1));
        s.command(1, 0x41, QByteArray(1, 1));
        QTRY_COMPARE(s.latest(1)->mode, 1);
        w.setContext(1, Version::V2, 1, true, false);
        auto current = child<QDoubleSpinBox>(w, "motionTarget2");
        auto send = child<QPushButton>(w, "sendTarget2");
        auto apply = child<QPushButton>(w, "applySpeedLimits");
        auto soft = child<QSpinBox>(w, "softLimitRpm");
        auto hard = child<QCheckBox>(w, "hardProtectionDraft");
        QVERIFY(current->isEnabled());
        QVERIFY(!child<QDoubleSpinBox>(w, "motionTarget1")->isEnabled());
        QVERIFY(!send->isEnabled());
        s.command(1, 0x40, QByteArray(1, 1));
        QTRY_VERIFY(s.latest(1)->enabled);
        w.setContext(1, Version::V2, 1, true, false);
        QVERIFY(send->isEnabled());
        QSignalSpy operationLogs(&s, &Session::log);
        current->setValue(-.350);
        send->click();
        auto hasAction = [&]() {
            for (const auto &row : operationLogs)
                if (row[0].toString() == "ACTION" && row[1].toString().contains("界面操作：下发目标电流") &&
                    row[1].toString().contains("-0.350 A") && row[1].toString().contains("CMD=0x03") &&
                    row[1].toString().contains("请求#"))
                    return true;
            return false;
        };
        QTRY_VERIFY(hasAction());
        QTRY_VERIFY(!s.sampling.targetEvents().empty());
        QTRY_VERIFY(s.sampling.targetEvents().back().state >= 1);
        QCOMPARE(s.sampling.targetEvents().back().values[2], -.350);
        QTRY_VERIFY(std::abs(s.latest(1)->values[0] + .350) < .05);
        QTRY_VERIFY(apply->isEnabled());
        soft->setValue(0);
        hard->setChecked(false);
        apply->click();
        auto softState = child<QLabel>(w, "softLimitState");
        auto hardState = child<QLabel>(w, "hardLimitState");
        QTRY_VERIFY(softState->text().contains("已关闭"));
        QVERIFY(hardState->text().contains("已关闭"));
        // Manual reads must not overwrite unsent edits or mistake unsolicited data for a reply.
        soft->setValue(66);
        child<QPushButton>(w, "readSpeedLimits")->click();
        s.replied(0, 1, 0x53, {});
        QTest::qWait(1100);
        QCOMPARE(soft->value(), 66);
        QVERIFY(softState->text().contains("已关闭"));
        w.setContext(1, Version::V2, 1, false, false);
        QVERIFY(!send->isEnabled());
        w.setContext(1, Version::V2, 6, true, false);
        QVERIFY(!send->isEnabled());
        w.setContext(1, Version::V2, 1, true, true);
        QVERIFY(!send->isEnabled());
        w.setContext(1, Version::V2, 1, true, false);
        child<QPushButton>(w, "zeroCurrent")->click();
        QTRY_VERIFY(std::abs(s.latest(1)->values[0]) < .05);
        w.setContext(2, Version::V2, 1, true, false);
        QCOMPARE(current->value(), 0.0);
        QVERIFY(!send->isEnabled());
        QVERIFY(hardState->text().contains("待确认"));
        s.shutdown();
        QVERIFY(s.wait(3000));
    }
    void limitsAreReadOnceThenOnlyOnClick() {
        Session s;
        MotionControl w(s);
        w.show();
        QSignalSpy replies(&s, &Session::replied);
        auto countReplies = [&](int command) {
            int count = 0;
            for (const auto &row : replies)
                if (row[2].toInt() == command)
                    ++count;
            return count;
        };
        s.open({});
        QTRY_VERIFY(s.telemetryFresh(1));
        w.setContext(1, Version::V2, 0, true, false);
        auto read = child<QPushButton>(w, "readSpeedLimits");
        auto apply = child<QPushButton>(w, "applySpeedLimits");
        QTRY_COMPARE(countReplies(0xb5), 1);
        QVERIFY(read->isEnabled());
        EnabledChanges changes;
        read->installEventFilter(&changes);
        // Repeated UI refreshes past the former 2-second expiry never send CAN commands.
        for (int i = 0; i < 25; ++i) {
            w.setContext(1, Version::V2, 0, true, false);
            QTest::qWait(100);
        }
        QCOMPARE(countReplies(0xb5), 1);
        QCOMPARE(countReplies(0xb6), 0);
        QVERIFY(apply->isEnabled());
        QVERIFY(child<QLabel>(w, "softLimitState")->text().contains("已开启"));
        // Rapid clicks during an in-flight read reuse it, without visual state churn.
        for (int i = 0; i < 8; ++i)
            read->click();
        QTRY_COMPARE(countReplies(0xb5), 2);
        QCOMPARE(changes.count, 0);
        child<QSpinBox>(w, "softLimitRpm")->setValue(45);
        child<QCheckBox>(w, "hardProtectionDraft")->setChecked(false);
        QTest::qWait(600);
        QCOMPARE(countReplies(0xb6), 0);
        apply->click();
        QTRY_COMPARE(countReplies(0xb6), 1);
        QTest::qWait(600);
        QCOMPARE(countReplies(0xb5), 2); // B6's own response is sufficient; no implicit B5.
        w.setContext(1, Version::V2, 0, true, true);
        QVERIFY(!read->isEnabled());
        w.setContext(1, Version::V2, 0, true, false);
        QVERIFY(read->isEnabled());
        w.setContext(2, Version::V2, 0, true, false);
        w.setContext(1, Version::V2, 1, true, false);
        QTest::qWait(600);
        QCOMPARE(countReplies(0xb5), 2); // Changing node/mode is not a manual query.
        QVERIFY(child<QLabel>(w, "softLimitState")->text().contains("待确认"));
        s.shutdown();
        QVERIFY(s.wait(3000));
        w.setContext(1, Version::V2, 0, true, false);
        QVERIFY(!read->isEnabled());
        s.open({});
        QTRY_VERIFY(s.telemetryFresh(1));
        w.setContext(1, Version::V2, 0, true, false);
        QTRY_COMPARE(countReplies(0xb5), 3);
        QTest::qWait(600);
        QCOMPARE(countReplies(0xb5), 3);
        QCOMPARE(countReplies(0xb6), 1);
        s.shutdown();
        QVERIFY(s.wait(3000));
    }
    void failedInitialReadDoesNotRetry() {
        Session s;
        MotionControl w(s);
        QSignalSpy logs(&s, &Session::log), failures(&s, &Session::commandFailed);
        auto sent = [&](int command) {
            int count = 0;
            for (const auto &row : logs) {
                auto bytes = row[1].toString().section(" · ", 1).split(' ');
                if (row[0].toString() == "TX" && bytes.value(1).toInt(nullptr, 16) == command)
                    ++count;
            }
            return count;
        };
        w.setContext(2, Version::V2, 0, true, false); // Demo has only Node 1.
        s.open({});
        QTRY_VERIFY(s.connected());
        w.setContext(2, Version::V2, 0, true, false);
        QTRY_COMPARE(sent(0xb5), 1);
        QTRY_COMPARE_WITH_TIMEOUT(failures.size(), 1, 4000);
        for (int i = 0; i < 105; ++i) {
            w.setContext(2, Version::V2, 0, true, false);
            QTest::qWait(100);
        }
        QCOMPARE(sent(0xb5), 1); // Also past the old 10-second automatic retry interval.
        QCOMPARE(sent(0xb6), 0);
        child<QPushButton>(w, "readSpeedLimits")->click();
        QTRY_COMPARE(sent(0xb5), 2);
        s.shutdown();
        QVERIFY(s.wait(3000));
    }
    void legacyTorqueLimiting() {
        Session s;
        Connection c;
        c.version = Version::V1;
        s.open(c);
        QTRY_VERIFY(s.telemetryFresh(1));
        s.command(1, 0x41, QByteArray(1, 2));
        QTRY_COMPARE(s.latest(1)->mode, 2);
        s.command(1, 0x40, QByteArray(1, 1));
        QTRY_VERIFY(s.latest(1)->enabled);
        s.command(1, 0x23, combinedTarget(1, 0x23, 0, 52, .5, Version::V1));
        MotionControl w(s);
        w.show();
        w.setContext(1, Version::V1, 2, true, false);
        QVERIFY(child<QDoubleSpinBox>(w, "motionTarget1")->isEnabled());
        QVERIFY(child<QDoubleSpinBox>(w, "motionTarget2")->isEnabled());
        QTRY_VERIFY(child<QLabel>(w, "limitingState")->text().contains("正在介入"));
        QTRY_VERIFY(s.latest(1)->values[0] < .2);
        s.shutdown();
        QVERIFY(s.wait(3000));
    }
};
QTEST_MAIN(MotionTests)
#include "motion_tests.moc"
