#include "backend.h"
#include "session.h"
#include <QLibrary>
#include <QtTest>
#include <atomic>
#include <mutex>
using namespace mt;
class NativeBackendTests : public QObject {
    Q_OBJECT
  private slots:
    void commandTimingIndependentOfUiDelivery() {
        Session session;
        Connection c;
        c.backend = "创芯科技 USBCAN-FD (200U)";
        c.driverDirectory = TEST_DRIVER_DIR;
        c.deviceIndex = 2;
        c.channel = "1";
        std::mutex guard;
        QStringList timing;
        qint64 sourceWall = 0;
        connect(
            &session, &Session::sourceLog, &session,
            [&](QString level, QString text, qint64 wall, qint64) {
                if (level == "TIMING") {
                    std::lock_guard lock(guard);
                    timing.append(text);
                    sourceWall = wall;
                }
            },
            Qt::DirectConnection);
        session.open(c);
        QTRY_VERIFY(session.connected());
        auto token = session.command(1, 0xb5);
        // Intentionally block UI event delivery; the TX/RX worker must continue.
        QThread::msleep(80);
        {
            std::lock_guard lock(guard);
            QVERIFY2(timing.size() >= 2, "Command/ACK processing waited for UI events");
            QVERIFY(sourceWall > 0);
            bool matched = false;
            for (const auto &line : timing) {
                qInfo().noquote() << line;
                if (line.contains(QString("请求#%1 ").arg(token)) && line.contains("适配器RX=") &&
                    line.contains("TX→主机取ACK="))
                    matched = true;
            }
            QVERIFY(matched);
        }
        session.shutdown();
        QVERIFY(session.wait(2000));
    }
    void legacyAbiAndIndependentReception() {
        QLibrary dll(QStringLiteral(TEST_DRIVER_DIR "/ControlCANFD.dll"));
        QVERIFY(dll.load());
        auto bad = reinterpret_cast<unsigned (*)()>(dll.resolve("TestBadCalls"));
        auto count = reinterpret_cast<unsigned (*)()>(dll.resolve("TestTxCount"));
        auto captured = reinterpret_cast<void (*)(void *)>(dll.resolve("TestLastTx"));
        auto hold = reinterpret_cast<void (*)(unsigned)>(dll.resolve("TestHoldRx"));
        auto entered = reinterpret_cast<unsigned (*)()>(dll.resolve("TestEnteredRx"));
        QVERIFY(bad && count && captured && hold && entered);
        Connection c;
        c.backend = "创芯科技 USBCAN-FD (200U)";
        c.driverDirectory = TEST_DRIVER_DIR;
        c.deviceIndex = 2;
        c.channel = "1";
        auto backend = createBackend(c);
        std::atomic<unsigned> notifications{0};
        backend->setReceiveNotifier([&] { ++notifications; });
        backend->open(c);
        QCOMPARE(bad(), 0u);
        // Hold a real native ReceiveFD call, then submit on the caller thread.
        hold(1);
        auto releaseRx = qScopeGuard([hold] { hold(0); });
        QTRY_VERIFY(entered());
        QElapsedTimer elapsed;
        elapsed.start();
        backend->send({1, pack(3, 0x40, QByteArray(1, 1)), 0});
        auto submitMs = elapsed.elapsed();
        hold(0); // Always release before assertions/destruction.
        QVERIFY2(submitMs < 100, "RX driver call blocked TX submission");
        QCOMPARE(count(), 1u);
        QByteArray actual(76, 0);
        captured(actual.data());
        // Legacy ctypes ZCAN_TransmitFD_Data: standard ID, BRS, CNT/CMD/01, normal TX.
        QByteArray expected = QByteArray::fromHex("0100000003010000034001");
        expected.resize(76, 0);
        QCOMPARE(actual, expected);
        std::vector<Frame> received;
        QTRY_VERIFY(([&] {
            if (received.empty())
                received = backend->receive();
            return !received.empty();
        })());
        QCOMPARE(received[0].node, quint32(1));
        QCOMPARE(received[0].bytes, QByteArray::fromHex("077e0340"));
        QCOMPARE(received[0].time, 0.0);
        QVERIFY(received[0].adapterRxUs.has_value());
        QVERIFY(received[0].hostRxNs >= received[0].readStartNs);
        QVERIFY(received[0].readStartNs > 0);
        QVERIFY(notifications.load() > 0);
        auto firstStamp = *received[0].adapterRxUs;
        backend->send({1, pack(4, 0xb5), 0});
        received.clear();
        QTRY_VERIFY(([&] {
            if (received.empty())
                received = backend->receive();
            return !received.empty();
        })());
        QCOMPARE(received[0].time, .001);
        QCOMPARE(*received[0].adapterRxUs - firstStamp, quint64(1000));
        QTest::qWait(30);
        QCOMPARE(count(), 2u); // No duplicate/flush/query traffic while idle.
        QVERIFY(received[0].adapterRxValid);
        backend->send({1, pack(0xee, 0xb5), 0});
        received.clear();
        QTRY_VERIFY(([&] {
            if (received.empty())
                received = backend->receive();
            return !received.empty();
        })());
        QVERIFY(!received[0].adapterRxValid);
        QCOMPARE(*received[0].adapterRxUs, quint64(18446743947967255416ull));
        QCOMPARE(received[0].time, 0.0); // Never convert the malformed stamp to a waveform time.
        backend->close();
        QCOMPARE(bad(), 0u);
    }
};
QTEST_GUILESS_MAIN(NativeBackendTests)
#include "native_backend_tests.moc"
