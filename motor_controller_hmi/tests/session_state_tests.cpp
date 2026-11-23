#include "session.h"
#include <QElapsedTimer>
#include <QSignalSpy>
#include <QtTest>
#include <QtEndian>
#include <cmath>
#include <mutex>
using namespace mt;
enum class Scenario { NoResult, AcceptedButEnabled, HoldEnable, NoTelemetry, Occupied, LateLimits, IapReject };
struct ScriptState {
    Scenario scenario = Scenario::NoResult;
    std::atomic<int> sentEnable{0}, sentDisable{0};
    std::atomic<int> sentLimits{0}, sentIapData{0}, sentIapReset{0};
};
class ScriptBackend final : public CanBackend {
    std::shared_ptr<ScriptState> state_;
    QElapsedTimer clock_;
    bool enabled_ = true, telemetry_ = true;
    Version version_ = Version::V2;
    quint8 seq_ = 0;
    std::vector<Frame> replies_;
    std::vector<Frame> later_;
    qint64 laterAt_ = 0;
    quint8 oldRequest_ = 0;

  public:
    explicit ScriptBackend(std::shared_ptr<ScriptState> state) : state_(std::move(state)) {}
    void open(const Connection &c) override {
        if (state_->scenario == Scenario::Occupied)
            throw std::runtime_error("设备已被其他进程占用（模拟）");
        version_ = c.version;
        enabled_ = state_->scenario != Scenario::HoldEnable && state_->scenario != Scenario::IapReject;
        clock_.start();
    }
    void close() override {}
    void send(const Frame &f) override {
        int cmd = quint8(f.bytes[1]);
        if (state_->scenario == Scenario::IapReject) {
            if (cmd == 0xfa) ++state_->sentIapData;
            if (cmd == 0xff) ++state_->sentIapReset;
            if (cmd == 0xf8 || cmd == 0xfc)
                replies_.push_back({1, pack(seq_++, 0xfc, QByteArray::fromHex("0180")), 0});
            if (cmd == 0xf9)
                replies_.push_back({1, pack(seq_++, 0xfc, QByteArray::fromHex("0690")), 0});
            return;
        }
        if (state_->scenario == Scenario::LateLimits && cmd == 0xb5) {
            if (++state_->sentLimits == 1) {
                oldRequest_ = quint8(f.bytes[0]);
                return;
            }
            auto response = [&](int rpm) {
                return pack(seq_++, 0xb5, QByteArray::fromHex("0300003c") + be16(rpm) + QByteArray(14, 0));
            };
            replies_.push_back({1, response(54), 0});
            replies_.push_back(
                {1, pack(seq_++, 0x7e, QByteArray(1, char(oldRequest_)) + QByteArray(1, char(0xb5))), 0});
            later_.push_back({1, response(55), 0});
            later_.push_back({1, pack(seq_++, 0x7e, f.bytes.left(2)), 0});
            laterAt_ = clock_.elapsed() + 300;
            return;
        }
        if (cmd != 0x40)
            return;
        bool requested = f.bytes[2] != 0;
        if (requested)
            ++state_->sentEnable;
        else
            ++state_->sentDisable;
        if (state_->scenario == Scenario::NoResult) {
            enabled_ = requested;
            // Replay the screenshot's delayed *previous command* result/ACK.
            replies_.push_back({1, pack(seq_++, 0x41, QByteArray(1, 0)), 0});
            replies_.push_back(
                {1, pack(seq_++, 0x7e, QByteArray(1, char(quint8(f.bytes[0]) - 1)) + QByteArray(1, 0x41)),
                 0});
            return;
        }
        if (state_->scenario == Scenario::HoldEnable && !requested)
            enabled_ = false;
        if (state_->scenario == Scenario::NoTelemetry)
            telemetry_ = false;
        if (f.node) {
            replies_.push_back({1, pack(seq_++, 0x40, QByteArray(1, 0)), 0});
            replies_.push_back({1, pack(seq_++, 0x7e, f.bytes.left(2)), 0});
        }
    }
    std::vector<Frame> receive() override {
        std::vector<Frame> out;
        out.swap(replies_);
        if (laterAt_ && clock_.elapsed() >= laterAt_) {
            out.insert(out.end(), later_.begin(), later_.end());
            later_.clear();
            laterAt_ = 0;
        }
        if (telemetry_) {
            QByteArray p(6, 0);
            if (version_ == Version::V2)
                p += QByteArray(8, 0);
            p += char((enabled_ ? 0x10 : 0) | 1);
            p += char(0);
            if (version_ == Version::V2)
                p += QByteArray::fromHex("20ef");
            out.push_back({1, pack(seq_++, 0, p), clock_.nsecsElapsed() / 1e9});
        }
        return out;
    }
};
class SessionStateTests : public QObject {
    Q_OBJECT
    static Session::BackendFactory factory(const std::shared_ptr<ScriptState> &state) {
        return [state](const Connection &) { return std::make_unique<ScriptBackend>(state); };
    }
    static QJsonObject result(const QSignalSpy &replies, quint64 token) {
        for (const auto &row : replies)
            if (row[0].toULongLong() == token)
                return row[3].toJsonObject();
        return {};
    }
  private slots:
    void oversizedDeviceReplyStopsUpgrade() {
        auto state = std::make_shared<ScriptState>();
        state->scenario = Scenario::IapReject;
        Session s(nullptr, factory(state));
        QSignalSpy finished(&s, &Session::iapFinished);
        s.open({});
        QTRY_VERIFY(s.telemetryFresh(1));
        QByteArray body(100, char(0xab)), image(32, 0);
        image.replace(0, 4, "ASIX");
        qToLittleEndian<quint16>(3, image.data()+4);
        qToBigEndian<quint32>(32, image.data()+8);
        qToBigEndian<quint32>(132, image.data()+12);
        qToLittleEndian<quint32>(~checksum(body), image.data()+20);
        s.upgrade(1, image+body);
        QTRY_VERIFY(!finished.empty());
        QVERIFY(!finished[0][0].toBool());
        QVERIFY(finished[0][1].toString().contains("非法操作"));
        QVERIFY(finished[0][1].toString().contains("0x90"));
        QTest::qWait(200);
        QCOMPARE(state->sentIapData.load(), 0);
        QCOMPARE(state->sentIapReset.load(), 0);
        s.shutdown(); QVERIFY(s.wait(3000));
    }

    void onlySubmittedTargetsAreRecorded() {
        auto state = std::make_shared<ScriptState>();
        Session s(nullptr, factory(state));
        s.open({});
        QTRY_VERIFY(s.telemetryFresh(1));
        QVERIFY(std::isnan(s.latest(1)->values[11]));
        s.command(1, 3, target(1, .123, 2, Version::V2));
        QTRY_COMPARE(s.latest(1)->values[11], .123);
        QVERIFY(std::isnan(s.latest(1)->values[9]));
        // Torque mode rejects position: rejected local edits must never become
        // references.
        QSignalSpy failures(&s, &Session::commandFailed);
        s.command(1, 1, target(1, 1, 0, Version::V2));
        QTRY_VERIFY(!failures.empty());
        QVERIFY(std::isnan(s.latest(1)->values[9]));
        QCOMPARE(s.sampling.snapshot().back().values[11], .123);
        s.shutdown();
        QVERIFY(s.wait(3000));
        s.open({});
        QTRY_VERIFY(s.telemetryFresh(1));
        QVERIFY(std::isnan(s.latest(1)->values[11]));
        s.shutdown();
        QVERIFY(s.wait(3000));
    }
    void latePreviousLimitsCannotCompleteNewRead() {
        auto state = std::make_shared<ScriptState>();
        state->scenario = Scenario::LateLimits;
        Session s(nullptr, factory(state));
        QSignalSpy replies(&s, &Session::replied), failures(&s, &Session::commandFailed),
            logs(&s, &Session::log);
        s.open({});
        QTRY_VERIFY(s.telemetryFresh(1));
        auto old = s.command(1, 0xb5);
        QTRY_VERIFY_WITH_TIMEOUT(!failures.empty(), 3000);
        QCOMPARE(failures[0][0].toULongLong(), old);
        auto current = s.command(1, 0xb5);
        QTRY_COMPARE(state->sentLimits.load(), 2);
        QTest::qWait(100);
        QVERIFY(result(replies, current).isEmpty());
        QTRY_COMPARE(result(replies, current)["soft_rpm"].toInt(), 55);
        QCOMPARE(state->sentLimits.load(), 2);
        bool lateAck = false;
        for (const auto &row : logs)
            lateAck |= row[1].toString().contains("旧请求的迟到 ACK");
        QVERIFY(lateAck);
        s.shutdown();
        QVERIFY(s.wait(3000));
    }
    void disableWithDelayedPreviousReply_data() {
        QTest::addColumn<bool>("legacy");
        QTest::newRow("V2") << false;
        QTest::newRow("V1") << true;
    }
    void disableWithDelayedPreviousReply() {
        QFETCH(bool, legacy);
        auto state = std::make_shared<ScriptState>();
        Session s(nullptr, factory(state));
        QSignalSpy replies(&s, &Session::replied), failures(&s, &Session::commandFailed);
        Connection config;
        config.version = legacy ? Version::V1 : Version::V2;
        s.open(config);
        QTRY_VERIFY(s.telemetryFresh(1));
        QVERIFY(s.latest(1)->enabled);
        auto token = s.command(1, 0x40, QByteArray(1, 0));
        QTRY_VERIFY(result(replies, token)["telemetry_confirmed"].toBool());
        QVERIFY(result(replies, token)["ok"].toBool());
        QVERIFY(!result(replies, token)["enabled"].toBool());
        QVERIFY(!result(replies, token)["api_reply_received"].toBool());
        QVERIFY(!result(replies, token)["dispatch_ack_received"].toBool());
        QVERIFY(failures.empty());
        s.shutdown();
        QVERIFY(s.wait(3000));
    }
    void invalidEnableValueNeverTransmits() {
        auto state = std::make_shared<ScriptState>();
        Session s(nullptr, factory(state));
        QSignalSpy failures(&s, &Session::commandFailed);
        s.open({});
        QTRY_VERIFY(s.telemetryFresh(1));
        auto token = s.command(1, 0x40, QByteArray(1, 2));
        QTRY_VERIFY(!failures.empty());
        QCOMPARE(failures[0][0].toULongLong(), token);
        QVERIFY(failures[0][1].toString().contains("00 或 01"));
        QCOMPARE(state->sentEnable.load(), 0);
        QCOMPARE(state->sentDisable.load(), 0);
        s.shutdown();
        QVERIFY(s.wait(3000));
    }
    void acceptanceAndAckDoNotProveDisabled() {
        auto state = std::make_shared<ScriptState>();
        state->scenario = Scenario::AcceptedButEnabled;
        Session s(nullptr, factory(state));
        QSignalSpy replies(&s, &Session::replied), failures(&s, &Session::commandFailed);
        s.open({});
        QTRY_VERIFY(s.telemetryFresh(1));
        auto token = s.command(1, 0x40, QByteArray(1, 0));
        QTRY_VERIFY_WITH_TIMEOUT(!failures.empty(), 4000);
        QCOMPARE(failures[0][0].toULongLong(), token);
        QVERIFY(failures[0][1].toString().contains("仍为使能"));
        QVERIFY(result(replies, token).isEmpty());
        QVERIFY(s.latest(1)->enabled);
        s.shutdown();
        QVERIFY(s.wait(3000));
    }
    void disablePreemptsUnconfirmedEnable() {
        auto state = std::make_shared<ScriptState>();
        state->scenario = Scenario::HoldEnable;
        Session s(nullptr, factory(state));
        QSignalSpy replies(&s, &Session::replied), failures(&s, &Session::commandFailed);
        s.open({});
        QTRY_VERIFY(s.telemetryFresh(1));
        auto enable = s.command(1, 0x40, QByteArray(1, 1));
        QTRY_COMPARE(state->sentEnable.load(), 1);
        auto disable = s.command(1, 0x40, QByteArray(1, 0));
        QTRY_VERIFY_WITH_TIMEOUT(result(replies, disable)["telemetry_confirmed"].toBool(), 700);
        QVERIFY(result(replies, enable).isEmpty());
        QTRY_VERIFY(!failures.empty());
        QCOMPARE(failures[0][0].toULongLong(), enable);
        QVERIFY(failures[0][1].toString().contains("取消"));
        s.shutdown();
        QVERIFY(s.wait(3000));
    }
    void lostTelemetryGatesAsDisabled() {
        auto state = std::make_shared<ScriptState>();
        state->scenario = Scenario::NoTelemetry;
        Session s(nullptr, factory(state));
        QSignalSpy replies(&s, &Session::replied), failures(&s, &Session::commandFailed);
        s.open({});
        QTRY_VERIFY(s.telemetryFresh(1));
        auto token = s.command(1, 0x40, QByteArray(1, 0));
        QTRY_VERIFY_WITH_TIMEOUT(!s.telemetryFresh(1), 2000);
        QTRY_VERIFY(!failures.empty());
        QVERIFY(s.latest(1)->enabled);
        QVERIFY(!s.motorEnabled(1));
        QVERIFY(result(replies, token).isEmpty());
        s.shutdown();
        QVERIFY(s.wait(3000));
    }
    void connectionFailureKeepsOriginalReason() {
        auto state = std::make_shared<ScriptState>();
        state->scenario = Scenario::Occupied;
        Session s(nullptr, factory(state));
        QSignalSpy connection(&s, &Session::connectionChanged);
        s.open({});
        QTRY_VERIFY(!connection.empty());
        QVERIFY(s.wait(3000));
        QCoreApplication::processEvents();
        QCOMPARE(connection.size(), 1);
        QVERIFY(!connection[0][0].toBool());
        QVERIFY(connection[0][1].toString().contains("占用"));
        QVERIFY(!s.connected());
    }
};
QTEST_GUILESS_MAIN(SessionStateTests)
#include "session_state_tests.moc"
