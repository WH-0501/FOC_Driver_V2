#include "archive.h"
#include "protocol.h"
#include "sampling.h"
#include "session.h"
#include <QJsonArray>
#include <QTemporaryDir>
#include <QtEndian>
#include <QtTest>
#include <cmath>
using namespace mt;
class CoreTests : public QObject {
    Q_OBJECT
  private slots:
    void fftKeepsAcquisitionClockWhenHostBatchesFrames() {
        std::vector<Sample> samples;
        for (int i = 0; i < 1024; ++i) {
            Sample s;
            s.time = (i / 10) * .01;
            s.acquisitionTime = i * .001;
            s.values[0] = sin(2 * 3.141592653589793 * 125 * s.acquisitionTime);
            samples.push_back(s);
        }
        auto fft = spectrum(samples, 0);
        QVERIFY(!fft.empty());
        auto peak =
            std::max_element(fft.begin(), fft.end(), [](auto a, auto b) { return a.second < b.second; });
        QVERIFY(std::abs(peak->first - 125) < .01);
    }
    void hostEventsUseExactQueueTimeAndPreserveLifecycle() {
        Sampling history(200);
        history.recordTargets(1, .1, decodeTargets(3, target(1, 0, 2, Version::V2), Version::V2));
        history.updateTargetEvent(1, 1, .101);
        history.recordTargets(2, .505, decodeTargets(3, target(1, .2, 2, Version::V2), Version::V2));
        history.updateTargetEvent(2, 1, .506);
        history.updateTargetEvent(2, 2, .507);
        history.recordTargets(3, .7, decodeTargets(3, target(1, -.2, 2, Version::V2), Version::V2));
        history.updateTargetEvent(3, 3, .701, "cancelled");
        for (int i = 0; i < 150; ++i) {
            Sample sample;
            sample.node = 1;
            sample.time = i * .01;
            sample.values[0] = i < 51 ? 0 : .2;
            history.append(sample);
        }
        auto frozen = history.frozenCopy();
        auto rows = hostStepResponses(history.snapshot(), history.targetEvents(), 0, .002);
        QVERIFY(!rows.empty());
        auto row = rows.last().toObject();
        QCOMPARE(row["step_time_s"].toDouble(), .505);
        QCOMPARE(row["first_feedback_after_event_s"].toDouble(), .51);
        QCOMPARE(row["request_token"].toString(), QString("2"));
        QVERIFY(row["source"].toString().contains("端到端"));
        history.clear();
        QVERIFY(history.targetEvents().empty());
        QCOMPARE(frozen->targetEvents().size(), size_t(3));
        QCOMPARE(frozen->targetEvents()[1].state, 2);
        QCOMPARE(frozen->targetEvents()[2].state, 3);
        QTemporaryDir dir;
        QString error;
        QVERIFY(exportTargetEvents(dir.filePath("events.json"), frozen->targetEvents(), &error));
        QFile file(dir.filePath("events.json"));
        QVERIFY(file.open(QIODevice::ReadOnly));
        QVERIFY(file.readAll().contains("rejected_or_cancelled"));
    }
    void debugDwtWrapPreservesTime() {
        DebugClock clock;
        bool restarted;
        QCOMPARE(*clock.advance(23860000, restarted, 1), 0.0);
        auto wrapped = clock.advance(100, restarted, 1.00103);
        QVERIFY(wrapped);
        QVERIFY(!restarted);
        QVERIFY(std::abs(*wrapped - (4294967296.0 / 180 - 23860000 + 100) / 1e6) < 1e-9);
        QVERIFY(!clock.advance(23860900, restarted, 1.0011)); // delayed pre-wrap frame
        auto next = clock.advance(1100, restarted, 1.00203);
        QVERIFY(next && *next > *wrapped);
        QVERIFY(std::abs(*next - *wrapped - .001) < 1e-9);
        QVERIFY(!clock.advance(900, restarted, 1.0021)); // reordering
        QVERIFY(!restarted);
        DebugClock reset;
        reset.advance(5000000, restarted, 1);
        auto before = reset.advance(5001000, restarted, 1.001);
        auto after = reset.advance(100, restarted, 1.002);
        QVERIFY(restarted);
        QVERIFY(after && *after > *before);
        auto continued = reset.advance(1100, restarted, 1.003);
        QVERIFY(continued && *continued > *after);
        QVERIFY(!restarted);
        DebugClock continuous;
        double last = -1;
        for (quint64 i = 0; i < 100000; ++i) {
            quint32 cycles = quint32(i * 180000);
            quint32 tick = quint32(float(cycles) * (1.0f / 180));
            auto t = continuous.advance(tick, restarted, i * .001);
            QVERIFY(t && *t > last);
            QVERIFY(!restarted);
            last = *t;
        }
        QVERIFY(std::abs(last - 99.999) < .00001);
        // Endpoints near a DWT wrap after a long outage are not a proven wrap.
        DebugClock gap;
        gap.advance(23860000, restarted, 1);
        QVERIFY(gap.advance(100, restarted, 30));
        QVERIFY(restarted);
    }
    void clippedDebugReferenceIsNotAStep() {
        std::vector<Sample> data;
        for (int i = 0; i < 500; ++i) {
            Sample v;
            v.debugLoop = 1;
            v.enabled = true;
            v.time = i * .001;
            // Variable reference repeatedly reaches a constant current limit.
            v.values[11] = (i % 20 < 10) ? -.2 : -.15 + (i % 10) * .001;
            v.values[0] = v.values[11];
            data.push_back(v);
        }
        QVERIFY(stepResponses(data, 0, .002, true).empty());
        for (int i = 0; i < 500; ++i)
            data[i].values[0] = data[i].values[11] = i < 150 ? .2 : i < 350 ? -.2 : 0;
        auto rows = stepResponses(data, 0, .002, true);
        QCOMPARE(rows.size(), 2);
        QCOMPARE(rows[0].toObject()["previous_target"].toDouble(), .2);
        QCOMPARE(rows[0].toObject()["target"].toDouble(), -.2);
        QVERIFY(rows[0].toObject().contains("initial"));
    }
    void debugWireAndDeviceClock() {
        QByteArray payload(62, 0);
        payload[0] = 1;
        payload[1] = 3;
        qToBigEndian<quint32>(0xffffff00U, payload.data() + 2);
        for (int i = 0; i < 14; ++i) {
            float value = float(i + 1) / 10;
            quint32 bits;
            memcpy(&bits, &value, 4);
            qToBigEndian<quint32>(bits, payload.data() + 6 + i * 4);
        }
        auto sample = decodeDebug({7, pack(1, 0xb9, payload), 99});
        QVERIFY(sample);
        QCOMPARE(sample->node, quint32(7));
        QCOMPARE(sample->debugLoop, quint8(1));
        QVERIFY(sample->enabled);
        QVERIFY(std::abs(sample->values[11] - .1) < 1e-6);
        QVERIFY(std::abs(sample->values[0] - .2) < 1e-6);
        QVERIFY(std::abs(sample->values[14] - 1.4) < 1e-6);
        QVERIFY(std::abs(sample->values[16] - 11000) < .01);
        QCOMPARE(sample->values[18], double(0xffffff00U));
        QVERIFY(debugChannelNames(1)[0].contains("rad/s"));
        QCOMPARE(debugChannelNames(0).size(), channelCount);
        payload[1] = 11; // active, ramp and upper-layer command reference
        auto commanded = decodeDebug({7, pack(1, 0xb9, payload), 99});
        QVERIFY(commanded);
        QVERIFY(std::abs(commanded->values[11] - .1) < 1e-6);
        QVERIFY(std::isnan(commanded->values[15]));
        payload[0] = 5;
        qToBigEndian<quint16>(1234, payload.data() + 42);
        qToBigEndian<quint16>(99, payload.data() + 52);
        qToBigEndian<quint16>(4, payload.data() + 54);
        qToBigEndian<quint16>(22, payload.data() + 56);
        auto timed = decodeDebug({7, pack(1, 0xb9, payload), 99});
        QVERIFY(timed);
        QCOMPARE(timed->values[20], 1234.0);
        QCOMPARE(timed->values[26], 4.0);
        QCOMPARE(timed->values[27], 22.0);
        qToBigEndian<quint32>(0x7fc00000U, payload.data() + 6);
        QVERIFY(!decodeDebug({7, pack(1, 0xba, payload), 99}));
        QVERIFY(!response(0xba, payload, Version::V2)["valid"].toBool());
        DebugClock clock;
        bool restarted;
        QCOMPARE(*clock.advance(0xfffffff0U, restarted), 0.0);
        QVERIFY(!clock.advance(0xfffffff0U, restarted));
        QVERIFY(std::abs(*clock.advance(0x20, restarted) - .000048) < 1e-12);
        QVERIFY(!restarted);
        QVERIFY(!clock.advance(0x10, restarted));
        QVERIFY(!restarted);
    }
    void debugStepUsesDeviceReferenceAndUnits() {
        std::vector<Sample> data;
        for (int i = 0; i < 300; ++i) {
            Sample s;
            s.debugLoop = 2;
            s.streamEpoch = 1;
            s.enabled = true;
            s.time = i * .01;
            s.values[11] = i < 50 ? 0 : 2;
            s.values[0] = i < 50 ? 0 : 2 * (1 - std::exp(-(i - 50) * .01 / .1));
            data.push_back(s);
        }
        auto rows = stepResponses(data, 0);
        QCOMPARE(rows.size(), 1);
        auto o = rows[0].toObject();
        QCOMPARE(o["target"].toDouble(), 2.0);
        QVERIFY(o["source"].toString().contains("设备"));
        QVERIFY(o["channel"].toString().contains("rad/s"));
        QVERIFY(std::abs(o["sample_interval_ms"].toDouble() - 10) < 1e-9);
        QVERIFY(o.contains("rise_seconds"));
        QTemporaryDir dir;
        QString path = dir.filePath("debug.csv"), error;
        QVERIFY(exportCsv(path, data, &error));
        QFile f(path);
        QVERIFY(f.open(QIODevice::ReadOnly));
        auto lines = f.readAll().split('\n');
        QVERIFY(lines[0].contains("device_time_s"));
        QCOMPARE(lines[0].split(',').size(), 32);
        QCOMPARE(lines[1].split(',').size(), 32);
        std::atomic<bool> cancelled{true};
        QVERIFY(!exportCsv(dir.filePath("cancelled.csv"), data, &error, &cancelled));
        QVERIFY(!QFile::exists(dir.filePath("cancelled.csv")));
        auto ramp = data;
        for (size_t i = 0; i < ramp.size(); ++i)
            ramp[i].values[11] = i * .001;
        QVERIFY(stepResponses(ramp, 0, .002, true).isEmpty());
        for (auto &s : data)
            s.enabled = false;
        QVERIFY(!stepResponses(data, 0)[0].toObject().contains("rise_seconds"));
    }
    void allStepsAreIsolatedAndSelectable() {
        std::vector<Sample> data;
        double actual = 0;
        for (int i = 0; i < 2000; ++i) {
            Sample s;
            s.time = i * .001;
            double goal = i < 300 ? 0 : i < 600 ? .2 : i < 900 ? 0 : i < 1200 ? -.2 : 0;
            s.values[11] = goal;
            actual += (goal - actual) * .05;
            s.values[0] = actual + .0001 * std::sin(i);
            data.push_back(s);
        }
        auto rows = stepResponses(data, 0);
        QCOMPARE(rows.size(), 4);
        QCOMPARE(rows[0].toObject()["target"].toDouble(), .2);
        QCOMPARE(rows[1].toObject()["target"].toDouble(), 0.0);
        QCOMPARE(rows[2].toObject()["target"].toDouble(), -.2);
        QCOMPARE(rows[3].toObject()["target"].toDouble(), 0.0);
        for (int i = 0; i < rows.size(); ++i) {
            auto row = rows[i].toObject();
            QCOMPARE(row["step_time_s"].toDouble(), (i + 1) * .3);
            QVERIFY(row.contains("first_rise_ms"));
            QVERIFY(row.contains("qualified_rise_ms"));
            QVERIFY(row["first_rise_ms"].toDouble() > 35);
            QVERIFY(row["first_rise_ms"].toDouble() < 50);
            QVERIFY(row["first90_time_s"].toDouble() <= row["window_end_s"].toDouble());
        }
        auto first = rows[0].toObject();
        for (size_t i = 600; i < data.size(); ++i)
            data[i].values[0] = 100;
        QCOMPARE(stepResponses(data, 0)[0].toObject(),
                 first); // Later motor action cannot corrupt prior result.
        auto shortData = std::vector<Sample>(data.begin(), data.begin() + 903);
        rows = stepResponses(shortData, 0);
        QCOMPARE(rows.size(), 3);
        QVERIFY(rows[2].toObject()["status"].toString().contains("不足"));
        QVERIFY(!rows[2].toObject().contains("first_rise_ms"));
    }
    void detectRecordedSteps() {
        for (double goal : {.2, -.2}) {
            std::vector<Sample> data;
            for (int i = 0; i < 1500; ++i) {
                Sample s;
                s.time = i * .001;
                s.values[11] = i < 300 ? 0 : goal;
                s.values[0] = i < 300 ? 0 : goal * (1 - std::exp(-(i - 300) * .001 / .05));
                s.values[0] += .0001 * std::sin(i);
                if (i == 310)
                    s.values[0] += 2; // Single spike must not fake a rise crossing.
                data.push_back(s);
            }
            auto result = stepResponse(data, 0, 1, true);
            QCOMPARE(result["target"].toDouble(), goal); // Ignores old UI default 1.0.
            QCOMPARE(result["step_time_s"].toDouble(), .3);
            QVERIFY(std::abs(result["rise_seconds"].toDouble() - .05 * std::log(9)) < .006);
            QVERIFY(result.contains("settling_seconds"));
            QVERIFY(result["settling_seconds"].toDouble() < .22);
            QCOMPARE(stepResponse(data, 11, 1, true)["target"].toDouble(), goal);
            // Most recent falling/rising edge is measured from its own baseline.
            for (int i = 1000; i < 1500; ++i) {
                data[i].values[11] = 0;
                data[i].values[0] = goal * std::exp(-(i - 1000) * .001 / .05);
            }
            result = stepResponse(data, 0, 1, true);
            QCOMPARE(result["step_time_s"].toDouble(), 1.0);
            QCOMPARE(result["target"].toDouble(), 0.0);
            QVERIFY(std::abs(result["rise_seconds"].toDouble() - .05 * std::log(9)) < .006);
        }
    }
    void stepFailuresAreExplicit() {
        std::vector<Sample> data(200);
        for (int i = 0; i < 200; ++i) {
            data[i].time = i * .001;
            data[i].values[11] = 0;
        }
        QVERIFY(stepResponse(data, 0, 1, true)["status"].toString().contains("没有目标阶跃"));
        for (int i = 100; i < 200; ++i)
            data[i].values[11] = .2;
        auto result = stepResponse(data, 0, 1, true);
        QVERIFY(result.contains("rise_status")); // Target exists but motor did not respond.
        QVERIFY(!result.contains("rise_seconds"));
        QVERIFY(!result.contains("settling_seconds"));
        data[120].values[0] = NAN;
        QVERIFY(stepResponse(data, 0, 1, true)["status"].toString().contains("缺失数据"));
        data[120].values[0] = 0;
        for (int i = 120; i < 200; ++i)
            data[i].time += 1;
        QVERIFY(stepResponse(data, 0, 1, true)["status"].toString().contains("采样中断"));
        QVERIFY(stepResponse(data, 7, 1, true)["status"].toString().contains("没有对应目标"));
    }
    void targetReferences() {
        Sample empty;
        QVERIFY(std::isnan(empty.values[9]));
        for (auto version : {Version::V1, Version::V2}) {
            auto payload = combinedTarget(2, 0x30, 1, -23, -.125, version);
            auto refs = decodeTargets(0x30, payload, version);
            QCOMPARE(refs.size(), size_t(1));
            QCOMPARE(refs[0].node, quint32(2));
            QVERIFY(std::abs(refs[0].values[9] - 1) < .004);
            QCOMPARE(refs[0].values[10], -23.0);
            QCOMPARE(refs[0].values[11], -.125);
        }
        auto refs =
            decodeTargets(3, target(1, .3, 2, Version::V2) + target(2, -.4, 2, Version::V2), Version::V2);
        QCOMPARE(refs.size(), size_t(2));
        QVERIFY(std::isnan(refs[0].values[9]));
        QCOMPARE(refs[0].values[11], .3);
        QCOMPARE(refs[1].values[11], -.4);
        QVERIFY(decodeTargets(0x40, QByteArray(1, 1), Version::V2).empty());
        QVERIFY(decodeTargets(3, QByteArray(2, 1), Version::V2).empty());
        Sampling history;
        refs[0].time = 1;
        history.append(refs[0]);
        refs[0].time = 2;
        refs[0].values[11] = -2;
        history.append(refs[0]);
        auto frame = history.envelope(0, 3, 1);
        QCOMPARE(frame.bins[0].values.low[11], -2.0);
        QCOMPARE(frame.bins[0].values.high[11], .3);
    }
    void wireVectors() {
        QCOMPARE(pack(7, 0x42, pid(1, .5, 0)).toHex(), QByteArray("0742100008000000"));
        QCOMPARE(target(3, 1, 0, Version::V2).toHex(), QByteArray("030080"));
        QCOMPARE(target(3, std::numbers::pi, 0, Version::V1).toHex(), QByteArray("035a00"));
        QVERIFY_EXCEPTION_THROWN(target(1, 1e10, 0, Version::V2), std::out_of_range);
        QVERIFY_EXCEPTION_THROWN(pid(-1, 0, 0), std::out_of_range);
        QVERIFY_EXCEPTION_THROWN(pack(0, 1, QByteArray(63, 0)), std::invalid_argument);
    }
    void telemetry() {
        QByteArray b = be16(-128) + be16(-50) + be16(250) + be16(-125) + be16(100) + be16(-50) + be16(-50) +
                       QByteArray::fromHex("25002aef");
        auto s = decode({7, pack(1, 0, b), 1.234}, Version::V2);
        QVERIFY(s);
        QCOMPARE(s->values[6], -1.0);
        QCOMPARE(s->values[1], -.125);
        QCOMPARE(s->values[7], 23.9);
        QCOMPARE(s->values[8], 42.0);
        QVERIFY(s->enabled);
        QCOMPARE(s->mode, 5);
        QByteArray legacy = be16(128) + be16(3) + be16(-250) + QByteArray::fromHex("2400");
        s = decode({2, pack(0, 0, legacy), 0}, Version::V1);
        QVERIFY(s);
        QVERIFY(!s->enabled);
        QVERIFY(!s->extended);
        QVERIFY(std::isnan(s->values[8]));
        QVERIFY(std::abs(s->values[6] - std::numbers::pi / 180) < 1e-12);
        QVERIFY(!decode({1, QByteArray(3, 0), 0}, Version::V2));
    }
    void ackNotSuccess() {
        auto o = response(0x7e, QByteArray::fromHex("014d"), Version::V2);
        QVERIFY(!o.contains("ok"));
        QCOMPARE(o["dispatched_command"].toInt(), 0x4d);
        o = response(0x4d, QByteArray(1, 3), Version::V2);
        QVERIFY(!o["ok"].toBool());
        QVERIFY(requiresDisabled(0xab));
        QVERIFY(requiresDisabled(0x54));
    }
    void firmwareResponseSemantics() {
        QVERIFY(response(0x4d, QByteArray(1, 1), Version::V2)["ok"].toBool());
        QVERIFY(!response(0x4d, QByteArray(1, 0), Version::V2)["ok"].toBool());
        QVERIFY(response(0x40, QByteArray(1, 0), Version::V2)["ok"].toBool());
        QCOMPARE(responseCommand(0x42), 0x47);
        auto o = response(0x42, pid(1, .5, .25), Version::V1);
        QCOMPARE(o["P"].toDouble(), 1.0);
        QCOMPARE(o["I"].toDouble(), .5);
        QCOMPARE(combinedTarget(1, 0x30, 1, 10, .25, Version::V2).toHex(), QByteArray("010080000a00fa"));
    }
    void demoIap() {
        Session s;
        QSignalSpy finished(&s, &Session::iapFinished);
        s.open({});
        QTRY_VERIFY(s.latest(1).has_value());
        QByteArray body(1024, char(0x5a)), image(32, 0);
        image.replace(0, 4, "ASIX");
        qToLittleEndian<quint16>(3, image.data() + 4);
        qToBigEndian<quint32>(1056, image.data() + 12);
        qToLittleEndian<quint32>(~checksum(body), image.data() + 20);
        image += body;
        s.upgrade(1, image);
        QTRY_VERIFY_WITH_TIMEOUT(finished.size() == 1, 5000);
        QVERIFY(finished[0][0].toBool());
        QVERIFY(!s.upgrading());
        s.shutdown();
        QVERIFY(s.wait(3000));
    }
    void bufferAndTrigger() {
        Sampling b(100);
        b.arm({0, .5, .02, .02, 1});
        for (int i = 0; i < 200; i++) {
            Sample s;
            s.time = i * .001;
            s.values[0] = i < 50 ? 0 : 1;
            b.append(s);
        }
        QCOMPARE(b.snapshot().size(), size_t(100));
        QCOMPARE(b.overwritten(), quint64(100));
        auto cap = b.capture();
        QVERIFY(cap.size() >= 39 && cap.size() <= 42);
        QVERIFY(cap.front().time <= .031);
        QVERIFY(cap.back().time >= .07);
        b.pause(true);
        Sample s;
        s.time = 99;
        b.append(s);
        QVERIFY(b.snapshot().back().time < 1);
    }
    void faultWindow() {
        Sampling b(20000);
        for (int i = 0; i < 12000; i++) {
            Sample s;
            s.time = i * .001;
            s.error = i >= 6000 ? 2 : 0;
            b.append(s);
        }
        auto f = b.faultCapture();
        QVERIFY(f.size() >= 9999 && f.size() <= 10002);
        QVERIFY(std::abs(f.front().time - 1) < .002);
        QVERIFY(std::abs(f.back().time - 11) < .002);
    }
    void fftAndStatistics() {
        std::vector<Sample> a;
        for (int i = 0; i < 4096; i++) {
            Sample s;
            s.time = i / 1024.0;
            s.values[0] = 2 * sin(2 * std::numbers::pi * 32 * s.time);
            a.push_back(s);
        }
        auto stats = statistics(a, 0);
        QVERIFY(std::abs(stats.rms - std::sqrt(2)) < 1e-9);
        auto bins = spectrum(a, 0);
        QVERIFY(!bins.empty());
        auto peak =
            std::max_element(bins.begin(), bins.end(), [](auto &a, auto &b) { return a.second < b.second; });
        QCOMPARE(peak->first, 32.0);
        QVERIFY(std::abs(peak->second - 2) < .002);
        a[3000].time += .01;
        QVERIFY(spectrum(a, 0).empty());
    }
    void imageValidation() {
        QByteArray body(101, char(0xab));
        QByteArray b(32, 0);
        b.replace(0, 4, "ASIX");
        qToLittleEndian<quint16>(3, b.data() + 4);
        qToBigEndian<quint32>(133, b.data() + 12);
        qToLittleEndian<quint32>(~checksum(body), b.data() + 20);
        b += body;
        QCOMPARE(firmwareImage(b), b);
        auto chunks = firmwareChunks(b);
        QCOMPARE(chunks.size(), size_t(3));
        QCOMPARE(chunks.back().size(), 62);
        QCOMPARE(quint8(chunks.back().back()), quint8(255));
        b[40] ^= 1;
        QVERIFY_EXCEPTION_THROWN(firmwareImage(b), std::invalid_argument);
        QCOMPARE(checksum(QByteArray::fromHex("ffffffff01000000")), quint32(1));
    }
    void archiveVersions() {
        QTemporaryDir dir;
        Archive a;
        QString error;
        QVERIFY2(a.open(dir.path() + "/db.sqlite", &error), qPrintable(error));
        QJsonObject o{{"node", 3}, {"53", QJsonObject{{"aligned", true}}}};
        auto id = a.save("SN-1", 3, o, true, &error);
        QVERIFY(id > 0);
        QVERIFY(a.verify(id, o, &error));
        auto second = a.save("SN-1", 3, o, true, &error);
        QVERIFY(second > id);
        auto records = a.search("SN-1");
        QCOMPARE(records.size(), 2);
        QCOMPARE(records[0].toObject()["version"].toInt(), 2);
        o["node"] = 4;
        QVERIFY(!a.verify(second, o, &error));
    }
    void demoSafety() {
        Session s;
        QSignalSpy connection(&s, &Session::connectionChanged), replies(&s, &Session::replied),
            failures(&s, &Session::commandFailed);
        s.open({});
        QTRY_VERIFY_WITH_TIMEOUT(s.connected(), 3000);
        QTRY_VERIFY_WITH_TIMEOUT(s.latest(1).has_value(), 3000);
        s.command(1, 0x40, QByteArray(1, 1));
        QTRY_VERIFY(failures.size() > 0);
        s.command(1, 0x41, QByteArray(1, 6));
        QTRY_VERIFY(s.latest(1)->mode == 6);
        s.command(1, 0x40, QByteArray(1, 1));
        QTRY_VERIFY(s.latest(1)->enabled);
        int n = failures.size();
        s.command(1, 0x4d);
        QTRY_VERIFY(failures.size() > n);
        s.emergency();
        QTRY_VERIFY(!s.latest(1)->enabled);
        QVERIFY(s.rxCount > 0);
        s.shutdown();
        QVERIFY(s.wait(3000));
    }
    void performance() {
        Sampling b;
        QElapsedTimer timer;
        timer.start();
        for (int i = 0; i < 100000; i++) {
            Sample s;
            s.time = i * .001;
            s.values[0] = i % 100;
            b.append(s);
        }
        qInfo() << "100k ingest ms" << timer.elapsed();
        QVERIFY(timer.elapsed() < 5000);
        auto a = b.snapshot();
        QCOMPARE(a.size(), size_t(100000));
    }
};
QTEST_GUILESS_MAIN(CoreTests)
#include "core_tests.moc"
