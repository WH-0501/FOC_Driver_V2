#include "sampling.h"
#include <QFile>
#include <QTemporaryDir>
#include <QtTest>
#include <cmath>
using namespace mt;
class SamplingTests : public QObject {
    Q_OBJECT
    static Sample sample(int i) {
        Sample s;
        s.time = i * .001 + (i >= 300 ? 2 : 0);
        s.node = 1;
        for (int c = 0; c < channelCount; ++c)
            s.values[c] = c == 8 ? NAN : std::sin(i * .13 + c);
        if (i % 137 == 0)
            s.values[0] = 1234;
        if (i % 181 == 0)
            s.values[1] = -5678;
        return s;
    }
    static void compare(const Sampling &sampling, double start, double end, int pixels) {
        auto raw = sampling.snapshot();
        auto frame = sampling.envelope(start, end, pixels);
        QCOMPARE(frame.bins.size(), size_t(pixels));
        size_t total = 0;
        for (int x = 0; x < pixels; ++x) {
            double left = start + (end - start) * x / pixels;
            double right = start + (end - start) * (x + 1) / pixels;
            SampleBounds expected;
            size_t count = 0;
            for (const auto &s : raw)
                if (s.time >= left && (x + 1 == pixels ? s.time <= right : s.time < right)) {
                    expected.add(s);
                    ++count;
                }
            QCOMPARE(frame.bins[size_t(x)].count, count);
            for (int c = 0; c < channelCount; ++c) {
                QCOMPARE(frame.bins[size_t(x)].values.low[c], expected.low[c]);
                QCOMPARE(frame.bins[size_t(x)].values.high[c], expected.high[c]);
            }
            total += count;
        }
        QCOMPARE(frame.points, total);
    }
  private slots:
    void wrapAndPartialBlocks() {
        for (size_t capacity : {1, 63, 64, 65, 100, 257, 4096}) {
            Sampling sampling(capacity);
            for (int i = 0; i < int(capacity * 3 + 197); ++i) {
                sampling.append(sample(i));
                if (i % 113 == 0) {
                    auto info = sampling.info();
                    compare(sampling, info.first - .001, info.last + .001, 17);
                    compare(sampling, std::max(info.first, info.last - .0513), info.last + .001, 7);
                }
            }
            QCOMPARE(sampling.info().size, capacity);
            QCOMPARE(sampling.overwritten(), quint64(capacity * 2 + 197));
        }
    }
    void peaksGapsAndReset() {
        Sampling sampling(10000);
        for (int i = 0; i < 6000; ++i)
            sampling.append(sample(i));
        compare(sampling, -.001, 8, 1);
        compare(sampling, 0, 8, 1024);
        compare(sampling, 1, 2, 100); // Empty gap must not invent samples.
        auto data = sampling.range(2.3, 2.4, 16);
        QCOMPARE(data.size(), size_t(16));
        QCOMPARE(data.back().time, 2.4);
        auto nearest = sampling.nearest(2.3012);
        QVERIFY(nearest);
        QCOMPARE(nearest->time, sample(301).time);
        QVERIFY(!sampling.nearest(-1));
        auto generation = sampling.info().generation;
        sampling.clear();
        QVERIFY(sampling.info().generation > generation);
        QCOMPARE(sampling.info().size, size_t(0));
        sampling.append(sample(0));
        compare(sampling, -.001, .001, 8);
        sampling.configure(65);
        QCOMPARE(sampling.info().capacity, size_t(65));
        QCOMPARE(sampling.info().size, size_t(0));
    }
    void millionPointEnvelope() {
        Sampling sampling(1000000);
        for (int i = 0; i < 1000000; ++i)
            sampling.append(sample(i));
        auto frame = sampling.envelope(0, 1002, 1920);
        QCOMPARE(frame.points, size_t(1000000));
        QCOMPARE(frame.bins.size(), size_t(1920));
        double high = -INFINITY, low = INFINITY;
        for (auto &bin : frame.bins) {
            high = std::max(high, bin.values.high[0]);
            low = std::min(low, bin.values.low[1]);
        }
        QCOMPARE(high, 1234.0);
        QCOMPARE(low, -5678.0);
        QCOMPARE(sampling.range(997, 1002, 16384).size(), size_t(5000));
        QCOMPARE(sampling.info().size, size_t(1000000));
    }
    void fullResolutionExport() {
        Sampling sampling(1000);
        for (int i = 0; i < 1500; ++i)
            sampling.append(sample(i));
        QTemporaryDir dir;
        QString error;
        QString path = dir.path() + "/samples.csv";
        QVERIFY2(sampling.exportHistory(path, 1, &error), qPrintable(error));
        QFile file(path);
        QVERIFY(file.open(QIODevice::ReadOnly));
        auto rows = file.readAll().trimmed().split('\n');
        QCOMPARE(rows.size(), 1001);
        QVERIFY(rows[0].startsWith("time_s,node,iq_A,id_A,ia_A,ib_A,ic_A,velocity_rpm"));
        QCOMPARE(rows[1].split(',')[0].toDouble(), 2.5);
        QCOMPARE(rows.last().split(',')[0].toDouble(), 3.499);
        QCOMPARE(rows[1].split(',').size(), 17);
        QCOMPARE(rows[0].split(',')[14], QByteArray("target_position_rad"));
        QCOMPARE(rows[0].split(',')[16], QByteArray("target_current_A"));
        QCOMPARE(sampling.info().size, size_t(1000));
        file.close();
        std::atomic<bool> cancel{true};
        QVERIFY(!sampling.exportHistory(path, 1, &error, &cancel));
        QVERIFY(file.open(QIODevice::ReadOnly));
        QCOMPARE(file.readAll().trimmed().split('\n').size(),
                 1001); // Cancel keeps previous file intact.
    }
};
QTEST_GUILESS_MAIN(SamplingTests)
#include "sampling_tests.moc"
