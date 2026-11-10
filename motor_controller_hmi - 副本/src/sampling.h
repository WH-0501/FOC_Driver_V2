#pragma once
#include "protocol.h"
#include <QJsonArray>
#include <QMutex>
#include <atomic>
#include <deque>
#include <limits>
#include <memory>
namespace mt {
struct TargetEvent {
    quint64 token = 0;
    quint32 node = 1;
    double queued = 0, sent = -1, ack = -1;
    std::array<double, 3> values{};
    int state = 0; // queued, submitted, acknowledged, rejected/cancelled
    QString reason;
};
struct Trigger {
    int channel = 0;
    double level = 0.1, pre = 1, post = 1;
    int edge = 1;
};
struct SampleBounds {
    std::array<double, channelCount> low, high;
    SampleBounds() {
        low.fill(std::numeric_limits<double>::infinity());
        high.fill(-std::numeric_limits<double>::infinity());
    }
    void add(const Sample &);
    void merge(const SampleBounds &);
};
struct HistoryInfo {
    size_t size = 0, capacity = 0;
    quint64 revision = 0, generation = 0;
    double first = 0, last = 0;
};
struct Envelope {
    SampleBounds values;
    size_t count = 0;
};
struct PlotFrame {
    std::vector<Envelope> bins;
    double start = 0, end = 0, typicalDt = .001;
    size_t points = 0;
};
class Sampling {
  public:
    explicit Sampling(size_t capacity = 1000000);
    std::shared_ptr<Sampling> frozenCopy() const;
    void configure(size_t capacity);
    void append(const Sample &);
    void recordTargets(quint64 token, double queued, const std::vector<Sample> &targets);
    void updateTargetEvent(quint64 token, int state, double time, const QString &reason = {});
    std::vector<TargetEvent> targetEvents() const;
    HistoryInfo info() const;
    quint64 captureRevision(int source) const;
    PlotFrame envelope(double start, double end, int pixels) const;
    std::vector<Sample> range(double start, double end, size_t maxPoints = SIZE_MAX) const;
    std::optional<Sample> nearest(double time) const;
    bool exportHistory(const QString &, quint32 node, QString *error = nullptr,
                       const std::atomic<bool> *cancel = nullptr) const;
    std::vector<Sample> snapshot(quint32 node = 0) const;
    std::vector<Sample> capture() const;
    std::vector<Sample> faultCapture() const;
    void pause(bool);
    void clear();
    void arm(Trigger);
    QString triggerState() const;
    quint64 overwritten() const;

  private:
    mutable QMutex mutex_;
    size_t capacity_;
    static constexpr size_t blockSize_ = 64;
    size_t leaves_ = 1;
    std::vector<SampleBounds> tree_;
    quint64 firstSequence_ = 0, nextSequence_ = 0, revision_ = 0, generation_ = 0;
    quint64 captureRevision_ = 0, faultRevision_ = 0;
    SampleBounds rangeBounds(quint64 begin, quint64 end) const;
    std::deque<Sample> ring_, pre_, faultPre_;
    std::deque<TargetEvent> events_;
    std::vector<Sample> triggered_, fault_;
    bool paused_ = false, armed_ = false, fired_ = false, capturingFault_ = false;
    double end_ = 0, faultEnd_ = 0;
    Trigger trigger_;
    quint64 dropped_ = 0;
    std::optional<Sample> previous_;
};
struct Stats {
    double mean = 0, rms = 0, min = 0, max = 0, dt = 0;
    size_t count = 0;
};
Stats statistics(const std::vector<Sample> &, int channel, double t1 = -1e100, double t2 = 1e100);
std::vector<std::pair<double, double>> spectrum(const std::vector<Sample> &, int channel);
QJsonObject stepResponse(const std::vector<Sample> &, int channel, double target, bool automatic = false);
QJsonArray stepResponses(const std::vector<Sample> &, int channel, double crossingHoldSeconds = .002,
                         bool requireDebugPlateau = false);
bool exportCsv(const QString &, const std::vector<Sample> &, QString *error = nullptr,
               const std::atomic<bool> *cancel = nullptr);
} // namespace mt

namespace mt {
QJsonArray hostStepResponses(const std::vector<Sample> &, const std::vector<TargetEvent> &, int channel,
                             double hold);
bool exportTargetEvents(const QString &, const std::vector<TargetEvent> &, QString *error);
} // namespace mt
