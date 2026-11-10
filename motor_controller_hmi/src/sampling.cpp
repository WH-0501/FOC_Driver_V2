#include "sampling.h"
#include <QJsonDocument>
#include <QMutexLocker>
#include <QSaveFile>
#include <QTextStream>
#include <algorithm>
#include <cmath>
#include <complex>
namespace mt {
void SampleBounds::add(const Sample &s) {
    for (int c = 0; c < channelCount; ++c)
        if (std::isfinite(s.values[c])) {
            low[c] = std::min(low[c], s.values[c]);
            high[c] = std::max(high[c], s.values[c]);
        }
}
void SampleBounds::merge(const SampleBounds &other) {
    for (int c = 0; c < channelCount; ++c) {
        low[c] = std::min(low[c], other.low[c]);
        high[c] = std::max(high[c], other.high[c]);
    }
}
Sampling::Sampling(size_t capacity) {
    configure(capacity);
}
std::shared_ptr<Sampling> Sampling::frozenCopy() const {
    auto copy = std::make_shared<Sampling>(1);
    QMutexLocker lock(&mutex_);
    copy->capacity_ = capacity_;
    copy->leaves_ = leaves_;
    copy->tree_ = tree_;
    copy->ring_ = ring_;
    copy->events_ = events_;
    copy->firstSequence_ = firstSequence_;
    copy->nextSequence_ = nextSequence_;
    copy->revision_ = revision_;
    copy->generation_ = generation_;
    return copy;
}
void Sampling::recordTargets(quint64 token, double queued, const std::vector<Sample> &targets) {
    if (targets.empty())
        return;
    QMutexLocker lock(&mutex_);
    for (const auto &target : targets) {
        TargetEvent e;
        e.token = token;
        e.node = target.node;
        e.queued = queued;
        for (int c = 0; c < 3; ++c)
            e.values[c] = target.values[9 + c];
        events_.push_back(e);
    }
    while (events_.size() > 10000)
        events_.pop_front();
    ++revision_;
}
void Sampling::updateTargetEvent(quint64 token, int state, double time, const QString &reason) {
    QMutexLocker lock(&mutex_);
    for (auto &e : events_)
        if (e.token == token) {
            e.state = state;
            e.reason = reason;
            if (state == 1)
                e.sent = time;
            if (state == 2)
                e.ack = time;
            ++revision_;
        }
}
std::vector<TargetEvent> Sampling::targetEvents() const {
    QMutexLocker lock(&mutex_);
    return {events_.begin(), events_.end()};
}
void Sampling::configure(size_t capacity) {
    if (!capacity || capacity > 10000000)
        throw std::invalid_argument("history capacity outside 1..10000000");
    size_t leaves = 1;
    while (leaves < (capacity + blockSize_ - 1) / blockSize_ + 1)
        leaves *= 2;
    std::vector<SampleBounds> tree(leaves * 2);
    clear();
    QMutexLocker lock(&mutex_);
    capacity_ = capacity;
    leaves_ = leaves;
    tree_ = std::move(tree);
}
HistoryInfo Sampling::info() const {
    QMutexLocker lock(&mutex_);
    return {ring_.size(),
            capacity_,
            revision_,
            generation_,
            ring_.empty() ? 0 : ring_.front().time,
            ring_.empty() ? 0 : ring_.back().time};
}
quint64 Sampling::captureRevision(int source) const {
    QMutexLocker lock(&mutex_);
    return source == 1 ? captureRevision_ : faultRevision_;
}
SampleBounds Sampling::rangeBounds(quint64 begin, quint64 end) const {
    SampleBounds out;
    while (begin < end && begin % blockSize_)
        out.add(ring_[size_t(begin++ - firstSequence_)]);
    while (begin < end && end % blockSize_)
        out.add(ring_[size_t(--end - firstSequence_)]);
    auto mergeTree = [&](size_t a, size_t b) {
        for (a += leaves_, b += leaves_; a < b; a /= 2, b /= 2) {
            if (a & 1)
                out.merge(tree_[a++]);
            if (b & 1)
                out.merge(tree_[--b]);
        }
    };
    if (begin < end) {
        size_t a = size_t(begin / blockSize_ % leaves_), n = size_t((end - begin) / blockSize_);
        size_t first = std::min(n, leaves_ - a);
        mergeTree(a, a + first);
        if (first < n)
            mergeTree(0, n - first);
    }
    return out;
}
PlotFrame Sampling::envelope(double start, double end, int pixels) const {
    PlotFrame out;
    out.start = start;
    out.end = end;
    if (!(end > start) || !std::isfinite(start) || !std::isfinite(end))
        return out;
    pixels = std::clamp(pixels, 1, 4096);
    out.bins.resize(size_t(pixels));
    QMutexLocker lock(&mutex_);
    auto begin = std::lower_bound(ring_.begin(), ring_.end(), start,
                                  [](const Sample &s, double t) { return s.time < t; });
    auto finish =
        std::upper_bound(begin, ring_.end(), end, [](double t, const Sample &s) { return t < s.time; });
    out.points = size_t(finish - begin);
    if (finish - begin > 1)
        out.typicalDt = std::max(1e-9, (begin + 1)->time - begin->time);
    for (int x = 0; x < pixels && begin != finish; ++x) {
        double boundary = start + (end - start) * (x + 1) / pixels;
        auto next = x + 1 == pixels ? finish
                                    : std::lower_bound(begin, finish, boundary,
                                                       [](const Sample &s, double t) { return s.time < t; });
        auto &bin = out.bins[size_t(x)];
        bin.count = size_t(next - begin);
        if (bin.count)
            bin.values = rangeBounds(firstSequence_ + quint64(begin - ring_.begin()),
                                     firstSequence_ + quint64(next - ring_.begin()));
        begin = next;
    }
    return out;
}
std::vector<Sample> Sampling::range(double start, double end, size_t maxPoints) const {
    QMutexLocker lock(&mutex_);
    auto begin = std::lower_bound(ring_.begin(), ring_.end(), start,
                                  [](const Sample &s, double t) { return s.time < t; });
    auto finish =
        std::upper_bound(begin, ring_.end(), end, [](double t, const Sample &s) { return t < s.time; });
    if (size_t(finish - begin) > maxPoints)
        begin = finish - ptrdiff_t(maxPoints);
    return {begin, finish};
}
std::optional<Sample> Sampling::nearest(double time) const {
    QMutexLocker lock(&mutex_);
    if (ring_.empty() || time < ring_.front().time || time > ring_.back().time)
        return {};
    auto it = std::lower_bound(ring_.begin(), ring_.end(), time,
                               [](const Sample &s, double t) { return s.time < t; });
    if (it != ring_.begin() && time - (it - 1)->time < it->time - time)
        --it;
    return *it;
}
void Sampling::append(const Sample &s) {
    QMutexLocker l(&mutex_);
    if (paused_)
        return;
    if (ring_.size() >= capacity_) {
        ring_.pop_front();
        ++firstSequence_;
        dropped_++;
    }
    ring_.push_back(s);
    size_t leaf = leaves_ + size_t(nextSequence_ / blockSize_ % leaves_);
    if (nextSequence_ % blockSize_ == 0)
        tree_[leaf] = {};
    tree_[leaf].add(s);
    for (size_t parent = leaf / 2; parent; parent /= 2) {
        tree_[parent] = tree_[parent * 2];
        tree_[parent].merge(tree_[parent * 2 + 1]);
    }
    ++nextSequence_;
    ++revision_;
    faultPre_.push_back(s);
    while (!faultPre_.empty() && s.time - faultPre_.front().time > 5)
        faultPre_.pop_front();
    if (s.error && !capturingFault_ && (!previous_ || !previous_->error)) {
        fault_.assign(faultPre_.begin(), faultPre_.end());
        ++faultRevision_;
        capturingFault_ = true;
        faultEnd_ = s.time + 5;
    } else if (capturingFault_) {
        fault_.push_back(s);
        ++faultRevision_;
        if (s.time >= faultEnd_)
            capturingFault_ = false;
    }
    if (armed_) {
        double value = s.values[trigger_.channel],
               old = previous_ ? previous_->values[trigger_.channel] : value;
        bool crossing = trigger_.edge == 0  ? value >= trigger_.level
                        : trigger_.edge > 0 ? old < trigger_.level && value >= trigger_.level
                                            : old > trigger_.level && value <= trigger_.level;
        if (!fired_) {
            pre_.push_back(s);
            while (!pre_.empty() && s.time - pre_.front().time > trigger_.pre)
                pre_.pop_front();
            if (crossing) {
                triggered_.assign(pre_.begin(), pre_.end());
                ++captureRevision_;
                fired_ = true;
                end_ = s.time + trigger_.post;
            }
        } else {
            triggered_.push_back(s);
            ++captureRevision_;
            if (s.time >= end_)
                armed_ = false;
        }
    }
    previous_ = s;
}
std::vector<Sample> Sampling::snapshot(quint32 node) const {
    QMutexLocker l(&mutex_);
    std::vector<Sample> out;
    out.reserve(ring_.size());
    for (auto &s : ring_)
        if (!node || node == s.node)
            out.push_back(s);
    return out;
}
std::vector<Sample> Sampling::capture() const {
    QMutexLocker l(&mutex_);
    return triggered_;
}
std::vector<Sample> Sampling::faultCapture() const {
    QMutexLocker l(&mutex_);
    return fault_;
}
void Sampling::pause(bool p) {
    QMutexLocker l(&mutex_);
    paused_ = p;
}
void Sampling::clear() {
    QMutexLocker l(&mutex_);
    ring_.clear();
    events_.clear();
    pre_.clear();
    faultPre_.clear();
    triggered_.clear();
    fault_.clear();
    previous_.reset();
    armed_ = fired_ = capturingFault_ = false;
    dropped_ = 0;
    firstSequence_ = nextSequence_ = 0;
    ++revision_;
    ++generation_;
    ++captureRevision_;
    ++faultRevision_;
    std::fill(tree_.begin(), tree_.end(), SampleBounds{});
}
void Sampling::arm(Trigger t) {
    QMutexLocker l(&mutex_);
    trigger_ = t;
    trigger_.channel = std::clamp(t.channel, 0, channelCount - 1);
    trigger_.pre = std::clamp(t.pre, 0.0, 30.0);
    trigger_.post = std::clamp(t.post, 0.0, 30.0);
    armed_ = true;
    fired_ = false;
    triggered_.clear();
    ++captureRevision_;
    pre_.clear();
    for (auto &s : ring_)
        if (!ring_.empty() && ring_.back().time - s.time <= trigger_.pre)
            pre_.push_back(s);
}
QString Sampling::triggerState() const {
    QMutexLocker l(&mutex_);
    return armed_ ? (fired_ ? "采集触发后数据" : "等待触发") : (fired_ ? "抓拍完成" : "未布防");
}
quint64 Sampling::overwritten() const {
    QMutexLocker l(&mutex_);
    return dropped_;
}
Stats statistics(const std::vector<Sample> &a, int c, double t1, double t2) {
    Stats r;
    double sum = 0, sq = 0, start = 0, end = 0;
    r.min = INFINITY;
    r.max = -INFINITY;
    for (auto &s : a)
        if (s.time >= t1 && s.time <= t2 && std::isfinite(s.values[c])) {
            double v = s.values[c];
            if (!r.count)
                start = s.time;
            end = s.time;
            r.count++;
            sum += v;
            sq += v * v;
            r.min = std::min(r.min, v);
            r.max = std::max(r.max, v);
        }
    if (r.count) {
        r.mean = sum / r.count;
        r.rms = sqrt(sq / r.count);
        r.dt = end - start;
    } else
        r.min = r.max = 0;
    return r;
}
std::vector<std::pair<double, double>> spectrum(const std::vector<Sample> &a, int c) {
    // FFT uses adapter sample spacing when available, never host USB batch timing.
    auto begin = a.begin() + (a.size() > 16384 ? a.size() - 16384 : 0);
    if (begin != a.end() &&
        std::all_of(begin, a.end(), [](const Sample &s) { return std::isfinite(s.acquisitionTime); })) {
        std::vector<Sample> clocked(begin, a.end());
        for (auto &s : clocked) {
            s.time = s.acquisitionTime;
            s.acquisitionTime = std::numeric_limits<double>::quiet_NaN();
        }
        return spectrum(clocked, c);
    }
    std::vector<std::pair<double, double>> out;
    if (a.size() < 16)
        return out;
    size_t n = 1;
    while (n * 2 <= a.size() && n < 16384)
        n *= 2;
    double dt = (a.back().time - a[a.size() - n].time) / (n - 1);
    if (dt <= 0)
        return out;
    std::vector<std::complex<double>> x(n);
    double mean = 0;
    for (size_t i = 0; i < n; i++)
        mean += a[a.size() - n + i].values[c];
    mean /= n;
    if (!std::isfinite(mean))
        return out;
    // Reject gaps/jitter beyond 20%; frequency labels must not imply uniform
    // sampling when it isn't.
    for (size_t i = a.size() - n + 1; i < a.size(); i++)
        if (std::abs(a[i].time - a[i - 1].time - dt) > dt * .2)
            return out;
    double gain = 0;
    for (size_t i = 0; i < n; i++) {
        double w = .5 - .5 * cos(2 * std::numbers::pi * i / (n - 1));
        gain += w;
        x[i] = (a[a.size() - n + i].values[c] - mean) * w;
    }
    for (size_t i = 1, j = 0; i < n; i++) {
        size_t bit = n >> 1;
        for (; j & bit; bit >>= 1)
            j ^= bit;
        j ^= bit;
        if (i < j)
            std::swap(x[i], x[j]);
    }
    for (size_t len = 2; len <= n; len *= 2) {
        auto wlen = std::polar(1.0, -2 * std::numbers::pi / len);
        for (size_t i = 0; i < n; i += len) {
            std::complex<double> w = 1;
            for (size_t j = 0; j < len / 2; j++) {
                auto u = x[i + j], v = x[i + j + len / 2] * w;
                x[i + j] = u + v;
                x[i + j + len / 2] = u - v;
                w *= wlen;
            }
        }
    }
    for (size_t i = 0; i <= n / 2; i++)
        out.push_back({i / (dt * n), std::abs(x[i]) / gain * (i && i < n / 2 ? 2 : 1)});
    return out;
}
QJsonObject stepResponse(const std::vector<Sample> &a, int c, double goal, bool automatic) {
    auto fail = [](const QString &reason) { return QJsonObject{{"status", reason}}; };
    if (c < 0 || c >= channelCount || a.size() < 20)
        return fail("有效样本不足：需要至少 20 点，并包含阶跃前基线与阶跃后响应");
    if (a.front().debugLoop && c != 0 && c != 11)
        return fail("PID阶跃分析请选择环内反馈或环内给定");
    if (a.front().debugLoop)
        c = 0;
    if (c >= 9)
        c = c == 9 ? 6 : c == 10 ? 5 : 0;
    const int reference = c == 0 ? 11 : c == 5 ? 10 : c == 6 ? 9 : -1;
    size_t onset = 0, baselineStart = 0;
    if (automatic) {
        if (reference < 0)
            return fail("此通道没有对应目标曲线，请选择 Iq、速度、位置或使用手动目标");
        // Search the last transition, not the largest response spike. NaN -> known
        // is a first submission and can use the preceding measured baseline.
        for (size_t i = a.size() - 1; i > 0; --i) {
            double next = a[i].values[reference], prev = a[i - 1].values[reference];
            if (std::isfinite(next) && (!std::isfinite(prev) || std::abs(next - prev) > 1e-9)) {
                onset = i;
                goal = next;
                break;
            }
        }
        if (!onset)
            return fail("当前记录没有目标阶跃；请保留阶跃前后数据，或切换手动目标分析");
        baselineStart = onset > 100 ? onset - 100 : 0;
        for (size_t i = onset - 1; i > baselineStart; --i)
            if (std::isfinite(a[i].values[reference]) && std::isfinite(a[i - 1].values[reference]) &&
                std::abs(a[i].values[reference] - a[i - 1].values[reference]) > 1e-9) {
                baselineStart = i;
                break;
            }
    } else {
        if (!std::isfinite(goal))
            return fail("手动目标值无效");
        onset = std::min<size_t>(100, std::max<size_t>(5, a.size() / 20));
    }
    if (onset - baselineStart < 5 || a.size() - onset < 10)
        return fail("已找到目标变化，但阶跃前基线或阶跃后响应不足，请延长记录");
    auto median = [](std::vector<double> values) {
        auto middle = values.begin() + values.size() / 2;
        std::nth_element(values.begin(), middle, values.end());
        return *middle;
    };
    std::vector<double> baseline;
    for (size_t i = baselineStart; i < onset; ++i)
        if (std::isfinite(a[i].values[c]))
            baseline.push_back(a[i].values[c]);
    if (baseline.size() < 5)
        return fail("阶跃前基线有效点不足");
    double initial = median(baseline), delta = goal - initial;
    if (std::abs(delta) < 1e-9)
        return fail("阶跃幅值为零：目标与基线相同");
    for (size_t i = baselineStart + 1; i < a.size(); ++i)
        if (!std::isfinite(a[i].values[c]) ||
            (a[i].debugLoop ? a[i].time <= a[i - 1].time : a[i].time < a[i - 1].time) ||
            a[i].node != a[onset].node ||
            (a[i].debugLoop && (!a[i].enabled || a[i].streamEpoch != a[onset].streamEpoch)))
            return fail("分析区间包含缺失数据、未执行环、运行段/节点变化或非递增时间戳，请重新选择连续区间");
    double dt = (a.back().time - a[baselineStart].time) / (a.size() - 1 - baselineStart);
    for (size_t i = baselineStart + 1; i < a.size(); ++i)
        if (a[i].time - a[i - 1].time > dt * 5)
            return fail("分析区间存在采样中断，请选择连续区间");
    // Causal five-sample median rejects isolated spikes without changing stored data.
    auto filtered = [&](size_t i) {
        std::array<double, 5> values;
        for (size_t j = 0; j < values.size(); ++j)
            values[j] = a[i >= j ? i - j : 0].values[c];
        std::nth_element(values.begin(), values.begin() + 2, values.end());
        return values[2];
    };
    if (!automatic) {
        size_t detected = a.size();
        for (size_t i = onset; i < a.size(); ++i)
            if ((filtered(i) - initial) / delta >= .1) {
                detected = i;
                break;
            }
        if (detected == a.size())
            return fail("未检测到朝向手动目标的响应，请核对目标值、单位及采样区间");
        onset = detected;
    }
    double t0 = a[onset].time, t10 = -1, t90 = -1, peak = 0;
    size_t lastOut = onset;
    bool outside = false;
    for (size_t i = onset; i < a.size(); ++i) {
        double value = filtered(i), ratio = (value - initial) / delta;
        if (t10 < 0 && ratio >= .1)
            t10 = a[i].time;
        if (t90 < 0 && ratio >= .9)
            t90 = a[i].time;
        peak = std::max(peak, ratio);
        // Settling uses raw data: noise outside 2% must not become false stability.
        if (std::abs(a[i].values[c] - goal) > .02 * std::abs(delta)) {
            lastOut = i;
            outside = true;
        }
    }
    std::vector<double> tail;
    size_t tailCount = std::max<size_t>(5, (a.size() - onset) / 10);
    for (size_t i = a.size() - tailCount; i < a.size(); ++i)
        tail.push_back(a[i].values[c]);
    QJsonObject o{
        {"status", "已检测到阶跃"},
        {"source", automatic ? "已提交目标曲线" : "手动目标 / 响应10%交点"},
        {"channel", a.front().debugLoop ? debugChannelNames(a.front().debugLoop - 1)[0] : channelNames[c]},
        {"step_time_s", t0},
        {"initial", initial},
        {"target", goal},
        {"amplitude", delta},
        {"steady_value", median(tail)},
        {"steady_error", goal - median(tail)},
        {"overshoot_percent", std::max(0.0, (peak - 1) * 100)},
        {"note",
         "上升时间与超调采用5点因果中值滤波；稳定时间采用原始值±2%幅值带，末尾需连续保持至少5点或响应窗5%"}};
    if (t10 >= 0 && t90 >= 0)
        o["rise_seconds"] = t90 - t10;
    else
        o["rise_status"] = "响应未达到目标幅值的90%，无法计算完整上升时间";
    size_t stableStart = outside ? lastOut + 1 : onset;
    double hold = std::max(dt * 4, (a.back().time - t0) * .05);
    if (stableStart < a.size() && a.size() - stableStart >= 5 && a.back().time - a[stableStart].time >= hold)
        o["settling_seconds"] = a[stableStart].time - t0;
    else
        o["settling"] = "未在采样窗内持续稳定于±2%幅值带（噪声也可能导致超带）";
    return o;
}
QJsonArray stepResponses(const std::vector<Sample> &a, int channel, double holdSeconds,
                         bool requireDebugPlateau) {
    QJsonArray rows;
    int c = channel >= 9 ? (channel == 9 ? 6 : channel == 10 ? 5 : channel == 11 ? 0 : -1) : channel;
    if (!a.empty() && a.front().debugLoop) {
        if (channel != 0 && channel != 11)
            return rows;
        c = 0;
    }
    int ref = c == 0 ? 11 : c == 5 ? 10 : c == 6 ? 9 : -1;
    if (ref < 0 || !std::isfinite(holdSeconds) || holdSeconds < 0)
        return rows;
    std::vector<size_t> changes;
    for (size_t i = 1; i < a.size(); ++i) {
        double before = a[i - 1].values[ref], after = a[i].values[ref];
        if (a[i].node == a[i - 1].node && std::isfinite(after) &&
            (!std::isfinite(before) || std::abs(after - before) > 1e-9))
            changes.push_back(i);
    }
    for (size_t k = 0; k < changes.size(); ++k) {
        size_t start = changes[k], end = k + 1 < changes.size() ? changes[k + 1] : a.size();
        if (requireDebugPlateau && a[start].debugLoop && end - start < 5)
            continue;
        if (requireDebugPlateau && a[start].debugLoop) {
            // A clipped, continuously changing inner-loop reference is not a
            // commanded step: require a stable baseline as well as a plateau.
            size_t previous = k ? changes[k - 1] : 0;
            if (start - previous < 5)
                continue;
            bool continuous = true;
            for (size_t i = start - 5; i < start + 5; ++i)
                if (!a[i].enabled || a[i].node != a[start].node || a[i].streamEpoch != a[start].streamEpoch ||
                    (i && a[i].time <= a[i - 1].time))
                    continuous = false;
            if (!continuous)
                continue;
        }
        size_t pre = std::max(k ? changes[k - 1] : size_t(0), start > 100 ? start - 100 : size_t(0));
        if (a[start].debugLoop) {
            while (pre < start && a[pre].streamEpoch != a[start].streamEpoch)
                ++pre;
            for (size_t i = start; i < end; ++i)
                if (a[i].streamEpoch != a[start].streamEpoch) {
                    end = i;
                    break;
                }
        }
        std::vector<Sample> segment(a.begin() + pre, a.begin() + end);
        auto o = stepResponse(segment, c, 0, true);
        // Keep every detected command edge, even if its response cannot be measured.
        o["index"] = int(rows.size() + 1);
        o["channel"] = channelNames[c];
        o["source"] = std::isfinite(a[start - 1].values[ref]) ? "目标指令变化" : "首次目标提交（前指令未知）";
        o["step_time_s"] = a[start].time;
        o["target"] = a[start].values[ref];
        if (std::isfinite(a[start - 1].values[ref]))
            o["previous_target"] = a[start - 1].values[ref];
        o["window_end_s"] = a[end - 1].time;
        o["window_ms"] = (a[end - 1].time - a[start].time) * 1000;
        o["crossing_hold_ms"] = holdSeconds * 1000;
        if (o.contains("initial") && o.contains("amplitude")) {
            double initial = o["initial"].toDouble(), delta = o["amplitude"].toDouble();
            o["level10"] = initial + .1 * delta;
            o["level90"] = initial + .9 * delta;
            double first10 = -1, first90 = -1, qualified10 = -1, qualified90 = -1;
            double run10 = -1, run90 = -1, peak = 0;
            for (size_t i = start; i < end; ++i) {
                double ratio = (a[i].values[c] - initial) / delta;
                peak = std::max(peak, ratio);
                auto crossing = [&](double threshold, double &first, double &run, double &qualified) {
                    if (ratio >= threshold) {
                        if (first < 0)
                            first = a[i].time;
                        if (run < 0)
                            run = a[i].time;
                        if (qualified < 0 && a[i].time - run + 1e-12 >= holdSeconds)
                            qualified = run;
                    } else
                        run = -1;
                };
                crossing(.1, first10, run10, qualified10);
                crossing(.9, first90, run90, qualified90);
            }
            if (first10 >= 0)
                o["first10_time_s"] = first10;
            if (first90 >= 0)
                o["first90_time_s"] = first90;
            if (first10 >= 0 && first90 >= 0)
                o["first_rise_ms"] = (first90 - first10) * 1000;
            if (qualified10 >= 0)
                o["qualified10_time_s"] = qualified10;
            if (qualified90 >= 0)
                o["qualified90_time_s"] = qualified90;
            if (qualified10 >= 0 && qualified90 >= 0)
                o["qualified_rise_ms"] = (qualified90 - qualified10) * 1000;
            o["raw_overshoot_percent"] = std::max(0.0, (peak - 1) * 100);
            o["crossing_note"] = "首次交越采用原始值，可能受尖峰影响；保持交越需连续达到阈值指定时间，记录的"
                                 "是该保持段起点。rise_seconds 保留5点中值滤波结果。";
        }
        if (a[start].debugLoop) {
            o["channel"] = debugChannelNames(a[start].debugLoop - 1)[0];
            const bool commanded = std::isfinite(a[start].values[19]) && (int(a[start].values[19]) & 8) != 0;
            o["source"] = commanded ? "设备上层目标 / PID更新时间" : "设备环内给定 / PID更新时间";
            if (commanded)
                o["reference_note"] = "设备已接收的上层目标，未经过软限速修正；反馈、积分和输出仍是实际控制状"
                                      "态。限速或串级控制介入时，不等同独立PID固定输入响应。";
            o["timing_note"] =
                "按调试抽样帧的PID更新时间估算；阶跃时刻位于前一帧与首个新给定帧之间，不等同完整控制周期录波";
            o["step_interval_start_s"] = a[start - 1].time;
            o["step_interval_end_s"] = a[start].time;
            o["sample_interval_ms"] = (a[start].time - a[start - 1].time) * 1000;
        }
        rows.append(o);
    }
    return rows;
}
static bool exportDebugCsv(const QString &path, const std::vector<Sample> &samples, QString *error,
                           const std::atomic<bool> *cancel) {
    QSaveFile file(path);
    if (!file.open(QIODevice::WriteOnly)) {
        if (error)
            *error = file.errorString();
        return false;
    }
    QTextStream out(&file);
    out.setRealNumberPrecision(12);
    out << "device_time_s,node,loop,epoch,feedback,error,iq_A,id_A,pid_dt_s,integral,output,vbus_V,shaft_rad_"
           "s,kp,ki_config,reference,limit,i_limit,ramp,p_term,ki_effective,active,tick_us,flags,"
           "cur_loop_us,vel_loop_us,pos_loop_us,ctrl_run_us,control_loop_us,sense_update_us,current_sense_us,encoder_update_us\n";
    for (const auto &s : samples) {
        if (cancel && cancel->load()) {
            file.cancelWriting();
            if (error)
                *error = "导出已取消";
            return false;
        }
        out << s.time << ',' << s.node << ',' << int(s.debugLoop) << ',' << s.streamEpoch;
        for (double value : s.values) {
            out << ',';
            if (std::isfinite(value))
                out << value;
        }
        out << '\n';
    }
    out.flush();
    if (!file.commit()) {
        if (error)
            *error = file.errorString();
        return false;
    }
    return true;
}
bool exportCsv(const QString &path, const std::vector<Sample> &a, QString *error,
               const std::atomic<bool> *cancel) {
    if (!a.empty() && a.front().debugLoop)
        return exportDebugCsv(path, a, error, cancel);
    QSaveFile f(path);
    if (!f.open(QIODevice::WriteOnly)) {
        if (error)
            *error = f.errorString();
        return false;
    }
    QTextStream t(&f);
    t << "time_s,node,iq_A,id_A,ia_A,ib_A,ic_A,velocity_rpm,position_rad,vbus_V,"
         "temperature_C,enabled,mode,error,target_position_rad,target_velocity_rpm,target_current_A\n";
    t.setRealNumberPrecision(12);
    for (auto &s : a) {
        if (cancel && cancel->load()) {
            f.cancelWriting();
            if (error)
                *error = "导出已取消";
            return false;
        }
        t << s.time << ',' << s.node;
        for (int c = 0; c < 9; ++c) {
            double v = s.values[c];
            t << ',';
            if (std::isfinite(v))
                t << v;
        }
        t << ',' << s.enabled << ',' << int(s.mode) << ',' << int(s.error);
        for (int c = 9; c < normalChannelCount; ++c) {
            t << ',';
            if (std::isfinite(s.values[c]))
                t << s.values[c];
        }
        t << '\n';
    }
    t.flush();
    bool ok = f.commit();
    if (!ok && error)
        *error = f.errorString();
    return ok;
}
bool Sampling::exportHistory(const QString &path, quint32 node, QString *error,
                             const std::atomic<bool> *cancel) const {
    quint64 begin, end, generation;
    {
        QMutexLocker lock(&mutex_);
        begin = firstSequence_;
        end = nextSequence_;
        generation = generation_;
    }
    QSaveFile file(path);
    auto fail = [&](const QString &why) {
        if (error)
            *error = why;
        file.cancelWriting();
        return false;
    };
    if (!file.open(QIODevice::WriteOnly))
        return fail(file.errorString());
    QTextStream stream(&file);
    stream.setRealNumberPrecision(12);
    stream << "time_s,node,iq_A,id_A,ia_A,ib_A,ic_A,velocity_rpm,position_rad,"
              "vbus_V,temperature_C,enabled,mode,error,target_position_rad,target_velocity_rpm,target_"
              "current_A\n";
    // Snapshot boundary is fixed at export start. Copy only a small batch under
    // the RX lock.
    for (; begin < end;) {
        if (cancel && cancel->load())
            return fail("导出已取消");
        std::vector<Sample> batch;
        {
            QMutexLocker lock(&mutex_);
            if (generation != generation_ || begin < firstSequence_ || end > nextSequence_)
                return fail("导出期间历史被覆盖或节点已切换，请提高记录容量后重试");
            size_t count = size_t(std::min<quint64>(4096, end - begin));
            auto first = ring_.begin() + ptrdiff_t(begin - firstSequence_);
            batch.assign(first, first + ptrdiff_t(count));
            begin += count;
        }
        for (const auto &sample : batch) {
            if (node && sample.node != node)
                continue;
            stream << sample.time << ',' << sample.node;
            for (int c = 0; c < 9; ++c) {
                double value = sample.values[c];
                stream << ',';
                if (std::isfinite(value))
                    stream << value;
            }
            stream << ',' << sample.enabled << ',' << int(sample.mode) << ',' << int(sample.error);
            for (int c = 9; c < normalChannelCount; ++c) {
                stream << ',';
                if (std::isfinite(sample.values[c]))
                    stream << sample.values[c];
            }
            stream << '\n';
        }
        if (stream.status() != QTextStream::Ok)
            return fail("CSV 写入失败，请检查磁盘空间");
    }
    stream.flush();
    if (stream.status() != QTextStream::Ok)
        return fail("CSV 写入失败，请检查磁盘空间");
    if (!file.commit()) {
        if (error)
            *error = file.errorString();
        return false;
    }
    return true;
}
} // namespace mt

namespace mt {
QJsonArray hostStepResponses(const std::vector<Sample> &data, const std::vector<TargetEvent> &events,
                             int channel, double hold) {
    QJsonArray rows;
    if (data.empty() || data.front().debugLoop)
        return rows;
    int ref = channel == 0 || channel == 11   ? 11
              : channel == 5 || channel == 10 ? 10
              : channel == 6 || channel == 9  ? 9
                                              : -1;
    if (ref < 0)
        return rows;
    std::vector<TargetEvent> accepted;
    for (const auto &e : events)
        if ((e.state == 1 || e.state == 2) && e.node == data.front().node && std::isfinite(e.values[ref - 9]))
            accepted.push_back(e);
    for (size_t k = 0; k < accepted.size(); ++k) {
        const auto &e = accepted[k];
        if (e.queued < data.front().time || e.queued > data.back().time)
            continue;
        const double previous =
            k ? accepted[k - 1].values[ref - 9] : std::numeric_limits<double>::quiet_NaN();
        if (std::isfinite(previous) && previous == e.values[ref - 9])
            continue;
        auto onset = std::lower_bound(data.begin(), data.end(), e.queued,
                                      [](const Sample &s, double t) { return s.time < t; });
        auto begin = onset - std::min<ptrdiff_t>(100, onset - data.begin());
        if (k)
            begin = std::lower_bound(begin, onset, accepted[k - 1].queued,
                                     [](const Sample &s, double t) { return s.time < t; });
        auto end = k + 1 < accepted.size()
                       ? std::lower_bound(onset, data.end(), accepted[k + 1].queued,
                                          [](const Sample &s, double t) { return s.time < t; })
                       : data.end();
        std::vector<Sample> segment(begin, end);
        for (auto &sample : segment)
            sample.values[ref] = sample.time < e.queued ? previous : e.values[ref - 9];
        auto detected = stepResponses(segment, channel, hold);
        QJsonObject row = detected.isEmpty()
                              ? QJsonObject{{"status", "事件已记录，但缺少阶跃前后反馈样本，无法测量"}}
                              : detected.first().toObject();
        row["index"] = rows.size() + 1;
        row["source"] = "主机入队事件 / 主机接收反馈时间（端到端）";
        row["request_token"] = QString::number(e.token);
        row["step_time_s"] = e.queued;
        row["driver_submitted_s"] = e.sent;
        if (e.ack >= 0)
            row["ack_s"] = e.ack;
        row["target"] = e.values[ref - 9];
        if (std::isfinite(previous))
            row["previous_target"] = previous;
        if (onset != data.end())
            row["first_feedback_after_event_s"] = onset->time;
        if (row.contains("first90_time_s"))
            row["enqueue_to_90_ms"] = (row["first90_time_s"].toDouble() - e.queued) * 1000;
        row["timing_note"] = "入队与反馈取帧使用同一主机单调时钟；包含排队、通信和回传延迟，不是纯PID响应。批"
                             "量取帧可具有相同时间，不伪造设备采样间隔。";
        rows.append(row);
    }
    return rows;
}
bool exportTargetEvents(const QString &path, const std::vector<TargetEvent> &events, QString *error) {
    QSaveFile f(path);
    if (!f.open(QIODevice::WriteOnly)) {
        if (error)
            *error = f.errorString();
        return false;
    }
    QJsonArray rows;
    for (const auto &e : events) {
        QJsonObject o{
            {"request", QString::number(e.token)},
            {"node", int(e.node)},
            {"queued_host_s", e.queued},
            {"sent_host_s", e.sent},
            {"ack_host_s", e.ack},
            {"state",
             QStringList{"queued", "submitted", "acknowledged", "rejected_or_cancelled"}.value(e.state)},
            {"reason", e.reason}};
        for (int c = 0; c < 3; ++c)
            if (std::isfinite(e.values[c]))
                o[QStringList{"position_rad", "velocity_rpm", "current_A"}[c]] = e.values[c];
        rows.append(o);
    }
    f.write(QJsonDocument(rows).toJson());
    if (!f.commit()) {
        if (error)
            *error = f.errorString();
        return false;
    }
    return true;
}
} // namespace mt
