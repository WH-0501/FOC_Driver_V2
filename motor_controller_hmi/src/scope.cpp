#include "scope.h"
#include <QMouseEvent>
#include <QPainter>
#include <QPainterPath>
#include <QWheelEvent>
#include <algorithm>
#include <cmath>
namespace mt {
Scope::Scope(QWidget *p) : QWidget(p) {
    setMinimumSize(400, 280);
    for (int c = 12; c < channelCount; ++c)
        colors[c] = QColor::fromHsv((c * 37) % 360, 140, 245);
    while (aliases.size() < channelCount)
        aliases.append(QString("通道%1").arg(aliases.size()));
    setMouseTracking(true);
    setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
}
void Scope::setSamples(std::vector<Sample> s) {
    data_ = std::move(s);
    dirty_ = true;
    if (follow && !data_.empty())
        end_ = data_.back().time;
    update();
}
void Scope::bind(Sampling *history, int source) {
    if (history_ == history && source_ == source)
        return;
    history_ = history;
    source_ = source;
    sourceRevision_ = UINT64_MAX;
    data_.clear();
    dirty_ = true;
    bool wasPaused = displayPaused;
    frozen_.reset();
    displayPaused = false;
    refresh();
    setDisplayPaused(wasPaused);
}
void Scope::setDisplayPaused(bool paused) {
    if (paused == displayPaused)
        return;
    if (paused && history_ && !source_) {
        frozen_ = history_->frozenCopy();
        info_ = frozen_->info();
        if (follow && info_.size)
            end_ = info_.last;
    } else if (!paused) {
        frozen_.reset();
    }
    displayPaused = paused;
    dirty_ = true;
    sourceRevision_ = UINT64_MAX;
    if (!paused)
        refresh();
    update();
}
std::vector<Sample> Scope::recordedSamples(quint32 node) const {
    return displayHistory() && !source_ ? displayHistory()->snapshot(node) : data_;
}
std::vector<TargetEvent> Scope::targetEvents() const {
    if (debugProfile || !displayHistory())
        return {};
    auto events = source_ ? captureEvents_ : displayHistory()->targetEvents();
    auto info = displayHistory()->info();
    auto last = displayHistory()->nearest(info.last);
    if (last)
        std::erase_if(events, [&](const auto &e) { return e.node != last->node; });
    return events;
}
void Scope::refresh() {
    if (!history_ || displayPaused)
        return;
    auto info = history_->info();
    if (info.generation != info_.generation) {
        manualY_.fill(std::nullopt);
        drawnAxis_ = -1;
    }
    quint64 revision = source_ ? history_->captureRevision(source_) : info.revision;
    if (revision == sourceRevision_)
        return;
    sourceRevision_ = revision;
    if (source_) {
        captureEvents_ = history_->targetEvents();
        setSamples(source_ == 1 ? history_->capture() : history_->faultCapture());
    } else {
        bool changed =
            info.generation != info_.generation || info.first != info_.first || follow || info_.last < end_;
        if (follow && info.size)
            end_ = info.last;
        dirty_ = dirty_ || changed;
    }
    info_ = info;
    if (dirty_)
        update();
}
std::vector<Sample> Scope::samples() const {
    if (history_ && !source_)
        return displayHistory()->range(cursors ? std::min(t1, t2) : end_ - windowSeconds,
                                       cursors ? std::max(t1, t2) : end_);
    return data_;
}
void Scope::fitHistory() {
    auto info = history_ && !source_ ? displayHistory()->info() : HistoryInfo{};
    if (source_ && !data_.empty()) {
        info.first = data_.front().time;
        info.last = data_.back().time;
        info.size = data_.size();
    }
    if (info.size) {
        end_ = info.last;
        windowSeconds = std::max(.001, info.last - info.first);
    }
    follow = false;
    update();
}
void Scope::clearDisplay() {
    captureEvents_.clear();
    if (displayPaused && history_ && !source_)
        frozen_ = history_->frozenCopy();
    data_.clear();
    frame_ = {};
    base_ = {};
    info_ = {};
    sourceRevision_ = UINT64_MAX;
    drawnAxis_ = -1;
    cursors = false;
    t1 = t2 = 0;
    dragging_ = selecting_ = false;
    dragCursor_ = 0;
    end_ = 5;
    resetView();
}
void Scope::resetView() {
    manualY_.fill(std::nullopt);
    dirty_ = true;
    follow = true;
    windowSeconds = 5;
    update();
}
int Scope::scaleGroup(int c) const {
    if (debugProfile)
        // The PID timing block occupies the trailing channels, including nested timings.
        return c >= 20 && c < channelCount ? 20 : (c == 11 ? 0 : c);
    return c == 9 ? 6 : c == 10 ? 5 : c == 11 ? 0 : c;
}
double Scope::timeAt(double x) const {
    return end_ - windowSeconds + (x - plot_.left()) / plot_.width() * windowSeconds;
}
void Scope::paintEvent(QPaintEvent *) {
    QString key = QString::number(width()) + ':' + QString::number(height()) + ':' +
                  QString::number(devicePixelRatioF()) + ':' + QString::number(view) + ':' +
                  QString::number(windowSeconds, 'g', 17) + ':' + QString::number(end_, 'g', 17) + ':' +
                  QString::number(decimation) + ":" + QString::number(axisChannel);
    for (int c = 0; c < channelCount; ++c)
        key += QString::number(visible[c]) + colors[c].name() + aliases.value(c);
    if (dirty_ || key != cacheKey_ || base_.isNull()) {
        base_ = QPixmap(size() * devicePixelRatioF());
        base_.setDevicePixelRatio(devicePixelRatioF());
        QPainter basePainter(&base_);
        drawBase(basePainter);
        cacheKey_ = key;
        dirty_ = false;
    }
    QPainter p(this);
    p.drawPixmap(0, 0, base_);
    drawOverlay(p);
}
void Scope::drawBase(QPainter &p) {
    // Dense envelopes use single-pixel raster lines; AA stroking of overlapping
    // paths is far more expensive and adds no detail at this sampling density.
    p.setRenderHint(QPainter::Antialiasing, false);
    p.fillRect(rect(), QColor("#0b0d10"));
    int legendRows = std::max(1, (int(std::count(visible.begin(), visible.end(), true)) + 2) / 3);
    plot_ = QRectF(88, 46, width() - 112, height() - (debugProfile ? 88 + 18 * legendRows : 160));
    p.setFont(QFont("Consolas", 9));
    p.setPen(QColor("#252829"));
    for (int i = 0; i <= 10; i++) {
        double x = plot_.left() + plot_.width() * i / 10;
        p.drawLine(QPointF(x, plot_.top()), QPointF(x, plot_.bottom()));
    }
    for (int i = 0; i <= 8; i++) {
        double y = plot_.top() + plot_.height() * i / 8;
        p.drawLine(QPointF(plot_.left(), y), QPointF(plot_.right(), y));
    }
    p.setPen(QColor("#9c9c9d"));
    p.drawText(QPointF(16, 18), view == 1      ? "XY · Id / Iq [A]"
                                : view == 2    ? "FFT · Hann · peak amplitude"
                                : debugProfile ? "DEVICE PID TIME  /  sampled loop state"
                                               : "HOST RECEIVE TIME  /  queued target events");
    if (history_ && !source_) {
        if (view == 0)
            frame_ = displayHistory()->envelope(end_ - windowSeconds, end_,
                                                std::max(1, int(plot_.width()) / std::max(1, decimation)));
        else
            data_ = displayHistory()->range(end_ - windowSeconds, end_, 16384);
    }
    if ((history_ && !source_ && view == 0) ? !frame_.points : data_.empty()) {
        p.setPen(QColor("#9c9c9d"));
        p.drawText(plot_, Qt::AlignCenter,
                   history_ && displayHistory()->info().size
                       ? "当前视窗无数据\n点击回到实时或查看全部记录"
                       : "等待遥测数据\n连接 Demo 或 CAN FD 设备开始采集");
        return;
    }
    if (view == 1) {
        double range = .01;
        for (auto &s : data_)
            if (s.time >= end_ - windowSeconds)
                range = std::max({range, std::abs(s.values[0]), std::abs(s.values[1])});
        range *= 1.15;
        double scale = std::min(plot_.width(), plot_.height()) / (2 * range);
        auto center = plot_.center();
        p.setClipRect(plot_);
        p.setPen(QColor("#434345"));
        p.drawEllipse(center, scale * range * .8, scale * range * .8);
        QPainterPath path;
        bool first = true;
        for (auto &s : data_)
            if (s.time >= end_ - windowSeconds && std::isfinite(s.values[1])) {
                QPointF q(center.x() + s.values[1] * scale, center.y() - s.values[0] * scale);
                if (first) {
                    path.moveTo(q);
                    first = false;
                } else
                    path.lineTo(q);
            }
        p.setPen(QPen(colors[0], 1));
        p.drawPath(path);
        p.setClipping(false);
        p.drawText(QPointF(plot_.left(), height() - 16),
                   QString("等比例坐标 · ±%1 A · 最近最多 16384 点").arg(range, 0, 'g', 4));
        return;
    }
    if (view == 2) {
        int c = 0;
        while (c < channelCount - 1 && !visible[c])
            c++;
        auto bins = spectrum(data_, c);
        if (bins.empty()) {
            p.drawText(plot_, Qt::AlignCenter,
                       "FFT 需要至少 16 "
                       "个等间隔有效样本\n暂停/丢帧间隙不会被误算为连续数据");
            return;
        }
        double max = .00001;
        for (auto &v : bins)
            max = std::max(max, v.second);
        QPainterPath path;
        for (size_t i = 0; i < bins.size(); i++) {
            QPointF q(plot_.left() + i * plot_.width() / (bins.size() - 1),
                      plot_.bottom() - bins[i].second / max * plot_.height() * .9);
            if (!i)
                path.moveTo(q);
            else
                path.lineTo(q);
        }
        p.setPen(QPen(colors[c], 1, (debugProfile ? c == 11 : c >= 9) ? Qt::DashLine : Qt::SolidLine));
        p.drawPath(path);
        p.drawText(QPointF(plot_.left(), height() - 16), QString("0 → %1 Hz  |  peak %2  |  %3")
                                                             .arg(bins.back().first, 0, 'f', 1)
                                                             .arg(max, 0, 'g', 4)
                                                             .arg(aliases[c]));
        return;
    }
    double start = end_ - windowSeconds;
    if (!history_ || source_) {
        frame_ = {};
        frame_.start = start;
        frame_.end = end_;
        frame_.bins.resize(size_t(std::max(1, int(plot_.width()) / std::max(1, decimation))));
        auto begin =
            std::lower_bound(data_.begin(), data_.end(), start, [](auto &s, double t) { return s.time < t; });
        auto finish =
            std::upper_bound(begin, data_.end(), end_, [](double t, auto &s) { return t < s.time; });
        frame_.points = size_t(finish - begin);
        if (finish - begin > 1)
            frame_.typicalDt = (begin + 1)->time - begin->time;
        for (auto it = begin; it != finish; ++it) {
            size_t x = std::min(frame_.bins.size() - 1,
                                size_t((it->time - start) / windowSeconds * frame_.bins.size()));
            auto &bin = frame_.bins[x];
            ++bin.count;
            bin.values.add(*it);
        }
    }
    const auto hostEvents = targetEvents();
    int legendIndex = 0;
    int axis = axisChannel;
    std::array<double, channelCount> low, high, groupLow, groupHigh;
    low.fill(INFINITY);
    high.fill(-INFINITY);
    groupLow = low;
    groupHigh = high;
    for (int c = 0; c < channelCount; ++c) {
        if (!visible[c]) continue;
        for (const auto &bin : frame_.bins) {
            low[c] = std::min(low[c], bin.values.low[c]);
            high[c] = std::max(high[c], bin.values.high[c]);
        }
        const int group = scaleGroup(c);
        groupLow[group] = std::min(groupLow[group], low[c]);
        groupHigh[group] = std::max(groupHigh[group], high[c]);
    }
    auto hasValues = [&](int c) { return std::isfinite(low[c]); };
    if (axis < 0 || axis >= channelCount || !visible[axis] || !hasValues(axis)) {
        axis = -1;
        for (int c = 0; c < channelCount; ++c)
            if (visible[c] && hasValues(c)) {
                axis = c;
                break;
            }
    }
    drawnAxis_ = axis;
    if (axis >= 0) {
        p.setPen(colors[axis]);
        p.drawText(QPointF(plot_.left(), 35), QString("Y: %1").arg(aliases[axis]));
    }
    for (int c = 0; c < channelCount; c++)
        if (visible[c] && hasValues(c)) {
            const int group = scaleGroup(c);
            double lo = groupLow[group], hi = groupHigh[group];
            double pad = std::max((hi - lo) * .12, .001);
            lo -= pad;
            hi += pad;
            if (manualY_[group]) {
                lo = manualY_[group]->first;
                hi = manualY_[group]->second;
            }
            drawnY_[c] = {lo, hi};
            if (c == axis) {
                p.setPen(colors[c]);
                for (int tick = 0; tick <= 8; ++tick) {
                    double y = plot_.bottom() - tick * plot_.height() / 8;
                    double value = lo + (hi - lo) * tick / 8;
                    p.drawText(QRectF(0, y - 9, plot_.left() - 8, 18), Qt::AlignRight | Qt::AlignVCenter,
                               QString::number(value, 'g', 5));
                    p.drawLine(QPointF(plot_.left() - 4, y), QPointF(plot_.left(), y));
                }
            }
            int pixels = int(frame_.bins.size());
            p.setClipRect(plot_);
            p.setPen(QPen(colors[c], 1, (debugProfile ? c == 11 : c >= 9) ? Qt::DashLine : Qt::SolidLine));
            QPainterPath path;
            QVector<QLineF> peaks;
            peaks.reserve(pixels);
            bool first = true;
            int lastPixel = -1;
            double typicalDt = frame_.typicalDt;
            double maxPixelGap = std::max(2.0, typicalDt * 2.5 / windowSeconds * pixels);
            auto y = [&](double v) { return plot_.bottom() - (v - lo) / (hi - lo) * plot_.height(); };
            for (int x = 0; x < pixels; x++)
                if (std::isfinite(frame_.bins[size_t(x)].values.low[c])) {
                    double low = frame_.bins[size_t(x)].values.low[c],
                           high = frame_.bins[size_t(x)].values.high[c];
                    double pixelX = plot_.left() + x * plot_.width() / pixels;
                    QPointF q(pixelX, y((low + high) * .5));
                    if (first || x - lastPixel > maxPixelGap) {
                        path.moveTo(q);
                        first = false;
                    } else {
                        if (debugProfile ? c == 11 : c >= 9)
                            path.lineTo(QPointF(q.x(), path.currentPosition().y()));
                        path.lineTo(q);
                    }
                    lastPixel = x;
                    if (high > low)
                        peaks.append(QLineF(pixelX, y(low), pixelX, y(high)));
                }
            if (!debugProfile && c >= 9 && c < 12 && !hostEvents.empty()) {
                QPainterPath steps;
                bool started = false;
                double lastValue = 0;
                for (const auto &e : hostEvents) {
                    if ((e.state != 1 && e.state != 2) || !std::isfinite(e.values[c - 9]) || e.queued > end_)
                        continue;
                    double x =
                        plot_.left() + (std::max(e.queued, end_ - windowSeconds) - (end_ - windowSeconds)) /
                                           windowSeconds * plot_.width();
                    if (e.queued < end_ - windowSeconds) {
                        steps = QPainterPath();
                        steps.moveTo(plot_.left(), y(e.values[c - 9]));
                        started = true;
                        lastValue = e.values[c - 9];
                        continue;
                    }
                    if (started)
                        steps.lineTo(x, y(lastValue));
                    else {
                        steps.moveTo(x, y(e.values[c - 9]));
                        started = true;
                    }
                    steps.lineTo(x, y(e.values[c - 9]));
                    lastValue = e.values[c - 9];
                }
                if (started) {
                    steps.lineTo(plot_.right(), y(lastValue));
                    p.drawPath(steps);
                } else {
                    p.drawLines(peaks);
                    p.drawPath(path);
                }
            } else {
                p.drawLines(peaks);
                p.drawPath(path);
            }
            p.setClipping(false);
            QString legend = QString("%1 · 窗口 [%2, %3]").arg(aliases[c]).arg(low[c], 0, 'g', 6).arg(high[c], 0, 'g', 6);
            double legendWidth = plot_.width() / 3;
            p.drawText(QPointF(64 + (legendIndex % 3) * legendWidth,
                               height() - (debugProfile ? 6 + 18 * legendRows : 78) + (legendIndex / 3) * 18),
                       p.fontMetrics().elidedText(legend, Qt::ElideRight, int(legendWidth - 12)));
            ++legendIndex;
        }
    p.setPen(QColor("#9c9c9d"));
    for (int i = 0; i <= 5; i++) {
        double t = start + windowSeconds * i / 5;
        p.drawText(QPointF(plot_.left() + plot_.width() * i / 5 - 12, plot_.bottom() + 19),
                   QString::number(t, 'f', 3));
    }
    p.drawText(QPointF(width() - 38, plot_.bottom() + 19), "s");
}
void Scope::drawOverlay(QPainter &p) {
    if (view != 0)
        return;
    p.setFont(QFont("Consolas", 9));
    double start = end_ - windowSeconds;
    if (cursors) {
        for (auto pair : {qMakePair(t1, QString("T1")), qMakePair(t2, QString("T2"))}) {
            double x = plot_.left() + (pair.first - start) / windowSeconds * plot_.width();
            p.setPen(QPen(QColor("#ffbc33"), 1, Qt::DashLine));
            p.drawLine(QPointF(x, plot_.top()), QPointF(x, plot_.bottom()));
            p.drawText(QPointF(x + 4, plot_.top() + 16), pair.second);
        }
        p.setPen(QColor("#ffbc33"));
        p.drawText(QPointF(plot_.right() - 160, 18),
                   QString("Δt %1 ms").arg(std::abs(t2 - t1) * 1000, 0, 'f', 3));
    }
    if (plot_.contains(last_)) {
        double t = timeAt(last_.x());
        std::optional<Sample> nearest;
        if (history_ && !source_)
            nearest = displayHistory()->nearest(t);
        else {
            auto it =
                std::lower_bound(data_.begin(), data_.end(), t, [](auto &s, double x) { return s.time < x; });
            if (it != data_.end()) {
                if (it != data_.begin() && t - (it - 1)->time < it->time - t)
                    --it;
                nearest = *it;
            }
        }
        if (nearest) {
            double x = plot_.left() + (nearest->time - start) / windowSeconds * plot_.width();
            p.setPen(QPen(QColor("#6a6b6c"), 1, Qt::DotLine));
            p.drawLine(QPointF(x, plot_.top()), QPointF(x, plot_.bottom()));
            QStringList readings;
            for (int c = 0; c < channelCount; ++c) {
                if (!visible[c]) continue;
                double value = nearest->values[c];
                readings << QString("%1 = %2").arg(aliases.value(c),
                    std::isfinite(value) ? QString::number(value, 'g', 7) : QStringLiteral("—"));
            }
            QString label = QString(debugProfile ? "设备 t=%1 s" : "t=%1 s").arg(nearest->time, 0, 'f', 6);
            if (!readings.isEmpty()) label += "\n" + readings.join("    ");
            QRectF area = plot_.adjusted(8, 8, -8, -8);
            int flags = Qt::AlignLeft | Qt::AlignTop | Qt::TextWordWrap;
            QRectF box = p.boundingRect(area.adjusted(6, 3, -6, -3), flags, label)
                            .adjusted(-6, -3, 6, 3).intersected(area);
            p.fillRect(box, QColor("#1b1c1e"));
            p.setPen(Qt::white);
            p.drawText(box.adjusted(6, 3, -6, -3), flags, label);
        }
    }
    if (selecting_) {
        p.setPen(QColor("#55b3ff"));
        p.fillRect(QRectF(rubberStart_, rubberEnd_).normalized(), QColor(85, 179, 255, 30));
        p.drawRect(QRectF(rubberStart_, rubberEnd_).normalized());
    }
}
void Scope::wheelEvent(QWheelEvent *e) {
    if (view == 0 && (e->modifiers() & Qt::ControlModifier || e->position().x() < plot_.left())) {
        if (drawnAxis_ >= 0 && plot_.height() > 0) {
            double fraction = std::clamp((plot_.bottom() - e->position().y()) / plot_.height(), 0.0, 1.0);
            std::array<bool, channelCount> updated{};
            for (int c = 0; c < channelCount; ++c) {
                if (!visible[c] || !std::any_of(frame_.bins.begin(), frame_.bins.end(), [c](const auto &bin) {
                        return std::isfinite(bin.values.low[c]);
                    }))
                    continue;
                int group = scaleGroup(c);
                if (updated[group])
                    continue; // A target/feedback pair shares one transform.
                updated[group] = true;
                auto [lo, hi] = manualY_[group].value_or(drawnY_[c]);
                double anchor = lo + fraction * (hi - lo);
                double span = std::clamp((hi - lo) * std::pow(.8, e->angleDelta().y() / 120.0), 1e-9, 1e12);
                manualY_[group] = std::pair{anchor - fraction * span, anchor + (1 - fraction) * span};
            }
            dirty_ = true;
            update();
        }
        e->accept();
        return;
    }
    double anchor = timeAt(e->position().x());
    double fraction = (e->position().x() - plot_.left()) / plot_.width();
    windowSeconds = std::clamp(windowSeconds * std::pow(.8, e->angleDelta().y() / 120.0), .001, 86400.0);
    end_ = anchor + (1 - fraction) * windowSeconds;
    follow = false;
    update();
}
void Scope::mousePressEvent(QMouseEvent *e) {
    last_ = e->position();
    if (e->button() != Qt::LeftButton)
        return;
    if (e->modifiers() & Qt::ShiftModifier) {
        selecting_ = true;
        rubberStart_ = rubberEnd_ = last_;
    } else if (cursors) {
        double t = timeAt(last_.x());
        dragCursor_ = std::abs(t - t1) < std::abs(t - t2) ? 1 : 2;
    } else
        dragging_ = true;
}
void Scope::mouseMoveEvent(QMouseEvent *e) {
    if (dragging_) {
        end_ -= (e->position().x() - last_.x()) / plot_.width() * windowSeconds;
        follow = false;
    }
    if (selecting_)
        rubberEnd_ = e->position();
    if (dragCursor_) {
        (dragCursor_ == 1 ? t1 : t2) = timeAt(e->position().x());
        emit cursorChanged(t1, t2);
    }
    last_ = e->position();
    update();
}
void Scope::mouseReleaseEvent(QMouseEvent *) {
    if (selecting_ && std::abs(rubberEnd_.x() - rubberStart_.x()) > 8) {
        double a = timeAt(rubberStart_.x()), b = timeAt(rubberEnd_.x());
        end_ = std::max(a, b);
        windowSeconds = std::abs(a - b);
        follow = false;
    }
    dragging_ = selecting_ = false;
    dragCursor_ = 0;
    update();
}
} // namespace mt
