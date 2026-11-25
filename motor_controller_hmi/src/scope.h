#pragma once
#include "sampling.h"
#include <QColor>
#include <QPixmap>
#include <QWidget>
#include <array>
namespace mt {
class Scope : public QWidget {
    Q_OBJECT
  public:
    explicit Scope(QWidget *parent = nullptr);
    void setSamples(std::vector<Sample> samples);
    void bind(Sampling *, int source = 0);
    void refresh();
    void fitHistory();
    double viewEnd() const { return end_; }
    void setDisplayPaused(bool);
    bool isDisplayPaused() const { return displayPaused; }
    std::vector<Sample> recordedSamples(quint32 node = 0) const;
    std::vector<TargetEvent> targetEvents() const;
    void resetView();
    void clearDisplay();
    std::array<bool, channelCount> visible = {true, true, false, false, false, false, false, false, false};
    std::array<QColor, channelCount> colors = {QColor("#55b3ff"), QColor("#5fc992"), QColor("#ffbc33"),
                                               QColor("#c49bff"), QColor("#FF6363"), QColor("#f3b17a"),
                                               QColor("#67dfdf"), QColor("#c0c0c0"), QColor("#ed91bc"),
                                               QColor("#ff8080"), QColor("#50c8ff"), QColor("#50ffa0")};
    QStringList aliases = channelNames;
    int view = 0;
    bool debugProfile = false;
    QString timeAxisLabel; // Optional acquisition-clock label for imported/captured data.
    int axisChannel = 0; // Numeric Y ticks for this channel; other scales remain independent.
    double windowSeconds = 5, t1 = 0, t2 = 0;
    bool cursors = false, follow = true;
    int decimation = 1;
    std::vector<Sample> samples() const;
  signals:
    void cursorChanged(double, double);

  protected:
    void paintEvent(QPaintEvent *) override;
    void wheelEvent(QWheelEvent *) override;
    void mousePressEvent(QMouseEvent *) override;
    void mouseMoveEvent(QMouseEvent *) override;
    void mouseReleaseEvent(QMouseEvent *) override;

  private:
    std::vector<Sample> data_;
    std::vector<TargetEvent> captureEvents_;
    Sampling *history_ = nullptr;
    std::shared_ptr<Sampling> frozen_;
    Sampling *displayHistory() const { return frozen_ ? frozen_.get() : history_; }
    bool displayPaused = false;
    int source_ = 0;
    quint64 sourceRevision_ = UINT64_MAX;
    HistoryInfo info_;
    PlotFrame frame_;
    QPixmap base_;
    QString cacheKey_;
    bool dirty_ = true;
    void drawBase(QPainter &);
    void drawOverlay(QPainter &);
    QRectF plot_;
    double end_ = 5;
    int drawnAxis_ = -1;
    std::array<std::optional<std::pair<double, double>>, channelCount> manualY_{};
    std::array<std::pair<double, double>, channelCount> drawnY_{};
    QPointF last_, rubberStart_, rubberEnd_;
    bool dragging_ = false, selecting_ = false;
    int dragCursor_ = 0;
    double timeAt(double x) const;
    int scaleGroup(int c) const;
};
} // namespace mt
