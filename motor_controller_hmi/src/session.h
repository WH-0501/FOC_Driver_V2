#pragma once
#include "backend.h"
#include "sampling.h"
#include "sweep_recording.h"
#include <QJsonObject>
#include <QMutex>
#include <QThread>
#include <QWaitCondition>
#include <atomic>
#include <chrono>
#include <deque>
#include <functional>
namespace mt {
struct Request {
    quint32 node = 1;
    int cmd = 0;
    QByteArray payload;
    quint64 token = 0;
    int expectedMode = -1;
    qint64 enteredNs = 0, enqueuedNs = 0;
    std::chrono::steady_clock::time_point queuedAt = std::chrono::steady_clock::now();
};
class Session : public QThread {
    Q_OBJECT
  public:
    using BackendFactory = std::function<std::unique_ptr<CanBackend>(const Connection &)>;
    explicit Session(QObject *parent = nullptr, BackendFactory factory = {});
    ~Session() override;
    void open(Connection);
    void shutdown();
    quint64 command(quint32, int, const QByteArray & = {}, int expectedMode = -1,
                    const QString &operation = {});
    void emergency();
    void upgrade(quint32, const QByteArray &);
    void selectNode(quint32);
    quint32 selectedNode() const { return node_; }
    bool connected() const { return connected_; }
    bool upgrading() const { return upgrading_; }
    std::optional<Sample> latest(quint32 node) const;
    bool telemetryFresh(quint32 node) const;
    /** Last sample enabled bit; false when telemetry is stale (safe gating default). */
    bool motorEnabled(quint32 node) const;
    QJsonObject parameters(quint32 node) const;
    QJsonObject faultSnapshot() const;
    void beginSweepRecording(quint32 node, quint16 run);
    SweepRecording sweepRecording() const;
    Sampling sampling;
    std::array<Sampling, 3> debugSampling{Sampling(120000), Sampling(120000), Sampling(120000)};
    std::atomic<quint64> rxCount{0}, txCount{0};
  signals:
    void connectionChanged(bool, QString);
    void log(QString level, QString message);
    void sourceLog(QString level, QString message, qint64 wallMs, qint64 originNs);
    void replied(quint64 token, quint32 node, int command, QJsonObject result);
    void commandFailed(quint64 token, QString reason);
    void nodeSeen(quint32);
    void iapProgress(int percent, QString phase);
    void iapFinished(bool, QString);

  protected:
    void run() override;

  private:
    SweepRecording sweepRecording_;
    Connection config_;
    BackendFactory backendFactory_;
    mutable QMutex mutex_;
    QWaitCondition wake_;
    std::atomic<bool> rxReady_{false};
    std::deque<Request> queue_;
    QMap<quint32, Sample> latest_;
    QMap<quint32, std::chrono::steady_clock::time_point> receivedAt_;
    QMap<quint32, QJsonObject> params_;
    QJsonObject faultSnapshot_;
    QByteArray image_;
    quint32 iapNode_ = 1;
    std::atomic<bool> stopping_{false}, urgent_{false}, connected_{false}, upgrading_{false};
    std::atomic<quint32> node_{1};
    std::atomic<quint64> token_{0};
    std::atomic<qint64> originNs_{0};
};
} // namespace mt
