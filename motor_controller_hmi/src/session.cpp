#include "session.h"
#include <QDateTime>
#include <QElapsedTimer>
#include <QMutexLocker>
#include <QSet>
#include <cmath>
namespace mt {
Session::Session(QObject *p, BackendFactory factory) : QThread(p), backendFactory_(std::move(factory)) {
    connect(
        this, &Session::log, this,
        [this](QString level, QString message) {
            if (level != "ACTION")
                emit sourceLog(level, message, QDateTime::currentMSecsSinceEpoch(), monotonicNs());
        },
        Qt::DirectConnection);
    connect(
        this, &Session::commandFailed, this,
        [this](quint64 token, QString reason) {
            sampling.updateTargetEvent(token, 3, (monotonicNs() - originNs_) / 1e9, reason);
        },
        Qt::DirectConnection);
}
Session::~Session() {
    shutdown();
    wait();
}
void Session::open(Connection c) {
    if (isRunning())
        return;
    config_ = c;
    originNs_ = monotonicNs();
    stopping_ = urgent_ = false;
    rxCount = txCount = 0;
    {
        QMutexLocker l(&mutex_);
        queue_.clear();
        latest_.clear();
        receivedAt_.clear();
        params_.clear();
        image_.clear();
    }
    sampling.configure(c.historyCapacity);
    for (auto &history : debugSampling)
        history.clear();
    sampling.pause(false);
    start();
}
void Session::shutdown() {
    stopping_ = true;
    urgent_ = true;
    wake_.wakeAll();
}
quint64 Session::command(quint32 n, int c, const QByteArray &p, int expectedMode, const QString &operation) {
    const auto enteredNs = monotonicNs();
    const auto enteredWallMs = QDateTime::currentMSecsSinceEpoch();
    auto t = ++token_;
    QString action = operation;
    if (action.isEmpty()) {
        switch (c) {
        case 0x01:
            action = "下发目标位置";
            break;
        case 0x02:
            action = "下发目标速度";
            break;
        case 0x03:
            action = "下发目标电流";
            break;
        case 0x23:
        case 0x30:
            action = "下发组合目标";
            break;
        case 0x39:
            action = "异常复位";
            break;
        case 0x40:
            action = p == QByteArray(1, 1) ? "使能电机" : "失能电机";
            break;
        case 0x41:
            action = QString("应用控制模式：%1").arg(p.isEmpty() ? -1 : quint8(p[0]));
            break;
        case 0x52:
            action = "扫描节点";
            break;
        case 0xb5:
            action = "读取限速（手动或连接后首次读取）";
            break;
        case 0xb6:
            action = "发送限速配置";
            break;
        case 0xb8:
            action = p.size() >= 4 ? QString("%1调试上报 · mask=%2 · %3 Hz/环")
                                         .arg(p[1] ? "启用/应用" : "停止")
                                         .arg(quint8(p[1]))
                                         .arg(u16(p, 2))
                                   : "配置调试上报";
            break;
        default:
            action = "协议操作";
            break;
        }
    }
    QMutexLocker l(&mutex_);
    if (!connected_ || upgrading_ || queue_.size() >= 256) {
        sampling.recordTargets(t, (monotonicNs() - originNs_) / 1e9, decodeTargets(c, p, config_.version));
        QMetaObject::invokeMethod(
            this,
            [this, t, n, c] {
                emit commandFailed(t, QString("Node %1 · 0x%2：连接未就绪、升级占用或命令队列已满")
                                          .arg(n)
                                          .arg(c, 2, 16, QChar('0')));
            },
            Qt::QueuedConnection);
        return t;
    }
    Request request{n, c, p, t, expectedMode};
    request.enteredNs = enteredNs;
    request.enqueuedNs = monotonicNs();
    bool disabling = c == 0x39 || (c == 0x40 && p == QByteArray(1, 0));
    if (disabling) {
        // A user stop must not sit behind an unanswered enable or motion command.
        for (auto it = queue_.begin(); it != queue_.end();) {
            if (it->node == n && (it->cmd == 0x40 || it->cmd == 0x41 || (it->cmd >= 1 && it->cmd <= 0x30))) {
                auto cancelled = it->token;
                QMetaObject::invokeMethod(
                    this,
                    [this, cancelled] {
                        emit commandFailed(cancelled, "已取消：失能/复位请求撤销未执行的运动/模式操作");
                    },
                    Qt::QueuedConnection);
                it = queue_.erase(it);
            } else
                ++it;
        }
        request.enqueuedNs = monotonicNs();
        request.queuedAt = std::chrono::steady_clock::now();
        queue_.push_front(request);
    } else {
        request.enqueuedNs = monotonicNs();
        request.queuedAt = std::chrono::steady_clock::now();
        queue_.push_back(request);
    }
    sampling.recordTargets(t, (request.enqueuedNs - originNs_) / 1e9, decodeTargets(c, p, config_.version));
    emit sourceLog("ACTION",
                   QString("请求#%1 · Node %2 · %3 · CMD=0x%4 · 数据=%5 · 待发送")
                       .arg(t)
                       .arg(n)
                       .arg(action)
                       .arg(c, 2, 16, QChar('0'))
                       .arg(p.isEmpty() ? "无" : QString::fromLatin1(p.toHex(' '))),
                   enteredWallMs, enteredNs);
    l.unlock();
    wake_.wakeAll();
    emit log("ACTION", QString("请求#%1 · Node %2 · %3 · CMD=0x%4 · 数据=%5 · 待发送")
                           .arg(t)
                           .arg(n)
                           .arg(action)
                           .arg(c, 2, 16, QChar('0'))
                           .arg(p.isEmpty() ? "无" : QString::fromLatin1(p.toHex(' '))));
    return t;
}
void Session::emergency() {
    urgent_ = true;
    wake_.wakeAll();
}
void Session::upgrade(quint32 n, const QByteArray &b) {
    firmwareImage(b);
    QMutexLocker l(&mutex_);
    if (!connected_ || upgrading_)
        throw std::runtime_error("连接未就绪或正在升级");
    iapNode_ = n;
    image_ = b;
    upgrading_ = true;
}
void Session::selectNode(quint32 n) {
    QMutexLocker selection(&mutex_);
    for (auto &history : debugSampling)
        history.clear();
    node_ = n;
    sampling.clear();
}
std::optional<Sample> Session::latest(quint32 n) const {
    QMutexLocker l(&mutex_);
    if (!latest_.contains(n))
        return {};
    return latest_[n];
}
bool Session::telemetryFresh(quint32 n) const {
    QMutexLocker l(&mutex_);
    return connected_ && receivedAt_.contains(n) &&
           std::chrono::steady_clock::now() - receivedAt_.value(n) < std::chrono::milliseconds(750);
}
QJsonObject Session::parameters(quint32 n) const {
    QMutexLocker l(&mutex_);
    return params_.value(n);
}
QJsonObject Session::faultSnapshot() const {
    QMutexLocker l(&mutex_);
    return faultSnapshot_;
}
void Session::run() {
    std::unique_ptr<CanBackend> backend;
    QString connectionError;
    quint8 counter = 0;
    QElapsedTimer clock;
    clock.start();
    QMap<quint32, double> lastSeen;
    QSet<quint32> nodes;
    QSet<quint32> staleWarned;
    struct Pending {
        Request r;
        double deadline;
        double sentAt = 0;
        quint8 wireCounter = 0;
        bool acknowledged = false;
        QJsonObject apiReply;
        bool needsAck = false;
        QJsonObject deferredReply;
    };
    std::vector<Pending> pending;
    std::deque<Pending> expired;
    std::vector<QByteArray> chunks;
    int phase = 0;
    size_t chunk = 0;
    double deadline = 0, lastQuery = 0;
    bool queryNext = false;
    auto now = [&] { return (monotonicNs() - originNs_) / 1e9; };
    double lastInvalidDebugWarning = -2;
    std::array<DebugClock, 3> debugClocks;
    std::array<quint32, 3> debugNodes{}, debugEpochs{};
    std::array<bool, 3> debugActive{};
    QMap<quint32, int> debugMasks;
    struct TimingTrace {
        quint64 token;
        quint32 node;
        int cmd;
        quint8 cnt;
        qint64 entry, queued, txBegin, txEnd;
    };
    std::deque<TimingTrace> traces;
    QMap<quint32, Sample> submittedTargets;
    QMap<quint32, std::array<double, 3>> submittedAt;
    auto send = [&](quint32 n, int c, const QByteArray &p = QByteArray(), bool iap = false) {
        quint8 sequence = counter++;
        auto bytes = pack(sequence, quint8(c), p, iap);
        double started = now();
        backend->send({n, bytes, now()});
        txCount++;
        if (c == 0xb8 && p.size() >= 4)
            debugMasks[n] = quint8(p[1]);
        if (!iap)
            for (const auto &update : decodeTargets(c, p, config_.version)) {
                auto &state = submittedTargets[update.node];
                for (int ch = 9; ch < normalChannelCount; ++ch)
                    if (std::isfinite(update.values[ch])) {
                        state.values[ch] = update.values[ch];
                        submittedAt[update.node][ch - 9] = started;
                    }
            }
        if (c != 0xfa)
            emit log("TX", QString("Node %1 · %2 | 驱动提交 %3 ms")
                               .arg(n)
                               .arg(QString::fromLatin1(bytes.toHex(' ')))
                               .arg((now() - started) * 1000, 0, 'f', 3));
        return sequence;
    };
    auto cancelQueue = [&](const QString &why) {
        {
            QMutexLocker l(&mutex_);
            for (auto &r : queue_)
                emit commandFailed(r.token, why);
            queue_.clear();
        }
        for (auto &p : pending)
            emit commandFailed(p.r.token, why);
        pending.clear();
    };
    auto iapFinish = [&](bool ok, const QString &reason) {
        phase = 0;
        upgrading_ = false;
        sampling.pause(false);
        {
            QMutexLocker l(&mutex_);
            image_.clear();
        }
        emit iapFinished(ok, reason);
    };
    auto stop = [&] {
        cancelQueue("急停取消未执行操作");
        if (upgrading_)
            iapFinish(false, "升级已取消；设备可能仍处于 IAP，请重新上电检查");
        send(0, 0x40, QByteArray(1, 0));
        send(0, 0x39);
        emit log("WARNING", "急停：广播失能并异常复位，等待遥测确认");
    };
    try {
        backend = backendFactory_ ? backendFactory_(config_) : createBackend(config_);
        backend->setReceiveNotifier([this] {
            QMutexLocker notificationLock(&mutex_);
            rxReady_ = true;
            wake_.wakeAll();
        });
        backend->open(config_);
        connected_ = true;
        emit connectionChanged(true,
                               backend->description().isEmpty() ? config_.backend : backend->description());
        while (!stopping_) {
            double t = now();
            while (!expired.empty() && (t - expired.front().sentAt > 30 || expired.size() > 256))
                expired.pop_front();
            if (urgent_.exchange(false))
                stop();
            if (upgrading_ && !phase) {
                auto state = latest(iapNode_);
                if (!state || state->enabled || t - lastSeen.value(iapNode_, -100) > 0.75) {
                    iapFinish(false, "升级要求目标节点新鲜遥测确认已失能");
                } else {
                    cancelQueue("升级独占链路");
                    {
                        QMutexLocker l(&mutex_);
                        chunks = firmwareChunks(image_);
                    }
                    sampling.pause(true);
                    phase = 1;
                    chunk = 0;
                    deadline = t + 3;
                    lastQuery = t;
                    send(iapNode_, 0xf8);
                    send(iapNode_, 0xfc);
                    emit iapProgress(0, "ENTER · 等待 READY");
                }
            }
            if (!upgrading_) {
                Request r;
                bool have = false;
                {
                    QMutexLocker l(&mutex_);
                    if (!queue_.empty()) {
                        auto &first = queue_.front();
                        bool disabling = first.cmd == 0x40 && first.payload == QByteArray(1, 0);
                        bool conflict = false;
                        for (auto &p : pending)
                            if (p.r.node == first.node &&
                                responseCommand(p.r.cmd) == responseCommand(first.cmd))
                                conflict = true;
                        if (!conflict || disabling) {
                            r = first;
                            queue_.pop_front();
                            have = true;
                        }
                    }
                }
                if (have) {
                    if (r.cmd == 0x39 || (r.cmd == 0x40 && r.payload == QByteArray(1, 0))) {
                        for (auto it = pending.begin(); it != pending.end();) {
                            if (it->r.node == r.node && (it->r.cmd == 0x40 || it->r.cmd == 0x41)) {
                                emit commandFailed(it->r.token,
                                                   "已取消：失能/复位请求优先于尚未确认的使能/模式操作");
                                it = pending.erase(it);
                            } else
                                ++it;
                        }
                    }
                    QString rejection;
                    auto s = latest(r.node);
                    bool fresh = s && t - lastSeen.value(r.node, -100) < 0.75;
                    bool enabling = r.cmd == 0x40 && !r.payload.isEmpty() && r.payload[0] != 0;
                    bool motion = r.cmd >= 1 && r.cmd <= 0x30;
                    bool limits = r.cmd == 0xb6;
                    if (r.cmd == 0x40 && (r.payload.size() != 1 || quint8(r.payload[0]) > 1))
                        rejection = "使能/失能参数必须为单字节 00 或 01";
                    else if (r.node == 0 && (enabling || requiresDisabled(r.cmd) || motion || limits))
                        rejection = "高风险命令仅允许单播已确认节点";
                    else if ((requiresDisabled(r.cmd) || enabling || motion || limits) && !fresh)
                        rejection = "缺少 750ms 内的目标节点遥测";
                    else if (requiresDisabled(r.cmd) && s->enabled)
                        rejection = "必须先失能并等待遥测确认";
                    else if (enabling && (s->error || (config_.version == Version::V2 && s->mode == 0)))
                        rejection = "故障或 IDLE 模式不允许使能";
                    else if (motion && (!s->enabled || s->error))
                        rejection = "运动指令要求已使能且无故障";
                    else if (motion && r.expectedMode >= 0 && s->mode != r.expectedMode)
                        rejection = "设备模式已变化，请重新确认目标给定";
                    else if (r.cmd >= 1 && r.cmd <= 3 &&
                             !modeAcceptsTarget(config_.version, s->mode, r.cmd - 1))
                        rejection = "设备当前模式不支持该目标给定";
                    else if (limits && s->error)
                        rejection = "故障状态不允许修改限速配置";
                    else if (r.cmd == 0xad && (!fresh || !s->enabled))
                        rejection = "KTH71 校准要求新鲜使能状态";
                    if (!rejection.isEmpty())
                        emit commandFailed(r.token, QString("Node %1 · 0x%2：%3")
                                                        .arg(r.node)
                                                        .arg(r.cmd, 2, 16, QChar('0'))
                                                        .arg(rejection));
                    else {
                        double sentAt = now();
                        const auto txBeginNs = monotonicNs();
                        quint8 sequence;
                        try {
                            sequence = send(r.node, r.cmd, r.payload);
                        } catch (const std::exception &e) {
                            emit commandFailed(r.token, QString::fromUtf8(e.what()));
                            throw;
                        }
                        sampling.updateTargetEvent(r.token, 1, now());
                        const auto txEndNs = monotonicNs();
                        traces.push_back({r.token, r.node, r.cmd, sequence, r.enteredNs, r.enqueuedNs,
                                          txBeginNs, txEndNs});
                        while (traces.size() > 256 ||
                               (!traces.empty() && txEndNs - traces.front().txBegin > 30000000000LL))
                            traces.pop_front();
                        emit log("TIMING", QString("请求#%1 Node %2 CNT=%3 CMD=0x%4 · 入口→入队=%5 ms · "
                                                   "入队→发送=%6 ms · 入口→发送=%7 ms · TX调用及记录=%8 ms")
                                               .arg(r.token)
                                               .arg(r.node)
                                               .arg(sequence, 2, 16, QChar('0'))
                                               .arg(r.cmd, 2, 16, QChar('0'))
                                               .arg((r.enqueuedNs - r.enteredNs) / 1e6, 0, 'f', 3)
                                               .arg((txBeginNs - r.enqueuedNs) / 1e6, 0, 'f', 3)
                                               .arg((txBeginNs - r.enteredNs) / 1e6, 0, 'f', 3)
                                               .arg((txEndNs - txBeginNs) / 1e6, 0, 'f', 3));
                        emit log("INFO", QString("请求#%4 · CNT=%1 CMD=0x%2 · 主机排队 %3 ms")
                                             .arg(sequence, 2, 16, QChar('0'))
                                             .arg(r.cmd, 2, 16, QChar('0'))
                                             .arg(std::chrono::duration<double, std::milli>(
                                                      std::chrono::steady_clock::now() - r.queuedAt)
                                                          .count() -
                                                      (now() - sentAt) * 1000,
                                                  0, 'f', 3)
                                             .arg(r.token));
                        if (r.cmd >= 0x39 && r.cmd != 0x7f && r.node) {
                            bool ambiguous =
                                std::any_of(expired.begin(), expired.end(), [&](const Pending &old) {
                                    return old.r.node == r.node &&
                                           responseCommand(old.r.cmd) == responseCommand(r.cmd);
                                });
                            pending.push_back({r,
                                               now() + ((r.cmd == 0xab || r.cmd == 0xb1) ? 30 : 2),
                                               sentAt,
                                               sequence,
                                               false,
                                               {},
                                               ambiguous,
                                               {}});

                        } else
                            emit replied(r.token, r.node, r.cmd, {{"sent", true}});
                    }
                }
            }
            for (auto f : backend->receive()) {
                const auto parsedNs = monotonicNs();
                rxCount++;
                if (f.bytes.size() < 2)
                    continue;
                int c = quint8(f.bytes[1]);
                auto payload = f.bytes.mid(2);
                if (c == 0) {
                    const double acquisitionTime =
                        f.adapterRxValid ? f.time : std::numeric_limits<double>::quiet_NaN();
                    // Host-event analysis uses receive time, never an adapter-clock offset.
                    f.time = ((f.hostRxNs ? f.hostRxNs : parsedNs) - originNs_) / 1e9;
                    auto decoded = decode(f, config_.version);
                    if (!decoded)
                        continue;
                    auto s = *decoded;
                    s.acquisitionTime = acquisitionTime;
                    if (submittedTargets.contains(f.node)) {
                        const auto &reference = submittedTargets[f.node];
                        for (int ch = 9; ch < normalChannelCount; ++ch)
                            if (s.time >= submittedAt[f.node][ch - 9])
                                s.values[ch] = reference.values[ch];
                    }
                    {
                        QMutexLocker l(&mutex_);
                        if (f.node == node_ && s.error &&
                            (!latest_.contains(f.node) || !latest_[f.node].error)) {
                            faultSnapshot_ = QJsonObject{
                                {"captured_at", QDateTime::currentDateTimeUtc().toString(Qt::ISODate)},
                                {"node", int(f.node)},
                                {"sample_time", s.time},
                                {"error", s.error},
                                {"error_name", telemetryErrorName(s.error, config_.version)},
                                {"mode", s.mode},
                                {"enabled", s.enabled},
                                {"vbus", s.values[7]},
                                {"temperature", s.values[8]},
                                {"parameters", params_.value(f.node)},
                                {"note", "参数为故障前最近成功回读缓存；未读取的字段不会伪造"}};
                        }
                        latest_[f.node] = s;
                        receivedAt_[f.node] = std::chrono::steady_clock::now();
                    }
                    lastSeen[f.node] = t;
                    staleWarned.remove(f.node);
                    // API acceptance and dispatch ACK are not motor-state confirmation.
                    // Only a newly timestamped sample after this TX can complete
                    // 0x40/0x41.
                    for (auto it = pending.begin(); it != pending.end();) {
                        bool stateCommand = it->r.cmd == 0x40 || it->r.cmd == 0x41;
                        bool match = stateCommand && it->r.node == f.node && !it->r.payload.isEmpty() &&
                                     s.time >= it->sentAt;
                        if (match && it->r.cmd == 0x40)
                            match = s.enabled == (it->r.payload[0] != 0) && (!s.enabled || !s.error);
                        if (match && it->r.cmd == 0x41)
                            match = s.mode == quint8(it->r.payload[0]);
                        if (match) {
                            QString message = it->r.cmd == 0x41 ? "遥测已确认模式生效"
                                              : s.enabled       ? "遥测已确认使能"
                                                                : "遥测已确认失能";
                            emit replied(it->r.token, f.node, it->r.cmd,
                                         {{"ok", true},
                                          {"telemetry_confirmed", true},
                                          {"enabled", s.enabled},
                                          {"mode", s.mode},
                                          {"api_reply_received", !it->apiReply.isEmpty()},
                                          {"dispatch_ack_received", it->acknowledged},
                                          {"message", message}});
                            it = pending.erase(it);
                        } else
                            ++it;
                    }
                    if (!nodes.contains(f.node)) {
                        nodes.insert(f.node);
                        emit nodeSeen(f.node);
                    }
                    {
                        QMutexLocker selection(&mutex_);
                        if (f.node == node_)
                            sampling.append(s);
                    }
                    continue;
                }
                auto data = response(quint8(c), payload, config_.version);
                if (!data["valid"].toBool()) {
                    bool debug = c >= 0xb9 && c <= 0xbb;
                    if (!debug || now() - lastInvalidDebugWarning >= 1) {
                        emit log(
                            "WARNING",
                            QString("拒绝格式不完整或包含非有限值的应答 0x%1").arg(c, 2, 16, QChar('0')));
                        if (debug)
                            lastInvalidDebugWarning = now();
                    }
                    continue;
                }
                if (c >= 0xb9 && c <= 0xbb) {
                    auto decoded = decodeDebug(f);
                    if (!decoded)
                        continue;
                    int loop = c - 0xb9;
                    QMutexLocker selection(&mutex_);
                    if (f.node == node_) {
                        if (debugNodes[loop] != f.node) {
                            debugNodes[loop] = f.node;
                            debugClocks[loop] = {};
                            ++debugEpochs[loop];
                        }
                        if (debugActive[loop] != decoded->enabled)
                            ++debugEpochs[loop];
                        debugActive[loop] = decoded->enabled;
                        bool restarted;
                        auto time = debugClocks[loop].advance(quint32(decoded->values[18]), restarted, now());
                        if (restarted) {
                            ++debugEpochs[loop];
                            emit log("INFO",
                                     QString("PID时间不连续：Node %1 环 "
                                             "%2，原始tick=%3；保留历史并分段，跨段时间不用于阶跃测量")
                                         .arg(f.node)
                                         .arg(loop + 1)
                                         .arg(decoded->values[18], 0, 'f', 0));
                        }
                        if (time) {
                            decoded->time = *time;
                            decoded->streamEpoch = debugEpochs[loop];
                            debugSampling[loop].append(*decoded);
                        }
                    }
                    data["received_utc_ms"] = double(QDateTime::currentMSecsSinceEpoch());
                    auto o = params_.value(f.node);
                    o[QString::number(c, 16)] = data;
                    params_[f.node] = o;
                    continue;
                }
                emit log("RX",
                         QString("Node %1 · %2").arg(f.node).arg(QString::fromLatin1(f.bytes.toHex(' '))));
                if (c == 0x7e) {
                    if (payload.size() >= 2) {
                        auto matches = [&](const TimingTrace &trace) {
                            return trace.node == f.node && trace.cnt == quint8(payload[0]) &&
                                   trace.cmd == quint8(payload[1]) &&
                                   parsedNs - trace.txBegin < 30000000000LL;
                        };
                        const auto count = std::count_if(traces.begin(), traces.end(), matches);
                        if (count == 1) {
                            auto it = std::find_if(traces.begin(), traces.end(), matches);
                            const qint64 rxNs = f.hostRxNs ? f.hostRxNs : parsedNs;
                            emit log(
                                "TIMING",
                                QString("请求#%1 Node %2 CNT=%3 CMD=0x%4 · TX→%5=%6 ms · RX队列→解析=%7 "
                                        "ms · 驱动取帧调用=%8 ms · 入口→解析=%9 ms · 适配器RX=%10 "
                                        "us（独立时钟；无TX时间戳，不计算总线RTT）")
                                    .arg(it->token)
                                    .arg(f.node)
                                    .arg(it->cnt, 2, 16, QChar('0'))
                                    .arg(it->cmd, 2, 16, QChar('0'))
                                    .arg(f.hostRxNs ? "主机取ACK" : "ACK解析（无原生取帧计时）")
                                    .arg((rxNs - it->txBegin) / 1e6, 0, 'f', 3)
                                    .arg((parsedNs - rxNs) / 1e6, 0, 'f', 3)
                                    .arg(f.readStartNs ? QString::number((rxNs - f.readStartNs) / 1e6, 'f', 3)
                                                       : "不可用")
                                    .arg((parsedNs - it->entry) / 1e6, 0, 'f', 3)
                                    .arg(f.adapterRxUs ? QString::number(*f.adapterRxUs) +
                                                             (f.adapterRxValid ? "（有效范围）"
                                                                               : "（异常范围，仅保存原值）")
                                                       : "不可用"));
                            sampling.updateTargetEvent(it->token, 2, (rxNs - originNs_) / 1e9);
                            traces.erase(it);
                        } else if (count > 1) {
                            emit log("TIMING", "ACK计数器复用：对应请求不唯一，不计算延迟");
                        }
                    }
                    for (auto it = expired.begin(); it != expired.end(); ++it)
                        if (it->r.node == f.node && payload.size() >= 2 &&
                            it->wireCounter == quint8(payload[0]) && it->r.cmd == quint8(payload[1])) {
                            emit log("WARNING", QString("Node %1 · 收到旧请求的迟到 ACK：CNT=%2 "
                                                        "CMD=0x%3，延迟 %4 ms；不作为当前操作成功")
                                                    .arg(f.node)
                                                    .arg(it->wireCounter, 2, 16, QChar('0'))
                                                    .arg(it->r.cmd, 2, 16, QChar('0'))
                                                    .arg((now() - it->sentAt) * 1000, 0, 'f', 1));
                            for (auto &entry : pending)
                                if (entry.r.node == f.node && entry.needsAck &&
                                    responseCommand(entry.r.cmd) == responseCommand(it->r.cmd))
                                    entry.deferredReply = {};
                            expired.erase(it);
                            break;
                        }
                    for (auto &entry : pending)
                        if (entry.r.node == f.node && payload.size() >= 2 &&
                            entry.wireCounter == quint8(payload[0]) && entry.r.cmd == quint8(payload[1])) {
                            entry.acknowledged = true;
                            emit log("INFO", QString("Node %1 · ACK CNT=%2 CMD=0x%3 · 往返 %4 ms（分发确认）")
                                                 .arg(f.node)
                                                 .arg(entry.wireCounter, 2, 16, QChar('0'))
                                                 .arg(entry.r.cmd, 2, 16, QChar('0'))
                                                 .arg((now() - entry.sentAt) * 1000, 0, 'f', 3));
                        }
                    for (auto it = pending.begin(); it != pending.end();) {
                        if (!it->acknowledged || it->deferredReply.isEmpty()) {
                            ++it;
                            continue;
                        }
                        auto result = it->deferredReply;
                        it->deferredReply = {};
                        if ((it->r.cmd == 0x40 || it->r.cmd == 0x41) && result["ok"].toBool()) {
                            it->apiReply = result;
                            ++it;
                        } else {
                            int response = responseCommand(it->r.cmd);
                            {
                                QMutexLocker l(&mutex_);
                                params_[it->r.node][QString::number(response, 16)] = result;
                            }
                            emit replied(it->r.token, it->r.node, response, result);
                            it = pending.erase(it);
                        }
                    }
                    if (payload.size() >= 2 && quint8(payload[1]) == 0x50) {
                        for (auto it = pending.begin(); it != pending.end(); ++it)
                            if (it->r.node == f.node && it->r.cmd == 0x50 &&
                                it->wireCounter == quint8(payload[0])) {
                                emit replied(
                                    it->r.token, f.node, 0x50,
                                    {{"dispatched", true}, {"message", "仅确认分发，请回读位置确认原点"}});
                                pending.erase(it);
                                break;
                            }
                    }
                    continue; // Dispatch ACK never implies hardware success.
                }
                if (c == 0x52 && !nodes.contains(f.node)) {
                    nodes.insert(f.node);
                    emit nodeSeen(f.node);
                }
                bool deferred = false;
                for (auto &entry : pending)
                    if (entry.r.node == f.node && responseCommand(entry.r.cmd) == c && entry.needsAck &&
                        !entry.acknowledged) {
                        entry.deferredReply = data;
                        deferred = true;
                        break;
                    }
                if (deferred)
                    continue; // A same-command retry cannot claim an older untagged typed
                              // result.
                {
                    QMutexLocker l(&mutex_);
                    auto o = params_.value(f.node);
                    o[QString::number(c, 16)] = data;
                    params_[f.node] = o;
                }
                bool matched = false;
                for (auto it = pending.begin(); it != pending.end(); ++it) {
                    bool nodeMatch = it->r.node == f.node || (c == 0x4c && it->r.payload.size() >= 2 &&
                                                              u16(it->r.payload, 0) == f.node);
                    if (nodeMatch && (responseCommand(it->r.cmd) == c || it->r.cmd == c)) {
                        if ((it->r.cmd == 0x40 || it->r.cmd == 0x41) && data["ok"].toBool()) {
                            it->apiReply = data;
                            matched = true;
                            break; // Acceptance still waits for the requested telemetry
                                   // state.
                        }
                        emit replied(it->r.token, f.node, c, data);
                        pending.erase(it);
                        matched = true;
                        break;
                    }
                }
                if (!matched)
                    emit replied(0, f.node, c, data);
                if (phase && c == 0xfc && f.node == iapNode_ && payload.size() >= 2) {
                    int state = quint8(payload[0]), err = quint8(payload[1]);
                    if (err != 0x80) {
                        iapFinish(false, QString("IAP 状态 %1 / 错误 0x%2；当前整包引擎要求 ASIX "
                                                 "0x80 状态格式")
                                             .arg(state)
                                             .arg(err, 2, 16, QChar('0')));
                        continue;
                    }
                    if (phase == 1 && state == 1) {
                        send(iapNode_, 0xf9, chunks[0], true);
                        chunk = 1;
                        phase = 2;
                        deadline = t + 120;
                        emit iapProgress(1, "START · 等待擦除应答");
                    } else if (phase == 2 && state == 2) {
                        phase = 3;
                        deadline = t + 3;
                        emit iapProgress(2, "DATA · 传输固件");
                    } else if (phase == 4 && state == 3) {
                        send(iapNode_, 0xfe);
                        send(iapNode_, 0xfc);
                        phase = 5;
                        deadline = t + 10;
                        lastQuery = t;
                        emit iapProgress(98, "VERIFY · 校验固件");
                    } else if (phase == 5 && state == 5) {
                        send(iapNode_, 0xff);
                        iapFinish(true, "固件校验通过，已发送 RESET；请重新连接验证版本");
                    }
                }
            }
            if (phase == 3) {
                if (chunk < chunks.size()) {
                    send(iapNode_, 0xfa, chunks[chunk++], true);
                    emit iapProgress(int(chunk * 95 / chunks.size()), "DATA");
                    deadline = t + 3;
                } else {
                    send(iapNode_, 0xfb);
                    send(iapNode_, 0xfc);
                    phase = 4;
                    deadline = t + 5;
                    lastQuery = t;
                }
            }
            if (phase && t > deadline)
                iapFinish(false, "IAP 等待超时；未自动重发 Flash 数据，请检查设备状态");
            if ((phase == 1 || phase == 4 || phase == 5) && t - lastQuery > .5) {
                send(iapNode_, 0xfc);
                lastQuery = t;
            }
            for (auto it = pending.begin(); it != pending.end();)
                if (t > it->deadline) {
                    QString reason;
                    if (it->r.cmd == 0x40 || it->r.cmd == 0x41) {
                        auto state = latest(it->r.node);
                        bool fresh = state && t - lastSeen.value(it->r.node, -100) < .75;
                        QString action = it->r.cmd == 0x41                                ? "模式切换"
                                         : (!it->r.payload.isEmpty() && it->r.payload[0]) ? "使能"
                                                                                          : "失能";
                        QString actual = !fresh ? "没有新鲜遥测，无法确认电机状态"
                                                : QString("设备当前%1，模式 %2")
                                                      .arg(state->enabled ? "仍为使能" : "为失能")
                                                      .arg(state->mode);
                        reason =
                            QString("Node %1 · %2未确认（2 秒）：%3。%4")
                                .arg(it->r.node)
                                .arg(action, actual,
                                     it->apiReply.isEmpty() ? "未收到专用成功应答；分发 ACK 不代表执行成功"
                                                            : "已收到成功应答，但遥测未确认目标状态");
                    } else
                        reason = QString("Node %1 · 0x%2 专用应答超时；0x7E 不表示成功")
                                     .arg(it->r.node)
                                     .arg(it->r.cmd, 2, 16, QChar('0'));
                    emit commandFailed(it->r.token, reason);
                    expired.push_back(*it);
                    it = pending.erase(it);
                } else
                    ++it;
            // Warn once per telemetry outage.
            if (!upgrading_) {
                bool stale = false;
                {
                    QMutexLocker l(&mutex_);
                    for (auto it = latest_.begin(); it != latest_.end(); ++it)
                        if (it->enabled && t - lastSeen.value(it.key(), -100) > .75 &&
                            !staleWarned.contains(it.key())) {
                            stale = true;
                            staleWarned.insert(it.key()); // Never invent a disabled telemetry state.
                        }
                }
                if (stale) {
                    emit log("WARNING", "运行节点遥测超时，请检查连接和设备状态");
                }
            }
            Q_UNUSED(queryNext);
            QMutexLocker sleepLock(&mutex_);
            if (queue_.empty() && !urgent_ && !stopping_ && !rxReady_.exchange(false))
                wake_.wait(&mutex_, 1);
        }
        for (auto it = debugMasks.begin(); it != debugMasks.end(); ++it)
            if (it.value())
                send(it.key(), 0xb8, QByteArray::fromHex("01000064"));
        stop();
        backend->close();
    } catch (const std::exception &e) {
        connectionError = QString::fromUtf8(e.what());
        emit log("ERROR", connectionError);
        try {
            if (connected_)
                stop();
            if (backend)
                backend->close();
        } catch (...) {
        }
    }
    cancelQueue("连接已关闭");
    if (upgrading_)
        iapFinish(false, "连接关闭");
    connected_ = false;
    emit connectionChanged(false, connectionError.isEmpty() ? QString("已断开") : connectionError);
}
} // namespace mt
