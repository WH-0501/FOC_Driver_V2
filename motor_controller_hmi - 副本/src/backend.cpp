#include "backend.h"
#include <QCoreApplication>
#include <QDir>
#include <QElapsedTimer>
#include <QJsonArray>
#include <QJsonDocument>
#include <QLibrary>
#include <QProcess>
#include <QScopeGuard>
#include <QtEndian>
#include <atomic>
#include <cmath>
#include <deque>
#include <mutex>
#include <stdexcept>
#include <thread>
#ifdef Q_OS_WIN
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>
#endif
namespace mt {
static void fail(const QString &s) {
    throw std::runtime_error(s.toUtf8().constData());
}
class DemoBackend final : public CanBackend {
    QElapsedTimer timer;
    double next = 0, position = 0;
    bool enabled = false;
    int mode = 0, hz = 1000;
    Version version = Version::V2;
    quint32 node = 1;
    quint8 count = 0;
    double targets[3]{};
    std::deque<Frame> replies;
    QMap<int, QByteArray> parameters;
    int iapState = 0;
    bool aligned = false;
    int softRpm = 54;
    int debugMask = 0, debugHz = 100;
    double nextDebug = 0;
    double debugFeedback[3]{};
    quint32 debugTicks[3]{};
    bool hardProtection = true;

  public:
    void open(const Connection &c) override {
        version = c.version;
        timer.start();
        parameters[0x47] = pid(1, .1, 0);
        parameters[0x48] = pid(.5, .02, 0);
        parameters[0x49] = pid(.2, .001, 0);
        parameters[0x4a] = be16(4096) + be16(100);
        parameters[0x4b] = be16(80) + be16(5000) + QByteArray::fromHex("1220");
    }
    void close() override { enabled = false; }
    void send(const Frame &f) override {
        if (f.node != node && f.node != 0 && f.node != 0x7ff)
            return;
        int cmd = quint8(f.bytes[1]);
        auto p = f.bytes.mid(2);
        QByteArray result;
        bool reply = true;
        int err = requiresDisabled(cmd) && enabled ? 3 : 0;
        if (err)
            result = QByteArray(1, char(err));
        else if (cmd == 0x40) {
            enabled = !p.isEmpty() && p[0];
            result = QByteArray(1, 0);
        } else if (cmd == 0xb8 && p.size() >= 4 && quint8(p[0]) == 1) {
            int mask = quint8(p[1]), rate = u16(p, 2);
            int loops = (mask & 1) + ((mask >> 1) & 1) + ((mask >> 2) & 1);
            int error = mask > 7 || (mask && (rate < 1 || rate * loops > 1000)) ? 2 : 0;
            if (!error) {
                if (debugMask != mask || debugHz != rate)
                    nextDebug = timer.nsecsElapsed() / 1e9;
                debugMask = mask;
                debugHz = rate ? rate : 100;
            }
            result = QByteArray(1, 1) + QByteArray(1, char(error)) + QByteArray(1, char(debugMask)) +
                     be16(debugHz);
        } else if (cmd == 0x41) {
            mode = quint8(p[0]);
            result = QByteArray(1, 0);
        } else if (cmd == 0x39) {
            enabled = false;
            for (double &t : targets)
                t = 0;
            result = QByteArray(1, 0);
        } else if (cmd >= 1 && cmd <= 3 && p.size() >= 3) {
            targets[cmd - 1] =
                i16(p, 1) / (cmd == 1   ? 128.0 * (version == Version::V1 ? 180 / std::numbers::pi : 1)
                             : cmd == 3 ? 1000.0
                                        : 1);
            reply = false;
        } else if (cmd == 0x12 || cmd == 0x13 || cmd == 0x23 || cmd == 0x30) {
            int offset = 1;
            if (cmd != 0x23) {
                targets[0] = i16(p, offset) / (128.0 * (version == Version::V1 ? 180 / std::numbers::pi : 1));
                offset += 2;
            }
            if (cmd != 0x13) {
                targets[1] = i16(p, offset);
                offset += 2;
            }
            if (cmd != 0x12)
                targets[2] = i16(p, offset) / 1000.0;
            reply = false;
        } else if (parameters.contains(cmd))
            result = parameters[cmd];
        else if (cmd >= 0x42 && cmd <= 0x46) {
            parameters[cmd + 5] = p.left(cmd == 0x45 ? 4 : 6);
            result = parameters[cmd + 5];
            cmd += 5;
        } else if (cmd == 0xb5 || cmd == 0xb6) {
            int error = 0;
            if (cmd == 0xb6) {
                if (p.size() < 2 || u16(p, 0) > 3000)
                    error = 1;
                else {
                    softRpm = u16(p, 0);
                    if (p.size() >= 4)
                        hardProtection = bool(u16(p, 2) & 1);
                }
            }
            double speed = enabled ? targets[1] : 0;
            bool active = enabled && mode == (version == Version::V2 ? 1 : 2) && softRpm &&
                          std::abs(speed) > softRpm * .9 && speed * targets[2] > 0;
            double limited =
                active ? targets[2] * std::clamp((softRpm - std::abs(speed)) / (.1 * softRpm), 0.0, 1.0)
                       : targets[2];
            result = QByteArray(20, 0);
            result[0] = 3;
            result[1] = char(error);
            qToBigEndian<quint16>(3000, result.data() + 2);
            qToBigEndian<quint16>(quint16(softRpm), result.data() + 4);
            // This response uses the internal controller mode, also for V1 wire mode CST.
            result[6] = char(version == Version::V1 && mode == 2 ? 1 : mode);
            result[7] = char((hardProtection ? 2 : 0) | (active ? 1 : 0));
            qToBigEndian<qint32>(qRound(speed * 1000), result.data() + 8);
            qToBigEndian<qint32>(qRound(targets[2] * 1000), result.data() + 12);
            qToBigEndian<qint32>(enabled && result[6] == 1 ? qRound(limited * 1000) : 0, result.data() + 16);
        } else if (cmd == 0x52)
            result = QByteArray(1, 0);
        else if (cmd == 0x53) {
            result = QByteArray(16, 0);
            result[0] = 7;
            result[1] = 1;
            qToBigEndian<quint16>(1000, result.data() + 2);
            qToBigEndian<quint16>(16384, result.data() + 12);
            result[14] = aligned ? 1 : 0;
        } else if (cmd == 0xab) {
            aligned = true;
            result = QByteArray(14, 0);
            result[1] = p[0];
            result[2] = 1;
            result[3] = 8;
        } else if (cmd == 0x4c) {
            node = u16(p, 0);
            result = be16(int(node));
        } else if (cmd == 0x7f) {
            hz = u16(p, 0);
            reply = false;
        } else if (cmd == 0xfd)
            result = "V2.0.1_DEMO";
        else if (cmd == 0xf8) {
            iapState = 1;
            hz = 0;
            enabled = false;
            reply = false;
        } else if (cmd == 0xf9) {
            iapState = 2;
            replies.push_back({node, pack(count++, 0xfc, QByteArray::fromHex("0280")), 0});
            reply = false;
        } else if (cmd == 0xfa)
            reply = false;
        else if (cmd == 0xfb) {
            iapState = 3;
            reply = false;
        } else if (cmd == 0xfe) {
            iapState = 5;
            reply = false;
        } else if (cmd == 0xfc)
            result = QByteArray(1, char(iapState)) + QByteArray(1, char(0x80));
        else if (cmd == 0xff) {
            iapState = 0;
            hz = 1000;
            reply = false;
        } else if (cmd == 0xae)
            result = QByteArray(1, char(3));
        else if (cmd == 0x4d || cmd == 0x4e || cmd == 0x54 || cmd == 0xac)
            result = QByteArray(1, 1);
        else if (cmd == 0x50) {
            position = 0;
            reply = false;
        } else if (hasResult(cmd))
            result = QByteArray(1, 0);
        else
            reply = false;
        if (reply)
            replies.push_back({node, pack(count++, quint8(cmd), result), 0});
        if (cmd < 0xf8 && f.node)
            replies.push_back({node, pack(count++, 0x7e, f.bytes.left(2)), 0});
    }
    std::vector<Frame> receive() override {
        std::vector<Frame> out;
        while (!replies.empty()) {
            out.push_back(replies.front());
            replies.pop_front();
        }
        double now = timer.nsecsElapsed() / 1e9;
        int debugLimit = 0;
        while (debugMask && nextDebug <= now && debugLimit++ < 256) {
            double stamp = nextDebug;
            nextDebug += 1.0 / debugHz;
            for (int loop = 0; loop < 3; ++loop)
                if (debugMask & (1 << loop)) {
                    bool active = enabled && (loop == 0   ? mode >= 1 && mode <= 3
                                              : loop == 1 ? mode == 2 || mode == 3
                                                          : mode == 3);
                    double reference = loop == 0   ? targets[2]
                                       : loop == 1 ? targets[1] * 2 * std::numbers::pi / 60
                                                   : targets[0];
                    if (active) {
                        debugFeedback[loop] +=
                            (reference - debugFeedback[loop]) * (1 - std::exp(-1.0 / debugHz / .04));
                        debugTicks[loop] = quint32(quint64(stamp * 1e6));
                    }
                    float fields[] = {float(reference),
                                      float(debugFeedback[loop]),
                                      float(enabled ? targets[2] : 0),
                                      0,
                                      .00005f,
                                      .1f,
                                      float(reference - debugFeedback[loop]),
                                      24,
                                      float(targets[1] * 2 * std::numbers::pi / 60),
                                      1,
                                      .001f,
                                      10,
                                      5,
                                      0};
                    QByteArray p(62, 0);
                    p[0] = 1;
                    p[1] = active ? 1 : 0;
                    qToBigEndian<quint32>(debugTicks[loop], p.data() + 2);
                    for (int i = 0; i < 14; ++i) {
                        quint32 bits;
                        memcpy(&bits, &fields[i], 4);
                        qToBigEndian<quint32>(bits, p.data() + 6 + i * 4);
                    }
                    out.push_back({node, pack(count++, quint8(0xb9 + loop), p), stamp});
                }
        }
        const int reportHz = debugMask ? 5 : hz;
        if (!reportHz) {
            next = now;
            return out;
        }
        int limit = 0;
        while (next <= now && limit++ < 512) {
            double t = next;
            next += 1.0 / reportHz;
            double iq = enabled ? targets[2] + .04 * sin(t * 2 * std::numbers::pi * 12)
                                : .015 * sin(t * 2 * std::numbers::pi * 6);
            double vel = enabled ? targets[1] : 0;
            if (enabled && mode == (version == Version::V2 ? 1 : 2) && softRpm &&
                std::abs(vel) > .9 * softRpm && vel * targets[2] > 0) {
                iq = targets[2] * std::clamp((softRpm - std::abs(vel)) / (.1 * softRpm), 0.0, 1.0);
            }
            position += vel * 2 * std::numbers::pi / 60 / hz;
            QByteArray p;
            auto w = [&](double x) { p += be16(int(std::clamp(x, -32768.0, 32767.0))); };
            w(position * 128 * (version == Version::V1 ? 180 / std::numbers::pi : 1));
            w(vel);
            w(iq * 1000);
            if (version == Version::V2) {
                w(8 * sin(t * 9));
                for (int k = 0; k < 3; k++)
                    w(iq * 1000 * sin(t * 30 + k * 2 * std::numbers::pi / 3));
            }
            p.append(char((enabled ? 0x10 : 0) | mode));
            p.append(char(0));
            if (version == Version::V2) {
                p.append(char(32 + (enabled ? 2 : 0)));
                p.append(char(240));
            }
            out.push_back({node, pack(count++, 0, p), t});
        }
        return out;
    }
};
#ifdef Q_OS_WIN
#define MT_CALL __stdcall
// Both vendor ABIs are 64-bit here; ControlCANFD uses uint64 handles, ZLG uses pointers.
struct NativeFd {
    quint32 id;
    quint8 len, flags, res0, res1;
    quint8 data[64];
};
struct NativeTx {
    NativeFd frame;
    quint32 type;
};
struct NativeRx {
    NativeFd frame;
    quint64 stamp;
};
struct NativeCan {
    quint32 id;
    quint8 len, pad[3], data[8];
};
struct NativeCanRx {
    NativeCan frame;
    quint64 stamp;
};
struct NativeInit {
    quint32 type, code, mask, abit, dbit, brp;
    quint8 filter, mode;
    quint16 pad;
    quint32 reserved;
};
static_assert(sizeof(NativeTx) == 76 && sizeof(NativeRx) == 80 && sizeof(NativeInit) == 32);
static_assert(offsetof(NativeFd, data) == 8 && offsetof(NativeTx, type) == 72 &&
              offsetof(NativeRx, stamp) == 72 && sizeof(NativeCanRx) == 24);
class NativeBackend final : public CanBackend {
    QLibrary dll;
    quint64 device = 0, channel = 0;
    bool control = false;
    std::optional<quint64> stampOrigin;
    std::vector<DLL_DIRECTORY_COOKIE> searchDirectories;
    template <class T> T fn(const char *n) {
        auto p = dll.resolve(n);
        if (!p)
            fail(QString("缺少驱动导出 %1: %2").arg(n, dll.fileName()));
        return reinterpret_cast<T>(p);
    }
    using Close = quint32(MT_CALL *)(quint64);
    using Open = quint64(MT_CALL *)(quint32, quint32, quint32);
    using Init = quint64(MT_CALL *)(quint64, quint32, NativeInit *);
    using Set = quint32(MT_CALL *)(quint64, quint32, quint32);
    using Value = quint32(MT_CALL *)(quint64, const char *, const char *);
    using Tx = quint32(MT_CALL *)(quint64, NativeTx *, quint32);
    using Rx = quint32(MT_CALL *)(quint64, NativeRx *, quint32, int);
    using RxCan = quint32(MT_CALL *)(quint64, NativeCanRx *, quint32, int);
    using Count = quint32(MT_CALL *)(quint64, quint32);
    Tx tx = nullptr;
    Rx rx = nullptr;
    RxCan rxCan = nullptr;
    Count countFn = nullptr;
    Close reset = nullptr, closeFn = nullptr;
    std::thread receiver;
    std::atomic<bool> receiving{false};
    std::mutex receiveMutex;
    std::deque<Frame> receivedFrames;
    std::function<void()> receiveNotifier;
    QString receiveError;

  public:
    ~NativeBackend() override { close(); }
    void setReceiveNotifier(std::function<void()> notify) override { receiveNotifier = std::move(notify); }
    void open(const Connection &c) override {
        control = c.backend.contains("创芯");
        QString name = control ? "ControlCANFD.dll" : "zlgcan.dll";
        QStringList dirs = {c.driverDirectory, QCoreApplication::applicationDirPath() + "/library",
                            QCoreApplication::applicationDirPath() + "/compat/library"};
        QString path;
        for (auto &d : dirs)
            if (!d.isEmpty() && QFileInfo::exists(d + "/" + name)) {
                path = d + "/" + name;
                break;
            }
        if (path.isEmpty())
            fail(QString("%1 缺失：请在连接设置中选择含 64 位 %2 的目录").arg(c.backend, name));
        SetDefaultDllDirectories(LOAD_LIBRARY_SEARCH_DEFAULT_DIRS);
        const auto directory = QFileInfo(path).absolutePath();
        for (const auto &dependencyPath : {directory, directory + "/kerneldlls"}) {
            if (QDir(dependencyPath).exists()) {
                auto cookie = AddDllDirectory(reinterpret_cast<LPCWSTR>(dependencyPath.utf16()));
                if (cookie)
                    searchDirectories.push_back(cookie);
            }
        }
        dll.setFileName(path);
        if (!dll.load())
            fail(dll.errorString());
        closeFn = fn<Close>("ZCAN_CloseDevice");
        reset = fn<Close>("ZCAN_ResetCAN");
        tx = fn<Tx>("ZCAN_TransmitFD");
        rx = fn<Rx>("ZCAN_ReceiveFD");
        rxCan = reinterpret_cast<RxCan>(dll.resolve("ZCAN_Receive"));
        countFn = fn<Count>("ZCAN_GetReceiveNum");
        int type = c.backend.contains("200U") ? 41 : 42;
        device = fn<Open>("ZCAN_OpenDevice")(type, c.deviceIndex, 0);
        if (!device)
            fail("设备打开失败，请检查驱动、型号、索引及其他软件占用");
        int ch = c.channel.toInt();
        if (ch < 0 || ch >= (type == 41 ? 2 : 1))
            fail("硬件通道超出范围");
        NativeInit cfg{};
        cfg.type = 1;
        cfg.mask = 0xffffffff;
        if (control) {
            for (int k = 0; k < (type == 41 ? 2 : 1); k++) {
                if (fn<Set>("ZCAN_SetAbitBaud")(device, k, 1000000) != 1 ||
                    fn<Set>("ZCAN_SetDbitBaud")(device, k, 5000000) != 1 ||
                    fn<Set>("ZCAN_SetCANFDStandard")(device, k, 0) != 1)
                    fail("创芯设置 1M/5M ISO FD 失败");
            }
        } else {
            auto setter = fn<Value>("ZCAN_SetValue");
            for (auto pair :
                 {qMakePair("canfd_standard", "0"), qMakePair("canfd_abit_baud_rate", "1000000"),
                  qMakePair("canfd_dbit_baud_rate", "5000000"), qMakePair("initenal_resistance", "1")}) {
                auto key = QString("%1/%2").arg(ch).arg(pair.first).toLatin1();
                setter(device, key.constData(), pair.second);
            }
            cfg.abit = 101166;
            cfg.dbit = 4260362;
        }
        channel = fn<Init>("ZCAN_InitCAN")(device, ch, &cfg);
        if (!channel)
            fail("InitCAN 失败");
        if (control) {
            using Filter = quint32(MT_CALL *)(quint64, quint32);
            fn<Close>("ZCAN_ClearFilter")(channel);
            fn<Filter>("ZCAN_SetFilterMode")(channel, 0);
            fn<Filter>("ZCAN_SetFilterStartID")(channel, 0);
            fn<Filter>("ZCAN_SetFilterEndID")(channel, 0x7ff);
            fn<Close>("ZCAN_AckFilter")(channel);
        }
        auto rc = fn<Close>("ZCAN_StartCAN")(channel);
        if (control && rc != 1)
            fail("StartCAN 失败");
        stampOrigin.reset();
        receiving = true;
        // Same concurrency contract as the working Python client: TX must not
        // wait for an RX driver call, telemetry decoding or waveform processing.
        receiver = std::thread([this] {
            // No process-wide timer resolution change; fall back on older Windows.
            HANDLE idleTimer = CreateWaitableTimerExW(nullptr, nullptr, 0x2, TIMER_ALL_ACCESS);
            auto timerCleanup = qScopeGuard([&] {
                if (idleTimer)
                    CloseHandle(idleTimer);
            });
            try {
                while (receiving) {
                    auto batch = readDriver();
                    if (batch.empty()) {
                        LARGE_INTEGER due;
                        due.QuadPart = -5000; // 0.5 ms, relative 100 ns units
                        if (idleTimer && SetWaitableTimer(idleTimer, &due, 0, nullptr, nullptr, FALSE))
                            WaitForSingleObject(idleTimer, INFINITE);
                        else
                            std::this_thread::sleep_for(std::chrono::milliseconds(1));
                        continue;
                    }
                    {
                        std::lock_guard lock(receiveMutex);
                        if (receivedFrames.size() + batch.size() > 65536)
                            throw std::runtime_error("CAN 接收处理积压超过 65536 帧，已停止连接");
                        receivedFrames.insert(receivedFrames.end(), std::make_move_iterator(batch.begin()),
                                              std::make_move_iterator(batch.end()));
                    }
                    if (receiveNotifier)
                        receiveNotifier();
                }
            } catch (const std::exception &e) {
                std::lock_guard lock(receiveMutex);
                receiveError = QString::fromUtf8(e.what());
                receiving = false;
            }
        });
    }
    void close() override {
        receiving = false;
        if (receiver.joinable())
            receiver.join();
        if (channel && reset)
            reset(channel);
        channel = 0;
        if (device && closeFn)
            closeFn(device);
        device = 0;
        dll.unload();
        for (auto cookie : searchDirectories)
            RemoveDllDirectory(cookie);
        searchDirectories.clear();
        std::lock_guard lock(receiveMutex);
        receivedFrames.clear();
        receiveError.clear();
    }
    void send(const Frame &f) override {
        NativeTx t{};
        if (f.bytes.size() > 64 || f.node > 0x7ff)
            fail("CAN FD 帧长度或标准 ID 非法");
        t.frame.id = f.node;
        t.frame.len = quint8(f.bytes.size());
        t.frame.flags = 1;
        memcpy(t.frame.data, f.bytes.constData(), size_t(f.bytes.size()));
        auto submitted = tx(channel, &t, 1);
        if (submitted != 1)
            fail(QString("ZCAN_TransmitFD 返回 %1（请求 1 帧，未自动重发）").arg(submitted));
    }
    std::vector<Frame> receive() override {
        std::lock_guard lock(receiveMutex);
        if (!receiveError.isEmpty())
            fail(receiveError);
        std::vector<Frame> out;
        const auto count = std::min<size_t>(256, receivedFrames.size());
        out.reserve(count);
        for (size_t i = 0; i < count; ++i) {
            out.push_back(std::move(receivedFrames.front()));
            receivedFrames.pop_front();
        }
        return out;
    }

  private:
    std::vector<Frame> readDriver() {
        NativeRx b[256]{};
        // Follow the vendor/legacy API's available-count contract for both queues.
        const auto readStart = monotonicNs();
        auto available = countFn(channel, 1);
        if (available == 0xffffffffu)
            fail("CAN FD 接收队列查询失败");
        auto n = available ? rx(channel, b, std::min<quint32>(256, available), 0) : 0;
        const auto readEnd = monotonicNs();
        if (n > 256)
            fail("CAN FD 接收失败");
        std::vector<Frame> out;
        out.reserve(n);
        for (unsigned i = 0; i < n; i++)
            if (b[i].frame.len <= 64 && !(b[i].frame.id & 0xe0000000)) {
                const bool validStamp = b[i].stamp <= quint64(std::numeric_limits<qint64>::max());
                if (validStamp && !stampOrigin)
                    stampOrigin = b[i].stamp;
                double seconds = validStamp && stampOrigin && b[i].stamp >= *stampOrigin
                                     ? double(b[i].stamp - *stampOrigin) / 1e6
                                     : 0;
                out.push_back({b[i].frame.id & 0x7ff,
                               QByteArray(reinterpret_cast<char *>(b[i].frame.data), b[i].frame.len), seconds,
                               b[i].stamp, readEnd, readStart, validStamp});
            }
        auto classicAvailable = rxCan ? countFn(channel, 0) : 0;
        if (classicAvailable == 0xffffffffu)
            fail("经典 CAN 接收队列查询失败");
        if (rxCan && classicAvailable > 0) {
            NativeCanRx classic[128]{};
            const auto classicStart = monotonicNs();
            auto received = rxCan(channel, classic, std::min<quint32>(128, classicAvailable), 0);
            const auto classicEnd = monotonicNs();
            if (received > 128)
                fail("经典 CAN 接收失败");
            for (unsigned k = 0; k < received; k++) {
                auto &f = classic[k];
                if (f.frame.len > 8 || (f.frame.id & 0xe0000000))
                    continue;
                const bool validStamp = f.stamp <= quint64(std::numeric_limits<qint64>::max());
                if (validStamp && !stampOrigin)
                    stampOrigin = f.stamp;
                double seconds = validStamp && stampOrigin && f.stamp >= *stampOrigin
                                     ? double(f.stamp - *stampOrigin) / 1e6
                                     : 0;
                out.push_back({f.frame.id & 0x7ff,
                               QByteArray(reinterpret_cast<char *>(f.frame.data), f.frame.len), seconds,
                               f.stamp, classicEnd, classicStart, validStamp});
            }
        }
        return out;
    }
};
#endif
class BridgeBackend final : public CanBackend {
    QProcess process;
    QByteArray pending;
    std::vector<Frame> frames;
    bool opened = false;
    void write(QJsonObject o) {
        auto data = QJsonDocument(o).toJson(QJsonDocument::Compact) + '\n';
        if (process.write(data) != data.size())
            fail("兼容进程写入失败");
        process.waitForBytesWritten(10);
    }
    void drain() {
        pending += process.readAllStandardOutput();
        if (pending.size() > 4 * 1024 * 1024)
            fail("兼容进程输出超限");
        while (pending.contains('\n')) {
            int i = int(pending.indexOf('\n'));
            auto line = pending.left(i);
            pending.remove(0, i + 1);
            auto o = QJsonDocument::fromJson(line).object();
            if (o.contains("error"))
                fail(o["error"].toString());
            if (o["opened"].toBool())
                opened = true;
            for (auto v : o["frames"].toArray()) {
                auto a = v.toArray();
                frames.push_back({quint32(a[0].toInt()), QByteArray::fromHex(a[1].toString().toLatin1()),
                                  a[2].toDouble()});
            }
        }
    }

  public:
    ~BridgeBackend() override {
        try {
            close();
        } catch (...) {
            process.kill();
            process.waitForFinished(1000);
        }
    }
    void open(const Connection &c) override {
        QString script = QCoreApplication::applicationDirPath() + "/compat/bridge.py";
        process.start(c.python, {"-u", script});
        if (!process.waitForStarted(3000))
            fail(process.errorString());
        write({{"op", "open"},
               {"profile", c.backend},
               {"index", c.deviceIndex},
               {"channel", c.channel},
               {"can_channel", c.channel.toInt()}});
        QElapsedTimer t;
        t.start();
        while (!opened && t.elapsed() < 8000) {
            process.waitForReadyRead(50);
            drain();
            if (process.state() == QProcess::NotRunning)
                fail(QString::fromUtf8(process.readAllStandardError()));
        }
        if (!opened)
            fail("兼容后端连接超时");
    }
    void close() override {
        if (process.state() != QProcess::NotRunning) {
            write({{"op", "close"}});
            if (!process.waitForFinished(500)) {
                process.kill();
                process.waitForFinished(1000);
            }
        }
    }
    void send(const Frame &f) override {
        if (process.bytesToWrite() > 65536)
            fail("兼容后端发送队列拥塞");
        write({{"op", "send"}, {"node", int(f.node)}, {"data", QString::fromLatin1(f.bytes.toHex())}});
    }
    std::vector<Frame> receive() override {
        if (process.state() == QProcess::NotRunning)
            fail(QString("兼容进程退出: %1").arg(QString::fromUtf8(process.readAllStandardError())));
        drain();
        std::vector<Frame> result;
        result.swap(frames);
        return result;
    }
};
class AutoBackend final : public CanBackend {
    std::unique_ptr<CanBackend> active;
    QString selected;
    std::function<void()> notify_;

  public:
    void open(const Connection &config) override {
        QStringList errors;
        for (const QString &name :
             QStringList{"ZQWL-UCANFD-100E", "周立功 USBCANFD-200U", "创芯科技 USBCAN-FD (100E 单通道)",
                         "创芯科技 USBCAN-FD (200U)"}) {
            auto candidate = config;
            candidate.backend = name;
            try {
                auto backend = createBackend(candidate);
                backend->setReceiveNotifier(notify_);
                backend->open(candidate);
                active = std::move(backend);
                selected = name;
                return;
            } catch (const std::exception &e) {
                errors.append(name + ": " + QString::fromUtf8(e.what()));
            }
        }
        fail("自动探测未找到可打开的 CAN FD 设备：\n" + errors.join('\n'));
    }
    void close() override {
        if (active)
            active->close();
    }
    void send(const Frame &f) override { active->send(f); }
    std::vector<Frame> receive() override { return active->receive(); }
    void setReceiveNotifier(std::function<void()> notify) override {
        notify_ = std::move(notify);
        if (active)
            active->setReceiveNotifier(notify_);
    }
    QString description() const override { return "自动 → " + selected; }
};
QStringList backendNames() {
    return {"自动（原生 CAN FD）",
            "Demo",
            "ZQWL-UCANFD-100E",
            "周立功 USBCANFD-200U",
            "创芯科技 USBCAN-FD (100E 单通道)",
            "创芯科技 USBCAN-FD (200U)",
            "SLCAN 适配器 (CANable 等)",
            "PCAN-USB (经典 CAN)",
            "SocketCAN (Linux CAN)",
            "SocketCAN (Linux CAN-FD)",
            "Virtual CAN-FD (软件测试)",
            "PCAN-USB"};
}
std::unique_ptr<CanBackend> createBackend(const Connection &c) {
    if (c.backend.startsWith("自动"))
        return std::make_unique<AutoBackend>();
    if (c.backend == "Demo")
        return std::make_unique<DemoBackend>();
#ifdef Q_OS_WIN
    if (c.backend.contains("ZQWL") || c.backend.contains("周立功") || c.backend.contains("创芯"))
        return std::make_unique<NativeBackend>();
#endif
    return std::make_unique<BridgeBackend>();
}
} // namespace mt
