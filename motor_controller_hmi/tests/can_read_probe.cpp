// Explicit read-only diagnostic, never registered with CTest or installed.
#include "backend.h"
#include <QCoreApplication>
#include <QElapsedTimer>
#include <QFile>
#include <QJsonArray>
#include <QJsonDocument>
#include <QThread>
int main(int argc, char **argv) {
    QCoreApplication app(argc, argv);
    if (argc != 4) return 2; // driver directory, output JSON, channel
    QJsonObject report{{"backend", "C++ native"}, {"query", "one B5 only"}};
    mt::Connection c;
    c.backend = "创芯科技 USBCAN-FD (200U)";
    c.driverDirectory = QString::fromLocal8Bit(argv[1]);
    c.channel = QString::fromLocal8Bit(argv[3]);
    report["channel"] = c.channel;
    auto backend = mt::createBackend(c);
    try {
        backend->open(c);
        QElapsedTimer timer;
        timer.start();
        bool nodeSeen = false;
        int telemetry = 0;
        while (timer.elapsed() < 1500) {
            for (auto &f : backend->receive()) {
                if (f.node == 1 && f.bytes.size() >= 10 && f.bytes[1] == 0) { nodeSeen = true; ++telemetry; }
            }
            QThread::msleep(1);
        }
        if (!nodeSeen) throw std::runtime_error("所选通道无 Node 1 周期遥测，未发送任何查询");
        timer.restart();
        backend->send({1, mt::pack(0, 0xb5), 0});
        report["driver_submit_ms"] = timer.nsecsElapsed() / 1e6;
        QJsonArray responses;
        while (timer.elapsed() < 2500) {
            for (auto &f : backend->receive()) {
                if (f.bytes.size() < 2) continue;
                if (f.bytes[1] == 0) ++telemetry;
                else responses.append(QJsonObject{{"at_ms", timer.nsecsElapsed() / 1e6},
                    {"node", int(f.node)}, {"data", QString::fromLatin1(f.bytes.toHex(' '))}});
            }
            QThread::msleep(1);
        }
        report["telemetry_frames"] = telemetry;
        report["responses"] = responses;
    } catch (const std::exception &e) { report["error"] = QString::fromUtf8(e.what()); }
    backend->close(); // Native close resets only the adapter channel, no motor commands.
    QFile out(QString::fromLocal8Bit(argv[2]));
    if (!out.open(QIODevice::WriteOnly)) return 3;
    out.write(QJsonDocument(report).toJson());
    return report.contains("error") ? 1 : 0;
}
