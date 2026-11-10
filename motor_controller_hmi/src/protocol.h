#pragma once
#include <QByteArray>
#include <QJsonObject>
#include <QStringList>
#include <array>
#include <chrono>
#include <limits>
#include <optional>
#include <vector>
namespace mt {
enum class Version { V1, V2 };
inline qint64 monotonicNs() {
    return std::chrono::duration_cast<std::chrono::nanoseconds>(
               std::chrono::steady_clock::now().time_since_epoch())
        .count();
}
struct Frame {
    quint32 node = 1;
    QByteArray bytes;
    double time = 0;
    std::optional<quint64> adapterRxUs{};
    qint64 hostRxNs = 0, readStartNs = 0;
    bool adapterRxValid = false;
};
inline constexpr int normalChannelCount = 12;
inline constexpr int channelCount = 28;
struct Sample {
    double time = 0;
    quint32 node = 1;
    quint8 debugLoop = 0; // 0 normal, 1..3 current/speed/position
    quint32 streamEpoch = 0;
    double acquisitionTime = std::numeric_limits<double>::quiet_NaN();
    std::array<double, channelCount> values = [] {
        std::array<double, channelCount> v{};
        for (int c = 9; c < channelCount; ++c)
            v[c] = std::numeric_limits<double>::quiet_NaN();
        return v;
    }();
    bool enabled = false;
    quint8 mode = 0, error = 0;
    bool extended = false, environment = false;
};
// Canonical channels: Iq, Id, Ia, Ib, Ic [A], velocity [RPM], position [rad],
// bus [V], temperature [C].
inline const QStringList channelNames = {"Iq · A",    "Id · A",         "Ia · A",         "Ib · A",
                                         "Ic · A",    "速度 · RPM",     "位置 · rad",     "母线 · V",
                                         "温度 · °C", "目标位置 · rad", "目标速度 · RPM", "目标电流 · A"};
// Host-submitted references, never device-confirmed setpoints. Unset channels
// remain NaN.
std::vector<Sample> decodeTargets(int command, const QByteArray &, Version);
QStringList debugChannelNames(int loop);
std::optional<Sample> decodeDebug(const Frame &);
class DebugClock {
  public:
    std::optional<double> advance(quint32 tick, bool &restarted, double receivedAt = -1);

  private:
    std::optional<quint32> previous_;
    double elapsed_ = 0;
    double receivedAt_ = -1;
};
quint16 u16(const QByteArray &, int);
qint16 i16(const QByteArray &, int);
QByteArray be16(int);
QByteArray pack(quint8 counter, quint8 cmd, const QByteArray &payload = {}, bool iap = false);
std::optional<Sample> decode(const Frame &, Version);
QByteArray target(quint8 node, double value, int channel, Version);
QByteArray pid(double p, double i, double d);
QJsonObject response(quint8 cmd, const QByteArray &, Version);
QString errorName(int code);
QStringList modes(Version);
bool modeAcceptsTarget(Version, int mode, int channel);
QString telemetryErrorName(int code, Version);
QByteArray combinedTarget(quint8 node, int command, double position, double velocity, double current,
                          Version);
bool requiresDisabled(int cmd);
bool hasResult(int cmd);
int responseCommand(int cmd);
quint32 checksum(const QByteArray &body);
QByteArray firmwareImage(const QByteArray &file);
std::vector<QByteArray> firmwareChunks(const QByteArray &image);
} // namespace mt
