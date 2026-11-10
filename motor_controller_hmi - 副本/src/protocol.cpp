#include "protocol.h"
#include <QtEndian>
#include <cmath>
#include <limits>
#include <stdexcept>
namespace mt {
QStringList debugChannelNames(int loop) {
    QString unit = loop == 0 ? "A" : loop == 1 ? "rad/s" : "rad";
    QString output = loop == 0 ? "V" : loop == 1 ? "A" : "rad/s";
    return {"环内反馈 · " + unit,
            "误差 · " + unit,
            "Iq · A",
            "Id · A",
            "PID dt · s",
            "积分状态 · " + output,
            "PID输出 · " + output,
            "母线 · V",
            "轴速度 · rad/s",
            "Kp",
            "Ki配置",
            "设备给定 · " + unit,
            "输出限幅 · " + output,
            "积分限幅 · " + output,
            "输出斜坡",
            "P项 · " + output,
            "有效Ki",
            "环执行标志",
            "设备Tick · us",
            "状态Flags"};
}
std::optional<Sample> decodeDebug(const Frame &f) {
    if (f.bytes.size() != 64 || quint8(f.bytes[1]) < 0xb9 || quint8(f.bytes[1]) > 0xbb || f.bytes[2] != 1)
        return {};
    Sample s;
    s.node = f.node;
    s.debugLoop = quint8(f.bytes[1]) - 0xb9 + 1;
    s.values.fill(std::numeric_limits<double>::quiet_NaN());
    constexpr int mapping[] = {11, 0, 2, 3, 4, 5, 6, 7, 8, 9, 10, 12, 13, 14};
    for (int k = 0; k < 14; ++k) {
        quint32 bits = qFromBigEndian<quint32>(f.bytes.constData() + 8 + k * 4);
        float value;
        memcpy(&value, &bits, 4);
        if (!std::isfinite(value))
            return {};
        s.values[mapping[k]] = value;
    }
    s.values[1] = s.values[11] - s.values[0];
    // An upper-layer command is not the limited/cascaded PID input. Do not
    // present command tracking error times Kp as the controller's P term.
    s.values[15] = (quint8(f.bytes[3]) & 8) ? std::numeric_limits<double>::quiet_NaN()
                                          : s.values[9] * s.values[1];
    s.values[16] = s.values[10] * (s.debugLoop == 1 ? 10000 : 1);
    s.values[19] = quint8(f.bytes[3]);
    s.enabled = (quint8(f.bytes[3]) & 1) != 0;
    s.error = (quint8(f.bytes[3]) & 4) ? 1 : 0;
    s.values[17] = s.enabled;
    s.values[18] = qFromBigEndian<quint32>(f.bytes.constData() + 4);
    return s;
}
std::optional<double> DebugClock::advance(quint32 tick, bool &restarted, double receivedAt) {
    restarted = false;
    if (!previous_) {
        previous_ = tick;
        receivedAt_ = receivedAt;
        return 0.0;
    }
    quint32 delta = tick - *previous_;
    if (!delta)
        return {};
    // The current AT32 firmware converts the 180 MHz CYCCNT directly to us.
    // Its period is 2^32 / 180 us, NOT 2^32 us. Match both endpoints;
    // keep support for firmware using a genuine uint32 microsecond clock.
    constexpr double dwtPeriod = 4294967296.0 / 180.0;
    const double hostGap = receivedAt >= 0 && receivedAt_ >= 0 ? receivedAt - receivedAt_ : -1;
    const bool nearDwtWrap =
        *previous_ > dwtPeriod - 1000000 && *previous_ <= dwtPeriod + 2 && tick < 1000000;
    const double wrappedDelta = dwtPeriod - *previous_ + tick;
    if (nearDwtWrap && (hostGap < 0 || (hostGap < 2 && std::abs(wrappedDelta / 1e6 - hostGap) < .25))) {
        elapsed_ += wrappedDelta;
        previous_ = tick;
        receivedAt_ = receivedAt;
        return elapsed_ / 1e6;
    }
    // A pre-wrap frame arriving after the wrap must not jump the plot forward.
    if (*previous_ < 1000000 && tick > dwtPeriod - 1000000 && tick <= dwtPeriod + 2 && hostGap >= 0 &&
        hostGap < 1)
        return {};
    if (delta > 0x7fffffffU) {
        // Ignore small reordering without changing the accepted clock anchor.
        if (*previous_ - tick <= 100000)
            return {};
        // Unknown reset: retain history on a monotonic display timeline. The
        // epoch boundary prevents analysis treating this host-spliced gap as
        // device timing; raw ticks remain available in CSV.
        elapsed_ += hostGap > 0 ? hostGap * 1e6 : 1;
        restarted = true;
    } else
        elapsed_ += delta;
    previous_ = tick;
    receivedAt_ = receivedAt;
    return elapsed_ / 1e6;
}
quint16 u16(const QByteArray &b, int o) {
    if (o < 0 || o + 2 > b.size())
        throw std::invalid_argument("short u16");
    return qFromBigEndian<quint16>(b.constData() + o);
}
qint16 i16(const QByteArray &b, int o) {
    return static_cast<qint16>(u16(b, o));
}
QByteArray be16(int v) {
    QByteArray b(2, 0);
    qToBigEndian<quint16>(quint16(v), b.data());
    return b;
}
QByteArray pack(quint8 cnt, quint8 cmd, const QByteArray &p, bool iap) {
    if (p.size() > 62)
        throw std::invalid_argument("CAN FD payload exceeds 62 bytes");
    QByteArray b;
    b.append(char(cnt)).append(char(cmd)).append(p);
    constexpr int dlc[] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 12, 16, 20, 24, 32, 48, 64};
    for (int n : dlc)
        if (b.size() <= n) {
            b.append(n - b.size(), iap ? char(0xff) : char(0));
            break;
        }
    return b;
}
std::optional<Sample> decode(const Frame &f, Version v) {
    if (f.bytes.size() < 10 || quint8(f.bytes[1]) != 0)
        return {};
    auto p = f.bytes.mid(2);
    Sample s;
    s.time = f.time;
    s.node = f.node;
    s.extended = p.size() >= 16;
    s.environment = p.size() >= 18;
    s.values[6] = i16(p, 0) / 128.0 * (v == Version::V1 ? std::numbers::pi / 180 : 1);
    s.values[5] = i16(p, 2);
    s.values[0] = i16(p, 4) / 1000.0;
    for (int k = 1; k < 5; k++)
        s.values[k] = s.extended ? i16(p, 4 + k * 2) / 1000.0 : std::numeric_limits<double>::quiet_NaN();
    auto status = quint8(p[s.extended ? 14 : 6]);
    s.mode = status & 15;
    s.enabled = v == Version::V1 ? (status >> 4) == 1 : (status >> 4) != 0;
    s.error = quint8(p[s.extended ? 15 : 7]);
    s.values[7] = s.environment ? quint8(p[17]) / 10.0 : std::numeric_limits<double>::quiet_NaN();
    s.values[8] = s.environment ? quint8(p[16]) : std::numeric_limits<double>::quiet_NaN();
    return s;
}
QByteArray target(quint8 node, double value, int ch, Version v) {
    if (!node || !std::isfinite(value))
        throw std::invalid_argument("invalid target/node");
    double scaled = value * (ch == 0   ? 128 * (v == Version::V1 ? 180 / std::numbers::pi : 1)
                             : ch == 2 ? 1000
                                       : 1);
    if (scaled < -32768 || scaled > 32767)
        throw std::out_of_range("target outside int16 wire range");
    return QByteArray(1, char(node)) + be16(int(std::round(scaled)));
}
std::vector<Sample> decodeTargets(int cmd, const QByteArray &payload, Version version) {
    const std::array<bool, 3> present = {cmd == 1 || cmd == 0x12 || cmd == 0x13 || cmd == 0x30,
                                         cmd == 2 || cmd == 0x12 || cmd == 0x23 || cmd == 0x30,
                                         cmd == 3 || cmd == 0x13 || cmd == 0x23 || cmd == 0x30};
    int stride = 1;
    for (bool included : present)
        if (included)
            stride += 2;
    if (stride == 1 || payload.isEmpty() || payload.size() % stride)
        return {};
    std::vector<Sample> result;
    for (int offset = 0; offset < payload.size(); offset += stride) {
        Sample s;
        s.node = quint8(payload[offset]);
        if (!s.node)
            return {};
        int field = offset + 1;
        for (int c = 0; c < 3; ++c)
            if (present[c]) {
                double scale = c == 0   ? (version == Version::V1 ? std::numbers::pi / 180 : 1) / 128.0
                               : c == 2 ? .001
                                        : 1;
                s.values[9 + c] = i16(payload, field) * scale;
                field += 2;
            }
        result.push_back(s);
    }
    return result;
}
QByteArray pid(double p, double i, double d) {
    QByteArray b;
    for (double k : {p, i, d}) {
        if (!std::isfinite(k) || k < 0 || k > 65535.0 / 4096)
            throw std::out_of_range("PID Q12 range");
        b += be16(int(std::round(k * 4096)));
    }
    return b;
}
QByteArray combinedTarget(quint8 node, int command, double position, double velocity, double current,
                          Version v) {
    QByteArray b(1, char(node));
    if (command == 0x12 || command == 0x13 || command == 0x30)
        b += target(node, position, 0, v).mid(1);
    if (command == 0x12 || command == 0x23 || command == 0x30)
        b += target(node, velocity, 1, v).mid(1);
    if (command == 0x13 || command == 0x23 || command == 0x30)
        b += target(node, current, 2, v).mid(1);
    if (b.size() == 1)
        throw std::invalid_argument("invalid combined target command");
    return b;
}
QString telemetryErrorName(int code, Version version) {
    static const QStringList fault = {
        "正常",           "硬件过流",     "软件过流",    "硬件过压", "软件过压",   "硬件欠压",
        "软件欠压",       "缺相",         "编码器异常",  "过温",     "通信异常",   "位置给定异常",
        "速度给定异常",   "电流给定异常", "回零超时",    "丢帧",     "轴超速",     "对齐失败",
        "标定失败",       "控制饱和",     "看门狗",      "参数错误", "短路",       "堵转",
        "三相电流不平衡", "电流传感器",   "存储故障",    "制动异常", "去饱和保护", "驱动 nFAULT",
        "逻辑欠压",       "W 相开路",     "用户故障指令"};
    static const QStringList iap = {
        "正常", "无效命令", "无效参数", "Flash 擦除", "Flash 写入", "Flash 校验", "CRC",        "超时",
        "忙",   "不支持",   "通信错误", "响应超时",   "拒绝升级",   "内存不足",   "Flash 切换", "丢帧"};
    int sub = code & 127;
    if (code & 128)
        return QString("IAP · ") + (sub < iap.size() ? iap[sub] : QString::number(sub));
    if (sub < fault.size() && (version == Version::V2 || sub < 16))
        return fault[sub];
    return QString("未知保护码 0x%1").arg(sub, 2, 16, QChar('0'));
}
QString errorName(int c) {
    static const QStringList names = {"正常",         "空指针",     "参数非法", "状态不允许", "超时",
                                      "未实现",       "通信失败",   "存储失败", "传感器失败", "执行失败",
                                      "对齐离散过大", "转子未吸合", "对齐失败"};
    if (c >= 0 && c < names.size())
        return names[c];
    switch (c) {
    case 20:
        return "锁存故障";
    case 21:
        return "偏置校准失败";
    case 22:
        return "模式无效";
    case 23:
        return "标定中";
    case 24:
        return "标定类型无效";
    case 25:
        return "无锁存故障";
    default:
        return QString("错误 0x%1").arg(c, 2, 16, QChar('0'));
    }
}
QStringList modes(Version v) {
    if (v == Version::V1)
        return {"PP", "CSP", "CST", "HM", "CSV", "MIT"};
    return {"IDLE", "TORQUE", "SPEED",       "POSITION",   "CSP",           "CSV",    "CST",
            "MIT",  "HOMING", "TORQUE OPEN", "SPEED OPEN", "POSITION OPEN", "VOLTAGE"};
}
bool modeAcceptsTarget(Version v, int mode, int channel) {
    if (v == Version::V1) {
        if (channel == 0)
            return mode == 0 || mode == 1 || mode == 5;
        if (channel == 1)
            return mode == 2 || mode == 4 || mode == 5;
        if (channel == 2)
            return mode == 2 || mode == 5;
    } else {
        if (channel == 0)
            return mode == 3 || mode == 4 || mode == 7 || mode == 11;
        if (channel == 1)
            return mode == 2 || mode == 5 || mode == 7 || mode == 10;
        if (channel == 2)
            return mode == 1 || mode == 6 || mode == 7 || mode == 9;
    }
    return false;
}
bool requiresDisabled(int c) {
    return (c >= 0x41 && c <= 0x46) || c == 0x4c || c == 0x4d || c == 0x4e || c == 0x50 || c == 0x54 ||
           c == 0xab || c == 0xac || c == 0xb1;
}
int responseCommand(int c) {
    return c >= 0x42 && c <= 0x46 ? c + 5 : c;
}
bool hasResult(int c) {
    return c == 0x39 || c == 0x40 || requiresDisabled(c) || c == 0x4f || c == 0xb8 || c == 0xad ||
           c == 0xaf || c == 0xb0;
}
QJsonObject response(quint8 c, const QByteArray &p, Version) {
    QJsonObject o;
    o["command"] = c;
    if (c >= 0x42 && c <= 0x46 && p.size() >= (c == 0x45 ? 4 : 6))
        c = quint8(responseCommand(c));
    o["hex"] = QString::fromLatin1(p.toHex(' '));
    if (c >= 0x47 && c <= 0x49 && p.size() >= 6) {
        o["P"] = u16(p, 0) / 4096.0;
        o["I"] = u16(p, 2) / 4096.0;
        o["D"] = u16(p, 4) / 4096.0;
    } else if (c == 0x4a && p.size() >= 4) {
        o["stiffness"] = u16(p, 0) / 4096.0;
        o["damping"] = u16(p, 2) / 4096.0;
    } else if (c == 0x4b && p.size() >= 6) {
        o["temperature"] = u16(p, 0);
        o["current_mA"] = u16(p, 2);
        o["under_voltage"] = quint8(p[4]);
        o["over_voltage"] = quint8(p[5]);
    } else if (c == 0x53 && p.size() >= 16) {
        o["pole_pairs"] = quint8(p[0]);
        o["direction"] = qint8(p[1]);
        o["gear_ratio"] = u16(p, 2) / 1000.0;
        o["electrical_offset"] = qFromBigEndian<qint32>(p.constData() + 4) / 1e6;
        o["mechanical_offset"] = qFromBigEndian<qint32>(p.constData() + 8) / 1e6;
        o["encoder_cpr"] = u16(p, 12);
        o["aligned"] = bool(quint8(p[14]) & 1);
    } else if (c == 0x52 && !p.isEmpty())
        o["motor_type"] = quint8(p[0]);
    else if (c == 0x4c && p.size() >= 2)
        o["node"] = u16(p, 0);
    else if (c == 0xae && !p.isEmpty())
        o["calibration_state"] = quint8(p[0]);
    else if (c == 0xfc && p.size() >= 2) {
        o["state"] = quint8(p[0]);
        o["error"] = quint8(p[1]);
    } else if (c == 0xfd && p.size() >= 7) {
        if (p[0] == 'V' && p[2] == '.' && quint8(p[1]) < 48) {
            QString s = QString("V%1.%2.%3_").arg(quint8(p[1])).arg(quint8(p[3])).arg(quint8(p[5]));
            for (int k = 7; k < qMin(13, p.size()); k++)
                s += QString("%1").arg(quint8(p[k]), 2, 10, QChar('0'));
            o["version"] = s;
        } else
            o["version"] = QString::fromLatin1(p.constData());
    } else if (c == 0x7e && p.size() >= 2) {
        o["dispatched_counter"] = quint8(p[0]);
        o["dispatched_command"] = quint8(p[1]);
    } else if ((c == 0x4d || c == 0x4e || c == 0x54 || c == 0xac) && !p.isEmpty()) {
        o["ok"] = p[0] == 1;
        o["message"] = p[0] == 1 ? "操作成功" : "设备返回失败";
    } else if (c == 0xb8 && p.size() >= 5 && p[0] == 1) {
        o["ok"] = p[1] == 0;
        o["error"] = quint8(p[1]);
        o["mask"] = quint8(p[2]);
        o["hz"] = u16(p, 3);
        o["message"] = errorName(quint8(p[1]));
    } else if (c == 0xb5 || c == 0xb6) {
        // Formats 2/3 share the complete 20-byte payload; CAN FD padding is
        // allowed.
        if (p.size() < 20 || (p[0] != 2 && p[0] != 3) || !u16(p, 2) || u16(p, 4) > u16(p, 2)) {
            o["valid"] = false;
            o["ok"] = false;
            o["message"] = "限速应答版本或数据不匹配";
            return o;
        }
        o["format"] = quint8(p[0]);
        o["ok"] = p[1] == 0;
        o["message"] = errorName(quint8(p[1]));
        o["hard_rpm"] = u16(p, 2);
        o["soft_rpm"] = u16(p, 4);
        o["mode"] = quint8(p[6]);
        o["flags"] = quint8(p[7]);
        // Format 2 predates the hard-protection switch: it is always enabled.
        o["hard_enabled"] = p[0] == 2 || bool(quint8(p[7]) & 2);
        o["limiting"] = bool(quint8(p[7]) & 1);
        if (p.size() >= 16) {
            o["speed_rpm"] = qFromBigEndian<qint32>(p.constData() + 8) / 1000.0;
            o["iq_reference"] = qFromBigEndian<qint32>(p.constData() + 12) / 1000.0;
        }
        if (p.size() >= 20)
            o["iq_limited"] = qFromBigEndian<qint32>(p.constData() + 16) / 1000.0;
    } else if (c == 0xb1 && p.size() >= 12 && p[0] == 1) {
        o["ok"] = p[1] == 0;
        o["error"] = quint8(p[1]);
        o["message"] = errorName(quint8(p[1]));
        o["session"] = u16(p, 2);
        o["count"] = u16(p, 4);
        o["max_error_rad"] = qFromBigEndian<qint32>(p.constData() + 8) / 1e6;
    } else if (c == 0xb2 && p.size() >= 32 && p[0] == 1) {
        o["phase"] = quint8(p[1]);
        o["settled"] = bool(quint8(p[2]) & 1);
        o["session"] = u16(p, 4);
        o["sequence"] = u16(p, 6);
        o["time_s"] = qFromBigEndian<quint32>(p.constData() + 8) / 1000.0;
        QStringList keys = {"target", "measured", "error_rad", "mechanical", "expected_mechanical"};
        for (int k = 0; k < 5; k++)
            o[keys[k]] = qFromBigEndian<qint32>(p.constData() + 12 + 4 * k) / 1e6;
    } else if (hasResult(c) && c != 0xb1 && c != 0xb6 && c != 0xb8 && !p.isEmpty()) {
        o["error"] = quint8(p[0]);
        o["ok"] = p[0] == 0;
        o["message"] = errorName(quint8(p[0]));
    }
    if (c == 0xab && p.size() >= 14) {
        o["method"] = quint8(p[1]);
        o["direction"] = qint8(p[2]);
        o["flags"] = quint8(p[3]);
        o["electrical_angle"] = qFromBigEndian<qint32>(p.constData() + 4) / 1e6;
        o["offset"] = qFromBigEndian<qint32>(p.constData() + 8) / 1e6;
        o["probe_iq"] = i16(p, 12) / 1000.0;
    }
    if (c >= 0xb9 && c <= 0xbb && p.size() == 62 && p[0] == 1) {
        auto decoded = decodeDebug({1, QByteArray(1, 0) + QByteArray(1, char(c)) + p, 0});
        if (!decoded)
            return {{"valid", false}};
        o["active"] = decoded->enabled;
        o["flags"] = quint8(p[1]);
        o["tick_us"] = double(qFromBigEndian<quint32>(p.constData() + 2));
        QStringList keys = {"reference", "feedback",    "iq", "id",        "dt",    "integral", "output",
                            "vbus",      "shaft_speed", "kp", "ki_config", "limit", "i_limit",  "ramp"};
        for (int k = 0; k < 14; k++) {
            quint32 bits = qFromBigEndian<quint32>(p.constData() + 6 + k * 4);
            float val;
            memcpy(&val, &bits, 4);
            if (std::isfinite(val))
                o[keys[k]] = val;
        }
    }
    o["valid"] = o.size() > 2;
    return o;
}
quint32 checksum(const QByteArray &b) {
    quint32 sum = 0;
    for (int k = 0; k < b.size(); k += 4) {
        quint32 w = 0;
        for (int j = 0; j < 4 && k + j < b.size(); j++)
            w |= quint32(quint8(b[k + j])) << (8 * j);
        quint32 old = sum;
        sum += w;
        if (sum < old)
            sum++;
    }
    return sum;
}
QByteArray firmwareImage(const QByteArray &b) {
    if (b.size() <= 32 || b.size() > 61 * 1024 || b.left(4) != "ASIX")
        throw std::invalid_argument("需要有效 ASIX .efw，大小 33..62464 字节");
    if ((quint8(b.at(4)) | (quint16(quint8(b.at(5))) << 8)) != 3 ||
        (quint8(b.at(6)) | (quint16(quint8(b.at(7))) << 8)) != 0 ||
        qFromBigEndian<quint32>(b.constData() + 12) != quint32(b.size()))
        throw std::invalid_argument("固件类型/压缩/声明长度不匹配");
    if (qFromLittleEndian<quint32>(b.constData() + 20) != ~checksum(b.mid(32)))
        throw std::invalid_argument("ASIX checksum32 不匹配");
    return b;
}
std::vector<QByteArray> firmwareChunks(const QByteArray &b) {
    firmwareImage(b);
    std::vector<QByteArray> out;
    for (int k = 0; k < b.size(); k += 62) {
        auto part = b.mid(k, 62);
        part.append(62 - part.size(), char(0xff));
        out.push_back(part);
    }
    return out;
}
} // namespace mt
