#include "identification.h"
#include <QtEndian>
#include <cmath>
#include <cstring>
#include <stdexcept>
namespace mt {
QString validateIdentification(const IdentificationConfig &c) {
    auto in = [](double x, double lo, double hi) { return std::isfinite(x) && x >= lo && x <= hi; };
    if (!in(c.current,.02,43.69) || !in(c.currentTrip,1.5*c.current,65.535))
        return "电流跳闸值须至少为测试电流的 1.5 倍，并符合电机与驱动器限值";
    if (!in(c.voltage,.05,65.535) || !in(c.injection,.01,c.voltage))
        return "注入电压须为 0.01 V 至电压上限，电压上限须为 0.05–65.535 V";
    if (!in(c.frequency,20,1000)) return "20 kHz 固件的注入频率范围为 20–1000 Hz";
    if (!in(c.speed,2,200) || !in(c.speedTrip,1.25*c.speed,300))
        return "轴速跳闸值须至少为目标轴速的 1.25 倍，且不超过 300 rad/s";
    return {};
}
QByteArray identificationStart(int mode, const IdentificationConfig &c) {
    const auto error=validateIdentification(c);
    if (mode<1 || mode>3 || !error.isEmpty())
        throw std::invalid_argument(error.isEmpty()?"辨识模式无效":error.toStdString());
    return QByteArray(1,1)+QByteArray(1,char(mode))+be16(qRound(c.current*1000))+
           be16(qRound(c.voltage*1000))+be16(qRound(c.injection*1000))+be16(qRound(c.frequency))+
           be16(qRound(c.speed*100))+be16(qRound(c.currentTrip*1000))+be16(qRound(c.speedTrip*100));
}
QString identificationStage(int stage) {
    static const QStringList names={"空闲","Rs · 低电流点","Rs · 高电流点","Ld · d 轴电感",
        "Lq · q 轴电感","磁链 · 加速与稳速","磁链 · 采样","辨识完成","辨识失败","已取消"};
    return names.value(stage,"未知阶段");
}
QString identificationError(int error) {
    static const QStringList names={"无错误","配置无效","电流超过辨识阈值","转速或静止阶段位移超限",
        "超时：检查堵转、负载和电压裕量","有效信号不足或转速不稳定","采样包含非有限值",
        "母线电压或电压裕量不足","外部故障或功率级失能","电流采样周期异常"};
    return names.value(error,QString("未知辨识错误 %1").arg(error));
}
QJsonObject identificationResponse(const QByteArray &p) {
    QJsonObject invalid{{"valid",false},{"ok",false},{"message","参数辨识应答长度、版本或结果无效"}};
    if(p.size()<28 || p[0]!=1) return invalid;
    int stage=quint8(p[2]), error=quint8(p[3]), valid=quint8(p[4]);
    if(stage>9 || error>9 || valid>3 || (valid && stage!=7) ||
       (stage==7 && (!valid || error)) || (stage!=8 && error)) return invalid;
    QJsonObject o{{"valid",true},{"ok",p[1]==0},{"error",quint8(p[1])},
        {"message",errorName(quint8(p[1]))},{"stage",stage},{"ident_error",error},
        {"valid_mask",valid},{"elapsed_ms",double(qFromBigEndian<quint32>(p.constData()+8))}};
    const QStringList keys={"rs","ld","lq","flux"};
    for(int i=0;i<4;++i) {
        quint32 bits=qFromBigEndian<quint32>(p.constData()+12+4*i);
        float v;std::memcpy(&v,&bits,4);
        if(!std::isfinite(v) || v<0 || ((valid&(i==3?2:1)) && v<=0))return invalid;
        o[keys[i]]=v;
    }
    return o;
}
}
