#pragma once
#include "protocol.h"
namespace mt {
inline constexpr int identControl = 0xc0, identStatus = 0xc1, identApply = 0xc2;
struct IdentificationConfig {
    double current = .3, voltage = 3, injection = .2, frequency = 200;
    double speed = 20, currentTrip = 1, speedTrip = 30;
};
QString validateIdentification(const IdentificationConfig &);
QByteArray identificationStart(int mode, const IdentificationConfig &);
QJsonObject identificationResponse(const QByteArray &);
QString identificationStage(int stage);
QString identificationError(int error);
inline bool identificationActive(int stage) { return stage >= 1 && stage <= 6; }
}
