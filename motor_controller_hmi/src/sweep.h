#pragma once
#include <QString>
#include <QJsonObject>
#include <vector>
namespace mt {
struct SweepSample {
  double time, input, output;
};
struct SweepPoint {
  double hz = 0, input = 0, output = 0, gainDb = 0, phaseDeg = 0, inputFit = 0,
         outputFit = 0;
  int count = 0;
  bool valid = false;
  QString status;
};
SweepPoint measureSweep(double hz, const std::vector<SweepSample> &samples);
// Relative to the mean of the first two valid, flat low-frequency points.
// Never bridge rejected points or extrapolate an unmeasured crossing.
QString sweepBandwidth(const std::vector<SweepPoint> &points);
QString sweepAbortMessage(const QJsonObject &status);
} // namespace mt
