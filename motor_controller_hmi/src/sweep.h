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
struct SweepBandwidthResult {
  bool found=false, baselineValid=false;
  double hz=0, lowHz=0, highHz=0, baselineDb=0;
  QString message;
};
SweepBandwidthResult sweepBandwidthResult(const std::vector<SweepPoint> &points);
QString sweepBandwidth(const std::vector<SweepPoint> &points);
// Closed-loop 0 dB downward crossing, >=100 Hz; diagnostic, not open-loop crossover.
// A later adjacent valid bracket can yield a candidate after a gap, but cannot
// confirm that it is the first crossing. No interpolation bridges invalid data.
struct SweepCrossingResult {
  bool found=false, firstConfirmed=false;
  double hz=0, lowHz=0, highHz=0;
  QString message;
};
SweepCrossingResult sweepClosedLoopCrossingResult(const std::vector<SweepPoint> &points);
// Conservative convenience API: returns only a crossing with no earlier gap.
double sweepClosedLoopCrossing(const std::vector<SweepPoint> &points);
QString sweepAbortMessage(const QJsonObject &status);
} // namespace mt
