#pragma once
#include "sweep_recording.h"
namespace mt {
struct WelchMatch {
  bool available=false, interpolated=false;
  double gainDb=0, phaseDeg=0, gainDelta=0, phaseDelta=0;
};
struct WelchBin {
  double inputPsd=0, outputPsd=0, crossReal=0, crossImag=0;
  // Input-energy participation ratio, NOT independent averages/degrees of freedom.
  double energySegments=0;
  WelchMatch raw, stepped;
};
struct WelchReferenceComparison {
  QString source;
  SweepPoint reference;
  WelchMatch welch; // Interpolate only adjacent valid native Welch bins onto this reference frequency.
};
struct WelchAnalysis {
  QString error;
  bool localRetest=false;
  unsigned segmentLength=0, segments=0, usedSamples=0, tailSamples=0;
  double sampleHz=0, binHz=0, enbwHz=0;
  std::vector<SweepPoint> points; // All native FFT bins in the measured range, including invalid ones.
  std::vector<WelchBin> bins;
  std::vector<WelchReferenceComparison> comparisons;
  bool ok() const { return error.isEmpty() && !points.empty(); }
};
// Raw, complete periodic Chirp only. Discard the first second; Hann periodic,
// constant detrend, 50% overlap, no zero padding, arithmetic PSD/CSD average.
WelchAnalysis analyzeWelch(const SweepCapture &, unsigned segmentLength=4096,
                          const std::vector<SweepPoint> &raw={},
                          const std::vector<SweepPoint> &stepped={});
QString welchAnalysisSummary(const WelchAnalysis &);
QByteArray welchAnalysisCsv(const WelchAnalysis &);
QByteArray welchComparisonCsv(const WelchAnalysis &);
} // namespace mt
