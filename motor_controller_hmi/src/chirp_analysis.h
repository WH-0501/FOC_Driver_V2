#pragma once
#include "sweep_recording.h"
#include <array>
namespace mt {
enum class ChirpFilter { Off, Both, FeedbackOnly, FeedbackMean16, FeedbackLpf };
struct ChirpAnalysisOptions {
  ChirpFilter filter=ChirpFilter::Off;
  double cutoffHz=3000;
  unsigned neighborHalfWidthHz=0; // optional raw-spectrum diagnostic, never replaces raw H1
};
struct ChirpCycleStats {
  double mean=0, rms=0, peak=0, residualRms=0;
  unsigned over02A=0; // observation count, never deletes or replaces samples
};
struct ChirpFrequencyCheck {
  std::array<double,3> gain{}, phase{};
  double gainSpread=0, phaseSpread=0, filterDb=0;
  bool referenceAvailable=false, referenceInterpolated=false;
  double referenceGain=0, referencePhase=0, gainDelta=0, phaseDelta=0;
  double spikeLineRms=0, residualLineRms=0;
  double bandLowHz=0, bandHighHz=0;
  unsigned bandLines=0;
};
struct ChirpSpike {
  unsigned cycle=0, firstIndex=0, lastIndex=0, peakIndex=0;
  double input=0, peak=0, before=0, after=0;
};
struct ChirpAnalysis {
  QString error;
  std::vector<SweepPoint> raw, filtered;
  std::vector<SweepPoint> neighborhood;
  std::vector<ChirpSpike> spikes;
  std::array<std::vector<SweepPoint>,3> cycles;
  std::vector<ChirpFrequencyCheck> checks;
  std::array<ChirpCycleStats,3> cycleStats;
  double inputRepeatMax=0, inputRepeatRms=0, outputResidualRms=0;
  double filteredOutputRms=0, outputRms=0;
  double spikeEnergyFraction=0;
  // Measurement cycles only: t=1..4 s. Raw capture is never modified.
  std::vector<double> filteredInput, filteredOutput;
  std::vector<double> filteredRecording; // Full capture, for the matching waveform comparison.
  bool ok() const { return error.isEmpty() && !raw.empty(); }
};
// Analysis frequencies only; does not change the excitation or device clock.
std::vector<SweepPoint> chirpFrequencyGrid(double low, double high, unsigned count);
ChirpAnalysis analyzeChirp(const SweepCapture &, const ChirpAnalysisOptions &,
                          const std::vector<SweepPoint> &stepped={});
QString chirpAnalysisSummary(const SweepCapture &, const ChirpAnalysisOptions &, const ChirpAnalysis &);
QByteArray chirpAnalysisCsv(const ChirpAnalysis &);
QByteArray chirpSpikesCsv(const ChirpAnalysis &, double sampleHz);
// Causal 16-sample mean. First 15 samples after start/gaps are NaN; raw data is immutable.
std::vector<double> feedbackMean16(const SweepRecording &);
QString feedbackMean16Description(double sampleHz);
std::vector<double> feedbackLpf(const SweepRecording &, double sampleHz, double cutoffHz);
QString feedbackLpfDescription(double sampleHz, double cutoffHz);
} // namespace mt
