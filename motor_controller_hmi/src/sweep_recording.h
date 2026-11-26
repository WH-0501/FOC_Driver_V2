#pragma once
#include "sweep.h"
#include <QByteArray>
#include <QtGlobal>
#include <QIODevice>
namespace mt {
// Legacy defaults are used only for version-1 captures/known legacy devices.
struct ChirpTiming {
  unsigned periodMs=1000, settlePeriods=1, measurePeriods=3;
  quint32 periodSamples(double fs) const;
  quint32 totalSamples(double fs) const;
  bool valid(double fs) const;
  QString description() const;
  bool operator==(const ChirpTiming &) const = default;
};
bool readChirpTiming(const QJsonObject &, double fs, ChirpTiming &);
struct SweepRawSample { quint32 index; float input, output; };
// Session worker owns ingestion under its mutex; GUI consumes immutable snapshots.
// Device sample indices, never PC/CAN receive timestamps, define the time axis.
struct SweepRecording {
  quint32 node = 0;
  quint16 run = 0;
  quint64 missing = 0, rejected = 0;
  std::vector<SweepRawSample> samples;
  void reset(quint32 n, quint16 r);
  bool append(quint32 n, const QByteArray &payload);
  bool complete(quint32 expected) const;
  QString summary(quint32 expected, bool supported, bool measurementCompleted=false) const;
};
// Explicit mode-2 plan; unique integer periods, rejecting ranges with too few periods.
std::vector<SweepPoint> steppedFrequencyGrid(double low, double high, unsigned count, double fs);
// Whole-period block H1 on uploaded x[n], y[n], sharing the device's windows.
// No interpolation of lost samples and no phase shift to hide the control delay.
std::vector<SweepPoint> recordingH1(const SweepRecording &, int method, double fs,
                                  const std::vector<SweepPoint> &devicePoints,
                                  quint32 expected, const ChirpTiming &timing = {});
struct SweepCapture {
  SweepRecording recording;
  int method=0;
  double fs=0;
  quint32 expected=0;
  bool completed=false;
  QString diagnostic;
  std::vector<SweepPoint> points;
  ChirpTiming timing;
};
bool readSweepCapture(QIODevice &, const QByteArray &metadata, SweepCapture &, QString &error);
bool writeSweepCapture(QIODevice &, const SweepCapture &);
} // namespace mt
