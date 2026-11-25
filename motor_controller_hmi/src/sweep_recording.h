#pragma once
#include "sweep.h"
#include <QByteArray>
#include <QtGlobal>
#include <QIODevice>
namespace mt {
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
  QString summary(quint32 expected, bool supported) const;
};
// Whole-period block H1 on uploaded x[n], y[n], sharing the device's windows.
// No interpolation of lost samples and no phase shift to hide the control delay.
std::vector<SweepPoint> recordingH1(const SweepRecording &, int method, double fs,
                                  const std::vector<SweepPoint> &devicePoints,
                                  quint32 expected);
struct SweepCapture {
  SweepRecording recording;
  int method=0;
  double fs=0;
  quint32 expected=0;
  bool completed=false;
  QString diagnostic;
  std::vector<SweepPoint> points;
};
bool readSweepCapture(QIODevice &, const QByteArray &metadata, SweepCapture &, QString &error);
bool writeSweepCapture(QIODevice &, const SweepCapture &);
} // namespace mt
