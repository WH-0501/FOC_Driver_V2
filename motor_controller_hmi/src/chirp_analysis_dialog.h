#pragma once
#include "sweep_recording.h"
#include <QDialog>
namespace mt {
// Purely offline: does not hold a Session or send any motor commands.
QDialog *createChirpAnalysisDialog(QWidget *parent,const SweepCapture &capture={});
}
