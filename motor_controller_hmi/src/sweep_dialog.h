#pragma once
#include "session.h"
#include <QWidget>
namespace mt {
void showSweepDialog(Session &, Version, quint32 node, QWidget *parent);
}
