#pragma once
#include "session.h"
#include <QtWidgets>

namespace mt {
// UI drafts and confirmed device values deliberately have separate lifetimes.
class MotionControl : public QGroupBox {
    Q_OBJECT
  public:
    explicit MotionControl(Session &, QWidget *parent = nullptr);
    void setContext(quint32 node, Version, int selectedMode, bool workspaceSafe, bool busy);

  private:
    Session &session_;
    quint32 node_ = 0;
    Version version_ = Version::V2;
    int mode_ = -1;
    bool connected_ = false, safe_ = false, busy_ = false, dirty_ = false;
    int revision_ = 0, submittedRevision_ = 0;
    quint64 readToken_ = 0, writeToken_ = 0, targetToken_ = 0;
    bool initialReadPending_ = false;
    QString limitsReadAt_;
    QJsonObject limits_;
    std::array<QDoubleSpinBox *, 3> targets_{};
    std::array<QPushButton *, 3> send_{};
    QSpinBox *soft_;
    QCheckBox *hard_;
    QPushButton *read_, *apply_, *zero_, *combined_;
    QLabel *motionState_, *motionResult_, *softState_, *hardState_, *activeState_, *feedback_, *limitResult_;
    QString targetDescription_;
    void reset();
    void updateState();
    void readLimits();
    void writeLimits();
    void sendTarget(int channel, bool zero = false);
    void acceptLimits(const QJsonObject &, bool written);
    void reply(quint64, quint32, int, const QJsonObject &);
    void failed(quint64, const QString &);
};
} // namespace mt
