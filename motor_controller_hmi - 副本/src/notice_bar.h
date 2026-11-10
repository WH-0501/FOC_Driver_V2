#pragma once
#include <QtWidgets>
namespace mt {
// Persistent, non-modal errors remain visible on every workspace page.
class NoticeBar : public QFrame {
    Q_OBJECT
  public:
    explicit NoticeBar(QWidget *parent = nullptr);
    void post(const QString &level, const QString &message, bool newOperation = false);

  private:
    QLabel *message_;
    QString level_, fullText_;
};
} // namespace mt
