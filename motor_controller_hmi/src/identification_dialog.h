#pragma once
#include "identification.h"
#include "session.h"
#include <QDialog>
#include <QElapsedTimer>
#include <QTimer>
class QComboBox; class QDoubleSpinBox; class QCheckBox; class QLabel;
class QProgressBar; class QTableWidget; class QPushButton; class QGroupBox;
namespace mt {
class IdentificationDialog : public QDialog {
    Q_OBJECT
  public:
    IdentificationDialog(Session &,Version,quint32,bool demo,QWidget *parent=nullptr,int mode=3);
  signals:
    void parametersApplied(QJsonObject);
    void parametersSaved(QJsonObject);
  protected:
    void done(int) override;
  private:
    Session &session_;
    Version version_;
    quint32 node_;
    bool demo_, aligned_=false, known_=false, eligible_=false, applied_=false;
    bool mayRun_=false, polling_=true;
    quint64 query_=0, operation_=0, alignment_=0;
    int operationCommand_=0, operationAction_=0, runMode_=3;
    QJsonObject status_, appliedResult_, actionResult_;
    QTimer timer_;
    QElapsedTimer lastStatus_;
    QComboBox *mode_;
    std::array<QDoubleSpinBox*,7> values_{};
    QCheckBox *safe_;
    QGroupBox *config_;
    QLabel *phase_, *message_, *storage_;
    QProgressBar *progress_;
    QTableWidget *results_;
    QPushButton *start_, *stop_, *queryButton_, *apply_, *save_;
    IdentificationConfig config() const;
    bool fresh() const;
    bool idleTelemetry() const;
    void refresh();
    void query();
    void start();
    void stop();
    void apply();
    void save();
    void acceptStatus(const QJsonObject &);
    void failed(quint64,const QString &);
    void replied(quint64,quint32,int,const QJsonObject &);
    void message(const QString &);
};
}
