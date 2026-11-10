#pragma once
#include "archive.h"
#include "motion_control.h"
#include "notice_bar.h"
#include "scope.h"
#include "session.h"
#include <QMainWindow>
#include <QtWidgets>
#include <functional>
namespace mt {
class Window : public QMainWindow {
    Q_OBJECT
  public:
    explicit Window(bool factory = false, bool locked = false, bool smoke = false);
    ~Window() override;
    Session session;
    void startDemo();
    bool smokeCheck() const;
    void startDemoChecks();

  protected:
    void closeEvent(QCloseEvent *) override;
    bool eventFilter(QObject *, QEvent *) override;

  private:
    Connection connection_;
    Archive archive_;
    Scope *scope_ = nullptr;
    MotionControl *motion_ = nullptr;
    NoticeBar *notice_ = nullptr;
    quint64 enableToken_ = 0, disableToken_ = 0, resetToken_ = 0;
    QPushButton *disable_ = nullptr, *faultReset_ = nullptr;
    QScrollArea *controlSidebar_ = nullptr;
    QStackedWidget *pages_;
    QListWidget *navigation_;
    QComboBox *nodes_, *mode_, *role_;
    QLabel *connectionLabel_, *live_, *triggerLabel_, *statsLabel_;
    QPushButton *connect_, *enable_;
    QPlainTextEdit *log_ = nullptr;
    QTimer refresh_, experimentTimer_;
    QFutureWatcher<QString> exportWatcher_;
    std::atomic<bool> exportCancelled_{false};
    QCheckBox *unloaded_;
    QLineEdit *sn_;
    QProgressBar *iapBar_;
    QLabel *iapLabel_;
    QTableWidget *testTable_;
    QPlainTextEdit *report_;
    QListWidget *steps_;
    QLabel *stepDetail_;
    QPushButton *nextStep_;
    QLineEdit *query_;
    std::array<std::array<QDoubleSpinBox *, 3>, 3> pid_{};
    QJsonObject testResults_;
    QHash<quint64, std::function<void(QJsonObject)>> callbacks_;
    QHash<quint64, std::function<void(QString)>> failures_;
    QJsonObject savedSnapshot_;
    qint64 archiveId_ = -1;
    int wizardStep_ = 0, testIndex_ = -1, experimentTicks_ = 0;
    bool smoke_ = false, locked_ = false, paused_ = false, testsRunning_ = false;
    QCheckBox *allowMotion_, *stopOnFail_;
    QJsonObject persistedTest_;
    double experimentStart_ = 0, experimentDuration_ = 0, experimentAmplitude_ = 0, temperatureLimit_ = 60;
    int experimentKind_ = 0;
    QElapsedTimer uptime_;
    QStringList logs_;
    quint64 refreshCount_ = 0;
    int captureMode_ = 0;
    void buildUi();
    Sampling &activeHistory();
    void updateScopeProfile();
    void configureDebug(bool stop = false);
    void refreshDebug();
    QComboBox *scopeProfile_ = nullptr, *debugLoop_ = nullptr, *debugMaskChoice_ = nullptr;
    QSpinBox *debugRate_ = nullptr;
    QWidget *debugControls_ = nullptr;
    QLabel *debugStatus_ = nullptr, *analysisContext_ = nullptr;
    quint64 debugToken_ = 0, debugGeneration_ = 0;
    quint32 debugNode_ = 0;
    int debugMask_ = 0, debugHz_ = 100, shownProfile_ = 0;
    bool debugConfirmed_ = false, debugPending_ = false;
    QString debugError_;
    std::array<QCheckBox *, channelCount> channelBoxes_{};
    std::array<bool, channelCount> normalVisible_{true, true}, debugVisible_{};
    QStringList normalAliases_ = channelNames;
    QComboBox *scopeAxis_ = nullptr, *triggerChannel_ = nullptr, *analysisChannel_ = nullptr;
    QComboBox *scopeView_ = nullptr;
    QToolButton *debugMoreSignals_ = nullptr;
    QPushButton *debugApply_ = nullptr, *debugStop_ = nullptr;
    QWidget *scopePage();
    QWidget *configPage();
    QWidget *pidPage();
    QWidget *analysisPage();
    QWidget *experimentPage();
    QWidget *torquePage();
    QWidget *iapPage();
    QWidget *factoryPage();
    QWidget *testPage();
    QWidget *reportPage();
    QWidget *extensionPage();
    QWidget *logPage();
    void connectDialog();
    void openConnection();
    void refresh();
    void log(QString, QString);
    void recordLog(QString, QString, qint64 wallMs);
    void safeAction(int, const QByteArray &, const QString &, std::function<void(QJsonObject)> done = {});
    quint32 node() const;
    bool confirm(const QString &);
    void querySnapshot(std::function<void(QJsonObject)>);
    void runNextTest();
    void testVerdict(int, const QString &, const QString &);
    void runWizardStep();
    void exportReport();
    void stopExperiment();
    void updateRole();
    quint64 request(int, const QByteArray &payload = {}, std::function<void(QJsonObject)> done = {},
                    std::function<void(QString)> fail = {});
};
} // namespace mt
