#include "window.h"
#include <QApplication>
#include <QCommandLineParser>
int main(int argc, char **argv) {
    QApplication app(argc, argv);
    app.setOrganizationName("MotorTune");
    app.setApplicationName("MotorTune");
    app.setApplicationVersion(MOTORTUNE_VERSION);
    app.setStyle("Fusion");
    QFont font("Inter", 10);
    font.setFamilies({"Inter", "Microsoft YaHei UI", "Segoe UI"});
    font.setWeight(QFont::Medium);
    font.setLetterSpacing(QFont::AbsoluteSpacing, .2);
    app.setFont(font);
    app.setStyleSheet(R"(
QWidget{background:#07080a;color:#f9f9f9;}QLabel{background:transparent;}QLabel#brand{font-size:22px;font-weight:600;}QLabel#pageTitle{font-size:24px;font-weight:600;}QLabel#muted{color:#9c9c9d;}QLabel#warning{color:#ffbc33;padding:12px;border:1px solid #434345;border-radius:8px;}
QPushButton{background:qlineargradient(x1:0,y1:0,x2:0,y2:1,stop:0 #222428,stop:1 #151719);border:1px solid #34373b;border-top:1px solid #484b50;border-bottom:1px solid #202124;border-radius:6px;padding:8px 12px;min-height:18px;font-weight:600;}QPushButton:hover{color:#b9b9bb;border-color:#686b70;}QPushButton:pressed{background:#101111;}QPushButton:disabled{color:#6a6b6c;border-color:#252829;}QPushButton#danger{color:#FF6363;background:#301b1e;border-color:#704044;}
QLineEdit,QSpinBox,QDoubleSpinBox,QComboBox{background:#101111;border:1px solid #303338;border-radius:6px;padding:7px;min-height:18px;selection-background-color:#294d68;}QLineEdit:focus,QSpinBox:focus,QDoubleSpinBox:focus,QComboBox:focus{border-color:#55b3ff;}QComboBox::drop-down{border:0;width:22px;}QComboBox QAbstractItemView{background:#1b1c1e;selection-background-color:#294d68;}
QGroupBox{background:#101111;border:1px solid #252829;border-radius:12px;margin-top:14px;padding:16px;}QGroupBox::title{subcontrol-origin:margin;left:16px;padding:0 6px;color:#cecece;}QListWidget#navigation{background:#101111;border:0;border-right:1px solid #252829;padding:14px 8px;}QListWidget::item{padding:12px 10px;border-radius:6px;margin-bottom:4px;color:#9c9c9d;}QListWidget::item:selected{background:#20313e;color:#f9f9f9;}QFrame#livePanel{background:#101111;border-left:1px solid #252829;}
QPlainTextEdit,QTableWidget,QListWidget{background:#101111;border:1px solid #252829;border-radius:8px;selection-background-color:#294d68;}QPlainTextEdit{font-family:Consolas,'Microsoft YaHei UI';padding:10px;}QHeaderView::section{background:#1b1c1e;padding:10px;border:0;border-bottom:1px solid #34373b;}QTableWidget{gridline-color:#252829;}QCheckBox{spacing:7px;}QCheckBox::indicator{width:16px;height:16px;}QProgressBar{background:#101111;border:1px solid #252829;border-radius:6px;text-align:center;height:24px;}QProgressBar::chunk{background:#55b3ff;border-radius:4px;}QStatusBar{background:#101111;color:#9c9c9d;border-top:1px solid #252829;}QScrollBar:vertical{background:#101111;width:10px;}QScrollBar::handle:vertical{background:#434345;min-height:24px;border-radius:5px;}QScrollBar::add-line:vertical,QScrollBar::sub-line:vertical{height:0;}
)");
    QCommandLineParser p;
    p.addHelpOption();
    p.addVersionOption();
    p.addOption({"demo", "Start simulated backend"});
    p.addOption({"factory", "Factory role"});
    p.addOption({"factory-locked", "Lock factory role"});
    p.addOption({"smoke-test", "Offscreen lifecycle / rendering smoke test"});
    p.addOption({"screenshot", "Save screenshot after Demo warmup", "path"});
    p.addOption({"capture-pages", "Capture all application pages", "directory"});
    p.process(app);
    bool smoke = p.isSet("smoke-test");
    QSettings installation(QCoreApplication::applicationDirPath() + "/installation.ini",
                           QSettings::IniFormat);
    mt::Window window(p.isSet("factory") || installation.value("factory", false).toBool(),
                      p.isSet("factory-locked") || installation.value("factoryLocked", false).toBool(),
                      smoke);
    window.show();
    if (p.isSet("demo") || smoke || p.isSet("screenshot"))
        window.startDemo();
    if (p.isSet("screenshot"))
        QTimer::singleShot(6000, &window, [&] {
            window.grab().save(p.value("screenshot"));
            if (!smoke && !p.isSet("demo"))
                app.quit();
        });
    if (p.isSet("capture-pages")) {
        QString directory = p.value("capture-pages");
        QDir().mkpath(directory);
        for (int i = 0; i < 12; i++)
            QTimer::singleShot(2500 + i * 120, &window, [&, i, directory] {
                window.findChild<QListWidget *>("navigation")->setCurrentRow(i);
                QTimer::singleShot(60, &window, [&, i, directory] {
                    window.grab().save(directory + QString("/page-%1.png").arg(i, 2, 10, QChar('0')));
                });
            });
    }
    if (smoke)
        QTimer::singleShot(600, &window, [&] { window.startDemoChecks(); });
    if (smoke)
        QTimer::singleShot(10000, &window, [&] {
            bool ok = window.smokeCheck();
            window.session.shutdown();
            window.session.wait();
            app.exit(ok ? 0 : 2);
        });
    return app.exec();
}
