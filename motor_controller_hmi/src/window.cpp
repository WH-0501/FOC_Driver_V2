#include "window.h"
#include "sweep_dialog.h"
#include <QJsonDocument>
#include <QSaveFile>
#include <QStandardPaths>
#include <QtConcurrent>
#include <cmath>
namespace mt {
namespace {
QPushButton *button(QBoxLayout *l, const QString &text, std::function<void()> fn) {
    auto b = new QPushButton(text);
    l->addWidget(b);
    QObject::connect(b, &QPushButton::clicked, b, std::move(fn));
    return b;
}
QWidget *page(const QString &title, const QString &description, QVBoxLayout *&l) {
    auto w = new QWidget;
    l = new QVBoxLayout(w);
    l->setContentsMargins(24, 20, 24, 20);
    l->setSpacing(16);
    auto h = new QLabel(title);
    h->setObjectName("pageTitle");
    l->addWidget(h);
    auto d = new QLabel(description);
    d->setWordWrap(true);
    d->setObjectName("muted");
    l->addWidget(d);
    return w;
}
QDoubleSpinBox *number(double low, double high, double value, int digits = 4) {
    auto w = new QDoubleSpinBox;
    w->setRange(low, high);
    w->setDecimals(digits);
    w->setValue(value);
    w->setSingleStep(std::pow(10, -qMin(digits, 2)));
    return w;
}
} // namespace
Window::Window(bool factory, bool locked, bool smoke) : smoke_(smoke), locked_(locked) {
    qApp->installEventFilter(this);
    uptime_.start();
    setWindowTitle(QString("MotorTune %1 · FOC 调试工作台").arg(MOTORTUNE_VERSION));
    resize(1480, 920);
    setMinimumSize(1080, 720);
    QString embedded = QCoreApplication::applicationDirPath() + "/compat/python/python.exe";
    connection_.python =
        QSettings().value("python", QFileInfo::exists(embedded) ? embedded : QString("python")).toString();
    connection_.driverDirectory = QSettings().value("driverDirectory").toString();
    connection_.historyCapacity =
        size_t(std::clamp(QSettings().value("historyCapacity", 1000000).toInt(), 120000, 10000000));
    auto root = QStandardPaths::writableLocation(QStandardPaths::AppLocalDataLocation);
    if (smoke_)
        root = QDir::tempPath() + "/motortune-smoke-" + QString::number(QCoreApplication::applicationPid());
    QDir().mkpath(root);
    QString error;
    if (!archive_.open(root + "/factory_archive.sqlite", &error))
        QMessageBox::warning(this, "归档不可用", error);
    buildUi();
    role_->setCurrentIndex(factory || locked ? 1 : 0);
    role_->setEnabled(!locked);
    updateRole();
    connect(
        &session, &Session::sourceLog, this,
        [this](QString level, QString text, qint64 wallMs, qint64 originNs) {
            if (level == "ACTION" || level == "TX" || level == "TIMING")
                text += QString(" | UI投递 %1 ms").arg((monotonicNs() - originNs) / 1e6, 0, 'f', 3);
            recordLog(level, text, wallMs);
        },
        Qt::QueuedConnection);
    debugVisible_[0] = debugVisible_[11] = true;
    connect(&session, &Session::connectionChanged, this, [this](bool on, QString text) {
        connectionLabel_->setText(on ? "●  " + text : "○  未连接");
        connectionLabel_->setToolTip(text);
        connect_->setText(on ? "断开连接" : "连接设备");
        connect_->setEnabled(on || !session.isRunning());
        if (!on) {
            enableToken_ = disableToken_ = resetToken_ = 0;
            debugToken_ = 0;
            debugMask_ = 0;
            debugConfirmed_ = debugPending_ = false;
            ++debugGeneration_;
        }
        log(on || text == "已断开" ? "INFO" : "ERROR", text);
        if (on)
            notice_->post("SUCCESS", "连接成功：" + text, true);
        else if (text == "已断开")
            notice_->post("INFO", text);
    });
    connect(&session, &QThread::finished, this, [this] { connect_->setEnabled(true); });
    connect(&session, &Session::nodeSeen, this, [this](quint32 id) {
        if (nodes_->findData(id) < 0)
            nodes_->addItem(QString("Node %1").arg(id), id);
    });
    connect(&session, &Session::replied, this, [this](quint64 token, quint32 n, int c, QJsonObject o) {
        if (token && token == resetToken_)
            resetToken_ = 0;
        bool stateRequest = token && (token == enableToken_ || token == disableToken_);
        if (token && token == enableToken_)
            enableToken_ = 0;
        if (token && token == disableToken_)
            disableToken_ = 0;
        if (o.contains("ok") && !o["ok"].toBool()) {
            auto f = failures_.take(token);
            callbacks_.remove(token);
            QString reason = QString("Node %1 · 0x%2 操作失败：%3")
                                 .arg(n)
                                 .arg(c, 2, 16, QChar('0'))
                                 .arg(o["message"].toString("设备拒绝该操作"));
            log("ERROR", reason);
            if (f)
                f(reason);
            return;
        }
        if (stateRequest && o["telemetry_confirmed"].toBool()) {
            auto text = QString("Node %1 · %2").arg(n).arg(o["message"].toString());
            log("INFO", text);
            notice_->post("SUCCESS", text, true);
        }
        auto fn = callbacks_.take(token);
        failures_.remove(token);
        if (fn)
            fn(o);
        if (n == node() && c >= 0x47 && c <= 0x49 && o.contains("P")) {
            QStringList keys = {"P", "I", "D"};
            for (int k = 0; k < 3; k++) {
                QSignalBlocker block(pid_[c - 0x47][k]);
                pid_[c - 0x47][k]->setValue(o[keys[k]].toDouble());
                pid_[c - 0x47][k]->setStyleSheet("");
            }
        }
    });
    connect(&session, &Session::commandFailed, this, [this](quint64 t, QString why) {
        if (t == resetToken_)
            resetToken_ = 0;
        if (t == enableToken_)
            enableToken_ = 0;
        if (t == disableToken_)
            disableToken_ = 0;
        callbacks_.remove(t);
        auto fn = failures_.take(t);
        log(why.contains("取消") || why == "连接已关闭" ? "INFO" : "ERROR", why);
        if (fn)
            fn(why);
    });
    connect(&session, &Session::iapProgress, this, [this](int n, QString s) {
        iapBar_->setValue(n);
        iapLabel_->setText(s);
    });
    connect(&session, &Session::iapFinished, this, [this](bool ok, QString s) {
        iapLabel_->setText(s);
        if (ok)
            iapBar_->setValue(100);
        log(ok ? "INFO" : "ERROR", s);
        if (ok)
            notice_->post("SUCCESS", s);
    });
    connect(&refresh_, &QTimer::timeout, this, &Window::refresh);
    refresh_.start(33);
    connect(&experimentTimer_, &QTimer::timeout, this, [this] {
        double elapsed = uptime_.elapsed() / 1000.0 - experimentStart_;
        auto s = session.latest(node());
        if (!s || s->error || elapsed >= experimentDuration_ ||
            (std::isfinite(s->values[8]) && s->values[8] >= temperatureLimit_)) {
            stopExperiment();
            return;
        }
        double value = experimentAmplitude_;
        if (experimentKind_ == 1)
            value *= sin(2 * std::numbers::pi * (.5 * elapsed + 2 * elapsed * elapsed / experimentDuration_));
        request(3, target(quint8(node()), value, 2, connection_.version));
    });
    if (!smoke_)
        restoreGeometry(QSettings().value("geometry").toByteArray());
}
Window::~Window() {
    exportCancelled_ = true;
    session.shutdown();
    session.wait();
    exportWatcher_.waitForFinished();
}
bool Window::eventFilter(QObject *watched, QEvent *event) {
    if (event->type() == QEvent::KeyPress) {
        const auto *key = static_cast<QKeyEvent *>(event);
        if (key->key() == Qt::Key_E && (key->modifiers() & (Qt::ControlModifier | Qt::ShiftModifier)) ==
                                           (Qt::ControlModifier | Qt::ShiftModifier)) {
            stopExperiment();
            testsRunning_ = false;
            session.emergency();
            return true;
        }
    }
    return QMainWindow::eventFilter(watched, event);
}
void Window::closeEvent(QCloseEvent *e) {
    exportCancelled_ = true;
    stopExperiment();
    session.shutdown();
    if (!smoke_)
        QSettings().setValue("geometry", saveGeometry());
    e->accept();
}
quint32 Window::node() const {
    return nodes_->currentData().toUInt();
}
void Window::startDemo() {
    connection_.backend = "Demo";
    openConnection();
}
void Window::openConnection() {
    connect_->setEnabled(false);
    connect_->setText("正在连接…");
    connectionLabel_->setText("○  正在连接");
    notice_->post("INFO", "正在连接 " + connection_.backend, true);
    try {
        session.open(connection_);
    } catch (const std::exception &e) {
        connect_->setEnabled(true);
        connect_->setText("连接设备");
        connectionLabel_->setText("○  未连接");
        log("ERROR", QString("连接失败：%1").arg(QString::fromUtf8(e.what())));
    }
}
void Window::startDemoChecks() {
    if (connection_.backend != "Demo")
        return;
    sn_->setText("DEMO-SMOKE");
    request(0x4d, {}, [this](QJsonObject o) {
        if (!o["ok"].toBool())
            return;
        querySnapshot([this](QJsonObject snapshot) {
            QString e;
            archiveId_ = archive_.save(sn_->text(), int(node()), snapshot, true, &e);
            savedSnapshot_ = snapshot;
            testResults_ = {};
            testIndex_ = -1;
            testsRunning_ = true;
            runNextTest();
        });
    });
}
bool Window::smokeCheck() const {
    bool passed = true;
    for (auto value : testResults_)
        if (value.toObject()["verdict"].toString() == "FAIL")
            passed = false;
    if (!passed)
        qWarning() << QJsonDocument(testResults_).toJson();
    return passed && archiveId_ > 0 && testResults_.size() == 31 && !testsRunning_ && session.rxCount > 100 &&
           refreshCount_ > 10 && scope_ && !scope_->samples().empty();
}
void Window::buildUi() {
    auto central = new QWidget;
    auto root = new QVBoxLayout(central);
    root->setContentsMargins(0, 0, 0, 0);
    root->setSpacing(0);
    auto toolbar = new QHBoxLayout;
    toolbar->setContentsMargins(20, 14, 20, 14);
    auto logo = new QLabel("M /  MotorTune");
    logo->setObjectName("brand");
    toolbar->addWidget(logo);
    role_ = new QComboBox;
    role_->addItems({"研发工作台", "产线工作台"});
    toolbar->addWidget(role_);
    toolbar->addStretch();
    connectionLabel_ = new QLabel("○  未连接");
    toolbar->addWidget(connectionLabel_);
    connect_ = button(toolbar, "连接设备", [this] {
        if (session.isRunning()) {
            connect_->setEnabled(false);
            connect_->setText("正在断开…");
            session.shutdown();
            return;
        }
        connectDialog();
    });
    root->addLayout(toolbar);
    auto controls = new QHBoxLayout;
    controls->setContentsMargins(20, 8, 20, 12);
    nodes_ = new QComboBox;
    nodes_->addItem("Node 1", 1);
    nodes_->setMinimumWidth(115);
    controls->addWidget(nodes_);
    button(controls, "扫描节点", [this] {
        request(0x52);
        session.command(0, 0x52);
    });
    mode_ = new QComboBox;
    mode_->addItems(modes(connection_.version));
    mode_->setCurrentIndex(5);
    controls->addWidget(mode_);
    button(controls, "应用模式", [this] {
        notice_->post("INFO", "模式切换已请求，等待设备遥测确认", true);
        request(0x41, QByteArray(1, char(mode_->currentIndex())), [this](QJsonObject o) {
            notice_->post("SUCCESS", o["message"].toString("模式已确认"), true);
        });
    });
    enable_ = button(controls, "使能", [this] {
        if (!unloaded_->isChecked()) {
            log("WARNING", "请先确认空载和工作区安全");
            return;
        }
        if (confirm("使能电机可能产生运动，确认工作区安全？")) {
            notice_->post("INFO", "使能已请求，等待设备遥测确认", true);
            enableToken_ = request(0x40, QByteArray(1, 1));
        }
    });
    disable_ = button(controls, "失能", [this] {
        // Cancel producers before enqueueing the priority stop; do not enqueue an
        // emergency here that would cancel this very disable transaction.
        experimentTimer_.stop();
        testsRunning_ = false;
        notice_->post("INFO", "失能已请求，等待设备遥测确认", true);
        disableToken_ = request(0x40, QByteArray(1, 0));
    });
    faultReset_ = button(controls, "异常复位", [this] {
        experimentTimer_.stop();
        testsRunning_ = false;
        notice_->post("INFO", "异常复位已请求，等待设备应答", true);
        resetToken_ = request(0x39);
        faultReset_->setEnabled(false);
    });
    faultReset_->setObjectName("faultReset");
    faultReset_->setEnabled(false);
    faultReset_->setToolTip("对当前节点发送异常复位 0x39；停止本机自动运动任务，不自动重新使能。");
    unloaded_ = new QCheckBox("空载 / 工作区安全");
    controls->addWidget(unloaded_);
    controls->addStretch();
    auto emergency = button(controls, "■  急停   Ctrl+Shift+E", [this] {
        stopExperiment();
        testsRunning_ = false;
        session.emergency();
    });
    emergency->setObjectName("danger");
    root->addLayout(controls);
    notice_ = new NoticeBar;
    root->addWidget(notice_);
    auto shortcut = new QShortcut(QKeySequence("Ctrl+Shift+E"), this);
    shortcut->setContext(Qt::ApplicationShortcut);
    connect(shortcut, &QShortcut::activated, this, [emergency] { emergency->click(); });
    auto body = new QHBoxLayout;
    body->setSpacing(0);
    navigation_ = new QListWidget;
    navigation_->setObjectName("navigation");
    navigation_->setFixedWidth(178);
    body->addWidget(navigation_);
    pages_ = new QStackedWidget;
    body->addWidget(pages_, 1);
    auto liveBox = new QFrame;
    liveBox->setObjectName("livePanel");
    liveBox->setFixedWidth(380);
    auto ll = new QVBoxLayout(liveBox);
    ll->setContentsMargins(12, 16, 12, 16);
    ll->setSpacing(10);
    auto statusCard = new QGroupBox("实时状态 / LIVE TELEMETRY");
    statusCard->setObjectName("liveStatusCard");
    statusCard->setStyleSheet("QGroupBox#liveStatusCard{padding:8px;margin-top:10px;}");
    statusCard->setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Fixed);
    auto statusLayout = new QVBoxLayout(statusCard);
    live_ = new QLabel("等待连接");
    live_->setTextFormat(Qt::RichText);
    live_->setWordWrap(true);
    live_->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Preferred);
    statusLayout->addWidget(live_);
    ll->addWidget(statusCard);
    auto separator = new QFrame;
    separator->setObjectName("sidebarSeparator");
    separator->setFixedHeight(1);
    separator->setStyleSheet("background:#303338;border:0;");
    ll->addWidget(separator);
    // Only the controls scroll; live status and the link summary stay pinned.
    auto controlContent = new QWidget;
    auto controlLayout = new QVBoxLayout(controlContent);
    controlLayout->setContentsMargins(0, 0, 0, 0);
    controlLayout->setSpacing(0);
    motion_ = new MotionControl(session);
    controlLayout->addWidget(motion_);
    controlLayout->addStretch();
    controlSidebar_ = new QScrollArea;
    controlSidebar_->setObjectName("controlSidebar");
    controlSidebar_->setWidgetResizable(true);
    controlSidebar_->setFrameShape(QFrame::NoFrame);
    controlSidebar_->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    controlSidebar_->setWidget(controlContent);
    ll->addWidget(controlSidebar_, 1);
    auto hint = new QLabel("1 Mbps / 5 Mbps · ISO CAN FD + BRS\n急停独立优先队列 "
                           "· 遥测超时仅告警");
    hint->setWordWrap(true);
    hint->setObjectName("muted");
    ll->addWidget(hint);
    body->addWidget(liveBox);
    root->addLayout(body, 1);
    setCentralWidget(central);
    QStringList names = {"◉   实时示波",    "◇   配置标定",   "≋   环路调参",    "∿   数据分析",
                         "↗   激励 / 温升", "⊙   力矩标定",   "↑   固件升级",    "①   标定向导",
                         "②   自动测试",    "③   报告与回读", "⌘   扩展 / 脚本", "≡   通信日志"};
    QList<QWidget *> all = {scopePage(),      configPage(), pidPage(),       analysisPage(),
                            experimentPage(), torquePage(), iapPage(),       factoryPage(),
                            testPage(),       reportPage(), extensionPage(), logPage()};
    for (int i = 0; i < all.size(); i++) {
        navigation_->addItem(names[i]);
        auto scroll = new QScrollArea;
        scroll->setWidgetResizable(true);
        scroll->setFrameShape(QFrame::NoFrame);
        scroll->setWidget(all[i]);
        pages_->addWidget(scroll);
    }
    connect(navigation_, &QListWidget::currentRowChanged, pages_, &QStackedWidget::setCurrentIndex);
    navigation_->setCurrentRow(0);
    connect(role_, &QComboBox::currentIndexChanged, this, [this] { updateRole(); });
    connect(nodes_, &QComboBox::currentIndexChanged, this, [this] {
        enableToken_ = disableToken_ = resetToken_ = 0;
        stopExperiment();
        testsRunning_ = false;
        if (debugMask_ && session.connected() && !session.upgrading())
            session.command(debugNode_, 0xb8, QByteArray::fromHex("01000064"));
        ++debugGeneration_;
        debugMask_ = 0;
        debugToken_ = 0;
        debugConfirmed_ = debugPending_ = false;
        session.selectNode(node());
        scope_->clearDisplay();
    });
    statusBar()->showMessage("MotorTune " MOTORTUNE_VERSION " · Ready");
}
void Window::updateRole() {
    bool f = role_->currentIndex() == 1;
    motion_->setVisible(!f);
    for (int i = 0; i < navigation_->count(); i++)
        navigation_->item(i)->setHidden(f ? !(i >= 7 && i <= 9) : i == 7);
    navigation_->setCurrentRow(f ? 7 : 0);
}
void Window::connectDialog() {
    QDialog d(this);
    d.setWindowTitle("连接 CAN / CAN FD");
    auto l = new QFormLayout(&d);
    auto backend = new QComboBox;
    backend->addItems(backendNames());
    l->addRow("设备后端", backend);
    auto version = new QComboBox;
    version->addItems({"V2.0.0（优先）", "V1.0.0"});
    l->addRow("协议版本", version);
    auto channelEditors = new QStackedWidget;
    auto canChannel = new QComboBox;
    canChannel->setObjectName("connectionCanChannel");
    canChannel->setMinimumWidth(180);
    auto channel = new QLineEdit("0");
    channelEditors->addWidget(canChannel);
    channelEditors->addWidget(channel);
    auto channelLabel = new QLabel;
    l->addRow(channelLabel, channelEditors);
    auto refreshChannels = [backend, canChannel, channel, channelEditors, channelLabel] {
        const auto model = backend->currentText();
        const bool native = model.contains("ZQWL") || model.contains("周立功") || model.contains("创芯") ||
                            model.startsWith("自动") || model == "Demo";
        if (native) {
            const int count = model.contains("200U") ? 2 : 1;
            const int previous = canChannel->currentData().toInt();
            canChannel->clear();
            for (int ch = 0; ch < count; ++ch)
                canChannel->addItem(QString("通道%1 (API CAN%2)").arg(ch + 1).arg(ch), ch);
            canChannel->setCurrentIndex(qBound(0, previous, count - 1));
            channelEditors->setCurrentWidget(canChannel);
            channelLabel->setText("CAN 通道");
        } else {
            channelEditors->setCurrentWidget(channel);
            channelLabel->setText("通道 / COM / can0");
        }
    };
    connect(backend, &QComboBox::currentTextChanged, &d, refreshChannels);
    refreshChannels();
    auto idx = new QSpinBox;
    idx->setRange(0, 15);
    l->addRow("设备索引", idx);
    auto directory = new QLineEdit(connection_.driverDirectory);
    l->addRow("64 位驱动 DLL 目录", directory);
    auto browse = new QPushButton("选择驱动目录");
    l->addRow(browse);
    connect(browse, &QPushButton::clicked, &d, [&] {
        auto path = QFileDialog::getExistingDirectory(&d, "驱动 DLL 目录");
        if (!path.isEmpty())
            directory->setText(path);
    });
    auto python = new QLineEdit(connection_.python);
    l->addRow("兼容后端 Python", python);
    auto help = new QLabel("原生：Demo / ZQWL / 周立功 / "
                           "创芯。其他后端使用隔离兼容进程。\nV1 位置为度 Q7；V2 为弧度 "
                           "Q7。连接后禁止更改协议。\n经典 CAN 不支持超过 8B 的帧及完整 "
                           "V2/IAP 能力。");
    help->setWordWrap(true);
    l->addRow(help);
    auto buttons = new QDialogButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel);
    l->addRow(buttons);
    connect(buttons, &QDialogButtonBox::accepted, &d, &QDialog::accept);
    connect(buttons, &QDialogButtonBox::rejected, &d, &QDialog::reject);
    if (d.exec() != QDialog::Accepted)
        return;
    const QString selectedChannel = channelEditors->currentWidget() == canChannel
                                        ? QString::number(canChannel->currentData().toInt())
                                        : channel->text();
    connection_ = {backend->currentText(), selectedChannel,
                   python->text(),         directory->text(),
                   idx->value(),           version->currentIndex() ? Version::V1 : Version::V2};
    QSettings().setValue("python", connection_.python);
    connection_.historyCapacity =
        size_t(std::clamp(QSettings().value("historyCapacity", 1000000).toInt(), 120000, 10000000));
    QSettings().setValue("driverDirectory", connection_.driverDirectory);
    mode_->clear();
    mode_->addItems(modes(connection_.version));
    mode_->setCurrentIndex(connection_.version == Version::V2 ? 5 : 4);
    openConnection();
}
void Window::refresh() {
    refreshDebug();
    refreshCount_++;
    if (pages_->currentIndex() == 0 || pages_->currentIndex() == 3) {
        scope_->bind(&activeHistory(), captureMode_);
        scope_->refresh();
    }
    // Numeric telemetry and controls do not need the chart's 30 Hz repaint rate.
    if (refreshCount_ % 3)
        return;
    motion_->setContext(node(), connection_.version, mode_->currentIndex(), unloaded_->isChecked(),
                        experimentTimer_.isActive() || testsRunning_);
    auto s = session.latest(node());
    if (s) {
        auto val = [&](int c) {
            return std::isfinite(s->values[c])
                       ? QString::number(s->values[c], 'f', (c == 6 || c == 9 || c == 11) ? 3 : 2)
                       : QString("—");
        };
        bool fresh = session.telemetryFresh(node());
        bool gateEnabled = session.motorEnabled(node());
        live_->setText(QString("<span style='color:%1'>● %2</span>"
                               "<table width='100%' cellspacing='4'>"
                               "<tr><td><small>位置 / rad</small><br><span "
                               "style='font-size:20px;font-weight:600'>%3</span></td>"
                               "<td><small>速度 / RPM</small><br><span "
                               "style='font-size:20px;font-weight:600'>%4</span></td></tr>"
                               "<tr><td><small>Iq / A</small><br><span "
                               "style='font-size:20px;font-weight:600'>%5</span></td>"
                               "<td><small>母线 / V</small><br><span "
                               "style='font-size:20px;font-weight:600'>%6</span></td></tr>"
                               "<tr><td><small>MCU / "
                               "°C</small><br><b>%7</b></td><td><small>设备模式</"
                               "small><br><b>%8</b></td></tr>"
                               "</table><small>故障码 0x%9 · %10</small>"
                               "<br><small>已提交目标（非设备确认）<br>位置 %11 rad · 速度 "
                               "%12 RPM · 电流 %13 A</small>")
                           .arg(!fresh       ? "#ffbc33"
                                : s->error   ? "#FF6363"
                                : gateEnabled ? "#5fc992"
                                              : "#9c9c9d")
                           .arg(!fresh ? "通信异常 · 按失能处理"
                                : gateEnabled ? "已使能"
                                              : "已失能")
                           .arg(val(6), val(5), val(0), val(7), val(8))
                           .arg(modes(connection_.version).value(s->mode, "未知"))
                           .arg(s->error, 2, 16, QChar('0'))
                           .arg(telemetryErrorName(s->error, connection_.version))
                           .arg(val(9), val(10), val(11)));
        enable_->setEnabled(session.telemetryFresh(node()) && !gateEnabled && !session.upgrading() &&
                            !enableToken_ && !disableToken_);
    } else {
        live_->setText("等待当前节点遥测");
        enable_->setEnabled(false);
    }
    enable_->setText(enableToken_ ? "使能确认中…" : "使能");
    disable_->setEnabled(session.connected() && !session.upgrading() && !disableToken_);
    disable_->setText(disableToken_ ? "失能确认中…" : "失能");
    faultReset_->setEnabled(session.connected() && !session.upgrading() && !resetToken_);
    faultReset_->setText(resetToken_ ? "复位处理中…" : "异常复位");
    triggerLabel_->setText(activeHistory().triggerState());
    auto history = activeHistory().info();
    statusBar()->showMessage(
        QString("%1  |  协议 %2  |  Node %3  |  RX %4  TX %5  |  环形覆盖 %6  |  "
                "30 Hz 波形  |  v%7  |  记录 "
                "%8 / %9 点 · %10 s")
            .arg(connection_.backend, connection_.version == Version::V2 ? "V2.0.0" : "V1.0.0")
            .arg(node())
            .arg(session.rxCount.load())
            .arg(session.txCount.load())
            .arg(session.sampling.overwritten())
            .arg(MOTORTUNE_VERSION)
            .arg(history.size)
            .arg(history.capacity)
            .arg(history.last - history.first, 0, 'f', 1));
}
void Window::log(QString level, QString text) {
    recordLog(level, text, QDateTime::currentMSecsSinceEpoch());
}
void Window::recordLog(QString level, QString text, qint64 wallMs) {
    QString line =
        QDateTime::fromMSecsSinceEpoch(wallMs).toString("HH:mm:ss.zzz") + " [" + level + "] " + text;
    logs_.append(line);
    while (logs_.size() > 3000)
        logs_.removeFirst();
    if (log_ && line.contains(log_->property("filter").toString(), Qt::CaseInsensitive))
        log_->appendPlainText(line);
    if (notice_ && (level == "ERROR" || level == "WARNING"))
        notice_->post(level, text);
}
bool Window::confirm(const QString &text) {
    if (smoke_)
        return false;
    return QMessageBox::question(this, "确认操作", text, QMessageBox::Yes | QMessageBox::No,
                                 QMessageBox::No) == QMessageBox::Yes;
}
quint64 Window::request(int c, const QByteArray &p, std::function<void(QJsonObject)> done,
                        std::function<void(QString)> fail) {
    auto token = session.command(node(), c, p);
    if (done) {
        auto selected = node();
        callbacks_[token] = [this, selected, done = std::move(done)](QJsonObject o) {
            if (node() == selected && session.connected())
                done(o);
            else
                log("WARNING", "节点已切换，忽略旧操作的界面回调");
        };
    }
    if (fail)
        failures_[token] = std::move(fail);
    return token;
}
void Window::safeAction(int c, const QByteArray &p, const QString &title,
                        std::function<void(QJsonObject)> done) {
    auto s = session.latest(node());
    if (!s || session.motorEnabled(node())) {
        log("WARNING", session.telemetryFresh(node()) ? "操作要求目标节点已失能"
                                                      : "操作要求目标节点失能或遥测已确认失能；通信异常时请先确认现场安全");
        return;
    }
    if (confirm(title))
        request(c, p, std::move(done));
}
Sampling &Window::activeHistory() {
    return scopeProfile_ && scopeProfile_->currentIndex() == 1
               ? session.debugSampling[size_t(debugLoop_->currentIndex())]
               : session.sampling;
}
void Window::updateScopeProfile() {
    bool debug = scopeProfile_->currentIndex() == 1;
    if (shownProfile_ == 0) {
        normalVisible_ = scope_->visible;
        normalAliases_ = scope_->aliases;
    } else
        debugVisible_ = scope_->visible;
    shownProfile_ = debug ? 1 : 0;
    scope_->debugProfile = debug;
    scope_->aliases = debug ? debugChannelNames(debugLoop_->currentIndex()) : normalAliases_;
    scope_->visible = debug ? debugVisible_ : normalVisible_;
    debugControls_->setVisible(debug);
    debugLoop_->setVisible(debug);
    debugMoreSignals_->setVisible(debug);
    for (int c = 0; c < channelCount; ++c) {
        QSignalBlocker blocker(channelBoxes_[c]);
        bool primary = c == 0 || c == 1 || c == 2 || c == 3 || c == 5 || c == 6 || c == 11;
        channelBoxes_[c]->setVisible(debug ? primary || debugMoreSignals_->isChecked() || scope_->visible[c]
                                           : c < normalChannelCount);
        channelBoxes_[c]->setText(scope_->aliases.value(c));
        channelBoxes_[c]->setChecked(scope_->visible[c]);
        channelBoxes_[c]->setToolTip(debug    ? "设备调试快照；给定与反馈来自同一帧，时间轴为本环PID更新时间"
                                     : c >= 9 ? "主机已提交目标，不表示设备执行确认"
                                              : "设备正常遥测");
    }
    for (auto combo : {scopeAxis_, triggerChannel_, analysisChannel_})
        if (combo) {
            QSignalBlocker blocker(combo);
            combo->clear();
            combo->addItems(debug ? debugChannelNames(debugLoop_->currentIndex()) : channelNames);
        }
    scope_->axisChannel = 0;
    if (debug && scopeView_->currentIndex() == 1)
        scopeView_->setCurrentIndex(0);
    if (auto model = qobject_cast<QStandardItemModel *>(scopeView_->model()))
        model->item(1)->setEnabled(!debug);
    scope_->clearDisplay();
    scope_->bind(&activeHistory(), captureMode_);
    if (analysisContext_)
        analysisContext_->setText(
            debug
                ? debugLoop_->currentText() +
                      " · 设备环内给定 / "
                      "PID更新时间；自动阶跃要求变化前后给定各保持至少5帧。调试为抽样快照，非完整控制周期录波"
                : "正常遥测 · 主机入队目标 / 主机接收反馈时间（端到端，非纯PID响应）");
    if (auto table = findChild<QTableWidget *>("stepList"))
        table->setRowCount(0);
    if (auto details = findChild<QPlainTextEdit *>("stepDetails"))
        details->clear();
}
void Window::configureDebug(bool stop) {
    if (!session.connected() || session.upgrading()) {
        debugConfirmed_ = false;
        log("WARNING", "调试流需要已连接且未进行固件升级");
        return;
    }
    if (connection_.version != Version::V2 && !stop) {
        log("WARNING", "PID调试流需要V2协议固件支持");
        return;
    }
    int mask = stop ? 0 : debugMaskChoice_->currentIndex() ? 7 : (1 << debugLoop_->currentIndex());
    int hz = debugRate_->value();
    if (hz * (mask == 7 ? 3 : 1) > 1000) {
        log("WARNING", "调试总帧率不能超过1000 Hz；三环每环最高333 Hz");
        return;
    }
    if (debugToken_ && !stop)
        return;
    {
        ++debugGeneration_;
        debugPending_ = true;
        debugConfirmed_ = false;
    }
    if (stop)
        debugMask_ = 0;
    quint32 target = stop ? (debugNode_ ? debugNode_ : node()) : node();
    auto generation = debugGeneration_;
    debugNode_ = target;
    debugError_.clear();
    auto failure = [this, generation](QString why) {
        if (generation != debugGeneration_)
            return;
        debugToken_ = 0;
        debugPending_ = debugConfirmed_ = false;
        debugError_ = why;
    };
    debugToken_ = request(
        0xb8, QByteArray(1, 1) + QByteArray(1, char(mask)) + be16(hz),
        [this, generation, mask, hz, failure](QJsonObject reply) {
            if (generation != debugGeneration_)
                return;
            debugToken_ = 0;
            debugPending_ = false;
            if (reply["mask"].toInt(-1) != mask || (mask && reply["hz"].toInt() != hz)) {
                failure("调试配置回读与请求不一致，未确认切换成功");
                log("ERROR", debugError_);
                return;
            }
            debugMask_ = mask;
            debugHz_ = hz;
            debugConfirmed_ = mask != 0;
        },
        failure);
}
void Window::refreshDebug() {
    if (!debugStatus_)
        return;
    debugApply_->setEnabled(session.connected() && !session.upgrading() &&
                            connection_.version == Version::V2 && !debugPending_);
    debugStop_->setEnabled(session.connected() && !session.upgrading());
    if (scopeProfile_->currentIndex() != 1)
        return;
    auto all = session.parameters(node());
    int loop = debugLoop_->currentIndex();
    auto values = all[QString::number(0xb9 + loop, 16)].toObject();
    QString state = debugPending_     ? "配置确认中…"
                    : debugConfirmed_ ? "调试流已确认"
                                      : "调试流未开启 / 已停止";
    if (!debugError_.isEmpty())
        state = debugError_;
    if (debugConfirmed_ && !(debugMask_ & (1 << loop)))
        state += " · 此环未订阅";
    bool stale =
        values.isEmpty() || QDateTime::currentMSecsSinceEpoch() - values["received_utc_ms"].toDouble() >
                                std::max(3000.0, 3000.0 / debugHz_);
    state += stale                       ? " · 等待新调试帧"
             : values["active"].toBool() ? " · 环正在执行"
                                         : " · 环未执行，显示历史PID状态";
    debugStatus_->setText(state);
}
QWidget *Window::scopePage() {
    QVBoxLayout *l;
    auto w = page("实时示波器",
                  "原始点完整记录 · 多级峰值索引 · 暂停显示继续采集    /    "
                  "滚轮缩放，拖动平移，Shift 框选",
                  l);
    l->setSpacing(8);
    auto workspace = new QHBoxLayout;
    scopeProfile_ = new QComboBox;
    scopeProfile_->setObjectName("scopeProfile");
    scopeProfile_->addItems({"正常遥测", "PID 调试"});
    workspace->addWidget(scopeProfile_);
    debugLoop_ = new QComboBox;
    debugLoop_->setObjectName("debugLoop");
    debugLoop_->addItems({"电流环 · A → V", "速度环 · rad/s → A", "位置环 · rad → rad/s"});
    debugLoop_->hide();
    workspace->addWidget(debugLoop_);
    workspace->addStretch();
    button(workspace, "阶跃 / 数据分析", [this] { navigation_->setCurrentRow(3); });
    button(workspace, "PID 参数", [this] { navigation_->setCurrentRow(2); });
    l->addLayout(workspace);
    debugControls_ = new QWidget;
    auto debugBar = new QHBoxLayout(debugControls_);
    debugBar->setContentsMargins(0, 0, 0, 0);
    debugMaskChoice_ = new QComboBox;
    debugMaskChoice_->setObjectName("debugSubscription");
    debugMaskChoice_->addItems({"订阅当前环", "订阅三环"});
    debugRate_ = new QSpinBox;
    debugRate_->setObjectName("debugRate");
    debugRate_->setRange(1, 1000);
    debugRate_->setValue(100);
    debugRate_->setSuffix(" Hz/环");
    debugBar->addWidget(debugMaskChoice_);
    debugBar->addWidget(debugRate_);
    debugApply_ = button(debugBar, "开始 / 应用", [this] { configureDebug(); });
    debugApply_->setObjectName("debugApply");
    debugStop_ = button(debugBar, "停止调试流", [this] { configureDebug(true); });
    debugStatus_ = new QLabel("设备 PID 时间轴 · 给定/反馈来自同一调试帧");
    debugStatus_->setWordWrap(true);
    debugStatus_->setToolTip("给定/反馈/积分/输出采用设备PID更新时间；Iq/"
                             "Id、母线和轴速度是发送时快照。调试流只抽样，不能还原每个控制周期。");
    debugBar->addWidget(debugStatus_, 1);
    debugControls_->hide();
    l->addWidget(debugControls_);
    auto bar = new QHBoxLayout;
    auto pauseDisplay = new QPushButton("Ⅱ 暂停显示");
    bar->addWidget(pauseDisplay);
    connect(pauseDisplay, &QPushButton::clicked, this, [this, pauseDisplay] {
        paused_ = !paused_;
        scope_->setDisplayPaused(paused_);
        pauseDisplay->setText(paused_ ? "▶ 继续显示" : "Ⅱ 暂停显示");
        log("INFO", paused_ ? "显示已冻结；缩放、分析与导出使用暂停记录，后台继续采集" : "已恢复显示");
    });
    button(bar, "重置视图", [this] { scope_->resetView(); });
    auto clearData = button(bar, "清除波形 / 数据", [this] {
        exportCancelled_ = true;
        activeHistory().clear();
        scope_->clearDisplay();
        statsLabel_->setText("记录已清除，等待新数据");
        if (auto table = findChild<QTableWidget *>("stepList"))
            table->setRowCount(0);
        if (auto details = findChild<QPlainTextEdit *>("stepDetails"))
            details->clear();
        if (auto summary = findChild<QLabel *>("stepSummary"))
            summary->setText("记录已清除，请重新采集后分析。");
        log("INFO", "已清除内存波形、采样历史、触发/故障抓拍和分析结果；连接与采集继续，已导出文件保留");
    });
    clearData->setObjectName("clearWaveData");
    clearData->setToolTip("清空内存记录与分析结果；连接时继续记录新数据，不删除已导出的文件。");
    auto view = new QComboBox;
    scopeView_ = view;
    view->addItems({"时域", "Id / Iq · XY", "FFT"});
    bar->addWidget(view);
    auto source = new QComboBox;
    source->addItems({"实时滚动", "触发抓拍", "故障快照"});
    bar->addWidget(source);
    button(bar, "T1 / T2", [this] {
        scope_->cursors = !scope_->cursors;
        if (scope_->cursors) {
            scope_->t1 = scope_->viewEnd() - scope_->windowSeconds * .75;
            scope_->t2 = scope_->viewEnd() - scope_->windowSeconds * .25;
        }
        scope_->update();
    });
    auto exportButton = button(bar, "导出 CSV", [this] {
        if (exportWatcher_.isRunning())
            return;
        auto file = QFileDialog::getSaveFileName(this, "导出波形", {}, "CSV (*.csv)");
        if (!file.isEmpty()) {
            int source = captureMode_;
            auto selectedNode = node();
            auto fault = session.faultSnapshot();
            auto history = &activeHistory();
            bool debug = scopeProfile_->currentIndex() == 1;
            const bool frozen = scope_->isDisplayPaused();
            auto targetEvents = scope_->targetEvents();
            auto frozenData = frozen ? scope_->recordedSamples(selectedNode) : std::vector<Sample>{};
            exportCancelled_ = false;
            log("INFO", "正在后台导出完整原始记录：" + file);
            exportWatcher_.setFuture(
                QtConcurrent::run([this, file, source, selectedNode, fault, history, debug, frozen,
                                   targetEvents, frozenData = std::move(frozenData)] {
                    QString error;
                    bool ok = source == 0 && !debug && !frozen
                                  ? history->exportHistory(file, selectedNode, &error, &exportCancelled_)
                                  : exportCsv(file,
                                              frozen        ? frozenData
                                              : source == 0 ? history->snapshot(selectedNode)
                                              : source == 1 ? history->capture()
                                                            : history->faultCapture(),
                                              &error, &exportCancelled_);
                    if (!ok)
                        return error;
                    if (!debug && !exportTargetEvents(file + ".targets.json", targetEvents, &error))
                        return error;
                    if (source == 2) {
                        QSaveFile meta(file + ".json");
                        if (!meta.open(QIODevice::WriteOnly))
                            return meta.errorString();
                        meta.write(QJsonDocument(fault).toJson());
                        if (!meta.commit())
                            return meta.errorString();
                    }
                    return QString();
                }));
        }
    });
    connect(&exportWatcher_, &QFutureWatcher<QString>::started, exportButton, [exportButton] {
        exportButton->setEnabled(false);
        exportButton->setText("正在导出…");
    });
    connect(&exportWatcher_, &QFutureWatcher<QString>::finished, this, [this, exportButton] {
        exportButton->setEnabled(true);
        exportButton->setText("导出 CSV");
        QString error = exportWatcher_.result();
        log(error.isEmpty() ? "INFO" : "ERROR", error.isEmpty() ? "原始记录导出完成" : error);
    });
    button(bar, "截图", [this] {
        auto f = QFileDialog::getSaveFileName(this, "示波截图", {}, "PNG (*.png)");
        if (!f.isEmpty() && !scope_->grab().save(f))
            log("ERROR", "截图保存失败");
    });
    l->addLayout(bar);
    scope_ = new Scope;
    scope_->bind(&session.sampling);
    l->addWidget(scope_, 1);
    auto axisRow = new QHBoxLayout;
    axisRow->addWidget(new QLabel("纵轴刻度"));
    auto axisChoice = new QComboBox;
    scopeAxis_ = axisChoice;
    axisChoice->setMinimumWidth(190);
    axisChoice->setObjectName("scopeYAxis");
    axisChoice->addItems(channelNames);
    axisRow->addWidget(axisChoice);
    auto axisHint =
        new QLabel("纵轴处滚轮或 Ctrl+滚轮同步缩放所有可见波形；图内滚轮缩放时间，重置视图恢复自动。");
    axisHint->setWordWrap(true);
    axisHint->setObjectName("muted");
    axisRow->addWidget(axisHint, 1);
    debugMoreSignals_ = new QToolButton;
    debugMoreSignals_->setText("更多调试信号");
    debugMoreSignals_->setCheckable(true);
    debugMoreSignals_->hide();
    axisRow->addWidget(debugMoreSignals_);
    connect(debugMoreSignals_, &QToolButton::toggled, this, [this](bool more) {
        if (scopeProfile_->currentIndex() != 1)
            return;
        for (int c = 0; c < channelCount; ++c) {
            bool primary = c == 0 || c == 1 || c == 2 || c == 3 || c == 5 || c == 6 || c == 11;
            channelBoxes_[c]->setVisible(primary || more || scope_->visible[c]);
        }
    });
    l->addLayout(axisRow);
    connect(axisChoice, &QComboBox::currentIndexChanged, scope_, [this](int c) {
        scope_->axisChannel = c;
        scope_->update();
    });
    connect(view, &QComboBox::currentIndexChanged, scope_, [this](int i) {
        scope_->view = i;
        scope_->update();
    });
    connect(source, &QComboBox::currentIndexChanged, this, [this](int i) {
        captureMode_ = i;
        scope_->follow = true;
        scope_->bind(&activeHistory(), i);
    });
    auto channelLayout = new QGridLayout;
    for (int i = 0; i < channelCount; i++) {
        auto box = new QCheckBox(channelNames.value(i));
        channelBoxes_[i] = box;
        box->setVisible(i < normalChannelCount);
        box->setChecked(scope_->visible[i]);
        box->setStyleSheet(QString("color:%1").arg(scope_->colors[i].name()));
        channelLayout->addWidget(box, i / 6, i % 6);
        if (i >= 9 && i < normalChannelCount)
            box->setToolTip("阶梯虚线：已提交驱动的主机目标，阶跃取入队时刻，不代表设备已执行；与对应反馈共"
                            "用量程。未发送时无曲线。");
        connect(box, &QCheckBox::toggled, scope_, [this, i](bool b) {
            scope_->visible[i] = b;
            scope_->update();
        });
        box->setContextMenuPolicy(Qt::CustomContextMenu);
        connect(box, &QWidget::customContextMenuRequested, this, [this, i, box] {
            auto color = QColorDialog::getColor(scope_->colors[i], this);
            if (color.isValid()) {
                scope_->colors[i] = color;
                box->setStyleSheet("color:" + color.name());
            }
            bool ok;
            auto alias =
                QInputDialog::getText(this, "通道别名", "名称", QLineEdit::Normal, scope_->aliases[i], &ok);
            if (ok && !alias.isEmpty()) {
                scope_->aliases[i] = alias;
                box->setText(alias);
            }
        });
    }
    l->addLayout(channelLayout);
    auto row = new QHBoxLayout;
    auto hz = new QSpinBox;
    hz->setRange(0, 1000);
    hz->setValue(1000);
    auto normalRateLabel = new QLabel("设备 Hz");
    row->addWidget(normalRateLabel);
    row->addWidget(hz);
    auto normalApply = button(row, "应用", [this, hz] { request(0x7f, be16(hz->value())); });
    hz->setToolTip("正常遥测频率；PID调试频率在上方单独配置");
    connect(scopeProfile_, &QComboBox::currentIndexChanged, hz,
            [hz, normalApply, normalRateLabel](int profile) {
                hz->setVisible(profile == 0);
                normalApply->setVisible(profile == 0);
                normalRateLabel->setVisible(profile == 0);
            });
    auto window = number(.001, 86400, 5, 3);
    row->addWidget(new QLabel("窗口 s"));
    row->addWidget(window);
    connect(window, &QDoubleSpinBox::valueChanged, scope_, [this](double v) { scope_->windowSeconds = v; });
    auto dec = new QSpinBox;
    dec->setRange(1, 100);
    row->addWidget(new QLabel("像素合并"));
    dec->setToolTip("只合并显示像素，保留每个区间的最大/最小值；原始记录不抽点");
    row->addWidget(dec);
    connect(dec, &QSpinBox::valueChanged, scope_, [this](int v) { scope_->decimation = v; });
    l->addLayout(row);
    auto recording = new QHBoxLayout;
    recording->addWidget(new QLabel("记录容量"));
    auto capacity = new QComboBox;
    for (int count : {120000, 1000000, 5000000, 10000000})
        capacity->addItem(QString("%1 万点 · 约 %2 MB")
                              .arg(count / 10000)
                              .arg((count * sizeof(Sample) + count * 5) / 1000000),
                          count);
    int capacityIndex = capacity->findData(int(connection_.historyCapacity));
    capacity->setCurrentIndex(capacityIndex < 0 ? 1 : capacityIndex);
    capacity->setToolTip("切换容量在下次连接时生效。缓存满后覆盖最早的点；导出 "
                         "CSV 保存完整原始记录。");
    recording->addWidget(capacity);
    connect(capacity, &QComboBox::currentIndexChanged, this, [this, capacity] {
        connection_.historyCapacity = size_t(capacity->currentData().toInt());
        QSettings().setValue("historyCapacity", int(connection_.historyCapacity));
        log("INFO", "记录容量将在下次连接时生效");
    });
    auto capacityHint = new QLabel("下次连接生效");
    recording->addWidget(capacityHint);
    connect(scopeProfile_, &QComboBox::currentIndexChanged, capacity, [capacity, capacityHint](int profile) {
        capacity->setVisible(profile == 0);
        capacityHint->setText(profile ? "调试每环独立保留 12 万点 · 仅清除当前环" : "下次连接生效");
    });
    recording->addStretch();
    button(recording, "查看全部记录", [this] { scope_->fitHistory(); });
    button(recording, "回到实时", [this, pauseDisplay] {
        paused_ = false;
        pauseDisplay->setText("Ⅱ 暂停显示");
        scope_->setDisplayPaused(false);
        scope_->resetView();
        scope_->refresh();
    });
    l->addLayout(recording);
    auto trigger = new QHBoxLayout;
    auto ch = new QComboBox;
    triggerChannel_ = ch;
    ch->addItems(channelNames);
    auto edge = new QComboBox;
    edge->addItems({"上升沿", "下降沿", "高电平"});
    auto level = number(-10000, 10000, .05);
    auto pre = number(0, 30, 1, 2), post = number(0, 30, 1, 2);
    trigger->addWidget(ch);
    trigger->addWidget(edge);
    trigger->addWidget(level);
    trigger->addWidget(new QLabel("前 / 后 s"));
    trigger->addWidget(pre);
    trigger->addWidget(post);
    button(trigger, "布防触发", [this, ch, edge, level, pre, post] {
        activeHistory().arm({ch->currentIndex(), level->value(), pre->value(), post->value(),
                             edge->currentIndex() == 0   ? 1
                             : edge->currentIndex() == 1 ? -1
                                                         : 0});
    });
    triggerLabel_ = new QLabel("未布防");
    trigger->addWidget(triggerLabel_);
    l->addLayout(trigger);
    connect(scopeProfile_, &QComboBox::currentIndexChanged, this, [this](int profile) {
        if (profile == 0 && (debugMask_ || debugConfirmed_))
            configureDebug(true);
        updateScopeProfile();
    });
    connect(debugLoop_, &QComboBox::currentIndexChanged, this, [this] { updateScopeProfile(); });
    return w;
}
QWidget *Window::pidPage() {
    QVBoxLayout *l;
    auto w = page("环路调参",
                  "Q12 精确量化 · 参数修改仅保存在本地，写入后回读确认。电流环 "
                  "Ki_phys = Ki_config × 10000。",
                  l);
    QStringList names = {"位置环 / POSITION", "速度环 / VELOCITY", "电流环 / CURRENT"};
    for (int loop = 0; loop < 3; loop++) {
        auto group = new QGroupBox(names[loop]);
        auto form = new QFormLayout(group);
        for (int k = 0; k < 3; k++) {
            pid_[loop][k] = number(0, 65535.0 / 4096, 0, 6);
            form->addRow(QStringList{"Kp", "Ki_config", "Kd"}[k], pid_[loop][k]);
            connect(pid_[loop][k], &QDoubleSpinBox::valueChanged, this,
                    [this, loop, k] { pid_[loop][k]->setStyleSheet("border-color:#55b3ff;color:#55b3ff"); });
        }
        auto row = new QHBoxLayout;
        button(row, "读取", [this, loop] { request(0x47 + loop); });
        button(row, "写入并回读", [this, loop] {
            safeAction(0x42 + loop,
                       pid(pid_[loop][0]->value(), pid_[loop][1]->value(), pid_[loop][2]->value()),
                       "写入该环 PID？", [this, loop](QJsonObject) { request(0x47 + loop); });
        });
        form->addRow(row);
        l->addWidget(group);
    }
    auto row = new QHBoxLayout;
    auto stiffness = number(0, 15.9997, 1, 5), damping = number(0, 15.9997, .1, 5);
    row->addWidget(new QLabel("刚度 / 阻尼"));
    row->addWidget(stiffness);
    row->addWidget(damping);
    button(row, "读取", [this, stiffness, damping] {
        request(0x4a, {}, [=](QJsonObject o) {
            stiffness->setValue(o["stiffness"].toDouble());
            damping->setValue(o["damping"].toDouble());
        });
    });
    button(row, "写入", [=, this] {
        safeAction(0x45, pid(stiffness->value(), damping->value(), 0).left(4), "写入刚度和阻尼？");
    });
    l->addLayout(row);
    auto shared = new QHBoxLayout;
    button(shared, "在实时示波器中打开 PID 调试", [this] {
        scopeProfile_->setCurrentIndex(1);
        navigation_->setCurrentRow(0);
    });
    l->addLayout(shared);
    return w;
}
QWidget *Window::configPage() {
    QVBoxLayout *l;
    auto w = page("配置与标定", "写入与 Flash 保存分开；电角、Node ID 和保护阈值操作均受失能门禁保护。", l);
    auto output = new QPlainTextEdit;
    output->setReadOnly(true);
    output->setMaximumHeight(180);
    auto show = [output](QJsonObject o) {
        output->setPlainText(QString::fromUtf8(QJsonDocument(o).toJson(QJsonDocument::Indented)));
    };
    auto row = new QHBoxLayout;
    button(row, "读取电机类型", [this, show] { request(0x52, {}, show); });
    button(row, "读取本体参数", [this, show] { request(0x53, {}, show); });
    button(row, "清除锁存故障", [this, show] { request(0x4f, {}, show); });
    l->addLayout(row);
    l->addWidget(output);
    auto form = new QFormLayout;
    auto id = new QSpinBox;
    id->setRange(1, 255);
    id->setValue(1);
    auto idrow = new QHBoxLayout;
    idrow->addWidget(id);
    button(idrow, "修改 Node ID", [this, id] {
        safeAction(0x4c, be16(id->value()), "修改 Node ID？随后需要保存并重新连接。",
                   [this, id](QJsonObject o) {
                       if (o["node"].toInt() == id->value()) {
                           int i = nodes_->findData(id->value());
                           if (i < 0) {
                               nodes_->addItem(QString("Node %1").arg(id->value()), id->value());
                               i = nodes_->count() - 1;
                           }
                           nodes_->setCurrentIndex(i);
                           request(0x52);
                       }
                   });
    });
    form->addRow("地址", idrow);
    auto method = new QComboBox;
    method->addItems({"Ud", "Uq", "Ud 双点", "Uq 双点", "HFI"});
    auto align = new QHBoxLayout;
    align->addWidget(method);
    button(align, "电角对齐", [this, method, show] {
        if (!unloaded_->isChecked()) {
            log("WARNING", "电角对齐必须确认空载");
            return;
        }
        safeAction(0xab, QByteArray(1, char(method->currentIndex())), "电角对齐会转动电机，确认执行？", show);
    });
    form->addRow("电角对齐", align);
    auto temp = number(-40, 150, 25, 2);
    auto tempRow = new QHBoxLayout;
    tempRow->addWidget(temp);
    button(tempRow, "温度校准", [this, temp, show] {
        safeAction(0xac, be16(int(std::round(temp->value() * 100))), "使用当前参考温度校准？", show);
    });
    form->addRow("参考温度 °C", tempRow);
    l->addLayout(form);
    auto protect = new QGroupBox("保护参数");
    auto pf = new QFormLayout(protect);
    auto otp = number(0, 150, 80, 0), ocp = number(0, 65535, 5000, 0), uv = number(0, 255, 18, 0),
         ov = number(0, 255, 32, 0);
    pf->addRow("过温 °C", otp);
    pf->addRow("过流 mA", ocp);
    pf->addRow("欠压 V", uv);
    pf->addRow("过压 V", ov);
    auto pr = new QHBoxLayout;
    button(pr, "读取", [this, otp, ocp, uv, ov] {
        request(0x4b, {}, [=](QJsonObject o) {
            otp->setValue(o["temperature"].toDouble());
            ocp->setValue(o["current_mA"].toDouble());
            uv->setValue(o["under_voltage"].toDouble());
            ov->setValue(o["over_voltage"].toDouble());
        });
    });
    button(pr, "写入保护参数", [this, otp, ocp, uv, ov] {
        if (uv->value() >= ov->value()) {
            log("ERROR", "欠压阈值必须小于过压阈值");
            return;
        }
        safeAction(0x46,
                   be16(int(otp->value())) + be16(int(ocp->value())) + QByteArray(1, char(uv->value())) +
                       QByteArray(1, char(ov->value())),
                   "确认修改保护阈值？");
    });
    pf->addRow(pr);
    l->addWidget(protect);
    auto actions = new QHBoxLayout;
    button(actions, "保存到 Flash", [this] {
        safeAction(0x4d, {}, "将当前参数写入设备 Flash？", [this](QJsonObject) {
            log("INFO", "SaveParam 成功");
            if (sn_ && !sn_->text().trimmed().isEmpty())
                querySnapshot([this](QJsonObject o) {
                    QString e;
                    archiveId_ =
                        archive_.save(sn_->text(), int(node()), o, connection_.backend == "Demo", &e);
                    savedSnapshot_ = o;
                    log(archiveId_ > 0 ? "INFO" : "ERROR", archiveId_ > 0 ? "SN 参数自动归档成功" : e);
                });
        });
    });
    button(actions, "设置原点", [this] { safeAction(0x50, {}, "将当前位置设置为机械原点？"); });
    button(actions, "恢复三环 PID", [this] { safeAction(0x54, {}, "仅恢复三环 PID 出厂值并写 Flash？"); });
    button(actions, "全部恢复出厂",
           [this] { safeAction(0x4e, {}, "恢复全部参数并写 Flash？现有配置将被覆盖。"); });
    l->addLayout(actions);
    auto kth = new QHBoxLayout;
    button(kth, "KTH71 状态", [this, show] { request(0xae, {}, show); });
    button(kth, "进入 KTH71 校准", [this] {
        if (unloaded_->isChecked() && confirm("KTH71 校准要求电机已使能，确认进入校准会话？"))
            request(0xad);
    });
    button(kth, "触发 ANLC", [this] {
        if (unloaded_->isChecked() && confirm("触发 KTH71 ANLC？"))
            request(0xb0);
    });
    button(kth, "停止 KTH71", [this] { request(0xaf); });
    l->addLayout(kth);
    auto missing = new QLabel("固件未提供：写机型、齿槽补偿、HoldStall、Rs/Ls/磁链辨识入口。\n0x53 "
                              "是本体参数读取；0x54 恢复 PID；0xAE 是 KTH71 状态。不会复用冲突命令。");
    missing->setWordWrap(true);
    missing->setObjectName("warning");
    l->addWidget(missing);
    auto limits = new QHBoxLayout;
    limits->addWidget(new QLabel("软限速、硬超速保护和实时介入状态位于右侧运动控制区。"));
    button(limits, "打开运动控制 / 限速", [this] {
        controlSidebar_->ensureWidgetVisible(motion_);
        motion_->setFocus();
    });
    l->addLayout(limits);
    auto verifyData = std::make_shared<QJsonArray>();
    auto verifySession = std::make_shared<int>(0);
    connect(&session, &Session::replied, w,
            [this, verifyData, verifySession, output](quint64, quint32 n, int c, QJsonObject o) {
                if (n != node() || c != 0xb2 || o["session"].toInt() != *verifySession)
                    return;
                if (verifyData->size() < 1000)
                    verifyData->append(o);
                output->setPlainText(QString("电角验证：采样 %1，seq %2，误差 %3 rad")
                                         .arg(verifyData->size())
                                         .arg(o["sequence"].toInt())
                                         .arg(o["error_rad"].toDouble()));
            });
    auto verification = new QHBoxLayout;
    button(verification, "电角动态验证", [this, verifyData, verifySession, output] {
        if (!unloaded_->isChecked()) {
            log("WARNING", "请先确认空载");
            return;
        }
        *verifySession = (*verifySession + 1) & 65535;
        *verifyData = QJsonArray{};
        safeAction(0xb1, QByteArray(1, 1) + be16(*verifySession), "执行电角动态验证？电机会转动。",
                   [verifyData, verifySession, output](QJsonObject o) {
                       bool complete =
                           o["session"].toInt() == *verifySession && o["count"].toInt() == verifyData->size();
                       int settled = 0;
                       double previous = -1;
                       for (int i = 0; i < verifyData->size(); i++) {
                           auto a = (*verifyData)[i].toObject();
                           complete =
                               complete && a["sequence"].toInt() == i && a["time_s"].toDouble() >= previous;
                           previous = a["time_s"].toDouble();
                           settled += a["settled"].toBool();
                       }
                       complete = complete && settled == 9 && previous >= 7;
                       output->setPlainText(QString(complete && o["ok"].toBool() ? "验证完整并通过\n"
                                                                                 : "验证失败或样本不完整\n") +
                                            QString::fromUtf8(QJsonDocument(o).toJson()));
                   });
    });
    button(verification, "导出电角验证", [this, verifyData] {
        auto path = QFileDialog::getSaveFileName(this, "电角验证数据", {}, "JSON (*.json)");
        QSaveFile file(path);
        if (file.open(QIODevice::WriteOnly)) {
            file.write(QJsonDocument(*verifyData).toJson());
            if (!file.commit())
                log("ERROR", file.errorString());
        }
    });
    l->addLayout(verification);
    return w;
}
QWidget *Window::analysisPage() {
    QVBoxLayout *l;
    auto w = page("数据分析", "使用示波器原始缓冲。T1/T2 区间统计、Hann FFT 与阶跃响应支持离线复核。", l);
    analysisContext_ = new QLabel("正常遥测 · 主机入队目标 / 主机接收反馈时间（端到端，非纯PID响应）");
    analysisContext_->setWordWrap(true);
    l->addWidget(analysisContext_);
    auto returnRow = new QHBoxLayout;
    button(returnRow, "返回共享示波器", [this] { navigation_->setCurrentRow(0); });
    button(returnRow, "扫频 / 带宽测试", [this] {
        if (experimentTimer_.isActive() || testsRunning_ || session.upgrading()) { log("ERROR", "请先停止其他实验或升级任务"); return; }
        showSweepDialog(session, connection_.version, node(), this);
    });
    l->addLayout(returnRow);
    auto c = new QComboBox;
    analysisChannel_ = c;
    c->addItems(channelNames);
    l->addWidget(c);
    statsLabel_ = new QLabel("等待有效采样");
    statsLabel_->setWordWrap(true);
    l->addWidget(statsLabel_);
    auto row = new QHBoxLayout;
    button(row, "计算 T1 / T2 统计", [this, c] {
        auto s = statistics(scope_->samples(), c->currentIndex(),
                            scope_->cursors ? std::min(scope_->t1, scope_->t2) : -1e100,
                            scope_->cursors ? std::max(scope_->t1, scope_->t2) : 1e100);
        statsLabel_->setText(QString("N=%1    Δt=%2 s\n均值 %3    RMS %4\n最小 %5  "
                                     "  最大 %6    峰峰值 %7")
                                 .arg(s.count)
                                 .arg(s.dt, 0, 'g', 8)
                                 .arg(s.mean, 0, 'g', 8)
                                 .arg(s.rms, 0, 'g', 8)
                                 .arg(s.min, 0, 'g', 8)
                                 .arg(s.max, 0, 'g', 8)
                                 .arg(s.max - s.min, 0, 'g', 8));
    });
    button(row, "FFT 峰值", [this, c] {
        auto bins = spectrum(scope_->samples(), c->currentIndex());
        if (bins.empty()) {
            statsLabel_->setText("无等间隔有效数据，无法 FFT");
            return;
        }
        auto peak =
            std::max_element(bins.begin(), bins.end(), [](auto &a, auto &b) { return a.second < b.second; });
        statsLabel_->setText(QString("主频 %1 Hz\n幅值 %2\n频率分辨率 %3 Hz\nFFT 图形位于示波器 → FFT")
                                 .arg(peak->first)
                                 .arg(peak->second)
                                 .arg(bins[1].first));
    });
    l->addLayout(row);
    auto targetValue = number(-32768, 32767, 1);
    auto autoStep = new QCheckBox("从目标曲线检测全部阶跃（位置 / 速度 / Iq）");
    autoStep->setObjectName("autoStepDetection");
    autoStep->setChecked(true);
    l->addWidget(autoStep);
    auto stepSource = new QComboBox;
    stepSource->addItems({"完整记录 · 全部阶跃", "示波器当前窗口 / T1-T2 / 触发捕获"});
    l->addWidget(stepSource);
    auto manualLabel = new QLabel("手动目标值（单位与所选反馈通道一致）");
    l->addWidget(manualLabel);
    manualLabel->hide();
    targetValue->setEnabled(false);
    targetValue->hide();
    connect(autoStep, &QCheckBox::toggled, targetValue, [targetValue, manualLabel](bool automatic) {
        targetValue->setEnabled(!automatic);
        targetValue->setVisible(!automatic);
        manualLabel->setVisible(!automatic);
    });
    l->addWidget(targetValue);
    auto result = new QPlainTextEdit;
    result->setObjectName("stepDetails");
    result->setReadOnly(true);
    result->setMinimumHeight(150);
    auto stepTable = new QTableWidget(0, 14);
    stepTable->setObjectName("stepList");
    stepTable->setHorizontalHeaderLabels(
        {"来源", "阶跃时刻 / s", "原指令", "目标", "实际初值", "首次10–90% / ms", "保持10–90% / ms",
         "首次10%时刻 / s", "首次90%时刻 / s", "10%阈值", "90%阈值", "原始超调 / %", "观察窗 / ms", "状态"});
    stepTable->setSelectionBehavior(QAbstractItemView::SelectRows);
    stepTable->setSelectionMode(QAbstractItemView::SingleSelection);
    stepTable->setEditTriggers(QAbstractItemView::NoEditTriggers);
    stepTable->setMinimumHeight(180);
    auto summary = new QLabel("点击阶跃响应生成列表，再选择某一条查看详情；数值单位与分析通道一致。");
    summary->setObjectName("stepSummary");
    summary->setWordWrap(true);
    auto crossingHold = number(0, 1000, 2, 3);
    crossingHold->setSuffix(" ms");
    auto holdRow = new QHBoxLayout;
    holdRow->addWidget(new QLabel("10% / 90% 交越保持"));
    holdRow->addWidget(crossingHold);
    holdRow->addStretch();
    connect(stepTable, &QTableWidget::itemSelectionChanged, result, [stepTable, result] {
        auto item = stepTable->item(stepTable->currentRow(), 0);
        if (item)
            result->setPlainText(
                QString::fromUtf8(QJsonDocument(item->data(Qt::UserRole).toJsonObject()).toJson()));
    });
    auto actions = new QHBoxLayout;
    actions->addLayout(holdRow);
    auto analyzeStep = new QPushButton("阶跃响应");
    actions->addWidget(analyzeStep);
    connect(
        analyzeStep, &QPushButton::clicked, this,
        [this, c, targetValue, result, autoStep, stepSource, analyzeStep, stepTable, summary, crossingHold] {
            auto data = stepSource->currentIndex() == 0 ? scope_->recordedSamples(node()) : scope_->samples();
            analyzeStep->setEnabled(false);
            stepTable->setRowCount(0);
            summary->setText("正在分析记录快照…");
            result->setPlainText("正在按目标变化分段计算…");
            auto watcher = new QFutureWatcher<QJsonArray>(result);
            auto history = &activeHistory();
            auto generation = history->info().generation;
            QString context = scope_->aliases.value(c->currentIndex()) + " · " + stepSource->currentText();
            connect(
                watcher, &QFutureWatcher<QJsonArray>::finished, result,
                [this, history, generation, watcher, result, analyzeStep, stepTable, summary, context] {
                    if (&activeHistory() != history || history->info().generation != generation) {
                        analyzeStep->setEnabled(true);
                        watcher->deleteLater();
                        return;
                    }
                    auto rows = watcher->result();
                    const QStringList fields = {
                        "source",         "step_time_s",   "previous_target",   "target",
                        "initial",        "first_rise_ms", "qualified_rise_ms", "first10_time_s",
                        "first90_time_s", "level10",       "level90",           "raw_overshoot_percent",
                        "window_ms",      "status"};
                    stepTable->setRowCount(rows.size());
                    int measured = 0;
                    for (int r = 0; r < rows.size(); ++r) {
                        auto row = rows[r].toObject();
                        if (row.contains("first_rise_ms") || row.contains("rise_seconds"))
                            ++measured;
                        for (int col = 0; col < fields.size(); ++col) {
                            auto value = row[fields[col]];
                            QString text = value.isDouble()   ? QString::number(value.toDouble(), 'g', 9)
                                           : value.isString() ? value.toString()
                                                              : "—";
                            if (fields[col] == "status" && row.contains("settling"))
                                text += " · 未稳定";
                            auto item = new QTableWidgetItem(text);
                            item->setToolTip(text);
                            if (col == 0)
                                item->setData(Qt::UserRole, row);
                            stepTable->setItem(r, col, item);
                        }
                    }
                    stepTable->resizeColumnsToContents();
                    stepTable->setColumnWidth(0, 150);
                    stepTable->setColumnWidth(13, 260);
                    summary->setText(
                        context +
                        QString(
                            " · 识别 %1 条，%2 条可测上升时间。每条在下一次目标变化前结束；点击行查看详情。")
                            .arg(rows.size())
                            .arg(measured));
                    if (!rows.empty())
                        stepTable->selectRow(0);
                    else
                        result->setPlainText(
                            "未发现目标指令阶跃。请检查通道和记录范围；实际反馈的跳变不一定是目标指令变化。没"
                            "有目标记录时可取消自动检测，输入真实目标分析。PID自动分析要求变化前后给定各保持"
                            "至少5帧；连"
                            "续变化请用手动目标和T1/T2区间。");
                    analyzeStep->setEnabled(true);
                    watcher->deleteLater();
                });
            auto targetEvents = scope_->targetEvents();
            watcher->setFuture(
                QtConcurrent::run([targetEvents, data = std::move(data), channel = c->currentIndex(),
                                   goal = targetValue->value(), automatic = autoStep->isChecked(),
                                   hold = crossingHold->value() / 1000] {
                    if (automatic) {
                        if (!data.empty() && !data.front().debugLoop && !targetEvents.empty())
                            return hostStepResponses(data, targetEvents, channel, hold);
                        return stepResponses(data, channel, hold, !data.empty() && data.front().debugLoop);
                    }
                    auto row = stepResponse(data, channel, goal, false);
                    return QJsonArray{row};
                }));
        });
    button(actions, "导出分析", [this, result, stepTable, summary] {
        auto f = QFileDialog::getSaveFileName(this, "导出分析", {}, "Text (*.txt)");
        if (!f.isEmpty()) {
            QSaveFile file(f);
            if (file.open(QIODevice::WriteOnly)) {
                QJsonArray rows;
                for (int r = 0; r < stepTable->rowCount(); ++r)
                    rows.append(stepTable->item(r, 0)->data(Qt::UserRole).toJsonObject());
                file.write((summary->text() + "\n选中阶跃详情\n" + result->toPlainText() + "\n全部阶跃\n" +
                            QString::fromUtf8(QJsonDocument(rows).toJson()))
                               .toUtf8());
                if (!file.commit())
                    log("ERROR", file.errorString());
            }
        }
    });
    l->addLayout(actions);
    l->addWidget(summary);
    auto stepSplit = new QSplitter(Qt::Vertical);
    stepSplit->addWidget(stepTable);
    stepSplit->addWidget(result);
    stepSplit->setStretchFactor(0, 1);
    stepSplit->setStretchFactor(1, 1);
    l->addWidget(stepSplit, 1);
    return w;
}
QWidget *Window::experimentPage() {
    QVBoxLayout *l;
    auto w = page("激励与温升",
                  "运动实验需要明确空载或夹具确认；超时、故障、遥测丢失、温度超限"
                  "均结束实验并失能。",
                  l);
    auto form = new QFormLayout;
    auto position = number(-250, 250, 0), velocity = number(-32768, 32767, 0, 0),
         current = number(-32, 32, 0);
    form->addRow("位置 rad", position);
    form->addRow("速度 RPM", velocity);
    form->addRow("电流 A", current);
    auto targets = new QHBoxLayout;
    for (int i = 0; i < 3; i++) {
        auto input = QList<QDoubleSpinBox *>{position, velocity, current}[i];
        button(targets, QStringList{"发送位置", "发送速度", "发送电流"}[i], [this, input, i] {
            if (!unloaded_->isChecked()) {
                log("WARNING", "需要空载确认");
                return;
            }
            try {
                request(i + 1, target(quint8(node()), input->value(), i, connection_.version));
            } catch (std::exception &e) {
                log("ERROR", QString::fromUtf8(e.what()));
            }
        });
    }
    form->addRow(targets);
    auto combined = new QHBoxLayout;
    auto commands = new QComboBox;
    commands->addItems(
        {"位置 + 速度 · 0x12", "位置 + 电流 · 0x13", "速度 + 电流 · 0x23", "位置 + 速度 + 电流 · 0x30"});
    combined->addWidget(commands);
    auto nodeIds = new QLineEdit;
    nodeIds->setPlaceholderText("批量 Node，如 1,2,3；留空当前节点");
    combined->addWidget(nodeIds);
    button(combined, "组合给定", [this, commands, nodeIds, position, velocity, current] {
        if (!unloaded_->isChecked()) {
            log("WARNING", "需要空载确认");
            return;
        }
        auto ids = nodeIds->text().trimmed().isEmpty() ? QStringList{QString::number(node())}
                                                       : nodeIds->text().split(',');
        if (ids.size() > 64) {
            log("ERROR", "最多 64 个 Node");
            return;
        }
        int command = QList<int>{0x12, 0x13, 0x23, 0x30}[commands->currentIndex()];
        try {
            QList<QPair<quint32, QByteArray>> prepared;
            for (auto idText : ids) {
                bool ok;
                int n = idText.trimmed().toInt(&ok);
                if (!ok || n < 1 || n > 255)
                    throw std::invalid_argument("目标 Node 必须为 1..255");
                prepared.append(
                    {quint32(n), combinedTarget(quint8(n), command, position->value(), velocity->value(),
                                                current->value(), connection_.version)});
            }
            for (auto item : prepared)
                session.command(item.first, command, item.second);
        } catch (std::exception &e) {
            log("ERROR", QString::fromUtf8(e.what()));
        }
    });
    form->addRow(combined);
    l->addLayout(form);
    auto type = new QComboBox;
    type->addItems({"电流阶跃", "主机电流扫频 0.5 → 4.5 Hz", "堵转温升"});
    auto amplitude = number(-2, 2, .08), duration = number(.1, 3600, 2, 2), limit = number(20, 100, 60, 0);
    auto fixture = new QCheckBox("堵转夹具已固定，确认电流和散热条件");
    auto f = new QFormLayout;
    f->addRow("实验类型", type);
    f->addRow("电流幅值 A", amplitude);
    f->addRow("时长 s", duration);
    f->addRow("温度上限 °C", limit);
    f->addRow(fixture);
    l->addLayout(f);
    auto row = new QHBoxLayout;
    button(row, "开始实验", [this, type, amplitude, duration, limit, fixture] {
        auto s = session.latest(node());
        if (!s || !s->enabled || s->mode != (connection_.version == Version::V2 ? 6 : 2)) {
            log("ERROR", "实验要求已使能的 CST 模式，请先完成模式设置和使能");
            return;
        }
        if ((type->currentIndex() == 2 && !fixture->isChecked()) ||
            (type->currentIndex() != 2 && !unloaded_->isChecked())) {
            log("ERROR", "实验确认条件未满足");
            return;
        }
        if (type->currentIndex() == 2 && !s->environment) {
            log("ERROR", "温升实验需要有效温度遥测");
            return;
        }
        if (!confirm("启动电流实验？电机可能运动或发热。"))
            return;
        experimentKind_ = type->currentIndex();
        experimentAmplitude_ = amplitude->value();
        experimentDuration_ = duration->value();
        temperatureLimit_ = limit->value();
        experimentStart_ = uptime_.elapsed() / 1000.0;
        experimentTimer_.start(20);
        activeHistory().clear();
        session.sampling.pause(false);
    });
    button(row, "停止并失能", [this] {
        stopExperiment();
        session.emergency();
    });
    l->addLayout(row);
    auto info = new QLabel("主机扫频用于低频系统检查；Windows "
                           "定时器不适用于电流环带宽辨识。\n固件高速扫频入口未确"
                           "认，保持禁用。实验完成后可在"
                           "数据分析页计算阶跃指标。");
    info->setWordWrap(true);
    l->addWidget(info);
    l->addStretch();
    return w;
}
void Window::stopExperiment() {
    if (experimentTimer_.isActive()) {
        experimentTimer_.stop();
        session.emergency();
        log("INFO", "实验结束，已请求急停和给定清零");
    }
}
QWidget *Window::torquePage() {
    QVBoxLayout *l;
    auto w = page("Iq–力矩标定",
                  "导入力矩传感器 "
                  "CSV（iq_A,torque_Nm）或手工录入同步测量点，最小二乘拟合 Kt。",
                  l);
    auto table = new QTableWidget(0, 2);
    table->setHorizontalHeaderLabels({"Iq / A", "Torque / Nm"});
    table->horizontalHeader()->setSectionResizeMode(QHeaderView::Stretch);
    l->addWidget(table);
    auto result = new QLabel("传感器：离线 / CSV 输入");
    l->addWidget(result);
    auto row = new QHBoxLayout;
    button(row, "添加测量点", [this, table] {
        int r = table->rowCount();
        table->insertRow(r);
        auto s = session.latest(node());
        table->setItem(r, 0, new QTableWidgetItem(s ? QString::number(s->values[0]) : "0"));
        table->setItem(r, 1, new QTableWidgetItem("0"));
    });
    button(row, "导入 CSV", [this, table] {
        auto path = QFileDialog::getOpenFileName(this, "力矩 CSV", {}, "CSV (*.csv)");
        QFile f(path);
        if (!f.open(QIODevice::ReadOnly))
            return;
        table->setRowCount(0);
        while (!f.atEnd()) {
            auto a = QString::fromUtf8(f.readLine()).trimmed().split(',');
            if (a.size() < 2)
                continue;
            bool x, y;
            a[0].toDouble(&x);
            a[1].toDouble(&y);
            if (!x || !y)
                continue;
            int r = table->rowCount();
            table->insertRow(r);
            table->setItem(r, 0, new QTableWidgetItem(a[0]));
            table->setItem(r, 1, new QTableWidgetItem(a[1]));
        }
    });
    button(row, "拟合 Kt", [table, result] {
        double sx = 0, sy = 0, sxx = 0, sxy = 0;
        int n = table->rowCount();
        for (int r = 0; r < n; r++) {
            if (!table->item(r, 0) || !table->item(r, 1))
                return;
            bool a, b;
            double x = table->item(r, 0)->text().toDouble(&a), y = table->item(r, 1)->text().toDouble(&b);
            if (!a || !b || !std::isfinite(x) || !std::isfinite(y)) {
                result->setText("测量点必须为有限数字");
                return;
            }
            sx += x;
            sy += y;
            sxx += x * x;
            sxy += x * y;
        }
        double den = n * sxx - sx * sx;
        if (n < 2 || std::abs(den) < 1e-12) {
            result->setText("至少需要两个不同电流的测量点");
            return;
        }
        double k = (n * sxy - sx * sy) / den;
        result->setText(QString("Kt = %1 Nm/A    偏置 = %2 Nm    N=%3")
                            .arg(k, 0, 'g', 8)
                            .arg((sy - k * sx) / n, 0, 'g', 8)
                            .arg(n));
    });
    button(row, "导出", [this, table, result] {
        auto path = QFileDialog::getSaveFileName(this, "力矩 CSV", {}, "CSV (*.csv)");
        QSaveFile f(path);
        if (!f.open(QIODevice::WriteOnly))
            return;
        f.write("iq_A,torque_Nm\n");
        for (int r = 0; r < table->rowCount(); r++)
            if (table->item(r, 0) && table->item(r, 1))
                f.write((table->item(r, 0)->text() + "," + table->item(r, 1)->text() + "\n").toUtf8());
        if (!f.commit())
            log("ERROR", f.errorString());
    });
    l->addLayout(row);
    return w;
}
QWidget *Window::iapPage() {
    QVBoxLayout *l;
    auto w = page("固件升级",
                  "ASIX .efw 整包 · 校验头、长度、Checksum32 · ENTER → START → "
                  "DATA → END → VERIFY → RESET",
                  l);
    auto file = new QLineEdit;
    file->setReadOnly(true);
    l->addWidget(file);
    auto metadata = new QLabel("未选择固件");
    metadata->setWordWrap(true);
    l->addWidget(metadata);
    auto bytes = std::make_shared<QByteArray>();
    auto row = new QHBoxLayout;
    button(row, "选择 .efw", [this, file, metadata, bytes] {
        QString path = QFileDialog::getOpenFileName(this, "选择固件", {}, "Firmware (*.efw)");
        QFile f(path);
        if (!f.open(QIODevice::ReadOnly))
            return;
        try {
            *bytes = firmwareImage(f.readAll());
            file->setText(path);
            metadata->setText(QString("ASIX / ImageType 3 / %1 B / checksum32 通过").arg(bytes->size()));
        } catch (std::exception &e) {
            bytes->clear();
            metadata->setText(QString::fromUtf8(e.what()));
        }
    });
    button(row, "查询设备版本", [this, metadata] {
        request(0xfd, {}, [metadata](QJsonObject o) { metadata->setText(o["version"].toString()); });
    });
    button(row, "升级当前 Node", [this, bytes] {
        if (bytes->isEmpty())
            return;
        if (confirm(QString("向 Node %1 写入固件？请保持供电和链路稳定。").arg(node()))) {
            try {
                session.upgrade(node(), *bytes);
            } catch (std::exception &e) {
                log("ERROR", QString::fromUtf8(e.what()));
            }
        }
    });
    button(row, "取消", [this] { session.emergency(); });
    l->addLayout(row);
    iapBar_ = new QProgressBar;
    l->addWidget(iapBar_);
    iapLabel_ = new QLabel("空闲");
    iapLabel_->setWordWrap(true);
    l->addWidget(iapLabel_);
    auto warning = new QLabel("升级时暂停示波并独占命令链路；不会在超时后自动重发 Flash "
                              "数据。\n默认单播升级，避免广播导致多台设备同时刷写。旧状态格"
                              "式会明确拒绝，必须先核对 Bootloader。");
    warning->setWordWrap(true);
    warning->setObjectName("warning");
    l->addWidget(warning);
    l->addStretch();
    return w;
}
QWidget *Window::logPage() {
    QVBoxLayout *l;
    auto w = page("通信日志", "有界日志 · TX / RX 原始帧 · 操作结果 · 专用应答与分发 ACK 分离", l);
    auto filter = new QLineEdit;
    filter->setPlaceholderText("输入 ERROR / WARNING / Node 等过滤");
    l->addWidget(filter);
    log_ = new QPlainTextEdit;
    log_->setReadOnly(true);
    log_->setMaximumBlockCount(3000);
    l->addWidget(log_);
    connect(filter, &QLineEdit::textChanged, this, [this](QString s) {
        log_->setProperty("filter", s);
        QStringList found;
        for (auto &line : logs_)
            if (line.contains(s, Qt::CaseInsensitive))
                found.append(line);
        log_->setPlainText(found.join('\n'));
    });
    auto row = new QHBoxLayout;
    button(row, "清空", [this] {
        logs_.clear();
        log_->clear();
    });
    button(row, "导出日志", [this] {
        auto path = QFileDialog::getSaveFileName(this, "导出日志", {}, "Log (*.log)");
        QSaveFile f(path);
        if (f.open(QIODevice::WriteOnly)) {
            f.write(logs_.join('\n').toUtf8());
            if (!f.commit())
                log("ERROR", f.errorString());
        }
    });
    button(row, "关于", [this] {
        QMessageBox::about(this, "MotorTune",
                           QString("MotorTune %1\nQt %2 · C++20\n构建 %3 %4\nV2.0.0 / V1.0.0 · "
                                   "SQLite schema 1")
                               .arg(MOTORTUNE_VERSION, QT_VERSION_STR, __DATE__, __TIME__));
    });
    l->addLayout(row);
    return w;
}
} // namespace mt
