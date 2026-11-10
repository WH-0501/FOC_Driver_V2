#include "motion_control.h"
#include <cmath>

namespace mt {
MotionControl::MotionControl(Session &session, QWidget *parent)
    : QGroupBox("运动控制", parent), session_(session) {
    setObjectName("motionControl");
    setStyleSheet("QGroupBox#motionControl{padding:8px;margin-top:10px;}"
                  "QGroupBox#motionControl QPushButton{padding:5px 8px;}"
                  "QGroupBox#motionControl QDoubleSpinBox,QGroupBox#motionControl QSpinBox{padding:4px;}"
                  "QGroupBox#motionControl QDoubleSpinBox:disabled{color:#6a6b6c;}");
    auto layout = new QVBoxLayout(this);
    layout->setSpacing(6);
    auto caption = new QLabel("目标给定 · 编辑后点击发送");
    caption->setObjectName("muted");
    layout->addWidget(caption);
    auto targetGrid = new QGridLayout;
    targetGrid->setHorizontalSpacing(6);
    targetGrid->setColumnStretch(1, 1);
    const QStringList names = {"目标位置", "目标速度", "目标电流 Iq"};
    const QStringList units = {" rad", " RPM", " A"};
    for (int i = 0; i < 3; ++i) {
        targetGrid->addWidget(new QLabel(names[i]), i, 0);
        targets_[i] = new QDoubleSpinBox;
        targets_[i]->setObjectName(QString("motionTarget%1").arg(i));
        targets_[i]->setDecimals(i == 0 ? 4 : i == 1 ? 0 : 3);
        targets_[i]->setSuffix(units[i]);
        targets_[i]->setKeyboardTracking(false);
        targets_[i]->setSingleStep(i == 1 ? 1 : .01);
        targets_[i]->setMinimumWidth(0);
        targets_[i]->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Fixed);
        targetGrid->addWidget(targets_[i], i, 1);
        send_[i] = new QPushButton("发送");
        send_[i]->setObjectName(QString("sendTarget%1").arg(i));
        targetGrid->addWidget(send_[i], i, 2);
        connect(send_[i], &QPushButton::clicked, this, [this, i] { sendTarget(i); });
    }
    layout->addLayout(targetGrid);
    auto stateRow = new QVBoxLayout;
    motionState_ = new QLabel;
    motionState_->setWordWrap(true);
    motionState_->setObjectName("motionState");
    motionResult_ = new QLabel("尚未发送目标");
    motionResult_->setWordWrap(true);
    stateRow->addWidget(motionState_);
    stateRow->addWidget(motionResult_);
    layout->addLayout(stateRow);
    auto actions = new QHBoxLayout;
    combined_ = new QPushButton("发送组合目标");
    combined_->setObjectName("sendCombinedTarget");
    connect(combined_, &QPushButton::clicked, this, [this] { sendTarget(3); });
    actions->addWidget(combined_);
    zero_ = new QPushButton("电流归零");
    zero_->setObjectName("zeroCurrent");
    connect(zero_, &QPushButton::clicked, this, [this] { sendTarget(2, true); });
    actions->addWidget(zero_);
    layout->addLayout(actions);

    auto divider = new QFrame;
    divider->setFrameShape(QFrame::HLine);
    layout->addWidget(divider);
    auto limitsTitle = new QLabel("限速与保护 · 上次回读");
    limitsTitle->setObjectName("muted");
    layout->addWidget(limitsTitle);
    auto statusRow = new QVBoxLayout;
    softState_ = new QLabel;
    hardState_ = new QLabel;
    activeState_ = new QLabel;
    softState_->setObjectName("softLimitState");
    hardState_->setObjectName("hardLimitState");
    activeState_->setObjectName("limitingState");
    for (auto label : {softState_, hardState_, activeState_}) {
        label->setWordWrap(true);
        statusRow->addWidget(label);
    }
    layout->addLayout(statusRow);
    auto limitRow = new QHBoxLayout;
    limitRow->addWidget(new QLabel("软限速 / 输出轴"));
    soft_ = new QSpinBox;
    soft_->setObjectName("softLimitRpm");
    soft_->setRange(0, 65535);
    soft_->setSuffix(" RPM");
    soft_->setSpecialValueText("0 · 关闭软限速");
    soft_->setKeyboardTracking(false);
    limitRow->addWidget(soft_, 1);
    layout->addLayout(limitRow);
    hard_ = new QCheckBox("硬超速保护（待发送）");
    hard_->setObjectName("hardProtectionDraft");
    layout->addWidget(hard_);
    read_ = new QPushButton("读取限速");
    read_->setObjectName("readSpeedLimits");
    apply_ = new QPushButton("发送限速");
    apply_->setObjectName("applySpeedLimits");
    auto limitActions = new QHBoxLayout;
    limitActions->addWidget(read_);
    limitActions->addWidget(apply_);
    layout->addLayout(limitActions);
    feedback_ = new QLabel;
    feedback_->setObjectName("limitFeedback");
    feedback_->setWordWrap(true);
    layout->addWidget(feedback_);
    limitResult_ = new QLabel;
    limitResult_->setWordWrap(true);
    layout->addWidget(limitResult_);
    auto details = new QToolButton;
    details->setText("限速说明 · RAM 配置，重启恢复默认");
    details->setCheckable(true);
    details->setToolButtonStyle(Qt::ToolButtonTextBesideIcon);
    details->setArrowType(Qt::RightArrow);
    details->setStyleSheet("QToolButton{border:0;color:#9c9c9d;text-align:left;padding:2px;}");
    layout->addWidget(details);
    auto hint = new QLabel("软限速仅作用于电流/扭矩控制，90% 起削减加速电流，保留制动电流。0 "
                           "仅关闭软限速；硬保护独立，超速锁存故障并关断。配置仅存 RAM，重启恢复默认。");
    hint->setObjectName("muted");
    hint->setWordWrap(true);
    hint->hide();
    layout->addWidget(hint);
    connect(details, &QToolButton::toggled, this, [details, hint](bool expanded) {
        details->setArrowType(expanded ? Qt::DownArrow : Qt::RightArrow);
        hint->setVisible(expanded);
    });
    auto changed = [this] {
        dirty_ = true;
        ++revision_;
        limitResult_->setText("限速配置已编辑 · 尚未发送（上方为设备回读状态）");
        updateState();
    };
    connect(soft_, &QSpinBox::valueChanged, this, changed);
    connect(hard_, &QCheckBox::toggled, this, changed);
    connect(read_, &QPushButton::clicked, this, &MotionControl::readLimits);
    connect(apply_, &QPushButton::clicked, this, &MotionControl::writeLimits);
    connect(&session_, &Session::replied, this, &MotionControl::reply);
    connect(&session_, &Session::commandFailed, this, &MotionControl::failed);
    connect(&session_, &Session::connectionChanged, this, [this](bool on, const QString &) {
        if (connected_ == on)
            return;
        connected_ = on;
        reset();
        initialReadPending_ = on;
    });
    reset();
}
void MotionControl::reset() {
    readToken_ = writeToken_ = targetToken_ = 0;
    limits_ = {};
    limitsReadAt_.clear();
    dirty_ = false;
    revision_ = 0;
    for (int i = 0; i < 3; ++i) {
        const double scale = i == 0   ? 128 * (version_ == Version::V1 ? 180 / std::numbers::pi : 1)
                             : i == 1 ? 1
                                      : 1000;
        // Spin-box endpoints are rounded inward so every editable value fits int16.
        const double precision = std::pow(10, targets_[i]->decimals());
        targets_[i]->setRange(std::ceil(-32768 / scale * precision) / precision,
                              std::floor(32767 / scale * precision) / precision);
        targets_[i]->setValue(0);
    }
    QSignalBlocker a(soft_), b(hard_);
    soft_->setValue(0);
    hard_->setChecked(true);
    motionResult_->setText("尚未发送目标");
    limitResult_->setText("等待设备回读；编辑值不代表设备当前配置");
    updateState();
}
void MotionControl::setContext(quint32 node, Version version, int selectedMode, bool workspaceSafe,
                               bool busy) {
    const bool changed = node_ != node || version_ != version || connected_ != session_.connected();
    const bool connectionChanged = connected_ != session_.connected();
    node_ = node;
    version_ = version;
    connected_ = session_.connected();
    mode_ = selectedMode;
    safe_ = workspaceSafe;
    busy_ = busy;
    if (changed)
        reset();
    if (connectionChanged)
        initialReadPending_ = connected_;
    updateState();
    // Exactly one initial read per connection. Context/page/mode changes never
    // arm a new read; a failure is retried only by an explicit button click.
    if (initialReadPending_ && read_->isEnabled())
        readLimits();
}
void MotionControl::updateState() {
    auto sample = session_.latest(node_);
    bool fresh = session_.telemetryFresh(node_) && !session_.upgrading();
    QString state;
    if (!connected_)
        state = "未连接 · 可先编辑目标";
    else if (!fresh)
        state = "遥测过期或升级占用 · 禁止发送";
    else if (!node_ || node_ > 255)
        state = "目标给定仅支持 Node 1–255";
    else if (sample->mode != mode_)
        state = "所选模式未生效 · 请失能后应用模式";
    else if (sample->error)
        state = "设备故障 · 禁止发送";
    else if (!sample->enabled)
        state = "设备已失能 · 使能后可发送目标";
    else if (!safe_)
        state = "请先确认空载 / 工作区安全";
    else if (busy_)
        state = "自动测试 / 激励占用 · 手动给定暂停";
    else
        state = "已使能 · 可发送目标";
    auto names = modes(version_);
    QString actual = fresh ? names.value(sample->mode, "未知模式") : "待确认";
    motionState_->setText(QString("设备：%1 · %2").arg(actual, state));
    bool canSend = fresh && node_ > 0 && node_ <= 255 && sample->mode == mode_ && sample->enabled &&
                   !sample->error && safe_ && !busy_ && !targetToken_;
    for (int i = 0; i < 3; ++i) {
        bool accepts = modeAcceptsTarget(version_, mode_, i);
        targets_[i]->setEnabled(accepts);
        send_[i]->setEnabled(accepts && canSend);
        targets_[i]->setToolTip(accepts ? "本地目标值；点击发送才下发" : "所选模式不使用此目标");
    }
    zero_->setEnabled(canSend && modeAcceptsTarget(version_, mode_, 2));
    bool combined = mode_ == (version_ == Version::V2 ? 7 : 5) || (version_ == Version::V1 && mode_ == 2);
    combined_->setVisible(combined);
    combined_->setEnabled(canSend && combined);
    // Reading is safe without fresh motion telemetry; writing still requires it.
    read_->setEnabled(connected_ && node_ > 0 && !session_.upgrading() && !busy_ && !writeToken_);
    bool confirmed = connected_ && !limits_.isEmpty();
    hard_->setEnabled(confirmed && limits_["format"].toInt() >= 3);
    apply_->setEnabled(confirmed && fresh && !sample->error && !busy_ && !writeToken_ &&
                       soft_->value() <= limits_["hard_rpm"].toInt());
    if (!confirmed) {
        for (auto label : {softState_, hardState_, activeState_})
            label->setStyleSheet("color:#9c9c9d");
        softState_->setText("软限速：待确认");
        hardState_->setText("硬超速保护：待确认");
        activeState_->setText("限速介入：待查询");
        feedback_->setText("设备回读：输出轴 — RPM · 用户 Iq — A · 限制后 Iq — A");
        return;
    }
    int rpm = limits_["soft_rpm"].toInt();
    bool hard = limits_["hard_enabled"].toBool(), active = limits_["limiting"].toBool();
    softState_->setText(rpm ? QString("软限速：已开启 · %1 RPM").arg(rpm) : "软限速：已关闭");
    softState_->setStyleSheet(rpm ? "color:#5fc992" : "color:#ffbc33");
    hardState_->setText(QString("硬保护：%1 · 阈值 %2 RPM%3")
                            .arg(hard ? "已开启" : "已关闭")
                            .arg(limits_["hard_rpm"].toInt())
                            .arg(limits_["format"].toInt() == 2 ? "（固定开启）" : ""));
    hardState_->setStyleSheet(hard ? "color:#5fc992" : "color:#ff6363");
    activeState_->setText(active ? "回读时：软限速正在介入" : "回读时：软限速未介入");
    activeState_->setStyleSheet(active ? "color:#ffbc33;font-weight:600" : "color:#9c9c9d");
    QString applied = limits_["mode"].toInt() == 1
                          ? QString::number(limits_["iq_limited"].toDouble(), 'f', 3) + " A"
                          : "—（当前非电流控制）";
    feedback_->setText(QString("回读快照 %4：输出轴 %1 RPM · 用户 Iq %2 A → 限制后 Iq %3")
                           .arg(limits_["speed_rpm"].toDouble(), 0, 'f', 1)
                           .arg(limits_["iq_reference"].toDouble(), 0, 'f', 3)
                           .arg(applied)
                           .arg(limitsReadAt_));
}
void MotionControl::readLimits() {
    updateState();
    if (!read_->isEnabled() || readToken_ || writeToken_)
        return;
    const bool automatic = initialReadPending_;
    initialReadPending_ = false;
    readToken_ = session_.command(node_, 0xb5, {}, -1,
                                  automatic ? "自动操作：连接后首次读取限速" : "界面操作：读取限速");
    updateState();
}
void MotionControl::writeLimits() {
    updateState();
    if (!apply_->isEnabled())
        return;
    QByteArray payload = be16(soft_->value());
    if (limits_["format"].toInt() >= 3)
        payload += be16(hard_->isChecked() ? 1 : 0);
    submittedRevision_ = revision_;
    writeToken_ =
        session_.command(node_, 0xb6, payload, -1,
                         QString("界面操作：发送限速 · 输出轴软限速=%1 RPM · 硬保护=%2")
                             .arg(soft_->value())
                             .arg(limits_["format"].toInt() >= 3 ? (hard_->isChecked() ? "开启" : "关闭")
                                                                 : "本协议不支持修改"));
    limitResult_->setText("限速已发送 · 等待设备应答确认");
    updateState();
}
void MotionControl::sendTarget(int channel, bool zero) {
    updateState();
    if (channel < 3 ? !send_[channel]->isEnabled() : !combined_->isEnabled())
        return;
    try {
        if (zero)
            targets_[2]->setValue(0);
        int cmd = channel + 1;
        QByteArray payload;
        if (channel < 3) {
            payload = target(quint8(node_), targets_[channel]->value(), channel, version_);
            targetDescription_ =
                QStringList{"位置 ", "速度 ", "电流 Iq "}[channel] + targets_[channel]->text();
        } else {
            cmd = version_ == Version::V1 && mode_ == 2 ? 0x23 : 0x30;
            payload = combinedTarget(quint8(node_), cmd, targets_[0]->value(), targets_[1]->value(),
                                     targets_[2]->value(), version_);
            targetDescription_ = QString("组合目标：位置 %1，速度 %2，电流 %3")
                                     .arg(targets_[0]->text(), targets_[1]->text(), targets_[2]->text());
        }
        targetToken_ = session_.command(node_, cmd, payload, mode_,
                                        "界面操作：" + (zero ? QString("电流归零 · ") : QString("下发目标")) +
                                            targetDescription_);
        motionResult_->setText("等待发送：" + targetDescription_);
    } catch (const std::exception &e) {
        motionResult_->setText(QString::fromUtf8(e.what()));
    }
    updateState();
}
void MotionControl::acceptLimits(const QJsonObject &o, bool written) {
    limits_ = o;
    limitsReadAt_ = QTime::currentTime().toString("HH:mm:ss");
    if (written && submittedRevision_ == revision_)
        dirty_ = false;
    if (!dirty_) {
        QSignalBlocker a(soft_), b(hard_);
        soft_->setValue(o["soft_rpm"].toInt());
        hard_->setChecked(o["hard_enabled"].toBool());
    }
    soft_->setToolTip(QString("设备硬阈值 %1 RPM；0 仅关闭软限速").arg(o["hard_rpm"].toInt()));
    if (written)
        limitResult_->setText(dirty_ ? "设备已应答；仍有新的本地修改未发送" : "设备已确认限速配置（RAM）");
    else if (!dirty_ && !writeToken_)
        limitResult_->setText("已读取 · 后续仅点击查询，显示上次回读快照");
}
void MotionControl::reply(quint64 token, quint32 node, int, const QJsonObject &o) {
    if (!token)
        return; // Unsolicited telemetry/parameter replies are not our transactions.
    if (node != node_ || (token != readToken_ && token != writeToken_ && token != targetToken_))
        return;
    if (o.contains("ok") && !o["ok"].toBool()) {
        failed(token, o["message"].toString());
        return;
    }
    if (token == targetToken_) {
        targetToken_ = 0;
        motionResult_->setText("已发送 " + targetDescription_ + " · 实际响应见波形/遥测");
    } else if (token == writeToken_) {
        writeToken_ = 0;
        acceptLimits(o, true);
    } else {
        readToken_ = 0;
        if (!writeToken_)
            acceptLimits(o, false);
    }
    updateState();
}
void MotionControl::failed(quint64 token, const QString &reason) {
    if (token == targetToken_) {
        targetToken_ = 0;
        motionResult_->setText("目标未发送：" + reason);
    } else if (token == readToken_ || token == writeToken_) {
        if (token == readToken_)
            readToken_ = 0;
        else {
            writeToken_ = 0;
            limits_ = {};
            limitsReadAt_.clear();
        }
        limitResult_->setText("本次限速操作未确认：" + reason + "；请点击读取，不自动重试");
    }
    updateState();
}
} // namespace mt
