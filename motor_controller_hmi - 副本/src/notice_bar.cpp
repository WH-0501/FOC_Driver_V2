#include "notice_bar.h"
namespace mt {
NoticeBar::NoticeBar(QWidget *parent) : QFrame(parent) {
    setObjectName("operationNotice");
    auto row = new QHBoxLayout(this);
    row->setContentsMargins(16, 8, 16, 8);
    message_ = new QLabel;
    message_->setObjectName("operationNoticeText");
    message_->setTextFormat(Qt::PlainText);
    message_->setWordWrap(true);
    message_->setTextInteractionFlags(Qt::TextSelectableByMouse);
    row->addWidget(message_, 1);
    auto details = new QPushButton("详细信息");
    details->setObjectName("noticeDetails");
    row->addWidget(details);
    auto dismiss = new QPushButton("关闭提示");
    dismiss->setObjectName("dismissNotice");
    row->addWidget(dismiss);
    connect(dismiss, &QPushButton::clicked, this, &QWidget::hide);
    connect(details, &QPushButton::clicked, this, [this] {
        auto dialog = new QDialog(this);
        dialog->setAttribute(Qt::WA_DeleteOnClose);
        dialog->setWindowTitle("操作提示详情");
        dialog->resize(700, 320);
        auto layout = new QVBoxLayout(dialog);
        auto text = new QPlainTextEdit(fullText_);
        text->setReadOnly(true);
        layout->addWidget(text);
        auto close = new QPushButton("关闭");
        layout->addWidget(close);
        connect(close, &QPushButton::clicked, dialog, &QDialog::close);
        dialog->show(); // No modal loop: motor controls and emergency shortcut remain reachable.
    });
    hide();
}
void NoticeBar::post(const QString &level, const QString &message, bool newOperation) {
    if (message.isEmpty())
        return;
    bool important = level == "ERROR" || level == "WARNING";
    // Normal progress belongs on the initiating control and in the log. Never
    // expand the workspace for successful operations or connection progress.
    if (!important) {
        if (newOperation && level == "INFO")
            hide();
        return;
    }
    if (!newOperation && !isHidden() && level_ == level && fullText_ == message)
        return;
    level_ = level;
    fullText_ = message;
    setProperty("level", level);
    QString color = level == "ERROR" ? "#ff6363" : "#ffbc33";
    QString title = level == "ERROR" ? "操作失败 / 未确认" : "请注意";
    QString summary = message.section('\n', 0, 2).left(400);
    if (summary != message)
        summary += "…（可展开详情）";
    message_->setText(title + " · " + summary);
    message_->setToolTip(message);
    setStyleSheet(QString("QFrame#operationNotice{background:#17191c;border:1px solid "
                          "%1;}QLabel#operationNoticeText{color:%1;background:transparent;}")
                      .arg(color));
    show();
}
} // namespace mt
