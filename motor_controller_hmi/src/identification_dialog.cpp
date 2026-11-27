#include "identification_dialog.h"
#include <QtWidgets>
namespace mt {
static bool sameResult(const QJsonObject &a,const QJsonObject &b) {
    for(const auto &key:{"valid_mask","rs","ld","lq","flux"})
        if(!a.contains(key) || a[key]!=b[key])return false;
    return true;
}
IdentificationDialog::IdentificationDialog(Session &s,Version v,quint32 n,bool demo,QWidget *parent,int mode)
    :QDialog(parent),session_(s),version_(v),node_(n),demo_(demo) {
    setObjectName("identificationDialog");
    setWindowTitle(QString("电机参数辨识 · Node %1%2").arg(n).arg(demo?" · DEMO 模拟":""));
    resize(820,760);
    auto layout=new QVBoxLayout(this);
    auto intro=new QLabel("Rs / Ld / Lq / 永磁磁链\n先完成电角对齐，再在空载条件下辨识。目标和跳闸速度均为电机轴 rad/s。");
    intro->setWordWrap(true);layout->addWidget(intro);
    if(demo) { auto label=new QLabel("DEMO · 所有结果均为模拟值，不代表真实电机参数。");label->setObjectName("warning");layout->addWidget(label); }
    config_=new QGroupBox("辨识配置");auto grid=new QGridLayout(config_);
    mode_=new QComboBox;mode_->setObjectName("identMode");
    mode_->addItem("完整流程 · Rs → Ld/Lq → 磁链",3);mode_->addItem("仅 Rs / Ld / Lq",1);mode_->addItem("仅磁链 · 使用已应用的 Rs/Ld/Lq",2);
    mode_->setCurrentIndex(mode_->findData(mode));grid->addWidget(new QLabel("模式"),0,0);grid->addWidget(mode_,0,1,1,3);
    const QStringList labels={"测试电流 A","电压上限 V","注入峰值 V","注入频率 Hz","目标轴速 rad/s","电流跳闸 A","轴速跳闸 rad/s"};
    const QStringList names={"identCurrent","identVoltage","identInjection","identFrequency","identSpeed","identCurrentTrip","identSpeedTrip"};
    const double initial[]={.3,3,.2,200,20,1,30};
    const double low[]={.02,.05,.01,20,2,.03,2.5};
    const double high[]={43.69,65.535,65.535,1000,200,65.535,300};
    for(int i=0;i<7;++i) {
        auto spin=new QDoubleSpinBox;values_[i]=spin;spin->setObjectName(names[i]);
        spin->setDecimals(i==3?0:i==4||i==6?2:3);spin->setRange(low[i],high[i]);spin->setValue(initial[i]);
        spin->setSingleStep(i==3?10:i==4||i==6?1:.05);
        int row=1+i/2,col=2*(i%2);grid->addWidget(new QLabel(labels[i]),row,col);grid->addWidget(spin,row,col+1);
        connect(spin,&QDoubleSpinBox::valueChanged,this,[this]{refresh();});
    }
    layout->addWidget(config_);
    safe_=new QCheckBox("已确认空载、允许转轴，测试电流及跳闸值符合电机额定能力");
    safe_->setObjectName("identSafe");layout->addWidget(safe_);connect(safe_,&QCheckBox::toggled,this,[this]{refresh();});
    auto controls=new QHBoxLayout;
    auto button=[&](const QString &text,const char *name,auto slot) {
        auto b=new QPushButton(text);b->setObjectName(name);controls->addWidget(b);connect(b,&QPushButton::clicked,this,slot);return b;
    };
    start_=button("启动辨识","identStart",&IdentificationDialog::start);
    stop_=button("停止辨识","identStop",&IdentificationDialog::stop);
    queryButton_=button("重新查询","identQuery",[this]{
        polling_=true;
        if(!alignment_)alignment_=session_.command(node_,0x53,{},-1,"读取辨识前电角对齐状态");
        query();
    });
    layout->addLayout(controls);
    phase_=new QLabel("等待设备辨识状态");phase_->setObjectName("identPhase");layout->addWidget(phase_);
    progress_=new QProgressBar;progress_->setObjectName("identProgress");progress_->setRange(0,6);progress_->setValue(0);layout->addWidget(progress_);
    results_=new QTableWidget(5,3);results_->setObjectName("identResults");
    results_->setHorizontalHeaderLabels({"参数","结果","单位 / 含义"});
    results_->horizontalHeader()->setSectionResizeMode(QHeaderView::Stretch);results_->verticalHeader()->hide();
    results_->verticalHeader()->setSectionResizeMode(QHeaderView::Stretch);
    results_->setEditTriggers(QAbstractItemView::NoEditTriggers);results_->setMinimumHeight(210);
    const QStringList keys={"Rs","Ld","Lq","Ls（参考）","永磁磁链 ψf"};
    const QStringList units={"Ω","mH","mH","mH · (Ld+Lq)/2，低凸极电机参考","Wb"};
    for(int i=0;i<5;++i){results_->setItem(i,0,new QTableWidgetItem(keys[i]));results_->setItem(i,1,new QTableWidgetItem("—"));results_->setItem(i,2,new QTableWidgetItem(units[i]));}
    layout->addWidget(results_);
    auto actions=new QHBoxLayout;
    apply_=new QPushButton("应用到 RAM");apply_->setObjectName("identApply");actions->addWidget(apply_);
    save_=new QPushButton("保存到 Flash");save_->setObjectName("identSave");actions->addWidget(save_);
    connect(apply_,&QPushButton::clicked,this,&IdentificationDialog::apply);connect(save_,&QPushButton::clicked,this,&IdentificationDialog::save);
    auto close=new QPushButton("关闭");actions->addWidget(close);connect(close,&QPushButton::clicked,this,&QDialog::reject);layout->addLayout(actions);
    storage_=new QLabel("尚未应用；辨识不会自动覆盖配置或保存 Flash。");storage_->setObjectName("identStorage");storage_->setWordWrap(true);layout->addWidget(storage_);
    message_=new QLabel;message_->setObjectName("identMessage");message_->setWordWrap(true);layout->addWidget(message_);
    connect(&s,&Session::replied,this,&IdentificationDialog::replied);
    connect(&s,&Session::commandFailed,this,&IdentificationDialog::failed);
    connect(&s,&Session::connectionChanged,this,[this](bool connected,const QString &){
        known_=aligned_=eligible_=applied_=false;query_=operation_=alignment_=0;polling_=connected;
        storage_->setText("连接发生变化，原应用/保存状态已失效。");
        message(connected?"连接已恢复，请重新查询并确认电角对齐。":"连接已断开，无法确认电机已停止；结果暂不可应用或保存。");
        if(connected)alignment_=session_.command(node_,0x53);refresh();
    });
    auto emergency=new QShortcut(QKeySequence("Ctrl+Shift+E"),this);
    connect(emergency,&QShortcut::activated,this,[this]{eligible_=applied_=false;session_.emergency();message("已发送急停请求，等待设备状态确认。");});
    connect(&timer_,&QTimer::timeout,this,[this]{refresh();if(polling_&&!operation_&&!query_)query();});
    timer_.start(100);
    message("读取设备能力与电角对齐状态；旧固件无专用应答时不会允许启动。");
    if(v!=Version::V2)message("参数辨识仅支持 V2 协议，请关闭窗口并切换协议后重新连接。");
    else if(!n||n>=0x7ff)message("请选择有效的单播节点；参数辨识不接受广播或 IAP 地址。");
    if(session_.connected()&&v==Version::V2&&n)alignment_=session_.command(n,0x53);
    query();refresh();
}
IdentificationConfig IdentificationDialog::config() const {
    return {values_[0]->value(),values_[1]->value(),values_[2]->value(),values_[3]->value(),
            values_[4]->value(),values_[5]->value(),values_[6]->value()};
}
bool IdentificationDialog::fresh() const {return known_&&lastStatus_.isValid()&&lastStatus_.elapsed()<750;}
bool IdentificationDialog::idleTelemetry() const {
    auto s=session_.latest(node_);return session_.connected()&&!session_.upgrading()&&session_.telemetryFresh(node_)&&s&&!s->enabled&&!s->error;
}
void IdentificationDialog::message(const QString &text) {message_->setText(text);}
void IdentificationDialog::refresh() {
    bool link=session_.connected()&&!session_.upgrading()&&version_==Version::V2&&node_>0&&node_<0x7ff;
    bool active=known_&&identificationActive(status_["stage"].toInt());
    bool writable=link&&idleTelemetry()&&session_.selectedNode()==node_&&fresh()&&!active&&!operation_;
    auto error=validateIdentification(config());
    start_->setEnabled(writable&&aligned_&&safe_->isChecked()&&error.isEmpty()&&!mayRun_);
    start_->setToolTip(!error.isEmpty()?error:!aligned_?"先关闭此窗口，完成电角对齐后重新打开":"使用当前配置启动离线辨识");
    stop_->setEnabled(link);queryButton_->setEnabled(link&&!operation_&&!query_);
    config_->setEnabled(!mayRun_&&!active&&!operation_);
    apply_->setEnabled(writable&&eligible_&&status_["stage"].toInt()==7&&status_["valid_mask"].toInt()!=0);
    save_->setEnabled(writable&&applied_&&sameResult(status_,appliedResult_));
    if(known_&&!fresh())phase_->setText("状态已过期 · 等待设备确认，不能据此应用或保存");
}
void IdentificationDialog::query() {
    if(!session_.connected()||session_.upgrading()||version_!=Version::V2||!node_||query_||operation_)return;
    query_=session_.command(node_,identStatus,QByteArray(1,1),-1,"查询参数辨识进度");
}
void IdentificationDialog::start() {
    refresh();if(!start_->isEnabled())return;
    QByteArray p;
    try{p=identificationStart(mode_->currentData().toInt(),config());}
    catch(const std::exception &e){message(QString::fromUtf8(e.what()));return;}
    query_=0;known_=eligible_=applied_=false;mayRun_=true;runMode_=mode_->currentData().toInt();
    for(int i=0;i<5;++i)results_->item(i,1)->setText("—");
    storage_->setText("本轮结果尚未产生；原设备参数保持不变。");
    operationCommand_=identControl;operationAction_=1;
    operation_=session_.command(node_,identControl,p,-1,"启动电机参数辨识");
    message("启动请求已提交，等待设备专用应答。");refresh();
}
void IdentificationDialog::stop() {
    if(!session_.connected())return;
    query_=0;eligible_=applied_=false;operationCommand_=identControl;operationAction_=0;
    operation_=session_.command(node_,identControl,QByteArray::fromHex("0100"),-1,"停止电机参数辨识");
    polling_=true;message("停止请求已提交，等待设备确认；尚未确认停机。");refresh();
}
void IdentificationDialog::apply() {
    refresh();if(!apply_->isEnabled())return;
    query_=0;operationCommand_=identApply;operationAction_=2;actionResult_=status_;
    operation_=session_.command(node_,identApply,QByteArray(1,1),-1,"应用辨识结果到 RAM");
    applied_=false;message("等待设备确认应用结果。");refresh();
}
void IdentificationDialog::save() {
    refresh();if(!save_->isEnabled())return;
    query_=0;operationCommand_=0x4d;operationAction_=3;
    operation_=session_.command(node_,0x4d,{},-1,"保存已应用的辨识参数到 Flash");
    message("正在保存设备当前全部配置到 Flash，等待专用成功应答。");refresh();
}
void IdentificationDialog::acceptStatus(const QJsonObject &o) {
    int old=status_["stage"].toInt(-1),stage=o["stage"].toInt(),mask=o["valid_mask"].toInt();
    if(applied_&&!sameResult(o,appliedResult_)){applied_=false;storage_->setText("设备结果已变化，需要重新应用。");}
    status_=o;known_=true;lastStatus_.restart();
    mayRun_=identificationActive(stage);
    if(stage==0||stage==8||stage==9)eligible_=false;
    if(stage==7&&mask!=(runMode_==3?3:runMode_==1?1:2))eligible_=false;
    phase_->setText(QString("%1 · %2 s%3").arg(identificationStage(stage)).arg(o["elapsed_ms"].toDouble()/1000,0,'f',2)
        .arg(stage==8?" · "+identificationError(o["ident_error"].toInt()):""));
    int total=runMode_==1?4:runMode_==2?2:6;
    progress_->setRange(0,total);progress_->setValue(stage==7?total:stage>=1&&stage<=6?qBound(0,stage-(runMode_==2?5:1),total):0);
    progress_->setFormat("%v / %m 阶段完成");
    const double vals[]={o["rs"].toDouble(),o["ld"].toDouble()*1000,o["lq"].toDouble()*1000,
        (o["ld"].toDouble()+o["lq"].toDouble())*500,o["flux"].toDouble()};
    for(int i=0;i<5;++i)results_->item(i,1)->setText(stage==7&&(mask&(i==4?2:1))?QString::number(vals[i],'g',6):"—");
    if(stage==7&&old!=7)message(eligible_?"辨识完成。检查结果后可应用到 RAM，再保存到 Flash。":"设备留有辨识结果，仅供查看；本窗口未确认本轮启动，不能应用旧结果。");
    if(stage==8)message("辨识失败："+identificationError(o["ident_error"].toInt())+"。原参数未被覆盖。");
    if(stage==9)message("设备已确认取消辨识；原参数未被覆盖。");
}
void IdentificationDialog::replied(quint64 token,quint32 n,int cmd,const QJsonObject &o) {
    if(n!=node_||!token)return;
    if(token==alignment_&&cmd==0x53){alignment_=0;aligned_=o["valid"].toBool()&&o["aligned"].toBool();
        if(!aligned_)message("设备未确认电角对齐。请关闭此窗口，先完成电角对齐。");refresh();return;}
    if(token==query_&&cmd==identStatus){query_=0;if(o["valid"].toBool()&&o["ok"].toBool())acceptStatus(o);else {known_=false;message(o["message"].toString());}refresh();return;}
    if(token!=operation_||cmd!=operationCommand_)return;
    operation_=0;
    if(!o["valid"].toBool()||!o["ok"].toBool()) {
        if(operationAction_==1)eligible_=false;
        message("设备拒绝操作："+o["message"].toString());
        if(cmd==identControl&&o["valid"].toBool()){known_=false;mayRun_=identificationActive(o["stage"].toInt());}
        refresh();return;
    }
    if(cmd==identControl){
        eligible_=operationAction_==1;acceptStatus(o);
        if(operationAction_==1&&mayRun_)message("设备已启动辨识，请等待各阶段完成。");
    }else if(cmd==identApply){
        acceptStatus(o);applied_=o["stage"].toInt()==7&&eligible_&&sameResult(o,actionResult_);
        if(applied_){appliedResult_=o;storage_->setText("已应用到 RAM · 尚未保存到 Flash");message("设备已确认应用；正常运行 PID 不会自动重整定。");emit parametersApplied(o);}
        else message("应答与待应用结果不一致，不能确认应用；请重新辨识。");
    }else if(cmd==0x4d){storage_->setText(demo_?"DEMO · 模拟保存成功":"已保存到 Flash · 设备返回成功");message("保存完成。掉电保持需在实机上重启验证。");emit parametersSaved(appliedResult_);}
    polling_=true;refresh();
}
void IdentificationDialog::failed(quint64 token,const QString &why) {
    if(token!=query_&&token!=operation_&&token!=alignment_)return;
    if(token==alignment_){alignment_=0;aligned_=false;}
    if(token==query_){query_=0;known_=false;polling_=false;}
    if(token==operation_){operation_=0;applied_=false;known_=false;eligible_=false;polling_=false;}
    if(!known_)phase_->setText("设备状态未知 · 等待重新查询确认");
    message(why+"\n请检查固件是否支持参数辨识及通信状态；可重新查询或请求停止，不能把超时视为成功。");refresh();
}
void IdentificationDialog::done(int result) {
    timer_.stop();
    if(mayRun_||(known_&&identificationActive(status_["stage"].toInt()))) {
        session_.command(node_,identControl,QByteArray::fromHex("0100"),-1,"关闭窗口：请求停止辨识");
        session_.command(node_,0x40,QByteArray(1,0),-1,"关闭辨识窗口：请求失能");
    }
    QDialog::done(result);
}
}
