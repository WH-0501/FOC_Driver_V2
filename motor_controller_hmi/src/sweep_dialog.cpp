#include "sweep_dialog.h"
#include "sweep.h"
#include "scope.h"
#include "sweep_recording.h"
#include "chirp_analysis_dialog.h"
#include "chirp_analysis.h"
#include <QtWidgets>
#include <cmath>
#include <map>
#include <numbers>
namespace mt {
namespace {
class BodePlot : public QWidget {
public:
  std::vector<SweepPoint> points, comparison;
  QString comparisonName;
  BodePlot() { setMinimumHeight(300); }
  void paintEvent(QPaintEvent *) override {
    QPainter p(this);
    p.fillRect(rect(), QColor("#0b0e10"));
    p.setRenderHint(QPainter::Antialiasing);
    if (points.empty()) {
      p.setPen(Qt::gray);
      p.drawText(rect(), Qt::AlignCenter, "扫频后显示幅频 / 相频曲线");
      return;
    }
    double lo = std::log10(points.front().hz),
           hi = std::log10(points.back().hz);
    if (hi <= lo)
      hi = lo + 1;
    double minGain = -6, maxGain = 3;
    for (auto &v : points)
      if (v.valid) {
        minGain = std::min(minGain, v.gainDb - 2);
        maxGain = std::max(maxGain, v.gainDb + 2);
      }
    for(const auto &v:comparison)if(v.valid){minGain=std::min(minGain,v.gainDb-2);maxGain=std::max(maxGain,v.gainDb+2);}
    if(!comparison.empty()){p.setPen(QColor("#55b3ff"));p.drawText(65,16,"原始 H1");p.setPen(QColor("#ed91bc"));p.drawText(170,16,comparisonName);}
    for (int panel = 0; panel < 3; ++panel) {
      QRectF r(65, 38 + panel * ((height()-18) / 3.), width() - 110,
               (height()-18) / 3. - 48);
      double low = panel==2?0:panel ? -180 : minGain, high = panel==2?1:panel ? 180 : maxGain;
      auto pos = [&](double hz, double v) {
        return QPointF(r.left() + (std::log10(hz) - lo) / (hi - lo) * r.width(),
                       r.bottom() - (v - low) / (high - low) * r.height());
      };
      for (int j = 0; j <= 4; ++j) {
        double v = low + (high - low) * j / 4;
        double y = pos(points.front().hz, v).y();
        p.setPen(QColor("#252829"));
        p.drawLine(QPointF(r.left(), y), QPointF(r.right(), y));
        p.setPen(Qt::gray);
        p.drawText(QRectF(0, y - 9, 60, 18), Qt::AlignRight,
                   QString::number(v, 'f', 1));
      }
      p.setPen(Qt::lightGray);
      p.drawText(QPointF(r.left(), r.top() - 5),
                 panel==2 ? "相干性 γ²" : panel ? "相位 / °（主值）" : "增益 / dB");
      for (int j = 0; j <= 4; ++j) {
        double hz = std::pow(10., lo + (hi - lo) * j / 4);
        double x = pos(hz, 0).x();
        p.drawText(QRectF(x - 35, r.bottom() + 3, 70, 18), Qt::AlignCenter,
                   QString::number(hz, 'g', 3) + " Hz");
      }
      if(!panel && points.size()>1 && points[0].valid && points[1].valid) {
        double threshold=(points[0].gainDb+points[1].gainDb)/2-3;
        double y=pos(points.front().hz,threshold).y();
        p.setPen(QPen(QColor("#ffbc33"),1,Qt::DashLine));p.drawLine(QPointF(r.left(),y),QPointF(r.right(),y));
        p.drawText(QPointF(r.left()+5,y-4),"原始基准 −3 dB");
      }
      QPainterPath path;
      bool pen = false;
      for (auto &v : points) {
        if (!v.valid && panel!=2) {
          pen = false;
          continue;
        }
        auto pt = pos(v.hz, panel==2 ? v.inputFit : panel ? v.phaseDeg : v.gainDb);
        if (pen)
          path.lineTo(pt);
        else
          path.moveTo(pt);
        pen = true;
        p.setPen(QColor(panel ? "#5fc992" : "#55b3ff"));
        p.drawEllipse(pt, 3, 3);
      }
      p.setPen(QPen(QColor(panel ? "#5fc992" : "#55b3ff"), 1.5));
      p.drawPath(path);
      QPainterPath overlay;bool join=false;double lastPhase=0;
      for(const auto &v:comparison){double y=panel==2?v.inputFit:panel?v.phaseDeg:v.gainDb;
        if(!std::isfinite(y)||(!v.valid&&panel!=2)){join=false;continue;}
        auto q=pos(v.hz,y);if(join&&!(panel==1&&std::abs(y-lastPhase)>180))overlay.lineTo(q);else overlay.moveTo(q);
        join=true;lastPhase=y;p.setPen(QColor("#ed91bc"));p.drawEllipse(q,2,2);}
      p.setPen(QPen(QColor("#ed91bc"),1.4,Qt::DashLine));p.drawPath(overlay);
    }
  }
};
class SweepDialog : public QDialog {
  QComboBox *methodChoice;
  bool deviceMode=false,deviceData=false, awaitingPlan=false;
  double planFs=0;
  quint64 deviceToken=0;
  quint16 runId=0;
  int nextPoint=0,deviceCount=0;
  Session &session;
  Version version;
  quint32 node;
  QDoubleSpinBox *startHz, *endHz, *amplitude;
  QSpinBox *count;
  QLabel *status, *bandwidth;
  QTableWidget *table;
  BodePlot *plot;
  Scope *wave;
  QComboBox *filterChoice;
  QDoubleSpinBox *filterCutoff;
  std::vector<double> comparisonWave;
  QLabel *meanNote;
  std::vector<SweepPoint> meanResults;
  QString meanError;
  bool meanActive() const {return deviceData&&filterChoice->currentIndex()!=0&&recordMethod==3;}
  bool lpfSelected() const {return filterChoice->currentIndex()==2;}
  ChirpAnalysisOptions filterOptions() const {ChirpAnalysisOptions o;o.filter=lpfSelected()?ChirpFilter::FeedbackLpf:ChirpFilter::FeedbackMean16;o.cutoffHz=filterCutoff->value();return o;}
  QString comparisonName() const {return lpfSelected()?"LPF 对照（含滤波器）":"16 点均值对照（含滤波器）";}
  void updateMeanResults() {
    meanResults.clear();meanError.clear();comparisonWave.clear();
    if(!meanActive())return;
    SweepCapture c{recording,recordMethod,recordFs,recordExpected,deviceCompleted,source,results.empty()?requestedPoints:results};
    auto o=filterOptions();
    auto a=analyzeChirp(c,o);meanResults=std::move(a.filtered);meanError=a.error;comparisonWave=std::move(a.filteredRecording);
  }
  QLabel *recordStatus;
  QComboBox *resultChoice;
  SweepRecording recording;
  std::vector<SweepPoint> recordedResults;
  double recordFs=0;
  quint32 recordExpected=0;
  int recordMethod=0;
  bool recordSupported=false, resultsReady=false, deviceCompleted=false, hostH1=false;
  std::vector<SweepPoint> requestedPoints;
  qint64 drainDeadline=0, lastWaveDraw=0;
  void refreshRecording(bool draw=false) {
    auto snapshot=session.sweepRecording();
    if(snapshot.node==node && snapshot.run==runId) recording=std::move(snapshot);
    recordStatus->setText(recording.summary(recordExpected,recordSupported,deviceCompleted));
    if(draw || (running && clock.elapsed()-lastWaveDraw>=300)) {
      drawRecording();lastWaveDraw=clock.elapsed();
    }
  }
  void drawRecording(bool preserveView=false) {
    std::vector<Sample> data;
    const auto smooth=meanActive()?(lpfSelected()?comparisonWave:feedbackMean16(recording)):std::vector<double>{};
    wave->aliases[11]=smooth.empty()?"Iq 反馈（原始）/ A":lpfSelected()?"Iq 反馈（LPF 对照）/ A":"Iq 反馈（16 点均值对照）/ A";
    if(recordFs>0) {
      data.reserve(recording.samples.size());
      quint32 next=0;
      size_t i=0;for(const auto &v:recording.samples) {
        if(v.index!=next) {Sample gap;gap.time=(next+.5)/recordFs;gap.values.fill(std::numeric_limits<double>::quiet_NaN());data.push_back(gap);}
        Sample point;point.time=v.index/recordFs;point.node=node;
        point.values.fill(std::numeric_limits<double>::quiet_NaN());
        point.values[0]=v.input;point.values[11]=smooth.empty()?v.output:smooth[i];data.push_back(point);next=v.index+1;++i;
      }
    }
    wave->setSamples(std::move(data));
    if(!preserveView && (!running || resultsReady))wave->fitHistory();
  }
  void completeDevice() {
    refreshRecording(true);
    QString analysisError;
    if(hostH1) {
      SweepCapture c{recording,recordMethod,recordFs,recordExpected,deviceCompleted,source,requestedPoints};
      if(recordMethod==2) {
        results=deviceCompleted?recordingH1(recording,2,recordFs,requestedPoints,recordExpected):std::vector<SweepPoint>{};
        recordedResults.clear();
        if(results.empty()){analysisError="逐频录波不完整或频点窗口不匹配，不计算带宽";results=requestedPoints;}
        else source+="\n上位机原始 H1（未滤波） · 每点舍弃整周期稳定段，4 段 × 4 周期同步估计";
        resultChoice->setItemText(0,"上位机原始 H1");
      } else {
      auto a=analyzeChirp(c,{});
      results=a.raw;recordedResults.clear();analysisError=a.error;
      if(a.ok()) source+=QString("\n上位机原始 H1（未滤波） · 输入周期最大差 %1 A · 反馈非周期残差 RMS %2 A")
        .arg(a.inputRepeatMax,0,'g',5).arg(a.outputResidualRms,0,'g',5);
      else {results=requestedPoints;source+="\n"+analysisError;}
      resultChoice->setItemText(0,"上位机原始 H1");
      }
    } else {
      recordedResults=deviceCompleted?recordingH1(recording,recordMethod,recordFs,results,recordExpected):std::vector<SweepPoint>{};
      resultChoice->setItemText(0,"历史设备 H1");
    }
    updateMeanResults();drawRecording();
    running=false;timer.stop();deviceToken=0;
    start->setEnabled(true);import->setEnabled(true);methodChoice->setEnabled(true);
    startHz->setEnabled(true);endHz->setEnabled(true);amplitude->setEnabled(true);count->setEnabled(true);
    resultChoice->setEnabled(!recordedResults.empty());
    status->setText(analysisError.isEmpty()?QString("设备辨识完成并失能 · 含上传、分析与显示 %1 ms").arg(clock.elapsed()):"采集/分析未完成："+analysisError);render();
  }
  QPushButton *start, *import;
  QCheckBox *safe;
  QTimer timer;
  QElapsedTimer clock;
  std::vector<SweepPoint> results;
  std::vector<std::vector<SweepSample>> raw;
  bool running = false;
  QString source = "尚无数据";
  QString deviceAbortDetail;
  QJsonObject spikeSnapshot;
  unsigned spikePages=0;
  QString spikeSummary() const {
    if(spikePages!=7)return "尖峰现场：未取得完整诊断快照（旧固件或测量中止时不可用）";
    if(!spikeSnapshot["spike_present"].toBool())return "尖峰现场：本次未捕获 |Iq| > 0.2 A（仅为诊断触发值）";
    auto v=[&](const char *k){return spikeSnapshot[k].toDouble();};
    return QString("尖峰现场：样本 %1，Iq=%2 A，Id=%3 A；ADC=[%4,%5,%6]，前一拍=[%7,%8,%9]，零偏=[%10,%11,%12]；PWM 预装载 U/V/W=[%13,%14,%15]，ADC 读取时 CNT=%16（%17计数）；电角度 %18→%19 rad，编码器计数 %20→%21。CCR 并非有效锁存值；CNT 并非硬件采样时刻。")
      .arg(v("spike_sample")).arg(v("spike_iq"),0,'g',9).arg(v("spike_id"),0,'g',9)
      .arg(v("spike_adc0")).arg(v("spike_adc1")).arg(v("spike_adc2"))
      .arg(v("spike_previous_adc0")).arg(v("spike_previous_adc1")).arg(v("spike_previous_adc2"))
      .arg(v("spike_offset0")).arg(v("spike_offset1")).arg(v("spike_offset2"))
      .arg(v("spike_ccr0")).arg(v("spike_ccr1")).arg(v("spike_ccr2"))
      .arg(v("spike_counter")).arg((int(v("spike_timer_ctrl"))&16)?"向下":"向上")
      .arg(v("spike_previous_theta"),0,'g',9).arg(v("spike_theta"),0,'g',9)
      .arg(v("spike_previous_shaft")).arg(v("spike_shaft"));
  }
  QByteArray deviceRequest(int action) {
    QByteArray p; p.append(char(1));p.append(char(action));p+=be16(runId);
    p+=be16(int(startHz->value()));p+=be16(int(endHz->value()));p.append(char(count->value()));p+=be16(qRound(amplitude->value()*1000));
    if(action) {p.append(char(0xa9));p.append(char(1));}
    if(action==2){p.append(char(0xad));p.append(char(1));p+=be16(qRound(planFs));
      for(const auto &v:requestedPoints)p+=be16(qRound(planFs/v.hz));}
    return p;
  }
  void beginDevice() {
    auto s=session.latest(node);
    if(version!=Version::V2 || !safe->isChecked() || !session.connected() || !session.telemetryFresh(node) || !s || !s->enabled || s->mode!=6) {
      status->setText("设备辨识要求 V2 固件、CST 已使能、零给定及工作区确认");return;
    }
    awaitingPlan=methodChoice->currentData().toInt()==2;
    requestedPoints=awaitingPlan?std::vector<SweepPoint>{}:chirpFrequencyGrid(startHz->value(),endHz->value(),unsigned(count->value()));
    if(!awaitingPlan && requestedPoints.empty()){status->setText("频段或频点数无效：请减少频点数量或扩大频段");return;}
    runId=quint16(QRandomGenerator::global()->bounded(1,65536));
    results.clear();meanResults.clear();meanError.clear();comparisonWave.clear();raw.clear();deviceAbortDetail.clear();deviceMode=deviceData=true;running=true;nextPoint=0;deviceCount=0;
    spikeSnapshot={};spikePages=0;
    session.beginSweepRecording(node,runId);recording.reset(node,runId);
    recordFs=0;recordExpected=0;recordMethod=methodChoice->currentData().toInt();
    recordSupported=resultsReady=deviceCompleted=hostH1=false;lastWaveDraw=0;recordedResults.clear();
    resultChoice->setCurrentIndex(0);resultChoice->setEnabled(false);wave->clearDisplay();
    wave->follow=true;
    clock.start();start->setEnabled(false);import->setEnabled(false);methodChoice->setEnabled(false);
    startHz->setEnabled(false);endHz->setEnabled(false);count->setEnabled(false);amplitude->setEnabled(false);
    source="设备同步采样 · 闭环给定→实际 Iq · H1 频响 / 多段相干性 · 未收到结果";
    if(methodChoice->currentData().toInt()==3) source+=" · 周期线性 Chirp，4 s 采集";
    deviceToken=awaitingPlan?session.command(node,0xbd,{},-1,"查询逐频录波能力和采样率"):
      session.command(node,0xbc,deviceRequest(methodChoice->currentData().toInt()),-1,"启动设备电流环辨识（一次配置，无续租）");
    timer.start(100);status->setText("等待固件确认（不支持的固件将超时，不回退到主机发波）");render();
  }
  void deviceReply(quint64 token,quint32 n,int cmd,const QJsonObject &o) {
    if(!running || !deviceMode || token!=deviceToken || n!=node)return;
    deviceToken=0;
    if(!o["ok"].toBool()){finish(o["message"].toString("设备辨识应答无效"));return;}
    if(awaitingPlan) {
      if(cmd!=0xbd || !o["host_h1"].toBool() || !o["host_stepped"].toBool() || o["chirp_periods"].toInt()!=4) {
        finish("固件不支持上位机逐频分析，请升级配套固件");return;
      }
      planFs=o["fs"].toDouble();
      requestedPoints=steppedFrequencyGrid(startHz->value(),endHz->value(),unsigned(count->value()),planFs);
      if(requestedPoints.empty()){finish("逐频频段可用整周期频点不足或参数无效，请减少点数或扩大频段");return;}
      awaitingPlan=false;
      deviceToken=session.command(node,0xbc,deviceRequest(2),-1,"启动逐频正弦同步录波（AD/1 周期计划）");return;
    }
    if(o["run"].toInt()!=runId){finish("辨识会话编号不匹配");return;}
    int state=o["state"].toInt();
    if(cmd!=0xbe && o["method"].toInt()!=methodChoice->currentData().toInt()){finish("固件返回的激励模式不匹配");return;}

    if(cmd!=0xbe){
      recordFs=o["fs"].toDouble();recordExpected=quint32(o["samples"].toDouble());
      recordSupported=recordSupported || o["recording"].toBool();
      if(o["host_h1"].toBool()) {
        if(o["chirp_periods"].toInt()!=4){finish("不支持的测量分段配置");return;}
        hostH1=true;
      }
      if(recordMethod==2 && state!=3 && !o["flags"].toInt() &&
         (!hostH1 || !o["host_stepped"].toBool() || !recordSupported || recordFs!=planFs)) {
        finish("固件逐频能力或采样率与计划不匹配");return;
      }
      refreshRecording();
      deviceCount=o["count"].toInt();
      if(deviceCount<3 || deviceCount>16 || (hostH1 && deviceCount!=int(requestedPoints.size()))){finish("固件返回频点数量无效");return;}
      source=QString("设备采样 %1 Hz · 采集 %2 ms · 最长 FOC 回调 %3 μs · %4 · 同步给定/实际 Iq，H1 频响").arg(o["fs"].toDouble()).arg(o["samples"].toDouble()/std::max(1.,o["fs"].toDouble())*1000,0,'f',1).arg(o["max_us"].toInt()).arg(methodChoice->currentData().toInt()==1?"快速多正弦":methodChoice->currentData().toInt()==3?"周期线性 Chirp（1 s 稳定 + 3 × 1 s 估计）":"逐频正弦");
      if(o.contains("record_dropped")) source+=QString(" · 上传状态快照：设备缓冲丢样 %1 点，已提交 CAN %2 点（非主机收妥）").arg(o["record_dropped"].toDouble(),0,'f',0).arg(o["record_submitted"].toDouble(),0,'f',0);
      status->setText(QString("设备进度 %1/%2 · 主机经过 %3 ms").arg(o["progress"].toInt()).arg(deviceCount).arg(clock.elapsed()));
      if(o.contains("timing_revision"))source+=QString(" · 诊断 A6/%1 · 分段峰值：采样/编码器 %2 μs，控制 %3 μs（含激励 %4 μs）").arg(o["timing_revision"].toInt()).arg(o["sense_us"].toInt()).arg(o["control_us"].toInt()).arg(o["fra_us"].toInt());
      if(o.contains("peak_cycles")) {
        if(o["base_period_us"].toInt()>0)
          source+=QString(" · 普通 CST 耗时 %1 μs，间隔 %2 μs").arg(o["base_us"].toInt()).arg(o["base_period_us"].toInt());
        else source+=QString(" · 普通 CST 间隔未采得");
        source+=QString(" · 峰值回调 DWT %1 周期（按 180 MHz 折算 %2 μs）").arg(o["peak_cycles"].toDouble(),0,'f',0).arg(o["peak_cycles"].toDouble()/180.,0,'f',1);
      }
      if(o.contains("encoder_us"))source+=QString(" · 编码器 %1 μs，双电流 PID %2 μs，调制/PWM %3 μs（均为父段内耗时，不另加）").arg(o["encoder_us"].toInt()).arg(o["pid_us"].toInt()).arg(o["pwm_us"].toInt());
      if(o.contains("timing_reason") && o["interval_max_us"].toInt()==0)source+=QString(" · 周期间隔：尚无相邻样本");
      else if(o.contains("timing_reason"))source+=QString(" · 周期间隔 %1–%2 μs · 最后 %3 μs").arg(o["interval_min_us"].toInt()).arg(o["interval_max_us"].toInt()).arg(o["interval_last_us"].toInt());
      if(state==3 || o["flags"].toInt()) {
        deviceAbortDetail=sweepAbortMessage(o);
        source+="\n"+deviceAbortDetail;
        finish(deviceAbortDetail);return;
      }
      if(state!=2)return;
      deviceCompleted=true;
      if(hostH1) {
        resultsReady=true;drainDeadline=clock.elapsed()+1500;refreshRecording();
        if(recording.complete(recordExpected))completeDevice();
        else status->setText("设备已失能，等待录波尾帧；收齐后上位机计算频响");
        return;
      }
    }else{
      if(o["index"].toInt()!=nextPoint || state!=2 || o["flags"].toInt()){finish("设备频点索引或状态不匹配");return;}
      if(o.contains("spike_page")) {
        for(auto it=o.begin();it!=o.end();++it)if(it.key().startsWith("spike_"))spikeSnapshot[it.key()]=it.value();
        spikePages|=1u<<o["spike_page"].toInt();
        if(spikePages==7)source+="\n"+spikeSummary();
      }
      SweepPoint p;p.hz=o["hz"].toDouble();p.input=o["input"].toDouble();p.output=o["output"].toDouble();
      double re=o["real"].toDouble(),im=o["imag"].toDouble(),g=std::hypot(re,im);
      p.gainDb=g>0?20*std::log10(g):0;p.phaseDeg=std::atan2(im,re)*180/std::numbers::pi;
      p.inputFit=o["coherence"].toDouble();p.outputFit=o["blocks"].toInt();p.count=o["blocks"].toInt();
      p.valid=p.hz>0 && p.input>1e-6 && g>1e-6 && p.inputFit>=.9 && p.outputFit>=3;
      p.status=p.valid?(methodChoice->currentData().toInt()==1?"有效·快速估计，建议逐频确认交点":methodChoice->currentData().toInt()==3?"有效·周期 Chirp，建议逐频复核交点":"有效·逐频测量"):"相干性/幅值不足，请逐频复测或检查信噪比";
      results.push_back(p);++nextPoint;render();
    }
    if(nextPoint<deviceCount){deviceToken=session.command(node,0xbe,be16(runId)+QByteArray(1,char(nextPoint)),-1,"读取设备辨识频点");}
    else {resultsReady=true;drainDeadline=clock.elapsed()+1500;
      refreshRecording();
      if(!recordSupported || recording.complete(recordExpected))completeDevice();
      else status->setText("设备已失能，等待录波尾帧；频响结果已保留");
    }
  }
  void render() {
    resultChoice->setVisible(true);
    filterChoice->setEnabled(recordMethod!=2 || !deviceData);
    filterCutoff->setEnabled(lpfSelected() && (recordMethod!=2 || !deviceData));
    const auto &shown=resultChoice->currentIndex()==1 && !recordedResults.empty()?recordedResults:results;
    meanNote->setText(meanActive()?(lpfSelected()?feedbackLpfDescription(recordFs,filterCutoff->value()):feedbackMean16Description(recordFs))+
      "\n原始 H1、相干性及带宽判断保留；粉色曲线/附加列仅对照。"+(meanError.isEmpty()?QString{}:"\n频响对照不可用："+meanError):"给定与反馈共用电流纵轴；原始录波及 H1 始终保留。滤波对照仅适用于 Chirp。");
    const bool compare=meanActive()&&!meanResults.empty();
    table->setColumnCount(compare?12:9);
    table->setHorizontalHeaderLabels(deviceData?QStringList{"Hz","输入谱线 A","反馈谱线 A","H1 增益 dB","相位 °","相干性 γ²","平均段数","段数","质量"}:QStringList{"Hz","输入幅值","反馈幅值","增益 dB","相位 °","输入 R²","反馈 R²","点数","质量"});
    if(compare){table->setHorizontalHeaderItem(9,new QTableWidgetItem("对照增益 dB"));table->setHorizontalHeaderItem(10,new QTableWidgetItem("对照相位 °"));table->setHorizontalHeaderItem(11,new QTableWidgetItem("对照 γ²"));}
    table->setColumnHidden(7,deviceData);
    table->setRowCount(int(shown.size()));
    for (int r = 0; r < int(shown.size()); ++r) {
      auto &p = shown[r];
      QStringList cells = {QString::number(p.hz, 'g', 6),
                           QString::number(p.input, 'g', 6),
                           QString::number(p.output, 'g', 6),
                           p.valid ? QString::number(p.gainDb, 'f', 3) : "—",
                           p.valid ? QString::number(p.phaseDeg, 'f', 2) : "—",
                           QString::number(p.inputFit, 'f', 3),
                           QString::number(p.outputFit, 'f', 3),
                           QString::number(p.count),
                           p.status};
      if(compare){auto it=std::find_if(meanResults.begin(),meanResults.end(),[&](const auto &q){return std::abs(q.hz-p.hz)<1e-6;});
        if(it!=meanResults.end()){cells<<(it->valid?QString::number(it->gainDb,'f',3):"—")<<(it->valid?QString::number(it->phaseDeg,'f',2):"—")<<QString::number(it->inputFit,'f',3);}else cells<<"—"<<"—"<<"—";}
      for (int c = 0; c < cells.size(); ++c)
        table->setItem(r, c, new QTableWidgetItem(cells[c]));
    }
    plot->points = shown;plot->comparison=compare?meanResults:std::vector<SweepPoint>{};plot->comparisonName=comparisonName();
    plot->update();
    bandwidth->setText((deviceData?"原始 H1：":"")+sweepBandwidth(shown)+"\n"+source);
  }
  void finish(QString text) {
    bool was = running;
    if(was && deviceMode) { refreshRecording(true); wave->fitHistory(); }
    if(was && deviceMode && session.connected())session.command(node,0xbc,deviceRequest(0),-1,"停止设备辨识");
    deviceToken=0;methodChoice->setEnabled(true);
    running = false;
    timer.stop();
    start->setEnabled(true);
    import->setEnabled(true);
    startHz->setEnabled(true);
    endHz->setEnabled(true);
    amplitude->setEnabled(true);
    count->setEnabled(true);
    if (was)
      session.emergency();
    status->setText(text + (was ? " · 已请求清零与失能" : ""));
    render();
  }
  void tick() {
    if (!running)
      return;
    if(deviceMode) {
      if(!session.connected() || session.selectedNode()!=node || !safe->isChecked() || clock.elapsed()>60000){finish("设备辨识中止：连接/节点/确认条件变化或超过 60 s");return;}
      refreshRecording();
      if(resultsReady) {
        if(recording.complete(recordExpected)||clock.elapsed()>=drainDeadline)completeDevice();
        return;
      }
      if(!deviceToken) deviceToken=session.command(node,0xbd,{},-1,"查询设备辨识进度（不续租）");
      return;
    }
  }
  void begin() { beginDevice(); }
  void loadRecording(QIODevice &f,const QByteArray &metadata) {
    SweepCapture capture;QString error;
    if(!readSweepCapture(f,metadata,capture,error)){status->setText(error);return;}
    recording=std::move(capture.recording);recordMethod=capture.method;recordFs=capture.fs;
    recordExpected=capture.expected;deviceCompleted=capture.completed;
    deviceData=true;deviceMode=false;recordSupported=true;hostH1=false;results=std::move(capture.points);
    resultChoice->setItemText(0,"文件内原始 H1");
    source="导入设备录波 · "+capture.diagnostic;
    recordedResults=deviceCompleted?recordingH1(recording,recordMethod,recordFs,results,recordExpected):std::vector<SweepPoint>{};
    resultChoice->setCurrentIndex(0);resultChoice->setEnabled(!recordedResults.empty());
    recordStatus->setText(recording.summary(recordExpected,true,deviceCompleted));
    updateMeanResults();drawRecording();render();status->setText("已导入录波及同次设备频响，可选择完整录波复算 H1");
  }
  void load() {
    QString path = QFileDialog::getOpenFileName(this, "导入逐频点稳态数据", {},
                                                "CSV (*.csv)");
    if (path.isEmpty())
      return;
    QFile f(path);
    if (!f.open(QIODevice::ReadOnly)) {
      status->setText(f.errorString());
      return;
    }
    auto firstLine=f.readLine().trimmed();
    const QByteArray recordingPrefix="# motortune_sweep ";
    if(firstLine.startsWith(recordingPrefix)) {loadRecording(f,firstLine.mid(recordingPrefix.size()));return;}
    if (firstLine != "frequency_hz,time_s,input,output") {
      status->setText("表头必须为 "
                      "frequency_hz,time_s,input,"
                      "output；输入与反馈必须同单位、同一时钟，且已去除过渡段");
      return;
    }
    std::map<double, std::vector<SweepSample>> grouped;
    int line = 1;
    size_t total = 0;
    while (!f.atEnd()) {
      auto parts = f.readLine().trimmed().split(',');
      ++line;
      if (parts.size() == 1 && parts[0].isEmpty())
        continue;
      double v[4];
      bool valid = parts.size() == 4;
      if (valid)
        for (int i = 0; i < 4; ++i) {
          bool ok = false;
          v[i] = parts[i].toDouble(&ok);
          valid = valid && ok && std::isfinite(v[i]);
        }
      if (!valid || v[0] <= 0 || ++total > 2000000) {
        status->setText(QString("CSV 第 %1 行无效或超过 200 万点").arg(line));
        return;
      }
      grouped[v[0]].push_back({v[1], v[2], v[3]});
      if (grouped.size() > 200) {
        status->setText("最多 200 个频点");
        return;
      }
    }
    deviceMode=deviceData=false;
    resultChoice->setCurrentIndex(0);resultChoice->setEnabled(false);recordedResults.clear();
    wave->clearDisplay();recordStatus->setText("导入逐点稳态 CSV；无完整设备时序");
    results.clear();
    raw.clear();
    for (auto &[hz, data] : grouped) {
      results.push_back(measureSweep(hz, data));
      raw.push_back(std::move(data));
    }
    source = "导入数据（用户须确认同单位、同一采样时钟及稳态截取）：" +
             QFileInfo(path).fileName();
    status->setText("导入完成；无效频点保留原因，不用于带宽计算");
    render();
  }
  void save() {
    if (results.empty() && recording.samples.empty() && !(deviceData && !deviceAbortDetail.isEmpty())) {
      status->setText("尚无扫频数据");
      return;
    }
    auto path =
        QFileDialog::getSaveFileName(this, "导出频响", {}, "CSV (*.csv)");
    if (path.isEmpty())
      return;
    QSaveFile f(path);
    if (!f.open(QIODevice::WriteOnly)) {
      status->setText(f.errorString());
      return;
    }
    QByteArray out =
        "frequency_hz,input_amplitude,output_amplitude,gain_db,phase_deg,input_"
        "fit_r2,output_fit_r2,count,valid,status\n";
    if(deviceData)out="frequency_hz,input_amplitude,output_amplitude,h1_gain_db,phase_deg,coherence,averages,block_count,valid,status\n";
    for (auto &p : results)
      out += QString("%1,%2,%3,%4,%5,%6,%7,%8,%9,%10\n")
                 .arg(p.hz, 0, 'g', 17)
                 .arg(p.input, 0, 'g', 17)
                 .arg(p.output, 0, 'g', 17)
                 .arg(p.valid ? QString::number(p.gainDb, 'g', 17) : "")
                 .arg(p.valid ? QString::number(p.phaseDeg, 'g', 17) : "")
                 .arg(p.inputFit)
                 .arg(p.outputFit)
                 .arg(p.count)
                 .arg(p.valid ? 1 : 0)
                 .arg(p.status)
                 .toUtf8();
    f.write(out);
    if (!f.commit()) {
      status->setText(f.errorString());
      return;
    }
    if(deviceData) {
      if(meanActive()&&!meanResults.empty()) {
        SweepCapture c{recording,recordMethod,recordFs,recordExpected,deviceCompleted,source,results};auto o=filterOptions();
        auto a=analyzeChirp(c,o);QString suffix=lpfSelected()?".lpf":".mean16";QSaveFile comparison(path+suffix+".csv"),description(path+suffix+".txt");
        if(!comparison.open(QIODevice::WriteOnly)||!description.open(QIODevice::WriteOnly)){status->setText("原始频响已保存，滤波对比文件打开失败");return;}
        auto bytes=chirpAnalysisCsv(a),notes=chirpAnalysisSummary(c,o,a).toUtf8();
        if(comparison.write(bytes)!=bytes.size()||description.write(notes)!=notes.size()||!comparison.commit()||!description.commit()){status->setText("滤波对比导出失败");return;}
      }
      QSaveFile samples(path+".samples.csv"),note(path+".txt");
      if(!samples.open(QIODevice::WriteOnly)||!note.open(QIODevice::WriteOnly)) {status->setText("频响已保存，录波/说明文件打开失败");return;}
      SweepCapture capture{recording,recordMethod,recordFs,recordExpected,deviceCompleted,source,results};
      if(!writeSweepCapture(samples,capture)){status->setText("频响已保存，录波写入失败");return;}
      note.write((source+"\n"+sweepBandwidth(results)+"\n"+recording.summary(recordExpected,recordSupported,deviceCompleted)+
        "\n时间轴=设备采样序号/采样率；x[n] 为本周期施加的给定，y[n] 在施加前采样，保留真实控制延迟。缺样不插值；新固件仅上传原始数据，由上位机计算 H1，滤波对比单独显示。\n").toUtf8());
      bool a=samples.commit(),b=note.commit();
      status->setText(a&&b?(meanActive()&&!meanResults.empty()?"已导出原始频响与录波；滤后对比单独保存为 .lpf 或 .mean16 附件":"已导出设备频响、同步录波和诊断说明"):"频响已保存，但录波/说明保存失败");return;
    }
    QSaveFile samples(path + ".samples.csv"), note(path + ".txt");
    if (!samples.open(QIODevice::WriteOnly) ||
        !note.open(QIODevice::WriteOnly)) {
      status->setText("结果已保存，但原始数据/说明文件打开失败");
      return;
    }
    samples.write("frequency_hz,time_s,input,output\n");
    for (size_t i = 0; i < raw.size(); ++i)
      for (auto &p : raw[i])
        samples.write(QString("%1,%2,%3,%4\n")
                          .arg(results[i].hz, 0, 'g', 17)
                          .arg(p.time, 0, 'g', 17)
                          .arg(p.input, 0, 'g', 17)
                          .arg(p.output, 0, 'g', 17)
                          .toUtf8());
    note.write((source + "\n" + sweepBandwidth(results) +
                "\n闭环输入→反馈幅比；不计算开环增益裕度或相位裕度。\n")
                   .toUtf8());
    if (!samples.commit() || !note.commit())
      status->setText("结果已保存，但原始数据/说明文件写入失败");
  }

public:
  SweepDialog(Session &s, Version v, quint32 n, QWidget *parent)
      : QDialog(parent), session(s), version(v), node(n) {
    qApp->installEventFilter(this);
    setWindowTitle("扫频 / 带宽测试");
    resize(1080, 800);
    setObjectName("sweepDialog");
    auto l = new QVBoxLayout(this);
    methodChoice=new QComboBox;methodChoice->setObjectName("sweepMethod");
    methodChoice->addItem("设备周期线性 Chirp · 上位机频响分析（4 s）",3);
    methodChoice->addItem("设备逐频正弦（精测）· 上位机频响分析",2);l->addWidget(methodChoice);
    auto help = new QLabel("CST / 零给定开始 · 设备只生成 Chirp、同步采样并上传，完成自动失能\n1 s 周期重复 4 次：首周期稳定，后三周期在上位机计算原始 H1 / 相干性。起始 >40 Hz 时实际从 20 Hz 扫起并保留 20/40 Hz 基准。低相干性频点不计算带宽；可导入历史逐频数据对照。",this);
    help->setWordWrap(true);
    l->addWidget(help);
    auto settings = new QHBoxLayout;
    auto spin = [&](QString title, double low, double high, double val) {
      settings->addWidget(new QLabel(title));
      auto w = new QDoubleSpinBox;
      w->setRange(low, high);
      w->setDecimals(3);
      w->setValue(val);
      settings->addWidget(w);
      return w;
    };
    startHz = spin("起始 Hz", 20, 2000, 20);
    endHz = spin("结束 Hz", 20, 2000, 2000);
    amplitude = spin("幅值 A", .005, .5, .05);
    settings->addWidget(new QLabel("对数频点"));
    count = new QSpinBox;
    count->setRange(3, 16);
    count->setValue(16);
    startHz->setDecimals(0);endHz->setDecimals(0);
  startHz->setObjectName("sweepStartHz"); endHz->setObjectName("sweepEndHz"); amplitude->setObjectName("sweepAmplitude"); count->setObjectName("sweepCount");
    settings->addWidget(count);
    l->addLayout(settings);
    safe = new QCheckBox(
        "确认空载、工作区安全；完成/停止后清零并失能（Ctrl+Shift+E 急停）");
    l->addWidget(safe);
    auto row = new QHBoxLayout;
    auto btn = [&](QString title, auto fn) {
      auto b = new QPushButton(title);
      row->addWidget(b);
      connect(b, &QPushButton::clicked, this, fn);
      return b;
    };
    start = btn("开始 Chirp 测试", [this] { begin(); });
    start->setObjectName("startSweep");
    const QString chirpHelp=help->text();
    connect(methodChoice,qOverload<int>(&QComboBox::currentIndexChanged),this,[this,help,chirpHelp]{
      const bool stepped=methodChoice->currentData().toInt()==2;
      start->setText(stepped?"开始逐频精测":"开始 Chirp 测试");
      help->setText(stepped?"CST / 零给定开始 · 设备只生成逐频正弦、同步采样并上传，完成自动失能。\n每点稳定至少 20 ms 且至少 4 周期（向上取整周期），再采集 4 段 × 4 周期；上位机计算原始 H1 / 相干性。起始 >40 Hz 保留 20/40 Hz 基准；实际频率按整数采样周期量化，相邻频点避开重复周期，可用周期不足时拒绝。滤波对照仅适用于 Chirp。":chirpHelp);
    });
    safe->setObjectName("sweepSafe");
    btn("停止并失能", [this] { finish("已停止"); });
    import = btn("导入扫频采样 CSV", [this] { load(); });
    btn("导出频响与原始数据", [this] { save(); });
    l->addLayout(row);
    status = new QLabel("等待测试或导入");
    status->setWordWrap(true);
    l->addWidget(status);
    bandwidth = new QLabel;
    bandwidth->setWordWrap(true);
    l->addWidget(bandwidth);
    auto filterRow=new QHBoxLayout;filterRow->addWidget(new QLabel("反馈 Iq 对照"));
    filterChoice=new QComboBox;filterChoice->setObjectName("sweepFeedbackFilter");filterChoice->addItems({"不滤波（原始）","16 点均值对照","Butterworth LPF 对照"});filterRow->addWidget(filterChoice);
    filterRow->addWidget(new QLabel("LPF 单程截止"));filterCutoff=new QDoubleSpinBox;filterCutoff->setObjectName("sweepLpfCutoff");filterCutoff->setRange(20,8999);filterCutoff->setValue(3000);filterCutoff->setSuffix(" Hz");filterCutoff->setEnabled(false);filterRow->addWidget(filterCutoff);
    filterRow->addWidget(new QLabel("原始 H1 判定带宽；滤后仅对照"));filterRow->addStretch();l->addLayout(filterRow);
    meanNote=new QLabel("给定与反馈共用电流纵轴；原始录波始终保留。16 点均值仅适用于 Chirp。");meanNote->setWordWrap(true);l->addWidget(meanNote);
    auto traceRow=new QHBoxLayout;
    recordStatus=new QLabel("等待设备同步录波");recordStatus->setWordWrap(true);traceRow->addWidget(recordStatus,1);
    resultChoice=new QComboBox;resultChoice->setObjectName("sweepResultSource");
    resultChoice->addItems({"上位机原始 H1","历史录波复算 H1"});resultChoice->setEnabled(false);traceRow->addWidget(resultChoice);
    auto offline=new QPushButton("Chirp 离线分析");offline->setObjectName("chirpOfflineOpen");traceRow->addWidget(offline);
    connect(offline,&QPushButton::clicked,this,[this]{
      if(running){status->setText("请在当前采集结束后打开离线分析");return;}
      SweepCapture c{recording,recordMethod,recordFs,recordExpected,deviceCompleted,source,results};
      createChirpAnalysisDialog(this,c)->show();
    });
    auto fit=new QPushButton("录波全览");traceRow->addWidget(fit);l->addLayout(traceRow);
    auto plots=new QSplitter(Qt::Horizontal);
    auto wavePanel=new QWidget;auto waveLayout=new QVBoxLayout(wavePanel);waveLayout->setContentsMargins(0,0,0,0);
    waveLayout->addWidget(new QLabel("设备时域 / A · 蓝：给定 · 绿：Iq反馈 · 共用纵轴 · 滚轮缩放，拖动平移"));
    wave=new Scope;wave->sampledTraces=true;wave->setObjectName("sweepWaveform");wave->setMinimumSize(280,300);
    wave->timeAxisLabel="设备采样时间 / s · 采样序号 ÷ 采样率";
    wave->aliases=channelNames;wave->aliases[0]="同步给定 / A";wave->aliases[11]="Iq 反馈（原始）/ A";
    wave->visible.fill(false);wave->visible[0]=wave->visible[11]=true;wave->colors[11]=QColor("#5fc992");
    waveLayout->addWidget(wave,1);plots->addWidget(wavePanel);
    plot = new BodePlot;plot->setObjectName("sweepBode");plot->setMinimumWidth(350);plots->addWidget(plot);
    plots->setSizes({500,500});l->addWidget(plots,2);
    connect(fit,&QPushButton::clicked,wave,&Scope::fitHistory);
    connect(resultChoice,qOverload<int>(&QComboBox::currentIndexChanged),this,[this]{render();});
    table = new QTableWidget(0, 9);
    table->setHorizontalHeaderLabels({"Hz", "输入幅值", "反馈幅值", "增益 dB",
                                      "相位 °", "输入 R²", "反馈 R²", "点数",
                                      "质量"});
    table->setEditTriggers(QAbstractItemView::NoEditTriggers);
    table->horizontalHeader()->setSectionResizeMode(
        QHeaderView::ResizeToContents);
    table->horizontalHeader()->setStretchLastSection(true);
    l->addWidget(table, 1);
    connect(filterChoice,qOverload<int>(&QComboBox::currentIndexChanged),this,[this]{updateMeanResults();drawRecording(true);render();});
    connect(filterCutoff,&QDoubleSpinBox::editingFinished,this,[this]{updateMeanResults();drawRecording(true);render();});
    connect(&session,&Session::replied,this,[this](quint64 t,quint32 n,int c,QJsonObject o){deviceReply(t,n,c,o);});
    timer.setTimerType(Qt::PreciseTimer);
    connect(&timer, &QTimer::timeout, this, [this] { tick(); });
    connect(&session, &Session::commandFailed, this,
            [this](quint64 token, QString reason) {
              if (running && (deviceMode && token==deviceToken))
                finish("指令失败：" + reason);
            });
    auto emergency = new QShortcut(QKeySequence("Ctrl+Shift+E"), this);
    connect(emergency, &QShortcut::activated, this, [this] {
      finish("急停");
      session.emergency();
    });
  }
  bool eventFilter(QObject *, QEvent *event) override {
    if (event->type() == QEvent::KeyPress) {
      auto key = static_cast<QKeyEvent *>(event);
      if (key->key() == Qt::Key_E &&
          (key->modifiers() & (Qt::ControlModifier | Qt::ShiftModifier)) ==
              (Qt::ControlModifier | Qt::ShiftModifier)) {
        finish("急停");
        session.emergency();
        return true;
      }
    }
    return false;
  }
  void done(int r) override {
    if (running)
      finish("已关闭测试");
    QDialog::done(r);
  }
};
} // namespace
void showSweepDialog(Session &s, Version v, quint32 node, QWidget *parent) {
  SweepDialog dialog(s, v, node, parent);
  dialog.exec();
}
} // namespace mt
