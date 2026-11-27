#include "chirp_analysis_dialog.h"
#include "chirp_analysis.h"
#include "welch_analysis.h"
#include "scope.h"
#include <QtWidgets>
#include <cmath>
namespace mt { namespace {
struct Curve {QString name;QColor color;std::vector<SweepPoint> points;Qt::PenStyle style=Qt::SolidLine;bool breakLargeGap=false;};
class ComparisonPlot: public QWidget {
public:
  std::vector<Curve> curves;
  bool localRetest=false;
  ComparisonPlot(){setMinimumHeight(330);}
  void paintEvent(QPaintEvent*)override{
    QPainter p(this);p.fillRect(rect(),QColor("#0b0e10"));p.setRenderHint(QPainter::Antialiasing);
    double lo=1e9,hi=0,low=-6,high=3;
    for(const auto &c:curves)for(const auto &v:c.points){lo=std::min(lo,v.hz);hi=std::max(hi,v.hz);if(v.valid){low=std::min(low,v.gainDb-1);high=std::max(high,v.gainDb+1);}}
    if(!(hi>lo)){p.setPen(Qt::lightGray);p.drawText(rect(),Qt::AlignCenter,"导入完整 Chirp 录波后显示离线对比");return;}
    int x=65;
    for(const auto &c:curves){p.setPen(c.color);p.drawText(x,18,c.name);x+=p.fontMetrics().horizontalAdvance(c.name)+22;}
    const auto bw=sweepBandwidthResult(curves.empty()?std::vector<SweepPoint>{}:curves[0].points,localRetest);
    const auto crossing=localRetest?SweepCrossingResult{}:sweepClosedLoopCrossingResult(curves[0].points);
    const double fc=crossing.found?crossing.hz:0;
    p.setPen(QColor("#ffbc33"));p.drawText(65,36,bw.found?QString("f_bandwidth = %1 Hz (−3 dB)").arg(bw.hz,0,'g',6):(localRetest?"局部复测 · 不判定绝对带宽":"f_bandwidth：未确认"));
    p.setPen(QColor("#b6a0ff"));p.drawText(65,54,p.fontMetrics().elidedText(crossing.message,Qt::ElideRight,std::max(0,width()-85)));
    for(int panel=0;panel<3;++panel){
      QRectF r(65,78+panel*(height()-78)/3.,width()-105,(height()-78)/3.-35);
      double a=panel==0?low:panel==1?-180:0,b=panel==0?high:panel==1?180:1;
      auto pt=[&](double f,double v){return QPointF(r.left()+std::log(f/lo)/std::log(hi/lo)*r.width(),r.bottom()-(v-a)/(b-a)*r.height());};
      p.setPen(Qt::lightGray);p.drawText(QPointF(65,r.top()-5),panel==0?"增益 / dB":panel==1?"相位 / °":"相干性 γ²（多周期 / 分段估计）");
      for(int j=0;j<=4;++j){double v=a+(b-a)*j/4.,y=pt(lo,v).y();p.setPen(QColor("#283039"));p.drawLine(QPointF(r.left(),y),QPointF(r.right(),y));p.setPen(Qt::lightGray);p.drawText(QRectF(0,y-8,59,18),Qt::AlignRight,QString::number(v,'f',1));
        double f=lo*std::pow(hi/lo,j/4.);QString tick=QString::number(f,'g',3);double tw=p.fontMetrics().horizontalAdvance(tick)+8;
        p.drawText(QRectF(std::clamp(pt(f,a).x()-tw/2,0.,std::max(0.,width()-tw)),r.bottom()+3,tw,18),Qt::AlignCenter,tick);}
      if(panel==0&&bw.baselineValid){
        {double y=pt(lo,bw.baselineDb-3).y();
          p.setPen(QPen(QColor("#bc9c51"),1,Qt::DashLine));p.drawLine(QPointF(r.left(),y),QPointF(r.right(),y));p.drawText(QPointF(r.left()+6,y-4),"原始基准 −3 dB");}}
      for(auto [f,color]:{std::pair{bw.found?bw.hz:0.,QColor("#ffbc33")},std::pair{fc,QColor("#b6a0ff")}})if(f>0){double x=pt(f,a).x();p.setPen(QPen(color,1,Qt::DashLine));p.drawLine(QPointF(x,r.top()),QPointF(x,r.bottom()));}
      for(const auto &c:curves){QPainterPath path;bool join=false;double last=0,lastHz=0;
        for(const auto &v:c.points){double y=panel==0?v.gainDb:panel==1?v.phaseDeg:v.inputFit;
          if(!std::isfinite(y)||(!v.valid&&panel!=2)){join=false;continue;}
          auto pos=pt(v.hz,y);if(join&&!(c.breakLargeGap&&v.hz/lastHz>1.3)&&!(panel==1&&std::abs(y-last)>180))path.lineTo(pos);else path.moveTo(pos);
          p.setPen(c.color);p.drawEllipse(pos,2,2);join=true;last=y;lastHz=v.hz;}
        p.setPen(QPen(c.color,1.4,c.style));p.drawPath(path);}
    }
  }
};
bool loadCapture(const QString &path,SweepCapture &out,QString &error){
  QFile f(path);if(!f.open(QIODevice::ReadOnly)){error=f.errorString();return false;}
  auto first=f.readLine().trimmed();QByteArray prefix="# motortune_sweep ";
  if(!first.startsWith(prefix)){error="请选择带设备元数据的 *.samples.csv，不是频点结果 CSV";return false;}
  return readSweepCapture(f,first.mid(prefix.size()),out,error);
}
class OfflineDialog:public QDialog {
  SweepCapture capture,steppedCapture;
  std::vector<SweepPoint> stepped;
  ChirpAnalysis analysis;
  WelchAnalysis welch;
  ChirpAnalysisOptions options;
  QString capturePath="当前扫频录波副本",stepPath;
  QLabel *summary,*referenceLabel,*welchSummary;
  QComboBox *filter,*neighbor,*welchLength;
  QDoubleSpinBox *cutoff;
  QCheckBox *showCycles;
  ComparisonPlot *plot;
  Scope *wave,*spikeWave;
  QTableWidget *cycles,*points,*spikes,*welchPoints,*welchCompare;
  QPushButton *exportButton;
  static void cell(QTableWidget *t,int r,int c,const QString &s){t->setItem(r,c,new QTableWidgetItem(s));}
  static QString number(double x){return QString::number(x,'g',7);}
  void redrawPlot(){
    plot->localRetest=capture.localRetest;plot->curves.clear();
    if(analysis.ok()){
      plot->curves.push_back({"原始 H1",QColor("#55b3ff"),analysis.raw});
      if(showCycles->isChecked())for(int b=0;b<int(analysis.cycles.size());++b)plot->curves.push_back({QString("周期 %1").arg(b+1),QColor::fromHsv((260+b*65)%360,120,230),analysis.cycles[b],Qt::DotLine});
      if(!analysis.filtered.empty())plot->curves.push_back({options.filter==ChirpFilter::FeedbackMean16?"16 点均值对照":options.filter==ChirpFilter::FeedbackLpf?"Butterworth LPF 对照":"零相位对比",QColor("#ffbc33"),analysis.filtered,Qt::DashLine});
      if(!analysis.neighborhood.empty())plot->curves.push_back({"邻频 H1（诊断）",QColor("#ed91bc"),analysis.neighborhood,Qt::DashDotLine});
      if(welch.ok())plot->curves.push_back({"Welch/CSD（诊断）",QColor("#ff984b"),welch.points});
      if(!stepped.empty())plot->curves.push_back({"逐频正弦",QColor("#5fc992"),stepped,Qt::SolidLine,true});
    }
    plot->update();
  }
  void calculate(){
    options.filter=ChirpFilter(filter->currentIndex());options.cutoffHz=cutoff->value();
    options.neighborHalfWidthHz=neighbor->currentData().toUInt();
    cutoff->setEnabled(options.filter!=ChirpFilter::FeedbackMean16);
    wave->aliases[11]=options.filter==ChirpFilter::FeedbackMean16?"16 点均值后 Iq / A":"零相位滤后 Iq / A";
    analysis=analyzeChirp(capture,options,stepped);
    welch=WelchAnalysis{};
    if(welchLength->currentData().toUInt())welch=analyzeWelch(capture,welchLength->currentData().toUInt(),analysis.raw,stepped);
    welchSummary->setText(welchLength->currentData().toUInt()?welchAnalysisSummary(welch):"Welch/CSD 对照已关闭；原始 H1 不变。");
    welchPoints->setRowCount(int(welch.points.size()));
    for(size_t i=0;i<welch.points.size();++i){const auto &p=welch.points[i];const auto &b=welch.bins[i];
      QStringList row{number(p.hz),number(p.gainDb),number(p.phaseDeg),number(p.inputFit),p.valid?"有效·仅对照":"无效",number(b.inputPsd),number(b.outputPsd),number(b.energySegments)};
      for(const auto &m:{b.raw,b.stepped}){if(m.available)row<<number(m.gainDb)<<number(m.gainDelta)<<number(m.phaseDelta)<<(m.interpolated?"有效邻点插值":"同频");else row<<"—"<<"—"<<"—"<<"无有效邻点";}
      for(int j=0;j<row.size();++j)cell(welchPoints,int(i),j,row[j]);}

    welchCompare->setRowCount(int(welch.comparisons.size()));
    for(size_t i=0;i<welch.comparisons.size();++i){const auto &c=welch.comparisons[i];const auto &p=c.reference;const auto &m=c.welch;
      QStringList row{c.source,number(p.hz),number(p.gainDb),number(p.phaseDeg),number(p.inputFit)};
      if(m.available)row<<number(m.gainDb)<<number(m.phaseDeg)<<number(m.gainDelta)<<number(m.phaseDelta)<<(m.interpolated?"Welch 有效邻点插值":"同频");else row<<"—"<<"—"<<"—"<<"—"<<"无有效邻点/参考无效";
      for(int j=0;j<row.size();++j)cell(welchCompare,int(i),j,row[j]);}
    summary->setText(chirpAnalysisSummary(capture,options,analysis));exportButton->setEnabled(analysis.ok());
    referenceLabel->setText(stepPath.isEmpty()?"逐频对照：未载入。仅离线分析，不发送控制指令。":
      QString("逐频对照 run %1：%2\n频率不同时仅在相邻有效点间按对数频率插值（间距比 ≤1.3）；不跨无效点、不外推。尚未核验两次 PID、固件版本及温度/工况一致。")
        .arg(steppedCapture.recording.run).arg(sweepBandwidth(stepped,steppedCapture.localRetest)));
    cycles->setRowCount(0);points->setRowCount(0);spikes->setRowCount(0);spikeWave->clearDisplay();redrawPlot();wave->clearDisplay();
    if(!analysis.ok())return;
    QStringList headers{"Hz","原始 dB","原始相位 °","原始 γ²","质量"};
    for(const QString &unit:{QString("dB"),QString("°")})for(size_t b=0;b<analysis.cycles.size();++b)headers<<QString("周期%1 %2").arg(b+1).arg(unit);
    headers<<QStringList{"增益跨度 dB","相位跨度 °","滤后 dB","滤后 °","滤波器 dB","逐频 dB","Chirp−逐频 dB","Chirp−逐频 °","对照来源","尖峰谱线 RMS / mA","周期残差谱线 RMS / mA","邻频起点 Hz","邻频终点 Hz","谱线数","邻频 dB","邻频相位 °","邻频 γ²","邻频质量"};
    points->setColumnCount(headers.size());points->setHorizontalHeaderLabels(headers);
    cycles->setRowCount(int(analysis.cycles.size()));
    for(int b=0;b<int(analysis.cycles.size());++b){const auto &s=analysis.cycleStats[b];QStringList row{QString::number(b+1),QString("%1–%2").arg(analysis.settleSeconds+b*analysis.periodSeconds).arg(analysis.settleSeconds+(b+1)*analysis.periodSeconds),number(s.mean),number(s.rms),number(s.peak),number(s.residualRms),QString::number(s.over02A)};for(int j=0;j<row.size();++j)cell(cycles,b,j,row[j]);}
    points->setRowCount(int(analysis.raw.size()));
    for(size_t i=0;i<analysis.raw.size();++i){const auto &p=analysis.raw[i];const auto &c=analysis.checks[i];
      QStringList row{number(p.hz),number(p.gainDb),number(p.phaseDeg),number(p.inputFit),p.valid?"有效":"低相干/幅值不足"};
      for(double v:c.gain)row<<number(v);for(double v:c.phase)row<<number(v);
      row<<number(c.gainSpread)<<number(c.phaseSpread);
      if(analysis.filtered.empty())row<<"—"<<"—"<<"—";else row<<number(analysis.filtered[i].gainDb)<<number(analysis.filtered[i].phaseDeg)<<number(c.filterDb);
      if(c.referenceAvailable)row<<number(c.referenceGain)<<number(c.gainDelta)<<number(c.phaseDelta)<<(c.referenceInterpolated?"插值":"实测同频");else row<<"—"<<"—"<<"—"<<"无有效邻点";
      row<<number(c.spikeLineRms*1000)<<number(c.residualLineRms*1000);
      if(analysis.neighborhood.empty())row<<"—"<<"—"<<"—"<<"—"<<"—"<<"—"<<"关闭";
      else{const auto &b=analysis.neighborhood[i];row<<number(c.bandLowHz)<<number(c.bandHighHz)<<QString::number(c.bandLines)<<number(b.gainDb)<<number(b.phaseDeg)<<number(b.inputFit)<<(b.valid?"通过门槛，仅诊断":"无效");}
      for(int j=0;j<row.size();++j)cell(points,int(i),j,row[j]);}
    std::vector<Sample> samples;size_t n=capture.timing.periodSamples(capture.fs)*capture.timing.settlePeriods;const size_t measured=capture.expected-n;samples.reserve(measured);
    for(size_t i=0;i<measured;++i){Sample s;s.time=(i+n)/capture.fs;s.values.fill(std::numeric_limits<double>::quiet_NaN());
      s.values[0]=capture.recording.samples[n+i].output;
      if(!analysis.filteredOutput.empty())s.values[11]=analysis.filteredOutput[i];
      samples.push_back(s);}
    wave->setSamples(std::move(samples));wave->fitHistory();
    spikes->setRowCount(int(analysis.spikes.size()));
    for(size_t i=0;i<analysis.spikes.size();++i){const auto &s=analysis.spikes[i];QStringList row{
      QString::number(s.cycle),QString::number(s.peakIndex),number(s.peakIndex/capture.fs),number(s.peakIndex/capture.fs-analysis.settleSeconds-(s.cycle-1)*analysis.periodSeconds),
      QString::number(s.lastIndex-s.firstIndex+1),number(s.input),number(s.peak),number(s.before),number(s.after)};
      for(int j=0;j<row.size();++j)cell(spikes,int(i),j,row[j]);}
    if(spikes->rowCount())spikes->setCurrentCell(0,0);
  }
  void showSpike(int row){
    spikeWave->clearDisplay();if(row<0||size_t(row)>=analysis.spikes.size())return;
    const auto &event=analysis.spikes[size_t(row)];unsigned start=event.firstIndex>20?event.firstIndex-20:0;
    unsigned stop=std::min(unsigned(capture.recording.samples.size()),event.lastIndex+21);
    std::vector<Sample> samples;
    for(unsigned i=start;i<stop;++i){const auto &v=capture.recording.samples[i];Sample s;s.time=v.index/capture.fs;s.values.fill(std::numeric_limits<double>::quiet_NaN());s.values[0]=v.output;s.values[11]=v.input;samples.push_back(s);}
    spikeWave->setSamples(std::move(samples));spikeWave->fitHistory();
  }
  void load(bool reference){
    QString path=QFileDialog::getOpenFileName(this,reference?"载入完整逐频正弦录波":"载入完整 Chirp 录波",{},"设备录波 (*.samples.csv);;CSV (*.csv)");
    if(path.isEmpty())return;SweepCapture c;QString error;
    if(!loadCapture(path,c,error)){QMessageBox::warning(this,"录波无效",error);return;}
    if(reference){
      if(c.method!=2||!c.completed||!c.recording.complete(c.expected)){QMessageBox::warning(this,"无法对照","要求完整、无缺样的逐频正弦设备录波");return;}
      auto p=recordingH1(c.recording,2,c.fs,c.points,c.expected);
      if(p.empty()){QMessageBox::warning(this,"无法对照","逐频录波窗口或频点不合法");return;}
      stepped=std::move(p);steppedCapture=std::move(c);stepPath=path;
    }else{capture=std::move(c);capturePath=path;}
    calculate();
  }
  void save(){
    if(!analysis.ok())return;
    QString path=QFileDialog::getSaveFileName(this,"导出离线对比",QString("chirp-run%1.chirp-analysis.csv").arg(capture.recording.run),"CSV (*.csv)");
    if(path.isEmpty())return;
    // Never overwrite a raw capture even if its name was selected in the dialog.
    if(!path.endsWith(".chirp-analysis.csv",Qt::CaseInsensitive))path+=".chirp-analysis.csv";
    QSaveFile csv(path),note(path+".txt"),events(path+".spikes.csv"),welchFile(path+".welch.csv"),compareFile(path+".welch-comparison.csv");
    if(!csv.open(QIODevice::WriteOnly)||!note.open(QIODevice::WriteOnly)||!events.open(QIODevice::WriteOnly)||(welch.ok()&&(!welchFile.open(QIODevice::WriteOnly)||!compareFile.open(QIODevice::WriteOnly)))){QMessageBox::warning(this,"导出失败","无法打开输出文件");return;}
    auto data=chirpAnalysisCsv(analysis);
    QString text="Chirp 来源："+capturePath+"\n逐频来源："+stepPath+"\n"+summary->text()+"\n"+referenceLabel->text()+"\n"+welchSummary->text()+"\n周期,均值A,RMS_A,峰值绝对值A,周期残差RMS_A,abs_Iq_gt_0.2A样本数\n";
    for(int b=0;b<int(analysis.cycles.size());++b){const auto &s=analysis.cycleStats[b];text+=QString("%1,%2,%3,%4,%5,%6\n").arg(b+1).arg(number(s.mean)).arg(number(s.rms)).arg(number(s.peak)).arg(number(s.residualRms)).arg(s.over02A);}
    auto notes=text.toUtf8(),spikeData=chirpSpikesCsv(analysis,capture.fs);bool good=csv.write(data)==data.size()&&note.write(notes)==notes.size()&&events.write(spikeData)==spikeData.size();
    auto welchData=welchAnalysisCsv(welch),comparisonData=welchComparisonCsv(welch);if(welch.ok())good=good&&welchFile.write(welchData)==welchData.size()&&compareFile.write(comparisonData)==comparisonData.size();
    bool a=good&&csv.commit(),b=good&&note.commit(),c=good&&events.commit(),d=!welch.ok()||(good&&welchFile.commit()&&compareFile.commit());
    if(!a||!b||!c||!d)QMessageBox::warning(this,"导出失败","分析结果、说明或尖峰列表未完整保存");else QMessageBox::information(this,"已导出","已保存离线对比 CSV、分析说明及尖峰列表；开启 Welch 时另存 .welch.csv 与 .welch-comparison.csv。原始录波未修改。");
  }
public:
  OfflineDialog(QWidget *parent,const SweepCapture &c):QDialog(parent),capture(c){
    setWindowTitle("Chirp 离线分析 · 原始 H1 / Welch-CSD / 逐频对照");resize(1400,950);setAttribute(Qt::WA_DeleteOnClose);
    auto layout=new QVBoxLayout(this);auto row=new QHBoxLayout;
    auto open=new QPushButton("载入 Chirp 录波"),reference=new QPushButton("载入逐频录波"),clear=new QPushButton("清除逐频对照");
    row->addWidget(open);row->addWidget(reference);row->addWidget(clear);
    filter=new QComboBox;filter->setObjectName("chirpOfflineFilter");filter->addItems({"不滤波（默认）","给定与反馈同滤","仅滤反馈（查看偏差）","反馈 16 点均值（含延迟）","反馈 Butterworth LPF（零相位对照）"});row->addWidget(filter);
    cutoff=new QDoubleSpinBox;cutoff->setObjectName("chirpOfflineCutoff");cutoff->setRange(20,17999);cutoff->setValue(3000);cutoff->setSuffix(" Hz");cutoff->setToolTip("FIR 模式：过渡中心；Butterworth LPF：单程 −3 dB 截止，正反向组合在此处为 −6 dB。详见对照说明。");row->addWidget(cutoff);
    auto apply=new QPushButton("重新分析");apply->setObjectName("chirpOfflineApply");row->addWidget(apply);
    exportButton=new QPushButton("导出对比");row->addWidget(exportButton);layout->addLayout(row);
    auto nr=new QHBoxLayout;nr->addWidget(new QLabel("邻频原始谱平均"));neighbor=new QComboBox;neighbor->setObjectName("chirpOfflineNeighbor");neighbor->addItem("关闭（默认）",0);
    for(unsigned v:{5u,10u,20u,40u})neighbor->addItem(QString("请求 ±%1 Hz").arg(v),v);
    nr->addWidget(neighbor);nr->addWidget(new QLabel("仅诊断；保留原始带宽及 0.9 相干性门槛，不删除尖峰。"));nr->addStretch();layout->addLayout(nr);
    auto wr=new QHBoxLayout;wr->addWidget(new QLabel("Welch/CSD 分段对照"));
    welchLength=new QComboBox;welchLength->setObjectName("chirpOfflineWelch");welchLength->addItem("关闭",0);
    for(unsigned n:{2048u,4096u,8192u,16384u})welchLength->addItem(QString("%1 点 Hann 窗").arg(n),n);
    welchLength->setCurrentIndex(2);wr->addWidget(welchLength);
    auto wh=new QLabel("未滤波原始录波 · 50% 重叠 · γ² ≥0.9 · 带宽说明见 Welch 数据页");wh->setWordWrap(true);wr->addWidget(wh,1);layout->addLayout(wr);
    summary=new QLabel;summary->setObjectName("chirpOfflineSummary");summary->setWordWrap(true);summary->setTextInteractionFlags(Qt::TextSelectableByMouse);layout->addWidget(summary);
    referenceLabel=new QLabel;referenceLabel->setWordWrap(true);layout->addWidget(referenceLabel);
    showCycles=new QCheckBox("叠加各测量周期（单周期 Y/X）");layout->addWidget(showCycles);
    auto tabs=new QTabWidget;plot=new ComparisonPlot;plot->setObjectName("chirpOfflinePlot");tabs->addTab(plot,"频响对比");
    auto wp=new QWidget;auto wl=new QVBoxLayout(wp);welchSummary=new QLabel;welchSummary->setObjectName("chirpWelchSummary");welchSummary->setWordWrap(true);welchSummary->setTextInteractionFlags(Qt::TextSelectableByMouse);wl->addWidget(welchSummary);
    auto wm=new QLabel("差值仅在双方有效时显示；原始 H1 / 逐频参考只在相邻有效点间插值（频率比 ≤1.3），不跨无效点，不外推。能量等效段数不代表独立平均次数。PSD 单位 A²/Hz。");wm->setWordWrap(true);wl->addWidget(wm);
    welchPoints=new QTableWidget(0,16);welchPoints->setObjectName("chirpWelchPoints");welchPoints->setHorizontalHeaderLabels({"Hz","Welch dB","Welch °","γ²","质量","输入 PSD","反馈 PSD","输入能量等效段数","原始 H1 dB","Welch−原始 dB","Welch−原始 °","原始来源","逐频 dB","Welch−逐频 dB","Welch−逐频 °","逐频来源"});wl->addWidget(welchPoints);tabs->addTab(wp,"Welch/CSD 数据");
    auto wc=new QWidget;auto wcl=new QVBoxLayout(wc);auto wch=new QLabel("按原始 H1 / 逐频正弦的频率比较：仅在相邻有效 Welch 谱线之间插值，不跨无效点、不外推；插值行不参与任何带宽判定。差值为 Welch 减参考。");wch->setWordWrap(true);wcl->addWidget(wch);
    welchCompare=new QTableWidget(0,10);welchCompare->setObjectName("chirpWelchCompare");welchCompare->setHorizontalHeaderLabels({"参考","Hz","参考 dB","参考 °","参考 γ²","Welch dB","Welch °","差值 dB","差值 °","来源/质量"});wcl->addWidget(welchCompare);tabs->addTab(wc,"Welch 对照差值");
    auto consistency=new QWidget;auto cl=new QVBoxLayout(consistency);
    cl->addWidget(new QLabel("周期残差＝本周期反馈减去所有测量周期的逐样本均值。0.2 A 只计数，不作为删点或保护阈值。"));
    cycles=new QTableWidget(0,7);cycles->setHorizontalHeaderLabels({"周期","时间 / s","均值 / A","RMS / A","绝对峰值 / A","残差 RMS / A","|Iq| >0.2 A 点数"});cl->addWidget(cycles);tabs->addTab(consistency,"周期一致性");
    points=new QTableWidget(0,29);points->setObjectName("chirpOfflinePoints");points->setColumnCount(0);tabs->addTab(points,"逐频点数据");
    auto sp=new QWidget;auto sl=new QVBoxLayout(sp);auto help=new QLabel("选中事件查看前后各 20 点原始录波。|Iq| >0.2 A 仅作观察标记，不修改 H1。尖峰谱线贡献与其他样本可相长或相消，不可视为独立噪声占比。");help->setWordWrap(true);sl->addWidget(help);
    spikes=new QTableWidget(0,9);spikes->setObjectName("chirpOfflineSpikes");spikes->setSelectionBehavior(QAbstractItemView::SelectRows);spikes->setHorizontalHeaderLabels({"周期","峰值序号","设备时间 s","周期内时间 s","连续点数","给定 A","Iq 峰值 A","事件前 Iq A","事件后 Iq A"});sl->addWidget(spikes);
    spikeWave=new Scope;spikeWave->sampledTraces=true;spikeWave->aliases[0]="原始 Iq / A";spikeWave->aliases[11]="原始给定 / A";spikeWave->visible.fill(false);spikeWave->visible[0]=spikeWave->visible[11]=true;spikeWave->colors[11]=QColor("#5fc992");spikeWave->timeAxisLabel="设备采样时间 / s（原始值）";sl->addWidget(spikeWave,1);tabs->addTab(sp,"尖峰位置");
    wave=new Scope;wave->sampledTraces=true;wave->aliases=channelNames;wave->aliases[0]="原始 Iq / A";wave->aliases[11]="零相位滤后 Iq / A";wave->visible.fill(false);wave->visible[0]=wave->visible[11]=true;wave->colors[11]=QColor("#5fc992");wave->timeAxisLabel="原始设备采样时间 / s（仅设备定义的测量段）";tabs->addTab(wave,"原始/滤后录波");
    for(auto t:{cycles,points,spikes,welchPoints,welchCompare}){t->setEditTriggers(QAbstractItemView::NoEditTriggers);t->horizontalHeader()->setSectionResizeMode(QHeaderView::ResizeToContents);t->horizontalHeader()->setStretchLastSection(true);}
    layout->addWidget(tabs,1);
    connect(open,&QPushButton::clicked,this,[this]{load(false);});connect(reference,&QPushButton::clicked,this,[this]{load(true);});
    connect(clear,&QPushButton::clicked,this,[this]{stepped.clear();stepPath.clear();calculate();});
    connect(apply,&QPushButton::clicked,this,[this]{calculate();});connect(exportButton,&QPushButton::clicked,this,[this]{save();});
    connect(welchLength,&QComboBox::currentIndexChanged,this,[this]{calculate();});
    connect(showCycles,&QCheckBox::toggled,this,[this]{redrawPlot();});
    connect(spikes,&QTableWidget::currentCellChanged,this,[this](int row,int,int,int){showSpike(row);});calculate();
  }
};
} // namespace
QDialog *createChirpAnalysisDialog(QWidget *parent,const SweepCapture &capture){return new OfflineDialog(parent,capture);}
} // namespace mt
