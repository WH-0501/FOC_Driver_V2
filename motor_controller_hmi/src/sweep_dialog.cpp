#include "sweep_dialog.h"
#include "sweep.h"
#include <QtWidgets>
#include <cmath>
#include <map>
#include <numbers>
namespace mt {
namespace {
class BodePlot : public QWidget {
public:
  std::vector<SweepPoint> points;
  BodePlot() { setMinimumHeight(230); }
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
    for (int panel = 0; panel < 2; ++panel) {
      QRectF r(65, 20 + panel * (height() / 2.), width() - 110,
               height() / 2. - 48);
      double low = panel ? -180 : minGain, high = panel ? 180 : maxGain;
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
                 panel ? "相位 / °（主值）" : "增益 / dB");
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
        p.drawText(QPointF(r.left()+5,y-4),"−3 dB 阈值");
      }
      QPainterPath path;
      bool pen = false;
      for (auto &v : points) {
        if (!v.valid) {
          pen = false;
          continue;
        }
        auto pt = pos(v.hz, panel ? v.phaseDeg : v.gainDb);
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
    }
  }
};
class SweepDialog : public QDialog {
  QComboBox *methodChoice;
  bool deviceMode=false,deviceData=false;
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
  QPushButton *start, *import;
  QCheckBox *safe;
  QTimer timer;
  QElapsedTimer clock;
  std::vector<double> frequencies;
  std::vector<SweepPoint> results;
  std::vector<std::vector<SweepSample>> raw;
  int index = 0;
  quint64 firstToken = 0, lastToken = 0;
  bool running = false;
  QString source = "尚无数据";
  double nextTick = 0;
  QByteArray deviceRequest(int action) {
    QByteArray p; p.append(char(1));p.append(char(action));p+=be16(runId);
    p+=be16(int(startHz->value()));p+=be16(int(endHz->value()));p.append(char(count->value()));p+=be16(int(amplitude->value()*1000));return p;
  }
  void beginDevice() {
    auto s=session.latest(node);
    if(version!=Version::V2 || !safe->isChecked() || !session.connected() || !session.telemetryFresh(node) || !s || !s->enabled || s->mode!=6) {
      status->setText("设备辨识要求 V2 固件、CST 已使能、零给定及工作区确认");return;
    }
    if(startHz->value()>=endHz->value() || (startHz->value()>40 && count->value()<4) || (methodChoice->currentIndex()==1 && std::floor(endHz->value()/20)-std::ceil(startHz->value()/20)+1<count->value())) {status->setText("频段过窄或频点过多：快速法使用 20 Hz 整数栅格");return;}
    runId=quint16(QRandomGenerator::global()->bounded(1,65536));
    results.clear();raw.clear();deviceMode=deviceData=true;running=true;nextPoint=0;deviceCount=0;
    clock.start();start->setEnabled(false);import->setEnabled(false);methodChoice->setEnabled(false);
    startHz->setEnabled(false);endHz->setEnabled(false);count->setEnabled(false);amplitude->setEnabled(false);
    source="设备同步采样 · 闭环给定→实际 Iq · H1 频响 / 多段相干性 · 未收到结果";
    deviceToken=session.command(node,0xbc,deviceRequest(methodChoice->currentIndex()),-1,"启动设备电流环辨识（一次配置，无续租）");
    timer.start(100);status->setText("等待固件确认（不支持的固件将超时，不回退到主机发波）");render();
  }
  void deviceReply(quint64 token,quint32 n,int cmd,const QJsonObject &o) {
    if(!running || !deviceMode || token!=deviceToken || n!=node)return;
    deviceToken=0;
    if(!o["ok"].toBool()){finish(o["message"].toString("设备辨识应答无效"));return;}
    if(o["run"].toInt()!=runId){finish("辨识会话编号不匹配");return;}
    int state=o["state"].toInt();

    if(cmd!=0xbe){
      deviceCount=o["count"].toInt();
      if(deviceCount<3 || deviceCount>16){finish("固件返回频点数量无效");return;}
      source=QString("设备采样 %1 Hz · 采集 %2 ms · 最长 FOC 回调 %3 μs · %4 · 同步给定/实际 Iq，H1 频响").arg(o["fs"].toDouble()).arg(o["samples"].toDouble()/std::max(1.,o["fs"].toDouble())*1000,0,'f',1).arg(o["max_us"].toInt()).arg(methodChoice->currentIndex()==1?"快速多正弦":"逐频正弦");
      status->setText(QString("设备进度 %1/%2 · 主机经过 %3 ms").arg(o["progress"].toInt()).arg(deviceCount).arg(clock.elapsed()));
      if(o.contains("timing_revision"))source+=QString(" · 诊断 A6/%1 · 分段峰值：采样/编码器 %2 μs，控制 %3 μs（含 FRA %4 μs）").arg(o["timing_revision"].toInt()).arg(o["sense_us"].toInt()).arg(o["control_us"].toInt()).arg(o["fra_us"].toInt());
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
        QString reason=o["timing_reason"].toInt()==1?"采样周期间隔异常":o["timing_reason"].toInt()==2?"FOC 执行耗时超限":"保护或停止";
        finish(QString("设备中止：%1 · flags=0x%2 · 已采集 %3 点 · 异常位置 %4").arg(reason).arg(o["flags"].toInt(),0,16).arg(o["samples"].toInt()).arg(o["failure_sample"].toInt()));return;
      }
      if(state!=2)return;
    }else{
      if(o["index"].toInt()!=nextPoint || state!=2 || o["flags"].toInt()){finish("设备频点索引或状态不匹配");return;}
      SweepPoint p;p.hz=o["hz"].toDouble();p.input=o["input"].toDouble();p.output=o["output"].toDouble();
      double re=o["real"].toDouble(),im=o["imag"].toDouble(),g=std::hypot(re,im);
      p.gainDb=g>0?20*std::log10(g):0;p.phaseDeg=std::atan2(im,re)*180/std::numbers::pi;
      p.inputFit=o["coherence"].toDouble();p.outputFit=o["blocks"].toInt();p.count=o["blocks"].toInt();
      p.valid=p.hz>0 && p.input>1e-6 && g>1e-6 && p.inputFit>=.9 && p.outputFit>=3;
      p.status=p.valid?(methodChoice->currentIndex()==1?"有效·快速估计，建议逐频确认交点":"有效·逐频测量"):"相干性/幅值不足，请逐频复测或检查信噪比";
      results.push_back(p);++nextPoint;render();
    }
    if(nextPoint<deviceCount){deviceToken=session.command(node,0xbe,be16(runId)+QByteArray(1,char(nextPoint)),-1,"读取设备辨识频点");}
    else {running=false;timer.stop();start->setEnabled(true);import->setEnabled(true);methodChoice->setEnabled(true);startHz->setEnabled(true);endHz->setEnabled(true);amplitude->setEnabled(true);count->setEnabled(true);status->setText(QString("设备辨识完成并失能 · 含传输与显示 %1 ms").arg(clock.elapsed()));render();}
  }
  void render() {
    table->setHorizontalHeaderLabels(deviceData?QStringList{"Hz","输入基波 A","反馈基波 A","H1 增益 dB","相位 °","相干性 γ²","平均段数","段数","质量"}:QStringList{"Hz","输入幅值","反馈幅值","增益 dB","相位 °","输入 R²","反馈 R²","点数","质量"});
    table->setColumnHidden(7,deviceData);
    table->setRowCount(int(results.size()));
    for (int r = 0; r < int(results.size()); ++r) {
      auto &p = results[r];
      QStringList cells = {QString::number(p.hz, 'g', 6),
                           QString::number(p.input, 'g', 6),
                           QString::number(p.output, 'g', 6),
                           p.valid ? QString::number(p.gainDb, 'f', 3) : "—",
                           p.valid ? QString::number(p.phaseDeg, 'f', 2) : "—",
                           QString::number(p.inputFit, 'f', 3),
                           QString::number(p.outputFit, 'f', 3),
                           QString::number(p.count),
                           p.status};
      for (int c = 0; c < cells.size(); ++c)
        table->setItem(r, c, new QTableWidgetItem(cells[c]));
    }
    plot->points = results;
    plot->update();
    bandwidth->setText(sweepBandwidth(results) + "\n" + source);
  }
  void finish(QString text) {
    bool was = running;
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
      if(!deviceToken) deviceToken=session.command(node,0xbd,{},-1,"查询设备辨识进度（不续租）");
      return;
    }
    auto sample = session.latest(node);
    if (!session.connected() || session.upgrading() ||
        session.selectedNode() != node || !session.telemetryFresh(node) ||
        !sample || !sample->enabled || sample->error ||
        sample->mode != (version == Version::V2 ? 6 : 2) ||
        !safe->isChecked() ||
        (sample->environment && sample->values[8] >= 60)) {
      finish("测试中止：连接、使能、模式、温度或遥测条件失效");
      return;
    }
    double hz = frequencies[index], elapsed = clock.nsecsElapsed() / 1e9;
    if (elapsed >= 7 / hz) {
      auto events = session.sampling.targetEvents();
      auto first = std::find_if(events.begin(), events.end(),
                                [&](auto &e) { return e.token == firstToken; });
      std::vector<SweepSample> data;
      if (first != events.end()) {
        double begin = first->queued + 2 / hz, end = first->queued + 7 / hz;
        auto samples = session.sampling.range(begin, end);
        size_t cursor = size_t(first - events.begin());
        double input = 0;
        bool have = false, failed = false;
        for (auto &s : samples) {
          while (cursor < events.size() && events[cursor].queued <= s.time) {
            auto &e = events[cursor++];
            if (e.node != node || !std::isfinite(e.values[2]))
              continue;
            if (e.token < firstToken || e.token > lastToken)
              continue;
            if (e.state == 3) {
              failed = true;
              continue;
            }
            if (e.state == 1 || e.state == 2) {
              input = e.values[2];
              have = true;
            }
          }
          if (s.node == node && have)
            data.push_back({s.time, input, s.values[0]});
        }
        if (failed) {
          finish("测试中止：目标请求失败，当前频点未计入");
          return;
        }
      }
      results.push_back(measureSweep(hz, data));
      raw.push_back(std::move(data));
      render();
      if (++index == int(frequencies.size())) {
        finish("扫频完成");
        return;
      }
      firstToken = 0;
      clock.restart();
      elapsed = 0;
    }
    // Use actual elapsed time, never emit a burst of catch-up commands after a
    // stalled UI.
    if (elapsed - nextTick > .15 && firstToken) {
      finish("测试中止：发送调度停顿超过 150 ms");
      return;
    }
    double value = amplitude->value() * std::sin(2 * std::numbers::pi *
                                                 frequencies[index] * elapsed);
    lastToken =
        session.command(node, 3, target(quint8(node), value, 2, version), -1,
                        QString("扫频电流给定 %1 Hz · %2 A")
                            .arg(frequencies[index])
                            .arg(value));
    if (!firstToken)
      firstToken = lastToken;
    nextTick = elapsed;
    status->setText(QString("频点 %1/%2 · %3 Hz · %4 · %5 s / %6 s")
                        .arg(index + 1)
                        .arg(frequencies.size())
                        .arg(frequencies[index], 0, 'g', 4)
                        .arg(elapsed < 2 / frequencies[index]
                                 ? "等待稳定（2 周期）"
                                 : "测量（5 周期）")
                        .arg(elapsed, 0, 'f', 1)
                        .arg(7 / frequencies[index], 0, 'f', 1));
  }
  void begin() {
    if(methodChoice->currentIndex()>0){beginDevice();return;}
    deviceMode=deviceData=false;
    if (!safe->isChecked()) {
      status->setText("请确认空载和工作区安全");
      return;
    }
    if (startHz->value() >= endHz->value()) {
      status->setText("结束频率必须大于起始频率");
      return;
    }
    auto s = session.latest(node);
    if (!session.connected() || !session.telemetryFresh(node) || !s ||
        !s->enabled || s->mode != (version == Version::V2 ? 6 : 2)) {
      status->setText("在线扫频要求当前节点已使能 CST 模式，且遥测有效");
      return;
    }
    frequencies.clear();
    results.clear();
    raw.clear();
    for (int i = 0; i < count->value(); ++i)
      frequencies.push_back(startHz->value() *
                            std::pow(endHz->value() / startHz->value(),
                                     double(i) / (count->value() - 1)));
    source = QString("Node %1 · 主机电流给定→主机接收 Iq · "
                     "端到端低频频响，含通信/调度延迟；非设备电流环带宽")
                 .arg(node);
    index = 0;
    firstToken = 0;
    nextTick = 0;
    running = true;
    clock.start();
    start->setEnabled(false);
    import->setEnabled(false);
    startHz->setEnabled(false);
    endHz->setEnabled(false);
    amplitude->setEnabled(false);
    count->setEnabled(false);
    methodChoice->setEnabled(false);
    timer.start(20);
    tick();
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
    if (f.readLine().trimmed() != "frequency_hz,time_s,input,output") {
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
    if (results.empty()) {
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
      QSaveFile note(path+".txt");if(note.open(QIODevice::WriteOnly)){note.write((source+"\n"+sweepBandwidth(results)+"\n设备数据 CSV 标注相干性和平均段数；设备只返回频域统计，无原始时序文件。\n").toUtf8());if(!note.commit())status->setText(note.errorString());}else status->setText(note.errorString());return;
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
    methodChoice->addItems({"主机低频（兼容 0.2–4 Hz）","设备快速多正弦（200 ms 采集）","设备逐频正弦（精测）"});l->addWidget(methodChoice);
    auto help = new QLabel("设备辨识：CST / 零给定开始 · 快速法 20 Hz 栅格 · 逐频法整周期平均 · 完成自动失能\n需要配套 FRA 固件。幅值为总峰值上限；起始 >40 Hz 时保留 20/40 Hz 基准点。低相干性频点需复测。",this);
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
    startHz = spin("起始 Hz", .2, 4, .5);
    endHz = spin("结束 Hz", .2, 4, 4);
    amplitude = spin("幅值 A", .005, .5, .05);
    settings->addWidget(new QLabel("对数频点"));
    count = new QSpinBox;
    count->setRange(3, 30);
    count->setValue(10);
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
    start = btn("开始逐频点测试", [this] { begin(); });
    start->setObjectName("startSweep");
    safe->setObjectName("sweepSafe");
    btn("停止并失能", [this] { finish("已停止"); });
    import = btn("导入扫频采样 CSV", [this] { load(); });
    btn("导出频响与原始数据", [this] { save(); });
    btn("转逐频精测",[this]{
      if(running)return;
      double lo=200,hi=1600;
      if(deviceData && results.size()>2){double threshold=(results[0].gainDb+results[1].gainDb)/2-3;
        for(size_t i=1;i<results.size();++i)if(results[i-1].valid&&results[i].valid&&results[i].gainDb<=threshold){lo=std::max(20.,results[i-1].hz*.5);hi=std::min(2000.,results[i].hz*1.5);break;}}
      methodChoice->setCurrentIndex(2);startHz->setValue(lo);endHz->setValue(hi);count->setValue(16);
      status->setText("已设置精测频段；重新确认零给定与使能后点击开始，不自动启动运动");
    });
    l->addLayout(row);
    status = new QLabel("等待测试或导入");
    status->setWordWrap(true);
    l->addWidget(status);
    bandwidth = new QLabel;
    bandwidth->setWordWrap(true);
    l->addWidget(bandwidth);
    plot = new BodePlot;
    l->addWidget(plot, 1);
    table = new QTableWidget(0, 9);
    table->setHorizontalHeaderLabels({"Hz", "输入幅值", "反馈幅值", "增益 dB",
                                      "相位 °", "输入 R²", "反馈 R²", "点数",
                                      "质量"});
    table->setEditTriggers(QAbstractItemView::NoEditTriggers);
    table->horizontalHeader()->setSectionResizeMode(
        QHeaderView::ResizeToContents);
    table->horizontalHeader()->setStretchLastSection(true);
    l->addWidget(table, 1);
    connect(methodChoice,qOverload<int>(&QComboBox::currentIndexChanged),this,[this](int mode){
      startHz->setRange(mode?20:.2,mode?2000:4);endHz->setRange(mode?20:.2,mode?2000:4);
      startHz->setValue(mode?20:.5);endHz->setValue(mode?2000:4);count->setRange(3,mode?16:30);count->setValue(mode?16:10);
    });
    connect(&session,&Session::replied,this,[this](quint64 t,quint32 n,int c,QJsonObject o){deviceReply(t,n,c,o);});
    methodChoice->setCurrentIndex(1);
    timer.setTimerType(Qt::PreciseTimer);
    connect(&timer, &QTimer::timeout, this, [this] { tick(); });
    connect(&session, &Session::commandFailed, this,
            [this](quint64 token, QString reason) {
              if (running && ((deviceMode && token==deviceToken) || (!deviceMode && token >= firstToken && token <= lastToken)))
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
