#include "sweep_dialog.h"
#include "scope.h"
#include <QtWidgets>
#include <QtTest>
#include <QLabel>
#include <QtEndian>
#include <cstring>
using namespace mt;
// Replays actual native FOC output through the real worker/decoder/dialog.
// No CAN driver is loaded and no physical motor is accessed.
class CaptureBackend:public CanBackend {
  std::vector<QByteArray> raw,results;
  std::vector<Frame> replies;
  size_t cursor=0;
  int method,state=0;
  quint16 run=0;
  quint32 samples=0;
  bool enabled=true,drop,abort;
  quint8 counter=0;
  void queue(int cmd,const QByteArray &p){replies.push_back({1,pack(counter++,cmd,p),0});}
public:
  CaptureBackend(int m,bool d,bool a):method(m),drop(d),abort(a){}
  void open(const Connection &) override {
    QFile f(qEnvironmentVariable("MOTORTUNE_CAPTURE_FIXTURES")+QString("/capture-%1.frames").arg(method));
    if(!f.open(QIODevice::ReadOnly))throw std::runtime_error("Missing native capture fixture");
    while(!f.atEnd()){
      auto line=f.readLine().trimmed();
      if(line.startsWith("R ")){
        auto fields=line.split(' ');QByteArray p(34,0);p[0]=1;p[4]=char(results.size());p[5]=2;
        for(int i=0;i<6;++i){float v=fields[i+1].toFloat();quint32 bits;memcpy(&bits,&v,4);qToBigEndian(bits,p.data()+8+4*i);}
        qToBigEndian<quint16>(method==2?4:3,p.data()+32);results.push_back(p);
      }else {auto p=QByteArray::fromHex(line);samples=qFromBigEndian<quint32>(p.constData()+4)+quint8(p[1]);raw.push_back(p);}
    }
  }
  void close() override {}
  void send(const Frame &f) override {
    int cmd=quint8(f.bytes[1]);auto p=f.bytes.mid(2);
    queue(0x7e,f.bytes.left(2));
    if(cmd==0xbc||cmd==0xbd){
      if(cmd==0xbc && p.size()>=13 && p[1]){run=u16(p,2);state=1;cursor=0;}
      if(cmd==0xbc && p.size()>1 && !p[1]){state=3;enabled=false;}
      if(abort && state==1 && cursor>=500){state=3;enabled=false;}
      if(state==1 && cursor==raw.size()){state=2;enabled=false;}
      QByteArray out(method==3?52:43,0);out[0]=1;out[2]=char(state);out[3]=char(method);
      qToBigEndian(run,out.data()+4);out[6]=char(results.size());out[7]=state==2?out[6]:0;
      qToBigEndian<quint32>(20000,out.data()+8);qToBigEndian(samples,out.data()+12);
      if(abort && state==3){qToBigEndian<quint16>(4,out.data()+18);}
      out[38]=char(0xa6);out[39]=1;out[40]=char(0xa9);out[41]=out[42]=1;
      if(method==3){out[43]=char(0xaa);qToBigEndian<quint32>(quint32(cursor*6),out.data()+48);}
      queue(cmd,out);
    }else if(cmd==0xbe&&p.size()>=3&&quint8(p[2])<results.size()){
      auto out=results[quint8(p[2])];qToBigEndian(run,out.data()+2);queue(cmd,out);
    }else if(cmd==0x39 || cmd==0x40){enabled=false;queue(cmd,QByteArray(1,0));}
  }
  std::vector<Frame> receive() override {
    std::vector<Frame> out;out.swap(replies);
    QByteArray t(18,0);t[14]=char(6|(enabled?16:0));t[16]=25;t[17]=240;
    out.push_back({1,pack(counter++,0,t),0});
    if(state==1)for(int i=0;i<100 && cursor<raw.size();++i,++cursor){
      if(drop && cursor==10)continue;
      auto p=raw[cursor];qToBigEndian(run,p.data()+2);out.push_back({1,pack(counter++,0xbf,p),0});
    }
    return out;
  }
};
struct PlanProbe { QMutex mutex; QByteArray request; };
// Accelerated 28-second capture through the real worker, decoder and dialog.
class TimedCaptureBackend:public CanBackend {
  std::shared_ptr<PlanProbe> probe;
  std::vector<Frame> replies;quint32 cursor=0;quint16 run=0;quint8 sequence=0;
  int state=0,plannedCount=8;bool enabled=true;double feedback=0;
  void queue(int cmd,const QByteArray &p){replies.push_back({1,pack(sequence++,cmd,p),0});}
public:
  explicit TimedCaptureBackend(std::shared_ptr<PlanProbe> p={}):probe(std::move(p)){}
  void open(const Connection &) override {}
  void close() override {}
  void send(const Frame &f) override {
    int cmd=quint8(f.bytes[1]);auto p=f.bytes.mid(2);queue(0x7e,f.bytes.left(2));
    if(cmd==0xbc||cmd==0xbd){
      int error=0;
      if(cmd==0xbc&&p.size()>1&&p[1]) {
        if(probe){QMutexLocker lock(&probe->mutex);probe->request=p;return;}
        if(p.size()<19||p.mid(13,6)!=QByteArray::fromHex("ae020fa00106"))error=2;
        else {run=u16(p,2);state=1;cursor=0;plannedCount=quint8(p[8]);}
      }
      if(cmd==0xbc&&p.size()>1&&!p[1]){state=3;enabled=false;}
      if(state==1&&cursor==560000){state=2;enabled=false;}
      QByteArray out(62,0);out[0]=1;out[1]=char(error);out[2]=char(state);out[3]=3;
      qToBigEndian(run,out.data()+4);out[6]=char(plannedCount);out[7]=state==2?out[6]:0;
      qToBigEndian<quint32>(20000,out.data()+8);qToBigEndian(cursor,out.data()+12);
      out[40]=char(0xa9);out[41]=1;out[42]=state?1:0;out[43]=char(0xaa);qToBigEndian(cursor,out.data()+48);
      out[52]=char(0xac);out[53]=1;out[54]=7;out[55]=7;
      out.replace(56,6,QByteArray::fromHex("ae020fa00106"));queue(cmd,out);
    }else if(cmd==0x39||cmd==0x40){enabled=false;state=3;queue(cmd,QByteArray(1,0));}
  }
  std::vector<Frame> receive() override {
    std::vector<Frame> out;out.swap(replies);
    QByteArray t(18,0);t[14]=char(6|(enabled?16:0));t[16]=25;t[17]=240;out.push_back({1,pack(sequence++,0,t),0});
    for(unsigned frame=0;state==1&&cursor<560000&&frame<600;++frame){
      unsigned n=std::min(6u,560000-cursor);QByteArray p(8+8*n,0);p[0]=1;p[1]=char(n);qToBigEndian(run,p.data()+2);qToBigEndian(cursor,p.data()+4);
      for(unsigned j=0;j<n;++j){double ts=(cursor%80000)/20000.;float x=float(.05*std::sin(2*3.141592653589793*(20*ts+247.5*ts*ts))),y=float(feedback);
        feedback+=.22*(x-feedback);quint32 xb,yb;memcpy(&xb,&x,4);memcpy(&yb,&y,4);qToBigEndian(xb,p.data()+8+8*j);qToBigEndian(yb,p.data()+12+8*j);++cursor;}
      out.push_back({1,pack(sequence++,0xbf,p),0});
    }return out;
  }
};
class CaptureUiTests:public QObject{
 Q_OBJECT
private slots:
 void initTestCase(){
   QFontDatabase::addApplicationFont("C:/Windows/Fonts/msyh.ttc");
   QFontDatabase::addApplicationFont("C:/Windows/Fonts/segoeui.ttf");
   qApp->setStyle("Fusion");qApp->setFont(QFont("Microsoft YaHei UI",10));
   QFile f(qEnvironmentVariable("MOTORTUNE_SWEEP_STYLE"));
   if(f.open(QIODevice::ReadOnly))qApp->setStyleSheet(QString::fromUtf8(f.readAll()));
 }

 void outgoingPlanPreservesWireContract() {
   for(int method:{2,3})for(bool local:{false,true}) {
     auto probe=std::make_shared<PlanProbe>();
     Session session(nullptr,[probe](const Connection &){return std::make_unique<TimedCaptureBackend>(probe);});
     session.open({});QTRY_VERIFY(session.telemetryFresh(1));QWidget parent;
     QTimer::singleShot(20,&parent,[&]{
       auto d=parent.findChild<QDialog*>("sweepDialog");QVERIFY(d);
       d->findChild<QComboBox*>("sweepMethod")->setCurrentIndex(method==2?1:0);
       d->findChild<QComboBox*>("sweepPurpose")->setCurrentIndex(local?1:0);
       d->findChild<QDoubleSpinBox*>("sweepStartHz")->setValue(600);
       d->findChild<QDoubleSpinBox*>("sweepEndHz")->setValue(1500);
       d->findChild<QSpinBox*>("sweepCount")->setValue(3);
       d->findChild<QCheckBox*>("sweepSafe")->setChecked(true);
       d->findChild<QPushButton*>("startSweep")->click();
       QTimer::singleShot(500,d,&QDialog::reject);
     });
     showSweepDialog(session,Version::V2,1,&parent);session.shutdown();
     QMutexLocker lock(&probe->mutex);const auto &p=probe->request;
     QVERIFY(p.size()>=19);QCOMPARE(quint8(p[1]),method);
     QCOMPARE(u16(p,4),local?600:20);QCOMPARE(u16(p,6),1500);
     QCOMPARE(quint8(p[8]),local?3:9);QCOMPARE(quint8(p[14]),2);
     if(method==2) {
       QCOMPARE(quint8(p[13]),0xad);const int payloadSize=17+2*(local?3:9);
       QVERIFY(p.size()>=payloadSize);QCOMPARE(p.mid(payloadSize),QByteArray(p.size()-payloadSize,0));
       QCOMPARE(u16(p,15),20000);QCOMPARE(u16(p,17),local?33:1000);
       QCOMPARE(u16(p,payloadSize-2),13);
     } else {QCOMPARE(quint8(p[13]),0xae);QCOMPARE(p.mid(19),QByteArray(p.size()-19,0));}
   }
 }
 void deviceTimingLongCapture_data() {QTest::addColumn<bool>("local");QTest::newRow("full")<<false;QTest::newRow("local")<<true;}
 void deviceTimingLongCapture() {
   QFETCH(bool,local);
   Session session(nullptr,[](const Connection &){return std::make_unique<TimedCaptureBackend>();});
   session.open({});QTRY_VERIFY(session.telemetryFresh(1));QWidget parent;
   bool complete=false,timing=false;int rows=0,blocks=0;QTimer poll;QElapsedTimer deadline;deadline.start();
   QTimer::singleShot(20,&parent,[&]{auto d=parent.findChild<QDialog*>("sweepDialog");QVERIFY(d);
     d->findChild<QComboBox*>("sweepPurpose")->setCurrentIndex(local?1:0);
     d->findChild<QSpinBox*>("sweepCount")->setValue(8);d->findChild<QCheckBox*>("sweepSafe")->setChecked(true);
     d->findChild<QPushButton*>("startSweep")->click();
     connect(&poll,&QTimer::timeout,d,[&,d]{
       if(!d->findChild<QPushButton*>("startSweep")->isEnabled()&&deadline.elapsed()<25000)return;
       auto table=d->findChild<QTableWidget*>();rows=table->rowCount();if(rows)blocks=int(table->item(0,6)->text().toDouble());
       for(auto label:d->findChildren<QLabel*>()){complete|=label->text().contains("560000 / 560000")&&label->text().contains("测量完成");timing|=label->text().contains("6 个测量周期");}
       bool localText=false;for(auto label:d->findChildren<QLabel*>())localText|=label->text().contains("局部复测：仅显示原始 H1");QCOMPARE(localText,local);
       auto purpose=d->findChild<QComboBox*>("sweepPurpose");QVERIFY(purpose->isEnabled());purpose->setCurrentIndex(local?0:1);
       localText=false;for(auto label:d->findChildren<QLabel*>())localText|=label->text().contains("局部复测：仅显示原始 H1");QCOMPARE(localText,!local);
       poll.stop();d->reject();
     });poll.start(100);
   });
   showSweepDialog(session,Version::V2,1,&parent);
   QVERIFY(complete);QVERIFY(timing);QCOMPARE(rows,local?8:10);QCOMPARE(blocks,6);QCOMPARE(session.sweepRecording().samples.size(),560000u);
 }
 void simultaneousWaveAndBode_data(){
   QTest::addColumn<int>("method");QTest::addColumn<bool>("drop");QTest::addColumn<bool>("abort");
   QTest::newRow("chirp")<<3<<false<<false;
   QTest::newRow("missing-packet")<<3<<true<<false;
   QTest::newRow("timing-abort")<<3<<false<<true;
 }
 void simultaneousWaveAndBode(){
   QFETCH(int,method);QFETCH(bool,drop);QFETCH(bool,abort);
   if(qEnvironmentVariable("MOTORTUNE_CAPTURE_FIXTURES").isEmpty())QSKIP("Requires native fixtures");
   Session s(nullptr,[=](const Connection &){return std::make_unique<CaptureBackend>(method,drop,abort);});
   s.open({});QTRY_VERIFY(s.telemetryFresh(1));
   QWidget parent;bool done=false;int rows=0;bool canRecompute=false;size_t rawCount=0;double span=0,expectedSpan=0;
   QTimer poll;QElapsedTimer deadline;deadline.start();
   QTimer::singleShot(20,&parent,[&]{
     auto d=parent.findChild<QDialog*>("sweepDialog");QVERIFY(d);
     d->findChild<QComboBox*>("sweepMethod")->setCurrentIndex(0);
     d->findChild<QSpinBox*>("sweepCount")->setValue(8);
     d->findChild<QCheckBox*>("sweepSafe")->setChecked(true);
     d->findChild<QPushButton*>("startSweep")->click();
     connect(&poll,&QTimer::timeout,d,[&,d]{
       auto start=d->findChild<QPushButton*>("startSweep");
       if(!start->isEnabled() && deadline.elapsed()<15000)return;
       if(method==3){
         bool diagnostic=false;
         for(auto label:d->findChildren<QLabel*>())diagnostic|=label->text().contains("设备缓冲丢样 0 点");
         QVERIFY(diagnostic);
       }
       rows=d->findChild<QTableWidget*>()->rowCount();
       auto choice=d->findChild<QComboBox*>("sweepResultSource");
       canRecompute=choice->isEnabled();
       auto wave=d->findChild<Scope*>("sweepWaveform");
       auto data=wave->recordedSamples();rawCount=data.size();span=wave->windowSeconds;
       if(!data.empty())expectedSpan=data.back().time-data.front().time;
       if(canRecompute)choice->setCurrentIndex(1);
       if(method==3&&!drop&&!abort) {
         auto path=qEnvironmentVariable("MOTORTUNE_CAPTURE_PREVIEW");
         if(!path.isEmpty()){d->resize(1320,960);d->grab().save(path);}
       }
       done=true;poll.stop();d->reject();
     });poll.start(100);
   });
   showSweepDialog(s,Version::V2,1,&parent);
   QVERIFY(done);QCOMPARE(rows,abort?0:8);QVERIFY(rawCount>1000);QCOMPARE(canRecompute,!drop&&!abort);
   QVERIFY(std::abs(span-expectedSpan)<1e-6);
   QTRY_VERIFY(!s.latest(1)->enabled);s.shutdown();
 }
};
QTEST_MAIN(CaptureUiTests)
#include "sweep_capture_ui_tests.moc"
