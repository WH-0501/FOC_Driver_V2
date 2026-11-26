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
