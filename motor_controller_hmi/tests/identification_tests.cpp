#include "identification_dialog.h"
#include <QtTest>
#include <QtWidgets>
#include <QtEndian>
#include <atomic>
#include <cstring>
using namespace mt;
struct Probe {
    std::atomic<int> starts{0},stops{0},applies{0},saves{0};
    std::atomic<bool> legacy{false},holdStart{false},saveFailure{false},noTelemetry{false},wrongNode{false};
};
class ProbeBackend : public CanBackend {
    std::unique_ptr<CanBackend> demo_=createBackend(Connection{});
    std::shared_ptr<Probe> probe_;
  public:
    explicit ProbeBackend(std::shared_ptr<Probe> p):probe_(std::move(p)){}
    void open(const Connection &c) override {demo_->open(c);}
    void close() override {demo_->close();}
    void send(const Frame &f) override {
        int c=quint8(f.bytes[1]);
        if(c==identControl){if(f.bytes.size()>3&&f.bytes[3])++probe_->starts;else ++probe_->stops;}
        if(c==identApply)++probe_->applies;
        if(c==0x4d)++probe_->saves;
        demo_->send(f);
    }
    std::vector<Frame> receive() override {
        std::vector<Frame> out;
        for(auto f:demo_->receive()) {
            int c=quint8(f.bytes[1]);
            if(c==0&&probe_->noTelemetry)continue;
            if(c>=identControl&&c<=identApply){
                if(probe_->legacy)continue;
                if(c==identControl&&probe_->holdStart&&f.bytes.size()>4&&identificationActive(quint8(f.bytes[4])))continue;
                if(probe_->wrongNode)f.node=2;
            }
            if(c==0x4d&&probe_->saveFailure)f.bytes[2]=0;
            out.push_back(f);
        }
        return out;
    }
};
class IdentificationTests : public QObject {
    Q_OBJECT
    static Session::BackendFactory factory(std::shared_ptr<Probe> p) {
        return [p](const Connection &){return std::make_unique<ProbeBackend>(p);};
    }
    static QPushButton *button(IdentificationDialog &d,const char *name){return d.findChild<QPushButton*>(name);}
    static void open(Session &s) {
        s.open({});QTRY_VERIFY(s.telemetryFresh(1));s.command(1,0xab,QByteArray(1,0));
        QTRY_VERIFY(s.parameters(1)["ab"].toObject()["ok"].toBool());
    }
    static QByteArray validReply() {
        QByteArray p(28,0);p[0]=1;p[2]=7;p[4]=3;
        const float vals[]={1.5f,.0012f,.0018f,.015f};
        for(int i=0;i<4;++i){quint32 b;std::memcpy(&b,&vals[i],4);qToBigEndian(b,p.data()+12+4*i);}
        qToBigEndian<quint32>(4600,p.data()+8);return p;
    }
  private slots:
    void initTestCase() {
        const auto path=qEnvironmentVariable("MOTORTUNE_IDENT_STYLE");
        if(!path.isEmpty()) {
            QFile file(path);QVERIFY(file.open(QIODevice::ReadOnly));
            qApp->setStyle("Fusion");qApp->setStyleSheet(QString::fromUtf8(file.readAll()));
            QFontDatabase::addApplicationFont("C:/Windows/Fonts/msyh.ttc");
            qApp->setFont(QFont("Microsoft YaHei UI",10));
        }
    }
    void wireAndValidation() {
        QCOMPARE(identificationStart(3,{}),QByteArray::fromHex("0103012c0bb800c800c807d003e80bb8"));
        auto c=IdentificationConfig{};c.currentTrip=.4;QVERIFY(!validateIdentification(c).isEmpty());
        QVERIFY_EXCEPTION_THROWN(identificationStart(3,c),std::invalid_argument);
        c={};c.injection=4;QVERIFY(!validateIdentification(c).isEmpty());
        c={};c.speedTrip=24;QVERIFY(!validateIdentification(c).isEmpty());
        c={};c.current=std::numeric_limits<double>::quiet_NaN();QVERIFY(!validateIdentification(c).isEmpty());
        auto p=validReply();auto o=response(identStatus,p,Version::V2);
        QVERIFY(o["valid"].toBool());QVERIFY(o["ok"].toBool());QCOMPARE(o["valid_mask"].toInt(),3);
        QCOMPARE(o["elapsed_ms"].toInt(),4600);QCOMPARE(o["rs"].toDouble(),1.5);
        QVERIFY(std::abs(o["ld"].toDouble()-.0012)<1e-8);
        QVERIFY(response(identStatus,p+QByteArray(2,0),Version::V2)["valid"].toBool());
        for(int size=0;size<28;++size)QVERIFY(!response(identStatus,p.left(size),Version::V2)["valid"].toBool());
        p[0]=2;QVERIFY(!identificationResponse(p)["valid"].toBool());p=validReply();p[2]=4;
        QVERIFY(!identificationResponse(p)["valid"].toBool());p=validReply();qToBigEndian<quint32>(0x7fc00000,p.data()+12);
        QVERIFY(!identificationResponse(p)["valid"].toBool());
        QVERIFY(!response(0x4d,QByteArray(1,0),Version::V2)["ok"].toBool());
        QVERIFY(!response(0x7e,QByteArray::fromHex("01c2"),Version::V2).contains("ok"));
    }
    void completeApplyAndSave() {
        auto probe=std::make_shared<Probe>();Session s(nullptr,factory(probe));open(s);
        IdentificationDialog d(s,Version::V2,1,true);d.show();
        QSignalSpy applied(&d,&IdentificationDialog::parametersApplied),saved(&d,&IdentificationDialog::parametersSaved);
        auto start=button(d,"identStart"),apply=button(d,"identApply"),save=button(d,"identSave");
        QVERIFY(!start->isEnabled());QVERIFY(!apply->isEnabled());QVERIFY(!save->isEnabled());
        d.findChild<QCheckBox*>("identSafe")->setChecked(true);
        QTRY_VERIFY(start->isEnabled());start->click();QTRY_COMPARE(probe->starts.load(),1);
        QVERIFY(!save->isEnabled());QTRY_VERIFY_WITH_TIMEOUT(apply->isEnabled(),7000);
        QCOMPARE(d.findChild<QTableWidget*>("identResults")->item(0,1)->text(),QString("1.5"));
        QVERIFY(!save->isEnabled());apply->click();QTRY_COMPARE(applied.size(),1);QTRY_VERIFY(save->isEnabled());
        save->click();QTRY_COMPARE(saved.size(),1);QCOMPARE(probe->applies.load(),1);QCOMPARE(probe->saves.load(),1);
        QVERIFY(d.findChild<QLabel*>("identStorage")->text().contains("模拟保存成功"));
        const auto preview=qEnvironmentVariable("MOTORTUNE_IDENT_PREVIEW");if(!preview.isEmpty())QVERIFY(d.grab().save(preview));
        // Changing selection never retargets an existing dialog or leaves Apply enabled.
        s.selectNode(2);QTest::qWait(150);QVERIFY(!apply->isEnabled());QVERIFY(!save->isEnabled());
        d.reject();s.shutdown();QVERIFY(s.wait(3000));
    }
    void stopPreemptsPendingStartAndCloseDisables() {
        auto p=std::make_shared<Probe>();p->holdStart=true;Session s(nullptr,factory(p));open(s);
        IdentificationDialog d(s,Version::V2,1,true);d.show();d.findChild<QCheckBox*>("identSafe")->setChecked(true);
        QTRY_VERIFY(button(d,"identStart")->isEnabled());button(d,"identStart")->click();QTRY_COMPARE(p->starts.load(),1);
        button(d,"identStop")->click();QTRY_COMPARE_WITH_TIMEOUT(p->stops.load(),1,800);
        QTRY_VERIFY_WITH_TIMEOUT(d.findChild<QLabel*>("identPhase")->text().contains("已取消"),1000);
        QVERIFY(!button(d,"identApply")->isEnabled());QVERIFY(!button(d,"identSave")->isEnabled());
        p->holdStart=false;QTRY_VERIFY(button(d,"identStart")->isEnabled());button(d,"identStart")->click();
        QTRY_COMPARE(p->starts.load(),2);d.reject();
        QTRY_VERIFY(!s.latest(1)->enabled);
        // Explicit status after close must observe cancellation, not a fake complete.
        QTest::qWait(100);s.command(1,identStatus,QByteArray(1,1));
        QTRY_COMPARE(s.parameters(1)["c1"].toObject()["stage"].toInt(),9);
        s.shutdown();QVERIFY(s.wait(3000));
    }
    void legacyFirmwareAckCannotEnableStart() {
        auto p=std::make_shared<Probe>();p->legacy=true;Session s(nullptr,factory(p));open(s);
        IdentificationDialog d(s,Version::V2,1,false);d.show();d.findChild<QCheckBox*>("identSafe")->setChecked(true);
        QTRY_VERIFY_WITH_TIMEOUT(d.findChild<QLabel*>("identMessage")->text().contains("超时"),3500);
        QVERIFY(!button(d,"identStart")->isEnabled());QVERIFY(!button(d,"identApply")->isEnabled());
        QCOMPARE(p->starts.load(),0);button(d,"identStop")->click();QTRY_COMPARE(p->stops.load(),1);
        d.reject();s.shutdown();QVERIFY(s.wait(3000));
    }
    void otherNodeReplyCannotUnlockControls() {
        auto p=std::make_shared<Probe>();p->wrongNode=true;Session s(nullptr,factory(p));open(s);
        IdentificationDialog d(s,Version::V2,1,false);d.show();d.findChild<QCheckBox*>("identSafe")->setChecked(true);
        QTRY_VERIFY_WITH_TIMEOUT(d.findChild<QLabel*>("identMessage")->text().contains("超时"),3500);
        QVERIFY(!button(d,"identStart")->isEnabled());QVERIFY(!button(d,"identApply")->isEnabled());
        QCOMPARE(p->starts.load(),0);d.reject();s.shutdown();QVERIFY(s.wait(3000));
    }
    void saveFailureAndDisconnectInvalidate() {
        auto p=std::make_shared<Probe>();p->saveFailure=true;Session s(nullptr,factory(p));open(s);
        IdentificationDialog d(s,Version::V2,1,true,nullptr,1);d.show();d.findChild<QCheckBox*>("identSafe")->setChecked(true);
        QSignalSpy saved(&d,&IdentificationDialog::parametersSaved);
        QTRY_VERIFY(button(d,"identStart")->isEnabled());button(d,"identStart")->click();
        QTRY_VERIFY_WITH_TIMEOUT(button(d,"identApply")->isEnabled(),5000);button(d,"identApply")->click();
        QTRY_VERIFY(button(d,"identSave")->isEnabled());button(d,"identSave")->click();
        QTRY_VERIFY(d.findChild<QLabel*>("identMessage")->text().contains("拒绝"));QCOMPARE(saved.size(),0);
        QVERIFY(!d.findChild<QLabel*>("identStorage")->text().contains("保存成功"));
        s.shutdown();QVERIFY(s.wait(3000));QTest::qWait(100);
        QVERIFY(!button(d,"identApply")->isEnabled());QVERIFY(!button(d,"identSave")->isEnabled());d.reject();
    }
    void sessionGatesStaleBroadcastAndV1() {
        auto p=std::make_shared<Probe>();Session s(nullptr,factory(p));open(s);QSignalSpy errors(&s,&Session::commandFailed);
        s.command(0,identControl,identificationStart(3,{}));QTRY_COMPARE(errors.size(),1);QCOMPARE(p->starts.load(),0);
        p->noTelemetry=true;QTest::qWait(850);
        s.command(1,identControl,identificationStart(3,{}));QTRY_COMPARE(errors.size(),2);QCOMPARE(p->starts.load(),0);
        s.command(1,identControl,QByteArray::fromHex("0100"));QTRY_COMPARE(p->stops.load(),1);
        s.shutdown();QVERIFY(s.wait(3000));
        Connection c;c.version=Version::V1;p->noTelemetry=false;s.open(c);QTRY_VERIFY(s.telemetryFresh(1));
        auto before=errors.size();s.command(1,identControl,identificationStart(3,{}));QTRY_COMPARE(errors.size(),before+1);
        QCOMPARE(p->starts.load(),0);s.shutdown();QVERIFY(s.wait(3000));
    }
};
QTEST_MAIN(IdentificationTests)
#include "identification_tests.moc"
