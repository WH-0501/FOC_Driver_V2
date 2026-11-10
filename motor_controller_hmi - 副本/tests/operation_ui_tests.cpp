#include "window.h"
#include <QtTest>
using namespace mt;
class OperationUiTests : public QObject {
    Q_OBJECT
  private slots:
    void pausedDebugSurvivesOverwriteAndZoom() {
        Sampling history(200);
        for (int i = 0; i < 200; ++i) {
            Sample v;
            v.time = i * .01;
            v.debugLoop = 1;
            v.enabled = true;
            v.values[11] = i < 100 ? 0 : .2;
            v.values[0] = i < 100 ? 0 : .2;
            history.append(v);
        }
        Scope scope;
        scope.debugProfile = true;
        scope.resize(900, 600);
        scope.bind(&history);
        scope.show();
        scope.fitHistory();
        scope.setDisplayPaused(true);
        auto before = scope.grab().toImage();
        QCOMPARE(scope.recordedSamples().size(), size_t(200));
        QCOMPARE(stepResponses(scope.recordedSamples(), 0, .002, true).size(), 1);
        for (int i = 200; i < 600; ++i) {
            Sample v;
            v.time = i * .01;
            v.debugLoop = 1;
            v.enabled = true;
            v.values[0] = v.values[11] = -.5;
            history.append(v);
        }
        scope.refresh();
        QPointF point(450, 300);
        QWheelEvent wheel(point, scope.mapToGlobal(point.toPoint()), {}, QPoint(0, 120), Qt::NoButton,
                          Qt::NoModifier, Qt::NoScrollPhase, false);
        QApplication::sendEvent(&scope, &wheel);
        QVERIFY(!scope.samples().empty());
        scope.fitHistory();
        QCOMPARE(scope.grab().toImage(), before);
        QCOMPARE(scope.recordedSamples().front().time, 0.0);
        QCOMPARE(stepResponses(scope.recordedSamples(), 0, .002, true).size(), 1);
        scope.setDisplayPaused(false);
        scope.fitHistory();
        QCOMPARE(scope.recordedSamples().front().time, 4.0);
        QVERIFY(stepResponses(scope.recordedSamples(), 0, .002, true).empty());
    }
    void sharedPidScopeKeepsDeviceDataSeparate() {
        const auto preview = qEnvironmentVariable("MOTORTUNE_PID_PREVIEW");
        if (!preview.isEmpty()) {
            qApp->setStyle("Fusion");
            qApp->setFont(QFont("Microsoft YaHei UI", 10));
            QFile style(qEnvironmentVariable("MOTORTUNE_PID_STYLE"));
            if (style.open(QIODevice::ReadOnly))
                qApp->setStyleSheet(QString::fromUtf8(style.readAll()));
        }
        Window w(false, false, true);
        if (!preview.isEmpty())
            w.resize(1680, 1050);
        w.show();
        w.startDemo();
        QTRY_VERIFY(w.session.telemetryFresh(1));
        w.session.command(1, 0x41, QByteArray(1, 3));
        QTRY_COMPARE(w.session.latest(1)->mode, 3);
        w.session.command(1, 0x40, QByteArray(1, 1));
        QTRY_VERIFY(w.session.latest(1)->enabled);
        auto profile = w.findChild<QComboBox *>("scopeProfile");
        auto loop = w.findChild<QComboBox *>("debugLoop");
        auto subscription = w.findChild<QComboBox *>("debugSubscription");
        auto apply = w.findChild<QPushButton *>("debugApply");
        QVERIFY(profile && loop && subscription && apply);
        profile->setCurrentIndex(1);
        subscription->setCurrentIndex(1);
        QTRY_VERIFY(apply->isEnabled());
        apply->click();
        QTRY_VERIFY(w.session.debugSampling[0].info().size > 15);
        QVERIFY(w.session.sampling.info().size > 0);
        w.session.command(1, 0x30, combinedTarget(1, 0x30, .5, 30, .2, Version::V2));
        QTRY_VERIFY(w.session.debugSampling[0].snapshot().back().values[11] > .19);
        QTRY_VERIFY(w.session.debugSampling[1].snapshot().back().values[11] > 3);
        loop->setCurrentIndex(1);
        auto scope = w.findChild<Scope *>();
        QTRY_VERIFY(!scope->samples().empty());
        QTRY_COMPARE(scope->samples().back().debugLoop, quint8(2));
        QVERIFY(scope->aliases[0].contains("rad/s"));
        auto before = w.session.debugSampling[1].info().revision;
        QSignalSpy logs(&w.session, &Session::log);
        QTest::qWait(3400); // Subscription persists without any B8 renewal.
        for (const auto &entry : logs) {
            const auto text = entry[1].toString();
            QVERIFY2(!(entry[0].toString() == "TX" && text.contains(" b8 ")), qPrintable(text));
        }
        QVERIFY(w.session.debugSampling[1].info().revision > before + 100);
        auto rows = stepResponses(w.session.debugSampling[1].snapshot(), 0);
        QVERIFY(!rows.empty());
        QVERIFY(rows.last().toObject()["source"].toString().contains("设备"));
        QCOMPARE(w.session.latest(1)->mode, quint8(3)); // Subscribing never changes control mode.
        QVERIFY(w.session.latest(1)->enabled);
        if (!preview.isEmpty())
            QVERIFY(w.grab().save(preview));
        w.findChild<QListWidget *>("navigation")->setCurrentRow(3);
        QPushButton *analyze = nullptr;
        for (auto b : w.findChildren<QPushButton *>())
            if (b->text() == "阶跃响应")
                analyze = b;
        QVERIFY(analyze);
        analyze->click();
        auto table = w.findChild<QTableWidget *>("stepList");
        QTRY_VERIFY(table->rowCount() > 0);
        auto detail = table->item(0, 0)->data(Qt::UserRole).toJsonObject();
        QVERIFY(detail["channel"].toString().contains("rad/s"));
        QVERIFY(detail["source"].toString().contains("设备"));
        if (!preview.isEmpty())
            QVERIFY(w.grab().save(preview + ".analysis.png"));
        w.findChild<QListWidget *>("navigation")->setCurrentRow(0);
        auto normalSize = w.session.sampling.info().size;
        auto otherSize = w.session.debugSampling[0].info().size;
        w.findChild<QPushButton *>("clearWaveData")->click();
        QVERIFY(w.session.sampling.info().size >= normalSize);
        QVERIFY(w.session.debugSampling[0].info().size >= otherSize);
        profile->setCurrentIndex(0);
        QTest::qWait(200);
        auto stopped = w.session.debugSampling[1].info().revision;
        QTest::qWait(100);
        QCOMPARE(w.session.debugSampling[1].info().revision, stopped);
        QTRY_COMPARE(scope->samples().back().debugLoop, quint8(0));
        QVERIFY(scope->visible[0] && scope->visible[1]);
        w.close();
    }
    void verticalWheelScalesEveryVisibleTrace() {
        Scope scope;
        scope.resize(900, 600);
        scope.visible.fill(false);
        scope.visible[0] = scope.visible[1] = scope.visible[11] = true;
        std::vector<Sample> samples;
        for (int i = 0; i < 1000; ++i) {
            Sample s;
            s.time = i * .005;
            s.values[0] = (i % 100) * .01;
            s.values[1] = (i % 130) * .03;
            s.values[11] = .25;
            samples.push_back(s);
        }
        scope.setSamples(samples);
        auto before = scope.grab().toImage();
        QPointF point(30, 200); // Wheel over Y ticks, without modifiers.
        QWheelEvent event(point, scope.mapToGlobal(point.toPoint()), {}, QPoint(0, 120), Qt::NoButton,
                          Qt::NoModifier, Qt::NoScrollPhase, false);
        QApplication::sendEvent(&scope, &event);
        auto after = scope.grab().toImage();
        for (int channel : {0, 1, 11}) {
            int pixelsBefore = 0, changed = 0;
            QRgb color = scope.colors[channel].rgb();
            for (int y = 50; y < 435; ++y)
                for (int x = 90; x < 875; ++x) {
                    bool a = before.pixel(x, y) == color, b = after.pixel(x, y) == color;
                    pixelsBefore += a;
                    changed += a != b;
                }
            QVERIFY(pixelsBefore > 20);
            QVERIFY2(changed > 20, qPrintable(QString("Channel %1 did not zoom").arg(channel)));
        }
        QCOMPARE(scope.windowSeconds, 5.0);
        QVERIFY(scope.follow);
        scope.resetView();
        QCOMPARE(scope.grab().toImage(), before);
    }
    void verticalWheelAndClearHistory() {
        Window w(false, false, true);
        w.show();
        for (int i = 0; i < 1000; ++i) {
            Sample s;
            s.time = i * .001;
            s.values[0] = i * .001;
            w.session.sampling.append(s);
        }
        auto scope = w.findChild<Scope *>();
        QVERIFY(scope);
        scope->refresh();
        auto before = scope->grab().toImage();
        double seconds = scope->windowSeconds;
        QPointF point(scope->width() / 2.0, scope->height() / 2.0);
        QWheelEvent vertical(point, scope->mapToGlobal(point.toPoint()), {}, QPoint(0, 120), Qt::NoButton,
                             Qt::ControlModifier, Qt::NoScrollPhase, false);
        QApplication::sendEvent(scope, &vertical);
        QCOMPARE(scope->windowSeconds, seconds);
        QVERIFY(scope->follow);
        auto zoomed = scope->grab().toImage();
        QVERIFY(zoomed != before);
        scope->resetView();
        QCOMPARE(scope->grab().toImage(), before);
        QWheelEvent horizontal(point, scope->mapToGlobal(point.toPoint()), {}, QPoint(0, 120), Qt::NoButton,
                               Qt::NoModifier, Qt::NoScrollPhase, false);
        QApplication::sendEvent(scope, &horizontal);
        QVERIFY(scope->windowSeconds < seconds);
        scope->setDisplayPaused(true);
        auto generation = w.session.sampling.info().generation;
        auto clear = w.findChild<QPushButton *>("clearWaveData");
        QVERIFY(clear);
        clear->click();
        QCOMPARE(w.session.sampling.info().size, size_t(0));
        QVERIFY(w.session.sampling.info().generation > generation);
        QVERIFY(scope->samples().empty());
        QVERIFY(w.session.sampling.capture().empty());
        QVERIFY(w.session.sampling.faultCapture().empty());
        w.close();
    }
    void resetCurrentMotorWithoutReenable() {
        Window w(false, false, true);
        w.show();
        auto reset = w.findChild<QPushButton *>("faultReset");
        QVERIFY(reset);
        QVERIFY(!reset->isEnabled());
        w.startDemo();
        QTRY_VERIFY(w.session.telemetryFresh(1));
        w.session.command(1, 0x41, QByteArray(1, 1));
        QTRY_COMPARE(w.session.latest(1)->mode, 1);
        w.session.command(1, 0x40, QByteArray(1, 1));
        QTRY_VERIFY(w.session.latest(1)->enabled);
        QTRY_VERIFY(reset->isEnabled());
        reset->click();
        QTRY_VERIFY(!w.session.latest(1)->enabled);
        QTRY_VERIFY(reset->isEnabled());
        QVERIFY(w.findChild<NoticeBar *>("operationNotice")->isHidden());
        w.close();
    }
    void stepListSelectionChangesDetails() {
        Window w(false, false, true);
        w.show();
        double actual = 0;
        for (int i = 0; i < 2000; ++i) {
            Sample s;
            s.time = i * .001;
            s.values[11] = i < 300 ? 0 : i < 600 ? .2 : i < 900 ? 0 : i < 1200 ? -.2 : 0;
            actual += (s.values[11] - actual) * .05;
            s.values[0] = actual;
            w.session.sampling.append(s);
        }
        w.findChild<QListWidget *>("navigation")->setCurrentRow(3);
        QPushButton *analyze = nullptr;
        for (auto b : w.findChildren<QPushButton *>())
            if (b->text() == "阶跃响应")
                analyze = b;
        QVERIFY(analyze);
        auto table = w.findChild<QTableWidget *>("stepList");
        auto details = w.findChild<QPlainTextEdit *>("stepDetails");
        QVERIFY(table && details);
        analyze->click();
        QTRY_COMPARE(table->rowCount(), 4);
        QTRY_VERIFY(analyze->isEnabled());
        table->selectRow(2);
        auto row = QJsonDocument::fromJson(details->toPlainText().toUtf8()).object();
        QCOMPARE(row["target"].toDouble(), -.2);
        QCOMPARE(row["index"].toInt(), 3);
        table->selectRow(0);
        row = QJsonDocument::fromJson(details->toPlainText().toUtf8()).object();
        QCOMPARE(row["target"].toDouble(), .2);
        QCOMPARE(row["index"].toInt(), 1);
        w.close();
    }
    void targetChannelsWithDemo() {
        Window w(false, false, true);
        w.resize(1600, 1000);
        w.show();
        w.startDemo();
        QTRY_VERIFY(w.session.telemetryFresh(1));
        w.session.command(1, 0x41, QByteArray(1, 1));
        QTRY_COMPARE(w.session.latest(1)->mode, 1);
        w.session.command(1, 0x40, QByteArray(1, 1));
        QTRY_VERIFY(w.session.latest(1)->enabled);
        w.session.command(1, 0x30, combinedTarget(1, 0x30, 1.25, 12, .1, Version::V2));
        QTRY_COMPARE(w.session.latest(1)->values[9], 1.25);
        QTRY_COMPARE(w.session.latest(1)->values[10], 12.0);
        QTRY_COMPARE(w.session.latest(1)->values[11], .1);
        int selected = 0;
        for (auto box : w.findChildren<QCheckBox *>())
            if (box->text().startsWith("目标") && box->text().contains(" · ")) {
                box->setChecked(true);
                ++selected;
            }
        QCOMPARE(selected, 3);
        auto scope = w.findChild<Scope *>();
        QVERIFY(scope);
        QVERIFY(scope->visible[9] && scope->visible[10] && scope->visible[11]);
        QTest::qWait(150);
        bool readout = false;
        for (auto label : w.findChildren<QLabel *>())
            readout |= label->text().contains("已提交目标") && label->text().contains("1.250");
        QVERIFY(readout);
        const auto screenshot = qEnvironmentVariable("MOTORTUNE_TARGET_SCREENSHOT");
        if (!screenshot.isEmpty())
            QVERIFY(w.grab().save(screenshot));
        w.close();
    }
    void errorsAreVisibleOnCurrentPage() {
        Window w(false, false, true);
        w.show();
        auto notice = w.findChild<NoticeBar *>("operationNotice");
        auto text = w.findChild<QLabel *>("operationNoticeText");
        QVERIFY(notice);
        w.session.connectionChanged(false, "设备打开失败：其他软件占用");
        QVERIFY(notice->isVisible());
        QVERIFY(text->text().contains("占用"));
        auto navigation = w.findChild<QListWidget *>("navigation");
        navigation->setCurrentRow(3);
        QVERIFY(notice->isVisible());
        notice->post("INFO", "已断开");
        QVERIFY(text->text().contains("占用"));
        w.session.commandFailed(999, "Node 1 · 失能未确认：设备仍为使能");
        QVERIFY(text->text().contains("失能未确认"));
        QVERIFY(notice->isVisible());
        notice->findChild<QPushButton *>("dismissNotice")->click();
        QVERIFY(notice->isHidden());
        w.close();
    }
    void normalOperationsDoNotOpenNotice() {
        NoticeBar notice;
        notice.post("INFO", "正在连接", true);
        QVERIFY(notice.isHidden());
        notice.post("SUCCESS", "连接成功", true);
        QVERIFY(notice.isHidden());
        notice.post("ERROR", "使能未确认");
        QVERIFY(!notice.isHidden());
        notice.post("SUCCESS", "其他操作成功", true);
        QVERIFY(!notice.isHidden());
        notice.post("INFO", "用户重新发起操作", true);
        QVERIFY(notice.isHidden());
    }
    void disableShowsPendingOnButtonWithoutNotice() {
        Window w(false, false, true);
        w.show();
        w.startDemo();
        QTRY_VERIFY(w.session.telemetryFresh(1));
        w.session.command(1, 0x41, QByteArray(1, 1));
        QTRY_COMPARE(w.session.latest(1)->mode, 1);
        w.session.command(1, 0x40, QByteArray(1, 1));
        QTRY_VERIFY(w.session.latest(1)->enabled);
        QPushButton *disable = nullptr;
        for (auto button : w.findChildren<QPushButton *>())
            if (button->text() == "失能")
                disable = button;
        QVERIFY(disable);
        QTRY_VERIFY(disable->isEnabled());
        disable->click();
        auto notice = w.findChild<NoticeBar *>("operationNotice");
        QVERIFY(notice->isHidden());
        QTRY_VERIFY(!w.session.latest(1)->enabled);
        QTRY_COMPARE(disable->text(), QString("失能"));
        QVERIFY(notice->isHidden());
        w.close();
    }
};
QTEST_MAIN(OperationUiTests)
#include "operation_ui_tests.moc"
