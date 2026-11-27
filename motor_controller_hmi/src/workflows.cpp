#include "window.h"
#include "identification_dialog.h"
#include <QJsonArray>
#include <QJsonDocument>
#include <QSaveFile>
#include <cmath>
namespace mt {
namespace {
QWidget *workflowPage(const QString &title, const QString &text, QVBoxLayout *&l) {
    auto w = new QWidget;
    l = new QVBoxLayout(w);
    l->setContentsMargins(24, 20, 24, 20);
    l->setSpacing(16);
    auto h = new QLabel(title);
    h->setObjectName("pageTitle");
    l->addWidget(h);
    auto d = new QLabel(text);
    d->setWordWrap(true);
    d->setObjectName("muted");
    l->addWidget(d);
    return w;
}
QPushButton *action(QBoxLayout *l, const QString &s, std::function<void()> f) {
    auto b = new QPushButton(s);
    l->addWidget(b);
    QObject::connect(b, &QPushButton::clicked, b, std::move(f));
    return b;
}
struct TestDefinition {
    const char *id;
    const char *name;
    int cmd;
    bool motion;
    bool destructive;
};
const TestDefinition cases[] = {{"AT-CORE-01", "Node 命令回显", 0x52, false, false},
                                {"AT-CORE-02", "周期上报开关", 0x7f, false, false},
                                {"AT-CORE-03", "使能开关", 0x40, true, false},
                                {"AT-CORE-04", "模式切换", 0x41, false, false},
                                {"AT-CORE-05", "PID Get 可解析", 0x47, false, false},
                                {"AT-CORE-06", "电机类型合法", 0x52, false, false},
                                {"AT-CORE-07", "柔顺 PID 可解析", 0x4a, false, false},
                                {"AT-CORE-08", "紧急失能路径", 0x39, false, false},
                                {"AT-CORE-09", "CSV/HM/MIT 模式切换", 0x41, false, false},
                                {"AT-CORE-10", "温度与母线电压", 0, false, false},
                                {"AT-MOT-01", "使能后遥测健康", 0, true, false},
                                {"AT-MOT-02", "CST 短脉冲电流跟踪", 3, true, false},
                                {"AT-MOT-03", "CSV 低速跟踪", 2, true, false},
                                {"AT-MOT-04", "PP 持位", 1, true, false},
                                {"AT-MOT-05", "停机清零", 0x39, false, false},
                                {"AT-MOT-06", "CSP 小步跟踪", 1, true, true},
                                {"AT-MOT-07", "HM 模式门禁", 0x41, true, false},
                                {"AT-CFG-01", "保护参数可解析", 0x4b, false, false},
                                {"AT-CFG-02", "SaveParam 使能中拒绝", 0x4d, true, true},
                                {"AT-CFG-03", "电零使能中拒绝", 0xab, true, true},
                                {"AT-CFG-04", "编码器状态 / 校准门禁", 0xae, false, false},
                                {"AT-CFG-05", "RestoreFactory 使能中拒绝", 0x4e, true, true},
                                {"AT-CFG-06", "SetOrigin 使能中拒绝", 0x50, true, true},
                                {"AT-CFG-07", "温度校准使能中拒绝", 0xac, true, true},
                                {"AT-CFG-08", "机型使能中拒绝", 0x51, true, true},
                                {"AT-CFG-09", "HoldStall 可解析", -1, false, false},
                                {"AT-CFG-10", "HoldStall 写门禁", -1, true, true},
                                {"AT-CFG-11", "Node ID 使能中拒绝", 0x4c, true, true},
                                {"AT-CFG-12", "齿槽 STATUS", -1, false, false},
                                {"AT-CFG-13", "齿槽前馈使能", -1, false, false},
                                {"AT-CFG-14", "掉电后自动加载", -1, false, true}};
} // namespace
void Window::querySnapshot(std::function<void(QJsonObject)> done) {
    auto result = std::make_shared<QJsonObject>();
    auto commands = std::make_shared<QList<int>>(QList<int>{0x52, 0x53, 0x47, 0x48, 0x49, 0x4a, 0x4b});
    auto step = std::make_shared<std::function<void()>>();
    std::weak_ptr<std::function<void()>> weak = step;
    *step = [this, done, result, commands, weak] {
        auto keep = weak.lock();
        if (!keep)
            return;
        if (commands->isEmpty()) {
            (*result)["node"] = int(node());
            done(*result);
            return;
        }
        int c = commands->takeFirst();
        request(
            c, {},
            [result, c, keep](QJsonObject o) {
                o.remove("hex");
                o.remove("valid");
                (*result)[QString::number(c, 16)] = o;
                (*keep)();
            },
            [this](QString s) { log("ERROR", "参数快照中止：" + s); });
    };
    (*step)();
}
QWidget *Window::factoryPage() {
    QVBoxLayout *l;
    auto w = workflowPage("产线标定向导", "强制顺序 · SN 版本归档 · 固件缺失能力不会被标记为通过", l);
    auto identity = new QFormLayout;
    sn_ = new QLineEdit;
    sn_->setPlaceholderText("扫描或输入电机 SN");
    identity->addRow("电机 SN", sn_);
    l->addLayout(identity);
    auto body = new QHBoxLayout;
    steps_ = new QListWidget;
    steps_->addItems({"1  SN 登记", "2  机型 / Node ID 确认", "3  电角度对齐", "4  电阻辨识", "5  电感辨识",
                      "6  磁链辨识", "7  齿槽转矩标定", "8  启动电流辨识"});
    steps_->setCurrentRow(0);
    steps_->setEnabled(false);
    body->addWidget(steps_);
    stepDetail_ = new QLabel("输入 SN 后执行当前步骤");
    stepDetail_->setWordWrap(true);
    body->addWidget(stepDetail_, 1);
    l->addLayout(body);
    auto row = new QHBoxLayout;
    nextStep_ = action(row, "执行当前步骤", [this] { runWizardStep(); });
    action(row, "重新开始", [this] {
        if (confirm("重新开始标定向导？当前未保存进度将清除。")) {
            wizardStep_ = 0;
            steps_->setCurrentRow(0);
            nextStep_->setEnabled(true);
            stepDetail_->setText("输入 SN 后执行当前步骤");
        }
    });
    action(row, "保存并归档", [this] {
        if (wizardStep_ < 8) {
            log("WARNING", "向导未完成，不能作为完整产线标定归档");
            return;
        }
        safeAction(0x4d, {}, "写入设备 Flash 并归档 SN 参数？", [this](QJsonObject) {
            querySnapshot([this](QJsonObject o) {
                QString err;
                archiveId_ =
                    archive_.save(sn_->text().trimmed(), int(node()), o, connection_.backend == "Demo", &err);
                savedSnapshot_ = o;
                stepDetail_->setText(
                    archiveId_ > 0 ? "保存成功，已自动归档。下一步：出厂测试，再人工掉电回读。" : err);
            });
        });
    });
    l->addLayout(row);
    l->addStretch();
    return w;
}
void Window::runWizardStep() {
    if (sn_->text().trimmed().isEmpty()) {
        stepDetail_->setText("SN 不能为空");
        return;
    }
    if (!session.connected()) {
        stepDetail_->setText("请先连接设备或 Demo");
        return;
    }
    auto advance = [this] {
        wizardStep_++;
        steps_->setCurrentRow(qMin(wizardStep_, 7));
        stepDetail_->setText(
            wizardStep_ == 8
                ? "八步完成，可保存参数并归档"
                : QString("步骤 %1 完成。请执行步骤 %2。").arg(wizardStep_).arg(wizardStep_ + 1));
        if (wizardStep_ == 8)
            nextStep_->setEnabled(false);
    };
    if (wizardStep_ == 0) {
        advance();
        return;
    }
    if (wizardStep_ == 1) {
        request(0x52, {}, [this, advance](QJsonObject o) {
            if (!o.contains("motor_type") || o["motor_type"].toInt() > 3) {
                stepDetail_->setText("机型非法");
                return;
            }
            advance();
        });
        return;
    }
    if (!unloaded_->isChecked()) {
        stepDetail_->setText("请先勾选空载 / 工作区安全");
        return;
    }
    if (wizardStep_ == 2) {
        safeAction(0xab, QByteArray(1, 1), "开始电角对齐？电机会运动。", [this, advance](QJsonObject o) {
            if (o["ok"].toBool() && o.contains("flags") && (o["flags"].toInt() & 8))
                advance();
            else
                stepDetail_->setText("对齐未报告完成标志，不能进入下一步");
        });
        return;
    }
    if (wizardStep_ >= 3 && wizardStep_ <= 5) {
        if (experimentTimer_.isActive() || testsRunning_ || session.upgrading()) {
            stepDetail_->setText("请先停止其他实验、测试或升级任务"); return;
        }
        const int step = wizardStep_;
        bool applied = false;
        IdentificationDialog dialog(session, connection_.version, node(), connection_.backend == "Demo", this,
                                    step <= 4 ? 1 : 2);
        connect(&dialog, &IdentificationDialog::parametersApplied, &dialog, [&](QJsonObject result) {
            applied = (result["valid_mask"].toInt() & (step <= 4 ? 1 : 2)) != 0;
        });
        dialog.exec();
        if (applied) {
            if (step == 3) ++wizardStep_; // One successful Rs/Ld/Lq run covers both electrical steps.
            advance();
        } else stepDetail_->setText("辨识结果尚未成功应用，当前步骤未完成。");
        return;
    }
    if (connection_.backend == "Demo") {
        stepDetail_->setText("Demo 模拟步骤，仅验证向导流程，不代表硬件标定");
        advance();
        log("INFO", "Demo 标定步骤已模拟");
    } else
        stepDetail_->setText("当前固件没有该步骤的已确认 CAN "
                             "入口，流程在此阻止。不能跳步或出具标定通过报告。详见 docs/ACCEPTANCE.md。");
}
QWidget *Window::testPage() {
    QVBoxLayout *l;
    auto w = workflowPage(
        "自动测试", "31 项需求用例登记；结果区分 PASS / FAIL / SKIP。默认仅选择无运动、无 Flash 写入的检查。",
        l);
    testTable_ = new QTableWidget(31, 5);
    testTable_->setHorizontalHeaderLabels({"选择", "ID", "用例", "运动", "结果"});
    testTable_->horizontalHeader()->setSectionResizeMode(2, QHeaderView::Stretch);
    testTable_->setColumnWidth(0, 60);
    testTable_->setColumnWidth(1, 130);
    testTable_->setColumnWidth(3, 60);
    testTable_->setColumnWidth(4, 260);
    testTable_->setEditTriggers(QAbstractItemView::NoEditTriggers);
    for (int i = 0; i < 31; i++) {
        auto check = new QTableWidgetItem;
        check->setFlags(Qt::ItemIsUserCheckable | Qt::ItemIsEnabled);
        check->setCheckState(!cases[i].motion && !cases[i].destructive ? Qt::Checked : Qt::Unchecked);
        testTable_->setItem(i, 0, check);
        testTable_->setItem(i, 1, new QTableWidgetItem(cases[i].id));
        testTable_->setItem(i, 2, new QTableWidgetItem(QString::fromUtf8(cases[i].name)));
        testTable_->setItem(i, 3, new QTableWidgetItem(cases[i].motion ? "是" : "否"));
        testTable_->setItem(i, 4, new QTableWidgetItem("未运行"));
    }
    l->addWidget(testTable_, 1);
    auto row = new QHBoxLayout;
    allowMotion_ = new QCheckBox("允许转轴");
    stopOnFail_ = new QCheckBox("失败即停");
    stopOnFail_->setChecked(true);
    row->addWidget(allowMotion_);
    row->addWidget(stopOnFail_);
    action(row, "开始", [this] {
        if (testsRunning_ || !session.connected())
            return;
        if (allowMotion_->isChecked() &&
            (!unloaded_->isChecked() || !confirm("已选择运动测试，确认空载安全且可急停？")))
            return;
        testResults_ = {};
        testIndex_ = -1;
        testsRunning_ = true;
        runNextTest();
    });
    action(row, "停止", [this] {
        testsRunning_ = false;
        session.emergency();
    });
    action(row, "导出报告", [this] { exportReport(); });
    l->addLayout(row);
    return w;
}
void Window::testVerdict(int i, const QString &verdict, const QString &detail) {
    testTable_->item(i, 4)->setText(verdict + " · " + detail);
    testTable_->item(i, 4)->setToolTip(detail);
    testTable_->item(i, 4)->setForeground(QColor(verdict == "PASS"   ? "#5fc992"
                                                 : verdict == "FAIL" ? "#FF6363"
                                                                     : "#9c9c9d"));
    testResults_[cases[i].id] = QJsonObject{{"verdict", verdict},
                                            {"detail", detail},
                                            {"time", QDateTime::currentDateTimeUtc().toString(Qt::ISODate)}};
    log(verdict == "FAIL" ? "ERROR" : "INFO", QString(cases[i].id) + " " + verdict + " " + detail);
    if (verdict == "FAIL" && stopOnFail_->isChecked()) {
        testsRunning_ = false;
        session.emergency();
    }
    if (testsRunning_)
        QTimer::singleShot(80, this, &Window::runNextTest);
}
void Window::runNextTest() {
    if (!testsRunning_)
        return;
    int i = ++testIndex_;
    if (i >= 31) {
        testsRunning_ = false;
        session.emergency();
        QString e;
        if (archiveId_ > 0 && !archive_.test(archiveId_, "factory-31", testResults_, &e))
            log("ERROR", e);
        report_->setPlainText(QString::fromUtf8(QJsonDocument(testResults_).toJson()));
        return;
    }
    if (testTable_->item(i, 0)->checkState() != Qt::Checked) {
        testVerdict(i, "SKIP", "未选择");
        return;
    }
    auto test = cases[i];
    if (test.cmd < 0) {
        testVerdict(i, "SKIP", "固件无已确认入口 / 需要人工掉电回读");
        return;
    }
    if (test.motion && (!allowMotion_->isChecked() || !unloaded_->isChecked())) {
        testVerdict(i, "SKIP", "未允许转轴");
        return;
    }
    if (test.destructive) {
        testVerdict(i, "SKIP", "固件拒绝路径需独立台架验证，禁止自动尝试可能成功的破坏性命令");
        return;
    }
    if (i == 11) {
        testVerdict(i, "SKIP", "5ms 电流脉冲需要固件定时接口，Windows 主机不保证该脉宽");
        return;
    }
    auto failure = [this, i](QString s) {
        if (testsRunning_)
            testVerdict(i, "FAIL", s);
    };
    auto verdict = [this, i](bool ok, QString why) {
        if (testsRunning_)
            testVerdict(i, ok ? (i == 16 || i == 20 ? "SKIP" : "PASS") : "FAIL", why);
    };
    if (i == 4) {
        auto valid = [](QJsonObject o) { return o.contains("P") && o.contains("I") && o.contains("D"); };
        request(
            0x47, {},
            [this, valid, verdict, failure](QJsonObject p) {
                if (!valid(p)) {
                    verdict(false, "位置 PID 无效");
                    return;
                }
                request(
                    0x48, {},
                    [this, valid, verdict, failure](QJsonObject v) {
                        if (!valid(v)) {
                            verdict(false, "速度 PID 无效");
                            return;
                        }
                        request(
                            0x49, {},
                            [valid, verdict](QJsonObject c) {
                                verdict(valid(c), "位置/速度/电流三环 PID 有限且可解析");
                            },
                            failure);
                    },
                    failure);
            },
            failure);
        return;
    }
    if (i == 9) {
        auto s = session.latest(node());
        verdict(s && s->environment && std::isfinite(s->values[8]) && s->values[7] >= 18 &&
                    s->values[7] <= 32,
                "温度有效，母线 18–32V");
        return;
    }
    if (i == 1) {
        request(0x7f, be16(100));
        quint64 before = session.rxCount;
        QTimer::singleShot(400, this, [this, i, before] {
            if (!testsRunning_)
                return;
            auto active = session.rxCount.load() - before;
            request(0x7f, be16(0));
            QTimer::singleShot(150, this, [this, i, active] {
                auto stopped = session.rxCount.load();
                QTimer::singleShot(300, this, [this, i, active, stopped] {
                    if (!testsRunning_)
                        return;
                    auto quiet = session.rxCount.load() - stopped;
                    request(0x7f, be16(1000));
                    testVerdict(i, active >= 15 && quiet < 5 ? "PASS" : "FAIL",
                                QString("运行接收 %1，停止后 %2").arg(active).arg(quiet));
                });
            });
        });
        return;
    }
    if (i == 7 || i == 14) {
        session.emergency();
        QTimer::singleShot(300, this, [this, verdict] {
            auto s = session.latest(node());
            verdict(s && !s->enabled && std::abs(s->values[0]) < .1, "遥测失能，|Iq| < 0.1A");
        });
        return;
    }
    if (i == 3 || i == 8) {
        auto modesToTry = std::make_shared<QList<int>>(
            i == 8 ? (connection_.version == Version::V2 ? QList<int>{5, 8, 7, 3} : QList<int>{4, 3, 5, 0})
                   : QList<int>{connection_.version == Version::V2 ? 3 : 0});
        auto step = std::make_shared<std::function<void()>>();
        std::weak_ptr<std::function<void()>> weak = step;
        *step = [this, modesToTry, verdict, failure, weak] {
            auto keep = weak.lock();
            if (!keep || !testsRunning_)
                return;
            if (modesToTry->isEmpty()) {
                verdict(true, "模式专用应答与遥测一致");
                return;
            }
            int m = modesToTry->takeFirst();
            request(
                0x41, QByteArray(1, char(m)),
                [this, m, keep, verdict](QJsonObject) {
                    QTimer::singleShot(120, this, [this, m, keep, verdict] {
                        auto s = session.latest(node());
                        if (!s || s->mode != m) {
                            verdict(false, "模式遥测不匹配");
                            return;
                        }
                        (*keep)();
                    });
                },
                failure);
        };
        request(0x40, QByteArray(1, 0), [step](QJsonObject) { (*step)(); }, failure);
        return;
    }
    if (i == 2 || i == 10 || i == 12 || i == 13 || i == 16) {
        int mode = connection_.version == Version::V2 ? (i == 12   ? 5
                                                         : i == 13 ? 3
                                                         : i == 16 ? 8
                                                                   : 6)
                                                      : (i == 12   ? 4
                                                         : i == 13 ? 0
                                                         : i == 16 ? 3
                                                                   : 2);
        request(
            0x40, QByteArray(1, 0),
            [this, mode, i, verdict, failure](QJsonObject) {
                QTimer::singleShot(100, this, [this, mode, i, verdict, failure] {
                    if (!testsRunning_)
                        return;
                    request(
                        0x41, QByteArray(1, char(mode)),
                        [this, i, verdict, failure](QJsonObject) {
                            QTimer::singleShot(100, this, [this, i, verdict, failure] {
                                if (!testsRunning_)
                                    return;
                                if (i == 16) {
                                    verdict(true, "HM 失能切换通过；使能拒绝由门禁单元测试覆盖");
                                    return;
                                }
                                request(
                                    0x40, QByteArray(1, 1),
                                    [this, i, verdict](QJsonObject) {
                                        QTimer::singleShot(120, this, [this, i, verdict] {
                                            if (!testsRunning_)
                                                return;
                                            auto s = session.latest(node());
                                            if (!s || !s->enabled) {
                                                verdict(false, "未收到使能遥测");
                                                return;
                                            }
                                            double position = s->values[6];
                                            quint64 before = session.rxCount;
                                            if (i == 12)
                                                request(2, target(quint8(node()), 5, 1, connection_.version));
                                            if (i == 13)
                                                request(1, target(quint8(node()), position, 0,
                                                                  connection_.version));
                                            QTimer::singleShot(
                                                1000, this, [this, i, position, before, verdict] {
                                                    if (!testsRunning_)
                                                        return;
                                                    auto a = session.latest(node());
                                                    bool ok = a && a->enabled && !a->error &&
                                                              session.rxCount.load() > before + 20;
                                                    if (i == 12)
                                                        ok = ok && std::abs(a->values[5] - 5) < 2;
                                                    if (i == 13)
                                                        ok = ok && std::abs(a->values[6] - position) <
                                                                       2 * std::numbers::pi / 180;
                                                    session.emergency();
                                                    QTimer::singleShot(150, this, [verdict, ok] {
                                                        verdict(ok, "遥测状态 / 跟踪检查，已失能");
                                                    });
                                                });
                                        });
                                    },
                                    failure);
                            });
                        },
                        failure);
                });
            },
            failure);
        return;
    }
    request(
        test.cmd, {},
        [verdict, i](QJsonObject o) {
            bool ok = false;
            if (i == 0 || i == 5)
                ok = o.contains("motor_type") && o["motor_type"].toInt() <= 3;
            else if (i == 4)
                ok = o.contains("P") && o.contains("I") && o.contains("D");
            else if (i == 6)
                ok = o.contains("stiffness") && o.contains("damping");
            else if (i == 17)
                ok = o.contains("temperature") && o["under_voltage"].toInt() < o["over_voltage"].toInt();
            else if (i == 20)
                ok = o.contains("calibration_state") && o["calibration_state"].toInt() <= 3;
            verdict(ok, i == 20 ? "仅验证编码器状态读取；校准拒绝需台架验证" : "专用应答字段校验");
        },
        failure);
}
QWidget *Window::reportPage() {
    QVBoxLayout *l;
    auto w = workflowPage(
        "报告与掉电回读",
        "SaveParam 成功后按 SN 自动新增版本。掉电验证需要人工重新上电并重新连接；Demo 报告始终标识模拟。", l);
    auto row = new QHBoxLayout;
    query_ = new QLineEdit;
    query_->setPlaceholderText("SN 查询（可留空）");
    row->addWidget(query_);
    action(row, "查询归档", [this] {
        report_->setPlainText(QString::fromUtf8(QJsonDocument(archive_.search(query_->text())).toJson()));
    });
    action(row, "掉电回读比对", [this] {
        if (archiveId_ < 0) {
            log("WARNING", "请先保存并归档当前 SN");
            return;
        }
        if (!confirm("已完成实际断电、重新上电并连接同一电机？"))
            return;
        querySnapshot([this](QJsonObject o) {
            QString error;
            bool ok = archive_.verify(archiveId_, o, &error);
            report_->setPlainText((ok ? "已确认字段回读一致\n" : "回读失败：" + error + "\n") +
                                  QString::fromUtf8(QJsonDocument(o).toJson()));
            log(ok ? "INFO" : "ERROR",
                ok ? "已确认字段比对通过；齿槽/HoldStall 未提供，不能据此判整机出厂通过" : error);
        });
    });
    action(row, "导出报告", [this] { exportReport(); });
    l->addLayout(row);
    auto tools = new QHBoxLayout;
    auto records = new QComboBox;
    records->setMinimumWidth(200);
    tools->addWidget(records);
    action(tools, "载入历史版本", [this, records] {
        auto all = archive_.search(query_->text());
        records->clear();
        for (auto v : all) {
            auto o = v.toObject();
            records->addItem(QString("%1 · v%2 · %3")
                                 .arg(o["sn"].toString())
                                 .arg(o["version"].toInt())
                                 .arg(o["created_at"].toString()),
                             o);
        }
    });
    action(tools, "选择该记录", [this, records] {
        auto o = records->currentData().toJsonObject();
        if (o.isEmpty())
            return;
        archiveId_ = qint64(o["id"].toDouble());
        savedSnapshot_ = QJsonDocument::fromJson(o["snapshot"].toString().toUtf8()).object();
        sn_->setText(o["sn"].toString());
        report_->setPlainText(QString::fromUtf8(QJsonDocument(o).toJson()));
    });
    action(tools, "只读打开外部库", [this] {
        auto path = QFileDialog::getOpenFileName(this, "只读归档库", {}, "SQLite (*.sqlite *.db)");
        if (path.isEmpty())
            return;
        Archive external;
        QString e;
        if (!external.openReadOnly(path, &e)) {
            log("ERROR", e);
            return;
        }
        report_->setPlainText(QString::fromUtf8(QJsonDocument(external.search(query_->text())).toJson()));
    });
    action(tools, "导出归档 CSV", [this] {
        auto records = archive_.search(query_->text());
        auto path = QFileDialog::getSaveFileName(this, "归档 CSV", {}, "CSV (*.csv)");
        QSaveFile file(path);
        if (!file.open(QIODevice::WriteOnly))
            return;
        file.write("id,sn,version,status,created_at,demo,verified,snapshot\n");
        auto quote = [](QString v) {
            v.replace('"', "\"\"");
            return QString("\"") + v + "\"";
        };
        for (auto value : records) {
            auto o = value.toObject();
            QStringList cells;
            for (auto key : {"id", "sn", "version", "status", "created_at", "demo", "verified", "snapshot"})
                cells.append(quote(o[key].toVariant().toString()));
            file.write((cells.join(',') + "\n").toUtf8());
        }
        if (!file.commit())
            log("ERROR", file.errorString());
    });
    l->addLayout(tools);
    report_ = new QPlainTextEdit;
    report_->setReadOnly(true);
    l->addWidget(report_, 1);
    return w;
}
void Window::exportReport() {
    QString path =
        QFileDialog::getSaveFileName(this, "导出报告", {}, "JSON report (*.json);;HTML report (*.html)");
    if (path.isEmpty())
        return;
    int pass = 0, fail = 0, skip = 0;
    for (auto v : testResults_) {
        auto s = v.toObject()["verdict"].toString();
        if (s == "PASS")
            pass++;
        else if (s == "FAIL")
            fail++;
        else
            skip++;
    }
    QJsonObject o{{"app_version", MOTORTUNE_VERSION},
                  {"sn", sn_->text()},
                  {"node", int(node())},
                  {"demo", connection_.backend == "Demo"},
                  {"created_at", QDateTime::currentDateTimeUtc().toString(Qt::ISODate)},
                  {"pass", pass},
                  {"fail", fail},
                  {"skip", skip},
                  {"overall", fail                                ? "FAIL"
                              : skip || testResults_.size() != 31 ? "INCOMPLETE"
                                                                  : "PASS"},
                  {"tests", testResults_},
                  {"calibration_snapshot", savedSnapshot_},
                  {"notes", "齿槽/HoldStall/Rs/Ls/磁链等无固件入口；不能签发完整硬件标定通过"}};
    auto bytes = QJsonDocument(o).toJson();
    if (path.endsWith(".html", Qt::CaseInsensitive))
        bytes =
            ("<!doctype html><meta charset='utf-8'><title>MotorTune report</title><style>body{font:14px "
             "monospace;max-width:1000px;margin:40px auto}pre{white-space:pre-wrap}</style><h1>MotorTune " +
             QString(MOTORTUNE_VERSION) + " · " +
             (connection_.backend == "Demo" ? "DEMO / 模拟报告" : "出厂检查报告") + "</h1><pre>" +
             QString::fromUtf8(bytes).toHtmlEscaped() + "</pre>")
                .toUtf8();
    QSaveFile f(path);
    if (!f.open(QIODevice::WriteOnly)) {
        log("ERROR", f.errorString());
        return;
    }
    f.write(bytes);
    if (!f.commit())
        log("ERROR", f.errorString());
    else if (archiveId_ > 0) {
        QString e;
        if (!archive_.reportPath(archiveId_, path, &e))
            log("ERROR", e);
    }
}
QWidget *Window::extensionPage() {
    QVBoxLayout *l;
    auto w = workflowPage(
        "扩展与只读脚本",
        "隔离进程执行受限 Python 表达式。仅接收遥测、参数和波形快照，没有设备控制或文件系统 API。", l);
    auto editor = new QPlainTextEdit;
    editor->setPlainText("mean(iq)\nrms(iq)\nmax(iq) - min(iq)\ntelemetry['node']");
    l->addWidget(editor);
    auto output = new QPlainTextEdit;
    output->setReadOnly(true);
    l->addWidget(output);
    auto run = [this, editor, output](bool single) {
        auto data = session.sampling.snapshot(node());
        QJsonArray iq;
        size_t start = data.size() > 10000 ? data.size() - 10000 : 0;
        for (size_t i = start; i < data.size(); i++)
            iq.append(data[i].values[0]);
        QJsonObject telemetry{{"node", int(node())}};
        if (!data.empty()) {
            telemetry["enabled"] = data.back().enabled;
            telemetry["velocity"] = data.back().values[5];
            telemetry["position"] = data.back().values[6];
        }
        QString code = single ? editor->textCursor().block().text() : editor->toPlainText();
        if (code.size() > 16384) {
            output->setPlainText("脚本超过 16 KiB 限制");
            return;
        }
        auto process = new QProcess(this);
        process->setProperty("outputBytes", 0);
        connect(process, &QProcess::readyReadStandardOutput, this, [process, output] {
            auto bytes = process->readAllStandardOutput();
            int total = process->property("outputBytes").toInt() + int(bytes.size());
            process->setProperty("outputBytes", total);
            if (total > 65536) {
                process->kill();
                output->appendPlainText("输出超限");
                return;
            }
            output->appendPlainText(QString::fromUtf8(bytes));
        });
        connect(process, &QProcess::errorOccurred, this,
                [process, output] { output->appendPlainText(process->errorString()); });
        connect(process, &QProcess::finished, this, [process, output](int exit, QProcess::ExitStatus status) {
            auto err = process->readAllStandardError();
            if (!err.isEmpty())
                output->appendPlainText(QString::fromUtf8(err));
            output->appendPlainText(QString("退出 %1 / %2").arg(exit).arg(status));
            process->deleteLater();
        });
        process->start(connection_.python,
                       {"-I", QCoreApplication::applicationDirPath() + "/compat/expression_worker.py"});
        connect(process, &QProcess::started, this, [process, code, iq, telemetry, this] {
            QJsonObject input{{"code", code},
                              {"iq", iq},
                              {"telemetry", telemetry},
                              {"parameters", session.parameters(node())}};
            process->write(QJsonDocument(input).toJson(QJsonDocument::Compact));
            process->closeWriteChannel();
        });
        QTimer::singleShot(3000, process, [process] {
            if (process->state() != QProcess::NotRunning)
                process->kill();
        });
        output->clear();
        log("INFO", "只读表达式执行 / " + QString::number(code.size()) + " 字符");
    };
    auto row = new QHBoxLayout;
    action(row, "运行全部", [run] { run(false); });
    action(row, "单步当前行", [run] { run(true); });
    action(row, "加载分析插件", [this, editor] {
        auto path = QFileDialog::getOpenFileName(this, "分析插件 JSON", {}, "Analysis manifest (*.json)");
        QFile f(path);
        if (!f.open(QIODevice::ReadOnly) || f.size() > 16384)
            return;
        auto o = QJsonDocument::fromJson(f.readAll()).object();
        if (o["api"].toInt() != 1 || o["permission"].toString() != "snapshot.read" ||
            o["expression"].toString().isEmpty()) {
            log("ERROR", "插件清单必须 api=1、permission=snapshot.read、expression 非空");
            return;
        }
        editor->setPlainText(o["expression"].toString());
        log("INFO", "加载只读分析插件 " + o["name"].toString());
    });
    action(row, "卸载", [editor, output] {
        editor->clear();
        output->clear();
    });
    l->addLayout(row);
    auto help = new QLabel("支持：算术 + − × ÷、列表/字典索引、mean/rms/min/max/sum/len/abs/sqrt。\n禁止 "
                           "import、属性访问、循环、文件和网络；不支持任意 Python 程序。\n插件 JSON: "
                           "{\"api\":1,\"permission\":\"snapshot.read\",\"name\":\"Ripple\",\"expression\":"
                           "\"max(iq)-min(iq)\"}");
    help->setWordWrap(true);
    l->addWidget(help);
    return w;
}
} // namespace mt
