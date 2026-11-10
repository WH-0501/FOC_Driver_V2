#include "archive.h"
#include <QDateTime>
#include <QJsonArray>
#include <QJsonDocument>
#include <QSqlError>
#include <QSqlQuery>
#include <QUuid>
namespace mt {
Archive::Archive() {
    db_ = QSqlDatabase::addDatabase("QSQLITE", QUuid::createUuid().toString());
}
Archive::~Archive() {
    QString n = db_.connectionName();
    db_.close();
    db_ = QSqlDatabase();
    QSqlDatabase::removeDatabase(n);
}
bool Archive::openReadOnly(const QString &path, QString *error) {
    db_.setConnectOptions("QSQLITE_OPEN_READONLY");
    db_.setDatabaseName(path);
    if (!db_.open()) {
        *error = db_.lastError().text();
        return false;
    }
    QSqlQuery q(db_);
    if (!q.exec("SELECT schema_version FROM meta") || !q.next() || q.value(0).toInt() != 1) {
        *error = "不支持的归档 schema";
        db_.close();
        return false;
    }
    return true;
}
bool Archive::reportPath(qint64 id, const QString &path, QString *error) {
    QSqlQuery q(db_);
    q.prepare("UPDATE calib_record SET report_path=? WHERE id=?");
    q.addBindValue(path);
    q.addBindValue(id);
    if (!q.exec()) {
        *error = q.lastError().text();
        return false;
    }
    return true;
}
bool Archive::open(const QString &path, QString *error) {
    db_.setDatabaseName(path);
    if (!db_.open()) {
        *error = db_.lastError().text();
        return false;
    }
    QStringList sql = {
        "PRAGMA foreign_keys=ON",
        "PRAGMA journal_mode=WAL",
        "CREATE TABLE IF NOT EXISTS meta(schema_version INTEGER,created_at TEXT)",
        "INSERT INTO meta SELECT 1,datetime('now') WHERE NOT EXISTS(SELECT 1 FROM meta)",
        "CREATE TABLE IF NOT EXISTS device(sn TEXT PRIMARY KEY,node_id INTEGER,motor_type TEXT,first_seen_at "
        "TEXT,last_calib_id INTEGER)",
        "CREATE TABLE IF NOT EXISTS calib_record(id INTEGER PRIMARY KEY,sn TEXT REFERENCES "
        "device(sn),version INTEGER,is_current INTEGER,status TEXT,snapshot TEXT,readback TEXT,verified "
        "INTEGER DEFAULT 0,app_version TEXT,created_at TEXT,demo INTEGER,report_path "
        "TEXT,UNIQUE(sn,version))",
        "CREATE TABLE IF NOT EXISTS test_record(id INTEGER PRIMARY KEY,calib_id INTEGER REFERENCES "
        "calib_record(id),suite TEXT,detail TEXT,created_at TEXT)",
        "CREATE UNIQUE INDEX IF NOT EXISTS one_current ON calib_record(sn) WHERE is_current=1"};
    for (auto &s : sql) {
        QSqlQuery q(db_);
        if (!q.exec(s)) {
            *error = q.lastError().text();
            return false;
        }
    }
    return true;
}
qint64 Archive::save(const QString &sn, int node, const QJsonObject &snap, bool demo, QString *error) {
    if (sn.trimmed().isEmpty()) {
        *error = "SN 不能为空";
        return -1;
    }
    if (!db_.transaction()) {
        *error = db_.lastError().text();
        return -1;
    }
    QSqlQuery q(db_);
    auto run = [&](const QString &sql, const QVariantList &args) {
        q.prepare(sql);
        for (auto &a : args)
            q.addBindValue(a);
        if (!q.exec()) {
            *error = q.lastError().text();
            return false;
        }
        return true;
    };
    QString time = QDateTime::currentDateTimeUtc().toString(Qt::ISODate);
    bool ok =
        run("INSERT INTO device(sn,node_id,first_seen_at) VALUES(?,?,?) ON CONFLICT(sn) DO UPDATE SET "
            "node_id=excluded.node_id",
            {sn, node, time}) &&
        run("UPDATE calib_record SET is_current=0 WHERE sn=?", {sn}) &&
        run("INSERT INTO calib_record(sn,version,is_current,status,snapshot,app_version,created_at,demo) "
            "SELECT ?,COALESCE(MAX(version),0)+1,1,'saved',?,?,?,? FROM calib_record WHERE sn=?",
            {sn, QString::fromUtf8(QJsonDocument(snap).toJson(QJsonDocument::Compact)), MOTORTUNE_VERSION,
             time, demo, sn});
    qint64 id = ok ? q.lastInsertId().toLongLong() : -1;
    if (ok)
        ok = run("UPDATE device SET last_calib_id=? WHERE sn=?", {id, sn});
    if (ok && db_.commit())
        return id;
    db_.rollback();
    return -1;
}
bool Archive::test(qint64 id, const QString &suite, const QJsonObject &details, QString *error) {
    QSqlQuery q(db_);
    q.prepare("INSERT INTO test_record(calib_id,suite,detail,created_at) VALUES(?,?,?,?)");
    q.addBindValue(id);
    q.addBindValue(suite);
    q.addBindValue(QString::fromUtf8(QJsonDocument(details).toJson()));
    q.addBindValue(QDateTime::currentDateTimeUtc().toString(Qt::ISODate));
    if (!q.exec()) {
        *error = q.lastError().text();
        return false;
    }
    return true;
}
bool Archive::verify(qint64 id, const QJsonObject &actual, QString *error) {
    QSqlQuery q(db_);
    q.prepare("SELECT snapshot FROM calib_record WHERE id=?");
    q.addBindValue(id);
    if (!q.exec() || !q.next()) {
        *error = "找不到标定记录";
        return false;
    }
    auto expected = QJsonDocument::fromJson(q.value(0).toString().toUtf8()).object();
    bool match = expected == actual;
    q.prepare("UPDATE calib_record SET readback=?,verified=?,status=? WHERE id=?");
    q.addBindValue(QString::fromUtf8(QJsonDocument(actual).toJson()));
    q.addBindValue(match);
    q.addBindValue(match ? "verified" : "mismatch");
    q.addBindValue(id);
    if (!q.exec()) {
        *error = q.lastError().text();
        return false;
    }
    if (!match)
        *error = "掉电回读与保存快照不一致";
    return match;
}
QJsonArray Archive::search(const QString &sn) const {
    QSqlQuery q(db_);
    q.prepare("SELECT c.id,c.sn,c.version,c.status,c.created_at,c.demo,c.verified,c.snapshot FROM "
              "calib_record c WHERE c.sn LIKE ? ORDER BY c.id DESC LIMIT 500");
    q.addBindValue("%" + sn + "%");
    QJsonArray out;
    if (q.exec())
        while (q.next()) {
            QJsonObject o;
            QStringList k = {"id", "sn", "version", "status", "created_at", "demo", "verified", "snapshot"};
            for (int i = 0; i < k.size(); i++)
                o[k[i]] = QJsonValue::fromVariant(q.value(i));
            out.append(o);
        }
    return out;
}
} // namespace mt
