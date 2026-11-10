#pragma once
#include <QJsonObject>
#include <QSqlDatabase>
namespace mt {
class Archive {
  public:
    Archive();
    ~Archive();
    bool open(const QString &path, QString *error);
    qint64 save(const QString &sn, int node, const QJsonObject &snapshot, bool demo, QString *error);
    bool test(qint64 id, const QString &suite, const QJsonObject &details, QString *error);
    bool verify(qint64 id, const QJsonObject &actual, QString *error);
    QJsonArray search(const QString &sn) const;
    bool openReadOnly(const QString &, QString *error);
    bool reportPath(qint64 id, const QString &, QString *error);

  private:
    QSqlDatabase db_;
};
} // namespace mt
