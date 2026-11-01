// Declares validated, atomic access to persisted application settings.
#ifndef SETTINGSSTORE_H
#define SETTINGSSTORE_H

#include <QJsonArray>
#include <QString>

class SettingsStore
{
public:
    explicit SettingsStore(const QString& filePath = "rtppp_settings.ini");

    QJsonArray loadStreams() const;
    bool loadStreams(QJsonArray& streams, QString* error = nullptr) const;
    void saveStreams(const QJsonArray& streams);
    bool saveStreams(const QJsonArray& streams, QString* error = nullptr);
    QJsonArray loadCasters() const;
    bool loadCasters(QJsonArray& casters, QString* error = nullptr) const;
    void saveCasters(const QJsonArray& casters);
    bool saveCasters(const QJsonArray& casters, QString* error = nullptr);
    static QString normalizedArAlgorithm(const QString& value);
    bool validate(QString* error = nullptr) const;
    QString filePath() const { return _filePath; }
    void setFilePath(const QString& filePath) { _filePath = filePath; }

private:
    QString _filePath;
};

#endif // SETTINGSSTORE_H
