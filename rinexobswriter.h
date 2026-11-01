// Declares the writer for corrected RINEX observation epochs and headers.
#pragma once

#include <QDate>
#include <QDateTime>
#include <QFile>
#include <QMap>
#include <QTextStream>

class RinexObsWriter
{
public:
    RinexObsWriter();
    ~RinexObsWriter();

    void setOutputDirectory(const QString& directory);
    bool isEnabled() const;
    void close();

    bool writeEpoch(int gpsWeek,
                    double gpsSow,
                    const QMap<QString, QMap<QString, double>>& satObservations);

private:
    void flushPendingEpoch();
    void mergeObservations(QMap<QString, QMap<QString, double>>& dst,
                           const QMap<QString, QMap<QString, double>>& src);
    void ensureOpen(const QDateTime& epochUtc);
    void closeFile();
    QString resolveFilePath(const QDateTime& epochUtc) const;
    void writeHeader(const QDateTime& epochUtc);
    QString formatObs(double value) const;

    QString _directory;
    QFile _file;
    QTextStream _stream;
    QDate _openDate;
    bool _headerWritten = false;
    QMap<QChar, QStringList> _obsTypes;
    QDateTime _pendingEpochUtc;
    QMap<QString, QMap<QString, double>> _pendingSatObservations;
};
