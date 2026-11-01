// Declares SP3 orbit and clock product output.
#pragma once

#include "gnss_utils.h"

#include <QDate>
#include <QFile>
#include <QTextStream>
#include <QSet>

class Sp3Writer
{
public:
    Sp3Writer();
    ~Sp3Writer();

    void setOutputDirectory(const QString& directory);
    void setSampleInterval(int seconds);
    bool isEnabled() const;
    void close();

    bool writeEpoch(int gpsWeek,
                    double gpsSow,
                    const QString& prn,
                    const std::array<double,3>& position,
                    double clockSec,
                    const std::array<double,3>& velocity,
                    double clockRate);

private:
    void ensureOpen(const QDateTime& epochUtc);
    void closeFile();
    void writeHeader(const QDateTime& epochUtc);
    void writeClkHeader(const QDateTime& epochUtc);
    void updateHeaderSats();
    QString resolveFilePath(const QDateTime& epochUtc) const;
    QString resolveClkFilePath(const QDateTime& epochUtc) const;

    QString _directory;
    int _sampleInterval = 10;
    QFile _file;
    QTextStream _stream;
    QFile _clkFile;
    QTextStream _clkStream;
    QDate _openDate;
    QDate _clkOpenDate;
    QDateTime _lastEpoch;
    QSet<QString> _epochSatSet;
    QSet<QString> _allSats;
    QVector<qint64> _headerSatPos;
    QVector<qint64> _headerAccPos;
    int _headerSatLines = 10;
};
