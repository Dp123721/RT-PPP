// Writes corrected observations in RINEX format.
#include "rinexobswriter.h"

#include "gnss_signals.h"
#include "gnss_utils.h"

#include <cstdlib>
#include <cmath>
#include <limits>
#include <QDir>
#include <QtMath>

RinexObsWriter::RinexObsWriter() : _stream(&_file) {}

RinexObsWriter::~RinexObsWriter() {
    close();
}

void RinexObsWriter::setOutputDirectory(const QString& directory) {
    const QString normalized = directory.trimmed();
    if (_directory == normalized) {
        return;
    }
    close();
    _directory = normalized;
}

bool RinexObsWriter::isEnabled() const {
    return !_directory.isEmpty();
}

void RinexObsWriter::close() {
    flushPendingEpoch();
    closeFile();
}

bool RinexObsWriter::writeEpoch(int gpsWeek,
                                double gpsSow,
                                const QMap<QString, QMap<QString, double>>& satObservations) {
    constexpr qint64 kEpochToleranceMs = 250;

    if (!isEnabled()) {
        return false;
    }
    if (satObservations.isEmpty()) {
        return false;
    }

    const QDateTime epochUtc = GnssUtils::dateTimeFromGpsWeek(gpsWeek, gpsSow).toUTC();
    if (!_pendingEpochUtc.isValid()) {
        ensureOpen(epochUtc);
        if (!_file.isOpen()) {
            return false;
        }
        _pendingEpochUtc = epochUtc;
        _pendingSatObservations = satObservations;
        return true;
    }

    const qint64 deltaMs = std::llabs(_pendingEpochUtc.msecsTo(epochUtc));
    if (deltaMs <= kEpochToleranceMs) {
        mergeObservations(_pendingSatObservations, satObservations);
        return true;
    }

    flushPendingEpoch();
    ensureOpen(epochUtc);
    if (!_file.isOpen()) {
        return false;
    }
    _pendingEpochUtc = epochUtc;
    _pendingSatObservations = satObservations;
    return true;
}

void RinexObsWriter::flushPendingEpoch() {
    if (_pendingSatObservations.isEmpty() || !_pendingEpochUtc.isValid()) {
        _pendingEpochUtc = QDateTime();
        _pendingSatObservations.clear();
        return;
    }
    if (!_file.isOpen()) {
        ensureOpen(_pendingEpochUtc);
    }
    if (!_file.isOpen()) {
        _pendingEpochUtc = QDateTime();
        _pendingSatObservations.clear();
        return;
    }

    const QDateTime utc = _pendingEpochUtc.toUTC();
    const QTime time = utc.time();
    const double sec = static_cast<double>(time.second()) + static_cast<double>(time.msec()) / 1000.0;
    const QString epochLine = QString("> %1 %2 %3 %4 %5%6  0%7")
                                  .arg(utc.date().year(), 4, 10, QLatin1Char('0'))
                                  .arg(utc.date().month(), 2, 10, QLatin1Char('0'))
                                  .arg(utc.date().day(), 2, 10, QLatin1Char('0'))
                                  .arg(time.hour(), 2, 10, QLatin1Char('0'))
                                  .arg(time.minute(), 2, 10, QLatin1Char('0'))
                                  .arg(sec, 11, 'f', 7)
                                  .arg(_pendingSatObservations.size(), 3, 10, QLatin1Char('0'));
    _stream << epochLine << "\n";

    for (auto it = _pendingSatObservations.constBegin(); it != _pendingSatObservations.constEnd(); ++it) {
        const QString prn = it.key();
        const QMap<QString, double>& obsMap = it.value();
        const QChar sys = prn.isEmpty() ? QChar() : prn.at(0);
        const QStringList types = _obsTypes.value(sys);
        if (types.isEmpty()) {
            continue;
        }

        QString line = prn.leftJustified(3, ' ');
        for (const QString& type : types) {
            const double val = obsMap.value(type, std::numeric_limits<double>::quiet_NaN());
            line += formatObs(val);
        }
        _stream << line << "\n";
    }

    _stream.flush();
    _file.flush();
    _pendingEpochUtc = QDateTime();
    _pendingSatObservations.clear();
}

void RinexObsWriter::mergeObservations(QMap<QString, QMap<QString, double>>& dst,
                                       const QMap<QString, QMap<QString, double>>& src) {
    for (auto it = src.constBegin(); it != src.constEnd(); ++it) {
        QMap<QString, double>& dstObs = dst[it.key()];
        const QMap<QString, double>& srcObs = it.value();
        for (auto sit = srcObs.constBegin(); sit != srcObs.constEnd(); ++sit) {
            dstObs[sit.key()] = sit.value();
        }
    }
}

void RinexObsWriter::ensureOpen(const QDateTime& epochUtc) {
    const QDate date = epochUtc.date();
    if (_file.isOpen() && date == _openDate) {
        return;
    }
    closeFile();
    if (_directory.isEmpty()) {
        return;
    }

    QDir dir(_directory);
    if (!dir.exists()) {
        dir.mkpath(".");
    }
    _file.setFileName(resolveFilePath(epochUtc));
    if (!_file.open(QIODevice::WriteOnly | QIODevice::Text)) {
        return;
    }
    _stream.setCodec("UTF-8");
    _openDate = date;
    writeHeader(epochUtc);
}

void RinexObsWriter::closeFile() {
    if (_file.isOpen()) {
        _file.close();
    }
    _openDate = QDate();
    _headerWritten = false;
}

QString RinexObsWriter::resolveFilePath(const QDateTime& epochUtc) const {
    const QString dateStr = epochUtc.toUTC().toString("yyyyMMdd");
    const QString fileName = QString("rtppp_%1.rnx").arg(dateStr);
    QDir dir(_directory);
    return dir.filePath(fileName);
}

void RinexObsWriter::writeHeader(const QDateTime& epochUtc) {
    if (_headerWritten) {
        return;
    }
    _stream << "     3.04           OBSERVATION DATA    M (MIXED)           RINEX VERSION / TYPE\n";
    _stream << QString("%1%2%3")
                   .arg("RT-PPP", -20)
                   .arg("SSR-CORRECTED", -20)
                   .arg(QDateTime::currentDateTimeUtc().toString("yyyyMMdd hhmmss 'UTC'"), -20)
            << "PGM / RUN BY / DATE\n";

    _obsTypes.clear();
    const QList<QChar> systems = {QLatin1Char('G'), QLatin1Char('R'), QLatin1Char('E'),
                                   QLatin1Char('C'), QLatin1Char('J'), QLatin1Char('S'),
                                   QLatin1Char('I')};
    for (const QChar& sys : systems) {
        const QStringList types = rinexObsTypesForSystem(sys);
        if (types.isEmpty()) {
            continue;
        }
        _obsTypes[sys] = types;

        int idx = 0;
        while (idx < types.size()) {
            const int perLine = 13;
            const int count = qMin(perLine, types.size() - idx);
            QString line;
            if (idx == 0) {
                line = QString("%1  %2").arg(sys).arg(types.size(), 3);
            } else {
                line = "      ";
            }
            for (int i = 0; i < count; ++i) {
                line += QString("%1").arg(types[idx + i], 4, QLatin1Char(' '));
            }
            line = line.leftJustified(60, ' ');
            line += "SYS / # / OBS TYPES";
            _stream << line << "\n";
            idx += count;
        }
    }

    const QDateTime utc = epochUtc.toUTC();
    const QTime time = utc.time();
    const double sec = static_cast<double>(time.second()) + static_cast<double>(time.msec()) / 1000.0;
    _stream << QString("%1%2%3%4%5%6%7")
                   .arg(utc.date().year(), 6)
                   .arg(utc.date().month(), 6)
                   .arg(utc.date().day(), 6)
                   .arg(time.hour(), 6)
                   .arg(time.minute(), 6)
                   .arg(sec, 13, 'f', 7)
                   .arg("GPS", 8)
                   .leftJustified(60)
            << "TIME OF FIRST OBS\n";

    _stream << "                                                            END OF HEADER\n";
    _headerWritten = true;
}

QString RinexObsWriter::formatObs(double value) const {
    if (std::isnan(value)) {
        return QString().leftJustified(16, QLatin1Char(' '));
    }
    return QString("%1  ").arg(value, 14, 'f', 3);
}
