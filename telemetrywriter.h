// Declares structured writers for MSM, receiver, SSR, and PPP telemetry.
#pragma once

#include "ppp_engine.h"
#include "rtcmdecoder.h"

#include <QByteArray>
#include <QDateTime>
#include <QFile>
#include <QMap>
#include <QObject>
#include <QPair>
#include <QString>

class QTimer;

// Writes decoded observation telemetry as CSV and SSR products in SSR ASCII form.
class TelemetryWriter : public QObject
{
public:
    explicit TelemetryWriter(QObject* parent = nullptr);
    ~TelemetryWriter() override;

    void setOutputDirectory(const QString& directory);
    void setSourceNames(const QMap<QString, QString>& names);
    bool isEnabled() const;

    void writeMsm(const QString& sourceId,
                  const QDateTime& receivedUtc,
                  const QDateTime& epochUtc,
                  const RtcmMsmMessage& message,
                  const QMap<QString, QPair<double, double>>& skyBySatellite);
    void writeSsr(const QString& sourceId,
                  const QDateTime& receivedUtc,
                  const QDateTime& epochUtc,
                  const RtcmSsrMessage& message,
                  const RtcmSsrSatelliteData& satellite,
                  bool adopted,
                  const QString& reason);
    void writeVtec(const QString& sourceId,
                   const QDateTime& epochUtc,
                   const RtcmVtecMessage& message);
    void writeReception(const QString& sourceId,
                        const QDateTime& receivedUtc,
                        const QString& messageFamily,
                        int messageType,
                        const QChar& system,
                        const QString& status,
                        const QString& reason,
                        const QString& detail);
    void writePppResiduals(const QString& sourceId, const PppSolution& solution);
    void flush();
    void close();

private:
    struct FileState {
        QFile* file = nullptr;
        QByteArray buffer;
    };
    struct SsrBlock {
        QString sourceId;
        QString kind;
        QDateTime epochUtc;
        RtcmSsrMessage message;
        QMap<QString, QString> records;
        QStringList satelliteOrder;
        QString consistency;
    };

    FileState* fileState(const QString& family,
                         const QDateTime& utc);
    void appendRow(const QString& family,
                   const QDateTime& utc,
                   const QStringList& fields);
    void appendSsrBlock(const QString& sourceId,
                        const QDateTime& epochUtc,
                        const RtcmSsrMessage& message,
                        const RtcmSsrSatelliteData& satellite,
                        const QString& kind,
                        const QString& record);
    void flushSsrBlocks(const QString& productKey = QString(),
                        const QDateTime& beforeEpoch = QDateTime());
    void writeSsrBlock(const SsrBlock& block);
    QString sourceName(const QString& sourceId) const;
    static void flushState(FileState& state);
    static QString csv(const QString& value);
    static QString utcText(const QDateTime& value);
    static QString number(double value);
    static QString boolText(bool value);
    static QString satelliteText(const RtcmSatId& satellite);

    QString _directory;
    QMap<QString, FileState> _files;
    QMap<QString, QString> _sourceNames;
    QMap<QString, SsrBlock> _ssrBlocks;
    QMap<QString, RtcmSsrSatelliteData> _lastClockCorrections;
    QMap<QString, int> _orbitIods;
    QMap<QString, int> _clockSsrIods;
    QTimer* _flushTimer = nullptr;
};
