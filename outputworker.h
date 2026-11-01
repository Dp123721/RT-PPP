// Declares the bounded output queue and its single-owner writer lifecycle.
#pragma once

#include "pppjsonwriter.h"
#include "rinexobswriter.h"
#include "sp3writer.h"
#include "telemetrywriter.h"

#include <QAtomicInt>
#include <QDateTime>
#include <QMap>
#include <QMutex>
#include <QObject>
#include <QPair>
#include <QString>
#include <QVector>

#include <array>

// Serializes non-critical disk output away from the GUI and PPP worker.
// Jobs are copied into a bounded dispatcher; the mathematical processing
// chain never calls a writer directly.
class OutputWorker : public QObject
{
public:
    struct Config {
        bool sp3Enabled = false;
        QString sp3Directory;
        int sp3SampleSec = 10;
        bool rinexEnabled = false;
        QString rinexDirectory;
        bool pppJsonEnabled = false;
        QString pppJsonDirectory;
        int pppJsonReorderBufferMs = 2000;
        int pppJsonFlushIntervalMs = 1000;
        QString ztdDirectory;
        QString logFilePath;
        QString telemetryDirectory;
        QMap<QString, QString> telemetrySourceNames;
        QString arLogFilePath;
    };

    explicit OutputWorker(QObject* parent = nullptr);
    ~OutputWorker() override;

    void configure(const Config& config);
    void enqueueSp3(int gpsWeek,
                    double gpsSow,
                    const QString& prn,
                    const std::array<double, 3>& position,
                    double clockSec,
                    const std::array<double, 3>& velocity,
                    double clockRate);
    void enqueueRinex(int gpsWeek,
                      double gpsSow,
                      const QMap<QString, QMap<QString, double>>& observations);
    void enqueuePppJson(const QString& stationId,
                        const PppSolution& solution,
                        const std::array<double, 3>& referenceEcef);
    void enqueueZtd(const QString& source,
                    const QString& station,
                    const PppSolution& solution);
    void enqueueLog(const QString& text);
    void enqueueTelemetryMsm(
        const QString& sourceId,
        const QDateTime& receivedUtc,
        const QDateTime& epochUtc,
        const RtcmMsmMessage& message,
        const QMap<QString, QPair<double, double>>& skyBySatellite);
    void enqueueTelemetrySsr(const QString& sourceId,
                             const QDateTime& receivedUtc,
                             const QDateTime& epochUtc,
                             const RtcmSsrMessage& message,
                             const RtcmSsrSatelliteData& satellite,
                             bool adopted,
                             const QString& reason);
    void enqueueTelemetryVtec(const QString& sourceId,
                              const QDateTime& epochUtc,
                              const RtcmVtecMessage& message);
    void enqueueTelemetryReception(const QString& sourceId,
                                    const QDateTime& receivedUtc,
                                    const QString& messageFamily,
                                    int messageType,
                                    const QChar& system,
                                    const QString& status,
                                    const QString& reason,
                                    const QString& detail);
    void enqueueTelemetryPppResiduals(const QString& sourceId,
                                      const PppSolution& solution);
    void enqueueArLog(const PppSolution& solution);

    // Must be invoked in the worker thread with a blocking queued call.
    void finalizePppJson();

    // Must be invoked in the worker thread with a blocking queued call.
    void shutdown();

private:
    enum class Kind {
        Configure,
        Sp3,
        Rinex,
        PppJson,
        Ztd,
        Log,
        TelemetryMsm,
        TelemetrySsr,
        TelemetryVtec,
        TelemetryReception,
        TelemetryPppResiduals,
        ArLog
    };

    struct Job {
        Kind kind = Kind::Configure;
        Config config;
        int gpsWeek = 0;
        double gpsSow = 0.0;
        QString prn;
        std::array<double, 3> position = {0.0, 0.0, 0.0};
        std::array<double, 3> velocity = {0.0, 0.0, 0.0};
        double clockSec = 0.0;
        double clockRate = 0.0;
        QMap<QString, QMap<QString, double>> observations;
        QString stationId;
        QString source;
        QString station;
        QString logText;
        QDateTime receivedUtc;
        QDateTime epochUtc;
        RtcmMsmMessage msm;
        QMap<QString, QPair<double, double>> skyBySatellite;
        RtcmSsrMessage ssr;
        RtcmSsrSatelliteData ssrSatellite;
        RtcmVtecMessage vtec;
        bool adopted = false;
        QString reason;
        QString messageFamily;
        int messageType = 0;
        QChar system;
        QString status;
        QString detail;
        PppSolution solution;
        std::array<double, 3> referenceEcef = {0.0, 0.0, 0.0};
    };

    void submit(Job job);
    void scheduleDrain();
    void drain();
    void process(const Job& job);
    void applyConfig(const Config& config);
    void writeZtd(const Job& job);
    void writeLog(const Job& job);
    void writeArLog(const Job& job);
    void reportError(const QString& family, const QString& error) const;

    mutable QMutex _mutex;
    QVector<Job> _jobs;
    QAtomicInt _drainScheduled = 0;
    QAtomicInt _accepting = 1;
    Config _config;
    Sp3Writer _sp3Writer;
    RinexObsWriter _rinexWriter;
    PppJsonWriter _pppJsonWriter;
    TelemetryWriter* _telemetryWriter = nullptr;
    QString _ztdDirectory;
    QString _logFilePath;
    QString _arLogFilePath;
};
