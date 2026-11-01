// Declares the worker that prepares source-isolated RTCM, correction, and observation state.
#pragma once

#include "rtcmdecoder.h"
#include "gnss_ephemeris.h"

#include <QAtomicInt>
#include <QDateTime>
#include <QMap>
#include <QMutex>
#include <QObject>
#include <QPair>
#include <QSet>
#include <QString>
#include <QVector>

#include <array>
#include <memory>

class RtcmProcessingService : public QObject
{
    Q_OBJECT

public:
    struct MsmSystemSummary {
        QChar system;
        int messageType = 0;
        int epochTime = 0;
        int satCount = 0;
        int signalCount = 0;
        int cellCount = 0;
        QMap<int, double> satCnr;
        QDateTime receivedUtc;
    };

    struct MsmStatusUpdate {
        RtcmMsmMessage mergedMessage;
        MsmSystemSummary summary;
        qint64 epochKey = 0;
        bool firstForSource = false;
        bool receiverPositionValid = false;
        QMap<QString, QPair<double, double>> skyBySatellite;
        int gpsWeek = 0;
        double gpsSow = 0.0;
        QMap<QString, QMap<QString, double>> correctedObservations;
        int correctedCodeCount = 0;
        int correctedPhaseCount = 0;
        QMap<int, int> glonassChannels;
    };

    struct DataPreparationConfig {
        bool enabled = false;
        QString broadcastStreamId;
        QString orbitStreamId;
        QString clockStreamId;
        QString codeBiasStreamId;
        QString phaseBiasStreamId;
        QString obsStreamId;
        bool sp3Enabled = false;
        int sp3SampleSec = 10;
        bool rinexEnabled = false;
        QString broadcastMountpoint;
        QString orbitMountpoint;
    };

    struct PreparedState {
        RtcmSatId sat;
        int broadcastIod = -1;
        bool orbitApplied = false;
        bool clockApplied = false;
        bool codeApplied = false;
        bool phaseApplied = false;
        QString orbitStatus;
        QString clockStatus;
        int satRefDatum = -1;
        QString datumText;
        double orbitDtSec = 0.0;
        double clockDtSec = 0.0;
        double orbitRsw[3] = {0.0, 0.0, 0.0};
        double clockSec = 0.0;
        int codeBiasCount = 0;
        int phaseBiasCount = 0;
        QDateTime updatedUtc;
    };

    struct Sp3Record {
        int gpsWeek = 0;
        double gpsSow = 0.0;
        QString prn;
        std::array<double, 3> position = {0.0, 0.0, 0.0};
        double clockSec = 0.0;
        std::array<double, 3> velocity = {0.0, 0.0, 0.0};
        double clockRate = 0.0;
    };

    struct SsrTelemetryEvent {
        QString sourceId;
        QDateTime receivedUtc;
        QDateTime epochUtc;
        RtcmSsrMessage message;
        RtcmSsrSatelliteData satellite;
        bool adopted = false;
        QString reason;
    };

    struct Result {
        QString sourceId;
        RtcmDecodeBatch batch;
        QVector<MsmStatusUpdate> msmStatusUpdates;
        bool preparedStatesChanged = false;
        QMap<QString, PreparedState> preparedStates;
        QVector<Sp3Record> sp3Records;
        QVector<SsrTelemetryEvent> ssrTelemetryEvents;
    };

    explicit RtcmProcessingService(QObject* parent = nullptr);

    void takePending(QVector<Result>& results);
    void requestStop();
    void drainAllIngress();

    // Thread-safe ingress used by network clients.  It coalesces adjacent
    // chunks before scheduling one event on the RTCM processing thread.
    void submitPayload(const QString& sourceId,
                       const QByteArray& payload,
                       const std::array<double, 3>& receiverEcef);

public slots:
    void enqueuePayload(const QString& sourceId,
                        const QByteArray& payload,
                        const std::array<double, 3>& receiverEcef);
    void setObservationSampleSec(int sampleSec);
    void setObservationSuppressed(const QString& sourceId, bool suppressed);
    void setDataPreparationConfig(const DataPreparationConfig& config);
    void clearSource(const QString& sourceId);
    void clearAll();
    void shutdown();
    void drainIngress();

signals:
    void ready();
    // During a GFS bootstrap, high-rate product messages bypass the GUI. The
    // runtime retains a bounded SSR snapshot while observation epochs wait.
    void broadcastAvailable(const QString& sourceId, const RtcmBroadcastEphemeris& ephemeris);
    void ssrAvailable(const QString& sourceId, const RtcmSsrMessage& ssr);

private:
    bool acceptObservationMessage(const RtcmMsmMessage& msm,
                                  const QDateTime& receivedUtc);
    MsmStatusUpdate updateMsmStatus(const QString& sourceId,
                                    const RtcmMsmMessage& msm,
                                    const QDateTime& receivedUtc,
                                    const std::array<double, 3>& receiverEcef);
    void ingestBroadcast(const QString& sourceId, const RtcmBroadcastEphemeris& eph);
    void ingestSsr(const QString& sourceId,
                   const RtcmSsrMessage& ssr,
                   const QDateTime& receivedUtc,
                   QVector<SsrTelemetryEvent>& telemetryEvents,
                   bool collectTelemetry = true);
    void recomputePreparedStates(Result& result);
    void computeCorrectedObservations(const QString& sourceId,
                                      const RtcmMsmMessage& msm,
                                      MsmStatusUpdate& update);
    void computeSky(const RtcmMsmMessage& msm,
                    const std::array<double, 3>& receiverEcef,
                    MsmStatusUpdate& update) const;

    struct BroadcastState {
        RtcmBroadcastEphemeris eph;
        std::shared_ptr<GnssEphemeris> model;
        QString sourceId;
        QDateTime receivedUtc;
    };
    struct OrbitCorrectionState {
        RtcmSatId sat;
        QString sourceId;
        int iod = -1;
        int updateIntervalIndex = -1;
        int satRefDatum = -1;
        int epochTime = -1;
        double xr[3] = {0.0, 0.0, 0.0};
        double dotXr[3] = {0.0, 0.0, 0.0};
        QDateTime receivedUtc;
    };
    struct ClockCorrectionState {
        RtcmSatId sat;
        QString sourceId;
        int iod = -1;
        int updateIntervalIndex = -1;
        int epochTime = -1;
        double dClk = 0.0;
        double dotDClk = 0.0;
        double dotDotDClk = 0.0;
        QDateTime receivedUtc;
    };
    struct CodeBiasState {
        RtcmSatId sat;
        int messageType = 0;
        QVector<RtcmSsrCodeBiasEntry> biases;
        QDateTime receivedUtc;
    };
    struct PhaseBiasState {
        RtcmSatId sat;
        int messageType = 0;
        QVector<RtcmSsrPhaseBiasEntry> biases;
        QDateTime receivedUtc;
    };

    struct PendingPayload {
        QString sourceId;
        QByteArray payload;
        std::array<double, 3> receiverEcef = {0.0, 0.0, 0.0};
    };

    QMap<QString, Rtcm3Decoder> _decoders;
    QMap<QString, QMap<QChar, RtcmMsmMessage>> _lastMsmMessages;
    QMap<QString, BroadcastState> _broadcastStates;
    QMap<QString, OrbitCorrectionState> _orbitCorrections;
    QMap<QString, ClockCorrectionState> _clockCorrections;
    QMap<QString, CodeBiasState> _codeBiases;
    QMap<QString, PhaseBiasState> _phaseBiases;
    QMap<QString, QMap<int, int>> _glonassChannelsBySource;
    QSet<QString> _observationSuppressedSources;
    DataPreparationConfig _dataConfig;
    int _observationSampleSec = 1;
    QAtomicInt _accepting = 1;
    QAtomicInt _ingressScheduled = 0;
    QMutex _ingressMutex;
    QVector<PendingPayload> _ingress;
    QMutex _pendingMutex;
    QVector<Result> _pendingResults;
};
