// Declares the queued PPP worker that serializes access to PppEngine.
#pragma once

#include "ppp_engine.h"
#include "rtcmdecoder.h"
#include "gfs_zwd.h"

#include <QDateTime>
#include <QMap>
#include <QMutex>
#include <QObject>
#include <QAtomicInt>
#include <QSet>
#include <QStringList>
#include <QVector>

using GloChannelMap = QMap<int, int>;
struct PppRuntimePolicy {
    int maxQueueSize = 120;
    int corrWaitTimeSec = 5;
    int dropLagSec = 60;
    int epochIntervalSec = 1;
};

class PppWorker : public QObject
{
    Q_OBJECT

public:
    using RuntimePolicy = PppRuntimePolicy;

    struct SolutionEvent {
        QString sourceId;
        PppSolution solution;
        QVector<PppEngine::SsrYawDebug> yawDebug;
        QVector<QString> antexWarnings;
    };

    explicit PppWorker(QObject* parent = nullptr);

    void takePending(QVector<SolutionEvent>& solutions, QStringList& logs);

public slots:
    void requestStop();
    void setRuntimePolicy(const PppRuntimePolicy& policy);
    void setOptions(const PppOptions& options);
    void setStreamSelection(const PppStreamSelection& selection);
    void updateVtecModel(const PppVtecModel& vtec);
    void clearVtecModel();
    void updateBroadcast(const QString& sourceId, const RtcmBroadcastEphemeris& eph);
    void updateSsrMessage(const QString& sourceId, const RtcmSsrMessage& ssr);
    void enqueueMsm(const QString& sourceId,
                    const RtcmMsmMessage& msm,
                    const GloChannelMap& gloChannels);
    void clearSource(const QString& sourceId);
    void clearAll();
    void startProcessing();
    void stopProcessing();
    void completeGfsPrior(const QString& sourceId, quint64 requestId, bool ok,
                          const PppTropospherePrior& prior, const QString& error,
                          int droppedEpochs);

signals:
    void ready();
    void gfsPriorRequested(const QString& sourceId, quint64 requestId,
                           const GfsZwdRequest& request, GfsZwdMethod method);

private:
    struct PendingEpoch {
        QString sourceId;
        RtcmMsmMessage msm;
        GloChannelMap gloChannels;
        QDateTime queuedUtc;
        int gpsWeek = 0;
        double gpsSow = 0.0;
        bool gpsTimeValid = false;
    };

    struct PendingBiasBatch {
        QString sourceId;
        QChar system;
        RtcmSsrCorrectionKind kind = RtcmSsrCorrectionKind::Unknown;
        QVector<RtcmSsrMessage> messages;
        QDateTime updatedUtc;
    };

    void processQueue();
    void clearRuntimeState(bool clearPendingOutput);
    void publishBufferedSsr(const QString& sourceId, const RtcmSsrMessage& ssr);
    void updateLastClockCorrectionTime(const RtcmSsrMessage& ssr);
    bool shouldEmitLog(QDateTime& lastUtc, int intervalSec) const;
    void pushLog(const QString& text);
    bool requestGfsPrior(const PendingEpoch& epoch,
                         const std::array<double, 3>* seedXyz = nullptr);

    RuntimePolicy _policy;

    PppEngine _engine;
    bool _active = false;
    QAtomicInt _stopRequested = 0;
    QVector<PendingEpoch> _epochQueue;
    int _lastClockCorrWeek = 0;
    double _lastClockCorrSow = 0.0;
    bool _lastClockCorrValid = false;
    QMap<QString, GnssUtils::GpsTime> _lastProcessedTimeBySource;
    QDateTime _lastDropLogUtc;
    QDateTime _lastOverflowLogUtc;
    QDateTime _lastProcessFailLogUtc;
    QMap<QString, QMap<qint64, QVector<RtcmSsrMessage>>> _pendingSsrBySource;
    QMap<QString, PendingBiasBatch> _pendingBiasBatches;
    QSet<QString> _gfsPreparedSources;
    QSet<QString> _gfsPendingSources;
    QMap<QString, quint64> _gfsRequestIds;
    QMap<QString, int> _gfsDroppedEpochs;
    quint64 _nextGfsRequestId = 0;

    mutable QMutex _pendingMutex;
    QVector<SolutionEvent> _pendingSolutions;
    QStringList _pendingLogs;
};
