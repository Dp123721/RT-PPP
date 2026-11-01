// Declares coordinated PPP worker startup, configuration delivery, and shutdown.
#pragma once

#include "ppp_worker.h"
#include "gfs_zwd.h"

#include <QMap>
#include <QMutex>
#include <QObject>

#include <functional>

class QThread;

class PppRuntimeService : public QObject
{
    Q_OBJECT

public:
    explicit PppRuntimeService(QObject* parent = nullptr);
    ~PppRuntimeService() override;

    void configure(const PppOptions& options,
                   const PppRuntimePolicy& policy,
                   const PppStreamSelection& selection);
    void setOptions(const PppOptions& options);
    void setRuntimePolicy(const PppRuntimePolicy& policy);
    void setStreamSelection(const PppStreamSelection& selection);

    void start();
    void requestStop();
    void stopAndClear();
    void shutdown();

    void updateVtecModel(const PppVtecModel& vtec);
    void updateBroadcast(const QString& sourceId, const RtcmBroadcastEphemeris& eph);
    void updateSsrMessage(const QString& sourceId, const RtcmSsrMessage& ssr);
    void enqueueMsm(const QString& sourceId,
                    const RtcmMsmMessage& msm,
                    const GloChannelMap& gloChannels);
    void clearSource(const QString& sourceId);
    void clearAll();
    void takePending(QVector<PppWorker::SolutionEvent>& solutions, QStringList& logs);

signals:
    void ready();
    void gfsInitializationChanged(const QString& sourceId, bool pending);
    void gfsProgress(const QString& sourceId, const QString& message);

private:
    void invokeBlocking(const std::function<void(PppWorker*)>& operation);
    void invokeQueued(const std::function<void(PppWorker*)>& operation);
    void startGfsPrior(const QString& sourceId, quint64 requestId,
                       const GfsZwdRequest& request, GfsZwdMethod method);
    void handleGfsPriorCompleted(const QString& sourceId, quint64 requestId, bool ok,
                                 const PppTropospherePrior& prior, const QString& error);
    void cancelGfsPrior(const QString& sourceId = QString(), bool disableResults = true);

    QThread* _thread = nullptr;
    PppWorker* _worker = nullptr;
    GfsZwdAsyncService* _gfsService = nullptr;
    mutable QMutex _gfsMutex;
    QMap<QString, quint64> _gfsPendingRequests;
    QMap<QString, int> _gfsDroppedEpochs;
    bool _acceptGfsResults = false;
};
