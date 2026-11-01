// Owns the PPP worker thread and sequences runtime lifecycle and queued updates.
#include "pppruntimeservice.h"

#include <QMetaObject>
#include <QMutexLocker>
#include <QThread>

namespace {

} // namespace

PppRuntimeService::PppRuntimeService(QObject* parent)
    : QObject(parent),
      _thread(new QThread(this)),
      _worker(new PppWorker()),
      _gfsService(new GfsZwdAsyncService(this)) {
    _worker->moveToThread(_thread);
    connect(_thread, &QThread::finished, _worker, &QObject::deleteLater);
    connect(_worker, &PppWorker::ready, this, &PppRuntimeService::ready,
            Qt::QueuedConnection);
    // This request signal is deliberately direct: it closes the ingress gate
    // in the calling thread before any subsequent MSM can be queued.
    connect(_worker, &PppWorker::gfsPriorRequested, this,
            [this](const QString& sourceId, quint64 requestId,
                   const GfsZwdRequest& request, GfsZwdMethod method) {
                startGfsPrior(sourceId, requestId, request, method);
            }, Qt::DirectConnection);
    connect(_gfsService, &GfsZwdAsyncService::completed, this,
            [this](const QString& sourceId, quint64 requestId, bool ok,
                   const PppTropospherePrior& prior, const QString& error) {
                handleGfsPriorCompleted(sourceId, requestId, ok, prior, error);
            }, Qt::QueuedConnection);
    connect(_gfsService, &GfsZwdAsyncService::progress, this,
            [this](const QString& sourceId, quint64, const QString& message) {
                emit gfsProgress(sourceId, message);
            }, Qt::QueuedConnection);
    _thread->start();
    stopAndClear();
}

PppRuntimeService::~PppRuntimeService() {
    shutdown();
}

void PppRuntimeService::configure(const PppOptions& options,
                                  const PppRuntimePolicy& policy,
                                  const PppStreamSelection& selection) {
    cancelGfsPrior(QString(), false);
    invokeBlocking([options, policy, selection](PppWorker* worker) {
        worker->setStreamSelection(selection);
        worker->setRuntimePolicy(policy);
        worker->setOptions(options);
    });
}

void PppRuntimeService::setOptions(const PppOptions& options) {
    cancelGfsPrior(QString(), false);
    invokeBlocking([options](PppWorker* worker) { worker->setOptions(options); });
}

void PppRuntimeService::setRuntimePolicy(const PppRuntimePolicy& policy) {
    invokeBlocking([policy](PppWorker* worker) { worker->setRuntimePolicy(policy); });
}

void PppRuntimeService::setStreamSelection(const PppStreamSelection& selection) {
    cancelGfsPrior(QString(), false);
    invokeBlocking([selection](PppWorker* worker) { worker->setStreamSelection(selection); });
}

void PppRuntimeService::start() {
    {
        QMutexLocker locker(&_gfsMutex);
        _acceptGfsResults = true;
    }
    invokeBlocking([](PppWorker* worker) { worker->startProcessing(); });
}

void PppRuntimeService::requestStop() {
    cancelGfsPrior();
    if (_worker) {
        _worker->requestStop();
    }
}

void PppRuntimeService::stopAndClear() {
    requestStop();
    invokeBlocking([](PppWorker* worker) { worker->stopProcessing(); });
}

void PppRuntimeService::shutdown() {
    QThread* thread = _thread;
    if (!thread) {
        _worker = nullptr;
        return;
    }
    stopAndClear();
    thread->quit();
    thread->wait();
    delete thread;
    _thread = nullptr;
    _worker = nullptr;
}

void PppRuntimeService::updateVtecModel(const PppVtecModel& vtec) {
    invokeQueued([vtec](PppWorker* worker) { worker->updateVtecModel(vtec); });
}

void PppRuntimeService::updateBroadcast(const QString& sourceId,
                                        const RtcmBroadcastEphemeris& eph) {
    invokeQueued([sourceId, eph](PppWorker* worker) { worker->updateBroadcast(sourceId, eph); });
}

void PppRuntimeService::updateSsrMessage(const QString& sourceId,
                                         const RtcmSsrMessage& ssr) {
    invokeQueued([sourceId, ssr](PppWorker* worker) { worker->updateSsrMessage(sourceId, ssr); });
}

void PppRuntimeService::enqueueMsm(const QString& sourceId,
                                   const RtcmMsmMessage& msm,
                                   const GloChannelMap& gloChannels) {
    {
        QMutexLocker locker(&_gfsMutex);
        if (_gfsPendingRequests.contains(sourceId)) {
            ++_gfsDroppedEpochs[sourceId];
            return;
        }
    }
    invokeQueued([sourceId, msm, gloChannels](PppWorker* worker) {
        worker->enqueueMsm(sourceId, msm, gloChannels);
    });
}

void PppRuntimeService::clearSource(const QString& sourceId) {
    cancelGfsPrior(sourceId);
    invokeBlocking([sourceId](PppWorker* worker) { worker->clearSource(sourceId); });
}

void PppRuntimeService::clearAll() {
    cancelGfsPrior();
    invokeBlocking([](PppWorker* worker) { worker->clearAll(); });
}

void PppRuntimeService::startGfsPrior(const QString& sourceId, quint64 requestId,
                                      const GfsZwdRequest& request, GfsZwdMethod method) {
    {
        QMutexLocker locker(&_gfsMutex);
        if (!_acceptGfsResults) {
            return;
        }
        _gfsPendingRequests.insert(sourceId, requestId);
        _gfsDroppedEpochs.insert(sourceId, 0);
    }
    emit gfsInitializationChanged(sourceId, true);
    _gfsService->start(sourceId, requestId, request, method);
}

void PppRuntimeService::handleGfsPriorCompleted(const QString& sourceId, quint64 requestId,
                                                bool ok, const PppTropospherePrior& prior,
                                                const QString& error) {
    // Keep the ingress gate closed until the worker has installed the prior.
    // A queued completion followed by an early gate release would allow a
    // concurrently arriving MSM to overtake the completion event.
    int dropped = 0;
    {
        QMutexLocker locker(&_gfsMutex);
        if (!_acceptGfsResults || _gfsPendingRequests.value(sourceId, 0) != requestId) {
            return;
        }
        dropped = _gfsDroppedEpochs.value(sourceId, 0);
    }
    invokeBlocking([sourceId, requestId, ok, prior, error, dropped](PppWorker* worker) {
        worker->completeGfsPrior(sourceId, requestId, ok, prior, error, dropped);
    });
    {
        QMutexLocker locker(&_gfsMutex);
        if (_gfsPendingRequests.value(sourceId, 0) == requestId) {
            _gfsPendingRequests.remove(sourceId);
            _gfsDroppedEpochs.remove(sourceId);
        }
    }
    emit gfsInitializationChanged(sourceId, false);
}

void PppRuntimeService::cancelGfsPrior(const QString& sourceId, bool disableResults) {
    QMap<QString, quint64> cancelled;
    {
        QMutexLocker locker(&_gfsMutex);
        if (sourceId.isEmpty()) {
            cancelled = _gfsPendingRequests;
            _gfsPendingRequests.clear();
            _gfsDroppedEpochs.clear();
            if (disableResults) {
                _acceptGfsResults = false;
            }
        } else {
            const auto it = _gfsPendingRequests.find(sourceId);
            if (it != _gfsPendingRequests.constEnd()) {
                cancelled.insert(sourceId, it.value());
                _gfsPendingRequests.erase(it);
            }
            _gfsDroppedEpochs.remove(sourceId);
        }
    }
    for (auto it = cancelled.constBegin(); it != cancelled.constEnd(); ++it) {
        _gfsService->cancel(it.key(), it.value());
        emit gfsInitializationChanged(it.key(), false);
    }
}

void PppRuntimeService::takePending(QVector<PppWorker::SolutionEvent>& solutions,
                                    QStringList& logs) {
    if (_worker) {
        _worker->takePending(solutions, logs);
    }
}

void PppRuntimeService::invokeBlocking(
    const std::function<void(PppWorker*)>& operation) {
    PppWorker* worker = _worker;
    QThread* thread = _thread;
    if (!worker || !thread || !thread->isRunning()) {
        return;
    }
    if (worker->thread() == QThread::currentThread()) {
        operation(worker);
        return;
    }
    QMetaObject::invokeMethod(worker, [worker, operation]() { operation(worker); },
                              Qt::BlockingQueuedConnection);
}

void PppRuntimeService::invokeQueued(
    const std::function<void(PppWorker*)>& operation) {
    PppWorker* worker = _worker;
    QThread* thread = _thread;
    if (!worker || !thread || !thread->isRunning()) {
        return;
    }
    const Qt::ConnectionType type = worker->thread() == QThread::currentThread()
        ? Qt::DirectConnection : Qt::QueuedConnection;
    QMetaObject::invokeMethod(worker, [worker, operation]() { operation(worker); }, type);
}
