// Drains PPP input/configuration queues and forwards engine events from its worker thread.
#include "ppp_worker.h"

#include "gnss_utils.h"
#include <QMutexLocker>
#include <cmath>
#include <iterator>
#include <limits>

namespace {

bool isClockLikeCorrection(const RtcmSsrCorrectionKind kind) {
    return kind == RtcmSsrCorrectionKind::Clock ||
           kind == RtcmSsrCorrectionKind::OrbitClock ||
           kind == RtcmSsrCorrectionKind::HighRateClock;
}

bool isBiasCorrection(const RtcmSsrCorrectionKind kind) {
    return kind == RtcmSsrCorrectionKind::CodeBias ||
           kind == RtcmSsrCorrectionKind::PhaseBias;
}

QChar ssrSystem(const RtcmSsrMessage& ssr) {
    if (!ssr.system.isNull()) {
        return ssr.system;
    }
    if (!ssr.satellites.isEmpty()) {
        return ssr.satellites.first().sat.system;
    }
    return QChar();
}

QString biasBatchKey(const QString& sourceId, const RtcmSsrMessage& ssr) {
    QStringList fields;
    fields << sourceId
           << QString::number(static_cast<int>(ssr.kind))
           << QString(ssrSystem(ssr))
           << QString::number(ssr.messageType)
           << QString::number(ssr.epochTime)
           << QString::number(ssr.ssrIod)
           << QString::number(ssr.providerId)
           << QString::number(ssr.solutionId);
    return fields.join(QLatin1Char('|'));
}

qint64 gpsEpochKeyMs(int week, double sow) {
    return static_cast<qint64>(week) * 604800000LL +
           static_cast<qint64>(std::llround(sow * 1000.0));
}

} // namespace

PppWorker::PppWorker(QObject* parent) : QObject(parent) {
}

void PppWorker::setRuntimePolicy(const PppRuntimePolicy& policy) {
    _policy.maxQueueSize = qMax(1, policy.maxQueueSize);
    _policy.corrWaitTimeSec = 0;
    _policy.dropLagSec = qMax(1, policy.dropLagSec);
    _policy.epochIntervalSec = qMax(1, policy.epochIntervalSec);
    for (auto it = _epochQueue.begin(); it != _epochQueue.end();) {
        const GnssUtils::GpsTime time = GnssUtils::normalizeGpsTime(it->gpsWeek, it->gpsSow);
        it = GnssUtils::isGpsTimeOnSampleGrid(time, _policy.epochIntervalSec)
            ? std::next(it) : _epochQueue.erase(it);
    }
    QMutexLocker locker(&_pendingMutex);
    for (auto it = _pendingSolutions.begin(); it != _pendingSolutions.end();) {
        const GnssUtils::GpsTime time = GnssUtils::normalizeGpsTime(
            it->solution.gpsWeek, it->solution.gpsSow);
        it = GnssUtils::isGpsTimeOnSampleGrid(time, _policy.epochIntervalSec)
            ? std::next(it) : _pendingSolutions.erase(it);
    }
}

void PppWorker::requestStop() {
    _stopRequested.storeRelease(1);
    {
        QMutexLocker locker(&_pendingMutex);
        _pendingSolutions.clear();
        _pendingLogs.clear();
    }
}

void PppWorker::takePending(QVector<SolutionEvent>& solutions, QStringList& logs) {
    QMutexLocker locker(&_pendingMutex);
    solutions.swap(_pendingSolutions);
    logs.swap(_pendingLogs);
}

void PppWorker::setOptions(const PppOptions& options) {
    _engine.setOptions(options);
    clearRuntimeState(false);
}

void PppWorker::setStreamSelection(const PppStreamSelection& selection) {
    _engine.setStreamSelection(selection);
    clearRuntimeState(false);
}

void PppWorker::updateVtecModel(const PppVtecModel& vtec) {
    if (!_active || _stopRequested.loadAcquire()) {
        return;
    }
    _engine.updateVtecModel(vtec);
    processQueue();
}

void PppWorker::clearVtecModel() {
    if (!_active || _stopRequested.loadAcquire()) {
        return;
    }
    _engine.clearVtecModel();
}

void PppWorker::updateBroadcast(const QString& sourceId, const RtcmBroadcastEphemeris& eph) {
    if (!_active || _stopRequested.loadAcquire()) {
        return;
    }
    std::shared_ptr<GnssEphemeris> model;
    GnssEphemeris::fromRtcm(eph, model);
    _engine.updateBroadcast(eph, model, sourceId, QDateTime::currentDateTimeUtc());
}

void PppWorker::updateSsrMessage(const QString& sourceId, const RtcmSsrMessage& ssr) {
    if (!_active || _stopRequested.loadAcquire()) {
        return;
    }
    if (ssr.satellites.isEmpty() || ssr.epochTime < 0) {
        return;
    }

    if (isBiasCorrection(ssr.kind)) {
        const QDateTime nowUtc = QDateTime::currentDateTimeUtc();
        for (auto it = _pendingBiasBatches.begin(); it != _pendingBiasBatches.end();) {
            if (it.value().updatedUtc.isValid() && it.value().updatedUtc.secsTo(nowUtc) > 120) {
                it = _pendingBiasBatches.erase(it);
            } else {
                ++it;
            }
        }

        const QString batchKey = biasBatchKey(sourceId, ssr);
        PendingBiasBatch& batch = _pendingBiasBatches[batchKey];
        if (batch.messages.isEmpty()) {
            batch.sourceId = sourceId;
            batch.system = ssrSystem(ssr);
            batch.kind = ssr.kind;
        }
        batch.messages.push_back(ssr);
        batch.updatedUtc = nowUtc;
        if (ssr.multipleMessage) {
            return;
        }

        const QVector<RtcmSsrMessage> completeBatch = batch.messages;
        _pendingBiasBatches.remove(batchKey);
        _engine.beginBiasEpoch(sourceId, ssr);
        for (const RtcmSsrMessage& message : completeBatch) {
            publishBufferedSsr(sourceId, message);
        }
        processQueue();
        return;
    }

    const QDateTime nowUtc = QDateTime::currentDateTimeUtc();
    QChar sys = ssr.system;
    if (sys.isNull()) {
        sys = ssr.satellites.first().sat.system;
    }
    if (sys.isNull()) {
        sys = QLatin1Char('G');
    }
    const GnssUtils::GpsTime ssrTime = GnssUtils::gpsTimeFromSsr(sys, ssr.epochTime, nowUtc);
    const qint64 epochKey = gpsEpochKeyMs(ssrTime.week, ssrTime.sow);

    QMap<qint64, QVector<RtcmSsrMessage>>& byEpoch = _pendingSsrBySource[sourceId];
    byEpoch[epochKey].push_back(ssr);

    QVector<RtcmSsrMessage> ready;
    for (auto it = byEpoch.begin(); it != byEpoch.end();) {
        if (it.key() < epochKey || byEpoch.size() > 8) {
            ready += it.value();
            it = byEpoch.erase(it);
        } else {
            ++it;
        }
    }

    for (const RtcmSsrMessage& msg : ready) {
        publishBufferedSsr(sourceId, msg);
    }
    processQueue();
}

void PppWorker::publishBufferedSsr(const QString& sourceId, const RtcmSsrMessage& ssr) {
    if (!_active || _stopRequested.loadAcquire()) {
        return;
    }
    _engine.updateSsrMessage(sourceId, ssr);
    updateLastClockCorrectionTime(ssr);
}

void PppWorker::updateLastClockCorrectionTime(const RtcmSsrMessage& ssr) {
    if (!_active || _stopRequested.loadAcquire()) {
        return;
    }
    if (ssr.satellites.isEmpty() || !isClockLikeCorrection(ssr.kind)) {
        return;
    }

    const QDateTime nowUtc = QDateTime::currentDateTimeUtc();
    QChar sys = ssr.system;
    if (sys.isNull()) {
        sys = ssr.satellites.first().sat.system;
    }
    if (sys.isNull()) {
        sys = QLatin1Char('G');
    }
    const GnssUtils::GpsTime ssrTime = GnssUtils::gpsTimeFromSsr(sys, ssr.epochTime, nowUtc);
    _lastClockCorrWeek = ssrTime.week;
    _lastClockCorrSow = ssrTime.sow;
    _lastClockCorrValid = ssr.epochTime >= 0;
}

void PppWorker::enqueueMsm(const QString& sourceId,
                           const RtcmMsmMessage& msm,
                           const GloChannelMap& gloChannels) {
    if (!_active || _stopRequested.loadAcquire()) {
        return;
    }
    if (_gfsPendingSources.contains(sourceId)) {
        ++_gfsDroppedEpochs[sourceId];
        return;
    }
    PendingEpoch epoch;
    epoch.sourceId = sourceId;
    epoch.msm = msm;
    epoch.gloChannels = gloChannels;
    epoch.queuedUtc = QDateTime::currentDateTimeUtc();
    const GnssUtils::GpsTime msmTime = GnssUtils::gpsTimeFromMsm(msm, epoch.queuedUtc);
    if (!GnssUtils::isGpsTimeOnSampleGrid(msmTime, _policy.epochIntervalSec)) {
        return;
    }
    epoch.gpsWeek = msmTime.week;
    epoch.gpsSow = msmTime.sow;
    epoch.gpsTimeValid = true;
    _epochQueue.push_back(epoch);

    while (_epochQueue.size() > _policy.maxQueueSize) {
        const PendingEpoch dropped = _epochQueue.takeFirst();
        if (shouldEmitLog(_lastOverflowLogUtc, 2)) {
            pushLog(QString("PPP queue overflow: drop oldest epoch source=%1 queue=%2")
                        .arg(dropped.sourceId)
                        .arg(_epochQueue.size()));
        }
    }
    processQueue();
}

void PppWorker::clearSource(const QString& sourceId) {
    {
        QMutexLocker locker(&_pendingMutex);
        for (auto it = _pendingSolutions.begin(); it != _pendingSolutions.end();) {
            if (it->sourceId == sourceId) {
                it = _pendingSolutions.erase(it);
            } else {
                ++it;
            }
        }
    }
    if (!_active || _stopRequested.loadAcquire()) {
        return;
    }
    for (auto it = _epochQueue.begin(); it != _epochQueue.end();) {
        if (it->sourceId == sourceId) {
            it = _epochQueue.erase(it);
        } else {
            ++it;
        }
    }
    _pendingSsrBySource.remove(sourceId);
    _gfsPreparedSources.remove(sourceId);
    for (auto it = _pendingBiasBatches.begin(); it != _pendingBiasBatches.end();) {
        if (it.value().sourceId == sourceId) {
            it = _pendingBiasBatches.erase(it);
        } else {
            ++it;
        }
    }
}

void PppWorker::clearAll() {
    clearRuntimeState(true);
}

void PppWorker::startProcessing() {
    _stopRequested.storeRelease(0);
    _active = true;
    clearRuntimeState(true);
}

void PppWorker::stopProcessing() {
    _stopRequested.storeRelease(1);
    _active = false;
    clearRuntimeState(true);
}

void PppWorker::clearRuntimeState(bool clearPendingOutput) {
    _epochQueue.clear();
    _pendingSsrBySource.clear();
    _pendingBiasBatches.clear();
    _gfsPreparedSources.clear();
    _gfsPendingSources.clear();
    _gfsRequestIds.clear();
    _gfsDroppedEpochs.clear();
    _lastClockCorrValid = false;
    _lastProcessedTimeBySource.clear();
    _lastDropLogUtc = QDateTime();
    _lastOverflowLogUtc = QDateTime();
    _lastProcessFailLogUtc = QDateTime();
    _engine.resetProcessingState();
    if (clearPendingOutput) {
        QMutexLocker locker(&_pendingMutex);
        _pendingSolutions.clear();
        _pendingLogs.clear();
    }
}

void PppWorker::processQueue() {
    if (!_active || _stopRequested.loadAcquire()) {
        return;
    }
    for (;;) {
        if (_stopRequested.loadAcquire()) {
            _active = false;
            clearRuntimeState(true);
            return;
        }
        if (_epochQueue.isEmpty()) {
            return;
        }

        const QDateTime nowUtc = QDateTime::currentDateTimeUtc();
        if (!_active || _stopRequested.loadAcquire()) {
            return;
        }
        const PendingEpoch oldest = _epochQueue.first();
        const qint64 lagSec = oldest.queuedUtc.secsTo(nowUtc);
        if (lagSec > _policy.dropLagSec) {
            _epochQueue.removeFirst();
            if (shouldEmitLog(_lastDropLogUtc, 2)) {
                pushLog(QString("PPP epoch dropped: source=%1 lag=%2s queue=%3")
                            .arg(oldest.sourceId)
                            .arg(lagSec)
                            .arg(_epochQueue.size()));
            }
            pushLog(QString("PPP filter state retained: source=%1 dropped lagged epoch")
                        .arg(oldest.sourceId));
            continue;
        }

        _epochQueue.removeFirst();
        const bool gfsEnabled =
            _engine.options().troposphereMode == PppTroposphereMode::GfsZwdInitialization;
        const bool hasConfiguredCoordinate =
            _engine.options().hasApriori() ||
            _engine.options().crdStationsBySource.value(oldest.sourceId).valid;
        const bool gfsPending = gfsEnabled && _gfsPendingSources.contains(oldest.sourceId);
        if (gfsPending) {
            ++_gfsDroppedEpochs[oldest.sourceId];
            continue;
        }
        const bool gfsReady = !gfsEnabled || _gfsPreparedSources.contains(oldest.sourceId);
        const bool needsBancroftSeed = gfsEnabled && !gfsReady && !hasConfiguredCoordinate;
        if (gfsEnabled && !gfsReady && hasConfiguredCoordinate) {
            if (requestGfsPrior(oldest)) {
                pushLog(QString("GFS initialization started asynchronously: source=%1")
                            .arg(oldest.sourceId));
                continue;
            }
        }
        if (oldest.gpsTimeValid && _lastProcessedTimeBySource.contains(oldest.sourceId)) {
            const GnssUtils::GpsTime oldestTime =
                GnssUtils::normalizeGpsTime(oldest.gpsWeek, oldest.gpsSow);
            const GnssUtils::GpsTime previousTime =
                _lastProcessedTimeBySource.value(oldest.sourceId);
            const double epochDt = GnssUtils::gpsTimeDifferenceSeconds(oldestTime, previousTime);
            if (epochDt < -0.5) {
                pushLog(QString("PPP epoch discarded: source=%1 outOfOrderGap=%2s")
                            .arg(oldest.sourceId)
                            .arg(epochDt, 0, 'f', 3));
                continue;
            }
            if (epochDt < 0.0 || epochDt > 60.0) {
                pushLog(QString("PPP filter state retained: source=%1 epochGap=%2s")
                            .arg(oldest.sourceId)
                            .arg(epochDt, 0, 'f', 3));
            }
        }

        PppSolution solution;
        bool accepted = _engine.processMsm(oldest.msm, oldest.sourceId, oldest.gloChannels, solution);
        if (needsBancroftSeed) {
            std::array<double, 3> bancroftXyz{};
            if (_engine.lastBancroftPosition(oldest.sourceId, bancroftXyz)) {
                // The engine deliberately rejects changing a prior after a
                // filter exists. Remove only this provisional seed filter
                // before fetching and installing the GFS prior.
                _engine.discardGfsSeedEpoch(oldest.sourceId,
                                            oldest.gpsWeek, oldest.gpsSow);
                const bool requested = requestGfsPrior(oldest, &bancroftXyz);
                // This epoch is a position seed only. It must never be
                // published or used to initialize the PPP filter; the next
                // epoch starts after the asynchronous GFS result arrives.
                if (requested) {
                    accepted = false;
                    pushLog(QString("GFS seed epoch discarded: source=%1 status=pending")
                                .arg(oldest.sourceId));
                }
            } else {
                pushLog(QString("GFS Bancroft seed unavailable: source=%1; retaining fallback solution")
                            .arg(oldest.sourceId));
            }
        }
        if (accepted) {
            if (_stopRequested.loadAcquire()) {
                _active = false;
                clearRuntimeState(true);
                return;
            }
            if (oldest.gpsTimeValid) {
                _lastProcessedTimeBySource[oldest.sourceId] =
                    GnssUtils::normalizeGpsTime(oldest.gpsWeek, oldest.gpsSow);
            }
            SolutionEvent event;
            event.sourceId = oldest.sourceId;
            event.solution = solution;
            event.yawDebug = _engine.lastYawDebug();
            event.antexWarnings = _engine.lastAntexWarnings();
            {
                QMutexLocker locker(&_pendingMutex);
                _pendingSolutions.push_back(event);
            }
            emit ready();
        } else if (_stopRequested.loadAcquire()) {
            _active = false;
            clearRuntimeState(true);
            return;
        } else if (!_engine.lastFailureReason().isEmpty() &&
                   shouldEmitLog(_lastProcessFailLogUtc, 2)) {
            pushLog(QString("PPP no solution: source=%1 %2")
                        .arg(oldest.sourceId, _engine.lastFailureReason()));
        }
    }
}

bool PppWorker::requestGfsPrior(const PendingEpoch& epoch,
                                const std::array<double, 3>* seedXyz) {
    if (_gfsPreparedSources.contains(epoch.sourceId) ||
        _gfsPendingSources.contains(epoch.sourceId) ||
        _engine.options().troposphereMode != PppTroposphereMode::GfsZwdInitialization) {
        return false;
    }

    std::array<double, 3> ecef = {_engine.options().aprXyz[0],
                                  _engine.options().aprXyz[1],
                                  _engine.options().aprXyz[2]};
    const PppCrdStationOptions station = _engine.options().crdStationsBySource.value(epoch.sourceId);
    if (station.valid) {
        ecef = {station.aprXyz[0], station.aprXyz[1], station.aprXyz[2]};
    } else if (seedXyz) {
        ecef = *seedXyz;
    }
    double latitudeDeg = 0.0;
    double longitudeDeg = 0.0;
    double heightM = 0.0;
    if (!GfsZwdProvider::ecefToGeodetic(ecef, latitudeDeg, longitudeDeg, heightM)) {
        // RTCM 1006 may arrive after the first MSM. Leave this source eligible
        // so that a later epoch can start the GFS request once its ECEF is known.
        pushLog(QString("GFS ZWD prior deferred: source=%1 reason=no valid receiver coordinate")
                    .arg(epoch.sourceId));
        return false;
    }
    GfsZwdRequest request;
    request.requestedUtc = GnssUtils::dateTimeFromGpsWeek(epoch.gpsWeek, epoch.gpsSow);
    request.latitudeDeg = latitudeDeg;
    request.longitudeDeg = longitudeDeg;
    request.receiverHeightM = heightM;
    request.cacheDirectory = _engine.options().gfsCacheDirectory;
    request.downloadBaseUrl = _engine.options().gfsDownloadBaseUrl;
    request.timeoutSec = _engine.options().gfsPrefetchTimeoutSec;
    request.maxAgeSec = _engine.options().gfsMaxAgeSec;
    const quint64 requestId = ++_nextGfsRequestId;
    _gfsPendingSources.insert(epoch.sourceId);
    _gfsRequestIds.insert(epoch.sourceId, requestId);
    emit gfsPriorRequested(epoch.sourceId, requestId, request,
                           _engine.options().gfsZwdMethod);
    return true;
}

void PppWorker::completeGfsPrior(const QString& sourceId, quint64 requestId, bool ok,
                                 const PppTropospherePrior& prior,
                                 const QString& error, int droppedEpochs) {
    if (!_gfsPendingSources.contains(sourceId) ||
        _gfsRequestIds.value(sourceId, 0) != requestId) {
        return;
    }
    _gfsPendingSources.remove(sourceId);
    _gfsRequestIds.remove(sourceId);
    _gfsPreparedSources.insert(sourceId);
    const int localDropped = _gfsDroppedEpochs.take(sourceId) + droppedEpochs;
    if (ok && prior.valid) {
        _engine.setTropospherePrior(sourceId, prior);
        pushLog(QString("GFS ZWD prior ready: source=%1 zwd=%2m status=%3 product=%4+f%5 selection=%6 droppedDuringInit=%7")
                     .arg(sourceId)
                     .arg(prior.zwdM, 0, 'f', 4)
                     .arg(prior.status)
                     .arg(prior.cycle)
                     .arg(prior.forecastLeadSec / 3600, 3, 10, QLatin1Char('0'))
                     .arg(prior.productSelection.isEmpty() ? QStringLiteral("nominal") : prior.productSelection)
                     .arg(localDropped));
    } else {
        PppTropospherePrior fallback = prior;
        fallback.valid = false;
        fallback.source = QStringLiteral("GFS profile integral");
        fallback.status = QStringLiteral("gfs_unavailable_fallback");
        fallback.reason = error.isEmpty() ? QStringLiteral("invalid GFS prior") : error;
        _engine.setTropospherePrior(sourceId, fallback);
        pushLog(QString("GFS ZWD prior fallback: source=%1 reason=%2 droppedDuringInit=%3")
                    .arg(sourceId, fallback.reason).arg(localDropped));
    }
    processQueue();
}

bool PppWorker::shouldEmitLog(QDateTime& lastUtc, int intervalSec) const {
    const QDateTime nowUtc = QDateTime::currentDateTimeUtc();
    if (!lastUtc.isValid() || lastUtc.secsTo(nowUtc) >= intervalSec) {
        lastUtc = nowUtc;
        return true;
    }
    return false;
}

void PppWorker::pushLog(const QString& text) {
    if (!_active || _stopRequested.loadAcquire()) {
        return;
    }
    {
        QMutexLocker locker(&_pendingMutex);
        _pendingLogs.push_back(text);
    }
    emit ready();
}
