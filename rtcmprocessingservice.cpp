// Processes RTCM off the UI thread and publishes immutable preparation snapshots.
#include "rtcmprocessingservice.h"

#include "gnss_utils.h"
#include "gnss_signals.h"

#include <QMutexLocker>
#include <QMetaObject>
#include <QSet>

#include <algorithm>
#include <cmath>
#include <iterator>
#include <limits>
#include <utility>

namespace {

void normalizeUniqueIntVector(QVector<int>& values) {
    std::sort(values.begin(), values.end());
    values.erase(std::unique(values.begin(), values.end()), values.end());
}

void mergeMsm(RtcmMsmMessage& destination, const RtcmMsmMessage& source) {
    if (destination.system.isNull()) {
        destination = source;
        normalizeUniqueIntVector(destination.signalIds);
        return;
    }

    destination.messageType = source.messageType;
    destination.msmSubtype = source.msmSubtype;
    destination.stationId = source.stationId;
    destination.epochTime = source.epochTime;
    destination.glonassDay = source.glonassDay;
    destination.multipleMessage = source.multipleMessage;
    destination.iods = source.iods;
    destination.clockSteeringIndicator = source.clockSteeringIndicator;
    destination.externalClockIndicator = source.externalClockIndicator;
    destination.smoothingIndicator = source.smoothingIndicator;
    destination.smoothingInterval = source.smoothingInterval;
    destination.gpsTimeValid = source.gpsTimeValid;
    destination.gpsWeek = source.gpsWeek;
    destination.gpsSow = source.gpsSow;

    destination.signalIds += source.signalIds;
    normalizeUniqueIntVector(destination.signalIds);

    QMap<int, int> satelliteIndex;
    for (int i = 0; i < destination.satellites.size(); ++i) {
        satelliteIndex[destination.satellites[i].satId] = i;
    }
    for (const RtcmMsmSatelliteData& satellite : source.satellites) {
        const auto it = satelliteIndex.constFind(satellite.satId);
        if (it == satelliteIndex.constEnd()) {
            satelliteIndex[satellite.satId] = destination.satellites.size();
            destination.satellites.push_back(satellite);
        } else if (destination.satellites[it.value()].roughRangeIntegerMillis < 0 &&
                   satellite.roughRangeIntegerMillis >= 0) {
            destination.satellites[it.value()] = satellite;
        }
    }

    QSet<QString> cellKeys;
    for (const RtcmMsmCellData& cell : destination.cells) {
        cellKeys.insert(QStringLiteral("%1:%2").arg(cell.satId).arg(cell.signalId));
    }
    for (const RtcmMsmCellData& cell : source.cells) {
        const QString key = QStringLiteral("%1:%2").arg(cell.satId).arg(cell.signalId);
        if (!cellKeys.contains(key)) {
            destination.cells.push_back(cell);
            cellKeys.insert(key);
        }
    }
}

qint64 msmEpochKey(const RtcmMsmMessage& msm) {
    return (static_cast<qint64>(msm.glonassDay) << 32) ^
           static_cast<quint32>(msm.epochTime);
}

QString sourceSatelliteKey(const QString& sourceId, const QString& satelliteKey) {
    return QStringLiteral("%1|%2").arg(sourceId, satelliteKey);
}

int navTypeFor(QChar system, int satelliteNumber) {
    switch (system.toLatin1()) {
    case 'G':
    case 'J':
    case 'I': return RtcmNavLnav;
    case 'R': return RtcmNavFdmaM;
    case 'E': return RtcmNavInav;
    case 'C':
        return satelliteNumber < 6 || (satelliteNumber > 58 && satelliteNumber < 63)
            ? RtcmNavD2 : RtcmNavD1;
    case 'S': return RtcmNavSbasL1;
    default: return RtcmNavUnknown;
    }
}

double wrappedTowDifference(double first, double second) {
    double difference = first - second;
    while (difference > 302400.0) difference -= 604800.0;
    while (difference < -302400.0) difference += 604800.0;
    return difference;
}

QString datumText(int satRefDatum, const QString& mountpoint) {
    if (mountpoint.contains(QStringLiteral("SSRA"), Qt::CaseInsensitive)) return QStringLiteral("A(APC)");
    if (mountpoint.contains(QStringLiteral("SSRC"), Qt::CaseInsensitive)) return QStringLiteral("C(CoM)");
    if (satRefDatum == 0) return QStringLiteral("C?/ITRF");
    if (satRefDatum == 1) return QStringLiteral("A?/LOCAL");
    return QStringLiteral("-");
}

bool validEcef(const std::array<double, 3>& position) {
    return std::isfinite(position[0]) && std::isfinite(position[1]) &&
           std::isfinite(position[2]) &&
           (std::fabs(position[0]) > 1.0e-6 || std::fabs(position[1]) > 1.0e-6 ||
            std::fabs(position[2]) > 1.0e-6);
}

bool azimuthElevation(const std::array<double, 3>& receiver,
                      const std::array<double, 3>& satellite,
                      double& azimuth,
                      double& elevation) {
    if (!validEcef(receiver) || !validEcef(satellite)) return false;
    const double flattening = 1.0 / GnssUtils::kFInv;
    const double eccentricity2 = flattening * (2.0 - flattening);
    const double longitude = std::atan2(receiver[1], receiver[0]);
    const double horizontal = std::hypot(receiver[0], receiver[1]);
    double latitude = std::atan2(receiver[2], horizontal * (1.0 - eccentricity2));
    double height = 0.0;
    for (int iteration = 0; iteration < 10; ++iteration) {
        const double sinLatitude = std::sin(latitude);
        const double radius = GnssUtils::kAell /
            std::sqrt(1.0 - eccentricity2 * sinLatitude * sinLatitude);
        height = horizontal / std::cos(latitude) - radius;
        const double updated = std::atan2(
            receiver[2], horizontal * (1.0 - eccentricity2 * radius / (radius + height)));
        if (std::fabs(updated - latitude) < 1.0e-12) {
            latitude = updated;
            break;
        }
        latitude = updated;
    }
    const std::array<double, 3> difference = {
        satellite[0] - receiver[0], satellite[1] - receiver[1], satellite[2] - receiver[2]
    };
    const double sinLatitude = std::sin(latitude);
    const double cosLatitude = std::cos(latitude);
    const double sinLongitude = std::sin(longitude);
    const double cosLongitude = std::cos(longitude);
    const double north = -sinLatitude * cosLongitude * difference[0] -
                         sinLatitude * sinLongitude * difference[1] +
                         cosLatitude * difference[2];
    const double east = -sinLongitude * difference[0] + cosLongitude * difference[1];
    const double up = cosLatitude * cosLongitude * difference[0] +
                      cosLatitude * sinLongitude * difference[1] +
                      sinLatitude * difference[2];
    const double localHorizontal = std::hypot(north, east);
    if (localHorizontal < 1.0e-6) return false;
    azimuth = std::atan2(east, north);
    if (azimuth < 0.0) azimuth += 2.0 * GnssUtils::kPi;
    elevation = std::atan2(up, localHorizontal);
    return std::isfinite(azimuth) && std::isfinite(elevation);
}

} // namespace

RtcmProcessingService::RtcmProcessingService(QObject* parent)
    : QObject(parent) {
}

void RtcmProcessingService::takePending(QVector<Result>& results) {
    QMutexLocker locker(&_pendingMutex);
    results.swap(_pendingResults);
}

void RtcmProcessingService::requestStop() {
    _accepting.storeRelease(0);
}

void RtcmProcessingService::submitPayload(
    const QString& sourceId,
    const QByteArray& payload,
    const std::array<double, 3>& receiverEcef) {
    if (!_accepting.loadAcquire() || sourceId.isEmpty() || payload.isEmpty()) {
        return;
    }

    bool schedule = false;
    {
        QMutexLocker locker(&_ingressMutex);
        if (!_accepting.loadAcquire()) return;

        // Adjacent socket chunks for one source are equivalent to one larger
        // byte stream.  This removes one queued Qt event per network read and
        // preserves RTCM frame boundaries for the decoder.
        if (!_ingress.isEmpty() && _ingress.last().sourceId == sourceId) {
            _ingress.last().payload += payload;
            _ingress.last().receiverEcef = receiverEcef;
        } else {
            PendingPayload pending;
            pending.sourceId = sourceId;
            pending.payload = payload;
            pending.receiverEcef = receiverEcef;
            _ingress.push_back(std::move(pending));
        }
        schedule = _ingressScheduled.testAndSetOrdered(0, 1);
    }
    if (schedule) {
        QMetaObject::invokeMethod(this, "drainIngress", Qt::QueuedConnection);
    }
}

void RtcmProcessingService::drainIngress() {
    constexpr int kMaxPayloadsPerTurn = 64;
    for (int count = 0; count < kMaxPayloadsPerTurn; ++count) {
        PendingPayload pending;
        {
            QMutexLocker locker(&_ingressMutex);
            if (_ingress.isEmpty()) break;
            pending = std::move(_ingress.front());
            _ingress.removeFirst();
        }
        enqueuePayload(pending.sourceId, pending.payload, pending.receiverEcef);
    }

    _ingressScheduled.storeRelease(0);
    bool schedule = false;
    {
        QMutexLocker locker(&_ingressMutex);
        if (!_ingress.isEmpty()) {
            schedule = _ingressScheduled.testAndSetOrdered(0, 1);
        }
    }
    if (schedule) {
        QMetaObject::invokeMethod(this, "drainIngress", Qt::QueuedConnection);
    }
}

void RtcmProcessingService::drainAllIngress() {
    for (;;) {
        {
            QMutexLocker locker(&_ingressMutex);
            if (_ingress.isEmpty()) break;
        }
        drainIngress();
    }
}

void RtcmProcessingService::enqueuePayload(
    const QString& sourceId,
    const QByteArray& payload,
    const std::array<double, 3>& receiverEcef) {
    if (!_accepting.loadAcquire() || sourceId.isEmpty() || payload.isEmpty()) {
        return;
    }

    Rtcm3Decoder& decoder = _decoders[sourceId];
    if (payload.indexOf(char(0xD3)) < 0 && !decoder.hasBufferedData()) {
        return;
    }

    RtcmDecodeBatch batch = decoder.pushBytes(payload);
    if (batch.messages.isEmpty() && batch.crcErrors == 0 && batch.malformedFrames == 0) {
        return;
    }

    Result result;
    result.sourceId = sourceId;
    result.batch.crcErrors = batch.crcErrors;
    result.batch.malformedFrames = batch.malformedFrames;
    const QDateTime receivedUtc = QDateTime::currentDateTimeUtc();
    bool preparationStateChanged = false;
    const bool observationSuppressed = _observationSuppressedSources.contains(sourceId);
    const bool anyObservationSuppressed = !_observationSuppressedSources.isEmpty();
    for (RtcmDecodedMessage& message : batch.messages) {
        const bool containsMsm = message.hasMsm;
        if (message.hasBroadcastEphemeris) {
            ingestBroadcast(sourceId, message.broadcastEphemeris);
            preparationStateChanged = true;
            if (anyObservationSuppressed) {
                emit broadcastAvailable(sourceId, message.broadcastEphemeris);
            }
        }
        if (message.hasSsr) {
            ingestSsr(sourceId, message.ssr, receivedUtc, result.ssrTelemetryEvents,
                      true);
            preparationStateChanged = true;
            if (anyObservationSuppressed) {
                emit ssrAvailable(sourceId, message.ssr);
            }
        }
        if (message.hasMsm && !observationSuppressed) {
            if (!acceptObservationMessage(message.msm, receivedUtc)) {
                continue;
            }
            result.msmStatusUpdates.push_back(
                updateMsmStatus(sourceId, message.msm, receivedUtc, receiverEcef));
        }
        if (containsMsm && observationSuppressed) {
            // Keep SSR/broadcast payloads in mixed frames, but remove the
            // observation leg so the GUI cannot perform MSM-side work while
            // this source is waiting for its GFS prior.
            if (!message.hasSsr && !message.hasBroadcastEphemeris && !message.hasVtec) {
                continue;
            }
            message.hasMsm = false;
        }
        if (anyObservationSuppressed && !message.hasMsm &&
            (message.hasSsr || message.hasBroadcastEphemeris) && !message.hasVtec) {
            // The direct product signals above preserve the PPP inputs. Do
            // not send this high-rate state-only message through the GUI
            // queue while it is waiting for the GFS initialization callback.
            continue;
        }
        result.batch.messages.push_back(std::move(message));
    }
    if (preparationStateChanged && !anyObservationSuppressed) {
        recomputePreparedStates(result);
    }
    if (result.batch.messages.isEmpty() && result.batch.crcErrors == 0 &&
        result.batch.malformedFrames == 0 && result.ssrTelemetryEvents.isEmpty()) {
        return;
    }

    {
        QMutexLocker locker(&_pendingMutex);
        if (!_accepting.loadAcquire()) {
            return;
        }
        _pendingResults.push_back(std::move(result));
    }
    emit ready();
}

void RtcmProcessingService::setObservationSampleSec(int sampleSec) {
    const int normalized = qMax(1, sampleSec);
    if (_observationSampleSec == normalized) {
        return;
    }
    _observationSampleSec = normalized;
}

void RtcmProcessingService::setObservationSuppressed(const QString& sourceId,
                                                     bool suppressed) {
    const QString key = sourceId.trimmed();
    if (key.isEmpty()) return;
    if (suppressed) {
        _observationSuppressedSources.insert(key);
    } else {
        _observationSuppressedSources.remove(key);
        if (!_observationSuppressedSources.isEmpty()) {
            return;
        }
        Result result;
        recomputePreparedStates(result);
        {
            QMutexLocker locker(&_pendingMutex);
            if (_accepting.loadAcquire()) {
                _pendingResults.push_back(std::move(result));
            }
        }
        emit ready();
    }
}

void RtcmProcessingService::setDataPreparationConfig(
    const DataPreparationConfig& config) {
    _dataConfig = config;
    Result result;
    recomputePreparedStates(result);
    {
        QMutexLocker locker(&_pendingMutex);
        if (!_accepting.loadAcquire()) return;
        _pendingResults.push_back(std::move(result));
    }
    emit ready();
}

bool RtcmProcessingService::acceptObservationMessage(
    const RtcmMsmMessage& msm,
    const QDateTime& receivedUtc) {
    return GnssUtils::isGpsTimeOnSampleGrid(
        GnssUtils::gpsTimeFromMsm(msm, receivedUtc), _observationSampleSec);
}

RtcmProcessingService::MsmStatusUpdate RtcmProcessingService::updateMsmStatus(
    const QString& sourceId,
    const RtcmMsmMessage& msm,
    const QDateTime& receivedUtc,
    const std::array<double, 3>& receiverEcef) {
    MsmStatusUpdate update;
    QMap<QChar, RtcmMsmMessage>& sourceMessages = _lastMsmMessages[sourceId];
    update.firstForSource = sourceMessages.isEmpty();
    RtcmMsmMessage& merged = sourceMessages[msm.system];
    const bool sameEpoch = !merged.system.isNull() &&
                           merged.stationId == msm.stationId &&
                           merged.system == msm.system &&
                           merged.epochTime == msm.epochTime &&
                           merged.glonassDay == msm.glonassDay;
    if (sameEpoch) {
        mergeMsm(merged, msm);
    } else {
        merged = msm;
        normalizeUniqueIntVector(merged.signalIds);
    }

    update.mergedMessage = merged;
    update.epochKey = msmEpochKey(merged);
    MsmSystemSummary& summary = update.summary;
    summary.system = merged.system;
    summary.messageType = merged.messageType;
    summary.epochTime = merged.epochTime;
    summary.satCount = merged.satellites.size();
    summary.signalCount = merged.signalIds.size();
    summary.cellCount = merged.cells.size();
    summary.receivedUtc = receivedUtc;
    QMap<int, QPair<double, int>> cnrStats;
    for (const RtcmMsmCellData& cell : merged.cells) {
        if (cell.hasCnr) {
            auto& entry = cnrStats[cell.satId];
            entry.first += cell.cnrDbHz;
            entry.second += 1;
        }
    }
    for (auto it = cnrStats.constBegin(); it != cnrStats.constEnd(); ++it) {
        if (it.value().second > 0) {
            summary.satCnr[it.key()] =
                it.value().first / static_cast<double>(it.value().second);
        }
    }
    const GnssUtils::GpsTime msmTime = merged.gpsTimeValid
        ? GnssUtils::normalizeGpsTime(merged.gpsWeek, merged.gpsSow)
        : GnssUtils::gpsTimeFromMsm(merged, receivedUtc);
    update.gpsWeek = msmTime.week;
    update.gpsSow = msmTime.sow;
    computeSky(merged, receiverEcef, update);
    computeCorrectedObservations(sourceId, merged, update);
    update.glonassChannels = _glonassChannelsBySource.value(sourceId);
    return update;
}

void RtcmProcessingService::ingestBroadcast(
    const QString& sourceId, const RtcmBroadcastEphemeris& ephemeris) {
    if (!ephemeris.sat.isValid()) return;
    BroadcastState state;
    state.eph = ephemeris;
    state.sourceId = sourceId;
    state.receivedUtc = QDateTime::currentDateTimeUtc();
    GnssEphemeris::fromRtcm(ephemeris, state.model);
    _broadcastStates[sourceSatelliteKey(sourceId, ephemeris.sat.key())] = std::move(state);
}

void RtcmProcessingService::ingestSsr(
    const QString& sourceId,
    const RtcmSsrMessage& message,
    const QDateTime& receivedUtc,
    QVector<SsrTelemetryEvent>& telemetryEvents,
    bool collectTelemetry) {
    if (message.satellites.isEmpty()) return;
    const QDateTime epochUtc = GnssUtils::gpsTimeFromSsr(
        message.system, message.epochTime, receivedUtc).utc;

    for (const RtcmSsrSatelliteData& satellite : message.satellites) {
        if (!satellite.sat.isValid()) continue;
        if (collectTelemetry) {
            SsrTelemetryEvent telemetry;
            telemetry.sourceId = sourceId;
            telemetry.receivedUtc = receivedUtc;
            telemetry.epochUtc = epochUtc;
            telemetry.message = message;
            telemetry.satellite = satellite;

            QString selectedSource;
            bool sourceAccepted = false;
            switch (message.kind) {
            case RtcmSsrCorrectionKind::Orbit:
                selectedSource = _dataConfig.orbitStreamId;
                sourceAccepted = sourceId == selectedSource;
                break;
            case RtcmSsrCorrectionKind::Clock:
                selectedSource = _dataConfig.clockStreamId;
                sourceAccepted = sourceId == selectedSource;
                break;
            case RtcmSsrCorrectionKind::OrbitClock:
                sourceAccepted = sourceId == _dataConfig.orbitStreamId ||
                                 sourceId == _dataConfig.clockStreamId;
                break;
            case RtcmSsrCorrectionKind::CodeBias:
                selectedSource = _dataConfig.codeBiasStreamId;
                sourceAccepted = sourceId == selectedSource;
                break;
            case RtcmSsrCorrectionKind::PhaseBias:
                selectedSource = _dataConfig.phaseBiasStreamId;
                sourceAccepted = sourceId == selectedSource;
                break;
            default:
                break;
            }
            telemetry.adopted = _dataConfig.enabled && sourceAccepted;
            if (!telemetry.adopted) {
                telemetry.reason = QStringLiteral("rejected");
            } else if (message.kind != RtcmSsrCorrectionKind::CodeBias &&
                       message.kind != RtcmSsrCorrectionKind::PhaseBias) {
                const auto broadcast = _broadcastStates.constFind(
                    sourceSatelliteKey(_dataConfig.broadcastStreamId, satellite.sat.key()));
                if (broadcast == _broadcastStates.constEnd()) {
                    telemetry.adopted = false;
                    telemetry.reason = QStringLiteral("unknown");
                } else if (satellite.iod >= 0 && satellite.iod != broadcast->eph.iod) {
                    telemetry.adopted = false;
                    telemetry.reason = QStringLiteral("iod_mismatch");
                } else {
                    const double updateSeconds =
                        GnssUtils::ssrUpdateIntervalSeconds(message.updateIntervalIndex);
                    if (epochUtc.isValid() && updateSeconds > 0.0 &&
                        epochUtc.secsTo(receivedUtc) > qMax(60.0, 2.0 * updateSeconds)) {
                        telemetry.adopted = false;
                        telemetry.reason = QStringLiteral("stale");
                    }
                }
            }
            telemetryEvents.push_back(std::move(telemetry));
        }

        const QString key = sourceSatelliteKey(sourceId, satellite.sat.key());
        if (message.kind == RtcmSsrCorrectionKind::Orbit ||
            message.kind == RtcmSsrCorrectionKind::OrbitClock) {
            OrbitCorrectionState correction;
            correction.sat = satellite.sat;
            correction.sourceId = sourceId;
            correction.iod = satellite.iod;
            correction.updateIntervalIndex = message.updateIntervalIndex;
            correction.satRefDatum = message.satRefDatum;
            correction.epochTime = message.epochTime;
            correction.xr[0] = satellite.deltaRadial;
            correction.xr[1] = satellite.deltaAlongTrack;
            correction.xr[2] = satellite.deltaCrossTrack;
            correction.dotXr[0] = satellite.dotDeltaRadial;
            correction.dotXr[1] = satellite.dotDeltaAlongTrack;
            correction.dotXr[2] = satellite.dotDeltaCrossTrack;
            correction.receivedUtc = receivedUtc;
            _orbitCorrections[key] = correction;
        }
        if (message.kind == RtcmSsrCorrectionKind::Clock ||
            message.kind == RtcmSsrCorrectionKind::OrbitClock) {
            ClockCorrectionState correction;
            correction.sat = satellite.sat;
            correction.sourceId = sourceId;
            correction.iod = satellite.iod;
            correction.updateIntervalIndex = message.updateIntervalIndex;
            correction.epochTime = message.epochTime;
            correction.dClk = satellite.deltaC0 / GnssUtils::kC;
            correction.dotDClk = satellite.deltaC1 / GnssUtils::kC;
            correction.dotDotDClk = satellite.deltaC2 / GnssUtils::kC;
            correction.receivedUtc = receivedUtc;
            _clockCorrections[key] = correction;
        }
        if (message.kind == RtcmSsrCorrectionKind::CodeBias) {
            CodeBiasState correction;
            correction.sat = satellite.sat;
            correction.messageType = message.messageType;
            correction.biases = satellite.codeBiases;
            correction.receivedUtc = receivedUtc;
            _codeBiases[key] = correction;
        }
        if (message.kind == RtcmSsrCorrectionKind::PhaseBias) {
            PhaseBiasState correction;
            correction.sat = satellite.sat;
            correction.messageType = message.messageType;
            correction.biases = satellite.phaseBiases;
            correction.receivedUtc = receivedUtc;
            _phaseBiases[key] = correction;
        }
    }
}

void RtcmProcessingService::recomputePreparedStates(Result& result) {
    result.preparedStatesChanged = true;
    const GnssUtils::GpsTime now = GnssUtils::currentGpsTime();
    for (auto it = _broadcastStates.constBegin(); it != _broadcastStates.constEnd(); ++it) {
        const BroadcastState& broadcast = it.value();
        if (broadcast.sourceId != _dataConfig.broadcastStreamId) continue;
        PreparedState prepared;
        prepared.sat = broadcast.eph.sat;
        prepared.broadcastIod = broadcast.eph.iod;
        prepared.updatedUtc = broadcast.receivedUtc;
        const QString satelliteKey = broadcast.eph.sat.key();

        if (!_dataConfig.orbitStreamId.isEmpty()) {
            const OrbitCorrectionState correction = _orbitCorrections.value(
                sourceSatelliteKey(_dataConfig.orbitStreamId, satelliteKey));
            if (correction.sat.isValid()) {
                const bool iodMatches = correction.iod < 0 || correction.iod == prepared.broadcastIod;
                prepared.orbitApplied = _dataConfig.enabled && iodMatches;
                prepared.orbitStatus = iodMatches ? QStringLiteral("OK")
                                                  : QStringLiteral("IOD mismatch %1").arg(correction.iod);
                prepared.satRefDatum = correction.satRefDatum;
                prepared.datumText = datumText(correction.satRefDatum, _dataConfig.orbitMountpoint);
                std::copy(std::begin(correction.xr), std::end(correction.xr), prepared.orbitRsw);
                const GnssUtils::GpsTime epoch = GnssUtils::gpsTimeFromSsr(
                    correction.sat.system, correction.epochTime, now.utc);
                double dt = wrappedTowDifference(now.sow, epoch.sow);
                const double interval = GnssUtils::ssrUpdateIntervalSeconds(correction.updateIntervalIndex);
                if (interval > 0.0) dt -= 0.5 * interval;
                prepared.orbitDtSec = dt;
                prepared.updatedUtc = correction.receivedUtc;
            }
        }
        if (!_dataConfig.clockStreamId.isEmpty()) {
            const ClockCorrectionState correction = _clockCorrections.value(
                sourceSatelliteKey(_dataConfig.clockStreamId, satelliteKey));
            if (correction.sat.isValid()) {
                const bool iodMatches = correction.iod < 0 || correction.iod == prepared.broadcastIod;
                prepared.clockApplied = _dataConfig.enabled && iodMatches;
                prepared.clockStatus = iodMatches ? QStringLiteral("OK")
                                                  : QStringLiteral("IOD mismatch %1").arg(correction.iod);
                const GnssUtils::GpsTime epoch = GnssUtils::gpsTimeFromSsr(
                    correction.sat.system, correction.epochTime, now.utc);
                double dt = wrappedTowDifference(now.sow, epoch.sow);
                const double interval = GnssUtils::ssrUpdateIntervalSeconds(correction.updateIntervalIndex);
                if (interval > 0.0) dt -= 0.5 * interval;
                prepared.clockDtSec = dt;
                prepared.clockSec = correction.dClk + correction.dotDClk * dt +
                                    correction.dotDotDClk * dt * dt;
                prepared.updatedUtc = correction.receivedUtc;
            }
        }
        const CodeBiasState codeBias = _codeBiases.value(
            sourceSatelliteKey(_dataConfig.codeBiasStreamId, satelliteKey));
        if (codeBias.sat.isValid()) {
            prepared.codeApplied = _dataConfig.enabled;
            prepared.codeBiasCount = codeBias.biases.size();
            prepared.updatedUtc = codeBias.receivedUtc;
        }
        const PhaseBiasState phaseBias = _phaseBiases.value(
            sourceSatelliteKey(_dataConfig.phaseBiasStreamId, satelliteKey));
        if (phaseBias.sat.isValid()) {
            prepared.phaseApplied = _dataConfig.enabled;
            prepared.phaseBiasCount = phaseBias.biases.size();
            prepared.updatedUtc = phaseBias.receivedUtc;
        }
        if (prepared.datumText.isEmpty()) {
            prepared.datumText = datumText(prepared.satRefDatum, _dataConfig.broadcastMountpoint);
        }
        result.preparedStates[satelliteKey] = prepared;
    }

    if (!_dataConfig.enabled || !_dataConfig.sp3Enabled ||
        _dataConfig.broadcastStreamId.isEmpty()) return;
    double outputSow = now.sow;
    if (_dataConfig.sp3SampleSec > 0) {
        outputSow = std::floor(outputSow / _dataConfig.sp3SampleSec) * _dataConfig.sp3SampleSec;
    }
    for (auto it = result.preparedStates.constBegin(); it != result.preparedStates.constEnd(); ++it) {
        const BroadcastState broadcast = _broadcastStates.value(
            sourceSatelliteKey(_dataConfig.broadcastStreamId, it.key()));
        if (!broadcast.model) continue;
        GnssEphemeris::State baseState;
        if (!broadcast.model->computeState(now.week, outputSow, baseState)) continue;
        SsrOrbitCorrection orbit;
        SsrClockCorrection clock;
        const OrbitCorrectionState orbitState = _orbitCorrections.value(
            sourceSatelliteKey(_dataConfig.orbitStreamId, it.key()));
        if (!_dataConfig.orbitStreamId.isEmpty()) {
            if (!orbitState.sat.isValid() ||
                (orbitState.iod >= 0 && orbitState.iod != it->broadcastIod)) continue;
            orbit.valid = true;
            orbit.iod = orbitState.iod;
            orbit.updateIntervalIndex = orbitState.updateIntervalIndex;
            orbit.epochTime = static_cast<int>(GnssUtils::gpsTimeFromSsr(
                orbitState.sat.system, orbitState.epochTime, now.utc).sow);
            orbit.xr = {orbitState.xr[0], orbitState.xr[1], orbitState.xr[2]};
            orbit.dotXr = {orbitState.dotXr[0], orbitState.dotXr[1], orbitState.dotXr[2]};
        }
        const ClockCorrectionState clockState = _clockCorrections.value(
            sourceSatelliteKey(_dataConfig.clockStreamId, it.key()));
        if (!_dataConfig.clockStreamId.isEmpty()) {
            if (!clockState.sat.isValid() ||
                (clockState.iod >= 0 && clockState.iod != it->broadcastIod)) continue;
            clock.valid = true;
            clock.iod = clockState.iod;
            clock.updateIntervalIndex = clockState.updateIntervalIndex;
            clock.epochTime = static_cast<int>(GnssUtils::gpsTimeFromSsr(
                clockState.sat.system, clockState.epochTime, now.utc).sow);
            clock.dClk = clockState.dClk;
            clock.dotDClk = clockState.dotDClk;
            clock.dotDotDClk = clockState.dotDotDClk;
        }
        GnssEphemeris::State corrected;
        applySsrOrbitClock(baseState, orbit.valid ? &orbit : nullptr,
                           clock.valid ? &clock : nullptr,
                           now.week, outputSow, corrected);
        Sp3Record record;
        record.gpsWeek = now.week;
        record.gpsSow = outputSow;
        record.prn = QStringLiteral("%1%2").arg(it->sat.system).arg(it->sat.number, 2, 10, QLatin1Char('0'));
        record.position = corrected.position;
        record.clockSec = corrected.clockSec;
        record.velocity = corrected.velocity;
        record.clockRate = corrected.clockRate;
        result.sp3Records.push_back(record);
    }
}

void RtcmProcessingService::computeSky(
    const RtcmMsmMessage& msm,
    const std::array<double, 3>& receiverEcef,
    MsmStatusUpdate& update) const {
    update.receiverPositionValid = validEcef(receiverEcef);
    if (!update.receiverPositionValid) return;
    for (const RtcmMsmSatelliteData& rawSatellite : msm.satellites) {
        RtcmSatId satellite;
        satellite.system = msm.system;
        satellite.number = rawSatellite.satId;
        satellite.navType = navTypeFor(msm.system, rawSatellite.satId);
        const BroadcastState* broadcast = nullptr;
        const auto exact = _broadcastStates.constFind(
            sourceSatelliteKey(_dataConfig.broadcastStreamId, satellite.key()));
        if (exact != _broadcastStates.constEnd()) {
            broadcast = &exact.value();
        } else {
            for (auto it = _broadcastStates.constBegin(); it != _broadcastStates.constEnd(); ++it) {
                if (it->eph.sat.system == satellite.system &&
                    it->eph.sat.number == satellite.number) {
                    broadcast = &it.value();
                    break;
                }
            }
        }
        if (!broadcast || !broadcast->model) continue;
        GnssEphemeris::State state;
        if (!broadcast->model->computeState(update.gpsWeek, update.gpsSow, state)) continue;
        double azimuth = 0.0;
        double elevation = 0.0;
        if (!azimuthElevation(receiverEcef, state.position, azimuth, elevation) || elevation < 0.0) continue;
        const QString prn = QStringLiteral("%1%2")
                                .arg(satellite.system)
                                .arg(satellite.number, 2, 10, QLatin1Char('0'));
        update.skyBySatellite[prn] = qMakePair(azimuth, elevation);
    }
}

void RtcmProcessingService::computeCorrectedObservations(
    const QString& sourceId,
    const RtcmMsmMessage& msm,
    MsmStatusUpdate& update) {
    QMap<int, int>& glonassChannels = _glonassChannelsBySource[sourceId];
    QMap<int, RtcmMsmSatelliteData> satelliteData;
    for (const RtcmMsmSatelliteData& satellite : msm.satellites) {
        satelliteData[satellite.satId] = satellite;
        if (msm.system == QLatin1Char('R') && satellite.extendedInfo >= 0 &&
            satellite.extendedInfo < 14) {
            glonassChannels[satellite.satId] = satellite.extendedInfo - 7;
        }
    }
    if (!_dataConfig.enabled || !_dataConfig.rinexEnabled ||
        sourceId != _dataConfig.obsStreamId) return;

    for (const RtcmMsmCellData& cell : msm.cells) {
        const RtcmMsmSatelliteData satellite = satelliteData.value(cell.satId);
        if (satellite.roughRangeIntegerMillis < 0) continue;
        const QString signal = msmSignalToRinexType2ch(msm.system, cell.signalId, msm.signalIds);
        if (signal.isEmpty()) continue;
        const double roughMillis = satellite.roughRangeIntegerMillis + satellite.roughRangeModuloMillis;
        RtcmSatId satelliteId;
        satelliteId.system = msm.system;
        satelliteId.number = cell.satId;
        satelliteId.navType = navTypeFor(msm.system, cell.satId);
        const QString satelliteKey = satelliteId.key();
        const QString prn = QStringLiteral("%1%2").arg(msm.system).arg(cell.satId, 2, 10, QLatin1Char('0'));

        double codeBiasMeters = 0.0;
        const CodeBiasState codeBias = _codeBiases.value(
            sourceSatelliteKey(_dataConfig.codeBiasStreamId, satelliteKey));
        for (const RtcmSsrCodeBiasEntry& bias : codeBias.biases) {
            if (ssrSignalToRinexType2ch(msm.system, bias.signalCode, codeBias.messageType) == signal) {
                codeBiasMeters = bias.biasMeters;
                break;
            }
        }
        double phaseBiasMeters = 0.0;
        const PhaseBiasState phaseBias = _phaseBiases.value(
            sourceSatelliteKey(_dataConfig.phaseBiasStreamId, satelliteKey));
        for (const RtcmSsrPhaseBiasEntry& bias : phaseBias.biases) {
            if (ssrSignalToRinexType2ch(msm.system, bias.signalCode, phaseBias.messageType) == signal) {
                phaseBiasMeters = bias.biasMeters;
                break;
            }
        }
        int glonassChannel = glonassChannels.value(cell.satId, 0);
        BroadcastState broadcast = _broadcastStates.value(
            sourceSatelliteKey(_dataConfig.broadcastStreamId, satelliteKey));
        if (!broadcast.eph.sat.isValid()) {
            for (auto it = _broadcastStates.constBegin(); it != _broadcastStates.constEnd(); ++it) {
                if (it->eph.sat.system == satelliteId.system &&
                    it->eph.sat.number == satelliteId.number) {
                    broadcast = it.value();
                    break;
                }
            }
        }
        if (msm.system == QLatin1Char('R') && !glonassChannels.contains(cell.satId) &&
            broadcast.eph.sat.isValid()) {
            glonassChannel = broadcast.eph.glonassFreqNum;
        }
        QMap<QString, double>& observations = update.correctedObservations[prn];
        if (cell.hasFinePseudorange) {
            observations[QStringLiteral("C") + signal] =
                (roughMillis + cell.finePseudorangeMillis) * 1.0e-3 * GnssUtils::kC +
                codeBiasMeters;
            if (std::fabs(codeBiasMeters) > 1.0e-9) ++update.correctedCodeCount;
        }
        if (cell.hasFinePhaseRange) {
            const double wavelength = signalWavelengthMeters(msm.system, signal, glonassChannel);
            if (wavelength > 0.0) {
                observations[QStringLiteral("L") + signal] =
                    ((roughMillis + cell.finePhaseRangeMillis) * 1.0e-3 * GnssUtils::kC +
                     phaseBiasMeters) / wavelength;
                if (std::fabs(phaseBiasMeters) > 1.0e-9) ++update.correctedPhaseCount;
            }
        }
        if (cell.hasCnr) observations[QStringLiteral("S") + signal] = cell.cnrDbHz;
    }
}

void RtcmProcessingService::clearSource(const QString& sourceId) {
    _observationSuppressedSources.remove(sourceId);
    _decoders.remove(sourceId);
    _lastMsmMessages.remove(sourceId);
    _glonassChannelsBySource.remove(sourceId);
    const QString prefix = sourceId + QLatin1Char('|');
    for (auto it = _orbitCorrections.begin(); it != _orbitCorrections.end();) {
        it = it.key().startsWith(prefix) ? _orbitCorrections.erase(it) : std::next(it);
    }
    for (auto it = _clockCorrections.begin(); it != _clockCorrections.end();) {
        it = it.key().startsWith(prefix) ? _clockCorrections.erase(it) : std::next(it);
    }
    for (auto it = _codeBiases.begin(); it != _codeBiases.end();) {
        it = it.key().startsWith(prefix) ? _codeBiases.erase(it) : std::next(it);
    }
    for (auto it = _phaseBiases.begin(); it != _phaseBiases.end();) {
        it = it.key().startsWith(prefix) ? _phaseBiases.erase(it) : std::next(it);
    }
    for (auto it = _broadcastStates.begin(); it != _broadcastStates.end();) {
        it = it.key().startsWith(prefix) ? _broadcastStates.erase(it) : std::next(it);
    }
    QMutexLocker locker(&_pendingMutex);
    for (auto it = _pendingResults.begin(); it != _pendingResults.end();) {
        if (it->sourceId == sourceId) {
            it = _pendingResults.erase(it);
        } else {
            ++it;
        }
    }
}

void RtcmProcessingService::clearAll() {
    {
        QMutexLocker locker(&_ingressMutex);
        _ingress.clear();
    }
    _ingressScheduled.storeRelease(0);
    _decoders.clear();
    _lastMsmMessages.clear();
    _broadcastStates.clear();
    _orbitCorrections.clear();
    _clockCorrections.clear();
    _codeBiases.clear();
    _phaseBiases.clear();
    _glonassChannelsBySource.clear();
    _observationSuppressedSources.clear();
    QMutexLocker locker(&_pendingMutex);
    _pendingResults.clear();
}

void RtcmProcessingService::shutdown() {
    requestStop();
    clearAll();
}
