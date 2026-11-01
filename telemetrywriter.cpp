// TelemetryWriter serializes structured MSM, SSR, and PPP records to daily files.
#include "telemetrywriter.h"

#include "gnss_signals.h"
#include "gnss_utils.h"

#include <cmath>
#include <limits>
#include <QCryptographicHash>
#include <QDir>
#include <QFileInfo>
#include <QRegularExpression>
#include <QTimer>

namespace {

QStringList msmHeader() {
    return {"source_id", "received_utc", "epoch_utc", "station_id", "system",
            "message_type", "msm_subtype", "multiple_message", "iods",
            "clock_steering_indicator", "external_clock_indicator", "smoothing_indicator",
            "smoothing_interval", "satellite", "sat_id", "signal_id", "signal",
            "cell_active", "has_fine_pseudorange", "fine_pseudorange_ms",
            "has_fine_phase_range", "fine_phase_range_ms", "has_lock_time",
            "lock_time_indicator", "lock_time_seconds", "has_half_cycle",
            "half_cycle_ambiguity", "has_cnr", "cnr_dbhz", "has_fine_phase_rate",
            "fine_phase_rate_mps", "azimuth_deg", "elevation_deg"};
}

QStringList receptionHeader() {
    return {"source_id", "received_utc", "message_family", "message_type", "system",
            "status", "reason", "detail"};
}

QStringList pppResidualHeader() {
    return {"source_id", "epoch_utc", "satellite", "signal",
            "observation_type", "prefit_m", "postfit_m", "sigma_m",
            "elevation_deg", "adopted", "reason"};
}

QStringList headerFor(const QString& family) {
    if (family == "msm") return msmHeader();
    if (family == "ppp_residual") return pppResidualHeader();
    return receptionHeader();
}

QString ssrSourceFileToken(const QString& sourceId) {
    QString readable = sourceId.trimmed();
    readable.replace(QRegularExpression(QStringLiteral("[^A-Za-z0-9_-]+")), QStringLiteral("_"));
    readable = readable.left(40);
    if (readable.isEmpty()) readable = QStringLiteral("source");
    const QByteArray digest = QCryptographicHash::hash(sourceId.toUtf8(), QCryptographicHash::Sha256)
                                  .toHex().left(16);
    return QStringLiteral("%1_%2").arg(readable, QString::fromLatin1(digest));
}

QString fileNameToken(const QString& value, const QString& fallback) {
    QString token = value.trimmed();
    token.replace(QRegularExpression(QStringLiteral("[^A-Za-z0-9_-]+")), QStringLiteral("_"));
    token = token.left(80);
    return token.isEmpty() ? fallback : token;
}

QString ssrRelativePath(const QString& sourceId,
                        const QString& sourceName,
                        int providerId,
                        int solutionId,
                        const QDateTime& gpsEpoch,
                        const QString& family) {
    // Keep receiver connections and SSR providers in separate parent
    // directories so compatible daily names cannot mix correction sources.
    const QString mountPoint = fileNameToken(sourceName, QStringLiteral("source"));
    const QString sourceDirectory = ssrSourceFileToken(sourceId);
    const QString productDirectory = QStringLiteral("p%1_s%2").arg(providerId).arg(solutionId);
    const QString date = QStringLiteral("%1%2%3")
        .arg(gpsEpoch.date().year())
        .arg(gpsEpoch.date().dayOfYear(), 3, 10, QLatin1Char('0'))
        .arg(QStringLiteral("0000"));
    const QString fileName = QStringLiteral("%1_S_%2_01D_%3.ssr")
        .arg(mountPoint, date, family);
    return QStringLiteral("ssr/%1/%2/%3/%4")
        .arg(mountPoint, sourceDirectory, productDirectory, fileName);
}

} // namespace

TelemetryWriter::TelemetryWriter(QObject* parent)
    : QObject(parent), _flushTimer(new QTimer(this)) {
    _flushTimer->setInterval(500);
    connect(_flushTimer, &QTimer::timeout, this, [this]() { flush(); });
}

TelemetryWriter::~TelemetryWriter() {
    close();
}

void TelemetryWriter::setSourceNames(const QMap<QString, QString>& names) {
    _sourceNames = names;
}

QString TelemetryWriter::sourceName(const QString& sourceId) const {
    QString name = _sourceNames.value(sourceId, sourceId).trimmed();
    name.replace(QRegularExpression(QStringLiteral("\\s+")), QStringLiteral("_"));
    return name.isEmpty() ? QStringLiteral("source") : name;
}

void TelemetryWriter::setOutputDirectory(const QString& directory) {
    const QString normalized = directory.trimmed().isEmpty()
        ? QString() : QDir::cleanPath(directory.trimmed());
    if (_directory == normalized) {
        if (isEnabled() && !_flushTimer->isActive()) {
            _flushTimer->start();
        }
        return;
    }

    close();
    _directory = normalized;
    if (isEnabled()) {
        _flushTimer->start();
    }
}

bool TelemetryWriter::isEnabled() const {
    return !_directory.trimmed().isEmpty();
}

void TelemetryWriter::writeMsm(const QString& sourceId,
                               const QDateTime& receivedUtc,
                               const QDateTime& epochUtc,
                               const RtcmMsmMessage& message,
                               const QMap<QString, QPair<double, double>>& skyBySatellite) {
    if (!isEnabled()) return;
    for (const RtcmMsmCellData& cell : message.cells) {
        RtcmSatId satellite;
        satellite.system = message.system;
        satellite.number = cell.satId;
        const QString satelliteId = satelliteText(satellite);
        const QPair<double, double> sky = skyBySatellite.value(satelliteId);
        const bool hasSky = skyBySatellite.contains(satelliteId);
        const QString signal = msmSignalToRinexType2ch(message.system, cell.signalId, message.signalIds);
        appendRow("msm", receivedUtc,
                  {sourceId, utcText(receivedUtc), utcText(epochUtc), QString::number(message.stationId),
                   QString(message.system), QString::number(message.messageType),
                   QString::number(message.msmSubtype), boolText(message.multipleMessage),
                   QString::number(message.iods), QString::number(message.clockSteeringIndicator),
                   QString::number(message.externalClockIndicator), boolText(message.smoothingIndicator),
                   QString::number(message.smoothingInterval), satelliteId, QString::number(cell.satId),
                   QString::number(cell.signalId), signal, boolText(cell.active),
                   boolText(cell.hasFinePseudorange), cell.hasFinePseudorange ? number(cell.finePseudorangeMillis) : QString(),
                   boolText(cell.hasFinePhaseRange), cell.hasFinePhaseRange ? number(cell.finePhaseRangeMillis) : QString(),
                   boolText(cell.hasLockTime), cell.hasLockTime ? QString::number(cell.lockTimeIndicator) : QString(),
                   cell.hasLockTime ? number(cell.lockTimeSec) : QString(), boolText(cell.hasHalfCycle),
                   cell.hasHalfCycle ? QString::number(cell.halfCycleAmbiguity) : QString(), boolText(cell.hasCnr),
                   cell.hasCnr ? number(cell.cnrDbHz) : QString(), boolText(cell.hasFinePhaseRate),
                   cell.hasFinePhaseRate ? number(cell.finePhaseRateMps) : QString(),
                   hasSky ? number(sky.first) : QString(), hasSky ? number(sky.second) : QString()});
    }
}

void TelemetryWriter::writeSsr(const QString& sourceId,
                               const QDateTime& receivedUtc,
                               const QDateTime& epochUtc,
                               const RtcmSsrMessage& message,
                               const RtcmSsrSatelliteData& satellite,
                               bool adopted,
                               const QString& reason) {
    if (!isEnabled()) return;
    Q_UNUSED(receivedUtc);
    Q_UNUSED(adopted);
    Q_UNUSED(reason);

    const QString prn = satelliteText(satellite.sat);
    const QString satKey = QStringLiteral("%1|%2|%3|%4")
        .arg(ssrSourceFileToken(sourceId)).arg(message.providerId).arg(message.solutionId).arg(satellite.sat.key());
    const QString iod = QString::number(satellite.iod).rightJustified(11, QLatin1Char(' '));
    auto fixed = [](double value, int width = 10, int precision = 4) {
        return QString::number(value, 'f', precision).rightJustified(width, QLatin1Char(' '));
    };
    if ((message.kind == RtcmSsrCorrectionKind::Orbit ||
         message.kind == RtcmSsrCorrectionKind::OrbitClock) && satellite.hasOrbit) {
        _orbitIods[satKey] = satellite.iod;
        const QString record = prn + QLatin1Char(' ') + iod + QLatin1Char(' ') +
            fixed(satellite.deltaRadial) + QLatin1Char(' ') +
            fixed(satellite.deltaAlongTrack) + QLatin1Char(' ') +
            fixed(satellite.deltaCrossTrack) + QStringLiteral("    ") +
            fixed(satellite.dotDeltaRadial * 1000.0) + QLatin1Char(' ') +
            fixed(satellite.dotDeltaAlongTrack * 1000.0) + QLatin1Char(' ') +
            fixed(satellite.dotDeltaCrossTrack * 1000.0) + QLatin1Char('\n');
        appendSsrBlock(sourceId, epochUtc, message, satellite, QStringLiteral("ORBIT"), record);
    }
    if ((message.kind == RtcmSsrCorrectionKind::Clock ||
         message.kind == RtcmSsrCorrectionKind::OrbitClock) && satellite.hasClock) {
        RtcmSsrSatelliteData correction = satellite;
        if (correction.iod < 0 && _orbitIods.contains(satKey)) correction.iod = _orbitIods.value(satKey);
        _lastClockCorrections[satKey] = correction;
        _clockSsrIods[satKey] = message.ssrIod;
        const QString record = prn + QLatin1Char(' ') +
            QString::number(correction.iod).rightJustified(11, QLatin1Char(' ')) + QLatin1Char(' ') +
            fixed(satellite.deltaC0) + QLatin1Char(' ') +
            fixed(satellite.deltaC1 * 1000.0) + QLatin1Char(' ') +
            fixed(satellite.deltaC2 * 1000.0) + QLatin1Char('\n');
        appendSsrBlock(sourceId, epochUtc, message, satellite, QStringLiteral("CLOCK"), record);
    }
    if (message.kind == RtcmSsrCorrectionKind::HighRateClock && satellite.hasHighRateClock) {
        auto base = _lastClockCorrections.find(satKey);
        if (base != _lastClockCorrections.end() && _clockSsrIods.value(satKey, -2) == message.ssrIod) {
            RtcmSsrSatelliteData correction = base.value();
            correction.deltaC0 += satellite.highRateClock;
            const QString record = prn + QLatin1Char(' ') +
                QString::number(correction.iod).rightJustified(11, QLatin1Char(' ')) + QLatin1Char(' ') +
                fixed(correction.deltaC0) + QLatin1Char(' ') +
                fixed(correction.deltaC1 * 1000.0) + QLatin1Char(' ') +
                fixed(correction.deltaC2 * 1000.0) + QLatin1Char('\n');
            appendSsrBlock(sourceId, epochUtc, message, correction, QStringLiteral("CLOCK"), record);
        }
    }
    if (message.kind == RtcmSsrCorrectionKind::CodeBias) {
        QStringList entries;
        for (const RtcmSsrCodeBiasEntry& bias : satellite.codeBiases) {
            const QString signal = ssrSignalToRinexType2ch(message.system, bias.signalCode,
                                                           message.messageType);
            if (!signal.isEmpty()) {
                entries << signal << fixed(bias.biasMeters);
            }
        }
        QString record = prn + QStringLiteral("   ") +
            QString::number(entries.size() / 2).rightJustified(2, QLatin1Char(' '));
        for (int i = 0; i + 1 < entries.size(); i += 2) {
            record += QStringLiteral("   ") + entries.at(i) + QLatin1Char(' ') +
                fixed(entries.at(i + 1).toDouble());
        }
        record += QLatin1Char('\n');
        appendSsrBlock(sourceId, epochUtc, message, satellite, QStringLiteral("CODE_BIAS"), record);
    }
    if (message.kind == RtcmSsrCorrectionKind::PhaseBias) {
        QStringList entries;
        for (const RtcmSsrPhaseBiasEntry& bias : satellite.phaseBiases) {
            const QString signal = ssrSignalToRinexType2ch(message.system, bias.signalCode,
                                                           message.messageType);
            if (!signal.isEmpty()) {
                entries << signal << fixed(bias.biasMeters)
                        << QString::number(bias.signalIntegerIndicator)
                        << QString::number(bias.wideLaneIntegerIndicator)
                        << QString::number(bias.discontinuityCounter);
            }
        }
        QString record = QStringLiteral(" %1   %2\n")
            .arg(message.dispersiveBiasConsistency).arg(message.mwConsistency);
        record += prn + QLatin1Char(' ') +
            fixed(satellite.yawAngleRad * 180.0 / GnssUtils::kPi, 12, 8) + QLatin1Char(' ') +
            fixed(satellite.yawRateRadPerSec * 180.0 / GnssUtils::kPi, 12, 8) + QStringLiteral("   ") +
            QString::number(entries.size() / 5).rightJustified(2, QLatin1Char(' '));
        for (int i = 0; i + 4 < entries.size(); i += 5) {
            record += QStringLiteral("   ") + entries.at(i) + QLatin1Char(' ') +
                fixed(entries.at(i + 1).toDouble()) + QLatin1Char(' ') +
                QString::number(entries.at(i + 2).toInt()).rightJustified(3, QLatin1Char(' ')) + QLatin1Char(' ') +
                QString::number(entries.at(i + 3).toInt()).rightJustified(3, QLatin1Char(' ')) + QLatin1Char(' ') +
                QString::number(entries.at(i + 4).toInt()).rightJustified(3, QLatin1Char(' '));
        }
        record += QLatin1Char('\n');
        appendSsrBlock(sourceId, epochUtc, message, satellite, QStringLiteral("PHASE_BIAS"), record);
    }
}

void TelemetryWriter::writeVtec(const QString& sourceId,
                                const QDateTime& epochUtc,
                                const RtcmVtecMessage& message) {
    if (!isEnabled() || !epochUtc.isValid() || message.layers.isEmpty()) return;
    QVector<const RtcmVtecLayer*> layers;
    for (const RtcmVtecLayer& layer : message.layers) {
        if (layer.cosine.isEmpty() || layer.sine.size() != layer.cosine.size()) continue;
        const int order = layer.cosine.first().size();
        if (order <= 0) continue;
        bool valid = true;
        for (int n = 0; n < layer.cosine.size(); ++n) {
            valid = valid && layer.cosine.at(n).size() == order &&
                    layer.sine.at(n).size() == order;
        }
        if (valid) layers.push_back(&layer);
    }
    if (layers.isEmpty()) return;
    int gpsWeek = 0;
    double gpsSow = 0.0;
    GnssUtils::gpsWeekFromDateTime(epochUtc, gpsWeek, gpsSow);
    const QDateTime gpsEpoch = GnssUtils::gpsCalendarDateTime(gpsWeek, gpsSow);
    const QString pathName = ssrRelativePath(sourceId, sourceName(sourceId),
                                             message.providerId, message.solutionId,
                                             gpsEpoch, QStringLiteral("ION"));
    const QString key = QStringLiteral("ssr|%1").arg(pathName);
    auto existing = _files.find(key);
    FileState* state = nullptr;
    if (existing != _files.end()) {
        state = &existing.value();
    } else {
        QDir directory(_directory);
        if (!directory.mkpath(QFileInfo(pathName).path())) return;
        FileState newState;
        newState.file = new QFile(directory.filePath(pathName), this);
        if (!newState.file->open(QIODevice::WriteOnly | QIODevice::Append)) {
            delete newState.file;
            return;
        }
        state = &_files.insert(key, newState).value();
    }

    const QString epoch = gpsEpoch.toString(QStringLiteral("yyyy MM dd HH mm ss")) + QStringLiteral(".0");
    state->buffer.append(QStringLiteral("> VTEC %1 %2 %3 %4\n")
                             .arg(epoch).arg(message.updateIntervalIndex)
                             .arg(layers.size()).arg(sourceName(sourceId)).toUtf8());
    for (int i = 0; i < layers.size(); ++i) {
        const RtcmVtecLayer& layer = *layers.at(i);
        const int degree = layer.cosine.size() - 1;
        const int order = layer.cosine.first().size() - 1;
        state->buffer.append(QStringLiteral("%1 %2 %3 %4\n")
                                 .arg(i + 1, 2).arg(degree, 2).arg(order, 2)
                                 .arg(QString::number(layer.heightMeters, 'f', 1)).toUtf8());
        QStringList cosine;
        QStringList sine;
        for (int n = 0; n <= degree; ++n) {
            for (int m = 0; m <= order; ++m) {
                cosine << QString::number(layer.cosine.at(n).at(m), 'f', 4);
                sine << QString::number(layer.sine.at(n).at(m), 'f', 4);
            }
        }
        if (cosine.isEmpty() || sine.isEmpty()) continue;
        state->buffer.append(cosine.join(QLatin1Char(' ')).toUtf8());
        state->buffer.append('\n');
        state->buffer.append(sine.join(QLatin1Char(' ')).toUtf8());
        state->buffer.append('\n');
    }
    constexpr int kFlushThresholdBytes = 64 * 1024;
    if (state->buffer.size() >= kFlushThresholdBytes) flushState(*state);
}

void TelemetryWriter::writeReception(const QString& sourceId,
                                     const QDateTime& receivedUtc,
                                     const QString& messageFamily,
                                     int messageType,
                                     const QChar& system,
                                     const QString& status,
                                     const QString& reason,
                                     const QString& detail) {
    if (!isEnabled()) return;
    appendRow("reception", receivedUtc,
              {sourceId, utcText(receivedUtc), messageFamily, QString::number(messageType),
               QString(system), status, reason, detail});
}

void TelemetryWriter::writePppResiduals(const QString& sourceId, const PppSolution& solution) {
    if (!isEnabled() || !solution.valid || !solution.utc.isValid()) return;
    for (const PppObservationResidual& residual : solution.observationResiduals) {
        appendRow("ppp_residual", solution.utc,
                  {sourceId, utcText(solution.utc), residual.satellite, residual.signal,
                   residual.observationType, number(residual.prefitM),
                   number(residual.postfitM), number(residual.sigmaM),
                   number(residual.elevationDeg), boolText(residual.adopted), residual.reason});
    }
}

void TelemetryWriter::appendRow(const QString& family,
                                const QDateTime& utc,
                                const QStringList& fields) {
    if (!isEnabled()) return;
    FileState* state = fileState(family, utc);
    if (!state) return;

    QStringList escaped;
    escaped.reserve(fields.size());
    for (const QString& field : fields) escaped.push_back(csv(field));
    state->buffer.append(escaped.join(',').toUtf8());
    state->buffer.append('\n');
    constexpr int kFlushThresholdBytes = 64 * 1024;
    if (state->buffer.size() >= kFlushThresholdBytes) {
        flushState(*state);
    }
}

void TelemetryWriter::appendSsrBlock(const QString& sourceId,
                                     const QDateTime& epochUtc,
                                     const RtcmSsrMessage& message,
                                     const RtcmSsrSatelliteData& satellite,
                                     const QString& kind,
                                     const QString& record) {
    if (!isEnabled() || !epochUtc.isValid()) return;
    const QString productKey = ssrSourceFileToken(sourceId) + QLatin1Char('|') +
        QString::number(message.providerId) + QLatin1Char('|') + QString::number(message.solutionId);
    flushSsrBlocks(productKey, epochUtc);
    const QString rank = kind == QLatin1String("ORBIT") ? QStringLiteral("0") :
                         kind == QLatin1String("CLOCK") ? QStringLiteral("1") :
                         kind == QLatin1String("CODE_BIAS") ? QStringLiteral("2") : QStringLiteral("3");
    const QString key = productKey + QLatin1Char('|') +
        QString::number(epochUtc.toMSecsSinceEpoch()) + QLatin1Char('|') + rank + kind +
        QLatin1Char('|') + QString::number(message.updateIntervalIndex) + QLatin1Char('|') +
        (kind == QLatin1String("PHASE_BIAS")
             ? QStringLiteral("%1_%2").arg(message.dispersiveBiasConsistency).arg(message.mwConsistency)
             : QString());
    SsrBlock& block = _ssrBlocks[key];
    block.sourceId = sourceId;
    block.kind = kind;
    block.epochUtc = epochUtc;
    block.message = message;
    block.message.satellites.clear();
    QString satRecord = record;
    if (kind == QLatin1String("PHASE_BIAS")) {
        const int newline = satRecord.indexOf(QLatin1Char('\n'));
        if (newline < 0) return;
        block.consistency = satRecord.left(newline);
        satRecord = satRecord.mid(newline + 1);
    }
    const QString prn = satelliteText(satellite.sat);
    if (!block.records.contains(prn)) block.satelliteOrder.push_back(prn);
    block.records[prn] = satRecord;
}

void TelemetryWriter::flushSsrBlocks(const QString& productKey,
                                     const QDateTime& beforeEpoch) {
    for (auto it = _ssrBlocks.begin(); it != _ssrBlocks.end();) {
        if ((!productKey.isEmpty() && !it.key().startsWith(productKey + QLatin1Char('|'))) ||
            (beforeEpoch.isValid() && it->epochUtc >= beforeEpoch)) {
            ++it;
            continue;
        }
        writeSsrBlock(it.value());
        it = _ssrBlocks.erase(it);
    }
}

void TelemetryWriter::writeSsrBlock(const SsrBlock& block) {
    if (block.records.isEmpty()) return;
    const QString& sourceId = block.sourceId;
    const QDateTime& epochUtc = block.epochUtc;
    const RtcmSsrMessage& message = block.message;

    int gpsWeek = 0;
    double gpsSow = 0.0;
    GnssUtils::gpsWeekFromDateTime(epochUtc, gpsWeek, gpsSow);
    const QDateTime gpsEpoch = GnssUtils::gpsCalendarDateTime(gpsWeek, gpsSow);
    const QString pathName = ssrRelativePath(sourceId, sourceName(sourceId),
                                             message.providerId, message.solutionId,
                                             gpsEpoch, QStringLiteral("MC"));
    const QString key = QStringLiteral("ssr|%1").arg(pathName);
    auto existing = _files.find(key);
    FileState* state = nullptr;
    if (existing != _files.end()) {
        state = &existing.value();
    } else {
        QDir directory(_directory);
        if (!directory.mkpath(QFileInfo(pathName).path())) return;
        const QString path = directory.filePath(pathName);
        FileState newState;
        newState.file = new QFile(path, this);
        if (!newState.file->open(QIODevice::WriteOnly | QIODevice::Append)) {
            delete newState.file;
            return;
        }
        state = &_files.insert(key, newState).value();
    }

    const QString epoch = gpsEpoch.toString(QStringLiteral("yyyy MM dd HH mm ss")) + QStringLiteral(".0");
    const QString header = QStringLiteral("> %1 %2 %3 %4 %5\n")
        .arg(block.kind, epoch, QString::number(message.updateIntervalIndex))
        .arg(block.records.size()).arg(sourceName(sourceId));
    state->buffer.append(header.toUtf8());
    if (!block.consistency.isEmpty()) state->buffer.append((block.consistency + QLatin1Char('\n')).toUtf8());
    for (const QString& prn : block.satelliteOrder)
        state->buffer.append(block.records.value(prn).toUtf8());
    constexpr int kFlushThresholdBytes = 64 * 1024;
    if (state->buffer.size() >= kFlushThresholdBytes) flushState(*state);
}

TelemetryWriter::FileState* TelemetryWriter::fileState(const QString& family,
                                                       const QDateTime& utc) {
    const QDateTime dateTime = utc.isValid() ? utc.toUTC() : QDateTime::currentDateTimeUtc();
    const QString date = dateTime.toString("yyyyMMdd");
    QString key = family + QLatin1Char('|') + date;
    QString pathName = QStringLiteral("%1_%2.csv").arg(family, date);
    auto existing = _files.find(key);
    if (existing != _files.end()) {
        return &existing.value();
    }

    QDir directory(_directory);
    if (!directory.exists() && !directory.mkpath(".")) return nullptr;
    const QString path = directory.filePath(pathName);
    const QFileInfo info(path);
    const bool writeHeader = !info.exists() || info.size() == 0;

    FileState state;
    state.file = new QFile(path, this);
    if (!state.file->open(QIODevice::WriteOnly | QIODevice::Append)) {
        delete state.file;
        return nullptr;
    }
    if (writeHeader) {
        state.buffer.append(headerFor(family).join(',').toUtf8());
        state.buffer.append('\n');
    }
    auto inserted = _files.insert(key, state);
    return &inserted.value();
}

void TelemetryWriter::flushState(FileState& state) {
    if (!state.file || !state.file->isOpen() || state.buffer.isEmpty()) return;
    while (!state.buffer.isEmpty()) {
        const qint64 written = state.file->write(state.buffer);
        if (written <= 0) {
            break;
        }
        state.buffer.remove(0, static_cast<int>(written));
    }
    state.file->flush();
}

void TelemetryWriter::flush() {
    for (auto it = _files.begin(); it != _files.end(); ++it) {
        flushState(it.value());
    }
}

void TelemetryWriter::close() {
    if (_flushTimer) {
        _flushTimer->stop();
    }
    flushSsrBlocks();
    flush();
    for (auto it = _files.begin(); it != _files.end(); ++it) {
        if (!it.value().file) continue;
        it.value().file->close();
        delete it.value().file;
        it.value().file = nullptr;
    }
    _files.clear();
    _lastClockCorrections.clear();
    _orbitIods.clear();
    _clockSsrIods.clear();
}

QString TelemetryWriter::csv(const QString& value) {
    QString escaped = value;
    escaped.replace('"', "\"\"");
    return '"' + escaped + '"';
}

QString TelemetryWriter::utcText(const QDateTime& value) {
    return value.isValid() ? value.toUTC().toString(Qt::ISODateWithMs) : QString();
}

QString TelemetryWriter::number(double value) {
    return std::isfinite(value) ? QString::number(value, 'g', 16) : QString();
}

QString TelemetryWriter::boolText(bool value) { return value ? "true" : "false"; }

QString TelemetryWriter::satelliteText(const RtcmSatId& satellite) {
    if (!satellite.isValid()) return QString();
    return QString("%1%2").arg(satellite.system).arg(satellite.number, 2, 10, QLatin1Char('0'));
}

