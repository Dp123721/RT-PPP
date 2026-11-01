// Encodes PPP solutions into the compact JSON schema and manages daily files.
#include "pppjsonwriter.h"

#include "gnss_utils.h"

#include <QCryptographicHash>
#include <QDateTime>
#include <QDir>
#include <QFile>
#include <QFileInfo>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QJsonParseError>
#include <QRegularExpression>
#include <QTime>
#include <QtGlobal>
#include <algorithm>
#include <cmath>
#include <limits>

namespace {

const QByteArray kSolutionsMarker = QByteArrayLiteral(",\"solutions\":[\n");
const QByteArray kFinalSuffix = QByteArrayLiteral("]}\n");
constexpr int kCurrentSchemaVersion = 1;

QJsonValue finiteValue(double value) {
    return std::isfinite(value) ? QJsonValue(value) : QJsonValue(QJsonValue::Null);
}

QJsonArray numericArray(const double* values, int count) {
    QJsonArray result;
    for (int i = 0; i < count; ++i) {
        result.append(finiteValue(values[i]));
    }
    return result;
}

QJsonArray stringArray(const QStringList& values) {
    QJsonArray result;
    for (const QString& value : values) {
        result.append(value);
    }
    return result;
}

int systemIndex(QChar system) {
    switch (system.toUpper().unicode()) {
    case 'G': return 1;
    case 'R': return 2;
    case 'E': return 3;
    case 'C': return 4;
    case 'J': return 5;
    case 'S': return 6;
    case 'I': return 7;
    default: return 0;
    }
}

int groupIndex(const QString& group) {
    const QString normalized = group.trimmed().toUpper();
    if (normalized == QLatin1String("L1")) return 1;
    if (normalized == QLatin1String("L2")) return 2;
    if (normalized == QLatin1String("NL")) return 3;
    if (normalized == QLatin1String("WL")) return 4;
    return 0;
}

QJsonValue satelliteId(const QString& satellite) {
    const QString normalized = satellite.trimmed().toUpper();
    if (normalized.size() < 2) {
        return normalized.isEmpty() ? QJsonValue(QJsonValue::Null) : QJsonValue(normalized);
    }
    bool ok = false;
    const int prn = normalized.mid(1).toInt(&ok);
    if (!ok || prn <= 0 || prn >= 100) {
        return QJsonValue(normalized);
    }
    const int system = systemIndex(normalized[0]);
    if (system <= 0) {
        return QJsonValue(normalized);
    }
    return QJsonValue((system - 1) * 100 + prn);
}

QJsonArray satelliteArray(const QStringList& satellites) {
    QJsonArray result;
    for (const QString& satellite : satellites) {
        result.append(satelliteId(satellite));
    }
    return result;
}

QJsonArray groupArray(const QVector<PppArGroupResult>& groups) {
    QJsonArray result;
    for (const PppArGroupResult& group : groups) {
        QJsonArray item;
        item.append(systemIndex(group.system));
        item.append(groupIndex(group.group));
        item.append(satelliteId(group.referenceSatellite));
        item.append(group.fixedCount);
        item.append(group.eligibleCount);
        item.append(finiteValue(group.maxFractionCycle));
        item.append(finiteValue(group.maxSigmaCycle));
        result.append(item);
    }
    return result;
}

int algorithmIndex(const PppSolution& solution) {
    if (!solution.arEnabled) return 0;
    if (solution.arAlgorithm.compare(QStringLiteral("BIE"), Qt::CaseInsensitive) == 0) return 2;
    return 1;
}

int statusIndex(const PppSolution& solution) {
    if (!solution.arEnabled || solution.arStatus == QLatin1String("disabled")) return 0;
    if (solution.arStatus == QLatin1String("partial_fix")) return 2;
    if (solution.arStatus == QLatin1String("fixed")) return 3;
    return 1;
}

QJsonValue arData(const PppSolution& solution) {
    if (!solution.arEnabled) {
        return QJsonValue(QJsonValue::Null);
    }
    QJsonArray data;
    if (solution.arAlgorithm.compare(QStringLiteral("BIE"), Qt::CaseInsensitive) == 0) {
        data.append(solution.arCandidateCount > 0
                        ? QJsonValue(solution.arCandidateCount)
                        : QJsonValue(QJsonValue::Null));
        data.append(solution.arEffectiveCandidates > 0.0
                        ? finiteValue(solution.arEffectiveCandidates)
                        : QJsonValue(QJsonValue::Null));
    } else {
        data.append(solution.arRatio > 0.0
                        ? finiteValue(solution.arRatio)
                        : QJsonValue(QJsonValue::Null));
        data.append(solution.arBootstrap > 0.0
                        ? finiteValue(solution.arBootstrap)
                        : QJsonValue(QJsonValue::Null));
    }
    data.append(solution.arConstraintsApplied);
    data.append(groupArray(solution.arGroups));
    return data;
}

QJsonArray solutionRecord(qint64 offsetMs, const PppSolution& solution, int schemaVersion) {
    QJsonArray record;
    record.append(static_cast<double>(offsetMs));
    record.append(algorithmIndex(solution));
    record.append(statusIndex(solution));
    record.append(numericArray(solution.xyz, 3));
    if (schemaVersion >= 2) {
        QJsonArray enu;
        enu.append(finiteValue(solution.neu[1]));
        enu.append(finiteValue(solution.neu[0]));
        enu.append(finiteValue(solution.neu[2]));
        record.append(enu);
    } else {
        record.append(numericArray(solution.neu, 3));
    }
    record.append(numericArray(solution.cov, 6));

    QJsonArray troposphere;
    troposphere.append(finiteValue(solution.ztd));
    troposphere.append(finiteValue(solution.trp));
    troposphere.append(finiteValue(solution.ztdStdev));
    record.append(troposphere);

    const int usedCount = solution.usedSatellites.isEmpty()
        ? solution.numSat
        : solution.usedSatellites.size();
    QJsonArray satelliteSummary;
    satelliteSummary.append(usedCount);
    satelliteSummary.append(solution.arEligibleSatellites.size());
    satelliteSummary.append(solution.arFixedSatellites.size());
    satelliteSummary.append(finiteValue(solution.hDop));
    record.append(satelliteSummary);

    QJsonArray satelliteSets;
    satelliteSets.append(satelliteArray(solution.usedSatellites));
    satelliteSets.append(satelliteArray(solution.arEligibleSatellites));
    satelliteSets.append(satelliteArray(solution.arFixedSatellites));
    record.append(satelliteSets);
    record.append(arData(solution));
    record.append(0);
    return record;
}

QDateTime gpsOutputTime(const PppSolution& solution) {
    if (solution.gpsTime.isValid()) return solution.gpsTime;
    int gpsWeek = solution.gpsWeek;
    double gpsSow = solution.gpsSow;
    if (gpsWeek <= 0 || !std::isfinite(gpsSow)) {
        GnssUtils::gpsWeekFromDateTime(solution.utc, gpsWeek, gpsSow);
    }
    return GnssUtils::gpsCalendarDateTime(gpsWeek, gpsSow);
}

QJsonObject metadata(const QString& stationId,
                     const QString& fileStationId,
                     const QDate& date,
                     const PppSolution& firstSolution,
                     const std::array<double, 3>& referenceEcef) {
    QJsonObject root;
    root.insert(QStringLiteral("schema"), QStringLiteral("rtppp.compact"));
    root.insert(QStringLiteral("version"), kCurrentSchemaVersion);

    QJsonObject station;
    station.insert(QStringLiteral("id"), stationId);
    station.insert(QStringLiteral("fileId"), fileStationId);
    station.insert(QStringLiteral("referenceEcef"), numericArray(referenceEcef.data(), 3));
    root.insert(QStringLiteral("station"), station);

    const QDateTime midnight(date, QTime(0, 0), Qt::UTC);
    const QDateTime firstGpsTime = gpsOutputTime(firstSolution);
    const qint64 firstOffsetMs = midnight.msecsTo(firstGpsTime);
    int baseGpsWeek = firstSolution.gpsWeek;
    double baseGpsSow = firstSolution.gpsSow - static_cast<double>(firstOffsetMs) / 1000.0;
    while (baseGpsSow < 0.0) {
        baseGpsSow += 604800.0;
        --baseGpsWeek;
    }
    while (baseGpsSow >= 604800.0) {
        baseGpsSow -= 604800.0;
        ++baseGpsWeek;
    }
    QJsonObject timeBase;
    timeBase.insert(QStringLiteral("system"), QStringLiteral("GPS"));
    timeBase.insert(QStringLiteral("date"), date.toString(QStringLiteral("yyyy-MM-dd")));
    timeBase.insert(QStringLiteral("epoch"), midnight.toString(Qt::ISODateWithMs));
    timeBase.insert(QStringLiteral("unit"), QStringLiteral("ms"));
    timeBase.insert(QStringLiteral("gpsWeek"), baseGpsWeek);
    timeBase.insert(QStringLiteral("gpsSow"), finiteValue(baseGpsSow));
    root.insert(QStringLiteral("timeBase"), timeBase);

    root.insert(QStringLiteral("algorithms"),
                stringArray(QStringList() << "NONE" << "LAMBDA" << "BIE"));
    root.insert(QStringLiteral("arStatus"),
                stringArray(QStringList() << "disabled" << "float" << "partial_fix" << "fixed"));
    root.insert(QStringLiteral("systems"),
                stringArray(QStringList() << "?" << "G" << "R" << "E" << "C" << "J" << "S" << "I"));
    root.insert(QStringLiteral("ambiguityGroups"),
                stringArray(QStringList() << "" << "L1" << "L2" << "NL" << "WL"));

    QJsonObject satelliteEncoding;
    satelliteEncoding.insert(QStringLiteral("formula"), QStringLiteral("systemBase + PRN"));
    QJsonObject systemBases;
    systemBases.insert(QStringLiteral("G"), 0);
    systemBases.insert(QStringLiteral("R"), 100);
    systemBases.insert(QStringLiteral("E"), 200);
    systemBases.insert(QStringLiteral("C"), 300);
    systemBases.insert(QStringLiteral("J"), 400);
    systemBases.insert(QStringLiteral("S"), 500);
    systemBases.insert(QStringLiteral("I"), 600);
    satelliteEncoding.insert(QStringLiteral("systemBase"), systemBases);
    satelliteEncoding.insert(QStringLiteral("unknown"), QStringLiteral("original string"));
    root.insert(QStringLiteral("satelliteIdEncoding"), satelliteEncoding);

    QJsonObject layout;
    layout.insert(QStringLiteral("solution"), stringArray(QStringList()
        << "timeOffsetMs" << "algorithm" << "arStatus" << "ecefM" << "enuM"
        << "ecefCovarianceM2" << "troposphere" << "satelliteSummary"
        << "satelliteSets" << "arData" << "flags"));
    layout.insert(QStringLiteral("ecefCovarianceM2"),
                  stringArray(QStringList() << "qxx" << "qxy" << "qxz" << "qyy" << "qyz" << "qzz"));
    layout.insert(QStringLiteral("troposphere"),
                  stringArray(QStringList() << "ztdM" << "zwdM" << "ztdStdM"));
    layout.insert(QStringLiteral("satelliteSummary"),
                  stringArray(QStringList() << "used" << "arEligible" << "fixed" << "hdop"));
    layout.insert(QStringLiteral("satelliteSets"),
                  stringArray(QStringList() << "used" << "arEligible" << "fixed"));
    layout.insert(QStringLiteral("lambdaArData"),
                  stringArray(QStringList() << "ratio" << "bootstrap" << "constraintsApplied" << "groups"));
    layout.insert(QStringLiteral("bieArData"),
                  stringArray(QStringList() << "candidateCount" << "effectiveCandidates" << "constraintsApplied" << "groups"));
    layout.insert(QStringLiteral("arGroup"), stringArray(QStringList()
        << "system" << "ambiguityGroup" << "referenceSatellite" << "fixed"
        << "eligible" << "maxFractionCycle" << "maxSigmaCycle"));
    root.insert(QStringLiteral("layout"), layout);
    return root;
}

void assignError(QString* target, const QString& message) {
    if (target) {
        *target = message;
    }
}

} // namespace

struct PppJsonWriter::StationState {
    QString stationId;
    QString safeStationId;
    QDate date;
    QString workingPath;
    QFile file;
    QMap<qint64, QByteArray> pending;
    qint64 latestOffsetMs = -1;
    qint64 lastWrittenOffsetMs = -1;
    qint64 lastDiskFlushMs = 0;
    bool hasRecords = false;
    int schemaVersion = kCurrentSchemaVersion;
};

PppJsonWriter::~PppJsonWriter() {
    close();
}

bool PppJsonWriter::setOptions(const Options& options, QString* error) {
    Options normalized = options;
    normalized.outputDirectory = QDir::cleanPath(normalized.outputDirectory.trimmed());
    normalized.reorderBufferMs = qMax(0, normalized.reorderBufferMs);
    normalized.flushIntervalMs = qMax(0, normalized.flushIntervalMs);
    if (normalized.enabled && normalized.outputDirectory.isEmpty()) {
        assignError(error, QStringLiteral("PPP JSON output directory is empty"));
        return false;
    }

    const bool destinationChanged = _options.enabled != normalized.enabled ||
                                    _options.outputDirectory != normalized.outputDirectory;
    if (destinationChanged && !_states.isEmpty()) {
        QString closeError;
        if (!close(&closeError)) {
            assignError(error, closeError);
            return false;
        }
    }
    _options = normalized;
    return true;
}

QString PppJsonWriter::safeStationId(const QString& stationId) {
    const QString normalized = stationId.trimmed().toUpper();
    QString safe = normalized;
    safe.replace(QRegularExpression(QStringLiteral("[^A-Z0-9_\\-]")), QStringLiteral("_"));
    safe.replace(QRegularExpression(QStringLiteral("_+")), QStringLiteral("_"));
    safe = safe.left(48);
    if (safe.isEmpty()) {
        safe = QStringLiteral("UNKNOWN");
    }
    if (safe != normalized) {
        const QByteArray digest = QCryptographicHash::hash(normalized.toUtf8(), QCryptographicHash::Sha1).toHex();
        safe += QStringLiteral("_") + QString::fromLatin1(digest.left(8)).toUpper();
    }
    return safe;
}

bool PppJsonWriter::append(const QString& stationId,
                           const PppSolution& solution,
                           const std::array<double, 3>& referenceEcef,
                           QString* error) {
    if (!_options.enabled) {
        return true;
    }
    if (!solution.valid || !solution.utc.isValid()) {
        assignError(error, QStringLiteral("PPP JSON received an invalid solution or internal UTC epoch"));
        return false;
    }

    QString station = stationId.trimmed();
    if (station.isEmpty()) {
        station = QStringLiteral("UNKNOWN");
    }
    const QString stateKey = station.toUpper();
    const QString fileStation = safeStationId(station);
    const QDateTime gpsTime = gpsOutputTime(solution);
    if (!gpsTime.isValid()) {
        assignError(error, QStringLiteral("PPP JSON received an invalid GPS epoch"));
        return false;
    }
    const QDate date = gpsTime.date();
    StationState* state = _states.value(stateKey, nullptr);
    if (state && state->date != date) {
        QString finalizeError;
        if (!finalizeState(*state, &finalizeError)) {
            assignError(error, finalizeError);
            return false;
        }
        _states.remove(stateKey);
        delete state;
        state = nullptr;
    }
    if (!state && !openState(station, fileStation, date, solution, referenceEcef, state, error)) {
        return false;
    }
    if (!state) {
        assignError(error, QStringLiteral("PPP JSON internal station state is unavailable"));
        return false;
    }
    _states[stateKey] = state;

    const QDateTime midnight(date, QTime(0, 0), Qt::UTC);
    const qint64 offsetMs = midnight.msecsTo(gpsTime);
    if (offsetMs < 0 || offsetMs >= 86400000LL) {
        assignError(error, QStringLiteral("PPP JSON GPS offset is outside the station day"));
        return false;
    }
    if (offsetMs <= state->lastWrittenOffsetMs) {
        return true;
    }

    const QJsonArray record = solutionRecord(offsetMs, solution, state->schemaVersion);
    state->pending[offsetMs] = QJsonDocument(record).toJson(QJsonDocument::Compact);
    state->latestOffsetMs = qMax(state->latestOffsetMs, offsetMs);
    return flushPending(*state, false, error);
}

bool PppJsonWriter::openState(const QString& stationId,
                              const QString& safeId,
                              const QDate& date,
                              const PppSolution& firstSolution,
                              const std::array<double, 3>& referenceEcef,
                              StationState*& state,
                              QString* error) {
    QDir root(_options.outputDirectory);
    if (!root.exists() && !root.mkpath(QStringLiteral("."))) {
        assignError(error, QStringLiteral("Cannot create PPP JSON root directory: %1").arg(root.path()));
        return false;
    }
    if (!root.mkpath(safeId)) {
        assignError(error, QStringLiteral("Cannot create PPP JSON station directory: %1").arg(root.filePath(safeId)));
        return false;
    }
    QDir stationDir(root.filePath(safeId));
    const QString baseName = QStringLiteral("ppp_%1_%2.json")
                                 .arg(safeId, date.toString(QStringLiteral("yyyyMMdd")));

    StationState* opened = new StationState();
    opened->stationId = stationId;
    opened->safeStationId = safeId;
    opened->date = date;
    opened->workingPath = stationDir.filePath(baseName);
    const QString legacyPartPath = opened->workingPath + QStringLiteral(".part");

    const bool hasWorkingFile = QFile::exists(opened->workingPath);
    const bool hasLegacyPartFile = QFile::exists(legacyPartPath);
    if (hasWorkingFile && hasLegacyPartFile) {
        assignError(error, QStringLiteral("PPP JSON recovery conflict: both active and legacy files exist: %1 ; %2")
                               .arg(opened->workingPath, legacyPartPath));
        delete opened;
        return false;
    }
    if (hasLegacyPartFile) {
        if (!QFile::rename(legacyPartPath, opened->workingPath)) {
            assignError(error, QStringLiteral("Cannot migrate legacy PPP JSON part file to %1")
                                   .arg(opened->workingPath));
            delete opened;
            return false;
        }
    }

    opened->file.setFileName(opened->workingPath);
    if (QFile::exists(opened->workingPath)) {
        if (!recoverOpenFile(*opened, error)) {
            delete opened;
            return false;
        }
    } else {
        if (!opened->file.open(QIODevice::ReadWrite | QIODevice::Truncate)) {
            assignError(error, QStringLiteral("Cannot create PPP JSON file %1: %2")
                                   .arg(opened->workingPath, opened->file.errorString()));
            delete opened;
            return false;
        }
        QByteArray prefix = QByteArrayLiteral("{\"meta\":");
        prefix += QJsonDocument(metadata(stationId, safeId, date, firstSolution, referenceEcef))
                      .toJson(QJsonDocument::Compact);
        prefix += kSolutionsMarker;
        if (opened->file.write(prefix) != prefix.size() || !opened->file.flush()) {
            assignError(error, QStringLiteral("Cannot write PPP JSON header %1: %2")
                                   .arg(opened->workingPath, opened->file.errorString()));
            opened->file.close();
            delete opened;
            return false;
        }
    }
    opened->lastDiskFlushMs = QDateTime::currentMSecsSinceEpoch();
    state = opened;
    return true;
}

bool PppJsonWriter::recoverOpenFile(StationState& state, QString* error) {
    if (!state.file.open(QIODevice::ReadWrite)) {
        assignError(error, QStringLiteral("Cannot open PPP JSON file %1: %2")
                               .arg(state.workingPath, state.file.errorString()));
        return false;
    }
    const QByteArray head = state.file.read(256 * 1024);
    const int markerIndex = head.indexOf(kSolutionsMarker);
    if (markerIndex < 0) {
        assignError(error, QStringLiteral("PPP JSON file has an unknown schema: %1").arg(state.workingPath));
        state.file.close();
        return false;
    }
    const QByteArray metaPrefix = QByteArrayLiteral("{\"meta\":");
    if (!head.startsWith(metaPrefix)) {
        assignError(error, QStringLiteral("PPP JSON file has an invalid metadata prefix: %1").arg(state.workingPath));
        state.file.close();
        return false;
    }
    QJsonParseError metaParseError;
    const QJsonDocument metaDocument = QJsonDocument::fromJson(
        head.mid(metaPrefix.size(), markerIndex - metaPrefix.size()), &metaParseError);
    if (metaParseError.error != QJsonParseError::NoError || !metaDocument.isObject()) {
        assignError(error, QStringLiteral("PPP JSON file has invalid metadata: %1").arg(state.workingPath));
        state.file.close();
        return false;
    }
    state.schemaVersion = metaDocument.object().value(QStringLiteral("version")).toInt(1);
    const qint64 recordsStart = markerIndex + kSolutionsMarker.size();

    qint64 size = state.file.size();
    const qint64 tailStart = qMax(recordsStart, size - 2LL * 1024LL * 1024LL);
    state.file.seek(tailStart);
    QByteArray tail = state.file.readAll();
    const QByteArray trimmedTail = tail.trimmed();
    if (trimmedTail.endsWith(QByteArrayLiteral("]}"))) {
        const int footerInTail = tail.lastIndexOf(QByteArrayLiteral("]}"));
        if (footerInTail >= 0) {
            size = tailStart + footerInTail;
            if (!state.file.resize(size)) {
                assignError(error, QStringLiteral("Cannot reopen finalized PPP JSON file: %1").arg(state.workingPath));
                state.file.close();
                return false;
            }
        }
    }

    size = state.file.size();
    const qint64 recoveryStart = qMax(recordsStart, size - 2LL * 1024LL * 1024LL);
    state.file.seek(recoveryStart);
    tail = state.file.readAll();
    const int lastNewline = tail.lastIndexOf('\n');
    const qint64 completeSize = lastNewline >= 0 ? recoveryStart + lastNewline + 1 : recordsStart;
    if (completeSize < size && !state.file.resize(completeSize)) {
        assignError(error, QStringLiteral("Cannot truncate incomplete PPP JSON record: %1").arg(state.workingPath));
        state.file.close();
        return false;
    }

    state.file.seek(recoveryStart);
    tail = state.file.readAll();
    const QList<QByteArray> lines = tail.split('\n');
    for (int i = lines.size() - 1; i >= 0; --i) {
        QByteArray line = lines[i].trimmed();
        if (line.isEmpty() || line.contains(QByteArrayLiteral("\"solutions\":["))) {
            continue;
        }
        if (line.startsWith(',')) {
            line.remove(0, 1);
        }
        QJsonParseError parseError;
        const QJsonDocument document = QJsonDocument::fromJson(line, &parseError);
        if (parseError.error == QJsonParseError::NoError && document.isArray() && !document.array().isEmpty()) {
            state.lastWrittenOffsetMs = static_cast<qint64>(document.array().first().toDouble(-1.0));
            state.latestOffsetMs = state.lastWrittenOffsetMs;
            state.hasRecords = true;
            break;
        }
    }
    state.file.seek(state.file.size());
    return true;
}

bool PppJsonWriter::flushPending(StationState& state, bool force, QString* error) {
    const qint64 threshold = force
        ? std::numeric_limits<qint64>::max()
        : state.latestOffsetMs - _options.reorderBufferMs;
    auto it = state.pending.begin();
    while (it != state.pending.end() && it.key() <= threshold) {
        QByteArray line;
        if (state.hasRecords) {
            line += ',';
        }
        line += it.value();
        line += '\n';
        if (state.file.write(line) != line.size()) {
            assignError(error, QStringLiteral("Cannot append PPP JSON record %1: %2")
                                   .arg(state.workingPath, state.file.errorString()));
            return false;
        }
        state.hasRecords = true;
        state.lastWrittenOffsetMs = it.key();
        it = state.pending.erase(it);
    }

    const qint64 nowMs = QDateTime::currentMSecsSinceEpoch();
    if (force || _options.flushIntervalMs == 0 ||
        nowMs - state.lastDiskFlushMs >= _options.flushIntervalMs) {
        if (!state.file.flush()) {
            assignError(error, QStringLiteral("Cannot flush PPP JSON file %1: %2")
                                   .arg(state.workingPath, state.file.errorString()));
            return false;
        }
        state.lastDiskFlushMs = nowMs;
    }
    return true;
}

bool PppJsonWriter::finalizeState(StationState& state, QString* error) {
    if (!flushPending(state, true, error)) {
        return false;
    }
    if (state.file.write(kFinalSuffix) != kFinalSuffix.size() || !state.file.flush()) {
        assignError(error, QStringLiteral("Cannot finalize PPP JSON file %1: %2")
                               .arg(state.workingPath, state.file.errorString()));
        return false;
    }
    state.file.close();
    return true;
}

bool PppJsonWriter::close(QString* error) {
    QStringList errors;
    for (auto it = _states.begin(); it != _states.end(); ++it) {
        StationState* state = it.value();
        QString stateError;
        if (state && !finalizeState(*state, &stateError)) {
            errors.push_back(stateError);
        }
        delete state;
    }
    _states.clear();
    if (!errors.isEmpty()) {
        assignError(error, errors.join(QStringLiteral("; ")));
        return false;
    }
    return true;
}
