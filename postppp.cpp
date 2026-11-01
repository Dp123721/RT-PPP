// PostPppService coordinates offline observation, navigation, and SSR processing.
#include "postppp.h"

#include "gnss_signals.h"
#include "gnss_utils.h"
#include "gfs_zwd.h"
#include "pppjsonwriter.h"

#include <QDir>
#include <QFile>
#include <QFileInfo>
#include <QCryptographicHash>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QRegularExpression>
#include <QSet>
#include <QTextStream>

#include <algorithm>
#include <cmath>
#include <limits>

namespace {

double number(QString value, bool* ok = nullptr) {
    value = value.trimmed();
    value.replace(QLatin1Char('D'), QLatin1Char('E'));
    value.replace(QLatin1Char('d'), QLatin1Char('e'));
    return value.toDouble(ok);
}

QStringList fields(const QString& line) {
    return line.simplified().split(QLatin1Char(' '), QString::SkipEmptyParts);
}

void setError(QString* error, const QString& value) {
    if (error) *error = value;
}

bool isPlausibleEcefPosition(const double xyz[3]) {
    constexpr double kMinEarthRadiusMeters = 5.0e6;
    constexpr double kMaxEarthRadiusMeters = 7.0e6;
    if (!std::isfinite(xyz[0]) || !std::isfinite(xyz[1]) || !std::isfinite(xyz[2])) return false;
    const double radius = std::sqrt(xyz[0] * xyz[0] + xyz[1] * xyz[1] + xyz[2] * xyz[2]);
    return radius >= kMinEarthRadiusMeters && radius <= kMaxEarthRadiusMeters;
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

QString rejectionCategory(const QString& reason) {
    if (reason.startsWith(QStringLiteral("usable satellites=")) ||
        reason.startsWith(QStringLiteral("usable satellites collapsed"))) {
        return QStringLiteral("insufficient usable satellites");
    }
    if (reason.startsWith(QStringLiteral("filter rejected epoch"))) {
        return QStringLiteral("filter rejected epoch");
    }
    if (reason.startsWith(QStringLiteral("standardized epoch contains no observations"))) {
        return QStringLiteral("empty standardized observations");
    }
    if (reason.isEmpty()) return QStringLiteral("unspecified engine rejection");
    return reason.left(240);
}

bool parseDateGps(const QStringList& f, int offset, int& week, double& sow) {
    if (f.size() < offset + 6) return false;
    bool ok = true;
    const int year = f.at(offset).toInt(&ok);
    const int month = f.at(offset + 1).toInt(&ok);
    const int day = f.at(offset + 2).toInt(&ok);
    const int hour = f.at(offset + 3).toInt(&ok);
    const int minute = f.at(offset + 4).toInt(&ok);
    const double second = number(f.at(offset + 5), &ok);
    if (!ok || !QDate(year, month, day).isValid()) return false;
    const QDateTime t(QDate(year, month, day), QTime(0, 0), Qt::UTC);
    const double unixSec = static_cast<double>(t.toSecsSinceEpoch()) +
                           hour * 3600.0 + minute * 60.0 + second;
    const QDateTime gpsEpoch(QDate(1980, 1, 6), QTime(0, 0), Qt::UTC);
    const double gpsSec = unixSec - static_cast<double>(gpsEpoch.toSecsSinceEpoch());
    week = static_cast<int>(std::floor(gpsSec / 604800.0));
    sow = gpsSec - week * 604800.0;
    return true;
}

QChar systemForSat(const QString& prn) {
    return prn.trimmed().isEmpty() ? QChar() : prn.trimmed().at(0).toUpper();
}

int navTypeForSystem(QChar system) {
    switch (system.toLatin1()) {
    case 'G': return RtcmNavLnav;
    case 'R': return RtcmNavFdmaM;
    case 'E': return RtcmNavInav;
    case 'C': return RtcmNavD1;
    default: return RtcmNavUnknown;
    }
}

bool supportsBroadcastNavigation(QChar system) {
    return system == QLatin1Char('G') || system == QLatin1Char('R') ||
           system == QLatin1Char('E') || system == QLatin1Char('C');
}

bool parseFixedNavigationFields(const QString& line, int firstColumn, int count,
                                QVector<double>& values) {
    values.clear();
    values.reserve(count);
    for (int i = 0; i < count; ++i) {
        const QString field = line.mid(firstColumn + i * 19, 19).trimmed();
        // RINEX navigation records commonly leave unused fields on their
        // final line blank. The reader only consumes fields required by the particular
        // system/message, so preserve blank optional fields as zero here.
        if (field.isEmpty()) {
            values.push_back(0.0);
            continue;
        }
        bool ok = false;
        const double value = number(field, &ok);
        if (!ok || !std::isfinite(value)) return false;
        values.push_back(value);
    }
    return true;
}

bool parseNavigationEpoch(const QString& line, int& week, double& sow,
                          QVector<double>& firstRow) {
    const QStringList dateFields = fields(line.mid(3, 20));
    if (!parseDateGps(dateFields, 0, week, sow)) return false;
    QVector<double> clocks;
    if (!parseFixedNavigationFields(line, 23, 3, clocks)) return false;
    firstRow.clear();
    firstRow.reserve(9);
    for (const QString& field : dateFields) {
        bool ok = false;
        const double value = number(field, &ok);
        if (!ok || !std::isfinite(value)) return false;
        firstRow.push_back(value);
    }
    firstRow += clocks;
    return firstRow.size() == 9;
}

void addGpsLeapSeconds(int year, int month, int day, int& week, double& sow) {
    sow += GnssUtils::gnumleap(year, month, day);
    while (sow >= 604800.0) {
        sow -= 604800.0;
        ++week;
    }
}

void parseObsField(const QString& raw, const QString& type,
                  RinexObservation& obs) {
    if (raw.size() < 1) return;
    const QString valueText = raw.left(14).trimmed();
    bool ok = false;
    const double value = number(valueText, &ok);
    if (ok && std::isfinite(value)) obs.values[type] = value;
    if (raw.size() > 14 && !raw.mid(14, 1).trimmed().isEmpty()) {
        obs.lli[type] = raw.mid(14, 1).trimmed().toInt();
    }
    if (raw.size() > 15 && !raw.mid(15, 1).trimmed().isEmpty()) {
        obs.ssi[type] = raw.mid(15, 1).trimmed().toInt();
    }
}

bool rinexObservationGpsTime(const QStringList& fields,
                              const QString& timeSystem,
                              int& week,
                              double& sow) {
    if (fields.size() < 6) return false;
    bool ok = true;
    const int year = fields.at(0).toInt(&ok);
    const int month = fields.at(1).toInt(&ok);
    const int day = fields.at(2).toInt(&ok);
    const int hour = fields.at(3).toInt(&ok);
    const int minute = fields.at(4).toInt(&ok);
    const double second = number(fields.at(5), &ok);
    if (!ok || !QDate(year, month, day).isValid() || hour < 0 || hour > 23 ||
        minute < 0 || minute > 59 || second < 0.0 || second >= 61.0) return false;
    const QDateTime midnight(QDate(year, month, day), QTime(0, 0), Qt::UTC);
    double unixSeconds = static_cast<double>(midnight.toSecsSinceEpoch()) +
        hour * 3600.0 + minute * 60.0 + second;
    const QString scale = timeSystem.trimmed().toUpper();
    // RINEX OBS epochs are expressed in their declared time system. Convert
    // the supported non-GPS scales to the GPS week/SOW used by the core.
    double gpsSeconds = unixSeconds;
    if (scale == QLatin1String("GLO")) {
        const double utcSeconds = unixSeconds - 3.0 * 3600.0;
        const QDateTime utc = QDateTime::fromMSecsSinceEpoch(
            static_cast<qint64>(std::floor(utcSeconds * 1000.0)), Qt::UTC);
        gpsSeconds = utcSeconds + GnssUtils::gnumleap(utc.date().year(), utc.date().month(), utc.date().day());
    } else if (scale == QLatin1String("BDT")) {
        // BDT is 14 seconds behind GPST.  Unlike UTC, no leap-second term is
        // added when converting a BDT calendar label to GPS week/SOW.
        gpsSeconds = unixSeconds + 14.0;
    } else if (scale == QLatin1String("UTC")) {
        const QDateTime utc = QDateTime::fromMSecsSinceEpoch(
            static_cast<qint64>(std::floor(unixSeconds * 1000.0)), Qt::UTC);
        gpsSeconds = unixSeconds + GnssUtils::gnumleap(utc.date().year(), utc.date().month(), utc.date().day());
    }
    const QDateTime gpsEpoch(QDate(1980, 1, 6), QTime(0, 0), Qt::UTC);
    const double elapsed = gpsSeconds - static_cast<double>(gpsEpoch.toSecsSinceEpoch());
    week = static_cast<int>(std::floor(elapsed / 604800.0));
    sow = elapsed - week * 604800.0;
    return std::isfinite(sow);
}

bool readLineTokens(QTextStream& in, int count, QVector<double>& values) {
    while (values.size() < count && !in.atEnd()) {
        const QStringList f = fields(in.readLine());
        for (const QString& value : f) {
            bool ok = false;
            const double parsed = number(value, &ok);
            if (ok) values.push_back(parsed);
        }
    }
    return values.size() >= count;
}

QString satKey(const RtcmSatId& sat) {
    return sat.key();
}

void sortStates(QMap<QString, QVector<PreciseSatelliteState>>& states) {
    for (auto it = states.begin(); it != states.end(); ++it) {
        std::sort(it.value().begin(), it.value().end(), [](const auto& a, const auto& b) {
            return a.gpsWeek * 604800.0 + a.gpsSow < b.gpsWeek * 604800.0 + b.gpsSow;
        });
    }
}

bool sameSsrSatellite(const RtcmSsrSatelliteData& left, const RtcmSsrSatelliteData& right) {
    if (left.sat.key() != right.sat.key() || left.iod != right.iod ||
        left.hasOrbit != right.hasOrbit || left.hasClock != right.hasClock ||
        left.hasHighRateClock != right.hasHighRateClock ||
        left.deltaRadial != right.deltaRadial || left.deltaAlongTrack != right.deltaAlongTrack ||
        left.deltaCrossTrack != right.deltaCrossTrack || left.dotDeltaRadial != right.dotDeltaRadial ||
        left.dotDeltaAlongTrack != right.dotDeltaAlongTrack || left.dotDeltaCrossTrack != right.dotDeltaCrossTrack ||
        left.deltaC0 != right.deltaC0 || left.deltaC1 != right.deltaC1 || left.deltaC2 != right.deltaC2 ||
        left.highRateClock != right.highRateClock || left.yawAngleRad != right.yawAngleRad ||
        left.yawRateRadPerSec != right.yawRateRadPerSec ||
        left.codeBiases.size() != right.codeBiases.size() || left.phaseBiases.size() != right.phaseBiases.size()) {
        return false;
    }
    for (int i = 0; i < left.codeBiases.size(); ++i) {
        const RtcmSsrCodeBiasEntry& a = left.codeBiases.at(i);
        const RtcmSsrCodeBiasEntry& b = right.codeBiases.at(i);
        if (a.signalCode != b.signalCode || a.biasMeters != b.biasMeters) return false;
    }
    for (int i = 0; i < left.phaseBiases.size(); ++i) {
        const RtcmSsrPhaseBiasEntry& a = left.phaseBiases.at(i);
        const RtcmSsrPhaseBiasEntry& b = right.phaseBiases.at(i);
        if (a.signalCode != b.signalCode || a.biasMeters != b.biasMeters ||
            a.signalIntegerIndicator != b.signalIntegerIndicator ||
            a.wideLaneIntegerIndicator != b.wideLaneIntegerIndicator ||
            a.discontinuityCounter != b.discontinuityCounter) return false;
    }
    return true;
}

bool sameSsrHeader(const RtcmSsrMessage& left, const RtcmSsrMessage& right) {
    return left.kind == right.kind && left.system == right.system &&
           left.gpsWeek == right.gpsWeek && left.epochTime == right.epochTime &&
           left.epochTimeIsGps == right.epochTimeIsGps &&
           left.updateIntervalIndex == right.updateIntervalIndex &&
           left.ssrIod == right.ssrIod && left.providerId == right.providerId &&
           left.solutionId == right.solutionId && left.satRefDatum == right.satRefDatum &&
           left.dispersiveBiasConsistency == right.dispersiveBiasConsistency &&
           left.mwConsistency == right.mwConsistency;
}

bool sameVtecModel(const PppVtecModel& left, const PppVtecModel& right) {
    if (left.gpsWeek != right.gpsWeek ||
        std::fabs(left.gpsSow - right.gpsSow) > 1.0e-9 ||
        left.updateIntervalIndex != right.updateIntervalIndex ||
        left.stationId != right.stationId ||
        left.layers.size() != right.layers.size()) {
        return false;
    }
    for (int layerIndex = 0; layerIndex < left.layers.size(); ++layerIndex) {
        const PppVtecLayer& a = left.layers.at(layerIndex);
        const PppVtecLayer& b = right.layers.at(layerIndex);
        if (a.heightMeters != b.heightMeters || a.cosine != b.cosine || a.sine != b.sine) {
            return false;
        }
    }
    return true;
}

bool selectedBands(const PppOptions& options, QChar system, QChar& first, QChar& second) {
    const QStringList tokens = options.signalPriorities.split(QLatin1Char(' '), QString::SkipEmptyParts);
    for (const QString& token : tokens) {
        const int colon = token.indexOf(QLatin1Char(':'));
        const int amp = token.indexOf(QLatin1Char('&'), colon + 1);
        if (colon != 1 || amp <= colon + 1 || token.at(0) != system) continue;
        const QString bands = token.mid(colon + 1, amp - colon - 1);
        for (const QChar& band : bands) {
            if (first.isNull()) first = band;
            else if (band != first && second.isNull()) second = band;
        }
    }
    return !first.isNull();
}

void addSatelliteDiagnostics(QMap<QString, PppSatelliteStageDiagnostic>& totals,
                             const QMap<QString, PppSatelliteStageDiagnostic>& epoch) {
    for (auto it = epoch.constBegin(); it != epoch.constEnd(); ++it) {
        if (it.key().isEmpty() ||
            (it.key().at(0) != QLatin1Char('E') && it.key().at(0) != QLatin1Char('C'))) {
            continue;
        }
        PppSatelliteStageDiagnostic& total = totals[it.key()];
        total.observationAvailable += it.value().observationAvailable;
        total.broadcastMatched += it.value().broadcastMatched;
        total.ssrMatched += it.value().ssrMatched;
        total.iodRejected += it.value().iodRejected;
        total.residualRejected += it.value().residualRejected;
        total.acceptedAfterResidual += it.value().acceptedAfterResidual;
    }
}

QJsonObject satelliteDiagnosticsJson(const QMap<QString, PppSatelliteStageDiagnostic>& totals) {
    QJsonObject result;
    for (auto it = totals.constBegin(); it != totals.constEnd(); ++it) {
        QJsonObject stage;
        stage[QStringLiteral("observationAvailable")] = it.value().observationAvailable;
        stage[QStringLiteral("broadcastMatched")] = it.value().broadcastMatched;
        stage[QStringLiteral("ssrMatched")] = it.value().ssrMatched;
        stage[QStringLiteral("iodRejected")] = it.value().iodRejected;
        stage[QStringLiteral("residualRejected")] = it.value().residualRejected;
        stage[QStringLiteral("acceptedAfterResidual")] = it.value().acceptedAfterResidual;
        result[it.key()] = stage;
    }
    return result;
}

QJsonObject pppOptionsJson(const PppOptions& options) {
    QJsonObject result;
    result[QStringLiteral("sigmaC1")] = options.sigmaC1;
    result[QStringLiteral("sigmaL1")] = options.sigmaL1;
    result[QStringLiteral("maxResC1")] = options.maxResC1;
    result[QStringLiteral("maxResL1")] = options.maxResL1;
    result[QStringLiteral("minEleRad")] = options.minEleRad;
    result[QStringLiteral("minObs")] = options.minObs;
    result[QStringLiteral("eleWgtCode")] = options.eleWgtCode;
    result[QStringLiteral("eleWgtPhase")] = options.eleWgtPhase;
    result[QStringLiteral("aprSigCrd")] = QJsonArray{options.aprSigCrd[0], options.aprSigCrd[1], options.aprSigCrd[2]};
    result[QStringLiteral("noiseCrd")] = QJsonArray{options.noiseCrd[0], options.noiseCrd[1], options.noiseCrd[2]};
    result[QStringLiteral("aprSigClk")] = options.aprSigClk;
    result[QStringLiteral("aprSigTrp")] = options.aprSigTrp;
    result[QStringLiteral("noiseTrp")] = options.noiseTrp;
    result[QStringLiteral("aprSigAmb")] = options.aprSigAmb;
    result[QStringLiteral("aprSigIon")] = options.aprSigIon;
    result[QStringLiteral("noiseIon")] = options.noiseIon;
    result[QStringLiteral("aprSigCodeBias")] = options.aprSigCodeBias;
    result[QStringLiteral("noiseCodeBias")] = options.noiseCodeBias;
    result[QStringLiteral("aprSigPhaseBias")] = options.aprSigPhaseBias;
    result[QStringLiteral("noisePhaseBias")] = options.noisePhaseBias;
    result[QStringLiteral("seedingTime")] = options.seedingTime;
    result[QStringLiteral("aprXyz")] = QJsonArray{options.aprXyz[0], options.aprXyz[1], options.aprXyz[2]};
    result[QStringLiteral("neuEcc")] = QJsonArray{options.neuEcc[0], options.neuEcc[1], options.neuEcc[2]};
    result[QStringLiteral("crdFile")] = options.crdFile;
    result[QStringLiteral("useAntex")] = options.useAntex;
    result[QStringLiteral("antexFile")] = options.antexFile;
    result[QStringLiteral("roverAntenna")] = options.roverAntenna;
    result[QStringLiteral("isApc")] = options.isApc;
    result[QStringLiteral("useOceanTide")] = options.useOceanTide;
    result[QStringLiteral("blqFile")] = options.blqFile;
    result[QStringLiteral("oceanStation")] = options.oceanStation;
    result[QStringLiteral("vtecFile")] = options.vtecFile;
    result[QStringLiteral("logSsrYawDetails")] = options.logSsrYawDetails;
    result[QStringLiteral("logBiasMatchDetails")] = options.logBiasMatchDetails;
    result[QStringLiteral("useGps")] = options.useGps;
    result[QStringLiteral("useGlo")] = options.useGlo;
    result[QStringLiteral("useGal")] = options.useGal;
    result[QStringLiteral("useBds")] = options.useBds;
    result[QStringLiteral("strictCorrectionSource")] = options.strictCorrectionSource;
    result[QStringLiteral("signalPriorities")] = options.signalPriorities;
    result[QStringLiteral("lcGps")] = options.lcGps;
    result[QStringLiteral("lcGlo")] = options.lcGlo;
    result[QStringLiteral("lcGal")] = options.lcGal;
    result[QStringLiteral("lcBds")] = options.lcBds;
    result[QStringLiteral("pseudoObsIono")] = options.pseudoObsIono;
    result[QStringLiteral("estimateTroposphere")] = options.estimateTroposphere;
    result[QStringLiteral("troposphereMode")] = options.troposphereMode == PppTroposphereMode::GfsZwdInitialization
        ? QStringLiteral("gfs_zwd_initialization") : QStringLiteral("original");
    result[QStringLiteral("gfsZwdMethod")] = options.gfsZwdMethod == GfsZwdMethod::ProfileIntegral
        ? QStringLiteral("profile_integral") : QStringLiteral("askne_nordius");
    result[QStringLiteral("gfsCacheDirectory")] = options.gfsCacheDirectory;
    result[QStringLiteral("gfsDownloadBaseUrl")] = options.gfsDownloadBaseUrl;
    result[QStringLiteral("gfsPrefetchTimeoutSec")] = options.gfsPrefetchTimeoutSec;
    result[QStringLiteral("gfsMaxAgeSec")] = options.gfsMaxAgeSec;
    result[QStringLiteral("estimateIonosphere")] = options.estimateIonosphere;
    result[QStringLiteral("pppRtk")] = options.pppRtk;
    result[QStringLiteral("sigmaGIM")] = options.sigmaGIM;
    result[QStringLiteral("maxResGIM")] = options.maxResGIM;
    result[QStringLiteral("enablePppAr")] = options.enablePppAr;
    result[QStringLiteral("arAlgorithm")] = options.arAlgorithm;
    result[QStringLiteral("arUseGps")] = options.arUseGps;
    result[QStringLiteral("arUseGlonass")] = options.arUseGlonass;
    result[QStringLiteral("arUseGalileo")] = options.arUseGalileo;
    result[QStringLiteral("arUseBds")] = options.arUseBds;
    result[QStringLiteral("arMinEleDeg")] = options.arMinEleDeg;
    result[QStringLiteral("arMinMwSamples")] = options.arMinMwSamples;
    result[QStringLiteral("arMinFixPairs")] = options.arMinFixPairs;
    result[QStringLiteral("arWlMaxDev")] = options.arWlMaxDev;
    result[QStringLiteral("arWlMaxSig")] = options.arWlMaxSig;
    result[QStringLiteral("arNlRatio")] = options.arNlRatio;
    result[QStringLiteral("arNlBoot")] = options.arNlBoot;
    result[QStringLiteral("arNlMaxDev")] = options.arNlMaxDev;
    result[QStringLiteral("arConstraintSigma")] = options.arConstraintSigma;
    result[QStringLiteral("arBieMinEpochs")] = options.arBieMinEpochs;
    result[QStringLiteral("arBieMinSat")] = options.arBieMinSat;
    result[QStringLiteral("arBieMaxFrac")] = options.arBieMaxFrac;
    result[QStringLiteral("arBieMaxSig")] = options.arBieMaxSig;
    result[QStringLiteral("arBieConstraintSigma")] = options.arBieConstraintSigma;
    result[QStringLiteral("arBiePersistConstraints")] = options.arBiePersistConstraints;
    result[QStringLiteral("arBiePersistConfirmEpochs")] = options.arBiePersistConfirmEpochs;
    result[QStringLiteral("arBiePersistMaxVerticalM")] = options.arBiePersistMaxVerticalM;
    result[QStringLiteral("arBiePersistMaxZtdM")] = options.arBiePersistMaxZtdM;
    result[QStringLiteral("arUseHistoryAid")] = options.arUseHistoryAid;
    result[QStringLiteral("arHistMinFixEpochs")] = options.arHistMinFixEpochs;
    result[QStringLiteral("arHistMinLockEpochs")] = options.arHistMinLockEpochs;
    result[QStringLiteral("arPartialWarmupLockEpochs")] = options.arPartialWarmupLockEpochs;
    result[QStringLiteral("arHistRelaxFactor")] = options.arHistRelaxFactor;
    result[QStringLiteral("arLogFile")] = options.arLogFile;
    QJsonObject stationOptions;
    for (auto it = options.crdStationsBySource.constBegin(); it != options.crdStationsBySource.constEnd(); ++it) {
        const PppCrdStationOptions& station = it.value();
        QJsonObject value;
        value[QStringLiteral("valid")] = station.valid;
        value[QStringLiteral("station")] = station.station;
        value[QStringLiteral("aprXyz")] = QJsonArray{station.aprXyz[0], station.aprXyz[1], station.aprXyz[2]};
        value[QStringLiteral("neuEcc")] = QJsonArray{station.neuEcc[0], station.neuEcc[1], station.neuEcc[2]};
        value[QStringLiteral("roverAntenna")] = station.roverAntenna;
        value[QStringLiteral("receiver")] = station.receiver;
        stationOptions[it.key()] = value;
    }
    result[QStringLiteral("crdStationsBySource")] = stationOptions;
    return result;
}

int signalScore(const PppOptions& options, QChar system, const QString& type) {
    if (type.size() != 3) return std::numeric_limits<int>::max();
    const QChar band = type.at(1);
    const QChar attribute = type.at(2);
    int best = std::numeric_limits<int>::max();
    const QStringList tokens = options.signalPriorities.split(QLatin1Char(' '), QString::SkipEmptyParts);
    for (const QString& token : tokens) {
        const int colon = token.indexOf(QLatin1Char(':'));
        const int amp = token.indexOf(QLatin1Char('&'), colon + 1);
        if (colon != 1 || amp < 0 || token.at(0) != system) continue;
        const int bandPos = token.mid(colon + 1, amp - colon - 1).indexOf(band);
        const int attrPos = token.mid(amp + 1).indexOf(attribute);
        if (bandPos >= 0 && attrPos >= 0) best = qMin(best, bandPos * 100 + attrPos);
    }
    return best;
}

void selectRinexBand(const RinexObservation& input, const PppOptions& options,
                     QChar band, bool firstBand, PppStandardizedObservation& output) {
    if (band.isNull()) return;
    QString bestCode;
    QString bestPhase;
    int codeScore = std::numeric_limits<int>::max();
    int phaseScore = std::numeric_limits<int>::max();
    for (auto it = input.values.constBegin(); it != input.values.constEnd(); ++it) {
        const QString& type = it.key();
        if (type.size() != 3 || type.at(1) != band ||
            (type.at(0) != QLatin1Char('C') && type.at(0) != QLatin1Char('L'))) continue;
        const int score = signalScore(options, input.sat.system, type);
        if (score == std::numeric_limits<int>::max()) continue;
        if (type.at(0) == QLatin1Char('C') && score < codeScore) { bestCode = type; codeScore = score; }
        if (type.at(0) == QLatin1Char('L') && score < phaseScore) { bestPhase = type; phaseScore = score; }
    }
    const QString signal = !bestCode.isEmpty() ? bestCode.mid(1) : bestPhase.mid(1);
    if (firstBand) {
        output.b1 = band; output.signal1 = signal; output.score1 = qMin(codeScore, phaseScore);
        if (!bestCode.isEmpty()) { output.c1 = input.values.value(bestCode); output.hasC1 = true; }
        if (!bestPhase.isEmpty()) {
            output.l1 = input.values.value(bestPhase); output.hasL1 = true;
            output.lock1 = input.lli.value(bestPhase, -1); output.sii1 = input.ssi.value(bestPhase, 0);
            output.slip = output.slip || input.lli.value(bestPhase, 0) != 0;
        }
    } else {
        output.b2 = band; output.signal2 = signal; output.score2 = qMin(codeScore, phaseScore);
        if (!bestCode.isEmpty()) { output.c2 = input.values.value(bestCode); output.hasC2 = true; }
        if (!bestPhase.isEmpty()) {
            output.l2 = input.values.value(bestPhase); output.hasL2 = true;
            output.lock2 = input.lli.value(bestPhase, -1); output.sii2 = input.ssi.value(bestPhase, 0);
            output.slip = output.slip || input.lli.value(bestPhase, 0) != 0;
        }
    }
}

PppStandardizedEpoch standardizeEpoch(const RinexEpoch& input, const PppOptions& options,
                                      const QMap<int, int>& gloChannels) {
    PppStandardizedEpoch result;
    result.gpsWeek = input.gpsWeek;
    result.gpsSow = input.gpsSow;
    for (const RinexObservation& raw : input.observations) {
        QChar b1, b2;
        if (!raw.sat.isValid() || !selectedBands(options, raw.sat.system, b1, b2)) continue;
        PppStandardizedObservation observation;
        observation.satKey = raw.sat.key();
        observation.gloChannel = raw.sat.system == QLatin1Char('R')
            ? gloChannels.value(raw.sat.number, 0) : 0;
        selectRinexBand(raw, options, b1, true, observation);
        selectRinexBand(raw, options, b2, false, observation);
        if (observation.hasC1) result.observations.push_back(observation);
    }
    return result;
}

QString inputFingerprint(const QStringList& paths) {
    QCryptographicHash hash(QCryptographicHash::Sha256);
    for (const QString& path : paths) {
        QFile file(path);
        const QFileInfo info(path);
        hash.addData(info.absoluteFilePath().toUtf8());
        hash.addData(QByteArray::number(info.size()));
        hash.addData(QByteArray::number(info.lastModified().toMSecsSinceEpoch()));
        if (file.open(QIODevice::ReadOnly)) hash.addData(file.read(4096));
    }
    return QString::fromLatin1(hash.result().toHex());
}

bool preciseStateAtTransmitTime(const QVector<PreciseSatelliteState>& samples,
                                const RinexEpoch& epoch,
                                const PppStandardizedObservation& observation,
                                const std::array<double, 3>& receiver,
                                PreciseSatelliteState& state,
                                QString* diagnostic) {
    if (!observation.hasC1) {
        if (diagnostic) *diagnostic = QStringLiteral("missing pseudorange for transmit time");
        return false;
    }
    if (!std::isfinite(observation.c1) || observation.c1 <= 1.0e7 || observation.c1 >= 1.0e8) {
        if (diagnostic) *diagnostic = QStringLiteral("invalid pseudorange for transmit time");
        return false;
    }
    const auto norm = [](const std::array<double, 3>& value) {
        return std::sqrt(value[0] * value[0] + value[1] * value[1] + value[2] * value[2]);
    };
    const double receiverNorm = norm(receiver);
    if (!std::isfinite(receiverNorm) || receiverNorm < 5.0e6 || receiverNorm > 7.0e6) {
        if (diagnostic) *diagnostic = QStringLiteral("receiver coordinate unavailable for transmit time");
        return false;
    }

    // The product is sampled at transmit time. Iterate the geometric range
    // because the observation is timestamped at reception, not emission.
    const double receiveSeconds = epoch.gpsWeek * 604800.0 + epoch.gpsSow;
    double targetSeconds = receiveSeconds - observation.c1 / GnssUtils::kC;
    int targetWeek = static_cast<int>(std::floor(targetSeconds / 604800.0));
    double targetSow = targetSeconds - targetWeek * 604800.0;
    if (!PreciseProductReader::interpolate(samples, targetWeek, targetSow,
                                           state, diagnostic)) {
        return false;
    }
    for (int iteration = 0; iteration < 3; ++iteration) {
        const std::array<double, 3> delta = {
            state.position[0] - receiver[0],
            state.position[1] - receiver[1],
            state.position[2] - receiver[2]};
        const double range = norm(delta);
        if (!std::isfinite(range) || range <= 1.0e7 || range >= 1.0e8) {
            if (diagnostic) *diagnostic = QStringLiteral("invalid geometric range for transmit time");
            return false;
        }
        targetSeconds = receiveSeconds - range / GnssUtils::kC;
        targetWeek = static_cast<int>(std::floor(targetSeconds / 604800.0));
        targetSow = targetSeconds - targetWeek * 604800.0;
        PreciseSatelliteState next;
        if (!PreciseProductReader::interpolate(samples, targetWeek, targetSow, next, diagnostic)) {
            return false;
        }
        state = next;
    }
    return true;
}

} // namespace

bool RinexReader::readObservation(const QString& path,
                                  RinexObservationFile& out,
                                  QString* error) {
    QFile file(path);
    if (!file.open(QIODevice::ReadOnly | QIODevice::Text)) {
        setError(error, QStringLiteral("cannot open RINEX OBS: %1").arg(path));
        return false;
    }
    out = RinexObservationFile();
    out.path = path;
    QTextStream in(&file);
    QString line;
    QChar currentSystem;
    int expectedTypes = 0;
    int remainingGloSlots = 0;
    while (!in.atEnd()) {
        line = in.readLine();
        const QString label = line.size() >= 60 ? line.mid(60).trimmed() : QString();
        if (label == QLatin1String("RINEX VERSION / TYPE")) {
            out.version = number(line.left(9));
            if (out.version < 3.0 || out.version >= 4.0) {
                setError(error, QStringLiteral("only RINEX 3.x is supported"));
                return false;
            }
        } else if (label == QLatin1String("MARKER NAME")) {
            out.markerName = line.left(60).trimmed();
        } else if (label == QLatin1String("REC # / TYPE / VERS")) {
            out.receiver = line.left(60).trimmed();
        } else if (label == QLatin1String("ANT # / TYPE")) {
            out.antenna = line.left(60).trimmed();
        } else if (label == QLatin1String("TIME OF FIRST OBS")) {
            const QString declared = line.mid(48, 3).trimmed();
            if (!declared.isEmpty()) out.timeSystem = declared.toUpper();
        } else if (label == QLatin1String("APPROX POSITION XYZ")) {
            const QStringList f = fields(line.left(60));
            if (f.size() >= 3) {
                bool ok1 = false, ok2 = false, ok3 = false;
                out.approximatePosition[0] = number(f.at(0), &ok1);
                out.approximatePosition[1] = number(f.at(1), &ok2);
                out.approximatePosition[2] = number(f.at(2), &ok3);
                out.hasApproximatePosition = ok1 && ok2 && ok3 &&
                    isPlausibleEcefPosition(out.approximatePosition);
            }
        } else if (label == QLatin1String("SYS / # / OBS TYPES")) {
            currentSystem = line.isEmpty() ? QChar() : line.at(0).toUpper();
            expectedTypes = line.mid(3, 3).trimmed().toInt();
            QStringList& types = out.observationTypes[currentSystem];
            for (int i = 0; i < 13 && types.size() < expectedTypes; ++i) {
                const QString type = line.mid(7 + i * 4, 3).trimmed();
                if (!type.isEmpty()) types.push_back(type);
            }
            while (types.size() < expectedTypes && !in.atEnd()) {
                const QString continuation = in.readLine();
                for (int i = 0; i < 13 && types.size() < expectedTypes; ++i) {
                    const QString type = continuation.mid(7 + i * 4, 3).trimmed();
                    if (!type.isEmpty()) types.push_back(type);
                }
            }
        } else if (label == QLatin1String("GLONASS SLOT / FRQ #")) {
            const QStringList slotFields = fields(line.left(60));
            int index = 0;
            if (remainingGloSlots == 0) {
                remainingGloSlots = slotFields.value(0).toInt();
                index = 1;
            }
            while (index + 1 < slotFields.size() && remainingGloSlots > 0) {
                const QString prn = slotFields.at(index++).toUpper();
                bool ok = false;
                const int channel = slotFields.at(index++).toInt(&ok);
                if (ok && prn.size() >= 2 && prn.at(0) == QLatin1Char('R')) {
                    out.glonassChannels[prn.mid(1).toInt()] = channel;
                    --remainingGloSlots;
                }
            }
        } else if (label == QLatin1String("END OF HEADER")) {
            break;
        }
    }
    if (out.version < 3.0 || out.observationTypes.isEmpty()) {
        setError(error, QStringLiteral("invalid RINEX OBS header"));
        return false;
    }

    while (!in.atEnd()) {
        line = in.readLine();
        if (!line.startsWith(QLatin1String(">"))) continue;
        const QStringList f = fields(line.mid(1));
        if (f.size() < 8) continue;
        bool flagOk = false;
        bool countOk = false;
        const int flag = f.at(6).toInt(&flagOk);
        const int count = f.at(7).toInt(&countOk);
        if (!flagOk || !countOk || count < 0) continue;
        int week = 0;
        double sow = 0.0;
        const QStringList epochFields = f.mid(0, 6);
        if (!rinexObservationGpsTime(epochFields, out.timeSystem, week, sow)) continue;
        RinexEpoch outEpoch;
        outEpoch.gpsWeek = week;
        outEpoch.gpsSow = sow;
        outEpoch.flag = flag;
        if (flag > 1) {
            // Special-event epochs carry event records, not satellite
            // observation records. Consume them without feeding them into PPP.
            for (int i = 0; i < count && !in.atEnd(); ++i) in.readLine();
            continue;
        }
        for (int i = 0; i < count && !in.atEnd(); ++i) {
            QString satLine = in.readLine();
            if (satLine.size() < 3) { --i; continue; }
            const QString prn = satLine.left(3).trimmed().toUpper();
            const QChar sys = systemForSat(prn);
            const QStringList types = out.observationTypes.value(sys);
            if (types.isEmpty()) continue;
            QString values = satLine.mid(3);
            // Standard RINEX wraps after five fields; some writers keep all
            // fields for a satellite on one long line.
            // Consume continuation lines only until all declared fields are
            // present, so both representations remain aligned.
            const int expectedValueLength = types.size() * 16;
            while (values.size() < expectedValueLength && !in.atEnd()) {
                values += in.readLine().mid(3);
            }
            RinexObservation obs;
            obs.sat.system = sys;
            obs.sat.number = prn.mid(1).toInt();
            obs.sat.navType = navTypeForSystem(sys);
            for (int t = 0; t < types.size(); ++t) {
                const int offset = t * 16;
                parseObsField(values.mid(offset, 16), types.at(t), obs);
            }
            if (!obs.values.isEmpty()) outEpoch.observations.push_back(obs);
        }
        if (!outEpoch.observations.isEmpty()) out.epochs.push_back(outEpoch);
    }
    if (out.epochs.isEmpty()) {
        setError(error, QStringLiteral("RINEX OBS contains no usable epochs"));
        return false;
    }
    return true;
}

bool RinexReader::readNavigation(const QString& path,
                                 QVector<RtcmBroadcastEphemeris>& out,
                                 QString* error) {
    QFile file(path);
    if (!file.open(QIODevice::ReadOnly | QIODevice::Text)) {
        setError(error, QStringLiteral("cannot open RINEX NAV: %1").arg(path));
        return false;
    }
    out.clear();
    QTextStream in(&file);
    QString line;
    double version = 0.0;
    while (!in.atEnd()) {
        line = in.readLine();
        const QString label = line.size() >= 60 ? line.mid(60).trimmed() : QString();
        if (label == QLatin1String("RINEX VERSION / TYPE")) version = number(line.left(9));
        if (label == QLatin1String("END OF HEADER")) break;
    }
    if (version < 3.0 || version >= 4.0) {
        setError(error, QStringLiteral("only RINEX 3.x NAV is supported"));
        return false;
    }
    QString pendingLine;
    bool hasPendingLine = false;
    while (hasPendingLine || !in.atEnd()) {
        line = hasPendingLine ? pendingLine : in.readLine();
        hasPendingLine = false;
        if (line.trimmed().isEmpty()) continue;
        const QString prn = line.left(3).trimmed().toUpper();
        if (prn.size() < 3) continue;
        const QChar sys = systemForSat(prn);
        const QString recordLine = line;
        // GLONASS and SBAS records contain four lines; all other RINEX 3
        // navigation records contain eight. Consume unsupported records with
        // their native length so they cannot desynchronize supported records.
        const int recordLines = sys == QLatin1Char('S') ? 4 :
            (sys == QLatin1Char('R') ? 4 : 8);
        QVector<QVector<double>> rows;
        QVector<double> row0;
        int recordWeek = 0;
        double recordSow = 0.0;
        const bool firstValid = parseNavigationEpoch(line, recordWeek, recordSow, row0);
        if (firstValid) rows.push_back(row0);
        bool continuationValid = true;
        for (int r = 1; r < recordLines && !in.atEnd(); ++r) {
            QVector<double> row;
            if (!parseFixedNavigationFields(in.readLine(), 4, 4, row)) continuationValid = false;
            rows.push_back(row);
        }
        // RINEX 3.05 permits an optional fifth GLONASS line, while many
        // valid mixed files still use the four-line legacy record.
        if (sys == QLatin1Char('R') && version >= 3.05 && !in.atEnd()) {
            const QString candidate = in.readLine();
            if (candidate.startsWith(QLatin1Char(' ')) || candidate.startsWith(QLatin1Char('\t'))) {
                QVector<double> row;
                if (parseFixedNavigationFields(candidate, 4, 4, row)) rows.push_back(row);
            } else {
                pendingLine = candidate;
                hasPendingLine = true;
            }
        }
        if (!supportsBroadcastNavigation(sys) || !firstValid || !continuationValid) continue;
        RtcmBroadcastEphemeris eph;
        eph.sat.system = sys;
        eph.sat.number = prn.mid(1).toInt();
        eph.sat.navType = navTypeForSystem(sys);
        eph.messageType = eph.sat.navType;
        eph.validOrbit = true;
        if (sys == QLatin1Char('R')) {
            if (rows.size() < 4 || rows.at(0).size() < 9) continue;
            const QStringList dateFields = fields(recordLine.mid(3, 20));
            const int year = dateFields.value(0).toInt();
            const int month = dateFields.value(1).toInt();
            const int day = dateFields.value(2).toInt();
            // RINEX GLONASS navigation epochs are UTC(SU)+03:00. Convert
            // the civil label to UTC before applying UTC->GPST leap seconds.
            const int hour = dateFields.value(3).toInt();
            const int minute = dateFields.value(4).toInt();
            const double second = number(dateFields.value(5));
            const QDateTime gloLabel(QDate(year, month, day),
                                     QTime(hour, minute, static_cast<int>(std::floor(second))),
                                     Qt::UTC);
            const QDateTime utcLabel = gloLabel.addSecs(-3 * 3600);
            recordSow -= 3.0 * 3600.0;
            while (recordSow < 0.0) {
                recordSow += 604800.0;
                --recordWeek;
            }
            addGpsLeapSeconds(utcLabel.date().year(), utcLabel.date().month(),
                              utcLabel.date().day(), recordWeek, recordSow);
            eph.week = recordWeek;
            eph.tocSec = recordSow;
            eph.glonassTkSec = static_cast<int>(std::llround(rows.at(0).value(8)));
            eph.glonassTbSec = 0;
            eph.glonassXPosKm = rows.at(1).value(0);
            eph.glonassXVelKmps = rows.at(1).value(1);
            eph.glonassXAccKmps2 = rows.at(1).value(2);
            eph.glonassYPosKm = rows.at(2).value(0);
            eph.glonassYVelKmps = rows.at(2).value(1);
            eph.glonassYAccKmps2 = rows.at(2).value(2);
            eph.glonassZPosKm = rows.at(3).value(0);
            eph.glonassZVelKmps = rows.at(3).value(1);
            eph.glonassZAccKmps2 = rows.at(3).value(2);
            eph.glonassFreqNum = static_cast<int>(std::llround(rows.at(2).value(3)));
            eph.glonassTauSec = rows.at(0).value(6);
            eph.glonassGamma = rows.at(0).value(7);
        } else {
            if (rows.size() < 7) continue;
            const int rinexWeek = static_cast<int>(std::llround(rows.at(5).value(2, recordWeek)));
            // RINEX Galileo week values are continuous GPS weeks. The shared
            // RTCM ephemeris model stores the native GST week and applies its
            // 1024-week origin when evaluating a Galileo state.
            eph.week = sys == QLatin1Char('E') ? rinexWeek - 1024 : rinexWeek;
            eph.tocSec = recordSow;
            eph.toeSec = rows.at(3).value(0);
            eph.af0 = rows.at(0).value(6);
            eph.af1 = rows.at(0).value(7);
            eph.af2 = rows.at(0).value(8);
            eph.crs = rows.at(1).value(1);
            eph.deltaN = rows.at(1).value(2);
            eph.M0 = rows.at(1).value(3);
            eph.cuc = rows.at(2).value(0);
            eph.e = rows.at(2).value(1);
            eph.cus = rows.at(2).value(2);
            eph.sqrtA = rows.at(2).value(3);
            eph.cic = rows.at(3).value(1);
            eph.omega0 = rows.at(3).value(2);
            eph.cis = rows.at(3).value(3);
            eph.i0 = rows.at(4).value(0);
            eph.crc = rows.at(4).value(1);
            eph.omega = rows.at(4).value(2);
            eph.omegaDot = rows.at(4).value(3);
            eph.idot = rows.at(5).value(0);
            eph.health = static_cast<int>(std::llround(rows.at(6).value(1)));
            eph.tgd1 = rows.at(6).value(2);
            eph.fitIntervalFlag = static_cast<int>(std::llround(rows.at(7).value(1)));
            if (sys == QLatin1Char('C')) {
                eph.bdsWeek = rinexWeek;
                eph.iod = (static_cast<int>(std::floor(eph.toeSec + 14.0)) / 720) % 240;
            } else {
                eph.iod = static_cast<int>(std::llround(rows.at(1).value(0)));
            }
        }
        out.push_back(eph);
    }
    if (out.isEmpty()) {
        setError(error, QStringLiteral("RINEX NAV contains no usable ephemerides"));
        return false;
    }
    return true;
}

bool SsrFileReader::read(const QStringList& paths, Result& out, QString* error) {
    out = Result();
    for (const QString& path : paths) {
        QFile file(path);
        if (!file.open(QIODevice::ReadOnly | QIODevice::Text)) {
            setError(error, QStringLiteral("cannot open SSR: %1").arg(path));
            return false;
        }
        QTextStream in(&file);
        QString line;
        while (!in.atEnd()) {
            line = in.readLine();
            if (!line.startsWith(QLatin1String(">"))) continue;
            const QStringList h = fields(line.mid(1));
            if (h.size() < 10) continue;
            const QString kind = h.at(0).toUpper();
            int week = 0; double sow = 0.0;
            if (!parseDateGps(h, 1, week, sow)) continue;
            const int update = h.at(7).toInt();
            const int count = h.at(8).toInt();
            const QString source = h.at(9);
            if (count < 0) continue;
            auto makeMessage = [&](RtcmSsrCorrectionKind k, QChar sys) {
                RtcmSsrMessage msg;
                msg.kind = k;
                msg.system = sys;
                msg.epochTime = static_cast<int>(std::llround(sow));
                msg.gpsWeek = week;
                msg.epochTimeIsGps = true;
                msg.updateIntervalIndex = update;
                msg.messageType = 0;
                msg.ssrIod = 0;
                msg.multipleMessage = false;
                Q_UNUSED(source);
                return msg;
            };
            if (kind == QLatin1String("VTEC")) {
                PppVtecModel model;
                model.gpsWeek = week; model.gpsSow = sow;
                model.updateIntervalIndex = update; model.stationId = source;
                for (int layerNo = 0; layerNo < count; ++layerNo) {
                    QVector<double> header;
                    if (!readLineTokens(in, 4, header)) break;
                    const int maxDeg = static_cast<int>(std::llround(header.at(1)));
                    const int maxOrd = static_cast<int>(std::llround(header.at(2)));
                    if (maxDeg < 0 || maxOrd < 0 || maxDeg > 30 || maxOrd > 30) break;
                    PppVtecLayer layer;
                    layer.heightMeters = header.at(3);
                    layer.cosine = QVector<QVector<double>>(maxDeg + 1, QVector<double>(maxOrd + 1));
                    layer.sine = QVector<QVector<double>>(maxDeg + 1, QVector<double>(maxOrd + 1));
                    QVector<double> coeff;
                    const int n = (maxDeg + 1) * (maxOrd + 1);
                    if (!readLineTokens(in, 2 * n, coeff)) break;
                    for (int i = 0; i < n; ++i) {
                        layer.cosine[i / (maxOrd + 1)][i % (maxOrd + 1)] = coeff.at(i);
                        layer.sine[i / (maxOrd + 1)][i % (maxOrd + 1)] = coeff.at(n + i);
                    }
                    model.layers.push_back(layer);
                }
                if (model.isValid()) out.vtec.push_back(model);
                continue;
            }
            if (kind == QLatin1String("PHASE_BIAS")) {
                const QStringList consistency = fields(in.readLine());
                const int dispersive = consistency.value(0).toInt();
                const int mw = consistency.value(1).toInt();
                QMap<QChar, RtcmSsrMessage> bySystem;
                for (int i = 0; i < count && !in.atEnd(); ++i) {
                    const QStringList f = fields(in.readLine());
                    if (f.size() < 4) continue;
                    const QString prn = f.at(0).toUpper();
                    const QChar sys = systemForSat(prn);
                    RtcmSsrMessage& msg = bySystem[sys];
                    if (msg.satellites.isEmpty()) msg = makeMessage(RtcmSsrCorrectionKind::PhaseBias, sys);
                    RtcmSsrSatelliteData sat;
                    sat.sat.system = sys; sat.sat.number = prn.mid(1).toInt();
                    sat.sat.navType = navTypeForSystem(sys);
                    sat.yawAngleRad = number(f.value(1)) * 3.14159265358979323846 / 180.0;
                    sat.yawRateRadPerSec = number(f.value(2)) * 3.14159265358979323846 / 180.0;
                    const int nBias = f.value(3).toInt();
                    int p = 4;
                    for (int b = 0; b < nBias && p + 4 < f.size(); ++b) {
                        RtcmSsrPhaseBiasEntry entry;
                        entry.signalCode = rinexType2chToSsrSignalCode(sys, f.at(p++));
                        entry.biasMeters = number(f.at(p++));
                        entry.signalIntegerIndicator = f.at(p++).toInt();
                        entry.wideLaneIntegerIndicator = f.at(p++).toInt();
                        entry.discontinuityCounter = f.at(p++).toInt();
                        if (entry.signalCode >= 0) sat.phaseBiases.push_back(entry);
                    }
                    msg.dispersiveBiasConsistency = dispersive;
                    msg.mwConsistency = mw;
                    msg.satellites.push_back(sat);
                }
                for (auto it = bySystem.constBegin(); it != bySystem.constEnd(); ++it) out.messages.push_back(it.value());
                continue;
            }
            QMap<QChar, RtcmSsrMessage> bySystem;
            for (int i = 0; i < count && !in.atEnd(); ++i) {
                const QStringList f = fields(in.readLine());
                if (f.size() < 2) continue;
                const QString prn = f.at(0).toUpper();
                const QChar sys = systemForSat(prn);
                RtcmSsrCorrectionKind messageKind = RtcmSsrCorrectionKind::Unknown;
                if (kind == QLatin1String("ORBIT")) messageKind = RtcmSsrCorrectionKind::Orbit;
                else if (kind == QLatin1String("CLOCK")) messageKind = RtcmSsrCorrectionKind::Clock;
                else if (kind == QLatin1String("CODE_BIAS")) messageKind = RtcmSsrCorrectionKind::CodeBias;
                else { out.warnings << QStringLiteral("unsupported SSR block %1 in %2").arg(kind, path); break; }
                RtcmSsrMessage& msg = bySystem[sys];
                if (msg.satellites.isEmpty()) msg = makeMessage(messageKind, sys);
                RtcmSsrSatelliteData sat;
                sat.sat.system = sys; sat.sat.number = prn.mid(1).toInt(); sat.sat.navType = navTypeForSystem(sys);
                sat.iod = f.value(1).toInt();
                if (messageKind == RtcmSsrCorrectionKind::Orbit && f.size() >= 8) {
                    sat.hasOrbit = true;
                    sat.deltaRadial = number(f.at(2)); sat.deltaAlongTrack = number(f.at(3)); sat.deltaCrossTrack = number(f.at(4));
                    sat.dotDeltaRadial = number(f.at(5)) * 1e-3; sat.dotDeltaAlongTrack = number(f.at(6)) * 1e-3; sat.dotDeltaCrossTrack = number(f.at(7)) * 1e-3;
                } else if (messageKind == RtcmSsrCorrectionKind::Clock && f.size() >= 5) {
                    sat.hasClock = true; sat.deltaC0 = number(f.at(2)); sat.deltaC1 = number(f.at(3)) * 1e-3; sat.deltaC2 = number(f.at(4)) * 1e-3;
                } else if (messageKind == RtcmSsrCorrectionKind::CodeBias) {
                    const int nBias = f.value(1).toInt();
                    int p = 2;
                    for (int b = 0; b < nBias && p + 1 < f.size(); ++b) {
                        RtcmSsrCodeBiasEntry entry;
                        entry.signalCode = rinexType2chToSsrSignalCode(sys, f.at(p++)); entry.biasMeters = number(f.at(p++));
                        if (entry.signalCode >= 0) sat.codeBiases.push_back(entry);
                    }
                }
                msg.satellites.push_back(sat);
            }
            for (auto it = bySystem.constBegin(); it != bySystem.constEnd(); ++it) out.messages.push_back(it.value());
        }
    }
    QVector<RtcmSsrMessage> merged;
    QMap<QString, int> groupIndex;
    QMap<QString, RtcmSsrSatelliteData> satelliteIndex;
    for (const RtcmSsrMessage& message : out.messages) {
        const QString groupKey = QString("%1|%2|%3|%4")
            .arg(static_cast<int>(message.kind)).arg(message.gpsWeek)
            .arg(message.epochTime).arg(message.system);
        int index = groupIndex.value(groupKey, -1);
        if (index < 0) {
            index = merged.size();
            RtcmSsrMessage empty = message;
            empty.satellites.clear();
            merged.push_back(empty);
            groupIndex[groupKey] = index;
        } else if (!sameSsrHeader(merged.at(index), message)) {
            setError(error, QStringLiteral("conflicting SSR headers at %1").arg(groupKey));
            return false;
        }
        for (const RtcmSsrSatelliteData& satellite : message.satellites) {
            const QString key = groupKey + QLatin1Char('|') + satellite.sat.key();
            if (satelliteIndex.contains(key)) {
                if (!sameSsrSatellite(satelliteIndex.value(key), satellite)) {
                    setError(error, QStringLiteral("conflicting SSR satellite data at %1").arg(key));
                    return false;
                }
                continue;
            }
            satelliteIndex[key] = satellite;
            merged[index].satellites.push_back(satellite);
        }
    }
    out.messages = merged;
    QVector<PppVtecModel> mergedVtec;
    for (const PppVtecModel& model : out.vtec) {
        int duplicate = -1;
        for (int i = 0; i < mergedVtec.size(); ++i) {
            const PppVtecModel& existing = mergedVtec.at(i);
            if (existing.gpsWeek == model.gpsWeek &&
                std::fabs(existing.gpsSow - model.gpsSow) <= 1.0e-9 &&
                existing.stationId == model.stationId) {
                duplicate = i;
                break;
            }
        }
        if (duplicate < 0) {
            mergedVtec.push_back(model);
        } else if (!sameVtecModel(mergedVtec.at(duplicate), model)) {
            setError(error, QStringLiteral("conflicting VTEC data at week=%1 sow=%2 source=%3")
                         .arg(model.gpsWeek).arg(model.gpsSow, 0, 'f', 3).arg(model.stationId));
            return false;
        }
    }
    out.vtec = mergedVtec;
    if (out.messages.isEmpty() && out.vtec.isEmpty()) {
        setError(error, QStringLiteral("SSR contains no supported blocks"));
        return false;
    }
    return true;
}

bool PreciseProductReader::readSp3(const QString& path,
                                   QMap<QString, QVector<PreciseSatelliteState>>& out,
                                   QString* error) {
    QFile file(path);
    if (!file.open(QIODevice::ReadOnly | QIODevice::Text)) { setError(error, QStringLiteral("cannot open SP3: %1").arg(path)); return false; }
    out.clear(); QTextStream in(&file); QString line; int week = 0; double sow = 0.0;
    while (!in.atEnd()) {
        line = in.readLine();
        if (line.startsWith(QLatin1String("*"))) {
            const QStringList f = fields(line.mid(1));
            if (f.size() >= 6 && !parseDateGps(f, 0, week, sow)) {
                week = 0;
                sow = 0.0;
            }
        } else if (line.startsWith(QLatin1String("P")) && line.size() >= 4) {
            const QString prn = line.mid(1, 3).trimmed().toUpper();
            const QStringList f = fields(line.mid(4));
            if (f.size() < 4) continue;
            bool okX = false, okY = false, okZ = false;
            const double x = number(f.at(0), &okX) * 1000.0;
            const double y = number(f.at(1), &okY) * 1000.0;
            const double z = number(f.at(2), &okZ) * 1000.0;
            const double radius = std::sqrt(x * x + y * y + z * z);
            PreciseSatelliteState state; state.sat.system = systemForSat(prn); state.sat.number = prn.mid(1).toInt(); state.sat.navType = navTypeForSystem(state.sat.system); state.gpsWeek = week; state.gpsSow = sow;
            if (!okX || !okY || !okZ || !state.sat.isValid() || !std::isfinite(radius) ||
                radius < 2.0e7 || radius > 6.0e7) continue;
            state.position = {x, y, z};
            // The precise-mode contract requires an explicit CLK file.  SP3
            // clock columns are deliberately not used as a fallback.
            state.hasClock = false;
            state.clockSec = 0.0;
            out[satKey(state.sat)].push_back(state);
        }
    }
    sortStates(out);
    if (out.isEmpty()) { setError(error, QStringLiteral("SP3 contains no satellite positions")); return false; }
    return true;
}

bool PreciseProductReader::readClock(const QString& path,
                                     QMap<QString, QVector<PreciseSatelliteState>>& states,
                                     QString* error) {
    QFile file(path);
    if (!file.open(QIODevice::ReadOnly | QIODevice::Text)) { setError(error, QStringLiteral("cannot open CLK: %1").arg(path)); return false; }
    QTextStream in(&file); QString line;
    while (!in.atEnd()) {
        line = in.readLine();
        if (!line.startsWith(QLatin1String("AS "))) continue;
        const QStringList f = fields(line);
        if (f.size() < 10) continue;
        const QString prn = f.at(1).toUpper(); int week = 0; double sow = 0.0;
        if (!parseDateGps(f, 2, week, sow)) continue;
        const double clock = number(f.at(9));
        if (!std::isfinite(clock)) continue;
        const QString key = QStringLiteral("%1%2:%3").arg(prn.at(0)).arg(prn.mid(1).toInt(), 2, 10, QLatin1Char('0')).arg(navTypeForSystem(prn.at(0)));
        bool found = false;
        for (PreciseSatelliteState& state : states[key]) {
            if (state.gpsWeek == week && std::fabs(state.gpsSow - sow) < 0.001) { state.clockSec = clock; state.hasClock = true; found = true; break; }
        }
        if (!found) { PreciseSatelliteState state; state.sat.system = prn.at(0); state.sat.number = prn.mid(1).toInt(); state.sat.navType = navTypeForSystem(state.sat.system); state.gpsWeek = week; state.gpsSow = sow; state.clockSec = clock; state.hasClock = true; states[key].push_back(state); }
    }
    sortStates(states);
    return true;
}

bool PreciseProductReader::interpolate(const QVector<PreciseSatelliteState>& samples,
                                       int gpsWeek, double gpsSow,
                                       PreciseSatelliteState& out,
                                       QString* diagnostic) {
    auto reject = [diagnostic](const QString& reason) {
        if (diagnostic) *diagnostic = reason;
        return false;
    };
    const double target = gpsWeek * 604800.0 + gpsSow;

    struct Window {
        QVector<int> indices;
        int nearest = -1;
        double distance = std::numeric_limits<double>::max();
    };
    const auto makeWindow = [&](bool clock, Window& window, QString* reason) {
        QVector<int> available;
        for (int i = 0; i < samples.size(); ++i) {
            const bool usable = clock
                ? samples.at(i).hasClock && std::isfinite(samples.at(i).clockSec)
                : std::isfinite(samples.at(i).position[0]) &&
                  std::isfinite(samples.at(i).position[1]) &&
                  std::isfinite(samples.at(i).position[2]);
            if (usable) available.push_back(i);
        }
        if (available.size() < 9) {
            if (reason) *reason = clock
                ? QStringLiteral("insufficient clock samples")
                : QStringLiteral("insufficient position samples");
            return false;
        }
        for (int i = 0; i < available.size(); ++i) {
            const int sampleIndex = available.at(i);
            const double t = samples.at(sampleIndex).gpsWeek * 604800.0 + samples.at(sampleIndex).gpsSow;
            const double distance = std::fabs(t - target);
            if (distance < window.distance) {
                window.distance = distance;
                window.nearest = i;
            }
        }
        if (window.distance > 900.0) {
            if (reason) *reason = QStringLiteral("target outside product coverage");
            return false;
        }
        if (window.nearest < 4 || window.nearest + 4 >= available.size()) {
            if (reason) *reason = QStringLiteral("9-point interpolation window unavailable");
            return false;
        }
        for (int i = window.nearest - 4; i < window.nearest + 4; ++i) {
            const double left = samples.at(available.at(i)).gpsWeek * 604800.0 + samples.at(available.at(i)).gpsSow;
            const double right = samples.at(available.at(i + 1)).gpsWeek * 604800.0 + samples.at(available.at(i + 1)).gpsSow;
            const double interval = right - left;
            if (interval <= 0.0 || interval > 1800.0) {
                if (reason) *reason = QStringLiteral("invalid product interval");
                return false;
            }
            if (i > window.nearest - 4) {
                const double previousLeft = samples.at(available.at(i - 1)).gpsWeek * 604800.0 + samples.at(available.at(i - 1)).gpsSow;
                const double previousInterval = left - previousLeft;
                if (std::fabs(interval - previousInterval) > 1e-3) {
                    if (reason) *reason = QStringLiteral("product interval discontinuity");
                    return false;
                }
            }
        }
        for (int i = window.nearest - 4; i <= window.nearest + 4; ++i) {
            window.indices.push_back(available.at(i));
        }
        return true;
    };

    Window positionWindow;
    QString reason;
    if (!makeWindow(false, positionWindow, &reason)) return reject(reason);
    Window clockWindow;
    if (!makeWindow(true, clockWindow, &reason)) {
        return reject(reason);
    }
    out = samples.at(positionWindow.indices.at(4));
    out.gpsWeek = gpsWeek;
    out.gpsSow = gpsSow;
    const auto lagrange = [&](const QVector<int>& indices, int component, bool clock) {
        double value = 0.0;
        for (int i = 0; i < indices.size(); ++i) {
            const double ti = samples.at(indices.at(i)).gpsWeek * 604800.0 + samples.at(indices.at(i)).gpsSow;
            double weight = 1.0;
            for (int j = 0; j < indices.size(); ++j) if (i != j) {
                const double tj = samples.at(indices.at(j)).gpsWeek * 604800.0 + samples.at(indices.at(j)).gpsSow;
                if (std::fabs(ti - tj) < 1e-9) return std::numeric_limits<double>::quiet_NaN();
                weight *= (target - tj) / (ti - tj);
            }
            value += weight * (clock ? samples.at(indices.at(i)).clockSec
                                     : samples.at(indices.at(i)).position[component]);
        }
        return value;
    };
    for (int component = 0; component < 3; ++component) {
        out.position[component] = lagrange(positionWindow.indices, component, false);
        if (!std::isfinite(out.position[component])) return reject(QStringLiteral("duplicate product epoch"));
    }
    out.clockSec = lagrange(clockWindow.indices, 0, true);
    if (!std::isfinite(out.clockSec)) return reject(QStringLiteral("duplicate clock epoch"));
    out.hasClock = true;
    out.hasVelocity = false;
    for (int c = 0; c < 3; ++c) {
        const double ta = samples.at(positionWindow.indices.at(3)).gpsWeek * 604800.0 + samples.at(positionWindow.indices.at(3)).gpsSow;
        const double tb = samples.at(positionWindow.indices.at(5)).gpsWeek * 604800.0 + samples.at(positionWindow.indices.at(5)).gpsSow;
        if (tb != ta) out.velocity[c] =
            (samples.at(positionWindow.indices.at(5)).position[c] -
             samples.at(positionWindow.indices.at(3)).position[c]) / (tb - ta);
    }
    out.hasVelocity = true;
    // CLK is a mandatory product in post-processing precise mode. Do not
    // silently substitute a neighbouring clock or the SP3 clock column.
    if (!out.hasClock) return reject(QStringLiteral("clock unavailable"));
    if (diagnostic) diagnostic->clear();
    return true;
}

PostPppService::PostPppService(QObject* parent) : QObject(parent) {}

void PostPppService::cancel() { _cancelRequested.storeRelease(1); }

void PostPppService::run(PostPppJobConfig config) {
    _cancelRequested.storeRelease(0);
    const auto fail = [this, &config](const QString& reason) {
        QJsonObject manifest;
        manifest[QStringLiteral("status")] = QStringLiteral("failed");
        manifest[QStringLiteral("failureReason")] = reason;
        manifest[QStringLiteral("cancelled")] = false;
        manifest[QStringLiteral("jobName")] = config.jobName;
        manifest[QStringLiteral("observation")] = config.observationPath;
        manifest[QStringLiteral("navigation")] = config.navigationPath;
        manifest[QStringLiteral("sp3")] = config.sp3Path;
        manifest[QStringLiteral("clock")] = config.clockPath;
        manifest[QStringLiteral("ssrFiles")] = QJsonArray::fromStringList(config.ssrPaths);
        manifest[QStringLiteral("productMode")] = config.productMode;
        manifest[QStringLiteral("sampleSec")] = config.sampleSec;
        manifest[QStringLiteral("jsonEnabled")] = config.jsonEnabled;
        manifest[QStringLiteral("jsonReorderBufferMs")] = config.jsonReorderBufferMs;
        manifest[QStringLiteral("jsonFlushIntervalMs")] = config.jsonFlushIntervalMs;
        manifest[QStringLiteral("pppOptions")] = pppOptionsJson(config.options);
        manifest[QStringLiteral("inputFingerprint")] = inputFingerprint(QStringList()
            << config.observationPath << config.navigationPath << config.sp3Path
            << config.clockPath << config.ssrPaths);
        QDir output(config.outputDirectory);
        if (!config.outputDirectory.trimmed().isEmpty() &&
            (output.exists() || output.mkpath(QStringLiteral(".")))) {
            const QByteArray payload = QJsonDocument(manifest).toJson(QJsonDocument::Indented);
            for (const QString& name : {QStringLiteral("manifest.json"), QStringLiteral("summary.json")}) {
                QFile file(output.filePath(name));
                if (file.open(QIODevice::WriteOnly | QIODevice::Text) &&
                    file.write(payload) == payload.size()) {
                    file.flush();
                }
            }
        }
        emit failed(reason);
    };
    if (config.productMode != QLatin1String("broadcast") &&
        config.productMode != QLatin1String("precise")) {
        fail(QStringLiteral("unsupported post-processing product mode"));
        return;
    }
    if (config.navigationPath.isEmpty()) {
        fail(QStringLiteral("post-processing requires a Broadcast NAV file"));
        return;
    }
    if (config.productMode == QLatin1String("broadcast") && config.ssrPaths.isEmpty()) {
        fail(QStringLiteral("Broadcast NAV mode requires SSR files"));
        return;
    }
    if (config.productMode == QLatin1String("precise") && !config.ssrPaths.isEmpty()) {
        fail(QStringLiteral("Broadcast NAV + SP3+CLK mode is mutually exclusive with SSR files"));
        return;
    }
    RinexObservationFile obs;
    QString error;
    if (!RinexReader::readObservation(config.observationPath, obs, &error)) { fail(error); return; }
    QVector<RtcmBroadcastEphemeris> nav;
    if (!RinexReader::readNavigation(config.navigationPath, nav, &error)) { fail(error); return; }
    QMap<QChar, int> navigationRecordCounts;
    for (const RtcmBroadcastEphemeris& eph : nav) ++navigationRecordCounts[eph.sat.system];
    QStringList navigationSummary;
    for (auto it = navigationRecordCounts.constBegin(); it != navigationRecordCounts.constEnd(); ++it) {
        navigationSummary << QStringLiteral("%1=%2").arg(it.key()).arg(it.value());
    }
    emit logMessage(QStringLiteral("Post-processing NAV records: %1")
                        .arg(navigationSummary.join(QLatin1Char(' '))));
    SsrFileReader::Result ssr;
    if (!config.ssrPaths.isEmpty() && !SsrFileReader::read(config.ssrPaths, ssr, &error)) { fail(error); return; }
    QMap<QString, QVector<PreciseSatelliteState>> precise;
    if (config.productMode == QLatin1String("precise")) {
        if (!PreciseProductReader::readSp3(config.sp3Path, precise, &error) ||
            !PreciseProductReader::readClock(config.clockPath, precise, &error)) { fail(error); return; }
    }
    QDir output(config.outputDirectory);
    if (!output.exists() && !output.mkpath(QStringLiteral("."))) { fail(QStringLiteral("cannot create post-processing output directory")); return; }
    QDir ztdOutput(config.ztdOutputDirectory.isEmpty() ? config.outputDirectory : config.ztdOutputDirectory);
    if (!ztdOutput.exists() && !ztdOutput.mkpath(QStringLiteral("."))) { fail(QStringLiteral("cannot create post-processing ZTD output directory")); return; }
    const int sample = qMax(1, config.sampleSec);
    const auto isSampled = [sample](const RinexEpoch& epoch) {
        GnssUtils::GpsTime time;
        time.week = epoch.gpsWeek;
        time.sow = epoch.gpsSow;
        time.valid = true;
        return GnssUtils::isGpsTimeOnSampleGrid(time, sample);
    };
    int totalSelected = 0;
    for (const RinexEpoch& epoch : obs.epochs) {
        if (isSampled(epoch)) ++totalSelected;
    }
    emit started(config.outputDirectory, totalSelected);
    PppEngine engine;
    PppOptions options = config.options;
    PppCrdStationOptions station = config.station;
    QString coordinateSource = QStringLiteral("PPP Settings manual coordinate");
    if (station.valid) {
        coordinateSource = QStringLiteral("CRD match");
    } else if (obs.hasApproximatePosition) {
        station.valid = true;
        station.station = obs.markerName;
        station.roverAntenna = obs.antenna;
        station.receiver = obs.receiver;
        for (int i = 0; i < 3; ++i) station.aprXyz[i] = obs.approximatePosition[i];
        coordinateSource = QStringLiteral("RINEX OBS approximate coordinate");
    }
    if (station.valid) options.crdStationsBySource[QStringLiteral("POST")] = station;
    engine.setOptions(options);
    PppStreamSelection selection;
    selection.broadcastId = QStringLiteral("POST_NAV");
    if (!ssr.messages.isEmpty()) { selection.orbitId = config.productMode == QLatin1String("broadcast") ? QStringLiteral("POST_SSR") : QString(); selection.clockId = selection.orbitId; selection.codeBiasId = QStringLiteral("POST_SSR"); selection.phaseBiasId = QStringLiteral("POST_SSR"); }
    engine.setStreamSelection(selection);
    PppTropospherePrior tropoPrior;
    QString gfsPriorDiagnostic;
    if (options.troposphereMode == PppTroposphereMode::GfsZwdInitialization) {
        std::array<double, 3> ecef = {options.aprXyz[0], options.aprXyz[1], options.aprXyz[2]};
        if (station.valid) {
            ecef = {station.aprXyz[0], station.aprXyz[1], station.aprXyz[2]};
        }
        double latitudeDeg = 0.0;
        double longitudeDeg = 0.0;
        double heightM = 0.0;
        if (!GfsZwdProvider::ecefToGeodetic(ecef, latitudeDeg, longitudeDeg, heightM)) {
            tropoPrior.source = QStringLiteral("GFS profile integral");
            tropoPrior.status = QStringLiteral("gfs_unavailable_fallback");
            tropoPrior.reason = QStringLiteral("no valid receiver ECEF coordinate for GFS request");
            gfsPriorDiagnostic = tropoPrior.reason;
        } else {
            const RinexEpoch* firstSelected = nullptr;
            for (const RinexEpoch& candidate : obs.epochs) {
                if (isSampled(candidate)) {
                    firstSelected = &candidate;
                    break;
                }
            }
            if (!firstSelected) {
                tropoPrior.source = QStringLiteral("GFS profile integral");
                tropoPrior.status = QStringLiteral("gfs_unavailable_fallback");
                tropoPrior.reason = QStringLiteral("observation file has no sampled epochs");
                gfsPriorDiagnostic = tropoPrior.reason;
            } else {
                GfsZwdRequest request;
                request.requestedUtc = GnssUtils::dateTimeFromGpsWeek(firstSelected->gpsWeek,
                                                                       firstSelected->gpsSow);
                request.latitudeDeg = latitudeDeg;
                request.longitudeDeg = longitudeDeg;
                request.receiverHeightM = heightM;
                request.cacheDirectory = options.gfsCacheDirectory;
                request.downloadBaseUrl = options.gfsDownloadBaseUrl;
                request.timeoutSec = options.gfsPrefetchTimeoutSec;
                request.maxAgeSec = options.gfsMaxAgeSec;
                if (!GfsZwdProvider::fetchPrior(request, options.gfsZwdMethod, tropoPrior,
                                                &gfsPriorDiagnostic)) {
                    tropoPrior.valid = false;
                    tropoPrior.source = QStringLiteral("GFS profile integral");
                    tropoPrior.status = QStringLiteral("gfs_unavailable_fallback");
                    tropoPrior.reason = gfsPriorDiagnostic;
                }
            }
        }
        engine.setTropospherePrior(QStringLiteral("POST"), tropoPrior);
        emit logMessage(tropoPrior.valid
            ? QString("GFS ZWD prior ready: zwd=%1m status=%2 product=%3+f%4 selection=%5")
                  .arg(tropoPrior.zwdM, 0, 'f', 4).arg(tropoPrior.status)
                  .arg(tropoPrior.cycle)
                  .arg(tropoPrior.forecastLeadSec / 3600, 3, 10, QLatin1Char('0'))
                  .arg(tropoPrior.productSelection.isEmpty() ? QStringLiteral("nominal") : tropoPrior.productSelection)
            : QString("GFS ZWD prior fallback: %1")
                  .arg(tropoPrior.reason.isEmpty() ? gfsPriorDiagnostic : tropoPrior.reason));
    }
    struct PostNavigationRecord {
        RtcmBroadcastEphemeris eph;
        std::shared_ptr<GnssEphemeris> model;
        double referenceSeconds = 0.0;
        bool hasReference = false;
    };
    QVector<PostNavigationRecord> navigationRecords;
    navigationRecords.reserve(nav.size());
    QMap<QChar, int> navigationModelCounts;
    const RinexEpoch& firstEpoch = obs.epochs.first();
    for (const RtcmBroadcastEphemeris& eph : nav) {
        std::shared_ptr<GnssEphemeris> model;
        if (GnssEphemeris::fromRtcm(eph, model)) {
            int tocWeek = 0;
            int toeWeek = 0;
            double tocSow = 0.0;
            double toeSow = 0.0;
            PostNavigationRecord record;
            record.eph = eph;
            record.model = model;
            record.hasReference = model->computeReferenceTimes(firstEpoch.gpsWeek, firstEpoch.gpsSow,
                                                               tocWeek, tocSow, toeWeek, toeSow);
            if (record.hasReference) {
                record.referenceSeconds = static_cast<double>(tocWeek) * 604800.0 + tocSow;
            }
            navigationRecords.push_back(record);
            ++navigationModelCounts[eph.sat.system];
        }
    }
    std::sort(navigationRecords.begin(), navigationRecords.end(), [](const PostNavigationRecord& left,
                                                                     const PostNavigationRecord& right) {
        if (left.hasReference != right.hasReference) return left.hasReference;
        return left.referenceSeconds < right.referenceSeconds;
    });
    QStringList modelSummary;
    for (auto it = navigationModelCounts.constBegin(); it != navigationModelCounts.constEnd(); ++it) {
        modelSummary << QStringLiteral("%1=%2").arg(it.key()).arg(it.value());
    }
    emit logMessage(QStringLiteral("Post-processing NAV models: %1").arg(modelSummary.join(QLatin1Char(' '))));
    QVector<RtcmSsrMessage> sortedSsr = ssr.messages;
    std::sort(sortedSsr.begin(), sortedSsr.end(), [](const RtcmSsrMessage& a, const RtcmSsrMessage& b) {
        return a.gpsWeek == b.gpsWeek ? a.epochTime < b.epochTime : a.gpsWeek < b.gpsWeek;
    });
    QVector<PppVtecModel> sortedVtec = ssr.vtec;
    std::sort(sortedVtec.begin(), sortedVtec.end(), [](const PppVtecModel& a, const PppVtecModel& b) {
        return a.gpsWeek * 604800.0 + a.gpsSow < b.gpsWeek * 604800.0 + b.gpsSow;
    });
    int ssrCursor = 0;
    int vtecCursor = 0;
    int navigationCursor = 0;
    PppJsonWriter jsonWriter;
    PppJsonWriter::Options jsonOptions;
    jsonOptions.enabled = config.jsonEnabled;
    jsonOptions.outputDirectory = config.outputDirectory;
    jsonOptions.reorderBufferMs = config.jsonReorderBufferMs;
    jsonOptions.flushIntervalMs = config.jsonFlushIntervalMs;
    if (!jsonWriter.setOptions(jsonOptions, &error)) { fail(error); return; }
    QFile ztdFile(ztdOutput.filePath(QStringLiteral("ztd.txt")));
    QFile arFile(output.filePath(QStringLiteral("ppp_ar.log")));
    if (!ztdFile.open(QIODevice::WriteOnly | QIODevice::Text) ||
        !arFile.open(QIODevice::WriteOnly | QIODevice::Text)) {
        fail(QStringLiteral("cannot create post-processing ZTD/AR output"));
        return;
    }
    QTextStream ztdOut(&ztdFile);
    QTextStream arOut(&arFile);
    QMap<int, int> gloChannels = obs.glonassChannels;
    for (const RtcmBroadcastEphemeris& eph : nav) {
        if (eph.sat.system == QLatin1Char('R') && !gloChannels.contains(eph.sat.number) &&
            eph.glonassFreqNum != 0) {
            gloChannels[eph.sat.number] = eph.glonassFreqNum;
        }
    }
    const bool phaseBiasAvailable = std::any_of(ssr.messages.cbegin(), ssr.messages.cend(), [](const RtcmSsrMessage& m) {
        return m.kind == RtcmSsrCorrectionKind::PhaseBias && !m.satellites.isEmpty();
    });
    if (options.enablePppAr && !phaseBiasAvailable) {
        emit logMessage(QStringLiteral("PPP-AR unavailable: no usable SSR phase-bias block; continuing with float PPP/ZTD."));
    }
    int processed = 0, accepted = 0, skippedNoObservation = 0, skippedPrecise = 0, rejected = 0;
    QMap<QString, int> rejectionCounts;
    QMap<QString, int> preciseDiagnostics;
    QMap<QString, PppSatelliteStageDiagnostic> satelliteDiagnostics;
    QString outputFailure;
    const std::array<double, 3> receiver = station.valid
        ? std::array<double, 3>{station.aprXyz[0], station.aprXyz[1], station.aprXyz[2]}
        : std::array<double, 3>{options.aprXyz[0], options.aprXyz[1], options.aprXyz[2]};
    for (const RinexEpoch& epoch : obs.epochs) {
        if (_cancelRequested.loadAcquire()) break;
        if (!isSampled(epoch)) continue;
        const double epochSeconds = epoch.gpsWeek * 604800.0 + epoch.gpsSow;
        while (navigationCursor < navigationRecords.size() &&
               navigationRecords.at(navigationCursor).hasReference &&
               navigationRecords.at(navigationCursor).referenceSeconds <= epochSeconds) {
            const PostNavigationRecord& record = navigationRecords.at(navigationCursor++);
            engine.updateBroadcast(record.eph, record.model, QStringLiteral("POST_NAV"),
                                    QDateTime::currentDateTimeUtc());
        }
        while (ssrCursor < sortedSsr.size() &&
               sortedSsr.at(ssrCursor).gpsWeek * 604800.0 + sortedSsr.at(ssrCursor).epochTime <= epochSeconds) {
            const RtcmSsrMessage& message = sortedSsr.at(ssrCursor++);
            if (config.productMode == QLatin1String("precise") &&
                message.kind != RtcmSsrCorrectionKind::CodeBias &&
                message.kind != RtcmSsrCorrectionKind::PhaseBias) continue;
            if (message.kind == RtcmSsrCorrectionKind::CodeBias || message.kind == RtcmSsrCorrectionKind::PhaseBias) engine.beginBiasEpoch(QStringLiteral("POST_SSR"), message);
            engine.updateSsrMessage(QStringLiteral("POST_SSR"), message);
        }
        while (vtecCursor < sortedVtec.size() &&
               sortedVtec.at(vtecCursor).gpsWeek * 604800.0 + sortedVtec.at(vtecCursor).gpsSow <= epoch.gpsWeek * 604800.0 + epoch.gpsSow) {
            engine.updateVtecModel(sortedVtec.at(vtecCursor++));
        }
        const PppStandardizedEpoch standardized = standardizeEpoch(epoch, options, gloChannels);
        if (config.productMode == QLatin1String("precise")) {
            engine.clearPreciseStates();
            int preciseCount = 0;
            for (const PppStandardizedObservation& observation : standardized.observations) {
                const auto it = precise.constFind(observation.satKey);
                if (it == precise.constEnd()) {
                    ++preciseDiagnostics[QStringLiteral("satellite absent from SP3")];
                    continue;
                }
                PreciseSatelliteState state;
                QString preciseDiagnostic;
                if (preciseStateAtTransmitTime(it.value(), epoch, observation, receiver,
                                               state, &preciseDiagnostic)) {
                    engine.updatePreciseState(state);
                    ++preciseCount;
                } else {
                    ++preciseDiagnostics[preciseDiagnostic.isEmpty()
                        ? QStringLiteral("precise state unavailable") : preciseDiagnostic];
                }
            }
            if (preciseCount == 0) ++skippedPrecise;
        }
        if (standardized.observations.isEmpty()) {
            ++skippedNoObservation;
        } else {
            PppSolution solution;
            if (engine.processStandardizedEpoch(standardized, QStringLiteral("POST"), gloChannels, solution)) {
                addSatelliteDiagnostics(satelliteDiagnostics, engine.lastSatelliteDiagnostics());
                ++accepted;
                const QString stationId = !station.station.trimmed().isEmpty() ? station.station :
                    (!obs.markerName.trimmed().isEmpty() ? obs.markerName : QFileInfo(obs.path).baseName());
                const std::array<double, 3> reference = station.valid
                    ? std::array<double, 3>{station.aprXyz[0], station.aprXyz[1], station.aprXyz[2]}
                    : std::array<double, 3>{options.aprXyz[0], options.aprXyz[1], options.aprXyz[2]};
                QString writeError;
                if (!jsonWriter.append(stationId, solution, reference, &writeError)) {
                    outputFailure = QStringLiteral("PPP JSON output failed: %1")
                        .arg(writeError.isEmpty() ? QStringLiteral("unknown writer error") : writeError);
                    break;
                }
                if (solution.valid && std::isfinite(solution.ztd)) {
                    ztdOut << gpsOutputTime(solution).toString(Qt::ISODateWithMs) << ' '
                           << solution.ztd << ' ' << solution.ztdStdev << '\n';
                    if (ztdOut.status() != QTextStream::Ok) {
                        outputFailure = QStringLiteral("ZTD output failed: %1").arg(ztdFile.errorString());
                        break;
                    }
                }
                for (const QString& line : solution.arLogLines) arOut << gpsOutputTime(solution).toString(Qt::ISODateWithMs) << ' ' << line << '\n';
                if (arOut.status() != QTextStream::Ok) {
                    outputFailure = QStringLiteral("PPP-AR log output failed: %1").arg(arFile.errorString());
                    break;
                }
                emit solutionReady(QStringLiteral("POST"), solution);
            } else {
                addSatelliteDiagnostics(satelliteDiagnostics, engine.lastSatelliteDiagnostics());
                ++rejected;
                const QString reason = engine.lastFailureReason();
                const QString category = rejectionCategory(reason);
                const bool firstOfCategory = !rejectionCounts.contains(category);
                ++rejectionCounts[category];
                if (firstOfCategory) {
                    emit logMessage(QStringLiteral("Post-processing epoch rejection [%1]: %2")
                                        .arg(category, reason.isEmpty() ? QStringLiteral("no reason reported") : reason));
                }
            }
        }
        ++processed;
        emit progress(processed, totalSelected, GnssUtils::gpsCalendarDateTime(epoch.gpsWeek, epoch.gpsSow).toString(Qt::ISODate));
    }
    QString closeError;
    if (!jsonWriter.close(&closeError) && outputFailure.isEmpty()) {
        outputFailure = QStringLiteral("PPP JSON finalization failed: %1")
            .arg(closeError.isEmpty() ? QStringLiteral("unknown writer error") : closeError);
    }
    ztdOut.flush();
    arOut.flush();
    if (ztdOut.status() != QTextStream::Ok && outputFailure.isEmpty()) {
        outputFailure = QStringLiteral("ZTD output flush failed: %1").arg(ztdFile.errorString());
    }
    if (arOut.status() != QTextStream::Ok && outputFailure.isEmpty()) {
        outputFailure = QStringLiteral("PPP-AR log flush failed: %1").arg(arFile.errorString());
    }
    if (!ztdFile.flush() && outputFailure.isEmpty()) {
        outputFailure = QStringLiteral("ZTD output flush failed: %1").arg(ztdFile.errorString());
    }
    if (!arFile.flush() && outputFailure.isEmpty()) {
        outputFailure = QStringLiteral("PPP-AR log flush failed: %1").arg(arFile.errorString());
    }
    ztdFile.close();
    arFile.close();
    const bool cancelled = _cancelRequested.loadAcquire() != 0;
    const QString summaryPath = output.filePath(QStringLiteral("manifest.json"));
    QJsonObject manifest;
    manifest[QStringLiteral("status")] = outputFailure.isEmpty()
        ? (cancelled ? QStringLiteral("cancelled") : QStringLiteral("completed"))
        : QStringLiteral("failed");
    if (!outputFailure.isEmpty()) manifest[QStringLiteral("failureReason")] = outputFailure;
    manifest[QStringLiteral("jobName")] = config.jobName;
    manifest[QStringLiteral("observation")] = config.observationPath;
    manifest[QStringLiteral("navigation")] = config.navigationPath;
    manifest[QStringLiteral("sp3")] = config.sp3Path;
    manifest[QStringLiteral("clock")] = config.clockPath;
    manifest[QStringLiteral("ssrFiles")] = QJsonArray::fromStringList(config.ssrPaths);
    manifest[QStringLiteral("productMode")] = config.productMode;
    manifest[QStringLiteral("outputDirectory")] = config.outputDirectory;
    manifest[QStringLiteral("ztdOutputDirectory")] = ztdOutput.absolutePath();
    manifest[QStringLiteral("sampleSec")] = sample;
    manifest[QStringLiteral("jsonEnabled")] = config.jsonEnabled;
    manifest[QStringLiteral("jsonReorderBufferMs")] = config.jsonReorderBufferMs;
    manifest[QStringLiteral("jsonFlushIntervalMs")] = config.jsonFlushIntervalMs;
    manifest[QStringLiteral("pppOptions")] = pppOptionsJson(options);
    manifest[QStringLiteral("troposphereMode")] = options.troposphereMode == PppTroposphereMode::GfsZwdInitialization
        ? QStringLiteral("gfs_zwd_initialization") : QStringLiteral("original");
    manifest[QStringLiteral("gfsZwdMethod")] = options.gfsZwdMethod == GfsZwdMethod::ProfileIntegral
        ? QStringLiteral("profile_integral") : QStringLiteral("askne_nordius");
    manifest[QStringLiteral("gfsPriorStatus")] = tropoPrior.status;
    manifest[QStringLiteral("gfsPriorM")] = tropoPrior.valid ? tropoPrior.zwdM : QJsonValue(QJsonValue::Null);
    manifest[QStringLiteral("gfsPriorReason")] = tropoPrior.reason.isEmpty() ? gfsPriorDiagnostic : tropoPrior.reason;
    manifest[QStringLiteral("coordinateSource")] = coordinateSource;
    manifest[QStringLiteral("observationTimeSystem")] = obs.timeSystem;
    QJsonObject navigationRecordsBySystem;
    for (auto it = navigationRecordCounts.constBegin(); it != navigationRecordCounts.constEnd(); ++it) {
        navigationRecordsBySystem[QString(it.key())] = it.value();
    }
    manifest[QStringLiteral("navigationRecordsBySystem")] = navigationRecordsBySystem;
    manifest[QStringLiteral("inputEpochs")] = obs.epochs.size();
    manifest[QStringLiteral("sampledEpochs")] = totalSelected;
    manifest[QStringLiteral("processedEpochs")] = processed;
    manifest[QStringLiteral("acceptedSolutions")] = accepted;
    manifest[QStringLiteral("rejectedEpochs")] = rejected;
    manifest[QStringLiteral("skippedNoObservation")] = skippedNoObservation;
    manifest[QStringLiteral("skippedPreciseCoverage")] = skippedPrecise;
    QJsonObject preciseDiagnosticJson;
    for (auto it = preciseDiagnostics.constBegin(); it != preciseDiagnostics.constEnd(); ++it) {
        preciseDiagnosticJson[it.key()] = it.value();
    }
    manifest[QStringLiteral("preciseDiagnostics")] = preciseDiagnosticJson;
    QJsonObject rejectionSummary;
    for (auto it = rejectionCounts.constBegin(); it != rejectionCounts.constEnd(); ++it) {
        rejectionSummary[it.key()] = it.value();
    }
    manifest[QStringLiteral("rejectionSummary")] = rejectionSummary;
    manifest[QStringLiteral("satelliteDiagnostics")] = satelliteDiagnosticsJson(satelliteDiagnostics);
    manifest[QStringLiteral("arStatus")] = options.enablePppAr
        ? (phaseBiasAvailable ? QStringLiteral("requested") : QStringLiteral("AR unavailable"))
        : QStringLiteral("disabled");
    manifest[QStringLiteral("cancelled")] = cancelled;
    manifest[QStringLiteral("inputFingerprint")] = inputFingerprint(QStringList() << config.observationPath
        << config.navigationPath << config.sp3Path << config.clockPath << config.ssrPaths);
    auto writeSummaryFile = [](const QString& path, const QByteArray& payload, QString& failure) {
        QFile file(path);
        if (!file.open(QIODevice::WriteOnly | QIODevice::Text) ||
            file.write(payload) != payload.size() || !file.flush()) {
            failure = QStringLiteral("cannot write post-processing summary: %1")
                .arg(file.errorString());
            return false;
        }
        file.close();
        return true;
    };
    QFile manifestFile(summaryPath);
    QString manifestError;
    const QByteArray manifestPayload = QJsonDocument(manifest).toJson(QJsonDocument::Indented);
    if (!manifestFile.open(QIODevice::WriteOnly | QIODevice::Text) ||
        manifestFile.write(manifestPayload) != manifestPayload.size() ||
        !manifestFile.flush()) {
        manifestError = QStringLiteral("cannot write post-processing manifest: %1")
            .arg(manifestFile.errorString());
    }
    manifestFile.close();
    if (manifestError.isEmpty()) {
        writeSummaryFile(output.filePath(QStringLiteral("summary.json")), manifestPayload, manifestError);
    }
    if (!manifestError.isEmpty()) {
        emit failed(manifestError);
        return;
    }
    if (!outputFailure.isEmpty()) {
        emit failed(outputFailure);
        return;
    }
    emit finished(cancelled, processed, accepted, summaryPath);
}
