// Exercises post-processing with recorded WTZR products through a guarded BIE commit.
#include "../postppp.h"
#include "../gnss_ephemeris.h"

#include <QCoreApplication>
#include <QDir>
#include <QFile>
#include <QJsonDocument>
#include <QJsonObject>
#include <QTemporaryDir>

#include <iostream>

#include <algorithm>
#include <memory>

namespace {

bool check(bool value, const char* message) {
    if (!value) std::cerr << "postppp WTZR integration failure: " << message << '\n';
    return value;
}

bool hasObservedSatellite(const QJsonObject& diagnostics, QChar system) {
    for (auto it = diagnostics.constBegin(); it != diagnostics.constEnd(); ++it) {
        if (it.key().isEmpty() || it.key().at(0) != system) continue;
        if (it.value().toObject().value(QStringLiteral("observationAvailable")).toInt() > 0) {
            return true;
        }
    }
    return false;
}

bool hasDiagnosticField(const QJsonObject& diagnostics, QChar system, const QString& field) {
    for (auto it = diagnostics.constBegin(); it != diagnostics.constEnd(); ++it) {
        if (it.key().isEmpty() || it.key().at(0) != system) continue;
        if (it.value().toObject().contains(field)) return true;
    }
    return false;
}

bool hasOnlyBareSatelliteKeys(const QJsonObject& diagnostics, QChar system) {
    for (auto it = diagnostics.constBegin(); it != diagnostics.constEnd(); ++it) {
        if (it.key().isEmpty() || it.key().at(0) != system) continue;
        if (it.key().contains(QLatin1Char(':'))) return false;
    }
    return true;
}

bool hasStageCount(const QJsonObject& diagnostics, QChar system, const QString& field) {
    for (auto it = diagnostics.constBegin(); it != diagnostics.constEnd(); ++it) {
        if (it.key().isEmpty() || it.key().at(0) != system) continue;
        if (it.value().toObject().value(field).toInt() > 0) return true;
    }
    return false;
}

void printDiagnostics(const QJsonObject& diagnostics, QChar system) {
    int observations = 0;
    int broadcasts = 0;
    int ssr = 0;
    int iod = 0;
    int residuals = 0;
    for (auto it = diagnostics.constBegin(); it != diagnostics.constEnd(); ++it) {
        if (it.key().isEmpty() || it.key().at(0) != system) continue;
        const QJsonObject stage = it.value().toObject();
        observations += stage.value(QStringLiteral("observationAvailable")).toInt();
        broadcasts += stage.value(QStringLiteral("broadcastMatched")).toInt();
        ssr += stage.value(QStringLiteral("ssrMatched")).toInt();
        iod += stage.value(QStringLiteral("iodRejected")).toInt();
        residuals += stage.value(QStringLiteral("residualRejected")).toInt();
    }
    std::cout << system.toLatin1() << " diagnostics: obs=" << observations
              << " nav=" << broadcasts << " ssr=" << ssr
              << " iod=" << iod << " residual=" << residuals << '\n';
}

} // namespace

int main(int argc, char* argv[]) {
    QCoreApplication app(argc, argv);
    const QDir root(QDir::currentPath());
    const QString obsPath = root.filePath(QStringLiteral("output/data/WTZR00DEU_S_20260650000_01D_15S_MO.rnx"));
    const QString navPath = root.filePath(QStringLiteral("output/data/BRDC00WRD_S_20260650000_01D_MN.rnx"));
    const QString ssrPath = root.filePath(QStringLiteral("output/data/SSRA00CNE0_S_20260650000_01D_MC.ssr"));
    if (!check(QFile::exists(obsPath) && QFile::exists(navPath) && QFile::exists(ssrPath),
               "WTZR input products are unavailable")) return 2;

    RinexObservationFile parsedObs;
    QVector<RtcmBroadcastEphemeris> parsedNav;
    QString parseError;
    if (!check(RinexReader::readObservation(obsPath, parsedObs, &parseError),
               "WTZR OBS cannot be parsed")) return 1;
    if (!check(RinexReader::readNavigation(navPath, parsedNav, &parseError),
               "WTZR NAV cannot be parsed")) return 1;
    const auto e02 = std::find_if(parsedNav.cbegin(), parsedNav.cend(), [](const RtcmBroadcastEphemeris& eph) {
        return eph.sat.system == QLatin1Char('E') && eph.sat.number == 2;
    });
    if (!check(e02 != parsedNav.cend(), "WTZR Galileo E02 NAV record is missing")) return 1;
    std::shared_ptr<GnssEphemeris> e02Model;
    if (!check(GnssEphemeris::fromRtcm(*e02, e02Model), "WTZR Galileo E02 model conversion")) return 1;
    GnssEphemeris::State e02State;
    const RinexEpoch& firstEpoch = parsedObs.epochs.first();
    const bool e02StateOk = e02Model->computeState(firstEpoch.gpsWeek, firstEpoch.gpsSow, e02State);
    std::cout << "E02 NAV week=" << e02->week << " toc=" << e02->tocSec
              << " toe=" << e02->toeSec << " state=" << (e02StateOk ? "ok" : "fail") << '\n';
    if (!check(e02StateOk, "WTZR Galileo E02 broadcast state cannot be computed")) return 1;
    for (const QChar system : {QLatin1Char('G'), QLatin1Char('R'), QLatin1Char('C')}) {
        const auto record = std::find_if(parsedNav.cbegin(), parsedNav.cend(), [system](const RtcmBroadcastEphemeris& eph) {
            return eph.sat.system == system;
        });
        if (!check(record != parsedNav.cend(), qPrintable(QStringLiteral("WTZR %1 NAV record is missing").arg(system)))) return 1;
        std::shared_ptr<GnssEphemeris> model;
        if (!check(GnssEphemeris::fromRtcm(*record, model),
                   qPrintable(QStringLiteral("WTZR %1 NAV model conversion").arg(system)))) return 1;
        GnssEphemeris::State state;
        if (!check(model->computeState(firstEpoch.gpsWeek, firstEpoch.gpsSow, state),
                   qPrintable(QStringLiteral("WTZR %1 broadcast state cannot be computed").arg(system)))) return 1;
        if (system == QLatin1Char('R')) {
            if (!check(record->glonassFreqNum != 0, "WTZR GLONASS frequency channel is missing")) return 1;
        }
    }

    QTemporaryDir temporary;
    if (!check(temporary.isValid(), "temporary output directory")) return 1;

    PostPppJobConfig config;
    config.jobName = QStringLiteral("WTZR-integration");
    config.observationPath = obsPath;
    config.navigationPath = navPath;
    config.ssrPaths << ssrPath;
    config.outputDirectory = temporary.filePath(QStringLiteral("ppp_json"));
    config.ztdOutputDirectory = temporary.filePath(QStringLiteral("ztd"));
    config.productMode = QStringLiteral("broadcast");
    config.sampleSec = 5;
    config.options.useGps = true;
    config.options.useGlo = false;
    config.options.useGal = true;
    config.options.useBds = true;
    config.options.enablePppAr = true;
    config.options.arAlgorithm = QStringLiteral("BIE");
    config.options.arBiePersistConstraints = true;
    config.options.arBiePersistConfirmEpochs = 2;
    // Disable only the individual BIE screening thresholds in this integration
    // fixture so the persistent-batch state machine is exercised on WTZR.
    config.options.arBieMaxFrac = 0.0;
    config.options.arBieMaxSig = 0.0;
    // The production U/ZTD transition gates are separately exercised with
    // their defaults.  Disable them here to require an actual posterior
    // commit after the two-epoch arc confirmation.
    config.options.arBiePersistMaxVerticalM = 0.0;
    config.options.arBiePersistMaxZtdM = 0.0;
    config.options.logBiasMatchDetails = true;
    config.options.lcGps = QStringLiteral("Pi&Li");
    config.options.lcGal = QStringLiteral("Pi&Li");
    config.options.lcBds = QStringLiteral("Pi&Li");
    // Separate E1/E5 rules are intentionally retained to verify aggregation.
    config.options.signalPriorities = QStringLiteral("G:12&CWPSLX R:12&CP E:1&CBX E:5&QIX C:26&IQX");
    config.station.valid = true;
    config.station.station = QStringLiteral("WTZR00DEU");
    config.station.aprXyz[0] = 4075580.3925;
    config.station.aprXyz[1] = 931854.14;
    config.station.aprXyz[2] = 4801568.4674;

    QString failure;
    QString manifestPath;
    int accepted = 0;
    bool sawBiePersistenceCommit = false;
    QString persistenceDecision;
    bool cancelled = false;
    int solutionCount = 0;
    PostPppService service;
    QObject::connect(&service, &PostPppService::failed, [&failure](const QString& reason) {
        failure = reason;
    });
    QObject::connect(&service, &PostPppService::logMessage, [](const QString& message) {
        if (message.startsWith(QStringLiteral("Post-processing NAV"))) {
            std::cout << message.toStdString() << '\n';
        }
    });
    QObject::connect(&service, &PostPppService::finished,
                     [&manifestPath, &accepted, &cancelled](bool wasCancelled, int, int acceptedCount, const QString& path) {
        accepted = acceptedCount;
        cancelled = wasCancelled;
        manifestPath = path;
    });
    QObject::connect(&service, &PostPppService::solutionReady,
                     [&service, &sawBiePersistenceCommit, &persistenceDecision, &solutionCount](const QString&, const PppSolution& solution) {
        ++solutionCount;
        for (const QString& line : solution.arLogLines) {
            if (line.startsWith(QStringLiteral("BIE PERSISTENCE status=committed"))) {
                sawBiePersistenceCommit = true;
                persistenceDecision = line;
                service.cancel();
                return;
            }
            if (line.startsWith(QStringLiteral("BIE PERSISTENCE status=reject"))) {
                persistenceDecision = line;
                service.cancel();
                return;
            }
        }
        // This real-product integration test needs only the first guarded
        // decision.  Bound a data/product regression that never reaches one.
        if (solutionCount >= 60) service.cancel();
    });
    service.run(config);
    if (!check(failure.isEmpty(), qPrintable(failure))) return 1;
    if (!check(accepted > 0 && QFile::exists(manifestPath), "post-processing did not produce solutions")) return 1;
    if (!check(cancelled, "integration test did not stop after its decision window")) return 1;
    if (!check(sawBiePersistenceCommit,
               "persistent BIE never reached a guarded posterior commit")) return 1;
    std::cout << "persistence decision: " << persistenceDecision.toStdString() << '\n';

    QFile manifestFile(manifestPath);
    if (!check(manifestFile.open(QIODevice::ReadOnly), "cannot read post-processing manifest")) return 1;
    const QJsonObject manifest = QJsonDocument::fromJson(manifestFile.readAll()).object();
    if (!check(manifest.value(QStringLiteral("cancelled")).toBool(),
               "cancelled persistence run was not recorded in the manifest")) return 1;
    const QJsonObject diagnostics = manifest.value(QStringLiteral("satelliteDiagnostics")).toObject();
    if (!check(!diagnostics.isEmpty(), "real-product run did not emit satellite diagnostics")) return 1;
    const QStringList fields = {
        QStringLiteral("observationAvailable"), QStringLiteral("broadcastMatched"),
        QStringLiteral("ssrMatched"), QStringLiteral("iodRejected"),
        QStringLiteral("residualRejected"), QStringLiteral("acceptedAfterResidual")
    };
    for (const QChar system : {QLatin1Char('E'), QLatin1Char('C')}) {
        // The test intentionally stops at its first persistent BIE decision.
        // A constellation absent in that prefix is covered by the full-run
        // post-processing regression, not required by this bounded test.
        if (!hasObservedSatellite(diagnostics, system)) continue;
        if (system == QLatin1Char('E') &&
            !check(hasStageCount(diagnostics, system, QStringLiteral("broadcastMatched")),
                   "Galileo observations did not match any broadcast ephemeris")) return 1;
        if (!check(hasOnlyBareSatelliteKeys(diagnostics, system),
                   qPrintable(QStringLiteral("diagnostic keys retain navigation suffix for %1").arg(system)))) return 1;
        for (const QString& field : fields) {
            if (!check(hasDiagnosticField(diagnostics, system, field),
                       qPrintable(QStringLiteral("missing %1 diagnostic field for %2").arg(field, system)))) {
                return 1;
            }
        }
        printDiagnostics(diagnostics, system);
    }

    std::cout << "postppp WTZR integration passed" << std::endl;
    return 0;
}
