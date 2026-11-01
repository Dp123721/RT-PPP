// Checks offline product parsing, SSR round trips, and generated RINEX/SP3 output.
#include "postppp.h"
#include "telemetrywriter.h"
#include "gnss_signals.h"

#include <QCoreApplication>
#include <QDateTime>
#include <QDir>
#include <QDirIterator>
#include <QEvent>
#include <QEventLoop>
#include <QFile>
#include <QJsonDocument>
#include <QJsonObject>
#include <QJsonParseError>
#include <QPointer>
#include <QTemporaryDir>
#include <QTextStream>
#include <QThread>
#include <QTimer>

#include <cmath>
#include <iostream>

namespace {

bool check(bool value, const char* message) {
    if (!value) std::cerr << "postppp smoke failure: " << message << '\n';
    return value;
}

QString headerLine(const QString& value, const QString& label) {
    return value.leftJustified(60, QLatin1Char(' ')) + label + QLatin1Char('\n');
}

QString timeSystemLine(const QString& timeSystem) {
    QString line(60, QLatin1Char(' '));
    line.replace(0, 43, QStringLiteral("  2024     1     1     0     0    0.0000000"));
    line.replace(48, 3, timeSystem);
    return line + QStringLiteral("TIME OF FIRST OBS\n");
}

QString obsField(double value, int lli);
QString obsTypesLine();

QString oneSatelliteObservation(const QString& epoch) {
    return epoch + QStringLiteral("  0  1\nG01") +
           obsField(22000000.0, 0) + obsField(110000000.0, 0) +
           obsField(22000010.0, 0) + obsField(85000000.0, 0) + QLatin1Char('\n');
}

QString makeTimeSystemObservation(const QString& timeSystem) {
    QString result = headerLine(QStringLiteral("     3.04           OBSERVATION DATA    M"),
                                "RINEX VERSION / TYPE");
    result += timeSystemLine(timeSystem);
    result += obsTypesLine();
    result += headerLine(QString(), "END OF HEADER");
    result += oneSatelliteObservation(QStringLiteral("> 2024 01 01 00 00 00.0000000"));
    return result;
}

QString obsTypesLine() {
    QString line(60, QLatin1Char(' '));
    line[0] = QLatin1Char('G');
    line.replace(3, 3, QStringLiteral("  4"));
    line.replace(7, 3, QStringLiteral("C1C"));
    line.replace(11, 3, QStringLiteral("L1C"));
    line.replace(15, 3, QStringLiteral("C2W"));
    line.replace(19, 3, QStringLiteral("L2W"));
    return line + QStringLiteral("SYS / # / OBS TYPES\n");
}

QString obsTypesLineWithContinuation() {
    QString line(60, QLatin1Char(' '));
    line[0] = QLatin1Char('G');
    line.replace(3, 3, QStringLiteral("  6"));
    line.replace(7, 3, QStringLiteral("C1C"));
    line.replace(11, 3, QStringLiteral("L1C"));
    line.replace(15, 3, QStringLiteral("S1C"));
    line.replace(19, 3, QStringLiteral("C2W"));
    line.replace(23, 3, QStringLiteral("L2W"));
    line.replace(27, 3, QStringLiteral("S2W"));
    return line + QStringLiteral("SYS / # / OBS TYPES\n");
}

QString obsField(double value, int lli) {
    return QString("%1%2 ").arg(value, 14, 'f', 3).arg(lli);
}

QString navField(double value) {
    return QString("%1").arg(value, 19, 'E', 12);
}

QString navRecord(const QString& prn, int lineCount, int week = 0) {
    QString record = QStringLiteral("%1 2024 01 01 00 00 00").arg(prn);
    record += navField(0.0) + navField(0.0) + navField(0.0) + QLatin1Char('\n');
    for (int row = 1; row < lineCount; ++row) {
        const double thirdField = row == 5 ? static_cast<double>(week) : 0.0;
        record += QStringLiteral("    ") + navField(0.0);
        if (row == lineCount - 1) {
            record += QString(57, QLatin1Char(' '));
        } else {
            record += navField(0.0) + navField(thirdField) + navField(0.0);
        }
        record += QLatin1Char('\n');
    }
    return record;
}

bool writeText(const QString& path, const QString& text) {
    QFile file(path);
    if (!file.open(QIODevice::WriteOnly | QIODevice::Text)) return false;
    QTextStream stream(&file);
    stream << text;
    return true;
}

} // namespace

int main(int argc, char* argv[]) {
    QCoreApplication app(argc, argv);
    QTemporaryDir temp;
    if (!check(temp.isValid(), "temporary directory")) return 1;

    const QString obsPath = temp.filePath(QStringLiteral("test.obs"));
    QString obs = headerLine(QStringLiteral("     3.04           OBSERVATION DATA    M"), "RINEX VERSION / TYPE");
    obs += headerLine(QStringLiteral("TEST"), "MARKER NAME");
    obs += headerLine(QStringLiteral("3875000.0 332000.0 5029000.0"), "APPROX POSITION XYZ");
    obs += obsTypesLine();
    obs += headerLine(QString(), "END OF HEADER");
    obs += QStringLiteral("> 2024 01 01 00 00 00.0000000  0  1\nG01") +
           obsField(22000000.0, 0) + obsField(110000000.0, 1) +
           obsField(22000010.0, 0) + obsField(85000000.0, 0) + QLatin1Char('\n');
    if (!check(writeText(obsPath, obs), "write obs")) return 1;
    RinexObservationFile readObs;
    QString error;
    if (!check(RinexReader::readObservation(obsPath, readObs, &error), "read RINEX OBS")) return 1;
    if (!check(readObs.markerName == QLatin1String("TEST") && readObs.epochs.size() == 1,
               "OBS header and epoch")) return 1;
    if (!check(readObs.epochs.first().observations.first().lli.value(QStringLiteral("L1C")) == 1,
               "OBS LLI")) return 1;
    if (!check(readObs.hasApproximatePosition, "plausible OBS approximate coordinate")) return 1;

    const QStringList timeSystems = QStringList() << QStringLiteral("GPS")
                                                   << QStringLiteral("GLO")
                                                   << QStringLiteral("BDT")
                                                   << QStringLiteral("UTC");
    QMap<QString, double> timeSystemSow;
    for (const QString& timeSystem : timeSystems) {
        const QString path = temp.filePath(QStringLiteral("time-%1.obs").arg(timeSystem));
        if (!check(writeText(path, makeTimeSystemObservation(timeSystem)),
                   "write RINEX time-system OBS")) return 1;
        RinexObservationFile parsed;
        if (!check(RinexReader::readObservation(path, parsed, &error),
                   "read RINEX time-system OBS")) return 1;
        if (!check(parsed.timeSystem == timeSystem && parsed.epochs.size() == 1,
                   "RINEX time-system header")) return 1;
        timeSystemSow[timeSystem] = parsed.epochs.first().gpsSow;
    }
    const double gpsSow = timeSystemSow.value(QStringLiteral("GPS"));
    if (!check(std::fabs(timeSystemSow.value(QStringLiteral("UTC")) - (gpsSow + 18.0)) < 1.0e-6,
               "UTC to GPS conversion")) return 1;
    if (!check(std::fabs(timeSystemSow.value(QStringLiteral("BDT")) - (gpsSow + 14.0)) < 1.0e-6,
               "BDT to GPS conversion")) return 1;
    if (!check(std::fabs(timeSystemSow.value(QStringLiteral("GLO")) - (gpsSow - 10782.0)) < 1.0e-6,
               "GLO to GPS conversion")) return 1;

    const QString placeholderObsPath = temp.filePath(QStringLiteral("placeholder-coordinate.obs"));
    QString placeholderObs = headerLine(QStringLiteral("     3.04           OBSERVATION DATA    M"), "RINEX VERSION / TYPE");
    placeholderObs += headerLine(QStringLiteral("0.0 0.0 0.0"), "APPROX POSITION XYZ");
    placeholderObs += obsTypesLine();
    placeholderObs += headerLine(QString(), "END OF HEADER");
    placeholderObs += QStringLiteral("> 2024 01 01 00 00 00.0000000  0  1\nG01") +
        obsField(22000000.0, 0) + obsField(110000000.0, 1) +
        obsField(22000010.0, 0) + obsField(85000000.0, 0) + QLatin1Char('\n');
    if (!check(writeText(placeholderObsPath, placeholderObs), "write placeholder-coordinate OBS")) return 1;
    RinexObservationFile placeholderObsFile;
    if (!check(RinexReader::readObservation(placeholderObsPath, placeholderObsFile, &error),
               "read placeholder-coordinate OBS")) return 1;
    if (!check(!placeholderObsFile.hasApproximatePosition,
               "zero OBS approximate coordinate is unavailable")) return 1;

    const QString continuationObsPath = temp.filePath(QStringLiteral("continued-observation.obs"));
    QString continuationObs = headerLine(QStringLiteral("     3.04           OBSERVATION DATA    M"), "RINEX VERSION / TYPE");
    continuationObs += obsTypesLineWithContinuation();
    continuationObs += headerLine(QString(), "END OF HEADER");
    continuationObs += QStringLiteral("> 2024 01 01 00 00 00.0000000  0  1\nG01") +
        obsField(1.0, 0) + obsField(2.0, 0) + obsField(3.0, 0) +
        obsField(4.0, 0) + obsField(5.0, 0) + QLatin1Char('\n') +
        QStringLiteral("   ") + obsField(6.0, 0) + QLatin1Char('\n');
    if (!check(writeText(continuationObsPath, continuationObs), "write continued-observation OBS")) return 1;
    RinexObservationFile continuationObsFile;
    if (!check(RinexReader::readObservation(continuationObsPath, continuationObsFile, &error),
               "read continued-observation OBS")) return 1;
    if (!check(continuationObsFile.epochs.first().observations.first().values.value(QStringLiteral("S2W")) == 6.0,
               "OBS continuation field")) return 1;

    const QString longLineObsPath = temp.filePath(QStringLiteral("long-observation.obs"));
    QString longLineObs = headerLine(QStringLiteral("     3.04           OBSERVATION DATA    M"), "RINEX VERSION / TYPE");
    longLineObs += obsTypesLineWithContinuation();
    longLineObs += headerLine(QString(), "END OF HEADER");
    longLineObs += QStringLiteral("> 2024 01 01 00 00 00.0000000  0  1\nG01") +
        obsField(1.0, 0) + obsField(2.0, 0) + obsField(3.0, 0) +
        obsField(4.0, 0) + obsField(5.0, 0) + obsField(6.0, 0) + QLatin1Char('\n');
    if (!check(writeText(longLineObsPath, longLineObs), "write long-observation OBS")) return 1;
    RinexObservationFile longLineObsFile;
    if (!check(RinexReader::readObservation(longLineObsPath, longLineObsFile, &error),
               "read long-observation OBS")) return 1;
    if (!check(longLineObsFile.epochs.first().observations.first().values.value(QStringLiteral("S2W")) == 6.0,
               "OBS long-line field")) return 1;

    const QString navPath = temp.filePath(QStringLiteral("test.nav"));
    QString nav = headerLine(QStringLiteral("     3.04           NAVIGATION DATA     M"), "RINEX VERSION / TYPE");
    nav += headerLine(QString(), "END OF HEADER");
    nav += navRecord(QStringLiteral("S01"), 4);
    nav += navRecord(QStringLiteral("G01"), 8);
    nav += navRecord(QStringLiteral("E01"), 8, 2408);
    nav += navRecord(QStringLiteral("R01"), 4);
    if (!check(writeText(navPath, nav), "write nav")) return 1;
    QVector<RtcmBroadcastEphemeris> navRecords;
    if (!check(RinexReader::readNavigation(navPath, navRecords, &error) && navRecords.size() == 3,
               "read RINEX NAV")) return 1;
    const auto galileo = std::find_if(navRecords.cbegin(), navRecords.cend(), [](const RtcmBroadcastEphemeris& eph) {
        return eph.sat.system == QLatin1Char('E');
    });
    if (!check(galileo != navRecords.cend() && galileo->week == 1384,
               "RINEX Galileo week conversion")) return 1;
    const auto glonass = std::find_if(navRecords.cbegin(), navRecords.cend(), [](const RtcmBroadcastEphemeris& eph) {
        return eph.sat.system == QLatin1Char('R');
    });
    if (!check(glonass != navRecords.cend() &&
               std::fabs(glonass->tocSec - timeSystemSow.value(QStringLiteral("GLO"))) < 1.0e-6,
               "RINEX GLONASS UTC(SU)+3 conversion")) return 1;

    const QString ssrPath = temp.filePath(QStringLiteral("test.ssr"));
    const QString ssr = QStringLiteral("> CODE_BIAS 2024 01 01 00 00 00.0 2 1 TEST\n"
                                       "G01 1 1C -2.0000\n"
                                       "> PHASE_BIAS 2024 01 01 00 00 00.0 2 1 TEST\n"
                                       "0 1\n"
                                       "G01 0.0 0.0 1 1C -0.1000 1 1 0\n");
    if (!check(writeText(ssrPath, ssr), "write SSR")) return 1;
    SsrFileReader::Result ssrResult;
    if (!check(SsrFileReader::read(QStringList() << ssrPath, ssrResult, &error), "read SSR")) return 1;
    if (!check(ssrResult.messages.size() == 2 && ssrResult.messages.first().epochTimeIsGps &&
               ssrResult.messages.first().gpsWeek > 0, "SSR GPS epoch")) return 1;
    SsrFileReader::Result duplicateSsr;
    if (!check(SsrFileReader::read(QStringList() << ssrPath << ssrPath, duplicateSsr, &error) &&
               duplicateSsr.messages.size() == 2, "SSR duplicate merge")) return 1;
    const QString conflictPath = temp.filePath(QStringLiteral("conflict.ssr"));
    if (!check(writeText(conflictPath, QStringLiteral("> CODE_BIAS 2024 01 01 00 00 00.0 2 1 TEST\n"
                                                     "G01 1 1C -2.1000\n")), "write conflicting SSR")) return 1;
    SsrFileReader::Result conflictSsr;
    if (!check(!SsrFileReader::read(QStringList() << ssrPath << conflictPath, conflictSsr, &error),
               "SSR conflict rejection")) return 1;

    const QString generatedSsrDirectory = temp.filePath(QStringLiteral("generated-ssr"));
    TelemetryWriter telemetryWriter;
    telemetryWriter.setOutputDirectory(generatedSsrDirectory);
    RtcmSsrMessage generatedMessage;
    generatedMessage.kind = RtcmSsrCorrectionKind::OrbitClock;
    generatedMessage.system = QLatin1Char('G');
    generatedMessage.messageType = 1060;
    generatedMessage.updateIntervalIndex = 2;
    generatedMessage.providerId = 74;
    generatedMessage.solutionId = 1;
    RtcmSsrSatelliteData generatedSatellite;
    generatedSatellite.sat.system = QLatin1Char('G');
    generatedSatellite.sat.number = 1;
    generatedSatellite.iod = 7;
    generatedSatellite.hasOrbit = true;
    generatedSatellite.deltaRadial = 0.125;
    generatedSatellite.hasClock = true;
    generatedSatellite.deltaC0 = -0.25;
    const QDateTime generatedEpoch(QDate(2024, 1, 1), QTime(0, 0), Qt::UTC);
    telemetryWriter.writeSsr(QStringLiteral("test-source"), generatedEpoch, generatedEpoch,
                             generatedMessage, generatedSatellite, true, QString());
    generatedSatellite.sat.number = 2;
    telemetryWriter.writeSsr(QStringLiteral("test-source"), generatedEpoch, generatedEpoch,
                             generatedMessage, generatedSatellite, true, QString());
    generatedMessage.kind = RtcmSsrCorrectionKind::CodeBias;
    generatedMessage.messageType = 1059;
    RtcmSsrCodeBiasEntry codeBias;
    codeBias.signalCode = rinexType2chToSsrSignalCode(QLatin1Char('G'), QStringLiteral("1C"));
    codeBias.biasMeters = 0.25;
    generatedSatellite.codeBiases.push_back(codeBias);
    telemetryWriter.writeSsr(QStringLiteral("test-source"), generatedEpoch, generatedEpoch,
                             generatedMessage, generatedSatellite, true, QString());
    generatedMessage.kind = RtcmSsrCorrectionKind::PhaseBias;
    generatedMessage.messageType = 1265;
    generatedMessage.dispersiveBiasConsistency = 0;
    generatedMessage.mwConsistency = 1;
    RtcmSsrPhaseBiasEntry phaseBias;
    phaseBias.signalCode = codeBias.signalCode;
    phaseBias.biasMeters = -0.125;
    generatedSatellite.phaseBiases.push_back(phaseBias);
    telemetryWriter.writeSsr(QStringLiteral("test-source"), generatedEpoch, generatedEpoch,
                             generatedMessage, generatedSatellite, true, QString());
    RtcmVtecMessage generatedVtec;
    generatedVtec.updateIntervalIndex = 2;
    generatedVtec.providerId = 74;
    generatedVtec.solutionId = 1;
    RtcmVtecLayer generatedLayer;
    generatedLayer.heightMeters = 450000.0;
    generatedLayer.cosine = {{1.25}};
    generatedLayer.sine = {{-0.5}};
    generatedVtec.layers.push_back(generatedLayer);
    telemetryWriter.writeVtec(QStringLiteral("test-source"), generatedEpoch, generatedVtec);
    telemetryWriter.close();
    QStringList generatedPaths;
    QDirIterator generatedIterator(generatedSsrDirectory, QStringList() << QStringLiteral("*.ssr"),
                                   QDir::Files, QDirIterator::Subdirectories);
    while (generatedIterator.hasNext()) generatedPaths << generatedIterator.next();
    generatedPaths.sort();
    if (!check(generatedPaths.size() == 2, "writer did not split MC and ION product files")) return 1;
    QString generatedMcText;
    for (const QString& path : generatedPaths) {
        if (!path.endsWith(QStringLiteral("_MC.ssr"))) continue;
        QFile generatedMcFile(path);
        if (generatedMcFile.open(QIODevice::ReadOnly | QIODevice::Text)) {
            generatedMcText = QString::fromUtf8(generatedMcFile.readAll());
        }
        break;
    }
    if (!check(generatedMcText.contains(QStringLiteral("G01           7     0.1250")),
               "generated ORBIT fields do not follow the fixed-width layout")) return 1;
    if (!check(generatedMcText.contains(QStringLiteral("G02    1   1C     0.2500")),
               "generated CODE_BIAS spacing does not match the fixed-width layout")) return 1;
    if (!check(generatedMcText.contains(QStringLiteral(" 0   1\nG02   0.00000000   0.00000000    1   1C    -0.1250   0   0   0")),
               "generated PHASE_BIAS line layout does not match the fixed-width layout")) return 1;
    SsrFileReader::Result generatedResult;
    if (!check(SsrFileReader::read(generatedPaths, generatedResult, &error),
               "SSR reader cannot read generated SSR")) return 1;
    if (!check(generatedResult.messages.size() == 4 && generatedResult.vtec.size() == 1,
               "generated SSR blocks did not round-trip through the SSR reader")) return 1;
    if (!check(generatedResult.messages.first().satellites.size() == 2 &&
               std::fabs(generatedResult.messages.first().satellites.first().deltaRadial - 0.125) < 1.0e-9,
               "generated orbit correction did not round-trip")) return 1;
    const auto generatedCode = std::find_if(generatedResult.messages.cbegin(), generatedResult.messages.cend(),
        [](const RtcmSsrMessage& m) { return m.kind == RtcmSsrCorrectionKind::CodeBias; });
    const auto generatedPhase = std::find_if(generatedResult.messages.cbegin(), generatedResult.messages.cend(),
        [](const RtcmSsrMessage& m) { return m.kind == RtcmSsrCorrectionKind::PhaseBias; });
    if (!check(generatedCode != generatedResult.messages.cend() &&
               generatedCode->satellites.first().codeBiases.size() == 1 &&
               std::fabs(generatedCode->satellites.first().codeBiases.first().biasMeters - 0.25) < 1.0e-9 &&
               generatedPhase != generatedResult.messages.cend() &&
               generatedPhase->satellites.first().phaseBiases.size() == 1 &&
               std::fabs(generatedPhase->satellites.first().phaseBiases.first().biasMeters + 0.125) < 1.0e-9,
               "generated SSR biases did not round-trip")) return 1;
    if (!check(std::fabs(generatedResult.vtec.first().layers.first().cosine.first().first() - 1.25) < 1.0e-9 &&
               std::fabs(generatedResult.vtec.first().layers.first().sine.first().first() + 0.5) < 1.0e-9,
               "generated VTEC coefficients did not round-trip")) return 1;

    QString serviceFailure;
    PostPppService service;
    QObject::connect(&service, &PostPppService::failed, [&serviceFailure](const QString& reason) {
        serviceFailure = reason;
    });
    PostPppJobConfig mixedProducts;
    mixedProducts.productMode = QStringLiteral("precise");
    mixedProducts.navigationPath = QStringLiteral("nav.rnx");
    mixedProducts.ssrPaths << QStringLiteral("correction.ssr");
    mixedProducts.outputDirectory = temp.filePath(QStringLiteral("failed-mixed-job"));
    service.run(mixedProducts);
    if (!check(serviceFailure.contains(QStringLiteral("mutually exclusive")),
               "SSR and SP3+CLK mutual exclusion")) return 1;
    const QString failedManifestPath = QDir(mixedProducts.outputDirectory).filePath(QStringLiteral("manifest.json"));
    QFile failedManifestFile(failedManifestPath);
    if (!check(failedManifestFile.open(QIODevice::ReadOnly),
               "preflight failure manifest was not created")) return 1;
    QJsonParseError failedManifestError;
    const QJsonDocument failedManifest = QJsonDocument::fromJson(failedManifestFile.readAll(), &failedManifestError);
    if (!check(failedManifestError.error == QJsonParseError::NoError &&
               failedManifest.object().value(QStringLiteral("status")).toString() == QLatin1String("failed"),
               "preflight failure manifest is invalid")) return 1;
    serviceFailure.clear();
    PostPppJobConfig missingSsr;
    missingSsr.productMode = QStringLiteral("broadcast");
    missingSsr.navigationPath = QStringLiteral("nav.rnx");
    service.run(missingSsr);
    if (!check(serviceFailure.contains(QStringLiteral("requires SSR")),
               "broadcast SSR requirement")) return 1;

    QPointer<QThread> postThread(new QThread());
    QPointer<PostPppService> postService(new PostPppService());
    postService->moveToThread(postThread);
    QEventLoop postLoop;
    bool asyncFailure = false;
    QObject::connect(postThread, &QThread::finished, postService, &QObject::deleteLater);
    QObject::connect(postThread, &QThread::finished, postThread, &QObject::deleteLater);
    QObject::connect(postThread, &QThread::finished, &postLoop, &QEventLoop::quit);
    QObject::connect(postService, &PostPppService::failed, &app, [&](const QString&) {
        asyncFailure = true;
        postThread->quit();
    });
    postThread->start();
    QMetaObject::invokeMethod(postService, [service = postService.data(), mixedProducts]() {
        service->run(mixedProducts);
    }, Qt::QueuedConnection);
    QTimer::singleShot(5000, &postLoop, &QEventLoop::quit);
    postLoop.exec();
    if (!check(asyncFailure, "asynchronous post-processing failure was not delivered")) return 1;
    if (postThread && postThread->isRunning()) {
        postThread->quit();
        postThread->wait();
        return check(false, "post-processing thread did not stop");
    }
    QCoreApplication::sendPostedEvents(nullptr, QEvent::DeferredDelete);
    if (!check(postThread.isNull(), "post-processing thread was not deleted after finishing")) return 1;

    const QString sp3Path = temp.filePath(QStringLiteral("test.sp3"));
    const QString clkPath = temp.filePath(QStringLiteral("test.clk"));
    QString sp3;
    QString clk;
    const QDateTime start(QDate(2024, 1, 1), QTime(0, 0), Qt::UTC);
    for (int i = 0; i < 9; ++i) {
        const QDateTime time = start.addSecs(i * 900);
        sp3 += QStringLiteral("* %1\n").arg(time.toString(QStringLiteral("yyyy MM dd HH mm ss")));
        sp3 += QStringLiteral("PG01 %1 %2 %3 0.0\n")
            .arg(20000.0 + i).arg(21000.0 + i).arg(22000.0 + i);
    }
    // CLK products commonly have a denser cadence than SP3. Keep the two
    // products independent so interpolation must not require equal epochs.
    for (int i = 0; i <= 240; ++i) {
        const QDateTime time = start.addSecs(i * 30);
        clk += QStringLiteral("AS G01 %1 1 %2\n")
            .arg(time.toString(QStringLiteral("yyyy MM dd HH mm ss")))
            .arg(1.0e-6 + i * 1.0e-10, 0, 'f', 10);
    }
    if (!check(writeText(sp3Path, sp3) && writeText(clkPath, clk), "write precise products")) return 1;
    QMap<QString, QVector<PreciseSatelliteState>> precise;
    if (!check(PreciseProductReader::readSp3(sp3Path, precise, &error) &&
               PreciseProductReader::readClock(clkPath, precise, &error), "read SP3 and CLK")) return 1;
    const QString invalidSp3Path = temp.filePath(QStringLiteral("invalid.sp3"));
    if (!check(writeText(invalidSp3Path, QStringLiteral("*  2024 01 01 00 00 00.00000000\n"
                                                         "PG01 999999.999999 999999.999999 999999.999999 0.0\n")),
               "write invalid SP3")) return 1;
    QMap<QString, QVector<PreciseSatelliteState>> invalidPrecise;
    if (!check(!PreciseProductReader::readSp3(invalidSp3Path, invalidPrecise, &error),
               "reject invalid SP3 position")) return 1;
    int week = 0;
    double sow = 0.0;
    GnssUtils::gpsWeekFromDateTime(start.addSecs(3600), week, sow);
    PreciseSatelliteState interpolated;
    const QVector<PreciseSatelliteState> samples = precise.constBegin().value();
    if (!check(PreciseProductReader::interpolate(samples, week, sow, interpolated) && interpolated.hasClock,
               "9-point precise interpolation")) return 1;
    GnssUtils::gpsWeekFromDateTime(start, week, sow);
    QString interpolationDiagnostic;
    if (!check(!PreciseProductReader::interpolate(samples, week, sow, interpolated,
                                                  &interpolationDiagnostic) &&
               interpolationDiagnostic.contains(QStringLiteral("window")),
               "precise edge window rejection")) return 1;

    const QString vtecPath = temp.filePath(QStringLiteral("test-vtec.ssr"));
    const QString vtec = QStringLiteral("> VTEC 2024 01 01 00 00 00.0 6 1 TEST\n"
                                        "  1 1 1 450000.0\n"
                                        "  1.0 2.0 3.0 4.0 3.0 4.0 5.0 6.0\n");
    if (!check(writeText(vtecPath, vtec), "write VTEC")) return 1;
    SsrFileReader::Result vtecResult;
    if (!check(SsrFileReader::read(QStringList() << vtecPath, vtecResult, &error) &&
               vtecResult.vtec.size() == 1 && vtecResult.vtec.first().layers.size() == 1,
               "read VTEC model")) return 1;
    const PppVtecLayer& vtecLayer = vtecResult.vtec.first().layers.first();
    if (!check(vtecLayer.cosine.size() == 2 && vtecLayer.cosine.at(0).size() == 2 &&
               std::fabs(vtecLayer.cosine.at(0).at(0) - 1.0) < 1.0e-12 &&
               std::fabs(vtecLayer.sine.at(0).at(0) - 3.0) < 1.0e-12,
               "VTEC coefficient layout")) return 1;
    SsrFileReader::Result duplicateVtec;
    if (!check(SsrFileReader::read(QStringList() << vtecPath << vtecPath, duplicateVtec, &error) &&
               duplicateVtec.vtec.size() == 1,
               "VTEC duplicate merge")) return 1;
    const QString conflictVtecPath = temp.filePath(QStringLiteral("conflict-vtec.ssr"));
    const QString conflictVtec = QStringLiteral("> VTEC 2024 01 01 00 00 00.0 6 1 TEST\n"
                                                 "  1 1 1 450000.0\n"
                                                 "  1.0 2.0 3.0 4.0 3.0 4.0 5.0 6.1\n");
    if (!check(writeText(conflictVtecPath, conflictVtec), "write conflicting VTEC")) return 1;
    SsrFileReader::Result conflictingVtec;
    if (!check(!SsrFileReader::read(QStringList() << vtecPath << conflictVtecPath,
                                   conflictingVtec, &error),
               "VTEC conflict rejection")) return 1;

    std::cout << "postppp smoke test passed\n";
    return 0;
}
