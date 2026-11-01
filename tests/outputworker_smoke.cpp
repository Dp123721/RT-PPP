// Checks queued output serialization, file isolation, and generated product names.
#include "outputworker.h"
#include "gnss_utils.h"

#include <QCoreApplication>
#include <QDir>
#include <QDirIterator>
#include <QFile>
#include <QFileInfo>
#include <QJsonDocument>
#include <QJsonObject>
#include <QRegularExpression>
#include <QTemporaryDir>

#include <array>
#include <iostream>

namespace {

bool check(bool condition, const char* message) {
    if (!condition) {
        std::cerr << "outputworker smoke test failed: " << message << "\n";
    }
    return condition;
}

QStringList findSsrFiles(const QString& directory) {
    QStringList paths;
    QDirIterator it(directory, QStringList() << QStringLiteral("*.ssr"), QDir::Files,
                    QDirIterator::Subdirectories);
    while (it.hasNext()) paths << it.next();
    paths.sort();
    return paths;
}

} // namespace

int main(int argc, char* argv[]) {
    QCoreApplication app(argc, argv);
    QTemporaryDir temporary;
    if (!check(temporary.isValid(), "temporary directory creation failed")) return 1;

    const QDateTime utc(QDate(2026, 8, 25), QTime(11, 59, 57), Qt::UTC);
    const GnssUtils::GpsTime gpsTime = GnssUtils::currentGpsTime(utc);

    OutputWorker::Config config;
    config.sp3Enabled = true;
    config.sp3Directory = QDir(temporary.path()).filePath("sp3");
    config.sp3SampleSec = 1;
    config.rinexEnabled = true;
    config.rinexDirectory = QDir(temporary.path()).filePath("rinex");
    config.pppJsonEnabled = true;
    config.pppJsonDirectory = QDir(temporary.path()).filePath("ppp_json");
    config.pppJsonReorderBufferMs = 0;
    config.pppJsonFlushIntervalMs = 0;
    config.ztdDirectory = QDir(temporary.path()).filePath("ztd");
    config.logFilePath = QDir(temporary.path()).filePath("runtime.log");
    config.telemetryDirectory = QDir(temporary.path()).filePath("telemetry");
    config.telemetrySourceNames.insert(QStringLiteral("caster/a"), QStringLiteral("SSRC00WHU0"));
    // The two receiver connections intentionally share a mountpoint.  Their SSR
    // products must still be kept in separate source directories.
    config.telemetrySourceNames.insert(QStringLiteral("caster:a"), QStringLiteral("SSRC00WHU0"));
    config.arLogFilePath = QDir(temporary.path()).filePath("pppar.log");

    PppSolution solution;
    solution.valid = true;
    solution.utc = utc;
    solution.gpsWeek = gpsTime.week;
    solution.gpsSow = gpsTime.sow;
    solution.xyz[0] = 1111111.0;
    solution.xyz[1] = 2222222.0;
    solution.xyz[2] = 3333333.0;
    solution.ztd = 2.345;
    solution.ztdStdev = 0.012;
    solution.trp = 0.123;
    solution.numSat = 12;
    solution.hDop = 1.2;
    solution.arLogLines << QStringLiteral("AR smoke line");

    OutputWorker worker;
    worker.configure(config);
    worker.enqueueLog(QStringLiteral("runtime smoke line"));
    worker.enqueueArLog(solution);
    worker.enqueueZtd(QStringLiteral("OBS"), QStringLiteral("BJFS"), solution);
    worker.enqueuePppJson(QStringLiteral("BJFS"), solution,
                          {solution.xyz[0], solution.xyz[1], solution.xyz[2]});
    worker.enqueueTelemetryReception(QStringLiteral("OBS"), utc, QStringLiteral("MSM"),
                                      1077, QLatin1Char('G'), QStringLiteral("received"),
                                      QString(), QStringLiteral("smoke"));
    RtcmSsrMessage ssr;
    ssr.kind = RtcmSsrCorrectionKind::OrbitClock;
    ssr.system = QLatin1Char('G');
    ssr.messageType = 1060;
    ssr.epochTime = 43200;
    ssr.updateIntervalIndex = 2;
    ssr.ssrIod = 7;
    ssr.providerId = 74;
    ssr.solutionId = 1;
    RtcmSsrSatelliteData ssrSatellite;
    ssrSatellite.sat.system = QLatin1Char('G');
    ssrSatellite.sat.number = 1;
    ssrSatellite.iod = 2;
    ssrSatellite.hasOrbit = true;
    ssrSatellite.deltaRadial = 0.1234;
    ssrSatellite.deltaAlongTrack = -0.2345;
    ssrSatellite.deltaCrossTrack = 0.3456;
    ssrSatellite.dotDeltaRadial = 0.0012;
    ssrSatellite.hasClock = true;
    ssrSatellite.deltaC0 = -0.4567;
    worker.enqueueTelemetrySsr(QStringLiteral("caster/a"), utc, utc, ssr,
                               ssrSatellite, true, QStringLiteral("accepted"));
    RtcmSsrSatelliteData secondSatellite = ssrSatellite;
    secondSatellite.sat.system = QLatin1Char('R');
    secondSatellite.sat.number = 2;
    ssr.system = QLatin1Char('R');
    worker.enqueueTelemetrySsr(QStringLiteral("caster/a"), utc, utc, ssr,
                               secondSatellite, true, QStringLiteral("accepted"));
    ssr.system = QLatin1Char('G');
    RtcmVtecMessage vtec;
    vtec.messageType = 1264;
    vtec.epochTime = 43200;
    vtec.updateIntervalIndex = 2;
    vtec.providerId = 74;
    vtec.solutionId = 1;
    RtcmVtecLayer vtecLayer;
    vtecLayer.heightMeters = 450000.0;
    vtecLayer.cosine = {{1.25}};
    vtecLayer.sine = {{-0.5}};
    vtec.layers.push_back(vtecLayer);
    worker.enqueueTelemetryVtec(QStringLiteral("caster/a"), utc, vtec);
    worker.enqueueTelemetrySsr(QStringLiteral("caster:a"), utc, utc, ssr,
                               ssrSatellite, false, QStringLiteral("rejected"));
    ssr.providerId = 75;
    ssr.solutionId = 2;
    worker.enqueueTelemetrySsr(QStringLiteral("caster/a"), utc, utc, ssr,
                               ssrSatellite, true, QStringLiteral("accepted"));
    worker.enqueueSp3(gpsTime.week, gpsTime.sow, QStringLiteral("G01"),
                      {15600000.0, 20100000.0, 21700000.0}, 1.0e-4,
                      {0.0, 0.0, 0.0}, 0.0);
    QMap<QString, QMap<QString, double>> observations;
    observations[QStringLiteral("G01")][QStringLiteral("C1C")] = 22000000.0;
    observations[QStringLiteral("G01")][QStringLiteral("L1C")] = 110000000.0;
    worker.enqueueRinex(gpsTime.week, gpsTime.sow, observations);
    worker.finalizePppJson();

    const QString day = utc.toString(QStringLiteral("yyyyMMdd"));
    if (!check(QFile::exists(config.logFilePath), "runtime log was not written")) return 1;
    if (!check(QFile::exists(config.arLogFilePath), "AR log was not written")) return 1;
    const QString ztdPath = QDir(config.ztdDirectory).filePath("ztd_BJFS.txt");
    if (!check(QFile::exists(ztdPath),
               "ZTD file was not written")) return 1;
    QFile ztdFile(ztdPath);
    if (!check(ztdFile.open(QIODevice::ReadOnly), "ZTD file cannot be opened")) return 1;
    const QByteArray ztd = ztdFile.readAll();
    if (!check(ztd.contains(QByteArrayLiteral("GPS ISO8601")) &&
                   ztd.contains(QByteArrayLiteral("2026-08-25T12:00:15.000Z")),
               "ZTD output does not use the GPS calendar epoch")) return 1;
    QFile arFile(config.arLogFilePath);
    if (!check(arFile.open(QIODevice::ReadOnly), "AR log cannot be opened")) return 1;
    if (!check(arFile.readAll().startsWith(QByteArrayLiteral("2026-08-25 12:00:15.000")),
               "AR log does not use the GPS calendar epoch")) return 1;
    if (!check(QFile::exists(QDir(config.pppJsonDirectory)
                                 .filePath("BJFS/ppp_BJFS_20260825.json")),
               "PPP JSON file was not written")) return 1;
    const QString pppJsonPath = QDir(config.pppJsonDirectory)
                                    .filePath("BJFS/ppp_BJFS_20260825.json");
    QFile pppJsonFile(pppJsonPath);
    if (!check(pppJsonFile.open(QIODevice::ReadOnly), "PPP JSON file cannot be opened")) return 1;
    const QByteArray pppJson = pppJsonFile.readAll();
    if (!check(pppJson.endsWith(QByteArrayLiteral("]}\n")), "PPP JSON was not finalized")) return 1;
    if (!check(!QJsonDocument::fromJson(pppJson).isNull(), "PPP JSON is not valid after finalization")) return 1;
    const QJsonObject pppRoot = QJsonDocument::fromJson(pppJson).object();
    if (!check(pppRoot.value(QStringLiteral("meta")).toObject()
                   .value(QStringLiteral("timeBase")).toObject()
                   .value(QStringLiteral("system")).toString() == QLatin1String("GPS"),
               "PPP JSON is not labelled as GPS time")) return 1;
    if (!check(QFile::exists(QDir(config.telemetryDirectory)
                                 .filePath(QString("reception_%1.csv").arg(day))),
               "telemetry file was not written")) return 1;
    if (!check(QFile::exists(QDir(config.sp3Directory)
                                 .filePath(QString("rtppp_%1.sp3").arg(day))),
               "SP3 file was not written")) return 1;
    if (!check(QFile::exists(QDir(config.sp3Directory)
                                 .filePath(QString("rtppp_%1.clk").arg(day))),
               "CLK file was not written")) return 1;
    if (!check(!QDir(config.rinexDirectory)
                    .entryList(QStringList() << QStringLiteral("*.rnx"), QDir::Files)
                    .isEmpty(),
               "RINEX file was not written")) return 1;

    worker.shutdown();

    const QStringList ssrFiles = findSsrFiles(config.telemetryDirectory);
    if (!check(ssrFiles.size() == 4,
               "SSR telemetry was not split by source, product, and SSR product family")) return 1;
    bool sourceAProduct1 = false;
    bool sourceBProduct1 = false;
    bool sourceAProduct2 = false;
    const QString expectedDate = QStringLiteral("%1%2%3")
        .arg(utc.date().year()).arg(utc.date().dayOfYear(), 3, 10, QLatin1Char('0'))
        .arg(QStringLiteral("0000"));
    const QRegularExpression expectedSsrFileName(
        QStringLiteral("^SSRC00WHU0_S_%1_01D_(MC|ION)\\.ssr$").arg(expectedDate));
    int sourceDirectories = 0;
    const QDir mountDirectory(QDir(config.telemetryDirectory).filePath("ssr/SSRC00WHU0"));
    for (const QFileInfo& entry : mountDirectory.entryInfoList(QDir::Dirs | QDir::NoDotAndDotDot)) {
        if (entry.isDir()) ++sourceDirectories;
    }
    if (!check(sourceDirectories == 2,
               "same-mountpoint receiver connections were not isolated into source directories")) return 1;
    for (const QString& path : ssrFiles) {
        const QString fileName = QFileInfo(path).fileName();
        if (!check(expectedSsrFileName.match(fileName).hasMatch(),
                   "SSR file name is not daily SSR format")) return 1;
        QFile ssrFile(path);
        if (!check(ssrFile.open(QIODevice::ReadOnly), "SSR telemetry file cannot be opened")) return 1;
        const QByteArray contents = ssrFile.readAll();
        const bool isIon = fileName.endsWith(QStringLiteral("_ION.ssr"));
        const QString relative = QDir(config.telemetryDirectory).relativeFilePath(path);
        const bool product1 = relative.contains(QStringLiteral("/p74_s1/"));
        const bool sourceA = relative.contains(QStringLiteral("/caster_a_")) && product1;
        if (isIon) {
            if (!check(contents.contains(QByteArrayLiteral("> VTEC ")),
                       "VTEC output does not use SSR coefficient layout")) return 1;
            if (!check(sourceA, "VTEC product was not isolated to its source/product file")) return 1;
            continue;
        }
        const bool twoSatellites = contents.count(QByteArrayLiteral("R02")) == 2;
        if (!check(contents.count('\n') == (twoSatellites ? 6 : 4),
                   "SSR file has an unexpected block count")) return 1;
        if (!check(contents.contains(QByteArrayLiteral("> ORBIT ")) &&
                       contents.contains(QByteArrayLiteral("> CLOCK ")) &&
                       contents.contains(QByteArrayLiteral("0.1234")) &&
                       contents.contains(QByteArrayLiteral("-0.4567")) &&
                       contents.count(QByteArrayLiteral("> ORBIT ")) == 1 &&
                       contents.count(QByteArrayLiteral("> CLOCK ")) == 1,
                   "SSR output is not in SSR ASCII block format")) return 1;
        if (!check(contents.contains(QByteArrayLiteral("SSRC00WHU0")),
                   "SSR source identity is absent from its metadata")) return 1;
        if (twoSatellites && !check(contents.contains(QByteArrayLiteral("2 2 SSRC00WHU0")) &&
                                    contents.count(QByteArrayLiteral("R02")) == 2,
                                    "multiple systems were not grouped into a single epoch")) return 1;
        if (product1 && twoSatellites) {
            sourceAProduct1 = true;
        } else if (product1) {
            sourceBProduct1 = true;
        } else if (relative.contains(QStringLiteral("/p75_s2/"))) {
            sourceAProduct2 = true;
        } else {
            return check(false, "SSR file contains an unexpected product name");
        }
    }
    if (!check(sourceAProduct1 && sourceBProduct1 && sourceAProduct2,
               "SSR source/product rows were mixed or lost")) return 1;

    std::cout << "outputworker smoke test passed\n";
    return 0;
}
