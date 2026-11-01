// Checks compact PPP JSON metadata, records, ordering, and recovery behavior.
#include "pppjsonwriter.h"
#include "gnss_utils.h"

#include <QCoreApplication>
#include <QDir>
#include <QFile>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QTemporaryDir>
#include <cmath>
#include <iostream>
#include <limits>

namespace {

PppSolution makeSolution(const QDateTime& utc, double x) {
    PppSolution solution;
    solution.valid = true;
    solution.utc = utc;
    GnssUtils::gpsWeekFromDateTime(utc, solution.gpsWeek, solution.gpsSow);
    solution.xyz[0] = x;
    solution.xyz[1] = 2.0;
    solution.xyz[2] = 3.0;
    solution.neu[0] = 0.1;
    solution.neu[1] = 0.2;
    solution.neu[2] = 0.3;
    solution.cov[0] = std::numeric_limits<double>::quiet_NaN();
    solution.cov[3] = 0.04;
    solution.cov[5] = 0.09;
    solution.ztd = 2.4;
    solution.trp = 0.2;
    solution.ztdStdev = 0.01;
    solution.hDop = 1.2;
    solution.numSat = 3;
    solution.arEnabled = true;
    solution.arAlgorithm = QStringLiteral("BIE");
    solution.arStatus = QStringLiteral("partial_fix");
    solution.arCandidateCount = 100;
    solution.arEffectiveCandidates = 2.5;
    solution.arConstraintsApplied = 2;
    solution.usedSatellites = QStringList() << "G01" << "R02" << "C03";
    solution.arEligibleSatellites = QStringList() << "G01" << "R02";
    solution.arFixedSatellites = QStringList() << "G01";
    PppArGroupResult group;
    group.system = QLatin1Char('G');
    group.group = QStringLiteral("L1");
    group.referenceSatellite = QStringLiteral("G01");
    group.fixedCount = 1;
    group.eligibleCount = 2;
    group.maxFractionCycle = 0.005;
    group.maxSigmaCycle = 0.002;
    solution.arGroups.push_back(group);
    return solution;
}

QJsonObject readRoot(const QString& path) {
    QFile file(path);
    if (!file.open(QIODevice::ReadOnly)) {
        return QJsonObject();
    }
    return QJsonDocument::fromJson(file.readAll()).object();
}

bool check(bool condition, const char* message) {
    if (!condition) {
        std::cerr << message << std::endl;
    }
    return condition;
}

} // namespace

int main(int argc, char* argv[]) {
    QCoreApplication app(argc, argv);
    QTemporaryDir temporary;
    if (!check(temporary.isValid(), "temporary directory creation failed")) return 1;

    const std::array<double, 3> reference = {1.0, 2.0, 3.0};
    const QDate day(2026, 7, 27);
    const QDateTime midnight(day, QTime(0, 0), Qt::UTC);
    PppJsonWriter::Options options;
    options.outputDirectory = temporary.path();
    options.reorderBufferMs = 2000;
    options.flushIntervalMs = 0;

    QString error;
    {
        PppJsonWriter writer;
        if (!check(writer.setOptions(options, &error), qPrintable(error))) return 1;
        if (!check(writer.append("BJFS", makeSolution(midnight.addMSecs(1000), 10.0), reference, &error), qPrintable(error))) return 1;
        const QString activePath = QDir(temporary.path()).filePath(QStringLiteral("BJFS/ppp_BJFS_20260727.json"));
        if (!check(QFile::exists(activePath), "active PPP JSON file was not created")) return 1;
        if (!check(!QFile::exists(activePath + QStringLiteral(".part")), "normal PPP JSON writing created a part file")) return 1;
        if (!check(writer.append("BJFS", makeSolution(midnight.addMSecs(1000), 11.0), reference, &error), qPrintable(error))) return 1;
        if (!check(writer.append("BJFS", makeSolution(midnight.addMSecs(5000), 12.0), reference, &error), qPrintable(error))) return 1;
        if (!check(writer.append("WHU2", makeSolution(midnight.addMSecs(1500), 20.0), reference, &error), qPrintable(error))) return 1;
        if (!check(writer.append("BJFS", makeSolution(midnight.addDays(1).addMSecs(500), 30.0), reference, &error), qPrintable(error))) return 1;
        if (!check(writer.close(&error), qPrintable(error))) return 1;
    }

    const QString bjfsDay1 = QDir(temporary.path()).filePath(QStringLiteral("BJFS/ppp_BJFS_20260727.json"));
    const QString bjfsDay2 = QDir(temporary.path()).filePath(QStringLiteral("BJFS/ppp_BJFS_20260728.json"));
    const QString whuDay1 = QDir(temporary.path()).filePath(QStringLiteral("WHU2/ppp_WHU2_20260727.json"));
    QJsonObject bjfsRoot = readRoot(bjfsDay1);
    QJsonObject whuRoot = readRoot(whuDay1);
    if (!check(!bjfsRoot.isEmpty() && !whuRoot.isEmpty() && QFile::exists(bjfsDay2), "station/day files were not created")) return 1;
    QJsonArray bjfsSolutions = bjfsRoot.value(QStringLiteral("solutions")).toArray();
    if (!check(bjfsSolutions.size() == 2, "duplicate epoch was not replaced")) return 1;
    if (!check(bjfsSolutions[0].toArray()[0].toDouble() == 19000.0,
               "PPP JSON time offset does not use the GPS calendar epoch")) return 1;
    if (!check(bjfsSolutions[0].toArray()[3].toArray()[0].toDouble() == 11.0, "latest duplicate was not retained")) return 1;
    const QJsonObject metadata = bjfsRoot.value(QStringLiteral("meta")).toObject();
    if (!check(metadata.value(QStringLiteral("version")).toInt() == 1, "schema version is incorrect")) return 1;
    if (!check(metadata.value(QStringLiteral("timeBase")).toObject()
                   .value(QStringLiteral("system")).toString() == QLatin1String("GPS"),
               "PPP JSON time base is not labelled GPS")) return 1;
    if (!check(!metadata.contains(QStringLiteral("enuOrder")), "v1 metadata contains an unknown field")) return 1;
    const QJsonArray enu = bjfsSolutions[0].toArray()[4].toArray();
    if (!check(enu.size() == 3 && enu[0].toDouble() == 0.1 &&
               enu[1].toDouble() == 0.2 && enu[2].toDouble() == 0.3,
               "internal NEU values were not serialized in compact v1 order")) return 1;
    if (!check(bjfsSolutions[0].toArray()[5].toArray()[0].isNull(), "non-finite number was not encoded as null")) return 1;
    if (!check(whuRoot.value(QStringLiteral("meta")).toObject()
                   .value(QStringLiteral("station")).toObject()
                   .value(QStringLiteral("id")).toString() == QLatin1String("WHU2"),
               "station metadata is incorrect")) return 1;

    {
        PppJsonWriter writer;
        if (!check(writer.setOptions(options, &error), qPrintable(error))) return 1;
        if (!check(writer.append("BJFS", makeSolution(midnight.addMSecs(6000), 13.0), reference, &error), qPrintable(error))) return 1;
        if (!check(writer.close(&error), qPrintable(error))) return 1;
    }
    bjfsRoot = readRoot(bjfsDay1);
    bjfsSolutions = bjfsRoot.value(QStringLiteral("solutions")).toArray();
    if (!check(bjfsSolutions.size() == 3, "same-day restart did not append to the daily file")) return 1;

    const QString bjfsPart = bjfsDay1 + QStringLiteral(".part");
    {
        QFile finalized(bjfsDay1);
        if (!check(finalized.open(QIODevice::ReadOnly), "cannot open finalized JSON for recovery test")) return 1;
        QByteArray interrupted = finalized.readAll();
        finalized.close();
        if (!check(interrupted.endsWith(QByteArrayLiteral("]}\n")), "unexpected finalized JSON suffix")) return 1;
        interrupted.chop(3);
        interrupted += QByteArrayLiteral(",[1234");
        if (!check(QFile::rename(bjfsDay1, bjfsPart), "cannot create interrupted part file")) return 1;
        QFile part(bjfsPart);
        if (!check(part.open(QIODevice::WriteOnly | QIODevice::Truncate), "cannot write interrupted part file")) return 1;
        if (!check(part.write(interrupted) == interrupted.size(), "cannot persist interrupted part file")) return 1;
    }
    {
        PppJsonWriter writer;
        if (!check(writer.setOptions(options, &error), qPrintable(error))) return 1;
        if (!check(writer.append("BJFS", makeSolution(midnight.addMSecs(7000), 14.0), reference, &error), qPrintable(error))) return 1;
        if (!check(writer.close(&error), qPrintable(error))) return 1;
    }
    bjfsRoot = readRoot(bjfsDay1);
    bjfsSolutions = bjfsRoot.value(QStringLiteral("solutions")).toArray();
    if (!check(bjfsSolutions.size() == 4, "interrupted part file was not recovered")) return 1;
    if (!check(!QFile::exists(bjfsPart), "legacy part file was not migrated")) return 1;

    if (!check(QFile::copy(bjfsDay1, bjfsPart), "cannot create PPP JSON recovery conflict")) return 1;
    {
        PppJsonWriter writer;
        error.clear();
        if (!check(writer.setOptions(options, &error), qPrintable(error))) return 1;
        if (!check(!writer.append("BJFS", makeSolution(midnight.addMSecs(8000), 15.0), reference, &error),
                   "PPP JSON recovery conflict was accepted")) return 1;
        if (!check(error.contains(QStringLiteral("recovery conflict")),
                   "PPP JSON recovery conflict did not report its cause")) return 1;
    }

    std::cout << "pppjsonwriter smoke test passed" << std::endl;
    return 0;
}
