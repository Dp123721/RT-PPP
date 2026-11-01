// Declares offline PPP processing inputs, progress events, and service controls.
#pragma once

#include "ppp_engine.h"

#include <QAtomicInt>
#include <QDateTime>
#include <QMap>
#include <QObject>
#include <QStringList>
#include <QVector>

#include <array>
#include <memory>

struct RinexObservation {
    RtcmSatId sat;
    QMap<QString, double> values;
    QMap<QString, int> lli;
    QMap<QString, int> ssi;
};

struct RinexEpoch {
    int gpsWeek = 0;
    double gpsSow = 0.0;
    int flag = 0;
    QVector<RinexObservation> observations;
};

struct RinexObservationFile {
    QString path;
    QString markerName;
    QString receiver;
    QString antenna;
    QString timeSystem = QStringLiteral("GPS");
    double approximatePosition[3] = {0.0, 0.0, 0.0};
    bool hasApproximatePosition = false;
    QMap<QChar, QStringList> observationTypes;
    QMap<int, int> glonassChannels;
    QVector<RinexEpoch> epochs;
    double version = 0.0;
};

class RinexReader {
public:
    static bool readObservation(const QString& path,
                                RinexObservationFile& out,
                                QString* error = nullptr);
    static bool readNavigation(const QString& path,
                               QVector<RtcmBroadcastEphemeris>& out,
                               QString* error = nullptr);
};

class SsrFileReader {
public:
    struct Result {
        QVector<RtcmSsrMessage> messages;
        QVector<PppVtecModel> vtec;
        QStringList warnings;
    };

    static bool read(const QStringList& paths, Result& out, QString* error = nullptr);
};

using PreciseSatelliteState = PppPreciseSatelliteState;

class PreciseProductReader {
public:
    static bool readSp3(const QString& path,
                        QMap<QString, QVector<PreciseSatelliteState>>& out,
                        QString* error = nullptr);
    static bool readClock(const QString& path,
                          QMap<QString, QVector<PreciseSatelliteState>>& states,
                          QString* error = nullptr);
    static bool interpolate(const QVector<PreciseSatelliteState>& samples,
                            int gpsWeek,
                            double gpsSow,
                            PreciseSatelliteState& out,
                            QString* diagnostic = nullptr);
};

struct PostPppJobConfig {
    QString jobName;
    QString observationPath;
    QString navigationPath;
    QStringList ssrPaths;
    QString sp3Path;
    QString clockPath;
    QString outputDirectory;
    QString ztdOutputDirectory;
    QString productMode = QStringLiteral("broadcast");
    int sampleSec = 5;
    bool jsonEnabled = true;
    int jsonReorderBufferMs = 2000;
    int jsonFlushIntervalMs = 1000;
    PppOptions options;
    PppCrdStationOptions station;
};

Q_DECLARE_METATYPE(PppSolution)

class PostPppService : public QObject {
    Q_OBJECT
public:
    explicit PostPppService(QObject* parent = nullptr);

public slots:
    void run(PostPppJobConfig config);
    void cancel();

signals:
    void started(const QString& jobDirectory, int totalEpochs);
    void progress(int processed, int total, const QString& epochText);
    void solutionReady(const QString& sourceId, const PppSolution& solution);
    void logMessage(const QString& message);
    void finished(bool cancelled, int processed, int accepted, const QString& summaryPath);
    void failed(const QString& reason);

private:
    QAtomicInt _cancelRequested = 0;
};
