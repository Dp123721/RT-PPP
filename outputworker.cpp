// Serializes accepted output jobs on one worker thread and closes writers cleanly.
#include "outputworker.h"

#include "gnss_utils.h"

#include <QDir>
#include <QDebug>
#include <QFile>
#include <QFileInfo>
#include <QRegularExpression>
#include <QTextStream>
#include <QMetaObject>

#include <cmath>

namespace {

QString safeStationName(QString station) {
    station = station.trimmed().toUpper();
    if (station.isEmpty()) {
        station = QStringLiteral("UNKNOWN");
    }
    station.replace(QRegularExpression(QStringLiteral("[^A-Z0-9_\\-]")), QStringLiteral("_"));
    return station;
}

QString number(double value) {
    return std::isfinite(value) ? QString::number(value, 'f', 6) : QString();
}

QDateTime gpsOutputTime(const PppSolution& solution) {
    if (solution.gpsTime.isValid()) {
        return solution.gpsTime;
    }
    int gpsWeek = solution.gpsWeek;
    double gpsSow = solution.gpsSow;
    if (gpsWeek <= 0 || !std::isfinite(gpsSow)) {
        GnssUtils::gpsWeekFromDateTime(solution.utc, gpsWeek, gpsSow);
    }
    return GnssUtils::gpsCalendarDateTime(gpsWeek, gpsSow);
}

} // namespace

OutputWorker::OutputWorker(QObject* parent)
    : QObject(parent) {
}

OutputWorker::~OutputWorker() {
    shutdown();
}

void OutputWorker::configure(const Config& config) {
    Job job;
    job.kind = Kind::Configure;
    job.config = config;
    submit(std::move(job));
}

void OutputWorker::enqueueSp3(int gpsWeek,
                              double gpsSow,
                              const QString& prn,
                              const std::array<double, 3>& position,
                              double clockSec,
                              const std::array<double, 3>& velocity,
                              double clockRate) {
    Job job;
    job.kind = Kind::Sp3;
    job.gpsWeek = gpsWeek;
    job.gpsSow = gpsSow;
    job.prn = prn;
    job.position = position;
    job.clockSec = clockSec;
    job.velocity = velocity;
    job.clockRate = clockRate;
    submit(std::move(job));
}

void OutputWorker::enqueueRinex(int gpsWeek,
                                double gpsSow,
                                const QMap<QString, QMap<QString, double>>& observations) {
    Job job;
    job.kind = Kind::Rinex;
    job.gpsWeek = gpsWeek;
    job.gpsSow = gpsSow;
    job.observations = observations;
    submit(std::move(job));
}

void OutputWorker::enqueuePppJson(const QString& stationId,
                                  const PppSolution& solution,
                                  const std::array<double, 3>& referenceEcef) {
    Job job;
    job.kind = Kind::PppJson;
    job.stationId = stationId;
    job.solution = solution;
    job.referenceEcef = referenceEcef;
    submit(std::move(job));
}

void OutputWorker::enqueueZtd(const QString& source,
                              const QString& station,
                              const PppSolution& solution) {
    Job job;
    job.kind = Kind::Ztd;
    job.source = source;
    job.station = station;
    job.solution = solution;
    submit(std::move(job));
}

void OutputWorker::enqueueLog(const QString& text) {
    if (text.isEmpty()) {
        return;
    }
    Job job;
    job.kind = Kind::Log;
    job.logText = text;
    submit(std::move(job));
}

void OutputWorker::enqueueTelemetryMsm(
    const QString& sourceId,
    const QDateTime& receivedUtc,
    const QDateTime& epochUtc,
    const RtcmMsmMessage& message,
    const QMap<QString, QPair<double, double>>& skyBySatellite) {
    Job job;
    job.kind = Kind::TelemetryMsm;
    job.source = sourceId;
    job.receivedUtc = receivedUtc;
    job.epochUtc = epochUtc;
    job.msm = message;
    job.skyBySatellite = skyBySatellite;
    submit(std::move(job));
}

void OutputWorker::enqueueTelemetrySsr(const QString& sourceId,
                                       const QDateTime& receivedUtc,
                                       const QDateTime& epochUtc,
                                       const RtcmSsrMessage& message,
                                       const RtcmSsrSatelliteData& satellite,
                                       bool adopted,
                                       const QString& reason) {
    Job job;
    job.kind = Kind::TelemetrySsr;
    job.source = sourceId;
    job.receivedUtc = receivedUtc;
    job.epochUtc = epochUtc;
    job.ssr = message;
    job.ssrSatellite = satellite;
    job.adopted = adopted;
    job.reason = reason;
    submit(std::move(job));
}

void OutputWorker::enqueueTelemetryVtec(const QString& sourceId,
                                        const QDateTime& epochUtc,
                                        const RtcmVtecMessage& message) {
    Job job;
    job.kind = Kind::TelemetryVtec;
    job.source = sourceId;
    job.epochUtc = epochUtc;
    job.vtec = message;
    submit(std::move(job));
}

void OutputWorker::enqueueTelemetryReception(const QString& sourceId,
                                              const QDateTime& receivedUtc,
                                              const QString& messageFamily,
                                              int messageType,
                                              const QChar& system,
                                              const QString& status,
                                              const QString& reason,
                                              const QString& detail) {
    Job job;
    job.kind = Kind::TelemetryReception;
    job.source = sourceId;
    job.receivedUtc = receivedUtc;
    job.messageFamily = messageFamily;
    job.messageType = messageType;
    job.system = system;
    job.status = status;
    job.reason = reason;
    job.detail = detail;
    submit(std::move(job));
}

void OutputWorker::enqueueTelemetryPppResiduals(const QString& sourceId,
                                                const PppSolution& solution) {
    Job job;
    job.kind = Kind::TelemetryPppResiduals;
    job.source = sourceId;
    job.solution = solution;
    submit(std::move(job));
}

void OutputWorker::enqueueArLog(const PppSolution& solution) {
    if (solution.arLogLines.isEmpty()) {
        return;
    }
    Job job;
    job.kind = Kind::ArLog;
    job.solution = solution;
    submit(std::move(job));
}

void OutputWorker::submit(Job job) {
    if (!_accepting.loadAcquire()) {
        return;
    }
    {
        QMutexLocker locker(&_mutex);
        if (!_accepting.loadAcquire()) {
            return;
        }
        constexpr int kMaxQueuedOutputJobs = 20000;
        if (_jobs.size() >= kMaxQueuedOutputJobs &&
            job.kind != Kind::Configure && job.kind != Kind::Log) {
            qWarning("Output queue full; output job dropped");
            return;
        }
        _jobs.push_back(std::move(job));
    }
    scheduleDrain();
}

void OutputWorker::scheduleDrain() {
    if (!_drainScheduled.testAndSetOrdered(0, 1)) {
        return;
    }
    QMetaObject::invokeMethod(this, [this]() { drain(); }, Qt::QueuedConnection);
}

void OutputWorker::drain() {
    QVector<Job> batch;
    {
        QMutexLocker locker(&_mutex);
        const int count = qMin(128, _jobs.size());
        batch.reserve(count);
        for (int i = 0; i < count; ++i) {
            batch.push_back(std::move(_jobs[i]));
        }
        if (count > 0) {
            _jobs.remove(0, count);
        }
    }

    for (const Job& job : batch) {
        process(job);
    }

    _drainScheduled.storeRelease(0);
    bool hasMore = false;
    {
        QMutexLocker locker(&_mutex);
        hasMore = !_jobs.isEmpty();
    }
    if (hasMore) {
        scheduleDrain();
    }
}

void OutputWorker::process(const Job& job) {
    switch (job.kind) {
    case Kind::Configure:
        applyConfig(job.config);
        break;
    case Kind::Sp3:
        if (_config.sp3Enabled &&
            !_sp3Writer.writeEpoch(job.gpsWeek, job.gpsSow, job.prn,
                                   job.position, job.clockSec,
                                   job.velocity, job.clockRate)) {
            reportError(QStringLiteral("SP3"), QStringLiteral("writeEpoch failed"));
        }
        break;
    case Kind::Rinex:
        if (_config.rinexEnabled &&
            !_rinexWriter.writeEpoch(job.gpsWeek, job.gpsSow, job.observations)) {
            reportError(QStringLiteral("RINEX"), QStringLiteral("writeEpoch failed"));
        }
        break;
    case Kind::PppJson: {
        if (_config.pppJsonEnabled) {
            QString error;
            if (!_pppJsonWriter.append(job.stationId, job.solution, job.referenceEcef, &error)) {
                reportError(QStringLiteral("PPP JSON"), error);
            }
        }
        break;
    }
    case Kind::Ztd:
        writeZtd(job);
        break;
    case Kind::Log:
        writeLog(job);
        break;
    case Kind::TelemetryMsm:
        if (_telemetryWriter) {
            _telemetryWriter->writeMsm(job.source, job.receivedUtc, job.epochUtc,
                                       job.msm, job.skyBySatellite);
        }
        break;
    case Kind::TelemetrySsr:
        if (_telemetryWriter) {
            _telemetryWriter->writeSsr(job.source, job.receivedUtc, job.epochUtc,
                                       job.ssr, job.ssrSatellite, job.adopted, job.reason);
        }
        break;
    case Kind::TelemetryVtec:
        if (_telemetryWriter) {
            _telemetryWriter->writeVtec(job.source, job.epochUtc, job.vtec);
        }
        break;
    case Kind::TelemetryReception:
        if (_telemetryWriter) {
            _telemetryWriter->writeReception(job.source, job.receivedUtc, job.messageFamily,
                                             job.messageType, job.system, job.status,
                                             job.reason, job.detail);
        }
        break;
    case Kind::TelemetryPppResiduals:
        if (_telemetryWriter) {
            _telemetryWriter->writePppResiduals(job.source, job.solution);
        }
        break;
    case Kind::ArLog:
        writeArLog(job);
        break;
    }
}

void OutputWorker::applyConfig(const Config& config) {
    const bool sp3Changed = _config.sp3Enabled != config.sp3Enabled ||
                            _config.sp3Directory != config.sp3Directory ||
                            _config.sp3SampleSec != config.sp3SampleSec;
    const bool rinexChanged = _config.rinexEnabled != config.rinexEnabled ||
                              _config.rinexDirectory != config.rinexDirectory;
    if (sp3Changed) {
        _sp3Writer.close();
        _sp3Writer.setOutputDirectory(config.sp3Enabled ? config.sp3Directory : QString());
        _sp3Writer.setSampleInterval(config.sp3SampleSec);
    }
    if (rinexChanged) {
        _rinexWriter.close();
        _rinexWriter.setOutputDirectory(config.rinexEnabled ? config.rinexDirectory : QString());
    }
    if (_config.pppJsonEnabled != config.pppJsonEnabled ||
        _config.pppJsonDirectory != config.pppJsonDirectory ||
        _config.pppJsonReorderBufferMs != config.pppJsonReorderBufferMs ||
        _config.pppJsonFlushIntervalMs != config.pppJsonFlushIntervalMs) {
        PppJsonWriter::Options options;
        options.enabled = config.pppJsonEnabled;
        options.outputDirectory = config.pppJsonDirectory;
        options.reorderBufferMs = config.pppJsonReorderBufferMs;
        options.flushIntervalMs = config.pppJsonFlushIntervalMs;
        QString error;
        if (!_pppJsonWriter.setOptions(options, &error)) {
            reportError(QStringLiteral("PPP JSON"), error);
        }
    }
    _ztdDirectory = config.ztdDirectory;
    _logFilePath = config.logFilePath;
    _arLogFilePath = config.arLogFilePath;
    if (!_telemetryWriter) {
        _telemetryWriter = new TelemetryWriter();
    }
    _telemetryWriter->setOutputDirectory(config.telemetryDirectory);
    _telemetryWriter->setSourceNames(config.telemetrySourceNames);
    _config = config;
}

void OutputWorker::writeZtd(const Job& job) {
    if (_ztdDirectory.trimmed().isEmpty() || !job.solution.valid ||
        !std::isfinite(job.solution.ztd)) {
        return;
    }
    QDir dir(_ztdDirectory);
    if (!dir.exists() && !dir.mkpath(QStringLiteral("."))) {
        reportError(QStringLiteral("ZTD"), QStringLiteral("cannot create output directory"));
        return;
    }
    const QString station = safeStationName(job.station);
    const QString path = dir.filePath(QStringLiteral("ztd_%1.txt").arg(station));
    const bool newFile = !QFile::exists(path);
    QFile file(path);
    if (!file.open(QIODevice::WriteOnly | QIODevice::Text | QIODevice::Append)) {
        reportError(QStringLiteral("ZTD"), QStringLiteral("cannot open %1").arg(path));
        return;
    }
    QTextStream ts(&file);
    if (newFile) {
        ts << "# RT-PPP ZTD series\n";
        ts << "# Columns: GPS ISO8601, station, source, ztd_m, ztd_sigma_m, zwd_m, sat, hdop\n";
    }
    ts << gpsOutputTime(job.solution).toString(Qt::ISODateWithMs) << ","
       << job.station << ","
       << job.source << ","
       << number(job.solution.ztd) << ","
       << number(job.solution.ztdStdev) << ","
       << number(job.solution.trp) << ","
       << job.solution.numSat << ","
       << QString::number(job.solution.hDop, 'f', 3) << "\n";
}

void OutputWorker::writeLog(const Job& job) {
    if (_logFilePath.trimmed().isEmpty() || job.logText.isEmpty()) {
        return;
    }
    const QFileInfo info(_logFilePath);
    const QDir parent = info.dir();
    if (!parent.exists() && !parent.mkpath(QStringLiteral("."))) {
        reportError(QStringLiteral("LOG"), QStringLiteral("cannot create output directory"));
        return;
    }
    QFile file(_logFilePath);
    if (!file.open(QIODevice::WriteOnly | QIODevice::Append | QIODevice::Text)) {
        reportError(QStringLiteral("LOG"), QStringLiteral("cannot open %1").arg(_logFilePath));
        return;
    }
    QTextStream out(&file);
    out << job.logText << '\n';
}

void OutputWorker::writeArLog(const Job& job) {
    if (_arLogFilePath.trimmed().isEmpty() || job.solution.arLogLines.isEmpty()) {
        return;
    }
    const QFileInfo info(_arLogFilePath);
    const QDir parent = info.dir();
    if (!parent.exists() && !parent.mkpath(QStringLiteral("."))) {
        reportError(QStringLiteral("PPP-AR LOG"),
                    QStringLiteral("cannot create output directory"));
        return;
    }
    QFile file(_arLogFilePath);
    if (!file.open(QIODevice::WriteOnly | QIODevice::Append | QIODevice::Text)) {
        reportError(QStringLiteral("PPP-AR LOG"),
                    QStringLiteral("cannot open %1").arg(_arLogFilePath));
        return;
    }
    QTextStream out(&file);
    for (const QString& line : job.solution.arLogLines) {
        out << gpsOutputTime(job.solution).toString(QStringLiteral("yyyy-MM-dd HH:mm:ss.zzz"))
            << ' ' << line << '\n';
    }
}

void OutputWorker::shutdown() {
    _accepting.storeRelease(0);
    finalizePppJson();
    if (_telemetryWriter) {
        _telemetryWriter->close();
        delete _telemetryWriter;
        _telemetryWriter = nullptr;
    }
    _rinexWriter.close();
    _sp3Writer.close();
}

void OutputWorker::finalizePppJson() {
    for (;;) {
        QVector<Job> batch;
        {
            QMutexLocker locker(&_mutex);
            if (_jobs.isEmpty()) {
                break;
            }
            batch.swap(_jobs);
        }
        for (const Job& job : batch) {
            process(job);
        }
    }
    QString error;
    if (!_pppJsonWriter.close(&error)) {
        reportError(QStringLiteral("PPP JSON"), error);
    }
}

void OutputWorker::reportError(const QString& family, const QString& error) const {
    if (!error.isEmpty()) {
        qWarning().noquote() << family << error;
    }
}
