// Interpolates GPT3/VMF3 grids for tropospheric mapping and delay parameters.
#include "gpt3_vmf3.h"

#include "gnss_utils.h"

#include <QCoreApplication>
#include <QDir>
#include <QElapsedTimer>
#include <QFileInfo>
#include <QProcess>
#include <QtGlobal>

#include <cmath>

namespace {

constexpr int kStartupTimeoutMs = 3000;
constexpr int kRequestTimeoutMs = 3000;

QString nativeWorkerName() {
#ifdef Q_OS_WIN
    return QStringLiteral("gpt3_vmf3_worker.exe");
#else
    return QStringLiteral("gpt3_vmf3_worker");
#endif
}

bool finitePositive(double value) {
    return std::isfinite(value) && value > 0.0;
}

QString workerFailureDetail(QProcess& process) {
    const QString stderrText = QString::fromLocal8Bit(process.readAllStandardError()).trimmed();
    return stderrText.isEmpty() ? process.errorString() : stderrText;
}

} // namespace

Gpt3Vmf3Worker::Gpt3Vmf3Worker() = default;

Gpt3Vmf3Worker::~Gpt3Vmf3Worker() {
    stop();
}

QString Gpt3Vmf3Worker::findWorker() {
    QStringList candidates;
    const QString overridePath = qEnvironmentVariable("RTPPP_GPT3_WORKER").trimmed();
    if (!overridePath.isEmpty()) candidates << overridePath;

    const QDir appDir(QCoreApplication::applicationDirPath());
    const QString name = nativeWorkerName();
    candidates << appDir.filePath(QStringLiteral("gpt3/") + name);
    candidates << appDir.filePath(QStringLiteral("../../../gpt3/") + name);
    candidates << appDir.filePath(QStringLiteral("../../../../gpt3/") + name);
    candidates << appDir.filePath(QStringLiteral("../gpt3/") + name);
    candidates << QDir::current().filePath(QStringLiteral("gpt3/") + name);

    for (const QString& candidate : candidates) {
        const QFileInfo workerInfo(candidate);
        if (!workerInfo.isFile()) continue;
        const QFileInfo gridInfo(workerInfo.dir().filePath(QStringLiteral("gpt3_5.grd")));
        if (gridInfo.isFile()) return workerInfo.absoluteFilePath();
    }
    return QString();
}

void Gpt3Vmf3Worker::setError(QString* error, const QString& value) const {
    if (error) *error = value;
}

bool Gpt3Vmf3Worker::ensureStarted(QString* error) {
    if (_process && _process->state() == QProcess::Running) return true;

    stop();
    _workerPath = findWorker();
    if (_workerPath.isEmpty()) {
        setError(error, QStringLiteral("GPT3/VMF3 worker or gpt3_5.grd is not deployed; run gpt3/build_worker for this application output directory"));
        return false;
    }

    _process = std::make_unique<QProcess>();
    _process->setProcessChannelMode(QProcess::SeparateChannels);
    _process->setWorkingDirectory(QFileInfo(_workerPath).absolutePath());
    _process->start(_workerPath);
    if (!_process->waitForStarted(kStartupTimeoutMs)) {
        setError(error, QStringLiteral("GPT3/VMF3 worker failed to start: %1").arg(workerFailureDetail(*_process)));
        stop();
        return false;
    }
    return true;
}

bool Gpt3Vmf3Worker::mapping(const QDateTime& utc, double latitudeRad, double longitudeRad,
                              double ellipsoidalHeightM, double zenithDistanceRad,
                              Gpt3Vmf3Mapping& result, QString* error) {
    result = Gpt3Vmf3Mapping();
    if (!utc.isValid() || !std::isfinite(latitudeRad) || !std::isfinite(longitudeRad) ||
        !std::isfinite(ellipsoidalHeightM) || !std::isfinite(zenithDistanceRad) ||
        zenithDistanceRad < 0.0 || zenithDistanceRad >= 1.5707963267948966) {
        setError(error, QStringLiteral("GPT3/VMF3 request has invalid UTC, receiver coordinates, or elevation"));
        return false;
    }
    if (!ensureStarted(error)) return false;

    int mjdDay = 0;
    double mjdFraction = 0.0;
    GnssUtils::mjdFromDateTime(utc.toUTC(), mjdDay, mjdFraction);
    const double mjd = static_cast<double>(mjdDay) + mjdFraction;
    const QString request = QStringLiteral("MAP %1 %2 %3 %4 %5\n")
        .arg(mjd, 0, 'g', 17)
        .arg(latitudeRad, 0, 'g', 17)
        .arg(longitudeRad, 0, 'g', 17)
        .arg(ellipsoidalHeightM, 0, 'g', 17)
        .arg(zenithDistanceRad, 0, 'g', 17);
    _process->write(request.toLatin1());
    if (!_process->waitForBytesWritten(kRequestTimeoutMs)) {
        setError(error, QStringLiteral("GPT3/VMF3 worker write timed out: %1").arg(workerFailureDetail(*_process)));
        stop();
        return false;
    }

    QElapsedTimer timer;
    timer.start();
    QByteArray response;
    while (timer.elapsed() < kRequestTimeoutMs) {
        if (_process->canReadLine()) {
            response = _process->readLine().trimmed();
            break;
        }
        if (!_process->waitForReadyRead(qMax(1, kRequestTimeoutMs - static_cast<int>(timer.elapsed())))) break;
    }
    if (response.isEmpty()) {
        setError(error, QStringLiteral("GPT3/VMF3 worker response timed out: %1").arg(workerFailureDetail(*_process)));
        stop();
        return false;
    }

    const QStringList fields = QString::fromLatin1(response).split(QLatin1Char(' '), QString::SkipEmptyParts);
    if (fields.size() < 4 || fields.at(0) != QLatin1String("OK")) {
        setError(error, QStringLiteral("GPT3/VMF3 worker rejected request: %1").arg(QString::fromLatin1(response)));
        return false;
    }
    bool hydroOk = false;
    bool wetOk = false;
    bool tmOk = false;
    result.hydrostatic = fields.at(1).toDouble(&hydroOk);
    result.wet = fields.at(2).toDouble(&wetOk);
    result.meanWaterVaporTemperatureK = fields.at(3).toDouble(&tmOk);
    if (!hydroOk || !wetOk || !tmOk || !finitePositive(result.hydrostatic) ||
        !finitePositive(result.wet) || !finitePositive(result.meanWaterVaporTemperatureK)) {
        setError(error, QStringLiteral("GPT3/VMF3 worker returned invalid mapping values: %1").arg(QString::fromLatin1(response)));
        return false;
    }
    return true;
}

void Gpt3Vmf3Worker::stop() {
    if (!_process) return;
    if (_process->state() != QProcess::NotRunning) {
        _process->write("QUIT\n");
        _process->waitForBytesWritten(100);
        if (!_process->waitForFinished(500)) {
            _process->terminate();
            if (!_process->waitForFinished(500)) _process->kill();
        }
    }
    _process.reset();
}
