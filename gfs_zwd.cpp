// Loads GFS atmospheric profiles and derives zenith wet-delay priors.
#include "gfs_zwd.h"

#include <QCoreApplication>
#include <QDebug>
#include <QDir>
#include <QFileInfo>
#include <QMetaObject>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QProcess>
#include <QProcessEnvironment>
#include <QElapsedTimer>
#include <QMutexLocker>
#include <QThread>

#include <algorithm>
#include <cmath>
#include <functional>
#include <limits>

namespace {

constexpr double kK2PrimePa = 0.221; // K/Pa
constexpr double kK3Pa = 3739.0;     // K^2/Pa
constexpr double kPi = 3.14159265358979323846;

void setError(QString* target, const QString& value) {
    if (target) *target = value;
}

using ProgressCallback = std::function<void(const QString&)>;

void reportProgress(const ProgressCallback& callback, const QString& message) {
    qInfo().noquote() << message;
    if (callback) callback(message);
}

bool finiteProfileLevel(const GfsProfileLevel& level) {
    return std::isfinite(level.pressurePa) && level.pressurePa > 0.0 &&
           std::isfinite(level.temperatureK) && level.temperatureK > 100.0 &&
           std::isfinite(level.specificHumidity) && level.specificHumidity >= 0.0 &&
           std::isfinite(level.geopotentialHeightM);
}

bool cancelled(const GfsZwdRequest& request) {
    return request.cancelToken && request.cancelToken->loadAcquire() != 0;
}

double wetRefractivity(const GfsProfileLevel& level) {
    if (!finiteProfileLevel(level)) return std::numeric_limits<double>::quiet_NaN();
    const double ePa = level.specificHumidity * level.pressurePa /
                       (0.622 + 0.378 * level.specificHumidity);
    const double nWet = kK2PrimePa * ePa / level.temperatureK +
                        kK3Pa * ePa / (level.temperatureK * level.temperatureK);
    return std::isfinite(nWet) && nWet >= 0.0 ? nWet : std::numeric_limits<double>::quiet_NaN();
}

QStringList workerScriptCandidates() {
    QStringList candidates;
    const QString overridePath = qEnvironmentVariable("RTPPP_GFS_WORKER").trimmed();
    if (!overridePath.isEmpty()) candidates << overridePath;
    const QDir appDir(QCoreApplication::applicationDirPath());
    candidates << appDir.filePath(QStringLiteral("gfs_python/gfs_worker.py"));
    candidates << appDir.filePath(QStringLiteral("../../../gfs_python/gfs_worker.py"));
    candidates << QDir::current().filePath(QStringLiteral("gfs_python/gfs_worker.py"));
    return candidates;
}

QString workerScriptPath() {
    for (const QString& candidate : workerScriptCandidates()) {
        const QFileInfo info(candidate);
        if (info.isFile()) return info.absoluteFilePath();
    }
    return QString();
}

QStringList interpreterCandidates(const QString& scriptPath) {
    QStringList candidates;
    const QString overridePath = qEnvironmentVariable("RTPPP_GFS_PYTHON").trimmed();
    if (!overridePath.isEmpty()) candidates << overridePath;
    const QDir runtimeDir(QFileInfo(scriptPath).absolutePath());
#ifdef Q_OS_WIN
    candidates << runtimeDir.filePath(QStringLiteral(".venv/Scripts/python.exe"));
#else
    candidates << runtimeDir.filePath(QStringLiteral(".venv/bin/python"));
#endif
    // The fallback is useful for diagnostics and development. Deployment
    // should run bootstrap.ps1/bootstrap.sh and use the private venv above.
    candidates << QStringLiteral("python");
#ifdef Q_OS_UNIX
    candidates << QStringLiteral("python3");
#endif
    return candidates;
}

QString workerInterpreter(const QString& scriptPath) {
    for (const QString& candidate : interpreterCandidates(scriptPath)) {
        if (candidate == QLatin1String("python") || candidate == QLatin1String("python3")) {
            return candidate;
        }
        if (QFileInfo(candidate).isFile()) return QFileInfo(candidate).absoluteFilePath();
    }
    return QString();
}

bool parseProfile(const QJsonObject& root, GfsAtmosphericProfile& profile, QString* error) {
    const QJsonObject json = root.value(QStringLiteral("profile")).toObject();
    if (json.isEmpty()) {
        setError(error, QStringLiteral("GFS Python response has no profile"));
        return false;
    }
    profile = GfsAtmosphericProfile();
    profile.validTime = QDateTime::fromString(json.value(QStringLiteral("validTime")).toString(), Qt::ISODate);
    profile.productTime = QDateTime::fromString(json.value(QStringLiteral("productTime")).toString(), Qt::ISODate);
    profile.cycle = json.value(QStringLiteral("cycle")).toString();
    profile.forecastLeadSec = json.value(QStringLiteral("forecastLeadSec")).toInt(-1);
    profile.productSelection = root.value(QStringLiteral("selection")).toString();
    profile.fallbackUsed = root.value(QStringLiteral("fallbackUsed")).toBool(false);
    profile.latitudeDeg = json.value(QStringLiteral("latitudeDeg")).toDouble();
    profile.longitudeDeg = json.value(QStringLiteral("longitudeDeg")).toDouble();
    profile.spatialInterpolated = json.value(QStringLiteral("spatialInterpolated")).toBool(false);
    const QJsonArray levels = json.value(QStringLiteral("levels")).toArray();
    for (const QJsonValue& value : levels) {
        const QJsonObject item = value.toObject();
        GfsProfileLevel level;
        level.pressurePa = item.value(QStringLiteral("pressurePa")).toDouble();
        level.temperatureK = item.value(QStringLiteral("temperatureK")).toDouble();
        level.specificHumidity = item.value(QStringLiteral("specificHumidity")).toDouble();
        level.geopotentialHeightM = item.value(QStringLiteral("geopotentialHeightM")).toDouble();
        if (finiteProfileLevel(level)) profile.levels.push_back(level);
    }
    if (profile.levels.size() < 2) {
        setError(error, QStringLiteral("GFS Python response has fewer than two valid profile levels"));
        return false;
    }
    return true;
}

bool runPythonWorker(const GfsZwdRequest& request, GfsAtmosphericProfile& profile,
                     QString* error, const ProgressCallback& progress = ProgressCallback()) {
    if (cancelled(request)) {
        setError(error, QStringLiteral("GFS request cancelled"));
        return false;
    }
    const QString script = workerScriptPath();
    if (script.isEmpty()) {
        setError(error, QStringLiteral("GFS Python worker not found; expected gfs_python/gfs_worker.py"));
        qInfo().noquote() << QStringLiteral("GFS Python worker unavailable: script not found");
        return false;
    }
    const QString interpreter = workerInterpreter(script);
    if (interpreter.isEmpty()) {
        setError(error, QStringLiteral("Python interpreter not found for GFS worker"));
        qInfo().noquote() << QStringLiteral("GFS Python worker unavailable: interpreter not found");
        return false;
    }

    QJsonObject input;
    input[QStringLiteral("requestedUtc")] = request.requestedUtc.toUTC().toString(Qt::ISODate);
    input[QStringLiteral("latitudeDeg")] = request.latitudeDeg;
    input[QStringLiteral("longitudeDeg")] = request.longitudeDeg;
    input[QStringLiteral("receiverHeightM")] = request.receiverHeightM;
    input[QStringLiteral("cacheDirectory")] = request.cacheDirectory.trimmed().isEmpty()
        ? QString()
        : QDir(request.cacheDirectory).absolutePath();
    input[QStringLiteral("downloadBaseUrl")] = request.downloadBaseUrl;
    input[QStringLiteral("timeoutSec")] = request.timeoutSec;
    input[QStringLiteral("maxAgeSec")] = request.maxAgeSec;

    QProcess process;
    process.setProgram(interpreter);
    process.setArguments({script, QStringLiteral("--request"), QStringLiteral("-")});
    process.setWorkingDirectory(QFileInfo(script).absolutePath());
    QProcessEnvironment environment = QProcessEnvironment::systemEnvironment();
    environment.insert(QStringLiteral("PYTHONUTF8"), QStringLiteral("1"));
    process.setProcessEnvironment(environment);
    reportProgress(progress,
                   QStringLiteral("GFS Python worker start: interpreter=%1 script=%2 validUtc=%3")
                       .arg(interpreter, script, request.requestedUtc.toUTC().toString(Qt::ISODate)));
    QElapsedTimer elapsed;
    elapsed.start();
    process.start();
    if (!process.waitForStarted(5000)) {
        setError(error, QStringLiteral("cannot start GFS Python worker: %1").arg(process.errorString()));
        return false;
    }
    const QByteArray requestBytes = QJsonDocument(input).toJson(QJsonDocument::Compact);
    process.write(requestBytes);
    process.closeWriteChannel();
    // The worker may try the nominal cycle and two older fallback cycles.
    // timeoutSec applies to each HTTP attempt, not to the whole worker.
    const int perAttemptMs = qMax(30, request.timeoutSec) * 1000;
    // Each attempt has a socket inactivity budget, followed by GRIB2 decode
    // and profile construction.  The old formula left only 30 seconds for
    // decoding and killed otherwise valid 50-60 MB filtered products.
    const int decodeBudgetMs = 180000;
    const int waitMs = qBound(60000, perAttemptMs * 3 + decodeBudgetMs,
                              15 * 60 * 1000);
    reportProgress(progress,
                   QStringLiteral("GFS Python worker wait budget: perAttemptMs=%1 maxAttempts=3 totalLimitMs=%2")
                       .arg(perAttemptMs)
                       .arg(waitMs));
    QElapsedTimer waitTimer;
    waitTimer.start();
    bool finished = false;
    QByteArray stderrBytes;
    QByteArray stderrLine;
    qint64 lastHeartbeatMs = 0;
    auto consumeStderr = [&](bool flushPartial) {
        const QByteArray chunk = process.readAllStandardError();
        if (!chunk.isEmpty()) {
            stderrBytes += chunk;
            stderrLine += chunk;
        }
        for (;;) {
            const int lineEnd = stderrLine.indexOf('\n');
            if (lineEnd < 0) break;
            const QString line = QString::fromUtf8(stderrLine.left(lineEnd)).trimmed();
            stderrLine.remove(0, lineEnd + 1);
            if (!line.isEmpty()) reportProgress(progress, line);
        }
        if (flushPartial && !stderrLine.trimmed().isEmpty()) {
            reportProgress(progress, QString::fromUtf8(stderrLine).trimmed());
            stderrLine.clear();
        }
    };
    while (waitTimer.elapsed() < waitMs) {
        const int remainingMs = qMax(1, waitMs - static_cast<int>(waitTimer.elapsed()));
        if (process.waitForFinished(qMin(250, remainingMs))) {
            finished = true;
            consumeStderr(true);
            break;
        }
        consumeStderr(false);
        if (waitTimer.elapsed() - lastHeartbeatMs >= 5000) {
            lastHeartbeatMs = waitTimer.elapsed();
            reportProgress(progress,
                           QStringLiteral("GFS Python worker still running: elapsedSec=%1 limitSec=%2")
                               .arg(lastHeartbeatMs / 1000)
                               .arg(waitMs / 1000));
        }
        if (cancelled(request)) {
            process.kill();
            process.waitForFinished(2000);
            consumeStderr(true);
            setError(error, QStringLiteral("GFS request cancelled"));
            return false;
        }
    }
    if (!finished) {
        const QByteArray stdoutBeforeKill = process.readAllStandardOutput();
        consumeStderr(false);
        process.kill();
        process.waitForFinished(2000);
        const QByteArray stdoutAfterKill = process.readAllStandardOutput();
        consumeStderr(true);
        const QString diagnostics = QString::fromUtf8(stderrBytes).trimmed();
        const QString stdoutText = QString::fromUtf8(stdoutBeforeKill + stdoutAfterKill).trimmed();
        QString timeoutLog = QStringLiteral("GFS Python worker timeout: elapsedMs=%1 limitMs=%2 state=%3 exitCode=%4 exitStatus=%5 termination=timeout-kill")
                                 .arg(elapsed.elapsed())
                                 .arg(waitMs)
                                 .arg(static_cast<int>(process.state()))
                                 .arg(process.exitCode())
                                 .arg(static_cast<int>(process.exitStatus()));
        timeoutLog += QStringLiteral(" processError=") + process.errorString();
        timeoutLog += QStringLiteral(" stderr=") +
                      (diagnostics.isEmpty() ? QStringLiteral("-") : diagnostics);
        timeoutLog += QStringLiteral(" stdout=") +
                      (stdoutText.isEmpty() ? QStringLiteral("-") : stdoutText);
        reportProgress(progress, timeoutLog);
        setError(error, QStringLiteral("GFS Python worker timed out after %1 ms (%2)")
                         .arg(waitMs)
                         .arg(diagnostics.isEmpty() ? process.errorString() : diagnostics));
        return false;
    }
    const QByteArray stdoutBytes = process.readAllStandardOutput();
    consumeStderr(true);
    QJsonParseError parseError;
    const QJsonDocument response = QJsonDocument::fromJson(stdoutBytes, &parseError);
    if (!response.isObject()) {
        setError(error, QStringLiteral("GFS Python returned invalid JSON: %1").arg(parseError.errorString()));
        return false;
    }
    const QJsonObject root = response.object();
    if (!root.value(QStringLiteral("ok")).toBool(false)) {
        setError(error, root.value(QStringLiteral("error")).toString(
                     QStringLiteral("GFS Python worker failed with exit code %1").arg(process.exitCode())));
        return false;
    }
    if (!parseProfile(root, profile, error)) return false;
    reportProgress(progress,
                   QStringLiteral("GFS Python worker complete: source=%1 selection=%2 fallback=%3 product=%4+f%5 cache=%6 levels=%7")
                       .arg(root.value(QStringLiteral("source")).toString())
                       .arg(root.value(QStringLiteral("selection")).toString())
                       .arg(root.value(QStringLiteral("fallbackUsed")).toBool(false) ? QStringLiteral("yes") : QStringLiteral("no"))
                       .arg(root.value(QStringLiteral("productCycle")).toString())
                       .arg(root.value(QStringLiteral("forecastLeadSec")).toInt() / 3600, 3, 10, QLatin1Char('0'))
                        .arg(root.value(QStringLiteral("cachePath")).toString())
                        .arg(profile.levels.size()));
    return true;
}

bool priorForProduct(const GfsZwdRequest& request, const QDateTime& validTime,
                     GfsZwdMethod method, PppTropospherePrior& prior, QString* error,
                     const ProgressCallback& progress = ProgressCallback()) {
    if (cancelled(request)) {
        setError(error, QStringLiteral("GFS request cancelled"));
        return false;
    }
    GfsAtmosphericProfile profile;
    GfsZwdRequest productRequest = request;
    productRequest.requestedUtc = validTime;
    if (!runPythonWorker(productRequest, profile, error, progress)) return false;
    prior = GfsZwdProvider::integrateProfile(profile, request.receiverHeightM, method);
    if (!prior.valid) {
        setError(error, prior.reason);
        return false;
    }
    return true;
}

} // namespace

bool GfsZwdProvider::ecefToGeodetic(const std::array<double, 3>& ecef,
                                    double& latitudeDeg, double& longitudeDeg,
                                    double& heightM) {
    const double x = ecef[0];
    const double y = ecef[1];
    const double z = ecef[2];
    const double a = 6378137.0;
    const double f = 1.0 / 298.257223563;
    const double b = a * (1.0 - f);
    const double e2 = f * (2.0 - f);
    const double p = std::hypot(x, y);
    if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z) ||
        p < 1.0 || std::hypot(p, z) < 5.0e6) return false;
    longitudeDeg = std::atan2(y, x) * 180.0 / kPi;
    double latitude = std::atan2(z, p * (1.0 - e2));
    for (int i = 0; i < 12; ++i) {
        const double sinLat = std::sin(latitude);
        const double n = a / std::sqrt(1.0 - e2 * sinLat * sinLat);
        const double next = std::atan2(z + e2 * n * sinLat, p);
        if (std::fabs(next - latitude) < 1.0e-13) {
            latitude = next;
            break;
        }
        latitude = next;
    }
    const double sinLat = std::sin(latitude);
    const double n = a / std::sqrt(1.0 - e2 * sinLat * sinLat);
    if (std::fabs(std::cos(latitude)) < 1.0e-12) heightM = std::fabs(z) - b;
    else heightM = p / std::cos(latitude) - n;
    latitudeDeg = latitude * 180.0 / kPi;
    return std::isfinite(latitudeDeg) && std::isfinite(longitudeDeg) &&
           std::isfinite(heightM) && latitudeDeg >= -90.0 && latitudeDeg <= 90.0;
}

bool GfsZwdProvider::pythonWorkerAvailable() {
    const QString script = workerScriptPath();
    return !script.isEmpty() && !workerInterpreter(script).isEmpty();
}

PppTropospherePrior GfsZwdProvider::integrateProfile(const GfsAtmosphericProfile& profile,
                                                      double receiverHeightM,
                                                      GfsZwdMethod method) {
    PppTropospherePrior prior;
    prior.source = QStringLiteral("GFS %1").arg(method == GfsZwdMethod::ProfileIntegral
                                                       ? QStringLiteral("profile integral")
                                                       : QStringLiteral("Askne-Nordius diagnostic"));
    prior.productTime = profile.productTime;
    prior.cycle = profile.cycle;
    prior.forecastLeadSec = profile.forecastLeadSec;
    prior.productSelection = profile.productSelection;
    prior.fallbackUsed = profile.fallbackUsed;
    prior.spatialInterpolated = profile.spatialInterpolated;
    if (!std::isfinite(receiverHeightM)) {
        prior.reason = QStringLiteral("receiver height is not finite");
        prior.status = QStringLiteral("invalid");
        return prior;
    }
    if (method != GfsZwdMethod::ProfileIntegral) {
        prior.reason = QStringLiteral("Askne-Nordius path is diagnostic only; profile integration is required for PPP initialization");
        prior.status = QStringLiteral("unsupported_method");
        return prior;
    }
    QVector<GfsProfileLevel> levels;
    levels.reserve(profile.levels.size());
    for (const GfsProfileLevel& level : profile.levels) {
        if (finiteProfileLevel(level)) levels.push_back(level);
    }
    std::sort(levels.begin(), levels.end(), [](const GfsProfileLevel& left, const GfsProfileLevel& right) {
        return left.geopotentialHeightM < right.geopotentialHeightM;
    });
    if (levels.size() < 2) {
        prior.reason = QStringLiteral("GFS profile has fewer than two valid levels");
        prior.status = QStringLiteral("invalid_profile");
        return prior;
    }
    QVector<double> heights;
    QVector<double> refractivity;
    for (const GfsProfileLevel& level : levels) {
        const double nWet = wetRefractivity(level);
        if (!std::isfinite(nWet)) continue;
        heights.push_back(level.geopotentialHeightM);
        refractivity.push_back(nWet);
    }
    if (heights.size() < 2) {
        prior.reason = QStringLiteral("GFS profile has no finite wet refractivity");
        prior.status = QStringLiteral("invalid_profile");
        return prior;
    }
    double zwd = 0.0;
    int start = 0;
    double startHeight = receiverHeightM;
    double startN = refractivity.first();
    if (receiverHeightM <= heights.first()) {
        zwd += 1.0e-6 * refractivity.first() * (heights.first() - receiverHeightM);
        prior.heightExtrapolated = receiverHeightM < heights.first();
        startHeight = heights.first();
        startN = refractivity.first();
    } else if (receiverHeightM >= heights.last()) {
        prior.reason = QStringLiteral("receiver is above the highest usable GFS pressure level");
        prior.status = QStringLiteral("invalid_profile");
        return prior;
    } else {
        while (start + 1 < heights.size() && heights.at(start + 1) < receiverHeightM) ++start;
        const double dz = heights.at(start + 1) - heights.at(start);
        const double fraction = dz > 0.0 ? (receiverHeightM - heights.at(start)) / dz : 0.0;
        startN = refractivity.at(start) + fraction * (refractivity.at(start + 1) - refractivity.at(start));
    }
    for (int i = start; i + 1 < heights.size(); ++i) {
        const double lowerH = i == start ? startHeight : heights.at(i);
        const double lowerN = i == start ? startN : refractivity.at(i);
        const double upperH = heights.at(i + 1);
        const double upperN = refractivity.at(i + 1);
        if (upperH > lowerH) zwd += 1.0e-6 * 0.5 * (lowerN + upperN) * (upperH - lowerH);
    }
    if (!std::isfinite(zwd) || zwd < 0.0 || zwd > 2.0) {
        prior.reason = QStringLiteral("GFS integrated ZWD is outside physical bounds");
        prior.status = QStringLiteral("invalid_profile");
        return prior;
    }
    prior.valid = true;
    prior.zwdM = zwd;
    prior.status = profile.fallbackUsed
        ? QStringLiteral("gfs_profile_integral_fallback_cycle")
        : QStringLiteral("gfs_profile_integral");
    return prior;
}

bool fetchPriorImpl(const GfsZwdRequest& request, GfsZwdMethod method,
                    PppTropospherePrior& prior, QString* error,
                    const ProgressCallback& progress = ProgressCallback()) {
    reportProgress(progress,
                   QStringLiteral("GFS fetch request: pythonWorker=%1 requestedUtc=%2 lat=%3 lon=%4 heightM=%5 cache=%6 timeoutSec=%7 maxAgeSec=%8 method=%9")
                       .arg(GfsZwdProvider::pythonWorkerAvailable() ? QStringLiteral("available") : QStringLiteral("unavailable"))
                       .arg(request.requestedUtc.toUTC().toString(Qt::ISODate))
                       .arg(request.latitudeDeg, 0, 'f', 5)
                       .arg(request.longitudeDeg, 0, 'f', 5)
                       .arg(request.receiverHeightM, 0, 'f', 2)
                       .arg(request.cacheDirectory)
                       .arg(request.timeoutSec)
                       .arg(request.maxAgeSec)
                       .arg(method == GfsZwdMethod::ProfileIntegral ? QStringLiteral("profile") : QStringLiteral("askne")));
    if (!request.requestedUtc.isValid() || !std::isfinite(request.latitudeDeg) ||
        !std::isfinite(request.longitudeDeg)) {
        setError(error, QStringLiteral("GFS request requires valid UTC, latitude, and longitude"));
        return false;
    }
    QDateTime lowerTime = request.requestedUtc.toUTC();
    lowerTime.setTime(QTime(lowerTime.time().hour(), 0, 0, 0));
    PppTropospherePrior first;
    if (!priorForProduct(request, lowerTime, method, first, error, progress)) return false;
    if (cancelled(request)) {
        setError(error, QStringLiteral("GFS request cancelled"));
        return false;
    }
    const qint64 fractionalMillis = lowerTime.msecsTo(request.requestedUtc.toUTC());
    if (fractionalMillis <= 0) {
        prior = first;
        return true;
    }
    PppTropospherePrior second;
    QString upperError;
    reportProgress(progress, QStringLiteral("GFS temporal interpolation: requesting upper-hour profile"));
    if (!priorForProduct(request, lowerTime.addSecs(3600), method, second, &upperError, progress)) {
        prior = first;
        prior.status = QStringLiteral("gfs_profile_integral_time_nearest");
        prior.reason = upperError;
        return true;
    }
    const double fraction = qBound(0.0, fractionalMillis / 3600000.0, 1.0);
    prior = first;
    prior.zwdM = first.zwdM + fraction * (second.zwdM - first.zwdM);
    prior.temporalInterpolated = true;
    prior.status = QStringLiteral("gfs_profile_integral_time_interpolated");
    return true;
}

bool GfsZwdProvider::fetchPrior(const GfsZwdRequest& request, GfsZwdMethod method,
                                 PppTropospherePrior& prior, QString* error) {
    return fetchPriorImpl(request, method, prior, error);
}

class GfsZwdAsyncRunner : public QObject {
public slots:
    void run(const QString& sourceId, quint64 requestId,
             GfsZwdRequest request, GfsZwdMethod method) {
        PppTropospherePrior prior;
        QString error;
        const bool ok = fetchPriorImpl(
            request, method, prior, &error,
            [this, sourceId, requestId](const QString& message) {
                if (_progress) _progress(sourceId, requestId, message);
            });
        if (_callback) {
            _callback(sourceId, requestId, ok, prior, error);
        }
    }

    std::function<void(const QString&, quint64, bool,
                       const PppTropospherePrior&, const QString&)> _callback;
    std::function<void(const QString&, quint64, const QString&)> _progress;
};

GfsZwdAsyncService::GfsZwdAsyncService(QObject* parent)
    : QObject(parent),
      _thread(new QThread(this)),
      _runner(new GfsZwdAsyncRunner()) {
    qRegisterMetaType<GfsZwdRequest>("GfsZwdRequest");
    qRegisterMetaType<GfsZwdMethod>("GfsZwdMethod");
    qRegisterMetaType<PppTropospherePrior>("PppTropospherePrior");
    _runner->moveToThread(_thread);
    connect(_thread, &QThread::finished, _runner, &QObject::deleteLater);
    static_cast<GfsZwdAsyncRunner*>(_runner)->_callback =
        [this](const QString& sourceId, quint64 completedId, bool ok,
               const PppTropospherePrior& prior, const QString& error) {
            QMetaObject::invokeMethod(this,
                [this, sourceId, completedId, ok, prior, error]() {
                    handleRunnerFinished(sourceId, completedId, ok, prior, error);
                }, Qt::QueuedConnection);
        };
    static_cast<GfsZwdAsyncRunner*>(_runner)->_progress =
        [this](const QString& sourceId, quint64 completedId, const QString& message) {
            QMetaObject::invokeMethod(this,
                [this, sourceId, completedId, message]() {
                    handleRunnerProgress(sourceId, completedId, message);
                }, Qt::QueuedConnection);
        };
    _thread->start();
}

GfsZwdAsyncService::~GfsZwdAsyncService() {
    cancelAll();
    _thread->quit();
    _thread->wait();
    _thread = nullptr;
    _runner = nullptr;
}

void GfsZwdAsyncService::start(const QString& sourceId, quint64 requestId,
                               const GfsZwdRequest& request, GfsZwdMethod method) {
    const QSharedPointer<QAtomicInt> token(new QAtomicInt(0));
    {
        QMutexLocker locker(&_mutex);
        _tokens.insert(requestId, token);
    }
    GfsZwdRequest asyncRequest = request;
    asyncRequest.cancelToken = token;
    GfsZwdAsyncRunner* runner = static_cast<GfsZwdAsyncRunner*>(_runner);
    QMetaObject::invokeMethod(runner,
        [runner, sourceId, requestId, asyncRequest, method]() {
            runner->run(sourceId, requestId, asyncRequest, method);
        }, Qt::QueuedConnection);
}

void GfsZwdAsyncService::handleRunnerFinished(const QString& sourceId, quint64 requestId,
                                              bool ok, const PppTropospherePrior& prior,
                                              const QString& error) {
    {
        QMutexLocker locker(&_mutex);
        _tokens.remove(requestId);
    }
    emit completed(sourceId, requestId, ok, prior, error);
}

void GfsZwdAsyncService::handleRunnerProgress(const QString& sourceId, quint64 requestId,
                                              const QString& message) {
    QMutexLocker locker(&_mutex);
    if (!_tokens.contains(requestId)) return;
    locker.unlock();
    emit progress(sourceId, requestId, message);
}

void GfsZwdAsyncService::cancel(const QString&, quint64 requestId) {
    QMutexLocker locker(&_mutex);
    const auto it = _tokens.constFind(requestId);
    if (it != _tokens.constEnd() && !it.value().isNull()) {
        it.value()->storeRelease(1);
    }
}

void GfsZwdAsyncService::cancelAll() {
    QMutexLocker locker(&_mutex);
    for (auto it = _tokens.constBegin(); it != _tokens.constEnd(); ++it) {
        if (!it.value().isNull()) {
            it.value()->storeRelease(1);
        }
    }
}
