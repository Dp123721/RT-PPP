// Declares GFS-based zenith wet-delay initialization and its worker interface.
#pragma once

#include "ppp_engine.h"

#include <QDateTime>
#include <QAtomicInt>
#include <QMap>
#include <QMutex>
#include <QObject>
#include <QString>
#include <QSharedPointer>
#include <QThread>
#include <QVector>

#include <array>

struct GfsProfileLevel {
    double pressurePa = 0.0;
    double temperatureK = 0.0;
    double specificHumidity = 0.0;
    double geopotentialHeightM = 0.0;
};

struct GfsAtmosphericProfile {
    QDateTime validTime;
    QDateTime productTime;
    QString cycle;
    int forecastLeadSec = -1;
    QString productSelection;
    bool fallbackUsed = false;
    double latitudeDeg = 0.0;
    double longitudeDeg = 0.0;
    QVector<GfsProfileLevel> levels;
    bool spatialInterpolated = false;
};

struct GfsZwdRequest {
    QDateTime requestedUtc;
    double latitudeDeg = 0.0;
    double longitudeDeg = 0.0;
    double receiverHeightM = 0.0;
    QString cacheDirectory;
    QString downloadBaseUrl;
    // Socket inactivity timeout.  NOMADS may spend time generating a filtered
    // GRIB2 response before sending the first byte, so this is deliberately
    // independent from the C++ worker's total process budget.
    int timeoutSec = 90;
    int maxAgeSec = 4 * 3600;
    // Set by the asynchronous fetch service. The synchronous provider keeps
    // this optional so existing callers retain their original behavior.
    QSharedPointer<QAtomicInt> cancelToken;
};

Q_DECLARE_METATYPE(GfsZwdRequest)
Q_DECLARE_METATYPE(PppTropospherePrior)

// GFS I/O and GRIB2 decoding run in the isolated gfs_python worker. The C++
// side keeps the PPP-facing profile and ZWD integration contract.
class GfsZwdProvider {
public:
    static bool ecefToGeodetic(const std::array<double, 3>& ecef,
                               double& latitudeDeg, double& longitudeDeg,
                               double& heightM);
    static bool pythonWorkerAvailable();
    static PppTropospherePrior integrateProfile(const GfsAtmosphericProfile& profile,
                                                double receiverHeightM,
                                                GfsZwdMethod method = GfsZwdMethod::ProfileIntegral);
    static bool fetchPrior(const GfsZwdRequest& request,
                           GfsZwdMethod method,
                           PppTropospherePrior& prior,
                           QString* error = nullptr);
};

// Runs the existing isolated Python/GRIB path outside the PPP worker thread.
// The service owns one serialized fetch thread; callers can cancel requests
// without allowing observation processing to block behind QProcess.
class GfsZwdAsyncService : public QObject {
    Q_OBJECT

public:
    explicit GfsZwdAsyncService(QObject* parent = nullptr);
    ~GfsZwdAsyncService() override;

    void start(const QString& sourceId, quint64 requestId,
               const GfsZwdRequest& request, GfsZwdMethod method);
    void cancel(const QString& sourceId, quint64 requestId);
    void cancelAll();

signals:
    void completed(const QString& sourceId, quint64 requestId, bool ok,
                   const PppTropospherePrior& prior, const QString& error);
    void progress(const QString& sourceId, quint64 requestId, const QString& message);

private:
    void handleRunnerFinished(const QString& sourceId, quint64 requestId, bool ok,
                               const PppTropospherePrior& prior, const QString& error);
    void handleRunnerProgress(const QString& sourceId, quint64 requestId,
                              const QString& message);
    QThread* _thread = nullptr;
    QObject* _runner = nullptr;
    QMutex _mutex;
    QMap<quint64, QSharedPointer<QAtomicInt>> _tokens;
};
