// Checks GFS profile loading and zenith wet-delay derivation.
#include "gfs_zwd.h"

#include <QCoreApplication>
#include <QDateTime>
#include <QEventLoop>
#include <QTimer>

#include <cmath>
#include <iostream>

namespace {

bool check(bool value, const char* message) {
    if (!value) std::cerr << message << '\n';
    return value;
}

} // namespace

int main(int argc, char** argv) {
    QCoreApplication application(argc, argv);
    GfsAtmosphericProfile profile;
    profile.productTime = QDateTime(QDate(2026, 9, 2), QTime(0, 0), Qt::UTC);
    profile.cycle = QStringLiteral("2026090200");
    profile.forecastLeadSec = 3 * 3600;
    profile.spatialInterpolated = true;
    // A deliberately simple, physically reasonable moist profile. The result
    // should be millimetres to decimetres, never a pressure-unit-scale number.
    profile.levels = {
        {100000.0, 290.0, 0.0100, 100.0},
        { 90000.0, 282.0, 0.0060, 1000.0},
        { 80000.0, 274.0, 0.0030, 2000.0},
        { 70000.0, 266.0, 0.0010, 3200.0}
    };
    const PppTropospherePrior prior = GfsZwdProvider::integrateProfile(profile, 250.0);
    if (!check(prior.valid, "profile integration should succeed") ||
        !check(prior.zwdM > 0.01 && prior.zwdM < 1.0, "ZWD magnitude is not physical") ||
        !check(prior.spatialInterpolated, "spatial interpolation flag lost") ||
        !check(prior.cycle == profile.cycle, "product cycle lost")) return 1;

    const PppTropospherePrior high = GfsZwdProvider::integrateProfile(profile, 4000.0);
    if (!check(!high.valid, "above-profile receiver must be rejected")) return 1;

    const PppTropospherePrior diagnostic = GfsZwdProvider::integrateProfile(
        profile, 250.0, GfsZwdMethod::AskneNordius);
    if (!check(!diagnostic.valid, "unvalidated Askne-Nordius path must not produce a prior")) return 1;

    const std::array<double, 3> ecef = {6378137.0, 0.0, 0.0};
    double latitudeDeg = 0.0;
    double longitudeDeg = 0.0;
    double heightM = 0.0;
    if (!check(GfsZwdProvider::ecefToGeodetic(ecef, latitudeDeg, longitudeDeg, heightM),
               "ECEF conversion should succeed") ||
        !check(std::fabs(latitudeDeg) < 1.0e-9 && std::fabs(longitudeDeg) < 1.0e-9 &&
               std::fabs(heightM) < 1.0e-3, "ECEF conversion is inaccurate")) return 1;

    // Exercise the asynchronous path without network or Python dependencies:
    // validation fails in the worker and the completion must return to this
    // event loop before the service is destroyed.
    GfsZwdAsyncService asyncService;
    GfsZwdRequest invalidRequest;
    bool asyncCompleted = false;
    bool asyncOk = true;
    QString asyncError;
    QEventLoop loop;
    QTimer timeout;
    timeout.setSingleShot(true);
    QObject::connect(&asyncService, &GfsZwdAsyncService::completed,
                     &loop, [&](const QString&, quint64, bool ok,
                                const PppTropospherePrior&, const QString& error) {
        asyncCompleted = true;
        asyncOk = ok;
        asyncError = error;
        loop.quit();
    });
    QObject::connect(&timeout, &QTimer::timeout, &loop, &QEventLoop::quit);
    asyncService.start(QStringLiteral("SMOKE"), 1, invalidRequest,
                       GfsZwdMethod::ProfileIntegral);
    timeout.start(3000);
    loop.exec();
    if (!check(asyncCompleted && !asyncOk && !asyncError.isEmpty(),
               "asynchronous GFS completion was not delivered")) return 1;

    std::cout << "gfs_zwd smoke test passed\n";
    return 0;
}
