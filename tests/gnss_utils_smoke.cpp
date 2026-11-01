// Checks GNSS time conversion, coordinate helpers, and shared RTCM semantics.
#include "../gnss_utils.h"
#include "../rtcmdecoder.h"

#include <QCoreApplication>

#include <cmath>
#include <cstdlib>
#include <iostream>

namespace {

bool closeTo(double actual, double expected, double tolerance = 1.0e-6) {
    return std::fabs(actual - expected) <= tolerance;
}

bool check(bool condition, const char* message) {
    if (!condition) {
        std::cerr << "gnss_utils smoke test failed: " << message << "\n";
    }
    return condition;
}

} // namespace

int main(int argc, char* argv[]) {
    QCoreApplication app(argc, argv);

    const GnssUtils::GpsTime previousWeek = GnssUtils::normalizeGpsTime(2200, -0.5);
    if (!check(previousWeek.week == 2199 && closeTo(previousWeek.sow, 604799.5),
               "negative SOW should normalize into the previous GPS week")) {
        return 1;
    }

    const GnssUtils::GpsTime nextWeek = GnssUtils::normalizeGpsTime(2200, 604800.5);
    if (!check(nextWeek.week == 2201 && closeTo(nextWeek.sow, 0.5),
               "overflow SOW should normalize into the next GPS week")) {
        return 1;
    }

    const GnssUtils::GpsTime first = GnssUtils::normalizeGpsTime(2200, 604799.75);
    const GnssUtils::GpsTime second = GnssUtils::normalizeGpsTime(2201, 0.25);
    if (!check(closeTo(GnssUtils::gpsTimeDifferenceSeconds(second, first), 0.5),
               "GPS time difference should cross a week boundary")) {
        return 1;
    }

    for (int secondOffset = 0; secondOffset < 10; ++secondOffset) {
        const GnssUtils::GpsTime sampleTime =
            GnssUtils::normalizeGpsTime(2200, 100.0 + secondOffset);
        const bool expected = secondOffset == 0 || secondOffset == 5;
        if (!check(GnssUtils::isGpsTimeOnSampleGrid(sampleTime, 5) == expected,
                   "5-second sampling must reject intermediate 1-second epochs")) {
            return 1;
        }
    }
    if (!check(!GnssUtils::isGpsTimeOnSampleGrid(
                   GnssUtils::normalizeGpsTime(2200, 604799.0), 5) &&
                   GnssUtils::isGpsTimeOnSampleGrid(
                       GnssUtils::normalizeGpsTime(2201, 0.0), 5),
               "5-second sampling grid should remain aligned across a GPS week boundary")) {
        return 1;
    }

    const QDateTime reference(QDate(2026, 8, 25), QTime(11, 59, 57), Qt::UTC);
    const GnssUtils::GpsTime referenceGps = GnssUtils::currentGpsTime(reference);
    if (!check(std::llabs(reference.msecsTo(referenceGps.utc)) <= 1,
               "UTC to GPS week/SOW conversion should round-trip")) {
        return 1;
    }
    const QDateTime gpsCalendar = GnssUtils::gpsCalendarDateTime(referenceGps.week, referenceGps.sow);
    if (!check(gpsCalendar == reference.addSecs(18) &&
                   gpsCalendar.time().second() % 5 == 0,
               "GPS calendar output must preserve the 5-second GPS epoch grid")) {
        return 1;
    }

    const QDateTime beforeLeap(QDate(2016, 12, 31), QTime(23, 59, 59), Qt::UTC);
    const QDateTime afterLeap(QDate(2017, 1, 1), QTime(0, 0, 0), Qt::UTC);
    const GnssUtils::GpsTime beforeLeapGps = GnssUtils::currentGpsTime(beforeLeap);
    const GnssUtils::GpsTime afterLeapGps = GnssUtils::currentGpsTime(afterLeap);
    if (!check(closeTo(GnssUtils::gpsTimeDifferenceSeconds(afterLeapGps, beforeLeapGps), 2.0) &&
                   std::llabs(beforeLeap.msecsTo(beforeLeapGps.utc)) <= 1 &&
                   std::llabs(afterLeap.msecsTo(afterLeapGps.utc)) <= 1,
               "the 2016 leap second should be represented symmetrically")) {
        return 1;
    }

    const int epochMilliseconds = 345678000;
    const int epochSeconds = epochMilliseconds / 1000;

    RtcmMsmMessage gpsMsm;
    gpsMsm.system = QLatin1Char('G');
    gpsMsm.epochTime = epochMilliseconds;
    RtcmSsrMessage gpsSsr;
    gpsSsr.system = QLatin1Char('G');
    gpsSsr.epochTime = epochSeconds;
    const GnssUtils::GpsTime gpsMsmTime = GnssUtils::gpsTimeFromMsm(gpsMsm, reference);
    const GnssUtils::GpsTime gpsSsrTime = GnssUtils::gpsTimeFromSsr('G', epochSeconds, reference);
    if (!check(gpsMsmTime.valid && gpsSsrTime.valid &&
                   gpsMsmTime.week == gpsSsrTime.week &&
                   closeTo(gpsMsmTime.sow, gpsSsrTime.sow, 1.0e-3),
               "GPS MSM and SSR epochs should use the same week/SOW convention")) {
        return 1;
    }

    const GnssUtils::GpsTime bdsMsmTime = [&]() {
        RtcmMsmMessage bdsMsm = gpsMsm;
        bdsMsm.system = QLatin1Char('C');
        return GnssUtils::gpsTimeFromMsm(bdsMsm, reference);
    }();
    const GnssUtils::GpsTime bdsSsrTime = GnssUtils::gpsTimeFromSsr('C', epochSeconds, reference);
    if (!check(closeTo(bdsMsmTime.sow - gpsMsmTime.sow, 14.0, 1.0e-3) &&
                   closeTo(bdsSsrTime.sow - gpsSsrTime.sow, 14.0, 1.0e-3),
               "BDS conversion should apply the GPS/BDS 14 second offset")) {
        return 1;
    }

    const GnssUtils::GpsTime bdsWeekBoundary =
        GnssUtils::gpsTimeFromSsr('C', 604790, reference);
    if (!check(bdsWeekBoundary.week == referenceGps.week + 1 &&
                   closeTo(bdsWeekBoundary.sow, 4.0),
               "BDS offset should normalize across the GPS week boundary")) {
        return 1;
    }

    RtcmMsmMessage gloMsm;
    gloMsm.system = QLatin1Char('R');
    gloMsm.epochTime = 43200000;
    const GnssUtils::GpsTime gloMsmTime = GnssUtils::gpsTimeFromMsm(gloMsm, reference);
    const GnssUtils::GpsTime gloSsrTime = GnssUtils::gpsTimeFromSsr('R', 43200, reference);
    if (!check(gloMsmTime.valid && gloSsrTime.valid &&
                   gloMsmTime.week == gloSsrTime.week &&
                   closeTo(gloMsmTime.sow, gloSsrTime.sow, 1.0e-3),
               "GLONASS MSM and SSR epochs should share the reference day conversion")) {
        return 1;
    }

    RtcmMsmMessage gloDayStart = gloMsm;
    gloDayStart.epochTime = 0;
    RtcmMsmMessage gloDayEnd = gloMsm;
    gloDayEnd.epochTime = 86399999;
    const double gloDaySpan = GnssUtils::gpsTimeDifferenceSeconds(
        GnssUtils::gpsTimeFromMsm(gloDayEnd, reference),
        GnssUtils::gpsTimeFromMsm(gloDayStart, reference));
    if (!check(closeTo(gloDaySpan, 86399.999, 1.0e-3),
               "GLONASS day boundary should preserve the millisecond epoch")) {
        return 1;
    }

    std::cout << "gnss_utils smoke test passed\n";
    return 0;
}
