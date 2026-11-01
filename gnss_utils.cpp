// Implements shared GNSS time conversion, coordinate transforms, and identifiers.
#include "gnss_utils.h"

#include <cmath>

#include "rtcmdecoder.h"

namespace {

struct LeapSecond {
    int day;
    int month;
    int year;
    int taicount;
};

static const LeapSecond kLeapTable[] = {
    {30, 6, 1981, 20},
    {30, 6, 1982, 21},
    {30, 6, 1983, 22},
    {30, 6, 1985, 23},
    {31, 12, 1987, 24},
    {31, 12, 1989, 25},
    {31, 12, 1990, 26},
    {30, 6, 1992, 27},
    {30, 6, 1993, 28},
    {30, 6, 1994, 29},
    {31, 12, 1995, 30},
    {30, 6, 1997, 31},
    {31, 12, 1998, 32},
    {31, 12, 2005, 33},
    {31, 12, 2008, 34},
    {30, 6, 2012, 35},
    {30, 6, 2015, 36},
    {31, 12, 2016, 37},
    {0, 0, 0, 0}
};

constexpr int kGpsLeapStart = 19;
constexpr int kMonths[] = {0,31,28,31,30,31,30,31,31,30,31,30,31};

int longyear(int year, int month) {
    if (!(year % 4) && (!(year % 400) || (year % 100))) {
        if (!month || month == 2) {
            return 1;
        }
    }
    return 0;
}

double djul(int year, int month, double tt) {
    int yy = year;
    int mm = month;
    if (mm <= 2) {
        yy -= 1;
        mm += 12;
    }
    const long ii = yy / 100;
    const long kk = 2 - ii + ii / 4;
    double mjd = (365.25 * yy - std::fmod(365.25 * yy, 1.0)) - 679006.0;
    mjd = mjd + std::floor(30.6001 * (mm + 1)) + tt + kk;
    return mjd;
}

} // namespace

namespace GnssUtils {

double ssrUpdateIntervalSeconds(int index) {
    static const QVector<int> table = {
        1, 2, 5, 10, 15, 30, 60, 120, 240, 300, 600, 900, 1800, 3600, 7200, 10800
    };
    if (index >= 0 && index < table.size()) {
        return static_cast<double>(table[index]);
    }
    return 0.0;
}

int gnumleap(int year, int month, int day) {
    int ls = 0;
    for (const LeapSecond* it = kLeapTable; it->taicount != 0; ++it) {
        if (year >= it->year) {
            if (year > it->year || month > it->month || (month == it->month && day > it->day)) {
                ls = it->taicount - kGpsLeapStart;
            }
        }
    }
    return ls;
}

void updatetime(int* week, int* secOfWeek, int msecOfWeek, bool fixnumleap) {
    int y, m, d, k, l, nul;
    unsigned int j = (*week) * (7 * 24 * 60 * 60) + (*secOfWeek) + 5 * 24 * 60 * 60 + 3 * 60 * 60;
    int gloTimeOfDay;
    for (y = 1980; j >= static_cast<unsigned int>((k = (l = (365 + longyear(y, 0))) * 24 * 60 * 60) + gnumleap(y + 1, 1, 1)); ++y) {
        j -= k;
    }
    for (m = 1; j >= static_cast<unsigned int>((k = (l = kMonths[m] + longyear(y, m)) * 24 * 60 * 60) + gnumleap(y, m + 1, 1)); ++m) {
        j -= k;
    }
    for (d = 1; j >= static_cast<unsigned int>(24UL * 60UL * 60UL + gnumleap(y, m, d + 1)); ++d) {
        j -= 24 * 60 * 60;
    }
    nul = gnumleap(y, m, d);
    gloTimeOfDay = static_cast<int>(j) - nul;

    if (msecOfWeek < 4 * 60 * 60 * 1000 && gloTimeOfDay > 20 * 60 * 60) {
        *secOfWeek += 24 * 60 * 60;
    } else if (gloTimeOfDay < 4 * 60 * 60 && msecOfWeek > 20 * 60 * 60 * 1000) {
        *secOfWeek -= 24 * 60 * 60;
    }

    *secOfWeek += msecOfWeek / 1000 - gloTimeOfDay;
    if (fixnumleap) {
        *secOfWeek -= nul;
    }
    if (*secOfWeek < 0) {
        *secOfWeek += 24 * 60 * 60 * 7;
        --(*week);
    }
    if (*secOfWeek >= 24 * 60 * 60 * 7) {
        *secOfWeek -= 24 * 60 * 60 * 7;
        ++(*week);
    }
}

double gpjd(double second, int week) {
    return 44244.0 + week * 7.0 + second / 86400.0;
}

void jdgp(double mjd, double& second, long& week) {
    const double deltat = mjd - 44244.0;
    week = static_cast<long>(std::floor(deltat / 7.0));
    second = (deltat - week * 7.0) * 86400.0;
}

void mjdFromDateTime(const QDateTime& dateTime, int& mjd, double& dayfrac) {
    static const QDate zeroDate(1858, 11, 17);
    mjd = zeroDate.daysTo(dateTime.date());
    dayfrac = (dateTime.time().hour() +
               (dateTime.time().minute() +
                (dateTime.time().second() + dateTime.time().msec() / 1000.0) / 60.0) / 60.0) / 24.0;
}

void gpsWeekFromDateTime(const QDateTime& dateTime, int& week, double& sec) {
    const int leap = gnumleap(dateTime.date().year(), dateTime.date().month(), dateTime.date().day());
    const QDateTime gpsDateTime = dateTime.addSecs(leap).toUTC();
    const double mjd = djul(gpsDateTime.date().year(),
                            gpsDateTime.date().month(),
                            gpsDateTime.date().day()) +
                       (gpsDateTime.time().hour() * 3600.0 +
                        gpsDateTime.time().minute() * 60.0 +
                        gpsDateTime.time().second() +
                        gpsDateTime.time().msec() / 1000.0) / 86400.0;
    long weekLong = 0;
    jdgp(mjd, sec, weekLong);
    week = static_cast<int>(weekLong);
}

QDateTime gpsCalendarDateTime(int week, double sec) {
    const double mjd = gpjd(sec, week);
    const int wholeMjd = static_cast<int>(std::floor(mjd));
    const double frac = mjd - wholeMjd;
    static const QDate zeroDate(1858, 11, 17);
    const QDate date = zeroDate.addDays(wholeMjd);
    const int ms = static_cast<int>(std::round(frac * 86400.0 * 1000.0));
    const QTime time(0, 0, 0, 0);
    return QDateTime(date, time, Qt::UTC).addMSecs(ms);
}

QDateTime dateTimeFromGpsWeek(int week, double sec) {
    const QDateTime gpsDateTime = gpsCalendarDateTime(week, sec);
    QDateTime utc = gpsDateTime;
    for (int i = 0; i < 3; ++i) {
        const int leap = gnumleap(utc.date().year(), utc.date().month(), utc.date().day());
        const QDateTime adjusted = gpsDateTime.addSecs(-leap);
        if (adjusted == utc) {
            break;
        }
        utc = adjusted;
    }
    return utc;
}

GpsTime normalizeGpsTime(int week, double sow) {
    while (sow < 0.0) {
        sow += 604800.0;
        --week;
    }
    while (sow >= 604800.0) {
        sow -= 604800.0;
        ++week;
    }
    GpsTime result;
    result.week = week;
    result.sow = sow;
    result.utc = dateTimeFromGpsWeek(week, sow).toUTC();
    result.valid = result.utc.isValid();
    return result;
}

GpsTime currentGpsTime(const QDateTime& referenceUtc) {
    const QDateTime utc = referenceUtc.isValid()
        ? referenceUtc.toUTC()
        : QDateTime::currentDateTimeUtc();
    int week = 0;
    double sow = 0.0;
    gpsWeekFromDateTime(utc, week, sow);
    return normalizeGpsTime(week, sow);
}

GpsTime gpsTimeFromMsm(const RtcmMsmMessage& msm, const QDateTime& referenceUtc) {
    GpsTime reference = currentGpsTime(referenceUtc);
    double sow = static_cast<double>(msm.epochTime) / 1000.0;
    if (msm.system == QLatin1Char('R')) {
        int week = reference.week;
        int secOfWeek = static_cast<int>(std::floor(reference.sow));
        updatetime(&week, &secOfWeek, msm.epochTime, false);
        sow = static_cast<double>(secOfWeek) +
              static_cast<double>(msm.epochTime % 1000) / 1000.0;
        return normalizeGpsTime(week, sow);
    }
    if (msm.system == QLatin1Char('C')) {
        sow += 14.0;
    }
    return normalizeGpsTime(reference.week, sow);
}

GpsTime gpsTimeFromSsr(QChar system, int epochTime, const QDateTime& referenceUtc) {
    GpsTime reference = currentGpsTime(referenceUtc);
    if (system == QLatin1Char('R')) {
        int week = reference.week;
        int secOfWeek = static_cast<int>(std::floor(reference.sow));
        updatetime(&week, &secOfWeek, epochTime * 1000, false);
        return normalizeGpsTime(week, static_cast<double>(secOfWeek));
    }
    double sow = static_cast<double>(epochTime);
    if (system == QLatin1Char('C')) {
        sow += 14.0;
    }
    return normalizeGpsTime(reference.week, sow);
}

double gpsTimeDifferenceSeconds(const GpsTime& first, const GpsTime& second) {
    return static_cast<double>(first.week - second.week) * 604800.0 +
           (first.sow - second.sow);
}

bool isGpsTimeOnSampleGrid(const GpsTime& time, int sampleSec) {
    if (sampleSec <= 1) {
        return true;
    }
    if (!time.valid || !std::isfinite(time.sow)) {
        return false;
    }
    const qint64 intervalMs = static_cast<qint64>(sampleSec) * 1000LL;
    const qint64 epochMs = static_cast<qint64>(time.week) * 604800000LL +
                           static_cast<qint64>(std::llround(time.sow * 1000.0));
    const qint64 remainder = ((epochMs % intervalMs) + intervalMs) % intervalMs;
    return remainder == 0;
}

std::array<double,3> cross(const std::array<double,3>& a, const std::array<double,3>& b) {
    return {
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0]
    };
}

double dot(const std::array<double,3>& a, const std::array<double,3>& b) {
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}

double norm(const std::array<double,3>& a) {
    return std::sqrt(dot(a, a));
}

void rswToXyz(const std::array<double,3>& rr,
              const std::array<double,3>& vv,
              const std::array<double,3>& rsw,
              std::array<double,3>& xyz) {
    const double vNorm = norm(vv);
    if (vNorm <= 0.0) {
        xyz = rsw;
        return;
    }

    std::array<double,3> along = {vv[0] / vNorm, vv[1] / vNorm, vv[2] / vNorm};
    std::array<double,3> crossV = cross(rr, vv);
    const double cNorm = norm(crossV);
    if (cNorm <= 0.0) {
        xyz = rsw;
        return;
    }
    crossV = {crossV[0] / cNorm, crossV[1] / cNorm, crossV[2] / cNorm};
    const std::array<double,3> radial = cross(along, crossV);

    xyz[0] = radial[0] * rsw[0] + along[0] * rsw[1] + crossV[0] * rsw[2];
    xyz[1] = radial[1] * rsw[0] + along[1] * rsw[1] + crossV[1] * rsw[2];
    xyz[2] = radial[2] * rsw[0] + along[2] * rsw[1] + crossV[2] * rsw[2];
}

void xyzToRsw(const std::array<double,3>& rr,
              const std::array<double,3>& vv,
              const std::array<double,3>& xyz,
              std::array<double,3>& rsw) {
    const double vNorm = norm(vv);
    if (vNorm <= 0.0) {
        rsw = xyz;
        return;
    }
    std::array<double,3> along = {vv[0] / vNorm, vv[1] / vNorm, vv[2] / vNorm};
    std::array<double,3> crossV = cross(rr, vv);
    const double cNorm = norm(crossV);
    if (cNorm <= 0.0) {
        rsw = xyz;
        return;
    }
    crossV = {crossV[0] / cNorm, crossV[1] / cNorm, crossV[2] / cNorm};
    const std::array<double,3> radial = cross(along, crossV);

    rsw[0] = dot(xyz, radial);
    rsw[1] = dot(xyz, along);
    rsw[2] = dot(xyz, crossV);
}

} // namespace GnssUtils
