// Declares shared GNSS time, coordinate, satellite, and geodetic utilities.
#pragma once

#include <array>
#include <QChar>
#include <QDateTime>
#include <QVector>

struct RtcmMsmMessage;

namespace GnssUtils {

constexpr double kPi = 3.1415926535897932384626433832795;
constexpr double kC = 299792458.0;
constexpr double kOmegaEarth = 7292115.1467e-11;
constexpr double kOmegaBds = 7292115.0000e-11;
constexpr double kAell = 6378137.000;
constexpr double kFInv = 298.2572236;

double ssrUpdateIntervalSeconds(int index);

int gnumleap(int year, int month, int day);
void updatetime(int* week, int* secOfWeek, int msecOfWeek, bool fixnumleap);

double gpjd(double second, int week);
void jdgp(double mjd, double& second, long& week);
void mjdFromDateTime(const QDateTime& dateTime, int& mjd, double& dayfrac);
void gpsWeekFromDateTime(const QDateTime& dateTime, int& week, double& sec);
// Calendar representation of GPS week/SOW. Its Qt UTC spec is only a neutral
// container; callers must label the displayed value as GPS, not UTC.
QDateTime gpsCalendarDateTime(int week, double sec);
QDateTime dateTimeFromGpsWeek(int week, double sec);

struct GpsTime {
    int week = 0;
    double sow = 0.0;
    QDateTime utc;
    bool valid = false;
};

GpsTime normalizeGpsTime(int week, double sow);
GpsTime currentGpsTime(const QDateTime& referenceUtc = QDateTime());
GpsTime gpsTimeFromMsm(const RtcmMsmMessage& msm,
                       const QDateTime& referenceUtc = QDateTime());
GpsTime gpsTimeFromSsr(QChar system,
                       int epochTime,
                       const QDateTime& referenceUtc = QDateTime());
double gpsTimeDifferenceSeconds(const GpsTime& first, const GpsTime& second);
bool isGpsTimeOnSampleGrid(const GpsTime& time, int sampleSec);

std::array<double,3> cross(const std::array<double,3>& a, const std::array<double,3>& b);
double dot(const std::array<double,3>& a, const std::array<double,3>& b);
double norm(const std::array<double,3>& a);

void rswToXyz(const std::array<double,3>& rr,
              const std::array<double,3>& vv,
              const std::array<double,3>& rsw,
              std::array<double,3>& xyz);

void xyzToRsw(const std::array<double,3>& rr,
              const std::array<double,3>& vv,
              const std::array<double,3>& xyz,
              std::array<double,3>& rsw);

} // namespace GnssUtils
