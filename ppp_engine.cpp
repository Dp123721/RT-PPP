// PppEngine owns PPP state estimation, measurement updates, ambiguity resolution, and diagnostics.
#include "ppp_engine.h"

#include "gnss_signals.h"
#include "gnss_utils.h"
#include "ambbie.h"
#include "ambfix.h"
#include "gpt3_vmf3.h"

#include <QtMath>
#include <QDebug>
#include <QFile>
#include <QDir>
#include <QSet>
#include <QTextStream>
#include <algorithm>
#include <cmath>
#include <limits>
#include <sstream>

namespace {

constexpr double kPi = 3.1415926535897932384626433832795;
constexpr double kRhoDeg = 180.0 / kPi;
constexpr double kRhoSec = 3600.0 * kRhoDeg;
constexpr double kMjdJ2000 = 51544.5;
constexpr double kGloWeightFactor = 5.0;
constexpr double kBdsWeightFactor = 2.0;
constexpr double kFreqGpsL1Hz = 1575.42e6;
constexpr double kGmEarth = 3.986004418e14;
constexpr double kGeocentricEarthRadius = 6370000.0;

// Keep the numerical core compatible with older GCC/libstdc++ versions
// available on supported Ubuntu installations.
double clampValue(double value, double lower, double upper) {
    return value < lower ? lower : (value > upper ? upper : value);
}

struct PppTime {
    int gpsWeek = 0;
    double gpsSow = 0.0;
    QDateTime utc;
    bool valid = false;
};

enum class ObservationCombination {
    Dummy,
    C1,
    C2,
    CIf,
    L1,
    L2,
    LIf,
    Gim
};

QString combinationLabel(ObservationCombination lc) {
    switch (lc) {
    case ObservationCombination::C1: return QStringLiteral("c1");
    case ObservationCombination::C2: return QStringLiteral("c2");
    case ObservationCombination::CIf: return QStringLiteral("cIF");
    case ObservationCombination::L1: return QStringLiteral("l1");
    case ObservationCombination::L2: return QStringLiteral("l2");
    case ObservationCombination::LIf: return QStringLiteral("lIF");
    case ObservationCombination::Gim: return QStringLiteral("GIM");
    case ObservationCombination::Dummy:
    default: break;
    }
    return QString();
}

bool combinationContainsPhase(ObservationCombination lc) {
    return lc == ObservationCombination::L1 || lc == ObservationCombination::L2 || lc == ObservationCombination::LIf;
}

QString combinationModeForSystem(const PppOptions& opt, QChar system) {
    switch (system.toLatin1()) {
    case 'R': return opt.lcGlo;
    case 'E': return opt.lcGal;
    case 'C': return opt.lcBds;
    case 'G':
    default:
        return opt.lcGps;
    }
}

QVector<ObservationCombination> combinationsForSystem(const PppOptions& opt, QChar system) {
    QVector<ObservationCombination> lcs;
    if (!opt.useSystem(system)) {
        return lcs;
    }

    const QString mode = combinationModeForSystem(opt, system).trimmed();
    if (mode == QLatin1String("Pi&Li") || mode == QLatin1String("L1L2")) {
        lcs << ObservationCombination::C1 << ObservationCombination::C2 << ObservationCombination::L1 << ObservationCombination::L2;
        if (opt.pseudoObsIono) lcs << ObservationCombination::Gim;
    } else if (mode == QLatin1String("Pi")) {
        lcs << ObservationCombination::C1 << ObservationCombination::C2;
        if (opt.pseudoObsIono) lcs << ObservationCombination::Gim;
    } else if (mode == QLatin1String("P1&L1")) {
        lcs << ObservationCombination::C1 << ObservationCombination::L1;
        if (opt.pseudoObsIono) lcs << ObservationCombination::Gim;
    } else if (mode == QLatin1String("P1")) {
        lcs << ObservationCombination::C1;
        if (opt.pseudoObsIono) lcs << ObservationCombination::Gim;
    } else if (mode == QLatin1String("P3")) {
        lcs << ObservationCombination::CIf;
    } else if (mode == QLatin1String("L3")) {
        lcs << ObservationCombination::LIf;
    } else {
        lcs << ObservationCombination::CIf << ObservationCombination::LIf;
    }
    return lcs;
}

QVector<ObservationCombination> ambiguityCombinationsForSystem(const PppOptions& opt, QChar system) {
    QVector<ObservationCombination> phaseLCs;
    for (ObservationCombination lc : combinationsForSystem(opt, system)) {
        if (combinationContainsPhase(lc)) {
            phaseLCs << lc;
        }
    }

    QVector<ObservationCombination> ans;
    if (phaseLCs.size() == 1) {
        ans << phaseLCs.front();
    } else if (phaseLCs.size() > 1) {
        ans << ObservationCombination::L1 << ObservationCombination::L2;
    }
    return ans;
}

bool usesIonFreeCombination(const PppOptions& opt, QChar system) {
    for (ObservationCombination lc : combinationsForSystem(opt, system)) {
        if (lc == ObservationCombination::CIf || lc == ObservationCombination::LIf) {
            return true;
        }
    }
    return false;
}

bool frequencyBandsFromPriorities(const QString& signalPriorities, QChar sys, QChar& b1, QChar& b2) {
    const QStringList tokens = signalPriorities.split(QLatin1Char(' '), QString::SkipEmptyParts);
    for (const QString& token : tokens) {
        const int colon = token.indexOf(QLatin1Char(':'));
        const int amp = token.indexOf(QLatin1Char('&'), colon + 1);
        if (colon != 1 || amp < 0 || token.at(0) != sys) {
            continue;
        }
        const QString bands = token.mid(colon + 1, amp - colon - 1);
        for (const QChar& band : bands) {
            if (b1.isNull()) {
                b1 = band;
            } else if (band != b1 && b2.isNull()) {
                b2 = band;
            }
        }
    }
    if (!b1.isNull() && !b2.isNull()) return true;
    if (sys == QLatin1Char('G') || sys == QLatin1Char('R')) { b1 = QLatin1Char('1'); b2 = QLatin1Char('2'); return true; }
    if (sys == QLatin1Char('E')) { b1 = QLatin1Char('1'); b2 = QLatin1Char('5'); return true; }
    if (sys == QLatin1Char('C')) { b1 = QLatin1Char('2'); b2 = QLatin1Char('6'); return true; }
    return false;
}

PppTime timeFromGps(int week, double sow) {
    PppTime out;
    out.gpsWeek = week;
    out.gpsSow = sow;
    out.utc = GnssUtils::dateTimeFromGpsWeek(week, sow);
    out.valid = out.utc.isValid();
    return out;
}

double gpsSeconds(const PppTime& t) {
    return static_cast<double>(t.gpsWeek) * 604800.0 + t.gpsSow;
}

double diffSeconds(const PppTime& a, const PppTime& b) {
    return gpsSeconds(a) - gpsSeconds(b);
}

QString sourceSatKey(const QString& source, const QString& satKey) {
    // RTCM navigation message type is not part of satellite identity and can
    // differ between an MSM/broadcast source and an SSR source. Including it
    // in the key makes valid SSR corrections appear unavailable.
    const int separator = satKey.indexOf(QLatin1Char(':'));
    const QString prn = separator > 0 ? satKey.left(separator) : satKey;
    return QString("%1|%2").arg(source, prn);
}

// Internal product maps distinguish navigation type; task diagnostics do not.
QString diagnosticSatKey(const QString& satKey) {
    const int suffix = satKey.indexOf(QLatin1Char(':'));
    return suffix < 0 ? satKey : satKey.left(suffix);
}

QString sourceSatIodKey(const QString& source, const QString& satKey, int iod) {
    return QString("%1|%2|%3").arg(source, satKey).arg(iod);
}

template <typename Map>
void trimMapToSize(Map& map, int maxSize) {
    while (map.size() > maxSize) {
        map.erase(map.begin());
    }
}

qint64 gpsMsKey(int week, double sow) {
    return static_cast<qint64>(week) * 604800000LL +
           static_cast<qint64>(std::llround(sow * 1000.0));
}

qint64 gpsMsKey(const PppTime& time) {
    return gpsMsKey(time.gpsWeek, time.gpsSow);
}

int navTypeFromSystemAndSat(const QChar& system, int satNum) {
    switch (system.toLatin1()) {
    case 'G':
    case 'J':
    case 'I':
        return RtcmNavLnav;
    case 'R':
        return RtcmNavFdmaM;
    case 'E':
        return RtcmNavInav;
    case 'C':
        if (satNum < 6 || (satNum > 58 && satNum < 63)) {
            return RtcmNavD2;
        }
        return RtcmNavD1;
    case 'S':
        return RtcmNavSbasL1;
    default:
        break;
    }
    return RtcmNavUnknown;
}

double convertSsrEpochToGpsSow(const QChar& system, int epochTime, int gpsWeek, double gpsSow) {
    const GnssUtils::GpsTime converted = GnssUtils::gpsTimeFromSsr(
        system, epochTime, GnssUtils::dateTimeFromGpsWeek(gpsWeek, gpsSow));
    return converted.sow;
}

double ssrEpochToGpsSow(const QChar& system, int epochTime, int gpsWeek,
                        double gpsSow, bool epochTimeIsGps) {
    if (epochTimeIsGps) {
        double sow = static_cast<double>(epochTime);
        while (sow < 0.0) sow += 604800.0;
        while (sow >= 604800.0) sow -= 604800.0;
        return sow;
    }
    return convertSsrEpochToGpsSow(system, epochTime, gpsWeek, gpsSow);
}

double wrapTowDiff(double nowSow, double epochSow) {
    double dt = nowSow - epochSow;
    while (dt > 302400.0) dt -= 604800.0;
    while (dt < -302400.0) dt += 604800.0;
    return dt;
}

bool ssrOrbitClockOutOfRange(const RtcmSsrSatelliteData& sat, RtcmSsrCorrectionKind kind) {
    const auto inRange = [](double value, double minVal, double maxVal) {
        return std::isfinite(value) && value >= minVal && value <= maxVal;
    };
    if (kind == RtcmSsrCorrectionKind::Orbit || kind == RtcmSsrCorrectionKind::OrbitClock) {
        if (!inRange(sat.deltaRadial,     -209.7151,   209.7151)) return true;
        if (!inRange(sat.deltaAlongTrack, -209.7148,   209.7148)) return true;
        if (!inRange(sat.deltaCrossTrack, -209.7148,   209.7148)) return true;
        if (!inRange(sat.dotDeltaRadial,    -1.048575,   1.048575)) return true;
        if (!inRange(sat.dotDeltaAlongTrack,-1.048572,   1.048572)) return true;
        if (!inRange(sat.dotDeltaCrossTrack,-1.048572,   1.048572)) return true;
    }
    if (kind == RtcmSsrCorrectionKind::Clock || kind == RtcmSsrCorrectionKind::OrbitClock) {
        if (!inRange(sat.deltaC0, -209.7151,    209.7151)) return true;
        if (!inRange(sat.deltaC1,   -1.048575,    1.048575)) return true;
        if (!inRange(sat.deltaC2,   -1.34217726,  1.34217726)) return true;
    }
    if (kind == RtcmSsrCorrectionKind::HighRateClock) {
        if (!inRange(sat.highRateClock, -209.7151, 209.7151)) return true;
    }
    return false;
}

bool outDatedBcepLike(const GnssEphemeris& eph, const PppTime& tt) {
    int tocWeek = 0;
    int toeWeek = 0;
    double tocSow = 0.0;
    double toeSow = 0.0;
    if (!eph.computeReferenceTimes(tt.gpsWeek, tt.gpsSow, tocWeek, tocSow, toeWeek, toeSow)) {
        return true;
    }
    (void)toeWeek;
    (void)toeSow;
    const double dt = static_cast<double>(tt.gpsWeek - tocWeek) * 604800.0 + (tt.gpsSow - tocSow);
    switch (eph.type) {
    case GnssEphemeris::Type::Gps:
        return (dt > 14400.0 || dt < -7200.0);
    case GnssEphemeris::Type::Galileo:
        return (dt > 14400.0 || dt < 0.0);
    case GnssEphemeris::Type::Glonass:
        return (dt > 2100.0 || dt < -2100.0);
    case GnssEphemeris::Type::Beidou:
        return (dt > 3900.0 || dt < 0.0);
    default:
        break;
    }
    return true;
}

double msmEpochToGpsSow(const RtcmMsmMessage& msm, int& gpsWeek, double nowSow) {
    (void)nowSow;
    const GnssUtils::GpsTime converted = GnssUtils::gpsTimeFromMsm(
        msm, GnssUtils::dateTimeFromGpsWeek(gpsWeek, nowSow));
    gpsWeek = converted.week;
    return converted.sow;
}

qint64 gpsEpochToMsKey(int gpsWeek, double gpsSow) {
    int sowMs = static_cast<int>(std::llround(gpsSow * 1000.0));
    constexpr int kWeekMs = 604800000;
    while (sowMs >= kWeekMs) {
        sowMs -= kWeekMs;
        ++gpsWeek;
    }
    while (sowMs < 0) {
        sowMs += kWeekMs;
        --gpsWeek;
    }
    return static_cast<qint64>(gpsWeek) * static_cast<qint64>(kWeekMs) + static_cast<qint64>(sowMs);
}

void mergeMsmEpoch(RtcmMsmMessage& dst, const RtcmMsmMessage& src) {
    if (dst.system.isNull()) {
        dst = src;
        return;
    }
    dst.messageType = src.messageType;
    dst.msmSubtype = src.msmSubtype;
    dst.stationId = src.stationId;
    dst.epochTime = src.epochTime;
    dst.glonassDay = src.glonassDay;
    dst.multipleMessage = src.multipleMessage;
    dst.iods = src.iods;
    dst.clockSteeringIndicator = src.clockSteeringIndicator;
    dst.externalClockIndicator = src.externalClockIndicator;
    dst.smoothingIndicator = src.smoothingIndicator;
    dst.smoothingInterval = src.smoothingInterval;
    dst.gpsTimeValid = src.gpsTimeValid;
    dst.gpsWeek = src.gpsWeek;
    dst.gpsSow = src.gpsSow;

    QSet<int> sigSet;
    for (const int id : dst.signalIds) {
        sigSet.insert(id);
    }
    for (const int id : src.signalIds) {
        if (!sigSet.contains(id)) {
            dst.signalIds.push_back(id);
            sigSet.insert(id);
        }
    }

    QMap<int, int> satIndex;
    for (int i = 0; i < dst.satellites.size(); ++i) {
        satIndex[dst.satellites[i].satId] = i;
    }
    for (const RtcmMsmSatelliteData& sat : src.satellites) {
        const auto it = satIndex.constFind(sat.satId);
        if (it == satIndex.constEnd()) {
            satIndex[sat.satId] = dst.satellites.size();
            dst.satellites.push_back(sat);
            continue;
        }
        RtcmMsmSatelliteData& oldSat = dst.satellites[it.value()];
        if (oldSat.roughRangeIntegerMillis < 0 && sat.roughRangeIntegerMillis >= 0) {
            oldSat = sat;
        }
    }

    QSet<QString> cellSet;
    for (const RtcmMsmCellData& cell : dst.cells) {
        cellSet.insert(QString("%1:%2").arg(cell.satId).arg(cell.signalId));
    }
    for (const RtcmMsmCellData& cell : src.cells) {
        const QString key = QString("%1:%2").arg(cell.satId).arg(cell.signalId);
        if (!cellSet.contains(key)) {
            dst.cells.push_back(cell);
            cellSet.insert(key);
        }
    }
}

struct Vec3 {
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
};

Vec3 operator+(const Vec3& a, const Vec3& b) { return {a.x + b.x, a.y + b.y, a.z + b.z}; }
Vec3 operator-(const Vec3& a, const Vec3& b) { return {a.x - b.x, a.y - b.y, a.z - b.z}; }
Vec3 operator*(double s, const Vec3& a) { return {s * a.x, s * a.y, s * a.z}; }

struct Mat3 {
    double m[3][3] = {{0.0, 0.0, 0.0},
                      {0.0, 0.0, 0.0},
                      {0.0, 0.0, 0.0}};
};

double frac(double x) {
    return x - std::floor(x);
}

Mat3 rotX(double angle) {
    const double c = std::cos(angle);
    const double s = std::sin(angle);
    Mat3 uu{};
    uu.m[0][0] = 1.0;
    uu.m[1][1] = c;
    uu.m[1][2] = s;
    uu.m[2][1] = -s;
    uu.m[2][2] = c;
    return uu;
}

Mat3 rotY(double angle) {
    const double c = std::cos(angle);
    const double s = std::sin(angle);
    Mat3 uu{};
    uu.m[0][0] = c;
    uu.m[0][2] = -s;
    uu.m[1][1] = 1.0;
    uu.m[2][0] = s;
    uu.m[2][2] = c;
    return uu;
}

Mat3 rotZ(double angle) {
    const double c = std::cos(angle);
    const double s = std::sin(angle);
    Mat3 uu{};
    uu.m[0][0] = c;
    uu.m[0][1] = s;
    uu.m[1][0] = -s;
    uu.m[1][1] = c;
    uu.m[2][2] = 1.0;
    return uu;
}

Mat3 multiply(const Mat3& a, const Mat3& b) {
    Mat3 out{};
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            double sum = 0.0;
            for (int k = 0; k < 3; ++k) {
                sum += a.m[i][k] * b.m[k][j];
            }
            out.m[i][j] = sum;
        }
    }
    return out;
}

Vec3 multiply(const Mat3& a, const Vec3& v) {
    return {
        a.m[0][0] * v.x + a.m[0][1] * v.y + a.m[0][2] * v.z,
        a.m[1][0] * v.x + a.m[1][1] * v.y + a.m[1][2] * v.z,
        a.m[2][0] * v.x + a.m[2][1] * v.y + a.m[2][2] * v.z
    };
}

double dot(const Vec3& a, const Vec3& b) {
    return a.x * b.x + a.y * b.y + a.z * b.z;
}

double norm(const Vec3& a) {
    return std::sqrt(dot(a, a));
}

bool isReasonableReceiverXyz(const Vec3& xyz) {
    const double r = norm(xyz);
    return std::isfinite(r) && r > 5.0e6 && r < 7.5e6;
}

Vec3 cross(const Vec3& a, const Vec3& b) {
    return {a.y * b.z - a.z * b.y,
            a.z * b.x - a.x * b.z,
            a.x * b.y - a.y * b.x};
}

Vec3 unit(const Vec3& a) {
    const double n = norm(a);
    if (n <= 0.0) return {};
    return {a.x / n, a.y / n, a.z / n};
}

double gmst(double mjdUt1) {
    const double secs = 86400.0;
    const double mjd0 = std::floor(mjdUt1);
    const double ut1 = secs * (mjdUt1 - mjd0);
    const double t0 = (mjd0 - kMjdJ2000) / 36525.0;
    const double t = (mjdUt1 - kMjdJ2000) / 36525.0;
    double gmstSec = 24110.54841 + 8640184.812866 * t0 + 1.002737909350795 * ut1
                     + (0.093104 - 6.2e-6 * t) * t * t;
    return 2.0 * kPi * frac(gmstSec / secs);
}

Mat3 nutMatrix(double mjdTt) {
    const double t = (mjdTt - kMjdJ2000) / 36525.0;
    const double ls = 2.0 * kPi * frac(0.993133 + 99.997306 * t);
    const double d = 2.0 * kPi * frac(0.827362 + 1236.853087 * t);
    const double f = 2.0 * kPi * frac(0.259089 + 1342.227826 * t);
    const double n = 2.0 * kPi * frac(0.347346 - 5.372447 * t);

    const double dpsi = (-17.200 * std::sin(n) - 1.319 * std::sin(2.0 * (f - d + n))
                         - 0.227 * std::sin(2.0 * (f + n))
                         + 0.206 * std::sin(2.0 * n) + 0.143 * std::sin(ls)) / kRhoSec;
    const double deps = (9.203 * std::cos(n) + 0.574 * std::cos(2.0 * (f - d + n))
                         + 0.098 * std::cos(2.0 * (f + n))
                         - 0.090 * std::cos(2.0 * n)) / kRhoSec;

    const double eps = 0.4090928 - 2.2696e-4 * t;
    return multiply(multiply(rotX(-eps - deps), rotZ(-dpsi)), rotX(eps));
}

Mat3 precMatrix(double mjd1, double mjd2) {
    const double t = (mjd1 - kMjdJ2000) / 36525.0;
    const double dT = (mjd2 - mjd1) / 36525.0;

    const double zeta = ((2306.2181 + (1.39656 - 0.000139 * t) * t)
                         + ((0.30188 - 0.000344 * t) + 0.017998 * dT) * dT) * dT / kRhoSec;
    const double z = zeta + ((0.79280 + 0.000411 * t) + 0.000205 * dT) * dT * dT / kRhoSec;
    const double theta = ((2004.3109 - (0.85330 + 0.000217 * t) * t)
                          - ((0.42665 + 0.000217 * t) + 0.041833 * dT) * dT) * dT / kRhoSec;

    return multiply(multiply(rotZ(-z), rotY(theta)), rotZ(-zeta));
}

Vec3 sunPositionEcef(double mjdTt) {
    const double eps = 23.43929111 / kRhoDeg;
    const double t = (mjdTt - kMjdJ2000) / 36525.0;
    const double M = 2.0 * kPi * frac(0.9931267 + 99.9973583 * t);
    const double L = 2.0 * kPi * frac(0.7859444 + M / (2.0 * kPi)
                                      + (6892.0 * std::sin(M) + 72.0 * std::sin(2.0 * M)) / 1296.0e3);
    const double r = 149.619e9 - 2.499e9 * std::cos(M) - 0.021e9 * std::cos(2.0 * M);
    Vec3 rSun{r * std::cos(L), r * std::sin(L), 0.0};
    rSun = multiply(rotX(-eps), rSun);
    Mat3 rot = multiply(multiply(rotZ(gmst(mjdTt)), nutMatrix(mjdTt)), precMatrix(kMjdJ2000, mjdTt));
    return multiply(rot, rSun);
}

Vec3 moonPositionEcef(double mjdTt) {
    const double eps = 23.43929111 / kRhoDeg;
    const double t = (mjdTt - kMjdJ2000) / 36525.0;
    const double L0 = frac(0.606433 + 1336.851344 * t);
    const double l = 2.0 * kPi * frac(0.374897 + 1325.552410 * t);
    const double lp = 2.0 * kPi * frac(0.993133 + 99.997361 * t);
    const double d = 2.0 * kPi * frac(0.827361 + 1236.853086 * t);
    const double f = 2.0 * kPi * frac(0.259086 + 1342.227825 * t);

    const double dL = 22640 * std::sin(l) - 4586 * std::sin(l - 2.0 * d) + 2370 * std::sin(2.0 * d)
                      + 769 * std::sin(2.0 * l)
                      - 668 * std::sin(lp) - 412 * std::sin(2.0 * f) - 212 * std::sin(2.0 * l - 2.0 * d)
                      - 206 * std::sin(l + lp - 2.0 * d)
                      + 192 * std::sin(l + 2.0 * d) - 165 * std::sin(lp - 2.0 * d) - 125 * std::sin(d)
                      - 110 * std::sin(l + lp)
                      + 148 * std::sin(l - lp) - 55 * std::sin(2.0 * f - 2.0 * d);

    const double L = 2.0 * kPi * frac(L0 + dL / 1296.0e3);
    const double S = f + (dL + 412 * std::sin(2.0 * f) + 541 * std::sin(lp)) / kRhoSec;
    const double h = f - 2.0 * d;
    const double N = -526 * std::sin(h) + 44 * std::sin(l + h) - 31 * std::sin(-l + h)
                     - 23 * std::sin(lp + h)
                     + 11 * std::sin(-lp + h) - 25 * std::sin(-2.0 * l + f) + 21 * std::sin(-l + f);

    const double B = (18520.0 * std::sin(S) + N) / kRhoSec;
    const double cosB = std::cos(B);

    const double R = 385000e3 - 20905e3 * std::cos(l) - 3699e3 * std::cos(2.0 * d - l)
                     - 2956e3 * std::cos(2.0 * d)
                     - 570e3 * std::cos(2.0 * l) + 246e3 * std::cos(2.0 * l - 2.0 * d)
                     - 205e3 * std::cos(lp - 2.0 * d)
                     - 171e3 * std::cos(l + 2.0 * d) - 152e3 * std::cos(l + lp - 2.0 * d);

    Vec3 rMoon{R * std::cos(L) * cosB, R * std::sin(L) * cosB, R * std::sin(B)};
    rMoon = multiply(rotX(-eps), rMoon);
    Mat3 rot = multiply(multiply(rotZ(gmst(mjdTt)), nutMatrix(mjdTt)), precMatrix(kMjdJ2000, mjdTt));
    return multiply(rot, rMoon);
}

struct Ellipsoid {
    double lat = 0.0;
    double lon = 0.0;
    double h = 0.0;
};

Ellipsoid xyzToEll(const Vec3& xyz) {
    const double a = GnssUtils::kAell;
    const double f = 1.0 / GnssUtils::kFInv;
    const double e2 = f * (2.0 - f);

    double lon = std::atan2(xyz.y, xyz.x);
    double p = std::sqrt(xyz.x * xyz.x + xyz.y * xyz.y);
    double lat = std::atan2(xyz.z, p * (1.0 - e2));
    double h = 0.0;
    for (int i = 0; i < 10; ++i) {
        double sinLat = std::sin(lat);
        double n = a / std::sqrt(1.0 - e2 * sinLat * sinLat);
        h = p / std::cos(lat) - n;
        double latNew = std::atan2(xyz.z, p * (1.0 - e2 * n / (n + h)));
        if (std::fabs(latNew - lat) < 1e-12) {
            lat = latNew;
            break;
        }
        lat = latNew;
    }
    return {lat, lon, h};
}

Vec3 xyzToGeoc(const Vec3& xyz) {
    const Ellipsoid ell = xyzToEll(xyz);
    const double a = GnssUtils::kAell;
    const double f = 1.0 / GnssUtils::kFInv;
    const double b = a * (1.0 - f);
    const double e2 = (a * a - b * b) / (a * a);
    const double sinLat = std::sin(ell.lat);
    const double rn = a / std::sqrt(1.0 - e2 * sinLat * sinLat);
    const double rho = norm(xyz);
    Vec3 geoc;
    geoc.x = std::atan((1.0 - e2 * rn / (rn + ell.h)) * std::tan(ell.lat));
    geoc.y = ell.lon;
    geoc.z = rho - kGeocentricEarthRadius;
    return geoc;
}

Vec3 xyzToNeu(const Ellipsoid& ell, const Vec3& xyz) {
    const double sinLat = std::sin(ell.lat);
    const double cosLat = std::cos(ell.lat);
    const double sinLon = std::sin(ell.lon);
    const double cosLon = std::cos(ell.lon);

    Vec3 out;
    out.x = -sinLat * cosLon * xyz.x - sinLat * sinLon * xyz.y + cosLat * xyz.z;
    out.y = -sinLon * xyz.x + cosLon * xyz.y;
    out.z = cosLat * cosLon * xyz.x + cosLat * sinLon * xyz.y + sinLat * xyz.z;
    return out;
}

Vec3 neuToXyz(const Ellipsoid& ell, const Vec3& neu) {
    const double sinLat = std::sin(ell.lat);
    const double cosLat = std::cos(ell.lat);
    const double sinLon = std::sin(ell.lon);
    const double cosLon = std::cos(ell.lon);

    Vec3 out;
    out.x = -sinLat * cosLon * neu.x - sinLon * neu.y + cosLat * cosLon * neu.z;
    out.y = -sinLat * sinLon * neu.x + cosLon * neu.y + cosLat * sinLon * neu.z;
    out.z = cosLat * neu.x + sinLat * neu.z;
    return out;
}

Mat covNeuToXyz(const Ellipsoid& ell, const double qN, const double qE, const double qU) {
    const double sinLat = std::sin(ell.lat);
    const double cosLat = std::cos(ell.lat);
    const double sinLon = std::sin(ell.lon);
    const double cosLon = std::cos(ell.lon);
    const double c[3][3] = {
        {-sinLat * cosLon, -sinLat * sinLon,  cosLat},
        {-sinLon,           cosLon,           0.0},
        { cosLat * cosLon,  cosLat * sinLon,  sinLat}
    };
    const double qNeu[3] = {qN, qE, qU};
    Mat qXyz(3, Vec(3, 0.0));
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            for (int k = 0; k < 3; ++k) {
                qXyz[i][j] += c[k][i] * qNeu[k] * c[k][j];
            }
        }
    }
    return qXyz;
}

double factorialInt(int n) {
    double out = 1.0;
    for (int i = 2; i <= n; ++i) {
        out *= static_cast<double>(i);
    }
    return out;
}

double associatedLegendreFunction(int n, int m, double t) {
    double sum = 0.0;
    const int r = static_cast<int>(std::floor((n - m) / 2.0));
    for (int k = 0; k <= r; ++k) {
        sum += std::pow(-1.0, static_cast<double>(k)) * factorialInt(2 * n - 2 * k)
             / (factorialInt(k) * factorialInt(n - k) * factorialInt(n - m - 2 * k))
             * std::pow(t, static_cast<double>(n - m - 2 * k));
    }
    const double fac = std::pow(2.0, static_cast<double>(-n)) *
                       std::pow(1.0 - t * t, static_cast<double>(m) / 2.0);
    return sum * fac;
}

double vtecSingleLayerContribution(const PppVtecLayer& layer, double phiPp, double lonS) {
    if (layer.cosine.isEmpty()) {
        return 0.0;
    }
    const int nMax = layer.cosine.size() - 1;
    int mMax = 0;
    for (const QVector<double>& row : layer.cosine) {
        mMax = qMax(mMax, row.size() - 1);
    }
    for (const QVector<double>& row : layer.sine) {
        mMax = qMax(mMax, row.size() - 1);
    }

    double vtec = 0.0;
    for (int n = 0; n <= nMax; ++n) {
        for (int m = 0; m <= qMin(n, mMax); ++m) {
            const double c = (n < layer.cosine.size() && m < layer.cosine[n].size())
                ? layer.cosine[n][m] : 0.0;
            const double s = (n < layer.sine.size() && m < layer.sine[n].size())
                ? layer.sine[n][m] : 0.0;
            double pnm = associatedLegendreFunction(n, m, std::sin(phiPp));
            const double a = factorialInt(n - m);
            const double b = factorialInt(n + m);
            const double fac = (m == 0)
                ? std::sqrt(2.0 * n + 1.0)
                : std::sqrt(2.0 * (2.0 * n + 1.0) * a / b);
            pnm *= fac;
            vtec += (c * std::cos(m * lonS) + s * std::sin(m * lonS)) * pnm;
        }
    }
    return qMax(0.0, vtec);
}

double vtecStecTecu(const PppVtecModel& vtec, const PppTime& time,
                    const Vec3& staXyz, const Vec3& satXyz,
                    double signalPropagationTime) {
    if (!vtec.isValid() || !std::isfinite(signalPropagationTime)) {
        return 0.0;
    }

    const Vec3 geocSta = xyzToGeoc(staXyz);
    const double omegaZ = GnssUtils::kOmegaEarth * signalPropagationTime;
    const double cosO = std::cos(omegaZ);
    const double sinO = std::sin(omegaZ);
    const Vec3 xyzSatRec{
        satXyz.x * cosO + satXyz.y * sinO,
        satXyz.y * cosO - satXyz.x * sinO,
        satXyz.z
    };

    const Vec3 rhoV = xyzSatRec - staXyz;
    const double rho = norm(rhoV);
    if (rho <= 0.0) {
        return 0.0;
    }
    const Ellipsoid sphEll{geocSta.x, geocSta.y, geocSta.z};
    const Vec3 neu = xyzToNeu(sphEll, rhoV);
    double cosZen = std::sqrt(neu.x * neu.x + neu.y * neu.y) / rho;
    cosZen = clampValue(cosZen, -1.0, 1.0);
    double sphEle = std::acos(cosZen);
    if (neu.z < 0.0) {
        sphEle *= -1.0;
    }
    const double sphAzi = std::atan2(neu.y, neu.x);
    const double epoch = std::fmod(time.gpsSow, 86400.0);

    double stec = 0.0;
    for (const PppVtecLayer& layer : vtec.layers) {
        if (layer.heightMeters <= 0.0) {
            continue;
        }
        const double q = (kGeocentricEarthRadius + geocSta.z) /
                         (kGeocentricEarthRadius + layer.heightMeters);
        const double asinArg = clampValue(q * std::cos(sphEle), -1.0, 1.0);
        const double psiPp = kPi / 2.0 - sphEle - std::asin(asinArg);
        const double phiPp = std::asin(std::sin(geocSta.x) * std::cos(psiPp) +
                                       std::cos(geocSta.x) * std::sin(psiPp) * std::cos(sphAzi));

        double lambdaPp = 0.0;
        const double cosPhi = std::cos(phiPp);
        if (std::fabs(cosPhi) < 1e-12) {
            lambdaPp = geocSta.y;
        } else {
            const double arg = clampValue(std::sin(psiPp) * std::sin(sphAzi) / cosPhi, -1.0, 1.0);
            if (((geocSta.x * kRhoDeg > 0.0) &&
                 (std::tan(psiPp) * std::cos(sphAzi) > std::tan(kPi / 2.0 - geocSta.x))) ||
                ((geocSta.x * kRhoDeg < 0.0) &&
                 (-(std::tan(psiPp) * std::cos(sphAzi)) > std::tan(kPi / 2.0 + geocSta.x)))) {
                lambdaPp = geocSta.y + kPi - std::asin(arg);
            } else {
                lambdaPp = geocSta.y + std::asin(arg);
            }
        }
        const double lonS = std::fmod(lambdaPp + (epoch - 50400.0) * kPi / 43200.0, 2.0 * kPi);
        stec += vtecSingleLayerContribution(layer, phiPp, lonS) * std::sin(sphEle + psiPp);
    }
    return stec;
}

double vtecL1DelayMeters(const PppVtecModel& vtec, const PppTime& time,
                         const Vec3& staXyz, const Vec3& satXyz,
                         double signalPropagationTime) {
    const double stecTecu = vtecStecTecu(vtec, time, staXyz, satXyz, signalPropagationTime);
    if (stecTecu == 0.0) {
        return 0.0;
    }
    return 40.3e16 / (kFreqGpsL1Hz * kFreqGpsL1Hz) * stecTecu;
}

QString antexFreqKey(QChar system, QChar band) {
    switch (system.toLatin1()) {
    case 'G':
        if (band == QLatin1Char('1')) return "G01";
        if (band == QLatin1Char('2')) return "G02";
        if (band == QLatin1Char('5')) return "G05";
        break;
    case 'R':
        if (band == QLatin1Char('1')) return "R01";
        if (band == QLatin1Char('2')) return "R02";
        break;
    case 'E':
        if (band == QLatin1Char('1')) return "E01";
        if (band == QLatin1Char('5')) return "E05";
        if (band == QLatin1Char('6')) return "E06";
        if (band == QLatin1Char('7')) return "E07";
        if (band == QLatin1Char('8')) return "E08";
        break;
    case 'C':
        if (band == QLatin1Char('1')) return "C01";
        if (band == QLatin1Char('2')) return "C02";
        if (band == QLatin1Char('6')) return "C06";
        if (band == QLatin1Char('7')) return "C07";
        break;
    case 'J':
        if (band == QLatin1Char('1')) return "J01";
        if (band == QLatin1Char('2')) return "J02";
        if (band == QLatin1Char('5')) return "J05";
        if (band == QLatin1Char('6')) return "J06";
        break;
    default:
        break;
    }
    return QString();
}

std::pair<QString, QString> defaultAntennaFrequencyPair(QChar system) {
    switch (system.toLatin1()) {
    case 'R':
        return {QStringLiteral("R01"), QStringLiteral("R02")};
    case 'E':
        return {QStringLiteral("E01"), QStringLiteral("E05")};
    case 'C':
        return {QStringLiteral("C02"), QStringLiteral("C06")};
    default:
        return {QStringLiteral("G01"), QStringLiteral("G02")};
    }
}

class AntexModel {
public:
    struct FrequencyMap {
        double neu[3] = {0.0, 0.0, 0.0};
        QVector<double> pattern;
    };

    struct AntennaMap {
        QString antName;
        double zen1 = 0.0;
        double zen2 = 0.0;
        double dZen = 0.0;
        QMap<QString, FrequencyMap> frqMap;
    };

    bool load(const QString& filePath) {
        _maps.clear();
        if (filePath.trimmed().isEmpty()) {
            return false;
        }

        QFile inFile(filePath);
        if (!inFile.open(QIODevice::ReadOnly | QIODevice::Text)) {
            return false;
        }
        QTextStream in(&inFile);

        bool inAntenna = false;
        bool inFreq = false;
        AntennaMap ant;
        FrequencyMap frq;
        QString freqKey;

        while (!in.atEnd()) {
            const QString line = in.readLine();
            const QString label = line.size() >= 60 ? line.mid(60).trimmed() : QString();
            const QString left = line.left(60);

            if (label == QLatin1String("START OF ANTENNA")) {
                inAntenna = true;
                inFreq = false;
                ant = AntennaMap();
                frq = FrequencyMap();
                freqKey.clear();
                continue;
            }

            if (!inAntenna) {
                continue;
            }

            if (label == QLatin1String("END OF ANTENNA")) {
                if (!ant.antName.isEmpty()) {
                    _maps[ant.antName.toUpper()] = ant;
                }
                inAntenna = false;
                inFreq = false;
                continue;
            }

            if (label == QLatin1String("TYPE / SERIAL NO")) {
                const QString typeField = line.left(20);
                if (typeField.startsWith("BLOCK I", Qt::CaseInsensitive) ||
                    typeField.startsWith("GLONASS", Qt::CaseInsensitive) ||
                    typeField.startsWith("QZSS", Qt::CaseInsensitive) ||
                    typeField.startsWith("BEIDOU", Qt::CaseInsensitive) ||
                    typeField.startsWith("GALILEO", Qt::CaseInsensitive) ||
                    typeField.startsWith("NAVIC", Qt::CaseInsensitive)) {
                    ant.antName = line.mid(20, 3).trimmed().toUpper();
                } else {
                    ant.antName = typeField.trimmed().toUpper();
                }
                continue;
            }

            if (label == QLatin1String("ZEN1 / ZEN2 / DZEN")) {
                QString parse = left;
                QTextStream ts(&parse, QIODevice::ReadOnly);
                ts >> ant.zen1 >> ant.zen2 >> ant.dZen;
                continue;
            }

            if (label == QLatin1String("START OF FREQUENCY")) {
                inFreq = true;
                frq = FrequencyMap();
                freqKey = line.mid(3, 3).trimmed().toUpper();
                continue;
            }

            if (label == QLatin1String("END OF FREQUENCY")) {
                if (inFreq && !freqKey.isEmpty()) {
                    ant.frqMap[freqKey] = frq;
                }
                inFreq = false;
                freqKey.clear();
                continue;
            }

            if (!inFreq) {
                continue;
            }

            if (label == QLatin1String("NORTH / EAST / UP")) {
                QString parse = left;
                QTextStream ts(&parse, QIODevice::ReadOnly);
                ts >> frq.neu[0] >> frq.neu[1] >> frq.neu[2];
                frq.neu[0] *= 1e-3;
                frq.neu[1] *= 1e-3;
                frq.neu[2] *= 1e-3;
                continue;
            }

            if (left.mid(3, 5) == QLatin1String("NOAZI")) {
                const int nPat = (ant.dZen > 0.0) ? static_cast<int>((ant.zen2 - ant.zen1) / ant.dZen + 1.5) : 0;
                frq.pattern.clear();
                frq.pattern.reserve(std::max(0, nPat));
                QString parse = left;
                QTextStream ts(&parse, QIODevice::ReadOnly);
                QString dummy;
                ts >> dummy;
                for (int i = 0; i < nPat; ++i) {
                    double value = 0.0;
                    ts >> value;
                    frq.pattern.push_back(value * 1e-3);
                }
            }
        }

        return !_maps.isEmpty();
    }

    double satCorr(const QString& prn, const QString& frqKey,
                   double elTx, double azTx, bool& found) const {
        const QString antKey = prn.mid(0, 3).toUpper();
        const auto itAnt = _maps.constFind(antKey);
        if (itAnt == _maps.constEnd()) {
            found = false;
            return 0.0;
        }
        const auto itFrq = itAnt->frqMap.constFind(frqKey.toUpper());
        if (itFrq == itAnt->frqMap.constEnd()) {
            found = false;
            return 0.0;
        }
        return corrValue(*itAnt, itFrq.value(), elTx, azTx, found);
    }

    double rcvCorr(const QString& antName, const QString& frqKey,
                   double eleSat, double azSat, bool& found) const {
        if (antName.contains("NULLANTENNA", Qt::CaseInsensitive)) {
            found = true;
            return 0.0;
        }
        const QString key = antName.trimmed().toUpper();
        const auto itAnt = _maps.constFind(key);
        if (itAnt == _maps.constEnd()) {
            found = false;
            return 0.0;
        }
        const auto itFrq = itAnt->frqMap.constFind(frqKey.toUpper());
        if (itFrq == itAnt->frqMap.constEnd()) {
            found = false;
            return 0.0;
        }
        return corrValue(*itAnt, itFrq.value(), eleSat, azSat, found);
    }

    bool isLoaded() const {
        return !_maps.isEmpty();
    }

private:
    static double corrValue(const AntennaMap& ant, const FrequencyMap& frq,
                            double ele, double az, bool& found) {
        double var = 0.0;
        if (!frq.pattern.isEmpty() && ant.dZen > 0.0) {
            double zenDiff = 999.999;
            const double zenSat = 90.0 - ele * kRhoDeg;
            for (int i = 0; i < frq.pattern.size(); ++i) {
                const double zen = ant.zen1 + static_cast<double>(i) * ant.dZen;
                const double newZenDiff = std::fabs(zen - zenSat);
                if (newZenDiff < zenDiff) {
                    zenDiff = newZenDiff;
                    var = frq.pattern[i];
                }
            }
        }
        found = true;
        return var
               - frq.neu[0] * std::cos(az) * std::cos(ele)
               - frq.neu[1] * std::sin(az) * std::cos(ele)
               - frq.neu[2] * std::sin(ele);
    }

    QMap<QString, AntennaMap> _maps;
};

class OceanTideModel {
public:
    struct BlqData {
        double amplitudes[3][11] = {{0.0}};
        double phases[3][11] = {{0.0}};
    };

    bool load(const QString& filePath) {
        _blqMap.clear();
        if (filePath.trimmed().isEmpty()) {
            return false;
        }

        QFile inFile(filePath);
        if (!inFile.open(QIODevice::ReadOnly | QIODevice::Text)) {
            return false;
        }
        QTextStream in(&inFile);

        int row = 0;
        QString site;
        BlqData blq{};

        while (!in.atEnd()) {
            QString line = in.readLine();
            if (line.contains("$$")) {
                continue;
            }
            line = line.trimmed();
            if (line.isEmpty()) {
                continue;
            }
            QString parse = line;
            QTextStream ts(&parse, QIODevice::ReadOnly);

            switch (row) {
            case 0:
                site = line.toUpper();
                blq = BlqData{};
                break;
            case 1:
            case 2:
            case 3:
                for (int ii = 0; ii < 11; ++ii) {
                    ts >> blq.amplitudes[row - 1][ii];
                }
                break;
            case 4:
            case 5:
            case 6:
                for (int ii = 0; ii < 11; ++ii) {
                    ts >> blq.phases[row - 4][ii];
                }
                if (!site.isEmpty()) {
                    _blqMap[site] = blq;
                }
                row = -1;
                break;
            default:
                break;
            }
            ++row;
        }
        return !_blqMap.isEmpty();
    }

    Vec3 ocean(const PppTime& time, const Vec3& xyz, const QString& station) const {
        if (!time.valid || station.trimmed().isEmpty()) {
            return {};
        }
        const QString staKey = station.trimmed().toUpper();
        auto it = _blqMap.constFind(staKey);
        if (it == _blqMap.constEnd()) {
            return {};
        }
        const BlqData& blq = it.value();

        const double speed[11] = {1.40519e-4, 1.45444e-4, 1.3788e-4, 1.45842e-4, 7.2921e-5,
                                  6.7598e-5,  7.2523e-5,  6.4959e-5, 5.3234e-6,  2.6392e-6, 3.982e-7};
        const double angfac[4][11] = {
            {2.0, 0.0, 2.0, 2.0, 1.0, 1.0, -1.0, 1.0, 0.0, 0.0, 2.0},
            {-2.0, 0.0, -3.0, 0.0, 0.0, -2.0, 0.0, -3.0, 2.0, 1.0, 0.0},
            {0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, -1.0, 0.0},
            {0.0, 0.0, 0.0, 0.0, 0.25, -0.25, -0.25, -0.25, 0.0, 0.0, 0.0}
        };

        const QDateTime utc = time.utc.toUTC();
        const int iyear = utc.date().year() - 2000;
        const int doy = utc.date().dayOfYear();
        const double fday = static_cast<double>(utc.time().msecsSinceStartOfDay()) / 1000.0;
        const int icapd = doy + 365 * (iyear - 75) + ((iyear - 73) / 4);
        const double capt = (icapd * 1.000000035 + 27392.500528) / 36525.0;
        const double dtr = kPi / 180.0;

        const double h0 = (279.69668 + (36000.768930485 + 3.03e-4 * capt) * capt) * dtr;
        const double s0 = (((1.9e-6 * capt - 0.001133) * capt + 481267.88314137) * capt + 270.434358) * dtr;
        const double p0 = (((-1.2e-5 * capt - 0.010325) * capt + 4069.0340329577) * capt + 334.329653) * dtr;

        double angle[11] = {0.0};
        for (int k = 0; k < 11; ++k) {
            angle[k] = speed[k] * fday
                       + angfac[0][k] * h0
                       + angfac[1][k] * s0
                       + angfac[2][k] * p0
                       + angfac[3][k] * (2.0 * kPi);
            angle[k] = std::fmod(angle[k], 2.0 * kPi);
            if (angle[k] < 0.0) {
                angle[k] += 2.0 * kPi;
            }
        }

        double rws[3] = {0.0, 0.0, 0.0};
        for (int rr = 0; rr < 3; ++rr) {
            for (int cc = 0; cc < 11; ++cc) {
                rws[rr] += blq.amplitudes[rr][cc] * std::cos(angle[cc] - blq.phases[rr][cc] / kRhoDeg);
            }
        }

        Vec3 dneu{};
        dneu.x = -rws[2];
        dneu.y = -rws[1];
        dneu.z = rws[0];
        const Ellipsoid ell = xyzToEll(xyz);
        return neuToXyz(ell, dneu);
    }

    bool isLoaded() const {
        return !_blqMap.isEmpty();
    }

private:
    QMap<QString, BlqData> _blqMap;
};


Mat zeros(int r, int c) {
    return Mat(r, Vec(c, 0.0));
}

Mat transpose(const Mat& a) {
    if (a.isEmpty()) return Mat();
    Mat out = zeros(a[0].size(), a.size());
    for (int i = 0; i < a.size(); ++i) {
        for (int j = 0; j < a[0].size(); ++j) {
            out[j][i] = a[i][j];
        }
    }
    return out;
}

Mat multiply(const Mat& a, const Mat& b) {
    if (a.isEmpty() || b.isEmpty()) return Mat();
    Mat out = zeros(a.size(), b[0].size());
    for (int i = 0; i < a.size(); ++i) {
        for (int k = 0; k < b.size(); ++k) {
            const double aik = a[i][k];
            if (aik == 0.0) continue;
            for (int j = 0; j < b[0].size(); ++j) {
                out[i][j] += aik * b[k][j];
            }
        }
    }
    return out;
}

Vec multiply(const Mat& a, const Vec& v) {
    Vec out(a.size(), 0.0);
    for (int i = 0; i < a.size(); ++i) {
        double sum = 0.0;
        for (int j = 0; j < v.size(); ++j) {
            sum += a[i][j] * v[j];
        }
        out[i] = sum;
    }
    return out;
}

bool invert(const Mat& a, Mat& inv) {
    const int n = a.size();
    if (n == 0 || a[0].size() != n) {
        return false;
    }
    Mat aug = zeros(n, 2 * n);
    for (int i = 0; i < n; ++i) {
        for (int j = 0; j < n; ++j) {
            aug[i][j] = a[i][j];
        }
        aug[i][n + i] = 1.0;
    }
    for (int i = 0; i < n; ++i) {
        int pivot = i;
        double maxVal = std::fabs(aug[i][i]);
        for (int r = i + 1; r < n; ++r) {
            double val = std::fabs(aug[r][i]);
            if (val > maxVal) {
                maxVal = val;
                pivot = r;
            }
        }
        if (maxVal < 1e-12) {
            return false;
        }
        if (pivot != i) {
            std::swap(aug[i], aug[pivot]);
        }
        double diag = aug[i][i];
        for (int c = 0; c < 2 * n; ++c) {
            aug[i][c] /= diag;
        }
        for (int r = 0; r < n; ++r) {
            if (r == i) continue;
            double factor = aug[r][i];
            if (factor == 0.0) continue;
            for (int c = 0; c < 2 * n; ++c) {
                aug[r][c] -= factor * aug[i][c];
            }
        }
    }
    inv = zeros(n, n);
    for (int i = 0; i < n; ++i) {
        for (int j = 0; j < n; ++j) {
            inv[i][j] = aug[i][n + j];
        }
    }
    return true;
}

bool choleskyDecompose(const Mat& a, Mat& l) {
    const int n = a.size();
    if (n == 0 || a[0].size() != n) {
        return false;
    }
    l = zeros(n, n);
    for (int i = 0; i < n; ++i) {
        for (int j = 0; j <= i; ++j) {
            double sum = a[i][j];
            for (int k = 0; k < j; ++k) {
                sum -= l[i][k] * l[j][k];
            }
            if (!std::isfinite(sum)) {
                return false;
            }
            if (i == j) {
                if (sum <= 1e-12) {
                    return false;
                }
                l[i][j] = std::sqrt(sum);
                if (!std::isfinite(l[i][j])) {
                    return false;
                }
            } else {
                if (std::fabs(l[j][j]) < 1e-12) {
                    return false;
                }
                l[i][j] = sum / l[j][j];
                if (!std::isfinite(l[i][j])) {
                    return false;
                }
            }
        }
    }
    return true;
}

bool solveUpperTriangular(const Mat& u, const Vec& b, Vec& x) {
    const int n = u.size();
    if (n == 0 || b.size() != n) {
        return false;
    }
    x = Vec(n, 0.0);
    for (int i = n - 1; i >= 0; --i) {
        double sum = b[i];
        for (int j = i + 1; j < n; ++j) {
            sum -= u[i][j] * x[j];
        }
        if (std::fabs(u[i][i]) < 1e-12) {
            return false;
        }
        x[i] = sum / u[i][i];
        if (!std::isfinite(x[i])) {
            return false;
        }
    }
    return true;
}

bool invertUpperTriangular(const Mat& u, Mat& inv) {
    const int n = u.size();
    if (n == 0 || u[0].size() != n) {
        return false;
    }
    inv = zeros(n, n);
    for (int col = 0; col < n; ++col) {
        Vec rhs(n, 0.0);
        rhs[col] = 1.0;
        Vec sol;
        if (!solveUpperTriangular(u, rhs, sol)) {
            return false;
        }
        for (int row = 0; row < n; ++row) {
            inv[row][col] = sol[row];
        }
    }
    return true;
}

bool qrzUpper(Mat a, Mat& upper) {
    const int nRows = a.size();
    if (nRows == 0) {
        return false;
    }
    const int nCols = a[0].size();
    upper = zeros(nCols, nCols);
    if (nCols == 0) {
        return true;
    }

    for (int iCol = 0; iCol < nCols; ++iCol) {
        const int remain = nCols - iCol;
        for (int jOff = 0; jOff < remain; ++jOff) {
            double sum = 0.0;
            for (int row = 0; row < nRows; ++row) {
                sum += a[row][iCol] * a[row][iCol + jOff];
            }
            if (!std::isfinite(sum)) {
                return false;
            }
            upper[iCol][iCol + jOff] += sum;
        }

        double diagSq = upper[iCol][iCol];
        if (diagSq < 0.0 && std::fabs(diagSq) < 1e-10) {
            diagSq = 0.0;
        }
        if (!std::isfinite(diagSq) || diagSq < 0.0) {
            return false;
        }
        const double norm = std::sqrt(diagSq);
        upper[iCol][iCol] = norm;
        if (norm == 0.0) {
            for (int j = iCol + 1; j < nCols; ++j) {
                upper[iCol][j] = 0.0;
            }
            for (int row = 0; row < nRows; ++row) {
                a[row][iCol] = 0.0;
            }
            continue;
        }

        for (int j = iCol + 1; j < nCols; ++j) {
            upper[iCol][j] /= norm;
        }

        for (int row = 0; row < nRows; ++row) {
            const double xi = a[row][iCol] / norm;
            a[row][iCol] = xi;
            for (int j = iCol + 1; j < nCols; ++j) {
                a[row][j] -= upper[iCol][j] * xi;
            }
        }
    }

    return true;
}

void sanitizeCovariance(Mat& q) {
    if (q.isEmpty()) {
        return;
    }
    const int n = q.size();
    for (int i = 0; i < n; ++i) {
        if (q[i].size() != n) {
            return;
        }
    }
    for (int i = 0; i < n; ++i) {
        for (int j = i + 1; j < n; ++j) {
            double sij = 0.5 * (q[i][j] + q[j][i]);
            if (!std::isfinite(sij)) {
                sij = 0.0;
            }
            q[i][j] = sij;
            q[j][i] = sij;
        }
        if (!std::isfinite(q[i][i]) || q[i][i] < 1e-12) {
            q[i][i] = 1e-12;
        }
    }
}

struct PppSatData {
    PppTime tt;
    QString prn;
    double P3 = 0.0;
    double L3 = 0.0;
    double C1 = 0.0;
    double C2 = 0.0;
    double L1 = 0.0;
    double L2 = 0.0;
    double P3sig = 0.0;
    double L3sig = 0.0;
    double lkA = 0.0;
    double lkB = 0.0;
    double lambda3 = 0.0;
    double lambda1 = 0.0;
    double lambda2 = 0.0;
    double f1 = 0.0;
    double f2 = 0.0;
    Vec3 satPos;
    Vec3 satVel;
    double clk = 0.0;
    double eleSat = 0.0;
    double azSat = 0.0;
    double rho = 0.0;
    int obsIndex = -1;
    bool slipFlag = false;
    QChar bandA;
    QChar bandB;
    bool useSsrYaw = false;
    double yawRad = 0.0;
    int sii1 = 0;
    int sii2 = 0;
    int wli1 = 0;
    int wli2 = 0;
    int lock1 = -1;
    int lock2 = -1;
    int disc1 = -1;
    int disc2 = -1;
    bool hasCodeBias1 = false;
    bool hasCodeBias2 = false;
    bool hasBias1 = false;
    bool hasBias2 = false;
    double codeBias1 = 0.0;
    double codeBias2 = 0.0;
    double phaseBias1 = 0.0;
    double phaseBias2 = 0.0;
    QString phaseBiasDiag;
    double stecSat = 0.0;
    int satRefDatum = -1;
    bool modelSet = false;
    bool outlier = false;
    double modelRho = 0.0;
    double modelSatClkM = 0.0;
    double modelRecClkM = 0.0;
    double modelSagnac = 0.0;
    double modelAntEcc = 0.0;
    double modelTropo = 0.0;
    double modelWetMapping = 0.0;
    double modelTideEarth = 0.0;
    double modelTideOcean = 0.0;
    double modelRel = 0.0;
    double modelAntPco1 = 0.0;
    double modelAntPco2 = 0.0;
    double modelWindUp = 0.0;

    QChar system() const { return prn.isEmpty() ? QChar() : prn.at(0); }
};

bool hasCompleteArBiases(const PppOptions& options, const PppSatData& sat) {
    if (!options.enablePppAr ||
        options.arAlgorithm.compare(QStringLiteral("BIE"), Qt::CaseInsensitive) != 0) {
        return true;
    }

    bool usesFrequency1 = false;
    bool usesFrequency2 = false;
    for (ObservationCombination lc : combinationsForSystem(options, sat.system())) {
        switch (lc) {
        case ObservationCombination::C1:
        case ObservationCombination::L1:
            usesFrequency1 = true;
            break;
        case ObservationCombination::C2:
        case ObservationCombination::L2:
            usesFrequency2 = true;
            break;
        case ObservationCombination::CIf:
        case ObservationCombination::LIf:
            usesFrequency1 = true;
            usesFrequency2 = true;
            break;
        case ObservationCombination::Gim:
        case ObservationCombination::Dummy:
            break;
        }
    }

    if ((usesFrequency1 && sat.C1 != 0.0 && !sat.hasCodeBias1) ||
        (usesFrequency2 && sat.C2 != 0.0 && !sat.hasCodeBias2)) {
        return false;
    }
    if (options.arUseSystem(sat.system()) &&
        ((usesFrequency1 && sat.L1 != 0.0 && !sat.hasBias1) ||
         (usesFrequency2 && sat.L2 != 0.0 && !sat.hasBias2))) {
        return false;
    }
    return true;
}

} // namespace

struct PppEpochData {
    PppTime tt;
    QMap<QString, PppSatData> satData;
    unsigned sizeSys(QChar sys) const {
        unsigned ans = 0;
        for (auto it = satData.constBegin(); it != satData.constEnd(); ++it) {
            if (it.value().system() == sys) ++ans;
        }
        return ans;
    }
    unsigned sizeAll() const { return satData.size(); }
};

class PppFilter {
public:
    enum ParamType {CRD_X, CRD_Y, CRD_Z,
                    CLK_G, CLK_R, CLK_E, CLK_C,
                    TROPO, IONO_L1, AMB_L1, AMB_L2, AMB_IF,
                    CBIAS_G1, CBIAS_R1, CBIAS_E1, CBIAS_C1,
                    PBIAS_G1, PBIAS_R1, PBIAS_E1, PBIAS_C1,
                    CBIAS_G2, CBIAS_R2, CBIAS_E2, CBIAS_C2,
                    PBIAS_G2, PBIAS_R2, PBIAS_E2, PBIAS_C2};
    enum ObsKind {CODE1, CODE2, CODE_IF, PHASE1, PHASE2, PHASE_IF, GIM};

    struct Param {
        ParamType type;
        double value = 0.0;
        double x0 = 0.0;
        int index = 0;
        int indexOld = 0;
        QString prn;
        ObservationCombination lc = ObservationCombination::Dummy;
        PppTime firstObsTime;
        PppTime lastObsTime;
        double ambEleSat = 0.0;
        unsigned ambNumEpo = 0;
        bool isCrd() const { return type == CRD_X || type == CRD_Y || type == CRD_Z; }
    };
    struct OutlierInfo {
        QString prn;
        ObsKind kind = CODE1;
        double residual = 0.0;
    };
    struct ObsRow {
        QString prn;
        ObsKind kind = CODE1;
        ObservationCombination lc = ObservationCombination::C1;
        double meas = 0.0;
        double sigma = 1.0;
    };

    explicit PppFilter(const PppOptions* opt, const PppVtecModel* vtecModel,
                       const PppTropospherePrior& tropoPrior)
        : _opt(opt), _vtecModel(vtecModel), _tropoPrior(tropoPrior) {
        initModels();
        resetCurrentGeometry();
        reset();
    }

    QVector<QString> takeWarnings() {
        QVector<QString> out = _warnings;
        _warnings.clear();
        return out;
    }

    const QString& failureReason() const {
        return _failureReason;
    }

    std::array<double, 3> bancroftPosition() const {
        return {_xcBanc.x, _xcBanc.y, _xcBanc.z};
    }

    bool hasBancroftPosition() const {
        return isReasonableReceiverXyz(_xcBanc);
    }

    bool update(PppEpochData& epoData, PppSolution& solution) {
        _warnings.clear();
        _warningSet.clear();
        _solutionLogLines.clear();
        _diagLines.clear();
        _observationResiduals.clear();
        _antApcDiagLogged = false;
        _arSummary.clear();
        _arEpochLogLines.clear();
        _arFixedPrns.clear();
        _arEligiblePrns.clear();
        _arRatio = 0.0;
        _arBootstrap = 0.0;
        _arCandidateCount = 0;
        _arEffectiveCandidates = 0.0;
        _arConstraintsApplied = 0;
        _arGroups.clear();
        _bieFixedResultLogged = false;
        _restoreBieState = false;
        _bieFloatParams.clear();
        _bieFloatQ.clear();
        _failureReason.clear();
        _time = epoData.tt;
        if (!update_p(epoData)) {
            restoreBieFloatState();
            return false;
        }
        // In default compatible mode xFix/QFix are epoch-local copies. Persistent
        // BIE mode intentionally leaves accepted constraints in _params/_Q.
        restoreBieFloatState();

        solution.valid = true;
        solution.utc = epoData.tt.utc;
        solution.gpsWeek = epoData.tt.gpsWeek;
        solution.gpsSow = epoData.tt.gpsSow;
        solution.gpsTime = GnssUtils::gpsCalendarDateTime(solution.gpsWeek, solution.gpsSow);
        const Vec3 dXyz{paramValue(CRD_X), paramValue(CRD_Y), paramValue(CRD_Z)};
        const Vec3 solXyz = _baseXyz + dXyz;
        solution.xyz[0] = solXyz.x;
        solution.xyz[1] = solXyz.y;
        solution.xyz[2] = solXyz.z;
        solution.trp = paramEstimate(TROPO);
        solution.trpStdev = paramStd(TROPO);
        if (!std::isfinite(solution.xyz[0]) ||
            !std::isfinite(solution.xyz[1]) ||
            !std::isfinite(solution.xyz[2]) ||
            !std::isfinite(solution.trp) ||
            !std::isfinite(solution.trpStdev)) {
            _failureReason = QString("non-finite solution after filter: xyz=(%1,%2,%3) trp=%4 trpStd=%5 base=(%6,%7,%8) dX=(%9,%10,%11)")
                                 .arg(solution.xyz[0], 0, 'g', 12)
                                 .arg(solution.xyz[1], 0, 'g', 12)
                                 .arg(solution.xyz[2], 0, 'g', 12)
                                 .arg(solution.trp, 0, 'g', 12)
                                 .arg(solution.trpStdev, 0, 'g', 12)
                                 .arg(_baseXyz.x, 0, 'g', 12)
                                 .arg(_baseXyz.y, 0, 'g', 12)
                                 .arg(_baseXyz.z, 0, 'g', 12)
                                 .arg(dXyz.x, 0, 'g', 12)
                                 .arg(dXyz.y, 0, 'g', 12)
                                 .arg(dXyz.z, 0, 'g', 12);
            restoreBieFloatState();
            reset();
            return false;
        }
        if (!isReasonableReceiverXyz(solXyz)) {
            _failureReason = QString("unreasonable solution after filter: xyz=(%1,%2,%3) norm=%4 base=(%5,%6,%7) dX=(%8,%9,%10) trp=%11 trpStd=%12")
                                 .arg(solXyz.x, 0, 'f', 3)
                                 .arg(solXyz.y, 0, 'f', 3)
                                 .arg(solXyz.z, 0, 'f', 3)
                                 .arg(norm(solXyz), 0, 'f', 3)
                                 .arg(_baseXyz.x, 0, 'f', 3)
                                 .arg(_baseXyz.y, 0, 'f', 3)
                                 .arg(_baseXyz.z, 0, 'f', 3)
                                 .arg(dXyz.x, 0, 'f', 3)
                                 .arg(dXyz.y, 0, 'f', 3)
                                 .arg(dXyz.z, 0, 'f', 3)
                                 .arg(solution.trp, 0, 'g', 12)
                                 .arg(solution.trpStdev, 0, 'g', 12);
            restoreBieFloatState();
            reset();
            return false;
        }
        solution.zhdModel = saastamoinenDrySlant(kPi / 2.0);
        solution.ztdModel = solution.zhdModel;
        solution.zwdPrior = tropospherePriorZwd();
        solution.tropoPriorSource = _tropoPrior.source;
        solution.tropoPriorStatus = _tropoPrior.status;
        solution.gfsZwdPriorValid = _tropoPrior.valid &&
                                    _tropoPrior.source.startsWith(QStringLiteral("GFS"), Qt::CaseInsensitive);
        solution.ztd = solution.zhdModel + solution.trp;
        solution.ztdStdev = solution.trpStdev;
        if (!std::isfinite(solution.ztdModel) ||
            !std::isfinite(solution.ztd) ||
            !std::isfinite(solution.ztdStdev)) {
            _failureReason = QString("non-finite ZTD after filter: zhd=%1 zwd=%2 ztd=%3 ztdStd=%4")
                                 .arg(solution.zhdModel, 0, 'g', 12)
                                 .arg(solution.trp, 0, 'g', 12)
                                 .arg(solution.ztd, 0, 'g', 12)
                                 .arg(solution.ztdStdev, 0, 'g', 12)
                                 ;
            restoreBieFloatState();
            reset();
            return false;
        }
        solution.numSat = _numSat;
        solution.hDop = _hDop;
        solution.arEnabled = _opt->enablePppAr;
        solution.arAlgorithm = _opt->arAlgorithm;
        solution.arFixedSatCount = _arFixedPrns.size();
        solution.arTotalSatCount = solution.numSat;
        solution.arRatio = _arRatio;
        solution.arBootstrap = _arBootstrap;
        solution.arCandidateCount = _arCandidateCount;
        solution.arEffectiveCandidates = _arEffectiveCandidates;
        solution.arConstraintsApplied = _arConstraintsApplied;
        for (auto it = epoData.satData.constBegin(); it != epoData.satData.constEnd(); ++it) {
            if (!it.value().outlier) {
                solution.usedSatellites.push_back(it.key());
            }
        }
        solution.usedSatellites.sort();
        solution.arEligibleSatellites = _arEligiblePrns.values();
        solution.arEligibleSatellites.sort();
        solution.arFixedSatellites = _arFixedPrns.values();
        solution.arFixedSatellites.sort();
        solution.arGroups = _arGroups;
        if (!solution.arEnabled) {
            solution.arStatus = QStringLiteral("disabled");
        } else if (solution.arFixedSatellites.isEmpty()) {
            solution.arStatus = QStringLiteral("float");
        } else if (!solution.arEligibleSatellites.isEmpty() &&
                   solution.arFixedSatellites.size() >= solution.arEligibleSatellites.size()) {
            solution.arStatus = QStringLiteral("fixed");
        } else {
            solution.arStatus = QStringLiteral("partial_fix");
        }
        solution.diagSummary = _diagLines.join(QStringLiteral("; "));
        solution.observationResiduals = _observationResiduals.values().toVector();
        if (_Q.size() >= 3) {
            solution.cov[0] = _Q[0][0];
            solution.cov[1] = _Q[0][1];
            solution.cov[2] = _Q[0][2];
            solution.cov[3] = _Q[1][1];
            solution.cov[4] = _Q[1][2];
            solution.cov[5] = _Q[2][2];
        }

        std::ostringstream log;
        log.setf(std::ios::fixed);
        Vec3 refXyz{};
        if (_opt->hasApriori()) {
            refXyz = {_opt->aprXyz[0], _opt->aprXyz[1], _opt->aprXyz[2]};
            _neuRefInit = false;
        } else {
            if (!_neuRefInit) {
                _neuRefXyz = {solution.xyz[0], solution.xyz[1], solution.xyz[2]};
                _neuRefInit = true;
            }
            refXyz = _neuRefXyz;
        }
        Vec3 diff{solXyz.x - refXyz.x,
                  solXyz.y - refXyz.y,
                  solXyz.z - refXyz.z};
        Ellipsoid ellRef = xyzToEll(refXyz);
        Vec3 neu = xyzToNeu(ellRef, diff);
        solution.neu[0] = neu.x;
        solution.neu[1] = neu.y;
        solution.neu[2] = neu.z;
        if (!_bieFixedResultLogged) {
            appendResultLog(solution, neu);
        }
        for (const QString& line : _solutionLogLines) {
            log << line.toStdString() << '\n';
        }
        if (!_arSummary.isEmpty()) {
            log << " " << _arSummary.toStdString();
        }
        solution.log = QString::fromStdString(log.str());
        solution.arLogLines = _arEpochLogLines;
        restoreBieFloatState();
        _lastTimeOK = epoData.tt;
        return true;
    }

private:
    const PppOptions* _opt;
    const PppVtecModel* _vtecModel;
    PppTropospherePrior _tropoPrior;
    QVector<Param> _params;
    Mat _Q;
    PppTime _time;
    PppTime _lastTimeOK;
    PppTime _startTime;
    Vec3 _xcBanc;
    Vec3 _baseXyz;
    double _clkBanc = 0.0;
    Ellipsoid _ellBanc;
    int _numSat = 0;
    double _hDop = 0.0;
    QMap<QString, double> _windUpTime;
    QMap<QString, double> _windUpSum;
    std::unique_ptr<Gpt3Vmf3Worker> _gpt3Worker;
    AntexModel _antex;
    OceanTideModel _ocean;
    bool _antexLoaded = false;
    bool _oceanLoaded = false;
    QVector<QString> _warnings;
    QSet<QString> _warningSet;
    QStringList _solutionLogLines;
    QStringList _diagLines;
    QMap<QString, PppObservationResidual> _observationResiduals;
    bool _antApcDiagLogged = false;
    QMap<QString, int> _lastDisc1;
    QMap<QString, int> _lastDisc2;
    QMap<QString, int> _slipDisc1;
    QMap<QString, int> _slipDisc2;
    QMap<QString, int> _slipLock1;
    QMap<QString, int> _slipLock2;
    QMap<QString, PppTime> _lastObsSeen;
    struct MwState {
        int count = 0;
        double mean = 0.0;
        double m2 = 0.0;
        int lastDisc1 = -1;
        int lastDisc2 = -1;
    };
    QMap<QString, MwState> _mwStates;
    struct ArSatHistory {
        int lastLock1 = -1;
        int lastLock2 = -1;
        int lastDisc1 = -1;
        int lastDisc2 = -1;
        int lastSeenEpoch = -1;
        int arcEpochs = 0;
        int lockEpochs = 0;
        int wlFixEpochs = 0;
        int nlFixEpochs = 0;
    };
    QMap<QString, ArSatHistory> _arSatHistory;
    QString _arSummary;
    QStringList _arEpochLogLines;
    QSet<QString> _arFixedPrns;
    QSet<QString> _arEligiblePrns;
    double _arRatio = 0.0;
    double _arBootstrap = 0.0;
    int _arCandidateCount = 0;
    double _arEffectiveCandidates = 0.0;
    int _arConstraintsApplied = 0;
    QVector<PppArGroupResult> _arGroups;
    QMap<QChar, QString> _lastArFixedSignature;
    QMap<QChar, int> _lastArFixedEpoch;
    QMap<QChar, QString> _lastArReference;
    QMap<QString, QString> _lastBieReferenceByGroup;
    int _arEpochCounter = 0;
    QVector<Param> _bieFloatParams;
    Mat _bieFloatQ;
    bool _bieFixedResultLogged = false;
    bool _restoreBieState = false;

    // A BIE posterior is tied to one satellite/frequency phase arc.  Do not
    // use a single batch identity here: real-time visibility and reference
    // selection legitimately change while unaffected arcs remain continuous.
    struct BiePersistentHypothesis {
        QString prn;
        qint64 integerCandidate = 0;
        int confirmations = 0;
        int lastEpoch = -1;
    };
    QMap<QString, BiePersistentHypothesis> _biePersistent;

    struct BiePosteriorApplyDiag {
        bool applied = false;
        QString reason;
        int dimension = 0;
        double verticalCorrectionM = 0.0;
        double ztdCorrectionM = 0.0;
        double maxStateCorrection = 0.0;
        double covarianceRegularization = 0.0;
    };

    void restoreBieFloatState() {
        if (!_restoreBieState) return;
        _params = _bieFloatParams;
        _Q = _bieFloatQ;
        rebuildParamIndexCache();
        _restoreBieState = false;
        _bieFloatParams.clear();
        _bieFloatQ.clear();
    }

    static QString biePersistenceGroupKey(const BieAlgorithm::Constraint& constraint) {
        return QString("%1|%2").arg(QString(constraint.system), constraint.group);
    }

    static QString biePersistenceKey(const BieAlgorithm::Constraint& constraint) {
        return QString("%1|%2|%3")
            .arg(QString(constraint.system), constraint.group, constraint.prn);
    }

    BieAlgorithm::Result selectBiePosterior(const BieAlgorithm::Result& result,
                                      const QVector<int>& selectedRows) const {
        BieAlgorithm::Result selected;
        if (selectedRows.isEmpty()) return selected;
        const int count = result.constraints.size();
        if (result.posteriorStateIndices.size() != count ||
            result.posteriorMean.size() != count ||
            result.posteriorCovariance.size() != count) {
            return selected;
        }
        selected.constraints.reserve(selectedRows.size());
        selected.posteriorStateIndices.reserve(selectedRows.size());
        selected.posteriorMean.reserve(selectedRows.size());
        selected.posteriorCovariance = Mat(selectedRows.size(), Vec(selectedRows.size(), 0.0));
        for (int row = 0; row < selectedRows.size(); ++row) {
            const int sourceRow = selectedRows[row];
            if (sourceRow < 0 || sourceRow >= count ||
                result.posteriorCovariance[sourceRow].size() != count) {
                return BieAlgorithm::Result();
            }
            selected.constraints.push_back(result.constraints[sourceRow]);
            selected.posteriorStateIndices.push_back(result.posteriorStateIndices[sourceRow]);
            selected.posteriorMean.push_back(result.posteriorMean[sourceRow]);
            for (int col = 0; col < selectedRows.size(); ++col) {
                const int sourceCol = selectedRows[col];
                if (sourceCol < 0 || sourceCol >= count) return BieAlgorithm::Result();
                selected.posteriorCovariance[row][col] =
                    result.posteriorCovariance[sourceRow][sourceCol];
            }
        }
        return selected;
    }

    QVector<int> advanceBiePersistence(const BieAlgorithm::Result& result,
                                        QStringList& arLines) {
        const int required = qMax(1, _opt->arBiePersistConfirmEpochs);
        const int minStablePerGroup = qMax(1, _opt->arBieMinSat - 1);
        QMap<QString, QVector<int>> allRowsByGroup;
        QMap<QString, QVector<int>> stableRowsByGroup;
        for (int row = 0; row < result.constraints.size(); ++row) {
            const BieAlgorithm::Constraint& constraint = result.constraints[row];
            if (constraint.prn.isEmpty() || !std::isfinite(constraint.value)) continue;
            const QString key = biePersistenceKey(constraint);
            const qint64 integerCandidate = std::llround(constraint.value);
            BiePersistentHypothesis& hypothesis = _biePersistent[key];
            const bool contiguous = hypothesis.lastEpoch == _arEpochCounter - 1;
            if (contiguous && hypothesis.integerCandidate == integerCandidate) {
                ++hypothesis.confirmations;
            } else {
                hypothesis.confirmations = 1;
                hypothesis.integerCandidate = integerCandidate;
            }
            hypothesis.prn = constraint.prn;
            hypothesis.lastEpoch = _arEpochCounter;

            const QString groupKey = biePersistenceGroupKey(constraint);
            allRowsByGroup[groupKey].push_back(row);
            if (hypothesis.confirmations >= required) {
                stableRowsByGroup[groupKey].push_back(row);
            }
        }

        QVector<int> selectedRows;
        for (auto it = allRowsByGroup.constBegin(); it != allRowsByGroup.constEnd(); ++it) {
            const QVector<int> stableRows = stableRowsByGroup.value(it.key());
            if (stableRows.size() >= minStablePerGroup) {
                selectedRows += stableRows;
                arLines << QString("BIE PERSISTENCE group=%1 status=ready stable=%2 active=%3 required=%4")
                               .arg(it.key())
                               .arg(stableRows.size())
                               .arg(it.value().size())
                               .arg(required);
            } else {
                arLines << QString("BIE PERSISTENCE group=%1 status=pending stable=%2 active=%3 minStable=%4 required=%5")
                               .arg(it.key())
                               .arg(stableRows.size())
                               .arg(it.value().size())
                               .arg(minStablePerGroup)
                               .arg(required);
            }
        }
        return selectedRows;
    }

    void clearBiePersistenceForPrn(const QString& prn) {
        if (prn.isEmpty()) return;
        int cleared = 0;
        for (auto it = _biePersistent.begin(); it != _biePersistent.end();) {
            if (it->prn == prn) {
                it = _biePersistent.erase(it);
                ++cleared;
            } else {
                ++it;
            }
        }
        if (cleared > 0) {
            _solutionLogLines << QString("%1 BIE PERSISTENCE cleared sat=%2 hypotheses=%3 reason=arc_reset")
                                .arg(epochTimeLabel(), prn)
                                .arg(cleared);
        }
    }

    bool calculateBiePosteriorUpdate(const BieAlgorithm::Result& result,
                                     QVector<Param>& updatedParams,
                                     Mat& updatedCovariance,
                                     BiePosteriorApplyDiag& diag) const {
        const int nPar = _params.size();
        const int count = result.posteriorStateIndices.size();
        diag.dimension = count;
        if (count == 0 || result.posteriorMean.size() != count ||
            result.posteriorCovariance.size() != count || nPar == 0 || _Q.size() != nPar) {
            diag.reason = QStringLiteral("invalid_posterior_dimensions");
            return false;
        }

        Mat targetCovariance = result.posteriorCovariance;
        QSet<int> indices;
        Vec currentMean(count, 0.0);
        Mat priorCovariance = zeros(count, count);
        Mat crossCovariance = zeros(nPar, count);
        for (int row = 0; row < count; ++row) {
            const int index = result.posteriorStateIndices[row];
            if (index < 0 || index >= nPar || indices.contains(index) ||
                targetCovariance[row].size() != count ||
                !std::isfinite(result.posteriorMean[row])) {
                diag.reason = QStringLiteral("invalid_posterior_state_index");
                return false;
            }
            indices.insert(index);
            currentMean[row] = paramAt(index);
            for (int col = 0; col < count; ++col) {
                const int other = result.posteriorStateIndices[col];
                if (other < 0 || other >= nPar || targetCovariance[col].size() != count ||
                    !std::isfinite(targetCovariance[row][col])) {
                    diag.reason = QStringLiteral("invalid_posterior_covariance");
                    return false;
                }
                priorCovariance[row][col] = _Q[index][other];
                if (row == col && targetCovariance[row][col] > priorCovariance[row][col] + 1e-10) {
                    diag.reason = QStringLiteral("posterior_not_tighter_than_float");
                    return false;
                }
            }
            for (int state = 0; state < nPar; ++state) {
                crossCovariance[state][row] = _Q[state][index];
            }
        }
        sanitizeCovariance(targetCovariance);

        // A posterior may only remove ambiguity uncertainty.  Checking the
        // diagonal alone is insufficient because a non-PSD covariance
        // reduction can still have smaller marginal variances.
        Mat covarianceReductionCheck = zeros(count, count);
        double covarianceScale = 1.0;
        for (int row = 0; row < count; ++row) {
            for (int col = 0; col < count; ++col) {
                const double value = priorCovariance[row][col] - targetCovariance[row][col];
                if (!std::isfinite(value)) {
                    diag.reason = QStringLiteral("posterior_nonfinite_covariance_reduction");
                    return false;
                }
                covarianceReductionCheck[row][col] = value;
                covarianceScale = std::max(covarianceScale, std::fabs(priorCovariance[row][col]));
            }
        }
        const double psdTolerance = covarianceScale * 1.0e-10;
        for (int row = 0; row < count; ++row) {
            for (int col = row + 1; col < count; ++col) {
                const double symmetric = 0.5 * (covarianceReductionCheck[row][col] +
                                                covarianceReductionCheck[col][row]);
                covarianceReductionCheck[row][col] = symmetric;
                covarianceReductionCheck[col][row] = symmetric;
            }
            if (covarianceReductionCheck[row][row] < -psdTolerance) {
                diag.reason = QStringLiteral("posterior_covariance_not_tighter_than_float");
                return false;
            }
            covarianceReductionCheck[row][row] += psdTolerance;
        }
        Mat reductionCholesky;
        if (!choleskyDecompose(covarianceReductionCheck, reductionCholesky)) {
            diag.reason = QStringLiteral("posterior_covariance_not_tighter_than_float");
            return false;
        }

        Mat inversePrior;
        if (!invert(priorCovariance, inversePrior)) {
            diag.reason = QStringLiteral("posterior_prior_inverse_failed");
            return false;
        }
        const Mat gain = multiply(crossCovariance, inversePrior);
        Vec innovation(count, 0.0);
        for (int i = 0; i < count; ++i) {
            innovation[i] = result.posteriorMean[i] - currentMean[i];
        }
        const Vec correction = multiply(gain, innovation);
        if (correction.size() != nPar) {
            diag.reason = QStringLiteral("posterior_correction_failed");
            return false;
        }

        updatedParams = _params;
        for (Param& parameter : updatedParams) {
            const int index = parameter.index - 1;
            if (index < 0 || index >= correction.size() || !std::isfinite(correction[index])) {
                diag.reason = QStringLiteral("posterior_nonfinite_correction");
                return false;
            }
            parameter.value += correction[index];
            diag.maxStateCorrection = std::max(diag.maxStateCorrection, std::fabs(correction[index]));
        }

        Mat covarianceReduction = priorCovariance;
        for (int row = 0; row < count; ++row) {
            for (int col = 0; col < count; ++col) {
                covarianceReduction[row][col] -= targetCovariance[row][col];
            }
        }
        const Mat covarianceCorrection = multiply(multiply(gain, covarianceReduction), transpose(gain));
        if (covarianceCorrection.size() != nPar) {
            diag.reason = QStringLiteral("posterior_covariance_correction_failed");
            return false;
        }
        updatedCovariance = _Q;
        for (int row = 0; row < nPar; ++row) {
            if (covarianceCorrection[row].size() != nPar) {
                diag.reason = QStringLiteral("posterior_covariance_shape_failed");
                return false;
            }
            for (int col = 0; col < nPar; ++col) {
                updatedCovariance[row][col] -= covarianceCorrection[row][col];
            }
        }
        sanitizeCovariance(updatedCovariance);
        Mat cholesky;
        if (!choleskyDecompose(updatedCovariance, cholesky)) {
            // The algebra above is PSD when Qaa-QBIE is PSD.  A failed
            // Cholesky factorization here can therefore be round-off from
            // subtracting strongly correlated, differently scaled states.
            // Only a scale-bounded diagonal loading is allowed; a material
            // indefiniteness remains a rejected persistence transition.
            double maxVariance = 1.0;
            for (int row = 0; row < nPar; ++row) {
                maxVariance = std::max(maxVariance, std::fabs(updatedCovariance[row][row]));
            }
            const double maximumLoading = maxVariance * 1.0e-7;
            double loading = std::max(1.0e-12, maxVariance * 1.0e-12);
            bool regularized = false;
            while (loading <= maximumLoading) {
                Mat candidate = updatedCovariance;
                for (int row = 0; row < nPar; ++row) candidate[row][row] += loading;
                if (choleskyDecompose(candidate, cholesky)) {
                    updatedCovariance = candidate;
                    diag.covarianceRegularization = loading;
                    regularized = true;
                    break;
                }
                loading *= 10.0;
            }
            if (!regularized) {
                diag.reason = QStringLiteral("posterior_covariance_not_spd");
                return false;
            }
        }

        auto valueFor = [](const QVector<Param>& parameters, ParamType type) {
            for (const Param& parameter : parameters) {
                if (parameter.type == type) return parameter.value;
            }
            return 0.0;
        };
        const Vec3 xyzBefore{_baseXyz.x + valueFor(_params, CRD_X),
                             _baseXyz.y + valueFor(_params, CRD_Y),
                             _baseXyz.z + valueFor(_params, CRD_Z)};
        const Vec3 xyzAfter{_baseXyz.x + valueFor(updatedParams, CRD_X),
                            _baseXyz.y + valueFor(updatedParams, CRD_Y),
                            _baseXyz.z + valueFor(updatedParams, CRD_Z)};
        const Vec3 delta{xyzAfter.x - xyzBefore.x, xyzAfter.y - xyzBefore.y, xyzAfter.z - xyzBefore.z};
        diag.verticalCorrectionM = xyzToNeu(xyzToEll(xyzBefore), delta).z;
        diag.ztdCorrectionM = valueFor(updatedParams, TROPO) - valueFor(_params, TROPO);
        if (!std::isfinite(diag.verticalCorrectionM) || !std::isfinite(diag.ztdCorrectionM)) {
            diag.reason = QStringLiteral("posterior_nonfinite_transition");
            return false;
        }
        diag.applied = true;
        return true;
    }
    QString _failureReason;
    bool _neuRefInit = false;
    Vec3 _neuRefXyz;
    QMap<int, int> _idxScalar;
    QMap<QString, int> _idxIono;
    QMap<QString, int> _idxAmb1;
    QMap<QString, int> _idxAmb2;
    QMap<QString, int> _idxAmbIf;

    void warnAntexNotFound(const QString& antenna, const QString& freq) {
        const QString msg = QString("ANTEX: antenna >%1 %2< not found").arg(antenna, freq);
        if (!_warningSet.contains(msg)) {
            _warningSet.insert(msg);
            _warnings.push_back(msg);
        }
    }

    QString epochTimeLabel() const {
        return GnssUtils::gpsCalendarDateTime(_time.gpsWeek, _time.gpsSow)
            .toString(QStringLiteral("yyyy-MM-dd HH:mm:ss.zzz"));
    }

    static QString obsKindText(ObsKind kind) {
        switch (kind) {
        case CODE1: return QStringLiteral("c1");
        case CODE2: return QStringLiteral("c2");
        case CODE_IF: return QStringLiteral("cIF");
        case PHASE1: return QStringLiteral("l1");
        case PHASE2: return QStringLiteral("l2");
        case PHASE_IF: return QStringLiteral("lIF");
        case GIM: return QStringLiteral("GIM");
        }
        return QString();
    }

    static ObsKind obsKindForLc(ObservationCombination lc) {
        switch (lc) {
        case ObservationCombination::C1: return CODE1;
        case ObservationCombination::C2: return CODE2;
        case ObservationCombination::CIf: return CODE_IF;
        case ObservationCombination::L1: return PHASE1;
        case ObservationCombination::L2: return PHASE2;
        case ObservationCombination::LIf: return PHASE_IF;
        case ObservationCombination::Gim: return GIM;
        case ObservationCombination::Dummy:
        default: break;
        }
        return CODE1;
    }

    static ParamType ambParamTypeForLc(ObservationCombination lc) {
        if (lc == ObservationCombination::L1) return AMB_L1;
        if (lc == ObservationCombination::L2) return AMB_L2;
        return AMB_IF;
    }

    static QString paramText(const Param& par) {
        switch (par.type) {
        case CRD_X: return QStringLiteral("CRD_X");
        case CRD_Y: return QStringLiteral("CRD_Y");
        case CRD_Z: return QStringLiteral("CRD_Z");
        case CLK_G: return QStringLiteral("REC_CLK  G  ");
        case CLK_R: return QStringLiteral("REC_CLK  R  ");
        case CLK_E: return QStringLiteral("REC_CLK  E  ");
        case CLK_C: return QStringLiteral("REC_CLK  C  ");
        case TROPO: return QStringLiteral("TRP         ");
        case IONO_L1: return QStringLiteral("ION          ") + par.prn;
        case AMB_L1: return QStringLiteral("AMB  l1  ") + par.prn;
        case AMB_L2: return QStringLiteral("AMB  l2  ") + par.prn;
        case AMB_IF: return QStringLiteral("AMB  lIF ") + par.prn;
        case CBIAS_G1: case PBIAS_G1:
        case CBIAS_G2: case PBIAS_G2:
            return QStringLiteral("BIA  ") + combinationLabel(par.lc).leftJustified(3, QLatin1Char(' ')) + QStringLiteral(" G  ");
        case CBIAS_R1: case PBIAS_R1:
        case CBIAS_R2: case PBIAS_R2:
            return QStringLiteral("BIA  ") + combinationLabel(par.lc).leftJustified(3, QLatin1Char(' ')) + QStringLiteral(" R  ");
        case CBIAS_E1: case PBIAS_E1:
        case CBIAS_E2: case PBIAS_E2:
            return QStringLiteral("BIA  ") + combinationLabel(par.lc).leftJustified(3, QLatin1Char(' ')) + QStringLiteral(" E  ");
        case CBIAS_C1: case PBIAS_C1:
        case CBIAS_C2: case PBIAS_C2:
            return QStringLiteral("BIA  ") + combinationLabel(par.lc).leftJustified(3, QLatin1Char(' ')) + QStringLiteral(" C  ");
        }
        return QString();
    }

    QString scalarParamDiag(ParamType type, const QString& label) const {
        const auto it = _idxScalar.constFind(static_cast<int>(type));
        if (it == _idxScalar.constEnd() || it.value() < 0 || it.value() >= _params.size()) {
            return QString();
        }
        const Param& par = _params[it.value()];
        return QString("%1=%2").arg(label, QString::number(par.value, 'f', 4));
    }

    QString filterStateDiag() const {
        QStringList parts;
        parts << QString("bancClk=%1m").arg(_clkBanc, 0, 'f', 3);
        const QString clkG = scalarParamDiag(CLK_G, QStringLiteral("clkG"));
        const QString clkR = scalarParamDiag(CLK_R, QStringLiteral("clkR"));
        const QString clkE = scalarParamDiag(CLK_E, QStringLiteral("clkE"));
        const QString clkC = scalarParamDiag(CLK_C, QStringLiteral("clkC"));
        if (!clkG.isEmpty()) parts << clkG;
        if (!clkR.isEmpty()) parts << clkR;
        if (!clkE.isEmpty()) parts << clkE;
        if (!clkC.isEmpty()) parts << clkC;
        const QString trp = scalarParamDiag(TROPO, QStringLiteral("trp"));
        if (!trp.isEmpty()) parts << trp;
        const int ix = paramIndex(CRD_X);
        const int iy = paramIndex(CRD_Y);
        const int iz = paramIndex(CRD_Z);
        if (ix >= 0 && iy >= 0 && iz >= 0) {
            parts << QString("dXYZ=(%1,%2,%3)")
                         .arg(_params[ix].value, 0, 'f', 3)
                         .arg(_params[iy].value, 0, 'f', 3)
                         .arg(_params[iz].value, 0, 'f', 3);
        }
        return parts.join(QLatin1Char(' '));
    }

    void appendPrefitDiag(QChar system, const QVector<QString>& prns,
                          const QVector<ObsKind>& kinds,
                          const Vec& prefit) {
        struct Stats {
            int n = 0;
            double minVal = 0.0;
            double maxVal = 0.0;
            double maxAbs = 0.0;
            QString maxPrn;
        };
        QMap<int, Stats> byKind;
        for (int i = 0; i < prefit.size() && i < kinds.size(); ++i) {
            const double val = prefit[i];
            if (!std::isfinite(val)) {
                continue;
            }
            Stats& st = byKind[static_cast<int>(kinds[i])];
            if (st.n == 0) {
                st.minVal = val;
                st.maxVal = val;
                st.maxAbs = std::fabs(val);
                if (i < prns.size()) st.maxPrn = prns[i];
            } else {
                st.minVal = qMin(st.minVal, val);
                st.maxVal = qMax(st.maxVal, val);
                if (std::fabs(val) > st.maxAbs) {
                    st.maxAbs = std::fabs(val);
                    if (i < prns.size()) st.maxPrn = prns[i];
                }
            }
            ++st.n;
        }
        QStringList parts;
        for (auto it = byKind.constBegin(); it != byKind.constEnd(); ++it) {
            const ObsKind kind = static_cast<ObsKind>(it.key());
            const Stats& st = it.value();
            parts << QString("%1:n=%2 min=%3 max=%4 maxAbs=%5@%6")
                         .arg(obsKindText(kind))
                         .arg(st.n)
                         .arg(st.minVal, 0, 'f', 3)
                         .arg(st.maxVal, 0, 'f', 3)
                         .arg(st.maxAbs, 0, 'f', 3)
                         .arg(st.maxPrn);
        }
        if (!parts.isEmpty()) {
            _diagLines << QString("prefit %1 %2").arg(QString(system), parts.join(QStringLiteral(" | ")));
        }
    }

    void appendPostfitDiag(QChar system, const QVector<QString>& prns,
                           const QVector<ObsKind>& kinds,
                           const Vec& residuals) {
        struct MaxRes {
            double absVal = 0.0;
            double val = 0.0;
            QString prn;
            ObsKind kind = CODE1;
            int n = 0;
        } maxRes;
        for (int i = 0; i < residuals.size() && i < kinds.size(); ++i) {
            const double val = residuals[i];
            if (!std::isfinite(val)) {
                continue;
            }
            ++maxRes.n;
            if (std::fabs(val) > maxRes.absVal) {
                maxRes.absVal = std::fabs(val);
                maxRes.val = val;
                maxRes.kind = kinds[i];
                if (i < prns.size()) maxRes.prn = prns[i];
            }
        }
        if (maxRes.n > 0) {
            _diagLines << QString("postfit %1 n=%2 max=%3 %4@%5 state{%6}")
                              .arg(QString(system))
                              .arg(maxRes.n)
                              .arg(maxRes.val, 0, 'f', 4)
                              .arg(obsKindText(maxRes.kind), maxRes.prn,
                                   filterStateDiag());
        }
    }

    static bool isPhaseKind(ObsKind kind) {
        return kind == PHASE1 || kind == PHASE2 || kind == PHASE_IF;
    }

    void appendObservationResiduals(const QVector<ObsRow>& rows,
                                    const Vec& prefit,
                                    const Vec& postfit,
                                    const Vec& sigma,
                                    const PppEpochData& epoData,
                                    bool adopted,
                                    const QString& reason,
                                    const QString& onlySatellite = QString(),
                                    int onlyKind = -1) {
        for (int i = 0; i < rows.size() && i < prefit.size() &&
                        i < postfit.size() && i < sigma.size(); ++i) {
            const ObsRow& row = rows[i];
            if ((!onlySatellite.isEmpty() && row.prn != onlySatellite)
                || (onlyKind >= 0 && static_cast<int>(row.kind) != onlyKind)) {
                continue;
            }
            const auto satellite = epoData.satData.constFind(row.prn);
            if (satellite == epoData.satData.constEnd()) {
                continue;
            }
            PppObservationResidual residual;
            residual.satellite = row.prn;
            residual.signal = combinationLabel(row.lc);
            residual.observationType = isPhaseKind(row.kind)
                ? QStringLiteral("phase")
                : (row.kind == GIM ? QStringLiteral("iono") : QStringLiteral("code"));
            residual.prefitM = prefit[i];
            residual.postfitM = postfit[i];
            residual.sigmaM = sigma[i];
            residual.elevationDeg = satellite.value().eleSat * kRhoDeg;
            residual.adopted = adopted;
            residual.reason = reason;
            const QString key = residual.satellite + QLatin1Char('|') +
                                residual.observationType + QLatin1Char('|') + residual.signal;
            _observationResiduals.insert(key, residual);
        }
    }

    bool findObservationOutlier(int iPhase,
                        const QVector<ObsRow>& usedRows,
                        const Vec& residuals,
                        const PppEpochData& epoData,
                        OutlierInfo& outlier) const {
        outlier.prn.clear();
        double maxOutlierAbs = 0.0;
        double maxOutlier = 0.0;
        int maxOutlierIndex = -1;

        for (int i = 0; i < residuals.size() && i < usedRows.size(); ++i) {
            const ObsRow& obs = usedRows[i];
            const bool phaseRow = isPhaseKind(obs.kind);
            if ((iPhase == 1 && !phaseRow) || (iPhase == 0 && phaseRow)) {
                continue;
            }
            const auto itSat = epoData.satData.constFind(obs.prn);
            if (itSat == epoData.satData.constEnd()) {
                continue;
            }
            const double res = residuals[i];
            if (!std::isfinite(res)) {
                continue;
            }
            const double absRes = std::fabs(res);
            const double threshold = obsMaxRes(itSat.value(), obs.kind);
            if (threshold > 0.0 && absRes > threshold && absRes > maxOutlierAbs) {
                maxOutlierAbs = absRes;
                maxOutlier = res;
                maxOutlierIndex = i;
            }
        }

        if (maxOutlierIndex >= 0 && maxOutlierIndex < usedRows.size()) {
            const ObsRow& obs = usedRows[maxOutlierIndex];
            outlier.prn = obs.prn;
            outlier.kind = obs.kind;
            outlier.residual = maxOutlier;
            return true;
        }
        return false;
    }

    void appendAllSystemPrefitDiag(const QVector<QString>& prns,
                                   const QVector<ObsKind>& kinds,
                                   const Vec& prefit) {
        const QVector<QChar> systemOrder = {QLatin1Char('G'), QLatin1Char('R'),
                                            QLatin1Char('E'), QLatin1Char('C')};
        for (const QChar& sys : systemOrder) {
            QVector<QString> sysPrns;
            QVector<ObsKind> sysKinds;
            Vec sysPrefit;
            for (int i = 0; i < prefit.size() && i < prns.size() && i < kinds.size(); ++i) {
                if (!prns[i].isEmpty() && prns[i].at(0) == sys) {
                    sysPrns.push_back(prns[i]);
                    sysKinds.push_back(kinds[i]);
                    sysPrefit.push_back(prefit[i]);
                }
            }
            if (!sysPrns.isEmpty()) {
                appendPrefitDiag(sys, sysPrns, sysKinds, sysPrefit);
            }
        }
    }

    void appendAllSystemPostfitDiag(const QVector<QString>& prns,
                                    const QVector<ObsKind>& kinds,
                                    const Vec& residuals) {
        const QVector<QChar> systemOrder = {QLatin1Char('G'), QLatin1Char('R'),
                                            QLatin1Char('E'), QLatin1Char('C')};
        for (const QChar& sys : systemOrder) {
            QVector<QString> sysPrns;
            QVector<ObsKind> sysKinds;
            Vec sysResiduals;
            for (int i = 0; i < residuals.size() && i < prns.size() && i < kinds.size(); ++i) {
                if (!prns[i].isEmpty() && prns[i].at(0) == sys) {
                    sysPrns.push_back(prns[i]);
                    sysKinds.push_back(kinds[i]);
                    sysResiduals.push_back(residuals[i]);
                }
            }
            if (!sysPrns.isEmpty()) {
                appendPostfitDiag(sys, sysPrns, sysKinds, sysResiduals);
            }
        }
    }

    void appendResultLog(const PppSolution& solution, const Vec3& neu) {
        const QString t = epochTimeLabel();
        for (const Param& par : _params) {
            if (par.type == CRD_X || par.type == CRD_Y || par.type == CRD_Z) {
                continue;
            }
            const int idx = par.index - 1;
            if (idx < 0 || idx >= _Q.size()) {
                continue;
            }
            const double stdev = std::sqrt(qMax(0.0, _Q[idx][idx]));
            const double apr = par.x0;
            _solutionLogLines << QString("%1 %2 %3 %4 +- %5")
                .arg(t,
                     paramText(par),
                     QString::number(apr, 'f', 4),
                     QString::number(par.value, 'f', 4),
                     QString::number(stdev, 'f', 4));
        }
        _solutionLogLines << QString("%1 ROVER X = %2 Y = %3 Z = %4 dN = %5 dE = %6 dU = %7")
            .arg(t,
                 QString::number(solution.xyz[0], 'f', 4),
                 QString::number(solution.xyz[1], 'f', 4),
                 QString::number(solution.xyz[2], 'f', 4),
                 QString::number(neu.x, 'f', 4),
                 QString::number(neu.y, 'f', 4),
                 QString::number(neu.z, 'f', 4));
    }

    QString appendBieFixedResultLog() {
        PppSolution fixedSolution;
        const Vec3 delta{paramValue(CRD_X), paramValue(CRD_Y), paramValue(CRD_Z)};
        const Vec3 xyz = _baseXyz + delta;
        fixedSolution.xyz[0] = xyz.x;
        fixedSolution.xyz[1] = xyz.y;
        fixedSolution.xyz[2] = xyz.z;
        fixedSolution.zhdModel = saastamoinenDrySlant(kPi / 2.0);
        fixedSolution.ztdModel = fixedSolution.zhdModel;
        fixedSolution.zwdPrior = tropospherePriorZwd();
        fixedSolution.tropoPriorSource = _tropoPrior.source;
        fixedSolution.tropoPriorStatus = _tropoPrior.status;
        fixedSolution.trp = paramEstimate(TROPO);
        fixedSolution.trpStdev = paramStd(TROPO);
        fixedSolution.ztd = fixedSolution.zhdModel + fixedSolution.trp;
        fixedSolution.ztdStdev = fixedSolution.trpStdev;

        Vec3 reference = xyz;
        if (_opt->hasApriori()) {
            reference = {_opt->aprXyz[0], _opt->aprXyz[1], _opt->aprXyz[2]};
        } else if (_neuRefInit) {
            reference = _neuRefXyz;
        }
        const Vec3 difference{xyz.x - reference.x,
                              xyz.y - reference.y,
                              xyz.z - reference.z};
        const Vec3 neu = xyzToNeu(xyzToEll(reference), difference);
        appendResultLog(fixedSolution, neu);
        _bieFixedResultLogged = true;

        return QString("BIE FIXED_SOLUTION X=%1 Y=%2 Z=%3 dN=%4 dE=%5 dU=%6 ZTD=%7 ZTD_SIGMA=%8")
            .arg(xyz.x, 0, 'f', 4)
            .arg(xyz.y, 0, 'f', 4)
            .arg(xyz.z, 0, 'f', 4)
            .arg(neu.x, 0, 'f', 4)
            .arg(neu.y, 0, 'f', 4)
            .arg(neu.z, 0, 'f', 4)
            .arg(fixedSolution.ztd, 0, 'f', 6)
            .arg(fixedSolution.ztdStdev, 0, 'f', 6);
    }

    bool findObsStateIndices(const QString& prn, int& ionIdx, int& amb1Idx, int& amb2Idx) const {
        ionIdx = -1;
        amb1Idx = -1;
        amb2Idx = -1;
        for (const Param& p : _params) {
            if (p.prn != prn) {
                continue;
            }
            if (p.type == IONO_L1) {
                ionIdx = p.index - 1;
            } else if (p.type == AMB_L1) {
                amb1Idx = p.index - 1;
            } else if (p.type == AMB_L2) {
                amb2Idx = p.index - 1;
            }
        }
        return ionIdx >= 0;
    }

    double paramAt(int idx) const {
        for (const Param& p : _params) {
            if (p.index - 1 == idx) {
                return p.value;
            }
        }
        return 0.0;
    }

    double paramEstimateAt(int idx) const {
        for (const Param& p : _params) {
            if (p.index - 1 == idx) {
                return p.x0 + p.value;
            }
        }
        return 0.0;
    }

    void resetArSatHistory(const QString& prn) {
        _mwStates.remove(prn);
        _arSatHistory.remove(prn);
        clearBiePersistenceForPrn(prn);
        if (!prn.isEmpty()) {
            const QChar sys = prn.at(0);
            _lastArFixedSignature.remove(sys);
            _lastArFixedEpoch.remove(sys);
            if (_lastArReference.value(sys) == prn) {
                _lastArReference.remove(sys);
            }
            for (auto it = _lastBieReferenceByGroup.begin(); it != _lastBieReferenceByGroup.end();) {
                if (it.value() == prn) {
                    it = _lastBieReferenceByGroup.erase(it);
                } else {
                    ++it;
                }
            }
        }
    }

    ArSatHistory& updateArSatHistory(const PppSatData& sat,
                                     QString* arcBreakReason = nullptr,
                                     bool* arcReset = nullptr) {
        ArSatHistory& hist = _arSatHistory[sat.prn];
        bool arcBreak = sat.slipFlag;
        QString breakReason = sat.slipFlag ? QStringLiteral("slip_flag") : QString();
        if (!arcBreak && hist.lastSeenEpoch >= 0 && _arEpochCounter > hist.lastSeenEpoch + 1) {
            arcBreak = true;
            breakReason = QStringLiteral("epoch_gap");
        }
        if (!arcBreak && sat.disc1 >= 0 && hist.lastDisc1 >= 0 && sat.disc1 != hist.lastDisc1) {
            arcBreak = true;
            breakReason = QStringLiteral("disc1_change");
        }
        if (!arcBreak && sat.disc2 >= 0 && hist.lastDisc2 >= 0 && sat.disc2 != hist.lastDisc2) {
            arcBreak = true;
            breakReason = QStringLiteral("disc2_change");
        }
        if (!arcBreak && sat.lock1 >= 0 && hist.lastLock1 >= 0 && sat.lock1 < hist.lastLock1) {
            arcBreak = true;
            breakReason = QStringLiteral("lock1_reset");
        }
        if (!arcBreak && sat.lock2 >= 0 && hist.lastLock2 >= 0 && sat.lock2 < hist.lastLock2) {
            arcBreak = true;
            breakReason = QStringLiteral("lock2_reset");
        }
        if (arcBreakReason) {
            *arcBreakReason = breakReason;
        }
        if (arcReset) {
            *arcReset = arcBreak;
        }
        if (arcBreak) {
            hist = ArSatHistory();
        }

        hist.lastSeenEpoch = _arEpochCounter;
        if (sat.disc1 >= 0) hist.lastDisc1 = sat.disc1;
        if (sat.disc2 >= 0) hist.lastDisc2 = sat.disc2;
        if (sat.lock1 >= 0) hist.lastLock1 = sat.lock1;
        if (sat.lock2 >= 0) hist.lastLock2 = sat.lock2;

        hist.arcEpochs = qMax(1, hist.arcEpochs + 1);
        int lockEpochs = -1;
        if (sat.lock1 >= 0 && sat.lock2 >= 0) {
            lockEpochs = qMin(sat.lock1, sat.lock2);
        } else if (sat.lock1 >= 0) {
            lockEpochs = sat.lock1;
        } else if (sat.lock2 >= 0) {
            lockEpochs = sat.lock2;
        }
        hist.lockEpochs = (lockEpochs >= 0) ? lockEpochs : hist.arcEpochs;
        return hist;
    }

    void updateArFixHistory(const QVector<AmbFix::Sat>& sats,
                            const QVector<AmbFix::FixedSet>& fixedSets,
                            QStringList& arLines) {
        QSet<QString> activePrns;
        for (const AmbFix::Sat& sat : sats) {
            activePrns.insert(sat.prn);
        }

        QSet<QString> fixedPrns;
        for (const AmbFix::FixedSet& fixedSet : fixedSets) {
            for (const QString& prn : fixedSet.fixedPrns) {
                if (!prn.isEmpty()) {
                    fixedPrns.insert(prn);
                }
            }
        }

        for (const QString& prn : activePrns) {
            ArSatHistory& hist = _arSatHistory[prn];
            if (fixedPrns.contains(prn)) {
                ++hist.wlFixEpochs;
                ++hist.nlFixEpochs;
            } else {
                hist.wlFixEpochs = 0;
                hist.nlFixEpochs = 0;
            }
            arLines << QString("AR HIST SAT=%1 lock=%2 wlFix=%3 nlFix=%4 arc=%5 fixedNow=%6")
                           .arg(prn)
                           .arg(hist.lockEpochs)
                           .arg(hist.wlFixEpochs)
                           .arg(hist.nlFixEpochs)
                           .arg(hist.arcEpochs)
                           .arg(fixedPrns.contains(prn) ? "yes" : "no");
        }
    }

    struct ConstraintApplyDiag {
        bool applied = false;
        double hxBefore = 0.0;
        double residualBefore = 0.0;
        double hxAfter = 0.0;
        double residualAfter = 0.0;
        double denom = 0.0;
        double maxStateDelta = 0.0;
        double minVarBefore = std::numeric_limits<double>::quiet_NaN();
        double minVarAfter = std::numeric_limits<double>::quiet_NaN();
        double maxVarBefore = std::numeric_limits<double>::quiet_NaN();
        double maxVarAfter = std::numeric_limits<double>::quiet_NaN();
        QStringList paramParts;
    };

    struct ConstraintApplySummary {
        int count = 0;
        int applied = 0;
        double maxAbsResidualBefore = 0.0;
        double maxAbsResidualAfter = 0.0;
        double maxStateDelta = 0.0;
        double minDenom = std::numeric_limits<double>::infinity();
        double maxDenom = -std::numeric_limits<double>::infinity();
        double minVarBefore = std::numeric_limits<double>::infinity();
        double maxVarBefore = -std::numeric_limits<double>::infinity();
        double minVarAfter = std::numeric_limits<double>::infinity();
        double maxVarAfter = -std::numeric_limits<double>::infinity();

        void add(const ConstraintApplyDiag& diag) {
            ++count;
            if (diag.applied) {
                ++applied;
            }
            maxAbsResidualBefore = std::max(maxAbsResidualBefore, std::fabs(diag.residualBefore));
            maxAbsResidualAfter = std::max(maxAbsResidualAfter, std::fabs(diag.residualAfter));
            maxStateDelta = std::max(maxStateDelta, diag.maxStateDelta);
            if (std::isfinite(diag.denom)) {
                minDenom = std::min(minDenom, diag.denom);
                maxDenom = std::max(maxDenom, diag.denom);
            }
            if (std::isfinite(diag.minVarBefore)) {
                minVarBefore = std::min(minVarBefore, diag.minVarBefore);
            }
            if (std::isfinite(diag.maxVarBefore)) {
                maxVarBefore = std::max(maxVarBefore, diag.maxVarBefore);
            }
            if (std::isfinite(diag.minVarAfter)) {
                minVarAfter = std::min(minVarAfter, diag.minVarAfter);
            }
            if (std::isfinite(diag.maxVarAfter)) {
                maxVarAfter = std::max(maxVarAfter, diag.maxVarAfter);
            }
        }
    };

    ConstraintApplyDiag applyConstraint(const QMap<int, double>& coeff, double z, double sigma,
                                        bool correctionStateOnly = false) {
        ConstraintApplyDiag diag;
        const int nPar = _params.size();
        if (coeff.isEmpty() || nPar == 0) return diag;
        Vec h(nPar, 0.0);
        for (auto it = coeff.constBegin(); it != coeff.constEnd(); ++it) {
            if (it.key() >= 0 && it.key() < nPar) {
                h[it.key()] = it.value();
            }
        }

        Vec qh(nPar, 0.0);
        for (int i = 0; i < nPar; ++i) {
            double sum = 0.0;
            for (int j = 0; j < nPar; ++j) {
                sum += _Q[i][j] * h[j];
            }
            qh[i] = sum;
        }

        double denom = 0.0;
        for (int i = 0; i < nPar; ++i) {
            denom += h[i] * qh[i];
        }
        const double r = sigma * sigma;
        denom += r;
        diag.denom = denom;
        if (denom <= 0.0) return diag;

        double hx = 0.0;
        for (const Param& p : _params) {
            const int idx = p.index - 1;
            if (idx >= 0 && idx < nPar) {
                hx += h[idx] * (correctionStateOnly ? p.value : (p.x0 + p.value));
            }
        }
        diag.hxBefore = hx;
        diag.residualBefore = z - hx;
        QVector<double> valueBefore(nPar, 0.0);
        for (const Param& p : _params) {
            const int idx = p.index - 1;
            if (idx >= 0 && idx < nPar) {
                valueBefore[idx] = p.value;
            }
        }
        bool firstVar = true;
        for (auto it = coeff.constBegin(); it != coeff.constEnd(); ++it) {
            const int idx = it.key();
            if (idx < 0 || idx >= nPar || idx >= _Q.size() || idx >= _Q[idx].size()) {
                continue;
            }
            const double varBefore = _Q[idx][idx];
            if (firstVar) {
                diag.minVarBefore = varBefore;
                diag.maxVarBefore = varBefore;
                firstVar = false;
            } else {
                diag.minVarBefore = std::min(diag.minVarBefore, varBefore);
                diag.maxVarBefore = std::max(diag.maxVarBefore, varBefore);
            }
            QString name = QString("idx%1").arg(idx);
            for (const Param& p : _params) {
                if (p.index - 1 == idx) {
                    name = paramText(p).trimmed();
                    break;
                }
            }
            diag.paramParts << QString("%1 coeff=%2 varBefore=%3")
                                   .arg(name)
                                   .arg(it.value(), 0, 'g', 12)
                                   .arg(varBefore, 0, 'g', 12);
        }
        const double v = z - hx;
        for (Param& p : _params) {
            const int idx = p.index - 1;
            if (idx >= 0 && idx < nPar) {
                p.value += qh[idx] * v / denom;
            }
        }

        Mat Qnew = _Q;
        for (int i = 0; i < nPar; ++i) {
            for (int j = 0; j < nPar; ++j) {
                Qnew[i][j] = _Q[i][j] - qh[i] * qh[j] / denom;
            }
        }
        _Q = Qnew;
        double hxAfter = 0.0;
        for (const Param& p : _params) {
            const int idx = p.index - 1;
            if (idx >= 0 && idx < nPar) {
                hxAfter += h[idx] * (correctionStateOnly ? p.value : (p.x0 + p.value));
                diag.maxStateDelta = std::max(diag.maxStateDelta, std::fabs(p.value - valueBefore[idx]));
            }
        }
        diag.hxAfter = hxAfter;
        diag.residualAfter = z - hxAfter;
        firstVar = true;
        for (auto it = coeff.constBegin(); it != coeff.constEnd(); ++it) {
            const int idx = it.key();
            if (idx < 0 || idx >= nPar || idx >= _Q.size() || idx >= _Q[idx].size()) {
                continue;
            }
            const double varAfter = _Q[idx][idx];
            if (firstVar) {
                diag.minVarAfter = varAfter;
                diag.maxVarAfter = varAfter;
                firstVar = false;
            } else {
                diag.minVarAfter = std::min(diag.minVarAfter, varAfter);
                diag.maxVarAfter = std::max(diag.maxVarAfter, varAfter);
            }
        }
        diag.applied = true;
        return diag;
    }

    struct BieConstraintApplyDiag {
        bool applied = false;
        double residualBefore = 0.0;
        double residualAfter = 0.0;
    };

    bool applyBieConstraintsBatch(const QVector<BieAlgorithm::Constraint>& constraints,
                                  QVector<BieConstraintApplyDiag>& diagnostics) {
        diagnostics.clear();
        const int nPar = _params.size();
        if (constraints.isEmpty() || nPar == 0) return false;

        Mat h = zeros(constraints.size(), nPar);
        Vec targets(constraints.size(), 0.0);
        Vec variances(constraints.size(), 0.0);
        diagnostics.resize(constraints.size());
        QSet<int> constrainedIndices;
        for (int row = 0; row < constraints.size(); ++row) {
            const BieAlgorithm::Constraint& constraint = constraints[row];
            const int idx = constraint.stateIndex;
            if (idx < 0 || idx >= nPar || constrainedIndices.contains(idx) ||
                !std::isfinite(constraint.value) || !std::isfinite(constraint.sigma) ||
                constraint.sigma <= 0.0) {
                return false;
            }
            constrainedIndices.insert(idx);
            h[row][idx] = 1.0;
            targets[row] = constraint.value;
            variances[row] = constraint.sigma * constraint.sigma;
            diagnostics[row].residualBefore = constraint.value - paramAt(idx);
        }

        Vec residuals;
        if (!applyKalmanUpdate(h, targets, variances, residuals) || residuals.size() != constraints.size()) {
            return false;
        }
        for (int row = 0; row < diagnostics.size(); ++row) {
            diagnostics[row].applied = true;
            diagnostics[row].residualAfter = residuals[row];
        }
        return true;
    }

    static bool melbourneWubbenaCycle(const PppSatData& sat, double& mw, double& phaseCycles, double& codeCycles) {
        mw = 0.0;
        phaseCycles = 0.0;
        codeCycles = 0.0;
        if (sat.C1 == 0.0 || sat.C2 == 0.0 || sat.L1 == 0.0 || sat.L2 == 0.0 ||
            sat.lambda1 <= 0.0 || sat.lambda2 <= 0.0 ||
            sat.f1 <= 0.0 || sat.f2 <= 0.0) {
            return false;
        }
        const double lambdaWl = GnssUtils::kC / (sat.f1 - sat.f2);
        const double narrowDenom = sat.lambda1 + sat.lambda2;
        if (!std::isfinite(lambdaWl) || std::fabs(lambdaWl) < 1e-9 ||
            std::fabs(narrowDenom) < 1e-9) {
            return false;
        }

        // GREAT smooths satdata.MW_cycle() after observation bias application.
        // Keep the same convention here: correct the raw measurements first, then form MW in cycles.
        phaseCycles = (sat.L1 + sat.phaseBias1 / sat.lambda1) -
                      (sat.L2 + sat.phaseBias2 / sat.lambda2);
        const double codeNlMeters = (sat.lambda2 * (sat.C1 + sat.codeBias1) +
                                     sat.lambda1 * (sat.C2 + sat.codeBias2)) / narrowDenom;
        codeCycles = codeNlMeters / lambdaWl;
        mw = phaseCycles - codeCycles;
        return std::isfinite(mw);
    }

    bool updateMwState(const PppSatData& sat, double& mwMean, double& mwSig, int& mwCount) {
        mwMean = 0.0;
        mwSig = 0.0;
        mwCount = 0;
        double mw = 0.0;
        double phaseCycles = 0.0;
        double codeCycles = 0.0;
        if (!melbourneWubbenaCycle(sat, mw, phaseCycles, codeCycles)) {
            return false;
        }
        MwState& st = _mwStates[sat.prn];

        const bool discChanged = (sat.disc1 >= 0 && st.lastDisc1 >= 0 && sat.disc1 != st.lastDisc1) ||
                                 (sat.disc2 >= 0 && st.lastDisc2 >= 0 && sat.disc2 != st.lastDisc2);
        if (sat.slipFlag || discChanged) {
            st = MwState();
        }
        if (sat.disc1 >= 0) st.lastDisc1 = sat.disc1;
        if (sat.disc2 >= 0) st.lastDisc2 = sat.disc2;

        st.count += 1;
        const double delta = mw - st.mean;
        st.mean += delta / static_cast<double>(st.count);
        st.m2 += delta * (mw - st.mean);

        mwMean = st.mean;
        mwCount = st.count;
        if (st.count > 1) {
            const double sampleVar = st.m2 / static_cast<double>(st.count - 1);
            mwSig = std::sqrt(sampleVar / static_cast<double>(st.count));
        } else {
            mwSig = std::numeric_limits<double>::infinity();
        }
        return true;
    }

    bool useBieAlgorithmAlgorithm() const {
        return _opt->arAlgorithm.compare(QStringLiteral("BIE"), Qt::CaseInsensitive) == 0;
    }

    static int acceptedSatelliteCount(const PppEpochData& epoData) {
        int count = 0;
        for (auto it = epoData.satData.constBegin(); it != epoData.satData.constEnd(); ++it) {
            if (!it.value().outlier) {
                ++count;
            }
        }
        return count;
    }

    void tryBieAlgorithm(PppEpochData& epoData) {
        QStringList arLines;
        ++_arEpochCounter;
        arLines << QString("BIE EPOCH begin algorithm=BIE gps=%1 week=%2 sow=%3 params=%4 minEle=%5 minEpochs=%6 minSat=%7 maxFrac=%8 maxSig=%9 constraintSigma=%10 persistence=%11")
                       .arg(GnssUtils::gpsCalendarDateTime(epoData.tt.gpsWeek, epoData.tt.gpsSow).toString(Qt::ISODateWithMs))
                       .arg(epoData.tt.gpsWeek)
                       .arg(epoData.tt.gpsSow, 0, 'f', 3)
                       .arg(_params.size())
                       .arg(_opt->arMinEleDeg, 0, 'f', 2)
                       .arg(_opt->arBieMinEpochs)
                       .arg(_opt->arBieMinSat)
                       .arg(_opt->arBieMaxFrac, 0, 'g', 8)
                       .arg(_opt->arBieMaxSig, 0, 'g', 8)
                       .arg(_opt->arBieConstraintSigma, 0, 'g', 8)
                       .arg(_opt->arBiePersistConstraints ? "on" : "off");

        QVector<BieAlgorithm::Ambiguity> ambiguities;
        QVector<int> stateIndices;
        for (const Param& p : _params) {
            if (p.type != AMB_L1 && p.type != AMB_L2) {
                continue;
            }
            const auto satIt = epoData.satData.constFind(p.prn);
            if (satIt == epoData.satData.constEnd() || satIt->outlier || p.index <= 0) {
                continue;
            }
            const PppSatData& sat = satIt.value();
            if (!_opt->arUseSystem(sat.system())) {
                continue;
            }
            bool biasUsable = false;
            QString group;
            if (p.type == AMB_L1) {
                biasUsable = sat.hasBias1 && sat.sii1 != 0;
                group = QStringLiteral("L1");
            } else if (p.type == AMB_L2) {
                biasUsable = sat.hasBias2 && sat.sii2 != 0;
                group = QStringLiteral("L2");
            }
            if (!biasUsable) {
                arLines << QString("BIE SAT=%1 group=%2 status=skip reason=missing_integer_phase_bias")
                               .arg(p.prn, group);
                continue;
            }
            const int stateIndex = p.index - 1;
            if (stateIndex < 0 || stateIndex >= _Q.size()) {
                continue;
            }
            BieAlgorithm::Ambiguity ambiguity;
            ambiguity.prn = p.prn;
            ambiguity.system = sat.system();
            ambiguity.group = group;
            ambiguity.stateIndex = stateIndex;
            ambiguity.value = p.value;
            ambiguity.elevationDeg = p.ambEleSat * kRhoDeg;
            ambiguity.numEpochs = static_cast<int>(p.ambNumEpo);
            ambiguities.push_back(ambiguity);
            stateIndices.push_back(stateIndex);
            if (ambiguity.numEpochs >= _opt->arBieMinEpochs &&
                ambiguity.elevationDeg >= _opt->arMinEleDeg) {
                _arEligiblePrns.insert(ambiguity.prn);
            }
        }

        BieAlgorithm::Mat covariance(ambiguities.size(), BieAlgorithm::Vec(ambiguities.size(), 0.0));
        for (int i = 0; i < ambiguities.size(); ++i) {
            for (int j = 0; j < ambiguities.size(); ++j) {
                covariance[i][j] = _Q[stateIndices[i]][stateIndices[j]];
            }
        }
        BieAlgorithm::Options options;
        options.minElevationDeg = _opt->arMinEleDeg;
        options.minNumEpochs = _opt->arBieMinEpochs;
        options.minNumSat = _opt->arBieMinSat;
        options.maxFrac = _opt->arBieMaxFrac;
        options.maxSigma = _opt->arBieMaxSig;
        options.constraintSigma = _opt->arBieConstraintSigma;
        options.candidateCount = 100;
        options.preferredReferenceByGroup = _lastBieReferenceByGroup;
        const BieAlgorithm::Result result = BieAlgorithm::run(ambiguities, covariance, options);
        _arCandidateCount = result.candidateCount;
        _arEffectiveCandidates = result.effectiveCandidates;
        for (const BieAlgorithm::FixedGroup& fixedGroup : result.fixedGroups) {
            PppArGroupResult group;
            group.system = fixedGroup.system;
            group.group = fixedGroup.group;
            group.referenceSatellite = fixedGroup.referencePrn;
            group.fixedCount = qMin(fixedGroup.ambiguityCount, fixedGroup.fixedCount + 1);
            group.eligibleCount = fixedGroup.ambiguityCount;
            group.maxFractionCycle = fixedGroup.maxFraction;
            group.maxSigmaCycle = fixedGroup.maxSigma;
            _arGroups.push_back(group);
        }
        arLines += result.logLines;
        for (const BieAlgorithm::FixedGroup& fixedGroup : result.fixedGroups) {
            const QString groupKey = QString("%1|%2")
                                         .arg(QString(fixedGroup.system), fixedGroup.group);
            const QString previousReference = _lastBieReferenceByGroup.value(groupKey);
            if (!previousReference.isEmpty() && previousReference != fixedGroup.referencePrn) {
                arLines << QString("BIE REFERENCE group=%1 previous=%2 current=%3 changed=yes relation_state=preserved")
                               .arg(groupKey, previousReference, fixedGroup.referencePrn);
            }
            _lastBieReferenceByGroup[groupKey] = fixedGroup.referencePrn;
        }

        int applied = 0;
        QVector<BieConstraintApplyDiag> constraintDiagnostics;
        bool batchApplied = false;
        QString constraintState = QStringLiteral("none");
        BiePosteriorApplyDiag posteriorDiag;
        QString epochLocalFixedResult;
        QSet<int> persistedRows;
        const auto applyEpochLocal = [&]() {
            _bieFloatParams = _params;
            _bieFloatQ = _Q;
            _restoreBieState = true;
            batchApplied = applyBieConstraintsBatch(result.constraints, constraintDiagnostics);
            if (batchApplied) {
                epochLocalFixedResult = appendBieFixedResultLog();
            }
            restoreBieFloatState();
        };

        if (!result.constraints.isEmpty()) {
            if (!_opt->arBiePersistConstraints) {
                applyEpochLocal();
                constraintState = batchApplied ? QStringLiteral("epoch_local") : QStringLiteral("failed");
            } else if (result.posteriorStateIndices.size() != result.constraints.size()) {
                applyEpochLocal();
                constraintState = batchApplied ? QStringLiteral("epoch_local_invalid_posterior")
                                               : QStringLiteral("failed");
                arLines << QStringLiteral("BIE PERSISTENCE status=skip reason=posterior_contract_missing");
            } else {
                const QVector<int> stableRows = advanceBiePersistence(result, arLines);
                const BieAlgorithm::Result persistentResult = selectBiePosterior(result, stableRows);
                if (stableRows.isEmpty()) {
                    applyEpochLocal();
                    constraintState = batchApplied ? QStringLiteral("epoch_local_pending")
                                                   : QStringLiteral("failed");
                    arLines << QStringLiteral("BIE PERSISTENCE status=pending reason=no_group_with_stable_arc_set");
                } else if (persistentResult.constraints.size() != stableRows.size()) {
                    applyEpochLocal();
                    constraintState = batchApplied ? QStringLiteral("epoch_local_invalid_posterior")
                                                   : QStringLiteral("failed");
                    arLines << QStringLiteral("BIE PERSISTENCE status=skip reason=stable_posterior_contract_missing");
                } else {
                    QVector<Param> posteriorParams;
                    Mat posteriorCovariance;
                    const bool posteriorCalculated = calculateBiePosteriorUpdate(
                        persistentResult, posteriorParams, posteriorCovariance, posteriorDiag);
                    const double maxVertical = qMax(0.0, _opt->arBiePersistMaxVerticalM);
                    const double maxZtd = qMax(0.0, _opt->arBiePersistMaxZtdM);
                    const bool transitionAccepted = posteriorCalculated &&
                        (maxVertical <= 0.0 || std::fabs(posteriorDiag.verticalCorrectionM) <= maxVertical) &&
                        (maxZtd <= 0.0 || std::fabs(posteriorDiag.ztdCorrectionM) <= maxZtd);
                    if (transitionAccepted) {
                        _params = posteriorParams;
                        _Q = posteriorCovariance;
                        rebuildParamIndexCache();
                        batchApplied = true;
                        applied = persistentResult.constraints.size();
                        constraintState = QStringLiteral("committed_posterior");
                        for (int row : stableRows) persistedRows.insert(row);
                        arLines << QString("BIE PERSISTENCE status=committed method=posterior_moment_match arcs=%1 dimension=%2 dU=%3 dZTD=%4 maxState=%5 covarianceLoading=%6")
                                       .arg(stableRows.size())
                                       .arg(posteriorDiag.dimension)
                                       .arg(posteriorDiag.verticalCorrectionM, 0, 'g', 12)
                                       .arg(posteriorDiag.ztdCorrectionM, 0, 'g', 12)
                                       .arg(posteriorDiag.maxStateCorrection, 0, 'g', 12)
                                       .arg(posteriorDiag.covarianceRegularization, 0, 'g', 12);
                    } else {
                        const QString reason = posteriorCalculated ? QStringLiteral("transition_gate")
                                                                   : posteriorDiag.reason;
                        arLines << QString("BIE PERSISTENCE status=reject reason=%1 arcs=%2 dU=%3 limitU=%4 dZTD=%5 limitZTD=%6")
                                       .arg(reason)
                                       .arg(stableRows.size())
                                       .arg(posteriorDiag.verticalCorrectionM, 0, 'g', 12)
                                       .arg(maxVertical, 0, 'g', 12)
                                       .arg(posteriorDiag.ztdCorrectionM, 0, 'g', 12)
                                       .arg(maxZtd, 0, 'g', 12);
                        for (int row : stableRows) {
                            _biePersistent.remove(biePersistenceKey(result.constraints[row]));
                        }
                        applyEpochLocal();
                        constraintState = batchApplied ? QStringLiteral("epoch_local_rejected")
                                                       : QStringLiteral("failed");
                    }
                }
            }
        } else if (_opt->arBiePersistConstraints) {
            // Entries are deliberately retained only as metadata.  Their
            // lastEpoch forces reconfirmation if the satellite later returns.
            arLines << QStringLiteral("BIE PERSISTENCE status=hold reason=no_accepted_constraints");
        }
        arLines << QString("BIE constraintBatch status=%1 constraints=%2 state=%3")
                       .arg(batchApplied ? "applied" : (result.constraints.isEmpty() ? "skip" : "failed"))
                       .arg(result.constraints.size())
                       .arg(constraintState);
        for (int i = 0; i < result.constraints.size(); ++i) {
            const BieAlgorithm::Constraint& constraint = result.constraints[i];
            const BieConstraintApplyDiag diag = constraintDiagnostics.value(i);
            const bool posteriorApplied = persistedRows.contains(i);
            const bool constraintApplied = persistedRows.isEmpty()
                ? batchApplied : persistedRows.contains(i);
            if (constraintApplied) {
                _arFixedPrns.insert(constraint.prn);
            }
            const QString residualBefore = posteriorApplied ? QStringLiteral("-")
                : QString::number(diag.residualBefore, 'g', 12);
            const QString residualAfter = posteriorApplied ? QStringLiteral("-")
                : QString::number(diag.residualAfter, 'g', 12);
            arLines << QString("BIE CONSTRAINT sys=%1 group=%2 sat=%3 ref=%4 value=%5 sigma=%6 applied=%7 update=%8 residualBefore=%9 residualAfter=%10")
                           .arg(QString(constraint.system), constraint.group,
                                constraint.prn, constraint.referencePrn)
                           .arg(constraint.value, 0, 'g', 12)
                           .arg(constraint.sigma, 0, 'g', 12)
                           .arg(constraintApplied ? "yes" : "no")
                           .arg(posteriorApplied ? QStringLiteral("posterior") :
                                (constraintApplied ? QStringLiteral("epoch_local") : QStringLiteral("none")))
                           .arg(residualBefore, residualAfter);
        }
        _arConstraintsApplied = persistedRows.isEmpty()
            ? (batchApplied ? result.constraints.size() : 0) : persistedRows.size();
        applied = _arConstraintsApplied;
        if (!epochLocalFixedResult.isEmpty()) {
            arLines << epochLocalFixedResult;
        } else if (applied > 0) {
            arLines << appendBieFixedResultLog();
        }
        arLines << QString("BIE EPOCH end status=%1 constraints=%2 applied=%3 fixedSats=%4/%5 summary=%6")
                       .arg(applied > 0 ? "fixed" : "float")
                       .arg(result.constraints.size())
                       .arg(applied)
                       .arg(_arFixedPrns.size())
                       .arg(acceptedSatelliteCount(epoData))
                       .arg(result.summary.isEmpty() ? QStringLiteral("-") : result.summary);
        if (applied > 0) {
            _arSummary = result.summary;
        }
        _arEpochLogLines += arLines;
    }

    void tryAmbiguityFix(PppEpochData& epoData) {
        if (!_opt->enablePppAr) return;
        if (useBieAlgorithmAlgorithm()) {
            tryBieAlgorithm(epoData);
            return;
        }

        QStringList arLines;
        ++_arEpochCounter;
        QStringList previousFixedSystems;
        for (auto it = _lastArFixedSignature.constBegin(); it != _lastArFixedSignature.constEnd(); ++it) {
            previousFixedSystems << QString(it.key());
        }
        arLines << QString("AR EPOCH begin gps=%1 week=%2 sow=%3 sats=%4 params=%5 modeG=%6 modeR=%7 modeE=%8 modeC=%9 minEle=%10 minMw=%11 minPairs=%12 wlDev=%13 wlSig=%14 nlRatio=%15 nlBoot=%16 nlMaxDev=%17 sigma=%18 glo=%19")
                       .arg(GnssUtils::gpsCalendarDateTime(epoData.tt.gpsWeek, epoData.tt.gpsSow).toString(Qt::ISODateWithMs))
                       .arg(epoData.tt.gpsWeek)
                       .arg(epoData.tt.gpsSow, 0, 'f', 3)
                       .arg(epoData.satData.size())
                       .arg(_params.size())
                       .arg(_opt->lcGps, _opt->lcGlo, _opt->lcGal, _opt->lcBds)
                       .arg(_opt->arMinEleDeg, 0, 'f', 2)
                       .arg(_opt->arMinMwSamples)
                       .arg(_opt->arMinFixPairs)
                       .arg(_opt->arWlMaxDev, 0, 'f', 4)
                       .arg(_opt->arWlMaxSig, 0, 'f', 4)
                       .arg(_opt->arNlRatio, 0, 'f', 3)
                       .arg(_opt->arNlBoot, 0, 'f', 4)
                       .arg(_opt->arNlMaxDev, 0, 'f', 4)
                       .arg(_opt->arConstraintSigma, 0, 'f', 5)
                       .arg(_opt->arUseGlonass ? "on" : "off");
        arLines << QString("AR EPOCH histAid=%1 histFixMin=%2 histLockMin=%3 warmLockMin=%4 relax=%5")
                       .arg(_opt->arUseHistoryAid ? "on" : "off")
                       .arg(_opt->arHistMinFixEpochs)
                       .arg(_opt->arHistMinLockEpochs)
                       .arg(_opt->arPartialWarmupLockEpochs)
                       .arg(_opt->arHistRelaxFactor, 0, 'f', 3);
        arLines << QString("AR EPOCH index=%1 previousFixedSystems=%2")
                       .arg(_arEpochCounter)
                       .arg(previousFixedSystems.isEmpty()
                                ? QStringLiteral("-")
                                : previousFixedSystems.join(QLatin1Char(',')));
        int phaseStateSeenSats = 0;
        int phaseDualBiasMatchedSats = 0;
        for (auto it = epoData.satData.constBegin(); it != epoData.satData.constEnd(); ++it) {
            const PppSatData& sat = it.value();
            if (sat.phaseBiasDiag.contains(QStringLiteral("phaseState=Y"))) {
                ++phaseStateSeenSats;
            }
            if (sat.hasBias1 && sat.hasBias2) {
                ++phaseDualBiasMatchedSats;
            }
        }
        arLines << QString("AR EPOCH ssrPhase epochStateSats=%1 dualBiasMatchedSats=%2")
                       .arg(phaseStateSeenSats)
                       .arg(phaseDualBiasMatchedSats);
        QMap<QString, int> skipCounts;
        auto skipSat = [&](const PppSatData& sat, const QString& reason, const QString& detail = QString()) {
            ++skipCounts[reason];
            arLines << QString("AR SAT=%1 status=pre_skip reason=%2%3")
                           .arg(sat.prn,
                                reason,
                                detail.isEmpty() ? QString() : QString(" %1").arg(detail));
        };

        QVector<AmbFix::Sat> sats;
        sats.reserve(epoData.satData.size());

        for (auto it = epoData.satData.constBegin(); it != epoData.satData.constEnd(); ++it) {
            const PppSatData& sat = it.value();
            if (sat.L1 == 0.0 || sat.L2 == 0.0 || sat.C1 == 0.0 || sat.C2 == 0.0) {
                skipSat(sat, QStringLiteral("missing_dual_freq_obs"),
                        QString("C1=%1 C2=%2 L1=%3 L2=%4")
                            .arg(sat.C1 != 0.0 ? "Y" : "N")
                            .arg(sat.C2 != 0.0 ? "Y" : "N")
                            .arg(sat.L1 != 0.0 ? "Y" : "N")
                            .arg(sat.L2 != 0.0 ? "Y" : "N"));
                continue;
            }
            if (!sat.hasBias1 || !sat.hasBias2) {
                skipSat(sat, QStringLiteral("missing_phase_bias"),
                        QString("bias1=%1 bias2=%2 bands=%3/%4 %5")
                            .arg(sat.hasBias1 ? "Y" : "N")
                            .arg(sat.hasBias2 ? "Y" : "N")
                            .arg(QString(sat.bandA))
                            .arg(QString(sat.bandB))
                            .arg(sat.phaseBiasDiag.isEmpty() ? QStringLiteral("phaseDiag=-") : sat.phaseBiasDiag));
                continue;
            }
            if (sat.sii1 == 0 || sat.sii2 == 0) {
                skipSat(sat, QStringLiteral("non_integer_phase_bias"),
                        QString("sii1=%1 sii2=%2").arg(sat.sii1).arg(sat.sii2));
                continue;
            }
            if (sat.system() == QLatin1Char('R') && !_opt->arUseGlonass) {
                skipSat(sat, QStringLiteral("glonass_disabled"));
                continue;
            }
            if (sat.eleSat * kRhoDeg < _opt->arMinEleDeg) {
                skipSat(sat, QStringLiteral("low_elevation"),
                        QString("ele=%1 min=%2").arg(sat.eleSat * kRhoDeg, 0, 'f', 2).arg(_opt->arMinEleDeg, 0, 'f', 2));
                continue;
            }
            if (sat.disc1 >= 0) {
                if (_lastDisc1.contains(sat.prn) && _lastDisc1[sat.prn] != sat.disc1) {
                    const int oldDisc1 = _lastDisc1.value(sat.prn);
                    _lastDisc1[sat.prn] = sat.disc1;
                    resetArSatHistory(sat.prn);
                    skipSat(sat, QStringLiteral("phase_bias_disc1_changed"),
                            QString("old=%1 new=%2").arg(oldDisc1).arg(sat.disc1));
                    continue;
                }
                _lastDisc1[sat.prn] = sat.disc1;
            }
            if (sat.disc2 >= 0) {
                if (_lastDisc2.contains(sat.prn) && _lastDisc2[sat.prn] != sat.disc2) {
                    const int oldDisc2 = _lastDisc2.value(sat.prn);
                    _lastDisc2[sat.prn] = sat.disc2;
                    resetArSatHistory(sat.prn);
                    skipSat(sat, QStringLiteral("phase_bias_disc2_changed"),
                            QString("old=%1 new=%2").arg(oldDisc2).arg(sat.disc2));
                    continue;
                }
                _lastDisc2[sat.prn] = sat.disc2;
            }

            if (sat.lambda1 <= 0.0 || sat.lambda2 <= 0.0) {
                skipSat(sat, QStringLiteral("invalid_wavelength"),
                        QString("lam1=%1 lam2=%2").arg(sat.lambda1, 0, 'g', 12).arg(sat.lambda2, 0, 'g', 12));
                continue;
            }

            int ionIdx = -1;
            int amb1Idx = -1;
            int amb2Idx = -1;
            if (!findObsStateIndices(sat.prn, ionIdx, amb1Idx, amb2Idx)) {
                skipSat(sat, QStringLiteral("missing_iono_or_amb_state"),
                        QString("ion=%1 amb1=%2 amb2=%3").arg(ionIdx).arg(amb1Idx).arg(amb2Idx));
                continue;
            }
            if (amb1Idx < 0 || amb2Idx < 0) {
                skipSat(sat, QStringLiteral("missing_l1_l2_amb_state"),
                        QString("amb1=%1 amb2=%2").arg(amb1Idx).arg(amb2Idx));
                continue;
            }

            const double f1sq = sat.f1 * sat.f1;
            const double f2sq = sat.f2 * sat.f2;
            const double denom = f1sq - f2sq;
            if (std::fabs(denom) < 1e-6) {
                skipSat(sat, QStringLiteral("invalid_frequency_pair"),
                        QString("f1=%1 f2=%2").arg(sat.f1, 0, 'g', 12).arg(sat.f2, 0, 'g', 12));
                continue;
            }
            QString arcBreakReason;
            bool arcReset = false;
            ArSatHistory& hist = updateArSatHistory(sat, &arcBreakReason, &arcReset);
            const double a1 = f1sq / denom;
            const double a2 = -f2sq / denom;
            const double amb1Cycles = paramEstimateAt(amb1Idx);
            const double amb2Cycles = paramEstimateAt(amb2Idx);

            AmbFix::Sat entry;
            entry.prn = sat.prn;
            entry.sys = sat.system();
            entry.eleRad = sat.eleSat;
            entry.C1 = sat.C1;
            entry.C2 = sat.C2;
            entry.L1 = sat.L1;
            entry.L2 = sat.L2;
            entry.lambda1 = sat.lambda1;
            entry.lambda2 = sat.lambda2;
            entry.f1 = sat.f1;
            entry.f2 = sat.f2;
            entry.sii1 = sat.sii1;
            entry.sii2 = sat.sii2;
            entry.wli1 = sat.wli1;
            entry.wli2 = sat.wli2;
            entry.hasBias1 = sat.hasBias1;
            entry.hasBias2 = sat.hasBias2;
            entry.codeBias1 = sat.codeBias1;
            entry.codeBias2 = sat.codeBias2;
            entry.phaseBias1 = sat.phaseBias1;
            entry.phaseBias2 = sat.phaseBias2;
            entry.ambIndex1 = amb1Idx;
            entry.ambIndex2 = amb2Idx;
            entry.ambCoeff1 = a1 * sat.lambda1;
            entry.ambCoeff2 = a2 * sat.lambda2;
            entry.ambMeters = entry.ambCoeff1 * amb1Cycles + entry.ambCoeff2 * amb2Cycles;
            entry.lockEpochs = hist.lockEpochs;
            entry.wlFixEpochs = hist.wlFixEpochs;
            entry.nlFixEpochs = hist.nlFixEpochs;
            entry.newArc = (hist.arcEpochs <= 1);
            double mwMean = 0.0;
            double mwSig = 0.0;
            int mwCount = 0;
            if (updateMwState(sat, mwMean, mwSig, mwCount)) {
                entry.hasMwStat = true;
                entry.mw = mwMean;
                entry.mwSig = mwSig;
                entry.mwCount = mwCount;
            }
            if (arcReset) {
                arLines << QString("AR HIST SAT=%1 event=arc_reset reason=%2 epoch=%3")
                               .arg(entry.prn)
                               .arg(arcBreakReason.isEmpty() ? QStringLiteral("-") : arcBreakReason)
                               .arg(_arEpochCounter);
            }
            arLines << QString("AR SAT=%1 status=pre_accept sys=%2 ele=%3 C1=%4 C2=%5 L1=%6 L2=%7 lam1=%8 lam2=%9 ambIdx1=%10 ambIdx2=%11 ambN1=%12 ambN2=%13 ambIFm=%14 sii=%15/%16 wli=%17/%18 disc=%19/%20 mwN=%21 mwMean=%22 mwSig=%23 histLock=%24 histWlFix=%25 histNlFix=%26 newArc=%27 bias=%28/%29")
                           .arg(entry.prn)
                           .arg(QString(entry.sys))
                           .arg(entry.eleRad * kRhoDeg, 0, 'f', 2)
                           .arg(entry.C1, 0, 'f', 4)
                           .arg(entry.C2, 0, 'f', 4)
                           .arg(entry.L1, 0, 'f', 4)
                           .arg(entry.L2, 0, 'f', 4)
                           .arg(entry.lambda1, 0, 'g', 12)
                           .arg(entry.lambda2, 0, 'g', 12)
                           .arg(entry.ambIndex1)
                           .arg(entry.ambIndex2)
                           .arg(amb1Cycles, 0, 'f', 4)
                           .arg(amb2Cycles, 0, 'f', 4)
                           .arg(entry.ambMeters, 0, 'f', 4)
                           .arg(entry.sii1)
                           .arg(entry.sii2)
                           .arg(entry.wli1)
                           .arg(entry.wli2)
                           .arg(sat.disc1)
                           .arg(sat.disc2)
                           .arg(entry.hasMwStat ? entry.mwCount : 0)
                           .arg(entry.hasMwStat ? entry.mw : 0.0, 0, 'f', 4)
                           .arg(entry.hasMwStat ? entry.mwSig : -1.0, 0, 'f', 4)
                           .arg(entry.lockEpochs)
                           .arg(entry.wlFixEpochs)
                           .arg(entry.nlFixEpochs)
                           .arg(entry.newArc ? "yes" : "no")
                           .arg(sat.phaseBias1, 0, 'f', 4)
                           .arg(sat.phaseBias2, 0, 'f', 4);
            sats.push_back(entry);
            _arEligiblePrns.insert(entry.prn);
        }

        QStringList skipParts;
        for (auto it = skipCounts.constBegin(); it != skipCounts.constEnd(); ++it) {
            skipParts << QString("%1=%2").arg(it.key()).arg(it.value());
        }
        arLines << QString("AR EPOCH prefilter accepted=%1 skipped=%2 skipCounts=%3")
                       .arg(sats.size())
                       .arg(epoData.satData.size() - sats.size())
                       .arg(skipParts.isEmpty() ? QStringLiteral("-") : skipParts.join(QLatin1Char(',')));

        AmbFix::Options opt;
        opt.sigmaC1 = _opt->sigmaC1;
        opt.sigmaL1 = _opt->sigmaL1;
        opt.useGlonass = _opt->arUseGlonass;
        opt.minEleDeg = _opt->arMinEleDeg;
        opt.minFixPairs = _opt->arMinFixPairs;
        opt.minMwSamples = _opt->arMinMwSamples;
        opt.wlMaxDev = _opt->arWlMaxDev;
        opt.wlMaxSig = _opt->arWlMaxSig;
        opt.nlRatio = _opt->arNlRatio;
        opt.nlBoot = _opt->arNlBoot;
        opt.nlMaxDev = _opt->arNlMaxDev;
        opt.constraintSigma = _opt->arConstraintSigma;
        opt.useHistoryAid = _opt->arUseHistoryAid;
        opt.histMinFixEpochs = _opt->arHistMinFixEpochs;
        opt.histMinLockEpochs = _opt->arHistMinLockEpochs;
        opt.partialWarmupLockEpochs = _opt->arPartialWarmupLockEpochs;
        opt.histRelaxFactor = _opt->arHistRelaxFactor;
        opt.preferredReferenceBySystem = _lastArReference;

        const AmbFix::Result res = AmbFix::run(epoData.tt.utc, sats, _Q, opt);
        arLines += res.logLines;
        for (const AmbFix::FixedSet& fixedSet : res.fixedSets) {
            const QString previousReference = _lastArReference.value(fixedSet.sys);
            const bool referenceChanged = !previousReference.isEmpty() &&
                                          previousReference != fixedSet.refPrn;
            if (referenceChanged) {
                arLines << QString("AR SYS=%1 REFERENCE previous=%2 current=%3 changed=yes relation_state=preserved")
                               .arg(QString(fixedSet.sys), previousReference, fixedSet.refPrn);
            }
            _lastArReference[fixedSet.sys] = fixedSet.refPrn;
            const QString prevSig = _lastArFixedSignature.value(fixedSet.sys);
            QString fixClass = QStringLiteral("fresh_fix");
            if (!prevSig.isEmpty()) {
                fixClass = (prevSig == fixedSet.signature) ? QStringLiteral("hold") : QStringLiteral("reacquire");
            }
            const int prevEpoch = _lastArFixedEpoch.value(fixedSet.sys, -1);
            arLines << QString("AR SYS=%1 FIX_STATE class=%2 type=%3 epoch=%4 prevEpoch=%5 ref=%6 WL=%7 NL=%8 ratio=%9 ratioRaw=%10 boot=%11 maxNlDev=%12 qddMin=%13 qddMax=%14 qddAvg=%15 worstDD=%16 histWlQualified=%17 histNlQualified=%18 warmupDropped=%19 fixedDD=%20 droppedDD=%21 signature=%22 prevSignature=%23 referenceChanged=%24")
                           .arg(QString(fixedSet.sys))
                           .arg(fixClass)
                           .arg(fixedSet.partial ? QStringLiteral("partial") : QStringLiteral("full"))
                           .arg(_arEpochCounter)
                           .arg(prevEpoch)
                           .arg(fixedSet.refPrn)
                           .arg(fixedSet.wlCount)
                           .arg(fixedSet.nlCount)
                           .arg(fixedSet.ratio, 0, 'f', 4)
                           .arg(fixedSet.ratioRaw, 0, 'g', 12)
                           .arg(fixedSet.boot, 0, 'f', 4)
                           .arg(fixedSet.maxNlDev, 0, 'f', 4)
                           .arg(fixedSet.qddMin, 0, 'g', 12)
                           .arg(fixedSet.qddMax, 0, 'g', 12)
                           .arg(fixedSet.qddAvg, 0, 'g', 12)
                           .arg(fixedSet.worstDd)
                           .arg(fixedSet.histQualifiedWlDdCount)
                           .arg(fixedSet.histQualifiedNlDdCount)
                           .arg(fixedSet.warmupDroppedCount)
                           .arg(fixedSet.fixedDds.isEmpty() ? QStringLiteral("-") : fixedSet.fixedDds.join(QLatin1Char(',')))
                           .arg(fixedSet.droppedDds.isEmpty() ? QStringLiteral("-") : fixedSet.droppedDds.join(QLatin1Char(',')))
                           .arg(fixedSet.signature)
                           .arg(prevSig.isEmpty() ? QStringLiteral("-") : prevSig)
                           .arg(referenceChanged ? "yes" : "no");
        }
        int constraintIndex = 0;
        ConstraintApplySummary applySummary;
        QMap<QChar, ConstraintApplySummary> applySummaryBySys;
        for (const AmbFix::Constraint& cons : res.constraints) {
            QStringList coeffParts;
            for (auto it = cons.coeff.constBegin(); it != cons.coeff.constEnd(); ++it) {
                coeffParts << QString("%1:%2").arg(it.key()).arg(it.value(), 0, 'g', 12);
            }
            arLines << QString("AR CONSTRAINT apply index=%1 sys=%2 sat=%3 ref=%4 z=%5 NLfloat=%6 NLfix=%7 WLfix=%8 sigma=%9 coeff=%10")
                           .arg(constraintIndex++)
                           .arg(QString(cons.sys))
                           .arg(cons.satPrn)
                           .arg(cons.refPrn)
                           .arg(cons.z, 0, 'f', 6)
                           .arg(cons.nlFloat, 0, 'f', 6)
                           .arg(cons.nlFixed, 0, 'f', 6)
                           .arg(cons.wlFixed)
                           .arg(opt.constraintSigma, 0, 'f', 6)
                           .arg(coeffParts.join(QLatin1Char(',')));
            const ConstraintApplyDiag diag = applyConstraint(cons.coeff, cons.z, opt.constraintSigma);
            applySummary.add(diag);
            applySummaryBySys[cons.sys].add(diag);
            if (diag.applied) {
                _arFixedPrns.insert(cons.refPrn);
                _arFixedPrns.insert(cons.satPrn);
            }
            arLines << QString("AR CONSTRAINT result applied=%1 hxBefore=%2 residualBefore=%3 hxAfter=%4 residualAfter=%5 denom=%6 maxStateDelta=%7 varBefore=[%8,%9] varAfter=[%10,%11] params=%12")
                           .arg(diag.applied ? "yes" : "no")
                           .arg(diag.hxBefore, 0, 'f', 6)
                           .arg(diag.residualBefore, 0, 'f', 6)
                           .arg(diag.hxAfter, 0, 'f', 6)
                           .arg(diag.residualAfter, 0, 'f', 6)
                           .arg(diag.denom, 0, 'g', 12)
                           .arg(diag.maxStateDelta, 0, 'g', 12)
                           .arg(diag.minVarBefore, 0, 'g', 12)
                           .arg(diag.maxVarBefore, 0, 'g', 12)
                           .arg(diag.minVarAfter, 0, 'g', 12)
                           .arg(diag.maxVarAfter, 0, 'g', 12)
                           .arg(diag.paramParts.isEmpty() ? QStringLiteral("-") : diag.paramParts.join(QStringLiteral(" | ")));
        }
        if (applySummary.count > 0) {
            arLines << QString("AR CONSTRAINT summary count=%1 applied=%2 maxAbsResidualBefore=%3 maxAbsResidualAfter=%4 maxStateDelta=%5 denomRange=[%6,%7] varBefore=[%8,%9] varAfter=[%10,%11]")
                           .arg(applySummary.count)
                           .arg(applySummary.applied)
                           .arg(applySummary.maxAbsResidualBefore, 0, 'g', 12)
                           .arg(applySummary.maxAbsResidualAfter, 0, 'g', 12)
                           .arg(applySummary.maxStateDelta, 0, 'g', 12)
                           .arg(applySummary.minDenom, 0, 'g', 12)
                           .arg(applySummary.maxDenom, 0, 'g', 12)
                           .arg(applySummary.minVarBefore, 0, 'g', 12)
                           .arg(applySummary.maxVarBefore, 0, 'g', 12)
                           .arg(applySummary.minVarAfter, 0, 'g', 12)
                           .arg(applySummary.maxVarAfter, 0, 'g', 12);
            for (const AmbFix::FixedSet& fixedSet : res.fixedSets) {
                const ConstraintApplySummary sysSummary = applySummaryBySys.value(fixedSet.sys, applySummary);
                const bool alreadyInteger = sysSummary.maxAbsResidualBefore < 1e-4 &&
                                            sysSummary.maxStateDelta < 1e-6;
                arLines << QString("AR SYS=%1 FIX_EVIDENCE note=%2 maxAbsResidualBefore=%3 maxStateDelta=%4 qddMax=%5 ratioRaw=%6")
                               .arg(QString(fixedSet.sys))
                               .arg(alreadyInteger ? QStringLiteral("already_integer_before_constraint")
                                                   : QStringLiteral("constraint_moved_state"))
                               .arg(sysSummary.maxAbsResidualBefore, 0, 'g', 12)
                               .arg(sysSummary.maxStateDelta, 0, 'g', 12)
                               .arg(fixedSet.qddMax, 0, 'g', 12)
                               .arg(fixedSet.ratioRaw, 0, 'g', 12);
            }
        }
        QVector<AmbFix::FixedSet> appliedFixedSets;
        for (const AmbFix::FixedSet& fixedSet : res.fixedSets) {
            const ConstraintApplySummary sysSummary = applySummaryBySys.value(fixedSet.sys);
            if (sysSummary.count > 0 && sysSummary.count == sysSummary.applied) {
                appliedFixedSets.push_back(fixedSet);
                if (fixedSet.ratio > 0.0 && (_arRatio <= 0.0 || fixedSet.ratio < _arRatio)) {
                    _arRatio = fixedSet.ratio;
                }
                if (fixedSet.boot > 0.0 && (_arBootstrap <= 0.0 || fixedSet.boot < _arBootstrap)) {
                    _arBootstrap = fixedSet.boot;
                }
            }
        }
        _arConstraintsApplied = applySummary.applied;
        for (const AmbFix::FixedSet& fixedSet : appliedFixedSets) {
            int eligibleCount = 0;
            for (const AmbFix::Sat& sat : sats) {
                if (sat.sys == fixedSet.sys) {
                    ++eligibleCount;
                }
            }
            PppArGroupResult group;
            group.system = fixedSet.sys;
            group.group = QStringLiteral("NL");
            group.referenceSatellite = fixedSet.refPrn;
            group.fixedCount = fixedSet.fixedPrns.size();
            group.eligibleCount = eligibleCount;
            group.maxFractionCycle = fixedSet.maxNlDev;
            group.maxSigmaCycle = fixedSet.qddMax >= 0.0 ? std::sqrt(fixedSet.qddMax) : 0.0;
            _arGroups.push_back(group);
        }
        updateArFixHistory(sats, appliedFixedSets, arLines);
        arLines << QString("AR EPOCH end status=%1 constraints=%2 fixedSats=%3/%4 summary=%5")
                       .arg(res.constraints.isEmpty() ? "float" : "fixed")
                       .arg(res.constraints.size())
                       .arg(_arFixedPrns.size())
                       .arg(acceptedSatelliteCount(epoData))
                       .arg(res.summary.isEmpty() ? QStringLiteral("-") : res.summary);

        if (res.fixedSets.isEmpty()) {
            if (!_arSatHistory.isEmpty()) {
                arLines << QString("AR EPOCH note=float_epoch_history_preserved trackedSats=%1")
                               .arg(_arSatHistory.size());
            }
            if (!_lastArFixedSignature.isEmpty()) {
                QStringList dropped;
                for (auto it = _lastArFixedSignature.constBegin(); it != _lastArFixedSignature.constEnd(); ++it) {
                    dropped << QString(it.key());
                }
                arLines << QString("AR FIX_STATE cleared systems=%1 reason=current_epoch_float")
                               .arg(dropped.join(QLatin1Char(',')));
            }
            _lastArFixedSignature.clear();
            _lastArFixedEpoch.clear();
        } else {
            QMap<QChar, QString> nextSignatures;
            QMap<QChar, int> nextEpochs;
            for (const AmbFix::FixedSet& fixedSet : res.fixedSets) {
                nextSignatures[fixedSet.sys] = fixedSet.signature;
                nextEpochs[fixedSet.sys] = _arEpochCounter;
            }
            for (auto it = _lastArFixedSignature.constBegin(); it != _lastArFixedSignature.constEnd(); ++it) {
                if (!nextSignatures.contains(it.key())) {
                    arLines << QString("AR SYS=%1 FIX_STATE class=lost epoch=%2 prevEpoch=%3 prevSignature=%4")
                                   .arg(QString(it.key()))
                                   .arg(_arEpochCounter)
                                   .arg(_lastArFixedEpoch.value(it.key(), -1))
                                   .arg(it.value());
                }
            }
            _lastArFixedSignature = nextSignatures;
            _lastArFixedEpoch = nextEpochs;
        }

        if (!res.summary.isEmpty()) {
            if (!_arSummary.isEmpty()) {
                _arSummary += " ";
            }
            _arSummary += res.summary;
        }
        _arEpochLogLines += arLines;
    }

    void initModels() {
        _antexLoaded = false;
        _oceanLoaded = false;
        if (_opt->useAntex && !_opt->antexFile.trimmed().isEmpty()) {
            _antexLoaded = _antex.load(_opt->antexFile);
        }
        if (_opt->useOceanTide && !_opt->blqFile.trimmed().isEmpty()) {
            _oceanLoaded = _ocean.load(_opt->blqFile);
        }
    }

    void reset() {
        _mwStates.clear();
        _arSatHistory.clear();
        _lastDisc1.clear();
        _lastDisc2.clear();
        _slipDisc1.clear();
        _slipDisc2.clear();
        _slipLock1.clear();
        _slipLock2.clear();
        _lastObsSeen.clear();
        _lastArFixedSignature.clear();
        _lastArFixedEpoch.clear();
        _lastArReference.clear();
        _lastBieReferenceByGroup.clear();
        _biePersistent.clear();
        _arEpochCounter = 0;
        _arFixedPrns.clear();
        _arEligiblePrns.clear();
        _arRatio = 0.0;
        _arBootstrap = 0.0;
        _arCandidateCount = 0;
        _arEffectiveCandidates = 0.0;
        _arConstraintsApplied = 0;
        _arGroups.clear();
        _params.clear();
        _Q.clear();
        rebuildParamIndexCache();
    }

    void resetCurrentGeometry() {
        _clkBanc = 0.0;
        _baseXyz = {};
        _xcBanc = {};
        _ellBanc = {};
        _neuRefInit = false;
        _neuRefXyz = {};
    }

    void rebuildParamIndexCache() {
        _idxScalar.clear();
        _idxIono.clear();
        _idxAmb1.clear();
        _idxAmb2.clear();
        _idxAmbIf.clear();
        for (const Param& p : _params) {
            if (p.index <= 0) {
                continue;
            }
            if (p.type == IONO_L1) {
                _idxIono[p.prn] = p.index - 1;
            } else if (p.type == AMB_L1) {
                _idxAmb1[p.prn] = p.index - 1;
            } else if (p.type == AMB_L2) {
                _idxAmb2[p.prn] = p.index - 1;
            } else if (p.type == AMB_IF) {
                _idxAmbIf[p.prn] = p.index - 1;
            } else {
                _idxScalar[static_cast<int>(p.type)] = p.index - 1;
            }
        }
    }

    double paramValue(ParamType type) const {
        const auto it = _idxScalar.constFind(static_cast<int>(type));
        if (it != _idxScalar.constEnd() && it.value() >= 0 && it.value() < _params.size()) {
            return _params[it.value()].value;
        }
        return 0.0;
    }

    double paramValue(ParamType type, const QString& prn) const {
        const QMap<QString, int>* mapPtr = nullptr;
        if (type == IONO_L1) {
            mapPtr = &_idxIono;
        } else if (type == AMB_L1) {
            mapPtr = &_idxAmb1;
        } else if (type == AMB_L2) {
            mapPtr = &_idxAmb2;
        } else if (type == AMB_IF) {
            mapPtr = &_idxAmbIf;
        } else {
            return 0.0;
        }
        const auto it = mapPtr->constFind(prn);
        if (it != mapPtr->constEnd() && it.value() >= 0 && it.value() < _params.size()) {
            return _params[it.value()].value;
        }
        return 0.0;
    }

    double paramStd(ParamType type) const {
        const auto it = _idxScalar.constFind(static_cast<int>(type));
        if (it != _idxScalar.constEnd() && it.value() >= 0 && it.value() < _Q.size()) {
            const double diag = _Q[it.value()][it.value()];
            if (!std::isfinite(diag)) {
                return 0.0;
            }
            return std::sqrt(qMax(0.0, diag));
        }
        return 0.0;
    }

    double paramEstimate(ParamType type) const {
        const auto it = _idxScalar.constFind(static_cast<int>(type));
        if (it != _idxScalar.constEnd() && it.value() >= 0 && it.value() < _params.size()) {
            const Param& param = _params[it.value()];
            return param.x0 + param.value;
        }
        return 0.0;
    }

    int paramIndex(ParamType type) const {
        const auto it = _idxScalar.constFind(static_cast<int>(type));
        if (it != _idxScalar.constEnd() && it.value() >= 0 && it.value() < _params.size()) {
            return it.value();
        }
        return -1;
    }

    ParamType clockParamForSystem(QChar system) const {
        switch (system.toLatin1()) {
        case 'R': return CLK_R;
        case 'E': return CLK_E;
        case 'C': return CLK_C;
        case 'G':
        default:
            return CLK_G;
        }
    }

    static bool isCodeBiasParam(ParamType type) {
        return type == CBIAS_G1 || type == CBIAS_R1 || type == CBIAS_E1 || type == CBIAS_C1 ||
               type == CBIAS_G2 || type == CBIAS_R2 || type == CBIAS_E2 || type == CBIAS_C2;
    }

    static bool isPhaseBiasParam(ParamType type) {
        return type == PBIAS_G1 || type == PBIAS_R1 || type == PBIAS_E1 || type == PBIAS_C1 ||
               type == PBIAS_G2 || type == PBIAS_R2 || type == PBIAS_E2 || type == PBIAS_C2;
    }

    static bool isBiasParam(ParamType type) {
        return isCodeBiasParam(type) || isPhaseBiasParam(type);
    }

    static QChar biasSystem(ParamType type) {
        switch (type) {
        case CBIAS_R1: case CBIAS_R2: case PBIAS_R1: case PBIAS_R2: return QLatin1Char('R');
        case CBIAS_E1: case CBIAS_E2: case PBIAS_E1: case PBIAS_E2: return QLatin1Char('E');
        case CBIAS_C1: case CBIAS_C2: case PBIAS_C1: case PBIAS_C2: return QLatin1Char('C');
        case CBIAS_G1: case CBIAS_G2: case PBIAS_G1: case PBIAS_G2: return QLatin1Char('G');
        default: break;
        }
        return QChar();
    }

    static ObservationCombination biasLc(ParamType type) {
        switch (type) {
        case CBIAS_G1: case CBIAS_R1: case CBIAS_E1: case CBIAS_C1: return ObservationCombination::C1;
        case CBIAS_G2: case CBIAS_R2: case CBIAS_E2: case CBIAS_C2: return ObservationCombination::C2;
        case PBIAS_G1: case PBIAS_R1: case PBIAS_E1: case PBIAS_C1: return ObservationCombination::L1;
        case PBIAS_G2: case PBIAS_R2: case PBIAS_E2: case PBIAS_C2: return ObservationCombination::L2;
        default: break;
        }
        return ObservationCombination::Dummy;
    }

    ParamType codeBiasParamFor(QChar system, ObservationCombination lc) const {
        if (lc == ObservationCombination::C2) {
            switch (system.toLatin1()) {
            case 'R': return CBIAS_R2;
            case 'E': return CBIAS_E2;
            case 'C': return CBIAS_C2;
            case 'G':
            default: return CBIAS_G2;
            }
        }
        switch (system.toLatin1()) {
        case 'R': return CBIAS_R1;
        case 'E': return CBIAS_E1;
        case 'C': return CBIAS_C1;
        case 'G':
        default: return CBIAS_G1;
        }
    }

    ParamType phaseBiasParamFor(QChar system, ObservationCombination lc) const {
        if (lc == ObservationCombination::L2) {
            switch (system.toLatin1()) {
            case 'R': return PBIAS_R2;
            case 'E': return PBIAS_E2;
            case 'C': return PBIAS_C2;
            case 'G':
            default: return PBIAS_G2;
            }
        }
        switch (system.toLatin1()) {
        case 'R': return PBIAS_R1;
        case 'E': return PBIAS_E1;
        case 'C': return PBIAS_C1;
        case 'G':
        default: return PBIAS_G1;
        }
    }

    double ionCoeff(const PppSatData& sat, ObsKind obsKind) const {
        if (sat.f1 <= 0.0 || sat.f2 <= 0.0) {
            return 0.0;
        }
        const double mu1 = (kFreqGpsL1Hz * kFreqGpsL1Hz) / (sat.f1 * sat.f1);
        const double mu2 = (kFreqGpsL1Hz * kFreqGpsL1Hz) / (sat.f2 * sat.f2);
        switch (obsKind) {
        case CODE1:  return  mu1;
        case CODE2:  return  mu2;
        case CODE_IF: return 0.0;
        case PHASE1: return -mu1;
        case PHASE2: return -mu2;
        case PHASE_IF: return 0.0;
        case GIM: return 0.0;
        }
        return 0.0;
    }

    double obsSigma(const PppSatData& sat, ObsKind obsKind) const {
        if (obsKind == GIM) {
            return qMax(1e-6, _opt->sigmaGIM);
        }
        if (obsKind == CODE_IF || obsKind == PHASE_IF) {
            const ObsKind k1 = (obsKind == CODE_IF) ? CODE1 : PHASE1;
            const ObsKind k2 = (obsKind == CODE_IF) ? CODE2 : PHASE2;
            const double s1 = obsSigma(sat, k1);
            const double s2 = obsSigma(sat, k2);
            return std::sqrt(sat.lkA * sat.lkA * s1 * s1 +
                             sat.lkB * sat.lkB * s2 * s2);
        }
        const bool isCode = (obsKind == CODE1 || obsKind == CODE2);
        const bool isPhase = !isCode;
        double sigma = isCode ? _opt->sigmaC1 : _opt->sigmaL1;

        if (sat.system() == QLatin1Char('R')) {
            sigma *= kGloWeightFactor;
        } else if (sat.system() == QLatin1Char('C')) {
            sigma *= kBdsWeightFactor;
        }

        const bool useEleWeight = (isCode && _opt->eleWgtCode) || (isPhase && _opt->eleWgtPhase);
        if (useEleWeight) {
            const double eleDeg = sat.eleSat * kRhoDeg;
            const double hlp = std::fabs(90.0 - eleDeg);
            sigma *= (1.0 + hlp * hlp * hlp * 0.000004);
        }
        return qMax(1e-6, sigma);
    }

    double obsMaxRes(const PppSatData& sat, ObsKind obsKind) const {
        if (obsKind == GIM) {
            return std::sqrt(_opt->maxResGIM * _opt->maxResGIM +
                             _opt->maxResGIM * _opt->maxResGIM);
        }
        if (obsKind == CODE_IF || obsKind == PHASE_IF) {
            const double base = (obsKind == CODE_IF) ? _opt->maxResC1 : _opt->maxResL1;
            return std::sqrt(sat.lkA * sat.lkA * base * base +
                             sat.lkB * sat.lkB * base * base);
        }
        return (obsKind == PHASE1 || obsKind == PHASE2) ? _opt->maxResL1 : _opt->maxResC1;
    }

    double obsValue(const PppSatData& sat, ObservationCombination lc) const {
        switch (lc) {
        case ObservationCombination::C1: return sat.C1;
        case ObservationCombination::C2: return sat.C2;
        case ObservationCombination::CIf: return sat.P3;
        case ObservationCombination::L1: return sat.L1 * sat.lambda1;
        case ObservationCombination::L2: return sat.L2 * sat.lambda2;
        case ObservationCombination::LIf: return sat.L3;
        case ObservationCombination::Gim: return sat.stecSat;
        case ObservationCombination::Dummy:
        default: break;
        }
        return 0.0;
    }

    bool obsAvailable(const PppSatData& sat, ObservationCombination lc) const {
        switch (lc) {
        case ObservationCombination::C1: return sat.C1 != 0.0;
        case ObservationCombination::C2: return sat.C2 != 0.0;
        case ObservationCombination::CIf: return sat.C1 != 0.0 && sat.C2 != 0.0 && sat.P3 != 0.0;
        case ObservationCombination::L1: return sat.L1 != 0.0 && sat.lambda1 > 0.0;
        case ObservationCombination::L2: return sat.L2 != 0.0 && sat.lambda2 > 0.0;
        case ObservationCombination::LIf: return sat.L1 != 0.0 && sat.L2 != 0.0 && sat.L3 != 0.0 && sat.lambda3 > 0.0;
        case ObservationCombination::Gim: return sat.stecSat != 0.0;
        case ObservationCombination::Dummy:
        default: break;
        }
        return false;
    }

    double lambdaForLc(const PppSatData& sat, ObservationCombination lc) const {
        switch (lc) {
        case ObservationCombination::L1: return sat.lambda1;
        case ObservationCombination::L2: return sat.lambda2;
        case ObservationCombination::LIf: return sat.lambda3;
        default: break;
        }
        return 0.0;
    }

    double ambiguitySeed(const PppSatData& sat, ObservationCombination lc) {
        if (lc == ObservationCombination::LIf) {
            if (!obsAvailable(sat, lc)) {
                return 0.0;
            }
            const double cmp = cmpValue(const_cast<PppSatData&>(sat), PHASE_IF);
            return obsValue(sat, lc) - cmp;
        }
        const double lambda = lambdaForLc(sat, lc);
        if (lambda <= 0.0 || !obsAvailable(sat, lc)) {
            return 0.0;
        }
        const ObsKind kind = obsKindForLc(lc);
        const double cmp = cmpValue(const_cast<PppSatData&>(sat), kind);
        return std::floor((obsValue(sat, lc) - cmp) / lambda + 0.5);
    }

    double partial(const Param& par, const PppSatData& sat, ObsKind obsKind) const {
        if (obsKind == PHASE_IF && par.type == AMB_IF) {
            return (sat.prn == par.prn) ? 1.0 : 0.0;
        }
        if (obsKind == CODE_IF) {
            return sat.lkA * partial(par, sat, CODE1) +
                   sat.lkB * partial(par, sat, CODE2);
        }
        if (obsKind == PHASE_IF) {
            return sat.lkA * partial(par, sat, PHASE1) +
                   sat.lkB * partial(par, sat, PHASE2);
        }
        if (obsKind == GIM) {
            return (par.type == IONO_L1 && sat.prn == par.prn) ? 1.0 : 0.0;
        }
        const Vec3 rec = _baseXyz;
        if (par.type == CRD_X) return (rec.x - sat.satPos.x) / sat.rho;
        if (par.type == CRD_Y) return (rec.y - sat.satPos.y) / sat.rho;
        if (par.type == CRD_Z) return (rec.z - sat.satPos.z) / sat.rho;
        if (par.type == CLK_G) return sat.system() == QLatin1Char('G') ? 1.0 : 0.0;
        if (par.type == CLK_R) return sat.system() == QLatin1Char('R') ? 1.0 : 0.0;
        if (par.type == CLK_E) return sat.system() == QLatin1Char('E') ? 1.0 : 0.0;
        if (par.type == CLK_C) return sat.system() == QLatin1Char('C') ? 1.0 : 0.0;
        if (par.type == TROPO) {
            return sat.modelWetMapping;
        }
        if (par.type == IONO_L1 && sat.prn == par.prn) {
            return ionCoeff(sat, obsKind);
        }
        if (par.type == AMB_L1) return (obsKind == PHASE1 && sat.prn == par.prn) ? sat.lambda1 : 0.0;
        if (par.type == AMB_L2) return (obsKind == PHASE2 && sat.prn == par.prn) ? sat.lambda2 : 0.0;
        if (isCodeBiasParam(par.type) && sat.system() == biasSystem(par.type)) {
            const ObservationCombination lc = biasLc(par.type);
            if ((lc == ObservationCombination::C1 && obsKind == CODE1) ||
                (lc == ObservationCombination::C2 && obsKind == CODE2)) {
                return 1.0;
            }
        }
        if (isPhaseBiasParam(par.type) && sat.system() == biasSystem(par.type)) {
            const ObservationCombination lc = biasLc(par.type);
            if ((lc == ObservationCombination::L1 && obsKind == PHASE1) ||
                (lc == ObservationCombination::L2 && obsKind == PHASE2)) {
                return 1.0;
            }
        }
        return 0.0;
    }

    void cmpEle(PppSatData& sat) {
        Vec3 rr = sat.satPos - _baseXyz;
        sat.rho = norm(rr);
        if (!std::isfinite(sat.rho) || sat.rho <= 1.0) {
            sat.eleSat = -kPi / 2.0;
            sat.azSat = 0.0;
            return;
        }
        Vec3 neu = xyzToNeu(_ellBanc, rr);
        double cosZen = std::sqrt(neu.x * neu.x + neu.y * neu.y) / sat.rho;
        cosZen = clampValue(cosZen, -1.0, 1.0);
        sat.eleSat = std::acos(cosZen);
        if (neu.z < 0.0) {
            sat.eleSat *= -1.0;
        }
        sat.azSat = std::atan2(neu.y, neu.x);
    }

    static double lorentz4(const Vec& a, const Vec& b) {
        if (a.size() < 4 || b.size() < 4) {
            return 0.0;
        }
        return a[0] * b[0] + a[1] * b[1] + a[2] * b[2] - a[3] * b[3];
    }

    double bancroftCmpCorrection(const PppSatData& sat, ObservationCombination lc) {
        if (!sat.modelSet) {
            return -sat.clk;
        }
        double dispPart = 0.0;
        if (lc == ObservationCombination::C1) {
            dispPart = sat.modelAntPco1 - sat.codeBias1;
        } else if (lc == ObservationCombination::C2) {
            dispPart = sat.modelAntPco2 - sat.codeBias2;
        } else if (lc == ObservationCombination::CIf) {
            dispPart = sat.lkA * (sat.modelAntPco1 - sat.codeBias1) +
                       sat.lkB * (sat.modelAntPco2 - sat.codeBias2);
        }
        return -sat.modelSatClkM + sat.modelAntEcc + sat.modelTropo
             + sat.modelTideEarth + sat.modelTideOcean + sat.modelRel + dispPart;
    }

    bool solveBancroft(const QVector<PppSatData>& sats, ObservationCombination lc, Vec3& pos, double& clkMeters) {
        if (sats.size() < 4) {
            return false;
        }

        Vec sol(4, 0.0);
        for (int iter = 0; iter < 2; ++iter) {
            Mat B = zeros(sats.size(), 4);
            for (int i = 0; i < sats.size(); ++i) {
                const PppSatData& sat = sats[i];
                double x = sat.satPos.x;
                double y = sat.satPos.y;
                double travelTime = 0.072;
                if (iter > 0) {
                    const double rho = std::sqrt((x - sol[0]) * (x - sol[0]) +
                                                 (y - sol[1]) * (y - sol[1]) +
                                                 (sat.satPos.z - sol[2]) * (sat.satPos.z - sol[2]));
                    travelTime = rho / GnssUtils::kC;
                }
                const double angle = travelTime * GnssUtils::kOmegaEarth;
                const double ca = std::cos(angle);
                const double sa = std::sin(angle);
                B[i][0] = ca * x + sa * y;
                B[i][1] = -sa * x + ca * y;
                B[i][2] = sat.satPos.z;
                const double codeSeed = obsValue(sat, lc);
                B[i][3] = codeSeed - bancroftCmpCorrection(sat, lc);
            }

            Mat Bt = transpose(B);
            Mat Binv;
            if (sats.size() > 4) {
                Mat n = multiply(Bt, B);
                Mat nInv;
                if (!invert(n, nInv)) {
                    return false;
                }
                Binv = multiply(nInv, Bt);
            } else {
                if (!invert(B, Binv)) {
                    return false;
                }
            }

            Vec ee(sats.size(), 1.0);
            Vec alpha(sats.size(), 0.0);
            for (int i = 0; i < sats.size(); ++i) {
                alpha[i] = lorentz4(B[i], B[i]) / 2.0;
            }
            Vec be = multiply(Binv, ee);
            Vec ba = multiply(Binv, alpha);
            const double aa = lorentz4(be, be);
            const double bb = lorentz4(be, ba) - 1.0;
            const double cc = lorentz4(ba, ba);
            const double disc = bb * bb - aa * cc;
            if (!std::isfinite(disc) || disc < 0.0 || std::fabs(aa) < 1e-20) {
                return false;
            }
            const double root = std::sqrt(disc);
            Vec candA(4, 0.0);
            Vec candB(4, 0.0);
            for (int i = 0; i < 4; ++i) {
                candA[i] = (-bb - root) / aa * be[i] + ba[i];
                candB[i] = (-bb + root) / aa * be[i] + ba[i];
            }
            candA[3] = -candA[3];
            candB[3] = -candB[3];
            auto omc = [&](const Vec& c) {
                return B[0][3] - std::sqrt((B[0][0] - c[0]) * (B[0][0] - c[0]) +
                                           (B[0][1] - c[1]) * (B[0][1] - c[1]) +
                                           (B[0][2] - c[2]) * (B[0][2] - c[2])) - c[3];
            };
            sol = (std::fabs(omc(candA)) > std::fabs(omc(candB))) ? candB : candA;
        }

        pos = {sol[0], sol[1], sol[2]};
        clkMeters = sol[3];
        return isReasonableReceiverXyz(pos) && std::isfinite(clkMeters);
    }

    bool cmpBancroft(PppEpochData& epoData, bool useExistingModel, bool print) {
        QVector<PppSatData> work;
        work.reserve(epoData.satData.size());
        ObservationCombination bancLc = ObservationCombination::Dummy;
        for (auto it = epoData.satData.constBegin(); it != epoData.satData.constEnd(); ++it) {
            const PppSatData& sat = it.value();
            if (bancLc == ObservationCombination::Dummy) {
                if (obsAvailable(sat, ObservationCombination::CIf)) bancLc = ObservationCombination::CIf;
                else if (obsAvailable(sat, ObservationCombination::C1)) bancLc = ObservationCombination::C1;
                else if (obsAvailable(sat, ObservationCombination::C2)) bancLc = ObservationCombination::C2;
            }
            if (bancLc != ObservationCombination::Dummy && obsAvailable(sat, bancLc) &&
                (!useExistingModel || !sat.modelSet || sat.eleSat >= _opt->minEleRad)) {
                work.push_back(sat);
            }
        }
        if (bancLc == ObservationCombination::Dummy) {
            return false;
        }

        Vec3 p{};
        double clk = 0.0;
        const double blunder = 1500.0;
        while (work.size() >= _opt->minObs) {
            if (!solveBancroft(work, bancLc, p, clk)) {
                return false;
            }
            double maxRes = 0.0;
            int maxIndex = -1;
            for (int i = 0; i < work.size(); ++i) {
                const PppSatData& sat = work[i];
                const double rho = norm(sat.satPos - p);
                const double res = rho - obsValue(sat, bancLc) - (sat.clk - clk);
                if (std::fabs(res) > maxRes) {
                    maxRes = std::fabs(res);
                    maxIndex = i;
                }
            }
            if (maxRes < blunder || maxIndex < 0) {
                break;
            }
            _solutionLogLines << QString("t_pppClient::cmpBancroft Outlier %1 %2")
                .arg(work[maxIndex].prn, QString::number(maxRes, 'f', 4));
            epoData.satData.remove(work[maxIndex].prn);
            work.removeAt(maxIndex);
        }

        if (!isReasonableReceiverXyz(p)) {
            return false;
        }

        _xcBanc = p;
        if (_params.isEmpty() || paramIndex(CRD_X) < 0 || paramIndex(CRD_Y) < 0 || paramIndex(CRD_Z) < 0) {
            _baseXyz = _opt->hasApriori() ? Vec3{_opt->aprXyz[0], _opt->aprXyz[1], _opt->aprXyz[2]} : p;
        }
        if (norm(_baseXyz) <= 1.0 || !isReasonableReceiverXyz(_baseXyz)) {
            _baseXyz = _opt->hasApriori() ? Vec3{_opt->aprXyz[0], _opt->aprXyz[1], _opt->aprXyz[2]} : p;
        }
        _clkBanc = clk;
        _ellBanc = xyzToEll(_baseXyz);
        if (print) {
            _solutionLogLines << QString("%1 BANCROFT: %2 %3 %4 %5")
                .arg(epochTimeLabel(),
                     QString::number(p.x, 'f', 3),
                     QString::number(p.y, 'f', 3),
                     QString::number(p.z, 'f', 3),
                     QString::number(clk, 'f', 3));
        }
        return true;
    }

    bool cmpModelForBancroft(PppEpochData& epoData) {
        for (auto it = epoData.satData.begin(); it != epoData.satData.end();) {
            PppSatData& sat = it.value();
            if (!cmpModel(sat) || sat.eleSat < _opt->minEleRad) {
                it = epoData.satData.erase(it);
            } else {
                ++it;
            }
        }
        return static_cast<int>(epoData.satData.size()) >= _opt->minObs;
    }

    bool saastamoinenAtmosphere(double& pressureHpa, double& temperatureK,
                                double& vaporPressureHpa, double& bCorrection) const {
        const Vec3 xyz = _baseXyz;
        if (!std::isfinite(xyz.x) || !std::isfinite(xyz.y) || !std::isfinite(xyz.z)) {
            return false;
        }
        Ellipsoid ell = xyzToEll(xyz);
        double height = ell.h;
        if (!std::isfinite(height)) {
            return false;
        }
        if (height > 40000.0) {
            pressureHpa = temperatureK = vaporPressureHpa = bCorrection = 0.0;
            return true;
        }
        pressureHpa = 1013.25 * std::pow(1.0 - 2.26e-5 * height, 5.225);
        temperatureK = 18.0 - height * 0.0065 + 273.15;
        const double humidity = 50.0 * std::exp(-6.396e-4 * height);
        vaporPressureHpa = humidity / 100.0 * std::exp(-37.2465 + 0.213166 * temperatureK -
                                                        0.000256908 * temperatureK * temperatureK);
        double hKm = height / 1000.0;
        if (hKm < 0.0) hKm = 0.0;
        if (hKm > 5.0) hKm = 5.0;
        int ii = static_cast<int>(hKm + 1.0);
        if (ii < 1) ii = 1;
        if (ii > 5) ii = 5;
        const double href = static_cast<double>(ii - 1);
        const double bCor[6] = {1.156, 1.006, 0.874, 0.757, 0.654, 0.563};
        bCorrection = bCor[ii - 1] + (bCor[ii] - bCor[ii - 1]) * (hKm - href);
        return std::isfinite(pressureHpa) && std::isfinite(temperatureK) &&
               std::isfinite(vaporPressureHpa) && std::isfinite(bCorrection);
    }

    double saastamoinenDrySlant(double ele) const {
        double pressureHpa = 0.0;
        double temperatureK = 0.0;
        double vaporPressureHpa = 0.0;
        double bCorrection = 0.0;
        if (!saastamoinenAtmosphere(pressureHpa, temperatureK, vaporPressureHpa, bCorrection)) {
            return 0.0;
        }
        double zen  = kPi/2.0 - ele;
        const double cosZen = std::cos(zen);
        if (!std::isfinite(cosZen) || std::fabs(cosZen) < 1e-6) {
            return 0.0;
        }
        const double val = (0.002277 / cosZen) *
                           (pressureHpa - bCorrection * (std::tan(zen) * std::tan(zen)));
        if (!std::isfinite(val)) {
            return 0.0;
        }
        return val;
    }

    double saastamoinenWetZenith() const {
        double pressureHpa = 0.0;
        double temperatureK = 0.0;
        double vaporPressureHpa = 0.0;
        double bCorrection = 0.0;
        if (!saastamoinenAtmosphere(pressureHpa, temperatureK, vaporPressureHpa, bCorrection)) {
            return 0.0;
        }
        const double val = 0.002277 * ((1255.0 / temperatureK) + 0.05) * vaporPressureHpa;
        return std::isfinite(val) ? val : 0.0;
    }

    double tropospherePriorZwd() const {
        if (_tropoPrior.valid && std::isfinite(_tropoPrior.zwdM) && _tropoPrior.zwdM >= 0.0) {
            return _tropoPrior.zwdM;
        }
        return saastamoinenWetZenith();
    }

    bool mappingFactors(double elevationRad, double& hydrostatic, double& wet) {
        const double sinEl = std::sin(elevationRad);
        if (!std::isfinite(sinEl) || std::fabs(sinEl) <= 1e-12) {
            _failureReason = QStringLiteral("invalid satellite elevation for troposphere mapping");
            return false;
        }
        if (_opt->mappingMode == PppMappingMode::SimpleSin) {
            hydrostatic = 1.0 / sinEl;
            wet = hydrostatic;
            return true;
        }
        if (!_gpt3Worker) _gpt3Worker = std::make_unique<Gpt3Vmf3Worker>();
        Gpt3Vmf3Mapping mapping;
        QString error;
        const double zenithDistance = kPi / 2.0 - elevationRad;
        if (!_gpt3Worker->mapping(_time.utc, _ellBanc.lat, _ellBanc.lon, _ellBanc.h,
                                  zenithDistance, mapping, &error)) {
            _failureReason = QStringLiteral("GPT3/VMF3 mapping failed: %1").arg(error);
            return false;
        }
        hydrostatic = mapping.hydrostatic;
        wet = mapping.wet;
        return true;
    }

    Vec3 earthTides(const Vec3& xRec) const {
        if (!_time.valid) {
            return {};
        }
        int mjdInt = 0;
        double dayfrac = 0.0;
        GnssUtils::mjdFromDateTime(_time.utc, mjdInt, dayfrac);
        const double mjd = static_cast<double>(mjdInt) + dayfrac;
        Vec3 xSun = sunPositionEcef(mjd);
        Vec3 xMoon = moonPositionEcef(mjd);
        const double rSun = norm(xSun);
        const double rMoon = norm(xMoon);
        if (rSun <= 0.0 || rMoon <= 0.0) {
            return {};
        }
        xSun = (1.0 / rSun) * xSun;
        xMoon = (1.0 / rMoon) * xMoon;

        const double rRec = norm(xRec);
        if (rRec <= 0.0) {
            return {};
        }
        const Vec3 xyzUnit = (1.0 / rRec) * xRec;

        const double H2 = 0.6078;
        const double L2 = 0.0847;

        const double scSun = dot(xyzUnit, xSun);
        const double scMoon = dot(xyzUnit, xMoon);
        const double p2Sun = 3.0 * (H2 / 2.0 - L2) * scSun * scSun - H2 / 2.0;
        const double p2Moon = 3.0 * (H2 / 2.0 - L2) * scMoon * scMoon - H2 / 2.0;
        const double x2Sun = 3.0 * L2 * scSun;
        const double x2Moon = 3.0 * L2 * scMoon;

        const double gmWgs = 398.6005e12;
        const double gmSun = 1.3271250e20;
        const double gmMoon = 4.9027890e12;

        const double facSun = gmSun / gmWgs * (rRec * rRec * rRec * rRec) / (rSun * rSun * rSun);
        const double facMoon = gmMoon / gmWgs * (rRec * rRec * rRec * rRec) / (rMoon * rMoon * rMoon);

        return facSun * (x2Sun * xSun + p2Sun * xyzUnit) +
               facMoon * (x2Moon * xMoon + p2Moon * xyzUnit);
    }

    Vec3 oceanTides(const Vec3& xRec) const {
        if (!_oceanLoaded || !_opt->useOceanTide) {
            return {};
        }
        return _ocean.ocean(_time, xRec, _opt->oceanStation);
    }

    double antennaPco(const PppSatData& sat, bool firstFreq, bool emitWarnings) {
        if (!_opt->useAntex || !_antexLoaded) {
            return 0.0;
        }

        const QChar obsBand = firstFreq ? sat.bandA : sat.bandB;
        const auto defaultFrq = defaultAntennaFrequencyPair(sat.system());
        QString frq = antexFreqKey(sat.system(), obsBand);
        if (frq.isEmpty()) {
            frq = firstFreq ? defaultFrq.first : defaultFrq.second;
        }
        if (frq.isEmpty()) {
            return 0.0;
        }

        int mjdInt = 0;
        double dayfrac = 0.0;
        GnssUtils::mjdFromDateTime(_time.utc, mjdInt, dayfrac);
        const double mjd = static_cast<double>(mjdInt) + dayfrac;

        const Vec3 xRec = _baseXyz;
        const Vec3 rho = unit(xRec - sat.satPos);
        const Vec3 xSun = unit(sunPositionEcef(mjd));
        const Vec3 sz = unit(-1.0 * sat.satPos);
        const Vec3 sy = unit(cross(sz, xSun));
        const Vec3 sx = unit(cross(sy, sz));
        const Vec3 u{dot(sx, rho), dot(sy, rho), dot(sz, rho)};
        const double elTx = std::atan2(u.z, std::sqrt(u.x * u.x + u.y * u.y));
        const double azTx = std::atan2(u.y, u.x);

        double antPco = 0.0;
        bool found = false;
        double satA = _antex.satCorr(sat.prn, frq, elTx, azTx, found);
        const bool useApc = _opt->isApc;
        if (emitWarnings && !_antApcDiagLogged) {
            _diagLines << QString("ANTEX APC mode: useApc=%1 satRefDatum=%2 ignored=yes (compatible)")
                              .arg(useApc ? "yes" : "no")
                              .arg(sat.satRefDatum);
            _antApcDiagLogged = true;
        }
        if (found) {
            if (useApc && !defaultFrq.first.isEmpty()) {
                bool foundRef = false;
                const double satRefA = _antex.satCorr(sat.prn, defaultFrq.first, elTx, azTx, foundRef);
                if (foundRef) {
                    satA -= satRefA;
                } else if (emitWarnings) {
                    warnAntexNotFound(sat.prn.mid(0, 3), defaultFrq.first);
                }
            }
            antPco += satA;
        } else if (emitWarnings) {
            warnAntexNotFound(sat.prn.mid(0, 3), frq);
        }

        if (!_opt->roverAntenna.trimmed().isEmpty()) {
            double rcvA = _antex.rcvCorr(_opt->roverAntenna, frq, sat.eleSat, sat.azSat, found);
            if (!found) {
                const QString fallback = firstFreq ? QStringLiteral("G01") : QStringLiteral("G02");
                rcvA = _antex.rcvCorr(_opt->roverAntenna, fallback, sat.eleSat, sat.azSat, found);
            }
            if (found) {
                antPco += rcvA;
            } else if (emitWarnings) {
                warnAntexNotFound(_opt->roverAntenna, frq);
            }
        }

        return antPco;
    }

    double windUp(const QString& prn, const Vec3& rSat, const Vec3& rRec,
                  const Vec3& vSat, bool useSsrYaw, double yawRad) {
        int mjdInt = 0;
        double dayfrac = 0.0;
        GnssUtils::mjdFromDateTime(_time.utc, mjdInt, dayfrac);
        const double mjd = static_cast<double>(mjdInt) + dayfrac;

        if (!_windUpTime.contains(prn)) {
            _windUpSum[prn] = 0.0;
        }
        if (!_windUpTime.contains(prn) || _windUpTime[prn] != mjd) {
            _windUpTime[prn] = mjd;

            Vec3 rho = unit(rRec - rSat);
            Vec3 sz = unit(-1.0 * rSat);
            Vec3 sHlp;
            if (!useSsrYaw) {
                sHlp = unit(sunPositionEcef(mjd));
            } else {
                sHlp = unit(vSat + cross({0.0, 0.0, GnssUtils::kOmegaEarth}, rSat));
            }
            Vec3 sy = unit(cross(sz, sHlp));
            Vec3 sx = unit(cross(sy, sz));
            if (useSsrYaw) {
                const Mat3 rot = rotZ(yawRad);
                sx = multiply(rot, sx);
                sy = multiply(rot, sy);
                sz = multiply(rot, sz);
            }

            Vec3 dipSat = sx - dot(rho, sx) * rho - cross(rho, sy);

            Ellipsoid ell = xyzToEll(rRec);
            Vec3 rx = neuToXyz(ell, {1.0, 0.0, 0.0});
            Vec3 ry = neuToXyz(ell, {0.0, -1.0, 0.0});

            Vec3 dipRec = rx - dot(rho, rx) * rho + cross(rho, ry);

            const double dipNorm = norm(dipSat) * norm(dipRec);
            if (dipNorm <= 1e-12) {
                return _windUpSum[prn];
            }
            double alpha = dot(dipSat, dipRec) / dipNorm;
            alpha = clampValue(alpha, -1.0, 1.0);
            double dphi = std::acos(alpha) / (2.0 * kPi);
            if (dot(rho, cross(dipSat, dipRec)) < 0.0) {
                dphi = -dphi;
            }
            _windUpSum[prn] = std::floor(_windUpSum[prn] - dphi + 0.5) + dphi;
        }

        return _windUpSum[prn];
    }

    bool cmpModel(PppSatData& sat) {
        sat.modelSet = false;
        sat.modelRho = 0.0;
        sat.modelSatClkM = 0.0;
        sat.modelRecClkM = 0.0;
        sat.modelSagnac = 0.0;
        sat.modelAntEcc = 0.0;
        sat.modelTropo = 0.0;
        sat.modelWetMapping = 0.0;
        sat.modelTideEarth = 0.0;
        sat.modelTideOcean = 0.0;
        sat.modelRel = 0.0;
        sat.modelAntPco1 = 0.0;
        sat.modelAntPco2 = 0.0;
        sat.modelWindUp = 0.0;
        sat.stecSat = 0.0;

        const Vec3 xRec = _baseXyz;
        const Vec3 rhoV = sat.satPos - xRec;
        sat.rho = norm(rhoV);
        sat.modelRho = sat.rho;
        if (!std::isfinite(sat.rho) || sat.rho <= 1.0) {
            sat.eleSat = -kPi / 2.0;
            sat.azSat = 0.0;
            return false;
        }

        Vec3 neu = xyzToNeu(_ellBanc, rhoV);
        double cosZen = std::sqrt(neu.x * neu.x + neu.y * neu.y) / sat.rho;
        cosZen = clampValue(cosZen, -1.0, 1.0);
        sat.eleSat = std::acos(cosZen);
        if (neu.z < 0.0) {
            sat.eleSat *= -1.0;
        }
        sat.azSat = std::atan2(neu.y, neu.x);

        sat.modelSatClkM = sat.clk;
        sat.modelRecClkM = _clkBanc;
        const Vec3 omega{0.0, 0.0, GnssUtils::kOmegaEarth / GnssUtils::kC};
        sat.modelSagnac = dot(omega, cross(sat.satPos, xRec));
        sat.modelAntEcc = -_opt->neuEcc[0] * std::cos(sat.azSat) * std::cos(sat.eleSat)
                          -_opt->neuEcc[1] * std::sin(sat.azSat) * std::cos(sat.eleSat)
                          -_opt->neuEcc[2] * std::sin(sat.eleSat);
        double hydrostaticMapping = 0.0;
        double wetMapping = 0.0;
        if (!mappingFactors(sat.eleSat, hydrostaticMapping, wetMapping)) {
            return false;
        }
        sat.modelTropo = _opt->mappingMode == PppMappingMode::SimpleSin
            ? saastamoinenDrySlant(sat.eleSat)
            : hydrostaticMapping * saastamoinenDrySlant(kPi / 2.0);
        sat.modelWetMapping = wetMapping;
        if (_vtecModel && _vtecModel->isValid() &&
            (_opt->pseudoObsIono || _opt->pppRtk) &&
            !usesIonFreeCombination(*_opt, sat.system())) {
            sat.stecSat = vtecL1DelayMeters(*_vtecModel, _time, xRec, sat.satPos,
                                            sat.rho / GnssUtils::kC);
        }

        const Vec3 rhoUnit = (1.0 / sat.rho) * rhoV;
        sat.modelTideEarth = -dot(earthTides(xRec), rhoUnit);
        sat.modelTideOcean = -dot(oceanTides(xRec), rhoUnit);

        const double a = norm(sat.satPos) + norm(xRec);
        const double b = sat.rho;
        if (a > b + 1e-6) {
            const double ratio = (a + b) / (a - b);
            if (ratio > 0.0 && std::isfinite(ratio)) {
                sat.modelRel = 2.0 * kGmEarth / (GnssUtils::kC * GnssUtils::kC) * std::log(ratio);
            }
        }

        sat.modelAntPco1 = antennaPco(sat, true, true);
        sat.modelAntPco2 = antennaPco(sat, false, true);
        sat.modelWindUp = windUp(sat.prn, sat.satPos, xRec, sat.satVel, sat.useSsrYaw, sat.yawRad);

        sat.modelSet = std::isfinite(sat.modelRho) &&
                       std::isfinite(sat.modelSatClkM) &&
                       std::isfinite(sat.modelRecClkM) &&
                       std::isfinite(sat.modelSagnac) &&
                       std::isfinite(sat.modelAntEcc) &&
                       std::isfinite(sat.modelTropo) &&
                       std::isfinite(sat.modelWetMapping) && sat.modelWetMapping > 0.0 &&
                       std::isfinite(sat.modelTideEarth) &&
                       std::isfinite(sat.modelTideOcean) &&
                       std::isfinite(sat.modelRel);
        return sat.modelSet;
    }

    double cmpValue(PppSatData& sat, ObsKind obsKind) {
        if (obsKind == CODE_IF) {
            return sat.lkA * cmpValue(sat, CODE1) +
                   sat.lkB * cmpValue(sat, CODE2);
        }
        if (obsKind == PHASE_IF) {
            return sat.lkA * cmpValue(sat, PHASE1) +
                   sat.lkB * cmpValue(sat, PHASE2);
        }
        if (obsKind == GIM) {
            return sat.stecSat;
        }
        if (!sat.modelSet && !cmpModel(sat)) {
            return 0.0;
        }
        const bool phase = (obsKind == PHASE1 || obsKind == PHASE2);
        const bool firstFreq = (obsKind == CODE1 || obsKind == PHASE1);
        const double obsLambda = firstFreq ? sat.lambda1 : sat.lambda2;
        const double sinEl = std::sin(sat.eleSat);
        if (!std::isfinite(sinEl)) {
            return 0.0;
        }
        const double ssrBias = firstFreq
            ? (phase ? sat.phaseBias1 : sat.codeBias1)
            : (phase ? sat.phaseBias2 : sat.codeBias2);
        const double antPco = firstFreq ? sat.modelAntPco1 : sat.modelAntPco2;
        const double wind = (phase && obsLambda > 0.0) ? sat.modelWindUp * obsLambda : 0.0;
        const double nonDisp = sat.modelRho + sat.modelRecClkM - sat.modelSatClkM
                             + sat.modelSagnac + sat.modelAntEcc + sat.modelTropo
                             + sat.modelTideEarth + sat.modelTideOcean + sat.modelRel;
        double dispPart = antPco - ssrBias + wind;
        if (_opt->pppRtk && sat.stecSat != 0.0) {
            dispPart += ionCoeff(sat, obsKind) * sat.stecSat;
        }
        return nonDisp + dispPart;
    }

    void updateAmbiguities(PppEpochData& epoData, bool usePhase,
                           bool firstCrd, bool setCrdNoiseToZero) {
        const QVector<Param> oldParams = _params;
        const Mat Q_old = _Q;
        const double keepGapSec = 60.0;

        for (auto it = epoData.satData.constBegin(); it != epoData.satData.constEnd(); ++it) {
            _lastObsSeen[it.key()] = _time;
        }

        auto isClockParam = [&](ParamType type) {
            return type == CLK_G || type == CLK_R || type == CLK_E || type == CLK_C;
        };
        auto pushUnique = [&](QVector<Param>& target, ParamType type, const QString& prn = QString(), ObservationCombination lc = ObservationCombination::Dummy) {
            for (const Param& pp : target) {
                if (pp.type == type && pp.prn == prn && pp.lc == lc) {
                    return;
                }
            }
            Param p;
            p.type = type;
            p.prn = prn;
            p.lc = (lc != ObservationCombination::Dummy) ? lc : biasLc(type);
            target.push_back(p);
        };

        QSet<QChar> usedSystems;
        for (auto it = epoData.satData.constBegin(); it != epoData.satData.constEnd(); ++it) {
            const QChar sys = it.value().system();
            if (_opt->useSystem(sys)) {
                usedSystems.insert(sys);
            }
        }

        QVector<Param> required;
        pushUnique(required, CRD_X);
        pushUnique(required, CRD_Y);
        pushUnique(required, CRD_Z);
        if (usedSystems.contains(QLatin1Char('G'))) pushUnique(required, CLK_G);
        if (usedSystems.contains(QLatin1Char('R'))) pushUnique(required, CLK_R);
        if (usedSystems.contains(QLatin1Char('E'))) pushUnique(required, CLK_E);
        if (usedSystems.contains(QLatin1Char('C'))) pushUnique(required, CLK_C);
        if (_opt->estimateTroposphere) {
            pushUnique(required, TROPO);
        }

        if (_opt->estimateIonosphere && !_opt->pppRtk) {
            for (auto it = epoData.satData.constBegin(); it != epoData.satData.constEnd(); ++it) {
                const PppSatData& sat = it.value();
                if (!usesIonFreeCombination(*_opt, sat.system())) {
                    pushUnique(required, IONO_L1, it.key());
                }
            }
        }

        for (auto it = epoData.satData.constBegin(); it != epoData.satData.constEnd(); ++it) {
            const PppSatData& sat = it.value();
            if (!usePhase) {
                continue;
            }
            for (ObservationCombination lc : ambiguityCombinationsForSystem(*_opt, sat.system())) {
                if (obsAvailable(sat, lc)) {
                    pushUnique(required, ambParamTypeForLc(lc), it.key(), lc);
                }
            }
        }

        const QVector<QChar> enabledSystems = {QLatin1Char('G'), QLatin1Char('R'),
                                               QLatin1Char('E'), QLatin1Char('C')};

        if (_opt->pseudoObsIono) {
            for (QChar sys : enabledSystems) {
                if (!_opt->useSystem(sys)) {
                    continue;
                }
                const QVector<ObservationCombination> lcs = combinationsForSystem(*_opt, sys);
                if (lcs.contains(ObservationCombination::C2)) {
                    pushUnique(required, codeBiasParamFor(sys, ObservationCombination::C2), QString(), ObservationCombination::C2);
                }
            }
        }
        if (_opt->pppRtk) {
            for (QChar sys : enabledSystems) {
                if (!_opt->useSystem(sys)) {
                    continue;
                }
                const QVector<ObservationCombination> lcs = combinationsForSystem(*_opt, sys);
                if (lcs.contains(ObservationCombination::C1)) {
                    pushUnique(required, codeBiasParamFor(sys, ObservationCombination::C1), QString(), ObservationCombination::C1);
                }
                if (lcs.contains(ObservationCombination::C2)) {
                    pushUnique(required, codeBiasParamFor(sys, ObservationCombination::C2), QString(), ObservationCombination::C2);
                }
                if (lcs.contains(ObservationCombination::L1)) {
                    pushUnique(required, phaseBiasParamFor(sys, ObservationCombination::L1), QString(), ObservationCombination::L1);
                }
                if (lcs.contains(ObservationCombination::L2)) {
                    pushUnique(required, phaseBiasParamFor(sys, ObservationCombination::L2), QString(), ObservationCombination::L2);
                }
            }
        }

        for (const Param& pOld : oldParams) {
            if (!(pOld.type == IONO_L1 || pOld.type == AMB_L1 ||
                  pOld.type == AMB_L2 || pOld.type == AMB_IF || isBiasParam(pOld.type))) {
                continue;
            }
            if (!pOld.prn.isEmpty() && epoData.satData.contains(pOld.prn)) {
                continue;
            }
            const PppTime lastSeen = pOld.prn.isEmpty() ? pOld.lastObsTime : _lastObsSeen.value(pOld.prn);
            if (!lastSeen.valid) {
                if (!pOld.prn.isEmpty()) _lastObsSeen.remove(pOld.prn);
                continue;
            }
            const double gap = diffSeconds(_time, lastSeen);
            if (gap < 0.0 || gap > keepGapSec) {
                if (!pOld.prn.isEmpty()) _lastObsSeen.remove(pOld.prn);
                continue;
            }
            pushUnique(required, pOld.type, pOld.prn, pOld.lc);
        }

        int idx = 0;
        for (Param& p : required) {
            p.indexOld = 0;
            p.value = 0.0;

            const bool forceNewAmb = false;
            if (!isClockParam(p.type) && !forceNewAmb) {
                for (const Param& pOld : oldParams) {
                    if (pOld.type == p.type && pOld.prn == p.prn && pOld.lc == p.lc) {
                        p.indexOld = pOld.index;
                        p.value = pOld.value;
                        p.x0 = pOld.x0;
                        p.firstObsTime = pOld.firstObsTime;
                        p.lastObsTime = pOld.lastObsTime;
                        p.ambEleSat = pOld.ambEleSat;
                        p.ambNumEpo = pOld.ambNumEpo;
                        break;
                    }
                }
            }
            if (p.prn.isEmpty()) {
                p.lastObsTime = _time;
                if (!p.firstObsTime.valid) p.firstObsTime = _time;
            } else if (epoData.satData.contains(p.prn)) {
                p.lastObsTime = _time;
                if (!p.firstObsTime.valid) p.firstObsTime = _time;
                if (p.type == AMB_L1 || p.type == AMB_L2 || p.type == AMB_IF) {
                    p.ambEleSat = epoData.satData.value(p.prn).eleSat;
                    ++p.ambNumEpo;
                }
            }
            p.index = ++idx;
        }

        _params = required;
        rebuildParamIndexCache();

        for (Param& p : _params) {
            if (p.indexOld != 0) {
                continue;
            }
            if (p.type == CRD_X) {
                p.value = 0.0;
            } else if (p.type == CRD_Y) {
                p.value = 0.0;
            } else if (p.type == CRD_Z) {
                p.value = 0.0;
            } else if (isClockParam(p.type)) {
                p.value = 0.0;
            } else if (p.type == TROPO) {
                p.value = 0.0;
                p.x0 = tropospherePriorZwd();
            } else if (p.type == IONO_L1) {
                p.value = 0.0;
            }
        }
        for (Param& p : _params) {
            if (p.indexOld != 0) {
                continue;
            }
            const auto itSat = epoData.satData.constFind(p.prn);
            if (itSat == epoData.satData.constEnd()) {
                continue;
            }
            PppSatData sat = itSat.value();
            if (p.type == AMB_IF && obsAvailable(sat, ObservationCombination::LIf)) {
                p.x0 = ambiguitySeed(sat, ObservationCombination::LIf);
                p.value = 0.0;
            } else if (p.type == AMB_L1 && obsAvailable(sat, ObservationCombination::L1)) {
                p.x0 = ambiguitySeed(sat, ObservationCombination::L1);
                p.value = 0.0;
            } else if (p.type == AMB_L2 && obsAvailable(sat, ObservationCombination::L2)) {
                p.x0 = ambiguitySeed(sat, ObservationCombination::L2);
                p.value = 0.0;
            }
        }

        Mat Qnew = zeros(_params.size(), _params.size());
        for (int i = 0; i < _params.size(); ++i) {
            const Param& p1 = _params[i];
            if (p1.indexOld <= 0 || p1.indexOld - 1 >= Q_old.size()) {
                continue;
            }
            for (int j = 0; j < _params.size(); ++j) {
                const Param& p2 = _params[j];
                if (p2.indexOld > 0 && p2.indexOld - 1 < Q_old.size()) {
                    Qnew[i][j] = Q_old[p1.indexOld - 1][p2.indexOld - 1];
                }
            }
        }

        bool firstCrdState = false;
        double qNeu[3] = {0.0, 0.0, 0.0};
        bool addCrdNoise = false;
        for (int i = 0; i < _params.size(); ++i) {
            const Param& p = _params[i];
            if (p.indexOld == 0) {
                if (p.type == CRD_X || p.type == CRD_Y || p.type == CRD_Z) {
                    firstCrdState = true;
                    if (p.type == CRD_X) qNeu[0] = _opt->aprSigCrd[0] * _opt->aprSigCrd[0];
                    else if (p.type == CRD_Y) qNeu[1] = _opt->aprSigCrd[1] * _opt->aprSigCrd[1];
                    else if (p.type == CRD_Z) qNeu[2] = _opt->aprSigCrd[2] * _opt->aprSigCrd[2];
                } else if (isClockParam(p.type)) Qnew[i][i] = _opt->aprSigClk * _opt->aprSigClk;
                else if (p.type == TROPO) Qnew[i][i] = _opt->aprSigTrp * _opt->aprSigTrp;
                else if (p.type == IONO_L1) Qnew[i][i] = _opt->aprSigIon * _opt->aprSigIon;
                else if (p.type == AMB_L1 || p.type == AMB_L2 || p.type == AMB_IF) Qnew[i][i] = _opt->aprSigAmb * _opt->aprSigAmb;
                else if (isCodeBiasParam(p.type)) Qnew[i][i] = _opt->aprSigCodeBias * _opt->aprSigCodeBias;
                else if (isPhaseBiasParam(p.type)) Qnew[i][i] = _opt->aprSigPhaseBias * _opt->aprSigPhaseBias;
            } else {
                double qAdd = 0.0;
                if (p.type == CRD_X || p.type == CRD_Y || p.type == CRD_Z) {
                    if (!setCrdNoiseToZero) {
                        addCrdNoise = true;
                        if (p.type == CRD_X) qNeu[0] = _opt->noiseCrd[0] * _opt->noiseCrd[0];
                        else if (p.type == CRD_Y) qNeu[1] = _opt->noiseCrd[1] * _opt->noiseCrd[1];
                        else if (p.type == CRD_Z) qNeu[2] = _opt->noiseCrd[2] * _opt->noiseCrd[2];
                    }
                } else if (p.type == TROPO) qAdd = _opt->noiseTrp * _opt->noiseTrp;
                else if (p.type == IONO_L1) qAdd = _opt->noiseIon * _opt->noiseIon;
                else if (isCodeBiasParam(p.type)) qAdd = _opt->noiseCodeBias * _opt->noiseCodeBias;
                else if (isPhaseBiasParam(p.type)) qAdd = _opt->noisePhaseBias * _opt->noisePhaseBias;
                Qnew[i][i] += qAdd;
            }
        }
        const int ix = paramIndex(CRD_X);
        const int iy = paramIndex(CRD_Y);
        const int iz = paramIndex(CRD_Z);
        if (ix >= 0 && iy >= 0 && iz >= 0 && (firstCrdState || addCrdNoise)) {
            const Mat qCrdXyz = covNeuToXyz(_ellBanc, qNeu[0], qNeu[1], qNeu[2]);
            const int idx[3] = {ix, iy, iz};
            for (int r = 0; r < 3; ++r) {
                for (int c = 0; c < 3; ++c) {
                    if (firstCrdState) {
                        Qnew[idx[r]][idx[c]] = qCrdXyz[r][c];
                    } else {
                        Qnew[idx[r]][idx[c]] += qCrdXyz[r][c];
                    }
                }
            }
        }

        for (Param& p : _params) {
            if (isClockParam(p.type)) {
                p.value = 0.0;
                const int i = p.index - 1;
                if (i >= 0 && i < Qnew.size()) {
                    for (int j = 0; j < Qnew.size(); ++j) {
                        Qnew[i][j] = 0.0;
                        Qnew[j][i] = 0.0;
                    }
                    Qnew[i][i] = _opt->aprSigClk * _opt->aprSigClk;
                }
            }
        }

        if (firstCrd) {
            for (Param& p : _params) {
                if (p.type == CRD_X || p.type == CRD_Y || p.type == CRD_Z) {
                    p.value = 0.0;
                }
            }
        }

        sanitizeCovariance(Qnew);
        _Q = Qnew;
        rebuildParamIndexCache();
    }

    void predict(int iPhase, PppEpochData& epoData, bool usePhase) {
        if (iPhase != 0) {
            return;
        }

        const double maxSolGap = 60.0;
        bool firstCrd = false;
        if (!_lastTimeOK.valid) {
            firstCrd = true;
            _startTime = epoData.tt;
            reset();
            _baseXyz = _opt->hasApriori() ? Vec3{_opt->aprXyz[0], _opt->aprXyz[1], _opt->aprXyz[2]} : _xcBanc;
            _ellBanc = xyzToEll(_baseXyz);
        } else if (maxSolGap > 0.0 && diffSeconds(_time, _lastTimeOK) > maxSolGap) {
            // Restart coordinate seeding after a gap while retaining filter state.
            _startTime = epoData.tt;
        }

        bool setCrdNoiseToZero = false;
        if (_opt->seedingTime > 0.0 && diffSeconds(epoData.tt, _startTime) < _opt->seedingTime) {
            setCrdNoiseToZero = true;
        }
        updateAmbiguities(epoData, usePhase, firstCrd, setCrdNoiseToZero);
    }

    bool resetAmbiguity(const QString& prn, ObservationCombination lc, const PppSatData& sat) {
        const ParamType ambType = ambParamTypeForLc(lc);
        bool reset = false;
        for (Param& p : _params) {
            if (p.type != ambType || p.prn != prn || p.index <= 0) {
                continue;
            }
            const int idx = p.index - 1;
            if (idx < 0 || idx >= _Q.size()) {
                continue;
            }
            _solutionLogLines << QString("%1 RESET %2").arg(epochTimeLabel(), paramText(p));
            for (int jj = 0; jj < _Q.size(); ++jj) {
                _Q[jj][idx] = 0.0;
                _Q[idx][jj] = 0.0;
            }
            _Q[idx][idx] = _opt->aprSigAmb * _opt->aprSigAmb;
            p.x0 = ambiguitySeed(sat, lc);
            p.value = 0.0;
            p.lastObsTime = _time;
            if (!p.firstObsTime.valid) p.firstObsTime = _time;
            p.ambEleSat = sat.eleSat;
            p.ambNumEpo = 0;
            resetArSatHistory(prn);
            reset = true;
        }
        return reset;
    }

    bool resetAllAmbiguities(const QString& prn, const PppSatData& sat) {
        bool reset = false;
        reset = resetAmbiguity(prn, ObservationCombination::L1, sat) || reset;
        reset = resetAmbiguity(prn, ObservationCombination::L2, sat) || reset;
        reset = resetAmbiguity(prn, ObservationCombination::LIf, sat) || reset;
        return reset;
    }

    void detectCycleSlips(PppEpochData& epoData) {
        for (auto it = epoData.satData.begin(); it != epoData.satData.end(); ++it) {
            PppSatData& sat = it.value();
            bool slip = sat.slipFlag;
            if (sat.slipFlag) {
                _solutionLogLines << QString("%1 Cycle slip set (obs) %2")
                    .arg(epochTimeLabel(), sat.prn);
            }

            bool lockChanged = false;
            if (sat.lock1 >= 0) {
                lockChanged = _slipLock1.contains(sat.prn) && sat.lock1 < _slipLock1[sat.prn];
                _slipLock1[sat.prn] = sat.lock1;
            }
            if (sat.lock2 >= 0) {
                lockChanged = (_slipLock2.contains(sat.prn) && sat.lock2 < _slipLock2[sat.prn]) ||
                              lockChanged;
                _slipLock2[sat.prn] = sat.lock2;
            }
            if (lockChanged) {
                _solutionLogLines << QString("%1 Cycle slip set (obsSlipCounter) %2")
                    .arg(epochTimeLabel(), sat.prn);
                slip = true;
            }

            bool discontinuityChanged = false;
            if (sat.disc1 >= 0) {
                discontinuityChanged = _slipDisc1.contains(sat.prn) && _slipDisc1[sat.prn] != sat.disc1;
                _slipDisc1[sat.prn] = sat.disc1;
            }
            if (sat.disc2 >= 0) {
                discontinuityChanged = (_slipDisc2.contains(sat.prn) && _slipDisc2[sat.prn] != sat.disc2) ||
                                       discontinuityChanged;
                _slipDisc2[sat.prn] = sat.disc2;
            }
            if (discontinuityChanged) {
                _solutionLogLines << QString("%1 Cycle slip set (biasJumpCounter) %2")
                    .arg(epochTimeLabel(), sat.prn);
                slip = true;
            }

            if (slip) {
                resetAllAmbiguities(sat.prn, sat);
            }
        }
    }

    bool applyKalmanUpdate(const Mat& H, const Vec& ll, const Vec& rDiag, Vec& residuals,
                        Vec* prefitInnovations = nullptr,
                        Vec* innovationSigmas = nullptr) {
        const int nPar = _params.size();
        const int nObs = ll.size();
        if (nPar == 0 || nObs == 0 || H.size() != nObs) {
            return false;
        }

        if (_Q.size() != nPar) {
            return false;
        }

        Mat cholLower;
        if (!choleskyDecompose(_Q, cholLower)) {
            return false;
        }
        const Mat ss = transpose(cholLower);
        const Mat sa = multiply(ss, transpose(H));

        const int nAll = nObs + nPar;
        Mat srf = zeros(nAll, nAll);
        for (int i = 0; i < nObs; ++i) {
            if (i >= rDiag.size() || !std::isfinite(rDiag[i]) || rDiag[i] <= 0.0) {
                return false;
            }
            srf[i][i] = std::sqrt(rDiag[i]);
        }
        for (int i = 0; i < nPar; ++i) {
            for (int j = 0; j < nObs; ++j) {
                srf[nObs + i][j] = sa[i][j];
            }
            for (int j = 0; j < nPar; ++j) {
                srf[nObs + i][nObs + j] = ss[i][j];
            }
        }

        Mat uu;
        if (!qrzUpper(srf, uu)) {
            return false;
        }

        Mat ssPost = zeros(nPar, nPar);
        Mat sh = zeros(nObs, nObs);
        Mat yy = zeros(nObs, nPar);
        for (int i = 0; i < nPar; ++i) {
            for (int j = 0; j < nPar; ++j) {
                ssPost[i][j] = uu[nObs + i][nObs + j];
            }
        }
        for (int i = 0; i < nObs; ++i) {
            for (int j = 0; j < nObs; ++j) {
                sh[i][j] = uu[i][j];
            }
            for (int j = 0; j < nPar; ++j) {
                yy[i][j] = uu[i][nObs + j];
            }
        }

        Mat shInv;
        if (!invertUpperTriangular(sh, shInv)) {
            return false;
        }
        const Mat kt = multiply(shInv, yy);

        Vec x(nPar, 0.0);
        for (const Param& par : _params) {
            const int idx = par.index - 1;
            if (idx >= 0 && idx < nPar) {
                x[idx] = par.value;
            }
        }

        Vec innovation = ll;
        const Vec hx = multiply(H, x);
        for (int i = 0; i < innovation.size() && i < hx.size(); ++i) {
            innovation[i] -= hx[i];
        }
        if (prefitInnovations) {
            *prefitInnovations = innovation;
        }
        if (innovationSigmas) {
            innovationSigmas->fill(0.0, nObs);
            for (int i = 0; i < nObs; ++i) {
                double variance = rDiag[i];
                for (int j = 0; j < nPar; ++j) {
                    for (int k = 0; k < nPar; ++k) {
                        variance += H[i][j] * _Q[j][k] * H[i][k];
                    }
                }
                if (!std::isfinite(variance) || variance <= 0.0) {
                    return false;
                }
                (*innovationSigmas)[i] = std::sqrt(variance);
            }
        }

        const Vec dx = multiply(transpose(kt), innovation);
        for (double d : dx) {
            if (!std::isfinite(d)) {
                return false;
            }
        }
        for (Param& par : _params) {
            const int idx = par.index - 1;
            if (idx >= 0 && idx < dx.size()) {
                par.value += dx[idx];
                x[idx] = par.value;
            }
        }

        Mat Qnew = multiply(transpose(ssPost), ssPost);
        sanitizeCovariance(Qnew);
        _Q = Qnew;

        residuals = ll;
        const Vec hxPost = multiply(H, x);
        for (int i = 0; i < residuals.size() && i < ll.size(); ++i) {
            residuals[i] -= hxPost[i];
        }
        return true;
    }

    bool kalmanUpdateSystem(int iPhase, const QChar& system,
                            PppEpochData& epoData, bool usePhase,
                            bool keepStateOnOutlier,
                            OutlierInfo& outlier) {
        const QVector<Param> paramsBefore = _params;
        const Mat qBefore = _Q;

        QVector<ObsRow> obsRows;
        obsRows.reserve(epoData.satData.size() * 2);
        int nSat = 0;
        const QVector<ObservationCombination> lcs = combinationsForSystem(*_opt, system);
        const bool hasGimLc = lcs.contains(ObservationCombination::Gim);
        int usedRealLcs = 0;
        for (ObservationCombination lc : lcs) {
            if (lc != ObservationCombination::Gim) {
                ++usedRealLcs;
            }
        }

        for (auto it = epoData.satData.begin(); it != epoData.satData.end(); ++it) {
            PppSatData& sat = it.value();
            if (sat.system() != system) {
                continue;
            }
            if (sat.outlier) {
                continue;
            }
            bool satUsable = usedRealLcs > 0;
            for (ObservationCombination lc : lcs) {
                if (lc == ObservationCombination::Gim) {
                    continue;
                }
                if (combinationContainsPhase(lc) && iPhase != 1) {
                    continue;
                }
                if (!obsAvailable(sat, lc)) {
                    satUsable = false;
                    break;
                }
            }
            if (!satUsable) {
                continue;
            }
            ++nSat;
            for (ObservationCombination lc : lcs) {
                if (lc == ObservationCombination::Gim) {
                    continue;
                }
                if (combinationContainsPhase(lc) && (iPhase != 1 || !usePhase)) {
                    continue;
                }
                const ObsKind kind = obsKindForLc(lc);
                obsRows.push_back({it.key(), kind, lc, obsValue(sat, lc), obsSigma(sat, kind)});
            }
        }
        if (_opt->pseudoObsIono && hasGimLc) {
            for (auto it = epoData.satData.begin(); it != epoData.satData.end(); ++it) {
                PppSatData& sat = it.value();
                if (sat.system() != system || sat.outlier || !obsAvailable(sat, ObservationCombination::Gim)) {
                    continue;
                }
                obsRows.push_back({it.key(), GIM, ObservationCombination::Gim, obsValue(sat, ObservationCombination::Gim), obsSigma(sat, GIM)});
            }
        }

        if (nSat == 0) {
            _params = paramsBefore;
            _Q = qBefore;
            outlier.prn.clear();
            _failureReason = QString("filter system %1 has no satellites usable for configured LC").arg(system);
            return false;
        }

        const int nObs = obsRows.size();
        if (nObs == 0) {
            _params = paramsBefore;
            _Q = qBefore;
            outlier.prn.clear();
            _failureReason = QString("filter system %1 has no observation rows for configured LC").arg(system);
            return false;
        }

        const int nPar = _params.size();
        Mat H = zeros(nObs, nPar);
        Vec v(nObs, 0.0);
        Vec Rdiag(nObs, 0.0);

        int row = 0;
        QVector<ObsRow> usedRows;
        usedRows.reserve(nObs);
        QVector<QString> usedPrns;
        QVector<ObsKind> usedKinds;
        usedPrns.reserve(nObs);
        usedKinds.reserve(nObs);
        for (const ObsRow& obs : obsRows) {
            auto itSat = epoData.satData.find(obs.prn);
            if (itSat == epoData.satData.end()) {
                continue;
            }
            PppSatData& sat = itSat.value();
            const double sigma = qMax(1e-6, obs.sigma);
            const double cmp = cmpValue(sat, obs.kind);
            if (!std::isfinite(obs.meas) || !std::isfinite(cmp)) {
                continue;
            }
            Rdiag[row] = sigma * sigma;
            if (!std::isfinite(Rdiag[row]) || Rdiag[row] <= 0.0) {
                continue;
            }
            double x0Dot = 0.0;
            for (int i = 0; i < nPar; ++i) {
                H[row][i] = partial(_params[i], sat, obs.kind);
                if (!std::isfinite(H[row][i])) {
                    H[row][i] = 0.0;
                }
                x0Dot += _params[i].x0 * H[row][i];
            }
            v[row] = obs.meas - cmp - x0Dot;
            if (!std::isfinite(v[row])) {
                continue;
            }
            usedRows.push_back(obs);
            usedPrns.push_back(obs.prn);
            usedKinds.push_back(obs.kind);
            ++row;
        }
        if (row == 0) {
            _params = paramsBefore;
            _Q = qBefore;
            outlier.prn.clear();
            _failureReason = QString("filter system %1 has no finite observed-computed rows").arg(system);
            return false;
        }
        if (row != nObs) {
            Mat H2 = zeros(row, nPar);
            Vec v2(row, 0.0);
            Vec r2(row, 0.0);
            for (int i = 0; i < row; ++i) {
                v2[i] = v[i];
                r2[i] = Rdiag[i];
                for (int j = 0; j < nPar; ++j) {
                    H2[i][j] = H[i][j];
                }
            }
            H = H2;
            v = v2;
            Rdiag = r2;
        }

        Vec residuals;
        Vec prefitInnovations;
        Vec innovationSigmas;
        appendPrefitDiag(system, usedPrns, usedKinds, v);
        if (!applyKalmanUpdate(H, v, Rdiag, residuals,
                            &prefitInnovations, &innovationSigmas)) {
            _params = paramsBefore;
            _Q = qBefore;
            outlier.prn.clear();
            _failureReason = QString("filter system %1 Kalman update failed: nObs=%2 nPar=%3")
                                 .arg(system)
                                 .arg(H.size())
                                 .arg(_params.size());
            return false;
        }
        appendPostfitDiag(system, usedPrns, usedKinds, residuals);

        findObservationOutlier(iPhase, usedRows, residuals, epoData, outlier);
        if (!outlier.prn.isEmpty()) {
            appendObservationResiduals(usedRows, prefitInnovations, residuals,
                                       innovationSigmas, epoData, false,
                                       QStringLiteral("rejected"), outlier.prn,
                                       static_cast<int>(outlier.kind));
            if (!keepStateOnOutlier) {
                _params = paramsBefore;
                _Q = qBefore;
            }
            return false;
        }
        appendObservationResiduals(usedRows, prefitInnovations, residuals,
                                   innovationSigmas, epoData, true, QString());
        for (ObservationCombination lc : lcs) {
            for (int i = 0; i < residuals.size() && i < usedRows.size(); ++i) {
                if (usedRows[i].lc != lc) {
                    continue;
                }
                _solutionLogLines << QString("%1 RES %2 %3 %4")
                    .arg(epochTimeLabel(),
                         combinationLabel(lc).leftJustified(3, QLatin1Char(' ')),
                         usedRows[i].prn,
                         QString::number(residuals[i], 'f', 4));
            }
        }
        return true;
    }

    bool kalmanUpdateAllSystems(int iPhase,
                                PppEpochData& epoData,
                                bool usePhase,
                                bool keepStateOnOutlier,
                                OutlierInfo& outlier) {
        const QVector<Param> paramsBefore = _params;
        const Mat qBefore = _Q;
        const QVector<QChar> systemOrder = {QLatin1Char('G'), QLatin1Char('R'),
                                            QLatin1Char('E'), QLatin1Char('C')};

        QVector<ObsRow> obsRows;
        obsRows.reserve(epoData.satData.size() * 2);
        int nSat = 0;

        for (auto it = epoData.satData.begin(); it != epoData.satData.end(); ++it) {
            PppSatData& sat = it.value();
            const QChar sys = sat.system();
            if (!_opt->useSystem(sys) || sat.outlier) {
                continue;
            }

            const QVector<ObservationCombination> lcs = combinationsForSystem(*_opt, sys);
            bool useObs = false;
            for (ObservationCombination lc : lcs) {
                if (lc == ObservationCombination::Gim) {
                    continue;
                }
                if (combinationContainsPhase(lc)) {
                    if (iPhase == 1 && usePhase) {
                        useObs = true;
                    }
                } else if (iPhase == 0) {
                    useObs = true;
                }
            }
            if (!useObs) {
                sat.obsIndex = 0;
                continue;
            }

            bool satUsable = true;
            QVector<ObsRow> satRows;
            for (ObservationCombination lc : lcs) {
                if (lc == ObservationCombination::Gim) {
                    continue;
                }
                const bool isPhase = combinationContainsPhase(lc);
                if ((iPhase == 0 && isPhase) || (iPhase == 1 && !isPhase)) {
                    continue;
                }
                if (!obsAvailable(sat, lc)) {
                    satUsable = false;
                    break;
                }
                const ObsKind kind = obsKindForLc(lc);
                satRows.push_back({it.key(), kind, lc, obsValue(sat, lc), obsSigma(sat, kind)});
            }
            if (!satUsable || satRows.isEmpty()) {
                sat.obsIndex = 0;
                continue;
            }
            ++nSat;
            for (const ObsRow& row : satRows) {
                obsRows.push_back(row);
            }
        }

        if (_opt->pseudoObsIono) {
            for (auto it = epoData.satData.begin(); it != epoData.satData.end(); ++it) {
                PppSatData& sat = it.value();
                if (!_opt->useSystem(sat.system()) || sat.outlier ||
                    !combinationsForSystem(*_opt, sat.system()).contains(ObservationCombination::Gim) ||
                    !obsAvailable(sat, ObservationCombination::Gim)) {
                    continue;
                }
                obsRows.push_back({it.key(), GIM, ObservationCombination::Gim, obsValue(sat, ObservationCombination::Gim), obsSigma(sat, GIM)});
            }
        }

        if (nSat < _opt->minObs || obsRows.isEmpty()) {
            _params = paramsBefore;
            _Q = qBefore;
            outlier.prn.clear();
            _failureReason = QString("filter all systems has insufficient observations: sats=%1 rows=%2 minObs=%3")
                                 .arg(nSat)
                                 .arg(obsRows.size())
                                 .arg(_opt->minObs);
            return false;
        }

        const int nPar = _params.size();
        Mat H = zeros(obsRows.size(), nPar);
        Vec v(obsRows.size(), 0.0);
        Vec Rdiag(obsRows.size(), 0.0);

        int row = 0;
        QVector<ObsRow> usedRows;
        QVector<QString> usedPrns;
        QVector<ObsKind> usedKinds;
        usedRows.reserve(obsRows.size());
        usedPrns.reserve(obsRows.size());
        usedKinds.reserve(obsRows.size());

        for (const ObsRow& obs : obsRows) {
            auto itSat = epoData.satData.find(obs.prn);
            if (itSat == epoData.satData.end()) {
                continue;
            }
            PppSatData& sat = itSat.value();
            const double sigma = qMax(1e-6, obs.sigma);
            const double cmp = cmpValue(sat, obs.kind);
            if (!std::isfinite(obs.meas) || !std::isfinite(cmp)) {
                continue;
            }
            Rdiag[row] = sigma * sigma;
            if (!std::isfinite(Rdiag[row]) || Rdiag[row] <= 0.0) {
                continue;
            }
            double x0Dot = 0.0;
            for (int i = 0; i < nPar; ++i) {
                H[row][i] = partial(_params[i], sat, obs.kind);
                if (!std::isfinite(H[row][i])) {
                    H[row][i] = 0.0;
                }
                x0Dot += _params[i].x0 * H[row][i];
            }
            v[row] = obs.meas - cmp - x0Dot;
            if (!std::isfinite(v[row])) {
                continue;
            }
            sat.obsIndex = row + 1;
            usedRows.push_back(obs);
            usedPrns.push_back(obs.prn);
            usedKinds.push_back(obs.kind);
            ++row;
        }

        if (row == 0) {
            _params = paramsBefore;
            _Q = qBefore;
            outlier.prn.clear();
            _failureReason = QStringLiteral("filter all systems has no finite observed-computed rows");
            return false;
        }
        if (row != obsRows.size()) {
            Mat H2 = zeros(row, nPar);
            Vec v2(row, 0.0);
            Vec r2(row, 0.0);
            for (int i = 0; i < row; ++i) {
                v2[i] = v[i];
                r2[i] = Rdiag[i];
                for (int j = 0; j < nPar; ++j) {
                    H2[i][j] = H[i][j];
                }
            }
            H = H2;
            v = v2;
            Rdiag = r2;
        }

        Vec residuals;
        Vec prefitInnovations;
        Vec innovationSigmas;
        appendAllSystemPrefitDiag(usedPrns, usedKinds, v);
        if (!applyKalmanUpdate(H, v, Rdiag, residuals,
                            &prefitInnovations, &innovationSigmas)) {
            _params = paramsBefore;
            _Q = qBefore;
            outlier.prn.clear();
            _failureReason = QString("filter all systems Kalman update failed: nObs=%1 nPar=%2")
                                 .arg(H.size())
                                 .arg(_params.size());
            return false;
        }
        appendAllSystemPostfitDiag(usedPrns, usedKinds, residuals);

        findObservationOutlier(iPhase, usedRows, residuals, epoData, outlier);
        if (!outlier.prn.isEmpty()) {
            appendObservationResiduals(usedRows, prefitInnovations, residuals,
                                       innovationSigmas, epoData, false,
                                       QStringLiteral("rejected"), outlier.prn,
                                       static_cast<int>(outlier.kind));
            if (!keepStateOnOutlier) {
                _params = paramsBefore;
                _Q = qBefore;
            }
            return false;
        }
        appendObservationResiduals(usedRows, prefitInnovations, residuals,
                                   innovationSigmas, epoData, true, QString());

        for (ObservationCombination lc : {ObservationCombination::C1, ObservationCombination::C2, ObservationCombination::CIf, ObservationCombination::L1, ObservationCombination::L2, ObservationCombination::LIf, ObservationCombination::Gim}) {
            for (int i = 0; i < residuals.size() && i < usedRows.size(); ++i) {
                if (usedRows[i].lc != lc) {
                    continue;
                }
                _solutionLogLines << QString("%1 RES %2 %3 %4")
                    .arg(epochTimeLabel(),
                         combinationLabel(lc).leftJustified(3, QLatin1Char(' ')),
                         usedRows[i].prn,
                         QString::number(residuals[i], 'f', 4));
            }
        }
        return true;
    }

    void cmpDop(PppEpochData& epoData) {
        _numSat = 0;
        for (auto it = epoData.satData.constBegin(); it != epoData.satData.constEnd(); ++it) {
            if (!it.value().outlier) {
                ++_numSat;
            }
        }
        _hDop = 0.0;
        if (_numSat < 4) return;
        Mat A = zeros(_numSat, 4);
        int row = 0;
        for (auto it = epoData.satData.constBegin(); it != epoData.satData.constEnd(); ++it) {
            if (it.value().outlier) {
                continue;
            }
            for (int i = 0; i < 4; ++i) {
                A[row][i] = partial(_params[i], it.value(), CODE1);
            }
            ++row;
        }
        Mat Q;
        if (!invert(multiply(transpose(A), A), Q)) return;
        const double sinLat = std::sin(_ellBanc.lat);
        const double cosLat = std::cos(_ellBanc.lat);
        const double sinLon = std::sin(_ellBanc.lon);
        const double cosLon = std::cos(_ellBanc.lon);
        const double r[3][3] = {
            {-sinLat * cosLon, -sinLat * sinLon,  cosLat},
            {-sinLon,           cosLon,           0.0},
            { cosLat * cosLon,  cosLat * sinLon,  sinLat}
        };
        double qNeu[3][3] = {{0.0}};
        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) {
                for (int k = 0; k < 3; ++k) {
                    for (int l = 0; l < 3; ++l) {
                        qNeu[i][j] += r[i][k] * Q[k][l] * r[j][l];
                    }
                }
            }
        }
        _hDop = std::sqrt(qMax(0.0, qNeu[0][0] + qNeu[1][1]));
    }

    bool update_p(PppEpochData& epoData) {
        for (int iter = 1; iter <= 2; ++iter) {
            if (!cmpBancroft(epoData, iter > 1, iter == 2)) {
                _failureReason = QString("Bancroft failed at iteration %1 with satellites=%2")
                                     .arg(iter)
                                     .arg(static_cast<int>(epoData.satData.size()));
                return false;
            }
            if (!cmpModelForBancroft(epoData)) {
                const QString modelReason = _failureReason;
                _failureReason = QString("model setup failed at iteration %1: satellites below minObs after elevation/model screening (%2/%3)%4")
                                     .arg(iter)
                                     .arg(static_cast<int>(epoData.satData.size()))
                                     .arg(_opt->minObs)
                                     .arg(modelReason.isEmpty() ? QString() : QStringLiteral("; ") + modelReason);
                return false;
            }
        }

        bool usePhase = false;
        for (auto it = epoData.satData.constBegin(); it != epoData.satData.constEnd(); ++it) {
            const QVector<ObservationCombination> lcs = combinationsForSystem(*_opt, it.value().system());
            for (ObservationCombination lc : lcs) {
                if (combinationContainsPhase(lc) && obsAvailable(it.value(), lc)) {
                    usePhase = true;
                    break;
                }
            }
            if (usePhase) {
                break;
            }
        }

        const int lastPhase = usePhase ? 1 : 0;
        const QVector<QChar> systemOrder = {QLatin1Char('G'), QLatin1Char('R'),
                                            QLatin1Char('E'), QLatin1Char('C')};
        for (const QChar& sys : systemOrder) {
            if (!_opt->useSystem(sys)) {
                continue;
            }
            int satCount = 0;
            for (auto it = epoData.satData.constBegin(); it != epoData.satData.constEnd(); ++it) {
                if (it.value().system() == sys) {
                    ++satCount;
                }
            }
            if (satCount > 0) {
                _solutionLogLines << QString("%1 SATNUM %2 %3")
                    .arg(epochTimeLabel(),
                         QString(sys),
                         QString::number(satCount).rightJustified(2, QLatin1Char(' ')));
            }
        }
        for (int iPhase = 0; iPhase <= lastPhase; ++iPhase) {
            predict(iPhase, epoData, usePhase);
            if (iPhase == 0) {
                detectCycleSlips(epoData);
            }

            for (const QChar& sys : systemOrder) {
                if (!_opt->useSystem(sys)) {
                    continue;
                }
                int satCount = 0;
                for (auto it = epoData.satData.constBegin(); it != epoData.satData.constEnd(); ++it) {
                    if (it.value().system() == sys && !it.value().outlier) {
                        ++satCount;
                    }
                }
                if (satCount == 0) {
                    continue;
                }

                QVector<Param> paramWork = _params;
                Mat qWork = _Q;
                const int maxOutlier = qMax(1, satCount * qMax(1, combinationsForSystem(*_opt, sys).size()));
                bool systemOk = false;
                for (int iter = 0; iter < maxOutlier; ++iter) {
                    _params = paramWork;
                    _Q = qWork;
                    rebuildParamIndexCache();

                    OutlierInfo outlier;
                    const bool lastOutlierIteration = (iter + 1 >= maxOutlier);
                    const int diagSizeBefore = _diagLines.size();
                    if (kalmanUpdateSystem(iPhase, sys, epoData, usePhase, lastOutlierIteration, outlier)) {
                        systemOk = true;
                        break;
                    }
                    if (outlier.prn.isEmpty() || !epoData.satData.contains(outlier.prn)) {
                        return false;
                    }
                    while (_diagLines.size() > diagSizeBefore) {
                        _diagLines.removeLast();
                    }
                    _solutionLogLines << QString("%1 Outlier %2 %3 %4")
                        .arg(epochTimeLabel(),
                             obsKindText(outlier.kind),
                             outlier.prn,
                             QString::number(outlier.residual, 'f', 4));
                    if (lastOutlierIteration) {
                        _failureReason = QString("filter system %1 exceeded outlier loop").arg(sys);
                        return false;
                    }

                    const bool phaseOutlier = (outlier.kind == PHASE1 ||
                                               outlier.kind == PHASE2 ||
                                               outlier.kind == PHASE_IF);
                    if (phaseOutlier) {
                        ParamType ambType = AMB_IF;
                        if (outlier.kind == PHASE1) ambType = AMB_L1;
                        else if (outlier.kind == PHASE2) ambType = AMB_L2;
                        bool ambReset = false;
                        PppSatData sat = epoData.satData.value(outlier.prn);
                        for (Param& p : paramWork) {
                            if (p.type == ambType && p.prn == outlier.prn && p.index > 0) {
                                const int idx = p.index - 1;
                                for (int jj = 0; jj < qWork.size(); ++jj) {
                                    qWork[jj][idx] = 0.0;
                                    qWork[idx][jj] = 0.0;
                                }
                                qWork[idx][idx] = _opt->aprSigAmb * _opt->aprSigAmb;
                                if (outlier.kind == PHASE_IF && obsAvailable(sat, ObservationCombination::LIf)) {
                                    p.x0 = ambiguitySeed(sat, ObservationCombination::LIf);
                                    p.value = 0.0;
                                } else if (outlier.kind == PHASE1 && obsAvailable(sat, ObservationCombination::L1)) {
                                    p.x0 = ambiguitySeed(sat, ObservationCombination::L1);
                                    p.value = 0.0;
                                } else if (outlier.kind == PHASE2 && obsAvailable(sat, ObservationCombination::L2)) {
                                    p.x0 = ambiguitySeed(sat, ObservationCombination::L2);
                                    p.value = 0.0;
                                } else {
                                    p.value = 0.0;
                                }
                                p.ambNumEpo = 0;
                                ambReset = true;
                                _solutionLogLines << QString("%1 RESET %2").arg(epochTimeLabel(), paramText(p));
                            }
                        }
                        if (ambReset) {
                            continue;
                        }
                    }

                    epoData.satData[outlier.prn].outlier = true;
                }
                if (!systemOk) {
                    return false;
                }
            }
        }

        // A valid PPP epoch must still have the configured minimum number of
        // satellites after standard residual/outlier screening.  The input
        // stage checks this before filtering, but each system update may mark
        // additional satellites as outliers.  Without this second check a
        // five-satellite epoch could be published with only two or three
        // satellites, producing a rank-deficient, non-PPP solution.
        int finalSatCount = 0;
        for (auto it = epoData.satData.constBegin(); it != epoData.satData.constEnd(); ++it) {
            if (!it.value().outlier) {
                ++finalSatCount;
            }
        }
        if (finalSatCount < _opt->minObs) {
            _failureReason = QString("filter left insufficient satellites after screening: sats=%1 minObs=%2")
                                 .arg(finalSatCount)
                                 .arg(_opt->minObs);
            return false;
        }

        if (_params.isEmpty()) {
            _failureReason = QStringLiteral("filter produced no parameters");
            return false;
        }

        tryAmbiguityFix(epoData);
        cmpDop(epoData);
        return true;
    }
};

namespace {

bool computeSatState(const PppTime& tt,
                     double prange,
                     const GnssEphemeris& eph,
                     const SsrOrbitCorrection* orbitCorr,
                     const SsrClockCorrection* clockCorr,
                     Vec3& satPos,
                     Vec3& satVel,
                     double& clkMeters) {
    if (prange == 0.0) return false;

    double clkSat = 0.0;
    for (int ii = 0; ii < 10; ++ii) {
        double dt = prange / GnssUtils::kC + clkSat;
        int week = tt.gpsWeek;
        double sow = tt.gpsSow - dt;
        while (sow < 0.0) { sow += 604800.0; --week; }
        while (sow > 604800.0) { sow -= 604800.0; ++week; }

        GnssEphemeris::State state;
        if (!eph.computeState(week, sow, state)) return false;

        GnssEphemeris::State corrected = state;
        applySsrOrbitClock(state, orbitCorr, clockCorr, week, sow, corrected);

        if (std::fabs(corrected.clockSec - clkSat) * GnssUtils::kC < 1e-4) {
            satPos = {corrected.position[0], corrected.position[1], corrected.position[2]};
            satVel = {corrected.velocity[0], corrected.velocity[1], corrected.velocity[2]};
            clkMeters = corrected.clockSec * GnssUtils::kC;
            return true;
        }
        clkSat = corrected.clockSec;
    }
    return false;
}

double wavelengthForPair(QChar system, QChar band, int gloChannel) {
    const QString type2ch = QString(band) + QLatin1Char('C');
    return signalWavelengthMeters(system, type2ch, gloChannel);
}

QDateTime dateTimeFromYmdHms(int year, int month, int day, int hour, int minute, double seconds) {
    const int secInt = static_cast<int>(std::floor(seconds));
    int msec = static_cast<int>(std::round((seconds - secInt) * 1000.0));
    int sec = secInt;
    if (msec >= 1000) {
        msec -= 1000;
        ++sec;
    }
    return QDateTime(QDate(year, month, day), QTime(hour, minute, sec, msec), Qt::UTC);
}

QVector<PppVtecModel> readVtecFile(const QString& path) {
    QVector<PppVtecModel> models;
    if (path.trimmed().isEmpty()) {
        return models;
    }

    QFile file(path);
    if (!file.open(QIODevice::ReadOnly | QIODevice::Text)) {
        return models;
    }

    QTextStream in(&file);
    QString marker;
    while (!in.atEnd()) {
        in >> marker;
        if (marker.isEmpty()) {
            break;
        }
        if (marker != QLatin1String(">")) {
            continue;
        }

        QString type;
        int year = 0;
        int month = 0;
        int day = 0;
        int hour = 0;
        int minute = 0;
        double second = 0.0;
        int updateInt = -1;
        int numLayers = 0;
        QString stationId;
        in >> type >> year >> month >> day >> hour >> minute >> second
           >> updateInt >> numLayers >> stationId;
        if (type != QLatin1String("VTEC") || in.status() != QTextStream::Ok || numLayers <= 0) {
            continue;
        }

        PppVtecModel model;
        model.utc = dateTimeFromYmdHms(year, month, day, hour, minute, second);
        if (!model.utc.isValid()) {
            continue;
        }
        GnssUtils::gpsWeekFromDateTime(model.utc, model.gpsWeek, model.gpsSow);
        model.updateIntervalIndex = updateInt;
        model.stationId = stationId;
        model.layers.reserve(numLayers);

        bool ok = true;
        for (int iLayer = 0; iLayer < numLayers; ++iLayer) {
            int dummy = 0;
            int maxDeg = -1;
            int maxOrd = -1;
            double height = 0.0;
            in >> dummy >> maxDeg >> maxOrd >> height;
            if (in.status() != QTextStream::Ok || maxDeg < 0 || maxOrd < 0) {
                ok = false;
                break;
            }

            PppVtecLayer layer;
            layer.heightMeters = height;
            layer.cosine = QVector<QVector<double>>(maxDeg + 1, QVector<double>(maxOrd + 1, 0.0));
            layer.sine = QVector<QVector<double>>(maxDeg + 1, QVector<double>(maxOrd + 1, 0.0));
            for (int iDeg = 0; iDeg <= maxDeg && ok; ++iDeg) {
                for (int iOrd = 0; iOrd <= maxOrd; ++iOrd) {
                    in >> layer.cosine[iDeg][iOrd];
                    if (in.status() != QTextStream::Ok) {
                        ok = false;
                        break;
                    }
                }
            }
            for (int iDeg = 0; iDeg <= maxDeg && ok; ++iDeg) {
                for (int iOrd = 0; iOrd <= maxOrd; ++iOrd) {
                    in >> layer.sine[iDeg][iOrd];
                    if (in.status() != QTextStream::Ok) {
                        ok = false;
                        break;
                    }
                }
            }
            if (!ok) {
                break;
            }
            model.layers.push_back(layer);
        }

        if (ok && model.isValid()) {
            models.push_back(model);
        }
    }

    return models;
}

} // namespace

PppEngine::PppEngine() = default;

PppEngine::~PppEngine() = default;

void PppEngine::setOptions(const PppOptions& options) {
    _options = options;
    if (_options.signalPriorities.trimmed().isEmpty()) {
        _options.signalPriorities = QStringLiteral("G:12&CWPSLX R:12&CP E:1&CBX E:5&QIX C:26&IQX");
    }
    if (_loadedVtecFile != _options.vtecFile) {
        _vtecModelsByTime.clear();
        _latestVtecModel = PppVtecModel();
        _activeVtecModel = PppVtecModel();
        _loadedVtecFile = _options.vtecFile;
        const QVector<PppVtecModel> models = readVtecFile(_options.vtecFile);
        for (const PppVtecModel& model : models) {
            _vtecModelsByTime[gpsMsKey(model.gpsWeek, model.gpsSow)] = model;
            _latestVtecModel = model;
        }
    }
    _filters.clear();
    _sourceOptions.clear();
    _pendingMsmBySystem.clear();
    _pendingMsmByEpoch.clear();
    _lastProcessedEpochPerSystem.clear();
    _lastClosedEpochPerSource.clear();
    _lastOutputEpochPerSource.clear();
    _lastOrbitIodBySourceSat.clear();
    _broadcastStateQueues.clear();
    _preciseStates.clear();
    _orbitCorrectionsByIod.clear();
    _clockCorrectionsByIod.clear();
    _lastAcceptedSatCountBySource.clear();
    _tropospherePriorsBySource.clear();
    _biasMatchDiagKeys.clear();
    _lastFailureReason.clear();
}

PppOptions& PppEngine::options() {
    return _options;
}

void PppEngine::setStreamSelection(const PppStreamSelection& selection) {
    _selection = selection;
    resetProcessingState();
}

void PppEngine::resetProcessingState() {
    _pendingMsmBySystem.clear();
    _pendingMsmByEpoch.clear();
    _lastProcessedEpochPerSystem.clear();
    _lastClosedEpochPerSource.clear();
    _lastOutputEpochPerSource.clear();
    _filters.clear();
    _preciseStates.clear();
    _sourceOptions.clear();
    _tropospherePriorsBySource.clear();
    _lastAcceptedSatCountBySource.clear();
    _biasMatchDiagKeys.clear();
    _lastFailureReason.clear();
}

void PppEngine::updateVtecModel(const PppVtecModel& vtec) {
    _latestVtecModel = vtec;
    if (_latestVtecModel.isValid() && !_latestVtecModel.utc.isValid()) {
        _latestVtecModel.utc = GnssUtils::dateTimeFromGpsWeek(_latestVtecModel.gpsWeek, _latestVtecModel.gpsSow);
    }
    if (_latestVtecModel.isValid() && _latestVtecModel.utc.isValid() &&
        (_latestVtecModel.gpsWeek == 0 && _latestVtecModel.gpsSow == 0.0)) {
        GnssUtils::gpsWeekFromDateTime(_latestVtecModel.utc, _latestVtecModel.gpsWeek, _latestVtecModel.gpsSow);
    }
    if (_latestVtecModel.isValid()) {
        _vtecModelsByTime[gpsMsKey(_latestVtecModel.gpsWeek, _latestVtecModel.gpsSow)] = _latestVtecModel;
        while (_vtecModelsByTime.size() > 4096 && _loadedVtecFile.isEmpty()) {
            _vtecModelsByTime.erase(_vtecModelsByTime.begin());
        }
    }
}

void PppEngine::clearVtecModel() {
    _activeVtecModel = PppVtecModel();
    _latestVtecModel = PppVtecModel();
    _vtecModelsByTime.clear();
    _loadedVtecFile.clear();
}

void PppEngine::updatePreciseState(const PppPreciseSatelliteState& state) {
    if (state.sat.isValid()) {
        _preciseStates[state.sat.key()] = state;
    }
}

void PppEngine::clearPreciseStates() {
    _preciseStates.clear();
}

void PppEngine::setTropospherePrior(const QString& sourceId, const PppTropospherePrior& prior) {
    const QString key = sourceId.trimmed();
    if (key.isEmpty()) {
        return;
    }
    // A PppFilter owns its prior. Deliberately do not replace an active filter:
    // doing so would change x0 without transforming the Kalman covariance.
    if (_filters.find(key) != _filters.end()) {
        return;
    }
    _tropospherePriorsBySource[key] = prior;
}

bool PppEngine::lastBancroftPosition(const QString& sourceId, std::array<double, 3>& xyz) const {
    const auto it = _filters.find(sourceId.trimmed());
    if (it == _filters.end() || !it->second || !it->second->hasBancroftPosition()) {
        return false;
    }
    xyz = it->second->bancroftPosition();
    return true;
}

void PppEngine::discardGfsSeedEpoch(const QString& sourceId, int gpsWeek, double gpsSow) {
    Q_UNUSED(gpsWeek);
    Q_UNUSED(gpsSow);
    const QString key = sourceId.trimmed();
    if (key.isEmpty()) return;
    _filters.erase(key);
    _sourceOptions.erase(key);
    _lastOutputEpochPerSource.remove(key);
    _lastAcceptedSatCountBySource.remove(key);
    _lastClosedEpochPerSource.remove(key);
    for (auto it = _lastProcessedEpochPerSystem.begin(); it != _lastProcessedEpochPerSystem.end();) {
        if (it.key().startsWith(key + QLatin1Char('|'))) it = _lastProcessedEpochPerSystem.erase(it);
        else ++it;
    }
}

void PppEngine::updateBroadcast(const RtcmBroadcastEphemeris& eph,
                                const std::shared_ptr<GnssEphemeris>& model,
                                const QString& sourceId,
                                const QDateTime& receivedUtc) {
    if (!eph.sat.isValid()) {
        return;
    }
    BroadcastState state;
    state.eph = eph;
    state.model = model;
    state.sourceId = sourceId;
    state.receivedUtc = receivedUtc;
    const QString satKey = eph.sat.key();
    auto attachCorrections = [&](const QString& corrSourceId) {
        const QString exactKey = sourceSatIodKey(corrSourceId, satKey, eph.iod);
        const QString noIodKey = sourceSatIodKey(corrSourceId, satKey, -1);
        auto orbitIt = _orbitCorrectionsByIod.constFind(exactKey);
        if (orbitIt == _orbitCorrectionsByIod.constEnd()) {
            orbitIt = _orbitCorrectionsByIod.constFind(noIodKey);
        }
        if (orbitIt != _orbitCorrectionsByIod.constEnd()) {
            state.hasOrbitCorrection = true;
            state.orbitCorrection = orbitIt.value();
        }
        auto clockIt = _clockCorrectionsByIod.constFind(exactKey);
        if (clockIt == _clockCorrectionsByIod.constEnd()) {
            clockIt = _clockCorrectionsByIod.constFind(noIodKey);
        }
        if (clockIt != _clockCorrectionsByIod.constEnd()) {
            state.hasClockCorrection = true;
            state.clockCorrection = clockIt.value();
        }
    };
    if (!_selection.orbitId.isEmpty()) {
        attachCorrections(_selection.orbitId);
    }
    if (!_selection.clockId.isEmpty() && _selection.clockId != _selection.orbitId) {
        attachCorrections(_selection.clockId);
    }
    if (!_options.strictCorrectionSource) {
        attachCorrections(sourceId);
    }
    _broadcastStates[eph.sat.key()] = state;
    QVector<BroadcastState>& queue = _broadcastStateQueues[satKey];
    for (auto it = queue.begin(); it != queue.end();) {
        if (it->sourceId == sourceId && it->eph.iod == eph.iod) {
            it = queue.erase(it);
        } else {
            ++it;
        }
    }
    queue.prepend(state);
    while (queue.size() > 8) {
        queue.removeLast();
    }
}

void PppEngine::beginBiasEpoch(const QString& sourceId, const RtcmSsrMessage& ssr) {
    QChar system = ssr.system;
    if (system.isNull() && !ssr.satellites.isEmpty()) {
        system = ssr.satellites.first().sat.system;
    }
    if (system.isNull()) {
        return;
    }

    if (ssr.kind == RtcmSsrCorrectionKind::CodeBias) {
        for (auto it = _codeBiases.begin(); it != _codeBiases.end();) {
            if (it.value().sourceId == sourceId && it.value().sat.system == system) {
                it = _codeBiases.erase(it);
            } else {
                ++it;
            }
        }
    } else if (ssr.kind == RtcmSsrCorrectionKind::PhaseBias) {
        for (auto it = _phaseBiases.begin(); it != _phaseBiases.end();) {
            if (it.value().sourceId == sourceId && it.value().sat.system == system) {
                it = _phaseBiases.erase(it);
            } else {
                ++it;
            }
        }
    }
}

void PppEngine::updateSsrMessage(const QString& sourceId, const RtcmSsrMessage& ssr) {
    if (ssr.satellites.isEmpty()) {
        return;
    }

    for (const RtcmSsrSatelliteData& sat : ssr.satellites) {
        if (!sat.sat.isValid()) continue;
        if (ssrOrbitClockOutOfRange(sat, ssr.kind)) continue;
        const QString key = sourceSatKey(sourceId, sat.sat.key());
        const QDateTime now = QDateTime::currentDateTimeUtc();

        if (ssr.kind == RtcmSsrCorrectionKind::Orbit || ssr.kind == RtcmSsrCorrectionKind::OrbitClock) {
            OrbitCorrectionState corr;
            corr.sat = sat.sat;
            corr.sourceId = sourceId;
            corr.iod = sat.iod;
            corr.updateIntervalIndex = ssr.updateIntervalIndex;
            corr.satRefDatum = ssr.satRefDatum;
            corr.epochTime = ssr.epochTime;
            corr.epochTimeIsGps = ssr.epochTimeIsGps;
            corr.messageType = ssr.messageType;
            corr.xr[0] = sat.deltaRadial;
            corr.xr[1] = sat.deltaAlongTrack;
            corr.xr[2] = sat.deltaCrossTrack;
            corr.dotXr[0] = sat.dotDeltaRadial;
            corr.dotXr[1] = sat.dotDeltaAlongTrack;
            corr.dotXr[2] = sat.dotDeltaCrossTrack;
            corr.receivedUtc = now;
            _orbitCorrections[key] = corr;
            _orbitCorrectionsByIod[sourceSatIodKey(sourceId, sat.sat.key(), corr.iod)] = corr;
            trimMapToSize(_orbitCorrectionsByIod, 4096);
            _lastOrbitIodBySourceSat[key] = corr.iod;
            const bool selectedOrbitSource =
                sourceId == _selection.orbitId ||
                (!_options.strictCorrectionSource &&
                 (sourceId == _selection.clockId || sourceId == _selection.broadcastId));
            auto bIt = _broadcastStates.find(sat.sat.key());
            if (selectedOrbitSource &&
                bIt != _broadcastStates.end() && bIt.value().model &&
                (bIt.value().eph.iod == corr.iod || corr.iod < 0)) {
                bIt.value().hasOrbitCorrection = true;
                bIt.value().orbitCorrection = corr;
            }
            if (selectedOrbitSource) {
                QVector<BroadcastState>& queue = _broadcastStateQueues[sat.sat.key()];
                for (BroadcastState& state : queue) {
                    if (state.model && (state.eph.iod == corr.iod || corr.iod < 0)) {
                        state.hasOrbitCorrection = true;
                        state.orbitCorrection = corr;
                    }
                }
            }
        }

        if (ssr.kind == RtcmSsrCorrectionKind::Clock || ssr.kind == RtcmSsrCorrectionKind::OrbitClock) {
            const bool combinedClock = (ssr.kind == RtcmSsrCorrectionKind::OrbitClock);
            const auto lastIodIt = _lastOrbitIodBySourceSat.constFind(key);
            if (!combinedClock && lastIodIt == _lastOrbitIodBySourceSat.constEnd()) {
                continue;
            }
            ClockCorrectionState corr;
            corr.sat = sat.sat;
            corr.sourceId = sourceId;
            corr.iod = combinedClock ? sat.iod : lastIodIt.value();
            corr.updateIntervalIndex = ssr.updateIntervalIndex;
            corr.epochTime = ssr.epochTime;
            corr.epochTimeIsGps = ssr.epochTimeIsGps;
            corr.messageType = ssr.messageType;
            corr.dClk = sat.deltaC0 / GnssUtils::kC;
            corr.dotDClk = sat.deltaC1 / GnssUtils::kC;
            corr.dotDotDClk = sat.deltaC2 / GnssUtils::kC;
            corr.receivedUtc = now;
            _clockCorrections[key] = corr;
            _clockCorrectionsByIod[sourceSatIodKey(sourceId, sat.sat.key(), corr.iod)] = corr;
            trimMapToSize(_clockCorrectionsByIod, 4096);
            const bool selectedClockSource =
                sourceId == _selection.clockId ||
                (!_options.strictCorrectionSource &&
                 (sourceId == _selection.orbitId || sourceId == _selection.broadcastId));
            auto bIt = _broadcastStates.find(sat.sat.key());
            if (selectedClockSource &&
                bIt != _broadcastStates.end() && bIt.value().model &&
                (bIt.value().eph.iod == corr.iod || corr.iod < 0)) {
                bIt.value().hasClockCorrection = true;
                bIt.value().clockCorrection = corr;
            }
            if (selectedClockSource) {
                QVector<BroadcastState>& queue = _broadcastStateQueues[sat.sat.key()];
                for (BroadcastState& state : queue) {
                    if (state.model && (state.eph.iod == corr.iod || corr.iod < 0)) {
                        state.hasClockCorrection = true;
                        state.clockCorrection = corr;
                    }
                }
            }
        }

        if (ssr.kind == RtcmSsrCorrectionKind::HighRateClock && sat.hasHighRateClock) {
            HighRateClockState corr;
            corr.sat = sat.sat;
            corr.sourceId = sourceId;
            corr.updateIntervalIndex = ssr.updateIntervalIndex;
            corr.epochTime = ssr.epochTime;
            corr.epochTimeIsGps = ssr.epochTimeIsGps;
            corr.messageType = ssr.messageType;
            corr.dClk = sat.highRateClock / GnssUtils::kC;
            corr.receivedUtc = now;
            _highRateClockCorrections[key] = corr;
        }

        if (ssr.kind == RtcmSsrCorrectionKind::CodeBias) {
            CodeBiasState bias;
            bias.sat = sat.sat;
            bias.sourceId = sourceId;
            bias.updateIntervalIndex = ssr.updateIntervalIndex;
            bias.epochTime = ssr.epochTime;
            bias.epochTimeIsGps = ssr.epochTimeIsGps;
            bias.messageType = ssr.messageType;
            bias.biases = sat.codeBiases;
            bias.receivedUtc = now;
            _codeBiases[key] = bias;
        }

        if (ssr.kind == RtcmSsrCorrectionKind::PhaseBias) {
            PhaseBiasState bias;
            bias.sat = sat.sat;
            bias.sourceId = sourceId;
            bias.updateIntervalIndex = ssr.updateIntervalIndex;
            bias.epochTime = ssr.epochTime;
            bias.epochTimeIsGps = ssr.epochTimeIsGps;
            bias.messageType = ssr.messageType;
            bias.dispersiveConsistency = ssr.dispersiveBiasConsistency;
            bias.mwConsistency = ssr.mwConsistency;
            bias.yawRad = sat.yawAngleRad;
            bias.yawRateRadPerSec = sat.yawRateRadPerSec;
            bias.biases = sat.phaseBiases;
            bias.receivedUtc = now;
            _phaseBiases[key] = bias;
            if (_options.logBiasMatchDetails) {
                QStringList sigs;
                for (const RtcmSsrPhaseBiasEntry& entry : sat.phaseBiases) {
                    sigs << QString("%1:%2:SII%3:WLI%4")
                                .arg(entry.signalCode)
                                .arg(ssrSignalToRinexType2ch(sat.sat.system, entry.signalCode, ssr.messageType))
                                .arg(entry.signalIntegerIndicator)
                                .arg(entry.wideLaneIntegerIndicator);
                }
                const QString diagKey = QString("SSRPHASE|%1|%2|%3")
                                            .arg(sourceId, sat.sat.key())
                                            .arg(ssr.epochTime);
                if (!_biasMatchDiagKeys.contains(diagKey) && _biasMatchDiagKeys.size() < 512) {
                    _biasMatchDiagKeys.insert(diagKey);
                    qInfo().noquote() << QString("PPP bias ingest: source=%1 sat=%2 mt=%3 epoch=%4 phaseBiases=%5 signals=[%6] yawDeg=%7 yawRateDeg=%8")
                                             .arg(sourceId)
                                             .arg(sat.sat.key())
                                             .arg(ssr.messageType)
                                             .arg(ssr.epochTime)
                                             .arg(sat.phaseBiases.size())
                                             .arg(sigs.join(QLatin1Char(',')))
                                             .arg(sat.yawAngleRad * kRhoDeg, 0, 'f', 3)
                                             .arg(sat.yawRateRadPerSec * kRhoDeg, 0, 'f', 6);
                }
            }
        }
    }
}

bool PppEngine::processMsm(const RtcmMsmMessage& msm,
                           const QString& sourceId,
                           const QMap<int, int>& gloChannels,
                           PppSolution& solution) {
    return processMsmInternal(&msm, sourceId, gloChannels, nullptr, solution);
}

PppEngine::MsmAssembly PppEngine::prepareMsmEpochs(
    const RtcmMsmMessage* msm,
    const QString& sourceId,
    const QMap<int, int>& gloChannels,
    const PppStandardizedEpoch* standardizedEpoch) {
    MsmAssembly result;
    result.directMode = standardizedEpoch && !standardizedEpoch->observations.isEmpty();
    result.gloChannels = gloChannels;

    const QDateTime nowUtc = QDateTime::currentDateTimeUtc();
    int gpsWeekNow = 0;
    double nowSow = 0.0;
    GnssUtils::gpsWeekFromDateTime(nowUtc, gpsWeekNow, nowSow);
    int currentGpsWeek = result.directMode
        ? standardizedEpoch->gpsWeek
        : (msm->gpsTimeValid ? msm->gpsWeek : gpsWeekNow);
    const double currentGpsSow = result.directMode
        ? standardizedEpoch->gpsSow
        : (msm->gpsTimeValid ? msm->gpsSow : msmEpochToGpsSow(*msm, currentGpsWeek, nowSow));
    const qint64 currentEpochMsKey = gpsEpochToMsKey(currentGpsWeek, currentGpsSow);
    const QString firstStandardizedSat = result.directMode
        ? standardizedEpoch->observations.first().satKey
        : QString();
    const QChar inputSystem = result.directMode
        ? (firstStandardizedSat.isEmpty() ? QChar() : firstStandardizedSat.at(0))
        : msm->system;
    result.systemEpochKey = QString("%1|%2").arg(sourceId, QString(inputSystem));
    result.gpsWeek = currentGpsWeek;
    result.gpsSow = currentGpsSow;
    result.epochMsKey = currentEpochMsKey;

    if (result.directMode) {
        result.ready = true;
        return result;
    }

    PendingMsmEpoch& pending = _pendingMsmBySystem[result.systemEpochKey];
    RtcmMsmMessage msmWork;
    int msmWorkGpsWeek = gpsWeekNow;
    double msmWorkGpsSow = 0.0;
    bool haveEpochToProcess = false;

    if (!pending.valid) {
        pending.valid = true;
        pending.epochMsKey = currentEpochMsKey;
        pending.gpsWeek = currentGpsWeek;
        pending.gpsSow = currentGpsSow;
        pending.msm = *msm;
    } else if (pending.epochMsKey == currentEpochMsKey) {
        mergeMsmEpoch(pending.msm, *msm);
    } else {
        msmWork = pending.msm;
        result.epochMsKey = pending.epochMsKey;
        msmWorkGpsWeek = pending.gpsWeek;
        msmWorkGpsSow = pending.gpsSow;
        haveEpochToProcess = true;
        pending.valid = true;
        pending.epochMsKey = currentEpochMsKey;
        pending.gpsWeek = currentGpsWeek;
        pending.gpsSow = currentGpsSow;
        pending.msm = *msm;
    }

    if (!haveEpochToProcess) {
        if (pending.msm.multipleMessage) {
            return result;
        }
        msmWork = pending.msm;
        result.epochMsKey = pending.epochMsKey;
        msmWorkGpsWeek = pending.gpsWeek;
        msmWorkGpsSow = pending.gpsSow;
        pending.valid = false;
        haveEpochToProcess = true;
    }
    result.gpsWeek = msmWorkGpsWeek;
    result.gpsSow = msmWorkGpsSow;

    if (_lastClosedEpochPerSource.contains(sourceId) &&
        result.epochMsKey <= _lastClosedEpochPerSource.value(sourceId) + 50) {
        return result;
    }
    if (_lastProcessedEpochPerSystem.contains(result.systemEpochKey) &&
        _lastProcessedEpochPerSystem.value(result.systemEpochKey) == result.epochMsKey) {
        return result;
    }
    _lastProcessedEpochPerSystem[result.systemEpochKey] = result.epochMsKey;

    constexpr qint64 kMaxSyncMs = 50;
    QString epochGroupKey;
    for (auto it = _pendingMsmByEpoch.constBegin(); it != _pendingMsmByEpoch.constEnd(); ++it) {
        const PendingMsmGroup& candidate = it.value();
        if (candidate.valid && candidate.sourceId == sourceId &&
            qAbs(candidate.epochMsKey - result.epochMsKey) <= kMaxSyncMs) {
            epochGroupKey = it.key();
            break;
        }
    }
    if (epochGroupKey.isEmpty()) {
        epochGroupKey = QString("%1|%2").arg(sourceId, QString::number(result.epochMsKey));
    }
    PendingMsmGroup& group = _pendingMsmByEpoch[epochGroupKey];
    if (!group.valid) {
        group.valid = true;
        group.sourceId = sourceId;
        group.gpsWeek = result.gpsWeek;
        group.gpsSow = result.gpsSow;
        group.epochMsKey = result.epochMsKey;
        group.receivedUtc = nowUtc;
    }
    PendingMsmEpoch readyMsm;
    readyMsm.valid = true;
    readyMsm.gpsWeek = result.gpsWeek;
    readyMsm.gpsSow = result.gpsSow;
    readyMsm.epochMsKey = result.epochMsKey;
    readyMsm.msm = msmWork;
    group.msmBySystem[msmWork.system] = readyMsm;
    for (auto itGlo = gloChannels.constBegin(); itGlo != gloChannels.constEnd(); ++itGlo) {
        group.gloChannels[itGlo.key()] = itGlo.value();
    }

    QString readyGroupKey;
    qint64 oldestEpochMsKey = std::numeric_limits<qint64>::max();
    for (auto it = _pendingMsmByEpoch.constBegin(); it != _pendingMsmByEpoch.constEnd(); ++it) {
        const PendingMsmGroup& candidate = it.value();
        if (!candidate.valid || candidate.sourceId != sourceId) continue;
        if (candidate.epochMsKey < oldestEpochMsKey) {
            oldestEpochMsKey = candidate.epochMsKey;
            readyGroupKey = it.key();
        }
    }
    bool hasNewerEpoch = false;
    if (!readyGroupKey.isEmpty()) {
        for (auto it = _pendingMsmByEpoch.constBegin(); it != _pendingMsmByEpoch.constEnd(); ++it) {
            const PendingMsmGroup& candidate = it.value();
            if (candidate.valid && candidate.sourceId == sourceId && candidate.epochMsKey > oldestEpochMsKey) {
                hasNewerEpoch = true;
                break;
            }
        }
    }
    if (readyGroupKey.isEmpty() || !hasNewerEpoch) {
        return result;
    }

    const PendingMsmGroup groupWork = _pendingMsmByEpoch.take(readyGroupKey);
    result.epochs.reserve(groupWork.msmBySystem.size());
    for (auto it = groupWork.msmBySystem.constBegin(); it != groupWork.msmBySystem.constEnd(); ++it) {
        result.epochs.push_back(it.value());
    }
    result.gloChannels = groupWork.gloChannels;
    result.gpsWeek = groupWork.gpsWeek;
    result.gpsSow = groupWork.gpsSow;
    result.epochMsKey = groupWork.epochMsKey;
    _lastClosedEpochPerSource[sourceId] = result.epochMsKey;
    result.ready = true;
    return result;
}

QStringList PppEngine::correctionSourceCandidates(const QString& selectedSourceId,
                                                  const QString& observationSourceId,
                                                  CorrectionRole role) const {
    QStringList sources;
    auto appendUnique = [&sources](const QString& sourceId) {
        if (!sourceId.isEmpty() && !sources.contains(sourceId)) {
            sources.push_back(sourceId);
        }
    };
    appendUnique(selectedSourceId);
    if (!_options.strictCorrectionSource) {
        if (role == CorrectionRole::Orbit) {
            appendUnique(_selection.clockId);
            appendUnique(_selection.broadcastId);
        } else if (role == CorrectionRole::Clock) {
            appendUnique(_selection.orbitId);
            appendUnique(_selection.broadcastId);
        } else {
            appendUnique(_selection.orbitId);
            appendUnique(_selection.clockId);
            appendUnique(_selection.broadcastId);
            appendUnique(observationSourceId);
        }
    }
    return sources;
}

bool PppEngine::correctionIsFresh(QChar system, int epochTime, bool epochTimeIsGps,
                                  int updateIntervalIndex, int gpsWeek, double gpsSow) const {
    if (epochTime < 0) {
        return false;
    }
    double updateInterval = GnssUtils::ssrUpdateIntervalSeconds(updateIntervalIndex);
    if (!std::isfinite(updateInterval) || updateInterval <= 0.0) {
        updateInterval = 30.0;
    }
    const double correctionSow = ssrEpochToGpsSow(system, epochTime, gpsWeek, gpsSow,
                                                   epochTimeIsGps);
    const double age = wrapTowDiff(gpsSow, correctionSow);
    const double maxAge = clampValue(3.0 * updateInterval, 30.0, 300.0);
    return std::isfinite(age) && std::fabs(age) <= maxAge;
}

PppEngine::PairObservationBuild PppEngine::buildPairObservations(
    const MsmAssembly& assembly, const QString& sourceId,
    const PppStandardizedEpoch* standardizedEpoch) {
    PairObservationBuild build;
    auto& pairObs = build.observations;
    const bool directMode = assembly.directMode;
    const QVector<PendingMsmEpoch>& msmEpochs = assembly.epochs;
    const QMap<int, int>& epochGloChannels = assembly.gloChannels;
    int skippedStaleCodeBias = 0;
    int skippedStalePhaseBias = 0;
    QStringList staleBiasExamples;
    QSet<QString> staleBiasExampleKeys;
    auto biasMaxAgeSec = [](int updateIntervalIndex) {
        double ui = GnssUtils::ssrUpdateIntervalSeconds(updateIntervalIndex);
        if (!std::isfinite(ui) || ui <= 0.0) {
            ui = 30.0;
        }
        return clampValue(3.0 * ui, 30.0, 300.0);
    };
    auto biasAgeSec = [](QChar system, int epochTime, int gpsWeek, double gpsSow,
                         bool epochTimeIsGps) {
        if (epochTime < 0) {
            return std::numeric_limits<double>::quiet_NaN();
        }
        const double epochSow = ssrEpochToGpsSow(system, epochTime, gpsWeek, gpsSow,
                                                 epochTimeIsGps);
        return wrapTowDiff(gpsSow, epochSow);
    };
    auto noteStaleBias = [&](const QString& kind,
                             const QString& satKey,
                             const QString& source,
                             int epochTime,
                             double age,
                             double maxAge) {
        const QString key = QString("%1|%2|%3|%4").arg(kind, satKey, source).arg(epochTime);
        if (staleBiasExampleKeys.contains(key)) {
            return;
        }
        staleBiasExampleKeys.insert(key);
        if (staleBiasExamples.size() < 4) {
            staleBiasExamples << QString("%1:%2 src=%3 epoch=%4 age=%5 max=%6")
                                     .arg(kind, satKey, source)
                                     .arg(epochTime)
                                     .arg(age, 0, 'f', 1)
                                     .arg(maxAge, 0, 'f', 1);
        }
    };

    auto pickBands = [&](QChar sys, QChar& b1, QChar& b2) -> bool {
        return frequencyBandsFromPriorities(_options.signalPriorities, sys, b1, b2);
    };
    auto signalPriorityScore = [&](QChar sys, const QString& type2ch) {
        if (type2ch.size() < 2) {
            return std::numeric_limits<int>::max();
        }
        const QChar band = type2ch.at(0);
        const QChar attr = type2ch.at(1);
        int best = std::numeric_limits<int>::max();
        const QStringList tokens = _options.signalPriorities.split(QLatin1Char(' '), QString::SkipEmptyParts);
        for (const QString& token : tokens) {
            const int colon = token.indexOf(QLatin1Char(':'));
            const int amp = token.indexOf(QLatin1Char('&'), colon + 1);
            if (colon != 1 || amp < 0 || token.at(0) != sys) {
                continue;
            }
            const QString bands = token.mid(colon + 1, amp - colon - 1);
            const QString attrs = token.mid(amp + 1);
            const int bandPos = bands.indexOf(band);
            const int attrPos = attrs.indexOf(attr);
            if (bandPos >= 0 && attrPos >= 0) {
                best = qMin(best, bandPos * 100 + attrPos);
            }
        }
        return best;
    };

    if (directMode) {
        for (const PppStandardizedObservation& direct : standardizedEpoch->observations) {
            if (direct.satKey.isEmpty()) continue;
            PairObs& target = pairObs[direct.satKey];
            target.c1 = direct.c1; target.c2 = direct.c2;
            target.l1 = direct.l1; target.l2 = direct.l2;
            target.hasC1 = direct.hasC1; target.hasC2 = direct.hasC2;
            target.hasL1 = direct.hasL1; target.hasL2 = direct.hasL2;
            target.signal1 = direct.signal1; target.signal2 = direct.signal2;
            target.b1 = direct.b1; target.b2 = direct.b2;
            target.gloChannel = direct.gloChannel; target.slip = direct.slip;
            target.useSsrYaw = direct.useSsrYaw; target.yawRad = direct.yawRad;
            target.sii1 = direct.sii1; target.sii2 = direct.sii2;
            target.wli1 = direct.wli1; target.wli2 = direct.wli2;
            target.lock1 = direct.lock1; target.lock2 = direct.lock2;
            target.disc1 = direct.disc1; target.disc2 = direct.disc2;
            target.hasCodeBias1 = direct.hasCodeBias1; target.hasCodeBias2 = direct.hasCodeBias2;
            target.hasBias1 = direct.hasBias1; target.hasBias2 = direct.hasBias2;
            target.codeBias1 = direct.codeBias1; target.codeBias2 = direct.codeBias2;
            target.phaseBias1 = direct.phaseBias1; target.phaseBias2 = direct.phaseBias2;
            target.phaseStateSeen = direct.phaseStateSeen;
            target.phaseStateSource = direct.phaseStateSource;
            target.phaseStateMessageType = direct.phaseStateMessageType;
            target.phaseStateEpoch = direct.phaseStateEpoch;
            target.phaseStateSignalCount = direct.phaseStateSignalCount;
            target.score1 = direct.score1; target.score2 = direct.score2;
            target.gpsWeek = standardizedEpoch->gpsWeek; target.gpsSow = standardizedEpoch->gpsSow;
            target.gpsTimeValid = true;

            const QChar sys = direct.satKey.at(0);
            const QStringList directSignals = QStringList() << direct.signal1 << direct.signal2;
            const bool bie = _options.arAlgorithm.compare(QStringLiteral("BIE"), Qt::CaseInsensitive) == 0;
            for (int bandIndex = 0; bandIndex < 2; ++bandIndex) {
                const QString signal = directSignals.at(bandIndex);
                if (signal.isEmpty()) continue;
                bool* hasCode = bandIndex == 0 ? &target.hasCodeBias1 : &target.hasCodeBias2;
                bool* hasPhase = bandIndex == 0 ? &target.hasBias1 : &target.hasBias2;
                double* codeValue = bandIndex == 0 ? &target.codeBias1 : &target.codeBias2;
                double* phaseValue = bandIndex == 0 ? &target.phaseBias1 : &target.phaseBias2;
                int* sii = bandIndex == 0 ? &target.sii1 : &target.sii2;
                int* wli = bandIndex == 0 ? &target.wli1 : &target.wli2;
                int* disc = bandIndex == 0 ? &target.disc1 : &target.disc2;
                for (const QString& sid : correctionSourceCandidates(
                         _selection.codeBiasId, sourceId, CorrectionRole::Bias)) {
                    const CodeBiasState state = _codeBiases.value(sourceSatKey(sid, direct.satKey));
                    if (!state.sat.isValid()) continue;
                    const double age = biasAgeSec(sys, state.epochTime, standardizedEpoch->gpsWeek,
                                                  standardizedEpoch->gpsSow, state.epochTimeIsGps);
                    if (!std::isfinite(age) || std::fabs(age) > biasMaxAgeSec(state.updateIntervalIndex)) continue;
                    for (const RtcmSsrCodeBiasEntry& entry : state.biases) {
                        if (ssrSignalToRinexType2ch(sys, entry.signalCode, state.messageType) == signal) {
                            *hasCode = true; *codeValue = entry.biasMeters; break;
                        }
                    }
                    if (*hasCode) break;
                }
                for (const QString& sid : (bie && !_options.enablePppAr)
                                              ? QStringList()
                                              : correctionSourceCandidates(_selection.phaseBiasId, sourceId,
                                                                           CorrectionRole::Bias)) {
                    const PhaseBiasState state = _phaseBiases.value(sourceSatKey(sid, direct.satKey));
                    if (!state.sat.isValid()) continue;
                    const double age = biasAgeSec(sys, state.epochTime, standardizedEpoch->gpsWeek,
                                                  standardizedEpoch->gpsSow, state.epochTimeIsGps);
                    if (!std::isfinite(age) || std::fabs(age) > biasMaxAgeSec(state.updateIntervalIndex)) continue;
                    for (const RtcmSsrPhaseBiasEntry& entry : state.biases) {
                        if (bie && entry.signalIntegerIndicator == 0) continue;
                        const QString biasSignal = ssrSignalToRinexType2ch(sys, entry.signalCode, state.messageType);
                        if (biasSignal.isEmpty() || biasSignal.at(0) != signal.at(0)) continue;
                        *hasPhase = true; *phaseValue = entry.biasMeters;
                        *sii = entry.signalIntegerIndicator; *wli = entry.wideLaneIntegerIndicator;
                        *disc = entry.discontinuityCounter;
                        target.phaseStateSeen = true; target.phaseStateSource = state.sourceId;
                        target.phaseStateMessageType = state.messageType;
                        target.phaseStateEpoch = state.epochTime; target.phaseStateSignalCount = state.biases.size();
                        target.useSsrYaw = true; target.yawRad = state.yawRad;
                        break;
                    }
                    if (*hasPhase) break;
                }
            }
        }
    }

    if (!directMode) for (const PendingMsmEpoch& pendingMsm : msmEpochs) {
        const RtcmMsmMessage& msmWork = pendingMsm.msm;
        const int msmGpsWeek = pendingMsm.gpsWeek;
        const double msmGpsSow = pendingMsm.gpsSow;
        QMap<int, RtcmMsmSatelliteData> satDataMap;
        for (const RtcmMsmSatelliteData& sat : msmWork.satellites) {
            satDataMap[sat.satId] = sat;
        }

        for (const RtcmMsmCellData& cell : msmWork.cells) {
        if (!cell.active) continue;
        if (!cell.hasFinePseudorange || !cell.hasFinePhaseRange) continue;
        const RtcmMsmSatelliteData satData = satDataMap.value(cell.satId);
        if (satData.roughRangeIntegerMillis < 0) continue;
        const QString signalType = msmSignalToRinexType2ch(msmWork.system, cell.signalId, msmWork.signalIds);
        if (signalType.isEmpty()) continue;

        QChar b1, b2;
        if (!pickBands(msmWork.system, b1, b2)) continue;
        const QChar band = signalType.at(0);
        if (band != b1 && band != b2) continue;
        const int signalScore = signalPriorityScore(msmWork.system, signalType);
        if (signalScore == std::numeric_limits<int>::max()) continue;

        RtcmSatId satId;
        satId.system = msmWork.system;
        satId.number = cell.satId;
        satId.navType = navTypeFromSystemAndSat(msmWork.system, cell.satId);
        const QString satKey = satId.key();
        PairObs& obs = pairObs[satKey];
        obs.gpsWeek = msmGpsWeek;
        obs.gpsSow = msmGpsSow;
        obs.gpsTimeValid = true;

        const double roughMillis = static_cast<double>(satData.roughRangeIntegerMillis) +
                                   satData.roughRangeModuloMillis;
        double codeMeters = std::numeric_limits<double>::quiet_NaN();
        if (cell.hasFinePseudorange) {
            codeMeters = (roughMillis + cell.finePseudorangeMillis) * 1e-3 * GnssUtils::kC;
        }
        double phaseMeters = std::numeric_limits<double>::quiet_NaN();
        if (cell.hasFinePhaseRange) {
            phaseMeters = (roughMillis + cell.finePhaseRangeMillis) * 1e-3 * GnssUtils::kC;
        }

        double selectedCodeBias = 0.0;
        bool matchedCodeBias = false;
        for (const QString& sid : correctionSourceCandidates(
                 _selection.codeBiasId, sourceId, CorrectionRole::Bias)) {
            const QString key = sourceSatKey(sid, satKey);
            const CodeBiasState biasState = _codeBiases.value(key);
            if (!biasState.sat.isValid()) {
                continue;
            }
            const double age = biasAgeSec(biasState.sat.system, biasState.epochTime, msmGpsWeek,
                                          msmGpsSow, biasState.epochTimeIsGps);
            const double maxAge = biasMaxAgeSec(biasState.updateIntervalIndex);
            if (!std::isfinite(age) || std::fabs(age) > maxAge) {
                ++skippedStaleCodeBias;
                noteStaleBias(QStringLiteral("code"), satKey, sid, biasState.epochTime, age, maxAge);
                continue;
            }
            bool matched = false;
            for (const RtcmSsrCodeBiasEntry& bias : biasState.biases) {
                if (ssrSignalToRinexType2ch(msmWork.system, bias.signalCode, biasState.messageType) == signalType) {
                    selectedCodeBias = bias.biasMeters;
                    matched = true;
                    break;
                }
            }
            if (matched) {
                matchedCodeBias = true;
                break;
            }
        }

        if ((band == b1 && signalScore >= obs.score1) ||
            (band == b2 && signalScore >= obs.score2)) {
            continue;
        }
        if (band == b1) {
            obs.hasC1 = false;
            obs.hasL1 = false;
            obs.sii1 = 0;
            obs.wli1 = 0;
            obs.lock1 = -1;
            obs.disc1 = -1;
            obs.hasBias1 = false;
            obs.hasCodeBias1 = matchedCodeBias;
            obs.codeBias1 = 0.0;
            obs.phaseBias1 = 0.0;
            obs.codeBias1 = selectedCodeBias;
        } else {
            obs.hasC2 = false;
            obs.hasL2 = false;
            obs.sii2 = 0;
            obs.wli2 = 0;
            obs.lock2 = -1;
            obs.disc2 = -1;
            obs.hasBias2 = false;
            obs.hasCodeBias2 = matchedCodeBias;
            obs.codeBias2 = 0.0;
            obs.phaseBias2 = 0.0;
            obs.codeBias2 = selectedCodeBias;
        }
        PhaseBiasState selectedPhaseState;
        bool havePhaseState = false;
        QStringList phaseStateSignals;
        const bool bieMode =
            _options.arAlgorithm.compare(QStringLiteral("BIE"), Qt::CaseInsensitive) == 0;
        const bool usePhaseBias = !bieMode ||
                                  (_options.enablePppAr && _options.arUseSystem(msmWork.system));
        for (const QString& sid : usePhaseBias
                                      ? correctionSourceCandidates(_selection.phaseBiasId, sourceId,
                                                                   CorrectionRole::Bias)
                                      : QStringList()) {
            const QString key = sourceSatKey(sid, satKey);
            const PhaseBiasState biasState = _phaseBiases.value(key);
            if (!biasState.sat.isValid()) {
                continue;
            }
            const double age = biasAgeSec(biasState.sat.system, biasState.epochTime, msmGpsWeek,
                                          msmGpsSow, biasState.epochTimeIsGps);
            const double maxAge = biasMaxAgeSec(biasState.updateIntervalIndex);
            if (!std::isfinite(age) || std::fabs(age) > maxAge) {
                ++skippedStalePhaseBias;
                noteStaleBias(QStringLiteral("phase"), satKey, sid, biasState.epochTime, age, maxAge);
                continue;
            }
            selectedPhaseState = biasState;
            havePhaseState = true;
            phaseStateSignals.clear();
            for (const RtcmSsrPhaseBiasEntry& bias : biasState.biases) {
                phaseStateSignals << QString("%1:%2:SII%3:WLI%4:D%5")
                                         .arg(bias.signalCode)
                                         .arg(ssrSignalToRinexType2ch(msmWork.system, bias.signalCode, biasState.messageType))
                                         .arg(bias.signalIntegerIndicator)
                                         .arg(bias.wideLaneIntegerIndicator)
                                         .arg(bias.discontinuityCounter);
            }
            bool matched = false;
            for (const RtcmSsrPhaseBiasEntry& bias : biasState.biases) {
                if (bieMode && bias.signalIntegerIndicator == 0) {
                    continue;
                }
                const QString biasSignal = ssrSignalToRinexType2ch(
                    msmWork.system, bias.signalCode, biasState.messageType);
                if (!signalType.isEmpty() && !biasSignal.isEmpty() &&
                    biasSignal.at(0) == signalType.at(0)) {
                    if (band == b1) {
                        obs.sii1 = bias.signalIntegerIndicator;
                        obs.wli1 = bias.wideLaneIntegerIndicator;
                        obs.disc1 = bias.discontinuityCounter;
                        obs.hasBias1 = true;
                        obs.phaseBias1 = bias.biasMeters;
                    } else if (band == b2) {
                        obs.sii2 = bias.signalIntegerIndicator;
                        obs.wli2 = bias.wideLaneIntegerIndicator;
                        obs.disc2 = bias.discontinuityCounter;
                        obs.hasBias2 = true;
                        obs.phaseBias2 = bias.biasMeters;
                    }
                    matched = true;
                    break;
                }
            }
            if (matched) {
                break;
            }
        }
        if (_options.logBiasMatchDetails && havePhaseState &&
            ((band == b1 && !obs.hasBias1) || (band == b2 && !obs.hasBias2))) {
            const QString diagKey = QString("BIASMATCH|%1|%2|%3|%4|%5")
                                        .arg(sourceId, satKey, signalType)
                                        .arg(msmWork.messageType)
                                        .arg(selectedPhaseState.epochTime);
            if (!_biasMatchDiagKeys.contains(diagKey) && _biasMatchDiagKeys.size() < 512) {
                _biasMatchDiagKeys.insert(diagKey);
                qInfo().noquote() << QString("PPP bias match miss: obsSource=%1 phaseSource=%2 sat=%3 msmType=%4 msmSignalId=%5 msmSignal=%6 band=%7 selectedBands=%8/%9 phaseMt=%10 phaseEpoch=%11 phaseSignals=[%12]")
                                         .arg(sourceId)
                                         .arg(selectedPhaseState.sourceId)
                                         .arg(satKey)
                                         .arg(msmWork.messageType)
                                         .arg(cell.signalId)
                                         .arg(signalType)
                                         .arg(QString(band))
                                         .arg(QString(b1))
                                         .arg(QString(b2))
                                         .arg(selectedPhaseState.messageType)
                                         .arg(selectedPhaseState.epochTime)
                                         .arg(phaseStateSignals.join(QLatin1Char(',')));
            }
        }

        if (havePhaseState) {
            obs.phaseStateSeen = true;
            obs.phaseStateSource = selectedPhaseState.sourceId;
            obs.phaseStateMessageType = selectedPhaseState.messageType;
            obs.phaseStateEpoch = selectedPhaseState.epochTime;
            obs.phaseStateSignalCount = selectedPhaseState.biases.size();
            double yaw = selectedPhaseState.yawRad;
            double epochSow = 0.0;
            double dtRawSec = 0.0;
            double dtSec = 0.0;
            double ui = 0.0;
            if (selectedPhaseState.epochTime >= 0) {
                epochSow = ssrEpochToGpsSow(satId.system, selectedPhaseState.epochTime,
                                             msmGpsWeek, msmGpsSow,
                                             selectedPhaseState.epochTimeIsGps);
                dtRawSec = wrapTowDiff(msmGpsSow, epochSow);
                dtSec = dtRawSec;
                ui = GnssUtils::ssrUpdateIntervalSeconds(selectedPhaseState.updateIntervalIndex);
                if (ui > 0.0) {
                    dtSec -= 0.5 * ui;
                }
                yaw += selectedPhaseState.yawRateRadPerSec * dtSec;
            }
            obs.useSsrYaw = true;
            obs.yawRad = yaw;
            if (_options.logSsrYawDetails && _lastYawDebug.size() < 32) {
                SsrYawDebug dbg;
                dbg.prn = QString("%1%2").arg(satId.system).arg(satId.number, 2, 10, QLatin1Char('0'));
                dbg.sourceId = selectedPhaseState.sourceId;
                dbg.messageType = selectedPhaseState.messageType;
                dbg.updateIntervalIndex = selectedPhaseState.updateIntervalIndex;
                dbg.epochSow = epochSow;
                dbg.recvSow = msmGpsSow;
                dbg.dtRawSec = dtRawSec;
                dbg.uiSec = ui;
                dbg.dtSec = dtSec;
                dbg.yawRad = yaw;
                dbg.yawRateRad = selectedPhaseState.yawRateRadPerSec;
                _lastYawDebug.push_back(dbg);
            }
        }

        obs.b1 = b1;
        obs.b2 = b2;
        if (msmWork.system == QLatin1Char('R')) {
            const auto itG = epochGloChannels.constFind(cell.satId);
            if (itG != epochGloChannels.constEnd()) obs.gloChannel = itG.value();
        }
        if (cell.hasHalfCycle && cell.halfCycleAmbiguity != 0) {
            obs.slip = true;
        }
        if (cell.hasLockTime) {
            if (band == b1) {
                obs.lock1 = cell.lockTimeIndicator;
            } else if (band == b2) {
                obs.lock2 = cell.lockTimeIndicator;
            }
        }

        if (band == b1) {
            if (signalScore >= obs.score1) {
                continue;
            }
            obs.score1 = signalScore;
            if (!std::isnan(codeMeters)) { obs.c1 = codeMeters; obs.hasC1 = true; }
            if (!std::isnan(phaseMeters)) {
                const double wl = wavelengthForPair(msmWork.system, b1, obs.gloChannel);
                if (wl > 0.0) { obs.l1 = phaseMeters / wl; obs.hasL1 = true; }
            }
        } else {
            if (signalScore >= obs.score2) {
                continue;
            }
            obs.score2 = signalScore;
            if (!std::isnan(codeMeters)) { obs.c2 = codeMeters; obs.hasC2 = true; }
            if (!std::isnan(phaseMeters)) {
                const double wl = wavelengthForPair(msmWork.system, b2, obs.gloChannel);
                if (wl > 0.0) { obs.l2 = phaseMeters / wl; obs.hasL2 = true; }
            }
        }
    }
    }

    build.skippedStaleCodeBias = skippedStaleCodeBias;
    build.skippedStalePhaseBias = skippedStalePhaseBias;
    build.staleBiasExamples = staleBiasExamples;
    return build;
}

bool PppEngine::buildEpochData(const QString& sourceId,
                               const MsmAssembly& assembly,
                               const PairObservationBuild& pairBuild,
                               PppEpochData& epochData,
                               EpochBuildDiagnostics& diagnostics,
                               QString& failureReason) {
    const QMap<int, int>& epochGloChannels = assembly.gloChannels;
    int gpsWeek = assembly.gpsWeek;
    const bool ssrConfigured = !_selection.orbitId.isEmpty() || !_selection.clockId.isEmpty();
    auto fail = [&failureReason](const QString& reason) {
        failureReason = reason;
        return false;
    };
    const QMap<QString, PairObs>& pairObs = pairBuild.observations;
    int skippedStaleCodeBias = pairBuild.skippedStaleCodeBias;
    int skippedStalePhaseBias = pairBuild.skippedStalePhaseBias;
    int skippedStaleOrbit = 0;
    int skippedStaleClock = 0;
    const QStringList staleBiasExamples = pairBuild.staleBiasExamples;
    auto pairObsAvailable = [](const PairObs& obs, ObservationCombination lc) {
        switch (lc) {
        case ObservationCombination::C1: return obs.hasC1 && obs.c1 != 0.0;
        case ObservationCombination::C2: return obs.hasC2 && obs.c2 != 0.0;
        case ObservationCombination::CIf: return obs.hasC1 && obs.hasC2 && obs.c1 != 0.0 && obs.c2 != 0.0;
        case ObservationCombination::L1: return obs.hasL1 && obs.l1 != 0.0;
        case ObservationCombination::L2: return obs.hasL2 && obs.l2 != 0.0;
        case ObservationCombination::LIf: return obs.hasL1 && obs.hasL2 && obs.l1 != 0.0 && obs.l2 != 0.0;
        case ObservationCombination::Gim:
        case ObservationCombination::Dummy:
        default: break;
        }
        return true;
    };
    auto pairObsValidForOptions = [&](QChar sys, const PairObs& obs) {
        for (ObservationCombination lc : combinationsForSystem(_options, sys)) {
            if (lc == ObservationCombination::Gim) {
                continue;
            }
            if (!pairObsAvailable(obs, lc)) {
                return false;
            }
        }
        return true;
    };

    double firstObsSec = 0.0;
    double meanDt = 0.0;
    int syncObsCount = 0;
    bool syncTimeSet = false;
    for (auto it = pairObs.constBegin(); it != pairObs.constEnd(); ++it) {
        const PairObs& obs = it.value();
        const QChar sys = it.key().isEmpty() ? QChar() : it.key().at(0);
        if (!obs.gpsTimeValid || !obs.hasC1 || sys.isNull() ||
            !_options.useSystem(sys) || !pairObsValidForOptions(sys, obs)) {
            continue;
        }
        ++_lastSatelliteDiagnostics[diagnosticSatKey(it.key())].observationAvailable;
        const double obsSec = static_cast<double>(obs.gpsWeek) * 604800.0 + obs.gpsSow;
        if (!syncTimeSet) {
            firstObsSec = obsSec;
            syncTimeSet = true;
        }
        const double dt = obsSec - firstObsSec;
        if (std::fabs(dt) > 0.05) {
            return fail(QString("asynchronous MSM observations: dt=%1s").arg(dt, 0, 'f', 3));
        }
        meanDt += dt;
        ++syncObsCount;
    }
    if (syncObsCount == 0) {
        return fail(QStringLiteral("no valid dual/single-frequency observations after signal selection"));
    }
    const double meanObsSec = firstObsSec + meanDt / static_cast<double>(syncObsCount);
    gpsWeek = static_cast<int>(std::floor(meanObsSec / 604800.0));
    double epochMeanSow = meanObsSec - static_cast<double>(gpsWeek) * 604800.0;
    while (epochMeanSow < 0.0) {
        epochMeanSow += 604800.0;
        --gpsWeek;
    }
    while (epochMeanSow >= 604800.0) {
        epochMeanSow -= 604800.0;
        ++gpsWeek;
    }

    epochData = PppEpochData();
    epochData.tt = timeFromGps(gpsWeek, epochMeanSow);
    int skippedNoC1 = 0;
    int skippedNoBroadcast = 0;
    int skippedBroadcastOutdated = 0;
    int skippedNoLambda = 0;
    int skippedNoOrbit = 0;
    int skippedOrbitIod = 0;
    int skippedOrbitStale = 0;
    int skippedNoClock = 0;
    int skippedClockIod = 0;
    int skippedClockStale = 0;
    int skippedSatState = 0;
    int skippedIncompleteBias = 0;
    for (auto it = pairObs.constBegin(); it != pairObs.constEnd(); ++it) {
        RtcmSatId satId;
        const QString satKey = it.key();
        const QString diagSatKey = diagnosticSatKey(satKey);
        const QStringList satParts = satKey.split(':');
        if (satParts.isEmpty() || satParts.at(0).size() < 3) continue;
        satId.system = satParts.at(0).at(0);
        satId.number = satParts.at(0).mid(1).toInt();
        satId.navType = navTypeFromSystemAndSat(satId.system, satId.number);
        if (!_options.useSystem(satId.system)) continue;

        const PairObs& obs = it.value();
        if (!obs.hasC1) {
            ++skippedNoC1;
            continue;
        }

        const PppPreciseSatelliteState preciseState = _preciseStates.value(satKey);
        const bool havePrecise = preciseState.sat.isValid() &&
                                 preciseState.hasClock &&
                                 std::isfinite(preciseState.clockSec);
        const BroadcastState* bState = nullptr;
        OrbitCorrectionState selectedOrbitState;
        ClockCorrectionState selectedClockState;
        bool haveSelectedOrbit = false;
        bool haveSelectedClock = false;
        auto isUsableBroadcast = [&](const BroadcastState& state) {
            if (!state.model) {
                return false;
            }
            if (!_selection.broadcastId.isEmpty() && state.sourceId != _selection.broadcastId) {
                return false;
            }
            if (state.eph.sat.system != satId.system || state.eph.sat.number != satId.number) {
                return false;
            }
            if (outDatedBcepLike(*state.model, epochData.tt)) {
                return false;
            }
            return true;
        };
        const QStringList orbitSourceIds = correctionSourceCandidates(
            _selection.orbitId, sourceId, CorrectionRole::Orbit);
        const QStringList clockSourceIds = correctionSourceCandidates(
            _selection.clockId, sourceId, CorrectionRole::Clock);
        auto findOrbitForState = [&](const BroadcastState& state,
                                     OrbitCorrectionState& out,
                                     bool& iodMismatch) {
            if (state.hasOrbitCorrection &&
                orbitSourceIds.contains(state.orbitCorrection.sourceId) &&
                (state.orbitCorrection.iod < 0 || state.orbitCorrection.iod == state.eph.iod) &&
                correctionIsFresh(state.orbitCorrection.sat.system,
                                  state.orbitCorrection.epochTime,
                                  state.orbitCorrection.epochTimeIsGps,
                                  state.orbitCorrection.updateIntervalIndex,
                                  gpsWeek, epochMeanSow)) {
                out = state.orbitCorrection;
                return true;
            }
            for (const QString& sid : orbitSourceIds) {
                const QString exactKey = sourceSatIodKey(sid, satKey, state.eph.iod);
                const auto exactIt = _orbitCorrectionsByIod.constFind(exactKey);
                if (exactIt != _orbitCorrectionsByIod.constEnd() && exactIt.value().sat.isValid()) {
                    const OrbitCorrectionState& candidate = exactIt.value();
                    if (correctionIsFresh(candidate.sat.system, candidate.epochTime,
                                          candidate.epochTimeIsGps, candidate.updateIntervalIndex,
                                          gpsWeek, epochMeanSow)) {
                        out = candidate;
                        return true;
                    }
                    ++skippedStaleOrbit;
                }
                const QString noIodKey = sourceSatIodKey(sid, satKey, -1);
                const auto noIodIt = _orbitCorrectionsByIod.constFind(noIodKey);
                if (noIodIt != _orbitCorrectionsByIod.constEnd() && noIodIt.value().sat.isValid()) {
                    const OrbitCorrectionState& candidate = noIodIt.value();
                    if (correctionIsFresh(candidate.sat.system, candidate.epochTime,
                                          candidate.epochTimeIsGps, candidate.updateIntervalIndex,
                                          gpsWeek, epochMeanSow)) {
                        out = candidate;
                        return true;
                    }
                    ++skippedStaleOrbit;
                }
                const auto latestIt = _orbitCorrections.constFind(sourceSatKey(sid, satKey));
                if (latestIt != _orbitCorrections.constEnd() && latestIt.value().sat.isValid() &&
                    latestIt.value().iod >= 0 && latestIt.value().iod != state.eph.iod) {
                    iodMismatch = true;
                }
            }
            return false;
        };
        auto findClockForState = [&](const BroadcastState& state,
                                     ClockCorrectionState& out,
                                     bool& iodMismatch) {
            if (state.hasClockCorrection &&
                clockSourceIds.contains(state.clockCorrection.sourceId) &&
                (state.clockCorrection.iod < 0 || state.clockCorrection.iod == state.eph.iod) &&
                correctionIsFresh(state.clockCorrection.sat.system,
                                  state.clockCorrection.epochTime,
                                  state.clockCorrection.epochTimeIsGps,
                                  state.clockCorrection.updateIntervalIndex,
                                  gpsWeek, epochMeanSow)) {
                out = state.clockCorrection;
                return true;
            }
            for (const QString& sid : clockSourceIds) {
                const QString exactKey = sourceSatIodKey(sid, satKey, state.eph.iod);
                const auto exactIt = _clockCorrectionsByIod.constFind(exactKey);
                if (exactIt != _clockCorrectionsByIod.constEnd() && exactIt.value().sat.isValid()) {
                    const ClockCorrectionState& candidate = exactIt.value();
                    if (correctionIsFresh(candidate.sat.system, candidate.epochTime,
                                          candidate.epochTimeIsGps, candidate.updateIntervalIndex,
                                          gpsWeek, epochMeanSow)) {
                        out = candidate;
                        return true;
                    }
                    ++skippedStaleClock;
                }
                const QString noIodKey = sourceSatIodKey(sid, satKey, -1);
                const auto noIodIt = _clockCorrectionsByIod.constFind(noIodKey);
                if (noIodIt != _clockCorrectionsByIod.constEnd() && noIodIt.value().sat.isValid()) {
                    const ClockCorrectionState& candidate = noIodIt.value();
                    if (correctionIsFresh(candidate.sat.system, candidate.epochTime,
                                          candidate.epochTimeIsGps, candidate.updateIntervalIndex,
                                          gpsWeek, epochMeanSow)) {
                        out = candidate;
                        return true;
                    }
                    ++skippedStaleClock;
                }
                const auto latestIt = _clockCorrections.constFind(sourceSatKey(sid, satKey));
                if (latestIt != _clockCorrections.constEnd() && latestIt.value().sat.isValid() &&
                    latestIt.value().iod >= 0 && latestIt.value().iod != state.eph.iod) {
                    iodMismatch = true;
                }
            }
            return false;
        };

        bool sawBroadcast = false;
        bool sawBroadcastOutdated = false;
        bool sawNoOrbit = false;
        bool sawOrbitIod = false;
        bool sawNoClock = false;
        bool sawClockIod = false;
        bool broadcastMatched = false;
        auto tryBroadcast = [&](const BroadcastState& state) {
            if (state.model && state.eph.sat.system == satId.system &&
                state.eph.sat.number == satId.number &&
                (!_selection.broadcastId.isEmpty() ? state.sourceId == _selection.broadcastId : true)) {
                sawBroadcast = true;
                const bool outdated = outDatedBcepLike(*state.model, epochData.tt);
                if (outdated) {
                    sawBroadcastOutdated = true;
                }
            }
            if (!isUsableBroadcast(state)) {
                return false;
            }
            if (!broadcastMatched) {
                ++_lastSatelliteDiagnostics[diagSatKey].broadcastMatched;
                broadcastMatched = true;
            }
            if (!ssrConfigured) {
                bState = &state;
                return true;
            }
            OrbitCorrectionState orbitState;
            ClockCorrectionState clockState;
            bool orbitIodMismatch = false;
            bool clockIodMismatch = false;
            const bool hasOrbit = findOrbitForState(state, orbitState, orbitIodMismatch);
            const bool hasClock = findClockForState(state, clockState, clockIodMismatch);
            sawNoOrbit = sawNoOrbit || !hasOrbit;
            sawOrbitIod = sawOrbitIod || orbitIodMismatch;
            sawNoClock = sawNoClock || !hasClock;
            sawClockIod = sawClockIod || clockIodMismatch;
            if (!hasOrbit || !hasClock) {
                return false;
            }
            bState = &state;
            selectedOrbitState = orbitState;
            selectedClockState = clockState;
            haveSelectedOrbit = true;
            haveSelectedClock = true;
            return true;
        };

        const QVector<BroadcastState> queuedStates = _broadcastStateQueues.value(satKey);
        for (const BroadcastState& state : queuedStates) {
            if (tryBroadcast(state)) {
                break;
            }
        }
        if (!bState) {
            const auto itBroadcast = _broadcastStates.constFind(satKey);
            if (itBroadcast != _broadcastStates.constEnd()) {
                tryBroadcast(itBroadcast.value());
            }
        }
        if (!bState) {
            for (auto itB = _broadcastStates.constBegin(); itB != _broadcastStates.constEnd(); ++itB) {
                if (tryBroadcast(itB.value())) {
                    break;
                }
            }
        }
        if (!havePrecise && (!bState || !bState->model)) {
            if (sawOrbitIod || sawClockIod) {
                ++_lastSatelliteDiagnostics[diagSatKey].iodRejected;
            }
            if (sawBroadcastOutdated) {
                ++skippedBroadcastOutdated;
            } else if (ssrConfigured && sawNoOrbit) {
                if (sawOrbitIod) ++skippedOrbitIod;
                else ++skippedNoOrbit;
            } else if (ssrConfigured && sawNoClock) {
                if (sawClockIod) ++skippedClockIod;
                else ++skippedNoClock;
            } else if (!sawBroadcast) {
                ++skippedNoBroadcast;
            } else {
                ++skippedNoBroadcast;
            }
            continue;
        }

        int gloChan = obs.gloChannel;
        if (!havePrecise && satId.system == QLatin1Char('R') &&
            !epochGloChannels.contains(satId.number) &&
            bState->eph.glonassFreqNum != 0) {
            gloChan = bState->eph.glonassFreqNum;
        }

        const double lambda1 = wavelengthForPair(satId.system, obs.b1, gloChan);
        const double lambda2 = wavelengthForPair(satId.system, obs.b2, gloChan);
        if (lambda1 <= 0.0) {
            ++skippedNoLambda;
            continue;
        }
        const double f1 = GnssUtils::kC / lambda1;
        const double f2 = (lambda2 > 0.0) ? (GnssUtils::kC / lambda2) : 0.0;
        double a1 = 0.0;
        double a2 = 0.0;
        if (f1 > 0.0 && f2 > 0.0 && std::fabs(f1 * f1 - f2 * f2) > 1e-6) {
            a1 = f1 * f1 / (f1 * f1 - f2 * f2);
            a2 = -f2 * f2 / (f1 * f1 - f2 * f2);
        }

        PppSatData sat;
        sat.tt = epochData.tt;
        sat.prn = QString("%1%2").arg(satId.system).arg(satId.number, 2, 10, QLatin1Char('0'));
        if (obs.hasC1 && obs.hasC2 && a1 != 0.0) {
            sat.P3 = a1 * obs.c1 + a2 * obs.c2;
        }
        if (obs.hasL1 && obs.hasL2 && a1 != 0.0) {
            sat.L3 = a1 * (obs.l1 * lambda1) + a2 * (obs.l2 * lambda2);
        }
        sat.C1 = obs.c1;
        sat.C2 = obs.c2;
        sat.L1 = obs.l1;
        sat.L2 = obs.l2;
        sat.lambda1 = lambda1;
        sat.lambda2 = qMax(0.0, lambda2);
        sat.f1 = f1;
        sat.f2 = f2;
        sat.slipFlag = obs.slip;
        sat.lkA = a1;
        sat.lkB = a2;
        sat.lambda3 = (f1 > 0.0 && f2 > 0.0) ? GnssUtils::kC / (f1 + f2) : 0.0;
        sat.bandA = obs.b1;
        sat.bandB = obs.b2;
        sat.useSsrYaw = obs.useSsrYaw;
        sat.yawRad = obs.yawRad;
        sat.sii1 = obs.sii1;
        sat.sii2 = obs.sii2;
        sat.wli1 = obs.wli1;
        sat.wli2 = obs.wli2;
        sat.lock1 = obs.lock1;
        sat.lock2 = obs.lock2;
        sat.disc1 = obs.disc1;
        sat.disc2 = obs.disc2;
        sat.hasCodeBias1 = obs.hasCodeBias1;
        sat.hasCodeBias2 = obs.hasCodeBias2;
        sat.hasBias1 = obs.hasBias1;
        sat.hasBias2 = obs.hasBias2;
        sat.codeBias1 = obs.codeBias1;
        sat.codeBias2 = obs.codeBias2;
        sat.phaseBias1 = obs.phaseBias1;
        sat.phaseBias2 = obs.phaseBias2;
        sat.phaseBiasDiag = QString("phaseSource=%1 phaseState=%2 phaseStateSource=%3 phaseMt=%4 phaseEpoch=%5 phaseSignals=%6")
                                .arg(_selection.phaseBiasId.isEmpty() ? QStringLiteral("-") : _selection.phaseBiasId)
                                .arg(obs.phaseStateSeen ? QStringLiteral("Y") : QStringLiteral("N"))
                                .arg(obs.phaseStateSource.isEmpty() ? QStringLiteral("-") : obs.phaseStateSource)
                                .arg(obs.phaseStateMessageType)
                                .arg(obs.phaseStateEpoch)
                                .arg(obs.phaseStateSignalCount);
        sat.satRefDatum = -1;

        SsrOrbitCorrection orbitCorr{};
        SsrOrbitCorrection* orbitPtr = nullptr;
        bool orbitIodMismatch = false;
        bool orbitStale = false;
        auto assignOrbitCorr = [&](const OrbitCorrectionState& corr) {
            sat.satRefDatum = corr.satRefDatum;
            orbitCorr.valid = true;
            orbitCorr.iod = corr.iod;
            orbitCorr.updateIntervalIndex = corr.updateIntervalIndex;
            orbitCorr.epochTime = static_cast<int>(std::round(ssrEpochToGpsSow(
                satId.system, corr.epochTime, epochData.tt.gpsWeek, epochData.tt.gpsSow,
                corr.epochTimeIsGps)));
            orbitCorr.xr = {corr.xr[0], corr.xr[1], corr.xr[2]};
            orbitCorr.dotXr = {corr.dotXr[0], corr.dotXr[1], corr.dotXr[2]};
            orbitPtr = &orbitCorr;
        };
        if (!havePrecise && haveSelectedOrbit) {
            assignOrbitCorr(selectedOrbitState);
        } else if (!havePrecise && bState->hasOrbitCorrection) {
            assignOrbitCorr(bState->orbitCorrection);
        } else if (!havePrecise) {
            for (const QString& sid : orbitSourceIds) {
                const auto corr = _orbitCorrectionsByIod.value(sourceSatIodKey(sid, satKey, bState->eph.iod));
                if (!corr.sat.isValid()) {
                    continue;
                }
                assignOrbitCorr(corr);
                break;
            }
        }

        SsrClockCorrection clockCorr{};
        SsrClockCorrection* clockPtr = nullptr;
        bool clockIodMismatch = false;
        bool clockStale = false;
        auto assignClockCorr = [&](const ClockCorrectionState& corr) {
            clockCorr.valid = true;
            clockCorr.iod = corr.iod;
            clockCorr.updateIntervalIndex = corr.updateIntervalIndex;
            clockCorr.epochTime = static_cast<int>(std::round(ssrEpochToGpsSow(
                satId.system, corr.epochTime, epochData.tt.gpsWeek, epochData.tt.gpsSow,
                corr.epochTimeIsGps)));
            clockCorr.dClk = corr.dClk;
            clockCorr.dotDClk = corr.dotDClk;
            clockCorr.dotDotDClk = corr.dotDotDClk;
            clockPtr = &clockCorr;
        };
        if (!havePrecise && haveSelectedClock) {
            assignClockCorr(selectedClockState);
        } else if (!havePrecise && bState->hasClockCorrection) {
            assignClockCorr(bState->clockCorrection);
        } else if (!havePrecise) {
            for (const QString& sid : clockSourceIds) {
                const auto corr = _clockCorrectionsByIod.value(sourceSatIodKey(sid, satKey, bState->eph.iod));
                if (!corr.sat.isValid()) {
                    continue;
                }
                assignClockCorr(corr);
                break;
            }
        }

        if (!havePrecise && ssrConfigured) {
            if (!orbitPtr) {
                if (orbitIodMismatch) ++skippedOrbitIod;
                else if (orbitStale) ++skippedOrbitStale;
                else ++skippedNoOrbit;
                continue;
            }
            if (!clockPtr) {
                if (clockIodMismatch) ++skippedClockIod;
                else if (clockStale) ++skippedClockStale;
                else ++skippedNoClock;
                continue;
            }
        }
        if (ssrConfigured && (havePrecise || (orbitPtr && clockPtr))) {
            ++_lastSatelliteDiagnostics[diagSatKey].ssrMatched;
        }

        const double codeForTransmission = (sat.P3 != 0.0) ? sat.P3 :
                                           ((sat.C1 != 0.0) ? sat.C1 : sat.C2);
        if (havePrecise) {
            sat.satPos = {preciseState.position[0], preciseState.position[1], preciseState.position[2]};
            sat.satVel = {preciseState.velocity[0], preciseState.velocity[1], preciseState.velocity[2]};
            sat.clk = preciseState.clockSec * GnssUtils::kC;
        } else if (!computeSatState(epochData.tt, codeForTransmission, *bState->model, orbitPtr, clockPtr,
                                    sat.satPos, sat.satVel, sat.clk)) {
                ++skippedSatState;
                continue;
        }
        if (!hasCompleteArBiases(_options, sat)) {
            ++skippedIncompleteBias;
            continue;
        }
        epochData.satData[sat.prn] = sat;
    }

    if (static_cast<int>(epochData.sizeAll()) < _options.minObs) {
        return fail(QString("usable satellites=%1 < minObs=%2; skips: noC1=%3 noBroadcast=%4 bcepOutdated=%5 noLambda=%6 noOrbit=%7 orbitIod=%8 orbitStale=%9 staleOrbit=%10 noClock=%11 clockIod=%12 clockStale=%13 staleClock=%14 satState=%15 incompleteBias=%16")
                        .arg(static_cast<int>(epochData.sizeAll()))
                        .arg(_options.minObs)
                        .arg(skippedNoC1)
                        .arg(skippedNoBroadcast)
                        .arg(skippedBroadcastOutdated)
                        .arg(skippedNoLambda)
                        .arg(skippedNoOrbit)
                        .arg(skippedOrbitIod)
                        .arg(skippedOrbitStale)
                        .arg(skippedStaleOrbit)
                        .arg(skippedNoClock)
                        .arg(skippedClockIod)
                        .arg(skippedClockStale)
                        .arg(skippedStaleClock)
                        .arg(skippedSatState)
                        .arg(skippedIncompleteBias));
    }

    diagnostics.skippedStaleCodeBias = skippedStaleCodeBias;
    diagnostics.skippedStalePhaseBias = skippedStalePhaseBias;
    diagnostics.skippedStaleOrbit = skippedStaleOrbit;
    diagnostics.skippedStaleClock = skippedStaleClock;
    diagnostics.skippedNoC1 = skippedNoC1;
    diagnostics.skippedNoBroadcast = skippedNoBroadcast;
    diagnostics.skippedBroadcastOutdated = skippedBroadcastOutdated;
    diagnostics.skippedNoLambda = skippedNoLambda;
    diagnostics.skippedNoOrbit = skippedNoOrbit;
    diagnostics.skippedOrbitIod = skippedOrbitIod;
    diagnostics.skippedOrbitStale = skippedOrbitStale;
    diagnostics.skippedNoClock = skippedNoClock;
    diagnostics.skippedClockIod = skippedClockIod;
    diagnostics.skippedClockStale = skippedClockStale;
    diagnostics.skippedSatState = skippedSatState;
    diagnostics.skippedIncompleteBias = skippedIncompleteBias;
    diagnostics.staleBiasExamples = staleBiasExamples;
    return true;
}

bool PppEngine::processMsmInternal(const RtcmMsmMessage* msm,
                                   const QString& sourceId,
                                   const QMap<int, int>& gloChannels,
                                   const PppStandardizedEpoch* standardizedEpoch,
                                   PppSolution& solution) {
    solution = PppSolution();
    _lastFailureReason.clear();
    _lastSatelliteDiagnostics.clear();
    _lastYawDebug.clear();
    _lastAntexWarnings.clear();
    auto fail = [this](const QString& reason) {
        _lastFailureReason = reason;
        return false;
    };
    if (_selection.broadcastId.isEmpty()) {
        return fail(QStringLiteral("broadcast stream is not selected"));
    }
    const bool ssrConfigured = !_selection.orbitId.isEmpty() || !_selection.clockId.isEmpty();
    if (ssrConfigured && (_selection.orbitId.isEmpty() || _selection.clockId.isEmpty())) {
        return fail(QStringLiteral("SSR correction requires both orbit and clock streams"));
    }
    const MsmAssembly assembly = prepareMsmEpochs(msm, sourceId, gloChannels, standardizedEpoch);
    if (!assembly.ready) return false;
    const qint64 epochMsKey = assembly.epochMsKey;

    const PairObservationBuild pairBuild = buildPairObservations(assembly, sourceId, standardizedEpoch);
    PppEpochData epoData;
    EpochBuildDiagnostics epochDiagnostics;
    QString epochBuildFailure;
    if (!buildEpochData(sourceId, assembly, pairBuild, epoData, epochDiagnostics, epochBuildFailure)) {
        return fail(epochBuildFailure);
    }
    const int skippedStaleCodeBias = epochDiagnostics.skippedStaleCodeBias;
    const int skippedStalePhaseBias = epochDiagnostics.skippedStalePhaseBias;
    const int skippedStaleOrbit = epochDiagnostics.skippedStaleOrbit;
    const int skippedStaleClock = epochDiagnostics.skippedStaleClock;
    const int skippedNoOrbit = epochDiagnostics.skippedNoOrbit;
    const int skippedOrbitIod = epochDiagnostics.skippedOrbitIod;
    const int skippedOrbitStale = epochDiagnostics.skippedOrbitStale;
    const int skippedNoClock = epochDiagnostics.skippedNoClock;
    const int skippedClockIod = epochDiagnostics.skippedClockIod;
    const int skippedClockStale = epochDiagnostics.skippedClockStale;
    const int skippedIncompleteBias = epochDiagnostics.skippedIncompleteBias;
    const QStringList staleBiasExamples = epochDiagnostics.staleBiasExamples;
    const int usableSatCount = static_cast<int>(epoData.sizeAll());
    const int correctionSkipCount = skippedNoOrbit + skippedOrbitIod + skippedOrbitStale + skippedStaleOrbit
                                  + skippedNoClock + skippedClockIod + skippedClockStale + skippedStaleClock;
    const int lastAcceptedSatCount = _lastAcceptedSatCountBySource.value(sourceId, 0);
    if (ssrConfigured && lastAcceptedSatCount >= 12 &&
        usableSatCount < lastAcceptedSatCount - 6 &&
        correctionSkipCount >= 4) {
        return fail(QString("usable satellites collapsed from %1 to %2 during SSR correction transition; skips: noOrbit=%3 orbitIod=%4 noClock=%5 clockIod=%6")
                        .arg(lastAcceptedSatCount)
                        .arg(usableSatCount)
                        .arg(skippedNoOrbit)
                        .arg(skippedOrbitIod)
                        .arg(skippedNoClock)
                        .arg(skippedClockIod));
    }

    _activeVtecModel = PppVtecModel();
    if (!_vtecModelsByTime.isEmpty()) {
        auto itVtec = _vtecModelsByTime.upperBound(gpsMsKey(epoData.tt));
        if (itVtec != _vtecModelsByTime.begin()) {
            --itVtec;
            _activeVtecModel = itVtec.value();
        }
    } else if (_latestVtecModel.isValid()) {
        _activeVtecModel = _latestVtecModel;
    }

    const PppOptions* filterOptions = &_options;
    const PppCrdStationOptions stationOptions = _options.crdStationsBySource.value(sourceId);
    if (stationOptions.valid) {
        PppOptions& sourceOptions = _sourceOptions[sourceId];
        sourceOptions = _options;
        for (int i = 0; i < 3; ++i) {
            sourceOptions.aprXyz[i] = stationOptions.aprXyz[i];
            sourceOptions.neuEcc[i] = stationOptions.neuEcc[i];
        }
        sourceOptions.roverAntenna = stationOptions.roverAntenna;
        if (sourceOptions.oceanStation.trimmed().isEmpty()) {
            sourceOptions.oceanStation = stationOptions.station;
        }
        filterOptions = &sourceOptions;
    }

    std::unique_ptr<PppFilter>& filter = _filters[sourceId];
    if (!filter) {
        PppTropospherePrior tropoPrior;
        const auto configuredPrior = _tropospherePriorsBySource.constFind(sourceId);
        if (filterOptions->troposphereMode == PppTroposphereMode::GfsZwdInitialization &&
            configuredPrior != _tropospherePriorsBySource.constEnd() &&
            configuredPrior->valid && std::isfinite(configuredPrior->zwdM) &&
            configuredPrior->zwdM >= 0.0) {
            tropoPrior = configuredPrior.value();
            if (tropoPrior.status.trimmed().isEmpty()) {
                tropoPrior.status = QStringLiteral("gfs_initialized");
            }
        } else {
            tropoPrior.valid = false;
            tropoPrior.source = QStringLiteral("Saastamoinen humidity climatology");
            tropoPrior.status = filterOptions->troposphereMode == PppTroposphereMode::GfsZwdInitialization
                ? QStringLiteral("gfs_unavailable_fallback")
                : QStringLiteral("original");
            if (configuredPrior != _tropospherePriorsBySource.constEnd()) {
                tropoPrior.reason = configuredPrior->reason;
            }
        }
        filter = std::make_unique<PppFilter>(filterOptions, &_activeVtecModel, tropoPrior);
    }
    const bool filterAccepted = filter->update(epoData, solution);
    for (auto it = epoData.satData.constBegin(); it != epoData.satData.constEnd(); ++it) {
        PppSatelliteStageDiagnostic& diagnostic = _lastSatelliteDiagnostics[diagnosticSatKey(it.key())];
        if (it.value().outlier) {
            ++diagnostic.residualRejected;
        } else {
            ++diagnostic.acceptedAfterResidual;
        }
    }
    if (filterAccepted) {
        if (!solution.valid || solution.numSat < _options.minObs) {
            const int reportedSatellites = solution.numSat;
            solution = PppSolution();
            return fail(QString("filter returned a degenerate solution: sats=%1 minObs=%2")
                            .arg(reportedSatellites)
                            .arg(_options.minObs));
        }
        _lastAntexWarnings = filter->takeWarnings();
        QStringList extraDiag;
        extraDiag << QString("ZTD parts zhd=%1 zwd=%2 prior=%3 [%4]")
                         .arg(solution.zhdModel, 0, 'f', 4)
                         .arg(solution.trp, 0, 'f', 4)
                         .arg(solution.zwdPrior, 0, 'f', 4)
                         .arg(solution.tropoPriorStatus);
        if (skippedStaleCodeBias > 0 || skippedStalePhaseBias > 0) {
            QString stale = QString("SSR bias stale skipped code=%1 phase=%2")
                                .arg(skippedStaleCodeBias)
                                .arg(skippedStalePhaseBias);
            if (!staleBiasExamples.isEmpty()) {
                stale += QString(" examples=[%1]").arg(staleBiasExamples.join(QStringLiteral("; ")));
            }
            extraDiag << stale;
        }
        if (skippedStaleOrbit > 0 || skippedStaleClock > 0) {
            extraDiag << QString("SSR orbit/clock stale skipped orbit=%1 clock=%2")
                             .arg(skippedStaleOrbit)
                             .arg(skippedStaleClock);
        }
        if (skippedIncompleteBias > 0) {
            extraDiag << QString("BIE incomplete-bias satellites skipped=%1")
                             .arg(skippedIncompleteBias);
        }
        if (!extraDiag.isEmpty()) {
            if (!solution.diagSummary.isEmpty()) {
                solution.diagSummary += QStringLiteral("; ");
            }
            solution.diagSummary += extraDiag.join(QStringLiteral("; "));
        }
        _lastSolution = solution;
        _lastAcceptedSatCountBySource[sourceId] = solution.numSat;
        if (_lastOutputEpochPerSource.contains(sourceId) &&
            _lastOutputEpochPerSource.value(sourceId) == epochMsKey) {
            return false;
        }
        _lastOutputEpochPerSource[sourceId] = epochMsKey;
        return true;
    }
    const QString filterReason = filter->failureReason();
    return fail(QString("filter rejected epoch: usable satellites=%1%2")
                    .arg(static_cast<int>(epoData.sizeAll()))
                    .arg(filterReason.isEmpty() ? QString() : QString("; %1").arg(filterReason)));
}

bool PppEngine::processStandardizedEpoch(const PppStandardizedEpoch& epoch,
                                         const QString& sourceId,
                                         const QMap<int, int>& gloChannels,
                                         PppSolution& solution) {
    if (epoch.observations.isEmpty()) {
        _lastFailureReason = QStringLiteral("standardized epoch contains no observations");
        return false;
    }
    return processMsmInternal(nullptr, sourceId, gloChannels, &epoch, solution);
}

const PppSolution& PppEngine::lastSolution() const {
    return _lastSolution;
}
