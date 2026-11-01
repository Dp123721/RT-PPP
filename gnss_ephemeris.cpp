// Propagates broadcast satellite states and applies SSR orbit and clock corrections.
#include "gnss_ephemeris.h"

#include <cmath>

namespace {

constexpr double kGmGps = 398.6005e12;
constexpr double kGmGal = 398.6004418e12;
constexpr double kGmBds = 398.6004418e12;
constexpr double kGmWgs = 398.60044e12;
constexpr double kRelGps = 4.442807633e-10;
constexpr double kRelGalBds = 4.442807309e-10;

double diffWeekSow(int w1, double s1, int w2, double s2) {
    return static_cast<double>(w1 - w2) * 604800.0 + (s1 - s2);
}

void normalizeWeekSow(int& week, double& sow) {
    while (sow >= 604800.0) {
        sow -= 604800.0;
        ++week;
    }
    while (sow < 0.0) {
        sow += 604800.0;
        --week;
    }
}

bool solveKepler(double ecc, double meanAnomaly, double semiMajor, double& eccentricAnomaly) {
    eccentricAnomaly = meanAnomaly;
    double prev = eccentricAnomaly;
    int loops = 0;
    do {
        prev = eccentricAnomaly;
        eccentricAnomaly = meanAnomaly + ecc * std::sin(prev);
        ++loops;
        if (loops >= 100) {
            return false;
        }
    } while (std::fabs(eccentricAnomaly - prev) * semiMajor > 0.001);
    return true;
}

void rotX(double angle, const std::array<double,3>& in, std::array<double,3>& out) {
    const double c = std::cos(angle);
    const double s = std::sin(angle);
    out[0] = in[0];
    out[1] = c * in[1] - s * in[2];
    out[2] = s * in[1] + c * in[2];
}

void rotZ(double angle, const std::array<double,3>& in, std::array<double,3>& out) {
    const double c = std::cos(angle);
    const double s = std::sin(angle);
    out[0] = c * in[0] + s * in[1];
    out[1] = -s * in[0] + c * in[1];
    out[2] = in[2];
}

bool computeGpsLike(const GnssEphemeris& eph,
                    double mu,
                    double omegaEarth,
                    double relativity,
                    int gpsWeek,
                    double gpsSow,
                    GnssEphemeris::State& state) {
    const double a0 = eph.sqrtA * eph.sqrtA;
    if (a0 == 0.0) {
        return false;
    }

    int toeWeek = eph.week;
    double toeSec = eph.toeSec;
    int tocWeek = eph.week;
    double tocSec = eph.tocSec;

    if (eph.type == GnssEphemeris::Type::Galileo) {
        toeWeek = eph.week + 1024;
        tocWeek = toeWeek;
    }

    if (eph.type == GnssEphemeris::Type::Beidou) {
        toeWeek = eph.bdsWeek + 1356;
        toeSec = eph.toeSec + 14.0;
        tocWeek = eph.bdsWeek + 1356;
        tocSec = eph.tocSec + 14.0;
        normalizeWeekSow(toeWeek, toeSec);
        normalizeWeekSow(tocWeek, tocSec);
    }

    const double n0 = std::sqrt(mu / (a0 * a0 * a0));
    const double tk = diffWeekSow(gpsWeek, gpsSow, toeWeek, toeSec);
    const double n = n0 + eph.deltaN;
    const double M = eph.M0 + n * tk;

    double E = 0.0;
    if (!solveKepler(eph.e, M, a0, E)) {
        return false;
    }

    const double v = 2.0 * std::atan(std::sqrt((1.0 + eph.e) / (1.0 - eph.e)) * std::tan(E / 2.0));
    const double u0 = v + eph.omega;
    const double sin2u0 = std::sin(2.0 * u0);
    const double cos2u0 = std::cos(2.0 * u0);
    const double r = a0 * (1.0 - eph.e * std::cos(E)) + eph.crc * cos2u0 + eph.crs * sin2u0;
    const double i = eph.i0 + eph.idot * tk + eph.cic * cos2u0 + eph.cis * sin2u0;
    const double u = u0 + eph.cuc * cos2u0 + eph.cus * sin2u0;
    const double xp = r * std::cos(u);
    const double yp = r * std::sin(u);

    if (eph.type == GnssEphemeris::Type::Beidou) {
        const double iMaxGeo = 10.0 / 180.0 * GnssUtils::kPi;
        if (eph.i0 > iMaxGeo) {
            const double OM = eph.omega0 + (eph.omegaDot - omegaEarth) * tk - omegaEarth * eph.toeSec;
            const double sinom = std::sin(OM);
            const double cosom = std::cos(OM);
            const double sini = std::sin(i);
            const double cosi = std::cos(i);
            state.position[0] = xp * cosom - yp * cosi * sinom;
            state.position[1] = xp * sinom + yp * cosi * cosom;
            state.position[2] = yp * sini;

            const double tanv2 = std::tan(v / 2.0);
            const double dEdM = 1.0 / (1.0 - eph.e * std::cos(E));
            const double dotv = std::sqrt((1.0 + eph.e) / (1.0 - eph.e)) /
                                std::cos(E / 2.0) / std::cos(E / 2.0) /
                                (1.0 + tanv2 * tanv2) * dEdM * n;
            const double dotu = dotv + (-eph.cuc * sin2u0 + eph.cus * cos2u0) * 2.0 * dotv;
            const double dotom = eph.omegaDot - GnssUtils::kOmegaEarth;
            const double doti = eph.idot + (-eph.cic * sin2u0 + eph.cis * cos2u0) * 2.0 * dotv;
            const double dotr = a0 * eph.e * std::sin(E) * dEdM * n
                                + (-eph.crc * sin2u0 + eph.crs * cos2u0) * 2.0 * dotv;
            const double dotx = dotr * std::cos(u) - r * std::sin(u) * dotu;
            const double doty = dotr * std::sin(u) + r * std::cos(u) * dotu;

            state.velocity[0] = cosom * dotx - cosi * sinom * doty
                                - xp * sinom * dotom - yp * cosi * cosom * dotom
                                + yp * sini * sinom * doti;
            state.velocity[1] = sinom * dotx + cosi * cosom * doty + xp * cosom * dotom
                                - yp * cosi * sinom * dotom - yp * sini * cosom * doti;
            state.velocity[2] = sini * doty + yp * cosi * doti;
        } else {
            const double OM = eph.omega0 + eph.omegaDot * tk - GnssUtils::kOmegaBds * eph.toeSec;
            const double ll = GnssUtils::kOmegaBds * tk;
            const double sinom = std::sin(OM);
            const double cosom = std::cos(OM);
            const double sini = std::sin(i);
            const double cosi = std::cos(i);

            const std::array<double,3> x1 = {
                xp * cosom - yp * cosi * sinom,
                xp * sinom + yp * cosi * cosom,
                yp * sini
            };

            std::array<double,3> x2;
            std::array<double,3> x1r;
            rotX(-5.0 / 180.0 * GnssUtils::kPi, x1, x1r);
            rotZ(ll, x1r, x2);
            state.position = x2;

            const double tanv2 = std::tan(v / 2.0);
            const double dEdM = 1.0 / (1.0 - eph.e * std::cos(E));
            const double dotv = std::sqrt((1.0 + eph.e) / (1.0 - eph.e)) /
                                std::cos(E / 2.0) / std::cos(E / 2.0) /
                                (1.0 + tanv2 * tanv2) * dEdM * n;
            const double dotu = dotv + (-eph.cuc * sin2u0 + eph.cus * cos2u0) * 2.0 * dotv;
            const double doti = eph.idot + (-eph.cic * sin2u0 + eph.cis * cos2u0) * 2.0 * dotv;
            const double dotr = a0 * eph.e * std::sin(E) * dEdM * n
                                + (-eph.crc * sin2u0 + eph.crs * cos2u0) * 2.0 * dotv;
            const double dotx = dotr * std::cos(u) - r * std::sin(u) * dotu;
            const double doty = dotr * std::sin(u) + r * std::cos(u) * dotu;
            const double dotom = eph.omegaDot;

            const std::array<double,3> vIn = {
                cosom * dotx - cosi * sinom * doty - xp * sinom * dotom
                    - yp * cosi * cosom * dotom + yp * sini * sinom * doti,
                sinom * dotx + cosi * cosom * doty + xp * cosom * dotom
                    - yp * cosi * sinom * dotom - yp * sini * cosom * doti,
                sini * doty + yp * cosi * doti
            };

            std::array<double,3> v1;
            rotX(-5.0 / 180.0 * GnssUtils::kPi, vIn, v1);
            std::array<double,3> v2;
            rotZ(ll, v1, v2);

            const double c = std::cos(ll);
            const double s = std::sin(ll);
            const std::array<double,3> rdotz = {
                GnssUtils::kOmegaBds * (-s * x1r[0] + c * x1r[1]),
                GnssUtils::kOmegaBds * (-c * x1r[0] - s * x1r[1]),
                0.0
            };

            state.velocity[0] = v2[0] + rdotz[0];
            state.velocity[1] = v2[1] + rdotz[1];
            state.velocity[2] = v2[2] + rdotz[2];
        }
    } else {
        const double OM = eph.omega0 + (eph.omegaDot - omegaEarth) * tk - omegaEarth * toeSec;
        const double sinom = std::sin(OM);
        const double cosom = std::cos(OM);
        const double sini = std::sin(i);
        const double cosi = std::cos(i);
        state.position[0] = xp * cosom - yp * cosi * sinom;
        state.position[1] = xp * sinom + yp * cosi * cosom;
        state.position[2] = yp * sini;

        const double tanv2 = std::tan(v / 2.0);
        const double dEdM = 1.0 / (1.0 - eph.e * std::cos(E));
        const double dotv = std::sqrt((1.0 + eph.e) / (1.0 - eph.e)) /
                            std::cos(E / 2.0) / std::cos(E / 2.0) /
                            (1.0 + tanv2 * tanv2) * dEdM * n;
        const double dotu = dotv + (-eph.cuc * sin2u0 + eph.cus * cos2u0) * 2.0 * dotv;
        const double dotom = eph.omegaDot - omegaEarth;
        const double doti = eph.idot + (-eph.cic * sin2u0 + eph.cis * cos2u0) * 2.0 * dotv;
        const double dotr = a0 * eph.e * std::sin(E) * dEdM * n
                            + (-eph.crc * sin2u0 + eph.crs * cos2u0) * 2.0 * dotv;
        const double dotx = dotr * std::cos(u) - r * std::sin(u) * dotu;
        const double doty = dotr * std::sin(u) + r * std::cos(u) * dotu;

        state.velocity[0] = cosom * dotx - cosi * sinom * doty
                            - xp * sinom * dotom - yp * cosi * cosom * dotom
                            + yp * sini * sinom * doti;
        state.velocity[1] = sinom * dotx + cosi * cosom * doty + xp * cosom * dotom
                            - yp * cosi * sinom * dotom - yp * sini * cosom * doti;
        state.velocity[2] = sini * doty + yp * cosi * doti;
    }

    const double tc = diffWeekSow(gpsWeek, gpsSow, tocWeek, tocSec);
    state.clockSec = eph.af0 + eph.af1 * tc + eph.af2 * tc * tc;
    state.clockSec -= relativity * eph.e * std::sqrt(a0) * std::sin(E);
    state.clockRate = eph.af1 + eph.af2 * tc;
    state.clockAcc = eph.af2;

    return true;
}

bool computeGlonass(const GnssEphemeris& eph, int gpsWeek, double gpsSow, GnssEphemeris::State& state) {
    int tocWeek = gpsWeek;
    double tocSow = 0.0;
    const bool hasRinexReference = eph.week > 0 &&
        std::isfinite(eph.tocSec) && eph.tocSec > 0.0;
    if (hasRinexReference) {
        tocWeek = eph.week;
        tocSow = eph.tocSec;
    } else {
        int secOfWeek = static_cast<int>(std::floor(gpsSow));
        GnssUtils::updatetime(&tocWeek, &secOfWeek, eph.glonassTbSec * 1000, false);
        tocSow = static_cast<double>(secOfWeek);
    }

    const double dtPos = diffWeekSow(gpsWeek, gpsSow, tocWeek, tocSow);
    if (std::fabs(dtPos) > 24.0 * 3600.0) {
        return false;
    }

    std::array<double, 6> xv = {
        eph.glonassXPosKm * 1e3,
        eph.glonassYPosKm * 1e3,
        eph.glonassZPosKm * 1e3,
        eph.glonassXVelKmps * 1e3,
        eph.glonassYVelKmps * 1e3,
        eph.glonassZVelKmps * 1e3
    };
    const std::array<double, 3> acc = {
        eph.glonassXAccKmps2 * 1e3,
        eph.glonassYAccKmps2 * 1e3,
        eph.glonassZAccKmps2 * 1e3
    };

    const double nominalStep = 10.0;
    const int nSteps = static_cast<int>(std::fabs(dtPos) / nominalStep) + 1;
    const double step = dtPos / static_cast<double>(nSteps);

    auto deriv = [&acc](const std::array<double,6>& in) -> std::array<double,6> {
        const std::array<double,3> rr = {in[0], in[1], in[2]};
        const std::array<double,3> vv = {in[3], in[4], in[5]};
        const double rho = GnssUtils::norm(rr);
        const double rho2 = rho * rho;
        const double z2 = rr[2] * rr[2];

        const double AE = 6378136.0;
        const double C20 = -1082.6257e-6;
        const double t1 = -kGmWgs / (rho * rho * rho);
        const double t2 = 1.5 * C20 * (kGmWgs * AE * AE) / (rho * rho * rho * rho * rho);
        const double t3 = GnssUtils::kOmegaEarth * GnssUtils::kOmegaEarth;
        const double t4 = 2.0 * GnssUtils::kOmegaEarth;

        std::array<double,6> out = {};
        out[0] = vv[0];
        out[1] = vv[1];
        out[2] = vv[2];
        out[3] = (t1 + t2 * (1.0 - 5.0 * z2 / rho2) + t3) * rr[0] + t4 * vv[1] + acc[0];
        out[4] = (t1 + t2 * (1.0 - 5.0 * z2 / rho2) + t3) * rr[1] - t4 * vv[0] + acc[1];
        out[5] = (t1 + t2 * (3.0 - 5.0 * z2 / rho2)) * rr[2] + acc[2];
        return out;
    };

    for (int stepIndex = 0; stepIndex < nSteps; ++stepIndex) {
        const std::array<double,6> k1 = deriv(xv);
        std::array<double,6> tmp = {};
        for (int i = 0; i < 6; ++i) tmp[i] = xv[i] + 0.5 * step * k1[i];
        const std::array<double,6> k2 = deriv(tmp);
        for (int i = 0; i < 6; ++i) tmp[i] = xv[i] + 0.5 * step * k2[i];
        const std::array<double,6> k3 = deriv(tmp);
        for (int i = 0; i < 6; ++i) tmp[i] = xv[i] + step * k3[i];
        const std::array<double,6> k4 = deriv(tmp);
        for (int i = 0; i < 6; ++i) {
            xv[i] += (step / 6.0) * (k1[i] + 2.0 * k2[i] + 2.0 * k3[i] + k4[i]);
        }
    }

    state.position = {xv[0], xv[1], xv[2]};
    state.velocity = {xv[3], xv[4], xv[5]};

    const double dtClk = diffWeekSow(gpsWeek, gpsSow, tocWeek, tocSow);
    state.clockSec = -eph.glonassTauSec + eph.glonassGamma * dtClk;
    state.clockRate = eph.glonassGamma;
    state.clockAcc = 0.0;
    return true;
}

} // namespace

bool GnssEphemeris::computeState(int gpsWeek, double gpsSow, State& state) const {
    switch (type) {
    case Type::Gps:
        return computeGpsLike(*this, kGmGps, GnssUtils::kOmegaEarth, kRelGps, gpsWeek, gpsSow, state);
    case Type::Galileo:
        return computeGpsLike(*this, kGmGal, GnssUtils::kOmegaEarth, kRelGalBds, gpsWeek, gpsSow, state);
    case Type::Beidou:
        return computeGpsLike(*this, kGmBds, GnssUtils::kOmegaBds, kRelGalBds, gpsWeek, gpsSow, state);
    case Type::Glonass:
        return computeGlonass(*this, gpsWeek, gpsSow, state);
    default:
        break;
    }
    return false;
}

bool GnssEphemeris::computeReferenceTimes(int gpsWeek,
                                          double gpsSow,
                                          int& tocWeekOut,
                                          double& tocSowOut,
                                          int& toeWeekOut,
                                          double& toeSowOut) const {
    switch (type) {
    case Type::Gps:
        tocWeekOut = week;
        tocSowOut = tocSec;
        toeWeekOut = week;
        toeSowOut = toeSec;
        return true;
    case Type::Galileo:
        tocWeekOut = week + 1024;
        tocSowOut = tocSec;
        toeWeekOut = week + 1024;
        toeSowOut = toeSec;
        return true;
    case Type::Beidou:
        tocWeekOut = bdsWeek + 1356;
        tocSowOut = tocSec + 14.0;
        toeWeekOut = bdsWeek + 1356;
        toeSowOut = toeSec + 14.0;
        normalizeWeekSow(tocWeekOut, tocSowOut);
        normalizeWeekSow(toeWeekOut, toeSowOut);
        return true;
    case Type::Glonass: {
        const bool hasRinexReference = week > 0 &&
            std::isfinite(tocSec) && tocSec > 0.0;
        if (hasRinexReference) {
            tocWeekOut = week;
            tocSowOut = tocSec;
        } else {
            tocWeekOut = gpsWeek;
            int sec = static_cast<int>(std::floor(gpsSow));
            GnssUtils::updatetime(&tocWeekOut, &sec, glonassTbSec * 1000, false);
            tocSowOut = static_cast<double>(sec);
        }
        toeWeekOut = tocWeekOut;
        toeSowOut = tocSowOut;
        return true;
    }
    default:
        break;
    }
    return false;
}

bool GnssEphemeris::fromRtcm(const RtcmBroadcastEphemeris& rtcm, std::shared_ptr<GnssEphemeris>& out) {
    if (!rtcm.sat.isValid() || !rtcm.validOrbit) {
        return false;
    }
    std::shared_ptr<GnssEphemeris> eph = std::make_shared<GnssEphemeris>();
    eph->sat = rtcm.sat;
    eph->iod = rtcm.iod;
    eph->navMessageType = rtcm.messageType;
    eph->week = rtcm.week;
    eph->bdsWeek = rtcm.bdsWeek;
    eph->toeSec = rtcm.toeSec;
    eph->tocSec = rtcm.tocSec;
    eph->sqrtA = rtcm.sqrtA;
    eph->e = rtcm.e;
    eph->M0 = rtcm.M0;
    eph->deltaN = rtcm.deltaN;
    eph->i0 = rtcm.i0;
    eph->idot = rtcm.idot;
    eph->omega = rtcm.omega;
    eph->omega0 = rtcm.omega0;
    eph->omegaDot = rtcm.omegaDot;
    eph->cuc = rtcm.cuc;
    eph->cus = rtcm.cus;
    eph->crc = rtcm.crc;
    eph->crs = rtcm.crs;
    eph->cic = rtcm.cic;
    eph->cis = rtcm.cis;
    eph->af0 = rtcm.af0;
    eph->af1 = rtcm.af1;
    eph->af2 = rtcm.af2;
    eph->glonassFreqNum = rtcm.glonassFreqNum;
    eph->glonassTbSec = rtcm.glonassTbSec;
    eph->glonassXPosKm = rtcm.glonassXPosKm;
    eph->glonassYPosKm = rtcm.glonassYPosKm;
    eph->glonassZPosKm = rtcm.glonassZPosKm;
    eph->glonassXVelKmps = rtcm.glonassXVelKmps;
    eph->glonassYVelKmps = rtcm.glonassYVelKmps;
    eph->glonassZVelKmps = rtcm.glonassZVelKmps;
    eph->glonassXAccKmps2 = rtcm.glonassXAccKmps2;
    eph->glonassYAccKmps2 = rtcm.glonassYAccKmps2;
    eph->glonassZAccKmps2 = rtcm.glonassZAccKmps2;
    eph->glonassTauSec = rtcm.glonassTauSec;
    eph->glonassGamma = rtcm.glonassGamma;

    if (rtcm.sat.system == QLatin1Char('G')) eph->type = Type::Gps;
    else if (rtcm.sat.system == QLatin1Char('R')) eph->type = Type::Glonass;
    else if (rtcm.sat.system == QLatin1Char('E')) eph->type = Type::Galileo;
    else if (rtcm.sat.system == QLatin1Char('C')) eph->type = Type::Beidou;
    else return false;

    out = eph;
    return true;
}

bool applySsrOrbitClock(const GnssEphemeris::State& broadcastState,
                        const SsrOrbitCorrection* orbitCorr,
                        const SsrClockCorrection* clockCorr,
                        int gpsWeek,
                        double gpsSow,
                        GnssEphemeris::State& correctedState,
                        double* orbitDtSec,
                        double* clockDtSec) {
    correctedState = broadcastState;

    if (orbitCorr && orbitCorr->valid) {
        double dtO = gpsSow - static_cast<double>(orbitCorr->epochTime);
        while (dtO > 302400.0) dtO -= 604800.0;
        while (dtO < -302400.0) dtO += 604800.0;
        const double ui = GnssUtils::ssrUpdateIntervalSeconds(orbitCorr->updateIntervalIndex);
        if (ui > 0.0) {
            dtO -= 0.5 * ui;
        }
        if (orbitDtSec) {
            *orbitDtSec = dtO;
        }

        std::array<double,3> rsw = {
            orbitCorr->xr[0] + orbitCorr->dotXr[0] * dtO,
            orbitCorr->xr[1] + orbitCorr->dotXr[1] * dtO,
            orbitCorr->xr[2] + orbitCorr->dotXr[2] * dtO
        };
        std::array<double,3> dx = {0.0, 0.0, 0.0};
        GnssUtils::rswToXyz(correctedState.position, correctedState.velocity, rsw, dx);
        correctedState.position[0] -= dx[0];
        correctedState.position[1] -= dx[1];
        correctedState.position[2] -= dx[2];

        std::array<double,3> dv = {0.0, 0.0, 0.0};
        GnssUtils::rswToXyz(correctedState.position, correctedState.velocity, orbitCorr->dotXr, dv);
        correctedState.velocity[0] -= dv[0];
        correctedState.velocity[1] -= dv[1];
        correctedState.velocity[2] -= dv[2];
    }

    if (clockCorr && clockCorr->valid) {
        double dtC = gpsSow - static_cast<double>(clockCorr->epochTime);
        while (dtC > 302400.0) dtC -= 604800.0;
        while (dtC < -302400.0) dtC += 604800.0;
        const double ui = GnssUtils::ssrUpdateIntervalSeconds(clockCorr->updateIntervalIndex);
        if (ui > 0.0) {
            dtC -= 0.5 * ui;
        }
        if (clockDtSec) {
            *clockDtSec = dtC;
        }
        correctedState.clockSec += clockCorr->dClk +
                                   clockCorr->dotDClk * dtC +
                                   clockCorr->dotDotDClk * dtC * dtC;
    }

    (void)gpsWeek;
    return true;
}
