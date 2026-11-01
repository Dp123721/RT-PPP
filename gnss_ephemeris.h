// Declares broadcast ephemeris propagation and SSR orbit/clock correction APIs.
#pragma once

#include "rtcmdecoder.h"
#include "gnss_utils.h"

#include <array>
#include <memory>

class GnssEphemeris
{
public:
    enum class Type {
        Unknown = 0,
        Gps,
        Glonass,
        Galileo,
        Beidou
    };

    struct State {
        std::array<double, 3> position = {0.0, 0.0, 0.0};
        std::array<double, 3> velocity = {0.0, 0.0, 0.0};
        double clockSec = 0.0;
        double clockRate = 0.0;
        double clockAcc = 0.0;
    };

    Type type = Type::Unknown;
    RtcmSatId sat;
    int iod = -1;
    int navMessageType = 0;

    int week = 0;
    int bdsWeek = 0;
    double toeSec = 0.0;
    double tocSec = 0.0;

    double sqrtA = 0.0;
    double e = 0.0;
    double M0 = 0.0;
    double deltaN = 0.0;
    double i0 = 0.0;
    double idot = 0.0;
    double omega = 0.0;
    double omega0 = 0.0;
    double omegaDot = 0.0;
    double cuc = 0.0;
    double cus = 0.0;
    double crc = 0.0;
    double crs = 0.0;
    double cic = 0.0;
    double cis = 0.0;
    double af0 = 0.0;
    double af1 = 0.0;
    double af2 = 0.0;

    int glonassFreqNum = 0;
    int glonassTbSec = 0;
    double glonassXPosKm = 0.0;
    double glonassYPosKm = 0.0;
    double glonassZPosKm = 0.0;
    double glonassXVelKmps = 0.0;
    double glonassYVelKmps = 0.0;
    double glonassZVelKmps = 0.0;
    double glonassXAccKmps2 = 0.0;
    double glonassYAccKmps2 = 0.0;
    double glonassZAccKmps2 = 0.0;
    double glonassTauSec = 0.0;
    double glonassGamma = 0.0;

    bool computeState(int gpsWeek, double gpsSow, State& state) const;
    bool computeReferenceTimes(int gpsWeek, double gpsSow, int& tocWeekOut, double& tocSowOut,
                               int& toeWeekOut, double& toeSowOut) const;

    static bool fromRtcm(const RtcmBroadcastEphemeris& rtcm, std::shared_ptr<GnssEphemeris>& out);
};

struct SsrOrbitCorrection {
    bool valid = false;
    int iod = -1;
    int updateIntervalIndex = -1;
    int epochTime = -1;
    std::array<double,3> xr = {0.0, 0.0, 0.0};
    std::array<double,3> dotXr = {0.0, 0.0, 0.0};
};

struct SsrClockCorrection {
    bool valid = false;
    int iod = -1;
    int updateIntervalIndex = -1;
    int epochTime = -1;
    double dClk = 0.0;
    double dotDClk = 0.0;
    double dotDotDClk = 0.0;
};

bool applySsrOrbitClock(const GnssEphemeris::State& broadcastState,
                        const SsrOrbitCorrection* orbitCorr,
                        const SsrClockCorrection* clockCorr,
                        int gpsWeek,
                        double gpsSow,
                        GnssEphemeris::State& correctedState,
                        double* orbitDtSec = nullptr,
                        double* clockDtSec = nullptr);
