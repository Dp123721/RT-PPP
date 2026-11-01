// Declares RTCM3 frame assembly, message records, and decoder interfaces.
#ifndef RTCMDECODER_H
#define RTCMDECODER_H

#include <QByteArray>
#include <QChar>
#include <QMap>
#include <QString>
#include <QVector>

enum class RtcmSsrCorrectionKind {
    Unknown = 0,
    Orbit,
    Clock,
    OrbitClock,
    CodeBias,
    PhaseBias,
    HighRateClock,
    Ura
};

enum RtcmNavType {
    RtcmNavUnknown = 0,
    RtcmNavLnav = 1,
    RtcmNavFdmaM = 2,
    RtcmNavInav = 3,
    RtcmNavD1 = 4,
    RtcmNavD2 = 5,
    RtcmNavSbasL1 = 6
};

struct RtcmSatId {
    QChar system;
    int number = 0;
    int navType = 0;

    bool isValid() const {
        return !system.isNull() && number > 0;
    }

    QString key() const {
        if (!isValid()) {
            return QString();
        }
        return QString("%1%2:%3")
            .arg(system)
            .arg(number, 2, 10, QLatin1Char('0'))
            .arg(navType);
    }
};

struct RtcmBroadcastEphemeris {
    RtcmSatId sat;
    int iod = -1;
    int messageType = 0;
    bool validOrbit = false;

    int week = 0;
    int bdsWeek = 0;
    double toeSec = 0.0;
    double tocSec = 0.0;
    double tocSsrSec = 0.0;

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

    double tgd1 = 0.0;
    double tgd2 = 0.0;
    int health = 0;
    int fitIntervalFlag = 0;

    int glonassFreqNum = 0;
    int glonassTkSec = 0;
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
    int glonassAlmanacHealth = 0;
    int glonassAlmanacHealthAvailability = 0;
};

struct RtcmSsrCodeBiasEntry {
    int signalCode = -1;
    double biasMeters = 0.0;
};

struct RtcmSsrPhaseBiasEntry {
    int signalCode = -1;
    int signalIntegerIndicator = 0;
    int wideLaneIntegerIndicator = 0;
    int discontinuityCounter = 0;
    double biasMeters = 0.0;
};

struct RtcmSsrSatelliteData {
    RtcmSatId sat;
    int iod = -1;
    int toe = -1;

    bool hasOrbit = false;
    double deltaRadial = 0.0;
    double deltaAlongTrack = 0.0;
    double deltaCrossTrack = 0.0;
    double dotDeltaRadial = 0.0;
    double dotDeltaAlongTrack = 0.0;
    double dotDeltaCrossTrack = 0.0;

    bool hasClock = false;
    double deltaC0 = 0.0;
    double deltaC1 = 0.0;
    double deltaC2 = 0.0;

    bool hasHighRateClock = false;
    double highRateClock = 0.0;

    QVector<RtcmSsrCodeBiasEntry> codeBiases;
    QVector<RtcmSsrPhaseBiasEntry> phaseBiases;

    double yawAngleRad = 0.0;
    double yawRateRadPerSec = 0.0;
};

struct RtcmSsrMessage {
    RtcmSsrCorrectionKind kind = RtcmSsrCorrectionKind::Unknown;
    QChar system;
    int messageType = 0;
    int epochTime = -1;
    // Set by file adapters whose epoch header is an absolute GPS timestamp.
    // Live RTCM messages retain zero and use their native system epoch rules.
    int gpsWeek = 0;
    int updateIntervalIndex = -1;
    bool multipleMessage = false;
    int ssrIod = -1;
    int providerId = -1;
    int solutionId = -1;
    int satRefDatum = -1;
    int dispersiveBiasConsistency = -1;
    int mwConsistency = -1;
    bool epochTimeIsGps = false;
    QVector<RtcmSsrSatelliteData> satellites;
};

struct RtcmVtecLayer {
    double heightMeters = 0.0;
    QVector<QVector<double>> cosine;
    QVector<QVector<double>> sine;
};

struct RtcmVtecMessage {
    int messageType = 0;
    int epochTime = -1;
    int updateIntervalIndex = -1;
    bool multipleMessage = false;
    int ssrIod = -1;
    int providerId = -1;
    int solutionId = -1;
    double quality = 0.0;
    QVector<RtcmVtecLayer> layers;
};

struct RtcmMsmSatelliteData {
    int satId = 0;
    int roughRangeIntegerMillis = -1;
    double roughRangeModuloMillis = 0.0;
    int extendedInfo = 15;
    int roughPhaseRangeRate = 0;
};

struct RtcmMsmCellData {
    int satId = 0;
    int signalId = 0;
    bool active = false;
    bool hasFinePseudorange = false;
    bool hasFinePhaseRange = false;
    bool hasLockTime = false;
    bool hasHalfCycle = false;
    bool hasCnr = false;
    bool hasFinePhaseRate = false;

    double finePseudorangeMillis = 0.0;
    double finePhaseRangeMillis = 0.0;
    double lockTimeSec = -1.0;
    int lockTimeIndicator = 0;
    int halfCycleAmbiguity = 0;
    double cnrDbHz = 0.0;
    double finePhaseRateMps = 0.0;
};

struct RtcmMsmMessage {
    QChar system;
    int messageType = 0;
    int msmSubtype = 0;
    int stationId = 0;
    int epochTime = 0;
    int glonassDay = 0;
    bool multipleMessage = false;
    int iods = 0;
    int clockSteeringIndicator = 0;
    int externalClockIndicator = 0;
    bool smoothingIndicator = false;
    int smoothingInterval = 0;
    // Set by offline adapters.  Realtime MSMs leave this false and retain
    // the legacy epoch-time reconstruction path.
    bool gpsTimeValid = false;
    int gpsWeek = 0;
    double gpsSow = 0.0;

    QVector<RtcmMsmSatelliteData> satellites;
    QVector<int> signalIds;
    QVector<RtcmMsmCellData> cells;
};

struct RtcmDecodedMessage {
    int type = 0;
    int payloadLength = 0;
    QString category;
    QString summary;
    bool ignored = false;
    bool hasMsm = false;
    RtcmMsmMessage msm;
    bool hasBroadcastEphemeris = false;
    RtcmBroadcastEphemeris broadcastEphemeris;
    bool hasSsr = false;
    RtcmSsrMessage ssr;
    bool hasVtec = false;
    RtcmVtecMessage vtec;
};

struct RtcmDecodeBatch {
    QVector<RtcmDecodedMessage> messages;
    int crcErrors = 0;
    int malformedFrames = 0;
};

class Rtcm3Decoder
{
public:
    RtcmDecodeBatch pushBytes(const QByteArray& bytes);
    void reset();
    bool hasBufferedData() const;
    QMap<int, quint64> typeCounters() const;

private:
    RtcmDecodedMessage decodePayload(const QByteArray& payload);

    static QString decodeMsmSummary(int type, const QByteArray& payload);
    static QString decodeLegacyObsSummary(int type, const QByteArray& payload);
    static QString decodeAntennaPositionSummary(int type, const QByteArray& payload);
    static QString decodeAntennaReceiverSummary(int type, const QByteArray& payload);
    static QString decodeGloBiasSummary(const QByteArray& payload);
    static QString decodeEphemerisSummary(int type, const QByteArray& payload);
    static QString decodeSsrSummary(int type, const QByteArray& payload);
    static QString decodeCrsSummary(int type, const QByteArray& payload);
    static QString categoryForType(int type);
    static QString ssrKindToText(RtcmSsrCorrectionKind kind);

    QByteArray _buffer;
    QMap<int, quint64> _typeCounters;
    int _referenceGpsWeek = 0;
    double _referenceGpsSow = 0.0;
    bool _referenceGpsTimeValid = false;
};

#endif // RTCMDECODER_H
