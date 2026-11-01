// Assembles RTCM3 frames, validates CRCs, and decodes supported messages.
#include "rtcmdecoder.h"
#include "gnss_signals.h"
#include "gnss_utils.h"

#include <QtGlobal>
#include <QStringList>
#include <QDateTime>

namespace {

constexpr int kMaxRtcmPayloadBytes = 1023;
constexpr int kMaxBufferedBytes = 1024 * 128;
constexpr double kPi = 3.1415926535897932384626433832795;

class BitCursor
{
public:
    explicit BitCursor(const QByteArray& data) : _data(data) {}

    bool readUnsigned(int bitCount, quint64& out) {
        if (bitCount < 0 || bitCount > 64 || bitCount > remainingBits()) {
            return false;
        }
        out = 0;
        for (int i = 0; i < bitCount; ++i) {
            const int byteIndex = _bitPos / 8;
            const int bitIndex = 7 - (_bitPos % 8);
            const quint8 byte = static_cast<quint8>(_data.at(byteIndex));
            out = (out << 1) | ((byte >> bitIndex) & 0x01U);
            ++_bitPos;
        }
        return true;
    }

    bool readSigned(int bitCount, qint64& out) {
        quint64 unsignedValue = 0;
        if (!readUnsigned(bitCount, unsignedValue)) {
            return false;
        }
        if (bitCount <= 0) {
            out = 0;
            return true;
        }
        if (bitCount >= 64) {
            out = static_cast<qint64>(unsignedValue);
            return true;
        }
        const quint64 signBit = (1ULL << (bitCount - 1));
        if (unsignedValue & signBit) {
            const quint64 mask = (~0ULL) << bitCount;
            out = static_cast<qint64>(unsignedValue | mask);
        } else {
            out = static_cast<qint64>(unsignedValue);
        }
        return true;
    }

    bool skip(int bitCount) {
        if (bitCount < 0 || bitCount > remainingBits()) {
            return false;
        }
        _bitPos += bitCount;
        return true;
    }

    int remainingBits() const {
        return _data.size() * 8 - _bitPos;
    }

private:
    const QByteArray& _data;
    int _bitPos = 0;
};

quint32 crc24q(const char* data, int size) {
    quint32 crc = 0;
    for (int i = 0; i < size; ++i) {
        crc ^= (static_cast<quint32>(static_cast<quint8>(data[i])) << 16);
        for (int bit = 0; bit < 8; ++bit) {
            crc <<= 1;
            if (crc & 0x1000000U) {
                crc ^= 0x01864CFBU;
            }
        }
    }
    return crc & 0x00FFFFFFU;
}

int countBits(quint64 value) {
    int count = 0;
    while (value != 0) {
        count += (value & 1ULL) ? 1 : 0;
        value >>= 1;
    }
    return count;
}

QVector<int> msmSignalIdsFromMask(quint64 sigMask) {
    QVector<int> ids;
    ids.reserve(16);
    for (int bit = 0; bit < 32; ++bit) {
        if ((sigMask >> (31 - bit)) & 0x1ULL) {
            ids.push_back(bit + 1);
        }
    }
    return ids;
}

QString msmSignalInfoText(QChar system, const QVector<int>& sigIds) {
    if (sigIds.isEmpty()) {
        return QString();
    }
    QStringList tokens;
    const int maxShow = 8;
    for (int i = 0; i < sigIds.size() && i < maxShow; ++i) {
        const int signalId = sigIds.at(i);
        const QString sigType = msmSignalToRinexType2ch(system, signalId, sigIds);
        if (sigType.isEmpty()) {
            tokens << QString("%1:?").arg(signalId);
            continue;
        }
        const double wavelength = signalWavelengthMeters(system, sigType, 0);
        if (wavelength > 0.0) {
            const double mhz = GnssUtils::kC / wavelength / 1e6;
            tokens << QString("%1:%2@%3MHz")
                          .arg(signalId)
                          .arg(sigType)
                          .arg(mhz, 0, 'f', 2);
        } else {
            tokens << QString("%1:%2").arg(signalId).arg(sigType);
        }
    }
    if (sigIds.size() > maxShow) {
        tokens << QString("...+%1").arg(sigIds.size() - maxShow);
    }
    return tokens.join(",");
}

int currentGpsWeek() {
    int week = 0;
    double sow = 0.0;
    GnssUtils::gpsWeekFromDateTime(QDateTime::currentDateTimeUtc(), week, sow);
    return week;
}

GnssUtils::GpsTime streamMsmGpsTime(const RtcmMsmMessage& msm,
                                    int referenceWeek,
                                    double referenceSow,
                                    bool referenceValid) {
    const QDateTime referenceUtc = referenceValid
        ? GnssUtils::dateTimeFromGpsWeek(referenceWeek, referenceSow)
        : QDateTime::currentDateTimeUtc();
    GnssUtils::GpsTime time = GnssUtils::gpsTimeFromMsm(msm, referenceUtc);
    if (!referenceValid || msm.system == QLatin1Char('R')) {
        return time;
    }

    double systemSow = static_cast<double>(msm.epochTime) / 1000.0;
    if (msm.system == QLatin1Char('C')) {
        systemSow += 14.0;
    }
    const GnssUtils::GpsTime candidate = GnssUtils::normalizeGpsTime(referenceWeek, systemSow);
    const double dt = GnssUtils::gpsTimeDifferenceSeconds(
        candidate, GnssUtils::normalizeGpsTime(referenceWeek, referenceSow));
    if (dt < -302400.0) {
        time = GnssUtils::normalizeGpsTime(candidate.week + 1, candidate.sow);
    } else if (dt > 302400.0) {
        time = GnssUtils::normalizeGpsTime(candidate.week - 1, candidate.sow);
    } else {
        time = candidate;
    }
    return time;
}

int expandWeekRollover(int rawWeek, int bits, int referenceWeek) {
    if (bits <= 0 || bits >= 30) {
        return rawWeek;
    }
    const int cycle = (1 << bits);
    int fullWeek = rawWeek + (referenceWeek / cycle) * cycle;
    if (fullWeek - referenceWeek > cycle / 2) {
        fullWeek -= cycle;
    } else if (referenceWeek - fullWeek > cycle / 2) {
        fullWeek += cycle;
    }
    return fullWeek;
}

bool readSignMagnitude(BitCursor& cursor, int bits, double scale, double& out) {
    quint64 raw = 0;
    if (!cursor.readUnsigned(bits, raw)) {
        return false;
    }
    if (bits <= 1) {
        out = 0.0;
        return true;
    }
    const quint64 sign = raw >> (bits - 1);
    const quint64 magMask = (bits == 64) ? ~0ULL : ((1ULL << (bits - 1)) - 1ULL);
    const quint64 mag = raw & magMask;
    out = static_cast<double>(mag) * scale;
    if (sign != 0ULL) {
        out = -out;
    }
    return true;
}

bool readAsciiString(BitCursor& cursor, QString& out) {
    quint64 length = 0;
    if (!cursor.readUnsigned(8, length)) {
        return false;
    }
    QByteArray bytes;
    bytes.reserve(static_cast<int>(length));
    for (quint64 i = 0; i < length; ++i) {
        quint64 ch = 0;
        if (!cursor.readUnsigned(8, ch)) {
            return false;
        }
        bytes.push_back(static_cast<char>(ch));
    }
    out = QString::fromLatin1(bytes);
    return true;
}

bool isLegacyGpsObs(int type) {
    return type >= 1001 && type <= 1004;
}

bool isLegacyGloObs(int type) {
    return type >= 1009 && type <= 1012;
}

bool isMsm(int type) {
    return (type >= 1071 && type <= 1137);
}

bool isSupportedSolutionSystem(QChar system) {
    return system == QLatin1Char('G') ||
           system == QLatin1Char('R') ||
           system == QLatin1Char('E') ||
           system == QLatin1Char('C');
}

QString msmSystem(int type) {
    if (type >= 1071 && type <= 1077) return "G";
    if (type >= 1081 && type <= 1087) return "R";
    if (type >= 1091 && type <= 1097) return "E";
    if (type >= 1101 && type <= 1107) return "S";
    if (type >= 1111 && type <= 1117) return "J";
    if (type >= 1121 && type <= 1127) return "C";
    if (type >= 1131 && type <= 1137) return "I";
    return QString();
}

QString ssrSystem(int type, int* baseType) {
    if (type >= 1057 && type <= 1062) { if (baseType) *baseType = 1057; return "G"; }
    if (type >= 1063 && type <= 1068) { if (baseType) *baseType = 1063; return "R"; }
    if (type >= 1240 && type <= 1245) { if (baseType) *baseType = 1240; return "E"; }
    if (type >= 1246 && type <= 1251) { if (baseType) *baseType = 1246; return "J"; }
    if (type >= 1252 && type <= 1257) { if (baseType) *baseType = 1252; return "S"; }
    if (type >= 1258 && type <= 1263) { if (baseType) *baseType = 1258; return "C"; }
    return QString();
}

QChar broadcastSystem(int type) {
    switch (type) {
    case 1019: return QLatin1Char('G');
    case 1020: return QLatin1Char('R');
    case 1042: return QLatin1Char('C');
    case 1045:
    case 1046: return QLatin1Char('E');
    case 1041: return QLatin1Char('I');
    case 1043: return QLatin1Char('S');
    case 1044: return QLatin1Char('J');
    default: break;
    }
    return QChar();
}

bool readIgsSsrSystemAndKind(const QByteArray& payload,
                             QChar& system,
                             RtcmSsrCorrectionKind& kind) {
    BitCursor cursor(payload);
    quint64 messageType = 0;
    quint64 version = 0;
    quint64 igsType = 0;
    if (!cursor.readUnsigned(12, messageType) ||
        !cursor.readUnsigned(3, version) ||
        !cursor.readUnsigned(8, igsType)) {
        return false;
    }

    struct IgsBase {
        int base;
        QChar system;
    };
    const IgsBase bases[] = {
        {21, QLatin1Char('G')},
        {41, QLatin1Char('R')},
        {61, QLatin1Char('E')},
        {81, QLatin1Char('J')},
        {101, QLatin1Char('C')},
        {121, QLatin1Char('S')}
    };

    int offset = -1;
    for (const IgsBase& base : bases) {
        if (igsType >= static_cast<quint64>(base.base) &&
            igsType < static_cast<quint64>(base.base + 7)) {
            system = base.system;
            offset = static_cast<int>(igsType) - base.base;
            break;
        }
    }
    if (offset < 0) {
        return false;
    }

    kind = RtcmSsrCorrectionKind::Unknown;
    switch (offset) {
    case 0: kind = RtcmSsrCorrectionKind::Orbit; break;
    case 1: kind = RtcmSsrCorrectionKind::Clock; break;
    case 2: kind = RtcmSsrCorrectionKind::OrbitClock; break;
    case 3: kind = RtcmSsrCorrectionKind::HighRateClock; break;
    case 4: kind = RtcmSsrCorrectionKind::CodeBias; break;
    case 5: kind = RtcmSsrCorrectionKind::PhaseBias; break;
    case 6: kind = RtcmSsrCorrectionKind::Ura; break;
    default: break;
    }
    return kind != RtcmSsrCorrectionKind::Unknown;
}

QString ssrSubtypeName(int subtype) {
    switch (subtype) {
    case 0: return "orbit";
    case 1: return "clock";
    case 2: return "code-bias";
    case 3: return "orbit-clock";
    case 4: return "ura";
    case 5: return "high-rate-clock";
    default: return "unknown";
    }
}

int satIdBitsForSystem(QChar system) {
    if (system == QLatin1Char('J')) {
        return 4;
    }
    if (system == QLatin1Char('R')) {
        return 5;
    }
    return 6;
}

int navTypeFromSsrStandard(QChar system, int satNum) {
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
        return RtcmNavUnknown;
    }
}

bool readSsrSystemAndKind(int messageType, QChar& system, RtcmSsrCorrectionKind& kind) {
    if (messageType >= 1265 && messageType <= 1270) {
        switch (messageType) {
        case 1265: system = QLatin1Char('G'); break;
        case 1266: system = QLatin1Char('R'); break;
        case 1267: system = QLatin1Char('E'); break;
        case 1268: system = QLatin1Char('J'); break;
        case 1269: system = QLatin1Char('S'); break;
        case 1270: system = QLatin1Char('C'); break;
        default: break;
        }
        kind = RtcmSsrCorrectionKind::PhaseBias;
        return true;
    }

    int base = 0;
    const QString sys = ssrSystem(messageType, &base);
    if (!sys.isEmpty()) {
        system = sys.at(0);
        switch (messageType - base) {
        case 0:
            kind = RtcmSsrCorrectionKind::Orbit;
            return true;
        case 1:
            kind = RtcmSsrCorrectionKind::Clock;
            return true;
        case 2:
            kind = RtcmSsrCorrectionKind::CodeBias;
            return true;
        case 3:
            kind = RtcmSsrCorrectionKind::OrbitClock;
            return true;
        case 4:
            kind = RtcmSsrCorrectionKind::Ura;
            return true;
        case 5:
            kind = RtcmSsrCorrectionKind::HighRateClock;
            return true;
        default:
            kind = RtcmSsrCorrectionKind::Unknown;
            return true;
        }
    }

    return false;
}

bool parseBroadcastEphemeris(int type, const QByteArray& payload,
                             RtcmBroadcastEphemeris& eph, int referenceWeek) {
    BitCursor cursor(payload);
    quint64 mt = 0;
    if (!cursor.readUnsigned(12, mt)) {
        return false;
    }

    switch (type) {
    case 1019: {
        quint64 prn = 0;
        quint64 week = 0;
        quint64 ura = 0;
        quint64 l2Codes = 0;
        quint64 iod = 0;
        quint64 tocRaw = 0;
        qint64 idotRaw = 0;
        if (!cursor.readUnsigned(6, prn) ||
            !cursor.readUnsigned(10, week) ||
            !cursor.readUnsigned(4, ura) ||
            !cursor.readUnsigned(2, l2Codes) ||
            !cursor.readSigned(14, idotRaw) ||
            !cursor.readUnsigned(8, iod) ||
            !cursor.readUnsigned(16, tocRaw)) {
            return false;
        }

        auto readSigned = [&cursor](int bits, double scale, double& out) -> bool {
            qint64 raw = 0;
            if (!cursor.readSigned(bits, raw)) return false;
            out = static_cast<double>(raw) * scale;
            return true;
        };
        auto readUnsignedScaled = [&cursor](int bits, double scale, double& out) -> bool {
            quint64 raw = 0;
            if (!cursor.readUnsigned(bits, raw)) return false;
            out = static_cast<double>(raw) * scale;
            return true;
        };

        const double r2rPi = kPi;
        eph.idot = static_cast<double>(idotRaw) * r2rPi / (double)(1ULL << 30) / (double)(1ULL << 13);

        const double tocSec = static_cast<double>(tocRaw << 4);
        if (!readSigned(8, 1.0 / (double)(1ULL << 30) / (double)(1ULL << 25), eph.af2) ||
            !readSigned(16, 1.0 / (double)(1ULL << 30) / (double)(1ULL << 13), eph.af1) ||
            !readSigned(22, 1.0 / (double)(1ULL << 30) / (double)(1ULL << 1), eph.af0) ||
            !cursor.skip(10) ||
            !readSigned(16, 1.0 / (double)(1ULL << 5), eph.crs) ||
            !readSigned(16, r2rPi / (double)(1ULL << 30) / (double)(1ULL << 13), eph.deltaN) ||
            !readSigned(32, r2rPi / (double)(1ULL << 30) / (double)(1ULL << 1), eph.M0) ||
            !readSigned(16, 1.0 / (double)(1ULL << 29), eph.cuc) ||
            !readUnsignedScaled(32, 1.0 / (double)(1ULL << 30) / (double)(1ULL << 3), eph.e) ||
            !readSigned(16, 1.0 / (double)(1ULL << 29), eph.cus) ||
            !readUnsignedScaled(32, 1.0 / (double)(1ULL << 19), eph.sqrtA)) {
            return false;
        }

        quint64 toeRaw = 0;
        if (!cursor.readUnsigned(16, toeRaw)) {
            return false;
        }
        const double toeSec = static_cast<double>(toeRaw << 4);

        if (!readSigned(16, 1.0 / (double)(1ULL << 29), eph.cic) ||
            !readSigned(32, r2rPi / (double)(1ULL << 30) / (double)(1ULL << 1), eph.omega0) ||
            !readSigned(16, 1.0 / (double)(1ULL << 29), eph.cis) ||
            !readSigned(32, r2rPi / (double)(1ULL << 30) / (double)(1ULL << 1), eph.i0) ||
            !readSigned(16, 1.0 / (double)(1ULL << 5), eph.crc) ||
            !readSigned(32, r2rPi / (double)(1ULL << 30) / (double)(1ULL << 1), eph.omega) ||
            !readSigned(24, r2rPi / (double)(1ULL << 30) / (double)(1ULL << 13), eph.omegaDot) ||
            !readSigned(8, 1.0 / (double)(1ULL << 30) / (double)(1ULL << 1), eph.tgd1)) {
            return false;
        }

        quint64 health = 0;
        quint64 l2PFlag = 0;
        quint64 fitFlag = 0;
        if (!cursor.readUnsigned(6, health) ||
            !cursor.readUnsigned(1, l2PFlag) ||
            !cursor.readUnsigned(1, fitFlag)) {
            return false;
        }

        eph.sat.system = QLatin1Char('G');
        eph.sat.number = static_cast<int>(prn);
        eph.sat.navType = navTypeFromSsrStandard(eph.sat.system, eph.sat.number);
        eph.iod = static_cast<int>(iod);
        eph.messageType = type;
        eph.week = expandWeekRollover(static_cast<int>(week), 10,
                                      referenceWeek > 0 ? referenceWeek : currentGpsWeek());
        eph.tocSec = tocSec;
        eph.toeSec = toeSec;
        eph.health = static_cast<int>(health);
        eph.fitIntervalFlag = static_cast<int>(fitFlag);
        eph.validOrbit = true;
        return true;
    }
    case 1020: {
        quint64 slot = 0;
        quint64 freq = 0;
        quint64 health = 0;
        quint64 almanacHealth = 0;
        quint64 almanacAvail = 0;
        quint64 P1 = 0;
        quint64 tk1 = 0;
        quint64 tk2 = 0;
        quint64 tk3 = 0;
        quint64 P2 = 0;
        quint64 tb = 0;
        if (!cursor.readUnsigned(6, slot) ||
            !cursor.readUnsigned(5, freq) ||
            !cursor.readUnsigned(1, almanacHealth) ||
            !cursor.readUnsigned(1, almanacAvail) ||
            !cursor.readUnsigned(2, P1) ||
            !cursor.readUnsigned(5, tk1) ||
            !cursor.readUnsigned(6, tk2) ||
            !cursor.readUnsigned(1, tk3) ||
            !cursor.readUnsigned(1, health) ||
            !cursor.readUnsigned(1, P2) ||
            !cursor.readUnsigned(7, tb)) {
            return false;
        }

        double xVel = 0.0;
        double xPos = 0.0;
        double xAcc = 0.0;
        double yVel = 0.0;
        double yPos = 0.0;
        double yAcc = 0.0;
        double zVel = 0.0;
        double zPos = 0.0;
        double zAcc = 0.0;
        double gamma = 0.0;
        double tau = 0.0;
        double deltaTau = 0.0;

        if (!readSignMagnitude(cursor, 24, 1.0 / (double)(1ULL << 20), xVel) ||
            !readSignMagnitude(cursor, 27, 1.0 / (double)(1ULL << 11), xPos) ||
            !readSignMagnitude(cursor, 5, 1.0 / (double)(1ULL << 30), xAcc) ||
            !readSignMagnitude(cursor, 24, 1.0 / (double)(1ULL << 20), yVel) ||
            !readSignMagnitude(cursor, 27, 1.0 / (double)(1ULL << 11), yPos) ||
            !readSignMagnitude(cursor, 5, 1.0 / (double)(1ULL << 30), yAcc) ||
            !readSignMagnitude(cursor, 24, 1.0 / (double)(1ULL << 20), zVel) ||
            !readSignMagnitude(cursor, 27, 1.0 / (double)(1ULL << 11), zPos) ||
            !readSignMagnitude(cursor, 5, 1.0 / (double)(1ULL << 30), zAcc) ||
            !cursor.skip(1) ||
            !readSignMagnitude(cursor, 11, 1.0 / (double)(1ULL << 30) / (double)(1ULL << 10), gamma) ||
            !cursor.skip(2) ||
            !cursor.skip(1) ||
            !readSignMagnitude(cursor, 22, 1.0 / (double)(1ULL << 30), tau) ||
            !readSignMagnitude(cursor, 5, 1.0 / (double)(1ULL << 30), deltaTau) ||
            !cursor.skip(5) ||
            !cursor.skip(1) ||
            !cursor.skip(4) ||
            !cursor.skip(11) ||
            !cursor.skip(2) ||
            !cursor.skip(1) ||
            !cursor.skip(11) ||
            !readSignMagnitude(cursor, 32, 1.0 / (double)(1ULL << 30) / (double)(1ULL << 1), eph.tgd1) ||
            !cursor.skip(5) ||
            !readSignMagnitude(cursor, 22, 1.0 / (double)(1ULL << 30), eph.tgd2) ||
            !cursor.skip(1)) {
            return false;
        }

        const int tkSec = static_cast<int>(tk1 * 3600 + tk2 * 60 + tk3 * 30);
        const int tbSec = static_cast<int>(tb) * 15 * 60;

        eph.sat.system = QLatin1Char('R');
        eph.sat.number = static_cast<int>(slot);
        eph.sat.navType = navTypeFromSsrStandard(eph.sat.system, eph.sat.number);
        eph.iod = static_cast<int>(tb);
        eph.messageType = type;
        eph.glonassFreqNum = static_cast<int>(freq) - 7;
        eph.glonassTkSec = tkSec;
        eph.glonassTbSec = tbSec;
        eph.glonassXPosKm = xPos;
        eph.glonassYPosKm = yPos;
        eph.glonassZPosKm = zPos;
        eph.glonassXVelKmps = xVel;
        eph.glonassYVelKmps = yVel;
        eph.glonassZVelKmps = zVel;
        eph.glonassXAccKmps2 = xAcc;
        eph.glonassYAccKmps2 = yAcc;
        eph.glonassZAccKmps2 = zAcc;
        eph.glonassTauSec = tau;
        eph.glonassGamma = gamma;
        eph.glonassAlmanacHealth = static_cast<int>(almanacHealth);
        eph.glonassAlmanacHealthAvailability = static_cast<int>(almanacAvail);
        eph.validOrbit = true;
        return true;
    }
    case 1042: {
        quint64 prn = 0;
        quint64 week = 0;
        quint64 ura = 0;
        quint64 aode = 0;
        quint64 tocRaw = 0;
        qint64 idotRaw = 0;
        if (!cursor.readUnsigned(6, prn) ||
            !cursor.readUnsigned(13, week) ||
            !cursor.readUnsigned(4, ura) ||
            !cursor.readSigned(14, idotRaw) ||
            !cursor.readUnsigned(5, aode) ||
            !cursor.readUnsigned(17, tocRaw)) {
            return false;
        }

        auto readSigned = [&cursor](int bits, double scale, double& out) -> bool {
            qint64 raw = 0;
            if (!cursor.readSigned(bits, raw)) return false;
            out = static_cast<double>(raw) * scale;
            return true;
        };
        auto readUnsignedScaled = [&cursor](int bits, double scale, double& out) -> bool {
            quint64 raw = 0;
            if (!cursor.readUnsigned(bits, raw)) return false;
            out = static_cast<double>(raw) * scale;
            return true;
        };

        const double r2rPi = kPi;
        eph.idot = static_cast<double>(idotRaw) * r2rPi / (double)(1ULL << 30) / (double)(1ULL << 13);
        const double tocSec = static_cast<double>(tocRaw << 3);

        if (!readSigned(11, 1.0 / (double)(1ULL << 30) / (double)(1ULL << 30) / (double)(1ULL << 6), eph.af2) ||
            !readSigned(22, 1.0 / (double)(1ULL << 30) / (double)(1ULL << 20), eph.af1) ||
            !readSigned(24, 1.0 / (double)(1ULL << 30) / (double)(1ULL << 3), eph.af0) ||
            !cursor.skip(5) ||
            !readSigned(18, 1.0 / (double)(1ULL << 6), eph.crs) ||
            !readSigned(16, r2rPi / (double)(1ULL << 30) / (double)(1ULL << 13), eph.deltaN) ||
            !readSigned(32, r2rPi / (double)(1ULL << 30) / (double)(1ULL << 1), eph.M0) ||
            !readSigned(18, 1.0 / (double)(1ULL << 30) / (double)(1ULL << 1), eph.cuc) ||
            !readUnsignedScaled(32, 1.0 / (double)(1ULL << 30) / (double)(1ULL << 3), eph.e) ||
            !readSigned(18, 1.0 / (double)(1ULL << 30) / (double)(1ULL << 1), eph.cus) ||
            !readUnsignedScaled(32, 1.0 / (double)(1ULL << 19), eph.sqrtA)) {
            return false;
        }

        quint64 toeRaw = 0;
        if (!cursor.readUnsigned(17, toeRaw)) {
            return false;
        }
        const double toeSec = static_cast<double>(toeRaw << 3);

        if (!readSigned(18, 1.0 / (double)(1ULL << 30) / (double)(1ULL << 1), eph.cic) ||
            !readSigned(32, r2rPi / (double)(1ULL << 30) / (double)(1ULL << 1), eph.omega0) ||
            !readSigned(18, 1.0 / (double)(1ULL << 30) / (double)(1ULL << 1), eph.cis) ||
            !readSigned(32, r2rPi / (double)(1ULL << 30) / (double)(1ULL << 1), eph.i0) ||
            !readSigned(18, 1.0 / (double)(1ULL << 6), eph.crc) ||
            !readSigned(32, r2rPi / (double)(1ULL << 30) / (double)(1ULL << 1), eph.omega) ||
            !readSigned(24, r2rPi / (double)(1ULL << 30) / (double)(1ULL << 13), eph.omegaDot) ||
            !readSigned(10, 1e-10, eph.tgd1) ||
            !readSigned(10, 1e-10, eph.tgd2)) {
            return false;
        }

        quint64 satH1 = 0;
        if (!cursor.readUnsigned(1, satH1)) {
            return false;
        }

        const int gpsSec = static_cast<int>((toeSec + 14.0));
        eph.iod = (gpsSec / 720) % 240;
        eph.health = static_cast<int>(satH1);

        eph.sat.system = QLatin1Char('C');
        eph.sat.number = static_cast<int>(prn);
        eph.sat.navType = navTypeFromSsrStandard(eph.sat.system, eph.sat.number);
        eph.messageType = type;
        {
            const int currentBdsWeek = (referenceWeek > 0 ? referenceWeek : currentGpsWeek()) - 1356;
            eph.bdsWeek = expandWeekRollover(static_cast<int>(week), 13, currentBdsWeek);
        }
        eph.tocSec = tocSec;
        eph.toeSec = toeSec;
        eph.validOrbit = true;
        return true;
    }
    case 1045:
    case 1046: {
        quint64 prn = 0;
        quint64 week = 0;
        quint64 iodNav = 0;
        quint64 sisa = 0;
        quint64 tocRaw = 0;
        qint64 idotRaw = 0;
        if (!cursor.readUnsigned(6, prn) ||
            !cursor.readUnsigned(12, week) ||
            !cursor.readUnsigned(10, iodNav) ||
            !cursor.readUnsigned(8, sisa) ||
            !cursor.readSigned(14, idotRaw) ||
            !cursor.readUnsigned(14, tocRaw)) {
            return false;
        }

        auto readSigned = [&cursor](int bits, double scale, double& out) -> bool {
            qint64 raw = 0;
            if (!cursor.readSigned(bits, raw)) return false;
            out = static_cast<double>(raw) * scale;
            return true;
        };
        auto readUnsignedScaled = [&cursor](int bits, double scale, double& out) -> bool {
            quint64 raw = 0;
            if (!cursor.readUnsigned(bits, raw)) return false;
            out = static_cast<double>(raw) * scale;
            return true;
        };

        const double r2rPi = kPi;
        eph.idot = static_cast<double>(idotRaw) * r2rPi / (double)(1ULL << 30) / (double)(1ULL << 13);
        const double tocSec = static_cast<double>(tocRaw) * 60.0;

        if (!readSigned(6, 1.0 / (double)(1ULL << 30) / (double)(1ULL << 29), eph.af2) ||
            !readSigned(21, 1.0 / (double)(1ULL << 30) / (double)(1ULL << 16), eph.af1) ||
            !readSigned(31, 1.0 / (double)(1ULL << 30) / (double)(1ULL << 4), eph.af0) ||
            !readSigned(16, 1.0 / (double)(1ULL << 5), eph.crs) ||
            !readSigned(16, r2rPi / (double)(1ULL << 30) / (double)(1ULL << 13), eph.deltaN) ||
            !readSigned(32, r2rPi / (double)(1ULL << 30) / (double)(1ULL << 1), eph.M0) ||
            !readSigned(16, 1.0 / (double)(1ULL << 29), eph.cuc) ||
            !readUnsignedScaled(32, 1.0 / (double)(1ULL << 30) / (double)(1ULL << 3), eph.e) ||
            !readSigned(16, 1.0 / (double)(1ULL << 29), eph.cus) ||
            !readUnsignedScaled(32, 1.0 / (double)(1ULL << 19), eph.sqrtA)) {
            return false;
        }

        quint64 toeRaw = 0;
        if (!cursor.readUnsigned(14, toeRaw)) {
            return false;
        }
        const double toeSec = static_cast<double>(toeRaw) * 60.0;

        if (!readSigned(16, 1.0 / (double)(1ULL << 29), eph.cic) ||
            !readSigned(32, r2rPi / (double)(1ULL << 30) / (double)(1ULL << 1), eph.omega0) ||
            !readSigned(16, 1.0 / (double)(1ULL << 29), eph.cis) ||
            !readSigned(32, r2rPi / (double)(1ULL << 30) / (double)(1ULL << 1), eph.i0) ||
            !readSigned(16, 1.0 / (double)(1ULL << 5), eph.crc) ||
            !readSigned(32, r2rPi / (double)(1ULL << 30) / (double)(1ULL << 1), eph.omega) ||
            !readSigned(24, r2rPi / (double)(1ULL << 30) / (double)(1ULL << 13), eph.omegaDot) ||
            !readSigned(10, 1.0 / (double)(1ULL << 30) / (double)(1ULL << 2), eph.tgd1)) {
            return false;
        }

        if (type == 1046) {
            if (!readSigned(10, 1.0 / (double)(1ULL << 30) / (double)(1ULL << 2), eph.tgd2)) {
                return false;
            }
            if (!cursor.skip(2) ||
                !cursor.skip(1) ||
                !cursor.skip(2) ||
                !cursor.skip(1)) {
                return false;
            }
        } else {
            if (!cursor.skip(2) ||
                !cursor.skip(1)) {
                return false;
            }
        }

        eph.sat.system = QLatin1Char('E');
        eph.sat.number = static_cast<int>(prn);
        eph.sat.navType = navTypeFromSsrStandard(eph.sat.system, eph.sat.number);
        eph.iod = static_cast<int>(iodNav);
        eph.messageType = type;
        {
            const int currentGalWeek = (referenceWeek > 0 ? referenceWeek : currentGpsWeek()) - 1024;
            eph.week = expandWeekRollover(static_cast<int>(week), 12, currentGalWeek);
        }
        eph.tocSec = tocSec;
        eph.toeSec = toeSec;
        eph.validOrbit = true;
        return true;
    }
    default:
        break;
    }
    return false;
}

bool parseSsrMessage(int type, const QByteArray& payload, RtcmSsrMessage& out) {
    if (type == 4076) {
        QChar system;
        RtcmSsrCorrectionKind kind = RtcmSsrCorrectionKind::Unknown;
        if (!readIgsSsrSystemAndKind(payload, system, kind)) {
            return false;
        }

        BitCursor cursor(payload);
        if (!cursor.skip(12 + 3 + 8)) {
            return false;
        }
        quint64 epoch = 0;
        quint64 updateInterval = 0;
        quint64 multiple = 0;
        quint64 ssrIod = 0;
        quint64 provider = 0;
        quint64 solution = 0;
        quint64 satRefDatum = 0;
        quint64 disp = 0;
        quint64 mw = 0;
        quint64 satCount = 0;

        if (!cursor.readUnsigned(20, epoch) ||
            !cursor.readUnsigned(4, updateInterval) ||
            !cursor.readUnsigned(1, multiple) ||
            !cursor.readUnsigned(4, ssrIod) ||
            !cursor.readUnsigned(16, provider) ||
            !cursor.readUnsigned(4, solution)) {
            return false;
        }

        if (kind == RtcmSsrCorrectionKind::Orbit || kind == RtcmSsrCorrectionKind::OrbitClock) {
            if (!cursor.readUnsigned(1, satRefDatum)) {
                return false;
            }
        }

        if (kind == RtcmSsrCorrectionKind::PhaseBias) {
            if (!cursor.readUnsigned(1, disp) ||
                !cursor.readUnsigned(1, mw)) {
                return false;
            }
        }

        if (!cursor.readUnsigned(6, satCount)) {
            return false;
        }

        out.kind = kind;
        out.system = system;
        out.messageType = type;
        out.epochTime = static_cast<int>(epoch);
        out.updateIntervalIndex = static_cast<int>(updateInterval);
        out.multipleMessage = (multiple != 0);
        out.ssrIod = static_cast<int>(ssrIod);
        out.providerId = static_cast<int>(provider);
        out.solutionId = static_cast<int>(solution);
        out.satRefDatum = (kind == RtcmSsrCorrectionKind::Orbit || kind == RtcmSsrCorrectionKind::OrbitClock)
            ? static_cast<int>(satRefDatum) : -1;
        if (kind == RtcmSsrCorrectionKind::PhaseBias) {
            out.dispersiveBiasConsistency = static_cast<int>(disp);
            out.mwConsistency = static_cast<int>(mw);
        }

        out.satellites.reserve(static_cast<int>(satCount));
        for (quint64 i = 0; i < satCount; ++i) {
            RtcmSsrSatelliteData satData;
            quint64 satId = 0;
            if (!cursor.readUnsigned(6, satId)) {
                return false;
            }
            satData.sat.system = system;
            satData.sat.number = static_cast<int>(satId);
            satData.sat.navType = navTypeFromSsrStandard(system, satData.sat.number);

            if (kind == RtcmSsrCorrectionKind::Orbit || kind == RtcmSsrCorrectionKind::OrbitClock) {
                quint64 iod = 0;
                if (!cursor.readUnsigned(8, iod)) {
                    return false;
                }
                satData.iod = static_cast<int>(iod);
                qint64 raw = 0;
                if (!cursor.readSigned(22, raw)) return false;
                satData.deltaRadial = static_cast<double>(raw) / 10000.0;
                if (!cursor.readSigned(20, raw)) return false;
                satData.deltaAlongTrack = static_cast<double>(raw) / 2500.0;
                if (!cursor.readSigned(20, raw)) return false;
                satData.deltaCrossTrack = static_cast<double>(raw) / 2500.0;
                if (!cursor.readSigned(21, raw)) return false;
                satData.dotDeltaRadial = static_cast<double>(raw) / 1000000.0;
                if (!cursor.readSigned(19, raw)) return false;
                satData.dotDeltaAlongTrack = static_cast<double>(raw) / 250000.0;
                if (!cursor.readSigned(19, raw)) return false;
                satData.dotDeltaCrossTrack = static_cast<double>(raw) / 250000.0;
                satData.hasOrbit = true;
            }

            if (kind == RtcmSsrCorrectionKind::Clock || kind == RtcmSsrCorrectionKind::OrbitClock) {
                qint64 raw = 0;
                if (!cursor.readSigned(22, raw)) return false;
                satData.deltaC0 = static_cast<double>(raw) / 10000.0;
                if (!cursor.readSigned(21, raw)) return false;
                satData.deltaC1 = static_cast<double>(raw) / 1000000.0;
                if (!cursor.readSigned(27, raw)) return false;
                satData.deltaC2 = static_cast<double>(raw) / 50000000.0;
                satData.hasClock = true;
            }

            if (kind == RtcmSsrCorrectionKind::HighRateClock) {
                qint64 raw = 0;
                if (!cursor.readSigned(22, raw)) return false;
                satData.highRateClock = static_cast<double>(raw) / 10000.0;
                satData.hasHighRateClock = true;
            }

            if (kind == RtcmSsrCorrectionKind::Ura) {
                quint64 ura = 0;
                if (!cursor.readUnsigned(6, ura)) return false;
            }

            if (kind == RtcmSsrCorrectionKind::CodeBias) {
                quint64 nBias = 0;
                if (!cursor.readUnsigned(5, nBias)) {
                    return false;
                }
                satData.codeBiases.reserve(static_cast<int>(nBias));
                for (quint64 ib = 0; ib < nBias; ++ib) {
                    quint64 sig = 0;
                    qint64 raw = 0;
                    if (!cursor.readUnsigned(5, sig) || !cursor.readSigned(14, raw)) {
                        return false;
                    }
                    RtcmSsrCodeBiasEntry bias;
                    bias.signalCode = static_cast<int>(sig);
                    bias.biasMeters = static_cast<double>(raw) / 100.0;
                    satData.codeBiases.push_back(bias);
                }
            }

            if (kind == RtcmSsrCorrectionKind::PhaseBias) {
                quint64 nBias = 0;
                quint64 yaw = 0;
                qint64 yawRate = 0;
                if (!cursor.readUnsigned(5, nBias) ||
                    !cursor.readUnsigned(9, yaw) ||
                    !cursor.readSigned(8, yawRate)) {
                    return false;
                }
                satData.yawAngleRad = static_cast<double>(yaw) * kPi / 256.0;
                satData.yawRateRadPerSec = static_cast<double>(yawRate) * kPi / 8192.0;
                satData.phaseBiases.reserve(static_cast<int>(nBias));
                for (quint64 ib = 0; ib < nBias; ++ib) {
                    quint64 sig = 0;
                    quint64 integerIndicator = 0;
                    quint64 wideLaneIndicator = 0;
                    quint64 discon = 0;
                    qint64 rawBias = 0;
                    if (!cursor.readUnsigned(5, sig) ||
                        !cursor.readUnsigned(1, integerIndicator) ||
                        !cursor.readUnsigned(2, wideLaneIndicator) ||
                        !cursor.readUnsigned(4, discon) ||
                        !cursor.readSigned(20, rawBias)) {
                        return false;
                    }
                    RtcmSsrPhaseBiasEntry bias;
                    bias.signalCode = static_cast<int>(sig);
                    bias.signalIntegerIndicator = static_cast<int>(integerIndicator);
                    bias.wideLaneIntegerIndicator = static_cast<int>(wideLaneIndicator);
                    bias.discontinuityCounter = static_cast<int>(discon);
                    bias.biasMeters = static_cast<double>(rawBias) / 10000.0;
                    satData.phaseBiases.push_back(bias);
                }
            }

            out.satellites.push_back(satData);
        }

        return true;
    }

    QChar system;
    RtcmSsrCorrectionKind kind = RtcmSsrCorrectionKind::Unknown;
    if (!readSsrSystemAndKind(type, system, kind)) {
        return false;
    }
    if (kind == RtcmSsrCorrectionKind::Unknown) {
        return false;
    }

    BitCursor cursor(payload);
    quint64 mt = 0;
    if (!cursor.readUnsigned(12, mt)) {
        return false;
    }

    const int epochBits = (system == QLatin1Char('R')) ? 17 : 20;
    quint64 epoch = 0;
    quint64 ui = 0;
    quint64 mmi = 0;
    quint64 ssrIod = 0;
    quint64 provider = 0;
    quint64 solution = 0;
    quint64 satRefDatum = 0;
    quint64 satCount = 0;

    if (!cursor.readUnsigned(epochBits, epoch) ||
        !cursor.readUnsigned(4, ui) ||
        !cursor.readUnsigned(1, mmi)) {
        return false;
    }

    const bool withRefDatum = (kind == RtcmSsrCorrectionKind::Orbit ||
                               kind == RtcmSsrCorrectionKind::OrbitClock);
    if (withRefDatum) {
        if (!cursor.readUnsigned(1, satRefDatum)) {
            return false;
        }
    }

    if (!cursor.readUnsigned(4, ssrIod) ||
        !cursor.readUnsigned(16, provider) ||
        !cursor.readUnsigned(4, solution)) {
        return false;
    }

    quint64 dispersive = 0;
    quint64 mw = 0;
    if (kind == RtcmSsrCorrectionKind::PhaseBias) {
        if (!cursor.readUnsigned(1, dispersive) ||
            !cursor.readUnsigned(1, mw)) {
            return false;
        }
    }

    if (!cursor.readUnsigned(6, satCount)) {
        return false;
    }

    out.kind = kind;
    out.system = system;
    out.messageType = type;
    out.epochTime = static_cast<int>(epoch);
    out.updateIntervalIndex = static_cast<int>(ui);
    out.multipleMessage = (mmi != 0);
    out.ssrIod = static_cast<int>(ssrIod);
    out.providerId = static_cast<int>(provider);
    out.solutionId = static_cast<int>(solution);
    out.satRefDatum = withRefDatum ? static_cast<int>(satRefDatum) : -1;
    if (kind == RtcmSsrCorrectionKind::PhaseBias) {
        out.dispersiveBiasConsistency = static_cast<int>(dispersive);
        out.mwConsistency = static_cast<int>(mw);
    }

    out.satellites.reserve(static_cast<int>(satCount));
    const int satIdBits = satIdBitsForSystem(system);

    for (quint64 i = 0; i < satCount; ++i) {
        RtcmSsrSatelliteData satData;
        quint64 satId = 0;
        if (!cursor.readUnsigned(satIdBits, satId)) {
            return false;
        }
        satData.sat.system = system;
        satData.sat.number = static_cast<int>(satId);
        satData.sat.navType = navTypeFromSsrStandard(system, satData.sat.number);

        if (kind == RtcmSsrCorrectionKind::Orbit ||
            kind == RtcmSsrCorrectionKind::OrbitClock) {
            quint64 iod = 0;
            if (system == QLatin1Char('E')) {
                if (!cursor.readUnsigned(10, iod)) {
                    return false;
                }
            } else if (system == QLatin1Char('S')) {
                quint64 toeMod = 0;
                quint64 iodCrc = 0;
                if (!cursor.readUnsigned(9, toeMod) ||
                    !cursor.readUnsigned(24, iodCrc)) {
                    return false;
                }
                satData.toe = static_cast<int>(toeMod * 16);
                satData.iod = static_cast<int>(iodCrc);
            } else if (system == QLatin1Char('C')) {
                quint64 toeMod = 0;
                if (!cursor.readUnsigned(10, toeMod) ||
                    !cursor.readUnsigned(8, iod)) {
                    return false;
                }
                satData.toe = static_cast<int>(toeMod * 8);
            } else {
                if (!cursor.readUnsigned(8, iod)) {
                    return false;
                }
            }
            if (satData.iod < 0) {
                satData.iod = static_cast<int>(iod);
            }

            qint64 raw = 0;
            if (!cursor.readSigned(22, raw)) return false;
            satData.deltaRadial = static_cast<double>(raw) / 10000.0;
            if (!cursor.readSigned(20, raw)) return false;
            satData.deltaAlongTrack = static_cast<double>(raw) / 2500.0;
            if (!cursor.readSigned(20, raw)) return false;
            satData.deltaCrossTrack = static_cast<double>(raw) / 2500.0;
            if (!cursor.readSigned(21, raw)) return false;
            satData.dotDeltaRadial = static_cast<double>(raw) / 1000000.0;
            if (!cursor.readSigned(19, raw)) return false;
            satData.dotDeltaAlongTrack = static_cast<double>(raw) / 250000.0;
            if (!cursor.readSigned(19, raw)) return false;
            satData.dotDeltaCrossTrack = static_cast<double>(raw) / 250000.0;
            satData.hasOrbit = true;
        }

        if (kind == RtcmSsrCorrectionKind::Clock ||
            kind == RtcmSsrCorrectionKind::OrbitClock) {
            qint64 raw = 0;
            if (!cursor.readSigned(22, raw)) return false;
            satData.deltaC0 = static_cast<double>(raw) / 10000.0;
            if (!cursor.readSigned(21, raw)) return false;
            satData.deltaC1 = static_cast<double>(raw) / 1000000.0;
            if (!cursor.readSigned(27, raw)) return false;
            satData.deltaC2 = static_cast<double>(raw) / 50000000.0;
            satData.hasClock = true;
        }

        if (kind == RtcmSsrCorrectionKind::HighRateClock) {
            qint64 raw = 0;
            if (!cursor.readSigned(22, raw)) return false;
            satData.highRateClock = static_cast<double>(raw) / 10000.0;
            satData.hasHighRateClock = true;
        }

        if (kind == RtcmSsrCorrectionKind::Ura) {
            quint64 ura = 0;
            if (!cursor.readUnsigned(6, ura)) return false;
        }

        if (kind == RtcmSsrCorrectionKind::CodeBias) {
            quint64 nBias = 0;
            if (!cursor.readUnsigned(5, nBias)) {
                return false;
            }
            satData.codeBiases.reserve(static_cast<int>(nBias));
            for (quint64 ib = 0; ib < nBias; ++ib) {
                quint64 sig = 0;
                qint64 raw = 0;
                if (!cursor.readUnsigned(5, sig) || !cursor.readSigned(14, raw)) {
                    return false;
                }
                RtcmSsrCodeBiasEntry bias;
                bias.signalCode = static_cast<int>(sig);
                bias.biasMeters = static_cast<double>(raw) / 100.0;
                satData.codeBiases.push_back(bias);
            }
        }

        if (kind == RtcmSsrCorrectionKind::PhaseBias) {
            quint64 nBias = 0;
            quint64 yaw = 0;
            qint64 yawRate = 0;
            if (!cursor.readUnsigned(5, nBias) ||
                !cursor.readUnsigned(9, yaw) ||
                !cursor.readSigned(8, yawRate)) {
                return false;
            }
            satData.yawAngleRad = static_cast<double>(yaw) * kPi / 256.0;
            satData.yawRateRadPerSec = static_cast<double>(yawRate) * kPi / 8192.0;
            satData.phaseBiases.reserve(static_cast<int>(nBias));
            for (quint64 ib = 0; ib < nBias; ++ib) {
                quint64 sig = 0;
                quint64 integerIndicator = 0;
                quint64 wideLaneIndicator = 0;
                quint64 discon = 0;
                qint64 rawBias = 0;
                if (!cursor.readUnsigned(5, sig) ||
                    !cursor.readUnsigned(1, integerIndicator) ||
                    !cursor.readUnsigned(2, wideLaneIndicator) ||
                    !cursor.readUnsigned(4, discon) ||
                    !cursor.readSigned(20, rawBias)) {
                    return false;
                }
                RtcmSsrPhaseBiasEntry bias;
                bias.signalCode = static_cast<int>(sig);
                bias.signalIntegerIndicator = static_cast<int>(integerIndicator);
                bias.wideLaneIntegerIndicator = static_cast<int>(wideLaneIndicator);
                bias.discontinuityCounter = static_cast<int>(discon);
                bias.biasMeters = static_cast<double>(rawBias) / 10000.0;
                satData.phaseBiases.push_back(bias);
            }
        }

        out.satellites.push_back(satData);
    }

    return true;
}

bool parseVtecMessage(int type, const QByteArray& payload, RtcmVtecMessage& out) {
    if (type != 1264 && type != 4076) {
        return false;
    }

    BitCursor cursor(payload);
    quint64 messageType = 0;
    if (!cursor.readUnsigned(12, messageType)) {
        return false;
    }
    if (type == 4076) {
        quint64 version = 0;
        quint64 igsType = 0;
        if (!cursor.readUnsigned(3, version) ||
            !cursor.readUnsigned(8, igsType) ||
            igsType != 201) {
            return false;
        }
    }

    quint64 epoch = 0;
    quint64 ui = 0;
    quint64 mmi = 0;
    quint64 ssrIod = 0;
    quint64 provider = 0;
    quint64 solution = 0;
    quint64 qualityRaw = 0;
    quint64 layerCountRaw = 0;

    if (!cursor.readUnsigned(20, epoch) ||
        !cursor.readUnsigned(4, ui) ||
        !cursor.readUnsigned(1, mmi) ||
        !cursor.readUnsigned(4, ssrIod) ||
        !cursor.readUnsigned(16, provider) ||
        !cursor.readUnsigned(4, solution) ||
        !cursor.readUnsigned(9, qualityRaw) ||
        !cursor.readUnsigned(2, layerCountRaw)) {
        return false;
    }

    out.messageType = type;
    out.epochTime = static_cast<int>(epoch);
    out.updateIntervalIndex = static_cast<int>(ui);
    out.multipleMessage = (mmi != 0);
    out.ssrIod = static_cast<int>(ssrIod);
    out.providerId = static_cast<int>(provider);
    out.solutionId = static_cast<int>(solution);
    out.quality = static_cast<double>(qualityRaw) / 20.0;

    const int layerCount = static_cast<int>(layerCountRaw) + 1;
    out.layers.reserve(layerCount);
    for (int l = 0; l < layerCount; ++l) {
        quint64 heightRaw = 0;
        quint64 degreeRaw = 0;
        quint64 orderRaw = 0;
        if (!cursor.readUnsigned(8, heightRaw) ||
            !cursor.readUnsigned(4, degreeRaw) ||
            !cursor.readUnsigned(4, orderRaw)) {
            return false;
        }
        const int degree = static_cast<int>(degreeRaw) + 1;
        const int order = static_cast<int>(orderRaw) + 1;
        if (degree < 0 || order < 0 || degree > 32 || order > 32) {
            return false;
        }

        RtcmVtecLayer layer;
        layer.heightMeters = static_cast<double>(heightRaw) * 10000.0;
        layer.cosine = QVector<QVector<double>>(degree + 1, QVector<double>(order + 1, 0.0));
        layer.sine = QVector<QVector<double>>(degree + 1, QVector<double>(order + 1, 0.0));

        for (int o = 0; o <= order; ++o) {
            for (int d = o; d <= degree; ++d) {
                qint64 raw = 0;
                if (!cursor.readSigned(16, raw)) {
                    return false;
                }
                layer.cosine[d][o] = static_cast<double>(raw) / 200.0;
            }
        }
        for (int o = 1; o <= order; ++o) {
            for (int d = o; d <= degree; ++d) {
                qint64 raw = 0;
                if (!cursor.readSigned(16, raw)) {
                    return false;
                }
                layer.sine[d][o] = static_cast<double>(raw) / 200.0;
            }
        }

        out.layers.push_back(layer);
    }

    return !out.layers.isEmpty();
}

double lockTimeIndicatorToSeconds(int messageType, int lti) {
    const int subtype = messageType % 10;
    if (subtype == 2 || subtype == 3 || subtype == 4 || subtype == 5) {
        switch (lti) {
        case 0: return 0.0;
        case 1: return 32e-3;
        case 2: return 64e-3;
        case 3: return 128e-3;
        case 4: return 256e-3;
        case 5: return 512e-3;
        case 6: return 1024e-3;
        case 7: return 2048e-3;
        case 8: return 4096e-3;
        case 9: return 8192e-3;
        case 10: return 16384e-3;
        case 11: return 32768e-3;
        case 12: return 65536e-3;
        case 13: return 131072e-3;
        case 14: return 262144e-3;
        case 15: return 524288e-3;
        default: return -1.0;
        }
    }
    if (subtype == 6 || subtype == 7) {
        if (lti < 0) return -1.0;
        if (lti < 64) return (1.0 * lti) * 1e-3;
        if (lti < 96) return (2.0 * lti - 64.0) * 1e-3;
        if (lti < 128) return (4.0 * lti - 256.0) * 1e-3;
        if (lti < 160) return (8.0 * lti - 768.0) * 1e-3;
        if (lti < 192) return (16.0 * lti - 2048.0) * 1e-3;
        if (lti < 224) return (32.0 * lti - 5120.0) * 1e-3;
        if (lti < 256) return (64.0 * lti - 12288.0) * 1e-3;
        if (lti < 288) return (128.0 * lti - 28672.0) * 1e-3;
        if (lti < 320) return (256.0 * lti - 65536.0) * 1e-3;
        if (lti < 352) return (512.0 * lti - 147456.0) * 1e-3;
        if (lti < 384) return (1024.0 * lti - 327680.0) * 1e-3;
        if (lti < 416) return (2048.0 * lti - 720896.0) * 1e-3;
        if (lti < 448) return (4096.0 * lti - 1572864.0) * 1e-3;
        if (lti < 480) return (8192.0 * lti - 3407872.0) * 1e-3;
        if (lti < 512) return (16384.0 * lti - 7340032.0) * 1e-3;
        if (lti < 544) return (32768.0 * lti - 15728640.0) * 1e-3;
        if (lti < 576) return (65536.0 * lti - 33554432.0) * 1e-3;
        if (lti < 608) return (131072.0 * lti - 71303168.0) * 1e-3;
        if (lti < 640) return (262144.0 * lti - 150994944.0) * 1e-3;
        if (lti < 672) return (524288.0 * lti - 318767104.0) * 1e-3;
        if (lti < 704) return (1048576.0 * lti - 671088640.0) * 1e-3;
        if (lti == 704) return (2097152.0 * lti - 1409286144.0) * 1e-3;
        return -1.0;
    }
    return -1.0;
}

bool parseMsmMessage(int type, const QByteArray& payload, RtcmMsmMessage& out) {
    if (type < 1071 || type > 1137) {
        return false;
    }
    const int subtype = type % 10;
    if (subtype < 4 || subtype > 7) {
        return false;
    }

    BitCursor cursor(payload);
    quint64 messageType = 0;
    quint64 stationId = 0;
    if (!cursor.readUnsigned(12, messageType) ||
        !cursor.readUnsigned(12, stationId)) {
        return false;
    }

    QChar system;
    if (type >= 1071 && type <= 1077) system = QLatin1Char('G');
    else if (type >= 1081 && type <= 1087) system = QLatin1Char('R');
    else if (type >= 1091 && type <= 1097) system = QLatin1Char('E');
    else if (type >= 1101 && type <= 1107) system = QLatin1Char('S');
    else if (type >= 1111 && type <= 1117) system = QLatin1Char('J');
    else if (type >= 1121 && type <= 1127) system = QLatin1Char('C');
    else if (type >= 1131 && type <= 1137) system = QLatin1Char('I');
    else return false;

    quint64 epoch = 0;
    quint64 glonassDay = 0;
    if (system == QLatin1Char('R')) {
        if (!cursor.readUnsigned(3, glonassDay) ||
            !cursor.readUnsigned(27, epoch)) {
            return false;
        }
    } else {
        if (!cursor.readUnsigned(30, epoch)) {
            return false;
        }
    }

    quint64 multiple = 0;
    quint64 iods = 0;
    quint64 reserved = 0;
    quint64 clockSteering = 0;
    quint64 externalClock = 0;
    quint64 smoothing = 0;
    quint64 smoothingInt = 0;
    if (!cursor.readUnsigned(1, multiple) ||
        !cursor.readUnsigned(3, iods) ||
        !cursor.readUnsigned(7, reserved) ||
        !cursor.readUnsigned(2, clockSteering) ||
        !cursor.readUnsigned(2, externalClock) ||
        !cursor.readUnsigned(1, smoothing) ||
        !cursor.readUnsigned(3, smoothingInt)) {
        return false;
    }

    QVector<int> satIds;
    satIds.reserve(32);
    for (int bit = 0; bit < 64; ++bit) {
        quint64 satMaskBit = 0;
        if (!cursor.readUnsigned(1, satMaskBit)) {
            return false;
        }
        if (satMaskBit) {
            satIds.push_back(bit + 1);
        }
    }

    QVector<int> sigIds;
    sigIds.reserve(16);
    for (int bit = 0; bit < 32; ++bit) {
        quint64 sigMaskBit = 0;
        if (!cursor.readUnsigned(1, sigMaskBit)) {
            return false;
        }
        if (sigMaskBit) {
            sigIds.push_back(bit + 1);
        }
    }

    const int numSat = satIds.size();
    const int numSig = sigIds.size();
    const int numCells = numSat * numSig;
    if (numCells <= 0) {
        out.system = system;
        out.messageType = type;
        out.msmSubtype = subtype;
        out.stationId = static_cast<int>(stationId);
        out.epochTime = static_cast<int>(epoch);
        out.glonassDay = static_cast<int>(glonassDay);
        out.multipleMessage = (multiple != 0);
        out.iods = static_cast<int>(iods);
        out.clockSteeringIndicator = static_cast<int>(clockSteering);
        out.externalClockIndicator = static_cast<int>(externalClock);
        out.smoothingIndicator = (smoothing != 0);
        out.smoothingInterval = static_cast<int>(smoothingInt);
        out.signalIds = sigIds;
        return true;
    }

    QVector<bool> activeCells(numCells, false);
    int activeCount = 0;
    for (int cell = 0; cell < numCells; ++cell) {
        quint64 active = 0;
        if (!cursor.readUnsigned(1, active)) {
            return false;
        }
        activeCells[cell] = (active != 0);
        if (activeCells[cell]) {
            ++activeCount;
        }
    }

    QVector<RtcmMsmSatelliteData> satData(numSat);
    for (int i = 0; i < numSat; ++i) {
        satData[i].satId = satIds[i];
        satData[i].roughRangeIntegerMillis = -1;
    }

    if (subtype == 4 || subtype == 5 || subtype == 6 || subtype == 7) {
        for (int i = 0; i < numSat; ++i) {
            quint64 rrInt = 0;
            if (!cursor.readUnsigned(8, rrInt)) return false;
            satData[i].roughRangeIntegerMillis = (rrInt == 255ULL) ? -1 : static_cast<int>(rrInt);
        }
        if (subtype == 5 || subtype == 7) {
            for (int i = 0; i < numSat; ++i) {
                quint64 ext = 0;
                if (!cursor.readUnsigned(4, ext)) return false;
                satData[i].extendedInfo = static_cast<int>(ext);
            }
        }
        for (int i = 0; i < numSat; ++i) {
            quint64 rrMod = 0;
            if (!cursor.readUnsigned(10, rrMod)) return false;
            satData[i].roughRangeModuloMillis = static_cast<double>(rrMod) / 1024.0;
        }
        if (subtype == 5 || subtype == 7) {
            for (int i = 0; i < numSat; ++i) {
                qint64 rdop = 0;
                if (!cursor.readSigned(14, rdop)) return false;
                satData[i].roughPhaseRangeRate = static_cast<int>(rdop);
            }
        }
    }

    QVector<RtcmMsmCellData> cells;
    cells.reserve(activeCount);
    for (int satIndex = 0; satIndex < numSat; ++satIndex) {
        for (int sigIndex = 0; sigIndex < numSig; ++sigIndex) {
            const int idx = satIndex * numSig + sigIndex;
            if (!activeCells[idx]) {
                continue;
            }
            RtcmMsmCellData cell;
            cell.active = true;
            cell.satId = satIds[satIndex];
            cell.signalId = sigIds[sigIndex];
            cells.push_back(cell);
        }
    }

    auto readPerActiveSignedNullable = [&cursor](QVector<RtcmMsmCellData>& target, int bits,
                                                  qint64 invalidRaw, double scale,
                                                  auto setter) -> bool {
        for (RtcmMsmCellData& cell : target) {
            qint64 raw = 0;
            if (!cursor.readSigned(bits, raw)) {
                return false;
            }
            if (raw == invalidRaw) {
                setter(cell, false, 0.0);
            } else {
                setter(cell, true, static_cast<double>(raw) * scale);
            }
        }
        return true;
    };
    auto readPerActiveUnsigned = [&cursor](QVector<RtcmMsmCellData>& target, int bits, auto setter) -> bool {
        for (RtcmMsmCellData& cell : target) {
            quint64 raw = 0;
            if (!cursor.readUnsigned(bits, raw)) {
                return false;
            }
            setter(cell, raw);
        }
        return true;
    };

    if (subtype == 4 || subtype == 5) {
        if (!readPerActiveSignedNullable(cells, 15, -(1LL << 14), 1.0 / static_cast<double>(1 << 24),
                                 [](RtcmMsmCellData& cell, bool valid, double v) {
                                     cell.hasFinePseudorange = valid;
                                     cell.finePseudorangeMillis = valid ? v : 0.0;
                                 })) return false;
        if (!readPerActiveSignedNullable(cells, 22, -(1LL << 21), 1.0 / static_cast<double>(1 << 29),
                                 [](RtcmMsmCellData& cell, bool valid, double v) {
                                     cell.hasFinePhaseRange = valid;
                                     cell.finePhaseRangeMillis = valid ? v : 0.0;
                                 })) return false;
        if (!readPerActiveUnsigned(cells, 4,
                                   [type](RtcmMsmCellData& cell, quint64 v) {
                                       cell.hasLockTime = true;
                                       cell.lockTimeIndicator = static_cast<int>(v);
                                       cell.lockTimeSec = lockTimeIndicatorToSeconds(type, cell.lockTimeIndicator);
                                   })) return false;
        if (!readPerActiveUnsigned(cells, 1,
                                   [](RtcmMsmCellData& cell, quint64 v) {
                                       cell.hasHalfCycle = true;
                                       cell.halfCycleAmbiguity = static_cast<int>(v);
                                   })) return false;
        if (!readPerActiveUnsigned(cells, 6,
                                   [](RtcmMsmCellData& cell, quint64 v) {
                                       cell.hasCnr = true;
                                       cell.cnrDbHz = static_cast<double>(v);
                                   })) return false;
        if (subtype == 5) {
            if (!readPerActiveSignedNullable(cells, 15, -(1LL << 14), 0.0001,
                                     [](RtcmMsmCellData& cell, bool valid, double v) {
                                         cell.hasFinePhaseRate = valid;
                                         cell.finePhaseRateMps = valid ? v : 0.0;
                                     })) return false;
        }
    } else if (subtype == 6 || subtype == 7) {
        if (!readPerActiveSignedNullable(cells, 20, -(1LL << 19), 1.0 / static_cast<double>(1 << 29),
                                 [](RtcmMsmCellData& cell, bool valid, double v) {
                                     cell.hasFinePseudorange = valid;
                                     cell.finePseudorangeMillis = valid ? v : 0.0;
                                 })) return false;
        if (!readPerActiveSignedNullable(cells, 24, -(1LL << 23), 1.0 / static_cast<double>(1U << 31),
                                 [](RtcmMsmCellData& cell, bool valid, double v) {
                                     cell.hasFinePhaseRange = valid;
                                     cell.finePhaseRangeMillis = valid ? v : 0.0;
                                 })) return false;
        if (!readPerActiveUnsigned(cells, 10,
                                   [type](RtcmMsmCellData& cell, quint64 v) {
                                       cell.hasLockTime = true;
                                       cell.lockTimeIndicator = static_cast<int>(v);
                                       cell.lockTimeSec = lockTimeIndicatorToSeconds(type, cell.lockTimeIndicator);
                                   })) return false;
        if (!readPerActiveUnsigned(cells, 1,
                                   [](RtcmMsmCellData& cell, quint64 v) {
                                       cell.hasHalfCycle = true;
                                       cell.halfCycleAmbiguity = static_cast<int>(v);
                                   })) return false;
        if (!readPerActiveUnsigned(cells, 10,
                                   [](RtcmMsmCellData& cell, quint64 v) {
                                       cell.hasCnr = true;
                                       cell.cnrDbHz = static_cast<double>(v) / 16.0;
                                   })) return false;
        if (subtype == 7) {
            if (!readPerActiveSignedNullable(cells, 15, -(1LL << 14), 0.0001,
                                     [](RtcmMsmCellData& cell, bool valid, double v) {
                                         cell.hasFinePhaseRate = valid;
                                         cell.finePhaseRateMps = valid ? v : 0.0;
                                     })) return false;
        }
    }

    out.system = system;
    out.messageType = type;
    out.msmSubtype = subtype;
    out.stationId = static_cast<int>(stationId);
    out.epochTime = static_cast<int>(epoch);
    out.glonassDay = static_cast<int>(glonassDay);
    out.multipleMessage = (multiple != 0);
    out.iods = static_cast<int>(iods);
    out.clockSteeringIndicator = static_cast<int>(clockSteering);
    out.externalClockIndicator = static_cast<int>(externalClock);
    out.smoothingIndicator = (smoothing != 0);
    out.smoothingInterval = static_cast<int>(smoothingInt);
    out.satellites = satData;
    out.signalIds = sigIds;
    out.cells = cells;
    return true;
}

} // namespace

RtcmDecodeBatch Rtcm3Decoder::pushBytes(const QByteArray& bytes) {
    RtcmDecodeBatch batch;
    if (bytes.isEmpty()) {
        return batch;
    }

    _buffer += bytes;

    while (true) {
        if (_buffer.size() < 3) {
            break;
        }

        const int preamblePos = _buffer.indexOf(char(0xD3));
        if (preamblePos < 0) {
            _buffer.clear();
            break;
        }
        if (preamblePos > 0) {
            _buffer.remove(0, preamblePos);
            if (_buffer.size() < 3) {
                break;
            }
        }

        const quint8 b1 = static_cast<quint8>(_buffer.at(1));
        const quint8 b2 = static_cast<quint8>(_buffer.at(2));
        const int payloadLength = ((b1 & 0x03) << 8) | b2;
        if (payloadLength < 0 || payloadLength > kMaxRtcmPayloadBytes) {
            ++batch.malformedFrames;
            _buffer.remove(0, 1);
            continue;
        }

        const int fullSize = payloadLength + 6;
        if (_buffer.size() < fullSize) {
            break;
        }

        const quint32 computedCrc = crc24q(_buffer.constData(), payloadLength + 3);
        const quint32 receivedCrc =
            (static_cast<quint32>(static_cast<quint8>(_buffer.at(payloadLength + 3))) << 16) |
            (static_cast<quint32>(static_cast<quint8>(_buffer.at(payloadLength + 4))) << 8) |
            static_cast<quint32>(static_cast<quint8>(_buffer.at(payloadLength + 5)));

        if (computedCrc != receivedCrc) {
            ++batch.crcErrors;
            _buffer.remove(0, 1);
            continue;
        }

        const QByteArray payload = _buffer.mid(3, payloadLength);
        RtcmDecodedMessage message = decodePayload(payload);
        message.payloadLength = payloadLength;
        _typeCounters[message.type] += 1;
        if (!message.ignored) {
            batch.messages.push_back(message);
        }

        _buffer.remove(0, fullSize);
    }

    if (_buffer.size() > kMaxBufferedBytes) {
        _buffer = _buffer.right(2048);
        ++batch.malformedFrames;
    }

    return batch;
}

void Rtcm3Decoder::reset() {
    _buffer.clear();
    _typeCounters.clear();
    _referenceGpsWeek = 0;
    _referenceGpsSow = 0.0;
    _referenceGpsTimeValid = false;
}

bool Rtcm3Decoder::hasBufferedData() const {
    return !_buffer.isEmpty();
}

QMap<int, quint64> Rtcm3Decoder::typeCounters() const {
    return _typeCounters;
}

RtcmDecodedMessage Rtcm3Decoder::decodePayload(const QByteArray& payload) {
    RtcmDecodedMessage out;
    if (payload.size() < 2) {
        out.summary = "payload too short";
        return out;
    }

    BitCursor cursor(payload);
    quint64 type = 0;
    if (!cursor.readUnsigned(12, type)) {
        out.summary = "cannot read message type";
        return out;
    }

    out.type = static_cast<int>(type);
    out.category = categoryForType(out.type);

    if (isMsm(out.type)) {
        out.summary = decodeMsmSummary(out.type, payload);
        out.hasMsm = parseMsmMessage(out.type, payload, out.msm);
        if (out.hasMsm) {
            const GnssUtils::GpsTime time = streamMsmGpsTime(
                out.msm, _referenceGpsWeek, _referenceGpsSow, _referenceGpsTimeValid);
            out.msm.gpsTimeValid = time.valid;
            out.msm.gpsWeek = time.week;
            out.msm.gpsSow = time.sow;
            if (time.valid) {
                _referenceGpsWeek = time.week;
                _referenceGpsSow = time.sow;
                _referenceGpsTimeValid = true;
            }
        }
        if (out.hasMsm && !isSupportedSolutionSystem(out.msm.system)) {
            out.ignored = true;
            out.summary = QString("ignored unsupported MSM system %1").arg(out.msm.system);
        }
    } else if (isLegacyGpsObs(out.type) || isLegacyGloObs(out.type)) {
        out.summary = decodeLegacyObsSummary(out.type, payload);
    } else {
        switch (out.type) {
        case 1005:
        case 1006:
            out.summary = decodeAntennaPositionSummary(out.type, payload);
            break;
        case 1007:
        case 1008:
        case 1033:
            out.summary = decodeAntennaReceiverSummary(out.type, payload);
            break;
        case 1019:
        case 1020:
        case 1041:
        case 1042:
        case 1043:
        case 1044:
        case 1045:
        case 1046:
            out.summary = decodeEphemerisSummary(out.type, payload);
            {
                const QChar system = broadcastSystem(out.type);
                if (!isSupportedSolutionSystem(system)) {
                    out.ignored = true;
                    out.summary = QString("ignored unsupported ephemeris system %1").arg(system);
                    break;
                }
            }
            out.hasBroadcastEphemeris = parseBroadcastEphemeris(
                out.type, payload, out.broadcastEphemeris,
                _referenceGpsTimeValid ? _referenceGpsWeek : currentGpsWeek());
            if (out.hasBroadcastEphemeris &&
                !isSupportedSolutionSystem(out.broadcastEphemeris.sat.system)) {
                out.ignored = true;
                out.summary = QString("ignored unsupported ephemeris system %1")
                                  .arg(out.broadcastEphemeris.sat.system);
            }
            break;
        case 1230:
            out.summary = decodeGloBiasSummary(payload);
            break;
        case 1300:
        case 1301:
        case 1302:
        case 35:
            out.summary = decodeCrsSummary(out.type, payload);
            break;
        default:
            if ((out.type == 1264 && parseVtecMessage(out.type, payload, out.vtec)) ||
                (out.type == 4076 && parseVtecMessage(out.type, payload, out.vtec))) {
                out.summary = decodeSsrSummary(out.type, payload);
                out.hasVtec = true;
            } else if ((out.type >= 1057 && out.type <= 1068) ||
                (out.type >= 1240 && out.type <= 1270) ||
                out.type == 4076) {
                out.summary = decodeSsrSummary(out.type, payload);
                out.hasSsr = parseSsrMessage(out.type, payload, out.ssr);
                if (out.hasSsr && !isSupportedSolutionSystem(out.ssr.system)) {
                    out.ignored = true;
                    out.summary = QString("ignored unsupported SSR system %1").arg(out.ssr.system);
                }
            } else {
                quint64 possibleStation = 0;
                if (cursor.readUnsigned(12, possibleStation)) {
                    out.summary = QString("sta=%1").arg(possibleStation);
                } else {
                    out.summary = "no specialized decoder";
                }
            }
            break;
        }
    }

    if (out.summary.isEmpty()) {
        out.summary = "decoded";
    }
    return out;
}

QString Rtcm3Decoder::decodeMsmSummary(int type, const QByteArray& payload) {
    BitCursor cursor(payload);
    quint64 value = 0;
    if (!cursor.readUnsigned(12, value)) {
        return "truncated MSM";
    }
    quint64 station = 0;
    if (!cursor.readUnsigned(12, station)) {
        return "truncated MSM";
    }

    const QString system = msmSystem(type);
    if (system.isEmpty()) {
        return "unsupported MSM system";
    }

    quint64 day = 0;
    quint64 epoch = 0;
    if (system == "R") {
        if (!cursor.readUnsigned(3, day) || !cursor.readUnsigned(27, epoch)) {
            return "truncated MSM epoch";
        }
    } else {
        if (!cursor.readUnsigned(30, epoch)) {
            return "truncated MSM epoch";
        }
    }

    quint64 multipleMessage = 0;
    quint64 iods = 0;
    quint64 reserved = 0;
    quint64 clockSteer = 0;
    quint64 externalClock = 0;
    quint64 smooth = 0;
    quint64 smoothInterval = 0;
    if (!cursor.readUnsigned(1, multipleMessage) ||
        !cursor.readUnsigned(3, iods) ||
        !cursor.readUnsigned(7, reserved) ||
        !cursor.readUnsigned(2, clockSteer) ||
        !cursor.readUnsigned(2, externalClock) ||
        !cursor.readUnsigned(1, smooth) ||
        !cursor.readUnsigned(3, smoothInterval)) {
        return "truncated MSM header";
    }

    quint64 satMask = 0;
    quint64 sigMask = 0;
    if (!cursor.readUnsigned(64, satMask) || !cursor.readUnsigned(32, sigMask)) {
        return "truncated MSM mask";
    }

    const int satCount = countBits(satMask);
    const int sigCount = countBits(sigMask);
    const int expectedCellBits = satCount * sigCount;
    const QVector<int> sigIds = msmSignalIdsFromMask(sigMask);

    int cellCount = 0;
    for (int i = 0; i < expectedCellBits; ++i) {
        quint64 cellBit = 0;
        if (!cursor.readUnsigned(1, cellBit)) {
            return QString("MSM header parsed, truncated cell mask sats=%1 sigs=%2")
                .arg(satCount).arg(sigCount);
        }
        if (cellBit) {
            ++cellCount;
        }
    }

    QString epochText;
    if (system == "R") {
        epochText = QString("d%1:%2").arg(day).arg(epoch);
    } else {
        epochText = QString::number(epoch);
    }

    const QString sigInfo = msmSignalInfoText(system.at(0), sigIds);
    QString out = QString("sys=%1 sta=%2 msm=%3 epoch=%4 sats=%5 sigs=%6 cells=%7 iod=%8 mmi=%9")
        .arg(system)
        .arg(station)
        .arg(type % 10)
        .arg(epochText)
        .arg(satCount)
        .arg(sigCount)
        .arg(cellCount)
        .arg(iods)
        .arg(multipleMessage);
    if (!sigInfo.isEmpty()) {
        out += QString(" sigmap=%1").arg(sigInfo);
    }
    return out;
}

QString Rtcm3Decoder::decodeLegacyObsSummary(int type, const QByteArray& payload) {
    BitCursor cursor(payload);
    quint64 value = 0;
    quint64 station = 0;
    if (!cursor.readUnsigned(12, value) || !cursor.readUnsigned(12, station)) {
        return "truncated observation header";
    }

    quint64 epoch = 0;
    quint64 sync = 0;
    quint64 satCount = 0;
    if (isLegacyGpsObs(type)) {
        if (!cursor.readUnsigned(30, epoch) ||
            !cursor.readUnsigned(1, sync) ||
            !cursor.readUnsigned(5, satCount)) {
            return "truncated GPS legacy observation header";
        }
        return QString("legacy-GPS sta=%1 tow=%2ms sats=%3 sync=%4")
            .arg(station).arg(epoch).arg(satCount).arg(sync);
    }

    if (isLegacyGloObs(type)) {
        if (!cursor.readUnsigned(27, epoch) ||
            !cursor.readUnsigned(1, sync) ||
            !cursor.readUnsigned(5, satCount)) {
            return "truncated GLO legacy observation header";
        }
        return QString("legacy-GLO sta=%1 tk=%2 sats=%3 sync=%4")
            .arg(station).arg(epoch).arg(satCount).arg(sync);
    }

    return "legacy observation";
}

QString Rtcm3Decoder::decodeAntennaPositionSummary(int type, const QByteArray& payload) {
    BitCursor cursor(payload);
    quint64 messageType = 0;
    quint64 station = 0;
    quint64 itrfYear = 0;
    quint64 gpsIndicator = 0;
    quint64 gloIndicator = 0;
    quint64 galIndicator = 0;
    quint64 referenceStation = 0;
    qint64 x = 0;
    qint64 y = 0;
    qint64 z = 0;

    if (!cursor.readUnsigned(12, messageType) ||
        !cursor.readUnsigned(12, station) ||
        !cursor.readUnsigned(6, itrfYear) ||
        !cursor.readUnsigned(1, gpsIndicator) ||
        !cursor.readUnsigned(1, gloIndicator) ||
        !cursor.readUnsigned(1, galIndicator) ||
        !cursor.readUnsigned(1, referenceStation) ||
        !cursor.readSigned(38, x) ||
        !cursor.skip(2) ||
        !cursor.readSigned(38, y) ||
        !cursor.skip(2) ||
        !cursor.readSigned(38, z)) {
        return "truncated antenna position";
    }

    const double xx = static_cast<double>(x) * 1e-4;
    const double yy = static_cast<double>(y) * 1e-4;
    const double zz = static_cast<double>(z) * 1e-4;

    QString heightText;
    if (type == 1006) {
        quint64 h = 0;
        if (cursor.readUnsigned(16, h)) {
            heightText = QString(" h=%1m").arg(static_cast<double>(h) * 1e-4, 0, 'f', 4);
        }
    }

    return QString("sta=%1 itrf=%2 xyz=(%3,%4,%5)%6 g=%7 r=%8 e=%9 ref=%10")
        .arg(station)
        .arg(itrfYear)
        .arg(xx, 0, 'f', 4)
        .arg(yy, 0, 'f', 4)
        .arg(zz, 0, 'f', 4)
        .arg(heightText)
        .arg(gpsIndicator)
        .arg(gloIndicator)
        .arg(galIndicator)
        .arg(referenceStation);
}

QString Rtcm3Decoder::decodeAntennaReceiverSummary(int type, const QByteArray& payload) {
    BitCursor cursor(payload);
    quint64 messageType = 0;
    quint64 station = 0;
    if (!cursor.readUnsigned(12, messageType) || !cursor.readUnsigned(12, station)) {
        return "truncated antenna/receiver";
    }

    QString antenna;
    if (!readAsciiString(cursor, antenna)) {
        return "truncated antenna descriptor";
    }

    quint64 setupId = 0;
    if (!cursor.readUnsigned(8, setupId)) {
        return "truncated antenna setup id";
    }

    QString antennaSerial;
    if (type == 1008 || type == 1033) {
        if (!readAsciiString(cursor, antennaSerial)) {
            return "truncated antenna serial";
        }
    }

    QString receiver;
    QString firmware;
    QString receiverSerial;
    if (type == 1033) {
        if (!readAsciiString(cursor, receiver) ||
            !readAsciiString(cursor, firmware) ||
            !readAsciiString(cursor, receiverSerial)) {
            return "truncated receiver block";
        }
    }

    QString summary = QString("sta=%1 ant=%2 setup=%3")
        .arg(station)
        .arg(antenna.isEmpty() ? "-" : antenna)
        .arg(setupId);
    if (!antennaSerial.isEmpty()) {
        summary += QString(" antSN=%1").arg(antennaSerial);
    }
    if (!receiver.isEmpty()) {
        summary += QString(" rec=%1").arg(receiver);
    }
    if (!firmware.isEmpty()) {
        summary += QString(" fw=%1").arg(firmware);
    }
    if (!receiverSerial.isEmpty()) {
        summary += QString(" recSN=%1").arg(receiverSerial);
    }
    return summary;
}

QString Rtcm3Decoder::decodeGloBiasSummary(const QByteArray& payload) {
    BitCursor cursor(payload);
    quint64 messageType = 0;
    quint64 station = 0;
    quint64 indicator = 0;
    quint64 mask = 0;
    if (!cursor.readUnsigned(12, messageType) ||
        !cursor.readUnsigned(12, station) ||
        !cursor.readUnsigned(1, indicator) ||
        !cursor.skip(3) ||
        !cursor.readUnsigned(4, mask)) {
        return "truncated GLO code-phase bias";
    }

    QStringList parts;
    const auto appendBias = [&cursor, &parts](bool enabled, const char* label) {
        if (!enabled) {
            return true;
        }
        qint64 raw = 0;
        if (!cursor.readSigned(16, raw)) {
            return false;
        }
        parts << QString("%1=%2m").arg(label).arg(static_cast<double>(raw) * 0.02, 0, 'f', 2);
        return true;
    };

    if (!appendBias(mask & 0x1, "L1C") ||
        !appendBias(mask & 0x2, "L1P") ||
        !appendBias(mask & 0x4, "L2C") ||
        !appendBias(mask & 0x8, "L2P")) {
        return "truncated GLO code-phase bias values";
    }

    return QString("sta=%1 align=%2 %3")
        .arg(station)
        .arg(indicator)
        .arg(parts.join(" "));
}

QString Rtcm3Decoder::decodeEphemerisSummary(int type, const QByteArray& payload) {
    BitCursor cursor(payload);
    quint64 messageType = 0;
    if (!cursor.readUnsigned(12, messageType)) {
        return "truncated ephemeris";
    }

    quint64 sat = 0;
    quint64 week = 0;
    switch (type) {
    case 1019:
        if (!cursor.readUnsigned(6, sat) || !cursor.readUnsigned(10, week)) {
            return "truncated GPS ephemeris";
        }
        return QString("GPS prn=%1 week=%2").arg(sat).arg(week);
    case 1020: {
        quint64 frq = 0;
        if (!cursor.readUnsigned(6, sat) || !cursor.readUnsigned(5, frq)) {
            return "truncated GLONASS ephemeris";
        }
        return QString("GLO slot=%1 frq=%2").arg(sat).arg(static_cast<int>(frq) - 7);
    }
    case 1041:
        if (!cursor.readUnsigned(6, sat) || !cursor.readUnsigned(10, week)) {
            return "truncated NavIC ephemeris";
        }
        return QString("NavIC prn=%1 week=%2").arg(sat).arg(week);
    case 1042:
        if (!cursor.readUnsigned(6, sat) || !cursor.readUnsigned(13, week)) {
            return "truncated BDS ephemeris";
        }
        return QString("BDS prn=%1 week=%2").arg(sat).arg(week);
    case 1043: {
        quint64 iodn = 0;
        if (!cursor.readUnsigned(6, sat) || !cursor.readUnsigned(8, iodn)) {
            return "truncated SBAS ephemeris";
        }
        return QString("SBAS prn=%1 iodn=%2").arg(sat).arg(iodn);
    }
    case 1044: {
        quint64 toc = 0;
        if (!cursor.readUnsigned(4, sat) || !cursor.readUnsigned(16, toc)) {
            return "truncated QZSS ephemeris";
        }
        return QString("QZSS sat=%1 toc=%2").arg(sat).arg(toc);
    }
    case 1045:
    case 1046:
        if (!cursor.readUnsigned(6, sat) || !cursor.readUnsigned(12, week)) {
            return "truncated Galileo ephemeris";
        }
        return QString("GAL sat=%1 week=%2 nav=%3")
            .arg(sat)
            .arg(week)
            .arg(type == 1046 ? "I/NAV" : "F/NAV");
    default:
        return "ephemeris";
    }
}

QString Rtcm3Decoder::decodeSsrSummary(int type, const QByteArray& payload) {
    if (type == 4076) {
        BitCursor cursor(payload);
        quint64 messageType = 0;
        quint64 version = 0;
        quint64 igsType = 0;
        if (!cursor.readUnsigned(12, messageType) ||
            !cursor.readUnsigned(3, version) ||
            !cursor.readUnsigned(8, igsType)) {
            return "truncated IGS SSR header";
        }
        QChar system;
        RtcmSsrCorrectionKind kind = RtcmSsrCorrectionKind::Unknown;
        if (readIgsSsrSystemAndKind(payload, system, kind)) {
            return QString("IGS SSR subtype=%1 version=%2 sys=%3 type=%4")
                .arg(igsType)
                .arg(version)
                .arg(system)
                .arg(ssrKindToText(kind));
        }
        return QString("IGS SSR subtype=%1 version=%2 type=unknown")
            .arg(igsType)
            .arg(version);
    }

    if (type == 1264) {
        return "VTEC ionosphere message";
    }

    if (type >= 1265 && type <= 1270) {
        QChar system;
        switch (type) {
        case 1265: system = QLatin1Char('G'); break;
        case 1266: system = QLatin1Char('R'); break;
        case 1267: system = QLatin1Char('E'); break;
        case 1268: system = QLatin1Char('J'); break;
        case 1269: system = QLatin1Char('S'); break;
        case 1270: system = QLatin1Char('C'); break;
        default: break;
        }

        BitCursor cursor(payload);
        quint64 messageType = 0;
        quint64 epoch = 0;
        quint64 updateInterval = 0;
        quint64 multipleMessage = 0;
        quint64 iodSsr = 0;
        quint64 provider = 0;
        quint64 solution = 0;
        quint64 disp = 0;
        quint64 mw = 0;
        quint64 satCount = 0;
        const int epochBits = (system == QLatin1Char('R')) ? 17 : 20;

        if (!cursor.readUnsigned(12, messageType) ||
            !cursor.readUnsigned(epochBits, epoch) ||
            !cursor.readUnsigned(4, updateInterval) ||
            !cursor.readUnsigned(1, multipleMessage) ||
            !cursor.readUnsigned(4, iodSsr) ||
            !cursor.readUnsigned(16, provider) ||
            !cursor.readUnsigned(4, solution) ||
            !cursor.readUnsigned(1, disp) ||
            !cursor.readUnsigned(1, mw) ||
            !cursor.readUnsigned(6, satCount)) {
            return "truncated SSR phase-bias";
        }

        return QString("sys=%1 type=phase-bias epoch=%2 ui=%3 iod=%4 provider=%5 solution=%6 sats=%7 mmi=%8 disp=%9 mw=%10")
            .arg(system)
            .arg(epoch)
            .arg(updateInterval)
            .arg(iodSsr)
            .arg(provider)
            .arg(solution)
            .arg(satCount)
            .arg(multipleMessage)
            .arg(disp)
            .arg(mw);
    }

    int base = 0;
    const QString system = ssrSystem(type, &base);
    if (system.isEmpty()) {
        return "SSR";
    }

    BitCursor cursor(payload);
    quint64 messageType = 0;
    if (!cursor.readUnsigned(12, messageType)) {
        return "truncated SSR";
    }

    quint64 epoch = 0;
    const int epochBits = (system == "R") ? 17 : 20;
    if (!cursor.readUnsigned(epochBits, epoch)) {
        return "truncated SSR epoch";
    }

    quint64 updateInterval = 0;
    quint64 multipleMessage = 0;
    quint64 iodSsr = 0;
    quint64 provider = 0;
    quint64 solution = 0;
    if (!cursor.readUnsigned(4, updateInterval) ||
        !cursor.readUnsigned(1, multipleMessage) ||
        !cursor.readUnsigned(4, iodSsr) ||
        !cursor.readUnsigned(16, provider) ||
        !cursor.readUnsigned(4, solution)) {
        return "truncated SSR header";
    }

    quint64 satCount = 0;
    if (!cursor.readUnsigned(6, satCount)) {
        return "truncated SSR sat-count";
    }

    const int subtype = type - base;
    return QString("sys=%1 type=%2 epoch=%3 ui=%4 iod=%5 provider=%6 solution=%7 sats=%8 mmi=%9")
        .arg(system)
        .arg(ssrSubtypeName(subtype))
        .arg(epoch)
        .arg(updateInterval)
        .arg(iodSsr)
        .arg(provider)
        .arg(solution)
        .arg(satCount)
        .arg(multipleMessage);
}

QString Rtcm3Decoder::decodeCrsSummary(int type, const QByteArray& payload) {
    BitCursor cursor(payload);
    quint64 messageType = 0;
    if (!cursor.readUnsigned(12, messageType)) {
        return "truncated CRS";
    }

    if (type == 1301) {
        return "Helmert transformation parameters";
    }

    quint64 nameLength = 0;
    if (!cursor.readUnsigned(5, nameLength)) {
        return "truncated CRS name length";
    }

    QByteArray nameBytes;
    nameBytes.reserve(static_cast<int>(nameLength));
    for (quint64 i = 0; i < nameLength; ++i) {
        quint64 ch = 0;
        if (!cursor.readUnsigned(8, ch)) {
            return "truncated CRS name";
        }
        nameBytes.push_back(static_cast<char>(ch));
    }
    const QString name = QString::fromLatin1(nameBytes);

    if (type == 1300) {
        quint64 ce = 0;
        if (!cursor.readUnsigned(16, ce)) {
            return QString("service-crs=%1").arg(name);
        }
        return QString("service-crs=%1 epoch=%2").arg(name).arg(static_cast<double>(ce) / 100.0, 0, 'f', 2);
    }

    if (type == 1302 || type == 35) {
        quint64 anchor = 0;
        quint64 plate = 0;
        if (cursor.readUnsigned(1, anchor) && cursor.readUnsigned(5, plate)) {
            return QString("rtcm-crs=%1 anchor=%2 plate=%3").arg(name).arg(anchor).arg(plate);
        }
        return QString("rtcm-crs=%1").arg(name);
    }

    return QString("crs=%1").arg(name);
}

QString Rtcm3Decoder::categoryForType(int type) {
    if (isMsm(type)) {
        return "Observation MSM";
    }
    if (isLegacyGpsObs(type) || isLegacyGloObs(type)) {
        return "Observation Legacy";
    }
    if ((type >= 1057 && type <= 1068) || (type >= 1240 && type <= 1270) || type == 4076) {
        return "SSR";
    }
    if (type == 1005 || type == 1006 || type == 1007 || type == 1008 || type == 1033 || type == 1230) {
        return "Station Metadata";
    }
    if (type == 1019 || type == 1020 || type == 1041 || type == 1042 ||
        type == 1043 || type == 1044 || type == 1045 || type == 1046) {
        return "Ephemeris";
    }
    if (type == 1300 || type == 1301 || type == 1302 || type == 35) {
        return "CRS";
    }
    return "Other";
}

QString Rtcm3Decoder::ssrKindToText(RtcmSsrCorrectionKind kind) {
    switch (kind) {
    case RtcmSsrCorrectionKind::Orbit:
        return "orbit";
    case RtcmSsrCorrectionKind::Clock:
        return "clock";
    case RtcmSsrCorrectionKind::OrbitClock:
        return "orbit-clock";
    case RtcmSsrCorrectionKind::CodeBias:
        return "code-bias";
    case RtcmSsrCorrectionKind::PhaseBias:
        return "phase-bias";
    case RtcmSsrCorrectionKind::HighRateClock:
        return "high-rate-clock";
    case RtcmSsrCorrectionKind::Ura:
        return "ura";
    default:
        break;
    }
    return "unknown";
}
