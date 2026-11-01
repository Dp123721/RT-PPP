// Maps GNSS signal identifiers to frequencies and observation combinations.
#include "gnss_signals.h"

#include "gnss_utils.h"

#include <QMap>

namespace {

using SignalMap = QMap<int, QString>;

const SignalMap& bdsMsmMap() {
    static const SignalMap bds = {
        {2, "2I"}, {3, "2Q"}, {4, "2X"},
        {8, "6I"}, {9, "6Q"}, {10, "6X"},
        {14, "7I"}, {15, "7Q"}, {16, "7X"},
        {22, "5D"}, {23, "5P"}, {24, "5X"},
        {25, "7D"},
        {30, "1D"}, {31, "1P"}, {32, "1X"}
    };
    return bds;
}

const SignalMap& msmMap(QChar system) {
    static const SignalMap gps = {
        {2, "1C"}, {3, "1P"}, {4, "1W"},
        {8, "2C"}, {9, "2P"}, {10, "2W"},
        {15, "2S"}, {16, "2L"}, {17, "2X"},
        {22, "5I"}, {23, "5Q"}, {24, "5X"},
        {30, "1S"}, {31, "1L"}, {32, "1X"}
    };
    static const SignalMap glo = {
        {2, "1C"}, {3, "1P"}, {8, "2C"}, {9, "2P"},
        {10, "4A"}, {11, "4B"}, {12, "4X"},
        {13, "6A"}, {14, "6B"}, {15, "6X"},
        {16, "3I"}, {17, "3Q"}, {18, "3X"}
    };
    static const SignalMap gal = {
        {2, "1C"}, {3, "1A"}, {4, "1B"}, {5, "1X"}, {6, "1Z"},
        {8, "6C"}, {9, "6A"}, {10, "6B"}, {11, "6X"}, {12, "6Z"},
        {14, "7I"}, {15, "7Q"}, {16, "7X"},
        {18, "8I"}, {19, "8Q"}, {20, "8X"},
        {22, "5I"}, {23, "5Q"}, {24, "5X"}
    };
    static const SignalMap qzss = {
        {2, "1C"},
        {9, "6S"}, {10, "6L"}, {11, "6X"},
        {15, "2S"}, {16, "2L"}, {17, "2X"},
        {22, "5I"}, {23, "5Q"}, {24, "5X"},
        {30, "1S"}, {31, "1L"}, {32, "1X"}
    };
    static const SignalMap sbas = {
        {2, "1C"},
        {22, "5I"}, {23, "5Q"}, {24, "5X"}
    };
    static const SignalMap navic = {
        {8, "9A"},
        {22, "5A"}
    };

    switch (system.toLatin1()) {
    case 'G': return gps;
    case 'R': return glo;
    case 'E': return gal;
    case 'J': return qzss;
    case 'C': return bdsMsmMap();
    case 'S': return sbas;
    case 'I': return navic;
    default: break;
    }
    static const SignalMap empty;
    return empty;
}

const SignalMap& ssrMap(QChar system) {
    static const SignalMap gps = {
        {0, "1C"}, {1, "1P"}, {2, "1W"},
        {5, "2C"}, {6, "2D"}, {7, "2S"}, {8, "2L"}, {9, "2X"},
        {10, "2P"}, {11, "2W"},
        {14, "5I"}, {15, "5Q"}, {16, "5X"},
        {17, "1S"}, {18, "1L"}, {19, "1X"}
    };
    static const SignalMap glo = {
        {0, "1C"}, {1, "1P"}, {2, "2C"}, {3, "2P"},
        {4, "4A"}, {5, "4B"}, {6, "4X"},
        {7, "6A"}, {8, "6B"}, {9, "6X"},
        {10, "3I"}, {11, "3Q"}, {12, "3X"}
    };
    static const SignalMap gal = {
        {0, "1A"}, {1, "1B"}, {2, "1C"}, {3, "1X"}, {4, "1Z"},
        {5, "5I"}, {6, "5Q"}, {7, "5X"},
        {8, "7I"}, {9, "7Q"}, {10, "7X"},
        {11, "8I"}, {12, "8Q"}, {13, "8X"},
        {14, "6A"}, {15, "6B"}, {16, "6C"}, {17, "6X"}, {18, "6Z"}
    };
    static const SignalMap qzss = {
        {0, "1C"}, {1, "1S"}, {2, "1L"}, {3, "2S"}, {4, "2L"}, {5, "2X"},
        {6, "5I"}, {7, "5Q"}, {8, "5X"},
        {9, "6S"}, {10, "6L"}, {11, "6X"},
        {12, "1X"}, {13, "1Z"},
        {14, "5D"}, {15, "5P"}, {16, "5Z"},
        {17, "6E"}, {18, "6Z"}
    };
    static const SignalMap sbas = {
        {0, "1C"}, {1, "5I"}, {2, "5Q"}, {3, "5X"}
    };
    static const SignalMap bds = {
        {0, "2I"}, {1, "2Q"}, {2, "2X"},
        {3, "6I"}, {4, "6Q"}, {5, "6X"},
        {6, "7I"}, {7, "7Q"}, {8, "7X"},
        {9, "1D"}, {10, "1P"}, {11, "1X"},
        {12, "5D"}, {13, "5P"}, {14, "5X"}
    };

    switch (system.toLatin1()) {
    case 'G': return gps;
    case 'R': return glo;
    case 'E': return gal;
    case 'J': return qzss;
    case 'S': return sbas;
    case 'C': return bds;
    default: break;
    }
    static const SignalMap empty;
    return empty;
}

const SignalMap& ssrIgsMap(QChar system) {
    static const SignalMap gps = {
        {0, "1C"}, {1, "1P"}, {2, "1W"},
        {3, "1S"}, {4, "1L"},
        {5, "2C"}, {6, "2D"}, {7, "2S"}, {8, "2L"},
        {10, "2P"}, {11, "2W"},
        {14, "5I"}, {15, "5Q"}
    };
    static const SignalMap glo = {
        {0, "1C"}, {1, "1P"}, {2, "2C"}, {3, "2P"},
        {4, "4A"}, {5, "4B"},
        {7, "6A"}, {8, "6B"},
        {10, "3I"}, {11, "3Q"}
    };
    static const SignalMap gal = {
        {0, "1A"}, {1, "1B"}, {2, "1C"},
        {5, "5I"}, {6, "5Q"},
        {8, "7I"}, {9, "7Q"},
        {14, "6A"}, {15, "6B"}, {16, "6C"}
    };
    static const SignalMap qzss = {
        {0, "1C"}, {1, "1S"}, {2, "1L"},
        {3, "2S"}, {4, "2L"},
        {6, "5I"}, {7, "5Q"},
        {9, "6S"}, {10, "6L"}, {11, "6E"}
    };
    static const SignalMap sbas = {
        {0, "1C"}, {1, "5I"}, {2, "5Q"}
    };
    static const SignalMap bds = {
        {0, "2I"}, {1, "2Q"},
        {3, "6I"}, {4, "6Q"}, {5, "6A"},
        {6, "7I"}, {7, "7Q"},
        {9, "1D"}, {10, "1P"}, {12, "5D"}, {13, "5P"},
        {15, "1A"}
    };

    switch (system.toLatin1()) {
    case 'G': return gps;
    case 'R': return glo;
    case 'E': return gal;
    case 'J': return qzss;
    case 'S': return sbas;
    case 'C': return bds;
    default: break;
    }
    static const SignalMap empty;
    return empty;
}

double freqForSignal(QChar system, const QString& type2ch, int glonassChannel) {
    if (type2ch.isEmpty()) {
        return 0.0;
    }
    const QChar band = type2ch.at(0);
    switch (system.toLatin1()) {
    case 'G':
    case 'S':
        if (band == QLatin1Char('1')) return 1575420000.0;
        if (band == QLatin1Char('2')) return 1227600000.0;
        if (band == QLatin1Char('5')) return 1176450000.0;
        break;
    case 'E':
        if (band == QLatin1Char('1')) return 1575420000.0;
        if (band == QLatin1Char('5')) return 1176450000.0;
        if (band == QLatin1Char('6')) return 1278750000.0;
        if (band == QLatin1Char('7')) return 1207140000.0;
        if (band == QLatin1Char('8')) return 1191795000.0;
        break;
    case 'J':
        if (band == QLatin1Char('1')) return 1575420000.0;
        if (band == QLatin1Char('2')) return 1227600000.0;
        if (band == QLatin1Char('5')) return 1176450000.0;
        if (band == QLatin1Char('6')) return 1278750000.0;
        break;
    case 'C':
        if (band == QLatin1Char('1')) return 1575420000.0;
        if (band == QLatin1Char('2')) return 1561098000.0;
        if (band == QLatin1Char('5')) return 1176450000.0;
        if (band == QLatin1Char('6')) return 1268520000.0;
        if (band == QLatin1Char('7')) return 1207140000.0;
        if (band == QLatin1Char('8')) return 1191795000.0;
        break;
    case 'I':
        if (band == QLatin1Char('5')) return 1176450000.0;
        if (band == QLatin1Char('9')) return 2492028000.0;
        break;
    case 'R':
        if (band == QLatin1Char('1')) return 1602000000.0 + 562500.0 * glonassChannel;
        if (band == QLatin1Char('2')) return 1246000000.0 + 437500.0 * glonassChannel;
        if (band == QLatin1Char('3')) return 1202025000.0;
        if (band == QLatin1Char('4')) return 1600995000.0;
        if (band == QLatin1Char('6')) return 1248060000.0;
        break;
    default:
        break;
    }
    return 0.0;
}

} // namespace

QString msmSignalToRinexType2ch(QChar system, int signalId) {
    return msmMap(system).value(signalId);
}

QString msmSignalToRinexType2ch(QChar system, int signalId, const QVector<int>& signalIdsInMessage) {
    Q_UNUSED(signalIdsInMessage);
    return msmSignalToRinexType2ch(system, signalId);
}

int rinexType2chToMsmSignalId(QChar system, const QString& type2ch) {
    const SignalMap& map = msmMap(system);
    for (auto it = map.constBegin(); it != map.constEnd(); ++it) {
        if (it.value() == type2ch) return it.key();
    }
    return -1;
}

QString ssrSignalToRinexType2ch(QChar system, int signalCode) {
    return ssrMap(system).value(signalCode);
}

QString ssrSignalToRinexType2ch(QChar system, int signalCode, int messageType) {
    if (messageType == 4076) {
        return ssrIgsMap(system).value(signalCode);
    }
    return ssrSignalToRinexType2ch(system, signalCode);
}

int rinexType2chToSsrSignalCode(QChar system, const QString& type2ch) {
    const SignalMap& map = ssrMap(system);
    for (auto it = map.constBegin(); it != map.constEnd(); ++it) {
        if (it.value() == type2ch) return it.key();
    }
    const SignalMap& igs = ssrIgsMap(system);
    for (auto it = igs.constBegin(); it != igs.constEnd(); ++it) {
        if (it.value() == type2ch) return it.key();
    }
    return -1;
}

double signalWavelengthMeters(QChar system, const QString& type2ch, int glonassChannel) {
    const double frequency = freqForSignal(system, type2ch, glonassChannel);
    if (frequency <= 0.0) {
        return 0.0;
    }
    return GnssUtils::kC / frequency;
}

QStringList rinexObsTypesForSystem(QChar system) {
    QStringList out;
    const SignalMap mm = msmMap(system);
    for (auto it = mm.constBegin(); it != mm.constEnd(); ++it) {
        if (!it.value().isEmpty()) {
            const QString c = "C" + it.value();
            const QString l = "L" + it.value();
            const QString s = "S" + it.value();
            if (!out.contains(c)) out.append(c);
            if (!out.contains(l)) out.append(l);
            if (!out.contains(s)) out.append(s);
        }
    }
    return out;
}
