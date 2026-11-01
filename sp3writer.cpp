// Writes satellite orbit and clock records in SP3-compatible files.
#include "sp3writer.h"

#include <algorithm>
#include <cmath>
#include <QDir>
#include <QFileInfo>

namespace {

QString padOrTrimRight(const QString& text, int width) {
    QString out = text;
    if (out.size() > width) {
        out = out.left(width);
    }
    while (out.size() < width) {
        out += QLatin1Char(' ');
    }
    return out;
}

QString satToken(const QString& prn) {
    QString p = prn.trimmed().toUpper();
    if (p.size() == 2) {
        p = QString("G%1").arg(p);
    }
    if (p.size() < 3) {
        p = p.leftJustified(3, QLatin1Char('0'));
    }
    return p.left(3);
}

} // namespace

Sp3Writer::Sp3Writer() : _stream(&_file), _clkStream(&_clkFile) {}

Sp3Writer::~Sp3Writer() {
    close();
}

void Sp3Writer::setOutputDirectory(const QString& directory) {
    const QString normalized = directory.trimmed();
    if (_directory == normalized) {
        return;
    }
    close();
    _directory = normalized;
}

void Sp3Writer::setSampleInterval(int seconds) {
    const int normalized = qMax(1, seconds);
    if (_sampleInterval == normalized) {
        return;
    }
    close();
    _sampleInterval = normalized;
}

bool Sp3Writer::isEnabled() const {
    return !_directory.isEmpty();
}

void Sp3Writer::close() {
    closeFile();
}

bool Sp3Writer::writeEpoch(int gpsWeek,
                           double gpsSow,
                           const QString& prn,
                           const std::array<double,3>& position,
                           double clockSec,
                           const std::array<double,3>& velocity,
                           double clockRate) {
    if (!isEnabled()) {
        return false;
    }

    const QDateTime epochUtc = GnssUtils::dateTimeFromGpsWeek(gpsWeek, gpsSow);
    ensureOpen(epochUtc);
    if (!_file.isOpen() || !_clkFile.isOpen()) {
        return false;
    }

    const QString sat = satToken(prn);
    if (!_allSats.contains(sat)) {
        _allSats.insert(sat);
        updateHeaderSats();
    }

    if (!_lastEpoch.isValid() || epochUtc != _lastEpoch) {
        if (_lastEpoch.isValid() && _sampleInterval > 0) {
            QDateTime next = _lastEpoch.addSecs(_sampleInterval);
            while (next < epochUtc) {
                _stream << "\n*  " + next.toUTC().toString("yyyy MM dd hh mm ss.zzzzzzz").left(27);
                next = next.addSecs(_sampleInterval);
            }
        }

        _stream << "\n*  " + epochUtc.toUTC().toString("yyyy MM dd hh mm ss.zzzzzzz").left(27);
        _lastEpoch = epochUtc;
        _epochSatSet.clear();
    }

    if (_epochSatSet.contains(sat)) {
        return true;
    }
    _epochSatSet.insert(sat);

    _stream << QString("\nP%1%2%3%4%5")
                   .arg(sat)
                   .arg(position[0] / 1000.0, 14, 'f', 6)
                   .arg(position[1] / 1000.0, 14, 'f', 6)
                   .arg(position[2] / 1000.0, 14, 'f', 6)
                   .arg(clockSec * 1e6, 14, 'f', 6);

    if (clockRate != 0.0) {
        _stream << QString("\nV%1%2%3%4%5")
                       .arg(sat)
                       .arg(velocity[0] * 10.0, 14, 'f', 6)
                       .arg(velocity[1] * 10.0, 14, 'f', 6)
                       .arg(velocity[2] * 10.0, 14, 'f', 6)
                       .arg(clockRate * 1e2, 14, 'f', 6);
    }

    _clkStream << QString("AS %1 %2 %3")
                      .arg(sat)
                      .arg(epochUtc.toUTC().toString("yyyy MM dd hh mm ss.zzzzzz").left(26))
                      .arg(clockSec, 19, 'e', 12)
               << "\n";

    _stream.flush();
    _file.flush();
    _clkStream.flush();
    _clkFile.flush();
    return true;
}

void Sp3Writer::ensureOpen(const QDateTime& epochUtc) {
    const QDate date = epochUtc.date();
    if (_file.isOpen() && _clkFile.isOpen() && date == _openDate && date == _clkOpenDate) {
        return;
    }

    closeFile();
    if (_directory.isEmpty()) {
        return;
    }

    QDir dir(_directory);
    if (!dir.exists()) {
        dir.mkpath(".");
    }

    _allSats.clear();
    _headerSatPos.clear();
    _headerAccPos.clear();

    const QString sp3Path = resolveFilePath(epochUtc);
    _file.setFileName(sp3Path);
    if (!_file.open(QIODevice::ReadWrite | QIODevice::Text)) {
        return;
    }
    _stream.setCodec("UTF-8");
    _openDate = date;
    writeHeader(epochUtc);

    const QString clkPath = resolveClkFilePath(epochUtc);
    _clkFile.setFileName(clkPath);
    if (!_clkFile.open(QIODevice::WriteOnly | QIODevice::Text)) {
        closeFile();
        return;
    }
    _clkStream.setCodec("UTF-8");
    _clkOpenDate = date;
    writeClkHeader(epochUtc);
}

void Sp3Writer::closeFile() {
    if (_file.isOpen()) {
        _stream << "\nEOF\n";
        _stream.flush();
        _file.flush();
        _file.close();
    }
    if (_clkFile.isOpen()) {
        _clkStream << "EOF\n";
        _clkStream.flush();
        _clkFile.flush();
        _clkFile.close();
    }
    _openDate = QDate();
    _clkOpenDate = QDate();
    _lastEpoch = QDateTime();
    _epochSatSet.clear();
    _allSats.clear();
    _headerSatPos.clear();
    _headerAccPos.clear();
}

void Sp3Writer::writeHeader(const QDateTime& epochUtc) {
    const int numEpoch = (24 * 3600) / qMax(1, _sampleInterval);
    int gpsWeek = 0;
    double gpsSow = 0.0;
    GnssUtils::gpsWeekFromDateTime(epochUtc, gpsWeek, gpsSow);

    int mjd = 0;
    double dayfrac = 0.0;
    GnssUtils::mjdFromDateTime(epochUtc, mjd, dayfrac);
    const double sec = std::fmod(gpsSow, 60.0);

    _stream << QString("#dP%1 %2%3 d+D IGS20 FIT RTPPP\n")
                   .arg(epochUtc.toUTC().toString("yyyy MM dd hh mm"))
                   .arg(sec, 12, 'f', 8)
                   .arg(numEpoch, 7);
    _stream << QString("## %1%2%3%4%5\n")
                   .arg(gpsWeek, 4)
                   .arg(gpsSow, 16, 'f', 8)
                   .arg(static_cast<double>(_sampleInterval), 15, 'f', 8)
                   .arg(mjd, 6)
                   .arg(dayfrac, 16, 'f', 13);

    _headerSatLines = 10;
    for (int i = 0; i < _headerSatLines; ++i) {
        const qint64 pos = _file.pos();
        _headerSatPos.push_back(pos);
        _stream << "+   0";
        for (int k = 0; k < 17; ++k) {
            _stream << "   ";
        }
        _stream << "\n";
    }
    for (int i = 0; i < _headerSatLines; ++i) {
        const qint64 pos = _file.pos();
        _headerAccPos.push_back(pos);
        _stream << "++      ";
        for (int k = 0; k < 17; ++k) {
            _stream << " 0";
        }
        _stream << "\n";
    }

    _stream << "%c M  cc GPS ccc cccc cccc cccc cccc ccccc ccccc ccccc ccccc\n";
    _stream << "%c cc cc ccc ccc cccc cccc cccc cccc ccccc ccccc ccccc ccccc\n";
    _stream << "%f  1.2500000  1.025000000  0.00000000000  0.000000000000000\n";
    _stream << "%f  0.0000000  0.000000000  0.00000000000  0.000000000000000\n";
    _stream << "%i    0    0    0    0      0      0      0      0      0\n";
    _stream << "%i    0    0    0    0      0      0      0      0      0\n";
    _stream << "/* RT-PPP generated SSR-corrected SP3\n";
    _stream << "/* ORB: corrected broadcast ephemeris\n";
    _stream << "/* CLK: see paired CLK file in same directory\n";
    _stream << "/*\n";
    _stream.flush();
    _file.flush();
}

void Sp3Writer::writeClkHeader(const QDateTime& epochUtc) {
    _clkStream << "     3.04           C                   RINEX VERSION / TYPE\n";
    _clkStream << "RT-PPP              OPENAI              "
               << epochUtc.toUTC().toString("yyyyMMdd HHmmss UTC")
               << " PGM / RUN BY / DATE\n";
    _clkStream << "SSR corrected satellite clocks                          COMMENT\n";
    _clkStream << "END OF HEADER\n";
}

void Sp3Writer::updateHeaderSats() {
    if (!_file.isOpen() || _headerSatPos.isEmpty() || _headerAccPos.isEmpty()) {
        return;
    }
    QStringList sats = _allSats.values();
    std::sort(sats.begin(), sats.end());
    const int satCount = sats.size();
    const int linesNeeded = qMax(1, (satCount + 16) / 17);
    if (linesNeeded > _headerSatLines) {
        return;
    }

    for (int i = 0; i < _headerSatLines; ++i) {
        QString line = (i == 0)
            ? QString("+%1").arg(satCount, 4)
            : QString("+    ");
        for (int k = 0; k < 17; ++k) {
            const int idx = i * 17 + k;
            if (idx < satCount) {
                line += satToken(sats[idx]);
            } else {
                line += "   ";
            }
        }
        line = padOrTrimRight(line, 60) + "\n";
        _file.seek(_headerSatPos[i]);
        _file.write(line.toLatin1());
    }

    for (int i = 0; i < _headerSatLines; ++i) {
        QString line("++      ");
        for (int k = 0; k < 17; ++k) {
            const int idx = i * 17 + k;
            if (idx < satCount) {
                line += " 4";
            } else {
                line += " 0";
            }
        }
        line = padOrTrimRight(line, 60) + "\n";
        _file.seek(_headerAccPos[i]);
        _file.write(line.toLatin1());
    }
    _file.seek(_file.size());
    _stream.seek(_file.size());
}

QString Sp3Writer::resolveFilePath(const QDateTime& epochUtc) const {
    const QString dateStr = epochUtc.toUTC().toString("yyyyMMdd");
    const QString fileName = QString("rtppp_%1.sp3").arg(dateStr);
    QDir dir(_directory);
    return dir.filePath(fileName);
}

QString Sp3Writer::resolveClkFilePath(const QDateTime& epochUtc) const {
    const QString dateStr = epochUtc.toUTC().toString("yyyyMMdd");
    const QString fileName = QString("rtppp_%1.clk").arg(dateStr);
    QDir dir(_directory);
    return dir.filePath(fileName);
}
