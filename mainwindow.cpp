// MainWindow binds the UI, applies saved settings, coordinates runtime services, and presents results.
#include "mainwindow.h"
#include "ui_mainwindow.h"
#include "ui_addmqttdialog.h"
#include "ui_addntripcasterdialog.h"
#include "ui_addtcpdialog.h"
#include "gnss_utils.h"
#include "gfs_zwd.h"

#include <algorithm>
#include <cmath>
#include <functional>
#include <iterator>
#include <limits>
#include <QAbstractItemView>
#include <QCloseEvent>
#include <QCheckBox>
#include <QComboBox>
#include <QCoreApplication>
#include <QDateTime>
#include <QDebug>
#include <QDialog>
#include <QDialogButtonBox>
#include <QDir>
#include <QDoubleSpinBox>
#include <QFormLayout>
#include <QFileDialog>
#include <QFile>
#include <QFileInfo>
#include <QGroupBox>
#include <QGridLayout>
#include <QHeaderView>
#include <QHBoxLayout>
#include <QJsonArray>
#include <QJsonObject>
#include <QJsonValue>
#include <QLabel>
#include <QMessageBox>
#include <QPainter>
#include <QPainterPath>
#include <QPlainTextEdit>
#include <QPushButton>
#include <QProgressBar>
#include <QFont>
#include <QRegularExpression>
#include <QLineEdit>
#include <QMetaObject>
#include <QScrollArea>
#include <QGraphicsScene>
#include <QGraphicsTextItem>
#include <QGraphicsView>
#include <QSet>
#include <QSettings>
#include <QSignalBlocker>
#include <QSpinBox>
#include <QTableWidget>
#include <QTableWidgetItem>
#include <QThread>
#include <QSslSocket>
#include <QUrl>
#include <QTabWidget>
#include <QTimer>
#include <QTextStream>
#include <QTextCursor>
#include <QtGlobal>
#include <QVBoxLayout>

namespace {

constexpr double kPi = 3.1415926535897932384626433832795;
constexpr double kRhoDeg = 180.0 / kPi;

QString bytesToText(quint64 bytes) {
    const double b = static_cast<double>(bytes);
    if (b < 1024.0) {
        return QString("%1 B").arg(bytes);
    }
    const double kb = b / 1024.0;
    if (kb < 1024.0) {
        return QString("%1 KB").arg(kb, 0, 'f', kb < 10.0 ? 2 : 1);
    }
    const double mb = kb / 1024.0;
    if (mb < 1024.0) {
        return QString("%1 MB").arg(mb, 0, 'f', mb < 10.0 ? 2 : 1);
    }
    const double gb = mb / 1024.0;
    return QString("%1 GB").arg(gb, 0, 'f', gb < 10.0 ? 2 : 1);
}

QByteArray rtpppNtripUserAgent() {
    return QByteArrayLiteral("NTRIP RT-PPP/1.0");
}

QByteArray decodedNtripCredential(const QString& text) {
    QByteArray out = text.trimmed().toLatin1();
    for (int i = 0; i < 2 && out.contains('%'); ++i) {
        const QByteArray decoded = QUrl::fromPercentEncoding(out).toLatin1();
        if (decoded == out) {
            break;
        }
        out = decoded;
    }
    return out;
}

QString dataPreviewToHex(const QByteArray& payload) {
    QByteArray hex = payload.left(64).toHex(' ').toUpper();
    if (payload.size() > 64) {
        hex += " ...";
    }
    return QString::fromLatin1(hex);
}

bool isStandardPhaseBiasMt(int type) {
    return type >= 1265 && type <= 1270;
}

QString summarizeDecodedTypes(const QVector<RtcmDecodedMessage>& messages) {
    QMap<int, int> counters;
    for (const RtcmDecodedMessage& message : messages) {
        counters[message.type] += 1;
    }
    QStringList parts;
    for (auto it = counters.constBegin(); it != counters.constEnd(); ++it) {
        parts << QString("MT%1x%2").arg(it.key()).arg(it.value());
    }
    return parts.join(", ");
}

bool shouldLogNow(QDateTime& last, int intervalSec) {
    const QDateTime now = QDateTime::currentDateTimeUtc();
    if (!last.isValid() || last.secsTo(now) >= intervalSec) {
        last = now;
        return true;
    }
    return false;
}

QDateTime gpsOutputTime(const PppSolution& solution) {
    if (solution.gpsTime.isValid()) return solution.gpsTime;
    int gpsWeek = solution.gpsWeek;
    double gpsSow = solution.gpsSow;
    if (gpsWeek <= 0 || !std::isfinite(gpsSow)) {
        GnssUtils::gpsWeekFromDateTime(solution.utc, gpsWeek, gpsSow);
    }
    return GnssUtils::gpsCalendarDateTime(gpsWeek, gpsSow);
}

bool isSupportedSolutionSystem(const QChar& system) {
    return system == QLatin1Char('G') ||
           system == QLatin1Char('R') ||
           system == QLatin1Char('E') ||
           system == QLatin1Char('C');
}

QString prepStreamLabel(const QString& id, const QString& mountpoint, const QString& display) {
    if (id.isEmpty()) {
        return "-";
    }
    if (!mountpoint.isEmpty()) {
        return mountpoint + " [" + id + "]";
    }
    return display.isEmpty() ? id : (display + " [" + id + "]");
}

QString navTypeToString(int navType) {
    switch (navType) {
    case RtcmNavLnav:
        return "LNAV";
    case RtcmNavFdmaM:
        return "FDMA_M";
    case RtcmNavInav:
        return "INAV";
    case RtcmNavD1:
        return "D1";
    case RtcmNavD2:
        return "D2";
    case RtcmNavSbasL1:
        return "SBASL1";
    default:
        break;
    }
    return "UNKNOWN";
}

QColor systemColor(const QChar& sys) {
    if (sys == QLatin1Char('G')) return QColor(0, 170, 0);
    if (sys == QLatin1Char('R')) return QColor(200, 0, 0);
    if (sys == QLatin1Char('E')) return QColor(0, 90, 200);
    if (sys == QLatin1Char('C')) return QColor(220, 140, 0);
    if (sys == QLatin1Char('J')) return QColor(140, 0, 180);
    if (sys == QLatin1Char('S')) return QColor(90, 90, 90);
    if (sys == QLatin1Char('I')) return QColor(0, 150, 150);
    return QColor(120, 120, 120);
}

QString systemDisplayName(const QChar& sys) {
    if (sys == QLatin1Char('G')) return "GPS (G)";
    if (sys == QLatin1Char('R')) return "GLONASS (R)";
    if (sys == QLatin1Char('E')) return "Galileo (E)";
    if (sys == QLatin1Char('C')) return "BeiDou (C)";
    if (sys == QLatin1Char('J')) return "QZSS (J)";
    if (sys == QLatin1Char('S')) return "SBAS (S)";
    if (sys == QLatin1Char('I')) return "NavIC (I)";
    return QString("Unknown (%1)").arg(sys);
}

struct EllipsoidCoord {
    double lat = 0.0;
    double lon = 0.0;
    double h = 0.0;
};

EllipsoidCoord xyzToEll(const std::array<double,3>& xyz) {
    const double a = GnssUtils::kAell;
    const double f = 1.0 / GnssUtils::kFInv;
    const double e2 = f * (2.0 - f);

    const double x = xyz[0];
    const double y = xyz[1];
    const double z = xyz[2];
    const double lon = std::atan2(y, x);
    const double p = std::sqrt(x * x + y * y);
    double lat = std::atan2(z, p * (1.0 - e2));
    double h = 0.0;
    for (int i = 0; i < 10; ++i) {
        const double sinLat = std::sin(lat);
        const double n = a / std::sqrt(1.0 - e2 * sinLat * sinLat);
        h = p / std::cos(lat) - n;
        const double latNew = std::atan2(z, p * (1.0 - e2 * n / (n + h)));
        if (std::fabs(latNew - lat) < 1e-12) {
            lat = latNew;
            break;
        }
        lat = latNew;
    }
    return {lat, lon, h};
}

std::array<double,3> ellToXyz(double latRad, double lonRad, double height) {
    const double a = GnssUtils::kAell;
    const double f = 1.0 / GnssUtils::kFInv;
    const double e2 = f * (2.0 - f);
    const double sinLat = std::sin(latRad);
    const double cosLat = std::cos(latRad);
    const double sinLon = std::sin(lonRad);
    const double cosLon = std::cos(lonRad);
    const double n = a / std::sqrt(1.0 - e2 * sinLat * sinLat);
    return {
        (n + height) * cosLat * cosLon,
        (n + height) * cosLat * sinLon,
        (n * (1.0 - e2) + height) * sinLat
    };
}

struct NtripSelectionRow {
    NtripStreamConfig cfg;
    QString decoder;
    QString lat;
    QString lon;
    QString nmea;
};

struct NtripCasterEntry {
    QString name;
    QString host;
    int port = 2101;
    QString ntripVersion = "2";
    QString user;
    QString password;
};

QString normalizedNtripVersion(QString version) {
    version = version.trimmed().toLower();
    if (version == "1" || version == "v1" || version == "http/1.0") {
        return "1";
    }
    if (version == "2s" || version == "ssl" || version == "https") {
        return "2s";
    }
    return "2";
}

bool isSupportedNtripVersion(QString version) {
    version = version.trimmed().toLower();
    return version.isEmpty() || version == "1" || version == "v1" ||
           version == "http/1.0" || version == "2" || version == "v2" ||
           version == "http/1.1" || version == "2s" || version == "ssl" ||
           version == "https";
}

QString casterDisplayName(const NtripCasterEntry& entry) {
    if (!entry.name.trimmed().isEmpty()) {
        return entry.name.trimmed();
    }
    if (!entry.host.trimmed().isEmpty()) {
        return QString("%1:%2").arg(entry.host.trimmed()).arg(entry.port);
    }
    return "Unnamed caster";
}

QVector<NtripCasterEntry> defaultCasterEntries() {
    return {
        {"IGS products", "products.igs-ip.net", 80, "2", "", ""},
        {"WHU GNSSLab", "ntrip.gnsslab.cn", 2101, "2", "", ""},
        {"RTK2go", "rtk2go.com", 2101, "2", "", ""}
    };
}

QJsonObject casterEntryToJson(const NtripCasterEntry& entry) {
    QJsonObject o;
    o["name"] = entry.name.trimmed();
    o["host"] = entry.host.trimmed();
    o["port"] = entry.port;
    o["version"] = normalizedNtripVersion(entry.ntripVersion);
    o["user"] = entry.user;
    o["password"] = entry.password;
    return o;
}

NtripCasterEntry casterEntryFromJson(const QJsonObject& obj) {
    NtripCasterEntry entry;
    entry.name = obj.value("name").toString();
    entry.host = obj.value("host").toString().trimmed();
    entry.port = obj.value("port").toInt(2101);
    entry.ntripVersion = normalizedNtripVersion(
        obj.value("version").toString(obj.value("ntripVersion").toString("2")));
    entry.user = obj.value("user").toString();
    entry.password = obj.value("password").toString();
    return entry;
}

QVector<NtripCasterEntry> casterEntriesFromJson(const QJsonArray& arr) {
    QVector<NtripCasterEntry> entries;
    for (const QJsonValue& value : arr) {
        if (!value.isObject()) {
            continue;
        }
        NtripCasterEntry entry = casterEntryFromJson(value.toObject());
        if (entry.host.isEmpty() || entry.port <= 0) {
            continue;
        }
        if (entry.name.trimmed().isEmpty()) {
            entry.name = casterDisplayName(entry);
        }
        entries.push_back(entry);
    }
    if (entries.isEmpty()) {
        entries = defaultCasterEntries();
    }
    return entries;
}

QJsonArray casterEntriesToJson(const QVector<NtripCasterEntry>& entries) {
    QJsonArray arr;
    for (const NtripCasterEntry& entry : entries) {
        if (entry.host.trimmed().isEmpty() || entry.port <= 0) {
            continue;
        }
        arr.push_back(casterEntryToJson(entry));
    }
    return arr;
}

void configureSortableResizableTable(QTableWidget* table) {
    if (!table) {
        return;
    }
    QHeaderView* header = table->horizontalHeader();
    if (header) {
        header->setSectionsClickable(true);
        header->setSectionResizeMode(QHeaderView::Interactive);
        header->setStretchLastSection(true);
    }
    const QString selectedStyle =
        "QTableWidget{selection-color:#1f2937;}"
        "QTableWidget::item:selected{color:#1f2937;background:#dbeafe;}";
    if (!table->styleSheet().contains("QTableWidget::item:selected")) {
        table->setStyleSheet(table->styleSheet() + selectedStyle);
    }
    table->setSortingEnabled(true);
}

QString tableRowStreamId(const QTableWidget* table, int row) {
    if (!table || row < 0 || row >= table->rowCount()) {
        return QString();
    }
    const QTableWidgetItem* item = table->item(row, 0);
    return item ? item->data(Qt::UserRole).toString() : QString();
}

int tableRowForStreamId(const QTableWidget* table, const QString& streamId) {
    if (!table || streamId.isEmpty()) {
        return -1;
    }
    for (int row = 0; row < table->rowCount(); ++row) {
        if (tableRowStreamId(table, row) == streamId) {
            return row;
        }
    }
    return -1;
}

class ScopedTableSorting
{
public:
    explicit ScopedTableSorting(QTableWidget* table)
        : _table(table), _sorting(table ? table->isSortingEnabled() : false) {
        if (_table) {
            _table->setSortingEnabled(false);
        }
    }

    ~ScopedTableSorting() {
        if (_table) {
            _table->setSortingEnabled(_sorting);
        }
    }

private:
    QTableWidget* _table = nullptr;
    bool _sorting = false;
};

struct PppCrdEntry {
    QString station;
    double xyz[3] = {0.0, 0.0, 0.0};
    double neu[3] = {0.0, 0.0, 0.0};
    QString antenna;
    QString receiver;
};

bool takeCrdToken(QString& rest, QString& token) {
    rest = rest.trimmed();
    if (rest.isEmpty()) {
        return false;
    }
    int pos = 0;
    while (pos < rest.size() && !rest.at(pos).isSpace()) {
        ++pos;
    }
    token = rest.left(pos);
    rest = rest.mid(pos);
    return true;
}

bool takeCrdDouble(QString& rest, double& value) {
    QString token;
    if (!takeCrdToken(rest, token)) {
        return false;
    }
    bool ok = false;
    const double parsed = token.toDouble(&ok);
    if (!ok || !std::isfinite(parsed)) {
        return false;
    }
    value = parsed;
    return true;
}

QMap<QString, PppCrdEntry> readPppCrdFile(const QString& filePath) {
    QMap<QString, PppCrdEntry> entries;
    if (filePath.trimmed().isEmpty()) {
        return entries;
    }

    QFile file(filePath);
    if (!file.open(QIODevice::ReadOnly | QIODevice::Text)) {
        return entries;
    }

    QTextStream in(&file);
    while (!in.atEnd()) {
        QString line = in.readLine().trimmed();
        if (line.isEmpty() || line.startsWith(QLatin1Char('#')) || line.startsWith(QLatin1Char('!'))) {
            continue;
        }

        QString station;
        QString rest;
        const int q1 = line.indexOf(QLatin1Char('"'));
        if (q1 >= 0) {
            const int q2 = line.indexOf(QLatin1Char('"'), q1 + 1);
            if (q2 < 0) {
                continue;
            }
            station = line.mid(q1 + 1, q2 - q1 - 1).trimmed();
            rest = line.mid(q2 + 1);
        } else {
            int pos = 0;
            while (pos < line.size() && !line.at(pos).isSpace()) {
                ++pos;
            }
            station = line.left(pos).trimmed();
            rest = line.mid(pos);
        }

        if (station.isEmpty()) {
            continue;
        }

        PppCrdEntry entry;
        entry.station = station.toUpper();
        if (!takeCrdDouble(rest, entry.xyz[0]) ||
            !takeCrdDouble(rest, entry.xyz[1]) ||
            !takeCrdDouble(rest, entry.xyz[2])) {
            continue;
        }
        if (!rest.trimmed().isEmpty()) {
            QString probe = rest;
            double neu[3] = {0.0, 0.0, 0.0};
            if (takeCrdDouble(probe, neu[0]) &&
                takeCrdDouble(probe, neu[1]) &&
                takeCrdDouble(probe, neu[2])) {
                entry.neu[0] = neu[0];
                entry.neu[1] = neu[1];
                entry.neu[2] = neu[2];
                rest = probe;
            }
        }

        QString antAndReceiver = rest.trimmed();
        if (!antAndReceiver.isEmpty()) {
            entry.antenna = antAndReceiver.left(20).toUpper();
            entry.receiver = antAndReceiver.mid(20).trimmed();
        }
        entries[entry.station] = entry;
    }
    return entries;
}

QStringList pppCrdStationCandidates(const QString& sourceId,
                                    const QString& mountpoint,
                                    const QString& label) {
    QStringList candidates;
    auto add = [&candidates](const QString& value) {
        const QString v = value.trimmed().toUpper();
        if (!v.isEmpty() && !candidates.contains(v)) {
            candidates << v;
        }
        if (v.size() > 9 && !candidates.contains(v.left(9))) {
            candidates << v.left(9);
        }
        if (v.size() > 4 && !candidates.contains(v.left(4))) {
            candidates << v.left(4);
        }
    };
    add(mountpoint);
    add(label);
    add(sourceId);
    return candidates;
}

std::array<double,3> xyzCovToNeuStd(const EllipsoidCoord& ell, const double cov[6]) {
    const double sinLat = std::sin(ell.lat);
    const double cosLat = std::cos(ell.lat);
    const double sinLon = std::sin(ell.lon);
    const double cosLon = std::cos(ell.lon);
    const double rot[3][3] = {
        {-sinLat * cosLon, -sinLat * sinLon,  cosLat},
        {-sinLon,           cosLon,           0.0},
        { cosLat * cosLon,  cosLat * sinLon,  sinLat}
    };
    const double q[3][3] = {
        {cov[0], cov[1], cov[2]},
        {cov[1], cov[3], cov[4]},
        {cov[2], cov[4], cov[5]}
    };
    std::array<double,3> stdNeu{};
    for (int i = 0; i < 3; ++i) {
        double v = 0.0;
        for (int j = 0; j < 3; ++j) {
            for (int k = 0; k < 3; ++k) {
                v += rot[i][j] * q[j][k] * rot[i][k];
            }
        }
        stdNeu[i] = (std::isfinite(v) && v >= 0.0) ? std::sqrt(v) : std::numeric_limits<double>::quiet_NaN();
    }
    return stdNeu;
}

class AddNtripCasterDialog : public QDialog
{
public:
    explicit AddNtripCasterDialog(const QJsonArray& savedCasters, QWidget* parent = nullptr)
        : QDialog(parent), _casters(casterEntriesFromJson(savedCasters)) {
        _ui.setupUi(this);
        _ui.ntripVersionCombo->setCurrentText("2");
        configureSortableResizableTable(_ui.sourceTableWidget);

        connect(_ui.casterPresetButton, &QPushButton::clicked, this, &AddNtripCasterDialog::chooseCasterPreset);
        connect(_ui.mapButton, &QPushButton::clicked, this, [this]() {
            QMessageBox::information(this, "Map", "Map view is not wired in this initial version.");
        });
        connect(_ui.closeButton, &QPushButton::clicked, this, &QDialog::reject);
        connect(_ui.getTableButton, &QPushButton::clicked, this, &AddNtripCasterDialog::requestTable);
        connect(_ui.selectButton, &QPushButton::clicked, this, &AddNtripCasterDialog::finishSelect);
    }

    QList<NtripSelectionRow> selectedRows() const { return _selected; }
    QJsonArray castersJson() const { return casterEntriesToJson(_casters); }

private:
    void chooseCasterPreset() {
        QDialog dialog(this);
        dialog.setWindowTitle("Casters table");
        dialog.resize(760, 360);

        QVBoxLayout* root = new QVBoxLayout(&dialog);
        QTableWidget* table = new QTableWidget(&dialog);
        table->setColumnCount(6);
        table->setHorizontalHeaderLabels({"Name", "Host", "Port", "NTRIP", "User", "Password"});
        configureSortableResizableTable(table);
        table->setSortingEnabled(false);
        table->setSelectionBehavior(QAbstractItemView::SelectRows);
        table->setSelectionMode(QAbstractItemView::SingleSelection);
        table->setEditTriggers(QAbstractItemView::DoubleClicked |
                               QAbstractItemView::EditKeyPressed |
                               QAbstractItemView::AnyKeyPressed);
        root->addWidget(table);

        auto setCell = [table](int row, int col, const QString& text) {
            table->setItem(row, col, new QTableWidgetItem(text));
        };
        auto insertCaster = [table, setCell](const NtripCasterEntry& entry) {
            const int row = table->rowCount();
            table->insertRow(row);
            setCell(row, 0, casterDisplayName(entry));
            setCell(row, 1, entry.host);
            setCell(row, 2, QString::number(entry.port));
            setCell(row, 3, normalizedNtripVersion(entry.ntripVersion));
            setCell(row, 4, entry.user);
            setCell(row, 5, entry.password);
        };
        for (const NtripCasterEntry& entry : _casters) {
            insertCaster(entry);
        }
        table->setSortingEnabled(true);
        if (table->rowCount() > 0) {
            table->selectRow(0);
        }

        QHBoxLayout* buttons = new QHBoxLayout();
        QPushButton* addButton = new QPushButton("Add", &dialog);
        QPushButton* removeButton = new QPushButton("Delete", &dialog);
        QPushButton* applyButton = new QPushButton("Apply", &dialog);
        QPushButton* saveButton = new QPushButton("Save", &dialog);
        QPushButton* closeButton = new QPushButton("Close", &dialog);
        buttons->addWidget(addButton);
        buttons->addWidget(removeButton);
        buttons->addStretch();
        buttons->addWidget(applyButton);
        buttons->addWidget(saveButton);
        buttons->addWidget(closeButton);
        root->addLayout(buttons);

        connect(addButton, &QPushButton::clicked, &dialog, [table, setCell]() {
            ScopedTableSorting sorting(table);
            const int row = table->rowCount();
            table->insertRow(row);
            setCell(row, 0, "New caster");
            setCell(row, 1, "");
            QTableWidgetItem* hostItem = table->item(row, 1);
            setCell(row, 2, "2101");
            setCell(row, 3, "2");
            setCell(row, 4, "");
            setCell(row, 5, "");
            table->setCurrentItem(hostItem);
            table->editItem(hostItem);
        });
        connect(removeButton, &QPushButton::clicked, &dialog, [table]() {
            const auto ranges = table->selectedRanges();
            QVector<int> rows;
            for (const QTableWidgetSelectionRange& range : ranges) {
                for (int row = range.topRow(); row <= range.bottomRow(); ++row) {
                    rows.push_back(row);
                }
            }
            std::sort(rows.begin(), rows.end(), std::greater<int>());
            rows.erase(std::unique(rows.begin(), rows.end()), rows.end());
            for (int row : rows) {
                table->removeRow(row);
            }
        });
        connect(saveButton, &QPushButton::clicked, &dialog, [this, table, &dialog]() {
            _casters = readCastersFromTable(table);
            dialog.accept();
        });
        connect(applyButton, &QPushButton::clicked, &dialog, [this, table, &dialog]() {
            const int row = table->currentRow();
            if (row < 0) {
                QMessageBox::information(&dialog, "Apply", "Please select one caster row.");
                return;
            }
            const NtripCasterEntry entry = casterFromTableRow(table, row);
            if (entry.host.trimmed().isEmpty() || entry.port <= 0) {
                QMessageBox::warning(&dialog, "Invalid", "Caster host and port are required.");
                return;
            }
            _casters = readCastersFromTable(table);
            applyCasterEntry(entry);
            dialog.accept();
        });
        connect(closeButton, &QPushButton::clicked, &dialog, &QDialog::reject);

        if (dialog.exec() != QDialog::Accepted) {
            return;
        }
    }

    QString tableText(const QTableWidget* table, int row, int col) const {
        const QTableWidgetItem* item = table->item(row, col);
        return item ? item->text() : QString();
    }

    NtripCasterEntry casterFromTableRow(const QTableWidget* table, int row) const {
        NtripCasterEntry entry;
        entry.name = tableText(table, row, 0).trimmed();
        entry.host = tableText(table, row, 1).trimmed();
        bool ok = false;
        entry.port = tableText(table, row, 2).trimmed().toInt(&ok);
        if (!ok) {
            entry.port = 0;
        }
        entry.ntripVersion = normalizedNtripVersion(tableText(table, row, 3));
        entry.user = tableText(table, row, 4);
        entry.password = tableText(table, row, 5);
        return entry;
    }

    QVector<NtripCasterEntry> readCastersFromTable(const QTableWidget* table) const {
        QVector<NtripCasterEntry> entries;
        for (int row = 0; row < table->rowCount(); ++row) {
            NtripCasterEntry entry = casterFromTableRow(table, row);
            if (entry.host.trimmed().isEmpty() || entry.port <= 0) {
                continue;
            }
            if (entry.name.trimmed().isEmpty()) {
                entry.name = casterDisplayName(entry);
            }
            entries.push_back(entry);
        }
        return entries;
    }

    void applyCasterEntry(const NtripCasterEntry& entry) {
        _ui.hostEdit->setText(entry.host.trimmed());
        _ui.portEdit->setText(QString::number(entry.port));
        _ui.userEdit->setText(entry.user);
        _ui.passEdit->setText(entry.password);
        _ui.ntripVersionCombo->setCurrentText(normalizedNtripVersion(entry.ntripVersion));
    }

    void requestTable() {
        ScopedTableSorting sorting(_ui.sourceTableWidget);
        _ui.sourceTableWidget->setRowCount(0);
        _selected.clear();
        _rawBuffer.clear();

        const QString host = _ui.hostEdit->text().trimmed();
        const int port = _ui.portEdit->text().toInt();
        if (host.isEmpty() || port <= 0) {
            QMessageBox::warning(this, "Invalid", "Please enter a valid host and port.");
            return;
        }

        const QString ntripVer = _ui.ntripVersionCombo->currentText();

        // Keep the socket local so the request ends with the dialog operation.
        QSslSocket socket;
        if (ntripVer == "2s") {
            socket.connectToHostEncrypted(host, static_cast<quint16>(port));
            if (!socket.waitForEncrypted(12000)) {
                QMessageBox::warning(this, "Get table failed", socket.errorString());
                return;
            }
            if (!socket.sslErrors().isEmpty()) {
                socket.ignoreSslErrors();
            }
        } else {
            socket.connectToHost(host, static_cast<quint16>(port));
            if (!socket.waitForConnected(10000)) {
                QMessageBox::warning(this, "Get table failed", socket.errorString());
                return;
            }
        }

        if (socket.state() != QAbstractSocket::ConnectedState) {
            QMessageBox::warning(this, "Get table failed", socket.errorString());
            return;
        }

        const bool v2 = (ntripVer == "2" || ntripVer == "2s");
        QByteArray req;
        req += v2 ? "GET / HTTP/1.1\r\n" : "GET / HTTP/1.0\r\n";
        if (v2) {
            req += "Host: " + host.toLatin1() + "\r\n";
            req += "Ntrip-Version: Ntrip/2.0\r\n";
            req += "User-Agent: " + rtpppNtripUserAgent() + "\r\n";
            if (!_ui.userEdit->text().isEmpty()) {
                const QByteArray auth = (decodedNtripCredential(_ui.userEdit->text()) + ":" +
                                         decodedNtripCredential(_ui.passEdit->text())).toBase64();
                req += "Authorization: Basic " + auth + "\r\n";
            }
            req += "Connection: close\r\n";
        } else {
            req += "User-Agent: " + rtpppNtripUserAgent() + "\r\n";
            req += "Host: " + host.toLatin1() + "\r\n";
            if (!_ui.userEdit->text().isEmpty() || !_ui.passEdit->text().isEmpty()) {
                const QByteArray auth = (decodedNtripCredential(_ui.userEdit->text()) + ":" +
                                         decodedNtripCredential(_ui.passEdit->text())).toBase64();
                req += "Authorization: Basic " + auth + "\r\n";
            }
        }
        req += "\r\n";
        if (socket.write(req) < 0 || !socket.waitForBytesWritten(5000)) {
            QMessageBox::warning(this, "Get table failed", socket.errorString());
            return;
        }

        while (socket.state() == QAbstractSocket::ConnectedState) {
            if (!socket.waitForReadyRead(5000)) {
                break;
            }
            _rawBuffer += socket.readAll();
        }
        _rawBuffer += socket.readAll();

        QByteArray payload = _rawBuffer;
        const int sep = payload.indexOf("\r\n\r\n");
        if (sep >= 0) {
            payload = payload.mid(sep + 4);
        }

        const QList<QByteArray> lines = payload.split('\n');
        for (const QByteArray& rawLine : lines) {
            const QString line = QString::fromUtf8(rawLine).trimmed();
            if (!line.startsWith("STR;")) {
                continue;
            }
            const QStringList p = line.split(';');
            if (p.size() < 12) {
                continue;
            }
            const int row = _ui.sourceTableWidget->rowCount();
            _ui.sourceTableWidget->insertRow(row);
            _ui.sourceTableWidget->setItem(row, 0, new QTableWidgetItem(p.value(1)));
            _ui.sourceTableWidget->setItem(row, 1, new QTableWidgetItem(p.value(2)));
            _ui.sourceTableWidget->setItem(row, 2, new QTableWidgetItem(p.value(3)));
            _ui.sourceTableWidget->setItem(row, 3, new QTableWidgetItem(p.value(4)));
            _ui.sourceTableWidget->setItem(row, 4, new QTableWidgetItem(p.value(5)));
            _ui.sourceTableWidget->setItem(row, 5, new QTableWidgetItem(p.value(6)));
            _ui.sourceTableWidget->setItem(row, 6, new QTableWidgetItem(p.value(9)));
            _ui.sourceTableWidget->setItem(row, 7, new QTableWidgetItem(p.value(10)));
        }
        if (_ui.sourceTableWidget->rowCount() == 0) {
            QMessageBox::information(this, "Get table",
                                     "No STR records parsed. Please verify caster credentials.");
        }
    }

    void finishSelect() {
        _selected.clear();
        const auto ranges = _ui.sourceTableWidget->selectedRanges();
        if (ranges.isEmpty()) {
            QMessageBox::information(this, "Select", "Please select at least one mountpoint row.");
            return;
        }

        QSet<int> selectedRowSet;
        for (const QTableWidgetSelectionRange& range : ranges) {
            for (int row = range.topRow(); row <= range.bottomRow(); ++row) {
                selectedRowSet.insert(row);
            }
        }

        const QString host = _ui.hostEdit->text().trimmed();
        const int port = _ui.portEdit->text().toInt();
        for (int row : selectedRowSet) {
            NtripSelectionRow out;
            out.cfg.host = host;
            out.cfg.port = port;
            out.cfg.mountPoint = _ui.sourceTableWidget->item(row, 0)->text();
            out.cfg.user = _ui.userEdit->text();
            out.cfg.password = _ui.passEdit->text();
            out.cfg.ntripVersion = _ui.ntripVersionCombo->currentText();
            out.cfg.reconnectMs = 3000;
            out.decoder = _ui.sourceTableWidget->item(row, 2)->text();
            out.lat = _ui.sourceTableWidget->item(row, 6)->text();
            out.lon = _ui.sourceTableWidget->item(row, 7)->text();
            out.nmea = "no";
            _selected.push_back(out);
        }
        accept();
    }

    Ui::AddNtripCasterDialog _ui;
    QByteArray _rawBuffer;
    QList<NtripSelectionRow> _selected;
    QVector<NtripCasterEntry> _casters;
};

class AddTcpDialog : public QDialog
{
public:
    explicit AddTcpDialog(QWidget* parent = nullptr) : QDialog(parent) {
        _ui.setupUi(this);
        connect(_ui.buttonBox, &QDialogButtonBox::accepted, this, &QDialog::accept);
        connect(_ui.buttonBox, &QDialogButtonBox::rejected, this, &QDialog::reject);
    }

    QJsonObject toConfig() const {
        QJsonObject o;
        o["host"] = _ui.hostEdit->text().trimmed();
        o["port"] = _ui.portEdit->text().toInt();
        o["reconnectMs"] = qMax(200, _ui.reconnectEdit->text().toInt());
        return o;
    }

private:
    Ui::AddTcpDialog _ui;
};

class AddMqttDialog : public QDialog
{
public:
    explicit AddMqttDialog(QWidget* parent = nullptr) : QDialog(parent) {
        _ui.setupUi(this);
        connect(_ui.buttonBox, &QDialogButtonBox::accepted, this, &QDialog::accept);
        connect(_ui.buttonBox, &QDialogButtonBox::rejected, this, &QDialog::reject);
    }

    QJsonObject toConfig() const {
        QJsonObject o;
        o["host"] = _ui.hostEdit->text().trimmed();
        o["port"] = _ui.portEdit->text().toInt();
        o["topic"] = _ui.topicEdit->text().trimmed();
        o["clientId"] = _ui.clientIdEdit->text().trimmed();
        o["user"] = _ui.userEdit->text();
        o["password"] = _ui.passEdit->text();
        o["reconnectMs"] = qMax(200, _ui.reconnectEdit->text().toInt());
        o["keepAliveSec"] = qMax(5, _ui.keepAliveEdit->text().toInt());
        return o;
    }

private:
    Ui::AddMqttDialog _ui;
};

} // namespace

MainWindow::MainWindow(const QString& settingsFilePath, QWidget* parent)
    : QMainWindow(parent),
      ui(new Ui::MainWindow),
      _settingsStore(settingsFilePath.trimmed().isEmpty()
                         ? QStringLiteral("rtppp_settings.ini")
                         : QFileInfo(settingsFilePath).absoluteFilePath()) {
    qInfo().noquote() << QStringLiteral("RT-PPP startup: application=%1 currentDir=%2 settings=%3 settingsAbs=%4 gfsPythonWorker=%5")
                         .arg(QCoreApplication::applicationFilePath())
                         .arg(QDir::currentPath())
                         .arg(_settingsStore.filePath())
                         .arg(QFileInfo(_settingsStore.filePath()).absoluteFilePath())
                         .arg(GfsZwdProvider::pythonWorkerAvailable() ? QStringLiteral("configured")
                                                                 : QStringLiteral("unavailable"));
    ui->setupUi(this);
    setMinimumSize(1460, 900);
    resize(1680, 980);
    setStyleSheet(
        "QMainWindow{background:#f4f7fb;color:#1f2937;}"
        "QGroupBox{border:1px solid #d8e1ee;border-radius:8px;margin-top:10px;padding-top:10px;background:#ffffff;}"
        "QGroupBox::title{subcontrol-origin:margin;left:10px;padding:0 6px;color:#0f172a;font-weight:600;}"
        "QTableWidget,QPlainTextEdit,QLineEdit,QComboBox,QSpinBox,QDoubleSpinBox{"
        "background:#ffffff;border:1px solid #d1d9e6;border-radius:6px;color:#1f2937;"
        "selection-background-color:#dbeafe;selection-color:#1f2937;}"
        "QTableWidget::item:selected{color:#1f2937;background:#dbeafe;}"
        "QHeaderView::section{background:#eaf0f8;color:#334155;padding:6px;border:0px;}"
        "QPushButton{background:#2563eb;color:white;border:0;border-radius:6px;padding:6px 12px;}"
        "QPushButton:hover{background:#1d4ed8;}"
        "QPushButton:pressed{background:#1e40af;}"
        "QTabWidget::pane{border:1px solid #d8e1ee;background:#ffffff;}"
        "QTabBar::tab{background:#e7edf6;padding:7px 14px;border-top-left-radius:6px;border-top-right-radius:6px;margin-right:2px;}"
        "QTabBar::tab:selected{background:#ffffff;color:#0f172a;font-weight:600;}"
    );
    setupNtripSettingsUi();
    setupDataPreparationUi();
    setupStatusUi();
    setupPppSettingsUi();
    setupPppResultsUi();
    setupOthersUi();
    setupSignalBindings();
    _logFlushTimer = new QTimer(this);
    _logFlushTimer->setSingleShot(true);
    _logFlushTimer->setInterval(100);
    connect(_logFlushTimer, &QTimer::timeout, this, &MainWindow::flushPendingLogs);
    _pppDisplayRefreshTimer = new QTimer(this);
    _pppDisplayRefreshTimer->setSingleShot(true);
    _pppDisplayRefreshTimer->setInterval(500);
    connect(_pppDisplayRefreshTimer, &QTimer::timeout, this, [this]() {
        if (_closing) return;
        refreshPppStationSelector();
        refreshPppNeuPlot();
        refreshZtdPlot();
        refreshZwdPlot();
    });
    _skyPlotRefreshTimer = new QTimer(this);
    _skyPlotRefreshTimer->setSingleShot(true);
    _skyPlotRefreshTimer->setInterval(1000);
    connect(_skyPlotRefreshTimer, &QTimer::timeout, this, [this]() {
        if (_closing) return;
        updateStatusDetailsForRow(rowForStreamId(selectedStatusPlotStreamId()));
    });
    initializeOutputWorker();
    initializeRtcmProcessingService();
    initializePppWorker();
    QString streamLoadError;
    if (!loadStreamsFromSettings(&streamLoadError)) {
        appendLog("SYSTEM", QString("Configuration load failed: %1").arg(streamLoadError));
    }
    rebuildStreamsTable();
    QString settingsValidationError;
    if (!_settingsStore.validate(&settingsValidationError)) {
        appendLog("SYSTEM", QString("Configuration validation failed; PPP and output settings were not applied: %1")
                              .arg(settingsValidationError));
    } else {
        loadDataPreparationFromSettings();
        applyDataPreparationSelection(false);
        loadPppSettingsFromSettings();
        applyPppSettings(false);
    }
}

bool MainWindow::startHeadless(QString* error) {
    if (error) error->clear();
    QString validationError;
    if (!_settingsStore.validate(&validationError)) {
        const QString message = QStringLiteral("Headless startup failed: invalid configuration: %1")
                                    .arg(validationError);
        qCritical().noquote() << message;
        if (error) *error = message;
        return false;
    }
    if (_postModeCombo && _postModeCombo->currentData().toString() == QLatin1String("post")) {
        const QString message = QStringLiteral(
            "Headless mode currently supports saved realtime streams only; "
            "set ppp/processingMode=realtime in the configuration.");
        qCritical().noquote() << message;
        if (error) *error = message;
        return false;
    }
    if (_rows.isEmpty()) {
        const QString message = QStringLiteral("Headless startup failed: no streams are configured in %1")
                                    .arg(QFileInfo(settingsFilePath()).absoluteFilePath());
        qCritical().noquote() << message;
        if (error) *error = message;
        return false;
    }

    startAllStreams();
    int started = 0;
    for (const StreamRow& row : _rows) {
        if (row.client && row.client->runtimeState().running) {
            ++started;
        }
    }
    if (started == 0) {
        const QString message = QStringLiteral("Headless startup failed: no configured stream client could be started");
        qCritical().noquote() << message;
        stopAllStreams();
        if (error) *error = message;
        return false;
    }
    qInfo().noquote() << QStringLiteral("RT-PPP headless realtime startup complete: streams=%1 config=%2")
                          .arg(started)
                          .arg(QFileInfo(settingsFilePath()).absoluteFilePath());
    return true;
}

MainWindow::~MainWindow() {
    _closing = true;
    if (_postService) {
        _postService->cancel();
    }
    if (_postThread) {
        _postThread->quit();
        _postThread->wait();
        _postThread = nullptr;
        _postService = nullptr;
    }
    if (_logFlushTimer) {
        _logFlushTimer->stop();
    }
    if (_pppDisplayRefreshTimer) {
        _pppDisplayRefreshTimer->stop();
    }
    if (_skyPlotRefreshTimer) {
        _skyPlotRefreshTimer->stop();
    }
    flushPendingLogs();
    for (StreamRow& row : _rows) {
        stopAndDeleteClient(row);
    }
    shutdownRtcmProcessingService();
    shutdownPppWorker();
    shutdownOutputWorker();
    delete ui;
}

void MainWindow::initializeOutputWorker() {
    if (_outputThread || _outputWorker) {
        return;
    }
    _outputThread = new QThread(this);
    _outputWorker = new OutputWorker();
    _outputWorker->moveToThread(_outputThread);
    connect(_outputThread, &QThread::finished, _outputWorker, &QObject::deleteLater);
    _outputThread->start();
}

void MainWindow::shutdownOutputWorker() {
    QThread* thread = _outputThread;
    OutputWorker* worker = _outputWorker;
    if (!thread) {
        _outputWorker = nullptr;
        return;
    }
    if (worker && thread->isRunning()) {
        QMetaObject::invokeMethod(worker, [worker]() { worker->shutdown(); },
                                  Qt::BlockingQueuedConnection);
    }
    thread->quit();
    thread->wait();
    delete thread;
    _outputThread = nullptr;
    _outputWorker = nullptr;
}

void MainWindow::finalizePppJsonOutput() {
    OutputWorker* worker = _outputWorker;
    QThread* thread = _outputThread;
    if (!worker || !thread || !thread->isRunning()) {
        return;
    }
    const Qt::ConnectionType type = worker->thread() == QThread::currentThread()
        ? Qt::DirectConnection : Qt::BlockingQueuedConnection;
    QMetaObject::invokeMethod(worker, [worker]() { worker->finalizePppJson(); }, type);
}

void MainWindow::configureOutputWorker() {
    OutputWorker* worker = _outputWorker;
    if (!worker) {
        return;
    }
    OutputWorker::Config config;
    config.sp3Enabled = _prepConfig.enabled && _prepConfig.sp3Enabled;
    config.sp3Directory = _prepConfig.sp3Directory;
    config.sp3SampleSec = _prepConfig.sp3SampleSec;
    config.rinexEnabled = _prepConfig.enabled && _prepConfig.rnxEnabled;
    config.rinexDirectory = _prepConfig.rnxDirectory;
    config.pppJsonEnabled = _pppJsonEnabled;
    config.pppJsonDirectory = _pppJsonOutputDirectory;
    config.pppJsonReorderBufferMs = _pppJsonReorderBufferMs;
    config.pppJsonFlushIntervalMs = _pppJsonFlushIntervalMs;
    config.ztdDirectory = _ztdOutputDirectory;
    config.logFilePath = resolvedPrepLogFilePath();
    config.telemetryDirectory = _telemetryEnabled
        ? _prepConfig.telemetryDirectory : QString();
    for (const StreamRow& row : _rows) {
        const QString mount = row.config.value(QStringLiteral("mountPoint")).toString().trimmed();
        config.telemetrySourceNames.insert(row.id, mount.isEmpty() ? row.id : mount);
    }
    config.arLogFilePath = _pppOptions.arLogFile;
    worker->configure(config);
}

void MainWindow::postOutputSp3(int gpsWeek,
                               double gpsSow,
                               const QString& prn,
                               const std::array<double, 3>& position,
                               double clockSec,
                               const std::array<double, 3>& velocity,
                               double clockRate) {
    OutputWorker* worker = _outputWorker;
    if (!worker) {
        return;
    }
    worker->enqueueSp3(gpsWeek, gpsSow, prn, position, clockSec,
                       velocity, clockRate);
}

void MainWindow::postOutputRinex(
    int gpsWeek, double gpsSow, const QMap<QString, QMap<QString, double>>& observations) {
    OutputWorker* worker = _outputWorker;
    if (!worker) {
        return;
    }
    worker->enqueueRinex(gpsWeek, gpsSow, observations);
}

void MainWindow::postOutputPppJson(const QString& stationId,
                                   const PppSolution& solution,
                                   const std::array<double, 3>& referenceEcef) {
    OutputWorker* worker = _outputWorker;
    if (!worker) {
        return;
    }
    worker->enqueuePppJson(stationId, solution, referenceEcef);
}

void MainWindow::postOutputZtd(const QString& source,
                               const QString& station,
                               const PppSolution& solution) {
    OutputWorker* worker = _outputWorker;
    if (!worker) {
        return;
    }
    worker->enqueueZtd(source, station, solution);
}

void MainWindow::postOutputLog(const QString& text) {
    OutputWorker* worker = _outputWorker;
    if (!worker || text.isEmpty()) {
        return;
    }
    worker->enqueueLog(text);
}

void MainWindow::postOutputArLog(const PppSolution& solution) {
    OutputWorker* worker = _outputWorker;
    if (!worker || solution.arLogLines.isEmpty()) {
        return;
    }
    worker->enqueueArLog(solution);
}

void MainWindow::initializeRtcmProcessingService() {
    if (_rtcmProcessingThread || _rtcmProcessingService) {
        return;
    }
    _rtcmProcessingThread = new QThread(this);
    _rtcmProcessingService = new RtcmProcessingService();
    _rtcmProcessingService->moveToThread(_rtcmProcessingThread);
    connect(_rtcmProcessingThread, &QThread::finished,
            _rtcmProcessingService, &QObject::deleteLater);
    connect(_rtcmProcessingService, &RtcmProcessingService::ready,
            this, &MainWindow::drainRtcmProcessingResults, Qt::QueuedConnection);
    _rtcmProcessingThread->start();
}

void MainWindow::shutdownRtcmProcessingService() {
    QThread* thread = _rtcmProcessingThread;
    RtcmProcessingService* service = _rtcmProcessingService;
    if (!thread) {
        _rtcmProcessingService = nullptr;
        return;
    }
    if (service) {
        if (thread->isRunning()) {
            QMetaObject::invokeMethod(service, [service]() {
                service->drainAllIngress();
                service->requestStop();
            }, Qt::BlockingQueuedConnection);
            drainRtcmProcessingResults();
            QMetaObject::invokeMethod(service, "shutdown", Qt::BlockingQueuedConnection);
        } else {
            service->requestStop();
        }
    }
    thread->quit();
    thread->wait();
    delete thread;
    _rtcmProcessingThread = nullptr;
    _rtcmProcessingService = nullptr;
}

void MainWindow::clearRtcmProcessingState() {
    RtcmProcessingService* service = _rtcmProcessingService;
    if (!service) {
        return;
    }
    const Qt::ConnectionType type = service->thread() == QThread::currentThread()
        ? Qt::DirectConnection : Qt::BlockingQueuedConnection;
    QMetaObject::invokeMethod(service, "clearAll", type);
}

void MainWindow::configureRtcmProcessingService() {
    RtcmProcessingService* service = _rtcmProcessingService;
    if (!service) {
        return;
    }
    const int sampleSec = qMax(1, _prepConfig.observationSampleSec);
    RtcmProcessingService::DataPreparationConfig config;
    config.enabled = _prepConfig.enabled;
    config.broadcastStreamId = _prepConfig.broadcastStreamId;
    config.orbitStreamId = _prepConfig.orbitStreamId;
    config.clockStreamId = _prepConfig.clockStreamId;
    config.codeBiasStreamId = _prepConfig.codeBiasStreamId;
    config.phaseBiasStreamId = _prepConfig.phaseBiasStreamId;
    config.obsStreamId = _prepConfig.obsStreamId;
    config.sp3Enabled = _prepConfig.sp3Enabled;
    config.sp3SampleSec = _prepConfig.sp3SampleSec;
    config.rinexEnabled = _prepConfig.rnxEnabled;
    config.broadcastMountpoint = streamMountpointById(_prepConfig.broadcastStreamId);
    config.orbitMountpoint = streamMountpointById(_prepConfig.orbitStreamId);
    const Qt::ConnectionType type = service->thread() == QThread::currentThread()
        ? Qt::DirectConnection : Qt::BlockingQueuedConnection;
    QMetaObject::invokeMethod(service, [service, sampleSec, config]() {
        service->setObservationSampleSec(sampleSec);
        service->setDataPreparationConfig(config);
    }, type);
}

void MainWindow::setTelemetryOutputDirectory(const QString& directory) {
    const QString outputDirectory = directory.trimmed();
    _telemetryEnabled = !outputDirectory.isEmpty();
}

void MainWindow::postTelemetryMsm(
    const QString& sourceId,
    const QDateTime& receivedUtc,
    const QDateTime& epochUtc,
    const RtcmMsmMessage& message,
    const QMap<QString, QPair<double, double>>& skyBySatellite) {
    OutputWorker* worker = _outputWorker;
    if (!_telemetryEnabled || !worker) return;
    worker->enqueueTelemetryMsm(sourceId, receivedUtc, epochUtc,
                                message, skyBySatellite);
}

void MainWindow::postTelemetrySsr(const QString& sourceId,
                                  const QDateTime& receivedUtc,
                                  const QDateTime& epochUtc,
                                  const RtcmSsrMessage& message,
                                  const RtcmSsrSatelliteData& satellite,
                                  bool adopted,
                                  const QString& reason) {
    OutputWorker* worker = _outputWorker;
    if (!_telemetryEnabled || !worker) return;
    worker->enqueueTelemetrySsr(sourceId, receivedUtc, epochUtc,
                                message, satellite, adopted, reason);
}

void MainWindow::postTelemetryReception(const QString& sourceId,
                                        const QDateTime& receivedUtc,
                                        const QString& messageFamily,
                                        int messageType,
                                        const QChar& system,
                                        const QString& status,
                                        const QString& reason,
                                        const QString& detail) {
    OutputWorker* worker = _outputWorker;
    if (!_telemetryEnabled || !worker) return;
    worker->enqueueTelemetryReception(sourceId, receivedUtc, messageFamily,
                                       messageType, system, status, reason, detail);
}

void MainWindow::postTelemetryPppResiduals(const QString& sourceId,
                                           const PppSolution& solution) {
    OutputWorker* worker = _outputWorker;
    if (!_telemetryEnabled || !worker) return;
    worker->enqueueTelemetryPppResiduals(sourceId, solution);
}

void MainWindow::initializePppWorker() {
    if (_pppRuntimeService) {
        return;
    }
    _pppRuntimeService = new PppRuntimeService(this);
    connect(_pppRuntimeService, &PppRuntimeService::ready,
            this, &MainWindow::drainPppWorkerResults, Qt::QueuedConnection);
    if (_rtcmProcessingService) {
        // PppRuntimeService product ingress only schedules work on the PPP
        // thread and protects its GFS cache with a mutex, so explicit direct
        // delivery prevents high-rate SSR from occupying the GUI event queue.
        connect(_rtcmProcessingService, &RtcmProcessingService::broadcastAvailable,
                _pppRuntimeService, &PppRuntimeService::updateBroadcast,
                Qt::DirectConnection);
        connect(_rtcmProcessingService, &RtcmProcessingService::ssrAvailable,
                _pppRuntimeService, &PppRuntimeService::updateSsrMessage,
                Qt::DirectConnection);
    }
    connect(_pppRuntimeService, &PppRuntimeService::gfsInitializationChanged,
            this, [this](const QString& sourceId, bool pending) {
        RtcmProcessingService* service = _rtcmProcessingService;
        if (!service || !service->thread()->isRunning()) {
            return;
        }
        QMetaObject::invokeMethod(service, [service, sourceId, pending]() {
            service->setObservationSuppressed(sourceId, pending);
        }, Qt::QueuedConnection);
    }, Qt::QueuedConnection);
    connect(_pppRuntimeService, &PppRuntimeService::gfsProgress,
            this, [this](const QString& sourceId, const QString& message) {
        appendLog(QStringLiteral("GFS/%1").arg(sourceId), message);
    }, Qt::QueuedConnection);
    _pppRuntimeService->configure(_pppOptions, _pppRuntimePolicy, _pppSelection);
}

void MainWindow::shutdownPppWorker() {
    PppRuntimeService* service = _pppRuntimeService;
    if (!service) {
        return;
    }
    _pppRuntimeActive = false;
    service->shutdown();
    delete service;
    _pppRuntimeService = nullptr;
}

void MainWindow::startPppRuntime() {
    if (!_pppRuntimeService) {
        _pppRuntimeActive = false;
        return;
    }
    _pppRuntimeActive = false;
    _pppRuntimeService->configure(_pppOptions, _pppRuntimePolicy, _pppSelection);
    _pppRuntimeService->start();
    _pppRuntimeActive = true;
}

void MainWindow::stopPppRuntimeAndClear() {
    _pppRuntimeActive = false;
    if (!_pppRuntimeService) {
        return;
    }
    _pppRuntimeService->stopAndClear();
}

void MainWindow::postPppOptions() {
    if (_pppRuntimeService) {
        _pppRuntimeService->setOptions(_pppOptions);
    }
}

void MainWindow::postPppRuntimePolicy() {
    if (_pppRuntimeService) {
        _pppRuntimeService->setRuntimePolicy(_pppRuntimePolicy);
    }
}

void MainWindow::postPppStreamSelection() {
    if (_pppRuntimeService) {
        _pppRuntimeService->setStreamSelection(_pppSelection);
    }
}

void MainWindow::requestPppWorkerStopNow() {
    if (_pppRuntimeService) {
        _pppRuntimeService->requestStop();
    }
}

void MainWindow::drainPppWorkerResults() {
    if (!_pppRuntimeService) {
        return;
    }
    QVector<PppWorker::SolutionEvent> events;
    QStringList logs;
    _pppRuntimeService->takePending(events, logs);

    for (const QString& log : logs) {
        appendLog("PPP", log);
    }

    for (const PppWorker::SolutionEvent& event : events) {
        postOutputArLog(event.solution);
        updatePppResults(event.sourceId, event.solution);
        if (shouldLogNow(_lastPppLog, 2)) {
            const double stdX = (std::isfinite(event.solution.cov[0]) && event.solution.cov[0] >= 0.0)
                ? std::sqrt(event.solution.cov[0]) : std::numeric_limits<double>::quiet_NaN();
            const double stdY = (std::isfinite(event.solution.cov[3]) && event.solution.cov[3] >= 0.0)
                ? std::sqrt(event.solution.cov[3]) : std::numeric_limits<double>::quiet_NaN();
            const double stdZ = (std::isfinite(event.solution.cov[5]) && event.solution.cov[5] >= 0.0)
                ? std::sqrt(event.solution.cov[5]) : std::numeric_limits<double>::quiet_NaN();
            std::array<double,3> stdRef = {event.solution.xyz[0], event.solution.xyz[1], event.solution.xyz[2]};
            const PppCrdStationOptions stationOptions = _pppOptions.crdStationsBySource.value(event.sourceId);
            if (stationOptions.valid) {
                stdRef = {stationOptions.aprXyz[0], stationOptions.aprXyz[1], stationOptions.aprXyz[2]};
            } else if (_pppOptions.hasApriori()) {
                stdRef = {_pppOptions.aprXyz[0], _pppOptions.aprXyz[1], _pppOptions.aprXyz[2]};
            }
            const std::array<double,3> stdNeu = xyzCovToNeuStd(xyzToEll(stdRef), event.solution.cov);
            appendLog(event.sourceId,
                      QString("PPP solution: week=%1 sow=%2 sat=%3 HDOP=%4 XYZ=(%5,%6,%7) STD_XYZ=(%8,%9,%10) NEU=(%11,%12,%13) STD_NEU=(%14,%15,%16) ZTD=%17 ZTDstd=%18 diag=%19")
                          .arg(event.solution.gpsWeek)
                          .arg(event.solution.gpsSow, 0, 'f', 3)
                          .arg(event.solution.numSat)
                          .arg(event.solution.hDop, 0, 'f', 2)
                          .arg(event.solution.xyz[0], 0, 'f', 3)
                          .arg(event.solution.xyz[1], 0, 'f', 3)
                          .arg(event.solution.xyz[2], 0, 'f', 3)
                          .arg(stdX, 0, 'f', 4)
                          .arg(stdY, 0, 'f', 4)
                          .arg(stdZ, 0, 'f', 4)
                          .arg(event.solution.neu[0], 0, 'f', 4)
                          .arg(event.solution.neu[1], 0, 'f', 4)
                          .arg(event.solution.neu[2], 0, 'f', 4)
                          .arg(stdNeu[0], 0, 'f', 4)
                          .arg(stdNeu[1], 0, 'f', 4)
                          .arg(stdNeu[2], 0, 'f', 4)
                          .arg(event.solution.ztd, 0, 'f', 4)
                          .arg(event.solution.ztdStdev, 0, 'f', 4)
                          .arg(event.solution.diagSummary.left(900)));
        }
        if (!event.solution.arLogLines.isEmpty() && shouldLogNow(_lastPppArLog, 2)) {
            appendLog(event.sourceId, QString("PPP-AR: %1 lines")
                                          .arg(event.solution.arLogLines.size()));
            for (const QString& line : event.solution.arLogLines) {
                appendLog(event.sourceId, line);
            }
        }
        if (!event.yawDebug.isEmpty() && shouldLogNow(_lastPppYawLog, 2)) {
            appendLog(event.sourceId,
                      QString("SSR-yaw align: count=%1 (showing up to 3)").arg(event.yawDebug.size()));
            const int maxLog = qMin(3, event.yawDebug.size());
            for (int i = 0; i < maxLog; ++i) {
                const auto& dbg = event.yawDebug[i];
                appendLog(event.sourceId,
                          QString("SSR-yaw %1 mt=%2 uiIdx=%3 uiSec=%4 epochSow=%5 recvSow=%6 dtRaw=%7 dtUsed=%8 yaw=%9deg yawRate=%10deg/s")
                              .arg(dbg.prn)
                              .arg(dbg.messageType)
                              .arg(dbg.updateIntervalIndex)
                              .arg(dbg.uiSec, 0, 'f', 1)
                              .arg(dbg.epochSow, 0, 'f', 1)
                              .arg(dbg.recvSow, 0, 'f', 1)
                              .arg(dbg.dtRawSec, 0, 'f', 2)
                              .arg(dbg.dtSec, 0, 'f', 2)
                              .arg(dbg.yawRad * kRhoDeg, 0, 'f', 3)
                              .arg(dbg.yawRateRad * kRhoDeg, 0, 'f', 5));
            }
        }
        for (const QString& warn : event.antexWarnings) {
            appendLog("PPP", warn);
        }
    }
}

void MainWindow::resetRuntimeStateForFreshStart() {
    stopPppRuntimeAndClear();
    clearRtcmProcessingState();
    _msmSummaries.clear();
    _satCountHistory.clear();
    _preparedStates.clear();
    _lastMsmMessages.clear();
    _liveSkyPoints.clear();
    _skyReceiverPositionValid.clear();
    _skyHistory.clear();
    _lastSkyHistoryEpochMs.clear();
    _lastSkyHistoryBucketMs.clear();
    _lastRtcmSummaryLog = QDateTime();
    _lastCorrSummaryLog = QDateTime();
    _lastSp3Log = QDateTime();
    _lastRnxLog = QDateTime();
    _lastPppLog = QDateTime();
    _lastPppYawLog = QDateTime();
    _lastPppArLog = QDateTime();
    _lastPppSource.clear();
    _lastPppSolution = PppSolution();
    _lastPppSolutionBySource.clear();
    _pppStationNameBySource.clear();
    _pppHistory.clear();

    refreshDataPreparationTable();
    refreshStatusPlotStationSelector();
    refreshPppStationSelector();
    refreshPppNeuPlot();
    refreshZtdPlot();
    refreshZwdPlot();
    updateStatusDetailsForRow(rowForStreamId(selectedStatusPlotStreamId()));
}

void MainWindow::setupNtripSettingsUi() {
    QVBoxLayout* root = qobject_cast<QVBoxLayout*>(ui->ntrip_settings->layout());
    if (!root) {
        root = new QVBoxLayout(ui->ntrip_settings);
    }
    root->setContentsMargins(10, 10, 10, 10);
    root->setSpacing(8);

    _addNtripButton = ui->ntrip_settings->findChild<QPushButton*>("btnAddNtrip");
    _addTcpButton = ui->ntrip_settings->findChild<QPushButton*>("btnAddTcp");
    _addMqttButton = ui->ntrip_settings->findChild<QPushButton*>("btnAddMqtt");
    _removeButton = ui->ntrip_settings->findChild<QPushButton*>("btnRemoveStream");
    _streamsTable = ui->ntrip_settings->findChild<QTableWidget*>("streamsTableWidget");
    _globalStartButton = ui->centralwidget->findChild<QPushButton*>("btnGlobalStart");
    _globalStopButton = ui->centralwidget->findChild<QPushButton*>("btnGlobalStop");
    _saveConfigButton = ui->centralwidget->findChild<QPushButton*>("btnSaveConfig");
    _reloadConfigButton = ui->centralwidget->findChild<QPushButton*>("btnReloadConfig");

    if (!_addNtripButton) _addNtripButton = new QPushButton("Add NTRIP Caster...");
    if (!_addTcpButton) _addTcpButton = new QPushButton("Add TCP/IP...");
    if (!_addMqttButton) _addMqttButton = new QPushButton("Add MQTT...");
    if (!_removeButton) _removeButton = new QPushButton("Remove");
    if (!_globalStartButton) _globalStartButton = new QPushButton("Start");
    if (!_globalStopButton) _globalStopButton = new QPushButton("Stop");
    if (!_saveConfigButton) _saveConfigButton = new QPushButton("Save Config");
    if (!_reloadConfigButton) _reloadConfigButton = new QPushButton("Reload Config");
    if (!_streamsTable) _streamsTable = new QTableWidget(0, 8, this);

    _streamCountLabel = new QLabel("Streams: 0");

    if (_addNtripButton->parent() != ui->ntrip_settings) {
        QHBoxLayout* topButtons = new QHBoxLayout;
        topButtons->addWidget(_addNtripButton);
        topButtons->addWidget(_addTcpButton);
        topButtons->addWidget(_addMqttButton);
        topButtons->addWidget(_removeButton);
        topButtons->addStretch();
        topButtons->addWidget(_streamCountLabel);
        root->addLayout(topButtons);
        root->addWidget(_streamsTable, 1);
    } else {
        root->addWidget(_streamCountLabel);
    }

    _streamsTable->setSelectionBehavior(QAbstractItemView::SelectRows);
    _streamsTable->setSelectionMode(QAbstractItemView::SingleSelection);
    _streamsTable->setEditTriggers(QAbstractItemView::NoEditTriggers);
    _streamsTable->setColumnCount(8);
    _streamsTable->setHorizontalHeaderLabels(
        QStringList() << "data url"
                      << "mountpoint"
                      << "decoder"
                      << "lat"
                      << "long"
                      << "nmea"
                      << "ntrip"
                      << "bytes");
    _streamsTable->verticalHeader()->setVisible(false);
    configureSortableResizableTable(_streamsTable);
}

void MainWindow::setupDataPreparationUi() {
    if (ui->MainTab) {
        const int tabIndex = ui->MainTab->indexOf(ui->data_perparation);
        if (tabIndex >= 0) {
            ui->MainTab->setTabText(tabIndex, "Data Preparation");
        }
    }

    QVBoxLayout* root = qobject_cast<QVBoxLayout*>(ui->data_perparation->layout());
    if (!root) {
        root = new QVBoxLayout(ui->data_perparation);
    }
    root->setContentsMargins(10, 10, 10, 10);
    root->setSpacing(8);

    QGroupBox* streamGroup = new QGroupBox("SSR correction mapping", ui->data_perparation);
    QFormLayout* form = new QFormLayout(streamGroup);
    form->setLabelAlignment(Qt::AlignRight | Qt::AlignVCenter);
    form->setContentsMargins(10, 10, 10, 10);
    form->setSpacing(6);

    _prepEnableCheck = new QCheckBox("Enable SSR correction preparation", streamGroup);
    _prepBroadcastCombo = new QComboBox(streamGroup);
    _prepOrbitCombo = new QComboBox(streamGroup);
    _prepClockCombo = new QComboBox(streamGroup);
    _prepCodeBiasCombo = new QComboBox(streamGroup);
    _prepPhaseBiasCombo = new QComboBox(streamGroup);

    form->addRow(QString(), _prepEnableCheck);
    form->addRow("Broadcast ephemeris stream*", _prepBroadcastCombo);
    form->addRow("Orbit correction stream", _prepOrbitCombo);
    form->addRow("Clock correction stream", _prepClockCombo);
    form->addRow("Code-bias stream", _prepCodeBiasCombo);
    form->addRow("Phase-bias stream", _prepPhaseBiasCombo);

    _prepSummaryLabel = new QLabel("No mapping applied.", ui->data_perparation);
    _prepSummaryLabel->setWordWrap(true);

    _prepTable = new QTableWidget(0, 8, ui->data_perparation);
    _prepTable->setEditTriggers(QAbstractItemView::NoEditTriggers);
    _prepTable->setSelectionMode(QAbstractItemView::NoSelection);
    _prepTable->setHorizontalHeaderLabels(
        QStringList() << "sat/nav"
                      << "broadcast IOD"
                      << "orbit"
                      << "clock"
                      << "code bias"
                      << "phase bias"
                      << "A/C datum"
                      << "updated");
    configureSortableResizableTable(_prepTable);

    root->addWidget(streamGroup);

    QGroupBox* outputGroup = new QGroupBox("Outputs", ui->data_perparation);
    QFormLayout* outputForm = new QFormLayout(outputGroup);
    outputForm->setLabelAlignment(Qt::AlignRight | Qt::AlignVCenter);
    outputForm->setContentsMargins(10, 10, 10, 10);
    outputForm->setSpacing(6);

    _sp3EnableCheck = new QCheckBox("Write SP3 for SSR-corrected broadcast ephemeris", outputGroup);
    _sp3PathEdit = new QLineEdit(outputGroup);
    _sp3SampleSpin = new QSpinBox(outputGroup);
    _sp3SampleSpin->setRange(1, 3600);
    _sp3SampleSpin->setValue(_prepConfig.sp3SampleSec);

    _prepObsCombo = new QComboBox(outputGroup);
    _obsSampleCombo = new QComboBox(outputGroup);
    _obsSampleCombo->addItem("1 s", 1);
    _obsSampleCombo->addItem("5 s", 5);
    _obsSampleCombo->addItem("10 s", 10);
    _obsSampleCombo->addItem("30 s", 30);
    _obsSampleCombo->setCurrentIndex(_obsSampleCombo->findData(_prepConfig.observationSampleSec));
    _rnxEnableCheck = new QCheckBox("Write RINEX3 OBS for corrected observations", outputGroup);
    _rnxPathEdit = new QLineEdit(outputGroup);
    _telemetryEnableCheck = new QCheckBox("Write structured MSM/SSR/PPP telemetry", outputGroup);
    _telemetryPathEdit = new QLineEdit(outputGroup);
    _prepLogPathEdit = new QLineEdit(outputGroup);
    _prepLogPathEdit->setPlaceholderText("Optional log output path, directory, or file");
    _prepLogBrowseButton = new QPushButton("Browse...", outputGroup);
    QHBoxLayout* logPathRow = new QHBoxLayout();
    logPathRow->addWidget(_prepLogPathEdit, 1);
    logPathRow->addWidget(_prepLogBrowseButton);

    outputForm->addRow(QString(), _sp3EnableCheck);
    outputForm->addRow("SP3/CLK output directory", _sp3PathEdit);
    outputForm->addRow("SP3 sample interval (s)", _sp3SampleSpin);
    outputForm->addRow("Observation stream", _prepObsCombo);
    outputForm->addRow("Processing epoch interval", _obsSampleCombo);
    outputForm->addRow(QString(), _rnxEnableCheck);
    outputForm->addRow("RINEX output directory", _rnxPathEdit);
    outputForm->addRow(QString(), _telemetryEnableCheck);
    outputForm->addRow("Telemetry output directory", _telemetryPathEdit);
    outputForm->addRow("Log output path", logPathRow);

    root->addWidget(outputGroup);
    root->addWidget(_prepSummaryLabel);
    root->addWidget(_prepTable, 1);
}

void MainWindow::setupStatusUi() {
    QVBoxLayout* root = qobject_cast<QVBoxLayout*>(ui->rtdata_status->layout());
    if (!root) {
        root = new QVBoxLayout(ui->rtdata_status);
    }
    root->setContentsMargins(10, 10, 10, 10);
    root->setSpacing(8);

    _statusTable = ui->rtdata_status->findChild<QTableWidget*>("statusTableWidget");
    _logTextEdit = ui->rtdata_status->findChild<QPlainTextEdit*>("logTextEdit");
    if (!_statusTable) {
        _statusTable = new QTableWidget(0, 5, this);
        _statusTable->setHorizontalHeaderLabels(QStringList() << "stream" << "type" << "state" << "bytes" << "last error");
        root->addWidget(_statusTable, 1);
    }

    _statusTable->setSelectionMode(QAbstractItemView::NoSelection);
    _statusTable->setEditTriggers(QAbstractItemView::NoEditTriggers);
    _statusTable->setSelectionMode(QAbstractItemView::SingleSelection);
    _statusTable->setSelectionBehavior(QAbstractItemView::SelectRows);
    configureSortableResizableTable(_statusTable);

    if (!_logTextEdit) {
        _logTextEdit = new QPlainTextEdit(this);
        _logTextEdit->setReadOnly(true);
    }

    QHBoxLayout* plotSelectorRow = new QHBoxLayout();
    plotSelectorRow->addWidget(new QLabel("Plot station:", ui->rtdata_status));
    _statusPlotStationCombo = new QComboBox(ui->rtdata_status);
    _statusPlotStationCombo->setMinimumWidth(260);
    plotSelectorRow->addWidget(_statusPlotStationCombo, 1);
    plotSelectorRow->addStretch();
    root->addLayout(plotSelectorRow);

    _statusDetailTabs = new QTabWidget(ui->rtdata_status);

    QWidget* logWidget = new QWidget(_statusDetailTabs);
    QVBoxLayout* logLayout = new QVBoxLayout(logWidget);
    logLayout->setContentsMargins(0, 0, 0, 0);
    logLayout->addWidget(_logTextEdit);
    _statusDetailTabs->addTab(logWidget, "Logs");

    QWidget* obsWidget = new QWidget(_statusDetailTabs);
    QVBoxLayout* obsLayout = new QVBoxLayout(obsWidget);
    _obsDetailLabel = new QLabel("Select one station above to inspect observation details.", obsWidget);
    _obsDetailLabel->setWordWrap(true);
    _obsCountTable = new QTableWidget(0, 3, obsWidget);
    _obsCountTable->setHorizontalHeaderLabels(QStringList() << "System" << "Satellites" << "Signals/Cells");
    _obsCountTable->setEditTriggers(QAbstractItemView::NoEditTriggers);
    _obsCountTable->setSelectionMode(QAbstractItemView::NoSelection);
    _obsCountTable->setAlternatingRowColors(true);
    _obsCountTable->setShowGrid(false);
    _obsCountTable->setStyleSheet("QTableWidget{background:#ffffff;} QTableWidget::item{padding:4px;color:#1f2937;}");
    _obsCountTable->verticalHeader()->setVisible(false);
    configureSortableResizableTable(_obsCountTable);
    _obsTrendScene = new QGraphicsScene(obsWidget);
    _obsTrendView = new QGraphicsView(_obsTrendScene, obsWidget);
    _obsTrendView->setMinimumHeight(220);
    _obsTrendView->setRenderHint(QPainter::Antialiasing, true);
    _obsTrendView->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    _obsTrendView->setVerticalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    obsLayout->addWidget(_obsDetailLabel);
    obsLayout->addWidget(_obsCountTable, 1);
    obsLayout->addWidget(_obsTrendView);
    _statusDetailTabs->addTab(obsWidget, "Observation Stats");

    QWidget* snrWidget = new QWidget(_statusDetailTabs);
    QVBoxLayout* snrLayout = new QVBoxLayout(snrWidget);
    _snrPlotScene = new QGraphicsScene(snrWidget);
    _snrPlotView = new QGraphicsView(_snrPlotScene, snrWidget);
    _snrPlotView->setMinimumHeight(300);
    _snrPlotView->setRenderHint(QPainter::Antialiasing, true);
    _snrPlotView->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    _snrPlotView->setVerticalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    snrLayout->addWidget(_snrPlotView);
    _statusDetailTabs->addTab(snrWidget, "SNR Plot");

    QWidget* skyWidget = new QWidget(_statusDetailTabs);
    QVBoxLayout* skyLayout = new QVBoxLayout(skyWidget);
    QHBoxLayout* skyControlRow = new QHBoxLayout();
    skyControlRow->addWidget(new QLabel("History:", skyWidget));
    _skyHistoryWindowCombo = new QComboBox(skyWidget);
    _skyHistoryWindowCombo->addItem("5 min", 5 * 60);
    _skyHistoryWindowCombo->addItem("1 hour", 60 * 60);
    _skyHistoryWindowCombo->addItem("6 hours", 6 * 60 * 60);
    _skyHistoryWindowCombo->addItem("1 day", 24 * 60 * 60);
    skyControlRow->addWidget(_skyHistoryWindowCombo);
    skyControlRow->addStretch();
    _skyPlotScene = new QGraphicsScene(skyWidget);
    _skyPlotView = new QGraphicsView(_skyPlotScene, skyWidget);
    _skyPlotView->setMinimumHeight(420);
    _skyPlotView->setRenderHint(QPainter::Antialiasing, true);
    _skyPlotView->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    _skyPlotView->setVerticalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    skyLayout->addLayout(skyControlRow);
    skyLayout->addWidget(_skyPlotView);
    _statusDetailTabs->addTab(skyWidget, "Sky Plot");

    root->addWidget(_statusDetailTabs, 2);

    _logTextEdit->setMaximumBlockCount(1000);
    _logTextEdit->setPlaceholderText("Connection logs...");

    connect(_statusTable, &QTableWidget::itemSelectionChanged, this, [this]() {
        const QModelIndexList selected = _statusTable->selectionModel()->selectedRows();
        if (selected.isEmpty()) {
            if (!_statusPlotStationCombo || _statusPlotStationCombo->currentIndex() < 0) {
                updateStatusDetailsForRow(-1);
            }
            return;
        }
        const QString streamId = tableRowStreamId(_statusTable, selected.first().row());
        if (!isStatusPlotStreamSource(streamId)) {
            updateStatusDetailsForRow(rowForStreamId(selectedStatusPlotStreamId()));
            return;
        }
        if (_statusPlotStationCombo) {
            const int comboIndex = _statusPlotStationCombo->findData(streamId);
            if (comboIndex >= 0 && comboIndex != _statusPlotStationCombo->currentIndex()) {
                QSignalBlocker blocker(_statusPlotStationCombo);
                _statusPlotStationCombo->setCurrentIndex(comboIndex);
            }
        }
        const int streamRow = rowForStreamId(streamId);
        updateStatusDetailsForRow(streamRow);
    });
}

void MainWindow::setupPppSettingsUi() {
    QVBoxLayout* root = qobject_cast<QVBoxLayout*>(ui->ppp_settings->layout());
    if (!root) {
        root = new QVBoxLayout(ui->ppp_settings);
    }
    root->setContentsMargins(10, 10, 10, 10);
    root->setSpacing(8);

    QScrollArea* scroll = new QScrollArea(ui->ppp_settings);
    scroll->setWidgetResizable(true);
    QWidget* content = new QWidget(scroll);
    QGridLayout* grid = new QGridLayout(content);
    grid->setContentsMargins(0, 0, 0, 0);
    grid->setHorizontalSpacing(16);
    grid->setVerticalSpacing(12);

    QLabel* hint = new QLabel("PPP stream inputs follow Data Preparation selections (broadcast/orbit/clock/code/phase/obs).",
                              content);
    hint->setWordWrap(true);
    grid->addWidget(hint, 0, 0, 1, 2);

    QGroupBox* modeGroup = new QGroupBox("Processing Mode", content);
    QGridLayout* modeGrid = new QGridLayout(modeGroup);
    _postModeCombo = new QComboBox(modeGroup);
    _postModeCombo->addItem("Realtime streams", "realtime");
    _postModeCombo->addItem("Post-processing files", "post");
    modeGrid->addWidget(new QLabel("Mode", modeGroup), 0, 0);
    modeGrid->addWidget(_postModeCombo, 0, 1);
    _postInputPanel = new QWidget(modeGroup);
    QGridLayout* postGrid = new QGridLayout(_postInputPanel);
    postGrid->setContentsMargins(0, 4, 0, 0);
    auto addPathRow = [this](QGridLayout* layout, QWidget* parent, int row,
                             const QString& label, QLineEdit*& edit,
                             QPushButton*& button, bool multi = false) {
        edit = new QLineEdit(parent);
        button = new QPushButton("Browse...", parent);
        layout->addWidget(new QLabel(label, parent), row, 0);
        layout->addWidget(edit, row, 1);
        layout->addWidget(button, row, 2);
        connect(button, &QPushButton::clicked, this, [this, edit, multi, label]() {
            if (multi) {
                const QStringList files = QFileDialog::getOpenFileNames(this, label, edit->text(), "SSR/Text (*.ssr *.SSR *.txt);;All files (*.*)");
                if (!files.isEmpty()) edit->setText(files.join(QLatin1Char(';')));
            } else {
                const QString file = QFileDialog::getOpenFileName(this, label, edit->text(), "GNSS files (*.*)");
                if (!file.isEmpty()) edit->setText(file);
            }
        });
    };
    _postJobNameEdit = new QLineEdit(_postInputPanel);
    _postJobNameEdit->setPlaceholderText("Defaults to RINEX marker or filename");
    postGrid->addWidget(new QLabel("Job name", _postInputPanel), 0, 0);
    postGrid->addWidget(_postJobNameEdit, 0, 1, 1, 2);
    QPushButton* obsBrowse = nullptr;
    addPathRow(postGrid, _postInputPanel, 1, "RINEX OBS", _postObsPathEdit, obsBrowse);
    QPushButton* navBrowse = nullptr;
    addPathRow(postGrid, _postInputPanel, 2, "Broadcast NAV", _postNavPathEdit, navBrowse);
    _postProductModeCombo = new QComboBox(_postInputPanel);
    _postProductModeCombo->addItem("Broadcast NAV + SSR", "broadcast");
    _postProductModeCombo->addItem("Broadcast NAV + SP3 + CLK", "precise");
    postGrid->addWidget(new QLabel("Product mode", _postInputPanel), 3, 0);
    postGrid->addWidget(_postProductModeCombo, 3, 1, 1, 2);

    _postSsrProductPanel = new QWidget(_postInputPanel);
    QGridLayout* ssrGrid = new QGridLayout(_postSsrProductPanel);
    ssrGrid->setContentsMargins(0, 0, 0, 0);
    QPushButton* ssrBrowse = nullptr;
    addPathRow(ssrGrid, _postSsrProductPanel, 0, "SSR files", _postSsrPathsEdit, ssrBrowse, true);
    postGrid->addWidget(_postSsrProductPanel, 4, 0, 1, 3);

    _postPreciseProductPanel = new QWidget(_postInputPanel);
    QGridLayout* preciseGrid = new QGridLayout(_postPreciseProductPanel);
    preciseGrid->setContentsMargins(0, 0, 0, 0);
    QPushButton* sp3Browse = nullptr;
    QPushButton* clkBrowse = nullptr;
    addPathRow(preciseGrid, _postPreciseProductPanel, 0, "SP3", _postSp3PathEdit, sp3Browse);
    addPathRow(preciseGrid, _postPreciseProductPanel, 1, "CLK", _postClkPathEdit, clkBrowse);
    postGrid->addWidget(_postPreciseProductPanel, 5, 0, 1, 3);
    _postSampleCombo = new QComboBox(_postInputPanel);
    _postSampleCombo->addItem("1 s", 1);
    _postSampleCombo->addItem("5 s", 5);
    _postSampleCombo->addItem("10 s", 10);
    _postSampleCombo->addItem("30 s", 30);
    _postSampleCombo->setCurrentIndex(1);
    postGrid->addWidget(new QLabel("Post-processing interval", _postInputPanel), 6, 0);
    postGrid->addWidget(_postSampleCombo, 6, 1, 1, 2);
    QHBoxLayout* postActions = new QHBoxLayout();
    _postStartButton = new QPushButton("Start", _postInputPanel);
    _postCancelButton = new QPushButton("Cancel", _postInputPanel);
    _postProgress = new QProgressBar(_postInputPanel);
    _postProgress->setRange(0, 100);
    _postProgress->setValue(0);
    _postStatusLabel = new QLabel("Idle", _postInputPanel);
    postActions->addWidget(_postStartButton);
    postActions->addWidget(_postCancelButton);
    postActions->addWidget(_postProgress, 1);
    postActions->addWidget(_postStatusLabel, 2);
    postGrid->addLayout(postActions, 7, 0, 1, 3);
    applyPostProductModeUi();
    modeGrid->addWidget(_postInputPanel, 1, 0, 1, 2);
    grid->addWidget(modeGroup, 1, 0, 1, 2);

    QGroupBox* staGroup = new QGroupBox("PPP Stations", content);
    QVBoxLayout* staLayout = new QVBoxLayout(staGroup);
    QLabel* staHint = new QLabel("Enable stations to be processed in parallel. Disabled stations keep cached results for plotting.",
                                 staGroup);
    staHint->setWordWrap(true);
    _pppStationTable = new QTableWidget(0, 3, staGroup);
    _pppStationTable->setHorizontalHeaderLabels(QStringList() << "Enable" << "Station" << "Source ID");
    _pppStationTable->horizontalHeader()->setSectionResizeMode(0, QHeaderView::ResizeToContents);
    _pppStationTable->horizontalHeader()->setSectionResizeMode(1, QHeaderView::Stretch);
    _pppStationTable->horizontalHeader()->setSectionResizeMode(2, QHeaderView::Stretch);
    _pppStationTable->setEditTriggers(QAbstractItemView::NoEditTriggers);
    _pppStationTable->setSelectionMode(QAbstractItemView::NoSelection);
    _pppStationTable->verticalHeader()->setVisible(false);
    _pppStationTable->setAlternatingRowColors(true);
    configureSortableResizableTable(_pppStationTable);
    staLayout->addWidget(staHint);
    staLayout->addWidget(_pppStationTable);
    grid->addWidget(staGroup, 2, 0, 1, 2);

    QGroupBox* sysGroup = new QGroupBox("GNSS Systems", content);
    QHBoxLayout* sysLayout = new QHBoxLayout(sysGroup);
    _pppUseGpsCheck = new QCheckBox("GPS", sysGroup);
    _pppUseGloCheck = new QCheckBox("GLONASS", sysGroup);
    _pppUseGalCheck = new QCheckBox("Galileo", sysGroup);
    _pppUseBdsCheck = new QCheckBox("BeiDou", sysGroup);
    sysLayout->addWidget(_pppUseGpsCheck);
    sysLayout->addWidget(_pppUseGloCheck);
    sysLayout->addWidget(_pppUseGalCheck);
    sysLayout->addWidget(_pppUseBdsCheck);
    sysLayout->addStretch();
    grid->addWidget(sysGroup, 3, 0, 1, 2);

    QGroupBox* lcGroup = new QGroupBox("Observation Combinations", content);
    QGridLayout* lcGrid = new QGridLayout(lcGroup);
    auto makeLcCombo = [lcGroup]() {
        QComboBox* combo = new QComboBox(lcGroup);
        combo->addItems(QStringList()
                        << "P3&L3"
                        << "P3"
                        << "L3"
                        << "Pi&Li"
                        << "Pi"
                        << "P1&L1"
                        << "P1");
        combo->setSizeAdjustPolicy(QComboBox::AdjustToContents);
        return combo;
    };
    _pppLcGpsCombo = makeLcCombo();
    _pppLcGloCombo = makeLcCombo();
    _pppLcGalCombo = makeLcCombo();
    _pppLcBdsCombo = makeLcCombo();
    _pppSignalPrioritiesEdit = new QLineEdit(lcGroup);
    _pppSignalPrioritiesEdit->setPlaceholderText("G:12&CWPSLX R:12&CP E:1&CBX E:5&QIX C:26&IQX");
    lcGrid->addWidget(new QLabel("GPS", lcGroup), 0, 0);
    lcGrid->addWidget(_pppLcGpsCombo, 0, 1);
    lcGrid->addWidget(new QLabel("GLONASS", lcGroup), 0, 2);
    lcGrid->addWidget(_pppLcGloCombo, 0, 3);
    lcGrid->addWidget(new QLabel("Galileo", lcGroup), 1, 0);
    lcGrid->addWidget(_pppLcGalCombo, 1, 1);
    lcGrid->addWidget(new QLabel("BeiDou", lcGroup), 1, 2);
    lcGrid->addWidget(_pppLcBdsCombo, 1, 3);
    lcGrid->addWidget(new QLabel("Signal priorities", lcGroup), 2, 0);
    lcGrid->addWidget(_pppSignalPrioritiesEdit, 2, 1, 1, 3);
    lcGrid->setColumnStretch(1, 1);
    lcGrid->setColumnStretch(3, 1);
    grid->addWidget(lcGroup, 4, 0, 1, 2);

    QGroupBox* obsGroup = new QGroupBox("Observation / Residual Settings", content);
    QFormLayout* obsForm = new QFormLayout(obsGroup);
    _pppMinObsSpin = new QSpinBox(obsGroup);
    _pppMinObsSpin->setRange(3, 20);
    _pppSigmaC1Spin = new QDoubleSpinBox(obsGroup);
    _pppSigmaC1Spin->setRange(0.01, 50.0);
    _pppSigmaC1Spin->setDecimals(3);
    _pppSigmaL1Spin = new QDoubleSpinBox(obsGroup);
    _pppSigmaL1Spin->setRange(0.0001, 5.0);
    _pppSigmaL1Spin->setDecimals(4);
    _pppMaxResC1Spin = new QDoubleSpinBox(obsGroup);
    _pppMaxResC1Spin->setRange(0.1, 100.0);
    _pppMaxResC1Spin->setDecimals(3);
    _pppMaxResL1Spin = new QDoubleSpinBox(obsGroup);
    _pppMaxResL1Spin->setRange(0.001, 10.0);
    _pppMaxResL1Spin->setDecimals(4);
    _pppMinEleSpin = new QDoubleSpinBox(obsGroup);
    _pppMinEleSpin->setRange(0.0, 40.0);
    _pppMinEleSpin->setDecimals(2);
    _pppEleWgtCodeCheck = new QCheckBox("Enable", obsGroup);
    _pppEleWgtPhaseCheck = new QCheckBox("Enable", obsGroup);
    _pppPseudoObsIonoCheck = new QCheckBox("Enable VTEC/GIM pseudo-observations", obsGroup);
    obsForm->addRow("Minimum observations", _pppMinObsSpin);
    obsForm->addRow("Code sigma (m)", _pppSigmaC1Spin);
    obsForm->addRow("Phase sigma (m)", _pppSigmaL1Spin);
    obsForm->addRow("Code max residual (m)", _pppMaxResC1Spin);
    obsForm->addRow("Phase max residual (m)", _pppMaxResL1Spin);
    obsForm->addRow("Minimum elevation (deg)", _pppMinEleSpin);
    obsForm->addRow("Elevation weighting (code)", _pppEleWgtCodeCheck);
    obsForm->addRow("Elevation weighting (phase)", _pppEleWgtPhaseCheck);
    obsForm->addRow(QString(), _pppPseudoObsIonoCheck);
    grid->addWidget(obsGroup, 5, 0);

    auto createDouble = [](QWidget* parent, double minV, double maxV, int decimals) {
        QDoubleSpinBox* box = new QDoubleSpinBox(parent);
        box->setRange(minV, maxV);
        box->setDecimals(decimals);
        box->setSingleStep(std::pow(10.0, -decimals));
        return box;
    };

    QGroupBox* filterGroup = new QGroupBox("Filter Noise / A-priori Sigmas", content);
    QGridLayout* filterGrid = new QGridLayout(filterGroup);
    filterGrid->setContentsMargins(8, 8, 8, 8);
    filterGrid->setHorizontalSpacing(16);
    QFormLayout* filterLeft = new QFormLayout();
    QFormLayout* filterRight = new QFormLayout();
    _pppAprSigXSpin = createDouble(filterGroup, 0.0, 1.0e6, 3);
    _pppAprSigYSpin = createDouble(filterGroup, 0.0, 1.0e6, 3);
    _pppAprSigZSpin = createDouble(filterGroup, 0.0, 1.0e6, 3);
    _pppNoiseXSpin = createDouble(filterGroup, 0.0, 1000.0, 4);
    _pppNoiseYSpin = createDouble(filterGroup, 0.0, 1000.0, 4);
    _pppNoiseZSpin = createDouble(filterGroup, 0.0, 1000.0, 4);
    _pppAprSigClkSpin = createDouble(filterGroup, 0.0, 1.0e9, 3);
    _pppAprSigTrpSpin = createDouble(filterGroup, 0.0, 100.0, 4);
    _pppNoiseTrpSpin = createDouble(filterGroup, 0.0, 10.0, 5);
    _pppAprSigAmbSpin = createDouble(filterGroup, 0.0, 1.0e8, 3);
    _pppSeedingSpin = createDouble(filterGroup, 0.0, 36000.0, 1);
    filterLeft->addRow("Apriori sigma X (m)", _pppAprSigXSpin);
    filterLeft->addRow("Apriori sigma Y (m)", _pppAprSigYSpin);
    filterLeft->addRow("Apriori sigma Z (m)", _pppAprSigZSpin);
    filterLeft->addRow("Process noise X (m/sqrt(s))", _pppNoiseXSpin);
    filterLeft->addRow("Process noise Y (m/sqrt(s))", _pppNoiseYSpin);
    filterLeft->addRow("Process noise Z (m/sqrt(s))", _pppNoiseZSpin);
    filterRight->addRow("Apriori sigma clock (m)", _pppAprSigClkSpin);
    filterRight->addRow("Apriori sigma tropo (m)", _pppAprSigTrpSpin);
    filterRight->addRow("Process noise tropo", _pppNoiseTrpSpin);
    filterRight->addRow("Apriori sigma ambiguity", _pppAprSigAmbSpin);
    filterRight->addRow("Seeding time (s)", _pppSeedingSpin);
    filterGrid->addLayout(filterLeft, 0, 0);
    filterGrid->addLayout(filterRight, 0, 1);
    filterGrid->setColumnStretch(0, 1);
    filterGrid->setColumnStretch(1, 1);
    grid->addWidget(filterGroup, 5, 1);

    QGroupBox* tropoGroup = new QGroupBox("Troposphere / GFS ZWD Prior", content);
    QFormLayout* tropoForm = new QFormLayout(tropoGroup);
    _pppTropoModeCombo = new QComboBox(tropoGroup);
    _pppTropoModeCombo->addItem("Original Saastamoinen wet prior", "original");
    _pppTropoModeCombo->addItem("GFS ZWD initialization", "gfs");
    _pppGfsMethodCombo = new QComboBox(tropoGroup);
    _pppGfsMethodCombo->addItem("Profile vertical integral", "profile");
    _pppGfsMethodCombo->addItem("Askne-Nordius (diagnostic)", "askne");
    _pppMappingModeCombo = new QComboBox(tropoGroup);
    _pppMappingModeCombo->addItem("Default (1/sin E)", "simple");
    _pppMappingModeCombo->addItem("GPT3 + VMF3", "gpt3");
    _pppGfsCachePathEdit = new QLineEdit(tropoGroup);
    _pppGfsCachePathEdit->setPlaceholderText("Local GFS GRIB2 cache directory");
    _pppGfsCacheBrowseButton = new QPushButton("Browse...", tropoGroup);
    QHBoxLayout* gfsCacheRow = new QHBoxLayout();
    gfsCacheRow->addWidget(_pppGfsCachePathEdit, 1);
    gfsCacheRow->addWidget(_pppGfsCacheBrowseButton);
    _pppGfsTimeoutSpin = new QDoubleSpinBox(tropoGroup);
    _pppGfsTimeoutSpin->setRange(30.0, 300.0);
    _pppGfsTimeoutSpin->setDecimals(0);
    _pppGfsTimeoutSpin->setSuffix(" s");
    tropoForm->addRow("Initialization", _pppTropoModeCombo);
    tropoForm->addRow("GFS ZWD method", _pppGfsMethodCombo);
    tropoForm->addRow("Troposphere mapping", _pppMappingModeCombo);
    tropoForm->addRow("GFS cache", gfsCacheRow);
    tropoForm->addRow("Download timeout", _pppGfsTimeoutSpin);
    grid->addWidget(tropoGroup, 8, 0, 1, 2);
    connect(_pppGfsCacheBrowseButton, &QPushButton::clicked, this, [this]() {
        const QString path = QFileDialog::getExistingDirectory(
            this, "Select GFS cache directory",
            _pppGfsCachePathEdit ? _pppGfsCachePathEdit->text() : QString());
        if (!path.isEmpty() && _pppGfsCachePathEdit) _pppGfsCachePathEdit->setText(path);
    });

    QGroupBox* posGroup = new QGroupBox("A-priori Position / Receiver Eccentricity", content);
    QFormLayout* posForm = new QFormLayout(posGroup);
    _pppAprXSpin = createDouble(posGroup, -5.0e7, 5.0e7, 4);
    _pppAprYSpin = createDouble(posGroup, -5.0e7, 5.0e7, 4);
    _pppAprZSpin = createDouble(posGroup, -5.0e7, 5.0e7, 4);
    _pppNeuNSpin = createDouble(posGroup, -10.0, 10.0, 4);
    _pppNeuESpin = createDouble(posGroup, -10.0, 10.0, 4);
    _pppNeuUSpin = createDouble(posGroup, -10.0, 10.0, 4);
    posForm->addRow("Apriori X (m)", _pppAprXSpin);
    posForm->addRow("Apriori Y (m)", _pppAprYSpin);
    posForm->addRow("Apriori Z (m)", _pppAprZSpin);
    posForm->addRow("NEU eccentricity N (m)", _pppNeuNSpin);
    posForm->addRow("NEU eccentricity E (m)", _pppNeuESpin);
    posForm->addRow("NEU eccentricity U (m)", _pppNeuUSpin);
    grid->addWidget(posGroup, 6, 0);

    QGroupBox* antGroup = new QGroupBox("Antenna / Ocean Tide (optional)", content);
    QFormLayout* antForm = new QFormLayout(antGroup);
    _pppCrdPathEdit = new QLineEdit(antGroup);
    _pppCrdPathEdit->setPlaceholderText("Optional coordinate file (*.CRD)");
    _pppCrdBrowseButton = new QPushButton("Browse...", antGroup);
    QHBoxLayout* crdRow = new QHBoxLayout();
    crdRow->addWidget(_pppCrdPathEdit, 1);
    crdRow->addWidget(_pppCrdBrowseButton);
    _pppUseAntexCheck = new QCheckBox("Enable ANTEX PCV/PCO", antGroup);
    _pppAntexPathEdit = new QLineEdit(antGroup);
    _pppAntexPathEdit->setPlaceholderText("Optional, e.g. igs20.atx");
    _pppAntexBrowseButton = new QPushButton("Browse...", antGroup);
    QHBoxLayout* antexRow = new QHBoxLayout();
    antexRow->addWidget(_pppAntexPathEdit, 1);
    antexRow->addWidget(_pppAntexBrowseButton);
    _pppRoverAntennaEdit = new QLineEdit(antGroup);
    _pppRoverAntennaEdit->setPlaceholderText("Receiver antenna name in ANTEX, e.g. LEIAR25.R4");
    _pppUseApcCheck = new QCheckBox("SSR reference is APC (SSRA)", antGroup);
    _pppUseOceanCheck = new QCheckBox("Enable ocean tide (BLQ)", antGroup);
    _pppBlqPathEdit = new QLineEdit(antGroup);
    _pppBlqPathEdit->setPlaceholderText("Optional BLQ file");
    _pppBlqBrowseButton = new QPushButton("Browse...", antGroup);
    QHBoxLayout* blqRow = new QHBoxLayout();
    blqRow->addWidget(_pppBlqPathEdit, 1);
    blqRow->addWidget(_pppBlqBrowseButton);
    _pppOceanStationEdit = new QLineEdit(antGroup);
    _pppOceanStationEdit->setPlaceholderText("Station code matching BLQ (optional)");
    _pppVtecPathEdit = new QLineEdit(antGroup);
    _pppVtecPathEdit->setPlaceholderText("Optional VTEC correction file (*.ssr)");
    _pppVtecBrowseButton = new QPushButton("Browse...", antGroup);
    QHBoxLayout* vtecRow = new QHBoxLayout();
    vtecRow->addWidget(_pppVtecPathEdit, 1);
    vtecRow->addWidget(_pppVtecBrowseButton);
    _pppArLogPathEdit = new QLineEdit(antGroup);
    _pppArLogPathEdit->setPlaceholderText("Optional PPP-AR log file path");
    _pppArLogBrowseButton = new QPushButton("Browse...", antGroup);
    QHBoxLayout* arLogRow = new QHBoxLayout();
    arLogRow->addWidget(_pppArLogPathEdit, 1);
    arLogRow->addWidget(_pppArLogBrowseButton);

    antForm->addRow("Coordinates file", crdRow);
    antForm->addRow(QString(), _pppUseAntexCheck);
    antForm->addRow("ANTEX file", antexRow);
    antForm->addRow("Receiver antenna name", _pppRoverAntennaEdit);
    antForm->addRow(QString(), _pppUseApcCheck);
    antForm->addRow(QString(), _pppUseOceanCheck);
    antForm->addRow("BLQ file", blqRow);
    antForm->addRow("Ocean station code", _pppOceanStationEdit);
    antForm->addRow("VTEC corr file", vtecRow);
    antForm->addRow("PPP-AR log file", arLogRow);
    grid->addWidget(antGroup, 6, 1);

    QGroupBox* arGroup = new QGroupBox("PPP-AR Thresholds / Screening", content);
    QFormLayout* arForm = new QFormLayout(arGroup);
    _pppArAlgorithmCombo = new QComboBox(arGroup);
    _pppArAlgorithmCombo->addItem("LAMBDA (MW + NL)", "LAMBDA");
    _pppArAlgorithmCombo->addItem("BIE", "BIE");
    _pppArBieMinEpochsSpin = new QSpinBox(arGroup);
    _pppArBieMinEpochsSpin->setRange(1, 10000);
    _pppArBieMinSatSpin = new QSpinBox(arGroup);
    _pppArBieMinSatSpin->setRange(2, 100);
    _pppArBieMaxFracSpin = createDouble(arGroup, 0.0, 1.0, 5);
    _pppArBieMaxSigSpin = createDouble(arGroup, 0.0, 10.0, 5);
    _pppArBieConstraintSigmaSpin = createDouble(arGroup, 0.000001, 1.0, 6);
    _pppArBiePersistConstraintsCheck = new QCheckBox("Persist BIE constraints to next epoch", arGroup);
    _pppArBiePersistConfirmEpochsSpin = new QSpinBox(arGroup);
    _pppArBiePersistConfirmEpochsSpin->setRange(1, 100);
    _pppArBiePersistMaxVerticalSpin = createDouble(arGroup, 0.001, 2.0, 3);
    _pppArBiePersistMaxZtdSpin = createDouble(arGroup, 0.001, 1.0, 3);
    _pppArEnableCheck = new QCheckBox("Enable PPP-AR", arGroup);
    _pppArUseGpsCheck = new QCheckBox("Include GPS in AR", arGroup);
    _pppArUseGloCheck = new QCheckBox("Include GLONASS in AR", arGroup);
    _pppArUseGalCheck = new QCheckBox("Include Galileo in AR", arGroup);
    _pppArUseBdsCheck = new QCheckBox("Include BeiDou in AR", arGroup);
    _pppArMinEleSpin = createDouble(arGroup, 0.0, 40.0, 2);
    _pppArMinMwSamplesSpin = new QSpinBox(arGroup);
    _pppArMinMwSamplesSpin->setRange(1, 200);
    _pppArMinFixPairsSpin = new QSpinBox(arGroup);
    _pppArMinFixPairsSpin->setRange(1, 20);
    _pppArWlMaxDevSpin = createDouble(arGroup, 0.0, 2.0, 4);
    _pppArWlMaxSigSpin = createDouble(arGroup, 0.0, 1.0, 4);
    _pppArNlRatioSpin = createDouble(arGroup, 1.0, 20.0, 3);
    _pppArNlBootSpin = createDouble(arGroup, -1.0, 1.0, 4);
    _pppArNlMaxDevSpin = createDouble(arGroup, 0.0, 5.0, 4);
    _pppArConstraintSigmaSpin = createDouble(arGroup, 0.0001, 10.0, 4);
    _pppArUseHistoryAidCheck = new QCheckBox("Use history-aided re-fix", arGroup);
    _pppArHistMinFixEpochsSpin = new QSpinBox(arGroup);
    _pppArHistMinFixEpochsSpin->setRange(0, 10000);
    _pppArHistMinLockEpochsSpin = new QSpinBox(arGroup);
    _pppArHistMinLockEpochsSpin->setRange(0, 100000);
    _pppArPartialWarmupLockEpochsSpin = new QSpinBox(arGroup);
    _pppArPartialWarmupLockEpochsSpin->setRange(0, 10000);
    _pppArHistRelaxFactorSpin = createDouble(arGroup, 1.0, 3.0, 3);
    arForm->addRow("AR algorithm", _pppArAlgorithmCombo);
    arForm->addRow("BIE minimum epochs", _pppArBieMinEpochsSpin);
    arForm->addRow("BIE minimum ambiguities", _pppArBieMinSatSpin);
    arForm->addRow("BIE max fractional part", _pppArBieMaxFracSpin);
    arForm->addRow("BIE max sigma", _pppArBieMaxSigSpin);
    arForm->addRow("BIE constraint sigma", _pppArBieConstraintSigmaSpin);
    arForm->addRow(QString(), _pppArBiePersistConstraintsCheck);
    arForm->addRow("BIE persist confirmation epochs", _pppArBiePersistConfirmEpochsSpin);
    arForm->addRow("BIE persist max U correction (m)", _pppArBiePersistMaxVerticalSpin);
    arForm->addRow("BIE persist max ZTD correction (m)", _pppArBiePersistMaxZtdSpin);
    arForm->addRow(QString(), _pppArEnableCheck);
    arForm->addRow(QString(), _pppArUseGpsCheck);
    arForm->addRow(QString(), _pppArUseGloCheck);
    arForm->addRow(QString(), _pppArUseGalCheck);
    arForm->addRow(QString(), _pppArUseBdsCheck);
    arForm->addRow(QString(), _pppArUseHistoryAidCheck);
    arForm->addRow("AR min elevation (deg)", _pppArMinEleSpin);
    arForm->addRow("MW minimum samples", _pppArMinMwSamplesSpin);
    arForm->addRow("NL minimum DD pairs", _pppArMinFixPairsSpin);
    arForm->addRow("WL max |MW-int| (cycle)", _pppArWlMaxDevSpin);
    arForm->addRow("WL max sigma (cycle)", _pppArWlMaxSigSpin);
    arForm->addRow("NL ratio threshold", _pppArNlRatioSpin);
    arForm->addRow("NL bootstrap threshold", _pppArNlBootSpin);
    arForm->addRow("NL max |float-fix| (cycle)", _pppArNlMaxDevSpin);
    arForm->addRow("History min fixed epochs", _pppArHistMinFixEpochsSpin);
    arForm->addRow("History min lock epochs", _pppArHistMinLockEpochsSpin);
    arForm->addRow("Warm-up lock epochs", _pppArPartialWarmupLockEpochsSpin);
    arForm->addRow("History relax factor", _pppArHistRelaxFactorSpin);
    arForm->addRow("Constraint sigma (m)", _pppArConstraintSigmaSpin);
    grid->addWidget(arGroup, 7, 1);

    QGroupBox* outGroup = new QGroupBox("PPP Outputs", content);
    QFormLayout* outForm = new QFormLayout(outGroup);
    _pppZtdPathEdit = new QLineEdit(outGroup);
    _pppZtdPathEdit->setPlaceholderText("Optional directory for ZTD exports");
    _pppZtdPathBrowseButton = new QPushButton("Browse...", outGroup);
    QHBoxLayout* ztdRow = new QHBoxLayout();
    ztdRow->addWidget(_pppZtdPathEdit, 1);
    ztdRow->addWidget(_pppZtdPathBrowseButton);
    outForm->addRow("ZTD output directory", ztdRow);
    _pppJsonEnableCheck = new QCheckBox("Write compact daily PPP JSON", outGroup);
    _pppJsonEnableCheck->setChecked(true);
    _pppJsonPathEdit = new QLineEdit(QStringLiteral("output/ppp_json"), outGroup);
    _pppJsonPathEdit->setPlaceholderText("One GPS-day JSON file per station");
    _pppJsonPathBrowseButton = new QPushButton("Browse...", outGroup);
    QHBoxLayout* jsonRow = new QHBoxLayout();
    jsonRow->addWidget(_pppJsonPathEdit, 1);
    jsonRow->addWidget(_pppJsonPathBrowseButton);
    outForm->addRow(QString(), _pppJsonEnableCheck);
    outForm->addRow("PPP JSON output directory", jsonRow);
    grid->addWidget(outGroup, 7, 0);

    grid->setRowStretch(9, 1);

    content->setLayout(grid);
    scroll->setWidget(content);
    root->addWidget(scroll);
}

void MainWindow::setupPppResultsUi() {
    QVBoxLayout* root = qobject_cast<QVBoxLayout*>(ui->ppp_results->layout());
    if (!root) {
        root = new QVBoxLayout(ui->ppp_results);
    }
    root->setContentsMargins(10, 10, 10, 10);
    root->setSpacing(8);

    _pppResultLabel = new QLabel("No PPP solution yet.", ui->ppp_results);
    _pppResultLabel->setWordWrap(true);
    root->addWidget(_pppResultLabel);

    QHBoxLayout* topBar = new QHBoxLayout();
    QLabel* stationLabel = new QLabel("Station:", ui->ppp_results);
    _pppStationCombo = new QComboBox(ui->ppp_results);
    _pppFixRateLabel = new QLabel("Fixed sats: -", ui->ppp_results);
    _pppFixRateLabel->setMinimumWidth(320);
    topBar->addWidget(stationLabel);
    topBar->addWidget(_pppStationCombo, 1);
    topBar->addWidget(_pppFixRateLabel);
    root->addLayout(topBar);

    QGroupBox* plotGroup = new QGroupBox("NEU Error (m)", ui->ppp_results);
    QVBoxLayout* plotLayout = new QVBoxLayout(plotGroup);
    _pppNeuScene = new QGraphicsScene(plotGroup);
    _pppNeuView = new QGraphicsView(_pppNeuScene, plotGroup);
    _pppNeuView->setRenderHint(QPainter::Antialiasing, true);
    _pppNeuView->setMinimumHeight(320);
    _pppNeuView->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    _pppNeuView->setVerticalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    plotLayout->addWidget(_pppNeuView);
    root->addWidget(plotGroup, 1);

    _pppResultTable = new QTableWidget(0, 11, ui->ppp_results);
    _pppResultTable->setHorizontalHeaderLabels(QStringList()
                                               << "GPS" << "Fixed/Total"
                                               << "dN (m)" << "dE (m)" << "dU (m)"
                                               << "sN" << "sE" << "sU" << "sZTD"
                                               << "Sat" << "HDOP");
    _pppResultTable->setEditTriggers(QAbstractItemView::NoEditTriggers);
    _pppResultTable->setSelectionBehavior(QAbstractItemView::SelectRows);
    _pppResultTable->setSelectionMode(QAbstractItemView::SingleSelection);
    _pppResultTable->setAlternatingRowColors(true);
    _pppResultTable->verticalHeader()->setVisible(false);
    configureSortableResizableTable(_pppResultTable);
    root->addWidget(_pppResultTable, 0);
}

void MainWindow::setupOthersUi() {
    QVBoxLayout* root = qobject_cast<QVBoxLayout*>(ui->others->layout());
    if (!root) {
        root = new QVBoxLayout(ui->others);
    }
    root->setContentsMargins(10, 10, 10, 10);
    root->setSpacing(8);

    QGroupBox* ztdPlotGroup = new QGroupBox("Real-time ZTD", ui->others);
    QVBoxLayout* ztdPlotLayout = new QVBoxLayout(ztdPlotGroup);
    QHBoxLayout* selectorRow = new QHBoxLayout();
    selectorRow->addWidget(new QLabel("Station:", ztdPlotGroup));
    _ztdStationCombo = new QComboBox(ztdPlotGroup);
    selectorRow->addWidget(_ztdStationCombo, 1);
    _ztdInfoLabel = new QLabel("No ZTD result yet.", ztdPlotGroup);
    selectorRow->addWidget(_ztdInfoLabel);
    ztdPlotLayout->addLayout(selectorRow);

    _ztdScene = new QGraphicsScene(ztdPlotGroup);
    _ztdView = new QGraphicsView(_ztdScene, ztdPlotGroup);
    _ztdView->setRenderHint(QPainter::Antialiasing, true);
    _ztdView->setMinimumHeight(300);
    _ztdView->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    _ztdView->setVerticalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    ztdPlotLayout->addWidget(_ztdView);
    root->addWidget(ztdPlotGroup, 1);

    QGroupBox* zwdPlotGroup = new QGroupBox("Zenith Wet Delay (ZWD)", ui->others);
    QVBoxLayout* zwdPlotLayout = new QVBoxLayout(zwdPlotGroup);
    _zwdScene = new QGraphicsScene(zwdPlotGroup);
    _zwdView = new QGraphicsView(_zwdScene, zwdPlotGroup);
    _zwdView->setRenderHint(QPainter::Antialiasing, true);
    _zwdView->setMinimumHeight(260);
    _zwdView->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    _zwdView->setVerticalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    zwdPlotLayout->addWidget(_zwdView);
    root->addWidget(zwdPlotGroup, 1);
}

void MainWindow::setupSignalBindings() {
    connect(_addNtripButton, &QPushButton::clicked, this, &MainWindow::addNtripFromCasterDialog);
    connect(_addTcpButton, &QPushButton::clicked, this, &MainWindow::addTcpDialog);
    connect(_addMqttButton, &QPushButton::clicked, this, &MainWindow::addMqttDialog);
    connect(_removeButton, &QPushButton::clicked, this, &MainWindow::removeSelectedStreams);
    connect(_globalStartButton, &QPushButton::clicked, this, &MainWindow::startAllStreams);
    connect(_globalStopButton, &QPushButton::clicked, this, &MainWindow::stopAllStreams);
    if (_postModeCombo) {
        connect(_postModeCombo, qOverload<int>(&QComboBox::currentIndexChanged), this, [this]() {
            const bool postMode = _postModeCombo->currentData().toString() == QLatin1String("post");
            if (postMode && (_pppRuntimeActive || std::any_of(_rows.cbegin(), _rows.cend(), [](const StreamRow& row) {
                return row.client && row.client->runtimeState().running;
            }))) {
                QMessageBox::warning(this, "Post-processing", "Stop all realtime streams before selecting post-processing mode.");
                QSignalBlocker blocker(_postModeCombo);
                _postModeCombo->setCurrentIndex(_postModeCombo->findData(QStringLiteral("realtime")));
                return;
            }
            if (_postInputPanel) _postInputPanel->setVisible(postMode);
            savePppSettingsToSettings();
        });
    }
    if (_postProductModeCombo) {
        connect(_postProductModeCombo, qOverload<int>(&QComboBox::currentIndexChanged), this, [this]() {
            applyPostProductModeUi();
            savePppSettingsToSettings();
        });
    }
    if (_postStartButton) connect(_postStartButton, &QPushButton::clicked, this, &MainWindow::startPostProcessing);
    if (_postCancelButton) connect(_postCancelButton, &QPushButton::clicked, this, &MainWindow::cancelPostProcessing);
    connect(_saveConfigButton, &QPushButton::clicked, this, &MainWindow::saveConfigNow);
    connect(_reloadConfigButton, &QPushButton::clicked, this, &MainWindow::reloadConfigNow);
    if (_pppStationCombo) {
        connect(_pppStationCombo, qOverload<int>(&QComboBox::currentIndexChanged), this, [this]() {
            refreshPppNeuPlot();
        });
    }
    if (_ztdStationCombo) {
        connect(_ztdStationCombo, qOverload<int>(&QComboBox::currentIndexChanged), this, [this]() {
            refreshZtdPlot();
            refreshZwdPlot();
        });
    }
    if (_statusPlotStationCombo) {
        connect(_statusPlotStationCombo, qOverload<int>(&QComboBox::currentIndexChanged), this, [this]() {
            const QString streamId = selectedStatusPlotStreamId();
            const int streamRow = rowForStreamId(streamId);
            if (_statusTable) {
                const int tableRow = tableRowForStreamId(_statusTable, streamId);
                if (tableRow >= 0) {
                    QSignalBlocker blocker(_statusTable);
                    _statusTable->selectRow(tableRow);
                }
            }
            updateStatusDetailsForRow(streamRow);
        });
    }
    if (_skyHistoryWindowCombo) {
        connect(_skyHistoryWindowCombo, qOverload<int>(&QComboBox::currentIndexChanged), this, [this]() {
            updateStatusDetailsForRow(rowForStreamId(selectedStatusPlotStreamId()));
        });
    }
    if (_pppStationTable) {
        connect(_pppStationTable, &QTableWidget::itemChanged, this, [this](QTableWidgetItem* item) {
            if (!item || item->column() != 0) {
                return;
            }
            const QString source = item->data(Qt::UserRole).toString();
            if (source.isEmpty()) {
                return;
            }
            const bool enabled = (item->checkState() == Qt::Checked);
            _pppStationEnabled[source] = enabled;
            savePppSettingsToSettings();
        });
    }
    if (_pppCrdBrowseButton && _pppCrdPathEdit) {
        connect(_pppCrdBrowseButton, &QPushButton::clicked, this, [this]() {
            const QString file = QFileDialog::getOpenFileName(this,
                                                              "Select coordinates file",
                                                              _pppCrdPathEdit->text().trimmed(),
                                                              "Coordinates (*.CRD *.crd);;All files (*.*)");
            if (!file.isEmpty()) {
                _pppCrdPathEdit->setText(file);
            }
        });
    }
    if (_pppAntexBrowseButton && _pppAntexPathEdit) {
        connect(_pppAntexBrowseButton, &QPushButton::clicked, this, [this]() {
            const QString file = QFileDialog::getOpenFileName(this,
                                                              "Select ANTEX file",
                                                              _pppAntexPathEdit->text().trimmed(),
                                                              "ANTEX (*.atx *.ATX);;All files (*.*)");
            if (!file.isEmpty()) {
                _pppAntexPathEdit->setText(file);
            }
        });
    }
    if (_pppBlqBrowseButton && _pppBlqPathEdit) {
        connect(_pppBlqBrowseButton, &QPushButton::clicked, this, [this]() {
            const QString file = QFileDialog::getOpenFileName(this,
                                                              "Select BLQ file (optional)",
                                                              _pppBlqPathEdit->text().trimmed(),
                                                              "BLQ (*.blq *.BLQ);;All files (*.*)");
            if (!file.isEmpty()) {
                _pppBlqPathEdit->setText(file);
            }
        });
    }
    if (_pppVtecBrowseButton && _pppVtecPathEdit) {
        connect(_pppVtecBrowseButton, &QPushButton::clicked, this, [this]() {
            const QString file = QFileDialog::getOpenFileName(this,
                                                              "Select VTEC correction file",
                                                              _pppVtecPathEdit->text().trimmed(),
                                                              "SSR/VTEC (*.ssr *.SSR *.txt);;All files (*.*)");
            if (!file.isEmpty()) {
                _pppVtecPathEdit->setText(file);
            }
        });
    }
    if (_pppArLogBrowseButton && _pppArLogPathEdit) {
        connect(_pppArLogBrowseButton, &QPushButton::clicked, this, [this]() {
            const QString file = QFileDialog::getSaveFileName(this,
                                                              "Select PPP-AR log file",
                                                              _pppArLogPathEdit->text().trimmed(),
                                                              "Log files (*.log *.txt);;All files (*.*)");
            if (!file.isEmpty()) {
                _pppArLogPathEdit->setText(file);
            }
        });
    }
    auto updateArMode = [this]() {
        if (_loadingPppSettingsUi) {
            return;
        }
        updatePppArAvailability();
        applyPppSettings(false);
    };
    if (_pppLcGpsCombo) {
        connect(_pppLcGpsCombo, qOverload<int>(&QComboBox::currentIndexChanged), this, updateArMode);
    }
    if (_pppLcGloCombo) {
        connect(_pppLcGloCombo, qOverload<int>(&QComboBox::currentIndexChanged), this, updateArMode);
    }
    if (_pppLcGalCombo) {
        connect(_pppLcGalCombo, qOverload<int>(&QComboBox::currentIndexChanged), this, updateArMode);
    }
    if (_pppLcBdsCombo) {
        connect(_pppLcBdsCombo, qOverload<int>(&QComboBox::currentIndexChanged), this, updateArMode);
    }
    if (_pppUseGpsCheck) {
        connect(_pppUseGpsCheck, &QCheckBox::toggled, this, updateArMode);
    }
    if (_pppUseGloCheck) {
        connect(_pppUseGloCheck, &QCheckBox::toggled, this, updateArMode);
    }
    if (_pppUseGalCheck) {
        connect(_pppUseGalCheck, &QCheckBox::toggled, this, updateArMode);
    }
    if (_pppUseBdsCheck) {
        connect(_pppUseBdsCheck, &QCheckBox::toggled, this, updateArMode);
    }
    if (_pppArEnableCheck) {
        connect(_pppArEnableCheck, &QCheckBox::toggled, this, [this]() {
            if (_loadingPppSettingsUi) {
                return;
            }
            updatePppArAvailability();
            applyPppSettings(false);
        });
    }
    if (_pppArAlgorithmCombo) {
        connect(_pppArAlgorithmCombo, qOverload<int>(&QComboBox::currentIndexChanged), this, updateArMode);
    }
    if (_pppArBiePersistConstraintsCheck) {
        connect(_pppArBiePersistConstraintsCheck, &QCheckBox::toggled, this, updateArMode);
    }
    if (_pppArUseGpsCheck) {
        connect(_pppArUseGpsCheck, &QCheckBox::toggled, this, updateArMode);
    }
    if (_pppArUseGloCheck) {
        connect(_pppArUseGloCheck, &QCheckBox::toggled, this, updateArMode);
    }
    if (_pppArUseGalCheck) {
        connect(_pppArUseGalCheck, &QCheckBox::toggled, this, updateArMode);
    }
    if (_pppArUseBdsCheck) {
        connect(_pppArUseBdsCheck, &QCheckBox::toggled, this, updateArMode);
    }
    if (_pppArUseHistoryAidCheck) {
        connect(_pppArUseHistoryAidCheck, &QCheckBox::toggled, this, [this]() {
            if (_loadingPppSettingsUi) {
                return;
            }
            updatePppArAvailability();
            applyPppSettings(false);
        });
    }
    if (_pppZtdPathBrowseButton && _pppZtdPathEdit) {
        connect(_pppZtdPathBrowseButton, &QPushButton::clicked, this, [this]() {
            const QString dir = QFileDialog::getExistingDirectory(this,
                                                                  "Select ZTD output directory",
                                                                  _pppZtdPathEdit->text().trimmed());
            if (!dir.isEmpty()) {
                _pppZtdPathEdit->setText(dir);
            }
        });
    }

    auto prepAutoApply = [this]() {
        applyDataPreparationSelection(false);
    };
    if (_prepEnableCheck) {
        connect(_prepEnableCheck, &QCheckBox::toggled, this, prepAutoApply);
    }
    if (_prepBroadcastCombo) {
        connect(_prepBroadcastCombo, qOverload<int>(&QComboBox::currentIndexChanged), this, prepAutoApply);
    }
    if (_prepOrbitCombo) {
        connect(_prepOrbitCombo, qOverload<int>(&QComboBox::currentIndexChanged), this, prepAutoApply);
    }
    if (_prepClockCombo) {
        connect(_prepClockCombo, qOverload<int>(&QComboBox::currentIndexChanged), this, prepAutoApply);
    }
    if (_prepCodeBiasCombo) {
        connect(_prepCodeBiasCombo, qOverload<int>(&QComboBox::currentIndexChanged), this, prepAutoApply);
    }
    if (_prepPhaseBiasCombo) {
        connect(_prepPhaseBiasCombo, qOverload<int>(&QComboBox::currentIndexChanged), this, prepAutoApply);
    }
    if (_prepObsCombo) {
        connect(_prepObsCombo, qOverload<int>(&QComboBox::currentIndexChanged), this, prepAutoApply);
    }
    if (_obsSampleCombo) {
        connect(_obsSampleCombo, qOverload<int>(&QComboBox::currentIndexChanged), this, prepAutoApply);
    }
    if (_sp3EnableCheck) {
        connect(_sp3EnableCheck, &QCheckBox::toggled, this, prepAutoApply);
    }
    if (_sp3PathEdit) {
        connect(_sp3PathEdit, &QLineEdit::editingFinished, this, prepAutoApply);
    }
    if (_sp3SampleSpin) {
        connect(_sp3SampleSpin, qOverload<int>(&QSpinBox::valueChanged), this, prepAutoApply);
    }
    if (_rnxEnableCheck) {
        connect(_rnxEnableCheck, &QCheckBox::toggled, this, prepAutoApply);
    }
    if (_rnxPathEdit) {
        connect(_rnxPathEdit, &QLineEdit::editingFinished, this, prepAutoApply);
    }
    if (_telemetryEnableCheck) {
        connect(_telemetryEnableCheck, &QCheckBox::toggled, this, prepAutoApply);
    }
    if (_telemetryPathEdit) {
        connect(_telemetryPathEdit, &QLineEdit::editingFinished, this, prepAutoApply);
    }
    if (_prepLogPathEdit) {
        connect(_prepLogPathEdit, &QLineEdit::editingFinished, this, prepAutoApply);
    }
    if (_prepLogBrowseButton && _prepLogPathEdit) {
        connect(_prepLogBrowseButton, &QPushButton::clicked, this, [this]() {
            const QString file = QFileDialog::getSaveFileName(this,
                                                              "Select log output file",
                                                              resolvedPrepLogFilePath(),
                                                              "Log files (*.log *.txt);;All files (*.*)");
            if (!file.isEmpty()) {
                _prepLogPathEdit->setText(file);
                applyDataPreparationSelection(false);
            }
        });
    }
    if (_pppJsonPathBrowseButton && _pppJsonPathEdit) {
        connect(_pppJsonPathBrowseButton, &QPushButton::clicked, this, [this]() {
            const QString dir = QFileDialog::getExistingDirectory(this,
                                                                  "Select PPP JSON output directory",
                                                                  _pppJsonPathEdit->text().trimmed());
            if (!dir.isEmpty()) {
                _pppJsonPathEdit->setText(dir);
                applyPppSettings(false);
            }
        });
    }
    if (_pppJsonEnableCheck) {
        connect(_pppJsonEnableCheck, &QCheckBox::toggled, this, [this]() {
            if (!_loadingPppSettingsUi) {
                applyPppSettings(false);
            }
        });
    }
    if (_pppJsonPathEdit) {
        connect(_pppJsonPathEdit, &QLineEdit::editingFinished, this, [this]() {
            if (!_loadingPppSettingsUi) {
                applyPppSettings(false);
            }
        });
    }
}

void MainWindow::addNtripFromCasterDialog() {
    AddNtripCasterDialog dlg(_settingsStore.loadCasters(), this);
    if (dlg.exec() != QDialog::Accepted) {
        QString error;
        if (!_settingsStore.saveCasters(dlg.castersJson(), &error)) {
            appendLog("SYSTEM", QString("Caster preset save failed: %1").arg(error));
        }
        return;
    }
    QString casterError;
    if (!_settingsStore.saveCasters(dlg.castersJson(), &casterError)) {
        appendLog("SYSTEM", QString("Caster preset save failed: %1").arg(casterError));
    }

    const QList<NtripSelectionRow> selected = dlg.selectedRows();
    for (const NtripSelectionRow& sel : selected) {
        StreamRow row;
        row.id = QString::number(_idSeed++);
        row.type = "NTRIP";
        row.sourceDisplay = QString("%1:%2/%3").arg(sel.cfg.host).arg(sel.cfg.port).arg(sel.cfg.mountPoint);
        row.decoder = sel.decoder.isEmpty() ? "RTCM" : sel.decoder;
        row.lat = sel.lat;
        row.lon = sel.lon;
        row.nmea = sel.nmea;
        row.stateText = "Stopped";
        row.bytes = 0;
        row.config = QJsonObject{
            {"host", sel.cfg.host},
            {"port", sel.cfg.port},
            {"mountPoint", sel.cfg.mountPoint},
            {"user", sel.cfg.user},
            {"password", sel.cfg.password},
            {"nmea", sel.cfg.nmea},
            {"ntripVersion", sel.cfg.ntripVersion},
            {"reconnectMs", sel.cfg.reconnectMs}
        };
        _rows.push_back(row);
    }
    rebuildStreamsTable();
    QString streamSaveError;
    if (!saveStreamsToSettings(&streamSaveError)) {
        const QString message = QStringLiteral("Configuration save failed: %1").arg(streamSaveError);
        appendLog("SYSTEM", message);
        QMessageBox::warning(this, QStringLiteral("Save Config"), message);
        return;
    }
}

void MainWindow::addTcpDialog() {
    AddTcpDialog dlg(this);
    if (dlg.exec() != QDialog::Accepted) {
        return;
    }
    const QJsonObject cfg = dlg.toConfig();
    if (cfg.value("host").toString().isEmpty() || cfg.value("port").toInt() <= 0) {
        QMessageBox::warning(this, "Invalid", "TCP host/port cannot be empty.");
        return;
    }

    StreamRow row;
    row.id = QString::number(_idSeed++);
    row.type = "TCP";
    row.sourceDisplay = QString("%1:%2").arg(cfg.value("host").toString()).arg(cfg.value("port").toInt());
    row.decoder = "RAW TCP";
    row.lat = "-";
    row.lon = "-";
    row.nmea = "no";
    row.stateText = "Stopped";
    row.config = cfg;
    _rows.push_back(row);

    rebuildStreamsTable();
    saveStreamsToSettings();
}

void MainWindow::addMqttDialog() {
    AddMqttDialog dlg(this);
    if (dlg.exec() != QDialog::Accepted) {
        return;
    }
    const QJsonObject cfg = dlg.toConfig();
    if (cfg.value("host").toString().isEmpty() || cfg.value("topic").toString().isEmpty()) {
        QMessageBox::warning(this, "Invalid", "MQTT host/topic cannot be empty.");
        return;
    }

    StreamRow row;
    row.id = QString::number(_idSeed++);
    row.type = "MQTT";
    row.sourceDisplay = QString("%1:%2/%3")
                            .arg(cfg.value("host").toString())
                            .arg(cfg.value("port").toInt())
                            .arg(cfg.value("topic").toString());
    row.decoder = "MQTT";
    row.lat = "-";
    row.lon = "-";
    row.nmea = "no";
    row.stateText = "Stopped";
    row.config = cfg;
    _rows.push_back(row);

    rebuildStreamsTable();
    saveStreamsToSettings();
}

void MainWindow::removeSelectedStreams() {
    const int row = selectedRow();
    if (row < 0 || row >= _rows.size()) {
        return;
    }
    stopAndDeleteClient(_rows[row]);
    _rows.removeAt(row);
    rebuildStreamsTable();
    saveStreamsToSettings();
}

void MainWindow::startAllStreams() {
    if (_postModeCombo && _postModeCombo->currentData().toString() == QLatin1String("post")) {
        appendLog("SYSTEM", "Realtime streams are disabled while Post-processing files mode is selected.");
        return;
    }
    if (_postRunning) {
        appendLog("SYSTEM", "Realtime streams cannot start while a post-processing task is running.");
        return;
    }
    startPppRuntime();
    for (int row = 0; row < _rows.size(); ++row) {
        startRow(row);
    }
    rebuildStatusTable();
}

void MainWindow::stopAllStreams() {
    _pppRuntimeActive = false;
    requestPppWorkerStopNow();
    for (int row = 0; row < _rows.size(); ++row) {
        stopRow(row);
    }
    resetRuntimeStateForFreshStart();
    finalizePppJsonOutput();
    rebuildStatusTable();
    appendLog("SYSTEM", "Runtime state cleared after Stop All.");
}

void MainWindow::updatePostProcessingUi(bool running) {
    _postRunning = running;
    if (!ui || !ui->ppp_settings) return;
    const QList<QWidget*> widgets = ui->ppp_settings->findChildren<QWidget*>();
    for (QWidget* widget : widgets) widget->setEnabled(!running);
    if (running) {
        if (_postInputPanel) _postInputPanel->setEnabled(true);
        if (_postCancelButton) _postCancelButton->setEnabled(true);
        if (_postProgress) _postProgress->setEnabled(true);
        if (_postStatusLabel) _postStatusLabel->setEnabled(true);
        if (_postStartButton) _postStartButton->setEnabled(false);
        if (_globalStartButton) _globalStartButton->setEnabled(false);
    } else {
        if (_globalStartButton) _globalStartButton->setEnabled(true);
        applyPostProductModeUi();
        updatePppArAvailability();
    }
}

void MainWindow::applyPostProductModeUi() {
    const bool precise = _postProductModeCombo &&
        _postProductModeCombo->currentData().toString() == QLatin1String("precise");
    if (_postSsrProductPanel) {
        _postSsrProductPanel->setVisible(!precise);
        _postSsrProductPanel->setEnabled(!_postRunning && !precise);
    }
    if (_postPreciseProductPanel) {
        _postPreciseProductPanel->setVisible(precise);
        _postPreciseProductPanel->setEnabled(!_postRunning && precise);
    }
}

void MainWindow::startPostProcessing() {
    if (_postRunning || !_postModeCombo || _postModeCombo->currentData().toString() != QLatin1String("post")) {
        return;
    }
    if (_postThread) {
        if (_postThread->isRunning()) {
            if (_postStatusLabel) _postStatusLabel->setText(QStringLiteral("Previous task is finishing"));
            return;
        }
        _postThread = nullptr;
        _postService = nullptr;
    }
    const bool realtimeActive = _pppRuntimeActive || std::any_of(_rows.cbegin(), _rows.cend(), [](const StreamRow& row) {
        return row.client && row.client->runtimeState().running;
    });
    if (realtimeActive) {
        QMessageBox::warning(this, "Post-processing", "Stop all realtime streams before starting a post-processing task.");
        return;
    }
    const QString obsPath = _postObsPathEdit ? _postObsPathEdit->text().trimmed() : QString();
    const QString productMode = _postProductModeCombo ? _postProductModeCombo->currentData().toString() : QStringLiteral("broadcast");
    const QString navPath = _postNavPathEdit ? _postNavPathEdit->text().trimmed() : QString();
    const QString sp3Path = _postSp3PathEdit ? _postSp3PathEdit->text().trimmed() : QString();
    const QString clkPath = _postClkPathEdit ? _postClkPathEdit->text().trimmed() : QString();
    const QStringList configuredSsrPaths = _postSsrPathsEdit
        ? _postSsrPathsEdit->text().split(QLatin1Char(';'), QString::SkipEmptyParts) : QStringList();
    const QStringList ssrPaths = productMode == QLatin1String("broadcast")
        ? configuredSsrPaths : QStringList();
    if (obsPath.isEmpty() || !QFileInfo::exists(obsPath) ||
        navPath.isEmpty() || !QFileInfo::exists(navPath) ||
        (productMode == QLatin1String("precise") && (sp3Path.isEmpty() || clkPath.isEmpty() ||
                                                       !QFileInfo::exists(sp3Path) || !QFileInfo::exists(clkPath)))) {
        QMessageBox::warning(this, "Post-processing", "Select an existing RINEX OBS, Broadcast NAV, and the required SSR or SP3+CLK products.");
        return;
    }
    if (productMode == QLatin1String("broadcast") && ssrPaths.isEmpty()) {
        QMessageBox::warning(this, "Post-processing", "Broadcast post-processing requires at least one SSR file.");
        return;
    }
    for (const QString& path : ssrPaths) {
        if (!QFileInfo::exists(path.trimmed())) {
            QMessageBox::warning(this, "Post-processing", QString("SSR file does not exist: %1").arg(path));
            return;
        }
    }
    applyPppSettings(false);
    QString jobName = _postJobNameEdit ? _postJobNameEdit->text().trimmed() : QString();
    if (jobName.isEmpty()) jobName = QFileInfo(obsPath).baseName();
    jobName.replace(QRegularExpression(QStringLiteral("[^A-Za-z0-9_.-]+")), QStringLiteral("_"));
    if (jobName.isEmpty()) jobName = QStringLiteral("post");
    const QString runName = QStringLiteral("%1-%2").arg(jobName,
        QDateTime::currentDateTimeUtc().toString(QStringLiteral("yyyyMMddTHHmmssZ")));
    const QString outputRoot = QDir(_pppJsonOutputDirectory).filePath(QStringLiteral("post/%1").arg(runName));

    PostPppJobConfig config;
    config.jobName = jobName;
    config.observationPath = obsPath;
    config.navigationPath = navPath;
    config.sp3Path = productMode == QLatin1String("precise") ? sp3Path : QString();
    config.clockPath = productMode == QLatin1String("precise") ? clkPath : QString();
    config.productMode = productMode;
    config.ssrPaths = ssrPaths;
    config.sampleSec = _postSampleCombo ? _postSampleCombo->currentData().toInt() : 5;
    config.jsonEnabled = _pppJsonEnabled;
    config.jsonReorderBufferMs = _pppJsonReorderBufferMs;
    config.jsonFlushIntervalMs = _pppJsonFlushIntervalMs;
    config.outputDirectory = outputRoot;
    config.ztdOutputDirectory = QDir(_ztdOutputDirectory).filePath(QStringLiteral("post/%1").arg(runName));
    config.options = _pppOptions;
    const QMap<QString, PppCrdEntry> entries = readPppCrdFile(_pppOptions.crdFile);
    for (const QString& candidate : pppCrdStationCandidates(QStringLiteral("POST"), jobName, QFileInfo(obsPath).baseName())) {
        const auto it = entries.constFind(candidate);
        if (it == entries.constEnd()) continue;
        config.station.valid = true;
        config.station.station = it->station;
        config.station.roverAntenna = it->antenna;
        config.station.receiver = it->receiver;
        for (int i = 0; i < 3; ++i) { config.station.aprXyz[i] = it->xyz[i]; config.station.neuEcc[i] = it->neu[i]; }
        break;
    }
    _postOutputDirectory = outputRoot;
    _postThread = new QThread(this);
    _postService = new PostPppService();
    QThread* workerThread = _postThread;
    PostPppService* workerService = _postService;
    _postService->moveToThread(workerThread);
    connect(workerThread, &QThread::finished, workerService, &QObject::deleteLater);
    connect(_postThread, &QThread::finished, _postThread, &QObject::deleteLater);
    connect(workerThread, &QThread::finished, this, [this, workerThread, workerService]() {
        if (_postThread == workerThread) _postThread = nullptr;
        if (_postService == workerService) _postService = nullptr;
    });
    connect(_postService, &PostPppService::started, this, [this](const QString& directory, int total) {
        if (_postProgress) { _postProgress->setRange(0, qMax(1, total)); _postProgress->setValue(0); }
        if (_postStatusLabel) _postStatusLabel->setText(QStringLiteral("Running: %1").arg(directory));
    });
    connect(_postService, &PostPppService::progress, this, [this](int done, int total, const QString& epoch) {
        if (_postProgress) { _postProgress->setRange(0, qMax(1, total)); _postProgress->setValue(done); }
        if (_postStatusLabel) _postStatusLabel->setText(QStringLiteral("%1 / %2, %3").arg(done).arg(total).arg(epoch));
    });
    connect(_postService, &PostPppService::solutionReady, this, [this](const QString&, const PppSolution& solution) {
        updatePppResults(QStringLiteral("POST"), solution);
    });
    connect(_postService, &PostPppService::logMessage, this, [this](const QString& message) { appendLog("POST", message); });
    auto finish = [this](bool cancelled, int processed, int accepted, const QString& summary) {
        appendLog("POST", QString("Post-processing %1: sampled=%2 accepted=%3 summary=%4")
                  .arg(cancelled ? "cancelled" : "finished").arg(processed).arg(accepted).arg(summary));
        if (_postStatusLabel) _postStatusLabel->setText(cancelled ? QStringLiteral("Cancelled") : QStringLiteral("Finished"));
        QThread* thread = _postThread;
        updatePostProcessingUi(false);
        if (thread) thread->quit();
    };
    connect(_postService, &PostPppService::finished, this, finish);
    connect(_postService, &PostPppService::failed, this, [this](const QString& reason) {
        appendLog("POST", QString("Post-processing failed: %1").arg(reason));
        if (_postStatusLabel) _postStatusLabel->setText(QStringLiteral("Failed: %1").arg(reason));
        QThread* thread = _postThread;
        updatePostProcessingUi(false);
        if (thread) thread->quit();
    });
    _postThread->start();
    updatePostProcessingUi(true);
    savePppSettingsToSettings();
    QMetaObject::invokeMethod(_postService, [service = _postService, config]() { service->run(config); }, Qt::QueuedConnection);
}

void MainWindow::cancelPostProcessing() {
    if (_postService) {
        _postService->cancel();
        if (_postStatusLabel) _postStatusLabel->setText(QStringLiteral("Cancelling..."));
    }
}

void MainWindow::saveConfigNow() {
    const QString file = QFileDialog::getSaveFileName(this,
                                                      "Save Config",
                                                      settingsFilePath(),
                                                      "Config (*.ini);;All files (*.*)");
    if (file.isEmpty()) {
        return;
    }
    const QString previousSettingsPath = _settingsStore.filePath();
    _settingsStore.setFilePath(file);
    applyDataPreparationSelection(false);
    applyPppSettings(false);
    QString streamSaveError;
    if (!saveStreamsToSettings(&streamSaveError)) {
        const QString message = QStringLiteral("Configuration save failed: %1").arg(streamSaveError);
        qWarning().noquote() << message;
        appendLog("SYSTEM", message);
        QMessageBox::warning(this, QStringLiteral("Save Config"), message);
        _settingsStore.setFilePath(previousSettingsPath);
        return;
    }
    QString outputSaveError;
    if (!saveDataPreparationToSettings(&outputSaveError) ||
        !savePppSettingsToSettings(&outputSaveError)) {
        const QString message = QStringLiteral("Configuration save failed: %1").arg(outputSaveError);
        qWarning().noquote() << message;
        appendLog("SYSTEM", message);
        QMessageBox::warning(this, QStringLiteral("Save Config"), message);
        _settingsStore.setFilePath(previousSettingsPath);
        return;
    }
    QSettings settings(settingsFilePath(), QSettings::IniFormat);
    settings.sync();
    if (settings.status() != QSettings::NoError) {
        const QString message = QStringLiteral("Configuration save failed: %1 (QSettings status=%2)")
                                    .arg(QFileInfo(settingsFilePath()).absoluteFilePath())
                                    .arg(static_cast<int>(settings.status()));
        qWarning().noquote() << message;
        appendLog("SYSTEM", message);
        QMessageBox::warning(this, QStringLiteral("Save Config"), message);
        _settingsStore.setFilePath(previousSettingsPath);
        return;
    }
    const QFileInfo savedInfo(settingsFilePath());
    if (!savedInfo.isFile() || savedInfo.size() <= 0) {
        const QString message = QStringLiteral("Configuration save did not produce a readable file: %1")
                                    .arg(savedInfo.absoluteFilePath());
        qWarning().noquote() << message;
        appendLog("SYSTEM", message);
        QMessageBox::warning(this, QStringLiteral("Save Config"), message);
        _settingsStore.setFilePath(previousSettingsPath);
        return;
    }
    appendLog("SYSTEM", QString("Configuration saved and applied: %1 (%2 bytes)")
                             .arg(savedInfo.absoluteFilePath())
                             .arg(savedInfo.size()));
}

void MainWindow::reloadConfigNow() {
    const QString file = QFileDialog::getOpenFileName(this,
                                                      "Open Config",
                                                      settingsFilePath(),
                                                      "Config (*.ini);;All files (*.*)");
    if (file.isEmpty()) {
        return;
    }
    const QFileInfo configInfo(file);
    if (!configInfo.isFile() || !configInfo.isReadable()) {
        const QString message = QStringLiteral("Configuration reload failed: file is missing or unreadable: %1")
                                    .arg(configInfo.absoluteFilePath());
        appendLog("SYSTEM", message);
        QMessageBox::warning(this, QStringLiteral("Reload Config"), message);
        return;
    }

    QString validationError;
    SettingsStore candidateStore(file);
    if (!candidateStore.validate(&validationError)) {
        const QString message = QStringLiteral("Configuration reload failed: %1").arg(validationError);
        appendLog("SYSTEM", message);
        QMessageBox::warning(this, QStringLiteral("Reload Config"), message);
        return;
    }
    QJsonArray streamArray;
    if (!candidateStore.loadStreams(streamArray, &validationError)) {
        const QString message = QStringLiteral("Configuration reload failed: %1").arg(validationError);
        appendLog("SYSTEM", message);
        QMessageBox::warning(this, QStringLiteral("Reload Config"), message);
        return;
    }
    QVector<StreamRow> candidateRows;
    if (!parseStreamArray(streamArray, candidateRows, &validationError)) {
        const QString message = QStringLiteral("Configuration reload failed: %1").arg(validationError);
        appendLog("SYSTEM", message);
        QMessageBox::warning(this, QStringLiteral("Reload Config"), message);
        return;
    }

    stopAllStreams();
    for (StreamRow& row : _rows) {
        stopAndDeleteClient(row);
    }
    _settingsStore.setFilePath(file);
    _rows = candidateRows;
    for (const StreamRow& row : _rows) {
        bool idOk = false;
        const int id = row.id.toInt(&idOk);
        if (idOk) _idSeed = qMax(_idSeed, id + 1);
    }
    clearRtcmProcessingState();
    _preparedStates.clear();
    rebuildStreamsTable();
    loadDataPreparationFromSettings();
    applyDataPreparationSelection(false);
    loadPppSettingsFromSettings();
    applyPppSettings(false);
    rebuildStatusTable();
    appendLog("SYSTEM", QString("Configuration reloaded: %1 (%2 streams)")
                             .arg(configInfo.absoluteFilePath()).arg(_rows.size()));
}

void MainWindow::appendLog(const QString& source, const QString& message) {
    if (_closing) {
        return;
    }
    if (QThread::currentThread() != thread()) {
        QMetaObject::invokeMethod(this, [this, source, message]() {
            appendLog(source, message);
        }, Qt::QueuedConnection);
        return;
    }
    if (!_logTextEdit) {
        return;
    }
    const QString text = QString("[%1] [%2] %3")
                             .arg(QDateTime::currentDateTimeUtc().toString("HH:mm:ss"))
                             .arg(source)
                             .arg(message);
    _pendingLogLines.push_back(text);
    if (_logFlushTimer && !_logFlushTimer->isActive()) {
        _logFlushTimer->start();
    }
}

void MainWindow::flushPendingLogs() {
    if (!_logTextEdit || _pendingLogLines.isEmpty()) {
        return;
    }
    const QString text = _pendingLogLines.join(QLatin1Char('\n'));
    _pendingLogLines.clear();
    _logTextEdit->appendPlainText(text);
    QTextCursor cursor = _logTextEdit->textCursor();
    cursor.movePosition(QTextCursor::End);
    _logTextEdit->setTextCursor(cursor);

    postOutputLog(text);
}

QString MainWindow::resolvedPrepLogFilePath() const {
    QString path = _prepLogPathEdit ? _prepLogPathEdit->text().trimmed()
                                    : _prepConfig.logOutputPath.trimmed();
    if (path.isEmpty()) {
        return QString();
    }

    path = QDir::fromNativeSeparators(path);
    if (path.endsWith(QLatin1Char('/'))) {
        return QDir(path).filePath(QStringLiteral("rtppp_log.txt"));
    }

    QFileInfo info(path);
    const bool looksLikeFile = !info.fileName().isEmpty() && info.fileName().contains(QLatin1Char('.'));
    if (info.exists() && info.isDir()) {
        return QDir(path).filePath(QStringLiteral("rtppp_log.txt"));
    }
    if (!looksLikeFile) {
        QDir dir(path);
        if (dir.isRelative()) {
            dir.setPath(QDir(QCoreApplication::applicationDirPath()).filePath(path));
        }
        return dir.filePath(QStringLiteral("rtppp_log.txt"));
    }
    if (info.isRelative()) {
        return QDir(QCoreApplication::applicationDirPath()).filePath(path);
    }
    return info.absoluteFilePath();
}

void MainWindow::appendDataPreview(const QString& source, qint64 bytes, const QByteArray& payload) {
    if (!_logRawFrames) {
        return;
    }
    appendLog(source, QString("%1 bytes | %2").arg(bytes).arg(dataPreviewToHex(payload)));
}

void MainWindow::decodeRtcmPayload(const QString& source, const QByteArray& payload) {
    RtcmProcessingService* service = _rtcmProcessingService;
    if (!service || source.isEmpty() || payload.isEmpty()) {
        return;
    }
    const std::array<double, 3> receiverEcef = receiverEcefForSource(source);
    service->submitPayload(source, payload, receiverEcef);
}

void MainWindow::drainRtcmProcessingResults() {
    RtcmProcessingService* service = _rtcmProcessingService;
    if (!service) {
        return;
    }
    QVector<RtcmProcessingService::Result> results;
    service->takePending(results);
    if (_closing) {
        if (_telemetryEnabled) {
            for (const RtcmProcessingService::Result& result : results) {
                for (const RtcmProcessingService::SsrTelemetryEvent& event : result.ssrTelemetryEvents)
                    postTelemetrySsr(event.sourceId, event.receivedUtc, event.epochUtc,
                                     event.message, event.satellite, event.adopted, event.reason);
                for (const RtcmDecodedMessage& message : result.batch.messages) {
                    if (message.hasVtec && _outputWorker) {
                        const QDateTime epoch = GnssUtils::gpsTimeFromSsr(
                            QLatin1Char('G'), message.vtec.epochTime, QDateTime::currentDateTimeUtc()).utc;
                        _outputWorker->enqueueTelemetryVtec(result.sourceId, epoch, message.vtec);
                    }
                }
            }
        }
        return;
    }

    for (const RtcmProcessingService::Result& result : results) {
        const QString& source = result.sourceId;
        const RtcmDecodeBatch& batch = result.batch;
        int msmStatusIndex = 0;

        if (result.preparedStatesChanged) {
            _preparedStates = result.preparedStates;
            refreshDataPreparationTable();
        }
        for (const RtcmProcessingService::Sp3Record& record : result.sp3Records) {
            postOutputSp3(record.gpsWeek, record.gpsSow, record.prn,
                          record.position, record.clockSec,
                          record.velocity, record.clockRate);
        }
        if (!result.sp3Records.isEmpty() && shouldLogNow(_lastSp3Log, 2)) {
            const RtcmProcessingService::Sp3Record& first = result.sp3Records.first();
            appendLog("SP3", QString("Queued epoch: week=%1 sow=%2 satellites=%3")
                                  .arg(first.gpsWeek)
                                  .arg(first.gpsSow, 0, 'f', 1)
                                  .arg(result.sp3Records.size()));
        }
        if (_telemetryEnabled) {
            for (const RtcmProcessingService::SsrTelemetryEvent& event : result.ssrTelemetryEvents) {
                postTelemetrySsr(event.sourceId, event.receivedUtc, event.epochUtc,
                                 event.message, event.satellite,
                                 event.adopted, event.reason);
            }
        }

        if (!batch.messages.isEmpty()) {
            for (const RtcmDecodedMessage& message : batch.messages) {
                if (_telemetryEnabled && _outputWorker && message.hasVtec) {
                    const QDateTime receivedUtc = QDateTime::currentDateTimeUtc();
                    const QDateTime epochUtc = GnssUtils::gpsTimeFromSsr(
                        QLatin1Char('G'), message.vtec.epochTime, receivedUtc).utc;
                    _outputWorker->enqueueTelemetryVtec(source, epochUtc, message.vtec);
                }
                if ((isStandardPhaseBiasMt(message.type) || message.type == 4076) &&
                    message.category == QLatin1String("SSR") && !message.hasSsr) {
                    appendLog(source,
                              QString("SSR decode warning: type=%1 not ingested as SSR (%2)")
                                  .arg(message.type)
                                  .arg(message.summary));
                }
                const RtcmProcessingService::MsmStatusUpdate* statusUpdate = nullptr;
                if (message.hasMsm && msmStatusIndex < result.msmStatusUpdates.size()) {
                    statusUpdate = &result.msmStatusUpdates[msmStatusIndex++];
                }
                ingestDecodedRtcmMessage(source, message, statusUpdate);
            }

            if (_logRtcmDetails) {
                appendLog(source,
                          QString("RTCM decoded %1 frame(s): %2")
                              .arg(batch.messages.size())
                              .arg(summarizeDecodedTypes(batch.messages)));
                const int detailLimit = qMin(4, batch.messages.size());
                for (int i = 0; i < detailLimit; ++i) {
                    const RtcmDecodedMessage& message = batch.messages[i];
                    appendLog(source,
                              QString("RTCM MT%1 [%2] %3")
                                  .arg(message.type)
                                  .arg(message.category)
                                  .arg(message.summary));
                }
                if (batch.messages.size() > detailLimit) {
                    appendLog(source, QString("RTCM ... %1 more frame(s) in this chunk")
                                          .arg(batch.messages.size() - detailLimit));
                }
            } else if (shouldLogNow(_lastRtcmSummaryLog, 30)) {
                appendLog(source,
                          QString("RTCM input active: decoded %1 frame(s), types=%2")
                              .arg(batch.messages.size())
                              .arg(summarizeDecodedTypes(batch.messages)));
            }
        }

        if (batch.crcErrors > 0) {
            appendLog(source, QString("RTCM CRC error(s): %1 (resync applied)").arg(batch.crcErrors));
        }
        if (batch.malformedFrames > 0) {
            appendLog(source, QString("RTCM malformed frame(s): %1").arg(batch.malformedFrames));
        }
    }
}

void MainWindow::ingestDecodedRtcmMessage(
    const QString& source,
    const RtcmDecodedMessage& message,
    const RtcmProcessingService::MsmStatusUpdate* msmStatusUpdate) {
    if (message.ignored) {
        return;
    }
    if (message.hasBroadcastEphemeris) {
        if (!isSupportedSolutionSystem(message.broadcastEphemeris.sat.system)) {
            return;
        }
        ingestBroadcastEphemeris(source, message.broadcastEphemeris);
    }
    if (message.hasSsr) {
        if (!isSupportedSolutionSystem(message.ssr.system)) {
            return;
        }
        ingestSsrMessage(source, message.ssr);
    }
    if (message.hasVtec && _pppRuntimeService) {
        PppVtecModel vtec;
        vtec.stationId = source;
        vtec.updateIntervalIndex = message.vtec.updateIntervalIndex;
        const GnssUtils::GpsTime referenceTime = GnssUtils::currentGpsTime();
        const GnssUtils::GpsTime vtecTime = GnssUtils::normalizeGpsTime(
            referenceTime.week, static_cast<double>(message.vtec.epochTime));
        vtec.gpsWeek = vtecTime.week;
        vtec.gpsSow = vtecTime.sow;
        vtec.utc = vtecTime.utc;
        vtec.layers.reserve(message.vtec.layers.size());
        for (const RtcmVtecLayer& inLayer : message.vtec.layers) {
            PppVtecLayer outLayer;
            outLayer.heightMeters = inLayer.heightMeters;
            outLayer.cosine = inLayer.cosine;
            outLayer.sine = inLayer.sine;
            vtec.layers.push_back(outLayer);
        }
        _pppRuntimeService->updateVtecModel(vtec);
    }
    if (message.hasMsm) {
        if (!isSupportedSolutionSystem(message.msm.system)) {
            return;
        }
        if (!msmStatusUpdate) {
            return;
        }
        if (_telemetryEnabled) {
            const QDateTime epochUtc = GnssUtils::dateTimeFromGpsWeek(
                msmStatusUpdate->gpsWeek, msmStatusUpdate->gpsSow).toUTC();
            postTelemetryMsm(source, msmStatusUpdate->summary.receivedUtc, epochUtc,
                             msmStatusUpdate->mergedMessage,
                             msmStatusUpdate->skyBySatellite);
            postTelemetryReception(source, msmStatusUpdate->summary.receivedUtc,
                                   "MSM", msmStatusUpdate->mergedMessage.messageType,
                                   msmStatusUpdate->mergedMessage.system, "received", QString(),
                                   QString("cells=%1").arg(msmStatusUpdate->mergedMessage.cells.size()));
        }
        const bool firstMsmForSource = msmStatusUpdate->firstForSource;
        const RtcmMsmMessage& cachedMsm = msmStatusUpdate->mergedMessage;
        const MsmSystemSummary& summary = msmStatusUpdate->summary;
        _lastMsmMessages[source][cachedMsm.system] = cachedMsm;
        _msmSummaries[source][summary.system] = summary;
        QVector<SkyPlotPoint> livePoints;
        livePoints.reserve(msmStatusUpdate->skyBySatellite.size());
        for (auto it = msmStatusUpdate->skyBySatellite.constBegin();
             it != msmStatusUpdate->skyBySatellite.constEnd(); ++it) {
            bool numberOk = false;
            const int number = it.key().mid(1).toInt(&numberOk);
            if (!numberOk || it.key().isEmpty()) {
                continue;
            }
            SkyPlotPoint point;
            point.utc = summary.receivedUtc;
            point.system = it.key().at(0);
            point.prn = number;
            point.az = it.value().first;
            point.el = it.value().second;
            livePoints.push_back(point);
        }
        _liveSkyPoints[source][cachedMsm.system] = livePoints;
        _skyReceiverPositionValid[source] = msmStatusUpdate->receiverPositionValid;
        if (firstMsmForSource) {
            refreshStatusPlotStationSelector();
        }

        QVector<SatCountPoint>& satSeries = _satCountHistory[source][summary.system];
        const qint64 epochKey = msmStatusUpdate->epochKey;
        if (!satSeries.isEmpty() && satSeries.last().epochKey == epochKey) {
            satSeries.last() = {summary.receivedUtc, epochKey, summary.satCount};
        } else {
            satSeries.push_back({summary.receivedUtc, epochKey, summary.satCount});
        }
        const int maxPoints = 180;
        while (satSeries.size() > maxPoints) {
            satSeries.removeFirst();
        }
        if (!msmStatusUpdate->correctedObservations.isEmpty()) {
            postOutputRinex(msmStatusUpdate->gpsWeek, msmStatusUpdate->gpsSow,
                            msmStatusUpdate->correctedObservations);
            if (shouldLogNow(_lastRnxLog, 2)) {
                appendLog(source,
                          QString("RINEX queued: week=%1 sow=%2 sats=%3 codeCorr=%4 phaseCorr=%5")
                              .arg(msmStatusUpdate->gpsWeek)
                              .arg(msmStatusUpdate->gpsSow, 0, 'f', 3)
                              .arg(msmStatusUpdate->correctedObservations.size())
                              .arg(msmStatusUpdate->correctedCodeCount)
                              .arg(msmStatusUpdate->correctedPhaseCount));
            }
        }

        if (_pppRuntimeActive && isPppStationEnabled(source) && _pppRuntimeService) {
            _pppRuntimeService->enqueueMsm(source, cachedMsm,
                                           msmStatusUpdate->glonassChannels);
        }

        if (source == selectedStatusPlotStreamId()) {
            scheduleSkyPlotRefresh();
        }
    }
}

std::array<double, 3> MainWindow::receiverEcefForSource(const QString& source) const {
    std::array<double, 3> receiver = {0.0, 0.0, 0.0};
    const PppSolution solution = _lastPppSolutionBySource.value(source);
    if (solution.valid) {
        receiver = {solution.xyz[0], solution.xyz[1], solution.xyz[2]};
    } else {
        const PppCrdStationOptions station = _pppOptions.crdStationsBySource.value(source);
        if (station.valid) {
            receiver = {station.aprXyz[0], station.aprXyz[1], station.aprXyz[2]};
        } else {
            const int row = rowForStreamId(source);
            bool latOk = false;
            bool lonOk = false;
            const double lat = row >= 0 ? _rows[row].lat.toDouble(&latOk) : 0.0;
            const double lon = row >= 0 ? _rows[row].lon.toDouble(&lonOk) : 0.0;
            if (latOk && lonOk && std::fabs(lat) <= 90.0 && std::fabs(lon) <= 180.0) {
                receiver = ellToXyz(lat * kPi / 180.0, lon * kPi / 180.0, 0.0);
            } else if (_pppOptions.hasApriori()) {
                receiver = {_pppOptions.aprXyz[0], _pppOptions.aprXyz[1], _pppOptions.aprXyz[2]};
            }
        }
    }
    return receiver;
}

void MainWindow::ingestBroadcastEphemeris(const QString& source, const RtcmBroadcastEphemeris& eph) {
    if (!eph.sat.isValid()) {
        return;
    }
    if (!isSupportedSolutionSystem(eph.sat.system)) {
        return;
    }
    if (_pppRuntimeService) {
        _pppRuntimeService->updateBroadcast(source, eph);
    }
}

void MainWindow::ingestSsrMessage(const QString& source, const RtcmSsrMessage& ssr) {
    if (ssr.satellites.isEmpty()) {
        return;
    }
    if (!isSupportedSolutionSystem(ssr.system)) {
        return;
    }
    if (_pppRuntimeService) {
        _pppRuntimeService->updateSsrMessage(source, ssr);
    }

    const QString kindText = [&]() -> QString {
        switch (ssr.kind) {
        case RtcmSsrCorrectionKind::Orbit: return "orbit";
        case RtcmSsrCorrectionKind::Clock: return "clock";
        case RtcmSsrCorrectionKind::OrbitClock: return "orbit+clock";
        case RtcmSsrCorrectionKind::CodeBias: return "code-bias";
        case RtcmSsrCorrectionKind::PhaseBias: return "phase-bias";
        case RtcmSsrCorrectionKind::HighRateClock: return "high-rate-clock";
        case RtcmSsrCorrectionKind::Ura: return "ura";
        default: break;
        }
        return "unknown";
    }();

    if (_telemetryEnabled) {
        postTelemetryReception(source, QDateTime::currentDateTimeUtc(), "SSR",
                               ssr.messageType, ssr.system, "received", QString(),
                               QString("kind=%1 satellites=%2")
                                   .arg(kindText).arg(ssr.satellites.size()));
    }

    if (ssr.kind == RtcmSsrCorrectionKind::PhaseBias || shouldLogNow(_lastCorrSummaryLog, 2)) {
        appendLog(source,
                  QString("SSR received: type=%1 kind=%2 sats=%3 epoch=%4 ui=%5")
                      .arg(ssr.messageType)
                      .arg(kindText)
                      .arg(ssr.satellites.size())
                      .arg(ssr.epochTime)
                      .arg(ssr.updateIntervalIndex));
    }

}

void MainWindow::rebuildDataPreparationStreamSelectors() {
    if (!_prepBroadcastCombo || !_prepOrbitCombo) {
        return;
    }
    auto rebuildCombo = [this](QComboBox* combo, bool allowNone, const QString& currentId) {
        const QSignalBlocker blocker(combo);
        combo->clear();
        if (allowNone) {
            combo->addItem("Not selected", QString());
        }
        for (const StreamRow& row : _rows) {
            const QString mp = row.config.value("mountPoint").toString();
            const QString label = mp.isEmpty()
                ? QString("%1 [%2]").arg(row.sourceDisplay).arg(row.id)
                : QString("%1 (%2) [%3]").arg(mp, row.sourceDisplay, row.id);
            combo->addItem(label, row.id);
        }
        int index = combo->findData(currentId);
        if (index < 0) {
            index = allowNone ? 0 : -1;
        }
        if (index >= 0) {
            combo->setCurrentIndex(index);
        }
    };

    rebuildCombo(_prepBroadcastCombo, false, _prepConfig.broadcastStreamId);
    rebuildCombo(_prepOrbitCombo, true, _prepConfig.orbitStreamId);
    rebuildCombo(_prepClockCombo, true, _prepConfig.clockStreamId);
    rebuildCombo(_prepCodeBiasCombo, true, _prepConfig.codeBiasStreamId);
    rebuildCombo(_prepPhaseBiasCombo, true, _prepConfig.phaseBiasStreamId);
    rebuildCombo(_prepObsCombo, true, _prepConfig.obsStreamId);
    {
        const QSignalBlocker blocker(_prepEnableCheck);
        _prepEnableCheck->setChecked(_prepConfig.enabled);
    }
    if (_sp3EnableCheck) {
        const QSignalBlocker blocker(_sp3EnableCheck);
        _sp3EnableCheck->setChecked(_prepConfig.sp3Enabled);
    }
    if (_sp3PathEdit) {
        const QSignalBlocker blocker(_sp3PathEdit);
        _sp3PathEdit->setText(_prepConfig.sp3Directory);
    }
    if (_sp3SampleSpin) {
        const QSignalBlocker blocker(_sp3SampleSpin);
        _sp3SampleSpin->setValue(_prepConfig.sp3SampleSec);
    }
    if (_obsSampleCombo) {
        const QSignalBlocker blocker(_obsSampleCombo);
        const int sampleIndex = _obsSampleCombo->findData(_prepConfig.observationSampleSec);
        _obsSampleCombo->setCurrentIndex(sampleIndex >= 0 ? sampleIndex : 0);
    }
    if (_rnxEnableCheck) {
        const QSignalBlocker blocker(_rnxEnableCheck);
        _rnxEnableCheck->setChecked(_prepConfig.rnxEnabled);
    }
    if (_rnxPathEdit) {
        const QSignalBlocker blocker(_rnxPathEdit);
        _rnxPathEdit->setText(_prepConfig.rnxDirectory);
    }
    if (_telemetryEnableCheck) {
        const QSignalBlocker blocker(_telemetryEnableCheck);
        _telemetryEnableCheck->setChecked(_prepConfig.telemetryEnabled);
    }
    if (_telemetryPathEdit) {
        const QSignalBlocker blocker(_telemetryPathEdit);
        _telemetryPathEdit->setText(_prepConfig.telemetryDirectory);
    }
    if (_prepLogPathEdit) {
        const QSignalBlocker blocker(_prepLogPathEdit);
        _prepLogPathEdit->setText(_prepConfig.logOutputPath);
    }
}

void MainWindow::applyDataPreparationSelection(bool writeLog) {
    if (!_prepBroadcastCombo) {
        return;
    }
    _prepConfig.enabled = _prepEnableCheck && _prepEnableCheck->isChecked();
    _prepConfig.broadcastStreamId = _prepBroadcastCombo->currentData().toString();
    _prepConfig.orbitStreamId = _prepOrbitCombo->currentData().toString();
    _prepConfig.clockStreamId = _prepClockCombo->currentData().toString();
    _prepConfig.codeBiasStreamId = _prepCodeBiasCombo->currentData().toString();
    _prepConfig.phaseBiasStreamId = _prepPhaseBiasCombo->currentData().toString();
    if (_prepConfig.codeBiasStreamId.isEmpty()) {
        _prepConfig.codeBiasStreamId = !_prepConfig.orbitStreamId.isEmpty()
                                           ? _prepConfig.orbitStreamId
                                           : _prepConfig.clockStreamId;
    }
    if (_prepConfig.phaseBiasStreamId.isEmpty()) {
        _prepConfig.phaseBiasStreamId = !_prepConfig.orbitStreamId.isEmpty()
                                            ? _prepConfig.orbitStreamId
                                            : _prepConfig.clockStreamId;
    }
    _prepConfig.obsStreamId = _prepObsCombo ? _prepObsCombo->currentData().toString() : QString();
    _prepConfig.observationSampleSec = _obsSampleCombo
        ? qBound(1, _obsSampleCombo->currentData().toInt(), 30) : 1;
    if (_prepConfig.observationSampleSec != 1 &&
        _prepConfig.observationSampleSec != 5 &&
        _prepConfig.observationSampleSec != 10 &&
        _prepConfig.observationSampleSec != 30) {
        _prepConfig.observationSampleSec = 1;
    }
    _prepConfig.sp3Enabled = _sp3EnableCheck && _sp3EnableCheck->isChecked();
    _prepConfig.sp3Directory = _sp3PathEdit ? _sp3PathEdit->text().trimmed() : QString();
    _prepConfig.sp3SampleSec = _sp3SampleSpin ? _sp3SampleSpin->value() : 10;
    _prepConfig.rnxEnabled = _rnxEnableCheck && _rnxEnableCheck->isChecked();
    _prepConfig.rnxDirectory = _rnxPathEdit ? _rnxPathEdit->text().trimmed() : QString();
    _prepConfig.telemetryEnabled = _telemetryEnableCheck && _telemetryEnableCheck->isChecked();
    _prepConfig.telemetryDirectory = _telemetryPathEdit ? _telemetryPathEdit->text().trimmed() : QString();
    _prepConfig.logOutputPath = _prepLogPathEdit ? _prepLogPathEdit->text().trimmed() : QString();
    _pppRuntimePolicy.epochIntervalSec = _prepConfig.observationSampleSec;
    postPppRuntimePolicy();

    PppStreamSelection pppSelection;
    pppSelection.broadcastId = _prepConfig.broadcastStreamId;
    pppSelection.orbitId = _prepConfig.orbitStreamId;
    pppSelection.clockId = _prepConfig.clockStreamId;
    pppSelection.codeBiasId = _prepConfig.codeBiasStreamId;
    pppSelection.phaseBiasId = _prepConfig.phaseBiasStreamId;
    pppSelection.obsId.clear();
    _pppSelection = pppSelection;
    postPppStreamSelection();

    const QString apcMount = [&]() -> QString {
        QString mp = streamMountpointById(_prepConfig.orbitStreamId);
        if (!mp.isEmpty()) return mp;
        mp = streamMountpointById(_prepConfig.clockStreamId);
        if (!mp.isEmpty()) return mp;
        return streamMountpointById(_prepConfig.broadcastStreamId);
    }();
    if (!apcMount.isEmpty()) {
        PppOptions opt = _pppOptions;
        opt.isApc = apcMount.contains("SSRA", Qt::CaseInsensitive);
        _pppOptions = opt;
        postPppOptions();
        if (_pppUseApcCheck) {
            _pppUseApcCheck->setChecked(opt.isApc);
        }
    }

    if (_prepConfig.sp3Enabled && _prepConfig.sp3Directory.isEmpty()) {
        _prepConfig.sp3Directory = "output/sp3";
        if (_sp3PathEdit) {
            _sp3PathEdit->setText(_prepConfig.sp3Directory);
        }
    }
    if (_prepConfig.rnxEnabled && _prepConfig.rnxDirectory.isEmpty()) {
        _prepConfig.rnxDirectory = "output/rinex";
        if (_rnxPathEdit) {
            _rnxPathEdit->setText(_prepConfig.rnxDirectory);
        }
    }
    if (_prepConfig.telemetryEnabled && _prepConfig.telemetryDirectory.isEmpty()) {
        _prepConfig.telemetryDirectory = "output/telemetry";
        if (_telemetryPathEdit) {
            _telemetryPathEdit->setText(_prepConfig.telemetryDirectory);
        }
    }

    saveDataPreparationToSettings();
    _preparedStates.clear();
    refreshDataPreparationTable();

    const QString summary = QString("Data Preparation: broadcast=%1 orbit=%2 clock=%3 code=%4 phase=%5 obs=%6 processing=%7s enabled=%8 sp3/clk=%9 (%10s) rnx=%11 log=%12")
                                .arg(streamMountpointById(_prepConfig.broadcastStreamId).isEmpty()
                                         ? streamLabelById(_prepConfig.broadcastStreamId)
                                         : streamMountpointById(_prepConfig.broadcastStreamId))
                                .arg(streamMountpointById(_prepConfig.orbitStreamId).isEmpty()
                                         ? streamLabelById(_prepConfig.orbitStreamId)
                                         : streamMountpointById(_prepConfig.orbitStreamId))
                                .arg(streamMountpointById(_prepConfig.clockStreamId).isEmpty()
                                         ? streamLabelById(_prepConfig.clockStreamId)
                                         : streamMountpointById(_prepConfig.clockStreamId))
                                .arg(streamMountpointById(_prepConfig.codeBiasStreamId).isEmpty()
                                         ? streamLabelById(_prepConfig.codeBiasStreamId)
                                         : streamMountpointById(_prepConfig.codeBiasStreamId))
                                .arg(streamMountpointById(_prepConfig.phaseBiasStreamId).isEmpty()
                                         ? streamLabelById(_prepConfig.phaseBiasStreamId)
                                         : streamMountpointById(_prepConfig.phaseBiasStreamId))
                                .arg(streamMountpointById(_prepConfig.obsStreamId).isEmpty()
                                         ? streamLabelById(_prepConfig.obsStreamId)
                                         : streamMountpointById(_prepConfig.obsStreamId))
                                .arg(_prepConfig.observationSampleSec)
                                .arg(_prepConfig.enabled ? "yes" : "no")
                                .arg(_prepConfig.sp3Enabled ? _prepConfig.sp3Directory : "off")
                                .arg(_prepConfig.sp3SampleSec)
                                .arg(_prepConfig.rnxEnabled ? _prepConfig.rnxDirectory : "off")
                                .arg(resolvedPrepLogFilePath().isEmpty() ? "off" : resolvedPrepLogFilePath());
    setTelemetryOutputDirectory(_prepConfig.telemetryEnabled
                                    ? _prepConfig.telemetryDirectory : QString());
    configureRtcmProcessingService();
    configureOutputWorker();
    if (_prepSummaryLabel) {
        _prepSummaryLabel->setText(summary);
    }
    if (writeLog) {
        appendLog("SYSTEM", summary);
    }

    if (shouldLogNow(_lastSp3Log, 2)) {
        appendLog("SP3",
                  QString("Output config: enabled=%1 dir=%2 sample=%3s broadcast=%4 orbit=%5 clock=%6")
                      .arg(_prepConfig.sp3Enabled ? "yes" : "no")
                      .arg(_prepConfig.sp3Directory.isEmpty() ? "-" : _prepConfig.sp3Directory)
                      .arg(_prepConfig.sp3SampleSec)
                      .arg(prepStreamLabel(_prepConfig.broadcastStreamId,
                                           streamMountpointById(_prepConfig.broadcastStreamId),
                                           streamLabelById(_prepConfig.broadcastStreamId)))
                      .arg(prepStreamLabel(_prepConfig.orbitStreamId,
                                           streamMountpointById(_prepConfig.orbitStreamId),
                                           streamLabelById(_prepConfig.orbitStreamId)))
                      .arg(prepStreamLabel(_prepConfig.clockStreamId,
                                           streamMountpointById(_prepConfig.clockStreamId),
                                           streamLabelById(_prepConfig.clockStreamId))));
    }
    if (shouldLogNow(_lastRnxLog, 2)) {
        appendLog("RINEX",
                  QString("Output config: enabled=%1 dir=%2 obs=%3 code=%4 phase=%5")
                      .arg(_prepConfig.rnxEnabled ? "yes" : "no")
                      .arg(_prepConfig.rnxDirectory.isEmpty() ? "-" : _prepConfig.rnxDirectory)
                      .arg(prepStreamLabel(_prepConfig.obsStreamId,
                                           streamMountpointById(_prepConfig.obsStreamId),
                                           streamLabelById(_prepConfig.obsStreamId)))
                      .arg(prepStreamLabel(_prepConfig.codeBiasStreamId,
                                           streamMountpointById(_prepConfig.codeBiasStreamId),
                                           streamLabelById(_prepConfig.codeBiasStreamId)))
                      .arg(prepStreamLabel(_prepConfig.phaseBiasStreamId,
                                           streamMountpointById(_prepConfig.phaseBiasStreamId),
                                           streamLabelById(_prepConfig.phaseBiasStreamId))));
    }
}

void MainWindow::refreshDataPreparationTable() {
    if (!_prepTable) {
        return;
    }
    ScopedTableSorting sorting(_prepTable);
    _prepTable->setRowCount(0);
    _prepTable->setRowCount(_preparedStates.size());
    int row = 0;
    for (auto it = _preparedStates.constBegin(); it != _preparedStates.constEnd(); ++it, ++row) {
        const PreparedState& prep = it.value();
        const QString satLabel = QString("%1%2/%3")
                                     .arg(prep.sat.system)
                                     .arg(prep.sat.number, 2, 10, QLatin1Char('0'))
                                     .arg(navTypeToString(prep.sat.navType));
        const QString broadcastIod = prep.broadcastIod >= 0 ? QString::number(prep.broadcastIod) : "-";
        const QString orbitStatus = prep.orbitApplied
            ? QString("Applied dt=%1s dR=(%2,%3,%4)")
                  .arg(prep.orbitDtSec, 0, 'f', 1)
                  .arg(prep.orbitRsw[0], 0, 'f', 4)
                  .arg(prep.orbitRsw[1], 0, 'f', 4)
                  .arg(prep.orbitRsw[2], 0, 'f', 4)
            : (prep.orbitStatus.isEmpty() ? "-" : prep.orbitStatus);
        const QString clockStatus = prep.clockApplied
            ? QString("Applied dt=%1s dT=%2s")
                  .arg(prep.clockDtSec, 0, 'f', 1)
                  .arg(prep.clockSec, 0, 'e', 3)
            : (prep.clockStatus.isEmpty() ? "-" : prep.clockStatus);
        const QString codeStatus = prep.codeApplied ? QString("Biases=%1").arg(prep.codeBiasCount) : "-";
        const QString phaseStatus = prep.phaseApplied ? QString("Biases=%1").arg(prep.phaseBiasCount) : "-";
        const QString datum = prep.datumText.isEmpty() ? "-" : prep.datumText;
        const QString updated = prep.updatedUtc.isValid()
            ? prep.updatedUtc.toString("HH:mm:ss") : "-";

        const QStringList values = {satLabel, broadcastIod, orbitStatus, clockStatus, codeStatus, phaseStatus, datum, updated};
        for (int col = 0; col < values.size(); ++col) {
            QTableWidgetItem* item = _prepTable->item(row, col);
            if (!item) {
                item = new QTableWidgetItem;
                _prepTable->setItem(row, col, item);
            }
            item->setText(values[col]);
        }
    }
}

QString MainWindow::streamLabelById(const QString& id) const {
    if (id.isEmpty()) {
        return "-";
    }
    for (const StreamRow& row : _rows) {
        if (row.id == id) {
            return row.sourceDisplay;
        }
    }
    return QString("stream-%1").arg(id);
}

QString MainWindow::streamMountpointById(const QString& id) const {
    for (const StreamRow& row : _rows) {
        if (row.id == id) {
            return row.config.value("mountPoint").toString();
        }
    }
    return QString();
}

QString MainWindow::selectedStatusPlotStreamId() const {
    if (_statusPlotStationCombo && _statusPlotStationCombo->currentIndex() >= 0) {
        const QString streamId = _statusPlotStationCombo->currentData().toString();
        if (!streamId.isEmpty()) {
            return streamId;
        }
    }
    if (_statusTable && _statusTable->selectionModel()) {
        const QModelIndexList selected = _statusTable->selectionModel()->selectedRows();
        if (!selected.isEmpty()) {
            const QString streamId = tableRowStreamId(_statusTable, selected.first().row());
            if (isStatusPlotStreamSource(streamId)) {
                return streamId;
            }
        }
    }
    return QString();
}

int MainWindow::rowForStreamId(const QString& streamId) const {
    if (streamId.isEmpty()) {
        return -1;
    }
    for (int row = 0; row < _rows.size(); ++row) {
        if (_rows[row].id == streamId) {
            return row;
        }
    }
    return -1;
}

bool MainWindow::isStatusPlotStreamSource(const QString& sourceId) const {
    if (sourceId.isEmpty()) {
        return false;
    }
    if (_msmSummaries.contains(sourceId) && !_msmSummaries.value(sourceId).isEmpty()) {
        return true;
    }
    if (_lastMsmMessages.contains(sourceId) && !_lastMsmMessages.value(sourceId).isEmpty()) {
        return true;
    }
    if (!_prepConfig.obsStreamId.isEmpty() && sourceId == _prepConfig.obsStreamId) {
        return true;
    }
    if (sourceId == _prepConfig.broadcastStreamId ||
        sourceId == _prepConfig.orbitStreamId ||
        sourceId == _prepConfig.clockStreamId ||
        sourceId == _prepConfig.codeBiasStreamId ||
        sourceId == _prepConfig.phaseBiasStreamId) {
        return false;
    }
    return false;
}

void MainWindow::refreshStatusPlotStationSelector() {
    if (!_statusPlotStationCombo) {
        return;
    }

    QString selectedId = _statusPlotStationCombo->currentData().toString();
    if (selectedId.isEmpty() && _statusTable && _statusTable->selectionModel()) {
        const QModelIndexList selected = _statusTable->selectionModel()->selectedRows();
        if (!selected.isEmpty()) {
            const QString streamId = tableRowStreamId(_statusTable, selected.first().row());
            if (isStatusPlotStreamSource(streamId)) {
                selectedId = streamId;
            }
        }
    }

    QSignalBlocker blocker(_statusPlotStationCombo);
    _statusPlotStationCombo->clear();
    for (const StreamRow& row : _rows) {
        if (!isStatusPlotStreamSource(row.id)) {
            continue;
        }
        const QString mountpoint = row.config.value("mountPoint").toString();
        const QString label = mountpoint.isEmpty() ? row.sourceDisplay : mountpoint;
        _statusPlotStationCombo->addItem(QString("%1 [%2]").arg(label, row.id), row.id);
    }

    int index = _statusPlotStationCombo->findData(selectedId);
    if (index < 0 && _statusPlotStationCombo->count() > 0) {
        index = 0;
    }
    if (index >= 0) {
        _statusPlotStationCombo->setCurrentIndex(index);
    }
    _statusPlotStationCombo->setEnabled(_statusPlotStationCombo->count() > 0);
}

bool MainWindow::saveDataPreparationToSettings(QString* error) {
    QSettings s(settingsFilePath(), QSettings::IniFormat);
    s.setValue("prep/enabled", _prepConfig.enabled);
    s.setValue("prep/broadcast", _prepConfig.broadcastStreamId);
    s.setValue("prep/orbit", _prepConfig.orbitStreamId);
    s.setValue("prep/clock", _prepConfig.clockStreamId);
    s.setValue("prep/code", _prepConfig.codeBiasStreamId);
    s.setValue("prep/phase", _prepConfig.phaseBiasStreamId);
    s.setValue("prep/obs", _prepConfig.obsStreamId);
    s.setValue("prep/observationSample", _prepConfig.observationSampleSec);
    s.setValue("prep/sp3Enabled", _prepConfig.sp3Enabled);
    s.setValue("prep/sp3Dir", _prepConfig.sp3Directory);
    s.setValue("prep/sp3Sample", _prepConfig.sp3SampleSec);
    s.setValue("prep/rnxEnabled", _prepConfig.rnxEnabled);
    s.setValue("prep/rnxDir", _prepConfig.rnxDirectory);
    s.setValue("prep/telemetryEnabled", _prepConfig.telemetryEnabled);
    s.setValue("prep/telemetryDir", _prepConfig.telemetryDirectory);
    s.setValue("prep/logOutputPath", _prepConfig.logOutputPath);
    s.sync();
    if (s.status() != QSettings::NoError) {
        const QString message = QStringLiteral("cannot write settings %1 (status=%2)")
                                    .arg(QFileInfo(settingsFilePath()).absoluteFilePath())
                                    .arg(static_cast<int>(s.status()));
        if (error) *error = message;
        else qWarning().noquote() << message;
        return false;
    }
    if (error) error->clear();
    return true;
}

void MainWindow::loadDataPreparationFromSettings() {
    QSettings s(settingsFilePath(), QSettings::IniFormat);
    _prepConfig.enabled = s.value("prep/enabled", false).toBool();
    _prepConfig.broadcastStreamId = s.value("prep/broadcast").toString();
    _prepConfig.orbitStreamId = s.value("prep/orbit").toString();
    _prepConfig.clockStreamId = s.value("prep/clock").toString();
    _prepConfig.codeBiasStreamId = s.value("prep/code").toString();
    _prepConfig.phaseBiasStreamId = s.value("prep/phase").toString();
    if (_prepConfig.codeBiasStreamId.isEmpty()) {
        _prepConfig.codeBiasStreamId = !_prepConfig.orbitStreamId.isEmpty()
                                           ? _prepConfig.orbitStreamId
                                           : _prepConfig.clockStreamId;
    }
    if (_prepConfig.phaseBiasStreamId.isEmpty()) {
        _prepConfig.phaseBiasStreamId = !_prepConfig.orbitStreamId.isEmpty()
                                            ? _prepConfig.orbitStreamId
                                            : _prepConfig.clockStreamId;
    }
    _prepConfig.obsStreamId = s.value("prep/obs").toString();
    const int configuredObservationSample = s.value("prep/observationSample", 1).toInt();
    _prepConfig.observationSampleSec = (configuredObservationSample == 5 ||
                                        configuredObservationSample == 10 ||
                                        configuredObservationSample == 30)
        ? configuredObservationSample : 1;
    _prepConfig.sp3Enabled = s.value("prep/sp3Enabled", false).toBool();
    _prepConfig.sp3Directory = s.value("prep/sp3Dir", QString("output/sp3")).toString();
    _prepConfig.sp3SampleSec = s.value("prep/sp3Sample", 10).toInt();
    _prepConfig.rnxEnabled = s.value("prep/rnxEnabled", false).toBool();
    _prepConfig.rnxDirectory = s.value("prep/rnxDir", QString("output/rinex")).toString();
    _prepConfig.telemetryEnabled = s.value("prep/telemetryEnabled", false).toBool();
    _prepConfig.telemetryDirectory = s.value("prep/telemetryDir", QString("output/telemetry")).toString();
    _prepConfig.logOutputPath = s.value("prep/logOutputPath", QString("output/logs")).toString();
    rebuildDataPreparationStreamSelectors();
}

bool MainWindow::isPppArModeAvailable() const {
    const bool useBieAlgorithm = _pppArAlgorithmCombo &&
                           _pppArAlgorithmCombo->currentData().toString() == QLatin1String("BIE");
    auto systemAllowsAr = [useBieAlgorithm](const QCheckBox* enabled,
                                      const QCheckBox* arEnabled,
                                      const QComboBox* lcCombo) {
        if (!enabled || !enabled->isChecked()) {
            return true;
        }
        if (useBieAlgorithm && (!arEnabled || !arEnabled->isChecked())) {
            return true;
        }
        const QString mode = lcCombo ? lcCombo->currentText().trimmed() : QString();
        return useBieAlgorithm ? (mode == QLatin1String("Pi&Li") || mode == QLatin1String("P1&L1"))
                         : mode == QLatin1String("Pi&Li");
    };
    return systemAllowsAr(_pppUseGpsCheck, _pppArUseGpsCheck, _pppLcGpsCombo) &&
           systemAllowsAr(_pppUseGloCheck, _pppArUseGloCheck, _pppLcGloCombo) &&
           systemAllowsAr(_pppUseGalCheck, _pppArUseGalCheck, _pppLcGalCombo) &&
           systemAllowsAr(_pppUseBdsCheck, _pppArUseBdsCheck, _pppLcBdsCombo);
}

void MainWindow::updatePppArAvailability() {
    if (!_pppArEnableCheck) {
        return;
    }
    const bool available = isPppArModeAvailable();
    const bool useBieAlgorithm = _pppArAlgorithmCombo &&
                           _pppArAlgorithmCombo->currentData().toString() == QLatin1String("BIE");
    if (!available && _pppArEnableCheck->isChecked()) {
        const QSignalBlocker blocker(_pppArEnableCheck);
        _pppArEnableCheck->setChecked(false);
    }
    _pppArEnableCheck->setEnabled(available);
    _pppArEnableCheck->setToolTip(available
        ? (useBieAlgorithm
               ? QStringLiteral("BIE uses cycle-valued L1/L2 ambiguity groups and standard screening.")
               : QStringLiteral("LAMBDA PPP-AR uses dual-frequency Pi&Li MW/NL processing."))
        : (useBieAlgorithm
               ? QStringLiteral("BIE requires Pi&Li or P1&L1 so ambiguity states are expressed in cycles.")
               : QStringLiteral("LAMBDA PPP-AR requires all enabled GNSS systems to use Pi&Li mode.")));
    const bool arEnabled = available && _pppArEnableCheck->isChecked();
    const bool historyEnabled = arEnabled && !useBieAlgorithm &&
                                _pppArUseHistoryAidCheck && _pppArUseHistoryAidCheck->isChecked();
    if (_pppArUseGpsCheck) _pppArUseGpsCheck->setEnabled(arEnabled && useBieAlgorithm);
    if (_pppArUseGloCheck) _pppArUseGloCheck->setEnabled(arEnabled);
    if (_pppArUseGalCheck) _pppArUseGalCheck->setEnabled(arEnabled && useBieAlgorithm);
    if (_pppArUseBdsCheck) _pppArUseBdsCheck->setEnabled(arEnabled && useBieAlgorithm);
    if (_pppArMinEleSpin) _pppArMinEleSpin->setEnabled(arEnabled);
    if (_pppArMinMwSamplesSpin) _pppArMinMwSamplesSpin->setEnabled(arEnabled);
    if (_pppArMinFixPairsSpin) _pppArMinFixPairsSpin->setEnabled(arEnabled);
    if (_pppArWlMaxDevSpin) _pppArWlMaxDevSpin->setEnabled(arEnabled);
    if (_pppArWlMaxSigSpin) _pppArWlMaxSigSpin->setEnabled(arEnabled);
    if (_pppArNlRatioSpin) _pppArNlRatioSpin->setEnabled(arEnabled);
    if (_pppArNlBootSpin) _pppArNlBootSpin->setEnabled(arEnabled);
    if (_pppArNlMaxDevSpin) _pppArNlMaxDevSpin->setEnabled(arEnabled);
    if (_pppArConstraintSigmaSpin) _pppArConstraintSigmaSpin->setEnabled(arEnabled);
    if (_pppArLogPathEdit) _pppArLogPathEdit->setEnabled(arEnabled);
    if (_pppArLogBrowseButton) _pppArLogBrowseButton->setEnabled(arEnabled);
    if (_pppArUseHistoryAidCheck) _pppArUseHistoryAidCheck->setEnabled(arEnabled && !useBieAlgorithm);
    if (_pppArHistMinFixEpochsSpin) _pppArHistMinFixEpochsSpin->setEnabled(historyEnabled);
    if (_pppArHistMinLockEpochsSpin) _pppArHistMinLockEpochsSpin->setEnabled(historyEnabled);
    if (_pppArPartialWarmupLockEpochsSpin) _pppArPartialWarmupLockEpochsSpin->setEnabled(historyEnabled);
    if (_pppArHistRelaxFactorSpin) _pppArHistRelaxFactorSpin->setEnabled(historyEnabled);
    const bool lambdaEnabled = arEnabled && !useBieAlgorithm;
    if (_pppArBieMinEpochsSpin) _pppArBieMinEpochsSpin->setEnabled(arEnabled && useBieAlgorithm);
    if (_pppArBieMinSatSpin) _pppArBieMinSatSpin->setEnabled(arEnabled && useBieAlgorithm);
    if (_pppArBieMaxFracSpin) _pppArBieMaxFracSpin->setEnabled(arEnabled && useBieAlgorithm);
    if (_pppArBieMaxSigSpin) _pppArBieMaxSigSpin->setEnabled(arEnabled && useBieAlgorithm);
    if (_pppArBieConstraintSigmaSpin) _pppArBieConstraintSigmaSpin->setEnabled(arEnabled && useBieAlgorithm);
    if (_pppArBiePersistConstraintsCheck) _pppArBiePersistConstraintsCheck->setEnabled(arEnabled && useBieAlgorithm);
    const bool biePersistenceEnabled = arEnabled && useBieAlgorithm &&
                                       _pppArBiePersistConstraintsCheck &&
                                       _pppArBiePersistConstraintsCheck->isChecked();
    if (_pppArBiePersistConfirmEpochsSpin) _pppArBiePersistConfirmEpochsSpin->setEnabled(biePersistenceEnabled);
    if (_pppArBiePersistMaxVerticalSpin) _pppArBiePersistMaxVerticalSpin->setEnabled(biePersistenceEnabled);
    if (_pppArBiePersistMaxZtdSpin) _pppArBiePersistMaxZtdSpin->setEnabled(biePersistenceEnabled);
    if (_pppArMinMwSamplesSpin) _pppArMinMwSamplesSpin->setEnabled(lambdaEnabled);
    if (_pppArMinFixPairsSpin) _pppArMinFixPairsSpin->setEnabled(lambdaEnabled);
    if (_pppArWlMaxDevSpin) _pppArWlMaxDevSpin->setEnabled(lambdaEnabled);
    if (_pppArWlMaxSigSpin) _pppArWlMaxSigSpin->setEnabled(lambdaEnabled);
    if (_pppArNlRatioSpin) _pppArNlRatioSpin->setEnabled(lambdaEnabled);
    if (_pppArNlBootSpin) _pppArNlBootSpin->setEnabled(lambdaEnabled);
    if (_pppArNlMaxDevSpin) _pppArNlMaxDevSpin->setEnabled(lambdaEnabled);
    if (_pppArConstraintSigmaSpin) _pppArConstraintSigmaSpin->setEnabled(lambdaEnabled);
}

void MainWindow::applyPppSettings(bool writeLog) {
    if (!_pppSigmaC1Spin) {
        return;
    }
    updatePppArAvailability();
    PppOptions opt = _pppOptions;
    opt.useGps = _pppUseGpsCheck && _pppUseGpsCheck->isChecked();
    opt.useGlo = _pppUseGloCheck && _pppUseGloCheck->isChecked();
    opt.useGal = _pppUseGalCheck && _pppUseGalCheck->isChecked();
    opt.useBds = _pppUseBdsCheck && _pppUseBdsCheck->isChecked();
    opt.signalPriorities = _pppSignalPrioritiesEdit ? _pppSignalPrioritiesEdit->text().trimmed() : opt.signalPriorities;
    if (opt.signalPriorities.isEmpty()) {
        opt.signalPriorities = QStringLiteral("G:12&CWPSLX R:12&CP E:1&CBX E:5&QIX C:26&IQX");
    }
    opt.lcGps = _pppLcGpsCombo ? _pppLcGpsCombo->currentText() : opt.lcGps;
    opt.lcGlo = _pppLcGloCombo ? _pppLcGloCombo->currentText() : opt.lcGlo;
    opt.lcGal = _pppLcGalCombo ? _pppLcGalCombo->currentText() : opt.lcGal;
    opt.lcBds = _pppLcBdsCombo ? _pppLcBdsCombo->currentText() : opt.lcBds;
    opt.pseudoObsIono = _pppPseudoObsIonoCheck && _pppPseudoObsIonoCheck->isChecked();
    opt.sigmaC1 = _pppSigmaC1Spin->value();
    opt.sigmaL1 = _pppSigmaL1Spin->value();
    opt.maxResC1 = _pppMaxResC1Spin->value();
    opt.maxResL1 = _pppMaxResL1Spin->value();
    opt.minEleRad = (_pppMinEleSpin->value() * kPi) / 180.0;
    opt.minObs = _pppMinObsSpin->value();
    opt.eleWgtCode = _pppEleWgtCodeCheck && _pppEleWgtCodeCheck->isChecked();
    opt.eleWgtPhase = _pppEleWgtPhaseCheck && _pppEleWgtPhaseCheck->isChecked();
    opt.aprSigCrd[0] = _pppAprSigXSpin->value();
    opt.aprSigCrd[1] = _pppAprSigYSpin->value();
    opt.aprSigCrd[2] = _pppAprSigZSpin->value();
    opt.noiseCrd[0] = _pppNoiseXSpin->value();
    opt.noiseCrd[1] = _pppNoiseYSpin->value();
    opt.noiseCrd[2] = _pppNoiseZSpin->value();
    opt.aprSigClk = _pppAprSigClkSpin->value();
    opt.aprSigTrp = _pppAprSigTrpSpin->value();
    opt.noiseTrp = _pppNoiseTrpSpin->value();
    opt.aprSigAmb = _pppAprSigAmbSpin->value();
    opt.seedingTime = _pppSeedingSpin->value();
    if (_pppTropoModeCombo) {
        opt.troposphereMode = _pppTropoModeCombo->currentData().toString() == QLatin1String("gfs")
            ? PppTroposphereMode::GfsZwdInitialization : PppTroposphereMode::Original;
    }
    if (_pppGfsMethodCombo) {
        opt.gfsZwdMethod = _pppGfsMethodCombo->currentData().toString() == QLatin1String("askne")
            ? GfsZwdMethod::AskneNordius : GfsZwdMethod::ProfileIntegral;
    }
    if (_pppMappingModeCombo) {
        opt.mappingMode = _pppMappingModeCombo->currentData().toString() == QLatin1String("gpt3")
            ? PppMappingMode::Gpt3Vmf3 : PppMappingMode::SimpleSin;
    }
    if (_pppGfsCachePathEdit) opt.gfsCacheDirectory = _pppGfsCachePathEdit->text().trimmed();
    if (_pppGfsTimeoutSpin) opt.gfsPrefetchTimeoutSec = qMax(1, static_cast<int>(std::round(_pppGfsTimeoutSpin->value())));
    opt.aprXyz[0] = _pppAprXSpin->value();
    opt.aprXyz[1] = _pppAprYSpin->value();
    opt.aprXyz[2] = _pppAprZSpin->value();
    opt.neuEcc[0] = _pppNeuNSpin->value();
    opt.neuEcc[1] = _pppNeuESpin->value();
    opt.neuEcc[2] = _pppNeuUSpin->value();
    opt.crdFile = _pppCrdPathEdit ? _pppCrdPathEdit->text().trimmed() : QString();
    opt.useAntex = _pppUseAntexCheck && _pppUseAntexCheck->isChecked();
    opt.antexFile = _pppAntexPathEdit ? _pppAntexPathEdit->text().trimmed() : QString();
    opt.roverAntenna = _pppRoverAntennaEdit ? _pppRoverAntennaEdit->text().trimmed().toUpper() : QString();
    opt.isApc = _pppUseApcCheck && _pppUseApcCheck->isChecked();
    opt.useOceanTide = _pppUseOceanCheck && _pppUseOceanCheck->isChecked();
    opt.blqFile = _pppBlqPathEdit ? _pppBlqPathEdit->text().trimmed() : QString();
    opt.oceanStation = _pppOceanStationEdit ? _pppOceanStationEdit->text().trimmed().toUpper() : QString();
    opt.vtecFile = _pppVtecPathEdit ? _pppVtecPathEdit->text().trimmed() : QString();
    if (_pppArAlgorithmCombo) {
        opt.arAlgorithm = _pppArAlgorithmCombo->currentData().toString();
    }
    opt.enablePppAr = isPppArModeAvailable() &&
                       _pppArEnableCheck &&
                       _pppArEnableCheck->isChecked();
    opt.arUseGps = _pppArUseGpsCheck && _pppArUseGpsCheck->isChecked();
    opt.arUseGlonass = _pppArUseGloCheck && _pppArUseGloCheck->isChecked();
    opt.arUseGalileo = _pppArUseGalCheck && _pppArUseGalCheck->isChecked();
    opt.arUseBds = _pppArUseBdsCheck && _pppArUseBdsCheck->isChecked();
    opt.arMinEleDeg = _pppArMinEleSpin ? _pppArMinEleSpin->value() : opt.arMinEleDeg;
    opt.arMinMwSamples = _pppArMinMwSamplesSpin ? _pppArMinMwSamplesSpin->value() : opt.arMinMwSamples;
    opt.arMinFixPairs = _pppArMinFixPairsSpin ? _pppArMinFixPairsSpin->value() : opt.arMinFixPairs;
    opt.arWlMaxDev = _pppArWlMaxDevSpin ? _pppArWlMaxDevSpin->value() : opt.arWlMaxDev;
    opt.arWlMaxSig = _pppArWlMaxSigSpin ? _pppArWlMaxSigSpin->value() : opt.arWlMaxSig;
    opt.arNlRatio = _pppArNlRatioSpin ? _pppArNlRatioSpin->value() : opt.arNlRatio;
    opt.arNlBoot = _pppArNlBootSpin ? _pppArNlBootSpin->value() : opt.arNlBoot;
    opt.arNlMaxDev = _pppArNlMaxDevSpin ? _pppArNlMaxDevSpin->value() : opt.arNlMaxDev;
    opt.arConstraintSigma = _pppArConstraintSigmaSpin ? _pppArConstraintSigmaSpin->value() : opt.arConstraintSigma;
    opt.arBieMinEpochs = _pppArBieMinEpochsSpin ? _pppArBieMinEpochsSpin->value() : opt.arBieMinEpochs;
    opt.arBieMinSat = _pppArBieMinSatSpin ? _pppArBieMinSatSpin->value() : opt.arBieMinSat;
    opt.arBieMaxFrac = _pppArBieMaxFracSpin ? _pppArBieMaxFracSpin->value() : opt.arBieMaxFrac;
    opt.arBieMaxSig = _pppArBieMaxSigSpin ? _pppArBieMaxSigSpin->value() : opt.arBieMaxSig;
    opt.arBieConstraintSigma = _pppArBieConstraintSigmaSpin ? _pppArBieConstraintSigmaSpin->value() : opt.arBieConstraintSigma;
    opt.arBiePersistConstraints = _pppArBiePersistConstraintsCheck &&
                                  _pppArBiePersistConstraintsCheck->isChecked();
    opt.arBiePersistConfirmEpochs = _pppArBiePersistConfirmEpochsSpin
        ? _pppArBiePersistConfirmEpochsSpin->value() : opt.arBiePersistConfirmEpochs;
    opt.arBiePersistMaxVerticalM = _pppArBiePersistMaxVerticalSpin
        ? _pppArBiePersistMaxVerticalSpin->value() : opt.arBiePersistMaxVerticalM;
    opt.arBiePersistMaxZtdM = _pppArBiePersistMaxZtdSpin
        ? _pppArBiePersistMaxZtdSpin->value() : opt.arBiePersistMaxZtdM;
    opt.arUseHistoryAid = _pppArUseHistoryAidCheck && _pppArUseHistoryAidCheck->isChecked();
    opt.arHistMinFixEpochs = _pppArHistMinFixEpochsSpin ? _pppArHistMinFixEpochsSpin->value() : opt.arHistMinFixEpochs;
    opt.arHistMinLockEpochs = _pppArHistMinLockEpochsSpin ? _pppArHistMinLockEpochsSpin->value() : opt.arHistMinLockEpochs;
    opt.arPartialWarmupLockEpochs = _pppArPartialWarmupLockEpochsSpin ? _pppArPartialWarmupLockEpochsSpin->value() : opt.arPartialWarmupLockEpochs;
    opt.arHistRelaxFactor = _pppArHistRelaxFactorSpin ? _pppArHistRelaxFactorSpin->value() : opt.arHistRelaxFactor;
    opt.arLogFile = _pppArLogPathEdit ? _pppArLogPathEdit->text().trimmed() : QString();
    _ztdOutputDirectory = _pppZtdPathEdit ? _pppZtdPathEdit->text().trimmed() : QString();
    _pppJsonEnabled = _pppJsonEnableCheck && _pppJsonEnableCheck->isChecked();
    _pppJsonOutputDirectory = _pppJsonPathEdit
        ? _pppJsonPathEdit->text().trimmed()
        : QStringLiteral("output/ppp_json");
    if (_pppJsonOutputDirectory.isEmpty()) {
        _pppJsonOutputDirectory = QStringLiteral("output/ppp_json");
        if (_pppJsonPathEdit) {
            _pppJsonPathEdit->setText(_pppJsonOutputDirectory);
        }
    }
    configureOutputWorker();

    QString crdMatchedStation;
    int crdMatchedCount = 0;
    opt.crdStationsBySource.clear();
    if (!opt.crdFile.isEmpty()) {
        QString roverSource = _prepConfig.obsStreamId;
        if (roverSource.isEmpty() || !isPppStationEnabled(roverSource)) {
            for (const StreamRow& row : _rows) {
                if (isPppStationEnabled(row.id)) {
                    roverSource = row.id;
                    break;
                }
            }
        }
        const QMap<QString, PppCrdEntry> crdEntries = readPppCrdFile(opt.crdFile);
        for (const StreamRow& row : _rows) {
            for (const QString& candidate : pppCrdStationCandidates(row.id,
                                                                    streamMountpointById(row.id),
                                                                    streamLabelById(row.id))) {
                const auto it = crdEntries.constFind(candidate);
                if (it == crdEntries.constEnd()) {
                    continue;
                }
                const PppCrdEntry& entry = it.value();
                PppCrdStationOptions stationOpt;
                stationOpt.valid = true;
                stationOpt.station = entry.station;
                stationOpt.roverAntenna = entry.antenna;
                stationOpt.receiver = entry.receiver;
                for (int i = 0; i < 3; ++i) {
                    stationOpt.aprXyz[i] = entry.xyz[i];
                    stationOpt.neuEcc[i] = entry.neu[i];
                }
                opt.crdStationsBySource[row.id] = stationOpt;
                ++crdMatchedCount;

                if (row.id == roverSource) {
                    for (int i = 0; i < 3; ++i) {
                        opt.aprXyz[i] = entry.xyz[i];
                        opt.neuEcc[i] = entry.neu[i];
                    }
                    opt.roverAntenna = entry.antenna;
                    crdMatchedStation = entry.station;
                    if (_pppAprXSpin) _pppAprXSpin->setValue(opt.aprXyz[0]);
                    if (_pppAprYSpin) _pppAprYSpin->setValue(opt.aprXyz[1]);
                    if (_pppAprZSpin) _pppAprZSpin->setValue(opt.aprXyz[2]);
                    if (_pppNeuNSpin) _pppNeuNSpin->setValue(opt.neuEcc[0]);
                    if (_pppNeuESpin) _pppNeuESpin->setValue(opt.neuEcc[1]);
                    if (_pppNeuUSpin) _pppNeuUSpin->setValue(opt.neuEcc[2]);
                    if (_pppRoverAntennaEdit) _pppRoverAntennaEdit->setText(opt.roverAntenna);
                    if (opt.oceanStation.isEmpty() && _pppOceanStationEdit) {
                        opt.oceanStation = entry.station;
                        _pppOceanStationEdit->setText(opt.oceanStation);
                    }
                }
                break;
            }
        }
    }

    _pppOptions = opt;
    postPppOptions();
    postPppRuntimePolicy();
    int enabledStationCount = 0;
    for (const StreamRow& row : _rows) {
        if (isPppStationEnabled(row.id)) {
            ++enabledStationCount;
        }
    }
    savePppSettingsToSettings();

    if (writeLog) {
        appendLog("PPP",
                  QString("PPP settings applied: minEle=%1deg minObs=%2 sigC1=%3 sigL1=%4 systems=%5%6%7%8 stations=%9/%10")
                      .arg(_pppMinEleSpin->value(), 0, 'f', 1)
                      .arg(opt.minObs)
                      .arg(opt.sigmaC1, 0, 'f', 3)
                      .arg(opt.sigmaL1, 0, 'f', 4)
                      .arg(opt.useGps ? "G" : "")
                      .arg(opt.useGlo ? "R" : "")
                      .arg(opt.useGal ? "E" : "")
                      .arg(opt.useBds ? "C" : "")
                      .arg(enabledStationCount)
                      .arg(_rows.size()));
        appendLog("PPP",
                  QString("PPP models: CRD=%1 match=%2/%3 ANTEX=%4 file=%5 ant=%6 ref=%7 ocean=%8 blq=%9 station=%10")
                      .arg(opt.crdFile.isEmpty() ? "-" : opt.crdFile)
                      .arg(crdMatchedStation.isEmpty() ? "-" : crdMatchedStation)
                      .arg(crdMatchedCount)
                      .arg(opt.useAntex ? "on" : "off")
                      .arg(opt.antexFile.isEmpty() ? "-" : opt.antexFile)
                      .arg(opt.roverAntenna.isEmpty() ? "-" : opt.roverAntenna)
                      .arg(opt.isApc ? "APC" : "CoM")
                      .arg(opt.useOceanTide ? "on" : "off")
                      .arg(opt.blqFile.isEmpty() ? "-" : opt.blqFile)
                      .arg(opt.oceanStation.isEmpty() ? "-" : opt.oceanStation));
        if (!opt.crdFile.isEmpty() && crdMatchedStation.isEmpty()) {
            appendLog("PPP", "CRD file is set but no station matched current rover mountpoint/station id; manual apriori and antenna fields are used.");
        }
        if (!opt.vtecFile.isEmpty()) {
            appendLog("PPP", QString("PPP VTEC corr file: %1").arg(opt.vtecFile));
        }
        appendLog("PPP",
                  QString("PPP observation model: compatible LC processing, GIM=%1, PPP-AR=%2.")
                      .arg(opt.pseudoObsIono ? "on" : "off")
                      .arg(opt.enablePppAr ? "on" : "off"));
        appendLog("PPP",
                  QString("PPP linear combination: G=%1 R=%2 E=%3 C=%4 priorities=%5")
                      .arg(opt.lcGps,
                           opt.lcGlo,
                           opt.lcGal,
                           opt.lcBds,
                           opt.signalPriorities));
        appendLog("PPP",
                   QString("PPP-AR thresholds: enable=%1 systems=G:%2/R:%3/E:%4/C:%5 ele>=%6 mwN>=%7 dd>=%8 WLdev<=%9 WLsig<=%10 ratio>=%11 boot>=%12 NLdev<=%13 sigma=%14 hist=%15 fix=%16 lock=%17 warm=%18 relax=%19")
                      .arg(opt.enablePppAr ? "on" : "off")
                      .arg(opt.arUseGps ? "on" : "off")
                      .arg(opt.arUseGlonass ? "on" : "off")
                      .arg(opt.arUseGalileo ? "on" : "off")
                      .arg(opt.arUseBds ? "on" : "off")
                      .arg(opt.arMinEleDeg, 0, 'f', 1)
                      .arg(opt.arMinMwSamples)
                      .arg(opt.arMinFixPairs)
                      .arg(opt.arWlMaxDev, 0, 'f', 4)
                      .arg(opt.arWlMaxSig, 0, 'f', 4)
                      .arg(opt.arNlRatio, 0, 'f', 3)
                      .arg(opt.arNlBoot, 0, 'f', 4)
                      .arg(opt.arNlMaxDev, 0, 'f', 4)
                      .arg(opt.arConstraintSigma, 0, 'f', 4)
                      .arg(opt.arUseHistoryAid ? "on" : "off")
                      .arg(opt.arHistMinFixEpochs)
                      .arg(opt.arHistMinLockEpochs)
                       .arg(opt.arPartialWarmupLockEpochs)
                       .arg(opt.arHistRelaxFactor, 0, 'f', 3));
        appendLog("PPP",
                  QString("PPP-AR algorithm=%1 BIE epochs>=%2 ambiguities>=%3 frac<=%4 sigma<=%5 constraintSigma=%6")
                      .arg(opt.arAlgorithm)
                      .arg(opt.arBieMinEpochs)
                      .arg(opt.arBieMinSat)
                      .arg(opt.arBieMaxFrac, 0, 'g', 8)
                      .arg(opt.arBieMaxSig, 0, 'g', 8)
                      .arg(opt.arBieConstraintSigma, 0, 'g', 8));
        appendLog("PPP",
                  QString("PPP tropo strategy: ZHD Saastamoinen + full ZWD state, prior=%1 method=%2 mapping=%3 cache=%4 outputDir=%5")
                      .arg(opt.troposphereMode == PppTroposphereMode::GfsZwdInitialization ? "GFS" : "Original")
                      .arg(opt.gfsZwdMethod == GfsZwdMethod::ProfileIntegral ? "profile-integral" : "Askne-Nordius-diagnostic")
                      .arg(opt.mappingMode == PppMappingMode::Gpt3Vmf3 ? "GPT3+VMF3" : "1/sin(E)")
                      .arg(opt.gfsCacheDirectory.isEmpty() ? "-" : opt.gfsCacheDirectory)
                      .arg(_ztdOutputDirectory.isEmpty() ? "-" : _ztdOutputDirectory));
        appendLog("PPP",
                  QString("PPP compact JSON: enabled=%1 outputDir=%2 station/day=file reorderBuffer=2s")
                      .arg(_pppJsonEnabled ? "on" : "off")
                      .arg(_pppJsonOutputDirectory));
        appendLog("PPP",
                  QString("PPP source policy: strict=%1 queueMax=%2 corrWait=%3s dropLag=%4s")
                      .arg(opt.strictCorrectionSource ? "on" : "off")
                      .arg(_pppRuntimePolicy.maxQueueSize)
                      .arg(_pppRuntimePolicy.corrWaitTimeSec)
                      .arg(_pppRuntimePolicy.dropLagSec));
        if (!opt.arLogFile.isEmpty()) {
            appendLog("PPP", QString("PPP-AR log file: %1").arg(opt.arLogFile));
        }
        if (opt.useAntex && (opt.antexFile.isEmpty() || opt.roverAntenna.isEmpty())) {
            appendLog("PPP", "ANTEX enabled but file or receiver antenna name is empty: antenna correction may be skipped.");
        }
        if (opt.useOceanTide && opt.blqFile.isEmpty()) {
            appendLog("PPP", "Ocean tide enabled but BLQ file is empty: ocean correction is skipped (optional input).");
        }
    }
}

bool MainWindow::savePppSettingsToSettings(QString* error) {
    QSettings s(settingsFilePath(), QSettings::IniFormat);
    const PppOptions opt = _pppOptions;
    s.setValue("ppp/useGps", opt.useGps);
    s.setValue("ppp/useGlo", opt.useGlo);
    s.setValue("ppp/useGal", opt.useGal);
    s.setValue("ppp/useBds", opt.useBds);
    s.setValue("ppp/strictCorrectionSource", opt.strictCorrectionSource);
    s.setValue("ppp/signalPriorities", opt.signalPriorities);
    s.setValue("ppp/lcGps", opt.lcGps);
    s.setValue("ppp/lcGlo", opt.lcGlo);
    s.setValue("ppp/lcGal", opt.lcGal);
    s.setValue("ppp/lcBds", opt.lcBds);
    s.setValue("ppp/pseudoObsIono", opt.pseudoObsIono);
    s.setValue("ppp/sigmaC1", opt.sigmaC1);
    s.setValue("ppp/sigmaL1", opt.sigmaL1);
    s.setValue("ppp/maxResC1", opt.maxResC1);
    s.setValue("ppp/maxResL1", opt.maxResL1);
    s.setValue("ppp/minEleDeg", opt.minEleRad * 180.0 / kPi);
    s.setValue("ppp/minObs", opt.minObs);
    s.setValue("ppp/eleWgtCode", opt.eleWgtCode);
    s.setValue("ppp/eleWgtPhase", opt.eleWgtPhase);
    s.setValue("ppp/aprSigX", opt.aprSigCrd[0]);
    s.setValue("ppp/aprSigY", opt.aprSigCrd[1]);
    s.setValue("ppp/aprSigZ", opt.aprSigCrd[2]);
    s.setValue("ppp/noiseX", opt.noiseCrd[0]);
    s.setValue("ppp/noiseY", opt.noiseCrd[1]);
    s.setValue("ppp/noiseZ", opt.noiseCrd[2]);
    s.setValue("ppp/aprSigClk", opt.aprSigClk);
    s.setValue("ppp/aprSigTrp", opt.aprSigTrp);
    s.setValue("ppp/noiseTrp", opt.noiseTrp);
    s.setValue("ppp/aprSigAmb", opt.aprSigAmb);
    s.setValue("ppp/aprSigIon", opt.aprSigIon);
    s.setValue("ppp/noiseIon", opt.noiseIon);
    s.setValue("ppp/aprSigCodeBias", opt.aprSigCodeBias);
    s.setValue("ppp/noiseCodeBias", opt.noiseCodeBias);
    s.setValue("ppp/aprSigPhaseBias", opt.aprSigPhaseBias);
    s.setValue("ppp/noisePhaseBias", opt.noisePhaseBias);
    s.setValue("ppp/seedingTime", opt.seedingTime);
    s.setValue("ppp/aprX", opt.aprXyz[0]);
    s.setValue("ppp/aprY", opt.aprXyz[1]);
    s.setValue("ppp/aprZ", opt.aprXyz[2]);
    s.setValue("ppp/neuN", opt.neuEcc[0]);
    s.setValue("ppp/neuE", opt.neuEcc[1]);
    s.setValue("ppp/neuU", opt.neuEcc[2]);
    s.setValue("ppp/crdFile", opt.crdFile);
    s.setValue("ppp/useAntex", opt.useAntex);
    s.setValue("ppp/antexFile", opt.antexFile);
    s.setValue("ppp/roverAntenna", opt.roverAntenna);
    s.setValue("ppp/isApc", opt.isApc);
    s.setValue("ppp/useOceanTide", opt.useOceanTide);
    s.setValue("ppp/blqFile", opt.blqFile);
    s.setValue("ppp/oceanStation", opt.oceanStation);
    s.setValue("ppp/vtecFile", opt.vtecFile);
    s.setValue("ppp/logSsrYawDetails", opt.logSsrYawDetails);
    s.setValue("ppp/logBiasMatchDetails", opt.logBiasMatchDetails);
    s.setValue("ppp/estimateTroposphere", opt.estimateTroposphere);
    s.setValue("ppp/troposphereMode", opt.troposphereMode == PppTroposphereMode::GfsZwdInitialization ? "gfs" : "original");
    s.setValue("ppp/gfsZwdMethod", opt.gfsZwdMethod == GfsZwdMethod::AskneNordius ? "askne" : "profile");
    s.setValue("ppp/mappingMode", opt.mappingMode == PppMappingMode::Gpt3Vmf3 ? "gpt3" : "simple");
    s.setValue("ppp/gfsCacheDirectory", opt.gfsCacheDirectory);
    s.setValue("ppp/gfsDownloadBaseUrl", opt.gfsDownloadBaseUrl);
    s.setValue("ppp/gfsPrefetchTimeoutSec", opt.gfsPrefetchTimeoutSec);
    s.setValue("ppp/gfsMaxAgeSec", opt.gfsMaxAgeSec);
    s.setValue("ppp/estimateIonosphere", opt.estimateIonosphere);
    s.setValue("ppp/pppRtk", opt.pppRtk);
    s.setValue("ppp/sigmaGim", opt.sigmaGIM);
    s.setValue("ppp/maxResGim", opt.maxResGIM);
    s.setValue("ppp/arEnable", opt.enablePppAr);
    s.setValue("ppp/arAlgorithm", opt.arAlgorithm);
    s.setValue("ppp/arUseGps", opt.arUseGps);
    s.setValue("ppp/arUseGlonass", opt.arUseGlonass);
    s.setValue("ppp/arUseGalileo", opt.arUseGalileo);
    s.setValue("ppp/arUseBds", opt.arUseBds);
    s.setValue("ppp/arMinEleDeg", opt.arMinEleDeg);
    s.setValue("ppp/arMinMwSamples", opt.arMinMwSamples);
    s.setValue("ppp/arMinFixPairs", opt.arMinFixPairs);
    s.setValue("ppp/arWlMaxDev", opt.arWlMaxDev);
    s.setValue("ppp/arWlMaxSig", opt.arWlMaxSig);
    s.setValue("ppp/arNlRatio", opt.arNlRatio);
    s.setValue("ppp/arNlBoot", opt.arNlBoot);
    s.setValue("ppp/arNlMaxDev", opt.arNlMaxDev);
    s.setValue("ppp/arConstraintSigma", opt.arConstraintSigma);
    s.setValue("ppp/arBieMinEpochs", opt.arBieMinEpochs);
    s.setValue("ppp/arBieMinSat", opt.arBieMinSat);
    s.setValue("ppp/arBieMaxFrac", opt.arBieMaxFrac);
    s.setValue("ppp/arBieMaxSig", opt.arBieMaxSig);
    s.setValue("ppp/arBieConstraintSigma", opt.arBieConstraintSigma);
    s.setValue("ppp/arBiePersistConstraints", opt.arBiePersistConstraints);
    s.setValue("ppp/arBiePersistConfirmEpochs", opt.arBiePersistConfirmEpochs);
    s.setValue("ppp/arBiePersistMaxVerticalM", opt.arBiePersistMaxVerticalM);
    s.setValue("ppp/arBiePersistMaxZtdM", opt.arBiePersistMaxZtdM);
    s.setValue("ppp/arUseHistoryAid", opt.arUseHistoryAid);
    s.setValue("ppp/arHistMinFixEpochs", opt.arHistMinFixEpochs);
    s.setValue("ppp/arHistMinLockEpochs", opt.arHistMinLockEpochs);
    s.setValue("ppp/arPartialWarmupLockEpochs", opt.arPartialWarmupLockEpochs);
    s.setValue("ppp/arHistRelaxFactor", opt.arHistRelaxFactor);
    s.setValue("ppp/arLogFile", opt.arLogFile);
    s.setValue("ppp/ztdOutputDir", _ztdOutputDirectory);
    s.setValue("ppp/jsonEnabled", _pppJsonEnabled);
    s.setValue("ppp/jsonOutputDir", _pppJsonOutputDirectory);
    s.setValue("ppp/jsonReorderBufferMs", _pppJsonReorderBufferMs);
    s.setValue("ppp/jsonFlushIntervalMs", _pppJsonFlushIntervalMs);
    s.setValue("ppp/queueMax", _pppRuntimePolicy.maxQueueSize);
    s.setValue("ppp/corrWaitSec", _pppRuntimePolicy.corrWaitTimeSec);
    s.setValue("ppp/dropLagSec", _pppRuntimePolicy.dropLagSec);
    if (_postModeCombo) s.setValue("ppp/processingMode", _postModeCombo->currentData().toString());
    if (_postJobNameEdit) s.setValue("ppp/postJobName", _postJobNameEdit->text().trimmed());
    if (_postObsPathEdit) s.setValue("ppp/postObsPath", _postObsPathEdit->text().trimmed());
    if (_postNavPathEdit) s.setValue("ppp/postNavPath", _postNavPathEdit->text().trimmed());
    if (_postSp3PathEdit) s.setValue("ppp/postSp3Path", _postSp3PathEdit->text().trimmed());
    if (_postClkPathEdit) s.setValue("ppp/postClockPath", _postClkPathEdit->text().trimmed());
    if (_postSsrPathsEdit) s.setValue("ppp/postSsrPaths", _postSsrPathsEdit->text().trimmed());
    if (_postProductModeCombo) s.setValue("ppp/postProductMode", _postProductModeCombo->currentData().toString());
    if (_postSampleCombo) s.setValue("ppp/postSampleSec", _postSampleCombo->currentData().toInt());
    QStringList enabledStations;
    QStringList disabledStations;
    for (auto it = _pppStationEnabled.constBegin(); it != _pppStationEnabled.constEnd(); ++it) {
        if (it.value()) {
            enabledStations << it.key();
        } else {
            disabledStations << it.key();
        }
    }
    s.setValue("ppp/enabledStations", enabledStations);
    s.setValue("ppp/disabledStations", disabledStations);
    s.sync();
    if (s.status() != QSettings::NoError) {
        const QString message = QStringLiteral("cannot write settings %1 (status=%2)")
                                    .arg(QFileInfo(settingsFilePath()).absoluteFilePath())
                                    .arg(static_cast<int>(s.status()));
        if (error) *error = message;
        else qWarning().noquote() << message;
        return false;
    }
    if (error) error->clear();
    return true;
}

void MainWindow::loadPppSettingsFromSettings() {
    QSettings s(settingsFilePath(), QSettings::IniFormat);
    qInfo().noquote() << QStringLiteral("PPP settings load: file=%1 abs=%2 status=%3")
                         .arg(settingsFilePath())
                         .arg(QFileInfo(settingsFilePath()).absoluteFilePath())
                         .arg(s.status() == QSettings::NoError ? QStringLiteral("ok") : QStringLiteral("error"));
    PppOptions opt;
    opt.useGps = s.value("ppp/useGps", opt.useGps).toBool();
    opt.useGlo = s.value("ppp/useGlo", opt.useGlo).toBool();
    opt.useGal = s.value("ppp/useGal", opt.useGal).toBool();
    opt.useBds = s.value("ppp/useBds", opt.useBds).toBool();
    opt.strictCorrectionSource = s.value("ppp/strictCorrectionSource", opt.strictCorrectionSource).toBool();
    auto normalizeLc = [](const QString& value, const QString& fallback) {
        const QString v = value.trimmed();
        if (v == QLatin1String("L1L2")) {
            return QStringLiteral("Pi&Li");
        }
        const QStringList modes = QStringList()
                                  << "P3&L3"
                                  << "P3"
                                  << "L3"
                                  << "Pi&Li"
                                  << "Pi"
                                  << "P1&L1"
                                  << "P1";
        return modes.contains(v) ? v : fallback;
    };
    opt.signalPriorities = s.value("ppp/signalPriorities", opt.signalPriorities).toString().trimmed();
    if (opt.signalPriorities.isEmpty()) {
        opt.signalPriorities = QStringLiteral("G:12&CWPSLX R:12&CP E:1&CBX E:5&QIX C:26&IQX");
    }
    opt.lcGps = normalizeLc(s.value("ppp/lcGps", opt.lcGps).toString(), QStringLiteral("P3&L3"));
    opt.lcGlo = normalizeLc(s.value("ppp/lcGlo", opt.lcGlo).toString(), QStringLiteral("P3&L3"));
    opt.lcGal = normalizeLc(s.value("ppp/lcGal", opt.lcGal).toString(), QStringLiteral("P3&L3"));
    opt.lcBds = normalizeLc(s.value("ppp/lcBds", opt.lcBds).toString(), QStringLiteral("P3&L3"));
    opt.pseudoObsIono = s.value("ppp/pseudoObsIono", opt.pseudoObsIono).toBool();
    opt.sigmaC1 = s.value("ppp/sigmaC1", opt.sigmaC1).toDouble();
    opt.sigmaL1 = s.value("ppp/sigmaL1", opt.sigmaL1).toDouble();
    opt.maxResC1 = s.value("ppp/maxResC1", opt.maxResC1).toDouble();
    opt.maxResL1 = s.value("ppp/maxResL1", opt.maxResL1).toDouble();
    const double minEleDeg = s.value("ppp/minEleDeg", opt.minEleRad * 180.0 / kPi).toDouble();
    opt.minEleRad = minEleDeg * kPi / 180.0;
    opt.minObs = s.value("ppp/minObs", opt.minObs).toInt();
    opt.eleWgtCode = s.value("ppp/eleWgtCode", opt.eleWgtCode).toBool();
    opt.eleWgtPhase = s.value("ppp/eleWgtPhase", opt.eleWgtPhase).toBool();
    opt.aprSigCrd[0] = s.value("ppp/aprSigX", opt.aprSigCrd[0]).toDouble();
    opt.aprSigCrd[1] = s.value("ppp/aprSigY", opt.aprSigCrd[1]).toDouble();
    opt.aprSigCrd[2] = s.value("ppp/aprSigZ", opt.aprSigCrd[2]).toDouble();
    opt.noiseCrd[0] = s.value("ppp/noiseX", opt.noiseCrd[0]).toDouble();
    opt.noiseCrd[1] = s.value("ppp/noiseY", opt.noiseCrd[1]).toDouble();
    opt.noiseCrd[2] = s.value("ppp/noiseZ", opt.noiseCrd[2]).toDouble();
    opt.aprSigClk = s.value("ppp/aprSigClk", opt.aprSigClk).toDouble();
    opt.aprSigTrp = s.value("ppp/aprSigTrp", opt.aprSigTrp).toDouble();
    opt.noiseTrp = s.value("ppp/noiseTrp", opt.noiseTrp).toDouble();
    opt.aprSigAmb = s.value("ppp/aprSigAmb", opt.aprSigAmb).toDouble();
    opt.aprSigIon = s.value("ppp/aprSigIon", opt.aprSigIon).toDouble();
    opt.noiseIon = s.value("ppp/noiseIon", opt.noiseIon).toDouble();
    opt.aprSigCodeBias = s.value("ppp/aprSigCodeBias", opt.aprSigCodeBias).toDouble();
    opt.noiseCodeBias = s.value("ppp/noiseCodeBias", opt.noiseCodeBias).toDouble();
    opt.aprSigPhaseBias = s.value("ppp/aprSigPhaseBias", opt.aprSigPhaseBias).toDouble();
    opt.noisePhaseBias = s.value("ppp/noisePhaseBias", opt.noisePhaseBias).toDouble();
    opt.seedingTime = s.value("ppp/seedingTime", opt.seedingTime).toDouble();
    opt.aprXyz[0] = s.value("ppp/aprX", opt.aprXyz[0]).toDouble();
    opt.aprXyz[1] = s.value("ppp/aprY", opt.aprXyz[1]).toDouble();
    opt.aprXyz[2] = s.value("ppp/aprZ", opt.aprXyz[2]).toDouble();
    opt.neuEcc[0] = s.value("ppp/neuN", opt.neuEcc[0]).toDouble();
    opt.neuEcc[1] = s.value("ppp/neuE", opt.neuEcc[1]).toDouble();
    opt.neuEcc[2] = s.value("ppp/neuU", opt.neuEcc[2]).toDouble();
    opt.crdFile = s.value("ppp/crdFile", opt.crdFile).toString().trimmed();
    opt.useAntex = s.value("ppp/useAntex", opt.useAntex).toBool();
    opt.antexFile = s.value("ppp/antexFile", opt.antexFile).toString();
    opt.roverAntenna = s.value("ppp/roverAntenna", opt.roverAntenna).toString().trimmed().toUpper();
    opt.isApc = s.value("ppp/isApc", opt.isApc).toBool();
    opt.useOceanTide = s.value("ppp/useOceanTide", opt.useOceanTide).toBool();
    opt.blqFile = s.value("ppp/blqFile", opt.blqFile).toString();
    opt.oceanStation = s.value("ppp/oceanStation", opt.oceanStation).toString().trimmed().toUpper();
    opt.vtecFile = s.value("ppp/vtecFile", opt.vtecFile).toString().trimmed();
    opt.logSsrYawDetails = s.value("ppp/logSsrYawDetails", opt.logSsrYawDetails).toBool();
    opt.logBiasMatchDetails = s.value("ppp/logBiasMatchDetails", opt.logBiasMatchDetails).toBool();
    opt.estimateTroposphere = s.value("ppp/estimateTroposphere", opt.estimateTroposphere).toBool();
    const QString tropoMode = s.value("ppp/troposphereMode", QStringLiteral("original")).toString().trimmed().toLower();
    opt.troposphereMode = tropoMode == QLatin1String("gfs")
        ? PppTroposphereMode::GfsZwdInitialization : PppTroposphereMode::Original;
    const QString gfsMethod = s.value("ppp/gfsZwdMethod", QStringLiteral("profile")).toString().trimmed().toLower();
    opt.gfsZwdMethod = gfsMethod == QLatin1String("askne")
        ? GfsZwdMethod::AskneNordius : GfsZwdMethod::ProfileIntegral;
    const QString mappingMode = s.value("ppp/mappingMode", QStringLiteral("simple")).toString().trimmed().toLower();
    opt.mappingMode = mappingMode == QLatin1String("gpt3")
        ? PppMappingMode::Gpt3Vmf3 : PppMappingMode::SimpleSin;
    opt.gfsCacheDirectory = s.value("ppp/gfsCacheDirectory", opt.gfsCacheDirectory).toString().trimmed();
    opt.gfsDownloadBaseUrl = s.value("ppp/gfsDownloadBaseUrl", opt.gfsDownloadBaseUrl).toString().trimmed();
    opt.gfsPrefetchTimeoutSec = qBound(30, s.value("ppp/gfsPrefetchTimeoutSec", opt.gfsPrefetchTimeoutSec).toInt(), 300);
    opt.gfsMaxAgeSec = qMax(0, s.value("ppp/gfsMaxAgeSec", opt.gfsMaxAgeSec).toInt());
    opt.estimateIonosphere = s.value("ppp/estimateIonosphere", opt.estimateIonosphere).toBool();
    opt.pppRtk = s.value("ppp/pppRtk", opt.pppRtk).toBool();
    opt.sigmaGIM = qMax(1.0e-6, s.value("ppp/sigmaGim", opt.sigmaGIM).toDouble());
    opt.maxResGIM = qMax(1.0e-6, s.value("ppp/maxResGim", opt.maxResGIM).toDouble());
    opt.enablePppAr = s.value("ppp/arEnable", opt.enablePppAr).toBool();
    opt.arAlgorithm = SettingsStore::normalizedArAlgorithm(
        s.value("ppp/arAlgorithm", opt.arAlgorithm).toString());
    opt.arUseGps = s.value("ppp/arUseGps", opt.arUseGps).toBool();
    opt.arUseGlonass = s.value("ppp/arUseGlonass", opt.arUseGlonass).toBool();
    opt.arUseGalileo = s.value("ppp/arUseGalileo", opt.arUseGalileo).toBool();
    opt.arUseBds = s.value("ppp/arUseBds", opt.arUseBds).toBool();
    opt.arMinEleDeg = s.value("ppp/arMinEleDeg", opt.arMinEleDeg).toDouble();
    opt.arMinMwSamples = s.value("ppp/arMinMwSamples", opt.arMinMwSamples).toInt();
    opt.arMinFixPairs = s.value("ppp/arMinFixPairs", opt.arMinFixPairs).toInt();
    opt.arWlMaxDev = s.value("ppp/arWlMaxDev", opt.arWlMaxDev).toDouble();
    opt.arWlMaxSig = s.value("ppp/arWlMaxSig", opt.arWlMaxSig).toDouble();
    opt.arNlRatio = s.value("ppp/arNlRatio", opt.arNlRatio).toDouble();
    opt.arNlBoot = s.value("ppp/arNlBoot", opt.arNlBoot).toDouble();
    opt.arNlMaxDev = s.value("ppp/arNlMaxDev", opt.arNlMaxDev).toDouble();
    opt.arConstraintSigma = s.value("ppp/arConstraintSigma", opt.arConstraintSigma).toDouble();
    opt.arBieMinEpochs = qMax(1, s.value("ppp/arBieMinEpochs", opt.arBieMinEpochs).toInt());
    opt.arBieMinSat = qMax(2, s.value("ppp/arBieMinSat", opt.arBieMinSat).toInt());
    opt.arBieMaxFrac = s.value("ppp/arBieMaxFrac", opt.arBieMaxFrac).toDouble();
    opt.arBieMaxSig = s.value("ppp/arBieMaxSig", opt.arBieMaxSig).toDouble();
    opt.arBieConstraintSigma = s.value("ppp/arBieConstraintSigma", opt.arBieConstraintSigma).toDouble();
    opt.arBiePersistConstraints = s.value("ppp/arBiePersistConstraints", opt.arBiePersistConstraints).toBool();
    opt.arBiePersistConfirmEpochs = qMax(1, s.value("ppp/arBiePersistConfirmEpochs", opt.arBiePersistConfirmEpochs).toInt());
    opt.arBiePersistMaxVerticalM = s.value("ppp/arBiePersistMaxVerticalM", opt.arBiePersistMaxVerticalM).toDouble();
    opt.arBiePersistMaxZtdM = s.value("ppp/arBiePersistMaxZtdM", opt.arBiePersistMaxZtdM).toDouble();
    opt.arUseHistoryAid = s.value("ppp/arUseHistoryAid", opt.arUseHistoryAid).toBool();
    opt.arHistMinFixEpochs = s.value("ppp/arHistMinFixEpochs", opt.arHistMinFixEpochs).toInt();
    opt.arHistMinLockEpochs = s.value("ppp/arHistMinLockEpochs", opt.arHistMinLockEpochs).toInt();
    opt.arPartialWarmupLockEpochs = s.value("ppp/arPartialWarmupLockEpochs", opt.arPartialWarmupLockEpochs).toInt();
    opt.arHistRelaxFactor = s.value("ppp/arHistRelaxFactor", opt.arHistRelaxFactor).toDouble();
    opt.arLogFile = s.value("ppp/arLogFile", opt.arLogFile).toString().trimmed();
    _ztdOutputDirectory = s.value("ppp/ztdOutputDir", QStringLiteral("output/ztd")).toString().trimmed();
    if (_ztdOutputDirectory.isEmpty()) {
        _ztdOutputDirectory = "output/ztd";
    }
    _pppJsonEnabled = s.value("ppp/jsonEnabled", true).toBool();
    _pppJsonOutputDirectory = s.value("ppp/jsonOutputDir", QStringLiteral("output/ppp_json")).toString().trimmed();
    if (_pppJsonOutputDirectory.isEmpty()) {
        _pppJsonOutputDirectory = QStringLiteral("output/ppp_json");
    }
    _pppJsonReorderBufferMs = qMax(0, s.value("ppp/jsonReorderBufferMs", 2000).toInt());
    _pppJsonFlushIntervalMs = qMax(0, s.value("ppp/jsonFlushIntervalMs", 1000).toInt());
    _pppRuntimePolicy = PppWorker::RuntimePolicy();
    _pppRuntimePolicy.epochIntervalSec = _prepConfig.observationSampleSec;
    _pppRuntimePolicy.maxQueueSize = qMax(1, s.value("ppp/queueMax", _pppRuntimePolicy.maxQueueSize).toInt());
    _pppRuntimePolicy.corrWaitTimeSec = qMax(0, s.value("ppp/corrWaitSec", _pppRuntimePolicy.corrWaitTimeSec).toInt());
    _pppRuntimePolicy.dropLagSec = qMax(1, s.value("ppp/dropLagSec", _pppRuntimePolicy.dropLagSec).toInt());
    const QString postMode = s.value("ppp/processingMode", QStringLiteral("realtime")).toString();
    const QString postJobName = s.value("ppp/postJobName").toString();
    const QString postObsPath = s.value("ppp/postObsPath").toString();
    const QString postNavPath = s.value("ppp/postNavPath").toString();
    const QString postSp3Path = s.value("ppp/postSp3Path").toString();
    const QString postClockPath = s.value("ppp/postClockPath").toString();
    const QString postSsrPaths = s.value("ppp/postSsrPaths").toString();
    const QString postProductMode = s.value("ppp/postProductMode", QStringLiteral("broadcast")).toString();
    const int postSampleSec = s.value("ppp/postSampleSec", 5).toInt();
    qInfo().noquote() << QStringLiteral("PPP settings loaded: processingMode=%1 tropoMode=%2 gfsMethod=%3 mapping=%4 gfsCache=%5 postSampleSec=%6")
                         .arg(postMode)
                         .arg(tropoMode)
                         .arg(gfsMethod)
                         .arg(mappingMode)
                         .arg(opt.gfsCacheDirectory)
                         .arg(postSampleSec);
    const QStringList enabledStations = s.value("ppp/enabledStations").toStringList();
    const QStringList disabledStations = s.value("ppp/disabledStations").toStringList();
    _pppStationEnabled.clear();
    for (const QString& id : enabledStations) {
        if (!id.trimmed().isEmpty()) {
            _pppStationEnabled[id] = true;
        }
    }
    for (const QString& id : disabledStations) {
        if (!id.trimmed().isEmpty()) {
            _pppStationEnabled[id] = false;
        }
    }

    _pppOptions = opt;
    postPppOptions();
    postPppRuntimePolicy();

    _loadingPppSettingsUi = true;
    if (_pppUseGpsCheck) _pppUseGpsCheck->setChecked(opt.useGps);
    if (_pppUseGloCheck) _pppUseGloCheck->setChecked(opt.useGlo);
    if (_pppUseGalCheck) _pppUseGalCheck->setChecked(opt.useGal);
    if (_pppUseBdsCheck) _pppUseBdsCheck->setChecked(opt.useBds);
    if (_pppSignalPrioritiesEdit) _pppSignalPrioritiesEdit->setText(opt.signalPriorities);
    if (_pppLcGpsCombo) _pppLcGpsCombo->setCurrentText(opt.lcGps);
    if (_pppLcGloCombo) _pppLcGloCombo->setCurrentText(opt.lcGlo);
    if (_pppLcGalCombo) _pppLcGalCombo->setCurrentText(opt.lcGal);
    if (_pppLcBdsCombo) _pppLcBdsCombo->setCurrentText(opt.lcBds);
    if (_pppSigmaC1Spin) _pppSigmaC1Spin->setValue(opt.sigmaC1);
    if (_pppSigmaL1Spin) _pppSigmaL1Spin->setValue(opt.sigmaL1);
    if (_pppMaxResC1Spin) _pppMaxResC1Spin->setValue(opt.maxResC1);
    if (_pppMaxResL1Spin) _pppMaxResL1Spin->setValue(opt.maxResL1);
    if (_pppMinEleSpin) _pppMinEleSpin->setValue(minEleDeg);
    if (_pppMinObsSpin) _pppMinObsSpin->setValue(opt.minObs);
    if (_pppEleWgtCodeCheck) _pppEleWgtCodeCheck->setChecked(opt.eleWgtCode);
    if (_pppEleWgtPhaseCheck) _pppEleWgtPhaseCheck->setChecked(opt.eleWgtPhase);
    if (_pppPseudoObsIonoCheck) _pppPseudoObsIonoCheck->setChecked(opt.pseudoObsIono);
    if (_pppAprSigXSpin) _pppAprSigXSpin->setValue(opt.aprSigCrd[0]);
    if (_pppAprSigYSpin) _pppAprSigYSpin->setValue(opt.aprSigCrd[1]);
    if (_pppAprSigZSpin) _pppAprSigZSpin->setValue(opt.aprSigCrd[2]);
    if (_pppNoiseXSpin) _pppNoiseXSpin->setValue(opt.noiseCrd[0]);
    if (_pppNoiseYSpin) _pppNoiseYSpin->setValue(opt.noiseCrd[1]);
    if (_pppNoiseZSpin) _pppNoiseZSpin->setValue(opt.noiseCrd[2]);
    if (_pppAprSigClkSpin) _pppAprSigClkSpin->setValue(opt.aprSigClk);
    if (_pppAprSigTrpSpin) _pppAprSigTrpSpin->setValue(opt.aprSigTrp);
    if (_pppNoiseTrpSpin) _pppNoiseTrpSpin->setValue(opt.noiseTrp);
    if (_pppAprSigAmbSpin) _pppAprSigAmbSpin->setValue(opt.aprSigAmb);
    if (_pppSeedingSpin) _pppSeedingSpin->setValue(opt.seedingTime);
    if (_pppTropoModeCombo) {
        const int index = _pppTropoModeCombo->findData(
            opt.troposphereMode == PppTroposphereMode::GfsZwdInitialization ? QStringLiteral("gfs") : QStringLiteral("original"));
        _pppTropoModeCombo->setCurrentIndex(index >= 0 ? index : 0);
    }
    if (_pppGfsMethodCombo) {
        const int index = _pppGfsMethodCombo->findData(
            opt.gfsZwdMethod == GfsZwdMethod::AskneNordius ? QStringLiteral("askne") : QStringLiteral("profile"));
        _pppGfsMethodCombo->setCurrentIndex(index >= 0 ? index : 0);
    }
    if (_pppMappingModeCombo) {
        const int index = _pppMappingModeCombo->findData(
            opt.mappingMode == PppMappingMode::Gpt3Vmf3 ? QStringLiteral("gpt3") : QStringLiteral("simple"));
        _pppMappingModeCombo->setCurrentIndex(index >= 0 ? index : 0);
    }
    if (_pppGfsCachePathEdit) _pppGfsCachePathEdit->setText(opt.gfsCacheDirectory);
    if (_pppGfsTimeoutSpin) _pppGfsTimeoutSpin->setValue(opt.gfsPrefetchTimeoutSec);
    if (_pppAprXSpin) _pppAprXSpin->setValue(opt.aprXyz[0]);
    if (_pppAprYSpin) _pppAprYSpin->setValue(opt.aprXyz[1]);
    if (_pppAprZSpin) _pppAprZSpin->setValue(opt.aprXyz[2]);
    if (_pppNeuNSpin) _pppNeuNSpin->setValue(opt.neuEcc[0]);
    if (_pppNeuESpin) _pppNeuESpin->setValue(opt.neuEcc[1]);
    if (_pppNeuUSpin) _pppNeuUSpin->setValue(opt.neuEcc[2]);
    if (_pppCrdPathEdit) _pppCrdPathEdit->setText(opt.crdFile);
    if (_pppUseAntexCheck) _pppUseAntexCheck->setChecked(opt.useAntex);
    if (_pppAntexPathEdit) _pppAntexPathEdit->setText(opt.antexFile);
    if (_pppRoverAntennaEdit) _pppRoverAntennaEdit->setText(opt.roverAntenna);
    if (_pppUseApcCheck) _pppUseApcCheck->setChecked(opt.isApc);
    if (_pppUseOceanCheck) _pppUseOceanCheck->setChecked(opt.useOceanTide);
    if (_pppArEnableCheck) _pppArEnableCheck->setChecked(opt.enablePppAr);
    if (_pppArAlgorithmCombo) {
        const int arIndex = _pppArAlgorithmCombo->findData(opt.arAlgorithm);
        _pppArAlgorithmCombo->setCurrentIndex(arIndex >= 0 ? arIndex : 0);
    }
    if (_pppArUseGpsCheck) _pppArUseGpsCheck->setChecked(opt.arUseGps);
    if (_pppArUseGloCheck) _pppArUseGloCheck->setChecked(opt.arUseGlonass);
    if (_pppArUseGalCheck) _pppArUseGalCheck->setChecked(opt.arUseGalileo);
    if (_pppArUseBdsCheck) _pppArUseBdsCheck->setChecked(opt.arUseBds);
    if (_pppArMinEleSpin) _pppArMinEleSpin->setValue(opt.arMinEleDeg);
    if (_pppArMinMwSamplesSpin) _pppArMinMwSamplesSpin->setValue(opt.arMinMwSamples);
    if (_pppArMinFixPairsSpin) _pppArMinFixPairsSpin->setValue(opt.arMinFixPairs);
    if (_pppArWlMaxDevSpin) _pppArWlMaxDevSpin->setValue(opt.arWlMaxDev);
    if (_pppArWlMaxSigSpin) _pppArWlMaxSigSpin->setValue(opt.arWlMaxSig);
    if (_pppArNlRatioSpin) _pppArNlRatioSpin->setValue(opt.arNlRatio);
    if (_pppArNlBootSpin) _pppArNlBootSpin->setValue(opt.arNlBoot);
    if (_pppArNlMaxDevSpin) _pppArNlMaxDevSpin->setValue(opt.arNlMaxDev);
    if (_pppArConstraintSigmaSpin) _pppArConstraintSigmaSpin->setValue(opt.arConstraintSigma);
    if (_pppArBieMinEpochsSpin) _pppArBieMinEpochsSpin->setValue(opt.arBieMinEpochs);
    if (_pppArBieMinSatSpin) _pppArBieMinSatSpin->setValue(opt.arBieMinSat);
    if (_pppArBieMaxFracSpin) _pppArBieMaxFracSpin->setValue(opt.arBieMaxFrac);
    if (_pppArBieMaxSigSpin) _pppArBieMaxSigSpin->setValue(opt.arBieMaxSig);
    if (_pppArBieConstraintSigmaSpin) _pppArBieConstraintSigmaSpin->setValue(opt.arBieConstraintSigma);
    if (_pppArBiePersistConstraintsCheck) _pppArBiePersistConstraintsCheck->setChecked(opt.arBiePersistConstraints);
    if (_pppArBiePersistConfirmEpochsSpin) _pppArBiePersistConfirmEpochsSpin->setValue(opt.arBiePersistConfirmEpochs);
    if (_pppArBiePersistMaxVerticalSpin) _pppArBiePersistMaxVerticalSpin->setValue(opt.arBiePersistMaxVerticalM);
    if (_pppArBiePersistMaxZtdSpin) _pppArBiePersistMaxZtdSpin->setValue(opt.arBiePersistMaxZtdM);
    if (_pppArUseHistoryAidCheck) _pppArUseHistoryAidCheck->setChecked(opt.arUseHistoryAid);
    if (_pppArHistMinFixEpochsSpin) _pppArHistMinFixEpochsSpin->setValue(opt.arHistMinFixEpochs);
    if (_pppArHistMinLockEpochsSpin) _pppArHistMinLockEpochsSpin->setValue(opt.arHistMinLockEpochs);
    if (_pppArPartialWarmupLockEpochsSpin) _pppArPartialWarmupLockEpochsSpin->setValue(opt.arPartialWarmupLockEpochs);
    if (_pppArHistRelaxFactorSpin) _pppArHistRelaxFactorSpin->setValue(opt.arHistRelaxFactor);
    if (_pppBlqPathEdit) _pppBlqPathEdit->setText(opt.blqFile);
    if (_pppOceanStationEdit) _pppOceanStationEdit->setText(opt.oceanStation);
    if (_pppVtecPathEdit) _pppVtecPathEdit->setText(opt.vtecFile);
    if (_pppArLogPathEdit) _pppArLogPathEdit->setText(opt.arLogFile);
    if (_pppZtdPathEdit) _pppZtdPathEdit->setText(_ztdOutputDirectory);
    if (_pppJsonEnableCheck) _pppJsonEnableCheck->setChecked(_pppJsonEnabled);
    if (_pppJsonPathEdit) _pppJsonPathEdit->setText(_pppJsonOutputDirectory);
    if (_postModeCombo) {
        const int index = _postModeCombo->findData(postMode == QLatin1String("post") ? postMode : QStringLiteral("realtime"));
        _postModeCombo->setCurrentIndex(index >= 0 ? index : 0);
    }
    if (_postJobNameEdit) _postJobNameEdit->setText(postJobName);
    if (_postObsPathEdit) _postObsPathEdit->setText(postObsPath);
    if (_postNavPathEdit) _postNavPathEdit->setText(postNavPath);
    if (_postSp3PathEdit) _postSp3PathEdit->setText(postSp3Path);
    if (_postClkPathEdit) _postClkPathEdit->setText(postClockPath);
    if (_postSsrPathsEdit) _postSsrPathsEdit->setText(postSsrPaths);
    if (_postProductModeCombo) {
        const int index = _postProductModeCombo->findData(postProductMode);
        _postProductModeCombo->setCurrentIndex(index >= 0 ? index : 0);
    }
    if (_postSampleCombo) {
        const int index = _postSampleCombo->findData(postSampleSec);
        _postSampleCombo->setCurrentIndex(index >= 0 ? index : 1);
    }
    if (_postInputPanel && _postModeCombo) _postInputPanel->setVisible(_postModeCombo->currentData().toString() == QLatin1String("post"));
    applyPostProductModeUi();
    _loadingPppSettingsUi = false;
    updatePppArAvailability();
    rebuildPppStationTable();
}

void MainWindow::refreshPppStationSelector() {
    if (!_pppStationCombo) {
        return;
    }
    const QString prev = _pppStationCombo->currentData().toString();
    const QString prevZtd = _ztdStationCombo ? _ztdStationCombo->currentData().toString() : QString();
    _pppStationCombo->blockSignals(true);
    _pppStationCombo->clear();
    if (_ztdStationCombo) {
        _ztdStationCombo->blockSignals(true);
        _ztdStationCombo->clear();
    }
    for (auto it = _pppHistory.constBegin(); it != _pppHistory.constEnd(); ++it) {
        if (it.value().isEmpty()) {
            continue;
        }
        const QString source = it.key();
        const QString label = _pppStationNameBySource.value(source, streamLabelById(source));
        _pppStationCombo->addItem(label + " [" + source + "]", source);
        if (_ztdStationCombo) {
            _ztdStationCombo->addItem(label + " [" + source + "]", source);
        }
    }
    int idx = _pppStationCombo->findData(prev);
    if (idx < 0) {
        idx = _pppStationCombo->findData(_lastPppSource);
    }
    if (idx < 0 && _pppStationCombo->count() > 0) {
        idx = 0;
    }
    if (idx >= 0) {
        _pppStationCombo->setCurrentIndex(idx);
    }
    _pppStationCombo->blockSignals(false);

    if (_ztdStationCombo) {
        int ztdIdx = _ztdStationCombo->findData(prevZtd);
        if (ztdIdx < 0) {
            ztdIdx = _ztdStationCombo->findData(_lastPppSource);
        }
        if (ztdIdx < 0 && _ztdStationCombo->count() > 0) {
            ztdIdx = 0;
        }
        if (ztdIdx >= 0) {
            _ztdStationCombo->setCurrentIndex(ztdIdx);
        }
        _ztdStationCombo->blockSignals(false);
    }
}

void MainWindow::refreshPppNeuPlot() {
    if (!_pppNeuScene || !_pppResultTable || !_pppFixRateLabel || !_pppStationCombo) {
        return;
    }
    const QString source = _pppStationCombo->currentData().toString();
    const QVector<PppEpochRecord> history = _pppHistory.value(source);

    _pppNeuScene->clear();
    _pppResultTable->setRowCount(0);
    if (history.isEmpty()) {
        _pppFixRateLabel->setText("Fixed sats: -");
        _pppNeuScene->addText("No PPP history for selected station")->setPos(-80, -10);
        return;
    }

    int fixedCount = 0;
    int validCount = 0;
    double lastRatio = 0.0;
    for (const PppEpochRecord& rec : history) {
        if (rec.hasNeu) {
            ++validCount;
        }
        if (rec.fixed) {
            ++fixedCount;
        }
        if (rec.ratio > 0.0) {
            lastRatio = rec.ratio;
        }
    }
    const double fixRate = validCount > 0 ? (100.0 * fixedCount / static_cast<double>(validCount)) : 0.0;
    const PppEpochRecord& lastRecord = history.last();
    const QString ratioText = lastRatio > 0.0 ? QString::number(lastRatio, 'f', 2) : QStringLiteral("-");
    _pppFixRateLabel->setText(QString("Fixed sats: %1/%2  Epochs: %3/%4 (%5%)  Ratio: %6")
                                  .arg(lastRecord.fixedSatCount)
                                  .arg(lastRecord.numSat)
                                  .arg(fixedCount)
                                  .arg(validCount)
                                  .arg(fixRate, 0, 'f', 1)
                                  .arg(ratioText));

    const int n = history.size();
    const double left = 50.0;
    const double top = 18.0;
    const double width = 860.0;
    const double height = 270.0;
    _pppNeuScene->setSceneRect(0.0, 0.0, left + width + 20.0, top + height + 35.0);

    double maxAbs = 0.0;
    for (const PppEpochRecord& rec : history) {
        if (!rec.hasNeu) continue;
        maxAbs = qMax(maxAbs, std::fabs(rec.neu[0]));
        maxAbs = qMax(maxAbs, std::fabs(rec.neu[1]));
        maxAbs = qMax(maxAbs, std::fabs(rec.neu[2]));
    }
    if (maxAbs < 0.05) {
        maxAbs = 0.05;
    }
    const double yRange = std::ceil(maxAbs * 100.0) / 100.0;

    _pppNeuScene->addRect(left, top, width, height, QPen(QColor(185, 185, 185)));
    const double y0 = top + height * 0.5;
    _pppNeuScene->addLine(left, y0, left + width, y0, QPen(QColor(170, 170, 170), 1, Qt::DashLine));

    auto yToScene = [&](double v) {
        return y0 - (v / yRange) * (height * 0.5 * 0.92);
    };
    auto xToScene = [&](int i) {
        if (n <= 1) return left;
        return left + width * (static_cast<double>(i) / static_cast<double>(n - 1));
    };

    _pppNeuScene->addText(QString("+%1").arg(yRange, 0, 'f', 2))->setPos(8.0, top - 6.0);
    _pppNeuScene->addText("0")->setPos(24.0, y0 - 10.0);
    _pppNeuScene->addText(QString("-%1").arg(yRange, 0, 'f', 2))->setPos(8.0, top + height - 14.0);

    const QVector<QColor> colors = {QColor(220, 0, 0), QColor(0, 120, 220), QColor(0, 160, 0)};
    const QStringList names = {"N", "E", "U"};
    for (int comp = 0; comp < 3; ++comp) {
        QPointF prev;
        bool hasPrev = false;
        for (int i = 0; i < n; ++i) {
            const PppEpochRecord& rec = history[i];
            if (!rec.hasNeu) {
                hasPrev = false;
                continue;
            }
            const QPointF cur(xToScene(i), yToScene(rec.neu[comp]));
            if (hasPrev) {
                _pppNeuScene->addLine(prev.x(), prev.y(), cur.x(), cur.y(), QPen(colors[comp], 1.5));
            }
            hasPrev = true;
            prev = cur;
        }
        auto* legend = _pppNeuScene->addText(names[comp]);
        legend->setDefaultTextColor(colors[comp]);
        legend->setPos(left + 8.0 + comp * 20.0, top + 2.0);
    }

    const QString tStart = history.first().gpsTime.toString("HH:mm:ss");
    const QString tEnd = history.last().gpsTime.toString("HH:mm:ss");
    _pppNeuScene->addText(tStart)->setPos(left - 8.0, top + height + 8.0);
    _pppNeuScene->addText(tEnd)->setPos(left + width - 50.0, top + height + 8.0);

    ScopedTableSorting resultSorting(_pppResultTable);
    _pppResultTable->setRowCount(0);
    _pppResultTable->setRowCount(qMin(n, 300));
    const int start = qMax(0, n - 300);
    for (int i = n - 1, row = 0; i >= start; --i, ++row) {
        const PppEpochRecord& rec = history[i];
        const QStringList values = {
            rec.gpsTime.toString("HH:mm:ss"),
            QString("%1/%2").arg(rec.fixedSatCount).arg(rec.numSat),
            QString::number(rec.neu[0], 'f', 3),
            QString::number(rec.neu[1], 'f', 3),
            QString::number(rec.neu[2], 'f', 3),
            rec.hasStd ? QString::number(rec.neuStd[0], 'f', 4) : "-",
            rec.hasStd ? QString::number(rec.neuStd[1], 'f', 4) : "-",
            rec.hasStd ? QString::number(rec.neuStd[2], 'f', 4) : "-",
            rec.hasZtd ? QString::number(rec.ztdSigma, 'f', 4) : "-",
            QString::number(rec.numSat),
            QString::number(rec.hDop, 'f', 2)
        };
        for (int col = 0; col < values.size(); ++col) {
            QTableWidgetItem* item = _pppResultTable->item(row, col);
            if (!item) {
                item = new QTableWidgetItem;
                _pppResultTable->setItem(row, col, item);
            }
            item->setText(values[col]);
            if (col == 1) {
                item->setForeground(rec.fixed ? QBrush(QColor(0, 145, 0)) : QBrush(QColor(180, 110, 0)));
            }
        }
    }
}

void MainWindow::refreshZtdPlot() {
    if (!_ztdScene || !_ztdStationCombo || !_ztdInfoLabel) {
        return;
    }
    const QString source = _ztdStationCombo->currentData().toString();
    const QVector<PppEpochRecord> history = _pppHistory.value(source);

    _ztdScene->clear();
    if (history.isEmpty()) {
        _ztdInfoLabel->setText("No ZTD result yet.");
        _ztdScene->addText("No ZTD history for selected station")->setPos(-80, -10);
        return;
    }

    double minZtd = std::numeric_limits<double>::max();
    double maxZtd = -std::numeric_limits<double>::max();
    int validCount = 0;
    for (const PppEpochRecord& rec : history) {
        if (!rec.hasZtd) continue;
        minZtd = qMin(minZtd, rec.ztd);
        maxZtd = qMax(maxZtd, rec.ztd);
        ++validCount;
    }
    if (validCount == 0) {
        _ztdInfoLabel->setText("No ZTD result yet.");
        _ztdScene->addText("No valid ZTD samples")->setPos(-60, -10);
        return;
    }

    PppEpochRecord lastRec{};
    for (int i = history.size() - 1; i >= 0; --i) {
        if (history[i].hasZtd) {
            lastRec = history[i];
            break;
        }
    }
    QString zwdText = QStringLiteral("GNSS ZWD=%1 m")
                          .arg(lastRec.hasZwd ? QString::number(lastRec.zwd, 'f', 4) : QStringLiteral("-"));
    if (lastRec.hasGfsZwdPrior) {
        zwdText += QStringLiteral("  GFS ZWD prior=%1 m")
                       .arg(lastRec.zwdPrior, 0, 'f', 4);
    } else {
        zwdText += QStringLiteral("  GFS ZWD prior=-");
    }
    _ztdInfoLabel->setText(QString("Last ZTD=%1 m  sigma=%2 m  %3")
                               .arg(lastRec.ztd, 0, 'f', 4)
                               .arg(lastRec.ztdSigma, 0, 'f', 4)
                               .arg(zwdText));

    const int n = history.size();
    const double left = 50.0;
    const double top = 18.0;
    const double width = 860.0;
    const double height = 240.0;
    _ztdScene->setSceneRect(0.0, 0.0, left + width + 20.0, top + height + 35.0);

    const double span = qMax(0.001, maxZtd - minZtd);
    const double pad = span * 0.15;
    const double yMin = minZtd - pad;
    const double yMax = maxZtd + pad;

    _ztdScene->addRect(left, top, width, height, QPen(QColor(185, 185, 185)));
    auto yToScene = [&](double v) {
        return top + height - (v - yMin) / (yMax - yMin) * height;
    };
    auto xToScene = [&](int i) {
        if (n <= 1) return left;
        return left + width * (static_cast<double>(i) / static_cast<double>(n - 1));
    };

    QPointF prev;
    bool hasPrev = false;
    for (int i = 0; i < n; ++i) {
        const PppEpochRecord& rec = history[i];
        if (!rec.hasZtd) {
            hasPrev = false;
            continue;
        }
        const QPointF cur(xToScene(i), yToScene(rec.ztd));
        if (hasPrev) {
            _ztdScene->addLine(prev.x(), prev.y(), cur.x(), cur.y(), QPen(QColor(0, 120, 220), 1.6));
        }
        prev = cur;
        hasPrev = true;
    }

    _ztdScene->addText(QString::number(yMax, 'f', 3))->setPos(8.0, top - 6.0);
    _ztdScene->addText(QString::number(yMin, 'f', 3))->setPos(8.0, top + height - 14.0);
    const QString tStart = history.first().gpsTime.toString("HH:mm:ss");
    const QString tEnd = history.last().gpsTime.toString("HH:mm:ss");
    _ztdScene->addText(tStart)->setPos(left - 8.0, top + height + 8.0);
    _ztdScene->addText(tEnd)->setPos(left + width - 50.0, top + height + 8.0);
}

void MainWindow::refreshZwdPlot() {
    if (!_zwdScene || !_ztdStationCombo) return;
    const QString source = _ztdStationCombo->currentData().toString();
    const QVector<PppEpochRecord> history = _pppHistory.value(source);
    _zwdScene->clear();
    if (history.isEmpty()) {
        _zwdScene->addText("No ZWD history for selected station")->setPos(-80, -10);
        return;
    }
    double minValue = std::numeric_limits<double>::max();
    double maxValue = -std::numeric_limits<double>::max();
    bool any = false;
    for (const PppEpochRecord& rec : history) {
        if (rec.hasZwd) {
            minValue = qMin(minValue, rec.zwd);
            maxValue = qMax(maxValue, rec.zwd);
            any = true;
        }
        if (rec.hasGfsZwdPrior) {
            minValue = qMin(minValue, rec.zwdPrior);
            maxValue = qMax(maxValue, rec.zwdPrior);
            any = true;
        }
    }
    if (!any) {
        _zwdScene->addText("No ZWD samples or GFS prior yet")->setPos(-90, -10);
        return;
    }
    const int n = history.size();
    const double left = 50.0, top = 18.0, width = 860.0, height = 220.0;
    _zwdScene->setSceneRect(0.0, 0.0, left + width + 20.0, top + height + 40.0);
    const double span = qMax(0.001, maxValue - minValue);
    const double yMin = minValue - span * 0.15;
    const double yMax = maxValue + span * 0.15;
    _zwdScene->addRect(left, top, width, height, QPen(QColor(185, 185, 185)));
    auto yToScene = [&](double value) { return top + height - (value - yMin) / (yMax - yMin) * height; };
    auto xToScene = [&](int index) { return n <= 1 ? left : left + width * static_cast<double>(index) / static_cast<double>(n - 1); };
    _zwdScene->addText(QString::number(yMax, 'f', 3))->setPos(8.0, top - 6.0);
    _zwdScene->addText(QString::number(yMin, 'f', 3))->setPos(8.0, top + height - 14.0);
    auto drawSeries = [&](bool prior) {
        QPointF previous;
        bool havePrevious = false;
        const QColor color = prior ? QColor(220, 125, 0) : QColor(0, 120, 220);
        for (int i = 0; i < n; ++i) {
            const PppEpochRecord& rec = history[i];
            const bool valid = prior ? rec.hasGfsZwdPrior : rec.hasZwd;
            if (!valid) { havePrevious = false; continue; }
            const QPointF current(xToScene(i), yToScene(prior ? rec.zwdPrior : rec.zwd));
            if (havePrevious) {
                QPen pen(color, 1.6);
                if (prior) pen.setStyle(Qt::DashLine);
                _zwdScene->addLine(previous.x(), previous.y(), current.x(), current.y(), pen);
            }
            previous = current;
            havePrevious = true;
        }
    };
    drawSeries(false);
    drawSeries(true);
    auto* gnssLegend = _zwdScene->addText("GNSS ZWD");
    gnssLegend->setDefaultTextColor(QColor(0, 120, 220));
    gnssLegend->setPos(left + 8.0, top + 2.0);
    auto* gfsLegend = _zwdScene->addText("GFS prior ZWD");
    gfsLegend->setDefaultTextColor(QColor(220, 125, 0));
    gfsLegend->setPos(left + 100.0, top + 2.0);
    _zwdScene->addText(history.first().gpsTime.toString("HH:mm:ss"))->setPos(left - 8.0, top + height + 8.0);
    _zwdScene->addText(history.last().gpsTime.toString("HH:mm:ss"))->setPos(left + width - 50.0, top + height + 8.0);
}

void MainWindow::rebuildPppStationTable() {
    if (!_pppStationTable) {
        return;
    }
    QSet<QString> liveIds;
    for (const StreamRow& s : _rows) {
        liveIds.insert(s.id);
    }
    for (auto it = _pppStationEnabled.begin(); it != _pppStationEnabled.end();) {
        it = liveIds.contains(it.key()) ? std::next(it) : _pppStationEnabled.erase(it);
    }

    ScopedTableSorting sorting(_pppStationTable);
    _pppStationTable->blockSignals(true);
    _pppStationTable->setRowCount(0);
    _pppStationTable->setRowCount(_rows.size());
    for (int row = 0; row < _rows.size(); ++row) {
        const StreamRow& s = _rows[row];
        const QString station = streamMountpointById(s.id).isEmpty()
            ? streamLabelById(s.id)
            : streamMountpointById(s.id);
        bool enabled = _pppStationEnabled.isEmpty() ? true : _pppStationEnabled.value(s.id, true);
        _pppStationEnabled[s.id] = enabled;

        QTableWidgetItem* enableItem = _pppStationTable->item(row, 0);
        if (!enableItem) {
            enableItem = new QTableWidgetItem;
            enableItem->setFlags(enableItem->flags() | Qt::ItemIsUserCheckable | Qt::ItemIsEnabled);
            _pppStationTable->setItem(row, 0, enableItem);
        }
        enableItem->setData(Qt::UserRole, s.id);
        enableItem->setCheckState(enabled ? Qt::Checked : Qt::Unchecked);

        QTableWidgetItem* stationItem = _pppStationTable->item(row, 1);
        if (!stationItem) {
            stationItem = new QTableWidgetItem;
            _pppStationTable->setItem(row, 1, stationItem);
        }
        stationItem->setText(station);

        QTableWidgetItem* idItem = _pppStationTable->item(row, 2);
        if (!idItem) {
            idItem = new QTableWidgetItem;
            _pppStationTable->setItem(row, 2, idItem);
        }
        idItem->setText(s.id);
    }
    _pppStationTable->blockSignals(false);
}

bool MainWindow::isPppStationEnabled(const QString& sourceId) const {
    if (_pppStationEnabled.isEmpty()) {
        return true;
    }
    return _pppStationEnabled.value(sourceId, true);
}

void MainWindow::updatePppResults(const QString& source, const PppSolution& solution) {
    _lastPppSource = source;
    _lastPppSolution = solution;
    _lastPppSolutionBySource[source] = solution;
    if (!solution.valid) {
        if (_pppResultLabel) {
            _pppResultLabel->setText("No valid PPP solution yet.");
        }
        return;
    }
    QString stationName = streamMountpointById(source).isEmpty()
        ? streamLabelById(source)
        : streamMountpointById(source);
    if (stationName.trimmed().isEmpty()) {
        stationName = source;
    }
    _pppStationNameBySource[source] = stationName;

    std::array<double,3> labelRefXyz = {solution.xyz[0], solution.xyz[1], solution.xyz[2]};
    const PppCrdStationOptions stationOptions = _pppOptions.crdStationsBySource.value(source);
    if (stationOptions.valid) {
        labelRefXyz = {stationOptions.aprXyz[0], stationOptions.aprXyz[1], stationOptions.aprXyz[2]};
    } else if (_pppOptions.hasApriori()) {
        labelRefXyz = {_pppOptions.aprXyz[0], _pppOptions.aprXyz[1], _pppOptions.aprXyz[2]};
    }
    const QString jsonStationName = stationOptions.valid && !stationOptions.station.trimmed().isEmpty()
        ? stationOptions.station.trimmed()
        : stationName;
    postOutputPppJson(jsonStationName, solution, labelRefXyz);
    postTelemetryPppResiduals(source, solution);

    if (!_pppResultLabel || !_pppResultTable) {
        return;
    }
    const std::array<double,3> labelNeuStd = xyzCovToNeuStd(xyzToEll(labelRefXyz), solution.cov);

    _pppResultLabel->setText(QString("Last PPP solution from %1 at %2  STD_NEU=(%3,%4,%5)m  ZTD=%6 m  sZTD=%7 m")
                                 .arg(stationName)
                                 .arg(gpsOutputTime(solution).toString("yyyy-MM-dd HH:mm:ss"))
                                 .arg(labelNeuStd[0], 0, 'f', 4)
                                 .arg(labelNeuStd[1], 0, 'f', 4)
                                 .arg(labelNeuStd[2], 0, 'f', 4)
                                 .arg(solution.ztd, 0, 'f', 4)
                                 .arg(solution.ztdStdev, 0, 'f', 4));

    PppEpochRecord record;
    record.gpsTime = gpsOutputTime(solution);
    record.neu[0] = solution.neu[0];
    record.neu[1] = solution.neu[1];
    record.neu[2] = solution.neu[2];
    record.hasNeu = solution.valid &&
                    std::isfinite(record.neu[0]) &&
                    std::isfinite(record.neu[1]) &&
                    std::isfinite(record.neu[2]);
    record.xyzStd[0] = (std::isfinite(solution.cov[0]) && solution.cov[0] >= 0.0) ? std::sqrt(solution.cov[0]) : std::numeric_limits<double>::quiet_NaN();
    record.xyzStd[1] = (std::isfinite(solution.cov[3]) && solution.cov[3] >= 0.0) ? std::sqrt(solution.cov[3]) : std::numeric_limits<double>::quiet_NaN();
    record.xyzStd[2] = (std::isfinite(solution.cov[5]) && solution.cov[5] >= 0.0) ? std::sqrt(solution.cov[5]) : std::numeric_limits<double>::quiet_NaN();
    std::array<double,3> refXyz = labelRefXyz;
    const EllipsoidCoord ellRef = xyzToEll(refXyz);
    const std::array<double,3> neuStd = xyzCovToNeuStd(ellRef, solution.cov);
    record.neuStd[0] = neuStd[0];
    record.neuStd[1] = neuStd[1];
    record.neuStd[2] = neuStd[2];
    record.hasStd = std::isfinite(record.neuStd[0]) &&
                    std::isfinite(record.neuStd[1]) &&
                    std::isfinite(record.neuStd[2]) &&
                    std::isfinite(record.xyzStd[0]) &&
                    std::isfinite(record.xyzStd[1]) &&
                    std::isfinite(record.xyzStd[2]);
    record.numSat = solution.arTotalSatCount > 0 ? solution.arTotalSatCount : solution.numSat;
    record.hDop = solution.hDop;
    record.ztd = solution.ztd;
    record.ztdSigma = solution.ztdStdev;
    record.hasZtd = solution.valid && std::isfinite(solution.ztd);
    record.zwd = solution.trp;
    record.zwdPrior = solution.zwdPrior;
    record.hasZwd = solution.valid && std::isfinite(solution.trp);
    record.hasGfsZwdPrior = solution.gfsZwdPriorValid && std::isfinite(solution.zwdPrior);

    record.fixedSatCount = qMax(0, solution.arFixedSatCount);
    record.fixed = record.fixedSatCount > 0;
    record.ratio = solution.arRatio;

    auto& history = _pppHistory[source];
    if (!history.isEmpty() && history.last().gpsTime == record.gpsTime) {
        history.last() = record;
    } else {
        history.push_back(record);
    }
    const int maxKeep = 600;
    if (history.size() > maxKeep) {
        history.remove(0, history.size() - maxKeep);
    }

    writeZtdRecord(source, stationName, solution);
    schedulePppDisplayRefresh();
}

void MainWindow::schedulePppDisplayRefresh() {
    if (_closing || !_pppDisplayRefreshTimer || _pppDisplayRefreshTimer->isActive()) {
        return;
    }
    _pppDisplayRefreshTimer->start();
}

void MainWindow::scheduleSkyPlotRefresh() {
    if (_closing || !_skyPlotRefreshTimer || _skyPlotRefreshTimer->isActive()) {
        return;
    }
    _skyPlotRefreshTimer->start();
}

void MainWindow::writeZtdRecord(const QString& source, const QString& stationName, const PppSolution& solution) {
    if (_ztdOutputDirectory.trimmed().isEmpty() || !solution.valid || !std::isfinite(solution.ztd)) {
        return;
    }
    postOutputZtd(source, stationName, solution);
}

void MainWindow::updateStatusDetailsForRow(int row) {
    if (!_obsDetailLabel || !_obsCountTable || !_skyPlotScene || !_obsTrendScene || !_snrPlotScene) {
        return;
    }
    if (row < 0 || row >= _rows.size()) {
        _obsDetailLabel->setText("Select one station above to inspect observation details.");
        _obsCountTable->setRowCount(0);
        _obsTrendScene->clear();
        _snrPlotScene->clear();
        _skyPlotScene->clear();
        return;
    }

    const QString streamId = _rows[row].id;
    const auto summaries = _msmSummaries.value(streamId);
    if (summaries.isEmpty()) {
        _obsDetailLabel->setText("No MSM observation data decoded for this stream yet.");
        _obsCountTable->setRowCount(0);
        _obsTrendScene->clear();
        _snrPlotScene->clear();
        _skyPlotScene->clear();
        return;
    }

    ScopedTableSorting obsSorting(_obsCountTable);
    int totalSat = 0;
    int totalCells = 0;
    QDateTime latest;
    for (auto it = summaries.constBegin(); it != summaries.constEnd(); ++it) {
        totalSat += it.value().satCount;
        totalCells += it.value().cellCount;
        if (!latest.isValid() || it.value().receivedUtc > latest) {
            latest = it.value().receivedUtc;
        }
    }
    _obsDetailLabel->setText(QString("Stream %1: MSM4/5/6/7 decoded. Total sats=%2, cells=%3, last=%4. Sky plot uses broadcast-ephemeris az/el.")
                             .arg(streamLabelById(streamId))
                             .arg(totalSat)
                             .arg(totalCells)
                             .arg(latest.isValid() ? latest.toString("HH:mm:ss") : "-"));

    _obsCountTable->setRowCount(0);
    _obsCountTable->setRowCount(summaries.size());
    int maxSat = 1;
    for (auto it = summaries.constBegin(); it != summaries.constEnd(); ++it) {
        maxSat = qMax(maxSat, it.value().satCount);
    }

    int rowIndex = 0;
    for (auto it = summaries.constBegin(); it != summaries.constEnd(); ++it, ++rowIndex) {
        const MsmSystemSummary& summary = it.value();
        QTableWidgetItem* sysItem = _obsCountTable->item(rowIndex, 0);
        if (!sysItem) {
            sysItem = new QTableWidgetItem;
            _obsCountTable->setItem(rowIndex, 0, sysItem);
        }
        sysItem->setText(systemDisplayName(summary.system));

        QTableWidgetItem* satItem = _obsCountTable->item(rowIndex, 1);
        if (!satItem) {
            satItem = new QTableWidgetItem;
            _obsCountTable->setItem(rowIndex, 1, satItem);
        }
        if (_obsCountTable->cellWidget(rowIndex, 1)) {
            _obsCountTable->removeCellWidget(rowIndex, 1);
        }
        satItem->setText(QString("%1").arg(summary.satCount));
        satItem->setForeground(QBrush(systemColor(summary.system)));
        _obsCountTable->setRowHeight(rowIndex, 24);

        QTableWidgetItem* sigItem = _obsCountTable->item(rowIndex, 2);
        if (!sigItem) {
            sigItem = new QTableWidgetItem;
            _obsCountTable->setItem(rowIndex, 2, sigItem);
        }
        sigItem->setText(QString("signals=%1 cells=%2").arg(summary.signalCount).arg(summary.cellCount));
    }

    _obsTrendScene->clear();
    const double trendW = (_obsTrendView && _obsTrendView->viewport())
        ? qMax(680.0, static_cast<double>(_obsTrendView->viewport()->width() - 4))
        : 760.0;
    const QRectF trendRect(0.0, 0.0, trendW, 220.0);
    _obsTrendScene->setSceneRect(trendRect);
    _obsTrendScene->addRect(trendRect.adjusted(40.0, 10.0, -10.0, -28.0), QPen(QColor("#d6dee9")));
    const QRectF plotRect = trendRect.adjusted(40.0, 10.0, -10.0, -28.0);
    const QFont titleFont(QStringLiteral("Microsoft YaHei UI"), 9, QFont::DemiBold);
    const QFont axisFont(QStringLiteral("Microsoft YaHei UI"), 8);
    const QFont legendFont(QStringLiteral("Microsoft YaHei UI"), 8);
    auto* trendTitle = _obsTrendScene->addText("Satellite Count Trend (last 5 min)");
    trendTitle->setFont(titleFont);
    trendTitle->setDefaultTextColor(QColor("#1e293b"));
    trendTitle->setPos(8.0, 2.0);

    int globalMaxSat = 1;
    const auto streamHist = _satCountHistory.value(streamId);
    const QDateTime trendEnd = latest.isValid() ? latest : QDateTime::currentDateTimeUtc();
    const QDateTime trendStart = trendEnd.addSecs(-300);
    QMap<QChar, QVector<SatCountPoint>> trendSeries;
    for (auto it = streamHist.constBegin(); it != streamHist.constEnd(); ++it) {
        QVector<SatCountPoint> points;
        points.reserve(it.value().size());
        for (const SatCountPoint& p : it.value()) {
            if (p.utc >= trendStart && p.utc <= trendEnd) {
                points.push_back(p);
            }
        }
        if (points.isEmpty() && !it.value().isEmpty()) {
            points.push_back(it.value().last());
        }
        std::sort(points.begin(), points.end(),
                  [](const SatCountPoint& a, const SatCountPoint& b) { return a.utc < b.utc; });
        trendSeries[it.key()] = points;
        for (const SatCountPoint& p : points) {
            globalMaxSat = qMax(globalMaxSat, p.satCount);
        }
    }
    globalMaxSat = qMax(globalMaxSat, maxSat);

    for (int yTick = 0; yTick <= 4; ++yTick) {
        const double ratio = static_cast<double>(yTick) / 4.0;
        const double y = plotRect.bottom() - ratio * plotRect.height();
        _obsTrendScene->addLine(plotRect.left(), y, plotRect.right(), y, QPen(QColor("#eef2f7")));
        const int tickVal = static_cast<int>(std::round(ratio * globalMaxSat));
        auto* label = _obsTrendScene->addText(QString::number(tickVal));
        label->setFont(axisFont);
        label->setDefaultTextColor(QColor("#64748b"));
        label->setPos(4.0, y - 9.0);
    }
    for (int m = 0; m <= 5; ++m) {
        const double ratio = static_cast<double>(m) / 5.0;
        const double x = plotRect.left() + ratio * plotRect.width();
        _obsTrendScene->addLine(x, plotRect.top(), x, plotRect.bottom(), QPen(QColor("#f3f6fb")));
        auto* label = _obsTrendScene->addText(trendStart.addSecs(m * 60).toString("HH:mm"));
        label->setFont(axisFont);
        label->setDefaultTextColor(QColor("#64748b"));
        label->setPos(x - 18.0, plotRect.bottom() + 4.0);
    }

    for (auto it = trendSeries.constBegin(); it != trendSeries.constEnd(); ++it) {
        const QChar sys = it.key();
        const QVector<SatCountPoint>& points = it.value();
        if (points.isEmpty()) {
            continue;
        }
        const QColor color = systemColor(sys);
        QPointF prev;
        bool havePrev = false;
        for (int i = 0; i < points.size(); ++i) {
            const double secFromStart = static_cast<double>(trendStart.msecsTo(points.at(i).utc)) / 1000.0;
            const double xRatio = qBound(0.0, secFromStart / 300.0, 1.0);
            const double yRatio = static_cast<double>(points.at(i).satCount) / static_cast<double>(globalMaxSat);
            const double x = plotRect.left() + xRatio * plotRect.width();
            const double y = plotRect.bottom() - yRatio * plotRect.height();
            if (!std::isfinite(x) || !std::isfinite(y)) {
                continue;
            }
            const QPointF cur(x, y);
            if (havePrev) {
                _obsTrendScene->addLine(prev.x(), prev.y(), cur.x(), cur.y(), QPen(color, 1.6));
            }
            prev = cur;
            havePrev = true;
        }
        if (havePrev) {
            _obsTrendScene->addEllipse(prev.x() - 2.0, prev.y() - 2.0, 4.0, 4.0, QPen(color), QBrush(color));
        }
    }

    double legendX = plotRect.left();
    const double legendY = plotRect.bottom() + 6.0;
    for (auto it = summaries.constBegin(); it != summaries.constEnd(); ++it) {
        const QColor c = systemColor(it.key());
        _obsTrendScene->addRect(legendX, legendY + 4.0, 12.0, 4.0, QPen(c), QBrush(c));
        auto* t = _obsTrendScene->addText(systemDisplayName(it.key()));
        t->setFont(legendFont);
        t->setDefaultTextColor(QColor("#334155"));
        t->setPos(legendX + 16.0, legendY - 4.0);
        legendX += 145.0;
    }
    if (_obsTrendView) {
        _obsTrendView->resetTransform();
        _obsTrendView->setSceneRect(_obsTrendScene->sceneRect());
        _obsTrendView->centerOn(_obsTrendScene->sceneRect().center());
    }

    _snrPlotScene->clear();
    const double snrW = (_snrPlotView && _snrPlotView->viewport())
        ? qMax(760.0, static_cast<double>(_snrPlotView->viewport()->width() - 4))
        : 860.0;
    const QRectF snrRect(0.0, 0.0, snrW, 520.0);
    _snrPlotScene->setSceneRect(snrRect);
    auto* snrTitle = _snrPlotScene->addText("SNR by PRN (dB-Hz)");
    snrTitle->setFont(titleFont);
    snrTitle->setDefaultTextColor(QColor("#1e293b"));
    snrTitle->setPos(8.0, 2.0);

    const QVector<QChar> snrSystems = {
        QLatin1Char('G'), QLatin1Char('R'), QLatin1Char('E'), QLatin1Char('C')
    };
    const double minSnr = 20.0;
    const double maxSnr = 60.0;
    const double maxReasonableSnr = 100.0;
    const double gapX = 20.0;
    const double gapY = 26.0;
    const double panelW = (snrW - 24.0 - gapX) / 2.0;
    const double panelH = 224.0;
    bool hasSnr = false;

    for (int idx = 0; idx < snrSystems.size(); ++idx) {
        const QChar sys = snrSystems[idx];
        const int rowPanel = idx / 2;
        const int colPanel = idx % 2;
        const QRectF panel(12.0 + colPanel * (panelW + gapX),
                           32.0 + rowPanel * (panelH + gapY),
                           panelW,
                           panelH);
        const QRectF barRect = panel.adjusted(42.0, 28.0, -10.0, -30.0);
        _snrPlotScene->addRect(panel, QPen(QColor("#d6dee9")), QBrush(QColor("#fbfdff")));
        _snrPlotScene->addRect(barRect, QPen(QColor("#d6dee9")));

        auto* title = _snrPlotScene->addText(systemDisplayName(sys));
        title->setFont(titleFont);
        title->setDefaultTextColor(systemColor(sys));
        title->setPos(panel.left() + 8.0, panel.top() + 4.0);

        for (int t = 0; t <= 4; ++t) {
            const double ratio = static_cast<double>(t) / 4.0;
            const double y = barRect.bottom() - ratio * barRect.height();
            _snrPlotScene->addLine(barRect.left(), y, barRect.right(), y, QPen(QColor("#edf2f7")));
            const double tick = minSnr + ratio * (maxSnr - minSnr);
            auto* label = _snrPlotScene->addText(QString::number(tick, 'f', 0));
            label->setFont(axisFont);
            label->setDefaultTextColor(QColor("#64748b"));
            label->setPos(panel.left() + 6.0, y - 9.0);
        }

        const auto itSummary = summaries.constFind(sys);
        if (itSummary == summaries.constEnd() || itSummary.value().satCnr.isEmpty()) {
            auto* empty = _snrPlotScene->addText("No data");
            empty->setFont(axisFont);
            empty->setDefaultTextColor(QColor("#94a3b8"));
            empty->setPos(barRect.center().x() - 22.0, barRect.center().y() - 10.0);
            continue;
        }

        QVector<QPair<int, double>> values;
        for (auto itSat = itSummary.value().satCnr.constBegin(); itSat != itSummary.value().satCnr.constEnd(); ++itSat) {
            const double snr = itSat.value();
            if (std::isfinite(snr) && snr >= 0.0 && snr <= maxReasonableSnr) {
                values.push_back(qMakePair(itSat.key(), snr));
            }
        }
        if (values.isEmpty()) {
            auto* empty = _snrPlotScene->addText("No valid SNR");
            empty->setFont(axisFont);
            empty->setDefaultTextColor(QColor("#94a3b8"));
            empty->setPos(barRect.center().x() - 34.0, barRect.center().y() - 10.0);
            continue;
        }
        std::sort(values.begin(), values.end(),
                  [](const QPair<int, double>& a, const QPair<int, double>& b) { return a.first < b.first; });

        hasSnr = true;
        double sumSnr = 0.0;
        for (const auto& v : values) {
            sumSnr += v.second;
        }
        const double avgSnr = sumSnr / static_cast<double>(values.size());
        const double slotW = barRect.width() / static_cast<double>(values.size());
        const double barW = qMax(3.0, qMin(18.0, slotW * 0.66));
        const QColor color = systemColor(sys);

        for (int i = 0; i < values.size(); ++i) {
            const int sat = values[i].first;
            const double snrRaw = values[i].second;
            const double snrPlot = qBound(minSnr, snrRaw, maxSnr);
            const double ratio = (snrPlot - minSnr) / (maxSnr - minSnr);
            const double x = barRect.left() + slotW * (i + 0.5);
            const double barTop = barRect.bottom() - ratio * barRect.height();
            const QRectF bar(x - barW / 2.0, barTop, barW, barRect.bottom() - barTop);
            _snrPlotScene->addRect(bar, QPen(color.darker(115)), QBrush(color));

            if (values.size() <= 18 || i % 2 == 0) {
                auto* label = _snrPlotScene->addText(QString("%1%2").arg(sys).arg(sat, 2, 10, QLatin1Char('0')));
                label->setFont(axisFont);
                label->setDefaultTextColor(QColor("#475569"));
                label->setRotation(-55.0);
                label->setPos(x - 12.0, barRect.bottom() + 18.0);
            }
        }

        const double avgPlot = qBound(minSnr, avgSnr, maxSnr);
        const double avgRatio = (avgPlot - minSnr) / (maxSnr - minSnr);
        const double avgY = barRect.bottom() - avgRatio * barRect.height();
        _snrPlotScene->addLine(barRect.left(), avgY, barRect.right(), avgY,
                               QPen(QColor("#dc2626"), 1.4, Qt::DashLine));
        auto* avgLabel = _snrPlotScene->addText(QString("avg %1").arg(avgSnr, 0, 'f', 1));
        avgLabel->setFont(axisFont);
        avgLabel->setDefaultTextColor(QColor("#dc2626"));
        avgLabel->setPos(barRect.right() - 54.0, avgY - 20.0);
    }
    if (!hasSnr) {
        auto* noSnr = _snrPlotScene->addText("No SNR data yet for selected stream.");
        noSnr->setFont(axisFont);
        noSnr->setPos(12.0, 24.0);
    }
    if (_snrPlotView) {
        _snrPlotView->resetTransform();
        _snrPlotView->setSceneRect(_snrPlotScene->sceneRect());
        _snrPlotView->centerOn(_snrPlotScene->sceneRect().center());
    }

    _skyPlotScene->clear();
    const double skyW = (_skyPlotView && _skyPlotView->viewport())
        ? qMax(760.0, static_cast<double>(_skyPlotView->viewport()->width() - 4))
        : 860.0;
    const double skyH = (_skyPlotView && _skyPlotView->viewport())
        ? qMax(390.0, static_cast<double>(_skyPlotView->viewport()->height() - 4))
        : 430.0;
    const QRectF skySceneRect(0.0, 0.0, skyW, skyH);
    _skyPlotScene->setSceneRect(skySceneRect);

    const double gap = 18.0;
    const QRectF livePanel(12.0, 10.0, (skyW - 36.0 - gap) / 2.0, skyH - 24.0);
    const QRectF histPanel(livePanel.right() + gap, 10.0, livePanel.width(), livePanel.height());

    auto drawSkyFrame = [&](const QRectF& panel, const QString& title) -> QPair<QPointF, double> {
        _skyPlotScene->addRect(panel, QPen(QColor("#d6dee9")), QBrush(QColor("#fbfdff")));
        auto* titleItem = _skyPlotScene->addText(title);
        titleItem->setFont(titleFont);
        titleItem->setDefaultTextColor(QColor("#1e293b"));
        titleItem->setPos(panel.left() + 8.0, panel.top() + 4.0);

        const double radius = qMax(90.0, qMin(panel.width() - 34.0, panel.height() - 58.0) * 0.5);
        const QPointF center(panel.center().x(), panel.top() + 34.0 + radius);
        _skyPlotScene->addEllipse(center.x() - radius, center.y() - radius, radius * 2, radius * 2, QPen(Qt::lightGray));
        _skyPlotScene->addEllipse(center.x() - radius * 0.66, center.y() - radius * 0.66,
                                  radius * 1.32, radius * 1.32, QPen(Qt::lightGray));
        _skyPlotScene->addEllipse(center.x() - radius * 0.33, center.y() - radius * 0.33,
                                  radius * 0.66, radius * 0.66, QPen(Qt::lightGray));
        _skyPlotScene->addLine(center.x() - radius, center.y(), center.x() + radius, center.y(), QPen(Qt::lightGray));
        _skyPlotScene->addLine(center.x(), center.y() - radius, center.x(), center.y() + radius, QPen(Qt::lightGray));
        _skyPlotScene->addText("N")->setPos(center.x() - 6.0, center.y() - radius - 18.0);
        _skyPlotScene->addText("E")->setPos(center.x() + radius + 6.0, center.y() - 6.0);
        _skyPlotScene->addText("S")->setPos(center.x() - 6.0, center.y() + radius + 2.0);
        _skyPlotScene->addText("W")->setPos(center.x() - radius - 18.0, center.y() - 6.0);
        return qMakePair(center, radius);
    };
    auto finishSkyView = [&]() {
        if (_skyPlotView) {
            _skyPlotView->resetTransform();
            _skyPlotView->setSceneRect(_skyPlotScene->sceneRect());
            _skyPlotView->fitInView(_skyPlotScene->sceneRect(), Qt::KeepAspectRatio);
            _skyPlotView->centerOn(_skyPlotScene->sceneRect().center());
        }
    };

    if (!_skyReceiverPositionValid.value(streamId, false)) {
        _obsDetailLabel->setText(_obsDetailLabel->text() +
                                 " Sky plot needs receiver position: set PPP apriori, stream lat/lon, or wait for PPP solution.");
        drawSkyFrame(livePanel, "Realtime Sky Plot");
        drawSkyFrame(histPanel, "History Sky Plot");
        _skyPlotScene->addText("No receiver position")->setPos(livePanel.center().x() - 58.0, livePanel.center().y());
        finishSkyView();
        return;
    }

    QVector<SkyPlotPoint> livePoints;
    const auto liveSystems = _liveSkyPoints.constFind(streamId);
    if (liveSystems != _liveSkyPoints.constEnd()) {
        for (auto it = liveSystems->constBegin(); it != liveSystems->constEnd(); ++it) {
            livePoints += it.value();
        }
    }
    if (livePoints.isEmpty()) {
        drawSkyFrame(livePanel, "Realtime Sky Plot");
        drawSkyFrame(histPanel, "History Sky Plot");
        _skyPlotScene->addText("No satellite positions available")->setPos(livePanel.center().x() - 92.0, livePanel.center().y());
        finishSkyView();
        return;
    }

    qint64 latestMsmEpochKey = 0;
    const auto msmSystems = _lastMsmMessages.constFind(streamId);
    if (msmSystems != _lastMsmMessages.constEnd()) {
        for (auto it = msmSystems->constBegin(); it != msmSystems->constEnd(); ++it) {
            const GnssUtils::GpsTime msmTime = GnssUtils::gpsTimeFromMsm(it.value());
            latestMsmEpochKey = qMax(latestMsmEpochKey,
                                     static_cast<qint64>(msmTime.week) * 604800000LL +
                                     static_cast<qint64>(std::llround(msmTime.sow * 1000.0)));
        }
    }
    if (!livePoints.isEmpty() && latestMsmEpochKey > 0 &&
        _lastSkyHistoryEpochMs.value(streamId) != latestMsmEpochKey) {
        QVector<SkyPlotPoint>& history = _skyHistory[streamId];
        constexpr qint64 kSkyHistoryBucketMs = 60 * 1000;
        for (const SkyPlotPoint& point : livePoints) {
            const qint64 pointMs = point.utc.isValid()
                ? point.utc.toMSecsSinceEpoch()
                : QDateTime::currentDateTimeUtc().toMSecsSinceEpoch();
            const qint64 bucketMs = (pointMs / kSkyHistoryBucketMs) * kSkyHistoryBucketMs;
            const QString satKey = QString("%1%2").arg(point.system).arg(point.prn, 2, 10, QLatin1Char('0'));
            const QString bucketKey = QString("%1|%2").arg(streamId, satKey);
            if (_lastSkyHistoryBucketMs.value(bucketKey, std::numeric_limits<qint64>::min()) == bucketMs) {
                continue;
            }
            _lastSkyHistoryBucketMs[bucketKey] = bucketMs;
            history.push_back(point);
        }
        _lastSkyHistoryEpochMs[streamId] = latestMsmEpochKey;
        const QDateTime keepAfter = (latest.isValid() ? latest : QDateTime::currentDateTimeUtc()).addSecs(-24 * 60 * 60);
        while (!history.isEmpty() && history.first().utc < keepAfter) {
            history.removeFirst();
        }
        constexpr int kMaxSkyHistoryPoints = 6000;
        if (history.size() > kMaxSkyHistoryPoints) {
            history.remove(0, history.size() - kMaxSkyHistoryPoints);
        }
        const qint64 keepAfterMs = keepAfter.toMSecsSinceEpoch();
        const QString bucketPrefix = streamId + "|";
        for (auto it = _lastSkyHistoryBucketMs.begin(); it != _lastSkyHistoryBucketMs.end();) {
            if (it.key().startsWith(bucketPrefix) && it.value() < keepAfterMs) {
                it = _lastSkyHistoryBucketMs.erase(it);
            } else {
                ++it;
            }
        }
    }

    const auto liveFrame = drawSkyFrame(livePanel, QString("Realtime Sky Plot (%1)").arg(livePoints.size()));
    const int histSec = _skyHistoryWindowCombo ? _skyHistoryWindowCombo->currentData().toInt() : 5 * 60;
    const QString histLabel = _skyHistoryWindowCombo ? _skyHistoryWindowCombo->currentText() : QStringLiteral("5 min");
    const QDateTime histEnd = latest.isValid() ? latest : QDateTime::currentDateTimeUtc();
    const QDateTime histStart = histEnd.addSecs(-qMax(60, histSec));
    QVector<SkyPlotPoint> histPoints;
    const QVector<SkyPlotPoint> history = _skyHistory.value(streamId);
    for (const SkyPlotPoint& p : history) {
        if (p.utc >= histStart && p.utc <= histEnd) {
            histPoints.push_back(p);
        }
    }
    QMap<QString, QVector<SkyPlotPoint>> histBySat;
    for (const SkyPlotPoint& point : histPoints) {
        const QString satKey = QString("%1%2").arg(point.system).arg(point.prn, 2, 10, QLatin1Char('0'));
        histBySat[satKey].push_back(point);
    }
    const auto histFrame = drawSkyFrame(histPanel, QString("History Sky Plot (%1, %2 pts)").arg(histLabel).arg(histPoints.size()));

    auto drawPoint = [&](const SkyPlotPoint& point, const QPointF& center, double radius, bool live) {
        const double r = radius * std::cos(point.el);
        const double x = center.x() + r * std::sin(point.az);
        const double y = center.y() - r * std::cos(point.az);
        QColor color = systemColor(point.system);
        if (!live) {
            color.setAlpha(70);
        }
        const double dot = live ? 5.0 : 2.2;
        _skyPlotScene->addEllipse(x - dot, y - dot, dot * 2.0, dot * 2.0, QPen(color), QBrush(color));
        if (live) {
            auto* label = _skyPlotScene->addText(QString("%1%2").arg(point.system).arg(point.prn, 2, 10, QLatin1Char('0')));
            label->setFont(axisFont);
            label->setDefaultTextColor(QColor("#334155"));
            label->setPos(x + 4.0, y + 4.0);
        }
    };
    auto skyPoint = [](const SkyPlotPoint& point, const QPointF& center, double radius) {
        const double r = radius * std::cos(point.el);
        return QPointF(center.x() + r * std::sin(point.az),
                       center.y() - r * std::cos(point.az));
    };
    for (auto it = histBySat.constBegin(); it != histBySat.constEnd(); ++it) {
        const QVector<SkyPlotPoint>& track = it.value();
        if (track.isEmpty()) {
            continue;
        }
        constexpr int kMaxSkyTrackSegments = 180;
        const int step = qMax(1, track.size() / kMaxSkyTrackSegments);
        QPointF prevPt;
        bool havePrev = false;
        for (int idx = 0; idx < track.size(); ++idx) {
            if (idx > 0 && idx < track.size() - 1 && (idx % step) != 0) {
                continue;
            }
            const SkyPlotPoint& point = track[idx];
            const QPointF pt = skyPoint(point, histFrame.first, histFrame.second);
            if (!std::isfinite(pt.x()) || !std::isfinite(pt.y())) {
                continue;
            }
            QColor color = systemColor(point.system);
            color.setAlpha(95);
            if (havePrev) {
                _skyPlotScene->addLine(prevPt.x(), prevPt.y(), pt.x(), pt.y(), QPen(color, 1.2));
            }
            prevPt = pt;
            havePrev = true;
        }
        QColor color = systemColor(track.first().system);
        color.setAlpha(95);
        const QPointF lastPt = skyPoint(track.last(), histFrame.first, histFrame.second);
        if (std::isfinite(lastPt.x()) && std::isfinite(lastPt.y())) {
            _skyPlotScene->addEllipse(lastPt.x() - 2.2, lastPt.y() - 2.2, 4.4, 4.4,
                                      QPen(color), QBrush(color));
        }
    }
    for (const SkyPlotPoint& point : livePoints) {
        drawPoint(point, liveFrame.first, liveFrame.second, true);
    }

    if (livePoints.isEmpty()) {
        _skyPlotScene->addText("No satellites with ephemeris")->setPos(livePanel.center().x() - 86.0, livePanel.center().y());
    }
    if (histPoints.isEmpty()) {
        _skyPlotScene->addText("No history in selected window")->setPos(histPanel.center().x() - 92.0, histPanel.center().y());
    }

    finishSkyView();
}

void MainWindow::rebuildStreamsTable() {
    ScopedTableSorting sorting(_streamsTable);
    _streamsTable->setRowCount(0);
    _streamsTable->setRowCount(_rows.size());
    for (int i = 0; i < _rows.size(); ++i) {
        updateTableRow(i);
    }
    _streamCountLabel->setText(QString("Streams: %1").arg(_rows.size()));
    rebuildDataPreparationStreamSelectors();
    rebuildPppStationTable();
    rebuildStatusTable();
}

void MainWindow::updateTableRow(int row) {
    if (row < 0 || row >= _rows.size()) {
        return;
    }
    const StreamRow& s = _rows[row];
    ScopedTableSorting sorting(_streamsTable);
    int tableRow = tableRowForStreamId(_streamsTable, s.id);
    if (tableRow < 0) {
        tableRow = row;
    }
    QString dataUrl = s.sourceDisplay;
    QString mountpoint = "-";
    if (s.type == "NTRIP") {
        const QString host = s.config.value("host").toString();
        const int port = s.config.value("port").toInt();
        const QString mp = s.config.value("mountPoint").toString();
        dataUrl = QString("ntrip://%1:%2").arg(host).arg(port);
        mountpoint = mp.isEmpty() ? "-" : mp;
    } else if (s.type == "TCP") {
        const QString host = s.config.value("host").toString();
        const int port = s.config.value("port").toInt();
        dataUrl = QString("tcp://%1:%2").arg(host).arg(port);
    } else if (s.type == "MQTT") {
        const QString host = s.config.value("host").toString();
        const int port = s.config.value("port").toInt();
        const QString topic = s.config.value("topic").toString();
        dataUrl = QString("mqtt://%1:%2/%3").arg(host).arg(port).arg(topic);
    }

    const QStringList values = {
        dataUrl, mountpoint, s.decoder, s.lat, s.lon, s.nmea, s.stateText, bytesToText(s.bytes)
    };
    for (int col = 0; col < values.size(); ++col) {
        QTableWidgetItem* item = _streamsTable->item(tableRow, col);
        if (!item) {
            item = new QTableWidgetItem;
            _streamsTable->setItem(tableRow, col, item);
        }
        item->setText(values[col]);
        item->setData(Qt::UserRole, s.id);
    }
}

void MainWindow::rebuildStatusTable() {
    QString selectedStatusId = selectedStatusPlotStreamId();
    if (_statusTable && _statusTable->selectionModel()) {
        const QModelIndexList selected = _statusTable->selectionModel()->selectedRows();
        if (selectedStatusId.isEmpty() && !selected.isEmpty()) {
            selectedStatusId = tableRowStreamId(_statusTable, selected.first().row());
        }
    }
    ScopedTableSorting sorting(_statusTable);
    _statusTable->setRowCount(0);
    _statusTable->setRowCount(_rows.size());
    for (int row = 0; row < _rows.size(); ++row) {
        updateStatusRow(row);
    }
    refreshStatusPlotStationSelector();
    selectedStatusId = selectedStatusPlotStreamId();
    const int selectedStatusRow = tableRowForStreamId(_statusTable, selectedStatusId);
    if (selectedStatusRow >= 0) {
        _statusTable->selectRow(selectedStatusRow);
        const int selectedStreamRow = rowForStreamId(selectedStatusId);
        updateStatusDetailsForRow(selectedStreamRow);
    } else {
        updateStatusDetailsForRow(rowForStreamId(selectedStatusPlotStreamId()));
    }
}

void MainWindow::updateStatusRow(int row) {
    if (row < 0 || row >= _rows.size()) {
        return;
    }
    const StreamRow& s = _rows[row];
    ScopedTableSorting sorting(_statusTable);
    int tableRow = tableRowForStreamId(_statusTable, s.id);
    if (tableRow < 0) {
        tableRow = row;
    }
    const QString lastError = (s.client ? s.client->runtimeState().lastError : QString());
    const QStringList vals = {s.sourceDisplay, s.type, s.stateText, bytesToText(s.bytes), lastError};
    for (int col = 0; col < vals.size(); ++col) {
        QTableWidgetItem* item = _statusTable->item(tableRow, col);
        if (!item) {
            item = new QTableWidgetItem;
            _statusTable->setItem(tableRow, col, item);
        }
        item->setText(vals[col]);
        item->setData(Qt::UserRole, s.id);
    }
}

int MainWindow::selectedRow() const {
    const QModelIndexList selected = _streamsTable->selectionModel()->selectedRows();
    if (selected.isEmpty()) {
        return -1;
    }
    const QString streamId = tableRowStreamId(_streamsTable, selected.first().row());
    if (streamId.isEmpty()) {
        return selected.first().row();
    }
    for (int row = 0; row < _rows.size(); ++row) {
        if (_rows[row].id == streamId) {
            return row;
        }
    }
    return -1;
}

BaseStreamClient* MainWindow::createClientForRow(int row) {
    if (row < 0 || row >= _rows.size()) {
        return nullptr;
    }
    StreamRow& s = _rows[row];
    if (s.client) {
        return s.client;
    }

    const QString sourceName = QString("%1 #%2").arg(s.type).arg(s.id);
    if (s.type == "NTRIP") {
        auto* c = new NtripStreamClient(this, sourceName);
        NtripStreamConfig cfg;
        cfg.host = s.config.value("host").toString();
        cfg.port = s.config.value("port").toInt();
        cfg.mountPoint = s.config.value("mountPoint").toString();
        cfg.user = s.config.value("user").toString();
        cfg.password = s.config.value("password").toString();
        cfg.nmea = s.config.value("nmea").toString();
        cfg.ntripVersion = s.config.value("ntripVersion").toString("2");
        cfg.reconnectMs = s.config.value("reconnectMs").toInt(3000);
        c->setConfig(cfg);
        s.client = c;
    } else if (s.type == "TCP") {
        auto* c = new TcpStreamClient(this, sourceName);
        TcpStreamConfig cfg;
        cfg.host = s.config.value("host").toString();
        cfg.port = s.config.value("port").toInt();
        cfg.reconnectMs = s.config.value("reconnectMs").toInt(3000);
        c->setConfig(cfg);
        s.client = c;
    } else if (s.type == "MQTT") {
        auto* c = new MqttStreamClient(this, sourceName);
        MqttStreamConfig cfg;
        cfg.host = s.config.value("host").toString();
        cfg.port = s.config.value("port").toInt(1883);
        cfg.topic = s.config.value("topic").toString();
        cfg.clientId = s.config.value("clientId").toString();
        cfg.user = s.config.value("user").toString();
        cfg.password = s.config.value("password").toString();
        cfg.reconnectMs = s.config.value("reconnectMs").toInt(3000);
        cfg.keepAliveSec = s.config.value("keepAliveSec").toInt(30);
        c->setConfig(cfg);
        s.client = c;
    }

    if (s.client) {
        connectClientSignals(s);
    }
    return s.client;
}

void MainWindow::connectClientSignals(StreamRow& row) {
    BaseStreamClient* c = row.client;
    if (!c) {
        return;
    }
    const QString id = row.id;

    connect(c, &BaseStreamClient::logMessage, this, &MainWindow::appendLog);
    connect(c, &BaseStreamClient::dataFrame, this,
            [this, id](const QString&, qint64 bytes, const QByteArray& data) {
                if (_closing) {
                    return;
                }
                appendDataPreview(id, bytes, data);
                decodeRtcmPayload(id, data);
            });
    connect(c, &BaseStreamClient::stateChanged, this, [this, id]() {
        if (_closing) {
            return;
        }
        for (int i = 0; i < _rows.size(); ++i) {
            if (_rows[i].id != id || !_rows[i].client) {
                continue;
            }
            const StreamRuntimeState st = _rows[i].client->runtimeState();
            const QString previousState = _lastStreamTelemetryState.value(id);
            const bool telemetryStateChanged = previousState != st.stateText;
            _lastStreamTelemetryState[id] = st.stateText;
            _rows[i].stateText = st.stateText;
            _rows[i].bytes = st.bytes;
            if (!st.lastError.isEmpty()) {
                appendLog(id, st.lastError);
            }
            if (telemetryStateChanged && _telemetryEnabled) {
                const QString status = st.stateText.trimmed().toLower().replace(' ', '_');
                const QString reason = (status == "reconnecting" || status == "error")
                    ? QStringLiteral("stream_interruption") : QString();
                postTelemetryReception(
                    id, QDateTime::currentDateTimeUtc(), "STREAM", 0, QChar(), status,
                    reason, st.lastError);
            }
            updateTableRow(i);
            updateStatusRow(i);
            break;
        }
    });
}

void MainWindow::startRow(int row) {
    configureOutputWorker();
    BaseStreamClient* client = createClientForRow(row);
    if (!client) {
        return;
    }
    client->start();
    _rows[row].stateText = client->runtimeState().stateText;
    updateTableRow(row);
    updateStatusRow(row);
}

void MainWindow::stopRow(int row) {
    if (row < 0 || row >= _rows.size() || !_rows[row].client) {
        return;
    }
    _rows[row].client->stop();
    _rows[row].stateText = "Stopped";
    updateTableRow(row);
    updateStatusRow(row);
}

void MainWindow::stopAndDeleteClient(StreamRow& row) {
    if (_pppRuntimeService && !_closing) {
        _pppRuntimeService->clearSource(row.id);
    }
    if (_rtcmProcessingService && !_closing) {
        const QString sourceId = row.id;
        const Qt::ConnectionType type =
            _rtcmProcessingService->thread() == QThread::currentThread()
                ? Qt::DirectConnection : Qt::BlockingQueuedConnection;
        QMetaObject::invokeMethod(_rtcmProcessingService,
                                  "clearSource",
                                  type,
                                  Q_ARG(QString, sourceId));
    }
    _msmSummaries.remove(row.id);
    _satCountHistory.remove(row.id);
    _lastMsmMessages.remove(row.id);
    _liveSkyPoints.remove(row.id);
    _skyReceiverPositionValid.remove(row.id);
    _skyHistory.remove(row.id);
    _lastSkyHistoryEpochMs.remove(row.id);
    _lastPppSolutionBySource.remove(row.id);
    _lastStreamTelemetryState.remove(row.id);
    const QString skyBucketPrefix = row.id + "|";
    for (auto it = _lastSkyHistoryBucketMs.begin(); it != _lastSkyHistoryBucketMs.end();) {
        if (it.key().startsWith(skyBucketPrefix)) {
            it = _lastSkyHistoryBucketMs.erase(it);
        } else {
            ++it;
        }
    }
    if (!row.client) {
        return;
    }
    row.client->stop();
    row.client->disconnect(this);
    delete row.client;
    row.client = nullptr;
    row.stateText = "Stopped";
}

bool MainWindow::loadStreamsFromSettings(QString* error) {
    if (error) error->clear();
    QJsonArray arr;
    if (!_settingsStore.loadStreams(arr, error)) {
        return false;
    }
    QVector<StreamRow> rows;
    if (!parseStreamArray(arr, rows, error)) {
        return false;
    }

    if (_pppRuntimeService) {
        _pppRuntimeService->clearAll();
    }
    _rows.clear();
    clearRtcmProcessingState();
    _msmSummaries.clear();
    _satCountHistory.clear();
    _lastMsmMessages.clear();
    _liveSkyPoints.clear();
    _skyReceiverPositionValid.clear();
    _preparedStates.clear();
    _lastPppSolutionBySource.clear();
    _rows = rows;
    for (const StreamRow& row : _rows) {
        bool ok = false;
        const int id = row.id.toInt(&ok);
        if (ok) _idSeed = qMax(_idSeed, id + 1);
    }
    return true;
}

bool MainWindow::parseStreamArray(const QJsonArray& arr, QVector<StreamRow>& parsedRows,
                                  QString* error) const {
    if (error) error->clear();
    QVector<StreamRow> rows;
    QSet<QString> ids;
    for (int index = 0; index < arr.size(); ++index) {
        const QJsonValue value = arr.at(index);
        if (!value.isObject()) {
            if (error) *error = QStringLiteral("streams/list_json entry %1 must be an object").arg(index);
            return false;
        }
        const QJsonObject object = value.toObject();
        if (object.contains(QStringLiteral("config")) && !object.value(QStringLiteral("config")).isObject()) {
            if (error) *error = QStringLiteral("streams/list_json entry %1 config must be an object").arg(index);
            return false;
        }
        const QJsonObject storedConfig = object.value(QStringLiteral("config")).toObject();
        const QStringList credentialKeys = {QStringLiteral("user"), QStringLiteral("password")};
        for (const QString& key : credentialKeys) {
            if ((object.contains(key) && !object.value(key).isString()) ||
                (storedConfig.contains(key) && !storedConfig.value(key).isString())) {
                if (error) *error = QStringLiteral("streams/list_json entry %1 has a non-text %2").arg(index).arg(key);
                return false;
            }
        }
        const QString id = object.value(QStringLiteral("id")).toString().trimmed();
        const QString type = object.value(QStringLiteral("type")).toString().trimmed().toUpper();
        if (id.isEmpty() || ids.contains(id)) {
            if (error) *error = QStringLiteral("streams/list_json entry %1 has an empty or duplicate id").arg(index);
            return false;
        }
        if (type != QLatin1String("NTRIP") && type != QLatin1String("TCP") && type != QLatin1String("MQTT")) {
            if (error) *error = QStringLiteral("streams/list_json entry %1 has unsupported type '%2'").arg(index).arg(type);
            return false;
        }
        if (type == QLatin1String("NTRIP")) {
            auto validateVersion = [&](const QJsonObject& values, const QString& key) {
                if (!values.contains(key)) return true;
                const QJsonValue value = values.value(key);
                if (value.isString() && isSupportedNtripVersion(value.toString())) return true;
                if (error) {
                    *error = QStringLiteral("streams/list_json entry %1 has invalid %2")
                                 .arg(index).arg(key);
                }
                return false;
            };
            if (!validateVersion(object, QStringLiteral("version")) ||
                !validateVersion(object, QStringLiteral("ntripVersion")) ||
                !validateVersion(storedConfig, QStringLiteral("ntripVersion"))) {
                return false;
            }
        }
        StreamRow row = rowFromJson(object);
        row.type = type;
        const QString host = row.config.value(QStringLiteral("host")).toString().trimmed();
        bool portOk = false;
        const int port = row.config.value(QStringLiteral("port")).toVariant().toInt(&portOk);
        const bool endpointValid = !host.isEmpty() && portOk && port > 0 && port <= 65535;
        const bool protocolValid = type == QLatin1String("MQTT")
            ? !row.config.value(QStringLiteral("topic")).toString().trimmed().isEmpty()
            : (type != QLatin1String("NTRIP") || !row.config.value(QStringLiteral("mountPoint")).toString().trimmed().isEmpty());
        if (!endpointValid || !protocolValid) {
            if (error) *error = QStringLiteral("streams/list_json entry %1 has an invalid host, port, mount point, or topic").arg(index);
            return false;
        }
        ids.insert(id);
        rows.push_back(row);
    }
    parsedRows = rows;
    return true;
}

bool MainWindow::saveStreamsToSettings(QString* error) {
    QJsonArray arr;
    for (const StreamRow& row : _rows) {
        arr.push_back(rowToJson(row));
    }
    QString localError;
    if (_settingsStore.saveStreams(arr, &localError)) {
        if (error) error->clear();
        return true;
    }
    if (error) {
        *error = localError;
    } else {
        const QString message = QStringLiteral("Stream configuration save failed: %1").arg(localError);
        qWarning().noquote() << message;
        appendLog("SYSTEM", message);
    }
    return false;
}

QJsonObject MainWindow::rowToJson(const StreamRow& row) const {
    QJsonObject o;
    o["id"] = row.id;
    o["type"] = row.type;
    o["decoder"] = row.decoder;
    o["lat"] = row.lat;
    o["lon"] = row.lon;
    o["nmea"] = row.nmea;
    o["config"] = row.config;
    if (row.type == "NTRIP") {
        o["host"] = row.config.value("host").toString();
        o["port"] = row.config.value("port").toInt(2101);
        o["mountPoint"] = row.config.value("mountPoint").toString();
        o["user"] = row.config.value("user").toString();
        o["password"] = row.config.value("password").toString();
        o["version"] = normalizedNtripVersion(row.config.value("ntripVersion").toString("2"));
        o["reconnectMs"] = row.config.value("reconnectMs").toInt(3000);
        const QString gga = row.config.value("nmea").toString();
        if (!gga.isEmpty()) {
            o["gga"] = gga;
        }
    } else {
        o["sourceDisplay"] = row.sourceDisplay;
    }
    return o;
}

MainWindow::StreamRow MainWindow::rowFromJson(const QJsonObject& obj) const {
    StreamRow row;
    row.id = obj.value("id").toString();
    row.type = obj.value("type").toString();
    row.sourceDisplay = obj.value("sourceDisplay").toString();
    row.decoder = obj.value("decoder").toString();
    row.lat = obj.value("lat").toString("-");
    row.lon = obj.value("lon").toString("-");
    row.nmea = obj.value("nmea").toString("no");
    row.stateText = "Stopped";
    row.bytes = 0;
    row.config = obj.value("config").toObject();
    if (row.type == "NTRIP") {
        // Nested config is canonical; copy root-level fields only from older files.
        auto copyLegacyTextIfMissing = [&](const QString& key, const QString& legacyKey) {
            const QJsonValue legacy = obj.value(legacyKey);
            if (!row.config.contains(key) && legacy.isString()) {
                row.config[key] = legacy;
            }
        };
        copyLegacyTextIfMissing(QStringLiteral("host"), QStringLiteral("host"));
        copyLegacyTextIfMissing(QStringLiteral("mountPoint"), QStringLiteral("mountPoint"));
        if (!obj.contains(QStringLiteral("mountPoint"))) {
            copyLegacyTextIfMissing(QStringLiteral("mountPoint"), QStringLiteral("mount"));
        }
        copyLegacyTextIfMissing(QStringLiteral("user"), QStringLiteral("user"));
        copyLegacyTextIfMissing(QStringLiteral("password"), QStringLiteral("password"));
        const QJsonValue legacyPort = obj.value(QStringLiteral("port"));
        if (!row.config.contains(QStringLiteral("port")) && legacyPort.isDouble() &&
            legacyPort.toInt() > 0) {
            row.config[QStringLiteral("port")] = legacyPort;
        }
        if (!row.config.contains("ntripVersion")) {
            row.config["ntripVersion"] = normalizedNtripVersion(
                obj.contains("version") ? obj.value("version").toString()
                : (obj.contains("ntripVersion") ? obj.value("ntripVersion").toString() : QStringLiteral("2")));
        } else {
            row.config["ntripVersion"] = normalizedNtripVersion(row.config.value("ntripVersion").toString());
        }
        const QJsonValue legacyReconnect = obj.value(QStringLiteral("reconnectMs"));
        if (!row.config.contains(QStringLiteral("reconnectMs")) && legacyReconnect.isDouble() &&
            legacyReconnect.toInt() > 0) {
            row.config[QStringLiteral("reconnectMs")] = legacyReconnect;
        }
        if (!row.config.contains("nmea") && obj.contains("gga")) row.config["nmea"] = obj.value("gga").toString();
        if (row.sourceDisplay.isEmpty()) {
            row.sourceDisplay = QString("%1:%2/%3")
                                    .arg(row.config.value("host").toString())
                                    .arg(row.config.value("port").toInt())
                                    .arg(row.config.value("mountPoint").toString());
        }
    }
    return row;
}

void MainWindow::closeEvent(QCloseEvent* event) {
    _closing = true;
    saveStreamsToSettings();
    saveDataPreparationToSettings();
    savePppSettingsToSettings();
    event->accept();
}

QString MainWindow::settingsFilePath() const {
    const QString path = _settingsStore.filePath();
    return path.isEmpty() ? QStringLiteral("rtppp_settings.ini") : path;
}
