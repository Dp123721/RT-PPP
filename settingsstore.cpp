// SettingsStore validates and persists application preferences and stream presets.
#include "settingsstore.h"

#include <QJsonDocument>
#include <QJsonObject>
#include <QFileInfo>
#include <QSettings>
#include <QVariant>
#include <cmath>

namespace {

void setError(QString* error, const QString& message) {
    if (error) *error = message;
}

bool loadArray(const QString& path, const QString& key, QJsonArray& output, QString* error) {
    QSettings settings(path, QSettings::IniFormat);
    if (settings.status() != QSettings::NoError) {
        setError(error, QStringLiteral("cannot read settings %1 (status=%2)")
                           .arg(QFileInfo(path).absoluteFilePath())
                           .arg(static_cast<int>(settings.status())));
        return false;
    }
    if (!settings.contains(key)) {
        output = QJsonArray();
        return true;
    }
    const QString jsonText = settings.value(key).toString();
    QJsonParseError parseError;
    const QJsonDocument document = QJsonDocument::fromJson(jsonText.toUtf8(), &parseError);
    if (parseError.error != QJsonParseError::NoError || !document.isArray()) {
        setError(error, QStringLiteral("invalid %1 in %2: %3")
                           .arg(key, QFileInfo(path).absoluteFilePath(),
                                parseError.errorString()));
        return false;
    }
    if (settings.status() != QSettings::NoError) {
        setError(error, QStringLiteral("cannot read settings %1 (status=%2)")
                           .arg(QFileInfo(path).absoluteFilePath())
                           .arg(static_cast<int>(settings.status())));
        return false;
    }
    output = document.array();
    return true;
}

bool saveArray(const QString& path, const QString& key, const QJsonArray& value,
               QString* error) {
    QSettings settings(path, QSettings::IniFormat);
    settings.setValue(key, QString::fromUtf8(QJsonDocument(value).toJson(QJsonDocument::Compact)));
    settings.sync();
    if (settings.status() != QSettings::NoError) {
        setError(error, QStringLiteral("cannot write settings %1 (status=%2)")
                           .arg(QFileInfo(path).absoluteFilePath())
                           .arg(static_cast<int>(settings.status())));
        return false;
    }
    return true;
}

bool parseBool(const QSettings& settings, const QString& key, QString* error) {
    if (!settings.contains(key)) return true;
    const QString value = settings.value(key).toString().trimmed().toLower();
    if (value == QStringLiteral("true") || value == QStringLiteral("false") ||
        value == QStringLiteral("1") || value == QStringLiteral("0") ||
        value == QStringLiteral("yes") || value == QStringLiteral("no") ||
        value == QStringLiteral("on") || value == QStringLiteral("off")) {
        return true;
    }
    setError(error, QStringLiteral("%1 must be a boolean (true/false)").arg(key));
    return false;
}

bool parseInteger(const QSettings& settings, const QString& key, int minimum,
                  int maximum, QString* error) {
    if (!settings.contains(key)) return true;
    bool ok = false;
    const QString text = settings.value(key).toString().trimmed();
    const int value = text.toInt(&ok);
    if (!ok || value < minimum || value > maximum) {
        setError(error, QStringLiteral("%1 must be an integer in [%2, %3]")
                           .arg(key).arg(minimum).arg(maximum));
        return false;
    }
    return true;
}

bool parseAllowedInteger(const QSettings& settings, const QString& key,
                         const QList<int>& allowed, QString* error) {
    if (!settings.contains(key)) return true;
    bool ok = false;
    const int value = settings.value(key).toString().trimmed().toInt(&ok);
    if (!ok || !allowed.contains(value)) {
        QStringList values;
        for (int item : allowed) values << QString::number(item);
        setError(error, QStringLiteral("%1 must be one of: %2")
                           .arg(key, values.join(QStringLiteral(", "))));
        return false;
    }
    return true;
}

bool parseDouble(const QSettings& settings, const QString& key, double minimum,
                 double maximum, QString* error) {
    if (!settings.contains(key)) return true;
    bool ok = false;
    const QString text = settings.value(key).toString().trimmed();
    const double value = text.toDouble(&ok);
    if (!ok || !std::isfinite(value) || value < minimum || value > maximum) {
        setError(error, QStringLiteral("%1 must be a finite number in [%2, %3]")
                           .arg(key).arg(minimum, 0, 'g', 12).arg(maximum, 0, 'g', 12));
        return false;
    }
    return true;
}

bool parseChoice(const QSettings& settings, const QString& key,
                 const QStringList& choices, QString* error) {
    if (!settings.contains(key)) return true;
    const QString value = settings.value(key).toString().trimmed().toLower();
    if (!choices.contains(value)) {
        setError(error, QStringLiteral("%1 has unsupported value '%2'").arg(key, value));
        return false;
    }
    return true;
}

bool validateCasterEntries(const QJsonArray& casters, QString* error) {
    const QStringList stringFields = {
        QStringLiteral("name"), QStringLiteral("host"), QStringLiteral("user"),
        QStringLiteral("password"), QStringLiteral("version"), QStringLiteral("ntripVersion")
    };
    const QStringList supportedVersions = {
        QStringLiteral(""), QStringLiteral("1"), QStringLiteral("v1"),
        QStringLiteral("http/1.0"), QStringLiteral("2"), QStringLiteral("v2"),
        QStringLiteral("http/1.1"), QStringLiteral("2s"), QStringLiteral("ssl"),
        QStringLiteral("https")
    };

    for (int index = 0; index < casters.size(); ++index) {
        if (!casters.at(index).isObject()) {
            setError(error, QStringLiteral("casters/list_json entry %1 must be an object").arg(index));
            return false;
        }
        const QJsonObject caster = casters.at(index).toObject();
        const QString host = caster.value(QStringLiteral("host")).toString().trimmed();
        bool portOk = false;
        const int port = caster.value(QStringLiteral("port")).toVariant().toInt(&portOk);
        if (host.isEmpty() || !portOk || port <= 0 || port > 65535) {
            setError(error, QStringLiteral("casters/list_json entry %1 has invalid host or port").arg(index));
            return false;
        }
        for (const QString& key : stringFields) {
            if (caster.contains(key) && !caster.value(key).isString()) {
                setError(error, QStringLiteral("casters/list_json entry %1 field %2 must be text")
                                   .arg(index).arg(key));
                return false;
            }
        }
        for (const QString& key : {QStringLiteral("version"), QStringLiteral("ntripVersion")}) {
            if (caster.contains(key) &&
                !supportedVersions.contains(caster.value(key).toString().trimmed().toLower())) {
                setError(error, QStringLiteral("casters/list_json entry %1 has unsupported %2")
                                   .arg(index).arg(key));
                return false;
            }
        }
    }
    return true;
}

} // namespace

SettingsStore::SettingsStore(const QString& filePath)
    : _filePath(filePath) {
}

QString SettingsStore::normalizedArAlgorithm(const QString& value) {
    const QString normalized = value.trimmed().toLower();
    if (normalized == QLatin1String("bie") || normalized == QLatin1String("bnc_bie")) {
        return QStringLiteral("BIE");
    }
    return QStringLiteral("LAMBDA");
}

QJsonArray SettingsStore::loadStreams() const {
    QJsonArray streams;
    loadStreams(streams);
    return streams;
}

void SettingsStore::saveStreams(const QJsonArray& streams) {
    saveStreams(streams, nullptr);
}

bool SettingsStore::loadStreams(QJsonArray& streams, QString* error) const {
    return loadArray(_filePath, QStringLiteral("streams/list_json"), streams, error);
}

bool SettingsStore::saveStreams(const QJsonArray& streams, QString* error) {
    QSettings s(_filePath, QSettings::IniFormat);
    s.setValue(QStringLiteral("streams/list_json"),
               QString::fromUtf8(QJsonDocument(streams).toJson(QJsonDocument::Compact)));
    s.remove("streams/ntrip_json");
    s.remove("streams/tcp_json");
    s.remove("streams/mqtt_json");
    s.sync();
    if (s.status() != QSettings::NoError) {
        setError(error, QStringLiteral("cannot update settings %1 (status=%2)")
                           .arg(QFileInfo(_filePath).absoluteFilePath())
                           .arg(static_cast<int>(s.status())));
        return false;
    }
    return true;
}

QJsonArray SettingsStore::loadCasters() const {
    QJsonArray casters;
    loadCasters(casters);
    return casters;
}

void SettingsStore::saveCasters(const QJsonArray& casters) {
    saveCasters(casters, nullptr);
}

bool SettingsStore::loadCasters(QJsonArray& casters, QString* error) const {
    return loadArray(_filePath, QStringLiteral("casters/list_json"), casters, error);
}

bool SettingsStore::saveCasters(const QJsonArray& casters, QString* error) {
    return saveArray(_filePath, QStringLiteral("casters/list_json"), casters, error);
}

bool SettingsStore::validate(QString* error) const {
    if (error) error->clear();
    QSettings settings(_filePath, QSettings::IniFormat);
    settings.sync();
    if (settings.status() != QSettings::NoError) {
        setError(error, QStringLiteral("cannot read settings %1 (status=%2)")
                           .arg(QFileInfo(_filePath).absoluteFilePath())
                           .arg(static_cast<int>(settings.status())));
        return false;
    }

    const QVector<QPair<QString, QStringList>> choices = {
        {QStringLiteral("ppp/processingMode"), {QStringLiteral("realtime"), QStringLiteral("post")} },
        {QStringLiteral("ppp/postProductMode"), {QStringLiteral("broadcast"), QStringLiteral("precise")} },
        {QStringLiteral("ppp/troposphereMode"), {QStringLiteral("original"), QStringLiteral("gfs")} },
        {QStringLiteral("ppp/gfsZwdMethod"), {QStringLiteral("profile"), QStringLiteral("askne")} },
        {QStringLiteral("ppp/mappingMode"), {QStringLiteral("simple"), QStringLiteral("gpt3")} },
        // Accept the historical BIE marker so existing INI files remain loadable.
        {QStringLiteral("ppp/arAlgorithm"),
         {QStringLiteral("lambda"), QStringLiteral("bie"), QStringLiteral("bnc_bie")} },
        {QStringLiteral("ppp/lcGps"), {QStringLiteral("p3&l3"), QStringLiteral("p3"), QStringLiteral("l3"), QStringLiteral("pi&li"), QStringLiteral("pi"), QStringLiteral("p1&l1"), QStringLiteral("p1"), QStringLiteral("l1l2")} },
        {QStringLiteral("ppp/lcGlo"), {QStringLiteral("p3&l3"), QStringLiteral("p3"), QStringLiteral("l3"), QStringLiteral("pi&li"), QStringLiteral("pi"), QStringLiteral("p1&l1"), QStringLiteral("p1"), QStringLiteral("l1l2")} },
        {QStringLiteral("ppp/lcGal"), {QStringLiteral("p3&l3"), QStringLiteral("p3"), QStringLiteral("l3"), QStringLiteral("pi&li"), QStringLiteral("pi"), QStringLiteral("p1&l1"), QStringLiteral("p1"), QStringLiteral("l1l2")} },
        {QStringLiteral("ppp/lcBds"), {QStringLiteral("p3&l3"), QStringLiteral("p3"), QStringLiteral("l3"), QStringLiteral("pi&li"), QStringLiteral("pi"), QStringLiteral("p1&l1"), QStringLiteral("p1"), QStringLiteral("l1l2")} }
    };
    for (const auto& choice : choices) {
        if (!parseChoice(settings, choice.first, choice.second, error)) return false;
    }

    const QStringList boolKeys = {
        QStringLiteral("prep/enabled"), QStringLiteral("prep/sp3Enabled"),
        QStringLiteral("prep/rnxEnabled"), QStringLiteral("prep/telemetryEnabled"),
        QStringLiteral("ppp/useGps"), QStringLiteral("ppp/useGlo"),
        QStringLiteral("ppp/useGal"), QStringLiteral("ppp/useBds"),
        QStringLiteral("ppp/strictCorrectionSource"), QStringLiteral("ppp/pseudoObsIono"),
        QStringLiteral("ppp/eleWgtCode"), QStringLiteral("ppp/eleWgtPhase"),
        QStringLiteral("ppp/useAntex"), QStringLiteral("ppp/isApc"),
        QStringLiteral("ppp/useOceanTide"), QStringLiteral("ppp/logSsrYawDetails"),
        QStringLiteral("ppp/logBiasMatchDetails"), QStringLiteral("ppp/estimateTroposphere"),
        QStringLiteral("ppp/estimateIonosphere"), QStringLiteral("ppp/pppRtk"),
        QStringLiteral("ppp/arEnable"), QStringLiteral("ppp/arUseGps"),
        QStringLiteral("ppp/arUseGlonass"), QStringLiteral("ppp/arUseGalileo"),
        QStringLiteral("ppp/arUseBds"), QStringLiteral("ppp/arBiePersistConstraints"),
        QStringLiteral("ppp/arUseHistoryAid"), QStringLiteral("ppp/jsonEnabled")
    };
    for (const QString& key : boolKeys) {
        if (!parseBool(settings, key, error)) return false;
    }

    const QStringList nonNegativeIntegers = {
        QStringLiteral("ppp/jsonReorderBufferMs"),
        QStringLiteral("ppp/jsonFlushIntervalMs"), QStringLiteral("ppp/corrWaitSec"),
        QStringLiteral("ppp/gfsMaxAgeSec"), QStringLiteral("ppp/arHistMinFixEpochs"),
        QStringLiteral("ppp/arHistMinLockEpochs"), QStringLiteral("ppp/arPartialWarmupLockEpochs")
    };
    for (const QString& key : nonNegativeIntegers) {
        if (!parseInteger(settings, key, 0, 2147483647, error)) return false;
    }
    if (!parseInteger(settings, QStringLiteral("prep/sp3Sample"), 1, 3600, error) ||
        !parseAllowedInteger(settings, QStringLiteral("prep/observationSample"), {1, 5, 10, 30}, error) ||
        !parseInteger(settings, QStringLiteral("ppp/minObs"), 3, 20, error) ||
        !parseInteger(settings, QStringLiteral("ppp/gfsPrefetchTimeoutSec"), 30, 300, error) ||
        !parseInteger(settings, QStringLiteral("ppp/arMinMwSamples"), 1, 200, error) ||
        !parseInteger(settings, QStringLiteral("ppp/arMinFixPairs"), 1, 20, error) ||
        !parseInteger(settings, QStringLiteral("ppp/arBieMinEpochs"), 1, 10000, error) ||
        !parseInteger(settings, QStringLiteral("ppp/arBieMinSat"), 2, 100, error) ||
        !parseInteger(settings, QStringLiteral("ppp/arBiePersistConfirmEpochs"), 1, 100, error) ||
        !parseInteger(settings, QStringLiteral("ppp/queueMax"), 1, 2147483647, error) ||
        !parseInteger(settings, QStringLiteral("ppp/dropLagSec"), 1, 2147483647, error) ||
        !parseAllowedInteger(settings, QStringLiteral("ppp/postSampleSec"), {1, 5, 10, 30}, error)) {
        return false;
    }

    const QStringList finiteDoubles = {
        QStringLiteral("ppp/sigmaC1"), QStringLiteral("ppp/sigmaL1"),
        QStringLiteral("ppp/maxResC1"), QStringLiteral("ppp/maxResL1"),
        QStringLiteral("ppp/minEleDeg"), QStringLiteral("ppp/aprSigX"),
        QStringLiteral("ppp/aprSigY"), QStringLiteral("ppp/aprSigZ"),
        QStringLiteral("ppp/noiseX"), QStringLiteral("ppp/noiseY"),
        QStringLiteral("ppp/noiseZ"), QStringLiteral("ppp/aprSigClk"),
        QStringLiteral("ppp/aprSigTrp"), QStringLiteral("ppp/noiseTrp"),
        QStringLiteral("ppp/aprSigAmb"), QStringLiteral("ppp/aprSigIon"),
        QStringLiteral("ppp/noiseIon"), QStringLiteral("ppp/aprSigCodeBias"),
        QStringLiteral("ppp/noiseCodeBias"), QStringLiteral("ppp/aprSigPhaseBias"),
        QStringLiteral("ppp/noisePhaseBias"), QStringLiteral("ppp/aprX"),
        QStringLiteral("ppp/aprY"), QStringLiteral("ppp/aprZ"),
        QStringLiteral("ppp/neuN"), QStringLiteral("ppp/neuE"),
        QStringLiteral("ppp/neuU"), QStringLiteral("ppp/arMinEleDeg"),
        QStringLiteral("ppp/arWlMaxDev"), QStringLiteral("ppp/arWlMaxSig"),
        QStringLiteral("ppp/arNlRatio"), QStringLiteral("ppp/arNlBoot"),
        QStringLiteral("ppp/arNlMaxDev"), QStringLiteral("ppp/arConstraintSigma"),
        QStringLiteral("ppp/arBieMaxFrac"), QStringLiteral("ppp/arBieMaxSig"),
        QStringLiteral("ppp/arBieConstraintSigma"), QStringLiteral("ppp/arBiePersistMaxVerticalM"),
        QStringLiteral("ppp/arBiePersistMaxZtdM"), QStringLiteral("ppp/arHistRelaxFactor"),
        QStringLiteral("ppp/sigmaGim"), QStringLiteral("ppp/maxResGim"),
        QStringLiteral("ppp/seedingTime")
    };
    for (const QString& key : finiteDoubles) {
        if (!parseDouble(settings, key, -1.0e12, 1.0e12, error)) return false;
    }
    const QVector<QPair<QString, QPair<double, double>>> boundedDoubles = {
        {QStringLiteral("ppp/sigmaC1"), {0.01, 50.0}},
        {QStringLiteral("ppp/sigmaL1"), {0.0001, 5.0}},
        {QStringLiteral("ppp/maxResC1"), {0.1, 100.0}},
        {QStringLiteral("ppp/maxResL1"), {0.001, 10.0}},
        {QStringLiteral("ppp/minEleDeg"), {0.0, 40.0}},
        {QStringLiteral("ppp/seedingTime"), {0.0, 36000.0}},
        {QStringLiteral("ppp/sigmaGim"), {1.0e-6, 1.0e12}},
        {QStringLiteral("ppp/maxResGim"), {1.0e-6, 1.0e12}},
        {QStringLiteral("ppp/arMinEleDeg"), {0.0, 40.0}},
        {QStringLiteral("ppp/arBieMaxFrac"), {0.0, 1.0}},
        {QStringLiteral("ppp/arBieMaxSig"), {0.0, 10.0}},
        {QStringLiteral("ppp/arBieConstraintSigma"), {1.0e-6, 1.0}},
        {QStringLiteral("ppp/arBiePersistMaxVerticalM"), {0.001, 2.0}},
        {QStringLiteral("ppp/arBiePersistMaxZtdM"), {0.001, 1.0}},
        {QStringLiteral("ppp/arWlMaxDev"), {0.0, 2.0}},
        {QStringLiteral("ppp/arWlMaxSig"), {0.0, 1.0}},
        {QStringLiteral("ppp/arNlRatio"), {1.0, 20.0}},
        {QStringLiteral("ppp/arNlBoot"), {-1.0, 1.0}},
        {QStringLiteral("ppp/arNlMaxDev"), {0.0, 5.0}},
        {QStringLiteral("ppp/arConstraintSigma"), {0.0001, 10.0}},
        {QStringLiteral("ppp/arHistRelaxFactor"), {1.0, 3.0}}
    };
    for (const auto& rule : boundedDoubles) {
        if (!parseDouble(settings, rule.first, rule.second.first, rule.second.second, error)) return false;
    }

    QJsonArray streams;
    if (!loadStreams(streams, error)) return false;
    QJsonArray casters;
    if (!loadCasters(casters, error)) return false;
    if (!validateCasterEntries(casters, error)) return false;
    Q_UNUSED(streams);
    return true;
}
