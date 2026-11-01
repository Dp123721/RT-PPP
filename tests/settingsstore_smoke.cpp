// Checks settings persistence, credential retention, validation, and reload errors.
#include "settingsstore.h"

#include <QCoreApplication>
#include <QJsonDocument>
#include <QJsonObject>
#include <QSettings>
#include <QTemporaryDir>

#include <iostream>

int main(int argc, char** argv) {
    QCoreApplication app(argc, argv);
    QTemporaryDir temp;
    if (!temp.isValid()) {
        std::cerr << "settingsstore smoke failure: temporary directory unavailable\n";
        return 1;
    }

    const QString path = temp.filePath(QStringLiteral("settings.ini"));
    SettingsStore store(path);
    const QJsonObject stream{
        {QStringLiteral("id"), QStringLiteral("1")},
        {QStringLiteral("type"), QStringLiteral("NTRIP")},
        {QStringLiteral("config"), QJsonObject{
            {QStringLiteral("host"), QStringLiteral("caster.example")},
            {QStringLiteral("port"), 2101},
            {QStringLiteral("user"), QStringLiteral("receiver-user")},
            {QStringLiteral("password"), QStringLiteral("persist-this-secret")}
        }}
    };
    QString error;
    if (!store.saveStreams(QJsonArray{stream}, &error)) {
        std::cerr << "settingsstore smoke failure: save failed\n";
        return 1;
    }

    SettingsStore reopened(path);
    QJsonArray loaded;
    if (!reopened.loadStreams(loaded, &error) || loaded.size() != 1 ||
        loaded.first().toObject().value(QStringLiteral("config")).toObject()
            .value(QStringLiteral("user")).toString() != QStringLiteral("receiver-user") ||
        loaded.first().toObject().value(QStringLiteral("config")).toObject()
            .value(QStringLiteral("password")).toString() != QStringLiteral("persist-this-secret")) {
        std::cerr << "settingsstore smoke failure: credentials did not survive reload\n";
        return 1;
    }

    const QJsonObject caster{
        {QStringLiteral("name"), QStringLiteral("Test caster")},
        {QStringLiteral("host"), QStringLiteral("caster.example")},
        {QStringLiteral("port"), 2101},
        {QStringLiteral("version"), QStringLiteral("2s")},
        {QStringLiteral("user"), QStringLiteral("caster-user")},
        {QStringLiteral("password"), QStringLiteral("caster-secret")}
    };
    if (!store.saveCasters(QJsonArray{caster}, &error)) {
        std::cerr << "settingsstore smoke failure: caster preset save failed\n";
        return 1;
    }
    QJsonArray loadedCasters;
    if (!reopened.loadCasters(loadedCasters, &error) || loadedCasters.size() != 1 ||
        loadedCasters.first().toObject().value(QStringLiteral("user")).toString() != QStringLiteral("caster-user") ||
        loadedCasters.first().toObject().value(QStringLiteral("password")).toString() != QStringLiteral("caster-secret")) {
        std::cerr << "settingsstore smoke failure: caster credentials did not survive reload\n";
        return 1;
    }

    QSettings currentAlgorithm(path, QSettings::IniFormat);
    currentAlgorithm.setValue(QStringLiteral("ppp/arAlgorithm"), QStringLiteral("BIE"));
    currentAlgorithm.sync();
    if (!reopened.validate(&error) ||
        SettingsStore::normalizedArAlgorithm(QStringLiteral("BIE")) != QStringLiteral("BIE")) {
        std::cerr << "settingsstore smoke failure: current BIE setting was rejected\n";
        return 1;
    }

    QSettings legacyAlgorithm(path, QSettings::IniFormat);
    legacyAlgorithm.setValue(QStringLiteral("ppp/arAlgorithm"), QStringLiteral("BNC_BIE"));
    legacyAlgorithm.sync();
    if (!reopened.validate(&error) ||
        SettingsStore::normalizedArAlgorithm(QStringLiteral("BNC_BIE")) != QStringLiteral("BIE")) {
        std::cerr << "settingsstore smoke failure: legacy BIE setting was rejected\n";
        return 1;
    }

    QSettings corrupt(path, QSettings::IniFormat);
    corrupt.setValue(QStringLiteral("streams/list_json"), QStringLiteral("[{broken-json]"));
    corrupt.sync();
    if (reopened.loadStreams(loaded, &error) || error.isEmpty() || loaded.size() != 1) {
        std::cerr << "settingsstore smoke failure: invalid JSON was accepted or discarded loaded data\n";
        return 1;
    }

    QSettings valid(path, QSettings::IniFormat);
    valid.setValue(QStringLiteral("streams/list_json"),
                   QString::fromUtf8(QJsonDocument(QJsonArray{stream}).toJson(QJsonDocument::Compact)));
    valid.setValue(QStringLiteral("ppp/useGps"), QStringLiteral("not-a-bool"));
    valid.sync();
    if (reopened.validate(&error) || !error.contains(QStringLiteral("ppp/useGps"))) {
        std::cerr << "settingsstore smoke failure: invalid scalar configuration was accepted\n";
        return 1;
    }

    QSettings invalidCaster(path, QSettings::IniFormat);
    invalidCaster.setValue(QStringLiteral("ppp/useGps"), true);
    invalidCaster.setValue(QStringLiteral("casters/list_json"),
                           QString::fromUtf8(QJsonDocument(QJsonArray{QJsonObject{
                               {QStringLiteral("host"), QStringLiteral("caster.example")},
                               {QStringLiteral("port"), 2101},
                               {QStringLiteral("version"), QStringLiteral("ntrip-9")}
                           }}).toJson(QJsonDocument::Compact)));
    invalidCaster.sync();
    if (reopened.validate(&error) || !error.contains(QStringLiteral("unsupported version"))) {
        std::cerr << "settingsstore smoke failure: invalid caster version was accepted\n";
        return 1;
    }

    std::cout << "settingsstore smoke test passed\n";
    return 0;
}
