// Declares the compact, versioned PPP JSON writer and per-station file lifecycle.
#pragma once

#include "ppp_engine.h"

#include <QDate>
#include <QMap>
#include <QString>
#include <array>

class PppJsonWriter
{
public:
    struct Options {
        bool enabled = true;
        QString outputDirectory = QStringLiteral("output/ppp_json");
        int reorderBufferMs = 2000;
        int flushIntervalMs = 1000;
    };

    PppJsonWriter() = default;
    ~PppJsonWriter();

    bool setOptions(const Options& options, QString* error = nullptr);
    bool append(const QString& stationId,
                const PppSolution& solution,
                const std::array<double, 3>& referenceEcef,
                QString* error = nullptr);
    bool close(QString* error = nullptr);

private:
    struct StationState;

    bool openState(const QString& stationId,
                   const QString& safeStationId,
                   const QDate& date,
                   const PppSolution& firstSolution,
                   const std::array<double, 3>& referenceEcef,
                   StationState*& state,
                   QString* error);
    bool recoverOpenFile(StationState& state, QString* error);
    bool flushPending(StationState& state, bool force, QString* error);
    bool finalizeState(StationState& state, QString* error);

    static QString safeStationId(const QString& stationId);

    Options _options;
    QMap<QString, StationState*> _states;
};
