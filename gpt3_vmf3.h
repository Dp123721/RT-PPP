// Declares GPT3/VMF3 tropospheric parameter interpolation.
#pragma once

#include <QDateTime>
#include <QString>
#include <memory>

class QProcess;

struct Gpt3Vmf3Mapping {
    double hydrostatic = 0.0;
    double wet = 0.0;
    double meanWaterVaporTemperatureK = 0.0;
};

// Invokes the unmodified GPT3 and VMF3 Fortran routines through a persistent
// local worker. The worker must be deployed with gpt3_5.grd beside it.
class Gpt3Vmf3Worker {
public:
    Gpt3Vmf3Worker();
    ~Gpt3Vmf3Worker();

    Gpt3Vmf3Worker(const Gpt3Vmf3Worker&) = delete;
    Gpt3Vmf3Worker& operator=(const Gpt3Vmf3Worker&) = delete;

    bool mapping(const QDateTime& utc, double latitudeRad, double longitudeRad,
                 double ellipsoidalHeightM, double zenithDistanceRad,
                 Gpt3Vmf3Mapping& result, QString* error = nullptr);
    void stop();

    static QString findWorker();

private:
    bool ensureStarted(QString* error);
    void setError(QString* error, const QString& value) const;

    std::unique_ptr<QProcess> _process;
    QString _workerPath;
};
