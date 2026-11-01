// Declares BIE ambiguity grouping, candidate search results, and constraints.
#pragma once

#include <QChar>
#include <QMap>
#include <QString>
#include <QStringList>
#include <QVector>

namespace BieAlgorithm {

using Vec = QVector<double>;
using Mat = QVector<Vec>;

struct Options {
    double minElevationDeg = 10.0;
    int minNumEpochs = 10;
    int minNumSat = 4;
    double maxFrac = 0.01;
    double maxSigma = 0.001;
    double constraintSigma = 1.0e-4;
    int candidateCount = 100;
    // Group key is system + frequency group, for example "G|L1".
    QMap<QString, QString> preferredReferenceByGroup;
};

struct Ambiguity {
    QString prn;
    QChar system;
    QString group;
    int stateIndex = -1;
    double value = 0.0;
    double elevationDeg = 0.0;
    int numEpochs = 0;
};

struct Constraint {
    QChar system;
    QString group;
    QString prn;
    QString referencePrn;
    int stateIndex = -1;
    // Index into Result::posteriorMean/posteriorCovariance. This keeps the
    // selected BIE posterior aligned with the filter ambiguity states.
    int posteriorIndex = -1;
    double value = 0.0;
    double sigma = 0.0;
};

struct FixedGroup {
    QChar system;
    QString group;
    QString referencePrn;
    int ambiguityCount = 0;
    int fixedCount = 0;
    double maxFraction = 0.0;
    double maxSigma = 0.0;
};

struct Result {
    QVector<Constraint> constraints;
    QVector<FixedGroup> fixedGroups;
    // BIE posterior for exactly the ambiguity states represented by
    // constraints, in the same order. Empty when no constraint is accepted.
    QVector<int> posteriorStateIndices;
    Vec posteriorMean;
    Mat posteriorCovariance;
    int candidateCount = 0;
    double effectiveCandidates = 0.0;
    QString summary;
    QStringList logLines;
};

bool search(const Vec& ambiguities, const Mat& covariance,
            Vec& bie, Mat& bieCovariance, int candidateCount = 100);
Result run(const QVector<Ambiguity>& ambiguities, const Mat& covariance, const Options& options);

} // namespace BieAlgorithm
