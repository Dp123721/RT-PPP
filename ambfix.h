// Declares the MW and narrow-lane LAMBDA ambiguity-resolution interface.
#pragma once

#include <QDateTime>
#include <QMap>
#include <QString>
#include <QStringList>
#include <QVector>

using Vec = QVector<double>;
using Mat = QVector<Vec>;

namespace AmbFix {

struct Options {
    double sigmaC1 = 1.0;
    double sigmaL1 = 0.01;
    bool useGlonass = false;
    double minEleDeg = 15.0;
    int minFixPairs = 2;
    int minMwSamples = 10;
    double wlMaxDev = 0.25;
    double wlMaxSig = 0.10;
    double nlRatio = 3.0;
    double nlBoot = -0.001;
    double nlMaxDev = 0.35;
    double constraintSigma = 0.01;
    bool enablePartialFix = true;
    bool useHistoryAid = true;
    int histMinFixEpochs = 20;
    int histMinLockEpochs = 200;
    int partialWarmupLockEpochs = 5;
    double histRelaxFactor = 1.1;
    // Reference selection is stateful at the engine boundary.  When the
    // previous reference is still a valid candidate, prefer it to avoid
    // changing the representation of an otherwise unchanged integer set.
    QMap<QChar, QString> preferredReferenceBySystem;
};

struct Sat {
    QString prn;
    QChar sys;
    double eleRad = 0.0;
    double C1 = 0.0;
    double C2 = 0.0;
    double L1 = 0.0;
    double L2 = 0.0;
    double lambda1 = 0.0;
    double lambda2 = 0.0;
    double f1 = 0.0;
    double f2 = 0.0;
    int sii1 = 0;
    int sii2 = 0;
    int wli1 = 0;
    int wli2 = 0;
    bool hasBias1 = false;
    bool hasBias2 = false;
    double codeBias1 = 0.0;
    double codeBias2 = 0.0;
    double phaseBias1 = 0.0;
    double phaseBias2 = 0.0;
    bool hasMwStat = false;
    double mw = 0.0;
    double mwSig = 0.0;
    int mwCount = 0;
    int ambIndex1 = -1;
    int ambIndex2 = -1;
    double ambCoeff1 = 0.0;
    double ambCoeff2 = 0.0;
    double ambMeters = 0.0;
    int lockEpochs = 0;
    int wlFixEpochs = 0;
    int nlFixEpochs = 0;
    bool newArc = false;
};

struct Constraint {
    QMap<int, double> coeff;
    double z = 0.0;
    QChar sys;
    QString refPrn;
    QString satPrn;
    double nlFloat = 0.0;
    double nlFixed = 0.0;
    int wlFixed = 0;
};

struct FixedSet {
    QChar sys;
    QString refPrn;
    // Reference-independent canonical integer relation identity.
    QString signature;
    int wlCount = 0;
    int nlCount = 0;
    double ratio = 0.0;
    double ratioRaw = 0.0;
    double boot = 0.0;
    double maxNlDev = 0.0;
    double qddMin = 0.0;
    double qddMax = 0.0;
    double qddAvg = 0.0;
    QString worstDd;
    double worstNlDev = 0.0;
    bool partial = false;
    int histQualifiedWlDdCount = 0;
    int histQualifiedNlDdCount = 0;
    int warmupDroppedCount = 0;
    QStringList fixedDds;
    QStringList droppedDds;
    QStringList fixedPrns;
};

struct Result {
    QString summary;
    QStringList logLines;
    QVector<Constraint> constraints;
    QVector<FixedSet> fixedSets;
};

Result run(const QDateTime& utc, const QVector<Sat>& sats, const Mat& Q, const Options& opt);

} // namespace AmbFix
