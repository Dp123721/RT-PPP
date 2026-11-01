// Declares PPP options, solution records, and the stateful PppEngine interface.
#pragma once

#include <QDateTime>
#include <QMap>
#include <QSet>
#include <QString>
#include <QStringList>
#include <QVector>
#include <QtGlobal>
#include <array>
#include <cmath>
#include <map>
#include <memory>
#include <limits>

#include "gnss_ephemeris.h"
#include "rtcmdecoder.h"

class PppFilter;
struct PppEpochData;

// The estimated TROPO state is always a full zenith wet delay (ZWD).  The
// mode controls only the immutable a-priori value used when a filter starts.
enum class PppTroposphereMode {
    Original,
    GfsZwdInitialization
};

enum class GfsZwdMethod {
    ProfileIntegral,
    AskneNordius
};

enum class PppMappingMode {
    SimpleSin,
    Gpt3Vmf3
};

struct PppTropospherePrior {
    bool valid = false;
    double zwdM = 0.0;
    QString source;
    QString status;
    QString reason;
    QDateTime productTime;
    QString cycle;
    int forecastLeadSec = -1;
    QString productSelection;
    bool fallbackUsed = false;
    bool spatialInterpolated = false;
    bool temporalInterpolated = false;
    bool heightExtrapolated = false;
};

struct PppCrdStationOptions {
    bool valid = false;
    QString station;
    double aprXyz[3] = {0.0, 0.0, 0.0};
    double neuEcc[3] = {0.0, 0.0, 0.0};
    QString roverAntenna;
    QString receiver;
};

struct PppOptions {
    double sigmaC1 = 1.0;
    double sigmaL1 = 0.01;
    double maxResC1 = 3.0;
    double maxResL1 = 0.03;
    double minEleRad = 10.0 * 3.14159265358979323846 / 180.0;
    int minObs = 4;
    bool eleWgtCode = true;
    bool eleWgtPhase = true;
    double aprSigCrd[3] = {100.0, 100.0, 100.0};
    double noiseCrd[3] = {0.01, 0.01, 0.01};
    double aprSigClk = 3.0e5;
    double aprSigTrp = 0.3;
    double noiseTrp = 0.01;
    double aprSigAmb = 1.0e4;
    double aprSigIon = 1.0e6;
    double noiseIon = 1.0e6;
    double aprSigCodeBias = 10000.0;
    double noiseCodeBias = 10000.0;
    double aprSigPhaseBias = 10000.0;
    double noisePhaseBias = 10000.0;
    double seedingTime = 0.0;
    double aprXyz[3] = {0.0, 0.0, 0.0};
    double neuEcc[3] = {0.0, 0.0, 0.0};
    QString crdFile;
    QMap<QString, PppCrdStationOptions> crdStationsBySource;
    bool useAntex = false;
    QString antexFile;
    QString roverAntenna;
    bool isApc = true;
    bool useOceanTide = false;
    QString blqFile;
    QString oceanStation;
    QString vtecFile;
    bool logSsrYawDetails = true;
    bool logBiasMatchDetails = true;
    bool useGps = true;
    bool useGlo = true;
    bool useGal = true;
    bool useBds = true;
    bool strictCorrectionSource = true;
    QString signalPriorities = "G:12&CWPSLX R:12&CP E:1&CBX E:5&QIX C:26&IQX";
    QString lcGps = "P3&L3";
    QString lcGlo = "P3&L3";
    QString lcGal = "P3&L3";
    QString lcBds = "P3&L3";
    bool pseudoObsIono = false;
    bool estimateTroposphere = true;
    PppTroposphereMode troposphereMode = PppTroposphereMode::Original;
    GfsZwdMethod gfsZwdMethod = GfsZwdMethod::ProfileIntegral;
    PppMappingMode mappingMode = PppMappingMode::SimpleSin;
    QString gfsCacheDirectory = QStringLiteral("output/gfs");
    QString gfsDownloadBaseUrl;
    int gfsPrefetchTimeoutSec = 90;
    int gfsMaxAgeSec = 4 * 3600;
    bool estimateIonosphere = true;
    bool pppRtk = false;
    double sigmaGIM = 1.0;
    double maxResGIM = 3.0;
    bool enablePppAr = false;
    QString arAlgorithm = "LAMBDA";
    bool arUseGps = true;
    bool arUseGlonass = false;
    bool arUseGalileo = true;
    bool arUseBds = true;
    double arMinEleDeg = 15.0;
    int arMinMwSamples = 10;
    int arMinFixPairs = 2;
    double arWlMaxDev = 0.25;
    double arWlMaxSig = 0.10;
    double arNlRatio = 3.0;
    double arNlBoot = -0.001;
    double arNlMaxDev = 0.35;
    double arConstraintSigma = 0.01;
    int arBieMinEpochs = 10;
    int arBieMinSat = 4;
    double arBieMaxFrac = 0.01;
    double arBieMaxSig = 0.001;
    double arBieConstraintSigma = 1.0e-4;
    // Disabled by default: BIE produces an epoch-local fixed copy. When enabled,
    // posterior updates persist only after stable-batch and state-transition checks.
    bool arBiePersistConstraints = false;
    int arBiePersistConfirmEpochs = 3;
    double arBiePersistMaxVerticalM = 0.10;
    double arBiePersistMaxZtdM = 0.03;
    bool arUseHistoryAid = true;
    int arHistMinFixEpochs = 20;
    int arHistMinLockEpochs = 200;
    int arPartialWarmupLockEpochs = 5;
    double arHistRelaxFactor = 1.1;
    QString arLogFile;

    bool useSystem(QChar system) const {
        switch (system.toLatin1()) {
        case 'G': return useGps;
        case 'R': return useGlo;
        case 'E': return useGal;
        case 'C': return useBds;
        default: break;
        }
        return false;
    }

    bool hasApriori() const {
        return std::fabs(aprXyz[0]) > 1e-6 ||
               std::fabs(aprXyz[1]) > 1e-6 ||
               std::fabs(aprXyz[2]) > 1e-6;
    }

    bool arUseSystem(QChar system) const {
        switch (system.toUpper().unicode()) {
        case 'G': return arUseGps;
        case 'R': return arUseGlonass;
        case 'E': return arUseGalileo;
        case 'C': return arUseBds;
        default: return false;
        }
    }
};

struct PppStreamSelection {
    QString broadcastId;
    QString orbitId;
    QString clockId;
    QString codeBiasId;
    QString phaseBiasId;
    QString obsId;
};

struct PppVtecLayer {
    double heightMeters = 0.0;
    QVector<QVector<double>> cosine;
    QVector<QVector<double>> sine;
};

struct PppVtecModel {
    QDateTime utc;
    int gpsWeek = 0;
    double gpsSow = 0.0;
    int updateIntervalIndex = -1;
    QString stationId;
    QVector<PppVtecLayer> layers;

    bool isValid() const {
        return !layers.isEmpty();
    }
};

struct PppArGroupResult {
    QChar system;
    QString group;
    QString referenceSatellite;
    int fixedCount = 0;
    int eligibleCount = 0;
    double maxFractionCycle = 0.0;
    double maxSigmaCycle = 0.0;
};

struct PppObservationResidual {
    QString satellite;
    QString signal;
    QString observationType;
    double prefitM = 0.0;
    double postfitM = 0.0;
    double sigmaM = 0.0;
    double elevationDeg = 0.0;
    bool adopted = false;
    QString reason;
};

struct PppSolution {
    bool valid = false;
    QDateTime utc;
    // GPS calendar time used by PPP result presentation and export only.
    QDateTime gpsTime;
    int gpsWeek = 0;
    double gpsSow = 0.0;
    double xyz[3] = {0.0, 0.0, 0.0};
    double neu[3] = {0.0, 0.0, 0.0};
    double cov[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    double trp = 0.0;
    double trpStdev = 0.0;
    double ztd = 0.0;
    double zhdModel = 0.0;
    double zwdPrior = 0.0;
    QString tropoPriorSource;
    QString tropoPriorStatus;
    bool gfsZwdPriorValid = false;
    // Kept for existing writers and integrations. It aliases zhdModel.
    double ztdModel = 0.0;
    double ztdStdev = 0.0;
    int numSat = 0;
    double hDop = 0.0;
    bool arEnabled = false;
    QString arAlgorithm;
    QString arStatus;
    int arFixedSatCount = 0;
    int arTotalSatCount = 0;
    double arRatio = 0.0;
    double arBootstrap = 0.0;
    int arCandidateCount = 0;
    double arEffectiveCandidates = 0.0;
    int arConstraintsApplied = 0;
    QStringList usedSatellites;
    QStringList arEligibleSatellites;
    QStringList arFixedSatellites;
    QVector<PppArGroupResult> arGroups;
    QVector<PppObservationResidual> observationResiduals;
    QString diagSummary;
    QString log;
    QStringList arLogLines;
};

// Per-epoch satellite lifecycle counters. Post-processing aggregates these
// diagnostics to expose why a constellation did not reach the PPP filter.
struct PppSatelliteStageDiagnostic {
    int observationAvailable = 0;
    int broadcastMatched = 0;
    int ssrMatched = 0;
    int iodRejected = 0;
    int residualRejected = 0;
    int acceptedAfterResidual = 0;
};

struct PppPreciseSatelliteState {
    RtcmSatId sat;
    int gpsWeek = 0;
    double gpsSow = 0.0;
    std::array<double, 3> position = {0.0, 0.0, 0.0};
    std::array<double, 3> velocity = {0.0, 0.0, 0.0};
    double clockSec = 0.0;
    bool hasVelocity = false;
    bool hasClock = false;
};

// Common observation representation used by both the realtime MSM adapter
// and offline RINEX adapters. Values are meters except wavelengths/frequency
// related fields supplied by the adapter.
struct PppStandardizedObservation {
    QString satKey;
    double c1 = 0.0;
    double c2 = 0.0;
    double l1 = 0.0;
    double l2 = 0.0;
    bool hasC1 = false;
    bool hasC2 = false;
    bool hasL1 = false;
    bool hasL2 = false;
    QString signal1;
    QString signal2;
    QChar b1;
    QChar b2;
    int gloChannel = 0;
    bool slip = false;
    bool useSsrYaw = false;
    double yawRad = 0.0;
    int sii1 = 0;
    int sii2 = 0;
    int wli1 = 0;
    int wli2 = 0;
    int lock1 = -1;
    int lock2 = -1;
    int disc1 = -1;
    int disc2 = -1;
    bool hasCodeBias1 = false;
    bool hasCodeBias2 = false;
    bool hasBias1 = false;
    bool hasBias2 = false;
    double codeBias1 = 0.0;
    double codeBias2 = 0.0;
    double phaseBias1 = 0.0;
    double phaseBias2 = 0.0;
    bool phaseStateSeen = false;
    QString phaseStateSource;
    int phaseStateMessageType = 0;
    int phaseStateEpoch = -1;
    int phaseStateSignalCount = 0;
    int score1 = std::numeric_limits<int>::max();
    int score2 = std::numeric_limits<int>::max();
};

struct PppStandardizedEpoch {
    int gpsWeek = 0;
    double gpsSow = 0.0;
    QVector<PppStandardizedObservation> observations;
};

class PppEngine
{
public:
    struct SsrYawDebug {
        QString prn;
        QString sourceId;
        int messageType = 0;
        int updateIntervalIndex = -1;
        double epochSow = 0.0;
        double recvSow = 0.0;
        double dtRawSec = 0.0;
        double uiSec = 0.0;
        double dtSec = 0.0;
        double yawRad = 0.0;
        double yawRateRad = 0.0;
    };

    PppEngine();
    ~PppEngine();

    void setOptions(const PppOptions& options);
    PppOptions& options();

    void setStreamSelection(const PppStreamSelection& selection);
    void resetProcessingState();
    void updateVtecModel(const PppVtecModel& vtec);
    void clearVtecModel();

    void updateBroadcast(const RtcmBroadcastEphemeris& eph,
                         const std::shared_ptr<GnssEphemeris>& model,
                         const QString& sourceId,
                         const QDateTime& receivedUtc);
    void beginBiasEpoch(const QString& sourceId, const RtcmSsrMessage& ssr);
    void updateSsrMessage(const QString& sourceId, const RtcmSsrMessage& ssr);
    void updatePreciseState(const PppPreciseSatelliteState& state);
    void clearPreciseStates();
    // Must be called before the first accepted epoch for a source. Existing
    // filters are intentionally left unchanged so a real-time update cannot
    // redefine a live Kalman state.
    void setTropospherePrior(const QString& sourceId, const PppTropospherePrior& prior);
    bool lastBancroftPosition(const QString& sourceId, std::array<double, 3>& xyz) const;
    void discardGfsSeedEpoch(const QString& sourceId, int gpsWeek, double gpsSow);

    bool processMsm(const RtcmMsmMessage& msm,
                    const QString& sourceId,
                    const QMap<int, int>& gloChannels,
                    PppSolution& solution);

    bool processStandardizedEpoch(const PppStandardizedEpoch& epoch,
                                  const QString& sourceId,
                                  const QMap<int, int>& gloChannels,
                                  PppSolution& solution);

    const PppSolution& lastSolution() const;
    const QString& lastFailureReason() const { return _lastFailureReason; }
    const QMap<QString, PppSatelliteStageDiagnostic>& lastSatelliteDiagnostics() const {
        return _lastSatelliteDiagnostics;
    }
    const QVector<SsrYawDebug>& lastYawDebug() const { return _lastYawDebug; }
    const QVector<QString>& lastAntexWarnings() const { return _lastAntexWarnings; }

private:
    bool processMsmInternal(const RtcmMsmMessage* msm,
                            const QString& sourceId,
                            const QMap<int, int>& gloChannels,
                            const PppStandardizedEpoch* standardizedEpoch,
                            PppSolution& solution);

    struct OrbitCorrectionState {
        RtcmSatId sat;
        QString sourceId;
        int iod = -1;
        int updateIntervalIndex = -1;
        int satRefDatum = -1;
        int epochTime = -1;
        bool epochTimeIsGps = false;
        int messageType = 0;
        double xr[3] = {0.0, 0.0, 0.0};
        double dotXr[3] = {0.0, 0.0, 0.0};
        QDateTime receivedUtc;
    };
    struct ClockCorrectionState {
        RtcmSatId sat;
        QString sourceId;
        int iod = -1;
        int updateIntervalIndex = -1;
        int epochTime = -1;
        bool epochTimeIsGps = false;
        int messageType = 0;
        double dClk = 0.0;
        double dotDClk = 0.0;
        double dotDotDClk = 0.0;
        QDateTime receivedUtc;
    };
    struct BroadcastState {
        RtcmBroadcastEphemeris eph;
        std::shared_ptr<GnssEphemeris> model;
        QString sourceId;
        QDateTime receivedUtc;
        bool hasOrbitCorrection = false;
        bool hasClockCorrection = false;
        OrbitCorrectionState orbitCorrection;
        ClockCorrectionState clockCorrection;
    };
    struct HighRateClockState {
        RtcmSatId sat;
        QString sourceId;
        int updateIntervalIndex = -1;
        int epochTime = -1;
        bool epochTimeIsGps = false;
        int messageType = 0;
        double dClk = 0.0;
        QDateTime receivedUtc;
    };
    struct CodeBiasState {
        RtcmSatId sat;
        QString sourceId;
        int updateIntervalIndex = -1;
        int epochTime = -1;
        bool epochTimeIsGps = false;
        int messageType = 0;
        QVector<RtcmSsrCodeBiasEntry> biases;
        QDateTime receivedUtc;
    };
    struct PhaseBiasState {
        RtcmSatId sat;
        QString sourceId;
        int updateIntervalIndex = -1;
        int epochTime = -1;
        bool epochTimeIsGps = false;
        int messageType = 0;
        int dispersiveConsistency = -1;
        int mwConsistency = -1;
        double yawRad = 0.0;
        double yawRateRadPerSec = 0.0;
        QVector<RtcmSsrPhaseBiasEntry> biases;
        QDateTime receivedUtc;
    };

    PppOptions _options;
    PppStreamSelection _selection;
    QMap<QString, BroadcastState> _broadcastStates;
    QMap<QString, QVector<BroadcastState>> _broadcastStateQueues;
    QMap<QString, PppPreciseSatelliteState> _preciseStates;
    QMap<QString, OrbitCorrectionState> _orbitCorrections;
    QMap<QString, ClockCorrectionState> _clockCorrections;
    QMap<QString, OrbitCorrectionState> _orbitCorrectionsByIod;
    QMap<QString, ClockCorrectionState> _clockCorrectionsByIod;
    QMap<QString, HighRateClockState> _highRateClockCorrections;
    QMap<QString, int> _lastOrbitIodBySourceSat;
    QMap<QString, CodeBiasState> _codeBiases;
    QMap<QString, PhaseBiasState> _phaseBiases;
    QSet<QString> _biasMatchDiagKeys;
    PppVtecModel _activeVtecModel;
    PppVtecModel _latestVtecModel;
    QMap<qint64, PppVtecModel> _vtecModelsByTime;
    QString _loadedVtecFile;
    struct PendingMsmEpoch {
        bool valid = false;
        int gpsWeek = 0;
        double gpsSow = 0.0;
        qint64 epochMsKey = 0;
        RtcmMsmMessage msm;
    };
    struct PendingMsmGroup {
        bool valid = false;
        QString sourceId;
        int gpsWeek = 0;
        double gpsSow = 0.0;
        qint64 epochMsKey = 0;
        QDateTime receivedUtc;
        QMap<QChar, PendingMsmEpoch> msmBySystem;
        QMap<int, int> gloChannels;
    };
    struct MsmAssembly {
        bool ready = false;
        bool directMode = false;
        QString systemEpochKey;
        int gpsWeek = 0;
        double gpsSow = 0.0;
        qint64 epochMsKey = 0;
        QVector<PendingMsmEpoch> epochs;
        QMap<int, int> gloChannels;
    };
    struct PairObs {
        double c1 = 0.0;
        double c2 = 0.0;
        double l1 = 0.0;
        double l2 = 0.0;
        bool hasC1 = false;
        bool hasC2 = false;
        bool hasL1 = false;
        bool hasL2 = false;
        QString signal1;
        QString signal2;
        QChar b1;
        QChar b2;
        int gloChannel = 0;
        bool slip = false;
        bool useSsrYaw = false;
        double yawRad = 0.0;
        int sii1 = 0;
        int sii2 = 0;
        int wli1 = 0;
        int wli2 = 0;
        int lock1 = -1;
        int lock2 = -1;
        int disc1 = -1;
        int disc2 = -1;
        bool hasCodeBias1 = false;
        bool hasCodeBias2 = false;
        bool hasBias1 = false;
        bool hasBias2 = false;
        double codeBias1 = 0.0;
        double codeBias2 = 0.0;
        double phaseBias1 = 0.0;
        double phaseBias2 = 0.0;
        int gpsWeek = 0;
        double gpsSow = 0.0;
        bool gpsTimeValid = false;
        bool phaseStateSeen = false;
        QString phaseStateSource;
        int phaseStateMessageType = 0;
        int phaseStateEpoch = -1;
        int phaseStateSignalCount = 0;
        int score1 = std::numeric_limits<int>::max();
        int score2 = std::numeric_limits<int>::max();
    };
    struct PairObservationBuild {
        QMap<QString, PairObs> observations;
        int skippedStaleCodeBias = 0;
        int skippedStalePhaseBias = 0;
        QStringList staleBiasExamples;
    };
    struct EpochBuildDiagnostics {
        int skippedStaleCodeBias = 0;
        int skippedStalePhaseBias = 0;
        int skippedStaleOrbit = 0;
        int skippedStaleClock = 0;
        int skippedNoC1 = 0;
        int skippedNoBroadcast = 0;
        int skippedBroadcastOutdated = 0;
        int skippedNoLambda = 0;
        int skippedNoOrbit = 0;
        int skippedOrbitIod = 0;
        int skippedOrbitStale = 0;
        int skippedNoClock = 0;
        int skippedClockIod = 0;
        int skippedClockStale = 0;
        int skippedSatState = 0;
        int skippedIncompleteBias = 0;
        QStringList staleBiasExamples;
    };
    enum class CorrectionRole {
        Orbit,
        Clock,
        Bias
    };
    MsmAssembly prepareMsmEpochs(const RtcmMsmMessage* msm,
                                 const QString& sourceId,
                                 const QMap<int, int>& gloChannels,
                                 const PppStandardizedEpoch* standardizedEpoch);
    PairObservationBuild buildPairObservations(const MsmAssembly& assembly,
                                               const QString& sourceId,
                                               const PppStandardizedEpoch* standardizedEpoch);
    bool buildEpochData(const QString& sourceId,
                        const MsmAssembly& assembly,
                        const PairObservationBuild& pairBuild,
                        PppEpochData& epochData,
                        EpochBuildDiagnostics& diagnostics,
                        QString& failureReason);
    QStringList correctionSourceCandidates(const QString& selectedSourceId,
                                           const QString& observationSourceId,
                                           CorrectionRole role) const;
    bool correctionIsFresh(QChar system, int epochTime, bool epochTimeIsGps,
                           int updateIntervalIndex, int gpsWeek, double gpsSow) const;
    PppSolution _lastSolution;
    QMap<QString, int> _lastAcceptedSatCountBySource;
    QString _lastFailureReason;
    QMap<QString, PppSatelliteStageDiagnostic> _lastSatelliteDiagnostics;
    QVector<SsrYawDebug> _lastYawDebug;
    QVector<QString> _lastAntexWarnings;
    QMap<QString, PppTropospherePrior> _tropospherePriorsBySource;
    std::map<QString, std::unique_ptr<PppFilter>> _filters;
    QMap<QString, PendingMsmEpoch> _pendingMsmBySystem;
    QMap<QString, PendingMsmGroup> _pendingMsmByEpoch;
    QMap<QString, qint64> _lastProcessedEpochPerSystem;
    QMap<QString, qint64> _lastClosedEpochPerSource;
    QMap<QString, qint64> _lastOutputEpochPerSource;
    std::map<QString, PppOptions> _sourceOptions;
};
