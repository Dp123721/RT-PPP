// Declares the main window, its presentation state, and runtime-service connections.
#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include "rtcmdecoder.h"
#include "ppp_engine.h"
#include "pppruntimeservice.h"
#include "rtcmprocessingservice.h"
#include "outputworker.h"
#include "settingsstore.h"
#include "streamclients.h"
#include "postppp.h"

#include <QDateTime>
#include <QJsonObject>
#include <QMainWindow>
#include <QMap>
#include <QVector>

QT_BEGIN_NAMESPACE
namespace Ui {
class MainWindow;
}
QT_END_NAMESPACE

class QCloseEvent;
class QCheckBox;
class QComboBox;
class QGraphicsScene;
class QGraphicsView;
class QLabel;
class QPlainTextEdit;
class QProgressBar;
class QPushButton;
class QTableWidget;
class QTabWidget;
class QLineEdit;
class QSpinBox;
class QDoubleSpinBox;
class QFileDialog;
class QFile;
class QRadioButton;
class QThread;
class QTimer;
class QWidget;

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(const QString& settingsFilePath = QString(), QWidget* parent = nullptr);
    ~MainWindow();

    // Starts the saved realtime stream configuration without making the window
    // visible. The Qt event loop remains required for socket, timer, PPP and
    // output-worker events.
    bool startHeadless(QString* error = nullptr);

private:
    struct StreamRow {
        QString id;
        QString type;
        QString sourceDisplay;
        QString decoder;
        QString lat;
        QString lon;
        QString nmea;
        QString stateText;
        quint64 bytes = 0;
        QJsonObject config;
        BaseStreamClient* client = nullptr;
    };

    void setupNtripSettingsUi();
    void setupDataPreparationUi();
    void setupStatusUi();
    void setupPppSettingsUi();
    void setupPppResultsUi();
    void setupOthersUi();
    void setupSignalBindings();
    void initializeOutputWorker();
    void shutdownOutputWorker();
    void finalizePppJsonOutput();
    void configureOutputWorker();
    void initializeRtcmProcessingService();
    void shutdownRtcmProcessingService();
    void configureRtcmProcessingService();
    void clearRtcmProcessingState();
    void drainRtcmProcessingResults();
    void initializePppWorker();
    void shutdownPppWorker();
    void startPppRuntime();
    void stopPppRuntimeAndClear();
    void postPppOptions();
    void postPppRuntimePolicy();
    void postPppStreamSelection();
    void requestPppWorkerStopNow();
    void drainPppWorkerResults();
    void resetRuntimeStateForFreshStart();

    void addNtripFromCasterDialog();
    void addTcpDialog();
    void addMqttDialog();
    void removeSelectedStreams();
    void startAllStreams();
    void stopAllStreams();
    void startPostProcessing();
    void cancelPostProcessing();
    void updatePostProcessingUi(bool running);
    void applyPostProductModeUi();
    void saveConfigNow();
    void reloadConfigNow();

    void appendLog(const QString& source, const QString& message);
    void appendDataPreview(const QString& source, qint64 bytes, const QByteArray& payload);
    void decodeRtcmPayload(const QString& source, const QByteArray& payload);
    void ingestDecodedRtcmMessage(
        const QString& source,
        const RtcmDecodedMessage& message,
        const RtcmProcessingService::MsmStatusUpdate* msmStatusUpdate);
    void ingestBroadcastEphemeris(const QString& source, const RtcmBroadcastEphemeris& eph);
    void ingestSsrMessage(const QString& source, const RtcmSsrMessage& ssr);
    void updateStatusDetailsForRow(int row);
    void refreshStatusPlotStationSelector();
    void rebuildStreamsTable();
    void updateTableRow(int row);
    void rebuildStatusTable();
    void updateStatusRow(int row);
    void rebuildDataPreparationStreamSelectors();
    void applyDataPreparationSelection(bool writeLog);
    void refreshDataPreparationTable();
    std::array<double, 3> receiverEcefForSource(const QString& source) const;
    QString streamLabelById(const QString& id) const;
    QString streamMountpointById(const QString& id) const;
    bool saveDataPreparationToSettings(QString* error = nullptr);
    void loadDataPreparationFromSettings();
    void applyPppSettings(bool writeLog);
    bool isPppArModeAvailable() const;
    void updatePppArAvailability();
    bool savePppSettingsToSettings(QString* error = nullptr);
    void loadPppSettingsFromSettings();
    void updatePppResults(const QString& source, const PppSolution& solution);
    void schedulePppDisplayRefresh();
    void scheduleSkyPlotRefresh();
    void refreshPppStationSelector();
    void refreshPppNeuPlot();
    void refreshZtdPlot();
    void refreshZwdPlot();
    void rebuildPppStationTable();
    bool isPppStationEnabled(const QString& sourceId) const;
    bool isStatusPlotStreamSource(const QString& sourceId) const;
    QString settingsFilePath() const;
    void writeZtdRecord(const QString& source, const QString& stationName, const PppSolution& solution);
    void flushPendingLogs();
    QString resolvedPrepLogFilePath() const;
    void setTelemetryOutputDirectory(const QString& directory);
    void postTelemetryMsm(const QString& sourceId,
                          const QDateTime& receivedUtc,
                          const QDateTime& epochUtc,
                          const RtcmMsmMessage& message,
                          const QMap<QString, QPair<double, double>>& skyBySatellite);
    void postTelemetrySsr(const QString& sourceId,
                          const QDateTime& receivedUtc,
                          const QDateTime& epochUtc,
                          const RtcmSsrMessage& message,
                          const RtcmSsrSatelliteData& satellite,
                          bool adopted,
                          const QString& reason);
    void postTelemetryReception(const QString& sourceId,
                                const QDateTime& receivedUtc,
                                const QString& messageFamily,
                                int messageType,
                                const QChar& system,
                                const QString& status,
                                const QString& reason,
                                const QString& detail);
    void postTelemetryPppResiduals(const QString& sourceId, const PppSolution& solution);
    void postOutputSp3(int gpsWeek,
                       double gpsSow,
                       const QString& prn,
                       const std::array<double, 3>& position,
                       double clockSec,
                       const std::array<double, 3>& velocity,
                       double clockRate);
    void postOutputRinex(int gpsWeek,
                         double gpsSow,
                         const QMap<QString, QMap<QString, double>>& observations);
    void postOutputPppJson(const QString& stationId,
                           const PppSolution& solution,
                           const std::array<double, 3>& referenceEcef);
    void postOutputZtd(const QString& source,
                       const QString& station,
                       const PppSolution& solution);
    void postOutputLog(const QString& text);
    void postOutputArLog(const PppSolution& solution);

    int selectedRow() const;
    QString selectedStatusPlotStreamId() const;
    int rowForStreamId(const QString& streamId) const;
    BaseStreamClient* createClientForRow(int row);
    void connectClientSignals(StreamRow& row);
    void startRow(int row);
    void stopRow(int row);
    void stopAndDeleteClient(StreamRow& row);

    bool loadStreamsFromSettings(QString* error = nullptr);
    bool saveStreamsToSettings(QString* error = nullptr);
    bool parseStreamArray(const QJsonArray& array, QVector<StreamRow>& rows,
                          QString* error = nullptr) const;
    QJsonObject rowToJson(const StreamRow& row) const;
    StreamRow rowFromJson(const QJsonObject& obj) const;

    void closeEvent(QCloseEvent* event) override;

    Ui::MainWindow* ui;
    SettingsStore _settingsStore;

    QVector<StreamRow> _rows;
    QMap<QString, QString> _lastStreamTelemetryState;
    int _idSeed = 1;
    bool _closing = false;

    QTableWidget* _streamsTable = nullptr;
    QPushButton* _addNtripButton = nullptr;
    QPushButton* _addTcpButton = nullptr;
    QPushButton* _addMqttButton = nullptr;
    QPushButton* _removeButton = nullptr;
    QPushButton* _globalStartButton = nullptr;
    QPushButton* _globalStopButton = nullptr;
    QPushButton* _saveConfigButton = nullptr;
    QPushButton* _reloadConfigButton = nullptr;

    QTableWidget* _statusTable = nullptr;
    QPlainTextEdit* _logTextEdit = nullptr;
    QLabel* _streamCountLabel = nullptr;
    QTabWidget* _statusDetailTabs = nullptr;
    QComboBox* _statusPlotStationCombo = nullptr;
    QLabel* _obsDetailLabel = nullptr;
    QTableWidget* _obsCountTable = nullptr;
    QGraphicsView* _obsTrendView = nullptr;
    QGraphicsScene* _obsTrendScene = nullptr;
    QGraphicsView* _snrPlotView = nullptr;
    QGraphicsScene* _snrPlotScene = nullptr;
    QGraphicsView* _skyPlotView = nullptr;
    QGraphicsScene* _skyPlotScene = nullptr;
    QComboBox* _skyHistoryWindowCombo = nullptr;
    QCheckBox* _prepEnableCheck = nullptr;
    QComboBox* _prepBroadcastCombo = nullptr;
    QComboBox* _prepOrbitCombo = nullptr;
    QComboBox* _prepClockCombo = nullptr;
    QComboBox* _prepCodeBiasCombo = nullptr;
    QComboBox* _prepPhaseBiasCombo = nullptr;
    QLabel* _prepSummaryLabel = nullptr;
    QTableWidget* _prepTable = nullptr;
    QComboBox* _prepObsCombo = nullptr;
    QComboBox* _obsSampleCombo = nullptr;
    QCheckBox* _sp3EnableCheck = nullptr;
    QLineEdit* _sp3PathEdit = nullptr;
    QSpinBox* _sp3SampleSpin = nullptr;
    QCheckBox* _rnxEnableCheck = nullptr;
    QLineEdit* _rnxPathEdit = nullptr;
    QCheckBox* _telemetryEnableCheck = nullptr;
    QLineEdit* _telemetryPathEdit = nullptr;
    QLineEdit* _prepLogPathEdit = nullptr;
    QPushButton* _prepLogBrowseButton = nullptr;
    QLabel* _pppResultLabel = nullptr;
    QLabel* _pppFixRateLabel = nullptr;
    QComboBox* _pppStationCombo = nullptr;
    QGraphicsView* _pppNeuView = nullptr;
    QGraphicsScene* _pppNeuScene = nullptr;
    QTableWidget* _pppStationTable = nullptr;
    QTableWidget* _pppResultTable = nullptr;
    QCheckBox* _pppUseGpsCheck = nullptr;
    QCheckBox* _pppUseGloCheck = nullptr;
    QCheckBox* _pppUseGalCheck = nullptr;
    QCheckBox* _pppUseBdsCheck = nullptr;
    QLineEdit* _pppSignalPrioritiesEdit = nullptr;
    QComboBox* _pppLcGpsCombo = nullptr;
    QComboBox* _pppLcGloCombo = nullptr;
    QComboBox* _pppLcGalCombo = nullptr;
    QComboBox* _pppLcBdsCombo = nullptr;
    QCheckBox* _pppEleWgtCodeCheck = nullptr;
    QCheckBox* _pppEleWgtPhaseCheck = nullptr;
    QCheckBox* _pppPseudoObsIonoCheck = nullptr;
    QLineEdit* _pppCrdPathEdit = nullptr;
    QPushButton* _pppCrdBrowseButton = nullptr;
    QCheckBox* _pppUseAntexCheck = nullptr;
    QLineEdit* _pppAntexPathEdit = nullptr;
    QPushButton* _pppAntexBrowseButton = nullptr;
    QLineEdit* _pppRoverAntennaEdit = nullptr;
    QCheckBox* _pppUseApcCheck = nullptr;
    QCheckBox* _pppUseOceanCheck = nullptr;
    QCheckBox* _pppArEnableCheck = nullptr;
    QComboBox* _pppArAlgorithmCombo = nullptr;
    QSpinBox* _pppArBieMinEpochsSpin = nullptr;
    QSpinBox* _pppArBieMinSatSpin = nullptr;
    QDoubleSpinBox* _pppArBieMaxFracSpin = nullptr;
    QDoubleSpinBox* _pppArBieMaxSigSpin = nullptr;
    QDoubleSpinBox* _pppArBieConstraintSigmaSpin = nullptr;
    QCheckBox* _pppArBiePersistConstraintsCheck = nullptr;
    QSpinBox* _pppArBiePersistConfirmEpochsSpin = nullptr;
    QDoubleSpinBox* _pppArBiePersistMaxVerticalSpin = nullptr;
    QDoubleSpinBox* _pppArBiePersistMaxZtdSpin = nullptr;
    QCheckBox* _pppArUseGpsCheck = nullptr;
    QCheckBox* _pppArUseGloCheck = nullptr;
    QCheckBox* _pppArUseGalCheck = nullptr;
    QCheckBox* _pppArUseBdsCheck = nullptr;
    QLineEdit* _pppBlqPathEdit = nullptr;
    QPushButton* _pppBlqBrowseButton = nullptr;
    QLineEdit* _pppOceanStationEdit = nullptr;
    QLineEdit* _pppVtecPathEdit = nullptr;
    QPushButton* _pppVtecBrowseButton = nullptr;
    QLineEdit* _pppArLogPathEdit = nullptr;
    QPushButton* _pppArLogBrowseButton = nullptr;
    QSpinBox* _pppMinObsSpin = nullptr;
    QDoubleSpinBox* _pppSigmaC1Spin = nullptr;
    QDoubleSpinBox* _pppSigmaL1Spin = nullptr;
    QDoubleSpinBox* _pppMaxResC1Spin = nullptr;
    QDoubleSpinBox* _pppMaxResL1Spin = nullptr;
    QDoubleSpinBox* _pppMinEleSpin = nullptr;
    QComboBox* _pppTropoModeCombo = nullptr;
    QComboBox* _pppGfsMethodCombo = nullptr;
    QComboBox* _pppMappingModeCombo = nullptr;
    QLineEdit* _pppGfsCachePathEdit = nullptr;
    QPushButton* _pppGfsCacheBrowseButton = nullptr;
    QDoubleSpinBox* _pppGfsTimeoutSpin = nullptr;
    QComboBox* _postModeCombo = nullptr;
    QWidget* _postInputPanel = nullptr;
    QWidget* _postSsrProductPanel = nullptr;
    QWidget* _postPreciseProductPanel = nullptr;
    QLineEdit* _postJobNameEdit = nullptr;
    QLineEdit* _postObsPathEdit = nullptr;
    QLineEdit* _postNavPathEdit = nullptr;
    QLineEdit* _postSp3PathEdit = nullptr;
    QLineEdit* _postClkPathEdit = nullptr;
    QLineEdit* _postSsrPathsEdit = nullptr;
    QComboBox* _postProductModeCombo = nullptr;
    QComboBox* _postSampleCombo = nullptr;
    QPushButton* _postStartButton = nullptr;
    QPushButton* _postCancelButton = nullptr;
    QProgressBar* _postProgress = nullptr;
    QLabel* _postStatusLabel = nullptr;
    QThread* _postThread = nullptr;
    PostPppService* _postService = nullptr;
    bool _postRunning = false;
    QString _postOutputDirectory;
    QDoubleSpinBox* _pppAprSigXSpin = nullptr;
    QDoubleSpinBox* _pppAprSigYSpin = nullptr;
    QDoubleSpinBox* _pppAprSigZSpin = nullptr;
    QDoubleSpinBox* _pppNoiseXSpin = nullptr;
    QDoubleSpinBox* _pppNoiseYSpin = nullptr;
    QDoubleSpinBox* _pppNoiseZSpin = nullptr;
    QDoubleSpinBox* _pppAprSigClkSpin = nullptr;
    QDoubleSpinBox* _pppAprSigTrpSpin = nullptr;
    QDoubleSpinBox* _pppNoiseTrpSpin = nullptr;
    QDoubleSpinBox* _pppAprSigAmbSpin = nullptr;
    QDoubleSpinBox* _pppSeedingSpin = nullptr;
    QDoubleSpinBox* _pppAprXSpin = nullptr;
    QDoubleSpinBox* _pppAprYSpin = nullptr;
    QDoubleSpinBox* _pppAprZSpin = nullptr;
    QDoubleSpinBox* _pppNeuNSpin = nullptr;
    QDoubleSpinBox* _pppNeuESpin = nullptr;
    QDoubleSpinBox* _pppNeuUSpin = nullptr;
    QDoubleSpinBox* _pppArMinEleSpin = nullptr;
    QSpinBox* _pppArMinMwSamplesSpin = nullptr;
    QSpinBox* _pppArMinFixPairsSpin = nullptr;
    QDoubleSpinBox* _pppArWlMaxDevSpin = nullptr;
    QDoubleSpinBox* _pppArWlMaxSigSpin = nullptr;
    QDoubleSpinBox* _pppArNlRatioSpin = nullptr;
    QDoubleSpinBox* _pppArNlBootSpin = nullptr;
    QDoubleSpinBox* _pppArNlMaxDevSpin = nullptr;
    QDoubleSpinBox* _pppArConstraintSigmaSpin = nullptr;
    QCheckBox* _pppArUseHistoryAidCheck = nullptr;
    QSpinBox* _pppArHistMinFixEpochsSpin = nullptr;
    QSpinBox* _pppArHistMinLockEpochsSpin = nullptr;
    QSpinBox* _pppArPartialWarmupLockEpochsSpin = nullptr;
    QDoubleSpinBox* _pppArHistRelaxFactorSpin = nullptr;
    QLineEdit* _pppZtdPathEdit = nullptr;
    QPushButton* _pppZtdPathBrowseButton = nullptr;
    QCheckBox* _pppJsonEnableCheck = nullptr;
    QLineEdit* _pppJsonPathEdit = nullptr;
    QPushButton* _pppJsonPathBrowseButton = nullptr;
    QComboBox* _ztdStationCombo = nullptr;
    QLabel* _ztdInfoLabel = nullptr;
    QGraphicsView* _ztdView = nullptr;
    QGraphicsScene* _ztdScene = nullptr;
    QGraphicsView* _zwdView = nullptr;
    QGraphicsScene* _zwdScene = nullptr;

    struct DataPreparationConfig {
        bool enabled = false;
        QString broadcastStreamId;
        QString orbitStreamId;
        QString clockStreamId;
        QString codeBiasStreamId;
        QString phaseBiasStreamId;
        QString obsStreamId;
        int observationSampleSec = 1;
        bool sp3Enabled = false;
        QString sp3Directory = "output/sp3";
        int sp3SampleSec = 10;
        bool rnxEnabled = false;
        QString rnxDirectory = "output/rinex";
        bool telemetryEnabled = false;
        QString telemetryDirectory = "output/telemetry";
        QString logOutputPath = "output/logs";
    };
    DataPreparationConfig _prepConfig;

    struct SatCountPoint {
        QDateTime utc;
        qint64 epochKey = 0;
        int satCount = 0;
    };
    struct SkyPlotPoint {
        QDateTime utc;
        QChar system;
        int prn = 0;
        double az = 0.0;
        double el = 0.0;
    };

    using MsmSystemSummary = RtcmProcessingService::MsmSystemSummary;
    using PreparedState = RtcmProcessingService::PreparedState;
    QMap<QString, QMap<QChar, MsmSystemSummary>> _msmSummaries;
    QMap<QString, QMap<QChar, QVector<SatCountPoint>>> _satCountHistory;
    QMap<QString, QVector<SkyPlotPoint>> _skyHistory;
    QMap<QString, qint64> _lastSkyHistoryEpochMs;
    QMap<QString, qint64> _lastSkyHistoryBucketMs;
    QMap<QString, QMap<QChar, QVector<SkyPlotPoint>>> _liveSkyPoints;
    QMap<QString, bool> _skyReceiverPositionValid;
    QMap<QString, PreparedState> _preparedStates;
    QMap<QString, QMap<QChar, RtcmMsmMessage>> _lastMsmMessages;
    bool _pppRuntimeActive = false;
    bool _logRawFrames = false;
    bool _logRtcmDetails = false;
    bool _loadingPppSettingsUi = false;
    QDateTime _lastRtcmSummaryLog;
    QDateTime _lastCorrSummaryLog;
    QDateTime _lastSp3Log;
    QDateTime _lastRnxLog;
    QDateTime _lastPppLog;
    QDateTime _lastPppYawLog;
    QDateTime _lastPppArLog;
    QString _lastPppSource;
    PppSolution _lastPppSolution;
    QMap<QString, PppSolution> _lastPppSolutionBySource;
    struct PppEpochRecord {
        QDateTime gpsTime;
        double neu[3] = {0.0, 0.0, 0.0};
        double neuStd[3] = {0.0, 0.0, 0.0};
        double xyzStd[3] = {0.0, 0.0, 0.0};
        bool hasNeu = false;
        bool hasStd = false;
        bool fixed = false;
        int fixedSatCount = 0;
        double ratio = 0.0;
        int numSat = 0;
        double hDop = 0.0;
        double ztd = 0.0;
        double ztdSigma = 0.0;
        bool hasZtd = false;
        double zwd = 0.0;
        double zwdPrior = 0.0;
        bool hasZwd = false;
        bool hasGfsZwdPrior = false;
    };
    QMap<QString, QString> _pppStationNameBySource;
    QMap<QString, QVector<PppEpochRecord>> _pppHistory;
    QMap<QString, bool> _pppStationEnabled;
    QString _ztdOutputDirectory = "output/ztd";
    bool _pppJsonEnabled = true;
    QString _pppJsonOutputDirectory = "output/ppp_json";
    int _pppJsonReorderBufferMs = 2000;
    int _pppJsonFlushIntervalMs = 1000;

    bool _telemetryEnabled = false;
    OutputWorker* _outputWorker = nullptr;
    QThread* _outputThread = nullptr;
    RtcmProcessingService* _rtcmProcessingService = nullptr;
    QThread* _rtcmProcessingThread = nullptr;
    PppOptions _pppOptions;
    PppWorker::RuntimePolicy _pppRuntimePolicy;
    PppStreamSelection _pppSelection;
    PppRuntimeService* _pppRuntimeService = nullptr;
    QTimer* _logFlushTimer = nullptr;
    QTimer* _pppDisplayRefreshTimer = nullptr;
    QTimer* _skyPlotRefreshTimer = nullptr;
    QStringList _pendingLogLines;
    QVector<PppWorker::SolutionEvent> _pendingPppEvents;
    QStringList _pendingPppLogs;
};

#endif // MAINWINDOW_H
