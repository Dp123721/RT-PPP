# Checks source ownership, thread boundaries, and runtime output contracts.
param()

$ErrorActionPreference = 'Stop'
$repoRoot = Split-Path -Parent $PSScriptRoot

function Read-RepoFile {
    param([string]$RelativePath)
    return Get-Content -Raw -LiteralPath (Join-Path $repoRoot $RelativePath)
}

function Assert-Contains {
    param([string]$Text, [string]$Pattern, [string]$Message)
    if ($Text -notmatch $Pattern) {
        throw $Message
    }
}

function Assert-NotContains {
    param([string]$Text, [string]$Pattern, [string]$Message)
    if ($Text -match $Pattern) {
        throw $Message
    }
}

$mainWindow = (Read-RepoFile 'mainwindow.h') + "`n" + (Read-RepoFile 'mainwindow.cpp')
$rtcmService = (Read-RepoFile 'rtcmprocessingservice.h') + "`n" +
               (Read-RepoFile 'rtcmprocessingservice.cpp')
$pppRuntime = (Read-RepoFile 'pppruntimeservice.h') + "`n" +
              (Read-RepoFile 'pppruntimeservice.cpp')
$pppWorker = Read-RepoFile 'ppp_worker.cpp'
$pppEngine = (Read-RepoFile 'ppp_engine.h') + "`n" + (Read-RepoFile 'ppp_engine.cpp')
$outputWorker = (Read-RepoFile 'outputworker.h') + "`n" +
                (Read-RepoFile 'outputworker.cpp')
$mainEntry = Read-RepoFile 'main.cpp'
$vcxProject = Read-RepoFile 'RT-PPP.vcxproj'

$oldMainWindowState = @(
    '_broadcastStates', '_orbitCorrections', '_clockCorrections',
    '_codeBiases', '_phaseBiases', '_gloMsmChannels',
    'recomputePreparedStates', 'writeSp3ForPreparedStates',
    'writeCorrectedRinexObs', 'computeMsmSky', 'classifySsrAdoption'
) -join '|'
Assert-NotContains $mainWindow $oldMainWindowState `
    'MainWindow regained RTCM preparation state or calculations.'

Assert-Contains $rtcmService 'Rtcm3Decoder' `
    'RTCM decoder ownership is missing from RtcmProcessingService.'
Assert-Contains $rtcmService 'computeCorrectedObservations' `
    'Corrected observation preparation is missing from RtcmProcessingService.'
Assert-Contains $rtcmService 'computeSky' `
    'Sky-coordinate calculation is missing from RtcmProcessingService.'
Assert-Contains $rtcmService '_glonassChannelsBySource' `
    'GLONASS channel state must remain isolated by source.'

Assert-Contains $pppRuntime 'stopAndClear' `
    'PPP lifecycle is not centralized in PppRuntimeService.'
Assert-Contains $pppRuntime 'Qt::BlockingQueuedConnection' `
    'PPP lifecycle configuration must preserve synchronous ordering.'
Assert-Contains $mainWindow 'const QSignalBlocker blocker\(combo\);' `
    'Restoring data-preparation stream selectors must not emit auto-apply signals.'
Assert-Contains $mainWindow 'const QSignalBlocker blocker\(_telemetryEnableCheck\);' `
    'Restoring output checkboxes must not trigger a partial configuration save.'
Assert-Contains $mainWindow 'bool startHeadless' `
    'MainWindow must expose the shared headless realtime startup path.'
Assert-Contains $mainWindow 'Headless mode currently supports saved realtime streams only' `
    'Headless mode must reject the GUI-only post-processing startup path.'
Assert-Contains $mainEntry 'headless' `
    'Application entry point is missing the headless command-line option.'
Assert-Contains $mainEntry 'QT_QPA_PLATFORM' `
    'Headless startup must select an offscreen Qt platform before QApplication.'
Assert-Contains $mainEntry 'SIGTERM' `
    'Headless runtime must handle service termination signals.'
Assert-Contains $mainEntry 'startHeadless' `
    'Application entry point must activate the shared headless startup path.'
Assert-Contains (Read-RepoFile 'tests/headless_smoke.ps1') '--headless' `
    'Headless startup must have an offline smoke probe.'
Assert-Contains (Read-RepoFile 'deploy/rtppp.service') '--headless' `
    'Linux deployment unit must use the headless startup path.'

Assert-Contains $pppWorker 'GnssUtils::gpsTimeFromMsm' `
    'PppWorker must use the common MSM time conversion.'
Assert-Contains $pppWorker 'GnssUtils::gpsTimeFromSsr' `
    'PppWorker must use the common SSR time conversion.'
Assert-Contains $pppWorker 'GnssUtils::isGpsTimeOnSampleGrid' `
    'PppWorker must enforce the configured processing epoch interval.'
Assert-NotContains $mainWindow 'crdStationsBySource\.insert\(row\.id,\s*station\)' `
    'NTRIP stream coordinates must not be installed as PPP apriori coordinates.'
Assert-Contains $pppEngine '_baseXyz = _opt->hasApriori\(\).*?: p;' `
    'Without a configured coordinate, PPP must initialize its position base from Bancroft.'
Assert-Contains $pppWorker 'lastBancroftPosition\(oldest\.sourceId, bancroftXyz\)' `
    'GFS initialization without configured coordinates must use the Bancroft position seed.'

Assert-Contains $pppEngine 'arBiePersistConstraints = false' `
    'BIE constraint persistence must default to compatible epoch-local behavior.'
Assert-Contains $pppEngine 'if \(!_opt->arBiePersistConstraints\)' `
    'BIE persistence must retain accepted constraints when explicitly enabled.'
Assert-Contains $pppEngine 'calculateBiePosteriorUpdate' `
    'Persistent BIE must use the returned posterior mean and covariance.'
Assert-Contains $pppEngine 'posterior_moment_match' `
    'Persistent BIE must log its posterior moment-matching update.'
Assert-Contains $pppEngine 'BIE PERSISTENCE status=pending' `
    'Persistent BIE must confirm an unchanged batch before committing it.'
Assert-Contains $pppEngine 'transition_gate' `
    'Persistent BIE must reject excessive coordinate or ZTD transitions.'
Assert-Contains $pppEngine 'clearBiePersistenceForPrn' `
    'Persistent BIE hypotheses must clear on ambiguity-arc reset.'
Assert-Contains $pppEngine 'BiePersistentHypothesis' `
    'Persistent BIE must retain continuity by satellite/frequency phase arc.'
Assert-Contains $pppEngine 'advanceBiePersistence' `
    'Persistent BIE must evaluate per-arc continuity before committing.'
Assert-Contains $pppEngine 'status=ready stable=' `
    'Persistent BIE must require a stable subset within each frequency group.'
Assert-Contains $pppEngine 'posterior_covariance_not_tighter_than_float' `
    'Persistent BIE must reject a posterior that does not reduce covariance.'
Assert-Contains $pppEngine 'covarianceLoading=%6' `
    'Persistent BIE must expose any bounded covariance regularization.'
Assert-NotContains $pppEngine 'BiePersistentTracker|candidateSignature|committedSignature' `
    'Persistent BIE must not require a global all-satellite batch signature.'
Assert-Contains $pppEngine 'state=%3' `
    'BIE AR logging must distinguish committed filter constraints from epoch-local output.'
Assert-Contains $mainWindow 'Persist BIE constraints to next epoch' `
    'The BIE persistence option must be exposed in the PPP settings UI.'
Assert-Contains $mainWindow 'ppp/arBiePersistConstraints' `
    'The BIE persistence option must be persisted in PPP settings.'
Assert-Contains $mainWindow 'ppp/arBiePersistConfirmEpochs' `
    'BIE persistent-batch confirmation must be persisted in PPP settings.'
Assert-Contains $mainWindow 'ppp/arBiePersistMaxVerticalM' `
    'BIE vertical transition screening must be persisted in PPP settings.'
Assert-Contains $mainWindow 'ppp/arBiePersistMaxZtdM' `
    'BIE ZTD transition screening must be persisted in PPP settings.'
Assert-Contains $mainWindow 's\.setValue\("ppp/logBiasMatchDetails"' `
    'PPP bias-match diagnostics must be written to the configuration.'
Assert-Contains $mainWindow 's\.value\("ppp/logBiasMatchDetails"' `
    'PPP bias-match diagnostics must be restored from the configuration.'
Assert-Contains $mainWindow 's\.setValue\("ppp/jsonReorderBufferMs"' `
    'PPP JSON writer buffering must be written to the configuration.'
Assert-Contains $mainWindow 's\.value\("ppp/jsonReorderBufferMs"' `
    'PPP JSON writer buffering must be restored from the configuration.'
Assert-Contains $mainWindow 'connect\(_postThread, &QThread::finished, _postThread, &QObject::deleteLater\)' `
    'Post-processing threads must be deleted only after they finish.'
Assert-NotContains $mainWindow 'thread->quit\(\); thread->deleteLater\(\)' `
    'Post-processing must not destroy a QThread before its event loop exits.'
Assert-Contains $rtcmService 'GnssUtils::gpsTimeFromMsm' `
    'RtcmProcessingService must use the common MSM time conversion.'
Assert-Contains $rtcmService 'GnssUtils::gpsTimeFromSsr' `
    'RtcmProcessingService must use the common SSR time conversion.'
Assert-Contains $rtcmService 'GnssUtils::isGpsTimeOnSampleGrid' `
    'RtcmProcessingService must apply the common absolute epoch sample grid.'

$runtimeOwners = $mainWindow + "`n" + $pppWorker + "`n" +
                 (Read-RepoFile 'ppp_engine.cpp') + "`n" + $rtcmService
Assert-NotContains $runtimeOwners 'QIODevice::(WriteOnly|Append)|std::ofstream|fopen\s*\(' `
    'A runtime component writes files outside OutputWorker.'
Assert-Contains $outputWorker 'enqueueSp3' 'OutputWorker is missing SP3 output.'
Assert-Contains $outputWorker 'enqueueRinex' 'OutputWorker is missing RINEX output.'
Assert-Contains $outputWorker 'enqueuePppJson' 'OutputWorker is missing PPP JSON output.'
Assert-Contains $outputWorker 'enqueueZtd' 'OutputWorker is missing ZTD output.'
Assert-Contains $outputWorker 'enqueueTelemetry' 'OutputWorker is missing telemetry output.'
Assert-Contains $outputWorker 'enqueueArLog' 'OutputWorker is missing PPP-AR log output.'

Assert-Contains $vcxProject 'Name="DeployQtRuntime" AfterTargets="Build"' `
    'The Release build no longer deploys its Qt runtime.'
Assert-Contains $vcxProject '\$\(QTDIR\)\\bin\\windeployqt\.exe' `
    'Qt deployment must use windeployqt from the configured MSVC Qt installation.'

foreach ($manifest in @('RT-PPP.pro', 'RT-PPP.vcxproj', 'RT-PPP.vcxproj.filters')) {
    $content = Read-RepoFile $manifest
    Assert-NotContains $content '(?i)research[\\/]|legacy[\\/]' `
        "$manifest includes an excluded research or legacy tree."
}

Write-Output 'Architecture boundary smoke test passed.'
