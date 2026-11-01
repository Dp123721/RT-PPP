# Builds the Qt application, runs focused C++ checks, and verifies shutdown behavior.
param(
    [string]$QtRoot = 'D:\Qt\5.15.2\msvc2019_64',
    [switch]$SkipAppBuild
)

$ErrorActionPreference = 'Stop'
$repoRoot = Split-Path -Parent $PSScriptRoot
$buildDir = Join-Path $PSScriptRoot 'build_smoke'
$vsRoot = 'C:\Program Files\Microsoft Visual Studio\2022\Community'
$vsDevCmd = Join-Path $vsRoot 'Common7\Tools\VsDevCmd.bat'
$msbuild = Join-Path $vsRoot 'MSBuild\Current\Bin\MSBuild.exe'
$qtMsBuild = Join-Path $env:LOCALAPPDATA 'QtMsBuild'

foreach ($requiredPath in @($vsDevCmd, $msbuild, (Join-Path $QtRoot 'bin\qmake.exe'))) {
    if (-not (Test-Path -LiteralPath $requiredPath)) {
        throw "Required tool not found: $requiredPath"
    }
}

New-Item -ItemType Directory -Force -Path $buildDir | Out-Null

if (-not $SkipAppBuild) {
    & $msbuild (Join-Path $repoRoot 'RT-PPP.sln') /m /t:Build `
        /p:Configuration=Debug /p:Platform=x64 `
        "/p:QtMsBuild=$qtMsBuild" "/p:QTDIR=$QtRoot" /v:minimal
    if ($LASTEXITCODE -ne 0) {
        throw 'RT-PPP Debug build failed'
    }
}

function Build-SmokeTest {
    param(
        [string]$Name,
        [string[]]$Sources,
        [switch]$Network
    )

    $sourceArgs = ($Sources | ForEach-Object {
        if ([IO.Path]::IsPathRooted($_)) {
            $source = $_
        } else {
            $source = Join-Path $repoRoot $_
        }
        '"' + $source + '"'
    }) -join ' '
    $exePath = Join-Path $buildDir "$Name.exe"
    $networkDefines = if ($Network) {
        '/DQT_NETWORK_LIB /I"' + $QtRoot + '\include\QtNetwork" '
    } else { '' }
    $networkLib = if ($Network) { 'Qt5Network.lib ' } else { '' }
    $compile = 'call "{0}" -arch=x64 && cd /d "{3}" && cl /nologo /std:c++17 /Zc:__cplusplus /EHsc /MD ' +
               '/DQT_CORE_LIB /DQT_NO_DEBUG /DUNICODE /D_UNICODE {6}' +
               '/I"{1}\include" /I"{1}\include\QtCore" /I"{5}" {2} ' +
               '/Fe:"{4}" /link /LIBPATH:"{1}\lib" Qt5Core.lib {7}'
    $command = $compile -f $vsDevCmd, $QtRoot, $sourceArgs, $buildDir, $exePath, $repoRoot, $networkDefines, $networkLib
    & cmd.exe /d /c $command
    if ($LASTEXITCODE -ne 0) {
        throw "$Name compilation failed"
    }
}

Build-SmokeTest 'gnss_utils_smoke' @(
    'tests\gnss_utils_smoke.cpp',
    'gnss_utils.cpp'
)
Build-SmokeTest 'pppjsonwriter_smoke' @(
    'tests\pppjsonwriter_smoke.cpp',
    'pppjsonwriter.cpp',
    'gnss_utils.cpp'
)
Build-SmokeTest 'settingsstore_smoke' @(
    'tests\settingsstore_smoke.cpp',
    'settingsstore.cpp'
)
Build-SmokeTest 'ambbie_smoke' @(
    'tests\ambbie_smoke.cpp',
    'ambbie.cpp'
)
$gfsMoc = Join-Path $buildDir 'moc_gfs_zwd.cpp'
& (Join-Path $QtRoot 'bin\moc.exe') (Join-Path $repoRoot 'gfs_zwd.h') "-o$gfsMoc"
if ($LASTEXITCODE -ne 0) {
    throw 'GFS ZWD MOC generation failed'
}
Build-SmokeTest 'gfs_zwd_smoke' @(
    'tests\gfs_zwd_smoke.cpp',
    'gfs_zwd.cpp',
    $gfsMoc
) -Network
Build-SmokeTest 'outputworker_smoke' @(
    'tests\outputworker_smoke.cpp',
    'outputworker.cpp',
    'pppjsonwriter.cpp',
    'sp3writer.cpp',
    'rinexobswriter.cpp',
    'telemetrywriter.cpp',
    'gnss_utils.cpp',
    'gnss_signals.cpp'
)
$postMoc = Join-Path $buildDir 'moc_postppp.cpp'
& (Join-Path $QtRoot 'bin\moc.exe') (Join-Path $repoRoot 'postppp.h') "-o$postMoc"
if ($LASTEXITCODE -ne 0) {
    throw 'postppp MOC generation failed'
}
Build-SmokeTest 'postppp_smoke' @(
    'tests\postppp_smoke.cpp',
    'postppp.cpp',
    'gfs_zwd.cpp',
    $gfsMoc,
    'ppp_engine.cpp',
    'ambbie.cpp',
    'ambfix.cpp',
    'gpt3_vmf3.cpp',
    'gnss_ephemeris.cpp',
    'gnss_utils.cpp',
    'gnss_signals.cpp',
    'rtcmdecoder.cpp',
    'telemetrywriter.cpp',
    'pppjsonwriter.cpp',
    $postMoc
) -Network

$env:PATH = (Join-Path $QtRoot 'bin') + ';' + $env:PATH
foreach ($testName in @(
    'gnss_utils_smoke',
    'pppjsonwriter_smoke',
    'settingsstore_smoke',
    'ambbie_smoke',
    'gfs_zwd_smoke',
    'outputworker_smoke',
    'postppp_smoke'
)) {
    & (Join-Path $buildDir "$testName.exe")
    if ($LASTEXITCODE -ne 0) {
        throw "$testName failed"
    }
}

& (Join-Path $PSScriptRoot 'architecture_smoke.ps1')
if ($LASTEXITCODE -ne 0) {
    throw 'architecture boundary smoke test failed'
}

$appPath = Join-Path $repoRoot 'debug\RT-PPP.exe'
if (Test-Path -LiteralPath $appPath) {
    $env:QT_QPA_PLATFORM = 'offscreen'
    $env:RTPPP_SMOKE_EXIT_MS = '250'
    try {
        $appProcess = Start-Process -FilePath $appPath -WorkingDirectory $buildDir `
            -PassThru -Wait -WindowStyle Hidden
        if ($appProcess.ExitCode -ne 0) {
            throw "RT-PPP lifecycle smoke test failed with exit code $($appProcess.ExitCode)"
        }
    } finally {
        Remove-Item Env:RTPPP_SMOKE_EXIT_MS
        Remove-Item Env:QT_QPA_PLATFORM
    }
}

Write-Output 'All C++ smoke tests passed.'
