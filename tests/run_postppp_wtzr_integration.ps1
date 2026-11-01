# Builds and runs the recorded-product PPP integration scenario.
param(
    [string]$QtRoot = 'D:\Qt\5.15.2\msvc2019_64'
)

$ErrorActionPreference = 'Stop'
$repoRoot = Split-Path -Parent $PSScriptRoot
$buildDir = Join-Path $PSScriptRoot 'build_smoke'
$vsDevCmd = 'C:\Program Files\Microsoft Visual Studio\2022\Community\Common7\Tools\VsDevCmd.bat'

$postMoc = Join-Path $buildDir 'moc_postppp.cpp'
& (Join-Path $QtRoot 'bin\moc.exe') (Join-Path $repoRoot 'postppp.h') "-o$postMoc"
if ($LASTEXITCODE -ne 0) { throw 'postppp MOC generation failed' }

$sources = @(
    'tests\postppp_wtzr_integration.cpp', 'postppp.cpp', 'gfs_zwd.cpp', 'ppp_engine.cpp',
    'ambbie.cpp', 'ambfix.cpp', 'gnss_ephemeris.cpp', 'gnss_utils.cpp',
    'gnss_signals.cpp', 'rtcmdecoder.cpp', 'pppjsonwriter.cpp',
    $postMoc
)
$sourceArgs = ($sources | ForEach-Object {
    $source = if ([IO.Path]::IsPathRooted($_)) { $_ } else { Join-Path $repoRoot $_ }
    '"' + $source + '"'
}) -join ' '
$exePath = Join-Path $buildDir 'postppp_wtzr_integration.exe'
$command = 'call "{0}" -arch=x64 && cd /d "{1}" && cl /nologo /std:c++17 /Zc:__cplusplus /EHsc /MD /DQT_CORE_LIB /DQT_NETWORK_LIB /DQT_NO_DEBUG /DUNICODE /D_UNICODE /I"{2}\include" /I"{2}\include\QtCore" /I"{2}\include\QtNetwork" /I"{1}" {3} /Fe:"{4}" /link /LIBPATH:"{2}\lib" Qt5Core.lib Qt5Network.lib' -f $vsDevCmd, $repoRoot, $QtRoot, $sourceArgs, $exePath
& cmd.exe /d /c $command
if ($LASTEXITCODE -ne 0) { throw 'postppp WTZR integration compilation failed' }

$env:PATH = (Join-Path $QtRoot 'bin') + ';' + $env:PATH
& $exePath
if ($LASTEXITCODE -ne 0) { throw 'postppp WTZR integration failed' }
