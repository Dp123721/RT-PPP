# Verifies that the configured application starts and exits in headless mode.
param(
    [string]$AppPath = (Join-Path $PSScriptRoot '..\build\vs\debug\RT-PPP.exe')
)

$ErrorActionPreference = 'Stop'
$app = (Resolve-Path -LiteralPath $AppPath).Path
$config = (Resolve-Path -LiteralPath (Join-Path $PSScriptRoot 'fixtures\headless_realtime.ini')).Path
$qtBin = 'D:\Qt\5.15.2\msvc2019_64\bin'

if (-not (Test-Path -LiteralPath $qtBin)) {
    throw "Qt runtime directory not found: $qtBin"
}

$previousPath = $env:PATH
$env:PATH = $qtBin + ';' + $env:PATH
try {
    $process = Start-Process -FilePath $app `
        -ArgumentList @('--headless', '--config', $config, '--run-seconds', '1') `
        -WorkingDirectory (Get-Location).Path -PassThru -Wait -NoNewWindow
    if ($process.ExitCode -ne 0) {
        throw "Headless startup probe failed with exit code $($process.ExitCode)"
    }
} finally {
    $env:PATH = $previousPath
}

Write-Output 'Headless startup smoke test passed.'
