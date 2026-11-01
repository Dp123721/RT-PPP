# Creates an isolated Python environment and installs the GFS worker dependencies.
$ErrorActionPreference = 'Stop'
$runtimeDir = Split-Path -Parent $MyInvocation.MyCommand.Path
$venvDir = Join-Path $runtimeDir '.venv'

if (-not (Test-Path (Join-Path $venvDir 'Scripts\python.exe'))) {
    $pythonCommand = Get-Command py -ErrorAction SilentlyContinue
    if ($null -ne $pythonCommand) {
        & $pythonCommand.Source -3 -m venv $venvDir
    } else {
        $pythonCommand = Get-Command python -ErrorAction SilentlyContinue
        if ($null -eq $pythonCommand) { throw 'Python 3 was not found. Install Python 3.10+ and rerun bootstrap.ps1.' }
        & $pythonCommand.Source -m venv $venvDir
    }
}

$venvPython = Join-Path $venvDir 'Scripts\python.exe'
$requirements = Join-Path $runtimeDir 'requirements.txt'
$wheelhouse = Join-Path $runtimeDir 'wheelhouse'
if ((Test-Path $wheelhouse) -and (Get-ChildItem $wheelhouse -Filter '*.whl' -ErrorAction SilentlyContinue)) {
    & $venvPython -m pip install --no-index --find-links $wheelhouse --requirement $requirements
} else {
    & $venvPython -m pip install --disable-pip-version-check --requirement $requirements
}
& $venvPython -c "import eccodes; print('RT-PPP GFS Python runtime ready:', eccodes.codes_get_api_version())"
