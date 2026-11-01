# Runs the research package's Python unit and schema-contract tests.
$ErrorActionPreference = "Stop"

$researchRoot = Split-Path -Parent $MyInvocation.MyCommand.Path
$repositoryRoot = Split-Path -Parent $researchRoot
$previousPythonPath = $env:PYTHONPATH

try {
    $env:PYTHONPATH = Join-Path $researchRoot "src"
    Push-Location $repositoryRoot

    python -c "from pathlib import Path; path = Path('research/main.py'); compile(path.read_text(encoding='utf-8'), str(path), 'exec')"
    if ($LASTEXITCODE -ne 0) {
        throw "entry-point syntax check failed with exit code $LASTEXITCODE"
    }

    python -m compileall -q research/src
    if ($LASTEXITCODE -ne 0) {
        throw "compileall failed with exit code $LASTEXITCODE"
    }

    python -m unittest discover -s research/tests -v
    if ($LASTEXITCODE -ne 0) {
        throw "unittest failed with exit code $LASTEXITCODE"
    }
}
finally {
    Pop-Location
    $env:PYTHONPATH = $previousPythonPath
}
