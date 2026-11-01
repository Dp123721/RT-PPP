# Builds the bundled troposphere worker from its Fortran sources.
param(
    [Parameter(Mandatory = $true)]
    [string]$OutputDirectory,
    [string]$FortranCompiler = 'gfortran'
)

$ErrorActionPreference = 'Stop'
$sourceDirectory = Split-Path -Parent $MyInvocation.MyCommand.Path
$compiler = Get-Command $FortranCompiler -ErrorAction Stop
New-Item -ItemType Directory -Force -Path $OutputDirectory | Out-Null
$worker = Join-Path $OutputDirectory 'gpt3_vmf3_worker.exe'

& $compiler.Source -O2 -std=f2008 -ffree-line-length-none -static-libgfortran -static-libgcc `
    (Join-Path $sourceDirectory 'gpt3_vmf3_worker.f90') `
    (Join-Path $sourceDirectory 'gpt3_5.f90') `
    (Join-Path $sourceDirectory 'vmf3_ht.f90') `
    '-o' $worker
if ($LASTEXITCODE -ne 0) {
    throw 'GPT3/VMF3 Fortran worker compilation failed.'
}

Copy-Item -LiteralPath (Join-Path $sourceDirectory 'gpt3_5.grd') `
    -Destination (Join-Path $OutputDirectory 'gpt3_5.grd') -Force
Write-Output "GPT3/VMF3 worker ready: $worker"
