param(
    [string]$Compiler = "gcc",
    [string]$Report
)

$ErrorActionPreference = "Stop"
$runArgs = @('-B', (Join-Path $PSScriptRoot 'run.py'), '--compiler', $Compiler)
if ($Report) {
    $runArgs += @('--report', $Report)
}
& python @runArgs
if ($LASTEXITCODE -ne 0) {
    exit $LASTEXITCODE
}
