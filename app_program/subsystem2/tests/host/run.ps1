param([string]$Compiler = 'gcc')

$ErrorActionPreference = 'Stop'
& python -B (Join-Path $PSScriptRoot 'run.py') --compiler $Compiler
exit $LASTEXITCODE
