param([string]$GccPath,[string]$RepoRoot,[string]$OutputDirectory)
$ErrorActionPreference='Stop'
Set-StrictMode -Version Latest
. (Join-Path $PSScriptRoot 'bench_common.ps1')
$context=Get-RobotBenchContext -ToolsDirectory $PSScriptRoot -RepoRoot $RepoRoot -OutputDirectory $OutputDirectory
$repoPath=$context.Repository;$outputPath=$context.Output
if(Test-Path -LiteralPath (Join-Path $repoPath 'app_program\robot_config.local.h')){throw 'Review/remove the local board override before building this calibrated replay profile.'}
if(-not $GccPath) {
    $candidate=Get-Command gcc -ErrorAction SilentlyContinue
    if($candidate){$GccPath=$candidate.Source}
    elseif(Test-Path -LiteralPath 'C:\msys64\ucrt64\bin\gcc.exe'){$GccPath='C:\msys64\ucrt64\bin\gcc.exe'}
    else{throw 'Native GCC was not found. Pass -GccPath with its full path.'}
}
$gcc=(Resolve-Path -LiteralPath $GccPath).Path
$names=@('testing/subsystem3/replay_barcode_capture.c','app_program/robot_config.h','app_program/robot_ir_calibration.h','app_program/robot_ir_config.h','app_program/subsystem3/buddy3_line.c','app_program/subsystem3/buddy3_line.h','app_program/subsystem3/buddy3_barcode.c','app_program/subsystem3/buddy3_barcode.h','app_program/subsystem3/buddy3_core.c','app_program/subsystem3/buddy3_core.h')
$hashes=[ordered]@{}
foreach($name in $names){$hashes[$name]=(Get-FileHash -LiteralPath (Join-Path $repoPath $name) -Algorithm SHA256).Hash}
$sources=@($names|Where-Object{$_ -like '*.c'}|ForEach-Object{Join-Path $repoPath $_})
$flags=@('-std=c11','-O2','-static','-Wall','-Wextra','-Werror','-Wshadow','-Wconversion','-pedantic','-DBOARD_IR_CALIBRATED_BENCH=1','-DBOARD_IR_MODE=2')
$tempExe=Join-Path $outputPath ('replay_build_'+[guid]::NewGuid().ToString('N')+'.exe')
$oldBenchPath=$env:PATH
try {
    $env:PATH=(Split-Path $gcc -Parent)+[System.IO.Path]::PathSeparator+$env:PATH
    & $gcc @flags @sources -o $tempExe
    if($LASTEXITCODE -ne 0){throw ('Host replay compilation failed: '+$LASTEXITCODE)}
    foreach($name in $names){if((Get-FileHash -LiteralPath (Join-Path $repoPath $name) -Algorithm SHA256).Hash -ne $hashes[$name]){throw ('Source changed during host build: '+$name)}}
    $manifest=[ordered]@{built=(Get-Date).ToString('o');repository=$repoPath;compiler=$gcc;flags=$flags;exe_sha256=(Get-FileHash -LiteralPath $tempExe -Algorithm SHA256).Hash;sources=$hashes;scope='Host production normalization/barcode replay only. No firmware build, serial command, calibration or physical pass.'}
    Move-Item -LiteralPath $tempExe -Destination (Join-Path $outputPath 'BARCODE_REPLAY_NATIVE.exe') -Force
    $manifest|ConvertTo-Json -Depth 5|Set-Content -LiteralPath (Join-Path $outputPath 'BARCODE_REPLAY_BUILD.json') -Encoding UTF8
    Write-Host ('Host replay built from '+$repoPath+'. Ten input hashes recorded; firmware and robot unchanged.')
} finally {$env:PATH=$oldBenchPath}
