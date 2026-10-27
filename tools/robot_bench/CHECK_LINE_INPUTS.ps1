param(
    [Parameter(Mandatory=$true)][string]$LogPath,
    [Parameter(Mandatory=$true)][string]$SummaryPath,
    [ValidateSet('White','RightBlack','LeftBlack','BothBlack','Observe')][string]$Expected='Observe',
    [string]$GccPath,
    [string]$RepoRoot,
    [string]$OutputDirectory)
$ErrorActionPreference='Stop'
Set-StrictMode -Version Latest
. (Join-Path $PSScriptRoot 'bench_common.ps1')
$context=Get-RobotBenchContext -ToolsDirectory $PSScriptRoot -RepoRoot $RepoRoot -OutputDirectory $OutputDirectory
function Get-LineFileHash([string]$Path){
    $stream=[IO.File]::OpenRead($Path);$sha=[Security.Cryptography.SHA256]::Create()
    try{return ([BitConverter]::ToString($sha.ComputeHash($stream))).Replace('-','')}
    finally{$sha.Dispose();$stream.Dispose()}
}
$base=[IO.Path]::Combine([IO.Path]::GetDirectoryName([IO.Path]::GetFullPath($LogPath)),[IO.Path]::GetFileNameWithoutExtension($LogPath))
$result=[ordered]@{status='ERROR';expected=$Expected;commands_sent=0;motion_validated=$false;calibration_applied=$false;scope='Stationary physical line-input recording replayed by production C on the PC. GP28 cannot affect line classification.'}
try {
    if(Test-Path -LiteralPath (Join-Path $context.Repository 'app_program/robot_config.local.h')){throw 'Reconcile the local board override before interpreting line captures.'}
    $capture=Get-Content -LiteralPath $SummaryPath -Raw | ConvertFrom-Json
    if($capture.status -ne 'CAPTURED' -or $capture.commands_sent -ne 0 -or $capture.procedure -notin @('IRMap','BarcodeCapture')){throw 'A completed passive capture is required.'}
    $text=Get-Content -LiteralPath $LogPath -Raw
    $controller=@([regex]::Matches($text,'(?m)^\[CSC02\] t=(\d+) phase=(\d+) fault=(\d+) armed=(\d+) cmd=(\d+) power=(\d+) flags=(\d+) bytes=(\d+)\r?$'))
    if($controller.Count -lt 1 -or [regex]::Matches($text,'\[CSC02\]').Count -ne $controller.Count){throw 'Complete controller reports are required.'}
    foreach($row in $controller){if($row.Groups[2].Value -ne '0' -or $row.Groups[3].Value -ne '0' -or $row.Groups[4].Value -ne '0' -or $row.Groups[5].Value -ne '0' -or $row.Groups[6].Value -ne '0'){throw 'Controller is not fault-free, DISARMED and STOPPED.'}}
    $rows=@([regex]::Matches($text,'(?m)^\[S3B\] t=(\d+) raw=(\d+),(\d+),(\d+) mask=(\d+) dropped=(\d+)\r?$'))
    if($rows.Count -lt 100 -or [regex]::Matches($text,'\[S3B\]').Count -ne $rows.Count -or $text.Contains('[S3BERR]')){throw 'Incomplete raw IR recording.'}
    $csv=[Collections.Generic.List[string]]::new()
    foreach($row in $rows){$csv.Add(((1..6|ForEach-Object{$row.Groups[$_].Value}) -join ','))}
    [IO.File]::WriteAllLines(($base+'_LINE.csv'),$csv,[Text.Encoding]::ASCII)
    if(-not $GccPath){$gccCommand=Get-Command gcc -ErrorAction SilentlyContinue;if($gccCommand){$GccPath=$gccCommand.Source}else{$GccPath='C:\msys64\ucrt64\bin\gcc.exe'}}
    $names=@('testing/subsystem3/replay_line_capture.c','app_program/robot_config.h','app_program/robot_ir_config.h','app_program/robot_ir_calibration.h','app_program/subsystem3/buddy3_line.c','app_program/subsystem3/buddy3_line.h','app_program/subsystem3/buddy3_core.c','app_program/subsystem3/buddy3_core.h','app_program/subsystem3/buddy3_barcode.c','app_program/subsystem3/buddy3_barcode.h')
    $hashes=[ordered]@{}
    foreach($name in $names){$hashes[$name]=Get-LineFileHash (Join-Path $context.Repository $name)}
    $exe=Join-Path $context.Output ('LINE_REPLAY_'+[guid]::NewGuid().ToString('N')+'.exe')
    $sources=@($names|Where-Object{$_ -like '*.c'}|ForEach-Object{Join-Path $context.Repository $_})
    & $GccPath -std=c11 -O2 -static -Wall -Wextra -Werror -Wshadow -Wconversion -pedantic -DBOARD_IR_CALIBRATED_BENCH=1 -DBOARD_IR_MODE=2 @sources -o $exe
    if($LASTEXITCODE -ne 0){throw 'Line replay compilation failed.'}
    foreach($name in $names){if((Get-LineFileHash (Join-Path $context.Repository $name)) -ne $hashes[$name]){throw ('Source changed during replay build: '+$name)}}
    $mask=@{White=0;RightBlack=1;LeftBlack=2;BothBlack=3;Observe=4}[$Expected]
    $native=@(& $exe ($base+'_LINE.csv') ([string]$mask) 2>&1)
    $nativeExit=$LASTEXITCODE
    $native | Set-Content -LiteralPath ($base+'_LINE.log') -Encoding UTF8
    if($nativeExit -notin @(0,2,3)){throw ('Line replay execution failed: '+$nativeExit)}
    $stats=($native -join "`n") | ConvertFrom-Json
    if($stats.rows -ne $rows.Count){throw 'Incomplete line replay result.'}
    $result.statistics=$stats;$result.source_hashes=$hashes
    $result.replay_exe_sha256=Get-LineFileHash $exe
    $result.log_sha256=Get-LineFileHash $LogPath
    $result.native_exit=$nativeExit
    $result.status=if($nativeExit -eq 3){'INVALID_CAPTURE'}elseif($nativeExit -eq 2){'CLASSIFICATION_MISMATCH'}elseif($Expected -eq 'Observe'){'OBSERVATION_ONLY'}else{'PASS_STATIONARY_LINE_INPUTS'}
    Write-Host ($result.status+': '+$rows.Count+' samples; right/GP26 '+$stats.raw_min[0]+'..'+$stats.raw_max[0]+'; left/GP27 '+$stats.raw_min[1]+'..'+$stats.raw_max[1]+'. No motor commands or calibration changes.')
} catch {$result.error=$_.Exception.Message;throw}
finally{$result|ConvertTo-Json -Depth 7|Set-Content -LiteralPath ($base+'_LINE_RESULT.json') -Encoding UTF8}
[pscustomobject]$result
