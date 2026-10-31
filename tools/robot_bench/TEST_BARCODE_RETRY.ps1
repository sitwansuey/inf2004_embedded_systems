param(
    [ValidateSet('A','Z')][string]$Symbol='Z',
    [ValidateRange(1,3)][int]$Attempts=3,
    [ValidatePattern('^COM[0-9]+$')][string]$Port,
    [string]$RepoRoot,
    [string]$OutputDirectory )
$ErrorActionPreference='Stop'
Set-StrictMode -Version Latest

. (Join-Path $PSScriptRoot 'bench_common.ps1')
$benchContext=Get-RobotBenchContext -ToolsDirectory $PSScriptRoot -RepoRoot $RepoRoot -OutputDirectory $OutputDirectory
$repoPath=$benchContext.Repository
$outputPath=$benchContext.Output
$record=[ordered]@{started=(Get-Date).ToString('o');symbol=$Symbol;status='IN_PROGRESS';commands_sent=0;attempts=@()}
try {
    Write-Host 'Use the currently installed barcode-capture firmware: normal USB, no BOOTSEL or flash. Keep motor power off and software motor lock ON.'
    Write-Host 'Keep the robot still. On the same flat surface, use a straight guide to slide the card sideways across the bars at a steady pace.'
    Write-Host 'Choose a path across uninterrupted bars, away from the printed A/Z letter. Keep the same sensor distance and do not adjust its screws.'
    Write-Host 'Show WHITE first. WAIT for SCAN NOW/start beep, then pass the WHOLE barcode ONCE in about 4-6 seconds. Finish on WHITE and leave it there; do not slide back.'
    for($attempt=1;$attempt -le $Attempts;$attempt++) {
        Read-Host ('Prepare '+$Symbol+' attempt '+$attempt+'/'+$Attempts+'. WHITE under GP28; press Enter when ready')|Out-Null
        $name='BARCODE_TRY_'+$Symbol+'_'+(Get-Date -Format 'yyyyMMdd_HHmmss_fff')
        $argsCapture=@{Seconds=18;Procedure='BarcodeCapture';LogName=$name}
        if($Port){$argsCapture.Port=$Port}
        & (Join-Path $PSScriptRoot 'TEST_ROBOT_USB.ps1') @argsCapture -RepoRoot $repoPath -OutputDirectory $outputPath
        $replay=& (Join-Path $PSScriptRoot 'REPLAY_BARCODE_LOG.ps1') -LogPath (Join-Path $outputPath ($name+'.log')) -SummaryPath (Join-Path $outputPath ($name+'_RESULT.json')) -Symbol $Symbol -RepoRoot $repoPath -OutputDirectory $outputPath
        $record.attempts+=@([pscustomobject]@{log=$name+'.log';result=$replay})
        $record|ConvertTo-Json -Depth 8|Set-Content -LiteralPath (Join-Path $outputPath 'BARCODE_RETRY_RESULT.json') -Encoding UTF8
        if($replay.status -eq 'PASS_SENSOR_CAPTURE_NATIVE_REPLAY'){$record.status='PASS_SENSOR_CAPTURE_NATIVE_REPLAY';break}
    }
    if($record.status -eq 'IN_PROGRESS'){$record.status='NOT_DECODED'}
    Write-Host ('Finished: '+$record.status+'. Logs preserved. Tell me done so I can review the recordings.')
} catch {$record.status='ERROR';$record.error=$_.Exception.Message;throw}
finally {$record|ConvertTo-Json -Depth 8|Set-Content -LiteralPath (Join-Path $outputPath 'BARCODE_RETRY_RESULT.json') -Encoding UTF8}
