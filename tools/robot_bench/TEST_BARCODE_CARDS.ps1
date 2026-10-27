param([string]$RepoRoot,
    [string]$OutputDirectory)
$ErrorActionPreference='Stop'
Set-StrictMode -Version Latest

. (Join-Path $PSScriptRoot 'bench_common.ps1')
$benchContext=Get-RobotBenchContext -ToolsDirectory $PSScriptRoot -RepoRoot $RepoRoot -OutputDirectory $OutputDirectory
$repoPath=$benchContext.Repository
$outputPath=$benchContext.Output
$steps=[System.Collections.Generic.List[object]]::new()
$record=[ordered]@{started=(Get-Date).ToString('o');status='IN_PROGRESS';commands_sent=0;decoder_replay_pending=$true;steps=$steps}
function Save-Result { $record|ConvertTo-Json -Depth 6|Set-Content -LiteralPath (Join-Path $outputPath 'BARCODE_CARDS_STEPS.json') -Encoding UTF8 }
try {
    Write-Host 'Robot stays still with motor power off and its software lock ON. Leave calibration, wires and sensor height unchanged.'
    Write-Host 'Only GP28 is used for this barcode optics test. No patches are required under GP26/GP27. Keep the chassis still.'
    Write-Host 'Begin on barcode WHITE and WAIT for SCAN NOW/start beep. Sweep the WHOLE barcode once, steadily, and finish on WHITE before the finish beep 6 seconds later.'
    foreach($symbol in @('A','Z')) {
        Read-Host ('Prepare card '+$symbol+'. Show WHITE to GP28, with its normal barcode reading distance/plane. Press Enter when ready')|Out-Null
        & (Join-Path $PSScriptRoot 'TEST_ROBOT_USB.ps1') -Seconds 25 -Procedure BarcodeCapture -LogName ('BARCODE_CARD_'+$symbol) -RepoRoot $repoPath -OutputDirectory $outputPath
        $summary=Get-Content -LiteralPath (Join-Path $outputPath ('BARCODE_CARD_'+$symbol+'_RESULT.json')) -Raw|ConvertFrom-Json
        if($summary.status -ne 'CAPTURED' -or $summary.controller_lines -lt 1 -or $summary.controller_fault_records -gt 0 -or $summary.incomplete_controller_records -gt 0 -or $summary.unexpected_motor_status_lines -gt 0 -or $summary.barcode_capture_unique_scans -lt 100 -or $summary.barcode_capture_valid_rows -ne $summary.barcode_capture_rows -or $summary.barcode_capture_incomplete_records -gt 0 -or $summary.barcode_capture_errors -gt 0) {throw ('Card '+$symbol+' capture failed; diagnostics saved. No decoder result claimed.')}
        $steps.Add([pscustomobject]@{symbol=$symbol;started=$summary.started;scans=$summary.barcode_capture_unique_scans;timing_gaps=$summary.barcode_capture_gap_records;decoder_replay_pending=$true})
        Save-Result
    }
    $record.status='CAPTURED_REPLAY_PENDING';Save-Result
    Write-Host 'Both card recordings saved. Tell me done so I can replay the timestamped scans through the production decoder. Capture completion is not a decoding pass.'
} catch {$record.status='ERROR';$record.error=$_.Exception.Message;Save-Result;throw}
