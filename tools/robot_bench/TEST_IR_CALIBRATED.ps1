param([string]$RepoRoot,
    [string]$OutputDirectory)
$ErrorActionPreference='Stop'
Set-StrictMode -Version Latest

. (Join-Path $PSScriptRoot 'bench_common.ps1')
$benchContext=Get-RobotBenchContext -ToolsDirectory $PSScriptRoot -RepoRoot $RepoRoot -OutputDirectory $OutputDirectory
$repoPath=$benchContext.Repository
$outputPath=$benchContext.Output
$steps=[System.Collections.Generic.List[object]]::new()
$record=[ordered]@{started=(Get-Date).ToString('o');status='IN_PROGRESS';commands_sent=0;steps=$steps;physical_line_following_validated=$false;physical_barcode_decoding_validated=$false}
function Save-Result { $record | ConvertTo-Json -Depth 7 | Set-Content -LiteralPath (Join-Path $outputPath 'IR_CALIBRATED_STEPS.json') -Encoding UTF8 }
function Capture-Step([string]$Name,[string]$Cue,[int]$Mask,[int]$Direction,[bool]$BarcodeBlack) {
    Read-Host ($Cue+' Press Enter when ready, then hold everything still') | Out-Null
    Start-Sleep -Seconds 3
    & (Join-Path $PSScriptRoot 'TEST_ROBOT_USB.ps1') -Seconds 8 -Procedure IRCalibrated -LogName ('IR_CAL_'+$Name) -RepoRoot $repoPath -OutputDirectory $outputPath
    $summary=Get-Content -LiteralPath (Join-Path $outputPath ('IR_CAL_'+$Name+'_RESULT.json')) -Raw | ConvertFrom-Json
    if($summary.status -ne 'CAPTURED' -or $summary.controller_lines -lt 1 -or $summary.controller_fault_records -gt 0 -or $summary.incomplete_controller_records -gt 0 -or $summary.unexpected_motor_status_lines -gt 0 -or $summary.ir_unexpected_steering -gt 0 -or $summary.ir_calibrated_incomplete_records -gt 0 -or $summary.ir_calibrated_unique_scans -lt 20 -or $summary.ir_calibrated_valid_rows -ne $summary.ir_calibrated_rows) {
        $codes=$summary.ir_calibrated_algorithm_errors | ConvertTo-Json -Compress
        throw ('Capture '+$Name+' failed status/freshness checks: fresh acquisitions='+$summary.ir_calibrated_acquired_rows+', valid interpretation='+$summary.ir_calibrated_valid_rows+', algorithm codes='+$codes+'. Diagnostic files saved; stop here.')
    }
    $wrong=@($summary.ir_calibrated_observations | Where-Object {
        $_.blackmask -ne $Mask -or
        ($Direction -ne 0 -and ($_.line -ne 1 -or $_.lineerror -ne $Direction)) -or
        ($Mask -eq 0 -and $_.line -notin @(2,3)) -or
        (($Mask -band 1) -ne 0 -and $_.right_darkness -lt 600) -or
        (($Mask -band 1) -eq 0 -and $_.right_darkness -gt 400) -or
        (($Mask -band 2) -ne 0 -and $_.left_darkness -lt 600) -or
        (($Mask -band 2) -eq 0 -and $_.left_darkness -gt 400) -or
        ($BarcodeBlack -and $_.barcode_darkness -lt 600) -or
        (-not $BarcodeBlack -and $_.barcode_darkness -gt 400)
    })
    $steps.Add([pscustomobject]@{name=$Name;started=$summary.started;valid_scans=$summary.ir_calibrated_unique_scans;expected_mask=$Mask;expected_direction=$Direction;expected_barcode_black=$BarcodeBlack;mismatches=$wrong.Count})
    Save-Result
    if($wrong.Count -gt 0) { throw ('Capture '+$Name+' has '+$wrong.Count+' unexpected or ambiguous surface classifications. Fully cover the selected sensing windows with flat targets at the calibrated height; logs saved for review. No settings or wires changed.') }
}
try {
    Write-Host 'Motor-locked stationary interpretation test. Leave wiring and adjustment screws unchanged.'
    Write-Host 'Use the same flat targets, sensor heights and barcode reading plane as the calibration captures.'
    Write-Host 'Fully cover each sensing face. Use firm backing and keep targets still for the whole capture; avoid seams, edges and uneven height.'
    Write-Host 'Barcode darkness is reflectance, not a decoded character. No drive or ARM commands are sent.'
    Save-Result
    Capture-Step 'WHITE' 'Place WHITE at both line sensors and at the GP28 barcode sensing face.' 0 0 $false
    Capture-Step 'RIGHT' 'Place BLACK only at RIGHT line GP26. LEFT GP27 and BARCODE GP28 stay WHITE.' 1 1000 $false
    Capture-Step 'LEFT' 'Place BLACK only at LEFT line GP27. RIGHT GP26 and BARCODE GP28 stay WHITE.' 2 -1000 $false
    Capture-Step 'BARCODE' 'Return both line sensors to WHITE. Place BLACK directly at BARCODE GP28 at its calibrated plane/distance.' 0 0 $true
    $record.status='PASS_STATIONARY_CLASSIFICATION'
    Save-Result
    Write-Host 'Stationary classification passed. Motor lock stays ON. Moving line following and barcode decoding remain untested.'
} catch {
    $record.status='ERROR';$record.error=$_.Exception.Message;Save-Result;throw
}
