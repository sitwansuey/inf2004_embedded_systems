param([ValidatePattern('^COM[0-9]+$')][string]$Port,
    [string]$RepoRoot,
    [string]$OutputDirectory )
$ErrorActionPreference='Stop'
Set-StrictMode -Version Latest

. (Join-Path $PSScriptRoot 'bench_common.ps1')
$benchContext=Get-RobotBenchContext -ToolsDirectory $PSScriptRoot -RepoRoot $RepoRoot -OutputDirectory $OutputDirectory
$repoPath=$benchContext.Repository
$outputPath=$benchContext.Output
$record=[ordered]@{started=(Get-Date).ToString('o');repository=$repoPath;status='IN_PROGRESS';gpio=28;commands_sent=0;calibration_applied=$false;barcode_decoding_validated=$false;motion_validated=$false;captures=@();reference_white=$null;reference_black=$null;reference_black_source='Local reviewed source calibration; not proof of installed firmware identity';threshold_low=400;threshold_high=600}
$resultPath=Join-Path $outputPath 'BARCODE_PLANE_RESULT.json'
function Save-Result {$record|ConvertTo-Json -Depth 8|Set-Content -LiteralPath $resultPath -Encoding UTF8}
try {
    $verified=Get-Content -LiteralPath (Join-Path $PSScriptRoot 'source_verification.json') -Raw|ConvertFrom-Json
    if((Get-FileHash -LiteralPath (Join-Path $repoPath 'app_program\robot_ir_calibration.h') -Algorithm SHA256).Hash -ne $verified.files.'app_program/robot_ir_calibration.h'){throw 'ADC endpoints changed since these reflectance references were verified. Reconcile the helper before collecting a new plane result.'}
    $reference=Get-RobotIrCalibrationReference -Repository $repoPath
    $record.reference_white=$reference.White[2];$record.reference_black=$reference.Black[2]
    Write-Host 'Use normal USB. Motor power OFF, software drive lock ON. No flash or robot commands. Comparisons use local source endpoints; installed firmware may differ.'
    Write-Host 'Keep the robot and GP28 mount fixed. Put a firm, flat backing under the card at the exact height and angle intended for future barcode scans.'
    Write-Host 'Only the GP28 barcode window needs the selected colour. Leave the line sensors, wiring and adjustment screws unchanged.'
    Write-Host 'Use the actual printed barcode card for both targets: a blank WHITE area, then the centre of its widest BLACK bar. Do not use the table, tape or another surface as the black reference.'
    Write-Host 'Keep the same card plane and gap. The printed black bar must cover the optical sensing spot; if it cannot, stop and inspect sensor alignment/height before calibrating.'
    Save-Result
    foreach($surface in @('WHITE','BLACK')) {
        Read-Host ('Show uniform '+$surface+' to GP28 at that flat plane. Press Enter, then hold it still for the whole recording')|Out-Null
        $name='BARCODE_PLANE_'+$surface+'_'+(Get-Date -Format 'yyyyMMdd_HHmmss_fff')
        $captureArgs=@{Seconds=8;Procedure='BarcodeCapture';BarcodeHold=$surface;LogName=$name}
        if($Port){$captureArgs.Port=$Port}
        & (Join-Path $PSScriptRoot 'TEST_ROBOT_USB.ps1') @captureArgs -RepoRoot $repoPath -OutputDirectory $outputPath
        $s=Get-Content -LiteralPath (Join-Path $outputPath ($name+'_RESULT.json')) -Raw|ConvertFrom-Json
        if($s.status -ne 'CAPTURED' -or $s.commands_sent -ne 0 -or $s.controller_lines -lt 1 -or $s.controller_fault_records -ne 0 -or $s.incomplete_controller_records -ne 0 -or $s.unexpected_motor_status_lines -ne 0 -or $s.barcode_hold -ne $surface -or $s.barcode_capture_unique_scans -lt 100 -or $s.barcode_capture_unique_scans -ne $s.barcode_capture_rows -or $s.barcode_capture_valid_rows -ne $s.barcode_capture_rows -or $s.barcode_capture_incomplete_records -ne 0 -or $s.barcode_capture_errors -ne 0 -or $s.barcode_capture_gap_records -ne 0 -or $s.barcode_capture_dropped_min -ne $s.barcode_capture_dropped_max) {throw ($surface+' capture failed continuity/controller checks; stop here. Logs retained.')}
        $path=Join-Path $outputPath ($name+'.log')
        $text=Get-Content -LiteralPath $path -Raw
        $rows=[regex]::Matches($text,'(?m)^\[S3B\] t=(\d+) raw=(\d+),(\d+),(\d+) mask=(\d+) dropped=(\d+)\r?$')
        if($rows.Count -ne $s.barcode_capture_rows -or $rows.Count -ne [regex]::Matches($text,'\[S3B\]').Count){throw 'Capture log/summary disagreement.'}
        $values=@($rows|ForEach-Object{[int]$_.Groups[4].Value}|Sort-Object)
        $stats=$values|Measure-Object -Minimum -Maximum -Average
        $median=if($values.Count%2){[double]$values[[int][Math]::Floor($values.Count/2)]}else{($values[$values.Count/2-1]+$values[$values.Count/2])/2.0}
        $wrong=0
        foreach($value in $values) {
            $darkness=[Math]::Max(0,[Math]::Min(1000,[Math]::Truncate(($value-$record.reference_white)*1000.0/($record.reference_black-$record.reference_white))))
            if(($surface -eq 'WHITE' -and $darkness -gt $record.threshold_low) -or ($surface -eq 'BLACK' -and $darkness -lt $record.threshold_high)){$wrong++}
        }
        $record.captures+=@([pscustomobject]@{surface=$surface;log=$name+'.log';sha256=(Get-FileHash -LiteralPath $path -Algorithm SHA256).Hash;scans=$values.Count;minimum=[int]$stats.Minimum;maximum=[int]$stats.Maximum;mean=[Math]::Round($stats.Average,3);median=$median;range=[int]($stats.Maximum-$stats.Minimum);current_calibration_label_mismatches=$wrong})
        Save-Result
    }
    $white=$record.captures[0];$black=$record.captures[1]
    $contrast=$black.median-$white.median
    $record.observed_contrast=$contrast
    $record.stability_limit=$contrast/4.0
    $record.stable=$contrast -gt 0 -and $white.range -le $record.stability_limit -and $black.range -le $record.stability_limit
    $record.status=if($record.stable -and $white.current_calibration_label_mismatches -eq 0 -and $black.current_calibration_label_mismatches -eq 0){'PASS_STATIONARY_GP28_REFLECTANCE'}else{'NEEDS_GEOMETRY_OR_CALIBRATION_REVIEW'}
    Write-Host ('Finished: '+$record.status+'. WHITE='+$white.minimum+'..'+$white.maximum+'; BLACK='+$black.minimum+'..'+$black.maximum+'.')
    Write-Host 'This checks stationary reflectance only. No endpoint, decoder limit or firmware was changed. Tell me done so I can review it.'
} catch {$record.status='ERROR';$record.error=$_.Exception.Message;throw}
finally {Save-Result}
