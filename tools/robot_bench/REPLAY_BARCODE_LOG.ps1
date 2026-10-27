param(
    [Parameter(Mandatory=$true)][string]$LogPath,
    [Parameter(Mandatory=$true)][string]$SummaryPath,
    [Parameter(Mandatory=$true)][ValidateSet('A','Z')][string]$Symbol,
    [string]$RepoRoot,
    [string]$OutputDirectory )
$ErrorActionPreference='Stop'
Set-StrictMode -Version Latest

. (Join-Path $PSScriptRoot 'bench_common.ps1')
$benchContext=Get-RobotBenchContext -ToolsDirectory $PSScriptRoot -RepoRoot $RepoRoot -OutputDirectory $OutputDirectory
$repoPath=$benchContext.Repository
$outputPath=$benchContext.Output
$base=[System.IO.Path]::ChangeExtension([System.IO.Path]::GetFullPath($LogPath),$null).TrimEnd('.')
$result=[ordered]@{status='ERROR';expected=$Symbol;commands_sent=0;onboard_decoding_validated=$false;motion_validated=$false}
try {
    if(Test-Path -LiteralPath (Join-Path $repoPath 'app_program\robot_config.local.h')){throw 'A local board override exists. Reconcile the replay profile before interpreting captures.'}
    $manifest=Get-Content -LiteralPath (Join-Path $outputPath 'BARCODE_REPLAY_BUILD.json') -Raw|ConvertFrom-Json
    $exe=Join-Path $outputPath 'BARCODE_REPLAY_NATIVE.exe'
    if((Get-FileHash -LiteralPath $exe -Algorithm SHA256).Hash -ne $manifest.exe_sha256){throw 'Host decoder executable hash mismatch.'}
    foreach($entry in $manifest.sources.PSObject.Properties) {
        $path=Join-Path $repoPath $entry.Name
        if((Get-FileHash -LiteralPath $path -Algorithm SHA256).Hash -ne $entry.Value){throw ('Host decoder is stale for '+$entry.Name+'. Rebuild the host helper; no firmware flash implied.')}
    }
    $summary=Get-Content -LiteralPath $SummaryPath -Raw|ConvertFrom-Json
    if($summary.status -ne 'CAPTURED' -or $summary.commands_sent -ne 0 -or $summary.procedure -ne 'BarcodeCapture'){throw 'Capture did not finish normally in read-only barcode mode.'}
    $all=Get-Content -LiteralPath $LogPath -Raw
    $controller=[regex]::Matches($all,'(?m)^\[CSC02\] t=(\d+) phase=(\d+) fault=(\d+) armed=(\d+) cmd=(\d+) power=(\d+) flags=(\d+) bytes=(\d+)\r?$')
    if($controller.Count -lt 1 -or [regex]::Matches($all,'\[CSC02\]').Count -ne $controller.Count){throw 'Complete controller status is required.'}
    foreach($row in $controller) {
        if($row.Groups[3].Value -ne '0' -or $row.Groups[4].Value -ne '0' -or $row.Groups[5].Value -ne '0' -or $row.Groups[6].Value -ne '0'){throw 'Controller fault or nonzero drive status; stop the test.'}
    }
    $rows=[regex]::Matches($all,'(?m)^\[S3B\] t=(\d+) raw=(\d+),(\d+),(\d+) mask=(\d+) dropped=(\d+)\r?$')
    if($rows.Count -lt 100 -or [regex]::Matches($all,'\[S3B\]').Count -ne $rows.Count -or $all.Contains('[S3BERR]')){throw 'Incomplete barcode recording; diagnostics retained.'}
    $csv=[System.Collections.Generic.List[string]]::new()
    foreach($row in $rows) {$csv.Add((1..6|ForEach-Object{$row.Groups[$_].Value}) -join ',')}
    # Preserve every original timestamp, mask, ADC value and drop counter.
    [System.IO.File]::WriteAllLines(($base+'_REPLAY.csv'),$csv,[System.Text.Encoding]::ASCII)
    $native=@(& $exe ($base+'_REPLAY.csv') $Symbol 2>&1)
    $nativeExit=$LASTEXITCODE
    $native|Set-Content -LiteralPath ($base+'_REPLAY.log') -Encoding UTF8
    if($nativeExit -ne 0 -and $nativeExit -ne 2){throw ('Host decoder input/execution error '+$nativeExit)}
    $text=$native -join "`n"
    $stats=[regex]::Match($text,'(?m)^SUMMARY rows=(\d+) segments=(\d+) discontinuities=(\d+) rejected=(\d+) decoded=(\d+) expected=(\d+) wrong=(\d+) max_gap_ms=30 timeout_ms=10000\r?$')
    if(-not $stats.Success -or [int]$stats.Groups[1].Value -ne $rows.Count){throw 'Host decoder returned incomplete results.'}
    $result.log_sha256=(Get-FileHash -LiteralPath $LogPath -Algorithm SHA256).Hash
    $result.rows=$rows.Count
    $result.discontinuities=[int]$stats.Groups[3].Value
    $result.rejected=[int]$stats.Groups[4].Value
    $result.decoded=[int]$stats.Groups[5].Value
    $result.expected_matches=[int]$stats.Groups[6].Value
    $result.wrong_matches=[int]$stats.Groups[7].Value
    $result.native_exit=$nativeExit
    $result.scope='Physical ADC recording replayed by production GP28 normalization/hysteresis/barcode module on this PC.'
    $result.status=if($nativeExit -eq 0 -and $result.discontinuities -eq 0 -and $result.expected_matches -gt 0 -and $result.wrong_matches -eq 0){'PASS_SENSOR_CAPTURE_NATIVE_REPLAY'}else{'NOT_DECODED'}
    if($result.status -eq 'PASS_SENSOR_CAPTURE_NATIVE_REPLAY') {Write-Host ('PASS: '+$Symbol+' decoded from this recording on the PC. Onboard decoding and driving are still untested.') -ForegroundColor Green}
    else {
        Write-Host ('NOT DECODED: '+$Symbol+'. No tolerances were changed.') -ForegroundColor Yellow
        if($result.discontinuities -gt 0){Write-Host 'Recording contains timing/validity/drop discontinuities; do not interpret it as a clean optical test.'}
        elseif($text -match 'REJECT status=6 '){Write-Host 'Sampling is continuous; the measured bar/space pattern failed width checks. Keep one steady pass, away from the printed letter.'}
        elseif($text -match 'REJECT status=5 '){Write-Host 'Sweep exceeded the 10-second decoder timeout.'}
        else{Write-Host 'Keep WHITE before/after a complete single sweep; hold still on WHITE after finishing.'}
    }
} catch {$result.error=$_.Exception.Message;throw}
finally {$result|ConvertTo-Json -Depth 5|Set-Content -LiteralPath ($base+'_REPLAY_RESULT.json') -Encoding UTF8}
[pscustomobject]$result
