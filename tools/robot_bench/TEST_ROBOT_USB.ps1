param(
    [ValidatePattern('^COM[0-9]+$')][string]$Port,
    [ValidateRange(5,120)][int]$Seconds = 30,
    [ValidateRange(6,15)][int]$ScanWindowSeconds = 6,
    [ValidateSet('Imu','Wiring','Ultrasonic','UltrasonicStill','IRWhite','IRBlack','IRMap','IRCalibrated','BarcodeCapture','EncoderStill','EncoderLeft','EncoderRight')][string]$Procedure = 'Imu',
    [ValidatePattern('^[A-Za-z0-9_-]{1,60}$')][string]$LogName = 'ROBOT_USB_TEST',
    [ValidateSet('WHITE','BLACK')][string]$BarcodeHold,
    [string]$RepoRoot,
    [string]$OutputDirectory )
$ErrorActionPreference = 'Stop'
Set-StrictMode -Version Latest

. (Join-Path $PSScriptRoot 'bench_common.ps1')
$benchContext=Get-RobotBenchContext -ToolsDirectory $PSScriptRoot -RepoRoot $RepoRoot -OutputDirectory $OutputDirectory
$repoPath=$benchContext.Repository
$outputPath=$benchContext.Output
$logPath = Join-Path $outputPath ($LogName+'.log')
$summaryPath = Join-Path $outputPath ($LogName+'_RESULT.json')
$record = [ordered]@{ started = (Get-Date).ToString('o'); seconds = $Seconds; procedure = $Procedure; commands_sent = 0; port = $Port; status = 'NOT_OPENED' }
$device = $null
$writer = $null
try {
    if ($BarcodeHold -and $Procedure -ne 'BarcodeCapture') { throw 'BarcodeHold requires Procedure BarcodeCapture.' }
    if ($BarcodeHold) { $record.barcode_hold = $BarcodeHold.ToUpperInvariant() }
    if ($Procedure -eq 'BarcodeCapture' -and -not $BarcodeHold) { $record.scan_window_seconds = $ScanWindowSeconds }
    if (-not $Port) {
        $candidates = @()
        $livePorts = @([System.IO.Ports.SerialPort]::GetPortNames())
        if ($livePorts.Count -eq 0) { throw 'Windows detects no USB serial ports. Reconnect the robot by normal USB with BOOTSEL released, then rerun. No recording has started.' }
        try {
            # VID/PID alone also matches the separate LoRaWAN Pico. Match the
            # robot's firmware serial identity to its CDC child by ContainerID.
            $usbRoot = 'Registry::HKEY_LOCAL_MACHINE\SYSTEM\CurrentControlSet\Enum\USB'
            $robot = Get-ItemProperty -LiteralPath ($usbRoot+'\VID_2E8A&PID_000A\MTK3PICO') -ErrorAction Stop
            if (-not $robot.ContainerID) { throw 'Robot USB identity has no container ID.' }
            $candidates = @(Get-ChildItem -LiteralPath ($usbRoot+'\VID_2E8A&PID_000A&MI_00') -ErrorAction Stop |
                ForEach-Object {
                    $identity = Get-ItemProperty -LiteralPath $_.PSPath -ErrorAction Stop
                    if ($identity.ContainerID -eq $robot.ContainerID -and $identity.Service -eq 'usbser') {
                        $parameters = Get-ItemProperty -LiteralPath (Join-Path $_.PSPath 'Device Parameters') -ErrorAction Stop
                        if ($parameters.PortName -in $livePorts) { [string]$parameters.PortName }
                    }
                })
            $record.identity_check = 'MTK3PICO serial identity + matching USB ContainerID + live COM port'
        } catch { $candidates = @() }
        $candidates = @($candidates | Sort-Object -Unique)
        if ($candidates.Count -ne 1) {
            $visible = [System.IO.Ports.SerialPort]::GetPortNames() -join ', '
            throw ('Could not identify exactly one robot USB port. Unplug/replug WITHOUT BOOTSEL; close other serial monitors. Available ports: ' + $visible + '. If needed, rerun with -Port COMnumber after identifying the robot in Device Manager.')
        }
        $Port = $candidates[0]
    }
    $record.port = [string]$Port
    Write-Host ('Reading ' + $Port + ' at 115200 for ' + $Seconds + ' seconds. No commands will be sent.')
    if ($Procedure -eq 'Wiring') {
        Write-Host 'Keep drive motor power OFF, support the robot with wheels clear, and keep the chassis still.'
        Write-Host 'Once CAPTURE START prints: 0-15 s leave wheels still; place a flat target about 40 cm in front of the ultrasonic sensor.'
        Write-Host '15-30 s manually rotate LEFT wheel only; 30-45 s manually rotate RIGHT wheel only.'
        Write-Host '45-60 s leave wheels still and move the flat target between about 10 and 30 cm. The servo may scan; keep fingers clear.'
    } elseif ($Procedure -in @('Ultrasonic','UltrasonicStill')) {
        Write-Host 'Keep the existing sensor wiring/supply unchanged and the drive lock ON. Support the robot, keeping chassis and wheels still.'
        if ($Procedure -eq 'UltrasonicStill') {
            Write-Host 'Place a large flat target 40-60 cm straight ahead, then leave target, robot and both wheels still for the entire capture. Keep the target beyond 25 cm; the servo may initialize.'
        } else {
            Write-Host 'After CAPTURE START, hold a large flat target straight ahead about 40 cm from the sensor for 10 seconds.'
            Write-Host 'At the prompts move it to about 60 cm, then back to about 40 cm. Keep it beyond 25 cm throughout; the servo may initialize.'
        }
    } elseif ($Procedure -in @('IRWhite','IRBlack')) {
        $surface = if ($Procedure -eq 'IRWhite') { 'WHITE' } else { 'BLACK' }
        Write-Host ('IR input-only test: drive lock stays ON. Keep robot and wheels stationary, with a uniform '+$surface+' surface under all three IR sensing windows at their normal working height. Do not turn the adjustment screws between captures.')
    } elseif ($Procedure -eq 'IRMap') {
        Write-Host 'IR mapping capture: keep the surface arrangement requested by the mapping helper and hold the robot still. No movement commands are sent.'
    } elseif ($Procedure -eq 'IRCalibrated') {
        Write-Host 'Calibrated IR observation: keep the selected targets and robot still, with drive lock ON. No commands sent.'
    } elseif ($Procedure -eq 'BarcodeCapture') {
        if ($BarcodeHold) {
            Write-Host ('Stationary GP28 '+$BarcodeHold+' check. Keep that uniform target still at the intended barcode plane/distance until capture ends. No sweep or scan beep.')
        } else {
        Write-Host 'No line-sensor patches required. Keep the sensing height fixed and begin with WHITE facing GP28. WAIT for SCAN NOW/start beep.'
        if ($ScanWindowSeconds -gt 6) {
            Write-Host ('At the scan beep, use the '+$ScanWindowSeconds+'-second window for one pass. Aim for 5 seconds from the FIRST barcode bar to the LAST, moving the sheet left/right across the bars. Finish on blank WHITE and keep the same sensing height.')
        } else {
            Write-Host 'At SCAN NOW/start beep, pass the WHOLE selected barcode across GP28 steadily in 4-6 seconds by moving the card or the unpowered robot. Finish on WHITE before the second beep. Keep its calibrated plane/distance.'
        }
        Write-Host 'End on WHITE until capture ends. Keep motor power disconnected; no robot commands are sent.'
        }
    } elseif ($Procedure -eq 'EncoderStill') {
        Write-Host 'Keep motor power OFF, wheels supported clear, and both wheels still throughout capture.'
    } elseif ($Procedure -eq 'EncoderLeft' -or $Procedure -eq 'EncoderRight') {
        $wheel = if ($Procedure -eq 'EncoderLeft') { 'LEFT' } else { 'RIGHT' }
        Write-Host ('Keep motor power OFF and wheels supported. Wait for CAPTURE START, then manually rotate only the '+$wheel+' wheel forward several turns until capture ends. Keep the other wheel still.')
    } else {
        Write-Host 'Keep motor power OFF. Rest flat for 5 seconds, gently tilt forward/back and left/right, then rest flat again.'
    }
    $device = New-Object System.IO.Ports.SerialPort
    $device.PortName = $Port
    $device.BaudRate = 115200
    $device.DataBits = 8
    $device.Parity = [System.IO.Ports.Parity]::None
    $device.StopBits = [System.IO.Ports.StopBits]::One
    $device.Handshake = [System.IO.Ports.Handshake]::None
    $device.DtrEnable = $true
    $device.ReadTimeout = 200
    $device.Open()
    $record.status = 'OPENED'
    # Let the firmware's bounded USB ring drain old pre-connection output.
    $warmup = [System.Diagnostics.Stopwatch]::StartNew()
    $discarded = 0
    $warmupText = New-Object System.Text.StringBuilder
    while ($warmup.Elapsed.TotalSeconds -lt 2) {
        $warmupChunk = $device.ReadExisting()
        $discarded += $warmupChunk.Length
        if ($Procedure -in @('IRWhite','IRBlack','IRMap','IRCalibrated','BarcodeCapture')) { [void]$warmupText.Append($warmupChunk) }
        Start-Sleep -Milliseconds 100
    }
    $record.warmup_seconds = 2
    $record.discarded_warmup_characters = $discarded
    if ($Procedure -in @('IRWhite','IRBlack','IRMap','IRCalibrated','BarcodeCapture')) {
        $record.warmup_log = Join-Path $outputPath ($LogName+'_WARMUP.log')
        [System.IO.File]::WriteAllText($record.warmup_log,$warmupText.ToString(),[System.Text.Encoding]::UTF8)
    }
    $writer = New-Object System.IO.StreamWriter($logPath,$false,[System.Text.Encoding]::UTF8)
    $writer.AutoFlush = $true
    $text = New-Object System.Text.StringBuilder
    Write-Host 'CAPTURE START'
    $timer = [System.Diagnostics.Stopwatch]::StartNew()
    $nextCue = if ($Procedure -eq 'BarcodeCapture') { 5 } elseif ($Procedure -eq 'Ultrasonic') { 10 } else { 15 }
    if ($BarcodeHold) { $nextCue = 1000 }
    $record.phase_cues = @()
    while ($timer.Elapsed.TotalSeconds -lt $Seconds) {
        if ((($Procedure -eq 'Wiring' -and $nextCue -le 45) -or ($Procedure -eq 'Ultrasonic' -and $nextCue -le 20)) -and $timer.Elapsed.TotalSeconds -ge $nextCue) {
            $phase = if ($Procedure -eq 'Ultrasonic') {
                if ($nextCue -eq 10) { 'TARGET_60_CM: move the flat target to about 60 cm straight ahead; keep robot still.' }
                else { 'TARGET_40_CM: return the flat target to about 40 cm straight ahead; keep robot still.' }
            } else { switch ($nextCue) {
                15 { 'LEFT_WHEEL: manually rotate only the left wheel; keep chassis still.' }
                30 { 'RIGHT_WHEEL: stop the left wheel; manually rotate only the right wheel.' }
                45 { 'ULTRASONIC_TARGET: stop both wheels; move a flat target between about 10 and 30 cm.' }
            } }
            Write-Host ("`nTEST PHASE: " + $phase)
            $record.phase_cues += [pscustomobject]@{elapsed_seconds=[Math]::Round($timer.Elapsed.TotalSeconds,2);phase=[string]$phase}
            $nextCue += $(if ($Procedure -eq 'Ultrasonic') { 10 } else { 15 })
        }
        if ($Procedure -eq 'BarcodeCapture' -and $nextCue -eq 5 -and $timer.Elapsed.TotalSeconds -ge 5) {
            Write-Host ('SCAN NOW: start the WHOLE barcode sweep; finish on WHITE within '+$ScanWindowSeconds+' seconds.')
            try { [Console]::Beep(750,150) } catch { }
            $record.phase_cues += [pscustomobject]@{elapsed_seconds=[Math]::Round($timer.Elapsed.TotalSeconds,2);phase='BARCODE_SCAN'}
            $nextCue=5+$ScanWindowSeconds
        }
        if ($Procedure -eq 'BarcodeCapture' -and $nextCue -eq (5+$ScanWindowSeconds) -and $timer.Elapsed.TotalSeconds -ge (5+$ScanWindowSeconds)) {
            Write-Host 'FINISH: keep WHITE facing GP28 until capture ends.'
            try { [Console]::Beep(1000,200) } catch { }
            $record.phase_cues += [pscustomobject]@{elapsed_seconds=[Math]::Round($timer.Elapsed.TotalSeconds,2);phase='BARCODE_FINISH'}
            $nextCue=1000
        }
        $chunk = $device.ReadExisting()
        if ($chunk.Length -gt 0) {
            if ($Procedure -ne 'BarcodeCapture') { Write-Host -NoNewline $chunk }
            $writer.Write($chunk)
            [void]$text.Append($chunk)
        }
        Start-Sleep -Milliseconds 100
    }
    $record.status = 'CAPTURED'
    $all = $text.ToString()
    # Only complete records can establish commanded motor state. A truncated
    # prefix is serial damage, not evidence that a motor was commanded to move.
    $statusRows = [regex]::Matches($all,'(?m)\[CSC02\] t=(\d+) phase=(\d+) fault=(\d+) armed=(\d+) cmd=(\d+) power=(\d+) flags=(\d+) bytes=(\d+)\r?$')
    $markers = [regex]::Matches($all,'\[CSC02\]').Count
    $unexpected = @($statusRows | Where-Object { $_.Groups[4].Value -ne '0' -or $_.Groups[5].Value -ne '0' -or $_.Groups[6].Value -ne '0' })
    $faults = @($statusRows | Where-Object { $_.Groups[3].Value -ne '0' })
    $record.controller_lines = $statusRows.Count
    $record.incomplete_controller_records = $markers - $statusRows.Count
    $record.controller_fault_records = $faults.Count
    $record.unexpected_motor_status_lines = $unexpected.Count
    $record.motor_lock_banner_seen = $all.Contains('MOTOR LOCK ON')
    $imuRows = [regex]::Matches($all,'(?m)\[S4\] (-?\d+(?:,-?\d+){16})\r?$')
    $record.imu_rows = $imuRows.Count
    $record.accel_usable_seen = @($imuRows | Where-Object { $_.Groups[1].Value.Split(',')[2] -eq '1' }).Count -gt 0
    $record.encoder_diagnostic_records = [regex]::Matches($all,'(?m)^\[S2ENC\] valid=\d+ m1=-?\d+ m2=-?\d+ m1ab=[01]{2} m2ab=[01]{2}\r?$').Count
    $sonarRows = [regex]::Matches($all,'(?m)^\[S5RAW\] copied=(\d+) t=(\d+) state=(\d+) fresh=(\d+) valid=(\d+) mm=(\d+) angle=(-?\d+) sampled=(-?\d+) echo=(\d+)(?: error=(-?\d+))?\r?$')
    $record.ultrasonic_diagnostic_records = $sonarRows.Count
    $record.sonar_status_copies = @($sonarRows | Where-Object { $_.Groups[1].Value -eq '1' }).Count
    $record.sonar_snapshot_unavailable = $sonarRows.Count - $record.sonar_status_copies
    $record.sonar_copy_error_codes = [ordered]@{}
    foreach ($row in $sonarRows) {
        if ($row.Groups[10].Success) {
            $copyCode = [string]$row.Groups[10].Value
            if (-not $record.sonar_copy_error_codes.Contains($copyCode)) { $record.sonar_copy_error_codes[$copyCode] = 0 }
            $record.sonar_copy_error_codes[$copyCode]++
        }
    }
    $record.sonar_no_rise_records = @($sonarRows | Where-Object { $_.Groups[1].Value -eq '1' -and $_.Groups[9].Value -eq '2' }).Count
    $validSonar = @($sonarRows | Where-Object { $_.Groups[1].Value -eq '1' -and $_.Groups[4].Value -eq '1' -and $_.Groups[5].Value -eq '1' -and $_.Groups[9].Value -eq '0' })
    $record.sonar_valid_records = $validSonar.Count
    $record.sonar_mm_min = $null
    $record.sonar_mm_max = $null
    if ($validSonar.Count -gt 0) {
        $ranges = @($validSonar | ForEach-Object { [int]$_.Groups[6].Value }) | Measure-Object -Minimum -Maximum
        $record.sonar_mm_min = [int]$ranges.Minimum
        $record.sonar_mm_max = [int]$ranges.Maximum
    }
    $irRows = [regex]::Matches($all,'(?m)^\[S3RAW\] copied=(\d+) error=(-?\d+) t=(\d+) acquisition=(-?\d+) mask=(\d+) raw=(\d+),(\d+),(\d+) configured=(\d+) steering=(\d+) fresh=(\d+)\r?$')
    $record.ir_diagnostic_records = $irRows.Count
    $validIr = @($irRows | Where-Object { $_.Groups[1].Value -eq '1' -and $_.Groups[2].Value -eq '0' -and $_.Groups[4].Value -eq '0' -and $_.Groups[5].Value -eq '7' -and $_.Groups[11].Value -eq '1' -and [int]$_.Groups[6].Value -le 4095 -and [int]$_.Groups[7].Value -le 4095 -and [int]$_.Groups[8].Value -le 4095 })
    $record.ir_complete_scans = $validIr.Count
    $record.ir_unique_complete_scans = @($validIr | ForEach-Object { $_.Groups[3].Value } | Sort-Object -Unique).Count
    $record.ir_unexpected_authority = @($irRows | Where-Object { $_.Groups[9].Value -ne '0' -or $_.Groups[10].Value -ne '0' }).Count
    if ($Procedure -eq 'IRCalibrated') {
        $calRows = [regex]::Matches($all,'(?m)^\[S3IR\] copied=(\d+) error=(-?\d+) t=(\d+) acquisition=(-?\d+) mask=(\d+) raw=(\d+),(\d+),(\d+) configured=(\d+) steering=(\d+) fresh=(\d+) line=(\d+) blackmask=(\d+) lineerror=(-?\d+) barcode_darkness=(\d+) algorithm_error=(\d+)(?: line_darkness=(\d+),(\d+))?\r?$')
        $acquiredCal = @($calRows | Where-Object { $_.Groups[1].Value -eq '1' -and $_.Groups[2].Value -eq '0' -and $_.Groups[4].Value -eq '0' -and $_.Groups[5].Value -eq '7' -and $_.Groups[9].Value -eq '1' -and $_.Groups[11].Value -eq '1' -and [int]$_.Groups[6].Value -le 4095 -and [int]$_.Groups[7].Value -le 4095 -and [int]$_.Groups[8].Value -le 4095 })
        $validCal = @($acquiredCal | Where-Object { $_.Groups[16].Value -eq '0' -and $_.Groups[17].Success -and $_.Groups[18].Success -and [int]$_.Groups[17].Value -le 1000 -and [int]$_.Groups[18].Value -le 1000 })
        $record.ir_calibrated_acquired_rows = $acquiredCal.Count
        $record.ir_calibrated_algorithm_errors = [ordered]@{}
        foreach($row in $calRows) {
            $code=[string]$row.Groups[16].Value
            if(-not $record.ir_calibrated_algorithm_errors.Contains($code)) { $record.ir_calibrated_algorithm_errors[$code]=0 }
            $record.ir_calibrated_algorithm_errors[$code]++
        }
        $record.ir_calibrated_rows = $calRows.Count
        $record.ir_calibrated_incomplete_records = [regex]::Matches($all,'\[S3IR\]').Count - $calRows.Count
        $record.ir_calibrated_valid_rows = $validCal.Count
        $record.ir_calibrated_unique_scans = @($validCal | ForEach-Object { $_.Groups[3].Value } | Sort-Object -Unique).Count
        $record.ir_unexpected_steering = @($calRows | Where-Object { $_.Groups[10].Value -ne '0' }).Count
        $record.ir_calibrated_observations = @($validCal | ForEach-Object { [pscustomobject]@{t=[uint32]$_.Groups[3].Value;line=[int]$_.Groups[12].Value;blackmask=[int]$_.Groups[13].Value;lineerror=[int]$_.Groups[14].Value;barcode_darkness=[int]$_.Groups[15].Value;right_darkness=[int]$_.Groups[17].Value;left_darkness=[int]$_.Groups[18].Value} })
        Write-Host ('Calibrated IR: '+$record.ir_calibrated_valid_rows+' valid records; unexpected steering: '+$record.ir_unexpected_steering)
    }
    if ($Procedure -eq 'BarcodeCapture') {
        $barcodeRows=[regex]::Matches($all,'(?m)^\[S3B\] t=(\d+) raw=(\d+),(\d+),(\d+) mask=(\d+) dropped=(\d+)\r?$')
        $validBarcode=@($barcodeRows | Where-Object { $_.Groups[5].Value -eq '7' -and [int64]$_.Groups[1].Value -le [uint32]::MaxValue -and [int64]$_.Groups[6].Value -le [uint32]::MaxValue -and [int]$_.Groups[2].Value -le 4095 -and [int]$_.Groups[3].Value -le 4095 -and [int]$_.Groups[4].Value -le 4095 })
        $record.barcode_capture_rows=$barcodeRows.Count
        $record.barcode_capture_valid_rows=$validBarcode.Count
        $record.barcode_capture_unique_scans=@($validBarcode|ForEach-Object{$_.Groups[1].Value}|Sort-Object -Unique).Count
        $record.barcode_capture_incomplete_records=[regex]::Matches($all,'\[S3B\]').Count-$barcodeRows.Count
        $record.barcode_capture_errors=[regex]::Matches($all,'\[S3BERR\]').Count
        $record.barcode_capture_gap_records=0
        $record.barcode_capture_dropped_min=$null
        $record.barcode_capture_dropped_max=$null
        $previousBarcode=$null
        foreach($row in $validBarcode) {
            $timestamp=[int64]$row.Groups[1].Value
            if($null -ne $previousBarcode) {
                $delta=($timestamp-$previousBarcode+4294967296)%4294967296
                if($delta -eq 0 -or $delta -gt 30){$record.barcode_capture_gap_records++}
            }
            $previousBarcode=$timestamp
        }
        if($validBarcode.Count -gt 0) {
            $drops=@($validBarcode|ForEach-Object{[uint32]$_.Groups[6].Value})|Measure-Object -Minimum -Maximum
            $record.barcode_capture_dropped_min=$drops.Minimum;$record.barcode_capture_dropped_max=$drops.Maximum
        }
        $captureScope = if ($BarcodeHold) { 'Stationary reflectance only; no symbol-decoding test.' } else { 'Decoder replay is still pending.' }
        Write-Host ('Barcode capture: '+$record.barcode_capture_valid_rows+' valid scans; timing gaps='+$record.barcode_capture_gap_records+'. '+$captureScope)
    }
    if ($record.incomplete_controller_records -gt 0) { $record.status = 'CAPTURED_WITH_SERIAL_DAMAGE' }
    if ($statusRows.Count -eq 0) { $record.status = 'NO_CONTROLLER_STATUS' }
    if ($faults.Count -gt 0) { $record.status = 'CONTROLLER_FAULT_REPORTED' }
    if ($unexpected.Count -gt 0) { $record.status = 'UNEXPECTED_MOTOR_STATUS' }
    Write-Host "`nSaved log: $logPath"
    Write-Host ('Controller reports: ' + $record.controller_lines + '; unexpected drive status: ' + $record.unexpected_motor_status_lines)
    Write-Host ('IMU rows: ' + $record.imu_rows + '; usable accelerometer seen: ' + $record.accel_usable_seen)
    Write-Host ('Sonar: ' + $record.sonar_valid_records + ' fresh valid records; ' + $record.sonar_no_rise_records + ' no-rise records; ' + $record.sonar_status_copies + ' successful copies')
    if ($Procedure -in @('IRWhite','IRBlack','IRMap')) {
        Write-Host ('IR: '+$record.ir_complete_scans+' complete raw scans; unexpected calibrated/steering authority: '+$record.ir_unexpected_authority)
    }
    Write-Host 'A boot banner can print before the monitor opens. Log capture is not proof of sensor accuracy or electrical motor shutdown.'
} catch {
    $record.status = 'ERROR'
    $record.error = $_.Exception.Message
    throw
} finally {
    if ($null -ne $writer) { $writer.Dispose() }
    if ($null -ne $device) { if ($device.IsOpen) { $device.Close() }; $device.Dispose() }
    $record | ConvertTo-Json -Depth 5 | Set-Content -LiteralPath $summaryPath -Encoding UTF8
}
