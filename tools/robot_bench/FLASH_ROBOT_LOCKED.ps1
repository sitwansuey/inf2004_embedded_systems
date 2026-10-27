param(
    [ValidatePattern('^[D-Zd-z]$')][string]$DriveLetter = 'D',
    [ValidateSet('Default','IRRawAdc','IRRawDigital','IRCalibrated','IRBarcodeCapture')][string]$FirmwareProfile = 'Default',
    [switch]$BuildOnly,
    [switch]$Flash,
    [Parameter(Mandatory=$true)][string]$PicoSdkPath,
    [string]$RepoRoot,
    [string]$OutputDirectory )
$ErrorActionPreference = 'Stop'
Set-StrictMode -Version Latest

. (Join-Path $PSScriptRoot 'bench_common.ps1')
$benchContext=Get-RobotBenchContext -ToolsDirectory $PSScriptRoot -RepoRoot $RepoRoot -OutputDirectory $OutputDirectory
$repoPath=$benchContext.Repository
$outputPath=$benchContext.Output
$sdkPath = (Resolve-Path -LiteralPath $PicoSdkPath).Path
if ($Flash -and $BuildOnly) { throw 'Choose Flash or BuildOnly, not both.' }
$BuildOnly = -not $Flash
$drivePath = $DriveLetter.ToUpperInvariant() + ':\'
$record = [ordered]@{ started = (Get-Date).ToString('o'); drive = $drivePath; motor_lock = $true; flashed = $false; firmware_profile = $FirmwareProfile }
$resultName = if ($BuildOnly) { 'ROBOT_BUILD_RESULT.json' } else { 'ROBOT_FLASH_RESULT.json' }
$transcriptName = if ($BuildOnly) { 'ROBOT_BUILD_TRANSCRIPT.log' } else { 'ROBOT_FLASH_TRANSCRIPT.log' }
Start-Transcript -LiteralPath (Join-Path $outputPath $transcriptName) -Force | Out-Null
try {
    Write-Host 'Robot Pico W only. Keep motor power off. No movement commands will be sent.'
    if (-not $BuildOnly) {
    $infoPath = [System.IO.Path]::Combine($drivePath,'INFO_UF2.TXT')
    if (-not [System.IO.File]::Exists($infoPath)) {
        throw ('Robot BOOTSEL drive ' + $drivePath + ' is not ready. With motor power off, unplug USB, hold BOOTSEL while reconnecting, then release. Wait for RPI-RP2 in File Explorer. If its letter differs, rerun with that -DriveLetter. Nothing copied.')
    }
    $boardInfo = Get-Content -LiteralPath $infoPath -Raw
    # RP2040 ROM says "Raspberry Pi RP2", not "RP2040" in this file.
    if ($boardInfo -notmatch '(?m)^Board-ID:\s*RPI-RP2\s*$' -or
        $boardInfo -notmatch '(?m)^Model:\s*Raspberry Pi RP2\s*$') {
        throw 'Target is not the expected RP2040 BOOTSEL board. Nothing copied.'
    }
    # Store only verified identity; Get-Content attaches provider metadata to strings.
    $record.board_info = 'Model: Raspberry Pi RP2; Board-ID: RPI-RP2'
    Write-Host $boardInfo
    }
    if (Test-Path -LiteralPath (Join-Path $repoPath 'app_program\robot_config.local.h')) {
        throw 'A local board override exists. Review its motor-lock setting before flashing.'
    }
    $verificationPath = Join-Path $PSScriptRoot 'source_verification.json'
    $verified = Get-Content -LiteralPath $verificationPath -Raw | ConvertFrom-Json
    foreach ($property in $verified.files.PSObject.Properties) {
        $sourcePath = Join-Path $repoPath $property.Name
        if ((Get-FileHash -LiteralPath $sourcePath -Algorithm SHA256).Hash -ne $property.Value) {
            throw ('Source changed since motor-lock tests: ' + $property.Name + '. Nothing copied.')
        }
    }
    $profileSuffix = switch ($FirmwareProfile) { 'IRRawAdc' { '_irraw2' } 'IRRawDigital' { '_irraw1' } 'IRCalibrated' { '_ircal' } 'IRBarcodeCapture' { '_irbarcode' } default { '' } }
    $firmwarePath = Join-Path $repoPath ('build_make\mtk3pico_smp1_usb_cdc_wifi_enc'+$profileSuffix+'.uf2')
    $previousHash = if (Test-Path -LiteralPath $firmwarePath) { (Get-FileHash -LiteralPath $firmwarePath -Algorithm SHA256).Hash } else { '' }
    & (Join-Path $repoPath 'tools\build_windows.ps1') -PicoSdkPath $sdkPath -Profile $FirmwareProfile
    if ($LASTEXITCODE -ne 0) { throw 'Firmware build failed. Nothing copied.' }
    foreach ($property in $verified.files.PSObject.Properties) {
        if ((Get-FileHash -LiteralPath (Join-Path $repoPath $property.Name) -Algorithm SHA256).Hash -ne $property.Value) {
            throw ('Source changed during build: ' + $property.Name + '. Nothing copied.')
        }
    }
    $firmware = [System.IO.File]::ReadAllBytes($firmwarePath)
    if ($firmware.Length -eq 0 -or $firmware.Length % 512 -ne 0) { throw 'Invalid UF2 length.' }
    $blockCount = [int]($firmware.Length / 512)
    $payload = New-Object System.IO.MemoryStream
    for ($block = 0; $block -lt $blockCount; $block++) {
        $offset = $block * 512
        $word = { param([int]$index) [BitConverter]::ToUInt32($firmware, $offset + $index) }
        if ((& $word 0) -ne 0x0A324655 -or (& $word 4) -ne [Convert]::ToUInt32('9E5D5157',16) -or (& $word 508) -ne 0x0AB16F30) { throw 'Invalid UF2 magic.' }
        if ((& $word 8) -ne 0x2000 -or (& $word 16) -ne 256 -or (& $word 20) -ne $block -or (& $word 24) -ne $blockCount -or (& $word 28) -ne [Convert]::ToUInt32('E48BFF56',16)) { throw 'Invalid UF2 block or RP2040 family.' }
        $address = & $word 12
        if ($address -lt 0x10000000 -or $address + 256 -gt 0x10200000) { throw 'UF2 writes outside Pico W application flash.' }
        if ($address -ne 0x10000000 + $block * 256) { throw 'Unexpected UF2 payload ordering.' }
        $payload.Write($firmware, $offset + 32, 256)
    }
    $payloadText = [System.Text.Encoding]::ASCII.GetString($payload.ToArray())
    $payload.Dispose()
    if ($payloadText -notmatch 'MOTOR LOCK ON') {
        throw 'Motor-lock banner is absent. Refusing to copy older/unlocked firmware.'
    }
    if ($verified.PSObject.Properties.Name -contains 'monotonic_uptime' -and $verified.monotonic_uptime -and
        $payloadText -notmatch 'Clock: kernel operating time \(monotonic uptime\)') {
        throw 'Updated uptime-clock banner is absent. Refusing to copy the previous firmware.'
    }
    if ($verified.PSObject.Properties.Name -contains 'sensor_diagnostics' -and $verified.sensor_diagnostics -and
        ($payloadText -notmatch '\[S2ENC\]' -or $payloadText -notmatch '\[S5RAW\]')) {
        throw 'Sensor diagnostic formats are absent. Refusing to copy older firmware.'
    }
    $currentHash = (Get-FileHash -LiteralPath $firmwarePath -Algorithm SHA256).Hash
    $irMarker = 'IR RAW BENCH: input-only; no calibrated line or barcode authority'
    if ($FirmwareProfile -in @('IRRawAdc','IRRawDigital')) {
        if (-not $payloadText.Contains($irMarker) -or -not $payloadText.Contains('[S3RAW]') -or
            $currentHash -eq 'C0A4A67A31CF1227D7A35A3CC68FA9450F74F03748D55F2F82A1762F863DE49D') {
            throw 'The raw IR bench image is absent or this is the previous sensor image. Nothing copied.'
        }
        $record.ir_raw_bench = $true
        if ($FirmwareProfile -eq 'IRRawAdc' -and
            $verified.PSObject.Properties.Name -contains 'ir_tick_fix' -and $verified.ir_tick_fix) {
            if (-not $payloadText.Contains('[S3] ADC timing: conversion<%ums reply<%ums') -or
                $currentHash -eq '21D0C1651C0C7C157ECAC7A20AEC2CAB7A1A7BB0673D0745D8C7AF9826759878') {
                throw 'The IR polling timing correction is absent or this is the previous timeout image. Nothing copied.'
            }
            $record.ir_tick_fix = $true
        }
        if ($FirmwareProfile -eq 'IRRawAdc' -and
            $verified.PSObject.Properties.Name -contains 'ir_current_scan_fix' -and $verified.ir_current_scan_fix) {
            if (-not $payloadText.Contains('[S3] ADC acquisition: bounded reply wait; no prefetched next scan') -or
                $currentHash -eq 'FD9B11D0220724D92566914818ACEF8A57EC5977971B77F0A06644623A846841') {
                throw 'The current-scan acquisition correction is absent or this is the previous polling image. Nothing copied.'
            }
            $record.ir_current_scan_fix = $true
        }
    } elseif ($FirmwareProfile -in @('IRCalibrated','IRBarcodeCapture')) {
        foreach ($marker in @('IR CALIBRATED BENCH: measured endpoints; motor lock required; no motion test',
            '[S3] roles: GP26=right GP27=left GP28=barcode line-mask=3',
            '[S3] ADC endpoints white=%u,%u,%u black=%u,%u,%u',
            '[S3] paused scan gaps reset observations; TRACK gaps still fault','line_darkness=','[S3IR]')) {
            if (-not $payloadText.Contains($marker)) { throw ('Calibrated IR marker absent: '+$marker+'. Nothing copied.') }
        }
        if ($payloadText.Contains($irMarker)) { throw 'Wrong raw profile selected. Nothing copied.' }
        if ($FirmwareProfile -eq 'IRBarcodeCapture') {
            if (-not $payloadText.Contains('IR BARCODE CAPTURE: copied scans; motor locked; host decoder replay') -or -not $payloadText.Contains('[S3B]') -or
                -not $payloadText.Contains('[S3] scan schedule: cyclic10; capture receive without status mutex')) {
                throw 'Bounded barcode capture image is absent. Nothing copied.'
            }
            $record.ir_barcode_capture = $true
        }
        # The boot banner now formats these same source arrays instead of
        # a duplicated literal. Source hashes are checked before and after build.
        $reference=Get-RobotIrCalibrationReference -Repository $repoPath
        $record.ir_adc_endpoints=@{white=$reference.White;black=$reference.Black}
        $record.ir_calibrated_bench = $true
        $record.ir_paused_timing_fix = $true
        $record.physical_line_following_validated = $false
        $record.physical_barcode_decoding_validated = $false
    } elseif ($payloadText.Contains($irMarker)) {
        throw 'The selected Default build unexpectedly contains the IR raw bench. Nothing copied.'
    }
    if ($verified.PSObject.Properties.Name -contains 'ultrasonic_pin_fix' -and $verified.ultrasonic_pin_fix) {
        if (-not $payloadText.Contains('[S5] wiring: TRIG=GP%u ECHO=GP%u') -or
            $currentHash -eq 'B9922457B5DFAE6101FBE91137CBD5A8C0D003B95B8EBE9BB268D2B3849A43CD') {
            throw 'The sonar pin correction is absent or this is the previous reversed-pin image. Nothing copied.'
        }
        $record.ultrasonic_wiring = 'TRIG=GP1 ECHO=GP0; restored from Buddy 5 app.h'
    }
    if ($currentHash -eq '38BE0E24A4796E5BFC3AC90A52EF293395F199AF4914CEC0123DCDD32C93FE56') {
        throw 'This is the old DISARMED image without the new motor lock. Nothing copied.'
    }
    if ($verified.PSObject.Properties.Name -contains 's5_status_copy_fix' -and $verified.s5_status_copy_fix) {
        if (-not $payloadText.Contains('[S5] status copies: bounded inheritance wait') -or
            $currentHash -eq '15644919870CD0CC8B775F6E24AA6C4A74D371E7FEF8A6DEAE318806401FD00B') {
            throw 'The status-copy correction is absent or this is the previous polling image. Nothing copied.'
        }
        $record.s5_status_copy_fix = $true
    }
    $record.firmware_sha256 = $currentHash
    $record.previous_sha256 = $previousHash
    $record.bytes = $firmware.Length
    $artifactName = if ($FirmwareProfile -eq 'IRBarcodeCapture') { 'CSC02_IR_BARCODE_CAPTURE_LOCKED.uf2' } elseif ($FirmwareProfile -eq 'IRCalibrated') { 'CSC02_IR_CALIBRATED_LOCKED.uf2' } else { 'CSC02_MOTORS_LOCKED.uf2' }
    $artifactPath = Join-Path $outputPath $artifactName
    Copy-Item -LiteralPath $firmwarePath -Destination $artifactPath -Force
    $record.build_verified = $true
    $record.artifact = $artifactPath
    if ($BuildOnly) {
        Write-Host 'Build and UF2 checks passed. Nothing copied to USB; the robot is unchanged.'
        return
    }
    # Recheck the same target immediately before the flash write.
    if ((Get-Content -LiteralPath $infoPath -Raw) -ne $boardInfo) { throw 'USB target changed. Nothing copied.' }
    Copy-Item -LiteralPath $artifactPath -Destination (Join-Path $drivePath 'CSC02_MOTORS_LOCKED.uf2')
    $record.flashed = $true
    $record.copy_completed = (Get-Date).ToString('o')
    Write-Host 'Firmware copied. The RP2040 should reboot and its BOOTSEL drive should disappear.'
    Write-Host 'Keep motor power off. Confirm the USB console says MOTOR LOCK ON before further testing.'
    Write-Host 'Copy completion does not verify successful boot or actual motor pin levels.'
} catch {
    $record.error = $_.Exception.Message
    Write-Error $_ -ErrorAction Continue
    throw
} finally {
    $record | ConvertTo-Json -Depth 5 | Set-Content -LiteralPath (Join-Path $outputPath $resultName) -Encoding UTF8
    Stop-Transcript | Out-Null
}
