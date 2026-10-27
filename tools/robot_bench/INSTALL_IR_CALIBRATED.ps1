param([ValidatePattern('^[D-Zd-z]$')][string]$DriveLetter='D',[switch]$BuildOnly,
    [Parameter(Mandatory=$true)][string]$PicoSdkPath,
    [string]$RepoRoot,
    [string]$OutputDirectory )
$ErrorActionPreference='Stop'
Set-StrictMode -Version Latest

. (Join-Path $PSScriptRoot 'bench_common.ps1')
$benchContext=Get-RobotBenchContext -ToolsDirectory $PSScriptRoot -RepoRoot $RepoRoot -OutputDirectory $OutputDirectory
$repoPath=$benchContext.Repository
$outputPath=$benchContext.Output
Write-Host 'Building the measured IR test first. The robot may stay connected normally for this step.'
& (Join-Path $PSScriptRoot 'FLASH_ROBOT_LOCKED.ps1') -FirmwareProfile IRCalibrated -BuildOnly -RepoRoot $repoPath -OutputDirectory $outputPath -PicoSdkPath $PicoSdkPath
$build=Get-Content -LiteralPath (Join-Path $outputPath 'ROBOT_BUILD_RESULT.json') -Raw | ConvertFrom-Json
if(-not $build.build_verified -or $build.flashed -or $build.firmware_profile -ne 'IRCalibrated') { throw 'Verified IR build is absent. Stop here.' }
if($BuildOnly) { return }
Read-Host ('Build verified. Keep motor power off. Put only the ROBOT board into BOOTSEL; press Enter when RPI-RP2 ('+$DriveLetter+':) is ready') | Out-Null
& (Join-Path $PSScriptRoot 'FLASH_ROBOT_LOCKED.ps1') -FirmwareProfile IRCalibrated -DriveLetter $DriveLetter -RepoRoot $repoPath -OutputDirectory $outputPath -PicoSdkPath $PicoSdkPath -Flash
$flash=Get-Content -LiteralPath (Join-Path $outputPath 'ROBOT_FLASH_RESULT.json') -Raw | ConvertFrom-Json
if(-not $flash.flashed -or $flash.firmware_profile -ne 'IRCalibrated') { throw 'IR flash copy did not complete. Stop here.' }
Read-Host 'Let the robot boot normally on USB, with BOOTSEL released and RPI-RP2 gone. Press Enter when ready for stationary tests' | Out-Null
& (Join-Path $PSScriptRoot 'TEST_IR_CALIBRATED.ps1') -RepoRoot $repoPath -OutputDirectory $outputPath
