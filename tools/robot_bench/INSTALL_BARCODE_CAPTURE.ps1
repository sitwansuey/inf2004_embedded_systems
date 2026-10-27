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
& (Join-Path $PSScriptRoot 'FLASH_ROBOT_LOCKED.ps1') -FirmwareProfile IRBarcodeCapture -BuildOnly -RepoRoot $repoPath -OutputDirectory $outputPath -PicoSdkPath $PicoSdkPath
if($BuildOnly){return}
Read-Host ('Capture build verified. Keep motor power off; put only the ROBOT into BOOTSEL and press Enter when RPI-RP2 ('+$DriveLetter+':) is ready')|Out-Null
& (Join-Path $PSScriptRoot 'FLASH_ROBOT_LOCKED.ps1') -FirmwareProfile IRBarcodeCapture -DriveLetter $DriveLetter -RepoRoot $repoPath -OutputDirectory $outputPath -PicoSdkPath $PicoSdkPath -Flash
Read-Host 'Let the robot boot normally on USB with BOOTSEL released and RPI-RP2 gone. Press Enter when ready for A/Z card captures'|Out-Null
& (Join-Path $PSScriptRoot 'TEST_BARCODE_CARDS.ps1') -RepoRoot $repoPath -OutputDirectory $outputPath
