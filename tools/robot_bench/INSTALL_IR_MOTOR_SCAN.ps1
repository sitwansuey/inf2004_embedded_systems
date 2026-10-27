param([Parameter(Mandatory=$true)][string]$PicoSdkPath,
      [Parameter(Mandatory=$true)][string]$PythonPath,
      [switch]$Flash,[ValidatePattern('^[D-Zd-z]$')][string]$DriveLetter='D',
      [string]$RepoRoot,[string]$OutputDirectory)
$ErrorActionPreference='Stop'
Set-StrictMode -Version Latest
. (Join-Path $PSScriptRoot 'bench_common.ps1')
$context=Get-RobotBenchContext -ToolsDirectory $PSScriptRoot -RepoRoot $RepoRoot -OutputDirectory $OutputDirectory
$repoPath=$context.Repository;$outputPath=$context.Output
$resultPath=Join-Path $outputPath 'IR_MOTOR_SCAN_BUILD_RESULT.json'
$drivePath=$DriveLetter.ToUpperInvariant()+':\'
$infoPath=[IO.Path]::Combine($drivePath,'INFO_UF2.TXT')
if($Flash){
    $info=[IO.File]::ReadAllText($infoPath)
    if($info -notmatch '(?m)^Board-ID:\s*RPI-RP2\s*$' -or $info -notmatch '(?m)^Model:\s*Raspberry Pi RP2\s*$'){throw 'Unexpected BOOTSEL target; nothing copied.'}
    Write-Host 'Installing on the operator-identified robot only. Motor power must be disconnected. New image boots STOPPED.'
}
if(Test-Path -LiteralPath (Join-Path $repoPath 'app_program/robot_config.local.h')){throw 'Review the unexpected local override before building.'}
$verified=Get-Content -LiteralPath (Join-Path $PSScriptRoot 'source_verification.json') -Raw|ConvertFrom-Json
foreach($entry in $verified.files.PSObject.Properties){
    if((Get-FileHash -LiteralPath (Join-Path $repoPath $entry.Name) -Algorithm SHA256).Hash -ne $entry.Value){throw ('Unreviewed source: '+$entry.Name)}
}
& (Join-Path $repoPath 'tools/build_windows.ps1') -Profile IRMotorScan -PicoSdkPath $PicoSdkPath
if($LASTEXITCODE -ne 0){throw 'Motor-scan ARM build failed; nothing copied.'}
& $PythonPath (Join-Path $PSScriptRoot 'verify_ir_motor_scan.py') --repo $repoPath --output $outputPath
if($LASTEXITCODE -ne 0){throw 'Motor-scan verification failed; nothing copied.'}
if($Flash){
    if(([IO.File]::ReadAllText($infoPath)) -ne $info){throw 'BOOTSEL target changed; nothing copied.'}
    $result=Get-Content -LiteralPath $resultPath -Raw|ConvertFrom-Json
    if((Get-FileHash -LiteralPath $result.artifact -Algorithm SHA256).Hash -ne $result.firmware_sha256){throw 'Artifact changed; nothing copied.'}
    [IO.File]::Copy($result.artifact,[IO.Path]::Combine($drivePath,'CSC02_IR_MOTOR_SCAN_USB.uf2'),$true)
    $result.flashed=$true
    $result|Add-Member -NotePropertyName copy_completed -NotePropertyValue (Get-Date).ToString('o')
    $result|ConvertTo-Json -Depth 6|Set-Content -LiteralPath $resultPath -Encoding UTF8
    Write-Host 'Copy completed. Keep motor power disconnected; verify normal USB with Mode Inspect next. Copy does not prove successful boot.'
}else{Write-Host 'Build-only verification passed. Nothing copied to USB.'}
