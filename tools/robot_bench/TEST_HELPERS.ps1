param([string]$RepoRoot,[string]$OutputDirectory,[string]$GccPath)
$ErrorActionPreference='Stop'
Set-StrictMode -Version Latest
. (Join-Path $PSScriptRoot 'bench_common.ps1')
$context=Get-RobotBenchContext -ToolsDirectory $PSScriptRoot -RepoRoot $RepoRoot -OutputDirectory $OutputDirectory
$repoPath=$context.Repository
$testOutput=Join-Path $context.Output ('helper-tests-'+[guid]::NewGuid().ToString('N'))
[void](New-Item -ItemType Directory -Path $testOutput)
$checks=[System.Collections.Generic.List[string]]::new()
foreach($file in Get-ChildItem -LiteralPath $PSScriptRoot -Filter '*.ps1') {
    $tokens=$null;$errors=$null
    [void][System.Management.Automation.Language.Parser]::ParseFile($file.FullName,[ref]$tokens,[ref]$errors)
    if($errors.Count){throw ($errors|Out-String)}
}
$checks.Add('PASS all bench PowerShell scripts parse')
$buildArgs=@{RepoRoot=$repoPath;OutputDirectory=$testOutput}
if($GccPath){$buildArgs.GccPath=$GccPath}
& (Join-Path $PSScriptRoot 'BUILD_HOST_REPLAY.ps1') @buildArgs
$fixtureSource=Get-Content -LiteralPath (Join-Path $repoPath 'testing/subsystem3/test_buddy3.c') -Raw
$summaryPath=Join-Path $testOutput 'fixture_summary.json'
@{status='CAPTURED';procedure='BarcodeCapture';commands_sent=0}|ConvertTo-Json|Set-Content -LiteralPath $summaryPath
foreach($symbol in @('A','Z')) {
    $m=[regex]::Match($fixtureSource,('sample_'+$symbol.ToLowerInvariant()+'\[29\] = \{([^}]+)\}'))
    if(-not $m.Success){throw 'Production test barcode fixture not found'}
    $baseUnits=@($m.Groups[1].Value -split ','|ForEach-Object{[int]$_.Trim()})
    foreach($reverse in @($false,$true)) {
        $units=@($baseUnits);if($reverse){[array]::Reverse($units)}
        $lines=[System.Collections.Generic.List[string]]::new()
        $lines.Add('[CSC02] t=0 phase=0 fault=0 armed=0 cmd=0 power=0 flags=0 bytes=50')
        $time=0
        $segments=@(@{raw=213;duration=400})
        for($i=0;$i -lt 29;$i++){$segments+=@(@{raw=$(if($i%2){213}else{3326});duration=$units[$i]*80})}
        $segments+=@(@{raw=213;duration=1600})
        foreach($segment in $segments){for($i=0;$i -lt $segment.duration;$i+=10){$lines.Add('[S3B] t='+$time+' raw=3214,3177,'+$segment.raw+' mask=7 dropped=0');$time+=10}}
        $log=Join-Path $testOutput ($symbol+'_'+$reverse+'.log')
        $lines|Set-Content -LiteralPath $log -Encoding ASCII
        $r=& (Join-Path $PSScriptRoot 'REPLAY_BARCODE_LOG.ps1') -RepoRoot $repoPath -OutputDirectory $testOutput -LogPath $log -SummaryPath $summaryPath -Symbol $symbol
        if($r.status -ne 'PASS_SENSOR_CAPTURE_NATIVE_REPLAY' -or $r.expected_matches -ne 1 -or $r.onboard_decoding_validated -or $r.motion_validated){throw 'Synthetic replay failed'}
        $checks.Add('PASS synthetic '+$symbol+' reverse='+$reverse+' using explicit new source/output paths')
    }
}
$r=& (Join-Path $PSScriptRoot 'REPLAY_BARCODE_LOG.ps1') -RepoRoot $repoPath -OutputDirectory $testOutput -LogPath (Join-Path $testOutput 'A_False.log') -SummaryPath $summaryPath -Symbol Z
if($r.status -ne 'NOT_DECODED' -or $r.wrong_matches -ne 1){throw 'Wrong symbol accepted'}
$checks.Add('PASS wrong symbol rejected')
$manifestPath=Join-Path $testOutput 'BARCODE_REPLAY_BUILD.json'
$manifestText=Get-Content -LiteralPath $manifestPath -Raw
$manifest=$manifestText|ConvertFrom-Json
$manifest.repository='X:\nonexistent-former-checkout'
$manifest|ConvertTo-Json -Depth 6|Set-Content -LiteralPath $manifestPath
$r=& (Join-Path $PSScriptRoot 'REPLAY_BARCODE_LOG.ps1') -RepoRoot $repoPath -OutputDirectory $testOutput -LogPath (Join-Path $testOutput 'Z_False.log') -SummaryPath $summaryPath -Symbol Z
if($r.status -ne 'PASS_SENSOR_CAPTURE_NATIVE_REPLAY'){throw 'Replay still depends on former source path'}
$checks.Add('PASS relocated manifest uses current checkout hashes, never the former absolute directory')
$manifest.exe_sha256='00';$manifest|ConvertTo-Json -Depth 6|Set-Content -LiteralPath $manifestPath
$blocked=$false;try{& (Join-Path $PSScriptRoot 'REPLAY_BARCODE_LOG.ps1') -RepoRoot $repoPath -OutputDirectory $testOutput -LogPath (Join-Path $testOutput 'Z_False.log') -SummaryPath $summaryPath -Symbol Z|Out-Null}catch{$blocked=$true}
if(-not $blocked){throw 'Executable hash mismatch accepted'}
$manifestText|Set-Content -LiteralPath $manifestPath
$checks.Add('PASS executable hash mismatch rejected')
$manifest=$manifestText|ConvertFrom-Json;$manifest.sources.'app_program/robot_ir_calibration.h'='00'
$manifest|ConvertTo-Json -Depth 6|Set-Content -LiteralPath $manifestPath
$blocked=$false;try{& (Join-Path $PSScriptRoot 'REPLAY_BARCODE_LOG.ps1') -RepoRoot $repoPath -OutputDirectory $testOutput -LogPath (Join-Path $testOutput 'Z_False.log') -SummaryPath $summaryPath -Symbol Z|Out-Null}catch{$blocked=$true}
if(-not $blocked){throw 'Stale source hash accepted'}
$manifestText|Set-Content -LiteralPath $manifestPath
$checks.Add('PASS source hash mismatch rejected')
$overrideRoot=Join-Path $testOutput 'override-root'
[void](New-Item -ItemType Directory -Path (Join-Path $overrideRoot 'app_program') -Force)
[void](New-Item -ItemType Directory -Path (Join-Path $overrideRoot 'tools') -Force)
Set-Content -LiteralPath (Join-Path $overrideRoot 'app_program/robot_config.h') -Value '/* test fixture only */'
Set-Content -LiteralPath (Join-Path $overrideRoot 'tools/build_windows.ps1') -Value '# test fixture only'
Set-Content -LiteralPath (Join-Path $overrideRoot 'app_program/robot_config.local.h') -Value '/* override fixture only */'
$blocked=$false;try{& (Join-Path $PSScriptRoot 'REPLAY_BARCODE_LOG.ps1') -RepoRoot $overrideRoot -OutputDirectory $testOutput -LogPath (Join-Path $testOutput 'Z_False.log') -SummaryPath $summaryPath -Symbol Z|Out-Null}catch{$blocked=$_.Exception.Message -like '*local board override*'}
if(-not $blocked){throw 'Local override was not explicitly rejected'}
$checks.Add('PASS an untracked local board override cannot bypass replay source/profile checks')
$real='BARCODE_TRY_Z_20261022_143940_356'
foreach($suffix in @('.log','_RESULT.json')){Copy-Item -LiteralPath (Join-Path $repoPath ('docs/evidence/2026-10-22/'+$real+$suffix)) -Destination $testOutput}
$r=& (Join-Path $PSScriptRoot 'REPLAY_BARCODE_LOG.ps1') -RepoRoot $repoPath -OutputDirectory $testOutput -LogPath (Join-Path $testOutput ($real+'.log')) -SummaryPath (Join-Path $testOutput ($real+'_RESULT.json')) -Symbol Z
if($r.status -ne 'NOT_DECODED' -or $r.rows -ne 1806 -or $r.discontinuities -ne 0){throw 'Archived real failed Z changed result'}
$checks.Add('PASS archived real Z remains NOT_DECODED with1806 continuous scans')

# Replace only the acquisition helper in an isolated folder. No serial API runs.
$mockTools=Join-Path $testOutput 'mock-tools';[void](New-Item -ItemType Directory -Path $mockTools)
foreach($name in @('bench_common.ps1','TEST_BARCODE_PLANE.ps1','TEST_BARCODE_RETRY.ps1','REPLAY_BARCODE_LOG.ps1','source_verification.json')){Copy-Item -LiteralPath (Join-Path $PSScriptRoot $name) -Destination $mockTools}
@'
param($Seconds,$Procedure,$BarcodeHold,$LogName,$RepoRoot,$OutputDirectory)
$cfg=Get-Content -LiteralPath (Join-Path $PSScriptRoot 'config.json') -Raw|ConvertFrom-Json
if($RepoRoot -ne $cfg.repo -or $OutputDirectory -ne $cfg.output){throw 'Parent source/output path was not forwarded'}
if($cfg.mode -eq 'marker'){
    Copy-Item -LiteralPath $cfg.marker -Destination (Join-Path $OutputDirectory ($LogName+'.log'))
    @{status='CAPTURED';procedure='BarcodeCapture';commands_sent=0}|ConvertTo-Json|Set-Content -LiteralPath (Join-Path $OutputDirectory ($LogName+'_RESULT.json'))
    return
}
if($Seconds -ne 8 -or $BarcodeHold -notin @('WHITE','BLACK')){throw 'Stationary hold/capture length not forwarded'}
$value=if($BarcodeHold -eq 'WHITE'){213}elseif($cfg.mode -eq 'weak'){889}else{3326}
$fault=if($cfg.mode -eq 'fault'){1}else{0}
$lines=[System.Collections.Generic.List[string]]::new();$lines.Add('[CSC02] t=0 phase=0 fault='+$fault+' armed=0 cmd=0 power=0 flags=0 bytes=50')
for($i=0;$i -lt 120;$i++){$lines.Add('[S3B] t='+($i*10)+' raw=3214,3177,'+$value+' mask=7 dropped=0')}
$lines|Set-Content -LiteralPath (Join-Path $OutputDirectory ($LogName+'.log')) -Encoding ASCII
@{status='CAPTURED';procedure='BarcodeCapture';commands_sent=0;controller_lines=1;controller_fault_records=$fault;incomplete_controller_records=0;unexpected_motor_status_lines=0;barcode_hold=$BarcodeHold;barcode_capture_unique_scans=120;barcode_capture_rows=120;barcode_capture_valid_rows=120;barcode_capture_incomplete_records=0;barcode_capture_errors=0;barcode_capture_gap_records=0;barcode_capture_dropped_min=0;barcode_capture_dropped_max=0}|ConvertTo-Json|Set-Content -LiteralPath (Join-Path $OutputDirectory ($LogName+'_RESULT.json'))
'@|Set-Content -LiteralPath (Join-Path $mockTools 'TEST_ROBOT_USB.ps1')
function Read-Host {param($Prompt);return ''}
foreach($mode in @('stable','weak','fault')) {
    @{mode=$mode;repo=$repoPath;output=$testOutput}|ConvertTo-Json|Set-Content -LiteralPath (Join-Path $mockTools 'config.json')
    $thrown=$false;try{& (Join-Path $mockTools 'TEST_BARCODE_PLANE.ps1') -RepoRoot $repoPath -OutputDirectory $testOutput}catch{$thrown=$true}
    $r=Get-Content -LiteralPath (Join-Path $testOutput 'BARCODE_PLANE_RESULT.json') -Raw|ConvertFrom-Json
    $expected=switch($mode){stable{'PASS_STATIONARY_GP28_REFLECTANCE'}weak{'NEEDS_GEOMETRY_OR_CALIBRATION_REVIEW'}fault{'ERROR'}}
    if($r.status -ne $expected -or $thrown -ne ($mode -eq 'fault') -or $r.calibration_applied -or $r.barcode_decoding_validated){throw ('Mock plane test failed: '+$mode)}
    $checks.Add('PASS mock plane '+$mode+' forwards paths and preserves scope/fault exit')
}
@{mode='marker';repo=$repoPath;output=$testOutput;marker=(Join-Path $testOutput 'Z_False.log')}|ConvertTo-Json|Set-Content -LiteralPath (Join-Path $mockTools 'config.json')
& (Join-Path $mockTools 'TEST_BARCODE_RETRY.ps1') -RepoRoot $repoPath -OutputDirectory $testOutput -Symbol Z -Attempts 1
$r=Get-Content -LiteralPath (Join-Path $testOutput 'BARCODE_RETRY_RESULT.json') -Raw|ConvertFrom-Json
if($r.status -ne 'PASS_SENSOR_CAPTURE_NATIVE_REPLAY' -or $r.attempts.Count -ne 1){throw 'Retry did not forward the active checkout and output to replay'}
$checks.Add('PASS mock retry forwards source/output through capture and replay')
$flash=Get-Content -LiteralPath (Join-Path $PSScriptRoot 'FLASH_ROBOT_LOCKED.ps1') -Raw
if($flash -notmatch '\$BuildOnly = -not \$Flash' -or $flash -notmatch 'if \(\$Flash -and \$BuildOnly\)'){throw 'Default build-only flash guard missing'}
$checks.Add('PASS flash copy requires explicit Flash switch; ordinary helper tests never invoke flasher')
$checks.Add('All positive captures above are synthetic/mock. No USB, hardware, firmware flash or remote operation performed.')
$checks|Set-Content -LiteralPath (Join-Path $context.Output 'HELPER_TESTS.log') -Encoding UTF8
$checks|ForEach-Object{Write-Host $_}
