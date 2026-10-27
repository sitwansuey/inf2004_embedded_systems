param([string]$RepoRoot)
$ErrorActionPreference='Stop';Set-StrictMode -Version Latest
if(-not $RepoRoot){$RepoRoot=Split-Path (Split-Path $PSScriptRoot -Parent) -Parent}
$tokens=$null;$errors=$null
$ast=[Management.Automation.Language.Parser]::ParseFile((Join-Path $RepoRoot 'tools/robot_bench/RUN_CONTINUOUS_20S.ps1'),[ref]$tokens,[ref]$errors)
if($errors.Count){throw ($errors|Out-String)}
foreach($name in @('Test-LineReadiness','Get-LineRunEvidence','Send-Frame','Read-For')){
 $fn=$ast.Find({param($n) $n -is [Management.Automation.Language.FunctionDefinitionAst] -and $n.Name -eq $name},$true)
 if(-not $fn){throw ('Missing '+$name)};Invoke-Expression $fn.Extent.Text
}
$checks=0
function Check([bool]$ok,[string]$label){if(-not $ok){throw $label};$script:checks++}
$ready='[IRLINE READY] v=6 active=0 prepared=0 cap=30 lease=250 max_ms=20000 adc=1 motor=1 pattern_stop=0 encoder_stop=0 start_ready=1'
Check (Test-LineReadiness $ready) 'v6 ready'
foreach($replacement in @(@('v=6','v=5'),@('cap=30','cap=50'),@('adc=1','adc=0'),@('motor=1','motor=0'),@('pattern_stop=0','pattern_stop=1'),@('encoder_stop=0','encoder_stop=1'),@('start_ready=1','start_ready=0'))){
 Check (-not (Test-LineReadiness $ready.Replace($replacement[0],$replacement[1]))) ('Refuse '+$replacement[0])
}
Check (-not (Test-LineReadiness ($ready+"`n"+$ready.Replace('adc=1','adc=0')))) 'latest invalid readiness overrides older'
$stop='[IRLINE] t=21000 active=0 prepared=0 mode=2 reason=8 power=0 motor_state=0 command=0 motor_power=0 m1=0 m2=0'
$done=$stop.Replace('reason=8','reason=1')
foreach($direction in @(1,5,6)){
 $moving='[IRLINE] t=100 active=1 prepared=0 mode=2 reason=0 power=30 motor_state=1 command='+$direction+' motor_power=30 m1=0 m2=0'
 $r=Get-LineRunEvidence ($moving+"`n"+$done) $stop Trace
 Check ($r.motor_activity_recorded -and $r.terminal_reason -eq 'COMPLETE' -and $r.motor_stop_acknowledged) '30 percent and stopped encoders are observed, not rejected'
}
$moving='[IRLINE] t=100 active=1 prepared=0 mode=2 reason=0 power=30 motor_state=1 command=1 motor_power=31 m1=0 m2=0'
$r=Get-LineRunEvidence ($moving+"`n"+$done) $stop Trace
Check (-not $r.motor_activity_recorded) 'above cap is not valid activity'
$r=Get-LineRunEvidence $done '' Trace;Check (-not $r.motor_stop_acknowledged) 'fresh STOP required'
$record=[ordered]@{commands_sent=0;manual_stop=$false};$capture=[Text.StringBuilder]::new();$scanSequence=0
$stopPath=Join-Path ([IO.Path]::GetTempPath()) ('continuous-host-test-'+[guid]::NewGuid().ToString()+'.flag')
$script:FakeWrites=0;$device=[pscustomobject]@{Text=''}
$device|Add-Member ScriptMethod Write {$script:FakeWrites++}
$device|Add-Member ScriptMethod ReadExisting {return $this.Text}
try{
 [IO.File]::WriteAllText($stopPath,'STOP')
 $timer=[Diagnostics.Stopwatch]::StartNew();Read-For 20700 $true;$timer.Stop()
 Check ($record.manual_stop -and $timer.ElapsedMilliseconds -lt 1000 -and $script:FakeWrites -eq 2) 'emergency file causes immediate s STOP'
}finally{if(Test-Path -LiteralPath $stopPath){Remove-Item -LiteralPath $stopPath}}
Write-Output ('PASS '+$checks+' continuous host checks; no USB opened.')
