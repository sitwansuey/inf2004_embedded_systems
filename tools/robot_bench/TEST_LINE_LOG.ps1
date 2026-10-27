param([string]$RepoRoot)
$ErrorActionPreference='Stop'
Set-StrictMode -Version Latest
if(-not $RepoRoot){$RepoRoot=Split-Path (Split-Path $PSScriptRoot -Parent) -Parent}
$tokens=$null;$errors=$null
$ast=[Management.Automation.Language.Parser]::ParseFile((Join-Path $RepoRoot 'tools/robot_bench/RUN_LINE_TRACE.ps1'),[ref]$tokens,[ref]$errors)
if($errors.Count){throw ($errors|Out-String)}
$function=$ast.Find({param($node) $node -is [Management.Automation.Language.FunctionDefinitionAst] -and $node.Name -eq 'Get-LineRunEvidence'},$true)
if(-not $function){throw 'Line reporting function missing.'}
Invoke-Expression $function.Extent.Text
$stop='[IRLINE] t=500 active=0 prepared=0 mode=2 reason=8 power=0 raw=200,200 darkness=0,0 mask=0 adc=1 white_ready=0 front=1 motor_state=0 command=0 motor_power=0 m1=5 m2=5'
$moving='[IRLINE] t=200 active=1 prepared=0 mode=2 reason=0 power=8 raw=3000,3000 darkness=950,950 mask=3 adc=1 white_ready=1 front=1 motor_state=1 command=1 motor_power=8 m1=3 m2=4'
$done=$stop.Replace('reason=8','reason=1')
$checks=0
function Check([bool]$Condition,[string]$Name){if(-not $Condition){throw $Name};$script:checks++}
$r=Get-LineRunEvidence ($moving+"`n"+$done) '' Trace
Check ($r.status -eq 'STOP_NOT_CONFIRMED') 'Pre-run STOP cannot satisfy final STOP.'
$r=Get-LineRunEvidence ($moving+"`n"+$done) $done Trace
Check (-not $r.motor_stop_acknowledged) 'Duration completion is not acknowledgement of requested STOP.'
foreach($direction in @(1,5,6)){
    $r=Get-LineRunEvidence ($moving.Replace('command=1',('command='+$direction))+"`n"+$done) $stop Trace
    Check ($r.status -eq 'TIMED_LINE_RUN_COMPLETE_PATH_NEEDS_OPERATOR_CHECK' -and $r.motor_activity_recorded) 'Each permitted wheel command has reported activity, without asserting physical tracking.'
}
$r=Get-LineRunEvidence ($moving+"`n"+$stop.Replace('reason=8','reason=11')) $stop Trace
Check ($r.status -eq 'EARLY_STOP_LINE_AMBIGUOUS') 'Ambiguous both-black observation must not pass as completed.'
$r=Get-LineRunEvidence ($moving+"`n"+$stop.Replace('reason=8','reason=2')) $stop Trace
Check ($r.status -eq 'EARLY_STOP_USB_LOST') 'Lost host lease must be reported.'
$r=Get-LineRunEvidence $done $stop Trace
Check ($r.status -eq 'RUN_ENDED_NO_MOTOR_ACTIVITY_RECORDED') 'A completed duration cannot establish motor output.'
$r=Get-LineRunEvidence ($moving+"`n"+$done) $stop Wheels
Check ($r.status -eq 'PULSE_COMPLETE_DIRECTION_NEEDS_OPERATOR_CHECK') 'Wheel direction requires physical observation.'
$r=Get-LineRunEvidence $stop $stop Inspect
Check ($r.status -eq 'FIRMWARE_READY_STOPPED' -and -not $r.motor_activity_recorded) 'Inspect sends no motion.'
$r=Get-LineRunEvidence ($moving+"`n"+$done) $stop Steer
Check ($r.status -eq 'RAISED_STEERING_RUN_COMPLETE_RESPONSES_NEED_OPERATOR_CHECK') 'Raised-wheel completion cannot establish floor tracking.'
$readyFunction=$ast.Find({param($node) $node -is [Management.Automation.Language.FunctionDefinitionAst] -and $node.Name -eq 'Test-LineReadiness'},$true)
if(-not $readyFunction){throw 'Readiness function missing.'}
Invoke-Expression $readyFunction.Extent.Text
$ready='[IRLINE READY] v=2 active=0 prepared=0 cap=20 lease=250 max_ms=6000 adc=1 motor=1 front=1 white_ready=1'
Check (Test-LineReadiness $ready) 'Fresh healthy v2 readiness accepted.'
Check (-not (Test-LineReadiness ($ready+"`n"+$ready.Replace('front=1','front=0')))) 'Latest invalid sonar must override older clearance.'
Check (-not (Test-LineReadiness $ready.Replace('white_ready=1','white_ready=0'))) 'Missing WHITE blocks start.'
Check (-not (Test-LineReadiness $ready.Replace('v=2','v=1'))) 'Old geometry profile must not start.'
Check (-not (Test-LineReadiness '')) 'Missing readiness must not start.'
Write-Output ('PASS '+$checks+' line reporting checks; no USB opened or robot commands sent.')
