param([string]$RepoRoot='C:\Users\godga\Downloads\ES\inf2004_embedded_systems')
$ErrorActionPreference='Stop'
Set-StrictMode -Version Latest
$tokens=$null;$parseErrors=$null
$ast=[Management.Automation.Language.Parser]::ParseFile((Join-Path $RepoRoot 'tools/robot_bench/RUN_IR_MOTOR_SCAN.ps1'),[ref]$tokens,[ref]$parseErrors)
if($parseErrors.Count){throw ($parseErrors|Out-String)}
$fn=$ast.Find({param($node) $node -is [Management.Automation.Language.FunctionDefinitionAst] -and $node.Name -eq 'Get-IrMotorRunEvidence'},$true)
if(-not $fn){throw 'Evidence function missing'}
Invoke-Expression $fn.Extent.Text
$stopped='[IRSCAN] t=500 active=0 prepared=0 mode=2 reason=8 power=0 raw=200 darkness=0 adc=1 front=1 motor_state=0 command=0 motor_power=0 m1=5 m2=5'
$moving='[IRSCAN] t=200 active=1 prepared=0 mode=2 reason=0 power=15 raw=200 darkness=0 adc=1 front=1 motor_state=1 command=1 motor_power=15 m1=3 m2=4'
$checks=0
function Check([bool]$Condition,[string]$Name){if(-not $Condition){throw $Name};$script:checks++}
$done=$stopped.Replace('reason=8','reason=1')
$bad=$stopped.Replace('reason=8','reason=10')
$run=$moving+"`n"+$done
$r=Get-IrMotorRunEvidence $run '' Scan
Check ($r.status -eq 'STOP_NOT_CONFIRMED') 'pre-run STOP must not satisfy final STOP'
$r=Get-IrMotorRunEvidence $run $done Scan
Check (-not $r.motor_stop_acknowledged) 'fresh status without requested STOP reason must not pass'
$r=Get-IrMotorRunEvidence $run $stopped Scan
Check ($r.status -eq 'MOTION_COMPLETE_NO_VALID_BARCODE' -and $r.motor_activity_recorded) 'timed scan'
$r=Get-IrMotorRunEvidence ($moving+"`n"+$bad+"`n[IRSCAN REJECT] status=3 t=300") $stopped Scan
Check ($r.status -eq 'EARLY_STOP_BAD_PATTERN' -and $r.terminal_reason_code -eq 10 -and $r.rejects.Count -eq 1) 'early NOISE must not be called completed'
$r=Get-IrMotorRunEvidence ($moving+"`n"+$stopped.Replace('reason=8','reason=2')) $stopped Scan
Check ($r.status -eq 'EARLY_STOP_USB_LOST') 'lease expiry reported'
$r=Get-IrMotorRunEvidence $done $stopped Wheels
Check ($r.status -eq 'RUN_ENDED_NO_MOTOR_ACTIVITY_RECORDED') 'START alone does not establish motor activity'
$r=Get-IrMotorRunEvidence $run $stopped Wheels
Check ($r.status -eq 'PULSE_COMPLETE_DIRECTION_NEEDS_OPERATOR_CHECK') 'pulse does not assert physical direction'
$r=Get-IrMotorRunEvidence ($moving+"`n[IRSCAN DECODE] symbol=A reverse=0 t=300`n"+$stopped.Replace('reason=8','reason=9')) $stopped Scan
Check ($r.status -eq 'ONBOARD_BARCODE_REPORTED' -and $r.decodes.Count -eq 1) 'matched decoder reason and report'
$r=Get-IrMotorRunEvidence $stopped $stopped Inspect
Check ($r.status -eq 'FIRMWARE_READY_STOPPED' -and -not $r.motor_activity_recorded) 'inspect remains read-only STOP query'
Write-Output ('PASS '+$checks+' motor-run reporting checks; no USB opened or motion commands sent.')
