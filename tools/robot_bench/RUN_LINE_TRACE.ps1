param([ValidateSet('Inspect','Wheels','Steer','Trace')][string]$Mode='Inspect',
      [ValidateRange(1,20)][int]$Power=8,
      [ValidateRange(500,6000)][int]$DurationMs=1000,
      [switch]$WheelsRaised,[switch]$ClearFloor,[switch]$DirectionVerified,[switch]$LineInputsVerified,[switch]$CentredOnTrack,
      [ValidatePattern('^COM[0-9]+$')][string]$Port,
      [string]$RepoRoot,[string]$OutputDirectory)
$ErrorActionPreference='Stop'
Set-StrictMode -Version Latest
. (Join-Path $PSScriptRoot 'bench_common.ps1')
$context=Get-RobotBenchContext -ToolsDirectory $PSScriptRoot -RepoRoot $RepoRoot -OutputDirectory $OutputDirectory
if($Mode -eq 'Wheels' -and -not $WheelsRaised){throw 'Wheels mode needs the operator-confirmed WheelsRaised switch.'}
if($Mode -eq 'Steer' -and (-not $WheelsRaised -or -not $LineInputsVerified -or -not $DirectionVerified)){throw 'Steer requires raised wheels, verified line inputs and confirmed forward motor direction. Start with both sensors seeing WHITE.'}
if($Mode -eq 'Trace' -and (-not $ClearFloor -or -not $DirectionVerified -or -not $LineInputsVerified -or -not $CentredOnTrack)){throw 'Trace requires verified inputs, confirmed wheel direction, a clear supervised floor course and manually centred robot; use LineInputsVerified, ClearFloor, DirectionVerified and CentredOnTrack. Both WHITE cannot prove the line is present.'}
$name='LINE_TRACE_'+$Mode.ToUpperInvariant()+'_'+(Get-Date -Format 'yyyyMMdd_HHmmss_fff')
$logPath=Join-Path $context.Output ($name+'.log')
$resultPath=Join-Path $context.Output ($name+'_RESULT.json')
$record=[ordered]@{mode=$Mode;status='NOT_OPENED';motor_power_requested=$Power;duration_ms=0;commands_sent=0;log=$logPath;flashed=$false;physical_braking_validated=$false}
$device=$null;$capture=[Text.StringBuilder]::new();$scanSequence=0
function Send-Frame([int]$Operation,[int]$Pwm=0,[int]$Millis=0){
    $frame=[byte[]]@(0xD3,$Operation,$Pwm,($Millis -band 255),($Millis -shr 8),$script:scanSequence)
    if($frame -contains 115){throw 'The requested duration contains the reserved STOP byte; choose another duration.'}
    $device.Write($frame,0,$frame.Length);$record.commands_sent++
}
function Read-For([int]$Milliseconds,[bool]$Renew=$false){
    $timer=[Diagnostics.Stopwatch]::StartNew();$nextRenew=0
    while($timer.ElapsedMilliseconds -lt $Milliseconds){
        if($Renew -and $timer.ElapsedMilliseconds -ge $nextRenew){
            $script:scanSequence=($script:scanSequence+1) -band 63
            Send-Frame 3;$nextRenew=$timer.ElapsedMilliseconds+50
        }
        [void]$capture.Append($device.ReadExisting())
        Start-Sleep -Milliseconds 10
    }
}
function Test-LineReadiness([string]$StatusText){
    $ready=@([regex]::Matches($StatusText,'(?m)^\[IRLINE READY\][^\r\n]*'))
    return $ready.Count -gt 0 -and $ready[-1].Value -match '^\[IRLINE READY\] v=2 active=0 prepared=0 cap=20 lease=250 max_ms=6000 adc=1 motor=1 front=1 white_ready=1$'
}
function Get-LineRunEvidence([string]$RunText,[string]$StoppedText,[string]$RunMode){
    $rows=@([regex]::Matches($RunText,'(?m)^\[IRLINE\] [^\r\n]*'))
    $reason=-1
    if($rows.Count -and $rows[-1].Value -match ' reason=(\d+) '){$reason=[int]$Matches[1]}
    $reasonNames=@('IDLE','COMPLETE','USB_LOST','ADC_LOST','MOTOR_FAULT','OBSTACLE','CLOCK','BAD_COMMAND','REQUESTED_STOP','DECODED','BAD_PATTERN','LINE_AMBIGUOUS')
    $reasonName=if($reason -ge 0 -and $reason -lt $reasonNames.Count){$reasonNames[$reason]}else{'MISSING'}
    # Require a new acknowledgement of our final STOP, not a pre-START status.
    $stop=$StoppedText -match '(?m)^\[IRLINE\] [^\r\n]*active=0 prepared=0[^\r\n]*reason=8 [^\r\n]*motor_state=0 command=0 motor_power=0'
    $moving=$RunText -match '(?m)^\[IRLINE\] [^\r\n]*active=1[^\r\n]*motor_state=1 command=[156] motor_power=([1-9]|1[0-9]|20) '
    $status=if(-not $stop){'STOP_NOT_CONFIRMED'}elseif($RunMode -eq 'Inspect'){'FIRMWARE_READY_STOPPED'}elseif($reason -ne 1){'EARLY_STOP_'+$reasonName}elseif(-not $moving){'RUN_ENDED_NO_MOTOR_ACTIVITY_RECORDED'}elseif($RunMode -eq 'Wheels'){'PULSE_COMPLETE_DIRECTION_NEEDS_OPERATOR_CHECK'}elseif($RunMode -eq 'Steer'){'RAISED_STEERING_RUN_COMPLETE_RESPONSES_NEED_OPERATOR_CHECK'}else{'TIMED_LINE_RUN_COMPLETE_PATH_NEEDS_OPERATOR_CHECK'}
    return [ordered]@{status=$status;motor_stop_acknowledged=[bool]$stop;motor_activity_recorded=[bool]$moving;terminal_reason_code=$reason;terminal_reason=$reasonName}
}
try {
    if(-not $Port){
        $live=@([IO.Ports.SerialPort]::GetPortNames());$root='Registry::HKEY_LOCAL_MACHINE\SYSTEM\CurrentControlSet\Enum\USB'
        $robot=Get-ItemProperty -LiteralPath ($root+'\VID_2E8A&PID_000A\MTK3PICO')
        $candidates=@(Get-ChildItem -LiteralPath ($root+'\VID_2E8A&PID_000A&MI_00')|ForEach-Object{
            $id=Get-ItemProperty -LiteralPath $_.PSPath
            if($id.ContainerID -eq $robot.ContainerID -and $id.Service -eq 'usbser'){
                $p=Get-ItemProperty -LiteralPath (Join-Path $_.PSPath 'Device Parameters')
                if($p.PortName -in $live){[string]$p.PortName}
            }
        }|Sort-Object -Unique)
        if($candidates.Count -ne 1){throw 'Robot normal USB identity not uniquely present. Reconnect normal USB with BOOTSEL released; no motion started.'}
        $Port=$candidates[0]
    }
    $record.port=$Port
    $device=[IO.Ports.SerialPort]::new($Port,115200)
    $device.DtrEnable=$true;$device.WriteTimeout=100;$device.ReadTimeout=100;$device.Open()
    $device.Write('s');$record.commands_sent++;Read-For 1000
    [void]$capture.Clear();Send-Frame 5;Read-For 400
    if($capture.ToString() -notmatch '\[IRLINE READY\] v=2 active=0 prepared=0 cap=20 lease=250 max_ms=6000 adc=1 motor=1'){
        throw 'Line-trace firmware did not acknowledge STOPPED/ADC/motor readiness. No START sent.'
    }
    $record.firmware_handshake=$true
    $record.line_inputs_operator_verified=[bool]$LineInputsVerified
    $record.centre_operator_confirmed=[bool]$CentredOnTrack
    $record.automatic_line_loss_detection=$false
    $record.physical_line_following_validated=$false
    if($Mode -in @('Steer','Trace')){
        for($readyAttempt=0;$readyAttempt -lt 8 -and -not (Test-LineReadiness $capture.ToString());$readyAttempt++){
            Send-Frame 5;Read-For 250
        }
        if(-not (Test-LineReadiness $capture.ToString())){throw 'Fresh forward clearance and stable WHITE are required. No PREPARE or START sent. Keep wheels raised; put a flat target beyond25 cm in front and WHITE under both line sensors.'}
    }
    if($Mode -ne 'Inspect'){
        $scanSequence=0;Send-Frame 4;Read-For 350
        if($capture.ToString() -notmatch '\[IRLINE\][^\r\n]*active=0 prepared=1'){throw 'PREPARE was refused. No START sent.'}
        $scanSequence=1
        $runMs=if($Mode -eq 'Wheels'){200}else{$DurationMs}
        $record.duration_ms=$runMs
        $record.operator_wheels_raised=[bool]$WheelsRaised
        $record.operator_clear_floor=[bool]$ClearFloor
        $record.operator_direction_verified=[bool]$DirectionVerified
        Write-Host ('Starting '+$Mode+' at '+$Power+'% PWM for at most '+$runMs+'ms. USB heartbeat active; s is STOP.')
        Send-Frame $(if($Mode -eq 'Wheels'){1}else{2}) $Power $runMs
        if($Mode -eq 'Steer'){
            Read-For 150 $true
            if($capture.ToString() -notmatch '\[IRLINE START\] mode=2'){throw 'Steering START was not acknowledged; no cue given. Inspect saved refusal reason.'}
            Write-Host 'STEER NOW: slide the selected BLACK track under the requested sensor only. Keep the wheels raised and fingers clear.'
            [Console]::Beep(1200,150)
            Read-For ($runMs+550) $true
        }else{Read-For ($runMs+700) $true}
        # A refused START still goes through the fresh final STOP check and
        # retains the firmware's refusal reason in the result. Never retry START.
        $record.start_acknowledged=[bool]($capture.ToString() -match '\[IRLINE START\]')
    }
    [void]$capture.Append($device.ReadExisting())
    $runText=$capture.ToString();$stopOffset=$capture.Length
    $record.motor_commands_seen=@([regex]::Matches($runText,'(?m)^\[IRLINE\][^\r\n]*active=1[^\r\n]*motor_state=1 command=([156]) motor_power=')|ForEach-Object{[int]$_.Groups[1].Value}|Sort-Object -Unique)
    $record.line_masks_seen=@([regex]::Matches($runText,'(?m)^\[IRLINE\][^\r\n]* mask=([0-3]) adc=1')|ForEach-Object{[int]$_.Groups[1].Value}|Sort-Object -Unique)
    $device.Write('s');$record.commands_sent++;Read-For 500
    $stopText=$capture.ToString($stopOffset,$capture.Length-$stopOffset)
    $outcome=Get-LineRunEvidence $runText $stopText $Mode
    foreach($key in $outcome.Keys){$record[$key]=$outcome[$key]}
    if(-not $record.motor_stop_acknowledged){
        throw 'A fresh acknowledgement of the final motor STOP was not received. Keep motor power disconnected/off and inspect the log.'
    }
    Write-Host ('Result: '+$record.status+'. Final commanded motor STOP acknowledged; physical direction/braking require observation.')
}catch{$record.status='ERROR';$record.error=$_.Exception.Message;throw}
finally{
    if($null -ne $device){
        if($device.IsOpen){try{$device.Write('s');$record.commands_sent++;[void]$capture.Append($device.ReadExisting())}catch{}; $device.Close()}
        $device.Dispose()
    }
    [IO.File]::WriteAllText($logPath,$capture.ToString(),[Text.Encoding]::UTF8)
    $record|ConvertTo-Json -Depth 6|Set-Content -LiteralPath $resultPath -Encoding UTF8
    Write-Host ('Saved: '+$resultPath)
}
