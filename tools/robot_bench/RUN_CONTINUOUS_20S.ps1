param([switch]$Run,[ValidatePattern('^COM[0-9]+$')][string]$Port,[string]$RepoRoot,[string]$OutputDirectory)
$ErrorActionPreference='Stop'
Set-StrictMode -Version Latest
. (Join-Path $PSScriptRoot 'bench_common.ps1')
$context=Get-RobotBenchContext -ToolsDirectory $PSScriptRoot -RepoRoot $RepoRoot -OutputDirectory $OutputDirectory
$name='CONTINUOUS_'+(Get-Date -Format 'yyyyMMdd_HHmmss_fff')
$logPath=Join-Path $context.Output ($name+'.log');$resultPath=Join-Path $context.Output ($name+'_RESULT.json')
$stopPath=Join-Path $context.Output 'STOP_CONTINUOUS.flag'
$record=[ordered]@{mode=$(if($Run){'Run'}else{'Inspect'});status='NOT_OPENED';power_percent=30;duration_ms=$(if($Run){20000}else{0});commands_sent=0;log=$logPath;manual_stop=$false;pattern_stop=$false;encoder_stop=$false;physical_line_following_validated=$false}
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
        if($Renew -and (Test-Path -LiteralPath $stopPath)){$device.Write('s');$record.commands_sent++;$record.manual_stop=$true;return}
        [void]$capture.Append($device.ReadExisting())
        if($Renew){
            $current=$capture.ToString();$started=$current.LastIndexOf('[IRLINE START]')
            if($started -ge 0){
                $terminal=@([regex]::Matches($current.Substring($started),'(?m)^\[IRLINE\][^\r\n]*'))
                if($terminal.Count -gt 0 -and $terminal[-1].Value -match 'active=0 prepared=0[^\r\n]*reason=([1-9]|1[0-5]) '){return}
            }
        }
        Start-Sleep -Milliseconds 10
    }
}
function Test-LineReadiness([string]$StatusText){
    $ready=@([regex]::Matches($StatusText,'(?m)^\[IRLINE READY\][^\r\n]*'))
    return $ready.Count -gt 0 -and $ready[-1].Value -match '^\[IRLINE READY\] v=6 active=0 prepared=0 cap=30 lease=250 max_ms=20000 adc=1 motor=1 pattern_stop=0 encoder_stop=0 start_ready=1$'
}
function Get-LineRunEvidence([string]$RunText,[string]$StoppedText,[string]$RunMode){
    $rows=@([regex]::Matches($RunText,'(?m)^\[IRLINE\] [^\r\n]*'))
    $reason=-1
    if($rows.Count -and $rows[-1].Value -match ' reason=(\d+) '){$reason=[int]$Matches[1]}
    $reasonNames=@('IDLE','COMPLETE','USB_LOST','ADC_LOST','MOTOR_FAULT','OBSTACLE','CLOCK','BAD_COMMAND','REQUESTED_STOP','DECODED','BAD_PATTERN','LINE_AMBIGUOUS','NO_PROGRESS','COURSE_TIMEOUT')
    $reasonName=if($reason -ge 0 -and $reason -lt $reasonNames.Count){$reasonNames[$reason]}else{'MISSING'}
    # Require a new acknowledgement of our final STOP, not a pre-START status.
    $stop=$StoppedText -match '(?m)^\[IRLINE\] [^\r\n]*active=0 prepared=0[^\r\n]*reason=8 [^\r\n]*motor_state=0 command=0 motor_power=0'
    $moving=$RunText -match '(?m)^\[IRLINE\] [^\r\n]*active=1[^\r\n]*motor_state=1 command=[156] motor_power=([1-9]|[12][0-9]|30) '
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
    if(-not (Test-LineReadiness $capture.ToString())){throw 'Continuous v6 / fresh IR / motor readiness missing; no START sent.'}

    if($Run){
        if(Test-Path -LiteralPath $stopPath){throw 'STOP file is present; no START sent. Remove it deliberately before another run.'}
        Send-Frame 4;Read-For 250
        if($capture.ToString() -notmatch '\[IRLINE\][^\r\n]*active=0 prepared=1'){throw 'PREPARE refused; no START sent.'}
        $scanSequence=1
        Write-Host 'RUN: 30% PWM, 20 seconds. IR patterns steer but do not stop the test. Manual STOP / USB / hardware faults retained.'
        [Console]::Beep(1200,150)
        Send-Frame 2 30 20000
        Read-For 20700 $true
    }
    [void]$capture.Append($device.ReadExisting());$runText=$capture.ToString();$stopOffset=$capture.Length
    $device.Write('s');$record.commands_sent++;Read-For 500
    $stopText=$capture.ToString($stopOffset,$capture.Length-$stopOffset)
    $outcome=Get-LineRunEvidence $runText $stopText $(if($Run){'Trace'}else{'Inspect'})
    foreach($key in $outcome.Keys){$record[$key]=$outcome[$key]}
    if(-not $record.motor_stop_acknowledged){throw 'Final STOP not acknowledged; turn motor power off.'}
    Write-Host ('Result: '+$record.status+'. STOP acknowledged.')
}catch{$record.status='ERROR';$record.error=$_.Exception.Message;throw}
finally{
    if($null -ne $device){
        if($device.IsOpen){try{$device.Write('s');$record.commands_sent++}catch{};$device.Close()}
        $device.Dispose()
    }
    [IO.File]::WriteAllText($logPath,$capture.ToString(),[Text.Encoding]::UTF8)
    $record|ConvertTo-Json -Depth 6|Set-Content -LiteralPath $resultPath -Encoding UTF8
    Write-Host ('Saved: '+$resultPath)
}
