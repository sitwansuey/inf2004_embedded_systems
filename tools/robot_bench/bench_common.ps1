Set-StrictMode -Version Latest
function Get-RobotBenchContext {
    param([Parameter(Mandatory=$true)][string]$ToolsDirectory,[string]$RepoRoot,[string]$OutputDirectory)
    if(-not $RepoRoot){$RepoRoot=Join-Path $ToolsDirectory '..\..'}
    $repository=(Resolve-Path -LiteralPath $RepoRoot -ErrorAction Stop).Path
    if(-not (Test-Path -LiteralPath (Join-Path $repository 'app_program\robot_config.h')) -or
       -not (Test-Path -LiteralPath (Join-Path $repository 'tools\build_windows.ps1'))){throw 'RepoRoot is not the robot firmware checkout.'}
    if(-not $OutputDirectory){$OutputDirectory=Join-Path $ToolsDirectory 'artifacts'}
    $output=[System.IO.Path]::GetFullPath($OutputDirectory)
    [void](New-Item -ItemType Directory -Path $output -Force -ErrorAction Stop)
    return [pscustomobject]@{Repository=$repository;Output=$output}
}

# Read the reviewed source endpoints; this does not identify installed firmware.
function Get-RobotIrCalibrationReference {
    param([Parameter(Mandatory=$true)][string]$Repository)
    $header=Get-Content -LiteralPath (Join-Path $Repository 'app_program/robot_ir_calibration.h') -Raw
    $values=@{}
    foreach($surface in @('WHITE','BLACK')) {
        $match=[regex]::Match($header,('(?m)^#define BOARD_IR_ADC_'+$surface+' \{([0-9]+)U?,([0-9]+)U?,([0-9]+)U?\}\r?$'))
        if(-not $match.Success){throw ('Cannot parse reviewed '+$surface+' ADC endpoints.')}
        $values[$surface]=@(1..3|ForEach-Object{[int]$match.Groups[$_].Value})
        if(@($values[$surface]|Where-Object{$_ -lt 0 -or $_ -gt 4095}).Count){throw 'ADC endpoint outside 12-bit range.'}
    }
    if($values.BLACK[2] -le $values.WHITE[2]){throw 'GP28 reference requires positive black-white contrast.'}
    return [pscustomobject]@{White=$values.WHITE;Black=$values.BLACK;Scope='Local source endpoints; installed firmware identity is not verified by this read.'}
}
