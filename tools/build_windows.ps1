param(
    [Parameter(Mandatory=$true)][string]$PicoSdkPath,
    [ValidateSet('Default','Mqtt','PassiveTest','SingleCore','Uart','S2Bench','S4Bench','IRRawAdc','IRRawDigital','IRCalibrated','IRBarcodeCapture','IRMotorScan','IRLineTrace','IRLineOnly','IRContinuous')]
    [string]$Profile='Default',
    [string]$Distribution='Ubuntu',
    [ValidateSet('WSL','Native')][string]$Backend='WSL',
    [string]$ArmToolchainBin='',
    [string]$MsysBin='C:\msys64\usr\bin'
)
$ErrorActionPreference='Stop'
$repoPath=(Resolve-Path (Join-Path $PSScriptRoot '..')).Path
$sdkPath=(Resolve-Path -LiteralPath $PicoSdkPath).Path
function ConvertTo-MountedLinuxPath([string]$WindowsPath) {
    if($WindowsPath -notmatch '^[A-Za-z]:\\') {throw 'Use an absolute Windows drive path'}
    return '/mnt/'+$WindowsPath.Substring(0,1).ToLowerInvariant()+'/'+$WindowsPath.Substring(3).Replace('\','/')
}
if($Backend -eq 'Native') {
    if(!(Test-Path -LiteralPath (Join-Path $ArmToolchainBin 'arm-none-eabi-gcc.exe'))) {
        throw 'Native builds require -ArmToolchainBin pointing to the compiler bin folder.'
    }
    $bash=Join-Path $MsysBin 'bash.exe'
    if(!(Test-Path -LiteralPath $bash)) {throw 'Native builds require MSYS2 bash and make.'}
    $nativeRepo=($repoPath.Substring(0,1).ToLowerInvariant()+$repoPath.Substring(2)).Replace('\','/')
    $nativeSdk=($sdkPath.Substring(0,1).ToLowerInvariant()+$sdkPath.Substring(2)).Replace('\','/')
    $nativeRepo='/'+$nativeRepo; $nativeSdk='/'+$nativeSdk
    $savedPath=$env:PATH
    try {
        $env:PATH=$ArmToolchainBin+';C:\msys64\ucrt64\bin;'+$MsysBin+';'+$savedPath
        & $bash --noprofile --norc ($nativeRepo+'/tools/build_firmware.sh') $nativeRepo $nativeSdk $Profile
        $buildExit=$LASTEXITCODE
    } finally {$env:PATH=$savedPath}
} else {
    $linuxRepo=ConvertTo-MountedLinuxPath $repoPath
    $linuxSdk=ConvertTo-MountedLinuxPath $sdkPath
    & wsl -d $Distribution --exec bash ($linuxRepo+'/tools/build_firmware.sh') $linuxRepo $linuxSdk $Profile
    $buildExit=$LASTEXITCODE
}
if($buildExit -ne 0){throw ('Firmware build failed: '+$buildExit)}
