param([string]$Compiler = 'gcc')

$ErrorActionPreference = 'Stop'
$repoPath = (Resolve-Path (Join-Path $PSScriptRoot '..\..')).Path
$tempRoot = [System.IO.Path]::GetFullPath([System.IO.Path]::GetTempPath())
$testRoot = Join-Path $tempRoot ('s2-host-' + [guid]::NewGuid().ToString('N'))

try {
    New-Item -ItemType Directory -Path (Join-Path $testRoot 'subsystem2') |
        Out-Null
    New-Item -ItemType Directory -Path (Join-Path $testRoot 'tk') | Out-Null
    Copy-Item -LiteralPath (Join-Path $PSScriptRoot 'app.h') -Destination $testRoot
    Copy-Item -LiteralPath (Join-Path $PSScriptRoot 'board_pins.h') `
        -Destination $testRoot
    Copy-Item -LiteralPath (Join-Path $PSScriptRoot 'tk\syslib.h') `
        -Destination (Join-Path $testRoot 'tk')
    Copy-Item -LiteralPath (Join-Path $PSScriptRoot 'subsystem2_host_test.c') `
        -Destination $testRoot
    foreach ($file in @('subsystem2.c', 'subsystem2.h', 'subsystem2_test.c')) {
        Copy-Item -LiteralPath (Join-Path $repoPath $file) `
            -Destination (Join-Path $testRoot 'subsystem2')
    }

    $modes = @(
        @{ Name = 'standalone-no-encoder'; Flags = @('-DAPP_S2_ONLY=1') },
        @{ Name = 'standalone-dual-encoder'; Flags = @(
            '-DAPP_S2_ONLY=1', '-DS2_ENCODER_ENABLED=1') },
        @{ Name = 'standalone-dual-encoder-swapped'; Flags = @(
            '-DAPP_S2_ONLY=1', '-DS2_ENCODER_ENABLED=1',
            '-DBOARD_MOTOR1_ENCODER_A_GPIO=4U',
            '-DBOARD_MOTOR1_ENCODER_B_GPIO=5U',
            '-DBOARD_MOTOR2_ENCODER_A_GPIO=16U',
            '-DBOARD_MOTOR2_ENCODER_B_GPIO=17U') },
        @{ Name = 'standalone-dual-encoder-shared-bank'; Flags = @(
            '-DAPP_S2_ONLY=1', '-DS2_ENCODER_ENABLED=1',
            '-DBOARD_MOTOR1_ENCODER_A_GPIO=2U',
            '-DBOARD_MOTOR1_ENCODER_B_GPIO=3U') },
        @{ Name = 'standalone-dual-encoder-cross-bank'; Flags = @(
            '-DAPP_S2_ONLY=1', '-DS2_ENCODER_ENABLED=1',
            '-DBOARD_MOTOR1_ENCODER_A_GPIO=7U',
            '-DBOARD_MOTOR1_ENCODER_B_GPIO=12U',
            '-DBOARD_MOTOR2_ENCODER_A_GPIO=14U',
            '-DBOARD_MOTOR2_ENCODER_B_GPIO=16U') },
        @{ Name = 'integrated-no-encoder'; Flags = @('-DAPP_S2_ONLY=0') },
        @{ Name = 'integrated-dual-encoder'; Flags = @(
            '-DAPP_S2_ONLY=0', '-DS2_ENCODER_ENABLED=1') },
        @{ Name = 'integrated-custom-timeout'; Flags = @(
            '-DAPP_S2_ONLY=0', '-DS2_ENCODER_ENABLED=1',
            '-DS2_COMMAND_TIMEOUT_MS=750U') }
    )
    foreach ($mode in $modes) {
        $binaryPath = Join-Path $testRoot ($mode.Name + '.exe')
        $compileArgs = @('-std=c11', '-Wall', '-Wextra', '-Werror',
            '-Wpedantic', '-DAPP_TEST=1', '-I', $testRoot)
        $compileArgs += $mode.Flags
        $compileArgs += @((Join-Path $testRoot 'subsystem2_host_test.c'),
            (Join-Path $testRoot 'subsystem2\subsystem2_test.c'),
            '-o', $binaryPath)
        & $Compiler @compileArgs
        if ($LASTEXITCODE -ne 0) {
            throw ('Compilation failed: ' + $mode.Name)
        }
        & $binaryPath
        if ($LASTEXITCODE -ne 0) {
            throw ('Tests failed: ' + $mode.Name)
        }
        Write-Output ('PASS ' + $mode.Name)
    }
} finally {
    $resolvedTestRoot = [System.IO.Path]::GetFullPath($testRoot)
    $tempPrefix = $tempRoot.TrimEnd('\') + '\'
    if (-not $resolvedTestRoot.StartsWith(
        $tempPrefix, [System.StringComparison]::OrdinalIgnoreCase)) {
        throw 'Host test cleanup path is outside the temporary directory.'
    }
    if ((Split-Path $resolvedTestRoot -Leaf) -notmatch '^s2-host-[a-f0-9]{32}$') {
        throw 'Host test cleanup path has an unexpected name.'
    }
    if (Test-Path -LiteralPath $resolvedTestRoot) {
        Remove-Item -LiteralPath $resolvedTestRoot -Recurse -Force
    }
}
