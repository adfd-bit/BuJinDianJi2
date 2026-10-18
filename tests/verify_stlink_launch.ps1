param([int]$Port = 61240)

$ErrorActionPreference = 'Stop'
$workspace = Split-Path -Parent $PSScriptRoot
$launch = Get-Content -LiteralPath (Join-Path $workspace '.vscode/launch.json') -Raw | ConvertFrom-Json
$config = @($launch.configurations | Where-Object { $_.type -eq 'cortex-debug' -and $_.servertype -eq 'stlink' })[0]
if ($null -eq $config) { throw 'No Cortex-Debug ST-LINK launch configuration found.' }
if ($config.runToEntryPoint -ne 'main') { throw 'Expected runToEntryPoint=main.' }
if (Get-Process -Name 'ST-LINK_gdbserver' -ErrorAction SilentlyContinue) {
    throw 'An existing ST-LINK debug server is running. Leave its session intact.'
}
if (Get-NetTCPConnection -LocalPort $Port -State Listen -ErrorAction SilentlyContinue) {
    throw "TCP port $Port is already in use."
}

$elf = $config.executable.Replace('${workspaceFolder}', $workspace)
$programmer = Join-Path $config.stm32cubeprogrammer 'STM32_Programmer_CLI.exe'
foreach ($path in @($elf, $config.gdbPath, $config.serverpath, $programmer)) {
    if (-not (Test-Path -LiteralPath $path -PathType Leaf)) { throw "Missing launch dependency: $path" }
}
$artifactDir = Join-Path $PSScriptRoot ('validation/stlink_launch_' + (Get-Date -Format 'yyyyMMdd_HHmmss_fff'))
New-Item -Path $artifactDir -ItemType Directory -Force | Out-Null
$gdbScript = Join-Path $artifactDir 'launch.gdb'
@"
set pagination off
set confirm off
set remotetimeout 10
target extended-remote localhost:$Port
monitor reset
load
monitor reset
tbreak main
continue
printf "ENTRY_MAIN_PC=0x%x\n", `$pc
backtrace 1
delete breakpoints
detach
echo MCU_DETACHED_RUNNING\n
quit
"@ | Set-Content -LiteralPath $gdbScript -Encoding ASCII

$server = $null
$gdb = $null
$passed = $false
$serverOut = Join-Path $artifactDir 'server.stdout.log'
$serverErr = Join-Path $artifactDir 'server.stderr.log'
$gdbOut = Join-Path $artifactDir 'gdb.stdout.log'
$gdbErr = Join-Path $artifactDir 'gdb.stderr.log'
try {
    $serverArgs = @('-p', "$Port", '-l', '1', '-d', '-cp', ('"' + $config.stm32cubeprogrammer + '"'))
    $serverArgs += @($config.serverArgs)
    $server = Start-Process -FilePath $config.serverpath -ArgumentList $serverArgs -WorkingDirectory $workspace -WindowStyle Hidden -PassThru -RedirectStandardOutput $serverOut -RedirectStandardError $serverErr
    $deadline = [DateTime]::UtcNow.AddSeconds(15)
    while (-not (Get-NetTCPConnection -LocalPort $Port -State Listen -ErrorAction SilentlyContinue)) {
        $server.Refresh()
        if ($server.HasExited) { throw 'ST-LINK server exited before listening; inspect its logs.' }
        if ([DateTime]::UtcNow -ge $deadline) { throw 'ST-LINK server startup exceeded 15 seconds.' }
        Start-Sleep -Milliseconds 200
    }
    $gdbArgs = @('--batch', '--nx', ('--command="' + $gdbScript + '"'), ('"' + $elf + '"'))
    $gdb = Start-Process -FilePath $config.gdbPath -ArgumentList $gdbArgs -WorkingDirectory $workspace -WindowStyle Hidden -PassThru -RedirectStandardOutput $gdbOut -RedirectStandardError $gdbErr
    # Keep the native handle alive so asynchronous Start-Process exposes ExitCode.
    $null = $gdb.Handle
    if (-not $gdb.WaitForExit(45000)) { throw 'GDB launch validation exceeded 45 seconds.' }
    $gdb.WaitForExit()
    $output = Get-Content -LiteralPath $gdbOut -Raw
    if ($gdb.ExitCode -ne 0 -or $output -notmatch 'Temporary breakpoint \d+, main\s*\(' -or $output -notmatch 'MCU_DETACHED_RUNNING') {
        throw 'GDB did not complete load, main breakpoint and detach; inspect its logs.'
    }
    $passed = $true
    Write-Output 'ST-LINK launch validation passed: loaded ELF, reset, stopped at main, detached running.'
    Write-Output $output
} finally {
    foreach ($process in @($gdb, $server)) {
        if ($null -eq $process) { continue }
        $process.Refresh()
        if (-not $process.HasExited) {
            Stop-Process -Id $process.Id -Force -ErrorAction SilentlyContinue
            $process.WaitForExit(5000) | Out-Null
        }
    }
    if (-not $passed -and $null -ne $server) {
        # Restore a running target after a failed or interrupted debug session.
        & $programmer -c port=SWD mode=UR -rst *> (Join-Path $artifactDir 'recovery.log')
        if ($LASTEXITCODE -ne 0) { Write-Warning 'Target reset recovery failed; inspect recovery.log.' }
    }
    Write-Output "Validation logs: $artifactDir"
}
