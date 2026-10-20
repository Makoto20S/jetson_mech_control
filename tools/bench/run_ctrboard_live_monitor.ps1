[CmdletBinding()]
param(
    [string]$JetsonHost = "nvidia@192.168.55.1",
    [string]$IdentityFile = "",
    [string]$RemoteRepository = "/home/nvidia/jetson_mech_control_git",
    [string]$RemoteInstallSetup = "/home/nvidia/jetson_mech_control_arm64_d1308d3/install/setup.bash",
    [string]$DevicePath = "/dev/ttyACM0",
    [ValidateRange(0, 86400)]
    [int]$DurationSeconds = 0
)

$ErrorActionPreference = "Stop"

function ConvertTo-ShellLiteral {
    param([Parameter(Mandatory)][string]$Value)
    if ($Value.Contains("'")) {
        throw "Remote paths must not contain a single quote: $Value"
    }
    return "'$Value'"
}

if (-not (Get-Command ssh -ErrorAction SilentlyContinue)) {
    throw "Windows OpenSSH client was not found."
}

if ([string]::IsNullOrWhiteSpace($IdentityFile)) {
    $candidates = @(
        (Join-Path $HOME ".ssh\codex_jetson_ed25519_v2"),
        (Join-Path $HOME ".ssh\id_ed25519")
    )
    $IdentityFile = $candidates | Where-Object { Test-Path -LiteralPath $_ } |
        Select-Object -First 1
}

$sshArguments = @("-tt", "-o", "ConnectTimeout=8")
if (-not [string]::IsNullOrWhiteSpace($IdentityFile)) {
    if (-not (Test-Path -LiteralPath $IdentityFile)) {
        throw "SSH identity file does not exist: $IdentityFile"
    }
    $sshArguments += @("-i", $IdentityFile)
}

$remoteScript = "$RemoteRepository/tools/bench/run_ctrboard_live_monitor.sh"
$remoteCommand = "MECH_INSTALL_SETUP=" +
    (ConvertTo-ShellLiteral $RemoteInstallSetup) + " bash " +
    (ConvertTo-ShellLiteral $remoteScript) + " " +
    (ConvertTo-ShellLiteral $DevicePath) + " " + $DurationSeconds

& ssh @sshArguments $JetsonHost $remoteCommand
exit $LASTEXITCODE
