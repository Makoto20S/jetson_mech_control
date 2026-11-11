[CmdletBinding()]
param(
    [ValidateSet('status','sync','build','test')][string]$Action = 'status',
    [ValidateSet('imu','state','servo_imu','all')][string]$Profile = 'imu',
    [string]$Python = ''
)
$ErrorActionPreference = 'Stop'
$repository = Split-Path (Split-Path $PSScriptRoot -Parent) -Parent
if (-not $Python) {
    $venvPython = Join-Path $repository '.local/venv/Scripts/python.exe'
    $Python = if (Test-Path -LiteralPath $venvPython -PathType Leaf) { $venvPython } else { 'python' }
}
$script = Join-Path $PSScriptRoot 'sync_orin.py'
$shared = Join-Path (Split-Path (Split-Path $repository -Parent) -Parent) 'SSH\Invoke-ObservedCommand.ps1'
$version = & $Python -c 'import sys; print(int(sys.version_info >= (3,10)))'
if ($LASTEXITCODE -ne 0 -or $version -ne '1') { throw 'Specify a Python 3.10+ interpreter with -Python.' }
if (Test-Path -LiteralPath $shared -PathType Leaf) {
    $quotedPython = $Python.Replace("'", "''")
    $quotedScript = $script.Replace("'", "''")
    $body = "`$env:PYTHONDONTWRITEBYTECODE='1'`n`$env:PYTHONIOENCODING='utf-8'`n& '$quotedPython' -B '$quotedScript' '$Action' --profile '$Profile'`nexit `$LASTEXITCODE"
    $encoded = [Convert]::ToBase64String([Text.Encoding]::UTF8.GetBytes($body))
    & $shared -Target Windows -WorkingDirectory $repository -ScriptBase64 $encoded
    exit $LASTEXITCODE
}
$env:PYTHONDONTWRITEBYTECODE = '1'
$env:PYTHONIOENCODING = 'utf-8'
& $Python -B $script $Action --profile $Profile
exit $LASTEXITCODE
