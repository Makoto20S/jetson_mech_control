[CmdletBinding()]
param(
    [ValidateSet('ros','bench')][string]$Mode = 'ros',
    [switch]$Preview,[switch]$Legacy,[switch]$Software,
    [string]$Result = '',[int]$Port = 0,[switch]$NoBrowser
)
$ErrorActionPreference='Stop'
if($Legacy -and $PSBoundParameters.ContainsKey('Mode') -and $Mode -ne 'bench'){throw '-Legacy与-Mode ros冲突'}
if($Legacy){$Mode='bench'}
$repository=Split-Path (Split-Path $PSScriptRoot -Parent) -Parent
$python=Join-Path $repository '.local/venv/Scripts/python.exe'
if (-not (Test-Path -LiteralPath $python)) {$python='python'}
$env:PYTHONDONTWRITEBYTECODE='1'
$env:PYTHONIOENCODING='utf-8'
$arguments=@('-B',(Join-Path $PSScriptRoot 'ph_exo_bench_panel.py'),'--mode',$Mode)
if ($Software) {$arguments+='--software'}
if ($Preview) {$arguments+='--preview'}
if ($Result) {$arguments+=@('--result',$Result)}
if ($Port) {$arguments+=@('--port',$Port)}
if ($NoBrowser) {$arguments+='--no-browser'}
& $python @arguments
exit $LASTEXITCODE
