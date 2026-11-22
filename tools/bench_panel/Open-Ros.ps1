[CmdletBinding()]
param([switch]$Preview,[switch]$Software,[string]$Result='',[int]$Port=0,[switch]$NoBrowser)
& (Join-Path $PSScriptRoot 'Open-Bench-Panel.ps1') -Mode ros @PSBoundParameters
exit $LASTEXITCODE
