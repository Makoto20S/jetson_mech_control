[CmdletBinding()]
param([ValidateSet('ros','bench')][string]$Mode='ros',[switch]$Preview,[switch]$Legacy,
      [switch]$Software,[string]$Result='',[int]$Port=0,[switch]$NoBrowser)
& (Join-Path $PSScriptRoot '../bench_panel/Open-Bench-Panel.ps1') @PSBoundParameters
exit $LASTEXITCODE
