[CmdletBinding()]
param([switch]$Preview,[switch]$Software,[string]$Result='',[int]$Port=0,[switch]$NoBrowser,[switch]$Nx)

$panelArgs = @{} + $PSBoundParameters
if ($Nx) {
    $ErrorActionPreference = 'Stop'
    $panelArgs.Remove('Nx')
    if ($Preview -or $Software -or $Result) { throw 'NX launcher requires live bench mode.' }
    if (!$Port) { $Port = 52989 }
    $panelArgs['Port'] = $Port
    $url = "http://127.0.0.1:$Port/"
    $page = $null
    try {
        $page = Invoke-WebRequest -UseBasicParsing -Uri $url -TimeoutSec 3 -MaximumRedirection 0
    } catch [System.Net.WebException] {
        if ($_.Exception.Status -ne [Net.WebExceptionStatus]::ConnectFailure) { throw }
    }
    if ($page) {
        $match = [regex]::Match($page.Content, 'TOKEN="([A-Za-z0-9_-]+)"')
        if (!$match.Success) { throw "Port $Port is in use by another application." }
        $state = Invoke-RestMethod -Uri ($url + 'state?after=2147483647') -TimeoutSec 3 -MaximumRedirection 0 -Headers @{'X-Panel-Token'=$match.Groups[1].Value}
        if ($state.mode -ne 'bench' -or $state.preview -or $state.software) { throw "Port $Port does not host the live bench panel." }
        if (!$NoBrowser) { Start-Process $url }
        Write-Output "PH-EXO NX: $url ($($state.actual))"
        exit 0
    }
}
& (Join-Path $PSScriptRoot 'Open-Bench-Panel.ps1') -Mode bench @panelArgs
exit $LASTEXITCODE
