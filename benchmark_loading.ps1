param(
    [int]$Runs = 5,
    [string]$Vehicle = "e400",
    [switch]$EnableTrace
)

$ErrorActionPreference = "Stop"

function Get-MedianMs {
    param([double[]]$Values)
    if (-not $Values -or $Values.Count -eq 0) {
        return $null
    }
    $sorted = $Values | Sort-Object
    $middle = [int]($sorted.Count / 2)
    if ($sorted.Count % 2 -eq 1) {
        return [math]::Round($sorted[$middle], 2)
    }
    return [math]::Round((($sorted[$middle - 1] + $sorted[$middle]) / 2.0), 2)
}

$exePath = Join-Path $PSScriptRoot "build-ode\\Release\\OpenBus.exe"
if (-not (Test-Path $exePath)) {
    throw "Missing executable at $exePath. Build OpenBus first."
}

$results = @()
for ($run = 1; $run -le $Runs; $run++) {
    Write-Host "Run $run/$Runs"

    if (Test-Path (Join-Path $PSScriptRoot "game.log")) {
        Remove-Item (Join-Path $PSScriptRoot "game.log") -Force
    }

    cmd /c "taskkill /F /IM OpenBus.exe >nul 2>&1" | Out-Null

    $env:OPENBUS_VEHICLE = $Vehicle
    $env:OPENBUS_VSYNC = "off"
    $env:OPENBUS_CAPTURE_VIEWS = "1"
    if ($EnableTrace) {
        $env:OPENBUS_TRACE = "1"
        $env:OPENBUS_TRACE_FILE = Join-Path $PSScriptRoot "openbus_trace_run$run.json"
    } else {
        Remove-Item Env:OPENBUS_TRACE -ErrorAction SilentlyContinue
        Remove-Item Env:OPENBUS_TRACE_FILE -ErrorAction SilentlyContinue
    }

    & $exePath

    $logPath = Join-Path $PSScriptRoot "game.log"
    if (-not (Test-Path $logPath)) {
        Write-Warning "Missing game.log for run $run"
        continue
    }
    $lines = Get-Content $logPath

    $objects = $lines | Select-String -Pattern "^\[(\d+)\] Game All objects loaded\."
    $textures = $lines | Select-String -Pattern "^\[(\d+)\] Game All textures loaded\."
    if (-not $objects -or -not $textures) {
        Write-Warning "Load completion markers missing for run $run"
        continue
    }

    $objectMs = [double]$objects[-1].Matches[0].Groups[1].Value
    $textureMs = [double]$textures[-1].Matches[0].Groups[1].Value
    $results += [pscustomobject]@{
        Run = $run
        ObjectsMs = $objectMs
        TexturesMs = $textureMs
    }
}

Remove-Item Env:OPENBUS_CAPTURE_VIEWS -ErrorAction SilentlyContinue
Remove-Item Env:OPENBUS_VSYNC -ErrorAction SilentlyContinue
Remove-Item Env:OPENBUS_VEHICLE -ErrorAction SilentlyContinue
if (-not $EnableTrace) {
    Remove-Item Env:OPENBUS_TRACE -ErrorAction SilentlyContinue
    Remove-Item Env:OPENBUS_TRACE_FILE -ErrorAction SilentlyContinue
}

if ($results.Count -eq 0) {
    throw "No successful benchmark runs."
}

$results | Format-Table -AutoSize

$objectMedian = Get-MedianMs ($results | ForEach-Object { $_.ObjectsMs })
$textureMedian = Get-MedianMs ($results | ForEach-Object { $_.TexturesMs })

Write-Host "Median objects loaded (ms): $objectMedian"
Write-Host "Median textures loaded (ms): $textureMedian"
