param(
    [ValidateRange(1, 100)]
    [int]$Runs = 5,
    [string]$BuildDirectory = "build-ninja",
    [string]$BusConfig = "Vehicles/MAN_DL05/MAN_DL05.bus",
    [string]$Map = "Cotterell",
    [switch]$EnableTrace
)

$ErrorActionPreference = "Stop"

$runningOpenBus = @(Get-Process -Name "OpenBus" -ErrorAction SilentlyContinue)
if ($runningOpenBus.Count -gt 0) {
    $processIds = ($runningOpenBus | ForEach-Object { $_.Id }) -join ", "
    throw "OpenBus is already running (PID(s): $processIds). Close it before benchmarking. No existing process was stopped."
}

function Get-MedianMs {
    param([double[]]$Values)
    $sorted = @($Values | Sort-Object)
    if ($sorted.Count -eq 0) {
        return $null
    }
    $middle = [int][math]::Floor($sorted.Count / 2.0)
    if ($sorted.Count % 2 -eq 1) {
        return [math]::Round($sorted[$middle], 2)
    }
    return [math]::Round((($sorted[$middle - 1] + $sorted[$middle]) / 2.0), 2)
}

function Assert-PerfTraceEnabledBuild {
    param([string]$BuildPath)

    $cachePath = Join-Path $BuildPath "CMakeCache.txt"
    if (-not (Test-Path -LiteralPath $cachePath -PathType Leaf) -or
        -not (Select-String -LiteralPath $cachePath `
            -Pattern '^OPENBUS_ENABLE_PERF_TRACE:BOOL=ON$' -Quiet)) {
        throw "Tracing was requested, but $BuildPath is not configured with OPENBUS_ENABLE_PERF_TRACE=ON. Configure and build the windows-clang-benchmark preset, or pass its build directory."
    }
}

if ([System.IO.Path]::IsPathRooted($BuildDirectory)) {
    $buildPath = $BuildDirectory
} else {
    $buildPath = Join-Path $PSScriptRoot $BuildDirectory
}
$exePath = Join-Path $buildPath "OpenBus.exe"
if (-not (Test-Path -LiteralPath $exePath -PathType Leaf)) {
    throw "Missing executable at $exePath. Build OpenBus first or pass -BuildDirectory."
}
if ($EnableTrace) {
    Assert-PerfTraceEnabledBuild $buildPath
}

$outputRoot = Join-Path $PSScriptRoot "render-benchmark-results"
$runDirectory = Join-Path $outputRoot ("loading-" + (Get-Date -Format "yyyyMMdd-HHmmss-fff"))
New-Item -ItemType Directory -Path $runDirectory -Force | Out-Null

$environmentNames = @(
    "OPENBUS_BENCHMARK",
    "OPENBUS_BUS_CONFIG",
    "OPENBUS_MAP_PATH",
    "OPENBUS_VSYNC",
    "OPENBUS_TRACE",
    "OPENBUS_TRACE_FILE",
    "OPENBUS_LOG_FILE",
    "OPENBUS_AI_BUS_CONFIG",
    "OPENBUS_AI_MODEL_CONFIG",
    "OPENBUS_AI_COUNT",
    "OPENBUS_MAP_AI_COUNT",
    "OPENBUS_MAP_BUS_COUNT",
    "OPENBUS_MAP_HUMAN_COUNT"
)
$originalEnvironment = @{}
foreach ($name in $environmentNames) {
    $originalEnvironment[$name] = [pscustomobject]@{
        Exists = Test-Path "Env:$name"
        Value = [Environment]::GetEnvironmentVariable($name, "Process")
    }
}

$results = @()
try {
    for ($run = 1; $run -le $Runs; $run++) {
        Write-Host "Run $run/$Runs (close the application window after the load completes)"

        $logPath = Join-Path $runDirectory "run-$run.log"
        [Environment]::SetEnvironmentVariable("OPENBUS_BENCHMARK", "1", "Process")
        [Environment]::SetEnvironmentVariable("OPENBUS_BUS_CONFIG", $BusConfig, "Process")
        [Environment]::SetEnvironmentVariable("OPENBUS_MAP_PATH", $Map, "Process")
        [Environment]::SetEnvironmentVariable("OPENBUS_VSYNC", "off", "Process")
        [Environment]::SetEnvironmentVariable("OPENBUS_LOG_FILE", $logPath, "Process")
        [Environment]::SetEnvironmentVariable("OPENBUS_AI_BUS_CONFIG", $null, "Process")
        [Environment]::SetEnvironmentVariable("OPENBUS_AI_MODEL_CONFIG", $null, "Process")
        [Environment]::SetEnvironmentVariable("OPENBUS_AI_COUNT", $null, "Process")
        [Environment]::SetEnvironmentVariable("OPENBUS_MAP_AI_COUNT", "0", "Process")
        [Environment]::SetEnvironmentVariable("OPENBUS_MAP_BUS_COUNT", "0", "Process")
        [Environment]::SetEnvironmentVariable("OPENBUS_MAP_HUMAN_COUNT", "0", "Process")
        [Environment]::SetEnvironmentVariable(
            "OPENBUS_TRACE", $(if ($EnableTrace) { "1" } else { "0" }), "Process")
        if ($EnableTrace) {
            $tracePath = Join-Path $runDirectory "run-$run-trace.json"
            [Environment]::SetEnvironmentVariable("OPENBUS_TRACE_FILE", $tracePath, "Process")
        } else {
            [Environment]::SetEnvironmentVariable("OPENBUS_TRACE_FILE", $null, "Process")
        }

        $process = Start-Process -FilePath $exePath -WorkingDirectory $PSScriptRoot `
            -PassThru -Wait -NoNewWindow
        if ($process.ExitCode -ne 0) {
            throw "OpenBus run $run exited with code $($process.ExitCode). See $logPath."
        }
        if (-not (Test-Path -LiteralPath $logPath -PathType Leaf)) {
            throw "OpenBus did not create its run log: $logPath."
        }
        if ($EnableTrace -and -not (Test-Path -LiteralPath $tracePath -PathType Leaf)) {
            throw "OpenBus did not create the requested performance trace: $tracePath."
        }

        $objects = Select-String -Path $logPath `
            -Pattern '^\[(\d+)\] (?:\[INFO\] )?Game All objects loaded\.'
        if (-not $objects) {
            throw "The object-load completion marker was not written for run $run. See $logPath."
        }

        $results += [pscustomobject]@{
            Run = $run
            ObjectsMs = [double]$objects[-1].Matches[0].Groups[1].Value
        }
    }
}
finally {
    foreach ($name in $environmentNames) {
        $original = $originalEnvironment[$name]
        if ($original.Exists) {
            [Environment]::SetEnvironmentVariable($name, $original.Value, "Process")
        } else {
            [Environment]::SetEnvironmentVariable($name, $null, "Process")
        }
    }
}

$results | Format-Table -AutoSize
$results | Export-Csv -LiteralPath (Join-Path $runDirectory "results.csv") -NoTypeInformation

$objectMedian = Get-MedianMs ($results | ForEach-Object { $_.ObjectsMs })
Write-Host "Median objects loaded (ms): $objectMedian"
Write-Host "Run logs and results: $runDirectory"
