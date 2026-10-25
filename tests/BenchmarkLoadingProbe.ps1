$ErrorActionPreference = "Stop"

$benchmarkPath = Join-Path (Split-Path $PSScriptRoot -Parent) "benchmark_loading.ps1"
$tokens = $null
$parseErrors = $null
$ast = [System.Management.Automation.Language.Parser]::ParseFile(
    $benchmarkPath, [ref]$tokens, [ref]$parseErrors)
if ($parseErrors.Count -gt 0) {
    throw "benchmark_loading.ps1 contains PowerShell syntax errors: $($parseErrors[0].Message)"
}

$function = $ast.Find(
    { param($node) $node -is [System.Management.Automation.Language.FunctionDefinitionAst] -and
        $node.Name -eq "Get-MedianMs" },
    $true)
if ($null -eq $function) {
    throw "Get-MedianMs was not found in benchmark_loading.ps1."
}
. ([scriptblock]::Create($function.Extent.Text))

$traceCheck = $ast.Find(
    { param($node) $node -is [System.Management.Automation.Language.FunctionDefinitionAst] -and
        $node.Name -eq "Assert-PerfTraceEnabledBuild" },
    $true)
if ($null -eq $traceCheck) {
    throw "Assert-PerfTraceEnabledBuild was not found in benchmark_loading.ps1."
}
. ([scriptblock]::Create($traceCheck.Extent.Text))

$cases = @(
    @{ Values = [double[]]@(); Expected = $null },
    @{ Values = [double[]]@(4); Expected = 4.0 },
    @{ Values = [double[]]@(1, 2, 3); Expected = 2.0 },
    @{ Values = [double[]]@(1, 2, 3, 4, 5); Expected = 3.0 },
    @{ Values = [double[]]@(1, 2, 3, 4); Expected = 2.5 }
)
foreach ($case in $cases) {
    $actual = Get-MedianMs $case.Values
    if ($null -eq $case.Expected) {
        if ($null -ne $actual) {
            throw "Get-MedianMs should return null for an empty sample."
        }
    } elseif ($actual -ne $case.Expected) {
        throw "Get-MedianMs returned $actual; expected $($case.Expected)."
    }
}

$traceBuild = Join-Path ([System.IO.Path]::GetTempPath()) (
    "openbus-benchmark-trace-probe-" + [guid]::NewGuid().ToString("N"))
New-Item -ItemType Directory -Path $traceBuild | Out-Null
try {
    $cachePath = Join-Path $traceBuild "CMakeCache.txt"
    Set-Content -LiteralPath $cachePath -Value "OPENBUS_ENABLE_PERF_TRACE:BOOL=ON"
    Assert-PerfTraceEnabledBuild $traceBuild

    Set-Content -LiteralPath $cachePath -Value "OPENBUS_ENABLE_PERF_TRACE:BOOL=OFF"
    $disabledRejected = $false
    try {
        Assert-PerfTraceEnabledBuild $traceBuild
    } catch {
        $disabledRejected = $true
    }
    if (-not $disabledRejected) {
        throw "Assert-PerfTraceEnabledBuild accepted a tracing-disabled build."
    }

    Remove-Item -LiteralPath $cachePath
    $missingRejected = $false
    try {
        Assert-PerfTraceEnabledBuild $traceBuild
    } catch {
        $missingRejected = $true
    }
    if (-not $missingRejected) {
        throw "Assert-PerfTraceEnabledBuild accepted a build without a CMake cache."
    }
} finally {
    Remove-Item -LiteralPath $traceBuild -Recurse -Force
}

Write-Output "Loading benchmark probe passed."
