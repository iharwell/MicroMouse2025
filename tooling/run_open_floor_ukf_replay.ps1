param(
    [string]$Root,
    [string]$Output = "",
    [string]$RunId = "",
    [string]$SampleCsv = "",
    [string]$Metrics = "",
    [switch]$SkipToolBuild
)

$ErrorActionPreference = "Stop"

$repoRoot = Split-Path -Parent $PSScriptRoot
if ([string]::IsNullOrWhiteSpace($Root)) {
    $Root = Join-Path $repoRoot "TestResults"
}

$projectPath = Join-Path $repoRoot "Tools\OpenFloorUkfReplay\OpenFloorUkfReplay.vcxproj"
$exePath = Join-Path $repoRoot "Tools\OpenFloorUkfReplay\x64\Release\OpenFloorUkfReplay.exe"
$mazeMapBinDir = Join-Path $repoRoot "MazeMap\MazeMap\x64\Release"
$mazeMapDllPath = Join-Path $mazeMapBinDir "MazeMap.dll"
$mazeMapLibPath = Join-Path $mazeMapBinDir "MazeMap.lib"
$mazeMapFreshnessInputs = @(
    (Join-Path $repoRoot "MazeMap\MazeMap\MouseUkfFacade.h"),
    (Join-Path $repoRoot "MazeMap\MazeMap\MouseUkfFacade.cpp"),
    (Join-Path $repoRoot "MazeMap\MazeMap\PlantModel.h"),
    (Join-Path $repoRoot "MazeMap\MazeMap\PlantModel.cpp"),
    (Join-Path $repoRoot "MazeMap\MazeMap\SrUkfCore.h"),
    (Join-Path $repoRoot "MazeMap\MazeMap\SrUkfCore.cpp")
)

$msbuild = (Get-Command msbuild.exe -ErrorAction SilentlyContinue | Select-Object -ExpandProperty Source -First 1)
if (-not $msbuild) {
    $vswhere = Join-Path ${env:ProgramFiles(x86)} "Microsoft Visual Studio\Installer\vswhere.exe"
    if (Test-Path $vswhere) {
        $installPath = & $vswhere -latest -products * -requires Microsoft.Component.MSBuild -property installationPath
        if ($installPath) {
            $candidate = Join-Path $installPath "MSBuild\Current\Bin\MSBuild.exe"
            if (Test-Path $candidate) {
                $msbuild = $candidate
            }
        }
    }
}

if (-not $msbuild) {
    throw "MSBuild.exe not found."
}

if (-not (Test-Path $mazeMapDllPath)) {
    throw "MazeMap.dll was not found at '$mazeMapDllPath'. Build MazeMap separately before running this tool."
}

if (-not (Test-Path $mazeMapLibPath)) {
    throw "MazeMap.lib was not found at '$mazeMapLibPath'. Build MazeMap separately before running this tool."
}

$mazeMapBinaryTimeUtc = @(
    (Get-Item -LiteralPath $mazeMapDllPath).LastWriteTimeUtc,
    (Get-Item -LiteralPath $mazeMapLibPath).LastWriteTimeUtc
) | Sort-Object | Select-Object -First 1

$staleSources = @(
    $mazeMapFreshnessInputs |
        Where-Object {
            (Test-Path -LiteralPath $_) -and
            ((Get-Item -LiteralPath $_).LastWriteTimeUtc -gt $mazeMapBinaryTimeUtc)
        }
)

if ($staleSources.Count -gt 0) {
    throw "MazeMap runtime appears stale relative to UKF sources. Rebuild MazeMap separately before replaying. Newer files: $($staleSources -join ', ')"
}

if (-not $SkipToolBuild) {
    & $msbuild $projectPath /m /p:Configuration=Release /p:Platform=x64 /nologo
    if ($LASTEXITCODE -ne 0) {
        exit $LASTEXITCODE
    }
}

if (Test-Path $mazeMapDllPath) {
    $env:PATH = "$mazeMapBinDir;$env:PATH"
}

$arguments = @("--root", $Root)
if (-not [string]::IsNullOrWhiteSpace($Output)) {
    $arguments += @("--output", $Output)
}
if (-not [string]::IsNullOrWhiteSpace($RunId)) {
    $arguments += @("--run-id", $RunId)
}
if (-not [string]::IsNullOrWhiteSpace($SampleCsv)) {
    $arguments += @("--sample-csv", $SampleCsv)
}
if (-not [string]::IsNullOrWhiteSpace($Metrics)) {
    $arguments += @("--metrics", $Metrics)
}

& $exePath @arguments
exit $LASTEXITCODE
