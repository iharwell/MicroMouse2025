param(
    [string]$Root,
    [string]$Output = "",
    [string]$RunId = "",
    [string]$Tuning = "",
    [string]$SampleCsv = "",
    [string]$FeedforwardSampleCsv = "",
    [string]$CompetitionArchiveRoot = "",
    [string]$Metrics = "",
    [switch]$KnownStationarySeed,
    [switch]$SkipCompetitionArchive,
    [switch]$SkipToolBuild
)

$ErrorActionPreference = "Stop"

$repoRoot = Split-Path -Parent $PSScriptRoot
if ([string]::IsNullOrWhiteSpace($Root)) {
    $Root = Join-Path $repoRoot "TestResults"
}
if ([string]::IsNullOrWhiteSpace($CompetitionArchiveRoot)) {
    $CompetitionArchiveRoot = Join-Path $repoRoot "TestResults\Competition Testing Data"
}
if ([string]::IsNullOrWhiteSpace($Output)) {
    $Output = Join-Path $Root ("open_floor_ukf_replay_" + (Get-Date -Format "yyyy-MM-dd_HH-mm-ss"))
}

$projectPath = Join-Path $repoRoot "Tools\OpenFloorUkfReplay\OpenFloorUkfReplay.vcxproj"
$exePath = Join-Path $repoRoot "Tools\OpenFloorUkfReplay\x64\Release\OpenFloorUkfReplay.exe"
$competitionAnalyzerPath = Join-Path $repoRoot "tooling\analyze_competition_feedforward.py"
$mazeMapBinDir = Join-Path $repoRoot "MazeMap\MazeMap\x64\Release"
$mazeMapDllPath = Join-Path $mazeMapBinDir "MazeMap.dll"
$mazeMapLibPath = Join-Path $mazeMapBinDir "MazeMap.lib"
$mazeMapFreshnessInputs = @(
    (Join-Path $repoRoot "MazeMap\MazeMap\Estimator.h"),
    (Join-Path $repoRoot "MazeMap\MazeMap\Estimator.cpp"),
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

$git = (Get-Command git.exe -ErrorAction SilentlyContinue | Select-Object -ExpandProperty Source -First 1)
if (-not $git) {
    $git = (Get-Command git -ErrorAction SilentlyContinue | Select-Object -ExpandProperty Source -First 1)
}

if (-not (Test-Path $mazeMapDllPath)) {
    throw "MazeMap.dll was not found at '$mazeMapDllPath'. Build MazeMap separately before running this tool."
}

if (-not (Test-Path $mazeMapLibPath)) {
    throw "MazeMap.lib was not found at '$mazeMapLibPath'. Build MazeMap separately before running this tool."
}

$mazeMapBinaryTimeUtc = (Get-Item -LiteralPath $mazeMapDllPath).LastWriteTimeUtc

$staleSources = @(
    $mazeMapFreshnessInputs |
        Where-Object {
            (Test-Path -LiteralPath $_) -and
            ((Get-Item -LiteralPath $_).LastWriteTimeUtc -gt $mazeMapBinaryTimeUtc)
        }
)

if ($staleSources.Count -gt 0) {
    if ($git) {
        $contentDirtyStaleSources = @()
        foreach ($sourcePath in $staleSources) {
            & $git -C $repoRoot diff --quiet -- $sourcePath
            $workingTreeClean = ($LASTEXITCODE -eq 0)
            & $git -C $repoRoot diff --cached --quiet -- $sourcePath
            $indexClean = ($LASTEXITCODE -eq 0)
            if (-not ($workingTreeClean -and $indexClean)) {
                $contentDirtyStaleSources += $sourcePath
            }
        }
        $staleSources = $contentDirtyStaleSources
    }
}

if ($staleSources.Count -gt 0) {
    throw "MazeMap runtime appears stale relative to UKF sources. Rebuild MazeMap separately before replaying. Newer changed files: $($staleSources -join ', ')"
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
$arguments += @("--output", $Output)
if (-not [string]::IsNullOrWhiteSpace($Tuning)) {
    $arguments += @("--tuning", $Tuning)
}
if (-not [string]::IsNullOrWhiteSpace($RunId)) {
    $arguments += @("--run-id", $RunId)
}
if (-not [string]::IsNullOrWhiteSpace($SampleCsv)) {
    $arguments += @("--sample-csv", $SampleCsv)
}
if (-not [string]::IsNullOrWhiteSpace($FeedforwardSampleCsv)) {
    $arguments += @("--feedforward-sample-csv", $FeedforwardSampleCsv)
}
if (-not [string]::IsNullOrWhiteSpace($Metrics)) {
    $arguments += @("--metrics", $Metrics)
}
if ($KnownStationarySeed) {
    $arguments += "--known-stationary-seed"
}

& $exePath @arguments
if ($LASTEXITCODE -ne 0) {
    exit $LASTEXITCODE
}

if (-not $SkipCompetitionArchive) {
    if (-not (Test-Path -LiteralPath $competitionAnalyzerPath)) {
        throw "Competition archive analyzer was not found at '$competitionAnalyzerPath'."
    }
    if (-not (Test-Path -LiteralPath $CompetitionArchiveRoot)) {
        throw "Competition archive root was not found at '$CompetitionArchiveRoot'."
    }

    $python = (Get-Command python.exe -ErrorAction SilentlyContinue | Select-Object -ExpandProperty Source -First 1)
    if (-not $python) {
        $python = (Get-Command python -ErrorAction SilentlyContinue | Select-Object -ExpandProperty Source -First 1)
    }
    if (-not $python) {
        throw "Python was not found, but the competition archive check requires it."
    }

    New-Item -ItemType Directory -Path $Output -Force | Out-Null
    $competitionReportPath = Join-Path $Output "competition_feedforward_report.txt"
    $competitionOutput = & $python $competitionAnalyzerPath --root $CompetitionArchiveRoot --repo-root $repoRoot 2>&1
    $competitionText = ($competitionOutput | Out-String)
    Set-Content -LiteralPath $competitionReportPath -Value $competitionText
    if (-not [string]::IsNullOrWhiteSpace($competitionText)) {
        Write-Host $competitionText.TrimEnd()
    }
    if ($LASTEXITCODE -ne 0) {
        exit $LASTEXITCODE
    }

    Write-Host "Competition archive feedforward report written to $competitionReportPath"
}

exit 0
