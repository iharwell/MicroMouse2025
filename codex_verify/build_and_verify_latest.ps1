param(
    [ValidateSet('Build', 'Rebuild')]
    [string]$HostBuildTarget = 'Build',
    [ValidateSet('Incremental', 'ProjectDefault')]
    [string]$HostLtcgMode = 'Incremental',
    [string]$LogFilePath
)

Set-StrictMode -Version Latest
$ErrorActionPreference = 'Stop'

$scriptRoot = Split-Path -Parent $MyInvocation.MyCommand.Path
$repoRoot = Split-Path -Parent $scriptRoot
$runStamp = Get-Date -Format 'yyyyMMdd_HHmmss_fff'
$defaultLogDirectory = Join-Path $scriptRoot 'logs'

if ([string]::IsNullOrWhiteSpace($LogFilePath)) {
    $LogFilePath = Join-Path $defaultLogDirectory ('build_and_verify_latest_' + $runStamp + '.txt')
}
elseif (-not [System.IO.Path]::IsPathRooted($LogFilePath)) {
    $LogFilePath = Join-Path $repoRoot $LogFilePath
}

$LogFilePath = [System.IO.Path]::GetFullPath($LogFilePath)
$logDirectory = Split-Path -Parent $LogFilePath
if (-not [string]::IsNullOrWhiteSpace($logDirectory)) {
    New-Item -ItemType Directory -Path $logDirectory -Force | Out-Null
}

@(
    'Build and verify latest log'
    ('Start time: ' + (Get-Date -Format 'yyyy-MM-dd HH:mm:ss.fff zzz'))
    ('Repository: ' + $repoRoot)
    ''
) | Set-Content -LiteralPath $LogFilePath -Encoding UTF8

function Write-LogLine {
    param(
        [AllowEmptyString()]
        [string]$Message = '',
        [string]$Color
    )

    Add-Content -LiteralPath $LogFilePath -Value $Message -Encoding UTF8
    if ($PSBoundParameters.ContainsKey('Color')) {
        Write-Host $Message -ForegroundColor $Color
    }
    else {
        Write-Host $Message
    }
}

function Write-LogFileContents {
    param(
        [Parameter(Mandatory = $true)]
        [string]$Path,
        [string]$Color
    )

    if (-not (Test-Path -LiteralPath $Path)) {
        return
    }

    Get-Content -LiteralPath $Path | ForEach-Object {
        if ($PSBoundParameters.ContainsKey('Color')) {
            Write-LogLine -Message $_ -Color $Color
        }
        else {
            Write-LogLine -Message $_
        }
    }
}

function Append-FileToLog {
    param(
        [Parameter(Mandatory = $true)]
        [string]$Path
    )

    if (-not (Test-Path -LiteralPath $Path)) {
        return
    }

    $content = [System.IO.File]::ReadAllText($Path)
    if ([string]::IsNullOrEmpty($content)) {
        return
    }

    [System.IO.File]::AppendAllText($LogFilePath, $content)
    if (-not $content.EndsWith([Environment]::NewLine, [System.StringComparison]::Ordinal)) {
        [System.IO.File]::AppendAllText($LogFilePath, [Environment]::NewLine)
    }
}

Write-LogLine ("Build-and-verify log: {0}" -f $LogFilePath) 'DarkCyan'

function Write-Step {
    param(
        [Parameter(Mandatory = $true)]
        [string]$Message
    )

    Write-LogLine ''
    Write-LogLine "==> $Message" 'Cyan'
}

function Assert-PathExists {
    param(
        [Parameter(Mandatory = $true)]
        [string]$Path,
        [Parameter(Mandatory = $true)]
        [string]$Description
    )

    if (-not (Test-Path -LiteralPath $Path)) {
        throw "$Description not found: $Path"
    }
}

function Test-IsCodexShell {
    return ($env:CODEX_SHELL -eq '1') -or (-not [string]::IsNullOrWhiteSpace($env:CODEX_INTERNAL_ORIGINATOR_OVERRIDE))
}

function Assert-SupportedBuildAndVerifyLauncher {
    if (-not (Test-IsCodexShell)) {
        return
    }

    if ($env:MM_BUILD_AND_VERIFY_LAUNCHED_FROM_CMD -eq '1') {
        return
    }

    $wrapperPath = Join-Path $scriptRoot 'build_and_verify_latest.cmd'
    throw ('ELEVATION_REQUIRED: Inside Codex, launch "' +
        $wrapperPath +
        '" and request elevation for that command. Do not invoke build_and_verify_latest.ps1 directly from Codex or bypass it with direct msbuild, arduino-cli, or vstest commands.')
}

function Assert-UnsandboxedVerifyEnvironment {
    $microsoftSdksRoot = Join-Path $env:LOCALAPPDATA 'Microsoft SDKs'
    Assert-PathExists -Path $microsoftSdksRoot -Description 'Microsoft SDKs directory'

    try {
        Get-ChildItem -LiteralPath $microsoftSdksRoot -Force -ErrorAction Stop | Select-Object -First 1 | Out-Null
    }
    catch {
        $isAccessDenied = ($_.Exception -is [System.UnauthorizedAccessException]) -or
            ($_.Exception.Message -match 'Access to the path .* is denied')
        if ($isAccessDenied) {
            throw ('ELEVATION_REQUIRED: build_and_verify_latest.ps1 must be run elevated outside the Codex sandbox because the sandbox blocks access to "' +
                $microsoftSdksRoot +
                '", which forces a full host rebuild. Re-run this build-and-verify flow through build_and_verify_latest.cmd with elevated permissions; do not bypass it with direct msbuild, arduino-cli, or vstest commands.')
        }

        throw
    }
}

function Invoke-External {
    param(
        [Parameter(Mandatory = $true)]
        [string]$FilePath,
        [Parameter(Mandatory = $true)]
        [string[]]$Arguments
    )

    Write-LogLine ($FilePath + ' ' + ($Arguments -join ' '))
    $stdoutPath = [System.IO.Path]::GetTempFileName()
    $stderrPath = [System.IO.Path]::GetTempFileName()

    try {
        $process = Start-Process -FilePath $FilePath -ArgumentList $Arguments -Wait -NoNewWindow -PassThru -RedirectStandardOutput $stdoutPath -RedirectStandardError $stderrPath
        Append-FileToLog -Path $stdoutPath
        Append-FileToLog -Path $stderrPath
        if ($process.ExitCode -ne 0) {
            Write-LogFileContents -Path $stdoutPath
            Write-LogFileContents -Path $stderrPath -Color 'Red'
            throw "Command failed with exit code $($process.ExitCode)."
        }
    }
    finally {
        Remove-Item -LiteralPath $stdoutPath, $stderrPath -Force -ErrorAction SilentlyContinue
    }
}

function Invoke-CmdChain {
    param(
        [Parameter(Mandatory = $true)]
        [string]$CommandLine
    )

    Write-LogLine ('cmd.exe /c ' + $CommandLine)
    $stdoutPath = [System.IO.Path]::GetTempFileName()
    $stderrPath = [System.IO.Path]::GetTempFileName()

    try {
        $process = Start-Process -FilePath 'cmd.exe' -ArgumentList '/c', $CommandLine -Wait -NoNewWindow -PassThru -RedirectStandardOutput $stdoutPath -RedirectStandardError $stderrPath
        Append-FileToLog -Path $stdoutPath
        Append-FileToLog -Path $stderrPath
        if ($process.ExitCode -ne 0) {
            Write-LogFileContents -Path $stdoutPath
            Write-LogFileContents -Path $stderrPath -Color 'Red'
            throw "Command failed with exit code $($process.ExitCode)."
        }
    }
    finally {
        Remove-Item -LiteralPath $stdoutPath, $stderrPath -Force -ErrorAction SilentlyContinue
    }
}

function New-MsBuildArgumentString {
    param(
        [Parameter(Mandatory = $true)]
        [string[]]$Arguments
    )

    return ($Arguments | ForEach-Object {
        if ($_ -match '\s') {
            '"' + $_ + '"'
        }
        else {
            $_
        }
    }) -join ' '
}

function Remove-FileIfPresent {
    param(
        [Parameter(Mandatory = $true)]
        [string]$Path
    )

    if (-not (Test-Path -LiteralPath $Path -PathType Leaf)) {
        return
    }

    Remove-Item -LiteralPath $Path -Force
}

function Ensure-ArduinoEigenLibrary {
    param(
        [Parameter(Mandatory = $true)]
        [string]$LibraryRoot,
        [Parameter(Mandatory = $true)]
        [string]$EigenSourceRoot
    )

    $eigenLibraryRoot = Join-Path $LibraryRoot 'Eigen'
    $srcRoot = Join-Path $eigenLibraryRoot 'src'
    $eigenIncludeRoot = Join-Path $srcRoot 'Eigen'
    $entryHeaderPath = Join-Path $srcRoot 'Eigen.h'
    $libraryPropertiesPath = Join-Path $eigenLibraryRoot 'library.properties'

    New-Item -ItemType Directory -Path $srcRoot -Force | Out-Null

    if (-not (Test-Path -LiteralPath $eigenIncludeRoot)) {
        New-Item -ItemType Junction -Path $eigenIncludeRoot -Target (Join-Path $EigenSourceRoot 'Eigen') | Out-Null
    }

    $unsupportedIncludeRoot = Join-Path $srcRoot 'unsupported'
    if (Test-Path -LiteralPath $unsupportedIncludeRoot) {
        Remove-Item -LiteralPath $unsupportedIncludeRoot -Force -Recurse
    }

    @(
        '#pragma once'
        ''
        '// Marker header for Arduino library discovery. Runtime code includes'
        '// Eigen through MazeMap/EigenCompat.h so this header stays zero-cost.'
    ) | Set-Content -Path $entryHeaderPath -Encoding ASCII

    @(
        'name=Eigen'
        'version=5.0.0'
        'author=Eigen Project'
        'maintainer=Eigen Project'
        'sentence=Repo-local staging wrapper for the shared Eigen checkout.'
        'paragraph=Generated by codex_verify/build_and_verify_latest.ps1 so Arduino can resolve Eigen headers without modifying the installed Teensy core.'
        'category=Data Processing'
        'includes=Eigen.h'
        'architectures=*'
    ) | Set-Content -Path $libraryPropertiesPath -Encoding ASCII

    return $LibraryRoot
}

function Get-TrackedPathsFromReadTLog {
    param(
        [Parameter(Mandatory = $true)]
        [string]$Path
    )

    $trackedPaths = [System.Collections.Generic.HashSet[string]]::new([System.StringComparer]::OrdinalIgnoreCase)
    foreach ($line in Get-Content -LiteralPath $Path) {
        if ([string]::IsNullOrWhiteSpace($line)) {
            continue
        }

        $normalizedLine = $line.Trim()
        if ($normalizedLine.StartsWith('^', [System.StringComparison]::Ordinal)) {
            $normalizedLine = $normalizedLine.Substring(1)
        }

        foreach ($segment in ($normalizedLine -split '\|')) {
            $candidatePath = $segment.Trim()
            if ([string]::IsNullOrWhiteSpace($candidatePath)) {
                continue
            }

            if (-not [System.IO.Path]::IsPathRooted($candidatePath)) {
                continue
            }

            try {
                $fullPath = [System.IO.Path]::GetFullPath($candidatePath)
            }
            catch {
                continue
            }

            [void]$trackedPaths.Add($fullPath)
        }
    }

    return @($trackedPaths)
}

function Test-IsPathUnderRoot {
    param(
        [Parameter(Mandatory = $true)]
        [string]$Path,
        [Parameter(Mandatory = $true)]
        [string]$Root
    )

    $fullPath = [System.IO.Path]::GetFullPath($Path)
    $fullRoot = [System.IO.Path]::GetFullPath($Root)
    if ($fullPath.Equals($fullRoot, [System.StringComparison]::OrdinalIgnoreCase)) {
        return $true
    }

    $rootPrefix = $fullRoot.TrimEnd('\') + '\'
    return $fullPath.StartsWith($rootPrefix, [System.StringComparison]::OrdinalIgnoreCase)
}

function Get-LatestRepoInputWriteTimeFromProjectTLogs {
    param(
        [Parameter(Mandatory = $true)]
        [psobject]$Project,
        [Parameter(Mandatory = $true)]
        [string]$RepoRoot
    )

    $repoLocalPaths = [System.Collections.Generic.HashSet[string]]::new([System.StringComparer]::OrdinalIgnoreCase)
    foreach ($tlogName in @('CL.read.1.tlog', 'link.read.1.tlog')) {
        $tlogPath = Join-Path $Project.TLogDir $tlogName
        Assert-PathExists -Path $tlogPath -Description ("{0} tracking log" -f $Project.Name)

        foreach ($trackedPath in Get-TrackedPathsFromReadTLog -Path $tlogPath) {
            if (Test-IsPathUnderRoot -Path $trackedPath -Root $RepoRoot) {
                [void]$repoLocalPaths.Add($trackedPath)
            }
        }
    }

    [void]$repoLocalPaths.Add([System.IO.Path]::GetFullPath($Project.ProjectFile))

    $missingRepoLocalPaths = [System.Collections.Generic.List[string]]::new()
    $latestWriteTime = [datetime]::MinValue

    foreach ($repoLocalPath in $repoLocalPaths) {
        if (-not (Test-Path -LiteralPath $repoLocalPath -PathType Leaf)) {
            $missingRepoLocalPaths.Add($repoLocalPath)
            continue
        }

        $item = Get-Item -LiteralPath $repoLocalPath
        if ($item.LastWriteTime -gt $latestWriteTime) {
            $latestWriteTime = $item.LastWriteTime
        }
    }

    if ($missingRepoLocalPaths.Count -gt 0) {
        $sampleMissingPaths = $missingRepoLocalPaths | Select-Object -First 5
        $sampleText = $sampleMissingPaths -join ', '
        if ($missingRepoLocalPaths.Count -gt $sampleMissingPaths.Count) {
            $sampleText += (", ... ({0} missing total)" -f $missingRepoLocalPaths.Count)
        }

        throw ("{0}: repo-local tracked inputs referenced by Release tracking logs are missing: {1}" -f $Project.Name, $sampleText)
    }

    if ($latestWriteTime -eq [datetime]::MinValue) {
        throw ("{0}: no repo-local tracked inputs were found in Release tracking logs under {1}" -f $Project.Name, $Project.TLogDir)
    }

    return $latestWriteTime
}

function Assert-ArtifactNotOlderThan {
    param(
        [Parameter(Mandatory = $true)]
        [string]$Path,
        [Parameter(Mandatory = $true)]
        [string]$Description,
        [Parameter(Mandatory = $true)]
        [datetime]$NotOlderThan
    )

    Assert-PathExists -Path $Path -Description $Description

    $item = Get-Item -LiteralPath $Path
    if ($item.LastWriteTime -lt $NotOlderThan) {
        throw ("{0} is stale. LastWriteTime={1}; expected at or after {2}." -f $Description, $item.LastWriteTime, $NotOlderThan)
    }

    return $item
}

function Get-TrackedObjectPathsFromClItemsTLog {
    param(
        [Parameter(Mandatory = $true)]
        [string]$ClItemsPath
    )

    $trackedObjectPaths = [System.Collections.Generic.HashSet[string]]::new([System.StringComparer]::OrdinalIgnoreCase)
    foreach ($line in Get-Content -LiteralPath $ClItemsPath) {
        if ([string]::IsNullOrWhiteSpace($line)) {
            continue
        }

        $segments = $line -split ';'
        if ($segments.Count -lt 2) {
            continue
        }

        $candidatePath = $segments[$segments.Count - 1].Trim()
        if ([string]::IsNullOrWhiteSpace($candidatePath)) {
            continue
        }

        [void]$trackedObjectPaths.Add([System.IO.Path]::GetFullPath($candidatePath))
    }

    return @($trackedObjectPaths)
}

function Assert-HostReleaseIntermediatesIntact {
    param(
        [Parameter(Mandatory = $true)]
        [object[]]$Projects
    )

    $failures = [System.Collections.Generic.List[string]]::new()

    foreach ($project in $Projects) {
        if (-not (Test-Path -LiteralPath $project.IntermediateDir -PathType Container)) {
            $failures.Add(("{0}: intermediate directory missing: {1}" -f $project.Name, $project.IntermediateDir))
            continue
        }

        if (-not (Test-Path -LiteralPath $project.TLogDir -PathType Container)) {
            $failures.Add(("{0}: tlog directory missing: {1}" -f $project.Name, $project.TLogDir))
            continue
        }

        $requiredTLogFiles = @(
            'Cl.items.tlog',
            'CL.command.1.tlog',
            'CL.read.1.tlog',
            'CL.write.1.tlog',
            'link.command.1.tlog',
            'link.read.1.tlog',
            'link.secondary.1.tlog',
            'link.write.1.tlog'
        )

        foreach ($requiredTLogFile in $requiredTLogFiles) {
            $requiredTLogPath = Join-Path $project.TLogDir $requiredTLogFile
            if (-not (Test-Path -LiteralPath $requiredTLogPath -PathType Leaf)) {
                $failures.Add(("{0}: required incremental tracking file missing: {1}" -f $project.Name, $requiredTLogPath))
            }
        }

        $lastBuildState = Get-ChildItem -LiteralPath $project.TLogDir -File -Filter '*.lastbuildstate' | Select-Object -First 1
        if ($null -eq $lastBuildState) {
            $failures.Add(("{0}: no *.lastbuildstate file present in {1}" -f $project.Name, $project.TLogDir))
        }

        $fileListManifest = Get-ChildItem -LiteralPath $project.IntermediateDir -File -Filter '*.vcxproj.FileListAbsolute.txt' | Select-Object -First 1
        if ($null -eq $fileListManifest) {
            $failures.Add(("{0}: vcxproj file list manifest missing in {1}" -f $project.Name, $project.IntermediateDir))
        }

        $clItemsPath = Join-Path $project.TLogDir 'Cl.items.tlog'
        if (-not (Test-Path -LiteralPath $clItemsPath -PathType Leaf)) {
            continue
        }

        $trackedObjectPaths = @(Get-TrackedObjectPathsFromClItemsTLog -ClItemsPath $clItemsPath)
        if ($trackedObjectPaths.Count -eq 0) {
            $failures.Add(("{0}: Cl.items.tlog contains no tracked object outputs: {1}" -f $project.Name, $clItemsPath))
            continue
        }

        $missingTrackedObjects = @($trackedObjectPaths | Where-Object {
            -not (Test-Path -LiteralPath $_ -PathType Leaf)
        })

        if ($missingTrackedObjects.Count -gt 0) {
            $sampleMissingObjects = $missingTrackedObjects | Select-Object -First 5
            $sampleText = $sampleMissingObjects -join ', '
            if ($missingTrackedObjects.Count -gt $sampleMissingObjects.Count) {
                $sampleText += (", ... ({0} missing total)" -f $missingTrackedObjects.Count)
            }

            $failures.Add(("{0}: tracked object outputs are missing: {1}" -f $project.Name, $sampleText))
        }
    }

    if ($failures.Count -eq 0) {
        return
    }

    $failureText = ($failures | ForEach-Object { ' - ' + $_ }) -join [Environment]::NewLine
    throw (
        'HOST_INTERMEDIATE_STATE_BROKEN: One or more host-side Release intermediate trees are missing or damaged.' +
        [Environment]::NewLine +
        $failureText +
        [Environment]::NewLine +
        'Agents: host-side intermediates were deleted or corrupted, which breaks the incremental build system.' +
        [Environment]::NewLine +
        'Human intervention is required to repair or intentionally recreate those artifacts.' +
        [Environment]::NewLine +
        'Stop immediately. Do not continue with msbuild, Clean, Rebuild, or further artifact deletion through build_and_verify_latest.ps1.'
    )
}

$arduinoCli = 'C:\Program Files\Arduino IDE\resources\app\lib\backend\resources\arduino-cli.exe'
$visualStudioRoot = 'C:\Program Files\Microsoft Visual Studio\18\Community'
$vsDevCmd = Join-Path $visualStudioRoot 'Common7\Tools\VsDevCmd.bat'
$vstest = Join-Path $visualStudioRoot 'Common7\IDE\CommonExtensions\Microsoft\TestWindow\vstest.console.exe'

$sketchDir = Join-Path $repoRoot 'MazeMap\MazeMap'
$eigenIncludeDir = Join-Path $repoRoot 'MazeMap\eigen-5.0.0'
$arduinoLibrariesDir = Join-Path $scriptRoot 'arduino_libraries'
$arduinoEigenLibraryDir = Join-Path $arduinoLibrariesDir 'Eigen'
$solutionPath = Join-Path $repoRoot 'MazeMap\MazeMap.sln'
$canonicalBuildPath = Join-Path $scriptRoot 'arduino_build'
$firmwareOutputDir = Join-Path $canonicalBuildPath 'firmware'
$buildPath = $firmwareOutputDir
$hexPath = Join-Path $firmwareOutputDir 'MazeMap.ino.hex'
$mazeMapDllPath = Join-Path $repoRoot 'MazeMap\x64\Release\MazeMap.dll'
$testDllPath = Join-Path $repoRoot 'MazeMap\x64\Release\MazeMapTest.dll'
$simulationExePath = Join-Path $repoRoot 'MazeMap\x64\Release\MazeSimulation.exe'
$fqbn = 'teensy:avr:teensy41'
$teensyBoardOptions = @('opt=o2lto')
$teensyOptimizationProfile = 'O2 + LTO'
$runStartedAt = Get-Date
Assert-PathExists -Path $arduinoCli -Description 'Arduino CLI'
Assert-PathExists -Path $vsDevCmd -Description 'Visual Studio developer command script'
Assert-PathExists -Path $vstest -Description 'VSTest console'
Assert-PathExists -Path $sketchDir -Description 'Arduino sketch directory'
Assert-PathExists -Path $eigenIncludeDir -Description 'Shared Eigen include directory'
Assert-PathExists -Path $solutionPath -Description 'MazeMap solution'

$eigenIncludeDir = (Resolve-Path -LiteralPath $eigenIncludeDir).Path
$arduinoLibrariesDir = Ensure-ArduinoEigenLibrary -LibraryRoot $arduinoLibrariesDir -EigenSourceRoot $eigenIncludeDir
$arduinoEigenLibraryDir = (Resolve-Path -LiteralPath $arduinoEigenLibraryDir).Path
$mazeMapHostProject = [pscustomobject]@{
    Name = 'MazeMap'
    IntermediateDir = Join-Path $repoRoot 'MazeMap\MazeMap\x64\Release'
    TLogDir = Join-Path $repoRoot 'MazeMap\MazeMap\x64\Release\MazeMap.tlog'
    ProjectFile = Join-Path $repoRoot 'MazeMap\MazeMap\MazeMap.vcxproj'
}
$mazeMapTestHostProject = [pscustomobject]@{
    Name = 'MazeMapTest'
    IntermediateDir = Join-Path $repoRoot 'MazeMap\MazeMapTest\x64\Release'
    TLogDir = Join-Path $repoRoot 'MazeMap\MazeMapTest\x64\Release\MazeMapTest.tlog'
    ProjectFile = Join-Path $repoRoot 'MazeMap\MazeMapTest\MazeMapTest.vcxproj'
}
$mazeSimulationHostProject = [pscustomobject]@{
    Name = 'MazeSimulation'
    IntermediateDir = Join-Path $repoRoot 'MazeMap\MazeSimulation\x64\Release'
    TLogDir = Join-Path $repoRoot 'MazeMap\MazeSimulation\x64\Release\MazeSimulation.tlog'
    ProjectFile = Join-Path $repoRoot 'MazeMap\MazeSimulation\MazeSimulation.vcxproj'
}
$hostReleaseIntermediateProjects = @(
    $mazeMapHostProject,
    $mazeMapTestHostProject,
    $mazeSimulationHostProject
)

Push-Location $repoRoot
try {
    Write-Step 'Checking build-and-verify launcher'
    Assert-SupportedBuildAndVerifyLauncher
    Write-LogLine 'Build-and-verify launcher check passed.' 'DarkCyan'

    Write-Step 'Checking verify environment'
    Assert-UnsandboxedVerifyEnvironment
    Write-LogLine 'Host build environment access check passed.' 'DarkCyan'

    Write-Step 'Checking host Release intermediates'
    Assert-HostReleaseIntermediatesIntact -Projects $hostReleaseIntermediateProjects
    Write-LogLine 'Host Release intermediate integrity check passed.' 'DarkCyan'

    New-Item -ItemType Directory -Path $canonicalBuildPath -Force | Out-Null
    New-Item -ItemType Directory -Path $buildPath -Force | Out-Null
    New-Item -ItemType Directory -Path $firmwareOutputDir -Force | Out-Null
    Write-LogLine ("Resetting upload artifact path: {0}" -f $hexPath) 'DarkCyan'
    Remove-FileIfPresent -Path $hexPath

    Write-Step 'Compiling the Teensy sketch'
    Write-LogLine ("Teensy compile profile: {0} ({1})" -f $teensyOptimizationProfile, ($teensyBoardOptions -join ', ')) 'DarkCyan'
    $teensyBuildStopwatch = [System.Diagnostics.Stopwatch]::StartNew()
    Invoke-External -FilePath $arduinoCli -Arguments @(
        'compile',
        '--fqbn', $fqbn,
        '--board-options', ($teensyBoardOptions -join ','),
        '--libraries', $arduinoLibrariesDir,
        '--library', $arduinoEigenLibraryDir,
        '--build-path', $buildPath,
        $sketchDir
    )
    $teensyBuildStopwatch.Stop()

    $firmwareImage = Assert-ArtifactNotOlderThan -Path $hexPath -Description 'Compiled firmware image' -NotOlderThan $runStartedAt
    Write-LogLine ("Built {0} ({1}, {2} bytes)" -f $firmwareImage.FullName, $firmwareImage.LastWriteTime, $firmwareImage.Length) 'Green'
    Write-LogLine ("Teensy compile completed in {0:n1}s" -f $teensyBuildStopwatch.Elapsed.TotalSeconds) 'DarkCyan'

    Write-Step 'Building the Release solution'
    Write-LogLine ("Host build target: {0} (Release|x64)" -f $HostBuildTarget) 'DarkCyan'
    Write-LogLine ("Host LTCG mode: {0}" -f $HostLtcgMode) 'DarkCyan'
    $hostMsBuildArguments = @(
        $solutionPath,
        '/m',
        ('/t:' + $HostBuildTarget),
        '/p:Configuration=Release',
        '/p:Platform=x64',
        '/v:m'
    )
    if ($HostLtcgMode -eq 'Incremental') {
        $hostMsBuildArguments += '/p:LinkTimeCodeGeneration=UseFastLinkTimeCodeGeneration'
    }
    $hostBuildStopwatch = [System.Diagnostics.Stopwatch]::StartNew()
    Invoke-CmdChain -CommandLine ('call "{0}" -no_logo && msbuild {1}' -f $vsDevCmd, (New-MsBuildArgumentString -Arguments $hostMsBuildArguments))
    $hostBuildStopwatch.Stop()

    $mazeMapSourceCutoff = Get-LatestRepoInputWriteTimeFromProjectTLogs -Project $mazeMapHostProject -RepoRoot $repoRoot
    $mazeMapTestSourceCutoff = Get-LatestRepoInputWriteTimeFromProjectTLogs -Project $mazeMapTestHostProject -RepoRoot $repoRoot
    $mazeSimulationSourceCutoff = Get-LatestRepoInputWriteTimeFromProjectTLogs -Project $mazeSimulationHostProject -RepoRoot $repoRoot

    $mazeMapDll = Assert-ArtifactNotOlderThan -Path $mazeMapDllPath -Description 'Release MazeMap host binary' -NotOlderThan $mazeMapSourceCutoff
    $simulationExe = Assert-ArtifactNotOlderThan -Path $simulationExePath -Description 'Release MazeSimulation host binary' -NotOlderThan $mazeSimulationSourceCutoff
    $testDll = Assert-ArtifactNotOlderThan -Path $testDllPath -Description 'Release test binary' -NotOlderThan $mazeMapTestSourceCutoff
    Write-LogLine ("Built {0} ({1}, {2} bytes)" -f $mazeMapDll.FullName, $mazeMapDll.LastWriteTime, $mazeMapDll.Length) 'Green'
    Write-LogLine ("Built {0} ({1}, {2} bytes)" -f $testDll.FullName, $testDll.LastWriteTime, $testDll.Length) 'Green'
    Write-LogLine ("Built {0} ({1}, {2} bytes)" -f $simulationExe.FullName, $simulationExe.LastWriteTime, $simulationExe.Length) 'Green'
    Write-LogLine ("Host build completed in {0:n1}s" -f $hostBuildStopwatch.Elapsed.TotalSeconds) 'DarkCyan'

    Write-Step 'Running the Release unit tests'
    $testStopwatch = [System.Diagnostics.Stopwatch]::StartNew()
    Invoke-External -FilePath $vstest -Arguments @($testDll.FullName)
    $testStopwatch.Stop()
    Write-LogLine ("Release tests completed in {0:n1}s" -f $testStopwatch.Elapsed.TotalSeconds) 'DarkCyan'

    Write-Step 'Build and test completed'
    Write-LogLine ("Latest firmware image: {0}" -f $firmwareImage.FullName) 'Green'
    Write-LogLine ("Release test binary: {0}" -f $testDll.FullName) 'Green'
    Write-LogLine ("Build-and-verify log: {0}" -f $LogFilePath) 'Green'
}
catch {
    Write-LogLine ("ERROR: {0}" -f $_.Exception.Message) 'Red'
    throw
}
finally {
    Add-Content -LiteralPath $LogFilePath -Value '' -Encoding UTF8
    Add-Content -LiteralPath $LogFilePath -Value ('End time: ' + (Get-Date -Format 'yyyy-MM-dd HH:mm:ss.fff zzz')) -Encoding UTF8
    Pop-Location
}
