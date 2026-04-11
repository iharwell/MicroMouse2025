param(
    [ValidateSet('Build', 'Rebuild')]
    [string]$HostBuildTarget = 'Build',
    [string]$LogFilePath
)

Set-StrictMode -Version Latest
$ErrorActionPreference = 'Stop'

$scriptRoot = Split-Path -Parent $MyInvocation.MyCommand.Path
$repoRoot = Split-Path -Parent $scriptRoot
$runStamp = Get-Date -Format 'yyyyMMdd_HHmmss_fff'
$defaultLogDirectory = Join-Path $scriptRoot 'logs'

if ([string]::IsNullOrWhiteSpace($LogFilePath)) {
    $LogFilePath = Join-Path $defaultLogDirectory ('verify_latest_build_' + $runStamp + '.txt')
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
    'Verify latest build log'
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

Write-LogLine ("Verify log: {0}" -f $LogFilePath) 'DarkCyan'

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

function Remove-DirectoryIfPresent {
    param(
        [Parameter(Mandatory = $true)]
        [string]$Path
    )

    if (-not (Test-Path -LiteralPath $Path)) {
        return
    }

    Remove-Item -LiteralPath $Path -Recurse -Force
}

function Remove-StaleArduinoBuildDirectories {
    param(
        [Parameter(Mandatory = $true)]
        [string]$ScriptRoot,
        [Parameter(Mandatory = $true)]
        [string[]]$PreservePaths
    )

    $normalizedPreservePaths = @($PreservePaths | ForEach-Object {
        [System.IO.Path]::GetFullPath($_).TrimEnd('\')
    })

    Get-ChildItem -LiteralPath $ScriptRoot -Directory | Where-Object { $_.Name -like 'arduino_build*' } | ForEach-Object {
        $candidatePath = [System.IO.Path]::GetFullPath($_.FullName).TrimEnd('\')
        if ($normalizedPreservePaths -contains $candidatePath) {
            return
        }

        if (-not $candidatePath.StartsWith([System.IO.Path]::GetFullPath($ScriptRoot).TrimEnd('\'), [System.StringComparison]::OrdinalIgnoreCase)) {
            throw "Refusing to remove Arduino build directory outside the helper root: $candidatePath"
        }

        Remove-Item -LiteralPath $candidatePath -Recurse -Force
    }
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
        'paragraph=Generated by codex_verify/verify_latest_build.ps1 so Arduino can resolve Eigen headers without modifying the installed Teensy core.'
        'category=Data Processing'
        'includes=Eigen.h'
        'architectures=*'
    ) | Set-Content -Path $libraryPropertiesPath -Encoding ASCII

    return $LibraryRoot
}

function Get-LatestWriteTime {
    param(
        [Parameter(Mandatory = $true)]
        [string[]]$Roots
    )

    $trackedExtensions = [System.Collections.Generic.HashSet[string]]::new([System.StringComparer]::OrdinalIgnoreCase)
    foreach ($extension in @('.c', '.cc', '.cpp', '.cxx', '.h', '.hh', '.hpp', '.hxx', '.inl', '.ixx', '.vcxproj', '.props', '.targets')) {
        [void]$trackedExtensions.Add($extension)
    }

    $latestWriteTime = [datetime]::MinValue

    foreach ($root in $Roots) {
        Assert-PathExists -Path $root -Description 'Freshness root'

        $items = if (Test-Path -LiteralPath $root -PathType Leaf) {
            @(Get-Item -LiteralPath $root)
        }
        else {
            Get-ChildItem -LiteralPath $root -Recurse -File | Where-Object {
                $trackedExtensions.Contains($_.Extension)
            }
        }

        foreach ($item in $items) {
            if ($item.LastWriteTime -gt $latestWriteTime) {
                $latestWriteTime = $item.LastWriteTime
            }
        }
    }

    if ($latestWriteTime -eq [datetime]::MinValue) {
        throw ('No tracked inputs were found under: ' + ($Roots -join ', '))
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
$buildPath = Join-Path $scriptRoot ('arduino_build_work_' + $runStamp)
$firmwareOutputDir = Join-Path $canonicalBuildPath 'firmware'
$hexPath = Join-Path $firmwareOutputDir 'MazeMap.ino.hex'
$mazeMapDllPath = Join-Path $repoRoot 'MazeMap\x64\Release\MazeMap.dll'
$testDllPath = Join-Path $repoRoot 'MazeMap\x64\Release\MazeMapTest.dll'
$simulationExePath = Join-Path $repoRoot 'MazeMap\x64\Release\MazeSimulation.exe'
$fqbn = 'teensy:avr:teensy41'
$teensyBoardOptions = @('opt=o2lto')
$teensyOptimizationProfile = 'O2 + LTO'
$runStartedAt = Get-Date
$mazeMapSourceRoot = Join-Path $repoRoot 'MazeMap\MazeMap'
$mazeMapTestSourceRoot = Join-Path $repoRoot 'MazeMap\MazeMapTest'
$mazeSimulationSourceRoot = Join-Path $repoRoot 'MazeMap\MazeSimulation'

Assert-PathExists -Path $arduinoCli -Description 'Arduino CLI'
Assert-PathExists -Path $vsDevCmd -Description 'Visual Studio developer command script'
Assert-PathExists -Path $vstest -Description 'VSTest console'
Assert-PathExists -Path $sketchDir -Description 'Arduino sketch directory'
Assert-PathExists -Path $eigenIncludeDir -Description 'Shared Eigen include directory'
Assert-PathExists -Path $solutionPath -Description 'MazeMap solution'
Assert-PathExists -Path $mazeMapSourceRoot -Description 'MazeMap source root'
Assert-PathExists -Path $mazeMapTestSourceRoot -Description 'MazeMapTest source root'
Assert-PathExists -Path $mazeSimulationSourceRoot -Description 'MazeSimulation source root'

$eigenIncludeDir = (Resolve-Path -LiteralPath $eigenIncludeDir).Path
$arduinoLibrariesDir = Ensure-ArduinoEigenLibrary -LibraryRoot $arduinoLibrariesDir -EigenSourceRoot $eigenIncludeDir
$arduinoEigenLibraryDir = (Resolve-Path -LiteralPath $arduinoEigenLibraryDir).Path
$mazeMapSourceCutoff = Get-LatestWriteTime -Roots @($mazeMapSourceRoot)
$mazeMapTestSourceCutoff = Get-LatestWriteTime -Roots @($mazeMapSourceRoot, $mazeMapTestSourceRoot)
$mazeSimulationSourceCutoff = Get-LatestWriteTime -Roots @($mazeSimulationSourceRoot)

Push-Location $repoRoot
try {
    Remove-StaleArduinoBuildDirectories -ScriptRoot $scriptRoot -PreservePaths @($canonicalBuildPath)
    Remove-DirectoryIfPresent -Path $canonicalBuildPath
    Remove-DirectoryIfPresent -Path $buildPath

    New-Item -ItemType Directory -Path $firmwareOutputDir -Force | Out-Null

    Write-Step 'Compiling the Teensy sketch'
    Write-LogLine ("Teensy compile profile: {0} ({1})" -f $teensyOptimizationProfile, ($teensyBoardOptions -join ', ')) 'DarkCyan'
    $teensyBuildStopwatch = [System.Diagnostics.Stopwatch]::StartNew()
    Invoke-External -FilePath $arduinoCli -Arguments @(
        'compile',
        '--clean',
        '--fqbn', $fqbn,
        '--board-options', ($teensyBoardOptions -join ','),
        '--libraries', $arduinoLibrariesDir,
        '--library', $arduinoEigenLibraryDir,
        '--build-path', $buildPath,
        '--output-dir', $firmwareOutputDir,
        $sketchDir
    )
    $teensyBuildStopwatch.Stop()

    $firmwareImage = Assert-ArtifactNotOlderThan -Path $hexPath -Description 'Compiled firmware image' -NotOlderThan $runStartedAt
    Write-LogLine ("Built {0} ({1}, {2} bytes)" -f $firmwareImage.FullName, $firmwareImage.LastWriteTime, $firmwareImage.Length) 'Green'
    Write-LogLine ("Teensy compile completed in {0:n1}s" -f $teensyBuildStopwatch.Elapsed.TotalSeconds) 'DarkCyan'

    Write-Step 'Building the Release solution'
    Write-LogLine ("Host build target: {0} (Release|x64)" -f $HostBuildTarget) 'DarkCyan'
    $hostBuildStopwatch = [System.Diagnostics.Stopwatch]::StartNew()
    Invoke-CmdChain -CommandLine ('call "{0}" -no_logo && msbuild "{1}" /m /t:{2} /p:Configuration=Release /p:Platform=x64 /v:m' -f $vsDevCmd, $solutionPath, $HostBuildTarget)
    $hostBuildStopwatch.Stop()

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

    Remove-StaleArduinoBuildDirectories -ScriptRoot $scriptRoot -PreservePaths @($canonicalBuildPath)
    Remove-DirectoryIfPresent -Path $buildPath

    Write-Step 'Build and test completed'
    Write-LogLine ("Latest firmware image: {0}" -f $firmwareImage.FullName) 'Green'
    Write-LogLine ("Release test binary: {0}" -f $testDll.FullName) 'Green'
    Write-LogLine ("Verify log: {0}" -f $LogFilePath) 'Green'
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
