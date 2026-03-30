Set-StrictMode -Version Latest
$ErrorActionPreference = 'Stop'

function Write-Step {
    param(
        [Parameter(Mandatory = $true)]
        [string]$Message
    )

    Write-Host ''
    Write-Host "==> $Message" -ForegroundColor Cyan
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

    Write-Host ($FilePath + ' ' + ($Arguments -join ' '))
    & $FilePath @Arguments
    if ($LASTEXITCODE -ne 0) {
        throw "Command failed with exit code $LASTEXITCODE."
    }
}

function Invoke-CmdChain {
    param(
        [Parameter(Mandatory = $true)]
        [string]$CommandLine
    )

    Write-Host ('cmd.exe /c ' + $CommandLine)
    & cmd.exe /c $CommandLine
    if ($LASTEXITCODE -ne 0) {
        throw "Command failed with exit code $LASTEXITCODE."
    }
}

function Assert-ArtifactFresh {
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
        throw ("{0} was not rebuilt by this run. LastWriteTime={1}; expected at or after {2}." -f $Description, $item.LastWriteTime, $NotOlderThan)
    }

    return $item
}

$scriptRoot = Split-Path -Parent $MyInvocation.MyCommand.Path
$repoRoot = Split-Path -Parent $scriptRoot

$arduinoCli = 'C:\Program Files\Arduino IDE\resources\app\lib\backend\resources\arduino-cli.exe'
$vsDevCmd = 'C:\Program Files\Microsoft Visual Studio\2022\Community\Common7\Tools\VsDevCmd.bat'
$vstest = 'C:\Program Files\Microsoft Visual Studio\2022\Community\Common7\IDE\CommonExtensions\Microsoft\TestWindow\vstest.console.exe'

$sketchDir = Join-Path $repoRoot 'MazeMap\MazeMap'
$sketchLibrariesDir = Join-Path $sketchDir 'libraries'
$solutionPath = Join-Path $repoRoot 'MazeMap\MazeMap.sln'
$buildPath = Join-Path $scriptRoot 'arduino_build'
$firmwareOutputDir = Join-Path $buildPath 'firmware'
$hexPath = Join-Path $firmwareOutputDir 'MazeMap.ino.hex'
$testDllPath = Join-Path $repoRoot 'MazeMap\x64\Release\MazeMapTest.dll'
$fqbn = 'teensy:avr:teensy41'
$runStartedAt = Get-Date

Assert-PathExists -Path $arduinoCli -Description 'Arduino CLI'
Assert-PathExists -Path $vsDevCmd -Description 'Visual Studio developer command script'
Assert-PathExists -Path $vstest -Description 'VSTest console'
Assert-PathExists -Path $sketchDir -Description 'Arduino sketch directory'
Assert-PathExists -Path $sketchLibrariesDir -Description 'Arduino sketch libraries directory'
Assert-PathExists -Path $solutionPath -Description 'MazeMap solution'

Push-Location $repoRoot
try {
    New-Item -ItemType Directory -Path $firmwareOutputDir -Force | Out-Null

    Write-Step 'Compiling the Teensy sketch'
    Invoke-External -FilePath $arduinoCli -Arguments @(
        'compile',
        '--clean',
        '--fqbn', $fqbn,
        '--libraries', $sketchLibrariesDir,
        '--build-path', $buildPath,
        '--output-dir', $firmwareOutputDir,
        $sketchDir
    )

    $firmwareImage = Assert-ArtifactFresh -Path $hexPath -Description 'Compiled firmware image' -NotOlderThan $runStartedAt
    Write-Host ("Built {0} ({1}, {2} bytes)" -f $firmwareImage.FullName, $firmwareImage.LastWriteTime, $firmwareImage.Length) -ForegroundColor Green

    Write-Step 'Rebuilding the Release solution'
    Invoke-CmdChain -CommandLine ('call "{0}" -no_logo && msbuild "{1}" /t:Rebuild /p:Configuration=Release /p:Platform=x64 /v:m' -f $vsDevCmd, $solutionPath)

    $testDll = Assert-ArtifactFresh -Path $testDllPath -Description 'Release test binary' -NotOlderThan $runStartedAt
    Write-Host ("Built {0} ({1}, {2} bytes)" -f $testDll.FullName, $testDll.LastWriteTime, $testDll.Length) -ForegroundColor Green

    Write-Step 'Running the Release unit tests'
    Invoke-External -FilePath $vstest -Arguments @($testDll.FullName)

    Write-Step 'Build and test completed'
    Write-Host ("Latest firmware image: {0}" -f $firmwareImage.FullName) -ForegroundColor Green
    Write-Host ("Release test binary: {0}" -f $testDll.FullName) -ForegroundColor Green
}
finally {
    Pop-Location
}
