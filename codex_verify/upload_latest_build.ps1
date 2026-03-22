param(
    [switch]$SkipUpload
)

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

function Get-LatestFirmwareImage {
    param(
        [Parameter(Mandatory = $true)]
        [string]$SearchRoot
    )

    $canonicalImages = @(
        (Join-Path $SearchRoot 'arduino_build\firmware\MazeMap.ino.hex'),
        (Join-Path $SearchRoot 'arduino_build\MazeMap.ino.hex')
    )
    $canonicalCandidates = @()
    foreach ($canonicalImage in $canonicalImages) {
        if (Test-Path -LiteralPath $canonicalImage) {
            $canonicalCandidates += Get-Item -LiteralPath $canonicalImage
        }
    }
    if ($canonicalCandidates.Count -gt 0) {
        return $canonicalCandidates |
            Sort-Object LastWriteTimeUtc -Descending |
            Select-Object -First 1
    }

    $preferredImages = @(Get-ChildItem -Path $SearchRoot -Recurse -File -Filter 'MazeMap.ino.hex' |
        Sort-Object LastWriteTimeUtc -Descending)
    if ($preferredImages.Count -gt 0) {
        return $preferredImages[0]
    }

    $allImages = @(Get-ChildItem -Path $SearchRoot -Recurse -File -Filter '*.hex' |
        Sort-Object LastWriteTimeUtc -Descending)
    if ($allImages.Count -eq 0) {
        throw "No firmware image (*.hex) was found under $SearchRoot."
    }

    return $allImages[0]
}

function Get-TeensyUploadPort {
    param(
        [Parameter(Mandatory = $true)]
        [string]$ArduinoCliPath,
        [Parameter(Mandatory = $true)]
        [string]$Fqbn
    )

    $lines = & $ArduinoCliPath 'board' 'list' '--discovery-timeout' '5s'
    if ($LASTEXITCODE -ne 0) {
        throw "Unable to enumerate connected boards."
    }

    $candidatePorts = @()
    foreach ($line in $lines) {
        if ($line -match '^(?<port>\S+)\s+(?<protocol>\S+)\s+.+?\s+(?<fqbn>\S+)\s+(?<core>\S+)\s*$' -and $Matches.fqbn -eq $Fqbn) {
            $candidatePorts += [pscustomobject]@{
                Port = $Matches.port
                Protocol = $Matches.protocol
                Line = $line.Trim()
            }
        }
    }

    if ($candidatePorts.Count -eq 0) {
        throw "No connected board matched $Fqbn."
    }

    $preferred = @($candidatePorts | Where-Object { $_.Protocol -eq 'teensy' })
    if ($preferred.Count -eq 1) {
        return $preferred[0]
    }

    if ($preferred.Count -gt 1) {
        $details = ($preferred | ForEach-Object { $_.Line }) -join [Environment]::NewLine
        throw "Multiple Teensy upload ports matched ${Fqbn}:`n$details"
    }

    if ($candidatePorts.Count -eq 1) {
        return $candidatePorts[0]
    }

    $allDetails = ($candidatePorts | ForEach-Object { $_.Line }) -join [Environment]::NewLine
    throw "Multiple board ports matched ${Fqbn}:`n$allDetails"
}

$scriptRoot = Split-Path -Parent $MyInvocation.MyCommand.Path
$repoRoot = Split-Path -Parent $scriptRoot

$arduinoCli = 'C:\Program Files\Arduino IDE\resources\app\lib\backend\resources\arduino-cli.exe'
$fqbn = 'teensy:avr:teensy41'

Assert-PathExists -Path $arduinoCli -Description 'Arduino CLI'
Assert-PathExists -Path $scriptRoot -Description 'Upload helper directory'

Push-Location $repoRoot
try {
    Write-Step 'Locating the latest firmware image'
    $firmwareImage = Get-LatestFirmwareImage -SearchRoot $scriptRoot
    Write-Host ("Using {0} ({1}, {2} bytes)" -f $firmwareImage.FullName, $firmwareImage.LastWriteTime, $firmwareImage.Length) -ForegroundColor Green

    if ($SkipUpload) {
        Write-Step 'Skipping upload by request'
        Write-Host "Firmware image is ready at $($firmwareImage.FullName)" -ForegroundColor Green
        return
    }

    Write-Step 'Detecting the Teensy 4.1 upload port'
    $portInfo = Get-TeensyUploadPort -ArduinoCliPath $arduinoCli -Fqbn $fqbn
    Write-Host ("Using port {0} ({1})" -f $portInfo.Port, $portInfo.Protocol) -ForegroundColor Green

    Write-Step 'Uploading the verified firmware image'
    Invoke-External -FilePath $arduinoCli -Arguments @(
        'upload',
        '-p', $portInfo.Port,
        '-l', $portInfo.Protocol,
        '-b', $fqbn,
        '--input-file', $firmwareImage.FullName
    )

    Write-Step 'Upload completed'
    Write-Host ("Uploaded {0} to {1}" -f $firmwareImage.FullName, $portInfo.Port) -ForegroundColor Green
}
finally {
    Pop-Location
}
