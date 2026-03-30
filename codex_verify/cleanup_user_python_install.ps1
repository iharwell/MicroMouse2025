[CmdletBinding()]
param(
    [switch]$Apply
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

function Write-Preview {
    param(
        [Parameter(Mandatory = $true)]
        [string]$Message
    )

    Write-Host "[dry-run] $Message" -ForegroundColor Yellow
}

function Write-Change {
    param(
        [Parameter(Mandatory = $true)]
        [string]$Message
    )

    Write-Host $Message -ForegroundColor Green
}

function Write-WarningMessage {
    param(
        [Parameter(Mandatory = $true)]
        [string]$Message
    )

    Write-Host $Message -ForegroundColor DarkYellow
}

function Normalize-PathString {
    param(
        [Parameter(Mandatory = $true)]
        [string]$Path
    )

    $expanded = [Environment]::ExpandEnvironmentVariables($Path).Trim()
    return $expanded.TrimEnd('\')
}

function Get-UserPathEntries {
    $rawPath = [Environment]::GetEnvironmentVariable('Path', 'User')
    if ([string]::IsNullOrWhiteSpace($rawPath)) {
        return @()
    }

    $entries = @($rawPath.Split(';', [System.StringSplitOptions]::RemoveEmptyEntries))
    return ,$entries
}

function Set-UserPathEntries {
    param(
        [Parameter(Mandatory = $true)]
        [string[]]$Entries
    )

    $joined = ($Entries | Where-Object { -not [string]::IsNullOrWhiteSpace($_) }) -join ';'
    [Environment]::SetEnvironmentVariable('Path', $joined, 'User')
}

function Remove-UserPathEntries {
    param(
        [Parameter(Mandatory = $true)]
        [string[]]$ExactEntries,
        [Parameter(Mandatory = $true)]
        [string[]]$PrefixEntries,
        [Parameter(Mandatory = $true)]
        [bool]$Commit
    )

    $entries = Get-UserPathEntries
    if ($entries.Count -eq 0) {
        Write-Host 'User PATH is empty.' -ForegroundColor DarkGray
        return
    }

    $exactLookup = @{}
    foreach ($entry in $ExactEntries) {
        $exactLookup[(Normalize-PathString -Path $entry)] = $true
    }

    $prefixList = @()
    foreach ($entry in $PrefixEntries) {
        $prefixList += (Normalize-PathString -Path $entry)
    }

    $keptEntries = New-Object System.Collections.Generic.List[string]
    $removedEntries = New-Object System.Collections.Generic.List[string]

    foreach ($entry in $entries) {
        $normalizedEntry = Normalize-PathString -Path $entry
        $remove = $false

        if ($exactLookup.ContainsKey($normalizedEntry)) {
            $remove = $true
        }
        else {
            foreach ($prefix in $prefixList) {
                if ($normalizedEntry.StartsWith($prefix, [System.StringComparison]::OrdinalIgnoreCase)) {
                    $remove = $true
                    break
                }
            }
        }

        if ($remove) {
            $removedEntries.Add($entry)
        }
        else {
            $keptEntries.Add($entry)
        }
    }

    if ($removedEntries.Count -eq 0) {
        Write-Host 'No matching user PATH entries found.' -ForegroundColor DarkGray
        return
    }

    foreach ($entry in $removedEntries) {
        if ($Commit) {
            Write-Change "Removing user PATH entry: $entry"
        }
        else {
            Write-Preview "Would remove user PATH entry: $entry"
        }
    }

    if ($Commit) {
        Set-UserPathEntries -Entries $keptEntries.ToArray()
    }
}

function Remove-PathIfPresent {
    param(
        [Parameter(Mandatory = $true)]
        [string]$Path,
        [Parameter(Mandatory = $true)]
        [ValidateSet('File', 'Directory')]
        [string]$PathType,
        [Parameter(Mandatory = $true)]
        [bool]$Commit
    )

    if (-not (Test-Path -LiteralPath $Path)) {
        Write-Host "Not present: $Path" -ForegroundColor DarkGray
        return
    }

    if (-not $Commit) {
        Write-Preview "Would remove ${PathType}: $Path"
        return
    }

    if ($PathType -eq 'Directory') {
        Remove-Item -LiteralPath $Path -Recurse -Force
    }
    else {
        Remove-Item -LiteralPath $Path -Force
    }

    Write-Change "Removed ${PathType}: $Path"
}

function Remove-MatchingDirectories {
    param(
        [Parameter(Mandatory = $true)]
        [string]$ParentPath,
        [Parameter(Mandatory = $true)]
        [string]$Filter,
        [Parameter(Mandatory = $true)]
        [bool]$Commit
    )

    if (-not (Test-Path -LiteralPath $ParentPath)) {
        Write-Host "Parent path not present: $ParentPath" -ForegroundColor DarkGray
        return
    }

    try {
        $matches = @(Get-ChildItem -LiteralPath $ParentPath -Directory -Filter $Filter -ErrorAction Stop)
    }
    catch [System.UnauthorizedAccessException] {
        if ($Commit) {
            throw
        }

        Write-WarningMessage "Skipping inaccessible directory enumeration: $ParentPath"
        return
    }

    if ($matches.Count -eq 0) {
        Write-Host "No directories matched $Filter under $ParentPath" -ForegroundColor DarkGray
        return
    }

    foreach ($match in $matches) {
        Remove-PathIfPresent -Path $match.FullName -PathType 'Directory' -Commit $Commit
    }
}

$localAppData = $env:LOCALAPPDATA
$appData = $env:APPDATA

$exactPathEntries = @(
    (Join-Path $localAppData 'Python\bin'),
    (Join-Path $localAppData 'Programs\Python\Python39'),
    (Join-Path $localAppData 'Programs\Python\Python39\Scripts')
)

$prefixPathEntries = @(
    (Join-Path $localAppData 'Programs\Python\Python314')
)

$deletePaths = @(
    @{
        Path = (Join-Path $localAppData 'Python')
        PathType = 'Directory'
    },
    @{
        Path = (Join-Path $localAppData 'Programs\Python\Launcher')
        PathType = 'Directory'
    },
    @{
        Path = (Join-Path $appData 'Python\pymanager.json')
        PathType = 'File'
    }
)

Write-Step 'Reviewing user PATH cleanup'
Remove-UserPathEntries -ExactEntries $exactPathEntries -PrefixEntries $prefixPathEntries -Commit:$Apply

Write-Step 'Reviewing leftover user-local Python directories and files'
foreach ($item in $deletePaths) {
    Remove-PathIfPresent -Path $item.Path -PathType $item.PathType -Commit:$Apply
}
Remove-MatchingDirectories -ParentPath (Join-Path $localAppData 'Programs\Python') -Filter 'Python314*' -Commit:$Apply

Write-Step 'Cleanup summary'
if ($Apply) {
    Write-Change 'Cleanup applied. Restart shells before re-checking PATH-dependent commands.'
}
else {
    Write-Preview 'No changes were made. Re-run with -Apply to perform the cleanup.'
}
