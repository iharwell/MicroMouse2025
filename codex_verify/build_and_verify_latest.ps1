param(
    [ValidateSet('BuildOnly', 'VerifyOnly', 'BuildAndVerify')]
    [string]$Mode = 'BuildAndVerify',
    [ValidateSet('Build', 'Rebuild')]
    [string]$HostBuildTarget = 'Build',
    [ValidateSet('Incremental', 'ProjectDefault')]
    [string]$HostLtcgMode = 'ProjectDefault',
    [ValidateRange(1, 128)]
    [int]$TeensyCompileJobs = 12,
    [string]$LogFilePath
)

Set-StrictMode -Version Latest
$ErrorActionPreference = 'Stop'

$scriptRoot = Split-Path -Parent $MyInvocation.MyCommand.Path
$repoRoot = Split-Path -Parent $scriptRoot
$runStamp = Get-Date -Format 'yyyyMMdd_HHmmss_fff'
$defaultLogDirectory = Join-Path $scriptRoot 'logs'
$requiresBuild = $Mode -ne 'VerifyOnly'
$requiresTests = $Mode -ne 'BuildOnly'
$artifactVerb = if ($requiresBuild) { 'Built' } else { 'Verified' }

function Get-NormalizedSingleLineText {
    param(
        [AllowNull()]
        [string]$Value
    )

    if ([string]::IsNullOrWhiteSpace($Value)) {
        return $null
    }

    return (($Value -replace '\s+', ' ').Trim())
}

function Test-IsElevatedProcess {
    try {
        $currentIdentity = [Security.Principal.WindowsIdentity]::GetCurrent()
        if ($null -eq $currentIdentity) {
            return $false
        }

        $principal = [Security.Principal.WindowsPrincipal]::new($currentIdentity)
        return $principal.IsInRole([Security.Principal.WindowsBuiltInRole]::Administrator)
    }
    catch {
        return $false
    }
}

function Get-ProcessSnapshot {
    param(
        [int]$ProcessId
    )

    if ($ProcessId -le 0) {
        return $null
    }

    try {
        $process = Get-CimInstance -ClassName Win32_Process -Filter ("ProcessId = {0}" -f $ProcessId) -ErrorAction Stop
        return [pscustomobject]@{
            ProcessId = [int]$process.ProcessId
            ParentProcessId = [int]$process.ParentProcessId
            Name = Get-NormalizedSingleLineText -Value $process.Name
            ExecutablePath = Get-NormalizedSingleLineText -Value $process.ExecutablePath
            CommandLine = Get-NormalizedSingleLineText -Value $process.CommandLine
        }
    }
    catch {
        try {
            $process = Get-Process -Id $ProcessId -ErrorAction Stop
            return [pscustomobject]@{
                ProcessId = [int]$process.Id
                ParentProcessId = 0
                Name = Get-NormalizedSingleLineText -Value $process.ProcessName
                ExecutablePath = Get-NormalizedSingleLineText -Value $process.Path
                CommandLine = $null
            }
        }
        catch {
            return $null
        }
    }
}

function Format-ProcessSnapshot {
    param(
        [AllowNull()]
        [psobject]$Process
    )

    if ($null -eq $Process) {
        return $null
    }

    $parts = [System.Collections.Generic.List[string]]::new()
    if (-not [string]::IsNullOrWhiteSpace($Process.Name)) {
        $parts.Add([string]$Process.Name)
    }

    $parts.Add(("PID {0}" -f $Process.ProcessId))

    if (-not [string]::IsNullOrWhiteSpace($Process.CommandLine)) {
        $parts.Add([string]$Process.CommandLine)
    }
    elseif (-not [string]::IsNullOrWhiteSpace($Process.ExecutablePath)) {
        $parts.Add([string]$Process.ExecutablePath)
    }

    return ($parts -join ' | ')
}

function Test-IsBuildWrapperProcess {
    param(
        [AllowNull()]
        [psobject]$Process
    )

    if ($null -eq $Process) {
        return $false
    }

    if ([string]::IsNullOrWhiteSpace($Process.Name) -or
        $Process.Name -notmatch '^(cmd\.exe|powershell\.exe|pwsh\.exe)$') {
        return $false
    }

    return (-not [string]::IsNullOrWhiteSpace($Process.CommandLine)) -and
        ($Process.CommandLine -match 'build_and_verify_latest|build_latest|test_latest_binaries')
}

function Get-ProcessLaunchChain {
    $chain = [System.Collections.Generic.List[string]]::new()
    $visitedProcessIds = [System.Collections.Generic.HashSet[int]]::new()
    $currentProcessId = $PID

    for ($depth = 0; $depth -lt 5 -and $currentProcessId -gt 0; $depth++) {
        if (-not $visitedProcessIds.Add($currentProcessId)) {
            break
        }

        $snapshot = Get-ProcessSnapshot -ProcessId $currentProcessId
        if ($null -eq $snapshot) {
            break
        }

        $formattedSnapshot = Format-ProcessSnapshot -Process $snapshot
        if (-not [string]::IsNullOrWhiteSpace($formattedSnapshot)) {
            $chain.Add($formattedSnapshot)
        }

        $currentProcessId = [int]$snapshot.ParentProcessId
    }

    if ($chain.Count -eq 0) {
        return $null
    }

    return ($chain -join ' <- ')
}

function Get-InvocationContext {
    $self = Get-ProcessSnapshot -ProcessId $PID
    $parent = $null
    if ($null -ne $self -and $self.ParentProcessId -gt 0) {
        $parent = Get-ProcessSnapshot -ProcessId $self.ParentProcessId
    }

    $selectedCallerProcess = $parent
    if (Test-IsBuildWrapperProcess -Process $parent) {
        $grandParent = Get-ProcessSnapshot -ProcessId $parent.ParentProcessId
        if ($null -ne $grandParent) {
            $selectedCallerProcess = $grandParent
        }
    }

    $currentIdentity = $null
    try {
        $currentIdentity = [Security.Principal.WindowsIdentity]::GetCurrent()
    }
    catch {
    }

    $caller = Get-NormalizedSingleLineText -Value $env:MM_BUILD_SCRIPT_CALLER
    if ([string]::IsNullOrWhiteSpace($caller)) {
        $caller = Get-NormalizedSingleLineText -Value $env:CODEX_INTERNAL_ORIGINATOR_OVERRIDE
    }

    if ([string]::IsNullOrWhiteSpace($caller) -and $null -ne $selectedCallerProcess) {
        $caller = Format-ProcessSnapshot -Process $selectedCallerProcess
    }

    $callerCommand = Get-NormalizedSingleLineText -Value $env:MM_BUILD_SCRIPT_CALLER_COMMAND
    if ([string]::IsNullOrWhiteSpace($callerCommand) -and
        $null -ne $selectedCallerProcess -and
        -not [string]::IsNullOrWhiteSpace($selectedCallerProcess.CommandLine)) {
        $callerCommand = [string]$selectedCallerProcess.CommandLine
    }

    return [pscustomobject]@{
        User = if ($null -ne $currentIdentity) {
            Get-NormalizedSingleLineText -Value $currentIdentity.Name
        }
        else {
            Get-NormalizedSingleLineText -Value $env:USERNAME
        }
        Elevated = Test-IsElevatedProcess
        Caller = if ([string]::IsNullOrWhiteSpace($caller)) { 'Unknown' } else { $caller }
        CallerCommand = $callerCommand
        LaunchChain = Get-ProcessLaunchChain
        LauncherPath = Get-NormalizedSingleLineText -Value $env:MM_BUILD_SCRIPT_LAUNCHER_PATH
        LauncherArguments = Get-NormalizedSingleLineText -Value $env:MM_BUILD_SCRIPT_LAUNCHER_ARGS
    }
}

switch ($Mode) {
    'BuildOnly' {
        $logFilePrefix = 'build_latest_'
        $logTitle = 'Build latest log'
        $logPathLabel = 'Build log'
        $launcherStepLabel = 'Checking build launcher'
        $launcherSuccessLabel = 'Build launcher check passed.'
        $modeWrapperPath = Join-Path $scriptRoot 'build_latest.cmd'
        $launcherEnvironmentVariables = @(
            'MM_BUILD_LATEST_LAUNCHED_FROM_CMD',
            'MM_BUILD_AND_VERIFY_LAUNCHED_FROM_CMD'
        )
        $finalStepLabel = 'Build completed'
    }
    'VerifyOnly' {
        $logFilePrefix = 'verify_latest_build_'
        $logTitle = 'Verify latest build log'
        $logPathLabel = 'Verify log'
        $launcherStepLabel = 'Checking verify launcher'
        $launcherSuccessLabel = 'Verify launcher check passed.'
        $modeWrapperPath = Join-Path $scriptRoot 'test_latest_binaries.cmd'
        $launcherEnvironmentVariables = @(
            'MM_VERIFY_LAUNCHED_FROM_CMD',
            'MM_BUILD_AND_VERIFY_LAUNCHED_FROM_CMD'
        )
        $finalStepLabel = 'Verify completed'
    }
    default {
        $logFilePrefix = 'build_and_verify_latest_'
        $logTitle = 'Build and verify latest log'
        $logPathLabel = 'Build-and-verify log'
        $launcherStepLabel = 'Checking build-and-verify launcher'
        $launcherSuccessLabel = 'Build-and-verify launcher check passed.'
        $modeWrapperPath = Join-Path $scriptRoot 'build_and_verify_latest.cmd'
        $launcherEnvironmentVariables = @('MM_BUILD_AND_VERIFY_LAUNCHED_FROM_CMD')
        $finalStepLabel = 'Build and verify completed'
    }
}

if ([string]::IsNullOrWhiteSpace($LogFilePath)) {
    $LogFilePath = Join-Path $defaultLogDirectory ($logFilePrefix + $runStamp + '.txt')
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
    $logTitle
    ('Start time: ' + (Get-Date -Format 'yyyy-MM-dd HH:mm:ss.fff zzz'))
    ('Repository: ' + $repoRoot)
    ('Mode: ' + $Mode)
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

Write-LogLine ("{0}: {1}" -f $logPathLabel, $LogFilePath) 'DarkCyan'
$invocationContext = Get-InvocationContext
if (-not [string]::IsNullOrWhiteSpace($invocationContext.User)) {
    Write-LogLine ("User: {0}" -f $invocationContext.User) 'DarkCyan'
}

Write-LogLine ("Elevation: {0}" -f $(if ($invocationContext.Elevated) { 'Elevated' } else { 'Not elevated' })) 'DarkCyan'
Write-LogLine ("Caller: {0}" -f $invocationContext.Caller) 'DarkCyan'
if (-not [string]::IsNullOrWhiteSpace($invocationContext.CallerCommand)) {
    Write-LogLine ("Caller command: {0}" -f $invocationContext.CallerCommand) 'DarkGray'
}

if (-not [string]::IsNullOrWhiteSpace($invocationContext.LauncherPath)) {
    $launcherLine = $invocationContext.LauncherPath
    if (-not [string]::IsNullOrWhiteSpace($invocationContext.LauncherArguments)) {
        $launcherLine = "{0} {1}" -f $launcherLine, $invocationContext.LauncherArguments
    }

    Write-LogLine ("Launcher: {0}" -f $launcherLine) 'DarkGray'
}

if (($invocationContext.Elevated -or $invocationContext.Caller -eq 'Unknown') -and
    -not [string]::IsNullOrWhiteSpace($invocationContext.LaunchChain)) {
    Write-LogLine ("Launch chain: {0}" -f $invocationContext.LaunchChain) 'DarkGray'
}

$script:SuppressTerminalErrorSummary = $false
$scriptExitCode = 0

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

function Resolve-ExistingPath {
    param(
        [Parameter(Mandatory = $true)]
        [string[]]$CandidatePaths,
        [Parameter(Mandatory = $true)]
        [string]$Description
    )

    foreach ($candidatePath in $CandidatePaths) {
        if (Test-Path -LiteralPath $candidatePath -PathType Leaf) {
            return [System.IO.Path]::GetFullPath($candidatePath)
        }
    }

    throw ("{0} not found. Checked: {1}" -f $Description, ($CandidatePaths -join ', '))
}

function Set-FileContentIfChanged {
    param(
        [Parameter(Mandatory = $true)]
        [string]$Path,
        [Parameter(Mandatory = $true)]
        [AllowEmptyString()]
        [string[]]$Lines,
        [ValidateSet('ASCII', 'UTF8')]
        [string]$Encoding = 'ASCII'
    )

    $newContent = ($Lines -join [Environment]::NewLine) + [Environment]::NewLine
    if (Test-Path -LiteralPath $Path -PathType Leaf) {
        $existingContent = [System.IO.File]::ReadAllText($Path)
        if ($existingContent -ceq $newContent) {
            return
        }
    }

    Set-Content -LiteralPath $Path -Value $Lines -Encoding $Encoding
}

function Ensure-JunctionTarget {
    param(
        [Parameter(Mandatory = $true)]
        [string]$Path,
        [Parameter(Mandatory = $true)]
        [string]$Target
    )

    $expectedTarget = [System.IO.Path]::GetFullPath($Target)
    $existingItem = Get-Item -LiteralPath $Path -Force -ErrorAction SilentlyContinue
    if ($null -ne $existingItem) {
        $existingTargets = @($existingItem.Target | Where-Object { -not [string]::IsNullOrWhiteSpace($_) })
        $isReparsePoint = (($existingItem.Attributes -band [System.IO.FileAttributes]::ReparsePoint) -ne 0)
        if ($isReparsePoint -and $existingTargets.Count -gt 0) {
            foreach ($existingTarget in $existingTargets) {
                if ([System.StringComparer]::OrdinalIgnoreCase.Equals(
                        [System.IO.Path]::GetFullPath($existingTarget),
                        $expectedTarget)) {
                    return
                }
            }
        }

        Remove-Item -LiteralPath $Path -Force -Recurse
    }

    New-Item -ItemType Junction -Path $Path -Target $expectedTarget | Out-Null
}

function Test-IsCodexShell {
    return ($env:CODEX_SHELL -eq '1') -or (-not [string]::IsNullOrWhiteSpace($env:CODEX_INTERNAL_ORIGINATOR_OVERRIDE))
}

function Assert-SupportedLauncher {
    param(
        [Parameter(Mandatory = $true)]
        [string]$WrapperPath,
        [Parameter(Mandatory = $true)]
        [string[]]$AcceptedEnvironmentVariables,
        [switch]$SuggestElevation
    )

    if (-not (Test-IsCodexShell)) {
        return
    }

    foreach ($environmentVariable in $AcceptedEnvironmentVariables) {
        $environmentEntry = Get-Item -LiteralPath ('Env:' + $environmentVariable) -ErrorAction SilentlyContinue
        if ($null -ne $environmentEntry -and $environmentEntry.Value -eq '1') {
            return
        }
    }

    if ($SuggestElevation) {
        throw ('ELEVATION_REQUIRED: Inside Codex, launch "' +
            $WrapperPath +
            '" and request elevation for that command. Do not invoke build_and_verify_latest.ps1 directly from Codex or bypass it with direct msbuild, arduino-cli, or vstest commands.')
    }

    throw ('LAUNCHER_REQUIRED: Inside Codex, launch "' +
        $WrapperPath +
        '". Do not invoke build_and_verify_latest.ps1 directly from Codex or bypass it with direct msbuild, arduino-cli, or vstest commands.')
}

function Assert-UnsandboxedBuildEnvironment {
    param(
        [Parameter(Mandatory = $true)]
        [string]$WrapperPath
    )

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
                '", which forces a full host rebuild. Re-run this flow through ' +
                $WrapperPath +
                ' with elevated permissions; do not bypass it with direct msbuild, arduino-cli, or vstest commands.')
        }

        throw
    }
}

function Write-VsTestFailureConsoleOutput {
    param(
        [Parameter(Mandatory = $true)]
        [string]$StdoutPath,
        [Parameter(Mandatory = $true)]
        [string]$StderrPath
    )

    $selectedLines = [System.Collections.Generic.List[string]]::new()
    $capturingFailureBlock = $false

    if (Test-Path -LiteralPath $StdoutPath -PathType Leaf) {
        foreach ($line in Get-Content -LiteralPath $StdoutPath) {
            if ($line -match '^\s*Failed\s+') {
                if ($selectedLines.Count -gt 0 -and $selectedLines[$selectedLines.Count - 1] -ne '') {
                    $selectedLines.Add('')
                }

                $capturingFailureBlock = $true
                $selectedLines.Add($line)
                continue
            }

            if ($line -match '^Total tests:' -or
                $line -match '^\s+Passed:' -or
                $line -match '^\s+Failed:' -or
                $line -match '^ Total time:' -or
                $line -match '^Test Run Failed\.') {
                $capturingFailureBlock = $false
                $selectedLines.Add($line)
                continue
            }

            if (-not $capturingFailureBlock) {
                continue
            }

            if ($line -match '^\s*Passed\s+' -or
                $line -match '^\s*Skipped\s+' -or
                $line -match '^\s*Not Run\s+') {
                $capturingFailureBlock = $false
                continue
            }

            $selectedLines.Add($line)
        }
    }

    if ($selectedLines.Count -eq 0) {
        Write-LogFileContents -Path $StdoutPath
    }
    else {
        Write-LogLine 'VSTest failure summary:' 'Red'
        foreach ($line in $selectedLines) {
            if ($line -match '^\s*Failed\s+' -or $line -match '^Test Run Failed\.') {
                Write-LogLine -Message $line -Color 'Red'
                continue
            }

            if ($line -match '^\s*Error Message:') {
                Write-LogLine -Message $line -Color 'Yellow'
                continue
            }

            if ($line -match '^\s*Stack Trace:') {
                Write-LogLine -Message $line -Color 'DarkYellow'
                continue
            }

            Write-LogLine -Message $line
        }
    }

    if (Test-Path -LiteralPath $StderrPath -PathType Leaf) {
        $stderrLines = @(Get-Content -LiteralPath $StderrPath | Where-Object { -not [string]::IsNullOrWhiteSpace($_) })
        if ($stderrLines.Count -gt 0) {
            Write-LogLine 'VSTest stderr:' 'Red'
            Write-LogFileContents -Path $StderrPath -Color 'Red'
        }
    }
}

function Invoke-External {
    param(
        [Parameter(Mandatory = $true)]
        [string]$FilePath,
        [Parameter(Mandatory = $true)]
        [string[]]$Arguments,
        [int]$SuccessTailLineCount = 0,
        [string]$SuccessTailHeader,
        [ValidateSet('All', 'VSTestFailuresOnly')]
        [string]$FailureConsoleOutputMode = 'All'
    )

    Write-LogLine ($FilePath + ' ' + ($Arguments -join ' '))
    $stdoutPath = [System.IO.Path]::GetTempFileName()
    $stderrPath = [System.IO.Path]::GetTempFileName()

    try {
        $previousErrorActionPreference = $ErrorActionPreference
        try {
            # Windows PowerShell 5.1 can surface native stderr as error records
            # even when the process exits successfully. Judge native tools by
            # their exit code and captured stderr instead of terminating here.
            $ErrorActionPreference = 'Continue'
            & $FilePath @Arguments 1> $stdoutPath 2> $stderrPath
            $exitCode = $LASTEXITCODE
        }
        finally {
            $ErrorActionPreference = $previousErrorActionPreference
        }
        Append-FileToLog -Path $stdoutPath
        Append-FileToLog -Path $stderrPath
        if ($exitCode -ne 0) {
            if ($FailureConsoleOutputMode -eq 'VSTestFailuresOnly') {
                $script:SuppressTerminalErrorSummary = $true
                Write-VsTestFailureConsoleOutput -StdoutPath $stdoutPath -StderrPath $stderrPath
            }
            else {
                Write-LogFileContents -Path $stdoutPath
                Write-LogFileContents -Path $stderrPath -Color 'Red'
            }

            throw "Command failed with exit code $exitCode."
        }

        if ($SuccessTailLineCount -gt 0) {
            $recentOutputLines = [System.Collections.Generic.List[string]]::new()
            foreach ($capturedPath in @($stdoutPath, $stderrPath)) {
                if (-not (Test-Path -LiteralPath $capturedPath -PathType Leaf)) {
                    continue
                }

                foreach ($line in Get-Content -LiteralPath $capturedPath) {
                    if ([string]::IsNullOrWhiteSpace($line)) {
                        continue
                    }

                    $recentOutputLines.Add($line)
                }
            }

            if ($recentOutputLines.Count -gt 0) {
                if (-not [string]::IsNullOrWhiteSpace($SuccessTailHeader)) {
                    Write-LogLine $SuccessTailHeader 'DarkCyan'
                }

                $tailStartIndex = [Math]::Max(0, $recentOutputLines.Count - $SuccessTailLineCount)
                for ($i = $tailStartIndex; $i -lt $recentOutputLines.Count; $i++) {
                    Write-LogLine $recentOutputLines[$i] 'DarkGray'
                }
            }
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
        & cmd.exe /c $CommandLine 1> $stdoutPath 2> $stderrPath
        $exitCode = $LASTEXITCODE
        Append-FileToLog -Path $stdoutPath
        Append-FileToLog -Path $stderrPath
        if ($exitCode -ne 0) {
            Write-LogFileContents -Path $stdoutPath
            Write-LogFileContents -Path $stderrPath -Color 'Red'
            throw "Command failed with exit code $exitCode."
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

function Invoke-HostMsBuild {
    param(
        [Parameter(Mandatory = $true)]
        [string]$VsDevCmd,
        [Parameter(Mandatory = $true)]
        [string]$SolutionPath,
        [Parameter(Mandatory = $true)]
        [string[]]$ProjectTargetNames,
        [Parameter(Mandatory = $true)]
        [string]$HostBuildTarget,
        [Parameter(Mandatory = $true)]
        [string]$HostLtcgMode
    )

    $vsDevCmdArguments = @(
        '-no_logo',
        '-host_arch=x64',
        '-arch=x64'
    )
    $hostTargetSpecification = ($ProjectTargetNames | ForEach-Object {
        if ($HostBuildTarget -eq 'Build') {
            $_
        }
        else {
            '{0}:{1}' -f $_, $HostBuildTarget
        }
    }) -join ';'
    $hostMsBuildArguments = @(
        $SolutionPath,
        '/m',
        ('/t:' + $hostTargetSpecification),
        '/p:Configuration=Release',
        '/p:Platform=x64',
        '/p:PreferredToolArchitecture=x64',
        '/p:UseMultiToolTask=false',
        '/p:BuildPassReferences=true',
        '/v:m'
    )

    if ($HostLtcgMode -eq 'Incremental') {
        $hostMsBuildArguments += '/p:LinkTimeCodeGeneration=UseFastLinkTimeCodeGeneration'
    }

    Invoke-CmdChain -CommandLine (
        'call "{0}" {1} && msbuild {2}' -f
        $VsDevCmd,
        ($vsDevCmdArguments -join ' '),
        (New-MsBuildArgumentString -Arguments $hostMsBuildArguments)
    )
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
    $eigenHeaderPath = Join-Path $EigenSourceRoot 'Eigen\Core'

    New-Item -ItemType Directory -Path $srcRoot -Force | Out-Null
    Assert-PathExists -Path $eigenHeaderPath -Description 'Eigen core header'

    Ensure-JunctionTarget -Path $eigenIncludeRoot -Target (Join-Path $EigenSourceRoot 'Eigen')

    Set-FileContentIfChanged -Path $entryHeaderPath -Encoding ASCII -Lines @(
        '#pragma once'
        ''
        '// Marker header for Arduino library discovery. Runtime code includes'
        '// Eigen through MazeMap/EigenCompat.h so this header stays zero-cost.'
    )

    Set-FileContentIfChanged -Path $libraryPropertiesPath -Encoding ASCII -Lines @(
        'name=Eigen'
        'version=5.0.0'
        'author=Eigen Project'
        'maintainer=Eigen Project'
        'sentence=Repo-local staging wrapper for the shared Eigen checkout.'
        'paragraph=Generated by codex_verify/build_and_verify_latest.ps1 so Arduino can resolve Eigen headers without modifying the installed Teensy core.'
        'category=Data Processing'
        'includes=Eigen.h'
        'architectures=*'
    )

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

function Get-RelativePathWithinRoot {
    param(
        [Parameter(Mandatory = $true)]
        [string]$Path,
        [Parameter(Mandatory = $true)]
        [string]$Root
    )

    if (-not (Test-IsPathUnderRoot -Path $Path -Root $Root)) {
        throw ("Path is not under root. Path={0}; Root={1}" -f $Path, $Root)
    }

    $fullPath = [System.IO.Path]::GetFullPath($Path)
    $rootPrefix = [System.IO.Path]::GetFullPath($Root).TrimEnd('\') + '\'
    if ($fullPath.Equals($rootPrefix.TrimEnd('\'), [System.StringComparison]::OrdinalIgnoreCase)) {
        return ''
    }

    return $fullPath.Substring($rootPrefix.Length)
}

function Get-TrackedPathsFromArduinoDependencyFile {
    param(
        [Parameter(Mandatory = $true)]
        [string]$Path
    )

    $trackedPaths = [System.Collections.Generic.HashSet[string]]::new([System.StringComparer]::OrdinalIgnoreCase)
    $foundDependencySeparator = $false

    foreach ($line in Get-Content -LiteralPath $Path) {
        if ([string]::IsNullOrWhiteSpace($line)) {
            continue
        }

        $normalizedLine = $line.Trim()
        if (-not $foundDependencySeparator) {
            $separatorIndex = $normalizedLine.IndexOf(':')
            if ($separatorIndex -lt 0) {
                continue
            }

            $normalizedLine = $normalizedLine.Substring($separatorIndex + 1).Trim()
            $foundDependencySeparator = $true
        }

        $normalizedLine = $normalizedLine.TrimEnd('\').Trim()
        if ([string]::IsNullOrWhiteSpace($normalizedLine)) {
            continue
        }

        $candidatePath = $normalizedLine -replace '\\ ', ' '
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

    return @($trackedPaths)
}

function Convert-ArduinoDependencyToRepoPath {
    param(
        [Parameter(Mandatory = $true)]
        [string]$Path,
        [Parameter(Mandatory = $true)]
        [string]$RepoRoot,
        [Parameter(Mandatory = $true)]
        [string]$CanonicalBuildPath,
        [Parameter(Mandatory = $true)]
        [string]$FirmwareSketchDir,
        [Parameter(Mandatory = $true)]
        [string]$SketchDir,
        [Parameter(Mandatory = $true)]
        [string]$ArduinoEigenLibraryDir,
        [Parameter(Mandatory = $true)]
        [string]$EigenSourceRoot,
        [Parameter(Mandatory = $true)]
        [string]$ArduinoStubHeaderPath
    )

    $fullPath = [System.IO.Path]::GetFullPath($Path)
    if (-not (Test-IsPathUnderRoot -Path $fullPath -Root $RepoRoot)) {
        return $null
    }

    if (-not (Test-IsPathUnderRoot -Path $fullPath -Root $CanonicalBuildPath)) {
        if (Test-IsPathUnderRoot -Path $fullPath -Root $ArduinoEigenLibraryDir) {
            return $null
        }

        return $fullPath
    }

    if (Test-IsPathUnderRoot -Path $fullPath -Root $FirmwareSketchDir) {
        $relativePath = Get-RelativePathWithinRoot -Path $fullPath -Root $FirmwareSketchDir
        if ($relativePath.EndsWith('.ino.cpp', [System.StringComparison]::OrdinalIgnoreCase)) {
            $relativePath = $relativePath.Substring(0, $relativePath.Length - 4)
        }

        $candidatePath = Join-Path $SketchDir $relativePath
        if (Test-Path -LiteralPath $candidatePath -PathType Leaf) {
            return [System.IO.Path]::GetFullPath($candidatePath)
        }

        return $null
    }

    $arduinoEigenSourceRoot = Join-Path $ArduinoEigenLibraryDir 'src'
    if (Test-IsPathUnderRoot -Path $fullPath -Root $arduinoEigenSourceRoot) {
        $relativePath = Get-RelativePathWithinRoot -Path $fullPath -Root $arduinoEigenSourceRoot
        if ([string]::IsNullOrWhiteSpace($relativePath) -or
            $relativePath.Equals('Eigen.h', [System.StringComparison]::OrdinalIgnoreCase) -or
            $relativePath.Equals('library.properties', [System.StringComparison]::OrdinalIgnoreCase)) {
            return $null
        }

        $candidatePath = Join-Path $EigenSourceRoot $relativePath
        if (Test-Path -LiteralPath $candidatePath -PathType Leaf) {
            return [System.IO.Path]::GetFullPath($candidatePath)
        }

        return $null
    }

    $firmwarePchDir = Join-Path $CanonicalBuildPath 'firmware\pch'
    if ((Test-IsPathUnderRoot -Path $fullPath -Root $firmwarePchDir) -and
        $fullPath.EndsWith('Arduino.h', [System.StringComparison]::OrdinalIgnoreCase) -and
        (Test-Path -LiteralPath $ArduinoStubHeaderPath -PathType Leaf)) {
        return [System.IO.Path]::GetFullPath($ArduinoStubHeaderPath)
    }

    return $null
}

function Get-LatestRepoInputWriteTimeFromArduinoDependencyFiles {
    param(
        [Parameter(Mandatory = $true)]
        [string]$FirmwareOutputDir,
        [Parameter(Mandatory = $true)]
        [string]$RepoRoot,
        [Parameter(Mandatory = $true)]
        [string]$CanonicalBuildPath,
        [Parameter(Mandatory = $true)]
        [string]$SketchDir,
        [Parameter(Mandatory = $true)]
        [string]$ArduinoEigenLibraryDir,
        [Parameter(Mandatory = $true)]
        [string]$EigenSourceRoot,
        [Parameter(Mandatory = $true)]
        [string]$ArduinoStubHeaderPath
    )

    $firmwareSketchDir = Join-Path $FirmwareOutputDir 'sketch'
    Assert-PathExists -Path $firmwareSketchDir -Description 'Arduino sketch dependency directory'

    $dependencyFiles = @(Get-ChildItem -LiteralPath $firmwareSketchDir -File -Filter '*.d')
    if ($dependencyFiles.Count -eq 0) {
        throw ("No Arduino dependency files were found under {0}" -f $firmwareSketchDir)
    }

    $repoLocalPaths = [System.Collections.Generic.HashSet[string]]::new([System.StringComparer]::OrdinalIgnoreCase)
    foreach ($dependencyFile in $dependencyFiles) {
        foreach ($trackedPath in Get-TrackedPathsFromArduinoDependencyFile -Path $dependencyFile.FullName) {
            $repoLocalPath = Convert-ArduinoDependencyToRepoPath `
                -Path $trackedPath `
                -RepoRoot $RepoRoot `
                -CanonicalBuildPath $CanonicalBuildPath `
                -FirmwareSketchDir $firmwareSketchDir `
                -SketchDir $SketchDir `
                -ArduinoEigenLibraryDir $ArduinoEigenLibraryDir `
                -EigenSourceRoot $EigenSourceRoot `
                -ArduinoStubHeaderPath $ArduinoStubHeaderPath
            if ([string]::IsNullOrWhiteSpace($repoLocalPath)) {
                continue
            }

            [void]$repoLocalPaths.Add($repoLocalPath)
        }
    }

    $sketchEntryPoint = Join-Path $SketchDir 'MazeMap.ino'
    if (Test-Path -LiteralPath $sketchEntryPoint -PathType Leaf) {
        [void]$repoLocalPaths.Add([System.IO.Path]::GetFullPath($sketchEntryPoint))
    }

    if ($repoLocalPaths.Count -eq 0) {
        throw ("No repo-local Arduino inputs were found in dependency files under {0}" -f $firmwareSketchDir)
    }

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

        throw ("Arduino dependency files reference missing repo-local inputs: {0}" -f $sampleText)
    }

    if ($latestWriteTime -eq [datetime]::MinValue) {
        throw ("No repo-local Arduino inputs were found under {0}" -f $firmwareSketchDir)
    }

    return $latestWriteTime
}

function Get-LatestRepoInputWriteTimeFromProjectTLogs {
    param(
        [Parameter(Mandatory = $true)]
        [psobject]$Project,
        [Parameter(Mandatory = $true)]
        [string]$RepoRoot
    )

    $repoLocalPaths = [System.Collections.Generic.HashSet[string]]::new([System.StringComparer]::OrdinalIgnoreCase)
    foreach ($tlogNames in @(
        @('CL.read.1.tlog', 'Microsoft.Build.CPPTasks.CL.read.1.tlog'),
        @('link.read.1.tlog')
    )) {
        $tlogPath = Resolve-ExistingPath `
            -CandidatePaths ($tlogNames | ForEach-Object { Join-Path $Project.TLogDir $_ }) `
            -Description ("{0} tracking log" -f $Project.Name)

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

function New-ArtifactVerificationResult {
    param(
        [Parameter(Mandatory = $true)]
        [string]$Category,
        [Parameter(Mandatory = $true)]
        [string]$Description,
        [Parameter(Mandatory = $true)]
        [bool]$IsCurrent,
        [AllowNull()]
        [System.IO.FileInfo]$Item,
        [AllowNull()]
        [string]$Message
    )

    return [pscustomobject]@{
        Category = $Category
        Description = $Description
        IsCurrent = $IsCurrent
        Item = $Item
        Message = $Message
    }
}

function Get-ArtifactVerificationResult {
    param(
        [Parameter(Mandatory = $true)]
        [string]$Category,
        [Parameter(Mandatory = $true)]
        [string]$Path,
        [Parameter(Mandatory = $true)]
        [string]$Description,
        [Parameter(Mandatory = $true)]
        [datetime]$NotOlderThan
    )

    try {
        Assert-PathExists -Path $Path -Description $Description

        $item = Get-Item -LiteralPath $Path
        if ($item.LastWriteTime -lt $NotOlderThan) {
            return New-ArtifactVerificationResult `
                -Category $Category `
                -Description $Description `
                -IsCurrent $false `
                -Item $item `
                -Message ("{0} is stale. LastWriteTime={1}; expected at or after {2}." -f $Description, $item.LastWriteTime, $NotOlderThan)
        }

        return New-ArtifactVerificationResult `
            -Category $Category `
            -Description $Description `
            -IsCurrent $true `
            -Item $item `
            -Message $null
    }
    catch {
        return New-ArtifactVerificationResult `
            -Category $Category `
            -Description $Description `
            -IsCurrent $false `
            -Item $null `
            -Message $_.Exception.Message
    }
}

function Write-ArtifactVerificationResult {
    param(
        [Parameter(Mandatory = $true)]
        [string]$ArtifactVerb,
        [Parameter(Mandatory = $true)]
        [psobject]$Result
    )

    if ($Result.IsCurrent) {
        Write-LogLine ("{0} {1} ({2}, {3} bytes)" -f $ArtifactVerb, $Result.Item.FullName, $Result.Item.LastWriteTime, $Result.Item.Length) 'Green'
        return
    }

    Write-LogLine ("{0} verification issue: {1}" -f $Result.Category, $Result.Message) 'Yellow'
}

function Get-AndLogReleaseArtifactStatus {
    param(
        [Parameter(Mandatory = $true)]
        [string]$ArtifactVerb,
        [Parameter(Mandatory = $true)]
        [string]$FirmwareOutputDir,
        [Parameter(Mandatory = $true)]
        [string]$HexPath,
        [Parameter(Mandatory = $true)]
        [string]$RepoRoot,
        [Parameter(Mandatory = $true)]
        [string]$CanonicalBuildPath,
        [Parameter(Mandatory = $true)]
        [string]$SketchDir,
        [Parameter(Mandatory = $true)]
        [string]$ArduinoEigenLibraryDir,
        [Parameter(Mandatory = $true)]
        [string]$EigenSourceRoot,
        [Parameter(Mandatory = $true)]
        [string]$ArduinoStubHeaderPath,
        [Parameter(Mandatory = $true)]
        [psobject]$MazeMapProject,
        [Parameter(Mandatory = $true)]
        [psobject]$MazeMapTestProject,
        [Parameter(Mandatory = $true)]
        [psobject]$MazeSimulationProject,
        [Parameter(Mandatory = $true)]
        [string]$MazeMapDllPath,
        [Parameter(Mandatory = $true)]
        [string]$TestDllPath,
        [Parameter(Mandatory = $true)]
        [string]$SimulationExePath,
        [Parameter(Mandatory = $true)]
        [bool]$RequireFirmwareImage,
        [Parameter(Mandatory = $true)]
        [bool]$RequireSimulationBinary
    )

    try {
        $firmwareSourceCutoff = Get-LatestRepoInputWriteTimeFromArduinoDependencyFiles `
            -FirmwareOutputDir $FirmwareOutputDir `
            -RepoRoot $RepoRoot `
            -CanonicalBuildPath $CanonicalBuildPath `
            -SketchDir $SketchDir `
            -ArduinoEigenLibraryDir $ArduinoEigenLibraryDir `
            -EigenSourceRoot $EigenSourceRoot `
            -ArduinoStubHeaderPath $ArduinoStubHeaderPath
        $firmwareResult = Get-ArtifactVerificationResult `
            -Category 'Teensy' `
            -Path $HexPath `
            -Description 'Compiled firmware image' `
            -NotOlderThan $firmwareSourceCutoff
    }
    catch {
        $firmwareResult = New-ArtifactVerificationResult `
            -Category 'Teensy' `
            -Description 'Compiled firmware image' `
            -IsCurrent $false `
            -Item $null `
            -Message ("Compiled firmware image could not be validated: {0}" -f $_.Exception.Message)
    }

    try {
        $mazeMapSourceCutoff = Get-LatestRepoInputWriteTimeFromProjectTLogs -Project $MazeMapProject -RepoRoot $RepoRoot
        $mazeMapResult = Get-ArtifactVerificationResult `
            -Category 'Host' `
            -Path $MazeMapDllPath `
            -Description 'Release MazeMap host binary' `
            -NotOlderThan $mazeMapSourceCutoff
    }
    catch {
        $mazeMapResult = New-ArtifactVerificationResult `
            -Category 'Host' `
            -Description 'Release MazeMap host binary' `
            -IsCurrent $false `
            -Item $null `
            -Message ("Release MazeMap host binary could not be validated: {0}" -f $_.Exception.Message)
    }

    try {
        $mazeMapTestSourceCutoff = Get-LatestRepoInputWriteTimeFromProjectTLogs -Project $MazeMapTestProject -RepoRoot $RepoRoot
        $testDllResult = Get-ArtifactVerificationResult `
            -Category 'Host' `
            -Path $TestDllPath `
            -Description 'Release test binary' `
            -NotOlderThan $mazeMapTestSourceCutoff
    }
    catch {
        $testDllResult = New-ArtifactVerificationResult `
            -Category 'Host' `
            -Description 'Release test binary' `
            -IsCurrent $false `
            -Item $null `
            -Message ("Release test binary could not be validated: {0}" -f $_.Exception.Message)
    }

    try {
        $mazeSimulationSourceCutoff = Get-LatestRepoInputWriteTimeFromProjectTLogs -Project $MazeSimulationProject -RepoRoot $RepoRoot
        $simulationResult = Get-ArtifactVerificationResult `
            -Category 'Host' `
            -Path $SimulationExePath `
            -Description 'Release MazeSimulation host binary' `
            -NotOlderThan $mazeSimulationSourceCutoff
    }
    catch {
        $simulationResult = New-ArtifactVerificationResult `
            -Category 'Host' `
            -Description 'Release MazeSimulation host binary' `
            -IsCurrent $false `
            -Item $null `
            -Message ("Release MazeSimulation host binary could not be validated: {0}" -f $_.Exception.Message)
    }

    $allResults = @($firmwareResult, $mazeMapResult, $testDllResult, $simulationResult)
    foreach ($result in $allResults) {
        Write-ArtifactVerificationResult -ArtifactVerb $ArtifactVerb -Result $result
    }

    $teensyReady = $firmwareResult.IsCurrent
    $hostTestReady = $mazeMapResult.IsCurrent -and $testDllResult.IsCurrent
    $hostReady = $hostTestReady -and $simulationResult.IsCurrent
    Write-LogLine ("Teensy verification status: {0}" -f $(if ($teensyReady) { 'ready' } else { 'issues detected' })) $(if ($teensyReady) { 'DarkCyan' } else { 'Yellow' })
    Write-LogLine ("Host test verification status: {0}" -f $(if ($hostTestReady) { 'ready' } else { 'issues detected' })) $(if ($hostTestReady) { 'DarkCyan' } else { 'Yellow' })
    Write-LogLine ("Host verification status: {0}" -f $(if ($hostReady) { 'ready' } else { 'issues detected' })) $(if ($hostReady) { 'DarkCyan' } else { 'Yellow' })

    $blockingFailureMessages = [System.Collections.Generic.List[string]]::new()
    $advisoryFailureMessages = [System.Collections.Generic.List[string]]::new()
    $resultPolicies = @(
        [pscustomobject]@{
            Result = $firmwareResult
            Required = $RequireFirmwareImage
        },
        [pscustomobject]@{
            Result = $mazeMapResult
            Required = $true
        },
        [pscustomobject]@{
            Result = $testDllResult
            Required = $true
        },
        [pscustomobject]@{
            Result = $simulationResult
            Required = $RequireSimulationBinary
        }
    )

    foreach ($resultPolicy in $resultPolicies) {
        $result = $resultPolicy.Result
        if (-not $result.IsCurrent) {
            $message = ("{0} verification issue: {1}" -f $result.Category, $result.Message)
            if ($resultPolicy.Required) {
                $blockingFailureMessages.Add($message)
            }
            else {
                $advisoryFailureMessages.Add($message)
            }
        }
    }

    return [pscustomobject]@{
        FirmwareImage = $firmwareResult.Item
        MazeMapDll = $mazeMapResult.Item
        TestDll = $testDllResult.Item
        SimulationExe = $simulationResult.Item
        TeensyReady = $teensyReady
        HostTestReady = $hostTestReady
        HostReady = $hostReady
        AllReady = $teensyReady -and $hostReady
        RequiredReady = $blockingFailureMessages.Count -eq 0
        BlockingFailureMessages = $blockingFailureMessages.ToArray()
        AdvisoryFailureMessages = $advisoryFailureMessages.ToArray()
    }
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
$buildPath = Join-Path $canonicalBuildPath 'build'
$firmwareOutputDir = Join-Path $canonicalBuildPath 'firmware'
$hexPath = Join-Path $firmwareOutputDir 'MazeMap.ino.hex'
$arduinoStubHeaderPath = Join-Path $scriptRoot 'Arduino.h'
$mazeMapDllPath = Join-Path $repoRoot 'MazeMap\x64\Release\MazeMap.dll'
$testDllPath = Join-Path $repoRoot 'MazeMap\x64\Release\MazeMapTest.dll'
$simulationExePath = Join-Path $repoRoot 'MazeMap\x64\Release\MazeSimulation.exe'
$fqbn = 'teensy:avr:teensy41'
$teensyBoardOptions = @('opt=o2lto')
$teensyOptimizationProfile = 'O2 + LTO'
Assert-PathExists -Path $sketchDir -Description 'Arduino sketch directory'
Assert-PathExists -Path $eigenIncludeDir -Description 'Shared Eigen include directory'
Assert-PathExists -Path $solutionPath -Description 'MazeMap solution'
Assert-PathExists -Path $arduinoStubHeaderPath -Description 'Arduino stub header'

if ($requiresBuild) {
    Assert-PathExists -Path $arduinoCli -Description 'Arduino CLI'
    Assert-PathExists -Path $vsDevCmd -Description 'Visual Studio developer command script'
}

if ($requiresTests) {
    Assert-PathExists -Path $vstest -Description 'VSTest console'
}

$eigenIncludeDir = (Resolve-Path -LiteralPath $eigenIncludeDir).Path
if ($requiresBuild) {
    $arduinoLibrariesDir = Ensure-ArduinoEigenLibrary -LibraryRoot $arduinoLibrariesDir -EigenSourceRoot $eigenIncludeDir
    $arduinoEigenLibraryDir = (Resolve-Path -LiteralPath $arduinoEigenLibraryDir).Path
}
else {
    $arduinoLibrariesDir = [System.IO.Path]::GetFullPath($arduinoLibrariesDir)
    $arduinoEigenLibraryDir = [System.IO.Path]::GetFullPath($arduinoEigenLibraryDir)
}
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
$requireFirmwareImage = $Mode -ne 'VerifyOnly'
$requireSimulationBinary = $Mode -ne 'VerifyOnly'
Push-Location $repoRoot
try {
    Write-Step $launcherStepLabel
    Assert-SupportedLauncher -WrapperPath $modeWrapperPath -AcceptedEnvironmentVariables $launcherEnvironmentVariables -SuggestElevation:$requiresBuild
    Write-LogLine $launcherSuccessLabel 'DarkCyan'

    if ($requiresBuild) {
        Write-Step 'Checking build environment'
        Assert-UnsandboxedBuildEnvironment -WrapperPath $modeWrapperPath
        Write-LogLine 'Host build environment access check passed.' 'DarkCyan'
    }

    if ($requiresBuild) {
        New-Item -ItemType Directory -Path $canonicalBuildPath -Force | Out-Null
        New-Item -ItemType Directory -Path $buildPath -Force | Out-Null
        New-Item -ItemType Directory -Path $firmwareOutputDir -Force | Out-Null
        Write-LogLine ("Resetting upload artifact path: {0}" -f $hexPath) 'DarkCyan'
        Remove-FileIfPresent -Path $hexPath

        Write-Step 'Compiling the Teensy sketch'
        Write-LogLine ("Teensy compile profile: {0} ({1})" -f $teensyOptimizationProfile, ($teensyBoardOptions -join ', ')) 'DarkCyan'
        Write-LogLine ("Teensy compile parallelism: --jobs {0}" -f $TeensyCompileJobs) 'DarkCyan'
        $teensyBuildStopwatch = [System.Diagnostics.Stopwatch]::StartNew()
        Invoke-External -FilePath $arduinoCli -Arguments @(
            'compile',
            '--jobs', $TeensyCompileJobs.ToString(),
            '--fqbn', $fqbn,
            '--board-options', ($teensyBoardOptions -join ','),
            '--libraries', $arduinoLibrariesDir,
            '--build-path', $buildPath,
            '--output-dir', $firmwareOutputDir,
            $sketchDir
        ) -SuccessTailLineCount 4 -SuccessTailHeader 'Teensy compile output tail:'
        $teensyBuildStopwatch.Stop()
        Write-LogLine ("Teensy compile completed in {0:n1}s" -f $teensyBuildStopwatch.Elapsed.TotalSeconds) 'DarkCyan'

        Write-Step 'Building the Release host targets'
        Write-LogLine ("Host build target: {0} (Release|x64)" -f $HostBuildTarget) 'DarkCyan'
        Write-LogLine ("Host LTCG mode: {0}" -f $HostLtcgMode) 'DarkCyan'
        Write-LogLine 'Host CL parallelism: project /MP settings from the vcxproj (UseMultiToolTask=false)' 'DarkCyan'
        Write-LogLine 'Host native project overlap: BuildPassReferences=true' 'DarkCyan'
        Write-LogLine 'Host toolchain entry: VsDevCmd -host_arch=x64 -arch=x64' 'DarkCyan'
        $hostProjectTargetNames = [System.Collections.Generic.List[string]]::new()
        $hostProjectTargetNames.Add($mazeMapTestHostProject.Name)
        if ($requireSimulationBinary) {
            $hostProjectTargetNames.Add($mazeSimulationHostProject.Name)
        }

        Write-LogLine ("Host project targets: {0}" -f ($hostProjectTargetNames -join ', ')) 'DarkCyan'
        $hostBuildStopwatch = [System.Diagnostics.Stopwatch]::StartNew()
        Invoke-HostMsBuild `
            -VsDevCmd $vsDevCmd `
            -SolutionPath $solutionPath `
            -ProjectTargetNames $hostProjectTargetNames.ToArray() `
            -HostBuildTarget $HostBuildTarget `
            -HostLtcgMode $HostLtcgMode
        $hostBuildStopwatch.Stop()
        Write-LogLine ("Host build completed in {0:n1}s" -f $hostBuildStopwatch.Elapsed.TotalSeconds) 'DarkCyan'
    }

    Write-Step 'Checking latest release artifacts'
    $releaseArtifacts = Get-AndLogReleaseArtifactStatus `
        -ArtifactVerb $artifactVerb `
        -FirmwareOutputDir $firmwareOutputDir `
        -HexPath $hexPath `
        -RepoRoot $repoRoot `
        -CanonicalBuildPath $canonicalBuildPath `
        -SketchDir $sketchDir `
        -ArduinoEigenLibraryDir $arduinoEigenLibraryDir `
        -EigenSourceRoot $eigenIncludeDir `
        -ArduinoStubHeaderPath $arduinoStubHeaderPath `
        -MazeMapProject $mazeMapHostProject `
        -MazeMapTestProject $mazeMapTestHostProject `
        -MazeSimulationProject $mazeSimulationHostProject `
        -MazeMapDllPath $mazeMapDllPath `
        -TestDllPath $testDllPath `
        -SimulationExePath $simulationExePath `
        -RequireFirmwareImage $requireFirmwareImage `
        -RequireSimulationBinary $requireSimulationBinary

    if ($requiresTests) {
        if ($releaseArtifacts.HostTestReady) {
            Write-Step 'Running the Release unit tests'
            $testStopwatch = [System.Diagnostics.Stopwatch]::StartNew()
            Invoke-External -FilePath $vstest -Arguments @($releaseArtifacts.TestDll.FullName) -FailureConsoleOutputMode 'VSTestFailuresOnly'
            $testStopwatch.Stop()
            Write-LogLine ("Release tests completed in {0:n1}s" -f $testStopwatch.Elapsed.TotalSeconds) 'DarkCyan'

            if (-not $releaseArtifacts.RequiredReady) {
                Write-LogLine 'Host Release tests completed even though blocking verification issues remain above.' 'Yellow'
            }
            elseif ($releaseArtifacts.AdvisoryFailureMessages.Count -gt 0) {
                Write-LogLine 'Host Release tests completed even though non-blocking verification issues remain above.' 'Yellow'
            }
        }
        else {
            Write-Step 'Skipping the Release unit tests'
            Write-LogLine 'Host test artifacts are not current, so the host tests cannot run.' 'Yellow'
        }
    }

    if (-not $releaseArtifacts.RequiredReady) {
        throw ($releaseArtifacts.BlockingFailureMessages -join ' ')
    }

    Write-Step $finalStepLabel
    if ($releaseArtifacts.TeensyReady -and $null -ne $releaseArtifacts.FirmwareImage) {
        Write-LogLine ("Latest firmware image: {0}" -f $releaseArtifacts.FirmwareImage.FullName) 'Green'
    }
    Write-LogLine ("Release test binary: {0}" -f $releaseArtifacts.TestDll.FullName) 'Green'
    if ($releaseArtifacts.AdvisoryFailureMessages.Count -gt 0) {
        Write-LogLine 'Advisory verification issues remain above but do not block this mode.' 'Yellow'
    }
    Write-LogLine ("{0}: {1}" -f $logPathLabel, $LogFilePath) 'Green'
}
catch {
    $scriptExitCode = 1
    $errorMessage = ("ERROR: {0}" -f $_.Exception.Message)
    if ($script:SuppressTerminalErrorSummary) {
        Add-Content -LiteralPath $LogFilePath -Value $errorMessage -Encoding UTF8
    }
    else {
        Write-LogLine $errorMessage 'Red'
    }
}
finally {
    Add-Content -LiteralPath $LogFilePath -Value '' -Encoding UTF8
    Add-Content -LiteralPath $LogFilePath -Value ('End time: ' + (Get-Date -Format 'yyyy-MM-dd HH:mm:ss.fff zzz')) -Encoding UTF8
    Pop-Location
}

if ($scriptExitCode -ne 0) {
    exit $scriptExitCode
}
