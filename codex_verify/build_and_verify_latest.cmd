@echo off
setlocal

set "SCRIPT_DIR=%~dp0"
set "POWERSHELL=C:\Windows\System32\WindowsPowerShell\v1.0\powershell.exe"
set "PAUSE_ON_EXIT=1"
set "PS_ARGS=%*"
set "MM_BUILD_AND_VERIFY_LAUNCHED_FROM_CMD=1"

if /I "%~1"=="--no-pause" (
    set "PAUSE_ON_EXIT=0"
    shift
    set "PS_ARGS=%*"
)

"%POWERSHELL%" -NoLogo -NoProfile -ExecutionPolicy Bypass -File "%SCRIPT_DIR%build_and_verify_latest.ps1" %PS_ARGS%
set "EXIT_CODE=%ERRORLEVEL%"

echo.
if "%PAUSE_ON_EXIT%"=="1" pause

exit /b %EXIT_CODE%
