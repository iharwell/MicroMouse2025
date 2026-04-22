@echo off
setlocal

set "SCRIPT_DIR=%~dp0"
for %%I in ("%SCRIPT_DIR%..") do set "REPO_ROOT=%%~fI"
set "MAZEMAP_INCLUDE=%REPO_ROOT%\MazeMap\MazeMap"
set "EIGEN_INCLUDE=%REPO_ROOT%\MazeMap\eigen-5.0.0"
set "MAZEMAP_DEBUG_DIR=%REPO_ROOT%\MazeMap\MazeMap\x64\Debug"
set "MANEUVER_DIAG_EXE=%SCRIPT_DIR%maneuver_diag.exe"
set "MANEUVER_DIAG_SOURCE=%SCRIPT_DIR%maneuver_diag.cpp"
set "MAZEMAP_DLL_SOURCE=%MAZEMAP_DEBUG_DIR%\MazeMap.dll"
set "MAZEMAP_DLL_TARGET=%SCRIPT_DIR%MazeMap.dll"

call "C:\Program Files\Microsoft Visual Studio\18\Community\Common7\Tools\VsDevCmd.bat" -no_logo
cl /nologo /EHsc /std:c++20 /I "%MAZEMAP_INCLUDE%" /I "%EIGEN_INCLUDE%" /Fe:"%MANEUVER_DIAG_EXE%" "%MANEUVER_DIAG_SOURCE%" /link /LIBPATH:"%MAZEMAP_DEBUG_DIR%" MazeMap.lib
if errorlevel 1 exit /b %errorlevel%
copy /Y "%MAZEMAP_DLL_SOURCE%" "%MAZEMAP_DLL_TARGET%" >nul
"%MANEUVER_DIAG_EXE%"
