@echo off
setlocal

set "SCRIPT_DIR=%~dp0"
for %%I in ("%SCRIPT_DIR%..") do set "REPO_ROOT=%%~fI"
set "MAZEMAP_INCLUDE=%REPO_ROOT%\MazeMap\MazeMap"
set "EIGEN_INCLUDE=%REPO_ROOT%\MazeMap\eigen-5.0.0"

call "C:\Program Files\Microsoft Visual Studio\18\Community\Common7\Tools\VsDevCmd.bat" -no_logo
cl /nologo /std:c++20 /EHsc /c "%SCRIPT_DIR%sqrt_host_vector.cpp" /I "%MAZEMAP_INCLUDE%" /I "%EIGEN_INCLUDE%"
