@echo off
call "C:\Program Files\Microsoft Visual Studio\18\Community\Common7\Tools\VsDevCmd.bat" -no_logo
cl /nologo /EHsc /std:c++20 /I "C:\Users\thene\source\repos\MicroMouse2025\MazeMap\MazeMap" /Fe:"C:\Users\thene\source\repos\MicroMouse2025\codex_verify\maneuver_diag.exe" "C:\Users\thene\source\repos\MicroMouse2025\codex_verify\maneuver_diag.cpp" /link /LIBPATH:"C:\Users\thene\source\repos\MicroMouse2025\MazeMap\MazeMap\x64\Debug" MazeMap.lib
if errorlevel 1 exit /b %errorlevel%
copy /Y "C:\Users\thene\source\repos\MicroMouse2025\MazeMap\MazeMap\x64\Debug\MazeMap.dll" "C:\Users\thene\source\repos\MicroMouse2025\codex_verify\MazeMap.dll" >nul
"C:\Users\thene\source\repos\MicroMouse2025\codex_verify\maneuver_diag.exe"
