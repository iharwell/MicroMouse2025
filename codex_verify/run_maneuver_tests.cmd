@echo off
call "C:\Program Files\Microsoft Visual Studio\18\Community\Common7\Tools\VsDevCmd.bat" -no_logo
vstest.console.exe MazeMap\MazeMapTest\x64\Debug\MazeMapTest.dll /Logger:trx
