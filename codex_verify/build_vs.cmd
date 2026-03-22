@echo off
call "C:\Program Files\Microsoft Visual Studio\2022\Community\Common7\Tools\VsDevCmd.bat" -no_logo
msbuild MazeMap\MazeMap\MazeMap.vcxproj /t:Build /p:Configuration=Debug /p:Platform=x64 /m
