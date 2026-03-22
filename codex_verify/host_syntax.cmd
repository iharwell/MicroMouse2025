@echo off
call "C:\Program Files\Microsoft Visual Studio\2022\Community\Common7\Tools\VsDevCmd.bat" -no_logo
cl /nologo /std:c++20 /EHsc /c codex_verify\defines_host_syntax.cpp /I MazeMap\MazeMap
