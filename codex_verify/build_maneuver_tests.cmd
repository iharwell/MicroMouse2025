@echo off
call "C:\Program Files\Microsoft Visual Studio\18\Community\Common7\Tools\VsDevCmd.bat" -no_logo
msbuild MazeMap\MazeMapTest\MazeMapTest.vcxproj /t:Build /p:Configuration=Debug /p:Platform=x64 /m /clp:ErrorsOnly;Summary /fl /flp:logfile=codex_verify\maneuver_test_build.log;verbosity=normal
