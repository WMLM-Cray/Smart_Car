@echo off
rem ASCII only on purpose: cmd parses .bat as the OEM codepage, so non-ASCII
rem bytes here get mangled and split the commands. Chinese lives in run_sim.sh.
rem
rem Double-click to restart the whole sim: kills leftovers, brings up Gazebo +
rem car + track, starts the policy and enables it. Close this window to stop.
title Gazebo sim
for /f "usebackq delims=" %%p in (`wsl -d Ubuntu-22.04 -e wslpath -a "%~dp0run_sim.sh"`) do set "SH=%%p"
wsl -d Ubuntu-22.04 -e bash -lc "bash '%SH%'"
echo.
echo Simulation exited.
pause
