@echo off
rem ASCII only: see sim_start.bat for why.
rem
rem Teleport the car back to the start and re-enable the policy. Takes seconds,
rem does not restart the sim. Do NOT use Gazebo's Reset World (Ctrl+R) instead:
rem that rewinds sim time and permanently breaks gazebo_ros2_control.
title Reset run
rem ROS_DOMAIN_ID must match run_sim.sh (43), or this talks to an empty graph
rem and silently teleports nothing.
for /f "usebackq delims=" %%p in (`wsl -d Ubuntu-22.04 -e wslpath -a "%~dp0"`) do set "DIR=%%p"
wsl -d Ubuntu-22.04 -e bash -lc "source '%DIR%/tools/_env.sh' && python3 \"$AUTOCAR/teleop/sim_reset.py\" %*"
timeout /t 3 >nul
