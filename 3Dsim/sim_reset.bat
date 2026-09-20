@echo off
rem ASCII only: see sim_start.bat for why.
rem
rem Teleport the car back to the start and re-enable the policy. Takes seconds,
rem does not restart the sim. Do NOT use Gazebo's Reset World (Ctrl+R) instead:
rem that rewinds sim time and permanently breaks gazebo_ros2_control.
title Reset run
wsl -d Ubuntu-22.04 -e bash -lc "source /opt/ros/humble/setup.bash && cd ~/autocar && python3 teleop/sim_reset.py"
timeout /t 3 >nul
