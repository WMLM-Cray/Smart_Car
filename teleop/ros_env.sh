# 实车 ROS 环境。~/.bashrc 第一行 source 它 —— 放在 bashrc 的"非交互就 return"之前,
# 这样 `ssh orange-pi@车 'python3 ~/autocar/xxx.py'` 这种一行命令也有 ROS 环境。
# 原来这几行在 bashrc 末尾,ssh 一行命令根本走不到,于是 No module named 'rclpy'、
# 域号是空的(看不到任何话题)—— 2026-09-22 反复踩。
# 必须静默:非交互 shell 里有任何输出都会把 scp 弄坏。
source /opt/ros/humble/setup.bash
source /home/orange-pi/racecar/install/setup.bash
export ROS_DOMAIN_ID=42
