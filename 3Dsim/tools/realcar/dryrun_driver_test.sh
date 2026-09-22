#!/bin/bash
# 无动力联调 wrapper:source ROS 后调 dryrun_driver_test.py(2026-09-16 控制链审查轮·任务三)
# 前提:电调开关物理断开;本脚本不向 /dev/car 发送任何字节(虚拟串口隔离)。
set -u
[ -f /opt/ros/humble/setup.bash ] && { set +u; source /opt/ros/humble/setup.bash; set -u; }
[ -f "$HOME/racecar/install/setup.bash" ] && { set +u; source "$HOME/racecar/install/setup.bash"; set -u; }
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
exec python3 "$SCRIPT_DIR/dryrun_driver_test.py" "${1:-$HOME/dryrun_$(date +%Y%m%d_%H%M%S)}"
