#!/usr/bin/env bash
# =============================================================================
# 重建 racecar_sim 包(WSL 内执行: bash ~/hwcart_sim/tools/rebuild_racecar.sh)
# 先清 build/install 下的旧产物再构建:setuptools 复制 data_files 是覆盖式,
# 源里已删除的文件(如静态 ros2_control.yaml)会在 install 残留误导排查
# =============================================================================
set -e
cd ~/hwcart_sim/hwcart_sim
rm -rf build/racecar_sim install/racecar_sim
source /opt/ros/humble/setup.bash
colcon build --packages-select racecar_sim
echo "BUILD_DONE"
