#!/usr/bin/env bash
# =============================================================================
# 重建 racecar_sim 包(WSL 内执行: bash <仓库>/3Dsim/tools/rebuild_racecar.sh)
# 先清 build/install 下的旧产物再构建:setuptools 复制 data_files 是覆盖式,
# 源里已删除的文件(如静态 ros2_control.yaml)会在 install 残留误导排查
# =============================================================================
set -e
# 工作区位置两边不一样(见 _env.sh 的注释),这里不能 source install/setup.bash
# —— 还没 build 出来。所以只解析路径,不 source。
REPO=$(cd "$(dirname "$0")/../.." && pwd)
for c in "$REPO/3Dsim/hwcart_sim" "$HOME/hwcart_sim/hwcart_sim" "$HOME/hw_ws"; do
    [ -d "$c/src" ] && WS=$c && break
done
[ -n "${WS:-}" ] || { echo "找不到 colcon 工作区,用 WS=<路径> 指定" >&2; exit 1; }
cd "$WS"
rm -rf build/racecar_sim install/racecar_sim
source /opt/ros/humble/setup.bash
colcon build --packages-select racecar_sim
echo "BUILD_DONE"
