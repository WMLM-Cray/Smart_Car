# 仿真脚本共用的环境解析。用法:在脚本开头 `source "$(dirname "$0")/_env.sh"`
# (run_sim.sh 在 3Dsim/ 下,路径是 "$(dirname "$0")/tools/_env.sh")
#
# 解决两件事,都是 2026-09-22 队友反馈踩到的:
#
# 1. **工作区路径两边不一样,互不存在。**
#    队友按 README 把仓库的 3Dsim/ 软链成 ~/hwcart_sim,所以他的工作区在
#    ~/hwcart_sim/hwcart_sim/install;而重建 WSL 时我建在 ~/hw_ws
#    (colcon 在 /mnt/c 的 9p 上编译慢一个数量级,没照 README 软链)。
#    结果 tools/*.sh 在我机器上跑不了,run_sim.sh 在他机器上跑不了。
#    这里按顺序找,找到哪个用哪个,并允许用 WS= 覆盖。
#
# 2. **域写死成 42。** 42 是**实车**的域。同网段下 DDS 会把仿真和实车的话题
#    按名字合并 —— /scan 凭空多一个发布者,不报错不告警,只能靠数点数发现
#    (实车 1667 点 12Hz,Gazebo 2000 点 8Hz)。仿真一律用 43。
#    写成 ${ROS_DOMAIN_ID:-43} 而不是写死,是为了调试时能临时指定。

export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-43}

# 仓库根:从 _env.sh 自身位置推,不依赖调用者的 cwd
_ENV_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
REPO=${REPO:-$(cd "$_ENV_DIR/../.." && pwd)}

# colcon 工作区(含 install/setup.bash 的那一级)
if [ -z "${WS:-}" ]; then
  for _c in "$REPO/3Dsim/hwcart_sim" "$HOME/hwcart_sim/hwcart_sim" "$HOME/hw_ws"; do
    [ -f "$_c/install/setup.bash" ] && WS=$_c && break
  done
fi
# 策略和 teleop 脚本所在目录
if [ -z "${AUTOCAR:-}" ]; then
  for _c in "$REPO" "$HOME/autocar"; do
    [ -f "$_c/teleop/steer_policy_node.py" ] && AUTOCAR=$_c && break
  done
fi
export REPO WS AUTOCAR

source /opt/ros/humble/setup.bash
# WS 是自己找到的还是调用者给的都要验一遍 —— 给错路径时要报得像话,
# 不能甩一行 "No such file or directory" 出去。
if [ -z "${WS:-}" ] || [ ! -f "$WS/install/setup.bash" ]; then
  echo "找不到 colcon 工作区${WS:+($WS/install/setup.bash 不存在)}。" >&2
  echo "找过:\$REPO/3Dsim/hwcart_sim、~/hwcart_sim/hwcart_sim、~/hw_ws" >&2
  echo "先 colcon build,或者用 WS=<含 install/ 的目录> 指定。" >&2
  return 1 2>/dev/null || exit 1
fi
source "$WS/install/setup.bash"
