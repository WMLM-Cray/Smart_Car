#!/usr/bin/env bash
# =============================================================================
# WSL2 Ubuntu 22.04 → ROS2 Humble + Gazebo Classic 11 + Nav2 仿真环境一键安装
# 用途:07-仿真系统 阶段0 环境搭建(国内镜像源加速)
# 用法(在 WSL 内以 root 运行): sudo bash wsl_setup_ros2.sh 2>&1 | tee setup.log
# 幂等:重复运行安全(apt/换源均为覆盖式)
# =============================================================================
set -euo pipefail

echo "==> [1/6] 换 Ubuntu apt 国内源(中科大)"
sed -i 's|http://archive.ubuntu.com|https://mirrors.ustc.edu.cn|g; s|http://security.ubuntu.com|https://mirrors.ustc.edu.cn|g' /etc/apt/sources.list
apt-get update

echo "==> [2/6] 基础工具"
apt-get install -y curl gnupg2 lsb-release git cmake build-essential wget \
  python3-pip software-properties-common

echo "==> [3/6] 配置 ROS2 清华源(keyrings 方式)"
install -d /usr/share/keyrings
KEYRING=/usr/share/keyrings/ros-archive-keyring.gpg
FPR=C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
# 2026-09-21 重装时踩的坑,旧写法三处都错,别改回去:
#  ① 抓的是 .../ros2/ubuntu/ros.asc —— 清华和中科大现在都 404,只有 GitHub 的
#     rosdistro/master/ros.key 还在(而且是二进制,不用 dearmor);
#  ② `curl | gpg --dearmor` 里 curl 的失败被管道吞掉,gpg 照样生成一个空文件;
#  ③ `gpg --list-keys` 对空 keyring 返回成功,所以守卫判定"密钥有效"直接放行。
# 结果是 apt update 报 NO_PUBKEY F42ED6FBAB17C654、ROS 源整个没进索引,
# 而 apt 的报错长得像包名写错(Unable to locate package ros-humble-xxx),极误导。
# 所以下面一律**按指纹**验收,不看退出码。
have_key() { gpg --no-default-keyring --keyring "$KEYRING" --fingerprint 2>/dev/null \
             | tr -d ' ' | grep -q "$FPR"; }
if ! have_key; then
  for url in https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
             https://cdn.jsdelivr.net/gh/ros/rosdistro@master/ros.key; do
    echo "  取密钥:$url"
    curl -fsSL --max-time 20 -o /tmp/ros.key "$url" || continue
    # 二进制直接用,ASCII armor 才 dearmor
    if head -c 5 /tmp/ros.key | grep -q -- '-----'; then
      gpg --dearmor -o "$KEYRING" < /tmp/ros.key
    else
      cp /tmp/ros.key "$KEYRING"
    fi
    have_key && break
  done
fi
if ! have_key; then
  echo "  网络取不到,退回 keyserver"
  rm -f "$KEYRING"
  gpg --no-default-keyring --keyring "$KEYRING" \
    --keyserver hkp://keyserver.ubuntu.com:80 --recv-keys F42ED6FBAB17C654
fi
have_key || { echo "密钥仍然不对,装不下去了(指纹应为 $FPR)"; exit 1; }
chmod 644 "$KEYRING"

# 镜像域名如果解析到 198.18.0.x,说明本机走代理的 fake-ip;apt 并发下载会随机
# 掉连接(Unable to connect to mirrors.ustc.edu.cn:https)。单连接 + 重试能过。
cat > /etc/apt/apt.conf.d/99-proxy-friendly <<'CONF'
Acquire::Retries "8";
Acquire::http::Pipeline-Depth "0";
Acquire::Queue-Host-Limit "1";
CONF
echo "deb [arch=amd64 signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] https://mirrors.tuna.tsinghua.edu.cn/ros2/ubuntu jammy main" \
  > /etc/apt/sources.list.d/ros2-latest.list
apt-get update

echo "==> [4/6] 安装 ROS2 Humble + 仿真全家桶(约 2-4GB,耐心等)"
DEBIAN_FRONTEND=noninteractive apt-get install -y \
  ros-humble-desktop \
  ros-humble-gazebo-ros-pkgs ros-humble-xacro \
  ros-humble-joint-state-publisher-gui \
  ros-humble-navigation2 ros-humble-nav2-bringup \
  ros-humble-slam-toolbox \
  ros-humble-robot-localization \
  ros-humble-cv-bridge python3-colcon-common-extensions \
  ros-humble-ros2-control ros-humble-ros2-controllers \
  ros-humble-tricycle-controller ros-humble-tricycle-steering-controller

echo "==> [5/6] rosdep(官方失败则退回 rosdepc 国内版)"
if ! rosdep init 2>/dev/null; then
  echo "rosdep init 失败(常见于网络),改装 rosdepc"
  pip3 install -i https://pypi.tuna.tsinghua.edu.cn/simple rosdepc
  rosdepc init
fi
su - "${SUDO_USER:-lzx}" -c 'rosdep update 2>/dev/null || rosdepc update'

echo "==> [6/6] 写入 ~/.bashrc 环境注入"
BRC="/home/${SUDO_USER:-lzx}/.bashrc"
grep -q 'ros2_setup' "$BRC" || cat >> "$BRC" <<'EOF'

# --- ros2_setup: 仿真工作区环境 ---
source /opt/ros/humble/setup.bash
# 43 = 仿真域。**别写 42** —— 那是实车的域,同网段下 DDS 会把两边的话题按名字
# 合并,/scan 凭空多一个发布者,不报错不告警(2026-09-20 踩过,见 SIM_CAR_MODELS.md)。
export ROS_DOMAIN_ID=43
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:~/hwcart_sim/models
EOF

echo "================ 完成 ================"
echo "验证: source ~/.bashrc && ros2 topic list / gazebo / rviz2"
echo "再运行本仓库 tools/check_environment.py 做完整检查"
