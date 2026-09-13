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
curl -fsSL https://mirrors.tuna.tsinghua.edu.cn/ros2/ubuntu/ros.asc \
  -o /usr/share/keyrings/ros-archive-keyring.gpg
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
  ros-humble-ackermann-steering-controllers

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
export ROS_DOMAIN_ID=42
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:~/hwcart_sim/models
EOF

echo "================ 完成 ================"
echo "验证: source ~/.bashrc && ros2 topic list / gazebo / rviz2"
echo "再运行本仓库 tools/check_environment.py 做完整检查"
