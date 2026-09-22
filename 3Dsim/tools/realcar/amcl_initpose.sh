#!/bin/bash
# 发 AMCL 初始位姿 (阶段2, 任务书 2026-09-17 三: 不得默认建图起点/硬写0,0,0)
# 用法: ./amcl_initpose.sh <x> <y> <yaw_deg> [cov_xy_var] [cov_yaw_var]
#   x/y      : 车辆在保存地图(map 系)中的真实位置, 米
#   yaw_deg  : 真实朝向, 角度制(东=0, 逆时针为正)
#   cov_xy_var : 位置方差 m^2, 默认 0.09 (std=0.30m; 依据: 初值来自扫图结束位姿读数,
#                收链后可能存在小幅挪动, 留 0.3m 量级容错由激光匹配收敛)
#   cov_yaw_var: 朝向方差 rad^2, 默认 0.03 (std≈10°)
# 消息: geometry_msgs/PoseWithCovarianceStamped -> /initialpose, frame_id=map (必须)
# 协方差矩阵按 PoseWithCovariance 语义: [x,y,z,roll,pitch,yaw] 对角线
set -u
X="${1:?用法: $0 <x> <y> <yaw_deg> [cov_xy_var 默认0.09] [cov_yaw_var 默认0.03]}"
Y="${2:?}"
YAWD="${3:?}"
COVXY="${4:-0.09}"
COVYAW="${5:-0.03}"

set +u
source /opt/ros/humble/setup.bash
set -u

read -r QZ QW YAW <<<"$(python3 - "$YAWD" <<'PYEOF'
import sys, math
yaw = float(sys.argv[1]) * math.pi / 180.0
print(math.sin(yaw/2.0), math.cos(yaw/2.0), yaw)
PYEOF
)"
CX=$(python3 -c "print($COVXY)")
CY=$(python3 -c "print($COVYAW)")

echo "发 /initialpose: x=$X y=$Y yaw=${YAWD}°(rad $YAW) cov_xy=$CX cov_yaw=$CY frame=map"
timeout 15 ros2 topic pub -1 /initialpose geometry_msgs/msg/PoseWithCovarianceStamped "
header:
  stamp:
    sec: 0
    nanosec: 0
  frame_id: map
pose:
  pose:
    position: {x: $X, y: $Y, z: 0.0}
    orientation: {x: 0.0, y: 0.0, z: $QZ, w: $QW}
  covariance: [$CX, 0.0, 0.0, 0.0, 0.0, 0.0,
              0.0, $CX, 0.0, 0.0, 0.0, 0.0,
              0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
              0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
              0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
              0.0, 0.0, 0.0, 0.0, 0.0, $CY]
" 2>&1 | tail -3
echo "完成. 验证: ros2 topic echo /amcl_pose --once | head -20"
