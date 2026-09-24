#!/bin/bash
# 实车传感器采集(2026-09-16 任务书:实车接入第一步·任务4)
#
# 用途:按厂家说明启动传感器(car.sh)后,采集实际话题/类型/frame_id/频率/TF,
#       并取得完整 LaserScan 角度与时间字段(仿真阶段遗留对账项)。
# 前提(安全边界,缺一不跑):
#   1. 电调开关【断开】——电机无动力(厂家手册实验八同款做法:关电调开导航取点);
#      racecar_driver 虽会随 car.sh 启动,但无人发 /car_cmd_vel,且电机无动力。
#   2. 车辆架起或轮子离地悬空更稳(防任何意外转动)。
#   3. 本脚本不发任何运动指令、不跑 teleop、不做 PWM 扫描。
# 用法:
#   终端A: cd ~/racecar && bash car.sh        (按厂家手册实验三)
#   终端B: bash capture_sensors.sh            (本脚本,跑 1-2 分钟)
set -u
[ -f /opt/ros/humble/setup.bash ] && { set +u; source /opt/ros/humble/setup.bash; set -u; }

OUT="${1:-$HOME/sensors_$(date +%Y%m%d_%H%M%S)}"
mkdir -p "$OUT"
echo "采集输出目录: $OUT"
echo "安全自检:电调开关已断开?(回车继续 / Ctrl+C 取消)"
read -r _

# ---- 1. 话题与类型总表 ----
ros2 topic list -t > "$OUT/00_topics.txt" 2>&1
ros2 node list > "$OUT/00_nodes.txt" 2>&1

# ---- 2. LaserScan 完整字段(单帧,重点:angle_min/max/increment/scan_time/time_increment) ----
# Humble 的 echo --once 在数据流上可用;超时 20s 防挂起
timeout 20 ros2 topic echo /scan --once > "$OUT/01_scan_full.txt" 2>&1
timeout 20 ros2 topic echo /scan --field header --once > "$OUT/02_scan_header.txt" 2>&1

# ---- 3. 其余关键传感器单帧 ----
timeout 20 ros2 topic echo /IMU_data --once > "$OUT/03_imu_once.txt" 2>&1
timeout 20 ros2 topic echo /encoder --once > "$OUT/04_encoder_once.txt" 2>&1
timeout 20 ros2 topic echo /odom_combined --once > "$OUT/05_odom_combined_once.txt" 2>&1

# ---- 4. 频率采样(静止状态;hz 自带统计,各采 12s/3s) ----
timeout 15 ros2 topic hz /scan > "$OUT/06_hz_scan.txt" 2>&1
timeout 6  ros2 topic hz /IMU_data > "$OUT/07_hz_imu.txt" 2>&1
timeout 6  ros2 topic hz /encoder > "$OUT/08_hz_encoder.txt" 2>&1

# ---- 5. TF 结构(静止;view_frames 生成 frames.pdf)+ 静态 TF 原文 ----
cd "$OUT" && timeout 30 ros2 run tf2_tools view_frames > /dev/null 2>&1; cd - > /dev/null
timeout 15 ros2 topic echo /tf_static > "$OUT/09_tf_static.txt" 2>&1 & TF_PID=$!
sleep 8; kill $TF_PID 2>/dev/null

# ---- 6. 汇总关键数(便于直接回填对账表) ----
{
    echo "== /scan 关键字段(从 01 提取) =="
    grep -E "angle_min|angle_max|angle_increment|scan_time|time_increment|range_min|range_max|frame_id" "$OUT/01_scan_full.txt" | head -12
    echo "== ranges 长度 =="
    grep -o "array('f', \[[0-9eE+.,inf -]*\]" "$OUT/01_scan_full.txt" | head -1 | tr ',' '\n' | wc -l
    echo "== 频率(hz 输出末行) =="
    tail -2 "$OUT/06_hz_scan.txt"; tail -2 "$OUT/07_hz_imu.txt"; tail -2 "$OUT/08_hz_encoder.txt"
} > "$OUT/10_summary.txt"

echo "完成: $(ls "$OUT")"
echo "重点看 10_summary.txt —— LaserScan 角度/时间字段与频率即仿真对齐缺的三项。"
echo "下一步:整个目录拖回电脑,回填 docs/reference/测量清单.md 与 SIM_INTEROP.md 对账。"
