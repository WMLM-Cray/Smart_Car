#!/bin/bash
# 实车只读盘点(2026-09-16 任务书:实车接入第一步·任务2)
#
# 用途:在车载电脑(orangepiaipro / davinci-mini)上运行,采集系统版本/CPU 架构/
#       ROS2 版本/厂家工作区/启动服务/设备接口,全部只读。
# 安全:不发运动指令、不跑 teleop、不动 PWM、不改任何文件、不安装任何东西;
#       不读取不输出任何密码/凭据文件(ssh key、shadow、wifi 配置一律不碰)。
# 输出:OUT 目录(默认 ~/probe_日期时间/)下一组 .txt,采完用 MobaXterm 下载回电脑。
set -u

OUT="${1:-$HOME/probe_$(date +%Y%m%d_%H%M%S)}"
mkdir -p "$OUT"
run() {  # run <文件名> <命令...>:命令失败不中断,错误也记录
    local name="$1"; shift
    { echo "\$ $*"; "$@" 2>&1; echo "[rc=$?]"; } > "$OUT/$name.txt" || true
}

echo "盘点输出目录: $OUT"

# ---- 1. 系统与硬件(只读) ----
run 01_os_release      cat /etc/os-release
run 02_kernel_arch     uname -a
run 03_cpu             lscpu
run 04_memory          free -h
run 05_disk            df -h / /home
run 06_npu             npu-smi info          # 昇腾 NPU;无此命令则记录 rc≠0
run 04b_board          cat /proc/device-tree/model 2>/dev/null || true

# ---- 2. ROS2 环境 ----
run 10_ros_dists       ls /opt/ros/ 2>/dev/null
if [ -f /opt/ros/humble/setup.bash ]; then
    set +u; source /opt/ros/humble/setup.bash; set -u
fi
run 11_ros_version     env | grep -E "^ROS_|^RMW_" | sort   # 只列变量名与值,无凭据
run 12_ros_pkg_count   bash -c 'ros2 pkg list 2>/dev/null | wc -l'

# ---- 3. 厂家工作区(只列不读源码内容) ----
run 20_home_listing    ls -la "$HOME"
run 21_racecar_top     ls -la "$HOME/racecar" 2>/dev/null
run 22_racecar_sh      head -50 "$HOME"/racecar/*.sh 2>/dev/null
run 23_src_packages    ls "$HOME/racecar/src/" 2>/dev/null
run 24_racecar_du      du -sh "$HOME/racecar" 2>/dev/null
# install/ 时间戳=最近是否重新编过;只看时间不读内容
run 25_install_stamps  ls -la --time-style=long-iso "$HOME/racecar/install/" 2>/dev/null

# ---- 4. 启动服务/自启动项(只列名单与状态) ----
run 30_services_sys    systemctl list-units --type=service --state=running --no-pager
run 31_services_ena    systemctl list-unit-files --type=service --state=enabled --no-pager
run 32_services_user   systemctl --user list-units --type=service --state=running --no-pager 2>/dev/null
run 33_crontab_root    ls -la /etc/cron.d/ 2>/dev/null
run 34_rc_local        cat /etc/rc.local 2>/dev/null
run 35_udev_rules      ls -la /etc/udev/rules.d/

# ---- 5. 设备接口(只列;不 open 串口) ----
run 40_tty_devices     ls -l /dev/ttyUSB* /dev/ttyACM* 2>/dev/null
run 41_dev_symlinks    ls -l /dev/car /dev/imu /dev/encoder /dev/laser 2>/dev/null
run 42_lsusb           lsusb
run 43_net_interfaces  ip -brief addr
run 44_hostname        hostname

# ---- 6. 环境快照(为将来复现装机;只有包名清单) ----
run 50_dpkg_sel        dpkg --get-selections
run 51_pip_list        bash -c 'pip3 list 2>/dev/null || pip list 2>/dev/null'

echo "完成。共 $(ls "$OUT" | wc -l) 个文件,全部只读采集。"
echo "下一步:用 MobaXterm(SFTP)把 $OUT 整个目录拖回电脑,交给验收。"
