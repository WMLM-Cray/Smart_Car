#!/bin/bash
# 实车 AMCL 定位链 (华为小车 阶段2, 任务书 2026-09-17)
# 用法:
#   ./amcl_localize.sh start <地图yaml路径>   起链: 前置检查+静态地图检查 -> Run_car(传感器/EKF/driver)
#                                            -> 验证无旧 map->odom 发布者 -> map_server+amcl+lifecycle -> 渲染监控
#   ./amcl_localize.sh check                 只读核对: 运动话题零发布者/TF发布者唯一性/amcl_pose/生命周期状态
#   ./amcl_localize.sh stop                  按序收链: 定位组(先停 map->odom) -> Run_car -> monitor -> daemon
# 边界: 不起 slam_toolbox/planner/controller/bt_navigator/waypoint_cycle/手柄/任何测试速度发布器
# 前置: 电调断开 (全程无运动指令输出是本轮通过标准之一)
set -u

TEAM="$HOME/racecar_team"
HERE="$(cd "$(dirname "$0")" && pwd)"
AMCL_YAML="$HERE/amcl_realcar.yaml"
LAUNCH="$HERE/localization_realcar.launch.py"
MONITOR="$TEAM/tools/slam/map_monitor.py"
LOG_DIR="$TEAM/logs/amcl"
STATE="$TEAM/run/amcl_localize.state"
mkdir -p "$LOG_DIR" "$TEAM/run"

set +u   # setup.bash 引用未定义变量, set -u 下必炸(6bb97d8/deploy 同款坑)
source /opt/ros/humble/setup.bash
source "$HOME/racecar/install/setup.bash"
set -u

TS="$(date +%Y%m%d_%H%M%S)"

alive() { [ -n "${1:-}" ] && kill -0 "$1" 2>/dev/null; }

load_state() {
    if [ ! -f "$STATE" ]; then
        echo "错误: 无运行状态文件(链未起或已收). 先 ./amcl_localize.sh start <地图yaml>"
        return 1
    fi
    . "$STATE"   # RUN_CAR_PID / LOC_PID
}

kill_group() {  # kill_group <pid> <名字>
    local pid="${1:-}" name="$2"
    if alive "$pid"; then
        echo "停止 $name (PGID=$pid) ..."
        kill -TERM -- "-$pid" 2>/dev/null
        for _ in 1 2 3 4 5 6 7 8 9 10; do
            alive "$pid" || break
            sleep 1
        done
        if alive "$pid"; then
            echo "  TERM 未退, 升级 KILL: $name"
            kill -KILL -- "-$pid" 2>/dev/null
            sleep 1
        fi
    else
        echo "$name (PGID=$pid) 已不在"
    fi
}

wait_topic_pub() {  # wait_topic_pub <话题> <最多秒>
    local topic="$1" max="$2" i
    for i in $(seq 1 "$max"); do
        if timeout 6 ros2 topic info "$topic" 2>/dev/null | grep -qE 'Publisher count: [1-9]'; then
            return 0
        fi
        sleep 1
    done
    return 1
}

static_map_check() {  # static_map_check <地图yaml>
    local m="$1"
    echo "== 静态检查: 地图文件 =="
    if [ ! -f "$m" ]; then echo "错误: 地图 YAML 不存在: $m"; return 1; fi
    local img res origin neg
    img="$(grep -E '^image:' "$m" | awk '{print $2}')"
    res="$(grep -E '^resolution:' "$m" | awk '{print $2}')"
    origin="$(grep -E '^origin:' "$m")"
    echo "  yaml=$m"
    echo "  image=$img  resolution=$res  $origin"
    local dir pgm dims
    dir="$(dirname "$m")"
    pgm="$dir/$img"
    if [ ! -f "$pgm" ]; then echo "错误: PGM 不存在: $pgm (YAML 引用断裂)"; return 1; fi
    dims="$(head -n 3 "$pgm" | sed -n '2p' | tr -d '\r')"   # 只取头3行(P5/宽高/255), 不让二进制像素进管道
    echo "  pgm=$pgm 尺寸=${dims} (期望 83 72), 分辨率期望 0.05"
    [ "$dims" = "83 72" ] || { echo "错误: PGM 尺寸非 83x72"; return 1; }
    echo "$res" | grep -q '^0.05' || { echo "错误: resolution 非 0.05"; return 1; }
    echo "  静态检查 PASS"
}

no_stale_map_odom() {  # 起 AMCL 前确认无旧 map->odom_combined 发布者 (任务书二.4)
    echo "== 检查无旧 map->odom_combined 发布者 =="
    local out
    out="$(timeout 5 ros2 run tf2_ros tf2_echo map odom_combined 2>/dev/null | head -3)"
    if echo "$out" | grep -q Translation; then
        echo "错误: map->odom_combined 已可解析, 存在旧发布者(slam/旧amcl 残留). 先收链!"
        return 1
    fi
    echo "  无发布者 (tf2_echo 不可解析) PASS"
}

die_nostate() {  # die_nostate <退出码> <信息>
    # 超时/异常退出也落 state 文件, 保证 stop 能按序收链(2026-09-17 实跑发现:
    # exit 3 直接退 -> 无 state -> stop 报"链未起" -> 残留进程组裸奔)
    echo "$2"
    cat > "$STATE" <<EOF
RUN_CAR_PID=$RUN_CAR_PID
LOC_PID=${LOC_PID:-}
MAP_YAML=${map_yaml:-}
START_TS=$TS
EOF
    exit "$1"
}

cmd_start() {
    local map_yaml="${1:-}"
    [ -z "$map_yaml" ] && { echo "用法: $0 start <地图yaml路径>"; exit 1; }
    [ ! -f "$map_yaml" ] && { echo "错误: $map_yaml 不存在"; exit 1; }
    [ -f "$AMCL_YAML" ] || { echo "错误: 缺 $AMCL_YAML"; exit 1; }
    [ -f "$LAUNCH" ]   || { echo "错误: 缺 $LAUNCH"; exit 1; }
    if alive "${RUN_CAR_PID:-}" || alive "${LOC_PID:-}"; then
        echo "错误: 已有链在跑. 先 ./amcl_localize.sh stop"; exit 1
    fi
    if pgrep -f 'online_async' >/dev/null 2>&1; then
        echo "错误: 检测到 slam_toolbox 在跑, 定位与建图不能并存"; exit 1
    fi

    echo "== 前置检查: 串口设备 =="
    local dev miss=0
    for dev in /dev/car /dev/imu /dev/encoder /dev/laser; do
        if [ -e "$dev" ]; then echo "  OK  $dev"; else echo "  缺  $dev"; miss=1; fi
    done
    [ "$miss" -eq 1 ] && { echo "串口别名缺失. 修复: sudo systemctl restart systemd-udevd 后重试"; exit 2; }

    static_map_check "$map_yaml" || exit 2

    ros2 daemon stop >/dev/null 2>&1   # 清坏态 daemon(已知坑), 首条命令自动重建

    echo "== 起 Run_car (传感器+EKF+driver, 电调断开) =="
    setsid ros2 launch racecar Run_car.launch.py > "$LOG_DIR/runcar_$TS.log" 2>&1 &
    RUN_CAR_PID=$!
    echo "  PGID=$RUN_CAR_PID 日志=$LOG_DIR/runcar_$TS.log"

    echo "== 等待 /scan 有发布者 (最多90s, DDS 发现~4s) =="
    wait_topic_pub /scan 90 || die_nostate 3 "超时: /scan 无发布者"
    echo "  /scan 就绪"

    echo "== 等待 EKF TF odom_combined->base_footprint =="
    local i
    for i in $(seq 1 30); do
        if timeout 4 ros2 run tf2_ros tf2_echo odom_combined base_footprint 2>/dev/null | grep -m1 -q Translation; then
            echo "  TF 就绪 (第 ${i} 次探测)"; break
        fi
        sleep 2
        [ "$i" -eq 30 ] && die_nostate 3 "超时: EKF TF 未出现"
    done

    no_stale_map_odom || die_nostate 5 "存在旧 map->odom_combined 发布者, 拒绝起 AMCL"

    echo "== 起定位三件: map_server + amcl + lifecycle_manager =="
    setsid ros2 launch "$LAUNCH" \
        map:="$map_yaml" amcl_params:="$AMCL_YAML" \
        > "$LOG_DIR/loc_$TS.log" 2>&1 &
    LOC_PID=$!
    echo "  PGID=$LOC_PID 日志=$LOG_DIR/loc_$TS.log"

    echo "== 等待 amcl 进入 active =="
    for i in $(seq 1 45); do
        if [ "$(timeout 6 ros2 lifecycle get /amcl 2>/dev/null | grep -oE 'active|inactive|unconfigured' | head -1)" = "active" ]; then
            echo "  amcl active (第 ${i} 次探测)"; break
        fi
        sleep 2
        [ "$i" -eq 45 ] && die_nostate 3 "超时: amcl 未 active. 查日志: tail -40 $LOG_DIR/loc_$TS.log"
    done

    echo "== 起渲染监控 (Map+LaserScan 叠加, 复用阶段1 map_monitor) =="
    [ -f "$MONITOR" ] || echo "  警告: 缺 $MONITOR, 跳过渲染监控"
    if [ -f "$MONITOR" ]; then
        setsid nohup python3 "$MONITOR" > "$LOG_DIR/monitor_$TS.log" 2>&1 < /dev/null &
        echo "  monitor PID=$!"
    fi

    cat > "$STATE" <<EOF
RUN_CAR_PID=$RUN_CAR_PID
LOC_PID=$LOC_PID
MAP_YAML=$map_yaml
START_TS=$TS
EOF
    echo "== 起链完成 =="
    echo "   下一步: ./amcl_initpose.sh <x> <y> <yaw_deg>   (车辆在保存地图中的真实位姿, 勿默认 0,0,0)"
    echo "   验证:    ./amcl_localize.sh check ; 采样: python3 $HERE/amcl_watch.py --duration 35"
}

cmd_check() {
    local out="$LOG_DIR/check_$TS.txt"
    {
        echo "# AMCL 定位链只读核对 $TS (车机钟, 相对刻度)"
        echo "== 生命周期状态 =="
        echo -n "  map_server: "; timeout 6 ros2 lifecycle get /map_server 2>/dev/null || echo "N/A"
        echo -n "  amcl:       "; timeout 6 ros2 lifecycle get /amcl 2>/dev/null || echo "N/A"
        echo "== 运动话题发布者(应全为 0 或不存在, 全程无运动指令输出) =="
        for t in /cmd_vel /car_cmd_vel /teleop_cmd_vel /teleop/cmd_vel; do
            echo "-- $t"
            timeout 10 ros2 topic info "$t" 2>/dev/null | grep -E 'count' || echo "  话题不存在"
        done
        echo "== 禁用组件确认不存在 =="
        for pat in 'planner_server' 'controller_server' 'bt_navigator' 'waypoint' 'online_async' 'slam_toolbox'; do
            if pgrep -f "$pat" >/dev/null 2>&1; then echo "  存在(违规): $pat"; else echo "  无 $pat OK"; fi
        done
        echo "== /tf 发布者(期望: ekf_filter_node + amcl + static, 无 slam) =="
        timeout 15 ros2 topic info /tf -v 2>/dev/null | grep -E 'Node name|Publisher count'
        echo "== map->odom_combined (应可解析, 发布者=amcl) =="
        timeout 5 ros2 run tf2_ros tf2_echo map odom_combined 2>/dev/null | grep -m1 -A2 Translation || echo "  不通(AMCL 未发 TF?)"
        echo "== /amcl_pose 一帧 =="
        timeout 12 ros2 topic echo /amcl_pose --once 2>/dev/null | sed -n '1,14p' || echo "  无输出(初始位姿未发?)"
        echo "== /particle_cloud 发布者在 =="
        timeout 8 ros2 topic info /particle_cloud 2>/dev/null | grep -E 'count' || echo "  话题不存在"
    } | tee "$out"
    echo "核对输出已存: $out"
}

cmd_stop() {
    load_state || exit 1
    echo "== 按序收链: 定位组(先停 map->odom) -> Run_car -> monitor -> daemon =="
    kill_group "$LOC_PID" "map_server+amcl+lifecycle"
    kill_group "$RUN_CAR_PID" "Run_car(含driver)"
    local mon_pid
    mon_pid="$(pgrep -f map_monitor.py | head -1 || true)"
    [ -n "$mon_pid" ] && kill_group "$mon_pid" "map_monitor"
    pkill -x map_server 2>/dev/null
    pkill -x amcl 2>/dev/null
    ros2 daemon stop >/dev/null 2>&1
    rm -f "$STATE"
    echo "== 收链完成, 车辆应静止 =="
}

case "${1:-}" in
    start) shift; cmd_start "${1:-}" ;;
    check) cmd_check ;;
    stop)  cmd_stop ;;
    *) echo "用法: $0 start <地图yaml> | check | stop"; exit 1 ;;
esac
