#!/bin/bash
# 实车扫图一键链 (华为小车 阶段1: 实车扫图, 任务书 2026-09-17)
# 用法:
#   ./slam_scan.sh start [rviz]    起传感器链(Run_car)+slam_toolbox; 带 rviz 参数在车机屏(:0)起 RViz
#   ./slam_scan.sh check           只读核对: 话题/雷达频率/TF发布者/AMCL不存在, 输出落盘
#   ./slam_scan.sh save <名字>     存图到 ~/racecar_team/maps/<名字>/ (PGM+YAML+序列化+配置快照)
#   ./slam_scan.sh stop            按序收链: slam(先停 map->odom) -> Run_car -> rviz -> daemon stop
# 前置: 电调断开(driver 无指令输入时仅发零速帧, 2026-09-16 6bb97d8 结论)
# 环境: 车机 Humble + 厂家 ~/racecar 工作区(不改厂家任何文件)
# 收链原则(同 teleop_manual.sh 45e4c74): 登记进程树按序杀, 不用全局 pkill
set -u

TEAM="$HOME/racecar_team"
HERE="$(cd "$(dirname "$0")" && pwd)"
YAML="$HERE/realcar_slam.yaml"
LOG_DIR="$TEAM/logs/slam"
STATE="$TEAM/run/slam_scan.state"
mkdir -p "$LOG_DIR" "$TEAM/run" "$TEAM/maps"

set +u   # setup.bash 引用未定义变量, set -u 下必炸(6bb97d8 deploy 同款坑)
source /opt/ros/humble/setup.bash
source "$HOME/racecar/install/setup.bash"
set -u

TS="$(date +%Y%m%d_%H%M%S)"

alive() {  # alive <pid>
    [ -n "${1:-}" ] && kill -0 "$1" 2>/dev/null
}

load_state() {
    if [ ! -f "$STATE" ]; then
        echo "错误: 无运行状态文件(链未起或已收). 先 ./slam_scan.sh start"
        return 1
    fi
    . "$STATE"   # 定义 RUN_CAR_PID / SLAM_PID / RVIZ_PID
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

wait_topic_pub() {  # wait_topic_pub <话题> <最多秒> ; 成功返回0
    local topic="$1" max="$2" i
    for i in $(seq 1 "$max"); do
        if timeout 6 ros2 topic info "$topic" 2>/dev/null | grep -qE 'Publisher count: [1-9]'; then
            return 0
        fi
        sleep 1
    done
    return 1
}

cmd_start() {
    if alive "${RUN_CAR_PID:-}"; then echo "错误: 已有链在跑(PID=$RUN_CAR_PID). 先 ./slam_scan.sh stop"; exit 1; fi
    if [ ! -f "$YAML" ]; then echo "错误: 缺 $YAML"; exit 1; fi

    echo "== 前置检查: 串口设备 =="
    local dev miss=0
    for dev in /dev/car /dev/imu /dev/encoder /dev/laser; do
        if [ -e "$dev" ]; then echo "  OK  $dev -> $(readlink -f $dev)"
        else echo "  缺  $dev"; miss=1; fi
    done
    if [ "$miss" -eq 1 ]; then
        echo "串口别名缺失(开机 udevd 漏建设备, 已知坑). 修复: sudo systemctl restart systemd-udevd 后重试"
        exit 2
    fi

    ros2 daemon stop >/dev/null 2>&1   # 清一次坏态 daemon(2026-09-17 已知坑), 首条命令会自动重建

    echo "== 起 Run_car (传感器+EKF+driver, 电调断开) =="
    setsid ros2 launch racecar Run_car.launch.py > "$LOG_DIR/runcar_$TS.log" 2>&1 &
    RUN_CAR_PID=$!
    echo "  PGID=$RUN_CAR_PID 日志=$LOG_DIR/runcar_$TS.log"

    echo "== 等待 /scan 有发布者 (DDS 发现~4s+驱动启动, 最多90s) =="
    if ! wait_topic_pub /scan 90; then
        echo "超时: /scan 无发布者. 查日志: tail -30 $LOG_DIR/runcar_$TS.log"
        exit 3
    fi
    echo "  /scan 就绪"

    echo "== 等待 EKF 的 odom_combined->base_footprint TF =="
    local i
    for i in $(seq 1 30); do
        if timeout 4 ros2 run tf2_ros tf2_echo odom_combined base_footprint 2>/dev/null | grep -m1 -q Translation; then
            echo "  TF 就绪 (第 ${i} 次探测)"; break
        fi
        sleep 2
        if [ "$i" -eq 30 ]; then echo "超时: EKF TF 未出现. 查日志: tail -30 $LOG_DIR/runcar_$TS.log"; exit 3; fi
    done

    echo "== 起 slam_toolbox (online_async, 参数=$YAML) =="
    setsid ros2 launch slam_toolbox online_async_launch.py \
        slam_params_file:="$YAML" use_sim_time:=false \
        > "$LOG_DIR/slam_$TS.log" 2>&1 &
    SLAM_PID=$!
    echo "  PGID=$SLAM_PID 日志=$LOG_DIR/slam_$TS.log"

    if ! wait_topic_pub /map 60; then
        echo "警告: 60s 内 /map 未出现, 查日志: tail -30 $LOG_DIR/slam_$TS.log"
    fi

    RVIZ_PID=""
    if [ "${1:-}" = "rviz" ]; then
        echo "== 起 RViz2 于车机屏 (:0) =="
        DISPLAY=:0 setsid rviz2 > "$LOG_DIR/rviz_$TS.log" 2>&1 &
        RVIZ_PID=$!
        echo "  PGID=$RVIZ_PID (需车机接显示器)"
    fi

    cat > "$STATE" <<EOF
RUN_CAR_PID=$RUN_CAR_PID
SLAM_PID=$SLAM_PID
RVIZ_PID=$RVIZ_PID
START_TS=$TS
EOF
    echo "== 起链完成, state=$STATE =="
    echo "   推车扫图; 中途核对: ./slam_scan.sh check ; 存图: ./slam_scan.sh save <名字>; 收链: ./slam_scan.sh stop"
}

cmd_check() {
    local out="$LOG_DIR/check_$TS.txt"
    {
        echo "# 扫图链只读核对 $TS (车机钟, 相对刻度)"
        echo "== 话题清单 =="
        timeout 15 ros2 topic list 2>/dev/null
        echo "== 运动话题发布者(应全为 0 或不存在) =="
        for t in /cmd_vel /car_cmd_vel /teleop_cmd_vel /teleop/cmd_vel; do
            echo "-- $t"
            timeout 10 ros2 topic info "$t" 2>/dev/null | grep -E 'count'
        done
        echo "== /scan 频率采样(应约12Hz) =="
        timeout 8 ros2 topic hz /scan 2>/dev/null | grep -m1 average
        echo "== /tf 发布者(期望: ekf_filter_node + slam_toolbox + 3个static) =="
        timeout 15 ros2 topic info /tf -v 2>/dev/null | grep -E 'Node name|Publisher count|Subscription count'
        echo "== AMCL 确认不在(无进程无话题) =="
        ps aux | grep -E '[a]mcl' || echo "  无 amcl 进程"
        timeout 10 ros2 topic info /amcl_pose 2>/dev/null | grep -E 'count' || echo "  /amcl_pose 话题不存在"
        echo "== TF 链通性 =="
        echo "-- map->odom_combined (slam_toolbox 发)"
        timeout 4 ros2 run tf2_ros tf2_echo map odom_combined 2>/dev/null | grep -m1 -A2 Translation || echo "  不通"
        echo "-- odom_combined->base_footprint (EKF 发)"
        timeout 4 ros2 run tf2_ros tf2_echo odom_combined base_footprint 2>/dev/null | grep -m1 -A2 Translation || echo "  不通"
        echo "-- base_footprint->laser_link (static)"
        timeout 4 ros2 run tf2_ros tf2_echo base_footprint laser_link 2>/dev/null | grep -m1 -A2 Translation || echo "  不通"
    } | tee "$out"
    echo "核对输出已存: $out"
}

cmd_save() {
    local name="${1:-}"
    if [ -z "$name" ]; then echo "用法: ./slam_scan.sh save <名字>"; exit 1; fi
    load_state || exit 1
    if ! alive "$SLAM_PID"; then echo "错误: slam_toolbox 不在跑, 无法 serialize"; exit 1; fi
    local dir="$TEAM/maps/$name"
    mkdir -p "$dir"
    echo "== serialize slam_toolbox 位姿图 =="
    timeout 30 ros2 service call /slam_toolbox/serialize_map \
        slam_toolbox/srv/SerializePoseGraph "{filename: '$dir/slam_map'}" 2>&1 | tail -3
    echo "== save_map_pgm.py 转存 PGM+YAML =="
    echo "  (Humble map_saver_cli 对 latched /map 收帧不稳 Failed to spin; 2026-09-17 改 python 转存)"
    timeout 60 python3 "$HERE/save_map_pgm.py" "$dir" map || exit 4
    echo "== 快照配置与核对输出 =="
    cp "$YAML" "$dir/realcar_slam.yaml"
    cp "$0" "$dir/slam_scan.sh.snapshot"
    ls -t "$LOG_DIR"/check_*.txt 2>/dev/null | head -1 | xargs -r -I{} cp {} "$dir/check_snapshot.txt"
    ls -la "$dir"
}

cmd_restart() {
    # 重扫: 只重置 slam_toolbox(丢掉当前图)与渲染监控(清轨迹), 保留 Run_car 传感器链
    # 执行时车必须静止(重启瞬间的车位姿=新地图原点)
    load_state || exit 1
    if ! alive "$RUN_CAR_PID"; then echo "错误: Run_car 传感器链不在跑, 请完整 start"; exit 1; fi
    echo "== 停止旧 slam_toolbox 与渲染监控 =="
    alive "$SLAM_PID" && kill_group "$SLAM_PID" "slam_toolbox"
    local mon_pid
    mon_pid="$(pgrep -f map_monitor.py | head -1 || true)"
    [ -n "$mon_pid" ] && kill_group "$mon_pid" "map_monitor"
    sleep 2
    echo "== 重起 slam_toolbox (全新地图) =="
    setsid ros2 launch slam_toolbox online_async_launch.py \
        slam_params_file:="$YAML" use_sim_time:=false \
        > "$LOG_DIR/slam_$TS.log" 2>&1 &
    SLAM_PID=$!
    echo "  PGID=$SLAM_PID 日志=$LOG_DIR/slam_$TS.log"
    if ! wait_topic_pub /map 60; then
        echo "超时: /map 未出现, 查日志: tail -30 $LOG_DIR/slam_$TS.log"; exit 3
    fi
    echo "== 重起渲染监控 =="
    setsid nohup python3 "$HERE/map_monitor.py" > "$LOG_DIR/monitor.log" 2>&1 < /dev/null &
    cat > "$STATE" <<EOF
RUN_CAR_PID=$RUN_CAR_PID
SLAM_PID=$SLAM_PID
RVIZ_PID=
START_TS=$TS
EOF
    echo "== 重置完成, 新地图原点=当前车位姿. 推车扫图 =="
}

cmd_stop() {
    load_state || exit 1
    echo "== 按序收链: slam(先停 map->odom) -> Run_car -> rviz -> monitor -> daemon =="
    kill_group "$SLAM_PID" "slam_toolbox"
    kill_group "$RUN_CAR_PID" "Run_car(含driver)"
    [ -n "$RVIZ_PID" ] && kill_group "$RVIZ_PID" "rviz2"
    local mon_pid
    mon_pid="$(pgrep -f map_monitor.py | head -1 || true)"
    [ -n "$mon_pid" ] && kill_group "$mon_pid" "map_monitor"
    ros2 daemon stop >/dev/null 2>&1
    rm -f "$STATE"
    echo "== 收链完成, 车辆应静止. 物理停止方式(断电调)仍适用 =="
}

case "${1:-}" in
    start) shift; cmd_start "${1:-}" ;;
    check) cmd_check ;;
    save)  shift; cmd_save "${1:-}" ;;
    restart) cmd_restart ;;
    stop)  cmd_stop ;;
    *) echo "用法: $0 start [rviz] | check | save <名字> | stop"; exit 1 ;;
esac
