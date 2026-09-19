#!/bin/bash
# display_monitor.sh —— 独立显示启动/关闭/状态(车机侧)
# 2026-09-19 新增: 不借 slam_scan.sh / amcl_localize.sh, 避免定位阶段误启另一套 SLAM 链。
# 启动边界: ①确认 ROS 环境已加载 ②确认输出目录存在 ③确认无已有 monitor 实例
#          ④nohup 后台启动 + 记录本轮 PID, 关闭时只停本轮进程(读 PIDFILE 精确 kill, 不用 pkill -f)。
# 用法: display_monitor.sh {start|stop|status}
set -u

HERE="$(cd "$(dirname "$0")" && pwd)"
MON_PY="$HERE/map_monitor.py"
LOG_DIR="$HOME/racecar_team/logs/slam"
LOG="$LOG_DIR/monitor.log"
PIDFILE="$LOG_DIR/monitor.pid"
OUT="$LOG_DIR/monitor.png"

# ---------- 启动前置: 确认 ROS 环境已加载 ----------
ensure_ros() {
  if command -v ros2 >/dev/null 2>&1; then
    return 0
  fi
  if [ -f /opt/ros/humble/setup.bash ]; then
    # shellcheck disable=SC1091
    . /opt/ros/humble/setup.bash
    command -v ros2 >/dev/null 2>&1 && return 0
  fi
  echo "错误: ros2 未加载且 /opt/ros/humble/setup.bash 不可用, 请先 source 环境" >&2
  return 1
}

# ---------- 已有实例检查 ----------
existing_pid() {
  if [ -f "$PIDFILE" ]; then
    local p
    p="$(cat "$PIDFILE" 2>/dev/null)"
    if [ -n "$p" ] && kill -0 "$p" 2>/dev/null; then
      echo "$p"
      return 0
    fi
  fi
  # PIDFILE 缺失/失效时, 兜底按命令名查(只报不杀)
  pgrep -f "map_monitor.py" 2>/dev/null | head -1
}

start() {
  ensure_ros || exit 1
  if [ ! -f "$MON_PY" ]; then
    echo "错误: 找不到 $MON_PY" >&2
    exit 2
  fi
  mkdir -p "$LOG_DIR"

  local p
  p="$(existing_pid)"
  if [ -n "$p" ]; then
    echo "已有 monitor 实例在跑 (PID=$p), 不重复启动。如确要重启请先 stop。"
    exit 3
  fi

  # 启动(独立进程, 记录本轮 PID)
  nohup python3 "$MON_PY" > "$LOG" 2>&1 < /dev/null &
  local pid=$!
  echo "$pid" > "$PIDFILE"
  sleep 1
  if kill -0 "$pid" 2>/dev/null; then
    echo "已启动 monitor (本轮 PID=$pid, 输出 $OUT)"
    echo "本轮日志: $LOG"
  else
    echo "启动失败, 见 $LOG" >&2
    rm -f "$PIDFILE"
    exit 4
  fi
}

stop() {
  if [ -f "$PIDFILE" ]; then
    local p
    p="$(cat "$PIDFILE" 2>/dev/null)"
    if [ -n "$p" ] && kill -0 "$p" 2>/dev/null; then
      kill "$p"
      echo "已停止本轮 monitor (PID=$p)"
    else
      echo "PIDFILE 记录 PID=$p 已不在, 无需停止"
    fi
    rm -f "$PIDFILE"
  else
    echo "无 PIDFILE, 未记录本轮 monitor; 如需清理请手动确认: pgrep -f map_monitor.py"
  fi
}

status() {
  local p=""
  if [ -f "$PIDFILE" ]; then
    p="$(cat "$PIDFILE" 2>/dev/null)"
  fi
  if [ -n "$p" ] && kill -0 "$p" 2>/dev/null; then
    echo "monitor 运行中 (PID=$p)"
  else
    echo "monitor 未运行 (PIDFILE=$PIDFILE)"
  fi
  if [ -f "$OUT" ]; then
    echo "最新 PNG: $OUT  ($(stat -c '%y' "$OUT" 2>/dev/null))"
  else
    echo "尚无 PNG 输出"
  fi
}

case "${1:-}" in
  start)  start ;;
  stop)   stop ;;
  status) status ;;
  *) echo "用法: $0 {start|stop|status}" >&2; exit 1 ;;
esac
