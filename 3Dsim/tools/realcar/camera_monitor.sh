#!/bin/bash
# camera_monitor.sh —— 相机帧只读显示独立启动/关闭/状态(车机侧)
# 2026-09-21 收口轮新增, 与 display_monitor.sh 同款边界: 只订图像话题, 不动控制链。
# 启动边界: ①确认 ROS 环境已加载 ②确认输出目录存在 ③确认无已有 camera_monitor 实例
#          ④nohup 后台启动 + 记录本轮 PID, 关闭只停本轮进程(读 PIDFILE 精确 kill, 不用 pkill -f)。
# 用法: camera_monitor.sh {start [image-topic]|stop|status}
#   start 可带图像话题名(默认 /image_raw)
set -u

HERE="$(cd "$(dirname "$0")" && pwd)"
CAM_PY="$HERE/camera_monitor.py"
LOG_DIR="$HOME/racecar_team/logs/slam"
LOG="$LOG_DIR/camera_monitor.log"
PIDFILE="$LOG_DIR/camera.pid"
OUT="$LOG_DIR/camera.png"

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

existing_pid() {
  if [ -f "$PIDFILE" ]; then
    local p
    p="$(cat "$PIDFILE" 2>/dev/null)"
    if [ -n "$p" ] && kill -0 "$p" 2>/dev/null; then
      echo "$p"
      return 0
    fi
  fi
  pgrep -f "camera_monitor.py" 2>/dev/null | head -1
}

start() {
  local image_topic="${1:-}"
  ensure_ros || exit 1
  if [ ! -f "$CAM_PY" ]; then
    echo "错误: 找不到 $CAM_PY" >&2
    exit 2
  fi
  mkdir -p "$LOG_DIR"

  local p
  p="$(existing_pid)"
  if [ -n "$p" ]; then
    echo "已有 camera_monitor 实例在跑 (PID=$p), 不重复启动。如确要重启请先 stop。"
    exit 3
  fi

  if [ -n "$image_topic" ]; then
    nohup python3 "$CAM_PY" --image-topic "$image_topic" > "$LOG" 2>&1 < /dev/null &
    echo "启动参数: --image-topic $image_topic"
  else
    nohup python3 "$CAM_PY" > "$LOG" 2>&1 < /dev/null &
  fi
  local pid=$!
  echo "$pid" > "$PIDFILE"
  sleep 1
  if kill -0 "$pid" 2>/dev/null; then
    echo "已启动 camera_monitor (本轮 PID=$pid, 输出 $OUT)"
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
      echo "已停止本轮 camera_monitor (PID=$p)"
    else
      echo "PIDFILE 记录 PID=$p 已不在, 无需停止"
    fi
    rm -f "$PIDFILE"
  else
    echo "无 PIDFILE, 未记录本轮 camera_monitor; 如需清理请手动确认: pgrep -f camera_monitor.py"
  fi
}

status() {
  local p=""
  if [ -f "$PIDFILE" ]; then
    p="$(cat "$PIDFILE" 2>/dev/null)"
  fi
  if [ -n "$p" ] && kill -0 "$p" 2>/dev/null; then
    echo "camera_monitor 运行中 (PID=$p)"
  else
    echo "camera_monitor 未运行 (PIDFILE=$PIDFILE)"
  fi
  if [ -f "$OUT" ]; then
    echo "最新 PNG: $OUT  ($(stat -c '%y' "$OUT" 2>/dev/null))"
  else
    echo "尚无 PNG 输出"
  fi
}

case "${1:-}" in
  start)  start "${2:-}" ;;
  stop)   stop ;;
  status) status ;;
  *) echo "用法: $0 {start [image-topic]|stop|status}" >&2; exit 1 ;;
esac