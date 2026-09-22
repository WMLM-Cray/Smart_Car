#!/bin/bash
# fetch_monitor.sh —— 队友端(Git Bash/WSL)拉图循环: 临时PNG + 原子替换 + 失败保留旧图
# 2026-09-19 新增。用法: fetch_monitor.sh [车机IP] [用户名]
#   默认 IP=192.168.5.100, 用户=orange-pi
set -u
CARIP="${1:-192.168.5.100}"
CARUSER="${2:-orange-pi}"
REMOTE="$CARUSER@$CARIP:~/racecar_team/logs/slam/monitor.png"

echo "拉图循环已启动: $REMOTE"
echo "每 2 秒一轮; Ctrl+C 退出。浏览器打开同目录 monitor.html 即可看实时图。"

while true; do
  if scp "$REMOTE" monitor.png.tmp >/dev/null 2>&1; then
    mv -f monitor.png.tmp monitor.png
    echo "[$(date '+%F %T')] 更新成功"
  else
    echo "[$(date '+%F %T')] 拉图失败, 保留旧 monitor.png"
    rm -f monitor.png.tmp
  fi
  sleep 2
done
