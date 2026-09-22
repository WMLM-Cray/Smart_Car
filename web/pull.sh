#!/bin/bash
# pull.sh —— 从车机拉取 monitor.png(地图) + camera.png(相机) 到本目录 media/, 供 serve.py 展示
# 延续 fetch_monitor.sh 的做法: 临时文件 + 原子替换 + 失败保留旧图 + 串行不堆积。
# 用法: pull.sh <车机IP> [用户名]   (IP 必填——车机走手机热点 DHCP, 地址每次可能变, 禁止写死)
#   例: pull.sh 192.168.184.215
# 依赖: ssh/scp 可直连车机(Windows 用 Git Bash 或 WSL)。
set -u
CARIP="${1:-}"
CARUSER="${2:-orange-pi}"
if [ -z "$CARIP" ]; then
  echo "用法: $0 <车机IP> [用户名=orange-pi]" >&2
  echo "  车机IP 是手机热点 DHCP 动态分配, 每次先查实际地址, 不写死。" >&2
  exit 1
fi
HERE="$(cd "$(dirname "$0")" && pwd)"
MEDIA="$HERE/media"
mkdir -p "$MEDIA"
REMOTE="~/racecar_team/logs/slam"

echo "拉图循环已启动: $CARUSER@$CARIP:$REMOTE -> $MEDIA"
echo "每 2 秒一轮, Ctrl+C 退出。另开终端跑: python3 serve.py, 浏览器/手机打开 http://<本机IP>:8123/"

while true; do
  for src in monitor.png camera.png; do
    if scp "$CARUSER@$CARIP:$REMOTE/$src" "$MEDIA/$src.tmp" >/dev/null 2>&1; then
      mv -f "$MEDIA/$src.tmp" "$MEDIA/$src"
      echo "[$(date '+%F %T')] $src 更新"
    else
      echo "[$(date '+%F %T')] $src 拉取失败(保留旧图)"
      rm -f "$MEDIA/$src.tmp"
    fi
  done
  sleep 2
done
