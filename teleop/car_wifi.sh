#!/usr/bin/env bash
# 把车连到指定 WiFi。在车上跑,不要在 PC 上跑:
#     bash car_wifi.sh "IPhone 33318" '你的密码'        # 加密网络
#     bash car_wifi.sh "某开放网络"                      # 开放网络就不给密码
#
# 密码在你自己手里输,脚本不保存也不回显。SSID 里有空格/特殊字符时务必带引号 ——
# 直接从 PowerShell 用 ssh 传参数容易被吞掉引号,先 ssh 进车再在车上跑这一行最稳。
set -euo pipefail

SSID=${1:?用法: bash car_wifi.sh "SSID" [密码]}
PASS=${2:-}
IFACE=${IFACE:-wlan0}

if ! nmcli -t -f DEVICE device | grep -qx "$IFACE"; then
    echo "没有网卡 $IFACE,当前无线网卡:"
    nmcli -t -f DEVICE,TYPE,STATE device | awk -F: '$2=="wifi" {print "  "$1"  "$3}'
    echo "用 IFACE=xxx bash car_wifi.sh ... 指定"
    exit 1
fi

# 车自己的热点如果就挂在这块网卡上,连上别的 WiFi 会把热点顶掉 —— 先提醒
AP_CONN=$(nmcli -t -f NAME,TYPE,DEVICE connection show --active |
          awk -F: -v i="$IFACE" '$2=="802-11-wireless" && $3==i {print $1}')
if [ -n "$AP_CONN" ]; then
    MODE=$(nmcli -g 802-11-wireless.mode connection show "$AP_CONN" 2>/dev/null || true)
    [ "$MODE" = "ap" ] && echo "⚠ $IFACE 上正跑着热点 $AP_CONN,连上 $SSID 后热点会断,192.168.5.100 就不通了"
fi

echo "扫描 $IFACE ..."
nmcli device wifi rescan ifname "$IFACE" 2>/dev/null || true
sleep 3
if ! nmcli -t -f SSID device wifi list ifname "$IFACE" | grep -qxF "$SSID"; then
    echo "扫不到 \"$SSID\"。附近能看到的:"
    nmcli -t -f SIGNAL,FREQ,SSID device wifi list ifname "$IFACE" | head -15
    echo "(网卡是 2.4G 的话看不到 5GHz 的 AP,让路由器把 2.4G 也开出来)"
    exit 1
fi

if [ -n "$PASS" ]; then
    nmcli device wifi connect "$SSID" password "$PASS" ifname "$IFACE"
else
    nmcli device wifi connect "$SSID" ifname "$IFACE"
fi
nmcli connection modify "$SSID" connection.autoconnect yes connection.autoconnect-priority 10

echo
echo "== 结果 =="
nmcli -g GENERAL.STATE,IP4.ADDRESS,IP4.GATEWAY device show "$IFACE"
echo "PC 上用这个地址 ssh: $(nmcli -g IP4.ADDRESS device show "$IFACE" | head -1 | cut -d/ -f1)"
