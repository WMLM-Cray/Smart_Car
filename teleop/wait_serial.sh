#!/bin/sh
# racecar-chassis 启动前等四个串口就绪**并且稳定**。以 root 跑(drop-in 里用 ExecStartPre=+)。
#
# 2026-09-23 开机实测两种坑:
#
# 1. 底盘 launch 起来时 /dev/laser 根本没建出来(雷达 exit 255),/dev/encoder 打不开
#    (encoder_node exit 1)。launch 不重启死掉的子进程,服务还显示 active。
# 2. 更阴的:内核日志里开机 13.7s 串口第一次枚举,**18.8s 整个 USB hub 复位,所有串口
#    断开**,23.4s 才重新枚举(名字不变)。只看"链接在不在"的话 16s 就放行了,厂家驱动
#    打开的是第一次枚举的 ttyUSB2,两秒后设备没了、它也不重开 —— 之后每条指令都写空,
#    驱动照样订阅、照样打日志,电调一点反应没有。架空扫到 1660 都不转就是这个。
#
# 所以这里要求四个设备**连续 STABLE 秒没被重建**(看设备节点的 inode 号:重新枚举会换新 inode,读写和 chmod 不会)。
# 第一次枚举到复位隔了 5.1s,STABLE 给 8s。等不齐也照样放行(exit 0),缺什么由
# carready.py 报出来,它也会检查电调串口是不是真的通。
#
# 装:/etc/systemd/system/racecar-chassis.service.d/wait-serial.conf
#   [Service]
#   ExecStartPre=+/bin/sh /home/orange-pi/autocar/wait_serial.sh

DEVS="/dev/laser /dev/encoder /dev/car /dev/imu"
STABLE=8
LIMIT=90

udevadm settle --timeout=30
i=0
calm=0
last=""
while [ $i -lt $LIMIT ]; do
    missing=""
    sig=""
    for d in $DEVS; do
        if [ -e "$d" ]; then
            sig="$sig $(stat -Lc %i "$d")"
        else
            missing="$missing $d"
        fi
    done
    if [ -z "$missing" ] && [ "$sig" = "$last" ]; then
        calm=$((calm + 1))
        if [ $calm -ge $STABLE ]; then
            echo "串口齐了,稳定 ${STABLE}s(开机后第 $(cut -d. -f1 /proc/uptime)s)"
            exit 0
        fi
    else
        calm=0
    fi
    last="$sig"
    if [ -n "$missing" ] && [ $((i % 5)) -eq 0 ]; then
        echo "缺$missing,重新触发 udev"
        udevadm trigger --subsystem-match=tty --action=add
        udevadm settle --timeout=5
    fi
    sleep 1
    i=$((i + 1))
done
echo "等了 ${LIMIT}s 串口还不齐或一直在变(缺:$missing)—— 照常启动,carready.py 会报" >&2
exit 0
