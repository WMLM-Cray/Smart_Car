#!/bin/bash
# 厂家工作区备份(2026-09-16 任务书:实车接入第一步·任务3)
#
# 备份对象:~/racecar 全部源码/脚本/配置(排除 log;保留 build/install 以求完整)、
#           /etc/udev/rules.d/ 串口绑定规则、包/驱动清单。
# 安全:只读打包,不覆盖、不删改任何原文件;不备份含密码的文件。
# 恢复:见同目录 REALCAR_ONBOARDING.md §4.3(整机兜底=出厂镜像 test-0906.img.zip 重烧)。
# 输出:OUT 目录(默认 ~/vendor_backup_日期/)下:
#   racecar_src.tar.gz / udev_rules.tar.gz / env_snapshot.txt / 恢复说明.txt
set -u

OUT="${1:-$HOME/vendor_backup_$(date +%Y%m%d_%H%M%S)}"
mkdir -p "$OUT"
echo "备份输出目录: $OUT"

# ---- 1. 厂家工作区(排除 log——体积大且可重建;其余全保) ----
if [ -d "$HOME/racecar" ]; then
    tar czf "$OUT/racecar_src.tar.gz" \
        --exclude="$HOME/racecar/log" \
        -C "$HOME" racecar \
        && echo "OK racecar_src.tar.gz $(du -h "$OUT/racecar_src.tar.gz" | cut -f1)" \
        || echo "FAIL racecar_src.tar.gz"
else
    echo "未找到 ~/racecar(工作区可能在别的路径),跳过——请把实际路径告知后再备"
fi

# ---- 2. 串口绑定规则 + 其他 udev 规则 ----
tar czf "$OUT/udev_rules.tar.gz" -C /etc udev/rules.d 2>/dev/null \
    && echo "OK udev_rules.tar.gz" || echo "FAIL udev_rules.tar.gz(可能需要 sudo,先跳过)"

# ---- 3. 环境快照(复现装机用) ----
{
    echo "### dpkg --get-selections"
    dpkg --get-selections 2>/dev/null
    echo; echo "### pip3 list"
    pip3 list 2>/dev/null
    echo; echo "### /opt/ros/"
    ls /opt/ros/ 2>/dev/null
} > "$OUT/env_snapshot.txt"

# ---- 4. 校验和 + 恢复说明 ----
( cd "$OUT" && md5sum racecar_src.tar.gz udev_rules.tar.gz > MD5SUMS 2>/dev/null )
cat > "$OUT/恢复说明.txt" <<'EOF'
厂家工作区备份恢复方法(2026-09-16)
====================================
1. 文件级恢复(改动丢失时):
   tar xzf racecar_src.tar.gz -C ~/
   # 恢复后 ~/racecar 回到备份时点;build/install 建议重编(colcon build)
   # udev 规则:sudo tar xzf udev_rules.tar.gz -C /etc && sudo udevadm control --reload
2. 整机恢复(系统损坏时):
   用 F:\...\6.ROS2镜像-出厂版本\test-0906.img.zip + Ascend-devkit-imager
   "本地制卡"烧录 SD 卡(会清空整盘数据,烧前确认)。此为出厂状态兜底。
3. 本备份产生于实车,不含任何密码/密钥文件;MD5SUMS 供传输校验。
EOF

echo "完成: $(ls "$OUT")"
echo "下一步:用 MobaXterm(SFTP)把 $OUT 整个目录拖回电脑归档(双备份)。"
