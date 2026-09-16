#!/bin/bash
# deploy_teleop.sh —— 从 Windows(Git Bash)向车机部署手动链(2026-09-17)
# 前置:车机供电开机且 ssh hwcar 可达(密钥直连);厂家 ~/racecar 原样不动。
# 幂等:重复执行安全;已部署文件先比对,不一致会提示而非静默覆盖。
set -u
REPO="$(cd "$(dirname "$0")/../.." && pwd)"
TMP_TAR="/d/Claude/tmp/racecar_teleop_deploy.tar"
[ -d /d/Claude/tmp ] || mkdir -p /d/Claude/tmp

echo "== 0. 车机连通性 =="
ssh -o ConnectTimeout=8 -o BatchMode=yes hwcar "echo SSH_OK && hostname" || {
  echo "!! ssh hwcar 不可达(车机没电/未开机/不在网)"; exit 2; }

echo "== 1. 车上 ~/racecar_team 就位检查(不覆盖已有成果,先比对) =="
ssh hwcar 'mkdir -p ~/racecar_team/teleop_ws/src'
for f in tools/teleop/teleop_manual.sh:racecar_team/teleop_manual.sh \
         tools/teleop/realcar_teleop.yaml:racecar_team/realcar_teleop.yaml; do
  src="${f%%:*}"; dst="${f##*:}"
  lmd5=$(git -C "$REPO" show "HEAD:$src" | md5sum | cut -d" " -f1)
  rmd5=$(ssh hwcar "md5sum ~/$dst 2>/dev/null" | cut -d" " -f1)
  if [ "$lmd5" = "$rmd5" ]; then
    echo "  一致: ~/$dst"
  elif [ -z "$rmd5" ]; then
    scp -q "$REPO/$src" "hwcar:~/$dst" && echo "  新部署: ~/$dst" || { echo "!! scp 失败: $dst"; exit 3; }
  else
    echo "  !! 车上已有且不同(~/$dst),本次不覆盖;确要用仓库版: scp '$REPO/$src' hwcar:~/$dst"
  fi
done
# 安全层:6bb97d8 已部署过,只比对
lmd5=$(git -C "$REPO" show "HEAD:tools/realcar/cmd_vel_safety.py" | md5sum | cut -d" " -f1)
rmd5=$(ssh hwcar "md5sum ~/racecar_team/cmd_vel_safety.py 2>/dev/null" | cut -d" " -f1)
if [ "$lmd5" = "$rmd5" ]; then echo "  一致: ~/racecar_team/cmd_vel_safety.py"
elif [ -z "$rmd5" ]; then scp -q "$REPO/tools/realcar/cmd_vel_safety.py" hwcar:~/racecar_team/ && echo "  新部署: cmd_vel_safety.py"
else echo "  !! 车上 cmd_vel_safety.py 与仓库 HEAD 不同,不覆盖(人工裁决)"; fi

echo "== 2. 打包手柄包(含 e2113b0 判据3b补丁)并上传 =="
git -C "$REPO" archive HEAD:hwcart_sim/src/racecar_teleop -o "$TMP_TAR" || {
  echo "!! git archive 失败"; exit 3; }
scp -q "$TMP_TAR" hwcar:'/tmp/racecar_teleop_deploy.tar' || {
  echo "!! 上传失败"; exit 3; }
# 远端 set -e:解包/构建/单测任一失败 → ssh 非零 → 本地中止(不打印"部署完成")
ssh hwcar '
  set -e
  rm -rf ~/racecar_team/teleop_ws/src/racecar_teleop
  mkdir -p ~/racecar_team/teleop_ws/src/racecar_teleop
  tar -xf /tmp/racecar_teleop_deploy.tar -C ~/racecar_team/teleop_ws/src/racecar_teleop
  echo "  已解包: $(ls ~/racecar_team/teleop_ws/src/racecar_teleop | tr "\n" " ")"
  set +u; source /opt/ros/humble/setup.bash; set -u
  cd ~/racecar_team/teleop_ws
  [ -d build ] && rm -rf build/racecar_teleop install/racecar_teleop   # clean 重建本包
  colcon build --packages-select racecar_teleop 2>&1 | tail -3
  source install/setup.bash
  cd src/racecar_teleop && python3 -m pytest test/ -q 2>&1 | tail -1
' || { echo "!! 车端解包/构建/单测失败(见上方输出),中止部署"; exit 4; }

echo "== 3. joy 包与手柄设备检查 =="
ssh hwcar '
  set +u; source /opt/ros/humble/setup.bash; set -u
  if ros2 pkg list 2>/dev/null | grep -q "^joy$"; then echo "  joy 包: 已安装"
  else echo "  joy 包: 未安装 → sudo apt install ros-humble-joy(车机需联网)"; fi
  ls -l /dev/input/js* 2>/dev/null || echo "  手柄未插(部署不受影响,插入后重查)"
'

echo
echo "== 部署完成。后续(按 TELEOP_MANUAL_GUIDE.md): =="
echo "  A. 插手柄 → ssh hwcar 'ros2 topic echo /joy' 标定轴/按键 → 回填 realcar_teleop.yaml"
echo "  B. 电调断开 → bash ~/racecar_team/teleop_manual.sh → 按文档做无动力验证+拔线实测"
echo "  C. 全链帧级(可选): dryrun_teleop_chain.py --serial /tmp/fakecar(车机,电调断开)"
