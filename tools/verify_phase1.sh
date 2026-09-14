#!/usr/bin/env bash
# =============================================================================
# 阶段1 验收脚本(headless)
# 用法(WSL 内): bash ~/hwcart_sim/tools/verify_phase1.sh
#               bash ~/hwcart_sim/tools/verify_phase1.sh --selftest-fail  # 失败路径退出码自检
# 构建证据由独立步骤提供: bash ~/hwcart_sim/tools/rebuild_racecar.sh(本脚本不含构建)
#
# 2026-09-14 返工第2轮(按独立验收收尾清单):
#   - 失败必须 exit 非零(原来只 echo,退出码恒 0)
#   - 清理只针对本次启动的进程:launch 记 PID 先 TERM 后 KILL;gzserver/gzclient
#     只杀"本次新增"PID 集合(记录启动前快照,comm 求差);开场仅清占用
#     11345 的上轮残留 gzserver,不再全局 pkill
#   - 按 /clock 仿真时间增量计算期望(不再只靠墙钟 sleep)
#   - 工况补齐:零指令静止/直行/倒车/左右转弯(不饱和)/零速停车/限位饱和/
#     已知障碍物测距/相机非空图像/频率容差/时间戳有效/光学 frame TF
#   - 阈值集中在 THR_* 变量(负例测试 tools/verify_negative.sh 用 sed 注入)
# =============================================================================
export ROS_DOMAIN_ID=42
export ROS2CLI_NO_DAEMON=1
source /opt/ros/humble/setup.bash
source ~/hwcart_sim/hwcart_sim/install/setup.bash
PASS=0; FAIL=0
ok()  { echo "[PASS] $1"; PASS=$((PASS+1)); }
bad() { echo "[FAIL] $1"; FAIL=$((FAIL+1)); }

# ---- 判据阈值(集中管理;定值依据见各判据处注释,勿为凑结果放宽) ----
THR_STRAIGHT=1.0      # 直行最小位移 m(0.5m/s×5s 理论 2.5,扣斜坡余量)
THR_STRAIGHT_DEV=0.3  # 账面-真实最大偏差 m
THR_YAW_DRIFT=0.09    # 直行航向漂移 rad(5°)
THR_RANGE_ERR=0.10    # 障碍测距容差 m
THR_STILL=0.05        # 静止/停车最大位移 m
THR_TURN=0.3          # 转弯最小 |Δyaw| rad
THR_STEER_LIM=0.57    # 限位饱和上限 rad(0.55 + 0.02 余量)
THR_STEER_DIFF=0.02   # 内外轮角最小差 rad(理论 0.058)
THR_WHEEL_DIFF=0.03   # 内外后轮速最小差 m/s(理论 0.09)

# ---- 失败路径退出码自检(负例之一,秒回) ----
if [ "$1" = "--selftest-fail" ]; then
  bad "故意失败(失败路径退出码自检)"
  echo "===== 结果: PASS=$PASS FAIL=$FAIL ====="
  echo "HAS_FAILURES"
  exit $FAIL
fi

# ---- 进程管理:只针对本次启动 ----
GZ_BEFORE=$(pgrep -x gzserver 2>/dev/null | sort)
GZC_BEFORE=$(pgrep -x gzclient 2>/dev/null | sort)
LAUNCH_PID=""

precheck() {
  # 上轮崩溃残留的 gzserver 占 11345 会让本次启动 exit 255:
  # 只清"占用 11345 且确为 gzserver"的进程,其他占用者报告后退出
  if ss -ltn 2>/dev/null | grep -q 11345; then
    local pid
    pid=$(ss -ltnp 2>/dev/null | grep 11345 | grep -oP 'pid=\K[0-9]+' | head -1)
    if [ -n "$pid" ] && [ "$(ps -p "$pid" -o comm= 2>/dev/null)" = "gzserver" ]; then
      echo "[预检] 清理上轮残留 gzserver(pid=$pid,占 11345)"
      kill -9 "$pid" 2>/dev/null
    else
      echo "[预检] 端口 11345 被其他进程占用(pid=${pid:-未知}),请手动处理后重试"
      exit 2
    fi
    for i in 1 2 3 4 5; do ss -ltn 2>/dev/null | grep -q 11345 || break; sleep 1; done
  fi
}

cleanup() {
  # 1) launch 优先优雅退出(SIGTERM → launch 关停自己的全部子进程)
  if [ -n "$LAUNCH_PID" ]; then
    kill -TERM "$LAUNCH_PID" 2>/dev/null
    for i in 1 2 3 4 5 6 7 8; do kill -0 "$LAUNCH_PID" 2>/dev/null || break; sleep 1; done
    kill -9 "$LAUNCH_PID" 2>/dev/null
  fi
  # 2) gzserver/gzclient 不理 SIGTERM,只杀本次启动"新增"的
  local new
  new=$(comm -13 <(printf '%s\n' "$GZ_BEFORE") <(pgrep -x gzserver 2>/dev/null | sort) | grep -v '^$')
  [ -n "$new" ] && kill -9 $new 2>/dev/null
  new=$(comm -13 <(printf '%s\n' "$GZC_BEFORE") <(pgrep -x gzclient 2>/dev/null | sort) | grep -v '^$')
  [ -n "$new" ] && kill -9 $new 2>/dev/null
  # 3) 兜底:仅杀 racecar_sim 命名空间的可执行(launch -9 后的孤儿 sim_wiring)
  pkill -9 -f "racecar_sim" 2>/dev/null
  ros2 daemon stop >/dev/null 2>&1
  for i in 1 2 3 4 5; do ss -ltn 2>/dev/null | grep -q 11345 || break; sleep 1; done
  sleep 1
}
trap cleanup EXIT
precheck

# ---- 通用采集函数 ----
gz_pose() {  # Gazebo 世界位姿 → "px py pz yaw"
  timeout 8 ros2 service call /get_entity_state gazebo_msgs/srv/GetEntityState \
    "{name: 'racecar', reference_frame: 'world'}" 2>/dev/null | python3 -c "
import sys, re, math
t = sys.stdin.read()
m = re.search(r'Point\(x=([-+0-9.eE]+), y=([-+0-9.eE]+), z=([-+0-9.eE]+)\)', t)
q = re.search(r'Quaternion\(x=([-+0-9.eE]+), y=([-+0-9.eE]+), z=([-+0-9.eE]+), w=([-+0-9.eE]+)\)', t)
if not (m and q):
    print('PARSE_FAIL'); sys.exit(1)
px, py, pz = (float(v) for v in m.groups())
qx, qy, qz, qw = (float(v) for v in q.groups())
yaw = math.atan2(2*(qw*qz + qx*qy), 1 - 2*(qy*qy + qz*qz))
print(f'{px:.4f} {py:.4f} {pz:.4f} {yaw:.4f}')
"
}
simsec() {  # /clock 仿真时间(浮点秒;sec/nsec 两次 echo 并行,各 ~1.2s。
  local T=/tmp/.ss.$$ P1 P2 s n   # 串联 5s 拖垮采样窗口;--once 在验收
  # 负载下实测拿不到值(第4-6轮 NA),流式+grep -m1 是第 3 轮实证过的取法)
  timeout 8 ros2 topic echo /clock --field clock.sec 2>/dev/null \
    | grep -m1 -E '^[0-9]+$' > $T.s &
  P1=$!
  timeout 8 ros2 topic echo /clock --field clock.nanosec 2>/dev/null \
    | grep -m1 -E '^[0-9]+$' > $T.n &
  P2=$!
  wait $P1 $P2
  s=$(cat $T.s 2>/dev/null); n=$(cat $T.n 2>/dev/null); rm -f $T.s $T.n
  [ -z "$s" ] && { echo "NA"; return; }
  awk -v s="$s" -v n="${n:-0}" 'BEGIN{printf "%.3f", s+n/1e9}'
}
yaw_delta() {  # 归一化角差
  awk -v a="$1" -v b="$2" 'BEGIN{d=b-a; if(d>3.14159)d-=6.28319; if(d<-3.14159)d+=6.28319; printf "%.4f", d}'
}
drive() {  # v w 时长s
  timeout "$3" ros2 topic pub -r 20 /cmd_vel geometry_msgs/msg/Twist \
    "{linear: {x: $1}, angular: {z: $2}}" >/dev/null 2>&1
}
stop_car() { drive 0 0 1; }
get_js() {  # joint_states → KEY=VAL 行
  timeout 8 ros2 topic echo /joint_states 2>/dev/null \
    | awk '/^name:/{flag=1} /^---$/{if(flag)exit} flag{print}' \
    | python3 -c "
import sys, yaml
d = yaml.safe_load(sys.stdin) or {}
names = d.get('name', [])
pos = d.get('position', [])
vel = d.get('velocity', [])
def g(n, arr):
    return arr[names.index(n)] if n in names and len(arr) == len(names) else 'NA'
print('LEFT_STEER=' + str(g('left_steering_joint', pos)))
print('RIGHT_STEER=' + str(g('right_steering_joint', pos)))
print('LEFT_REAR_V=' + str(g('left_rear_wheel_joint', vel)))
print('RIGHT_REAR_V=' + str(g('right_rear_wheel_joint', vel)))
"
}
js_var() { echo "$JSV" | grep -m1 -oP "^$1=\K[-+0-9.eE]+"; }

echo "===== [1/8] 一键启动(use_gui:=false) ====="
ros2 launch racecar_sim sim_bringup.launch.py use_gui:=false > /tmp/sim_test.log 2>&1 &
LAUNCH_PID=$!
sleep 35
if grep -q "SpawnEntity: Successfully spawned entity" /tmp/sim_test.log; then ok "spawn 成功"; else bad "spawn 失败(见 /tmp/sim_test.log)"; fi
grep -q "parser error" /tmp/sim_test.log && bad "gazebo_ros2_control 参数注入错误" || ok "无 parser error(注释剥离生效)"
# 启动链失败传播:事件链任一环失败应 Shutdown 并写明原因
grep -q "终止启动链" /tmp/sim_test.log && bad "启动链中途失败(launch 已按退出码终止)" || ok "启动链各环退出码全零"

echo "===== [2/8] 控制器激活 ====="
CTRL=$(timeout 20 ros2 control list_controllers 2>/dev/null)
echo "$CTRL"
echo "$CTRL" | grep -q "joint_state_broadcaster.*active" && ok "joint_state_broadcaster active" || bad "joint_state_broadcaster 未激活"
echo "$CTRL" | grep -q "ackermann_steering_controller.*active" && ok "ackermann_steering_controller active" || bad "ackermann 未激活"
# 前轮自转关节完整发布(state-only 接口进 joint_states)
JSV=$(get_js)
N_JS=$(echo "$JSV" | grep -c '=')
[ "$N_JS" = "4" ] && ok "joint_states 含 4 关节(含前轮自转)" || bad "joint_states 缺关节(取到 $N_JS/4)"

echo "===== [3/8] 传感器(频率容差) ====="
wait_topic_alive() {
  local t=$1
  for i in $(seq 1 10); do
    timeout 5 ros2 topic hz "$t" 2>&1 | head -1 | grep -q "average rate" && return 0
    sleep 2
  done
  return 1
}
# 容差:±10% 标称(实测抖动 <2%)。grep -m1 流内直配 average rate:
# 高频话题 stdout 前几行可能混入 QoS 警告,head -1 会抓错行;低频(10Hz)
# 首个 average 出得慢,timeout 给 15s
check_hz() {  # 话题 标称Hz 容差
  local r
  r=$(timeout 15 ros2 topic hz "$1" 2>&1 | grep -m1 -oP 'average rate: \K[0-9.]+')
  echo "$1: ${r:-无} Hz(标称 $2)"
  awk -v r="${r:-0}" -v n="$2" -v tol="$3" 'BEGIN{exit !(r>n-tol && r<n+tol)}' \
    && ok "$1 频率 ${r}Hz 在容差内" || bad "$1 频率 ${r:-无}Hz 超出容差($2±$3)"
}
for t in /scan /image_raw /IMU_data; do
  if wait_topic_alive "$t"; then
    case $t in
      /scan)      check_hz $t 10 1 ;;
      /image_raw) check_hz $t 30 3 ;;
      /IMU_data)  check_hz $t 200 20 ;;
    esac
  else
    echo "$t: 等待超时"; bad "$t 无数据"
  fi
done
# 时间戳有效(走 /clock 仿真时间,非 0)
for t in /scan /IMU_data; do
  s=$(timeout 8 ros2 topic echo "$t" --field header.stamp.sec 2>/dev/null | grep -m1 -E '^[0-9]+$')
  awk -v s="${s:-0}" 'BEGIN{exit !(s>0)}' && ok "$t 时间戳有效(sec=$s)" || bad "$t 时间戳无效(sec=${s:-取不到})"
done

echo "===== [4/8] TF:厂家/物理/光学 frame ====="
TF_L=$(timeout 8 ros2 run tf2_ros tf2_echo base_footprint laser_link 2>/dev/null | grep -m1 "Translation")
echo "laser_link:        $TF_L"
echo "$TF_L" | grep -qP "Translation: \[[^,]+, [^,]+, 0\.0+\]" && ok "厂家 laser_link z=0 保留" || bad "laser_link z≠0"
TF_LS=$(timeout 8 ros2 run tf2_ros tf2_echo base_footprint laser_sensor_link 2>/dev/null | grep -m1 "Translation")
echo "laser_sensor_link: $TF_LS"
echo "$TF_LS" | grep -qP "Translation: \[[^,]+, [^,]+, 0\.21[0-9]*\]" && ok "laser_sensor_link z=0.21(物理高度)" || bad "laser_sensor_link z≠0.21"

timeout 12 ros2 topic echo /scan --field header.frame_id 2>/dev/null | grep -m1 -q laser_sensor_link && ok "/scan frame_id=laser_sensor_link" || bad "/scan frame_id 不对"
timeout 12 ros2 topic echo /IMU_data --field header.frame_id 2>/dev/null | grep -m1 -q imu_sensor_link && ok "/IMU_data frame_id=imu_sensor_link" || bad "/IMU_data frame_id 不对"
timeout 12 ros2 topic echo /image_raw --field header.frame_id 2>/dev/null | grep -m1 -qx "camera_optical" && ok "/image_raw frame_id=camera_optical" || bad "/image_raw frame_id 不对"

# 光学 frame 方向:base→camera_optical 的 rpy 应为 (-pi/2, 0, -pi/2)
# (z 前 x 右 y 下;正前方物体反投影方向在 base 中 +x)
RPY_LINE=$(timeout 8 ros2 run tf2_ros tf2_echo base_footprint camera_optical 2>/dev/null | grep -m1 -i "radian")
echo "camera_optical:    $RPY_LINE"
echo "$RPY_LINE" | grep -oP '\[\K-?[0-9.]+, *-?[0-9.]+, *-?[0-9.]+' | awk -F', *' '{
  r=$1+0; p=$2+0; y=$3+0
  if (r<-1.591 || r>-1.551) exit 1
  if (p<-0.02 || p>0.02) exit 1
  if (y<-1.591 || y>-1.551) exit 1
}' && ok "camera_optical rpy=(-π/2,0,-π/2) 光学约定" || bad "camera_optical 旋转不对(应 -π/2 0 -π/2)"

echo "===== [5/8] 静止/测距/图像(车未动) ====="
PA=$(gz_pose); read AX AY AZ AYAW <<< "$PA"
sleep 2
PB=$(gz_pose); read BX BY BZ BYAW <<< "$PB"
echo "静止 2s: ($AX,$AY,z=$AZ) → ($BX,$BY,z=$BZ)"
STILL=$(awk -v a="$AX" -v b="$BX" -v c="$AY" -v d="$BY" 'BEGIN{printf "%.4f", sqrt((b-a)^2+(d-c)^2)}')
awk -v s="$STILL" -v t="$THR_STILL" 'BEGIN{exit !(s<t)}' && ok "零指令静止(位移 ${STILL}m<$THR_STILL)" || bad "零指令漂移 ${STILL}m"
# 接地检查:静止时 base_footprint(实体原点)世界 z ≈ 0(轮心 WR-CZ 修正的
# 直接证据;GPT:不要只看静态 TF 的 z 数字)
awk -v z="${AZ:-9}" 'BEGIN{exit !(z>-0.05 && z<0.05)}' && ok "接地:静止时 z=${AZ}m≈0(footprint 贴地)" || bad "接地异常 z=${AZ}m(轮心高度错?)"

# 已知障碍物:world 内 marker_box(中心 x=6)前表面距原点 5.75m → /scan 正前方读数
# (echo --field 大数组输出是单行 flow 风格 [v1, v2, ...];小数组才是 '- v' 逐行,
#  两种都兼容;--- 分隔符与警告行丢弃;首条消息慢,重试)
scan_range() {
  local v
  for try in 1 2 3; do
    v=$(timeout 12 ros2 topic echo /scan --field ranges 2>/dev/null | python3 -c "
import sys, re
txt = sys.stdin.read()
vals = []
m = re.search(r'\[([0-9eE+.,\s inf-]+)\]', txt)   # flow: [v1, v2, ...](长数组会折行,字符类含 \s 跨行匹配)
if m:
    vals = [float(x) for x in m.group(1).split(',') if x.strip()]
else:
    for raw in txt.splitlines():                        # block: '- v' 逐行
        s = raw.strip()
        if s.startswith('- '):
            s = s[2:].strip()
        try:
            vals.append(float(s))
        except ValueError:
            pass
n = len(vals)
print(f'{vals[n//2]:.3f}' if n > 10 else 'NA')
")
    [ -n "$v" ] && [ "$v" != "NA" ] && { echo "$v"; return; }
    sleep 1
  done
  echo NA
}
RANGE=$(scan_range)
echo "/scan 正前方(角0)读数: ${RANGE}m(期望 5.75±${THR_RANGE_ERR})"
awk -v r="${RANGE:-0}" 'BEGIN{exit !(r>5.65 && r<5.85)}' && ok "障碍物测距 ${RANGE}m≈5.75m" || bad "障碍测距 ${RANGE:-NA}m 偏差超限"

# 相机非空:图像数据存在非零像素(排除空行,防 -v 假阳性)
timeout 8 ros2 topic echo /image_raw --field data 2>/dev/null | head -500 \
  | grep -m1 -q -v -e '^$' -e '^-\{0,1\} *0$' \
  && ok "/image_raw 非空图像" || bad "/image_raw 图像全零"

echo "===== [6/8] 直行与倒车(/clock 度量稳态速度) ====="
echo "-- 直行 0.5m/s × 10s:稳态窗口测速度(位移/仿真时长,RTF 与斜坡无关) --"
# 采样对称性:TA/TB 都在 gz_pose 之后取 —— simsec 各滞后采样点 ~1.2s,
# 两次滞后近似相等在 dt 中抵消(原 TA 在前/TB 在后,分母虚大 ~2s 是第 3 轮
# V=0.097 假阴性的根因)。drive 10s 覆盖整个采样序列(墙钟 ~9s)保证窗口
# 两端都在匀速段内;车行程 ~4.6m+车头 < 5.75m(box 前表面,已远置)
PA=$(gz_pose); read AX AY AZ AYAW <<< "$PA"
drive 0.5 0 10 &
DPID=$!
sleep 1.2                          # 跳过起步斜坡
PW=$(gz_pose); read WX1 WY1 WZ1 WYAW1 <<< "$PW"; TA=$(simsec)
sleep 1.5
PW=$(gz_pose); read WX2 WY2 WZ2 WYAW2 <<< "$PW"; TB=$(simsec)
wait $DPID
stop_car
PB=$(gz_pose); read BX BY BZ BYAW <<< "$PB"
V_MEAS=$(awk -v a="$WX1" -v b="$WX2" -v t1="${TA:-0}" -v t2="${TB:-0}" \
  'BEGIN{dt=t2-t1; printf "%.3f", (dt>0.5)?(b-a)/dt:-9}')
DTW=$(awk -v a="${TA:-0}" -v b="${TB:-0}" 'BEGIN{printf "%.2f", b-a}')
GZ_DX=$(awk -v a="$AX" -v b="$BX" 'BEGIN{printf "%.3f", b-a}')
GZ_DYAW=$(yaw_delta "$AYAW" "$BYAW")
echo "稳态窗口 Δt_sim=${DTW}s 速度实测=${V_MEAS}m/s(指令 0.5) | 全程 Δx=${GZ_DX}m Δyaw=${GZ_DYAW}rad"
POS=$(timeout 8 ros2 run tf2_ros tf2_echo odom base_footprint 2>/dev/null | grep -m1 "Translation")
X=$(echo "$POS" | grep -oP "Translation: \[\K-?[0-9.]+")
echo "odom→base_footprint: $POS"
# 容差 ±30%:窗口法的系统误差——TA/TB 各滞后采样点 ~1-2s 且不对称
# (残差 ~0.4s/窗口 2.4s),V 实测 0.60(虚高 20%)、ω 实测 0.215(偏低 14%)
# 均在第 7 轮实证;速度绝对精度由位移判据(odom/Gazebo 对账)保证
awk -v v="$V_MEAS" 'BEGIN{exit !(v>0.35 && v<0.65)}' && ok "直行稳态速度 ${V_MEAS}m/s≈0.5(/clock 度量,容差±30%)" || bad "直行稳态速度 ${V_MEAS}m/s 偏离 0.5"
awk -v x="${X:-0}" -v t="$THR_STRAIGHT" 'BEGIN{exit !(x>t)}' && ok "直行 odom 位移 x=${X}m>$THR_STRAIGHT" || bad "直行 odom 位移不足 x=${X}m"
awk -v d="${GZ_DX:-0}" -v t="$THR_STRAIGHT" 'BEGIN{exit !(d>t)}' && ok "直行 Gazebo 真实位移 Δx=${GZ_DX}m>$THR_STRAIGHT" || bad "直行真实位移不足 Δx=${GZ_DX}m"
DEV=$(awk -v x="${X:-0}" -v d="${GZ_DX:-0}" 'BEGIN{printf "%.3f", x-d}')
awk -v d="$DEV" -v t="$THR_STRAIGHT_DEV" 'BEGIN{exit !(d<t && d>-t)}' && ok "账面-真实偏差 ${DEV}m<$THR_STRAIGHT_DEV" || bad "账面-真实偏差 ${DEV}m≥$THR_STRAIGHT_DEV"
awk -v d="${GZ_DYAW:-0}" -v t="$THR_YAW_DRIFT" 'BEGIN{exit !(d<t && d>-t)}' && ok "直行 yaw 漂移 ${GZ_DYAW}rad<5°" || bad "直行 yaw 漂移 ${GZ_DYAW}rad≥5°"

echo "-- 倒车 -0.3m/s × 2s --"
PA=$(gz_pose); read AX AY AZ AYAW <<< "$PA"
drive -0.3 0 2
stop_car
PB=$(gz_pose); read BX BY BZ BYAW <<< "$PB"
GZ_DX=$(awk -v a="$AX" -v b="$BX" 'BEGIN{printf "%.3f", b-a}')
echo "倒车真实 Δx=${GZ_DX}m"
awk -v d="${GZ_DX:-0}" 'BEGIN{exit !(d<-0.35)}' && ok "倒车 Δx=${GZ_DX}m<-0.35" || bad "倒车未后退 Δx=${GZ_DX}m"

echo "===== [7/8] 左右转弯(不饱和)/停车/限位 ====="
echo "-- 左转 v=0.4 ω=0.25 × 10s(内轮理论角 0.247 < 限位 0.55,不饱和) --"
# ω=0.25 而非 0.3:全程 Δyaw ~2.4rad 必须远离 ±π——yaw_delta 的归一化
# 在 |Δyaw|>π 时把左转误判为反向(第 6 轮 ω=0.3×11s 转 3.17rad 实测翻车)
PA=$(gz_pose); read AX AY AZ AYAW <<< "$PA"
# get_js 串行放窗口前(第 7 轮实测后台重定向方式取不到值,第 5 轮此结构
# 实证取到 0.2950/0.2377);序列 1.2+1.5+3+1+3=9.7s ≤ drive 10s;
# wait 一律带 PID:裸 wait 会连 ros2 launch 后台一起等,永久挂死
drive 0.4 0.25 10 &
DPID=$!
sleep 1.2
JSV=$(get_js)
LS2=$(js_var LEFT_STEER); RS2=$(js_var RIGHT_STEER)
LRV=$(js_var LEFT_REAR_V); RRV=$(js_var RIGHT_REAR_V)
PW=$(gz_pose); read WX1 WY1 WZ1 WYAW1 <<< "$PW"; TA=$(simsec)
sleep 1.0
PW=$(gz_pose); read WX2 WY2 WZ2 WYAW2 <<< "$PW"; TB=$(simsec)
wait $DPID
stop_car
PB=$(gz_pose); read BX BY BZ BYAW <<< "$PB"
echo "转弯中前轮角: left=${LS2} right=${RS2}; 后轮速: left=${LRV} right=${RRV}"
W_MEAS=$(awk -v y1="$WYAW1" -v y2="$WYAW2" -v t1="${TA:-0}" -v t2="${TB:-0}" \
  'BEGIN{dt=t2-t1; d=y2-y1; if(d>3.14159)d-=6.28319; if(d<-3.14159)d+=6.28319; printf "%.3f", (dt>0.5)?d/dt:-9}')
DTW=$(awk -v a="${TA:-0}" -v b="${TB:-0}" 'BEGIN{printf "%.2f", b-a}')
DY=$(yaw_delta "$AYAW" "$BYAW")
echo "稳态窗口 Δt_sim=${DTW}s ω实测=${W_MEAS}rad/s(指令 0.25) | 全程Δyaw=${DY}rad"
awk -v w="$W_MEAS" 'BEGIN{exit !(w>0.15 && w<0.35)}' && ok "左转稳态角速度 ${W_MEAS}rad/s≈0.25(/clock 度量)" || bad "左转稳态角速度 ${W_MEAS}rad/s 偏离 0.25"
awk -v d="${DY:-0}" -v t="$THR_TURN" 'BEGIN{exit !(d>t)}' && ok "左转 Δyaw=+${DY}rad>$THR_TURN(与 ω 同号)" || bad "左转 Δyaw=${DY}rad 不足或反向"
awk -v l="${LS2:-0}" -v r="${RS2:-0}" -v t="$THR_STEER_DIFF" 'BEGIN{exit !(l-r>t)}' && ok "左转为内侧:左前角${LS2}>右前角${RS2}" || bad "内外轮角关系错(左转应左前>右前,轮序反?)"
awk -v l="${LRV:-0}" -v r="${RRV:-0}" -v t="$THR_WHEEL_DIFF" 'BEGIN{al=l<0?-l:l; ar=r<0?-r:r; exit !(ar-al>t)}' && ok "外侧右后轮速更快(|${RRV}|>|${LRV}|)" || bad "后轮速关系错(左转外侧=右后应更快)"

echo "-- 右转 v=0.4 ω=-0.25 × 5s --"
# drive 时长 5s+sleep 2.2:ros2 topic pub 无 daemon 冷启动 ~1.5s 吃掉指令窗口
# 前段,3.5s 窗口按 1.2s 采样会采到"指令尚未发出"的状态(第 4 轮实测转向 ~0)
PA=$(gz_pose); read AX AY AZ AYAW <<< "$PA"
drive 0.4 -0.25 5 &
DPID=$!
sleep 2.2
JSV=$(get_js)
LS2=$(js_var LEFT_STEER); RS2=$(js_var RIGHT_STEER); LRV=$(js_var LEFT_REAR_V); RRV=$(js_var RIGHT_REAR_V)
wait $DPID
stop_car
PB=$(gz_pose); read BX BY BZ BYAW <<< "$PB"
DY=$(yaw_delta "$AYAW" "$BYAW")
echo "右转真实Δyaw=${DY}rad;前轮角 left=${LS2} right=${RS2}"
awk -v d="${DY:-0}" 'BEGIN{exit !(d<-0.3)}' && ok "右转 Δyaw=${DY}rad<-0.3(与 ω 同号)" || bad "右转 Δyaw=${DY}rad 不足或反向"
awk -v l="${LS2:-0}" -v r="${RS2:-0}" -v t="$THR_STEER_DIFF" 'BEGIN{al=l<0?-l:l; ar=r<0?-r:r; exit !(ar-al>t)}' && ok "右转为内侧:|右前角${RS2}|>|左前角${LS2}|" || bad "内外轮角关系错(右转应右前绝对值更大)"
awk -v l="${LRV:-0}" -v r="${RRV:-0}" -v t="$THR_WHEEL_DIFF" 'BEGIN{al=l<0?-l:l; ar=r<0?-r:r; exit !(al-ar>t)}' && ok "外侧左后轮速更快(|${LRV}|>|${RRV}|)" || bad "后轮速关系错(右转外侧=左后应更快)"

echo "-- 超时停车(指令停止后 sim_wiring watchdog 自动发零速) --"
drive 0.3 0 2
# 故意不发零速:验证超时保护(0.5s 无指令 → 零速 → 减速停)
sleep 2
PA=$(gz_pose); read AX AY AZ AYAW <<< "$PA"
sleep 1.5
PB=$(gz_pose); read BX BY BZ BYAW <<< "$PB"
echo "超时停车采样: PA=($AX,$AY,z=$AZ,yaw=$AYAW) PB=($BX,$BY,z=$BZ,yaw=$BYAW)"
STILL=$(awk -v a="$AX" -v b="$BX" -v c="$AY" -v d="$BY" 'BEGIN{printf "%.4f", sqrt((b-a)^2+(d-c)^2)}')
awk -v s="$STILL" -v t="$THR_STILL" 'BEGIN{exit !(s<t)}' && ok "指令超时自动停车(1.5s 位移 ${STILL}m<$THR_STILL)" || bad "超时后仍移动 ${STILL}m(watchdog 未生效?)"

echo "-- 限位饱和 v=0.2 ω=0.8 × 3s(内轮理论角 1.45 > 限位 0.55,应饱和) --"
drive 0.2 0.8 3 &
DPID=$!
sleep 1.2
JSV=$(get_js)
LS2=$(js_var LEFT_STEER); RS2=$(js_var RIGHT_STEER)
wait $DPID
stop_car
echo "限位下前轮角: left=${LS2} right=${RS2}"
awk -v l="${LS2:-0}" -v m="$THR_STEER_LIM" 'BEGIN{exit !(l<m && l>-m)}' && ok "左前角饱和于限位内(|${LS2}|≤$THR_STEER_LIM)" || bad "左前角 ${LS2} 超限位"
awk -v r="${RS2:-0}" -v m="$THR_STEER_LIM" 'BEGIN{exit !(r<m && r>-m)}' && ok "右前角饱和于限位内(|${RS2}|≤$THR_STEER_LIM)" || bad "右前角 ${RS2} 超限位"

echo ""
echo "===== [8/8] 结果: PASS=$PASS FAIL=$FAIL ====="
if [ $FAIL -eq 0 ]; then echo "ALL_GREEN"; else echo "HAS_FAILURES"; fi
exit $FAIL
