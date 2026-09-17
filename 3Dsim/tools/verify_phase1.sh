#!/usr/bin/env bash
# =============================================================================
# 阶段1 验收脚本(headless)
# 用法(WSL 内): bash ~/hwcart_sim/tools/verify_phase1.sh
#               bash ~/hwcart_sim/tools/verify_phase1.sh --selftest-fail  # 失败路径退出码自检
# 构建证据由独立步骤提供: bash ~/hwcart_sim/tools/rebuild_racecar.sh(本脚本不含构建)
#
# 2026-09-14 返工第2轮(独立验收收尾清单) + 第3轮定稿(源码审查 3 问题):
#   - 清理只针对本次进程组:launch 用 setsid 启动(独立进程组,PGID=launch PID),
#     cleanup 只向 -PGID 发 TERM/KILL;端口 11345 已占用直接 exit 2 不代杀;
#     删除 pkill -f / PID 快照差集(无法证明进程归属本次测试,可能误杀他方)
#   - 关节检查解析真实消息:N=6 关节名齐全(含前轮自转 spin joint)、
#     position/velocity 数组长度与 name 一致、值全部有限——不再数固定输出的行数
#   - 稳态速度同源度量:
#     ① 瞬时:控制器 /ackermann_steering_controller/odometry 同一条消息的
#        twist(header.stamp 与 twist 同源) → ±10% 紧判据
#     ② 全程:录制 odometry 消息流,取首条与"运动收敛点"(相邻消息位移<1mm)
#        两条消息的 stamp 差与位姿差——停车后的读窗不进分母,无采样拼接;
#        odom 位姿为账面,物理真实性由 Gazebo 世界系位移对账判据独立保证
#     (旧"两次订阅拼 sec/nanosec + 窗口差分 ±30% 容差"已删除)
#   - echo 输出过滤:QoS 警告会混入 stdout 且与 --- 粘连(tab 缩进炸 YAML),
#     解析一律 try/except,缺数据显式失败,空值判据前置非空检查
#   - 工况:零指令静止/直行/倒车/左右转弯(不饱和)/零速停车/限位饱和/
#     已知障碍物测距/相机非空图像/频率容差/时间戳有效/光学 frame TF
#   - 2026-09-15 第4轮(启动竞态修复):①launch 侧 spawn_entity.py 加
#     -timeout 60(内部服务等待默认 30s,WSL 下偶发不够);②gz_pose 测量
#     重试×3(第1轮唯一 FAIL=gz 服务单次无响应,非车辆问题,阈值未动);
#     ③precheck 增加残留 gzserver 检测(不代杀,与端口占用同策略)
#   - 2026-09-15 第5次加固(外部AI审查建议1/2):①启动段显式记录 spawn
#     就绪等待时长,启动失败自动重试 1 次(证据留痕 /tmp/sim_test*.log);
#     ②gz_pose 逐次尝试留痕(次数/失败类型),3 次皆败明确 FAIL
#   - 阈值集中在 THR_* 变量(负例 tools/verify_negative.sh 用 sed 注入)
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

# joint_states 必须齐全的 6 关节(= URDF ros2_control 块注册的全部:
# 2 转向 + 2 后驱 + 2 前轮自转 state-only;负例注入点:改坏任一名 → MISSING FAIL)
EXP_JOINTS="left_steering_joint right_steering_joint left_rear_wheel_joint right_rear_wheel_joint left_front_wheel_spin_joint right_front_wheel_spin_joint"

# ---- 失败路径退出码自检(负例之一,秒回) ----
if [ "$1" = "--selftest-fail" ]; then
  bad "故意失败(失败路径退出码自检)"
  echo "===== 结果: PASS=$PASS FAIL=$FAIL ====="
  echo "HAS_FAILURES"
  exit $FAIL
fi

# ---- 进程管理:只针对本次启动的进程组 ----
# launch 用 setsid 起在独立进程组(PGID=launch PID),gzserver/spawner/sim_wiring
# 全是该组子孙;cleanup 向 -PGID 发信号即精确覆盖本次全部进程,不碰组外任何进程
LAUNCH_PID=""

precheck() {
  # 端口被占一律退出,不代杀:占用者可能是别的会话正在跑的 Gazebo,
  # 脚本无法证明其归属,自动 kill 有误杀风险(2026-09-14 源码审查问题1)
  if ss -ltn 2>/dev/null | grep -q 11345; then
    echo "[预检] 端口 11345 已被占用,本脚本只管理自己启动的进程组,不代杀:"
    ss -ltnp 2>/dev/null | grep 11345
    echo "[预检] 请手动确认后清理(如为残留: kill -9 <上面查到的 PID>),再重试"
    exit 2
  fi
  # 残留 gzserver:未监听 11345 的僵尸也占 /dev/shm 与 CPU,会拖慢甚至阻塞
  # 下一轮 factory 插件起服(2026-09-15 第2轮启动失败的嫌疑环境因素)。
  # 与端口占用同策略:只报告不代杀
  if pgrep -x gzserver >/dev/null 2>&1; then
    echo "[预检] 检测到已运行的 gzserver(可能为上轮残留):"
    pgrep -ax gzserver
    echo "[预检] 请手动确认后清理(kill 对应 PID),再重试"
    exit 2
  fi
}

cleanup() {
  # 仅向本次进程组发信号:先 TERM 优雅关停(ros2 launch 会关停自己的子进程),
  # 超时再整组 KILL(组长已死时 -PGID 仍命中组内孤儿,如 gzserver)
  if [ -n "$LAUNCH_PID" ]; then
    kill -TERM -- "-$LAUNCH_PID" 2>/dev/null
    for i in 1 2 3 4 5 6 7 8; do
      kill -0 -- "-$LAUNCH_PID" 2>/dev/null || break
      sleep 1
    done
    kill -KILL -- "-$LAUNCH_PID" 2>/dev/null
  fi
  for i in 1 2 3 4 5; do ss -ltn 2>/dev/null | grep -q 11345 || break; sleep 1; done
}
trap cleanup EXIT
precheck

# ---- 通用采集函数 ----
_gz_pose_once() {  # Gazebo 世界位姿 → "px py pz yaw"(单次;服务无响应时 PARSE_FAIL)
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
gz_pose() {  # 测量鲁棒性(第4轮;第5次加固=外部AI审查建议2:逐次留痕)。
  # /get_entity_state 服务偶发无响应(第1轮唯一 FAIL 的根因,非车辆问题)。
  # 重试 3 次、间隔 1s,仅防测量单次抽风,判据阈值不变;每次尝试(含失败
  # 类型)均记录,3/3 皆败输出 PARSE_FAIL(=FAIL,不跳过、不用估算值替代)
  local out i
  for i in 1 2 3; do
    out=$(_gz_pose_once)
    if [ -n "$out" ] && [ "$out" != "PARSE_FAIL" ]; then
      [ "$i" -gt 1 ] && echo "[gz_pose] 尝试${i}/3 成功(前$((i-1))次失败已留痕)" >&2
      echo "$out"; return 0
    fi
    echo "[gz_pose] 尝试${i}/3 无响应(返回:${out:-空})" >&2
    sleep 1
  done
  echo "[gz_pose] 3/3 均失败,按取不到证据即FAIL口径输出 PARSE_FAIL" >&2
  echo "${out:-PARSE_FAIL}"
}
odom_twist() {  # 控制器 odometry 同一条消息的瞬时 twist 标量:
  # $1 = linear.x | angular.z。header.stamp 与 twist 同源同消息,无采样拼接
  local v
  v=$(timeout 8 ros2 topic echo /ackermann_steering_controller/odometry \
        --field "twist.twist.$1" 2>/dev/null | grep -m1 -E '^-?[0-9.]+$')
  echo "${v:-NA}"
}
odom_stream() {  # 后台录制 odometry 消息流到文件(运动全程,含起步前/停稳后)
  timeout 30 ros2 topic echo /ackermann_steering_controller/odometry \
    2>/dev/null > "$1"
}
odom_vw() {  # 解析录制文件 → "T1 X1 YAW1 T2 X2 YAW2 V_AVG W_AVG"
  # T1/X1=首条消息;T2/X2=运动收敛点(与下一条相邻位移<1mm 的首条)——
  # 停车后的读窗不计入分母;V 用 x 分量(直行工况),W 用归一化 yaw 差
  python3 - "$1" <<'PYEOF'
import sys, re, math, yaml
txt = open(sys.argv[1]).read()
msgs = []
for d in re.split(r'(?m)^---\s*$', txt):
    d = d.strip()
    if 'header' not in d:
        continue
    try:
        m = yaml.safe_load(d)
        t = m['header']['stamp']['sec'] + m['header']['stamp']['nanosec'] / 1e9
        p = m['pose']['pose']['position']
        q = m['pose']['pose']['orientation']
        yaw = math.atan2(2*(q['w']*q['z']+q['x']*q['y']), 1-2*(q['y']**2+q['z']**2))
        msgs.append((t, p['x'], p['y'], yaw))
    except Exception:
        continue   # QoS 警告混入 stdout 的坏文档跳过
if len(msgs) < 10:
    print('NA NA NA NA NA NA NA NA'); sys.exit(0)
t1, x1, y1, yaw1 = msgs[0]
end_i = len(msgs) - 1
for i in range(len(msgs) - 1):
    if math.hypot(msgs[i+1][1]-msgs[i][1], msgs[i+1][2]-msgs[i][2]) > 0.001:
        end_i = i + 1
t2, x2, y2, yaw2 = msgs[end_i]
dt = t2 - t1
v = (x2 - x1) / dt if dt > 0.5 else -9
dyaw = yaw2 - yaw1
if dyaw > math.pi: dyaw -= 2 * math.pi
if dyaw < -math.pi: dyaw += 2 * math.pi
w = dyaw / dt if dt > 0.5 else -9
print('%.3f %.4f %.4f %.3f %.4f %.4f %.3f %.3f' % (t1, x1, yaw1, t2, x2, yaw2, v, w))
PYEOF
}
yaw_delta() {  # 归一化角差
  awk -v a="$1" -v b="$2" 'BEGIN{d=b-a; if(d>3.14159)d-=6.28319; if(d<-3.14159)d+=6.28319; printf "%.4f", d}'
}
drive() {  # v w 时长s
  timeout "$3" ros2 topic pub -r 20 /cmd_vel geometry_msgs/msg/Twist \
    "{linear: {x: $1}, angular: {z: $2}}" >/dev/null 2>&1
}
stop_car() { drive 0 0 1; }
get_js() {  # joint_states 首条消息 → 单行 TSV(9 字段,bash 一次 read 解包):
  # N POS_LEN VEL_LEN MISSING NONFINITE LEFT_STEER RIGHT_STEER LEFT_REAR_V RIGHT_REAR_V
  # 解析真实消息——关节数/数组长度/缺失关节/非有限值全部暴露;单行原子输出,
  # 多行 print 存在偶发中间行丢失(第 10 轮 VEL_LEN 消失),单行结构上不可能。
  # awk 从 header:/name: 行起放行:QoS 警告会混入 stdout 且与 --- 粘连
  # (tab 缩进炸 YAML),不能从头直通
  timeout 8 ros2 topic echo /joint_states 2>/dev/null \
    | awk '/^(header|name):/{flag=1} flag && /^---$/{exit} flag{print}' \
    | python3 -c "
import sys, math, yaml
try:
    d = yaml.safe_load(sys.stdin) or {}
except Exception:
    d = {}
names = list(d.get('name') or [])
pos = list(d.get('position') or [])
vel = list(d.get('velocity') or [])
exp = '''$EXP_JOINTS'''.split()
missing = [j for j in exp if j not in names]
bad = [j for j, v in zip(names, pos) if not math.isfinite(v)]
bad += [j for j, v in zip(names, vel) if not math.isfinite(v)]
def g(n, arr):
    return arr[names.index(n)] if n in names and len(arr) == len(names) else 'NA'
print('\t'.join(str(x) for x in [
    len(names), len(pos), len(vel),
    ','.join(missing) if missing else 'ok',
    ','.join(bad) if bad else 'ok',
    g('left_steering_joint', pos), g('right_steering_joint', pos),
    g('left_rear_wheel_joint', vel), g('right_rear_wheel_joint', vel)]))
"
}
js_ok() { [ -n "$1" ] && [ "$1" != "NA" ]; }  # 数值字段非空且非 NA
pose_ok() { case "$1" in ""|*FAIL*) false;; *) true;; esac; }  # gz_pose 结果有效

echo "===== [1/8] 一键启动(use_gui:=false) ====="
# setsid:launch 及其全部子孙(gzserver/spawner/sim_wiring)落进独立进程组,
# PGID=$LAUNCH_PID;cleanup 的 -PGID 信号只覆盖这组,不碰组外进程
# 2026-09-15 第5次加固(外部AI审查建议1):显式记录 spawn 就绪等待时长;
# 启动失败自动重试 1 次(两次证据均留痕;判据不变,重试仅限启动段)
bringup_once() {  # 轮询启动征兆:成功→0;失败征兆/超时→1(时长与原因记录在案)
  SPAWN_WAIT="NA"; SPAWN_FAIL_REASON="无输出"
  local t0=$SECONDS i
  for i in $(seq 1 40); do            # 2s×40=80s 上限(> spawn -timeout 60 + 链尾)
    if grep -q "SpawnEntity: Successfully spawned entity" /tmp/sim_test.log 2>/dev/null; then
      SPAWN_WAIT=$(( SECONDS - t0 )); SPAWN_FAIL_REASON=""; return 0
    fi
    if grep -q "终止启动链" /tmp/sim_test.log 2>/dev/null; then
      SPAWN_WAIT=$(( SECONDS - t0 )); SPAWN_FAIL_REASON="启动链事件终止"; return 1
    fi
    sleep 2
  done
  SPAWN_WAIT=$(( SECONDS - t0 )); SPAWN_FAIL_REASON="80s 内未见证据"; return 1
}
setsid ros2 launch racecar_sim sim_bringup.launch.py use_gui:=false > /tmp/sim_test.log 2>&1 &
LAUNCH_PID=$!
if bringup_once; then
  echo "[启动] 第 1 次尝试:spawn 就绪等待 ${SPAWN_WAIT}s(launch 起算)"
else
  echo "[启动] 第 1 次尝试失败(${SPAWN_FAIL_REASON},等待 ${SPAWN_WAIT}s);首试证据留存 /tmp/sim_test_attempt1.log,按外部AI审查建议自动重试 1 次"
  cp /tmp/sim_test.log /tmp/sim_test_attempt1.log
  cleanup; sleep 3
  setsid ros2 launch racecar_sim sim_bringup.launch.py use_gui:=false > /tmp/sim_test.log 2>&1 &
  LAUNCH_PID=$!
  if bringup_once; then
    echo "[启动] 第 2 次尝试成功:spawn 就绪等待 ${SPAWN_WAIT}s(首试失败证据在 /tmp/sim_test_attempt1.log)"
  else
    echo "[启动] 第 2 次尝试仍失败(${SPAWN_FAIL_REASON},等待 ${SPAWN_WAIT}s)"
  fi
fi
sleep 20                               # spawn 后控制器/传感器稳定窗(与原 sleep 35 合计口径相当)
if grep -q "SpawnEntity: Successfully spawned entity" /tmp/sim_test.log; then ok "spawn 成功"; else bad "spawn 失败(见 /tmp/sim_test.log;首试证据 /tmp/sim_test_attempt1.log)"; fi
grep -q "parser error" /tmp/sim_test.log && bad "gazebo_ros2_control 参数注入错误" || ok "无 parser error(注释剥离生效)"
# 启动链失败传播:事件链任一环失败应 Shutdown 并写明原因
grep -q "终止启动链" /tmp/sim_test.log && bad "启动链中途失败(launch 已按退出码终止)" || ok "启动链各环退出码全零"

echo "===== [2/8] 控制器激活 ====="
CTRL=$(timeout 20 ros2 control list_controllers 2>/dev/null)
echo "$CTRL"
echo "$CTRL" | grep -q "joint_state_broadcaster.*active" && ok "joint_state_broadcaster active" || bad "joint_state_broadcaster 未激活"
echo "$CTRL" | grep -q "ackermann_steering_controller.*active" && ok "ackermann_steering_controller active" || bad "ackermann 未激活"
# 关节完整性:解析真实消息——6 关节名齐全(含前轮自转)、数组长度一致、值有限
JSV=$(get_js)
IFS=$'\t' read -r NJS PLEN VLEN MISS NFJ _ _ _ _ <<< "$JSV"
echo "joint_states: N=${NJS:-NA} POS_LEN=${PLEN:-NA} VEL_LEN=${VLEN:-NA} MISSING=${MISS:-NA} NONFINITE=${NFJ:-NA}"
[ "${NJS:-NA}" = "NA" ] && bad "joint_states 取不到消息(echo 无输出或解析失败)"
[ "$NJS" = "6" ] && ok "joint_states 含 6 关节(4 控制关节 + 2 前轮自转)" || bad "joint_states 关节数=${NJS:-取不到}(应 6)"
[ "$PLEN" = "6" ] && [ "$VLEN" = "6" ] && ok "position/velocity 数组长度=6 与关节名一致" || bad "数组长度不符(POS=${PLEN:-NA} VEL=${VLEN:-NA} 应 6)"
[ "$MISS" = "ok" ] && ok "6 关节名齐全(含前轮自转 spin joint)" || bad "joint_states 缺关节: ${MISS:-NA}"
[ "$NFJ" = "ok" ] && ok "位置/速度值全部有限" || bad "joint_states 非有限值: ${NFJ:-NA}"

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
# (z 前 x 右 y 下;正前方物体反投影方向在 base 中 +x)。
# 匹配失败(无输出)显式失败——空输入进 awk 会静默通过(源码审查问题4)
RPY_LINE=$(timeout 8 ros2 run tf2_ros tf2_echo base_footprint camera_optical 2>/dev/null | grep -m1 -i "radian")
echo "camera_optical:    $RPY_LINE"
RPY3=$(echo "$RPY_LINE" | grep -oP '\[\K-?[0-9.]+, *-?[0-9.]+, *-?[0-9.]+')
if [ -z "$RPY3" ]; then
  bad "camera_optical rpy 取不到数据(tf2_echo 无 Rotation 输出)"
else
  echo "$RPY3" | awk -F', *' '{
    r=$1+0; p=$2+0; y=$3+0
    if (r<-1.591 || r>-1.551) exit 1
    if (p<-0.02 || p>0.02) exit 1
    if (y<-1.591 || y>-1.551) exit 1
  }' && ok "camera_optical rpy=(-π/2,0,-π/2) 光学约定" || bad "camera_optical 旋转不对(应 -π/2 0 -π/2)"
fi

echo "===== [5/8] 静止/测距/图像(车未动) ====="
PA=$(gz_pose); read AX AY AZ AYAW <<< "$PA"
sleep 2
PB=$(gz_pose); read BX BY BZ BYAW <<< "$PB"
echo "静止 2s: ($AX,$AY,z=$AZ) → ($BX,$BY,z=$BZ)"
STILL=$(awk -v a="$AX" -v b="$BX" -v c="$AY" -v d="$BY" 'BEGIN{printf "%.4f", sqrt((b-a)^2+(d-c)^2)}')
if pose_ok "$PA" && pose_ok "$PB"; then
  awk -v s="$STILL" -v t="$THR_STILL" 'BEGIN{exit !(s<t)}' && ok "零指令静止(位移 ${STILL}m<$THR_STILL)" || bad "零指令漂移 ${STILL}m"
else
  bad "静止工况取不到 Gazebo 位姿(PARSE_FAIL=服务无响应)"
fi
# 接地检查:静止时 base_footprint(实体原点)世界 z ≈ 0(轮心 WR-CZ 修正的
# 直接证据;GPT:不要只看静态 TF 的 z 数字)
awk -v z="${AZ:-9}" 'BEGIN{exit !(z>-0.05 && z<0.05)}' && ok "接地:静止时 z=${AZ}m≈0(footprint 贴地)" || bad "接地异常 z=${AZ}m(轮心高度错?)"

# 已知障碍物:world 内 marker_box(中心 x=6)前表面距原点 5.75m → /scan 正前方读数
# (echo --field 大数组输出是单行 flow 风格 [v1, v2, ...];--- 分隔符与警告行丢弃)
scan_range() {
  local v
  for try in 1 2 3; do
    v=$(timeout 12 ros2 topic echo /scan --field ranges 2>/dev/null | python3 -c "
import sys, re
txt = sys.stdin.read()
vals = []
m = re.search(r'\[([0-9eE+.,\s inf-]+)\]', txt)   # flow: [v1, v2, ...](字符类含 \s 跨行匹配)
if m:
    vals = [float(x) for x in m.group(1).split(',') if x.strip()]
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

# 相机非空:解析消息 data 数组本身——数值个数与最大值;警告行/--- 分隔符
# 不是合法数值会被解析拒绝,不再用 grep 行匹配(源码审查问题4)
IMG_VERDICT=$(timeout 25 ros2 topic echo /image_raw --field data 2>/dev/null | python3 -c "
import sys, re
txt = sys.stdin.read()
m = re.search(r'\[([^\]]*)\]', txt)   # 首条消息的数组字面量(大数组为 repr 单行)
n = 0; vmax = -1
if m:
    for tok in m.group(1).split(','):
        tok = tok.strip()
        if not tok:
            continue
        try:
            x = int(tok)
        except ValueError:
            continue
        n += 1
        if x > vmax: vmax = x
print('IMG_OK n=%d max=%d' % (n, vmax) if n >= 1000 and vmax > 0 else 'IMG_BAD n=%d max=%d' % (n, vmax))
")
echo "/image_raw data: ${IMG_VERDICT}(期望 n≈921600=640×480×3)"
case "$IMG_VERDICT" in
  IMG_OK*) ok "/image_raw 非空图像(${IMG_VERDICT#IMG_OK })" ;;
  *)       bad "/image_raw 图像数据异常: ${IMG_VERDICT:-取不到}" ;;
esac

echo "===== [6/8] 直行与倒车(同消息稳态度量) ====="
echo "-- 直行 0.5m/s × 10s --"
# 稳态速度双证据(全部同源,无采样拼接):
#   ① 瞬时:odometry 同一条消息的 twist(header.stamp 与 twist 同源) → ±10%
#   ② 全程:录制 odometry 消息流,首条 ↔ 运动收敛点(相邻位移<1mm)的
#      stamp 差与 x 差——停车后读窗不进分母。odom 位姿是账面,
#      物理真实性由下面的 Gazebo 世界系位移判据独立保证
PA=$(gz_pose); read AX AY AZ AYAW <<< "$PA"
odom_stream /tmp/odom_str.txt & OPID=$!
sleep 1                            # 订阅建立(~1.2s)完成于车动之前
drive 0.5 0 10 &
DPID=$!
sleep 2.5                          # pub 冷启动 ~1.5s + 起步斜坡,进入稳态
V_INST=$(odom_twist linear.x)
wait $DPID
stop_car
sleep 1.5                          # watchdog 0.5s + 减速,确保流里含停稳后的消息
kill $OPID 2>/dev/null; wait $OPID 2>/dev/null
read _ _ _ _ OX2 _ V_AVG _ <<< "$(odom_vw /tmp/odom_str.txt)"
rm -f /tmp/odom_str.txt
PB=$(gz_pose); read BX BY BZ BYAW <<< "$PB"
echo "稳态瞬时 V=${V_INST}m/s | odom 全程平均 V=${V_AVG}m/s(收敛点法) | 指令 0.5"
awk -v v="${V_INST:-NA}" 'BEGIN{exit !(v>0.45 && v<0.55)}' && ok "直行稳态瞬时速度 ${V_INST}m/s≈0.5(odometry 同消息 twist,±10%)" || bad "直行稳态瞬时速度 ${V_INST}m/s 偏离 0.5 超过±10%"
awk -v v="${V_AVG:-NA}" 'BEGIN{exit !(v>0.40 && v<0.60)}' && ok "直行全程平均速度 ${V_AVG}m/s(odom stamp 同源,含斜坡损耗)" || bad "直行全程平均速度 ${V_AVG}m/s 异常"
if pose_ok "$PA" && pose_ok "$PB"; then
  GZ_DX=$(awk -v a="$AX" -v b="$BX" 'BEGIN{printf "%.3f", b-a}')
  GZ_DYAW=$(yaw_delta "$AYAW" "$BYAW")
  echo "Gazebo 全程 Δx=${GZ_DX}m Δyaw=${GZ_DYAW}rad"
  awk -v d="${GZ_DX:-0}" 'BEGIN{exit !(d>3.5 && d<5.5)}' && ok "直行 Gazebo 全程位移 Δx=${GZ_DX}m(0.5×10s 扣冷启动/斜坡)" || bad "直行 Gazebo 全程位移 Δx=${GZ_DX}m 异常"
else
  GZ_DX=""; GZ_DYAW=""
  bad "直行工况取不到 Gazebo 位姿(位移/偏差/漂移判据跳过)"
fi
POS=$(timeout 8 ros2 run tf2_ros tf2_echo odom base_footprint 2>/dev/null | grep -m1 "Translation")
X=$(echo "$POS" | grep -oP "Translation: \[\K-?[0-9.]+")
echo "odom→base_footprint: $POS"
awk -v x="${X:-0}" -v t="$THR_STRAIGHT" 'BEGIN{exit !(x>t)}' && ok "直行 odom 位移 x=${X}m>$THR_STRAIGHT" || bad "直行 odom 位移不足 x=${X}m"
if [ -n "$GZ_DX" ] && [ -n "$X" ]; then
  awk -v x="$X" -v d="$GZ_DX" -v t="$THR_STRAIGHT_DEV" 'BEGIN{exit !((x-d<t)&&(x-d>-t))}' && ok "账面-真实偏差 $(awk -v x="$X" -v d="$GZ_DX" 'BEGIN{printf "%.3f", x-d}')m<$THR_STRAIGHT_DEV" || bad "账面-真实偏差 $(awk -v x="$X" -v d="$GZ_DX" 'BEGIN{printf "%.3f", x-d}')m≥$THR_STRAIGHT_DEV"
fi
if [ -n "$GZ_DYAW" ]; then
  awk -v d="$GZ_DYAW" -v t="$THR_YAW_DRIFT" 'BEGIN{exit !(d<t && d>-t)}' && ok "直行 yaw 漂移 ${GZ_DYAW}rad<5°" || bad "直行 yaw 漂移 ${GZ_DYAW}rad≥5°"
fi

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
# 在 |Δyaw|>π 时把左转误判为反向(实测 ω=0.3×11s 转 3.17rad 翻车)
# 度量同直行:① odometry 同消息 twist 瞬时 ω;② 收敛点法全程平均 ω;
# 转弯中的内外轮角/后轮速由 get_js 串行采样(稳态段内)
PA=$(gz_pose); read AX AY AZ AYAW <<< "$PA"
odom_stream /tmp/odom_turn.txt & OPID=$!
sleep 1
drive 0.4 0.25 10 &
DPID=$!
sleep 2.5
W_INST=$(odom_twist angular.z)
JSV=$(get_js)
IFS=$'\t' read -r _ _ _ _ _ LS2 RS2 LRV RRV <<< "$JSV"
wait $DPID
stop_car
sleep 1.5
kill $OPID 2>/dev/null; wait $OPID 2>/dev/null
read _ _ _ _ _ _ _ W_AVG <<< "$(odom_vw /tmp/odom_turn.txt)"
rm -f /tmp/odom_turn.txt
PB=$(gz_pose); read BX BY BZ BYAW <<< "$PB"
echo "转弯中前轮角: left=${LS2:-NA} right=${RS2:-NA}; 后轮速: left=${LRV:-NA} right=${RRV:-NA}"
DY=$(yaw_delta "$AYAW" "$BYAW")
echo "稳态瞬时 ω=${W_INST}rad/s | odom 全程平均 ω=${W_AVG}rad/s(收敛点法) | Gazebo 全程 Δyaw=${DY}rad | 指令 0.25"
awk -v w="${W_INST:-NA}" 'BEGIN{exit !(w>0.225 && w<0.275)}' && ok "左转稳态瞬时角速度 ${W_INST}rad/s≈0.25(odometry 同消息 twist,±10%)" || bad "左转稳态瞬时角速度 ${W_INST}rad/s 偏离 0.25 超过±10%"
awk -v w="${W_AVG:-NA}" 'BEGIN{exit !(w>0.18 && w<0.32)}' && ok "左转全程平均角速度 ${W_AVG}rad/s(odom stamp 同源)" || bad "左转全程平均角速度 ${W_AVG}rad/s 异常"
awk -v d="${DY:-0}" -v t="$THR_TURN" 'BEGIN{exit !(d>t)}' && ok "左转 Δyaw=+${DY}rad>$THR_TURN(与 ω 同号)" || bad "左转 Δyaw=${DY}rad 不足或反向"
if js_ok "$LS2" && js_ok "$RS2"; then
  awk -v l="$LS2" -v r="$RS2" -v t="$THR_STEER_DIFF" 'BEGIN{exit !(l-r>t)}' && ok "左转为内侧:左前角${LS2}>右前角${RS2}" || bad "内外轮角关系错(左转应左前>右前,轮序反?)"
else
  bad "左转取不到前轮角(left=${LS2:-空} right=${RS2:-空})"
fi
if js_ok "$LRV" && js_ok "$RRV"; then
  awk -v l="$LRV" -v r="$RRV" -v t="$THR_WHEEL_DIFF" 'BEGIN{al=l<0?-l:l; ar=r<0?-r:r; exit !(ar-al>t)}' && ok "外侧右后轮速更快(|${RRV}|>|${LRV}|)" || bad "后轮速关系错(左转外侧=右后应更快)"
else
  bad "左转取不到后轮速(left=${LRV:-空} right=${RRV:-空})"
fi

echo "-- 右转 v=0.4 ω=-0.25 × 5s --"
# drive 时长 5s+sleep 2.2:ros2 topic pub 无 daemon 冷启动 ~1.5s 吃掉指令窗口
# 前段,3.5s 窗口按 1.2s 采样会采到"指令尚未发出"的状态(实测转向 ~0)
PA=$(gz_pose); read AX AY AZ AYAW <<< "$PA"
drive 0.4 -0.25 5 &
DPID=$!
sleep 2.2
JSV=$(get_js)
IFS=$'\t' read -r _ _ _ _ _ LS2 RS2 LRV RRV <<< "$JSV"
wait $DPID
stop_car
PB=$(gz_pose); read BX BY BZ BYAW <<< "$PB"
DY=$(yaw_delta "$AYAW" "$BYAW")
echo "右转真实Δyaw=${DY}rad;前轮角 left=${LS2:-NA} right=${RS2:-NA}"
awk -v d="${DY:-0}" 'BEGIN{exit !(d<-0.3)}' && ok "右转 Δyaw=${DY}rad<-0.3(与 ω 同号)" || bad "右转 Δyaw=${DY}rad 不足或反向"
if js_ok "$LS2" && js_ok "$RS2"; then
  awk -v l="$LS2" -v r="$RS2" -v t="$THR_STEER_DIFF" 'BEGIN{al=l<0?-l:l; ar=r<0?-r:r; exit !(ar-al>t)}' && ok "右转为内侧:|右前角${RS2}|>|左前角${LS2}|" || bad "内外轮角关系错(右转应右前绝对值更大)"
else
  bad "右转取不到前轮角(left=${LS2:-空} right=${RS2:-空})"
fi
if js_ok "$LRV" && js_ok "$RRV"; then
  awk -v l="$LRV" -v r="$RRV" -v t="$THR_WHEEL_DIFF" 'BEGIN{al=l<0?-l:l; ar=r<0?-r:r; exit !(al-ar>t)}' && ok "外侧左后轮速更快(|${LRV}|>|${RRV}|)" || bad "后轮速关系错(右转外侧=左后应更快)"
else
  bad "右转取不到后轮速(left=${LRV:-空} right=${RRV:-空})"
fi

echo "-- 超时停车(指令停止后 sim_wiring watchdog 自动发零速) --"
drive 0.3 0 2
# 故意不发零速:验证超时保护(0.5s 无指令 → 零速 → 减速停)
sleep 2
PA=$(gz_pose); read AX AY AZ AYAW <<< "$PA"
sleep 1.5
PB=$(gz_pose); read BX BY BZ BYAW <<< "$PB"
echo "超时停车采样: PA=($AX,$AY,z=$AZ,yaw=$AYAW) PB=($BX,$BY,z=$BZ,yaw=$BYAW)"
if pose_ok "$PA" && pose_ok "$PB"; then
  STILL=$(awk -v a="$AX" -v b="$BX" -v c="$AY" -v d="$BY" 'BEGIN{printf "%.4f", sqrt((b-a)^2+(d-c)^2)}')
  awk -v s="$STILL" -v t="$THR_STILL" 'BEGIN{exit !(s<t)}' && ok "指令超时自动停车(1.5s 位移 ${STILL}m<$THR_STILL)" || bad "超时后仍移动 ${STILL}m(watchdog 未生效?)"
else
  bad "超时停车工况取不到 Gazebo 位姿(PARSE_FAIL=服务无响应)"
fi

echo "-- 限位饱和 v=0.2 ω=0.8 × 3s(内轮理论角 1.45 > 限位 0.55,应饱和) --"
drive 0.2 0.8 3 &
DPID=$!
sleep 1.2
JSV=$(get_js)
IFS=$'\t' read -r _ _ _ _ _ LS2 RS2 _ _ <<< "$JSV"
wait $DPID
stop_car
echo "限位下前轮角: left=${LS2:-NA} right=${RS2:-NA}"
if js_ok "$LS2"; then
  awk -v l="$LS2" -v m="$THR_STEER_LIM" 'BEGIN{exit !(l<m && l>-m)}' && ok "左前角饱和于限位内(|${LS2}|≤$THR_STEER_LIM)" || bad "左前角 ${LS2} 超限位"
else
  bad "限位工况取不到左前角(${LS2:-空})"
fi
if js_ok "$RS2"; then
  awk -v r="$RS2" -v m="$THR_STEER_LIM" 'BEGIN{exit !(r<m && r>-m)}' && ok "右前角饱和于限位内(|${RS2}|≤$THR_STEER_LIM)" || bad "右前角 ${RS2} 超限位"
else
  bad "限位工况取不到右前角(${RS2:-空})"
fi

echo ""
echo "===== [8/8] 结果: PASS=$PASS FAIL=$FAIL ====="
if [ $FAIL -eq 0 ]; then echo "ALL_GREEN"; else echo "HAS_FAILURES"; fi
exit $FAIL
