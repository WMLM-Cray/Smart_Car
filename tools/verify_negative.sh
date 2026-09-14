#!/usr/bin/env bash
# =============================================================================
# 端到端负例套件(收尾清单要求:补故意失败的测试验证退出码;源码审查后扩充为
# 三类注入,覆盖"阈值不可能值/缺关节/缺消息"三种失败模式)
# 原理:sed 对 verify_phase1.sh 注入缺陷 → 跑完整验收 → 断言
#      ① 对应判据出现 [FAIL]  ② 输出含 HAS_FAILURES  ③ 退出码非零
# 用法(WSL 内): bash ~/hwcart_sim/tools/verify_negative.sh [序号]
#   不带参数 = 全跑(3 个注入 × ~3min);带 1/2/3 = 只跑对应注入
# 另有秒回自检: bash ~/hwcart_sim/tools/verify_phase1.sh --selftest-fail
#
# ⚠ 连续注入的环境残留(2026-09-14 实测):verify_phase1.sh 的 cleanup 用 KILL
#   兜底杀 gzserver 后,WSL 内会残留 fastrtps 共享内存段(/dev/shm/fastrtps_*),
#   且 WSLg 音频代理坏态无法在 WSL 内清理——下一次 launch 的 /spawn_entity 服务
#   30s 注册不上 → 全盘 FAIL(失败签名:spawn_entity Service unavailable +
#   gzserver "could not create service: rcl node's context is invalid" abort)。
#   软复位(ros2 daemon stop + rm /dev/shm/fastrtps_*)不保证有效;
#   **干净证据的跑法 = Windows 侧每个注入前 `wsl --shutdown` 再单注入**:
#     for i in 1 2 3; do wsl.exe --shutdown; sleep 8;
#       wsl.exe -d ubuntu2204-ros2 -- bash ~/hwcart_sim/tools/verify_negative.sh $i
#     done
#   判读注入轮证据时先看对应 verify_neg_N.out 里 spawn 是否成功——
#   spawn 失败的注入轮断言仍会"过",但失败面是环境而非注入本身
# =============================================================================
SRC=~/hwcart_sim/tools/verify_phase1.sh

# 注入定义:sed 表达式 + 判据特征串(注入后必须出现的 FAIL 行关键词)+ 说明
inject_and_run() {  # $1=序号 $2=sed表达式 $3=特征串 $4=说明
  local id="$1" sedexpr="$2" feat="$3" desc="$4"
  local neg="/tmp/verify_neg_${id}.sh" out="/tmp/verify_neg_${id}.out"
  echo "== 注入${id}: ${desc} =="
  if ! sed "$sedexpr" "$SRC" > "$neg"; then
    echo "[FAIL-注入${id}] sed 执行失败"; RESULT=1; return
  fi
  if cmp -s "$SRC" "$neg"; then
    echo "[FAIL-注入${id}] sed 未命中锚点(verify_phase1.sh 改动后请同步本脚本)"
    RESULT=1; return
  fi
  bash "$neg" > "$out" 2>&1
  local rc=$?
  local r1=0 r2=0 r3=0
  grep -q "$feat" "$out" && { echo "[PASS] 特征判据命中: ${feat}"; r1=1; } \
    || echo "[FAIL] 未见特征判据失败: ${feat}"
  grep -q "HAS_FAILURES" "$out" && { echo "[PASS] 输出含 HAS_FAILURES"; r2=1; } \
    || echo "[FAIL] 未见 HAS_FAILURES"
  [ "$rc" -ne 0 ] && { echo "[PASS] 退出码非零($rc)"; r3=1; } \
    || echo "[FAIL] 退出码为 0(失败也返回成功)"
  echo "--- 注入${id} 输出尾部 ---"
  tail -3 "$out"
  [ $r1 -eq 1 ] && [ $r2 -eq 1 ] && [ $r3 -eq 1 ] || RESULT=1
  sleep 5   # 等端口释放(cleanup 后 11345 复位需要数秒)
}

RESULT=0
ONLY="${1:-all}"

# 注入1:直行位移阈值 → 不可能值(阈值类失败)
[ "$ONLY" = "all" ] || [ "$ONLY" = "1" ] && inject_and_run 1 \
  "s/^THR_STRAIGHT=1.0/THR_STRAIGHT=99.0/" \
  "直行.*位移" \
  "THR_STRAIGHT=99.0(直行位移判据必失败)"

# 注入2:期望关节名改坏 → MISSING 类失败(关节完整性判据的负例:
# 证明"缺关节"真的会 FAIL,而非固定输出恒过)
[ "$ONLY" = "all" ] || [ "$ONLY" = "2" ] && inject_and_run 2 \
  's/^EXP_JOINTS="left_steering_joint/EXP_JOINTS="no_such_joint/' \
  "joint_states 缺关节" \
  "EXP_JOINTS 首名改坏(缺关节判据必失败)"

# 注入3:传感器话题改名 → 缺消息类失败(频率/时间戳/测距多处应 FAIL,
# 证明"取不到数据"显式失败而非静默通过)
[ "$ONLY" = "all" ] || [ "$ONLY" = "3" ] && inject_and_run 3 \
  's|/scan|/scan_neg|g' \
  "无数据" \
  "/scan 改名 /scan_neg(缺消息判据必失败)"

if [ "$ONLY" != "all" ] && [ "$ONLY" != "1" ] && [ "$ONLY" != "2" ] && [ "$ONLY" != "3" ]; then
  echo "用法: $0 [1|2|3|all]"; exit 2
fi

[ $RESULT -eq 0 ] && echo "===== 负例套件全过 =====" || echo "===== 负例套件有失败 ====="
exit $RESULT
