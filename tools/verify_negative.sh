#!/usr/bin/env bash
# =============================================================================
# 端到端负例(收尾清单要求:补一次故意失败的测试,验证退出码)
# 原理:sed 把直行位移阈值 THR_STRAIGHT 注入为不可能值 99,跑完整验收,
#      断言输出含 HAS_FAILURES 且退出码非零。
# 用法(WSL 内): bash ~/hwcart_sim/tools/verify_negative.sh
# 另有秒回自检: bash ~/hwcart_sim/tools/verify_phase1.sh --selftest-fail
# =============================================================================
SRC=~/hwcart_sim/tools/verify_phase1.sh
NEG=/tmp/verify_neg.sh
sed "s/^THR_STRAIGHT=1.0/THR_STRAIGHT=99.0/" "$SRC" > "$NEG"
if cmp -s "$SRC" "$NEG"; then
  echo "[FAIL] sed 未命中 THR_STRAIGHT 行(verify_phase1.sh 改动后请同步本脚本)"
  exit 1
fi
bash "$NEG" > /tmp/verify_neg.out 2>&1
RC=$?
echo "负例退出码: $RC"
R1=0; R2=0
grep -q "HAS_FAILURES" /tmp/verify_neg.out && { echo "[PASS] 输出含 HAS_FAILURES"; R1=1; } || echo "[FAIL] 未见 HAS_FAILURES"
[ "$RC" -ne 0 ] && { echo "[PASS] 退出码非零($RC)"; R2=1; } || echo "[FAIL] 退出码为 0(失败也返回成功)"
echo "--- 负例输出尾部 ---"
tail -4 /tmp/verify_neg.out
[ $R1 -eq 1 ] && [ $R2 -eq 1 ]
