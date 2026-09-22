# 二维核心逻辑审查+实时交互 入口验证产物(2026-09-17)

审查报告:docs/SIM2D_CORE_AUDIT.md。测试:pytest sim/tests/ 47/47(WSL)。

## 产物清单与生成命令

| 文件 | 生成命令 | 结果 |
|---|---|---|
| demo6.json | `python3 -m sim.demo --json demo6.json` | 六工况 6/6 PASS(直行/定舵圆周 R=L/tanδ 对账 0.00%/倒车/撞锥/停止/满舵兜圈 budget=8s→truncated=True) |
| interactive_demo.gif | `python3 -m sim.interactive --demo-gif interactive_demo.gif` | 键序无头渲染(起步→左转→松键回中→减速→倒车→急停),8.0s,0 撞锥;与实时入口同一 ManualDriver 状态机 |
| interactive_shot.png | `python3 -m sim.interactive --track trackA --shot interactive_shot.png` | **真 TkAgg 窗口**(WSLg)内 KeyEvent 注入驾驶 3s 后截画布:v=+0.15 m/s(=a0 0.1×v_cmd 1.5,数值自洽),松键回中生效 |
| trackA_pp_regression.png | `python3 -m sim.preview --track trackA --seed 0 --out ...` | look=1.2/throttle=0.4:**60.0s lap、0 撞锥**、进度 36.08m(方形基座判定下) |
| trackA_pp.gif | `python3 -m sim.video --track trackA --out ...` | look=0.9/throttle=0.35:**68.7s lap、0 撞锥**(与 3b7fbd5 基线数字一致) |

## 复核要点

- 两份纯追踪数字不同是**参数不同**(lookahead/throttle),各自写明,不挑参数;
- interactive_demo.gif 是脚本化键序的渲染,不冒充实时人工画面;实时窗口的证据
  是 interactive_shot.png(KeyEvent 走 matplotlib 真实事件分发路径);
- 窗口人工键盘驾驶未在无人值守环境执行,复验命令:
  `python3 -m sim.interactive --track trackA --mode manual`(键位见程序打印)。

## 已知限制

- TkAgg 依赖 GUI 会话(WSLg DISPLAY=:0 已验证);无头环境只能 --selftest/--demo-gif;
- matplotlib 版本差异:timer 创建做了 fig/canvas 兼容,KeyEvent 构造签名在
  极老版本可能不同(当前 WSL 3.5 级验证通过)。
