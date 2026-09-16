# 契约工具复验与接入证据(2026-09-17)

## 来源

Codex 独立交付目录:
`C:\Users\Administrator\Documents\Codex\2026-09-13\ros-0-ai-pc-pdf-2026\outputs\racecar_contract\`

主线接入位置:`tools/contract_audit/`(原样拷贝,零改动)。

## 主线独立复验过程与结果(2026-09-17)

1. 逐行阅读 5 份产物(HANDOFF/README/contract.json/check_contract.py/test_contract.py),
   审查要点:AST 字面量提取不 eval/不导入项目代码、symlink 逃逸防护、输出强制
   仓库外新文件(open('x'))、git 只读 per-command safe.directory、退出码语义
   (2=输入错/1=FAIL/0=无失败≠授权)。
2. 原目录复跑单测:`python -m unittest -v test_contract` → **9/9 OK**。
3. 对当前稳定工作区(HEAD `09fa2f95`)复跑检查,生成新报告(不覆盖原报告):
   → **21 项数值约束 PASS、exit 0**,与 Codex 原报告逐项一致。

## 非 PASS 项(两边一致,处置见下)

| 项 | 状态 | 处置 |
|---|---|---|
| `pwm.profile_separation`(sim 1550 vs 驱动 1540) | KNOWN_DIFFERENCE | 保留为不同配置;文档说明来源/单位/转换层;禁止训练 PWM 模型接实车(本轮后续提交处理文档) |
| `launcher.global_cleanup`(teleop_manual.sh 58-63 行全局 pkill) | REVIEW | 已修复:改为 collect_tree 递归登记进程树+setsid 组杀,复跑报告 REVIEW 消除(见下第三份报告;判据与工具零改动,残留命中仅 echo 文案字样,一并改写) |
| `scope.runtime` | UNVERIFIED | 如实保留;运行时参数/图/串口/停车行为不在静态检查范围 |

## 证据文件

| 文件 | 说明 |
|---|---|
| `contract-report-20260917.json` | Codex 原始报告(未动) |
| `contract-report-20260917-mainline-rerun.json` | 主线独立复跑报告(绑定同 HEAD 与源文件 SHA256) |
| `contract-report-20260917-manualchain.json` | 手动链清理修复后复跑:global_cleanup REVIEW 消除,仍 21 PASS / exit 0 |

两报告均绑定:读取源文件 SHA256、契约 SHA256、HEAD、UTC 时间。
**退出码 0 = 静态数值约束未发现失败,不是停车保护/运行时接口/运动轨迹的验证,
更不是动力授权。**

## 复现命令

```bash
cd tools/contract_audit
python -m unittest -v test_contract                                   # 单测
python check_contract.py --repo <仓库根> --output <仓库外新文件.json>  # 检查(输出必须在仓库外)
```
