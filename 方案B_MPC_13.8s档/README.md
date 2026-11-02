# 方案B MPC:第二圈从 18s 档推进到 13.83s

> 口径:FastSim 仿真,未实车验证。起点是 TD3 steer2 candidate_039(18.63s)与 BEV 基线 gripF3_c011。
> 详细分析见 `报告/BEV速度评测报告.md`。

## 成果(四赛道平均 lap2,cap 5.0,三种子组 120 局)

| 阶段 | 方案 | lap2 | 完赛 |
|---|---|---|---|
| BEV 基线 gripF3_c011 | 全动作 | 18-20s | cap↑ 通过率反降(C/D 12-31%) |
| 纯追踪 PP 调参 | lookahead/前馈/预瞄 | 15.7s | 调参无净收益,封顶 |
| **采样式 MPC** | 128条×12拍×3段转向 | **13.83s** | **119/120** |

分赛道:A 14.13 / B 13.29 / C 14.23 / D 13.62,单圈最好 12.70s。已贴住实测抓地(2.3-2.9 m/s²)下的物理下界 13.4-14.4s;10s 不可达,11s 需轮胎抓地提至 ~4.5 m/s²(1.7 倍)+执行链升级。

## 目录

- `代码/` — plan_b_ceiling(全知天花板)、plan_b_fast_pp(PP 扫描)、plan_b_mpc(MPC 主体,复现命令见模块 docstring)、gripF3_c011.npz(BEV 基线模型)
- `评测数据/` — 方案 B 全过程 runs(JSON,53 个文件;终验 = plan_b_mpc_v6_*.json 三种子组 120 局;BEV 天花板 = bev_gripF3_c011_full.json)
- `报告/` — BEV速度评测报告.md

## 代码依赖

plan_b 三件套基于 Auto_Car 仓库 origin/main(606535b)开发,依赖其中的 turn_drag/v_over/grip 仿真参数与 set_grip_feedback 等;在完整检出(feat/plan-b-mpc 分支)里运行:

    python -m evaluation.plan_b_mpc --tracks A B C D --caps 5.0 --a-lat 3.0 --count 10
