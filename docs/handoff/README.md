# 交接入口

更新：2026-09-24。先读 [项目文档入口](../README.md) 了解现行参数与研究状态。本目录集中保存给接手者的资料；下面五份旧交接原文是历史快照，不能直接作为当前实车操作指令。

## 接手时先对齐

- **车体**：前后轴距 **0.33 m**；左右轮距是另一项尺寸，不能用轴距代替。电调真中位 1540 PWM、前进硬上限 1620 PWM、舵机中位 81。实车与仿真参数以 `fastsim/params.py` 和 Gazebo `params.yaml` 当前版本为准。
- **速度控制**：`teleop/speed_loop.py` 的 `kp=8`、`ki=75`、`i_limit=80` 已有 12.5 V 直线试跑记录；转弯和亏电工况尚未完成同等验证。逐帧依据在 [`validation/2026-09-24-pid`](../validation/2026-09-24-pid/README.md)。
- **转向模型**：`runs/low_speed_a/candidate_060.npz` 是旧轴距 0.305 m 下训练的基线。旧 FastSim 956/1000 圈不能当作 0.33 m 或实车成绩；Phase B 未启动。当前诊断与租机顺序见 [`转角训练准备`](../research/CORNER_TRAINING_SERVER_PLAN_20260924.md)。
- **版本**：主仓库 `matxn/Auto_Car` 保留完整实车测量和验证数据；队友仓库 `MilkSliceKing/Astrahuaweicar` 同步的是挑选内容，两个 `main` 不要求同一提交。部署到车上的实际文件与服务状态仍须单独核对。

## 历史交接文件

| 文件 | 内容 | 使用时注意 |
| --- | --- | --- |
| [TRAINING_HANDOFF.md](TRAINING_HANDOFF.md) | 低速策略训练、旧 Gazebo 评估、已知问题 | 2026-09-22 快照；参数和远端同步口径已过时 |
| [TELEOP_HANDOVER_TEAMMATE.md](TELEOP_HANDOVER_TEAMMATE.md) | 遥控交给队友的早期步骤 | 与当前网页控制和服务状态核对 |
| [TELEOP_AUTOREPEAT_HANDOVER.md](TELEOP_AUTOREPEAT_HANDOVER.md) | 遥控历史交接 | 旧分支和包路径可能已变化 |
| [BASELINE_sim2real_20260922.md](BASELINE_sim2real_20260922.md) | Gazebo/实车对照基线 | 保留当时验证边界 |
| [SCENARIOS_HANDOFF_20260921.md](SCENARIOS_HANDOFF_20260921.md) | 二维场景对照交接 | 模型参数和控制口径需按当前版本重核 |

原始证据位于 [`validation/`](../validation/)，赛道与地图资料位于 [`track/`](../track/)；交接时应连同模型文件、具体提交号和对应实验结果一起提供。
