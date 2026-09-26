# 第一圈训练结果（历史摘要）

记录日期：2026-09-24。此文只概括当时的候选筛选，不代表当前路段训练，也不是当前模型验收。当前参数和训练路线见[项目状态](../../README.md)与[训练计划](../../research/CORNER_TRAINING_SERVER_PLAN_20260924.md)。

## 当时结论

当时优先候选为 `gazebo_margin/candidate_002.npz`，SHA256 前缀 `80b94d459d6b17c4`，由旧基线 `candidate_060.npz` 热启动。FastSim 200 局为 194/200（0.97）。

报告当时声称前三条 Gazebo S3 配对平均快约 1.20 秒，但 `track_03_margin.json` 缺失，实际最多两条有完整配对证据；`track_04` candidate 没有正式结果，`track_05` 未测。因此不能据此宣称胜过基线或定版。

## 候选比较

| 候选 | 当时结果 | 结论 |
|---|---|---|
| `gazebo_margin/candidate_002` | FastSim 194/200；Gazebo 样本不足 | 临时优先候选，未定版 |
| `gazebo_robust/candidate_002` | FastSim 195/200；另有 20 局/速度档扫描 | 对照；小样本不能定速度或稳定性 |
| `blind_gap/candidate_002` | 速度收益未带来撞锥改善 | 失败方向，数据保留 |
| `gazebo_sensor/candidate_002` | 缺少对应 Gazebo 结果 | 未判定 |

比较撞锥数时要区分旧圆形判据和矩形 SAT 判据；同一轨迹的计数可能不同。不得混表。

## 当前可用事实与限制

- 当时轴距已按用户测量统一到 0.33 m，轮距 0.28 m；车辆其余现值以[项目状态页](../../README.md)和当前参数文件为准。
- 当时的候选模型、配置和逐局 JSON 位于 [`models/`](models/)。FastSim、Gazebo、实车结果各自适用，不能互相替代。
- 旧低速基线在旧假设下的高通过率，不代表现在的 0.33 m 参数、当前路段课程或实车通过率。
- 本摘要不保留逐轮时间线和已过时操作命令。

完整原文可从整理提交恢复：

```bash
git show 66814ff:docs/handoff/2026-09-24/FIRST_LAP_ITERATION_20260924.md
```
