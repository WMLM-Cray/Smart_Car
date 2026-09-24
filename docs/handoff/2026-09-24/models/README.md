# 第一圈候选模型与实验证据

本目录是 [第一圈算法迭代汇总交接资料](../FIRST_LAP_ITERATION_20260924.md) 的配套资产。

**这里是快照副本。** 原始文件位于本机 `runs/` 目录下，该目录被 `.gitignore` 整体忽略，
所以模型和证据无法通过 `runs/` 路径入库。为了让接手者拿到可复现的权重，这里放了一份副本。

## 目录结构

每个候选一个子目录，目录名与原 `runs/` 目录名一致：

```text
<路线>_0p33_<日期>/
  candidate_001.npz      # 第 1 轮导出的权重
  candidate_002.npz      # 第 2 轮导出的权重（本轮讨论的候选）
  config.json            # 训练配置：mode、steps、updates、seed、eval 种子区间等
  eval_001.json          # 第 1 轮评估
  eval_002.json          # 第 2 轮评估
  validation_200.json    # 200 局独立验证
  speed_scan_20.json     # 多速度档扫描（部分目录有）
  diagnostic/            # 诊断重跑（部分目录有）
  validation_tracks/     # 逐赛道验证（部分目录有）
```

## 四个路线

| 目录 | mode | 说明 |
|---|---|---|
| `gazebo_margin_0p33_20260923` | `gazebo_margin` | **本轮选中**，`candidate_002` 是主推候选 |
| `gazebo_robust_0p33_20260923` | `gazebo_robust` | 对照。FastSim 分略高，但 Gazebo `track_02` 为 0 圈 |
| `gazebo_sensor_0p33_20260923` | `gazebo_sensor` | 对照，未见 Gazebo 结果 |
| `blind_gap_0p33_screen_20260924` | `blind_gap` | 失败实验，撞锥不降、无速度收益 |

## 权重校验

| 文件 | SHA256 前 16 位 |
|---|---|
| `gazebo_margin_0p33_20260923/candidate_002.npz` | `80b94d459d6b17c4` |
| `gazebo_robust_0p33_20260923/candidate_002.npz` | `80423075d8adfa6f` |
| `gazebo_sensor_0p33_20260923/candidate_002.npz` | `795c8fbd21f96c99` |
| `blind_gap_0p33_screen_20260924/candidate_002.npz` | `e07599c78175e7cb` |
| `runs/low_speed_a/candidate_060.npz`（基线，已在仓库中） | `af3a4de7f984a57f` |

## 没有放进来的东西

- **`latest.pt`** —— 每个 75–146 MB 的训练检查点（含 replay buffer 和 RNG 状态）。
  太大不适合入库，只在本机保留。
- **`*.launch.log`** —— 每个 400–690 KB 的 Gazebo 启动日志。
  被 `.gitignore` 的 `*.log` 规则挡住。需要时向原作者索取。

## 重要提醒

- 这些权重是在 `wheelbase=0.33 m`、`target_speed=0.6 m/s` 下训练和评估的。
- `candidate_002` 是**热启动**自 `runs/low_speed_a/candidate_060.npz`，不是从零训练。
- 引用撞锥数时**必须先确认判据版本**：本轮把 `track_eval.py` 从圆形判据改成了车体矩形 SAT 判据，
  两者结果不可直接比较。详见汇总文档第 7.1 和 8.2.1 节。
- 详细结论、待办和风险见 [汇总文档](../FIRST_LAP_ITERATION_20260924.md)。
