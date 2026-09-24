# 第一圈算法迭代 —— 汇总交接资料

日期：2026-09-24
阶段：第一圈算法（第二圈不在本轮范围）
验收主环境：Gazebo S3 / Ubuntu-22.04 WSL
资料性质：**交接汇总，不是最终模型声明**

本文把本轮第一圈迭代的全部信息集中到一份文档，供接手者整理与复核。文中所有数字都标注了来源文件；**标注"证据缺失"或"未核实"的地方，接手者必须自己重新确认，不能直接引用**。

---

## 1. 一句话结论

`candidate_002`（`gazebo_margin` 路线）在已完成的前三条随机 S3 Gazebo 配对赛道上没有出现稳定性回退，平均圈速约快 `1.20 s`。但只有三条完整配对样本，`track_04` candidate 未形成正式记录，`track_05` 未测试，因此**不能宣称已经超过 baseline，也不能定版**。

---

## 2. 任务边界

比赛运行两圈，分别计时，有效时间为：

```text
T = 第一圈耗时 × 30%
  + 第二圈耗时 × 70%
  + 加罚时间
  - 两次红绿灯时长
```

本阶段只优化第一圈。第一圈的优化优先级是：

1. 稳定完成一圈；
2. clean rate 不低于基线；
3. 在满足稳定性约束后缩短圈速；
4. 不改变实车控制接口，降低后续实车调试成本。

**第一圈的目标不是追求最高瞬时速度。**

---

## 3. 已确认的车辆与接口参数

| 参数 | 当前值 | 来源 / 说明 |
|---|---:|---|
| `wheelbase` | `0.33 m` | 用户复量确认；`fastsim/params.py` 与 Gazebo `params.yaml` 已统一 |
| `wheel_track` | `0.28 m` | 用户确认；左右轮中心距，与轴距是两项不同尺寸 |
| `lidar_hz` | `12.0 Hz` | 实测约 `11.99 / 12.019`；仿真目标取整 12 |
| `lidar_points` | `1667` | `LaserScan` 长度 |
| `imu_hz` | `100 Hz` | 实测 |
| `encoder_hz` | `50 Hz` | 实测约 `49.9 / 50.09` |
| `pwm_mid` | `1540` | 厂家驱动、滚停点、死区测试三条证据一致 |
| `coast_decel` | `0.73` | coast test，三个候选值一致 |
| `turn_mid` | `80` | 动态测量和体感；80/81 在数据上无法区分 |
| `max_steer_angle` | `0.55 rad` | **仍为假设值**，未实测舵角物理极限 |
| `steer_rate` | `4.0 rad/s` | **仍为假设值** |

### 3.1 已废弃的历史值（不要重新引入）

| 参数 | 废弃值 | 说明 |
|---|---:|---|
| `wheelbase` | `0.305` | 出厂源码值，属于另一辆车 |
| `wheelbase` | `0.36` | 2026-09-16 记录，已被 0.33 取代 |
| `wheel_track` | `0.30` | 旧假设值，已被 0.28 取代 |
| `pwm_mid` | `1550` | 旧值，已被 1540 取代 |

### 3.2 待办：舵机中位 80 / 81 的口径不一致

- `fastsim/params.py` 当前为 **80**（注释："用户确认，80/81 无显著差异"）。
- 但 `docs/README.md` 与 `docs/handoff/README.md` 的表格写的是 **81**，依据栏指向 `fastsim/params.py`。

代码与文档目前对不上。接手者需确认哪个是最终值，并统一。数据上 80/81 无法显著区分，如需钉死可跑：

```bash
python -m teleop.straight_trim --turns 79 80 81 --sec 6 --repeat 5
```

---

## 4. 模型资产

### 4.1 模型清单

| 角色 | 路径 | SHA256 前 16 位 | 备注 |
|---|---|---|---|
| **基线** | `runs/low_speed_a/candidate_060.npz` | `af3a4de7f984a57f` | 旧 `0.305 m` 轴距假设下训练 |
| **本轮候选** | `runs/gazebo_margin_0p33_20260923/candidate_002.npz` | `80b94d459d6b17c4` | 主推候选 |
| 对照 A | `runs/gazebo_robust_0p33_20260923/candidate_002.npz` | `80423075d8adfa6f` | `gazebo_robust` 路线 |
| 对照 B | `runs/blind_gap_0p33_screen_20260924/candidate_002.npz` | `e07599c78175e7cb` | `blind_gap` 路线，失败实验 |
| 对照 C | `runs/gazebo_sensor_0p33_20260923/candidate_002.npz` | 未取得 | `gazebo_sensor` 路线 |
| 历史候选 | `runs/final_candidate_0p33_20260923/` | 未取得 | 见第 8.3 节 |

`candidate_002` 的热启动来源是 `runs/low_speed_a/candidate_060.npz`，不是从零训练。

### 4.2 权重推送状态

`runs/*` 在 `.gitignore` 中被整体忽略，只放行三条例外：

```text
!runs/policy_*.npz
!runs/low_speed_a/candidate_*.npz
!runs/track_s3/*
```

**`runs/gazebo_margin_0p33_20260923/candidate_002.npz` 不匹配任何一条例外，因此不会被推送。**
`gazebo_robust`、`gazebo_sensor`、`blind_gap` 三个目录同样全部不入库。

接手者拉取仓库后**只有文档，没有本轮权重**。需要权重时须向原作者索取，或按第 10 节的训练命令重新产生。

---

## 5. 训练路线

### 5.1 路线定义

本轮使用 `fastsim/iterative_train.py`，它保留现有策略网络架构和教师（`SpeedPlanTeacher`），**只改训练分布与 replay 配比**，使各实验能用同一份基线权重对比。

已定义的 mode：

| mode | probability | geometry | sensor | blind_gap | nominal_fraction | 其他 |
|---|---:|---:|---:|---:|---:|---|
| `nominal` | 0.00 | 0.00 | 0.00 | 0.00 | 1.00 | — |
| `geometry` | 0.20 | 0.50 | 0.00 | 0.00 | 0.80 | — |
| `sensor` | 0.20 | 0.00 | 0.50 | 0.00 | 0.80 | — |
| `geometry_sensor` | 0.30 | 0.50 | 0.50 | 0.00 | 0.70 | — |
| `blind_gap` | 0.35 | 0.10 | 0.15 | 1.00 | 0.65 | — |
| `gazebo_robust` | 0.75 | 0.20 | 0.50 | 0.25 | 0.55 | `scan_hold=True, action_lag=1, action_smoothing=0.35` |
| **`gazebo_margin`** | 0.70 | 0.15 | 0.40 | 0.15 | 0.65 | `scan_hold=True, action_lag=1, action_smoothing=0.10` |
| `gazebo_sensor` | 0.65 | 0.10 | 0.50 | 0.10 | 0.70 | `scan_hold=True, action_lag=0, action_smoothing=0.0` |

`gazebo_*` 三个 mode 的设计意图是近似 Gazebo 部署回路：雷达 12 Hz 发布而控制跑 20 Hz，命令通路存在小延迟。

**`gazebo_margin` 与 `gazebo_robust` 的关键差异是 `action_smoothing`（0.10 vs 0.35）**：`gazebo_margin` 减弱动作平滑，以减少鲁棒训练造成的速度损失，同时把 `nominal_fraction` 从 0.55 提到 0.65。

### 5.2 本轮各候选的训练配置

| 目录 | mode | steps | updates | seed | eval_seed_start | eval_count | target_speed | lookahead |
|---|---|---:|---:|---:|---:|---:|---:|---:|
| `gazebo_margin_0p33_20260923` | `gazebo_margin` | 100000 | 1500 | 20260924 | 51000000 | 20 | 0.6 | 1.35 |
| `gazebo_robust_0p33_20260923` | `gazebo_robust` | 100000 | 1500 | 20260923 | 51000000 | 20 | — | — |
| `gazebo_sensor_0p33_20260923` | `gazebo_sensor` | 100000 | 1500 | 20260925 | 51000000 | 20 | 0.6 | 1.2 |
| `blind_gap_0p33_screen_20260924` | `blind_gap` | 50000 | 750 | 240924 | 52000000 | 50 | — | — |

全部四条的训练 `git_head` 均为 `f7131779a0923e94f3e933739fa7d8b5424bd1fc`。

### 5.3 没有采用的做法

- 没有用官方赛道 A 做模型选择；
- 没有把 FastSim 结果直接当作 Gazebo 结论；
- 没有同时启动多个 Gazebo 进程；
- 没有覆盖历史训练目录；
- 没有把 `0.36 m` 作为第二套轴距混入；
- 没有在验收通过前录制"最终模型"视频；
- 没有把单次少量局数结果当成最终鲁棒性证明。

---

## 6. FastSim 与 Gazebo 的分工

### FastSim

训练闭环与大样本筛选环境。直接调用 `fastsim.low_speed_train` / `fastsim.low_speed_eval`，每局重新生成轻量二维赛道。

适合：快速比较奖励/扰动/速度配置；淘汰明显失败候选；做 200/1000 局统计。

### Gazebo

闭环接口验收环境。策略经真实 ROS/Gazebo 节点运行，包含：ROS 话题与控制周期、`LaserScan` 离散采样/延迟/归一化、IMU 与编码器频率、车辆几何与动力学、摩擦与碰撞、随机 S3 锥桶布局、仿真启动与重置时序。

**FastSim 中更快的策略可能在 Gazebo 中因延迟、几何误差或动力学差异而撞锥、出界或无法完成一圈。FastSim 只能预筛选，最终结论必须来自配对 Gazebo S3。**

---

## 7. Gazebo 验收协议

```text
赛道：随机生成 S3
速度：POLICY_SPEED=0.5
最长运行时间：RUN_SECONDS=115
判据：track_eval.py
比较：同一随机赛道先后运行 baseline 和 candidate
```

每局应记录：随机种子与赛道布局、模型路径、Git commit、clean lap / 圈数、圈速、撞锥数、出界数、终止原因、运行参数。

候选通过门槛：

```text
candidate clean rate  >= baseline clean rate
candidate 平均圈速    优于 baseline
candidate 撞锥/出界   没有明显增加
```

**速度提升不能抵消稳定性下降。单次最快圈或少量 100% clean rate 不能代替扩大样本。**

### 7.1 `track_eval.py` 的判据变更（重要）

本轮把撞锥判据从**圆形**改成了**车体矩形 SAT 相交**（`Eval._rect_hits_box`），并传入真实车体长宽 `p.length × p.width`，而不是原来的外接圆半径。

原因：`/model_states` 给的是 `base_footprint`（后轴）位姿，不是车体几何中心。旧的外接圆半径 `hypot(length - rear_overhang, width/2)` 会**短算约 0.1 m**，车头顶着锥桶都可能判不出撞。

**后果：判据变更前后的撞锥数不可直接比较。** 见第 8.2.1 节的具体案例。

---

## 8. 已完成的 Gazebo 结果

### 8.1 前三条配对赛道（`gazebo_iteration_live_20260924`）

| 赛道 | baseline | candidate | 稳定性 | 来源文件 |
|---|---:|---:|---|---|
| `track_01` | `79.56 s` | `76.55 s` | 两者均 1 圈、0 撞锥、0 出界 | `track_01_baseline.json` / `track_01_margin.json` |
| `track_02` | `98.14 s` | `97.55 s` | 两者均 1 圈、0 撞锥、0 出界 | `track_02_baseline.json` / `track_02_margin.json` |
| `track_03` | `79.31 s` | `79.31 s` | **见下方"证据缺失"** | 仅 `track_03_baseline.json` |
| `track_04` | `83.87 s` | 未完成正式记录 | baseline 1 圈、0 撞锥、0 出界 | `track_04_baseline.json` |

前三条已完成配对赛道：candidate 平均圈速约快 `1.20 s`，未发现稳定性回退。

> **⚠️ 证据缺失 —— `track_03` candidate**
>
> 目录内**只有 `track_03_baseline.json` 与 `track_03_baseline.launch.log`**。
> **不存在 `track_03_margin.json`，也不存在 `track_03_margin.launch.log`。**
> 全仓库检索 `runs/` 未找到任何 `track_03` 的 candidate 结果文件。
>
> 因此 `track_03` candidate 的圈速 `79.31 s`、以及"两者均 1 圈 0 撞锥 0 出界"这句结论，
> **在本次资料整理中无法从产物文件核实**。这个数字同时出现在 `runs/HANDOFF_REPORT_20260924.md`
> 和 `docs/handoff/2026-09-24/TECHNICAL_REPORT.md` 的表格中，但两份文档都没有指向产物。
>
> 接手者必须先确认 `track_03` candidate 是否真的跑过并留下记录。**如果找不到，前三条配对就只剩两条**，
> "平均快 1.20 s"这个结论的依据要从两条赛道重新计算。
>
> 另外请注意：`track_01`/`track_02` candidate 的 `elapsed_s`（`115.4` / `116.1`）**大于 `RUN_SECONDS=115`**，
> 说明实际运行时长与协议记载不完全一致，接手时需核对当时的实际参数。

### 8.2 本轮未被采用的候选（对照实验）

这些候选**没有被选中**，但记录了失败原因，接手者不必重跑：

| 候选 | 路线 | FastSim 200 局 clean rate | 是否优于基线 | 结论 |
|---|---|---:|---|---|
| `candidate_060`（基线） | — | `0.95`（190/200） | — | 参考 |
| `gazebo_margin` | margin | `0.97`（194/200，Wilson `[0.936, 0.986]`） | 是 | **本轮选中** |
| `gazebo_robust` | robust | `0.975`（195/200，Wilson `[0.943, 0.989]`） | 是 | 见下方说明 |
| `blind_gap` | blind_gap | `0.97`（194/200） | 否（有速度收益但撞锥不降） | 失败实验，保留 |

来源：各目录下 `validation_200*.json`（`blind_gap` 为 `validation_200_same_seeds.json`）。
四个候选的目标速度均为 `0.6 m/s`，200 局全部 `complete: true`。

> **`gazebo_robust` 的 FastSim 成绩（0.975）实际略高于被选中的 `gazebo_margin`（0.97）。**
> 本轮选择 `gazebo_margin` 的依据是 Gazebo 表现而非 FastSim 分数（见 8.2.1）。
> 接手者如要复核这个取舍，需要看两者的 Gazebo 成对结果。

#### 8.2.1 撞锥判据变更的影响（`gazebo_robust` 案例）

`gazebo_robust` 的 `track_01` 单局结果在**不同判据下差异极大**：

| 文件 | 判据 | 圈速 | 撞锥 | 出界 |
|---|---|---:|---:|---:|
| `gazebo_track01_candidate_robust.json` | 旧（圆形） | `61.06 s` | **16** | 0 |
| `diagnostic/gazebo_track01_candidate_robust_rect.json` | 新（矩形 SAT） | `64.91 s` | **0** | 0 |
| `diagnostic/gazebo_track01_baseline_rect.json` | 新（矩形 SAT） | `68.74 s` | **0** | 0 |

**同一策略、同一赛道，撞锥数从 16 变成 0。** 这不是策略变了，是判据变了。

`blind_gap` 也有同样现象：旧判据 `14` 撞锥，`diagnostic/gazebo_track01_baseline_same_world_v2.json` 用新判据跑基线同样得到 `14` 撞锥（`64.21 s`）。

> **接手者注意**：`runs/` 下带有 `gazebo_track01_*.json`（不含 `_rect`）的文件都是**旧圆形判据**的产物。
> 引用撞锥数时必须先确认是哪个判据。`docs/handoff/2026-09-24/TECHNICAL_REPORT.md` 第 6 节的表格
> 没有标注判据版本，需要补充。

### 8.3 历史候选 `final_candidate_0p33_20260923`

`runs/final_candidate_0p33_20260923/validation/` 下有一次 `track_02` 成对结果：

| 模型 | 圈数 | 圈速 | 撞锥 | 结论 |
|---|---:|---:|---:|---|
| baseline | 1 | `95.96 s` | 0 | 参考 |
| final candidate | 1 | `97.90 s` | 0 | **比基线慢 1.94 s，未优于基线** |

该目录另有一次 `track_02.json` 为 `0 圈` 的失败尝试（`elapsed_s: 71.0`），不能与完成圈结果混合统计。

### 8.4 `track_04` 中断状态

曾用唯一 Gazebo 会话（`session_id=37465`）运行 `track_04` candidate，进程后被中断。中断前日志显示：

- 约 `107.1 s` 时已完成第 1 圈；
- 累计 `0` 撞锥；
- 累计 `0` 出界；
- **但未形成完整正式结果，不能计入最终 clean rate。**

产物现状：目录内只有 `track_04_margin.launch.log`，**没有 `track_04_margin.json`**。该 log 尾部只有 `ackermann_steering_controller` 的 Twist 弃用警告，未含评估结论。

后续应使用相同协议完整重跑，或在汇总中明确标记为 `incomplete`。

### 8.5 速度扫描（FastSim，仅预筛选）

`runs/gazebo_robust_0p33_20260923/speed_scan_20.json`，模型为 `gazebo_robust/candidate_002.npz`，每档 20 局：

| 速度 | clean | clean rate | 平均圈速 | 失败原因 |
|---:|---:|---:|---:|---|
| `0.4 m/s` | 17/20 | `0.85` | `97.35 s` | 出界 3 |
| `0.5 m/s` | 18/20 | `0.90` | `79.33 s` | 出界 2 |
| `0.6 m/s` | 18/20 | `0.90` | `65.72 s` | 出界 2 |
| `0.8 m/s` | 16/20 | `0.80` | `45.28 s` | 撞锥 1、出界 3 |
| `1.0 m/s` | 11/20 | `0.55` | `30.53 s` | 撞锥 6、出界 3 |

> **这份扫描的模型是 `gazebo_robust`，不是被选中的 `gazebo_margin`。**
> 且每档仅 20 局，Wilson 区间很宽（如 `0.4` 档为 `[0.64, 0.95]`），**只能作为预筛选信号，不能作为速度决策依据**。
> 扫描在 **FastSim** 中完成，不是 Gazebo。

### 8.6 FastSim 基线成绩（历史）

`runs/low_speed_a/validation_1000_060.json`：`candidate_060` 在 1000 局、目标速度 `0.6 m/s` 下

- clean rate `0.956`（`956 / 1000`）
- Wilson 95% 区间 `[0.9414, 0.9671]`
- 失败：撞锥 26 次、出界 18 次

> **这份成绩是在旧 `0.305 m` 轴距等仿真假设下取得的**，不能当作 `0.33 m` 或实车的通过率。
> `runs/current_0p33_baseline_20260924/candidate_060_200.json` 是 `0.33 m` 下重跑的 200 局基线（clean rate `0.95`），
> 可作为当前参数下的基线参考。

---

## 9. 当前瓶颈与后续路线

### 9.1 瓶颈

1. **证据量不足** —— 前三条（实际可能只有两条，见 8.1）结果是积极信号，但无法证明对随机 S3 的总体鲁棒性。
2. **稳定性与速度的权衡** —— 鲁棒增强过强会让策略变慢；速度奖励过强会增加弯道和窄道失败概率。
3. **FastSim 到 Gazebo 的迁移差异** —— 观测延迟、频率、车体几何、转向中值、PWM 中值、速度单位是否完全对齐，仍未逐项验证。
4. **验收耗时** —— 完整 Gazebo 单局耗时长，不适合一开始就对大量候选做全量验证。

### 9.2 建议执行顺序

**第一阶段：补齐最小正式验收**

1. 确认或重跑 `track_03` candidate（见 8.1 证据缺失）；
2. 完整重跑 `track_04` candidate；
3. 运行 `track_05` baseline；
4. 运行 `track_05` candidate；
5. 汇总配对结果；任何稳定性回退都先分析失败轨迹，不要直接增加训练步数。

**第二阶段：扩展随机 S3 样本**（10-20 条随机布局）

记录 clean rate、失败类型分布、平均完成圈速、最慢成功圈速、撞锥/出界分位数、不同弯道/窄道/非闭环片段的失败位置。

**第三阶段：速度扫描**（候选通过稳定性门槛后）

```text
0.4 / 0.5 / 0.6 / 0.8 / 1.0 m/s
```

先每档少量局数快速淘汰，再对满足 clean rate 门槛的最高速度做扩大样本。**不要一开始就对每档跑 200 局。**

**第四阶段：最终版本与视频**

只有当候选在随机 S3 上满足 `clean rate >= baseline`、`平均圈速优于 baseline`、`无明显增加撞锥/出界`，
才创建独立最终版本目录并录制 Gazebo 俯视角完整一圈视频。视频必须标明：模型版本、Git commit、
赛道随机种子、`POLICY_SPEED`、`wheelbase`、是否为最终验收样本。

---

## 10. 复现方法

### 10.1 环境

Ubuntu-22.04 WSL，工作目录 `/mnt/d/study/goboze/Auto_Car-main`。

**不要使用 Kali**，也不要把 `/root/Auto_Car/Auto_Car-main` 当作实验副本。

### 10.2 训练

`fastsim/iterative_train.py` 是本轮新增的热启动训练入口：

```bash
python -m fastsim.iterative_train \
  --mode gazebo_margin \
  --init-weights runs/low_speed_a/candidate_060.npz \
  --out runs/<新版本目录> \
  --steps 100000 --round-steps 50000 --updates 1500 \
  --seed 20260924 --eval-seed-start 51000000 --eval-count 20 \
  --target-speed 0.6 --lookahead 1.35
```

可选 mode 见第 5.1 节。**每次实验必须使用新的独立版本目录，不覆盖历史目录。**

### 10.3 FastSim 大样本评估

```bash
python -m fastsim.low_speed_eval \
  --model runs/<目录>/candidate_002.npz \
  --out runs/<目录>/validation_200.json \
  --count 200 --workers 4 --seed-start 30000000
```

`--seed-start` 是本轮新增参数，用于把训练选择种子与独立测试种子分开。

### 10.4 FastSim 速度扫描

```bash
python -m fastsim.speed_scan \
  --model runs/<目录>/candidate_002.npz \
  --out runs/<目录>/speed_scan_20.json \
  --count 20 --workers 4 --seed-start 60000000 \
  --speeds 0.4 0.5 0.6 0.8 1.0
```

### 10.5 Gazebo 单局

本轮新增 `3Dsim/tools/run_s3_policy_once.sh`：

```bash
bash 3Dsim/tools/run_s3_policy_once.sh <world-source> <model> <layout> <output>
```

脚本会：校验 world source 含 `gazebo_ros_state`（否则 `/model_states` 不可用）、设置 `ROS_DOMAIN_ID=43`、
禁用 Gazebo 模型数据库、启动 `sim_bringup.launch.py`（`use_gui:=false`）、等待 `/joint_states`、
拉起 `steer_policy_node.py` 与 `hold_enable.py`、调用 `track_eval.py`，并用 `trap cleanup` 收尾。

> **口径不一致（需修正）**：脚本默认 `POLICY_SPEED=0.6`、`RUN_SECONDS=120`，
> 而第 7 节验收协议写的是 `0.5` / `115`。脚本可用环境变量覆盖，不影响正确性，但两处应统一。

### 10.6 运行前检查清单

- 当前 Git commit；
- `wheelbase=0.33`、`wheel_track=0.28`；
- `POLICY_SPEED`、`RUN_SECONDS` 与协议一致；
- 没有其他 Gazebo 实例；
- 使用的模型路径及其 SHA256；
- 赛道随机种子；
- 输出目录是新的版本目录；
- 确认 `track_eval.py` 用的是矩形 SAT 判据。

---

## 11. 不可违反的实验约束

- 使用 Ubuntu-22.04 WSL，不使用 Kali；
- 不使用官方赛道 A 做模型选择；
- 不启动第二个训练或 Gazebo 进程；
- 不覆盖历史实验目录；
- 新实验使用独立版本目录；
- 不修改实车控制接口；
- 固定 `wheelbase=0.33 m`；
- 最终验收前不录制或宣称最终视频；
- 不把训练选择用的种子当作独立测试种子。

---

## 12. 本轮代码改动清单

以下改动在本地工作区完成，**尚未推送**。接手者需按类别分别处理。

### 12.1 轴距 `0.36 → 0.33` 统一（11 个文件）

| 文件 | 改动 |
|---|---|
| `3Dsim/hwcart_sim/src/racecar_teleop/config/generic.yaml` | `wheelbase: 0.36 → 0.33` |
| `3Dsim/hwcart_sim/src/racecar_teleop/racecar_teleop/mapping.py` | 同上 |
| `3Dsim/hwcart_sim/src/racecar_teleop/racecar_teleop/teleop_node.py` | 同上 |
| `3Dsim/hwcart_sim/src/racecar_teleop/test/test_mapping.py` | 同上 |
| `3Dsim/hwcart_sim/src/racecar_teleop/test/test_state.py` | 同上 |
| `3Dsim/tools/contract_audit/contract.json` | `wheelbase 0.36 → 0.33`（依据升级为 `user_confirmed_measurement`）；`pwm_model` `1550 → 1540` |
| `3Dsim/tools/contract_audit/HANDOFF.md` | 文字订正 |
| `3Dsim/tools/teleop/realcar_teleop.yaml` | `wheelbase: 0.36 → 0.33` |
| `3Dsim/tools/teleop/verify_teleop_task_spec.py` | 同上 |
| `3Dsim/tools/trackA/test_driver.py` | `WHEELBASE = 0.36 → 0.33` |
| `README.md` | 参数说明更新 |

方向与远端一致，属于补齐远端 `README.md` 自己标注的"早期手柄、Nav2 测试工具仍带旧 0.36 m 轴距常量"。

### 12.2 `track_eval.py` 判据重写

`3Dsim/tools/track_eval.py`：撞锥判据从圆形改为车体矩形 SAT 相交（见 7.1）。这是本轮最实质的代码改动。

### 12.3 `fastsim/` 兼容性与工具改动（5 个文件）

| 文件 | 改动 |
|---|---|
| `fastsim/low_speed_train.py` | `torch.load` 增加 `TypeError` 回退，兼容 Ubuntu 22.04 可能提供的旧 Torch（无 `weights_only` 参数） |
| `fastsim/low_speed_eval.py` | 新增 `--seed-start` 参数 |
| `fastsim/params.py` | `turn_mid: 81 → 80` |
| `fastsim/tests/test_low_speed.py` | `torch.testing.assert_close` → `torch.allclose`（旧 Torch 兼容） |
| `fastsim/tests/test_sim.py` | `ConeTrackEnv(terminate_on_cone=True)` 显式化；终止原因允许 `timeout` |

### 12.4 新增工具（3 个文件，未跟踪）

| 文件 | 用途 |
|---|---|
| `fastsim/iterative_train.py` | 热启动训练入口，8 个 mode（第 5 节） |
| `fastsim/speed_scan.py` | 多速度档位扫描（第 10.4 节） |
| `3Dsim/tools/run_s3_policy_once.sh` | Gazebo 单局自动化（第 10.5 节） |

### 12.5 文档改动（6 个文件，**路径已失效**）

`docs/SIM_INTEROP.md`、`docs/TRAINING_HANDOFF.md`、`docs/REALCAR_ONBOARDING.md`、`docs/SIM2D_CORE_AUDIT.md`、
`docs/TELEOP_INTEGRATION_GUIDE.md`、`docs/测量清单.md`。

远端已把这些文件移入 `docs/reference/`。**直接提交这些扁平路径会复活已删除的旧路径。**
内容本身仍有价值（补了远端未更新的 `0.305` 轴距、`1550` PWM、雷达频率表），应重新落到 `docs/reference/` 路径下再提交。

### 12.6 与远端冲突的文件

`README.md` —— 远端已被重写 131 行且已含 `0.33`。本地改动基于旧结构，**直接提交会把远端重写退回旧版**，建议丢弃或基于远端版本重做。

---

## 13. 待处理事项汇总

| # | 事项 | 性质 |
|---|---|---|
| 1 | `track_03` candidate 产物缺失，需确认或重跑 | **数据完整性** |
| 2 | `track_04` candidate 未完成正式记录 | 数据缺口 |
| 3 | `track_05` 未配对 | 数据缺口 |
| 4 | 舵机中位 `80` / `81` 代码与文档不一致 | 口径冲突 |
| 5 | `run_s3_policy_once.sh` 默认速度/时长与协议不一致 | 口径冲突 |
| 6 | `gazebo_track01_*.json` 旧圆形判据产物未标注 | 数据标注 |
| 7 | 本轮权重 `candidate_002.npz` 等未入库 | 资产可复现性 |
| 8 | `runs/` 下两份报告（`HANDOFF_REPORT_20260924.md`、`gazebo_iteration_report_20260924.md`）被 ignore 挡住 | 资产可复现性 |
| 9 | `docs/reference/` 下 6 个文档未同步本轮参数订正 | 文档滞后 |
| 10 | `wip/local-changes-20260923` 分支含 94 文件 / +16994 行未合并，且是扁平 docs 布局 | 分支清理 |

---

## 14. 需要接手者优先确认的技术风险

1. 轴距参数是否在控制器、Gazebo 车辆模型、FastSim 和评估脚本中一致；
2. `pwm_mid=1540`、`turn_mid=80` 是否被不同模块重复覆盖；
3. `LaserScan` 长度、角度范围、频率和归一化是否一致；
4. 控制器收到的观测是否存在一帧延迟或时间戳错位；
5. FastSim 的动作/速度单位是否和 Gazebo 完全一致；
6. `track_eval.py` 的 clean lap 判据是否和比赛要求一致；
7. 随机 S3 生成器是否包含足够的窄道、急弯、非闭环和视野变化；
8. 训练增强是否过强，导致 candidate 在 nominal 赛道上损失速度；
9. 评估是否存在复用旧进程、旧 world 或旧模型的风险；
10. 每次实验是否把模型、参数、随机种子、日志和 Git commit 保存到同一版本目录。

---

## 15. 资料来源

本文汇总自以下文件，接手者可直接查阅原文：

**本轮新写（未推送）**

- `docs/handoff/2026-09-24/TECHNICAL_REPORT.md`
- `docs/handoff/2026-09-24/REPRODUCTION_AND_ACCEPTANCE.md`
- `docs/handoff/2026-09-24/STATUS.md`
- `docs/handoff/2026-09-24/README.md`

**`runs/` 下报告（被 `.gitignore` 挡住，未推送）**

- `runs/HANDOFF_REPORT_20260924.md` —— 内容比 `TECHNICAL_REPORT.md` 更全
- `runs/gazebo_iteration_report_20260924.md`
- `runs/version_20260923_fastsim_gazebo/REPORT.md`

**原始产物**

- `runs/gazebo_iteration_live_20260924/` —— 前三条配对赛道 JSON 与日志
- `runs/gazebo_margin_0p33_20260923/` —— 本轮候选的 config、eval、validation_200
- `runs/gazebo_robust_0p33_20260923/` —— 对照候选 + `diagnostic/` 矩形判据重跑 + speed_scan
- `runs/blind_gap_0p33_screen_20260924/` —— 失败实验
- `runs/final_candidate_0p33_20260923/validation/` —— 历史候选
- `runs/current_0p33_baseline_20260924/` —— `0.33 m` 下的基线 200 局
- `runs/low_speed_a/validation_1000_060.json` —— 基线 1000 局（旧假设）

**远端现行文档**

- `docs/README.md` —— 项目文档入口
- `docs/handoff/README.md` —— 交接入口
- `docs/reference/README.md` —— 历史参考文档说明
- `docs/research/CORNER_TRAINING_SERVER_PLAN_20260924.md` —— 转角问题与租机准备

---

## 16. 交接一句话

本轮把工作从"盲目训练候选"推进到"以 `candidate_002` 为主、用随机 S3 Gazebo 做配对验收"的阶段，
已有正向信号但证据不足。接手后**第一件事是确认 `track_03` candidate 的产物是否存在**——
这直接决定"前三条配对、平均快 1.20 s"这个结论是否成立；然后补齐 `track_04`、`track_05`，
再扩大样本和做速度扫描，而不是直接增加训练步数或从历史 checkpoint 重新挑选模型。
