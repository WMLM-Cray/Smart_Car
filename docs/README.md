# 项目文档入口

更新：2026-09-24。本页是当前研究状态的入口；代码中的参数和当日原始测量是数值依据。旧方案保留在 [`reference/`](reference/README.md)，不能直接照旧方案向实车发指令。

## 当前已确定

| 项目 | 当前值 / 状态 | 依据 |
| --- | --- | --- |
| 前后轴距 `wheelbase` | **0.33 m**，用户复量后确定；FastSim、Gazebo 已统一 | `fastsim/params.py`、`3Dsim/hwcart_sim/src/racecar_sim/config/params.yaml` |
| 左右轮距 | 尚未在这里确认；与前后轴距是不同尺寸 | 测量时分别记录 |
| 电调真中位 / 前进硬上限 | 1540 / 1620 PWM | [`2026-09-23 曲线`](validation/2026-09-23-curve/README.md)、[`2026-09-24 PID`](validation/2026-09-24-pid/README.md) |
| 舵机中位 | 81，直线观察选定，精确重复标定仍待做 | `fastsim/params.py` |
| 直线速度闭环 | `kp=8`、`ki=75`、`i_limit=80`；12.5 V 直线试跑已记录 | [`2026-09-24 PID`](validation/2026-09-24-pid/README.md) |

## 转角模型研究

- 当前基线为 `runs/low_speed_a/candidate_060.npz`。旧 FastSim 评估是 956/1000 圈，但权重在 **0.305 m 轴距**等旧仿真假设下训练；这个成绩不能算 0.33 m 或实车的通过率。
- Phase B 尚未启动。2026-09-22 的 Gazebo 赛道 A 三次均未完圈，记录显示在雷达看不到的黄 V 胶带缺口驶出；这和其他转角差的现象需要分开分析。
- 最大前轮转角 `0.55 rad`、舵机速率 `4.0 rad/s` 仍是假设。下一步先记录弯前策略动作、实际转向和偏航响应，并量左右舵角，再决定是否从旧权重补训。
- 租机和训练决策见 [`转角训练准备`](research/CORNER_TRAINING_SERVER_PLAN_20260924.md)。给队友接手的入口见 [`handoff/`](handoff/README.md)；旧训练交接保留作历史快照，其中旧轴距、PWM 和远端同步叙述已失效。

## 去哪里找

| 目录 | 内容 | 用法 |
| --- | --- | --- |
| [`validation/`](validation/) | 原始 CSV、日志、图和当时结论 | 核对实测与实验；早期结论以较晚订正为准 |
| [`track/`](track/) | 赛道 A 几何、地图与仿真证据 | 场地和路线研究 |
| [`sim2real/`](sim2real/) | 二维仿真与实车差异分析 | 研究记录，先核对当时参数版本 |
| [`research/`](research/) | 当前待做研究及租机准备 | 从这里规划下一轮实验 |
| [`handoff/`](handoff/README.md) | 当前交接入口和历次交接文件 | 先读入口，再看具体历史交接 |
| [`reference/`](reference/README.md) | 原根目录方案与审查文档 | 历史资料；先核对日期和当前代码 |

整理只调整入口与文档位置，保留原始证据。根目录只维护本页，新的实验数据进 `validation/`，新的研究计划进 `research/`。
