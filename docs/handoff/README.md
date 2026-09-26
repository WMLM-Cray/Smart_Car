# 交接入口

当前车辆参数、训练路线和未验收边界见[项目状态](../README.md)。本目录里带日期的长报告是历史快照，保留当时证据，不作为当前操作指令。

## 接手前核对

- 轴距 0.33 m、轮距 0.28 m；ESC 中位/硬上限 1540/1620 PWM；舵机中位 81。
- 直线速度 PID `kp=8, ki=75, i_limit=80`；仅有直线试跑证据。
- 当前训练入口 `fastsim.road_train`，先走廊路段、再赛道；输出转向和速度目标，由实车 PID 控 PWM。
- 赛道图在根目录 `maps/`，是规则图估读坐标，精度限制见[赛道状态](../track/TRACK_AD_STATUS.md)。
- 运行前记录 Git 提交、模型、参数和种子；部署状态须在车上核对。

## 历史资料

| 文件 | 内容 |
|---|---|
| [TRAINING_HANDOFF.md](TRAINING_HANDOFF.md) | 旧低速训练和 Gazebo 结果 |
| [FIRST_LAP_ITERATION_20260924.md](2026-09-24/FIRST_LAP_ITERATION_20260924.md) | 第一圈模型对比及逐局证据 |
| [BASELINE_sim2real_20260922.md](BASELINE_sim2real_20260922.md) | 当时的仿真/实车基线 |
| [TELEOP_HANDOVER_TEAMMATE.md](TELEOP_HANDOVER_TEAMMATE.md)、[TELEOP_AUTOREPEAT_HANDOVER.md](TELEOP_AUTOREPEAT_HANDOVER.md) | 遥控交接快照 |
| [SCENARIOS_HANDOFF_20260921.md](SCENARIOS_HANDOFF_20260921.md) | 二维场景实验 |

逐帧测量和测试结果在 [`../validation/`](../validation/)；赛道数据及限制在 [`../track/`](../track/)。
