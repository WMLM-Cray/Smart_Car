# Auto Car

OrangePi AIpro 无人车项目：FastSim 训练、Gazebo 仿真、赛道数据、实车控制和 ROS 2 导航。

> 当前在临时精简分支 `codex/temporary-pruned-version`，不合并主线。删除内容见 [清单](docs/PRUNED_PATHS.txt)。

## 目录

| 路径 | 用途 |
|---|---|
| `fastsim/` | 车辆模型、路段生成、训练和评估 |
| `teleop/` | 实车网页、策略、速度闭环、看门狗和标定 |
| `3Dsim/hwcart_sim/` | ROS 2 Humble / Gazebo Classic 车模和传感器 |
| `3Dsim/tools/` | 赛道、导航、雷达及验证工具 |
| `maps/` | A–D 规则赛道图、桶位和布局数据 |
| `docs/validation/` | 测量、实验结果和失败证据 |
| `docs/reference/` | 历史方案；执行前核对日期和当前代码 |

## 当前训练方向

入口：`python -m fastsim.road_train --out runs/road_trial --workers 8`。
从可通的直道和弯道等路段开始，再逐级加入尖角、掉头、缺口和赛道闭环。速度从早期训练开始随机化。

策略输出转向和目标速度（0.45–1.0 m/s）；实车速度由 PID 换算为 PWM。最低速度和路段动力模型表达 ESC 起转平台，策略不直接输出 PWM。训练通过率不能代替 Gazebo 或实车验收。

## 当前车辆参数

轴距 0.33 m；轮距 0.28 m；舵机中位 81；ESC 中位 1540 PWM，前进硬上限 1620 PWM。转向用实测非线性曲线。速度闭环 `kp=8, ki=75, i_limit=80` 有直线试跑记录，转弯和低电量仍需单独验证。

实车 `/teleop_cmd_vel` 使用 PWM 和舵机刻度；Gazebo `/cmd_vel` 使用 m/s 和 rad/s。仿真 ROS 域为 43，实车为 42。每次运行先确认控制发布者唯一。

## 文档入口

- [当前状态和待验证项](docs/README.md)
- [交接资料](docs/handoff/README.md)
- [赛道数据状态](docs/track/TRACK_AD_STATUS.md)
- [文档精简规则](docs/DOCS_GUIDE.md)
- [精简版删除清单](docs/PRUNED_PATHS.txt)
