# 当前项目状态

更新：2026-09-26。参数以当前代码为准，测量以原始记录为准；旧结论不能视为新版本验收。

## 已确认

| 项目 | 当前值 | 依据 |
|---|---|---|
| 轴距 / 轮距 | 0.33 / 0.28 m | `fastsim/params.py`、Gazebo `params.yaml` |
| 舵机中位 | 81 | `fastsim/params.py` |
| 转向 | 实测非线性刻度表 | [转向测量](validation/2026-09-24-steering/CONCLUSION.md) |
| ESC 中位 / 上限 | 1540 / 1620 PWM | [速度曲线](validation/2026-09-23-curve/README.md) |
| 直线 PID | kp=8, ki=75, i_limit=80 | [闭环记录](validation/2026-09-24-pid/README.md)；转弯和低电量未验收 |

## 当前训练设计

`fastsim.road_train` 是当前训练入口。先生成走廊路段，逐级增加急弯、掉头、缺口，再评估兼容赛道的能力；各训练阶段从早期就随机速度。生成的地图须先通过连通性和车体净空检查，失败地图不进入训练。

策略观测为 53 维、叠 5 帧；输出转向和 0.45–1.0 m/s 目标速度。实车 PID 将速度目标转换为 PWM。ESC 最低起转平台要体现在训练动力模型和最低速度约束中，不用固定 PWM 代替速度输出。

训练代码存在不代表模型已验收。报告必须注明代码提交、配置、种子、速度档和逐局结果。FastSim 用于筛选，Gazebo 与实车单独验收。

## 赛道数据边界

A–D 文件放在根目录 `maps/`。坐标来自规则图估读；A 边界尚未与逐桶位置完成校准，B–D 尚未完成尺寸和全程通行验收。具体限制见 [赛道状态](track/TRACK_AD_STATUS.md)。不得将估读地图描述为官方精确场地图。

## 入口

- [FastSim](../fastsim/road_train.py)：训练实现；`road_check.py`：可通性检查；`road_eval.py` / `road_track.py`：评估。
- [实车控制](../teleop/steer_policy_node.py)、[速度闭环](../teleop/speed_loop.py)。
- [Gazebo](../3Dsim/hwcart_sim/)；实车和仿真接口单位不同，运行前核对 ROS 图。
- [实验记录](validation/)、[研究文档](research/)、[交接](handoff/README.md)、[历史资料](reference/README.md)。
