# HW-Autocar

OrangePi AIpro + Ubuntu 22.04 + ROS 2 Humble 的无人车项目。本仓库包含实车控制、FastSim 训练、Gazebo 验证和网页遥控。

**先看 [项目文档入口](docs/README.md)。** 它记录当前参数、转角模型研究状态及原始证据的位置。接手项目从 [交接目录](docs/handoff/README.md) 开始；早期方案放在 [历史参考](docs/reference/README.md)，不能直接当作现行车端配置。

## 代码与资料

| 路径 | 内容 |
| --- | --- |
| `fastsim/` | 轻量车辆/赛道仿真、转向策略训练和评估；部分推理模块也在车上使用 |
| `3Dsim/` | ROS 2 / Gazebo 车模、赛道生成与接近实车的验证 |
| `teleop/` | 实车网页遥控、速度闭环、策略节点、看门狗和标定工具 |
| `web/` | 网页界面相关代码 |
| [`docs/`](docs/README.md) | 当前状态、交接、研究计划、赛道资料和验证证据 |
| `runs/` | 已导出的模型权重与评估结果；部分完整训练断点未入库 |

Gazebo 环境与历史构建说明见 [仿真环境记录](docs/reference/SIM_CAR_MODELS.md)；FastSim 与 Gazebo 的接口说明见 [互通记录](docs/reference/SIM_INTEROP.md)。这两份记录含旧参数，运行前以当前代码为准。

## 当前车体与控制口径（2026-09-24）

| 项目 | 当前值 / 验证状态 |
| --- | --- |
| 前后轴距 `wheelbase` | **0.33 m**，用户复量后确定，FastSim 与 Gazebo 参数已统一 |
| 左右轮距 | 与前后轴距不同，尚未在当前记录中确认 |
| 电调真中位 / 前进硬上限 | **1540 / 1620 PWM** |
| 舵机中位 | **81**，精确重复标定仍待做 |
| 直线速度闭环 | `kp=8`、`ki=75`、`i_limit=80`；12.5 V 直线试跑已记录，转弯工况仍待测 |

数值来源见 [`fastsim/params.py`](fastsim/params.py)、[`Gazebo params.yaml`](3Dsim/hwcart_sim/src/racecar_sim/config/params.yaml) 及 [9 月 24 日 PID 实测](docs/validation/2026-09-24-pid/README.md)。车端实际部署文件和服务状态需在车上核对。

## 转角模型现在到哪一步

当前基线是 [`candidate_060.npz`](runs/low_speed_a/candidate_060.npz)。它在旧 FastSim 设定下的 1000 局中完成 956 局，但训练轴距为 **0.305 m**；该成绩不能代表现在的 0.33 m 车体或实车转角表现。Phase B 尚未启动。最大前轮转角和舵机速率仍是仿真假设。

下一步先区分“策略晚打方向”和“车收到命令后转向不足”，再决定沿用旧权重补训还是从零训练对照。诊断顺序和租机准备见 [转角训练计划](docs/research/CORNER_TRAINING_SERVER_PLAN_20260924.md)。旧训练过程见 [历史训练交接](docs/handoff/TRAINING_HANDOFF.md)。

## 仓库与交接

主仓库 `matxn/Auto_Car` 保存完整实车测量和验证数据；队友仓库 `MilkSliceKing/Astrahuaweicar` 是挑选同步的版本，两者的 `main` 不要求有相同提交。交接时注明具体提交号、模型文件、车端部署版本及对应实验数据。详见 [交接目录](docs/handoff/README.md)。
