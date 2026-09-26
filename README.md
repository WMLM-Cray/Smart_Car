# Auto Car 精简版

当前链路：实测参数 → 路段仿真 → DAgger → 独立评估。实车网页与旧版转向策略仍保留，road_v2 尚未接入实车。

| 目录 | 内容 |
|---|---|
| common | 车辆参数、转向映射、观测与策略推理 |
| simulation/fastsim | 快速车辆、雷达、碰撞与路段仿真 |
| simulation/gazebo | ROS 2 / Gazebo 最小验证工程 |
| training | 老师、可通性筛选与 DAgger |
| evaluation | 路段及 A–D 评估 |
| vehicle | 网页、旧版策略节点、PID、看门狗 |
| calibration | 现用实测工具 |
| maps | A–D 图示重建数据，评测用途 |
| models | 运行基线与模型说明 |
| runs | 自动生成的运行产物 |
| docs | 中文说明与实测依据 |

安装：`python -m pip install -r requirements.txt`。实车另需 ROS 2 Humble 和厂商驱动。

```sh
python -m pytest
python -m training.road_train --help
python -m training.road_train --out runs/road_trial --workers 14 --torch-threads 3
python -m evaluation.road_eval --help
python -m evaluation.road_track --model runs/road_trial/candidate_001.npz --tracks A B C D --fill-v
```

共享服务器所有任务合计不得超过约定的 17 核。训练命令本身不设置 CPU 配额。

见 [整理说明](docs/整理说明.md)、[实车运行](docs/实车运行.md)、[模型说明](models/模型说明.md)。

Gazebo 构建与验证边界见 [三维仿真说明](simulation/gazebo/说明.md)。
