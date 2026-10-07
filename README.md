# Auto Car

当前链路：实测参数 → 路段仿真 → DAgger → 独立评估。实车节点 vehicle/road_car_node.py(2026-09-29 起开机自启,现跑 road_v2.5/candidate_b036、限速 0.7,见 docs/实车运行.md)。

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


## 协作规则

所有 PR 按这一节审查，不符合的先改再合。

**流程**

1. 先开 issue，写清要解决什么、怎么验收。
2. 从最新的 `main` 拉分支（`feat/…`、`fix/…`、`docs/…`），一个 PR 只做一件事。
3. 开 PR，描述里写 `Closes #N`、做了什么、怎么验证的。
4. 队友审查，通过后在 GitHub 上点 **Approve**。审查通过不等于合并。
5. 由 matXn 合并（merge commit），合并后删远程分支。

**PR 必须在干净检出里能跑**

- 只依赖本仓库的代码，不 import、不读取仓库外的目录（例如别人本机的旧工作区）。
- 不写死本机路径（`/mnt/d/…`、`/home/…`、`C:\…`）；需要外部路径时用命令行参数或环境变量，并给出没设置时的报错。
- 合并前 `python -m pytest -q` 全部通过。测试依赖的文件必须在仓库里，或在缺失时 skip。

**不提交的内容**

- `runs/` 下的运行产物（`.gitignore` 已忽略，不要 `git add -f`）。
- 一次性的探针、实验脚本和中间报告；需要留档的结论写进一份汇总文档。
- 同一个文件的多份历史拷贝：版本交给 git 历史，不要另存 `*_v1.py`、`history/` 目录。
- 大文件和录像（扫描帧、视频等）放仓库外，文档里写清存放位置和校验值。

**单一来源**

- 车体几何、传感器安装位置和参数只从 `common/params.py`（`CarParams` / `TrackParams`）读取，不在别处另写一份。实测改了只改这里。
- 模型的速度契约由 schema 后缀描述，统一用 `common.road_policy.spec_of` 解析；不另起命名。已发布的旧名字在 `ALIASES` 里登记。
- 实车部署件是 `vehicle/road_car_node.py`；描述“部署行为”的代码和测试要以它为准。

**结论要带口径**

- 报成功率或圈速时写清：哪个仿真器（FastSim / Gazebo）、限速、种子段、局数（分母）。
- 两套仿真器的数字不能直接比；有配对结构就做配对比较。
- 只在仿真里验证过的，注明“未实车验证”。

见 [整理说明](docs/整理说明.md)、[实车运行](docs/实车运行.md)、[模型说明](models/模型说明.md)。

Gazebo 构建与验证边界见 [三维仿真说明](simulation/gazebo/说明.md)。
