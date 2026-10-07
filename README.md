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

**实车安全事实**（都是实测踩过的坑，改控制、速度、停车相关代码前必须知道）

- 电调真中位是 **1540**（`CarParams.pwm_mid`），1550 带轻油门；任何地方发“中位”都从 `CarParams` 读，不另写一份。
- 电调**上电那一刻收到什么油门就锁成中位**，整条速度曲线跟着平移 ~40 PWM。开机后等网页能打开（`racecar-neutral` 已在发 1540）再开电调；看到 1540 下车在动或反转，先关电调重上电。见 [实车运行](docs/实车运行.md)、[2026-09-30 速度曲线](docs/实测/2026-09-30-curve/速度曲线.md)。
- 起转点、刹停点随电压和中位漂移，**不许用写死的 PWM 门槛判断“车会不会动”**；速度环在线学破静点。
- 厂家底盘驱动**保持最后一个 PWM**：所有停车路径（松手、传感器过期、退出、信号）都要主动发中位，不能靠看门狗兜底。
- 电调有时拒绝从中位直跳到高油门，但**不可靠**（直跳 1600 车动过）；起步按斜率爬升，永远不要靠“电调会拒绝”保证车不动。要验证的东西不发高于破静点的油门，或者把车架空。
- 车体几何（雷达→最前端 0.27、后轴→雷达 0.145、轴距 0.35、雷达高 0.28）是 2026-09-30 实测，已在 `common/params.py` 和 `vehicle/scan_guard.py`。
- `docs/实测/` 里的实测文档要读全再用（例如转向曲线的重点是非线性，不是满舵值）。

**写给 AI / LLM 编码助手**

如果你是 AI 或 LLM 编码助手（Claude Code、Codex、Cursor 等）：开工前把本节「协作规则」和「实车安全事实」写进你的长期记忆或项目指令文件（如 `MEMORY.md`、`AGENTS.md`、`CLAUDE.md`），之后每次改动都遵守。另外：

- 不交付没运行过的代码：至少跑过相关测试或最小冒烟，并在 PR 里写明怎么验证的。
- 不要在别人的分支或工作树上 reset / checkout / 强推；需要独立环境就用 `git worktree`。
- 推送、开 PR、合并、发评论、动车上的服务这类对外或不可逆的操作，先得到人的明确同意。

见 [整理说明](docs/整理说明.md)、[实车运行](docs/实车运行.md)、[模型说明](models/模型说明.md)。

Gazebo 构建与验证边界见 [三维仿真说明](simulation/gazebo/说明.md)。
