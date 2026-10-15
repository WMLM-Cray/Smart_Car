# TD3算法模型

基于局部赛道感知的强化学习（TD3）控制策略——训练产物与评测数据归档。

> 来源仓库：FZU-ZCC-ADS-Lab/Auto_Car，分支 feat/rl-env（训练代码提交 7ed0c11）
> 口径：全部数据基于 FastSim 仿真，未实车验证。

## 内容清单

| 文件 | 说明 |
|---|---|
| [RL对比报告.md](RL对比报告.md) | 基线 / PPO / TD3 三列对比，含完整数据表与训练发现 |
| [RL复现.md](RL复现.md) | 从零复现全流程的命令手册（PPO 与 TD3）|
| [rl_td3.yaml](rl_td3.yaml) | 实验配置（双Q + 目标平滑 0.2 + 延迟更新 2 + bc 锚定 10.0 + rounds 8 交错更新）|
| 训练数据/config.json | 本次训练解析后的完整配置 + Git 提交号 |
| 训练数据/train.log | 训练日志（iter 0–78，每 5 轮评估，渐进劣化全程记录）|
| 训练数据/candidates/ | 每 5 轮导出的 15 个模型（各 539KB）；**最优为 candidate_004**（iter4，挑模型种子 eval 96.7%）|
| 训练数据/eval/ | 独立种子评测原始数据：路段评估（log）+ 官方赛道 A–D（JSON）|

## 关键结果（同种子、同口径与 DAgger 基线 / PPO 对比）

- 路段评估（各 500 局）：基线 94.5% / PPO 93.8% / **TD3 94.6%**（三者最高，5 路段中 4 个 ≥ 基线）
- 官方赛道 A–D（--fill-v，cap 0.5–1.0 每档 30 局）：TD3 23/24 档满分（唯一非满分档 A 0.50 为 29/30，撞 1），基线 24/24，PPO 18/24
- 圈速：TD3 在**全部 24 档均快于基线** 0.1–0.6s（cap 1.0 档：A/B/C/D = 35.6/34.4/35.3/34.5s），是唯一在 DAgger 基线之上再提速的模型
- 结论：与基线持平、速度更优；以稳态通过率优先则保留基线，接受低速档 1/30 局碰撞风险则可换 TD3

## 训练发现

- TD3 与 PPO 呈同一模式——**开局即峰、渐进劣化**（TD3 峰值 iter4 = 96.7%，PPO 峰值 iter9 = 97.5%，此后均缓降至提前终止）。两算法互相佐证：问题在奖励结构（crash=50 一次性扣分可被期望速度收益抵消），不在算法本身。
- TD3 特有工程问题：平坦 Q 函数下 actor 的 Adam 优化器累积漂移（冒烟测试策略漂移 2.3 全撞车），以 `bc: 10.0` 锚定初始模型动作压至 0.03；整批采集+集中更新会让策略漂移过远，改为 `rounds: 8` 交错更新后稳定。

## 赛道池说明

训练用的赛道池 `rl_pool.pkl`（4000 局，184MB）超过 GitHub 单文件 100MB 限制，未入库，
与 PPO 实验共用。在源仓库根目录执行一条命令即可完全复现（约 45 分钟）：

```bash
python -m training.road_rl pool --config training/config/rl_td3.yaml --out runs/rl_pool.pkl
```

## 模型加载

```python
from common.road_policy import RoadPolicy

pol = RoadPolicy('训练数据/candidates/candidate_004.npz')  # schema 后缀自带速度契约
pol.reset()                                  # 每局开始时清 5 帧历史
action = pol(obs)                            # 53 维单帧观测 -> [转向, 速度] 各 ±1
```
