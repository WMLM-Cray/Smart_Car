# PPO算法模型

基于局部赛道感知的强化学习（PPO）控制策略——训练产物与评测数据归档。

> 来源仓库：FZU-ZCC-ADS-Lab/Auto_Car，分支 feat/rl-env（训练配置提交 7c9f2c1）
> 口径：全部数据基于 FastSim 仿真，未实车验证。

## 内容清单

| 文件 | 说明 |
|---|---|
| [RL对比报告.md](RL对比报告.md) | 最终结论：RL(candidate_009) 全面略逊于 DAgger 基线，含完整数据表 |
| [RL复现.md](RL复现.md) | 从零复现全流程的命令手册 |
| [rl_ppo.yaml](rl_ppo.yaml) | 实验配置（保守化 PPO 超参：clip 0.15 / epochs 2 / lr 5e-5）|
| 训练数据/config.json | 本次训练解析后的完整配置 + Git 提交号 |
| 训练数据/train.log | 训练日志（iter 0–59，每 5 轮评估，渐进劣化全程记录）|
| 训练数据/candidates/ | 每 5 轮导出的 12 个模型（各 539KB）；**最优为 candidate_009**（eval 97.5%）|
| 训练数据/eval/ | 独立种子评测原始数据：路段评估（log）+ 官方赛道 A–D（JSON）|

## 关键结果

- 路段评估（各 500 局）：基线 94.5% vs RL 93.8%
- 官方赛道 A–D（--fill-v，cap 0.5–1.0 每档 30 局）：基线 24/24 档全通过、lap2 35.0–36.1s；RL 21/24 档、圈速慢 0.2–1.4s
- 训练发现：两轮训练均在 40–60 轮后渐进劣化，指向奖励结构问题（详见报告第 4 节）

## 赛道池说明

训练用的赛道池 `rl_pool.pkl`（4000 局，177MB）超过 GitHub 单文件 100MB 限制，未入库。
在源仓库根目录执行一条命令即可完全复现（约 45 分钟）：

```bash
python -m training.road_rl pool --config training/config/rl_ppo.yaml --out runs/rl_pool.pkl
```

## 模型加载

```python
from common.road_policy import RoadPolicy

pol = RoadPolicy('训练数据/candidates/candidate_009.npz')  # schema 后缀自带速度契约
pol.reset()                                  # 每局开始时清 5 帧历史
action = pol(obs)                            # 53 维单帧观测 -> [转向, 速度] 各 ±1
```
