# RL(PPO)与 DAgger 基线对比报告

> 状态:**骨架待填** —— 下表由实跑评测生成后填写,不提前写结论。
> 口径:FastSim 仿真,未实车验证。评测命令与种子见 [RL复现](RL复现.md)。

## 1. 实验条件(必填)

| 项 | 值 |
|---|---|
| Git 提交(基线) | e5381f3(models/road_v2.5/candidate_b036.npz) |
| Git 提交(RL 训练) | (填 runs/*/config.json 里的 git_commit) |
| 实验配置 | training/config/rl_ppo.yaml(如改过权重,注明与基准的差异) |
| 赛道池 | runs/rl_pool.pkl,pool_seed0=20M,共 N 局 |
| 评测种子 | road_eval 90M 起 / road_track 95M 起,与训练种子不重叠 |

## 2. 路段评估(同 seed、同 level、同 cap 分布)

| 模型 | success | by_reason(collision/off_track/timeout) | by_cap | 局数 |
|---|---|---|---|---|
| DAgger 基线 candidate_b036 | | | | 500/level |
| RL candidate_xxx | | | | 500/level |

## 3. 官方赛道 A–D(每档 cap 30 局,2 圈)

| 模型 | A 通过/圈速 | B 通过/圈速 | C 通过/圈速 | D 通过/圈速 |
|---|---|---|---|---|
| DAgger 基线 | | | | |
| RL | | | | |

## 4. 结论(如实)

- RL 是否优于/持平/劣于基线:(待填;若未达基线水平,如实写明,不宣称超越)
- 训练端累计奖励上升不作为有效性的依据,以本报告第 2/3 节的闭环指标为准。

## 5. 验收门槛(沿用仓库现有评测标准)

- 路段课程晋升门槛沿用 `road_train` 的 `success ≥ 0.95`(全部 level 且分 level 均达标)。
- 官方赛道口径:`--fill-v`(黄 V 以外部分)每档 cap 30 局,通过率与圈速与基线同表对比;
  达标线由团队评审后在配置/本报告中固化,不藏在代码里。

## 6. 风险与限制

- FastSim 抓地力 0.8g 为假设值,RL 会利用仿真与实车的差异(见 road_rl.py 文档字符串)。
- 本报告全部结论仅限仿真,未实车验证。
