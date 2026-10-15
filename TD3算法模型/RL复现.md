# RL(PPO/TD3)复现说明

从干净克隆的仓库出发,完整走一遍「基线评测 → RL 训练 → RL 评测 → 模型加载」。
所有命令都在仓库根目录执行;默认实验配置是 [training/config/rl_ppo.yaml](../training/config/rl_ppo.yaml)(PPO)
与 [training/config/rl_td3.yaml](../training/config/rl_td3.yaml)(TD3),
命令行参数可覆盖配置中的任意一项,解析后的完整配置连同 Git 提交号写进 `runs/*/config.json`。

> 口径:以下全部流程基于 **FastSim 仿真**(仿真端赛道进度奖励,不依赖预设赛车线/中心线),
> **未做实车验证**。种子划分(互不重叠):DAgger 训练 10M / 挑模型 50M / RL 赛道池 20M / 独立评测 90M。

## 1. 环境安装

```bash
git clone git@github.com:FZU-ZCC-ADS-Lab/Auto_Car.git
cd Auto_Car
python -m pip install -r requirements.txt   # numpy scipy torch PyYAML pytest
python -m pytest -q                          # 全量测试应全部通过
```

## 2. 复现 DAgger 基线(先于 RL,同条件比较的基准)

```bash
# 路段评估:模型 = 当前实车基线
python -m evaluation.road_eval --model models/road_v2.5/candidate_b036.npz --level L5 --count 500
# 官方赛道 A–D(圈速 + 通过率)
python -m evaluation.road_track --model models/road_v2.5/candidate_b036.npz --tracks A B C D --fill-v
```

评测种子默认从 90M 起(road_eval)/ 95M 起(road_track),与训练种子不重叠。

## 3. RL(PPO)训练

```bash
# 3.1 生成赛道池(老师先确认每局可通,一次生成反复用)
python -m training.road_rl pool --config training/config/rl_ppo.yaml --out runs/rl_pool.pkl

# 3.2 PPO 训练(从 DAgger 模型初始化;--iters 0 只导出初始化模型可用于对照)
python -m training.road_rl train --config training/config/rl_ppo.yaml \
    --pool runs/rl_pool.pkl --init models/road_v2.5/candidate_b036.npz \
    --out runs/road_v2.5_rl --workers 7
```

- 奖励权重(`crash/time_cost/slip/max_steps`)、赛道混合、种子、PPO 超参全部在 yaml 里改,不在代码里改。
- 训练日志:`runs/road_v2.5_rl/train.log`;每 `eval_every` 轮存一个 `candidate_xxx.npz` 并在挑模型种子上评估。
- 想换实验:复制一份 yaml 改名(如 `rl_ppo_crash10.yaml`)再 `--config` 指过去,不要覆盖基准配置。

## 4. RL(TD3)训练

```bash
# 4.1 赛道池与 PPO 共用(生成过一次就不用再生成)
# python -m training.road_rl pool --config training/config/rl_td3.yaml --out runs/rl_pool.pkl

# 4.2 TD3 训练(双Q + 目标平滑 + 延迟策略更新 + bc 锚定 + rounds 交错更新)
python -m training.road_td3 train --config training/config/rl_td3.yaml \
    --pool runs/rl_pool.pkl --init models/road_v2.5/candidate_b036.npz \
    --out runs/road_v2.5_td3 --workers 7

# 4.3 单元测试
python -m pytest training/test_rl_td3.py -q
```

- 与 PPO 完全同池、同奖励、同评测口径,差异只在算法侧(超参见 [rl_td3.yaml](../training/config/rl_td3.yaml))。
- 两个工程要点(冒烟期踩过):平坦 Q 下 actor 的 Adam 会累积漂移,靠 `bc: 10.0` 锚定初始模型压住;
  整批采集+集中更新会让策略漂移过远,靠 `rounds: 8` 交错更新解决。
- 训练日志:`runs/road_v2.5_td3/train.log`,每 `eval_every` 轮存 `candidate_xxx.npz`。

## 5. RL 评测(与基线同条件)

```bash
python -m evaluation.road_eval --model runs/road_v2.5_rl/candidate_299.npz --level L5 --count 500
python -m evaluation.road_track --model runs/road_v2.5_rl/candidate_299.npz --tracks A B C D --fill-v
# TD3 同理,模型路径换成 runs/road_v2.5_td3/candidate_004.npz
```

指标:完成率(goal 率)、圈速、结束原因分布(collision/off_track/timeout)、最小边界余量。
结果填进 [RL对比报告](RL对比报告.md);不能只看训练端累计奖励。

## 6. 加载导出的策略

```python
from common.road_policy import RoadPolicy

pol = RoadPolicy('runs/road_v2.5_rl/candidate_299.npz')  # schema 后缀自带速度契约
pol.reset()                                              # 每局开始时清 5 帧历史
action = pol(obs)                                        # 53 维单帧观测 -> [转向, 速度] 各 ±1
```

部署/实车链路(未实车验证的部分如实注明)见 [模型运行](模型运行.md) 与 `vehicle/road_car_node.py`。

## 7. RL 环境适配层(给新实验用的接口)

`training/rl_env.py` 的 `RlEnv` 把 FastSim 的 `RoadEnv` 包装成标准 RL 语义,
DAgger 训练与既有评测流程不受影响:

```python
from training.rl_env import RlEnv

env = RlEnv(road, pl, seed, start, crash=50.0, time_cost=0.02, slip=0.5, max_steps=3000)
obs = env.reset()                        # (265,) 已叠 5 帧
obs, reward, terminated, truncated, info = env.step(action)   # action = [转向, 速度] ±1
# info: reason(goal/collision/off_track/timeout) progress min_clear time cap slip
```

## 8. 记录与复现要求

- 每次训练的 `runs/*/config.json` 已含全部解析后配置 + Git 提交号,报告结论时引用它。
- 验收门槛与同条件对比的口径见 [RL对比报告](RL对比报告.md)。
