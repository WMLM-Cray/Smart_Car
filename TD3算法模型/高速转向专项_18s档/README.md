# 高速转向专项(18s 档)

TD3 高速化最终成果:官方赛道第二圈竞速圈速从 35s 档推进到 **平均 18.63s**,完成率 97-100%、入框率 100%。

> 来源仓库:FZU-ZCC-ADS-Lab/Auto_Car,分支 feat/rl-env(提交 24ffe97)
> 口径:FastSim 仿真 + plan 模式(速度由先验速度规划器给出,策略只输出转向),未实车验证。

## 最终成绩(champion: steer2/candidate_039,剖面 a_lat 2.6/急弯 1.8,cap2=2.4)

| 赛道 | 完成率(30 圈) | lap2 圈速 | 入框率 |
|---|---|---|---|
| A | 100% | 19.3s | 100% |
| B | 100% | 18.4s | 100% |
| C | 97% | 18.8s | 100% |
| D | 100% | 17.9s | 100% |

不同限速自适应(cap 1.0/1.5/2.0/2.4 → 平均 34.3/24.0/19.7/18.6s);训练集 1600 局全动作口径 81.6%。
完整演进链(35s→26s→20s→18.6s)与训练发现见 [../RL对比报告.md](../RL对比报告.md) 第 7 章。

## 演进链(四阶段,配置均在本目录)

| 阶段 | 配置 | 关键变化 | 结果 |
|---|---|---|---|
| 激进奖励 | rl_td3_aggr.yaml | crash 50→15、去 time_cost,热启动 fast c008 | 模型敢贴极限 |
| 转向一阶段 | rl_td3_steer.yaml | 采集时 v_plan 剖面(1.8/1.4)接管油门,TD3 只学转向 | 18.9s(2.6/1.8 外推) |
| **转向二阶段** | rl_td3_steer2.yaml | 训练剖面升 2.6/1.8,热启动一阶段 c002 | **18.63s(本档交付)** |
| 转向三阶段 | rl_td3_steer3.yaml | 内评加严 1600 局 + 7 级 | 无净收益(佐证 c039 为局部最优) |

## 内容清单

| 文件 | 说明 |
|---|---|
| rl_td3_*.yaml(4 份) | 各阶段实验配置 |
| 训练数据/train.log、config.json | 二阶段训练全程日志(iter 0-39)与配置 |
| 训练数据/models/candidate_039.npz | **champion(二阶段末轮)** |
| 训练数据/models/candidate_002.npz | 一阶段最优(参照) |
| 训练数据/eval/lap2_s2_c039_*.json | champion 验收原始数据:fix(lap1 修复后)、cap 1.0/1.5/2.0/2.4 限速档、low17/alat28 剖面扫描 |
| 训练数据/eval/lap2_s3_c*.json | 三阶段 7 候选筛选数据(均不超过 c039) |

## 评测命令(FastSim,road_lap2 plan 模式)

```bash
python3 -m evaluation.road_lap2 --model 训练数据/models/candidate_039.npz \
  --tracks A B C D --cap1 0.5 --cap2 2.4 \
  --a-lat-sup 2.6 --a-lat-low 1.8 --v-source plan --a-acc 0.6 --a-dec 0.55 \
  --count 30 --workers 8 --out lap2.json
```

## 风险与限制

- 18.6s 已接近当前仿真物理极限(v_plateau 0.45-0.70、accel_up 0.4-1.0、dead_time 0.28s 下理论极限 A/B≈17s、C/D≈19-21s)。
- plan 模式依赖速度规划器剖面假设(a_lat 2.6/1.8、a_acc 0.6、a_dec 0.55),实车抓地/制动若不足需整体降档。
- cap1.5 时 C 赛道完成率 83%(低速下转向与剖面偶有失配),中速档建议 cap2.0 起步。
- 全部结论仅限仿真,未实车验证。
