# 旧低速策略交接（历史摘要）

原文日期：2026-09-22。仅供理解旧低速转向模型，不作为当前训练或上车步骤。当前方案见[项目状态](../README.md)和[路段训练计划](../research/CORNER_TRAINING_SERVER_PLAN_20260924.md)。

当时的 `candidate_060.npz` 在旧 FastSim 设定下 1000 局完成 956 局；Gazebo 赛道 A 未完成。报告将其中一个主要原因定位为黄色 V 胶带区没有锥桶，雷达无法观测该区。该结果不代表当前 0.33 m 轴距模型或实车通过率。

旧文档中的赛道生成命令、PWM 映射、远端仓库、路径、候选评估种子和阶段计划均可能已失效。不要按旧命令启动车辆或继续旧训练流程。

相关证据：

- [第一圈候选摘要](2026-09-24/FIRST_LAP_ITERATION_20260924.md)
- [车辆测量与 PID](../validation/2026-09-24-pid/README.md)
- [赛道 A–D 数据边界](../track/TRACK_AD_STATUS.md)

完整原文可恢复：

```bash
git show 66814ff:docs/handoff/TRAINING_HANDOFF.md
```
