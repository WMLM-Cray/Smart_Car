# 2026-09-16 二维仿真选择性集成(sim/cone-track-env)

任务书范围:从队友分支 `sim/cone-track-env`@**9971895**(已并 origin/main=1dec856,PR #1)
选择性引入 `sim/` 及必要依赖/文档/测试,基线=本地 `integration/trackA-sdf`@381810d;
**不整体合并 main,不引入不启动 web_teleop**。来源已记录:引入文件=`git checkout 9971895 -- sim/ docs/SIM_INTEROP.md`,其余为本轮新写(见提交)。

## 六项任务落点

| # | 任务 | 落点 |
|---|---|---|
| 1 | Gazebo 对齐配置 | `sim/params.py`:wheelbase **0.305→0.36**(对齐主线 params.yaml;0.305 保留为源码候选值待卷尺实测)、lidar_max **50→25**(规则硬件量程;源码 50=软件裁剪记录不改值);主线 12Hz/1667/IMU100 已由 2d87bf3 同步,SIM_INTEROP.md 参数表更新 |
| 2 | 原始扫描 vs 48 扇区 + 周期关联 | `sim/env.py` **IMU 100Hz/encoder 50Hz 独立节流接线**(此前跟随雷达帧,参数表虚设);文档化三周期:physics 100Hz / control 20Hz / scan 12Hz·IMU 100Hz·encoder 50Hz,观测取最近采样;`env.n_scan/n_imu/n_encoder` 计数+测试断言 1s 内 12/100/50 |
| 3 | 随机模式保留 + A 赛道冻结入口 | 随机 make_track 不动;新增 `sim/trackA.py`:`load_trackA()` 读冻结 CSV(63 锥=37蓝+26红,坐标/颜色/出生(11.0,0.75,0)/黄V无锥区语义逐位保留,**不随机化**,任意 seed reset 位姿恒定,有测试);参考线常量与 test_driver 同源,整圈折线长 **36.073m**(逐段实测定标;阶段3"35.46m"是行驶轨迹里程,不同的量) |
| 4 | 两种 CSV 分列说明 | `docs/SIM_INTEROP.md` §4 对照表:字段/坐标系(格式一=起点归一;格式二=赛道世界系)/单位/消费工具;`load_trackA` 严格 schema 校验(id 前缀/颜色词/计数/有限值),不匹配立即报错,测试覆盖三种坏格式 |
| 5 | 同一工作区重跑测试+四工况 | WSL ~/hwcart_sim(同一集成工作区):pytest **30/30 全过**(队友 23 条含 1 条适配 + 新增 7 条);demo 四工况:直行 Δy=0.00000/倒车 Δy=0.00000/**定舵圆周 R拟合 1.1638 vs 理论 L/tanδ=1.1638(0.00%),ω 同 0.00%**/撞锥 0.60s 命中 |
| 6 | 可视化命令/说明/短视频/日志 | 见下表;**结果来源声明:全部为 Pure Pursuit 参考控制器或确定性运动学模型输出,RL 未训练,无任何 RL 结果** |

## 本目录证据

| 文件 | 内容 |
|---|---|
| `sim_tests_运行输出.txt` | pytest -v 全量 30/30 PASS |
| `sim_demo_运行输出.txt` / `sim_demo.json` | 四工况演示+圆周对账(0.00%) |
| `trackA_preview.png` | A 赛道纯追踪一圈静态图(lap/68.7s/撞锥 0/终点横向偏差 2e-6) |
| `trackA_pure_pursuit.gif` | A 赛道纯追踪短视频(230 帧@fps6,1.4MB) |

## 复验入口(WSL,仓库根 ~/hwcart_sim)

```bash
python3 -m pytest sim/tests/ -q                        # 30/30
python3 -m sim.demo                                    # 四工况+R=L/tanδ 对账
python3 -m sim.preview --track trackA --lookahead 0.9 --throttle 0.35 --out /tmp/t.png
python3 -m sim.video --track trackA --lookahead 0.9 --throttle 0.35 --out /tmp/t.gif
```
依赖:python3-numpy/pytest/matplotlib + gymnasium(1.3.0,WSL 用户级 pip)。

## 集成中的两处测试适配(如实记录)

1. **参考线长度定标 36.073m**:连续几何长 36.089(逐段解析),0.05m 离散+各段[1:-1]
   去重复合点后节点跨点直连再 −0.015;首轮手算 36.142 有误,以逐段机器输出为准定标。
2. **满舵测试动作 [1,1]→[1,0]**:轴距 0.305→0.36 后满舵兜圈半径 0.50→0.59m,seed0
   赛道可容纳该圈(实测 120s timeout 兜圈不终止);改用满速直行(实测 2.3s 撞锥)保持
   "不可持续动作→cone/off_track 终止链路可达"的测试本意。兜圈可持续是几何事实非缺陷。

## 保留事项(任务书明示)

- 角度端点约定差异保留:本仿真 2π/1667 不含右端 vs Gazebo 2π/1666 含两端,实车
  LaserScan 完整字段取得前两边都不改(48 扇区观测对此不敏感,有测试)。
- 不调 Gazebo 导航参数;不 push;web_teleop 未引入未审查。
