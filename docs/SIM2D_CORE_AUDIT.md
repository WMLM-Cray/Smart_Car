# 二维仿真核心逻辑审查报告(2026-09-17 任务书五-B)

> 审查对象:HEAD `09fa2f9` 时的 `sim/`(env/vehicle/sensors/raycast/track/trackA/
> pwm/interop/mapping/curriculum + 既有测试),修复与新增验证见文末变更表。
> 所有结论标注证据类型:【代码】静态审读 /【测试】pytest 可复现 / 【实测】跑出的数字。

## 1. 碰撞模型

| 项 | 审查结论 | 处置 |
|---|---|---|
| 全车体 vs 中心点 | 【代码】原本就是车体矩形(0.56×0.35,含后悬 0.13)对锥桶做相交判定,不是中心点 | 无需改 |
| 轴距与外形轮廓 | 【代码】运动学用 wheelbase=0.36,碰撞用外形 length/width/rear_overhang,两套量未混用 | 无需改 |
| 锥桶地面几何 | **原判定用内切圆 r=0.14,官方是 0.28×0.28 方形基座——内切圆是纯漏报方向近似(方形⊇圆,圆撞必方撞;4 角伸出 (√2−1)·0.14≈0.058m 角区漏判)** | 已改:`vehicle.footprint_hits_squares` 分离轴判定(车两轴+世界两轴),外接圆粗筛后精确 SAT;`env.step` 已接线 |
| 方形朝向假设 | 实际摆锥朝向未记录,取世界系轴对齐;朝向差 θ∈[0°,45°] 时投影宽 0.28~0.396m,误差方向随 θ 可正可负 | 如实披露,不引入无据的朝向对齐 |
| 单步穿越漏检 | 【代码+测试】物理子步 0.01s × 物理速度上限 3.0 m/s = 0.03m/步,远小于最小特征尺寸(基座 0.28m);dt=0.01 与 dt=0.002 首撞位置差 < 一个粗步位移(1.5 与 3.0 m/s 两档实测,`test_no_tunneling_at_max_speed`) | 无隧穿 |
| 碰撞 vs 出界 | 【代码】撞锥=reason `cone`(罚 5,可终止);出界=lateral 超半宽+0.5m(罚 10,终止 `off_track`);黄 V 无锥区≠允许出界,判据用中心线投影横向偏移,与"有没有锥"无关 | 已区分,无需改 |
| 最小间隙几何/采样 | 二维环境无"最小间隙"指标(该指标与 5cm 阈值属 Gazebo 导航链验收,不在本层,阈值不动);二维侧最近量是"参考线到锥距"(`test_trackA_centerline_clears_cones`,0.2m 采样,下限 0.40m) | 如实说明 |

证据:`sim/tests/test_core_audit.py`(SAT 与独立暴力多边形相交实现 300 随机位姿逐位
一致;圆/方单向超集关系;对角擦碰场景圆漏方撞;纯追踪 trackA 回归)。

## 2. 传感器模型

- 雷达【代码】:`cast_cones` 解析圆台求交——正是"锥在扫描高度 z=0.21 的圆形截面"
  语义(r(z) 随高度收缩),1667 点、±π、增量 2π/1667、min 0.15/max 25、高斯噪声
  σ=0.01m。锥互相遮挡由"每射线取最近交点"自然处理。
- **未建模(如实披露)**:车体自身对雷达的遮挡(实车雷达装在车上有车身盲区,
  仿真只有锥桶);测距的系统性偏差(只加白噪声);multi-echo/玻璃穿透等硬件行为。
- IMU/encoder【代码】:100Hz/50Hz 独立节流采样,gyro σ=0.002rad/s、encoder σ=0.01m。
- 建图【代码】:走整圈 1667 点原始扫描,与 48 维观测无关——与实车 gmapping
  单独订阅 /scan 同构。

## 3. 观测与 RL 接口(无泄漏审查)

- 48 扇区**只来自模拟扫描** `self.ranges`(`sectorize`),另 5 维 =
  encoder 速度、IMU ω、上一步动作(2)、剩余时间比例——全部是车自身传感器与
  动作回显,**不读中心线/锥真值/未来路线**【代码】。
- 参考线(中心线)只在 Pure Pursuit 控制器内部、preview/video/interactive 的
  **画布 debug 显示**与 reward/termination 中出现——ground truth 用于
  奖励与终止是设计允许,且与策略输入隔离【代码】。
- reset/step 契约【测试】:`check_env`(gymnasium 官方)通过;动作 Box(-1,1,(2,))
  双重 clip;返回 5 元组;同 seed 两次 reset 后 40 步轨迹+观测逐位一致
  (`test_same_seed_replays_identical_trajectory`)。

## 4. 终止 / 截断 / 奖励 / 圈数

- termination(cone/off_track/lap)与 truncation(时间上限)**在 API 上分离**
  (`terminated`/`truncated` 各自返回)【代码】。
- **"满舵兜圈 120s 不终止"事件复核**:轴距 0.305→0.36 后满舵兜圈半径
  0.50→0.59m,seed0 赛道容纳该圈,兜圈可持续是几何事实;当时测试改成满速直行
  验证终止链路(test_sim.py:270 注释自认)。**兜圈可持续≠允许不返回截断**:
  budget 到点 `truncated=True, reason="timeout"`。本轮已补回满舵动作本身的
  断言(`test_full_steer_circling_truncates_at_budget` + demo 工况⑥,
  budget=8s 实测 truncated=True/hits=0),并把 test_sim.py:270 注释更新为
  指向新测试——**不是改判据也不是继续绕开**。
- 奖励 farming【代码】:每步时间惩罚 −0.01(原地抖动净亏);lap +10 只在
  progress≥length 时发一次且立即终止(不能反复刷);倒车跨线 ds 有 wrap
  (`ds<-L/2 加 L` / `ds>L/2 减 L`),起点附近横跳正负抵消,progress 是净值。
- 圈数判定【代码】:**累计沿中心线弧长**(progress),不依赖出生点邻近——
  起点附近横跳不会假 lap,绕行大半圈回起点也不算 lap。

## 5. 时间结构(100/20/12/100/50 Hz)

- 物理 100Hz(control step 内 5 子步)→ 动作保持 20Hz;雷达 12Hz、IMU 100Hz、
  encoder 50Hz 按各自 `next_*` 时刻独立刷新,观测取最近采样【代码】。
- 计数器之外的新行为级证据【测试】:
  - 加速段 encoder/IMU 观测逐控制步变化(`test_observation_encoder_and_imu_update_every_control_step`);
  - 雷达 12<20Hz → 相邻控制步存在同帧复用(噪声下 48 扇区逐位相等⟺同帧),
    实测复用比例落在 (0.1,0.7)(`test_lidar_frame_reuse_fraction_matches_12hz_vs_20hz`)
    ——与实车"20Hz 决策、11.99Hz 雷达"同构,设计如此。

## 6. 倒车与阿克曼语义

- ω=v·tanδ/L:同 δ 下 v 反号 ω 反号【测试 `test_reverse_steering_sign_flips_yaw_rate`】;
- v=0 时 ω≡0(无原地转向);交互键盘层不自造转向语义,原样交给模型
  【测试 `test_manual_reverse_steering_gives_bicycle_model_omega_sign`】;
- 速度→PWM 只在 pwm.py 一层(且是二维专用模型,与实车现役公式为两套配置,
  见 sim/params.py 注释与 contract KNOWN_DIFFERENCE)。

## 7. 理想化假设清单(未实现的实车行为,如实披露)

打滑/轮胎侧偏、悬架与载荷转移、电机温升、电量电压跌落(v_gain 域随机化有、
物理模型无)、雷达车身遮挡与系统偏差、锥桶被撞后的位移(撞即终止,锥不动)、
地面摩擦各向异性。训练结论迁移实车前须过 Gazebo/实车验证,不在本层补。

## 8. 本轮变更与证据索引

| 变更 | 类型 | 证据 |
|---|---|---|
| `vehicle.footprint_hits_squares`(SAT)+`env.step` 接线 | 修正 | test_core_audit.py R1a/R1b/R1c |
| `demo.py` 四工况→六工况(+停止、满舵兜圈截断) | 补验证 | docs/validation/2026-09-17-sim2d/demo6.json(6/6 PASS) |
| `test_sim.py:270` 注释补正+满舵 truncated 专项测试 | 补验证 | test_core_audit.py R2 |
| 观测行为级测试(encoder/imu 逐步、雷达帧复用) | 补验证 | test_core_audit.py R3 |
| `sim/interactive.py` 实时交互入口(键盘/PP/暂停/单步/重置/种子) | 新功能 | test_interactive.py + validation 目录 GIF/截图 |
| trackA 纯追踪回归(新碰撞判定下) | 回归 | preview:60.0s lap/0 撞锥;video:68.7s lap/0 撞锥(与 3b7fbd5 基线一致) |

测试基线:pytest `sim/tests/` **47/47 通过**(WSL python3.10,2026-09-17)。
既有入口(random/trackA/pure_pursuit/preview/video/curriculum/interop)全部保留兼容。
