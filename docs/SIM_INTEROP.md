# 快仿真(sim/)与 Gazebo 仿真(hwcart_sim/)的对接口径

两套仿真不合并:`sim/` 纯 Python、无 ROS,一步 ~1ms,用来训练;`hwcart_sim/` 是 Gazebo,
贴近实车,用来验证。同一套策略要两边都能跑,只需要下面几个口径一致。对接代码在 `sim/interop.py`。

> 本文档 2026-09-16 按集成任务书更新(基线=integration/trackA-sdf 引入 sim/cone-track-env@9971895):
> 轴距/量程对齐主线、时间结构接线、冻结 A 赛道入口、两种 CSV 格式分列。

## 1. 控制口径:physical Twist

两边都用**物理单位**,不用 PWM:

| | 本仿真动作 | `/cmd_vel` |
|---|---|---|
| 纵向 | 车速 v(m/s) | `linear.x` |
| 横向 | 前轮转角 δ(rad,左正) | `angular.z` = v·tanδ/L |

`to_cmd_vel(v, steer)` / `from_cmd_vel(linear_x, angular_z)` 做换算。
**轴距 L 两边统一 0.36m**(2026-09-16 对齐:主线 `params.yaml wheel_base=0.36`,
`sim/params.py wheelbase=0.36`)。厂家另一版源码的 0.305 保留为**候选值**,卷尺实测前
不改已验证车辆基线——实测落地后两边同改。实车 `/car_cmd_vel` 收的是 PWM 刻度,那一层
在 `sim/pwm.py`,不要和这里混。

⚠️ 实车 PWM 公式待订正:`contract.json` 写的是 `linear.x*100+1500`,但车上
`racecar_teleop.py` 的电调中位是 **1550**、前进起转 1600、倒车起步 1360(2026-09-16 读源码),
两者对不上。以车上源码为准(注:1500 是驱动层 `racecar_driver_node.cpp` 的映射,1550 是
teleop 层电调中位,两层关系待实车确认)。

## 2. 时间结构:三个周期,互不混同(2026-09-16 接线)

| 周期 | 频率 | 说明 |
|---|---|---|
| `physics_dt=0.01s` | 100Hz | 运动学积分最内层(与 IMU 100Hz 同频,非巧合但互不依赖) |
| `control_dt=0.05s` | 20Hz | 动作保持:一个 `step()` 内 v/δ 指令恒定,内含 5 个物理子步 |
| `/scan` | 12Hz | 雷达帧,物理子步内按 `lidar_hz` 独立节流 |
| `/IMU_data` | 100Hz | 物理子步内按 `imu_hz` 独立节流(此前跟随雷达帧,已接线修正) |
| `/encoder` | 50Hz | 物理子步内按 `encoder_hz` 独立节流(同上) |

观测取"最近一次采样"。控制 20Hz > 雷达 12Hz:存在连续两个 control step 读到同一帧雷达
——与实车同构(策略 20Hz 决策,雷达 11.99Hz 到达)。刷新计数 `env.n_scan/n_imu/n_encoder`
供对账,测试 `test_sensor_refresh_periods_wired` 断言 1.0s 内 12/100/50 次。

## 3. 赛道口径:随机训练 + 冻结 A 赛道两种模式

**随机训练模式(默认,保留)**:`make_track(rng)` 随机生成双边锥桶环赛道,
可通过 `interop.export_track()` 导出 CSV 喂给 `track_generator`(格式一,见 §4):

```python
from sim.env import ConeTrackEnv
from sim.interop import export_track
env = ConeTrackEnv(); env.reset(seed=0)
export_track(env.track, "work/cones.csv", "work/gaps.json")
```
```bash
python3 hwcart_sim/src/track_generator/track_generator/generate.py \
    --cones work/cones.csv --gaps work/gaps.json --seed 0 \
    --board-thickness 0.1 --output work/track.world
```

**冻结 A 赛道(2026-09-16 新增,不随机化)**:`sim/trackA.py` 从
`docs/track/trackA_cones.csv`(阶段2 v5 冻结基线,63 锥=37蓝+26红)加载,
参考线常量与 `tools/trackA/test_driver.py` 同源(整圈折线长 36.142m;阶段3 记录的
"整圈 35.46m"是车辆实际行驶轨迹里程,两者是不同的量)。语义:
- 坐标/颜色/几何**逐位不动**,两次加载与任意 seed 的 reset 位姿完全一致(有测试);
- 出生位姿固定 `(11.0, 0.75, 0)`——与 Gazebo 验收基线 `sim_bringup` 的 spawn 一致,坐标系同;
- 逆时针行驶:左=红锥(内圈)、右=蓝锥(外圈),与随机模式同约定;
- **黄 V 无锥区语义保留**:E3/E4/I3/I4 段 CSV 本身无锥;黄 V 挡板不参与碰撞判定
  (判罚物=锥桶),参考线走 V 谷折线已避开挡板。

```python
from sim.env import ConeTrackEnv
from sim.trackA import load_trackA
env = ConeTrackEnv(track=load_trackA()); obs, info = env.reset(seed=0)
```

## 4. 两种 CSV 格式(字段/坐标系/单位/消费工具,不许靠文件名猜)

| | 格式一:训练随机赛道导出 | 格式二:A 赛道冻结基线 |
|---|---|---|
| 产生 | `sim/interop.export_track()` | 阶段2 尺寸链重建(人工定案,git 冻结) |
| 文件 | `cones.csv` + `gaps.json` 成对 | `docs/track/trackA_cones.csv` |
| 表头 | `id,color,x,y`(+可选) | `id,color,x,y,segment` |
| id 规则 | `r0,r1,…`(左红)/`b0,b1,…`(右蓝),**从 0 起无补零** | `b01…b37`(蓝外)/`r01…r26`(红内),**两位补零** |
| color 词 | `red`/`blue` | `blue`/`red`(蓝=外圈,红=内圈) |
| 坐标系 | **起点归一**:起点在原点、车头朝 +x | **赛道世界系**:y0=底边、x0=左竖边(与 Gazebo/地图一致) |
| 单位 | m | m |
| gaps | gaps.json:同色相邻间距 ∈[1.08,1.5) 的锥对,generate.py 随机选 3 处放挡板 | 无(挡板位置是冻结几何,不在 CSV) |
| 消费工具 | `track_generator/generate.py` → world | `tools/trackA/trackA_sdf.py` → trackA.world;`sim/trackA.py` → 二维环境 |
| 校验 | generate.py 查 id 正则/颜色/有限坐标 | `sim/trackA.load_trackA()` 严格 schema 校验(63 锥计数、id 前缀、颜色词、有限值),不匹配立即报错 |

两种格式**禁止混喂或静默换坐标**:id 从 0 还是 01 起、颜色内外语义、坐标系原点都不同,
任何转换必须显式过适配器并在文档留痕。

## 5. 观测口径:原始 1667 点扫描 vs 48 扇区压缩(两回事)

- **原始扫描**:`sim/sensors.py` 模拟完整 `/scan`(1667 点,建图 `mapping.py` 也用它)。
- **策略观测**:压成 48 个扇区最近距离(前 ±90° 每 5° 一格=36,后方每 15° 一格=12),
  对点数不敏感:
  ```python
  from sim.env import sector_index, sectorize
  obs48 = sectorize(scan.ranges, sector_index(angles), obs_range=10.0)   # angles 为车体系
  ```

**角度端点约定差异(保留,待实车 LaserScan 完整字段定案)**:
| | 角度轴 | 步进 | 端点 |
|---|---|---|---|
| 本仿真 | `-π + k·2π/1667` | 2π/1667 | **不含 +π**(首尾方向不重复) |
| Gazebo(实测 2026-09-16) | ±π 含两端 | 2π/1666 | **首尾两条射线同方向**(重复方向未消除) |
| 实车 | 未知 | 未知 | 待测 angle_min/max/increment |

48 扇区(5°/15° 粒度)对这 0.216° 级差异不敏感(测试
`test_sectorize_is_independent_of_scan_resolution` 覆盖),策略层两边通用;但**原始点云层
两边不是逐点一致**,拿到实车字段后三边统一,当前不强行改任何一边。

## 6. 已对齐参数(2026-09-16,主线=Gazebo 侧已同步)

| 项 | 主线 params.yaml(Gazebo) | sim/params.py | 依据 |
|---|---|---|---|
| 雷达频率 | 12 Hz | 12.0 | 实车实测 11.99 → 仿真目标取整 |
| 雷达点数 | 1667 | 1667 | 实车确认 |
| 雷达量程 | 25.0 m | 25.0 m | 规则硬件量程 25–30m(源码 50 为驱动软件裁剪,两边都记录不改值) |
| IMU 频率 | 100 Hz | 100.0 | 实车确认 |
| 轴距 | 0.36 m | 0.36 m | 主线已验证基线;0.305 为源码候选值待卷尺实测 |

仍未实测、两边都还是假设值:轴距卷尺复核、轮径、舵角物理极限、雷达离地高度、锥桶尺寸
现场核对、PWM→车速曲线(编码器是落地从动轮,必须落地跑 3–5m 直线才测得到)。

## 7. 演示与可视化入口(全部为 Pure Pursuit / 确定性模型输出)

| 命令(WSL,仓库根) | 说明 |
|---|---|
| `python3 -m sim.demo` | 四工况演示:直行/定舵圆周(与 R=L/tanδ 对账 ≤2%)/倒车/撞锥 |
| `python3 -m sim.preview --track trackA --out t.png` | A 赛道纯追踪一圈静态图(轨迹+末帧雷达) |
| `python3 -m sim.preview --seed 0` | 随机赛道同款预览 |
| `python3 -m sim.video --track trackA --out t.gif` | 纯追踪一圈 GIF 短视频 |
| `python3 -m pytest sim/tests/ -q` | 全部测试(随机赛道+A 赛道+周期接线) |

**结果来源声明**:以上全部输出来自 Pure Pursuit 参考控制器(用赛道真值,只用于预览/测试)
或确定性运动学模型;**RL 策略未训练,本仓库当前没有任何 RL 结果**。操作:改
`--lookahead`(前视距离,m)与 `--throttle`(0~1,乘 v_cmd_fwd)调 aggressiveness,
A 赛道 V 谷急弯建议 lookahead ≤0.9。
