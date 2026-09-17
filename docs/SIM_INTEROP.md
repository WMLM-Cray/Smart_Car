# 快仿真(fastsim/)与 Gazebo 仿真(3Dsim/)的对接口径

两套仿真不合并:`fastsim/` 纯 Python、无 ROS,一步 ~1ms,用来训练;`3Dsim/` 是 Gazebo,
贴近实车,用来验证。同一套策略要两边都能跑,只需要下面三个口径一致。对接代码在 `fastsim/interop.py`。

## 1. 控制口径:physical Twist

两边都用**物理单位**,不用 PWM:

| | 本仿真动作 | `/cmd_vel` |
|---|---|---|
| 纵向 | 车速 v(m/s) | `linear.x` |
| 横向 | 前轮转角 δ(rad,左正) | `angular.z` = v·tanδ/L |

`to_cmd_vel(v, steer)` / `from_cmd_vel(linear_x, angular_z)` 做换算,L 取 `wheel_base=0.305`
(两边必须同一个值,否则满舵半径对不上)。实车 `/car_cmd_vel` 收的是 PWM 刻度,那一层在
`fastsim/pwm.py`,不要和这里混。

⚠️ 实车 PWM 公式待订正:`contract.json` 写的是 `linear.x*100+1500`,但车上
`racecar_teleop.py` 的电调中位是 **1550**、前进起转 1600、倒车起步 1360(2026-09-16 读源码),
两者对不上。以车上源码为准。

## 2. 赛道口径:cones.csv + gaps.json

```python
from fastsim.env import ConeTrackEnv
from fastsim.interop import export_track
env = ConeTrackEnv(); env.reset(seed=0)
export_track(env.track, "work/cones.csv", "work/gaps.json")
```
再喂给 `track_generator` 生成 Gazebo world,两边就是同一条赛道:
```bash
python3 3Dsim/hwcart_sim/src/track_generator/track_generator/generate.py \
  --cones work/cones.csv --gaps work/gaps.json --seed 0 \
  --board-thickness 0.1 --output work/track.world
```
导出时赛道已搬到**起点在原点、车头朝 +x**,对齐 launch 里车的出生点。
左侧红 `r0,r1,…`、右侧蓝 `b0,b1,…` 沿行进方向编号;gaps 是同色相邻且间距在
[1.08, 1.5) m 内的位置(`generate.py` 从中随机选 3 处放挡板)。

## 3. 观测口径:48 扇区

策略的输入不是原始 `/scan`,而是压成的 48 个扇区最近距离
(前 ±90° 每 5° 一格,后方每 15° 一格),所以对点数不敏感:
```python
from fastsim.env import sector_index, sectorize
obs48 = sectorize(scan.ranges, sector_index(angles), obs_range=10.0)   # angles 为车体系
```
1667 点(实车/本仿真)与 2000 点(Gazebo)压出来的结果逐扇区一致,只有擦着锥桶边缘的
射线会落进相邻扇区(测试 `test_sectorize_is_independent_of_scan_resolution` 覆盖)。

## 4. 待统一的参数(以实车实测为准)

2026-09-16 在车上量到的值与 `3Dsim/hwcart_sim/src/racecar_sim/config/params.yaml` 的假设值不一致,建议改 Gazebo 侧:

| 项 | params.yaml | 实车实测 |
|---|---|---|
| `lidar_hz` | 10 | **11.99** |
| `lidar_samples` | 2000 | **1667**(`angle_increment`=2π/1667) |
| `imu_hz` | 200 | **100** |
| `lidar_max_range` | 25.0 | 驱动软件裁剪 50.0(硬件量程另计) |

仍未实测、两边都还是假设值:轴距复核、轮径、舵角物理极限、雷达离地高度、锥桶尺寸、
PWM→车速曲线(编码器是落地从动轮,必须落地跑 3–5m 直线才测得到)。
