> 历史参考：本文件保留写作时的记录；当前参数和状态见 [项目文档入口](../README.md)。

# 低速转向策略 —— 交接文档

写于 2026-09-22。读者:接手训练这条线的人。
详细的课程设计和预算见 [LOW_SPEED_TRAINING_PLAN.md](../reference/LOW_SPEED_TRAINING_PLAN.md),这里不重复,
只讲**东西在哪、怎么跑起来、现在到哪一步、接下来该改什么**。

## 0. 一句话状态

phase A 训完 3M 步,`candidate_060` 在 fastsim 的 s3 赛道上 1000 局完赛率 0.9560;
但在官方赛道 A 上 0 圈 —— 失败点不是转向,是赛道 A 有训练分布里不存在的东西。
**不建议从零重训**,理由见第 6 节。

---

## 1. 仓库

| 远端 | 地址 | 谁的 |
|---|---|---|
| `matxn` | github.com/matXn/Auto_Car | 本人(matXn / Oyzze) |
| `origin` | github.com/MilkSliceKing/Astrahuaweicar | 队友 MilkSliceKing |

两个远端的 `main` 保持一致,推的时候两边都推。

**还有一批东西只在 `origin` 的分支上,没合进任何 main**,接手前值得扫一眼:

```
origin/integration/trackA-sdf       官方赛道 A 几何 + 素材库(锥桶/挡板/交通灯/标志牌 SDF)
origin/feat/ekf-divergence-audit    EKF 750m 发散定位 + 编码器 CRC 修复(林锦锋)
origin/feat/nav-viz                 Nav2 纯追踪 + 静态地图 + RViz(林锦锋)
origin/feat/map-loc-diag, laq       racecar_diag 诊断工具(MilkSliceKing)
origin/fix/map-export-thresh        teleop/export_map.py 的 free_thresh 修正,只改 3 个文件
```

赛道 A 的 `trackA_layout.yaml` 已经拷进 `3Dsim/examples/`,不用切分支也能用。

## 2. 目录分工

```
fastsim/    轻量仿真 + 训练(纯 Python + numpy,训练要 torch)   ← 这条线
teleop/     实车部署节点、网页遥控、标定工具(要 rclpy,只在车/WSL 上跑)
3Dsim/      队友的 Gazebo 工作区 + 赛道生成 + 评估工具
runs/       训练产物。runs/* 默认不入库,例外见 .gitignore 注释
```

`fastsim/` 的四个模块**跑在实车上**(`teleop/steer_policy_node.py` import 它们):
`low_speed_policy.py`(推理 + observe)、`obs.py`(48 扇区)、`params.py`、`pwm.py`。
改这四个文件等于改车上的代码。

## 3. 环境

**训练在 Windows 上,不需要 ROS。**

```
python  C:\Users\Tozzy\AppData\Local\Python\bin\python.exe
        numpy 2.4.4   torch 2.11.0+cpu(没有 CUDA)
```

**仿真验证在 WSL 里。**

```
wsl -d Ubuntu-22.04           用户 tozzy,免密 sudo,systemd 已开
/opt/ros/humble               ROS2 Humble + Gazebo Classic 11.10.2
~/hw_ws/                      colcon 工作区(从 3Dsim/hwcart_sim/src rsync 过去的)
~/autocar/                    fastsim / runs / teleop 的副本
~/worlds/                     赛道 world
```

装环境:`bash 3Dsim/tools/wsl_setup_ros2.sh`(2026-09-21 修过取密钥那段,别用旧版)。
重建记录和踩过的坑见 [SIM_CAR_MODELS.md](../reference/SIM_CAR_MODELS.md) 的「WSL 环境重装」节。

**别把 `~/hw_ws` 软链到 `/mnt/c`** —— colcon 在 9p 文件系统上编译慢一个数量级。
代价是仓库改了代码要 rsync 一次。

## 4. 怎么跑

训练(Windows):

```
python -m fastsim.low_speed_train --out runs/low_speed_a --phase A --steps 3000000
python -m fastsim.low_speed_train --out runs/low_speed_a --resume          # 断了接着跑
```

大验证(和训练分开,多进程):

```
python -m fastsim.low_speed_eval runs/low_speed_a/candidate_060.npz --count 1000
```

Gazebo 里跑赛道 A(WSL,四步):

```bash
# 1 生成锥桶(直线 0.5m / 弧 0.4m,黄V默认留空)
python3 3Dsim/tools/gen_trackA_cones.py 3Dsim/examples/trackA_layout.yaml \
    -o work/trackA/cones.csv --spacing 0.5 --curve-spacing 0.4
# 2 转 SDF(--boards 0:赛道A规格里没有挡板,而且间距太密塞不下)
python3 3Dsim/hwcart_sim/src/track_generator/track_generator/generate.py \
    --cones work/trackA/cones.csv --gaps work/trackA/no_gaps.json \
    --seed 0 --board-thickness 0.05 --boards 0 --output work/trackA/trackA.world
python3 3Dsim/make_sim_world.py work/trackA/trackA.world \
    "$WS/src/racecar_sim/worlds/trackA_dense.world"
(cd "$WS" && colcon build --packages-select racecar_sim)
# 3 起仿真。world 收的是**包内文件名**不是绝对路径(2026-09-22 合队友基线后改的);
#   起点必须给,赛道A的起点不在原点。队友包里自带一份 trackA.world(63 锥,
#   冻结 CSV 坐标),想跑那份就 world:=trackA.world
ros2 launch racecar_sim sim_bringup.launch.py world:=trackA_dense.world \
    spawn_x:=11.0 spawn_y:=0.75 spawn_yaw:=0.0
# 4 策略 + 心跳 + 打分(三个终端)
python3 teleop/steer_policy_node.py runs/low_speed_a/candidate_060.npz --sim --speed 0.45
python3 teleop/hold_enable.py
python3 3Dsim/tools/track_eval.py 3Dsim/examples/trackA_layout.yaml \
    --cones work/trackA/cones.csv --seconds 150
```

两个容易踩的:

* **`/policy_enable` 是心跳不是开关**,`ros2 topic pub --once` 只能让车抖一下,
  必须 `hold_enable.py` 持续发。`3Dsim/run_sim.sh` 里那行 `--once` 是个 bug,已于
  2026-09-22 修掉(改成起 `hold_enable.py`)。
* **nohup 撑不住仿真** —— `wsl.exe` 一退出,WSL 把会话里的进程全收掉,gzserver 没了,
  重定向的日志文件一个字节都没有,看上去像根本没启动过。得有个 `wsl.exe` 前台挂着。

## 5. 现在到哪一步

phase A:固定 0.6 m/s,3M 步,60 轮。200 局验证:

```
020=0.845  035=0.920  040=0.875  045=0.870  050=0.925  055=0.940  060=0.960
```

`candidate_060` 1000 局:**clean_rate 0.9560,Wilson 95% CI [0.9414, 0.9671]**,
956 完圈 / 26 撞锥 / 18 出界,`teacher_failed_on_student_failures: 0`(学生失败的局
老师都能过,所以不是赛道本身过不去)。

phase B 没开始。入口门槛写在 `low_speed_train.py` 的 `phase_a_validation()`:
要求同一份权重上有 ≥200 局的完整验证报告且 clean_rate ≥0.95,sha256 对得上。

**但 0.9560 这个数是乐观的**,原因见下。

## 6. 五个已知问题(按我判断的严重度)

### 6.1 评估种子和训练重叠

训练里每轮用 `30000000 + [0, 50)` 算 `clean_rate`,跑了 60 轮,挑权重就是挑这个数 ——
等于在同一个验证集上做了 60 次模型选择。而 `low_speed_eval.py:81` 的 1000 局验证
**也是从 `30000000` 开始**,前 50 个完全重叠。

**先做这个**:换个种子基(比如 `50000000`)重跑 1000 局,看 0.9560 掉多少。
不先做这个,后面改完没法比。

### 6.2 训练赛道分布不覆盖赛道 A

训练**不重复赛道** —— 种子是 `10000000 + index`,每局 +1 从不回头,每局重新生成一条
随机赛道,`seed % 2` 时还整条镜像。所以不存在"记住了某条赛道"。

问题是**分布**。训练用 `s1`(1/3)+ `s3`(2/3),`s3` = `RuleTrackParams`。赛道 A 有三样
它从没见过:

| | 训练分布 | 赛道 A |
|---|---|---|
| 弯型 | 45/90/135° 折角 + 短 S,半径 1.0~2.0m | **持续 180° 定曲率掉头**,内径 1.25 / 外径 2.75 |
| 锥桶间距 | `cone_spacing = 1.15` 固定 | **0.5**(2026-09-22 按比赛录像改的) |
| 无参考区 | 没有 | **5.64m 黄 V 缺口**,铺胶带不放锥桶 |

2026-09-22 在 Gazebo 里实测(密锥桶 142 个,0.40/0.45/0.50 m/s 各一次):

```
三次全 0 圈,停在内场同一处 (2.3, 3.1)
右端 180° 掉头走得干净 —— 转向本身没问题
到黄 V 的肩点就直插内场:那 5.64m 两侧是胶带,雷达看不见,
48 个扇区在那个方向全空,策略没有锥桶就直着开
撞锥 0 —— 它是从缺口里开出去的,一个都没碰
```

轨迹图和评分见 `3Dsim/tools/track_eval.py` 的输出(带 `path` 字段)。

### 6.3 `max_steer` 是没量过的假设

`fastsim/params.py` 的 `max_steer = 0.55` 标着 `[假设]`,而 `steer_policy_node.py` 里
`steer = action * self.p.max_steer` —— 它直接乘在**每一次**转向输出上。真实舵角比
0.55 小的话,车实际转得比策略要的少,表现就是"转向太迟"(用户在实车上的原话)。

**把舵机打到满舵量一下前轮角度,十分钟的事**,比任何训练改动都便宜。
同一条链上 `turn_span = 60` 是 `[代码]`(舵机左右各 60 算满舵),所以量了就能定。

### 6.4 PWM→速度随电量平移,写死的映射都不可靠

同一台车、同一个电调,两次实测差了 95 个 PWM:

| | 2026-09-21 落地(电量偏低) | 2026-09-22 架空(电量足) |
|---|---|---|
| 起转 PWM | 1650 | **1555** |
| 1600 | 读数 0,带不动 | 转得很快 |
| 1610 | — | 目测 ≈2 m/s |

负载差异解释不了 95 —— 空载少了静摩擦和滚阻,起转点最多差 20~30。**厂商确认**是
电量低时同样 PWM 扭矩不够,不是电调有固定死区,并建议改用编码器速度 PID 闭环。

所以 `fastsim/params.py` 里 `pwm_mid 1550 [代码]`、`pwm_fwd_start 1600 [代码]`、
`v_fwd_full 3.0 [假设]` 全都不可靠,那里"低于 1600 会主动刹车"的注释是**错的**。
目标 0.6 m/s 大概在 1565~1575(起转点往上十几个),不是现在配的 1600。

这条不影响 fastsim 训练(训练里用物理量),但影响任何 PWM 映射和上车。
车上 `teleop/steer_policy_node.py` 的 `--pwm` 默认 1600、`PWM_HARD_MAX = 1620`
都要按落地实测重定。

### 6.6 轴距是 0.36 不是 0.305

2026-09-22 合入队友基线时发现,他们 2026-09-16 **卷尺量过**:

```
hwcart_sim/src/racecar_sim/config/params.yaml
  wheel_base: 0.36    # [实测 2026-09-16] 出厂源码 0.305 与本车不符,已关闭
```

而我们这边 `fastsim/params.py` 的 `wheelbase = 0.305 [假设]`、
`teleop/scan_guard.py` 的 `WHEELBASE = 0.305` 还是抄的出厂值 —— **差 18%**。
轴距进阿克曼几何:`angular.z = v·tanδ/L`(`fastsim/interop.py`)、防撞的扫掠
路径半径 `R = L/tan δ`、最小转弯半径,全都跟着错。

同一份 params.yaml 里 `lidar_samples 1667`、`lidar_hz 12` 和我们独立测到的一致,
所以那份实测是可信的。**订正 0.305→0.36 之前,训出来的策略在实车上的转向半径
就是偏的。** 这条应该排在 6.3(max_steer)前面一起做。

### 6.5 黄 V 是能力缺口不是调参问题

雷达打不到地上的胶带。过这一段只有三条路:相机认胶带(`/image_raw` 有,640×480
带出厂标定内参,但整条视觉链路是空的)、航位推算、或者训练时就把无参考区建模进去。

## 7. 方向:改分布,不从零重训

结论:**不重训。** phase A 学到的转向先验是有效的(180° 掉头走得干净,s3 上 0.956),
丢掉重来没道理。要改的是赛道生成器的**参数分布**,不是网络也不是拓扑。

具体三条,都是在 `RuleTrackParams` / 赛道生成里加参数,不动拓扑、不动评估口径:

1. **弯角加 180°**,半径范围覆盖 1.25 和 2.75
2. **`cone_spacing` 每局随机抽**(比如 0.4~1.2)而不是固定 1.15 —— 顺带解决"间距到底
   是多少"这个还没定论的问题,让策略对它不敏感
3. **每局随机挑一段做无锥桶区**(3~6m),模拟胶带

每一条都直指赛道 A 上实测出来的失效点。

### 关于"缩短赛道"和"改成非闭合"

* **缩短赛道 —— 值得做。** 每局 budget 是 `track.length / speed * 1.5 + 15`,0.6 m/s 下
  一条 35m 的赛道要近 90 秒仿真时间。缩短等于同样步数里见到更多条不同赛道,直接提高
  种子多样性,不改任何口径。
* **非闭合赛道 —— 不建议作为主力。** 闭环不是过拟合来源(赛道每局都换、还镜像),
  而比赛本身就是闭环两圈。开放赛道没有"跑完一圈"这个干净的完赛判据,`clean_rate`
  要重新定义,phase A/B 的门槛跟着改 —— 收益不明显,代价是把已经稳定的评估口径推倒。

### 改完怎么接着训

分布变了,旧 replay 里那 1M 帧是旧分布的,不该直接 `--resume`。建议**新开一个目录**,
用 `candidate_060` 的权重热启动、replay 从零。

**但 CLI 现在不支持这个** —— `--init` 只认 phase B(`low_speed_train.py`:
`elif args.init is not None: ap.error('--init is only supported for phase B')`)。
要么放开这个限制,要么就认了从随机初始化重训。这是个已知的实现缺口。

## 8. 别做的事(踩过的坑)

* **别把 `ROS_DOMAIN_ID` 写成 42。** 42 是实车,仿真是 43。同网段下 DDS 会把两边的
  话题按名字合并,`/scan` 凭空多一个发布者,**不报错不告警**,只能靠数点数发现
  (实车 1667 点 12.1Hz,Gazebo 2000 点 8.1Hz)。见 SIM_CAR_MODELS.md。
* **别用 Gazebo 菜单的 Reset World(Ctrl+R)。** 它把仿真时间倒回 0,
  `gazebo_ros2_control` 从此不再注册 update 回调,车变砖且救不回来,只能重启整个 launch。
  要复位用 `teleop/sim_reset.py`(走 `/set_entity_state` 传送,不动时间)。
* **别改 `steer_policy_node.py` 的 `PWM_HARD_MAX`(1620)。** 那是 2026-09-21 架空试跑
  冲太快之后定的安全线,不是标定值。跑不动就用占空比降速,不要抬上限。
* **同一时间只能跑一个策略节点。** 两个都发 `/teleop_cmd_vel` 会互相冲掉,不报错 ——
  2026-09-21 `pwm_sweep` 被没停的 `racecar-policy` 冲成一张全 0 的表,查了半天。
* **50 个种子分辨不了 0.95 这个阈值**(95% 区间约 ±0.06)。训练内的 `eval_<round>.json`
  只能当趋势看,过关判据必须用 ≥200 局的 `low_speed_eval` 报告。
