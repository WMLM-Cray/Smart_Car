# 两个 Gazebo 车模的区别(2026-09-20 核对)

结论:**用 `3Dsim/hwcart_sim/src/racecar_sim`,不要用 `ackermann_gazebo`。**

## 两者是什么

- **`racecar_sim`** —— 队友 MilkSliceKing 照着 `docs/SOURCE_AUDIT.md` 建的我们这辆车。
  在本仓库 `3Dsim/hwcart_sim/src/racecar_sim/`(commit `ed613bd`,2026-09-15,
  "阶段1仿真车,42 项验证全绿";`a6d7b26` 挪进 `3Dsim/`)。
  xacro 全部从 `config/params.yaml` 取值,每项标了 `[源码]/[规则]/[实测]` 出处。
- **`ackermann_gazebo`** —— 第三方通用阿克曼 demo(README:"Simple ros2 humble
  ackermann drive that use default gazebo ackermann plugin",自带 LICENSE 和 2024 年截图)。
  在 `C:\Users\Tozzy\Downloads\racecar_仿真\` 里,**不是厂家给我们车的模型**。

## 参数对照

| | ackermann_gazebo | racecar_sim | 实车 |
|---|---|---|---|
| 轴距 | 0.455 | **0.305** | 0.305 [出厂代码] |
| 最大转角 | 0.84 rad | **0.55** | 0.55 |
| 雷达角度范围 | 0..2π | **−π..+π** | −π..+π [实测] |
| 雷达点数 | 360 | 2000 | 1667 [实测] |
| 雷达频率 | 50 Hz | 10 Hz | **11.97 Hz** [实测] |
| 底盘质量 | 25 kg(+4×3kg 轮) | 8 kg | — |
| 控制话题 | `/cmd_vel`(m/s + rad) | 号称与实车同名 | `/car_cmd_vel`(PWM) |

轴距 0.455 是从 `ackermann_gazebo` 的 URDF 量出来的(前转向关节 x=0.247、
后轮关节 x=−0.2078,parent 都是 `chassis_link`)。0.305 的出处是厂家自己的控制器:
`racecar_05_27/src/racecar/src/car_controller_new.cpp:118`
`this->declare_parameter("L", 0.305);  // 车辆前后轴距`。
两个数属于两辆不同的车,不是同一辆车的两种测量 —— 卷尺复核仍值得做,但先信 0.305。

## 赛道

`racecar_sim/worlds/racecar.world` 是空的(30 行,只有 ground_plane + sun)。
`3Dsim/hwcart_sim/src/track_generator/` 能把锥桶坐标 CSV 转成 SDF 世界,但
README 自标"原型,官方 A-D 坐标待做",示例 `3Dsim/examples/cones.csv` 只是
8 个锥桶排成 3.6m 直走廊。要跑规则赛道得自己喂坐标(`fastsim/track_rules.py` 能出)。

`ackermann_gazebo` 那边的 `saidao.world` 是 15 段 2.5m 高的连续墙 + 3 个锥桶,
策略认的是 1.5m 宽的离散锥桶门,连续墙是分布外输入。

## 把 fastsim 的赛道喂给 racecar_sim(三步)

1. **fastsim 赛道 -> 锥桶 CSV**(`fastsim/interop.py` 的 `export_track`):

   ```
   python -c "import sys;sys.path.insert(0,'.');from fastsim.curriculum import STAGES;\
   from fastsim.interop import export_track;e=STAGES['s3'].make_env(mapping=False);\
   e.reset(seed=90000);print(export_track(e.track,'runs/track_s3/cones.csv','runs/track_s3/gaps.json'))"
   ```

   只挑间距落在 1.08~1.5m 的位置当挡板候选 —— 正好是 generate.py 那条
   "0.8m 挡板要塞得进两个 0.28m 锥桶之间"的判据,两边不用对口径。

2. **CSV -> SDF world**(队友的 `track_generator/generate.py`,commit 721afcc):

   ```
   python 3Dsim/hwcart_sim/src/track_generator/track_generator/generate.py \
     --cones runs/track_s3/cones.csv --gaps runs/track_s3/gaps.json \
     --seed 0 --board-thickness 0.05 --output runs/track_s3/track_s3.world
   ```

3. **补上 racecar_sim 需要的插件**(`3Dsim/make_sim_world.py`):

   ```
   python 3Dsim/make_sim_world.py runs/track_s3/track_s3.world runs/track_s3/track_s3_sim.world
   ```

   然后 `ros2 launch racecar_sim sim_bringup.launch.py world:=<绝对路径>`。
   不带 world 参数时行为和以前完全一样(仍是自带的 racecar.world)。

s3 种子 90000 出来是 63 个锥桶(左红 27 / 右蓝 36)、59 处候选挡板位、
16.0 x 8.1m 的闭环,起点在原点朝 +x,和 spawn 点对齐。实测低速策略
(1.5m/s)在这条赛道上连跑 100s / 140.7m 约 4.7 圈没停,首圈 23.7s。

## WSL 环境(已装好,2026-09-19)

发行版 `Ubuntu-22.04`,用户 `tozzy`,ROS2 Humble + Gazebo Classic 11 + colcon。
启动脚本 `~/sim.sh`,三处环境变量都是踩坑才加的,见脚本内注释:

1. `/usr/share/gazebo/setup.sh` 会覆盖 `GAZEBO_MODEL_DATABASE_URI`,必须在它之后再改;
   设成空串无效(Gazebo 回落默认值),要给一个立刻被拒的地址,否则 gzserver 卡在
   连 `models.gazebosim.org`,`/clock` 不发、`/spawn_entity` 永不返回。
2. Mesa 的 d3d12 默认挑 Intel 核显,那条路径跑 Gazebo 会 `D3D12: Removing Device`;
   `MESA_D3D12_DEFAULT_ADAPTER_NAME=NVIDIA` 点名 4060 后不再崩,RTF 1.00。
3. `SIM_SOFTWARE=1 ~/sim.sh` 退回软件渲染。

另注:`gazebo_ros` 的 `/clock` 固定 10 Hz,`publish_rate` 无论从命令行还是运行时
设都不生效。所以 `policy_node.py --sim` 用墙钟而不是仿真时间 —— 否则定时器快不过
10 Hz,而策略是按 20 Hz 训的。前提是 RTF≈1,跑之前 `gz stats -p` 看第一列。

## ROS 域隔离(2026-09-20 踩坑后加)

| 场合 | `ROS_DOMAIN_ID` | 在哪设 |
|---|---|---|
| 实车 | **42** | 车上 `~/.bashrc` + `systemctl set-environment` |
| 仿真 | **43** | `3Dsim/run_sim.sh`、`sim_reset.bat` |
| 默认 | 0 | 谁都能进来,别用 |

起因:车和笔记本第一次同时挂在同一个 iPhone 热点上,局域网里另一台机器
(`DESKTOP-8HRN3AF`)在跑 `racecar_sim`。ROS2 的 DDS 按多播自动发现,
`ROS_DOMAIN_ID` 相同 + 同网段 = 同一个 ROS 图,两个世界的话题按名字合并:

```
/scan  pub=2   实车雷达 1667 点 12.1 Hz  +  仿真雷达 2000 点 8.1 Hz
/IMU_data pub=2      /teleop_cmd_vel pub=2      /clock 车上 11 个订阅者
```

**不报错也不告警**。`ros2 topic hz` 只会显示 20.1 Hz,看着还挺健康 ——
要靠数每帧点数才能把真假帧分开。这状态下跑策略,车收到的是真假交替的雷达,
而对方的仿真车会执行发给实车的指令。

排查时的两个弯路,记下来:

1. 先怀疑本机 WSL 的 Gazebo,杀掉后假帧照旧。**WSL2 是 NAT 网络,车根本
   发现不到 WSL 里的 DDS** —— 本机仿真反而是唯一安全的那个。
2. `wsl -l -v` 有三个发行版,一开始只查了两个就下了"本机干净"的结论。

另:车上 sudo 只有 `systemctl` 免密,改不了 unit 文件,所以用
`systemctl set-environment`。**这个不跨重启**,要永久生效得写 drop-in:

```
sudo systemctl edit racecar-chassis   # 加 [Service] / Environment=ROS_DOMAIN_ID=42
```

三个服务都要:`racecar-chassis`、`racecar-web`、`racecar-watchdog`。
