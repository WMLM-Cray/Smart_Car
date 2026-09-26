# HW-Autocar：ROS 2 无人车与仿真项目

本项目围绕 OrangePi AIpro 小车、Ubuntu 22.04 和 ROS 2 Humble，覆盖 **Gazebo 车模与赛道、传感器和建图定位、导航、FastSim 策略训练、实车控制及网页/手柄遥控**。各方向的代码和实验记录都保留在本仓库；仿真、离线验证与实车验收是不同阶段。

**新成员先读这里了解全貌，再看 [项目文档入口](docs/README.md)了解当前参数与研究状态；分工和旧成果从 [交接目录](docs/handoff/README.md)进入。** 带日期的早期方案集中在 [历史参考](docs/reference/README.md)，其中部分参数已被实测更新。

## 1. 各方向在哪里

| 方向 | 代码入口 | 目前可用的内容和边界 |
| --- | --- | --- |
| Gazebo 车模与控制 | [`3Dsim/hwcart_sim/src/racecar_sim/`](3Dsim/hwcart_sim/src/racecar_sim/) | XACRO 四轮车、`ros2_control` 阿克曼控制器、雷达/相机/IMU、启动及接线节点；需要在 ROS 2 Humble + Gazebo Classic 环境运行 |
| 赛道与比赛素材 | [`track_generator/`](3Dsim/hwcart_sim/src/track_generator/)、[`racecar_test_assets/`](3Dsim/hwcart_sim/src/racecar_test_assets/)、[`racecar_strategy/`](3Dsim/hwcart_sim/src/racecar_strategy/) | 锥桶 CSV→SDF、赛道世界、模型素材和罚则；赛道几何与地图依据在 [`docs/track/`](docs/track/) |
| SLAM、定位、Nav2 | [`racecar_bringup_sim/`](3Dsim/hwcart_sim/src/racecar_bringup_sim/)、[`3Dsim/tools/realcar/`](3Dsim/tools/realcar/) | 仿真启动、地图/定位/导航工具及实车候选安全链；是否已在车上部署、是否通过动力导航，须按各轮记录分别核对 |
| FastSim 与策略 | [`fastsim/`](fastsim/)、[`runs/`](runs/) | 轻量环境、教师、低速转向训练与评估、导出权重；FastSim 成绩不能代替 Gazebo 或实车验收 |
| 实车控制和网页 | [`teleop/`](teleop/) | HTTP/WebSocket 遥控、雷达显示、速度闭环、转向策略、看门狗、标定脚本和 systemd 配置记录 |
| 手柄遥控 | [`racecar_teleop/`](3Dsim/hwcart_sim/src/racecar_teleop/) | 队友的 `/joy`→`/teleop/cmd_vel` 物理速度接口；与实车网页的 PWM 接口不同，接入前需确定唯一的控制仲裁路径 |
| 网页界面资料 | [`monitor/`](monitor/) | 独立界面与预览资料；车上的 HTTP 服务入口是 `teleop/server.py` |
| 仿真与实车对照 | [`3Dsim/tools/sim2real/`](3Dsim/tools/sim2real/)、[`docs/sim2real/`](docs/sim2real/) | 场景和动力学假设对照，按报告的参数版本解释结果 |

本仓库主线保存完整实车测量与验证数据。队友的 `MilkSliceKing/Astrahuaweicar` 仓库同步的是选定部分，两个仓库的 `main` 不保证提交号相同。

## 2. ROS 接口：先辨认运行环境与单位

`geometry_msgs/Twist` **仅说明消息格式，不保证字段单位相同**。以下是当前代码中的主要入口；实际车端部署版本和 ROS 图仍要在现场核对。

| 话题 | 类型 | 主要发布/订阅与语义 |
| --- | --- | --- |
| `/scan` | `sensor_msgs/LaserScan` | 雷达发布；网页、防撞、策略、SLAM 订阅。实车约 12 Hz、每帧 1667 点；Gazebo 配置目标为 12 Hz/1667 点 |
| `/encoder` | `nav_msgs/Odometry` | 实车编码器速度；网页与实车转向策略订阅。Gazebo 策略改订 `/ackermann_steering_controller/odometry` |
| `/IMU_data` | `sensor_msgs/Imu` | IMU 发布，策略、网页和定位链使用；Gazebo 配置为 100 Hz，车端就绪检查近期记录约 55 Hz，现场以实测为准 |
| `/image_raw` | `sensor_msgs/Image` | Gazebo 相机输出；实车相机话题、编码和持续出帧须现场确认 |
| `/odom_combined` | `nav_msgs/Odometry` | 厂家 EKF 的融合里程计输出；SLAM/定位链需要同时核对相应 TF |
| `/tf` | `tf2_msgs/TFMessage` | 发布坐标变换；Gazebo 的 `sim_wiring` 将控制器专用里程计 TF 转到此话题 |
| `/map` | `nav_msgs/OccupancyGrid` | SLAM 或地图服务发布；建图网页/监视工具与定位导航链使用 |
| `/policy_enable` | `std_msgs/Bool` | 网页给策略节点的持续使能心跳；松手或心跳中断应停止策略输出 |
| `/teleop_cmd_vel` | `geometry_msgs/Twist` | **当前实车网页与低速策略的 PWM 通道**：`linear.x` 为电调 PWM、`angular.z` 为舵机刻度；看门狗也在此通道工作 |
| `/cmd_vel` | `geometry_msgs/Twist` | **Gazebo 的物理量通道**：`linear.x` 为 m/s、`angular.z` 为偏航角速度 rad/s；Nav2 候选链也用物理量 |
| `/car_cmd_vel` | `geometry_msgs/Twist` | 实车 Nav2 候选安全中继输出的物理速度/偏航角速度，厂家驱动再换算；是否启用及驱动公式须核对车上实际版本 |
| `/joy` → `/teleop/cmd_vel` | `sensor_msgs/Joy` → `geometry_msgs/Twist` | 队友手柄包：输出物理速度和偏航角速度，**不等于** `/teleop_cmd_vel` 的 PWM 刻度 |

当前两条常见控制路径：

```text
实车网页/低速策略 ──PWM Twist──▶ /teleop_cmd_vel ──▶ 底盘驱动
Gazebo 策略/Nav2   ──物理 Twist──▶ /cmd_vel ──▶ sim_wiring ──▶ 阿克曼控制器
```

实车 Nav2 候选路径见 [`cmd_vel_safety.py`](3Dsim/tools/realcar/cmd_vel_safety.py)。任何一次运行都应确认只有预期的控制发布者，避免网页、策略、手柄或导航同时争用车辆。仿真使用 `ROS_DOMAIN_ID=43`，实车使用 `42`，避免同网段 ROS 图混在一起。接口历史与单位换算见 [`SIM_INTEROP`](docs/reference/SIM_INTEROP.md)；其中旧数值须以当前参数为准。

## 3. 几何与控制的现行数值

| 项目 | 当前记录 |
| --- | --- |
| 前后轴距 `wheelbase` | **0.33 m**，用户复量确定；`fastsim/params.py` 和 Gazebo `params.yaml` 已采用 |
| 左右轮中心距 `wheel_track` | **0.28 m**，用户确认；Gazebo `params.yaml` 已采用，与前后轴距是两项尺寸 |
| 电调真中位 / 前进硬上限 | **1540 / 1620 PWM** |
| 舵机中位 | **81**；更长距离的重复直线标定仍可补做 |
| 实车直线速度闭环 | `kp=8`、`ki=75`、`i_limit=80`，已有 12.5 V 直线试跑证据；转弯负载尚需复测 |

参数依据见 [`fastsim/params.py`](fastsim/params.py)、[`Gazebo params.yaml`](3Dsim/hwcart_sim/src/racecar_sim/config/params.yaml) 和 [实测记录](docs/validation/2026-09-24-pid/README.md)。部分早期手柄、Nav2 测试工具仍带旧 0.36 m 轴距常量；在使用那些路径前须单独核对，不能把“核心参数已更新”理解为所有历史脚本都已重标定。

## 4. 新成员怎样进入各方向

### Gazebo 与赛道

在 Ubuntu 22.04 / ROS 2 Humble / Gazebo Classic 环境中，先构建 [`3Dsim/hwcart_sim/`](3Dsim/hwcart_sim/)，再使用 `racecar_sim` 的 [`sim_bringup.launch.py`](3Dsim/hwcart_sim/src/racecar_sim/launch/sim_bringup.launch.py)。启动参数包括 `world`、`use_gui` 和 `spawn_x/y/yaw`；赛道 A 的世界文件在 `racecar_sim/worlds/trackA.world`。环境安装、历史构建问题和验证入口分别见 [WSL 环境记录](docs/reference/WSL_ENVIRONMENT.md)、[仿真车记录](docs/reference/SIM_CAR_MODELS.md) 与 `3Dsim/tools/verify_phase1.sh`。旧报告中的验收数字对应其当时提交，不代表本次参数更改后已重跑。

```bash
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=43
cd 3Dsim/hwcart_sim
colcon build --symlink-install
source install/setup.bash
ros2 launch racecar_sim sim_bringup.launch.py use_gui:=false
# 赛道 A：world:=trackA.world spawn_x:=11.0 spawn_y:=0.75 spawn_yaw:=0.0
```

现场核对实际发布者、订阅者和帧率可用 `ros2 topic info /scan -v`、`ros2 node info /sim_wiring`、`ros2 topic hz /scan`。不要只凭本文的话题表推断正在运行的 ROS 图。

### 建图、定位与导航

`racecar_bringup_sim/launch/` 分别提供 SLAM、定位和导航启动文件；`3Dsim/tools/realcar/` 有实车侧地图、相机、定位健康及安全中继工具。[赛道 A 资料](docs/track/) 和 [验证证据](docs/validation/) 记录各阶段结果。Nav2 仿真通过、无动力检查、实车动力验收是不同结论，不能混用。项目当前没有把这些候选导航工具概括成“实车已稳定完赛”。

### 训练、评估与上车

当前路段训练入口为 `python -m fastsim.road_train`，从初期随机速度并检查路段可通性，输出转向与目标速度。旧模型保留作对照。历史低速训练入口为 `python -m fastsim.low_speed_train`，评估入口为 `python -m fastsim.low_speed_eval`；Gazebo 验证使用 [`steer_policy_node.py`](teleop/steer_policy_node.py) 的仿真模式。当前导出权重是 [`candidate_060.npz`](runs/low_speed_a/candidate_060.npz)，在旧 **0.305 m** 训练假设下得到 FastSim 956/1000 圈；改成 0.33 m 后尚未取得同等验收。Phase B 未启动。现有转角问题与服务器准备见 [研究计划](docs/research/CORNER_TRAINING_SERVER_PLAN_20260924.md)。

### 实车网页与手柄

车上网页服务在 [`teleop/server.py`](teleop/server.py)，默认 HTTP 端口 8080；它订阅雷达、编码器和 IMU，并在手动模式发布 `/teleop_cmd_vel`，策略模式发送 `/policy_enable` 心跳。启动配置记录在 [`teleop/systemd/`](teleop/systemd/README.md)。队友手柄包另发 `/teleop/cmd_vel` 物理量，接入方案与早期验收见 [遥控集成文档](docs/reference/TELEOP_INTEGRATION_GUIDE.md)；实际车端选用哪条链须现场核对。

## 5. 文档和交接

- [docs/README.md](docs/README.md)：当前参数、研究状态与资料导航。
- [docs/handoff/](docs/handoff/README.md)：接手入口、训练/遥控/sim2real 历次交接。
- [docs/validation/](docs/validation/)：逐帧测量、日志、报告和回归证据。
- [docs/track/](docs/track/)：赛道几何、地图与仿真资料。
- [docs/sim2real/](docs/sim2real/)：二维实车约束对照。
- [docs/reference/](docs/reference/README.md)：保留原方案和审查记录；内容以写作日期为准。

向新成员交接时同时给出 **Git 提交号、运行环境、模型权重、参数版本、使用的话题链和对应证据**；仅提供一份旧操作卡不足以复现现状。
