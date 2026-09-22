# 阶段3 第三步:载入已存地图 + AMCL 定位验证(trackA)

> 任务书原文:"保存地图。重启场景,关闭SLAM和测试驾驶器,加载保存地图,
> 使用AMCL验证定位;先允许人工设置初始位姿。"
> 本步交付:一键启动链 + AMCL 参数 + 定位验收器 + 两张截图 + 验收报告。
> 上一步(第二步 测试驾驶器,commit a7152d3)已产出整圈地图
> `trackA_map_full.pgm/.yaml`(338×114 @0.05m,origin[-11.1,-0.863,0]),
> 本步直接载入该地图,**不含** slam_toolbox、测试驾驶器、teleop/手柄、Nav2。

---

## 1. 场景组成(与任务书逐条对应)

| 任务书要求 | 本步实现 |
|---|---|
| 重启场景 | `localization_bringup.launch.py` → 复用 `racecar_sim/sim_bringup.launch.py`(车+ackermann控制器+接线箱,出生点 11.0,0.75,yaw0 与建图时完全一致) |
| 关闭SLAM和测试驾驶器 | 启动链不含 slam_toolbox / test_driver / teleop / 手柄 / Nav2 / 任何常驻速度发布者(车顶多边形以 `ros2 launch` 参数表为准) |
| 加载保存地图 | `nav2_map_server` 载入包内 `maps/trackA_map_full.yaml`(launch 参数 `map:=` 可换) |
| 使用AMCL验证定位 | `nav2_amcl`(config/amcl.yaml);发布 `/amcl_pose` + `map→odom` TF |
| 先允许人工设置初始位姿 | 两条路径,见 §3 |

节点链:`gzserver` → 控制器(odom→base_footprint TF) → `map_server` → `amcl`
→ `lifecycle_manager_localization`(自动 configure/activate,顺序 map_server→amcl)。

## 2. 坐标系推导(为什么初始位姿是 (0,0,0))

- ros2_control 里程计从**单位元**启动 → 建图时 slam_toolbox 的 map 系与 odom
  系在出生点重合 → **map 系原点 = 出生点 Gazebo(11.0, 0.75)**。
- 本步从同一出生点重启 → 车在 map 系的真实位姿就是 **(0, 0, 0)**;
  真值换算(map = gz − (11.0, 0.75))由验收器执行,仅用于误差统计。
- 自洽性校验:地图 origin x=-11.1 → 地图右缘 = −11.1+338×0.05 = 5.8;
  赛道 Gazebo x∈[0,16.8] → map 系 x∈[−11.0, 5.8] ✓(y 同理)。

## 3. 初始位姿的两条路径

| 路径 | 做法 | 适用 |
|---|---|---|
| **人工交互(任务书首选)** | RViz 工具栏 **2D Pose Estimate** 在地图上点出生点(左下角走廊起点)→ 发 `/initialpose` → AMCL 重新撒粒子 | 演示/重定位;重启后换出生点也能用 |
| **无人值守(等效先给值)** | config/amcl.yaml `set_initial_pose: true` + `initial_pose.(x,y,z,yaw)=(0,0,0,0)` | 自动化验收(本步脚本即此路径);数值即上表推导的出生点位姿 |

两条路径走的都是 AMCL 同一入口(`initialPoseReceived`)。

## 4. AMCL 参数依据(config/amcl.yaml)

- 传感器(冻结值):360°/10Hz/2000 采样/0.15–25.0m/σ=0.01 →
  `laser_model_type: likelihood_field`,`laser_max_range: 25.0`,
  `max_beams: 180`(2000 束抽稀),`z_hit 0.9 / z_rand 0.1 / sigma_hit 0.15`。
- 运动模型:**`robot_model_type: "nav2_amcl::DifferentialMotionModel"`**
  ——⚠ 关键坑:Humble 该参数收**pluginlib 插件类名**,写 ROS1 风格的
  `"differential"` 会让 `initOdometry()` 的 `createSharedInstance` 抛异常,
  `on_configure` 失败,lifecycle 报 `Failed to change state for node: amcl`
  (2026-09-16 实测三轮后由 humble 源码定位;参数名 `max_beams` 也曾被误写为
  `laser_max_beams` 被静默忽略)。
- 帧链:`base_frame_id: base_footprint` / `odom_frame_id: odom` /
  `global_frame_id: map`,话题 `/scan`、`/map`,与 sim_wiring 桥接一致。
- 更新阈值:`update_min_d/a = 0.10/0.10`(低速短程验收取小,多出更新)。
- 粒子:500–2000;`recovery_alpha_slow/fast = 0.001/0.1`。
- AMCL 行为备注(源码+实测):给出初始位姿后**首帧激光即发布首个位姿并
  广播 map→odom TF**(force_publication 路径);**之后**的滤波更新需运动
  超过 update_min_d/a,静止时只重发上一拍 TF——属正常,不是掉线。

## 5. 定位验收器(tools/trackA/amcl_check.py)

身份:本步**验收器**,不是测试驾驶器——不跟踪路线、不做余量预检;
只发固定时长的开环速度(等效人工轻推),运动限于底道走廊安全区
(出生点前 2.5m,右端安全余量到右 U 弯前)。真值(`gazebo_msgs/ModelStates`)
**只用于误差统计,不进 AMCL**。

阶段序列(开环):轻推 0.3m 往返 → 静止 4s → 前进 2.5m → 停 4s →
倒回 2.5m → 静止 4s;每帧 `/amcl_pose` 与真值求平面距离。

判据(全过才 PASS):样本≥5;初始段均值≤0.25m;全程最大≤0.25m;
末段静止均值≤0.15m。

**结果(2026-09-16,docs/track/amcl_check_report.json)**:

| 指标 | 数值 | 判据 | 裕量 |
|---|---|---|---|
| 初始段误差均值 | **0.013 m** | ≤0.25 | 19× |
| 全程最大误差 | **0.066 m** | ≤0.25 | 3.8× |
| 全程 RMS | 0.036 m | — | — |
| 末段静止误差均值 | **0.041 m** | ≤0.15 | 3.7× |
| 样本数 | 43 | ≥5 | — |

结论:**PASS**——载入已存地图后,AMCL 在轻推往返 5m 运动中把定位误差
压在 0.07m 内(车长 0.36m 轴距的 1/5),静止段不漂移。

## 6. 截图(docs/track/)

- `amcl_rviz_0_initial.png`:初始时刻——地图(走廊+环)+红色激光簇+
  车辆模型在底走廊左端+青色粒子云(收敛在车周 ~0.3-0.5m)+
  绿色 AMCL 位姿箭头。
- `amcl_rviz_1_after.png`:轻推往返后——车辆沿走廊右移约 2.5m,
  AMCL 箭头贴合车体、朝向一致,粒子云仍收敛,里程计轨迹呈往复线段。
- RViz 粒子云显示必须用 **`nav2_rviz_plugins/ParticleCloud`**
  (nav2 发布 `nav2_msgs/ParticleCloud`,PoseArray 显示类型不匹配不渲染)。

## 7. 复验入口(WSL,已 source ROS2 与工作区)

```bash
# 交互演示(有屏):RViz 中用 2D Pose Estimate 重给初始位姿亦可
ros2 launch racecar_bringup_sim localization_bringup.launch.py \
    use_gui:=false world:=trackA.world \
    spawn_x:=11.0 spawn_y:=0.75 spawn_yaw:=0

# 无人值守全流程(Xvfb 无头+截图+验收器,约 3 分钟)
bash tools/trackA/amcl_demo_run.sh
# 产物:docs/track/amcl_check_report.json + amcl_rviz_0/1_*.png

# 关键状态核查
ros2 lifecycle get /map_server      # 期望 active
ros2 lifecycle get /amcl            # 期望 active
ros2 topic echo /amcl_pose --once   # map 系位姿(静止时也保持)
```

## 8. 涉及文件

| 文件 | 角色 |
|---|---|
| `hwcart_sim/src/racecar_bringup_sim/launch/localization_bringup.launch.py` | 一键启动(仿真+map_server+amcl+lifecycle+rviz) |
| `hwcart_sim/src/racecar_bringup_sim/config/amcl.yaml` | map_server/amcl/lifecycle 参数 |
| `hwcart_sim/src/racecar_bringup_sim/rviz/localization.rviz` | RViz 布局(Fixed Frame=map,含 2D Pose Estimate 工具) |
| `hwcart_sim/src/racecar_bringup_sim/maps/trackA_map_full.pgm/.yaml` | 包内地图(第二步产物副本,launch 默认载入) |
| `tools/trackA/amcl_check.py` | 定位验收器 |
| `tools/trackA/amcl_demo_run.sh` | 无人值守编排(Xvfb+截图+验收) |
| `docs/track/amcl_check_report.json`、`docs/track/amcl_rviz_0/1_*.png` | 证据 |

—— 阶段3 第三步完;下一步(第四步):Nav2 单点导航。
