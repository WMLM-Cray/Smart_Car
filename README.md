# 室外 ROS 无人车仿真系统(hwcart_sim)

> 目标:在 PC 上用 Gazebo 复现整个比赛(锥桶赛道+车+雷达+相机),
> 传感器话题/TF 与实车**完全同名**,算法代码一套、仿真实车两端通用。
> **当前状态:阶段 1(仿真车)完成,42 项一键验证全绿**(2026-09-15,完整日志见 `evidence/`)。

## 1. 环境要求(与实车一致)

| 项 | 值 |
|---|---|
| OS | Ubuntu 22.04(虚拟机 / WSL2 / 双系统均可) |
| ROS2 | Humble Desktop |
| 仿真器 | Gazebo Classic 11(`ros-humble-gazebo-ros-pkgs`) |
| 后续 | Nav2、slam_toolbox(阶段 3 才需要) |

环境自检:`python3 tools/check_environment.py`(输出 `ready: true` 即可)。
WSL2 从零装环境可跑 `tools/wsl_setup_ros2.sh`。

## 2. 目录结构

```
├── hwcart_sim/                  # ROS2 colcon 工作区(在这里构建)
│   └── src/
│       ├── racecar_sim/         # 仿真车(阶段1 ✅):XACRO 车模+阿克曼控制器+雷达/相机/IMU
│       ├── racecar_bringup_sim/ # launch 集合骨架
│       ├── track_generator/     # 赛道生成器:锥桶坐标 CSV → SDF 世界(原型,官方 A-D 坐标待做)
│       └── racecar_strategy/    # 比赛罚则库(撞锥5s/压线10s/闯红灯30s…,已测)
├── tools/                       # 环境自检/构建/一键验证/诊断脚本
├── docs/
│   ├── TELEOP_INTEGRATION_GUIDE.md  # 手柄遥控包对接文档(接 teleop 必读)
│   └── SOURCE_AUDIT.md          # 实车接口审计(PWM 公式/话题/参数出处)
├── examples/                    # 锥桶坐标、赛道间隙示例数据
├── tests/                       # 基础测试(pytest)
└── evidence/                    # 验证完整日志存档(未截断)
```

## 3. 快速开始

```bash
cd hwcart_sim
colcon build --symlink-install
source install/setup.bash

# 带 Gazebo 窗口:
ros2 launch racecar_sim sim_bringup.launch.py
# 无头模式(服务器/跑验证):
ros2 launch racecar_sim sim_bringup.launch.py use_gui:=false
```

RViz 配置文件:`hwcart_sim/src/racecar_sim/rviz/sim.rviz`。

⚠️ **改过 Python 包文件后必须清理重建**:`rm -rf build install log && colcon build`
(colcon 增量构建不会删 install 里已不存在的旧文件,会跑旧代码出假结果)。

## 4. 接口契约(与实车同名)

| 话题 | 类型 | 说明 |
|---|---|---|
| `/cmd_vel` | `geometry_msgs/Twist` | 控制输入。**物理单位**:linear.x=m/s,angular.z=rad/s |
| `/scan` | `sensor_msgs/LaserScan` | 激光雷达(对齐实车镭神 M10P:10Hz、50m 裁剪) |
| `/image_raw` | `sensor_msgs/Image` | 相机 640×480(出厂标定内参,含畸变) |
| `/IMU_data` | `sensor_msgs/Imu` | IMU 200Hz |

- 链路:`/cmd_vel` → `cmd_vel_relay` → `ackermann_steering_controller`(阿克曼 ω=v·tanδ/L)
- 车辆参数集中在 `hwcart_sim/src/racecar_sim/config/params.yaml`:
  - 轴距 `wheel_base: 0.305`(出厂代码值,**待实车卷尺复核**)
  - 相机工作模式 640×480/标定内参/hfov≈72.21°(设备标称 1080p/150° 是镜头能力,非工作模式)
  - 未实测项均带 `[实测假设]` 标记

## 5. 一键验证(42 项,headless)

```bash
bash tools/verify_phase1.sh
# 期望输出:===== [8/8] 结果: PASS=42 FAIL=0 ===== / ALL_GREEN
```

覆盖:启动链/阿克曼几何(直行、倒车、左右满舵)/TF 树/传感器频率与量程/障碍测距/超时停车 watchdog/负例(超速/超角拒绝)。失败会逐项给 FAIL 与证据;**取不到证据即 FAIL**,不用估算值替代。

## 6. 手柄遥控(teleop)对接

**接 teleop 包前必读 `docs/TELEOP_INTEGRATION_GUIDE.md`**(含编译/契约/两方案对比/接入日清单/验收清单/常见问题)。三条底线:

1. teleop 输出 `/teleop/cmd_vel` 是物理单位 Twist,不是 PWM 刻度;
2. 主线接入用**方案 B(仲裁节点)**:teleop 与自动导航各发各的话题,仲裁后输出唯一 `/cmd_vel`,沿用现有 relay——不要直接 remap 绕过仲裁;
3. 接入日把 wheelbase 三处统一 0.305(teleop 配置/主线 params/反解 L)并重跑端到端测试(满舵期望 ω≈0.5984 rad/s)。

## 7. 实车迁移注意(阶段 6)

- 实车厂家接口 `/car_cmd_vel` 收的是 **PWM 刻度**(速度 1500+100x、转向 1500−1300z),**不是物理单位**——必须加转换层,公式与出处见 `docs/SOURCE_AUDIT.md` 和 `hwcart_sim/src/racecar_sim/racecar_sim/contract.json`;
- 待实测参数:前后轮距、轮径、轮宽、质量、惯量、舵角物理极限、雷达物理高度(卷尺+秤一刻钟);轴距复核出厂值 0.305;
- 实体手柄在 WSL2 下需 usbipd-win 直通(详见对接文档"实体手柄与 WSL2"节)。

## 8. 已知限制

- WSL2 下 Gazebo spawn 偶发慢:验证脚本已记录就绪等待时长并自动重试一次(判据不变);
- 2026-09-15 曾出现一轮 41/42 且原因未定位(后续三轮未复现),已按残余风险记录;最新完整全绿日志在 `evidence/`;
- 官方 A-D 赛道锥桶坐标未入库(阶段 2 任务)。
