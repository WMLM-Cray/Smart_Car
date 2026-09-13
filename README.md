# 室外 ROS 无人车仿真系统(hwcart_sim)

> 目标:在 PC 上用 Gazebo 复现整个比赛(锥桶赛道+车+雷达+相机),
> 二维(SLAM 建图+Nav2 导航)与三维(Gazebo 场景+视觉感知)全链路仿真,
> 传感器话题/TF 与实车完全同名,**算法代码一套、仿真实车两端通用**。
> 同时作为技术手册的明星案例(创新性+复现性得分点)。

## 环境要求(与实车严格一致)

| 项 | 值 |
|---|---|
| OS | Ubuntu 22.04(虚拟机/WSL2/双系统) |
| ROS2 | Humble Desktop |
| 仿真器 | Gazebo Classic 11(humble 官方配套) |
| 导航 | Nav2(ros-humble-navigation2 + nav2-bringup) |
| SLAM | slam_toolbox(首选) / slam_gmapping(出厂自带) |

## 计划目录结构(ROS2 标准工作区)

```
07-仿真系统/
├── README.md          ← 本文件
├── .gitignore
└── hwcart_sim/        ← ROS2 colcon 工作区(在 Ubuntu 里构建)
    └── src/
        ├── racecar_sim/        # 仿真车:XACRO 车模+阿克曼插件+雷达/相机/IMU 传感器插件
        ├── track_generator/    # 赛道生成器:坐标 CSV → SDF 世界(锥桶/挡板/红绿灯/标志牌,可随机化)
        ├── racecar_bringup_sim/# 仿真 launch 集合(对齐实车 Run_car/Run_nav)
        └── racecar_strategy/   # 行为状态机:第一圈建图/第二圈冲刺/红绿灯停车/A·B 停车
```

## 队伍仓库调研结论(2026-09-13,Astra-Rover)

**结论:无可直接复用的仿真代码,仿真仍按本仓库 Gazebo+ROS2 方案走;但有可搬的软资产。**

Astra-Rover 是队友的自研校园车(RK3576+ESP32-S3,LD06 雷达+TOF 阵列+IMX415),
与昇腾比赛车硬件/软件栈完全不同;其 sim/ 是纯 Python 平面运动学 RL 仿真
(无相机成像、非 ROS2/Gazebo),链路与本比赛 SLAM+Nav2+视觉完全不同,不适用。

可搬的三样软资产:
1. **实验记录格式**:`docs/方向D_决策_实验记录.md` 的"结论速览表+证据强度+作废标记"
   —— 调 Nav2 参数时套用,直接产出技术手册高质量案例素材;
2. **参数扫描方法学**:根目录 12 个 `sweep_*.json`(改参数→跑仿真→JSON归档→对比)
   —— 阶段3调 Nav2/MPPI 参数时照此管理实验,技术手册"复现性"得分点;
3. **有数据的具体结论**:①激光运动畸变主要来自转弯而非车速(1.13m vs 0.07m),
   第二圈提速时 EKF/雷达去畸变要重点验;②相机 FOV≥120° 很关键(90° 到达率 57%→21%),
   比赛 150° 广角够用但仿真相机 FOV 别配小;③图像去畸变对传统规划器是必做项
   (DWA 20%→32%),仿真+实车视觉链路都要保留去畸变步骤;
4. (参考)感知选型教训:YOLOP 在非行车记录仪域会崩,选模型先看训练集域
   —— 锥桶/红绿灯检测选型时先确认训练数据场景匹配。

## 开发路线(六阶段)

| 阶段 | 内容 | 状态 |
|---|---|---|
| 0 | 搭 Ubuntu22.04+Humble+Gazebo 环境 | ⬜ |
| 1 | racecar_sim 仿真车(TF/传感器参数抄实车,见 ../04-出厂代码/出厂源码解读.md) | ⬜ |
| 2 | track_generator 赛道生成器(按规则图3/图9 尺寸,A/B/C/D 四种赛道) | ⬜ |
| 3 | 二维链路:/scan → SLAM → /map → Nav2 → 两圈导航 | ⬜ |
| 4 | 视觉链路:红绿灯 / A·B 标志牌 / 锥桶颜色 | ⬜ |
| 5 | 行为状态机 + 全流程比赛仿真 | ⬜ |
| 6 | 实车迁移(同话题名,只调参数) | ⬜ |

## 对齐实车的关键参数速查

- 雷达:2D,360°,10Hz,25m,frame=laser_link,话题=scan
- 相机:usb_cam,frame=camera,话题按实车 camer.launch.py
- TF:base_footprint→base_link(z0.15) / laser_link(x0.07) / IMU_link(x0.1653)
- 底盘:阿克曼转向(舵机+后驱),不是差速!仿真插件要用 ackermann
- Nav2:Run_nav.launch.py 已支持 use_sim_time:=true,仿真直接复用

## 版本管理约定

- 本目录是 git 仓库,在 Ubuntu 侧改代码后 commit+push(仓库地址待建)
- ⚠️ 本目录在百度网盘同步盘内:请在本机百度网盘客户端把 `07-仿真系统/.git` 加入忽略/不同步列表,防止多端并发写坏仓库;Windows 侧只作镜像备份,日常开发在 Ubuntu 内
- 提交信息格式:`[阶段n] 模块: 做了什么`,如 `[阶段1] racecar_sim: XACRO车模初版`
