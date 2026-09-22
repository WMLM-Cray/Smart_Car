# 实车接入第一步:连接·盘点·备份·传感器采集(2026-09-16)

任务书:仿真以 3b7fbd5 冻结,工作重心转实车。第一步交付目标=**电脑可查看实车传感器、
厂家配置已有备份、后续控制接入路径明确**。本文档汇总厂家说明(全部来自本地官方教程与
出厂代码,来源逐条标注),给出用户操作步骤与三个入库脚本的用法。

**第一步绝对边界**:不发运动指令、不跑 teleop、不做 PWM 扫描、不部署 web_teleop、
不刷机、不覆盖厂家工作区、不动 Gazebo/导航参数。传感器采集在**电调开关断开**下进行。

---

## §1 厂家说明摘要(任务1交付,来源逐条标注)

### 1.1 连接方式
| 项 | 厂家说法 | 来源 |
|---|---|---|
| 车载网络 | 车载华为路由器广播 WiFi,SSID=车身贴纸名称,**密码与名称一致** | 指导手册·实验一;路由器配置.pdf |
| 车载电脑 IP | 静态绑定 **192.168.5.100**(路由器后台 192.168.5.1) | 路由器重置/配置.pdf |
| 主机名(两批镜像不同) | `orangepiaipro-20t`(出厂代码对应批) / `davinci-mini`(手册旧版) | 路由器重置.pdf / 路由器配置.pdf;save.sh 注释还出现 `/home/orange-pi` |
| 登录用户名 | 手册截图显示 `racecar`、save.sh 用 `$HOME` 自适应——**以实车 `whoami` 为准** | 指导手册·实验二;出厂 save.sh |
| 登录工具 | Windows 装 MobaXterm(家庭便携版),连车 WiFi 后 SSH 到 192.168.5.100 | 指导手册·实验一 |
| 密码 | 车身贴纸/卖家提供,**用户自查,本文档与脚本一律不经手** | — |

### 1.2 供电
- **动力**:LIPO 电池 → 电调(ESC)。插上电池、打开**电调开关**,电机才有动力
  (指导手册·实验五)。**忘关电调开关长时间放置=电池损坏**(同页原话)。
- **充电**:M7 平衡充(允许输入 7-28V;LiPo 1-6S)。安全要求:电流与电池匹配、
  有人值守、不用时拔输入(M7 手册·安全注意事项)。具体串数/电流以电池标签为准。
- **车载电脑/传感器**:经拓展坞 USB 由车载电源供电(传感器介绍.pdf:四设备全插华为板
  拓展坞)。

### 1.3 关机
- 厂家教程只写了**停机流程**:Ctrl+C 终止程序 → 关终端 → **断开电调开关**
  (指导手册·实验五);未明示 Linux 系统关机命令。
- 本项目做法:先停程序,再 `sudo poweroff` 优雅关系统,屏幕黑后再断总电
  (标准 Linux 流程,标注为【本项目补充,非厂家原文】)。

### 1.4 电机使能链(任务5核心,出厂代码逐行核实)
```
物理层:LIPO 电池 + 电调开关 —— 动力总开关,断开则电机无动力
串口层:/dev/car(udev 绑定 ttyUSB1,绿板双 type-c 的左口)—— 唯一底盘通道
节点层:racecar_driver_node(随 car.sh 无条件启动!订阅 /car_cmd_vel)
         油门 PWM = linear.x×100 + 1500
         舵机 PWM = 1500 − angular.z×1300     ← racecar_driver_node.cpp:45-46
指令源:racecar_teleop.py → /teleop_cmd_vel(出厂代码 teleop.py:65,
         注释里 /cmd_vel 与 /car_cmd_vel 两版均被弃用)
        go*.py(多点导航执行器) → /car_cmd_vel 直发
        nav2(Run_nav.launch.py) → /cmd_vel(与 /car_cmd_vel 的桥接点在出厂包内
         未见,疑似缺口——实车 `ros2 node info` 盘点时确认)
```
**关键结论**:
1. **car.sh 不是纯传感器脚本**——`Run_car.launch.py` 同时启动雷达/IMU/编码器/EKF
   **和 racecar_driver_node**(底盘串口写手)。
2. 官方安全实践=手册实验八:**关闭电调开关**再跑定位/取点。我们全部传感器采集照此办理。
3. 舵机可能由 car 板 USB 供电,断电调只断**电机**动力;因此第一步还叠一层"不发任何指令"。

### 1.5 停止方式(按优先级)
1. **物理**:断开电调开关(最权威,任何失控立即执行);
2. 软件:Ctrl+C 杀 teleop/导航/car.sh(各 .sh 都带 SIGINT trap 清理);
3. 完全静止:断电调 + 停 car.sh。

### 1.6 传感器启动与话题(指导手册·实验三/四;串口配置.pdf)
- 启动:`cd ~/racecar && bash car.sh`(会弹 RViz,fixed frame 切 base_link 看雷达红点云)。
- 话题:`/scan`(镭神,lslidar_x10_launch.py)、`/IMU_data`(超核 hipnuc)、
  `/encoder`、`/odom_combined`(EKF 融合,ekf.yaml)。
- 串口默认:imu→ttyUSB0、car→ttyUSB1、encoder→ttyUSB2、laser→ttyACM0;
  绑定规则 `/etc/udev/rules.d/racecar.rules`(KERNELS+SYMLINK car/imu/encoder/laser)。
- 静态 TF(出厂 Run_car.launch.py):base_footprint→base_link z=0.15;
  base_footprint→laser_link x=0.07;base_footprint→IMU_link x=0.1653
  (**与仿真 params.yaml 已对齐的三处同源**)。
- 建图/导航:gmapping.sh(**自带 Run_car.launch.py**+slam_gmapping,所以手册说建图
  "无需再执行 car.sh")、nav.sh=Run_car+Run_nav(nav2,参数 nav_new.yaml,地图 map1.yaml)、
  nav_one.sh 单点版。三者均含 racecar_driver——跑前先断电调。

---

## §2 用户操作步骤(连接实车,给用户的具体动作)

**开机顺序**(首次按此执行,每步判据在括号里):
1. 确认 LIPO 电池有电(电显/充电器确认),装入车上,接电调线;**电调开关保持断开**。
2. 给车载电脑/路由器上电(按车上的电源开关;等 1-2 分钟启动完成)。
3. Windows 右下角连车 WiFi:SSID=车身贴纸名,密码=同名(连上后"无 Internet"是正常的)。
4. 浏览器进 `192.168.5.1`(管理密码=WiFi 名)→"终端管理"看有线设备在线
   (主机名 orangepiaipro-20t 或 davinci-mini)→**确认车机 IP=192.168.5.100**。
5. 打开 MobaXterm → Session→SSH → Remote host=`192.168.5.100` →
   Specify username 填车身/卖家给的账号(不确定就先不填,用默认试)→ 密码问卖家/贴纸。
   (连不上的排查顺序:ping 192.168.5.100 → 路由器终端管理里看设备 → 换用户名重试)
6. 连上后第一件事在车机终端跑:`whoami && hostname && ls ~` ——
   把输出发回,即完成任务2的"连接信息确认"。

**然后按 §3 顺序跑三个脚本**(脚本在 `07-仿真系统/tools/realcar/`,
用 MobaXterm 左侧 SFTP 直接拖进车机家目录)。

## §3 三个入库脚本(任务2/3/4)

| 脚本 | 任务 | 作用 | 在车机上的用法 |
|---|---|---|---|
| `probe_readonly.sh` | 2 | 只读盘点系统/CPU/ROS2/工作区/服务/设备 | `bash probe_readonly.sh` |
| `backup_vendor.sh` | 3 | 备份厂家工作区+udev 规则+环境快照+恢复说明 | `bash backup_vendor.sh` |
| `capture_sensors.sh` | 4 | 传感器话题/类型/频率/TF/完整 LaserScan 字段 | 先另开终端 `bash car.sh`,再 `bash capture_sensors.sh`(会先确认电调已断) |

每个脚本输出到 `~/名字_时间戳/` 目录,跑完用 MobaXterm 拖回电脑,
归档到 `07-仿真系统/docs/validation/2026-09-16-realcar/`(下轮验收用)。

**备份恢复方法**(backup_vendor.sh 会随备份生成"恢复说明.txt"):
- 文件级:tar 包解回 `~/`;udev 规则解回 /etc 后 `udevadm control --reload`。
- 整机兜底:`05-出厂镜像`(现名 `6.ROS2镜像-出厂版本`)/test-0906.img.zip +
  Ascend-devkit-imager 本地制卡烧录(**会清空整盘**,烧前确认)。

## §4 独立实车工作区部署清单(任务6,第一步只列不装)

原则:**厂家工作区 ~/racecar 原样不动**(它是出厂验证过的基线),我方一切进独立
`~/racecar_team` 工作区,靠 overlay(source 顺序)复用厂家驱动。

| 类别 | 内容 | 说明 |
|---|---|---|
| 复用(不动) | lslidar_driver / lslidar_msgs、hipnuc_imu、encoder、racecar_driver、serial | 出厂 5 个驱动包,以 ~/racecar/install 里的成品运行,不重编不改参 |
| 复用(只读) | udev 规则(/dev/car 等 4 个别名)、厂家 .sh 启动脚本 | 第二步起我方 launch 再自建 |
| 新建工作区 | `~/racecar_team/src`(overlay 在 /opt/ros/humble + ~/racecar/install 之上) | 本仓库 hwcart_sim 中 ROS 无关的纯 python 工具(trackA CSV/检查器/sim 二维)直接可拷 |
| 需装依赖 | python3-numpy/matplotlib(车机大概率已有,probe 脚本会列出);其余按移植包再定 | 第一步无新依赖 |
| 移植候选(控制接入后再说) | cmd_vel→/car_cmd_vel 桥(sim_wiring 先例)、测试驾驶器、trackA 资产 | **第一步不部署**;导航/SLAM 全部后置 |
| 明确不做 | 自主导航、web_teleop 自启动、PWM 标定、参数改动 | 任务书红线 |

## §5 轴距测量(任务7,需用户动手,卷尺即可)

**要量的是:前轮转向主销/前轴中心线 到 后轮轴中心线的距离**,不是车壳长!
1. 车放平地,前轮回正(目视方向盘/转向杆与纵轴平行);
2. 卷尺量:**前轮轴心高度处、左右前轮轴线中点** 到 **后轮轴线**(左右后轮轮心连线)的
   水平距离,就是轴距 L;
3. 读数精确到 5mm 即可;顺便量轮径(轮毂+胎,过轮心最宽处)——仿真轮径对账也欠着。
4. 结果回填:0.305(出厂 car_controller_new.cpp:118 默认值)vs 0.36(主线/仿真现值)
   **哪个对,以卷尺为准**,錯的一方改配置并记录(SIM_INTEROP §1 同步)。

**✅ 2026-09-16 实测落档**:用户卷尺实测轴距与轮径**均与主线 params.yaml 一致**
(wheel_base=0.36 / 轮径 180mm),出厂源码 0.305 与本车不符,候选关闭。
params.yaml/SIM_INTEROP 注释已同步;四项遗留中的"轴距待实测"就此销账。

## §6 已知遗留(不阻塞第一步,汇总备查)
- 导航最小间隙 0.041<0.05(仿真侧,后续独立任务);
- LaserScan 角度端点约定(±π 含两端 vs 不含)——**本轮 capture_sensors.sh 正好补齐证据**;
- 轴距 0.305 vs 0.36(§5 卷尺裁决);
- 队友手柄 01a3374 超时缺陷待修复后再接入;
- web_teleop 未引入未审查。

---
*来源清单:02-官方教程/无人驾驶竞速车(HW-Autocar)指导手册(2025).docx(实验一/三/四/五/六/八/十一/十二/十三)、3-远程连接.pdf、15-网络配置/路由器重置.pdf 与 路由器的配置.pdf、14-串口配置.pdf、2-传感器介绍.pdf、13-传感器的应用/实验文档.docx、1-电池充电器/M7 手册、11-镜像烧录与备份/镜像备份与烧录.pdf;04-出厂代码/racecar_05_27.tar.gz(car.sh/nav.sh/nav_one.sh/gmapping.sh/save.sh、Run_car.launch.py、Run_nav.launch.py、racecar_driver_node.cpp、racecar_teleop.py、car_controller_new.cpp)。*
