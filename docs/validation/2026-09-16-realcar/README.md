# 2026-09-16 实车接入第一步:连接·盘点·备份·传感器采集(全 PASS)

任务书交付目标:①电脑可查看实车传感器 ✅ ②厂家配置已有备份 ✅ ③后续控制接入路径明确 ✅(见
REALCAR_ONBOARDING.md §1.4/§4)。全程零运动指令:三个运动话题(/car_cmd_vel、/teleop_cmd_vel、
/motor_control)Publisher count=0 实测确认;采集期间电调开关物理断开(用户确认)。

## 执行方式

Claude 从 Windows 经 SSH 密钥直连车载电脑(免密,密钥对 `~/.ssh/hwcar`,用户手动部署公钥一次)。
`ssh hwcar` ≡ `ssh orange-pi@192.168.5.100`。用户在车旁负责物理安全。

## 1. 连接与环境盘点(probe_20260404_154244/,29 文件)

| 项 | 实车确认值 |
|---|---|
| 用户/主机/IP | `orange-pi@orangepiaipro-20t` / 192.168.5.100(三候选用户名中出厂代码批胜出) |
| 系统 | Ubuntu 22.04.3 LTS,aarch64,内核 5.10.0+(昇腾定制),4 核 |
| ROS2 | **Humble**(与 WSL 开发环境完全同构) |
| NPU | npu-smi 普通用户 errno=13 权限拒(非故障,未用 sudo) |
| 自启动服务 | **无任何 ROS/racecar 自启服务**;sshd/docker/lightdm 正常 |
| 串口绑定 | imu→ttyUSB0、**car→ttyUSB2、encoder→ttyUSB1**(car/encoder 与手册"通常"写法对调;udev 按 KERNELS 绑定,别名正确)、laser→ttyACM0;USB 芯片 CH340×2+CH343+CP210x 经 Genesys 拓展坞 |
| ⚠️ 车机时钟 | 慢约 5 个月(目录名 20260404;无外网无 NTP),文件时间戳解读须注意 |

## 2. 厂家工作区与备份(vendor_backup_20260404_154352/,MD5 双侧一致)

- `~/racecar`:11 个 src 包与出厂 tar 完全同名;**与 tar 包分叉**:多 `RedAndGreen/`、`.vscode/`,
  car.sh(4月8日)等时间戳早于 05_27 打包——**车上现状为准,已备份**。
- 车上 car.sh 实际起的是**加强版**(实测节点表含 slam_gmapping+rviz2;出厂 tar 版 Run_car.launch.py
  无 SLAM 无 RViz)——手册"会启动 rviz"的说法来自车上版。costmap/plan/particle_cloud 等话题是
  **RViz 导航面板插件注册的话题名,不是 nav2 在跑**。
- 备份件:racecar_src.tar.gz(46MB,排除 log)+ udev_rules.tar.gz + env_snapshot.txt(86KB)+
  恢复说明.txt;车机+本地 F: 双份。MD5:b75e482772dfe87c5cb446ce722c3790 / 0f708e62da845392fffcf63560b63336。
- 家目录另有:racecar_0403.zip(旧版)、racecar_05_27.tar.gz、find_throttle_mid.py(油门中值探测,来源待查)。

## 3. 传感器实测(sensors_20260404_154657/,电调断开,car.sh 起链后采集)

**/scan 完整字段(01b 未截断版,真实点数 1667)**:

| 字段 | 实车值 | Gazebo 仿真 | sim 二维 |
|---|---|---|---|
| frame_id | `laser_link` | laser_sensor_link(策略性不同名) | — |
| angle_min/max | **-π / +π(含两端)** | ±π(含两端) | -π..+π 不含右端 |
| angle_increment | **0.0037691574** | 2π/1666=0.0037712 | 2π/1667=0.0037690 |
| 点数(实测数组) | **1667** | 1667 | 1667 |
| time_increment / scan_time | **0.0 / 0.0** | 0.0 / 0.0 | — |
| range_min / range_max | **0.15 / 50.0** | 0.10?/25.0 | /25.0 |
| 频率 | **11.997 Hz**(149 帧窗口) | 11.905(30s 实测) | 12.0(参数) |

**IMU 99.94 Hz / encoder 49.76 Hz**(与仿真 100/50 目标吻合)。

### 对账结论(如实,不拔高)
1. **频率+点数**:实车 12Hz/1667 点与两边仿真目标完全对上——2d87bf3 对齐值被实车再次确认。
2. **角度端点**:实车三字段**非严格自洽**(increment≈2π/1667 更近,但 max-min=2π 且两端都填满;
   若 1667 点含两端应步进 2π/1666=0.0037712,不含右端应恰为 2π/1667)。**从消息字段无法唯一裁决
   首尾射线是否同向重复**,需驱动源码(lslidar 转 scan 段)或数据级验证。数值差异(万分之二)对
   48 扇区/SLAM 均无实际影响——两边维持各自约定+差异声明不变。
3. **time_increment/scan_time=0 是实车驱动同款行为**——回执三处修正之二(收窄为"当前插件/配置")
   被实车证实:逐点时间缺失是全链路(实车+Gazebo)共同约束。
4. **range_max=50.0 实锤**(驱动配置值;规则物理量程 25-30)。仿真 25 保持规则口径,差异记录不追改。
5. frame_id 实车=laser_link:将来实车跑 SLAM/导航时,仿真侧配置的 frame 名需适配(已知差异,不阻塞)。

## 本轮四个新坑(记录复用)
1. **ros2 topic echo 默认截断数组到 128 元素**(尾部 `- '...'`+`---`)——点数必须加
   `--truncate-length 3000`,否则数出来是 129。
2. **pkill -f 自杀**:ssh 远端 `bash -c` 命令行含模式字样,pkill -f 匹配到自身会话(exit 255)——
   模式用 `[R]un_car` 字符类打断自匹配。
3. **进程组收车不彻底**:ros2 launch 子进程脱离 car.sh 进程组,组 kill 后仍残留——须按名逐个
   pkill(带字符类)+ daemon stop。
4. capture_sensors.sh 首版 `TFPID/TFID` 变量名笔误(set -u 下 unbound 退出,第 5 节爆)——已修复
   入库,采集 1-4 节数据未受影响,5/6 节远程补跑。

## 复验入口(Windows,已配置 ssh hwcar)
```
ssh hwcar 'ls ~'                                    # 连通
ls docs/validation/2026-09-16-realcar/              # 三套产物+本 README
ssh hwcar 'md5sum ~/vendor_backup_2026*/racecar_src.tar.gz'   # 与本地 MD5SUMS 比对
```
传感器复采:`ssh hwcar` 后 `bash ~/racecar/car.sh`(先断电调)→另终端 `bash ~/capture_sensors.sh`。

## 下一步(控制接入路径,均未执行)
cmd_vel→/car_cmd_vel 桥(出厂包内无,队友 cmd_vel_relay 先例)→低速标定(PWM 1500/1550 两层关系
待实车确认,find_throttle_mid.py 或与此相关)→teleop 复核(只读盘点已明确链路)。
