> 历史参考：本文件保留写作时的记录；当前参数和状态见 [项目文档入口](../README.md)。

# 实车控制链审查与无动力联调(2026-09-16,"控制链审查与无动力联调"轮)

任务书四项:①完整审查控制链(以车上实际源码和启动配置为准);②决定是否需要桥接;
③无动力联调(驱动电机动力断开,隔离输出验证,不发未核对试探命令);④提交下一步上电测试方案。
附加:核查车机时钟与 PC 时间差。**本轮不运行 find_throttle_mid.py、不做 PWM 扫描、不做地面行驶。**

全部证据来自 2026-09-16 对车机的只读审查(SSH `ssh hwcar`),行号均指车上文件。

---

## §0 结论速览

| 问题 | 结论 |
|---|---|
| 油门公式(车上现役) | `motor_pwm = linear.x×100 + 1540`(出厂 tar 版是 +1500,**今天上午被改**) |
| 舵机公式 | `servo_pwm = 1500 − angular.z×1300` |
| 1500/1540/1550 各是什么 | **1500**=出厂公式偏置+舵机几何中位;**1540**=今天上午标定的电调中位(driver 公式偏置+install 版 teleop 中位);**1550**=src 版 teleop 中位(**未编译进 install,运行时不生效**)。三者非同层,详见 §3 |
| 驱动层超时保护 | **无**(无 watchdog、无限幅、无断流停车、无退出停车,断流时下位机保持最后指令;下位机行为在预编译库内不可见) |
| 上位机安全层(补齐) | **cmd_vel_safety 已部署并验证**:检查(NaN/inf→0)/限幅(±0.10m/s,±0.35)/超时零速(0.5s)/退出零,T11-T16 话题级全 PASS(§10);抓出并修复 rclpy SIGINT 先关 context 导致退出零必败的真 bug |
| 需要桥接吗 | **不需要新增任何桥**。厂家已给两套现成方案:node_one(驱动层订 /cmd_vel,nav_one.sh 用)与 go*.py(/cmd_vel→/car_cmd_vel 中继)。我方采纳 node_one 链,/cmd_vel=物理速度唯一入口,速度→PWM 转换只保留 driver 一处 |
| 自启动项 | **无**(无 systemd/crontab/bashrc 启动;bashrc 仅 source setup.bash) |
| find_throttle_mid.py | 车主(非厂家包)今天 12:18 创建的油门中位试探脚本,发 /teleop_cmd_vel 直通,1500→1600 每 1s +1 扫描。**本轮未运行**(§4) |
| 车机时钟 | 慢 PC **165 天 5 小时 40 分**(2026-04-04 15:57 vs 2026-09-16 21:37,同时区 +0800,无 NTP) |
| 无动力联调 | 已执行(虚拟串口隔离,§8),零指令/正反向/回绕/断流/退出/重启全矩阵,日志入库 |

---

## §1 控制链五层全景

```
[指令源层]                    [话题层]            [驱动层]                [串口层]       [执行器]
nav2 controller_server  ──▶  /cmd_vel ────────▶  racecar_driver_node_one  ─┐
  (Run_nav.launch.py,        (Twist, 物理语义:    (nav_one.sh 链)          │
   RPP 0.25m/s)               v=m/s, ω=rad/s)     motor=v×100+1540         │
                                                                     send_cmd()    /dev/car     电调(牵引电机)
racecar_teleop.py      ──▶  /teleop_cmd_vel ──▶  (node/node_one 同款       (38400 8N1?)  (ttyUSB2,     +
(手动 ros2 run;               (Twist, 直通语义:    teleop 回调,             帧格式在      udev 别名     舵机
 install 版中位 1540)          x=PWM µs,           motor=linear.x 直通,     预编译静态    /dev/car)
                              z=度刻度 0-180)      servo=2500−z×2000/180)   库内,§2.4)
find_throttle_mid.py   ──▶  /teleop_cmd_vel(同上,手动,本轮未运行)
go.py/go_new.py/       ──▶  /car_cmd_vel ─────▶  racecar_driver_node       ┘
go_light.py(手动中继)         (Twist, 物理语义)    (car.sh/nav.sh 链,
                              ← 它们转发 /cmd_vel    motor=v×100+1540)
car_control_all / my_car_control_new /
scan_node / laser_2d_new / car_controller_new
(手动 ros2 run,无 launch 引用,§5 警告:前两者+后者之一语义错位)
stop_publisher ──▶ /car/cmd_vel(打错的话题名,无订阅者,废节点)
```

**驱动层只有两个可执行,逐字相同,唯一区别是订阅话题**(`diff` 确认仅 32-33 行注释互换):
- `racecar_driver_node`:订 `/car_cmd_vel` + `/teleop_cmd_vel`——car.sh / nav.sh / gmapping.sh 链
- `racecar_driver_node_one`:订 `/cmd_vel` + `/teleop_cmd_vel`——nav_one.sh 链(官方单点导航)
- 两者节点名都叫 `racecar_driver`(`racecar_driver_node.cpp:14`),**不可同时运行**(话题双消费+同名节点)

## §2 驱动层细目(任务书逐项)

### 2.1 话题/类型/字段语义与单位
| 话题 | 消息类型 | linear.x 语义 | angular.z 语义 | 回调 |
|---|---|---|---|---|
| `/car_cmd_vel`(node)或 `/cmd_vel`(node_one) | geometry_msgs/Twist | **车速 m/s**(经公式映射) | **角速度刻度**(经 1500−z×1300 映射;z=0 → 舵机中位) | TwistCallback(cpp:41-53) |
| `/teleop_cmd_vel` | geometry_msgs/Twist | **电机 PWM 微秒值,直通不转换** | **度刻度 0-180**(90=舵机几何中位 1500) | teleop_TwistCallback(cpp:56-65) |

队列深度均为 1(depth=1,新指令覆盖旧指令)。

### 2.2 转换公式、限幅、默认值(车上 src `racecar_driver/src/racecar_driver_node.cpp:45-46`)
```cpp
linear_x = twist->linear.x * 100 + 1540;        // 出厂 tar 版为 +1500(§3)
angle    = 1500.0 - twist->angular.z * 1300;
send_cmd(static_cast<uint16_t>(linear_x), static_cast<uint16_t>(angle));
```
- **公式通道换算表**(车上 1540 版):v=0→1540;0.25→1565(=nav2 desired_linear_vel 对应油门);1.0→1640;
  −0.25→1290。z=0→1500;±0.385→±500(即 z≈±0.385 打满舵机行程端点 1000/2000)。
- **直通通道**:`send_cmd(linear.x, 2500−z×2000/180)`——teleop 的 1550(中位)/1600(前进)/1360(倒车)
  就是这一层的 PWM 微秒值,与公式通道**互不相干**。
- **限幅:无**。C++ 侧对结果只做 `static_cast<uint16_t>`:
  - 结果 >65535 或 <0 时**整型回绕**(如 v=−20 → −460 → 回绕 65076,直接下发);
  - 结果落在 500–2500 之外但在 0–65535 内(如 v=+10 → 2540)也会原样下发。
  README 声明 PWM 合法范围 500–2500µs,但节点不做任何 clamp——**限幅责任完全在上游指令源**。
- **默认值**:serial_port=`/dev/car`、baud_rate=38400(declare_parameter,cpp:16-17,launch 未覆盖)。

### 2.3 启动 / 无消息 / 断流 / 退出时发送什么
| 场景 | 实际行为(源码证据) |
|---|---|
| 启动 | `art_racecar_init()` 打开串口后**不发送任何初始化报文**(main=init→spin,cpp:77-83;构造函数无 send)。下位机上电自保持中位(未证实,静态库不可见) |
| 无消息 | 什么都不发(spin 阻塞等订阅) |
| 断流(上游停止发布) | **不发停车**——无任何定时器/watchdog,下位机**保持最后一条指令**(§8 T5 实证) |
| 节点退出(Ctrl+C) | **不发停车**——无析构发送、无 atexit;`rclcpp::shutdown()` 后串口直接关闭(§8 T6 实证) |
| 独立命令超时保护 | **无**(全代码无 timer/create_wall_timer) |

**含义**:任何指令源崩溃/断网后,车会以最后指令继续跑,唯一兜底是物理断电调开关。
这是上电方案(REALCAR_POWERON_PLAN.md)把"断电调开关"列为首位停止手段的依据。

### 2.4 串口报文(审查盲区,如实标注)
底层实现在预编译静态库 `lib/libracecar_driver.a`(8382 字节,May 15)中,源码不可见。
可确认(头文件+strings):
- API:`art_racecar_init(speed, dev)` / `send_cmd(motor_pwm, servo_pwm)`(µs) / `read_port`(有回读逻辑) / `check_uint`(疑似校验);
- **每帧发送前把完整报文以 hex 打印到 stdout**(`print_send_buff`,`"the send buff is: %02x"`)——
  这是无动力联调的现成观测点,也是逆向帧格式的入口(§8 T1/T2 字节差分);
- 报文字节布局(帧头/长度/校验)不可见,**本轮不逆向、不向真实 /dev/car 发任何字节**。

## §3 1500 / 1540 / 1550 三数定案(任务书重点)

### 3.1 层位
| 值 | 所在层 | 文件与证据 | 用途 |
|---|---|---|---|
| **1500** | 驱动层公式的**出厂偏置**;舵机公式的**几何中位**;tar 版 teleop 中位 | tar 版 racecar_driver_node.cpp:45(`×100+1500`);车上 cpp:46 舵机 `1500−z×1300`;tar 版 racecar_teleop.py:73 | 出厂假设的"标准电调中位"(RC 惯例值)+舵机中位基准 |
| **1540** | 驱动层公式的**现役偏置**;install 版 teleop 的**现役中位** | 车上 src cpp:45(今天 12:23 改);install racecar_teleop.py(diff 实证 speed_mid=1540) | **今天上午实测标定出的电调中位**:1540µs 时电机静止(推断,依据标定动作链),油门零点从 1500 修正到 1540 |
| **1550** | src 版 teleop 的中位(**未编译,运行时不生效**) | src racecar_teleop.py:73(今天 12:34 改,md5≠install) | 标定会话最后一笔修改(1540→1550),**没有 colcon build,`ros2 run` 仍跑 1540 版** |

**结论**:1500/1540 是同一层(电调中位/油门零点)在两个时刻的值——出厂假设 vs 今天实测修正;
1550 是同一层又一次未生效的修改。**不能说"1500 和 1550 都是电调中位"**:1500 是出厂写的(已被 1540 取代),
1550 是没生效的草稿,**当前真正生效的电调中位 = 1540**(driver 公式偏置与 install teleop 中位一致,自洽)。

### 3.2 时间线(文件 mtime × 车机钟换算,关键:车机钟慢 165 天,文件时间"Apr 4 12:xx"=真实今天中午)
| 车机时间 | 真实时间 | 事件(证据=mtime) |
|---|---|---|
| Apr 8 / May 15-17 / May 27 | 4月-5月(真实) | car.sh、launch/config/scripts、libracecar_driver.a 出厂调试;racecar_05_27.tar.gz 打包(其中 driver=+1500、teleop mid=1500/23) |
| **Apr 4 12:18** | **今天 12:18** | `~/find_throttle_mid.py` 创建(油门中位试探脚本,§4) |
| Apr 4 12:22-12:28 | 今天 12:22-12:28 | `colcon build`:racecar 包 C++ 全部重编(car_control_all 等 5 个二进制 mtime),teleop.py 装入 install(**当时 src 版=1540/20**);racecar_driver 包重编(install 二进制 12:25/26,即 **+1540 版**) |
| Apr 4 12:23 | 今天 12:23 | driver src 改 `+1500→+1540`(tar 对照实锤差异) |
| Apr 4 12:34 | 今天 12:34 | teleop src 改 `1540/20→1550/50`,**此后未再编译** |
| Apr 4 15:42-15:46 | 今天 15:42-15:46 | 我方部署 probe/backup/capture 三脚本(b47af14 轮) |

⇒ 今天上午发生了一次完整的**电调中位标定会话**(试探脚本→改 driver→重编→再调 teleop→忘了重编),
操作者待用户确认(用户上午在车上部署过公钥;可能是卖家远程指导或用户照教程操作)。
**本审查报告不猜测操作者身份,只记录证据链。**

### 3.3 由此产生的运行时真相(重要)
| 文件 | tar(5/27) | install(运行时) | src(当前) |
|---|---|---|---|
| racecar_driver_node(.cpp/二进制) | +1500 | **+1540**(今天编译) | +1540(同步) |
| racecar_teleop.py speed_mid / start | 1500 / 23 | **1540 / 20** | **1550 / 50**(未生效) |
| go*.py / get.py | — | 与 src md5 一致(未动) | 同 |

**陷阱**:照手册 `ros2 run racecar racecar_teleop.py` 跑的是 1540/20 版;直接 `python3 src/.../racecar_teleop.py`
才是 1550/50 版。**联调/上电前必须先统一**(上电方案 §4 处置)。

## §4 find_throttle_mid.py:来源与行为(本轮未运行)

- **来源**:车主家目录散装文件(~/find_throttle_mid.py,2082 字节,mtime=今天 12:18),
  **不在厂家 racecar 工作区、不在出厂 tar 内**(tar 清单无此文件),作者不详(§3.2 时间线指向今天上午的标定会话)。
- **行为**(全文件 70 行,逐行审读):
  1. 节点 `find_throttle_mid`,发布 `/teleop_cmd_vel`(**直通通道**,linear.x=PWM µs 原样下发);
  2. 每 1.0s 发一条 `linear.x=当前值, angular.z=90`(z=90 经直通公式=舵机 1500 几何中位,即测试期间前轮回正);
  3. 当前值从 **1500** 起每秒 **+1** 递增至 **1600** 后停住(100 级阶梯,每级 1s);
  4. Ctrl+C → finally 发一条 `linear.x=1500`(它自己假设的中位)恢复。
- **风险定性**:若 driver 在跑且电调供电,1500→1600 扫描期间**车会真的动**(每步只 +1µs,极缓,但 1600≈teleop
  前进起步值=明显前进)。这就是任务书"本轮不运行"的原因。它假设中位 1500 而写脚本,结论却把 driver 改成了
  1540——说明**实测发现 1500 静止段太窄/中位偏移,1540 才是"刚不动"的位置**(推断,以 §8 联调字节证据为准)。
- 本轮处置:**只读分析,未运行,不部署,不入自启动**。

## §5 所有可能发布运动指令的节点(全清单+自启动核查)

### 5.1 清单(以 `grep create_publisher` × 话题 全覆盖)
| 节点/脚本 | 发布到 | 启动方式 | 语义 | 状态 |
|---|---|---|---|---|
| racecar_teleop.py | /teleop_cmd_vel | 手动 ros2 run(无 launch 引用) | 直通 PWM | 可用(install=1540 版) |
| find_throttle_mid.py | /teleop_cmd_vel | 手动 python3 | 直通 PWM 扫描 | **本轮禁用** |
| go.py / go_new.py | /car_cmd_vel(转发 /cmd_vel) | 手动 ros2 run | 物理语义中继 | 与 node 版 driver 配套的桥 |
| go_light.py | /car_cmd_vel + NavigateThroughPoses | 手动 | 中继+红绿灯联动(订 /light) | 同上 |
| car_control_all | car_cmd_vel(相对名) | 手动 ros2 run | **语义错位**:发 1525 等 PWM 直通值到公式通道(×100+1540→uint16 回绕 22968) | ⚠️ 禁跑 |
| my_car_control_new | car_cmd_vel(相对名) | 手动 | **同上语义错位**(1500+baseSpeed/度刻度 PID) | ⚠️ 禁跑 |
| laser_2d_new | car_cmd_vel + /point_mid | 手动 | 直通刻度语义错位 | ⚠️ 禁跑 |
| scan_node | car_cmd_vel(相对名) | 手动 | **物理语义正确**(0.25 m/s/rad),雷达避障 demo | 未启用 |
| car_controller_new | /car_cmd_vel | 手动 | 物理语义(纯追踪,L=0.305 出处=cpp:118) | 未启用(轴距与本车不符) |
| stop_publisher | **/car/cmd_vel** | 手动 | 话题名打错(多个斜杠),**无任何订阅者** | 废节点 |
| nav2 controller_server(Run_nav) | /cmd_vel | nav.sh/nav_one.sh | 物理语义 | 消费者=node_one(仅 nav_one 链) |
| RedAndGreen.py | /light,/camera/image_result | 手动 | 纯感知,无运动指令 | 无风险 |

⚠️ 语义错位三件套(car_control_all/my_car_control_new/laser_2d_new)是厂家 demo 遗留,与现役 driver
公式不匹配:1525 经 ×100+1540=154040→uint16 截断=**22968µs**(超合法范围数倍)。**任何时候都不要运行**,
将来教学复现也须先改话题或改语义。

### 5.2 自启动核查(结论:无)
- systemd 用户/系统服务:无 ROS/racecar 单元(b47af14 probe 实测);
- crontab:bashrc 仅 `source racecar/install/setup.bash`(第 123 行,环境非启动);
- 桌面 autostart/rc.local:probe 无相关项。
⇒ **车上电后不会有任何节点自动发运动指令**;一切指令源都是人工显式启动。

## §6 启动配置矩阵(哪个 .sh 起哪些东西)
| 脚本 | 起 launch | driver 版 | nav2 | 完整性 |
|---|---|---|---|---|
| car.sh | Run_car.launch.py | node(/car_cmd_vel) | 无 | 传感器+底盘,teleop 手动配 |
| gmapping.sh | Run_car + slam_gmapping | node | 无 | 建图(teleop 开车) |
| nav.sh | Run_car + Run_nav | node | ✅ 但 /cmd_vel **无人消费** | **废链**(除非手动加跑 go*.py 中继) |
| **nav_one.sh** | **Run_car_one + Run_nav** | **node_one(/cmd_vel)** | ✅ 直连 | **官方完整链**(教程第 6 课单点导航) |

注:第一轮实车采集(00_nodes.txt)中出现 slam_gmapping+rviz2,超出 car.sh/Run_car.launch.py 所能启动的
范围(install 与 src 的 launch md5 一致,均无 gmapping/rviz)——推断当时另有启动入口(gmapping.sh 或手动
rviz2),不影响控制链结论(三运动话题 publisher=0 已实测)。

## §7 桥接决策(任务二)

**结论:不新增任何桥,不做 remap。采纳厂家 nav_one.sh 链的驱动(node_one)。**

| 候选 | 评价 | 裁决 |
|---|---|---|
| 新增 cmd_vel→car_cmd_vel 桥(我方 sim_wiring/队友 cmd_vel_relay 先例) | 厂家已有两套现成方案,再加=**第二处速度→PWM 转换链**,违反"唯一速度指令出口" | ❌ 不做 |
| go*.py 中继 | 现成,但它本身是"多点导航执行器"(带 stop 策略/路点逻辑),拿来纯当桥=黑盒多余行为 | ❌ 不选(不删除不修改) |
| **node_one(驱动层直订 /cmd_vel)** | 现成二进制、今天刚重编、nav_one.sh 官方链、零转换重复 | ✅ **采纳** |

**唯一速度指令出口约定**(我方后续开发遵守):
1. `/cmd_vel` =物理语义(v m/s,ω 刻度)唯一入口,消费者永远是 node_one;**速度→PWM 只发生在 driver 这一层**;
2. 我方策略/测试节点一律发 /cmd_vel,**禁止**直接发 /car_cmd_vel 或 /teleop_cmd_vel(后者仅保留给
   标定/应急,且必须电调断开或有人扶车);
3. nav2 与我方策略不同时跑(都发 /cmd_vel,会打架);需要并存时将来加仲裁节点(届时单独立项)。

## §8 无动力联调(任务三,已执行)

**隔离方式**:Python pty 虚拟串口(`dryrun_driver_test.py` 造 /tmp/fakecar→/dev/pts/N),
driver 以参数 `serial_port:=/tmp/fakecar` 启动——**真实 /dev/car 全程未被打开**,
电调开关保持物理断开(双保险)。字节证据两路:pty master 端原始字节 + driver stdout 的
`the send buff is` hex 打印。

测试矩阵与结果(2026-09-16 执行完毕,全部 PASS;证据 docs/validation/2026-09-16-control/dryrun_20260404_162846/):
| # | 工况 | 输入 | 预期 | 实测字节(motor/servo 字段) | 结果 |
|---|---|---|---|---|---|
| T0 | 启动 3s 无指令 | — | 无字节(无初始化报文) | 0B | PASS |
| T1 | 零指令 | v=0,z=0 | motor=1540,servo=1500 | 0x0604/0x05DC ✓ | PASS |
| T2 | 正向 v=0.25 | v=0.25 | motor=1565 | 0x061D ✓ | PASS |
| T2b | 正向 v=1.0 | v=1.0 | motor=1640 | 0x0668 ✓ | PASS |
| T3 | 反向 v=−0.25 | v=−0.25 | motor=1515(脚本初版误写 1290,公式=1540−25,以实测为准) | 0x05EB ✓ | PASS |
| T4a/b | 满舵两端 | z=±5/13 | servo=1000/2000 | 0x03E8/0x07D0 ✓ | PASS |
| T5 | 越界 v=−20 | v=−20 | uint16 应回绕 65076 | **motor 字段=0x0000(库内 check_uint 把超界值归零!)** | PASS(发现) |
| T6 | 断流 20s | 停止发布 | 无字节(无 watchdog,下位机保持最后指令) | 0B | PASS |
| T7 | SIGINT 退出 | Ctrl+C 等价 | 退出不发送 | 无字节;**退出码 −9(SIGINT 8s 不退,SIGKILL 才死)** | PASS(带新发现) |
| T8 | teleop 直通零位 | x=1540,z=90 | motor=1540,servo=1500 | 0x0604/0x05DC ✓ | PASS |
| T9a-c | node_one(/cmd_vel)复测 | 0/0.25/直通 | 同 T1/T2/T8 语义 | 全部一致 ✓ | PASS |
| T10 | 下位机失联模拟(关 pty 主端) | 拔线后再发指令 | — | driver **存活、无报错、指令照发**(write 错误静默吞) | 记录 |

**帧格式逆向(联调副产物,全工况字节验证)**:
```
00 AA m_lo m_hi s_lo s_hi ck 55   (8 字节/帧)
ck = (m_lo+m_hi+s_lo+s_hi) & 0xFF  (四数据字节和的低 8 位;帧头尾不参与)
motor/servo 均为小端 uint16,单位 µs
每条 /car_cmd_vel(/cmd_vel、/teleop_cmd_vel)消息恰好 1 帧,无周期重发、无状态回传
```

**联调四项新结论(修正/补充 §2)**:
1. **静态库存在越界检查**:T5 理论回绕值 0xFE54 未出现,实际字段=0x0000——`check_uint`
   把超界值归零。库不是裸透传,但"归零"对电调仍是异常信号(0µs≠中位),**上游限幅依然必要**;
2. **无 watchdog 实锤**(T6):静默 20s 零字节,下位机保持最后指令;
3. **Ctrl+C 退出不可靠**(T7):SIGINT 后 8 秒不退(ros2 run 包装层/驱动均未响应),SIGKILL 才死。
   nav_one.sh 的 trap 用 SIGTERM——**SIGTERM 下行为未测**,列入上电阶段0 验证项(0.4);
4. **下位机失联不可感知**(T10):拔线后 driver 无报错不退出,指令继续"发送"(写失败被吞)。
   上游永远不知道车还听不听——物理断电是唯一兜底(与 §2.3 推断一致,现为实证)。

脚本:`tools/realcar/dryrun_driver_test.py/.sh`(入库,含 T10);两轮日志(含首轮环境踩坑
`Package not found`=wrapper 未 source 厂家 overlay,已修复)归档 docs/validation/2026-09-16-control/。

## §9 遗留与风险清单
1. ~~串口帧格式未逆向~~ → **已逆向**(§8:00 AA + 4 数据字节 + 和校验 + 55);残余:check_uint
   的具体策略(归零 vs 钳位)只观察到归零一种;
2. **下位机(STM32 板)是否有自己的断流保护**:不可见;按"没有"做最坏假设(§2.3);
   安全层(cmd_vel_safety,见 §10)在上位机侧补齐超时停车;
3. **teleop 三版本分叉**(§3.3)上电前必须统一(POWERON 方案 §4 处置);
4. **语义错位 demo 三件套**永久禁跑(§5.1);
5. 车机时钟慢 165 天:多机联测前的时间同步方案见 REALCAR_POWERON_PLAN.md §5(不在导航运行中跳时);
6. 1540 标定的**实证**(1540µs 恰好静止)只能上电后低速验证,本轮仅字节级验证公式正确性;
7. **SIGTERM 下 driver 是否退出未测**(T7 只测了 SIGINT=不退),列入上电阶段0 验证。

## §10 安全层(cmd_vel_safety,2026-09-16 新增,补齐任务书"输入检查/限幅/超时停车")

driver 层零防护(§2)+下位机失联不可感知(T10)→ 在 /cmd_vel 与 node_one 之间加一个
rclpy 安全节点(`tools/realcar/cmd_vel_safety.py`,部署在 ~/racecar_team,不碰厂家工作区):

```
nav2 controller ──/cmd_vel──▶ [cmd_vel_safety] ──/cmd_vel_safe──▶ node_one ──▶ /dev/car
                               · 输入检查:非有限值(NaN/inf)→ 按 0 处理
                               · 限幅:|v|≤max_linear(默认0.10 m/s)、|z|≤max_angular(默认0.35)
                               · 超时停车:cmd_timeout(默认0.5s)无新指令 → 周期发零(v=0,z=0)
                               · 启动默认不发(发零需显式 publish_start_zero:=true)
```
- 速度→PWM 转换仍只在 driver 一处(唯一出口原则不破);
- 上游零改动:nav2 不知道安全层存在;安全层自带 safety_nav_one.sh 启动脚本
  (串好 nav2→安全层→node 版 driver 链,不起厂家 waypoint_cycle 循环路点执行器);
- **验证已通过**(2026-09-16 话题级,`tools/realcar/dryrun_safety_check.py`,纯 /cmd_vel→/car_cmd_vel,
  不碰串口;证据 docs/validation/2026-09-16-control/safety_check_20260404_164821/):

| # | 工况 | 结果 |
|---|---|---|
| T11 | 正常值透传 v=0.05,z=0.10 | PASS(窗口含 0.05/0.10) |
| T12 | 正向限幅 0.50/0.90 → 0.10/0.35 | PASS |
| T15 | 负向限幅 −0.50 → −0.10 | PASS |
| T13 | NaN/inf → 按 0 发(节点日志双重证实告警) | PASS |
| T14 | 静默超时:0.5s 无指令 → 每 0.5s 周期零速 | PASS |
| T16 | **非零透传中 SIGINT → 退出零**(不让非零成为遗言) | PASS |

  验证过程另抓出并修复一个真 bug:**rclpy 默认 SIGINT handler 先 context.shutdown(),
  finally 里的退出零必败**(publisher's context is invalid)——首跑 T16 实证。修复=卸载 rclpy
  默认信号处理、自己接管 SIGINT/SIGTERM 置 flag、context 仍有效时发退出零(修复后 T16:
  窗口 `[(0.05,0),(0.05,0),(0,0)]`,日志"发零速: 节点退出")。首跑日志(164535)一并归档。
  验证脚本 v1 曾因"等首条即断言"被安全层周期零速重发打乱成全 FAIL——**那是脚本时序 bug,
  非节点行为 bug**,v2 改固定窗口收集+any 断言后判据正确。
