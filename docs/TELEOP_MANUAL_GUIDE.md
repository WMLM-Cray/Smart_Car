# 实体手柄手动控制链·操作与验收文档(2026-09-17)

> 状态:代码/单测/全链虚拟验证通过(**离线+模拟注入级**);实体手柄、无动力、架空、
> 地面各级未执行。本文档按主线任务书(2026-09-17)交付,不作为真车放行依据。

## 1. 唯一指令链与通信拓扑

```
实体手柄(/dev/input/js*)
  │ SDL 事件
  ▼
joy_node            /joy 20Hz(autorepeat 显式 20.0,防"仅变化发布")
  │ sensor_msgs/Joy
  ▼
racecar_teleop      使能状态机+限幅+死区(物理速度,单位 m/s / rad/s)
  │ 发布 /teleop/cmd_vel ──remap──▶ /cmd_vel
  ▼
cmd_vel_safety      NaN/inf→0、限幅±0.10m/s·±0.35、0.5s 超时零速重发
  │ /car_cmd_vel
  ▼
racecar_driver_node(Run_car 内,速度→PWM 唯一转换点)
  │ motor=v×100+1540, servo=1500−ω×1300
  ▼
/dev/car 串口 → 帧 00 AA m_lo m_hi s_lo s_hi ck 55 → 电调/舵机
```

**两个易混话题(现场高危,务必分辨)**:
- `/teleop/cmd_vel`(**斜杠**,本链 teleop 输出,物理速度,进安全层)✅
- `/teleop_cmd_vel`(**下划线**,厂家直通通道,driver 另一订阅口,**不经安全层**,
  仅标定/应急用;本链绝不使用)⚠️

手动模式唯一指令源承诺:不起 Nav2、waypoint_cycle、键盘控制、厂家 teleop;
`/cmd_vel` 上只有 racecar_teleop 一个发布者。

## 2. 启动 / 停止 / 收车

**启动(一条命令,车机上)**:
```bash
bash ~/racecar_team/teleop_manual.sh          # 默认参数 realcar_teleop.yaml
```
前置:手柄插车机 USB;`teleop_ws` 已 colcon build;joy 包已装(`ros-humble-joy`);
无其他 driver 进程。脚本自查失败会拒启并给原因。

**停止优先级**(高→低):
1. 物理断电调开关(失控第一时间;注意:断电调后舵机仍可能动,舵机供电≠电调断电)
2. 松开手柄使能键(teleop 立即输出零→串口 1540/1500)
3. Ctrl+C 收链(trap 按登记 PID 进程树按序收链,**无全局按名清理**)

**收链顺序**(2026-09-17 契约审查轮修正,脚本 terminate() 实现):
```
1) 停指令源 teleop+joy  →  /cmd_vel 断流
2) 等 1.5s             →  安全层 0.5s 超时,周期零速帧经 driver 落到串口
3) 退 Run_car(driver)   →  停车消息已有人接收完毕后才退底盘链(含 setsid 组杀)
4) 安全层最后退          →  全程保证 /car_cmd_vel 有订阅者
5) 仅对登记树残余 SIGKILL(collect_tree 递归收集本链后代,不碰其他 ROS 任务)
```
证据分级:顺序与时序为**脚本逻辑+`bash -n`+离线审读**,实际串口停车帧
到达时序**须无动力实车复验**(今晚检查表 §8-T7)。

## 2a. 独立运行(不依赖 SSH 会话)

手柄链要"人离开电脑也能跑"。两种方式,按场景选:

**方式 A:前台 bash(默认,调试/首测用)** —— 即 §2 的 `bash teleop_manual.sh`。
SSH 断开与否直接影响(脚本挂在会话里),适合标定与首测阶段人在场。

**方式 B:systemd 手动服务(独立运行,推荐稳定后用)** ——
单元文件 `tools/teleop/systemd/racecar-teleop-manual.service`,**本轮只生成+离线
校验(`systemd-analyze verify` 通过),未部署、未 enable、不自启**。

| 操作 | 命令(车机上) |
|---|---|
| 一次性安装 | `sudo cp ~/racecar_team/systemd/racecar-teleop-manual.service /etc/systemd/system/ && sudo systemctl daemon-reload` |
| 启动 | `sudo systemctl start racecar-teleop-manual` |
| 停止(走脚本有序收链) | `sudo systemctl stop racecar-teleop-manual` |
| 状态 | `systemctl status racecar-teleop-manual` |
| 日志(链内四段进程输出) | `journalctl -u racecar-teleop-manual -f`;链分段日志在 `~/racecar_team/*_last.log` |
| 回滚/卸载 | `sudo systemctl stop racecar-teleop-manual; sudo rm /etc/systemd/system/racecar-teleop-manual.service; sudo systemctl daemon-reload`(链文件本体在 `~/racecar_team/`,删服务不动链) |

设计要点(与任务书四逐条对应):
- **SSH 断开不退链**:链进程全在服务 cgroup 内,与登录会话无关;
- **KillMode=mixed**:stop 先只对主进程(脚本)SIGTERM,触发 trap 按序收链;
  TimeoutStopSec=45 后 cgroup 残余才 SIGKILL——cgroup 即进程归属边界,不误杀他人;
- **Restart=no**:异常停机必须人工看日志后手动重启;重启也不恢复旧运动状态
  (teleop 上电锁定,判据1 保证零速起步);不用自动重启掩盖故障;
- 服务管理**不替代**下位机看门狗与物理断电调,失控仍第一时间物理断电;
- 明确 `User=orange-pi`、`WorkingDirectory=/home/orange-pi`,ROS 环境由脚本自身
  source,日志=journalctl+`~/racecar_team/*_last.log` 两级。

## 2b. 部署(Windows 侧一条命令)

`tools/teleop/deploy_teleop.sh`(Git Bash,车机开机后执行):比对 md5 幂等部署
teleop_manual.sh / realcar_teleop.yaml / cmd_vel_safety.py(不一致**不覆盖**只提示),
打包 `HEAD:hwcart_sim/src/racecar_teleop`(含 e2113b0 补丁)→ 车端解包+colcon
build+46 单测(任一失败 exit 3/4 中止,不打印"部署完成")→ 检查 joy 包与
`/dev/input/js*`。厂家 `~/racecar` 全程只读。

**首次接手柄必做标定**(不得按外壳印刷猜):
```bash
ros2 topic echo /joy        # 逐轴推动记录索引与正负向;逐键按下记录按钮号
```
回填 `~/racecar_team/realcar_teleop.yaml` 的 speed_axis/steering_axis/
enable_button/low_speed_button 与两个 inverted 开关。

## 3. 默认按键/参数表(通用双摇杆示例,实体待标定)

**用户手柄(2026-09-17 确认):Xbox 布局(XInput),车机 USB 直插。**
Linux xpad 驱动下 Xbox 布局的典型映射(axes: 左摇杆纵=1/右摇杆横=3;
buttons: LB=4/RB=5)与下表默认**恰好一致**,标定大概率只需逐项确认,
不同品牌仍可能有偏移——以 echo /joy 实测为准,不得跳过。

| 项 | 默认 | 说明 |
|---|---|---|
| speed_axis / steering_axis | 1 / 3 | 左摇杆纵轴 / 右摇杆横轴(待标定) |
| enable_button | 5 (RB) | **按住才允许运动**;启动/超时锁定后须先松开一次再按 |
| low_speed_button | 4 (LB) | 按住=低速模式(×0.4 → 0.04 m/s) |
| max_speed_forward/reverse | 0.10 / 0.10 m/s | **=安全层上限,推满即上限** |
| max_steering_angle | 0.35 rad | 等效自行车转向角(非单轮舵角限位 0.55) |
| wheelbase | 0.36 m | 卷尺实测(3fad10b) |
| deadzone | 0.08 | 包内死区;joy 侧另有 0.05,双死区串联总感受 ≈0.13 |
| input_timeout / publish_rate | 0.3 s / 20 Hz | 断流锁定;ω=v·tan(δ)/L |

## 4. 安全行为(六判据复验+全链 S1-S8 全过)

| 场景 | 行为 | 验证 |
|---|---|---|
| 上电即按住使能 | 不运动 | 判据1+S1 |
| 先松开再按住 | 允许运动 | 判据2+S2 |
| 断流>0.3s(含恢复消息先于tick到达) | 锁定零速,须松开重按 | 判据3a/3b+S5/S6(**e2113b0补丁**) |
| 非法输入(NaN/inf/截断) | 撤销许可并零速 | 判据5a |
| teleop 进程崩溃/被杀 | 安全层 0.5s 超时零速 | S8 |
| 超时时钟 | time.monotonic(),不受仿真暂停影响 | 判据6(代码审查) |

**已知残留风险(如实)**:
- **joy_node 拔线重发**:joy 的 autorepeat 是重发最后状态;若拔掉手柄后驱动仍以 20Hz
  重发旧 Joy,teleop 层无法区分(交接说明同款结论)。缓解=车端安全层独立兜底 +
  接入手柄时**实测拔线行为**(任务书四要求,未做);若确认重发,需加设备连接监控。
- 状态机判据3b 的"消息间隔检查"补丁覆盖了 tick 阻塞场景,但**不能**区分
  "joy 源头持续重发"(消息间隔正常)——同上一条,靠实测+连接监控收口。
- 蓝牙手柄的断连行为未验证(若有,同 USB 拔线实测要求)。

## 5. 分阶段验证状态(如实,未执行不得写 PASS)

| 阶段 | 状态 | 证据 |
|---|---|---|
| 离线:代码审查+46单测+六判据黑盒 | ✅ PASS(9/9) | docs/validation/2026-09-17-teleop/ |
| 全链虚拟(模拟/joy 注入,S1-S8) | ✅ PASS(8/8,WSL) | 同上 dryrun_chain_WSL.txt |
| 离线:契约静态复跑(清理修正后) | ✅ 21 PASS / exit 0,global_cleanup REVIEW 消除 | docs/validation/2026-09-17-contract/ 第三份报告 |
| 实体手柄+无动力(车机,真 joy_node) | ⬜ 未执行 | — |
| 架空短时(合电调,须用户确认后) | ⬜ 未执行 | — |
| 地面低速遥控 | ⬜ 未执行 | — |

串口帧层(→/dev/car)未在本轮重复实测:引用 6bb97d8 无动力联调对真实 driver 的
字节级验证结论(帧格式/公式);车上接入时可用
`dryrun_teleop_chain.py --serial /tmp/fakecar` 补帧级全链(电调断开前提)。

## 6. 今晚上电检查表(用户通知开机后执行;动力级另等确认)

前置门槛:全程**电调断开**(无动力);仅当用户明确确认现场条件后才进入架空/地面级。
每项做完即勾选并记时间;任一项异常→停止后续项,记录现象,物理断电。

| # | 检查项 | 操作 | 通过判据 |
|---|---|---|---|
| T1 | 部署 | Windows 侧 `bash tools/teleop/deploy_teleop.sh` | 末尾打印部署完成;车端 46 单测通过 |
| T2 | 接收器/手柄识别 | `lsusb`;`ls -l /dev/input/js*`;`ros2 run joy joy_node` 前台短跑 | js 设备出现;`cat /proc/bus/input/devices` 见手柄名 |
| T3 | /joy 标定 | 起 joy_node 后 `ros2 topic echo /joy`:逐轴推/逐键按 | 轴索引/方向/按键号记录在案,回填 realcar_teleop.yaml(与 §3 默认比对,不同即改) |
| T4 | 使能语义 | bash teleop_manual.sh 起链;上电即按住使能→车不动;松开再按→推杆有 /car_cmd_vel 输出 | `ros2 topic echo /car_cmd_vel` 见 0→非 0 变化;串口侧电调断开听不到,判据即话题层 |
| T5 | 低速/限位 | 按住 LB 推满(应 0.04);松 LB 推满(应 ≤0.10);满舵(≤0.35) | echo /car_cmd_vel 数值与限幅一致 |
| T6 | 停止三路 | ①松使能 ②按住使能直接 Ctrl+C 收链 ③杀 teleop 进程 | ①立即零;②③安全层 0.5s 内零且周期重发(观察 ≥2s) |
| T7 | 收链顺序实测 | 正常 Ctrl+C 收链,同时另终端 `ros2 topic echo /car_cmd_vel` | 零速帧持续到 driver 退出之后才断流(顺序证据) |
| T8 | **拔接收器重发检查(重点)** | 按住使能+推油门→**拔掉 USB 接收器**,观察 /joy 与 /car_cmd_vel | 若 /joy 仍 20Hz 重发旧值且车指令非零→确认"消息持续≠连接正常",记录并按 §4 残留风险处理(加连接监控前不得动力级);若断流→安全层 0.5s 零速,通过 |
| T9 | 关手柄电源 | 同 T8 但按手柄电源键(无线接收器留在车上) | 同 T8 判据 |
| T10 | 重连恢复 | 重插接收器/重开手柄 | /joy 恢复;teleop 处锁定态(须松开重按使能才动)——验证判据 3/4 实车表现 |
| T11 | SSH 断开独立运行(方式 B) | 安装 systemd 单元(§2a)→ start → 断开 SSH | 链存活(`systemctl status` active,echo /car_cmd_vel 仍可达);stop 走有序收链。**不 enable** |
| T12 | 帧级全链(可选) | `python3 dryrun_teleop_chain.py --serial /tmp/fakecar` | S1-S8 + 串口帧字节全过 |

T8/T9 是"消息持续到达不能证明手柄连接正常"的定奋试验:结论 whichever way 如实记录,
不因结果不好而复测到好为止。

**遗留说明**:导航链 `tools/realcar/safety_nav_one.sh` 存在同型全局 pkill 模式,
**本轮未改**(本轮范围=手动链;导航链收链修正留待 Nav2 轮按同方案 collect_tree 化)。

## 7. 本轮提交链

- `8384d8b` 手柄包引入(队友 racecar-teleop-standalone@ce0bdc9 原样,含 46 单测)
- `e2113b0` 判据3b 补丁(on_joy 消息间隔检查)+六判据复验器+证据
- `4db99cb`/`09fa2f9` 实车手动链脚本+全链虚拟验证器+本文档初版
- `44e18e1` 契约工具接入(tools/contract_audit/,来源 Codex 独立交付,复验 9/9+21 PASS)
- 本提交:手动链进程清理修正(零全局 pkill→登记进程树)+部署失败语义硬化+
  systemd 独立运行单元(仅生成离线校验)+sim/params.py 来源标注修正
  (1550/1540 两套配置边界;契约复跑 global_cleanup REVIEW 消除)
- 均在 integration/trackA-sdf 分支,**未 push**(按项目规则)
