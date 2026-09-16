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
3. Ctrl+C 收链(trap 按 PID+按名兜底清理)

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
| 实体手柄+无动力(车机,真 joy_node) | ⬜ 未执行 | — |
| 架空短时(合电调,须用户确认后) | ⬜ 未执行 | — |
| 地面低速遥控 | ⬜ 未执行 | — |

串口帧层(→/dev/car)未在本轮重复实测:引用 6bb97d8 无动力联调对真实 driver 的
字节级验证结论(帧格式/公式);车上接入时可用
`dryrun_teleop_chain.py --serial /tmp/fakecar` 补帧级全链(电调断开前提)。

## 6. 本轮提交链

- `8384d8b` 手柄包引入(队友 racecar-teleop-standalone@ce0bdc9 原样,含 46 单测)
- `e2113b0` 判据3b 补丁(on_joy 消息间隔检查)+六判据复验器+证据
- 本提交:实车手动链脚本(teleop_manual.sh/realcar_teleop.yaml)+全链虚拟验证器
  (dryrun_teleop_chain.py)+本文档
- 均在 integration/trackA-sdf 分支,**未 push**(按项目规则)
