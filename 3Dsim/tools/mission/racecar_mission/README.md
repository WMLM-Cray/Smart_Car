# racecar_mission —— 多点导航任务器

按顺序调度 Nav2 标准动作接口 `NavigateToPose`（`/navigate_to_pose`），记录每个目标的
结果并输出完整任务报告。**只做目标调度**：不发布 `/cmd_vel`、不复制 Nav2 控制器、
不改厂家驱动 / AMCL / 地图 / 安全层。

- 单点 / 连续多点两种模式
- 每点超时 / 成功 / 失败 / 取消
- 任务暂停 / 继续 / 取消
- 失败后按配置停止或跳过（`on_failure: stop | skip`）
- 循环模式（默认关闭，必须显式 `loop: true`）
- 完整 JSON 报告（发送时间、完成状态、耗时、最终位姿误差、恢复次数、失败原因）
- 检查点恢复（绝不自动恢复，见下文「检查点」）

## 目录

```
racecar_mission/
├── racecar_mission/          # Python 包
│   ├── goal.py               #   Goal / MissionConfig 数据结构（纯 Python）
│   ├── validate.py           #   YAML 读取与校验（纯 Python）
│   ├── engine.py             #   MissionEngine 状态机（纯 Python）
│   ├── report.py             #   报告 / 检查点序列化（纯 Python）
│   ├── executor_node.py      #   mission_executor 节点（ROS2）
│   ├── mock_nav2.py          #   mock_nav2 假动作服务器（离线回归用）
│   ├── offline_regression.py #   离线回归驱动（21 场景，含服务端事件断言）
│   ├── signal_exit_test.py   #   SIGINT/SIGTERM 退出时服务端确实结束在途目标（独立进程）
│   ├── use_sim_time_check.py #   launch 后 use_sim_time 参数生效验证（独立进程）
│   └── gen_goals.py          #   CSV/航点 → 任务 YAML 生成器
├── launch/                   # executor.launch.py / mock_regression.launch.py
├── config/                   # 任务 YAML 样例
└── test/                     # 纯 Python 单元测试（无 ROS 依赖）
```

纯 Python 核心（goal/validate/engine/report）**不依赖 rclpy**，可在无 ROS 环境跑
单元测试：

```sh
python -m unittest discover -s hwcart_sim/src/racecar_mission/test -v
```

## 任务 YAML

```yaml
mission:
  enabled: true            # 可选，默认 true；false 时任务器拒绝启动
  frame_id: map            # 必填，默认目标坐标系（须在 allowed_frames 内）
  robot_base_frame: base_footprint   # 量测最终位姿误差用的 frame
  mode: multi              # single=单点 / multi=连续多点（默认 multi）
  loop: false              # 循环，默认 false，必须显式开启
  on_failure: stop         # 失败策略，默认 stop
  default:                 # 可选，逐点缺省
    timeout: 30.0          # 单点超时秒；0 或省略=不设超时
    xy_tolerance: 0.1      # 仅验收记录，不覆盖 Nav2
    yaw_tolerance: 0.1
  goals:
    - name: g1             # 必填、非空、唯一
      seq: 1               # 必填、正整数、唯一
      x: 1.0               # 必填、有限数
      y: 0.0
      yaw: 0.0
      # 以下均可逐点覆盖：frame_id / timeout / on_failure / xy_tolerance / yaw_tolerance
```

**校验（立即失败，一次性报告全部问题）**：非有限坐标（NaN/Inf）、重复名称、重复或
非法序号、非法 frame（不在 `allowed_frames` 白名单内）、缺字段（name/seq/x/y/yaw/frame）。

**到达容差只作验收记录**：`xy_tolerance`/`yaw_tolerance` 仅用于在报告里判断
`within_tolerance`（最终位姿误差是否落在容差内），**绝不写进 Nav2 目标**，不改 Nav2
自身 goal_checker 参数。

## 接口

节点名 `mission_executor`，服务均 `std_srvs/srv/Trigger`：

| 服务 | 作用 |
|---|---|
| `~/start` | 开始任务：`resume_from` 已设则从检查点恢复，否则从 `goal_file` 全新开始；定位不健康拒绝启动 |
| `~/pause` | 暂停派发（在途目标跑完后停住，**不写终态报告**） |
| `~/resume` | 继续派发（恢复前重审定位健康，不健康则拒绝） |
| `~/cancel` | 取消整段任务（请求取消在途目标、等待其终态后再收尾；是普通目标取消，**不是硬件急停**） |
| `~/status` | 返回 JSON 状态（state/phase/index/total/laps/current_goal/localization/fault_reason） |

话题：`~/report`（`std_msgs/String`，任务结束/故障时发布 JSON 摘要）；
订阅 `localization_status_topic`（`std_msgs/Bool`，默认 `/localization_ok`）。

参数：`goal_file`、`checkpoint_file`（检查点**输出**）、`resume_from`（检查点**输入**，
显式恢复）、`report_file`、`nav_action`（默认 `/navigate_to_pose`）、`robot_base_frame`
（默认 `base_footprint`）、`allowed_frames`（默认 `[map, odom, odom_combined]`）、
`localization_status_topic`、`localization_watchdog`（秒，0=禁用）、`nav_wait_timeout`、
`cancel_timeout`（取消确认超时，默认 2.0s）、`shutdown_wait_timeout`（退出有界等待，
默认 2.0s）、`pose_error_max_age_s`（误差采样新鲜度上限，默认 1.0s）、
`tf_lookup_timeout`（TF 查询超时，默认 0.2s）、`use_sim_time`。

## 行为语义

- **单点模式**：只派发 `seq` 最小的 1 个目标，其余记 `not_attempted`（原因 single mode）。
- **失败策略**：`stop`=失败后停止，剩余记 `not_attempted`；`skip`=跳过继续下一个。
- **目标生命周期**：每个目标经过 `idle → waiting_accept → executing → 终态`；
  取消时进入 `waiting_cancel`，只有服务端确认该目标已终态，才允许结算当前目标并
  派发下一个。**「取消请求已发送」≠「取消已完成」**：取消必须等目标终态确认。
- **超时**：目标接受后 `timeout` 秒未终态，任务器请求取消该目标，并**等取消确认后**
  记 `timeout`（未确认不提前派发下一点）。
- **取消（整段任务）**：请求取消在途目标，等待其终态后收尾；在途目标记 `canceled`，
  剩余记 `not_attempted`；报告明确写「goal cancel, not a hardware e-stop」。
  若服务端**拒绝取消**或取消**超时未确认**，任务器进入 `fault` 锁定——不把「未确认
  取消」伪装成「已安全停止」，也不派发下一点。此时在途目标记 `unconfirmed`（结局
  未知，服务端可能仍在执行），**绝不记 `failed`/`canceled`**（那会谎称「旧目标已结束/
  车辆已停」）。
- **unconfirmed 目标状态**：目标已发出、动作服务消失或取消被拒/超时未确认，其结局
  未知。报告里 `unconfirmed` 计入 `dispatched` 但**不计入** `terminal`（确定终态），
  与「调度已锁定」（`state=fault`）明确区分——前者说「旧目标结局未确认」，后者说
  「任务器不再派发」。
- **暂停**：在途目标正常完成，之后停住（**不写终态报告**）；`resume` 从下一目标继续。
- **退出**：节点关闭时取消在途目标并收尾（`shutdown_wait_timeout` 内有界等待），
  同样不描述为硬件急停。
- **循环**：`loop: true` 时到队尾回卷继续（多圈）；默认关闭。

### 状态机

任务态 `idle/running/paused/completed/aborted/canceled/fault`，目标相位
`idle/waiting_accept/executing/waiting_cancel/fault`。`fault` 是终态：任务器自身无法
安全推进（动作服务消失、取消被拒、取消超时未确认）时锁定，并在报告中给出
`fault_reason`。所有服务/动作/定位/看门狗回调用同一把 `RLock` 串行化——rclpy 动作
客户端默认使用独立回调组，不与节点服务互斥，必须显式加锁避免「恢复」与「目标终态
结算」竞态。

### 定位健康门禁

订阅 `localization_status_topic`（Bool）并维护健康状态
`UNKNOWN/HEALTHY/UNHEALTHY/STALE`。收到 `False` **立即**判定不健康；开启
`localization_watchdog>0` 时超时无信号判 `STALE`。

- `~/start`：定位不健康（或未知）**拒绝启动**——绝不伪造健康。
- 运行中失效：取消在途目标（记 `failed`，原因 `localization lost`），**停止继续派发**，
  剩余记 `not_attempted`。
- `~/resume`：恢复前**重审**定位健康，不健康则拒绝恢复。

### 检查点（不自动恢复，严格校验）

任务运行中若设了 `checkpoint_file`（检查点**输出**路径），每完成一个目标即**原子**
写入检查点（临时文件 + `os.replace`），含已完成记录与下一目标下标。**节点重启后绝不
自动恢复**：只有显式设 `resume_from`（检查点**输入**路径，可与 `checkpoint_file` 分离）
**并显式调用 `~/start`（= 再次确认）** 才会恢复。检查点复用 `mission_from_dict` 做
严格校验（非有限坐标、非法 frame、类型错误的 `enabled`、越界 `next_index`、
非终态却含 `not_attempted` 记录等一律拒收）；已终态检查点拒绝恢复。

## 报告（JSON）

```json
{
  "mission": { "state": "completed",
               "counts": {"success": 3, "failed": 0, "timeout": 0, "canceled": 0,
                           "unconfirmed": 0, "not_attempted": 0},
               "dispatched": 3, "accepted": 3, "terminal": 3, "not_attempted": 0,
               "total_goals": 3, "started_at": ..., "finished_at": ... },
  "goals": [
    { "seq": 1, "name": "p1", "status": "success", "accepted": true,
      "send_time": ..., "finish_time": ..., "elapsed_s": ...,
      "position_error_m": 0.03, "yaw_error_rad": 0.01, "within_tolerance": true,
      "recoveries": 1, "nav_status": 4, "nav_error_code": 0, "reason": null }
  ]
}
```

计数语义：`dispatched`=实际发出的目标数（含未接受/超时/取消/`unconfirmed`，不含
`not_attempted`）；`accepted`=被服务端接受的目标数；`terminal`=进入确定终态
（成功/失败/取消/超时，**不含 `unconfirmed`**）的目标数；`not_attempted`=从未派发的
目标数。`position_error` / `yaw_error` 优先用 TF
（`goal.frame_id → robot_base_frame`），TF 不可用时回退到 Nav2 反馈 `current_pose`
（须同 frame）；都不可用则为 `null`。误差采样带**新鲜度上限**（`pose_error_max_age_s`）：
反馈样本超过该时限则视为陈旧丢弃；TF 查询有 `tf_lookup_timeout` 超时。

## 离线回归（无需 Gazebo / 真 Nav2 / 地图）

```sh
ros2 run racecar_mission run_offline_regression
```

用 `mock_nav2` 假动作服务器驱动任务器，覆盖 21 个场景并输出 PASS/FAIL + JSON 日志，
并**在服务端断言**（`mock.events`）确认「取消请求确实到达服务端、目标确实被服务端置为
终态」，而不只看任务器本地报告状态：
1. 单个直道目标；2. 单个含转弯目标；3. 三点短序列；4. 十一点整圈（如实报告，不挑成功轮次）；
5. 取消任务；6. 不可达目标；7. Nav2 未启动/动作服务消失；8. 中途定位失效停止派发；
9. 等待接受期间取消（迟到的 accept 到达后被主动取消）；10. 单点超时（取消确认后记账）；
11. 取消被服务端拒绝→`fault` 锁定（fault 后 start/resume 均拒、服务端未收后续目标）；
12. 定位 UNKNOWN 拒绝启动；13. 暂停不写终态报告、恢复后完成；
14. timeout+skip（旧目标终态确认后才派发下一点）；15. 取消接受但终态迟迟不返回→
cancel 超时 `fault`；16. 动作服务执行中消失→per-goal timeout→`fault`；17. 外部 CANCELED
后整段停止；18. 定位 False 拒绝启动；19. 定位证据过期（stale）拒绝启动；20. 暂停后定位
失效 resume 拒绝、恢复健康后继续完成；21. 取消请求无响应（服务端不回 ACCEPT/REJECT）→
cancel 超时 `fault`。

> 离线回归只验证「目标调度 + 动作生命周期 + 定位健康门禁」逻辑；它**不能**用
> Pure Pursuit / 二维真值路线 / 仿真测试驾驶器冒充 Nav2 多点任务成功。Nav2 多点任务的
> 成功须由主线在 Gazebo + 真 Nav2 上验证。

### 验证记录（WSL / ROS2 Humble，已跑通）

在 WSL（Ubuntu-22.04 / ROS2 Humble）构建并实跑，结果 **21/21 通过**（纯 Python 单元
测试 **49/49 通过**，`python -m unittest discover -s .../test -v`）；另独立进程验证
**SIGINT/SIGTERM 退出时服务端确实结束在途目标**、**launch 后 use_sim_time 参数生效**：

```sh
source /opt/ros/humble/setup.bash
cd ~/Astrahuaweicar/hwcart_sim
colcon build --symlink-install --packages-select racecar_mission
source install/setup.bash
ros2 run racecar_mission run_offline_regression
```

| # | 场景 | 期望 state / counts | 结果 |
|---|---|---|---|
| 1 | 单个直道 | completed / success:1 | PASS |
| 2 | 单个含转弯 | completed / success:1 | PASS |
| 3 | 三点短序列 | completed / success:3 | PASS |
| 4 | 十一点整圈 | completed / success:11 | PASS |
| 5 | 取消任务 | canceled / canceled:1 not_attempted:2 | PASS |
| 6 | 不可达目标 | aborted / failed:1 not_attempted:2 | PASS |
| 7 | Nav2 未启动/动作服务消失 | aborted / failed:1 | PASS |
| 8 | 中途定位失效 | aborted / success:1 failed:1 not_attempted:1 | PASS |
| 9 | 等待接受期间取消 | canceled / canceled:1 not_attempted:2 | PASS |
| 10 | 单点超时 | aborted / timeout:1 not_attempted:2 | PASS |
| 11 | 取消被拒→故障锁定 | fault / unconfirmed:1 not_attempted:2 | PASS |
| 12 | 定位 UNKNOWN 拒绝启动 | start 拒绝 / 无报告 | PASS |
| 13 | 暂停不写终态报告、恢复 | completed / success:3 | PASS |
| 14 | timeout+skip（终态确认后才发下一点） | completed / timeout:1 success:2 | PASS |
| 15 | 取消接受但终态迟迟不返回→fault | fault / unconfirmed:1 not_attempted:2 | PASS |
| 16 | 动作服务执行中消失→fault | fault / unconfirmed:1 | PASS |
| 17 | 外部 CANCELED 后整段停止 | canceled / canceled:1 not_attempted:2 | PASS |
| 18 | 定位 False 拒绝启动 | start 拒绝 / 无报告 | PASS |
| 19 | 定位证据过期（stale）拒绝启动 | start 拒绝 / 无报告 | PASS |
| 20 | 暂停后定位失效 resume 拒、恢复后完成 | completed / success:3 | PASS |
| 21 | 取消请求无响应→cancel 超时 fault | fault / unconfirmed:1 not_attempted:2 | PASS |

**退出与参数生效的独立进程验证**（不依赖 Gazebo，用 mock 子进程充当服务端）：

```sh
python3 -m racecar_mission.signal_exit_test --signal SIGINT   # 及 SIGTERM
python3 -m racecar_mission.use_sim_time_check                 # launch use_sim_time:=true 后 param get
```

- `signal_exit_test`：SIGINT / SIGTERM 下任务器干净退出（exit 0），且 mock 服务端事件
  落盘为 `goal_received → accepted → cancel_received → canceled`，报告在途目标记
  `canceled`（取消已确认），证明「退出时确实结束在途目标」而非仅本地记账。
- `use_sim_time_check`：`ros2 launch ... use_sim_time:=true` 后 `ros2 param get
  /mission_executor use_sim_time` 读回 `True`，证明 launch 声明 → 参数表 → 节点 clock
  选择这条链路参数确实生效。

rclpy Humble 集成要点（代码均已按此实现，非「变绿」改动）：

- `NavigateToPose` 结果体是 `std_msgs/Empty`，成功/失败/取消由 `GoalStatus.status`
  表达，**没有** `error_code`/`error_msg` 字段。
- 动作终端态（`succeed/abort/canceled`）须在 EXECUTING 之后置，且不接收结果参数。
- **取消确认语义**：Humble 中服务端 `cancel_callback` 返回 `REJECT` 时，`CancelGoal`
  响应 `return_code` 仍为 `ERROR_NONE`（0），且目标从 `goals_canceling` 移除；检测「取消
  被拒」须比对 `goals_canceling` 列表与 `handle.goal_id.uuid`，而非只看 `return_code`。
- **回调并发**：rclpy `ActionClient` 默认使用独立回调组，不与节点服务互斥，动作回调可
  与 `~/resume` 等并发；任务器用 `threading.RLock` 串行化所有叶回调，消除竞态。
- `rclpy.spin_once` 首参是 node 而非 executor；离线回归直接驱动 `executor.spin_once`。
- `use_sim_time` 由 rclpy 节点自动声明，不能重复 `declare_parameter`。
- 离线回归的「动作服务消失」场景用独有动作名 `/navigate_to_pose_missing`，避免前几例
  销毁的 mock 在 ROS 图中残留 `/navigate_to_pose` 造成 `wait_for_server` 误判。

手动驱动（模拟 mock + 任务器）：

```sh
ros2 launch racecar_mission mock_regression.launch.py \
  goal_file:=<绝对路径>/config/mission_3point.yaml report_file:=/tmp/r.json
ros2 service call /mission_executor/start std_srvs/srv/Trigger "{}"
ros2 service call /mission_executor/status std_srvs/srv/Trigger "{}"
```

## 仿真回归（待主线 Nav2 + 地图就绪）

在 Gazebo A 赛道 + 已保存地图上验证与离线回归相同的 8 个场景。启动任务器（不启动
Nav2/AMCL/地图，这些由主线提供）：

```sh
ros2 launch racecar_mission executor.launch.py \
  goal_file:=<绝对路径>/config/mission_3point.yaml \
  report_file:=/tmp/mission_report.json use_sim_time:=true
ros2 service call /mission_executor/start std_srvs/srv/Trigger "{}"
```

> ⚠️ 当前仓库尚未接入 Nav2 / SLAM / 地图（属主线未完成项），`sensor_test.world` 是
> 传感器测试夹具、非官方 A 赛道，无已保存地图。故仿真回归的「已保存地图」部分待主线
> 交付地图与 Nav2 集成后再执行；`config/mission_11point_lap.yaml` 坐标为占位矩形环，
> 须按官方 A 赛道重写。0.05m 通行间隙判据与 Nav2 参数不改动、不做「变绿」。

## 实车准备

- `config/mission_realcar_first.yaml`：实车首测目标集模板，`enabled: false` **默认禁用**；
  首个目标为车头前方短距离；坐标不写死地图值，主线定位完成后用
  `gen_goals` 依据实测原点/朝向生成正式文件。
- 实车多点模式必须在主线实车单点导航通过后才允许启用（改为 `enabled: true`）。
- 不直接运行实车；不与手柄控制同时启动；不 push 主分支。

```sh
ros2 run racecar_mission gen_goals --origin 0 0 0 -w 1.0,0.0,0.0 -w 2.0,0.0,0.0 \
  --mode multi --out mission_realcar.yaml   # 默认 enabled:false
```

## 已知限制

- 纯 Python 单元测试 48 项已跑通；`mission_executor` / `mock_nav2` /
  `offline_regression` / `signal_exit_test` / `use_sim_time_check` 已在 WSL（ROS2 Humble）
  构建并实跑，离线回归 21/21 通过（见「离线回归 → 验证记录」）。
- 位置误差依赖 TF 或 Nav2 反馈；两者皆无时报告为 `null`。
- 超时/取消依赖 Nav2 正确响应 cancel；若 Nav2 不响应取消，任务器在 `cancel_timeout`
  后进入 `fault` 锁定（不再派发下一点），而不是无界挂起。
- 循环模式无限循环，只能靠 `cancel` 退出。
- 暂停不影响已在途的目标（Nav2 动作不支持中途暂停）；暂停只暂停「后续派发」。
- 离线回归用 mock 假动作服务器验证调度/生命周期逻辑，**未**在 Gazebo + 真 Nav2 上
  验证多点任务成功（属主线未完成项）；mock 通过 ≠ 实车/Gazebo 导航通过。
