# 复审：手机手柄桥接 phone_bridge_source_0083df7.zip（任务四）

- 复审人：主线集成（AI）
- 交付包：`phone_bridge_source_0083df7.zip`（158147B，基线 0083df7，队友已回执 102b86e）
- 基线：racecar_phone_teleop 包（网关复用主线 racecar_teleop 状态机/映射，不重写运动安全）
- 结论口径：**离线 + 模拟注入级复核，未连接车机、未部署、未跑动力**；真实手机/实体手柄/蓝牙掉线实机一律记「未验证」，不作为真车放行依据。
- 附：58 单测本机对账（见 §4），与包内 `phone_bridge_unittest_verbose.txt` 声明的 58 项逐条吻合。

---

## 1. 架构一句话

```
手机浏览器 WSS ─▶ server(PhoneGamepadBridge)
  ─▶ protocol.parse(严格schema) ─▶ session.GatewayState(传输层:租约/challenge/seq/新鲜度)
  ─▶ adapter.TeleopMotionAdapter(复用主线 TeleopStateMachine + mapping)
  ─▶ sink.RosCmdVelSink ─▶ /phone/cmd_vel（隔离话题）
  ──(主线集成 launch 显式 remap，本包不做)──▶ /cmd_vel ─▶ cmd_vel_safety ─▶ /car_cmd_vel
```

本包只做到 `/phone/cmd_vel`。**不发布** `/car_cmd_vel`、不用厂家 `/teleop_cmd_vel`、不打开串口。remap 与安全层/驱动由主线集成层负责（M2 文档 §2.3 已如实声明）。

---

## 2. 可接入（复核通过项）

### 2.1 复用主线状态机，不另写一套（最关键加分项）
- `adapter.py` 直接 `import racecar_teleop.state.TeleopStateMachine` + `mapping.Parameters`，**运动安全全部复用主线**（含 `e2113b0` 断流恢复补丁 + `03db574` 油门回中门），不重写状态机、不另写常数。
- 依赖主线的公开新增接口 `TeleopStateMachine.lock()`（等价原私有 `_lock`，行为保留）。`RTO_state.py`（包内随附主线 state.py 副本）逐字核对：`lock()` 为 additive 变更，`on_joy` 含 e2113b0 的「刷新时间戳前先查超时」补丁，`on_tick` 含 03db574 油门门。与主线现役一致。

### 2.2 真实锁定而非注入零油门
- 断连 / 输入超时 / `stop=true` / `device_valid=false` / `page_visible=false` 五类触发**一律调用 `sm.lock()` 整链锁定**，恢复须「松开使能 + 油门回死区一帧」，不注入零油门冒充停车。
- 证据：`test_adapter.py::TestRealLock`（7 项：stop/stop 恢复/device_invalid/page_hidden/timeout/reconnect 满杆/reverse 门）+ `test_server.py`（断连锁定 / 蓝牙掉线超时锁定）。
- `server.py::_ws_handler` 的 `finally` 块：断开即 `release(session_id)` + `motion.lock()` + `_publish`，异常/心跳超时路径同样覆盖。

### 2.3 断连/恢复/第二客户端/无输入心跳 四项竞态（用户点名项）
| 场景 | 处理 | 验证 |
|---|---|---|
| 断连（WSS 关闭） | `finally` 释放租约 + `sm.lock()` | `test_disconnect_locks_output` |
| 恢复包先于定时器到达 | `FreshnessGate.stale_before_update` 在 `mark_input` **之前**检查超时并失效租约，旧 session/challenge 在下一步被拒 | `test_stale_packet_before_tick_cannot_refresh` |
| 重连满杆未松开/未回中 | 油门门 + 未 arm，不得起步 | `test_reconnect_full_stick_still_locked`（判据 7） |
| 第二客户端争抢 | 握手 `acquire` 抛 `LeaseHeld` → `reject(lease_held)` + close 1008 | `test_second_client_rejected` |
| 无输入（蓝牙/WiFi 停发） | 传输层 `tick` 失效租约 + 运动层 `on_tick` 超时，双重锁定 | `test_bluetooth_drop_timeout_locks` |
| 乱序/迟到/旧包 | `seq <= _last_seq` 拒绝；旧 challenge 拒绝 | `test_duplicate/out_of_order/stale` 系列 |

- **双重心跳语义区分诚实**：`WebSocketResponse(heartbeat=30.0, autoping=True)` 只证连接存活；有效输入新鲜度由 0.30s input_timeout 独立判断。challenge+seq 只证**传输层**新鲜度，**不证物理输入新鲜度**——文档已明示这是 M1 真机门槛（见 §5 未验证）。

### 2.4 协议严格校验
- `protocol.py`：version=1、seq/sample 为 int（排除 bool）、enable/stop/device_valid/page_visible 为 strict bool、throttle/steering [-1,1] 且 finite、未知字段拒绝、消息 ≤4096B。
- 证据：`test_protocol.py` 17 项，覆盖 NaN/Inf/越界/bool-vs-int/未知字段/缺字段/超大消息。

### 2.5 参数运行时加载 + 镜像防漂移
- `adapter.phone_parameters()` 运行时读随包 `config/realcar_teleop.yaml`（`importlib.resources` 优先，`__file__` 回退），不硬编码、不依赖源码树 `tools/` 路径。
- 随包参数与主线 `tools/teleop/realcar_teleop.yaml` 逐项一致：`max_speed_forward/reverse=0.10`（= cmd_vel_safety 上限，与本次任务一「0.55→0.10 回退」对齐）、`wheelbase=0.36`、`max_steering_angle=0.35`、`deadzone=0.08`、`input_timeout=0.3`。
- 三向测试 `test_phone_parameters_match_mainline_yaml` / `test_parameters_loaded_from_bundled_config_not_hardcoded` / `test_bundled_config_file_matches_mainline`：主线改参即回归失败。
- IP/端口/cert/key/Origin 全部参数化，**不写死** `192.168.5.100`（车机现网 STA 地址，非已确认热点网关）。

### 2.6 测试覆盖与 58 单测对账
- 主线 49 单测（含 `lock()` 回归）本机已独立复验过（任务三时重建包跑过，21/21 回归 + signal 用例 exit 0）。
- **手机桥 58 单测本机对账**（`TS_*.py` 逐方法计数）：
  - `test_protocol.py` = 17（解析 3 + 拒绝 13 + 大小 1）
  - `test_session.py` = 13（租约 3 + handle 4 + 新鲜度 3 + challenge 过期 1 + 断连 2）
  - `test_adapter.py` = 18（映射 7 + 真实锁定 7 + 公开 lock 接口 1 + 参数一致 3）
  - `test_server.py` = 10（欢迎解锁/第二客户端/断连锁定/重连旧包拒/seq 去重/origin/未知字段/锁屏门/蓝牙掉线/重连满杆）
  - **合计 58**，与包内 `phone_bridge_unittest_verbose.txt` 声明一致。
- WSL ROS 侧 P1-P5 / FC1-FC4 / W1-W7 / SE1-SE4 由包内 `tools/ros_verify_*.py` 支撑，证据在 `docs/validation/2026-09-17-phone-bridge/`；本轮未在 WSL 重跑（当前仅本地离线收尾，不连接车机），**记未复核**。

---

## 3. 集成待办 / 接入前置（非模块缺陷）

### 3.1 主线 remap 待办：`/phone/cmd_vel` 尚未接入安全链
- 本包只产出隔离话题 `/phone/cmd_vel`——**独立输出本就是隔离要求，是特性不是缺陷**（避免与 Nav2/键盘争夺 `/cmd_vel`）。主线 `cmd_vel_safety` 订阅 `/cmd_vel`，现役 `tools/realcar/` 下**尚无**把 `/phone/cmd_vel` remap 进 `/cmd_vel` 的集成 launch/remap → 属**主线集成待办**，不归咎于桥接模块。
- **接线纪律（关键）**：remap 必须走 `/phone/cmd_vel → /cmd_vel → cmd_vel_safety → /car_cmd_vel`，**严禁** `/phone/cmd_vel → /car_cmd_vel` 直连（直连绕过安全层）。
- **指令源互斥（接入前提）**：手机桥与 Nav2 不能同时占用 `/cmd_vel`。主线既有"唯一速度源"纪律（Nav2 速度输出不得与手柄/键盘争抢 cmd_vel），接入须确保同一时刻只有一个运动指令源在发 `/cmd_vel`。
- **桥进程退出停止链（接入前提，须实测验证）**：桥接节点崩溃 → `/phone/cmd_vel` 停发 → 经 remap 后 cmd_vel_safety 的 0.5s 超时发零速。**但这条停止链依赖安全节点、driver 与串口仍正常，不是"任何故障下都可靠停车"**——若 safety 或 driver 也死，仍只剩物理断电源（与主线现役已知缺口一致）。接入时须实测"杀桥接节点 → safety 超时零速 → driver 回中位"整链，不能仅凭 0.5s 看门狗存在就宣称可靠停车。

### 3.2 依赖主线 racecar_teleop 包（colcon 环境）
- 桥接 `import racecar_teleop.state/mapping`，要求 colcon 工作区已安装含 `lock()` 的主线 `racecar_teleop` 包。zip 随附的 `RTO_*.py` 只是基线副本（供核对），桥接运行时不读它。
- 交付提交链 `f69dcdb 依赖引入（主线基线 racecar_teleop + tools）` 已声明该依赖，接入时需确认车机工作区含同版本主线包。

### 3.3 state.py 漂移风险（低，非缺陷，仅提示）
- 参数一致性有测试防漂移（§2.5），但**主线 `state.py` 的行为一致性无自动测试**（桥接依赖 `lock()` + e2113b0 + 03db574 语义）。主线日后改 state.py 时，须保持 `lock()` 为 additive、不破坏「松开+回中恢复」语义，否则桥接层锁定语义静默漂移。

---

## 4. 未验证（如实，不写通过）

1. **M1 真实手机 + 实体手柄兼容性**：单列验收，**放行门槛**，未执行。`DOC_PROBE_CHECKLIST.md` 已给出完整清单（环境/API、手柄轴/键映射、关手柄/断蓝牙/离范围/切后台/锁屏的断连可辨识性）。核心判据：故障后浏览器是否给**可信断连信号**（`connected=false` 或 `getGamepads()` 消失）；若「蓝牙已断但持续回旧按住+油门」→ 判失败，不得靠网络心跳证明安全。
2. **蓝牙掉线真实行为**：「拔线/断蓝牙后浏览器是否重发旧值」未实测。传输层新鲜度 + 运动层超时双重兜底是**基线**，实测结论另行记录；若真机重发旧值，须另补物理输入新鲜度信号。
3. **热点 AP IP/子网/端口**：未核实；`192.168.5.100` 是 STA 地址，未写死（加分），但热点网关地址待真机确认。
4. **串口帧层**：真实 driver→/dev/car 由主线 6bb97d8/T12 字节级证据覆盖；手机链仅在 `ros_verify_serial_chain.py` 用测试侧 fake driver 按同公式编帧到 pty（禁真实 /dev/car），真实 driver 不在本包范围。
5. **真实设备/实车动力级**：未执行；以上结论全部为离线 + 模拟注入级。

---

## 5. M1 接入清单（供下轮真机阶段使用）

参考包内 `DOC_PROBE_CHECKLIST.md`，主线侧接入前需先过 M1：

- [ ] 手柄型号/连接方式（蓝牙 vs USB OTG vs 2.4G）如实记录；8BitDo 有线/2.4G 无蓝牙不得称蓝牙连接手机。
- [ ] 手机型号/系统/浏览器版本；证书部署后 `isSecureContext=true`、`navigator.getGamepads` 可用。
- [ ] 轴/键映射：哪根物理杆→轴几、正方向、扳机松开值、死区。
- [ ] **断连可辨识**（逐项记录时刻→页面变化）：关手柄 / 断蓝牙 / 离范围 / 切后台 / 锁屏。
- [ ] 判据：故障后出现可信断连信号（`connected=false` 或手柄消失）为过；持续回旧值判失败、停该组合动力接入。
- [ ] 主线 remap 接线：`/phone/cmd_vel → /cmd_vel → cmd_vel_safety`（**不得直连 /car_cmd_vel**），接入后 dryrun 验零速帧。

---

## 6. 总体结论

- **架构评级：可接入（有条件）**。设计克制、复用主线状态机而非重写、锁定语义真实、竞态处理到位、口径诚实（M1/蓝牙/热点/串口/实车五项未验证如实列出）。
- **接入前置**：① 主线补 `/phone/cmd_vel → /cmd_vel` 集成 remap（经 cmd_vel_safety，禁直连）；② 确认车机工作区含 `lock()` 主线 racecar_teleop；③ M1 真机兼容性先过（放行门槛），尤其是「拔线/断蓝牙是否重发旧值」。
- 与任务三 mission 模块并列：两个队友交付的**模块内部均自洽**，剩下的都是**主线集成待办**——mission 需主线定义并落地定位健康接口（非"补一个 Bool 节点"即完成）、phone bridge 需主线做 `/phone/cmd_vel` 集成 remap（隔离输出本就是要求，属接线待办非缺陷）。主线集成层（定位健康接口 + 指令源互斥 + 桥进程退出停止链）是下一轮真正要补的环节。
