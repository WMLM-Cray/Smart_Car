> 历史参考：本文件保留写作时的记录；当前参数和状态见 [项目文档入口](../README.md)。

# 两点演示入口就绪性检查（2026-09-21 收口轮，纯核对，不改码）

> 结论：两点**自动**链路（racecar_mission 任务器 + 定位健康发布端）代码与离线测试均已就绪，
> 但**不能立即跑**。它卡在三个现场前置上，且健康发布端 `verified=false` 不发 True。
> 未就绪时用「人工确认后顺序发送两个单点」回退，明确**不是自动多点**。

---

## 1. 启动依赖链（已核对）

两点任务器**只调度** Nav2，本身不启动 Nav2 / AMCL / 地图 / 安全层。完整依赖：

```
安全链(单点同款): safety_nav_one.sh
   └─ bringup_launch.py 起 Nav2(bt_navigator/controller_server/planner -> /navigate_to_pose)
   └─ cmd_vel_safety + encoder_gate(v2 默认) + Run_car(driver)
定位健康发布端: localization_health.py  ->  /localization_ok (Bool, 2Hz)
任务器:        mission_executor (executor.launch.py)  订 /localization_ok, 调 /navigate_to_pose
```

- 任务器动作名 `nav_action=/navigate_to_pose`（默认，Nav2 标准接口）。
- 任务器**不产生** `/cmd_vel`，速度仍走 Nav2 → cmd_vel_safety → driver，安全链不变。

## 2. 话题 / 接口（已核对）

| 项 | 值 | 状态 |
|---|---|---|
| 动作 | `/navigate_to_pose` (`nav2_msgs/NavigateToPose`) | Nav2 提供 |
| 定位健康 | `/localization_ok` (`std_msgs/Bool`) | 发布端已实现，未部署未标定 |
| 任务文件 | `goal_file` (YAML, `enabled` 门禁) | 模板存在，`enabled:false` |
| 服务 | `~/start` `~/pause` `~/resume` `~/cancel` `~/status` | 已实现 |

## 3. Watchdog（红线，不能省）

- 启动任务器**必须**显式 `localization_watchdog:=2.0`（0.0 = 禁用 = 绕过 STALE 检测）。
- 发布端内部断流超时与任务器 watchdog 是**两种超时**，分开。
- 必须实车验证「发布端停发 → 任务器判 STALE → 拒派发」，不能只写参数不验证。

## 4. 目标配置（保持禁用，真实坐标现场生成）

- `config/mission_realcar_first.yaml` 现为 `enabled:false`，模板坐标是占位**不是地图真值**。
- 现场流程：单点通过 → 读实测原点/朝向 → `gen_goals.py` 生成真实坐标 YAML → 显式 `--enable`。
- 在 `--enable` 之前，任务器 `~/start` 会因 `enabled:false` 拒绝启动（失效安全）。

## 5. 健康发布端三条红线（已核对，未违反）

- `verified=false` 未标定 → **永不发 True**，只发诊断（影子运行）。现场标定前不改 `verified=true`。
- 不恒 True、不把「AMCL 有消息/协方差小」当健康。
- 不把 `localization_watchdog` 设 0。

## 6. 取消 / 退出行为（已核对）

- `~/cancel`：取消整段——先取消在途目标并**等 Nav2 确认终态**，再收尾报告；取消被拒/丢响应 → 进 `fault` 锁定。
- `Ctrl+C/SIGTERM`：`on_shutdown` 自接管，先取消在途目标再收尾（**普通目标取消，不是硬件急停**）。
- 定位失效（False/STALE）：运行中取消在途目标 + 停止派发；恢复须显式 `~/resume` 重审。
- 车停保障始终由主线安全链（cmd_vel_safety）负责，任务器取消 ≠ 保证停车。

## 7. 未就绪时的回退：人工顺序两个单点（不是自动多点）

自动两点未就绪（三前置缺一）时，演示用**人工确认后顺序发送两个单点**：

1. 定位健康人工确认（AMCL 贴墙跟随、协方差有界）。
2. RViz 给第一个 `2D Nav Goal`，等车到位停稳。
3. 人工再确认位置，给第二个 `2D Nav Goal`。
4. 两次单点之间人工把关，**不宣称是自动多点循迹**。

---

**三前置（决定自动两点何时可跑，均待现场）**：① 单点地面通过 → ② 健康发布端影子运行 +
标定阈值后置 `verified=true` → ③ 真实坐标 `gen_goals` 生成 `enabled:true` 任务文件。