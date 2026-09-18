# racecar_mission_95d649c_rework.zip 复审报告(2026-09-18,纯离线)

> 对象: `06-队伍代码/racecar_mission_95d649c_rework.zip`(队友交付,多点导航任务器)
> 方式: 源码审查 + 纯 Python 核心**独立复跑**;ROS2 部分依据打包证据(不连车机、不跑 Nav2)。
> 口径: [复验]=本机独立重跑;[回执]=队友打包证据;[源码]=代码静态审查。

---

## 1. 版本核对

| 项 | 值 | 证据 |
|---|---|---|
| package.xml / setup.py 版本 | `0.1.0` | [源码] 两处一致 |
| git 提交链(新→旧) | `102b86e`(回执,复现取消未确认+补全21场景)→ `331a200`(取消/fault证据+退出确认+use_sim_time)→ `95d649c`(use_sim_time文档)→ `05a11db`(检查点)→ `04d8bf0`(取消确认)→ `4055d3e`(8/8)→ `25b3c22`(初版) | [回执] `00_git_log.txt` |
| 实际 HEAD | `102b86e` | [回执] git log 顶行 |
| 目录名标注 | `racecar_mission_95d649c_rework_331a200` | [回执] 打包目录 |

**版本小记(非功能缺陷)**: 语义版本 `0.1.0` 未随多轮 rework 递增;打包目录名 `331a200`
早于实际 HEAD `102b86e` 一个提交(打包时未改目录名)。追踪性建议: 后续交付以 git SHA
为准,目录名/版本号同步。

---

## 2. 五项审查结论

### 2.1 取消故障 / 未确认 —— ✅ 通过

- 取消三态全处理: `_on_cancel_response` 对 `REJECTED(1)`/`UNKNOWN_GOAL_ID(2)` 进入 fault;
  并专门处理 **Humble 怪癖**(取消被拒时 `return_code` 仍为 0,目标从 `goals_canceling` 移除),
  比对 `goals_canceling` 列表与 `goal_id.uuid` 判定被拒 → fault(executor_node.py:498-510)。
- 取消超时(`_on_cancel_timeout`)→ fault "cancel unconfirmed"。
- 未确认目标记 `unconfirmed`(**不记 failed/canceled**),明确语义"旧目标结局未知、服务端可能仍在
  执行",不谎称"已安全停止"(engine.py:117-136 `fault()`)。报告计数把 `unconfirmed` 计入
  `dispatched` 但**不计入** `terminal`(report.py:23-25)。
- 证据: 离线回归场景 11/15/21(取消被拒/终态不返回/无响应→fault)全 PASS;`regression_summary.json`
  `passed=21 total=21`。

### 2.2 旧目标终止先于新目标 —— ✅ 通过(含一处低风险备注)

- 核心不变量成立: 只有收到 result 终态(`_on_result`)才 `_finalize_goal`,后者才派发下一目标;
  派发严格串行,正常流天然满足"旧目标终态 → 新目标"(executor_node.py:414-443,616-638)。
- 迟到回调防护: `_active_gen` 世代计数 + `_serialized`(RLock 串行化全部状态机回调),
  迟到的旧目标 accept 被主动 `_safe_cancel`(executor_node.py:365-375)。
- 场景 14(timeout+skip,终态确认后才发下一点)覆盖。
- **备注(低风险)**: `_safe_cancel`(迟到旧目标)是"尽力取消、不等待终态、不进入 fault"
  (executor_node.py:477-482)。因派发严格串行,该路径仅在异常(如 fault 后重启残留)才可能触发;
  建议接受,但实车接入后留意日志里 "stale goal accepted late" 的出现频率。

### 2.3 定位门控 —— ⚠️ 逻辑通过,实车集成缺一硬依赖

- 逻辑完备: `_loc_healthy()` = 状态 healthy 且(watchdog 开时)证据未过期;`~/start` 不健康拒绝
  ("绝不伪造健康")、`~/resume` 恢复前重审、每次派发前重审(`_send_goal`)、运行中收到 False 立即
  判不健康并 `_handle_loc_lost`(取消在途/整段取消)。
- **硬依赖(需返工)**: 门控订阅 `localization_status_topic`(默认 `/localization_ok`,std_msgs/Bool)。
  主线 `tools/realcar/` 目前**没有该话题的发布者**;无发布者时 `_loc_state` 恒 `unknown`,
  executor 将**拒绝 start**。README 也写明"真实发布者由主线诊断模块/适配节点提供"。→ 实车接入
  前主线须提供一个适配节点(AMCL/EKF 健康 → Bool),否则任务器起不动。
- **默认参数隐患(需返工)**: `localization_watchdog=0.0`(默认禁用)。禁用时若发布者最后一次发
  True 后死亡,状态永远 healthy,失去 STALE 检测。实车应设 `localization_watchdog>0`(如 1.0~2.0s)。

### 2.4 SIGINT/SIGTERM 生命周期 —— ✅ 通过

- `rclpy.init(signal_handler_options=SignalHandlerOptions.NO)` 禁用 rclpy 默认信号处理,自装
  handler 置 flag,主循环 spin_once,`finally` 里 `on_shutdown()` 取消在途目标并**有界等待终态**
  (`shutdown_wait_timeout=2s`),未确认则 `_enter_fault`;最后才 `destroy_node`+`shutdown`
  (executor_node.py:863-892)。与主线 cmd_vel_safety 的 rclpy SIGINT 坑(T16)处理一致。
- 证据: `signal_SIGINT.json`/`signal_SIGTERM.json` 均 `exit_code=0`,mock 服务端事件
  `goal_received→accepted→cancel_received→canceled`,证明"退出时确实结束在途目标"而非仅本地记账。

### 2.5 checkpoint 恢复 —— ✅ 通过

- 原子写(tmp+`os.replace`),每完成一目标 + 收尾各写一次。
- `parse_checkpoint` 严格校验: version=1、有限坐标、frame 白名单、enabled 严格 bool、
  `next_index` 范围且与 attempted 记录一致、非终态不得含 not_attempted、已终态拒绝恢复
  (report.py:69-129)。
- `restore()` "绝不自动调用": 仅 `resume_from` 参数 + 显式 `~/start` 服务才恢复(engine.py:53-63)。

---

## 3. 可接入 / 需返工 / 未验证 清单

### 可接入(源码质量 + 离线验证充分)

| 项 | 结论 | 证据 |
|---|---|---|
| 取消故障/未确认(fault/unconfirmed) | ✅ 可接入 | 源码 + 回归 11/15/21 |
| 旧目标终止先于新目标 | ✅ 可接入(注意 stale-cancel 备注) | 源码 + 回归 14 |
| SIGINT/SIGTERM 生命周期 | ✅ 可接入 | 源码 + signal_*.json exit 0 |
| checkpoint 恢复(不自动恢复+严格校验) | ✅ 可接入 | 源码 + 单测 |
| YAML/检查点严格校验 | ✅ 可接入 | **49/49 单测独立复跑 OK** |

### 需返工(集成前提,非代码缺陷)

| 项 | 问题 | 处置 |
|---|---|---|
| `/localization_ok` 发布者缺失 | 无发布者 → 任务器拒 start(硬依赖) | 主线补适配节点(AMCL/EKF 健康→Bool) |
| `localization_watchdog` 默认 0.0 | 发布者死亡后失去 STALE 检测 | 实车 launch 显式设 >0 |
| 版本号/目录名未随 rework 递增 | 追踪性差 | 后续以 git SHA 为准 |

### 未验证(如实,非缺陷)

| 项 | 说明 |
|---|---|
| Gazebo + 真 Nav2 仿真回归 | README 明说"待主线 Nav2+地图就绪";mock 通过 ≠ 实车/Gazebo 导航通过 |
| 实车首测 | `mission_realcar_first.yaml` 为 `enabled:false` 占位模板,坐标待主线定位后用 `gen_goals` 生成;须主线实车单点导航通过后才改 `enabled:true` |

---

## 4. 独立复验记录

| 项 | 结果 |
|---|---|
| 纯 Python 单测(test_engine/report/validate) | **49/49 PASS**(本机 python3 复跑,`unittest discover`) |
| 离线回归 21 场景 | `passed=21 total=21` [回执] `regression_summary.json`,每场景含服务端 `events` 断言 |
| SIGINT/SIGTERM 退出 | [回执] 均 exit 0,mock 服务端确认 cancel 到达 |
| use_sim_time 参数生效 | [回执] `use_sim_time.json` = True |

## 5. 总结论

**可接入**(源码质量高,纯 Python 核心可离线验证,ROS2 生命周期处理严谨,五项关注点全部实现并有
离线证据)。**接入前提两件返工**: ①主线提供 `/localization_ok` 发布者;②实车设
`localization_watchdog>0`。**接入口径**: 任务器只调度 Nav2 `NavigateToPose`,不产生 `/cmd_vel`
(速度仍走主线安全链),与主线控制链无冲突;与手柄控制**不同时启动**(README 明示)。实车首测须
在主线单点导航通过后、用主线生成的真实坐标文件进行。
