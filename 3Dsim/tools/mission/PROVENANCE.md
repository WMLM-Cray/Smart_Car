# racecar_mission 来源与引入说明(2026-09-19,演示分支 demo-20260920)

## 来源

- 交付对象:`06-队伍代码/racecar_mission_95d649c_rework.zip`(队友交付,多点导航任务器)。
- 实际 HEAD = **`102b86effb35160791260ba8d6df77195c523ddc`**(打包目录名 `331a200` 早于实际 HEAD
  一个提交,以 git SHA 为准)。
- 复审报告:`docs/validation/2026-09-18-nav-onepoint-round2/RACE_MISSION_REVIEW.md`
  (结论:可接入,源码质量高;纯 Python 核心 49/49 独立复跑;离线回归 21/21)。

## 本次选择性引入

- 引入范围:整包源码(`源码/racecar_mission/`,含 `test/`),**未改队友源码任何一行**
  (保留 SHA 与测试完整性)。
- 未引入:`测试日志/`、`测试代码/` 等打包证据(已在复审报告归档,不重复搬入仓库)。
- 启用的配置:**仅两点演示**(`config/mission_realcar_first.yaml`,当前 `enabled:false` 占位);
  多圈/11 点等配置保留但**不用于本轮演示**。

## 接入前提(复审已列,非代码缺陷)

1. 主线须提供 `/localization_ok`(std_msgs/Bool)发布者 → 见 `LOCALIZATION_HEALTH_INTERFACE.md`。
2. 实车须显式设 `localization_watchdog > 0`(0.0 = 禁用 STALE 检测)。
3. 实际发两点目标须等主线单点地面通过,用主线定位后的真实坐标生成(见该 yaml 头注)。

## 模块行为要点

- 只调度 Nav2 `NavigateToPose`,**不产生 `/cmd_vel`**(速度仍走主线安全链)。
- 默认无 `/localization_ok` 发布者时定位状态 `UNKNOWN` → **拒绝 start(失效安全)**。
- 与手柄控制不同时启动(README 明示)。
