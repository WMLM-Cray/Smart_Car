# /localization_ok 定位健康发布端（主线，2026-09-20）

> 两点任务器 racecar_mission 的定位健康门禁，之前只有**接口定义**（`tools/mission/LOCALIZATION_HEALTH_INTERFACE.md`）
> 与**消费端**（`executor_node.py`），生产发布者缺失。本目录补齐**生产发布端**。
>
> 状态：**发布端已实现（本机离线，未部署车端、未现场标定）**。任务器就绪；发布端待现场影子运行 + 标定阈值 +
> 接两点任务器验证。**不宣称实车定位健康已验证。**

## 1. 交付清单（4 文件，均在 tools/realcar/）

| 文件 | 作用 |
|---|---|
| `loc_health_core.py` | 纯 Python 状态机核心，无 ROS 依赖，可离线单测 |
| `localization_health.py` | ROS 节点（订阅/TF/单调时钟/发布），喂 core |
| `localization_health.yaml` | 阈值模板（外置，`verified: false` 未标定） |
| `test_loc_health_core.py` | 离线状态机单测（本机 19/19 通过） |
| `loc_health_integration_test.py` | ROS 隔离集成测试（需 ROS2 环境，WSL/车机执行） |

## 2. 架构：实现与标定分开

- **状态机**（`loc_health_core.py`）：只做「证据 → 判定」，输入接收时刻（单调时钟）与字段值，
  输出 `Verdict{healthy, state, reasons}`。**不自己取时钟、不 import rclpy**，故可离线单测。
- **ROS 节点**（`localization_health.py`）：订 `/amcl_pose` `/odom_combined` `/scan`，查 TF
  `map→base_footprint`、`map→odom_combined`，用 `time.monotonic()` 作单调时钟，2Hz 持续发布
  `/localization_ok`（Bool）+ `/localization_health_diag`（String JSON 诊断）。

**现场之后才做**：对照真实数据定阈值 → 影子运行（发布但不接任务器）→ 核对误报漏报 → 接两点任务器。

## 3. 判据（AND，任一不满足 → False + 具体原因）

| # | 判据 | 失败态 | 阈值 |
|---|---|---|---|
| 0 | 阈值已标定 | `unverified_thresholds` | `verified` |
| 1 | 最近帧数值有效（无 NaN/Inf/无效四元数） | `input_abnormal` | — |
| 2 | header.stamp 无回退 | `time_abnormal` | `stamp_regress_tol` |
| 3 | 证据齐全（amcl/scan/odom/TF 均到达） | `evidence_insufficient` / `tf_abnormal` | — |
| 4 | scan/odom/TF 断流未超时 | `time_abnormal` / `tf_abnormal` | `scan_timeout` `odom_timeout` `tf_timeout` |
| 5 | AMCL 更新 vs 累计运动（自上次更新以来） | `time_abnormal` | `motion_cum_linear` `motion_cum_angular` `amcl_stale_moving` |
| 6 | 协方差有界（位置米/角度弧度分开） | `threshold_exceeded` | `cov_pos_sigma` `cov_yaw_sigma` |
| 7 | AMCL↔odom 一致性（辅助） | `threshold_exceeded` | `consistency_dist` |

**判据 5 的 AMCL 静止语义（关键）**：AMCL 静止无新 scan / 未达 update_min_d/a 门限时**不持续发布新估计**，
故不更新 ≠ 失效。判据按「**自上次 AMCL 更新以来的累计运动 + 持续时间**」判定，**不用瞬时速度 + 固定超时**：
- 累计位移/转角（由 odom 位姿相对上次 AMCL 更新时刻的快照计算）**未超门限**（静止/微动）→ AMCL 不更新是设计行为，
  **不判失效，无论持续多久**（无 `amcl_stale_static` 固定超时）；
- 累计运动**已超门限**（`motion_cum_linear`/`motion_cum_angular`）→ AMCL 早该更新；若仍不更新且持续超
  `amcl_stale_moving`（默认 1s，即几个 scan 周期）→ 判"定位未跟上运动"。
- `motion_cum_linear/angular` 应取**现役 AMCL update_min_d/a 的 2~3 倍**；现役值待现场取证
  （本地 `amcl_realcar.yaml`=0.25/0.2618 与 `nav_safe.yaml`=0.025/0.06 两套并存，须确认车端跑哪套）。
- odom 断流 → 累计运动无法计算 → 不确定 → **不得判健康**（`time_abnormal`）。

**判据 7 的局限（用户点名）**：AMCL 与 odom 共享里程计，两者会一起漂；一致性只作辅助，不作定位正确独立证明。

## 4. 失效安全语义

- 启动、配置不完整、关键证据缺失 → **不发 True**（默认 False）。
- `verified: false`（未标定）→ **永不发 True**，只发 False + 诊断，供影子运行记录。
- 持续输出当前判定（2Hz），任务器 watchdog 监控本节点失联（**两种超时分开**：发布端内部断流超时 vs 任务器 `localization_watchdog`）。
- 发布 False 只触发任务器既有处置（取消在途目标/拒派发），**不宣称能保证停车**——停车由主线安全链负责。
- 不用恒 True 桩、不把"AMCL 有消息/协方差小"当健康、watchdog 不设 0。

## 5. 启动方式

```bash
# 影子运行（标定阶段）：verified=false，只发 False + 诊断，不接任务器。
# 用独立话题 /localization_ok_shadow + /localization_health_diag_shadow，避免 False 误取消其他会话任务
cd ~/racecar_team
python3 localization_health.py \
  --ros-args -p thresholds_file:=$PWD/localization_health.yaml \
  -p loc_topic:=/localization_ok_shadow -p diag_topic:=/localization_health_diag_shadow

# 标定完成后：verified=true，才可能发 True
python3 localization_health.py \
  --ros-args -p thresholds_file:=$PWD/localization_health.yaml -p verified:=true

# 逐项 ROS 参数覆盖（不读 YAML 时，默认 verified=false）
python3 localization_health.py --ros-args -p verified:=true -p cov_pos_sigma:=0.3
```

任务器侧（接两点任务器时）必须显式 `localization_watchdog > 0`（建议 2.0），与发布端发布节奏分开。

## 6. 测试

- **离线状态机单测**（本机已跑）：`python -m pytest test_loc_health_core.py -q` → **20 passed**。
  覆盖：启动无数据 / 未标定 / 静止 AMCL 不更新仍健康（短静止+60s 长静止）/ 累计位移·累计转角超门限
  后不更新 / 累计运动未达门限（微动）不判失效 / scan·odom 断流 / TF 缺失·不可查 / NaN·Inf·无效四元数 /
  协方差位置·角度超阈值 / 一致性超阈值 / stamp 回退 / 故障恢复不锁死。
- **ROS 隔离集成测试**（本机 WSL ubuntu2204-ros2 已跑）：`python3 loc_health_integration_test.py` → **10/10 通过**。
  验 ROS 胶水层：无数据 False / 健康 True / 停 scan False / 停 odom False / 恢复 True / stamp 回退 False /
  未标定 False / 影子话题不干扰主话题 / 影子话题有消息 / 发布端退出停止发布。
- **任务器 watchdog 生效**（发布端退出 → 任务器 STALE）：由 racecar_mission 的 `offline_regression.py` 已覆盖，
  发布端不重复；本测试只验前提（发布端退出后确实停止发布）。

## 7. 现场标定流程（影子运行 → 标定 → 接任务器）

1. **先确认现场跑的是 AMCL 还是 SLAM**：若仍在建图，**不采 bag、不中断建图、不同时启动 AMCL**；
   待已有地图的 AMCL 定位链就绪后再采。
2. 单点导航地面通过后，起发布端（`verified=false`）影子运行，用**独立话题**（见 §5 影子运行）记录判定，
   避免 False 误取消其他会话任务。
3. 采集正常静止 + 短距离移动数据（分段短推，不固定 5m，含安全小转弯，记录每段实际起止与距离），
   据此定阈值（见 `localization_health.yaml` 头注标定方法）。
4. 核对误报（健康时误判 False）、漏报（失效时误判 True），调阈值。
5. 阈值稳定后写回 `localization_health.yaml` 置 `verified: true`，再起任务器（`localization_watchdog:=2.0`），
   验证停发→STALE→拒派发、恢复→resume→续跑。

**阈值证据红线**：一次正常 bag 只描述"正常范围"，**不能证明** `cov_pos_sigma/cov_yaw_sigma/amcl_stale_moving`
等阈值合理，也**无法**提供发散/丢失/时间回退等故障的区分能力。故 `verified` 在录完一次 bag 后**仍保持 false**，
须经误报/漏报核对后才置 true。

## 8. 红线（不违反）

- 本机离线，**不部署车端、不宣称定位健康已验证**。
- 阈值未标定前 `verified` 保持 false，**不偷偷用默认值放行**。
- 单点通过前发布端可影子运行；**接两点任务器仍须现场验证**。
- 停止链另行验证，发布 False ≠ 保证停车。
