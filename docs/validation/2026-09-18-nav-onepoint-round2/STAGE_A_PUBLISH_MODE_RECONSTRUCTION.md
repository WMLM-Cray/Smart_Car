# 阶段 A 实际发布模式 / 话题 / EKF 订阅关系重建（2026-09-19 离线实证）

> 状态：纯本地离线重建，不连接车机。依据 = 阶段 A 落盘 CSV 的 `published`/`fixed_vel`
> 列 + `stage0_evidence` 启动日志 + `ekf.yaml`（现役 / .bak_pregate0918）。
> 原则：**以实际日志/CSV 为准重建实际行为，不按阶段名称（"fixed 旁路"等）推定。**

---

## 0. 一句话结论

阶段 A 两次采字节（静止 998 帧 + 推车 10343 帧），`/encoder` 话题**实际发布的是
`legacy`（厂家语义顶替），不是 `fixed`**；`fixed` 只作为旁路记录在 CSV 的 `fixed_vel`
列并另发 `/encoder_fixed_obs`。而当时的 `legacy` = **B 版（索引搜索、符号错误）**，
因此推车后退段 `/encoder` 上**实际发布过 +669 量级的正大数**——只是阶段 A 全程 EKF 未运行
（无消费者），未污染位姿。这与 `STAGE_A_TAP_PROTOCOL.md` §6 所写命令
`--publish fixed` **不符**，属"计划 vs 执行差异"，本文如实记录。

---

## 1. 数据来源

| 证据 | 内容 | 结论 |
|---|---|---|
| `tap_static_20260918.csv` | 998 行，`published` 列全部 `legacy`，`fixed_vel` 列全部非空 | 静止采集 `/encoder` 发布 legacy |
| `tap_push_20260918.csv` | 10343 行，`published` 列全部 `legacy`，`fixed_vel` 列全部非空 | 推车对账 `/encoder` 发布 legacy（**非文档写的 fixed**） |
| `runcar_last.log` | 0918 现役 Run_car 链，`encoder_node-7` 现役节点因串口多访问崩溃（`SerialException`） | 该日志是现役链，非阶段 A tap；阶段 A 时 EKF/encoder 均未跑 |
| `safety_last.log` | `cmd_vel_safety` 起来，`max_linear=0.55`（0918 污染配置，现本地已回退 0.10） | 阶段 A 与安全层无直接关系；0.55 是当时事故残留 |
| `encoder_gate_last.log` | gate v1 崩溃 `RCLError: publisher's context is invalid` | gate 未参与阶段 A |
| `ekf_车端当前.yaml` | `odom0=encoder_gate` | 现役 EKF 走 gate 供血 |
| `ekf_bak_pregate0918.yaml` | `odom0=encoder` | 回退基线：EKF 直接订阅 `/encoder`，不经 gate |

---

## 2. 阶段 A 实际发布模式（由 CSV 逐行统计）

| 采集 | 帧数 | `published` 分布 | `/encoder` 实际发布值 | `/encoder_fixed_obs` 实际发布值 |
|---|---|---|---|---|
| 静止 | 998 | 100% `legacy` | legacy（当时 = B 版） | fixed（新解析器，旁路） |
| 推车 | 10343 | 100% `legacy` | legacy（当时 = B 版） | fixed（新解析器，旁路） |

- `encoder_tap.py` 的 `published` 列语义：`legacy` = `/encoder` 用 `--publish legacy`
  （**默认值**）发布 `legacy_vel`；`fixed` = 用 `--publish fixed` 发布 `fixed_vel`；
  `stopped` = `--stop-publish-after` 生效后停止发布。
- 两个 CSV `published` 全为 `legacy` → 两次采集**都用了默认 `--publish legacy`**，
  **没有传 `--publish fixed`**。

### 2.1 与文档的矛盾（计划 vs 执行差异）

`STAGE_A_TAP_PROTOCOL.md` §6 推车命令记为：
```
python3 encoder_tap.py --out-dir … --duration 210 --publish fixed
```
但 CSV `published` 列 10343/10343 全为 `legacy`。即**文档写的命令与实际执行不符**：
实际推车对账是 `--publish legacy`（默认）跑的，`fixed` 值虽被计算并记入 `fixed_vel` 列、
并发布到 `/encoder_fixed_obs`，但 `/encoder` 上发的不是它。

---

## 3. EKF 订阅关系（由 ekf.yaml 重建）

两条链共用的 EKF 配置（现役与 .bak 完全一致的部分）：

| 项 | 值 |
|---|---|
| `odom0` | 现役 `encoder_gate`；回退 `.bak_pregate0918` = `encoder` |
| `odom0_config` | `[false, false, false]`（vx/vy/vyaw 融合） |
| `odom0_relative` | `true` |
| `imu0` | `IMU_data`（IMU_publisher 话题，`/IMU_data`） |
| `two_d_mode` | `true` |
| `sensor_timeout` | `2.0` s |
| `frequency` | `20.0` Hz |
| `publish_tf` | `true` |
| `odom_frame` / `world_frame` | `odom_combined` |
| `base_link_frame` | `base_footprint` |

即 EKF 的里程计供血是 **`/encoder`（回退基线）或 `/encoder_gate`（现役）**，
融合 `vx/vy/vyaw` + IMU yaw。阶段 A 若起 EKF，`odom0` 必须指到 tap 发布的 `/encoder`
（回退 `.bak_pregate0918`），而 tap 当时发的是 B 版 legacy。

---

## 4. 影响与结论（如实）

1. **推车对账结论仍成立**：`analyze_push3.py` 的 `Δ速度积分` 是从原始 `resp_v` 字节
   按有符号 16 位独立算得，**不经过 `legacy_vel` 列**，后退段 Δ=−4923（负）与 Δ圈数
   −4956 符号一致。故对账用的是正确有符号解析，与 `published=legacy`（B 版）无关。
2. **`/encoder` 后退段实际发布过 +669 正大数**：因为当时 `legacy`=B 版符号错误，
   且 `published=legacy`。**仅因阶段 A 全程未起 EKF（无消费者），未造成位姿污染**。
   —— 这正是 `PARSER_SOURCE_PROVENANCE.md` §4.3 预警的"遗留风险"的**实际发生场景**，
   只是当时无下游接住。
3. **本次修复消除了该风险**：`DueVelData` 恢复为 A 版（符号正确），B 版隔离为
   `DueVelData_index_buggy`；离线重放 343/343 后退负帧 A 版符号全对、B 版全错
   （见 `verify_parser_fix_replay.py`）。但**修复只改了本地源码**，车端
   `~/racecar_team/` 仍是旧配置，下次上车第一步先核对车端再定。

---

## 5. 本次不做的（边界）

- 不据此改写 `STAGE_A_TAP_PROTOCOL.md` §6 的历史命令记录（那是当时执行现场的事实快照，
  保留原文，差异在本文 §2.1 交叉引用说明；是否回补注释由用户定）。
- 不扩展新工具、不改 `k`、不连接车机、不自动 push。
- 第④问失效实验的 A/fixed 决策见 `STAGE_A_Q4_FAILURE_TEST.md`（本文 §6 指向）。

---

## 6. 与第④问的衔接

第④问失效可见性实验清单已更新：明确 `/encoder` 用 **A 版 legacy**（修复后符号正确的
厂家原版），不用 fixed、不用 B 版；并写明理由与风险边界。详见
`STAGE_A_Q4_FAILURE_TEST.md`。
