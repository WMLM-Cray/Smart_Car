# 定位健康接口 /localization_ok 定义(两点模块接入前置,2026-09-19)

> 对象:racecar_mission 两点任务器订阅的 `localization_status_topic`(默认 `/localization_ok`,
> `std_msgs/Bool`)。本文定义「主线应如何产生这个 Bool」,供落地发布者 + 现场标定阈值。
> **本轮回不交付发布者节点**(见 §4),先定义判据与接口,避免"恒 True"伪安全。

---

## 1. 接口形态

| 项 | 值 |
|---|---|
| 话题 | `/localization_ok`(可经 launch 参数 `localization_status_topic` 覆盖) |
| 类型 | `std_msgs/msg/Bool` |
| 语义 | `True`=定位健康(可发目标)/ `False`=不健康 |
| 发布节奏 | 固定周期(建议 2Hz)持续发布;停止发布即失效(靠 mission 侧 watchdog 判 STALE) |
| QoS | 默认(RELIABLE + VOLATILE);mission 侧 `create_subscription(Bool,...,10)` 默认 QoS |

---

## 2. 健康判据 v0(三条 AND,阈值待现场标定)

> 红线:健康判据**不能**仅凭"AMCL 有消息"或"协方差小"就发 True。三条同时满足才发 True:

1. **新鲜度**:`/amcl_pose` 时间戳新鲜(age < `T_fresh`,建议 0.5s)且 TF `map→base_footprint`
   可查且新鲜,且 `/odom_combined` 新鲜(EKF 存活)。
2. **协方差有界且不发散**:AMCL pose covariance 的 x/y 对角线开方 σ_xy < `T_cov`(建议 0.5m),
   且窗口内 σ 不持续增长(发散阈值 `T_divg`,待标定)。—— 判据是"有界",不是"小";
   粒子收敛但定位偏错的场景靠 §3 一致性兜底。
3. **一致性**:AMCL 位姿与 `/odom_combined` 位姿差 < `T_cons`(建议 0.5m)。—— 防"协方差小
   但定位已漂移"的伪健康。

三条任一不满足 → 发 `False`;恢复满足后重新发 `True`(不锁死)。

---

## 3. 失效/过期/运行中处置(由 racecar_mission 已实现,此处对齐)

| 事件 | mission 行为 | 来源 |
|---|---|---|
| 收到 `False` | 立即判 `UNHEALTHY`;`~/start`/`~/resume` 拒绝;运行中取消在途目标 + 停止派发(`localization lost`) | 复审 §2.3 |
| watchdog 超时无新消息 | 判 `STALE`(等价不健康) | `localization_watchdog>0` 时生效 |
| 恢复健康 | 需显式 `~/resume` 重审后才续跑 | 复审 §2.3 |
| **无发布者** | 定位状态 `UNKNOWN` → **拒绝 start(失效安全,默认锁死)** | 复审 §2.3 |

**watchdog 设置(实车必做,不能省):**
- 启动显式传 `localization_watchdog:=2.0`(0.0 = 禁用,失去 STALE 检测,禁用=绕过门禁)。
- 必须**验证超时行为**:发布者停发后,mission 应在 watchdog 内判 STALE 并处置,
  **不能只把参数写上不验证**。

---

## 4. 本轮边界(明确不做)

- **不交付发布者节点**:判据阈值(`T_fresh/T_cov/T_divg/T_cons`)需现场数据标定,
  离线无法定。先落接口定义 + 判据,现场标定后再实现发布者。
- **不实现"恒 True"发布器**、**不把 AMCL 有消息/协方差小当健康**、**不把 watchdog 设 0**。
- 若两点演示前发布者未完成 → **保留单点演示,不临时削弱门禁**(单点不经过 mission 门控)。

---

## 5. 落地顺序(供主线后续)

1. 单点地面通过 → 用真实坐标生成两点 `mission_realcar_first.yaml`(`enabled:true`)。
2. 标定健康判据阈值(现场采集 AMCL pose/odom/TF 的时间戳与协方差)。
3. 实现发布者节点(按 §2 三条 AND,2Hz,严格不恒 True)。
4. 起 mission(`localization_watchdog:=2.0`),验证:停发→STALE→拒绝派发;恢复→resume→续跑。
