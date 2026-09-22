# 定位健康接口 /localization_ok 定义(两点模块接入前置,2026-09-19)

> 对象:racecar_mission 两点任务器订阅的 `localization_status_topic`(默认 `/localization_ok`,
> `std_msgs/Bool`)。本文定义「主线应如何产生这个 Bool」,供落地发布者 + 现场标定阈值。
> **本轮回不交付发布者节点**(见 §4),先定义判据与接口,避免"恒 True"伪安全。
> **当前状态(2026-09-20 更新):任务器就绪,健康发布端已实现(本机离线,未部署未标定)。** 发布端 =
> `tools/realcar/localization_health.py`(ROS 节点) + `loc_health_core.py`(纯 Python 状态机,离线 19/19 单测),
> 详见 `tools/realcar/LOCALIZATION_HEALTH.md`。**待现场影子运行 + 标定阈值 + 接两点任务器验证**;
> 阈值未标定前 `verified=false` 不发 True,不宣称实车定位健康已验证。

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

### 判据局限(2026-09-20 补,实现发布者前必须消化)

三条判据落地前须注意以下陷阱,否则会得到"看似健康实则失效"或"假失效":

1. **amcl_pose 静止不更新 ≠ 失效**:AMCL 在机器人静止、无新 scan 输入时**可能不持续发布新估计**。
   判据 1「新鲜度」不能简单用「`/amcl_pose` 多久没更新」判失效,应区分:
   - 有 scan 输入、有 TF 查询,AMCL 仍长时间不更新 → 真异常;
   - 机器人静止、无新观测,AMCL 自然不更新 → 不是失效。
   `T_fresh` 须与"有无新观测"联动,不能只看裸时间戳 age。
2. **AMCL↔odom 一致 ≠ 独立证明定位正确**:AMCL 以 odom 为运动模型输入,**两者共享里程计信息**。
   判据 3「一致性」只能证明"AMCL 没比 odom 多漂",**不能**证明"odom 本身没漂/定位确实正确"——
   里程计漂移时两者会**一起漂**,一致性照常成立,却是伪健康。
   一致性是"AMCL 内部自洽"的兜底,不是定位真值的独立旁证;真值只能靠 scan 匹配/回环/人工核对。
3. **保留 UNKNOWN/False/STALE + watchdog,不绕过**:无发布者时 `UNKNOWN`、收到 `False`、watchdog 超时 `STALE`
   三种都须拒绝 start/resume/dispatch(消费端已实现)。**不得**为演示恒发 True、禁用 watchdog
   (`localization_watchdog=0`)、或把"AMCL 有消息/协方差小"当健康。

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
3. ~~实现发布者节点~~ **已实现** → `tools/realcar/localization_health.py` + `loc_health_core.py`
   (离线单测 19/19,阈值外置 `verified=false` 未标定不发 True),详见 `tools/realcar/LOCALIZATION_HEALTH.md`。
4. 起 mission(`localization_watchdog:=2.0`),验证:停发→STALE→拒绝派发;恢复→resume→续跑。
