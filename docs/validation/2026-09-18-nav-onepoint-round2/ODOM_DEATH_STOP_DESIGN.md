# 里程计失效后如何真正停住 —— 设计 + 离线复现(2026-09-18,纯离线)

> 状态: **设计文档,非部署件**。本轮不部署新保护节点、不改 cmd_vel_safety 现役文件;
> 本方案给出"要不要停、靠什么停、怎么恢复"的最小改法,供下次上车前由用户拍板后再实施。
> 复现脚本: `tools/repro_odom_death.py`(行为模型推演,数值 [配置]/[源码] 推断,不冒充实测)。

---

## 1. 失效链: 编码器死在运动中,今天没有任何一层停车

链路: `encoder(/encoder) → gate(/encoder_gate) → EKF(odom_combined) → AMCL → controller(/cmd_vel) → cmd_vel_safety(/car_cmd_vel)`

逐层在"编码器串口死"后的真实行为(依据见 §2 证据):

| 层 | 失效后行为 | 会不会让车停 |
|---|---|---|
| encoder_node | timer 回调阻塞在 `ser.read(7)`(现役 Serial 无 timeout,源码行 100)→ `/encoder` **变 stale,不发零** | 否 |
| encoder_gate(v1) | 只在收到 `/encoder` 时转发 → `/encoder_gate` 也 stale | 否 |
| EKF | 到 `sensor_timeout=2.0s` 才剔除 odom0,但**仍按 20Hz 发 odom_combined**;vx 变纯预测(process_noise 1e-5 极小→不衰减,停在最后融合值),位置继续积分 = **静默漂移** | 否 |
| AMCL | 拿漂移 odom 继续跑运动模型 → 位姿错但**不知** | 否 |
| controller | 按 AMCL 位姿判断"没到目标" → **继续发 0.55**,`/cmd_vel` 一直新鲜 | 否 |
| cmd_vel_safety | 唯一停车条件 = `/cmd_vel` 超时 0.5s;`/cmd_vel` 新鲜 → **一直放行非零** | **否** |
| 物理 | 唯一可靠停 = 人手动断电源 | — |

**结论**: 现有链路没有任何一层会把"odom 源死了"转成停车。车会带病盲跑,直到撞障或人断电源。

---

## 2. 三个信号源的区分(关键,决定监控源选谁)

| 信号 | 失效后的表现 | 能否作"里程计健康"触发 |
|---|---|---|
| **编码器新鲜度** `/encoder_gate` 最后消息年龄 | 失效即 stale(age 持续增长) | ✅ **能,最直接**——直接反映物理里程计源是否还在供数据 |
| **EKF 输出时间戳** `odom_combined.header.stamp` | 失效后仍按 20Hz 推进(age 恒≈0) | ❌ **假健康**——EKF 预测期照发,时间戳不能证明内容有效 |
| **定位有效性** AMCL(协方差/粒子扩散/score) | 随漂移劣化,但 AMCL 不自断 | ⚠️ 慢噪声,只宜后验复核,不作主触发 |

依据([配置]/[源码],stage0_evidence 留存):
- `ekf_车端当前.yaml`: `odom0: encoder_gate`(唯一纵向供血)、`sensor_timeout: 2.0`、`frequency: 20.0`、`process_noise_covariance` 首项 `1e-5`(速度状态近乎不衰减)。
- 现役 `encoder_node_现役车端.py`: `create_timer(0.02)`→50Hz;`serial.Serial(port,baud)` 无 timeout → `read(7)` 阻塞。
- `cmd_vel_safety.py`: 仅 `on_cmd` 更新 `last_cmd_t`,`on_tick` 只查 `/cmd_vel` 超时,未订阅任何 odom 话题。

**选监控源 = `/encoder_gate`**。理由:(a) 它同时覆盖两类失效——encoder 节点死、gate v1 死锁(B1)/断流,都表现为 `/encoder_gate` stale;(b) 它在 EKF 之前,是最早的失效观测点,能在 `sensor_timeout=2s` 剔除之前就停。

---

## 3. 最小安全链方案(扩展现有 cmd_vel_safety,不新增节点)

把"odom 源新鲜度"作为与现有"cmd 新鲜度"并列的第二停车条件,复用既有 `publish_zero`/限幅/重发结构。

| 环节 | 设计 | 说明 |
|---|---|---|
| **监控源** | 订阅 `/encoder_gate`(参数 `odom_topic`,默认 `/encoder_gate`),记录 `last_odom_t` | 不订阅 odom_combined(假健康) |
| **故障触发** | `age(/encoder_gate) > odom_timeout`(默认 1.0s,可配)即判故障 | 1.0s = 50Hz 下 50 帧漏失,宽限充足;须 < 2.0s(EKF sensor_timeout)以便先停;须 > 0.5s 避瞬时抖动 |
| **置零路径** | 触发即 `publish_zero("odom 源失联")`,并按 `zero_repeat` 持续重发 | 复用现有 `publish_zero`(motor=1540 字节级已验证) |
| **锁存** | 故障进入 latched 态;latched 期间不因单帧新 odom 解除 | 防抖动;参考 gate v2 `RELOCK_N=3` 思路 |
| **恢复** | latched 退出 = odom 连续 N 帧新鲜(默认 3)+ age<odom_timeout | **不自动重发旧指令**(见 §4) |
| **宽限** | 触发宽限 = odom_timeout(非瞬时);恢复宽限 = 连续 N 帧 | 两侧都留裕量,不误停、不误恢复 |

参数建议(全部新增、有保守默认、与现有 cmd_timeout 并列):
`odom_topic=/encoder_gate`、`odom_timeout=1.0`、`odom_relock_n=3`、`odom_rearm_mode=manual`。

---

## 4. 恢复策略: 不得自动恢复旧指令

失效前 car 在 0.55 巡航,编码器死、车已漂移(可能已撞/已偏)。此刻若"odom 恢复新鲜"就自动续跑,等于把一段盲跑后的旧目标继续执行,危险。因此:

- **锁存期间**: 持续发零,**不缓存/不重发失效前那条非零指令**。
- **退出 latched 后**: 不主动恢复非零;只有当**收到一条新的 `/cmd_vel`**(即导航层重新决策后的新指令)且 odom 源已确认新鲜,才恢复正常中继。安全层不替导航层"补发旧目标"。
- 两档可选(参数 `odom_rearm_mode`):
  - `manual`(默认,推荐): 退出 latched 后仍需人工 re-arm(service/参数),因为"失而复得"后车已漂移,是否续跑应由人判断。
  - `auto`: 仅要求"新指令 + 连续新鲜",不要求人工确认(适合已验证低速、短距离场景)。
- 本次**推荐 manual**,与"低速标定未做、死区未确认"的现状一致:任何自动续跑都属"自动恢复旧指令"风险。

---

## 5. 复用现有机制清单(不新增节点/不改 driver/不改 EKF)

| 复用项 | 位置 | 说明 |
|---|---|---|
| `publish_zero` + `zero_repeat` | cmd_vel_safety.py:61/38 | 置零路径与重发周期,直接复用 |
| 限幅逻辑 | cmd_vel_safety.py:76-83 | odom 触发停车不经过限幅,但恢复后仍走原限幅 |
| 超时/latch 结构 | cmd_vel_safety.py:98-110 | 现有 `in_timeout` 已是一个"锁存-重发-恢复"范式,odom 条件并列为第二个同构条件 |
| SIGINT 退出零 | cmd_vel_safety.py:119-146 | 不动的现有机制 |

**明确不做**: 不新增独立"保护节点";不改 driver 公式;不改 EKF 配置(odom0 剔除逻辑不动);不扩展 encoder 工具(工具已冻结)。

---

## 6. 边界与已知缺口(如实,不写成已解决)

1. 本方案只能停"**odom 源死了**(无数据)";停不住"odom 活着但内容错"(如 0918 毛刺)——后者靠 gate/解析器修复(阶段 A/B/D)。
2. 停不住"odom 正常但 AMCL 定位错"——那是定位有效性问题,AMCL 自诊断是另一个话题,本方案仅把它列为可选后验复核,不作为主触发。
3. `/encoder_gate` 是 gate v1 现役的发布名;若阶段 D 换 v2,`odom_topic` 参数需同步确认(仍同名,无需改)。
4. 监控 `/encoder_gate` 无法感知"gate 自己发的帧内容错误但帧不断"——那是 gate 语义层问题,不是新鲜度层。
5. 本方案只在 **cmd_vel_safety 存活**时才生效;若 safety 节点也死,仍只剩物理断电源(现役无 watchdog 兜 safety 本身,属独立已知缺口)。

---

## 7. 上车验证项(仅阶段 D 之后,需动力,另行授权)

- 无动力下先验证"停发 encoder → safety 应发零"的可观测性(`--stop-publish-after` 类实验,复用 tap)。
- 有动力下验证"巡航中拔编码器串口 → 车应 ≤ odom_timeout 内停住",全程有人工断电源兜底。
- 验证 latched 退出不自动续跑、manual re-arm 路径。

---

## 8. 结论

现役链路在"编码器死"时**无自动停车**,唯一可靠停是物理断电源。最小改法 = 在现有
cmd_vel_safety 内加第二停车条件(监控 `/encoder_gate` 新鲜度),复用 `publish_zero` 置零,
锁存后不自动恢复旧指令(推荐 manual re-arm)。选监控源必须用 `/encoder_gate` 新鲜度,
**不能用 odom_combined 时间戳**(假健康)。本设计待用户拍板后实施,不在本轮离线收尾中部署。
