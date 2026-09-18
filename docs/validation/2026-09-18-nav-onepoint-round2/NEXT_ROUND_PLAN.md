# 下一轮分阶段验证清单(2026-09-18 早按用户 8 条约束修订,v2 取代昨晚版)

> **用户 8 条约束(0918 早,本轮总纲,原文要义)**:
> 1. 先拉历史日志/当前配置/车端文件校验值再启动测试;关机前日志不预设能找回,缺失保留缺口;
> 2. **原始串口采集只能有一个读取者**——不得另开程序与 encoder 节点同读一串口(分走字节制造不完整帧);
>    在唯一读取入口记录原始字节块+单调时钟+请求信息,同一份字节流送旧/新解析器比较;离线重放只诊断不注入控制链;
> 3. 实车响应协议必须由捕获字节确认(地址/功能码/长度/数据语义/CRC/异常响应);
>    请求 CRC 正确只支持协议判断不替代响应实证;确认前 fixed 输出只旁路观察不供 EKF;
> 4. 先验证解析器→再验证 gate→最后切 EKF 输入;**每次只改一个环节**,不同时换解析器/闸门/比例系数;
> 5. 无动力推车分别记录: 卷尺距离/原始计数/旧解析输出/新解析输出/gate 输入输出/EKF/AMCL;
>    2.2× 比例与本次跳变是**两个待核查项**,CRC 修复不代替距离标定;
> 6. ±10% 只是本轮初筛阈值,报告保留每次原始误差与重复性;至少重复前进+检查倒退;异常计数与过滤比例入记录;
> 7. gate 除"不伪造新鲜零值"外,须确认连续无有效数据时 EKF/导航如何识别处理;
>    停发 encoder 数据≠整车自动停车;先在隔离测试证明失效可见+主线处置路径,不让导航默默沿用预测位姿;
> 8. 无动力验证结束先汇报;动力与导航恢复按现场条件分别确认;不自动 push,不自动替换现役文件。

---

## 阶段 0:取证(不动链,第 1 条)

```bash
# 车机上线后,启动任何测试前:
ssh hwcar 'md5sum ~/racecar_team/*.py ~/racecar_team/*.yaml ~/racecar_team/*.sh 2>/dev/null' > 车端校验值.txt
ssh hwcar 'md5sum ~/racecar/install/racecar/share/racecar/config/ekf.yaml'          # 现役 EKF 配置指纹
scp hwcar:'~/racecar_team/{nav_last,safety_last,runcar_last,encoder_gate_last}.log' 日志目录/ 2>/dev/null || echo "日志缺失=保留缺口"
scp hwcar:~/racecar/install/racecar/share/racecar/config/ekf.yaml ekf_车端当前.yaml
```
- 有日志 → 归档对照 EVIDENCE_TIMELINE 补时间戳;无 → 记"U2 无法补证",继续。

## 阶段 A:读取者交接 + 全矩阵无动力推车(第 2/5/6/7 条;唯一被改环节=读取者)

**A0 链构成**:
```
Run_car.launch.py(全起) → kill 厂家 encoder 进程(仅进程,安装目录不动)
→ encoder_tap.py 独占 /dev/encoder:
     /encoder        = 旧解析语义顶替(EKF 输入话题/语义均不变)   ← 唯一改变=读取者
     /encoder_fixed_obs = 新解析旁路(无人消费,只记录)           ← 第 3 条
     tap_*.csv       = 原始字节+单调时钟+请求帧+双解析值逐行
→ gate v2 旁路并跑(--record-dir,订 /encoder 发 /encoder_gate 无人消费)  ← 输入输出双 CSV
→ EKF(输入 /encoder 不变) → AMCL 链
```
交接窗口注意: kill→tap 起之间 EKF 短暂无输入,静止状态 2s sensor_timeout 内无碍;
**严禁**在 tap 运行期间对 /dev/encoder 跑 cat/xxod/任何第二读取者(第 2 条)。

**A1 静态基线 30s**: 双解析值应一致且≈0;fixed_reject 应≈0(若静置就拒 → 协议与推断不符,
停,先离线分析 CSV 字节——这正是第 3 条的字节实证)。

**A2 推车矩阵**(第 5/6 条,每段独立记录):

| 段 | 动作 | 记录 |
|---|---|---|
| F1-F3 | 前进 1m ×3 次(卷尺量,慢推) | 卷尺值/tap CSV(原始计数+旧值+新值)/gate input+output/odom_combined/amcl_pose |
| R1 | 后退 1m ×1 | 同上(裁决 0x0003 符号语义 U4) |
| 电调试验 | 电调**断开状态下**仅记录(无动力轮不动,可省) | — |

±10% 仅初筛: 每段误差原值全部入表(不取平均了事),重复性=3 段间散布;
异常计数(legacy_err/timeout/fixed_reject/字节坏块)与 gate 过滤比例(drop/pass)入表。
**2.2× 与跳变分开核查**(第 5 条): 推车位移对账查比例;CSV 字节流查跳变成因;互不替代。

**A3 失效可见性隔离实验**(第 7 条):
```bash
# tap --stop-publish-after 60: 60s 后停发 /encoder(继续读串口记录)
```
观察并记录: ①EKF odom_combined TF 停更时刻(预期 ≈2s sensor_timeout)②AMCL 是否停止更新
(预期: 无运动输入不更新)③gate diag age_in/out 双停摆 ④**关键确认: 整车不会自动停**——
若此时导航在跑,cmd_vel 仍会来(安全层只兜 cmd_vel 断流,**不兜 odom 断流**)。
主线处置路径(写死): 无动力阶段=人工观察;动力阶段=发现 odom 停更即人工终止导航,
不依赖任何自动机制。此实验结论入报告后才允许进动力阶段。

**A4 阶段汇报(第 8 条)**: A 全部数据(CSV/对账表/失效实验)整理汇报,等用户确认后才有 B。

## 阶段 B:gate 切入(第 4 条;唯一被改环节=EKF 输入加 gate)

```
EKF: odom0 /encoder → /encoder_gate(gate v2 输出=tap 旧解析过滤后)
```
- ekf.yaml 改前 `cp ekf.yaml ekf.yaml.bak_0918_gateV2`;
- 推车复验 1 段(判据同 A,数据矩阵同记);
- 若有毛刺: 观察 gate 拒帧(diag+output.csv)而非 odom 跳=gate 生效直接证据;
- 汇报后进 C。

## 阶段 C:解析器切换(第 4 条;唯一被改环节=发布源)

```
tap --publish fixed  (/encoder 改发新解析值;旧解析仍记录对照;gate/EKF 不动)
```
- 前置: A 阶段 CSV 已确认响应协议(地址/功能码/长度/CRC/异常响应,第 3 条清单)且
  A2 中新解析与旧解析在干净段一致;
- 推车复验 1 段(含后退);毛刺窗口(若有)观察: 新解析拒于字节层(fixed_reject 而非 gate 拒帧);
- 汇报后,无动力验证整体结束。

## 阶段 D:动力前提(POWER_PREREQUISITES §五;需用户现场确认)

低速标定(0.50/0.55/0.60 卷尺+秒表)→可控最低速度→不可控则出控制标定方案不提速。
**D 前置硬条件: A/B/C 全过+失效可见性实验结论入档。**

## 阶段 E:导航恢复(最后)

安全链起→initpose(采帧对齐)→probe_now_cost 起点→单目标>0.6m 避左侧→电调断 2s 再通→
全程盯 gate diag+tap CSV→毛刺应见拒帧而非 odom 跳;仍跳→物理停止,证据保全,本轮终止。

## 终止条件(不变)

新阶跃跳变(>0.1m 单步)/gate diag 帧率塌陷/relocks 频发/车行为异常/用户喊停——任一即停,
当轮作废,**不二分猜因**(按 EVIDENCE_TIMELINE §五取证顺序: 先字节与逐帧值再结论)。

## 红线

不自动 push;不自动替换现役文件(tap/fixed/gate v2 全部旁路或进程级顶替,厂家文件零改动);
不自动启动动力链。

## 工具与交付物索引(0918 早新增)

| 文件 | 角色 |
|---|---|
| `tools/realcar/encoder_fix/encoder_tap.py` | **唯一读取者**: 原始字节+单调时钟+请求记录,双解析同流对照,/encoder 旧语义顶替+fixed 旁路,--publish 切换环节,--stop-publish-after 失效实验 |
| `test_encoder_proto.py` 40/40 | 含 tap 五用例(干净一致/毒字节矩阵/legacy IndexError 防护/超时不发零/混合统计) |
| `encoder_gate_v2.py` 13/13 | gate 旁路并跑用 --record-dir 双 CSV |
| 昨晚五文档 | EVIDENCE_TIMELINE/PROTOCOL_BASIS/GATE_REVIEW/POWER_PREREQUISITES/本文件(v2) |
