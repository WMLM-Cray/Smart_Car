# 下一轮分阶段验证清单(2026-09-18 制定,离线)

> 明日路线总则(任务书): **先无动力受控推行**验证原始计数/过滤结果/EKF/AMCL 连续性;
> **动力验证另行确认现场条件**;先验证停止和基本控制,再恢复导航;
> **出现新位姿跳变或异常运动立即终止该轮,不重复发车碰运气**。
> 红线: 不自动 push;不自动替换厂家程序;不自动启动动力链(动力动作须用户现场确认)。

## 阶段 0:开机先取证(不动任何链)——补 U2

```bash
# 车机开机、厂家底座自起后,第一时间把 0918 遗留日志拉回本地(可能已被覆盖则如实记录)
scp hwcar:'~/racecar_team/{nav_last,safety_last,runcar_last,encoder_gate_last}.log' \
    D:/Claude/projects/hwcart_amcl_stage2/logs_0918/
```
- 有 → 归档,对照 EVIDENCE_TIMELINE §一时间线补时间戳;
- 无/被覆盖 → 记录"U2 无法补证",不纠结,继续。

## 阶段 A:无动力被动验证(核心轮,零风险)

**A1 原始字节旁录(U1,最优先)**: 厂家链照常跑,车机上对 /dev/encoder 旁录:
```bash
# tee 双写方案: 停厂家 encoder 节点会断链,故用只读并发抓取
timeout 60 cat /dev/encoder | xxd > /tmp/enc_bytes_$(date +%s).hex   # 若独占冲突则改用厂家节点暂停窗口
```
- 推车前进 1m(慢)→ 停 → 后退 1m → 停,全程在录制窗口内;
- 离线分析: 响应帧 7 字节+CRC 复算吻合率;**电调断通瞬间各录 10s**(取证毛刺形态);
  前进/后退读数极性 → 裁决 U4 符号语义。

**A2 gate v2 只读并跑(不进 EKF)**:
```bash
python3 ~/racecar_team/encoder_fix/encoder_gate_v2.py --record-dir /tmp/gate_rec
# 观察输出话题名冲突: v2 订/encoder 发/encoder_gate——与 v1 同契约,不可与 v1 同跑!
# 先确认 v1 是否在链上(safety_nav_one 起则 kill v1 或直接不起导航链,只跑底座+gate v2)
```
- 慢推车: input.csv 全量、output.csv 应 ≈100% 放行(慢推 Δv<<0.35);
  若误拦 → 看 diag 帧率是否塌陷(GATE_REVIEW §一 B3),先查串口不急调门限。

**A3 EKF/AMCL 连续性**(与 A2 同窗): 起定位链 → 推车 1m → AMCL 位移对账 ±10%
(0917 同判据)+ odom 对账;**任何一次阶跃跳变 → 立即停,保全 input.csv/字节录,
终止当轮**(不二分猜因,按 EVIDENCE_TIMELINE §五取证顺序来)。

## 阶段 B:解析器切换(仅 A 全过后;每步可独立回滚)

| 步 | 操作 | 回滚 |
|---|---|---|
| B1 | 只读对照 5min: `encoder_node_fixed.py` 以 /encoder_fixed 话题并跑(不接 EKF),diag 行 ok≈50Hz 且值与厂家 /encoder 一致 | Ctrl+C 即回 |
| B2 | 切换: 停厂家 encoder 单节点(不动安装目录)→ 起 fixed 版 remap `/encoder_fixed:=/encoder`(或 EKF odom0 指 /encoder_fixed,同 .bak 机制) | 重启厂家节点+撤 remap |
| B3 | 切换后重跑 A3(推车对账 ±10%) | 同上 |
| B4 | gate 保留(v1 或换 v2,两者契约同): 解析器拦字节层,gate 拦语义层,双层独立 | — |

ekf.yaml 已有备份链: .bak_pregate0918(0918 留);每次改动前再 `cp ekf.yaml ekf.yaml.bak_<日期>_<事由>`。

## 阶段 C:动力前提(需用户现场确认电调/地面/清场)

按 POWER_PREREQUISITES §五执行: 低速标定(0.50/0.55/0.60 卷尺+秒表)→
定可控最低速度 → 不可控则出控制标定方案(不提速了事)。
**C 前置硬条件: A3+B3 全过且期间零跳变。**

## 阶段 D:导航恢复(最后)

1. 起安全链(safety_nav_one.sh,含 gate+0.55)→ initpose(采帧对齐,勿用旧候选硬套);
2. probe_now_cost 查起点干净;
3. **单目标 >0.6m**(0917 假达教训),方向避开左侧障碍;
4. 电调断 2s 再通(唤醒,0918 流程);
5. 全程盯 gate diag 1Hz + input.csv: 毛刺来时应见"拒帧"而非 odom 跳——**这是
   防线生效的直接证据;若 odom 仍跳 → 立即物理停止,本轮终止,证据保全**;
6. 顺利 → 1m/2m 多目标;最后归档提交。

## 终止条件(任一即停,当轮作废)

- AMCL/odom 出现新阶跃跳变(>0.1m 单步);
- gate diag 显示帧率塌陷(age_in 持续增长)或 relocks 频发;
- 车行为异常(异响/不响应停止/方向反);
- 用户喊停。

## 本轮(0918 离线)交付物索引

| 文件 | 内容 |
|---|---|
| docs/validation/2026-09-18-nav-onepoint-round2/EVIDENCE_TIMELINE.md | 证据时间线+未确认项+假设分级 |
| docs/validation/2026-09-18-nav-onepoint-round2/POWER_PREREQUISITES.md | 动力前提复核 |
| 本文件 | 下轮分阶段清单+回滚 |
| tools/realcar/encoder_fix/encoder_proto.py | 协议库(CRC+滑窗重同步) |
| tools/realcar/encoder_fix/encoder_node_fixed.py | 修复版节点(含厂家旧解析器对照) |
| tools/realcar/encoder_fix/test_encoder_proto.py | 35 单测(全过) |
| tools/realcar/encoder_fix/encoder_gate_v2.py | 闸门 v2(B1/B2/B3 修复) |
| tools/realcar/encoder_fix/test_gate.py | 13 单测(全过) |
| tools/realcar/encoder_fix/PROTOCOL_BASIS.md | 协议依据+缺陷清单+切换清单 |
| tools/realcar/encoder_fix/GATE_REVIEW.md | 闸门审查报告 |
