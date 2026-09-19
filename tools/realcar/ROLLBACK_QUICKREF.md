# 回滚速查卡（2026-09-19，现场用）

> 集中车端**所有已做配置变更** + 对应**回退基线** + **回滚命令**，队友现场踩坑时快速回退。
> 原则：**cp 备份、不 mv、不覆盖现役文件**；回退前先 `md5sum` 记录现役，回退后再 `md5sum` 核对。
> 全程只读核对 + 精确 kill，**不用 `pkill -f`**。

---

## 0. 单点操作卡入口（先看这个，再谈回滚）

| 操作卡 | 路径 | 阶段 |
|---|---|---|
| 第一轮（只读盘点 + 无动力 + 显示） | `tools/realcar/FIRST_ROUND_CHECKLIST.md` | 当前首选入口 |
| 单点导航轮（0917 排障） | `docs/validation/2026-09-17-nav-onepoint/NAV_ONEPOINT_ROUND.md` | 历史 |
| 单点导航第二轮（0918 发车/碰撞/根案） | `docs/validation/2026-09-18-nav-onepoint-round2/NAV_ONEPOINT_ROUND2.md` | 历史 |
| 下轮计划（阶段顺序 v3） | `docs/validation/2026-09-18-nav-onepoint-round2/NEXT_ROUND_PLAN.md` | 阶段顺序权威 |

---

## 1. 车端配置变更总表（时间线 + 回退基线）

| # | 变更 | 时间 | 现役值 | 回退基线 | 影响面 |
|---|---|---|---|---|---|
| 1 | `ekf.yaml` odom0 切源 | 0918 | `odom0=encoder_gate` | `.bak_pregate0918`（`odom0=encoder`） | EKF 里程计供血走 gate 还是直连 |
| 2 | `nav_safe.yaml` scan 三处话题 | 0917 | `/scan_filtered` | `.bak_scanfilter` | AMCL + 双 costmap 订哪个激光话题 |
| 3 | `nav_safe.yaml` desired_linear_vel | 0918 | 车端 `0.55`（污染） | 本地已回退 `0.10`（1c09973，**未部署车端**） | RPP 控制器期望速度（nav2） |
| 4 | gate 版本默认 | 0919 晚 | 本地 `v2`（未部署）/ 车端 `v1`（现役） | `GATE_VERSION=v1` 显式回滚 | `/encoder` 过滤闸门 |
| 5 | `cmd_vel_safety` 限幅入口 | 0920 | 本地 `MAX_LINEAR=0.10`/`MAX_ANGULAR=0.35`（未部署） | 不传参=0.10/0.35 | 安全层硬限幅（区别于 #3 期望速度） |

> ⚠️ **本地 ≠ 车端，车端版本待取证**：上表"现役值"是 0917/0918 历史部署记录，本轮本机修改
> （第 3、4 项）**未部署**；且**另一会话已在车上改 driver、回退、重编译**，车端当前版本未知。
> 下次上车第一步先**逐文件核对车端真实值**（内容 + 哈希 + 差异），勿当本地已生效、勿凭 .bak 文件名直接回滚。

---

## 2. 逐项回滚命令

> **先检查备份再执行**：任何回滚前，先核对 `.bak` 文件的**内容、哈希、与现役的差异**，
> 确认它确实是预期基线（如 `grep odom0 <bak>` 应是 `encoder`），**不凭 .bak 文件名直接 cp 覆盖**。
> 若 `.bak` 内容与预期不符，停下回报，不执行。

### 2.1 EKF odom0 回退（encoder_gate → encoder）

车端 EKF 配置路径：`~/racecar/install/racecar/share/racecar/config/ekf.yaml`

```bash
# 记录现役
md5sum ~/racecar/install/racecar/share/racecar/config/ekf.yaml
# 回退（cp 备份、不 mv；bak 就是基线）
cp ~/racecar/install/racecar/share/racecar/config/ekf.yaml ~/racecar/install/racecar/share/racecar/config/ekf.yaml.now_$(date +%Y%m%d)
cp ~/racecar/install/racecar/share/racecar/config/ekf.yaml.bak_pregate0918 ~/racecar/install/racecar/share/racecar/config/ekf.yaml
# 核对 odom0 已回 encoder
grep -A2 'odom0:' ~/racecar/install/racecar/share/racecar/config/ekf.yaml
```

> 若只是**临时**用 `odom0=encoder` 起链（阶段 A 第④问失效实验），**不要覆盖现役**，
> 用路径传参指 `.bak_pregate0918` 即可（见 `STAGE_A_Q4_FAILURE_TEST.md` §1.4）。

### 2.2 nav_safe.yaml 回退（desired 0.55 → 0.10；scan 过滤保留）

车端路径：`~/racecar_team/nav_safe.yaml`

```bash
# 回退限幅（本地已回 0.10，此处是把车端 0.55 污染改回 0.10）
sed -i 's/desired_linear_vel: 0.55/desired_linear_vel: 0.10/' ~/racecar_team/nav_safe.yaml
# scan 三处过滤是 0917 独立修复，**不回退**（.bak_scanfilter 仅存照，不用于回滚）
grep -n 'scan' ~/racecar_team/nav_safe.yaml   # 确认仍是 /scan_filtered
```

> 0.55 是 0918 事故残留（碰撞根因链的"限幅"一环），本地已在 `1c09973` 回退 0.10；
> **0.10 ≠ 已证起步**（0.10 < 电调死区 485mm/s，车推不动），"放宽限幅"仍是用户决策项。

### 2.3 gate 版本回退（v2 默认 → v1 显式，仅隔离诊断）

`safety_nav_one.sh` 从 2026-09-19 晚起**默认 `GATE_VERSION=v2`**（关闭 v1 的 B1 死锁 +
特定跳变输入下持续拒帧缺陷，`audit_gate_v1_demo.py` 已离线复现该风险机制；
**0918 当天是否触发需日志证明**，不能据"指令 0.55"认定编码器实测 0→0.55 阶跃）。
回退 v1 只需显式指定：

```bash
GATE_VERSION=v1 bash safety_nav_one.sh [地图yaml]
# 或临时 export
export GATE_VERSION=v1
```

> - **v1**（`~/racecar_team/encoder_gate.py`）：有 B1 死锁 + 特定跳变输入持续拒帧，**禁动力演示，
>   仅隔离诊断用**，不作动力演示备用。
> - **v2**（`~/racecar_team/encoder_fix/encoder_gate_v2.py`）：修复 B1/B2/B3，13/13 + 4/4 覆盖；
>   **未部署车端、未经验证，仅候选入口就绪**，动力导航须待阶段 D + 整链验证后另授权。
>   未部署时脚本会因缺文件中止（安全）。
> - **gate ≠ 里程计失效停车保护**：gate 只过滤毛刺，不监控断流新鲜度。

### 2.4 cmd_vel_safety 限幅入口（MAX_LINEAR / MAX_ANGULAR）

`safety_nav_one.sh` 从 2026-09-20 起支持**限幅可选入口**（透传 `cmd_vel_safety` 的
`max_linear`/`max_angular`，与 `GATE_VERSION` 同款环境变量模式）：

```bash
MAX_LINEAR=0.30 bash safety_nav_one.sh map1.yaml          # 现场显式放宽限幅
GATE_VERSION=v1 MAX_LINEAR=0.30 bash safety_nav_one.sh    # gate + 限幅同时选
# 不传任何参数 = 默认 0.10/0.35（保守基线，cmd_vel_safety 内部默认）
```

> - **默认保守**：不传 `MAX_LINEAR`/`MAX_ANGULAR` = `0.10`/`0.35`（任务书阶段4 上限），与历史行为一致。
> - **放宽=用户安全决策**：`0.10 < 电调死区（≈485mm/s）` 推不动车，但显式放宽须现场确认
>   电调/地面状态 + 用户明确授权，**不自动提速**。
> - **与本卡 #3 的区别**：#3 `desired_linear_vel` 是 nav2 RPP 控制器**期望速度**；
>   `MAX_LINEAR` 是安全层 `cmd_vel_safety` 的**硬限幅**（超过即截断）。两层各自独立，别混。

---

## 3. 回滚时的安全边界

1. **任何回滚前先断电调**：动力相关回滚（限幅/odom0）若车可能接电调，先断电调开关再动配置。
2. **回滚后重新起链**：改了 EKF/nav_safe/gate 后，**完整收链 → 核对配置 → 重新起链**，不热切换。
3. **精确 kill 收链**：`collect_tree` 登记 PID → 按 PID 精确 kill，收链顺序固化
   （指令源 → 零速帧 → driver → 安全层最后退），**不用 `pkill -f`**。
4. **每步只改一个因素**：回滚一次只动一个环节（odom0 / 限幅 / gate），改完对账再动下一个。
5. **不自动 push、不连接车机**：本卡是现场速查，我这边不动车端。
