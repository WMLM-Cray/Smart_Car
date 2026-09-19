# 车端 vs 本地 配置差分（2026-09-20，v1）

> 目的：把「车端现役配置」与「本仓库待交付配置」**逐文件**对账，让队友现场能一眼看清
> 「车端现在是什么、本地要交付什么、差在哪、怎么回滚」，交付前先消除"以为已生效"的错觉。
>
> 一句话状态：**本地已回退 0.10、gate 已切 v2 默认、新增限幅可选入口，全部未部署车端；
> 车端 driver / nav_safe / ekf 当前真值待逐文件取证。**

---

## 0. 读法（三个"值"严格区分，不混用）

| 列 | 含义 | 可信度 |
|---|---|---|
| **本地值（仓库）** | 本仓库 `tools/realcar/` + `work/vendor_src/` 的当前内容 | ✅ 本机已确认 |
| **车端历史记录** | 0916/0917/0918 的历史部署记录（含另一会话的现场改动） | ⚠️ 只作参照，**待取证** |
| **待取证** | 车端 `~/racecar_team/`、`~/racecar/install/` 的**当前真实值** | ❓ 队友现场逐文件核对后填入 |

> 红线（长期遵守）：**不连接车机、不自动 push、动力需用户/现场明确授权**。
> 车端已被另一会话改过 driver、回退、重编译，**不能写"车端仍旧配置"**，逐文件核对内容+哈希+差异后再动。

---

## 1. 差分总表

| # | 配置项 | 本地值（仓库，已确认） | 车端历史记录（待取证） | 差异 | 回滚基线 |
|---|---|---|---|---|---|
| 1 | driver 公式偏置 | `×100 + 1500`（vendor_src 出厂） | `×100 + 1540`（0916 改） | 1500 ↔ 1540 | 待取证 |
| 2 | driver 订阅话题 | node=`/car_cmd_vel`；node_one=`/cmd_vel`（两版） | 现役 node（待取证） | 待核对 | 待取证 |
| 3 | ekf.yaml odom0 切源 | `odom0=encoder`（vendor_src 出厂） | `odom0=encoder_gate`（0918 改） | encoder ↔ encoder_gate | `.bak_pregate0918` |
| 4 | nav_safe.yaml desired_linear_vel | `0.10`（本地已回退） | `0.55`（0918 事故试验残留） | 0.10 ↔ 0.55 | 本地已回 0.10 |
| 5 | nav_safe.yaml scan 三处话题 | `/scan`（仓库从未含 filtered） | `/scan_filtered`（0917 现场改） | /scan ↔ /scan_filtered | `.bak_scanfilter` |
| 6 | safety_nav_one.sh gate 默认 | `v2`（本地） | `v1`（车端现役） | v2 ↔ v1 | `GATE_VERSION=v1` |
| 7 | cmd_vel_safety 限幅 | 默认 `0.10/0.35`（**本轮新增可选入口**） | `0.10`（启动未传参） | 本轮加入口 | 不传参=0.10 |
| 8 | encoder_gate | v1（`encoder_gate.py`）+ v2（`encoder_fix/`） | v1 现役 | v2 未部署 | `GATE_VERSION=v1` |
| 9 | ±60µs 死区补偿 | **无**（本地无此改动） | 车端队友本地改动（未成文） | 待取证 | 待取证 |

---

## 2. 逐项明细

### 2.1 driver 公式偏置（1500 ↔ 1540）

- **本地**：`work/vendor_src/racecar/src/racecar_driver/src/racecar_driver_node.cpp:45`
  `linear_x = linear.x*100 + 1500`（出厂 tar 版，未编译）。
- **车端历史**：0916 12:23 现场改 1500→1540 并 `colcon build`（`REALCAR_CONTROL_AUDIT §3.3`）。
- **差异影响**：偏置错位会让"限幅 0.10 还是 0.55、正转还是倒车"的判读失真；1540=0916 实测标定电调中位。
- **待取证**：车上 src 第 45 行 + install 二进制的 `strings` 实际偏置（见 `docs/DRIVER_SOURCE_MANIFEST.md` 第三节回传表）。
- **完整对照**：driver 7 文件 sha256 指纹见 `docs/DRIVER_SOURCE_MANIFEST.md`，此处不重复。

### 2.2 driver 订阅话题（node ↔ node_one）

- **本地**：`racecar_driver_node.cpp` 订 `/car_cmd_vel`（经安全层）；`racecar_driver_node_one.cpp` 订 `/cmd_vel`（直连，不经安全层）。
- **待取证**：车端 `ps -ww -eo pid,args` 看现役是 node 还是 node_one（决定 nav 链走安全层还是直连）。

### 2.3 ekf.yaml odom0 切源（encoder ↔ encoder_gate）

- **本地**：`work/vendor_src/racecar/src/racecar/config/ekf.yaml:30` `odom0: encoder`（出厂）。
- **车端历史**：0918 改 `odom0=encoder_gate`（让 EKF 里程计供血走闸门过滤，防编码器毛刺→假 odom）。
- **回滚基线**：`.bak_pregate0918`（`odom0=encoder`，= 出厂值）。
- **注意**：停 gate 后 EKF 不会自动读 encoder（odom0 指向 gate 话题，非自动回退）；阶段 A 第④问失效实验临时用 `odom0=encoder` 应走路径传参指 `.bak`，**不覆盖现役**。

### 2.4 nav_safe.yaml desired_linear_vel（0.10 ↔ 0.55）

- **本地**：`tools/realcar/nav_safe.yaml:110` `desired_linear_vel: 0.10`（`c04c890` 已回退保守基线）。
- **车端历史**：0918 事故试验曾临时 `0.55`（过电调死区 485mm/s，`318b3d8`），车端可能仍是 0.55。
- **差异影响**：0.55 是碰撞根因链的"限幅"一环；0.10 保守但**低于电调死区、推不动车**。
- **回滚**：`sed -i 's/desired_linear_vel: 0.55/desired_linear_vel: 0.10/' ~/racecar_team/nav_safe.yaml`
- **红线**：0.10 ≠ 已证可稳定起步；"放宽限幅/提升速度"是**用户安全决策**，须低速动力标定后另行确认，不得自行提速。

### 2.5 nav_safe.yaml scan 三处话题（/scan ↔ /scan_filtered）

- **本地**：`tools/realcar/nav_safe.yaml` 三处均为 `/scan`（41 `scan_topic`、181 local voxel、217 global obstacle）。
  **仓库历史从未提交过 `/scan_filtered`**（`6bb97d8`/`318b3d8`/`c04c890` 里都是 `/scan`）。
- **车端历史**：0917 现场把三处 sed 成 `/scan_filtered`（过滤雷达后侧天线 ~0.15m 等距双点回波），备份 `.bak_scanfilter`。
- **⚠️ 关键差异**：`scan_filter.py` 源码只在 `docs/validation/2026-09-17-nav-onepoint/tools/scan_filter.py`（归档），
  **不在 `tools/realcar/` 主目录，也不在 `safety_nav_one.sh` 启动链里**。
  若队友现场用**本地 `nav_safe.yaml`（scan=/scan）直接覆盖车端**，会丢掉 scan_filtered 修复，AMCL/costmap 重新吃到天线回波。
- **待取证**：车端 `~/racecar_team/nav_safe.yaml` 三处 scan 话题当前实际值 + `scan_filter.py` 是否还在车端运行链。
- **决策留白**：scan 过滤是 0917 独立修复，是否仍需要、`scan_filter.py` 是否应纳入 `tools/realcar/` 主链，
  **待现场取证后由用户裁决**，不自动改、不自动纳入。

### 2.6 safety_nav_one.sh gate 默认（v2 ↔ v1）

- **本地**：`tools/realcar/safety_nav_one.sh` 默认 `GATE_VERSION=v2`（`e04648c`）。
- **车端历史**：现役 `encoder_gate.py`（v1）。
- **差异影响**：v1 有 B1 重锁死锁 + 特定跳变输入下持续拒帧（`audit_gate_v1_demo.py` 已离线复现），禁动力演示；
  v2 修复 B1/B2/B3（13/13 + 4/4），**未部署车端、未验证，仅候选入口就绪**。
- **回滚**：`GATE_VERSION=v1 bash safety_nav_one.sh`（仅隔离诊断，不作动力演示备用）。

### 2.7 cmd_vel_safety 限幅参数（本轮新增可选入口）

- **本地**：`tools/realcar/cmd_vel_safety.py` `declare_parameter("max_linear", 0.10)` / `max_angular 0.35`。
- **本轮改动**：`safety_nav_one.sh` 新增 `MAX_LINEAR` / `MAX_ANGULAR` 环境变量，透传给 `cmd_vel_safety.py`（默认不传=0.10/0.35，保守基线不变）。
- **红线**：显式 `MAX_LINEAR=0.55` 等**放宽限幅是用户安全决策**，须现场确认电调/地面状态 + 用户明确授权，默认一律保守。

### 2.8 encoder_gate v1 ↔ v2

- **本地**：`tools/realcar/encoder_gate.py`（v1）+ `tools/realcar/encoder_fix/encoder_gate_v2.py`（v2，含 `test_gate.py` 13/13 + `test_gate_v2_contract.py` 4/4）。
- **车端历史**：v1 现役。
- **待取证**：车端是否已有 v2 文件、`GATE_VERSION` 实际生效版本。

### 2.9 ±60µs 死区补偿（队友车端改动，未成文）

- **本地**：**无此改动**（本仓库 driver 仍是 vendor_src 出厂 +1500 / 0916 记录 +1540）。
- **车端历史**：队友回报车端 driver 加 ±60µs 死区补偿（中位 1540/起步 1600），**尚未形成已确认的 Git 提交**。
- **待取证**：见 `FACTS_CORRECTION_20260919.md` 第二节 5 项（修改前备份 diff / 编译路径哈希 / 进程 / 正零负 PWM 日志 / 补偿来源）。
- **口径并存**：±60µs（队友，中位 1540/起步 1600）与 318b3d8"485mm/s 死区 ≈1588µs"**两口径并存，待字节/标定裁决，不合并不覆盖**。

---

## 3. 本轮参数可选入口（`safety_nav_one.sh`）

| 环境变量 | 默认 | 作用 | 备注 |
|---|---|---|---|
| `GATE_VERSION` | `v2` | 选 gate v1/v2 | v1 仅隔离诊断，禁动力演示 |
| `MAX_LINEAR` | `0.10` | 透传 `cmd_vel_safety` `max_linear` | 放宽=用户安全决策 |
| `MAX_ANGULAR` | `0.35` | 透传 `cmd_vel_safety` `max_angular` | 同上 |

用法示例：

```bash
# 保守基线（默认，什么都不用传）
bash safety_nav_one.sh

# 现场显式选 gate 版本 + 限幅（限幅放宽须用户授权）
MAX_LINEAR=0.30 bash safety_nav_one.sh map1.yaml
GATE_VERSION=v1 MAX_LINEAR=0.30 bash safety_nav_one.sh
```

---

## 4. 队友取证清单（回传，全部只读，不新增运动）

1. driver 公式偏置：车上 `racecar_driver_node.cpp` 第 45 行 + install 二进制 `strings`（§2.1）。
2. driver 订阅话题：`ps -ww -eo pid,args | grep "[r]acecar_driver"` 现役 node / node_one（§2.2）。
3. ekf.yaml：`grep odom0 ~/racecar/install/racecar/share/racecar/config/ekf.yaml`（§2.3）。
4. nav_safe.yaml 限幅：`grep desired_linear_vel ~/racecar_team/nav_safe.yaml`（§2.4）。
5. nav_safe.yaml scan 三处 + `scan_filter.py` 是否在运行链（§2.5，**最高优先，本地与车端已确认不一致**）。
6. safety_nav_one.sh gate 默认 + encoder_fix 是否已部署（§2.6/2.8）。
7. ±60µs 死区补偿 5 项（§2.9，见 `FACTS_CORRECTION_20260919.md`）。

> 取证任何一项与实际不符就停下回报，不硬猜、不继续部署。

---

## 5. 部署顺序建议（先取证 → 再部署 → 后验证）

1. **先取证**：按第四节逐文件核对车端当前真值，填入本表"待取证"。
2. **再部署**：只部署已确认的差异项（如 gate v2、限幅入口脚本），每步只改一个因素。
3. **后验证**：无动力只读验证 → 阶段 D gate 四项 → 整链验证，动力导航另授权。

---

相关：[[DRIVER_SOURCE_MANIFEST]]（driver 7 文件指纹）· [[PWM_HISTORY_LOG_REQUEST]]（正零负 PWM 日志）·
[[ROLLBACK_QUICKREF]]（回滚速查卡）· [[FACTS_CORRECTION_20260919]]（事实更正单）
