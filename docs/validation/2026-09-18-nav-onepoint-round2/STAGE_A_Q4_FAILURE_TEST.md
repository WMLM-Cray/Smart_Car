# 阶段 A 第④问 —— 编码器停发失效可见性实验清单

> 状态：**待车机现场就绪执行**。本文为纯本地准备，不连接车机、不接电调、不发导航目标。
> 目的：观察"编码器停止发布后，现役 EKF / AMCL / 现有安全链的实际行为"，
> **不预设已有自动锁定停车机制**，如实记录，据此交四问④结论再决定是否进阶段 B。

---

## 0. 一句话范围

- 本实验 = 阶段 A 四问的第④问「停发后 EKF/AMCL 变化与失效可识别性」。
- 用 `encoder_tap.py` 顶替现役 encoder（独占串口、记录原始字节），`--stop-publish-after N`
  在 N 秒后停止发布 `/encoder` 但**继续读串口记录**（CSV `published=stopped`）。
- **不顺带切换**新解析器（`encoder_node_fixed`）或 `encoder_gate_v2`；gate（v1）本实验**不参与**。
- 全程**无动力**（电调断开）、**无导航运动源**（不起 planner/controller/bt_navigator）。

---

## 0.1 解析器决策：/encoder 用 A 版 legacy（明确，非 fixed、非 B 版）

> 2026-09-19 修复 `encoder_node_fixed.py` 后，`DueVelData` 恢复为 A 版（逐字节遍历、
> 符号正确），B 版（索引搜索、符号错误）已隔离为 `DueVelData_index_buggy`。本清单据此
> 明确第④问 `/encoder` 的发布解析器：

| 决策项 | 选择 | 理由 |
|---|---|---|
| `/encoder` 发布 | **A 版 legacy**（修复后 `DueVelData`，符号正确） | 第④问的观察对象是「停发」这个**失效行为**，不是解析器正确性；用 A 版 = 厂家原版语义顶替，保持"一次只改一个因素"（唯一改动 = 停发） |
| 不用 fixed | **否** | fixed 是新解析器（CRC 提取器），切换它会引入第二个变量，违背隔离原则 |
| 不用 B 版 | **否（已隔离）** | B 版符号错误，后退/负速度会发 +669 正大数，污染 EKF 输入，破坏"静止 + 停发"的干净观察 |

**两个"不得"（用户红线，逐条落实）：**

1. **不得默认 legacy 已安全**：A 版 legacy 仍是**无 CRC 校验的全局状态机**
   （`Bytenum_vel` 跨调用保留），存在错位污染缺陷。若第④问观察到 EKF 速度/位姿异常跳变，
   **须先区分是"停发"还是"A 版状态机错位"所致**——以 CSV 的 `frame_valid`/`note` 列
   辨别（`frame_valid=bad` 或 `note` 含 `legacy_TypeError` 即 legacy 侧异常，非停发）。
2. **不得为避开 B 静默切成 fixed**：本清单**明确声明**用的是 A 版 legacy，不是 fixed；
   若现场发现 A 版状态机错位干扰观察，**须停下来汇报**，由用户决定是否临时改用 fixed，
   **不得自行切换**。

> 附：本次修复 + A/B 隔离 + 离线重放验证见
> `D:\Claude\projects\hwcart_encoder_stageA\vendor_review_20260919\verify_parser_fix_replay.py`
> （343/343 后退负帧 A 版符号全对、B 版全错），测试 46/46 PASS。

---

## 1. 前置检查（现场，起链前逐条确认）

1. **电调断开、无导航运动源**
   - 物理确认电调断电；舵机可视需要通电（架空）。
   - `ros2 node list` 确认无 `planner_server / controller_server / bt_navigator / waypoint_*`；
     确认无任何进程在发布 `/cmd_vel`（无导航目标）。本实验 `/cmd_vel` 应为**无流**状态。
2. **核对车端现役配置（勿当本地已同步）**
   - 读车端 `~/racecar/install/racecar/share/racecar/config/ekf.yaml`：
     确认 `odom0` 当前是 `encoder` 还是 `encoder_gate`、`imu0=IMU_data`、`sensor_timeout`。
   - 与 `stage0_evidence/md5_car_0918.txt` 指纹比对；若车端已变，先记录差异**再继续**，不覆盖。
3. **唯一串口读取者**
   - `ps aux` 确认无 `encoder_node / encoder_gate / encoder_tap` 等进程在读 `/dev/encoder`。
   - `fuser /dev/encoder` 为空（本 tap 起来前）。
4. **记录本次唯一改动环节 + 基线回滚入口**
   - 若车端现役 `odom0=encoder_gate`，本实验 EKF 改用 `.bak_pregate0918`（`odom0=encoder`）起链，
     **记录本次唯一改动 = EKF odom0 回退 `/encoder`**（gate 不参与）。
   - 用**路径传参**指向 bak 配置（`cp` 备份、不 `mv` 不覆盖现役文件）；收链后确认现役 `ekf.yaml`
     仍在原地未动。不修改任何冻结工具源码。

---

## 2. 链路图（第④问目标链路）

```
[串口 /dev/encoder]  (唯一读取者)
        │
        ▼
encoder_tap.py ── /encoder (A 版 legacy 顶替, 厂家语义·符号正确) ──► EKF(ekf_filter_node)
        │                    odom0=encoder (用 .bak_pregate0918)
        │                    融合 encoder vx/vy/vyaw + IMU yaw
        │                    sensor_timeout=2.0s, 20Hz, publish_tf=true
        │                                        │
        └─ /encoder_fixed_obs (旁路, 不供 EKF)     ▼
                                     /odom_combined (Odometry + TF odom_combined→base_footprint)
                                              │
                                              ▼
[雷达 lslidar → /scan] ─────────────────► AMCL(nav2_amcl)
                                              │ odom_frame=odom_combined, scan_topic=scan
                                              │ robot_model=DifferentialMotionModel
                                              ▼
                                          /amcl_pose
```

- **不在本链路**：`encoder_gate`(v1/v2)、`encoder_node_fixed`(新解析器)、`slam_toolbox`、
  `planner_server / controller_server / bt_navigator / costmap`。
- **现有安全链 `cmd_vel_safety`**：本实验的**观察对象**（只监控 `/cmd_vel` 新鲜度，不监控里程计）。
  建议起一个（`publish_start_zero:=false`，不接任何指令源），以回答
  「现有安全链是否察觉编码器失效」。若不起，则如实记录「安全层未启动」。

---

## 3. 观察顺序（时间基准 = tap 的 `time.monotonic()`）

> 车机系统钟慢约 5 个月，只当相对刻度；跨进程对账一律以 tap CSV 的 `monotonic_s` 为准，
> ROS 侧 `ros2 topic hz/echo` 记录相对时刻即可。

### 3.0 起链（顺序固化，登记 PID，禁止 `pkill -f`）
1. 雷达（供 `/scan`；按现役 driver 层方式起）。
2. EKF —— 用 `.bak_pregate0918`（`odom0=encoder`）。
3. AMCL —— `localization_realcar.launch.py`（`map_server + amcl + lifecycle_manager`，autostart）。
4. `encoder_tap.py`（`--publish legacy` = **A 版 legacy**，修复后符号正确；见 §0.1）：
   ```
   python3 encoder_tap.py --out-dir <记录目录> --duration <T_total> \
        --stop-publish-after <T_stop> --publish legacy
   ```
5. `cmd_vel_safety.py`（观察对象，可选项，`publish_start_zero:=false`）。
6. 起链后 `collect_tree` 登记各节点 PID，写入实验记录；**收链一律按 PID 精确 kill**。

### 3.1 阶段 1 —— 正常输入基线（T0 ~ T_stop，建议 30s）
- tap 正常发布 `/encoder`（legacy）；EKF 正常融合；AMCL 静止。
- 采集基线：`/odom_combined` 位姿/速度/协方差、`/amcl_pose`、`/encoder` 消息频率、诊断。
- 记录静止下 odom 是否漂移（速度应 ≈0）—— 作为停发前后的对照。

### 3.2 阶段 2 —— 停发观察（T_stop ~ T_stop+T_obs，建议 T_obs = 15s）
- tap 在 T_stop 自动进入 `publishing=False`，**停止发布 `/encoder`、继续读串口记录**。
- 重点观察窗口 = 停发后 **0~2s（EKF `sensor_timeout=2.0s` 前后）**：
  - EKF 是否仍以 20Hz 发布 `/odom_combined`？（靠 IMU yaw 维持预测 vs 停止发布）
  - `/odom_combined` 速度/协方差是否变化？（静止下速度 ≈0，重点看**协方差是否膨胀**、**是否停止发布**）
  - AMCL 是否继续更新？（静止无 scan 变化通常不触发；记录实际）
  - `cmd_vel_safety` 是否输出任何告警/零速？（预期：无——因为它只看 `/cmd_vel`）
- 全程 `ros2 topic hz /encoder /odom_combined /amcl_pose` 采样记录。

### 3.3 阶段 3 —— 恢复发布
- `--stop-publish-after` 是**单向**的；恢复 = Ctrl+C 停旧 tap（flush CSV、释放串口），
  再起一个**不带** `--stop-publish-after` 的 tap 重新发布 `/encoder`。
- 观察 EKF 重新收到 `/encoder` 后是否恢复融合（速度/协方差回落）。

### 3.4 收链（顺序固化，按 PID）
1. 停 `cmd_vel_safety` → 2. 停 tap（Ctrl+C，flush）→ 3. 停 AMCL（lifecycle+amcl+map_server）
   → 4. 停 EKF → 5. 停雷达。**全程按登记 PID 精确 kill，不用 `pkill -f`。**

---

## 4. 记录指标（每阶段分别记录，落表）

| 层面 | 指标 | 来源 |
|---|---|---|
| 原始输入 | `monotonic_s`、`resp_v/resp_d` 原始字节 hex、`frame_valid`(valid/exc/bad/empty)、`legacy_vel`、`fixed_vel`、`published`(legacy/stopped)、`note` | tap CSV |
| 过滤输出新鲜度 | `/encoder` 消息时间戳间隔（停发前实测频率；停发后无） | `ros2 topic hz /encoder` |
| EKF | 位姿(x,y,yaw)、速度(vx,vy)、协方差对角线(重点 x/y/yaw)；区分「持续预测发布」vs「新测量更新」 | `ros2 topic echo /odom_combined` |
| AMCL | 位姿、协方差、是否 resample（`update_min_d/a` 未达则不更新） | `ros2 topic echo /amcl_pose` + 诊断 |
| 现有安全链 | 是否输出告警/零速（预期无，因无 `/cmd_vel` 流且不监控里程计） | cmd_vel_safety 日志 |
| 诊断 | 各阶段 `topic hz` / `topic echo` 采样、`ros2 doctor` 可选 | 命令输出 |

---

## 5. 实验边界（如实记录，不越级宣称）

- 本实验**只观察现有系统在编码器停发后的行为**，不预设已有自动锁定/停车机制。
- **静止 + 无运动指令源**下，速度恒 ≈0，停发前后 EKF 输出差异有限；
  本实验能确认的是「发布是否继续、协方差是否变化、下游能否识别失效」，
  **不能据此宣布验证了「非零→故障→停车」**——该验证须另走隔离输出或虚拟串口（动力获准后）。
- `cmd_vel_safety` 不监控里程计 = 已知缺口；本实验记录其「无察觉」即为结论之一，
  不擅自给安全层加保护。
- 停发后 EKF 是否继续发布、AMCL 是否继续更新，均**如实记录，不预设结果**。

---

## 6. 退出与恢复

1. **异常终止条件（任一触发即停，不二分猜因）**
   - 出现 `/car_cmd_vel` 非零（意外动力指令）
   - EKF 输出速度/位姿异常跳变
   - 串口连续出错 / `frame_valid` 大量 `bad`
   - 车异常 / 用户喊停
2. **进程清理**：按登记 PID 精确 kill，顺序 3.4 固化，**不用 `pkill -f`**。
3. **配置恢复**：若起链用了 `.bak_pregate0918`，收链后确认现役 `ekf.yaml`（`odom0=encoder_gate`）
   未被覆盖（cp 备份、不 mv）；恢复只需确认原文件仍在。
4. **证据保存**（拉回本机）：
   - tap CSV → `D:\Claude\projects\hwcart_encoder_stageA\tap_csv\`
   - ros2 日志 / topic 采样 / 诊断 → 同目录 `stageA_q4/` 子目录
   - 实验记录：车端配置 md5、起链命令、各阶段时间戳、PID 清单、观察到的行为逐条。

---

## 7. 交付后动作

- 交四问④结论（含原始数据 + 上表 + 阶段2观察到的实际行为）。
- 由用户定夺是否进入阶段 B（新解析器切换）；本清单交付后**停止**，不自动执行后续阶段。
