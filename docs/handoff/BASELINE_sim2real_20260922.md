> 历史参考：本文件保留写作时的记录；当前参数和状态见 [项目文档入口](../README.md)。

# 基线说明 — baseline/sim2real-20260922（开发与复现基线）

> 本分支是本机开发线的**候选基线发布**，目的是让队友通过一个明确的远端分支 + 完整 SHA
> 取得本机已完成的代码、文档与可复现结果。
>
> **基线性质 = 开发与复现基线，不代表实车动力导航已验收。**
> 本基线**不授权部署、不授权启动运动/导航链、不授权绕过安全链**（见 §6）。

## 0. 定位与范围

- 分支：`baseline/sim2real-20260922`
- 基于：`demo-20260920` @ `1035283`（本机开发线 HEAD，已包含 `origin/integration/trackA-sdf` 全量历史）
- 相对 `origin/integration/trackA-sdf`（= `c6e630a`）新增 15 个开发提交 + 本基线 4 个新增提交（§7）。

## 1. 本分支包含的成果与入口

### 1.1 二维实车约束敏感性分析（sim2real 首轮交付）
- 一页摘要：`docs/sim2real/SUMMARY_20260921.md`
- 参数对照表：`docs/sim2real/PARAM_COMPARISON_20260921.md`
- 队友交底：`docs/handoff/SCENARIOS_HANDOFF_20260921.md`
- 结果图 + 机器可读指标：`docs/sim2real/out/`（`scenario_s1/s2/s3.png`、`metrics.json`、`realcar_push_compare.png`）
- 复现脚本：`tools/sim2real/run_compare.py`、`tools/sim2real/plot_realcar_push.py`
- 推车取证原始数据：`docs/sim2real/data/tap_push_20260918.csv`（Modbus RTU tap，10344 行，仓库内）

### 1.2 手机只读前端（web）
- 入口：`web/README.md`（启动/演示/验证清单）、`web/INTERFACE_MAP.md`（接口对照 + 未接通项）
- 前端：`web/index.html` + `web/css/app.css` + `web/js/app.js` + `web/js/adapter.js`（无构建）
- 只读本地服务：`web/serve.py`（仅 GET，无控制端点）
- 车机拉图：`web/pull.sh`（地图 `monitor.png` + 相机 `camera.png` → `web/media/`，运行时创建）
- 配置与截图：`web/config.json`、`web/screenshots/`

### 1.3 定位健康发布端（两点任务器门禁）
- 入口文档：`tools/realcar/LOCALIZATION_HEALTH.md`
- 状态机核心（纯 Python，离线可测）：`tools/realcar/loc_health_core.py`
- ROS 节点：`tools/realcar/localization_health.py`
- 阈值模板（`verified:false` 未标定）：`tools/realcar/localization_health.yaml`
- 离线单测：`tools/realcar/test_loc_health_core.py`
- ROS 隔离集成测试：`tools/realcar/loc_health_integration_test.py`
- 接口定义更新：`tools/mission/LOCALIZATION_HEALTH_INTERFACE.md`

### 1.4 相机只读渲染 + gate/safety 测试 + 交接文档
- 相机只读渲染：`tools/realcar/camera_monitor.py` + `camera_monitor.sh`
- gate v2 ROS 层测试（单进程壳）：`tools/realcar/encoder_fix/test_encoder_gate_v2_ros.py`
- cmd_vel_safety 隔离测试：`tools/realcar/test_cmd_vel_safety.py`
- 交接/操作卡：`docs/reference/AVOIDANCE_DEMO_20260921.md`、`docs/reference/CAMERA_LIVEFEED_PLAN_20260921.md`、
  `docs/reference/FINAL_OPS_CARD_20260921.md`、`docs/reference/TWOPOINT_ENTRY_CHECK_20260921.md`

## 2. 已验证项目与验证环境

| 项目 | 验证方式 | 环境 | 结果 |
|---|---|---|---|
| 二维三场景对照 | `python tools/sim2real/run_compare.py` | Windows 11 · Python 3.11.9 · numpy 2.4.6 | 复现成功（metrics.json + 3 PNG 重新生成） |
| 实车推车对照图 | `python tools/sim2real/plot_realcar_push.py` | 同上 + matplotlib(Agg) | 复现成功（realcar_push_compare.png 重新生成） |
| 定位健康状态机 | `python -m pytest tools/realcar/test_loc_health_core.py -q` | 同上 · pytest 9.1.1 | **20 passed**（离线，无 ROS，本机已跑） |
| 定位健康 ROS 集成 | `loc_health_integration_test.py` | WSL ubuntu2204-ros2 | 10/10（此前已验证，本机无 ROS 未重跑） |
| cmd_vel_safety 隔离 | `test_cmd_vel_safety.py` | WSL ubuntu2204-ros2 | 4/4 场景（A/B/C/D，此前已验证） |
| gate v2 ROS 壳 | `test_encoder_gate_v2_ros.py` | WSL ubuntu2204-ros2 | 7/7 单进程壳（此前已验证） |

> ROS 依赖的三项（集成/隔离/壳）需 ROS2 环境，本基线 Windows 侧不重跑；
> 其文档与历史验证结果随提交一并给出，作为"已离线/隔离域验证"的证据而非本机本次复验。

## 3. 未验证项目与已知失败项

### 3.1 已知失败 / 未通过（如实）
- **S2-B 朝向通过余量很小**：0.349 rad vs 阈值 0.35 rad，余量仅 0.001 rad，恰在阈值内侧；
  S2-A（原模型）朝向 0.468 ✗。
- **S3 完整到点判据未通过**：S3-A/B 朝向分别为 0.894 / 0.666 ✗，因参考弧末端切线 −y 与终点
  直行 +x 存在 90° 不连续（参考路径几何特性，非车辆模型能力；要朝对齐需加调头段，本轮未含）。
- **S3 首版反应式控制器 A/B 均撞障碍**（min_clearance=−0.14，collided=True），已改用
  "给定参考路径 Pure Pursuit 跟踪"重做（方法修正记录见 `SUMMARY_20260921.md`）。
- **C 变体（假设 0.48 m/s 死区）三场景均未起步超时**：这是"假设死区模型"的结果，**非实车验证**，
  不据此建议提高限速。

### 3.2 未验证 / 未接通（诚实声明）
- **前端真实地图/相机手机链路未验收**：`web/INTERFACE_MAP.md` 中"已接通"仅限只读接口
  （`GET /media/*.png`、`/api/status`、`/api/capabilities`）；真实地图/相机画面是否出帧、
  手机能否访问热点下设备、后端监听地址，**均未现场验收，不得写已接通**。导航/停止/建图/多点/
  定位健康的 HTTP 接口均"未接通"（仅契约）。
- **gate 单进程 ROS 壳测试 ≠ 真实车端跨进程验证**：`test_encoder_gate_v2_ros.py` 是单进程 ROS 壳
  （import 真实 gate_check + 复现 main 壳），不等同于车端跨进程部署验证；现场须在隔离 ROS 域
  单独跑真实 `encoder_gate_v2.py` 才作数。
- **定位健康发布端未部署、未标定**：`verified:false` 不发 True；待现场影子运行 + 标定阈值 +
  接两点任务器验证。
- **权重存在、训练配置可追溯、实际上车使用是不同状态**：低速策略权重/训练配置在 `origin/main`
  训练线（见 §5），本基线不携带训练权重结论；**权重存在 ≠ 已上车使用**，实际部署须另经
  用户授权与现场验证。

## 4. 如何复现二维结果 / 如何启动只读前端

### 4.1 复现二维三场景对照
```bash
python tools/sim2real/run_compare.py        # 输出 docs/sim2real/out/{metrics.json, scenario_s1/s2/s3.png}
```

### 4.2 复现实车推车对照图
```bash
python tools/sim2real/plot_realcar_push.py  # 输出 docs/sim2real/out/realcar_push_compare.png
# 数据源 = docs/sim2real/data/tap_push_20260918.csv（已随本基线提交，仓库相对路径）
```

### 4.3 启动只读前端（不接控制）
见 `web/README.md`「快速启动」：车机侧 `display_monitor.sh start` / `camera_monitor.sh start`
（只读渲染）→ 本机 `web/pull.sh <车机IP>` 拉图 → `python web/serve.py` → 浏览器/手机打开
`http://<IP>:8123/`。红线：只读，不向任何控制话题发指令；`serve.py` 只有 GET；无软件急停按钮，
现场始终依靠物理断电。

### 4.4 定位健康发布端离线单测
```bash
python -m pytest tools/realcar/test_loc_health_core.py -q   # 20 passed
```
ROS 集成/隔离测试需在 WSL/车机 `source` ROS2 后执行（见各自文件头用法）。

## 5. 与远端 main 训练线的关系

- 共同祖先：`721afcc`（阶段0基础工程）。
- 本基线＝本机开发线：领先 `origin/main` **92 提交**、落后 **36 提交**，**未合并 main**。
- `origin/main`（= `e4da8af`，2026-09-22 fetch）＝队友训练线：teleop / sim 策略 / 低速策略训练 /
  mapping / 相机与路由 systemd 服务等 36 个独有提交。
- 两条线**平行、未合并**：本基线是本机"开发与复现"成果的独立落点，**不含 main 训练线的未合并改动**；
  后续合流由用户另行决策，本基线不做 merge。

## 6. 本基线不授权

- 不授权部署到车机、不授权启动运动/导航链、不授权绕过安全链。
- 本基线是代码/文档/可复现结果的候选发布，供队友只读取得与本地核对；
  实车任何上电/动力操作由队友现场会话按既有协作边界执行。
- 不创建"实车正式版"标签；不合并 main；不强推；不改写既有历史。

## 7. 本基线新增提交

| 顺序 | 提交主题 | 内容 |
|---|---|---|
| 1 | `feat(sim2real): 二维实车约束敏感性分析` | `docs/sim2real/` + `tools/sim2real/` + `docs/sim2real/data/tap_push_20260918.csv` |
| 2 | `feat(web): 手机只读前端` | `web/` |
| 3 | `feat(realcar): 定位健康发布端 + 相机监视 + gate/safety 测试 + 交接文档` | `tools/realcar/*` + `tools/mission/LOCALIZATION_HEALTH_INTERFACE.md` + `docs/*_20260921.md` |
| 4 | `docs(baseline): sim2real-20260922 开发与复现基线说明` | 本文件 |
