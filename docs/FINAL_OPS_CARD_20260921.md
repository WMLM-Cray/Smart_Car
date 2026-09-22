# 最终操作卡（2026-09-21 收口版）

> 只含当前候选版本 6 步：显示 → 定位初始化 → 单点 → 两点 → 取消 → 收车。
> ✅=已离线验证 / 待验证=未上车。
> 车机 `cd ~/racecar_team` 后执行。断电开关始终在手边，失控第一时间物理断电。

| 步 | 做 | 命令（车机） | 状态 |
|---|---|---|---|
| **① 显示（地图+视频）** | 地图窗口 | `./display_monitor.sh start` → `logs/slam/monitor.png` | 待验证（v3 队友已收 12/12，车上功能未验） |
| | 视频窗口（新增） | `./camera_monitor.sh start` → `logs/slam/camera.png` | 待验证（新脚本未上车；先确认图像话题） |
| **② 定位初始化** | 存图后加载地图 AMCL | `./amcl_localize.sh start <地图yaml>`，再 `./amcl_initpose.sh <x> <y> <yaw_deg>` | 待验证（地图是否保存 / AMCL 短推待回传） |
| **③ 单点** | 低速单目标 | `bash safety_nav_one.sh <地图yaml>`（gate v2 默认），RViz 给 2D Pose Estimate 后点一个 2D Nav Goal | 待验证 |
| **④ 两点** | 自动 | 三前置齐后：起定位健康发布端（影子→标定→`verified:true`）+ `ros2 launch racecar_mission executor.launch.py goal_file:=<真实YAML> localization_watchdog:=2.0` + `~/start` | 待验证（三前置未齐，见下） |
| | 回退 | 未就绪时**人工确认后顺序发两个单点**（两次 2D Nav Goal，非自动多点） | 待验证 |
| **⑤ 取消** | 停止 | 单点/整链 `Ctrl+C`；任务器 `ros2 service call /mission_executor/cancel std_srvs/srv/Trigger` | 代码离线验证，实车待验 |
| **⑥ 收车** | 断电 | 确认车静止后退电调开关 → 断电 | 物理操作，待现场 |

---

## 关键提醒（不再罗列历史包）

- **相机图像源**（厂家 usb_cam 实测约定）：`/dev/video0`、320×240 yuyv、frame `camera`、话题 `/image_raw`。
  现场先 `ros2 topic info /image_raw -v` 确认真实话题+编码；若实际是 `/image_raw/compressed` 或编码不同，
  回传一次即可，`camera_monitor.py` 用 `--image-topic` 覆盖，不猜不硬改。
- **gate 版本**：`safety_nav_one.sh` 默认 `GATE_VERSION=v2`（候选，未部署车端未验证），动力导航须待阶段 D 验证；
  v1 仅隔离诊断、禁动力演示。gate 只滤毛刺，不等于里程计失效停车保护。
- **限幅**：安全层默认 max_linear=0.10 / max_angular=0.35（保守基线）。0.10 可能低于电调死区推不动车，
  放宽是用户安全决策，默认不自动提速。
- **两点红线**：健康发布端 `verified=false` 前**绝不**发 True、不恒 True、任务器 `localization_watchdog` 不设 0。
- **收车无人值守**：全程只要有动力输出，必须有人把手放在电调开关上。

## 需要队友回传（一次，最小证据）

1. 地图是否已保存 + AMCL 无动力短推是否贴墙跟随（各一个字：是/否）。
2. `ros2 topic info /image_raw -v` 的话题名 + encoding（顺便）。