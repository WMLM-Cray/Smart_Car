# 接口对照表（功能 ↔ 实际接口 ↔ 是否接通 ↔ 缺口）

> 「已接通」= 前端当前真实可用的只读接口；「未接通」= 仅文档化契约，本轮未实现（涉及控制的一律不擅自实现）。

## 一、已接通（只读）

| 功能 | 实际接口 | 输入 → 输出 | 状态 |
|---|---|---|---|
| 地图监视 | `GET /media/map.png` | — → 车机 `map_monitor.py` 渲染的 PNG(map+scan+轨迹+可选路径) | ✅ 已接通（由 `pull.sh`/`fetch_monitor.sh` 从车机 `~/racecar_team/logs/slam/monitor.png` 拉取落地） |
| 相机画面 | `GET /media/camera.png` | — → 车机 `camera_monitor.py` 渲染的 PNG | ✅ 已接通（取决于相机是否出帧） |
| 状态快照 | `GET /api/status` | — → `{serverTime, mode, media:{map,camera:{exists,mtime,size,genTime}}}` | ✅ 已接通 |
| 能力声明 | `GET /api/capabilities` | — → `{mapView,camera,navigation,multipoint,slam,softwareStop,mapLayers,mapMetadata,videoKind}` | ✅ 已接通 |
| 连接设置 | 前端本地 | 后端地址输入/连接/断开/重试/最近地址 → `localStorage` | ✅ 已接通（前端）；不存 SSH 密码、不执行 SSH |
| 前端静态 | `GET /`、`css/*`、`js/*` | — → HTML/CSS/JS | ✅ 已接通 |

**新鲜度口径（如实）：** PNG 文件 `mtime` ≈ 本机拉取时间，**不等于**画面真实生成时间；
`/api/status` 里 `genTime` 恒为 `null`（PNG 无机器可读生成时刻）。真实新鲜度以地图/相机
画面左上角车机画的 `scan/tf/image: Ns ago [STALE]` 戳为准 —— 前端如实标注「源图生成时间未知」，
不冒充可以程序化判断。

## 二、未接通（仅契约，控制类不实现）

| 功能 | 应有接口(最小后端需求) | 缺口 | 前端当前表现 |
|---|---|---|---|
| 地图元数据 | `GET /api/mapmeta` → `{resolution, origin:{x,y,yaw}, width, height, frame_id}` | `/map` 消息里的元数据未通过 HTTP 暴露，PNG 是纯像素 | 地图页只可查看；「点图导航」禁用并说明 |
| 单点导航 | `POST /api/nav/goal` `{x,y,yaw}`；`GET /api/nav/state` | 无 HTTP 控制后端，无地图坐标换算 | 任务页「单点导航」卡禁用，标注未接入 |
| 多点导航 | `POST /api/mission` `[{x,y,yaw}...]`；`GET /api/mission/state` | 复用任务器后端的 HTTP 通路不存在 | 任务页「多点导航」卡禁用 |
| 建图/存图/载图 | `POST /api/slam/{start,stop,save,load}`；`GET /api/slam/state` | 无后端；建图 vs 定位须后端确认，页面不擅自同时启 SLAM+AMCL | 「建图与地图管理」卡禁用 |
| 软件停止 | `POST /api/stop`（仅绑定已核实后端） | 无已核实停止接口 | 不提供红色急停按钮；正文说明「软件停止未接入」 |
| 定位健康 | `GET /api/localization` | `localization_health.py` 只发 ROS 话题，无 HTTP | 总览「定位」=未接入 |
| 任务状态 | `GET /api/task` | 任务器无 HTTP 通路 | 总览/任务页显示「无（任务器未接入）」 |
| 雷达新鲜度 | —（需后端送 `/scan` 时间戳） | 前端无程序化判据 | 总览「雷达」=未知（画面戳目视判定） |

## 三、最小后端需求（未来接控制时，按此实现，且必须经用户批准）

接导航/停止前，后端须提供并把以下端点写进 `js/adapter.js` 的适配器（**不擅自实现**）：

```
GET  /api/capabilities   → 能力(控制类才置 true)
GET  /api/status         → 状态 + 各源新鲜度时间戳(权威, 非文件 mtime)
GET  /api/mapmeta        → 地图分辨率/原点/yaw/frame(点图导航前提)
POST /api/nav/goal       → 提交单点目标{x,y,yaw}     → 返回 {taskId, accepted}
POST /api/nav/cancel     → 取消当前目标
GET  /api/nav/state      → {state: pending|accepted|executing|succeeded|failed|cancelling|cancelled|unconfirmed}
POST /api/mission        → 多点 [{x,y,yaw}]
GET  /api/mission/state  → 逐点状态 + 总进度
POST /api/slam/{start,stop,save,load}
GET  /api/slam/state     → {mode: mapping|localizing|idle}
POST /api/stop           → 软件停止(仅绑定已核实接口; 返回状态分「请求已发送/后端已确认/车辆物理停止」三段)
GET  /api/localization   → 定位健康(新鲜度+协方差有界+一致性 三判据, 不恒 true)
```

硬约束（接控制时同样生效）：
- 目标状态至少区分 8 态（见上 nav/state）；取消导航 ≠ 停车；断线不自动重发。
- 静态地图不套雷达超时规则；历史轨迹不当作规划路径；旧规划不当作正在执行任务。
- 页面不向 `/car_cmd_vel`、`/teleop_cmd_vel`、`/teleop/cmd_vel` 发任何指令；所有运动只进已批准的后端控制与安全链。

## 四、接口 vs 话题命名红线（勿混淆）

| 名称 | 含义 | 本页是否使用 |
|---|---|---|
| `/teleop_cmd_vel` | 厂家下划线直通(绕过安全层) | 否 |
| `/teleop/cmd_vel` | 手动链输出(进安全层) | 否 |
| `/car_cmd_vel` | 驱动订阅(页面前端不直发) | 否 |
| `/media/*.png` | 本页只读图源 | 是 |

`web_teleop` 是 PWM 直通遥控模块，属旧方案，**本页不调用、不启动**。