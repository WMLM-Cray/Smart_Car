# 显示交接 —— 车机渲染 + 浏览器查看(2026-09-19,SSH+Edge,队友不装 ROS)

> 方案:车机本机跑 `map_monitor.py` 渲染 PNG,队友电脑拉 PNG + 浏览器(`monitor.html`)每 2s 刷新。
> **不是 web_teleop**(那是队友仓库里的遥控模块,无地图/雷达/路径显示,不启动,也非显示方案)。
> 队友环境已确认:Windows 11 + ssh/scp/curl + Edge,WSL Ubuntu-22.04 暂未启动,不要求装 ROS。
> 本版 = **v3 路径显示版**(在 v2 时间戳新鲜度基础上, 新增可选规划路径渲染 `--path-topic`, 默认关)。

---

## 1. 真实文件、依赖、启动/关闭

### 车端运行(唯一渲染器 + 独立启停脚本)

| 文件 | 作用 | 依赖(车机已有) |
|---|---|---|
| `map_monitor.py` | 订阅 `/map` + `/scan` + TF(+可选规划路径),渲染 PNG(黑墙/白自由/灰未知/红激光/蓝轨迹/绿车/青路径 + **左上角新鲜度状态**) | python3、`rclpy`、`numpy`、`matplotlib`(Agg,无 GUI)、`tf2_ros`、`nav_msgs`、`sensor_msgs` |
| `path_display.py` | 路径 frame 对齐 + 状态行文案的纯逻辑(无 ROS 依赖,可离线单测) | 仅 `math` |
| `display_monitor.sh` | **独立启动/关闭/状态**,不借 `slam_scan.sh`(避免误启另一套 SLAM 链) | bash |

- 输出:`~/racecar_team/logs/slam/monitor.png`(每 2s 原子替换 `os.replace`,拉图不会拉到半张)。
- **启动(车机,用独立脚本,自带启动边界检查)**:
  ```bash
  cd ~/racecar_team
  ./display_monitor.sh start     # 自动:确认 ROS 环境已加载→确认输出目录存在→确认无已有实例→启动并记 PID
  ./display_monitor.sh status    # 查看本轮 PID + PNG 最新时间
  ./display_monitor.sh stop      # 只停本轮 PID(读 PIDFILE 精确 kill,不用 pkill -f)
  ```
  脚本会拒绝重复启动、找不到 map_monitor.py、ROS 未加载这三类情况并给出明确提示。
- **启动边界(脚本已内置,人工也要知道)**:①先确认 ROS 环境已加载(`command -v ros2` 或 `source /opt/ros/humble/setup.bash`)②输出目录 `logs/slam/` 存在(`mkdir -p`)③没有已有 monitor 实例(有则先 stop 再 start)④记录本轮 PID,关闭只停本轮进程。**定位阶段不要为启动显示去调 `slam_scan.sh`**,那会启动另一套 SLAM 链。

### 队友电脑运行(拉图脚本 + 浏览器)

| 文件 | 作用 | 依赖 |
|---|---|---|
| `fetch_monitor.bat` | Windows cmd 拉图循环(临时 PNG + 原子替换 + 失败保留旧图) | ssh/scp |
| `fetch_monitor.sh` | Git Bash/WSL 同款拉图循环 | ssh/scp |
| `monitor.html` | 显示同目录 `monitor.png`,`<meta refresh 2>` 每 2s 重读 | 仅 Edge |

- 队友端流程(Windows):
  ```bat
  fetch_monitor.bat 192.168.5.100 orange-pi   REM 默认即这两个值, 可省略参数
  ```
  脚本每 2s 一轮,串行执行(不堆积 scp 进程):先 scp 到 `monitor.png.tmp`,成功才原子 rename 成
  `monitor.png`,失败保留旧图并打印失败/时间。然后 Edge 打开 `monitor.html`(与 monitor.png 同目录)。

---

## 2. 哪些放车端 / 哪些在队友电脑

| 文件 | 位置 |
|---|---|
| `map_monitor.py` | **车端** `~/racecar_team/`(渲染器) |
| `path_display.py` | **车端** `~/racecar_team/`(map_monitor.py 的依赖,必须同目录一起部署) |
| `display_monitor.sh` | **车端** `~/racecar_team/`(启停脚本,与 map_monitor.py 同目录) |
| `fetch_monitor.bat` / `fetch_monitor.sh` | **队友电脑**(拉图,任选一个) |
| `monitor.html` | **队友电脑**(与拉下来的 monitor.png 同目录) |
| `save_map_pgm.py` | **车端**(存图用,与显示无关,随包备用) |
| `realcar_slam.rviz` | 车端 RViz 可选方案(需车机接显示器,SSH 方案不用) |

---

## 3. 画面冻结识别(必须能识别,离线回放明确标注)

**先读这一句:现有三种"辅助检查"都只能辅助诊断,不能可靠判断输入是否新鲜;只有 PNG 上的
scan/TF 时间戳才是权威判据。**

| 方法 | 判据 | 性质 |
|---|---|---|
| **PNG 左上角时间戳** | `scan: Ns ago [STALE]` / `tf: Ns ago [STALE]` 红色醒目 = 输入冻结(权威) | ✅ 权威 |
| PNG mtime | `monitor.png` 超 ~10s 未更新 = 渲染器或输入冻结 | ⚠️ 辅助 |
| 渲染器进程 | `pgrep -f map_monitor.py` 无 = 渲染器死 | ⚠️ 辅助 |
| 画面内容 | 车动而轨迹/激光不动 = 输入冻结 | ⚠️ 辅助 |

关键澄清:
- **v2 已在 PNG 左上角画 `/scan` 最后接收时间 + TF 数据时间**,超过 2s 标红色 `[STALE]`。
  这直接回答"雷达是否仍在更新",**浏览器刷新、PNG mtime、进程存在都不能证明雷达在更新**——
  渲染器活着但输入停更时,mtime 仍每 2s 刷新,只有时间戳能暴露。
- **`/map` 是静态图,不持续更新是正常的,不能因 /map 不更新就报故障**。v2 只标"已加载(静态)",
  不判过期。
- 离线回放/录像必须在画面标注"录像"字样,不冒充实时。

---

## 4. 已确认 / 需现场发现 的接口(不只列话题名)

| 显示项 | 话题(类型) | QoS | 状态 |
|---|---|---|---|
| 地图 | `/map` (`nav_msgs/OccupancyGrid`) | TRANSIENT_LOCAL + RELIABLE | ✅ 已确认 |
| 扫描 | `/scan` (`sensor_msgs/LaserScan`) | BEST_EFFORT + VOLATILE | ✅ 已确认 |
| 轨迹/车位姿 | TF `map→base_footprint`(`tf2_ros`) | — | ✅ 已确认(蓝轨迹+绿三角) |
| 定位协方差/粒子 | `/amcl_pose`、`/particle_cloud` | — | ⚠️ 未渲染,需现场发现/后续扩展 |
| 规划路径 | `nav_msgs/Path`(话题名现场发现) | 订阅侧 TRANSIENT_LOCAL+RELIABLE | ✅ 已实现(`--path-topic` 启用,默认关) |
| 导航状态 | nav2 状态/目标反馈 | — | ⚠️ 未覆盖(路径线当前有效性无从判断) |

> 路径显示已实现: `display_monitor.sh start <路径话题名>`(或 `map_monitor.py --path-topic <话题名>`)启用,
> 默认不订阅 = 只画 map+scan+TF, 与 v2 一致。
> **现场发现路径话题时不能只看 `ros2 topic list` 的名字, 必须同时确认三件事:**
> ①话题类型是 `nav_msgs/msg/Path`(`ros2 topic info <话题> -v` 看 type) ②发布者是哪个节点 ③实际 QoS
> (订阅侧 TRANSIENT_LOCAL+RELIABLE; 若发布者 VOLATILE, 后启动的显示器收不到最后一条, 要等下一次规划)。

**路径显示的限制(如实)**
- **frame 对齐**: 路径 `header.frame_id` 与地图 frame(默认 `map`)一致时直接画; 不一致时用 TF 转换,
  TF 查不到则**拒绝绘制并在图上提示, 绝不直接套坐标**。
- **空路径清旧线**: 收到空 `Path` 清掉旧线(render 每帧重建图, 空路径不画)。
- **无路径消息/TF 缺失不崩**: 无路径只标"无数据"; frame 不同且无 TF 只提示不绘制; 均不崩溃。
- **旧线≠当前有效路径**: 本工具**没有导航状态订阅**, 路径线只是"最近一次收到的路径", 目标结束/取消后
  若发布者不发新路径或空路径, 旧线仍会显示——状态行只标"最近收到 Ns ago", 不做有效性判定。
- **后启动能否收到已有路径取决于发布者 QoS**: 发布者 TRANSIENT_LOCAL → 后启动能收到最后一条;
  发布者 VOLATILE → 后启动收不到, 等下一次规划。以 `ros2 topic info` 实测为准, 不预设。

---

## 5. 地图说明(本轮显示不依赖地图文件)

- **显示**:`map_monitor.py` 渲染 `/map` **话题**(扫图=slam_toolbox 实时出;定位=map_server 读已有地图),
  不读地图文件,显示阶段队友无需拿地图文件。
- **地图文件**(导航演示才需要,在车端):`~/racecar_team/maps/<名字>/` 或 `~/racecar/src/racecar/map/`。
  本地存档参考:`docs/validation/2026-09-17-slam/map.yaml`(09-17 实车 SLAM 图)。
- **队友第一轮只读盘点时核对**:车端现有哪些地图、哪张匹配明晚场地(户外新场地则旧房间图不适用)。

---

## 6. 部署/回滚(显示工具,幂等)

```bash
# 部署(车端已有同名文件先备份; map_monitor.py / display_monitor.sh 放 ~/racecar_team/)
ssh orange-pi@<车机IP> 'cp ~/racecar_team/map_monitor.py ~/racecar_team/map_monitor.py.bak_$(date +%Y%m%d) 2>/dev/null || true'
scp map_monitor.py path_display.py display_monitor.sh save_map_pgm.py orange-pi@<车机IP>:~/racecar_team/
ssh orange-pi@<车机IP> 'chmod +x ~/racecar_team/display_monitor.sh'
# 核验(比对 SHA256,见交付包 MANIFEST)
ssh orange-pi@<车机IP> 'sha256sum ~/racecar_team/map_monitor.py'
# 回滚
ssh orange-pi@<车机IP> 'mv ~/racecar_team/map_monitor.py.bak_* ~/racecar_team/map_monitor.py'
```

`monitor.html` + `fetch_monitor.*` 只在队友电脑本地,不部署到车端。
