# 显示交接 —— 车机渲染 + 浏览器查看(2026-09-19,SSH+Edge,队友不装 ROS)

> 方案:车机本机跑 `map_monitor.py` 渲染 PNG,队友电脑用 `scp` 拉 PNG + 浏览器(`monitor.html`)
> 每 2s 刷新。**不是 web_teleop**(那是队友仓库里的遥控模块,未经核对不启动,也非显示方案)。
> 队友环境已确认:Windows 11 + ssh/scp/curl + Edge,WSL Ubuntu-22.04 暂未启动,不要求装 ROS。

---

## 1. 真实文件、依赖、启动/关闭

### 车端运行(唯一渲染器)

| 文件 | 作用 | 依赖(车机已有,因它就在车机跑) |
|---|---|---|
| `map_monitor.py` | 订阅 `/map` + `/scan` + TF,渲染 PNG(黑墙/白自由/灰未知/红激光/蓝轨迹/绿车) | python3、`rclpy`、`numpy`、`matplotlib`(Agg 后端,无 GUI)、`tf2_ros`、`nav_msgs`、`sensor_msgs` |

- 输出:`~/racecar_team/logs/slam/monitor.png`(每 2s 原子替换 `os.replace`,拉图不会拉到半张)。
- 启动(车机,需先 source 环境):
  ```bash
  cd ~/racecar_team
  nohup python3 map_monitor.py > logs/slam/monitor.log 2>&1 < /dev/null &
  ```
  或由现成链自动带起:`slam_scan.sh restart`(扫图)、`amcl_localize.sh`(定位)都会拉起它。
- 关闭:`kill <map_monitor PID>`;链式收链(`slam_scan.sh stop` / `amcl_localize.sh stop`)也会停它。

### 队友电脑运行(只拉图 + 浏览器)

| 文件 | 作用 | 依赖 |
|---|---|---|
| `monitor.html` | 显示同目录 `monitor.png`,`<meta refresh 2>` 每 2s 重读本地 PNG | 仅 Edge |
| (拉图循环) | 每 ~2s 把车端 PNG 拉到本地 | ssh/scp(或 `ssh cat`) |

- 队友电脑侧流程(Windows,一条循环即可,无需 ROS):
  ```bat
  :loop
  scp orange-pi@<车机IP>:~/racecar_team/logs/slam/monitor.png monitor.png
  timeout /t 2 /nobreak >nul
  goto loop
  ```
  然后把 `monitor.html` 与 `monitor.png` 放同一目录,Edge 打开 `monitor.html`
  (页面 2s 自动刷新,重新读取同目录已被拉新覆盖的 `monitor.png`)。

---

## 2. 哪些放车端 / 哪些在队友电脑

| 文件 | 位置 |
|---|---|
| `map_monitor.py` | **车端** `~/racecar_team/`(渲染器,必须在车机 ROS 网络内) |
| `monitor.html` | **队友电脑**(与拉下来的 `monitor.png` 同目录) |
| `save_map_pgm.py` | **车端**(存图用,与显示无关,随包备用) |
| `realcar_slam.rviz` | 车端 RViz 可选方案(需车机接显示器,SSH 方案不用) |

---

## 3. 画面冻结识别(必须能识别,离线回放明确标注)

| 方法 | 判据 | 命令(队友侧) |
|---|---|---|
| PNG mtime | `monitor.png` 超过 ~10s 未更新 = 渲染器或输入冻结 | `ssh ... 'stat -c %y ~/racecar_team/logs/slam/monitor.png'` |
| 渲染器进程 | `map_monitor.py` 不在 = 渲染器死了 | `ssh ... 'pgrep -f map_monitor.py || echo 无'` |
| 画面内容 | 推车/转向时红激光点与蓝轨迹应变化;静止则红点仍随噪声微变 | 肉眼 |
| 输入侧(定位链) | `/scan` 无发布者 / TF 不通 = 渲染停更 | `slam_scan.sh check` 或 `amcl_localize.sh check` |

> 渲染器活着但输入(/map、/scan、TF)停更时,PNG mtime 仍会刷新(每 2s 重画同一张)——
> 所以"mtime 新鲜"不等于"画面新鲜"。**冻结判据要结合画面内容**:若车在动而轨迹/激光不动,
> 或地图长时间不变,即为输入冻结。离线回放/录像必须在画面标注"录像"字样,不冒充实时。

---

## 4. 已确认 / 需现场发现 的接口(不只列话题名)

| 显示项 | 话题(类型) | QoS | 状态 |
|---|---|---|---|
| 地图 | `/map` (`nav_msgs/OccupancyGrid`) | TRANSIENT_LOCAL + RELIABLE | ✅ 代码已确认(`map_monitor.py` 订阅) |
| 扫描 | `/scan` (`sensor_msgs/LaserScan`) | BEST_EFFORT + VOLATILE | ✅ 代码已确认 |
| 轨迹/车位姿 | TF `map→base_footprint`(`tf2_ros`) | — | ✅ 代码已确认(蓝轨迹+绿三角) |
| 定位协方差/粒子 | `/amcl_pose`、`/particle_cloud` | — | ⚠️ 当前工具**未渲染**,需现场发现/后续扩展 |
| 规划路径 | `/plan` 或 `/global_plan`(`nav_msgs/Path`) | — | ⚠️ 当前工具**未渲染**,需现场发现 |
| 导航状态 | nav2 状态/目标反馈 | — | ⚠️ 当前工具**未覆盖**,需现场发现 |

> 当前 `map_monitor.py` 只画 map+scan+TF 轨迹;路径/定位协方差/导航状态不在其中。
> **传感器上线后即可先验证显示(有 map+scan+轨迹即可看),不用等导航成功。**

---

## 5. 地图说明(本轮显示不依赖地图文件)

- **显示**:`map_monitor.py` 渲染的是 `/map` **话题**(扫图=slam_toolbox 实时出;定位=map_server
  读已有地图),**不读地图文件**,故显示阶段队友无需拿地图文件。
- **地图文件**(导航演示才需要,在车端):`~/racecar_team/maps/<名字>/` 或 `~/racecar/src/racecar/map/`。
  本地存档参考:`docs/validation/2026-09-17-slam/map.yaml`(09-17 实车 SLAM 图)。
- **队友第一轮只读盘点时核对**:车端现有哪些地图、哪张匹配明晚场地(户外新场地则旧房间图不适用,
  需另建图+定位验证,提前说明时间影响)。

---

## 6. 部署/回滚(显示工具,幂等)

```bash
# 部署(车端已有同名文件先备份)
ssh orange-pi@<车机IP> 'cp ~/racecar_team/map_monitor.py ~/racecar_team/map_monitor.py.bak_$(date +%Y%m%d) 2>/dev/null || true'
scp map_monitor.py save_map_pgm.py orange-pi@<车机IP>:~/racecar_team/
# 核验(比对 SHA256,见交付包 MANIFEST.sha256)
ssh orange-pi@<车机IP> 'sha256sum ~/racecar_team/map_monitor.py'
# 回滚
ssh orange-pi@<车机IP> 'mv ~/racecar_team/map_monitor.py.bak_* ~/racecar_team/map_monitor.py'
```

`monitor.html` 只在队友电脑本地,无需部署到车端。
