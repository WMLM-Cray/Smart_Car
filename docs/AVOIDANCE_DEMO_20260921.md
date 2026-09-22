# Nav2 自带避障：演示步骤 + 参数校核（2026-09-21）

> 结论：避障**已经是现役链的一部分**，不新增任何代码。原理 = Nav2 局部代价图
> `local_costmap`（`voxel_layer + inflation_layer`）订阅 `/scan`，导航时控制器据实
> 时代价图自动偏航绕障。本文只给"怎么演示"和"不绕时怎么校核"。

## 1. 避障在哪一段（已核对）

`bash safety_nav_one.sh <地图>` → `bringup_launch.py` → nav2 `navigation_launch.py`
→ `controller_server`（带 `local_costmap`）。现役参数 `tools/realcar/nav_safe.yaml`：

| 项 | 值 | 含义 |
|---|---|---|
| local_costmap plugins | `["voxel_layer","inflation_layer"]` | 障碍体素层 + 膨胀层，已启用 |
| observation_sources / topic | `scan` / `/scan` | 用雷达做障碍观测 |
| obstacle_max_range | 2.5 m | 2.5m 内的点才标成障碍 |
| obstacle_min_range | 0.1 m | 0.1m 内（盲区）不标 |
| inflation_radius | 0.20 m | 障碍外扩 0.2m，车保持此安全距离 |
| global_costmap 额外层 | `obstacle_layer` | 动态障碍也进全局图 |

> 注意：local 和 global 两处观察话题一处写 `/scan`、一处写 `scan`（无斜杠）。默认空
> namespace 下两者都解析到 `/scan`，但现场应 `ros2 topic info` 确认 costmap 实际订的是
> 和 AMCL 同一个雷达话题（见 §4 那件 `/scan` vs `/scan_filtered` 旧事）。

## 2. 演示步骤（结合相机同屏看）

终端布局（可同时三件事）：

```
终端1: bash safety_nav_one.sh <地图yaml>          # 车链+导航
终端2: ros2 launch racecar camer.launch.py         # 相机（前提 ros-humble-usb-cam 已装）
       ros2 run rqt_image_view rqt_image_view       # 车机显示器看实时画面
       # 或队友远程: ros2 run web_video_server web_video_server -> 浏览器
       #   http://<车机IP>:8080/stream?topic=/image_raw
终端3: ros2 run rviz2 rviz2（或沿用导航族 RViz）    # 给目标 + 看 local_costmap
```

避障演示流程：

1. 正常起车 + AMCL 定位（同单点），车静止。
2. 在车辆与导航目标之间的必经路径上放一个**纸箱/路障**，离车 1~2m、在雷达扫描平面高度、
   且 < 2.5m（能被雷达照到）。
3. RViz 给一个目标点，让目标在障碍物**后方**。
4. 观察 RViz `local_costmap`：障碍处出现**膨胀区**；车会**先减速 → 偏航绕开 → 回到目标线**。
5. 判定通过：车不碰纸箱、轨迹明显绕弯、最终到位。

## 3. 不绕 / 顶上去时的校核顺序（一次一个因素）

1. **雷达在不在出**：`ros2 topic hz /scan` 有稳定输出（正常 ~10Hz+）。
2. **话题对没对上**：`ros2 topic info /local_costmap/costmap_raw -v` 看订阅方是否就是雷达话题。
3. **障碍是否被打进图**：RViz 看 `local_costmap` 膨胀区有没有出现在纸箱位置；没有 → 是
   `obstacle_max_range=2.5` 之外、或话题不一致（§4）。
4. **绕得离障碍太近** → `inflation_radius` 0.20 调大到 0.30。
5. **绕得太远/被卡住频繁重规划** → 调小回 0.15~0.20。
6. 改完 `nav_safe.yaml` 记得重启导航链生效；只动这一个文件，不扩大。

## 4. 那件绕不开的旧事（一句话提醒，勿现在改）

0917 现场曾把雷达接成 `/scan_filtered`（scan_filter 过滤），与仓库默认 `/scan` 不一致。
**避障能不能生效，前提是 AMCL 和 local_costmap 订的是同一个在出帧的雷达话题**。现场先
`ros2 topic list | grep scan` 确认当前到底发的是 `/scan` 还是 `/scan_filtered`，再据此统一
`nav_safe.yaml` 里的 `scan_topic` 和两处 observation_sources.topic。默认按 `/scan`。

## 5. 安全边界（不夸大）

- 避障是"**绕**"，不是"**刹**"：最后一米停车的硬保险仍是 `cmd_vel_safety`（`/cmd_vel` 限幅
  + 超时零速），避障不替代它。
- 雷达盲区看不到：车身正下方、比激光平面高/低的悬空物、贴到车壳的障碍，Nav2 兜不住。
- 全程有人，手在电调开关上；失控物理断电优先于任何软件处置。