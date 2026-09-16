# trackA 阶段3 第一步:可见二维建图演示(交付记录)

日期:2026-09-16 | 分支:integration/trackA-sdf | 状态:**已完成并验证**

任务书要求:A 赛道 + 车辆 + slam_toolbox + RViz 同屏显示(地图/激光/车辆位置/轨迹),
复用短距离建图成果,把启动方式和配置正式入库。

## 一、结论

一键脚本完整跑通:起仿真 → slam_toolbox 在线建图 → RViz(虚拟屏)同屏显示 →
车辆行驶 2.36m → map_saver 正式存图。RViz 截图确认同屏四要素齐全
(占用栅格地图 + 红色激光点 + 机器人模型/TF + 里程计轨迹箭头链),Global Status Ok。

## 二、入库文件

| 文件 | 作用 |
|---|---|
| `hwcart_sim/src/racecar_bringup_sim/`(新包) | 仿真之上的演示层包 |
| ├ `launch/slam_bringup.launch.py` | 一键启动:复用 racecar_sim/sim_bringup(车辆+控制器+接线箱),追加 slam_toolbox online_async + RViz |
| ├ `rviz/slam.rviz` | RViz 配置:Map(/map, Transient Local)+ LaserScan(/scan)+ RobotModel + TF + Odometry 轨迹(Keep=100 箭头)+ SetInitialPose 工具(供第三步 AMCL 用) |
| ├ `setup.py` / `package.xml` / `setup.cfg` / `resource/` | 安装 launch+rviz 到 share,依赖 racecar_sim/slam_toolbox/rviz2 |
| `tools/trackA/slam_demo_run.sh` | 演示编排:Xvfb 虚拟屏 → slam_bringup → 行驶前截图 → 驱动 → 行驶后截图 → map_saver → 进程组清理 |
| `docs/track/slam_demo_rviz_0_start.png` | 行驶前 RViz 截图 |
| `docs/track/slam_demo_rviz_1_after.png` | 行驶后 RViz 截图(同屏四要素,已 AI 视觉复核) |
| `docs/track/trackA_map.pgm` / `.yaml` | map_saver 正式存图(335×113 @0.05m,origin [-11,-0.854,0]) |

## 三、启动方式(入库)

### 方式 A:一键脚本(WSL ubuntu2204-ros2 内)

```bash
bash /home/lzx/hwcart_sim/tools/trackA/slam_demo_run.sh
```

流程:起 Xvfb(:95, 1400×900)→ `slam_bringup.launch.py`(use_gui:=false,
spawn 11.0/0.75/0.0)→ 等 75s 就绪 → 行驶前截图 → `slam_smoke.py` 驱动
(前进 5s + 缓右弧 4s,共 2.36m)→ 行驶后截图 → `map_saver_cli -f
docs/track/trackA_map` → 杀进程组。全程约 3 分钟。

### 方式 B:手动分步(有显示器时,交互观察)

```bash
# 终端1:仿真+SLAM+RViz 一键(gzclient 图形界面也开)
ros2 launch racecar_bringup_sim slam_bringup.launch.py use_gui:=true
# 终端2:键盘开车(Twist,接力棒式点动,0.5s 无指令自动零速)
ros2 run teleop_twist_keyboard teleop_twist_keyboard -p speed:=0.3 -p turn:=0.4
# 行驶后存图:
ros2 run nav2_map_server map_saver_cli -f ~/hwcart_sim/docs/track/trackA_map
```

参数:use_sim_time(默认 true)/ world(默认 trackA.world)/ spawn_x|y|yaw
(默认 11.0/0.75/0.0)/ rviz(无头可置 false 仅建图)/ use_gui(默认 false,
RViz 为主屏)。

## 四、本次运行证据(2026-09-16 08:12,pgid 613)

启动节点(launch.log,0 error):robot_state_publisher、gzserver、spawn_entity.py、
**async_slam_toolbox_node**、rviz2(@Xvfb:95)、spawner×2( joint_state_broadcaster +
ackermann_steering_controller)、sim_wiring。

驱动日志(slam_smoke.py):
```
[OK] 起点 (11.00,0.75)
[OK] 终点 (13.28,0.15) 行驶 2.36m
[INFO] /map 335x113 res=0.050m free=12255 occ=376 unknown=25224
[PASS] 地图尺寸 / free 格 / occupied 格 三项判据全过
[OK] map_saver 已存 docs/track/trackA_map.pgm/.yaml
```

## 五、边界遵守(任务书约束)

- SLAM 输入只有模拟传感器 `/scan` + 正常 TF(odom→base_footprint,sim_wiring 桥),
  **未注入任何 Gazebo 真值定位**;
- slam_toolbox 零参数匹配本项目链路(base_frame=base_footprint / odom_frame=odom /
  scan_topic=/scan),launch 只显式传 use_sim_time;
- 未修改冻结赛道与车辆几何;
- 复用了短距离建图(slam_smoke.py)的驱动与判据,未重写。

## 六、遇到的问题与修复

1. `slam_bringup.launch.py` 首版导入名写错(`PythonLaunchDescription`,
   正确为 `PythonLaunchDescriptionSource`)→ launch 直接异常退出,车未 spawn,
   截图 306 字节空白。修正三处后重跑全绿。已固化在入库版本。
2. WSL VM 空闲重启会清空 /tmp(日志文件丢失),排障时改用 `~/.ros/log/` 持久
   目录取证。演示脚本输出同时落 tee 与终端,不受影响。

## 七、下一步

阶段3 第二步:仿真测试驾驶器(冻结几何车道内参考路线,整车轮廓余量校验,
阿克曼约束,分段扩展到整圈,失速/碰撞/偏出即停并报告)。
