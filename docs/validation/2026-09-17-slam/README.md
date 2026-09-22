# 阶段1: 实车扫图 归档 (2026-09-17)

任务书主线: 暂停手柄(冻结于 68b3e3f), 实车扫图 → AMCL → 单点导航 → 多点任务器。
本轮物理状态: 电调全程断开, 纯手推车, 车机在线(ssh hwcar)。

## 交付物

| 文件 | 说明 |
|---|---|
| map.pgm / map.yaml | 当前房间地图 (83x72 @ 0.05m, origin=(-2.500,-2.068)), map_server 标准格式 |
| final_monitor.png | 扫图结束时监控画面(车机渲染: 地图+激光+轨迹) |
| check_snapshot.txt | 链路只读核对(话题/TF发布者/AMCL缺席/频率) |
| mapserver_reload_log.txt | 重载验证: map_server 从盘读取 "83 X 72 map @ 0.05 m/cell" 与保存一致 |

车机侧完整目录 `~/racecar_team/maps/room_20260917/` 另含 slam_toolbox 序列化文件
(slam_map.posegraph 6.9MB + slam_map.data) 与启动配置快照。

## 扫图链 (tools/realcar/)

- `slam_scan.sh start` = 厂家 Run_car.launch.py(传感器+EKF+driver, 不改厂家文件) + slam_toolbox online_async
- `realcar_slam.yaml` = 车机官方默认参数为基线, 改动仅 4 项: odom_frame=odom_combined(对齐 EKF)、
  max_laser_range=12、minimum_travel_distance/heading=0.3、min_laser_range=0.15(实测 range_min)
- TF 链: map→(slam_toolbox)→odom_combined→(EKF)→base_footprint→(static)→laser_link
- `map_monitor.py` = 车机本机渲染 /map+/scan+轨迹→PNG (准实时观察, 见"偏离"节)
- `save_map_pgm.py` = latched /map → PGM+YAML 转存 (替代 Humble 不稳的 map_saver_cli)

## 判据核对 (任务书"扫图成功判据")

1. 地图覆盖计划导航区域: PASS — 房间 4.15×3.6m 全覆盖(用户确认房间尺寸)
2. 墙体轮廓连续无重影: PASS — 回环后用户画面确认无重影/跳变(final_monitor.png 留证)
3. 保存后能重新加载: PASS — map_server configure 读出 83X72@0.05 与保存一致(mapserver_reload_log.txt)
4. TF 无缺失或重复发布: PASS — /tf 发布者恰 3 个(slam_toolbox + ekf_filter_node + slam 内部),
   static 走 /tf_static; AMCL 无进程无话题(check_snapshot.txt)

过程记录: 第一轮扫图未存图(用户要求重来), slam_scan.sh restart 子命令重置后第二轮完成;
两轮均为低速手推(0.3~0.5m/s)+小角度转弯+回环回起点。

## 已知偏离 (如实记录)

- **任务书"在RViz显示"条款以等效工具满足**: 车机 map_monitor.py(matplotlib)渲染 + SSH 每2s拉图 +
  浏览器自动刷新页面, 准实时(2s 级)观察地图/激光/轨迹。
  原因: 无多余显示器接车机; WSL2 mirrored 网络(车机同网段)实测 **UDP 入站缺陷**——
  防火墙双层窄规则(Hyper-V + 主防火墙, 仅放行车机 IP 的 UDP 入站)与 FastDDS
  interfaceWhiteList 均无法让车机→WSL 的单播数据通过(发现层多播可达, 数据层不可达;
  WSL→车机方向全通), RViz 跨机方案放弃, 未再消耗时间攻 FastDDS TCP。
- 防火墙两条诊断遗留规则(仅放行来源 192.168.5.100 的入站 UDP, 无害, 可删):
  `Remove-NetFirewallHyperVRule -Name 'WSL-DDS-car'` / `Remove-NetFirewallRule -DisplayName 'WSL-DDS-car-inbound'`
- .wslconfig networkingMode NAT→mirrored(为跨机 DDS 所改, 保留——WSL 与车机互 ping/SSH 场景受益)

## 坑与修正 (本轮新增)

1. `set -u` + source ROS setup.bash → AMENT_TRACE_SETUP_FILES unbound(deploy_teleop 同款), 前后 set +u/-u
2. **Humble map_saver_cli 对 latched /map 不稳**: 默认 volatile 订阅收不到停链后无新帧的 latched 图
   (Failed to save); 加 `map_subscribe_transient_local:=true` 又 "Failed to spin map subscription"。
   改自写 save_map_pgm.py(transient_local 订阅+PGM P5 手写, 注意 OccupancyGrid 首行=y最小需翻转到 PGM 行序)
3. 初版 cmd_save 把 --occ 0.25 --free 0.65 写反(正确: occ=0.65 占据/free=0.25 自由), 修正
4. `pkill -f` 自杀坑两连: ssh/bash -c 命令行含模式字样必中招, 字符类只保一次出现;
   杀 map_server 用 `pkill -x`(精确进程名)
5. wsl.exe 管道传文件: 后台符 & 会把整条链(含读 stdin 的 cat)挂后台致 0 字节, 传文件单独跑
6. 车机 WiFi DDS 发现 ~4s(T12 已知), 脚本等待逻辑已按 90s 上限覆盖

## 启动/收车步骤 (复现)

```
车机: ~/racecar_team/tools/slam/slam_scan.sh start     # 起链+等就绪
车机: ~/racecar_team/tools/slam/slam_scan.sh check     # 只读核对
(推车扫图; 观察: Windows 侧 monitor.html + 拉图循环)
车机: ~/racecar_team/tools/slam/slam_scan.sh save <名字>  # serialize+PGM/YAML+快照
车机: ~/racecar_team/tools/slam/slam_scan.sh stop       # 按序收链 slam→Run_car→monitor→daemon
```

收车后物理停止优先级不变: 物理断电调 > 脚本 stop > Ctrl+C。

## 下一步

阶段2 AMCL: 关闭 SLAM, 加载本地图(map.yaml), AMCL 组件 + RViz 2D Pose Estimate 设初始位姿,
低速短距验证跟随无跳变。判据与排查清单按任务书执行。
