> 历史参考：本文件保留写作时的记录；当前参数和状态见 [项目文档入口](../README.md)。

# 实车首次导航试跑故障分析(2026-09-16 16:50-16:53,用户操作,事后只读取证)

## 0. 事实(证据:`~/.ros/log/` 2026-04-04-16-50-56 会话,车机钟=当天)

用户在车上手动跑厂家 `nav_one.sh`(Run_car_one + Run_nav,载 map1),RViz 发单目标,
车全程不动,约 2.5 分钟后 Ctrl+C 收链。事后经 SSH 只读分析,未重跑、未发任何指令。

## 1. 时间线(容器日志 component_container_isolated_31329)

| 车机时刻 | 事件 |
|---|---|
| 16:50:56 | Run_nav 起,nav2 全家装载(map_server/amcl/controller/planner/bt_navigator/velocity_smoother…)+waypoint_cycle |
| 16:51:01→16:51:28 | SmacPlannerHybrid 构建启发式查找表 27s(ARM 四核正常耗时,非故障) |
| 16:51:30 | **bt_navigator: Begin navigating (0.00,0.00)→(1.62,0.21)——规划成功,有路径** |
| 16:51:30-16:53:30 | controller 持续收路径(每 1.15s 重规划);**每 8s 一次 `Failed to make progress`→Abort→BT 重试**,共 4+ 轮;16:52:13 用户改点新目标 (1.26,0.18) 依旧 |
| 16:52:13-16:53:30 | AMCL 位姿始终 (0.00,0.00)/(-0.00,0.00)——**车零位移** |
| 16:53:30 | Ctrl+C;waypoint_cycle 在收链瞬间崩(ExternalShutdownException,与本次不动无关) |

`Failed to make progress` = SimpleProgressChecker:8s 内位移 <0.6m 判失败。即:**路径有、控制算、车没动**。

## 2. 根因分析(两层,按嫌疑排序)

### 主嫌疑:AMCL 初始位姿错误 → 实时激光在错误位置"画出虚拟墙" → RPP 速度被压

- 厂家 yaml `set_initial_pose:True` 且 (0,0,0):**AMCL 硬从建图原点启动,不等待 RViz 校正**;
- 用户 16:50 发车时车的实际位姿 ≠ 今天 15:27 建 map1 的起点位姿(位置/朝向都可能不同);
- 后果链:map→odom 错 → local costmap 把**车实际周围的墙/家具投影到地图 (0,0) 附近** →
  车被"实际不在此处的障碍"围住 → RPP(cost 调节开启)速度压到极低/0 → 8s 零位移 →
  Failed to make progress 循环。AMCL 粒子云未收敛(激光与地图不匹配),但仍有 TF 输出,
  所以规划层看起来一切正常——**这正是"有路径但车不动"的组合特征**。

### 次嫌疑:0.25 m/s 油门(1565µs)低于实际起步死区

- 公式 motor=0.25×100+1540=**1565µs**;今晨标定中位=1540,teleop 前进起步=1560;
  1565 只比 1560 高 5µs,**若实际起步死区 ≥1565,即使速度正确发出电机也不转**;
- 任务书预设了此情况:"若该速度低于实际可控起步速度,先报告,不自行加速"。

### 已排除

- waypoint_cycle 抢目标:其路点列表初始为空(count=0 不发目标),只有 RViz "Publish Point"
  点击才会入列;运行期无发布行为,收链瞬间才崩。**无辜**。
- 地图错载:Run_nav 默认 map_file 即 map1.yaml(当前房间图),正确。
- nav2 装载/生命周期:全节点 Configuring/Activating 成功,无 error。

## 3. 未决问题(下次测试时一并区分主/次嫌疑)

1. **看点**:跑安全链后 `ros2 topic echo /cmd_vel`(safety 输入)与 `/car_cmd_vel`(safety 输出):
   - 输出有非零 v 而车不动 → **次嫌疑成立**(起步死区,报告用户,不自行加速);
   - 输出恒 0/极小 → **主嫌疑成立**(定位/虚拟墙,先解决初始位姿)。
2. **上电后架空阶段(POWERON_PLAN 阶段1)顺带实测**:teleop 前进键(1560)与公式通道 0.25(1565)
   空转表现,直接量出起步死区,不再靠猜。

## 4. 下一步(按任务书阶段推进,不跳步)

- 阶段0:停止链路验证(电调断开,safety_nav_one.sh 链);
- 阶段1:架空——指令方向/零指令/断流/停止行为 + **起步死区实测**;
- 阶段2:**车摆回建图原点位姿(位置+朝向)**,起安全链,RViz 2D Pose Estimate 复核粒子云贴合,
  再谈单目标;AMCL 初始位姿是本轮第一教训;
- 全程用 safety_nav_one.sh(另修了 controller use_sim_time:True 残留、三个速度阈值反向抬速、
  desired 0.25→0.10 三处问题,见 REALCAR_CONTROL_AUDIT §10 与 nav_safe.yaml)。

---
取证:本文件由 2026-09-16 会话 SSH 只读分析生成;原始日志保留于车机 ~/.ros/log/(未删改)。
