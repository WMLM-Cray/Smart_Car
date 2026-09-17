# 单点导航轮 (2026-09-17 深夜) — 排障记录与次日路线

**状态:未完成开车。软件层排障到根因定案,已收链归档。**

## 一、目标

实车单点导航:电调接通授权下,initpose → 发目标 → 车自动行驶到点。

## 二、排障全程(三轮)

### 轮1 假达

goal(1.0m 沿车头)发出 0.44s 后 "Reached the goal!" 零输出。
根因:`xy_goal_tolerance: 0.6` > 目标距离 0.583m,goal_checker 首周期即判达标。非链路故障。
**判据:发目标距离必须 > 0.6m。**

### 轮2 规划失败 GridBased + 自动 spin

"GridBased failed to generate a valid path" 根因链:
车载天线贴身回波(≤0.25m,车正后方 ~0.15m 等距双点)→ costmap obstacle 层 mark → 膨胀 lethal 罩住雷达原点/车位姿 → start invalid → BT 触发行为器 spin(1.57rad,阿克曼原地扭动实际位移 ~0.2m,AMCL 位姿随之改变)。

修复(已落地):
- `scan_filter.py`:/scan → /scan_filtered,≤0.25m 有限值改 inf,SensorDataQoS,信号处理同 cmd_vel_safety 模式(`from rclpy.signals import uninstall_signal_handlers`)
- nav_safe.yaml 三处话题切换:41 行 AMCL scan_topic / 181 行 global costmap / 217 行 local costmap(备份 .bak_scanfilter)
- 部署车机 ~/racecar_team/

验证:`ros2 node info` 确认 costmap 与 AMCL **实际订阅 /scan_filtered**;probe 直订确认 filtered 帧无 ≤0.25 点。

### 轮3 costmap 雪花(本轮核心发现)

天线过滤生效后规划仍失败 "Starting point in lethal space"。
ASCII 渲染 costmap 发现**全图椒盐噪声**:大量散布 100/99 格,自由区只剩零星小岛——不是墙。

根因链:
1. initpose 候选 (0.570,-0.380,162.5°) 来自 spin 扭动后的脏数据对齐,score 仅 0.4827(当时已存疑)
2. **位姿错 → 激光点(墙+人腿)按错误 TF 投影到全图随机位置 mark → costmap 糊死**
3. 重启 bringup 后**逐格复现**(几何确定,非陈旧 mark、非膨胀、非静态层——三者均已排除)
4. 死锁机制:AMCL `update_min_d: 0.025` 静止不更新无法自纠,而 costmap 每帧都在按错误 TF 投影

干净环境帧(min 0.468,人已退开)重新全房域对齐:候选 **(0.010,-0.240,166.5°) score 0.565**,视觉验证中心区贴合尚可、外圈有 5-10° 角度残差。**待次日验证:推车触发 AMCL 收敛后复查。**

## 三、关键发现(独立于主线)

1. **echo 假象**:ros2 topic echo /scan 显示大片区(每帧 691 个 0.0),probe(rclpy 直订)显示 0 个——echo 数据不可信,**探针为权威**。假象与 bringup 伴生,机制未明。
2. **costmap 翻译表语义**:/global_costmap/costmap(OccupancyGrid)发布翻译后值:**100=lethal(原254)/99=inscribed(原253)/0=自由/NO_INFO→-1**。探针阈值不能写 253/254。
3. **膨胀配置**:footprint 0.64×0.36m(内切 0.18m)、robot_radius 0.60(被 footprint 覆盖)、inflation_radius 0.20、cost_scaling 8.0——膨胀小,不构成 lethal 主因。
4. **地图 205 灰度区**:map.yaml 标准阈值(free 0.25/occ 0.65)下 205(occ 0.196)=自由;大片 205=建图未扫区,静态层无罪。
5. **人=移动障碍**:人站车旁 0.5m 产生 0.47-0.6m 段 86 个腿点持续 mark。操作完毕人须退开 >1.5m 再采数/导航。
6. **电调死区(用户实测)≈485mm/s**(v×100+1540 → PWM≈1588)。**当前安全层限幅 ±0.10 远低于死区,链全通也推不动车**——"车不动"的要素之一。放宽限幅=用户安全决策项(起步 ≥0.5 过死区;弯道减速可能掉回死区以下,需实测)。
7. **fastrtps SHM 坏段再现**:本次打在容器 load_node(bringup 仅加载 2 节点卡死,重试无效)。处置:重启车机。服务 call/lifecycle get 挂时绕行:topic pub/echo + 日志 + node list(发现层)。
8. bringup 重启正确姿势:pkill launch 后 container 可能成孤儿,需单独 pkill component_container_isolated;重启后 costmap 重建可清动态污染(但位姿不正确时会立刻重新污染)。

## 四、工具清单(本地 D:\Claude\projects\hwcart_amcl_stage2\,均已 scp 到车机 ~/racecar_team\)

| 工具 | 用途 |
|---|---|
| scan_filter.py | 天线过滤(已部署,今晚运行正常) |
| probe_scan.py | 激光帧分段统计+角度(raw/flt 对比) |
| probe_costmap.py | costmap 抓取:起点 cost/邻域/直方图 |
| probe_ascii.py | costmap ASCII 渲染(排障主力,一眼定雪花) |
| save_scan.py | 采干净帧存 JSON(绕开 echo) |
| align_scan3.py / align_now.py | 全房域粗+精搜离线对齐 |
| plot_overlay.py | 候选位姿本地渲染视觉验证 |
| convert_scan_nav.py | echo YAML→JSON(备用) |

## 五、次日路线(建议顺序)

1. 重启车机(SHM 清干净;注意 udevd 漏 event0 的老坑,雷达枚举后 joy/ros2 daemon 相关见记忆)
2. 起链:`safety_nav_one.sh ~/racecar_team/maps/room_20260917/map.yaml` + scan_filter.py(setsid + source 进子 shell)
3. initpose (0.010, -0.240, 166.5) → **推车 0.5-1m 触发 AMCL 收敛**(update_min_d 0.025)→ 对账:amcl_pose 位移 vs 真实推车距离(±10% 内可信)
4. probe_ascii 复查 costmap:应见"墙线+大片自由";仍雪花 → 请用户把车摆回建图起始位姿,initpose 已知位姿重来
5. 发 >0.6m 目标 → 查 controller_server 日志 GridBased 出路径
6. **用户决策:限幅放宽方案**(cmd_vel_safety ±0.10 → 起步 ≥0.5)
7. 接电调(断 2s 再通唤醒,急停手边)→ 发车监测 /cmd_vel /car_cmd_vel /amcl_pose
8. 判定+归档

## 六、安全边界重申

- /teleop_cmd_vel(下划线)=厂家直通 PWM,**绝对禁用**
- 断连必须整链重新锁定,不灌零
- 限幅放宽 = 用户决策,不自行改
- 目标距离 >0.6m;车机时钟慢 5 个月,日志时间戳只作相对刻度
