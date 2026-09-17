# 阶段2 实车 AMCL 定位验证报告(2026-09-17 晚)

- 环境:orange-pi@192.168.5.100(ssh hwcar),Run_car(传感器+EKF+driver,电调全程断开)+ map_server + amcl + lifecycle_manager,工具链 `tools/realcar/amcl_localize.sh`(start/check/stop)
- 地图:阶段1 成果 room_20260917(83x72 @ 0.05m,提交 47575b1,未返工)
- 边界遵守:未发任何 Nav2 目标;未启动 planner/controller/bt_navigator/waypoint/手柄/slam;运动话题全程零发布者(check_20260404_194708.txt)
- ⚠️ 车机钟慢 ~5 个月(显示 2026-04-04),全部时刻只作相对刻度

## 结论总表

| 项 | 内容 | 判定 |
|---|---|---|
| 四.1 | 链路启动+前置检查(串口别名/地图文件/无 slam 并存) | PASS |
| 四.2 | 无旧 map→odom_combined 发布者;TF 边唯一性(见口径②) | PASS |
| 四.3 | 初始位姿推算+发布(非默认 0,0,0) | PASS |
| 四.4 | 静止观察 ≥30s(位姿/协方差/粒子云) | PASS(表述按口径①修正) |
| 四.5 | 粒子云集中到车附近 | PASS |
| 四.6 | stop→start 重启链复现 | PASS |
| 四.7 | 人工缓推 0.3~0.5m 三方对账 | **FAIL**(根因=轮式里程计比例高估 ~2.2×,AMCL 机制本身正常) |
| 四.8 | 收链按序停止 | PASS |

四.7 未过,阶段2 整体**不判通过**;修复路径见第五节(2026-09-17 晚用户决定:受控标定暂不执行,择期重测)。

---

## 一、口径修正说明(2026-09-17 用户终审指示,覆盖中期报告四处表述)

**① 静止期"0 漂移"的正确表述**
静止 60s(watch_static.txt)/重启后 15s(watch_restart.txt)采样中 /amcl_pose 均无新帧(latch 补发 1 帧 + 新帧 0)。这**只能证明"没有收到新的定位估计"**——AMCL 未达 update_min_d=0.25m / update_min_a=0.2618rad 更新门限,不重估、不发布,属设计行为;**不能将重复读取旧 latch 值的"0 漂移"表述为"持续重估且稳定"**。静止段的证据效力来自综合:/scan 持续输入(20s 窗 218~234 帧,10.9~11.7Hz)、TF 时效正常(odom_combined→base_footprint 实时)、以及四.7 移动触发的更新行为。

**② TF 边唯一性的证明依据(发布者数量不足为证)**
/tf 仅有 ekf_filter_node 与 amcl 两个动态发布者,**发布者数量本身不足以证明某一条 TF 边唯一**。实际核对(配置+行为双证据):
- ekf.yaml:`world_frame: odom_combined`、`base_link_frame: base_footprint`、`publish_tf: true` → EKF 发布 **odom_combined→base_footprint**(话题名 /odom_combined 亦实测,非 /odometry/filtered);
- amcl_realcar.yaml `odom_frame_id: odom_combined` → AMCL 发布 **map→odom_combined**;
- 行为验证:EKF 单独运行时 `tf2_echo map odom_combined` 不可解析(start 链里 no_stale_map_odom 检查),AMCL active 后才可解析 → map→odom_combined 仅由 amcl 发布。
两节点父子 frame 互不重叠,每条边唯一发布者成立——结论同中期报告,依据按本口径重述。

**③ initialpose 协方差以实际发出值为准**
amcl_initpose.sh 只传 (x, y, yaw) 三参数,实际发出的 PoseWithCovarianceStamped 协方差为默认 **cov_xy=0.09(σ=0.3m)/ cov_yaw=0.03 rad²(σ≈9.9°)**。中期报告所写 0.0225(σ=0.15m)/0.0076 是**计划值,未实际发出**。另:粒子云平均散布(静止收敛期 567 粒 mean 0.326m / max 1.029m)是粒子几何散布统计,**不能直接等同单轴标准差**;后续位姿协方差以 /amcl_pose 消息内 covariance 字段为准(静止收敛期 x≈0.068~0.070,即 σ≈0.26m)。

**④ 离线激光—地图对齐=初始化候选,非精度声明**
align_scan.py(本目录归档)粗搜 x,y∈±0.6m(步 0.1)、yaw 全周(步 2°),精搜最优邻域 ±0.15m(步 0.02)/±3°(步 0.5°),评分=激光点落入[占据∪膨胀1格]比例。得到的是**初始化候选位姿 (-0.049, +0.070, -26.4°)**;协方差是初始化设置,未经独立测量,**不称"对齐精度 0.15m/5°"**。现场核对:车位于建图终点附近、未被挪动(用户确认),车头朝向与候选一致;候选有效性由四次 /initialpose 发布后 AMCL 收敛互证(见第二节),粗/精搜终端评分输出未归档,如实记录。

## 二、四.3~四.6 摘要(初始位姿与收敛)

四次 /initialpose(均以离线候选为依据,非 0,0,0)后 AMCL 收敛对照:

| 次 | 收敛位姿 (x, y, yaw) | 备注 |
|---|---|---|
| #1 | (-0.049, +0.070, -26.4°) | 与候选一致 |
| #2 | (-0.059, +0.074, -26.5°) | 重复发布复现 |
| #3 | (-0.032, +0.043, -26.8°) | watch_static 全程位姿 |
| #4 | (-0.034, +0.080, -26.7°) | stop→start 重启链后(watch_restart) |

收敛带宽:x 2.7cm / y 3.7cm / yaw 0.4°;渲染图 amcl_initial.png、amcl_restart.png 激光均贴墙;数值全有限;粒子云 567 粒 mean 散布 0.326m(max 1.029m)。

## 三、四.7 推车对账与 FAIL 判定

操作:用户沿车头方向缓推两段(每段约 0.3~0.5m,四轮着地自然滚动,人工测距为独立参考)。20:00 前用户另有一次计划外挪位(已确认为用户操作,odom 记账 0.877m),推前基线取于挪位静止后。

| 量 | 推前 | 推后 | 增量 |
|---|---|---|---|
| odom_combined→base_footprint(odom系) | [0.167, 0.861, -171.6°] | [-1.943, 0.252, -168.6°] | **位移 2.20m, yaw +3°** |
| map→odom_combined | [-0.101, -0.493, -25.6°] | [0.792, -0.377, +5.3°] | AMCL 修正 0.90m + 30.8° |
| AMCL 估计(map系,上两行复合) | [0.422, 0.211, 162.8°] | [-1.166, -0.304, -163.3°] | **位移 1.67m, yaw +33.9°** |
| 人工测距 | — | — | 两段共 **~0.6–1.0m** |

自洽性(AMCL 机制本身正常的证据):watch 末帧 (-1.174, -0.307, -163.3°) 与 TF 复合推算差 8mm;运动期 /amcl_pose 新帧 6 个(移动确实触发更新);推后静止 20s 窗 0 新帧;/scan 10.9Hz、/odom 18.2Hz 持续;数值全有限。

**FAIL 依据(按用户判据)**:位移不合理(odom 2.20m、AMCL 1.67m vs 人工 0.6–1.0m)+ 推后贴墙变差(push_post.png 成片红点落自由区;协方差 x 0.30→0.66,粒子散布 mean 0.326→0.819m / max 3.596m、粒子数 567→2000,发散未充分收敛)+ map→odom 30.8° 旋转修正量异常。不把旧位姿或现状判为稳定通过。

## 四、根因:轮式里程计速度比例系统性高估 ~2.2×

证据链:
1. EKF(ekf.yaml)位置无绝对源:odom0=encoder 仅融合 twist.vx(odom0_config 第7位),位置全由轮速积分 + IMU yaw(relative)演化;
2. encoder_node.py(本目录归档副本 encoder_node_READ_ONLY.py)第 121 行换算:`C=227.45/1000`(轮周长);`Vel = 3.57*angle_v*C/1024.0/0.02*k*0.25*1.03`——`0.25`(疑似减速比)与 `1.03` 为厂家手调常数,无本车实测依据;静止 vx 零偏=0、vy/vyaw 恒 0(排除零偏与串轴);
3. 两次独立人工推行与 odom 记账比均 ~2.2×(19:47 段 0.877m / 20:0x 段 2.20m,人工各段同量级)→ 系统性比例错,排除打滑(打滑不会两次等比例);
4. AMCL 表现与根因自洽:静止贴墙好(不依赖 odom);运动中虚增的预测被激光匹配部分拉回(2.20→1.67m)但拉不干净,粒子被带偏发散。

地图尺度不受影响(slam_toolbox 建图靠扫描匹配,轮速仅作初值预测)。

## 五、修复路径(未执行,择期)

1. 卷尺 1.00m 受控标定:推前采 odom → 沿车头推 1.00m → 推后采 odom,比例 = Δodom/1.00;
2. encoder_node.py:121 `*1.03` → `*1.03/比例`(单点修改,其余常数不动);单独重启 encoder_vel 节点(EKF/AMCL 链不重启,无需重发 initialpose);
3. 重做四.7:odom ≈ 人工 ≈ AMCL 且推后贴墙保持 → 该项才判 PASS。

## 六、证据清单(本目录)

| 文件 | 内容 |
|---|---|
| amcl_initial.png / amcl_restart.png | 初始收敛/重启后 激光-地图叠加(贴墙) |
| push_pre.png / push_post.png | 推前/推后叠加(推后贴墙变差) |
| watch_static.txt / watch_restart.txt | 静止 60s / 重启后 15s 采样 |
| watch_push.txt | 推车 240s 窗(运动触发更新+发散记录) |
| check_20260404_192052 / 194708.txt | 只读核对(生命周期/运动话题零发布/TF 发布者) |
| align_scan.py | 离线对齐推算脚本(搜索范围见口径④) |
| encoder_node_READ_ONLY.py | 里程计换算源码副本(根因证据,只读) |
| (车机 logs/amcl/) | runcar/loc/monitor 全量日志(16 文件,另存 D:\Claude\projects\hwcart_amcl_stage2\logs_amcl) |

## 七、异常与坑记录

- fastrtps SHM 坏段残留(open_and_lock_file failed):ros2 service call 挂起、topic info -v 超时;话题层数据流正常,绕行=直接 topic pub/echo(服务不走);
- /amcl_pose 为 transient_local+reliable(latch),订阅建立即补发最后一帧,**stamp 可落后 ~159s,勿当漂移读**;/particle_cloud 为 SensorDataQoS 仅滤波更新瞬间发布;nav2 humble ParticleCloud 字段=particles[](每粒含 pose+weight),非 poses+顶层 weights;
- scp 回本地后文件 mtime 显示不可信,以字节数判新旧;
- Bash `A && B & C` 中 `&` 会把 `cd && (...)` 整体后台化,前台相对路径必炸(用分号/绝对路径)。
