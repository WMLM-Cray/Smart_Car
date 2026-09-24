> 历史参考：本文件保留写作时的记录；当前参数和状态见 [项目文档入口](../README.md)。

# 现场故障诊断数据需求清单（2026-09-20）

> 目的：**不预设根因**——CRC 错、gate 拒帧、时钟漂移都只是候选，不是答案。
> 按「指令 → 动作」链路逐层定位**异常最早出现层**，用数据说话，缺哪层补哪层。

## 0. 故障链路与定位思路

单目标导航链路：

```
RViz /goal_pose → bt_navigator(规划) → controller_server(/cmd_vel)
  → cmd_vel_safety(限幅/超时) → /car_cmd_vel → racecar_driver_node → /dev/car → 电调/舵机
传感/定位旁路：
  encoder → /encoder_gate → EKF(odom) → AMCL(定位)
  IMU → EKF(imu)
  LiDAR /scan → (scan_filter? 未纳入默认链) → AMCL + 双 costmap
  时钟：use_sim_time + TF 时间戳对齐
```

从**动作端往上**找断裂点：
- **车不动** → driver 没收到指令？safety 截断了？controller 没发？没规划？没定位？
- **车乱走/漂** → odom 假数据（encoder/gate/EKF）？AMCL 定位错（scan/IMU/TF）？

## 1. 四类必须回传的数据

### A. 实际启动命令和配置
1. 现场**实际敲的启动命令原文**（含全部环境变量前缀，例 `GATE_VERSION=v2 MAX_LINEAR=0.30 bash safety_nav_one.sh map1.yaml`）。
2. 用到的 `nav_safe.yaml`、`ekf.yaml` **全文**（或 `md5sum` + 关键字段）。
3. `env | sort` 里 ROS/项目相关变量（`ROS_DOMAIN_ID`、`RMW_IMPLEMENTATION`、`CYCLONEDDS_URI` 等）。
4. `scan_filter.py` 有没有在跑？怎么启动的（`python3 scan_filter.py`？挂在哪个脚本）？

### B. encoder / gate / IMU / EKF 同步数据
- **带时间戳的对齐样本**，不是各自单独截一段。
- encoder 原始帧（hex）+ 解析值 + 时间戳。
- gate 输入/输出（哪帧进、哪帧出、是否被拒）+ 时间戳。
- IMU 原始消息 + 时间戳。
- EKF 输出 odom（pose/twist/covariance）+ 时间戳。
- 最好 `ros2 bag record /encoder /encoder_gate /imu /odom /tf` 同时录，回传 bag 或 CSV。

### C. use_sim_time 与 TF 完整时间错误
1. 每个节点 `use_sim_time` 实际值：`ros2 param get /<node> use_sim_time`。
2. `ros2 topic echo /tf` **完整**输出（别截断，看每个 transform 的 `header.stamp`）。
3. `ros2 topic echo /clock`（若有）——判断时钟源是谁。
4. TF 报错**原文**（如 "Lookup would require extrapolation into the future/past"）。
5. 关键 topic 的 `ros2 topic hz` + stamp 是否单调递增（有无跳变/回退）。

### D. 发布者和运行版本
1. `ros2 node list` + 关键节点 `ros2 node info /<node>`（订阅/发布关系）。
2. `ros2 topic list -t`：每个 topic 的**发布者/订阅者数量**，尤其 `/scan_filtered`、`/encoder`、`/odom`、`/cmd_vel`、`/car_cmd_vel`（谁在发、有没有孤儿话题）。
3. 运行版本：`dpkg -l | grep ros-humble` 或 `rosversion`；driver / encoder 包版本、最近一次 `colcon build` 时间与产物路径。
4. `ros2 doctor` 输出。

## 2. 定位判据（按层，从动作端往上）

| 层 | 检查 | 异常 → 说明 |
|---|---|---|
| 指令源 | `/goal_pose` 是否发出、数值是否合理 | 没发/发错 → 问题在 RViz 或操作 |
| 规划 | `bt_navigator` 是否出 `/plan` | 规划失败 → map/scan/定位问题 |
| 控制 | `controller_server` 是否发 `/cmd_vel`、数值 | cmd_vel=0 或方向错 → 规划/反馈问题 |
| 安全 | `cmd_vel_safety` 是否放行 `/car_cmd_vel` | 被限幅/超时零速 → 看 safety 日志 |
| 执行 | driver 是否收 `/car_cmd_vel`、是否写 `/dev/car`、PWM | 收不到/PWM 不对 → driver/电调问题 |
| 传感 | encoder 回帧？gate 过滤？EKF 出 odom？ | odom 漂/假数据 → 这层 |
| 定位 | AMCL 位姿是否收敛 | 不收敛 → scan/IMU/TF 时钟问题 |

## 3. 回传格式约定
- 尽量**原样贴命令/日志**，不要口述转述（转述会丢字节级信息）。
- 每条数据**带时间戳**，说明"哪一刻车在做什么"。
- 先一句话说**现象**（车不动 / 乱走 / 碰撞 / 漂移），再贴对应时段的数据。
