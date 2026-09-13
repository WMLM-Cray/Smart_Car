# 一手资料核对

核对版本：racecar_05_27 解压源码；比赛规则 2026 年 8 月初稿（本地文件标注 2026-09-13 下载）。相对路径均指厂家工作区 src；文件摘要见 source_hashes.json。本记录不声称核对了后续规则更新。

| 项目 | 实际证据 | 实施影响 |
|---|---|---|
| 主驱动命令 | racecar_driver/src/racecar_driver_node.cpp:31-46 订阅 /car_cmd_vel；另一版本 node_one 才订阅 /cmd_vel | 保留策略 /cmd_vel → /car_cmd_vel 层，不可只按话题名称判断语义 |
| 转向与油门 | 同文件:45-46，linear.x×100+1500；1500−angular.z×1300 | PWM 比例不证明单位是 m/s 或 rad/s；需要标定，不能直接套标准 Twist 阿克曼插件 |
| TF | racecar/launch/Run_car.launch.py:98/104/110 | 三个子 frame 均直接挂 base_footprint；laser_link 与 IMU_link 的 z 确为 0 |
| EKF frame/input | racecar/config/ekf.yaml:23-30/57 | odom_combined → base_footprint；输入 encoder 与 IMU_data |
| EKF output | racecar/launch/ekf.launch.py，两处 remappings | /odometry/filtered 重映射到 odom_combined（话题与 frame 同名） |
| 摄像头 | racecar/launch/camer.launch.py:20-24 | 实际入口 320×240、yuyv、camera；没有加载 params_1.yaml，帧率未显式指定 |
| 相机备选配置 | racecar/config/camera/params_1.yaml | 640×480@10fps 是独立配置，不能视为实际 launch 生效值；图像话题需运行确认 |
| 雷达 | lslidar_driver/config/lslidar_x10.yaml | scan、lslidar_point_cloud、laser_link、最小 0.15m、软件裁剪最大 50m；硬件 25–30m 需区分 |

相机标称 150° 未说明是水平还是对角视角，仿真 horizontal_fov 暂不固化。0.22°、360° 和 20000 次/s 不是可以同时照抄为整数扫描点数的严格等式，需抓实时驱动输出核对（仅开发用途）。不静默修正厂家外参，也不使用错误 frame 标记抬高后的传感器。

## 规则差异与图形检查

- PDF 第 3 页 §2.1：两圈过程中每次遇到红灯均需停车，不能仅在冲刺圈启用。
- 第 8 页 §2.4.2(1)：红灯停车时任一车轮不在停车区域内 +20 秒；未停车 +50 秒；绿灯前通行 +30 秒。
- 第 8 页 §2.4.2(3)：压到锥桶支撑边角而锥桶没有位移，不加罚；仅检测接触不足以判撞锥。
- 第 8 页 §2.4.2(4)：折线压线按单圈处罚，不按接触帧累加。
- 第 8 页地图条款：模糊但位置正确或丢失两个及以下挡板和锥桶的情况与无效地图不同，不直接简化为“少一个挡板 +50 秒”。
- 第 4 页：实际赛道可以是四种样式的一个或多个，不能假定只抽取单一模板。
- 第 5 页交通灯摆位条款同时涉及右侧与上方居中安装，需要结合道具示意确认几何；未自行定案。
- 已渲染并查看第 11–12 页图 9：可读总横向标注 16.8m、宽 1.5m、局部 1.25m、4m、3.06m 及 90°/135°/45°。这不等于已完成锥桶坐标重建，A-D 精确模板仍待实现。

## 必须补齐的实车参数

轴距、前后轮距、轮径、轮宽、质量、惯量、舵角限位、速度和转角的 PWM 标定曲线、相机位姿和内参/畸变、雷达实际离地高度。560×350×230mm 只是外形尺寸，不能代替轴距。
