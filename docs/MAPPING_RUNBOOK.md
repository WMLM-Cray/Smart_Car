# 遥控建图现场步骤

目标:人开网页遥控,车在走廊里跑一趟,存出一张 pgm 地图。不涉及策略,不涉及 Gazebo。

前提(来自 docs/SOURCE_AUDIT.md,已核对过源码):
- `racecar/config/ekf.yaml` 用 `/encoder` + `/IMU_data` 融合,输出 TF `odom_combined → base_footprint`
- `racecar/launch/ekf.launch.py` 把 `/odometry/filtered` 重映射成 `odom_combined`
- 雷达话题 `/scan`,frame `laser_link`,直接挂 `base_footprint`,z=0

所以建图需要的三样(scan / odom TF / base frame)厂家都给了,**只差一个 SLAM 节点**。

---

## 0. 环境(2026-09-19 在车上实测,不用再查)

`slam_toolbox` 和 `nav2_map_server` 都已安装,**不需要写代码,也不需要 apt**。

## 1. 先确认只有一份底盘在跑

```bash
ros2 node list | sort | uniq -c | sort -rn | head
```

**每个节点的计数必须是 1。** 出现 2 说明 `Run_car.launch.py` 跑了两份 ——
底盘现在是开机自启的(commit fd797e9),再手工 `ros2 launch` 一次就会重复。
后果是两个 `ekf_filter_node` 同时往 TF 里发 `odom_combined → base_footprint`
互相打架,SLAM 吃到的位姿是两份的混合;两个 `racecar_driver` 同时往电调写 PWM。
2026-09-19 现场就撞上过这个。

重复了就杀掉**手工那份**(systemd 那份的 PID 用 `systemctl show racecar-chassis -p MainPID` 查,留它):

```bash
ps -ef | grep "ros2 launch racecar" | grep -v grep
```

## 2. 确认传感器和 TF

```bash
ros2 run tf2_ros tf2_echo odom_combined base_footprint
```

要能持续刷出数、推着车走数值要变。**这一步不过就别往下走** —— 没有 odom 的 SLAM 在走廊里必挂。
如果没输出,EKF 没起来:`ros2 launch racecar ekf.launch.py`。

实测基准(对不上说明有东西没起来):`/scan` 11.97 Hz,`/encoder` 50.6 Hz。

## 3. 起 SLAM

```bash
ros2 run slam_toolbox async_slam_toolbox_node --ros-args -p odom_frame:=odom_combined -p base_frame:=base_footprint -p map_frame:=map -p scan_topic:=/scan -p resolution:=0.05 -p max_laser_range:=12.0 -p minimum_travel_distance:=0.2
```

只改了和厂家 frame 名对齐的几个参数,其余走默认值,走廊够用。

确认它真的在建图 —— **要同时满足两条**:

```bash
ros2 topic hz /map
```

```bash
ros2 run tf2_ros tf2_echo map odom_combined
```

只有 `/map` 话题、没有 `map → odom_combined` 这个 TF,说明 SLAM 没在定位,
地图是死的(2026-09-19 就见过这个状态)。frame 名写错时不会报错,只会安静地不工作。

## 4. 开车

网页遥控已自启,浏览器开 `http://192.168.1.238:8080`。

走廊建图的三条注意事项(都是会直接毁掉地图的):

1. **慢**。用 Sloth 档。scan 12 Hz,开快了两帧之间转过的角度太大,扫描匹配对不上。
2. **别贴着一面墙直着开很久**。长直走廊是扫描匹配的经典病例 —— 沿走廊方向没有特征,地图会沿轴向被拉长或压扁。经过门口、拐角、消防栓这类特征时慢一点,让它吃到约束。
3. **回到起点**。绕回出发位置能触发回环检测,把累积漂移拉回去。只单程走一趟,地图末端的误差没有任何东西能修正。

## 5. 存图

```bash
ros2 run nav2_map_server map_saver_cli -f ~/corridor
```

生成 `~/corridor.pgm` + `~/corridor.yaml`(`nav2_map_server` 车上已装)。
也可以走 slam_toolbox 自己的服务,效果一样:

```bash
ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap "{name: {data: corridor}}"
```

想之后还能接着建图再补一个序列化:

```bash
ros2 service call /slam_toolbox/serialize_map slam_toolbox/srv/SerializePoseGraph "{filename: corridor}"
```

拿回本地看:

```bash
scp orange-pi@192.168.1.238:~/corridor.pgm .
```

## 6. 想实时看

笔记本上 `rviz2`,加 Map 显示订阅 `/map`,Fixed Frame 设 `map`。
前提是笔记本和车 `ROS_DOMAIN_ID` 一致,否则什么都看不到。

---

## 现场要顺手量的(上车前一直缺的数)

在走廊里反正要开,顺便把这两个数量了,仿真里它们现在都是 `[假设]`,而且正好卡在影响结论的区间:

- **满油门实际速度**:直走廊里给满油门,`ros2 topic echo /encoder` 读稳态值 → `params.py` 的 `v_fwd_full`(现在假设 3.0)
- **制动减速度**:跑起来后把 PWM 打到 1500,记录从稳态到停的时间和距离 → `accel_max`(现在假设 3.0)

轴距用卷尺量一下也就一分钟,`wheelbase` 现在是队友从别的版本代码抄的 0.305。
