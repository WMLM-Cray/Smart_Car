# 小车雷达数据工具

从 Orange Pi 小车上采集激光雷达 `/scan` 数据，传回本机分析、可视化。

配套的 `Auto_Car-main/` 是比赛项目仓库（`matXn/Auto_Car`）的副本，本项目不修改它，
只从车上取实测数据。

---

## 快速开始

```bash
# 1. 填连接信息(见下节)
# 2. 一键采集 20 秒
python grab_scan.py

# 3. 看结果
#    输出在 scan_<时间戳>/overview.png
```

---

## 连接配置

复制模板填到同目录的 `.car_env`：

```
CAR_HOST=192.168.1.238      # 车的 IP
CAR_USER=orange-pi          # 登录用户名
CAR_PASS=你的密码
# CAR_PORT=22               # 可选
# CAR_KEY=~/.ssh/id_rsa     # 可选,用密钥代替密码
```

**这个文件含密码，不要提交到 git、不要发群里。**

车有两个网段：自开热点时是 `192.168.5.100`，连路由器时是路由器分配的地址
（在车上跑 `ip a` 可查）。

---

## 脚本

### `grab_scan.py` —— 一键采集（常用）

录 → 传 → 校验 → 解析 → 出图 → 清理车上，一条龙。

```bash
python grab_scan.py                       # 录 20 秒
python grab_scan.py -d 30                 # 录 30 秒
python grab_scan.py -n cone_test          # 指定输出目录名
python grab_scan.py --no-plot             # 只取数据不画图
python grab_scan.py --topics /scan        # 只录指定话题
python grab_scan.py --no-health           # 跳过健康检查
```

默认录这些话题：`/scan`、`/lslidar_point_cloud`、`/tf`、`/tf_static`、
`/IMU_data`、`/encoder`、`/odom_combined`。

**录制前会自动做雷达健康检查** —— 检查驱动握着的串口 fd 是否有效、`/scan`
是否真有数据流、车是否静止。检查不通过会直接报错并给出恢复命令，不会白录一份空数据。

### `parse_bag.py` —— 解析 rosbag

自实现 CDR 反序列化，**不需要装 `rosbags`**（本机没装，也不想为了这个联网装）。

```bash
python parse_bag.py scan1/scan1_0.db3            # 打印统计摘要
python parse_bag.py scan1/scan1_0.db3 --npz      # 导出 scan.npz
python parse_bag.py scan1/scan1_0.db3 --csv      # 导出 scan.csv(逐点,文件大)
```

⚠️ `--csv` 是**逐点一行**，20 秒数据约 40 万行，Excel 打开会很慢。想用 Excel 建议先抽帧。

### `view_scan.py` —— 单次数据可视化

一次出四张图：极坐标、笛卡尔、时间-角度瀑布、诊断。

```bash
python view_scan.py scan1/scan.npz                     # 出 overview.png
python view_scan.py scan1/scan.npz --frame 100         # 看第 100 帧
python view_scan.py scan1/scan.npz --show              # 弹窗(可缩放拖动)
python view_scan.py scan1/scan1_0.db3                  # 也能直接吃 .db3
```

### `compare_scans.py` —— 多次数据对比

```bash
python compare_scans.py --dir scan1 scan2 scan3 scan4      # 对比四份
python compare_scans.py scan1/scan.npz scan2/scan.npz      # 也可以直接给文件
```

出指标表 + 三张图（有效点数曲线、帧间隔箱线图、抖动对比）。

### `car_ssh.py` —— SSH 执行器（底层）

在车上跑一条命令，输出直接流到本机。其他脚本都调它。

```bash
python car_ssh.py "ros2 topic list"
python car_ssh.py -t "top -b -n1"        # 强制分配 tty
```

---

## 目录结构

```
.
├── .car_env              连接信息(含密码,勿外传)
├── car_ssh.py            SSH 执行器
├── grab_scan.py          一键采集
├── parse_bag.py          解析 .db3
├── view_scan.py          单次可视化
├── compare_scans.py      多次对比
├── compare_scans.png     对比结果
├── Auto_Car-main/        比赛项目仓库副本(只读参考)
└── scan1/ scan2/ scan3/ scan4/
    ├── scanN_0.db3       原始 rosbag
    ├── metadata.yaml     rosbag 元信息
    ├── scan.npz          解析结果
    └── overview.png      可视化
```

---

## 雷达参数（实车实测）

车上的雷达是**镭神 M10P**（配置在 `/home/orange-pi/racecar/install/lslidar_driver/
share/lslidar_driver/config/lslidar_x10.yaml`）。

| 项 | 实测值 |
|---|---|
| 点数/帧 | **1667** |
| `angle_increment` | **0.0037691575** rad（= 2π/1667）|
| `angle_min` / `angle_max` | **-π / +π**（闭区间）|
| 频率 | **12.00 Hz** |
| `range_min` / `range_max` | **0.15 m / 50.0 m** |
| `frame_id` | **`laser_link`** |
| `scan_time` / `time_increment` | **0.0**（驱动未填充）|

---

## 四个必须知道的坑

### 1. `inf` 不是 0

没打到的方向是 `inf`（超 50m），不是 0 也不是 NaN。算均值前必须过滤：

```python
ok = np.isfinite(R)
print(R[ok].mean())
```

实测数据里约 **15% 是 `inf`**。

### 2. 角度是 ±π 闭区间

`angle_max - angle_min = 2π`，所以 **`-π` 和 `+π` 指向同一个方向**，有重复射线。
画散点时要只取一半，否则最后一点会叠在第一条上：

```python
half = angles < np.pi - 1e-9
```

### 3. `frame_id` 是 `laser_link`（z=0），但雷达实际装在 0.21m 高

转 XY 做俯视图没问题，但算三维点云或做 TF 变换时**得自己补上高度差**。

注意这跟 Gazebo 仿真不一致：仿真的 `racecar.xacro` 刻意新增了
`laser_sensor_link`（真实高度）并让 `/scan` 用它，以避免"扫描位置抬高、
消息仍标零高度"的错误 —— 但**实车用的就是 z=0 的厂家 frame**。

### 4. `bag_time` 和 `header.stamp` 是两回事

| | 含义 | 用途 |
|---|---|---|
| `bag_time` | 消息**到达录制端**的时刻 | 含调度抖动；看数据流什么时候到 |
| `header.stamp` | 雷达**采样**的时刻 | 反映传感器真实节奏 |

**测频率、算运动畸变、做时间同步都该用 `header.stamp`。**

实测对比（`scan3` 那次）：

| 时间源 | 间隔中位 | 最大间隔 | 抖动 σ |
|---|---|---|---|
| `header.stamp` | 83.38 ms | 85.47 ms | **0.67 ms** |
| `bag_time` | 83.4 ms | **386.2 ms** | **29.63 ms** |

`bag_time` 看起来像丢了 3 帧，`header.stamp` 一看就知道**一帧没丢** ——
是 `rosbag2_recorder` 进程自己卡了两下（录制开头建库/订阅发现）。
**别误判成雷达故障。**

---

## 常见问题

### 雷达没数据（话题在、进程在、但收不到）

**这是最常遇到的故障。** 症状：`ros2 topic hz /scan` 报
"topic does not appear to be published yet"。

原因：USB 串口掉线重连后**设备号变了**（`ttyACM0` → `ttyACM1`），
但驱动还攥着那个已删除的 fd：

```
/proc/<pid>/fd/29 -> /dev/ttyACM0 (deleted)
```

恢复：

```bash
ssh 进车
sudo systemctl restart racecar-chassis.service
```

车上有 `/etc/sudoers.d/racecar-systemctl` 的 NOPASSWD 规则，**不用输密码**。

`grab_scan.py` 的健康检查会自动发现这个问题并给出提示。

### 车连不上

先确认和车在同一网段：

```bash
ping 192.168.1.238
```

不通的话检查车是不是换了网段（开热点 = `192.168.5.100`）。

### 图里中文显示成方块

脚本的图标题用的是英文，避免字体问题。命令行输出中文乱码的话加环境变量：

```bash
PYTHONIOENCODING=utf-8 python grab_scan.py
```

### 修改脚本后行为没变

删掉 `__pycache__/` 再跑。

---

## 相关文档

比赛项目仓库里的文档（在 `Auto_Car-main/docs/`）：

- `SIM_INTEROP.md` —— 快仿真与 Gazebo 仿真的对接口径，含待统一的参数表
- `SOURCE_AUDIT.md` —— 厂家源码与比赛规则的一手资料核对
- `TELEOP_INTEGRATION_GUIDE.md` —— 手柄遥控包对接文档

本项目实测数据**印证了** `SIM_INTEROP.md` 里"实车实测"那一列的全部数值
（1667 点、11.99 Hz、软件裁剪 50m），并补充了该表未记录的三项：
`scan_time`/`time_increment` 为 0、`frame_id` 用 z=0 的 `laser_link`、
`angle_min/max` 是 ±π 闭区间。

---

## 待办

- [ ] `fastsim/params.py` 的 `lidar_sigma = 0.01` 是 `[假设]` 值，
      实测底噪只有 **4.1–5.3 mm**，比假设值小一半 —— 可用这批数据校核
- [ ] 查 USB 掉线根因（换 USB 口 / 加 udev 规则按序列号固定软链）
- [ ] bag 里的 `/IMU_data`、`/encoder`、`/tf` 还没解析
