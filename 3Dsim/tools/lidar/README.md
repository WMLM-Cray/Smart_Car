# 雷达数据采集与分析工具

本目录用于采集、解析和查看小车激光雷达 `/scan` 数据。

工具可以从 Orange Pi 小车上录制 ROS 2 bag，下载到本机后转换为 NumPy 数据、生成可视化图，并对多次采集结果进行对比。目录中的 `scan1`–`scan4` 是已经采集好的示例数据，可直接用于测试分析流程。

## 工具一览

| 文件 | 用途 |
| --- | --- |
| `car_ssh.py` | 通过 SSH 在小车上执行命令，只读地转发输出 |
| `grab_scan.py` | 录制、下载、校验、解析并可视化一份雷达数据 |
| `parse_bag.py` | 从 ROS 2 `.db3` 文件中解析 `/scan`，导出统计信息、`.npz` 或 `.csv` |
| `view_scan.py` | 查看单次采集的极坐标图、笛卡尔图、瀑布图和诊断图 |
| `compare_scans.py` | 对比多次采集的帧数、频率、有效点数和时间抖动 |
| `scan1/`–`scan4/` | 示例 rosbag、解析结果和可视化结果 |
| `.car_env.example` | SSH 连接配置模板，不含真实凭据 |

## 环境要求

本地分析需要 Python 3.10 或更高版本，以及：

```bash
pip install numpy matplotlib
```

只有从小车采集数据时才需要 SSH 依赖：

```bash
pip install paramiko
```

采集功能还要求小车上已经运行 ROS 2 和雷达驱动，并且本机可以通过 SSH 连接小车。

## 查看仓库中的示例数据

在本目录执行：

```bash
# 查看一份数据，输出四联图
python view_scan.py scan1/scan.npz --out scan1/overview.png

# 查看指定帧
python view_scan.py scan1/scan.npz --frame 100

# 对比四次采集
python compare_scans.py --dir scan1 scan2 scan3 scan4 --out compare_scans.png
```

也可以直接读取原始 `.db3` 文件：

```bash
python parse_bag.py scan1/scan1_0.db3
python view_scan.py scan1/scan1_0.db3
```

## 解析 rosbag

打印统计信息：

```bash
python parse_bag.py scan1/scan1_0.db3
```

导出 NumPy 文件。由于脚本把输出写到当前工作目录，建议进入目标目录后执行：

```bash
cd scan1
python ../parse_bag.py scan1_0.db3 --npz
```

导出逐点 CSV：

```bash
cd scan1
python ../parse_bag.py scan1_0.db3 --csv
```

CSV 是每个点一行，数据量可能很大，不建议直接用 Excel 打开完整文件。

## 从小车采集新数据

1. 复制配置模板并填写连接信息：

   ```bash
   cp .car_env.example .car_env
   ```

2. 编辑 `.car_env`：

   ```text
   CAR_HOST=小车IP
   CAR_USER=orange-pi
   CAR_PASS=小车密码
   # CAR_PORT=22
   # CAR_KEY=~/.ssh/id_rsa
   ```

   也可以通过环境变量提供这些配置。`.car_env` 只保存在本地，**不要提交到 Git**。

3. 录制一份数据：

   ```bash
   python grab_scan.py                 # 默认录制 20 秒
   python grab_scan.py -d 30           # 录制 30 秒
   python grab_scan.py -n cone_test    # 指定输出目录
   python grab_scan.py --no-plot       # 不生成图片
   ```

`grab_scan.py` 会在录制前检查驱动进程、串口和 `/scan` 数据流，随后完成下载、MD5 校验、解析和出图。默认会同时录制 `/scan`、`/tf`、`/tf_static`、`/IMU_data`、`/encoder` 等相关话题；如只需雷达数据，可以指定：

```bash
python grab_scan.py --topics /scan
```

SSH 底层工具也可以单独使用：

```bash
python car_ssh.py "ros2 topic list"
python car_ssh.py -t "top -b -n1"
```

## 数据格式

每个采集目录通常包含：

```text
scanN/
├── scanN_0.db3   # 原始 ROS 2 bag 数据
├── metadata.yaml # rosbag 元信息
├── scan.npz      # parse_bag.py 导出的数组
└── overview.png  # view_scan.py 生成的总览图
```

`.npz` 中主要包含：

- `ranges`：形状为 `(帧数, 点数)` 的距离数组，单位为米
- `stamp`：雷达消息中的采样时间
- `bag_time`：消息写入 rosbag 的时间
- `angle_min`、`angle_increment`：角度参数
- `range_min`、`range_max`：量程参数
- `frame_id`：雷达坐标系名称

## 当前实测参数

示例数据来自镭神 M10P 雷达：

| 参数 | 数值 |
| --- | --- |
| 点数/帧 | 1667 |
| 频率 | 约 12 Hz |
| 量程 | 0.15–50 m |
| `frame_id` | `laser_link` |
| 角度范围 | `-π` 到 `+π` |

分析数据时请注意：

- 没有回波的方向通常是 `inf`，不是 0；计算统计量前应使用 `np.isfinite()` 过滤。
- `bag_time` 反映录制端收到消息的时间，测量雷达自身节奏时应优先参考 `header.stamp`（导出的 `stamp`）。
- `-π` 和 `+π` 表示同一个方向，绘图或计算时注意不要把首尾方向重复计算。

## 目录位置

本 README 与工具位于仓库的：

```text
3Dsim/tools/lidar/
```

工具脚本之间通过相对位置调用，因此从本目录执行命令最简单；从其他目录运行时请使用脚本的完整路径。
