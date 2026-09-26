# 雷达数据工具

本目录采集和分析 ROS 2 `/scan`。运行采集前确认车端只有预期的 ROS 节点；分析工具可在本机运行，不需要连车。

## 工具

| 文件 | 用途 |
|---|---|
| `grab_scan.py` | 采集并下载一份 rosbag，校验后解析和画图 |
| `parse_bag.py` | `.db3` 转 `.npz` / `.csv` 与统计 |
| `view_scan.py` | 单次数据的极坐标、平面和诊断图 |
| `compare_scans.py` | 对比帧率、有效点和时间抖动 |
| `car_ssh.py` | 只读 SSH 命令转发 |
| `.car_env.example` | 配置模板；填写副本，勿提交密码 |

## 使用

Python 3.10+，需安装 `numpy`、`matplotlib` 和 ROS 2 bag 解析依赖。命令参数以各脚本 `--help` 为准：

```bash
python parse_bag.py capture.db3 --npz
python view_scan.py scan.npz --out scan.png
python compare_scans.py run1.npz run2.npz --out compare.png
```

采集脚本依赖本机 SSH 配置和车端 ROS 2 环境：

```bash
python grab_scan.py --help
```

真实连接配置只放本地未跟踪文件，禁止把密码、私钥或车端口令写入仓库。精简版已移除旧示例录包；有需要可从整理前提交恢复。

当前车辆雷达约 12 Hz、每帧 1667 点，配置值不代替现场帧率检查。采集日期、场景、电压和车体状态应随结果记录。
