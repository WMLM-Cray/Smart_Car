# 2026-09-16 实车传感器参数对齐验证

任务书要点:实车数据已确认(雷达发布 11.99Hz、每帧 1667 点、IMU 100Hz),
仿真按目标值对齐并验证。**仿真目标取整 12Hz,不为复现测量小数设 11.99。**

## 配置对照(本轮切换)

| 参数 | 旧值(2026-09-16 前) | 新值 | 依据 |
|---|---|---|---|
| 雷达 update_rate | 10 Hz | **12 Hz** | 实车实测 11.99Hz → 仿真目标取整 |
| 雷达 samples | 2000 点/帧 | **1667 点/帧** | 实车确认 |
| IMU update_rate | 200 Hz | **100 Hz** | 实车确认(旧值 200 为假设) |

修改唯一来源:`racecar_sim/config/params.yaml`(xacro 经 `${cfg[...]}`
引用,改 yaml 全局生效);同步更新 `tools/verify_phase1.sh` 频率判据
(12±1.2 / 100±10)与 `amcl.yaml` 注释。**本轮未改**:赛道、车辆几何、
相机、Nav2 调参、间隙阈值(0.05 保留)。

## 历史证据归属(不替换)

- `docs/validation/2026-09-15/`、`docs/track/nav_*`(含 R12 导航终验)、
  `docs/track/trackA_slam_smoke.pgm/.yaml` —— 全部产生于**旧配置
  (10Hz/2000 点/IMU200Hz)**,原样保留,本轮未重跑导航调参。
- 本轮新 SLAM 冒烟产物:`docs/track/trackA_slam_smoke_20260916_12hz1667.pgm/.yaml`。

## 本轮证据

| 文件 | 内容 |
|---|---|
| `sensor_align_report.json` | 30.1s 仿真时间采样验收报告(PASS) |
| `sensor_sim_运行日志.txt` | 仿真启动完整日志(.gitignore 拦 *.log,故 .txt 入库) |
| `slam_smoke_运行输出.txt` | 新参数下 SLAM 冒烟(PASS,2.37m 含转弯畸变样本) |

## 实测结果(2026-09-16,冷启动)

- /scan:**时间戳频率 11.905Hz**(判据 12±1.2 ✓);接收端 11.713Hz;
  实时因子 0.984(接收端偏低=运行速度,非传感器问题);359 帧
  **ranges 长度全为 1667**;时间戳严格递增;角度自洽
  (increment=0.003771417 rad=2π/1666,端点 ±π 含两端);
  frame_id 恒为 laser_sensor_link;话题 /scan 不变。
- /IMU_data:**时间戳频率 100.0Hz**(判据 100±10 ✓);3008 条;
  时间戳严格递增;frame_id 恒为 imu_sensor_link;话题不变。
- SLAM 冒烟:/map 335×116@0.05m,free=7436/occ=275,判据全 PASS。
  (仅证明新参数下建图链正常,不构成完整闭环导航验收。)

## 尚未逐项对齐(记录项)

- 实车消息的 angle_min/angle_max/angle_increment/scan_time/
  time_increment 数值未取得——本轮仅对齐频率与点数,角度端点仍为
  仿真默认 ±π 含两端。
- Gazebo classic 限制:无逐点时间戳与运动畸变(瞬时整帧扫描);
  本轮实测 scan_time=0.0、time_increment=0.0(插件不填),与实车
  (通常非零)不同属已知差异,待实车字段取得后评估影响。

## 复验入口(WSL,预期 rc=0 / PASS)

```bash
bash tools/sensor_align_run.sh        # 30s 仿真时间采样验收
bash tools/trackA/slam_smoke.sh       # SLAM 冒烟(注意会覆盖
                                      # docs/track/trackA_slam_smoke.pgm)
```
