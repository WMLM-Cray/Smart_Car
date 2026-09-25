# 2026 规则赛道 A–D：当前数据状态

规则来源：`赛项一-室外ROS无人车赛（高教组）比赛规则（初稿）.pdf` 第 4 页图 3 和第 11–12 页图 9。规则文字规定赛道宽度 **1.5 m**，相邻锥桶间距 **小于赛道宽度**；没有统一规定 0.5 m 或 0.4 m 的排桶数值。

| 赛道 | 当前交付 | 状态 |
| --- | --- | --- |
| A | `trackA_layout.yaml`、`trackA_cones.csv`、`trackA_topview.png`、`trackA_fig9_cone_overlay.png`，并同步 `3Dsim/examples/trackA_layout.yaml` | 对图 9A 中每一个锥桶逐点读图：36 蓝 / 35 红，未按均匀间距添加锥桶。黄色 V 胶带区不放锥桶。像素坐标由图示 16.8 m 总长换算为米；旧边界几何尚未与这些位置校准。 |
| B | `track_bcd_preview/trackB_layout.json`、`trackB_cones.csv`、PNG、SVG | 按图 9 估读的边界和锥桶示意数据，尚未完成尺寸、道宽和全圈仿真验收。 |
| C | `track_bcd_preview/trackC_layout.json`、`trackC_cones.csv`、PNG、SVG | 同上；左上为直角，下侧为 S 弯。 |
| D | `track_bcd_preview/trackD_layout.json`、`trackD_cones.csv`、PNG、SVG | 同上；左端圆弧，下侧为 S 弯。 |

**使用限制：**A 的 71 个锥桶在图上可逐点对照，但仍是图像估读值，不是现场测量坐标；其 YAML 中的旧边界段和发车姿态尚未按这 71 个锥桶完成校准，不应直接作为验收通过的可行驶赛道。相邻常规桶距蓝 0.480–1.475 m、红 0.452–1.145 m，黄色 V 断口单独排除。B–D JSON/CSV 的米制坐标从渲染图估读，图上未标的曲线控制点没有官方尺寸约束。抽样后的局部最近边界距离只有约 1.20–1.23 m，不能宣称其全程满足标称 1.5 m，也不能直接用于 Gazebo/FastSim 的最终通行率评测。A 的旧几何道宽疑点见 `trackA_fig9_尺寸归属报告.md`。

**复现：**

```bash
python 3Dsim/tools/trackA/trackA_geometry.py
python docs/track/draw_track_bcd.py
python -m unittest 3Dsim/tools/trackA/test_cone_density.py -v
```

完成 A–D 官方尺寸几何、坐标生成和仿真验收的剩余工作由 [issue #2](https://github.com/FZU-ZCC-ADS-Lab/Auto_Car/issues/2) 跟踪。
