# A–D 赛道数据状态

规则图规定赛道宽 1.5 m，相邻桶距小于赛道宽；图上标注的尺寸才按米制约束。根目录 [`maps/`](../../maps/) 是数据入口。

| 赛道 | 数据 | 状态 |
|---|---|---|
| A | `trackA_layout.yaml`、`trackA_cones.csv`、`trackA.png` | 71 桶逐点按图估读（蓝36、红35）；黄色 V 区无桶。总长按图示 16.8 m 标定，边界几何尚未和桶位完全校准。 |
| B–D | 各自 `trackX_layout.json`、`trackX_cones.csv`、`trackX.png` | 从规则图估读，尚无完整官方尺寸约束或全程道宽验收。 |

桶位坐标是图像估读，不是场地测量。A 常规桶距蓝 0.480–1.475 m、红 0.452–1.145 m，黄色 V 断口不计；B–D 局部估算宽度约 1.20–1.23 m，不能宣称全程满足 1.5 m。当前数据适合预览和开发，不作为最终赛事通行率结论。

生成入口：

```bash
python 3Dsim/tools/trackA/trackA_geometry.py
python docs/track/draw_track_bcd.py
```

地图/数据交付 issue #2 已关闭；几何精度和仿真验收仍未完成。详细尺寸依据、历史重建和验证结果见本目录其他报告。
