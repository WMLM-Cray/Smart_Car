# docs/img — 截图与渲染存证

| 文件 | 内容 | 生成方式 |
|---|---|---|
| `top_view_render.png` | `docs/top_view.svg` 的位图渲染（1501×604），作为顶视图交付存证 | 见下方"Windows" |

目标环境（Ubuntu + Gazebo Classic 11）需补充的截图（存入本目录，
命名后回填 `docs/validation.md` 第 2 节）：

| 建议文件名 | 内容 |
|---|---|
| `gazebo_top.png` | gzclient 俯视：两排锥桶 + 挡板 + 参考盒 |
| `gazebo_side.png` | 侧视：锥桶高度 0.63 m、参考盒 1.0 m |
| `gazebo_origin.png` | 车辆起点视角（原点朝 +X 看向参考盒） |

## 截图方法（目标环境）

gzclient 菜单 `File → Save Screenshot`，或用 Gazebo 自带相机工具：

```bash
# 若用 gzcamera 工具（视 Gazebo 版本可用性）：
gz camera -c default_camera --stream 640,480 <output.png>
```

最简单可靠的方式仍是 gzclient 窗口内直接截图（Print Screen 或
`File → Save Screenshot`）。

## Windows 本机渲染 SVG → PNG（本次交付用的方法）

本机无 Gazebo，顶视图用无头 Edge 渲染：

```bash
EDGE="/c/Program Files (x86)/Microsoft/Edge/Application/msedge.exe"
"$EDGE" --headless --disable-gpu \
  --screenshot="<绝对路径>/docs/img/top_view_render.png" \
  --window-size=1501,604 --default-background-color=FFFFFFFF \
  "file:///C:/.../docs/top_view.svg"
```

画布尺寸 1501×604 由 `tools/generate_top_view.py` 中的
`W/H` 计算得出，改布局后需同步 `--window-size`。
