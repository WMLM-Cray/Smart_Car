# screenshots/ — Gazebo 目视检查截图（5 张，实际捕获）

捕获环境：WSL2 Ubuntu 22.04 + Gazebo Classic 11.10.2 GUI（WSLg）；
模型来自本包 install 的 `GAZEBO_MODEL_PATH`。

| 文件 | world | 视角 | 验证点 |
|---|---|---|---|
| `01_overview.png` | sensor_test | 高处俯视全场 | 红(y=+0.75) 蓝(y=-0.75) 各 5 锥桶 + 挡板 + 参考盒在场、布局符合任务书 |
| `02_cones.png` | sensor_test | 红蓝近景 | 红/蓝颜色正确、锥体无白色反光带、锥台轮廓（非盒形） |
| `03_collision.png` | sensor_test | 侧视锥台外形 | 锥台自下而上收敛；碰撞网格 = 视觉网格同一 STL，故所见即碰撞外形 |
| `04_ground.png` | sensor_test | 近地仰视 | 底座底面贴地 z=0，无悬浮 / 无穿地 |
| `05_demo_overview.png` | demo_world | 高处俯视全场 | 五类模型齐全：红绿灯(原点)、A/B 标志牌、车道线、红蓝锥桶、蓝色挡板 |

捕获方法：
- 相机位姿经 `gz topic -p /gazebo/<world>/user_camera/joy_pose` 确定性下发
  （look-at 四元数：camera 朝向 local +X，forward = normalize(target-pos)，
  right = up×forward，up = forward×right；所有 orientation.x/y/z/w 字段必须齐备，
  否则 gazebo.msgs.Pose 拒绝）；
- 画面用 `xwd -id <Gazebo 主窗口 id>` 抓取（主窗口标题为 `Gazebo` 首字母大写、
  约 1908×987；GLX 窗口，需经 xwd 转码后 convert 为 RGB PNG）。

诚实说明（本会话所用模型不具备图像视觉，故"目视"判读改为程序化像素分析）：
- 用 PIL/numpy 对每张 PNG 做颜色分类（R=红占优 / G=绿占优 / B=蓝占优 / Y=黄 /
  W=近白 / 背景），生成 ASCII 色图人工判读；
- 判读结论：sensor_test 四张中红锥区 R 占优、蓝锥区 B 占优（颜色正确，锥体上无
  W 白反光带）；R/B 色块自下而上收窄（锥台轮廓，非盒形代理）；04 中锥底贴地；
  01 中红在 +y 侧、蓝在 -y 侧（布局符合任务书）；
- `05_demo_overview.png` 判读：红锥（右中）、蓝锥/蓝板（右）、白色车道线（中）、
  红绿灯（左中，红/黄/绿三色灯球像素在场）——五类模型均在场。
- "碰撞非盒形代理"由两部分证据共同证明：(a) 碰撞 frustum 为 96 三角形 STL trimesh，
  非 `<box>`（见 validation.log §B）；(b) 视觉与碰撞 frustum 是同一 STL
  （uri + scale + pose 一致），故 03 所见锥台外形就是实际碰撞外形。

若主开发者需要真眼确认，请打开这 5 张 PNG，或重跑 DELIVERY.md 第 7 步 GUI。
