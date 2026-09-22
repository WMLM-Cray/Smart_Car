# DELIVERY.md — racecar_test_assets 第 3 轮交付说明（五类模型 + 演示世界）

## 版本与来源

- **交付分支：`feature/racecar-test-assets`**（独立功能分支，**不合并主分支、不 force push**）。
  基于主分支 `main`（HEAD `dcd95d8`）切出，仅含 `hwcart_sim/src/racecar_test_assets` 的改动。
- **A 赛道几何冻结在本地 commit `7ba1e08`**：本分支**未**改动主项目已有车辆与赛道几何，
  也未用截图/旧版本覆盖任何主项目几何。最终几何与接口适配由主项目统一决定。
- 本分支**不包含**阶段 1 车辆文件（`racecar_sim/package.xml`、`setup.py`、`config/urdf`、
  `racecar_bringup_sim/package.xml`、`setup.py`、`launch`）与根目录调试脚本
  （`acceptance_test.sh`/`cleanup.sh`/`debug_*.sh`/`gdb_*.cmds`/`gdb_throwtype.py`/
  `grep_launch.sh`/`launch_test.sh`/`smoke_test.sh`/`throwtrace.cpp` 等）——这些不属于本交付。
- 完整提交号见本交付随附的交付说明（仓库地址、分支、完整 commit SHA、文件清单、
  复现命令、测试证据、已知问题一并给出）。

## 内容物

| 路径 | 说明 | 状态 |
|---|---|---|
| `racecar_test_assets/` | 完整源码包（8 模型 + 2 世界 + 生成/校验工具 + 测试 + 文档） | — |
| `models/` | 8 个模型（5 类道具 + 参考盒）：cone_red/blue、blue_board、reference_box、traffic_light、sign_a/b、lane_line | 随包 |
| `worlds/` | `sensor_test.world`（传感器测试）+ `demo_world.world`（五类模型演示） | 随包 |
| `config/` | `sensor_test.csv` + `demo_world.csv`（布局唯一来源） | 随包 |
| `delivery/DELIVERY.md` | 本文件 | — |
| `delivery/acceptance.md` | 逐项验收对照表（PASS / 未验证） | — |
| `delivery/build.log` | colcon 全新工作区构建 + install/env hook 校验 | PASS（实际执行） |
| `delivery/launch.log` | Gazebo 11.10.2 GUI 启动 demo_world | PASS（实际执行） |
| `delivery/validation.log` | 验收入口 135 项 + 干净副本安装检查 | PASS（实际执行） |
| `delivery/negative_test.log` | 4 种破坏的负例测试 | PASS（实际执行） |
| `delivery/screenshots/` | 5 张目视检查截图（sensor_test×4 + demo_world×1）+ README | PASS（实际捕获） |

## 生成环境

- WSL2 `Ubuntu-22.04`（Jammy）+ ROS 2 Humble + Gazebo Classic **11.10.2**（GUI，经 WSLg 显示）
- Python 3.10（标准库；PIL/numpy 仅用于截图判读）
- 全新测试工作区（非开发目录）：`src/racecar_test_assets` 从交付目录复制，
  `colcon build` 产出 `install/`。

## 主开发者复现命令

在任意 Ubuntu 22.04 + ROS 2 Humble + Gazebo Classic 11 机器上：

```bash
# 1. 建全新测试工作区并放入交付包
mkdir -p ~/acceptance_ws/src
cp -r <本交付>/racecar_test_assets ~/acceptance_ws/src/
cd ~/acceptance_ws

# 2. 构建 + 验证安装产物（= 交付的 build.log）
colcon build --packages-select racecar_test_assets 2>&1 | tee build.log
source install/setup.bash
echo "$GAZEBO_MODEL_PATH"            # 应包含 .../share/racecar_test_assets/models

# 3. 验收入口 135 项（= validation.log 第 1 部分）
cd src/racecar_test_assets && python3 tools/validate_assets.py

# 4. 干净副本安装检查（= validation.log 第 2 部分）
bash tools/acceptance_install_check.sh

# 5. 负例测试（= negative_test.log）
bash test/test_acceptance_negative.sh

# 6. 无头加载冒烟 + 全部本地校验
bash tools/gazebo_load_check.sh worlds/demo_world.world
bash test/run_all_tests.sh

# 7. GUI 目视（= launch.log + screenshots）
source ~/acceptance_ws/install/setup.bash
ros2 launch racecar_test_assets preview.launch.py world:=demo_world.world
```

## 已知限制（详见 README「已知限制」与 docs/assumptions.md、docs/models.md）

1. 所有道具 `static=true`：可验证感知，不能验证撞倒/位移判罚；
2. 无白色反光带（按任务书）；无挡板文字/支架（假设）；
3. 锥台为 24 段多边形近似（内切比外接小 0.86%），视觉与碰撞同一网格（同一 STL）；
4. 锥台碰撞体为 STL trimesh，本机 Gazebo 11.10.2 + ODE 已实际加载成功
   （launch.log 无加载报错），无需回退方案；
5. 建模假设（不得当作实测值）：底座厚度 0.05、锥台底/顶直径 0.24/0.08、
   挡板厚度 0.02、红绿灯杆/灯箱/灯球尺寸、标志牌牌面/字体、车道线厚度、
   全部颜色数值、地面与光照参数（见 docs/assumptions.md 全表）；
6. 不含车辆，不创建 /scan、/image_raw、/cmd_vel 等话题；与主工程车辆的联测项
   （激光 7.68 m、相机颜色、RViz 对齐等）待主开发者集成验收（见 acceptance.md §6）；
7. 红绿灯 `static=true` 且**无切换接口**，三灯常亮（docs/models.md §3）；
8. 车道线 0.01 m 厚贴地，2D 雷达（激光高度 0.21 m）预计不可见，仅作视觉标线；
9. 标志牌贴图跨机器重生成依赖系统字体，交付以已提交 PNG 为准（docs/models.md §4）。

## 目视检查的诚实说明

5 张截图确由 Gazebo GUI 实际捕获（相机位姿经 `gz topic -p .../user_camera/joy_pose`
确定性下发，画面经 `xwd -id <Gazebo 主窗口 id>` 抓取）。但本会话所用的模型
**不具备图像视觉**，故"红蓝颜色 / 无白反光带 / 锥台外形 / 接地 / 五类模型在场"
等人眼判读项，改由**程序化像素分析**（PIL/numpy 颜色分类 + ASCII 色图）完成；
叠加 validate_assets 的几何证据（视觉=碰撞同一 STL、96 三角形锥台、0.63 m / 0.28 m
实测、红绿灯/标志牌/车道线尺寸、demo_world 五类模型）共同支撑结论。
如主开发者需真眼确认，请打开 5 张 PNG 或重跑第 7 步 GUI。方法与判读结论详见
`delivery/screenshots/README.md`。

## 主开发者需要决定的问题

1. 建模假设数值是否认可（docs/assumptions.md 全表，尤其挡板厚度 0.02 m、
   锥台底/顶直径 0.24/0.08、红绿灯/标志牌/车道线全部尺寸）；
2. 推荐车辆位姿 `base_footprint (0, 0, yaw=0)`、激光相对 (0.07, 0, 0.21)
   → 前方理论测距 7.68 m，是否与主工程实际参数一致；
3. 模型资产与赛道坐标的最终合并与接口适配（本分支只交付，不与主项目合并）。
