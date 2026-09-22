# racecar_test_assets — 场景资源包（五类模型 + 演示世界）

华为昇腾比赛室外 ROS 2 无人车仿真的**标准测试世界资源包**。
用途：给主开发者验证雷达扫描、相机颜色识别、模型尺寸。
**这是一个自包含的测试夹具，不是官方 A/B/C/D 赛道**（官方赛道不在本轮范围）。

包含 8 个模型（5 类比赛/测试道具 + 1 个测距参考盒）+ 2 个世界，全部几何参数
的唯一来源是 `tools/asset_params.py`，布局的唯一来源是 `config/*.csv`：

| 模型 | 尺寸（长×宽×高 / 说明） | 数量 |
|---|---|---|
| `cone_red` / `cone_blue` | 总高 0.63 m；方底座 0.28×0.28 m + 圆台体（红蓝仅材质不同） | 各 5（sensor_test） |
| `blue_board` | 0.80 × 0.02(厚,假设) × 0.50 m，蓝色挡板 | 1 |
| `reference_box` | 0.5 × 1.0 × 1.0 m，灰色测距参考盒（测试夹具，非比赛道具） | 1 |
| `traffic_light` | 杆 r=0.05/h=1.80 + 灯箱 0.30×0.16×0.66 + 三色灯球 r=0.07（常亮，无切换接口） | 1（demo_world） |
| `sign_a` / `sign_b` | 杆 r=0.04/h=1.40 + 牌面 0.50×0.03×0.70，白底黑字 A/B | 各 1（demo_world） |
| `lane_line` | 3.00 × 0.15 × 0.01 m，白色地面标线 | 2（demo_world） |

> 逐类模型的六项说明（尺寸/单位/原点/朝向/地面高度、visual vs collision、
> 材质/贴图/mesh 相对路径、外部素材来源与许可证、雷达/相机检测能力与未测试项、
> 红绿灯控制接口或静态注明）见 **[docs/models.md](docs/models.md)**。

## 目录结构

```
racecar_test_assets/
├── config/sensor_test.csv     # sensor_test 布局唯一来源（id,model,x,y,z,yaw）
├── config/demo_world.csv      # demo_world 布局唯一来源（五类模型演示）
├── models/                    # 8 个模型（5 类 + 参考盒）
├── worlds/sensor_test.world   # 传感器测试世界（由 sensor_test.csv 生成）
├── worlds/demo_world.world    # 五类模型演示世界（由 demo_world.csv 生成）
├── launch/preview.launch.py   # ros2 launch 独立预览（world:= 参数，不 spawn 车辆）
├── tools/                     # 参数源 + 生成器 + 校验脚本（见下表）
├── test/                      # 测试套件（bash test/run_all_tests.sh）
├── docs/                      # 顶视图 / 假设清单 / 模型规格 / 验证状态
└── env-hooks/                 # 安装后自动追加 GAZEBO_MODEL_PATH
```

## 依赖

- **Gazebo Classic 11**（目标环境；本包不使用 Gazebo Sim / 新接口）
- ROS 2（仅 `launch/preview.launch.py` 和 colcon 构建需要；纯 Python/bash
  工具链不依赖 ROS）
- Python 3（仅标准库）、bash

## 构建

```bash
cd <workspace>
colcon build --packages-select racecar_test_assets
source install/setup.bash
```

`source` 之后 `GAZEBO_MODEL_PATH` 自动包含本包的 `models/`
（由 `env-hooks/racecar_test_assets.dsv` 注入），**无需手工拷贝模型**。

## 独立打开测试世界

```bash
# 方式 A：无需 ROS 2，只要求 gazebo 在 PATH 里
bash tools/preview.sh                          # sensor_test.world
bash tools/preview.sh demo_world.world         # demo_world.world（五类模型演示）

# 方式 B：ros2 launch（gui:=false 则只起 gzserver）
ros2 launch racecar_test_assets preview.launch.py                          # sensor_test.world
ros2 launch racecar_test_assets preview.launch.py world:=demo_world.world  # demo_world.world
```

世界内不含车辆——按任务书约定，车辆由主工程 bringup 负责 spawn。

## 主工程如何引用

1. **引用世界**：把主工程启动时传入的世界路径指向本包的
   `worlds/sensor_test.world`（绝对路径可用
   `$(find-pkg-share racecar_test_assets)/worlds/sensor_test.world`），
   或直接在主工程世界里 `<include>` 本包的 `model://` 模型；
2. **模型路径**：`source` 本包的环境后 `model://cone_red` 等即可解析；
   若主工程单独设置过 `GAZEBO_MODEL_PATH`，注意在原有值基础上**追加**
   而不是覆盖（本包 env hook 用 `prepend-non-duplicate`，与追加语义兼容）；
3. **改布局**：不要直接改 `.world`——改 `config/sensor_test.csv` 后重新生成
   （见"测试与再生成"），否则 world 与 CSV 会漂移（有测试把关）。

## 推荐车辆起始位姿（待主开发者确认）

- `base_footprint`：**(0, 0, 0)，yaw = 0**（朝 +X）；
- 主工程激光（z = 0.21 m，相对 base_footprint x = +0.07 m）；
- 激光正前方到参考盒前表面（世界坐标 x = 7.75 m）的**理论测距值
  7.68 m = 7.75 − 0.07**。注意区分：**7.75 m 是世界原点起算**，
  7.68 m 才是雷达读数预期。两侧锥桶中心距 y = ±0.75 m。

## 物体坐标清单（世界坐标，米；地面 z = 0）

与 `config/sensor_test.csv` 完全一致（`tools/check_world.py` 保证零漂移）。

| id | 模型 | x | y | z | yaw |
|---|---|---|---|---|---|
| red_01..red_05 | cone_red | 1.2 / 2.4 / 3.6 / 4.8 / 6.0 | +0.75 | 0 | 0 |
| blue_01..blue_05 | cone_blue | 1.2 / 2.4 / 3.6 / 4.8 / 6.0 | −0.75 | 0 | 0 |
| board_01 | blue_board | 3.0 | −0.75 | 0 | 0 |
| reference_box | reference_box | 8.0 | 0 | 0 | 0 |

- 同侧相邻锥桶间距 **1.2 m**，红蓝两排相距 **1.5 m**（y = ±0.75）；
- 挡板位于 blue_02 与 blue_03 之间，与两侧锥桶底座各留 **0.06 m** 间隙；
- 参考盒前表面（+X 面）位于 **x = 7.75 m**（中心 8.0 − 0.5/2）。

`demo_world` 的 8 个对象（五类模型演示）见 `config/demo_world.csv`：
红绿灯在原点、A/B 牌在 (±1.5, ±1.5)、车道线在 (3.5, ±0.75)、
红蓝锥桶在 (6.0, ±1.0)、挡板在 (7.5, 0)。

## 模型原点与朝向约定

| 模型 | 原点 | 局部轴 |
|---|---|---|
| 锥桶 | 底面中心 | —（旋转对称，yaw 无意义） |
| blue_board | 底边中点（底面中心） | X = 长 0.80，Y = 厚 0.02，Z = 高 0.50 |
| reference_box | 底面中心 | X = 0.5（深度），Y = 1.0（宽），Z = 1.0（高） |
| traffic_light | 杆底面中心 | X/Y 对称；灯箱正面 +Y（yaw 旋转） |
| sign_a / sign_b | 杆底面中心 | 牌面 X–Z（0.50×0.70），法线 ±Y |
| lane_line | 底面中心 | X = 长 3.00，Y = 宽 0.15，Z = 厚 0.01 |

视觉与碰撞几何完全一致（锥桶：方底座 box + 锥台 STL，同一份网格
同时用于 visual 与 collision；板/盒：box 同时用于两者）。

## 锥桶 z = 0.21 m 截面（主工程激光高度）

激光高度 0.21 m 落在**圆台段**（底座仅 0.05 m 厚），截面为 24 棱多边形：

- 外接圆直径 **0.1959 m**——2D 雷达可见最大宽度；
- 圆等效截面面积 0.0301 m²；内切圆直径 0.1942 m（比外接小 0.86%）。

`python3 tools/cone_section.py --z 0.21` 可复算任意高度截面。

## 建模假设（摘要）

规则只给了锥桶总高 0.63 / 底座 0.28、挡板 0.80×0.50 等少数尺寸，
以下均为**假设值、待确认，不得当作实测值**：底座厚度 0.05、锥台底/顶
直径 0.24/0.08、24 段离散、挡板厚度 0.02、所有颜色数值、地面与光照参数。
完整清单与理由见 **[docs/assumptions.md](docs/assumptions.md)**。

## 测试与再生成

```bash
bash test/run_all_tests.sh         # 全部校验（任一失败退出码非 0）
python3 tools/generate_world.py    # sensor_test.csv -> worlds/sensor_test.world
python3 tools/generate_world.py --config config/demo_world.csv \
    --output worlds/demo_world.world --world-name demo_world
python3 tools/generate_top_view.py # CSV -> docs/top_view.svg
python3 tools/generate_cone_mesh.py  # asset_params -> models/*/meshes/*.stl
python3 tools/generate_sign_texture.py  # 重生成 sign_a/sign_b 贴图（需 PIL）
python3 tools/check_models.py      # SDF/STL 对照 asset_params（107 项）
python3 tools/check_world.py       # world 对照 CSV（sensor_test）
python3 tools/check_world.py --config config/demo_world.csv \
    --output worlds/demo_world.world --world-name demo_world
python3 tools/validate_assets.py   # 验收入口（见下节）
bash test/test_acceptance_negative.sh    # 负例：4 种破坏都必须被点名 + 非 0 退出
bash tools/acceptance_install_check.sh   # 干净副本安装检查（git archive HEAD）
```

## 验收（交付检查）

交付验收不依赖 ROS/Gazebo，任何有 Python 3 + bash 的机器都能复跑：

| 命令 | 内容 |
|---|---|
| `python3 tools/validate_assets.py` | **单一验收入口**（135 项，只读交付文件，不重新生成）：资源引用可解析；红/蓝锥桶 0.63 m / 0.28×0.28 m（实测 SDF+STL，含 SDF scale 与 pose 后的最终外廓）；红蓝同几何仅颜色不同；挡板 0.80×0.50、厚度记录在案；原点=底面中心、z=0 接地；10 锥桶+1 挡板+1 参考盒；world 位姿**直接对照任务书坐标表**（不经过 CSV）且 world==CSV；ID 唯一；z=0.21 截面对**实际碰撞网格切片**验证；[H] 红绿灯/标志牌/车道线尺寸与贴图资源；[I] demo_world 五类模型 + world==CSV。任一失败退出码非 0 |
| `bash test/test_acceptance_negative.sh` | 负例：在临时副本中分别改坏锥桶高度、删除必需网格、改错锥桶坐标、制造重复 ID，验收器必须逐一点名缺陷并退出非 0；恢复后重新通过 |
| `bash tools/acceptance_install_check.sh` | 干净副本安装检查：`git archive HEAD` 把已提交状态提取到临时目录，验证无绝对路径、无网络下载、无主工程（racecar_sim）依赖、资源自包含可解析、CMake 安装规则覆盖每个文件、env hook 正确注入 GAZEBO_MODEL_PATH |

验收证据与逐项状态存放在 `delivery/`（DELIVERY.md、acceptance.md、validation.log、
negative_test.log 等）。Gazebo 相关验收（colcon 构建日志、启动日志、GUI 截图）
本机无法执行，`delivery/build.log`、`delivery/launch.log`、
`delivery/screenshots/README.md` 中给出了目标环境的精确命令。

**变更流程**：改 `tools/asset_params.py`（几何/材质）→ 重跑
`generate_cone_mesh.py` + `generate_world.py` + `generate_top_view.py` →
更新 `models/*/model.sdf` 中引用的尺寸 → `bash test/run_all_tests.sh`；
改布局只改 `config/sensor_test.csv` → 重跑两个生成器。
非法输入（重复 ID、NaN/Inf、未知模型等）会使生成器以非 0 退出，
world **不会**写出（有专门测试覆盖）。

## 已知限制

1. 所有道具 `static=true`：可验证感知，**不能**验证撞倒/位移判罚；
2. 无白色反光带、无挡板文字（按任务书与假设）；
3. 锥台为 24 段多边形近似（与圆相差 0.86%），视觉与碰撞一致；
4. 锥台碰撞体为 STL **trimesh**，已在交付轮于 Gazebo Classic 11 + ODE
   实际加载验证通过（`delivery/launch.log` 无加载报错）；若目标环境
   报错，回退方案是把 `model.sdf` 中的 mesh 碰撞换成 2–3 段圆柱叠加
   近似（视觉不变），几何参数已在 `asset_params.py` 备好；
5. 本包不含车辆、不创建 `/scan`、`/image_raw`、`/cmd_vel` 等话题；
   与主工程车辆的联测项见 `docs/validation.md` 第 3 节（待集成验证）；
6. 红绿灯 `static=true` 且**无切换接口**，三灯常亮，不能验证信号状态机/
   切换逻辑（详见 `docs/models.md` §3）；
7. 车道线 0.01 m 厚贴地，2D 雷达（激光高度 0.21 m）预计不可见，仅作视觉标线；
8. 标志牌贴图跨机器重生成依赖系统字体，交付以**已提交 PNG** 为准
   （`docs/models.md` §4）。

## 验证状态

详见 **[docs/validation.md](docs/validation.md)**：本机已通过的项
（107 项尺寸校验、两个 world==CSV、布局符合任务书、非法输入拒绝、字节级
可复现、截面计算、顶视图、五类模型/demo_world 验收）带原始输出；Gazebo 相关
项列出目标环境执行的命令与预期；主工程联测项列出待集成验证清单。
