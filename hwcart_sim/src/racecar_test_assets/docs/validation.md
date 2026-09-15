# 验证状态（racecar_test_assets 第 3 轮：五类模型 + demo_world）

验证分三级，状态如实标注：

- ✅ **已验证**——本机实际运行通过，有输出为证；
- ⏳ **待目标环境验证**——脚本已就绪，但本机（Windows 11，无 Gazebo/无 ROS 2）
  无法执行，需在 Ubuntu + Gazebo Classic 11 的目标机器上跑；
- 🔗 **待集成验证**——需要主开发者把本包接入主工程后才可执行。

原始输出存证：

- `delivery/validation.log` —— 验收入口 + 干净副本安装检查（交付证据）；
- `delivery/negative_test.log` —— 负例破坏测试；
- `docs/evidence/run_all_tests_windows.txt` —— 完整测试套件输出；
- 逐项 PASS/FAIL/未验证 对照表见 `delivery/acceptance.md`；
- 逐类模型六项说明（尺寸/原点/朝向/visual vs collision/材质贴图路径/
  外部素材与许可证/雷达相机检测/红绿灯接口）见 **[docs/models.md](models.md)**。

一条命令复跑全部本地校验：`bash test/run_all_tests.sh`
（任一失败退出码非 0；Gazebo 相关项在无 gzserver 的机器上按设计 SKIP）。

## 1. 已验证（✅ 本机通过）

### 1.1 验收入口 — `tools/validate_assets.py`（135 项全过）

单一入口，**只读交付的 SDF/STL/world，不重新生成、不只信配置数字**：

- 资源引用：所有 `model://` include 与网格 URI 在包内可解析、STL 结构合法；
  无网络 URI、无开发机绝对路径；
- 锥桶：总高 0.63 m、底座外廓 0.28×0.28 m——从 SDF 尺寸/位姿 + STL bbox
  **含 scale 与 pose 后的最终外廓**实测；红/蓝 STL 字节级一致、SDF 几何一致
  仅颜色不同；原点=底面中心、z=0 接地、无穿地；
- 挡板：0.80×0.50（规则）+ 厚度 0.02 记录在案；
- 布局：world 文件**直接对照任务书坐标表**（不经过 CSV），10 锥桶+1 挡板+
  1 参考盒、ID 唯一、挡板不侵入相邻锥桶底座、参考盒前表面 x=7.75 m，
  另有 world==CSV 一致性；
- z=0.21 截面：见 1.7；
- **[H] 新模型**：红绿灯（灯箱 0.30×0.16×0.66、杆接地、红/黄/绿灯球在）、
  sign_a/sign_b（材质脚本 `model://` 解析 + 贴图 PNG 随包）、lane_line
  （3.00×0.15×0.01）——逐类尺寸/贴图/资源检查；
- **[I] demo_world**：五类模型齐全、world 名正确、world 的 id 集与 model
  引用 == `config/demo_world.csv`。

```
== validate_assets: 135 checks: 135 passed, 0 failed ==
RESULT: PASSED
```

### 1.2 负例破坏测试 — `test/test_acceptance_negative.sh`

在**临时副本**中依次制造 4 种缺陷（不动真实交付物），验收器必须
**点名缺陷 + 退出非 0**：

```
[PASS] negative (broken cone height (cone_red total 0.66 m)): validator exit code 1
        [FAIL] cone_red: total height 0.63 m (rule; measured incl. scale + pose)  (0.6600 m)
[PASS] negative (deleted required mesh (cone_blue STL)): validator exit code 1
        [FAIL] cone_blue: mesh uri resolves + STL parses: ... not shipped
[PASS] negative (wrong cone coordinate (red_01 x=1.9)): validator exit code 1
        [FAIL] red_01: pose matches task book ...  (model://cone_red @ (1.9, 0.75, 0, yaw 0))
[PASS] negative (duplicate id (blue_02 twice)): validator exit code 1
        [FAIL] no duplicate object ids in world  (duplicates: ['blue_02'])
[PASS] restore: pristine copy passes again (nothing left broken)
RESULT: PASSED
```

### 1.3 干净副本安装检查 — `tools/acceptance_install_check.sh`

`git archive HEAD` 把**已提交状态**提取到全新临时目录（模拟"交付包放入
新测试工作区"，排除开发目录残留与 Gazebo 缓存干扰），对副本检查：
无绝对路径 / 无网络下载 / 无主工程（racecar_sim）依赖 / 资源自包含
（在副本上完整跑一遍 validate_assets）/ CMake 安装规则覆盖每个交付文件 /
env hook 注入 GAZEBO_MODEL_PATH。输出存 `delivery/validation.log`。

### 1.4 模型尺寸校验（开发粒度）— `tools/check_models.py`（107 项全过）

涵盖：锥桶总高 0.63、底座 0.28×0.28、红蓝几何字节级一致、
visual == collision、着地（z=0）、四色材质、STL 结构合法（96 三角形）；
红绿灯（杆/灯箱/三灯球尺寸与颜色）、sign_a/sign_b（杆/牌面 + 材质脚本 uri +
贴图随包）、lane_line（3.00×0.15×0.01 白）。

### 1.5 world 与 CSV 一致性 — `tools/check_world.py` + 布局符合任务书 —
`test/test_layout_spec.py`（14 项）

排距 1.5 m、同侧间距 1.2 m、挡板与 blue_02/03 底座间隙各 0.06 m、
参考盒前表面 7.75 m 等。`demo_world` 另以
`check_world.py --config config/demo_world.csv --output worlds/demo_world.world
--world-name demo_world` 做同样的 world==CSV 一致性（run_all_tests.sh 已纳入）。

### 1.6 非法输入拒绝 — `test/test_validation_errors.py`（9 拒 + 1 通过）

重复 ID、未知模型、NaN/Inf、非数字、表头错误、字段数错误、空文件、
非法 ID 字符：全部**非 0 退出 + stderr 明确报错 + 不产出 world 文件**。

### 1.7 锥桶 z = 0.21 m 碰撞截面（公式 + 实际网格双重验证）

`tools/validate_assets.py` 的 [G] 节对**实际碰撞 STL 做三角形切片**：

```
底座厚度      : 0.050 m（SDF 实测）
圆台高度      : 0.580 m（STL bbox 实测）
底/顶部半径   : 0.1200 / 0.0400 m（STL 环顶点实测）
理论半径 @z=0.21: 0.0979 m（线性插值） -> 外接圆直径 0.1959 m
实测网格截面  : 外接圆直径 0.1959 m（24 顶点），内切圆直径 0.1942 m
多边形近似误差: 0.86 %（24 棱外接 vs 内切）
[PASS] mesh slice: exactly 24 polygon vertices at the theoretical radius
[PASS] all other slice points lie on the polygon edges (section is the exact 24-gon)
```

切片验证证明：碰撞网格在激光高度的截面就是理论 24 边形（顶点在外接圆上、
对角线交点严格落在边上），不是"只算公式"。`python3 tools/cone_section.py`
可单独打印任意高度的解析值。

### 1.8 可复现性 — `test/test_determinism.sh`

同一 CSV 两次生成 world/SVG **字节级一致**，且与仓库内文件零漂移；
锥桶 STL 重新生成字节级稳定，红蓝两份一致。

### 1.9 顶视图 — `docs/top_view.svg` → `docs/img/top_view_render.png`

由 CSV 确定性生成，渲染成 PNG 后人工核对：红/蓝两排各 5 桶、排距/间距/
7.75 m 标注、坐标轴、图例均正确，无文字重叠与裁切。

## 2. 待目标环境验证（⏳ 需 Ubuntu + Gazebo Classic 11）

本机无 WSL/Gazebo，脚本已写好，到目标机器进入本包目录直接执行
（`delivery/build.log`、`delivery/launch.log`、`delivery/screenshots/README.md`
含精确命令与预期）：

| 项目 | 命令 | 预期 |
|---|---|---|
| 新工作区构建 | `cd <新测试工作区> && colcon build --packages-select racecar_test_assets` | 构建成功（构建日志存 delivery/build.log） |
| 安装后 env hook | `source install/setup.bash && echo $GAZEBO_MODEL_PATH` | 含 `share/racecar_test_assets/models`，无需手工拷贝模型 |
| 无 ROS 独立预览 | `bash tools/preview.sh` | Gazebo 打开，10 锥桶 + 挡板 + 参考盒 + 地面，无资源缺失报错 |
| 无头加载冒烟 | `bash tools/gazebo_load_check.sh` | 退出码 0，日志无 error 级输出 |
| 加载冒烟（含负向对照） | `bash test/test_gazebo_smoke.sh` | 正向加载干净；把 cone_red 换成不存在模型后**必须**报错 |
| **trimesh 碰撞加载** | 同上，观察 `[Err]` / 物理引擎警告 | STL 碰撞体在 ODE 正常加载。**若不支持**：按 README"已知限制"第 4 条的阶梯圆柱回退方案改 `models/*/model.sdf` |
| GUI 截图 5 张 | gzclient `File → Save Screenshot` | `01_overview / 02_cones / 03_collision / 04_ground`（sensor_test）+ `05_demo_overview`（demo_world 五类模型，存 delivery/screenshots/） |

本机从未安装过 Gazebo，**不存在用本机模型缓存掩盖缺文件的可能**；
资源自包含性已由 1.3 的干净副本检查证明。

## 3. 待集成验证（🔗 需主开发者接入主工程后执行）

| 项目 | 方法 | 预期 |
|---|---|---|
| 主工程加载本世界 | 在 `sim_bringup` 侧把 world 路径指向本包（或 include），GAZEBO_MODEL_PATH 由 env hook 提供 | 场景完整，无缺模型报错 |
| 车辆按建议位姿出生 | `base_footprint (0, 0, yaw=0)` | 车辆位于原点朝 +X，不与任何道具碰撞 |
| 激光测距核对 | 车辆静止，看 `/scan`（主工程话题）正前方 | 前方 ~**7.68 m** 处打到参考盒前表面（7.75 − 0.07，见 assumptions.md 第 5 节）；两侧 ~0.75 m 处打到锥桶 |
| 相机颜色识别 | 相机看 red_01 / blue_01 / board_01 | 红蓝可分，挡板呈蓝色 |
| 车辆与模型碰撞效果 | 开车轻碰锥桶 | 锥桶为 static，不会被推动——如需验证推倒/位移判罚需后续轮次的动态锥桶 |
| RViz 激光与模型对齐 | RViz 叠加 /scan 与场景 | 锥桶/参考盒点云位置与模型一致 |

> 按任务书要求：本轮**未**创建 `/scan`、`/image_raw` 等车辆话题，
> 也未为自测另造一辆车；以上各项由主开发者执行后回填结果。
