# acceptance.md — 逐项验收对照表（第 3 轮，五类模型 + demo_world，实际执行版）

规则：**只有实际执行过并有证据的项目才记 PASS**；未执行的记"未验证"并注明
需要什么条件、由谁执行。本轮全部验收在 WSL2 Ubuntu 22.04 + ROS 2 Humble +
Gazebo Classic 11.10.2 上实际执行。交付在独立分支 `feature/racecar-test-assets`
（不合并主分支、不 force push、不动主项目车辆与赛道几何；A 赛道几何冻结于
本地 commit `7ba1e08`）。证据文件与本表来自同一份交付包。

## 1. 从交付包重新构建

| 项目 | 状态 | 证据 / 说明 |
|---|---|---|
| 不依赖原开发目录的绝对路径 | PASS | validation.log §2：干净副本上 grep 无绝对机器路径 |
| 模型/网格/材质/world 随包完整（无缓存掩盖缺文件） | PASS | validation.log §2：全部 model:// 与网格 URI 在包内解析；launch.log 无 "Getting models from"；GAZEBO_MODEL_PATH 指向本包 install models |
| 不需要厂家源码 / 主工程代码 | PASS | validation.log §2：无 racecar_sim 引用 |
| 不需要临时下载模型 | PASS | validation.log §2：无网络 URI；地面内联定义 |
| 安装规则覆盖每个交付文件 | PASS | validation.log §2：逐文件覆盖检查通过 |
| 新工作区 colcon 构建 + 启动日志无资源缺失 | **PASS** | build.log：colcon exit 0；launch.log（demo_world）：无 [Err]/Unable to load/does not exist |

## 2. 自动检查尺寸与布局（python3 tools/validate_assets.py，135 项）

| 检查对象 | 必须满足 | 状态 | 证据 |
|---|---|---|---|
| 红蓝锥桶 | 高 0.63 m，底座外廓 0.28×0.28 m | PASS | validation.log §1 [B]：实测 SDF+STL（含 scale/pose 后最终外廓）0.6300 m / 0.2800×0.2800 m |
| 红蓝几何 | 几何相同，仅颜色不同 | PASS | §1 [C]：STL 字节级一致 + SDF 几何一致 + 颜色不同 |
| 挡板 | 长 0.80 m、高 0.50 m，厚度明确记录 | PASS | §1 [D]：0.800/0.500 实测；厚度 0.020 m 记录（假设值） |
| 模型原点 | 底面中心；z=0 接地 | PASS | §1 [B][D][E]：pose z = size_z/2，底面 z=0，无穿地 |
| 场地数量 | 10 锥桶 + 1 挡板 + 1 参考盒 | PASS | §1 [F]：cones 10, boards 1, refboxes 1, total 12 |
| 锥桶坐标 | 与任务书坐标表一致 | PASS | §1 [F]：world 直接对照任务书表（不经 CSV），12 个位姿逐一匹配 |
| 挡板 | (3.0, −0.75, 0)，yaw=0，不侵入相邻锥桶 | PASS | §1 [F]：位姿匹配 + 与 blue_02/03 底座间隙各 0.06 m |
| 参考盒 | 朝向起点的前表面 x=7.75 m | PASS | §1 [F]：8.0 − 0.5/2 = 7.750 m |
| ID | 唯一，无重复 | PASS | §1 [F]：12 个唯一 ID |
| 资源引用 | 所有引用文件可解析、可找到 | PASS | §1 [A]：model:// include、网格 URI、model.config 全部解析，STL 结构合法 |
| 红绿灯 | 灯箱 0.30×0.16×0.66、杆接地、红/黄/绿灯球在 | PASS | §1 [H]：逐尺寸 + 三色灯球 visual 检查 |
| 标志牌 sign_a/b | 材质脚本 `model://` 解析 + 贴图 PNG 随包（512×512） | PASS | §1 [H]：材质脚本 URI 解析 + PNG 尺寸检查 |
| 车道线 | 3.00×0.15×0.01 白 | PASS | §1 [H]：逐尺寸检查 |
| demo_world | 五类模型齐全、world 名正确、id 集与 model 引用 == CSV | PASS | §1 [I]：五类模型 + world==CSV（csv 8 vs world 8） |
| 失败返回非零退出码 | — | PASS | 第 3 节负例证明（4 种缺陷均 exit 1） |

合计 **135 项检查，135 通过，0 失败**（validation.log §1）。

## 3. 负例（证明检查不是摆设）

| 破坏方式 | 状态 | 证据 |
|---|---|---|
| 改坏锥桶高度（总高 0.66 m） | PASS（被点名 + exit 1） | negative_test.log：`[FAIL] cone_red: total height 0.63 m ... (0.6600 m)` |
| 删除必需网格（cone_blue STL） | PASS（被点名 + exit 1） | negative_test.log：`[FAIL] cone_blue: mesh uri resolves ... not shipped` |
| 改错锥桶坐标（red_01 x=1.9） | PASS（被点名 + exit 1） | negative_test.log：`[FAIL] red_01: pose matches task book ...` |
| 复制重复 ID（blue_02×2） | PASS（被点名 + exit 1） | negative_test.log：`[FAIL] no duplicate object ids ... ['blue_02']` |
| 恢复后重新通过、破坏物不留在交付中 | PASS | negative_test.log 结尾：pristine copy passes again；破坏仅存在于已删除的临时目录 |

## 4. Gazebo 目视检查

| 项目 | 状态 | 证据 / 说明 |
|---|---|---|
| 01_overview.png（sensor_test 全场） | **PASS** | 截图实际捕获；红(y=+0.75)蓝(y=-0.75)布局符合任务书（像素判读） |
| 02_cones.png 红蓝近景/无反光条 | **PASS** | R/B 颜色占优、锥体无 W 白反光带（像素判读） |
| 03_collision.png 碰撞外形 | **PASS** | 锥台自下而上收敛；碰撞=视觉同一 STL（validation.log §B） |
| 04_ground.png 接地/穿模 | **PASS** | 锥底贴地 z=0，无悬浮/穿地（像素判读 + §B 无 geometry below z=0） |
| 05_demo_overview.png（demo_world 全场） | **PASS** | 五类模型在场：红锥/蓝锥蓝板/白车道线/红绿灯三色灯球（像素判读） |

说明：本会话模型不具备图像视觉，"目视"判读由 PIL/numpy 像素分析 + ASCII 色图完成，
方法见 `screenshots/README.md`。

## 5. 锥桶 z=0.21 m 扫描截面说明

| 项目 | 状态 | 证据 |
|---|---|---|
| 底座厚度/圆台高度/底顶半径列出 | PASS | validation.log §1 [G]：0.050 m / 0.580 m / 0.1200 / 0.0400 m |
| 该高度理论半径计算 | PASS | 线性插值 r=0.0979 m（外接圆直径 0.1959 m） |
| 碰撞网格实际截面核对 | PASS | §1 [G]：对实际 STL 做三角形切片——24 顶点在外接圆上，24 个对角交点严格落在多边形边上 |
| 多边形近似误差写清 | PASS | 外接 0.1959 m vs 内切 0.1942 m，误差 0.86%（24 棱） |

## 6. 未验证项目（待主开发者集成验收）

以下项目**没有**验证，也未生成任何假数据：

- `/scan` 中的实际距离（理论预期：正前方 ~7.68 m 打到参考盒前表面 = 7.75 − 0.07 激光前移量）；
- `/image_raw` 中的红蓝锥桶画面；
- 车辆经过模型时的实际碰撞效果（模型 static=true，推倒/位移判罚本就无法由此验证）；
- RViz 中激光与模型对齐；
- 红绿灯切换控制（本模型无切换接口，三灯常亮，见 docs/models.md §3）。

## 7. 交付目录

```
racecar_test_assets/          完整源码包（本目录上一级）
delivery/
  DELIVERY.md                 版本来源、环境、命令、限制
  acceptance.md               本文件：逐项 PASS/未验证
  build.log                   PASS（实际构建）
  launch.log                  PASS（实际启动 demo_world）
  validation.log              PASS（135 项 + 干净副本安装检查）
  negative_test.log           PASS（4 种负例）
  screenshots/                PASS（5 张 PNG + README）
```
