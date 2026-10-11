# BEV 框架

把雷达回波、摄像头锥桶检测、摄像头地面分割按位姿累积成俯视语义栅格(BEV),策略吃这张图而不是单帧
扇区距离。动机:b036(road_v2.5)剩下的失败集中在稀桶、方角开口处,单帧里桶太少、"认错路"。

当前推荐模型 **bev_v2 / v2m3**(`bev_v2m3/candidate_005`,md5 `75b4c193…`),备选 v2m2(`6ad417d6…`)。
FastSim 结果,实车只跑过影子模式(不发控制)。

## 一张图看全

```
            ┌──────────────── 仿真(训练 / 评估) ────────────────┐      ┌──────────────── 实车 ────────────────┐
 路段生成    simulation/fastsim/road.py   级别 L1–L6x、LE3/LE4、官方 A–D(evaluation/road_track.official)
 传感器      雷达 raycast.py              | 真雷达 /scan
            假 YOLO   camera.FakeYolo    | (车上还没有 YOLO 节点,锥桶层为空)
            假地面分割 camera.FakeFloorSeg | vehicle/ground_seg_node.py:SegFormer-B0 @ NPU → common/ground_proj → /ground_free
 车辆        road_env.Plant / SpeedPlant  | 编码器 /encoder、IMU /IMU_data
                          │                                  │
 BEV 累积    common/bev.Bev  ←──── 同一份代码 ────→  common/bev_runtime.BevRuntime(位姿 = 编码器 + 陀螺积分)
 输入图      common/bev_policy.bev_image(6×80×80)+ scalars(3)
 策略        training/bev_net(torch)  ══ 导出 npz ══>  common/bev_policy.BevPolicy(纯 numpy 前向)
 老师 / 训练  training/bev_env.SeenTeacher、training/bev_train(DAgger)
 评估        evaluation/bev_testset、bev_track、bev_video            vehicle/road_car_node(真开)
                                                                    vehicle/bev_shadow(影子:只看不发)
```

## 代码在哪

| 层 | 文件 | 内容 |
|---|---|---|
| 共用 | `common/bev.py` | BEV 本体:5 cm 栅格、裁剪窗、格子类型、来源层、锥桶地标合并、20 s 过期 |
| | `common/bev_policy.py` | 输入规格(`bev_image`、`scalars`)、schema(bev_v1 = 5 通道,bev_v2 = 6 通道)、numpy 前向 |
| | `common/bev_runtime.py` | 车上运行时:位姿积分、按扫描/拍摄时刻登记、使能心跳、传感器过期,和 `RoadRuntime` 同一套输出 |
| | `common/ground_proj.py` | 相机投影(内参、畸变、外参,`GroundLut` 查表把分割图像素投到地面) |
| | `common/params.py` | 车辆几何、相机内外参(实测)、赛道参数(桶高 70 cm 等) |
| 仿真 | `simulation/fastsim/road.py` | 随机路段生成;边界材质 NONE / WALL / CONE / EDGE |
| | `simulation/fastsim/camera.py` | `FakeYolo`(视场、距离、遮挡、漏检、误差、误检)、`FakeFloorSeg`(地面分割,见下) |
| | `simulation/fastsim/road_env.py` | `RoadEnv`、`Plant`(每局随机车辆参数)、`SpeedPlant`(速度环近似) |
| 训练 | `training/bev_env.py` | `BevEnv` = RoadEnv + BEV + 假传感器;`SeenTeacher`(只按扫到过的边界规划,特权见文件头) |
| | `training/bev_net.py`、`bev_train.py` | torch 网络(与 numpy 前向逐元素一致)、DAgger |
| 评估 | `evaluation/bev_testset.py` | 随机路段配对测试集(带指纹)、跑模型、McNemar 配对比较 |
| | `evaluation/bev_track.py` | 官方 A–D,连跑 2 圈(第 1 圈含起步) |
| | `evaluation/bev_video.py` | 一局录成视频:左俯视真值,右模型输入 |
| 实车 | `vehicle/ground_seg_node.py` | 摄像头 → NPU SegFormer(地面判断并进模型)→ 投影 → `/ground_free`,10 Hz、每帧 ~67 ms |
| | `vehicle/road_car_node.py` | 真开:`load_policy` 认 `bev_*` schema 时用 BevRuntime |
| | `vehicle/bev_shadow.py` + `vehicle/shadow_feed/` | 影子模式:同一运行时只看不发;网页 8090 看 BEV 和决策、标记问题;C++ 节点把高频 IMU/编码器攒批 |
| | `vehicle/bev_logbook.py` | 问题记录本:按段(抱车自动分段)累积全局地图、路径、决策、标记现场 |
| | `vehicle/scan_guard.py` | 防撞:沿转向弧扫车身矩形,算到第一个碰撞点的后轴行程 |

约定(分辨率、格子类型、通道、网络结构)以 `common/bev.py`、`common/bev_policy.py` 文件头为准;权重和
这些数字绑定,改了要换 schema。

## 输入与网络

- 栅格 5 cm,窗口车前 6 m、车后 2 m、左右各 4 m;模型看 80×80(10 cm/格)。
- 通道:0 雷达记忆、1 锥桶地标、2 胶带、3 区域、4 本帧雷达、5 可行驶地面(bev_v2)。冲突优先级
  CONE > TRAFFIC_LIGHT > OBSTACLE > TAPE > ZONE > FREE。
- 锥桶按"地标"存:0.25 m 内的检测合并、至少 2 次命中才画。所有证据 20 s 没再更新就忘(限制里程计漂移);
  一圈 ~20 s,所以第 2 圈并没有第 1 圈的完整地图。
- 标量:车速、限速、偏航角速度。模型拍与拍之间无状态。
- 网络:4 层 stride-2 卷积 + 2 层 256 全连接(~54 万参数)。车上 numpy 前向 ~18 ms CPU —— 车上 numpy 链的是
  参考 BLAS,换 OpenBLAS 预计 1–2 ms(待做)。

## 仿真里的传感器

| | 模拟什么 | 和实车的对应 |
|---|---|---|
| 雷达 | 射线求交墙、桶(桶按高度的截面) | /scan 原样 |
| 假 YOLO | 锥桶落地点,带漏检/误差/误检/遮挡 | **车上还没有**,锥桶层实车为空 |
| 假地面分割 | 视场 70°×56°、有效 3 m;墙外、EDGE 外不算地面;扣桶底座;**桶后遮挡扇形**(桶 70 cm 高于相机 24 cm,一直挡到 3 m,2026-10-11 加);每帧整体偏移 5 cm、逐点抖 3 cm、5% 整帧失败 | SegFormer-B0 ADE20K 通用地面类,桶内外地板都算地面 |

地面分割的语义是"通用地面",锥桶赛道上不提供路沿,路沿只能靠锥桶层和雷达。它有用的地方是 EDGE
边界:路面换材质(地板/草地、马路牙子),雷达扫不到、也没有桶 —— 只有地面分割看得出。

## 路段与课程

随机路段由 `Level` 参数生成(路宽、弯角、内径、尖角、掉头、台阶、变宽、缺口、单侧无边界、墙、诱饵桶等)。
训练常用:L3、L4、L5、L5s、L6、L6h、L6f;测试另有 L4c。LE3 / LE4 = L3 / L4 的几何,边界全换成 EDGE。
官方 A–D 只用来评估,不进训练集。

当前训练配比(v2m2 / v2m3):`L5:0.12, L5s:0.12, L6h:0.1, L6f:0.1, L4:0.08, L6:0.05, L3:0.03, LE3:0.2, LE4:0.2`,
限速上限 2.5,老师弯中横向加速度 1.47 m/s²(0.15 G)。

```sh
python -m training.bev_train --schema bev_v2 --out runs/bev_v2m3 --init runs/v2m2_007.npz --beta0 0 \
  --cap-max 2.5 --a-lat 1.47 --rounds 6 --updates 4000 --lr 2e-4 --mix <上面的配比> \
  --eval-levels L5,L6h,L4c,L6f,LE3,LE4 --replay 1500000 --workers 7 --torch-threads 4
python -m evaluation.bev_testset run --eps runs/testset_93M.pkl --model <npz> --out runs/cones_x.json
python -m evaluation.bev_testset compare runs/cones_a.json runs/cones_b.json
python -m evaluation.bev_track --model <npz> --caps 1.8 2.5 --out runs/track_x.json
python -m evaluation.bev_video --model <npz> --track A --cap 1.8 --out runs/video_A.mp4
```

## 测试集

| 集 | 内容 | 指纹 |
|---|---|---|
| 锥桶 `testset_93M.pkl` | L5 / L6h / L4c / L6f 各 400 局,种子 93M 起、限速 ≤ 1.8 | `919e6387cd1fb4c8aa70692e2eafe7b7` |
| EDGE `testset_LE.pkl` | LE3 / LE4 各 400 局 | `1a69ffce…` |

指纹只管路段和车辆参数;传感器模型改了(如桶后遮挡)同一个指纹的成绩会变,对比要在同一份代码下跑。

## 模型谱系与结果

| 模型 | 起点 → 改动 | 锥桶集 | EDGE 集 | A–D |
|---|---|---|---|---|
| b036(road_v2.5) | 单帧扇区 | 95.87% | – | 1.8 档 29.8 s |
| bev_v1 d014 | 5 通道,0.15 G | 98.81% | – | 1.8 / 2.5:22.4 / 20.2 s |
| v2g | d014 热启动 + 地面层 | 98.69% ¹ | 0.13% | 1.8 / 2.5:22.3 / 20.0 s |
| v2e | 只训 EDGE(对照) | 4.9% | 98.75% | 0/16 |
| v2m | v2g + 锥桶/EDGE 混训 | 97.62% ¹ | 97.25% | A 2.5 档 15/16 |
| v2m2 | v2m 续训,真实相机参数 | 98.25% ² | 98.00% ² | 全 16/16,1.8:22.3–23.2 s |
| **v2m3** | v2m2 在桶后遮挡仿真上微调 6 轮 | **98.38%** ² | **98.00%** ² | 全 16/16,单圈和 v2m2 差 ≤ 0.3 s |

¹ 10-10 代码(无桶后遮挡)。² 10-11 代码(a707ed7,有桶后遮挡)。同代码下 v2m3 对 v2m2 配对 11 / 9(p = 0.82),
对 v2g 11 / 14(p = 0.69):都不显著。加遮挡前后 v2m2 自己 98.19% → 98.25%(配对 6 / 5)—— 遮挡几乎不影响,
因为它挡的主要是桶外地面,而路沿本来靠锥桶层和雷达。

A–D 单圈(v2m3,16 局平均):第 1 圈含起步 22.3–23.2 s(1.8)/ 20.1–21.0 s(2.5),第 2 圈 19.9–20.7 / 17.5–18.0 s。
第 1 圈慢的 ~2.7 s 来自仿真起步:死区空等 0.5–3.0 s(照实车 PWM 从中位爬到破静点)+ 平台以上加速 0.4–1.0 m/s²。

失败分析(d014 / v2g):测试集上 32 个失败局里,特权老师能开过 25 局,差距在模仿;续训(v2g2)loss 降一半但
成功率没有可测提升,这个口径下到平台了。

## 实车状态

- 相机内参、畸变、外参已实测(`common/params.py`:俯仰 −1.70°、偏航 +2.07°、横滚 −2.21°;停车位置不同
  俯仰差 ~1°,来自悬架)。地面分割节点在车上 10 Hz 跑通,NPU 和 CPU 上的 ONNX 掩码一致率 95.8–98.8%。
- 影子模式跑过两次记录本(10-10),没真开过 BEV 模型。v2m 在墙前想直冲,不会倒车 —— 真限制。
- 车上锥桶层为空(没有 YOLO 节点);训练时有假 YOLO,上锥桶赛道前要么接 YOLO,要么在仿真里关掉假 YOLO 验证。

## 待定 / 假设

- 加速度:`accel_up` 0.4–1.0 m/s² 只在 ≤ 1 m/s 量过;用户判断真车更快,待测,若真更快则仿真单圈偏悲观。
- 起步:死区空等是我们自己的 PWM 爬升(20 PWM/s)造成的,可改成先跳到破静点下方再爬,待测。
- 刹车:只按滑行减速(0.73–0.78 m/s²)规划;电调很可能有刹车(中位以下),待测,有的话入弯能更晚松油。
- 假 YOLO 的误差参数(`YoloSimConfig`)、假地面分割的误差参数(`FloorSegConfig`)是 [假设]。
- 位姿用仿真真值,实车用编码器 + 陀螺积分,没有漂移模型。
- 权重未入库,备份在本地 `runs/vm_backup_1010/`(v2m3、v2m2、v2g、bev_v1 等);发布时另开 PR 放进 `models/`。
