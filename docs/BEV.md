# BEV 框架与 bev_v1 模型

把雷达回波和摄像头检测按位姿累积成俯视语义栅格(BEV),策略吃这张图而不是单帧扇区距离。
动机:b036(road_v2.5)剩下的失败集中在稀桶、方角开口处,单帧里桶太少、"认错路"。

## 代码在哪

| 文件 | 内容 |
|---|---|
| `common/bev.py` | BEV 本体:分辨率、裁剪窗、格子类型、来源层、锥桶地标合并、过期清理。仿真和实车共用,只依赖 numpy |
| `common/bev_policy.py` | bev_v1 的输入规格(图 + 标量)、网络结构、纯 numpy 推理 `BevPolicy` |
| `simulation/fastsim/camera.py` | FastSim 的假 YOLO:视场、有效距离、遮挡、漏检、位置误差、误检 |
| `training/bev_env.py` | `BevEnv`(RoadEnv + BEV + 假 YOLO)、`SeenTeacher`(道路几何只用雷达已扫到、未过期的边界;合同见文件头) |
| `training/bev_net.py` / `bev_train.py` | torch 网络(与 numpy 前向逐元素一致)、DAgger 训练 |
| `evaluation/bev_track.py` | 官方 A–D 评估,和 `evaluation/road_track.py` 同口径 |
| `evaluation/bev_testset.py` | 随机路段配对测试集:生成(带指纹)、跑模型、McNemar 配对比较 |

约定(分辨率、格子类型、输入通道、网络结构)写在 `common/bev.py`、`common/bev_policy.py` 的文件头,
以那里为准;训练出来的权重和这些数字绑定,改了要换 schema。要点:

- 栅格 5 cm,窗口车前 6 m、车后 2 m、左右各 4 m(160×160);格子类型 UNKNOWN / FREE(预留)/
  OBSTACLE / CONE / TAPE / ZONE / TRAFFIC_LIGHT,冲突按 CONE > TRAFFIC_LIGHT > OBSTACLE > TAPE > ZONE。
- 锥桶按"地标"存:0.25 m 内的检测合并、至少 2 次命中才画。所有证据 20 s 没再更新就清掉(限制里程计漂移)。
- 模型输入:5×80×80(10 cm/格:雷达记忆、锥桶地标、胶带、区域、本帧雷达)+ 3 个标量(车速、限速、
  偏航角速度);不放上一步动作。输出同 road_v2.5(转向、目标速度,`spec_of` 解析速度契约)。
- 网络:4 层 stride-2 卷积 + 2 层 256 全连接,约 54 万参数,numpy 前向约 1.4 ms;全是标准算子,便于转 ONNX 上 NPU。

## 训练与评估

```sh
python -m training.bev_train --out runs/bev_v1_a --rounds 40 --replay 1500000 --workers 7
python -m training.bev_train --out runs/bev_v1_d --init runs/bev_v1_c/candidate_014.npz --beta0 0 \
  --cap-max 2.5 --a-lat 1.47 --rounds 15 --replay 1500000 --workers 7
python -m evaluation.bev_track --model runs/bev_v1_d/candidate_014.npz --caps 1.8 2.2 2.5 --count 16
```

训练只用随机路段(L3–L6 混合),A–D 不进训练集。课程:

| 阶段 | 起点 | 限速上限 | 老师弯中横向加速度 | 轮数 |
|---|---|---|---|---|
| a | 从零(前几轮老师开) | 1.8 | 0.45 m/s² | 40 |
| b | a/candidate_020 | 2.2 | 0.45 | 15 |
| c | b/candidate_014 | 2.5 | 0.45 | 15 |
| d | c/candidate_014 | 2.5 | 1.47(0.15 G) | 15 |

## 结果(FastSim,未实车验证)

> 下表是**修正前**的代码训练和评估的(a 段 bb5f415,b–d 段 8278c1d):本帧雷达按当前位姿投影(与记忆层
> 错开),老师在已知前方不足 0.5 m 时、以及初始化/首次重规划失败时用过真值道路。PR #30 评审后已修正,
> 要用新代码重训重测,数字会更新。

官方 A–D:黄 V 补桶、种子 95M 起、每档 16 局(车辆参数每局随机),第一圈平均秒数;各档学生全部通过,
除注明外。

| 模型 | 1.8 | 2.2 | 2.5 |
|---|---|---|---|
| b036(road_v2.5) | 29.8 | 29.6 | C 15/16(1 次碰撞) |
| bev_v1 a/candidate_020 | 29.7 | – | – |
| bev_v1 b/candidate_014 | 30.1 | 28.7 | 27.6 |
| bev_v1 c/candidate_014 | 30.0 | 29.0 | 28.4 |
| bev_v1 d/candidate_014 | 22.4 | 20.7 | 20.2 |

随机路段测试集:L5 / L6h / L4c / L6f 各 400 局(种子 93M 起、限速 ≤ 1.8,与 b036 同一批局、同噪声种子)。
由 `python -m evaluation.bev_testset build` 生成,指纹 `76fe401b818fa635e56d9949beadb9d7`(和训练机上的缓存逐局一致)。

| 模型 | 成功率 | 配对比较(b036 失败而它成功 / 反之) |
|---|---|---|
| b036 | 95.87% | – |
| bev_v1 a/candidate_020 | 98.94% | 58 / 9,McNemar p < 0.0001 |
| bev_v1 b/candidate_014 | 98.50% | 55 / 13 |
| bev_v1 c/candidate_014 | 98.56% | 56 / 13 |
| bev_v1 d/candidate_014 | 98.81% | 54 / 7;和 a/candidate_020 比 7 / 9,差异不显著 |

- 只提限速(b、c)圈速几乎不变:弯里被老师的横向加速度上限卡住。d 段放到 0.15 G 后 A–D 快约 25%,
  限速 ≤ 1.8 的测试集上成功率和 a 持平;训练时限速到 2.5 的随机路段评估从 ~0.984 降到 ~0.97
  (L6f、L4c 碰撞增多),高速档的安全代价还在。
- 车上电调不刹车,只能滑行减速(实测约 0.73 m/s²),入弯前的松油距离按这个规划。

## bev_v2:加可行驶地面层

第 6 个输入通道 = 摄像头地面分割(实车 SegFormer-B0 ADE20K)投到地面后登记的 FREE 层。网络只有第一层卷积
多一个输入通道,可从 bev_v1 热启动(新通道权重置 0,起点行为和 bev_v1 完全一样)。

- FastSim 的假地面分割(`simulation/fastsim/camera.py` 的 `FakeFloorSeg`)语义和实车通用分割一致:
  锥桶 / 胶带边界**内外**的地板都算可行驶,只有墙外不算,扣掉锥桶底座。锥桶赛道的路沿只能靠锥桶层和雷达认。
  最初一半的局"只认路内",等于给了模型实车拿不到的路沿信号,已删(2026-10-10),旧的 `bev_v2_a` 作废。
- 训练:`--schema bev_v2 --init <bev_v1 d> --beta0 0 --cap-max 2.5 --a-lat 1.47 --rounds 12`。
- 结果(`bev_v2g/candidate_011`):A–D 三档全过,1.8 / 2.5 平均 22.3 / 20.0 s(d 22.4 / 20.2);
  随机路段测试集 98.44%,同一批局上 d 98.56%,配对 7 / 9,McNemar p = 0.80,不显著。
  测试集用当前代码重新生成,指纹 `919e6387cd1fb4c8aa70692e2eafe7b7`,和旧缓存只差 1 局(seed 93000380,
  可通性拒绝采样卡在边界上)。
- 在锥桶赛道上地面层不提供路沿信息;它有用的地方是路面本身和周围不同、雷达又扫不到的边界(草地边、
  马路牙子、低于雷达扫描面的障碍物),FastSim 里目前没有这类边界。

## 待定 / 假设

- 相机位置 `camera_x`、`camera_z`、`camera_pitch` 是 [假设],待实测后改 `common/params.py`。
- 假 YOLO 的漏检率、位置误差、误检率、遮挡阈值(`YoloSimConfig`)是 [假设],需要实车标定。
- 位姿用 FastSim 真值(实车雷达里程计漂移很小,暂不建漂移模型)。
- 权重未入库:`bev_v1_a/candidate_020`(md5 eaa107607ef9006f343b542d75a4781b,代码 bb5f415)、
  `bev_v1_d/candidate_014`(md5 9644b6cd7269b248f40d1b5286c57562,代码 8278c1d)在训练虚拟机上,
  发布时另开 PR 放进 `models/`。
