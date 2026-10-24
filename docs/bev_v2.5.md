# bev_v2.5 原生 5 cm 近场分支

对应 issue #48，基于要求的 `feat/real-map-replay`（初始实现基线 cb9f377，PR 提交时同步至 3cd901f）。同步保留上游地面帧新鲜度、各层独立过期与实景回放启动逻辑。

## 输入和网络

主图沿用 bev_v2：6×80×80、10 cm/格，后轴为原点，车前 6 m、车后 2 m、左右各 4 m。近场图是同一次原始 5 cm BEV 裁图的 `[:, 68:132, 48:112]`，形状 6×64×64，车前 2.6 m、车后 0.6 m、左右各 1.6 m，不经过池化。两图通道顺序均为雷达记忆、锥桶、胶带、区域、本帧雷达、FREE。

`common.bev_policy.bev_images` 一次构造两图。扫描用扫描时刻位姿投影，两图共享裁图位姿；训练增强时也共享同一次平移和旋转。近场做 2×2 max 后严格等于主图的 `[:, 34:66, 24:56]`。雷达记忆先量化到 odom 格心，当前扫描直接投影原始点，旋转重裁时二者可以落入相邻格。

主分支为 6→16（5×5）→32→64→64；近场分支为 6→16→32→32→32（均为 3×3）；所有卷积 stride=2、同尺寸 padding、ReLU。主分支展平 1600 维，近场展平 512 维，与共享的 3 个标量拼接，接 256→256→2 全连接；隐藏层 tanh，推理输出裁到 ±1。共 **694,498** 个参数。

### FREE 层旋转采样（issue #51）

FREE 表示有地面证据的面积。裁图对每个目标格中心反查 odom 源格的 FREE 证据，避免格心正向投点在旋转后重复落格、留下假空洞。源格未知或该层证据已过期仍输出0；不做形态学填洞，也不会自动补成可行驶路面。类型图与模型输入共用这条采样路径，障碍和锥桶仍优先覆盖类型图中的 FREE。

雷达记忆、锥桶、胶带、区域和当前雷达的采样逻辑不变。`BevEnv` 和 `BevRuntime` 共用 `Bev.crop_layers`；bev_v2 与 bev_v2.5 的 FREE 输入都会更新，5cm近场与10cm主图的2×2 max对齐关系保持不变。

这会改变旧 bev_v2 模型收到的少量 FREE 格，旧NPZ的schema、形状和网络前向不变，但闭环输出不保证与修复前相同。复用旧模型前应重评，后续训练与车端应使用同一版本的裁图代码。裁图修复不代表闭环成绩提升，未实车验证。

修复在GCP的独立源码快照全量验证：`OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 MKL_NUM_THREADS=1 python -m pytest -q`，**403 passed, 5 subtests passed**（96.55秒）。回归覆盖5/10cm格、0/5/15/30/45/90/−30/−135°旋转、负坐标与亚格平移、未知孔及边界、独立过期与prune、稀疏层及类型优先级、主图近场对齐、旋转状态下仿真与运行时输入一致性。

同一GCP CPU、NumPy单线程的`crop_layers`短时微基准：位姿(0.013, 0.017, 0.37rad)，预热10次后测100次。无FREE的4000格雷达地图，修复前/后中位数0.289/0.292ms、p95 0.316/0.321ms；包含44000格FREE的地图，中位数3.629/3.676ms、p95 3.983/4.035ms。无FREE时快速返回。数字仅表示这两张地图的裁图开销，不是整网推理时延或车端实时性结论。

## 接口和兼容

- `BevPolicy(img, sc, near)`、Torch 网络 `net(img, sc, near)`；批量维度只用于 Torch。
- `BevEnv(..., free=True, near=True).observation()` 返回 `(img, sc, near)`；旧 schema 仍返回 `(img, sc)`。
- schema 示例：`bev_v2.5_s3_v0.45-3`。先匹配最长前缀，再由 `common.road_policy.spec_of` 解析速度契约。
- NPZ 保留 `schema`、`c0w/c0b…c3w/c3b`、`f0w/f0b…f2w/f2b`；增加 `n0w/n0b…n3w/n3b`。f0 输入顺序为主图、近场、标量。
- `load_into` 支持 v1/v2→v2.5：复制主分支和标量接口，新增近场 FC 列置零，近场卷积保留随机初始化。v1→六通道时新增 FREE 卷积权重置零。禁止将 v2.5 加载到旧网络。
- 旧 schema 的 NumPy forward 运算顺序保持原样。已发布旧 NPZ 逐元素相等回归、旧训练接口回归均有测试。
- 实车模型加载和运行时按 schema 自动选择双分支；last_img 继续供网页显示主图。评估入口 bev_track、bev_testset、real_map_replay、bev_sweep、l_test、tee_test、bev_video、sim_video 已接入。

回放按位压缩：每帧主图 4800 字节、近场 3072 字节。30 万帧两图约 2.36 GB；一致性训练保存第二对图时约 4.72 GB，另加标量与标签等开销。回放覆盖、解包、增强两图对齐均有回归测试。

## 验证（2026-10-19）

使用 Python 3.10.12、NumPy 1.26.4、SciPy 1.13.1、PyTorch 2.4.1+cpu、pytest 8.3.5，限制 BLAS/OMP 为单线程。

```sh
python -m pytest -q
```

同步上游并解决冲突后的最终结果：**380 passed**（189.21 s）；上游回放启动测试单独复测 **6 passed**。覆盖原生 5 cm 格与所有通道、裁图边界、扫描位姿、NumPy/Torch 数值一致、NPZ 导出/加载、旧模型行为、warm start、回放和运行时/训练输入一致。

两轮训练冒烟（输出目录须不存在）：

```sh
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 MKL_NUM_THREADS=1 \
python -m training.bev_train --schema bev_v2.5 \
  --out runs/issue48-v25-smoke-pr-final --rounds 2 \
  --episodes-per-round 1 --updates 1 --batch 2 --replay 32 \
  --workers 1 --eval-count 2 --eval-levels L3 --mix L3:1 \
  --torch-threads 1 --cap-max 1.0 --aug 0.05 1 --consist 1

python -m evaluation.l_test \
  runs/issue48-v25-smoke-pr-final/candidate_001.npz \
  --seeds 1 --workers 1 --widths 2.5
```

两轮均完成采集、成对增强、一致性损失、反向更新、EMA 评估和 NPZ 导出，同步上游后重新运行，loss 分别为 0.1254、0.0501。FastSim L3 采集种子 30000000、30000001，各 1/1 到达终点（含老师混合控制）；每轮评估种子 60000000–60000001、限速采样上限 1.0 m/s，均 0/2，均 collision。两次更新仅验证流程，不代表训练效果。

导出模型通过 L 测试 CLI：FastSim、限速 0.7 m/s、宽 2.5 m、种子 7000、朝向 -10/0/10°，共 3 局，0/3，均 guard。另完成 sweep 一档路宽/三个偏移、固定测试集一局（collision）、T 字口一局（guard），以及同步上游后合成走廊地图的 real_map_replay 短程执行（种子 48、限速 0.7 m/s、预算 2 s，等待新鲜地面帧后正常调用双分支模型，40 帧、约 1.03 m，timeout）。这些检查验证双分支接口能闭环执行，不构成策略质量评估；实景回放使用合成地图替代文件载入，未使用实车日志。

Windows worktree 在 WSL 执行训练时，已有的 git 元数据采集命令无法解析 Windows 绝对路径，config.json 的 git_head 为空；训练和导出正常。本次基线由上方提交号记录，未为此修改 Git 元数据。

指定基线没有 boundary_test.py，未新增含义不明确的边界评估器；裁图边界由单元测试覆盖。未开展正式训练对照，未实车验证，未测车载推理 p95，也未部署。运行产物仅存 runs/，不提交。
