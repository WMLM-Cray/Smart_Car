"""DAgger:让学生自己开,在它实际走到的状态上问老师该怎么打,迭代补数据。

背景(2026-09-18~19 的实验):
  纯追踪老师在 s3 上满速 20/20 零撞锥,48 扇区雷达观测也足够唯一确定正确转角
  (没训过的赛道上克隆误差 0.24°)。但一次性行为克隆闭环 0/20 —— 老师只走中线,
  学生一旦偏出去就进了训练集里不存在的状态,误差平方放大。加随机噪声采恢复样本
  能到 62%,但噪声幅度是拍脑袋猜的。PPO 微调三次都在把成绩越搞越低(62->57->43)。
  DAgger 把"猜哪些偏离状态要紧"换成"测出来":标注密度自动堆到学生真正犯错的地方。

每轮:学生跑 -> 记录它走到的状态 -> 老师在这些状态上给标签 -> 并入数据集 -> 重训。
第 0 轮没有学生,用老师自己跑(带噪声,先有个能开的起点)。

用法:python -m fastsim.legacy.dagger [stage] [轮数] [网络宽度]
"""
import sys
import time

sys.path.insert(0, '.')

import numpy as np
import torch
from torch import nn

from fastsim.curriculum import STAGES, evaluate, with_seeded_track
from fastsim.preview import pure_pursuit
from fastsim.train import FRAMES

THROTTLE = 1.0          # 老师满速 —— 实测这条赛道满速就能跑完,不需要减速
ACT_NOISE = 0.15        # 只用在第 0 轮:老师自己跑时抖一下转角,先凑点恢复样本
SEEDS_PER_ITER = 40
BATCH = 512
UPDATES_PER_ITER = 6000
# 挑检查点用验证种子,别用官方评估种子 —— 逐轮看着 90000+ 挑最好的那一版,
# 等于把评估集当成了调参集,最后报出来的数字是虚的。
# 30 个种子不够:2026-09-19 实测从第 13 轮起每轮都是 30/30 圈 0 撞锥,指标饱和,
# "选最好的检查点"退化成了"选最早满分的那个",结果比不选还差。50 个 + 把总撞锥
# (含出界局的)当第三关键字,才有分辨率。
VAL_SEEDS = list(range(91000, 91050))
EVAL_SEEDS_FULL = list(range(90000, 90060))


def make_net(dim, width=128, frames=FRAMES):
    # 128 是照搬 PPO 的 net_arch。聚合数据上它欠拟合:只用老师数据时拟合误差 6e-5
    # (转角 0.24°),聚合 6 轮之后 2.7e-3(1.7°)—— 学生走到的状态比中线复杂得多。
    return nn.Sequential(nn.Linear(dim * frames, width), nn.Tanh(),
                         nn.Linear(width, width), nn.Tanh(), nn.Linear(width, 2))


class Student:
    """policy(obs, env) -> action,签名对齐 curriculum.evaluate。"""

    def __init__(self, net, dim, frames=FRAMES):
        self.net, self.dim, self.frames = net, dim, frames
        self.reset()

    def reset(self):
        self.buf = np.zeros(self.dim * self.frames, np.float32)

    def push(self, obs):
        self.buf = np.roll(self.buf, -self.dim)
        self.buf[-self.dim:] = obs
        return self.buf

    def __call__(self, obs, env=None):
        with torch.no_grad():
            return self.net(torch.from_numpy(self.push(obs)[None])).numpy()[0]


def rollout(stage, seeds, dim, student, rng, frames=FRAMES, lookahead=1.2,
            teacher=None, car=None):
    """跑一遍,返回(叠帧观测, 老师在该状态下的动作)。

    student=None 就是老师自己开(第 0 轮),执行时加噪声但标签仍用干净动作;
    否则由学生开,标签是老师在学生走到的这个状态上会给的动作 —— 这是 DAgger 的全部。
    """
    env = stage.make_env(mapping=False) if car is None else stage.make_env(car, mapping=False)
    X, Y, reasons = [], [], {}
    for seed in seeds:
        obs, _ = env.reset(seed=seed)
        buf = np.zeros(dim * frames, np.float32)
        if student is not None:
            student.reset()
        if teacher is not None and hasattr(teacher, 'reset'):
            teacher.reset()
        while True:
            buf = np.roll(buf, -dim)
            buf[-dim:] = obs
            label = (pure_pursuit(env, lookahead, THROTTLE) if teacher is None
                     else np.asarray(teacher(obs, env), np.float32))
            X.append(buf.copy()); Y.append(label)
            if student is None:
                act = label.copy()
                act[1] = np.clip(act[1] + rng.normal(0, ACT_NOISE), -1, 1)
            else:
                act = student(obs)
            obs, _, term, trunc, info = env.step(act)
            if term or trunc:
                break
        r = info['termination_reason']
        reasons[r] = reasons.get(r, 0) + 1
    return np.array(X, np.float32), np.array(Y, np.float32), reasons


def fit(net, opt, X, Y, updates):
    xt, yt = torch.from_numpy(X), torch.from_numpy(Y)
    tot = 0.0
    for _ in range(updates):
        b = torch.randint(0, len(xt), (BATCH,))
        loss = ((net(xt[b]) - yt[b]) ** 2).mean()
        opt.zero_grad(); loss.backward(); opt.step()
        tot += loss.item()
    return tot / updates


def main(name, iters, width, resume=None, seed0=500, hard=False):
    stage = STAGES[name]
    # hard:只把"采数据"的赛道偏向已知的两类难点,评估仍然用标准 stage。
    # 2026-09-19 的残留失败全在这两类上 —— 60 条赛道里 58 条干净完赛,出问题的是
    # 挡板正好立在紧弯里(遮住锥桶视线)和半径 1.0~1.2m 的极限弯。标准赛道随机抽
    # 很少抽到,DAgger 也就补不到这些状态。
    # 难赛道和标准赛道交替采:纯难赛道练出来的策略会靠蹭锥桶硬挤紧弯,
    # 2026-09-19 实测圈数上到 20/20,完赛撞锥却从 2 涨到 20。
    hard_stage = with_seeded_track(stage, bend_radius_min=1.0, bend_radius_max=1.2, n_boards=8)
    dim = stage.make_env(mapping=False).observation_space.shape[0]
    rng = np.random.default_rng(0)
    net = make_net(dim, width)
    if resume:
        # 只接权重,不接数据集 —— 第 0 轮照旧用老师采一批打底,避免空数据集上过拟合
        ck = torch.load(resume)
        net.load_state_dict(ck['state_dict'])
        print(f'从 {resume} 续训(宽度 {ck.get("width", "?")})', flush=True)
    opt = torch.optim.Adam(net.parameters(), 1e-3)
    X = np.zeros((0, dim * FRAMES), np.float32)
    Y = np.zeros((0, 2), np.float32)

    best, best_key = None, None
    for it in range(iters):
        seeds = list(range(seed0 + it * SEEDS_PER_ITER, seed0 + (it + 1) * SEEDS_PER_ITER))
        t = time.time()
        who = None if it == 0 else Student(net, dim)
        use_hard = hard and it % 2 == 1
        nx, ny, reasons = rollout(hard_stage if use_hard else stage, seeds, dim, who, rng)
        X, Y = np.concatenate([X, nx]), np.concatenate([Y, ny])
        loss = fit(net, opt, X, Y, UPDATES_PER_ITER)
        r = evaluate(Student(net, dim), stage, seeds=VAL_SEEDS)
        key = (r.laps, -r.lap_cone_hits, -r.cone_hits)
        if best_key is None or key > best_key:
            best_key = key
            best = {k: v.clone() for k, v in net.state_dict().items()}
        print(f'第 {it} 轮  {"难" if use_hard else "标"}赛道 {len(nx):>6} 帧 {reasons}  '
              f'数据集 {len(X):>6}  拟合 {loss:.5f}  '
              f'验证 {r.laps:>2}/{len(VAL_SEEDS)} 圈 完赛撞锥 {r.lap_cone_hits:>3} 总撞锥 {r.cone_hits:>3}  '
              f'{time.time() - t:.0f}s', flush=True)

    # 最后一轮的权重也存一份:验证集在这个水平上已经分辨不出差异了,
    # "选中的"和"最后的"孰优要拿更大的种子集另比,别把其中一个丢掉。
    torch.save(dict(state_dict=net.state_dict(), obs_dim=dim, frames=FRAMES, width=width),
               f'runs/dagger_{name}_w{width}_last.pt')
    net.load_state_dict(best)
    print(f'-- 按验证集选中的检查点: {best_key[0]}/{len(VAL_SEEDS)} 圈,完赛撞锥 {-best_key[1]}')
    r = evaluate(Student(net, dim), stage, seeds=EVAL_SEEDS_FULL)
    reasons = {}
    for e in r.episodes:
        reasons[e['termination_reason']] = reasons.get(e['termination_reason'], 0) + 1
    print(f'== 最终 60 种子: {r.laps}/60 圈 ({r.laps / 60:.0%})  完赛撞锥 {r.lap_cone_hits}  '
          f'总撞锥 {r.cone_hits}  建图覆盖率 {r.mean_coverage:.3f}  {reasons}')
    out = f'runs/dagger_{name}_w{width}.pt'
    torch.save(dict(state_dict=net.state_dict(), obs_dim=dim, frames=FRAMES, width=width), out)
    print('存到', out)


if __name__ == '__main__':
    args = sys.argv[1:]
    main(args[0] if args else 's3',
         int(args[1]) if len(args) > 1 else 5,
         int(args[2]) if len(args) > 2 else 128,
         args[3] if len(args) > 3 else None,
         int(args[4]) if len(args) > 4 else 500,
         len(args) > 5 and args[5] == 'hard')
