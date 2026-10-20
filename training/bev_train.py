"""bev_v1/v2 与 bev_v2.5 的 DAgger 训练:老师 = SeenTeacher(只用已扫到信息),学生 = BEV 卷积网络。

    python -m training.bev_train --out runs/bev_v1_a --rounds 40 --workers 7

流程和 road_train 一样:每轮按 beta 混合老师/学生采一批局(β 1 -> 0,前几轮基本是老师开 =
行为克隆式预训练),老师逐拍给标签进回放池,训 updates 步,固定种子评估,导出 candidate_XXX.npz。
回放图按位压缩。bev_v2.5 每帧主图 4.8 KB + 近场图 3 KB,30 万帧约 2.36 GB。
"""
from __future__ import annotations

import os

for _k in ('OMP_NUM_THREADS', 'OPENBLAS_NUM_THREADS', 'MKL_NUM_THREADS'):
    os.environ.setdefault(_k, '1')

import argparse
import math
import json
import time
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path

import numpy as np

from common.bev_policy import (C2, H, NEAR_SIZE, N_SCALAR, SCHEMA, SCHEMA_V2, SCHEMA_V25, W,
                               channels_of, forward, forward_v25)
from common.road_policy import V_LO, Encoding, spec_of
from simulation.fastsim.road import LEVELS
from training.bev_env import BevEnv, SeenTeacher
from training.road_check import sample_episode

TRAIN_SEED0, SELECT_SEED0 = 30_000_000, 60_000_000     # 和 road_train(10M/50M)、测试(90M+)都不重叠


def run_episode(seed, level, weights=None, beta=0.0, collect=False, enc=None, a_lat=0.45, cap_max=None,
                free=False, seg=(0.1, 0.0), aug=(0.0, 0.0), pair=False, lag_max=None, yolo=True, guard=False,
                seg_lut=False, seg_fill=False, cap_min=None, near=False):
    """seg = (地面分割周期 s, 推理延迟 s),aug = (裁图平移抖动 m, 转角抖动 rad),见 BevEnv。
    pair:采集时每帧再裁一张(另一种亚格偏移),给一致性损失;lag_max:指令延迟拍数改成每局 0..lag_max 均匀抽。"""
    try:
        road, pl, start, P, _ = sample_episode(seed, level, enc=enc, a_lat=a_lat, cap_max=cap_max, cap_min=cap_min)
    except RuntimeError:
        return None
    if lag_max is not None:
        from dataclasses import replace
        pl = replace(pl, lag=int(np.random.default_rng(seed + 17).integers(0, lag_max + 1)))
    benv = BevEnv(road, pl, seed, start, noise=True, free=free, seg_period=seg[0], seg_lag=seg[1], aug=aug,
                  yolo=yolo, guard=guard, seg_lut=seg_lut, seg_fill=seg_fill, near=near)
    teacher = SeenTeacher(benv, P, a_lat)
    rng = np.random.default_rng(seed + 1)
    imgs, near_imgs, scs, ys, imgs2, near_imgs2 = [], [], [], [], [], []
    why, n = None, 0
    d = pl.enc.dead
    n_stop = n_brake = n_brake_label = 0          # 速度动作落在中位段 / 刹车段的拍数(只在带 _d 的契约下有)
    while why is None:
        obs = benv.observation()
        if near:
            img, sc, near_img = obs
        else:
            img, sc = obs
            near_img = None
        label = teacher()
        n_brake_label += bool(d) and label[1] <= -d
        if collect:
            imgs.append(np.packbits(img.astype(bool).ravel()))
            if near:
                near_imgs.append(np.packbits(near_img.astype(bool).ravel()))
            if pair:
                if near:
                    img2, near_img2 = benv.images()
                    near_imgs2.append(np.packbits(near_img2.astype(bool).ravel()))
                else:
                    img2 = benv.image()
                imgs2.append(np.packbits(img2.astype(bool).ravel()))
            scs.append(sc)
            ys.append(label)
        if weights is None or rng.random() < beta:
            act = label + (rng.normal(0, [0.05, 0.05]) if collect else 0.0)
        else:
            act = forward_v25(weights, img, sc, near_img) if near else forward(weights, img, sc)
        n_stop += bool(d) and abs(act[1]) < d
        n_brake += bool(d) and act[1] <= -d
        why = benv.step(act)
        n += 1
    row = dict(seed=seed, level=level, reason=why, cap=pl.cap, time=benv.env.t, steps=n,
               reverse_s=benv.env.speed.reverse_s,                 # 低速还在刹的时间(实车可能倒车,见 SpeedPlant)
               stop_frac=n_stop / n, brake_frac=n_brake / n, brake_label_frac=n_brake_label / n)
    if collect:
        common = (np.asarray(imgs), np.asarray(scs, np.float32), np.asarray(ys, np.float32), row,
                  (np.asarray(imgs2) if pair else None))
        if near:
            return common + (np.asarray(near_imgs), np.asarray(near_imgs2) if pair else None)
        return common
    return row


def _job(args):
    return run_episode(*args)


class Replay:
    def __init__(self, cap, c, pair=False, near=False, disk=None):
        """disk:图放到这个目录下的 np.memmap 文件(bev_v2.5 成对 150 万帧要 23 GB,虚拟机 30 GB 放不下),
        内存只留操作系统缓存得下的部分;标量和标签仍在内存。"""
        self.c, self.bits, self.near = c, c * H * W, near
        self.near_bits = c * NEAR_SIZE * NEAR_SIZE if near else 0

        def zeros(name, bits):
            shape = (cap, (bits + 7) // 8)
            if disk is None:
                return np.zeros(shape, np.uint8)
            disk.mkdir(parents=True, exist_ok=True)
            return np.memmap(disk / f'{name}.u8', np.uint8, 'w+', shape=shape)
        self.img = zeros('img', self.bits)
        self.img2 = zeros('img2', self.bits) if pair else None      # 同一帧另一种亚格偏移(一致性损失)
        self.near_img = zeros('near_img', self.near_bits) if near else None
        self.near_img2 = zeros('near_img2', self.near_bits) if pair and near else None
        self.sc = np.zeros((cap, N_SCALAR), np.float32)
        self.y = np.zeros((cap, 2), np.float32)
        self.size = self.ptr = 0

    def add(self, img, sc, y, img2=None, near_img=None, near_img2=None):
        for i in range(len(y)):
            self.img[self.ptr], self.sc[self.ptr], self.y[self.ptr] = img[i], sc[i], y[i]
            if self.img2 is not None:
                self.img2[self.ptr] = img2[i]
            if self.near:
                self.near_img[self.ptr] = near_img[i]
                if self.near_img2 is not None:
                    self.near_img2[self.ptr] = near_img2[i]
            self.ptr = (self.ptr + 1) % len(self.y)
            self.size = min(len(self.y), self.size + 1)

    def batch(self, rng, n):
        i = rng.integers(0, self.size, n)
        def unpack(a, bits, shape):
            return np.unpackbits(a[i], axis=1)[:, :bits].reshape((n,) + shape).astype(np.float32)
        img = unpack(self.img, self.bits, (self.c, H, W))
        img2 = unpack(self.img2, self.bits, (self.c, H, W)) if self.img2 is not None else None
        if not self.near:
            return img, self.sc[i], self.y[i], img2
        near_img = unpack(self.near_img, self.near_bits, (self.c, NEAR_SIZE, NEAR_SIZE))
        near_img2 = unpack(self.near_img2, self.near_bits, (self.c, NEAR_SIZE, NEAR_SIZE)) if self.near_img2 is not None else None
        return img, self.sc[i], self.y[i], img2, near_img, near_img2

def summarize(rows):
    ok = [r['reason'] == 'goal' for r in rows]
    return dict(n=len(rows), success=float(np.mean(ok)) if rows else 0.0,
                reasons={k: sum(r['reason'] == k for r in rows) for k in {r['reason'] for r in rows}},
                reverse_eps=sum(r.get('reverse_s', 0.0) > 0 for r in rows),
                # 执行的动作里中位 / 刹车的比例,和老师刹车标签的比例(老师从不给中位标签:学生停下来是回归把两种标签平均进了死区)
                **{k: float(np.mean([r[k] for r in rows])) if rows else 0.0
                   for k in ('stop_frac', 'brake_frac', 'brake_label_frac')})


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--out', type=Path, required=True)
    ap.add_argument('--rounds', type=int, default=40)
    ap.add_argument('--episodes-per-round', type=int, default=168)
    ap.add_argument('--updates', type=int, default=1500)
    ap.add_argument('--batch', type=int, default=256)
    ap.add_argument('--lr', type=float, default=1e-3)
    ap.add_argument('--replay', type=int, default=300_000)
    ap.add_argument('--replay-disk', type=Path, default=None,
                    help='回放池的图放到这个目录(np.memmap,内存不够时用;跑完可删)')
    ap.add_argument('--ema', type=float, default=0.999)
    ap.add_argument('--device', default='cpu', help='网络更新用的设备:cpu / mps(Apple GPU)/ cuda;采样始终在 CPU 进程里')
    ap.add_argument('--workers', type=int, default=7)
    ap.add_argument('--torch-threads', type=int, default=4)
    ap.add_argument('--seed', type=int, default=0)
    ap.add_argument('--speed-scale', type=float, default=3.0)
    ap.add_argument('--v-hi', type=float, default=3.0)
    ap.add_argument('--a-lat', type=float, default=0.45)
    ap.add_argument('--cap-max', type=float, default=1.8)
    ap.add_argument('--cap-min', type=float, default=None, help='速度上限抽取下限(默认契约的 v_lo)')
    ap.add_argument('--contract', default=None,
                    help='完整速度契约后缀,比如 _s5_v0.45-5_d0.2_log(带停车 / 刹车死区、对数刻度);'
                         '给了就不看 --speed-scale / --v-hi')
    ap.add_argument('--mix', default='L5:0.2,L5s:0.2,L6h:0.15,L6f:0.15,L4:0.15,L6:0.1,L3:0.05')
    ap.add_argument('--eval-levels', default='L5,L6h,L4c,L6f')
    ap.add_argument('--eval-count', type=int, default=192)
    ap.add_argument('--init', type=Path, default=None, help='从已有 bev_v1 模型接着训(课程下一阶段)')
    ap.add_argument('--beta0', type=float, default=1.0, help='第 0 轮老师开的比例;--init 续训时可给 0')
    ap.add_argument('--seg-period', type=float, nargs=2, default=(0.1, 0.1), metavar=('LO', 'HI'),
                    help='bev_v2 采集时每局地面分割周期在 [LO, HI] 均匀抽(实车 M2F-T 288 ~0.33 s)')
    ap.add_argument('--seg-lag', type=float, nargs=2, default=(0.0, 0.0), metavar=('LO', 'HI'),
                    help='采集时每局地面分割延迟在 [LO, HI] 均匀抽(实车 ~0.35 s)')
    ap.add_argument('--eval-seg', type=float, nargs=2, default=(0.1, 0.0), metavar=('PERIOD', 'LAG'),
                    help='每轮评估用的地面分割周期和延迟(按部署条件挑模型)')
    ap.add_argument('--aug', type=float, nargs=2, default=(0.0, 0.0), metavar=('SHIFT_M', 'YAW_DEG'),
                    help='采集时每拍裁图位姿抖动(±平移 m、±转角 °),治 BEV 格子量化敏感;评估不抖。v4 用 0.05 1')
    ap.add_argument('--consist', type=float, default=0.0,
                    help='一致性损失权重:同一帧两种亚格偏移的输出差的平方(要配 --aug);v4 用 1')
    ap.add_argument('--lag-max', type=int, default=None,
                    help='采集时指令延迟每局 0..N 拍均匀抽(默认按 Plant.sample 的 0..1);实车打方向延迟没量过,v4 用 3')
    ap.add_argument('--no-yolo', action='store_true',
                    help='锥桶/胶带/区域层恒空(车上没跑 YOLO);路上的锥桶照样在,雷达照样扫得到。采集和每轮评估都这样')
    ap.add_argument('--guard', action='store_true',
                    help='按车上的截停规则(扫车身矩形、保险杠前 < 0.25 m)判失败,采集和每轮评估都这样')
    ap.add_argument('--seg-lut', action='store_true',
                    help='假地面分割按车上的像素投影出点(72x96 像素中心,远处条纹;一半帧只到 2 m),采集和评估都这样')
    ap.add_argument('--seg-fill', action='store_true', help='配 --seg-lut:补行间的缝(车上 ground_seg_node --fill)')
    ap.add_argument('--schema', default=SCHEMA, choices=(SCHEMA, SCHEMA_V2, SCHEMA_V25), help='bev_v1(5 通道)/ bev_v2(6 通道)/ bev_v2.5(双图近场分支);支持从旧模型 --init 热启动')
    a = ap.parse_args()
    enc = spec_of('spica_v3' + a.contract) if a.contract else Encoding(a.speed_scale, V_LO, a.v_hi)

    import torch
    from training.bev_net import export, make_net, weights_of
    torch.set_num_threads(a.torch_threads)
    torch.manual_seed(a.seed)
    rng = np.random.default_rng(a.seed)
    c_in = channels_of(a.schema)
    free = c_in == C2
    near = a.schema == SCHEMA_V25
    dev = torch.device(a.device)
    net, ema = make_net(c_in, near=near), make_net(c_in, near=near)
    if a.init is not None:
        from common.bev_policy import BevPolicy
        if BevPolicy(a.init).enc != enc:
            raise SystemExit(f'{a.init} 的速度契约和这次训练({enc})不一致')
        from training.bev_net import load_into
        load_into(net, a.init)
    ema.load_state_dict(net.state_dict())
    net, ema = net.to(dev), ema.to(dev)
    opt = torch.optim.Adam(net.parameters(), lr=a.lr)
    a.out.mkdir(parents=True, exist_ok=False)
    cfg = {k: str(v) for k, v in vars(a).items()}
    cfg.update(enc=tuple(enc), git_head=os.popen('git rev-parse HEAD').read().strip())
    (a.out / 'config.json').write_text(json.dumps(cfg, indent=1), encoding='utf-8')
    log = open(a.out / 'train.log', 'a', encoding='utf-8', buffering=1)

    def say(msg):
        line = f'{time.strftime("%H:%M:%S")} {msg}'
        print(line, flush=True)
        log.write(line + '\n')

    mix = [(n, float(p)) for n, p in (x.split(':') for x in a.mix.split(','))]
    probs = np.array([p for _, p in mix]) / sum(p for _, p in mix)
    names = {l.name for l in LEVELS}
    assert all(n in names for n, _ in mix)
    ev_levels = a.eval_levels.split(',')
    pair = a.consist > 0
    if pair and tuple(a.aug) == (0.0, 0.0):
        raise SystemExit('--consist 要配 --aug(两张图一样,一致性损失恒为 0)')
    replay = Replay(a.replay, c_in, pair, near, a.replay_disk)
    brng = np.random.default_rng(a.seed + 1)
    wy = torch.tensor([1.0, 0.5], device=dev)
    seed = TRAIN_SEED0
    with ProcessPoolExecutor(a.workers) as ex:
        for r in range(a.rounds):
            beta = max(0.0, a.beta0 - r / 4)
            wts = None if (r == 0 and a.init is None) else weights_of(ema)
            jobs = []
            for _ in range(a.episodes_per_round):
                seg = (float(rng.uniform(*a.seg_period)), float(rng.uniform(*a.seg_lag)))
                jobs.append((seed, mix[int(rng.choice(len(mix), p=probs))][0], wts, beta, True, enc,
                             a.a_lat, a.cap_max, free, seg, (a.aug[0], math.radians(a.aug[1])), pair, a.lag_max,
                             not a.no_yolo, a.guard, a.seg_lut, a.seg_fill, a.cap_min, near))
                seed += 1
            t0 = time.time()
            got = [g for g in ex.map(_job, jobs, chunksize=2) if g is not None]
            for g in got:
                if near:
                    img, sc, y, _, img2, near_img, near_img2 = g
                    replay.add(img, sc, y, img2, near_img, near_img2)
                else:
                    img, sc, y, _, img2 = g
                    replay.add(img, sc, y, img2)
            collect = summarize([g[3] for g in got])
            t_collect = time.time() - t0

            t0 = time.time()
            losses = []
            for _ in range(a.updates):
                batch = replay.batch(brng, a.batch)
                if near:
                    img, sc, y, img2, near_img, near_img2 = batch
                    sc = torch.from_numpy(sc).to(dev)
                    pred = net(torch.from_numpy(img).to(dev), sc, torch.from_numpy(near_img).to(dev))
                else:
                    img, sc, y, img2 = batch
                    sc = torch.from_numpy(sc).to(dev)
                    pred = net(torch.from_numpy(img).to(dev), sc)
                loss = (((pred - torch.from_numpy(y).to(dev)) ** 2) * wy).mean()
                if pair:        # 同一帧换一种量化相位,输出该一样:不许按"墙落在哪一格"出转角
                    pred2 = net(torch.from_numpy(img2).to(dev), sc, torch.from_numpy(near_img2).to(dev)) if near                         else net(torch.from_numpy(img2).to(dev), sc)
                    loss = loss + a.consist * (((pred - pred2) ** 2) * wy).mean()
                opt.zero_grad()
                loss.backward()
                opt.step()
                with torch.no_grad():
                    for pe, pn in zip(ema.parameters(), net.parameters()):
                        pe.mul_(a.ema).add_(pn, alpha=1 - a.ema)
                losses.append(loss.detach())             # 不逐步 .item():GPU 上每步同步一次会拖慢
            t_update = time.time() - t0

            t0 = time.time()
            wts = weights_of(ema)
            ev = [e for e in ex.map(_job, [(SELECT_SEED0 + i, ev_levels[i % len(ev_levels)], wts, 0.0, False,
                                             enc, a.a_lat, a.cap_max, free, tuple(a.eval_seg), (0.0, 0.0), False,
                                             None, not a.no_yolo, a.guard, a.seg_lut, a.seg_fill, a.cap_min, near)
                                            for i in range(a.eval_count)],
                                    chunksize=2) if e is not None]
            summ = summarize(ev)
            by = {n: summarize([e for e in ev if e['level'] == n])['success'] for n in ev_levels}
            t_eval = time.time() - t0
            export(ema, a.out / f'candidate_{r:03d}.npz', enc, a.schema)
            say(f'round {r} beta={beta:.2f} samples={replay.size} loss={torch.stack(losses[-200:]).mean().item():.4f} '
                f'collect={collect["success"]:.3f} eval={summ["success"]:.3f} {by} {summ["reasons"]} '
                f'[{t_collect:.0f}s/{t_update:.0f}s/{t_eval:.0f}s]')
    say('done')


if __name__ == '__main__':
    main()
