"""可通性三层检查 + 一局的抽样。训练里的每一局都先过这里。

  1 几何   老师能在路面内规划出 R>=0.95m 的路线;沿路线摆车身,离墙/锥桶 >= SWEEP_CLEAR
  2 闭环   用这一局抽到的全部车辆参数(舵机速度、延迟、平台速度、上限……)让老师真开一遍,
           到终点且全程离障碍 >= RUN_CLEAR
  3 可见   沿路线每 0.5m,前方 ±90° 左右两半至少一边 10m 内有回波 —— 学生只有雷达,
           看不见的路段对它是不适定的

任何一层不过就连路带参数一起重抽,并按原因计数:某类路段作废率高,要改的是生成器。

    python -m simulation.road_check --level L3 --count 500 --workers 14
"""
from __future__ import annotations

import os

# 每个采样进程只用一个 BLAS 线程:32 个进程 x 32 线程会把机器压到负载 400(2026-09-25 实测)。
# 必须在 import numpy 之前设。
for _k in ('OMP_NUM_THREADS', 'OPENBLAS_NUM_THREADS', 'MKL_NUM_THREADS'):
    os.environ.setdefault(_k, '1')

import argparse
import collections
import json
import math
import time
from concurrent.futures import ProcessPoolExecutor

import numpy as np

from common.params import CarParams
from common.road_policy import DEFAULT_ENC
from simulation.fastsim.road import EDGE, LEVELS, generate, generate_loop
from simulation.fastsim.road_env import GOAL_BEFORE_END, Plant, RoadEnv, footprint_clearance
from training.road_teacher import A_LAT, R_MIN, Teacher, plan

SWEEP_CLEAR = 0.08
RUN_CLEAR = 0.05
LOOK_STEP = 0.5


def sample_start(rng, road, recovery: bool):
    """起点:一半从路口正中出发;一半在前 60% 路段随机位置、带横向和航向偏差(学补救)。"""
    if not recovery:
        return 0, 0.0, 0.0
    i = int(rng.integers(0, int(0.6 * len(road.centre))))
    room = max(0.0, road.half[i] - CarParams().width / 2 - 0.3)
    return i, float(rng.uniform(-0.6, 0.6) * room), float(rng.uniform(-0.2, 0.2))


def check(road, pl, start, a_lat: float = A_LAT, r_min: float = R_MIN) -> tuple[bool, str, object]:
    cp = CarParams()
    i0, d0, yaw0 = start
    P = plan(road, i0, d0, cp.width, pl, r_min=r_min, a_lat=a_lat)
    if isinstance(P, str):
        return False, P, None
    # 1 几何:沿路线摆车身
    env = RoadEnv(road, pl, 0, start, noise=False, sense=False)
    car = env.car
    heading = np.arctan2(*np.gradient(P.pts, axis=0)[:, ::-1].T)
    for k in range(0, len(P.pts), 2):
        car.x, car.y, car.yaw = P.pts[k, 0], P.pts[k, 1], heading[k]
        if footprint_clearance(car, road) < SWEEP_CLEAR:
            return False, 'sweep', None
    # 3 可见(放在闭环前面,便宜)。EDGE 路(无锥桶无墙)雷达本来就什么都看不到,路沿靠地面分割,不查
    edge_road = bool((np.asarray(road.mat_left) == EDGE).any() or (np.asarray(road.mat_right) == EDGE).any())
    last = -math.inf
    for k in range(0 if edge_road else len(P.pts)):
        if P.s[k] > P.s[-1] - GOAL_BEFORE_END:     # 过了终点线就结束了,路尽头前面本来就空
            break
        if P.s[k] - last < LOOK_STEP:
            continue
        last = P.s[k]
        r = env.scan_at(P.pts[k, 0], P.pts[k, 1], heading[k])
        rel = (env.angles + math.pi) % (2 * math.pi) - math.pi
        seen = r < 10.0
        if not (seen & (rel > 0) & (rel < math.pi / 2)).any() and \
                not (seen & (rel < 0) & (rel > -math.pi / 2)).any():
            return False, 'blind', None
    # 2 闭环:老师用这一局的参数真开
    env = RoadEnv(road, pl, 0, start, noise=False, sense=False)
    teacher = Teacher(env, P, a_lat=a_lat)
    why = None
    while why is None:
        why = env.step(teacher())
    if why != 'goal':
        return False, 'teacher_' + why, None
    if env.min_clear < RUN_CLEAR:
        return False, 'teacher_close', None
    return True, 'ok', P


def sample_episode(seed: int, level: str, max_tries: int = 200, enc=DEFAULT_ENC,
                   a_lat: float = A_LAT, cap_max: float | None = None, cap_min: float | None = None):
    """抽一局能通过三层检查的 (路, 车辆参数, 起点, 规划)。返回值附带作废原因计数。

    enc 是这一局的速度契约(观测尺度 + 动作解码区间);三层检查里老师就是按它开车的,
    所以标签存在的速度范围和学生能表达的范围一致。
    a_lat 透传给老师(见 road_teacher.A_LAT):默认值下 v_hi<=1 的契约里老师恒速。
    cap_max / cap_min 课程分档:速度上限的抽取范围(见 Plant.sample)。
    """
    rng = np.random.default_rng(seed)
    lv = next(l for l in LEVELS if l.name == level)
    rejects = collections.Counter()
    for _ in range(max_tries):
        if lv.real:
            from evaluation.road_track import real_track
            road = real_track(lv.real)[0]
        else:
            road = generate_loop(rng, lv) if lv.loop else generate(rng, lv)
        pl = Plant.sample(rng, enc, cap_max, cap_min)
        start = sample_start(rng, road, recovery=rng.random() < 0.5)
        ok, why, P = check(road, pl, start, a_lat)
        if ok:
            return road, pl, start, P, rejects
        rejects[why] += 1
    raise RuntimeError(f'{level} 抽了 {max_tries} 次都不可通: {dict(rejects)}')


def _one(args):
    seed, level = args
    t = time.time()
    *_, rejects = sample_episode(seed, level)
    return dict(rejects), time.time() - t


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--level', default='L3', choices=[l.name for l in LEVELS])
    ap.add_argument('--count', type=int, default=200)
    ap.add_argument('--workers', type=int, default=8)
    ap.add_argument('--seed-start', type=int, default=80_000_000)
    ap.add_argument('--out', default=None)
    a = ap.parse_args()
    total, secs = collections.Counter(), []
    with ProcessPoolExecutor(a.workers) as ex:
        for rej, dt in ex.map(_one, [(a.seed_start + i, a.level) for i in range(a.count)]):
            total.update(rej)
            secs.append(dt)
    tried = a.count + sum(total.values())
    report = dict(level=a.level, accepted=a.count, tried=tried,
                  reject_rate=sum(total.values()) / tried, rejects=dict(total),
                  mean_seconds=float(np.mean(secs)))
    print(json.dumps(report, ensure_ascii=False, indent=1))
    if a.out:
        with open(a.out, 'w', encoding='utf-8') as f:
            json.dump(report, f, ensure_ascii=False, indent=1)


if __name__ == '__main__':
    main()
