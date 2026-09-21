"""Offline speed-conditioned DAgger experiment; not a deployed car policy.

Run: python -m fastsim.multispeed --iterations 3 --episodes 10 --updates 500
"""
from __future__ import annotations

import argparse
from collections import deque
from dataclasses import replace
import json
from pathlib import Path

import gymnasium as gym
import numpy as np
import torch
from torch import nn

from .curriculum import STAGES
from .params import CarParams
from .planner import SpeedPlanTeacher

SPEED_SCALE = 2.5  # Same action and observation units in every episode.
SPEEDS = (0.5, 0.8, 1.2, 1.5, 2.0, 2.5)
FRAMES = 5


class SpeedConditionedEnv(gym.Wrapper):
    """Append requested speed; cap executed speed, retain requested action history.

    Delay and dynamics ranges are hypotheses for training, not measured car data.
    Targets change smoothly between endpoints when end_speed is supplied.
    """

    def __init__(self, env, speed, end_speed=None, transition_seconds=30., delay_steps=0):
        super().__init__(env)
        if not 0 < speed <= SPEED_SCALE or not 0 < (speed if end_speed is None else end_speed) <= SPEED_SCALE:
            raise ValueError('target speed must be in (0, 2.5] m/s')
        if transition_seconds <= 0 or delay_steps < 0:
            raise ValueError('invalid transition or delay')
        self.start_speed = speed
        self.end_speed = speed if end_speed is None else end_speed
        self.transition_seconds = transition_seconds
        self.delay_steps = delay_steps
        self.observation_space = gym.spaces.Box(-1., 1., (54,), dtype=np.float32)

    @property
    def target_speed(self):
        fraction = min(1., self.env.time / self.transition_seconds)
        return self.start_speed + fraction * (self.end_speed - self.start_speed)

    def observation(self, obs):
        return np.append(obs, self.target_speed / SPEED_SCALE).astype(np.float32)

    def reset(self, **kwargs):
        obs, info = self.env.reset(**kwargs)
        self.pending = deque([np.zeros(2, np.float32) for _ in range(self.delay_steps)])
        return self.observation(obs), info

    def step(self, action):
        requested = np.clip(np.asarray(action, np.float32), -1, 1)
        executed = requested.copy()
        # Forward-only training; the teacher never requests reverse.
        executed[0] = np.clip(executed[0], 0., self.target_speed / SPEED_SCALE)
        self.pending.append(executed)
        obs, reward, term, trunc, info = self.env.step(self.pending.popleft())
        self.env.last_action = requested.copy()
        obs = self.env._observation()
        info['target_speed'] = self.target_speed
        return self.observation(obs), reward, term, trunc, info


def make_env(seed, speed, *, randomized=False, end_speed=None, stage='s3', budget=180.):
    rng = np.random.default_rng(seed + 10_000_000)
    cp = CarParams(v_cmd_fwd=SPEED_SCALE)
    delay = 0
    if randomized:
        cp = replace(cp, speed_tau=float(rng.uniform(.25, .55)),
                     steer_rate=float(rng.uniform(2., 5.)),
                     accel_max=float(rng.uniform(1.5, 3.)),
                     a_lat_max=float(rng.uniform(5., 8.)))
        delay = int(rng.integers(0, 4))  # 0..150 ms at 20 Hz
    spec = replace(STAGES[stage], v_cmd_fwd=SPEED_SCALE, budget=budget)
    return SpeedConditionedEnv(spec.make_env(cp, mapping=False), speed, end_speed,
                               delay_steps=delay)


def make_net(width=256):
    return nn.Sequential(nn.Linear(54 * FRAMES, width), nn.Tanh(),
                         nn.Linear(width, width), nn.Tanh(), nn.Linear(width, 2))


def episode(net, seed, speed, *, collect=False, randomized=False, end_speed=None,
            stage='s3', budget=180., legacy_policy=None):
    env = make_env(seed, speed, randomized=randomized, end_speed=end_speed,
                   stage=stage, budget=budget)
    obs, info = env.reset(seed=seed)
    teacher = SpeedPlanTeacher()
    buf = np.zeros(54 * FRAMES, np.float32)
    xs, ys, errors = [], [], []
    rng = np.random.default_rng(seed)
    if legacy_policy is not None:
        legacy_policy.reset()
    while True:
        buf = np.roll(buf, -54)
        buf[-54:] = obs
        label = teacher(obs, env.unwrapped)
        label[0] = min(label[0], env.target_speed / SPEED_SCALE)
        if collect:
            xs.append(buf.copy())
            ys.append(label.copy())
        if legacy_policy is not None:
            e = env.unwrapped
            v, steer = legacy_policy.act(e.ranges, e.lidar.angles, e.encoder_speed,
                e.imu['angular_velocity_z'], max(0., 1. - e.time / legacy_policy.budget))
            action = np.array([v / SPEED_SCALE, steer / e.cp.max_steer])
        elif net is None:
            action = label.copy()
            if collect:
                action[1] = np.clip(action[1] + rng.normal(0, .1), -1, 1)
        else:
            with torch.no_grad():
                action = net(torch.from_numpy(buf)).numpy()
        obs, _, term, trunc, info = env.step(action)
        errors.append(info['lateral'] ** 2)
        if term or trunc:
            break
    result = {k: info[k] for k in ('termination_reason', 'cone_hits', 'time', 'lap_fraction')}
    result.update(seed=seed, speed=speed, end_speed=end_speed, randomized=randomized,
                  lateral_rmse=float(np.sqrt(np.mean(errors))))
    env.close()
    return xs, ys, result


def evaluate(net, seeds, speeds=SPEEDS, *, randomized=False, budget=180.):
    return [episode(net, seed, speed, randomized=randomized, budget=budget)[2]
            for speed in speeds for seed in seeds]


def summary(rows):
    result = {}
    for speed in sorted({r['speed'] for r in rows}):
        group = [r for r in rows if r['speed'] == speed]
        result[str(speed)] = dict(episodes=len(group),
            laps=sum(r['termination_reason'] == 'lap' for r in group),
            clean_laps=sum(r['termination_reason'] == 'lap' and r['cone_hits'] == 0 for r in group),
            total_cone_hits=sum(r['cone_hits'] for r in group),
            lateral_rmse=float(np.mean([r['lateral_rmse'] for r in group])))
    return result


def main():
    parser = argparse.ArgumentParser(__doc__)
    parser.add_argument('--iterations', type=int, default=3)
    parser.add_argument('--episodes', type=int, default=12)
    parser.add_argument('--updates', type=int, default=1000)
    parser.add_argument('--eval-seeds', type=int, default=3)
    parser.add_argument('--budget', type=float, default=180.)
    parser.add_argument('--out', type=Path, default=Path('runs/multispeed'))
    parser.add_argument('--teacher-only', action='store_true')
    parser.add_argument('--legacy-baseline', type=Path,
                        help='Evaluate an existing 53-dim npz under the same speed caps')
    args = parser.parse_args()
    if min(args.iterations, args.episodes, args.updates, args.eval_seeds, args.budget) <= 0:
        parser.error('counts and budget must be positive')
    args.out.mkdir(parents=True, exist_ok=True)
    torch.set_num_threads(2)
    torch.manual_seed(42)
    rng = np.random.default_rng(42)
    seeds = range(120000, 120000 + args.eval_seeds)
    if args.legacy_baseline:
        from .policy import Policy
        policy = Policy(args.legacy_baseline)
        if policy.obs_dim != 53:
            parser.error('legacy baseline requires a 53-dimensional policy')
        rows = [episode(None, seed, speed, budget=args.budget, legacy_policy=policy)[2]
                for speed in SPEEDS for seed in seeds]
        (args.out / 'legacy_baseline.json').write_text(json.dumps(rows, indent=2), encoding='utf-8')
        print(json.dumps(summary(rows)), flush=True)
        return
    if args.teacher_only:
        rows = evaluate(None, seeds, budget=args.budget)
        (args.out / 'teacher.json').write_text(json.dumps(rows, indent=2), encoding='utf-8')
        print(json.dumps(summary(rows)), flush=True)
        return
    net = make_net()
    opt = torch.optim.Adam(net.parameters(), lr=1e-3)
    xs, ys = [], []
    for iteration in range(args.iterations):
        for i in range(args.episodes):
            speed = SPEEDS[(iteration * args.episodes + i) % len(SPEEDS)]
            # Static and smoothly changing targets both occur during collection.
            end = float(rng.choice(SPEEDS)) if i % 2 else None
            x, y, _ = episode(None if iteration == 0 else net,
                100000 + iteration * args.episodes + i, speed,
                collect=True, randomized=True, end_speed=end,
                stage='s1' if rng.random() < 1 / 3 else 's3', budget=args.budget)
            xs.extend(x)
            ys.extend(y)
        # Bound replay memory by subsampling the current pool with a seeded RNG.
        if len(xs) > 200000:
            keep = rng.choice(len(xs), 200000, replace=False)
            xs, ys = [xs[i] for i in keep], [ys[i] for i in keep]
        xt, yt = torch.tensor(np.asarray(xs)), torch.tensor(np.asarray(ys))
        for _ in range(args.updates):
            ids = torch.randint(len(xt), (min(512, len(xt)),))
            loss = ((net(xt[ids]) - yt[ids]) ** 2).mean()
            opt.zero_grad()
            loss.backward()
            opt.step()
        # Save before evaluation so an interrupted evaluation never loses training.
        torch.save(dict(state_dict=net.state_dict(), obs_dim=54, frames=FRAMES,
                        width=256, schema='speed_conditioned_v1', speed_scale=SPEED_SCALE,
                        iteration=iteration, config=vars(args) | {'out': str(args.out)}),
                   args.out / f'iteration_{iteration:03d}.pt')
        rows = evaluate(net, seeds, budget=args.budget)
        robust = evaluate(net, seeds, randomized=True, budget=args.budget)
        report = dict(iteration=iteration, samples=len(xs), loss=float(loss.item()),
                      nominal=rows, randomized=robust)
        (args.out / f'iteration_{iteration:03d}.json').write_text(
            json.dumps(report, indent=2), encoding='utf-8')
        print(json.dumps(dict(iteration=iteration, samples=len(xs),
                              nominal=summary(rows), randomized=summary(robust))), flush=True)


if __name__ == '__main__':
    main()
