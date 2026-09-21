"""Offline low-speed steering DAgger, phase A/B with full round-boundary resume.

python -m fastsim.low_speed_train --out runs/low_speed_a
python -m fastsim.low_speed_train --out runs/low_speed_a --resume
Phase C is deliberately gated until new scene/dynamics validation is implemented.
"""
import argparse
from dataclasses import replace
import hashlib
import json
import math
import os
import shutil
from pathlib import Path
import time

import numpy as np
import torch
from torch import nn

from .curriculum import STAGES
from .low_speed_policy import DIM, FRAMES, SCHEMA, History, observe
from .params import CarParams
from .planner import SpeedPlanTeacher

# 0.4 is in here because the car itself runs there: PWM_CAP 1610 measures about
# 0.47 m/s, so the deployment point sits between 0.4 and 0.5. Without the lower
# rung the policy would have to extrapolate below its training range, and the
# target_speed observation channel is only meaningful across speeds it has seen.
SPEEDS = (.4, .5, .6, .8, 1.)

# Phase B entry gate. The threshold stays at 0.95; what changed (2026-09-21) is
# that it is now measured on a run large enough to resolve it, see phase_a_validation.
GATE_CLEAN_RATE = .95
MIN_VALIDATION_EPISODES = 200


def make_net():
    return nn.Sequential(nn.Linear(DIM * FRAMES, 256), nn.Tanh(),
                         nn.Linear(256, 256), nn.Tanh(), nn.Linear(256, 1))


def export(net, path):
    layers = [net[i] for i in (0, 2, 4)]
    np.savez(path, schema=SCHEMA,
             **{f'w{i}': layer.weight.detach().numpy() for i, layer in enumerate(layers)},
             **{f'b{i}': layer.bias.detach().numpy() for i, layer in enumerate(layers)})


def make_env(seed, speed, training=False, recovery_start=False):
    rng = np.random.default_rng(seed)
    stage = STAGES['s1' if training and rng.random() < 1 / 3 else 's3']
    env = replace(stage, v_cmd_fwd=1.5).make_env(CarParams(), mapping=False,
                                               terminate_on_cone=True)
    env.reset(seed=seed)
    # Mirror half the tracks; both left/right turns keep identical geometry.
    if seed % 2:
        for pts in (env.track.centerline, env.track.left, env.track.right, env.track.boards):
            pts[..., 1] *= -1
        env.obstacles = np.vstack([env.track.cones, env.track.board_points])
    start_index = int(rng.integers(len(env.track.centerline))) if training and recovery_start else 0
    x, y = env.track.centerline[start_index]
    _, _, heading = env.track.project((x, y))
    lateral = rng.uniform(-.12, .12) if training else 0.
    yaw_error = rng.uniform(-.08, .08) if training else 0.
    env.car.reset(x - math.sin(heading) * lateral, y + math.cos(heading) * lateral,
                  heading + yaw_error)
    # Recovery episodes start at training-only random positions, not validation failures.
    # Progress still measures a complete lap relative to the chosen start.
    if np.any(env.car.footprint_hits_circles(env.obstacles, env.tp.cone_base_radius)):
        x, y = env.track.centerline[0]
        _, _, heading = env.track.project((x, y))
        env.car.reset(x, y, heading)
    env.last_s = env.track.project((env.car.x, env.car.y))[0]
    env.budget = env.track.length / speed * 1.5 + 15.
    env._sense()
    return env


def episode(net, seed, speed, *, collect=False, teacher_probability=0., lookahead=1.2,
            recovery_start=False):
    env = make_env(seed, speed, training=collect, recovery_start=recovery_start)
    teacher = SpeedPlanTeacher(lookahead_min=lookahead, lookahead_max=lookahead)
    rng = np.random.default_rng(seed + 70000000)
    history = History()
    last = 0.
    xs, ys, lateral, speeds = [], [], [], []
    count = 0
    while True:
        obs = observe(env.ranges, env.lidar.angles, env.encoder_speed, speed,
                      env.imu['angular_velocity_z'], last)
        stacked = history.push(obs)
        label = float(teacher(None, env)[1])
        if collect:
            xs.append(stacked)
            ys.append([label])
        if net is None or rng.random() < teacher_probability:
            steer = label
            if collect:
                steer += float(rng.normal(0, .04))
        else:
            with torch.no_grad():
                steer = float(net(torch.from_numpy(stacked)).item())
        last = float(np.clip(steer, -1, 1))
        _, _, term, trunc, info = env.step([speed / env.v_cmd_fwd, last])
        count += 1
        lateral.append(info['lateral'] ** 2)
        speeds.append(info['speed'])
        if term or trunc:
            break
    row = dict(seed=seed, target_speed=speed, steps=count, recovery_start=recovery_start,
               reason=info['termination_reason'], cone_hits=info['cone_hits'],
               lap_fraction=info['lap_fraction'], time=info['time'],
               lateral_rmse=float(np.sqrt(np.mean(lateral))), mean_speed=float(np.mean(speeds)))
    env.close()
    return np.asarray(xs, np.float32), np.asarray(ys, np.float32), row


def evaluate(net, seed0, n, speeds, lookahead):
    return [episode(net, seed, speed, lookahead=lookahead)[2]
            for speed in speeds for seed in range(seed0, seed0 + n)]


def score(rows):
    return {str(speed): sum(r['reason'] == 'lap' and r['cone_hits'] == 0
                            for r in rows if r['target_speed'] == speed) /
            sum(r['target_speed'] == speed for r in rows)
            for speed in sorted({r['target_speed'] for r in rows})}


def atomic_save(state, path):
    replay_bytes = sum(x.nbytes + y.nbytes for x, y in state['replay'].values())
    if shutil.disk_usage(path.parent).free < replay_bytes + 512 * 1024 ** 2:
        raise RuntimeError('Insufficient disk space for atomic checkpoint; previous checkpoint retained')
    tmp = path.with_suffix('.tmp')
    torch.save(state, tmp)
    os.replace(tmp, path)


def load_state(path, net, opt, rng):
    # Local, self-created checkpoint includes numpy replay buffers and RNG state.
    state = torch.load(path, map_location='cpu', weights_only=False)
    if state['schema'] != SCHEMA:
        raise ValueError('Checkpoint schema mismatch')
    net.load_state_dict(state['net'])
    opt.load_state_dict(state['optimizer'])
    rng.bit_generator.state = state['numpy_rng']
    torch.set_rng_state(state['torch_rng'])
    return state


def validate_resume_config(previous, current):
    """Allow explicit budget extension/recovery curriculum, not silent architecture changes."""
    old = dict(previous)
    new = dict(current)
    old.setdefault('recovery_starts', False)
    if new['steps'] < old['steps']:
        raise ValueError('Cannot reduce the recorded training budget')
    for key in ('steps', 'recovery_starts'):
        old.pop(key)
        new.pop(key)
    if old != new:
        raise ValueError('Resume configuration changed outside allowed budget/curriculum fields')


def phase_a_validation(candidate):
    """Best adequately powered low_speed_eval report for exactly these weights, or None.

    The gate used to read eval_<round>.json, the 50-seed sweep run inside training.
    50 seeds cannot resolve a 0.95 threshold: the 95% interval is about +-0.06 wide,
    so a model whose true rate is 0.95 often reads 0.90 and is refused, and a weaker
    one occasionally reads 0.96 and is let through. Measured on candidate_060
    (2026-09-21): eval_060.json said 0.94, a 200-seed run said 0.960.

    sha256 is checked against the file on disk so a stale report from an earlier
    candidate cannot be mistaken for this one.
    """
    digest = hashlib.sha256(candidate.read_bytes()).hexdigest()
    best = None
    for path in sorted(candidate.parent.glob('validation_*.json')):
        try:
            report = json.loads(path.read_text(encoding='utf-8'))
            episodes = report['summary']['episodes']
        except (ValueError, KeyError, OSError):
            continue
        if (not report.get('complete') or report.get('sha256') != digest
                or episodes < MIN_VALIDATION_EPISODES):
            continue
        if best is None or episodes > best['summary']['episodes']:
            best = report
    return best


def main():
    ap = argparse.ArgumentParser(__doc__)
    ap.add_argument('--out', type=Path, default=Path('runs/low_speed_a'))
    ap.add_argument('--phase', choices=['A', 'B'], default='A')
    ap.add_argument('--steps', type=int, default=500000)
    ap.add_argument('--round-steps', type=int, default=50000)
    ap.add_argument('--updates', type=int, default=2500)
    ap.add_argument('--teacher-seeds', type=int, default=50)
    ap.add_argument('--eval-seeds', type=int, default=10)
    ap.add_argument('--full-eval-seeds', type=int, default=50)
    ap.add_argument('--seed', type=int, default=42)
    ap.add_argument('--lookahead', type=float, default=1.2)
    ap.add_argument('--resume', action='store_true')
    ap.add_argument('--recovery-starts', action='store_true',
                    help='Start one third of training episodes at random track positions')
    ap.add_argument('--init', type=Path, help='Phase B: a passed phase A checkpoint')
    args = ap.parse_args()
    if min(args.steps, args.round_steps, args.updates, args.teacher_seeds,
           args.eval_seeds, args.full_eval_seeds, args.lookahead) <= 0:
        ap.error('Budgets must be positive')
    args.out.mkdir(parents=True, exist_ok=True)
    checkpoint = args.out / 'latest.pt'
    if checkpoint.exists() and not args.resume:
        ap.error('Output already has a checkpoint; use --resume or another directory')
    torch.set_num_threads(2)
    torch.manual_seed(args.seed)
    rng = np.random.default_rng(args.seed)
    net = make_net()
    opt = torch.optim.Adam(net.parameters(), lr=1e-3)
    config = vars(args) | {'out': str(args.out), 'resume': False,
                           'init': str(args.init) if args.init else None}
    replay, total, index, round_id = {}, 0, 0, 0
    if args.resume:
        state = load_state(checkpoint, net, opt, rng)
        try:
            validate_resume_config(state['config'], config)
        except ValueError as error:
            ap.error(str(error))
        replay, total, index, round_id = (state[k] for k in ('replay', 'steps', 'episode_index', 'round'))
    elif args.phase == 'B':
        if args.init is None:
            ap.error('Phase B requires --init from validated phase A')
        previous = torch.load(args.init, map_location='cpu', weights_only=False)
        candidate = args.init.parent / f"candidate_{previous['round']:03d}.npz"
        report = phase_a_validation(candidate) if candidate.exists() else None
        if (previous['schema'] != SCHEMA or previous['config']['phase'] != 'A'
                or previous['steps'] < 500000 or report is None
                or report['summary']['clean_rate'] < GATE_CLEAN_RATE):
            ap.error(f'Phase A has not passed its training and validation gate '
                     f'(needs a complete low_speed_eval run of at least '
                     f'{MIN_VALIDATION_EPISODES} episodes on {candidate.name} '
                     f'reaching {GATE_CLEAN_RATE:.2f})')
        net.load_state_dict(previous['net'])
        opt.load_state_dict(previous['optimizer'])
    elif args.init is not None:
        ap.error('--init is only supported for phase B')
    speeds = (.6,) if args.phase == 'A' else SPEEDS
    print(json.dumps(dict(event='start', config=config, resumed_steps=total)), flush=True)
    gate = args.out / 'teacher.json'
    if not args.resume:
        rows = evaluate(None, 20000000, args.teacher_seeds, speeds, args.lookahead)
        gate.write_text(json.dumps(rows, indent=2), encoding='utf-8')
        print(json.dumps(dict(event='teacher_gate', clean_rates=score(rows))), flush=True)
        if min(score(rows).values()) < .95:
            raise SystemExit('Teacher gate failed; do not collect unreliable demonstrations')
    start = time.monotonic()
    initial_steps = total
    pending_report = args.out / f'eval_{round_id:03d}.json'
    if args.resume and round_id and not pending_report.exists():
        # A checkpoint is written before evaluation. Recover an interrupted
        # evaluation too, including when the final training budget was reached.
        full = round_id % 5 == 0 or total >= args.steps
        rows = evaluate(net, 30000000, args.full_eval_seeds if full else args.eval_seeds,
                        speeds, args.lookahead)
        export(net, args.out / f'candidate_{round_id:03d}.npz')
        pending_report.write_text(json.dumps(dict(round=round_id, steps=total,
            clean_rates=score(rows), full_evaluation=full, episodes=rows,
            recovered_evaluation=True), indent=2), encoding='utf-8')
    while total < args.steps:
        target = min(total + args.round_steps, args.steps)
        new_x, new_y = {s: [] for s in speeds}, {s: [] for s in speeds}
        # Keep some teacher coverage while increasing student-state collection.
        beta = 1. if round_id == 0 else max(.05, .5 * .75 ** (round_id - 1))
        while total < target:
            speed = speeds[index % len(speeds)]
            x, y, row = episode(None if round_id == 0 else net,
                (10000000 if args.phase == 'A' else 11000000) + index,
                speed, collect=True, teacher_probability=beta,
                lookahead=args.lookahead,
                recovery_start=args.recovery_starts and index % 3 == 0)
            index += 1
            total += row['steps']
            new_x[speed].append(x)
            new_y[speed].append(y)
            print(json.dumps(dict(event='episode', total_steps=total, **row)), flush=True)
        for speed in speeds:
            parts_x, parts_y = new_x[speed], new_y[speed]
            if speed in replay:
                parts_x.insert(0, replay[speed][0])
                parts_y.insert(0, replay[speed][1])
            x, y = np.concatenate(parts_x), np.concatenate(parts_y)
            capacity = 1000000 // len(speeds)
            if len(x) > capacity:
                keep = rng.choice(len(x), capacity, replace=False)
                x, y = x[keep], y[keep]
            replay[speed] = (x, y)
        for _ in range(args.updates):
            speed = speeds[int(rng.integers(len(speeds)))]
            x, y = replay[speed]
            ids = rng.integers(len(x), size=512)
            loss = ((net(torch.from_numpy(x[ids])) - torch.from_numpy(y[ids])) ** 2).mean()
            opt.zero_grad()
            loss.backward()
            opt.step()
        round_id += 1
        atomic_save(dict(schema=SCHEMA, config=config, net=net.state_dict(),
            optimizer=opt.state_dict(), numpy_rng=rng.bit_generator.state,
            torch_rng=torch.get_rng_state(), replay=replay, steps=total,
            episode_index=index, round=round_id), checkpoint)
        export(net, args.out / f'candidate_{round_id:03d}.npz')
        full = round_id % 5 == 0 or total >= args.steps
        rows = evaluate(net, 30000000, args.full_eval_seeds if full else args.eval_seeds,
                        speeds, args.lookahead)
        report = dict(round=round_id, steps=total, loss=float(loss.item()),
                      clean_rates=score(rows), full_evaluation=full, episodes=rows,
                      elapsed_seconds=time.monotonic() - start,
                      new_steps=total - initial_steps)
        (args.out / f'eval_{round_id:03d}.json').write_text(json.dumps(report, indent=2), encoding='utf-8')
        print(json.dumps({k: v for k, v in report.items() if k != 'episodes'}), flush=True)
    print('Phase finished; inspect full validation before advancing. No car deployment.', flush=True)


if __name__ == '__main__':
    main()
