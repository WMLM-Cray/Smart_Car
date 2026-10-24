#!/usr/bin/env python3
"""Owned paired Gazebo smoke runner; no ROS imports at module load.

Starts disabled policy/supervisor before bounded native graph polling. Hosts the unchanged
production Eval on the same executor; its first racecar callback supplies the real sim t0.
Publishes enable at t0+5 and stops at t0+140 (135 seconds driving). Default policy/ScanGuard
system clocks are retained. Each Popen root and descendant uses OwnedProcesses PID/starttime.
Installed launch is snapshotted and differs only in the generated controller output path.
Raw scan nonfinite values are reversibly labelled; /clock/world/odom/command events are kept.
The current-gap speed limiter is an experimental heuristic, not a safety proof. Scaling (v,w)
preserves command curvature, not actual trajectory. Odom is not world truth. Stop on command
silence belongs to sim_wiring's 0.5-sim-second watchdog; no real-car conclusion follows.
Whether simulation freezes during a host stall is not assumed.
"""
from __future__ import annotations

import csv as _csv
import importlib.util
import json
import math
import os
import signal
import socket
import statistics
import subprocess
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import gazebo_track_progress as track_progress                      # noqa: E402

# 运行解释器（装了 ROS humble / gazebo 依赖的那个 python）与队友 3Dsim 检出位置，都只从环境读，
# **不给机器路径默认值**：没设置就在 config_from_args 里报 ConfigError。
# 理由：带默认值的机器路径会让"在我这台机器上能跑"被当成仓库可复现。
VENV_ROOT = os.environ.get('VENV_ROOT')
VENV_PYTHON = os.environ.get('VENV_PYTHON') or (
    os.path.join(VENV_ROOT, 'bin', 'python') if VENV_ROOT else None)
# 反方向也补上：只给 VENV_PYTHON 时按它推 VENV_ROOT（<root>/bin/python），这样"一个解释器"
# 就够 —— child_env() 的"当前解释器就是这个 venv 的 python"校验需要 VENV_ROOT 参与比较。
# 跑批入口 evaluation/run_gazebo_supervised_smoke.sh 就是这么把 PYTHON 传下来的。
if VENV_PYTHON and not VENV_ROOT:
    VENV_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(VENV_PYTHON)))
SUP_REPO = Path(__file__).resolve().parents[1]
# 队友仓（拥有 3Dsim 打分工具、dev suite、遥操作节点的检出）必须显式给出：--team-repo 或 TEAM_REPO。
# 不再按"同级目录里叫 Auto_Car-main"去猜。

SCHEMA_VERSION = 2
R36_SHA256 = '74fb1412915e410633e2d66fb054ac479646807fd41607b73e08a52b582e9964'
PREREGISTRATION = 'supervised_smoke_v4'
# The cap ladder is a separate preregistration: it changes only --cap, keeps the production
# stack unmodified and runs the baseline arm alone (no supervisor).
PREREGISTRATION_CAP_LADDER = 'cap_ladder_v4'
FROZEN_CAP = 3.0
# Preregistered ladder rungs, ascending. 1.5 was the last cap with 0/12 stuck episodes on the
# A-D validation panel, but the first dev-suite rung (narrow_03 at 1.5) still showed the cap-3.0
# stall mode with 0-1 laps in 140 s, so the ladder must be able to bracket the frontier from
# below as well as above. v1-v3 stopped at 1.8, which left the whole 1.8 -> 3.0 stretch
# unexplored even though that is where the measured cliff lives (straight_01: 5 laps at 1.5,
# 0-1 laps at 3.0). v4 adds 1.9/2.0/2.2/2.4/2.6/2.8 to bisect it. Existing v1-v3 run
# directories keep the preregistration id they recorded and are never re-run or relabelled.
# --cap accepts the frozen 3.0 or one of these; other floats stay rejected.
CAP_LADDER_RUNGS = (1.2, 1.35, 1.5, 1.6, 1.65, 1.7, 1.75, 1.8,
                    1.9, 2.0, 2.2, 2.4, 2.6, 2.8)
DEV_SUITE_TRACKS = (
    'arc_01', 'arc_02', 'arc_03', 'arc_04',
    'narrow_01', 'narrow_02', 'narrow_03', 'narrow_04',
    'right_01', 'right_02', 'right_03', 'right_04',
    'sharp_01', 'sharp_02', 'sharp_03', 'sharp_04',
    'straight_01', 'straight_02', 'straight_03', 'straight_04',
    's_bend_01', 's_bend_02', 's_bend_03', 's_bend_04',
)
# The deployment panel: the four official A-D tracks. Until 2026-10-07 they were
# validation-only (see the rejection below). The deployment objective ("faster than r36 AND
# 100% clean on A-D") is judged on this panel, so it is now an authorised track set in its own
# right, with its own preregistration id. The assets live in the supervisor repo (not the team
# repo): evaluation/panel/track_{A..D} are byte-identical mirrors of
# Auto_Car-main/runs/gz_local_20260926/track_{A..D} (layout.yaml / centerline.npy / half.npy /
# track_sim.world) plus a generated cones.csv, because the team-repo copies have no cones.csv
# and REQUIRED_TRACK_FILES demands one. Nothing under the team repo is modified.
# 2026-10-08：面板赛道从 runs/gz_official/ 挪到 evaluation/panel/，因为 runs/ 不入库，
# 放在那里的资产干净检出根本跑不起来。
AD_PANEL_TRACKS = ('track_A', 'track_B', 'track_C', 'track_D')
AD_PANEL_DIRNAME = 'panel'
PREREGISTRATION_AD_PANEL = 'ad_panel_deploy_20261007'
OUT_PREFIX = 'gazebo_supervised_smoke_20261005'
ARMS = ('baseline', 'candidate')
WORKER_MIN = 20
DEFAULT_WORKER = 21
DOMAIN_BASE, PORT_BASE = 43, 11345
PROBE_DOMAIN, PROBE_PORT = 63, 11365          # gazebo_braking_probe.py:16 (must stay free)
THREAD_ENV = {'OMP_NUM_THREADS': '1', 'OPENBLAS_NUM_THREADS': '1', 'MKL_NUM_THREADS': '1'}

WARMUP_SIM_S = 5.0
SECONDS_SIM_S = 140.0
MAX_WALL_FACTOR = 3.0
READY_DEADLINE_WALL_S = 180.0
ANCHOR_DEADLINE_WALL_S = 30.0
GRAPH_POLL_INTERVAL_S = 0.5
CLOCK_STALE_WALL_S = 10.0
PAIRING_TOLERANCE_S = 0.100001
# The anchor comparison spans two different nodes on the same /model_states topic: the observer's
# first racecar sighting and the instrument's own t0. Each node pairs a world callback with its own
# last-received /clock value, and spin_once runs one callback at a time, so the two can disagree by
# up to two 0.1 s /clock steps (one step of clock lag per node). Observed on the dev suite: 0.1, 0.2.
# Gating that at one step made certification a callback-order coin flip (rung-1 cap 1.5: 0.2/0.2/0.1),
# so the cross-node bound is two steps. This is a labelling bound only; --seconds acceptance below
# keeps zero tolerance.
PAIRING_ANCHOR_TOLERANCE_S = 0.200001
PROGRESS_INTERVAL_WALL_S = 2.0
# How long the loop keeps spinning after its own /clock says the window is closed, waiting for
# the scoring instrument to observe the same closure. The runner node and the instrument are
# two separate /clock subscriptions and spin_once runs one callback at a time, so the runner's
# sim_s can lead the instrument's sim_t by the 0.1 s /clock step; closing on the runner alone
# truncated the measured window to 139.9 s in PAIR02. The grace delays the stop; it does not
# relax the window check (see the strict elapsed_s test in run_arm). It equals the accepted
# clock-receipt silence budget so that any stall tolerated elsewhere is tolerated here too.
SCORER_CLOSE_GRACE_WALL_S = 10.0

CMD_TOPIC = '/cmd_vel'
RAW_TOPIC = '/cmd_vel_raw'
CLOCK_TOPIC = '/clock'
WORLD_TOPIC = '/model_states'
ODOM_TOPIC = '/ackermann_steering_controller/odometry'
SCAN_TOPIC = '/scan'
REF_TOPIC = '/ackermann_steering_controller/reference_unstamped'
ENABLE_TOPIC = '/policy_enable'
SUPERVISOR_NODE = 'gazebo_speed_supervisor'
POLICY_NODE = 'road_policy_node'
SCORER_NODE = 'track_eval'
GUARD_LOAD_PREFIX = 'SUPERVISOR_GUARD_LOAD'
REQUIRED_TRACK_FILES = ('layout.yaml', 'cones.csv', 'track_sim.world')
WORLD_PLUGIN_MARKER = 'libgazebo_ros_state.so'


class ConfigError(Exception):
    """Raised for a caller mistake that must not start any process."""


class ConflictError(Exception):
    """Raised when another domain/port/world/process would be touched. Nothing is cleaned."""


class RunnerInterrupted(Exception):
    """Raised from a signal handler so the finally block cleans up and keeps evidence."""


def write_json(path, value):
    """Atomic-ish write: a half-written evidence file is worse than no file."""
    path = Path(path)
    tmp = path.with_suffix(path.suffix + '.tmp')
    tmp.write_text(json.dumps(serializable(value), indent=2, allow_nan=False, default=str) + '\n')
    tmp.replace(path)


def append_jsonl(path, value):
    with open(path, 'a', encoding='utf-8') as handle:
        handle.write(json.dumps(serializable(value), allow_nan=False, default=str) + '\n')


def sha256_file(path):
    import hashlib
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def serializable(value):
    if isinstance(value, dict):
        return {str(k): serializable(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [serializable(v) for v in value]
    if isinstance(value, Path):
        return str(value)
    if isinstance(value, float) and not math.isfinite(value):
        return 'NaN' if math.isnan(value) else '+Infinity' if value > 0 else '-Infinity'
    return value


def _load_module(name, path):
    spec = importlib.util.spec_from_file_location(name, Path(path))
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def load_supervisor_module(cfg):
    """Read-only import of our own supervisor module (pure helpers + production guard loader)."""
    return _load_module('gazebo_speed_supervisor',
                        Path(cfg['repo']) / 'evaluation' / 'gazebo_speed_supervisor.py')


def load_scorer(cfg, out):
    """Read-only import of the PRODUCTION scoring instrument, built exactly as track_eval.main does.

    Same class, same CarParams, same cones CSV, same start pose, same max_wall budget. No copy
    of the judging logic lives in this file, so the score cannot drift from the teammate's.
    """
    import yaml
    team, track_dir = Path(cfg['team_repo']), Path(out) / 'track'
    layout = yaml.safe_load((track_dir / 'layout.yaml').read_text(encoding='utf-8'))
    rows = list(_csv.DictReader((track_dir / 'cones.csv').open(encoding='utf-8-sig')))
    cones = [(row['id'], float(row['x']), float(row['y'])) for row in rows]
    sp = layout['start_pose']
    start = (sp['xy'][0], sp['xy'][1], math.radians(sp['yaw_deg']))
    for root in (team, team / '3Dsim', Path.cwd()):
        if (root / 'fastsim').is_dir():
            if str(root) not in sys.path:
                sys.path.insert(0, str(root))
            break
    else:
        raise ConfigError('fastsim/ not found under %s (CarParams is required)' % team)
    from fastsim.params import CarParams
    params = CarParams()
    module = _load_module('track_eval_instrument', team / '3Dsim' / 'tools' / 'track_eval.py')
    node = module.Eval(cones, start, params.length, params.width, cfg['seconds'],
                       max_wall=cfg['seconds'] * MAX_WALL_FACTOR)
    meta = dict(instrument=str(team / '3Dsim' / 'tools' / 'track_eval.py'),
                cones_n=len(cones), start=list(start), car_length=params.length,
                car_width=params.width, seconds=cfg['seconds'],
                max_wall=cfg['seconds'] * MAX_WALL_FACTOR,
                scorer_created_wall=node.created_wall, node_name=node.get_name())
    return node, meta


# --------------------------------------------------------------------------------------
# configuration
# --------------------------------------------------------------------------------------

VALUE_FLAGS = ('--arm', '--out', '--worker', '--track', '--npz', '--cap', '--guard',
               '--seconds', '--warmup-sim-s', '--team-repo', '--policy-entry')


def config_from_args(argv, repo=None, env=None):
    repo = Path(repo or SUP_REPO).resolve()
    env = dict(os.environ if env is None else env)
    opts, preflight_only = {}, False
    index = 0
    while index < len(argv):
        arg = argv[index]
        if arg == '--preflight-only':
            preflight_only = True
            index += 1
            continue
        if arg in VALUE_FLAGS:
            if index + 1 >= len(argv):
                raise ConfigError('missing value for %s' % arg)
            opts[arg] = argv[index + 1]
            index += 2
            continue
        raise ConfigError('unknown argument: %s' % arg)

    arm = opts.get('--arm', 'pair')
    if arm not in ARMS + ('pair',):
        raise ConfigError('--arm must be one of %s' % (ARMS + ('pair',)))
    team_arg = opts.get('--team-repo') or env.get('TEAM_REPO')
    if not team_arg:
        raise ConfigError('--team-repo (or TEAM_REPO) is required: this repo does not guess the '
                          'teammate 3Dsim checkout location')
    if not VENV_PYTHON:
        raise ConfigError('VENV_PYTHON (or VENV_ROOT, which implies VENV_ROOT/bin/python) is '
                          'required: the interpreter that has ROS humble / gazebo; this repo '
                          'carries no machine default. The smoke wrapper derives both from its '
                          'PYTHON variable.')
    team = Path(team_arg)
    worker_raw = opts.get('--worker') or env.get('WORKER') or str(DEFAULT_WORKER)
    try:
        worker = int(worker_raw)
    except ValueError:
        raise ConfigError('--worker must be an integer, got %r' % worker_raw)
    if worker < WORKER_MIN:
        raise ConfigError('WORKER must be >= %d (no shared domain/port fallback); got %d'
                          % (WORKER_MIN, worker))
    defaults = _supervisor_defaults(repo)
    # 策略入口的 CLI 契约是 `<model> --sim --speed <cap> --guard <guard>`（老 teleop 节点的 CLI，
    # arm_commands() 就按这个拼 argv）。本仓 `vehicle/road_policy_node.py` 是仿真生产节点，但 CLI
    # 不同（`--speed-cap/--sensor-timeout`），所以**不能**拿它当静默默认值；这里也不猜同级目录：
    # 显式 `--policy-entry` 优先；否则只认队友检出里还留着的老入口；再没有就报错说清要什么
    # （外部路径不给默认值，缺件要报错说明）。
    legacy_entry = team / 'teleop' / 'road_policy_node.py'
    if '--policy-entry' in opts:
        policy_entry = Path(opts['--policy-entry']).resolve()
    elif legacy_entry.is_file():
        policy_entry = legacy_entry.resolve()
    else:
        raise ConfigError(
            '--policy-entry is required: no policy entry to run. The fallback would be %s '
            '(legacy teleop node, CLI `<model> --sim --speed --guard`), which this repo no '
            'longer carries (main moved teleop/ -> vehicle/, and vehicle/road_policy_node.py '
            'takes --speed-cap/--sensor-timeout instead). Pass --policy-entry <file> '
            '(e.g. this repo\'s arms/corridor_arm.py, or your own node with that CLI).'
            % legacy_entry)
    # 臂身份只由环境变量 CORRIDOR_ARM 决定（arms/corridor_arm.py:60 不设就是 'D'）。缺它就是
    # 静默换臂：跑出来的数字对不上 arms/VERSIONS.md 与报告里的口径，所以这里拒绝而不是放行
    # （仓库里唯一会设它的入口是 evaluation/corridor_arm_run.sh，它把第一个参数 export 出去）。
    if policy_entry.name == 'corridor_arm.py' and not (env.get('CORRIDOR_ARM') or '').strip():
        raise ConfigError(
            'CORRIDOR_ARM is required with --policy-entry arms/corridor_arm.py: with it unset that '
            "file silently falls back to arm 'D', which is not the shipped calibre (the four-track "
            'panel runs use CORRIDOR_ARM=Y). Use evaluation/corridor_arm_run.sh <arm> <cap> <track> '
            '<reps> (it exports CORRIDOR_ARM from its first argument), or export CORRIDOR_ARM=<arm>.')
    cfg = dict(
        arm=arm, repo=repo, team_repo=team, worker=worker,
        domain=DOMAIN_BASE + worker, port=PORT_BASE + worker,
        probe_domain_overlap=(DOMAIN_BASE + worker == PROBE_DOMAIN),
        track=Path(opts.get('--track') or team / 'runs' / 'gen_suite_20260926' / 'narrow_03'),
        npz=Path(opts.get('--npz') or repo / 'models' / 'spica_v3geo_nola' / 'candidate_036.npz').resolve(),
        cap=float(opts.get('--cap', 3.0)), guard=float(opts.get('--guard', 0.25)),
        seconds=float(opts.get('--seconds', SECONDS_SIM_S)),
        warmup=float(opts.get('--warmup-sim-s', WARMUP_SIM_S)),
        L=float(defaults['L']), u=float(defaults['u']),
        reserve_m=float(defaults['reserve_m']), wheelbase=float(defaults['wheelbase']),
        preflight_only=preflight_only, thread_env=dict(THREAD_ENV),
        # 入口按上面的规则解析：显式 --policy-entry 优先，其次是队友检出里还留着的老 teleop 入口，
        # 两者都没有时已经在上面报错（不再有"猜一个仓外路径"的静默默认值）。
        policy_entry=policy_entry,
    )
    out = Path(opts['--out']) if '--out' in opts else repo / 'runs' / OUT_PREFIX
    if arm != 'pair' and '--out' not in opts:
        out = out / arm
    cfg['out'] = validate_out(out, repo)
    validate_config(cfg)
    return cfg


def _supervisor_defaults(repo):
    """L/u/reserve/wheelbase straight from the supervisor module, so they cannot drift."""
    try:
        module = _load_module('gazebo_speed_supervisor',
                              Path(repo) / 'evaluation' / 'gazebo_speed_supervisor.py')
        return dict(module.DEFAULT_PARAMS)
    except BaseException as exc:
        raise ConfigError('cannot load frozen supervisor defaults: %s' % exc)


def validate_out(out, repo):
    """The output directory must live under <repo>/runs/<name starting with OUT_PREFIX>."""
    repo, out = Path(repo).resolve(), Path(out).resolve()
    runs = repo / 'runs'
    try:
        rel = out.relative_to(runs)
    except ValueError:
        raise ConfigError('output must live under %s, got %s' % (runs, out))
    if not rel.parts or not rel.parts[0].startswith(OUT_PREFIX):
        raise ConfigError('output name must start with %r, got %r'
                          % (OUT_PREFIX, rel.parts[0] if rel.parts else out))
    if out.exists():
        raise ConfigError('output directory already exists (never reused): %s' % out)
    return out


def validate_config(cfg):
    # wheelbase is pinned to the repo's measured CarParams (common/params.py:26 = 0.35), because
    # the supervisor's steer estimate is the inverse of common/control.py to_cmd_vel; it used to
    # be the sim world's 0.33。
    if (cfg['L'], cfg['u'], cfg['reserve_m'], cfg['wheelbase']) != (.053618, .103084, 0., .35):
        raise ConfigError('supervisor defaults changed from preregistered heuristic')
    if cfg['probe_domain_overlap']:
        raise ConfigError('domain63/port11365 reserved for braking probe; use WORKER21')
    if (cfg['guard'], cfg['seconds'], cfg['warmup']) != (0.25, 140.0, 5.0):
        raise ConfigError('frozen smoke requires guard.25 seconds140 warmup5')
    if cfg['arm'] == 'baseline':
        if cfg['cap'] != FROZEN_CAP and cfg['cap'] not in CAP_LADDER_RUNGS:
            raise ConfigError('--cap must be the frozen %g or a preregistered ladder rung %s, got %r'
                              % (FROZEN_CAP, list(CAP_LADDER_RUNGS), cfg['cap']))
    elif cfg['cap'] != FROZEN_CAP:
        raise ConfigError('only the baseline arm may take a cap-ladder rung (the ladder is the '
                          'unmodified production stack); %s requires cap %g, got %r'
                          % (cfg['arm'], FROZEN_CAP, cfg['cap']))
    # 权重按 sha256 授权，不按路径：runs/ 不入库，硬写 <repo>/runs/... 会让干净检出直接跑不起来
    # canonical 副本在 models/spica_v3geo_nola/candidate_036.npz（main 里已跟踪）。
    if not 0.0 < cfg['cap'] <= 5.0:
        raise ConfigError('--cap must be in (0, 5.0], got %r' % cfg['cap'])
    if not cfg['guard'] > 0.0:
        raise ConfigError('--guard must be > 0, got %r' % cfg['guard'])
    if not cfg['seconds'] > cfg['warmup'] > 0.0:
        raise ConfigError('need 0 < warmup < seconds, got warmup=%r seconds=%r'
                          % (cfg['warmup'], cfg['seconds']))
    if not Path(cfg['npz']).is_file():
        raise ConfigError('policy npz not found: %s' % cfg['npz'])
    if sha256_file(cfg['npz']) != R36_SHA256:
        raise ConfigError('frozen r36 SHA256 mismatch: %s (expected %s)'
                          % (cfg['npz'], R36_SHA256))
    track = Path(cfg['track']).resolve()
    suite = (Path(cfg['team_repo']) / 'runs' / 'gen_suite_20260926').resolve()
    # The deployment panel lives in the supervisor repo, because the team-repo A-D copies carry
    # no cones.csv and the team repo is read-only for this workstream.
    ad_suite = (SUP_REPO / 'evaluation' / AD_PANEL_DIRNAME).resolve()
    on_dev_suite = track.parent == suite and track.name in DEV_SUITE_TRACKS
    on_ad_panel = track.parent == ad_suite and track.name in AD_PANEL_TRACKS
    if not (on_dev_suite or on_ad_panel):
        raise ConfigError('only the %d preregistered dev-suite tracks directly under %s, or the '
                          '%d deployment-panel tracks directly under %s, are authorized (got %s)'
                          % (len(DEV_SUITE_TRACKS), suite, len(AD_PANEL_TRACKS), ad_suite, track))
    if not track.is_dir():
        raise ConfigError('track directory not found: %s' % track)
    missing = [name for name in REQUIRED_TRACK_FILES if not (track / name).is_file()]
    if missing:
        raise ConfigError('track %s is missing %s' % (track, ', '.join(missing)))
    world = (track / 'track_sim.world').read_text(encoding='utf-8', errors='replace')
    if WORLD_PLUGIN_MARKER not in world:
        raise ConfigError('world %s has no %s (world truth would be missing)'
                          % (track / 'track_sim.world', WORLD_PLUGIN_MARKER))
    return cfg


def preregistration_id(cfg):
    """Which preregistration this run belongs to: the deployment (A-D) panel, the frozen smoke,
    or a cap-ladder rung."""
    track = Path(cfg['track']).resolve()
    ad_suite = (SUP_REPO / 'evaluation' / AD_PANEL_DIRNAME).resolve()
    if track.parent == ad_suite and track.name in AD_PANEL_TRACKS:
        return PREREGISTRATION_AD_PANEL
    return PREREGISTRATION_CAP_LADDER if cfg['cap'] != FROZEN_CAP else PREREGISTRATION


def child_env(cfg, base=None):
    env = dict(os.environ if base is None else base)
    env.update(cfg['thread_env'])
    env['PYTHONDONTWRITEBYTECODE'] = '1'
    env['SCAN_GUARD_DIR'] = str(Path(cfg['repo']) / 'vehicle')
    env['ROS_LOG_DIR'] = str(Path(cfg['out']) / 'ros_logs')
    env['ROS_DOMAIN_ID'] = str(cfg['domain'])
    env['GAZEBO_MASTER_URI'] = 'http://127.0.0.1:%d' % cfg['port']
    env['GAZEBO_MODEL_DATABASE_URI'] = 'http://127.0.0.1:1/'
    return env


def check_venv(env=None):
    env = dict(os.environ if env is None else env)
    real = os.path.realpath(env.get('VIRTUAL_ENV') or VENV_ROOT)
    interpreter = os.path.realpath(sys.executable)
    ok = real == os.path.realpath(VENV_ROOT) and interpreter == os.path.realpath(VENV_PYTHON)
    return dict(name='specified_venv', ok=bool(ok),
                detail='VIRTUAL_ENV=%s python=%s expected=%s' % (real, interpreter, VENV_PYTHON))


# --------------------------------------------------------------------------------------
# command construction (pure)
# --------------------------------------------------------------------------------------

def _fmt(value):
    return '%g' % float(value)


def arm_commands(arm, cfg, out, world_path):
    """Exact argv for one arm. Candidate adds the supervisor and the raw remap; nothing else."""
    out, team, sup = Path(out), Path(cfg['team_repo']), Path(cfg['repo'])
    world_path = Path(world_path)
    sim = [VENV_PYTHON, '/opt/ros/humble/bin/ros2', 'launch',
           str(out / 'owned_sim.launch.py'), 'use_gui:=false', 'world:=' + str(world_path),
           'spawn_x:=0', 'spawn_y:=0', 'spawn_yaw:=0']
    policy = [VENV_PYTHON, str(cfg['policy_entry']), str(cfg['npz']),
              '--sim', '--speed', _fmt(cfg['cap']), '--guard', _fmt(cfg['guard'])]
    commands = {
        'sim': dict(argv=sim, cwd=str(out), log=str(out / 'sim.log')),
        'policy': dict(argv=list(policy), cwd=str(out), log=str(out / 'policy.log')),
        'hold_enable': dict(argv=None, in_process=True,
                             detail='runner publishes Bool heartbeat on system clock after Eval.t0+5'),
        'scorer': dict(argv=None, in_process=True,
                       detail='read-only import of %s (Eval, same CarParams/cones/max_wall)'
                              % (team / '3Dsim' / 'tools' / 'track_eval.py'),
                       out=str(out / 'track_eval.json')),
        'supervisor': None,
    }
    if arm == 'candidate':
        commands['policy']['argv'] = list(policy) + ['--ros-args', '-r',
                                                     '%s:=%s' % (CMD_TOPIC, RAW_TOPIC)]
        commands['supervisor'] = dict(
            argv=[VENV_PYTHON, str(sup / 'evaluation' / 'gazebo_speed_supervisor.py'),
                  '--ros-args',
                  '-p', 'raw_topic:=' + RAW_TOPIC, '-p', 'out_topic:=' + CMD_TOPIC,
                  '-p', 'scan_topic:=' + SCAN_TOPIC, '-p', 'odom_topic:=' + ODOM_TOPIC,
                  '-p', 'L:=%.6f' % cfg['L'], '-p', 'u:=%.6f' % cfg['u'],
                  '-p', 'reserve_m:=%.6f' % cfg['reserve_m'],                   '-p', 'wheelbase:=%.6f' % cfg['wheelbase'],
                   '-p', 'scan_stale_s:=0.5',
                  '-p', 'debug_csv:=' + str(out / 'supervisor.csv')],
            cwd=str(out), log=str(out / 'supervisor.log'))
    return commands


def graph_expectation(arm):
    """Publisher identities this arm must show before the car is allowed to move."""
    if arm == 'candidate':
        return {CMD_TOPIC: dict(count=1, nodes=(SUPERVISOR_NODE,)),
                RAW_TOPIC: dict(count=1, nodes=(POLICY_NODE,)),
                REF_TOPIC: dict(count=1, nodes=('sim_wiring',)),
                ENABLE_TOPIC: dict(count=1, nodes=('supervised_runner',))}
    return {CMD_TOPIC: dict(count=1, nodes=(POLICY_NODE,)),
            RAW_TOPIC: dict(count=0, nodes=()),
            REF_TOPIC: dict(count=1, nodes=('sim_wiring',)),
            ENABLE_TOPIC: dict(count=1, nodes=('supervised_runner',))}


def evaluate_graph_poll(poll, expectation):
    """Judge ONE poll. A failed query, a missing topic or an unknown identity never passes."""
    if poll.get('error'):
        return dict(ok=False, reasons=['graph_query_failed: %s' % poll['error']])
    topics = poll.get('topics') or {}
    reasons = []
    for topic, spec in expectation.items():
        if topic not in topics:
            reasons.append('missing_topic_info: %s' % topic)
            continue
        publishers = list(topics[topic] or [])
        if len(publishers) != spec['count']:
            reasons.append('publisher_count_mismatch: %s expected %d got %d'
                           % (topic, spec['count'], len(publishers)))
        for entry in publishers:
            if entry.get('node_name') not in spec['nodes'] or entry.get('node_namespace') != '/':
                reasons.append('unexpected_publisher_identity: %s node=%r namespace=%r'
                               % (topic, entry.get('node_name'), entry.get('node_namespace')))
    wiring = [n for n, namespace in (poll.get('nodes') or [])
              if n == 'sim_wiring' and namespace == '/']
    if len(wiring) != 1:
        reasons.append('sim_wiring_count=%d' % len(wiring))
    subscribers = poll.get('subscribers', {})
    for topic, identity in ((CMD_TOPIC, 'sim_wiring'), (ENABLE_TOPIC, POLICY_NODE)):
        matches = [row for row in subscribers.get(topic, []) if row.get('node_name') == identity and row.get('node_namespace') == '/']
        if len(matches) != 1:
            reasons.append('subscriber_identity_mismatch:%s:%s' % (topic, identity))
    if expectation[RAW_TOPIC]['count']:
        matches = [row for row in subscribers.get(RAW_TOPIC, []) if row.get('node_name') == SUPERVISOR_NODE and row.get('node_namespace') == '/']
        if len(matches) != 1:
            reasons.append('raw_supervisor_subscription_missing_or_duplicate')
    return dict(ok=not reasons, reasons=reasons)


def readiness_report(state, poll, expectation):
    """Pure readiness gate. state carries wall/sim timestamps and topic receipt counts."""
    graph = evaluate_graph_poll(poll, expectation)
    reasons = list(graph['reasons'])
    sim_s, now_wall = state.get('sim_s'), state.get('now_wall')
    if sim_s is None:
        reasons.append('no_clock')
    elif now_wall is not None and state.get('clock_wall') is not None \
            and now_wall - state['clock_wall'] > CLOCK_STALE_WALL_S:
        reasons.append('clock_stale')
    world = state.get('world')
    if not world:
        reasons.append('no_model_states')
    else:
        if world.get('racecar_count') != 1:
            reasons.append('racecar_count=%s' % world.get('racecar_count'))
        if world.get('marker_box'):
            reasons.append('marker_box_present')
    if not state.get('odom_n'):
        reasons.append('no_odom')
    if not state.get('scan_n'):
        reasons.append('no_scan')
    return dict(ok=not reasons, reasons=reasons, graph=graph)


# --------------------------------------------------------------------------------------
# startup state machine (pure, drives exactly one enable)
# --------------------------------------------------------------------------------------

class StartupProtocol:
    """No command before readiness; scorer created at readiness; enable at scorer.t0 + WARMUP."""

    def __init__(self, arm, warmup_sim_s=WARMUP_SIM_S, seconds_sim_s=SECONDS_SIM_S,
                 ready_deadline_wall_s=READY_DEADLINE_WALL_S,
                 anchor_deadline_wall_s=ANCHOR_DEADLINE_WALL_S,
                 tolerance_s=PAIRING_TOLERANCE_S):
        self.arm = arm
        self.warmup_sim_s = float(warmup_sim_s)
        self.seconds_sim_s = float(seconds_sim_s)
        self.ready_deadline_wall_s = float(ready_deadline_wall_s)
        self.anchor_deadline_wall_s = float(anchor_deadline_wall_s)
        self.tolerance_s = float(tolerance_s)
        self.phase = 'waiting_ready'
        self.started_wall = None
        self.anchor_started_wall = None
        self.enable_sim_s = None
        self.enable_wall = None
        self.anchor_sim = None

    def next_action(self, state):
        """state: sim_s, now_wall, ready, ready_reasons, graph_ok, graph_reasons, anchor_sim,
        has_scan. Returns one action string; every abort is prefixed with 'abort:'."""
        sim_s = state.get('sim_s')
        now_wall = state.get('now_wall')
        if self.started_wall is None and now_wall is not None:
            self.started_wall = now_wall
        if self.phase == 'waiting_ready':
            if state.get('ready'):
                self.phase = 'waiting_anchor'
                self.anchor_started_wall = now_wall
                return 'start_scorer'
            if now_wall - self.started_wall > self.ready_deadline_wall_s:
                self.phase = 'failed'
                return 'abort:ready_timeout:%s' % ';'.join(state.get('ready_reasons') or [])
            return 'wait_ready'
        if self.phase == 'waiting_anchor':
            anchor = state.get('anchor_sim')
            if anchor is not None:
                self.anchor_sim = float(anchor)
                self.phase = 'warmup'
            elif now_wall - self.anchor_started_wall > self.anchor_deadline_wall_s:
                self.phase = 'failed'
                return 'abort:anchor_timeout'
            else:
                return 'wait_anchor'
        if self.phase == 'warmup':
            if sim_s is None:
                self.phase = 'failed'
                return 'abort:clock_lost_during_warmup'
            if sim_s < self.anchor_sim + self.warmup_sim_s:
                return 'wait_warmup'
            if not state.get('graph_ok'):
                self.phase = 'failed'
                return 'abort:graph_not_ready_at_enable:%s' % ';'.join(state.get('graph_reasons') or [])
            if not state.get('has_scan'):
                self.phase = 'failed'
                return 'abort:no_scan_at_enable'
            if sim_s > self.anchor_sim + self.warmup_sim_s + self.tolerance_s:
                self.phase = 'failed'
                return 'abort:late_enable'
            self.phase = 'running'
            self.enable_sim_s = float(sim_s)
            self.enable_wall = now_wall
            return 'start_hold_enable'
        if self.phase == 'running':
            if sim_s is None:
                self.phase = 'failed'
                return 'abort:clock_lost_during_window'
            if sim_s >= self.anchor_sim + self.seconds_sim_s:
                self.phase = 'done'
                return 'finished'
            return 'wait_window'
        return 'stopped'


# --------------------------------------------------------------------------------------
# pairing / evidence summaries (pure)
# --------------------------------------------------------------------------------------

def pairing_verdict(anchor_sim, anchor_source, clock_source, enable_sim_s, warmup_sim_s,
                    runner_first_racecar_sim=None, tolerance_s=PAIRING_TOLERANCE_S):
    """The anchor is the scoring instrument's own t0; never inferred from our process start."""
    reasons = []
    if anchor_sim is None:
        reasons.append('no_anchor_from_scoring_instrument')
    if clock_source != 'sim':
        reasons.append('clock_source=%r: a paired sim-second window needs the simulation clock'
                       % clock_source)
    offset = None
    if anchor_sim is not None and enable_sim_s is not None:
        offset = float(enable_sim_s) - float(anchor_sim)
    deviation = None if offset is None else abs(offset - float(warmup_sim_s))
    if enable_sim_s is None:
        reasons.append('no_enable_timestamp')
    if deviation is not None and deviation > tolerance_s:
        reasons.append('enable_offset_deviation_s=%.3f>%.3f' % (deviation, tolerance_s))
    if anchor_sim is not None and runner_first_racecar_sim is not None \
            and abs(float(anchor_sim) - float(runner_first_racecar_sim)) > PAIRING_ANCHOR_TOLERANCE_S:
        reasons.append('anchor_vs_runner_first_racecar_s=%.3f>%.3f'
                       % (abs(float(anchor_sim) - float(runner_first_racecar_sim)),
                          PAIRING_ANCHOR_TOLERANCE_S))
    return dict(certified=not reasons, anchor_sim=anchor_sim, anchor_source=anchor_source,
                clock_source=clock_source, enable_sim_s=enable_sim_s, enable_offset_s=offset,
                deviation_s=deviation, runner_first_racecar_sim=runner_first_racecar_sim,
                reasons=reasons, comparison_note=_comparison_note())


def _comparison_note():
    return ('motion window = SECONDS - enable_offset and is paired between arms only; it is '
            'NOT comparable to historical 140 s runs of the unmatched shell launcher')


def summarize_commands(records, enable_sim_s=None, tolerance_s=0.05):
    """records: [dict(topic, sim_s, linear, wall)]. Pre-enable traffic is a hard gate."""
    commands = [r for r in records if r.get('sim_s') is not None]
    before = [r for r in commands
              if enable_sim_s is not None and float(r['sim_s']) < float(enable_sim_s) - tolerance_s]
    nonzero = [r for r in commands if abs(float(r.get('linear') or 0.0)) > 0.0
               or abs(float(r.get('angular') or 0.0)) > 0.0]
    return dict(total=len(commands), before_enable=len(before),
                before_enable_detail=[dict(topic=r['topic'], sim_s=r['sim_s'],
                                           linear=r.get('linear')) for r in before],
                first_sim_s=commands[0]['sim_s'] if commands else None,
                first_nonzero_sim_s=nonzero[0]['sim_s'] if nonzero else None,
                first_nonzero_linear=nonzero[0].get('linear') if nonzero else None)


def summarize_supervisor_csv(path):
    """Candidate evidence: the supervisor must have produced rows and a loaded guard."""
    path = Path(path)
    result = dict(path=str(path), exists=path.is_file(), rows=0, reason_counts={},
                  scaled_rows=0, guard_unavailable_n=0, gap_unavailable_n=0,
                  min_k=None, max_gap_m=None)
    if not result['exists']:
        return result
    with path.open(newline='', encoding='utf-8') as handle:
        for row in _csv.DictReader(handle):
            result['rows'] += 1
            reason = row.get('passthrough_reason') or ''
            result['reason_counts'][reason] = result['reason_counts'].get(reason, 0) + 1
            if reason == 'guard_unavailable':
                result['guard_unavailable_n'] += 1
            if reason == 'gap_unavailable':
                result['gap_unavailable_n'] += 1
            try:
                k = float(row.get('k') or 'nan')
            except ValueError:
                k = float('nan')
            if math.isfinite(k) and k < 1.0:
                result['scaled_rows'] += 1
                result['min_k'] = k if result['min_k'] is None else min(result['min_k'], k)
            try:
                gap = float(row.get('gap') or 'nan')
            except ValueError:
                gap = float('nan')
            if math.isfinite(gap):
                result['max_gap_m'] = gap if result['max_gap_m'] is None \
                    else max(result['max_gap_m'], gap)
    return result


def candidate_evidence_verdict(guard_report, csv_summary, commands_summary):
    """A candidate that only ever passed through is not evidence for the supervisor."""
    reasons = []
    if not guard_report.get('ok'):
        reasons.append('guard_not_loaded: %s' % guard_report.get('report'))
    if not (csv_summary or {}).get('exists'):
        reasons.append('supervisor_csv_missing')
    elif not csv_summary.get('rows'):
        reasons.append('supervisor_csv_empty')
    if any(reason.startswith('exception:') for reason in (csv_summary or {}).get('reason_counts', {})):
        reasons.append('supervisor_runtime_exception')
    if not (csv_summary or {}).get('scaled_rows'):
        reasons.append('no_scaled_row: permanent_passthrough_not_valid_candidate')
    if (csv_summary or {}).get('guard_unavailable_n'):
        reasons.append('guard_unavailable_rows=%d' % csv_summary['guard_unavailable_n'])
    if (csv_summary or {}).get('gap_unavailable_n') \
            and csv_summary['gap_unavailable_n'] == csv_summary.get('rows'):
        reasons.append('every_row_gap_unavailable')
    if commands_summary.get('before_enable'):
        reasons.append('commands_before_enable=%d' % commands_summary['before_enable'])
    return dict(ok=not reasons, reasons=reasons)


def summarize_mechanism(rows):
    """Independent recomputation of the supervisor's own decision, from the production guard."""
    gaps = [r['gap'] for r in rows if r.get('gap') is not None and math.isfinite(r['gap'])]
    scalings = [r['k'] for r in rows if r.get('k') is not None]
    return dict(rows=len(rows), finite_gap_rows=len(gaps),
                gap_none_rows=sum(1 for r in rows if r.get('gap') is None),
                gap_min_m=min(gaps) if gaps else None,
                gap_median_m=statistics.median(gaps) if gaps else None,
                scalings=len(scalings),
                min_k=min(scalings) if scalings else None)


# --------------------------------------------------------------------------------------
# isolation + owned process identities
# --------------------------------------------------------------------------------------

def proc_table():
    here = str(Path(__file__).resolve().parent)
    if here not in sys.path:
        sys.path.insert(0, here)
    from gazebo_braking_probe import proc_table as _table      # read-only reuse
    return _table()


def default_port_probe(port):
    with socket.socket() as sock:
        try:
            sock.bind(('127.0.0.1', int(port)))
            return True
        except OSError:
            return False


def isolation_scan(domain, port, world_name, out, table=None, port_probe=None):
    """Find conflicts and STOP. Never signals anything that is not ours."""
    table = proc_table() if table is None else table
    out = str(out)
    conflicts = []
    ancestors = {os.getpid()}
    current = os.getpid()
    while current in table and table[current]['ppid'] not in ancestors:
        current = table[current]['ppid']
        ancestors.add(current)
    for pid, row in sorted(table.items()):
        if int(pid) in ancestors:
            continue
        env = [bytes(entry) for entry in (row.get('env') or [])]
        args = str(row.get('args') or '')
        if ('ROS_DOMAIN_ID=%d' % int(domain)).encode() in env:
            conflicts.append(dict(pid=int(pid), reason='ros_domain_in_use', start=row.get('start'), args=args))
        if ('GAZEBO_MASTER_URI=http://127.0.0.1:%d' % int(port)).encode() in env:
            conflicts.append(dict(pid=int(pid), reason='gazebo_master_uri_in_use', start=row.get('start'), args=args))
        if ('gzserver' in args or 'gzclient' in args) and str(Path(out) / 'track' / world_name) in args:
            conflicts.append(dict(pid=int(pid), reason='same_world_in_use', args=args))
        if out in args:
            conflicts.append(dict(pid=int(pid), reason='output_dir_in_use', args=args))
    port_free = bool((port_probe or default_port_probe)(port))
    detail = dict(domain=int(domain), port=int(port), port_free=port_free,
                  world_name=world_name, output=str(out), conflicts=conflicts)
    if conflicts or not port_free:
        raise ConflictError(json.dumps(detail, default=str))
    return dict(detail, checked_pids=len(table))


class OwnedSet:
    """Every process we started, keyed by PID + Linux starttime. No name or domain kills."""

    def __init__(self):
        self.roots = {}
        self.known = {}

    def add(self, pid, label):
        from gazebo_braking_probe import OwnedProcesses         # read-only reuse
        owner = OwnedProcesses(int(pid))
        self.roots[int(pid)] = dict(label=label, owner=owner)
        self.capture()
        return owner

    def capture(self):
        for entry in self.roots.values():
            entry['owner'].capture()
            for known_pid, start in entry['owner'].known.items():
                self.known[int(known_pid)] = dict(label=entry['label'], start=start)
        return dict(self.known)

    def live(self):
        table = proc_table()
        return sorted(pid for pid, record in self.known.items()
                      if pid in table and table[pid]['start'] == record['start']
                      and table[pid]['state'] != 'Z')

    def cleanup(self, order=None):
        labels = list(order or [])
        ordered = [pid for label in labels for pid in self.roots if self.roots[pid]['label'] == label]
        ordered += [pid for pid in self.roots if pid not in ordered]
        evidence = {}
        for pid in ordered:
            entry = self.roots[pid]
            evidence[entry['label']] = entry['owner'].cleanup()
        self.capture()
        return dict(owned_pid_starttimes=self.known, per_root=evidence,
                    remaining_live=self.live())


# --------------------------------------------------------------------------------------
# evidence
# --------------------------------------------------------------------------------------

def source_targets(cfg):
    """Every file that can change the meaning of a run, hashed into the manifest.

    2026-10-08：只登记**仓内**件 —— 被驱动的臂与仓内宿主、共享模块、
    本仓实测车模（simulation/gazebo）、面板赛道；队友仓只留裁判（track_eval）与跨环境
    一致性参照（fastsim/params.py）。预登记 md 属于实验过程记录，不再进仓库也不进清单。
    """
    repo, team, track = Path(cfg['repo']), Path(cfg['team_repo']), Path(cfg['track'])
    return [
        repo / 'evaluation' / 'gazebo_supervised_runner.py',
        repo / 'evaluation' / 'test_gazebo_supervised_runner.py',
        repo / 'evaluation' / 'gazebo_track_progress.py',
        repo / 'evaluation' / 'test_gazebo_track_progress.py',
        repo / 'evaluation' / 'gazebo_speed_supervisor.py',
        repo / 'evaluation' / 'test_gazebo_speed_supervisor.py',
        repo / 'evaluation' / 'run_gazebo_supervised_smoke.sh',
        repo / 'evaluation' / 'run_gazebo_supervised_once.sh',
        repo / 'evaluation' / 'gazebo_braking_probe.py',
        repo / 'arms' / 'corridor_arm.py',
        repo / 'arms' / 'arm_host.py',
        repo / 'common' / 'params.py',
        repo / 'common' / 'pwm.py',
        repo / 'common' / 'control.py',
        repo / 'common' / 'road_policy.py',
        repo / 'common' / 'road_runtime.py',
        repo / 'common' / 'corridor.py',
        repo / 'vehicle' / 'scan_guard.py',
        repo / 'simulation' / 'gazebo' / 'src' / 'racecar_sim' / 'config' / 'params.yaml',
        repo / 'simulation' / 'gazebo' / 'src' / 'racecar_sim' / 'urdf' / 'racecar.xacro',
        repo / 'simulation' / 'gazebo' / 'src' / 'racecar_sim' / 'launch' / 'sim_bringup.launch.py',
        repo / 'simulation' / 'gazebo' / 'src' / 'racecar_sim' / 'racecar_sim' / 'sim_wiring.py',
        team / '3Dsim' / 'tools' / 'track_eval.py',
        team / 'fastsim' / 'params.py',
        track / 'layout.yaml', track / 'cones.csv', track / 'track_sim.world',
    ]


def installed_targets(cfg):
    from ament_index_python.packages import get_package_share_directory
    share = Path(get_package_share_directory('racecar_sim'))
    prefix = share.parent.parent
    wiring = importlib.util.find_spec('racecar_sim.sim_wiring')
    if wiring is None or not wiring.origin:
        raise ConfigError('actual installed sim_wiring cannot be located')
    targets = [share / 'launch' / 'sim_bringup.launch.py',
               prefix / 'lib' / 'racecar_sim' / 'sim_wiring', Path(wiring.origin)]
    for directory in (share / 'config', share / 'urdf'):
        if not directory.is_dir():
            raise ConfigError('installed resource directory missing: %s' % directory)
        targets.extend(sorted(path for path in directory.rglob('*') if path.is_file()
                              and '__pycache__' not in path.parts))
    return targets


def check_sim_source(source, repo, sim_setup=None):
    """`racecar_sim` 必须来自本仓 `simulation/gazebo/install`，除非 SIM_SETUP 被显式指到别处。

    本仓 install/ 没 build 成功时（colcon 失败也会留下一个 stub `install/setup.bash`，所以入口脚本
    的"缺少 SIM_SETUP"那道检查拦不住），AMENT_PREFIX_PATH 里队友那份旧几何 racecar_sim 会顶上：
    整局照样跑完、照样认证，但跑的是旧车模（几何 0.33/0.07/0.21 对 0.35/0.145/0.28），读数不可比。
    要故意跑旧几何，就把 SIM_SETUP 显式指到那份 install/setup.bash —— 那是知情选择，不是静默换源。
    """
    source = Path(source)
    expected = Path(repo) / 'simulation' / 'gazebo' / 'install'
    below = expected == source or expected in source.parents
    sim_setup = (sim_setup or '').strip()
    explicit_foreign = bool(sim_setup) and Path(sim_setup) != expected / 'setup.bash'
    if not below and not explicit_foreign:
        raise ConfigError(
            "racecar_sim resolves to %s, not this repo's %s: the in-repo sim is not built (a failed "
            'colcon build leaves a stub install/setup.bash behind, so the entry starts and then silently '
            "runs the team's old-geometry car). Build it with: source /opt/ros/humble/setup.bash && "
            'cd simulation/gazebo && colcon build. To run the old-geometry world on purpose, point '
            'SIM_SETUP at that install/setup.bash explicitly.' % (source, expected))
    return dict(sim_in_repo=below, sim_source=str(source),
                sim_setup=sim_setup or str(expected / 'setup.bash'))


def sim_source_check(cfg):
    try:
        from ament_index_python.packages import get_package_share_directory
        share = Path(get_package_share_directory('racecar_sim'))
    except BaseException as exc:
        return dict(name='sim_source', ok=False, detail=repr(exc))
    try:
        info = check_sim_source(share / 'launch' / 'sim_bringup.launch.py', cfg['repo'],
                                os.environ.get('SIM_SETUP'))
    except ConfigError as exc:
        return dict(name='sim_source', ok=False, detail=str(exc))
    return dict(name='sim_source', ok=True, detail=info)


def source_hashes(manifest):
    hashes = {record['path']: record.get('sha256') for record in manifest.get('artifacts', [])}
    hashes[manifest['npz']['path']] = manifest['npz']['sha256']
    return hashes


def unchanged_sources(manifest):
    changed = []
    for path, expected in source_hashes(manifest).items():
        try:
            actual = sha256_file(path)
        except OSError:
            actual = None
        if not expected or actual != expected:
            changed.append(path)
    return dict(ok=not changed, changed=changed)


def snapshot_sources(out, cfg):
    """Copy + hash every source that can change the meaning of the run."""
    out = Path(out)
    folder = out / 'sources'
    folder.mkdir(exist_ok=True)
    records = []
    targets = list(dict.fromkeys(source_targets(cfg) + installed_targets(cfg)))
    for index, path in enumerate(targets):
        path = Path(path)
        record = dict(path=str(path))
        try:
            content = path.read_bytes()
        except OSError as exc:
            record.update(error='%s: %s' % (type(exc).__name__, exc), sha256=None)
            records.append(record)
            continue
        destination = folder / ('%02d_%s' % (index, path.name))
        destination.write_bytes(content)
        import hashlib
        record.update(sha256=hashlib.sha256(content).hexdigest(), bytes=len(content),
                      snapshot_path=str(destination))
        records.append(record)
    npz = Path(cfg['npz'])
    manifest = dict(schema_version=SCHEMA_VERSION, preregistration=preregistration_id(cfg),
                    arm=cfg['arm'], observer_only=False, physical_goal_complete=False,
                    config=serializable(cfg), artifacts=records,
                    npz=dict(path=str(npz), sha256=sha256_file(npz) if npz.is_file() else None),
                    versions=dict(python=sys.version, platform=sys.platform, argv=list(sys.argv)))
    import hashlib
    manifest['self_sha256'] = hashlib.sha256(
        json.dumps(manifest, sort_keys=True, default=str).encode()).hexdigest()
    write_json(out / 'manifest.json', manifest)
    missing = [record['path'] for record in records if not record.get('sha256')]
    if missing:
        raise ConfigError('required snapshot sources missing: %s' % missing)
    return manifest


# --------------------------------------------------------------------------------------
# live arm
# --------------------------------------------------------------------------------------

def controller_path_pattern():
    return r"(?m)^    ctrl_params_path = '/tmp/racecar_ros2_control_d%s.yaml' % \(\n        os.environ.get\('ROS_DOMAIN_ID', '0'\)\)"


def re_controller_path_matches(text):
    import re
    return len(re.findall(controller_path_pattern(), text)) == 1


def prepare_launch(cfg, out):
    from ament_index_python.packages import get_package_share_directory
    source = Path(get_package_share_directory('racecar_sim')) / 'launch' / 'sim_bringup.launch.py'
    # 静默换源守卫：本仓没 build 成功时这里会拿到队友那份旧几何 racecar_sim。
    info = check_sim_source(source, cfg['repo'], os.environ.get('SIM_SETUP'))
    # 恢复行为自证：盲倒默认关（只停不倒，与实车一致）。
    # 报告里要注明用的是哪种恢复行为，所以每局记下这一位。
    info['reverse_enabled'] = (os.environ.get('ARM_HOST_REVERSE') or '').strip().lower() \
        in ('1', 'true', 'yes', 'on')
    text = source.read_text(encoding='utf-8')
    result = prepare_launch_text(text, source, out)
    result.update(info)
    write_json(Path(out) / 'launch_manifest.json', result)
    return result


def prepare_launch_text(text, source, out):
    out = Path(out)
    # Keep installed node/controller chain; only relocate its generated config file.
    pattern = controller_path_pattern()
    import re
    import difflib
    patched, count = re.subn(pattern, lambda _: '    ctrl_params_path = ' + repr(str(out / 'generated_controller.yaml')), text)
    if count != 1:
        raise ConfigError('installed launch controller-path assignment not recognized')
    original = out / 'installed_sim_bringup.snapshot.py'
    generated = out / 'owned_sim.launch.py'
    original.write_text(text, encoding='utf-8')
    generated.write_text(patched, encoding='utf-8')
    (out / 'launch.diff').write_text(''.join(difflib.unified_diff(text.splitlines(True), patched.splitlines(True),
                                                              fromfile=str(source), tofile=str(generated))), encoding='utf-8')
    result = dict(installed_source=str(source), installed_sha256=sha256_file(original),
                  owned_sha256=sha256_file(generated), changes='controller output path only',
                  world_absolute_path=True)
    write_json(out / 'launch_manifest.json', result)
    return result


def _popen(command, env, out_dir, log_path, cmds_path, owned, label):
    handle = open(log_path, 'a', buffering=1)
    process = subprocess.Popen(command, cwd=str(out_dir), env=env, stdout=handle,
                               stderr=subprocess.STDOUT, start_new_session=True)
    owned.add(process.pid, label)
    append_jsonl(cmds_path, dict(event='spawn', wall=time.time(), pid=process.pid,
                                 argv=list(command), cwd=str(out_dir), log=str(log_path)))
    return process, handle


def _max_gap(records):
    walls = [r['wall'] for r in records]
    if len(walls) < 2:
        return None
    return max(b - a for a, b in zip(walls, walls[1:]))


def check_guard_import(cfg):
    """The candidate arm is only meaningful if the production ScanGuard really imports."""
    try:
        module = load_supervisor_module(cfg)
        guard_dir = str(Path(cfg['repo']) / 'vehicle')
        report = module.guard_load_report(guard_dir)
        cls = module.load_scan_guard(guard_dir)
        cls()
        loaded_path = Path(sys.modules[cls.__module__].__file__).resolve()
        if loaded_path != (Path(guard_dir) / 'scan_guard.py').resolve():
            raise ConfigError('ScanGuard loaded from wrong source: %s' % loaded_path)
        return dict(name='guard_import', ok=bool(report.get('ok')), report=report.get('report'),
                    scan_guard_py=report.get('scan_guard_py'))
    except BaseException as exc:                                   # noqa: BLE001
        return dict(name='guard_import', ok=False,
                    report='%s: %s' % (type(exc).__name__, exc))


def run_arm(cfg, out):
    """Run exactly one arm. Returns a process exit code (0 = arm completed with evidence)."""
    import rclpy
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.node import Node
    from rclpy.parameter import Parameter
    from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy)
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import LaserScan
    from std_msgs.msg import Bool
    from gazebo_msgs.msg import ModelStates
    from rosgraph_msgs.msg import Clock

    arm = cfg['arm']
    out = Path(out)
    out.mkdir(parents=True, exist_ok=False)
    (out / 'track').mkdir()
    for name in REQUIRED_TRACK_FILES:
        (out / 'track' / name).write_bytes((Path(cfg['track']) / name).read_bytes())
    world_path = out / 'track' / 'track_sim.world'
    world_name = world_path.name
    commands = arm_commands(arm, cfg, out, world_path)
    cmds_path = out / 'cmds.jsonl'
    polls_path = out / 'graph_polls.jsonl'
    progress_path = out / 'scorer_progress.jsonl'
    world_path_jsonl = out / 'world_samples.jsonl'
    frames_path = out / 'scan_frames.jsonl'
    mechanism_path = out / 'mechanism.jsonl'
    deadline = time.monotonic() + READY_DEADLINE_WALL_S + ANCHOR_DEADLINE_WALL_S + cfg['seconds'] * MAX_WALL_FACTOR

    report = dict(requested_n=1, schema_version=SCHEMA_VERSION, preregistration=preregistration_id(cfg), arm=arm,
                  status='running', physical_goal_complete=False,
                  track=dict(source=str(cfg['track']), world_sha256=sha256_file(world_path)),
                  isolation=None, failure=None)
    write_json(out / 'report.json', report)
    checks = [check_venv()]
    if arm == 'candidate':
        checks.append(check_guard_import(cfg))
    report['checks'] = checks
    guard_report = next((c for c in checks if c['name'] == 'guard_import'),
                        dict(name='guard_import', ok=False, report='not_checked'))

    failure, cleanup, stopped_by, manifest = None, None, None, None
    records = {key: [] for key in ('clock', 'world', 'odom', 'scan', 'cmd', 'raw', 'enable')}
    state = dict(sim_s=None, clock_wall=None, world=None, odom_n=0, scan_n=0,
                 first_racecar_after_scorer_sim=None, first_world_motion=None,
                 warmup_xy=None, scan_wall=None, odom_wall=None, world_wall=None)
    node = owned = sim = policy = supervisor = holder = scorer = None
    scorer_meta, guard, supervisor_module = None, None, None
    sim_handle = policy_handle = supervisor_handle = holder_handle = None
    executor = None
    enable_pub = heartbeat_timer = None
    original_signals = {}
    frames_bytes, frame_rows = 0, 0
    handles = []
    protocol = StartupProtocol(arm, cfg['warmup'], cfg['seconds'])

    def on_clock(message):
        value = message.clock.sec + message.clock.nanosec * 1e-9
        state['sim_s'] = value
        state['clock_wall'] = time.monotonic()
        records['clock'].append(dict(sim_s=value, wall=state['clock_wall']))
        append_jsonl(out / 'clock_samples.jsonl', records['clock'][-1])

    def on_world(message):
        names = list(message.name)
        payload = dict(names=names, racecar_count=names.count('racecar'),
                       marker_box='marker_box' in names)
        if payload['racecar_count'] == 1:
            index = names.index('racecar')
            pose = message.pose[index]
            twist = message.twist[index]
            payload.update(linear_x=float(twist.linear.x),
                           linear=[float(twist.linear.x), float(twist.linear.y), float(twist.linear.z)],
                           angular=[float(twist.angular.x), float(twist.angular.y), float(twist.angular.z)],
                           position=[float(pose.position.x), float(pose.position.y)],
                           orientation=[float(pose.orientation.x), float(pose.orientation.y),
                                        float(pose.orientation.z), float(pose.orientation.w)])
            if scorer is not None and state['first_racecar_after_scorer_sim'] is None:
                state['first_racecar_after_scorer_sim'] = state['sim_s']
            xy = payload['position']
            if scorer is not None and scorer.t0 is not None:
                if state['warmup_xy'] is None:
                    state['warmup_xy'] = xy
                motion = (math.hypot(xy[0] - state['warmup_xy'][0], xy[1] - state['warmup_xy'][1]) > 0.02
                          or math.hypot(twist.linear.x, twist.linear.y) > 0.05)
                if motion and state['first_world_motion'] is None:
                    state['first_world_motion'] = dict(sim_s=state['sim_s'], wall=time.monotonic(), position=xy)
                if motion and protocol.enable_sim_s is None:
                    raise RuntimeError('abort:world_motion_before_enable')
        state['world_wall'] = time.monotonic()
        state['world'] = payload
        records['world'].append(dict(sim_s=state['sim_s'], wall=time.monotonic(), **payload))
        append_jsonl(world_path_jsonl, records['world'][-1])

    def on_odom(message):
        state['odom_n'] += 1
        state['odom_wall'] = time.monotonic()
        records['odom'].append(dict(sim_s=state['sim_s'], wall=time.monotonic(),                                     stamp=[message.header.stamp.sec, message.header.stamp.nanosec],
                                     position=[message.pose.pose.position.x, message.pose.pose.position.y],
                                     orientation=[message.pose.pose.orientation.x, message.pose.pose.orientation.y,
                                                  message.pose.pose.orientation.z, message.pose.pose.orientation.w],
                                     linear_x=float(message.twist.twist.linear.x),
                                     angular_z=float(message.twist.twist.angular.z)))
        append_jsonl(out / 'odom_samples.jsonl', records['odom'][-1])

    def on_scan(message):
        nonlocal frames_bytes, frame_rows
        state['scan_n'] += 1
        wall, sim_s = time.monotonic(), state['sim_s']
        state['scan_wall'] = wall
        records['scan'].append(dict(sim_s=sim_s, wall=wall))
        if guard is not None:
            try:
                guard.update(message)
            except BaseException as exc:                            # noqa: BLE001
                append_jsonl(cmds_path, dict(event='observer_guard_update_failed',
                                             wall=time.time(),
                                             detail='%s: %s' % (type(exc).__name__, exc)))
        row = dict(wall=wall, sim_s=sim_s, n=len(message.ranges),
                   stamp=[message.header.stamp.sec, message.header.stamp.nanosec],
                   frame_id=message.header.frame_id, angle_max=float(message.angle_max),
                   time_increment=float(message.time_increment), scan_time=float(message.scan_time),
                   nonfinite_encoding='NaN/+Infinity/-Infinity strings, reversible; finite floats unrounded',
                   angle_min=float(message.angle_min),
                   angle_increment=float(message.angle_increment),
                   range_min=float(message.range_min), range_max=float(message.range_max))
        row['ranges'] = [float(value) for value in message.ranges]
        row['intensities'] = [float(value) for value in message.intensities]
        frames_bytes += 8 * (len(message.ranges) + len(message.intensities))
        frame_rows += 1
        append_jsonl(frames_path, row)

    def observe_raw(message):
        """Independent recomputation of the supervisor's decision (mechanism diagnosis only)."""
        linear, angular = float(message.linear.x), float(message.angular.z)
        row = dict(wall=time.monotonic(), sim_s=state['sim_s'], v_raw=linear, w_raw=angular,
                   steer=None, gap=None, gap_kind=None, v_allow=None, k=None)
        if supervisor_module is not None and guard is not None and abs(linear) > 1e-6:
            steer = supervisor_module.steer_from_yaw_rate(linear, angular, cfg['wheelbase'])
            gap = guard.gap(steer)
            row['steer'] = steer
            row['gap_kind'] = ('none' if gap is None else
                               'inf' if math.isinf(gap) else 'finite')
            row['gap'] = None if gap is None or not math.isfinite(gap) else float(gap)
            if gap is not None and math.isfinite(gap) and gap > 0.0:
                v_allow = supervisor_module.allowable_speed(gap, cfg['L'], cfg['u'])
                row['v_allow'] = v_allow
                if linear > 0.0:
                    row['k'] = min(1.0, v_allow / linear)
        append_jsonl(mechanism_path, row)
        return row

    def on_cmd(message):
        records['cmd'].append(dict(topic=CMD_TOPIC, sim_s=state['sim_s'], wall=time.monotonic(),
                                   linear=float(message.linear.x),
                                    angular=float(message.angular.z)))
        append_jsonl(out / 'commands.jsonl', records['cmd'][-1])
        if protocol.enable_sim_s is None and (message.linear.x != 0 or message.angular.z != 0):
            raise RuntimeError('abort:nonzero_cmd_before_enable')

    def on_raw(message):
        records['raw'].append(dict(topic=RAW_TOPIC, sim_s=state['sim_s'], wall=time.monotonic(),
                                   linear=float(message.linear.x), angular=float(message.angular.z)))
        append_jsonl(out / 'commands.jsonl', records['raw'][-1])
        if protocol.enable_sim_s is None and (message.linear.x != 0 or message.angular.z != 0):
            raise RuntimeError('abort:nonzero_raw_before_enable')
        observe_raw(message)

    def on_enable(message):
        records['enable'].append(dict(sim_s=state['sim_s'], wall=time.monotonic(),
                                      data=bool(message.data)))

    def on_signal(signum, _frame):
        raise RunnerInterrupted('signal %d' % signum)

    def graph_poll():
        try:
            topics = {}
            for topic in (CMD_TOPIC, RAW_TOPIC, REF_TOPIC, ENABLE_TOPIC):
                topics[topic] = [dict(node_name=info.node_name,
                                      node_namespace=info.node_namespace,
                                      endpoint_gid=bytes(info.endpoint_gid).hex(),
                                      topic_type=info.topic_type)
                                 for info in node.get_publishers_info_by_topic(topic)]
            nodes = [(name, namespace) for name, namespace in node.get_node_names_and_namespaces()]
            subscribers = {topic: [dict(node_name=info.node_name, node_namespace=info.node_namespace,
                                          endpoint_gid=bytes(info.endpoint_gid).hex())
                                   for info in node.get_subscriptions_info_by_topic(topic)]
                           for topic in (CMD_TOPIC, RAW_TOPIC, ENABLE_TOPIC)}
            return dict(error=None, wall=time.monotonic(), sim_s=state['sim_s'],
                        topics=topics, nodes=nodes, subscribers=subscribers)
        except BaseException as exc:                                # noqa: BLE001
            return dict(error='%s: %s' % (type(exc).__name__, exc), wall=time.monotonic(),
                        sim_s=state['sim_s'], topics={}, nodes=[])

    def poll_and_record(label):
        poll = graph_poll()
        verdict = evaluate_graph_poll(poll, graph_expectation(arm))
        append_jsonl(polls_path, dict(label=label, **poll, verdict=verdict))
        return poll, verdict

    def start_nodes():
        nonlocal policy, supervisor, guard, supervisor_module
        if arm == 'candidate':
            supervisor_module = load_supervisor_module(cfg)
            guard = supervisor_module.load_scan_guard(str(Path(cfg['repo']) / 'vehicle'))()
            supervisor, handle = _popen(commands['supervisor']['argv'], env, out,
                                         commands['supervisor']['log'], cmds_path, owned, 'supervisor')
            handles.append(handle)
        policy, handle = _popen(commands['policy']['argv'], env, out,
                                commands['policy']['log'], cmds_path, owned, 'policy')
        handles.append(handle)

    def start_scorer():
        nonlocal scorer, scorer_meta
        scorer, scorer_meta = load_scorer(cfg, out)
        executor.add_node(scorer)
        write_json(out / 'track_eval_meta.json', scorer_meta)
        append_jsonl(cmds_path, dict(event='lifecycle', wall=time.time(), status='scorer_created', **scorer_meta))

    def heartbeat(enabled=True):
        message = Bool()
        message.data = enabled
        enable_pub.publish(message)
        append_jsonl(cmds_path, dict(event='enable_publish', wall=time.monotonic(),
                                     sim_s=state['sim_s'], enabled=enabled))

    def start_enable():
        nonlocal heartbeat_timer
        poll, verdict = poll_and_record('pre_enable')
        if not verdict['ok']:
            raise RuntimeError('abort:graph_not_ready_before_enable:%s' % ';'.join(verdict['reasons']))
        if scorer.clock_source != 'sim' or abs(scorer.sim_t - (scorer.t0 + cfg['warmup'])) > PAIRING_TOLERANCE_S:
            raise RuntimeError('abort:scorer_enable_anchor_mismatch')
        protocol.enable_sim_s = scorer.sim_t
        heartbeat()
        heartbeat_timer = node.create_timer(0.1, heartbeat)

    for sig in (signal.SIGINT, signal.SIGTERM):
        original_signals[sig] = signal.signal(sig, on_signal)

    try:
        if not all(check['ok'] for check in checks):
            raise ConfigError('required venv/guard checks failed: %s' % checks)
        if sha256_file(cfg['npz']) != R36_SHA256:
            raise ConfigError('frozen r36 changed before launch')
        report['isolation'] = isolation_scan(cfg['domain'], cfg['port'], world_name, out)
        manifest = snapshot_sources(out, cfg)
        report['manifest_sha256'] = manifest['self_sha256']
        report['launch'] = prepare_launch(cfg, out)
        env = child_env(dict(cfg, out=out))
        env['ROAD_DEBUG'] = str(out / 'policy.csv')
        os.environ.update({key: env[key] for key in THREAD_ENV.keys() | {'ROS_DOMAIN_ID', 'GAZEBO_MASTER_URI', 'SCAN_GUARD_DIR', 'ROS_LOG_DIR', 'PYTHONDONTWRITEBYTECODE', 'GAZEBO_MODEL_DATABASE_URI'}})
        sys.dont_write_bytecode = True
        rclpy.init()
        executor = SingleThreadedExecutor()
        node = Node('supervised_runner',
                    parameter_overrides=[Parameter('use_sim_time', value=False)])
        executor.add_node(node)
        clock_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT,
                               durability=DurabilityPolicy.VOLATILE,
                               history=HistoryPolicy.KEEP_LAST)
        node.create_subscription(Clock, CLOCK_TOPIC, on_clock, clock_qos)
        node.create_subscription(ModelStates, WORLD_TOPIC, on_world, 10)
        node.create_subscription(Odometry, ODOM_TOPIC, on_odom, 10)
        node.create_subscription(LaserScan, SCAN_TOPIC, on_scan, clock_qos)
        node.create_subscription(Twist, CMD_TOPIC, on_cmd, 10)
        node.create_subscription(Twist, RAW_TOPIC, on_raw, 10)
        node.create_subscription(Bool, ENABLE_TOPIC, on_enable, 10)
        enable_pub = node.create_publisher(Bool, ENABLE_TOPIC, 1)
        owned = OwnedSet()
        sim, sim_handle = _popen(commands['sim']['argv'], env, out, commands['sim']['log'],
                                 cmds_path, owned, 'sim')
        handles.append(sim_handle)
        start_nodes()
        append_jsonl(cmds_path, dict(event='lifecycle', wall=time.time(),
                                     status='sim_launched', pid=sim.pid))

        next_poll_wall, next_progress_wall = 0.0, 0.0
        window_close_wall = None
        poll, verdict = dict(error=None, topics={}, nodes=[]), dict(ok=False, reasons=['no_poll'])
        while True:
            owned.capture()
            if time.monotonic() > deadline:
                raise RuntimeError('abort:runner_wall_deadline')
            executor.spin_once(timeout_sec=0.02)
            for label, process in (('policy', policy), ('supervisor', supervisor)):
                if process is not None and process.poll() is not None:
                    raise RuntimeError('abort:%s_exited_%s' % (label, process.returncode))
            now_wall = time.monotonic()
            if sim.poll() is not None:
                raise RuntimeError('abort:sim_launch_exited_%d' % sim.returncode)
            if now_wall >= next_poll_wall:
                next_poll_wall = now_wall + GRAPH_POLL_INTERVAL_S
                poll, verdict = poll_and_record(protocol.phase)
                if protocol.phase in ('warmup', 'running') and not verdict['ok']:
                    raise RuntimeError('abort:graph_identity_lost:%s' % verdict['reasons'])
            state_now = dict(sim_s=state['sim_s'], now_wall=now_wall, world=state['world'],
                             odom_n=state['odom_n'], scan_n=state['scan_n'],
                             clock_wall=state['clock_wall'], graph_ok=verdict['ok'],
                             graph_reasons=verdict['reasons'], has_scan=bool(state['scan_n']),
                             anchor_sim=(scorer.t0 if scorer is not None else None))
            ready = readiness_report(state_now, poll, graph_expectation(arm))
            state_now.update(ready=ready['ok'], ready_reasons=ready['reasons'])
            action = protocol.next_action(state_now)
            if action == 'start_scorer':
                start_scorer()
                append_jsonl(cmds_path, dict(event='lifecycle', wall=time.time(),
                                             status='children_started'))
                continue
            if scorer is not None and scorer.clock_source not in (None, 'sim'):
                raise RuntimeError('abort:scorer_wall_fallback')
            if scorer is not None and scorer.t0 is not None and time.time() - scorer.t0_wall > cfg['seconds'] * MAX_WALL_FACTOR:
                raise RuntimeError('abort:scorer_wall_budget')
            if protocol.phase in ('warmup', 'running') and (state['clock_wall'] is None or now_wall - state['clock_wall'] > CLOCK_STALE_WALL_S):
                raise RuntimeError('abort:clock_stale')
            if action == 'start_hold_enable':
                if not ready['ok'] or now_wall - state['scan_wall'] > 0.5 or now_wall - state['odom_wall'] > 2.0 or now_wall - state['world_wall'] > 2.0:
                    raise RuntimeError('abort:feedback_not_fresh_at_enable')
                start_enable()
                continue
            if action.startswith('abort:'):
                raise RuntimeError(action)
            closed_by_instrument = scorer is not None and scorer.finished()
            budget_met = action == 'finished' or protocol.phase == 'done'
            if budget_met or (protocol.phase == 'running' and closed_by_instrument):
                # `budget_met` is the runner's own /clock view. The instrument keeps its own
                # sim_t from its own /clock subscription, and spin_once runs one callback at a
                # time, so the runner can be one 0.1 s sample ahead. Stopping on the runner
                # alone truncated PAIR02's measured window to 139.9 s and failed the arm as
                # incomplete_sim_window. Wait (bounded) for the instrument to close its own
                # budget; the strict elapsed_s check below still rejects a genuinely short run.
                if budget_met and not closed_by_instrument and scorer is not None:
                    if window_close_wall is None:
                        window_close_wall = now_wall
                    if now_wall - window_close_wall <= SCORER_CLOSE_GRACE_WALL_S:
                        continue
                stopped_by = ('scorer_sim_budget' if closed_by_instrument
                              else 'protocol_sim_budget')
                break
            if scorer is not None and now_wall >= next_progress_wall:
                next_progress_wall = now_wall + PROGRESS_INTERVAL_WALL_S
                append_jsonl(progress_path, dict(wall=time.time(), sim_s=state['sim_s'],
                                                 phase=protocol.phase,
                                                 report=scorer.report()))
    except BaseException as exc:                                    # noqa: BLE001
        failure = '%s: %s' % (type(exc).__name__, exc)
    finally:
        for sig in original_signals:
            signal.signal(sig, signal.SIG_IGN)
        exit_codes = {}
        if heartbeat_timer is not None:
            try:
                heartbeat_timer.cancel()
            except BaseException as exc:
                failure = (failure or '') + ' heartbeat_cancel_failed:' + repr(exc)
        if enable_pub is not None:
            try:
                heartbeat(False)
                executor.spin_once(timeout_sec=0.02)
            except BaseException:
                pass
        for label, process in (('policy', policy), ('supervisor', supervisor), ('sim', sim)):
            if process is not None:
                exit_codes[label] = process.poll()
        if scorer is not None and executor is not None:
            try:
                executor.remove_node(scorer)
            except BaseException:                                   # noqa: BLE001
                pass
        if owned is not None:
            try:
                cleanup = owned.cleanup(order=['policy', 'supervisor', 'sim'])
            except BaseException as exc:
                failure = (failure or '') + ' cleanup_error:' + repr(exc)
                cleanup = dict(owned_pid_starttimes=owned.known, remaining_live=owned.live(), error=repr(exc))
        if sim is not None:
            try:
                sim.wait(timeout=5)
            except subprocess.TimeoutExpired:
                pass
        for handle in handles:
            try:
                handle.close()
            except OSError:
                pass
        if failure:
            write_json(out / 'failure.json', dict(arm=arm, failure=failure,
                                                  exit_codes=exit_codes, stopped_by=stopped_by,
                                                  sim_now_sim_s=state['sim_s'], wall=time.time()))
        if cleanup is not None:
            write_json(out / 'cleanup.json', cleanup)
        scorer_result = None
        if scorer is not None:
            try:
                scorer_result = scorer.report()
                write_json(out / 'track_eval.json', scorer_result)
            except BaseException as exc:                            # noqa: BLE001
                failure = (failure or '') + ' scorer_report_failed:%s' % type(exc).__name__
        # Observational only: does the driven path advance along the track? Read back from the
        # file just written, so the audit measures exactly what a later reader would. It feeds
        # no pass condition; `audit_report` swallows its own errors so a missing centreline can
        # never turn a scored episode into a failure.
        report['track_progress_audit'] = track_progress.audit_report(
            out, cfg['track'], cfg['seconds'])
        if executor is not None:
            try:
                executor.shutdown(timeout_sec=2.0)
            except BaseException:                                   # noqa: BLE001
                pass
        try:
            if node is not None:
                node.destroy_node()
            if scorer is not None:
                scorer.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
        except BaseException:                                       # noqa: BLE001
            pass
        generated_config = out / 'generated_controller.yaml'
        if manifest is not None:
            report['source_stability'] = unchanged_sources(manifest)
            if not report['source_stability']['ok']:
                failure = (failure or '') + ' required_source_changed'
        report['controller_sha256'] = sha256_file(generated_config) if generated_config.is_file() else None
        if not report['controller_sha256']:
            failure = (failure or '') + ' generated_controller_missing'
        for sig, handler in original_signals.items():
            signal.signal(sig, handler)
        timeline = dict(arm=arm,
                         score_anchor=None if scorer is None else scorer.t0,
                         enable_sim=protocol.enable_sim_s, first_world_motion=state['first_world_motion'],
                         first_nonzero_cmd=summarize_commands(records['cmd'])['first_nonzero_sim_s'],
                        protocol=dict(phase=protocol.phase, anchor_sim=protocol.anchor_sim,
                                      enable_sim_s=protocol.enable_sim_s,
                                      enable_wall=protocol.enable_wall,
                                      warmup_sim_s=cfg['warmup'], seconds_sim_s=cfg['seconds']),
                        stopped_by=stopped_by, exit_codes=exit_codes,
                        scorer_meta=scorer_meta,
                        counters={key: len(value) for key, value in records.items()},
                        scan_frames=frame_rows, scan_range_bytes=frames_bytes,
                        first_racecar_after_scorer_sim=state['first_racecar_after_scorer_sim'],
                        note=('host stalls and whether simulation time freezes during them are '
                              'not asserted here; only measured /clock and world receipts are '
                              'evidence'))
        timeline['cmd'] = summarize_commands(records['cmd'], protocol.enable_sim_s)
        timeline['raw'] = summarize_commands(records['raw'], protocol.enable_sim_s)
        timeline['enable_receipts'] = [r for r in records['enable'] if r.get('data')][:5]
        timeline['odom_max_gap_wall_s'] = _max_gap(records['odom'])
        timeline['clock_max_gap_wall_s'] = _max_gap(records['clock'])
        timeline['world_max_gap_wall_s'] = _max_gap(records['world'])
        timeline['pairing'] = pairing_verdict(
            None if scorer is None else scorer.t0,
            'track_eval.Eval.t0' if scorer is not None else 'none',
            None if scorer is None else (scorer.clock_source or 'unknown'),
            protocol.enable_sim_s, cfg['warmup'], state['first_racecar_after_scorer_sim'])
        timeline['scorer_result'] = scorer_result
        timeline['track_progress_audit'] = report['track_progress_audit']
        if scorer is not None:
            timeline['pairing']['motion_window_sim_s'] = (
                None if protocol.enable_sim_s is None
                else round(cfg['seconds'] - (protocol.enable_sim_s - scorer.t0), 3))
        csv_summary = summarize_supervisor_csv(out / 'supervisor.csv') if arm == 'candidate' else None
        timeline['supervisor_csv'] = csv_summary
        if arm == 'candidate':
            child_log = out / 'supervisor.log'
            child_guard_loaded = child_log.is_file() and 'SUPERVISOR_GUARD_LOAD ok=1' in child_log.read_text(encoding='utf-8', errors='replace')
            if not child_guard_loaded:
                failure = (failure or '') + ' child_supervisor_guard_load_unproven'
            try:
                mechanism = summarize_mechanism(
                    [json.loads(line) for line in mechanism_path.read_text().splitlines()])
            except OSError:
                mechanism = dict(rows=0)
            timeline['mechanism'] = mechanism
            timeline['raw_out_count_match'] = timeline['raw']['total'] == timeline['cmd']['total']
            timeline['candidate_evidence'] = candidate_evidence_verdict(
                guard_report, csv_summary, timeline['raw'])
            if not timeline['raw_out_count_match']:
                failure = (failure or '') + ' raw_out_count_mismatch'
        write_json(out / 'timeline.json', timeline)
        if not timeline['pairing']['certified']:
            failure = (failure or '') + ' pairing_not_certified'
        if timeline.get('candidate_evidence') and not timeline['candidate_evidence']['ok']:
            failure = (failure or '') + ' candidate_evidence_invalid:' + ';'.join(timeline['candidate_evidence']['reasons'])
        if scorer_result is None or scorer_result['clock_source'] != 'sim' or scorer_result['elapsed_s'] < cfg['seconds']:
            failure = (failure or '') + ' incomplete_sim_window'
        if cleanup is None:
            cleanup = dict(owned_pid_starttimes={}, remaining_live=[])
            write_json(out / 'cleanup.json', cleanup)
        report.update(status='ok' if not failure else 'failed', failure=failure,
                      cleanup_remaining_live=(cleanup or {}).get('remaining_live'),
                      pairing=timeline['pairing'], scorer_result=scorer_result,
                      evidence=timeline.get('candidate_evidence'))
        if (cleanup or {}).get('remaining_live'):
            report['status'] = 'failed'
            report['failure'] = (report['failure'] or '') + ' owned_process_cleanup_failed'
        write_json(out / 'report.json', report)
        if report['failure']:
            write_json(out / 'failure.json', dict(arm=arm, failure=report['failure'],
                                                 exit_codes=exit_codes, stopped_by=stopped_by))
    return 0 if report['status'] == 'ok' else 1


def run_pair(cfg):
    """The two requested arms always stay in the denominator, including startup failures."""
    root = Path(cfg['out'])
    root.mkdir(parents=True, exist_ok=False)
    requested = {arm: dict(status='requested', failure=None) for arm in ARMS}
    write_json(root / 'requests.json', dict(requested_n=2, arms=requested))
    arms, codes = {}, {}
    for arm in ARMS:
        arm_cfg = dict(cfg, arm=arm, out=root / arm)
        try:
            codes[arm] = run_arm(arm_cfg, arm_cfg['out'])
        except BaseException as exc:
            codes[arm] = 1
            arm_cfg['out'].mkdir(parents=True, exist_ok=True)
            write_json(arm_cfg['out'] / 'report.json', dict(status='failed', failure=repr(exc)))
        requested[arm] = dict(status='ok' if codes[arm] == 0 else 'failed', exit_code=codes[arm])
        timeline = arm_cfg['out'] / 'timeline.json'
        arms[arm] = json.loads(timeline.read_text()) if timeline.is_file() else None
        cleanup_file = arm_cfg['out'] / 'cleanup.json'
        cleanup = json.loads(cleanup_file.read_text()) if cleanup_file.is_file() else {}
        write_json(root / 'requests.json', dict(requested_n=2, arms=requested))
        if cleanup.get('remaining_live'):
            for pending in ARMS:
                if pending not in codes:
                    requested[pending] = dict(status='aborted_not_started', failure='previous_owned_cleanup_failed')
                    codes[pending] = 1
            break
    reports = [root / arm / 'report.json' for arm in ARMS]
    controller_hashes = [json.loads(path.read_text()).get('controller_sha256') if path.is_file() else None for path in reports]
    pairing_ok = all(arms.get(arm) and arms[arm]['pairing']['certified'] for arm in ARMS)
    config_equal = all(controller_hashes) and len(set(controller_hashes)) == 1
    manifests = [json.loads((root / arm / 'manifest.json').read_text()) if (root / arm / 'manifest.json').is_file() else None for arm in ARMS]
    sources_equal = all(manifests) and source_hashes(manifests[0]) == source_hashes(manifests[1])
    paired = dict(source_hashes_equal=bool(sources_equal), schema_version=SCHEMA_VERSION, preregistration=preregistration_id(cfg), requested_n=2,
                  requests=requested, exit_codes=codes, comparison=_paired_comparison(arms),
                  pairing_certified=bool(pairing_ok), controller_hashes=controller_hashes,
                  controller_content_equal=bool(config_equal), note=_comparison_note())
    write_json(root / 'requests.json', dict(requested_n=2, arms=requested))
    write_json(root / 'pairing.json', paired)
    return 0 if all(codes.get(arm) == 0 for arm in ARMS) and pairing_ok and config_equal and sources_equal else 1


def _paired_comparison(arms):
    fields = ('laps', 'lap_splits_s', 'cone_hits', 'cones_hit_ids', 'off_track_samples',
              'max_dist_to_nearest_cone_m', 'elapsed_s', 'elapsed_wall_s', 'clock_source')
    out = {}
    for arm in ARMS:
        timeline = arms.get(arm) or {}
        pairing = timeline.get('pairing') or {}
        scorer = timeline.get('scorer_result') or {}
        entry = dict(pairing_certified=pairing.get('certified'),
                     anchor_sim=pairing.get('anchor_sim'),
                     anchor_source=pairing.get('anchor_source'),
                     enable_sim_s=pairing.get('enable_sim_s'),
                     enable_offset_s=pairing.get('enable_offset_s'),
                     enable_offset_deviation_s=pairing.get('deviation_s'),
                     motion_window_sim_s=pairing.get('motion_window_sim_s'),
                     stopped_by=timeline.get('stopped_by'),
                     counters=timeline.get('counters'),
                     supervisor_csv=timeline.get('supervisor_csv'),
                     mechanism=timeline.get('mechanism'),
                     candidate_evidence=timeline.get('candidate_evidence'))
        for field in fields:
            entry[field] = scorer.get(field)
        out[arm] = entry
    return out


# --------------------------------------------------------------------------------------
# preflight + entry point
# --------------------------------------------------------------------------------------

def preflight(cfg):
    checks = [check_venv()]
    if cfg['arm'] in ('candidate', 'pair'):
        checks.append(check_guard_import(cfg))
    try:
        sources = source_targets(cfg) + installed_targets(cfg)
        hashes = {str(path): sha256_file(path) for path in sources}
        launch = installed_targets(cfg)[0]
        text = launch.read_text(encoding='utf-8')
        if not re_controller_path_matches(text):
            raise ConfigError('installed launch controller-path assignment not recognized')
        checks.append(dict(name='required_sources_and_installed_launch', ok=True, hashes=hashes))
    except BaseException as exc:
        checks.append(dict(name='required_sources_and_installed_launch', ok=False, detail=repr(exc)))
    checks.append(sim_source_check(cfg))
    track = Path(cfg['track'])
    checks.append(dict(name='track_files', ok=True,
                       detail={name: sha256_file(track / name) for name in REQUIRED_TRACK_FILES}))
    checks.append(dict(name='npz', ok=Path(cfg['npz']).is_file(), detail=str(cfg['npz'])))
    checks.append(dict(name='policy_entry', ok=Path(cfg['policy_entry']).is_file(),
                       detail=str(cfg['policy_entry'])))
    checks.append(dict(name='probe_domain_overlap', ok=not cfg['probe_domain_overlap'],
                       detail='worker %d resolves to domain %d/port %d (probe reserves %d/%d)'
                              % (cfg['worker'], cfg['domain'], cfg['port'], PROBE_DOMAIN,
                                 PROBE_PORT)))
    try:
        isolation = isolation_scan(cfg['domain'], cfg['port'],
                                   (track / 'track_sim.world').name, cfg['out'])
    except ConflictError as exc:
        checks.append(dict(name='isolation', ok=False, detail=str(exc)))
    else:
        checks.append(dict(name='isolation', ok=True, detail=isolation))
    arms = ARMS if cfg['arm'] == 'pair' else (cfg['arm'],)
    commands = {}
    for arm in arms:
        arm_out = cfg['out'] / arm if cfg['arm'] == 'pair' else cfg['out']
        commands[arm] = arm_commands(arm, cfg, arm_out, arm_out / 'track' / 'track_sim.world')
    payload = dict(preflight=True, config=serializable(cfg), checks=checks, commands=commands,
                   ok=all(check['ok'] for check in checks))
    print(json.dumps(payload, indent=2, default=str), flush=True)
    return 0 if payload['ok'] else 1


def main(argv=None):
    argv = list(sys.argv[1:] if argv is None else argv)
    try:
        cfg = config_from_args(argv)
    except ConfigError as exc:
        print('CONFIG ERROR: %s' % exc, file=sys.stderr, flush=True)
        return 2
    if cfg['preflight_only']:
        return preflight(cfg)
    try:
        if cfg['arm'] == 'pair':
            return run_pair(cfg)
        return run_arm(cfg, cfg['out'])
    except ConflictError as exc:
        print('ISOLATION CONFLICT (nothing was cleaned): %s' % exc, file=sys.stderr, flush=True)
        return 3
    except RunnerInterrupted as exc:
        print('INTERRUPTED: %s' % exc, file=sys.stderr, flush=True)
        return 4
    except BaseException as exc:                                    # noqa: BLE001
        print('ERROR: %s: %s' % (type(exc).__name__, exc), file=sys.stderr, flush=True)
        return 1


if __name__ == '__main__':
    raise SystemExit(main())
