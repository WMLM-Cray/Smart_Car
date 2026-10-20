"""Pure tests, including actual run_arm with fake transport and production Eval."""
import csv
import importlib.util
import json
import math
import os
from pathlib import Path
import sys
import types
from types import SimpleNamespace as NS

import pytest

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import gazebo_supervised_runner as r
import gazebo_braking_probe as probe

# 会话级自造的"队友仓"根目录（见 _build_fake_team）。测试与生产代码一样：不许依赖仓库外的
# 兄弟检出——本机有 D:\study\goboze\Auto_Car-main 才跑得过的测试等于没测。
FAKE_TEAM = None


def _build_fake_team(base):
    """按 validate_config 要的形状造一个队友仓：24 条 dev-suite 赛道目录（每条都带
    `REQUIRED_TRACK_FILES` = `layout.yaml` / `cones.csv` / `track_sim.world`）+ 一个入口文件。
    赛道资产从仓内 evaluation/panel/track_A 拷，入口只放一行注释（假 Popen 不会真执行它）。"""
    team = Path(base) / 'team'
    panel = r.SUP_REPO / 'evaluation' / r.AD_PANEL_DIRNAME / 'track_A'
    suite = team / 'runs' / 'gen_suite_20260926'
    suite.mkdir(parents=True)
    for track in r.DEV_SUITE_TRACKS:
        directory = suite / track
        directory.mkdir()
        for name in ('layout.yaml', 'cones.csv', 'track_sim.world'):
            (directory / name).write_bytes((panel / name).read_bytes())
    (team / 'teleop').mkdir()
    (team / 'teleop' / 'road_policy_node.py').write_text(
        '# fake team entry: this file only has to exist\n', encoding='utf-8')
    return team


def _real_team_repo():
    """集成用例要**真**评分器（队友仓的 `3Dsim/tools/track_eval.py`）。仓库不猜它在哪：没显式给
    `TEAM_REPO`（或给的目录缺件）就跳过，并在跳过理由里写清缺什么 —— 既不静默换成假评分器，
    也不再去看同级的 Auto_Car-main。"""
    raw = os.environ.get('TEAM_REPO')
    if not raw:
        pytest.skip('TEAM_REPO 未设置：本组集成用例要真评分器 3Dsim/tools/track_eval.py')
    team = Path(raw)
    required = (team / '3Dsim' / 'tools' / 'track_eval.py', team / 'fastsim' / 'params.py',
                team / 'runs' / 'gen_suite_20260926' / 'narrow_03' / 'track_sim.world')
    missing = [str(path) for path in required if not path.is_file()]
    if missing:
        pytest.skip('TEAM_REPO 缺件：%s' % ', '.join(missing))
    return team


@pytest.fixture(scope='session', autouse=True)
def _fake_team(tmp_path_factory):
    global FAKE_TEAM
    FAKE_TEAM = _build_fake_team(tmp_path_factory.mktemp('fake_team'))


@pytest.fixture(autouse=True)
def _explicit_venv(monkeypatch):
    """生产代码不再带机器路径默认值：测试显式给一个解释器路径，
    让 config_from_args 的"必填"校验在测试里也走真实分支。"""
    if not r.VENV_ROOT or not r.VENV_PYTHON:
        monkeypatch.setattr(r, 'VENV_ROOT', '/opt/test-venv')
        monkeypatch.setattr(r, 'VENV_PYTHON', '/opt/test-venv/bin/python')


def cfg(**updates):
    result = r.config_from_args(['--team-repo', str(FAKE_TEAM)])
    result.update(updates)
    return result


def graph(arm):
    expected = r.graph_expectation(arm)
    return dict(error=None, topics={topic: [dict(node_name=node, node_namespace='/', endpoint_gid=node) for node in value['nodes']]
                                    for topic, value in expected.items()},
                nodes=[('sim_wiring', '/'), ('ackermann_controller', '/')],
                subscribers={r.CMD_TOPIC: [dict(node_name='sim_wiring', node_namespace='/')],
                             r.ENABLE_TOPIC: [dict(node_name=r.POLICY_NODE, node_namespace='/')],
                             r.RAW_TOPIC: [dict(node_name=r.SUPERVISOR_NODE, node_namespace='/')] if arm == 'candidate' else []})


def test_frozen_defaults_and_no_shared_fallback():
    c = cfg()
    assert (c['worker'], c['domain'], c['port']) == (21, 64, 11366)
    assert (c['cap'], c['guard'], c['seconds'], c['warmup']) == (3, .25, 140, 5)
    assert c['npz'].as_posix().endswith('spica_v3geo_nola/candidate_036.npz')
    for args in (['--worker', '19'], ['--seconds', '139'], ['--warmup-sim-s', '6'],
                 ['--cap', '4'], ['--guard', '.2'], ['--domain', '63'], ['--port', '11365']):
        with pytest.raises(r.ConfigError):
            r.config_from_args(['--team-repo', str(FAKE_TEAM)] + args)
    env = r.child_env(c)
    assert env['ROS_DOMAIN_ID'] == '64' and env['GAZEBO_MASTER_URI'] == 'http://127.0.0.1:11366'
    assert all(env[key] == '1' for key in ('OMP_NUM_THREADS', 'OPENBLAS_NUM_THREADS', 'MKL_NUM_THREADS'))
    assert env['PYTHONDONTWRITEBYTECODE'] == '1'


def test_team_repo_and_venv_have_no_machine_default(monkeypatch):
    """队友仓路径与解释器都必须显式给出；仓库既不猜同级目录，也不带 /home/... 默认值。"""
    monkeypatch.setattr(r, 'VENV_ROOT', None)
    monkeypatch.setattr(r, 'VENV_PYTHON', None)
    with pytest.raises(r.ConfigError) as venv:
        r.config_from_args(['--team-repo', str(FAKE_TEAM)])
    assert 'VENV_ROOT' in str(venv.value)
    monkeypatch.setattr(r, 'VENV_ROOT', '/opt/test-venv')
    monkeypatch.setattr(r, 'VENV_PYTHON', '/opt/test-venv/bin/python')
    monkeypatch.delenv('TEAM_REPO', raising=False)
    with pytest.raises(r.ConfigError) as team:
        r.config_from_args([])
    assert '--team-repo' in str(team.value)


def test_arm_entry_requires_corridor_arm_env(monkeypatch, tmp_path):
    """臂身份只由 CORRIDOR_ARM 决定（`arms/corridor_arm.py:60` 不设就是 'D'）：缺了要报错，
    不能静默跑成 D 臂还叫"Y 的读数"。"""
    entry = tmp_path / 'corridor_arm.py'
    entry.write_text('# fake arm entry\n', encoding='utf-8')
    monkeypatch.delenv('CORRIDOR_ARM', raising=False)
    with pytest.raises(r.ConfigError) as missing:
        r.config_from_args(['--team-repo', str(FAKE_TEAM), '--policy-entry', str(entry)])
    assert 'CORRIDOR_ARM' in str(missing.value)
    monkeypatch.setenv('CORRIDOR_ARM', 'Y')
    c = r.config_from_args(['--team-repo', str(FAKE_TEAM), '--policy-entry', str(entry)])
    assert c['policy_entry'] == entry.resolve()


def test_foreign_sim_refused_unless_sim_setup_points_there(tmp_path):
    """本仓 sim 没 build 成功时（colcon 失败也留下 stub install/setup.bash），会静默跑队友旧几何
    racecar_sim：整局照样认证但读数不可比。要跑旧几何必须显式 SIM_SETUP 指过去。"""
    repo = tmp_path / 'port-dreamfish'
    foreign = tmp_path / 'Auto_Car-main' / '3Dsim' / 'hwcart_sim' / 'install' / 'racecar_sim' / 'share' / 'racecar_sim' / 'launch' / 'sim_bringup.launch.py'
    ours = repo / 'simulation' / 'gazebo' / 'install' / 'racecar_sim' / 'share' / 'racecar_sim' / 'launch' / 'sim_bringup.launch.py'
    with pytest.raises(r.ConfigError) as silent:
        r.check_sim_source(foreign, repo, None)
    assert 'old-geometry' in str(silent.value) and 'colcon build' in str(silent.value)
    # 显式指到队友那份 => 知情选择，放行并留下痕迹
    ok = r.check_sim_source(foreign, repo, str(tmp_path / 'Auto_Car-main' / '3Dsim' / 'hwcart_sim' / 'install' / 'setup.bash'))
    assert ok['sim_in_repo'] is False and 'Auto_Car-main' in ok['sim_source']
    # 本仓 install 里那份 => 正常
    assert r.check_sim_source(ours, repo, str(repo / 'simulation' / 'gazebo' / 'install' / 'setup.bash'))['sim_in_repo'] is True
    with pytest.raises(r.ConfigError):
        r.check_sim_source(foreign, repo, str(repo / 'simulation' / 'gazebo' / 'install' / 'setup.bash'))


def test_pair_commands_only_remap_and_supervisor_differ(tmp_path):
    c = cfg()
    b = r.arm_commands('baseline', c, tmp_path / 'baseline', tmp_path / 'baseline/world.world')
    a = r.arm_commands('candidate', c, tmp_path / 'candidate', tmp_path / 'candidate/world.world')
    assert a['policy']['argv'][:-3] == b['policy']['argv']
    assert a['policy']['argv'] == b['policy']['argv'] + ['--ros-args', '-r', '/cmd_vel:=/cmd_vel_raw']
    assert b['supervisor'] is None and a['supervisor']['argv'][0] == r.VENV_PYTHON
    assert a['hold_enable']['argv'] is None
    assert all(command['argv'][0] == r.VENV_PYTHON for command in a.values() if command and command['argv'])


def test_graph_identity_not_counts():
    for arm in ('baseline', 'candidate'):
        g = graph(arm)
        assert r.evaluate_graph_poll(g, r.graph_expectation(arm))['ok']
        g['topics'][r.CMD_TOPIC][0]['node_name'] = 'stranger'
        assert not r.evaluate_graph_poll(g, r.graph_expectation(arm))['ok']
    g = graph('candidate')
    g['topics'][r.RAW_TOPIC].append(dict(node_name=r.POLICY_NODE, node_namespace='/other'))
    assert not r.evaluate_graph_poll(g, r.graph_expectation('candidate'))['ok']
    g = graph('candidate')
    g['subscribers'][r.CMD_TOPIC] = []
    assert not r.evaluate_graph_poll(g, r.graph_expectation('candidate'))['ok']


def test_preflight_reports_conflicts_without_signaling(monkeypatch):
    monkeypatch.setattr(os, 'kill', lambda *_: pytest.fail('preflight must not signal'))
    table = {7: dict(ppid=1, start='10', state='S', env=[b'ROS_DOMAIN_ID=64'], args='teammate')}
    with pytest.raises(r.ConflictError, match='ros_domain_in_use'):
        r.isolation_scan(64, 11366, 'track_sim.world', '/owned/runs', table=table, port_probe=lambda _: True)


def test_preflight_unattributed_bound_socket_fails():
    with pytest.raises(r.ConflictError, match='"port_free": false'):
        r.isolation_scan(64, 11366, 'track_sim.world', '/owned/runs', table={}, port_probe=lambda _: False)


def test_owned_cleanup_never_signals_reused_pid(monkeypatch):
    tables = {10: dict(ppid=1, state='S', start='100'), 11: dict(ppid=10, state='S', start='101')}
    monkeypatch.setattr(probe, 'proc_table', lambda: dict(tables))
    monkeypatch.setattr(probe.signal, 'SIGKILL', 9, raising=False)
    monkeypatch.setattr(probe.signal, 'SIGKILL', 9, raising=False)
    signals = []
    def kill(pid, sig):
        signals.append(pid)
        tables[pid]['state'] = 'Z'
    monkeypatch.setattr(os, 'kill', kill)
    owned = r.OwnedSet()
    owned.add(10, 'sim')
    tables[11].update(ppid=1)  # reparented owned child
    tables[10]['start'] = '200'  # PID reused by unrelated process
    result = owned.cleanup()
    assert signals == [11]
    assert result['remaining_live'] == []
    assert result['owned_pid_starttimes'] == {10: dict(label='sim', start='100'), 11: dict(label='sim', start='101')}


def test_raw_nonfinite_scan_roundtrip(tmp_path):
    scan = NS(header=NS(stamp=NS(sec=2, nanosec=4), frame_id='laser'), angle_min=-1., angle_max=1.,
              angle_increment=.5, time_increment=.01, scan_time=.2, range_min=.05, range_max=10.,
              ranges=[.5, float('nan'), float('inf'), -float('inf')], intensities=[float('nan'), 3])
    meta = r.serializable(dict(ranges=list(scan.ranges), intensities=list(scan.intensities)))
    r.append_jsonl(tmp_path / 'scan.jsonl', meta)
    row = json.loads((tmp_path / 'scan.jsonl').read_text())
    assert row['ranges'] == [.5, 'NaN', '+Infinity', '-Infinity']
    decoded = [float(value) for value in row['ranges']]
    assert math.isnan(decoded[1]) and decoded[2] == math.inf and decoded[3] == -math.inf
    assert row['intensities'] == ['NaN', 3.]


@pytest.fixture
def fake_ros(monkeypatch, tmp_path):
    # Transport is fake; callback ordering, startup, production scoring and report are actual.
    class Twist:
        def __init__(self):
            self.linear, self.angular = NS(x=0., y=0., z=0.), NS(x=0., y=0., z=0.)
    class Bool:
        def __init__(self):
            self.data = False
    class Node:
        def __init__(self, name, **kwargs):
            self.name = name
            self.callbacks = {}
        def create_subscription(self, message, topic, callback, qos):
            self.callbacks[topic] = callback
        def get_name(self):
            return self.name
        def create_timer(self, interval, callback):
            state['timer'] = callback
            return NS(cancel=lambda: state.update(timer=None))
        def create_publisher(self, message, topic, qos):
            def publish(msg):
                state['published'].append(msg.data)
                state['enabled'] = msg.data
            return NS(publish=publish)
        def get_logger(self):
            return NS(warn=lambda *_: None, info=lambda *_: None)
        def get_node_names_and_namespaces(self):
            return [('sim_wiring', '/'), ('ackermann_controller', '/')]
        def get_publishers_info_by_topic(self, topic):
            names = r.graph_expectation(state['arm']).get(topic, {}).get('nodes', ())
            if state.get('bad_graph') and topic == r.CMD_TOPIC:
                names = ('stranger',)
            return [NS(node_name=name, node_namespace='/', endpoint_gid=b'ab', topic_type='geometry_msgs/msg/Twist') for name in names]
        def get_subscriptions_info_by_topic(self, topic):
            names = {r.CMD_TOPIC: ['sim_wiring'], r.ENABLE_TOPIC: [r.POLICY_NODE],
                     r.RAW_TOPIC: [r.SUPERVISOR_NODE] if state['arm'] == 'candidate' else []}.get(topic, [])
            return [NS(node_name=name, node_namespace='/', endpoint_gid=b'cd') for name in names]
        def destroy_node(self):
            pass
    state = dict(arm='baseline', enabled=False, sim=0., spins=0, published=[], popen=[], cleaned=[], executor_nodes=[], clock_history=[])
    clock_wall = [0.]
    monkeypatch.setattr(r.time, 'monotonic', lambda: clock_wall[0])
    monkeypatch.setattr(r.time, 'sleep', lambda duration: clock_wall.__setitem__(0, clock_wall[0] + duration))
    class Executor:
        def add_node(self, node):
            state['executor_nodes'].append(node)
        def remove_node(self, node):
            state['executor_nodes'].remove(node)
        def shutdown(self, timeout_sec=0):
            pass
        def spin_once(self, timeout_sec=0):
            state['spins'] += 1
            if state.get('interrupt_signal') and state['spins'] == 2:
                r.signal.getsignal(r.signal.SIGINT)(r.signal.SIGINT, None)
            clock_wall[0] += .2
            skip_clock = bool(state.get('enabled') and state.get('clock_silence_wall') and not state.get('silence_injected'))
            if skip_clock:
                clock_wall[0] += state['clock_silence_wall'] - .2
                state['silence_injected'] = True
            if state.get('timer'):
                state['timer']()
            if state.get('dead'):
                return
            step = .1
            state['sim'] += step
            sim = state['sim']
            clock = NS(clock=NS(sec=int(sim), nanosec=round((sim - int(sim)) * 1e9)))
            state['clock_history'].append(clock)
            x = .05 if state['enabled'] else sim * .5 if state.get('early_motion') else 0.
            pose = NS(position=NS(x=x, y=0., z=0.), orientation=NS(x=0., y=0., z=0., w=1.))
            world_twist = Twist()
            world_twist.linear.x = .5 if state.get('early_motion') or state['enabled'] else 0.
            world = NS(name=['racecar'], pose=[pose], twist=[world_twist])
            odom = NS(header=NS(stamp=NS(sec=int(sim), nanosec=0)),
                      pose=NS(pose=pose), twist=NS(twist=Twist()))
            scan = NS(header=NS(stamp=NS(sec=int(sim), nanosec=0), frame_id='laser'),
                      angle_min=-1., angle_max=1., angle_increment=.5, time_increment=0., scan_time=.1,
                      range_min=.05, range_max=10., ranges=[5., math.inf, 5., 5.], intensities=[])
            lag = int(state.get('scorer_lag') or 0)
            frozen = state.get('scorer_clock_off_at')
            # Modelled reality: heartbeat(False) is published just before the one spin in the
            # finally block, and a single spin usually delivers some other callback, so a
            # stopped arm does not advance the instrument's /clock any further.
            stopped = False in state['published']
            for index, node in enumerate(list(state['executor_nodes'])):
                # Index 0 is the runner node. In the real runner the instrument is a separate
                # /clock subscription and spin_once runs one callback at a time, so the runner
                # can observe the window close one 0.1 s sample before the instrument does.
                runner = index == 0
                stalled = not runner and (stopped or (frozen is not None and state['spins'] > frozen))
                node_clock = clock if runner or lag < 1 or len(state['clock_history']) <= lag \
                    else state['clock_history'][-1 - lag]
                for topic, msg in ((r.CLOCK_TOPIC, node_clock), (r.WORLD_TOPIC, world), (r.ODOM_TOPIC, odom), (r.SCAN_TOPIC, scan)):
                    if topic not in node.callbacks or (skip_clock and topic == r.CLOCK_TOPIC):
                        continue
                    if topic == r.CLOCK_TOPIC and stalled:
                        continue
                    node.callbacks[topic](msg)
            if state['enabled']:
                command = Twist()
                command.linear.x = 1.
                node = state['executor_nodes'][0]
                if state['arm'] == 'candidate':
                    node.callbacks[r.RAW_TOPIC](command)
                    out = state['out'] / 'supervisor.csv'
                    if not out.exists():
                        with out.open('w', newline='') as stream:
                            writer = csv.DictWriter(stream, fieldnames=['seq', 'wall', 'k', 'gap', 'passthrough_reason'])
                            writer.writeheader()
                    with out.open('a', newline='') as stream:
                        writer = csv.DictWriter(stream, fieldnames=['seq', 'wall', 'k', 'gap', 'passthrough_reason'])
                        writer.writerow(dict(seq=state['spins'], wall=clock_wall[0], k=.5, gap=.1, passthrough_reason='guard_unavailable' if state.get('guard_fail') else ''))
                node.callbacks[r.CMD_TOPIC](command)
    modules = {
        'rclpy': dict(init=lambda args=None: None, shutdown=lambda: None, ok=lambda: True),
        'rclpy.node': dict(Node=Node), 'rclpy.executors': dict(SingleThreadedExecutor=Executor),
        'rclpy.parameter': dict(Parameter=lambda name, value=None: NS(name=name, value=value)),
        'rclpy.qos': dict(QoSProfile=lambda **kwargs: NS(**kwargs), ReliabilityPolicy=NS(BEST_EFFORT=1),
                          DurabilityPolicy=NS(VOLATILE=1), HistoryPolicy=NS(KEEP_LAST=1)),
        'geometry_msgs.msg': dict(Twist=Twist), 'std_msgs.msg': dict(Bool=Bool),
        'sensor_msgs.msg': dict(LaserScan=object), 'nav_msgs.msg': dict(Odometry=object),
        'gazebo_msgs.msg': dict(ModelStates=object), 'rosgraph_msgs.msg': dict(Clock=object),
    }
    for name, values in modules.items():
        module = types.ModuleType(name)
        module.__dict__.update(values)
        monkeypatch.setitem(sys.modules, name, module)
    def fake_popen(argv, env, cwd, log_path, cmds_path, owned, label):
        label = 'sim' if 'ros2' in ' '.join(map(str, argv)) else 'supervisor' if 'gazebo_speed_supervisor.py' in str(argv) else 'policy'
        state['popen'].append(label)
        Path(log_path).write_text('SUPERVISOR_GUARD_LOAD ok=1\n' if label == 'supervisor' else '')
        if label == 'sim':
            (Path(cwd) / 'generated_controller.yaml').write_text('controller same contents')
        assert argv[0] == r.VENV_PYTHON
        return NS(wait=lambda timeout=None: 0, returncode=1 if state.get('child_exit') and label == 'policy' else None, pid=100 + len(state['popen']), poll=lambda: 1 if state.get('child_exit') and label == 'policy' else None), open(log_path, 'a')
    monkeypatch.setattr(r, '_popen', fake_popen)
    monkeypatch.setattr(r.OwnedSet, 'capture', lambda self: None)
    def fake_cleanup(self, order=None):
        state['cleaned'].append(list(state['popen']))
        return dict(remaining_live=({'111': 1} if state.get('unclean') else {}))
    monkeypatch.setattr(r.OwnedSet, 'cleanup', fake_cleanup)
    class Guard:
        def update(self, *_):
            pass
        def gap(self, steer):
            return .1
    module = r.load_supervisor_module(cfg())
    monkeypatch.setattr(module, 'load_scan_guard', lambda *_: Guard)
    monkeypatch.setattr(r, 'load_supervisor_module', lambda _: module)
    monkeypatch.setattr(r, 'check_venv', lambda: dict(name='specified_venv', ok=True))
    monkeypatch.setattr(r, 'check_guard_import', lambda _: dict(name='guard_import', ok=True))
    monkeypatch.setattr(r, 'isolation_scan', lambda *args, **kwargs: dict(ok=True))
    dev = _real_team_repo() / 'runs' / 'gen_suite_20260926' / 'narrow_03'
    layout = dev / 'layout.yaml'
    world = dev / 'track_sim.world'
    cones = dev / 'cones.csv'
    c = cfg(team_repo=dev.parents[2], repo=r.SUP_REPO, layout=layout, world=world, cones=cones)
    launch = tmp_path / 'installed_sim_bringup.launch.py'
    launch.write_text((r.SUP_REPO / 'simulation' / 'gazebo' / 'src' / 'racecar_sim' / 'launch'
                       / 'sim_bringup.launch.py').read_text(encoding='utf-8'), encoding='utf-8')
    monkeypatch.setattr(r, 'installed_targets', lambda _: [launch])
    monkeypatch.setattr(r, 'prepare_launch', lambda c, out: r.prepare_launch_text(launch.read_text(encoding='utf-8'), launch, out))
    # Keep production Eval creation/load_scorer, with actual CarParams and real narrow_03 CSV.
    def run(arm='baseline', **options):
        state.update(options, arm=arm, out=tmp_path / arm)
        rc = r.run_arm(dict(c, arm=arm, out=state['out']), state['out'])
        result = json.loads((state['out'] / 'report.json').read_text())
        result['rc'] = rc
        result['timing'] = json.loads((state['out'] / 'timeline.json').read_text())
        result['cleanup'] = json.loads((state['out'] / 'cleanup.json').read_text())
        return result
    return run, state


@pytest.mark.parametrize('arm', ['baseline', 'candidate'])
def test_actual_run_arm_starts_before_graph_anchors_real_eval_and_enable(fake_ros, arm):
    run, state = fake_ros
    result = run(arm)
    assert result['failure'] is None, result['failure']
    assert state['popen'] == (['sim', 'supervisor', 'policy'] if arm == 'candidate' else ['sim', 'policy'])
    assert ('supervisor' in state['popen']) == (arm == 'candidate')
    timing = result['timing']
    assert timing['score_anchor'] == pytest.approx(.2)  # production Eval callback, not runner clock startup .1
    assert timing['enable_sim'] == pytest.approx(timing['score_anchor'] + 5, abs=.100001)
    assert timing['first_nonzero_cmd'] >= timing['enable_sim']
    assert timing['first_world_motion']['sim_s'] >= timing['enable_sim']
    assert result['scorer_result']['elapsed_s'] >= 140
    assert result['controller_sha256'] is not None
    assert result['cleanup']['remaining_live'] == {}
    assert state['published'][0] is True and state['published'][-1] is False
    events = [json.loads(line) for line in (state['out'] / 'scan_frames.jsonl').read_text().splitlines()]
    assert events[0]['ranges'][1] == '+Infinity'
    if arm == 'candidate':
        assert result['timing']['raw_out_count_match']
        assert result['evidence']['ok'] and result['timing']['supervisor_csv']['scaled_rows'] > 0


@pytest.mark.parametrize('option,reason', [('early_motion', 'world_motion_before_enable'),
                                          ('child_exit', 'policy_exited_1'), ('dead', 'ready_timeout'),
                                          ('unclean', 'owned_process_cleanup_failed'), ('bad_graph', 'ready_timeout')])
def test_actual_run_arm_fail_closed_and_serializes_report(fake_ros, option, reason):
    run, state = fake_ros
    result = run('baseline', **{option: True})
    assert reason in result['failure']
    assert result['requested_n'] == 1
    assert json.loads((state['out'] / 'report.json').read_text())['failure'] == result['failure']
    if option != 'unclean':
        assert True not in state['published']


def test_candidate_guard_failure_invalid_not_passthrough_valid(fake_ros):
    run, _ = fake_ros
    result = run('candidate', guard_fail=True)
    assert 'guard_unavailable_rows' in result['failure']


def test_signal_interrupt_creates_failed_report(fake_ros):
    run, state = fake_ros
    result = run('baseline', interrupt_signal=True)
    assert 'signal' in result['failure']
    assert result['cleanup']['remaining_live'] == {}
    assert state['cleaned'] == [['sim', 'policy']]
    assert True not in state['published']
    assert (state['out'] / 'report.json').exists()


def test_pair_keeps_failed_requests_and_no_retry(tmp_path, monkeypatch):
    calls = []
    def failed(c, out):
        calls.append(c['arm'])
        Path(out).mkdir()
        r.write_json(Path(out) / 'report.json', dict(failure='test failure'))
        return 1
    monkeypatch.setattr(r, 'run_arm', failed)
    c = cfg(out=tmp_path / 'pair')
    assert r.run_pair(c) == 1 and calls == ['baseline', 'candidate']
    report = json.loads((tmp_path / 'pair/pairing.json').read_text())
    assert report['requested_n'] == 2
    assert all(row['status'] == 'failed' for row in report['requests'].values())
    with pytest.raises(FileExistsError):
        r.run_pair(c)


def test_preflight_failure_counts_both_requests_without_spawn(tmp_path, monkeypatch):
    monkeypatch.setattr(r, '_popen', lambda *_args, **_kw: pytest.fail('spawn on conflict'))
    monkeypatch.setattr(r, 'isolation_scan', lambda *_args: (_ for _ in ()).throw(r.ConflictError('domain busy')))
    c = cfg(out=tmp_path / 'pair')
    assert r.run_pair(c) == 1
    report = json.loads((tmp_path / 'pair/pairing.json').read_text())
    assert report['requested_n'] == 2
    assert all(row['status'] == 'failed' for row in report['requests'].values())
    assert all(json.loads((tmp_path / 'pair' / arm / 'report.json').read_text())['failure'] for arm in r.ARMS)


def test_required_source_missing_invalidates_snapshot(tmp_path, monkeypatch):
    missing = tmp_path / 'missing_required.py'
    monkeypatch.setattr(r, 'source_targets', lambda _: [missing])
    monkeypatch.setattr(r, 'installed_targets', lambda _: [])
    with pytest.raises(r.ConfigError, match='required snapshot sources missing'):
        r.snapshot_sources(tmp_path, cfg(arm='baseline'))
    manifest = json.loads((tmp_path / 'manifest.json').read_text())
    assert manifest['artifacts'][0]['sha256'] is None
    assert manifest['artifacts'][0]['error']


def test_source_mutation_after_snapshot_invalidates_stability(tmp_path, monkeypatch):
    source = tmp_path / 'required.py'
    source.write_text('frozen=1')
    monkeypatch.setattr(r, 'source_targets', lambda _: [source])
    monkeypatch.setattr(r, 'installed_targets', lambda _: [])
    manifest = r.snapshot_sources(tmp_path, cfg(arm='baseline'))
    assert r.unchanged_sources(manifest)['ok']
    source.write_text('frozen=2')
    verdict = r.unchanged_sources(manifest)
    assert verdict == dict(ok=False, changed=[str(source)])
    assert (tmp_path / 'sources/00_required.py').read_text() == 'frozen=1'


@pytest.mark.parametrize('mismatch', [False, True])
def test_pair_core_hash_mismatch_rejects_otherwise_valid_pair(tmp_path, monkeypatch, mismatch):
    def complete(c, out):
        Path(out).mkdir()
        r.write_json(Path(out) / 'report.json', dict(status='ok', failure=None, controller_sha256='same'))
        r.write_json(Path(out) / 'timeline.json', dict(pairing=dict(certified=True)))
        digest = 'changed' if mismatch and c['arm'] == 'candidate' else 'frozen'
        r.write_json(Path(out) / 'manifest.json', dict(artifacts=[dict(path='/actual/production', sha256=digest)], npz=dict(path='/actual/r36', sha256=r.R36_SHA256)))
        return 0
    monkeypatch.setattr(r, 'run_arm', complete)
    root = tmp_path / 'pair'
    assert r.run_pair(cfg(out=root)) == (1 if mismatch else 0)
    paired = json.loads((root / 'pairing.json').read_text())
    assert paired['requested_n'] == 2 and paired['controller_content_equal']
    assert paired['source_hashes_equal'] is (not mismatch)


def test_v2_readiness_budget_boundary():
    assert r.PREREGISTRATION == 'supervised_smoke_v4'
    assert r.READY_DEADLINE_WALL_S == 180.0
    protocol = r.StartupProtocol('baseline', 5, 140)
    state = dict(now_wall=0., ready=False, ready_reasons=['not ready'])
    assert protocol.next_action(state) == 'wait_ready'
    assert protocol.next_action(dict(state, now_wall=180.)) == 'wait_ready'
    assert protocol.next_action(dict(state, now_wall=180.001)).startswith('abort:ready_timeout')


@pytest.mark.parametrize('silence,accepted', [(9.999, True), (10.001, False)])
def test_v2_clock_silence_boundary_actual_executor(fake_ros, silence, accepted):
    assert r.CLOCK_STALE_WALL_S == 10.0
    run, state = fake_ros
    result = run('baseline', clock_silence_wall=silence)
    assert state['silence_injected']
    if accepted:
        assert result['failure'] is None
        assert result['scorer_result']['clock_source'] == 'sim'
        assert result['scorer_result']['elapsed_s'] >= 140
    else:
        assert 'abort:clock_stale' in result['failure']
        assert result['requested_n'] == 1
        assert result['cleanup']['remaining_live'] == {}


def test_v3_window_closes_on_the_instrument_clock(fake_ros):
    """PAIR02's exact failure: the runner's own /clock led the instrument's by one sample."""
    assert r.SCORER_CLOSE_GRACE_WALL_S == 10.0
    run, state = fake_ros
    result = run('baseline', scorer_lag=1)
    assert result['failure'] is None, result['failure']
    assert result['scorer_result']['clock_source'] == 'sim'
    assert result['scorer_result']['elapsed_s'] >= 140
    assert result['timing']['stopped_by'] == 'scorer_sim_budget'
    assert result['cleanup']['remaining_live'] == {}


def test_v3_short_window_still_fails_closed(fake_ros):
    """The grace only delays the stop; it must never relax the window check."""
    run, state = fake_ros
    result = run('baseline', scorer_clock_off_at=60)
    assert 'incomplete_sim_window' in result['failure']
    assert result['requested_n'] == 1
    assert result['cleanup']['remaining_live'] == {}


def test_v3_cap_ladder_gate_and_dev_suite_tracks():
    # v4, not the v3 values this test was first written against: the ladder was extended to 2.8
    # and re-labelled (cap_ladder_v4) on 2026-10-05, and this assertion had been stale since.
    assert r.PREREGISTRATION_CAP_LADDER == 'cap_ladder_v4'
    assert r.FROZEN_CAP == 3.0
    assert r.CAP_LADDER_RUNGS == (1.2, 1.35, 1.5, 1.6, 1.65, 1.7, 1.75, 1.8,
                                  1.9, 2.0, 2.2, 2.4, 2.6, 2.8)
    assert len(r.DEV_SUITE_TRACKS) == 24 and r.DEV_SUITE_TRACKS[4] == 'narrow_01'
    assert r.preregistration_id(cfg()) == 'supervised_smoke_v4'
    team = str(FAKE_TEAM)
    suite = FAKE_TEAM / 'runs' / 'gen_suite_20260926'
    accepted = r.config_from_args(['--team-repo', team, '--arm', 'baseline', '--cap', '1.6'])
    assert accepted['cap'] == 1.6
    assert r.preregistration_id(accepted) == 'cap_ladder_v4'
    low = r.config_from_args(['--team-repo', team, '--arm', 'baseline', '--cap', '1.2'])
    assert low['cap'] == 1.2
    assert r.preregistration_id(low) == 'cap_ladder_v4'
    for argv in (['--arm', 'baseline', '--cap', '4'],
                 ['--arm', 'baseline', '--cap', '1.55'],
                 ['--arm', 'candidate', '--cap', '1.6'],
                 ['--arm', 'pair', '--cap', '1.8'],
                 ['--arm', 'baseline', '--cap', '1.6', '--guard', '.2'],
                 ['--arm', 'baseline', '--cap', '1.6', '--seconds', '139'],
                 ['--arm', 'baseline', '--cap', '1.6', '--warmup-sim-s', '6']):
        with pytest.raises(r.ConfigError):
            r.config_from_args(['--team-repo', team] + argv)
    for track in (suite / 'arc_01', suite / 's_bend_04'):
        resolved = r.config_from_args(['--team-repo', team, '--arm', 'baseline', '--cap', '1.6',
                                       '--track', str(track)])
        assert resolved['track'] == track
    for track in (suite / 'arc_05', FAKE_TEAM / 'runs' / '_gzsuite_v3geo'):
        with pytest.raises(r.ConfigError):
            r.config_from_args(['--team-repo', team, '--arm', 'baseline', '--cap', '1.6',
                                '--track', str(track)])


def test_ad_panel_authorises_official_tracks_only():
    """A-D are an authorised track set of their own (the deployment panel), resolved inside the
    supervisor repo; every other name, and any other parent, stays rejected."""
    panel = r.SUP_REPO / 'evaluation' / r.AD_PANEL_DIRNAME
    assert r.AD_PANEL_TRACKS == ('track_A', 'track_B', 'track_C', 'track_D')
    assert r.PREREGISTRATION_AD_PANEL == 'ad_panel_deploy_20261007'
    team = str(FAKE_TEAM)
    for name in r.AD_PANEL_TRACKS:
        resolved = r.config_from_args(['--team-repo', team, '--arm', 'baseline', '--track', str(panel / name)])
        assert resolved['track'] == panel / name
        assert r.preregistration_id(resolved) == r.PREREGISTRATION_AD_PANEL
    for track in (panel / 'track_E',
                  panel / 'track_a',
                  panel.parent / 'gen_suite_20260926' / 'narrow_09'):
        with pytest.raises(r.ConfigError):
            r.config_from_args(['--team-repo', team, '--arm', 'baseline', '--track', str(track)])


def test_v4_pairing_anchor_tolerance_matches_two_clock_steps():
    """The cross-node anchor skew is bounded by two /clock steps, not one; the enable
    deviation stays scorer-internal and keeps the one-step bound."""
    assert r.PAIRING_TOLERANCE_S == 0.100001
    assert r.PAIRING_ANCHOR_TOLERANCE_S == 0.200001
    anchor, enable, warmup = 27.4, 32.3, 5.0
    # Rung-1 cap 1.5: the observer saw the racecar 0.2 s before the instrument's own t0.
    for skew in (0.0, 0.1, 0.2):
        verdict = r.pairing_verdict(anchor, 'track_eval.Eval.t0', 'sim', enable, warmup,
                                    runner_first_racecar_sim=anchor - skew)
        assert verdict['certified'], verdict['reasons']
    late = r.pairing_verdict(anchor, 'track_eval.Eval.t0', 'sim', enable, warmup,
                             runner_first_racecar_sim=anchor - 0.200002)
    assert late['certified'] is False
    assert late['reasons'] == ['anchor_vs_runner_first_racecar_s=0.200>0.200']
    # The scorer-internal enable deviation must NOT inherit the wider bound.
    slipped = r.pairing_verdict(anchor, 'track_eval.Eval.t0', 'sim', anchor + 5.2, warmup,
                                runner_first_racecar_sim=anchor)
    assert slipped['certified'] is False
    assert slipped['reasons'] == ['enable_offset_deviation_s=0.200>0.100']


