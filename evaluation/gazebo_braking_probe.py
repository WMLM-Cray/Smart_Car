"""Owned, opt-in Gazebo braking measurement. Offline imports do not require ROS."""
import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import signal
import socket
import statistics
import subprocess
import time
import xml.etree.ElementTree as ET

# 解释器只从环境读，**不给机器路径默认值**；没设置就在 main() 里报错。
VENV_ROOT = os.environ.get('VENV_ROOT')
PYTHON = os.environ.get('VENV_PYTHON') or (
    os.path.join(VENV_ROOT, 'bin', 'python') if VENV_ROOT else None)
# 只给 VENV_PYTHON 时同样能跑：按 <root>/bin/python 反推 VENV_ROOT（下面要用它做
# "当前解释器就是这个 venv 的 python" 的校验）。只给 VENV_ROOT 的旧用法不变。
if PYTHON and not VENV_ROOT:
    VENV_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(PYTHON)))
DOMAIN, PORT = 63, 11365
CASES = [(1.0, 2), (2.0, 2), (3.0, 1), (3.0, 2), (3.0, 3)]
CMD = '/cmd_vel'
REF = '/ackermann_steering_controller/reference_unstamped'
ODOM = '/ackermann_steering_controller/odometry'
WORLD = '/model_states'
CLOCK = '/clock'
SCHEMA_VERSION = 1
PREREGISTRATION = 'v4'
DEFAULT_OUT = 'runs/gazebo_braking_probe_20261004_v4'
# Publishing stays the deployment-equivalent wall 20Hz node timer, so simulated command
# timestamps land on the /clock sampling grid: a 0.0s/0.1s pattern with mean ~0.05s but a
# max gap equal to the grid. The cadence gate therefore bounds the mean by 20Hz spacing and
# the maximum by this case's own measured /clock sampling interval plus slack.
CADENCE_MEAN_LIMIT_S = .075
CADENCE_MAX_CLOCK_SLACK_S = .02
# A host wall stall freezes simulation time as well, and this host is measurably bursty: v3's
# target3.0_rep1 recorded /clock silences of 4518.40ms and 2005.44ms while the simulation kept
# advancing (v2 host stalls 0.628/1.336/1.236/1.321s). Silence alone therefore proves nothing;
# only CLOCK_SILENCE_LIMIT_S with no /clock receipt at all is treated as a dead simulation.
CLOCK_SILENCE_LIMIT_S = 10.
CONSECUTIVE_INFRASTRUCTURE_LIMIT = 3


def write_json(path, value):
    path = Path(path)
    tmp = path.with_suffix(path.suffix + '.tmp')
    tmp.write_text(json.dumps(value, indent=2, allow_nan=False) + '\n')
    tmp.replace(path)


def owned_empty_world(source, destination):
    """Remove all model obstacles from the owned copy, preserve original plugins."""
    root = ET.parse(source)
    world = root.find('world')
    if world is None or not world.findall("plugin[@filename='libgazebo_ros_state.so']"):
        raise ValueError('world must supply original gazebo_ros_state plugin')
    removed = [x.attrib.get('name') for x in world.findall('model')]
    if 'marker_box' not in removed:
        raise ValueError('expected original marker_box for source identity')
    for model in list(world.findall('model')):
        world.remove(model)
    for include in world.findall('include'):
        if include.findtext('uri') not in ('model://ground_plane', 'model://sun'):
            raise ValueError('unexpected nonempty world include')
    world.set('name', 'braking_probe_owned_empty')
    destination = Path(destination)
    if destination.exists():
        raise FileExistsError(destination)
    root.write(destination, encoding='utf-8', xml_declaration=True)
    return removed


class Recorder:
    def __init__(self, path):
        self.file = Path(path).open('x', buffering=1)
        self.seq = 0
        self.counts = {}
        self.events = []

    def add(self, topic, payload, *, node_ns, clock_ns, header_ns=None, pub_seq=None):
        event = dict(schema_version=SCHEMA_VERSION, seq=self.seq, topic=topic,
                     recv_monotonic_ns=time.monotonic_ns(), node_ros_time_ns=int(node_ns),
                     latest_clock_ns=clock_ns, header_stamp_ns=header_ns, payload=payload)
        if pub_seq is not None:
            event['pub_seq'] = pub_seq
        self.file.write(json.dumps(event, allow_nan=False) + '\n')
        self.seq += 1
        self.counts[topic] = self.counts.get(topic, 0) + 1
        self.events.append(event)
        return event

    def close(self):
        self.file.flush()
        os.fsync(self.file.fileno())
        self.file.close()


def _sim(event):
    return event['node_ros_time_ns'] / 1e9


def _intervals_s(times):
    return [b - a for a, b in zip(times, times[1:])]


def _percentile(ordered, fraction):
    """Linear interpolation between closest ranks, evaluated on an already sorted sample."""
    if len(ordered) == 1:
        return ordered[0]
    index = fraction * (len(ordered) - 1)
    low, high = math.floor(index), math.ceil(index)
    if low == high:
        return ordered[low]
    return ordered[low] + (ordered[high] - ordered[low]) * (index - low)


def interval_stats(intervals):
    """n / mean / median / p90 / max seconds; None entries when the segment has no interval."""
    values = sorted(float(v) for v in intervals)
    if not values:
        return dict(n=0, mean=None, median=None, p90=None, max=None)
    return dict(n=len(values), mean=sum(values) / len(values), median=statistics.median(values),
                p90=_percentile(values, .9), max=values[-1])


def real_time_factor(clock_events):
    """Sim span over the wall span of the same /clock receipts; a pace diagnostic, not world time."""
    if len(clock_events) < 2:
        return None
    sims = [e['payload']['clock_ns'] for e in clock_events]
    wall = [e['recv_monotonic_ns'] for e in clock_events]
    wall_span = (max(wall) - min(wall)) / 1e9
    if wall_span <= 0:
        return None
    return (max(sims) - min(sims)) / 1e9 / wall_span


def cadence_issue(phase, stats, clock_max, limit, failed):
    show = lambda v: 'none' if v is None else f'{v:.6f}'
    return (f'{phase}_command_cadence_violation: failed={",".join(failed)} '
            f'mean_interval_s={show(stats["mean"])} max_interval_s={show(stats["max"])} '
            f'mean_limit_s={CADENCE_MEAN_LIMIT_S:.6f} clock_max_interval_s={show(clock_max)} '
            f'max_limit_s={show(limit)} n={stats["n"]}')


def analyze(events, target):
    """Sample-bracketed measurement, never continuous-contact safety certification."""
    issues, warnings = [], []
    result = dict(valid=False, status='invalid', censored=False, issues=issues,
                  warnings=warnings, stop_evidence=False, target_m_s=target,
                  world_time_basis='node ROS simulation time at receipt, not a world header')
    clocks = [e['payload']['clock_ns'] for e in events if e['topic'] == CLOCK]
    if not clocks or any(b < a for a, b in zip(clocks, clocks[1:])):
        issues.append('missing_or_reversed_clock')
    commands = [e for e in events if e['topic'] == 'published_cmd']
    driving = [e for e in commands if e['payload']['phase'] == 'drive']
    zeros = [e for e in commands if e['payload']['phase'] == 'zero']
    world = [e for e in events if e['topic'] == WORLD]
    odom = [e for e in events if e['topic'] == ODOM]
    references = [e for e in events if e['topic'] == REF]
    clock_events = [e for e in events if e['topic'] == CLOCK]
    clock_stats = interval_stats(_intervals_s([e['payload']['clock_ns'] / 1e9 for e in clock_events]))
    clock_max = clock_stats['max']
    clock_limit = None if clock_max is None else clock_max + CADENCE_MAX_CLOCK_SLACK_S
    # Host-environment reporting only: wall gaps between /clock receipts say how bursty the host is.
    clock_wall_gaps = _intervals_s([e['recv_monotonic_ns'] / 1e9 for e in clock_events])
    result.update(clock_interval_stats_s=clock_stats, clock_max_interval_s=clock_max,
                  real_time_factor=real_time_factor(clock_events),
                  max_clock_recv_gap_wall_s=max(clock_wall_gaps, default=None),
                  clock_silences_over_0p5s_n=sum(1 for gap in clock_wall_gaps if gap > .5),
                  wall_stall_note=(
                      'host environment only: max_clock_recv_gap_wall_s is the largest wall gap between '
                      'consecutive /clock receipts of this case and clock_silences_over_0p5s_n counts wall '
                      'gaps above 0.5s. Simulation time does not advance while the host is stalled, so '
                      'these two fields are informational and never take part in validity, cadence or '
                      'stop evidence.'),
                  drive_command_interval_stats_s=interval_stats(_intervals_s([_sim(e) for e in driving])),
                  zero_command_interval_stats_s=interval_stats(_intervals_s([_sim(e) for e in zeros])),
                  command_cadence_rule=(
                      f'drive and zero phases are each valid only when mean command interval <= '
                      f'{CADENCE_MEAN_LIMIT_S:.6f}s and max command interval <= this case\'s measured max '
                      f'/clock interval + {CADENCE_MAX_CLOCK_SLACK_S:.6f}s. Command intervals use node ROS '
                      f'simulation time at receipt (the deployment-equivalent wall 20Hz timer quantizes them '
                      f'to the /clock grid); /clock, drive and zero stats are n/mean/median/p90/max seconds. '
                      f'real_time_factor is the /clock simulation span over the wall span of the same '
                      f'receipts.'),
                  interval_stats_note='p90 is linear interpolation between closest ranks on the sorted sample.')
    for name, stream in [('world', world), ('odom', odom), ('reference', references),
                         ('drive_commands', driving), ('zero_commands', zeros)]:
        if not stream:
            issues.append('missing_' + name)
    if not driving or not zeros or not world:
        result.update(status='censored', censored=True)
        return result
    first_zero, first_drive = zeros[0], driving[0]
    brake = _sim(first_zero)
    end = brake + 3.0
    result.update(first_drive_sim_s=_sim(first_drive), first_zero_sim_s=brake,
                  actual_drive_duration_s=brake - _sim(first_drive),
                  first_zero_pub_seq=first_zero['pub_seq'])
    if not 4.0 - 1e-8 <= result['actual_drive_duration_s'] <= 4.075:
        issues.append('drive_duration_not_4s_at_20hz')
    if _sim(zeros[-1]) < end - .075:
        issues.append('incomplete_3s_zero_command_window')
    if any(e['payload']['linear']['x'] != target for e in driving):
        issues.append('wrong_drive_target')
    if any(any(v != 0 for v in e['payload'][k].values())
           for e in zeros for k in ('linear', 'angular')):
        issues.append('nonzero_in_zero_phase')
    if any(b['recv_monotonic_ns'] < a['recv_monotonic_ns'] for a,b in zip(events, events[1:])):
        issues.append('receipt_monotonic_reversal')
    speeds = [e['payload']['longitudinal_velocity_m_s'] for e in world
              if brake - .5 <= _sim(e) < brake]
    result['prebrake_samples'] = len(speeds)
    result['prebrake_median_world_m_s'] = statistics.median(speeds) if speeds else None
    if not speeds or result['prebrake_median_world_m_s'] < target * .8:
        issues.append('prebrake_below_80_percent_target')
    window = [e for e in world if _sim(first_drive) <= _sim(e) <= end + .15]
    gaps = [_sim(b)-_sim(a) for a,b in zip(window,window[1:])]
    result['world_max_sample_gap_s'] = max(gaps, default=None)
    sparse = not gaps or min(gaps) < 0 or max(gaps) > .15 + 1e-9
    if sparse:
        issues.append('world_sampling_above_150ms_or_reversed')
        warnings.append('only_sample_brackets_no_precise_measurement')
    for phase, stream in [('drive', driving), ('zero', zeros)]:
        stats = result[phase + '_command_interval_stats_s']
        gaps = _intervals_s([_sim(e) for e in stream])
        result[phase + '_command_max_gap_s'] = stats['max']
        failed = []
        if not gaps or min(gaps) < 0:
            failed.append('missing_or_reversed_command_intervals')
        if stats['mean'] is None or stats['mean'] > CADENCE_MEAN_LIMIT_S + 1e-9:
            failed.append('mean_above_%gs' % CADENCE_MEAN_LIMIT_S)
        if clock_limit is None or stats['max'] is None or stats['max'] > clock_limit + 1e-9:
            failed.append('max_above_clock_plus_%gs' % CADENCE_MAX_CLOCK_SLACK_S)
        if failed:
            issues.append(cadence_issue(phase, stats, clock_max, clock_limit, failed))
    # A low-speed run must be seen for .5 seconds and complete within the fixed window.
    onset, previous, confirmed = None, None, None
    before_onset = None
    for event in world:
        t = _sim(event)
        if t < brake:
            previous = event
            continue
        if t > end + 1e-9:
            break
        speed = math.sqrt(sum(float(v)**2 for v in event['payload']['linear'].values()))
        if speed < .05:
            if onset is None:
                onset, before_onset = event, previous
            if t - _sim(onset) >= .5 - 1e-9:
                confirmed = event
                break
        else:
            onset, before_onset = None, None
        previous = event
    anchor_before = [e for e in world if _sim(e) <= brake]
    anchor_after = [e for e in world if _sim(e) >= brake]
    if not confirmed:
        result.update(status='censored', censored=True)
        issues.append('not_stopped_within_3s')
        if anchor_before and window:
            result['observed_displacement_until_last_sample_m'] = (
                window[-1]['payload']['position']['x'] - anchor_before[-1]['payload']['position']['x'])
        return result
    result['stop_evidence'] = True
    result['stop_confirmation_sim_s'] = _sim(confirmed)
    lo = max(brake, _sim(before_onset)) if before_onset else brake
    hi = _sim(onset)
    result['stop_onset_after_zero_s_interval'] = [max(0,lo-brake), max(0,hi-brake)]
    if not anchor_before or not anchor_after or _sim(anchor_after[0])-brake > .15:
        issues.append('missing_zero_pose_bracket')
    else:
        anchors = [anchor_before[-1]['payload']['position']['x'],
                   anchor_after[0]['payload']['position']['x']]
        ends = [onset['payload']['position']['x']]
        if before_onset:
            ends.append(before_onset['payload']['position']['x'])
        result['stop_displacement_x_m_interval'] = [min(ends)-max(anchors),max(ends)-min(anchors)]
        result['zero_world_position_x_m_interval'] = [min(anchors), max(anchors)]
        result['interval_note'] = ('Sample brackets on straight +x receipt-time alignment; '
                                   'not interpolation, continuous path or braking safety proof.')
    # Nearest receipt-time comparisons are diagnostics, not exact simultaneous physical states.
    odom_deltas=[]
    for e in window:
        if odom:
            o=min(odom,key=lambda q:abs(_sim(q)-_sim(e)))
            if abs(_sim(o)-_sim(e)) <= .15:
                odom_deltas.append(o['payload']['linear']['x']-e['payload']['longitudinal_velocity_m_s'])
    result['odom_minus_world_velocity_m_s_range'] = [min(odom_deltas),max(odom_deltas)] if odom_deltas else None
    receipts=[]
    for command in driving + zeros:
        matches=[ref for ref in references if ref['recv_monotonic_ns'] >= command['recv_monotonic_ns']
                 and ref['payload']['linear'] == command['payload']['linear']
                 and ref['payload']['angular'] == command['payload']['angular']]
        if matches:
            receipts.append((matches[0]['recv_monotonic_ns']-command['recv_monotonic_ns'])/1e6)
    result['cmd_to_next_matching_reference_receipt_ms_range'] = [min(receipts),max(receipts)] if receipts else None
    result['reference_matching_note'] = 'Repeated Twist has no sequence ID; next matching receipt is not unique correlation or controller consumption.'
    result['valid'] = not issues
    result['status'] = 'valid' if not issues else 'invalid'
    return result


def proc_table():
    table={}
    for entry in Path('/proc').iterdir():
        if not entry.name.isdigit():
            continue
        try:
            # comm can contain spaces and parentheses; parse only after its last ')'.
            fields=(entry/'stat').read_text().rsplit(')',1)[1].split()
            table[int(entry.name)] = dict(ppid=int(fields[1]), start=fields[19], state=fields[0],
                args=(entry/'cmdline').read_bytes().replace(b'\0',b' ').decode(errors='replace'),
                env=(entry/'environ').read_bytes().split(b'\0'))
        except (OSError, ValueError):
            continue
    return table


class OwnedProcesses:
    def __init__(self, root):
        self.root=root
        self.known={}
        self.capture()

    def capture(self):
        table=proc_table()
        parents={pid for pid,start in self.known.items()
                 if pid in table and table[pid]['start']==start}
        if self.root in table and (not self.known or self.known.get(self.root)==table[self.root]['start']):
            parents.add(self.root)
        changed=True
        while changed:
            changed=False
            for pid,row in table.items():
                if row['ppid'] in parents and pid not in parents:
                    parents.add(pid);changed=True
        for pid in parents:
            self.known[pid]=table[pid]['start']

    def live(self):
        table=proc_table()
        return [pid for pid,start in self.known.items() if pid in table
                and table[pid]['start']==start and table[pid]['state']!='Z']

    def cleanup(self):
        self.capture()
        signaled=[]
        # Only exact tracked PID + Linux starttime identities, including reparented children.
        for sig,timeout in [(signal.SIGINT,5.),(signal.SIGTERM,3.),(signal.SIGKILL,2.)]:
            for pid in self.live():
                try:
                    os.kill(pid,sig);signaled.append([pid,int(sig)])
                except ProcessLookupError:
                    pass
            limit=time.monotonic()+timeout
            while self.live() and time.monotonic()<limit:
                self.capture();time.sleep(.1)
        return dict(owned_pid_starttimes=self.known,signaled=signaled,remaining_live=self.live())


def isolation_preflight():
    table=proc_table()
    ancestors={os.getpid()}
    pid=os.getpid()
    while pid in table and table[pid]['ppid'] not in ancestors:
        pid=table[pid]['ppid'];ancestors.add(pid)
    conflicts=[]
    for pid,row in table.items():
        if pid not in ancestors and b'ROS_DOMAIN_ID=63' in row['env']:
            conflicts.append(dict(pid=pid,args=row['args']))
        if ('gzserver' in row['args'] or 'gzclient' in row['args']) and any(
                e == b'GAZEBO_MASTER_URI=http://127.0.0.1:11365' for e in row['env']):
            conflicts.append(dict(pid=pid,args=row['args']))
    with socket.socket() as sock:
        try:
            sock.bind(('127.0.0.1',PORT))
            port_free=True
        except OSError:
            port_free=False
    if conflicts or not port_free:
        raise RuntimeError('isolation conflict: '+json.dumps(dict(processes=conflicts,port_free=port_free)))
    return dict(domain=DOMAIN,port=PORT,port_free=port_free,conflicts=conflicts)


def snapshot_sources(out, repo):
    # 车模/世界用**本仓** simulation/gazebo（实测几何 0.35/0.145/0.28），不再去猜同级目录里的
    # Auto_Car-main（旧几何 0.33/0.07/0.21）。
    ws=repo/'simulation/gazebo'
    sources=[Path(__file__),repo/'evaluation/test_gazebo_braking_probe.py',repo/'evaluation/run_gazebo_braking_probe.sh']
    sources += [ws/'src/racecar_sim'/x for x in ['launch/sim_bringup.launch.py','config/params.yaml',
                'config/ros2_control.yaml.in','urdf/racecar.xacro','racecar_sim/sim_wiring.py','worlds/racecar.world']]
    build_wiring = ws/'build/racecar_sim/racecar_sim/sim_wiring.py'
    if build_wiring.is_file():
        sources += [build_wiring]
    sources += [Path('/opt/ros/humble/include/steering_controllers_library/steering_controllers_library_parameters.hpp'),
        Path('/opt/ros/humble/lib/libackermann_steering_controller.so'),
        Path('/opt/ros/humble/lib/libsteering_controllers_library.so'),
        Path('/opt/ros/humble/lib/libgazebo_ros2_control.so')]
    folder=out/'sources';folder.mkdir()
    records=[]
    for i,path in enumerate(sources):
        content=path.read_bytes();dest=folder/(str(i)+'_'+path.name);dest.write_bytes(content)
        records.append(dict(path=str(path),resolved=str(path.resolve()),sha256=hashlib.sha256(content).hexdigest(),
                            snapshot_path=str(dest)))
    cmd=['dpkg-query','-W','gazebo','ros-humble-ackermann-steering-controller',
         'ros-humble-steering-controllers-library','ros-humble-gazebo-ros2-control','ros-humble-rclpy']
    result=subprocess.run(cmd,capture_output=True,text=True)
    data=dict(schema_version=1,preregistration=PREREGISTRATION,observer_only=False,physical_goal_complete=False,model=None,
              model_note='No r36, guard, policy enable, training, reverse or A-D',
              cases=[dict(target=t,rep=r) for t,r in CASES],artifacts=records,
              versions=dict(command=cmd,returncode=result.returncode,stdout=result.stdout,stderr=result.stderr))
    write_json(out/'manifest.json',data)
    return data


def reference_identity(refs):
    """Unknown DDS discovery identities never authorize driving."""
    if len(refs)>1:
        raise RuntimeError('external_or_duplicate_reference_publisher')
    if not refs:
        return False
    r=refs[0]
    if r['node_name'] in ('', '_NODE_NAME_UNKNOWN_') or r['node_namespace'] in ('', '_NODE_NAMESPACE_UNKNOWN_'):
        return False
    if r['node_name']!='sim_wiring' or r['node_namespace']!='/':
        raise RuntimeError('external_or_duplicate_reference_publisher')
    return True

def run_case(case_dir, world_path, target, rep, diagnostic_only=False):
    # ROS imports deliberately inside the live entrypoint, absent from offline tests.
    import rclpy
    from rclpy.node import Node
    from rclpy.parameter import Parameter
    from rclpy.qos import qos_profile_sensor_data
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Odometry
    from gazebo_msgs.msg import ModelStates
    from rosgraph_msgs.msg import Clock
    isolation=isolation_preflight()
    case_dir.mkdir()
    rec=Recorder(case_dir/'events.jsonl')
    rclpy.init()
    node=Node('owned_braking_probe',parameter_overrides=[Parameter('use_sim_time',value=True)])
    pub=node.create_publisher(Twist,CMD,10)
    latest=None;pubseq=0;fatal=None;phase='waiting';start=None;brake=None
    world_latest=None;odom_latest=None;last_clock_wall=time.monotonic()
    launch=None;owned=None;cleanup=None;timer=None
    command=[PYTHON,'/opt/ros/humble/bin/ros2','launch','racecar_sim','sim_bringup.launch.py',
             'use_gui:=false','world:='+str(world_path),'spawn_x:=0','spawn_y:=0','spawn_yaw:=0']
    log=(case_dir/'launch.log').open('x')

    def now_ns():
        return node.get_clock().now().nanoseconds

    def add(topic,payload,header=None,ps=None):
        return rec.add(topic,payload,node_ns=now_ns(),clock_ns=latest,header_ns=header,pub_seq=ps)

    def vector(v):
        return dict(x=float(v.x),y=float(v.y),z=float(v.z))

    def quaternion(q):
        return dict(x=float(q.x),y=float(q.y),z=float(q.z),w=float(q.w))

    def on_clock(m):
        nonlocal latest,fatal,last_clock_wall
        ns=m.clock.sec*10**9+m.clock.nanosec
        if latest is not None and ns<latest:
            fatal='clock_reversal'
        latest=ns;last_clock_wall=time.monotonic()
        add(CLOCK,dict(clock_ns=ns))

    def on_world(m):
        nonlocal world_latest,fatal
        if m.name.count('racecar')!=1:
            if phase != 'waiting':
                fatal='missing_or_duplicate_world_racecar'
            add('world_startup',dict(names=list(m.name)))
            return
        if 'marker_box' in m.name:
            fatal='marker_box_present';return
        i=m.name.index('racecar');pose=m.pose[i];tw=m.twist[i];q=pose.orientation
        yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
        world_latest=add(WORLD,dict(model_name='racecar',names=list(m.name),position=vector(pose.position),
            orientation=quaternion(q),linear=vector(tw.linear),angular=vector(tw.angular),
            longitudinal_velocity_m_s=float(tw.linear.x*math.cos(yaw)+tw.linear.y*math.sin(yaw))))

    def on_odom(m):
        nonlocal odom_latest
        tw=m.twist.twist
        odom_latest=add(ODOM,dict(frame_id=m.header.frame_id,child_frame_id=m.child_frame_id,
            position=vector(m.pose.pose.position),orientation=quaternion(m.pose.pose.orientation),
            linear=vector(tw.linear),angular=vector(tw.angular),pose_covariance=list(m.pose.covariance),
            twist_covariance=list(m.twist.covariance)),m.header.stamp.sec*10**9+m.header.stamp.nanosec)

    def on_ref(m):
        add(REF,dict(linear=vector(m.linear),angular=vector(m.angular)))

    node.create_subscription(Clock,CLOCK,on_clock,qos_profile_sensor_data)
    node.create_subscription(ModelStates,WORLD,on_world,qos_profile_sensor_data)
    node.create_subscription(Odometry,ODOM,on_odom,qos_profile_sensor_data)
    node.create_subscription(Twist,REF,on_ref,qos_profile_sensor_data)

    def graph_check(ready=False):
        publishers=node.get_publishers_info_by_topic(CMD)
        if any(x.node_name!='owned_braking_probe' or x.node_namespace!='/' for x in publishers) or len(publishers)>1:
            raise RuntimeError('external_cmd_publisher')
        nodes=node.get_node_names_and_namespaces()
        if sum(n=='sim_wiring' for n,ns in nodes)>1:
            raise RuntimeError('duplicate_wiring')
        refs=node.get_publishers_info_by_topic(REF)
        identities=[dict(node_name=x.node_name,node_namespace=x.node_namespace,
                         gid=bytes(x.endpoint_gid).hex()) for x in refs]
        table=proc_table()
        processes=[dict(pid=p,start=r['start'],ppid=r['ppid'],args=r['args'],
                        owned=bool(owned and owned.known.get(p)==r['start']))
                   for p,r in table.items() if b'ROS_DOMAIN_ID=63' in r['env']]
        add('graph',dict(refs=identities,nodes=nodes,domain_processes=processes,
                        association_note='GID has no ROS graph PID mapping; PID ownership is independent evidence'))
        identified=reference_identity(identities)
        if not identified and not ready:
            raise RuntimeError('reference_identity_lost_during_drive')
        if ready:
            return len(publishers)==1 and identified and sum(n=='sim_wiring' and ns=='/' for n,ns in nodes)==1
        return True

    def issue(v,why):
        nonlocal pubseq
        m=Twist();m.linear.x=float(v)
        pub.publish(m)
        add('published_cmd',dict(topic=CMD,phase=why,linear=vector(m.linear),angular=vector(m.angular)),ps=pubseq)
        pubseq+=1

    def tick():
        nonlocal phase,start,brake,fatal
        try:
            if phase=='waiting':
                return
            ns=now_ns()
            if phase=='drive' and ns-start >= 4*10**9:
                phase='zero';brake=ns
            issue(target if phase=='drive' else 0.,phase)
        except Exception as exc:
            fatal=repr(exc)

    failure=None
    try:
        launch=subprocess.Popen(command,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
        owned=OwnedProcesses(launch.pid)
        add('lifecycle',dict(status='launch_started',command=command,pid=launch.pid,isolation=isolation))
        deadline=time.monotonic()+100.;graph_due=0.
        while phase=='waiting':
            owned.capture();rclpy.spin_once(node,timeout_sec=.03)
            if fatal:
                raise RuntimeError(fatal)
            if launch.poll() is not None:
                raise RuntimeError('launch_exited_'+str(launch.returncode))
            if time.monotonic()>deadline:
                raise TimeoutError('startup_timeout')
            if time.monotonic()>=graph_due:
                ready=graph_check(ready=True);graph_due=time.monotonic()+.2
                if ready and latest is not None and world_latest and odom_latest:
                    if _sim(world_latest)>0 and abs(world_latest['payload']['longitudinal_velocity_m_s'])<.05:
                        if diagnostic_only:
                            add('lifecycle',dict(status='diagnostic_identity_ready'))
                            break
                        phase='drive';start=now_ns();issue(target,'drive')
                        add('lifecycle',dict(status='drive_started',start_node_ros_time_ns=start))
        timer=node.create_timer(.05,tick)
        wall_deadline=time.monotonic()+90.
        while not diagnostic_only and (brake is None or now_ns()-brake < 3*10**9):
            owned.capture();rclpy.spin_once(node,timeout_sec=.02)
            if fatal:
                raise RuntimeError(fatal)
            if launch.poll() is not None:
                raise RuntimeError('launch_exited_'+str(launch.returncode))
            # Liveness only. A host stall freezes simulation time too, so a multi-second /clock silence
            # with the simulation still advancing is normal here (see CLOCK_SILENCE_LIMIT_S); a stall
            # cannot corrupt braking physics because nothing advances while it lasts.
            if time.monotonic()>wall_deadline or time.monotonic()-last_clock_wall>CLOCK_SILENCE_LIMIT_S:
                raise TimeoutError('wall_or_clock_stall_timeout')
            if time.monotonic()>=graph_due:
                graph_check();graph_due=time.monotonic()+.2
        # Keep the final zero at the window boundary, while already spinning world feedback.
        if not diagnostic_only:
            issue(0.,'zero')
        add('lifecycle',dict(status='observation_window_closed'))
    except BaseException as exc:
        failure=type(exc).__name__+': '+str(exc)
        add('lifecycle',dict(status='error',error=failure))
    finally:
        if timer is not None:
            timer.cancel()
        try:
            for _ in range(5):
                if rclpy.ok():
                    issue(0.,'safety_zero')
                    rclpy.spin_once(node,timeout_sec=.02)
        except Exception as exc:
            failure = failure or ('safety_zero_error: '+repr(exc))
        finally:
            node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
            cleanup=owned.cleanup() if owned else dict(remaining_live=[])
        if launch is not None:
            try:
                launch.wait(timeout=2)
            except subprocess.TimeoutExpired:
                pass
        log.close()
        rec.close()
    result=analyze(rec.events,target)
    if failure:
        result['issues'].append('runtime_exception: '+failure);result['valid']=False
        result['status']='error'
    if cleanup['remaining_live']:
        result['issues'].append('owned_process_cleanup_failed');result['valid']=False;result['status']='error'
    # Capture generated configuration without editing it or removing shared paths.
    generated=Path('/tmp/racecar_ros2_control_d63.yaml')
    if generated.exists():
        data=generated.read_bytes();(case_dir/'generated_controller.yaml').write_bytes(data)
        result['generated_controller_sha256']=hashlib.sha256(data).hexdigest()
    result.update(rep=rep,event_count=rec.seq,topic_counts=rec.counts,cleanup=cleanup,
                  command=command,world_path=str(world_path),world_sha256=hashlib.sha256(world_path.read_bytes()).hexdigest(),
                  physical_goal_complete=False)
    write_json(case_dir/'summary.json',result)
    return result


def infrastructure_failure(result):
    """True only when the probe itself died on a case (crash, launch failure, timeout, identity abort).

    Measurement-quality issues such as cadence, world sampling or pose brackets are expected on this
    bursty host: they invalidate their own row and stay in the denominator, but must not end a run.
    """
    return result.get('status')=='error'


def plan_abort(results, limit=CONSECUTIVE_INFRASTRUCTURE_LIMIT):
    """Abort the remaining cases only after `limit` consecutive infrastructure failures."""
    if len(results)<limit or not all(infrastructure_failure(x) for x in results[-limit:]):
        return None
    return 'repeated_infrastructure_error'


def execute_plan(out, world_path, rows, diagnostic_only, report, run_case_fn=run_case):
    """One pass over CASES: every requested case gets its own row and a failed case is never rerun."""
    for i,(target,rep) in enumerate(CASES):
        label=f'target{target:.1f}_rep{rep}'
        print('START',i+1,label,flush=True)
        rows[i]=run_case_fn(out/label,world_path,target,rep,diagnostic_only=diagnostic_only)
        write_json(out/'report.json',report)
        print('END',label,json.dumps(rows[i]),flush=True)
        reason='stop_evidence_missing_or_runtime_error' if diagnostic_only else plan_abort(rows[:i+1])
        if reason:
            report['stop_reason']=reason
            for rest in rows[i+1:]:
                rest['reason']='not_run_after_'+label
            return reason
    return None


def main(argv=None):
    parser=argparse.ArgumentParser()
    parser.add_argument('--out',default=DEFAULT_OUT,help='owned run directory (default: %(default)s)')
    parser.add_argument('--diagnostic-only',action='store_true')
    args=parser.parse_args(argv)
    if not PYTHON:
        raise RuntimeError('VENV_PYTHON (or VENV_ROOT, which implies VENV_ROOT/bin/python) is '
                           'required: this repo carries no machine default')
    if os.path.realpath(os.environ.get('VIRTUAL_ENV') or VENV_ROOT) != os.path.realpath(VENV_ROOT):
        raise RuntimeError('specified venv required')
    if os.environ.get('ROS_DOMAIN_ID')!='63' or os.environ.get('GAZEBO_MASTER_URI')!='http://127.0.0.1:11365':
        raise RuntimeError('fixed domain63 / port11365 required')
    repo=Path(__file__).resolve().parents[1]
    out=Path(args.out)
    out=(out if out.is_absolute() else repo/out).resolve()
    if out.parent!=repo/'runs' or not out.name.startswith('gazebo_braking_probe_20261004'):
        raise ValueError('output outside owned prefix')
    out.mkdir(exist_ok=False)
    rows=[dict(target_m_s=t,rep=r,status='not_run',valid=False) for t,r in CASES]
    report=dict(schema_version=1,preregistration=PREREGISTRATION,requested=len(CASES),rows=rows,
                clock_silence_limit_s=CLOCK_SILENCE_LIMIT_S,
                consecutive_infrastructure_limit=CONSECUTIVE_INFRASTRUCTURE_LIMIT,
                status='running',physical_goal_complete=False)
    write_json(out/'report.json',report)
    try:
        report['preflight']=isolation_preflight()
        manifest=snapshot_sources(out,repo)
        source=repo/'simulation/gazebo/src/racecar_sim/worlds/racecar.world'
        removed=owned_empty_world(source,out/'owned_empty.world')
        report['removed_world_models']=removed
        execute_plan(out,out/'owned_empty.world',rows,args.diagnostic_only,report)
        diffs=[]
        for artifact in manifest['artifacts']:
            if hashlib.sha256(Path(artifact['path']).read_bytes()).hexdigest()!=artifact['sha256']:
                diffs.append(artifact['path'])
        report['source_hash_differences']=diffs
        report['status']='closed' if not diffs else 'source_changed'
    except BaseException as exc:
        report.update(status='error',error=type(exc).__name__+': '+str(exc))
        print('ERROR',report['error'],flush=True)
    finally:
        report['valid_count']=sum(bool(x['valid']) for x in rows)
        report['completed_count']=sum(x['status']!='not_run' for x in rows)
        report['final_acceptance_gate']=False
        write_json(out/'report.json',report)
    return 0 if report['status']=='closed' else 1


if __name__=='__main__':
    raise SystemExit(main())