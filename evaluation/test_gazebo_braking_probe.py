import json
from pathlib import Path
import xml.etree.ElementTree as ET
import pytest
from evaluation.gazebo_braking_probe import (analyze, owned_empty_world, Recorder,
                                             OwnedProcesses, CASES, WORLD, ODOM, REF, CLOCK,
                                             CADENCE_MEAN_LIMIT_S, CLOCK_SILENCE_LIMIT_S,
                                             CONSECUTIVE_INFRASTRUCTURE_LIMIT,
                                             DEFAULT_OUT, PREREGISTRATION)


def events(target=3., stop=True, step=.05):
    out=[]
    def add(topic,t,payload,pubseq=None):
        e=dict(seq=len(out),topic=topic,node_ros_time_ns=round(t*1e9),
            recv_monotonic_ns=round(t*1e9),latest_clock_ns=round(t*1e9),header_stamp_ns=None,payload=payload)
        if pubseq is not None:e['pub_seq']=pubseq
        out.append(e)
    for k in range(141):
        t=round(k*.05,9)
        add(CLOCK,t,dict(clock_ns=round(t*1e9)))
        v=target if t<4 else (max(0,target*(1-(t-4)/.4)) if stop else target)
        pos=target*t if t<4 else target*4+(target*(t-4)-target/0.8*(t-4)**2 if t<4.4 else target*.2)
        if not stop:pos=target*t
        phase='drive' if t<4 else 'zero'
        linear=dict(x=target if t<4 else 0.,y=0.,z=0.)
        angular=dict(x=0.,y=0.,z=0.)
        add('published_cmd',t,dict(phase=phase,linear=linear,angular=angular),k)
        add(REF,t,dict(linear=linear,angular=angular))
        if abs((t/step)-round(t/step))<1e-6:
            add(WORLD,t,dict(position=dict(x=pos,y=0.,z=0.),linear=dict(x=v,y=0.,z=0.),
                            longitudinal_velocity_m_s=v))
        add(ODOM,t,dict(linear=dict(x=v,y=0.,z=0.)))
    return out


def grid_events(target=3., clock_step=.1, command_steps=(.1,0.), world_step=.1, horizon=7.,
                recv_stall=None):
    """Simulation time advances only in /clock quanta while commands keep the wall 20Hz pace.

    recv_stall=(from_s, seconds) shifts every receipt at or after from_s by that many wall seconds,
    modelling a host stall that leaves simulated time untouched.
    """
    out=[]
    def add(topic,t,payload,pubseq=None):
        recv=t+(recv_stall[1] if recv_stall and t>=recv_stall[0] else 0.)
        e=dict(seq=len(out),topic=topic,node_ros_time_ns=round(t*1e9),
            recv_monotonic_ns=round(recv*1e9),latest_clock_ns=round(t*1e9),header_stamp_ns=None,payload=payload)
        if pubseq is not None:e['pub_seq']=pubseq
        out.append(e)
    commands=[];t=0.;i=0
    while t<=horizon+1e-9:
        commands.append(round(t,9))
        t=round(t+command_steps[i%len(command_steps)],9);i+=1
    grid=[round(k*clock_step,9) for k in range(int(round(horizon/clock_step))+1)]
    ci=0
    for t in grid:
        add(CLOCK,t,dict(clock_ns=round(t*1e9)))
        while ci<len(commands) and commands[ci]<=t+1e-12:
            ct=commands[ci];phase='drive' if ct<4 else 'zero'
            linear=dict(x=target if phase=='drive' else 0.,y=0.,z=0.)
            angular=dict(x=0.,y=0.,z=0.)
            add('published_cmd',ct,dict(phase=phase,linear=linear,angular=angular),ci)
            add(REF,ct,dict(linear=linear,angular=angular))
            ci+=1
        v=target if t<4.4 else 0.
        if abs((t/world_step)-round(t/world_step))<1e-6:
            pos=target*t if t<4.4 else target*4.4
            add(WORLD,t,dict(position=dict(x=pos,y=0.,z=0.),linear=dict(x=v,y=0.,z=0.),
                            longitudinal_velocity_m_s=v))
        add(ODOM,t,dict(linear=dict(x=v,y=0.,z=0.)))
    return out


def test_fixed_preregistration():
    assert CASES==[(1.,2),(2.,2),(3.,1),(3.,2),(3.,3)]
    assert PREREGISTRATION=='v4'
    assert DEFAULT_OUT=='runs/gazebo_braking_probe_20261004_v4'
    assert CLOCK_SILENCE_LIMIT_S==10.
    assert CONSECUTIVE_INFRASTRUCTURE_LIMIT==3


def test_world_is_owned_copy_only(tmp_path):
    src=tmp_path/'original.world'
    src.write_text('<sdf><world name="default"><include><uri>model://ground_plane</uri></include>'
        '<plugin filename="libgazebo_ros_state.so"/><model name="marker_box"/></world></sdf>')
    original=src.read_bytes();dest=tmp_path/'owned.world'
    assert owned_empty_world(src,dest)==['marker_box']
    assert src.read_bytes()==original
    tree=ET.parse(dest)
    assert not tree.findall('.//model')
    assert tree.find('.//plugin').attrib['filename']=='libgazebo_ros_state.so'
    with pytest.raises(FileExistsError):owned_empty_world(src,dest)


@pytest.mark.parametrize('xml', ['<sdf/>','<sdf><world/></sdf>',
    '<sdf><world><plugin filename="libgazebo_ros_state.so"/></world></sdf>'])
def test_wrong_world_rejected(tmp_path,xml):
    p=tmp_path/'x';p.write_text(xml)
    with pytest.raises(ValueError):owned_empty_world(p,tmp_path/'out')


def test_valid_sample_intervals():
    r=analyze(events(),3.)
    assert r['valid'],r
    assert r['stop_evidence'] and not r['censored']
    assert r['stop_onset_after_zero_s_interval'][0]<=.4<=r['stop_onset_after_zero_s_interval'][1]+1e-9
    lo,hi=r['stop_displacement_x_m_interval']
    assert lo<=.6+1e-9 and hi>=.6-1e-9
    assert r['actual_drive_duration_s']==4.
    assert r['reference_matching_note'].startswith('Repeated Twist')


def test_not_stopped_censored():
    r=analyze(events(stop=False),3.)
    assert r['censored'] and not r['valid'] and not r['stop_evidence']
    assert 'not_stopped_within_3s' in r['issues']


def test_late_stop_not_certified():
    stream=events()
    for e in stream:
        if e['topic']==WORLD and 4<=e['node_ros_time_ns']/1e9<6.8:
            e['payload']['linear']['x']=1.
            e['payload']['longitudinal_velocity_m_s']=1.
    assert analyze(stream,3.)['censored']


def test_low_speed_excursion_resets_half_second_run():
    stream=events()
    for e in stream:
        if e['topic']==WORLD and 4.4<=e['node_ros_time_ns']/1e9<=7:
            e['payload']['linear']['x']=.1 if int(e['node_ros_time_ns']/1e8)%4==0 else 0.
    assert analyze(stream,3.)['censored']


def test_sideways_motion_is_not_stopped():
    stream=events()
    for e in stream:
        if e['topic']==WORLD:e['payload']['linear']['y']=.2
    assert analyze(stream,3.)['censored']


def test_weak_prebrake_invalid_no_extension():
    r=analyze(events(target=1.),3.)
    assert not r['valid'] and r['stop_evidence']
    assert 'prebrake_below_80_percent_target' in r['issues']


def test_sparse_retains_interval_only():
    r=analyze(events(step=.2),3.)
    assert not r['valid'] and r['stop_evidence']
    assert 'world_sampling_above_150ms_or_reversed' in r['issues']
    assert 'stop_displacement_x_m_interval' in r
    assert 'precise_distance_m' not in r


@pytest.mark.parametrize('topic', [WORLD,ODOM,REF,CLOCK,'published_cmd'])
def test_missing_stream_kept_invalid(topic):
    r=analyze([e for e in events() if e['topic']!=topic],3.)
    assert not r['valid']


def test_clock_backwards_invalid():
    stream=events();clock=[e for e in stream if e['topic']==CLOCK]
    clock[90]['payload']['clock_ns']=1
    assert not analyze(stream,3.)['valid']


def test_zero_window_and_cadence_not_relaxed():
    stream=events()
    filtered=[e for e in stream if not(e['topic']=='published_cmd' and e['node_ros_time_ns']>=6e9)]
    r=analyze(filtered,3.)
    assert not r['valid'] and 'incomplete_3s_zero_command_window' in r['issues']


def test_cadence_mean_ok_when_max_equals_clock_granularity():
    r=analyze(grid_events(command_steps=(.1,0.)),3.)
    assert r['valid'],r['issues']
    assert r['clock_interval_stats_s']['max']==pytest.approx(.1)
    for phase in ('drive','zero'):
        stats=r[phase+'_command_interval_stats_s']
        assert stats['mean']<CADENCE_MEAN_LIMIT_S and stats['max']==pytest.approx(.1)
        assert not [x for x in r['issues'] if x.startswith(phase+'_command_cadence_violation')]


def test_cadence_mean_above_limit_invalid():
    r=analyze(grid_events(command_steps=(.1,)),3.)
    assert not r['valid']
    for phase in ('drive','zero'):
        issue=[x for x in r['issues'] if x.startswith(phase+'_command_cadence_violation')]
        assert issue and 'failed=mean_above_0.075s ' in issue[0]+' '
        assert 'mean_limit_s=0.075000' in issue[0] and 'max_interval_s=0.100000' in issue[0]
        assert 'clock_max_interval_s=0.100000' in issue[0] and 'max_limit_s=0.120000' in issue[0]


def test_cadence_max_above_clock_plus_slack_invalid():
    r=analyze(grid_events(command_steps=(0.,0.,0.,.2)),3.)
    assert not r['valid']
    for phase in ('drive','zero'):
        stats=r[phase+'_command_interval_stats_s']
        assert stats['mean']<CADENCE_MEAN_LIMIT_S and stats['max']==pytest.approx(.2)
        issue=[x for x in r['issues'] if x.startswith(phase+'_command_cadence_violation')]
        assert issue and 'failed=max_above_clock_plus_0.02s ' in issue[0]+' '
        assert 'clock_max_interval_s=0.100000' in issue[0] and 'max_limit_s=0.120000' in issue[0]


def test_interval_statistics_and_real_time_factor_reported():
    r=analyze(grid_events(),3.)
    for key in ('clock_interval_stats_s','drive_command_interval_stats_s','zero_command_interval_stats_s'):
        stats=r[key]
        assert set(stats)=={'n','mean','median','p90','max'} and stats['n']>0
        assert stats['mean']<=stats['p90']<=stats['max'] and stats['median']<=stats['max']
        assert None not in stats.values()
    assert r['clock_max_interval_s']==pytest.approx(.1)
    assert r['real_time_factor']==pytest.approx(1.)
    assert 'command_cadence_rule' in r and 'interval_stats_note' in r


def test_interval_statistics_present_when_censored():
    r=analyze(events(stop=False),3.)
    assert r['censored']
    assert r['clock_interval_stats_s']['n']>0 and r['drive_command_interval_stats_s']['n']>0


def test_interval_statistics_degenerate_samples():
    from evaluation.gazebo_braking_probe import interval_stats
    assert interval_stats([])==dict(n=0,mean=None,median=None,p90=None,max=None)
    assert interval_stats([.05])==dict(n=1,mean=.05,median=.05,p90=.05,max=.05)


def _not_run_rows():
    return [dict(target_m_s=t,rep=r,status='not_run',valid=False) for t,r in CASES]


def test_wall_stall_fields_are_informational():
    r=analyze(grid_events(),3.)
    assert r['max_clock_recv_gap_wall_s']==pytest.approx(.1)
    assert r['clock_silences_over_0p5s_n']==0 and 'wall_stall_note' in r
    stalled=analyze(grid_events(recv_stall=(3.,4.)),3.)
    assert stalled['valid'],stalled['issues']
    assert stalled['max_clock_recv_gap_wall_s']==pytest.approx(4.1)
    assert stalled['clock_silences_over_0p5s_n']==1


def test_wall_stall_fields_present_when_censored_and_without_clock():
    r=analyze(events(stop=False),3.)
    assert r['censored'] and r['max_clock_recv_gap_wall_s']==pytest.approx(.05)
    assert r['clock_silences_over_0p5s_n']==0 and 'wall_stall_note' in r
    blind=analyze([e for e in events() if e['topic']!=CLOCK],3.)
    assert blind['max_clock_recv_gap_wall_s'] is None and blind['clock_silences_over_0p5s_n']==0


def test_plan_continues_after_a_single_case_failure(tmp_path):
    from evaluation import gazebo_braking_probe as m
    seen=[]
    def stub(case_dir,world_path,target,rep,diagnostic_only=False):
        seen.append((target,rep))
        failed=len(seen)==1
        return dict(target_m_s=target,rep=rep,status='error' if failed else 'valid',
                    valid=not failed,censored=False,
                    issues=['runtime_exception: boom'] if failed else [])
    rows=_not_run_rows();report=dict(rows=rows)
    m.execute_plan(tmp_path,None,rows,False,report,run_case_fn=stub)
    assert seen==list(CASES)
    assert rows[0]['status']=='error' and all(row['status']=='valid' for row in rows[1:])
    assert 'stop_reason' not in report


def test_not_stopped_censes_do_not_abort_the_plan(tmp_path):
    from evaluation import gazebo_braking_probe as m
    seen=[]
    def stub(case_dir,world_path,target,rep,diagnostic_only=False):
        seen.append((target,rep))
        return dict(target_m_s=target,rep=rep,status='invalid',valid=False,censored=True,
                    issues=['not_stopped_within_3s'])
    rows=_not_run_rows();report=dict(rows=rows)
    m.execute_plan(tmp_path,None,rows,False,report,run_case_fn=stub)
    assert seen==list(CASES) and 'stop_reason' not in report
    assert all(row['status']=='invalid' for row in rows)


def test_plan_aborts_after_three_consecutive_infrastructure_failures(tmp_path):
    from evaluation import gazebo_braking_probe as m
    seen=[]
    def stub(case_dir,world_path,target,rep,diagnostic_only=False):
        seen.append((target,rep))
        return dict(target_m_s=target,rep=rep,status='error',valid=False,censored=False,
                    issues=['runtime_exception: wall_or_clock_stall_timeout'])
    rows=_not_run_rows();report=dict(rows=rows)
    m.execute_plan(tmp_path,None,rows,False,report,run_case_fn=stub)
    assert seen==list(CASES[:3])
    assert report['stop_reason']=='repeated_infrastructure_error'
    assert [row['status'] for row in rows]==['error','error','error','not_run','not_run']
    assert rows[3]['reason']=='not_run_after_target3.0_rep1'==rows[4]['reason']


def test_collection_quality_issues_do_not_abort_the_plan(tmp_path):
    from evaluation import gazebo_braking_probe as m
    seen=[]
    issues_by_case=[['drive_command_cadence_violation: failed=max_above_clock_plus_0.02s'],
                    ['world_sampling_above_150ms_or_reversed'],
                    ['drive_duration_not_4s_at_20hz'],
                    ['missing_zero_pose_bracket'],
                    ['world_sampling_above_150ms_or_reversed']]
    def stub(case_dir,world_path,target,rep,diagnostic_only=False):
        seen.append((target,rep))
        return dict(target_m_s=target,rep=rep,status='invalid',valid=False,censored=False,
                    issues=issues_by_case[len(seen)-1])
    rows=_not_run_rows();report=dict(rows=rows)
    m.execute_plan(tmp_path,None,rows,False,report,run_case_fn=stub)
    assert seen==list(CASES) and 'stop_reason' not in report
    assert [row['status'] for row in rows]==['invalid']*len(CASES)
    assert [row['issues'] for row in rows]==issues_by_case


def test_valid_case_resets_the_consecutive_error_streak(tmp_path):
    from evaluation import gazebo_braking_probe as m
    seen=[]
    def stub(case_dir,world_path,target,rep,diagnostic_only=False):
        seen.append((target,rep))
        valid=len(seen)==3
        return dict(target_m_s=target,rep=rep,status='valid' if valid else 'error',valid=valid,
                    censored=False,issues=[] if valid else ['runtime_exception: boom'])
    rows=_not_run_rows();report=dict(rows=rows)
    m.execute_plan(tmp_path,None,rows,False,report,run_case_fn=stub)
    assert seen==list(CASES)
    assert [row['status'] for row in rows]==['error','error','valid','error','error']
    assert 'stop_reason' not in report


def test_only_error_status_counts_as_infrastructure_failure():
    from evaluation import gazebo_braking_probe as m
    valid=dict(target_m_s=1.,rep=1,status='valid',valid=True,censored=False,issues=[])
    not_stopped=dict(target_m_s=3.,rep=1,status='invalid',valid=False,censored=True,
                     issues=['not_stopped_within_3s'])
    slow=dict(target_m_s=3.,rep=1,status='invalid',valid=False,censored=False,
              issues=['prebrake_below_80_percent_target'])
    cadence=dict(target_m_s=3.,rep=1,status='invalid',valid=False,censored=False,
                 issues=['drive_command_cadence_violation: failed=mean_above_0.075s'])
    sparse=dict(target_m_s=3.,rep=1,status='invalid',valid=False,censored=False,
                issues=['world_sampling_above_150ms_or_reversed'])
    missing=dict(target_m_s=3.,rep=1,status='invalid',valid=False,censored=True,
                 issues=['missing_world'])
    error=dict(target_m_s=3.,rep=1,status='error',valid=False,censored=False,
               issues=['runtime_exception: wall_or_clock_stall_timeout'])
    for row in (valid,not_stopped,slow,cadence,sparse,missing):
        assert not m.infrastructure_failure(row)
    assert m.infrastructure_failure(error)
    assert m.plan_abort([cadence,sparse,missing]) is None
    assert m.plan_abort([not_stopped]*3) is None
    assert m.plan_abort([error,cadence,error]) is None
    assert m.plan_abort([error,error,error])=='repeated_infrastructure_error'


def test_recorder_observation_and_publisher_sequences_separate(tmp_path):
    p=tmp_path/'events.jsonl';rec=Recorder(p)
    rec.add(CLOCK,dict(clock_ns=9),node_ns=10,clock_ns=9)
    rec.add('published_cmd',{},node_ns=11,clock_ns=9,pub_seq=0)
    rec.close();rows=[json.loads(s) for s in p.read_text().splitlines()]
    assert [e['seq'] for e in rows]==[0,1]
    assert rows[1]['pub_seq']==0 and 'pub_seq' not in rows[0]
    assert rows[0]['header_stamp_ns'] is None
    assert rows[0]['latest_clock_ns']!=rows[0]['node_ros_time_ns']


def test_cleanup_live_rejects_pid_reuse_and_zombie(monkeypatch):
    from evaluation import gazebo_braking_probe as m
    owner=object.__new__(OwnedProcesses);owner.root=10;owner.known={10:'old',20:'same',30:'zombie'}
    monkeypatch.setattr(m,'proc_table',lambda:{10:dict(start='new',state='S'),
        20:dict(start='same',state='S'),30:dict(start='zombie',state='Z')})
    assert owner.live()==[20]


def test_process_tree_does_not_claim_shared_peer(monkeypatch):
    from evaluation import gazebo_braking_probe as m
    table={10:dict(start='a',ppid=1,state='S'),20:dict(start='b',ppid=10,state='S'),
           30:dict(start='c',ppid=20,state='S'),99:dict(start='peer',ppid=1,state='S')}
    monkeypatch.setattr(m,'proc_table',lambda:table)
    owner=OwnedProcesses(10)
    assert owner.known=={10:'a',20:'b',30:'c'} and 99 not in owner.live()
@pytest.mark.parametrize('name,ns',[('', '/'),('_NODE_NAME_UNKNOWN_','_NODE_NAMESPACE_UNKNOWN_'),('sim_wiring','')])
def test_unknown_reference_never_authorizes_drive(name,ns):
    from evaluation.gazebo_braking_probe import reference_identity
    assert not reference_identity([dict(node_name=name,node_namespace=ns,gid='a')])

@pytest.mark.parametrize('refs',[
    [dict(node_name='other',node_namespace='/',gid='a')],
    [dict(node_name='sim_wiring',node_namespace='/other',gid='a')],
    [dict(node_name='sim_wiring',node_namespace='/',gid='a')]*2])
def test_external_duplicate_reference_still_fails(refs):
    from evaluation.gazebo_braking_probe import reference_identity
    with pytest.raises(RuntimeError):reference_identity(refs)

def test_unknown_gid_resolves_without_whitelisting():
    from evaluation.gazebo_braking_probe import reference_identity
    assert not reference_identity([dict(node_name='_NODE_NAME_UNKNOWN_',node_namespace='/',gid='abc')])
    assert reference_identity([dict(node_name='sim_wiring',node_namespace='/',gid='abc')])
    assert not reference_identity([])