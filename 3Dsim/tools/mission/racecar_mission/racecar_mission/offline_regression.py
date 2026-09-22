"""离线回归：不依赖 Gazebo / 真 Nav2 / 地图，用 mock NavigateToPose 服务器驱动任务器，
覆盖仿真回归的 8 个场景 + 动作生命周期 / 定位健康 / 取消确认的边界场景，输出
PASS/FAIL 汇总并写 JSON 日志。

运行（WSL 内，需 source ROS2 Humble）：
  ros2 run racecar_mission run_offline_regression
  或  python3 -m racecar_mission.offline_regression

说明：本回归只验证“任务器目标调度 + 动作生命周期 + 定位健康门禁”逻辑；它不能用
Pure Pursuit / 二维真值路线 / 仿真测试驾驶器来冒充 Nav2 多点任务成功——那部分由
主线在 Gazebo+真 Nav2 上另验。服务端断言（mock.events）确认“取消请求确实到达服务端、
目标确实被服务端置为终态”，而不只看任务器本地报告状态。
"""
import json
import os
import sys
import time
import tempfile

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_srvs.srv import Trigger
from std_msgs.msg import Bool

from .executor_node import MissionExecutor
from .mock_nav2 import MockNav2

TERMINAL = ('completed', 'aborted', 'canceled', 'fault')


def _config(name):
    try:
        from ament_index_python.packages import get_package_share_directory
        base = os.path.join(get_package_share_directory('racecar_mission'), 'config')
    except Exception:
        base = os.path.join(os.path.dirname(__file__), '..', 'config')
    return os.path.join(base, name)


def _write_mission(path, timeout):
    txt = (
        'mission:\n'
        '  frame_id: map\n'
        '  mode: multi\n'
        '  loop: false\n'
        '  on_failure: stop\n'
        '  default:\n'
        '    timeout: %.2f\n'
        '    xy_tolerance: 0.1\n'
        '    yaw_tolerance: 0.1\n'
        '  goals:\n'
        '    - {name: p1, seq: 1, x: 1.0, y: 0.0, yaw: 0.0}\n'
        '    - {name: p2, seq: 2, x: 2.0, y: 0.5, yaw: 0.7854}\n'
        '    - {name: p3, seq: 3, x: 3.0, y: 0.0, yaw: 0.0}\n'
    ) % timeout
    with open(path, 'w', encoding='utf-8') as f:
        f.write(txt)


def _write_mission_skip(path, timeout):
    """on_failure=skip 的三点任务（测 timeout+skip：旧目标终态确认后才派发下一点）。"""
    txt = (
        'mission:\n'
        '  frame_id: map\n'
        '  mode: multi\n'
        '  loop: false\n'
        '  on_failure: skip\n'
        '  default:\n'
        '    timeout: %.2f\n'
        '    xy_tolerance: 0.1\n'
        '    yaw_tolerance: 0.1\n'
        '  goals:\n'
        '    - {name: p1, seq: 1, x: 1.0, y: 0.0, yaw: 0.0}\n'
        '    - {name: p2, seq: 2, x: 2.0, y: 0.5, yaw: 0.7854}\n'
        '    - {name: p3, seq: 3, x: 3.0, y: 0.0, yaw: 0.0}\n'
    ) % timeout
    with open(path, 'w', encoding='utf-8') as f:
        f.write(txt)


def _write_mission_single(path, timeout):
    """单个目标、带短 per-goal timeout（测动作服务消失 -> timeout 兜底 -> fault）。"""
    txt = (
        'mission:\n'
        '  frame_id: map\n'
        '  mode: single\n'
        '  loop: false\n'
        '  on_failure: stop\n'
        '  default:\n'
        '    timeout: %.2f\n'
        '    xy_tolerance: 0.1\n'
        '    yaw_tolerance: 0.1\n'
        '  goals:\n'
        '    - {name: p1, seq: 1, x: 1.0, y: 0.0, yaw: 0.0}\n'
    ) % timeout
    with open(path, 'w', encoding='utf-8') as f:
        f.write(txt)


class Driver(Node):
    def __init__(self):
        super().__init__('mission_driver')
        self.start = self.create_client(Trigger, '/mission_executor/start')
        self.pause = self.create_client(Trigger, '/mission_executor/pause')
        self.resume = self.create_client(Trigger, '/mission_executor/resume')
        self.cancel = self.create_client(Trigger, '/mission_executor/cancel')
        self.status = self.create_client(Trigger, '/mission_executor/status')
        self.loc = self.create_publisher(Bool, '/localization_ok', 10)


def _call(client, spin, timeout=10.0):
    deadline = time.time() + timeout
    while time.time() < deadline and not client.service_is_ready():
        spin()
    fut = client.call_async(Trigger.Request())
    deadline = time.time() + timeout
    while not fut.done() and time.time() < deadline:
        spin()
    if not fut.done():
        raise TimeoutError('service call timed out')
    return fut.result()


def _cancel_after_active(ex, drv, spin, report_file=None):
    deadline = time.time() + 5.0
    while time.time() < deadline and (ex._goal_handle is None or ex._goal_done):
        spin()
    _call(drv.cancel, spin)


def _cancel_during_wait_accept(ex, drv, spin, report_file=None):
    # 在“等待接受期间”（目标已发出、尚未被服务端接受）请求取消。
    deadline = time.time() + 5.0
    while time.time() < deadline and not (
            ex._active_goal is not None and ex._goal_handle is None
            and ex._phase == 'waiting_accept'):
        spin()
    _call(drv.cancel, spin)


def _loc_flip_after_first(ex, drv, spin, report_file=None):
    # 等第二个目标真正派发后（而非仅 index 推进）再发定位失效，消除竞态。
    deadline = time.time() + 5.0
    while time.time() < deadline and not (
            ex._active_goal is not None and ex._active_goal.seq == 2):
        spin()
    drv.loc.publish(Bool(data=False))


def _cancel_then_probe_fault(ex, drv, spin, report_file=None):
    # 等在途目标被接受，请求取消；等 fault 后探测 start/resume 均被拒。
    deadline = time.time() + 5.0
    while time.time() < deadline and not (ex._goal_handle is not None and not ex._goal_done):
        spin()
    _call(drv.cancel, spin)
    deadline = time.time() + 5.0
    while time.time() < deadline and not (
            ex.engine is not None and ex.engine.state == 'fault'):
        spin()
    start_resp = _call(drv.start, spin)
    resume_resp = _call(drv.resume, spin)
    return {'post_fault_start_ok': start_resp.success,
            'post_fault_start_msg': start_resp.message,
            'post_fault_resume_ok': resume_resp.success,
            'post_fault_resume_msg': resume_resp.message}


def _loc_setup_false(ex, drv, spin):
    # 发布定位 False，制造 unhealthy，供 start 拒绝门禁测试。
    drv.loc.publish(Bool(data=False))
    deadline = time.time() + 5.0
    while time.time() < deadline and ex._loc_state != 'unhealthy':
        spin()


def _loc_setup_stale(ex, drv, spin):
    # 先发布 True（healthy），再睡过 watchdog 让证据过期，供 start 拒绝（stale）测试。
    drv.loc.publish(Bool(data=True))
    deadline = time.time() + 5.0
    while time.time() < deadline and not ex._loc_healthy():
        spin()
    time.sleep(ex._loc_watchdog_s + 0.3)


def _pause_resume_rejected_then_recover(ex, drv, spin, report_file=None):
    # 暂停后定位变 False → resume 拒绝；恢复健康 → resume 成功，任务继续完成。
    deadline = time.time() + 5.0
    while time.time() < deadline and not (
            ex._active_goal is not None and not ex._goal_done
            and ex.engine is not None and ex.engine.index == 0):
        spin()
    _call(drv.pause, spin)
    deadline = time.time() + 5.0
    while time.time() < deadline and not (
            ex.engine is not None and ex.engine.state == 'paused'):
        spin()
    drv.loc.publish(Bool(data=False))
    deadline = time.time() + 5.0
    while time.time() < deadline and ex._loc_state != 'unhealthy':
        spin()
    rej = _call(drv.resume, spin)
    drv.loc.publish(Bool(data=True))
    deadline = time.time() + 5.0
    while time.time() < deadline and not ex._loc_healthy():
        spin()
    ok = _call(drv.resume, spin)
    return {'resume_after_unhealthy_ok': rej.success,
            'resume_after_healthy_ok': ok.success}


def _check_fault_report(detail):
    """故障报告须区分「调度已锁定」(state=fault) 与「旧目标结局未确认」(unconfirmed)。"""
    ms = detail.get('mission_state')
    gs = detail.get('goal_statuses') or []
    sr = detail.get('stop_reason') or ''
    if ms != 'fault':
        return False, 'mission_state=%r (want fault)' % ms
    if not gs or gs[0] != 'unconfirmed':
        return False, 'goal_statuses=%r (want first=unconfirmed)' % gs
    if 'cancel rejected' not in sr and 'cancel unconfirmed' not in sr \
            and 'disappeared' not in sr:
        return False, 'stop_reason=%r missing fault marker' % sr
    return True, 'fault report: state=fault + active goal unconfirmed + stop_reason=%r' % sr


def _check_timeout_skip_order(detail):
    """timeout+skip：旧目标(g1)终态确认(canceled)必须先于下一目标(g2)发出。"""
    events = detail.get('events') or []
    c1 = g2 = None
    for i, e in enumerate(events):
        if e == ('canceled', 1) and c1 is None:
            c1 = i
        if e[0] == 'goal_received' and e[1] == 2 and g2 is None:
            g2 = i
    if c1 is not None and g2 is not None and c1 < g2:
        return True, 'g1 canceled @%d before g2 received @%d' % (c1, g2)
    return False, 'order violated: g1 canceled=%s, g2 received=%s' % (c1, g2)


def _pause_resume(ex, drv, spin, report_file=None):
    # 在 g1 在途时暂停；等其完成并停住（paused）后检查是否误写终态报告，再恢复。
    deadline = time.time() + 5.0
    while time.time() < deadline and not (
            ex._active_goal is not None and not ex._goal_done
            and ex.engine is not None and ex.engine.index == 0):
        spin()
    _call(drv.pause, spin)
    deadline = time.time() + 5.0
    while time.time() < deadline and not (
            ex.engine is not None and ex.engine.state == 'paused'):
        spin()
    pause_wrote_report = os.path.exists(report_file)
    pause_state = ex.engine.state if ex.engine else None
    _call(drv.resume, spin)
    return {'pause_wrote_report': pause_wrote_report, 'pause_state': pause_state}


def run_case(name, goal_file, report_file, expect_state=None, expect_counts=None,
             mock=None, exec_overrides=None, action_after=None, expect_start=None,
             expect_events=None, extra_expect=None, healthy_loc=True,
             loc_setup=None, forbid_events=None, check=None):
    ov = dict(exec_overrides or {})
    ov['goal_file'] = goal_file
    ov['report_file'] = report_file
    ex = MissionExecutor(overrides=ov)
    drv = Driver()
    handler = MultiThreadedExecutor()
    handler.add_node(ex)
    handler.add_node(drv)
    if mock is not None:
        handler.add_node(mock)

    def spin():
        handler.spin_once(timeout_sec=0.02)

    # 等 start 服务就绪
    deadline = time.time() + 5.0
    while time.time() < deadline and not drv.start.service_is_ready():
        spin()
    # 等动作服务器就绪（若有 mock）
    if mock is not None:
        deadline = time.time() + 5.0
        while time.time() < deadline and not ex._client.server_is_ready():
            spin()

    # 定位健康：默认先发布 True 并等执行器接收，保证 start 健康门禁确定性通过。
    if loc_setup is not None:
        loc_setup(ex, drv, spin)
    elif healthy_loc:
        drv.loc.publish(Bool(data=True))
        deadline = time.time() + 5.0
        while time.time() < deadline and not ex._loc_healthy():
            spin()

    resp = _call(drv.start, spin)
    start_ok = resp.success
    start_msg = resp.message

    extra = {}
    if action_after is not None and start_ok:
        r = action_after(ex, drv, spin, report_file)
        if isinstance(r, dict):
            extra = r

    if start_ok:
        deadline = time.time() + 20.0
        while time.time() < deadline:
            spin()
            if ex.engine is not None and ex.engine.state in TERMINAL:
                break

        # 引擎状态在 executor 线程上先置为终态，收尾（写报告）在其后几步才落盘，
        # 补几轮 spin 等报告文件出现，避免读到还没写完的报告。
        report_deadline = time.time() + 3.0
        while time.time() < report_deadline and not os.path.exists(report_file):
            spin()

    state = ex.engine.state if ex.engine else None
    report = None
    # 报告由 _finish_mission 原子落盘（tmp + os.replace），文件存在即内容完整；这里再
    # 兜底重试解析，防止极窄的“存在但未写完”竞态把回归本身带崩（而非场景失败）。
    if os.path.exists(report_file):
        read_deadline = time.time() + 2.0
        while time.time() < read_deadline:
            try:
                with open(report_file, 'r', encoding='utf-8') as f:
                    report = json.load(f)
                break
            except Exception:
                spin()
                time.sleep(0.02)
    counts = report['mission']['counts'] if report else None
    events = list(mock.events) if mock is not None else None

    # 清理（先 stop mock 后台执行线程，再销毁）
    ex.on_shutdown()
    if mock is not None:
        mock.stop()
    handler.remove_node(ex)
    handler.remove_node(drv)
    if mock is not None:
        handler.remove_node(mock)
    ex.destroy_node()
    drv.destroy_node()
    if mock is not None:
        mock.destroy_node()
    handler.shutdown()

    ok = True
    detail = {'name': name, 'start_ok': start_ok, 'start_msg': start_msg,
              'state': state, 'counts': counts, 'events': events,
              'goal_received_count': sum(1 for e in (events or []) if e[0] == 'goal_received'),
              'mission_state': report['mission']['state'] if report else None,
              'stop_reason': report['mission']['stop_reason'] if report else None,
              'goal_statuses': [g['status'] for g in report['goals']] if report else None}
    detail.update(extra)
    if expect_state is not None:
        ok = ok and (state == expect_state)
        detail['expect_state'] = expect_state
    if expect_counts is not None:
        detail['expect_counts'] = expect_counts
        for k, v in expect_counts.items():
            ok = ok and (counts is not None and counts.get(k, 0) == v)
    if expect_start is not None:
        detail['expect_start'] = expect_start
        ok = ok and (start_ok == expect_start)
    if expect_events is not None:
        detail['expect_events'] = expect_events
        present = set(e[0] for e in (events or []))
        ok = ok and all(e in present for e in expect_events)
    if forbid_events is not None:
        detail['forbid_events'] = forbid_events
        present = set(e[0] for e in (events or []))
        ok = ok and not any(e in present for e in forbid_events)
    if extra_expect is not None:
        detail['extra_expect'] = extra_expect
        for k, v in extra_expect.items():
            ok = ok and (detail.get(k) == v)
    if check is not None:
        c_ok, c_msg = check(detail)
        ok = ok and c_ok
        detail['check'] = c_msg
    return ok, detail


def main(argv=None):
    rclpy.init(args=sys.argv[1:] if argv is None else argv)
    tmpdir = tempfile.mkdtemp(prefix='racecar_mission_reg_')
    results = []
    try:
        timeout_mission = os.path.join(tmpdir, 'mission_timeout.yaml')
        _write_mission(timeout_mission, 0.5)
        timeout_mission_skip = os.path.join(tmpdir, 'mission_timeout_skip.yaml')
        _write_mission_skip(timeout_mission_skip, 0.5)
        timeout_mission_single = os.path.join(tmpdir, 'mission_timeout_single.yaml')
        _write_mission_single(timeout_mission_single, 0.3)

        cases = [
            # 1 单个直道目标
            dict(name='1_single_straight',
                 goal=_config('mission_straight.yaml'),
                 mock=dict(behavior='success', delay=0.1),
                 expect_state='completed', expect_counts=dict(success=1)),
            # 2 单个含转弯目标
            dict(name='2_single_turn',
                 goal=_config('mission_turn.yaml'),
                 mock=dict(behavior='success', delay=0.1),
                 expect_state='completed', expect_counts=dict(success=1)),
            # 3 三点短序列
            dict(name='3_three_point',
                 goal=_config('mission_3point.yaml'),
                 mock=dict(behavior='success', delay=0.1),
                 expect_state='completed', expect_counts=dict(success=3)),
            # 4 十一点整圈（仅回归，如实报告）
            dict(name='4_eleven_lap',
                 goal=_config('mission_11point_lap.yaml'),
                 mock=dict(behavior='success', delay=0.05),
                 expect_state='completed', expect_counts=dict(success=11)),
            # 5 取消任务（服务端确实收到取消并置为 CANCELED）
            dict(name='5_cancel',
                 goal=_config('mission_3point.yaml'),
                 mock=dict(behavior='never', delay=0.1),
                 action_after=_cancel_after_active,
                 expect_state='canceled', expect_counts=dict(canceled=1, not_attempted=2),
                 expect_events=['goal_received', 'accepted', 'cancel_received', 'canceled']),
            # 6 不可达目标
            dict(name='6_unreachable',
                 goal=_config('mission_3point.yaml'),
                 mock=dict(behavior='abort', delay=0.1),
                 expect_state='aborted', expect_counts=dict(failed=1, not_attempted=2)),
            # 7 Nav2 未启动 / 动作服务不存在
            dict(name='7_no_server',
                 goal=_config('mission_straight.yaml'),
                 mock=None,
                 exec_overrides=dict(nav_wait_timeout=0.5,
                                     nav_action='/navigate_to_pose_missing'),
                 expect_state='aborted', expect_counts=dict(failed=1)),
            # 8 中途定位失效停止继续派发
            dict(name='8_localization_lost',
                 goal=_config('mission_3point.yaml'),
                 mock=dict(behavior='success,never', delay=0.1),
                 action_after=_loc_flip_after_first,
                 expect_state='aborted', expect_counts=dict(success=1, failed=1, not_attempted=1)),
            # ---- 动作生命周期边界 ----
            # 9 等待接受期间取消：取消意图被记录，迟到的 accept 到达后被主动取消。
            dict(name='9_cancel_during_wait_accept',
                 goal=_config('mission_3point.yaml'),
                 mock=dict(behavior='never', delay=0.1, accept_delay=0.8),
                 action_after=_cancel_during_wait_accept,
                 expect_state='canceled', expect_counts=dict(canceled=1, not_attempted=2),
                 expect_events=['goal_received', 'accepted', 'cancel_received', 'canceled']),
            # 10 单点超时：取消请求发到服务端、确认终态后按 timeout 记账（不丢下一点）。
            dict(name='10_timeout_cancel_confirmed',
                 goal=timeout_mission,
                 mock=dict(behavior='never', delay=0.1),
                 expect_state='aborted', expect_counts=dict(timeout=1, not_attempted=2),
                 expect_events=['goal_received', 'accepted', 'cancel_received', 'canceled']),
            # 11 取消被服务端拒绝 -> 故障锁定（不伪装成已安全取消）；fault 后 start/resume 均拒绝，
            #    且服务端未收到后续目标；报告区分「调度已锁定」与「旧目标结局未确认」。
            dict(name='11_cancel_rejected_fault',
                 goal=_config('mission_3point.yaml'),
                 mock=dict(behavior='never', delay=0.1, reject_cancel=True),
                 action_after=_cancel_then_probe_fault,
                 expect_state='fault', expect_counts=dict(unconfirmed=1, not_attempted=2),
                 expect_events=['goal_received', 'accepted', 'cancel_received'],
                 forbid_events=['canceled'],
                 extra_expect=dict(post_fault_start_ok=False, post_fault_resume_ok=False,
                                  goal_received_count=1),
                 check=_check_fault_report),
            # 12 定位 UNKNOWN（未收到任何健康证据）拒绝启动（start 绝不伪造健康）。
            dict(name='12_start_refuses_unknown_loc',
                 goal=_config('mission_straight.yaml'),
                 mock=dict(behavior='success', delay=0.1),
                 healthy_loc=False,
                 expect_start=False, expect_state=None),
            # 13 暂停不写终态报告，恢复后继续完成。
            dict(name='13_pause_resume_no_report',
                 goal=_config('mission_3point.yaml'),
                 mock=dict(behavior='success', delay=0.1),
                 action_after=_pause_resume,
                 expect_state='completed', expect_counts=dict(success=3),
                 extra_expect=dict(pause_wrote_report=False, pause_state='paused')),
            # ---- 补齐边界证据（返工回执追加）----
            # 14 timeout + skip：旧目标终态确认（canceled）后才派发下一点，随后两点成功。
            dict(name='14_timeout_skip',
                 goal=timeout_mission_skip,
                 mock=dict(behavior='never,success,success', delay=0.1),
                 expect_state='completed', expect_counts=dict(timeout=1, success=2),
                 expect_events=['goal_received', 'accepted', 'cancel_received', 'canceled',
                                'succeeded'],
                 check=_check_timeout_skip_order),
            # 15 取消被接受但终态迟迟不返回 -> cancel 超时 fault（unconfirmed，不谎称已停）。
            dict(name='15_cancel_stall_fault',
                 goal=_config('mission_3point.yaml'),
                 mock=dict(behavior='cancel_stall', delay=0.1),
                 exec_overrides=dict(cancel_timeout=0.5),
                 action_after=_cancel_after_active,
                 expect_state='fault', expect_counts=dict(unconfirmed=1, not_attempted=2),
                 expect_events=['goal_received', 'accepted', 'cancel_received'],
                 forbid_events=['canceled'],
                 check=_check_fault_report),
            # 16 动作服务执行中消失（销毁 action server，result 未来挂起）-> per-goal
            #    timeout 兜底 -> fault（结局未确认 unconfirmed，不谎称已停/已结束）。
            dict(name='16_action_server_dies',
                 goal=timeout_mission_single,
                 mock=dict(behavior='die', delay=0.1),
                 exec_overrides=dict(cancel_timeout=0.5),
                 expect_state='fault', expect_counts=dict(unconfirmed=1),
                 expect_events=['goal_received', 'accepted', 'died'],
                 check=_check_fault_report),
            # 17 外部 CANCELED（客户端未请求取消）-> 整段停止，剩余 not_attempted。
            dict(name='17_external_canceled',
                 goal=_config('mission_3point.yaml'),
                 mock=dict(behavior='cancel_externally', delay=0.1),
                 expect_state='canceled', expect_counts=dict(canceled=1, not_attempted=2),
                 expect_events=['goal_received', 'accepted', 'canceled'],
                 forbid_events=['cancel_received']),
            # 18 定位 False（unhealthy）拒绝启动。
            dict(name='18_start_refuses_false_loc',
                 goal=_config('mission_straight.yaml'),
                 mock=dict(behavior='success', delay=0.1),
                 loc_setup=_loc_setup_false,
                 expect_start=False, expect_state=None),
            # 19 定位证据过期（stale）拒绝启动（start 门禁做新鲜度检查）。
            dict(name='19_start_refuses_stale_loc',
                 goal=_config('mission_straight.yaml'),
                 mock=dict(behavior='success', delay=0.1),
                 exec_overrides=dict(localization_watchdog=0.5),
                 loc_setup=_loc_setup_stale,
                 expect_start=False, expect_state=None),
            # 20 暂停后定位变 False -> resume 拒绝；恢复健康 -> resume 成功，任务完成。
            dict(name='20_resume_rechecks_loc',
                 goal=_config('mission_3point.yaml'),
                 mock=dict(behavior='success', delay=0.1),
                 action_after=_pause_resume_rejected_then_recover,
                 expect_state='completed', expect_counts=dict(success=3),
                 extra_expect=dict(resume_after_unhealthy_ok=False,
                                  resume_after_healthy_ok=True)),
            # 21 取消请求无响应（服务端收到取消但不回 ACCEPT/REJECT）-> cancel 超时 fault
            #    （结局未确认 unconfirmed，不谎称已停/已结束）。
            dict(name='21_cancel_no_response_fault',
                 goal=_config('mission_3point.yaml'),
                 mock=dict(behavior='never', delay=0.1, no_cancel_response=True),
                 exec_overrides=dict(cancel_timeout=0.5),
                 action_after=_cancel_after_active,
                 expect_state='fault', expect_counts=dict(unconfirmed=1, not_attempted=2),
                 expect_events=['goal_received', 'accepted', 'cancel_received'],
                 forbid_events=['canceled'],
                 check=_check_fault_report),
        ]

        for c in cases:
            report_file = os.path.join(tmpdir, c['name'] + '.json')
            mock = MockNav2(overrides=c['mock']) if c.get('mock') is not None else None
            ok, detail = run_case(
                c['name'], c['goal'], report_file,
                expect_state=c.get('expect_state'), expect_counts=c.get('expect_counts'),
                mock=mock, exec_overrides=c.get('exec_overrides'),
                action_after=c.get('action_after'), expect_start=c.get('expect_start'),
                expect_events=c.get('expect_events'), extra_expect=c.get('extra_expect'),
                healthy_loc=c.get('healthy_loc', True),
                loc_setup=c.get('loc_setup'), forbid_events=c.get('forbid_events'),
                check=c.get('check'))
            detail['ok'] = ok
            results.append(detail)
            extra = detail.get('check') or ''
            if detail.get('extra_expect'):
                extra = (extra + ' | ' + str(detail['extra_expect'])).strip(' |')
            print('[%s] %s  state=%s counts=%s events=%s %s' % (
                'PASS' if ok else 'FAIL', c['name'], detail['state'], detail['counts'],
                detail['events'], extra))

        npass = sum(1 for r in results if r['ok'])
        print('=' * 60)
        print('offline regression: %d/%d passed' % (npass, len(results)))
        log_path = os.path.join(tmpdir, 'summary.json')
        with open(log_path, 'w', encoding='utf-8') as f:
            json.dump({'results': results, 'passed': npass, 'total': len(results)}, f, indent=2)
        print('log dir: %s' % tmpdir)
        return 0 if npass == len(results) else 1
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
