"""mission_executor：多点导航任务器节点。

职责边界（严格遵守）：
- 只做目标调度：顺序调用 Nav2 标准动作接口 ``/navigate_to_pose``（NavigateToPose）。
- 不发布 /cmd_vel、不复制 Nav2 控制器；/cmd_vel 仍由 Nav2 经主线安全层进入底盘。
- 不改厂家驱动 / AMCL / 地图 / 安全层。
- 退出时取消在途目标，并明确把取消描述为“普通目标取消”，不是硬件急停。
- 默认不循环，循环必须在任务文件显式 loop: true。
- 节点重启后绝不自动恢复未完成任务：只有显式设置 resume_from 参数 + 显式调用
  ``start`` 服务（= 再次确认）才会从检查点继续。

动作生命周期（核心不变量）：
- 一个目标依次经过 waiting_accept -> executing -> （waiting_cancel）-> terminal。
- 取消请求已发送 != 取消已完成：超时/定位失效/用户取消/退出都先请求取消并等待
  动作终态；只有收到终态（result 回调）才收尾或派发下一点。
- 取消被拒 / 丢响应 / 动作服务中途消失 => 进入 fault 故障锁定，禁止派发与新 start，
  并向外输出明确故障状态；车端停止保障仍由主线安全链负责。

定位健康（UNKNOWN/HEALTHY/UNHEALTHY/STALE）：
- 实车入口只有新鲜有效健康证据才允许 start/resume 与逐点派发；start 绝不伪造健康。
- 真实发布者由主线诊断模块/适配节点提供；mock 真值只用于测试。

服务（均 std_srvs/Trigger，节点命名空间下）：
- ~/start   开始任务：resume_from 已设则从检查点恢复，否则从 goal_file 全新开始
- ~/pause   暂停派发（在途目标跑完后停住）
- ~/resume  继续派发（恢复前重审定位健康）
- ~/cancel  取消整段任务（取消在途目标并等待确认后收尾报告）
- ~/status  返回 JSON 状态字符串
"""
import json
import math
import os
import signal
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from rclpy.action import ActionClient
from rclpy.duration import Duration
from action_msgs.msg import GoalStatus
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformListener

from . import engine as eng
from . import report as rep
from . import validate

# action_msgs CancelGoal.Response.return_code 取值
_CANCEL_ERROR_NONE = 0
_CANCEL_ERROR_REJECTED = 1
_CANCEL_ERROR_UNKNOWN_GOAL_ID = 2
_CANCEL_ERROR_GOAL_TERMINATED = 3

_PHASES = ('idle', 'waiting_accept', 'executing', 'waiting_cancel', 'fault')
_LOC_STATES = ('unknown', 'healthy', 'unhealthy', 'stale')


def _serialized(method):
    """保证状态机回调串行执行。

    ActionClient 默认使用独立回调组，其结果/接受回调与节点服务回调可在多线程
    executor 里并发交错；若不串行，`_finalize_goal` 的“暂停判定”可能被 `_cb_resume`
    抢先改状态，导致暂停时误写终态报告。这里用可重入锁把所有状态机回调串行化。
    """

    def wrapper(self, *args, **kwargs):
        with self._serial_lock:
            return method(self, *args, **kwargs)

    wrapper.__name__ = method.__name__
    return wrapper


class MissionExecutor(Node):
    def __init__(self, overrides=None):
        super().__init__('mission_executor')
        self._declare(overrides)
        self._client = ActionClient(self, NavigateToPose, self._nav_action)
        self._tf_buffer = Buffer(node=self)
        self._tf_listener = TransformListener(self._tf_buffer, self)

        self._loc_state = 'unknown'
        self._loc_stamp = None
        self._loc_sub = self.create_subscription(Bool, self._loc_topic, self._on_loc, 10)
        self._report_pub = self.create_publisher(String, '~/report', 10)

        self.create_service(Trigger, '~/start', self._cb_start)
        self.create_service(Trigger, '~/pause', self._cb_pause)
        self.create_service(Trigger, '~/resume', self._cb_resume)
        self.create_service(Trigger, '~/cancel', self._cb_cancel)
        self.create_service(Trigger, '~/status', self._cb_status)

        # 运行时状态
        self._serial_lock = threading.RLock()
        self.engine = None
        self._current_goal_file = ''
        self._mission_finished = False
        self._active_goal = None
        self._active_gen = 0
        self._goal_handle = None
        self._goal_send_time = None
        self._goal_done = False
        self._goal_accepted = False
        self._phase = 'idle'
        self._pending = None              # {'kind': user_cancel|timeout|loc_lost|shutdown}
        self._cancel_reason = None
        self._cancel_request_time = None
        self._cancel_timeout_timer = None
        self._fault_reason = None
        self._recoveries = 0
        self._last_feedback_pose = None
        self._timeout_timer = None
        self._loc_watchdog_timer = None

        self.get_logger().info('mission_executor ready (nav action=%s, goal_file=%r)'
                               % (self._nav_action, self._goal_file_param))

    # ---------------- 参数 ----------------
    def _declare(self, overrides=None):
        self.declare_parameter('goal_file', '')
        self.declare_parameter('checkpoint_file', '')
        self.declare_parameter('resume_from', '')
        self.declare_parameter('report_file', '')
        self.declare_parameter('nav_action', '/navigate_to_pose')
        self.declare_parameter('robot_base_frame', 'base_footprint')
        self.declare_parameter('allowed_frames', ['map', 'odom', 'odom_combined'])
        self.declare_parameter('localization_status_topic', '/localization_ok')
        self.declare_parameter('localization_watchdog', 0.0)
        self.declare_parameter('nav_wait_timeout', 5.0)
        self.declare_parameter('cancel_timeout', 2.0)
        self.declare_parameter('shutdown_wait_timeout', 2.0)
        self.declare_parameter('pose_error_max_age_s', 1.0)
        self.declare_parameter('tf_lookup_timeout', 0.2)
        # use_sim_time 由 rclpy 节点自动声明，无需（也不能）重复 declare。

        if overrides:
            from rclpy.parameter import Parameter
            self.set_parameters([
                Parameter(k, value=v) for k, v in overrides.items()])

        self._nav_action = self.get_parameter('nav_action').value
        self._robot_base_frame = self.get_parameter('robot_base_frame').value
        self._allowed_frames = tuple(self.get_parameter('allowed_frames').value)
        self._loc_topic = self.get_parameter('localization_status_topic').value
        self._loc_watchdog_s = float(self.get_parameter('localization_watchdog').value)
        self._nav_wait_timeout = float(self.get_parameter('nav_wait_timeout').value)
        self._cancel_timeout = float(self.get_parameter('cancel_timeout').value)
        self._shutdown_wait_timeout = float(self.get_parameter('shutdown_wait_timeout').value)
        self._pose_error_max_age_s = float(self.get_parameter('pose_error_max_age_s').value)
        self._tf_lookup_timeout = float(self.get_parameter('tf_lookup_timeout').value)
        self._goal_file_param = self.get_parameter('goal_file').value or ''
        # checkpoint_file = 检查点“输出”路径（写）；resume_from = 检查点“输入”路径（读）。
        self._checkpoint_file_param = self.get_parameter('checkpoint_file').value or ''
        self._resume_from_param = self.get_parameter('resume_from').value or ''
        self._report_file = self.get_parameter('report_file').value or ''

    # ---------------- 服务 ----------------
    @_serialized
    def _cb_start(self, req, resp):
        if self.engine is not None and self.engine.active:
            resp.success = False
            resp.message = 'mission already active (%s); cancel first' % self.engine.state
            return resp
        if self.engine is not None and self.engine.state == 'fault':
            resp.success = False
            resp.message = 'mission in fault state (%s); restart node to recover' % (
                self._fault_reason or 'unknown')
            return resp

        # 定位健康门禁：只有新鲜有效健康证据才允许启动（绝不伪造健康）。
        if not self._loc_healthy():
            resp.success = False
            resp.message = 'localization not healthy (state=%s); refuse to start' % self._loc_state
            return resp

        resume_from = self._resume_from_param.strip()
        if resume_from:
            try:
                with open(resume_from, 'r', encoding='utf-8') as f:
                    doc = json.load(f)
                cfg, goals, records, idx, state = rep.parse_checkpoint(doc, self._allowed_frames)
                if state in ('completed', 'aborted', 'canceled', 'fault'):
                    resp.success = False
                    resp.message = 'checkpoint is already final (%s); nothing to resume' % state
                    return resp
                if not cfg.enabled:
                    resp.success = False
                    resp.message = 'checkpoint mission is disabled (enabled: false)'
                    return resp
                self.engine = eng.MissionEngine(cfg, goals)
                self.engine.restore(records, idx, started_at=self._now())
                self._current_goal_file = doc.get('goal_file', '')
                msg = 'resumed from checkpoint %s at index %d' % (resume_from, idx)
            except Exception as e:
                resp.success = False
                resp.message = 'checkpoint load failed: %s' % e
                return resp
        else:
            gf = self._goal_file_param.strip()
            if not gf:
                resp.success = False
                resp.message = 'no goal_file parameter set (and no resume_from)'
                return resp
            try:
                cfg, goals = validate.load_mission_file(gf, self._allowed_frames)
            except Exception as e:
                resp.success = False
                resp.message = 'mission load failed: %s' % e
                return resp
            if not cfg.enabled:
                resp.success = False
                resp.message = 'mission file is disabled (enabled: false)'
                return resp
            self.engine = eng.MissionEngine(cfg, goals)
            self.engine.start(started_at=self._now())
            self._current_goal_file = gf
            msg = 'started mission from %s (%d goals, mode=%s, loop=%s)' % (
                gf, len(goals), cfg.mode, cfg.loop)

        self._mission_finished = False
        self._phase = 'idle'
        self._fault_reason = None
        self.get_logger().info(msg)
        self._dispatch_next()
        resp.success = True
        resp.message = msg
        return resp

    @_serialized
    def _cb_pause(self, req, resp):
        if self.engine is None or not self.engine.pause():
            resp.success = False
            resp.message = 'cannot pause (no running mission)'
            return resp
        self.get_logger().info('pause requested: in-flight goal will finish, then hold')
        resp.success = True
        resp.message = 'pause requested; in-flight goal will finish, then hold'
        return resp

    @_serialized
    def _cb_resume(self, req, resp):
        if self.engine is None or self.engine.state != 'paused':
            resp.success = False
            resp.message = 'cannot resume (not paused)'
            return resp
        if not self._loc_healthy():
            resp.success = False
            resp.message = 'localization not healthy (state=%s); cannot resume' % self._loc_state
            return resp
        self.engine.resume()
        self.get_logger().info('resumed')
        self._dispatch_next()
        resp.success = True
        resp.message = 'resumed'
        return resp

    @_serialized
    def _cb_cancel(self, req, resp):
        if self.engine is None or not self.engine.active:
            resp.success = False
            resp.message = 'no active mission'
            return resp
        if self._phase == 'fault':
            resp.success = False
            resp.message = 'mission in fault state; restart node to recover'
            return resp
        self._pending = {'kind': 'user_cancel'}
        self._cancel_reason = 'mission canceled by user'
        if self._active_goal is not None and not self._goal_done:
            # 有在途目标：请求取消并等待其终态；等待接受期间取消则记录意图、accept 后主动取消。
            self._phase = 'waiting_cancel'
            if self._goal_handle is not None:
                self._request_cancel(self._goal_handle, self._cancel_reason)
            else:
                self._start_cancel_timeout()
            resp.success = True
            resp.message = 'cancel requested; will finalize when Nav2 acknowledges'
        else:
            self._cancel_timer()
            self.engine.cancel(self._now(), 'mission canceled by user')
            self._finish_mission()
            resp.success = True
            resp.message = 'mission canceled'
        return resp

    @_serialized
    def _cb_status(self, req, resp):
        if self.engine is None:
            resp.success = True
            resp.message = json.dumps({'state': 'idle'})
            return resp
        cur = self.engine.current()
        resp.success = True
        resp.message = json.dumps({
            'state': self.engine.state,
            'phase': self._phase,
            'index': self.engine.index,
            'total': len(self.engine.goals),
            'laps': self.engine.laps,
            'current_goal': cur.name if cur else None,
            'localization': self._loc_state,
            'fault_reason': self._fault_reason,
        })
        return resp

    # ---------------- 派发 ----------------
    def _dispatch_next(self):
        if self.engine is None:
            return
        g = self.engine.current()
        if g is None:
            self._finish_mission()
            return
        self._send_goal(g)

    def _send_goal(self, goal):
        # 逐点派发前重审定位健康。
        if not self._loc_healthy():
            self.get_logger().error('localization not healthy; refusing dispatch of %s' % goal.name)
            self._handle_loc_lost()
            return

        self._active_goal = goal
        self._active_gen += 1
        gen = self._active_gen
        self._goal_done = False
        self._goal_accepted = False
        self._goal_handle = None
        self._goal_send_time = None
        self._recoveries = 0
        self._last_feedback_pose = None
        self._pending = None
        self._cancel_reason = None
        self._phase = 'waiting_accept'
        self._cancel_timer()
        self._start_loc_watchdog()

        msg = NavigateToPose.Goal()
        msg.pose = PoseStamped()
        msg.pose.header.frame_id = goal.frame_id
        msg.pose.header.stamp = self.get_clock().now().to_msg()
        msg.pose.pose.position.x = goal.x
        msg.pose.pose.position.y = goal.y
        msg.pose.pose.orientation.z = math.sin(goal.yaw / 2.0)
        msg.pose.pose.orientation.w = math.cos(goal.yaw / 2.0)
        # 注意：goal.xy_tolerance / goal.yaw_tolerance 只作验收记录，不写进 Nav2 目标。

        if not self._client.wait_for_server(timeout_sec=self._nav_wait_timeout):
            self.get_logger().error('Nav2 action server %s not available' % self._nav_action)
            self._finalize_goal(goal, 'failed', self._now(), reason='Nav2 action server %s unavailable'
                                % self._nav_action)
            return

        self._goal_send_time = self._now()
        fut = self._client.send_goal_async(
            msg, feedback_callback=lambda m, g=gen: self._on_feedback(m, g))
        fut.add_done_callback(lambda f, g=gen: self._on_goal_response(f, g))

    @_serialized
    def _on_goal_response(self, future, gen):
        if gen != self._active_gen:
            # 迟到的旧目标：若已被服务端接受，必须主动取消，禁止放任执行。
            try:
                handle = future.result()
            except Exception:
                return
            if getattr(handle, 'accepted', False):
                self.get_logger().warn('stale goal (gen %d) accepted late; canceling' % gen)
                self._safe_cancel(handle)
            return

        goal = self._active_goal
        try:
            handle = future.result()
        except Exception as e:
            self.get_logger().error('send_goal future failed: %s' % e)
            self._finalize_goal(goal, 'failed', self._now(), reason='send_goal failed: %s' % e)
            return
        if not handle.accepted:
            self.get_logger().error('goal %s rejected by Nav2' % goal.name)
            self._finalize_goal(goal, 'failed', self._now(), reason='goal rejected by Nav2 action server')
            return

        self._goal_handle = handle
        self._goal_accepted = True
        self._goal_send_time = self._now()   # 以接受时刻为发送时间

        result_future = handle.get_result_async()
        result_future.add_done_callback(lambda f, g=gen: self._on_result(f, g))

        if self._pending is not None:
            # 等待接受期间已请求停止：立即取消刚被接受的目标。
            self._phase = 'waiting_cancel'
            self._request_cancel(handle, self._cancel_reason or 'goal stop requested')
            return

        self._phase = 'executing'
        if goal.timeout and goal.timeout > 0:
            self._timeout_timer = self.create_timer(goal.timeout, self._on_timeout)

    @_serialized
    def _on_feedback(self, fb_msg, gen):
        if gen != self._active_gen:
            return
        self._last_feedback_pose = fb_msg.feedback.current_pose
        self._recoveries = fb_msg.feedback.number_of_recoveries

    @_serialized
    def _on_result(self, future, gen):
        if gen != self._active_gen or self._goal_done:
            return
        self._goal_done = True
        self._cancel_timer()
        self._cancel_timeout_cancel()
        goal = self._active_goal
        try:
            result = future.result()
        except Exception as e:
            self.get_logger().error('result future failed: %s' % e)
            self._enter_fault('action server disappeared / result failed: %s' % e)
            return
        status = result.status
        now = self._now()
        pending = self._pending
        self._pending = None

        if pending is not None:
            kind = pending.get('kind')
            if kind == 'user_cancel' or kind == 'shutdown':
                self._finalize_user_cancel(goal, status, now)
            elif kind == 'timeout':
                self._finalize_timeout(goal, status, now)
            elif kind == 'loc_lost':
                self._finalize_loc_lost(goal, status, now)
            else:
                self._finalize_normal(goal, status, now)
            return
        self._finalize_normal(goal, status, now)

    @_serialized
    def _on_timeout(self):
        if self._goal_done or self._active_goal is None:
            return
        goal = self._active_goal
        self._cancel_timer()
        self.get_logger().warn('goal %s timed out after %.1fs' % (goal.name, goal.timeout))
        if self._pending is not None:
            return
        self._pending = {'kind': 'timeout'}
        self._cancel_reason = 'per-goal timeout'
        self._phase = 'waiting_cancel'
        if self._goal_handle is not None:
            self._request_cancel(self._goal_handle, 'per-goal timeout')
        else:
            self._start_cancel_timeout()

    # ---------------- 取消确认 ----------------
    def _request_cancel(self, handle, reason):
        """发出公开的 cancel_goal_async 并等待取消响应与动作终态。"""
        self._cancel_reason = reason
        self._cancel_request_time = self._now()
        try:
            fut = handle.cancel_goal_async()
        except Exception as e:
            self.get_logger().warn('cancel request failed: %s' % e)
            self._enter_fault('cancel request failed: %s' % e)
            return False
        fut.add_done_callback(self._on_cancel_response)
        self._start_cancel_timeout()
        return True

    def _safe_cancel(self, handle):
        """尽力取消一个目标（用于迟到的旧目标），失败仅记录，不进入故障流程。"""
        try:
            handle.cancel_goal_async()
        except Exception as e:
            self.get_logger().warn('stale goal cancel failed: %s' % e)

    @_serialized
    def _on_cancel_response(self, future):
        if self._goal_done or self._phase != 'waiting_cancel':
            return
        try:
            resp = future.result()
        except Exception as e:
            self.get_logger().warn('cancel response failed: %s' % e)
            self._enter_fault('cancel response failed: %s' % e)
            return
        code = getattr(resp, 'return_code', _CANCEL_ERROR_NONE)
        if code == _CANCEL_ERROR_REJECTED:
            self._enter_fault('cancel rejected by action server (return_code=1)')
            return
        if code == _CANCEL_ERROR_UNKNOWN_GOAL_ID:
            self._enter_fault('cancel rejected: unknown goal id (return_code=2)')
            return
        if code == _CANCEL_ERROR_NONE and self._goal_handle is not None:
            # rclpy Humble：cancel 回调拒绝时 return_code 仍为 ERROR_NONE，仅把该目标从
            # goals_canceling 里移除。因此必须再确认本目标确实在“接受取消”列表中，
            # 否则判定为取消被拒（进入故障，而非假装已安全取消）。
            hid = list(self._goal_handle.goal_id.uuid)
            accepted = any(
                list(g.goal_id.uuid) == hid for g in getattr(resp, 'goals_canceling', []) or [])
            if not accepted:
                self._enter_fault('cancel rejected by action server (goal not in goals_canceling)')
                return
        # 接受取消（或目标已终态）：继续等待 result 终态。

    def _start_cancel_timeout(self):
        if not rclpy.ok():
            return
        self._cancel_timeout_cancel()
        self._cancel_timeout_timer = self.create_timer(self._cancel_timeout, self._on_cancel_timeout)

    def _cancel_timeout_cancel(self):
        if self._cancel_timeout_timer is not None:
            self._cancel_timeout_timer.cancel()
            try:
                self._cancel_timeout_timer.destroy()
            except Exception:
                pass
            self._cancel_timeout_timer = None

    @_serialized
    def _on_cancel_timeout(self):
        if self._goal_done or self._phase != 'waiting_cancel':
            return
        self._enter_fault(
            'cancel unconfirmed: goal did not reach terminal within %.1fs' % self._cancel_timeout)

    def _enter_fault(self, reason):
        if self._phase == 'fault':
            return
        self.get_logger().error('FAULT: %s' % reason)
        self._phase = 'fault'
        self._fault_reason = reason
        self._cancel_timer()
        self._cancel_timeout_cancel()
        self._goal_done = True
        self._pending = None
        if self.engine is not None and self.engine.active:
            self.engine.fault(self._now(), reason, active_goal=self._active_goal,
                              accepted=self._goal_accepted, send_time=self._goal_send_time)
            self._mission_finished = False
            self._finish_mission()
        self._publish_fault(reason)

    def _publish_fault(self, reason):
        try:
            m = String()
            m.data = json.dumps({'state': 'fault', 'reason': reason})
            self._report_pub.publish(m)
        except Exception:
            pass

    # ---------------- 定位健康 ----------------
    @_serialized
    def _on_loc(self, msg):
        self._loc_stamp = self.get_clock().now()
        if bool(msg.data):
            self._loc_state = 'healthy'
        else:
            self._loc_state = 'unhealthy'
            self._handle_loc_lost()

    @_serialized
    def _on_loc_watchdog(self):
        if self.engine is None or not self.engine.active or self._phase == 'fault':
            return
        if self._loc_watchdog_s > 0 and self._loc_stamp is not None and (
                self.get_clock().now() - self._loc_stamp).nanoseconds / 1e9 >= self._loc_watchdog_s:
            self._loc_state = 'stale'
            self._handle_loc_lost()

    def _loc_healthy(self):
        """定位健康 = 状态 healthy 且（若开了 watchdog）证据未过期。

        若最后一次健康证据距今已超过 localization_watchdog，即便状态仍是 healthy 也
        判 stale 并拒绝（覆盖 start/resume 之前无 watchdog 定时器运行的窗口）。
        """
        if self._loc_state != 'healthy':
            return False
        if self._loc_watchdog_s > 0 and self._loc_stamp is not None:
            age = (self.get_clock().now() - self._loc_stamp).nanoseconds / 1e9
            if age >= self._loc_watchdog_s:
                self._loc_state = 'stale'
                return False
        return True

    def _handle_loc_lost(self):
        if self.engine is None or not self.engine.active:
            return
        if self._phase in ('waiting_cancel', 'fault'):
            return
        self.get_logger().error('localization lost; stopping mission dispatch')
        if self._active_goal is not None and not self._goal_done:
            self._pending = {'kind': 'loc_lost'}
            self._cancel_reason = 'localization lost'
            self._phase = 'waiting_cancel'
            if self._goal_handle is not None:
                self._request_cancel(self._goal_handle, 'localization lost')
            else:
                self._start_cancel_timeout()
        elif self.engine.state != 'paused':
            # 无在途目标且非暂停：整段取消（running 下定位失效即停）。
            self.engine.cancel(self._now(), 'localization lost')
            self._finish_mission()
        # 暂停且无在途目标：只保留 unhealthy/stale 状态，不整段取消；
        # resume 会在恢复前重审定位健康（_cb_resume -> _loc_healthy）。

    # ---------------- 收尾 ----------------
    def _finalize_goal(self, goal, status, finish_time, force_stop=False, stop_reason=None,
                       force_state=None, reason=None, nav_status=None, nav_error_code=None):
        self._goal_done = True
        self._phase = 'idle'
        self._cancel_timer()
        self._cancel_timeout_cancel()
        fields = self._measure(goal, finish_time, nav_status, nav_error_code, reason)
        next_goal = self.engine.record(
            goal, status, finish_time,
            force_stop=force_stop, stop_reason=stop_reason, force_state=force_state, **fields)
        self.get_logger().info(
            'goal %s(seq %d) -> %s | elapsed=%s s | pos_err=%s m | recoveries=%d' % (
                goal.name, goal.seq, status,
                ('%.2f' % fields['elapsed_s']) if fields['elapsed_s'] is not None else '-',
                ('%.3f' % fields['position_error_m']) if fields['position_error_m'] is not None else '-',
                fields['recoveries']))
        self._write_checkpoint()
        if next_goal is not None:
            self._send_goal(next_goal)
        elif self.engine.state == 'paused':
            self.get_logger().info('paused: holding (in-flight goal finished)')
        else:
            self._finish_mission()

    def _finalize_normal(self, goal, status, now):
        if status == GoalStatus.STATUS_SUCCEEDED:
            self._finalize_goal(goal, 'success', now, nav_status=status)
        elif status == GoalStatus.STATUS_ABORTED:
            self._finalize_goal(goal, 'failed', now, nav_status=status,
                                reason='Nav2 aborted goal')
        elif status == GoalStatus.STATUS_CANCELED:
            self._finalize_goal(goal, 'canceled', now, nav_status=status,
                                reason='goal canceled (not a hardware e-stop)')
        else:
            self._finalize_goal(goal, 'failed', now, nav_status=status,
                                reason='unknown goal status %d' % status)

    def _finalize_user_cancel(self, goal, status, now):
        reason = self._cancel_reason or 'mission canceled by user'
        if status == GoalStatus.STATUS_CANCELED:
            self._finalize_goal(goal, 'canceled', now, force_stop=True, stop_reason=reason,
                                nav_status=status,
                                reason='canceled by user (goal cancel, not a hardware e-stop)')
        elif status == GoalStatus.STATUS_SUCCEEDED:
            self._finalize_goal(goal, 'success', now, force_stop=True, stop_reason=reason,
                                force_state='canceled', nav_status=status,
                                reason='goal succeeded before cancel took effect; mission canceled')
        else:
            self._finalize_goal(goal, 'failed', now, force_stop=True, stop_reason=reason,
                                force_state='canceled', nav_status=status,
                                reason='aborted during cancel')

    def _finalize_timeout(self, goal, status, now):
        if status == GoalStatus.STATUS_CANCELED:
            self._finalize_goal(goal, 'timeout', now, nav_status=status,
                                reason='per-goal timeout (%.1fs)' % goal.timeout)
        elif status == GoalStatus.STATUS_SUCCEEDED:
            self._finalize_goal(goal, 'success', now, nav_status=status,
                                reason='goal succeeded just before timeout cancel')
        else:
            self._finalize_goal(goal, 'failed', now, nav_status=status,
                                reason='aborted after timeout')

    def _finalize_loc_lost(self, goal, status, now):
        self._finalize_goal(goal, 'failed', now, force_stop=True, stop_reason='localization lost',
                            nav_status=status, reason='localization lost')

    def _measure(self, goal, finish_time, nav_status, nav_error_code, reason):
        send_time = self._goal_send_time
        elapsed = (finish_time - send_time) if send_time is not None else None
        err = self._pose_error(goal)
        pos_err = err['pos']
        yaw_err = err['yaw']
        within = None
        if pos_err is not None and (goal.xy_tolerance > 0 or goal.yaw_tolerance > 0):
            ok = True
            if goal.xy_tolerance > 0:
                ok = ok and pos_err <= goal.xy_tolerance
            if goal.yaw_tolerance > 0 and yaw_err is not None:
                ok = ok and yaw_err <= goal.yaw_tolerance
            within = ok
        return dict(send_time=send_time, elapsed_s=elapsed,
                    accepted=self._goal_accepted,
                    position_error_m=pos_err, yaw_error_rad=yaw_err, within_tolerance=within,
                    error_source=err['source'], error_frame=err['frame'],
                    error_sample_time=err['sample_time'], error_age_s=err['age'],
                    recoveries=self._recoveries, nav_status=nav_status,
                    nav_error_code=nav_error_code, reason=reason)

    def _pose_error(self, goal):
        """返回 {'pos','yaw','source','frame','sample_time','age'}。

        优先 TF（goal.frame_id -> robot_base_frame），带新鲜度检查；TF 不可用或过期时
        回退到 Nav2 反馈 current_pose（须同 frame，同样带新鲜度检查）；都不可用/过期
        则 pos/yaw 为 None（误差不可信，报告里明确为 null 并注明来源）。
        """
        now = self._now()
        max_age = self._pose_error_max_age_s
        # 1) TF（带新鲜度检查）
        try:
            t = self._tf_buffer.lookup_transform(
                goal.frame_id, self._robot_base_frame, rclpy.time.Time(),
                timeout=Duration(seconds=self._tf_lookup_timeout))
            sample = t.header.stamp
            sample_time = sample.sec + sample.nanosec * 1e-9
            age = now - sample_time
            if age <= max_age:
                tx = t.transform.translation.x
                ty = t.transform.translation.y
                yaw = self._yaw_from_quat(t.transform.rotation)
                return {'pos': math.hypot(tx - goal.x, ty - goal.y),
                        'yaw': self._wrap(yaw - goal.yaw),
                        'source': 'tf', 'frame': self._robot_base_frame,
                        'sample_time': sample_time, 'age': age}
        except Exception:
            pass
        # 2) feedback 回退（带新鲜度检查）
        fb = self._last_feedback_pose
        if fb is not None and fb.header.frame_id == goal.frame_id:
            st = fb.header.stamp
            sample_time = st.sec + st.nanosec * 1e-9
            age = now - sample_time
            if age <= max_age:
                pos = math.hypot(fb.pose.position.x - goal.x, fb.pose.position.y - goal.y)
                yaw = self._yaw_from_quat(fb.pose.orientation)
                return {'pos': pos, 'yaw': self._wrap(yaw - goal.yaw),
                        'source': 'feedback', 'frame': goal.frame_id,
                        'sample_time': sample_time, 'age': age}
        return {'pos': None, 'yaw': None, 'source': None, 'frame': None,
                'sample_time': None, 'age': None}

    def _finish_mission(self):
        if self.engine is None or self._mission_finished:
            return
        self._mission_finished = True
        self._cancel_timer()
        self._cancel_timeout_cancel()
        self._stop_loc_watchdog()
        if self.engine.finished_at is None:
            self.engine.finished_at = self._now()
        self._write_checkpoint()
        report = rep.build_report(self.engine, goal_file=self._current_goal_file)
        rf = self._report_file.strip() or os.path.join(os.getcwd(), 'mission_report.json')
        try:
            # 临时文件 + 原子替换：报告要么整体可见、要么不存在，避免“文件已建但内容未写完”
            # 时被回归/外部读取到空文件（open('w') 先截断再写，中间有空洞窗口）。
            tmp = rf + '.tmp'
            with open(tmp, 'w', encoding='utf-8') as f:
                json.dump(report, f, indent=2)
            os.replace(tmp, rf)
            self.get_logger().info('report written to %s' % rf)
        except Exception as e:
            self.get_logger().error('failed to write report: %s' % e)
        summary = String()
        summary.data = json.dumps({'state': self.engine.state,
                                   'counts': report['mission']['counts'],
                                   'fault_reason': self._fault_reason})
        try:
            # 汇总发布是 best-effort：节点关闭/上下文销毁期间发布器可能已失效，绝不因此抛
            # 异常传播到 result done-callback（否则会以“never retrieved”Future 异常泄漏）。
            self._report_pub.publish(summary)
        except Exception:
            pass
        self.get_logger().info('mission %s: %s' % (self.engine.state, report['mission']['counts']))

    def _write_checkpoint(self):
        cp = self._checkpoint_file_param.strip()
        if not cp or self.engine is None:
            return
        try:
            doc = rep.build_checkpoint(self.engine, self._current_goal_file, self.engine.index)
            tmp = cp + '.tmp'
            with open(tmp, 'w', encoding='utf-8') as f:
                json.dump(doc, f, indent=2)
            os.replace(tmp, cp)   # 临时文件 + 原子替换
        except Exception as e:
            self.get_logger().warn('checkpoint write failed: %s' % e)

    # ---------------- 工具 ----------------
    def _now(self):
        return self.get_clock().now().nanoseconds / 1e9

    def _cancel_timer(self):
        if self._timeout_timer is not None:
            self._timeout_timer.cancel()
            try:
                self._timeout_timer.destroy()
            except Exception:
                pass
            self._timeout_timer = None

    def _start_loc_watchdog(self):
        if self._loc_watchdog_s <= 0 or self._loc_watchdog_timer is not None:
            return
        # 采样周期取 min(watchdog, 0.5)（下限 0.05），截止时间严格等于 watchdog 参数，
        # 避免“>2*watchdog”式的错误放大与过大的周期采样延迟。
        period = max(0.05, min(self._loc_watchdog_s, 0.5))
        self._loc_watchdog_timer = self.create_timer(period, self._on_loc_watchdog)

    def _stop_loc_watchdog(self):
        if self._loc_watchdog_timer is not None:
            self._loc_watchdog_timer.cancel()
            try:
                self._loc_watchdog_timer.destroy()
            except Exception:
                pass
            self._loc_watchdog_timer = None

    @staticmethod
    def _yaw_from_quat(q):
        return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                          1.0 - 2.0 * (q.y * q.y + q.z * q.z))

    @staticmethod
    def _wrap(a):
        return (a + math.pi) % (2.0 * math.pi) - math.pi

    def _wait_for_terminal(self, timeout):
        """有界等待在途目标终态（shutdown 用）；上下文不可用则立即返回。"""
        if not rclpy.ok():
            return
        deadline = time.time() + timeout
        while time.time() < deadline and not self._goal_done:
            try:
                rclpy.spin_once(self, timeout_sec=0.02)
            except Exception:
                break

    def on_shutdown(self):
        if self._goal_handle is not None and not self._goal_done:
            self.get_logger().info(
                'canceling active navigation goal on shutdown (normal goal cancel, not a hardware e-stop)')
            self._pending = {'kind': 'shutdown'}
            self._cancel_reason = 'node shutdown'
            self._phase = 'waiting_cancel'
            # 上下文已失效时无法再发取消/建定时器，直接走未确认记账，避免 create_timer 崩。
            if rclpy.ok():
                self._request_cancel(self._goal_handle, 'node shutdown')
                self._wait_for_terminal(self._shutdown_wait_timeout)
            if not self._goal_done:
                self._enter_fault('shutdown: cancel unconfirmed (goal may still be active)')
        if self.engine is not None and self.engine.active:
            self.engine.cancel(self._now(), 'node shutdown')
            self._mission_finished = False
            self._finish_mission()


def main(args=None):
    # 关键：禁用 rclpy 默认信号处理（其 SIGINT/SIGTERM 处理器会先 rcl_shutdown 掉上下文，
    # 使 on_shutdown 无法再发取消请求——实测会崩在 create_timer 且服务端收不到取消）。
    # 改为自装处理器：仅置标志，主循环在上下文仍有效的窗口内先跑 on_shutdown（取消在途
    # 目标并等待终态），最后才 rclpy.shutdown()。
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = MissionExecutor()
    stop = threading.Event()

    def _on_signal(signum, frame):
        stop.set()

    signal.signal(signal.SIGINT, _on_signal)
    signal.signal(signal.SIGTERM, _on_signal)

    try:
        while rclpy.ok() and not stop.is_set():
            try:
                # 用全局 executor 手动 spin_once，与 on_shutdown 内部 _wait_for_terminal 的
                # rclpy.spin_once 同一 executor，保证动作回调能被处理。
                rclpy.spin_once(node, timeout_sec=0.1)
            except KeyboardInterrupt:
                break
            except Exception:
                break
    finally:
        node.on_shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
