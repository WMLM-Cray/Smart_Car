"""MissionEngine：多点导航任务的状态机。纯 Python，无 ROS/线程/定时器。

职责边界：只做“目标调度与结果记账”，不产生 /cmd_vel、不复制 Nav2 控制器。
由 executor_node 驱动：node 负责动作客户端、定时器、TF 量测与时钟，把每个目标的
最终状态、时间戳、位姿误差等通过 ``record`` 交给本引擎；引擎决定下一个目标是谁，
以及失败后的停止/跳过、暂停/继续、循环与整段收尾。

状态机::

    idle -> running -> (completed | aborted | canceled | fault)
                     -> paused -> running

字段语义：
- index 始终指向“下一个待派发目标”的下标（len(goals) 表示已全部派发）。
- record() 返回下一个要派发的 Goal 或 None；None 既可能是“整段收尾”也可能是
  “暂停”，调用方必须按 self.state 区分（paused 不是终态）。
- canceled 是终态：无论是否本任务器主动，收到 CANCELED 结果都终止整段（特殊
  “跳过被取消点”策略必须在调用方显式定义，本引擎默认终止）。
- fault 是故障锁定终态：取消未确认 / 动作服务消失等；禁止后续派发与新 start。
"""
from .goal import GOAL_STATUSES

STATUS_BAD = ('failed', 'timeout')   # 视为“该点失败”，触发 on_failure 策略
TERMINAL = ('completed', 'aborted', 'canceled', 'fault')


class MissionEngine:
    def __init__(self, config, goals):
        self.config = config
        self.goals = list(goals)          # 调用方保证按 seq 升序
        self.state = 'idle'
        self.index = 0                    # 下一个待派发目标下标
        self.records = []                 # 每个目标一条记录（含 not_attempted）
        self.laps = 0                     # 循环模式下已完成的圈数
        self.started_at = None
        self.finished_at = None
        self.stop_reason = None
        self._pause_requested = False

    # ---- 启动 / 恢复 ----
    def start(self, started_at=None):
        """开始（或重新开始）一条任务，返回第一个待派发 Goal（可能为 None）。"""
        self.state = 'running'
        self.index = 0
        self.records = []
        self.laps = 0
        self.started_at = started_at
        self.finished_at = None
        self.stop_reason = None
        self._pause_requested = False
        return self.current()

    def restore(self, records, index, started_at=None):
        """从检查点显式恢复（绝不自动调用）：还原已完成记录与下一目标下标。"""
        self.state = 'running'
        self.records = list(records)
        self.index = index
        self.laps = 0
        self.started_at = started_at
        self.finished_at = None
        self.stop_reason = None
        self._pause_requested = False
        return self.current()

    # ---- 查询 ----
    def current(self):
        """当前待派发目标；不在 running 状态或已派发完则返回 None。"""
        if self.state != 'running' or not (0 <= self.index < len(self.goals)):
            return None
        return self.goals[self.index]

    @property
    def active(self):
        return self.state in ('running', 'paused')

    # ---- 目标结果 ----
    def record(self, goal, status, finish_time, force_stop=False, stop_reason=None,
               force_state=None, **fields):
        """记录一个目标的最终状态，应用失败策略并推进。

        status 取值见 GOAL_STATUSES（success/failed/timeout/canceled）。
        force_stop=True 表示无条件停止后续派发（任务取消 / 定位失效）。
        force_state 用于在终态时显式指定整段状态（例如“成功但用户取消”记 canceled）。
        返回下一个待派发 Goal 或 None（已收尾或暂停，按 self.state 区分）。
        """
        rec = self._new_record(goal, status, **fields)
        rec['finish_time'] = finish_time
        self.records.append(rec)

        # 停止条件：显式强制停止，或该点失败且策略为 stop，或被取消（canceled 恒为终态）。
        if force_stop or status == 'canceled' or (status in STATUS_BAD and goal.on_failure == 'stop'):
            reason = stop_reason or ('stopped after goal %d %s' % (goal.seq, status))
            self._mark_remaining(reason)
            self.state = force_state or ('canceled' if status == 'canceled' else 'aborted')
            self.stop_reason = reason
            self.finished_at = finish_time
            return None

        if not self._advance(finish_time):
            return None                       # 已整段完成

        if self._pause_requested:
            self.state = 'paused'             # 停在此处，等 resume
            return None
        self.state = 'running'
        return self.current()

    def cancel(self, finish_time, reason='mission canceled'):
        """在“无在途目标”的情况下取消整段任务（在途目标用 record(status='canceled', force_stop=True)）。"""
        if self.state not in ('running', 'paused'):
            return
        self._mark_from_current(reason)
        self.state = 'canceled'
        self.stop_reason = reason
        self.finished_at = finish_time

    def fault(self, finish_time, reason, active_goal=None, **fields):
        """进入故障锁定：取消未确认 / 动作服务消失等。禁止后续派发与新 start。

        active_goal：若故障时仍有在途目标，其结局未确认（服务端可能仍在执行），
        记作 ``unconfirmed``（而非 failed/canceled——那会谎称“旧目标已结束/车辆已
        停”）；随后其余目标记 not_attempted。无在途目标则从当前下标起记
        not_attempted。终态为 fault（非 aborted/canceled，避免把未确认取消伪装成
        已安全停止）。``fields`` 可透传 accepted/send_time 等已知事实。
        """
        if self.state not in ('running', 'paused'):
            return
        if active_goal is not None and self.index < len(self.goals):
            rec = self._new_record(active_goal, 'unconfirmed', reason=reason, **fields)
            rec['finish_time'] = finish_time
            self.records.append(rec)
            self.index += 1
        self._mark_from_current(reason)
        self.state = 'fault'
        self.stop_reason = reason
        self.finished_at = finish_time

    # ---- 暂停 / 继续 ----
    def pause(self):
        """请求暂停：在途目标跑完后不再派发下一目标。"""
        if self.state != 'running':
            return False
        self._pause_requested = True
        return True

    def resume(self):
        if self.state != 'paused':
            return False
        self._pause_requested = False
        self.state = 'running'
        return True

    # ---- 内部 ----
    def _advance(self, finish_time):
        """推进到下一目标；返回是否还有目标可派发。会处理单点模式收尾与循环回卷。"""
        if self.config.mode == 'single':
            self._mark_remaining('single mode: only goal seq 1 dispatched')
            self.state = 'completed'
            self.stop_reason = None
            self.finished_at = finish_time
            return False
        self.index += 1
        if self.index < len(self.goals):
            return True
        if self.config.loop:
            self.index = 0
            self.laps += 1
            return True
        self.state = 'completed'
        self.stop_reason = None
        self.finished_at = finish_time
        return False

    def _mark_remaining(self, reason):
        """把当前目标之后的所有目标标记为 not_attempted。"""
        for g in self.goals[self.index + 1:]:
            self.records.append(self._new_record(g, 'not_attempted', reason=reason))

    def _mark_from_current(self, reason):
        """把当前目标起的所有目标标记为 not_attempted（无在途目标时的取消）。"""
        for g in self.goals[self.index:]:
            self.records.append(self._new_record(g, 'not_attempted', reason=reason))

    @staticmethod
    def _new_record(goal, status, **fields):
        rec = {
            'seq': goal.seq, 'name': goal.name, 'frame_id': goal.frame_id,
            'target': {'x': goal.x, 'y': goal.y, 'yaw': goal.yaw},
            'status': status,
            'send_time': None, 'finish_time': None, 'elapsed_s': None,
            'accepted': False,
            'position_error_m': None, 'yaw_error_rad': None, 'within_tolerance': None,
            'error_source': None, 'error_frame': None,
            'error_sample_time': None, 'error_age_s': None,
            'recoveries': 0, 'nav_status': None, 'nav_error_code': None, 'reason': None,
        }
        rec.update(fields)
        return rec
