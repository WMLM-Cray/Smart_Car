"""任务报告与检查点的组装/序列化。纯 Python（JSON）。"""
from .goal import GOAL_STATUSES
from .validate import ValidationError, mission_from_dict, DEFAULT_ALLOWED_FRAMES

# 到达“确定终态”的目标状态（区别于 unconfirmed=结局未确认、not_attempted=未发出）。
_TERMINAL_GOAL = ('success', 'failed', 'timeout', 'canceled')
_TERMINAL_STATES = ('completed', 'aborted', 'canceled', 'fault')


def build_report(engine, goal_file=None):
    """把引擎当前状态组装成完整报告字典。

    dispatched/accepted/terminal/not_attempted 分开计数，避免把未尝试目标算进
    “已派发”。dispatched=实际发出的目标数（status != not_attempted，含 unconfirmed），
    accepted=动作服务器接受的目标数，terminal=到达确定终态的目标数（不含 unconfirmed，
    因其结局未确认），not_attempted=未尝试。
    """
    recs = engine.records
    counts = {s: 0 for s in GOAL_STATUSES}
    for r in recs:
        if r['status'] in counts:
            counts[r['status']] += 1
    dispatched = sum(1 for r in recs if r['status'] != 'not_attempted')
    accepted = sum(1 for r in recs if r.get('accepted'))
    terminal = sum(1 for r in recs if r['status'] in _TERMINAL_GOAL)
    not_attempted = counts.get('not_attempted', 0)
    return {
        'mission': {
            'goal_file': goal_file,
            'mode': engine.config.mode,
            'loop': engine.config.loop,
            'on_failure': engine.config.on_failure,
            'frame_id': engine.config.frame_id,
            'robot_base_frame': engine.config.robot_base_frame,
            'state': engine.state,
            'stop_reason': engine.stop_reason,
            'started_at': engine.started_at,
            'finished_at': engine.finished_at,
            'laps': engine.laps,
            'total_goals': len(engine.goals),
            'dispatched': dispatched,
            'accepted': accepted,
            'terminal': terminal,
            'not_attempted': not_attempted,
            'counts': counts,
        },
        'goals': recs,
    }


def build_checkpoint(engine, goal_file, next_index):
    """生成可持久化的检查点（供“显式加载+再次确认”后恢复，绝不自动恢复）。"""
    return {
        'version': 1,
        'goal_file': goal_file,
        'state': engine.state,
        'mode': engine.config.mode,
        'loop': engine.config.loop,
        'on_failure': engine.config.on_failure,
        'frame_id': engine.config.frame_id,
        'robot_base_frame': engine.config.robot_base_frame,
        'enabled': engine.config.enabled,
        'goals': [g.to_dict() for g in engine.goals],
        'records': engine.records,
        'next_index': next_index,
    }


def parse_checkpoint(doc, allowed_frames=DEFAULT_ALLOWED_FRAMES):
    """解析并严格校验检查点，返回 (config, goals, records, next_index, state)。

    采用与任务 YAML 完全相同的严格校验：有限坐标、frame 白名单、enabled 严格 bool、
    序号/名称唯一等，绝不放行 NaN / 非法 frame / 字符串 "false" 等。另校验
    next_index 范围与 records 的一致性。
    """
    if not isinstance(doc, dict) or doc.get('version') != 1:
        raise ValidationError('not a valid mission checkpoint (version 1 required)')

    # 复用任务 YAML 的严格校验（有限坐标 / frame 白名单 / enabled 严格 bool / 序号唯一）。
    config, goals = mission_from_dict({
        'mission': {
            'frame_id': doc.get('frame_id'),
            'robot_base_frame': doc.get('robot_base_frame', 'base_footprint'),
            'mode': doc.get('mode', 'multi'),
            'loop': doc.get('loop', False),
            'on_failure': doc.get('on_failure', 'stop'),
            'enabled': doc.get('enabled', True),
            'default': doc.get('default', {}),
            'goals': doc.get('goals'),
        },
    }, allowed_frames)

    records = doc.get('records', [])
    if not isinstance(records, list):
        raise ValidationError('checkpoint records must be a list')

    name_by_seq = {g.seq: g.name for g in goals}
    attempted = 0
    for r in records:
        if not isinstance(r, dict):
            raise ValidationError('checkpoint record must be a mapping')
        status = r.get('status')
        if status not in GOAL_STATUSES:
            raise ValidationError('checkpoint record has invalid status %r' % (status,))
        seq = r.get('seq')
        if not isinstance(seq, int) or isinstance(seq, bool) or seq not in name_by_seq:
            raise ValidationError('checkpoint record references unknown goal seq %r' % (seq,))
        if r.get('name') not in (None, name_by_seq[seq]):
            raise ValidationError('checkpoint record name/seq mismatch for seq %r' % (seq,))
        if status != 'not_attempted':
            attempted += 1

    next_index = doc.get('next_index')
    if not isinstance(next_index, int) or isinstance(next_index, bool) or not (0 <= next_index <= len(goals)):
        raise ValidationError(
            'checkpoint next_index %r out of range 0..%d' % (next_index, len(goals)))

    state = doc.get('state', 'running')
    if state not in _TERMINAL_STATES:
        # 可恢复（running/paused）检查点：records 只能是“已完成目标”，next_index 必须与之吻合，
        # 且不允许出现 not_attempted（那属于终态快照）。
        if next_index != attempted:
            raise ValidationError(
                'checkpoint next_index %d inconsistent with %d attempted records'
                % (next_index, attempted))
        if any(r['status'] == 'not_attempted' for r in records):
            raise ValidationError('non-terminal checkpoint must not contain not_attempted records')

    return config, goals, records, next_index, state
