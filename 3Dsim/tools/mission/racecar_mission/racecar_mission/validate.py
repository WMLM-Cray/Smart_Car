"""任务 YAML 的读取与校验。纯 Python（文件读取用 PyYAML）。

校验契约（见 README）：
- 非有限坐标（NaN/Inf）、重复名称、重复/非法序号、非法 frame、缺字段 => 立即失败
  （一次性报告全部问题，不静默纠正）。
- 到达容差仅作验收记录，不覆盖 Nav2 参数。

顶层结构::

    mission:
      frame_id: map            # 必填，默认目标坐标系
      robot_base_frame: base_footprint   # 可选，量测误差用的机器人 frame
      mode: multi              # 可选 'single'|'multi'，默认 multi
      loop: false              # 可选 bool，默认 false
      on_failure: stop         # 可选 'stop'|'skip'，默认 stop
      default:                 # 可选，逐点缺省值
        timeout: 30.0
        xy_tolerance: 0.1
        yaw_tolerance: 0.1
      goals:                   # 必填，非空列表
        - name: g1             # 必填，非空，唯一
          seq: 1               # 必填，正整数，唯一
          x: 1.0               # 必填，有限数
          y: 0.0               # 必填，有限数
          yaw: 0.0             # 必填，有限数
          frame_id: map        # 可选，覆盖 mission.frame_id
          timeout: 25.0        # 可选，覆盖 default.timeout
          on_failure: skip     # 可选，覆盖 mission.on_failure
          xy_tolerance: 0.1    # 可选，仅验收记录
          yaw_tolerance: 0.1   # 可选，仅验收记录
"""
import math

from .goal import Goal, MissionConfig

DEFAULT_ALLOWED_FRAMES = ('map', 'odom', 'odom_combined')


class ValidationError(ValueError):
    """任务文件校验失败；消息里一次性列出所有问题。"""


def _is_number(v):
    return isinstance(v, (int, float)) and not isinstance(v, bool)


def _finite(v):
    return _is_number(v) and math.isfinite(v)


def mission_from_dict(doc, allowed_frames=DEFAULT_ALLOWED_FRAMES):
    """把已解析的 YAML 字典校验并转成 (MissionConfig, [Goal])，goals 按 seq 升序。"""
    allowed = set(allowed_frames)
    if not isinstance(doc, dict) or not isinstance(doc.get('mission'), dict):
        raise ValidationError("top level must be a mapping with a 'mission' mapping")
    m = doc['mission']
    problems = []

    def bad(msg):
        problems.append(msg)

    # ---- 任务级字段 ----
    frame_id = m.get('frame_id')
    if not isinstance(frame_id, str) or not frame_id:
        bad('mission.frame_id is required and must be a non-empty string')
    elif frame_id not in allowed:
        bad('mission.frame_id %r is not an allowed frame %s' % (frame_id, sorted(allowed)))

    mode = m.get('mode', 'multi')
    if mode not in ('single', 'multi'):
        bad("mission.mode must be 'single' or 'multi', got %r" % (mode,))

    loop = m.get('loop', False)
    if not isinstance(loop, bool):
        bad('mission.loop must be a bool, got %r' % (loop,))

    on_failure = m.get('on_failure', 'stop')
    if on_failure not in ('stop', 'skip'):
        bad("mission.on_failure must be 'stop' or 'skip', got %r" % (on_failure,))

    robot_base_frame = m.get('robot_base_frame', 'base_footprint')
    if not isinstance(robot_base_frame, str) or not robot_base_frame:
        bad('mission.robot_base_frame must be a non-empty string')

    enabled = m.get('enabled', True)
    if not isinstance(enabled, bool):
        bad('mission.enabled must be a bool, got %r' % (enabled,))
        enabled = True

    d = m.get('default') or {}
    if not isinstance(d, dict):
        bad('mission.default must be a mapping')
        d = {}

    def default_num(key, minimum, label):
        v = d.get(key)
        if v is None:
            return None
        if not _finite(v) or v < minimum:
            bad('%s must be a finite number >= %s, got %r' % (label, minimum, v))
            return None
        return float(v)

    default_timeout = default_num('timeout', 0.0, 'mission.default.timeout')
    default_xy = default_num('xy_tolerance', 0.0, 'mission.default.xy_tolerance')
    default_yaw = default_num('yaw_tolerance', 0.0, 'mission.default.yaw_tolerance')
    if default_xy is None:
        default_xy = 0.0
    if default_yaw is None:
        default_yaw = 0.0

    goals_raw = m.get('goals')
    if not isinstance(goals_raw, list) or not goals_raw:
        bad('mission.goals must be a non-empty list')

    # ---- 目标级字段（第一遍：只校验收集问题） ----
    seen_names = set()
    seen_seqs = set()
    for i, g in enumerate(goals_raw):
        where = 'mission.goals[%d]' % i
        if not isinstance(g, dict):
            bad('%s must be a mapping' % where)
            continue

        name = g.get('name')
        if not isinstance(name, str) or not name:
            bad('%s.name is required (non-empty string)' % where)
        elif name in seen_names:
            bad('%s.name %r duplicates an earlier goal name' % (where, name))
        seen_names.add(name)

        seq = g.get('seq')
        if not isinstance(seq, int) or isinstance(seq, bool) or seq < 1:
            bad('%s.seq must be a positive integer, got %r' % (where, seq))
        elif seq in seen_seqs:
            bad('%s.seq %d duplicates an earlier sequence number' % (where, seq))
        if isinstance(seq, int) and not isinstance(seq, bool) and seq >= 1:
            seen_seqs.add(seq)

        for field in ('x', 'y', 'yaw'):
            v = g.get(field)
            if not _finite(v):
                bad('%s.%s must be a finite number, got %r' % (where, field, v))

        gf = g.get('frame_id', frame_id)
        if not isinstance(gf, str) or not gf:
            bad('%s.frame_id is required (non-empty string)' % where)
        elif gf not in allowed:
            bad('%s.frame_id %r is not an allowed frame %s' % (where, gf, sorted(allowed)))

        t = g.get('timeout', default_timeout)
        if t is not None and (not _finite(t) or t < 0):
            bad('%s.timeout must be a finite number >= 0, got %r' % (where, t))

        of = g.get('on_failure', on_failure)
        if of not in ('stop', 'skip'):
            bad('%s.on_failure must be "stop" or "skip", got %r' % (where, of))

        xy = g.get('xy_tolerance', default_xy)
        yw = g.get('yaw_tolerance', default_yaw)
        if xy is not None and (not _finite(xy) or xy < 0):
            bad('%s.xy_tolerance must be a finite number >= 0, got %r' % (where, xy))
        if yw is not None and (not _finite(yw) or yw < 0):
            bad('%s.yaw_tolerance must be a finite number >= 0, got %r' % (where, yw))

    if problems:
        raise ValidationError('mission validation failed:\n- ' + '\n- '.join(problems))

    # ---- 第二遍：构造 Goal（此时已确保无问题） ----
    goals = []
    for g in goals_raw:
        gf = g.get('frame_id', frame_id)
        t = g.get('timeout', default_timeout)
        of = g.get('on_failure', on_failure)
        xy = g.get('xy_tolerance')
        if xy is None:
            xy = default_xy
        yw = g.get('yaw_tolerance')
        if yw is None:
            yw = default_yaw
        goals.append(Goal(
            seq=g['seq'], name=g['name'], frame_id=gf,
            x=float(g['x']), y=float(g['y']), yaw=float(g['yaw']),
            timeout=None if t is None else float(t), on_failure=of,
            xy_tolerance=float(xy), yaw_tolerance=float(yw),
        ))
    goals.sort(key=lambda g: g.seq)

    config = MissionConfig(
        mode=mode, loop=loop, on_failure=on_failure,
        frame_id=frame_id, robot_base_frame=robot_base_frame,
        enabled=enabled,
    )
    return config, goals


def load_mission_file(path, allowed_frames=DEFAULT_ALLOWED_FRAMES):
    """读取并校验任务 YAML 文件，返回 (MissionConfig, [Goal])。"""
    try:
        import yaml
    except ImportError as e:  # pragma: no cover - only hit outside ROS
        raise ValidationError('PyYAML is required to load mission files: %s' % e)
    with open(path, 'r', encoding='utf-8') as f:
        doc = yaml.safe_load(f)
    if doc is None:
        raise ValidationError('%s is empty' % path)
    return mission_from_dict(doc, allowed_frames)
