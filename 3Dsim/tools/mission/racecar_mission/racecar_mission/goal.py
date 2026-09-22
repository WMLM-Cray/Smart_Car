"""任务目标与任务配置数据结构。纯 Python，无 ROS。

到达容差（xy_tolerance / yaw_tolerance）只作为任务验收记录使用，
绝不前传覆盖 Nav2 自身的 goal_checker 参数（Nav2 的容差在其自身参数里）。
"""
from dataclasses import dataclass

# 单个目标的最终状态（任务报告字段）。
# unconfirmed = 目标已发出但结局未确认（取消被拒/未确认、动作服务消失等）——服务端
#   可能仍在执行该目标，故不得记作 failed/canceled（那会谎称“旧目标已结束/车辆已停”）。
GOAL_STATUSES = ('success', 'failed', 'timeout', 'canceled', 'unconfirmed', 'not_attempted')


@dataclass(frozen=True)
class Goal:
    """一个导航目标点。

    seq/name 为报告用序号与名称；frame_id 为坐标所在坐标系；x/y/yaw 为位姿；
    timeout 为该点超时秒数（None 表示不设超时）；on_failure 为该点失败后的策略
    （stop=停止整条任务 / skip=跳到下一点）；xy_tolerance / yaw_tolerance 仅作
    验收记录（记录最终位姿误差是否落在该容差内），不传给 Nav2。
    """
    seq: int
    name: str
    frame_id: str
    x: float
    y: float
    yaw: float
    timeout: float
    on_failure: str
    xy_tolerance: float
    yaw_tolerance: float

    def to_dict(self):
        return dict(
            seq=self.seq, name=self.name, frame_id=self.frame_id,
            x=self.x, y=self.y, yaw=self.yaw,
            timeout=self.timeout, on_failure=self.on_failure,
            xy_tolerance=self.xy_tolerance, yaw_tolerance=self.yaw_tolerance,
        )

    @classmethod
    def from_dict(cls, d):
        return cls(
            seq=int(d['seq']), name=str(d['name']), frame_id=str(d['frame_id']),
            x=float(d['x']), y=float(d['y']), yaw=float(d['yaw']),
            timeout=None if d.get('timeout') is None else float(d['timeout']),
            on_failure=str(d['on_failure']),
            xy_tolerance=float(d.get('xy_tolerance', 0.0)),
            yaw_tolerance=float(d.get('yaw_tolerance', 0.0)),
        )


@dataclass(frozen=True)
class MissionConfig:
    """整条任务的配置（对应 YAML 顶层 mission）。"""
    mode: str                 # 'single'（单点） | 'multi'（连续多点）
    loop: bool                # 是否循环；默认 False，必须显式开启
    on_failure: str           # 默认失败策略 'stop' | 'skip'
    frame_id: str             # 默认目标坐标系
    robot_base_frame: str     # 用于量测最终位姿误差的机器人基座 frame
    enabled: bool = True      # False 时任务器拒绝启动（如实车首测目标集默认禁用）

    def to_dict(self):
        return dict(
            mode=self.mode, loop=self.loop, on_failure=self.on_failure,
            frame_id=self.frame_id, robot_base_frame=self.robot_base_frame,
            enabled=self.enabled,
        )
