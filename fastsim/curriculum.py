"""训练课程:分阶段 + 过关判据。

目标(2026-09-16 定):车能自己跑完一圈,并且跑的过程中把赛道建出图。
S3 是按比赛规则元素拼出来的赛道(见 track_rules.py),最接近比赛当天抽到的 A-D。
S4(域随机化迁移实车)暂不做。
"""
from __future__ import annotations

from dataclasses import dataclass, field, replace

from .env import ConeTrackEnv
from .params import CarParams, RuleTrackParams, TrackParams


@dataclass
class Stage:
    name: str
    track: TrackParams
    v_cmd_fwd: float
    noise: bool
    budget: float
    min_laps: int          # 20 个评估种子里至少几个跑完整圈
    min_coverage: float    # 每个完成的种子,锥桶建图覆盖率下限
    max_cone_hits: int
    env_kw: dict = field(default_factory=dict)   # 额外的环境参数,做观测消融时用

    def make_env(self, car: CarParams = CarParams(), **kw) -> ConeTrackEnv:
        kw.setdefault("mapping", True)      # 训练时可关掉换速度,评估必须开(要看建图覆盖率)
        return ConeTrackEnv(car_params=car, track_params=self.track, v_cmd_fwd=self.v_cmd_fwd,
                            noise=self.noise, budget=self.budget, **{**self.env_kw, **kw})


STAGES: dict[str, Stage] = {
    # S1:接近正圆的大半径赛道 + 低速 + 无噪声,先学会顺着锥桶走
    "s1": Stage(name="s1", track=TrackParams(width=2.0, shape_noise=0.05, min_curvature_radius=3.0),
                v_cmd_fwd=1.0, noise=False, budget=120.0,
                min_laps=18, min_coverage=0.9, max_cone_hits=0),
    # S2:标准随机赛道 + 传感器噪声,按目标验收
    "s2": Stage(name="s2", track=TrackParams(), v_cmd_fwd=1.5, noise=True, budget=120.0,
                min_laps=18, min_coverage=0.9, max_cone_hits=1),
    # S3a:规则赛道但限速 0.8 —— S1/S2 满油门就能过,策略 96% 的时间把油门焊死在上限,
    # 到 S3 的紧弯(半径 1.0-1.8m)必须减速时就撞锥出局(2026-09-17 实测 0/20)。
    # 先在低速下学几何:同样的赛道纯追踪在 0.6m/s 下是 20/20,慢速时这活儿不难。
    "s3a": Stage(name="s3a", track=RuleTrackParams(), v_cmd_fwd=0.8, noise=True, budget=120.0,
                 min_laps=18, min_coverage=0.9, max_cone_hits=1),
    # S3:按比赛规则元素拼的赛道(45/90/135°弯、1.5m 宽、局部 1.25m、5 块蓝挡板)
    "s3": Stage(name="s3", track=RuleTrackParams(), v_cmd_fwd=1.5, noise=True, budget=120.0,
                min_laps=18, min_coverage=0.9, max_cone_hits=1),
}

EVAL_SEEDS = list(range(90000, 90020))      # 与训练种子隔离


@dataclass
class StageResult:
    stage: str
    laps: int = 0
    coverage: list = field(default_factory=list)
    cone_hits: int = 0          # 全部局的总撞锥数
    lap_cone_hits: int = 0      # 只算跑完整圈那些局的 —— 过关判据用这个,见下
    episodes: list = field(default_factory=list)

    @property
    def mean_coverage(self) -> float:
        return sum(self.coverage) / len(self.coverage) if self.coverage else 0.0


def evaluate(policy, stage: Stage, seeds=EVAL_SEEDS, car: CarParams = CarParams()) -> StageResult:
    """policy(obs, env) -> action。返回每个种子的结果与汇总。

    policy 有 reset() 的话每局开始前会被调用(叠帧策略要清空历史帧)。
    """
    env = stage.make_env(car)
    result = StageResult(stage=stage.name)
    for seed in seeds:
        obs, info = env.reset(seed=seed)
        if hasattr(policy, "reset"):
            policy.reset()
        while True:
            obs, _, terminated, truncated, info = env.step(policy(obs, env))
            if terminated or truncated:
                break
        lap = info["termination_reason"] == "lap"
        result.laps += int(lap)
        result.cone_hits += info["cone_hits"]
        if lap:
            result.lap_cone_hits += info["cone_hits"]
            result.coverage.append(info["cone_coverage"])
        result.episodes.append(dict(seed=seed, lap=lap, **{k: info[k] for k in
                                    ("time", "cone_hits", "cone_coverage", "termination_reason")}))
    return result


def passed(result: StageResult, stage: Stage) -> bool:
    """撞锥只数跑完整圈的局。

    撞锥改成不终止之后(2026-09-18,见 env.py),冲出赛道的那一局会一路把沿途的锥桶
    全犁一遍 —— 2026-09-19 实测 60 局总共 27 次撞锥,其中 24 次来自 3 局出界的,
    完赛的 57 局里 54 局一个没碰。拿总数当判据,衡量的其实是"摔得多惨"而不是"开得多干净",
    比赛关心的也只是完赛成绩上加罚多少秒。
    """
    return (result.laps >= stage.min_laps
            and result.lap_cone_hits <= stage.max_cone_hits
            and all(c >= stage.min_coverage for c in result.coverage))


def next_stage(name: str) -> str | None:
    order = list(STAGES)
    i = order.index(name)
    return order[i + 1] if i + 1 < len(order) else None


def with_seeded_track(stage: Stage, **track_kw) -> Stage:
    return replace(stage, track=replace(stage.track, **track_kw))
