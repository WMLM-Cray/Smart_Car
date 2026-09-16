"""训练课程:两个阶段 + 过关判据。

目标(2026-09-16 定):车能自己跑完一圈,并且跑的过程中把赛道建出图。
S3(高速窄道)/S4(域随机化迁移实车)暂不做。
"""
from __future__ import annotations

from dataclasses import dataclass, field, replace

from .env import ConeTrackEnv
from .params import CarParams, TrackParams


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

    def make_env(self, car: CarParams = CarParams(), **kw) -> ConeTrackEnv:
        return ConeTrackEnv(car_params=car, track_params=self.track, v_cmd_fwd=self.v_cmd_fwd,
                            noise=self.noise, budget=self.budget, mapping=True, **kw)


STAGES: dict[str, Stage] = {
    # S1:接近正圆的大半径赛道 + 低速 + 无噪声,先学会顺着锥桶走
    "s1": Stage(name="s1", track=TrackParams(width=2.0, shape_noise=0.05, min_curvature_radius=3.0),
                v_cmd_fwd=1.0, noise=False, budget=120.0,
                min_laps=18, min_coverage=0.9, max_cone_hits=0),
    # S2:标准随机赛道 + 传感器噪声,按目标验收
    "s2": Stage(name="s2", track=TrackParams(), v_cmd_fwd=1.5, noise=True, budget=120.0,
                min_laps=18, min_coverage=0.9, max_cone_hits=1),
}

EVAL_SEEDS = list(range(90000, 90020))      # 与训练种子隔离


@dataclass
class StageResult:
    stage: str
    laps: int = 0
    coverage: list = field(default_factory=list)
    cone_hits: int = 0
    episodes: list = field(default_factory=list)

    @property
    def mean_coverage(self) -> float:
        return sum(self.coverage) / len(self.coverage) if self.coverage else 0.0


def evaluate(policy, stage: Stage, seeds=EVAL_SEEDS, car: CarParams = CarParams()) -> StageResult:
    """policy(obs, env) -> action。返回每个种子的结果与汇总。"""
    env = stage.make_env(car)
    result = StageResult(stage=stage.name)
    for seed in seeds:
        obs, info = env.reset(seed=seed)
        while True:
            obs, _, terminated, truncated, info = env.step(policy(obs, env))
            if terminated or truncated:
                break
        lap = info["termination_reason"] == "lap"
        result.laps += int(lap)
        result.cone_hits += info["cone_hits"]
        if lap:
            result.coverage.append(info["cone_coverage"])
        result.episodes.append(dict(seed=seed, lap=lap, **{k: info[k] for k in
                                    ("time", "cone_hits", "cone_coverage", "termination_reason")}))
    return result


def passed(result: StageResult, stage: Stage) -> bool:
    return (result.laps >= stage.min_laps
            and result.cone_hits <= stage.max_cone_hits
            and all(c >= stage.min_coverage for c in result.coverage))


def next_stage(name: str) -> str | None:
    order = list(STAGES)
    i = order.index(name)
    return order[i + 1] if i + 1 < len(order) else None


def with_seeded_track(stage: Stage, **track_kw) -> Stage:
    return replace(stage, track=replace(stage.track, **track_kw))
