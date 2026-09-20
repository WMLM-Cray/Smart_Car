"""PPO 训练:按课程阶段跑,过关自动进下一阶段。

    python -m fastsim.train                      # 从 s1 开始,过关后自动接 s2
    python -m fastsim.train --stage s2 --steps 2000000 --envs 8

目标(2026-09-16 定):车能自己跑完一圈并把赛道建出图,过关判据见 curriculum.py。
观测叠 3 帧(单帧看不出速度变化趋势),训练时关掉建图换速度,评估时打开。
每个阶段产出 runs/<阶段>/:model.zip、eval.json、curve.png。
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from stable_baselines3 import PPO
from stable_baselines3.common.monitor import Monitor
from stable_baselines3.common.results_plotter import load_results, ts2xy
from stable_baselines3.common.vec_env import DummyVecEnv, SubprocVecEnv, VecFrameStack

from .curriculum import EVAL_SEEDS, STAGES, Stage, evaluate, next_stage, passed

FRAMES = 3


class StackedPolicy:
    """把叠帧还原到单环境评估上,帧顺序与 VecFrameStack 一致(旧 -> 新)。"""

    def __init__(self, model, obs_dim: int, frames: int = FRAMES, deterministic: bool = True):
        self.model, self.obs_dim, self.frames = model, obs_dim, frames
        self.deterministic = deterministic
        self.reset()

    def reset(self):
        self.buf = np.zeros(self.obs_dim * self.frames, np.float32)

    def __call__(self, obs, env=None):
        self.buf = np.roll(self.buf, -self.obs_dim)
        self.buf[-self.obs_dim:] = obs
        action, _ = self.model.predict(self.buf, deterministic=self.deterministic)
        return action


def make_vec_env(stage: Stage, n_envs: int, seed: int, log_dir: Path):
    def factory(rank: int):
        def _init():
            env = stage.make_env(mapping=False)        # 建图不进观测也不进奖励,训练时关掉
            env.reset(seed=seed + 1000 * rank)         # 各进程赛道序列不同
            return Monitor(env, str(log_dir / f"worker{rank}"))
        return _init

    envs = [factory(i) for i in range(n_envs)]
    vec = SubprocVecEnv(envs) if n_envs > 1 else DummyVecEnv(envs)
    return VecFrameStack(vec, FRAMES)


def plot_curve(log_dir: Path, result, out_png: Path):
    x, y = ts2xy(load_results(str(log_dir)), "timesteps")
    fig, ax = plt.subplots(figsize=(8, 4.5))
    plt.rcParams["font.sans-serif"] = ["Microsoft YaHei", "SimHei", "DejaVu Sans"]
    if len(x):
        ax.plot(x, y, color="#ccd", lw=0.6)
        w = min(101, max(1, len(y) // 10 | 1))          # 滑动平均窗口取奇数
        smooth = np.convolve(y, np.ones(w) / w, mode="valid")
        ax.plot(x[w - 1:], smooth, color="#1a7f4b", lw=1.8, label=f"滑动平均({w} 局)")
        ax.legend(loc="lower right", fontsize=8)
    ax.set_xlabel("环境步数")
    ax.set_ylabel("单局回报")
    ax.set_title(f"{result.stage}:{result.laps}/{len(result.episodes)} 局跑完整圈,"
                 f"平均建图覆盖率 {result.mean_coverage:.2f},撞锥 {result.cone_hits}")
    fig.savefig(out_png, dpi=120, bbox_inches="tight")
    plt.close(fig)


def train_stage(stage: Stage, steps: int, n_envs: int, seed: int, out_dir: Path,
                init_model: Path | None, seeds=EVAL_SEEDS):
    log_dir = out_dir / "monitor"
    log_dir.mkdir(parents=True, exist_ok=True)
    vec = make_vec_env(stage, n_envs, seed, log_dir)
    try:
        if init_model is not None:
            model = PPO.load(init_model, env=vec)       # 上一阶段的权重热启动
            model.set_env(vec)
        else:
            model = PPO("MlpPolicy", vec, learning_rate=3e-4, n_steps=512, batch_size=1024,
                        # gamma 0.995 的有效视野只有 200 步 = 10s,一圈要 27s 看不到头:
                        # 折现后"原地不动"值 -4 反而好过"冲出去摔"的 -10,策略就赖着不走了
                        # (2026-09-18 实测速度压到 0.05m/s 磨满 120s)。0.999 = 1000 步 = 50s。
                        n_epochs=10, gamma=0.999, gae_lambda=0.95, clip_range=0.2,
                        ent_coef=0.003, policy_kwargs=dict(net_arch=[128, 128]),
                        seed=seed, device="cpu", verbose=1)   # 小 MLP 在 CPU 上更快
        model.learn(total_timesteps=steps, reset_num_timesteps=init_model is None)
        model.save(out_dir / "model.zip")
    finally:
        vec.close()

    obs_dim = stage.make_env(mapping=False).observation_space.shape[0]
    result = evaluate(StackedPolicy(model, obs_dim), stage, seeds=seeds)
    (out_dir / "eval.json").write_text(json.dumps(
        dict(stage=result.stage, laps=result.laps, episodes_total=len(result.episodes),
             mean_coverage=result.mean_coverage, cone_hits=result.cone_hits,
             passed=passed(result, stage), episodes=result.episodes),
        ensure_ascii=False, indent=2), encoding="utf-8")
    plot_curve(log_dir, result, out_dir / "curve.png")
    return result


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--stage", default="s1", choices=list(STAGES))
    ap.add_argument("--steps", type=int, default=1_500_000, help="每个阶段的环境步数")
    ap.add_argument("--envs", type=int, default=8)
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--out", type=Path, default=Path("runs"))
    ap.add_argument("--no-advance", action="store_true", help="过关也不自动进下一阶段")
    ap.add_argument("--init", type=Path, help="从这个权重热启动(接着上一阶段练)")
    args = ap.parse_args()

    name, init_model = args.stage, args.init
    while name is not None:
        stage, out_dir = STAGES[name], args.out / name
        out_dir.mkdir(parents=True, exist_ok=True)
        print(f"===== 阶段 {name}:{args.steps} 步 x {args.envs} 进程 =====")
        result = train_stage(stage, args.steps, args.envs, args.seed, out_dir, init_model)
        ok = passed(result, stage)
        print(f"阶段 {name} {'过关' if ok else '未过关'}:"
              f"{result.laps}/{len(result.episodes)} 圈,覆盖率 {result.mean_coverage:.3f},"
              f"撞锥 {result.cone_hits} -> {out_dir}")
        if not ok or args.no_advance:
            break
        init_model, name = out_dir / "model.zip", next_stage(name)


if __name__ == "__main__":
    main()
