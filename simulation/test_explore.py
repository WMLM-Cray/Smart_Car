"""training/explore_env + explore_rl:奖励 = 新标地面面积,截停扣分;PPO 的均值和车上前向一致。"""
import numpy as np

from common.road_policy import V_LO, Encoding

ENC = Encoding(3.0, V_LO, 3.0)


def test_reward_is_new_floor_area_and_crash_costs():
    from training.explore_env import CRASH, make
    env, _ = make(1, ENC)
    assert env.b.env.t >= 2.0 and env.area0 > 0              # 原地攒过 BEV,出发前看到的记在 area0
    total, why, n = 0.0, None, 0
    while why is None and n < 400:
        r, why = env.step(np.array([0.0, 0.3]))
        total += r
        n += 1
    crash = CRASH if why in ('guard', 'collision') else 0.0
    assert abs(total - (env.area - env.area0 - crash)) < 1e-9
    assert env.area > env.area0                              # 往前开了,标出新地面


def test_mean_raw_matches_policy_forward():
    from common.bev_policy import C2, CONVS, H, HIDDEN, N_SCALAR, W, forward
    from training.explore_rl import mean_raw
    rng = np.random.default_rng(0)
    p = {}
    for k, (ci, co, ks) in enumerate(CONVS):
        p[f'c{k}w'] = rng.normal(0, 0.1, (co, C2 if k == 0 else ci, ks, ks)).astype(np.float32)
        p[f'c{k}b'] = np.zeros(co, np.float32)
    flat = CONVS[-1][1] * (H // 16) * (W // 16) + N_SCALAR
    for k, (i, o) in enumerate(((flat, HIDDEN), (HIDDEN, HIDDEN), (HIDDEN, 2))):
        p[f'f{k}w'] = rng.normal(0, 0.05, (o, i)).astype(np.float32)
        p[f'f{k}b'] = np.zeros(o, np.float32)
    img = (rng.random((C2, H, W)) < 0.1).astype(np.float32)
    sc = np.array([0.1, 0.2, 0.0], np.float32)
    assert np.allclose(np.clip(mean_raw(p, img, sc), -1, 1), forward(p, img, sc))


def test_privileged_features_shape_and_range():
    from training.explore_env import make
    from training.explore_rl import N_PRIV
    env, _ = make(2, ENC, 'room_n')
    f0 = env.privileged()
    for _ in range(40):
        env.step(np.array([0.0, 0.3]))
    f1 = env.privileged()
    assert f0.shape == (N_PRIV,) and np.all((f1 >= 0) & (f1 <= 1.5))
    assert f1[3] < f0[3]                                   # 剩余时间在减


def test_road_level_rewards_progress_and_off_road_costs_more():
    from training.explore_env import CRASH, OFF_ROAD, PROGRESS, make
    env, info = make(3, ENC, 'LW3')
    assert info is None and env.world is None
    s0 = env.s
    total, why, n = 0.0, None, 0
    while why is None and n < 300:
        r, why = env.step(np.array([0.0, 0.3]))
        total += r
        n += 1
    pen = OFF_ROAD if why == 'off_track' else CRASH if why in ('guard', 'collision') else 0.0
    assert abs(total - (PROGRESS * (env.s - s0) - pen)) < 1e-9     # 奖励是沿路前进,不是新标面积
    assert env.s > s0 and env.privileged().shape == (4,)


def test_road_level_driving_off_the_road_is_penalised():
    from training.explore_env import OFF_ROAD, make
    env, _ = make(3, ENC, 'LWO3')
    rs, why, n = [], None, 0
    while why is None and n < 600:                      # 打死一边开出路(这个种子是出界,不是截停)
        r, why = env.step(np.array([1.0, 0.3]))
        rs.append(r)
        n += 1
    assert why == 'off_track' and rs[-1] < -OFF_ROAD + 1.0


def test_rollout_and_evaluate_run_on_road_and_room_levels():
    from common.bev_policy import C2
    from training.bev_net import make_net, weights_of
    from training.explore_rl import evaluate, rollout
    w = {k: v.astype(np.float32) for k, v in weights_of(make_net(C2)).items()}
    for lv in ('LW3', 'room'):                          # 路段关卡的 RoadEnv 没有 dist(第三轮第一次评估就崩了)
        why, area, dist = evaluate((5, w, ENC, 3.0, lv))
        assert why in ('timeout', 'guard', 'collision', 'off_track', 'goal') and dist >= 0
        out = rollout((5, 1, w, np.log(np.full(2, 0.1)), ENC, 3.0, lv))
        assert len(out[0]) == len(out[4]) > 0


def test_rollout_returns_observation_after_last_action_on_timeout():
    """到点续算要用最后一拍动作之后的观测 s_T,不是最后存下的动作前观测 s_{T-1}(2026-10-18 之前取错了)。"""
    from common.bev_policy import C2
    from training.bev_net import make_net, weights_of
    from training.explore_rl import rollout
    w = {k: v.astype(np.float32) for k, v in weights_of(make_net(C2)).items()}
    out = rollout((5, 1, w, np.log(np.full(2, 0.1)), ENC, 3.0, 'room'))
    imgs, scs, why, last = out[0], out[1], out[5], out[9]
    assert why == 'timeout'                                     # 这个种子 3 s 内不撞
    p_T, sc_T, priv_T = last
    assert p_T.shape == imgs[-1].shape and sc_T.shape == scs[-1].shape and priv_T.shape == out[8][-1].shape
    assert not (np.array_equal(p_T, imgs[-1]) and np.array_equal(sc_T, scs[-1]))   # 动作之后车动过,观测变了
