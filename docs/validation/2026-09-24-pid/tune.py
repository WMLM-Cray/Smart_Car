"""拿实测曲线当被控对象,扫 speed_loop 的 kp/ki。本地跑:py -3 tune.py

被控对象(全部来自实测,见 ../2026-09-23-curve/README.md 和本目录 README):
  - 稳态 PWM->速度:12.5V / 11.24V 两条落地下降段曲线,分段线性
  - 刹停点 / 破静点:低于刹停点不出力,停住后要到破静点才重新动
  - 响应:纯滞后 θ=0.28s + 一阶 τ=0.10s(fit_tau.py,只有 τ+θ≈0.38 约束得住)
  - 减速上限 = 滑行减速度 0.75 m/s²(电调不刹车,出力小了也只能靠滑)
  - 编码器噪声 σ=0.02 m/s(静止逐帧噪声实测 19 mm/s)
"""
import random
import sys
from collections import deque
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[3] / 'teleop'))
from speed_loop import SpeedLoop, SpeedLoopConfig  # noqa: E402

DT = 0.05
THETA, TAU, COAST, NOISE = 0.28, 0.10, 0.75, 0.02

CURVES = {
    # 2026-09-24 落地,静态 12.5V(f5/f6.csv)
    '12.5V': dict(stall=1584, brk=1594, pts=[
        (1588, 0.660), (1602, 0.668), (1604, 0.683), (1606, 0.706), (1608, 0.743),
        (1610, 0.802), (1612, 0.896), (1614, 0.979), (1616, 1.064), (1618, 1.133),
        (1620, 1.200)]),                        # 1620 按斜率外推
    # 2026-09-23 落地,静态 11.24V 起测
    '11.24V': dict(stall=1588, brk=1594, pts=[
        (1590, 0.460), (1605, 0.490), (1608, 0.498), (1610, 0.522), (1612, 0.638),
        (1614, 0.684), (1616, 0.793), (1618, 0.837), (1619, 0.890), (1620, 0.933)]),
    # ~10V:只有 1619=786 一个点,按 11.24V 整条乘 0.88 估,刹停点假设 1590
    '10V(估)': dict(stall=1590, brk=1594, pts=[
        (1591, 0.405), (1605, 0.431), (1608, 0.438), (1610, 0.459), (1612, 0.561),
        (1614, 0.602), (1616, 0.698), (1618, 0.737), (1619, 0.786), (1620, 0.821)]),
}


class Plant:
    def __init__(self, curve, seed):
        self.c = curve
        self.v = 0.0
        self.queue = deque([1540.0] * round(THETA / DT))
        self.rng = random.Random(seed)

    def steady(self, pwm):
        pts = self.c['pts']
        if pwm <= pts[0][0]:
            return pts[0][1]
        for (p0, v0), (p1, v1) in zip(pts, pts[1:]):
            if pwm <= p1:
                return v0 + (v1 - v0) * (pwm - p0) / (p1 - p0)
        return pts[-1][1]

    def step(self, cmd):
        self.queue.append(float(round(cmd)))      # 驱动发的是整数 PWM
        pwm = self.queue.popleft()
        moving = self.v >= 0.05
        vt = self.steady(pwm) if pwm >= (self.c['stall'] if moving else self.c['brk']) else 0.0
        dv = (vt - self.v) * DT / TAU
        self.v = max(0.0, self.v + max(dv, -COAST * DT))
        return self.v

    def measure(self):
        return self.v + self.rng.gauss(0, NOISE) if self.v > 0 else 0.0


# 目标序列:起步 -> 升 -> 降 -> 升。都在两条曲线的可达范围内(平台以上、1620 以下)
SCHEDULE = [(0.75, 8), (0.9, 6), (0.7, 6), (0.85, 6)]


def simulate(cfg, curve, seed=0):
    loop, plant = SpeedLoop(cfg), Plant(curve, seed)
    stalls, rows, moved = 0, [], False
    for target, sec in SCHEDULE:
        seg = []
        for _ in range(int(sec / DT)):
            pwm = loop.update(target, plant.measure(), DT)
            v = plant.step(pwm)
            moved = moved or v > 0.3
            if moved and v == 0.0:
                stalls += 1
                moved = False
            seg.append((pwm, v))
        tail = seg[-int(3 / DT):]                    # 每段后 3 秒算稳态
        vs = [v for _, v in tail]
        ps = [p for p, _ in tail]
        reach = min(target, curve['pts'][-1][1])      # 够不着就按 1620 的速度算
        rows.append(dict(err=abs(sum(vs) / len(vs) - reach), vpp=max(vs) - min(vs),
                         ppp=max(ps) - min(ps)))
    return stalls, rows


def score(cfg, seeds=3):
    out = {}
    for name, curve in CURVES.items():
        st, err, vpp, ppp = 0, 0.0, 0.0, 0.0
        for s in range(seeds):
            k, rows = simulate(cfg, curve, s)
            st += k
            err = max(err, max(r['err'] for r in rows))
            vpp = max(vpp, max(r['vpp'] for r in rows))
            ppp = max(ppp, max(r['ppp'] for r in rows))
        out[name] = (st, err, vpp, ppp)
    return out


def fmt(res):
    return '  '.join(f'{n}: 停{s} 误差{e:.2f} 摆{v:.2f}m/s/{p:.0f}PWM'
                     for n, (s, e, v, p) in res.items())


if __name__ == '__main__':
    base = dict(pwm_floor=1590, pwm_breakaway=1594)
    print('现在的配置 kp=60 ki=40 i_limit=40')
    print('  ' + fmt(score(SpeedLoopConfig(**base))))
    print('\n扫 kp/ki(i_limit=80):')
    for kp in (5, 10, 20, 40, 60):
        for ki in (5, 10, 20, 45):
            cfg = SpeedLoopConfig(kp=kp, ki=ki, i_limit=80.0, **base)
            print(f'kp={kp:<3} ki={ki:<3} ' + fmt(score(cfg)))
