"""用 2026-09-23 四轮落地分段下扫的原始帧拟合 v' = (f(pwm(t-θ)) - v)/τ。
f 是同一天测的稳态曲线(每档后一半平均)。只拟合车在滚动的段(v > 0.2),
静止/起步那一下是静摩擦,不是这个模型管的。"""
import glob
import numpy as np

# 11.24V 落地下降段稳态点(mm/s),同 PWM 多次测量取平均;1620 是外推,只在 r4 顶端用到
PTS = {1590: 468, 1591: 464, 1592: 460, 1594: 456, 1595: 466, 1596: 468, 1597: 467,
       1598: 490, 1599: 489, 1600: 459, 1601: 461, 1602: 467, 1603: 477, 1604: 480,
       1605: 494, 1606: 484, 1608: 498, 1610: 522, 1612: 638, 1614: 684, 1616: 793,
       1618: 837, 1620: 880}
P = np.array(sorted(PTS)); V = np.array([PTS[p] for p in P]) / 1000


def f(pwm):
    return np.interp(pwm, P, V)


def sim(t, pwm, v0, tau, theta):
    v = np.empty_like(t); v[0] = v0
    for k in range(1, len(t)):
        dt = t[k] - t[k - 1]
        u = f(np.interp(t[k] - theta, t, pwm))
        v[k] = v[k - 1] + dt / tau * (u - v[k - 1])
    return v


runs = []
for fn in sorted(glob.glob('r*.csv')):
    d = np.genfromtxt(fn, delimiter=',', names=True)
    t, pwm, v = d['t'] - d['t'][0], d['pwm'], d['v']
    moving = np.where(v > 0.2)[0]
    if len(moving) == 0:
        continue
    # 从第一次滚起来到最后一帧滚动;中间塌掉的(r3 末尾)也截掉
    a = moving[0]; b = moving[-1] + 1
    runs.append((fn, t[a:b], pwm[a:b], v[a:b]))

best = None
for tau in np.arange(0.02, 1.51, 0.04):
    for theta in np.arange(0.0, 0.81, 0.04):
        err = np.concatenate([sim(t, p, v[0], tau, theta) - v for _, t, p, v in runs])
        rmse = float(np.sqrt(np.mean(err ** 2)))
        if best is None or rmse < best[0]:
            best = (rmse, tau, theta)
rmse, tau, theta = best
print(f'合并拟合: tau = {tau:.2f}s  theta = {theta:.2f}s  RMSE = {rmse*1000:.0f} mm/s')
for fn, t, p, v in runs:
    e = sim(t, p, v[0], tau, theta) - v
    print(f'  {fn}: {len(t)} 帧, RMSE {np.sqrt(np.mean(e**2))*1000:.0f} mm/s')
# 灵敏度:tau 取最优的一半/两倍时误差涨多少 —— 说明这个数被数据约束得有多紧
for k in (0.5, 2.0):
    err = np.concatenate([sim(t, p, v[0], tau * k, theta) - v for _, t, p, v in runs])
    print(f'  tau x{k}: RMSE {np.sqrt(np.mean(err**2))*1000:.0f} mm/s')
noise = np.concatenate([np.diff(v) for _, t, p, v in runs])
print(f'逐帧差分标准差(噪声量级) {np.std(noise)*1000:.0f} mm/s,帧率 {len(runs[0][1])/(runs[0][1][-1]-runs[0][1][0]):.0f} Hz')
# 只看 r4:唯一一段有大幅速度变化(爬过拐点 460 -> 880 再降回来)的数据
fn, t, p, v = [r for r in runs if r[0] == 'r4.csv'][0]
g = sorted((float(np.sqrt(np.mean((sim(t, p, v[0], ta, th) - v) ** 2))), ta, th)
           for ta in np.arange(0.02, 1.51, 0.04) for th in np.arange(0.0, 0.81, 0.04))
print('r4 单独最好的 5 组 (RMSE mm/s, tau, theta):')
for r, ta, th in g[:5]:
    print(f'  {r*1000:.0f}  {ta:.2f}  {th:.2f}')
