#!/usr/bin/env python3
"""车辆动力学实测:起步、加速、滑行、刹车、侧向抓地力。手机网页点开始,车自己跑一段脚本,全程录数据。

    source /opt/ros/humble/setup.bash && cd ~/autocar_bev2
    python3 -m calibration.dynamics_test                 # 网页 http://<车IP>:8091/,数据存 ~/drives/dyn_<时间>/

发 /teleop_cmd_vel(和网页遥控、coast_test 同一个话题;跑的时候别碰遥控页)。脚本:

  launch  起步:直接跳到 --jump PWM,再按 up_rate 往上爬,记"多久动起来"(测电调接不接受跳变)
  accel   加速:起步后一步给到 hi PWM,到 v_max / 时间 / 距离上限就松油,接着记滑行到停
  brake   刹车:加速到 v0 后给 brake PWM(低于中位),车速降到 BRAKE_RELEASE 就回中位(不等停稳,停住再发就是倒车)
  circle  侧向:固定转向绕圈,PWM 慢慢往上加,到 v_max 或开始打滑(实际横摆角速度 < 理论的 SLIP_RATIO)就松油

安全(任何一条触发都立即回中位并保持 1 s,不刹车 —— 刹车能力还没测):
  - 网页大红"急停"(任何页面都能按);点"开始"的那台设备心跳断 HB_TIMEOUT 秒(手机锁屏、断网)
    —— 只认它的心跳,别的页面开着也不会替它续命
  - 雷达扫车身矩形沿转向弧的净空 < 滑停距离 v²/(2·COAST) + GUARD_MARGIN;雷达掉线
  - 离起点超过 fence 米(里程计);车速超过 V_HARD
  - IMU:车身倾斜 > TILT_STOP 度、或横滚/俯仰角速度持续过大(撞台阶、掉下坎、侧翻)
  - 编码器掉线;发现别人也在发 /teleop_cmd_vel
⚠ 雷达扫描面离地 28 cm,比它矮的台阶、路沿、往下的坎都看不见:起点周围 fence 范围内要先人工确认是平地。
"""
from __future__ import annotations

import argparse
import json
import math
import os
import signal
import threading
import time
from collections import deque
from dataclasses import dataclass, field

import numpy as np

from common.params import CarParams

P = CarParams()
PWM_MID, TURN_MID = P.pwm_mid, P.turn_mid
PORT = 8091
TICK = 0.02                 # 控制周期 s
HB_TIMEOUT = 0.6            # 网页心跳断这么久就停
ENC_STALE = 0.3
COAST = 0.7                 # 滑停减速度 m/s²(实测 0.73–0.78,取小的)
GUARD_MARGIN = 0.5          # 滑停之后保险杠前还要剩的净空 m
GUARD_LOOKAHEAD = 7.0       # 本进程里把防撞沿弧看的距离放到这么远(默认 4 m,2.5 m/s 滑停就要 4.5 m)
V_HARD = 3.2                # 车速硬上限 m/s
TILT_STOP = 15.0            # 车身倾斜超过这个度就停
RATE_STOP = 2.0             # 横滚/俯仰角速度超过这个 rad/s ...
RATE_HOLD = 0.08            # ... 持续这么久就停
MOVING = 0.1                # 车速高于这个算动起来了
BRAKE_RELEASE = 0.15        # 刹车时降到这个车速就回中位
SLIP_RATIO = 0.75           # 绕圈时 实际 wz / (v/R) 低于这个算开始打滑
SLIP_HOLD = 0.3
STOP_HOLD = 1.0             # 停车后中位保持 s
LAUNCH_TIMEOUT = 3.0        # 起步这么久还没动就放弃(不然一直顶着油门)


class LaunchFailed(Exception):
    pass

DEFAULTS = {
    'launch': dict(jump=1575, up_rate=40, top=1615, t_max=4.0),
    'accel': dict(hi=1640, v_max=2.0, t_max=5.0, d_max=8.0, up_rate=40),
    'brake': dict(hi=1630, v0=1.5, brake=1440, t_max=6.0, d_max=8.0, up_rate=40),
    'circle': dict(turn=136, start=1600, rate=3.0, top=1680, v_max=2.5, t_max=25.0, up_rate=40),
}


def steer_of_turn(turn: float) -> float:
    """舵机值 -> 前轮转角 rad(左正),按实测 steer_curve 插值。"""
    pts = np.array(P.steer_curve, float)
    return math.radians(float(np.interp(turn, pts[:, 0], pts[:, 1])))


def stop_distance(v: float) -> float:
    return v * v / (2 * COAST) + GUARD_MARGIN


@dataclass
class Sensors:
    v: float = 0.0
    enc_t: float = 0.0
    imu: tuple = (0.0,) * 6             # wx wy wz ax ay az
    imu_t: float = 0.0
    roll: float = 0.0                   # 开跑后陀螺积出来的横滚 / 俯仰 rad(加速度计会被加减速、
    pitch: float = 0.0                  # 绕圈的侧向加速度带偏,起步一下就能偏 27°,不能用)
    x: float = 0.0
    y: float = 0.0
    yaw: float = 0.0
    dist: float = 0.0
    gap_fn: object = None
    lidar_ok: bool = False

    def tilt_deg(self) -> float:
        return math.degrees(math.hypot(self.roll, self.pitch))


@dataclass
class Safety:
    """每拍 check() 返回停车原因或 None。纯逻辑,不碰 ROS。"""
    fence: float = 8.0
    guard: bool = True
    rate_since: float | None = None
    estop: bool = False
    foreign: bool = False

    def check(self, now, s: Sensors, turn, hb_t) -> str | None:
        if self.estop:
            return '急停'
        if now - hb_t > HB_TIMEOUT:
            return '网页断开'
        if self.foreign:
            return '有别人在发 /teleop_cmd_vel'
        if now - s.enc_t > ENC_STALE:
            return '编码器掉线'
        if abs(s.v) > V_HARD:
            return f'超速 {s.v:.2f}'
        if math.hypot(s.x, s.y) > self.fence:
            return f'出围栏 {math.hypot(s.x, s.y):.1f} m'
        if s.tilt_deg() > TILT_STOP:
            return f'车身倾斜 {s.tilt_deg():.0f}°'
        wx, wy = s.imu[0], s.imu[1]
        if max(abs(wx), abs(wy)) > RATE_STOP:
            self.rate_since = now if self.rate_since is None else self.rate_since
            if now - self.rate_since >= RATE_HOLD:
                return f'车身晃动 {max(abs(wx), abs(wy)):.1f} rad/s'
        else:
            self.rate_since = None
        if self.guard:
            gap = s.gap_fn(steer_of_turn(turn)) if s.gap_fn else None
            if gap is None:
                return '雷达没数据'
            if s.v > 0.05 and gap < stop_distance(s.v):
                return f'前方 {gap:.2f} m < 滑停 {stop_distance(s.v):.2f} m'
        return None


# ---------------- 脚本:生成器,每拍 send((pwm, turn, 阶段)),外面每拍把最新 Sensors 给它 ----------------

def _launch(s, up_rate, turn=TURN_MID, start=1560, top=1615):
    """从 start 按 up_rate PWM/s 爬,动起来就返回当时的 PWM。"""
    pwm, t = float(start), 0.0
    while s().v < MOVING:
        if t > LAUNCH_TIMEOUT:
            raise LaunchFailed(f'{LAUNCH_TIMEOUT:.0f} s 没动起来(PWM 已到 {pwm:.0f}):电调没开 / 睡着了 / 中位锁偏')
        yield int(pwm), turn, 'launch'
        pwm = min(top, pwm + up_rate * TICK)
        t += TICK
    return pwm


def prog_launch(s, jump, up_rate, top, t_max):
    t = 0.0
    pwm = float(jump)
    while s().v < MOVING and t < t_max:
        yield int(pwm), TURN_MID, 'launch'
        pwm = min(top, pwm + up_rate * TICK)
        t += TICK


def prog_accel(s, hi, v_max, t_max, d_max, up_rate):
    yield from _launch(s, up_rate)
    t, d0 = 0.0, s().dist
    while s().v < v_max and t < t_max and s().dist - d0 < d_max:
        yield int(hi), TURN_MID, 'accel'
        t += TICK
    while s().v > 0.03:                                  # 滑行到停
        yield PWM_MID, TURN_MID, 'coast'


def prog_brake(s, hi, v0, brake, t_max, d_max, up_rate):
    yield from _launch(s, up_rate)
    t, d0 = 0.0, s().dist
    while s().v < v0 and t < t_max and s().dist - d0 < d_max:
        yield int(hi), TURN_MID, 'accel'
        t += TICK
    while s().v > BRAKE_RELEASE:
        yield int(brake), TURN_MID, 'brake'
    for _ in range(int(STOP_HOLD / TICK)):
        yield PWM_MID, TURN_MID, 'coast'


def prog_circle(s, turn, start, rate, top, v_max, t_max, up_rate):
    turn = int(turn)
    for _ in range(int(0.5 / TICK)):                     # 先把舵打好
        yield PWM_MID, turn, 'steer'
    pwm = max(float(start), (yield from _launch(s, up_rate, turn)))
    R = P.wheelbase / max(math.tan(abs(steer_of_turn(turn))), 1e-3)
    t, slip_since = 0.0, None
    while s().v < v_max and t < t_max:
        ss = s()
        ratio = abs(ss.imu[2]) * R / max(ss.v, 1e-3)
        if ss.v > 0.8 and ratio < SLIP_RATIO:
            slip_since = t if slip_since is None else slip_since
            if t - slip_since >= SLIP_HOLD:
                break
        else:
            slip_since = None
        yield int(pwm), turn, 'circle'
        pwm = min(top, pwm + rate * TICK)
        t += TICK
    while s().v > 0.03:
        yield PWM_MID, turn, 'coast'


PROGS = {'launch': prog_launch, 'accel': prog_accel, 'brake': prog_brake, 'circle': prog_circle}


# ---------------- 结果 ----------------

def smooth_diff(t, v, half=0.15):
    """v 对 t 的导数,按 ±half 秒窗口线性拟合(编码器 50 Hz 有噪声)。"""
    out = np.zeros_like(v)
    for i in range(len(t)):
        m = np.abs(t - t[i]) <= half
        if m.sum() >= 3:
            out[i] = np.polyfit(t[m] - t[i], v[m], 1)[0]
    return out


def summarize(rec: dict, prog: str) -> dict:
    t, v, ph = np.asarray(rec['t']), np.asarray(rec['v']), np.asarray(rec['phase'])
    if len(t) < 5:
        return {}
    t = t - t[0]
    a = smooth_diff(t, v)
    out = {'v_max': round(float(v.max()), 2)}
    mv = np.flatnonzero(v >= MOVING)
    if len(mv):
        out['动起来用时_s'] = round(float(t[mv[0]]), 2)
        pw = np.asarray(rec['pwm'])
        out['动起来时PWM'] = int(pw[mv[0]])
    for name, sel in (('加速', ph == 'accel'), ('滑行', ph == 'coast'), ('刹车', ph == 'brake'), ('绕圈', ph == 'circle')):
        if sel.sum() < 5:
            continue
        tt, vv = t[sel], v[sel]
        if name in ('滑行', '刹车'):
            moving = vv > BRAKE_RELEASE
            if moving.sum() >= 3:
                d = (vv[moving][0] - vv[moving][-1]) / max(tt[moving][-1] - tt[moving][0], 1e-3)
                out[f'{name}平均减速_m/s2'] = round(float(d), 2)
                out[f'{name}起始速度'] = round(float(vv[0]), 2)
                out[f'{name}距离_m'] = round(float(np.sum((vv[1:] + vv[:-1]) / 2 * np.diff(tt))), 2)
        if name == '加速':
            out['加速峰值_m/s2'] = round(float(a[sel].max()), 2)
            for target in (1.0, 1.5, 2.0, 2.5):
                k = np.flatnonzero(v >= target)
                if len(k):
                    out[f'到 {target} m/s_s'] = round(float(t[k[0]]), 2)
        if name == '绕圈':
            wz = np.abs(np.asarray(rec['wz']))[sel]
            ay_kin = vv * wz
            ay_imu = np.abs(np.asarray(rec['ay']))[sel]
            k = int(np.argmax(ay_kin))
            out['最大侧向_v·wz_m/s2'] = round(float(ay_kin[k]), 2)
            out['此时车速'] = round(float(vv[k]), 2)
            out['此时半径_m'] = round(float(vv[k] / max(wz[k], 1e-3)), 2)
            out['IMU ay 最大_m/s2'] = round(float(ay_imu.max()), 2)
            out['g'] = round(float(ay_kin[k]) / 9.81, 2)
    return out


# ---------------- ROS + 网页 ----------------

PAGE = """<!doctype html><html><head><meta charset='utf-8'><meta name='viewport' content='width=device-width,initial-scale=1'>
<style>body{font-family:sans-serif;margin:8px;background:#111;color:#eee}button{font-size:20px;padding:10px 16px;margin:4px}
#es{width:100%;height:120px;font-size:44px;background:#d00;color:#fff;border:0;border-radius:12px}
input{width:70px;font-size:16px}#st{font-size:20px;white-space:pre}.p{margin:6px 0}</style></head><body>
<button id='es' onclick="post('/estop')">急停</button>
<div id='st'>连接中…</div>
<div class='p'><select id='prog' style='font-size:20px' onchange='fill()'></select> <span id='params'></span></div>
<button onclick='start()' style='background:#2a2'>开始</button><button onclick="post('/clear')">解除急停</button>
<label><input type='checkbox' id='guard' checked> 雷达防撞</label> 围栏 <input id='fence' value='8'> m
<pre id='res'></pre>
<script>
const D=__DEFAULTS__;const sel=document.getElementById('prog');
for(const k in D){const o=document.createElement('option');o.value=k;o.text=k;sel.add(o)}
function fill(){const d=D[sel.value];document.getElementById('params').innerHTML=Object.keys(d).map(k=>k+" <input id='p_"+k+"' value='"+d[k]+"'>").join(' ')}
fill();
function post(u,b){return fetch(u,{method:'POST',body:b?JSON.stringify(b):''}).then(r=>r.text()).then(t=>{document.getElementById('res').textContent=t})}
function start(){const d=D[sel.value],p={};for(const k in d)p[k]=parseFloat(document.getElementById('p_'+k).value);
 post('/start',{prog:sel.value,params:p,guard:document.getElementById('guard').checked,fence:parseFloat(document.getElementById('fence').value)})}
setInterval(()=>fetch('/status').then(r=>r.json()).then(s=>{document.getElementById('st').textContent=
 (s.estop?'【急停中】 ':'')+s.state+'\\n车速 '+s.v.toFixed(2)+' m/s  前方 '+(s.gap<0?'--':s.gap.toFixed(2))+' m  离起点 '+s.r.toFixed(1)+' m\\nPWM '+s.pwm+'  舵 '+s.turn+'  倾斜 '+s.tilt.toFixed(1)+'°';
 if(s.result)document.getElementById('res').textContent=s.result}).catch(()=>{document.getElementById('st').textContent='连不上车'}),150)
</script></body></html>"""


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--out', default=os.path.expanduser(time.strftime('~/drives/dyn_%m%d_%H%M%S')))
    a = ap.parse_args(argv)
    os.makedirs(a.out, exist_ok=True)

    import http.server

    import rclpy
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Odometry
    from rclpy.node import Node
    from sensor_msgs.msg import Imu, LaserScan

    import vehicle.scan_guard as sg
    sg.MAX_LOOKAHEAD = GUARD_LOOKAHEAD                      # 只改本进程
    guard = sg.ScanGuard()

    from rclpy.signals import SignalHandlerOptions
    rclpy.init(args=[], signal_handler_options=SignalHandlerOptions.NO)   # rclpy 的处理只关 ROS,HTTP 线程照跑
    node = Node('dynamics_test')
    pub = node.create_publisher(Twist, '/teleop_cmd_vel', 5)
    lock = threading.Lock()
    S = Sensors(gap_fn=lambda steer: guard.gap(steer))
    safety = Safety()
    st = dict(state='空闲', hb=0.0, owner=None, running=False, pwm=PWM_MID, turn=TURN_MID, result='', sent=deque(maxlen=20),
              rec=None, gyro=deque(maxlen=100), bias=(0.0, 0.0))

    def on_enc(m):
        now = time.time()
        with lock:
            dt = now - S.enc_t if S.enc_t else 0.0
            S.v, S.enc_t = m.twist.twist.linear.x, now
            if 0 < dt < 0.2:
                S.yaw += S.imu[2] * dt
                S.x += S.v * math.cos(S.yaw) * dt
                S.y += S.v * math.sin(S.yaw) * dt
                S.dist += abs(S.v) * dt

    def on_imu(m):
        w, c = m.angular_velocity, m.linear_acceleration
        now = time.time()
        with lock:
            dt = now - S.imu_t if S.imu_t else 0.0
            S.imu, S.imu_t = (w.x, w.y, w.z, c.x, c.y, c.z), now
            if 0 < dt < 0.1:
                S.roll += (w.x - st['bias'][0]) * dt
                S.pitch += (w.y - st['bias'][1]) * dt
            if not st['running']:
                st['gyro'].append((w.x, w.y))

    def on_cmd(m):
        key = (round(m.linear.x), round(m.angular.z))
        with lock:
            if st['running'] and key not in st['sent'] and key[0] != PWM_MID:   # 别人补的中位(看门狗)不算抢车
                safety.foreign = True

    node.create_subscription(Odometry, '/encoder', on_enc, 20)
    node.create_subscription(Imu, '/IMU_data', on_imu, 20)
    node.create_subscription(LaserScan, '/scan', lambda m: guard.update(m), 1)
    node.create_subscription(Twist, '/teleop_cmd_vel', on_cmd, 20)
    threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()

    def send(pwm, turn):
        tw = Twist()
        tw.linear.x, tw.angular.z = float(pwm), float(turn)
        with lock:
            st['sent'].append((int(pwm), int(turn)))
            st['pwm'], st['turn'] = int(pwm), int(turn)
        pub.publish(tw)

    def run(prog, params, use_guard, fence):
        with lock:
            if len(st['gyro']) < 20:
                st['state'] = '等 IMU 数据'
                st['running'] = False
                return
            st['bias'] = tuple(np.mean(np.array(st['gyro']), axis=0))    # 开跑前静止时的陀螺零偏
            S.x = S.y = S.yaw = S.dist = S.roll = S.pitch = 0.0
            safety.guard, safety.fence, safety.foreign, safety.rate_since = use_guard, fence, False, None
        rec = {k: [] for k in ('t', 'pwm', 'turn', 'v', 'wx', 'wy', 'wz', 'ax', 'ay', 'az', 'gap', 'x', 'y', 'yaw', 'phase')}
        gen = PROGS[prog](lambda: S, **params)
        reason, phase = None, ''
        nxt = time.time()
        while True:
            now = time.time()
            with lock:
                reason = safety.check(now, S, st['turn'], st['hb'])
            if reason:
                break
            try:
                pwm, turn, phase = next(gen)
            except StopIteration:
                break
            except LaunchFailed as e:
                reason = str(e)
                break
            send(pwm, turn)
            with lock:
                gap = guard.gap(steer_of_turn(turn))
                for k, val in zip(('t', 'pwm', 'turn', 'v', 'wx', 'wy', 'wz', 'ax', 'ay', 'az', 'gap', 'x', 'y', 'yaw', 'phase'),
                                  (now, pwm, turn, S.v, *S.imu, -1.0 if gap is None else gap, S.x, S.y, S.yaw, phase)):
                    rec[k].append(val)
                st['state'] = f'{prog}:{phase}'
            nxt += TICK
            time.sleep(max(0.0, nxt - time.time()))
        end = time.time() + STOP_HOLD
        while time.time() < end or S.v > 0.05 and time.time() < end + 8:   # 回中位,等停稳(最多再 8 s)
            send(PWM_MID, TURN_MID)
            with lock:
                for k, val in zip(('t', 'pwm', 'turn', 'v', 'wx', 'wy', 'wz', 'ax', 'ay', 'az', 'gap', 'x', 'y', 'yaw', 'phase'),
                                  (time.time(), PWM_MID, TURN_MID, S.v, *S.imu, -1.0, S.x, S.y, S.yaw,
                                   'coast' if not reason else 'stop')):
                    rec[k].append(val)
            time.sleep(TICK)
        summ = summarize(rec, prog)
        k = len([f for f in os.listdir(a.out) if f.endswith('.npz')]) + 1
        name = f'{k:02d}_{prog}'
        np.savez(os.path.join(a.out, name + '.npz'), **{kk: np.array(vv) for kk, vv in rec.items()},
                 params=json.dumps(params), reason=reason or '')
        line = json.dumps(dict(run=name, params=params, stop=reason or '正常结束', **summ), ensure_ascii=False)
        with open(os.path.join(a.out, 'results.jsonl'), 'a', encoding='utf-8') as f:
            f.write(line + '\n')
        with lock:
            st['result'] = f'{name}  {"⚠ " + reason if reason else "正常结束"}\n' + \
                '\n'.join(f'  {kk}: {vv}' for kk, vv in summ.items())
            st['state'], st['running'] = '空闲', False
        node.get_logger().info(line)

    class Hd(http.server.BaseHTTPRequestHandler):
        def log_message(self, *_):
            pass

        def reply(self, body, ctype='text/plain; charset=utf-8'):
            body = body.encode() if isinstance(body, str) else body
            self.send_response(200)
            self.send_header('Content-Type', ctype)
            self.send_header('Cache-Control', 'no-store')
            self.end_headers()
            self.wfile.write(body)

        def do_GET(self):
            if self.path.startswith('/status'):
                with lock:
                    if self.client_address[0] == st['owner']:          # 只有点"开始"的那个页面算心跳
                        st['hb'] = time.time()
                    gap = guard.gap(steer_of_turn(st['turn']))
                    return self.reply(json.dumps(dict(
                        state=st['state'], estop=safety.estop, v=S.v, gap=-1 if gap is None else min(gap, 9.9),
                        r=math.hypot(S.x, S.y), pwm=st['pwm'], turn=st['turn'], tilt=S.tilt_deg(), result=st['result']),
                        ensure_ascii=False), 'application/json')
            self.reply(PAGE.replace('__DEFAULTS__', json.dumps(DEFAULTS)), 'text/html; charset=utf-8')

        def do_POST(self):
            n = int(self.headers.get('Content-Length') or 0)
            body = self.rfile.read(n).decode() if n else ''
            if self.path == '/estop':
                with lock:
                    safety.estop = True
                return self.reply('急停')
            if self.path == '/clear':
                with lock:
                    safety.estop = False
                return self.reply('急停已解除')
            if self.path == '/start':
                req = json.loads(body)
                with lock:
                    if safety.estop:
                        return self.reply('急停中,先解除')
                    if st['running']:
                        return self.reply('正在跑')
                    st['running'], st['hb'], st['result'] = True, time.time(), ''
                    st['owner'] = self.client_address[0]
                params = {k: float(v) for k, v in req['params'].items() if k in DEFAULTS[req['prog']]}
                threading.Thread(target=run, args=(req['prog'], params, bool(req['guard']), float(req['fence'])),
                                 daemon=True).start()
                return self.reply(f'开始 {req["prog"]}')
            self.reply('?')

    def quit_(*_):
        safety.estop = True                                 # 正在跑的脚本下一拍就停
        for _ in range(10):
            send(PWM_MID, TURN_MID)
            time.sleep(0.05)
        os._exit(0)
    signal.signal(signal.SIGTERM, quit_)
    signal.signal(signal.SIGINT, quit_)

    srv = http.server.ThreadingHTTPServer(('0.0.0.0', PORT), Hd)
    srv.daemon_threads = True
    node.get_logger().info(f'动力学测试 http://<车IP>:{PORT}/,数据存 {a.out}')
    srv.serve_forever()


if __name__ == '__main__':
    main()
