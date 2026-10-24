#!/usr/bin/env python3
"""HW-Autocar 网页遥控:HTTP + WebSocket(纯标准库) + ROS2 节点。

浏览器协议沿用 Astra-Rover 的 preview.html:
  t <油门‰> <方向盘‰> <刹车‰>   油门 -1000..1000(R 档为负),方向盘 >0 = 左
  e <0|1> 急停   l <0|1|2> 排挡 P/D/R   y <0|1|2> 模式 Sloth/Roaming/MAD MAX
策略模式下模式 = 策略限速(CarParams.policy_mode_caps),发 /policy_speed_cap 给 road_car_node。
输出和 racecar_teleop.py 一样发 /teleop_cmd_vel:linear.x = 电调 PWM,angular.z = 舵机值。
"""
import base64, hashlib, json, math, os, signal, socket, struct, sys, threading, time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from common.params import CarParams
from common.pwm import turn_to_steer

import rclpy
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, LaserScan
from std_msgs.msg import Bool, Float32

from vehicle.scan_guard import ScanGuard, guard_stop
from vehicle.scan_view import scan_payload
from vehicle.mapping_control import MappingControl

PORT = 8080
HTML = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'index.html')

# PWM 约定:中位和转向值从实测参数读取;前进/倒车起点仍按网页遥控设置。
# ⚠️ 顶速上限放在服务端,不信任浏览器发来的数。数值偏保守,架空实测后再往上调。
SPEED_MID, FWD_START, REV_START = CarParams().pwm_mid, 1600, 1500
# Sloth 1600 = 平台 ~0.67 m/s(对应策略限速 0.7);Roaming 1620 ~1.1~1.2;MAD MAX 1640(2026-10-21 用户定,
# 高位曲线外推 ~1.6 m/s,没实测;电调锁在低位时会快得多)。只有手动 MAD MAX 超过策略的 PWM 硬上限。
FWD_CAP = [1600, 1620, 1640]          # Sloth / Roaming / MAD MAX
# 手动前进 PWM 每秒最多加这么多:摇杆一把推到底不再瞬间给满,2026-10-20 MAD MAX(当时车上 1700)直跳翘头。
# 只限加油,松油/回中位不限。1600 -> 1620 用 0.5 s。
FWD_SLEW = 40.0
POLICY_CAPS = CarParams().policy_mode_caps
# 手动刹车:按刹车且车往前走得比 brake_release_speed 快就发 brake_pwm,慢了 / 编码器没数据回中位滑行
# (同策略节点:刹到低速还给低 PWM 会变倒车)。原来按刹车只回中位滑行。
BRAKE_PWM, BRAKE_RELEASE = CarParams().brake_pwm, CarParams().brake_release_speed
# ⚠️ 倒车:厂家 teleop 用 1360 起步,实车上会窜出去(2026-09-17)。
# 电调存在起转平台，平台速度随工况变化；手动 PWM 不保证目标车速。
# 倒车仅保留 PWM 上限；不根据编码器速度截断输出，不能保证实际车速上限。
REV_CAP = 1470
# 手动低速保留历史占空比方式；实际速度受起转、滞回和滑行影响。
# 所以摇杆前 CREEP_SPAN 段不调 PWM,而是在起步 PWM 和中位之间按比例通断,
# 周期 0.3s；占空比与平均速度不保证线性，策略模式另走编码器闭环。
CREEP_SPAN = 0.45                     # 摇杆前 45% 行程是占空比低速区
DUTY_TICKS = 6                        # 6 个 20Hz 周期 = 0.3s
# 舵机中位/满舵幅度从 common/params.py 读,别在这里写死 —— 2026-09-22 之前
# 四个脚本各抄一份 84,改中位时漏改一个就会两套行为(当前中位为 81)。
TURN_MID, TURN_SPAN = CarParams().turn_mid, CarParams().turn_span
DEADMAN = 0.3                         # 这么久没收到 t 就回中位
# 手动防撞(2026-10-24):按当前转角的扫掠路径看净空。净空小于按车速算的刹停距离 guard_stop(同策略)
# 就把前进档位压到最慢一档、让车减速(不刹车,用户定手动不用那么严);只有贴到 GUARD_STOP 才回中位。
# 原来只看正前方、固定 0.8 m 才压档,高速根本减不下来。
GUARD_STOP = 0.25                     # 保险杠前净空小于这个就不许再往前(米)
GUARD_V_STALE = 3.0                   # 编码器没数据时按这个车速算减速距离(宁可早减)
NEUTRAL_HOLD = 1.0                    # 回中位后续发这么久再静默,不跟导航的指令抢底盘
POLICY_THR = 50                       # 策略模式下油门推过这个值(0..1000)才放行,松手即停
# 策略模式的"摇杆数据多久算断"。比手动的 DEADMAN 宽:松手时网页立刻发 t 0,油门那条判据
# 马上就断,不靠这个超时;它只管"网断了"。手机热点上 t 的间隔会抖过 0.3s,2026-09-24 实车
# 日志里使能/停车一秒翻几次,每翻一次策略节点就回中位重新爬 3 秒 —— 现象是推到顶也半天不走。
POLICY_DEADMAN = 0.6
CONTROL_LEASE = 1.0                  # 一秒没收到控制指令,其他页面才能接管
REVERSE_PAUSE = 0.3                   # 前进<->倒车换向前先给中位,电调要求


class State:
    def __init__(self):
        self.lock = threading.Lock()
        self.thr = self.steer = self.brake = 0
        self.last_t = 0.0
        self.estop = 0
        self.range = 1
        self.mode = 1
        self.guard = 1                # 防撞开关,雷达掉线可以在网页上关掉
        self.policy = 0               # 策略模式:开了之后推着摇杆才让 policy_node 开车
        self.gap = None               # 保险杠前净空,None = 雷达没数据
        self.sg = None                # Bridge 的 ScanGuard,手动防撞按当前转角算净空
        self.blocked = 0
        self.rev_block = 0            # 2=编码器无数据不许倒车；不再做超速截断
        self.tick = 0                 # 20Hz 计数,占空比用
        self.last_dir = 0
        self.dir_change_t = 0.0
        self.pwm, self.turn = SPEED_MID, TURN_MID
        self.fwd_pwm, self.fwd_t = FWD_START, 0.0   # 手动前进上一拍的 PWM 和时刻(FWD_SLEW 用)
        self.spd = 0.0; self.enc_t = 0.0
        self.x = self.y = self.th = 0.0
        self.wz = self.ax = self.ay = 0.0
        self.scan = None
        self.scan_received = 0.0
        self.t_src = None             # 最近一条摇杆指令是哪个客户端发的(查"谁在开车")
        self.owner = None             # 同一时刻仅一个网页能改驾驶指令和模式
        self.owner_t = 0.0


S = State()
mapping = None


def creep_or_pwm(t, start, cap):
    """t 是 0..1 的行程。前 CREEP_SPAN 段用占空比跑低速,之后才真的加 PWM。"""
    if t >= CREEP_SPAN:
        return start + (t - CREEP_SPAN) / (1 - CREEP_SPAN) * (cap - start)
    on = round(t / CREEP_SPAN * DUTY_TICKS)          # 这一周期里有几拍通电
    return start if S.tick % DUTY_TICKS < on else SPEED_MID


def policy_held(now):
    """策略模式下,现在该不该放行 policy_node 开车。

    条件和人手上的动作一一对应:模式开着、没急停、摇杆数据是新的、油门确实推着。
    任何一条断了就返回 False —— policy_node 那边把 /policy_enable 当心跳,
    收不到就自己回中位,所以"松手即停"不依赖这里再补一条停车指令。
    """
    with S.lock:
        return bool(S.policy and not S.estop
                    and now - S.last_t <= POLICY_DEADMAN and S.thr > POLICY_THR)


def compute(now):
    """返回 (pwm, turn, active)。active=False 表示应该回中位。"""
    with S.lock:
        if S.estop or S.range == 0 or now - S.last_t > DEADMAN:
            return SPEED_MID, TURN_MID, False
        turn = round(TURN_MID + max(-1, min(1, S.steer / 1000)) * TURN_SPAN)
        t = max(-1.0, min(1.0, S.thr / 1000))
        want = 1 if t > 0 else -1 if t < 0 else 0
        if S.brake > 20 and now - S.enc_t < 0.3 and S.spd > BRAKE_RELEASE:
            return BRAKE_PWM, turn, True
        if S.brake > 20 or want == 0:
            return SPEED_MID, turn, True
        if want != S.last_dir:
            if S.last_dir != 0:
                S.dir_change_t = now
            S.last_dir = want
        if now - S.dir_change_t < REVERSE_PAUSE:
            return SPEED_MID, turn, True
        S.tick += 1
        if want > 0:
            cap = FWD_CAP[S.mode]
            if S.guard:
                fresh = now - S.enc_t < 0.3
                gap = S.sg.gap(turn_to_steer(turn)) if S.sg is not None else None
                if gap is None or gap < GUARD_STOP:   # 雷达没数据也当挡着,总比撞上去强
                    S.blocked = 1
                    return SPEED_MID, turn, True
                if gap < guard_stop(max(S.spd, 0.0) if fresh else GUARD_V_STALE, CarParams()):
                    cap = FWD_CAP[0]
            S.blocked = 0
            pwm = creep_or_pwm(t, FWD_START, cap)
            if now - S.fwd_t > 0.15:                  # 上一拍不在前进(回中位/倒车/截停):从起步 PWM 重新爬
                S.fwd_pwm, S.fwd_t = FWD_START, now
            pwm = min(pwm, S.fwd_pwm + FWD_SLEW * (now - S.fwd_t))
            S.fwd_pwm, S.fwd_t = max(pwm, FWD_START), now
        else:
            fresh = now - S.enc_t < 0.3
            if not fresh:
                # 编码器没数据就不知道倒多快,宁可不倒 —— 这车的 USB 掉过线
                S.rev_block = 2
                return SPEED_MID, turn, True
            S.rev_block = 0
            pwm = creep_or_pwm(-t, REV_START, REV_CAP)    # 倒车两端都比中位小
        return round(pwm), turn, True


class Bridge(Node):
    def __init__(self):
        super().__init__('web_teleop')
        topic = self.declare_parameter('cmd_topic', '/teleop_cmd_vel').value
        self.pub = self.create_publisher(Twist, topic, 5)
        self.enable_pub = self.create_publisher(Bool, '/policy_enable', 1)
        self.cap_pub = self.create_publisher(Float32, '/policy_speed_cap', 1)
        self.create_subscription(Odometry, '/encoder', self.on_enc, 10)
        # 策略模式下油门/转向是策略节点发的,这里不发;订阅回来给网页显示真实 PWM(2026-09-29)
        self.create_subscription(Twist, topic, self.on_cmd, 5)
        self.create_subscription(Odometry, '/odom_combined', self.on_odom, 10)
        self.create_subscription(Imu, '/IMU_data', self.on_imu, 10)
        self.guard = S.sg = ScanGuard()
        self.create_subscription(LaserScan, '/scan', self.on_scan, 1)
        self.idle_since = 0.0
        self.last_held = False
        self.create_timer(0.05, self.tick)
        self.get_logger().info(f'publishing to {topic}, http://0.0.0.0:{PORT}')

    def on_cmd(self, m):
        with S.lock:
            if S.policy:
                S.pwm, S.turn = round(m.linear.x), round(m.angular.z)

    def on_enc(self, m):
        with S.lock:
            S.spd = m.twist.twist.linear.x; S.enc_t = time.time()

    def on_odom(self, m):
        q = m.pose.pose.orientation
        with S.lock:
            S.x, S.y = m.pose.pose.position.x, m.pose.pose.position.y
            S.th = math.degrees(math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)))

    def on_imu(self, m):
        with S.lock:
            S.wz = math.degrees(m.angular_velocity.z)
            S.ax, S.ay = m.linear_acceleration.x, m.linear_acceleration.y

    def on_scan(self, m):
        self.guard.update(m)
        gap = self.guard.gap()
        scan = scan_payload(m)
        with S.lock:
            S.gap = gap
            S.scan = scan
            S.scan_received = time.monotonic()

    def tick(self):
        now = time.time()
        with S.lock:
            if S.gap is not None and time.time() - self.guard.stamp > 1.0:
                S.gap = None                      # 雷达掉线,别拿旧值当真

        with S.lock:
            policy_mode = bool(S.policy)
        if policy_mode:
            # 策略模式下这里一个字都不发。摇杆**只**是死人开关,不碰转向和油门。
            #
            # 2026-09-20 实车教训:原来写成"推着才不发、松手就接管",而网页摇杆
            # 前后和左右是同一根 —— 往侧面推时油门分量掉到阈值以下,服务器立刻
            # 按摇杆转向接管,松一点又切回策略。控制权在两者之间来回跳,现象是
            # "推杆能扳动方向"和"车莫名其妙往墙上走"。
            # 松手后的停车由 policy_node 的心跳超时负责,再不行还有 watchdog。
            held = policy_held(now)
            if held != self.last_held:
                with S.lock:
                    age, thr, src = now - S.last_t, S.thr, S.t_src
                log_event(f'放行 {"开" if held else "关"}(油门 {thr},摇杆 {age:.2f}s 前,来自 {src})')
                self.last_held = held
            with S.lock:
                cap = POLICY_CAPS[S.mode]
            self.cap_pub.publish(Float32(data=float(cap)))     # 先发限速再发放行
            self.enable_pub.publish(Bool(data=held))
            return

        pwm, turn, active = compute(now)
        if active:
            self.idle_since = 0.0
        else:
            if not self.idle_since:
                self.idle_since = now
            if now - self.idle_since > NEUTRAL_HOLD:
                return
        with S.lock:
            S.pwm, S.turn = pwm, turn
        tw = Twist(); tw.linear.x = float(pwm); tw.angular.z = float(turn)
        self.pub.publish(tw)


# ---------------- WebSocket(RFC 6455 最小实现) ----------------
clients = set()
clients_lock = threading.Lock()


EVENTS = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'events.log')


def log_event(text):
    """谁连上/断开、谁在推摇杆、策略开关和放行的每次变化。车自己跑的时候靠它查是谁的指令。"""
    try:
        with open(EVENTS, 'a') as f:
            f.write(time.strftime('%H:%M:%S') + f'.{int(time.time() * 1000) % 1000:03d} {text}\n')
    except OSError:
        pass


def ws_send(conn, text, wait=True):
    """wait=False 给遥测用:这个客户端收不动就断掉它,不许拖住别人。

    遥测是一个线程轮流发给所有页面。原来一律 sendall,某个页面的连接半死不活(手机热点抖、
    后台标签页被浏览器节流)时 sendall 会一直阻塞 —— 所有页面都收不到遥测,按钮状态全靠遥测
    刷新,于是"点了没反应、刷新也没用"(2026-09-24 实车)。
    """
    data = text.encode()
    n = len(data)
    head = bytes([0x81, n]) if n < 126 else bytes([0x81, 126]) + struct.pack('>H', n) if n < 65536 \
        else bytes([0x81, 127]) + struct.pack('>Q', n)
    sock, lock = conn
    if wait:
        with lock:
            sock.sendall(head + data)
        return
    if not lock.acquire(timeout=0.05):
        return                              # 这一帧跳过,下一帧再说
    try:
        buf = head + data
        if sock.send(buf, socket.MSG_DONTWAIT) != len(buf):
            raise BlockingIOError           # 只发出半帧,流已经坏了,只能断
    except (BlockingIOError, InterruptedError):
        log_event('遥测发不出去,断开一个收不动的客户端')
        try:
            sock.shutdown(socket.SHUT_RDWR)  # 它的接收线程会随之退出,走正常的断线清理
        except OSError:
            pass
    finally:
        lock.release()


def recv_exact(sock, n):
    buf = b''
    while len(buf) < n:
        chunk = sock.recv(n - len(buf))
        if not chunk:
            raise ConnectionError
        buf += chunk
    return buf


def ws_recv(sock):
    b1, b2 = recv_exact(sock, 2)
    op, n = b1 & 0x0F, b2 & 0x7F
    if n == 126:
        n = struct.unpack('>H', recv_exact(sock, 2))[0]
    elif n == 127:
        n = struct.unpack('>Q', recv_exact(sock, 8))[0]
    mask = recv_exact(sock, 4) if b2 & 0x80 else b'\0\0\0\0'
    data = bytes(c ^ mask[i % 4] for i, c in enumerate(recv_exact(sock, n)))
    return op, data


def handle_cmd(line, who='?'):
    p = line.split()
    if not p:
        return None
    if p[0] in ('p', 'g', 'e'):
        log_event(f'{who} 发 {line.strip()}')
    try:
        with S.lock:
            now = time.time()
            # 急停任何页面都能按;解除急停和其他控制只允许当前控制页面。
            if p[0] == 'e' and (len(p) == 1 or int(p[1]) != 0):
                S.estop = 1
                return '急停'
            if p[0] in ('t', 'e', 'l', 'y', 'g', 'p'):
                if S.owner != who:
                    if S.owner is not None and now - S.owner_t <= CONTROL_LEASE:
                        return '另一页面正在控制小车'
                    # 交接时先撤掉旧页面的油门与心跳。
                    S.thr = S.steer = S.brake = 0
                    S.last_t = 0.0
                    S.owner = who
                    log_event(f'控制页面换成 {who}')
                S.owner_t = now
            if p[0] == 't' and len(p) >= 3:
                S.thr, S.steer = int(p[1]), int(p[2])
                S.brake = int(p[3]) if len(p) > 3 else 0
                S.last_t = time.time()
                if who != S.t_src:
                    log_event(f'摇杆指令来源换成 {who}(油门 {S.thr})')
                    S.t_src = who
            elif p[0] == 'e':
                S.estop = int(p[1]) if len(p) > 1 else 1
                return '急停' if S.estop else '急停解除'
            elif p[0] == 'l' and int(p[1]) in (0, 1, 2):
                S.range = int(p[1])
            elif p[0] == 'y' and int(p[1]) in (0, 1, 2):
                S.mode = int(p[1])
            elif p[0] == 'g':
                S.guard = 1 if int(p[1]) else 0
                return '防撞已开' if S.guard else '⚠ 防撞已关,前进不再看雷达'
            elif p[0] == 'p':
                S.policy = 1 if int(p[1]) else 0
                return ('⚠ 策略模式:推着摇杆车自己开,松手即停'
                        if S.policy else '策略模式已关,回到手动遥控')
            elif p[0] == 'h':
                return 'HW-Autocar web teleop 已连接'
    except (ValueError, IndexError):
        return '命令格式错误: ' + line
    return None


def telemetry():
    with S.lock:
        return json.dumps({
            'spd': round(S.spd * 1000), 'x': round(S.x * 1000), 'y': round(S.y * 1000),
            'th': round(S.th, 1), 'wz': round(S.wz, 1), 'ax': round(S.ax, 2), 'ay': round(S.ay, 2),
            'pwm': S.pwm, 'turn': S.turn, 'es': S.estop, 'rng': S.range, 'mode': S.mode,
            'enc': round(time.time() - S.enc_t, 1) if S.enc_t else -1,
            'guard': S.guard, 'blocked': S.blocked, 'revb': S.rev_block,
            'pol': S.policy, 'polrun': int(bool(S.policy and not S.estop
                                                and time.time() - S.last_t <= POLICY_DEADMAN
                                                and S.thr > POLICY_THR)),
            'gap': -1 if S.gap is None else round(min(S.gap, 9.99), 2),
        })


def telemetry_loop():
    while True:
        time.sleep(0.1)
        msg = telemetry()
        with clients_lock:
            conns = list(clients)
        for c in conns:
            try:
                ws_send(c, msg, wait=False)
            except OSError:
                pass


class Handler(BaseHTTPRequestHandler):
    # ⚠️ 默认是 HTTP/1.0,握手会回 "HTTP/1.0 101",部分浏览器直接拒绝 WebSocket
    protocol_version = 'HTTP/1.1'

    def log_message(self, *a):
        pass

    def reply_json(self, value, code=200):
        body = json.dumps(value).encode()
        self.send_response(code)
        self.send_header('Content-Type', 'application/json')
        self.send_header('Cache-Control', 'no-store')
        self.send_header('Content-Length', str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def do_POST(self):
        # Custom header + same-origin policy prevent cross-site form commands.
        if self.headers.get('X-Mapping-Control') != '1':
            self.reply_json({'error': 'Missing control header'}, 403); return
        if self.headers.get('Origin') not in (None, 'http://' + self.headers.get('Host', '')):
            self.reply_json({'error': 'Origin rejected'}, 403); return
        if self.path not in ('/mapping/start', '/mapping/stop'):
            self.reply_json({'error': 'Not found'}, 404); return
        accepted = mapping.request(self.path.rsplit('/', 1)[-1])
        self.reply_json({'accepted': accepted}, 202 if accepted else 409)

    def do_GET(self):
        if self.path == '/mapping/status':
            self.reply_json(mapping.status()); return
        if self.path.split('?')[0] == '/scan.json':
            # Separate HTTP polling: scan rendering never sends a driving command.
            with S.lock:
                scan = S.scan
                age = time.monotonic() - S.scan_received if scan else None
            body = json.dumps({'scan': scan if age is not None and age < 1.0 else None,
                               'age': age}, allow_nan=False).encode()
            self.send_response(200)
            self.send_header('Content-Type', 'application/json')
            self.send_header('Cache-Control', 'no-store')
            self.send_header('Content-Length', str(len(body)))
            self.end_headers()
            self.wfile.write(body)
            return
        if self.path == '/ws' and self.headers.get('Upgrade', '').lower() == 'websocket':
            return self.websocket()
        if self.path.split('?')[0] not in ('/', '/index.html'):
            self.send_error(404); return
        with open(HTML, 'rb') as f:
            body = f.read()
        self.send_response(200)
        self.send_header('Content-Type', 'text/html; charset=utf-8')
        self.send_header('Content-Length', str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def websocket(self):
        key = self.headers['Sec-WebSocket-Key']
        acc = base64.b64encode(hashlib.sha1((key + '258EAFA5-E914-47DA-95CA-C5AB0DC85B11').encode()).digest())
        self.send_response(101)
        self.send_header('Upgrade', 'websocket')
        self.send_header('Connection', 'Upgrade')
        self.send_header('Sec-WebSocket-Accept', acc.decode())
        self.end_headers()
        sock = self.connection
        sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        conn = (sock, threading.Lock())
        who = '%s:%d' % self.client_address[:2]
        with clients_lock:
            clients.add(conn)
            log_event(f'{who} 连上,共 {len(clients)} 个页面')
        try:
            while True:
                op, data = ws_recv(sock)
                if op == 8:
                    break
                if op == 9:
                    with conn[1]:
                        sock.sendall(bytes([0x8A, len(data)]) + data)
                elif op == 1:
                    reply = handle_cmd(data.decode(errors='replace'), who)
                    if reply:
                        ws_send(conn, reply)
        except (ConnectionError, OSError):
            pass
        finally:
            with clients_lock:
                clients.discard(conn)
                log_event(f'{who} 断开,剩 {len(clients)} 个页面')
            # 断线立刻停,不等死人开关
            with S.lock:
                if S.owner == who:
                    S.thr = S.steer = S.brake = 0
                    S.last_t = 0.0
                    S.owner = None
                    S.owner_t = 0.0
            self.close_connection = True


def main():
    global mapping
    # ⚠️ 不用 rclpy 的信号处理:它收到 SIGTERM 只关 ROS,HTTP 线程照跑,进程停不下来。
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = Bridge()
    mapping = MappingControl(lambda: node.count_publishers('/map'))

    def stop(*_):
        # 退出前给底盘回中位 —— 不确定底盘收不到指令时会不会自己停
        tw = Twist(); tw.linear.x = float(SPEED_MID); tw.angular.z = float(TURN_MID)
        for _ in range(3):
            node.pub.publish(tw); time.sleep(0.05)
        os._exit(0)
    signal.signal(signal.SIGTERM, stop)
    signal.signal(signal.SIGINT, stop)

    threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()
    threading.Thread(target=telemetry_loop, daemon=True).start()
    ThreadingHTTPServer.daemon_threads = True
    ThreadingHTTPServer(('0.0.0.0', PORT), Handler).serve_forever()


if __name__ == '__main__':
    main()
