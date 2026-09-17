#!/usr/bin/env python3
"""HW-Autocar 网页遥控:HTTP + WebSocket(纯标准库) + ROS2 节点。

浏览器协议沿用 Astra-Rover 的 preview.html:
  t <油门‰> <方向盘‰> <刹车‰>   油门 -1000..1000(R 档为负),方向盘 >0 = 左
  e <0|1> 急停   l <0|1|2> 排挡 P/D/R   y <0|1|2> 模式 Sloth/Roaming/MAD MAX
输出和 racecar_teleop.py 一样发 /teleop_cmd_vel:linear.x = 电调 PWM,angular.z = 舵机值。
"""
import base64, hashlib, json, math, os, signal, socket, struct, threading, time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import rclpy
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, LaserScan

from scan_guard import ScanGuard

PORT = 8080
HTML = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'index.html')

# PWM 约定取自 racecar_teleop.py:中位 1550,前进 1600 起步,倒车 1360 起步;转向中位 84,±60。
# ⚠️ 顶速上限放在服务端,不信任浏览器发来的数。数值偏保守,架空实测后再往上调。
SPEED_MID, FWD_START, REV_START = 1550, 1600, 1500
FWD_CAP = [1610, 1640, 1700]          # Sloth / Roaming / MAD MAX
# ⚠️ 倒车:厂家 teleop 用 1360 起步,实车上会窜出去(2026-09-17)。
# 电调没有渐进区(实测前进 1595->0、1600->470mm/s 一步到位),光调 PWM 保证不了速度,
# 所以这里既把 PWM 收到贴近中位,又用编码器做闭环硬限速 REV_SPEED_CAP。
REV_CAP = 1470
REV_SPEED_CAP = 0.40                  # m/s,倒车超过就立刻回中位
# 低速靠占空比:电调解锁就是 470mm/s,1599->0、1600->470 中间没有值(2026-09-17 实测)。
# 所以摇杆前 CREEP_SPAN 段不调 PWM,而是在起步 PWM 和中位之间按比例通断,
# 平均速度 ≈ 占空比 x 最低速度。周期 0.3s:再快电调跟不上,再慢车会一顿一顿。
CREEP_SPAN = 0.45                     # 摇杆前 45% 行程是占空比低速区
DUTY_TICKS = 6                        # 6 个 20Hz 周期 = 0.3s
TURN_MID, TURN_SPAN = 84, 60
DEADMAN = 0.3                         # 这么久没收到 t 就回中位
GUARD_STOP = 0.25                     # 保险杠前净空小于这个就不许再往前(米)
GUARD_SLOW = 0.8                      # 小于这个就把前进档位压到最慢一档
NEUTRAL_HOLD = 1.0                    # 回中位后续发这么久再静默,不跟导航的指令抢底盘
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
        self.gap = None               # 保险杠前净空,None = 雷达没数据
        self.blocked = 0
        self.rev_block = 0            # 1=倒车超速被截 2=编码器无数据不许倒车
        self.tick = 0                 # 20Hz 计数,占空比用
        self.last_dir = 0
        self.dir_change_t = 0.0
        self.pwm, self.turn = SPEED_MID, TURN_MID
        self.spd = 0.0; self.enc_t = 0.0
        self.x = self.y = self.th = 0.0
        self.wz = self.ax = self.ay = 0.0


S = State()


def creep_or_pwm(t, start, cap):
    """t 是 0..1 的行程。前 CREEP_SPAN 段用占空比跑低速,之后才真的加 PWM。"""
    if t >= CREEP_SPAN:
        return start + (t - CREEP_SPAN) / (1 - CREEP_SPAN) * (cap - start)
    on = round(t / CREEP_SPAN * DUTY_TICKS)          # 这一周期里有几拍通电
    return start if S.tick % DUTY_TICKS < on else SPEED_MID


def compute(now):
    """返回 (pwm, turn, active)。active=False 表示应该回中位。"""
    with S.lock:
        if S.estop or S.range == 0 or now - S.last_t > DEADMAN:
            return SPEED_MID, TURN_MID, False
        turn = round(TURN_MID + max(-1, min(1, S.steer / 1000)) * TURN_SPAN)
        t = max(-1.0, min(1.0, S.thr / 1000))
        want = 1 if t > 0 else -1 if t < 0 else 0
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
                gap = S.gap
                if gap is None or gap < GUARD_STOP:   # 雷达没数据也当挡着,总比撞上去强
                    S.blocked = 1
                    return SPEED_MID, turn, True
                if gap < GUARD_SLOW:
                    cap = FWD_CAP[0]
            S.blocked = 0
            pwm = creep_or_pwm(t, FWD_START, cap)
        else:
            fresh = now - S.enc_t < 0.3
            if not fresh or abs(S.spd) > REV_SPEED_CAP:
                # 编码器没数据就不知道倒多快,宁可不倒 —— 这车的 USB 掉过线
                S.rev_block = 1 if fresh else 2
                return SPEED_MID, turn, True
            S.rev_block = 0
            pwm = creep_or_pwm(-t, REV_START, REV_CAP)    # 倒车两端都比中位小
        return round(pwm), turn, True


class Bridge(Node):
    def __init__(self):
        super().__init__('web_teleop')
        topic = self.declare_parameter('cmd_topic', '/teleop_cmd_vel').value
        self.pub = self.create_publisher(Twist, topic, 5)
        self.create_subscription(Odometry, '/encoder', self.on_enc, 10)
        self.create_subscription(Odometry, '/odom_combined', self.on_odom, 10)
        self.create_subscription(Imu, '/IMU_data', self.on_imu, 10)
        self.guard = ScanGuard()
        self.create_subscription(LaserScan, '/scan', self.on_scan, 1)
        self.idle_since = 0.0
        self.create_timer(0.05, self.tick)
        self.get_logger().info(f'publishing to {topic}, http://0.0.0.0:{PORT}')

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
        with S.lock:
            S.gap = gap

    def tick(self):
        now = time.time()
        with S.lock:
            if S.gap is not None and time.time() - self.guard.stamp > 1.0:
                S.gap = None                      # 雷达掉线,别拿旧值当真
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


def ws_send(conn, text):
    data = text.encode()
    n = len(data)
    head = bytes([0x81, n]) if n < 126 else bytes([0x81, 126]) + struct.pack('>H', n) if n < 65536 \
        else bytes([0x81, 127]) + struct.pack('>Q', n)
    lock = conn[1]
    with lock:
        conn[0].sendall(head + data)


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


def handle_cmd(line):
    p = line.split()
    if not p:
        return None
    try:
        with S.lock:
            if p[0] == 't' and len(p) >= 3:
                S.thr, S.steer = int(p[1]), int(p[2])
                S.brake = int(p[3]) if len(p) > 3 else 0
                S.last_t = time.time()
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
                ws_send(c, msg)
            except OSError:
                pass


class Handler(BaseHTTPRequestHandler):
    # ⚠️ 默认是 HTTP/1.0,握手会回 "HTTP/1.0 101",部分浏览器直接拒绝 WebSocket
    protocol_version = 'HTTP/1.1'

    def log_message(self, *a):
        pass

    def do_GET(self):
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
        with clients_lock:
            clients.add(conn)
        try:
            while True:
                op, data = ws_recv(sock)
                if op == 8:
                    break
                if op == 9:
                    with conn[1]:
                        sock.sendall(bytes([0x8A, len(data)]) + data)
                elif op == 1:
                    reply = handle_cmd(data.decode(errors='replace'))
                    if reply:
                        ws_send(conn, reply)
        except (ConnectionError, OSError):
            pass
        finally:
            with clients_lock:
                clients.discard(conn)
            # 断线立刻停,不等死人开关
            with S.lock:
                S.thr = S.steer = 0
                S.last_t = 0.0
            self.close_connection = True


def main():
    # ⚠️ 不用 rclpy 的信号处理:它收到 SIGTERM 只关 ROS,HTTP 线程照跑,进程停不下来。
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = Bridge()

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
