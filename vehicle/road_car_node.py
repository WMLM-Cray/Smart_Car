#!/usr/bin/env python3
"""road_v2 实车节点:转向 + 目标速度双输出模型,20Hz。

和 steer_policy_node(v1:只出转向、目标速度恒定)只差"大脑":观测、推理、转角与目标速度
都走 common.road_runtime,和仿真节点 road_policy_node、训练环境是同一份代码。
安全外壳照搬 v1 在车上验证过的做法:
  /policy_enable 是心跳(0.3s),松手立刻发中位;雷达/编码器/IMU 任一过期回中位;
  沿当前转角扫掠路径的防撞截停(0.7 m/s 以下 0.25m,更快按车速放大,见 guard_stop)+ 连续截停 3s 报卡死;
  限速跟网页模式走(/policy_speed_cap,Sloth/Roaming/MAD MAX),不超过 --speed-cap;
  SpeedLoop 编码器闭环出 PWM(爬升 30 PWM/s;硬上限 1620,网页 MAD MAX 档 1640);SIGINT/SIGTERM 时连发中位。
模型给的目标速度每拍都在变(0.45 ~ 速度上限)。SpeedLoop 的 floor 自标定按"当前目标"判塌速,
目标往下调(弯前减速)不会被当成塌速。按变化目标跟随的表现还没在车上测过,先架空再低速落地。

运行(车上,先 source vehicle/ros_env.sh):
  python3 -m vehicle.road_car_node models/road_v2/candidate_167.npz --speed-cap 0.5
输出 /teleop_cmd_vel:linear.x = PWM(电调刻度),angular.z = 舵机刻度。仿真用 road_policy_node。
"""
from __future__ import annotations

import argparse
import math
import os
import signal
import sys
import time
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parent.parent
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from common.bev import lidar_points
from common.params import CarParams
from common.pwm import steer_to_turn
from common.bev_policy import BevPolicy
from common.bev_runtime import BevRuntime
from common.road_policy import V_LO, RoadPolicy
from common.road_runtime import STOP, RoadRuntime
from vehicle.scan_guard import ScanGuard
from dataclasses import replace

from vehicle.speed_loop import PWM_FAST_MAX, PWM_HARD_MAX, SpeedLoop, SpeedLoopConfig

CMD_TOPIC = '/teleop_cmd_vel'
ENC_TOPIC = '/encoder'
FREE_TOPIC = '/ground_free'     # vehicle/ground_seg_node.py:车体系地面点,header.stamp = 拍摄时刻
CONTROL_PERIOD = 0.05       # 20Hz,和训练的 CONTROL_DT 一致
SENSOR_TIMEOUT = 0.3        # 实测 /encoder 34Hz、/scan 12Hz、/IMU_data 55Hz
ENABLE_TIMEOUT = 0.3        # /policy_enable 是心跳不是开关
GUARD_STOP = 0.25           # 同 steer_policy_node:保险杠前净空小于这个就回中位(米)
CAP_TOPIC = '/policy_speed_cap'     # vehicle/server.py 按网页模式发的限速(Float32,m/s)


def guard_stop(speed, p):
    """截停距离:GUARD_STOP 在 guard_base_speed 下实车验证过;更快时加上多出来的延迟距离和刹车距离。"""
    v0 = p.guard_base_speed
    extra = p.guard_latency * (speed - v0) + (speed * speed - v0 * v0) / (2 * p.guard_brake_decel)
    return GUARD_STOP + max(0.0, extra)


def guard_speed(clear, p):
    """guard_stop 的反函数:净空 clear 下不截停的最高车速;clear < GUARD_STOP 返回 0。"""
    if clear < GUARD_STOP:
        return 0.0
    L, D, v0 = p.guard_latency, p.guard_brake_decel, p.guard_base_speed
    c = clear - GUARD_STOP + L * v0 + v0 * v0 / (2 * D)
    return max(v0, D * (-L + math.sqrt(L * L + 2 * c / D)))
STUCK_SECONDS = 3.0         # 同 steer_policy_node:连续截停这么久就当卡死
# 转向避障:模型要走的路线 AVOID_CLEAR 内会碰到雷达障碍,就在 AVOID_STEERS 个候选转角里挑离模型最近、走得够远的;
# 都不够远就挑走得最远的。模型没见过迎面的障碍(仿真的路中间不会立一块板),会直直开过去(2026-10-15 走廊实测)
AVOID_CLEAR = 1.0
AVOID_STEERS = 15
AVOID_KEEP = 0.05           # 上一拍往哪边绕,这一拍同侧候选少算这么多 rad,别左右来回切
# 起步第一拍直接跳到这里;0.8 s 不动就回中位 0.3 s 再跳高 4(阶梯起步,见 SpeedLoopConfig.retry_hold:
# 10-12 实测电调不认从破静点以下慢爬上来的油门,回一次中位再直跳才认)。
# 破静点实测:11.24 V 和 12.5 V 都是 1594(09-23/24),09-30 高位 1594,10-12 走廊 ~1595;电压只挪 4~10。
# 跳到最低的 1594 下面一点 → 1593(SpeedLoopConfig 要求 < pwm_breakaway)。10-12 走廊实测电调接受从中位直跳
# (1590/1595/1600),车动 0.18–0.26 s,其中 ~0.2 s 是指令延迟;跳到 1586 再爬是 0.40 s,原来从 1540 慢爬 2.7 s。
# 跳得比破静点高几个也不猛:电调一过破静点就是平台速度(直跳 1600,动起来 0.3 s 后 0.44 m/s)。
# ⚠ 前提是电调锁在"高位"中位:电调上电那一刻收到 1540 才是高位;上电时收到的是 ~1500(车还没开机就先开
# 电调)会锁成低位,整条曲线下移 ~40(09-30:破静 1552,1575 已是 ~0.8 m/s),那时跳到 1593 等于直接
# ~1.5 m/s 冲出去。09-29 就是这么撤回的跳点。操作规程:车上服务起来、持续发 1540 之后再开电调。
LAUNCH_FROM = 1593
UP_RATE = 30.0              # 车在动时往上爬的速率 PWM/s;2026-10-22 用户定 30(原 20 偏保守)
FAST_CAP = 1.2              # 网页限速超过它(MAD MAX)才把 PWM 上限放到 PWM_FAST_MAX
# bev 模型开车时记全局地图(vehicle/bev_logbook.py,和影子模式同一份):一次放行开一段,
# 松手超过 SEG_GAP 才收尾存盘(网页松手/手机抖的短暂断开接着记;搬车回起点总要几秒)。
SEG_GAP = 5.0


class RoadCarController:
    """纯逻辑,不依赖 ROS;节点只管收发。

    tick() 返回 None(没使能,一个字都不发,不跟网页遥控抢)或 (PWM, 舵机刻度)。
    event 给节点打日志:None / 'stale'(心跳或传感器过期)/ 'guard'(截停)/ 'stuck'(截停超过 3s)。
    """

    def __init__(self, policy, speed_cap, params=None, loop=None, guard=None, book=None):
        self.p = params or CarParams()
        self.cap_max = float(speed_cap)                 # --speed-cap:网页模式再快也不超过它
        self.book = book            # bev_logbook.Logbook;只在使能时记
        self.release_t = None
        rt = BevRuntime if isinstance(policy, BevPolicy) else RoadRuntime
        self.rt = rt(policy, min(self.p.policy_mode_caps[0], self.cap_max), SENSOR_TIMEOUT, ENABLE_TIMEOUT, self.p)
        self.loop = loop or SpeedLoop(SpeedLoopConfig(launch_from=LAUNCH_FROM, up_rate=UP_RATE))
        self.guard = guard or ScanGuard()
        self.block_t = None         # 连续截停的起点
        self.seg_saved = None
        self.event = None
        self.gap = None
        self.cam_jpg = None         # 最新一张地面分割调试图(JPEG 字节),逐帧回看用
        self.avoid_side = 0.0       # 上一拍绕行的方向(+1 左 / -1 右 / 0 没绕)
        self.steer_cmd = None       # 这一拍实际打的转角(避障后)

    def neutral(self):
        """中位:油门回 1540、方向回正,速度环从中位重新爬(学到的 floor 保留)。"""
        self.loop.reset()
        return self.p.pwm_mid, steer_to_turn(0.0, self.p)

    def on_scan(self, msg, now):
        self.rt.on_scan(msg.ranges, msg.angle_min, msg.angle_increment, now)
        self.guard.update(msg)
        if self.book is not None and self.rt.enabled:
            sx, sy, yaw = self.rt.scan_pose
            self.book.add_lidar(lidar_points(self.rt.scan, self.rt.angles, self.rt.scan_pose, self.p,
                                             self.rt.bev.lidar_yaw, 8.0),
                                (sx + self.p.lidar_x * math.cos(yaw), sy + self.p.lidar_x * math.sin(yaw)))

    def set_speed_cap(self, cap):
        """网页模式给的限速;夹在模型训练下限和 --speed-cap 之间。返回实际用的值。"""
        self.rt.speed_cap = min(max(float(cap), self.rt.enc.v_lo), self.cap_max)
        top = PWM_FAST_MAX if self.rt.speed_cap > FAST_CAP else PWM_HARD_MAX
        if self.loop.cfg.pwm_max != top:
            self.loop.cfg = replace(self.loop.cfg, pwm_max=top)
        return self.rt.speed_cap

    def on_encoder(self, speed, now):
        self.rt.on_odometry(speed, now)

    def on_imu(self, yaw_rate, now):
        self.rt.on_imu(yaw_rate, now)

    def on_free(self, xy_body, stamp, now):
        if isinstance(self.rt, BevRuntime):
            self.rt.on_free(xy_body, stamp, now)
            if self.book is not None and self.rt.enabled:
                self.book.add_free(xy_body, self.rt.pose_at(stamp))

    def on_enable(self, enabled, now):
        """返回 True = 刚松手,要马上发一次中位(不靠看门狗补,2026-09-24 松手失控的教训)。"""
        was = self.rt.enabled
        release = self.rt.set_enabled(enabled, now)
        if enabled and not was:
            self.loop.reset()       # 重新使能不沿用上一次的油门状态
            self.block_t = None
            if self.book is not None and not self.book.active:
                self.book.resume()  # 上一段已收尾:开新段
        return release

    def book_tick(self, now, d):
        """记路径和决策;松手超过 SEG_GAP 收尾存盘。返回刚存好的段号,否则 None。"""
        if self.rt.enabled:
            self.release_t = None
            steer, target = d if isinstance(d, tuple) else (None, 0.0)
            self.book.add_pose(now, self.rt.odom.pose, steer, target, self.rt.speed)
        elif self.book.active and self.book.path:
            if self.release_t is None:
                self.release_t = now
            elif now - self.release_t > SEG_GAP:
                self.release_t = None
                self.book.pause()
                return self.book.seg
        return None

    def tick(self, now):
        cmd = self._tick(now)
        if self.book is not None and self.rt.enabled:
            d = self.d if isinstance(self.d, tuple) else (None, 0.0)
            self.book.add_frame(now, self.rt.odom.pose, d[0], d[1], self.rt.speed, self.event,
                                getattr(self.rt, 'last_img', None), self.cam_jpg)
        return cmd

    def _tick(self, now):
        d = self.d = self.rt.decide(now)
        self.seg_saved = self.book_tick(now, d) if self.book is not None else None
        if d is None:
            self.event = None
            return None
        if d is STOP:
            self.event, self.block_t = 'stale', None
            return self.neutral()
        model_steer, target = d
        steer = self.steer_cmd = self.avoid(model_steer)
        servo = steer_to_turn(steer, self.p)
        # 防撞按**当前要打的转角**算扫掠路径。截停时方向照发;车速高于 brake_release_speed 发刹车,低了回中位
        # (防倒车)。策略不出倒车,所以只报卡死、不自动倒车(雷达在车头,背后看不见)
        gap = self.guard.gap(steer)
        if gap is not None and gap < guard_stop(self.rt.speed, self.p):
            if self.block_t is None:
                self.block_t = now
            self.event = 'stuck' if now - self.block_t > STUCK_SECONDS else 'guard'
            self.gap = gap
            self.loop.reset()
            return (self.p.brake_pwm if self.rt.speed > self.p.brake_release_speed else self.p.pwm_mid), servo
        self.event = 'avoid' if steer != model_steer else None
        self.block_t = None
        if target < 0:      # BRAKE_CMD:车速够高才会到这里(低了 RoadRuntime 已改中位,防倒车),下一拍起步走阶梯
            self.loop.reset()
            return self.p.brake_pwm, servo
        return round(self.loop.update(target, self.rt.speed, CONTROL_PERIOD)), servo

    def avoid(self, steer):
        """转向避障(见 AVOID_CLEAR)。雷达没数据时原样返回(过期由运行时回中位)。"""
        g = self.guard.gap(steer)
        if g is None or g >= AVOID_CLEAR:
            self.avoid_side = 0.0
            return steer
        cands = np.linspace(-self.p.max_steer, self.p.max_steer, AVOID_STEERS)
        gaps = np.array([self.guard.gap(float(c)) or 0.0 for c in cands])
        ok = gaps >= AVOID_CLEAR
        if ok.any():
            cost = np.abs(cands - steer) - AVOID_KEEP * (np.sign(cands) == self.avoid_side)
            pick = float(cands[ok][np.argmin(cost[ok])])
        else:
            pick = float(cands[np.argmax(gaps)])
        self.avoid_side = float(np.sign(pick))
        return pick


def load_policy(path):
    """bev_v1_* / bev_v2_* / bev_v2.5_* 走 BevPolicy(BEV 输入),其余(road_v2 / road_v2.5 / spica_v3_*)走 RoadPolicy。"""
    with np.load(path) as z:
        schema = str(z['schema'])
    return BevPolicy(path) if schema.startswith('bev_') else RoadPolicy(path)


def parser():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('model', help='road_v2 / road_v2.5 / spica_v3_* / bev_v1_* / bev_v2_* / bev_v2.5_* .npz')
    p.add_argument('--speed-cap', type=float, required=True,
                   help=f'速度上限 m/s,要在模型自己的训练范围内(见 schema);网页模式限速不超过它;首次上车用 {V_LO}~0.5')
    return p


def create_node_class():
    """ROS 只在真正启动节点时 import;纯逻辑、测试和 --help 不需要 ROS。"""
    import rclpy
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Odometry
    from rclpy.node import Node
    from sensor_msgs.msg import CompressedImage, Imu, LaserScan, PointCloud2
    from std_msgs.msg import Bool, Float32

    class RoadCarNode(Node):
        def __init__(self, model, speed_cap, log_root=None):
            super().__init__('road_car_node')
            pol = load_policy(model)
            book = None
            if isinstance(pol, BevPolicy):
                from vehicle.bev_logbook import Logbook
                fov = None
                if pol.free:                                    # 摄像头视野里的全部地面格,让"没分成地面"的格减分
                    from common.ground_proj import GroundLut
                    lut = GroundLut(512 // 4, 512 // 4, CarParams(), 3.0)     # 和 ground_seg_node 默认输入一致
                    fov = lut.points(np.ones(lut.valid.shape, bool))
                book = Logbook(log_root or os.path.expanduser(time.strftime('~/drives/logbook_%m%d_%H%M%S')),
                               fov_body=fov)
            self.ctl = RoadCarController(pol, speed_cap, book=book)
            self.pub = self.create_publisher(Twist, CMD_TOPIC, 5)
            self.create_subscription(LaserScan, '/scan', self.on_scan, 1)
            self.create_subscription(Odometry, ENC_TOPIC, self.on_enc, 10)
            self.create_subscription(Imu, '/IMU_data', self.on_imu, 10)
            self.create_subscription(Bool, '/policy_enable', self.on_enable, 1)
            self.create_subscription(Float32, CAP_TOPIC, self.on_cap, 1)
            if getattr(self.ctl.rt.policy, 'free', False):            # bev_v2:订阅地面分割
                self.create_subscription(PointCloud2, FREE_TOPIC, self.on_free, 2)
                self.create_subscription(CompressedImage, '/ground_debug', self.on_cam, 1)   # 逐帧回看配图
            self.last_event = None
            self.last_floor = self.ctl.loop.floor
            self.create_timer(CONTROL_PERIOD, self.tick)
            self.get_logger().info(
                f'已载入 {model},速度上限 {self.ctl.rt.speed_cap:.2f} m/s(网页模式可调到 {speed_cap:.2f}),'
                f'编码器闭环 -> {CMD_TOPIC},等 /policy_enable')

        def now(self):
            return self.get_clock().now().nanoseconds * 1e-9

        def send(self, cmd):
            tw = Twist()
            tw.linear.x, tw.angular.z = float(cmd[0]), float(cmd[1])
            self.pub.publish(tw)

        def neutral(self):
            self.send(self.ctl.neutral())

        def save_map(self):
            """当前段全局图存成 map.png(段收尾、进程退出时;seg.npz 由 Logbook 自己存)。"""
            book = self.ctl.book
            if book is None or not book.path:
                return
            import cv2
            os.makedirs(book.dir(), exist_ok=True)
            cv2.imwrite(os.path.join(book.dir(), 'map.png'), book.render(cv2))

        def on_scan(self, m):
            self.ctl.on_scan(m, self.now())

        def on_enc(self, m):
            self.ctl.on_encoder(m.twist.twist.linear.x, self.now())

        def on_imu(self, m):
            self.ctl.on_imu(m.angular_velocity.z, self.now())

        def on_free(self, m):
            xy = np.frombuffer(bytes(m.data), np.float32).reshape(-1, 2).astype(np.float64)
            self.ctl.on_free(xy, m.header.stamp.sec + m.header.stamp.nanosec * 1e-9, self.now())

        def on_cam(self, m):
            self.ctl.cam_jpg = bytes(m.data)

        def on_cap(self, m):
            old = self.ctl.rt.speed_cap
            if self.ctl.set_speed_cap(m.data) != old:
                self.get_logger().info(f'限速 {old:.2f} -> {self.ctl.rt.speed_cap:.2f} m/s')

        def on_enable(self, m):
            was = self.ctl.rt.enabled
            if self.ctl.on_enable(m.data, self.now()):
                self.neutral()
                self.get_logger().info('松手,已发中位')
            elif m.data and not was:
                self.get_logger().info('使能')

        def tick(self):
            now = self.now()
            cmd = self.ctl.tick(now)
            if self.ctl.seg_saved is not None:
                self.save_map()
                self.get_logger().info(f'松手 {SEG_GAP:.0f}s,全局地图第 {self.ctl.seg_saved} 段已存到 {self.ctl.book.dir()}')
            if cmd is None:
                self.last_event = None
                return
            self.send(cmd)
            ev, log = self.ctl.event, self.get_logger()
            if abs(self.ctl.loop.floor - self.last_floor) >= 1.0:
                # floor 自标定只该在电量变化时慢慢动;频繁变说明被弯道减速之类骗了(2026-09-29)
                log.info(f'速度环 floor {self.last_floor:.0f} -> {self.ctl.loop.floor:.0f},'
                         f'车速 {self.ctl.rt.speed:.2f} m/s')
                self.last_floor = self.ctl.loop.floor
            if ev == 'stale':
                log.warn(f'{self.ctl.rt.stale_sensors(now) or ["心跳"]} 过期,回中位',
                         throttle_duration_sec=1.0)
            elif ev == 'guard':
                log.warn(f'前方 {self.ctl.gap:.2f}m,截停', throttle_duration_sec=1.0)
            elif ev == 'stuck' and self.last_event != 'stuck':
                log.error(f'已连续截停 {STUCK_SECONDS:.0f}s 以上,车卡住了 —— 前方 {self.ctl.gap:.2f}m,'
                          f'策略不会倒车,需要人工干预')
            elif ev == 'avoid' and self.last_event != 'avoid':
                d = self.ctl.d
                log.info(f'避障:模型 {math.degrees(d[0]):+.0f}° -> {math.degrees(self.ctl.steer_cmd):+.0f}°')
            elif ev in (None, 'avoid') and self.last_event in ('guard', 'stuck'):
                log.info('脱困,恢复行驶')
            self.last_event = ev

    return RoadCarNode


def main(argv=None):
    args = parser().parse_args(argv)
    if not Path(args.model).is_file():
        raise SystemExit(f'模型文件不存在: {args.model}')
    enc = load_policy(args.model).enc          # 速度范围跟模型走(#19)
    if not enc.v_lo <= args.speed_cap <= enc.v_hi:
        raise SystemExit(f'--speed-cap 要在训练范围 {enc.v_lo}~{enc.v_hi} m/s 内,给的是 {args.speed_cap}')

    import rclpy
    from rclpy.signals import SignalHandlerOptions

    # 自己接信号:rclpy 默认处理器会先关掉 context,finally 里再发中位时 publisher 已失效,
    # systemctl stop 时车收不到中位(steer_policy_node 2026-09-21 实测)。
    rclpy.init(args=[], signal_handler_options=SignalHandlerOptions.NO)
    node = create_node_class()(args.model, args.speed_cap)

    def stop(*_):
        for _ in range(5):
            node.neutral()
            time.sleep(0.02)
        node.get_logger().info('已回中位')
        if node.ctl.book is not None and node.ctl.book.active:
            node.ctl.book.save()            # 没收尾的那段也存下
            node.save_map()
        os._exit(0)             # sys.exit 会和 spin 抢,C++ 层直接 abort
    for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP):
        signal.signal(sig, stop)
    rclpy.spin(node)


if __name__ == '__main__':
    main()
