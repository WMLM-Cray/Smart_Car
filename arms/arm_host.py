#!/usr/bin/env python3
"""实验臂（arms/corridor_arm.py）专用的**仿真**宿主节点。

为什么不直接用仓内的两个节点
----------------------------
* `vehicle/road_policy_node.py`（仓内仿真节点）只有 `runtime.step()` → publish，
  **没有任何闸门**：不查 `ScanGuard`、不记 `block_t`、不调 `neutral()`、不倒车。
  但臂的四层（投影 `_use_project`、换弧 `_use_preempt/_detour`、预瞄 `_use_prearm`、
  七弧预扫）恰好全部挂在"被 guard 拦停"和"neutral 之前"这两个钩子上
  ⇒ 直接拿它当宿主，这四层会**静默失效**，跑出来的不是臂。
* `vehicle/road_car_node.py`（实车节点，main @ad25f56）有闸门
  （`GUARD_STOP = 0.25`、`STUCK_SECONDS = 3.0`）但**不倒车**：
  `:102-103`「截停时方向照发、油门回中位；策略不出倒车…（雷达在车头，背后看不见）」。

本文件 = 上面两者的交集，再补上 2026-10-05 认证 W 时那个仓外
`teleop/road_policy_node.py` 的时序，好让 W 的判决数字可以与整改后的版本对照：

    雷达/心跳闸门 → `guard.gap(steer) < guard_stop` 时**先 neutral**
    （arms/corridor_arm.py 的 neutral 补丁在这里接管）→ 连续截停满 3.0 s 后
    交给 `escape_reverse()`（臂的投影层挂在那里）。

⚠️ **倒车默认关**：`escape_reverse()` 默认只停不倒，与实车 `vehicle/road_car_node.py:102-103`
   一致（雷达在车头、背后看不见）：仿真默认必须和实车同行为，否则读数高估实车能力。
   旧的盲倒 1.5 s
   （`REVERSE_FRACTION = 0.5`）留作显式实验开关 `ARM_HOST_REVERSE=1`，用来复现旧 W/Y 面板。

CLI 与旧节点一致（`evaluation/gazebo_supervised_runner.py:400-401` 传的就是这套）：

    python3 arms/arm_host.py <model.npz> --sim --speed 2.2 --guard 0.25

顺带修掉的两条整改项：宿主不再吃仓外路径（旧 `TEAM_REPO=/mnt/.../Auto_Car-main/teleop/`），
几何与舵机曲线一律走 `common/params.CarParams` 与 `common/pwm`。

关于 import：臂在 **import 期**就要给 `RoadPolicyNode` 打补丁（`RPN.RoadPolicyNode.tick = …`），
所以这个类必须定义在模块级。没有 ROS 时本模块**仍可 import**（`Node` 退化成 `object`），
只是 `main()` 会拒绝启动 —— 这样 `pytest` 才能在不装 ROS 的机器上测这个宿主的时序。
"""
from __future__ import annotations

import argparse
import os
import sys
import time
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parent.parent
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from common.control import to_cmd_vel                                  # noqa: E402
from common.params import CarParams                                     # noqa: E402
from common.pwm import action_to_steer, steer_to_action                 # noqa: E402
from common.road_policy import RoadPolicy, spec_of                      # noqa: E402
from common.road_runtime import STOP, RoadRuntime                       # noqa: E402
from vehicle.scan_guard import ScanGuard                                # noqa: E402

try:                                # ROS 可选：见模块文档字符串最后一段
    import rclpy
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Odometry
    from rclpy.node import Node
    from sensor_msgs.msg import Imu, LaserScan
    from std_msgs.msg import Bool
    HAS_ROS = True
except ImportError:                 # pragma: no cover - 只在没有 ROS 的机器上走到
    rclpy = Twist = Odometry = Imu = LaserScan = Bool = None
    Node = object
    HAS_ROS = False

CMD_TOPIC = '/cmd_vel'
SCAN_TOPIC = '/scan'
ODOM_TOPIC = '/ackermann_steering_controller/odometry'
IMU_TOPIC = '/IMU_data'
ENABLE_TOPIC = '/policy_enable'
CONTROL_PERIOD = 0.05
CAR_DOMAIN = '42'               # vehicle/ros_env.sh：实车域，仿真禁止用

# ---- 闸门与恢复时序（与认证 W 时的 teleop 生产节点同口径）---------------------
STUCK_SECONDS = 3.0             # 连续截停多久算"卡住"
REVERSE_SECONDS = 1.5           # 每次盲倒持续多久
REVERSE_FRACTION = 0.5          # 盲倒速度 = cap * 该系数
ESCAPE_STEER = 0.35             # 盲倒方向（每次触发翻符号）
GUARD_STOP_DEFAULT = 0.25       # 与 vehicle/road_car_node.py:44 同值
SCAN_TIMEOUT_DEFAULT = 0.3
ENABLE_TIMEOUT_DEFAULT = 0.3


def _reverse_enabled():
    """盲倒脱困开关。默认**关**（= 只停不倒，与实车 `vehicle/road_car_node.py:102-103` 一致）。

    实车雷达只看前方、
    车尾还伸出 0.17 m 里程计，倒车撞了也不知道 ⇒ 仿真默认必须和实车同行为，否则读数
    高估实车能力。旧 W/Y 面板是在盲倒打开时跑的，要复现那批读数置 `ARM_HOST_REVERSE=1`。
    """
    return (os.environ.get('ARM_HOST_REVERSE') or '').strip().lower() in ('1', 'true', 'yes', 'on')


REVERSE_ENABLED = _reverse_enabled()


class _NullLogger:
    """无 ROS 时的 `get_logger()` 替身：臂会打 info/warn/error（可能带 kwargs）。"""

    def info(self, *args, **kwargs):
        pass

    warn = info
    error = info
    debug = info


_NULL_LOG = _NullLogger()


def _schema(path):
    with np.load(path) as z:
        return str(z['schema'])


def parser():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('model', help='policy .npz checkpoint')
    p.add_argument('--sim', action='store_true', help='（必须）仿真：输出 /cmd_vel Twist')
    p.add_argument('--speed', type=float, required=True, help='速度上限 m/s')
    p.add_argument('--guard', type=float, default=GUARD_STOP_DEFAULT,
                   help='截停净空阈值 m（默认 0.25，同实车节点）')
    p.add_argument('--scan-timeout', type=float, default=SCAN_TIMEOUT_DEFAULT,
                   help='雷达/里程计/IMU 样本的最大年龄（仿真秒）')
    p.add_argument('--enable-timeout', type=float, default=ENABLE_TIMEOUT_DEFAULT,
                   help='/policy_enable 心跳的最大年龄（仿真秒）')
    return p


class RoadPolicyNode(Node):
    """仿真宿主：策略 + 闸门 + 恢复时序。ROS 只出现在 `_wire_ros()` 与 `_cmd()` 里。"""

    def __init__(self, model, speed_cap, sensor_timeout, enable_timeout,
                 guard_stop=GUARD_STOP_DEFAULT):
        if HAS_ROS:
            Node.__init__(self, 'road_policy_node', parameter_overrides=[
                rclpy.parameter.Parameter('use_sim_time', value=True)])
        self.p = CarParams()
        self.params = self.p
        self.policy = RoadPolicy(model)
        self.runtime = RoadRuntime(self.policy, speed_cap, sensor_timeout,
                                   enable_timeout, params=self.p)
        self.sim = True
        self.cap = self.runtime.speed_cap
        self.guard_stop = float(guard_stop)
        self.guard = ScanGuard()
        # 闸门状态（臂会直接读写这几个）
        self.block_t = None
        self.stuck = False          # bool，与生产节点同（不是"卡了多久"）
        self.escape_until = None
        self.escape_steer = ESCAPE_STEER
        self._in_tick = False
        self.t0 = None              # 只有首次写诊断 CSV 时才有值（同生产节点）
        # 诊断 CSV：ROAD_DEBUG=<路径> 时每 tick 一行，字段与认证 W 时的生产节点
        # 逐字相同 —— runs/ 下的 pin_report.py / reverse_budget.py / w_arcs_report.py
        # 都读这个文件，runner 也总是设 ROAD_DEBUG，所以它不是可选装饰。
        self.dbg = None
        path = os.environ.get('ROAD_DEBUG')
        if path:
            self.dbg = open(path, 'w', buffering=1)
            self.dbg.write('t,speed,yaw_rate,a0,a1,v_cmd,steer_rad,gap,blocked\n')
        if HAS_ROS:
            self._wire_ros()
            self.get_logger().info(
                f'{Path(model).name} loaded; sim clock required; {CMD_TOPIC} output; '
                f'cap {self.cap:.2f} m/s; guard_stop {self.guard_stop:.2f} m; '
                f'stuck {STUCK_SECONDS:.1f}s -> '
                + (f'reverse {REVERSE_SECONDS:.1f}s x {REVERSE_FRACTION:.2f} '
                   f'(ARM_HOST_REVERSE=1, 实验宿主行为, 实车不倒车)'
                   if REVERSE_ENABLED else 'stop only (与实车同行为)')
                + '; waiting for /policy_enable heartbeat')

    # ---- ROS 接线 ---------------------------------------------------------------
    def _wire_ros(self):
        self.pub = self.create_publisher(Twist, CMD_TOPIC, 5)
        self.create_subscription(LaserScan, SCAN_TOPIC, self.on_scan, 1)
        self.create_subscription(Odometry, ODOM_TOPIC, self.on_odom, 10)
        self.create_subscription(Imu, IMU_TOPIC, self.on_imu, 10)
        self.create_subscription(Bool, ENABLE_TOPIC, self.on_enable, 1)
        self.create_timer(CONTROL_PERIOD, self.tick)

    def get_logger(self):
        """有 ROS 时用 rclpy 的 logger；没有时退化成空实现（臂会调它）。"""
        return Node.get_logger(self) if HAS_ROS else _NULL_LOG

    def now(self):
        """仿真钟：仿真里必须用 `/clock`，墙钟会让所有 timeout 失效。"""
        return self.get_clock().now().nanoseconds * 1e-9

    # ---- 只读转发 runtime（臂读 scan/scan_t/speed/last_action 等）--------------
    @property
    def scan(self):
        return self.runtime.scan

    @property
    def angles(self):
        return self.runtime.angles

    @property
    def scan_t(self):
        return self.runtime.scan_t

    @property
    def speed(self):
        return self.runtime.speed

    @property
    def yaw_rate(self):
        return self.runtime.yaw_rate

    @property
    def last_action(self):
        return self.runtime.last_action

    @property
    def enabled(self):
        return self.runtime.enabled

    @property
    def enable_t(self):
        return self.runtime.enable_t

    @property
    def scan_timeout(self):     # 旧宿主叫 scan_timeout，仓内叫 sensor_timeout
        return self.runtime.sensor_timeout

    # ---- 原始（未打补丁）的收发与输出 ------------------------------------------
    def on_scan(self, msg):
        now = self.now()
        self.runtime.on_scan(msg.ranges, msg.angle_min, msg.angle_increment, now)
        self.guard.update(msg)      # ← 臂的 gap() 全靠这行

    def on_odom(self, msg):
        self.runtime.on_odometry(msg.twist.twist.linear.x, self.now())

    def on_imu(self, msg):
        self.runtime.on_imu(msg.angular_velocity.z, self.now())

    def on_enable(self, msg):
        was_enabled = self.runtime.enabled
        if self.runtime.set_enabled(msg.data, self.now()):
            self.neutral()          # 松手就自己发中位，不靠看门狗补
            self.get_logger().info('停车')
        elif msg.data and not was_enabled:
            self.block_t = None
            self.stuck = False
            self.escape_until = None
            self.get_logger().info('使能')

    def _cmd(self, linear, angular):
        """唯一的输出出口。无 ROS 的测试可以换掉它来记录指令。"""
        msg = Twist()
        msg.linear.x, msg.angular.z = float(linear), float(angular)
        self.pub.publish(msg)

    def publish(self, v, steer_a):
        """臂的 publish 补丁末尾会调它：v 单位 m/s，steer_a 是**动作**（±1）。"""
        self._cmd(*to_cmd_vel(float(v), action_to_steer(float(steer_a)), self.p))

    def neutral(self):
        self._cmd(0.0, 0.0)

    def escape_reverse(self):
        """卡住满 3 s 后的恢复出口。

        默认**只停不倒**（`publish(0, 0)`）：与实车 `vehicle/road_car_node.py:102-103`
        同行为 —— 雷达在车头、背后看不见，实车永不倒车。
        置 `ARM_HOST_REVERSE=1` 才恢复旧的实验宿主盲倒
        （`-cap*REVERSE_FRACTION`、方向取 `escape_steer`），用于复现旧 W/Y 面板读数。
        """
        if not REVERSE_ENABLED:
            self.neutral()
            return
        self.publish(-self.cap * REVERSE_FRACTION, -self.escape_steer)

    def _csv_row(self, now, steer, target, gap):
        """诊断行：字段与认证 W 时的生产节点逐字相同（`t0` 在第一次写时定）。"""
        if self.dbg is None:
            return
        if self.t0 is None:
            self.t0 = now
        action = self.runtime.last_action
        self.dbg.write('%.3f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%s,%d\n' % (
            now - self.t0, float(self.runtime.speed), float(self.runtime.yaw_rate),
            float(action[0]), float(action[1]), float(target), float(steer),
            '' if gap is None else '%.4f' % gap,
            1 if (gap is not None and gap < self.guard_stop) else 0))

    def tick(self):
        now = self.now()
        if now <= 0.0:          # 仿真钟还没起来
            if self.runtime.enabled:
                self.neutral()
            return
        self._in_tick = True
        try:
            self._step(now)
        finally:
            self._in_tick = False

    def _step(self, now):
        d = self.runtime.decide(now)
        if d is None:           # 未使能：静默（旧节点同）
            return
        if d is STOP:           # 心跳或传感器过期：中位 + 告警（串照抄生产节点）
            if not self.runtime.enabled:
                self.get_logger().info('心跳断开,停车')
            else:
                self.get_logger().warn('雷达无数据,回中位',
                                       throttle_duration_sec=1.0)
            self.neutral()
            return
        steer, target = d
        steer_a = steer_to_action(steer, self.p)
        gap = self.guard.gap(steer)
        self._csv_row(now, steer, target, gap)
        if gap is not None and gap < self.guard_stop:
            if self.block_t is None:
                self.block_t = now
            held = now - self.block_t
            # 截停期间必须回中位：油门归零但转向照发时 guard 仍按满舵算扫掠弧，
            # 报的 gap 会比真实净空小 0.2m 左右，等于自己把阈值压低。
            self.neutral()      # ← 臂的 neutral 补丁在这里接管（投影/换弧）
            if held < STUCK_SECONDS:
                self.get_logger().warn(f'前方 {gap:.2f}m,截停',
                                       throttle_duration_sec=1.0)
                return
            # 连续截停超过 STUCK_SECONDS：车不动 = 观测不变 = 策略输出不变，
            # 再等下去场景永远不会变，这一局就废了。恢复出口见下。
            if not self.stuck:
                self.stuck = True
                self.get_logger().error(
                    f'已连续截停 {held:.1f}s 车卡住 —— 前方 {gap:.2f}m，'
                    + ('倒车脱困（实验宿主行为，实车不倒车）' if REVERSE_ENABLED
                       else '只停不倒（与实车 road_car_node.py:102-103 同行为）'))
            if REVERSE_ENABLED:
                if self.escape_until is None or now >= self.escape_until:
                    if self.escape_until is not None:
                        # 上一轮倒车结束还没拉开距离 -> 换一个方向再倒
                        self.escape_steer = -self.escape_steer
                    self.escape_until = now + REVERSE_SECONDS
                self.get_logger().warn(
                    f'前方 {gap:.2f}m,倒车脱困({REVERSE_SECONDS:.1f}s)',
                    throttle_duration_sec=1.0)
            # 恢复出口统一走 escape_reverse()：臂的投影层挂在这里（投影成功 = 继续
            # 往前走，不是倒车）；投影无解才落到宿主的盲倒 —— 盲倒关了就只停。
            self.escape_reverse()
            return
        if self.stuck:
            self.get_logger().info('脱困,恢复行驶')
        self.block_t, self.stuck, self.escape_until = None, False, None
        self.publish(target, steer_a)


def main(argv=None):
    args = parser().parse_args(argv)
    if not HAS_ROS:
        raise SystemExit('本模块是 ROS 节点入口：当前解释器里没有 rclpy，'
                         '请先 source 仿真环境（见 runs/corridor_arm_run.sh）')
    if not args.sim:
        raise SystemExit('这是实验宿主的仿真路径：必须显式加 --sim（实车指令由 '
                         'vehicle/road_car_node.py 发，不经本文件）')
    domain = os.environ.get('ROS_DOMAIN_ID')
    if domain is None or domain == CAR_DOMAIN:
        raise SystemExit(f'simulation only: set ROS_DOMAIN_ID to the sim domain '
                         f'(run_sim.sh default 60), not unset and not the car domain {CAR_DOMAIN}')
    if args.scan_timeout <= 0.0 or args.enable_timeout <= 0.0:
        raise SystemExit('timeouts must be positive')
    if not Path(args.model).is_file():
        raise SystemExit(f'model file not found: {args.model}')

    enc = spec_of(_schema(args.model))
    if not enc.v_lo <= args.speed <= enc.v_hi:
        raise SystemExit(f'--speed {args.speed} must be within this model\'s training range '
                         f'{enc.v_lo}..{enc.v_hi} m/s (from its schema)')

    from rclpy.signals import SignalHandlerOptions

    rclpy.init(args=[], signal_handler_options=SignalHandlerOptions.NO)
    node = None
    try:
        node = RoadPolicyNode(args.model, args.speed,
                              args.scan_timeout, args.enable_timeout, args.guard)
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            for _ in range(5):
                node.neutral()
                time.sleep(0.02)
            node.get_logger().info('shutdown: zero commands sent')
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
