#!/usr/bin/env python3
"""开机/测试前的一键就绪检查:清残留 -> 等话题图 -> 逐项体检 -> 说能不能跑。

用法(车上,或者 ssh 一行):
    python3 ~/autocar/carready.py

## 为什么要这个

2026-09-23 查下来,"开机不能马上跑、要清一堆东西"是三件事叠在一起:

1. 开机头 1~2 分钟负载 11~16(start-davinci 昇腾初始化 24s + docker),这段时间
   DDS 发现不全:开机 1 分钟时 /teleop_cmd_vel 上只看得到 policy,底盘驱动、
   watchdog、web 都还没被发现,订阅者 0。两分钟后同样的查询 0.2s 就全了。
   所以这里**等**,不是报错。
2. 上一轮测试脚本没退干净,还挂在 /teleop_cmd_vel 上。这里先发 SIGINT(那些脚本
   都接管了 SIGINT,会先补中位再退),2s 还不走再 SIGKILL。
3. ros2 CLI daemon 开机后状态坏掉,`ros2 topic` 报 `!rclpy.ok()`。这里顺手重置。
4. 底盘 launch 里的雷达/编码器节点开机时串口没就绪就直接死了,launch 不重启它们,
   racecar-chassis 还显示 active。根治在 wait_serial.sh;这里兜底:等了
   HEAL_AFTER 秒传感器还是没数据,就重启一次 racecar-chassis(只一次)。
5. GNOME 桌面空闲 20 分钟会让整机挂起,唤醒后电调不响应、其余检查全过。这里确认
   四个 sleep 相关的 target 都是 masked。

除了第 4 条,不起停任何 systemd 服务 —— 状态报给你,要不要动你决定。
"""
from __future__ import annotations

import os
import signal
import subprocess
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from common.params import CarParams

PWM_MID = CarParams().pwm_mid
TURN_MID = CarParams().turn_mid

# 会往 /teleop_cmd_vel 发油门的测试脚本。只认这些名字,别的 python 进程一概不碰。
STRAY_SCRIPTS = (
    'coast_test.py', 'esc_rearm.py', 'floor_probe.py', 'pwm_sweep.py',
    'loop_test.py', 'stair_scan.py', 'drain.py',
    'ramp_curve.py', 'straight_trim.py', 'turn_center.py', 'encoder_check.py',
    'hold_enable.py', 'policy_node.py', 'steer_policy_node.py', 'road_car_node.py',
)
REQUIRED_UNITS = ('racecar-chassis', 'racecar-watchdog', 'racecar-web')
# 2026-09-23:GNOME 桌面没人碰 20 分钟就挂起(不管 SSH/脚本),唤醒后话题、驱动 fd 全正常,
# 电调却不响应 —— 这里其余检查全过,查不出来。用户已经 mask 掉,这里只盯着它别被改回去。
SLEEP_UNITS = ('sleep.target', 'suspend.target', 'hibernate.target', 'hybrid-sleep.target')
SENSORS = {'/encoder': 20, '/scan': 8, '/IMU_data': 30}   # 最低 Hz,实测 34/12/55
KNOWN_CMD_PUBS = {'teleop_watchdog', 'web_teleop', 'steer_policy_node', 'road_car_node', 'carready'}
WINDOW = 2.0
WAIT = 90.0
HEAL_AFTER = 20.0       # 重启后实测 4s 就齐,20s 还没有就不是"发现慢"了


def find_strays(ps_lines, self_pid: int, keep_pids=()) -> list[tuple[int, str]]:
    """从 `ps -eo pid=,args=` 的输出里挑出残留测试脚本。

    按 argv 里的脚本文件名精确匹配,不用 `pkill -f`:那个会匹配到发起它的
    ssh 命令行本身,之前把自己的会话杀掉过好几次。
    """
    out = []
    for line in ps_lines:
        parts = line.split(None, 1)
        if len(parts) != 2 or not parts[0].isdigit():
            continue
        pid, args = int(parts[0]), parts[1]
        if pid == self_pid or pid in keep_pids:
            continue
        argv = args.split()
        if not argv or 'python' not in os.path.basename(argv[0]):
            continue
        name = os.path.basename(argv[1]) if len(argv) > 1 else ''
        if len(argv) > 2 and argv[1] == '-m' and argv[2].startswith(('vehicle.', 'calibration.')):
            name = argv[2].rsplit('.', 1)[-1] + '.py'
        if name in STRAY_SCRIPTS:
            out.append((pid, args))
    return out


def esc_link_ok(fd_links, car_target: str) -> bool:
    """厂家驱动手里有没有一个指向**当前** /dev/car 设备的 fd。

    2026-09-23:开机 USB hub 复位后,驱动还攥着第一次枚举的 ttyUSB2(readlink 显示
    `/dev/ttyUSB2 (deleted)`),或者干脆没打开。两种情况它都照样订阅、照样打日志,
    指令全写空 —— 只看话题图和传感器是查不出来的。
    """
    return car_target in fd_links


def driver_fd_links() -> list[str] | None:
    """racecar_driver_node 进程所有 fd 的 readlink;找不到进程返回 None。"""
    for pid in filter(str.isdigit, os.listdir('/proc')):
        try:
            with open(f'/proc/{pid}/cmdline', 'rb') as f:
                if b'lib/racecar_driver/racecar_driver_node' not in f.read():
                    continue
            fd_dir = f'/proc/{pid}/fd'
            return [os.readlink(os.path.join(fd_dir, fd)) for fd in os.listdir(fd_dir)]
        except OSError:
            continue
    return None


def problems(units: dict, rates: dict, cmd_pubs: list, cmd_subs: list,
             scan_pubs: int, idle_cmds: int, esc_ok: bool,
             sleep_states: dict) -> list[str]:
    """体检结果 -> 问题清单。空清单 = 可以跑。"""
    out = []
    awake = sorted(u for u in SLEEP_UNITS if sleep_states.get(u) != 'masked')
    if awake:
        out.append(f'系统自动睡眠没关({", ".join(awake)} 不是 masked):桌面空闲 20 分钟会挂起,'
                   f'唤醒后电调不响应。sudo systemctl mask {" ".join(awake)}')
    if not esc_ok:
        out.append('电调串口没通:底盘驱动没拿着当前的 /dev/car —— 指令发了也到不了电调')
    for u in REQUIRED_UNITS:
        if units.get(u) != 'active':
            out.append(f'{u} 是 {units.get(u)},要 active')
    for topic, lo in SENSORS.items():
        hz = rates.get(topic, 0.0)
        if hz < lo:
            out.append(f'{topic} {hz:.1f} Hz,低于 {lo}')
    if 'racecar_driver' not in cmd_subs:
        out.append('底盘驱动 racecar_driver 没订阅 /teleop_cmd_vel —— 发了指令也到不了电调')
    strangers = sorted(set(cmd_pubs) - KNOWN_CMD_PUBS)
    if strangers:
        out.append(f'/teleop_cmd_vel 上有不认识的发布者 {strangers}')
    if scan_pubs > 1:
        # 2026-09-20:同网段同域号的 Gazebo 混进来一个 2000 点的假 /scan,不报错
        out.append(f'/scan 有 {scan_pubs} 个发布者 —— 同网段有仿真用了同一个 ROS_DOMAIN_ID')
    if idle_cmds:
        out.append(f'空闲时 /teleop_cmd_vel 上 {WINDOW:.0f}s 收到 {idle_cmds} 条指令,有人在发')
    return out


def unit_state(name: str) -> str:
    r = subprocess.run(['systemctl', 'is-active', name], capture_output=True, text=True)
    return r.stdout.strip() or 'unknown'


def unit_enabled(name: str) -> str:
    r = subprocess.run(['systemctl', 'is-enabled', name], capture_output=True, text=True)
    return r.stdout.strip() or 'unknown'


def kill_strays(keep_pids):
    ps = subprocess.run(['ps', '-eo', 'pid=,args='], capture_output=True, text=True)
    strays = find_strays(ps.stdout.splitlines(), os.getpid(), keep_pids)
    for pid, args in strays:
        print(f'  清残留 {pid}: {args}', flush=True)
        try:
            os.kill(pid, signal.SIGINT)       # 它们会先补中位再退
        except ProcessLookupError:
            pass
    if strays:
        time.sleep(2.0)
        for pid, _ in strays:
            try:
                os.kill(pid, signal.SIGKILL)
                print(f'  {pid} 不响应 SIGINT,已 SIGKILL', flush=True)
            except ProcessLookupError:
                pass
    return len(strays)


def main():
    if 'ROS_DOMAIN_ID' not in os.environ:
        raise SystemExit('ROS_DOMAIN_ID 没设 —— 先 source ~/autocar/ros_env.sh')

    policy = unit_state('racecar-policy')
    policy_pid = subprocess.run(['systemctl', 'show', '-p', 'MainPID', '--value',
                                 'racecar-policy'], capture_output=True, text=True).stdout
    keep = {int(policy_pid)} if policy_pid.strip().isdigit() else set()

    print('1) 清残留测试脚本', flush=True)
    if not kill_strays(keep):
        print('  没有', flush=True)
    print('2) 重置 ros2 CLI daemon', flush=True)
    subprocess.run(['ros2', 'daemon', 'stop'], capture_output=True)

    import rclpy
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import Imu, LaserScan

    rclpy.init()
    node = rclpy.create_node('carready')
    stamps = {t: [] for t in (*SENSORS, '/teleop_cmd_vel')}

    def rec(topic):
        return lambda _m: stamps[topic].append(time.time())
    node.create_subscription(Odometry, '/encoder', rec('/encoder'), 50)
    node.create_subscription(LaserScan, '/scan', rec('/scan'), 10)
    node.create_subscription(Imu, '/IMU_data', rec('/IMU_data'), 50)
    node.create_subscription(Twist, '/teleop_cmd_vel', rec('/teleop_cmd_vel'), 50)

    print(f'3) 等话题图就绪(最多 {WAIT:.0f}s,开机头两分钟 DDS 发现慢是正常的)', flush=True)
    t0 = time.time()
    healed = False
    cmd_pub = node.create_publisher(Twist, '/teleop_cmd_vel', 5)
    neutral = Twist()
    neutral.linear.x, neutral.angular.z = float(PWM_MID), float(TURN_MID)
    while True:
        # 厂家库第一次发指令时才开串口,所以先喂几条真中位再查 fd。真中位不会让车动,
        # 也不会武装看门狗(它只认"会动"的指令)。发完再清计数,不算进"空闲时有人在发"。
        for _ in range(5):
            cmd_pub.publish(neutral)
            rclpy.spin_once(node, timeout_sec=0.05)
        drain = time.time() + 0.5           # 自己发的那几条收完再开始计数
        while time.time() < drain:
            rclpy.spin_once(node, timeout_sec=0.05)
        end = time.time() + WINDOW
        for s in stamps.values():
            s.clear()
        while time.time() < end:
            rclpy.spin_once(node, timeout_sec=0.05)
        found = problems(
            units={u: unit_state(u) for u in REQUIRED_UNITS},
            rates={t: len(stamps[t]) / WINDOW for t in SENSORS},
            cmd_pubs=[i.node_name for i in node.get_publishers_info_by_topic('/teleop_cmd_vel')],
            cmd_subs=[i.node_name for i in node.get_subscriptions_info_by_topic('/teleop_cmd_vel')],
            scan_pubs=node.count_publishers('/scan'),
            idle_cmds=len(stamps['/teleop_cmd_vel']),
            esc_ok=esc_link_ok(driver_fd_links() or [], os.path.realpath('/dev/car')),
            sleep_states={u: unit_enabled(u) for u in SLEEP_UNITS})
        waited = time.time() - t0
        if not found or waited > WAIT:
            break
        print(f'  {waited:4.0f}s 还差 {len(found)} 项:{found[0]}', flush=True)
        dead = [t for t in SENSORS if not stamps[t]]
        esc_dead = any(p.startswith('电调串口') for p in found)
        if (dead or esc_dead) and not healed and waited > HEAL_AFTER:
            healed = True
            what = dead + (['电调串口'] if esc_dead else [])
            print(f'  {what} 一直不通,多半是开机时串口没就绪/USB 复位过 —— '
                  f'重启一次 racecar-chassis', flush=True)
            subprocess.run(['sudo', '-n', 'systemctl', 'restart', 'racecar-chassis'])

    rates = '  '.join(f'{t} {len(stamps[t]) / WINDOW:.0f}Hz' for t in SENSORS)
    print(f'\n传感器:{rates}', flush=True)
    if policy == 'active':
        print('提醒:racecar-policy 在跑。它空闲不发指令,但网页上一点 enable 就会接管油门;'
              '做油门测试前 sudo systemctl stop racecar-policy', flush=True)
    if found:
        print(f'\n没就绪({waited:.0f}s):', flush=True)
        for p in found:
            print(f'  - {p}', flush=True)
        os._exit(1)
    print(f'\n就绪,可以跑({waited:.0f}s)', flush=True)
    os._exit(0)


if __name__ == '__main__':
    main()
