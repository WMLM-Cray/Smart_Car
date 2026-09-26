"""限时使能已启动的仿真策略，记录移动与松手停车；ROS 域须与 run_sim.sh 相同(默认 60),不能是实车的 42。"""
import argparse
import json
import math
import os
import time
from pathlib import Path


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--seconds', type=float, default=10)
    ap.add_argument('--out', default='runs/gazebo_probe.json')
    args = ap.parse_args()
    if os.environ.get('ROS_DOMAIN_ID') in (None, '42'):
        ap.error('ROS_DOMAIN_ID 要设成仿真域(run_sim.sh 默认 60),不能不设或用实车的 42')
    if not 0 < args.seconds <= 60:
        ap.error('--seconds 要在 (0, 60] 内')
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from nav_msgs.msg import Odometry
    from geometry_msgs.msg import Twist
    from std_msgs.msg import Bool
    rclpy.init()
    node = Node('model_chain_probe')
    pub = node.create_publisher(Bool, '/policy_enable', 1)
    odoms, commands = [], []
    def odom(m):
        odoms.append((time.monotonic(), m.pose.pose.position.x, m.pose.pose.position.y, m.twist.twist.linear.x))
    def command(m):
        commands.append((time.monotonic(), m.linear.x, m.angular.z))
    node.create_subscription(Odometry, '/ackermann_steering_controller/odometry', odom, qos_profile_sensor_data)
    node.create_subscription(Twist, '/cmd_vel', command, 10)
    try:
        deadline = time.monotonic() + 15
        while not odoms and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.1)
        if not odoms:
            raise RuntimeError('未收到仿真里程计')
        start = time.monotonic()
        enabled_until = start + args.seconds
        while time.monotonic() < enabled_until + 3:
            pub.publish(Bool(data=time.monotonic() < enabled_until))
            # 每 0.05s 把积压的回调全处理完:原来每步只处理一条再 sleep,20Hz 的 /cmd_vel
            # 只收到一半、时间戳晚到,松手前的指令被算成松手后(2026-09-26 服务器实测)
            tick_end = time.monotonic() + .05
            while time.monotonic() < tick_end:
                rclpy.spin_once(node, timeout_sec=.005)
        moving = [r for r in odoms if start <= r[0] < enabled_until]
        stopped = [r for r in commands if r[0] > enabled_until + .3]
        release_seen = any(r[0] >= enabled_until and abs(r[1])+abs(r[2]) < 1e-8 for r in commands)
        distance = sum(math.hypot(b[1]-a[1], b[2]-a[2]) for a,b in zip(moving,moving[1:]))
        result = dict(duration=args.seconds, distance=distance, odom_samples=len(moving),
                      max_speed=max((abs(r[3]) for r in moving), default=0),
                      command_samples=len(commands),
                      max_command_yaw=max((abs(r[2]) for r in commands),default=0),
                      nonzero_command=any(r[1] > 0 for r in commands),
                      released_to_zero=release_seen and all(abs(r[1])+abs(r[2]) < 1e-8 for r in stopped),
                      final_measured_speed=abs(odoms[-1][3]) if odoms else None)
        path=Path(args.out);path.parent.mkdir(parents=True,exist_ok=True)
        path.write_text(json.dumps(result,ensure_ascii=False,indent=2),encoding='utf-8')
        print(json.dumps(result))
        if not (result['nonzero_command'] and distance > .05 and result['released_to_zero']):
            raise SystemExit('仿真执行链未通过，请检查策略日志')
    finally:
        for _ in range(3):
            pub.publish(Bool(data=False))
            rclpy.spin_once(node,timeout_sec=.05)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
