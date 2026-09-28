#!/usr/bin/env python3
"""把仿真里的车传送回起点,重跑一圈不用重启仿真。

    python3 simulation/gazebo/sim_reset.py runs/gz_official/track_A/layout.yaml   # 回 layout 的起点
    python3 simulation/gazebo/sim_reset.py --x 11.0 --y 0.75 --yaw 0               # 任意位姿(yaw 弧度)
    然后重新起心跳:python3 simulation/gazebo/hold_enable.py

**别用 Gazebo 菜单的复位**:Reset Model Poses(/reset_world)搬不动运行时 spawn 的车;
Reset World(/reset_simulation)把仿真时间倒回 0,gazebo_ros2_control 从此不再
update,车变砖,只能重启 launch。这里用 /set_entity_state,只动位姿不动时间。

顺序和原因:
  1. **先停掉别的心跳源**(hold_enable.py / ros2 topic pub -r)。/policy_enable 是心跳,
     它在发 true 时这里发的 false 会被立刻盖掉,策略根本停不下来 —— 检测到就拒绝。
  2. 等策略节点订阅上再发 false,并连发一阵。刚建的发布者马上发,DDS 还没发现订阅方,
     第一条会丢(PR #10 版本实测:策略节点一条日志都没收到)。
  3. 策略收到 false 会发零速并清叠帧缓冲(不拿撞车前的历史接着开),然后传送。
  4. 本脚本**不负责重新使能**:单发一次 true 撑不过 0.3s 心跳超时。要接着跑就重新起
     hold_enable.py。

域必须和仿真一致(run_sim.sh 默认 60、run_*_once.sh 默认 43+worker),拒绝实车域 42。
"""
import argparse
import math
import os
import sys
import time

CAR_DOMAIN = '42'
ENTITY = 'racecar'      # launch 里 spawn_entity 的 -entity
SPAWN_Z = 0.02          # 同 launch 的 -z,略抬防穿模


def start_from_layout(path):
    import yaml
    with open(path, encoding='utf-8') as f:
        sp = yaml.safe_load(f)['start_pose']
    return float(sp['xy'][0]), float(sp['xy'][1]), math.radians(float(sp['yaw_deg']))


def parse(argv=None):
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('layout', nargs='?', help='official_to_gazebo.py 生成的 layout.yaml')
    ap.add_argument('--x', type=float)
    ap.add_argument('--y', type=float)
    ap.add_argument('--yaw', type=float, default=0.0, help='弧度')
    a = ap.parse_args(argv)
    if a.layout:
        return start_from_layout(a.layout)
    if a.x is None or a.y is None:
        ap.error('给 layout.yaml,或者同时给 --x --y')
    return a.x, a.y, a.yaw


def main(argv=None):
    x, y, yaw = parse(argv)
    domain = os.environ.get('ROS_DOMAIN_ID')
    if domain is None or domain == CAR_DOMAIN:
        sys.exit(f'ROS_DOMAIN_ID 要设成仿真域(不能不设,也不能是实车的 {CAR_DOMAIN})')

    import rclpy
    from gazebo_msgs.msg import EntityState
    from gazebo_msgs.srv import SetEntityState
    from std_msgs.msg import Bool

    rclpy.init()
    node = rclpy.create_node('sim_reset')
    pub = node.create_publisher(Bool, '/policy_enable', 10)
    cli = node.create_client(SetEntityState, '/set_entity_state')
    try:
        deadline = time.monotonic() + 5.0
        while pub.get_subscription_count() == 0 and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
        if pub.get_subscription_count() == 0:
            node.get_logger().warn('没有策略节点订阅 /policy_enable,只做传送')
        elif node.count_publishers('/policy_enable') > 1:
            sys.exit('还有别的 /policy_enable 发布者(hold_enable.py?)在发心跳,'
                     '先停掉它,否则这里发的 false 会被盖掉、策略停不下来')
        else:
            for _ in range(10):                         # 0.5s,策略发零速并清叠帧
                pub.publish(Bool(data=False))
                rclpy.spin_once(node, timeout_sec=0.05)

        if not cli.wait_for_service(timeout_sec=5.0):
            sys.exit('/set_entity_state 没响应:仿真没起来,或 ROS_DOMAIN_ID 和仿真不一致')
        state = EntityState()
        state.name = ENTITY
        state.pose.position.x, state.pose.position.y, state.pose.position.z = x, y, SPAWN_Z
        state.pose.orientation.z = math.sin(yaw / 2)
        state.pose.orientation.w = math.cos(yaw / 2)
        req = SetEntityState.Request()
        req.state = state                               # twist 默认全零:传送后是静止的
        fut = cli.call_async(req)
        rclpy.spin_until_future_complete(node, fut, timeout_sec=5.0)
        if fut.result() is None or not fut.result().success:
            sys.exit('传送失败')
        print(f'已传送到 ({x:.3f}, {y:.3f}, yaw {math.degrees(yaw):.1f}°);'
              f'接着跑请重新起 hold_enable.py')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
