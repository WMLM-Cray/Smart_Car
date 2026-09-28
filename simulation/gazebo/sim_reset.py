#!/usr/bin/env python3
"""把仿真里的车搬回起点重跑一圈。

**别用 Gazebo 菜单里的复位,用这个脚本。** 三条路只有传送这条能用:

| 做法 | 能搬动车 | 之后还能跑 |
|---|---|---|
| Edit→Reset Model Poses(Ctrl+Shift+R)= /reset_world | ✗ | — |
| Edit→Reset World(Ctrl+R)= /reset_simulation | ✓ | **✗ 再也跑不了** |
| 本脚本 = /set_entity_state 传送 | ✓ | ✓ |

1. **/reset_world 对这辆车没用。** 车是运行时 spawn_entity 进来的,不在 world
   文件里,reset_world 只复位 world 里原有的模型 —— 实测调完车还停在
   (12.14, 4.39) 没动。
2. **/reset_simulation 会把车弄成砖。** 它把仿真时间倒回 0,gazebo_ros2_control
   从此不再注册 update 回调:Gazebo 照常步进(RTF 0.96、/scan 9.6Hz),但
   /joint_states 掉到 0Hz,切换控制器报 "Switch controller timed out after 5s"。
   list_controllers 仍显示 active 是假象 —— 状态机是 active,实际没人调它。
   实测重新 activate、先 unpause 再 activate 都救不回来,只能重启整个 launch。
   所以这里用 /set_entity_state:只动位姿,不动时间。
3. **光传送不够。** 策略的叠帧缓冲里还压着撞车那几帧,接着跑等于拿撞车时的
   历史开车。所以顺序是:停策略(触发 policy.reset() 清缓冲)-> 传送 -> 再使能。

另注:启动这套栈时仿真不能是暂停的。控制器若在暂停状态下 configure+activate,
硬件接口接不上,之后解暂停也没用(现象和第 2 条一样),同样只能重启。

坐标默认是原点 —— `official_to_gazebo.py` 生成的 A–D 赛道已经把起点归零了
(spawn 硬编码在 (0,0,0)),所以不带参数就是对的。旧版 trackA.world 的起点不在
原点(11.0, 0.75),那种世界要显式给 `--x 11.0 --y 0.75`。

域必须和 `run_sim.sh` 一致(默认 60),否则脚本对着一个空图说话,什么也没传送
却不报错。

用法:python3 simulation/gazebo/sim_reset.py
      python3 simulation/gazebo/sim_reset.py --x 11.0 --y 0.75    # 旧 trackA.world
"""
import argparse
import math
import time

import rclpy
from rclpy.node import Node
from gazebo_msgs.msg import EntityState
from gazebo_msgs.srv import SetEntityState
from std_msgs.msg import Bool

ENTITY = 'racecar'      # launch 里 spawn_entity 的 -entity
SPAWN_Z = 0.02          # 同 launch 的 -z,略抬防穿模


class Resetter(Node):
    def __init__(self, x=0.0, y=0.0, yaw=0.0):
        super().__init__('sim_reset')
        self.x, self.y, self.yaw = float(x), float(y), float(yaw)
        self.enable = self.create_publisher(Bool, '/policy_enable', 1)
        self.cli = self.create_client(SetEntityState, '/set_entity_state')

    def run(self):
        self.enable.publish(Bool(data=False))       # 先停,policy.reset() 清叠帧
        time.sleep(0.5)
        if not self.cli.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('/set_entity_state 没响应,仿真起来了吗?')
            return

        state = EntityState()
        state.name = ENTITY
        state.pose.position.x = self.x
        state.pose.position.y = self.y
        state.pose.position.z = SPAWN_Z
        state.pose.orientation.z = math.sin(self.yaw / 2)
        state.pose.orientation.w = math.cos(self.yaw / 2)
        req = SetEntityState.Request()
        req.state = state
        fut = self.cli.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=5.0)
        if fut.result() is None or not fut.result().success:
            self.get_logger().error('传送失败')
            return

        time.sleep(1.0)                             # 等车落稳,别拿空中的位姿开跑
        self.enable.publish(Bool(data=True))
        self.get_logger().info('已复位到起点并重新使能')


def main():
    ap = argparse.ArgumentParser(description='把仿真里的车传送回起点')
    ap.add_argument('--x', type=float, default=0.0)
    ap.add_argument('--y', type=float, default=0.0)
    ap.add_argument('--yaw', type=float, default=0.0, help='rad')
    a = ap.parse_args()
    rclpy.init()
    node = Resetter(a.x, a.y, a.yaw)
    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
