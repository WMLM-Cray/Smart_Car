#!/usr/bin/env python3
# encoder_gate.py —— /encoder 合理性闸门(2026-09-18,team 侧,不碰厂家 encoder_node)
#
# 根因(2026-09-18 发车碰撞定案): 厂家 encoder_node 的 DueVelData 是无校验三字节状态机
# (帧头 01 03 02 + 2字节值,无CRC/无钳幅),串口错位时解出任意速度值 → EKF(odom0:/encoder
# 速度积分)产出 0.5~1m 假位移 → AMCL 位姿被拖偏 → 路径在现实中偏移 → 实车碰撞(已发生一次)。
# 事件证据: 三次假位移 0.47m/1.1m/0.5m+,均伴随电调接通或 cmd_vel 流量启动。
#
# 本节点: /encoder → 双门限过滤 → /encoder_gate → (ekf.yaml odom0 已指向此话题)
#   幅值门限: |linear.x| > 1.2 m/s → 丢帧 (限幅0.55+余量; 差速车无横移,vy原样转发一般=0)
#   变化率门限: |v - last_ok| > 0.35 m/s/帧 → 丢帧 (只拦跳变沿;真车受加速度1.3m/s²限制
#              帧间Δv≤0.1 不误拦; 撞墙堵转/毛刺瞬跳被拦——堵转轮速本不代表车体)
#   丢帧时 log 计数, 统计毛刺频率供后续真修 DueVelData 解析器对账。
import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node

try:  # 车机长期运行防 SIGINT 自接管(同 cmd_vel_safety 模式)
    from rclpy.signals import uninstall_signal_handlers
    uninstall_signal_handlers()
except Exception:
    pass

V_ABS_MAX = 1.2    # m/s 幅值门限
DV_MAX = 0.35      # m/s 每帧变化率门限(拦跳变沿)


def main():
    rclpy.init()
    n = Node('encoder_gate')
    pub = n.create_publisher(Odometry, '/encoder_gate', 100)
    st = {'last': None, 'drop_abs': 0, 'drop_dv': 0, 'pass': 0}

    def cb(m):
        v = m.twist.twist.linear.x  # Odometry.twist=TwistWithCovariance, 内层才是 Twist.linear
        if abs(v) > V_ABS_MAX:
            st['drop_abs'] += 1
            n.get_logger().warn(f'丢帧(幅值): v={v:.3f} 累计{st["drop_abs"]} 处理{st["pass"]}')
            return
        if st['last'] is not None and abs(v - st['last']) > DV_MAX:
            st['drop_dv'] += 1
            n.get_logger().warn(f'丢帧(跳变): {st["last"]:.3f}->{v:.3f} 累计{st["drop_dv"]} 处理{st["pass"]}')
            return
        st['last'] = v
        st['pass'] += 1
        pub.publish(m)

    n.create_subscription(Odometry, '/encoder', cb, 100)
    n.get_logger().info(f'encoder_gate 起来: |v|>{V_ABS_MAX} 或 |Δv|>{DV_MAX}/帧 丢弃')
    rclpy.spin(n)


if __name__ == '__main__':
    main()
