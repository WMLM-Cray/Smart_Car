"""Read-only check of mapping inputs, TF and existing publishers."""
import json
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from tf2_ros import Buffer, TransformListener

rclpy.init()
node = Node('mapping_input_probe')
buffer = Buffer()
listener = TransformListener(buffer, node)
counts = {'scan': 0, 'encoder': 0, 'odom': 0}
last = {}
def receive(key, msg):
    counts[key] += 1
    last[key] = msg
for topic, cls, key in [('/scan', LaserScan, 'scan'), ('/encoder', Odometry, 'encoder'),
                        ('/odom_combined', Odometry, 'odom')]:
    node.create_subscription(cls, topic, lambda msg, k=key: receive(k, msg), qos_profile_sensor_data)
end = time.monotonic() + 6
while time.monotonic() < end:
    rclpy.spin_once(node, timeout_sec=0.1)
result = {'received': counts, 'map_publishers': node.count_publishers('/map'),
          'nodes': node.get_node_names()}
for target, source in [('odom_combined', 'base_footprint'), ('base_footprint', 'laser_link')]:
    try:
        tf = buffer.lookup_transform(target, source, rclpy.time.Time())
        result[target + '<-' + source] = {'stamp': tf.header.stamp.sec, 'ok': True}
    except Exception as exc:
        result[target + '<-' + source] = {'error': str(exc)}
print(json.dumps(result))
node.destroy_node()
rclpy.shutdown()
