#!/usr/bin/env python3
"""Export the existing SLAM /map to a timestamped PGM+YAML ZIP. Never drives the car."""
import argparse
import json
import math
from pathlib import Path
import time
import zipfile


def encode_map(msg):
    w, h = msg.info.width, msg.info.height
    if w <= 0 or h <= 0 or len(msg.data) != w * h or msg.info.resolution <= 0:
        raise ValueError('Invalid occupancy grid')
    # ROS grid starts at bottom-left; PGM stores the top row first.
    pixels = bytearray()
    for y in range(h - 1, -1, -1):
        for value in msg.data[y*w:(y+1)*w]:
            pixels.append(205 if value < 0 or 25 <= value <= 65 else 0 if value > 65 else 254)
    pgm = f'P5\n{w} {h}\n255\n'.encode() + pixels
    p, q = msg.info.origin.position, msg.info.origin.orientation
    yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
    yaml = (f'image: map.pgm\nmode: trinary\nresolution: {msg.info.resolution}\n'
            f'origin: [{p.x}, {p.y}, {yaw}]\nnegate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.25\n')
    return pgm, yaml


def main():
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
    from nav_msgs.msg import OccupancyGrid
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--out', default=str(Path.home() / 'maps'))
    parser.add_argument('--timeout', type=float, default=20.)
    args = parser.parse_args()
    rclpy.init()
    node = Node('corridor_map_export')
    received = []
    subscriptions = []
    try:
        # Discover the publisher's actual durability, so both latched and live maps work.
        deadline = time.monotonic() + args.timeout
        while time.monotonic() < deadline and not subscriptions:
            rclpy.spin_once(node, timeout_sec=0.1)
            for info in node.get_publishers_info_by_topic('/map'):
                qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT,
                                 durability=info.qos_profile.durability)
                subscriptions.append(node.create_subscription(OccupancyGrid, '/map', received.append, qos))
        while time.monotonic() < deadline and not received:
            rclpy.spin_once(node, timeout_sec=0.1)
        if not received:
            raise RuntimeError('No /map received; confirm SLAM is publishing before exporting')
        msg = received[-1]
        pgm, yaml = encode_map(msg)
        dest = Path(args.out)
        dest.mkdir(parents=True, exist_ok=True)
        path = dest / ('corridor_' + time.strftime('%Y%m%d_%H%M%S') + '_' + str(time.time_ns())[-6:] + '.zip')
        metadata = {'frame': msg.header.frame_id, 'width': msg.info.width, 'height': msg.info.height,
                    'resolution': msg.info.resolution, 'map_stamp_sec': msg.header.stamp.sec,
                    'export_time_sec': time.time(),
                    'note': 'SLAM snapshot, not a guarantee of map accuracy. Check odometry and TF.'}
        with zipfile.ZipFile(path, 'x', zipfile.ZIP_DEFLATED) as archive:
            archive.writestr('map.pgm', pgm)
            archive.writestr('map.yaml', yaml)
            archive.writestr('metadata.json', json.dumps(metadata, indent=2))
        print(json.dumps({'file': str(path), **metadata}))
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
