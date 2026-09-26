"""Read-only, bounded LaserScan payload; no ROS dependency or motor commands."""
import math


def scan_payload(msg):
    points = []
    # Keep all returns (including thin obstacles); cap malformed/oversized messages.
    step = max(1, math.ceil(len(msg.ranges) / 2000))
    for i in range(0, len(msg.ranges), step):
        r = msg.ranges[i]
        if not math.isfinite(r) or not msg.range_min <= r <= msg.range_max:
            continue
        a = msg.angle_min + i * msg.angle_increment
        points.append([round(r * math.cos(a), 3), round(r * math.sin(a), 3)])
    return {'points': points, 'frame': msg.header.frame_id,
            'stamp': msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9}
