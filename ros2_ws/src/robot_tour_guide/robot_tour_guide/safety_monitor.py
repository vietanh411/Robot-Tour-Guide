"""Reactive safety monitor.

Watches /scan for any return below ``stop_distance_m`` in the forward arc.
When tripped, publishes a zero TwistStamped on /cmd_vel for ``hold_s`` seconds
to override Nav2's controller, and surfaces a warning on /safety/event.

Runs on the Pi for low-latency reflex behaviour. Does not preempt Nav2; once
the obstacle clears, normal navigation resumes.
"""

import math
import time

import rclpy
from geometry_msgs.msg import TwistStamped
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String


class SafetyMonitor(Node):

    def __init__(self):
        super().__init__('safety_monitor')
        self.declare_parameter('stop_distance_m', 0.25)
        self.declare_parameter('forward_arc_deg', 60.0)
        self.declare_parameter('hold_s', 0.5)
        self.declare_parameter('publish_rate_hz', 20.0)

        self.tripped_until = 0.0
        self.last_min_range = math.inf

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )
        self.create_subscription(LaserScan, '/scan', self.on_scan, sensor_qos)

        self.cmd_pub = self.create_publisher(TwistStamped, '/cmd_vel', 10)
        self.event_pub = self.create_publisher(String, '/safety/event', 5)

        rate = float(self.get_parameter('publish_rate_hz').value)
        self.create_timer(1.0 / rate, self.tick)

        self.get_logger().info('Safety monitor ready.')

    def on_scan(self, msg: LaserScan):
        n = len(msg.ranges)
        if n == 0:
            return
        arc = math.radians(float(self.get_parameter('forward_arc_deg').value))
        # Indices within +/- arc/2 of forward (angle 0).
        # Forward is at angle 0; convert to scan index range.
        forward_idx = int(round((-msg.angle_min) / msg.angle_increment))
        half_span = int(round((arc / 2.0) / msg.angle_increment))
        lo = max(0, forward_idx - half_span)
        hi = min(n, forward_idx + half_span + 1)
        window = [r for r in msg.ranges[lo:hi]
                  if math.isfinite(r) and r > msg.range_min]
        if not window:
            return
        self.last_min_range = min(window)
        if self.last_min_range < float(self.get_parameter('stop_distance_m').value):
            self.tripped_until = time.monotonic() + float(self.get_parameter('hold_s').value)
            self.event_pub.publish(String(data=f'STOP min_range={self.last_min_range:.2f}'))

    def tick(self):
        if time.monotonic() < self.tripped_until:
            stop = TwistStamped()
            stop.header.stamp = self.get_clock().now().to_msg()
            stop.header.frame_id = 'base_link'
            self.cmd_pub.publish(stop)


def main(args=None):
    rclpy.init(args=args)
    node = SafetyMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
