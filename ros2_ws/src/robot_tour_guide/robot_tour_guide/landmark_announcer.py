"""Standalone symbol-to-description announcer.

Subscribes to ``/landmarks/detected``, loads ``landmarks.yaml``, and publishes
the matching description to ``/tour/narration`` once per landmark per
``cooldown_s``. Lets the demo run *without* the tour planner / executive:
drive around with teleop, point the camera at a marker, hear the description.
"""

import json
import os
import time

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from std_msgs.msg import String


class LandmarkAnnouncer(Node):

    def __init__(self):
        super().__init__('landmark_announcer')

        self.declare_parameter('landmarks_file', '')
        self.declare_parameter('max_distance_m', 1.50)
        self.declare_parameter('dwell_s', 0.7)
        self.declare_parameter('cooldown_s', 12.0)

        path = self.get_parameter('landmarks_file').value
        if not path:
            path = os.path.join(
                get_package_share_directory('robot_tour_guide'),
                'config', 'landmarks.yaml',
            )
        try:
            with open(path, 'r') as f:
                data = yaml.safe_load(f) or {}
            self.landmarks = {int(k): v for k, v in (data.get('landmarks') or {}).items()}
        except FileNotFoundError:
            self.get_logger().warn(f'landmarks file not found: {path}')
            self.landmarks = {}

        self.first_seen: dict = {}      # id -> first-time-in-this-encounter
        self.last_announced: dict = {}  # id -> wall-clock when last spoken

        self.create_subscription(String, '/landmarks/detected', self.on_detect, 10)
        self.narration_pub = self.create_publisher(String, '/tour/narration', 10)

        self.get_logger().info(
            f'Landmark announcer ready ({len(self.landmarks)} landmarks loaded).'
        )

    def on_detect(self, msg: String):
        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError:
            return

        marker_id = int(payload['id'])
        distance = float(payload.get('distance_m', 1e9))
        max_d = float(self.get_parameter('max_distance_m').value)
        if distance > max_d:
            self.first_seen.pop(marker_id, None)
            return

        now = time.time()
        if marker_id not in self.first_seen:
            self.first_seen[marker_id] = now
            return

        dwell = now - self.first_seen[marker_id]
        if dwell < float(self.get_parameter('dwell_s').value):
            return

        last = self.last_announced.get(marker_id, 0.0)
        if now - last < float(self.get_parameter('cooldown_s').value):
            return

        entry = self.landmarks.get(marker_id)
        if entry is None:
            self.get_logger().info(f'Unknown landmark id={marker_id} (no DB entry).')
            text = f"Unknown marker {marker_id} detected at {distance:.2f} metres."
        else:
            name = entry.get('name', f'Landmark {marker_id}')
            description = entry.get('description', '').strip()
            text = f"{name}. {description}"

        self.last_announced[marker_id] = now
        self.first_seen.pop(marker_id, None)   # require a fresh dwell next time
        self.narration_pub.publish(String(data=text))


def main(args=None):
    rclpy.init(args=args)
    node = LandmarkAnnouncer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
