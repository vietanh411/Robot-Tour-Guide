"""World model: keeps a fused, time-decayed view of detected objects.

Subscribes to:
  /world/objects        — raw detections from semantic_perception
  /landmarks/detected   — ArUco detections from aruco_detector

Publishes:
  /world/state          — JSON snapshot consumed by the executive
  /world/markers        — RViz visualization (MarkerArray)

Persistence rules:
  * person          — short half-life (1.0 s) — they move
  * furniture       — long  half-life (30 s)  — they're approximately static
  * door_open       — medium half-life (3 s)  — flickery but worth keeping
  * landmark        — sticky for 5 s after last sight
"""

import json
import math
import time
from collections import OrderedDict

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point


HALFLIFE_S = {
    'person': 1.0,
    'person_candidate': 1.0,
    'furniture': 30.0,
    'furniture_candidate': 15.0,
    'door_open': 3.0,
    'small_obstacle': 5.0,
    'landmark': 5.0,
}

DEFAULT_HALFLIFE = 5.0


class WorldModel(Node):

    def __init__(self):
        super().__init__('world_model')

        self.declare_parameter('publish_period_s', 0.25)
        self.declare_parameter('drop_below_weight', 0.05)
        self.declare_parameter('frame_id', 'base_link')

        self.objects: "OrderedDict[str, dict]" = OrderedDict()
        self.landmarks_seen: dict = {}

        self.create_subscription(String, '/world/objects', self.on_objects, 10)
        self.create_subscription(String, '/landmarks/detected', self.on_landmark, 10)

        self.state_pub = self.create_publisher(String, '/world/state', 10)
        self.marker_pub = self.create_publisher(MarkerArray, '/world/markers', 5)

        self.create_timer(float(self.get_parameter('publish_period_s').value), self.tick)

        self.get_logger().info('World model ready.')

    # ----- ingest -----------------------------------------------------------
    def on_objects(self, msg: String):
        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        now = time.time()
        for o in payload.get('objects', []):
            key = self._object_key(o)
            self.objects[key] = {**o, 'last_seen': now, 'first_seen': self.objects.get(key, {}).get('first_seen', now)}

    def on_landmark(self, msg: String):
        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        landmark_id = int(payload['id'])
        now = time.time()
        self.landmarks_seen[landmark_id] = {
            **payload,
            'class': 'landmark',
            'last_seen': now,
            'first_seen': self.landmarks_seen.get(landmark_id, {}).get('first_seen', now),
        }

    # ----- emit -------------------------------------------------------------
    def tick(self):
        cutoff = float(self.get_parameter('drop_below_weight').value)
        now = time.time()
        kept = OrderedDict()
        for key, obj in self.objects.items():
            weight = self._decay_weight(obj['class'], now - obj['last_seen'])
            if weight >= cutoff:
                kept[key] = {**obj, 'weight': weight}
        self.objects = kept

        kept_lm = {}
        for lid, lm in self.landmarks_seen.items():
            weight = self._decay_weight('landmark', now - lm['last_seen'])
            if weight >= cutoff:
                kept_lm[lid] = {**lm, 'weight': weight}
        self.landmarks_seen = kept_lm

        snapshot = {
            'stamp': now,
            'frame_id': self.get_parameter('frame_id').value,
            'objects': list(self.objects.values()),
            'landmarks': list(self.landmarks_seen.values()),
            'counts': self._counts(),
        }
        self.state_pub.publish(String(data=json.dumps(snapshot)))
        self.marker_pub.publish(self._markers(snapshot))

    def _counts(self) -> dict:
        c: dict = {}
        for o in self.objects.values():
            c[o['class']] = c.get(o['class'], 0) + 1
        if self.landmarks_seen:
            c['landmark'] = len(self.landmarks_seen)
        return c

    @staticmethod
    def _decay_weight(cls: str, age_s: float) -> float:
        hl = HALFLIFE_S.get(cls, DEFAULT_HALFLIFE)
        return math.pow(0.5, age_s / hl)

    @staticmethod
    def _object_key(o: dict) -> str:
        # Cluster IDs are not stable across scans; fall back to coarse spatial bucket.
        bx = round(o.get('x', 0.0) / 0.25)
        by = round(o.get('y', 0.0) / 0.25)
        return f"{o.get('source', 'x')}:{o.get('class', 'unknown')}:{bx}:{by}"

    def _markers(self, snapshot: dict) -> MarkerArray:
        arr = MarkerArray()
        idx = 0
        frame = snapshot['frame_id']
        for o in snapshot['objects']:
            m = Marker()
            m.header.frame_id = frame
            m.header.stamp = self.get_clock().now().to_msg()
            m.ns = o['class']
            m.id = idx
            m.type = Marker.SPHERE
            m.action = Marker.ADD
            m.pose.position = Point(x=float(o.get('x', 0.0)),
                                    y=float(o.get('y', 0.0)),
                                    z=0.2)
            m.pose.orientation.w = 1.0
            scale = max(0.15, float(o.get('width', 0.2)))
            m.scale.x = m.scale.y = m.scale.z = scale
            r, g, b = self._color_for(o['class'])
            m.color.r, m.color.g, m.color.b = r, g, b
            m.color.a = float(o.get('weight', 0.5))
            m.lifetime.sec = 1
            arr.markers.append(m)
            idx += 1
        return arr

    @staticmethod
    def _color_for(cls: str):
        return {
            'person': (1.0, 0.2, 0.2),
            'person_candidate': (1.0, 0.5, 0.5),
            'furniture': (0.2, 0.6, 1.0),
            'furniture_candidate': (0.4, 0.7, 1.0),
            'door_open': (0.2, 1.0, 0.2),
            'small_obstacle': (0.6, 0.6, 0.6),
            'landmark': (1.0, 1.0, 0.0),
        }.get(cls, (0.8, 0.8, 0.8))


def main(args=None):
    rclpy.init(args=args)
    node = WorldModel()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
