"""Semantic perception: people, furniture, and doors.

Combines two cheap classifiers:
  * YOLO on the OAK-D RGB stream (people / chair / table / desk).
  * LiDAR scan clustering for shape-level checks (cluster width, gap detection
    used as a door-open hint).

YOLO is optional — if `ultralytics` is not installed the node silently
degrades to LiDAR-only output, which still lets the rest of the stack run.
"""

import json
import math
import time
from dataclasses import dataclass, field
from typing import List, Optional

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image, LaserScan
from std_msgs.msg import String

try:
    from cv_bridge import CvBridge
    HAVE_CV_BRIDGE = True
except ImportError:
    HAVE_CV_BRIDGE = False

try:
    from ultralytics import YOLO
    HAVE_YOLO = True
except ImportError:
    HAVE_YOLO = False


PERSON_CLASSES = {'person'}
FURNITURE_CLASSES = {'chair', 'couch', 'sofa', 'bench', 'dining table', 'desk'}


@dataclass
class TrackedObject:
    obj_class: str
    x: float
    y: float
    width: float
    confidence: float
    last_seen: float
    source: str
    id: int = 0
    extras: dict = field(default_factory=dict)


class SemanticPerception(Node):

    def __init__(self):
        super().__init__('semantic_perception')

        self.declare_parameter('image_topic', '/oakd/rgb/preview/image_raw')
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('yolo_weights', 'yolov8n.pt')
        self.declare_parameter('yolo_conf', 0.45)
        self.declare_parameter('yolo_period_s', 0.20)         # ~5 Hz inference
        self.declare_parameter('cluster_gap_m', 0.20)
        self.declare_parameter('min_cluster_points', 4)
        self.declare_parameter('max_cluster_range_m', 4.0)
        self.declare_parameter('person_width_range_m', [0.25, 0.80])
        self.declare_parameter('door_gap_min_m', 0.70)
        self.declare_parameter('door_gap_max_m', 1.20)
        self.declare_parameter('publish_period_s', 0.20)

        self.bridge = CvBridge() if HAVE_CV_BRIDGE else None
        self.last_yolo_time = 0.0
        self.latest_yolo: List[TrackedObject] = []
        self.latest_clusters: List[TrackedObject] = []

        self.model = None
        if HAVE_YOLO:
            try:
                weights = self.get_parameter('yolo_weights').value
                self.model = YOLO(weights)
                self.get_logger().info(f'YOLO loaded ({weights}).')
            except Exception as exc:
                self.get_logger().warn(f'YOLO unavailable, running LiDAR-only: {exc}')
        else:
            self.get_logger().warn('ultralytics not installed; running LiDAR-only perception.')

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )

        self.create_subscription(
            LaserScan,
            self.get_parameter('scan_topic').value,
            self.on_scan,
            sensor_qos,
        )
        if self.model is not None and self.bridge is not None:
            self.create_subscription(
                Image,
                self.get_parameter('image_topic').value,
                self.on_image,
                sensor_qos,
            )

        self.objects_pub = self.create_publisher(String, '/world/objects', 10)

        period = float(self.get_parameter('publish_period_s').value)
        self.create_timer(period, self.publish_state)

        self.get_logger().info('Semantic perception ready.')

    # ------------------------------------------------------------------ YOLO
    def on_image(self, msg: Image):
        now = time.monotonic()
        if now - self.last_yolo_time < float(self.get_parameter('yolo_period_s').value):
            return
        self.last_yolo_time = now

        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as exc:
            self.get_logger().warn(f'cv_bridge failure: {exc}')
            return

        conf = float(self.get_parameter('yolo_conf').value)
        try:
            results = self.model.predict(frame, conf=conf, verbose=False)
        except Exception as exc:
            self.get_logger().warn(f'YOLO inference failed: {exc}')
            return

        detections: List[TrackedObject] = []
        if not results:
            self.latest_yolo = []
            return
        names = results[0].names
        for box in results[0].boxes:
            cls_id = int(box.cls[0])
            cls_name = names.get(cls_id, str(cls_id)) if isinstance(names, dict) else names[cls_id]
            conf = float(box.conf[0])
            xyxy = box.xyxy[0].cpu().numpy().tolist()
            label = self._classify_yolo(cls_name)
            if label is None:
                continue
            detections.append(TrackedObject(
                obj_class=label,
                x=0.0, y=0.0,                   # image-only; world_model fuses with LiDAR
                width=0.0,
                confidence=conf,
                last_seen=time.time(),
                source='yolo',
                extras={'bbox': xyxy, 'raw_class': cls_name},
            ))
        self.latest_yolo = detections

    def _classify_yolo(self, cls_name: str) -> Optional[str]:
        if cls_name in PERSON_CLASSES:
            return 'person'
        if cls_name in FURNITURE_CLASSES:
            return 'furniture'
        return None

    # ----------------------------------------------------------------- LIDAR
    def on_scan(self, msg: LaserScan):
        gap = float(self.get_parameter('cluster_gap_m').value)
        min_pts = int(self.get_parameter('min_cluster_points').value)
        max_r = float(self.get_parameter('max_cluster_range_m').value)
        person_lo, person_hi = self.get_parameter('person_width_range_m').value
        door_lo = float(self.get_parameter('door_gap_min_m').value)
        door_hi = float(self.get_parameter('door_gap_max_m').value)

        ranges = np.asarray(msg.ranges, dtype=np.float32)
        n = ranges.size
        if n == 0:
            return
        angles = msg.angle_min + np.arange(n, dtype=np.float32) * msg.angle_increment
        valid = np.isfinite(ranges) & (ranges > msg.range_min) & (ranges < min(msg.range_max, max_r))

        clusters: List[TrackedObject] = []
        i = 0
        cluster_id = 0
        gap_starts: List[int] = []     # indices where a wall→space transition happens
        gap_ends: List[int] = []
        prev_valid = False
        while i < n:
            if not valid[i]:
                if prev_valid:
                    gap_starts.append(i - 1)
                prev_valid = False
                i += 1
                continue
            j = i
            xs, ys = [], []
            while j < n and valid[j] and (j == i or abs(ranges[j] - ranges[j - 1]) < gap):
                xs.append(ranges[j] * math.cos(angles[j]))
                ys.append(ranges[j] * math.sin(angles[j]))
                j += 1
            if (j - i) >= min_pts:
                xs_a, ys_a = np.array(xs), np.array(ys)
                width = float(math.hypot(xs_a[-1] - xs_a[0], ys_a[-1] - ys_a[0]))
                cx, cy = float(xs_a.mean()), float(ys_a.mean())
                label = self._classify_cluster(width, person_lo, person_hi)
                clusters.append(TrackedObject(
                    obj_class=label,
                    x=cx, y=cy,
                    width=width,
                    confidence=0.6,
                    last_seen=time.time(),
                    source='lidar',
                    id=cluster_id,
                    extras={'point_count': len(xs)},
                ))
                cluster_id += 1
            if not prev_valid and gap_starts and len(gap_ends) < len(gap_starts):
                gap_ends.append(i)
            prev_valid = True
            i = j

        # Door candidates: a passable gap framed by two LiDAR returns at similar range.
        # We approximate by looking for adjacent invalid runs whose chord-length is in [door_lo, door_hi].
        for s_idx, e_idx in zip(gap_starts, gap_ends):
            if not (0 <= s_idx < n and 0 <= e_idx < n):
                continue
            r1, r2 = float(ranges[s_idx]), float(ranges[e_idx])
            if not (math.isfinite(r1) and math.isfinite(r2)):
                continue
            a1, a2 = float(angles[s_idx]), float(angles[e_idx])
            x1, y1 = r1 * math.cos(a1), r1 * math.sin(a1)
            x2, y2 = r2 * math.cos(a2), r2 * math.sin(a2)
            chord = math.hypot(x2 - x1, y2 - y1)
            if door_lo <= chord <= door_hi and abs(r1 - r2) < 0.5:
                clusters.append(TrackedObject(
                    obj_class='door_open',
                    x=(x1 + x2) / 2.0,
                    y=(y1 + y2) / 2.0,
                    width=chord,
                    confidence=0.5,
                    last_seen=time.time(),
                    source='lidar',
                    extras={'frame_a': [x1, y1], 'frame_b': [x2, y2]},
                ))
        self.latest_clusters = clusters

    @staticmethod
    def _classify_cluster(width_m: float, p_lo: float, p_hi: float) -> str:
        if p_lo <= width_m <= p_hi:
            return 'person_candidate'
        if width_m > p_hi:
            return 'furniture_candidate'
        return 'small_obstacle'

    # ----------------------------------------------------------------- OUT
    def publish_state(self):
        payload = {
            'stamp': time.time(),
            'frame_id': 'base_link',
            'objects': [self._to_dict(o) for o in (self.latest_yolo + self.latest_clusters)],
        }
        self.objects_pub.publish(String(data=json.dumps(payload)))

    @staticmethod
    def _to_dict(o: TrackedObject) -> dict:
        return {
            'class': o.obj_class,
            'x': o.x, 'y': o.y,
            'width': o.width,
            'confidence': o.confidence,
            'last_seen': o.last_seen,
            'source': o.source,
            'id': o.id,
            'extras': o.extras,
        }


def main(args=None):
    rclpy.init(args=args)
    node = SemanticPerception()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
