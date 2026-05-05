"""Tour planner: orders points-of-interest into a tour and supports replanning.

Uses nearest-neighbour construction + 2-opt refinement on the Euclidean
distance matrix. This is exact enough for the small POI counts used in
REPF B4 (typically 4–10) while remaining a deliberative component.

The planner is a service-and-topic node:
  * Service ``/tour/plan``        — request (re)planning from a given start pose
  * Topic   ``/tour/current_plan`` — JSON of the active plan
  * Topic   ``/tour/drop_poi``     — String message with POI id to permanently drop
"""

import json
import math
import os
import time
from typing import List, Tuple

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from std_msgs.msg import String


class TourPlanner(Node):

    def __init__(self):
        super().__init__('tour_planner')

        self.declare_parameter('pois_file', '')
        self.declare_parameter('start_x', 0.0)
        self.declare_parameter('start_y', 0.0)
        self.declare_parameter('autoplan_on_start', True)

        path = self.get_parameter('pois_file').value
        if not path:
            path = os.path.join(
                get_package_share_directory('robot_tour_guide'),
                'config', 'pois.yaml',
            )
        self.pois = self._load_pois(path)
        self.dropped: set = set()

        self.plan_pub = self.create_publisher(String, '/tour/current_plan', 1)
        self.create_subscription(String, '/tour/replan_request', self.on_replan, 5)
        self.create_subscription(String, '/tour/drop_poi',       self.on_drop,   5)

        self.current_plan: List[dict] = []
        if self.get_parameter('autoplan_on_start').value:
            sx = float(self.get_parameter('start_x').value)
            sy = float(self.get_parameter('start_y').value)
            self.replan(sx, sy, reason='autoplan')

        self.create_timer(1.0, self._republish)
        self.get_logger().info(f'Tour planner ready. Loaded {len(self.pois)} POIs.')

    # ------------------------------------------------------------------ I/O
    def _load_pois(self, path: str) -> List[dict]:
        with open(path, 'r') as f:
            data = yaml.safe_load(f)
        pois = data.get('pois', [])
        for p in pois:
            assert {'id', 'name', 'x', 'y'}.issubset(p.keys()), \
                f'POI missing required fields: {p}'
        return pois

    def on_replan(self, msg: String):
        try:
            req = json.loads(msg.data)
        except json.JSONDecodeError:
            self.get_logger().warn('replan_request: bad JSON')
            return
        sx = float(req.get('start_x', 0.0))
        sy = float(req.get('start_y', 0.0))
        self.replan(sx, sy, reason=req.get('reason', 'external'))

    def on_drop(self, msg: String):
        try:
            poi_id = int(msg.data)
        except ValueError:
            return
        self.dropped.add(poi_id)
        self.get_logger().info(f'Dropping POI id={poi_id} from future tours.')
        if self.current_plan:
            sx = self.current_plan[0]['x'] if self.current_plan else 0.0
            sy = self.current_plan[0]['y'] if self.current_plan else 0.0
            self.replan(sx, sy, reason='poi_dropped')

    # -------------------------------------------------------------- planning
    def replan(self, start_x: float, start_y: float, reason: str = '') -> None:
        active = [p for p in self.pois if p['id'] not in self.dropped]
        if not active:
            self.current_plan = []
            self._republish()
            return
        order = self._nearest_neighbour(active, start_x, start_y)
        order = self._two_opt(order, start_x, start_y)
        self.current_plan = order
        self.get_logger().info(
            f'Plan ({reason}): ' + ' -> '.join(p['name'] for p in order)
        )
        self._republish()

    @staticmethod
    def _dist(a: Tuple[float, float], b: Tuple[float, float]) -> float:
        return math.hypot(a[0] - b[0], a[1] - b[1])

    def _nearest_neighbour(self, pois: List[dict], sx: float, sy: float) -> List[dict]:
        remaining = list(pois)
        order: List[dict] = []
        cur = (sx, sy)
        while remaining:
            nxt = min(remaining, key=lambda p: self._dist(cur, (p['x'], p['y'])))
            order.append(nxt)
            cur = (nxt['x'], nxt['y'])
            remaining.remove(nxt)
        return order

    def _two_opt(self, order: List[dict], sx: float, sy: float) -> List[dict]:
        if len(order) < 4:
            return order
        coords = [(sx, sy)] + [(p['x'], p['y']) for p in order]

        def total_len(seq):
            return sum(self._dist(seq[i], seq[i + 1]) for i in range(len(seq) - 1))

        best = list(range(len(coords)))
        best_len = total_len([coords[i] for i in best])
        improved = True
        iteration = 0
        while improved and iteration < 100:
            improved = False
            iteration += 1
            for i in range(1, len(best) - 2):
                for j in range(i + 1, len(best) - 1):
                    candidate = best[:i] + best[i:j + 1][::-1] + best[j + 1:]
                    candidate_len = total_len([coords[k] for k in candidate])
                    if candidate_len + 1e-6 < best_len:
                        best, best_len = candidate, candidate_len
                        improved = True
        return [order[k - 1] for k in best[1:]]

    # ---------------------------------------------------------------- output
    def _republish(self):
        msg = String()
        msg.data = json.dumps({
            'stamp': time.time(),
            'plan': self.current_plan,
            'dropped': sorted(self.dropped),
        })
        self.plan_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = TourPlanner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
