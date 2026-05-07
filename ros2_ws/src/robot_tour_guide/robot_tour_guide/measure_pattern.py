"""Interactive pattern measurement helper.

Drive the robot manually with teleop. Press Enter in this terminal each
time the robot reaches a stop. The tool prints the YAML step(s) needed to
get from the previous waypoint to the current one — paste them straight
into ``config/pattern.yaml``.

Usage (two terminals)::

    # Terminal 1 - this tool
    ros2 run robot_tour_guide measure_pattern

    # Terminal 2 - drive the robot
    ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -p stamped:=true

At each stop:
    * Just press Enter to record a waypoint with no announcement.
    * Type a marker id (e.g. ``2``) then Enter to add an ``announce`` step
      for that landmark.
    * Type ``q`` (or Ctrl-D) to finish and print the full pattern.
"""

import math
import sys
import threading
from typing import List, Optional, Tuple

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node


def quaternion_to_yaw(q) -> float:
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def normalize_angle(a: float) -> float:
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


class MeasurePattern(Node):

    def __init__(self):
        super().__init__('measure_pattern')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('forward_threshold_m', 0.05)
        self.declare_parameter('rotate_threshold_deg', 5.0)
        self.declare_parameter('dwell_s', 6.0)
        self.declare_parameter('timeout_s', 12.0)

        self.current_pose: Optional[Tuple[float, float, float]] = None
        self.waypoints: List[dict] = []
        self.fwd_thresh = float(self.get_parameter('forward_threshold_m').value)
        self.rot_thresh = math.radians(
            float(self.get_parameter('rotate_threshold_deg').value))
        self.dwell = float(self.get_parameter('dwell_s').value)
        self.timeout = float(self.get_parameter('timeout_s').value)

        self.create_subscription(
            Odometry,
            self.get_parameter('odom_topic').value,
            self._on_odom,
            10,
        )
        self.create_timer(0.5, self._show_status)

        threading.Thread(target=self._stdin_loop, daemon=True).start()

    def _on_odom(self, msg: Odometry) -> None:
        p = msg.pose.pose.position
        yaw = quaternion_to_yaw(msg.pose.pose.orientation)
        self.current_pose = (p.x, p.y, yaw)

    def _show_status(self) -> None:
        if self.current_pose is None:
            return
        x, y, yaw = self.current_pose
        sys.stderr.write(
            f'\rOdom: x={x:+7.3f}  y={y:+7.3f}  yaw={math.degrees(yaw):+7.1f}°  '
            f'waypoints={len(self.waypoints)}     '
        )
        sys.stderr.flush()

    def _stdin_loop(self) -> None:
        sys.stderr.write(
            '\n'
            'Position the robot at the START. Then press Enter.\n'
            'At each stop, press Enter (or type a landmark id like "2") to record.\n'
            'Type "q" or press Ctrl-D when done to print the pattern.\n\n'
        )
        sys.stderr.flush()
        try:
            for raw in sys.stdin:
                line = raw.strip()
                if line.lower() in ('q', 'quit', 'exit'):
                    break
                if self.current_pose is None:
                    sys.stderr.write(
                        '\n  (No odometry yet — wait for /odom messages.)\n')
                    sys.stderr.flush()
                    continue
                self._record(line)
        except (EOFError, KeyboardInterrupt):
            pass
        self._print_final()
        rclpy.shutdown()

    # ----- waypoint logic ---------------------------------------------------
    def _record(self, label: str) -> None:
        idx = len(self.waypoints)
        x, y, yaw = self.current_pose
        self.waypoints.append({'x': x, 'y': y, 'yaw': yaw, 'label': label})

        if idx == 0:
            sys.stderr.write(
                f'\n[WP 0 = START] x={x:+.3f}  y={y:+.3f}  '
                f'yaw={math.degrees(yaw):+.1f}°\n'
            )
        else:
            self._print_steps_between(self.waypoints[idx - 1], self.waypoints[idx])
        sys.stderr.flush()

    def _steps_between(self, prev: dict, cur: dict) -> List[str]:
        dx, dy = cur['x'] - prev['x'], cur['y'] - prev['y']
        # Project displacement onto the previous heading (forward axis).
        forward = dx * math.cos(prev['yaw']) + dy * math.sin(prev['yaw'])
        # Heading change you needed to make at the previous waypoint.
        d_yaw = normalize_angle(cur['yaw'] - prev['yaw'])

        steps: List[str] = []
        if abs(d_yaw) > self.rot_thresh:
            steps.append(f'  - {{ type: rotate, angle: {d_yaw:+.4f} }}')
        if abs(forward) > self.fwd_thresh:
            kind = 'forward' if forward >= 0 else 'backward'
            steps.append(f'  - {{ type: {kind}, distance: {abs(forward):.3f} }}')
        if cur['label'].isdigit():
            lid = int(cur['label'])
            steps.append(
                f'  - {{ type: announce, landmark_id: {lid}, '
                f'dwell: {self.dwell:.1f}, timeout: {self.timeout:.1f} }}'
            )
        return steps

    def _print_steps_between(self, prev: dict, cur: dict) -> None:
        x, y, yaw = cur['x'], cur['y'], cur['yaw']
        dx, dy = cur['x'] - prev['x'], cur['y'] - prev['y']
        dist = math.hypot(dx, dy)
        forward = dx * math.cos(prev['yaw']) + dy * math.sin(prev['yaw'])
        d_yaw = normalize_angle(cur['yaw'] - prev['yaw'])

        sys.stderr.write(
            f'\n[WP {len(self.waypoints) - 1}] '
            f'x={x:+.3f}  y={y:+.3f}  yaw={math.degrees(yaw):+.1f}°  '
            f'(label={cur["label"] or "-"})\n'
            f'  Distance from previous: {dist:.3f} m\n'
            f'  Forward (projected):    {forward:+.3f} m\n'
            f'  Heading change:         {math.degrees(d_yaw):+.1f}° '
            f'({d_yaw:+.4f} rad)\n'
            f'  Suggested YAML:\n'
        )
        for line in self._steps_between(prev, cur):
            sys.stderr.write(f'  {line}\n')

    def _print_final(self) -> None:
        sys.stderr.write('\n\n========== pattern.yaml ==========\n')
        sys.stderr.write('steps:\n')
        for i in range(1, len(self.waypoints)):
            for line in self._steps_between(
                    self.waypoints[i - 1], self.waypoints[i]):
                sys.stderr.write(f'{line}\n')
        sys.stderr.write('==================================\n')
        sys.stderr.flush()


def main(args=None):
    rclpy.init(args=args)
    node = MeasurePattern()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
