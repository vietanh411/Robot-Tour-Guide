"""Pattern-based tour executor with target-position navigation + reactive
obstacle avoidance.

For each ``forward`` / ``backward`` step the node:
  1. Locks in a *target pose* (start_pose + distance * heading) when the
     step begins.
  2. Continually steers toward that target while monitoring the LiDAR.
     If something gets in the way, deflects toward the side with more
     clearance and slows proportionally to proximity.
  3. Considers the step "arrived" only when the robot is within
     ``target_tolerance_m`` of the target *position*.
  4. Realigns to the original heading before declaring the step done so
     subsequent ``rotate`` steps remain correct.

This means a robot that has to detour around a chair will still end up at
the planned destination facing the planned direction — no skipped steps.

Step types in pattern.yaml::

    forward    distance: <m>   [timeout: <s>]
    backward   distance: <m>   [timeout: <s>]
    rotate     angle: <rad>             # +ve = CCW / left
    announce   landmark_id: <int>  dwell: <s>  timeout: <s>
    wait       duration: <s>

Backward steps disable LiDAR avoidance (the LiDAR can't see behind),
but target-seeking still applies — the robot reverses toward the target
position. Make sure the path behind is clear.
"""

import json
import math
import os
import time
from typing import Optional, Tuple

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String


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


class PatternTour(Node):

    def __init__(self):
        super().__init__('pattern_tour')

        self.declare_parameter('pattern_file', '')
        self.declare_parameter('linear_speed', 0.15)
        self.declare_parameter('angular_speed', 0.4)
        self.declare_parameter('distance_tolerance_m', 0.05)
        self.declare_parameter('target_tolerance_m', 0.15)
        self.declare_parameter('angle_tolerance_rad', 0.10)
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('landmark_topic', '/landmarks/detected')
        self.declare_parameter('start_delay_s', 3.0)

        self.declare_parameter('avoid_obstacles', True)
        self.declare_parameter('safety_distance_m', 0.50)
        self.declare_parameter('hard_stop_distance_m', 0.20)
        self.declare_parameter('forward_arc_deg', 60.0)
        self.declare_parameter('side_arc_deg', 60.0)
        self.declare_parameter('avoidance_angular_speed', 0.5)
        self.declare_parameter('default_step_timeout_s', 30.0)

        path = self.get_parameter('pattern_file').value
        if not path:
            path = os.path.join(
                get_package_share_directory('robot_tour_guide'),
                'config', 'pattern.yaml',
            )
        with open(path, 'r') as f:
            data = yaml.safe_load(f)
        self.steps = list(data.get('steps', []))

        self.linear_speed = float(self.get_parameter('linear_speed').value)
        self.angular_speed = float(self.get_parameter('angular_speed').value)
        self.dist_tol = float(self.get_parameter('distance_tolerance_m').value)
        self.target_tol = float(self.get_parameter('target_tolerance_m').value)
        self.angle_tol = float(self.get_parameter('angle_tolerance_rad').value)
        self.start_delay = float(self.get_parameter('start_delay_s').value)
        self.avoid = bool(self.get_parameter('avoid_obstacles').value)
        self.safety_d = float(self.get_parameter('safety_distance_m').value)
        self.hard_stop_d = float(self.get_parameter('hard_stop_distance_m').value)
        self.forward_arc = math.radians(float(self.get_parameter('forward_arc_deg').value))
        self.side_arc = math.radians(float(self.get_parameter('side_arc_deg').value))
        self.avoid_omega = float(self.get_parameter('avoidance_angular_speed').value)
        self.default_step_timeout = float(self.get_parameter('default_step_timeout_s').value)

        self.current_step_idx = 0
        self.step_started = False
        self.step_start_pose: Optional[Tuple[float, float, float]] = None
        self.step_target: Optional[Tuple[float, float]] = None
        self.step_phase = 'travel'        # 'travel' or 'realign'
        self.step_start_time = 0.0
        self.recent_landmarks: dict = {}
        self.current_pose: Optional[Tuple[float, float, float]] = None
        self.latest_scan: Optional[LaserScan] = None
        self.startup_time = time.time()
        self.was_avoiding = False

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )

        self.create_subscription(
            Odometry, self.get_parameter('odom_topic').value, self.on_odom, 10)
        self.create_subscription(
            String, self.get_parameter('landmark_topic').value, self.on_landmark, 10)
        self.create_subscription(
            LaserScan, self.get_parameter('scan_topic').value, self.on_scan, sensor_qos)

        self.cmd_pub = self.create_publisher(
            TwistStamped, self.get_parameter('cmd_vel_topic').value, 10)
        self.narration_pub = self.create_publisher(String, '/tour/narration', 10)

        self.create_timer(0.05, self.tick)
        self.get_logger().info(
            f'Pattern tour ready ({len(self.steps)} steps, '
            f'avoidance {"on" if self.avoid else "off"}). '
            f'Starting in {self.start_delay:.1f} s.'
        )

    # ----- inputs -----------------------------------------------------------
    def on_odom(self, msg: Odometry) -> None:
        p = msg.pose.pose.position
        yaw = quaternion_to_yaw(msg.pose.pose.orientation)
        self.current_pose = (p.x, p.y, yaw)

    def on_landmark(self, msg: String) -> None:
        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        marker_id = int(payload.get('id', -1))
        if marker_id >= 0:
            self.recent_landmarks[marker_id] = time.time()

    def on_scan(self, msg: LaserScan) -> None:
        self.latest_scan = msg

    # ----- main loop --------------------------------------------------------
    def tick(self) -> None:
        if self.current_pose is None:
            return
        if time.time() - self.startup_time < self.start_delay:
            self._publish_zero()
            return
        if self.current_step_idx >= len(self.steps):
            self._publish_zero()
            return

        step = self.steps[self.current_step_idx]
        if not self.step_started:
            self.step_start_pose = self.current_pose
            self.step_start_time = time.time()
            self.step_started = True
            self.step_phase = 'travel'
            self.was_avoiding = False
            self._init_step_target(step)
            self.get_logger().info(
                f'Step {self.current_step_idx + 1}/{len(self.steps)}: {step}'
            )

        if self._execute_step(step):
            self.current_step_idx += 1
            self.step_started = False
            self.step_target = None
            self._publish_zero()
            if self.current_step_idx >= len(self.steps):
                self.get_logger().info('Pattern tour complete.')
                self.narration_pub.publish(
                    String(data='Tour complete. Thank you for visiting!'))

    def _init_step_target(self, step: dict) -> None:
        kind = step.get('type', '').lower()
        if kind not in ('forward', 'backward'):
            self.step_target = None
            return
        sx, sy, syaw = self.step_start_pose
        distance = float(step.get('distance', 0.0))
        sign = 1.0 if kind == 'forward' else -1.0
        tx = sx + sign * distance * math.cos(syaw)
        ty = sy + sign * distance * math.sin(syaw)
        self.step_target = (tx, ty)
        self.get_logger().info(
            f'  target pose: ({tx:+.3f}, {ty:+.3f}) heading {math.degrees(syaw):+.1f}°'
        )

    def _execute_step(self, step: dict) -> bool:
        kind = step.get('type', 'wait').lower()
        if kind in ('forward', 'backward'):
            return self._step_translate(step, kind)
        if kind == 'rotate':
            return self._step_rotate(step)
        if kind == 'announce':
            return self._step_announce(step)
        if kind == 'wait':
            return self._step_wait(step)
        self.get_logger().warn(f'Unknown step type: {kind!r}; skipping.')
        return True

    # ----- translate (with obstacle avoidance) ------------------------------
    def _step_translate(self, step: dict, kind: str) -> bool:
        timeout = float(step.get('timeout', self.default_step_timeout))
        if time.time() - self.step_start_time > timeout:
            self.get_logger().warn(
                f'Translate step timed out; advancing to next step.')
            return True

        sign = 1.0 if kind == 'forward' else -1.0
        cx, cy, cyaw = self.current_pose
        tx, ty = self.step_target
        sx, sy, syaw = self.step_start_pose

        if self.step_phase == 'travel':
            dist_to_target = math.hypot(tx - cx, ty - cy)
            if dist_to_target < self.target_tol:
                self.step_phase = 'realign'
                self.get_logger().info(
                    f'  reached target ({dist_to_target:.3f} m); realigning heading.')
                self._publish_zero()
                return False
            omega, linear_factor = self._steering_to_target(tx, ty, sign)
            self._publish_velocity(
                linear=sign * self.linear_speed * linear_factor,
                angular=omega,
            )
            return False

        # Phase: realign with original step heading.
        heading_err = normalize_angle(syaw - cyaw)
        if abs(heading_err) <= self.angle_tol:
            return True
        rot_sign = 1.0 if heading_err >= 0.0 else -1.0
        self._publish_velocity(
            linear=0.0,
            angular=rot_sign * min(self.angular_speed, max(0.15, abs(heading_err))),
        )
        return False

    def _steering_to_target(self, tx: float, ty: float,
                            sign: float) -> Tuple[float, float]:
        """Attractive steering toward (tx, ty) with reactive obstacle deflection.

        Returns (angular_velocity, linear_factor in [0, 1]).
        """
        cx, cy, cyaw = self.current_pose
        dx, dy = tx - cx, ty - cy
        desired_yaw = math.atan2(dy, dx)
        if sign < 0:
            desired_yaw = normalize_angle(desired_yaw + math.pi)
        heading_err = normalize_angle(desired_yaw - cyaw)
        attractive = max(-self.angular_speed,
                         min(self.angular_speed, 1.5 * heading_err))

        if not self.avoid or self.latest_scan is None or sign < 0:
            return attractive, 1.0

        scan = self.latest_scan
        n = len(scan.ranges)
        if n == 0:
            return attractive, 1.0

        forward_idx = int(round((-scan.angle_min) / scan.angle_increment))
        f_half = int(round((self.forward_arc / 2.0) / scan.angle_increment))
        s_half = int(round((self.side_arc / 2.0) / scan.angle_increment))

        forward_min = self._sector_min(scan, max(0, forward_idx - f_half),
                                             min(n, forward_idx + f_half + 1))

        if not math.isfinite(forward_min) or forward_min > self.safety_d:
            if self.was_avoiding:
                self.get_logger().info(
                    f'Forward clear ({forward_min:.2f} m); resuming target heading.')
                self.was_avoiding = False
            return attractive, 1.0

        # Obstacle ahead: deflect to clearer side and slow down.
        right_avg = self._sector_avg(scan, max(0, forward_idx - f_half - s_half),
                                           max(0, forward_idx - f_half))
        left_avg = self._sector_avg(scan, min(n, forward_idx + f_half),
                                          min(n, forward_idx + f_half + s_half))
        omega = self.avoid_omega if left_avg >= right_avg else -self.avoid_omega

        span = max(0.01, self.safety_d - self.hard_stop_d)
        linear_factor = max(0.0, min(1.0, (forward_min - self.hard_stop_d) / span))

        if not self.was_avoiding:
            side = 'left' if omega > 0 else 'right'
            self.get_logger().info(
                f'Obstacle at {forward_min:.2f} m; deflecting {side}.')
            self.was_avoiding = True

        return omega, linear_factor

    @staticmethod
    def _sector_min(scan: LaserScan, lo: int, hi: int) -> float:
        vals = [r for r in scan.ranges[lo:hi]
                if math.isfinite(r) and r > scan.range_min]
        return min(vals) if vals else float('inf')

    @staticmethod
    def _sector_avg(scan: LaserScan, lo: int, hi: int) -> float:
        vals = [r for r in scan.ranges[lo:hi]
                if math.isfinite(r) and r > scan.range_min]
        return (sum(vals) / len(vals)) if vals else 0.0

    # ----- rotate / announce / wait ----------------------------------------
    def _step_rotate(self, step: dict) -> bool:
        target = float(step.get('angle', 0.0))
        _, _, sy = self.step_start_pose
        _, _, cy = self.current_pose
        rotated = normalize_angle(cy - sy)
        sign = 1.0 if target >= 0.0 else -1.0
        if abs(rotated) >= abs(target) - self.angle_tol:
            return True
        self._publish_velocity(linear=0.0, angular=sign * self.angular_speed)
        return False

    def _step_announce(self, step: dict) -> bool:
        self._publish_zero()
        landmark_id = int(step.get('landmark_id', -1))
        dwell = float(step.get('dwell', 5.0))
        timeout = float(step.get('timeout', dwell + 5.0))
        elapsed = time.time() - self.step_start_time
        seen = (landmark_id in self.recent_landmarks
                and (time.time() - self.recent_landmarks[landmark_id]) < 2.0)
        if seen and elapsed >= dwell:
            return True
        if elapsed >= timeout:
            self.get_logger().warn(
                f'Marker {landmark_id} not confirmed within {timeout:.1f}s; '
                f'continuing anyway.')
            return True
        return False

    def _step_wait(self, step: dict) -> bool:
        self._publish_zero()
        return time.time() - self.step_start_time >= float(step.get('duration', 1.0))

    # ----- velocity --------------------------------------------------------
    def _publish_velocity(self, linear: float, angular: float) -> None:
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.twist.linear.x = float(linear)
        msg.twist.angular.z = float(angular)
        self.cmd_pub.publish(msg)

    def _publish_zero(self) -> None:
        self._publish_velocity(0.0, 0.0)


def main(args=None):
    rclpy.init(args=args)
    node = PatternTour()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node._publish_zero()
        except Exception:
            pass
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
