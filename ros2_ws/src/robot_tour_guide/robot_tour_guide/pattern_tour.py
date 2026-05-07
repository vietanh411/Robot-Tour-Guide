"""Pattern-based open-loop tour executor.

A simpler alternative to the Nav2-based tour: drive a predefined sequence of
``forward / backward / rotate / announce / wait`` steps loaded from a YAML
file. Progress is tracked from odometry only, so no map or AMCL is required.

This trades robustness (will not avoid obstacles, will drift over long
sequences) for setup simplicity. Intended for short, controlled demo runs
in a clear space.

Step types in pattern.yaml::

    forward    distance: <metres>
    backward   distance: <metres>
    rotate     angle: <radians; +ve = CCW / left turn>
    announce   landmark_id: <int>  dwell: <s>  timeout: <s>
    wait       duration: <s>
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
        self.declare_parameter('angle_tolerance_rad', 0.05)
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('landmark_topic', '/landmarks/detected')
        self.declare_parameter('start_delay_s', 3.0)

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
        self.angle_tol = float(self.get_parameter('angle_tolerance_rad').value)
        self.start_delay = float(self.get_parameter('start_delay_s').value)

        self.current_step_idx = 0
        self.step_started = False
        self.step_start_pose: Optional[Tuple[float, float, float]] = None
        self.step_start_time = 0.0
        self.recent_landmarks: dict = {}      # marker_id -> last seen wall-clock
        self.current_pose: Optional[Tuple[float, float, float]] = None
        self.startup_time = time.time()

        self.create_subscription(
            Odometry,
            self.get_parameter('odom_topic').value,
            self.on_odom, 10,
        )
        self.create_subscription(
            String,
            self.get_parameter('landmark_topic').value,
            self.on_landmark, 10,
        )

        self.cmd_pub = self.create_publisher(
            TwistStamped,
            self.get_parameter('cmd_vel_topic').value,
            10,
        )
        self.narration_pub = self.create_publisher(String, '/tour/narration', 10)

        self.create_timer(0.05, self.tick)
        self.get_logger().info(
            f'Pattern tour ready with {len(self.steps)} steps. '
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

    # ----- main loop --------------------------------------------------------
    def tick(self) -> None:
        if self.current_pose is None:
            return  # waiting for first odom
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
            self.get_logger().info(
                f'Step {self.current_step_idx + 1}/{len(self.steps)}: {step}'
            )

        if self._execute_step(step):
            self.current_step_idx += 1
            self.step_started = False
            self._publish_zero()
            if self.current_step_idx >= len(self.steps):
                self.get_logger().info('Pattern tour complete.')
                self.narration_pub.publish(
                    String(data='Tour complete. Thank you for visiting!'))

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

    # ----- per-step handlers ------------------------------------------------
    def _step_translate(self, step: dict, kind: str) -> bool:
        distance = float(step.get('distance', 0.0))
        sign = 1.0 if kind == 'forward' else -1.0
        sx, sy, _ = self.step_start_pose
        cx, cy, _ = self.current_pose
        traveled = math.hypot(cx - sx, cy - sy)
        if traveled >= max(0.0, distance - self.dist_tol):
            return True
        self._publish_velocity(linear=sign * self.linear_speed, angular=0.0)
        return False

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
                f'continuing anyway.'
            )
            return True
        return False

    def _step_wait(self, step: dict) -> bool:
        self._publish_zero()
        return time.time() - self.step_start_time >= float(step.get('duration', 1.0))

    # ----- velocity publishing ---------------------------------------------
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
