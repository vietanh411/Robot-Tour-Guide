"""Executive: tour-running state machine with failure classification.

States
------
  IDLE            -> waiting for a plan
  NAVIGATING      -> Nav2 goal active to next POI
  AT_POI          -> arrived (Nav2 success OR matching ArUco landmark in range)
  NARRATING       -> publishing description, dwelling for ``dwell_s``
  RECOVERING      -> classified failure handling (wait, replan, drop POI)
  RETURNING_HOME  -> driving back to the recorded start pose
  DONE            -> tour finished

The deliberative novelty is :py:meth:`classify_failure`, which inspects the
world-model snapshot at the moment Nav2 reports failure and chooses an
appropriate response instead of blindly retrying.
"""

import json
import math
import os
import time
from enum import Enum
from typing import Optional, Tuple

import rclpy
import yaml
from action_msgs.msg import GoalStatus
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node
from std_msgs.msg import String


class State(Enum):
    IDLE = 'IDLE'
    NAVIGATING = 'NAVIGATING'
    AT_POI = 'AT_POI'
    NARRATING = 'NARRATING'
    RECOVERING = 'RECOVERING'
    RETURNING_HOME = 'RETURNING_HOME'
    DONE = 'DONE'


class FailureClass(Enum):
    PERSON_BLOCKING = 'person_blocking'
    FURNITURE_MOVED = 'furniture_moved'
    DOOR_CLOSED = 'door_closed'
    LOCALIZATION_LOST = 'localization_lost'
    POI_UNREACHABLE = 'poi_unreachable'
    UNKNOWN = 'unknown'


class Executive(Node):

    def __init__(self):
        super().__init__('executive')

        self.declare_parameter('landmarks_file', '')
        self.declare_parameter('arrival_radius_m', 0.50)
        self.declare_parameter('landmark_arrival_distance_m', 1.50)
        self.declare_parameter('landmark_dwell_s', 1.0)
        self.declare_parameter('narration_dwell_s', 6.0)
        self.declare_parameter('person_blocking_distance_m', 1.5)
        self.declare_parameter('person_block_wait_s', 10.0)
        self.declare_parameter('max_retries_per_poi', 2)
        self.declare_parameter('frame_id', 'map')

        self.landmarks = self._load_landmarks(self.get_parameter('landmarks_file').value)

        self.state = State.IDLE
        self.plan: list = []
        self.current_idx: int = 0
        self.current_pose = (0.0, 0.0)
        # First AMCL pose received; the robot drives back here at end of tour.
        self.home_pose: Optional[Tuple[float, float, float]] = None
        self.world: dict = {'objects': [], 'landmarks': [], 'counts': {}}
        self.recent_landmarks: dict = {}
        self.retries_left = int(self.get_parameter('max_retries_per_poi').value)
        self.last_state_change = time.time()
        self.recovery_until = 0.0
        self.narration_until = 0.0
        self.announced_landmarks: set = set()
        self._goal_handle = None

        self.create_subscription(String, '/tour/current_plan', self.on_plan, 5)
        self.create_subscription(String, '/world/state', self.on_world, 10)
        self.create_subscription(String, '/landmarks/detected', self.on_landmark, 10)
        self.create_subscription(
            PoseWithCovarianceStamped, '/amcl_pose', self.on_pose, 5,
        )

        self.narration_pub = self.create_publisher(String, '/tour/narration', 10)
        self.replan_pub = self.create_publisher(String, '/tour/replan_request', 5)
        self.drop_pub = self.create_publisher(String, '/tour/drop_poi', 5)
        self.status_pub = self.create_publisher(String, '/tour/status', 5)

        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        self.create_timer(0.25, self.tick)
        self.get_logger().info('Executive ready.')

    # ----- IO ---------------------------------------------------------------
    def _load_landmarks(self, path: str) -> dict:
        if not path:
            path = os.path.join(
                get_package_share_directory('robot_tour_guide'),
                'config', 'landmarks.yaml',
            )
        try:
            with open(path, 'r') as f:
                data = yaml.safe_load(f) or {}
            return {int(k): v for k, v in (data.get('landmarks') or {}).items()}
        except FileNotFoundError:
            self.get_logger().warn(f'landmarks file not found: {path}')
            return {}

    def on_plan(self, msg: String):
        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        new_plan = payload.get('plan', [])
        if [p['id'] for p in new_plan] != [p['id'] for p in self.plan]:
            self.plan = new_plan
            self.current_idx = 0
            self.retries_left = int(self.get_parameter('max_retries_per_poi').value)
            self.announced_landmarks.clear()
            if self.state == State.IDLE and self.plan:
                self._start_navigation()

    def on_world(self, msg: String):
        try:
            self.world = json.loads(msg.data)
        except json.JSONDecodeError:
            pass

    def on_landmark(self, msg: String):
        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        landmark_id = int(payload['id'])
        self.recent_landmarks[landmark_id] = {
            **payload,
            'observed_at': time.time(),
        }

    def on_pose(self, msg: PoseWithCovarianceStamped):
        p = msg.pose.pose.position
        self.current_pose = (p.x, p.y)
        if self.home_pose is None:
            q = msg.pose.pose.orientation
            yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                             1.0 - 2.0 * (q.y * q.y + q.z * q.z))
            self.home_pose = (p.x, p.y, yaw)
            self.get_logger().info(
                f'Captured home pose: x={p.x:+.2f} y={p.y:+.2f} '
                f'yaw={math.degrees(yaw):+.1f} deg. '
                'Robot will return here when the tour ends.'
            )

    # ----- main loop --------------------------------------------------------
    def tick(self):
        self._publish_status()
        if self.state == State.IDLE:
            if self.plan:
                self._start_navigation()
            return

        if self.state == State.NAVIGATING:
            self._check_landmark_arrival()
            return

        if self.state == State.AT_POI:
            self._enter_narration()
            return

        if self.state == State.NARRATING:
            if time.time() >= self.narration_until:
                self._advance_to_next_poi()
            return

        if self.state == State.RECOVERING:
            if time.time() >= self.recovery_until:
                self._post_recovery_action()
            return

    def _publish_status(self):
        self.status_pub.publish(String(data=json.dumps({
            'state': self.state.value,
            'current_idx': self.current_idx,
            'plan_len': len(self.plan),
            'current_poi': self.plan[self.current_idx]['name'] if self.plan and self.current_idx < len(self.plan) else None,
            'retries_left': self.retries_left,
        })))

    # ----- transitions ------------------------------------------------------
    def _start_navigation(self):
        if self.current_idx >= len(self.plan):
            self._finish_tour()
            return

        if not self.nav_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().warn('Nav2 action server unavailable; retrying...')
            return

        poi = self.plan[self.current_idx]
        goal = NavigateToPose.Goal()
        goal.pose = self._build_pose(
            float(poi['x']), float(poi['y']), float(poi.get('yaw', 0.0)),
        )
        future = self.nav_client.send_goal_async(goal)
        future.add_done_callback(self._on_goal_response)
        self._transition(State.NAVIGATING)
        self.get_logger().info(f"Navigating to POI '{poi['name']}' (id={poi['id']}).")

    def _build_pose(self, x: float, y: float, yaw: float) -> PoseStamped:
        ps = PoseStamped()
        ps.header.frame_id = self.get_parameter('frame_id').value
        ps.header.stamp = self.get_clock().now().to_msg()
        ps.pose.position.x = float(x)
        ps.pose.position.y = float(y)
        ps.pose.orientation.z = math.sin(yaw / 2.0)
        ps.pose.orientation.w = math.cos(yaw / 2.0)
        return ps

    def _on_goal_response(self, future):
        handle = future.result()
        if handle is None or not handle.accepted:
            self.get_logger().warn('Nav2 rejected the goal.')
            self._begin_recovery(FailureClass.UNKNOWN)
            return
        self._goal_handle = handle
        handle.get_result_async().add_done_callback(self._on_goal_result)

    def _on_goal_result(self, future):
        result = future.result()
        if result is None:
            return
        if self.state != State.NAVIGATING:
            # We already moved on (e.g. landmark-confirmed early arrival
            # cancelled this goal ourselves). The cancellation result is
            # stale -- treating it as a failure would trigger spurious
            # recoveries.
            self.get_logger().info(
                f'Ignoring stale Nav2 result '
                f'(state={self.state.value}, status={result.status}).'
            )
            return
        status = result.status
        if status == GoalStatus.STATUS_SUCCEEDED:
            self._transition(State.AT_POI)
        else:
            cls = self.classify_failure()
            self.get_logger().warn(f'Nav2 failed (status={status}); class={cls.value}')
            self._begin_recovery(cls)

    def _check_landmark_arrival(self):
        if not self.plan or self.current_idx >= len(self.plan):
            return
        target_id = self.plan[self.current_idx].get('landmark_id')
        if target_id is None:
            return
        max_d = float(self.get_parameter('landmark_arrival_distance_m').value)
        seen = self.recent_landmarks.get(target_id)
        if seen and seen['distance_m'] <= max_d and (time.time() - seen['observed_at']) < 1.0:
            self.get_logger().info(
                f"Landmark id={target_id} confirmed at {seen['distance_m']:.2f} m; arriving early."
            )
            if self._goal_handle is not None:
                try:
                    self._goal_handle.cancel_goal_async()
                except Exception:
                    pass
            self._transition(State.AT_POI)

    def _enter_narration(self):
        poi = self.plan[self.current_idx]
        landmark_id = poi.get('landmark_id')
        text = poi.get('description', f"This is {poi['name']}.")
        # Override with the marker DB if a landmark was confirmed.
        if landmark_id is not None and landmark_id in self.landmarks:
            entry = self.landmarks[landmark_id]
            text = f"{entry.get('name', poi['name'])}. {entry.get('description', '')}"
            self.announced_landmarks.add(landmark_id)
        self._narrate(text)
        self.narration_until = time.time() + float(self.get_parameter('narration_dwell_s').value)
        self._transition(State.NARRATING)

    def _advance_to_next_poi(self):
        self.current_idx += 1
        self.retries_left = int(self.get_parameter('max_retries_per_poi').value)
        if self.current_idx >= len(self.plan):
            self._finish_tour()
            return
        self._start_navigation()

    # ----- end-of-tour return-to-start --------------------------------------
    def _finish_tour(self):
        """Drive back to the recorded home pose, then transition to DONE."""
        self._narrate("Tour complete. Returning to the starting point.")
        if self.home_pose is None:
            self.get_logger().warn(
                'No home pose was ever captured (AMCL silent?); '
                'finishing in place.')
            self._transition(State.DONE)
            return
        if not self.nav_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().warn(
                'Nav2 unavailable for return-home; finishing in place.')
            self._transition(State.DONE)
            return
        hx, hy, hyaw = self.home_pose
        goal = NavigateToPose.Goal()
        goal.pose = self._build_pose(hx, hy, hyaw)
        future = self.nav_client.send_goal_async(goal)
        future.add_done_callback(self._on_home_goal_response)
        self._transition(State.RETURNING_HOME)
        self.get_logger().info(
            f'Returning home to ({hx:+.2f}, {hy:+.2f}, '
            f'{math.degrees(hyaw):+.1f} deg).'
        )

    def _on_home_goal_response(self, future):
        handle = future.result()
        if handle is None or not handle.accepted:
            self.get_logger().warn('Nav2 rejected return-home goal; finishing.')
            self._narrate("Thank you for visiting!")
            self._transition(State.DONE)
            return
        self._goal_handle = handle
        handle.get_result_async().add_done_callback(self._on_home_goal_result)

    def _on_home_goal_result(self, future):
        result = future.result()
        if result is not None and result.status == GoalStatus.STATUS_SUCCEEDED:
            self._narrate(
                "I have returned to the starting point. Thank you for visiting!")
        else:
            status = result.status if result is not None else 'unknown'
            self.get_logger().warn(
                f'Return-home Nav2 ended with status={status}; '
                f'finishing in place.')
            self._narrate(
                "I had trouble getting back to the start, but the tour is "
                "complete. Thank you for visiting!")
        self._transition(State.DONE)

    # ----- failure handling -------------------------------------------------
    def classify_failure(self) -> FailureClass:
        objects = self.world.get('objects', [])
        person_close = any(
            o['class'] in ('person', 'person_candidate')
            and math.hypot(o.get('x', 1e3), o.get('y', 1e3))
                < float(self.get_parameter('person_blocking_distance_m').value)
            for o in objects
        )
        if person_close:
            return FailureClass.PERSON_BLOCKING

        # Localisation lost -> AMCL pose covariance would tell us, but we proxy:
        # if we have no /amcl_pose in the world snapshot for >5 s assume bad.
        # (Kept conservative for now.)

        # New furniture-class detections close to current path?
        new_furniture = any(
            o['class'] in ('furniture', 'furniture_candidate')
            and (time.time() - o.get('first_seen', 0.0)) < 8.0
            for o in objects
        )
        if new_furniture:
            return FailureClass.FURNITURE_MOVED

        # No door_open within +/- 60 deg of forward where we expected one?
        forward_door = any(
            o['class'] == 'door_open' and abs(math.atan2(o.get('y', 0.0), o.get('x', 1.0))) < 1.05
            for o in objects
        )
        if not forward_door and self.retries_left == 0:
            return FailureClass.DOOR_CLOSED

        if self.retries_left <= 0:
            return FailureClass.POI_UNREACHABLE

        return FailureClass.UNKNOWN

    def _begin_recovery(self, cls: FailureClass):
        if cls == FailureClass.PERSON_BLOCKING:
            wait_s = float(self.get_parameter('person_block_wait_s').value)
            self._narrate("Excuse me, may I please come through?")
            self.recovery_until = time.time() + wait_s
        elif cls == FailureClass.FURNITURE_MOVED:
            self._narrate("It looks like the furniture has moved. Let me find another way.")
            self._request_replan(reason='furniture_moved')
            self.recovery_until = time.time() + 1.0
        elif cls == FailureClass.DOOR_CLOSED:
            self._narrate("This door appears to be closed. Skipping this stop.")
            self._drop_current_poi()
            self.recovery_until = time.time() + 1.0
        elif cls == FailureClass.LOCALIZATION_LOST:
            self._narrate("One moment while I get my bearings.")
            self.recovery_until = time.time() + 5.0
        elif cls == FailureClass.POI_UNREACHABLE:
            self._narrate("I am unable to reach this stop. Moving on.")
            self._drop_current_poi()
            self.recovery_until = time.time() + 1.0
        else:
            self.recovery_until = time.time() + 2.0
        self._transition(State.RECOVERING)
        self.retries_left = max(0, self.retries_left - 1)

    def _post_recovery_action(self):
        # In every recovery case the right next move is to (re)dispatch
        # navigation. If the POI was dropped, on_plan has already swapped the
        # plan and reset current_idx; if a replan was requested, the planner's
        # next publish does the same. Either way, _start_navigation will route
        # to the correct next POI -- or call _finish_tour if none remain.
        self._start_navigation()

    def _request_replan(self, reason: str):
        self.replan_pub.publish(String(data=json.dumps({
            'start_x': self.current_pose[0],
            'start_y': self.current_pose[1],
            'reason': reason,
        })))

    def _drop_current_poi(self):
        if self.current_idx >= len(self.plan):
            return
        poi_id = self.plan[self.current_idx]['id']
        self.drop_pub.publish(String(data=str(poi_id)))

    # ----- helpers ----------------------------------------------------------
    def _narrate(self, text: str):
        self.get_logger().info(f'[narrate] {text}')
        self.narration_pub.publish(String(data=text))

    def _transition(self, new_state: State):
        if new_state == self.state:
            return
        self.get_logger().info(f'State {self.state.value} -> {new_state.value}')
        self.state = new_state
        self.last_state_change = time.time()


def main(args=None):
    rclpy.init(args=args)
    node = Executive()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
