#!/usr/bin/env python3

"""Plan and execute one guarded Gazebo navigation goal to the incubator."""

import json
import math
import time

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from lifecycle_msgs.msg import State
from lifecycle_msgs.srv import GetState
from nav2_msgs.action import ComputePathToPose, NavigateToPose
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

from .square_localization import normalize_angle


def segment_intersects_rectangle(start, goal, bounds):
    """Return whether a line segment intersects an axis-aligned rectangle."""
    minimum_x, maximum_x, minimum_y, maximum_y = bounds
    lower = (minimum_x, minimum_y)
    upper = (maximum_x, maximum_y)
    start = (float(start[0]), float(start[1]))
    goal = (float(goal[0]), float(goal[1]))
    minimum_fraction = 0.0
    maximum_fraction = 1.0
    for axis in range(2):
        delta = goal[axis] - start[axis]
        if abs(delta) <= 1.0e-12:
            if start[axis] < lower[axis] or start[axis] > upper[axis]:
                return False
            axis_minimum = 0.0
            axis_maximum = 1.0
        else:
            first = (lower[axis] - start[axis]) / delta
            second = (upper[axis] - start[axis]) / delta
            axis_minimum = min(first, second)
            axis_maximum = max(first, second)
        minimum_fraction = max(minimum_fraction, axis_minimum)
        maximum_fraction = min(maximum_fraction, axis_maximum)
        if minimum_fraction > maximum_fraction:
            return False
    return True


def point_to_segment_distance(point, start, goal):
    """Return the planar distance from a point to a finite segment."""
    point_x, point_y = (float(value) for value in point)
    start_x, start_y = (float(value) for value in start)
    goal_x, goal_y = (float(value) for value in goal)
    delta_x = goal_x - start_x
    delta_y = goal_y - start_y
    squared_length = delta_x * delta_x + delta_y * delta_y
    if squared_length <= 1.0e-12:
        return math.hypot(point_x - start_x, point_y - start_y)
    fraction = (
        (point_x - start_x) * delta_x
        + (point_y - start_y) * delta_y
    ) / squared_length
    fraction = min(1.0, max(0.0, fraction))
    closest_x = start_x + fraction * delta_x
    closest_y = start_y + fraction * delta_y
    return math.hypot(point_x - closest_x, point_y - closest_y)


def locomotion_hold_ready(
        payload, planar_speed, position_tolerance, yaw_tolerance):
    """Accept either the legacy static hold or Unitree station keeping."""
    if payload.get('state') == 'CAPTURED_HOLD':
        return bool(
            payload.get('capture_latched') is True
            and planar_speed < 0.03)
    if payload.get('state') != 'STATION_KEEPING_GAIT':
        return False
    requested_command = payload.get('command', [])
    station_error = payload.get('station_error')
    station_yaw_error = payload.get('station_yaw_error')
    return bool(
        payload.get('zero_command_behavior')
        == 'recurrent_station_keeping'
        and len(requested_command) == 3
        and all(float(value) == 0.0 for value in requested_command)
        and station_error is not None
        and math.isfinite(float(station_error))
        and float(station_error) <= position_tolerance
        and station_yaw_error is not None
        and math.isfinite(float(station_yaw_error))
        and abs(float(station_yaw_error)) <= yaw_tolerance)


def locomotion_support_ready(payload, contact_forces):
    """Require static double support or dynamic gait weight support."""
    if len(contact_forces) != 2:
        return False
    try:
        forces = tuple(float(value) for value in contact_forces)
    except (TypeError, ValueError):
        return False
    if not all(math.isfinite(value) and value >= 0.0 for value in forces):
        return False
    if payload.get('state') == 'STATION_KEEPING_GAIT':
        return sum(forces) >= 300.0 and max(forces) > 0.0
    return all(force > 0.0 for force in forces)


def sustained_hold_sample_ready(
        payload, localization_valid, localized_goal_error,
        physical_goal_error, localized_yaw_error, planar_speed,
        position_tolerance, yaw_tolerance, up_z, height, contact_forces,
        minimum_up_z=0.98, minimum_height=0.95):
    """Require every sample in the long-hold envelope to remain valid."""
    numeric_values = (
        localized_goal_error, physical_goal_error, localized_yaw_error,
        up_z, height, minimum_up_z, minimum_height)
    if any(value is None for value in numeric_values):
        return False
    try:
        numeric_values = tuple(float(value) for value in numeric_values)
    except (TypeError, ValueError):
        return False
    if not all(math.isfinite(value) for value in numeric_values):
        return False
    (
        localized_goal_error, physical_goal_error, localized_yaw_error,
        up_z, height, minimum_up_z, minimum_height,
    ) = numeric_values
    return bool(
        localization_valid
        and localized_goal_error <= position_tolerance
        and physical_goal_error <= position_tolerance
        and abs(localized_yaw_error) <= yaw_tolerance
        and up_z >= minimum_up_z
        and height >= minimum_height
        and locomotion_hold_ready(
            payload, planar_speed, position_tolerance, yaw_tolerance)
        and locomotion_support_ready(payload, contact_forces))


def validate_incubator_staging_contract(
        startup_pose, goal_pose, workcell_bounds,
        robot_half_extents=(0.45, 0.40), clearance=0.05,
        maximum_goal_distance=0.20, wall_half_extent=2.50):
    """Validate the bounded axis-aligned startup-to-staging motion."""
    startup_pose = tuple(float(value) for value in startup_pose)
    goal_pose = tuple(float(value) for value in goal_pose)
    workcell_bounds = tuple(float(value) for value in workcell_bounds)
    robot_half_extents = tuple(float(value) for value in robot_half_extents)
    values = startup_pose + goal_pose + workcell_bounds + robot_half_extents + (
        float(clearance), float(maximum_goal_distance),
        float(wall_half_extent))
    if (
            len(startup_pose) != 3
            or len(goal_pose) != 3
            or len(workcell_bounds) != 4
            or len(robot_half_extents) != 2
            or not all(math.isfinite(value) for value in values)):
        raise ValueError('navigation contract values must be finite')
    minimum_x, maximum_x, minimum_y, maximum_y = workcell_bounds
    half_length, half_width = robot_half_extents
    if (
            minimum_x >= maximum_x
            or minimum_y >= maximum_y
            or half_length <= 0.0
            or half_width <= 0.0
            or clearance < 0.0
            or maximum_goal_distance <= 0.0
            or wall_half_extent <= 0.0):
        raise ValueError('navigation contract dimensions are invalid')
    if (
            abs(normalize_angle(startup_pose[2])) > math.radians(1.0)
            or abs(normalize_angle(goal_pose[2])) > math.radians(1.0)):
        raise ValueError('incubator staging contract requires zero yaw')
    goal_distance = math.hypot(
        goal_pose[0] - startup_pose[0],
        goal_pose[1] - startup_pose[1],
    )
    if goal_distance < 0.05 or goal_distance > maximum_goal_distance:
        raise ValueError('goal distance is outside the bounded motion envelope')
    for pose in (startup_pose, goal_pose):
        if (
                abs(pose[0]) + half_length + clearance >= wall_half_extent
                or abs(pose[1]) + half_width + clearance >= wall_half_extent):
            raise ValueError('robot footprint leaves the square-room interior')
    expanded_bounds = (
        minimum_x - half_length - clearance,
        maximum_x + half_length + clearance,
        minimum_y - half_width - clearance,
        maximum_y + half_width + clearance,
    )
    if segment_intersects_rectangle(
            startup_pose[:2], goal_pose[:2], expanded_bounds):
        raise ValueError('startup-to-goal segment enters the workcell keepout')
    return goal_distance, expanded_bounds


def _yaw_from_quaternion(quaternion):
    sine = 2.0 * (
        quaternion.w * quaternion.z
        + quaternion.x * quaternion.y)
    cosine = 1.0 - 2.0 * (
        quaternion.y * quaternion.y
        + quaternion.z * quaternion.z)
    return math.atan2(sine, cosine)


class IncubatorAutoNavigator(Node):
    """Send one fail-closed staging goal after all health gates pass."""

    def __init__(self):
        super().__init__('h1_2_incubator_auto_navigator')
        self.declare_parameter('enabled', False)
        self.declare_parameter('movement_authorized', False)
        self.declare_parameter(
            'locomotion_status_topic', '/h1_2_navigation/v12/status')
        self.declare_parameter('startup_x', -0.60)
        self.declare_parameter('startup_y', 0.0)
        self.declare_parameter('startup_yaw', 0.0)
        self.declare_parameter('goal_x', -0.45)
        self.declare_parameter('goal_y', 0.0)
        self.declare_parameter('goal_yaw', 0.0)
        self.declare_parameter('workcell_minimum_x', 0.20)
        self.declare_parameter('workcell_maximum_x', 1.00)
        self.declare_parameter('workcell_minimum_y', -0.60)
        self.declare_parameter('workcell_maximum_y', 0.60)
        self.declare_parameter('startup_position_tolerance', 0.08)
        self.declare_parameter('startup_yaw_tolerance', 0.10)
        self.declare_parameter('goal_position_tolerance', 0.08)
        self.declare_parameter('goal_yaw_tolerance', 0.20)
        self.declare_parameter('physical_corridor_tolerance', 0.12)
        self.declare_parameter('maximum_physical_displacement', 0.28)
        self.declare_parameter('maximum_planned_path_length', 0.30)
        self.declare_parameter('ready_sample_count', 10)
        self.declare_parameter('localization_sample_count', 5)
        self.declare_parameter('invalid_localization_limit', 3)
        self.declare_parameter('stable_sample_count', 50)
        self.declare_parameter('ready_timeout', 60.0)
        self.declare_parameter('navigation_timeout', 120.0)
        self.declare_parameter('stable_timeout', 25.0)
        self.declare_parameter('long_hold_duration', 30.0)
        self.declare_parameter('hold_status_timeout', 0.5)
        self.declare_parameter('long_hold_minimum_up_z', 0.98)
        self.declare_parameter('long_hold_minimum_height', 0.95)

        self.enabled = bool(self.get_parameter('enabled').value)
        self.movement_authorized = bool(self.get_parameter(
            'movement_authorized').value)
        self.locomotion_status_topic = str(self.get_parameter(
            'locomotion_status_topic').value)
        self.startup_pose = tuple(float(self.get_parameter(name).value)
                                  for name in (
                                      'startup_x', 'startup_y', 'startup_yaw'))
        self.goal_pose = tuple(float(self.get_parameter(name).value)
                               for name in (
                                   'goal_x', 'goal_y', 'goal_yaw'))
        self.workcell_bounds = tuple(float(self.get_parameter(name).value)
                                     for name in (
                                         'workcell_minimum_x',
                                         'workcell_maximum_x',
                                         'workcell_minimum_y',
                                         'workcell_maximum_y'))
        self.startup_position_tolerance = float(self.get_parameter(
            'startup_position_tolerance').value)
        self.startup_yaw_tolerance = float(self.get_parameter(
            'startup_yaw_tolerance').value)
        self.goal_position_tolerance = float(self.get_parameter(
            'goal_position_tolerance').value)
        self.goal_yaw_tolerance = float(self.get_parameter(
            'goal_yaw_tolerance').value)
        self.physical_corridor_tolerance = float(self.get_parameter(
            'physical_corridor_tolerance').value)
        self.maximum_physical_displacement = float(self.get_parameter(
            'maximum_physical_displacement').value)
        self.maximum_planned_path_length = float(self.get_parameter(
            'maximum_planned_path_length').value)
        self.ready_sample_target = int(self.get_parameter(
            'ready_sample_count').value)
        self.localization_sample_target = int(self.get_parameter(
            'localization_sample_count').value)
        self.invalid_localization_limit = int(self.get_parameter(
            'invalid_localization_limit').value)
        self.stable_sample_target = int(self.get_parameter(
            'stable_sample_count').value)
        self.ready_timeout = float(self.get_parameter('ready_timeout').value)
        self.navigation_timeout = float(self.get_parameter(
            'navigation_timeout').value)
        self.stable_timeout = float(self.get_parameter('stable_timeout').value)
        self.long_hold_duration = float(self.get_parameter(
            'long_hold_duration').value)
        self.hold_status_timeout = float(self.get_parameter(
            'hold_status_timeout').value)
        self.long_hold_minimum_up_z = float(self.get_parameter(
            'long_hold_minimum_up_z').value)
        self.long_hold_minimum_height = float(self.get_parameter(
            'long_hold_minimum_height').value)
        self.goal_distance, self.expanded_workcell_bounds = (
            validate_incubator_staging_contract(
                self.startup_pose,
                self.goal_pose,
                self.workcell_bounds,
            ))
        if min(
                self.startup_position_tolerance,
                self.startup_yaw_tolerance,
                self.goal_position_tolerance,
                self.goal_yaw_tolerance,
                self.physical_corridor_tolerance,
                self.maximum_physical_displacement,
                self.maximum_planned_path_length,
                self.ready_sample_target,
                self.localization_sample_target,
                self.invalid_localization_limit,
                self.stable_sample_target,
                self.ready_timeout,
                self.navigation_timeout,
                self.stable_timeout,
                self.long_hold_duration,
                self.hold_status_timeout,
                self.long_hold_minimum_up_z,
                self.long_hold_minimum_height) <= 0:
            raise ValueError('auto-navigation health gates must be positive')

        status_qos = QoSProfile(depth=1)
        status_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        status_qos.reliability = ReliabilityPolicy.RELIABLE
        self.status_publisher = self.create_publisher(
            String, '/h1_2_navigation/incubator_navigation/status',
            status_qos)
        self.plan_client = ActionClient(
            self, ComputePathToPose, '/compute_path_to_pose')
        self.navigation_client = ActionClient(
            self, NavigateToPose, '/navigate_to_pose')
        self.lifecycle_state_names = (
            'amcl',
            'planner_server',
            'controller_server',
            'behavior_server',
            'bt_navigator',
        )
        self.lifecycle_clients = {
            name: self.create_client(GetState, f'/{name}/get_state')
            for name in self.lifecycle_state_names
        }
        self.lifecycle_futures = {
            name: None for name in self.lifecycle_state_names
        }
        self.lifecycle_states = {
            name: State.PRIMARY_STATE_UNKNOWN
            for name in self.lifecycle_state_names
        }
        latest_qos = QoSProfile(depth=1)
        latest_qos.reliability = ReliabilityPolicy.BEST_EFFORT
        self.create_subscription(
            String, self.locomotion_status_topic,
            self._on_v12, latest_qos)
        self.create_subscription(
            String, '/mid360_localization/status',
            self._on_localization, latest_qos)
        self.create_subscription(
            PoseWithCovarianceStamped, '/mid360_localization/pose',
            self._on_pose, latest_qos)
        self.timer = self.create_timer(0.1, self._on_timer)

        self.started_at = time.monotonic()
        self.state_started_at = self.started_at
        self.state = 'WAITING_FOR_HEALTH'
        self.reason = 'startup'
        self.latest_pose = None
        self.latest_root_position = None
        self.latest_up_z = None
        self.latest_height = None
        self.latest_localization_valid = False
        self.v12_ready_samples = 0
        self.localization_ready_samples = 0
        self.invalid_localization_samples = 0
        self.stable_samples = 0
        self.last_locomotion_status_at = None
        self.plan_goal_handle = None
        self.navigation_goal_handle = None
        self.minimum_distance_remaining = math.inf
        self.planned_path_length = None
        self.last_status_published_at = -math.inf
        if not self.enabled:
            self._fail('AUTO_NAVIGATION_NOT_ENABLED')
        elif not self.movement_authorized:
            self._fail('MOVEMENT_NOT_AUTHORIZED')
        else:
            self._publish_status()

    def _elapsed(self):
        return time.monotonic() - self.started_at

    def _transition(self, state, reason):
        self.state = state
        self.reason = reason
        self.state_started_at = time.monotonic()
        self._publish_status()
        message = f'INCUBATOR_AUTO_NAVIGATION={state}:{reason}'
        if state == 'FAILED':
            self.get_logger().error(message)
        else:
            self.get_logger().info(message)

    def _publish_status(self):
        payload = {
            'state': self.state,
            'reason': self.reason,
            'elapsed_wall': self._elapsed(),
            'gazebo_only': True,
            'startup_pose': list(self.startup_pose),
            'goal_pose': list(self.goal_pose),
            'goal_distance_m': self.goal_distance,
            'expanded_workcell_bounds': list(
                self.expanded_workcell_bounds),
            'latest_pose': (
                list(self.latest_pose) if self.latest_pose is not None
                else None),
            'latest_root_position': (
                list(self.latest_root_position)
                if self.latest_root_position is not None else None),
            'latest_up_z': self.latest_up_z,
            'latest_height': self.latest_height,
            'localized_goal_error_m': self._localized_goal_error(),
            'physical_goal_error_m': self._physical_goal_error(),
            'planned_path_length_m': self.planned_path_length,
            'nav2_lifecycle_states': self.lifecycle_states,
            'minimum_distance_remaining': (
                self.minimum_distance_remaining
                if math.isfinite(self.minimum_distance_remaining) else None),
            'long_hold_duration_s': self.long_hold_duration,
            'long_hold_elapsed_s': (
                time.monotonic() - self.state_started_at
                if self.state in ('VALIDATING_LONG_HOLD', 'PASS')
                else None),
        }
        self.status_publisher.publish(String(data=json.dumps(
            payload, sort_keys=True, separators=(',', ':'))))
        self.last_status_published_at = time.monotonic()

    def _localized_goal_error(self):
        if self.latest_pose is None:
            return None
        return math.hypot(
            self.latest_pose[0] - self.goal_pose[0],
            self.latest_pose[1] - self.goal_pose[1],
        )

    def _physical_goal_error(self):
        if self.latest_root_position is None:
            return None
        return math.hypot(
            self.latest_root_position[0] - self.goal_pose[0],
            self.latest_root_position[1] - self.goal_pose[1],
        )

    def _fail(self, reason):
        if self.state == 'FAILED':
            return
        if self.navigation_goal_handle is not None:
            self.navigation_goal_handle.cancel_goal_async()
        if self.plan_goal_handle is not None and self.state == 'PLANNING':
            self.plan_goal_handle.cancel_goal_async()
        self._transition('FAILED', reason)

    def _on_timer(self):
        if self.state == 'FAILED':
            return
        self._update_lifecycle_states()
        state_elapsed = time.monotonic() - self.state_started_at
        if self.state == 'WAITING_FOR_HEALTH':
            if state_elapsed > self.ready_timeout:
                self._fail('HEALTH_GATE_TIMEOUT')
                return
            ready = (
                self.v12_ready_samples >= self.ready_sample_target
                and self.localization_ready_samples
                >= self.localization_sample_target
                and self.latest_pose is not None
                and self.latest_root_position is not None
                and self.plan_client.server_is_ready()
                and self.navigation_client.server_is_ready()
                and all(
                    state == State.PRIMARY_STATE_ACTIVE
                    for state in self.lifecycle_states.values())
            )
            if ready:
                position_error = math.hypot(
                    self.latest_pose[0] - self.startup_pose[0],
                    self.latest_pose[1] - self.startup_pose[1],
                )
                yaw_error = abs(normalize_angle(
                    self.latest_pose[2] - self.startup_pose[2]))
                physical_error = math.hypot(
                    self.latest_root_position[0] - self.startup_pose[0],
                    self.latest_root_position[1] - self.startup_pose[1],
                )
                if (
                        position_error > self.startup_position_tolerance
                        or yaw_error > self.startup_yaw_tolerance
                        or physical_error
                        > self.startup_position_tolerance):
                    self._fail('STARTUP_POSE_MISMATCH')
                    return
                self._send_plan()
        elif self.state in ('PLANNING', 'NAVIGATING'):
            if state_elapsed > self.navigation_timeout:
                self._fail('NAVIGATION_TIMEOUT')
        elif self.state == 'WAITING_FOR_STABLE_HOLD':
            if state_elapsed > self.stable_timeout:
                self._fail('STABLE_HOLD_TIMEOUT')
        elif self.state in ('VALIDATING_LONG_HOLD', 'PASS'):
            if (
                    self.last_locomotion_status_at is None
                    or time.monotonic() - self.last_locomotion_status_at
                    > self.hold_status_timeout):
                self._fail('LONG_HOLD_STATUS_TIMEOUT')
            elif (
                    self.state == 'VALIDATING_LONG_HOLD'
                    and state_elapsed >= self.long_hold_duration):
                self._transition(
                    'PASS', 'INCUBATOR_STAGING_GOAL_LONG_HOLD')
            elif (
                    self.state == 'PASS' and
                    time.monotonic() - self.last_status_published_at >= 0.2):
                self._publish_status()

    def _update_lifecycle_states(self):
        for name, client in self.lifecycle_clients.items():
            if (
                    self.lifecycle_futures[name] is not None
                    or not client.service_is_ready()):
                continue
            future = client.call_async(GetState.Request())
            self.lifecycle_futures[name] = future
            future.add_done_callback(
                lambda completed, node_name=name:
                self._on_lifecycle_state(node_name, completed))

    def _on_lifecycle_state(self, name, future):
        self.lifecycle_futures[name] = None
        try:
            response = future.result()
        except Exception:
            self.lifecycle_states[name] = State.PRIMARY_STATE_UNKNOWN
            return
        self.lifecycle_states[name] = int(response.current_state.id)

    def _pose_stamped(self):
        message = PoseStamped()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = 'map'
        message.pose.position.x = self.goal_pose[0]
        message.pose.position.y = self.goal_pose[1]
        message.pose.orientation.z = math.sin(0.5 * self.goal_pose[2])
        message.pose.orientation.w = math.cos(0.5 * self.goal_pose[2])
        return message

    def _send_plan(self):
        request = ComputePathToPose.Goal()
        request.goal = self._pose_stamped()
        request.planner_id = 'GridBased'
        request.use_start = False
        self._transition('PLANNING', 'HEALTH_GATES_PASS')
        future = self.plan_client.send_goal_async(request)
        future.add_done_callback(self._on_plan_response)

    def _on_plan_response(self, future):
        if self.state != 'PLANNING':
            return
        try:
            self.plan_goal_handle = future.result()
        except Exception as error:
            self._fail(f'PLAN_SEND_EXCEPTION:{error}')
            return
        if not self.plan_goal_handle.accepted:
            self._fail('PLAN_REJECTED')
            return
        future = self.plan_goal_handle.get_result_async()
        future.add_done_callback(self._on_plan_result)

    def _on_plan_result(self, future):
        if self.state != 'PLANNING':
            return
        try:
            wrapped = future.result()
        except Exception as error:
            self._fail(f'PLAN_RESULT_EXCEPTION:{error}')
            return
        path = wrapped.result.path.poses
        if (
                wrapped.status != GoalStatus.STATUS_SUCCEEDED
                or int(wrapped.result.error_code) != 0
                or len(path) < 2):
            self._fail(
                f'PLAN_FAILED:{wrapped.status}:'
                f'{int(wrapped.result.error_code)}')
            return
        path_points = [
            (pose.pose.position.x, pose.pose.position.y)
            for pose in path
        ]
        self.planned_path_length = sum(
            math.hypot(
                goal[0] - start[0],
                goal[1] - start[1],
            )
            for start, goal in zip(path_points, path_points[1:])
        )
        if self.planned_path_length > self.maximum_planned_path_length:
            self._fail('PLANNED_PATH_TOO_LONG')
            return
        for start, goal in zip(path_points, path_points[1:]):
            if segment_intersects_rectangle(
                    start, goal, self.expanded_workcell_bounds):
                self._fail('PLANNED_PATH_ENTERS_WORKCELL_KEEPOUT')
                return
        self._send_navigation_goal()

    def _send_navigation_goal(self):
        request = NavigateToPose.Goal()
        request.pose = self._pose_stamped()
        self._transition('NAVIGATING', 'PLAN_PASS')
        future = self.navigation_client.send_goal_async(
            request, feedback_callback=self._on_navigation_feedback)
        future.add_done_callback(self._on_navigation_response)

    def _on_navigation_response(self, future):
        if self.state != 'NAVIGATING':
            return
        try:
            self.navigation_goal_handle = future.result()
        except Exception as error:
            self._fail(f'GOAL_SEND_EXCEPTION:{error}')
            return
        if not self.navigation_goal_handle.accepted:
            self._fail('GOAL_REJECTED')
            return
        future = self.navigation_goal_handle.get_result_async()
        future.add_done_callback(self._on_navigation_result)

    def _on_navigation_feedback(self, message):
        self.minimum_distance_remaining = min(
            self.minimum_distance_remaining,
            float(message.feedback.distance_remaining),
        )

    def _on_navigation_result(self, future):
        if self.state != 'NAVIGATING':
            return
        try:
            wrapped = future.result()
        except Exception as error:
            self._fail(f'GOAL_RESULT_EXCEPTION:{error}')
            return
        if (
                wrapped.status != GoalStatus.STATUS_SUCCEEDED
                or int(wrapped.result.error_code) != NavigateToPose.Result.NONE):
            self._fail(
                f'GOAL_FAILED:{wrapped.status}:'
                f'{int(wrapped.result.error_code)}')
            return
        self._transition('WAITING_FOR_STABLE_HOLD', 'NAV2_GOAL_SUCCEEDED')

    def _on_pose(self, message):
        pose = message.pose.pose
        self.latest_pose = (
            float(pose.position.x),
            float(pose.position.y),
            _yaw_from_quaternion(pose.orientation),
        )

    def _on_localization(self, message):
        try:
            payload = json.loads(message.data)
        except json.JSONDecodeError:
            return
        valid = bool(
            payload.get('valid')
            and payload.get('map_to_odom_published'))
        self.latest_localization_valid = valid
        if valid:
            self.localization_ready_samples += 1
            self.invalid_localization_samples = 0
        else:
            if self.state == 'WAITING_FOR_HEALTH':
                self.localization_ready_samples = 0
            self.invalid_localization_samples += 1
            if (
                    self.state in (
                        'PLANNING', 'NAVIGATING',
                        'VALIDATING_LONG_HOLD', 'PASS')
                    and self.invalid_localization_samples
                    >= self.invalid_localization_limit):
                self._fail('LOCALIZATION_HEALTH_LOST')

    def _on_v12(self, message):
        self.last_locomotion_status_at = time.monotonic()
        try:
            payload = json.loads(message.data)
        except json.JSONDecodeError:
            return
        if payload.get('fault'):
            self._fail(f"V12_FAULT:{payload['fault']}")
            return
        state = payload.get('state')
        contacts = payload.get('contact_forces', [0.0, 0.0])
        velocity = payload.get('root_linear_velocity', [0.0, 0.0, 0.0])
        planar_speed = math.hypot(float(velocity[0]), float(velocity[1]))
        root_position = payload.get('root_position', [])
        root_position_valid = (
            len(root_position) >= 3
            and all(math.isfinite(float(value)) for value in root_position[:3])
        )
        self.latest_root_position = (
            tuple(float(value) for value in root_position[:2])
            if root_position_valid else None)
        if root_position_valid:
            self.latest_height = float(root_position[2])
        if 'height' in payload and math.isfinite(float(payload['height'])):
            self.latest_height = float(payload['height'])
        if 'up_z' in payload and math.isfinite(float(payload['up_z'])):
            self.latest_up_z = float(payload['up_z'])
        up_z = self.latest_up_z if self.latest_up_z is not None else -math.inf
        height = (
            self.latest_height
            if self.latest_height is not None else -math.inf)
        ready = (
            payload.get('fresh') is True
            and payload.get('movement_authorized') is True
            and state == 'STANDBY_HOLD'
            and up_z > 0.995
            and height > 1.0
            and locomotion_support_ready(payload, contacts)
        )
        if self.state == 'WAITING_FOR_HEALTH':
            self.v12_ready_samples = (
                self.v12_ready_samples + 1
                if ready and root_position_valid else 0)
        if self.state in (
                'PLANNING', 'NAVIGATING', 'WAITING_FOR_STABLE_HOLD',
                'VALIDATING_LONG_HOLD', 'PASS'):
            if not root_position_valid:
                self._fail('V12_ROOT_POSITION_INVALID')
                return
            physical_displacement = math.hypot(
                self.latest_root_position[0] - self.startup_pose[0],
                self.latest_root_position[1] - self.startup_pose[1],
            )
            corridor_error = point_to_segment_distance(
                self.latest_root_position,
                self.startup_pose[:2],
                self.goal_pose[:2],
            )
            if (
                    physical_displacement
                    > self.maximum_physical_displacement
                    or corridor_error > self.physical_corridor_tolerance):
                self._fail('PHYSICAL_PATH_ENVELOPE_EXCEEDED')
                return
        if (
                self.state in ('PLANNING', 'NAVIGATING')
                and (up_z <= 0.990 or height <= 0.95)):
            self._fail('UPRIGHT_HEALTH_LOST')
            return
        localized_goal_error = self._localized_goal_error()
        physical_goal_error = self._physical_goal_error()
        localized_yaw_error = (
            normalize_angle(self.latest_pose[2] - self.goal_pose[2])
            if self.latest_pose is not None else None)
        if self.state in ('VALIDATING_LONG_HOLD', 'PASS'):
            if not sustained_hold_sample_ready(
                    payload,
                    self.latest_localization_valid,
                    localized_goal_error,
                    physical_goal_error,
                    localized_yaw_error,
                    planar_speed,
                    self.goal_position_tolerance,
                    self.goal_yaw_tolerance,
                    up_z,
                    height,
                    contacts,
                    self.long_hold_minimum_up_z,
                    self.long_hold_minimum_height):
                reason = (
                    'POST_PASS_HOLD_ENVELOPE_EXCEEDED'
                    if self.state == 'PASS'
                    else 'LONG_HOLD_ENVELOPE_EXCEEDED')
                self._fail(reason)
            return
        stable = (
            self.state == 'WAITING_FOR_STABLE_HOLD'
            and self.latest_localization_valid
            and localized_goal_error is not None
            and localized_goal_error <= self.goal_position_tolerance
            and physical_goal_error is not None
            and physical_goal_error <= self.goal_position_tolerance
            and self.latest_pose is not None
            and abs(localized_yaw_error) <= self.goal_yaw_tolerance
            and payload.get('fresh') is True
            and locomotion_hold_ready(
                payload,
                planar_speed,
                self.goal_position_tolerance,
                self.goal_yaw_tolerance)
            and up_z > 0.995
            and height > 1.0
            and len(contacts) == 2
            and all(float(force) > 0.0 for force in contacts)
        )
        self.stable_samples = self.stable_samples + 1 if stable else 0
        if self.stable_samples >= self.stable_sample_target:
            self._transition(
                'VALIDATING_LONG_HOLD', 'SHORT_HOLD_GATE_PASS')


def main(args=None):
    """Run the guarded incubator staging navigator."""
    rclpy.init(args=args)
    node = IncubatorAutoNavigator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
