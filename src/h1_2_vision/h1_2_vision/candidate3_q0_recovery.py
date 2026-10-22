"""
Gazebo-only recovery along an already-certified candidate-3 prefix.

This utility exists only for a fail-closed execution that stopped before the
handle was actuated.  It accepts an explicitly hashed complete g5 result,
finds the current left-arm state on stage 0/message 0, and publishes the exact
certified prefix in reverse back to q0.  It never resets Gazebo or changes a
support, latch, gripper, object pose, or any controller other than left arm.
"""

from __future__ import annotations

import hashlib
import json
import math
import os
from pathlib import Path
import subprocess
import threading
import time
import uuid

from ament_index_python.packages import (
    get_package_prefix,
    get_package_share_directory,
)
from h1_2_perception_msgs.msg import TagObservation, TrackedObjectState
import numpy as np
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from .candidate3_contract import pose_matrix
from .candidate3_execution_contract import (
    CONTROLLER_JOINTS,
    load_result_file,
)
from .tag_config import load_tag_map


TOPIC = '/left_arm_controller/joint_trajectory'
SCHEMA = 'h1_candidate3_q0_recovery_evidence/v1'


class RecoveryFault(RuntimeError):
    """A recovery precondition or runtime invariant failed."""


def stamp_ns(stamp):
    """Convert a ROS stamp to integer nanoseconds."""
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def duration_ns(value):
    """Convert a serialized duration to integer nanoseconds."""
    return int(value['sec']) * 1_000_000_000 + int(value['nanosec'])


def ns_duration(value):
    """Convert nonnegative integer nanoseconds to ROS duration fields."""
    return int(value // 1_000_000_000), int(value % 1_000_000_000)


def rotation_error(left, right):
    """Return the geodesic rotation distance between two transforms."""
    relative = np.asarray(left)[:3, :3].T @ np.asarray(right)[:3, :3]
    cosine = (float(np.trace(relative)) - 1.0) * 0.5
    return math.acos(min(1.0, max(-1.0, cosine)))


class Candidate3Q0Recovery(Node):
    """Recover one stopped, pre-contact Gazebo execution to certified q0."""

    def __init__(self):
        super().__init__('h1_2_candidate3_q0_recovery')
        self.declare_parameter('execute', False)
        self.declare_parameter('execution_mode', 'disabled')
        self.declare_parameter('result', '')
        self.declare_parameter('result_sha256', '')
        self.declare_parameter('evidence_output_root', '')
        self.declare_parameter('maximum_telemetry_age_s', 0.75)
        self.declare_parameter('nearest_point_tolerance_rad', 0.003)
        self.declare_parameter('endpoint_tolerance_rad', 0.003)

        self.lock = threading.Lock()
        self.joints = {}
        self.joint_receive_ns = 0
        self.mechanism = {}
        self.mechanism_velocity = {}
        self.mechanism_receive_ns = 0
        self.latch = ''
        self.latch_receive_ns = 0
        self.right_support = ''
        self.right_support_receive_ns = 0
        self.left_support = ''
        self.left_support_receive_ns = 0
        self.cabinet = None
        self.cabinet_receive_ns = 0
        self.raw_cabinet = None
        self.raw_cabinet_stamp_ns = 0
        self.raw_cabinet_receive_ns = 0
        self.events = []
        self.motion_started = False
        self.done = threading.Event()
        self.bridge = None
        self.run_directory = None

        vision_share = Path(get_package_share_directory('h1_2_vision'))
        description_share = Path(
            get_package_share_directory('h1_2_description'))
        self.tag_config = load_tag_map(
            vision_share / 'config/tag_map.yaml', [
                description_share / 'models/incubator/materials/textures',
                description_share / 'models/green_cube/materials/textures',
            ])
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        self.publisher = self.create_publisher(JointTrajectory, TOPIC, qos)
        self.create_subscription(
            JointState, '/joint_states', self.joint_callback,
            qos_profile_sensor_data)
        self.create_subscription(
            JointState, '/incubator/joint_states', self.mechanism_callback,
            qos_profile_sensor_data)
        self.create_subscription(
            String, '/incubator/latch/state', self.latch_callback, 10)
        self.create_subscription(
            String, '/h1_2/right_door_support/state',
            self.right_support_callback, 10)
        self.create_subscription(
            String, '/h1_2/left_pose_grasp/state',
            self.left_support_callback, 10)
        self.create_subscription(
            TrackedObjectState, '/tag_pose/cabinet',
            self.cabinet_callback, 10)
        self.create_subscription(
            TagObservation, '/apriltag/detections',
            self.observation_callback, 20)
        threading.Thread(target=self.run, daemon=True).start()

    def event(self, status, **fields):
        """Record a monotonic and simulation-time audit event."""
        value = {
            'monotonic_ns': time.monotonic_ns(),
            'ros_time_ns': self.get_clock().now().nanoseconds,
            'status': status,
            **fields,
        }
        with self.lock:
            self.events.append(value)
        self.get_logger().info(json.dumps(value, sort_keys=True))

    def joint_callback(self, message):
        """Store finite, complete robot joint telemetry."""
        if len(message.name) != len(message.position):
            return
        values = dict(zip(message.name, map(float, message.position)))
        if len(values) != len(message.name) or not all(
                math.isfinite(value) for value in values.values()):
            return
        with self.lock:
            self.joints = values
            self.joint_receive_ns = time.monotonic_ns()

    def mechanism_callback(self, message):
        """Store finite door and handle telemetry."""
        if (len(message.name) != len(message.position) or
                len(message.name) != len(message.velocity)):
            return
        position = dict(zip(message.name, map(float, message.position)))
        velocity = dict(zip(message.name, map(float, message.velocity)))
        if (len(position) != len(message.name) or
                not all(math.isfinite(value) for value in position.values()) or
                not all(math.isfinite(value) for value in velocity.values())):
            return
        with self.lock:
            self.mechanism = position
            self.mechanism_velocity = velocity
            self.mechanism_receive_ns = time.monotonic_ns()

    def latch_callback(self, message):
        """Store the mature latch-plugin state."""
        with self.lock:
            self.latch = message.data.strip().lower()
            self.latch_receive_ns = time.monotonic_ns()

    def right_support_callback(self, message):
        """Store right-door support state from the read-only bridge."""
        with self.lock:
            self.right_support = message.data.strip().lower()
            self.right_support_receive_ns = time.monotonic_ns()

    def left_support_callback(self, message):
        """Store left-cube support state from the read-only bridge."""
        with self.lock:
            self.left_support = message.data.strip().lower()
            self.left_support_receive_ns = time.monotonic_ns()

    def cabinet_callback(self, message):
        """Store fused cabinet pose telemetry."""
        with self.lock:
            self.cabinet = message
            self.cabinet_receive_ns = time.monotonic_ns()

    def observation_callback(self, message):
        """Convert a valid raw cabinet tag through the hashed tag map."""
        record = self.tag_config['tags_by_id'].get(int(message.id))
        if (record is None or record['object_name'] != 'cabinet' or
                message.object_name != 'cabinet' or
                message.family != self.tag_config['family'] or
                message.world_frame != 'world' or
                message.config_hash != self.tag_config['config_hash'] or
                not message.valid or message.status != 'VALID'):
            return
        try:
            value = (
                pose_matrix(message.pose_world.pose) @
                np.linalg.inv(record['object_from_tag']) @
                self.tag_config['objects']['cabinet'][
                    'object_pose_correction'])
        except (ValueError, np.linalg.LinAlgError):
            return
        value_stamp_ns = stamp_ns(message.header.stamp)
        if value_stamp_ns <= 0:
            return
        with self.lock:
            self.raw_cabinet = value
            self.raw_cabinet_stamp_ns = value_stamp_ns
            self.raw_cabinet_receive_ns = time.monotonic_ns()

    @staticmethod
    def write_exclusive(path, payload):
        """Write owner-read-only evidence without following or replacing."""
        descriptor = os.open(
            path, os.O_WRONLY | os.O_CREAT | os.O_EXCL | os.O_NOFOLLOW,
            0o400)
        with os.fdopen(descriptor, 'w', encoding='utf-8') as stream:
            stream.write(payload)

    def start_bridge(self):
        """Bridge only the two already-preloaded support state topics."""
        executable = (Path(get_package_prefix('ros_gz_bridge')) / 'lib' /
                      'ros_gz_bridge' / 'parameter_bridge')
        self.bridge = subprocess.Popen(
            [
                str(executable),
                '/h1_2/right_door_support/state@'
                'std_msgs/msg/String[gz.msgs.StringMsg',
                '/h1_2/left_pose_grasp/state@'
                'std_msgs/msg/String[gz.msgs.StringMsg',
                '--ros-args', '-p', 'use_sim_time:=true',
            ],
            stdin=subprocess.DEVNULL, stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL, close_fds=True)
        time.sleep(0.2)
        if self.bridge.poll() is not None:
            raise RecoveryFault('SUPPORT_BRIDGE')

    def stop_bridge(self):
        """Stop the read-only support bridge at the terminal state."""
        if self.bridge is None or self.bridge.poll() is not None:
            return
        self.bridge.terminate()
        try:
            self.bridge.wait(timeout=3.0)
        except subprocess.TimeoutExpired:
            self.bridge.kill()
            self.bridge.wait(timeout=3.0)

    def require_fresh(self, plan, require_stable=False):
        """Enforce telemetry, q0 mechanism/support, and cabinet invariants."""
        maximum_age_ns = int(float(
            self.get_parameter('maximum_telemetry_age_s').value) * 1e9)
        now_wall = time.monotonic_ns()
        with self.lock:
            joints = dict(self.joints)
            joint_receive = self.joint_receive_ns
            mechanism = dict(self.mechanism)
            velocity = dict(self.mechanism_velocity)
            mechanism_receive = self.mechanism_receive_ns
            latch = self.latch
            latch_receive = self.latch_receive_ns
            right = self.right_support
            right_receive = self.right_support_receive_ns
            left = self.left_support
            left_receive = self.left_support_receive_ns
            cabinet = self.cabinet
            cabinet_receive = self.cabinet_receive_ns
            raw = None if self.raw_cabinet is None else self.raw_cabinet.copy()
            raw_stamp = self.raw_cabinet_stamp_ns
            raw_receive = self.raw_cabinet_receive_ns
        required = {name for names in CONTROLLER_JOINTS.values()
                    for name in names}
        if (required - joints.keys() or not joint_receive or
                now_wall - joint_receive > maximum_age_ns):
            raise RecoveryFault('STALE_ROBOT_STATE')
        if (not mechanism_receive or
                now_wall - mechanism_receive > maximum_age_ns or
                abs(mechanism.get('door_hinge_joint', math.inf)) >
                math.radians(0.2) or
                abs(mechanism.get('handle_latch_joint', math.inf)) >
                math.radians(3.0) or
                abs(velocity.get('door_hinge_joint', math.inf)) > 0.01 or
                abs(velocity.get('handle_latch_joint', math.inf)) > 0.01):
            raise RecoveryFault('MECHANISM_NOT_Q0_STATIONARY')
        if (latch != 'locked' or not latch_receive or
                now_wall - latch_receive > maximum_age_ns):
            raise RecoveryFault('LATCH_NOT_LOCKED')
        support_age_ns = 6_000_000_000
        if (right != 'detached' or left != 'detached' or
                not right_receive or not left_receive or
                now_wall - right_receive > support_age_ns or
                now_wall - left_receive > support_age_ns):
            raise RecoveryFault('SUPPORT_NOT_FRESH_DETACHED')
        stable = (
            cabinet is not None and cabinet.valid and cabinet.stable and
            cabinet.config_hash == self.tag_config['config_hash'] and
            cabinet_receive and now_wall - cabinet_receive <= maximum_age_ns)
        raw_valid = (
            not require_stable and raw is not None and raw_receive and
            now_wall - raw_receive <= maximum_age_ns)
        if stable:
            live = pose_matrix(cabinet.pose_world.pose)
            value_stamp = stamp_ns(cabinet.header.stamp)
        elif raw_valid:
            live = raw
            value_stamp = raw_stamp
        else:
            raise RecoveryFault('CABINET_TAG')
        stamp_age = self.get_clock().now().nanoseconds - value_stamp
        if stamp_age < 0 or stamp_age > maximum_age_ns:
            raise RecoveryFault('CABINET_TAG_STAMP')
        expected = np.asarray(plan['current_cabinet_pose'])
        translation = float(np.linalg.norm(live[:3, 3] - expected[:3, 3]))
        angle = rotation_error(expected, live)
        if translation > 0.003 or angle > math.radians(1.0):
            raise RecoveryFault(
                f'CABINET_POSE:{translation:.9f}:{angle:.9f}')
        return joints

    @staticmethod
    def require_close(joints, names, values, tolerance, status):
        """Require an observed joint vector within a maximum error."""
        if any(name not in joints for name in names):
            raise RecoveryFault(f'{status}:MISSING')
        error = max(abs(joints[name] - float(value))
                    for name, value in zip(names, values))
        if error > tolerance:
            raise RecoveryFault(f'{status}:{error:.9f}')
        return error

    def require_controller(self):
        """Require the one-publisher/one-subscriber left-arm topology."""
        deadline = time.monotonic() + 10.0
        while time.monotonic() < deadline:
            if (self.publisher.get_subscription_count() == 1 and
                    self.count_publishers(TOPIC) == 1):
                break
            time.sleep(0.05)
        else:
            raise RecoveryFault('CONTROLLER_TOPOLOGY')
        completed = subprocess.run(
            ['ros2', 'control', 'list_controllers'],
            stdin=subprocess.DEVNULL, capture_output=True, text=True,
            timeout=10.0, check=False)
        if completed.returncode != 0:
            raise RecoveryFault('CONTROLLER_LIST')
        expected = {
            'joint_state_broadcaster', 'left_arm_controller',
            'right_arm_controller', 'left_gripper_controller',
            'right_gripper_controller',
        }
        active = {
            line.split()[0] for line in completed.stdout.splitlines()
            if len(line.split()) >= 3 and line.split()[-1] == 'active'
        }
        if active != expected:
            raise RecoveryFault(f'CONTROLLERS_ACTIVE:{sorted(active)}')

    @staticmethod
    def reverse_prefix(record, index):
        """Build the exact point-index prefix in certified reverse order."""
        source = record['points']
        source_end = duration_ns(source[index]['time_from_start'])
        result = JointTrajectory()
        result.joint_names = list(record['joint_names'])
        for value in reversed(source[:index + 1]):
            point = JointTrajectoryPoint()
            point.positions = list(map(float, value['positions']))
            point.velocities = [-float(item) for item in value['velocities']]
            point.accelerations = list(map(float, value['accelerations']))
            point.effort = list(map(float, value['effort']))
            seconds, nanoseconds = ns_duration(
                source_end - duration_ns(value['time_from_start']))
            point.time_from_start.sec = seconds
            point.time_from_start.nanosec = nanoseconds
            result.points.append(point)
        return result, source_end

    def measured_hold(self):
        """Preempt the left-arm controller with the latest measured state."""
        with self.lock:
            joints = dict(self.joints)
        names = CONTROLLER_JOINTS[TOPIC]
        if any(name not in joints for name in names):
            return 'HOLD_FAIL:MISSING'
        message = JointTrajectory()
        message.joint_names = list(names)
        point = JointTrajectoryPoint()
        point.positions = [joints[name] for name in names]
        point.time_from_start.nanosec = 100_000_000
        message.points.append(point)
        for _ in range(3):
            self.publisher.publish(message)
            time.sleep(0.05)
        return 'HOLD_PASS'

    def run(self):
        """Validate, execute the reverse prefix, and seal evidence."""
        result = None
        source_sha = ''
        nearest_index = -1
        nearest_error = math.inf
        try:
            if (not bool(self.get_parameter('execute').value) or
                    str(self.get_parameter('execution_mode').value) !=
                    'gazebo'):
                raise RecoveryFault('NOT_EXPLICIT_GAZEBO')
            result_path = Path(str(self.get_parameter('result').value))
            expected_sha = str(
                self.get_parameter('result_sha256').value).lower()
            evidence_root = Path(str(
                self.get_parameter('evidence_output_root').value))
            if (not result_path.is_absolute() or not evidence_root.is_absolute()
                    or len(expected_sha) != 64):
                raise RecoveryFault('CONFIG_PATH_OR_HASH')
            source_sha = hashlib.sha256(result_path.read_bytes()).hexdigest()
            if source_sha != expected_sha:
                raise RecoveryFault('RESULT_SHA256')
            result = load_result_file(result_path, result_path.parent)
            evidence_root.mkdir(parents=True, exist_ok=True)
            self.run_directory = evidence_root / (
                f'run_{time.strftime("%Y%m%dT%H%M%S")}_{uuid.uuid4().hex}')
            self.run_directory.mkdir(mode=0o700)
            self.start_bridge()
            deadline = time.monotonic() + 10.0
            while time.monotonic() < deadline:
                try:
                    joints = self.require_fresh(
                        result['plan'], require_stable=True)
                    break
                except RecoveryFault:
                    time.sleep(0.02)
            else:
                raise RecoveryFault('TELEMETRY_PRECONDITION_TIMEOUT')
            self.require_controller()
            plan = result['plan']
            record = plan['stages'][0]['messages'][0]
            names = record['joint_names']
            untouched = (
                plan['stages'][0]['messages'][1],
                plan['stages'][2]['messages'][0],
                plan['stages'][2]['messages'][1],
            )
            for message in untouched:
                self.require_close(
                    joints, message['joint_names'],
                    message['points'][0]['positions'], 0.003,
                    'UNTOUCHED_CONTROLLER_Q0')
            errors = [
                max(abs(joints[name] - float(value))
                    for name, value in zip(names, point['positions']))
                for point in record['points']
            ]
            nearest_index = min(range(len(errors)), key=errors.__getitem__)
            nearest_error = errors[nearest_index]
            tolerance = float(
                self.get_parameter('nearest_point_tolerance_rad').value)
            endpoint_tolerance = float(
                self.get_parameter('endpoint_tolerance_rad').value)
            if (not 0.0 < tolerance <= 0.003 or nearest_error > tolerance or
                    not 0.0 < endpoint_tolerance <= 0.003 or
                    nearest_index <= 0 or nearest_index >= len(errors) - 1):
                raise RecoveryFault(
                    f'NOT_ON_CERTIFIED_PREFIX:{nearest_index}:'
                    f'{nearest_error:.9f}')
            trajectory, trajectory_duration_ns = self.reverse_prefix(
                record, nearest_index)
            self.event(
                'RECOVERY_ADMITTED', result_sha256=source_sha,
                plan_hash=result['plan_hash'], point_index=nearest_index,
                nearest_error_rad=nearest_error,
                point_count=len(trajectory.points))
            publish_wall_ns = time.monotonic_ns()
            publish_sim_ns = self.get_clock().now().nanoseconds
            self.motion_started = True
            self.publisher.publish(trajectory)
            deadline = time.monotonic() + 20.0
            stable_count = 0
            while time.monotonic() < deadline:
                joints = self.require_fresh(plan, require_stable=False)
                if self.get_clock().now().nanoseconds - publish_sim_ns >= (
                        trajectory_duration_ns):
                    try:
                        self.require_close(
                            joints, names, record['points'][0]['positions'],
                            endpoint_tolerance, 'RECOVERY_ENDPOINT')
                        if self.joint_receive_ns > publish_wall_ns:
                            stable_count += 1
                    except RecoveryFault:
                        stable_count = 0
                    if stable_count >= 3:
                        break
                time.sleep(0.005)
            else:
                raise RecoveryFault('RECOVERY_TIMEOUT')
            final_deadline = time.monotonic() + 3.0
            while time.monotonic() < final_deadline:
                try:
                    joints = self.require_fresh(plan, require_stable=True)
                    break
                except RecoveryFault:
                    time.sleep(0.02)
            else:
                raise RecoveryFault('FINAL_STABLE_TELEMETRY_TIMEOUT')
            endpoint_error = self.require_close(
                joints, names, record['points'][0]['positions'], 0.003,
                'RECOVERY_FINAL_Q0')
            self.event(
                'RECOVERY_PASS', endpoint_error_rad=endpoint_error,
                duration_s=trajectory_duration_ns * 1e-9)
            self.finish(
                'PASS', result, source_sha, nearest_index, nearest_error,
                endpoint_error_rad=endpoint_error)
        except Exception as error:
            hold = self.measured_hold() if self.motion_started else 'NO_MOTION'
            self.finish(
                'FAIL_CLOSED', result, source_sha, nearest_index,
                nearest_error, reason=f'{type(error).__name__}:{error};{hold}')

    def finish(self, status, result, source_sha, index, error, **fields):
        """Seal one terminal recovery evidence record."""
        try:
            self.event(status, **fields)
            if self.run_directory is not None:
                with self.lock:
                    payload = {
                        'schema': SCHEMA,
                        'status': status,
                        'gazebo_only': True,
                        'source_result_sha256': source_sha,
                        'plan_hash': result.get('plan_hash', '') if result else '',
                        'nearest_source_point_index': index,
                        'nearest_source_point_error_rad': (
                            error if math.isfinite(error) else None),
                        'motion_started': self.motion_started,
                        'events': list(self.events),
                        **fields,
                    }
                self.write_exclusive(
                    self.run_directory / 'recovery_result.json',
                    json.dumps(
                        payload, sort_keys=True, indent=2,
                        allow_nan=False) + '\n')
        finally:
            self.done.set()


def main(args=None):
    """Run the explicit Gazebo-only recovery node."""
    rclpy.init(args=args)
    node = Candidate3Q0Recovery()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        while rclpy.ok() and not node.done.is_set():
            executor.spin_once(timeout_sec=0.1)
    except KeyboardInterrupt:
        if node.motion_started:
            node.measured_hold()
        node.finish(
            'FAIL_CLOSED', None, '', -1, math.inf,
            reason='KEYBOARD_INTERRUPT')
    finally:
        node.stop_bridge()
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
