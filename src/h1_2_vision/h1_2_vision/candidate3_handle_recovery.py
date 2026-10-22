"""
Audit and execute a constrained Gazebo recovery from partial handle motion.

The node is default-disabled and accepts only a hash-bound complete g5 result.
It keeps the left gripper clamped, builds a dense connector from the measured
left-arm hold to the nearest certified handle path state, appends the exact
certified handle reverse, gripper release, and arm retreat, then invokes the
file-only C++ clearance auditor before publishing anything.  It never resets
Gazebo, changes an object pose, or detaches a support.
"""

from __future__ import annotations

import copy
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

from .candidate3_contract import canonical_hash, pose_matrix
from .candidate3_execution_contract import (
    CONTROLLER_JOINTS,
    load_result_file,
)
from .tag_config import load_tag_map


ARM_TOPIC = '/left_arm_controller/joint_trajectory'
GRIPPER_TOPIC = '/left_gripper_controller/joint_trajectory'
SCHEMA = 'h1_candidate3_handle_recovery_evidence/v1'
MAXIMUM_ADJACENT_CHANGE = math.radians(4.9)


class HandleRecoveryFault(RuntimeError):
    """A constrained handle-recovery gate failed."""


def stamp_ns(stamp):
    """Convert a ROS stamp to integer nanoseconds."""
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def serialized_duration_ns(point):
    """Read a serialized trajectory point time."""
    value = point['time_from_start']
    return int(value['sec']) * 1_000_000_000 + int(value['nanosec'])


def duration_fields(value_ns):
    """Return ROS duration fields for a nonnegative nanosecond value."""
    return {
        'sec': int(value_ns // 1_000_000_000),
        'nanosec': int(value_ns % 1_000_000_000),
    }


class Candidate3HandleRecovery(Node):
    """Recover a stopped, locked, partially rotated handle to full q0."""

    def __init__(self):
        super().__init__('h1_2_candidate3_handle_recovery')
        self.declare_parameter('execute', False)
        self.declare_parameter('execution_mode', 'disabled')
        self.declare_parameter('result', '')
        self.declare_parameter('result_sha256', '')
        self.declare_parameter('evidence_output_root', '')
        self.declare_parameter('maximum_telemetry_age_s', 0.75)

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
        self.publishers_by_topic = {
            ARM_TOPIC: self.create_publisher(JointTrajectory, ARM_TOPIC, qos),
            GRIPPER_TOPIC: self.create_publisher(
                JointTrajectory, GRIPPER_TOPIC, qos),
        }
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
        """Append one evidence event."""
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
        """Store finite robot joint telemetry."""
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
        """Store finite door and handle feedback."""
        if (len(message.name) != len(message.position) or
                len(message.name) != len(message.velocity)):
            return
        positions = dict(zip(message.name, map(float, message.position)))
        velocities = dict(zip(message.name, map(float, message.velocity)))
        if (len(positions) != len(message.name) or
                not all(math.isfinite(v) for v in positions.values()) or
                not all(math.isfinite(v) for v in velocities.values())):
            return
        with self.lock:
            self.mechanism = positions
            self.mechanism_velocity = velocities
            self.mechanism_receive_ns = time.monotonic_ns()

    def latch_callback(self, message):
        """Store mature latch state."""
        with self.lock:
            self.latch = message.data.strip().lower()
            self.latch_receive_ns = time.monotonic_ns()

    def right_support_callback(self, message):
        """Store right-door support state."""
        with self.lock:
            self.right_support = message.data.strip().lower()
            self.right_support_receive_ns = time.monotonic_ns()

    def left_support_callback(self, message):
        """Store left-cube support state."""
        with self.lock:
            self.left_support = message.data.strip().lower()
            self.left_support_receive_ns = time.monotonic_ns()

    def cabinet_callback(self, message):
        """Store fused cabinet tag state."""
        with self.lock:
            self.cabinet = message
            self.cabinet_receive_ns = time.monotonic_ns()

    def observation_callback(self, message):
        """Derive a cabinet pose from a valid raw tag observation."""
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
        value_stamp = stamp_ns(message.header.stamp)
        if value_stamp <= 0:
            return
        with self.lock:
            self.raw_cabinet = value
            self.raw_cabinet_stamp_ns = value_stamp
            self.raw_cabinet_receive_ns = time.monotonic_ns()

    @staticmethod
    def write_exclusive(path, payload):
        """Write owner-read-only evidence without replacing a path."""
        descriptor = os.open(
            path, os.O_WRONLY | os.O_CREAT | os.O_EXCL | os.O_NOFOLLOW,
            0o400)
        with os.fdopen(descriptor, 'w', encoding='utf-8') as stream:
            stream.write(payload)

    def start_bridge(self):
        """Start a read-only bridge for the preloaded support states."""
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
            raise HandleRecoveryFault('SUPPORT_BRIDGE')

    def stop_bridge(self):
        """Stop the read-only support bridge."""
        if self.bridge is None or self.bridge.poll() is not None:
            return
        self.bridge.terminate()
        try:
            self.bridge.wait(timeout=3.0)
        except subprocess.TimeoutExpired:
            self.bridge.kill()
            self.bridge.wait(timeout=3.0)

    def snapshot(self, plan, stable_cabinet=False,
                 stationary_mechanism=True):
        """Require and return the constrained live state."""
        maximum_age_ns = int(float(
            self.get_parameter('maximum_telemetry_age_s').value) * 1e9)
        now_wall = time.monotonic_ns()
        with self.lock:
            joints = dict(self.joints)
            joint_receive = self.joint_receive_ns
            mechanism = dict(self.mechanism)
            velocities = dict(self.mechanism_velocity)
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
            raise HandleRecoveryFault('STALE_ROBOT_STATE')
        door = mechanism.get('door_hinge_joint', math.inf)
        handle = mechanism.get('handle_latch_joint', math.inf)
        if (not mechanism_receive or
                now_wall - mechanism_receive > maximum_age_ns or
                abs(door) > math.radians(0.2) or
                not -math.radians(1.0) <= handle <= math.radians(95.0) or
                abs(velocities.get('door_hinge_joint', math.inf)) > 0.01 or
                abs(velocities.get('handle_latch_joint', math.inf)) >
                (0.01 if stationary_mechanism else 0.35 + 1e-6)):
            raise HandleRecoveryFault(
                'MECHANISM_CONSTRAINT:'
                f'handle_position={handle:.9f}:'
                f'handle_velocity={velocities.get("handle_latch_joint", math.inf):.9f}')
        if (latch != 'locked' or not latch_receive or
                now_wall - latch_receive > maximum_age_ns):
            raise HandleRecoveryFault('LATCH_NOT_LOCKED')
        if (right != 'detached' or left != 'detached' or
                not right_receive or not left_receive or
                now_wall - right_receive > 6_000_000_000 or
                now_wall - left_receive > 6_000_000_000):
            raise HandleRecoveryFault('SUPPORT_NOT_DETACHED')
        stable = (
            cabinet is not None and cabinet.valid and cabinet.stable and
            cabinet.config_hash == self.tag_config['config_hash'] and
            cabinet_receive and now_wall - cabinet_receive <= maximum_age_ns)
        raw_valid = (
            not stable_cabinet and raw is not None and raw_receive and
            now_wall - raw_receive <= maximum_age_ns)
        if stable:
            live = pose_matrix(cabinet.pose_world.pose)
            value_stamp = stamp_ns(cabinet.header.stamp)
        elif raw_valid:
            live = raw
            value_stamp = raw_stamp
        else:
            raise HandleRecoveryFault('CABINET_TAG')
        stamp_age = self.get_clock().now().nanoseconds - value_stamp
        if stamp_age < 0 or stamp_age > maximum_age_ns:
            raise HandleRecoveryFault('CABINET_TAG_STAMP')
        expected = np.asarray(plan['current_cabinet_pose'])
        translation = float(np.linalg.norm(live[:3, 3] - expected[:3, 3]))
        relative = expected[:3, :3].T @ live[:3, :3]
        angle = math.acos(min(1.0, max(
            -1.0, (float(np.trace(relative)) - 1.0) * 0.5)))
        if translation > 0.003 or angle > math.radians(1.0):
            raise HandleRecoveryFault('CABINET_POSE')
        return joints, mechanism

    def require_controller_topology(self):
        """Require only these two publishers and all five active controllers."""
        deadline = time.monotonic() + 10.0
        while time.monotonic() < deadline:
            if all(
                    publisher.get_subscription_count() == 1 and
                    self.count_publishers(topic) == 1
                    for topic, publisher in self.publishers_by_topic.items()):
                break
            time.sleep(0.05)
        else:
            raise HandleRecoveryFault('CONTROLLER_TOPOLOGY')
        completed = subprocess.run(
            ['ros2', 'control', 'list_controllers'],
            stdin=subprocess.DEVNULL, capture_output=True, text=True,
            timeout=10.0, check=False)
        expected = {
            'joint_state_broadcaster', 'left_arm_controller',
            'right_arm_controller', 'left_gripper_controller',
            'right_gripper_controller',
        }
        active = {
            line.split()[0] for line in completed.stdout.splitlines()
            if len(line.split()) >= 3 and line.split()[-1] == 'active'
        }
        if completed.returncode != 0 or active != expected:
            raise HandleRecoveryFault('CONTROLLERS_ACTIVE')

    @staticmethod
    def maximum_error(joints, names, positions):
        """Return maximum observed joint error."""
        if any(name not in joints for name in names):
            return math.inf
        return max(abs(joints[name] - float(value))
                   for name, value in zip(names, positions))

    @staticmethod
    def point(positions, value_ns):
        """Create a position-only serialized trajectory point."""
        return {
            'positions': list(map(float, positions)),
            'velocities': [],
            'accelerations': [],
            'effort': [],
            'time_from_start': duration_fields(value_ns),
        }

    def build_recovery(self, result, joints):
        """Build the dense audited connector and certified reverse sequence."""
        plan = result['plan']
        handle_forward = plan['stages'][0]['messages'][2]
        names = handle_forward['joint_names']
        actual = np.asarray([joints[name] for name in names])
        errors = [
            float(np.max(np.abs(actual - np.asarray(point['positions']))))
            for point in handle_forward['points']
        ]
        nearest_index = min(range(len(errors)), key=errors.__getitem__)
        nearest = np.asarray(
            handle_forward['points'][nearest_index]['positions'])
        connector_error = errors[nearest_index]
        if connector_error > MAXIMUM_ADJACENT_CHANGE:
            raise HandleRecoveryFault(
                f'CONNECTOR_TOO_LARGE:{connector_error:.9f}')
        connector_count = max(
            2, int(math.ceil(connector_error / math.radians(0.049))) + 1)
        connector_duration_s = max(
            0.05, 1.5 * connector_error / 1.0,
            math.sqrt(6.0 * connector_error / 2.0))
        connector_duration_ns = int(math.ceil(
            connector_duration_s * 1_000_000_000.0))
        arm_points = []
        for index in range(connector_count):
            fraction = index / (connector_count - 1)
            # Smoothstep gives zero endpoint velocity for the measured hold
            # and the connector endpoint while retaining the straight joint
            # segment whose every sample is replayed by the C++ auditor.
            smooth = fraction * fraction * (3.0 - 2.0 * fraction)
            position = actual + smooth * (nearest - actual)
            arm_points.append(self.point(
                position,
                int(round(connector_duration_ns * fraction))))
        nearest_time_ns = serialized_duration_ns(
            handle_forward['points'][nearest_index])
        recovery_time_scale = 2
        for source in reversed(handle_forward['points'][:nearest_index]):
            offset_ns = (
                connector_duration_ns + recovery_time_scale * (
                    nearest_time_ns - serialized_duration_ns(source)))
            arm_points.append(self.point(source['positions'], offset_ns))

        gripper_close = plan['stages'][0]['messages'][1]
        gripper_reverse = plan['stages'][6]['messages'][1]
        arm_retreat = plan['stages'][6]['messages'][2]
        gripper_names = list(gripper_reverse['joint_names'])
        actual_gripper = [joints[name] for name in gripper_names]
        gripper_points = [self.point(actual_gripper, 0)]
        gripper_points.append(self.point(
            gripper_reverse['points'][0]['positions'], 500_000_000))
        for point in gripper_reverse['points'][1:]:
            gripper_points.append(self.point(
                point['positions'],
                500_000_000 + serialized_duration_ns(point)))
        physical_messages = [
            {
                'name': 'handle_reverse_to_q0',
                'topic': ARM_TOPIC,
                'joint_names': list(names),
                'points': arm_points,
            },
            {
                'name': 'gripper_open_after_handle_q0',
                'topic': GRIPPER_TOPIC,
                'joint_names': gripper_names,
                'points': gripper_points,
            },
            {
                'name': 'arm_retreat_to_q0',
                'topic': ARM_TOPIC,
                'joint_names': list(arm_retreat['joint_names']),
                'points': [self.point(
                    point['positions'], serialized_duration_ns(point))
                    for point in arm_retreat['points']],
            },
        ]
        closed = gripper_close['points'][-1]['positions']
        closed_error = self.maximum_error(
            joints, gripper_close['joint_names'], closed)
        # A rigid handle can leave one finger short of the commanded preload
        # after the mechanism reaches its locked stop.  Recovery starts at
        # the exact measured finger state, then uses a 0.5 s clamping
        # connector before the certified opening path.  Admit only the
        # observed contact band; this does not widen any arm/path endpoint.
        if closed_error > 0.006:
            raise HandleRecoveryFault(
                f'GRIPPER_NOT_CLAMPED:{closed_error:.9f}')
        audit_messages = [{
            'joint_names': gripper_names,
            'points': [self.point(actual_gripper, 0)],
        }]
        audit_messages.extend({
            'joint_names': message['joint_names'],
            'points': message['points'],
        } for message in physical_messages)
        q0 = plan['stages'][0]['messages'][0]['points'][0]['positions']
        stages = [{
            'name': 'constrained_handle_recovery',
            'messages': audit_messages,
        }]
        for index in range(1, 7):
            stages.append({
                'name': f'post_recovery_q0_{index}',
                'messages': [{
                    'joint_names': list(CONTROLLER_JOINTS[ARM_TOPIC]),
                    'points': [self.point(q0, 0)],
                }],
            })
        maximum_change = 0.0
        previous = None
        for point in arm_points:
            current = np.asarray(point['positions'])
            if previous is not None:
                maximum_change = max(
                    maximum_change,
                    float(np.max(np.abs(current - previous))))
            previous = current
        if maximum_change > MAXIMUM_ADJACENT_CHANGE:
            raise HandleRecoveryFault('RECOVERY_CONTINUITY')
        return {
            'physical_messages': physical_messages,
            'audit_stages': stages,
            'nearest_index': nearest_index,
            'connector_error_rad': connector_error,
            'maximum_adjacent_change_rad': maximum_change,
            'connector_point_count': connector_count,
            'connector_duration_s': connector_duration_ns * 1e-9,
            'recovery_time_scale': recovery_time_scale,
            'gripper_clamp_error_rad': closed_error,
        }

    def run_cpp_audit(self, result_path, result, recovery):
        """Run the existing file-only C++ collision and 10 mm replay."""
        source_directory = result_path.parent
        audit_plan = {
            'scene_epoch_id': result['plan']['scene_epoch_id'],
            'scene_hash': result['plan']['scene_hash'],
            'candidate3_only': True,
            'contains_partial': False,
            'door_arc_target_count': 601,
            'stages': recovery['audit_stages'],
        }
        audit_hash = canonical_hash(audit_plan)
        audit_input = self.run_directory / 'clearance_audit_input.json'
        audit_output = self.run_directory / 'clearance_audit_result.json'
        self.write_exclusive(
            audit_input,
            json.dumps(
                {'plan': audit_plan, 'plan_hash': audit_hash},
                sort_keys=True, indent=2, allow_nan=False) + '\n')
        executable = (
            Path(get_package_prefix('h1_2_moveit_demo')) / 'lib' /
            'h1_2_moveit_demo' / 'candidate3_clearance_auditor')
        completed = subprocess.run(
            [
                str(executable),
                '--scene', str(source_directory / 'scene.cdr'),
                '--plan', str(audit_input),
                '--urdf', str(source_directory / 'clearance_audit_robot.urdf'),
                '--srdf', str(source_directory / 'clearance_audit_robot.srdf'),
                '--output', str(audit_output),
            ],
            stdin=subprocess.DEVNULL, capture_output=True, text=True,
            timeout=120.0, check=False)
        self.write_exclusive(
            self.run_directory / 'clearance_auditor.stdout.log',
            completed.stdout)
        self.write_exclusive(
            self.run_directory / 'clearance_auditor.stderr.log',
            completed.stderr)
        if completed.returncode not in (0, 2) or not audit_output.is_file():
            raise HandleRecoveryFault(
                f'CPP_AUDITOR_EXIT:{completed.returncode}')
        audit = json.loads(audit_output.read_text(encoding='utf-8'))
        os.chmod(audit_output, 0o400)
        if (audit.get('schema') !=
                'h1_candidate3_cpp_clearance_audit/v1' or
                audit.get('plan_hash') != audit_hash or
                audit.get('status') != 'PASS' or
                audit.get('collision_free') is not True or
                audit.get('clearance_passed') is not True or
                float(audit.get('minimum_environment_clearance_m', 0.0)) <
                0.010):
            raise HandleRecoveryFault(
                f'CPP_AUDIT:{audit.get("status", "INVALID")}')
        self.event(
            'RECOVERY_CPP_AUDIT_PASS', audit_hash=audit_hash,
            samples=audit['samples'],
            minimum_environment_clearance_m=(
                audit['minimum_environment_clearance_m']))
        return audit_hash, audit

    @staticmethod
    def trajectory_message(record):
        """Convert a serialized recovery record to a ROS message."""
        message = JointTrajectory()
        message.joint_names = list(record['joint_names'])
        for value in record['points']:
            point = JointTrajectoryPoint()
            point.positions = list(map(float, value['positions']))
            point.time_from_start.sec = int(value['time_from_start']['sec'])
            point.time_from_start.nanosec = int(
                value['time_from_start']['nanosec'])
            message.points.append(point)
        return message

    @staticmethod
    def trajectory_suffix(record, point_index):
        """Return an exact audited suffix with time rebased to zero."""
        if not 0 <= point_index < len(record['points']) - 1:
            raise HandleRecoveryFault('RECOVERY_SUFFIX_INDEX')
        result = copy.deepcopy(record)
        base_ns = serialized_duration_ns(result['points'][point_index])
        result['points'] = result['points'][point_index:]
        for point in result['points']:
            point['time_from_start'] = duration_fields(
                serialized_duration_ns(point) - base_ns)
        return result

    def publish_hold(self):
        """Hold both owned controllers at their measured positions."""
        with self.lock:
            joints = dict(self.joints)
        messages = {}
        for topic in (ARM_TOPIC, GRIPPER_TOPIC):
            names = CONTROLLER_JOINTS[topic]
            if any(name not in joints for name in names):
                return 'HOLD_FAIL:MISSING'
            message = JointTrajectory()
            message.joint_names = list(names)
            point = JointTrajectoryPoint()
            point.positions = [joints[name] for name in names]
            point.time_from_start.nanosec = 100_000_000
            message.points.append(point)
            messages[topic] = message
        for _ in range(3):
            for topic, message in messages.items():
                self.publishers_by_topic[topic].publish(message)
            time.sleep(0.05)
        return 'HOLD_PASS'

    def execute_message(self, plan, record):
        """Execute one audited recovery message with continuous live gates."""
        publisher = self.publishers_by_topic[record['topic']]
        if (publisher.get_subscription_count() != 1 or
                self.count_publishers(record['topic']) != 1):
            raise HandleRecoveryFault('CONTROLLER_TOPOLOGY_CHANGED')
        source_index = 0
        tag_holds = 0
        while True:
            active = (
                record if source_index == 0 else
                self.trajectory_suffix(record, source_index))
            joints, _ = self.snapshot(plan, stable_cabinet=True)
            start_error = self.maximum_error(
                joints, active['joint_names'],
                active['points'][0]['positions'])
            if start_error > 0.003:
                raise HandleRecoveryFault(
                    f'RECOVERY_START:{record["name"]}:{start_error:.9f}')
            message = self.trajectory_message(active)
            duration_ns = serialized_duration_ns(active['points'][-1])
            publish_sim_ns = self.get_clock().now().nanoseconds
            publish_wall_ns = time.monotonic_ns()
            self.motion_started = True
            publisher.publish(message)
            deadline = time.monotonic() + duration_ns * 1e-9 * 10.0 + 3.0
            stable_count = 0
            resume = False
            while time.monotonic() < deadline:
                try:
                    joints, _ = self.snapshot(
                        plan, stable_cabinet=False,
                        stationary_mechanism=False)
                except HandleRecoveryFault as error:
                    if not str(error).startswith('CABINET_TAG'):
                        raise
                    tag_holds += 1
                    if tag_holds > 20 or self.publish_hold() != 'HOLD_PASS':
                        raise HandleRecoveryFault('TAG_HOLD_LIMIT_OR_FAILURE')
                    self.event(
                        'RECOVERY_CABINET_TAG_HOLD', name=record['name'],
                        hold_count=tag_holds)
                    wait_deadline = time.monotonic() + 5.0
                    while time.monotonic() < wait_deadline:
                        try:
                            joints, _ = self.snapshot(
                                plan, stable_cabinet=True,
                                stationary_mechanism=True)
                            break
                        except HandleRecoveryFault as wait_error:
                            if (str(wait_error).startswith('CABINET_TAG') or
                                    str(wait_error).startswith(
                                        'MECHANISM_CONSTRAINT')):
                                time.sleep(0.01)
                                continue
                            raise
                    else:
                        raise HandleRecoveryFault(
                            'TAG_REACQUISITION_TIMEOUT')
                    errors = [
                        self.maximum_error(
                            joints, record['joint_names'], point['positions'])
                        for point in record['points'][source_index:]
                    ]
                    relative = min(
                        range(len(errors)), key=errors.__getitem__)
                    nearest = source_index + relative
                    nearest_error = errors[relative]
                    if nearest_error > 0.003:
                        raise HandleRecoveryFault(
                            f'TAG_RESUME_NOT_EXACT:{nearest_error:.9f}')
                    if nearest >= len(record['points']) - 1:
                        self.event(
                            'RECOVERY_TAG_REACQUIRED_AT_ENDPOINT',
                            name=record['name'], error_rad=nearest_error)
                        return
                    source_index = nearest
                    self.event(
                        'RECOVERY_TAG_REACQUIRED_RESUME_SUFFIX',
                        name=record['name'],
                        source_point_index=source_index,
                        error_rad=nearest_error)
                    resume = True
                    break
                if (self.get_clock().now().nanoseconds - publish_sim_ns >=
                        duration_ns):
                    error = self.maximum_error(
                        joints, record['joint_names'],
                        record['points'][-1]['positions'])
                    if (error <= 0.003 and
                            self.joint_receive_ns > publish_wall_ns):
                        stable_count += 1
                    else:
                        stable_count = 0
                    if stable_count >= 3:
                        self.event(
                            'RECOVERY_TRAJECTORY_PASS', name=record['name'],
                            points=len(record['points']),
                            duration_s=serialized_duration_ns(
                                record['points'][-1]) * 1e-9,
                            endpoint_error_rad=error,
                            tag_occlusion_holds=tag_holds)
                        return
                time.sleep(0.005)
            if resume:
                continue
            raise HandleRecoveryFault(
                f'TRAJECTORY_TIMEOUT:{record["name"]}')

    def wait_mechanism_q0(self):
        """Require the locked mechanism to settle at q0."""
        deadline = time.monotonic() + 3.0
        while time.monotonic() < deadline:
            with self.lock:
                mechanism = dict(self.mechanism)
                velocities = dict(self.mechanism_velocity)
                latch = self.latch
            if (latch == 'locked' and
                    abs(mechanism.get('door_hinge_joint', math.inf)) <=
                    math.radians(0.2) and
                    abs(mechanism.get('handle_latch_joint', math.inf)) <=
                    math.radians(3.0) and
                    abs(velocities.get('handle_latch_joint', math.inf)) <=
                    0.01):
                return
            time.sleep(0.02)
        raise HandleRecoveryFault('MECHANISM_Q0_TIMEOUT')

    def wait_mechanism_release_ready(self):
        """
        Require the locked stop before releasing contact preload.

        At the certified q0 arm endpoint the rigid fingers can hold the
        spring-loaded handle a few degrees above its unloaded q0.  The main
        certified sequence therefore opens before its final mechanism-q0
        checkpoint.  Preserve that ordering here: the latch must already be
        locked, the door closed, the handle stationary and within seven
        degrees; the normal three-degree q0 gate remains mandatory after the
        gripper opens.
        """
        deadline = time.monotonic() + 3.0
        while time.monotonic() < deadline:
            with self.lock:
                mechanism = dict(self.mechanism)
                velocities = dict(self.mechanism_velocity)
                latch = self.latch
            handle = mechanism.get('handle_latch_joint', math.inf)
            if (latch == 'locked' and
                    abs(mechanism.get('door_hinge_joint', math.inf)) <=
                    math.radians(0.2) and
                    -math.radians(1.0) <= handle <= math.radians(7.0) and
                    abs(velocities.get('handle_latch_joint', math.inf)) <=
                    0.01):
                self.event(
                    'MECHANISM_RELEASE_READY',
                    handle_position_rad=handle)
                return
            time.sleep(0.02)
        raise HandleRecoveryFault('MECHANISM_RELEASE_READY_TIMEOUT')

    def run(self):
        """Validate, audit, execute, and seal the constrained recovery."""
        result = None
        source_sha = ''
        recovery = None
        audit_hash = ''
        try:
            if (not bool(self.get_parameter('execute').value) or
                    str(self.get_parameter('execution_mode').value) !=
                    'gazebo'):
                raise HandleRecoveryFault('NOT_EXPLICIT_GAZEBO')
            result_path = Path(str(self.get_parameter('result').value))
            expected_sha = str(
                self.get_parameter('result_sha256').value).lower()
            evidence_root = Path(str(
                self.get_parameter('evidence_output_root').value))
            if (not result_path.is_absolute() or not evidence_root.is_absolute()
                    or len(expected_sha) != 64):
                raise HandleRecoveryFault('CONFIG_PATH_OR_HASH')
            source_sha = hashlib.sha256(result_path.read_bytes()).hexdigest()
            if source_sha != expected_sha:
                raise HandleRecoveryFault('RESULT_SHA256')
            result = load_result_file(result_path, result_path.parent)
            evidence_root.mkdir(parents=True, exist_ok=True)
            self.run_directory = evidence_root / (
                f'run_{time.strftime("%Y%m%dT%H%M%S")}_{uuid.uuid4().hex}')
            self.run_directory.mkdir(mode=0o700)
            self.start_bridge()
            deadline = time.monotonic() + 10.0
            while time.monotonic() < deadline:
                try:
                    joints, mechanism = self.snapshot(
                        result['plan'], stable_cabinet=True)
                    break
                except HandleRecoveryFault:
                    time.sleep(0.02)
            else:
                raise HandleRecoveryFault('TELEMETRY_TIMEOUT')
            self.require_controller_topology()
            recovery = self.build_recovery(result, joints)
            audit_hash, _ = self.run_cpp_audit(
                result_path, result, recovery)
            # Bind the C++-audited connector to the still-held live state.
            joints, current_mechanism = self.snapshot(
                result['plan'], stable_cabinet=True)
            first = recovery['physical_messages'][0]
            live_start_error = self.maximum_error(
                joints, first['joint_names'], first['points'][0]['positions'])
            if live_start_error > 0.003:
                raise HandleRecoveryFault(
                    f'LIVE_STATE_CHANGED:{live_start_error:.9f}')
            if abs(current_mechanism['handle_latch_joint'] -
                   mechanism['handle_latch_joint']) > math.radians(0.5):
                raise HandleRecoveryFault('HANDLE_CHANGED_DURING_AUDIT')
            self.event(
                'RECOVERY_ADMITTED', source_result_sha256=source_sha,
                plan_hash=result['plan_hash'], audit_hash=audit_hash,
                nearest_source_point_index=recovery['nearest_index'],
                connector_error_rad=recovery['connector_error_rad'],
                connector_point_count=recovery['connector_point_count'],
                maximum_adjacent_change_rad=(
                    recovery['maximum_adjacent_change_rad']),
                gripper_clamp_error_rad=(
                    recovery['gripper_clamp_error_rad']))
            for index, message in enumerate(recovery['physical_messages']):
                self.execute_message(result['plan'], message)
                if index == 0:
                    self.wait_mechanism_release_ready()
                elif index == 1:
                    self.wait_mechanism_q0()
            self.wait_mechanism_q0()
            joints, _ = self.snapshot(result['plan'], stable_cabinet=True)
            q0_message = result['plan']['stages'][0]['messages'][0]
            q0_error = self.maximum_error(
                joints, q0_message['joint_names'],
                q0_message['points'][0]['positions'])
            gripper_message = result['plan']['stages'][0]['messages'][1]
            gripper_error = self.maximum_error(
                joints, gripper_message['joint_names'],
                gripper_message['points'][0]['positions'])
            if q0_error > 0.003 or gripper_error > 0.003:
                raise HandleRecoveryFault(
                    f'FINAL_Q0:{q0_error:.9f}:{gripper_error:.9f}')
            self.finish(
                'PASS', result, source_sha, recovery, audit_hash,
                final_q0_error_rad=q0_error,
                final_gripper_error_rad=gripper_error)
        except Exception as error:
            hold = self.publish_hold() if self.motion_started else 'NO_MOTION'
            self.finish(
                'FAIL_CLOSED', result, source_sha, recovery, audit_hash,
                reason=f'{type(error).__name__}:{error};{hold}')

    def finish(self, status, result, source_sha, recovery, audit_hash, **fields):
        """Seal a terminal recovery result."""
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
                        'recovery_audit_hash': audit_hash,
                        'motion_started': self.motion_started,
                        'nearest_source_point_index': (
                            recovery['nearest_index'] if recovery else -1),
                        'connector_error_rad': (
                            recovery['connector_error_rad']
                            if recovery else None),
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
    """Run the explicit Gazebo-only constrained recovery."""
    rclpy.init(args=args)
    node = Candidate3HandleRecovery()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        while rclpy.ok() and not node.done.is_set():
            executor.spin_once(timeout_sec=0.1)
    except KeyboardInterrupt:
        if node.motion_started:
            node.publish_hold()
    finally:
        node.stop_bridge()
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
