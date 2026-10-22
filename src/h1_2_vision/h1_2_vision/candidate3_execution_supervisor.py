"""
Gazebo-only, certificate-gated executor for the complete candidate-3 plan.

The node defaults to disabled.  Explicit ``execute:=true`` and
``execution_mode:=gazebo`` are both required.  It preloads the already-audited
door/cube support plugins before admitting a fresh g5 result, publishes only
the four allow-listed controller topics, and preempts them with measured joint
holds on every runtime anomaly.  A failure never detaches a support, opens a
gripper, resets Gazebo, or changes an object pose.
"""

from __future__ import annotations

import copy
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
from h1_2_perception_msgs.msg import (
    Candidate3Plan,
    PlanCertificate,
    TagObservation,
    TrackedObjectState,
)
import numpy as np
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    qos_profile_sensor_data,
    QoSProfile,
    ReliabilityPolicy,
)
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from .candidate3_contract import canonical_hash, pose_matrix
from .candidate3_execution_contract import (
    CONTROLLER_JOINTS,
    load_result_file,
)
from .candidate3_post_nav_contract import validate_navigation_hold_status
from .tag_config import load_tag_map


SCHEMA = 'h1_candidate3_execution_evidence/v1'
RESULT_STATUS = 'PASS'
DOOR_ASSET_SHA256 = (
    '867548548197ae577580f394051e8118cc0fccefdea1494c1bbb41ccaa4a4a19')
CUBE_ASSET_SHA256 = (
    'acaec97384f16a736f6b246a3fa1a0897b7aad93e72a6204701610ebf4f7e705')
CLOSE_ASSET_SHA256 = (
    '5b212d78bb204fdde047c875d1e2e9585efb18d7d053c847be0a0ef59a4a9038')


class ExecutionFault(RuntimeError):
    """A runtime gate failed after the supervisor was explicitly enabled."""


def stamp_ns(stamp):
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def point_duration_ns(point):
    value = point['time_from_start']
    return int(value['sec']) * 1_000_000_000 + int(value['nanosec'])


def rotation_error(left, right):
    relative = np.asarray(left, dtype=float)[:3, :3].T @ np.asarray(
        right, dtype=float)[:3, :3]
    cosine = (float(np.trace(relative)) - 1.0) * 0.5
    return math.acos(min(1.0, max(-1.0, cosine)))


class Candidate3ExecutionSupervisor(Node):
    """Execute one fresh, complete g5 plan or stop-and-hold fail closed."""

    def __init__(self):
        super().__init__('h1_2_candidate3_execution_supervisor')
        self.declare_parameter('execute', False)
        self.declare_parameter('execution_mode', 'disabled')
        self.declare_parameter('workspace', '')
        self.declare_parameter('planner_output_root', '')
        self.declare_parameter('evidence_output_root', '')
        self.declare_parameter('door_recovery_owner_file', '')
        self.declare_parameter('door_recovery_already_attached', False)
        self.declare_parameter('cube_recovery_preload_file', '')
        self.declare_parameter('cube_recovery_already_attached', False)
        self.declare_parameter('maximum_telemetry_age_s', 0.75)
        self.declare_parameter('maximum_support_heartbeat_age_s', 6.0)
        self.declare_parameter('start_tolerance_rad', 0.003)
        self.declare_parameter('endpoint_tolerance_rad', 0.003)
        self.declare_parameter('cabinet_translation_tolerance_m', 0.003)
        self.declare_parameter(
            'cabinet_rotation_tolerance_rad', math.radians(1.0))
        self.declare_parameter('discovery_timeout_s', 20.0)
        # Gazebo commonly runs at RTF 0.2--0.3 with the D435 and MoveIt
        # monitors enabled.  This is only a wall-clock watchdog; trajectory
        # stamps, certified speed/acceleration limits, and the <=90 s
        # simulation-time action contract remain unchanged.
        self.declare_parameter('trajectory_timeout_factor', 10.0)
        self.declare_parameter('trajectory_timeout_padding_s', 3.0)
        self.declare_parameter('tag_reacquisition_timeout_s', 5.0)
        self.declare_parameter('maximum_tag_occlusion_holds', 20)
        self.declare_parameter('require_post_navigation_alignment', False)
        self.declare_parameter('post_navigation_goal_x', 0.0)
        self.declare_parameter('post_navigation_goal_y', 0.0)
        self.declare_parameter('post_navigation_goal_yaw', 0.0)
        # Deliberately narrow Gazebo recovery entry points.  They are exact
        # phase boundaries or already-supported cube-transport messages, so a
        # fail-closed stop can continue from fresh measured state without
        # reset, teleport, premature detach, or premature gripper release.
        self.declare_parameter('resume_stage_index', -1)
        self.declare_parameter('resume_message_index', -1)

        self.execute_enabled = bool(self.get_parameter('execute').value)
        self.execution_mode = str(
            self.get_parameter('execution_mode').value).strip()
        workspace_text = str(self.get_parameter('workspace').value).strip()
        planner_root_text = str(
            self.get_parameter('planner_output_root').value).strip()
        evidence_root_text = str(
            self.get_parameter('evidence_output_root').value).strip()
        self.workspace = Path(workspace_text) if workspace_text else None
        self.planner_output_root = (
            Path(planner_root_text) if planner_root_text else None)
        self.evidence_output_root = (
            Path(evidence_root_text) if evidence_root_text else None)
        owner_text = str(
            self.get_parameter('door_recovery_owner_file').value).strip()
        self.door_recovery_owner_file = (
            Path(owner_text) if owner_text else None)
        self.door_recovery_already_attached = bool(
            self.get_parameter('door_recovery_already_attached').value)
        cube_preload_text = str(
            self.get_parameter('cube_recovery_preload_file').value).strip()
        self.cube_recovery_preload_file = (
            Path(cube_preload_text) if cube_preload_text else None)
        self.cube_recovery_already_attached = bool(
            self.get_parameter('cube_recovery_already_attached').value)
        self.maximum_age_s = float(
            self.get_parameter('maximum_telemetry_age_s').value)
        self.maximum_support_age_s = float(
            self.get_parameter('maximum_support_heartbeat_age_s').value)
        self.start_tolerance = float(
            self.get_parameter('start_tolerance_rad').value)
        self.endpoint_tolerance = float(
            self.get_parameter('endpoint_tolerance_rad').value)
        self.cabinet_translation_tolerance = float(
            self.get_parameter('cabinet_translation_tolerance_m').value)
        self.cabinet_rotation_tolerance = float(
            self.get_parameter('cabinet_rotation_tolerance_rad').value)
        self.discovery_timeout = float(
            self.get_parameter('discovery_timeout_s').value)
        self.timeout_factor = float(
            self.get_parameter('trajectory_timeout_factor').value)
        self.timeout_padding = float(
            self.get_parameter('trajectory_timeout_padding_s').value)
        self.tag_reacquisition_timeout = float(
            self.get_parameter('tag_reacquisition_timeout_s').value)
        self.maximum_tag_occlusion_holds = int(
            self.get_parameter('maximum_tag_occlusion_holds').value)
        self.require_post_navigation_alignment = bool(
            self.get_parameter('require_post_navigation_alignment').value)
        self.post_navigation_goal = tuple(float(
            self.get_parameter(name).value) for name in (
                'post_navigation_goal_x', 'post_navigation_goal_y',
                'post_navigation_goal_yaw'))
        self.resume_stage_index = int(
            self.get_parameter('resume_stage_index').value)
        self.resume_message_index = int(
            self.get_parameter('resume_message_index').value)
        self.resume_enabled = self.resume_stage_index >= 0

        self.lock = threading.Lock()
        self.joint_positions = {}
        self.joint_receive_ns = 0
        self.mechanism_positions = {}
        self.mechanism_velocities = {}
        self.mechanism_receive_ns = 0
        self.cabinet = None
        self.cabinet_receive_ns = 0
        self.raw_cabinet_pose = None
        self.raw_cabinet_stamp_ns = 0
        self.raw_cabinet_receive_ns = 0
        self.raw_cabinet_tag_id = -1
        self.raw_cabinet_observation_count = 0
        self.navigation_status = None
        self.navigation_status_receive_ns = 0
        self.latch_state = ''
        self.latch_receive_ns = 0
        self.right_support_state = ''
        self.right_support_receive_ns = 0
        self.left_support_state = ''
        self.left_support_receive_ns = 0
        self.plan_messages = {}
        self.certificate_messages = {}
        self.ready = False
        self.motion_started = False
        self.execution_started = False
        self.fault_requested = ''
        self.done = threading.Event()
        self.start_monotonic_ns = time.monotonic_ns()
        self.armed_monotonic_ns = 0
        self.events = []
        self.run_directory = None
        self.final_status = ''
        self.support_bridge_process = None
        self.support_bridge_streams = []
        self.active_result_path = None
        self.tag_resume_audit_count = 0
        self.resume_record = None
        self.resume_source_index = 0

        vision_share = Path(get_package_share_directory('h1_2_vision'))
        description_share = Path(
            get_package_share_directory('h1_2_description'))
        self.declare_parameter(
            'tag_map', str(vision_share / 'config/tag_map.yaml'))
        self.tag_config = load_tag_map(
            Path(self.get_parameter('tag_map').value), [
                description_share / 'models/incubator/materials/textures',
                description_share / 'models/green_cube/materials/textures',
            ])

        volatile = QoSProfile(
            depth=10, reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE)
        transient = QoSProfile(
            depth=1, reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.trajectory_publishers = {
            topic: self.create_publisher(JointTrajectory, topic, volatile)
            for topic in CONTROLLER_JOINTS
        }
        self.status_pub = self.create_publisher(
            String, '/candidate3_execution/status', transient)
        self.create_subscription(
            JointState, '/joint_states', self.joint_callback,
            qos_profile_sensor_data)
        self.create_subscription(
            JointState, '/incubator/joint_states', self.mechanism_callback,
            qos_profile_sensor_data)
        self.create_subscription(
            TrackedObjectState, '/tag_pose/cabinet', self.cabinet_callback, 10)
        self.create_subscription(
            String, '/h1_2_navigation/incubator_navigation/status',
            self.navigation_status_callback, transient)
        self.create_subscription(
            TagObservation, '/apriltag/detections',
            self.tag_observation_callback, 20)
        self.create_subscription(
            String, '/incubator/latch/state', self.latch_callback, 10)
        # The ros_gz support-state bridges are lazy.  Creating these ROS
        # subscriptions before preload would create a Gazebo transport
        # endpoint and correctly make the mature helpers reject their exact
        # empty-topic precondition.  Enable them only after both detached
        # plugins have been installed and proved.
        self.support_subscriptions = []
        self.create_subscription(
            Candidate3Plan, '/candidate3_plan', self.plan_callback, transient)
        self.create_subscription(
            PlanCertificate, '/plan_certificate',
            self.certificate_callback, transient)
        self.create_subscription(
            String, '/candidate3_replan/status',
            self.replan_status_callback, transient)
        self.create_subscription(
            String, '/candidate3_execution/stop', self.stop_callback, 10)
        self.bootstrap_thread = threading.Thread(
            target=self.bootstrap, daemon=True)
        self.bootstrap_thread.start()

    def event(self, status, **fields):
        record = {
            'monotonic_ns': time.monotonic_ns(),
            'ros_time_ns': self.get_clock().now().nanoseconds,
            'status': status,
            **fields,
        }
        with self.lock:
            self.events.append(record)
        self.get_logger().info(json.dumps(record, sort_keys=True))

    def publish_status(self, status, **fields):
        message = String()
        message.data = json.dumps(
            {'schema': SCHEMA, 'status': status, **fields},
            sort_keys=True, separators=(',', ':'))
        self.status_pub.publish(message)

    def joint_callback(self, message):
        if len(message.name) != len(message.position):
            return
        positions = {}
        for name, value in zip(message.name, message.position):
            if name in positions or not math.isfinite(float(value)):
                return
            positions[name] = float(value)
        with self.lock:
            self.joint_positions = positions
            self.joint_receive_ns = time.monotonic_ns()

    def mechanism_callback(self, message):
        if (len(message.name) != len(message.position) or
                len(message.name) != len(message.velocity)):
            return
        positions = {}
        velocities = {}
        for name, position, velocity in zip(
                message.name, message.position, message.velocity):
            if (name in positions or not math.isfinite(float(position)) or
                    not math.isfinite(float(velocity))):
                return
            positions[name] = float(position)
            velocities[name] = float(velocity)
        with self.lock:
            self.mechanism_positions = positions
            self.mechanism_velocities = velocities
            self.mechanism_receive_ns = time.monotonic_ns()

    def cabinet_callback(self, message):
        with self.lock:
            self.cabinet = message
            self.cabinet_receive_ns = time.monotonic_ns()

    def navigation_status_callback(self, message):
        try:
            value = json.loads(message.data)
        except (TypeError, ValueError, json.JSONDecodeError):
            value = None
        with self.lock:
            self.navigation_status = value
            self.navigation_status_receive_ns = time.monotonic_ns()

    def tag_observation_callback(self, message):
        """
        Keep a quality-gated per-frame cabinet pose for motion monitoring.

        Admission still requires the fused cabinet state to be stable.  During
        motion, a robot arm can temporarily break the seven-frame stability
        window even while an individual cabinet tag remains valid.  The raw
        observation is converted with the same hashed tag-map transforms as
        the localizer, and the normal 3 mm / 1 degree live-pose gate remains
        in force.
        """
        record = self.tag_config['tags_by_id'].get(int(message.id))
        if (record is None or record['object_name'] != 'cabinet' or
                message.object_name != 'cabinet' or
                message.family != self.tag_config['family'] or
                message.world_frame != 'world' or
                message.config_hash != self.tag_config['config_hash'] or
                not message.valid or message.status != 'VALID'):
            return
        try:
            world_from_tag = pose_matrix(message.pose_world.pose)
            world_from_object = (
                world_from_tag @ np.linalg.inv(record['object_from_tag']) @
                self.tag_config['objects']['cabinet'][
                    'object_pose_correction'])
        except (ValueError, np.linalg.LinAlgError):
            return
        observed_stamp_ns = stamp_ns(message.header.stamp)
        if observed_stamp_ns <= 0:
            return
        with self.lock:
            self.raw_cabinet_pose = world_from_object
            self.raw_cabinet_stamp_ns = observed_stamp_ns
            self.raw_cabinet_receive_ns = time.monotonic_ns()
            self.raw_cabinet_tag_id = int(message.id)
            self.raw_cabinet_observation_count += 1

    def latch_callback(self, message):
        with self.lock:
            self.latch_state = message.data.strip().lower()
            self.latch_receive_ns = time.monotonic_ns()

    def right_support_callback(self, message):
        with self.lock:
            self.right_support_state = message.data.strip().lower()
            self.right_support_receive_ns = time.monotonic_ns()

    def left_support_callback(self, message):
        with self.lock:
            self.left_support_state = message.data.strip().lower()
            self.left_support_receive_ns = time.monotonic_ns()

    @staticmethod
    def key(message):
        return int(message.scene_epoch_id), message.scene_hash, message.plan_hash

    def plan_callback(self, message):
        with self.lock:
            self.plan_messages[self.key(message)] = (
                message, time.monotonic_ns())
            self.plan_messages = {self.key(message): self.plan_messages[
                self.key(message)]}

    def certificate_callback(self, message):
        with self.lock:
            self.certificate_messages[self.key(message)] = (
                message, time.monotonic_ns())
            self.certificate_messages = {self.key(message):
                                         self.certificate_messages[
                                             self.key(message)]}

    def stop_callback(self, message):
        value = message.data.strip()
        if value:
            with self.lock:
                self.fault_requested = f'EXTERNAL_STOP:{value}'

    def replan_status_callback(self, message):
        received_ns = time.monotonic_ns()
        try:
            value = json.loads(message.data)
        except json.JSONDecodeError:
            return
        if (value.get('status') != RESULT_STATUS or
                value.get('valid') is not True or not value.get('result')):
            return
        with self.lock:
            if (not self.ready or self.execution_started or
                    received_ns < self.armed_monotonic_ns):
                return
            self.execution_started = True
        thread = threading.Thread(
            target=self.execute_result, args=(value, received_ns), daemon=True)
        thread.start()

    def validate_configuration(self):
        if not self.execute_enabled:
            raise ExecutionFault('EXECUTE_FALSE')
        if self.execution_mode != 'gazebo':
            raise ExecutionFault('EXECUTION_MODE_NOT_GAZEBO')
        for value, name in (
                (self.workspace, 'workspace'),
                (self.planner_output_root, 'planner_output_root'),
                (self.evidence_output_root, 'evidence_output_root')):
            if value is None or not value.is_absolute():
                raise ExecutionFault(f'CONFIG_PATH:{name}')
        self.workspace = self.workspace.resolve(strict=True)
        self.planner_output_root.mkdir(parents=True, exist_ok=True)
        self.planner_output_root = self.planner_output_root.resolve(strict=True)
        self.evidence_output_root.mkdir(parents=True, exist_ok=True)
        self.evidence_output_root = self.evidence_output_root.resolve(strict=True)
        if any(
                not math.isfinite(value) or value <= 0.0 for value in (
                    self.maximum_age_s, self.maximum_support_age_s,
                    self.start_tolerance,
                    self.endpoint_tolerance,
                    self.cabinet_translation_tolerance,
                    self.cabinet_rotation_tolerance,
                    self.discovery_timeout, self.timeout_factor,
                    self.timeout_padding,
                    self.tag_reacquisition_timeout)):
            raise ExecutionFault('CONFIG_NUMERIC')
        if not all(math.isfinite(value)
                   for value in self.post_navigation_goal):
            raise ExecutionFault('CONFIG_POST_NAVIGATION_GOAL')
        if (self.start_tolerance > 0.003 or
                self.endpoint_tolerance > 0.003 or
                self.cabinet_translation_tolerance > 0.003 or
                self.cabinet_rotation_tolerance > math.radians(1.0) or
                self.maximum_support_age_s > 10.0 or
                self.tag_reacquisition_timeout > 5.0 or
                self.timeout_factor > 10.0 or
                not 1 <= self.maximum_tag_occlusion_holds <= 20):
            raise ExecutionFault('CONFIG_TOLERANCE_WIDENING')
        if ((self.resume_stage_index, self.resume_message_index) not in
                ((-1, -1), (1, 0), (2, 3), (3, 2), (3, 3), (6, 1))):
            raise ExecutionFault('CONFIG_RESUME_ENTRY_POINT')
        if (self.resume_stage_index, self.resume_message_index) == (2, 3):
            if (self.door_recovery_owner_file is None or
                    not self.door_recovery_owner_file.is_absolute() or
                    not self.door_recovery_owner_file.is_file() or
                    self.door_recovery_owner_file.is_symlink()):
                raise ExecutionFault('CONFIG_DOOR_RECOVERY_OWNER')
            self.door_recovery_owner_file = (
                self.door_recovery_owner_file.resolve(strict=True))
        elif self.door_recovery_already_attached:
            raise ExecutionFault('CONFIG_DOOR_ALREADY_ATTACHED_SCOPE')
        if (self.resume_stage_index, self.resume_message_index) == (3, 2):
            if (self.cube_recovery_preload_file is None or
                    not self.cube_recovery_preload_file.is_absolute() or
                    not self.cube_recovery_preload_file.is_file() or
                    self.cube_recovery_preload_file.is_symlink()):
                raise ExecutionFault('CONFIG_CUBE_RECOVERY_PRELOAD')
            self.cube_recovery_preload_file = (
                self.cube_recovery_preload_file.resolve(strict=True))
        elif self.cube_recovery_already_attached:
            raise ExecutionFault('CONFIG_CUBE_ALREADY_ATTACHED_SCOPE')
        for _ in range(10):
            path = self.evidence_output_root / (
                f'run_{time.strftime("%Y%m%dT%H%M%S")}_{uuid.uuid4().hex}')
            try:
                path.mkdir(mode=0o700)
                self.run_directory = path
                break
            except FileExistsError:
                continue
        if self.run_directory is None:
            raise ExecutionFault('EVIDENCE_DIRECTORY')

    def bootstrap(self):
        try:
            self.validate_configuration()
            self.publish_status('BOOTSTRAP_GAZEBO_ONLY')
            self.event('BOOTSTRAP_GAZEBO_ONLY')
            if self.resume_enabled:
                self.wait_for_recovery_inputs()
            else:
                self.wait_for_preload_inputs()
            self.require_controller_contract()
            if not self.resume_enabled:
                self.run_support_helper('door_preload', 'door', 'preload')
                self.run_support_helper('cube_preload', 'cube', 'preload')
            self.start_support_bridge()
            self.enable_support_subscriptions()
            expected_right = (
                'attached' if (
                    self.resume_stage_index == 3 or
                    ((self.resume_stage_index, self.resume_message_index) ==
                     (2, 3) and self.door_recovery_already_attached))
                else 'detached')
            expected_left = (
                'attached' if (
                    (self.resume_stage_index, self.resume_message_index) ==
                    (3, 2) and self.cube_recovery_already_attached)
                else 'detached')
            self.wait_support('right', expected_right, 5.0)
            self.wait_support('left', expected_left, 5.0)
            with self.lock:
                self.ready = True
                self.armed_monotonic_ns = time.monotonic_ns()
            self.event(
                'ARMED_WAITING_FRESH_PLAN', resume=self.resume_enabled,
                resume_stage_index=self.resume_stage_index,
                resume_message_index=self.resume_message_index)
            self.publish_status(
                'ARMED_WAITING_FRESH_PLAN', gazebo_only=True,
                resume=self.resume_enabled)
        except Exception as error:
            self.finalize_failure(f'BOOTSTRAP:{type(error).__name__}:{error}')

    def wait_for_preload_inputs(self):
        deadline = time.monotonic() + self.discovery_timeout
        required = {name for names in CONTROLLER_JOINTS.values()
                    for name in names}
        while time.monotonic() < deadline:
            with self.lock:
                positions = dict(self.joint_positions)
                joint_age = time.monotonic_ns() - self.joint_receive_ns
                mechanism = dict(self.mechanism_positions)
                mechanism_age = time.monotonic_ns() - self.mechanism_receive_ns
                latch = self.latch_state
                latch_age = time.monotonic_ns() - self.latch_receive_ns
            clock_ready = (
                bool(getattr(self.get_clock(), 'ros_time_is_active', False)) and
                self.get_clock().now().nanoseconds > 0)
            if (clock_ready and required <= positions.keys() and
                    joint_age <= int(self.maximum_age_s * 1e9) and
                    {'door_hinge_joint', 'handle_latch_joint'} <=
                    mechanism.keys() and
                    mechanism_age <= int(self.maximum_age_s * 1e9) and
                    latch == 'locked' and
                    latch_age <= int(self.maximum_age_s * 1e9)):
                if (abs(mechanism['door_hinge_joint']) > math.radians(0.2) or
                        abs(mechanism['handle_latch_joint']) >
                        math.radians(3.0)):
                    raise ExecutionFault('INITIAL_MECHANISM_NOT_Q0_LOCKED')
                return
            time.sleep(0.02)
        raise ExecutionFault('PRELOAD_INPUT_TIMEOUT')

    def wait_for_recovery_inputs(self):
        """Require the exact supported stage-3 stop state before bridging."""
        deadline = time.monotonic() + self.discovery_timeout
        required = {name for names in CONTROLLER_JOINTS.values()
                    for name in names}
        while time.monotonic() < deadline:
            with self.lock:
                positions = dict(self.joint_positions)
                joint_age = time.monotonic_ns() - self.joint_receive_ns
                mechanism = dict(self.mechanism_positions)
                velocities = dict(self.mechanism_velocities)
                mechanism_age = time.monotonic_ns() - self.mechanism_receive_ns
                latch = self.latch_state
                latch_age = time.monotonic_ns() - self.latch_receive_ns
            clock_ready = (
                bool(getattr(self.get_clock(), 'ros_time_is_active', False)) and
                self.get_clock().now().nanoseconds > 0)
            if (clock_ready and required <= positions.keys() and
                    joint_age <= int(self.maximum_age_s * 1e9) and
                    {'door_hinge_joint', 'handle_latch_joint'} <=
                    mechanism.keys() and
                    {'door_hinge_joint', 'handle_latch_joint'} <=
                    velocities.keys() and
                    mechanism_age <= int(self.maximum_age_s * 1e9) and
                    latch_age <= int(self.maximum_age_s * 1e9)):
                resume_entry = (
                    self.resume_stage_index, self.resume_message_index)
                if resume_entry == (6, 1):
                    if (latch != 'locked' or
                            abs(mechanism['door_hinge_joint']) >
                            math.radians(0.2) or
                            not -math.radians(1.0) <=
                            mechanism['handle_latch_joint'] <=
                            math.radians(7.0) or
                            abs(velocities['door_hinge_joint']) > 0.01 or
                            abs(velocities['handle_latch_joint']) > 0.01):
                        raise ExecutionFault(
                            'RECOVERY_MECHANISM_NOT_LOCKED_RELEASE_READY')
                elif resume_entry in ((1, 0), (2, 3)):
                    if (latch != 'unlocked' or
                            abs(mechanism['door_hinge_joint']) >
                            math.radians(0.2) or
                            abs(mechanism['handle_latch_joint'] -
                                math.pi / 2.0) > math.radians(3.0) or
                            abs(velocities['door_hinge_joint']) > 0.01 or
                            abs(velocities['handle_latch_joint']) > 0.01):
                        raise ExecutionFault(
                            'RECOVERY_MECHANISM_NOT_UNLOCKED_HANDLE_ENDPOINT')
                else:
                    if (latch != 'unlocked' or
                            abs(mechanism['door_hinge_joint'] -
                                math.pi / 3.0) > math.radians(0.5) or
                            abs(mechanism['handle_latch_joint'] -
                                math.pi / 2.0) > math.radians(3.0) or
                            abs(velocities['door_hinge_joint']) > 0.01 or
                            abs(velocities['handle_latch_joint']) > 0.01):
                        raise ExecutionFault(
                            'RECOVERY_MECHANISM_NOT_OPEN_SUPPORTED')
                return
            time.sleep(0.02)
        raise ExecutionFault('RECOVERY_INPUT_TIMEOUT')

    def enable_support_subscriptions(self):
        """Turn on the lazy ROS bridges only after exact empty-topic preload."""
        if self.support_subscriptions:
            raise ExecutionFault('SUPPORT_SUBSCRIPTIONS_ALREADY_ENABLED')
        self.support_subscriptions = [
            self.create_subscription(
                String, '/h1_2/right_door_support/state',
                self.right_support_callback, 10),
            self.create_subscription(
                String, '/h1_2/left_pose_grasp/state',
                self.left_support_callback, 10),
        ]

    def start_support_bridge(self):
        """Start only the two deferred GZ-to-ROS support-state bridges."""
        if self.support_bridge_process is not None:
            raise ExecutionFault('SUPPORT_BRIDGE_ALREADY_STARTED')
        executable = (Path(get_package_prefix('ros_gz_bridge')) / 'lib' /
                      'ros_gz_bridge' / 'parameter_bridge')
        if not executable.is_file() or not os.access(executable, os.X_OK):
            raise ExecutionFault('SUPPORT_BRIDGE_EXECUTABLE')
        descriptors = []
        try:
            for name in ('support_bridge.stdout.log',
                         'support_bridge.stderr.log'):
                descriptors.append(os.open(
                    self.run_directory / name,
                    os.O_WRONLY | os.O_CREAT | os.O_EXCL | os.O_NOFOLLOW,
                    0o400))
            self.support_bridge_process = subprocess.Popen(
                [
                    str(executable),
                    '/h1_2/right_door_support/state@'
                    'std_msgs/msg/String[gz.msgs.StringMsg',
                    '/h1_2/left_pose_grasp/state@'
                    'std_msgs/msg/String[gz.msgs.StringMsg',
                    '--ros-args', '-p', 'use_sim_time:=true',
                ],
                stdin=subprocess.DEVNULL,
                stdout=descriptors[0], stderr=descriptors[1],
                close_fds=True)
            self.support_bridge_streams = descriptors
        except Exception:
            for descriptor in descriptors:
                try:
                    os.close(descriptor)
                except OSError:
                    pass
            raise
        for descriptor in descriptors:
            os.close(descriptor)
        self.support_bridge_streams = []
        time.sleep(0.2)
        if self.support_bridge_process.poll() is not None:
            raise ExecutionFault('SUPPORT_BRIDGE_EARLY_EXIT')
        self.event('SUPPORT_BRIDGE_STARTED',
                   pid=self.support_bridge_process.pid)

    def stop_support_bridge(self):
        """Stop the read-only bridge after the supervisor reaches a terminal state."""
        process = self.support_bridge_process
        if process is None or process.poll() is not None:
            return
        process.terminate()
        try:
            process.wait(timeout=3.0)
        except subprocess.TimeoutExpired:
            process.kill()
            process.wait(timeout=3.0)

    def require_controller_contract(self):
        deadline = time.monotonic() + self.discovery_timeout
        while time.monotonic() < deadline:
            if all(
                    publisher.get_subscription_count() == 1 and
                    self.count_publishers(topic) == 1
                    for topic, publisher in self.trajectory_publishers.items()):
                break
            time.sleep(0.05)
        else:
            raise ExecutionFault('CONTROLLER_TOPIC_TOPOLOGY')
        completed = subprocess.run(
            ['ros2', 'control', 'list_controllers'],
            stdin=subprocess.DEVNULL, capture_output=True, text=True,
            timeout=10.0, check=False)
        if completed.returncode != 0:
            raise ExecutionFault('CONTROLLER_LIST')
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
            raise ExecutionFault(f'CONTROLLERS_ACTIVE:{sorted(active)}')
        self.event('CONTROLLER_CONTRACT_PASS', active=sorted(active))

    def helper_paths(self):
        source = self.workspace / 'src/h1_2_moveit_demo/scripts'
        assets = self.workspace / 'runtime_assets'
        snapshot = (self.workspace / 'install/h1_2_moveit_demo/lib/'
                    'h1_2_moveit_demo/h1_ros_snapshot_8251.py')
        return {
            'door_helper': source / 'h1_door_support_8251.sh',
            'cube_helper': source / 'h1_cube_support_8251.sh',
            'door_detach_helper': source / 'h1_door_support_detach_8253.sh',
            'door_asset': assets / 'full_robot_flow_8251/door.trajectory.json',
            'cube_asset': (assets / 'parallel_cube_8262/'
                           'cube.parallel90.neutral.trajectory.json'),
            'close_asset': (assets / 'close_and_relock_8253/'
                            'door_close.trajectory.json'),
            'snapshot': snapshot,
        }

    def run_support_helper(self, label, helper, operation):
        paths = self.helper_paths()
        environment = dict(os.environ)
        environment['LC_ALL'] = 'C'
        environment.update({
            'H1_FULL_WORKSPACE': str(self.workspace),
            'H1_FULL_ROS_SNAPSHOT_TOOL': str(paths['snapshot']),
            'H1_DOOR_ASSET': str(paths['door_asset']),
            'H1_DOOR_ASSET_SHA256': DOOR_ASSET_SHA256,
            'H1_DOOR_ATTACH_AFTER_INDEX': '6',
            'H1_CUBE_ASSET': str(paths['cube_asset']),
            'H1_CUBE_ASSET_SHA256': CUBE_ASSET_SHA256,
            'H1_CUBE_ATTACH_AFTER_INDEX': '2',
            'H1_CUBE_DETACH_AFTER_INDEX': '4',
            'H1_CUBE_RACK_EXPECTED_YAW': '90',
            'H1_CLOSE_WORKSPACE': str(self.workspace),
            'H1_CLOSE_ROS_SNAPSHOT_TOOL': str(paths['snapshot']),
            'H1_CLOSE_DOOR_ASSET': str(paths['close_asset']),
            'H1_CLOSE_DOOR_ASSET_SHA256': CLOSE_ASSET_SHA256,
            'H1_CLOSE_DOOR_DETACH_AFTER_INDEX': '7',
            'H1_CLOSE_SKIP_PRE_TRANSPORT': '1',
            'H1_CLOSE_SKIP_POST_TRANSPORT': '1',
        })
        if operation in ('attach', 'detach',
                         'recover_attach', 'recover_detach'):
            environment['H1_CUBE_SUPPORT_FAST_RESIDENT'] = '1'
        if helper == 'door' and operation in ('attach', 'recover_attach'):
            environment['H1_DOOR_SUPPORT_FAST_RESIDENT'] = '1'
        if helper == 'door' and operation == 'recover_attach':
            environment['H1_DOOR_RECOVERY_OWNER_FILE'] = str(
                self.door_recovery_owner_file)
        if helper == 'cube' and operation in (
                'recover_attach', 'recover_detach'):
            environment['H1_CUBE_RECOVERY_PRELOAD_FILE'] = str(
                self.cube_recovery_preload_file)
        helper_path = paths[
            {'door': 'door_helper', 'cube': 'cube_helper',
             'door_detach': 'door_detach_helper'}[helper]]
        for path in (helper_path, paths['snapshot']):
            if not path.is_file() or not os.access(path, os.X_OK):
                raise ExecutionFault(f'HELPER_UNAVAILABLE:{path}')
        started = time.monotonic_ns()
        completed = subprocess.run(
            [str(helper_path), operation], env=environment,
            stdin=subprocess.DEVNULL, capture_output=True, text=True,
            timeout=60.0, check=False)
        self.write_exclusive(
            self.run_directory / f'{label}.stdout.log', completed.stdout)
        self.write_exclusive(
            self.run_directory / f'{label}.stderr.log', completed.stderr)
        if completed.returncode != 0:
            raise ExecutionFault(
                f'HELPER:{label}:exit={completed.returncode}')
        self.event('SUPPORT_HELPER_PASS', helper=label,
                   duration_s=(time.monotonic_ns() - started) * 1e-9)

    @staticmethod
    def write_exclusive(path, payload):
        descriptor = os.open(
            path, os.O_WRONLY | os.O_CREAT | os.O_EXCL | os.O_NOFOLLOW, 0o400)
        try:
            with os.fdopen(descriptor, 'w', encoding='utf-8') as stream:
                stream.write(payload)
                if payload and not payload.endswith('\n'):
                    stream.write('\n')
        except Exception:
            try:
                os.close(descriptor)
            except OSError:
                pass
            raise

    def wait_support(self, side, expected, timeout_s):
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            with self.lock:
                if side == 'right':
                    value = self.right_support_state
                    received = self.right_support_receive_ns
                else:
                    value = self.left_support_state
                    received = self.left_support_receive_ns
            if (value == expected and received and
                    time.monotonic_ns() - received <=
                    int(self.maximum_age_s * 1e9)):
                return
            time.sleep(0.02)
        raise ExecutionFault(f'SUPPORT_STATE:{side}:{expected}')

    def matched_messages(self, key):
        """Wait briefly for all transient-local metadata from the same publish."""
        deadline = time.monotonic() + 1.0
        while time.monotonic() < deadline:
            with self.lock:
                plan_entry = self.plan_messages.get(key)
                certificate_entry = self.certificate_messages.get(key)
            if plan_entry is not None and certificate_entry is not None:
                return plan_entry, certificate_entry
            time.sleep(0.005)
        raise ExecutionFault('MISSING_MATCHED_ROS_METADATA')

    def validate_ros_metadata(self, result, plan_message, certificate_message):
        plan = result['plan']
        certificate = result['certificate']
        if any((
                not plan_message.valid,
                plan_message.status != 'PASS',
                not plan_message.candidate3_only,
                plan_message.contains_partial,
                plan_message.topology != plan['topology'],
                int(plan_message.door_arc_target_count) != 601,
                int(plan_message.stage_count) != 7,
                list(plan_message.stage_names) != plan['stage_names'],
                list(plan_message.stage_trajectory_hashes) !=
                plan['stage_hashes'],
                list(plan_message.stage_waypoint_counts) !=
                plan['stage_waypoint_counts'],
                not certificate_message.valid,
                certificate_message.status != 'PASS',
                certificate_message.certificate_hash !=
                result['certificate_hash'],
        )):
            raise ExecutionFault('ROS_METADATA_CONTRACT')
        for field in (
                'candidate3_only', 'ik_complete', 'collision_free',
                'original_stl_contact_passed', 'hinge_contract_passed',
                'forward_passed', 'reverse_passed', 'timing_passed',
                'support_topology_passed', 'no_partial'):
            if not bool(getattr(certificate_message, field)):
                raise ExecutionFault(f'ROS_CERTIFICATE:{field}')
        validity_ns = (int(certificate_message.validity.sec) * 1_000_000_000 +
                       int(certificate_message.validity.nanosec))
        age_ns = (self.get_clock().now().nanoseconds -
                  stamp_ns(certificate_message.header.stamp))
        if validity_ns <= 0 or age_ns < 0 or age_ns > validity_ns:
            raise ExecutionFault(
                f'CERTIFICATE_EXPIRED:age_ns={age_ns}:validity_ns={validity_ns}')
        if abs(float(certificate_message.minimum_environment_clearance_m) -
               float(certificate['minimum_environment_clearance_m'])) > 1e-12:
            raise ExecutionFault('ROS_CERTIFICATE_CLEARANCE')

    def execute_result(self, status, status_receive_ns):
        result = None
        try:
            result_path = Path(status['result'])
            if status_receive_ns < self.armed_monotonic_ns:
                raise ExecutionFault('STALE_STATUS_RECEIPT')
            result = load_result_file(result_path, self.planner_output_root)
            key = (
                int(result['plan']['scene_epoch_id']),
                result['plan']['scene_hash'], result['plan_hash'])
            (plan_message, plan_receive_ns), (
                certificate_message, certificate_receive_ns) = (
                    self.matched_messages(key))
            if min(plan_receive_ns, certificate_receive_ns) < self.armed_monotonic_ns:
                raise ExecutionFault('STALE_ROS_METADATA')
            if (status.get('scene_epoch_id'), status.get('scene_hash'),
                    status.get('plan_hash')) != key:
                raise ExecutionFault('STATUS_RESULT_BINDING')
            if status.get('certificate_hash') != result['certificate_hash']:
                raise ExecutionFault('STATUS_CERTIFICATE_BINDING')
            self.validate_ros_metadata(
                result, plan_message, certificate_message)
            if not all(
                    publisher.get_subscription_count() == 1 and
                    self.count_publishers(topic) == 1
                    for topic, publisher in
                    self.trajectory_publishers.items()):
                raise ExecutionFault('CONTROLLER_TOPIC_TOPOLOGY_CHANGED')
            self.active_result_path = result_path
            if self.resume_enabled:
                self.pre_motion_resume_gate(result)
            else:
                self.pre_motion_gate(result)
            # Recheck the short certificate window immediately before the
            # first publication.  It only authorizes starting this complete
            # run; subsequent safety is governed by live telemetry.
            self.validate_ros_metadata(
                result, plan_message, certificate_message)
            self.event(
                'EXECUTION_ADMITTED', plan_hash=result['plan_hash'],
                certificate_hash=result['certificate_hash'],
                result=str(result_path))
            self.publish_status(
                'EXECUTING', plan_hash=result['plan_hash'], gazebo_only=True)
            if self.resume_enabled:
                self.execute_remaining_plan(
                    result['plan'], result['certificate'])
            else:
                self.execute_plan(result['plan'], result['certificate'])
            self.finalize_success(result, result_path)
        except Exception as error:
            reason = f'{type(error).__name__}:{error}'
            if self.motion_started:
                hold_status = self.publish_measured_hold()
                reason = f'{reason};{hold_status}'
            self.finalize_failure(reason, result=result)

    def pre_motion_gate(self, result):
        plan = result['plan']
        certificate = result['certificate']
        if self.require_post_navigation_alignment:
            alignment_hash = plan.get('post_nav_alignment_hash', '')
            if (not isinstance(alignment_hash, str) or
                    len(alignment_hash) != 64 or any(
                        value not in '0123456789abcdef'
                        for value in alignment_hash)):
                raise ExecutionFault('POST_NAV_ALIGNMENT_CERTIFICATE')
            with self.lock:
                navigation_status = copy.deepcopy(self.navigation_status)
                navigation_receive_ns = self.navigation_status_receive_ns
            age_ns = time.monotonic_ns() - navigation_receive_ns
            if (navigation_receive_ns <= 0 or age_ns < 0 or
                    age_ns > int(self.maximum_age_s * 1.0e9)):
                raise ExecutionFault('POST_NAVIGATION_HOLD_STALE')
            try:
                validate_navigation_hold_status(
                    navigation_status,
                    expected_goal_pose=self.post_navigation_goal)
            except (TypeError, ValueError) as error:
                raise ExecutionFault(
                    f'POST_NAVIGATION_HOLD:{error}') from error
        self.runtime_gate(plan, certificate, require_cabinet=True)
        self.wait_support('right', 'detached', 1.0)
        self.wait_support('left', 'detached', 1.0)
        self.wait_latch('locked', 1.0)
        self.require_close_to_start(plan['stages'][0]['messages'][0])
        self.require_close_to_start(plan['stages'][2]['messages'][0])
        with self.lock:
            mechanism = dict(self.mechanism_positions)
        if (abs(mechanism.get('door_hinge_joint', math.inf)) >
                math.radians(0.2) or
                abs(mechanism.get('handle_latch_joint', math.inf)) >
                math.radians(3.0)):
            raise ExecutionFault('PRE_MOTION_MECHANISM')

    def pre_motion_resume_gate(self, result):
        """Admit only the exact, still-supported cube-transport stop state."""
        plan = result['plan']
        certificate = result['certificate']
        resume_entry = (
            self.resume_stage_index, self.resume_message_index)
        self.runtime_gate(plan, certificate, require_cabinet=True)
        expected_right = (
            'attached' if (
                self.resume_stage_index == 3 or
                (resume_entry == (2, 3) and
                 self.door_recovery_already_attached))
            else 'detached')
        self.wait_support('right', expected_right, 1.0)
        expected_left = (
            'attached' if (
                resume_entry == (3, 2) and
                self.cube_recovery_already_attached)
            else 'detached')
        self.wait_support('left', expected_left, 1.0)
        with self.lock:
            positions = dict(self.joint_positions)
        if resume_entry == (6, 1):
            self.wait_latch('locked', 1.0)
            with self.lock:
                mechanism = dict(self.mechanism_positions)
                velocities = dict(self.mechanism_velocities)
            if (abs(mechanism.get('door_hinge_joint', math.inf)) >
                    math.radians(0.2) or
                    not -math.radians(1.0) <=
                    mechanism.get('handle_latch_joint', math.inf) <=
                    math.radians(7.0) or
                    abs(velocities.get('handle_latch_joint', math.inf)) >
                    0.01):
                raise ExecutionFault('RECOVERY_HANDLE_RELEASE_READY')
            self.require_close_to_start(plan['stages'][6]['messages'][2])
            gripper_record = plan['stages'][6]['messages'][1]
            clamp_error = max(
                abs(positions[name] - float(value))
                for name, value in zip(
                    gripper_record['joint_names'],
                    gripper_record['points'][0]['positions']))
            if clamp_error > 0.006:
                raise ExecutionFault(
                    f'RECOVERY_HANDLE_GRIPPER_CLAMP:{clamp_error:.9f}')
            self.event(
                'RECOVERY_HANDLE_RELEASE_READY',
                handle_position_rad=mechanism['handle_latch_joint'],
                gripper_clamp_error_rad=clamp_error)
        elif resume_entry == (1, 0):
            self.wait_latch('unlocked', 2.0)
            self.wait_mechanism(0.0, math.pi / 2.0, 2.0)
            # The left arm must still be holding the handle at the exact
            # audited unlock endpoint while message 1:0 opens the gripper.
            self.require_close_to_start(plan['stages'][1]['messages'][1])
            self.event('RECOVERY_UNLOCK_ENDPOINT_READY')
        elif resume_entry == (2, 3):
            self.wait_latch('unlocked', 2.0)
            self.wait_mechanism(0.0, math.pi / 2.0, 2.0)
            self.require_close_to_start(plan['stages'][2]['messages'][3])
            gripper_record = plan['stages'][2]['messages'][2]
            self.require_close(
                positions, gripper_record['joint_names'],
                gripper_record['points'][-1]['positions'], 0.006,
                'RECOVERY_DOOR_GRIPPER_CLAMPED')
            # Repeat the unchanged attach helper after the mechanism has
            # settled; the 601-point door trajectory remains forbidden until
            # its fresh attached heartbeat is observed.
            if self.door_recovery_already_attached:
                self.wait_support('right', 'attached', 1.0)
                self.event('RECOVERY_DOOR_ALREADY_ATTACHED_READY')
            else:
                self.run_support_helper(
                    'door_attach_recovery', 'door', 'recover_attach')
                self.wait_support('right', 'attached', 2.0)
                self.event('RECOVERY_DOOR_ATTACH_READY')
        else:
            self.wait_latch('unlocked', 1.0)
            self.wait_mechanism(math.pi / 3.0, math.pi / 2.0, 2.0)
            self.require_close_to_start(plan['stages'][4]['messages'][0])
            cube_gripper = CONTROLLER_JOINTS[
                '/left_gripper_controller/joint_trajectory']
            self.require_close(
                positions, cube_gripper, [-0.014942, -0.014942],
                self.endpoint_tolerance, 'RECOVERY_CUBE_GRIPPER_CLAMPED')
            if resume_entry == (3, 2):
                if self.cube_recovery_already_attached:
                    self.wait_support('left', 'attached', 1.0)
                    self.event('RECOVERY_CUBE_ALREADY_ATTACHED_READY')
                else:
                    self.run_support_helper(
                        'cube_attach_recovery', 'cube', 'recover_attach')
                    self.wait_support('left', 'attached', 2.0)
                    self.event('RECOVERY_CUBE_ATTACH_READY')
            else:
                self.require_close_to_start(plan['stages'][3]['messages'][4])
        record = plan['stages'][self.resume_stage_index]['messages'][
            self.resume_message_index]
        active, source_index, error = self.audited_tag_resume_connector(
            plan, record, self.resume_stage_index, 0)
        self.resume_record = active
        self.resume_source_index = source_index
        self.event(
            'REMAINING_PATH_REPLAN_CPP_AUDIT_PASS',
            stage_index=self.resume_stage_index,
            message_index=self.resume_message_index,
            source_point_index=source_index,
            connector_error_rad=error)

    def execute_remaining_plan(self, plan, certificate):
        """Execute the audited stage-3 suffix and every untouched later stage."""
        if self.resume_record is None:
            raise ExecutionFault('RECOVERY_RECORD_MISSING')
        for stage_index in range(self.resume_stage_index, len(plan['stages'])):
            stage = plan['stages'][stage_index]
            first_message = (
                self.resume_message_index
                if stage_index == self.resume_stage_index else 0)
            self.event(
                'STAGE_START', stage_index=stage_index,
                stage=stage['name'], resumed=True)
            for message_index in range(first_message, len(stage['messages'])):
                override = (
                    self.resume_record
                    if (stage_index, message_index) == (
                        self.resume_stage_index, self.resume_message_index)
                    else None)
                self.execute_message(
                    plan, certificate, stage_index, message_index,
                    stage['messages'][message_index],
                    initial_override=override,
                    initial_source_index=(
                        self.resume_source_index if override is not None else 0))
                self.support_checkpoint(stage_index, message_index)
            self.stage_checkpoint(stage_index)
            self.event(
                'STAGE_PASS', stage_index=stage_index,
                stage=stage['name'], resumed=True)

    def runtime_gate(self, plan, certificate, require_cabinet=True,
                     allow_raw_cabinet=False):
        now_wall = time.monotonic_ns()
        maximum_age_ns = int(self.maximum_age_s * 1e9)
        with self.lock:
            fault = self.fault_requested
            joints = dict(self.joint_positions)
            joint_receive = self.joint_receive_ns
            mechanism = dict(self.mechanism_positions)
            velocities = dict(self.mechanism_velocities)
            mechanism_receive = self.mechanism_receive_ns
            cabinet = self.cabinet
            cabinet_receive = self.cabinet_receive_ns
            raw_cabinet = (
                None if self.raw_cabinet_pose is None else
                self.raw_cabinet_pose.copy())
            raw_cabinet_stamp = self.raw_cabinet_stamp_ns
            raw_cabinet_receive = self.raw_cabinet_receive_ns
            latch_receive = self.latch_receive_ns
            right_support = self.right_support_state
            right_support_receive = self.right_support_receive_ns
            left_support = self.left_support_state
            left_support_receive = self.left_support_receive_ns
        if fault:
            raise ExecutionFault(fault)
        required = {name for names in CONTROLLER_JOINTS.values()
                    for name in names}
        if (required - joints.keys() or not joint_receive or
                now_wall - joint_receive > maximum_age_ns):
            raise ExecutionFault('STALE_ROBOT_STATE')
        if (not mechanism_receive or
                now_wall - mechanism_receive > maximum_age_ns or
                {'door_hinge_joint', 'handle_latch_joint'} -
                mechanism.keys() or
                {'door_hinge_joint', 'handle_latch_joint'} -
                velocities.keys()):
            raise ExecutionFault('STALE_MECHANISM_STATE')
        door = mechanism['door_hinge_joint']
        handle = mechanism['handle_latch_joint']
        if (not -math.radians(2.0) <= door <= math.radians(62.0) or
                not -math.radians(5.0) <= handle <= math.radians(95.0)):
            raise ExecutionFault('MECHANISM_ENVELOPE')
        if not latch_receive or now_wall - latch_receive > maximum_age_ns:
            raise ExecutionFault('STALE_LATCH')
        support_age_ns = int(self.maximum_support_age_s * 1e9)
        if (right_support not in ('attached', 'detached') or
                left_support not in ('attached', 'detached') or
                not right_support_receive or not left_support_receive or
                now_wall - right_support_receive > support_age_ns or
                now_wall - left_support_receive > support_age_ns):
            raise ExecutionFault('STALE_SUPPORT_STATE')
        if require_cabinet:
            stable_available = (
                cabinet is not None and cabinet_receive and
                now_wall - cabinet_receive <= maximum_age_ns and
                cabinet.valid and cabinet.stable)
            raw_available = (
                allow_raw_cabinet and raw_cabinet is not None and
                raw_cabinet_receive and
                now_wall - raw_cabinet_receive <= maximum_age_ns)
            if stable_available:
                if cabinet.config_hash != certificate['tag_config_hash']:
                    raise ExecutionFault('LIVE_TAG_CONFIG_HASH')
                cabinet_stamp = stamp_ns(cabinet.header.stamp)
                live = pose_matrix(cabinet.pose_world.pose)
            elif raw_available:
                if self.tag_config['config_hash'] != certificate[
                        'tag_config_hash']:
                    raise ExecutionFault('LIVE_TAG_CONFIG_HASH')
                cabinet_stamp = raw_cabinet_stamp
                live = raw_cabinet
            else:
                raise ExecutionFault('STALE_CABINET_TAG')
            age_ns = self.get_clock().now().nanoseconds - cabinet_stamp
            if age_ns < 0 or age_ns > maximum_age_ns:
                raise ExecutionFault('CABINET_TAG_STAMP')
            expected = np.asarray(plan['current_cabinet_pose'], dtype=float)
            translation = float(np.linalg.norm(
                live[:3, 3] - expected[:3, 3]))
            angle = rotation_error(expected, live)
            if translation > self.cabinet_translation_tolerance:
                raise ExecutionFault(
                    f'CABINET_TRANSLATION:{translation:.9f}')
            if angle > self.cabinet_rotation_tolerance:
                raise ExecutionFault(f'CABINET_ROTATION:{angle:.9f}')

    def require_close_to_start(self, message):
        with self.lock:
            positions = dict(self.joint_positions)
        self.require_close(
            positions, message['joint_names'],
            message['points'][0]['positions'], self.start_tolerance,
            'TRAJECTORY_START')

    @staticmethod
    def require_close(actual, names, expected, tolerance, status):
        if any(name not in actual for name in names):
            raise ExecutionFault(f'{status}:MISSING_JOINT')
        maximum = max(abs(actual[name] - float(value))
                      for name, value in zip(names, expected))
        if maximum > tolerance:
            raise ExecutionFault(f'{status}:error={maximum:.9f}')

    @staticmethod
    def trajectory_message(record):
        message = JointTrajectory()
        message.joint_names = list(record['joint_names'])
        for value in record['points']:
            point = JointTrajectoryPoint()
            point.positions = [float(item) for item in value['positions']]
            point.velocities = [float(item) for item in value['velocities']]
            point.accelerations = [
                float(item) for item in value['accelerations']]
            point.effort = [float(item) for item in value['effort']]
            point.time_from_start.sec = int(value['time_from_start']['sec'])
            point.time_from_start.nanosec = int(
                value['time_from_start']['nanosec'])
            message.points.append(point)
        return message

    @staticmethod
    def trajectory_suffix(record, point_index):
        """Return an exact certified suffix with time rebased to zero."""
        if not 0 <= point_index < len(record['points']) - 1:
            raise ExecutionFault('TRAJECTORY_SUFFIX_INDEX')
        result = copy.deepcopy(record)
        base_ns = point_duration_ns(record['points'][point_index])
        result['points'] = result['points'][point_index:]
        for point in result['points']:
            value_ns = point_duration_ns(point) - base_ns
            point['time_from_start'] = {
                'sec': int(value_ns // 1_000_000_000),
                'nanosec': int(value_ns % 1_000_000_000),
            }
        return result

    def wait_for_stable_cabinet_after_hold(self, plan, certificate):
        """Wait a bounded interval for stable tags while all joints hold."""
        deadline = time.monotonic() + self.tag_reacquisition_timeout
        while time.monotonic() < deadline:
            try:
                self.runtime_gate(
                    plan, certificate, require_cabinet=True,
                    allow_raw_cabinet=False)
                return
            except ExecutionFault as error:
                if str(error) not in (
                        'STALE_CABINET_TAG', 'CABINET_TAG_STAMP'):
                    raise
            time.sleep(0.01)
        raise ExecutionFault('CABINET_TAG_PERSISTENT_LOSS')

    def nearest_certified_point(self, record, minimum_index):
        """Bind a held joint state to the remaining certified message path."""
        with self.lock:
            positions = dict(self.joint_positions)
        names = record['joint_names']
        if any(name not in positions for name in names):
            raise ExecutionFault('TAG_HOLD_RESUME_MISSING_JOINT')
        errors = [
            max(abs(positions[name] - float(value))
                for name, value in zip(names, point['positions']))
            for point in record['points'][minimum_index:]
        ]
        relative_index = min(range(len(errors)), key=errors.__getitem__)
        index = minimum_index + relative_index
        error = errors[relative_index]
        return index, error

    @staticmethod
    def position_only_point(positions, value_ns):
        """Return one finite position-only point for recovery auditing."""
        return {
            'positions': list(map(float, positions)),
            'velocities': [],
            'accelerations': [],
            'effort': [],
            'time_from_start': {
                'sec': int(value_ns // 1_000_000_000),
                'nanosec': int(value_ns % 1_000_000_000),
            },
        }

    def audited_tag_resume_connector(self, plan, record, stage_index,
                                     minimum_index):
        """
        C++-audit a dense measured-state connector to a plan suffix.

        The method is used only after a tag-triggered measured hold.  It does
        not loosen the 3 mrad exact-suffix gate: if controller dynamics leave
        the measured hold farther from the nearest certified point, the short
        connector and every remaining suffix point are replayed against the
        frozen scene with the same C++ collision and 10 mm clearance auditor.
        """
        if self.active_result_path is None:
            raise ExecutionFault('TAG_RESUME_RESULT_PATH')
        with self.lock:
            joints = dict(self.joint_positions)
        names = record['joint_names']
        if any(name not in joints for name in names):
            raise ExecutionFault('TAG_RESUME_MISSING_JOINT')
        actual = np.asarray([joints[name] for name in names], dtype=float)
        candidates = record['points'][minimum_index:]
        errors = [
            float(np.max(np.abs(
                actual - np.asarray(point['positions'], dtype=float))))
            for point in candidates
        ]
        relative_index = min(range(len(errors)), key=errors.__getitem__)
        source_index = minimum_index + relative_index
        error = errors[relative_index]
        if error > math.radians(4.9):
            raise ExecutionFault(
                f'TAG_RESUME_CONNECTOR_TOO_LARGE:{error:.9f}')
        target = np.asarray(
            record['points'][source_index]['positions'], dtype=float)
        connector_count = max(
            2, int(math.ceil(error / math.radians(0.049))) + 1)
        # Keep the connector within the audited plan's observed timing
        # envelope while making a milliradian-scale correction short enough
        # to complete between camera occlusion checks.  Smoothstep has peak
        # speed 1.5*delta/T and peak acceleration 6*delta/T^2; the bounds
        # below are stricter than the certified plan's observed 1.35 rad/s
        # and 2.47 rad/s^2 maxima.
        connector_duration_s = max(
            0.05, 1.5 * error / 1.0,
            math.sqrt(6.0 * error / 2.0))
        connector_duration_ns = int(math.ceil(
            connector_duration_s * 1_000_000_000.0))
        points = []
        for index in range(connector_count):
            fraction = index / (connector_count - 1)
            smooth = fraction * fraction * (3.0 - 2.0 * fraction)
            points.append(self.position_only_point(
                actual + smooth * (target - actual),
                int(round(connector_duration_ns * fraction))))
        source_time_ns = point_duration_ns(record['points'][source_index])
        for point in record['points'][source_index + 1:]:
            points.append(self.position_only_point(
                point['positions'],
                connector_duration_ns + point_duration_ns(point) -
                source_time_ns))
        active_record = copy.deepcopy(record)
        active_record['points'] = points

        all_names = [
            name for controller_names in CONTROLLER_JOINTS.values()
            for name in controller_names
        ]
        if any(name not in joints for name in all_names):
            raise ExecutionFault('TAG_RESUME_FULL_STATE')
        full_state = self.position_only_point(
            [joints[name] for name in all_names], 0)
        q0_message = plan['stages'][0]['messages'][0]
        q0_point = self.position_only_point(
            q0_message['points'][0]['positions'], 0)
        audit_stages = []
        for index, stage in enumerate(plan['stages']):
            if index == stage_index:
                messages = [
                    {'joint_names': all_names, 'points': [full_state]},
                    {'joint_names': list(names), 'points': points},
                ]
            else:
                messages = [{
                    'joint_names': list(q0_message['joint_names']),
                    'points': [q0_point],
                }]
            audit_stages.append({
                'name': stage['name'],
                'messages': messages,
            })
        audit_plan = {
            'scene_epoch_id': plan['scene_epoch_id'],
            'scene_hash': plan['scene_hash'],
            'candidate3_only': True,
            'contains_partial': False,
            'door_arc_target_count': 601,
            'stages': audit_stages,
        }
        audit_hash = canonical_hash(audit_plan)
        self.tag_resume_audit_count += 1
        prefix = f'tag_resume_audit_{self.tag_resume_audit_count:03d}'
        input_path = self.run_directory / f'{prefix}.input.json'
        result_path = self.run_directory / f'{prefix}.result.json'
        self.write_exclusive(
            input_path,
            json.dumps(
                {'plan': audit_plan, 'plan_hash': audit_hash},
                sort_keys=True, indent=2, allow_nan=False) + '\n')
        source_directory = self.active_result_path.parent
        executable = (
            Path(get_package_prefix('h1_2_moveit_demo')) / 'lib' /
            'h1_2_moveit_demo' / 'candidate3_clearance_auditor')
        completed = subprocess.run(
            [
                str(executable),
                '--scene', str(source_directory / 'scene.cdr'),
                '--plan', str(input_path),
                '--urdf', str(source_directory / 'clearance_audit_robot.urdf'),
                '--srdf', str(source_directory / 'clearance_audit_robot.srdf'),
                '--output', str(result_path),
            ],
            stdin=subprocess.DEVNULL, capture_output=True, text=True,
            timeout=120.0, check=False)
        self.write_exclusive(
            self.run_directory / f'{prefix}.stdout.log', completed.stdout)
        self.write_exclusive(
            self.run_directory / f'{prefix}.stderr.log', completed.stderr)
        if completed.returncode not in (0, 2) or not result_path.is_file():
            raise ExecutionFault(
                f'TAG_RESUME_CPP_EXIT:{completed.returncode}')
        audit = json.loads(result_path.read_text(encoding='utf-8'))
        os.chmod(result_path, 0o400)
        expected_scene = (
            f'h1_scene_epoch:{plan["scene_epoch_id"]}:'
            f'{plan["scene_hash"]}')
        if (audit.get('schema') !=
                'h1_candidate3_cpp_clearance_audit/v1' or
                audit.get('scene_name') != expected_scene or
                audit.get('plan_hash') != audit_hash or
                audit.get('status') != 'PASS' or
                audit.get('collision_free') is not True or
                audit.get('clearance_passed') is not True or
                float(audit.get('minimum_environment_clearance_m', 0.0)) <
                0.010):
            raise ExecutionFault(
                f'TAG_RESUME_CPP_AUDIT:{audit.get("status", "INVALID")}')
        # The robot remained held while the file-only replay ran.  Bind the
        # first published point back to fresh live telemetry.
        with self.lock:
            current = dict(self.joint_positions)
        live_error = max(abs(current[name] - float(value))
                         for name, value in zip(names, actual))
        if live_error > self.start_tolerance:
            raise ExecutionFault(
                f'TAG_RESUME_LIVE_STATE_CHANGED:{live_error:.9f}')
        self.event(
            'CABINET_TAG_RESUME_CONNECTOR_CPP_AUDIT_PASS',
            stage_index=stage_index, source_point_index=source_index,
            connector_error_rad=error,
            connector_point_count=connector_count,
            connector_duration_s=connector_duration_ns * 1e-9,
            audit_hash=audit_hash, samples=audit['samples'],
            minimum_environment_clearance_m=(
                audit['minimum_environment_clearance_m']))
        return active_record, source_index, error

    def execute_plan(self, plan, certificate):
        for stage_index, stage in enumerate(plan['stages']):
            self.event('STAGE_START', stage_index=stage_index,
                       stage=stage['name'])
            for message_index, message in enumerate(stage['messages']):
                self.execute_message(
                    plan, certificate, stage_index, message_index, message)
                self.support_checkpoint(stage_index, message_index)
            self.stage_checkpoint(stage_index)
            self.event('STAGE_PASS', stage_index=stage_index,
                       stage=stage['name'])

    def execute_message(self, plan, certificate, stage_index,
                        message_index, record, initial_override=None,
                        initial_source_index=0):
        self.runtime_gate(
            plan, certificate, require_cabinet=True,
            allow_raw_cabinet=True)
        if initial_override is None:
            self.require_close_to_start(record)
        publisher = self.trajectory_publishers[record['topic']]
        if (publisher.get_subscription_count() != 1 or
                self.count_publishers(record['topic']) != 1):
            raise ExecutionFault('CONTROLLER_TOPOLOGY_CHANGED')
        self.motion_started = True
        expected = record['points'][-1]['positions']
        if stage_index == 3 and message_index == 1:
            expected = [-0.014942, -0.014942]
        source_index = initial_source_index
        tag_holds = 0
        active_override = initial_override
        while True:
            if active_override is not None:
                active_record = active_override
                active_override = None
            else:
                active_record = (
                    record if source_index == 0 else
                    self.trajectory_suffix(record, source_index))
            self.require_close_to_start(active_record)
            outgoing = self.trajectory_message(active_record)
            publish_wall_ns = time.monotonic_ns()
            publish_sim_ns = self.get_clock().now().nanoseconds
            publisher.publish(outgoing)
            duration_ns = point_duration_ns(active_record['points'][-1])
            wall_deadline = time.monotonic() + max(
                self.maximum_age_s,
                duration_ns * 1e-9 * self.timeout_factor +
                self.timeout_padding)
            stable_count = 0
            resume_after_hold = False
            while time.monotonic() < wall_deadline:
                try:
                    self.runtime_gate(
                        plan, certificate, require_cabinet=True,
                        allow_raw_cabinet=True)
                except ExecutionFault as error:
                    if str(error) != 'STALE_CABINET_TAG':
                        raise
                    tag_holds += 1
                    if tag_holds > self.maximum_tag_occlusion_holds:
                        raise ExecutionFault(
                            'CABINET_TAG_OCCLUSION_HOLD_LIMIT')
                    hold_status = self.publish_measured_hold()
                    if hold_status != 'HOLD_PASS':
                        raise ExecutionFault(
                            f'TAG_OCCLUSION_{hold_status}')
                    self.event(
                        'CABINET_TAG_OCCLUSION_HOLD',
                        stage_index=stage_index,
                        message_index=message_index,
                        hold_count=tag_holds)
                    self.wait_for_stable_cabinet_after_hold(plan, certificate)
                    index, error_rad = self.nearest_certified_point(
                        record, source_index)
                    if (error_rad <= self.start_tolerance and
                            index >= len(record['points']) - 1):
                        with self.lock:
                            positions = dict(self.joint_positions)
                        self.require_close(
                            positions, record['joint_names'], expected,
                            self.endpoint_tolerance, 'TRAJECTORY_ENDPOINT')
                        self.event(
                            'CABINET_TAG_REACQUIRED_AT_ENDPOINT',
                            stage_index=stage_index,
                            message_index=message_index,
                            error_rad=error_rad)
                        return
                    if error_rad <= self.start_tolerance:
                        source_index = index
                        self.event(
                            'CABINET_TAG_REACQUIRED_RESUME_SUFFIX',
                            stage_index=stage_index,
                            message_index=message_index,
                            source_point_index=source_index,
                            error_rad=error_rad)
                    else:
                        active_override, source_index, error_rad = (
                            self.audited_tag_resume_connector(
                                plan, record, stage_index, source_index))
                        self.event(
                            'CABINET_TAG_REACQUIRED_RESUME_CPP_CONNECTOR',
                            stage_index=stage_index,
                            message_index=message_index,
                            source_point_index=source_index,
                            error_rad=error_rad)
                    resume_after_hold = True
                    break
                sim_elapsed = (
                    self.get_clock().now().nanoseconds - publish_sim_ns)
                if sim_elapsed < 0:
                    raise ExecutionFault('SIMULATION_CLOCK_REVERSED')
                if sim_elapsed >= duration_ns:
                    with self.lock:
                        positions = dict(self.joint_positions)
                        received = self.joint_receive_ns
                    try:
                        if received > publish_wall_ns:
                            self.require_close(
                                positions, record['joint_names'], expected,
                                self.endpoint_tolerance,
                                'TRAJECTORY_ENDPOINT')
                            stable_count += 1
                        else:
                            stable_count = 0
                    except ExecutionFault:
                        stable_count = 0
                    if stable_count >= 3:
                        self.event(
                            'TRAJECTORY_PASS', stage_index=stage_index,
                            message_index=message_index,
                            topic=record['topic'],
                            points=len(record['points']),
                            duration_s=point_duration_ns(
                                record['points'][-1]) * 1e-9,
                            tag_occlusion_holds=tag_holds)
                        return
                time.sleep(0.005)
            if resume_after_hold:
                continue
            raise ExecutionFault(
                f'TRAJECTORY_TIMEOUT:stage={stage_index}:'
                f'message={message_index}')

    def support_checkpoint(self, stage_index, message_index):
        if (stage_index, message_index) == (2, 2):
            self.run_support_helper('door_attach', 'door', 'attach')
            self.wait_support('right', 'attached', 2.0)
        elif (stage_index, message_index) == (3, 1):
            self.run_support_helper('cube_attach', 'cube', 'attach')
            self.wait_support('left', 'attached', 2.0)
        elif (stage_index, message_index) == (3, 2):
            if ((self.resume_stage_index, self.resume_message_index) ==
                    (3, 2) and self.cube_recovery_preload_file is not None):
                self.run_support_helper(
                    'cube_detach_recovery', 'cube', 'recover_detach')
            else:
                self.run_support_helper('cube_detach', 'cube', 'detach')
            self.wait_support('left', 'detached', 2.0)
        elif (stage_index, message_index) == (4, 0):
            self.run_support_helper(
                'door_detach', 'door_detach', 'detach')
            self.wait_support('right', 'detached', 2.0)

    def wait_latch(self, expected, timeout_s):
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            with self.lock:
                value = self.latch_state
                received = self.latch_receive_ns
            if (value == expected and received and
                    time.monotonic_ns() - received <=
                    int(self.maximum_age_s * 1e9)):
                return
            time.sleep(0.02)
        raise ExecutionFault(f'LATCH_STATE:{expected}')

    def wait_mechanism(self, door, handle, timeout_s):
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            with self.lock:
                positions = dict(self.mechanism_positions)
                velocities = dict(self.mechanism_velocities)
                received = self.mechanism_receive_ns
            if (received and time.monotonic_ns() - received <=
                    int(self.maximum_age_s * 1e9) and
                    abs(positions.get('door_hinge_joint', math.inf) - door) <=
                    math.radians(0.5) and
                    abs(positions.get('handle_latch_joint', math.inf) - handle)
                    <= math.radians(3.0) and
                    abs(velocities.get('door_hinge_joint', math.inf)) <= 0.01 and
                    abs(velocities.get('handle_latch_joint', math.inf)) <= 0.01):
                return
            time.sleep(0.02)
        raise ExecutionFault('MECHANISM_CHECKPOINT')

    def stage_checkpoint(self, stage_index):
        if stage_index == 0:
            self.wait_latch('unlocked', 2.0)
            self.wait_mechanism(0.0, math.pi / 2.0, 2.0)
        elif stage_index == 2:
            self.wait_support('right', 'attached', 1.0)
            self.wait_mechanism(math.pi / 3.0, math.pi / 2.0, 2.0)
        elif stage_index == 3:
            self.wait_support('left', 'detached', 1.0)
        elif stage_index == 4:
            self.wait_support('right', 'detached', 1.0)
            self.wait_mechanism(0.0, math.pi / 2.0, 2.0)
        elif stage_index == 6:
            self.wait_latch('locked', 2.0)
            # The latch return is a passive Gazebo joint response after the
            # gripper releases.  At the admitted RTF floor, ten wall seconds
            # provide the same two seconds of simulation settling used by the
            # mechanism contract; the unchanged positional/velocity gates are
            # still enforced by wait_mechanism().
            self.wait_mechanism(0.0, 0.0, 10.0)

    def publish_measured_hold(self):
        try:
            required = {name for names in CONTROLLER_JOINTS.values()
                        for name in names}
            # A first hold can arrest a moving controller outside the 3 mrad
            # endpoint band.  One bounded second measured hold is permitted;
            # it targets the now-arrested telemetry and never resumes motion.
            for hold_attempt in (1, 2):
                deadline = time.monotonic() + min(1.0, self.maximum_age_s)
                while time.monotonic() < deadline:
                    with self.lock:
                        positions = dict(self.joint_positions)
                        received = self.joint_receive_ns
                    if (required <= positions.keys() and received and
                            time.monotonic_ns() - received <=
                            int(self.maximum_age_s * 1e9)):
                        break
                    time.sleep(0.01)
                else:
                    return 'HOLD_FAIL:NO_FRESH_JOINT_STATE'
                targets = {}
                for topic, names in CONTROLLER_JOINTS.items():
                    if (self.trajectory_publishers[
                            topic].get_subscription_count() != 1):
                        return f'HOLD_FAIL:SUBSCRIBER:{topic}'
                    message = JointTrajectory()
                    message.joint_names = list(names)
                    point = JointTrajectoryPoint()
                    point.positions = [positions[name] for name in names]
                    point.time_from_start.sec = 0
                    point.time_from_start.nanosec = 100_000_000
                    message.points.append(point)
                    targets[topic] = (names, list(point.positions), message)
                for _ in range(3):
                    for topic, (_, _, message) in targets.items():
                        self.trajectory_publishers[topic].publish(message)
                    time.sleep(0.05)
                verify_deadline = time.monotonic() + 1.0
                while time.monotonic() < verify_deadline:
                    with self.lock:
                        actual = dict(self.joint_positions)
                    try:
                        for names, expected, _ in targets.values():
                            self.require_close(
                                actual, names, expected,
                                self.endpoint_tolerance, 'HOLD_ENDPOINT')
                        self.event(
                            'STOP_AND_HOLD_PASS',
                            hold_attempt=hold_attempt)
                        return 'HOLD_PASS'
                    except ExecutionFault:
                        time.sleep(0.02)
            return 'HOLD_FAIL:ENDPOINT'
        except Exception as error:
            return f'HOLD_FAIL:{type(error).__name__}:{error}'

    def finalize_success(self, result, result_path):
        self.final_status = 'PASS'
        self.event('COMPLETE_PASS')
        payload = self.evidence_payload(
            'PASS', result=result, result_path=result_path)
        self.write_exclusive(
            self.run_directory / 'execution_result.json',
            json.dumps(payload, sort_keys=True, indent=2, allow_nan=False) + '\n')
        self.publish_status(
            'PASS', plan_hash=result['plan_hash'], gazebo_only=True,
            evidence=str(self.run_directory / 'execution_result.json'))
        self.done.set()

    def finalize_failure(self, reason, result=None):
        if self.done.is_set():
            return
        self.final_status = reason
        try:
            self.event('FAIL_CLOSED', reason=reason,
                       motion_started=self.motion_started)
            if self.run_directory is not None:
                payload = self.evidence_payload('FAIL_CLOSED', result=result)
                payload['reason'] = reason
                self.write_exclusive(
                    self.run_directory / 'execution_result.json',
                    json.dumps(
                        payload, sort_keys=True, indent=2,
                        allow_nan=False) + '\n')
        except Exception as evidence_error:
            self.get_logger().error(f'evidence failure: {evidence_error}')
        self.publish_status(
            'FAIL_CLOSED', reason=reason,
            motion_started=self.motion_started, gazebo_only=True)
        self.done.set()

    def evidence_payload(self, status, result=None, result_path=None):
        with self.lock:
            events = list(self.events)
            latch = self.latch_state
            right = self.right_support_state
            left = self.left_support_state
            mechanism = dict(self.mechanism_positions)
            raw_cabinet_tag_id = self.raw_cabinet_tag_id
            raw_cabinet_observation_count = (
                self.raw_cabinet_observation_count)
        return {
            'schema': SCHEMA,
            'status': status,
            'gazebo_only': True,
            'execute_was_explicit': self.execute_enabled,
            'execution_mode': self.execution_mode,
            'resume_enabled': self.resume_enabled,
            'resume_stage_index': self.resume_stage_index,
            'resume_message_index': self.resume_message_index,
            'plan_hash': result.get('plan_hash', '') if result else '',
            'certificate_hash': (
                result.get('certificate_hash', '') if result else ''),
            'source_result': str(result_path) if result_path else '',
            'motion_started': self.motion_started,
            'final_latch_state': latch,
            'final_right_door_support_state': right,
            'final_left_cube_support_state': left,
            'final_mechanism_positions': mechanism,
            'raw_cabinet_tag_id': raw_cabinet_tag_id,
            'raw_cabinet_observation_count': raw_cabinet_observation_count,
            'events': events,
        }


def main(args=None):
    rclpy.init(args=args)
    node = Candidate3ExecutionSupervisor()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        while rclpy.ok() and not node.done.is_set():
            executor.spin_once(timeout_sec=0.1)
    except KeyboardInterrupt:
        if node.motion_started:
            node.publish_measured_hold()
        node.finalize_failure('KEYBOARD_INTERRUPT')
    finally:
        node.stop_support_bridge()
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
