"""
Plan-only bridge from frozen scene epochs to the candidate-3 worker.

The node intentionally has no trajectory publisher, action client, controller
client, or planning-scene writer.  A matched immutable epoch/scene pair is
serialized for an isolated MoveItPy worker; only plan metadata, its audit
certificate, and status are published.
"""

from __future__ import annotations

import json
from pathlib import Path
import subprocess
import threading
import uuid

from ament_index_python.packages import get_package_prefix
from builtin_interfaces.msg import Duration
from h1_2_perception_msgs.msg import (
    Candidate3Plan,
    PlanCertificate,
    SceneEpoch,
)
from moveit_msgs.msg import PlanningScene
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.serialization import serialize_message
from std_msgs.msg import String


class Candidate3Replanner(Node):
    """Bind one frozen scene epoch to one isolated plan-only audit."""

    def __init__(self):
        """Create only the three plan-metadata output publishers."""
        super().__init__('h1_2_candidate3_replanner')
        try:
            demo_prefix = Path(get_package_prefix('h1_2_moveit_demo'))
            default_clearance_auditor = str(
                demo_prefix / 'lib' / 'h1_2_moveit_demo' /
                'candidate3_clearance_auditor')
        except LookupError:
            default_clearance_auditor = ''
        self.declare_parameter('asset_root', '')
        self.declare_parameter('output_root', '/tmp/h1_candidate3_plan_only')
        self.declare_parameter('worker_python', '/usr/bin/python3')
        self.declare_parameter('worker_timeout_s', 300.0)
        self.declare_parameter(
            'clearance_auditor', default_clearance_auditor)
        self.declare_parameter('once', True)
        asset_root_value = str(
            self.get_parameter('asset_root').value).strip()
        self.asset_root = Path(asset_root_value) if asset_root_value else None
        self.output_root = Path(str(self.get_parameter('output_root').value))
        self.worker_python = str(self.get_parameter('worker_python').value)
        self.worker_timeout = float(
            self.get_parameter('worker_timeout_s').value)
        clearance_auditor = str(
            self.get_parameter('clearance_auditor').value).strip()
        self.clearance_auditor = (
            Path(clearance_auditor) if clearance_auditor else None)
        self.once = bool(self.get_parameter('once').value)

        transient = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.plan_pub = self.create_publisher(
            Candidate3Plan, '/candidate3_plan', transient)
        self.certificate_pub = self.create_publisher(
            PlanCertificate, '/plan_certificate', transient)
        self.status_pub = self.create_publisher(
            String, '/candidate3_replan/status', transient)
        self.create_subscription(
            SceneEpoch, '/dynamic_scene/epoch', self.epoch_callback,
            transient)
        self.create_subscription(
            PlanningScene, '/dynamic_scene/frozen_scene', self.scene_callback,
            transient)

        self.lock = threading.Lock()
        self.epochs = {}
        self.scenes = {}
        self.active_key = None
        self.completed_key = None
        self.published_key = None
        self.invalidated_key = None

        if self.asset_root is None:
            self.publish_status('CONFIG_ERROR:asset_root is required')
        elif (self.clearance_auditor is None or
              not self.clearance_auditor.is_file()):
            self.publish_status(
                'CONFIG_ERROR:clearance_auditor is required')
        else:
            self.publish_status('WAITING_FOR_MATCHED_FROZEN_SCENE')

    @staticmethod
    def key_for_epoch(message):
        return int(message.epoch_id), message.scene_hash

    @staticmethod
    def key_for_scene(message):
        prefix = 'h1_scene_epoch:'
        if message.is_diff or not message.name.startswith(prefix):
            return None
        fields = message.name[len(prefix):].split(':', 1)
        if len(fields) != 2:
            return None
        try:
            return int(fields[0]), fields[1]
        except ValueError:
            return None

    def publish_status(self, value, **fields):
        message = String()
        message.data = json.dumps(
            {'status': value, **fields}, sort_keys=True)
        self.status_pub.publish(message)

    def epoch_callback(self, message):
        key = self.key_for_epoch(message)
        with self.lock:
            self.epochs[key] = serialize_message(message)
            self.epochs = {key: self.epochs[key]}
        self.maybe_start(key)

    def scene_callback(self, message):
        key = self.key_for_scene(message)
        if key is None:
            self.publish_status('IGNORED_UNBOUND_SCENE')
            return
        with self.lock:
            self.scenes[key] = serialize_message(message)
            self.scenes = {key: self.scenes[key]}
        self.maybe_start(key)

    def maybe_start(self, key):
        with self.lock:
            if key not in self.epochs or key not in self.scenes:
                return
            if self.published_key is not None and key != self.published_key:
                if self.invalidated_key != key:
                    self.invalidated_key = key
                    self.publish_invalidated(key)
                if self.once:
                    return
            if self.active_key is not None or key == self.completed_key:
                return
            if self.asset_root is None:
                return
            if (self.clearance_auditor is None or
                    not self.clearance_auditor.is_file()):
                return
            epoch_bytes = self.epochs[key]
            scene_bytes = self.scenes[key]
            self.active_key = key
        self.publish_status(
            'PLANNING', scene_epoch_id=key[0], scene_hash=key[1])
        thread = threading.Thread(
            target=self.run_worker,
            args=(key, epoch_bytes, scene_bytes),
            daemon=True,
        )
        thread.start()

    def new_run_directory(self, key):
        self.output_root.mkdir(parents=True, exist_ok=True)
        for _ in range(10):
            path = self.output_root / (
                f'epoch_{key[0]}_{key[1][:12]}_{uuid.uuid4().hex}')
            try:
                path.mkdir(mode=0o700)
                return path
            except FileExistsError:
                continue
        raise RuntimeError('cannot create unique output directory')

    def run_worker(self, key, epoch_bytes, scene_bytes):
        try:
            run_directory = self.new_run_directory(key)
            epoch_path = run_directory / 'epoch.cdr'
            scene_path = run_directory / 'scene.cdr'
            result_path = run_directory / 'result.json'
            stdout_path = run_directory / 'worker.stdout.log'
            stderr_path = run_directory / 'worker.stderr.log'
            epoch_path.write_bytes(epoch_bytes)
            scene_path.write_bytes(scene_bytes)
            command = [
                self.worker_python, '-m', 'h1_2_vision.candidate3_worker',
                '--epoch', str(epoch_path),
                '--scene', str(scene_path),
                '--asset-root', str(self.asset_root),
                '--clearance-auditor', str(self.clearance_auditor),
                '--clearance-timeout-s', str(max(
                    1.0, self.worker_timeout - 10.0)),
                '--output', str(result_path),
            ]
            with stdout_path.open('wb') as stdout_file, (
                    stderr_path.open('wb')) as stderr_file:
                completed = subprocess.run(
                    command,
                    stdin=subprocess.DEVNULL,
                    stdout=stdout_file,
                    stderr=stderr_file,
                    timeout=self.worker_timeout,
                    check=False,
                )
            if not result_path.is_file():
                raise RuntimeError(
                    f'WORKER_NO_RESULT:exit={completed.returncode}')
            result = json.loads(result_path.read_text(encoding='utf-8'))
            self.publish_result(key, result, run_directory)
        except subprocess.TimeoutExpired:
            self.publish_failure(key, 'WORKER_TIMEOUT')
        except Exception as error:  # The boundary must always fail closed.
            self.publish_failure(
                key, f'{type(error).__name__}:{error}')
        finally:
            with self.lock:
                self.completed_key = key
                self.active_key = None

    def publish_invalidated(self, key):
        message = PlanCertificate()
        message.header.stamp = self.get_clock().now().to_msg()
        message.scene_epoch_id = key[0]
        message.scene_hash = key[1]
        message.candidate3_only = True
        message.no_partial = True
        message.valid = False
        message.status = 'SCENE_EPOCH_CHANGED'
        self.certificate_pub.publish(message)
        self.publish_status(
            message.status, scene_epoch_id=key[0], scene_hash=key[1])

    def publish_failure(self, key, status):
        message = PlanCertificate()
        message.header.stamp = self.get_clock().now().to_msg()
        message.scene_epoch_id = key[0]
        message.scene_hash = key[1]
        message.candidate3_only = True
        message.no_partial = True
        message.valid = False
        message.status = status
        self.certificate_pub.publish(message)
        self.publish_status(
            status, scene_epoch_id=key[0], scene_hash=key[1], valid=False)

    def publish_result(self, key, result, run_directory):
        plan = result.get('plan')
        certificate = result.get('certificate', {})
        now = self.get_clock().now().to_msg()

        plan_message = Candidate3Plan()
        plan_message.header.stamp = now
        plan_message.scene_epoch_id = key[0]
        plan_message.scene_hash = key[1]
        plan_message.plan_hash = result.get('plan_hash', '')
        if plan is not None:
            plan_message.topology = plan.get('topology', '')
            plan_message.door_arc_target_count = int(
                plan.get('door_arc_target_count', 0))
            plan_message.stage_count = len(plan.get('stage_names', []))
            plan_message.stage_names = plan.get('stage_names', [])
            plan_message.stage_trajectory_hashes = plan.get(
                'stage_hashes', [])
            plan_message.stage_waypoint_counts = plan.get(
                'stage_waypoint_counts', [])
            plan_message.candidate3_only = bool(
                plan.get('candidate3_only', False))
            plan_message.contains_partial = bool(
                plan.get('contains_partial', True))
        plan_message.valid = bool(certificate.get('valid', False))
        plan_message.status = certificate.get('status', 'MISSING_STATUS')

        certificate_message = PlanCertificate()
        certificate_message.header.stamp = now
        certificate_message.scene_epoch_id = key[0]
        certificate_message.scene_hash = key[1]
        certificate_message.plan_hash = result.get('plan_hash', '')
        certificate_message.certificate_hash = result.get(
            'certificate_hash', '')
        for field in (
                'robot_model_hash', 'stl_hash', 'tag_config_hash',
                'planner_version', 'auditor_version'):
            setattr(certificate_message, field, certificate.get(field, ''))
        certificate_message.door_arc_target_count = int(
            certificate.get('door_arc_target_count', 0))
        certificate_message.forward_dense_samples = int(
            certificate.get('forward_dense_samples', 0))
        certificate_message.reverse_dense_samples = int(
            certificate.get('reverse_dense_samples', 0))
        certificate_message.minimum_environment_clearance_m = float(
            certificate.get('minimum_environment_clearance_m', 0.0))
        certificate_message.maximum_adjacent_joint_change_rad = float(
            certificate.get('maximum_adjacent_joint_change_rad', 0.0))
        for field in (
                'candidate3_only', 'ik_complete', 'collision_free',
                'original_stl_contact_passed', 'hinge_contract_passed',
                'forward_passed', 'reverse_passed', 'timing_passed',
                'support_topology_passed', 'no_partial', 'valid'):
            setattr(certificate_message, field, bool(certificate.get(field, False)))
        validity_s = max(0.0, float(certificate.get('validity_s', 0.0)))
        seconds = int(validity_s)
        certificate_message.validity = Duration(
            sec=seconds,
            nanosec=int(round((validity_s - seconds) * 1e9)),
        )
        certificate_message.status = certificate.get(
            'status', 'MISSING_STATUS')

        self.plan_pub.publish(plan_message)
        self.certificate_pub.publish(certificate_message)
        with self.lock:
            self.published_key = key
        self.publish_status(
            certificate_message.status,
            scene_epoch_id=key[0],
            scene_hash=key[1],
            plan_hash=plan_message.plan_hash,
            certificate_hash=certificate_message.certificate_hash,
            valid=certificate_message.valid,
            result=str(run_directory / 'result.json'),
        )


def main(args=None):
    rclpy.init(args=args)
    node = Candidate3Replanner()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
