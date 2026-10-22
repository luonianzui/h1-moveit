"""
Capture one live, controller-quiescent middle-v1 scene pair.

This node is deliberately read-only.  It subscribes to the transient frozen
``SceneEpoch`` / ``PlanningScene`` pair, verifies the q90 middle seam and the
absence of robot-trajectory publishers, then writes the exact CDR bytes and a
small provenance document into a new private directory.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import stat
import threading
import time

from h1_2_perception_msgs.msg import SceneEpoch
from moveit_msgs.msg import PlanningScene
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.serialization import serialize_message

from .candidate3_middle_contract import (
    SEAM_JOINT_NAMES,
    SEAM_NOMINAL,
    SEAM_STATE_HASH,
    SEAM_TOLERANCES,
)


SCHEMA = 'h1_candidate3_middle_live_capture/v1'
CONTROLLER_TOPICS = (
    '/torso_controller/joint_trajectory',
    '/right_arm_controller/joint_trajectory',
    '/left_arm_controller/joint_trajectory',
    '/right_gripper_controller/joint_trajectory',
    '/left_gripper_controller/joint_trajectory',
)
REQUIRED_WORLD_IDS = frozenset((
    'environment_table_10mm_margin',
    'green_cube_10mm_guard',
    'green_cube_dynamic_exact',
    'incubator_body_exact',
    'incubator_door_exact',
    'incubator_handle_exact',
))


class CaptureError(RuntimeError):
    """A fail-closed live-capture error."""


def _sha256(value: bytes) -> str:
    return hashlib.sha256(value).hexdigest()


def _joint_map(message) -> dict[str, float]:
    if len(message.name) != len(message.position):
        raise CaptureError('JOINT_STATE_WIDTH')
    if len(set(message.name)) != len(message.name):
        raise CaptureError('JOINT_STATE_DUPLICATE')
    values = {}
    for name, position in zip(message.name, message.position):
        value = float(position)
        if not math.isfinite(value):
            raise CaptureError(f'JOINT_STATE_NONFINITE:{name}')
        values[str(name)] = value
    return values


def _mechanism_map(message) -> dict[str, float]:
    values = _joint_map(message)
    for required in ('door_hinge_joint', 'handle_latch_joint'):
        if required not in values:
            raise CaptureError(f'MECHANISM_JOINT_MISSING:{required}')
    return values


def _validate_pair(
        epoch: SceneEpoch, scene: PlanningScene, expected_handle: float,
        handle_tolerance: float, door_tolerance: float) -> dict:
    """Return deterministic metadata after validating one matched pair."""
    if not epoch.valid or not epoch.stable:
        raise CaptureError('EPOCH_NOT_STABLE_VALID')
    expected_name = f'h1_scene_epoch:{int(epoch.epoch_id)}:{epoch.scene_hash}'
    if scene.is_diff or scene.name != expected_name:
        raise CaptureError('SCENE_BINDING')
    if epoch.latch_state.strip().lower() != 'unlocked':
        raise CaptureError('LATCH_NOT_UNLOCKED')
    if not epoch.right_door_support_state.strip().upper().startswith('DETACHED'):
        raise CaptureError('RIGHT_DOOR_SUPPORT_NOT_DETACHED')
    if not epoch.left_cube_support_state.strip().upper().startswith('DETACHED'):
        raise CaptureError('LEFT_CUBE_SUPPORT_NOT_DETACHED')
    mechanism = _mechanism_map(epoch.mechanism_state)
    door = mechanism['door_hinge_joint']
    handle = mechanism['handle_latch_joint']
    if abs(door) > door_tolerance:
        raise CaptureError(f'DOOR_NOT_CLOSED:{door:.17g}')
    if abs(handle - expected_handle) > handle_tolerance:
        raise CaptureError(f'HANDLE_NOT_Q89_5:{handle:.17g}')

    robot = _joint_map(epoch.robot_state)
    for name, nominal, tolerance in zip(
            SEAM_JOINT_NAMES, SEAM_NOMINAL, SEAM_TOLERANCES):
        if name not in robot:
            raise CaptureError(f'SEAM_JOINT_MISSING:{name}')
        error = abs(robot[name] - nominal)
        if error > tolerance:
            raise CaptureError(f'SEAM_JOINT:{name}:{error:.17g}')

    scene_robot = _joint_map(scene.robot_state.joint_state)
    for name in SEAM_JOINT_NAMES:
        if name not in scene_robot or abs(scene_robot[name] - robot[name]) > 1e-12:
            raise CaptureError(f'SCENE_ROBOT_BINDING:{name}')
    world_ids = {value.id for value in scene.world.collision_objects}
    missing = sorted(REQUIRED_WORLD_IDS - world_ids)
    if missing:
        raise CaptureError(f'SCENE_WORLD_IDS:{",".join(missing)}')
    if scene.robot_state.attached_collision_objects:
        raise CaptureError('SCENE_ATTACHED_OBJECTS')

    return {
        'schema': SCHEMA,
        'scene_epoch_id': int(epoch.epoch_id),
        'scene_hash': epoch.scene_hash,
        'scene_name': scene.name,
        'latch_state': epoch.latch_state,
        'right_door_support_state': epoch.right_door_support_state,
        'left_cube_support_state': epoch.left_cube_support_state,
        'mechanism_names': ['door_hinge_joint', 'handle_latch_joint'],
        'mechanism_positions': [door, handle],
        'expected_handle_rad': expected_handle,
        'handle_tolerance_rad': handle_tolerance,
        'door_tolerance_rad': door_tolerance,
        'seam_state_hash': SEAM_STATE_HASH,
        'world_object_ids': sorted(world_ids),
        'controller_commands_issued': 0,
    }


def _write_new_private_directory(
        output: Path, epoch_bytes: bytes, scene_bytes: bytes,
        metadata: dict) -> None:
    output = output.resolve()
    if not output.is_absolute() or output.exists():
        raise CaptureError('OUTPUT_MUST_BE_ABSOLUTE_AND_ABSENT')
    output.parent.mkdir(parents=True, exist_ok=True)
    output.mkdir(mode=0o700)
    try:
        epoch_path = output / 'epoch.cdr'
        scene_path = output / 'scene.cdr'
        metadata_path = output / 'capture.json'
        epoch_path.write_bytes(epoch_bytes)
        scene_path.write_bytes(scene_bytes)
        metadata = dict(metadata)
        metadata['epoch_cdr_sha256'] = _sha256(epoch_bytes)
        metadata['scene_cdr_sha256'] = _sha256(scene_bytes)
        metadata_path.write_text(
            json.dumps(metadata, sort_keys=True, indent=2, allow_nan=False) + '\n',
            encoding='utf-8')
        for path in (epoch_path, scene_path, metadata_path):
            os.chmod(path, stat.S_IRUSR)
    except Exception:
        # Preserve any bytes already written for diagnosis; never overwrite an
        # existing capture or silently present a partial directory as valid.
        (output / 'INCOMPLETE').touch(mode=0o400, exist_ok=True)
        raise


class MiddleCapture(Node):
    """Subscribe only, pair one epoch/scene, and signal the main thread."""

    def __init__(self, expected_handle: float, handle_tolerance: float,
                 door_tolerance: float):
        """Initialize the read-only epoch and planning-scene subscriptions."""
        super().__init__('h1_candidate3_middle_live_capture')
        self.expected_handle = expected_handle
        self.handle_tolerance = handle_tolerance
        self.door_tolerance = door_tolerance
        self.lock = threading.Lock()
        self.epochs: dict[tuple[int, str], SceneEpoch] = {}
        self.scenes: dict[tuple[int, str], PlanningScene] = {}
        self.result = None
        self.error = None
        self.ready = threading.Event()
        transient = QoSProfile(
            depth=1, reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(
            SceneEpoch, '/dynamic_scene/epoch', self._epoch, transient)
        self.create_subscription(
            PlanningScene, '/dynamic_scene/frozen_scene', self._scene, transient)

    @staticmethod
    def _scene_key(message: PlanningScene):
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

    def _epoch(self, message: SceneEpoch) -> None:
        key = int(message.epoch_id), message.scene_hash
        with self.lock:
            self.epochs = {key: message}
        self._try_pair(key)

    def _scene(self, message: PlanningScene) -> None:
        key = self._scene_key(message)
        if key is None:
            return
        with self.lock:
            self.scenes = {key: message}
        self._try_pair(key)

    def _try_pair(self, key) -> None:
        with self.lock:
            if self.ready.is_set() or key not in self.epochs or key not in self.scenes:
                return
            epoch = self.epochs[key]
            scene = self.scenes[key]
        try:
            metadata = _validate_pair(
                epoch, scene, self.expected_handle, self.handle_tolerance,
                self.door_tolerance)
            publisher_counts = {
                topic: len(self.get_publishers_info_by_topic(topic))
                for topic in CONTROLLER_TOPICS
            }
            active = [topic for topic, count in publisher_counts.items() if count]
            if active:
                raise CaptureError(f'CONTROLLER_PUBLISHERS:{",".join(active)}')
            metadata['controller_publisher_counts'] = publisher_counts
            result = (
                bytes(serialize_message(epoch)),
                bytes(serialize_message(scene)), metadata)
            with self.lock:
                self.result = result
            self.ready.set()
        except Exception as error:  # Keep the boundary fail-closed.
            with self.lock:
                self.error = error


def _arguments(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--timeout-s', type=float, default=30.0)
    parser.add_argument('--expected-handle-rad', type=float,
                        default=math.pi / 2.0)
    parser.add_argument('--handle-tolerance-rad', type=float,
                        default=math.radians(1.0))
    parser.add_argument('--door-tolerance-rad', type=float,
                        default=math.radians(0.2))
    return parser.parse_args(argv)


def main(argv=None) -> int:
    """Capture and seal one verified middle-seam epoch/scene pair."""
    args = _arguments(argv)
    if (not args.output.is_absolute() or args.output.exists() or
            not math.isfinite(args.timeout_s) or args.timeout_s <= 0.0 or
            not math.isfinite(args.expected_handle_rad) or
            not math.isfinite(args.handle_tolerance_rad) or
            args.handle_tolerance_rad <= 0.0 or
            not math.isfinite(args.door_tolerance_rad) or
            args.door_tolerance_rad <= 0.0):
        print('CANDIDATE3_MIDDLE_CAPTURE=ERROR:ARGUMENTS')
        return 2
    # Custom capture arguments have already been consumed by argparse; do not
    # offer them to rclpy as ROS arguments a second time.
    rclpy.init(args=[])
    node = MiddleCapture(
        args.expected_handle_rad, args.handle_tolerance_rad,
        args.door_tolerance_rad)
    deadline = time.monotonic() + args.timeout_s
    try:
        while not node.ready.is_set() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
        if not node.ready.is_set():
            if node.error is not None:
                raise node.error
            raise CaptureError('TIMEOUT')
        if node.error is not None:
            raise node.error
        if node.result is None:
            raise CaptureError('MISSING_RESULT')
        epoch_bytes, scene_bytes, metadata = node.result
        _write_new_private_directory(
            args.output, epoch_bytes, scene_bytes, metadata)
        print(json.dumps({
            'event': 'CANDIDATE3_MIDDLE_CAPTURE_PASS',
            'output': str(args.output),
            'scene_epoch_id': metadata['scene_epoch_id'],
            'scene_hash': metadata['scene_hash'],
            'controller_commands_issued': 0,
        }, sort_keys=True))
        return 0
    except Exception as error:
        print(f'CANDIDATE3_MIDDLE_CAPTURE=ERROR:{error}')
        return 3
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
